#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "config.h"
#include "nav_types.h"
#include "nav_math.h"
#include "ekf.h"
#include "guidance.h"
#include "flight_sm.h"
#include "actuators.h"

// Mock hardware PWM sink to record actual pin writes during host testing
struct MockPwmCapture {
    uint16_t last_left_pin_us;
    uint16_t last_right_pin_us;
    uint32_t left_write_count;
    uint32_t right_write_count;
};

static MockPwmCapture g_mock_pwm = {0, 0, 0, 0};

static void mockPwmWriteCallback(uint8_t pin, uint16_t pulse_us) {
    if (pin == SERVO_LEFT_PIN) {
        g_mock_pwm.last_left_pin_us = pulse_us;
        g_mock_pwm.left_write_count++;
    } else if (pin == SERVO_RIGHT_PIN) {
        g_mock_pwm.last_right_pin_us = pulse_us;
        g_mock_pwm.right_write_count++;
    }
}

static void resetMockPwm() {
    g_mock_pwm.last_left_pin_us  = 0xFFFF;
    g_mock_pwm.last_right_pin_us = 0xFFFF;
    g_mock_pwm.left_write_count  = 0;
    g_mock_pwm.right_write_count = 0;
}

// Synthetic test fixture for host unit verification ONLY.
// Production firmware defaults to ACTUATOR_CALIBRATED = 0 (uncalibrated lockout).
static ContinuousServoSystemCal makeSyntheticTestCal() {
    ContinuousServoSystemCal cal;
    cal.calibrated = true;

    // Left channel: wind_polarity = -1 (pulse < 1500 us winds brake line)
    cal.left.neutral_pulse_us     = 1500;
    cal.left.neutral_deadband_us  = 20;
    cal.left.min_pulse_us         = 1050;
    cal.left.max_pulse_us         = 1950;
    cal.left.max_wind_delta_us    = 350;  // 1150 us at full wind
    cal.left.max_release_delta_us = 300;  // 1800 us at full release
    cal.left.wind_polarity        = -1;
    cal.left.rev_per_sec_at_max   = 1.25f;
    cal.left.mm_per_revolution    = 16.0f; // 20.0 mm/s linear speed at max
    cal.left.max_pull_mm          = 60.0f;

    // Right channel: wind_polarity = +1 (pulse > 1490 us winds brake line)
    cal.right.neutral_pulse_us     = 1490;
    cal.right.neutral_deadband_us  = 20;
    cal.right.min_pulse_us         = 1050;
    cal.right.max_pulse_us         = 1950;
    cal.right.max_wind_delta_us    = 350;  // 1840 us at full wind
    cal.right.max_release_delta_us = 300;  // 1190 us at full release
    cal.right.wind_polarity        = +1;
    cal.right.rev_per_sec_at_max   = 1.25f;
    cal.right.mm_per_revolution    = 16.0f; // 20.0 mm/s linear speed at max
    cal.right.max_pull_mm          = 60.0f;

    cal.position_deadband_mm    = 1.0f;
    cal.drift_uncertainty_ratio = 0.10f; // 10% uncertainty growth per mm moved
    cal.max_uncertainty_mm      = 50.0f; // 500 mm total open-loop travel budget
    cal.max_burst_duration_ms   = 400UL;
    cal.min_cooldown_ms         = 150UL;
    cal.command_timeout_ms      = 300UL;
    return cal;
}

static int g_passed = 0;

// 1. Initialization
static void test_01_initialization() {
    resetMockPwm();
    ActuatorManager mgr;
    mgr.setPwmWriteCallback(mockPwmWriteCallback);
    mgr.init();

    assert(!mgr.isCalibrated());
    assert(!mgr.areServosAttached());
    assert(!mgr.isActualPositionKnown());
    assert(mgr.getFaultCode() == ActuatorFaultCode::UNCALIBRATED);
    assert(mgr.getLeftSpoolDirection() == SpoolDirection::STOP);
    assert(mgr.getRightSpoolDirection() == SpoolDirection::STOP);
    assert(mgr.getLeftServoPulseUs() == 0);
    assert(mgr.getRightServoPulseUs() == 0);
    assert(g_mock_pwm.last_left_pin_us == 0);
    assert(g_mock_pwm.last_right_pin_us == 0);
    assert(!mgr.isBurnwirePinHigh());
    assert(!mgr.isBuzzerPinHigh());
    assert(!mgr.isFlywheelActive());
    assert(mgr.getFlywheelPulseUs() == ESC_MIN_PULSE_US);

    g_passed++;
    printf("[PASS] 01. Initialization sets safe detached outputs and uncalibrated lockout\n");
}

// 2. Uncalibrated lockout
static void test_02_uncalibrated_lockout() {
    resetMockPwm();
    ActuatorManager mgr;
    mgr.setPwmWriteCallback(mockPwmWriteCallback);
    mgr.init();

    mgr.applySteering(-1.0f, true, 100, FlightState::PRECISION_GUIDANCE);
    mgr.update(150);
    assert(mgr.getFaultCode() == ActuatorFaultCode::UNCALIBRATED);
    assert(mgr.getLeftSpoolDirection() == SpoolDirection::STOP);
    assert(mgr.getLeftServoPulseUs() == 0);
    assert(g_mock_pwm.last_left_pin_us == 0);
    assert(isnan(mgr.getLeftEstimatedPullMm()));
    assert(!mgr.isActualPositionKnown());

    g_passed++;
    printf("[PASS] 02. Uncalibrated lockout prevents all physical motor rotation\n");
}

// 3. Invalid calibration
static void test_03_invalid_calibration() {
    ActuatorManager mgr;
    ContinuousServoSystemCal base = makeSyntheticTestCal();

    ContinuousServoSystemCal c1 = base;
    c1.left.neutral_pulse_us = 900; // < min_pulse_us
    assert(!mgr.configureCalibration(c1));
    assert(mgr.getFaultCode() == ActuatorFaultCode::INVALID_CALIBRATION);

    ContinuousServoSystemCal c2 = base;
    c2.right.wind_polarity = 0;
    assert(!mgr.configureCalibration(c2));

    ContinuousServoSystemCal c3 = base;
    c3.left.mm_per_revolution = -2.0f;
    assert(!mgr.configureCalibration(c3));

    ContinuousServoSystemCal c4 = base;
    c4.drift_uncertainty_ratio = -0.1f;
    assert(!mgr.configureCalibration(c4));

    ContinuousServoSystemCal c5 = base;
    c5.max_uncertainty_mm = 0.0f;
    assert(!mgr.configureCalibration(c5));

    g_passed++;
    printf("[PASS] 03. Invalid calibration parameters are rejected with INVALID_CALIBRATION\n");
}

// 4. Steering direction & GNC sign convention
static void test_04_steering_direction() {
    resetMockPwm();
    ActuatorManager mgr;
    mgr.setPwmWriteCallback(mockPwmWriteCallback);
    ContinuousServoSystemCal cal = makeSyntheticTestCal();
    assert(mgr.configureCalibration(cal));

    // Left turn (< 0): winds left only
    mgr.applySteering(-0.75f, true, 1000, FlightState::PRECISION_GUIDANCE);
    assert(fabsf(mgr.getLeftBrake() - 0.75f) < 1e-4f);
    assert(fabsf(mgr.getRightBrake() - 0.0f) < 1e-4f);
    assert(mgr.getLeftSpoolDirection() == SpoolDirection::WIND_BRAKE);
    assert(mgr.getRightSpoolDirection() == SpoolDirection::STOP);
    assert(g_mock_pwm.last_left_pin_us < cal.left.neutral_pulse_us);
    assert(g_mock_pwm.last_right_pin_us == cal.right.neutral_pulse_us);

    // Reset and test Right turn (> 0): winds right only
    mgr.resetEstimatedLinePosition();
    mgr.applySteering(+0.75f, true, 2000, FlightState::PRECISION_GUIDANCE);
    assert(fabsf(mgr.getLeftBrake() - 0.0f) < 1e-4f);
    assert(fabsf(mgr.getRightBrake() - 0.75f) < 1e-4f);
    assert(mgr.getLeftSpoolDirection() == SpoolDirection::STOP);
    assert(mgr.getRightSpoolDirection() == SpoolDirection::WIND_BRAKE);
    assert(g_mock_pwm.last_left_pin_us == cal.left.neutral_pulse_us);
    assert(g_mock_pwm.last_right_pin_us > cal.right.neutral_pulse_us);

    g_passed++;
    printf("[PASS] 04. Steering sign convention maps left/right brake winding accurately\n");
}

// 5. Neutral stop
static void test_05_neutral_stop() {
    resetMockPwm();
    ActuatorManager mgr;
    mgr.setPwmWriteCallback(mockPwmWriteCallback);
    ContinuousServoSystemCal cal = makeSyntheticTestCal();
    assert(mgr.configureCalibration(cal));

    mgr.applySteering(0.0f, true, 1000, FlightState::PRECISION_GUIDANCE);
    assert(mgr.getLeftSpoolDirection() == SpoolDirection::STOP);
    assert(mgr.getRightSpoolDirection() == SpoolDirection::STOP);
    assert(g_mock_pwm.last_left_pin_us == cal.left.neutral_pulse_us);
    assert(g_mock_pwm.last_right_pin_us == cal.right.neutral_pulse_us);

    g_passed++;
    printf("[PASS] 05. Neutral steering outputs calibrated neutral stop pulse on both channels\n");
}

// 6. Sudden steering reversal
static void test_06_sudden_steering_reversal() {
    ActuatorManager mgr;
    ContinuousServoSystemCal cal = makeSyntheticTestCal();
    assert(mgr.configureCalibration(cal));

    // Wind left for 200 ms (-0.8 -> 4 mm pull)
    mgr.applySteering(-0.8f, true, 1000, FlightState::PRECISION_GUIDANCE);
    mgr.applySteering(-0.8f, true, 1100, FlightState::PRECISION_GUIDANCE);
    mgr.update(1200);
    assert(mgr.getLeftEstimatedPullMm() > 3.5f);

    // Suddenly reverse to full right (+0.8) at t = 1200 ms:
    // Left channel must immediately switch to RELEASE_BRAKE (unwinding its 4 mm pull),
    // while Right channel switches to WIND_BRAKE!
    mgr.applySteering(+0.8f, true, 1200, FlightState::PRECISION_GUIDANCE);
    assert(mgr.getLeftSpoolDirection() == SpoolDirection::RELEASE_BRAKE);
    assert(mgr.getLeftServoPulseUs() > cal.left.neutral_pulse_us); // Release polarity for left (+1)
    assert(mgr.getRightSpoolDirection() == SpoolDirection::WIND_BRAKE);
    assert(mgr.getRightServoPulseUs() > cal.right.neutral_pulse_us); // Wind polarity for right (+1)

    g_passed++;
    printf("[PASS] 06. Sudden steering reversal unwinds active side while winding opposite side\n");
}

// 7 & 8. Maximum burst & Cooldown enforcement
static void test_07_08_max_burst_and_cooldown() {
    ActuatorManager mgr;
    ContinuousServoSystemCal cal = makeSyntheticTestCal();
    cal.max_burst_duration_ms = 400UL;
    cal.min_cooldown_ms       = 150UL;
    assert(mgr.configureCalibration(cal));

    uint32_t t = 1000;
    mgr.applySteering(-1.0f, true, t, FlightState::PRECISION_GUIDANCE);
    for (int i = 0; i < 4; i++) {
        t += 100;
        mgr.update(t);
        if (i < 3) mgr.applySteering(-1.0f, true, t, FlightState::PRECISION_GUIDANCE);
    }

    assert(mgr.getFaultCode() == ActuatorFaultCode::MAX_BURST_EXCEEDED);
    assert(mgr.getLeftSpoolDirection() == SpoolDirection::STOP);
    assert(mgr.getLeftServoPulseUs() == cal.left.neutral_pulse_us);
    assert(mgr.getLeftChannelState().burst_limit_latched);

    // Cooldown enforced at +100 ms (< 150 ms)
    t += 100;
    mgr.applySteering(-1.0f, true, t, FlightState::PRECISION_GUIDANCE);
    assert(mgr.getLeftSpoolDirection() == SpoolDirection::STOP);

    // Cooldown expires at +50 ms (total 150 ms) -> winding resumes
    t += 50;
    mgr.applySteering(-1.0f, true, t, FlightState::PRECISION_GUIDANCE);
    assert(mgr.getLeftSpoolDirection() == SpoolDirection::WIND_BRAKE);

    g_passed += 2;
    printf("[PASS] 07 & 08. Maximum burst (400 ms) and mandatory cooldown (150 ms) enforced\n");
}

// 9. Stale commands
static void test_09_stale_command_watchdog() {
    ActuatorManager mgr;
    ContinuousServoSystemCal cal = makeSyntheticTestCal();
    cal.command_timeout_ms = 300UL;
    assert(mgr.configureCalibration(cal));

    mgr.applySteering(+1.0f, true, 1000, FlightState::PRECISION_GUIDANCE);
    assert(mgr.getRightSpoolDirection() == SpoolDirection::WIND_BRAKE);

    mgr.update(1305); // 305 ms > 300 ms timeout
    assert(mgr.getFaultCode() == ActuatorFaultCode::STALE_COMMAND);
    assert(mgr.getRightSpoolDirection() == SpoolDirection::STOP);
    assert(mgr.getRightServoPulseUs() == cal.right.neutral_pulse_us);

    g_passed++;
    printf("[PASS] 09. Stale guidance command (> 300 ms) stops servos at neutral\n");
}

// 10. Stalled update loop (Hardware Timer ISR Cutoff)
static void test_10_stalled_main_loop_hardware_isr_cutoff() {
    resetMockPwm();
    ActuatorManager mgr;
    mgr.setPwmWriteCallback(mockPwmWriteCallback);
    ContinuousServoSystemCal cal = makeSyntheticTestCal();
    cal.command_timeout_ms = 300UL;
    assert(mgr.configureCalibration(cal));

    // Main loop applies steering at t = 1000 ms, then main loop stalls (update() never called!)
    mgr.applySteering(-1.0f, true, 1000, FlightState::PRECISION_GUIDANCE);
    assert(mgr.getLeftSpoolDirection() == SpoolDirection::WIND_BRAKE);
    assert(g_mock_pwm.last_left_pin_us < cal.left.neutral_pulse_us);

    // Hardware timer ISR fires at t = 1100 ms (planned 100 ms step complete while loop() is stalled)
    mgr.onHardwareTimerTick(1100);
    assert(mgr.getLeftSpoolDirection() == SpoolDirection::STOP);
    assert(g_mock_pwm.last_left_pin_us == cal.left.neutral_pulse_us);

    // Hardware timer ISR fires at t = 1310 ms (> 300 ms command timeout while loop() is still stalled)
    mgr.onHardwareTimerTick(1310);
    assert(mgr.getFaultCode() == ActuatorFaultCode::LOOP_STALL_ISR_CUTOFF);
    assert(g_mock_pwm.last_left_pin_us == cal.left.neutral_pulse_us);
    assert(g_mock_pwm.last_right_pin_us == cal.right.neutral_pulse_us);

    g_passed++;
    printf("[PASS] 10. Hardware timer ISR stops servos in silicon when main control loop stalls\n");
}

// 11. Persistent steering requests & unverified neutral on release (Section 5 scenario)
static void test_11_persistent_steering_and_unverified_neutral_release() {
    ActuatorManager mgr;
    ContinuousServoSystemCal cal = makeSyntheticTestCal();
    cal.max_burst_duration_ms = 1000UL;
    assert(mgr.configureCalibration(cal));

    // Initially before any motion, ground reset marks neutral as verified
    assert(mgr.isLeftNeutralVerified());

    // Steps 1-5: Guidance requests persistent left turn (-0.5 -> 30 mm target) for 4.0 seconds (40 ticks)
    uint32_t t = 1000;
    for (int i = 0; i < 40; i++) {
        mgr.applySteering(-0.5f, true, t, FlightState::PRECISION_GUIDANCE);
        t += 100;
        mgr.update(t);
    }

    // Actuator MUST stop at ~30 mm and NOT keep winding indefinitely!
    assert(mgr.getLeftSpoolDirection() == SpoolDirection::STOP);
    assert(mgr.getLeftServoPulseUs() == cal.left.neutral_pulse_us);
    assert(fabsf(mgr.getLeftEstimatedPullMm() - 30.0f) <= cal.position_deadband_mm);
    assert(!mgr.isLeftNeutralVerified());
    assert(!mgr.isActualPositionKnown());

    // Steps 6-8: Guidance requests neutral steering (0.0)
    for (int i = 0; i < 40; i++) {
        mgr.applySteering(0.0f, true, t, FlightState::PRECISION_GUIDANCE);
        t += 100;
        mgr.update(t);
    }

    // Estimated pull returns to ~0.0 mm and motor stops, but controller MUST NOT
    // claim that the physical line is verified at neutral!
    assert(mgr.getLeftSpoolDirection() == SpoolDirection::STOP);
    assert(mgr.getLeftEstimatedPullMm() <= cal.position_deadband_mm);
    assert(!mgr.isLeftNeutralVerified());
    assert(!mgr.isActualPositionKnown());
    assert(mgr.getLeftEstimatedUncertaintyMm() > 0.0f);

    g_passed++;
    printf("[PASS] 11. Persistent turn stops at target pull; release does not claim verified physical neutral\n");
}

// 12 & 13. Repeated alternating turns (drift limit) & travel saturation
static void test_12_13_cumulative_drift_limit_and_travel_saturation() {
    ActuatorManager mgr;
    ContinuousServoSystemCal cal = makeSyntheticTestCal();
    cal.max_burst_duration_ms   = 2000UL;
    cal.drift_uncertainty_ratio = 0.25f; // 25% uncertainty per mm
    cal.max_uncertainty_mm      = 25.0f; // Exhausted after 100 mm cumulative travel
    assert(mgr.configureCalibration(cal));

    uint32_t t = 1000;
    // Wind full left (60 mm -> saturates at max_pull_mm = 60.0 mm)
    for (int i = 0; i < 35; i++) {
        mgr.applySteering(-1.0f, true, t, FlightState::PRECISION_GUIDANCE);
        t += 100;
        mgr.update(t);
    }
    assert(fabsf(mgr.getLeftEstimatedPullMm() - 60.0f) < 1e-3f);

    // Unwind back toward 0 (after 40 mm of release, total travel = 100 mm -> uncertainty = 25 mm -> OPEN_LOOP_DRIFT_LIMIT!)
    for (int i = 0; i < 30; i++) {
        mgr.applySteering(0.0f, true, t, FlightState::PRECISION_GUIDANCE);
        t += 100;
        mgr.update(t);
    }
    assert(mgr.getFaultCode() == ActuatorFaultCode::OPEN_LOOP_DRIFT_LIMIT);
    assert(mgr.getLeftChannelState().drift_limit_latched);
    assert(mgr.getLeftSpoolDirection() == SpoolDirection::STOP);
    assert(mgr.getLeftServoPulseUs() == cal.left.neutral_pulse_us);

    g_passed += 2;
    printf("[PASS] 12 & 13. Travel saturation at max_pull_mm and cumulative open-loop drift limit enforced\n");
}

// 14, 15, 16. Failsafe, Landing, and Prohibited flight states
static void test_14_15_16_flight_states_landing_failsafe() {
    ActuatorManager mgr;
    ContinuousServoSystemCal cal = makeSyntheticTestCal();
    assert(mgr.configureCalibration(cal));

    FlightState prohibited[] = {
        FlightState::BOOT,
        FlightState::LAUNCH_PAD,
        FlightState::ASCENT,
        FlightState::DROGUE_DESCENT,
        FlightState::PARAFOIL_DEPLOYMENT
    };
    for (size_t i = 0; i < sizeof(prohibited) / sizeof(prohibited[0]); i++) {
        mgr.applySteering(-1.0f, true, 1000 + (uint32_t)i * 100, prohibited[i]);
        assert(mgr.getFaultCode() == ActuatorFaultCode::PROHIBITED_FLIGHT_STATE);
        assert(mgr.getLeftSpoolDirection() == SpoolDirection::STOP);
        assert(mgr.getLeftServoPulseUs() == 0);
        assert(!mgr.areServosAttached());
    }

    // Active in FINAL_APPROACH -> LANDING
    mgr.applySteering(+1.0f, true, 3000, FlightState::FINAL_APPROACH);
    assert(mgr.getRightSpoolDirection() == SpoolDirection::WIND_BRAKE);
    mgr.applySteering(+1.0f, false, 3100, FlightState::LANDING);
    assert(mgr.getRightSpoolDirection() == SpoolDirection::STOP);
    assert(mgr.getRightServoPulseUs() == 0);
    assert(!mgr.areServosAttached());

    // Active in FINAL_APPROACH -> FAILSAFE
    mgr.applySteering(-1.0f, true, 3200, FlightState::FINAL_APPROACH);
    assert(mgr.getLeftSpoolDirection() == SpoolDirection::WIND_BRAKE);
    mgr.applySteering(-1.0f, true, 3300, FlightState::FAILSAFE);
    assert(mgr.getFaultCode() == ActuatorFaultCode::PROHIBITED_FLIGHT_STATE);
    assert(mgr.getLeftSpoolDirection() == SpoolDirection::STOP);
    assert(mgr.getLeftServoPulseUs() == 0);
    assert(!mgr.areServosAttached());

    g_passed += 3;
    printf("[PASS] 14, 15 & 16. Prohibited flight states, LANDING, and FAILSAFE immediately stop and detach\n");
}

// 17. Non-finite inputs & clamping
static void test_17_non_finite_and_clamping() {
    ActuatorManager mgr;
    ContinuousServoSystemCal cal = makeSyntheticTestCal();
    assert(mgr.configureCalibration(cal));

    mgr.applySteering(-99.0f, true, 1000, FlightState::PRECISION_GUIDANCE);
    assert(fabsf(mgr.getRequestedSteeringEffort() - (-1.0f)) < 1e-4f);
    assert(mgr.getLeftServoPulseUs() >= cal.left.min_pulse_us);

    mgr.applySteering(NAN, true, 1050, FlightState::PRECISION_GUIDANCE);
    assert(mgr.getFaultCode() == ActuatorFaultCode::NON_FINITE_INPUT);
    assert(mgr.getLeftSpoolDirection() == SpoolDirection::STOP);
    assert(mgr.getLeftServoPulseUs() == cal.left.neutral_pulse_us);

    mgr.applySteering(INFINITY, true, 1100, FlightState::PRECISION_GUIDANCE);
    assert(mgr.getFaultCode() == ActuatorFaultCode::NON_FINITE_INPUT);
    assert(mgr.getRightSpoolDirection() == SpoolDirection::STOP);

    g_passed++;
    printf("[PASS] 17. NaN/Inf inputs rejected with safe stop; out-of-range finite inputs clamped\n");
}

// 18. Time wraparound across uint32_t millis() rollover (0xFFFFFFFF -> 0x00000000)
static void test_18_millis_wraparound() {
    ActuatorManager mgr;
    ContinuousServoSystemCal cal = makeSyntheticTestCal();
    assert(mgr.configureCalibration(cal));

    uint32_t t0 = 0xFFFFFF9CUL; // 100 ms before 32-bit rollover (4294967196)
    mgr.applySteering(-1.0f, true, t0, FlightState::PRECISION_GUIDANCE);
    assert(mgr.getLeftSpoolDirection() == SpoolDirection::WIND_BRAKE);

    // Step +100 ms across 0xFFFFFFFF to t1 = 0x00000000
    uint32_t t1 = t0 + 100UL; // 0x00000000
    mgr.update(t1);
    assert(fabsf(mgr.getLeftEstimatedPullMm() - 2.0f) < 0.05f); // 20 mm/s * 0.1s = 2.0 mm!

    // Step another +100 ms after rollover to t2 = 100 (total 200 ms -> 4.0 mm)
    uint32_t t2 = t1 + 100UL;
    mgr.applySteering(-1.0f, true, t2, FlightState::PRECISION_GUIDANCE);
    assert(fabsf(mgr.getLeftEstimatedPullMm() - 4.0f) < 0.05f);
    mgr.update(t2 + 100UL); // total 300 ms -> 6.0 mm
    assert(fabsf(mgr.getLeftEstimatedPullMm() - 6.0f) < 0.05f);

    // Verify watchdog expiration across rollover
    uint32_t t_pre_wrap = 0xFFFFFF50UL;
    mgr.applySteering(-1.0f, true, t_pre_wrap, FlightState::PRECISION_GUIDANCE);
    mgr.update(t_pre_wrap + 350UL); // Wraps past 0 and exceeds 300 ms timeout
    assert(mgr.getFaultCode() == ActuatorFaultCode::STALE_COMMAND);
    assert(mgr.getLeftSpoolDirection() == SpoolDirection::STOP);

    g_passed++;
    printf("[PASS] 18. 32-bit millis() rollover (0xFFFFFFFF -> 0x00000000) handled without glitch\n");
}

// 19. Telemetry serialization & Full EKF + Guidance integration
static void test_19_telemetry_serialization_and_ekf_integration() {
    ActuatorManager mgr;
    mgr.init();

    char kv_uncal[256];
    mgr.formatTelemetryKeyValue(kv_uncal, sizeof(kv_uncal));
    assert(strstr(kv_uncal, "LEFT_BRAKE_CMD=0.00,RIGHT_BRAKE_CMD=0.00") != nullptr);
    assert(strstr(kv_uncal, "LEFT_SPOOL_DIR=STOP,RIGHT_SPOOL_DIR=STOP") != nullptr);
    assert(strstr(kv_uncal, "LEFT_SERVO_US=0,RIGHT_SERVO_US=0") != nullptr);
    assert(strstr(kv_uncal, "LEFT_EST_PULL_MM=UNCAL,RIGHT_EST_PULL_MM=UNCAL") != nullptr);
    assert(strstr(kv_uncal, "ACTUATOR_CAL=UNCAL_LOCKED,ACTUATOR_FAULT=UNCALIBRATED") != nullptr);
    assert(strstr(kv_uncal, "LEFT_SERVO_DEG") == nullptr); // No fake degrees!

    // Now test with EKF + GuidanceController + calibrated actuator
    ExtendedKalmanFilter ekf;
    ekf.reset(0.0f, 0.0f, 500.0f, 45.0f);
    ekf.predict(0.1f, 0.0f);
    ekf.correctBaro(499.75f);
    ekf.correctHeading(45.0f);
    ekf.correctGPS(0.5f, 0.5f, 5.0f, 5.0f, true);
    EKFStateEstimate est = ekf.getEstimate();

    WaypointManager wm;
    wm.init(LocalPoint(0.0f, 0.0f), LocalPoint(1200.0f, 0.0f), NUM_WAYPOINTS, WAYPOINT_ACCEPTANCE_RAD);
    PIDController pid(PID_KP, PID_KI, PID_KD, -1.0f, 1.0f, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
    GuidanceController guidance(&wm, &pid);
    GuidanceOutput g_out = guidance.update(LocalPoint(est.x, est.y), est.heading_deg, est.altitude, 0.1f);
    assert(g_out.steering > 0.0f);

    ContinuousServoSystemCal cal = makeSyntheticTestCal();
    assert(mgr.configureCalibration(cal));
    mgr.applySteering(g_out.steering, true, 1000, FlightState::PRECISION_GUIDANCE);
    mgr.update(1100);

    char kv_cal[256];
    mgr.formatTelemetryKeyValue(kv_cal, sizeof(kv_cal));
    assert(strstr(kv_cal, "RIGHT_SPOOL_DIR=WIND") != nullptr);
    assert(strstr(kv_cal, "ACTUATOR_CAL=CALIBRATED,ACTUATOR_FAULT=NONE") != nullptr);

    g_passed++;
    printf("[PASS] 19. v2 Telemetry serialization & full EKF + Guidance + Actuator pipeline verified\n");
}

int main() {
    printf("=== CAN-7USAT Integrated Flight Software Actuator Test Suite ===\n");
    test_01_initialization();
    test_02_uncalibrated_lockout();
    test_03_invalid_calibration();
    test_04_steering_direction();
    test_05_neutral_stop();
    test_06_sudden_steering_reversal();
    test_07_08_max_burst_and_cooldown();
    test_09_stale_command_watchdog();
    test_10_stalled_main_loop_hardware_isr_cutoff();
    test_11_persistent_steering_and_unverified_neutral_release();
    test_12_13_cumulative_drift_limit_and_travel_saturation();
    test_14_15_16_flight_states_landing_failsafe();
    test_17_non_finite_and_clamping();
    test_18_millis_wraparound();
    test_19_telemetry_serialization_and_ekf_integration();
    printf("================================================================\n");
    printf("ALL %d / 19 TEST CATEGORIES PASSED.\n", g_passed);
    return 0;
}
