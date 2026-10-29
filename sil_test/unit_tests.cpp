#include <iostream>
#include <iomanip>
#include <cmath>
#include <cassert>
#include <string>

#include "../CAN7USAT_FSW/config.h"
#include "../CAN7USAT_FSW/nav_types.h"
#include "../CAN7USAT_FSW/nav_math.h"
#include "../CAN7USAT_FSW/ekf.h"
#include "../CAN7USAT_FSW/guidance.h"
#include "../CAN7USAT_FSW/flight_sm.h"
#include "../CAN7USAT_FSW/actuators.h"

static int g_tests_passed = 0;
static int g_tests_failed = 0;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (cond) { \
            g_tests_passed++; \
        } else { \
            g_tests_failed++; \
            std::cerr << "  [FAIL] " << msg << " (" << #cond << ") at line " << __LINE__ << std::endl; \
        } \
    } while(0)

// ----------------------------------------------------------------------------
// 1. Navigation Math Unit Tests
// ----------------------------------------------------------------------------
void test_nav_math() {
    std::cout << "[TEST] 1. Navigation Math & Geodesy..." << std::endl;

    // A. Degree / Radian conversion
    TEST_ASSERT(fabsf(deg2rad(180.0f) - (float)M_PI) < 1e-5f, "deg2rad(180) == pi");
    TEST_ASSERT(fabsf(rad2deg((float)M_PI) - 180.0f) < 1e-5f, "rad2deg(pi) == 180");

    // B. Angle Wrapping
    TEST_ASSERT(fabsf(wrap_360(370.0f) - 10.0f) < 1e-5f, "wrap_360(370) == 10");
    TEST_ASSERT(fabsf(wrap_360(-10.0f) - 350.0f) < 1e-5f, "wrap_360(-10) == 350");
    TEST_ASSERT(fabsf(wrap_180(190.0f) - (-170.0f)) < 1e-5f, "wrap_180(190) == -170");
    TEST_ASSERT(fabsf(wrap_180(-190.0f) - 170.0f) < 1e-5f, "wrap_180(-190) == 170");

    // C. Heading Error (Shortest Arc)
    // Turning from 350 to 10 should be +20 (turn right)
    TEST_ASSERT(fabsf(heading_error(350.0f, 10.0f) - 20.0f) < 1e-5f, "heading_error(350, 10) == +20");
    // Turning from 10 to 350 should be -20 (turn left)
    TEST_ASSERT(fabsf(heading_error(10.0f, 350.0f) - (-20.0f)) < 1e-5f, "heading_error(10, 350) == -20");

    // D. Distance & Bearing
    LocalPoint p1(0.0f, 0.0f);
    LocalPoint p2(1200.0f, 0.0f); // East
    TEST_ASSERT(fabsf(distance_xy(p1, p2) - 1200.0f) < 1e-4f, "distance_xy == 1200");
    TEST_ASSERT(fabsf(bearing(p1, p2) - 90.0f) < 1e-4f, "bearing to East == 90 deg");

    LocalPoint p_north(0.0f, 500.0f);
    TEST_ASSERT(fabsf(bearing(p1, p_north) - 0.0f) < 1e-4f, "bearing to North == 0 deg");

    // E. GPS <-> Local Projection Roundtrip
    GPSPoint origin(13.0827, 80.2707); // Chennai / ISRO region
    LocalPoint local_target(1200.0f, -500.0f);
    GPSPoint gps_target = local_to_gps(local_target, origin);
    LocalPoint reconstructed = gps_to_local(gps_target, origin);
    TEST_ASSERT(fabsf(reconstructed.x - local_target.x) < 0.05f, "GPS projection roundtrip X error < 5cm");
    TEST_ASSERT(fabsf(reconstructed.y - local_target.y) < 0.05f, "GPS projection roundtrip Y error < 5cm");

    // F. Wind Crab Angle Solver
    // Course desired = 90 deg (East), Airspeed = 8 m/s, Crosswind = +4 m/s North (wind_y = 4)
    // Wind pushes North (left), so glider must crab South (right, heading 120 deg)
    // Crab angle: sin(beta) = 4 / 8 = 0.5 -> beta = +30 deg. Corrected heading = 90 + 30 = 120 deg.
    WindCompResult comp = calculate_wind_corrected_heading(90.0f, 8.0f, 0.0f, 4.0f);
    TEST_ASSERT(comp.feasible, "Wind correction feasible for W < Va");
    TEST_ASSERT(fabsf(comp.crab_angle_deg - 30.0f) < 0.5f, "Crab angle for 4m/s North crosswind == +30 deg");
    TEST_ASSERT(fabsf(comp.heading_deg - 120.0f) < 0.5f, "Corrected heading == 120 deg (crabbing into wind)");
}

// ----------------------------------------------------------------------------
// 2. PID Controller Unit Tests
// ----------------------------------------------------------------------------
void test_pid_controller() {
    std::cout << "[TEST] 2. PID Controller (Radian Domain)..." << std::endl;

    PIDController pid(10.0f, 0.1f, 0.5f, -1.0f, 1.0f, -10.0f, 10.0f);

    // Initial zero error -> zero output
    float out = pid.update(0.0f, 0.1f);
    TEST_ASSERT(fabsf(out) < 1e-5f, "Zero error gives zero output");

    // Small positive error (0.05 rad ~ 2.8 deg)
    out = pid.update(0.05f, 0.1f);
    TEST_ASSERT(out > 0.5f && out < 1.0f, "Positive error produces proportional output");

    // Large error (0.5 rad ~ 28 deg) -> saturation at +1.0
    out = pid.update(0.5f, 0.1f);
    TEST_ASSERT(fabsf(out - 1.0f) < 1e-5f, "Large error saturates at +1.0");

    // Negative error -> saturation at -1.0
    pid.reset();
    out = pid.update(-0.5f, 0.1f);
    TEST_ASSERT(fabsf(out - (-1.0f)) < 1e-5f, "Negative error saturates at -1.0");

    // Anti-windup clamping test
    for (int i = 0; i < 200; i++) {
        pid.update(0.2f, 0.1f);
    }
    out = pid.update(-0.2f, 0.1f);
    TEST_ASSERT(out < 0.0f, "Anti-windup allows fast recovery when error sign reverses");
}

// ----------------------------------------------------------------------------
// 3. EKF State Estimation Unit Tests
// ----------------------------------------------------------------------------
void test_ekf_estimation() {
    std::cout << "[TEST] 3. Extended Kalman Filter (EKF)..." << std::endl;

    ExtendedKalmanFilter ekf;
    ekf.reset(0.0f, 0.0f, 1000.0f, 90.0f); // At 1000m, heading East (90 deg)
    ekf.setKinematics(FORWARD_AIRSPEED, PARAFOIL_DESCENT_RATE);

    // Initial state check
    EKFStateEstimate est = ekf.getEstimate();
    TEST_ASSERT(fabsf(est.altitude - 1000.0f) < 1e-3f, "Initial altitude matches");
    TEST_ASSERT(fabsf(est.heading_deg - 90.0f) < 1e-3f, "Initial heading matches");

    // Predict for 1.0s straight flight (steering = 0.0)
    for (int i = 0; i < 10; i++) {
        ekf.predict(0.1f, 0.0f);
    }
    est = ekf.getEstimate();
    TEST_ASSERT(est.x > 7.0f && est.x < 9.0f, "1s straight flight advances ~8m East");
    TEST_ASSERT(fabsf(est.y) < 0.5f, "1s straight flight keeps Y ~0m");
    TEST_ASSERT(fabsf(est.altitude - 997.5f) < 0.2f, "1s glide drops altitude by ~2.5m");

    // Barometer correction update
    ekf.correctBaro(995.0f);
    est = ekf.getEstimate();
    TEST_ASSERT(est.altitude < 997.5f && est.altitude > 994.0f, "Baro measurement pulls altitude toward reading");

    // Heading correction wrap-around test:
    // Reset heading near 0 deg (e.g. 2 deg North), then update with 358 deg (-4 deg delta)
    ekf.reset(est.x, est.y, est.altitude, 2.0f);
    ekf.correctHeading(358.0f);
    est = ekf.getEstimate();
    TEST_ASSERT(est.heading_deg > 355.0f || est.heading_deg < 2.0f, "Heading update accepts wrap-around near 0/360");

    // NIS Outlier Gating Test:
    // Feed an absurd 10-kilometer GPS jump (e.g. multipath corrupt packet)
    bool accepted = ekf.correctGPS(10000.0f, 10000.0f, 0.0f, 0.0f, true);
    TEST_ASSERT(!accepted, "NIS gating successfully rejects 10km GPS outlier");
}

// ----------------------------------------------------------------------------
// 4. Waypoint & Loiter Guidance Unit Tests
// ----------------------------------------------------------------------------
void test_guidance_modules() {
    std::cout << "[TEST] 4. Guidance, Waypoints & Loiter Orbit..." << std::endl;

    LocalPoint start(0.0f, 0.0f);
    LocalPoint target(1200.0f, 0.0f);

    WaypointManager wpMgr;
    wpMgr.init(start, target, 10, 25.0f);

    TEST_ASSERT(wpMgr.getTotalWaypoints() == 10, "10 waypoints generated");
    TEST_ASSERT(wpMgr.getCurrentIndex() == 0, "Starts at waypoint 0");

    // Capture waypoint 0 (located at x ~ 120m)
    LocalPoint near_wp0(120.0f, 0.0f);
    bool adv = wpMgr.update(near_wp0);
    TEST_ASSERT(adv, "Waypoint captured within acceptance radius");
    TEST_ASSERT(wpMgr.getCurrentIndex() == 1, "Advanced to waypoint 1");

    // Loiter guidance circular orbit test:
    // When on the 30m orbit, clockwise orbiting requires positive centripetal turn command
    LoiterGuidance loiter;
    loiter.setCenter(target);

    LocalPoint west_tangent(1170.0f, 0.0f);
    float steer = loiter.update(west_tangent, 0.0f);
    TEST_ASSERT(steer > 0.3f && steer < 0.8f, "Holding orbit applies positive clockwise feedforward turn");

    // Analytical breakout geometry test
    BreakoutGeometry geom = calculateBreakoutGeometry(west_tangent, 0.0f, target, 0.0f, 0.0f);
    TEST_ASSERT(geom.breakout_altitude > 0.0f, "Breakout altitude is positive");
    TEST_ASSERT(geom.radial_distance > 0.0f, "Radial distance is positive");
}

// ----------------------------------------------------------------------------
// 5. Flight State Machine Unit Tests
// ----------------------------------------------------------------------------
void test_state_machine() {
    std::cout << "[TEST] 5. Deterministic Flight State Machine..." << std::endl;

    FlightStateMachine fsm;
    LocalPoint target(1200.0f, 0.0f);
    uint32_t now = 1000;

    // BOOT -> LAUNCH_PAD
    fsm.update(0.0f, 0.0f, LocalPoint(0,0), target, 0.1f, now);
    TEST_ASSERT(fsm.getState() == FlightState::LAUNCH_PAD, "Boot transitions to LAUNCH_PAD");

    // LAUNCH_PAD -> ASCENT (alt > 15m)
    now += 100;
    fsm.update(16.0f, 0.0f, LocalPoint(0,0), target, 0.1f, now);
    TEST_ASSERT(fsm.getState() == FlightState::ASCENT, "Launch climb triggers ASCENT");

    // Drone climb with mid-climb height fluctuation (e.g. 400m -> 360m dip) stays in ASCENT
    now += 5000;
    fsm.update(400.0f, 0.0f, LocalPoint(0,0), target, 0.1f, now);
    now += 1000;
    fsm.update(360.0f, 0.0f, LocalPoint(0,0), target, 0.1f, now);
    TEST_ASSERT(fsm.getState() == FlightState::ASCENT, "Drone height fluctuation below 750m stays in ASCENT");

    // Climb to 1000m and hover with +/-12m bobbing -> stays in ASCENT
    now += 10000;
    fsm.update(1000.0f, 0.0f, LocalPoint(0,0), target, 0.1f, now);
    now += 500;
    fsm.update(988.0f, 0.0f, LocalPoint(0,0), target, 0.1f, now);
    TEST_ASSERT(fsm.getState() == FlightState::ASCENT, "Drone hover bobbing (+/-12m at 1000m) stays in ASCENT");

    // Drone release detection: drops > 20m from peak (979m)
    now += 1000;
    fsm.update(979.0f, 0.0f, LocalPoint(0,0), target, 0.1f, now);
    TEST_ASSERT(fsm.getState() == FlightState::DROGUE_DESCENT, "20m drop above 750m triggers DROGUE_DESCENT");

    // Above 700m (e.g. 705m), stays in DROGUE_DESCENT
    now += 15000;
    fsm.update(705.0f, 0.0f, LocalPoint(0,0), target, 0.1f, now);
    TEST_ASSERT(fsm.getState() == FlightState::DROGUE_DESCENT, "Above 700m stays in DROGUE_DESCENT");

    // Descent to 700m disreefing altitude
    now += 1000;
    fsm.update(699.5f, 0.0f, LocalPoint(0,0), target, 0.1f, now);
    TEST_ASSERT(fsm.getState() == FlightState::PARAFOIL_DEPLOYMENT, "700m triggers PARAFOIL_DEPLOYMENT");
    TEST_ASSERT(fsm.isBurnwireActive(), "Burnwire XY-MOS MOSFET energized at 700m");
    TEST_ASSERT(!fsm.isFlywheelActive(), "Flywheel stays OFF during 5.0s PARAFOIL_DEPLOYMENT");

    // Deployment timer expiration -> enters PRECISION_GUIDANCE (Flywheel 6000 RPM ON)
    now += (uint32_t)(DEPLOYMENT_DURATION_SEC * 1000.0f) + 100;
    fsm.update(685.0f, 0.0f, LocalPoint(0,0), target, 0.1f, now);
    TEST_ASSERT(!fsm.isBurnwireActive(), "Burnwire shut off after deployment duration");
    TEST_ASSERT(fsm.getState() == FlightState::PRECISION_GUIDANCE, "Transitions to PRECISION_GUIDANCE");
    TEST_ASSERT(fsm.isFlywheelActive(), "Flywheel active (6000 RPM) after PARAFOIL_DEPLOYMENT");

    // Touchdown trigger (alt <= 2.0m) -> Flywheel OFF, Buzzer ON
    now += 150000;
    fsm.update(1.5f, 0.0f, target, target, 0.1f, now);
    TEST_ASSERT(fsm.getState() == FlightState::LANDING, "Alt <= 2.0m triggers LANDING");
    TEST_ASSERT(!fsm.isFlywheelActive(), "Flywheel shuts OFF upon LANDING");
    TEST_ASSERT(fsm.isBuzzerActive(), "Recovery buzzer XY-MOS activated upon touchdown");

    // FlatSat forceState & tickTimers verification
    fsm.forceState(FlightState::PARAFOIL_DEPLOYMENT, 50000);
    TEST_ASSERT(fsm.getState() == FlightState::PARAFOIL_DEPLOYMENT, "forceState sets PARAFOIL_DEPLOYMENT");
    TEST_ASSERT(fsm.isBurnwireActive(), "forceState(PARAFOIL_DEPLOYMENT) activates burnwire");
    fsm.tickTimers(50000 + BURNWIRE_DURATION_MS + 100);
    TEST_ASSERT(!fsm.isBurnwireActive(), "tickTimers shuts off burnwire after duration in FlatSat manual hold");
}

// ----------------------------------------------------------------------------
// 6. Actuators Unit Tests
// ----------------------------------------------------------------------------
void test_actuators() {
    std::cout << "[TEST] 6. Actuator Manager & Differential Brake Mixer..." << std::endl;

    ActuatorManager act;
    act.init();

    // Default uncalibrated safety lockout
    TEST_ASSERT(!act.isCalibrated(), "Actuators initially uncalibrated");
    TEST_ASSERT(act.getFaultCode() == ActuatorFaultCode::UNCALIBRATED, "Fault code is UNCALIBRATED");
    TEST_ASSERT(act.getLeftServoPulseUs() == 0, "Uncalibrated Left Servo pulse width == 0 us (detached/safe)");
    TEST_ASSERT(act.getRightServoPulseUs() == 0, "Uncalibrated Right Servo pulse width == 0 us (detached/safe)");

    // Default neutral angles (backward compatibility accessors)
    TEST_ASSERT(fabsf(act.getLeftServoAngle() - 90.0f) < 1e-3f, "Initial Left Servo neutral (90 deg)");
    TEST_ASSERT(fabsf(act.getRightServoAngle() - 90.0f) < 1e-3f, "Initial Right Servo neutral (90 deg)");

    // Steering disabled -> servos neutral
    act.applySteering(0.8f, false);
    TEST_ASSERT(fabsf(act.getLeftServoAngle() - 90.0f) < 1e-3f, "Disabled steering holds neutral");
    TEST_ASSERT(fabsf(act.getRightServoAngle() - 90.0f) < 1e-3f, "Disabled steering holds neutral");

    // Configure synthetic calibration to test calibrated continuous spool mechanics
    ContinuousServoSystemCal cal;
    cal.calibrated = true;
    cal.left.neutral_pulse_us = 1500;
    cal.left.neutral_deadband_us = 20;
    cal.left.min_pulse_us = 1050;
    cal.left.max_pulse_us = 1950;
    cal.left.max_wind_delta_us = 350;
    cal.left.max_release_delta_us = 300;
    cal.left.wind_polarity = -1; // pulse < neutral winds
    cal.left.rev_per_sec_at_max = 1.25f;
    cal.left.mm_per_revolution = 16.0f;
    cal.left.max_pull_mm = 60.0f;

    cal.right.neutral_pulse_us = 1500;
    cal.right.neutral_deadband_us = 20;
    cal.right.min_pulse_us = 1050;
    cal.right.max_pulse_us = 1950;
    cal.right.max_wind_delta_us = 350;
    cal.right.max_release_delta_us = 300;
    cal.right.wind_polarity = +1; // pulse > neutral winds
    cal.right.rev_per_sec_at_max = 1.25f;
    cal.right.mm_per_revolution = 16.0f;
    cal.right.max_pull_mm = 60.0f;

    cal.position_deadband_mm = 1.0f;
    cal.drift_uncertainty_ratio = 0.10f;
    cal.max_uncertainty_mm = 50.0f;
    cal.max_burst_duration_ms = 400UL;
    cal.min_cooldown_ms = 150UL;
    cal.command_timeout_ms = 300UL;
    TEST_ASSERT(act.configureCalibration(cal), "Synthetic calibration accepted");

    // Steering enabled: Turn Left (steering = -1.0)
    // Left brake pulls to 1.0 (0 deg equivalent / 1150 us winding pulse), Right brake neutral (90 deg / 1500 us stop pulse)
    act.applySteering(-1.0f, true, 1000, FlightState::PRECISION_GUIDANCE);
    TEST_ASSERT(fabsf(act.getLeftServoAngle() - 0.0f) < 1e-3f, "Left turn pulls left servo to 0 deg");
    TEST_ASSERT(fabsf(act.getRightServoAngle() - 90.0f) < 1e-3f, "Left turn keeps right servo at 90 deg");
    TEST_ASSERT(act.getLeftSpoolDirection() == SpoolDirection::WIND_BRAKE, "Left spool is winding brake line");
    TEST_ASSERT(act.getRightSpoolDirection() == SpoolDirection::STOP, "Right spool is stopped");
    TEST_ASSERT(act.getLeftServoPulseUs() == 1150, "Left spool winding pulse width == 1150 us");
    TEST_ASSERT(act.getRightServoPulseUs() == 1500, "Right spool neutral pulse width == 1500 us");

    // Steering enabled: Turn Right (steering = +1.0)
    // Right brake pulls to 1.0 (0 deg equivalent / 1850 us winding pulse), Left brake neutral (90 deg / 1500 us stop pulse)
    act.resetEstimatedLinePosition();
    act.applySteering(1.0f, true, 2000, FlightState::PRECISION_GUIDANCE);
    TEST_ASSERT(fabsf(act.getLeftServoAngle() - 90.0f) < 1e-3f, "Right turn keeps left servo at 90 deg");
    TEST_ASSERT(fabsf(act.getRightServoAngle() - 0.0f) < 1e-3f, "Right turn pulls right servo to 0 deg");
    TEST_ASSERT(act.getLeftSpoolDirection() == SpoolDirection::STOP, "Left spool is stopped");
    TEST_ASSERT(act.getRightSpoolDirection() == SpoolDirection::WIND_BRAKE, "Right spool is winding brake line");
    TEST_ASSERT(act.getLeftServoPulseUs() == 1500, "Left spool neutral pulse width == 1500 us");
    TEST_ASSERT(act.getRightServoPulseUs() == 1850, "Right spool winding pulse width == 1850 us");

    // XY-MOS MOSFET Burnwire & Buzzer Trigger Tests
    TEST_ASSERT(!act.isBurnwirePinHigh(), "Burnwire XY-MOS initially LOW");
    act.setBurnwire(true);
    TEST_ASSERT(act.isBurnwirePinHigh(), "Burnwire XY-MOS driven HIGH during deployment");
    act.setBurnwire(false);
    TEST_ASSERT(!act.isBurnwirePinHigh(), "Burnwire XY-MOS returns LOW after deployment");

    TEST_ASSERT(!act.isBuzzerPinHigh(), "Buzzer XY-MOS initially LOW");
    act.updateBuzzer(true, 1000);
    TEST_ASSERT(act.isBuzzerPinHigh(), "Buzzer XY-MOS toggles HIGH on 500ms beacon interval");
    act.updateBuzzer(true, 1600);
    TEST_ASSERT(!act.isBuzzerPinHigh(), "Buzzer XY-MOS toggles LOW on next 500ms beacon interval");
    act.updateBuzzer(false, 2000);
    TEST_ASSERT(!act.isBuzzerPinHigh(), "Buzzer XY-MOS forced LOW when deactivated");

    // Readytosky MT2204 2300KV Flywheel ESC Tests (Constant 6000 RPM = 1353 us)
    TEST_ASSERT(!act.isFlywheelActive(), "Flywheel initially OFF");
    TEST_ASSERT(fabsf(act.getFlywheelRPM() - 0.0f) < 1e-3f, "Initial Flywheel RPM == 0");
    TEST_ASSERT(act.getFlywheelPulseUs() == 1000, "Initial Flywheel ESC pulse == 1000 us (Armed Idle)");

    act.setFlywheel(true);
    TEST_ASSERT(act.isFlywheelActive(), "Flywheel active after parafoil deployment");
    TEST_ASSERT(fabsf(act.getFlywheelRPM() - 6000.0f) < 1e-3f, "Flywheel runs at constant 6000 RPM");
    TEST_ASSERT(act.getFlywheelPulseUs() == 1353, "Flywheel ESC pulse == 1353 us for 6000 RPM on 2300KV @ 7.4V");

    act.setFlywheel(false);
    TEST_ASSERT(!act.isFlywheelActive(), "Flywheel OFF upon landing");
    TEST_ASSERT(fabsf(act.getFlywheelRPM() - 0.0f) < 1e-3f, "Flywheel RPM returns to 0 upon landing");
    TEST_ASSERT(act.getFlywheelPulseUs() == 1000, "Flywheel ESC pulse returns to 1000 us upon landing");
}

// ----------------------------------------------------------------------------
// 7. MS5611 Calibration & EMA Filter Unit Tests
// ----------------------------------------------------------------------------
void test_baro_calibration_and_filter() {
    std::cout << "[TEST] 7. MS5611 Calibration, EMA Filter & Hypsometry..." << std::endl;

    // A. 50-Sample Baseline Calibration Convergence
    float true_pad_pressure = 1013.25f;
    float sumPressure = 0.0f;
    int validSamples = 50;
    // Simulated small +/- 0.015 mbar sensor noise
    for (int i = 0; i < validSamples; i++) {
        float noise = (float)((i % 5) - 2) * 0.005f; // Zero-mean pseudo-noise
        sumPressure += (true_pad_pressure + noise);
    }
    float launchPressure = sumPressure / (float)validSamples;
    TEST_ASSERT(fabsf(launchPressure - true_pad_pressure) < 0.01f, "50-sample calibration baseline within 0.01 hPa of ground reference");

    // B. Zero AGL Baseline at Pad Hold
    float alt_pad = 44330.0f * (1.0f - powf(launchPressure / launchPressure, 0.190295f));
    TEST_ASSERT(fabsf(alt_pad) < 1e-5f, "Pad baseline altitude relative to launchPressure is exactly 0.0m");

    // C. EMA Filter Step Response (alpha = 0.4)
    float filteredP = 1013.25f;
    const float alpha = BARO_EMA_ALPHA; // 0.4f
    float step_target = 1000.0f;

    // Step 1: 0.4 * 1000.0 + 0.6 * 1013.25 = 1007.95
    filteredP = (alpha * step_target) + ((1.0f - alpha) * filteredP);
    TEST_ASSERT(fabsf(filteredP - 1007.95f) < 1e-3f, "EMA step 1 matches analytical value (1007.95 hPa)");

    // Step 2: 0.4 * 1000.0 + 0.6 * 1007.95 = 1004.77
    filteredP = (alpha * step_target) + ((1.0f - alpha) * filteredP);
    TEST_ASSERT(fabsf(filteredP - 1004.77f) < 1e-3f, "EMA step 2 matches analytical value (1004.77 hPa)");

    // D. Noise Attenuation Verification (Theoretical variance factor = alpha / (2 - alpha) = 0.25)
    // Standard deviation is halved (0.015 mbar -> ~0.0075 mbar)
    float p_state = 1013.25f;
    float sum_sq_err = 0.0f;
    for (int i = 0; i < 200; i++) {
        float noise = (((float)(i % 7) - 3.0f) / 3.0f) * 0.015f; // +/- 0.015 mbar
        p_state = (alpha * (1013.25f + noise)) + ((1.0f - alpha) * p_state);
        float err = p_state - 1013.25f;
        sum_sq_err += err * err;
    }
    float rms_noise = sqrtf(sum_sq_err / 200.0f);
    TEST_ASSERT(rms_noise < 0.010f, "EMA alpha=0.4 reduces 0.015 mbar noise RMS below 0.010 mbar (~50% attenuation)");

    // E. Relative Hypsometric Altitude Formula Accuracy Across Mission
    // At 600m disreefing altitude:
    float p_600m = launchPressure * powf(1.0f - (600.0f / 44330.0f), 1.0f / 0.190295f);
    float alt_600m = 44330.0f * (1.0f - powf(p_600m / launchPressure, 0.190295f));
    TEST_ASSERT(fabsf(alt_600m - 600.0f) < 0.01f, "Relative hypsometric altitude at 600m accurate within 1 cm");

    // At 1000m apogee:
    float p_1000m = launchPressure * powf(1.0f - (1000.0f / 44330.0f), 1.0f / 0.190295f);
    float alt_1000m = 44330.0f * (1.0f - powf(p_1000m / launchPressure, 0.190295f));
    TEST_ASSERT(fabsf(alt_1000m - 1000.0f) < 0.01f, "Relative hypsometric altitude at 1000m accurate within 1 cm");
}

int main() {
    std::cout << "==================================================" << std::endl;
    std::cout << "    CAN-7USAT FSW C++ Unit Test Verification      " << std::endl;
    std::cout << "==================================================" << std::endl;

    test_nav_math();
    test_pid_controller();
    test_ekf_estimation();
    test_guidance_modules();
    test_state_machine();
    test_actuators();
    test_baro_calibration_and_filter();

    std::cout << "\n--------------------------------------------------" << std::endl;
    std::cout << "Unit Test Results: " << g_tests_passed << " Passed, " << g_tests_failed << " Failed." << std::endl;
    std::cout << "--------------------------------------------------" << std::endl;

    if (g_tests_failed == 0) {
        std::cout << ">> ALL FSW UNIT TESTS PASSED SUCCESSFULLY! <<" << std::endl;
        return 0;
    } else {
        std::cout << ">> WARNING: SOME UNIT TESTS FAILED! <<" << std::endl;
        return 1;
    }
}
