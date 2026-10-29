#include "actuators.h"
#include <stdio.h>

#if defined(ARDUINO) || defined(__arm__) || defined(TEENSYDUINO)
#include <Arduino.h>

// ============================================================================
// Teensy 4.1 Hardware FlexPWM Driver & IntervalTimer ISR Safety Cutoff
//   Pin 2 (Left Continuous Servo)  -> FlexPWM4_2_A (50 Hz)
//   Pin 3 (Right Continuous Servo) -> FlexPWM4_2_B (50 Hz)
//   Pin 6 (Flywheel ESC)           -> FlexPWM2_2_A (50 Hz)
// ============================================================================
static int hwFlywheelPulseUs = ESC_MIN_PULSE_US;
static uint32_t lastFlywheelRampMs = 0;
static ActuatorManager* g_activeActuatorMgr = nullptr;

#if defined(TEENSYDUINO)
static IntervalTimer g_servoSafetyTimer;
static void servoSafetyTimerIsr() {
    if (g_activeActuatorMgr != nullptr) {
        g_activeActuatorMgr->onHardwareTimerTick(millis());
    }
}
#endif

static inline void writeHardwarePwmUs(uint8_t pin, uint16_t pulse_us) {
    if (pulse_us == 0) {
        analogWrite(pin, 0); // 0% duty cycle -> Pin held LOW (detached / 0 mA standby)
        return;
    }
    // 50 Hz period = 20,000 us; 16-bit resolution = 65536 counts per 20,000 us
    uint32_t duty = ((uint32_t)pulse_us * 65536UL + 10000UL) / 20000UL;
    if (duty > 65535UL) duty = 65535UL;
    analogWrite(pin, (int)duty);
}
#endif

static inline float clampf(float val, float lo, float hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

// Wraparound-safe unsigned 32-bit modular elapsed time helper.
// Correctly computes elapsed milliseconds across the 49.7-day uint32_t rollover
// (e.g. past_ms = 0xFFFFFFF0, now_ms = 0x00000020 -> diff = 48 ms).
// Treats deltas >= 0x80000000UL (~24.8 days) as non-forward/out-of-order timestamps.
static inline uint32_t elapsedMsWrapSafe(uint32_t now_ms, uint32_t past_ms) {
    uint32_t diff = now_ms - past_ms;
    return (diff > 0U && diff < 0x80000000UL) ? diff : 0U;
}

static inline BrakeCommand splitSteeringToBrakes(float clamped_steering) {
    BrakeCommand b = {0.0f, 0.0f};
    if (clamped_steering < 0.0f) {
        b.left  = -clamped_steering;
        b.right = 0.0f;
    } else if (clamped_steering > 0.0f) {
        b.left  = 0.0f;
        b.right = clamped_steering;
    }
    return b;
}

ContinuousServoSystemCal ActuatorManager::getDefaultConfigCalibration() {
    ContinuousServoSystemCal cal;
    cal.calibrated = (ACTUATOR_CALIBRATED != 0);

    cal.left.neutral_pulse_us     = (uint16_t)SERVO_LEFT_NEUTRAL_US;
    cal.left.neutral_deadband_us  = (uint16_t)SERVO_LEFT_DEADBAND_US;
    cal.left.min_pulse_us         = (uint16_t)SERVO_LEFT_MIN_PULSE_US;
    cal.left.max_pulse_us         = (uint16_t)SERVO_LEFT_MAX_PULSE_US;
    cal.left.max_wind_delta_us    = (uint16_t)SERVO_LEFT_MAX_WIND_DELTA_US;
    cal.left.max_release_delta_us = (uint16_t)SERVO_LEFT_MAX_RELEASE_DELTA_US;
    cal.left.wind_polarity        = (int8_t)SERVO_LEFT_WIND_POLARITY;
    cal.left.rev_per_sec_at_max   = SERVO_LEFT_REV_PER_SEC_MAX;
    cal.left.mm_per_revolution    = SERVO_LEFT_MM_PER_REV;
    cal.left.max_pull_mm          = SERVO_LEFT_MAX_PULL_MM;

    cal.right.neutral_pulse_us     = (uint16_t)SERVO_RIGHT_NEUTRAL_US;
    cal.right.neutral_deadband_us  = (uint16_t)SERVO_RIGHT_DEADBAND_US;
    cal.right.min_pulse_us         = (uint16_t)SERVO_RIGHT_MIN_PULSE_US;
    cal.right.max_pulse_us         = (uint16_t)SERVO_RIGHT_MAX_PULSE_US;
    cal.right.max_wind_delta_us    = (uint16_t)SERVO_RIGHT_MAX_WIND_DELTA_US;
    cal.right.max_release_delta_us = (uint16_t)SERVO_RIGHT_MAX_RELEASE_DELTA_US;
    cal.right.wind_polarity        = (int8_t)SERVO_RIGHT_WIND_POLARITY;
    cal.right.rev_per_sec_at_max   = SERVO_RIGHT_REV_PER_SEC_MAX;
    cal.right.mm_per_revolution    = SERVO_RIGHT_MM_PER_REV;
    cal.right.max_pull_mm          = SERVO_RIGHT_MAX_PULL_MM;

    cal.position_deadband_mm    = SERVO_SPOOL_DEADBAND_MM;
    cal.drift_uncertainty_ratio = SERVO_DRIFT_UNCERTAINTY_RATIO;
    cal.max_uncertainty_mm      = SERVO_MAX_UNCERTAINTY_MM;
    cal.max_burst_duration_ms   = (uint32_t)SERVO_MAX_BURST_DURATION_MS;
    cal.min_cooldown_ms         = (uint32_t)SERVO_MIN_COOLDOWN_MS;
    cal.command_timeout_ms      = (uint32_t)SERVO_COMMAND_TIMEOUT_MS;
    return cal;
}

bool ActuatorManager::validateChannelCalibration(const ContinuousServoChannelCal& ch) {
    // 1. Safe RC PWM electrical envelope [800, 2200] us
    if (ch.min_pulse_us < 800 || ch.max_pulse_us > 2200) return false;
    if (ch.min_pulse_us >= ch.max_pulse_us) return false;

    // 2. Neutral pulse must sit strictly inside [min_pulse_us, max_pulse_us]
    if (ch.neutral_pulse_us <= ch.min_pulse_us || ch.neutral_pulse_us >= ch.max_pulse_us) return false;

    // 3. Deadband and deltas must be positive and fit within [min_pulse_us, max_pulse_us]
    if (ch.neutral_deadband_us == 0 || ch.neutral_deadband_us >= 150) return false;
    if (ch.max_wind_delta_us <= ch.neutral_deadband_us || ch.max_wind_delta_us > 700) return false;
    if (ch.max_release_delta_us <= ch.neutral_deadband_us || ch.max_release_delta_us > 700) return false;

    // 4. Winding polarity must be explicitly +1 or -1
    if (ch.wind_polarity != 1 && ch.wind_polarity != -1) return false;

    int32_t wind_limit_us = (int32_t)ch.neutral_pulse_us + (int32_t)ch.wind_polarity * (int32_t)ch.max_wind_delta_us;
    int32_t rel_limit_us  = (int32_t)ch.neutral_pulse_us - (int32_t)ch.wind_polarity * (int32_t)ch.max_release_delta_us;
    if (wind_limit_us < (int32_t)ch.min_pulse_us || wind_limit_us > (int32_t)ch.max_pulse_us) return false;
    if (rel_limit_us < (int32_t)ch.min_pulse_us || rel_limit_us > (int32_t)ch.max_pulse_us) return false;

    // 5. Bench-measured M4 spool kinematics must be finite and strictly positive
    if (!isfinite(ch.rev_per_sec_at_max) || ch.rev_per_sec_at_max <= 0.0f || ch.rev_per_sec_at_max > 20.0f) return false;
    if (!isfinite(ch.mm_per_revolution) || ch.mm_per_revolution <= 0.0f || ch.mm_per_revolution > 50.0f) return false;
    if (!isfinite(ch.max_pull_mm) || ch.max_pull_mm <= 0.0f || ch.max_pull_mm > 500.0f) return false;

    return true;
}

bool ActuatorManager::validateSystemCalibration(const ContinuousServoSystemCal& cal) {
    if (!cal.calibrated) return false;
    if (!validateChannelCalibration(cal.left)) return false;
    if (!validateChannelCalibration(cal.right)) return false;
    if (!isfinite(cal.position_deadband_mm) || cal.position_deadband_mm <= 0.0f ||
        cal.position_deadband_mm >= cal.left.max_pull_mm ||
        cal.position_deadband_mm >= cal.right.max_pull_mm) {
        return false;
    }
    if (!isfinite(cal.drift_uncertainty_ratio) || cal.drift_uncertainty_ratio < 0.0f || cal.drift_uncertainty_ratio > 1.0f) {
        return false;
    }
    if (!isfinite(cal.max_uncertainty_mm) || cal.max_uncertainty_mm <= 0.0f) {
        return false;
    }
    if (cal.max_burst_duration_ms == 0 || cal.max_burst_duration_ms > 5000UL) return false;
    if (cal.command_timeout_ms == 0 || cal.command_timeout_ms > 5000UL) return false;
    return true;
}

bool ActuatorManager::isFlightStateSteeringPermitted(FlightState state) {
    return (state == FlightState::PRECISION_GUIDANCE ||
            state == FlightState::TERMINAL_GUIDANCE ||
            state == FlightState::FINAL_APPROACH);
}

ActuatorManager::ActuatorManager()
    : calibration(getDefaultConfigCalibration()),
      calibration_valid(false),
      requested_steering_effort(0.0f),
      steering_enabled(false),
      servos_attached(false),
      has_command_timestamp(false),
      last_command_ms(0),
      last_update_ms(0),
      left_step_start_ms(0),
      right_step_start_ms(0),
      last_fault(ActuatorFaultCode::UNCALIBRATED),
      pwm_write_cb(nullptr),
      burnwire_state(false),
      buzzer_state(false),
      flywheel_state(false),
      current_flywheel_rpm(0.0f),
      current_flywheel_pulse_us(ESC_MIN_PULSE_US),
      last_buzzer_toggle_ms(0) {
    calibration_valid = validateSystemCalibration(calibration);
    initChannelState(left_state, calibration_valid);
    initChannelState(right_state, calibration_valid);
    last_fault = calibration_valid
        ? ActuatorFaultCode::NONE
        : (calibration.calibrated ? ActuatorFaultCode::INVALID_CALIBRATION : ActuatorFaultCode::UNCALIBRATED);
}

void ActuatorManager::initChannelState(ContinuousServoChannelState& ch, bool is_cal_valid) {
    ch.target_brake_norm         = 0.0f;
    ch.direction                 = SpoolDirection::STOP;
    ch.commanded_speed_norm      = 0.0f;
    ch.commanded_pulse_us        = 0;
    ch.planned_burst_ms          = 0;
    ch.current_burst_ms          = 0;
    ch.cooldown_remaining_ms     = 0;
    ch.cumulative_wind_ms        = 0;
    ch.cumulative_release_ms     = 0;
    ch.cumulative_travel_mm      = 0.0f;
    ch.estimated_line_pull_mm    = is_cal_valid ? 0.0f : NAN;
    ch.estimated_uncertainty_mm  = is_cal_valid ? 0.0f : NAN;
    ch.actual_position_known     = false; // Hardware has no position sensor
    ch.neutral_position_verified = is_cal_valid; // False once any open-loop motion occurs
    ch.burst_limit_latched       = false;
    ch.drift_limit_latched       = false;
    ch.latched_burst_direction   = SpoolDirection::STOP;
}

void ActuatorManager::init() {
#if defined(ARDUINO) || defined(__arm__) || defined(TEENSYDUINO)
    g_activeActuatorMgr = this;

    // XY-MOS MOSFET #1: Nichrome Burnwire Parafoil Disreefing
    pinMode(BURNWIRE_PIN, OUTPUT);
    digitalWrite(BURNWIRE_PIN, LOW);

    // XY-MOS MOSFET #2: Recovery Audio Beacon Buzzer
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // Configure 16-bit 50 Hz Hardware FlexPWM on Pins 2, 3, and 6
    analogWriteResolution(16);
    analogWriteFrequency(SERVO_LEFT_PIN, 50.0f);
    analogWriteFrequency(SERVO_RIGHT_PIN, 50.0f);
    analogWriteFrequency(FLYWHEEL_ESC_PIN, 50.0f);

    // Keep continuous-rotation servos detached initially (0% duty = LOW)
    writeHardwarePwmUs(SERVO_LEFT_PIN, 0);
    writeHardwarePwmUs(SERVO_RIGHT_PIN, 0);

    // Readytosky MT2204 2300KV Flywheel ESC: Arm at exact 1000 us (0 RPM) on startup
    hwFlywheelPulseUs = ESC_MIN_PULSE_US;
    writeHardwarePwmUs(FLYWHEEL_ESC_PIN, ESC_MIN_PULSE_US);

#if defined(TEENSYDUINO)
    // Start independent hardware PIT timer ISR (10 ms) to guarantee servo cutoff
    // even if the main control loop stalls in I2C or SD card I/O
    g_servoSafetyTimer.begin(servoSafetyTimerIsr, SERVO_SAFETY_ISR_PERIOD_US);
#endif
#endif
    calibration = getDefaultConfigCalibration();
    calibration_valid = validateSystemCalibration(calibration);
    initChannelState(left_state, calibration_valid);
    initChannelState(right_state, calibration_valid);

    requested_steering_effort = 0.0f;
    steering_enabled          = false;
    servos_attached           = false;
    has_command_timestamp     = false;
    last_command_ms           = 0;
    last_update_ms            = 0;
    left_step_start_ms        = 0;
    right_step_start_ms       = 0;
    last_fault = calibration_valid
        ? ActuatorFaultCode::NONE
        : (calibration.calibrated ? ActuatorFaultCode::INVALID_CALIBRATION : ActuatorFaultCode::UNCALIBRATED);

    burnwire_state            = false;
    buzzer_state              = false;
    flywheel_state            = false;
    current_flywheel_rpm      = 0.0f;
    current_flywheel_pulse_us = ESC_MIN_PULSE_US;
    writeServoOutputs();
}

bool ActuatorManager::configureCalibration(const ContinuousServoSystemCal& cal) {
    stopServosImmediate(true);
    calibration = cal;
    calibration_valid = validateSystemCalibration(cal);
    initChannelState(left_state, calibration_valid);
    initChannelState(right_state, calibration_valid);

    if (!calibration_valid) {
        last_fault = cal.calibrated ? ActuatorFaultCode::INVALID_CALIBRATION : ActuatorFaultCode::UNCALIBRATED;
        return false;
    }
    last_fault = ActuatorFaultCode::NONE;
    return true;
}

void ActuatorManager::resetEstimatedLinePosition() {
    initChannelState(left_state, calibration_valid);
    initChannelState(right_state, calibration_valid);
    if (calibration_valid && last_fault == ActuatorFaultCode::OPEN_LOOP_DRIFT_LIMIT) {
        last_fault = ActuatorFaultCode::NONE;
    }
}

void ActuatorManager::stopServosImmediate(bool detach_pwm) {
    left_state.direction            = SpoolDirection::STOP;
    right_state.direction           = SpoolDirection::STOP;
    left_state.commanded_speed_norm  = 0.0f;
    right_state.commanded_speed_norm = 0.0f;
    left_state.planned_burst_ms      = 0;
    right_state.planned_burst_ms     = 0;
    left_state.current_burst_ms      = 0;
    right_state.current_burst_ms     = 0;

    if (detach_pwm || !calibration_valid) {
        left_state.commanded_pulse_us  = 0;
        right_state.commanded_pulse_us = 0;
        servos_attached                = false;
    } else {
        left_state.commanded_pulse_us  = calibration.left.neutral_pulse_us;
        right_state.commanded_pulse_us = calibration.right.neutral_pulse_us;
    }
    writeServoOutputs();
}

void ActuatorManager::setDirectPulse(uint16_t left_us, uint16_t right_us) {
    left_state.commanded_pulse_us = left_us;
    right_state.commanded_pulse_us = right_us;
    steering_enabled = true;
    servos_attached = true;
    writeServoOutputs();
}

void ActuatorManager::writeServoOutputs() {
    if (pwm_write_cb != nullptr) {
        pwm_write_cb(SERVO_LEFT_PIN, left_state.commanded_pulse_us);
        pwm_write_cb(SERVO_RIGHT_PIN, right_state.commanded_pulse_us);
    }
#if defined(ARDUINO) || defined(__arm__) || defined(TEENSYDUINO)
    writeHardwarePwmUs(SERVO_LEFT_PIN, left_state.commanded_pulse_us);
    writeHardwarePwmUs(SERVO_RIGHT_PIN, right_state.commanded_pulse_us);
#endif
}

void ActuatorManager::advanceChannelEstimate(
    ContinuousServoChannelState& ch,
    const ContinuousServoChannelCal& cal,
    uint32_t dt_ms
) {
    ch.actual_position_known = false;
    if (!calibration_valid || dt_ms == 0) return;

    // 1. Tick cooldown timer if burst limit was previously latched
    if (ch.burst_limit_latched) {
        if (dt_ms >= ch.cooldown_remaining_ms) {
            ch.cooldown_remaining_ms   = 0;
            ch.burst_limit_latched     = false;
            ch.latched_burst_direction = SpoolDirection::STOP;
        } else {
            ch.cooldown_remaining_ms -= dt_ms;
        }
    }

    if (ch.direction == SpoolDirection::STOP || ch.commanded_speed_norm <= 0.0f) {
        ch.current_burst_ms = 0;
        ch.planned_burst_ms = 0;
        return;
    }

    // 2. Integrate TRUE elapsed physical motor rotation time dt_ms without artificial truncation.
    //    Any open-loop movement invalidates verified physical neutral status and increases uncertainty.
    float max_linear_speed_mm_s = cal.rev_per_sec_at_max * cal.mm_per_revolution;
    float d_pull_mm = ch.commanded_speed_norm * max_linear_speed_mm_s * ((float)dt_ms / 1000.0f);

    if (d_pull_mm > 0.0f) {
        ch.neutral_position_verified = false;
        ch.cumulative_travel_mm     += d_pull_mm;
        ch.estimated_uncertainty_mm += d_pull_mm * calibration.drift_uncertainty_ratio;
    }

    if (ch.direction == SpoolDirection::WIND_BRAKE) {
        ch.cumulative_wind_ms += dt_ms;
        float next_pull = ch.estimated_line_pull_mm + d_pull_mm;
        if (next_pull >= cal.max_pull_mm) {
            next_pull = cal.max_pull_mm;
            last_fault = ActuatorFaultCode::TRAVEL_LIMIT_REACHED;
        }
        ch.estimated_line_pull_mm = next_pull;
    } else if (ch.direction == SpoolDirection::RELEASE_BRAKE) {
        ch.cumulative_release_ms += dt_ms;
        float next_pull = ch.estimated_line_pull_mm - d_pull_mm;
        if (next_pull < 0.0f) {
            next_pull = 0.0f; // Floor at 0.0 mm estimated pull; neutral_position_verified remains false!
        }
        ch.estimated_line_pull_mm = next_pull;
    }

    // 3. Check cumulative open-loop drift uncertainty limit
    if (ch.estimated_uncertainty_mm >= calibration.max_uncertainty_mm) {
        ch.drift_limit_latched  = true;
        ch.direction            = SpoolDirection::STOP;
        ch.commanded_speed_norm = 0.0f;
        ch.commanded_pulse_us   = cal.neutral_pulse_us;
        ch.current_burst_ms     = 0;
        ch.planned_burst_ms     = 0;
        last_fault              = ActuatorFaultCode::OPEN_LOOP_DRIFT_LIMIT;
        return;
    }

    // 4. Enforce per-burst maximum continuous activation duration
    ch.current_burst_ms += dt_ms;
    if (ch.current_burst_ms >= calibration.max_burst_duration_ms) {
        ch.latched_burst_direction = ch.direction;
        ch.burst_limit_latched     = true;
        ch.cooldown_remaining_ms   = calibration.min_cooldown_ms;
        ch.direction               = SpoolDirection::STOP;
        ch.commanded_speed_norm    = 0.0f;
        ch.commanded_pulse_us      = cal.neutral_pulse_us;
        ch.current_burst_ms        = 0;
        ch.planned_burst_ms        = 0;
        last_fault                 = ActuatorFaultCode::MAX_BURST_EXCEEDED;
    }
}

void ActuatorManager::computeChannelCommand(
    ContinuousServoChannelState& ch,
    const ContinuousServoChannelCal& cal,
    uint32_t now_ms,
    uint32_t& step_start_ms
) {
    ch.actual_position_known = false;
    if (!calibration_valid || !steering_enabled) {
        ch.direction            = SpoolDirection::STOP;
        ch.commanded_speed_norm = 0.0f;
        ch.commanded_pulse_us   = 0;
        ch.planned_burst_ms     = 0;
        ch.current_burst_ms     = 0;
        return;
    }

    // 1. Hold STOP if cumulative open-loop drift uncertainty budget is exhausted
    if (ch.drift_limit_latched || ch.estimated_uncertainty_mm >= calibration.max_uncertainty_mm) {
        ch.drift_limit_latched  = true;
        ch.direction            = SpoolDirection::STOP;
        ch.commanded_speed_norm = 0.0f;
        ch.commanded_pulse_us   = cal.neutral_pulse_us;
        ch.planned_burst_ms     = 0;
        ch.current_burst_ms     = 0;
        last_fault              = ActuatorFaultCode::OPEN_LOOP_DRIFT_LIMIT;
        return;
    }

    float target_pull_mm = clampf(ch.target_brake_norm, 0.0f, 1.0f) * cal.max_pull_mm;
    float error_mm = target_pull_mm - ch.estimated_line_pull_mm;

    // 2. Within open-loop travel deadband -> STOP at calibrated neutral pulse
    if (fabsf(error_mm) <= calibration.position_deadband_mm) {
        ch.direction            = SpoolDirection::STOP;
        ch.commanded_speed_norm = 0.0f;
        ch.commanded_pulse_us   = cal.neutral_pulse_us;
        ch.planned_burst_ms     = 0;
        ch.current_burst_ms     = 0;
        return;
    }

    SpoolDirection desired_dir = (error_mm > 0.0f)
        ? SpoolDirection::WIND_BRAKE
        : SpoolDirection::RELEASE_BRAKE;

    // If reversing direction, reset current_burst_ms and clear opposite-direction burst latch
    if (ch.direction != desired_dir) {
        ch.current_burst_ms = 0;
        if (ch.burst_limit_latched && ch.latched_burst_direction != desired_dir) {
            ch.burst_limit_latched     = false;
            ch.cooldown_remaining_ms   = 0;
            ch.latched_burst_direction = SpoolDirection::STOP;
        }
    }

    // 3. Enforce burst cooldown hold if this direction hit max_burst_duration_ms
    if (ch.burst_limit_latched && ch.latched_burst_direction == desired_dir) {
        ch.direction            = SpoolDirection::STOP;
        ch.commanded_speed_norm = 0.0f;
        ch.commanded_pulse_us   = cal.neutral_pulse_us;
        ch.planned_burst_ms     = 0;
        return;
    }

    // 4. Enforce open-loop travel bounds [0.0, max_pull_mm]
    if (desired_dir == SpoolDirection::WIND_BRAKE && ch.estimated_line_pull_mm >= cal.max_pull_mm) {
        ch.direction            = SpoolDirection::STOP;
        ch.commanded_speed_norm = 0.0f;
        ch.commanded_pulse_us   = cal.neutral_pulse_us;
        ch.planned_burst_ms     = 0;
        ch.current_burst_ms     = 0;
        last_fault              = ActuatorFaultCode::TRAVEL_LIMIT_REACHED;
        return;
    }
    if (desired_dir == SpoolDirection::RELEASE_BRAKE && ch.estimated_line_pull_mm <= 0.0f) {
        ch.direction            = SpoolDirection::STOP;
        ch.commanded_speed_norm = 0.0f;
        ch.commanded_pulse_us   = cal.neutral_pulse_us;
        ch.planned_burst_ms     = 0;
        ch.current_burst_ms     = 0;
        return;
    }

    // 5. Compute speed_norm and planned_burst_ms so the channel reaches target_pull_mm
    //    without overshooting during the 10 Hz (100 ms) control interval or exceeding
    //    the remaining burst budget.
    uint32_t control_period_ms = 1000UL / CONTROL_LOOP_HZ;
    uint32_t rem_burst_ms = (ch.current_burst_ms < calibration.max_burst_duration_ms)
        ? (calibration.max_burst_duration_ms - ch.current_burst_ms)
        : 0;
    if (rem_burst_ms == 0) {
        ch.latched_burst_direction = desired_dir;
        ch.burst_limit_latched     = true;
        ch.cooldown_remaining_ms   = calibration.min_cooldown_ms;
        ch.direction               = SpoolDirection::STOP;
        ch.commanded_speed_norm    = 0.0f;
        ch.commanded_pulse_us      = cal.neutral_pulse_us;
        ch.planned_burst_ms        = 0;
        ch.current_burst_ms        = 0;
        last_fault                 = ActuatorFaultCode::MAX_BURST_EXCEEDED;
        return;
    }

    uint32_t step_budget_ms = (control_period_ms <= rem_burst_ms) ? control_period_ms : rem_burst_ms;
    float max_linear_speed_mm_s = cal.rev_per_sec_at_max * cal.mm_per_revolution;
    float max_step_mm = max_linear_speed_mm_s * ((float)step_budget_ms / 1000.0f);
    float abs_err_mm = fabsf(error_mm);

    float speed_norm = 1.0f;
    uint32_t planned_ms = step_budget_ms;

    if (max_step_mm > 1e-4f && abs_err_mm < max_step_mm) {
        float exact_speed = abs_err_mm / max_step_mm;
        if (exact_speed >= 0.20f) {
            speed_norm = exact_speed;
            planned_ms = step_budget_ms;
        } else {
            speed_norm = 0.20f;
            float needed_ms_f = (abs_err_mm / (0.20f * max_linear_speed_mm_s)) * 1000.0f;
            uint32_t needed_ms = (uint32_t)roundf(needed_ms_f);
            if (needed_ms < 1U) needed_ms = 1U;
            if (needed_ms > step_budget_ms) needed_ms = step_budget_ms;
            planned_ms = needed_ms;
        }
    }

    // 6. Convert speed_norm & direction into calibrated microsecond PWM pulse
    uint16_t max_delta_us = (desired_dir == SpoolDirection::WIND_BRAKE)
        ? cal.max_wind_delta_us
        : cal.max_release_delta_us;
    int32_t polarity_sign = (desired_dir == SpoolDirection::WIND_BRAKE)
        ? (int32_t)cal.wind_polarity
        : -(int32_t)cal.wind_polarity;

    float active_span_us = (float)(max_delta_us - cal.neutral_deadband_us);
    int32_t offset_us = (int32_t)cal.neutral_deadband_us + (int32_t)roundf(speed_norm * active_span_us);
    int32_t raw_pulse_us = (int32_t)cal.neutral_pulse_us + polarity_sign * offset_us;

    if (raw_pulse_us < (int32_t)cal.min_pulse_us) raw_pulse_us = (int32_t)cal.min_pulse_us;
    if (raw_pulse_us > (int32_t)cal.max_pulse_us) raw_pulse_us = (int32_t)cal.max_pulse_us;

    ch.direction            = desired_dir;
    ch.commanded_speed_norm = speed_norm;
    ch.commanded_pulse_us   = (uint16_t)raw_pulse_us;
    ch.planned_burst_ms     = planned_ms;
    step_start_ms           = now_ms;
}

void ActuatorManager::applySteering(
    float steering,
    bool enabled,
    uint32_t now_ms,
    FlightState flight_state
) {
#if SERVO_USE_STANDARD_POSITIONAL
    bool state_permitted = isFlightStateSteeringPermitted(flight_state);
    if (!enabled || !state_permitted) {
        requested_steering_effort     = 0.0f;
        left_state.target_brake_norm  = 0.0f;
        right_state.target_brake_norm = 0.0f;
        left_state.commanded_pulse_us  = 0;
        right_state.commanded_pulse_us = 0;
        steering_enabled              = false;
        servos_attached               = false;
        has_command_timestamp         = false;
        last_update_ms                = now_ms;
        writeServoOutputs();
        if (enabled && !state_permitted) {
            last_fault = ActuatorFaultCode::PROHIBITED_FLIGHT_STATE;
        }
        return;
    }

    if (!isfinite(steering)) steering = 0.0f;
    float clamped_steering = clampf(steering, -1.0f, 1.0f);
    BrakeCommand brakes = splitSteeringToBrakes(clamped_steering);

    requested_steering_effort     = clamped_steering;
    left_state.target_brake_norm  = brakes.left;
    right_state.target_brake_norm = brakes.right;
    steering_enabled              = true;
    servos_attached               = true;
    has_command_timestamp         = true;
    last_command_ms               = now_ms;
    last_update_ms                = now_ms;
    last_fault                    = ActuatorFaultCode::NONE;

    // Linear mapping from [0.0, 1.0] brake to [90.0, 0.0] degrees, then to microseconds
    float lAng = SERVO_NEUTRAL_ANGLE + brakes.left * (SERVO_MAX_BRAKE_ANGLE - SERVO_NEUTRAL_ANGLE);
    float rAng = SERVO_NEUTRAL_ANGLE + brakes.right * (SERVO_MAX_BRAKE_ANGLE - SERVO_NEUTRAL_ANGLE);

    float lRatio = clampf(lAng / 90.0f, 0.0f, 1.0f);
    float rRatio = clampf(rAng / 90.0f, 0.0f, 1.0f);

    uint16_t lPulse = (uint16_t)(ES3004_MIN_PULSE_US + lRatio * (ES3004_NEUTRAL_PULSE_US - ES3004_MIN_PULSE_US));
    uint16_t rPulse = (uint16_t)(ES3004_MIN_PULSE_US + rRatio * (ES3004_NEUTRAL_PULSE_US - ES3004_MIN_PULSE_US));

    left_state.commanded_pulse_us  = lPulse;
    right_state.commanded_pulse_us = rPulse;

    writeServoOutputs();
    return;
#else
    // 1. Integrate any elapsed active rotation time prior to applying the new command
    if (has_command_timestamp && steering_enabled) {
        uint32_t dt_ms = elapsedMsWrapSafe(now_ms, last_update_ms);
        if (dt_ms > 0U) {
            advanceChannelEstimate(left_state, calibration.left, dt_ms);
            advanceChannelEstimate(right_state, calibration.right, dt_ms);
        }
    }
    last_update_ms = now_ms;

    bool state_permitted = isFlightStateSteeringPermitted(flight_state);

    // 2. Flight-state & enable gate
    if (!enabled || !state_permitted) {
        requested_steering_effort     = 0.0f;
        left_state.target_brake_norm  = 0.0f;
        right_state.target_brake_norm = 0.0f;
        steering_enabled              = false;
        has_command_timestamp         = false;
        stopServosImmediate(true);

        if (enabled && !state_permitted) {
            last_fault = ActuatorFaultCode::PROHIBITED_FLIGHT_STATE;
        } else if (!calibration_valid) {
            last_fault = calibration.calibrated
                ? ActuatorFaultCode::INVALID_CALIBRATION
                : ActuatorFaultCode::UNCALIBRATED;
        }
        return;
    }

    // 3. Non-finite (NaN / Inf) input safety gate
    if (!isfinite(steering)) {
        requested_steering_effort = 0.0f;
        last_fault = ActuatorFaultCode::NON_FINITE_INPUT;
        stopServosImmediate(!servos_attached);
        return;
    }

    // 4. Clamp steering to [-1.0, +1.0] and split into left/right brake effort
    float clamped_steering = clampf(steering, -1.0f, 1.0f);
    BrakeCommand brakes = splitSteeringToBrakes(clamped_steering);

    requested_steering_effort     = clamped_steering;
    left_state.target_brake_norm  = brakes.left;
    right_state.target_brake_norm = brakes.right;

    has_command_timestamp = true;
    last_command_ms       = now_ms;

    // 5. Calibration lockout gate: NEVER rotate physical servos when uncalibrated!
    if (!calibration_valid) {
        steering_enabled = false;
        last_fault = calibration.calibrated
            ? ActuatorFaultCode::INVALID_CALIBRATION
            : ActuatorFaultCode::UNCALIBRATED;
        stopServosImmediate(true);
        return;
    }

    // Clear transient input/stale faults on valid calibrated update
    if (last_fault == ActuatorFaultCode::NON_FINITE_INPUT ||
        last_fault == ActuatorFaultCode::STALE_COMMAND ||
        last_fault == ActuatorFaultCode::LOOP_STALL_ISR_CUTOFF ||
        last_fault == ActuatorFaultCode::PROHIBITED_FLIGHT_STATE) {
        last_fault = ActuatorFaultCode::NONE;
    }

    steering_enabled = true;
    servos_attached  = true;

    computeChannelCommand(left_state, calibration.left, now_ms, left_step_start_ms);
    computeChannelCommand(right_state, calibration.right, now_ms, right_step_start_ms);
    writeServoOutputs();
#endif
}

void ActuatorManager::applySteering(float steering, bool enabled) {
    uint32_t now_ms = 0;
#if defined(ARDUINO) || defined(__arm__) || defined(TEENSYDUINO)
    now_ms = millis();
#else
    now_ms = last_update_ms + (1000UL / CONTROL_LOOP_HZ);
#endif
    FlightState assumed_state = enabled ? FlightState::PRECISION_GUIDANCE : FlightState::LAUNCH_PAD;
    applySteering(steering, enabled, now_ms, assumed_state);
}

void ActuatorManager::update(uint32_t now_ms) {
#if SERVO_USE_STANDARD_POSITIONAL
    // Standard positional servos hold angle continuously via Hardware FlexPWM
    last_update_ms = now_ms;
    return;
#else
    if (!steering_enabled || !calibration_valid) {
        last_update_ms = now_ms;
        return;
    }

    uint32_t dt_ms = elapsedMsWrapSafe(now_ms, last_update_ms);
    last_update_ms = now_ms;

    if (dt_ms > 0U) {
        advanceChannelEstimate(left_state, calibration.left, dt_ms);
        advanceChannelEstimate(right_state, calibration.right, dt_ms);
    }

    // Stale command watchdog check
    if (has_command_timestamp && elapsedMsWrapSafe(now_ms, last_command_ms) > calibration.command_timeout_ms) {
        last_fault = ActuatorFaultCode::STALE_COMMAND;
        stopServosImmediate(false); // Command calibrated neutral stop pulse
        return;
    }

    computeChannelCommand(left_state, calibration.left, now_ms, left_step_start_ms);
    computeChannelCommand(right_state, calibration.right, now_ms, right_step_start_ms);
    writeServoOutputs();
#endif
}

void ActuatorManager::onHardwareTimerTick(uint32_t isr_now_ms) {
#if SERVO_USE_STANDARD_POSITIONAL
    (void)isr_now_ms;
    return;
#else
    if (!steering_enabled || !calibration_valid) return;

    // 1. Check if the main control loop has stalled past command_timeout_ms
    if (has_command_timestamp && elapsedMsWrapSafe(isr_now_ms, last_command_ms) > calibration.command_timeout_ms) {
        uint32_t dt_ms = elapsedMsWrapSafe(isr_now_ms, last_update_ms);
        if (dt_ms > 0U) {
            advanceChannelEstimate(left_state, calibration.left, dt_ms);
            advanceChannelEstimate(right_state, calibration.right, dt_ms);
            last_update_ms = isr_now_ms;
        }
        last_fault = ActuatorFaultCode::LOOP_STALL_ISR_CUTOFF;
        stopServosImmediate(false);
        return;
    }

    // 2. Check if either active channel has reached its planned step duration or max burst cap
    uint32_t dt_since_update = elapsedMsWrapSafe(isr_now_ms, last_update_ms);
    bool left_needs_cutoff = (left_state.direction != SpoolDirection::STOP) &&
        ((left_state.planned_burst_ms > 0U && elapsedMsWrapSafe(isr_now_ms, left_step_start_ms) >= left_state.planned_burst_ms) ||
         (left_state.current_burst_ms + dt_since_update >= calibration.max_burst_duration_ms));
    bool right_needs_cutoff = (right_state.direction != SpoolDirection::STOP) &&
        ((right_state.planned_burst_ms > 0U && elapsedMsWrapSafe(isr_now_ms, right_step_start_ms) >= right_state.planned_burst_ms) ||
         (right_state.current_burst_ms + dt_since_update >= calibration.max_burst_duration_ms));

    if (left_needs_cutoff || right_needs_cutoff) {
        if (dt_since_update > 0U) {
            advanceChannelEstimate(left_state, calibration.left, dt_since_update);
            advanceChannelEstimate(right_state, calibration.right, dt_since_update);
            last_update_ms = isr_now_ms;
        }
        if (left_needs_cutoff && left_state.direction != SpoolDirection::STOP) {
            left_state.direction            = SpoolDirection::STOP;
            left_state.commanded_speed_norm = 0.0f;
            left_state.commanded_pulse_us   = calibration.left.neutral_pulse_us;
            left_state.planned_burst_ms     = 0;
        }
        if (right_needs_cutoff && right_state.direction != SpoolDirection::STOP) {
            right_state.direction            = SpoolDirection::STOP;
            right_state.commanded_speed_norm = 0.0f;
            right_state.commanded_pulse_us   = calibration.right.neutral_pulse_us;
            right_state.planned_burst_ms     = 0;
        }
        writeServoOutputs();
    }
#endif
}

size_t ActuatorManager::formatTelemetryKeyValue(char* out_buf, size_t buf_size) const {
    if (out_buf == nullptr || buf_size == 0) return 0;

    char left_pull_str[16];
    char right_pull_str[16];
    if (calibration_valid && isfinite(left_state.estimated_line_pull_mm)) {
        snprintf(left_pull_str, sizeof(left_pull_str), "%.1f", (double)left_state.estimated_line_pull_mm);
    } else {
        snprintf(left_pull_str, sizeof(left_pull_str), "UNCAL");
    }
    if (calibration_valid && isfinite(right_state.estimated_line_pull_mm)) {
        snprintf(right_pull_str, sizeof(right_pull_str), "%.1f", (double)right_state.estimated_line_pull_mm);
    } else {
        snprintf(right_pull_str, sizeof(right_pull_str), "UNCAL");
    }

    int written = snprintf(
        out_buf,
        buf_size,
        "LEFT_BRAKE_CMD=%.2f,RIGHT_BRAKE_CMD=%.2f,LEFT_SPOOL_DIR=%s,RIGHT_SPOOL_DIR=%s,"
        "LEFT_SERVO_US=%d,RIGHT_SERVO_US=%d,LEFT_EST_PULL_MM=%s,RIGHT_EST_PULL_MM=%s,"
        "ACTUATOR_CAL=%s,ACTUATOR_FAULT=%s",
        (double)left_state.target_brake_norm,
        (double)right_state.target_brake_norm,
        getSpoolDirectionName(left_state.direction),
        getSpoolDirectionName(right_state.direction),
        (int)left_state.commanded_pulse_us,
        (int)right_state.commanded_pulse_us,
        left_pull_str,
        right_pull_str,
        calibration_valid ? "CALIBRATED" : "UNCAL_LOCKED",
        getActuatorFaultName(last_fault)
    );
    return (written > 0 && (size_t)written < buf_size) ? (size_t)written : 0;
}

size_t ActuatorManager::formatTelemetryCsvColumns(char* out_buf, size_t buf_size) const {
    if (out_buf == nullptr || buf_size == 0) return 0;

    char left_pull_str[16];
    char right_pull_str[16];
    if (calibration_valid && isfinite(left_state.estimated_line_pull_mm)) {
        snprintf(left_pull_str, sizeof(left_pull_str), "%.1f", (double)left_state.estimated_line_pull_mm);
    } else {
        snprintf(left_pull_str, sizeof(left_pull_str), "UNCAL");
    }
    if (calibration_valid && isfinite(right_state.estimated_line_pull_mm)) {
        snprintf(right_pull_str, sizeof(right_pull_str), "%.1f", (double)right_state.estimated_line_pull_mm);
    } else {
        snprintf(right_pull_str, sizeof(right_pull_str), "UNCAL");
    }

    int written = snprintf(
        out_buf,
        buf_size,
        "%.2f,%.2f,%s,%s,%d,%d,%s,%s,%s,%s",
        (double)left_state.target_brake_norm,
        (double)right_state.target_brake_norm,
        getSpoolDirectionName(left_state.direction),
        getSpoolDirectionName(right_state.direction),
        (int)left_state.commanded_pulse_us,
        (int)right_state.commanded_pulse_us,
        left_pull_str,
        right_pull_str,
        calibration_valid ? "CALIBRATED" : "UNCAL_LOCKED",
        getActuatorFaultName(last_fault)
    );
    return (written > 0 && (size_t)written < buf_size) ? (size_t)written : 0;
}

void ActuatorManager::setBurnwire(bool active) {
    burnwire_state = active;
#if defined(ARDUINO) || defined(__arm__) || defined(TEENSYDUINO)
    digitalWrite(BURNWIRE_PIN, active ? HIGH : LOW);
#endif
}

void ActuatorManager::setFlywheel(bool active) {
    flywheel_state = active;
    if (active) {
        current_flywheel_rpm = FLYWHEEL_TARGET_RPM;
        current_flywheel_pulse_us = ESC_TARGET_6000RPM_US;
    } else {
        current_flywheel_rpm = 0.0f;
        current_flywheel_pulse_us = ESC_MIN_PULSE_US;
    }
#if defined(ARDUINO) || defined(__arm__) || defined(TEENSYDUINO)
    if (!active) {
        hwFlywheelPulseUs = ESC_MIN_PULSE_US;
        writeHardwarePwmUs(FLYWHEEL_ESC_PIN, ESC_MIN_PULSE_US);
    } else {
        uint32_t now = millis();
        if (hwFlywheelPulseUs < 1060) {
            hwFlywheelPulseUs = 1060;
            lastFlywheelRampMs = now;
        } else if (elapsedMsWrapSafe(now, lastFlywheelRampMs) >= 80U) {
            lastFlywheelRampMs = now;
            hwFlywheelPulseUs += 8;
            if (hwFlywheelPulseUs > ESC_TARGET_6000RPM_US) {
                hwFlywheelPulseUs = ESC_TARGET_6000RPM_US;
            }
        }
        writeHardwarePwmUs(FLYWHEEL_ESC_PIN, (uint16_t)hwFlywheelPulseUs);
    }
#endif
}

void ActuatorManager::updateBuzzer(bool active, uint32_t now_ms) {
    if (!active) {
#if defined(ARDUINO) || defined(__arm__) || defined(TEENSYDUINO)
        digitalWrite(BUZZER_PIN, LOW);
#endif
        buzzer_state = false;
        return;
    }

    if (elapsedMsWrapSafe(now_ms, last_buzzer_toggle_ms) >= 500U) {
        last_buzzer_toggle_ms = now_ms;
        buzzer_state = !buzzer_state;
#if defined(ARDUINO) || defined(__arm__) || defined(TEENSYDUINO)
        digitalWrite(BUZZER_PIN, buzzer_state ? HIGH : LOW);
#endif
    }
}
