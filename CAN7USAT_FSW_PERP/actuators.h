#pragma once

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "config.h"
#include "nav_types.h"

// ============================================================================
// Continuous-Rotation Servo & M4 Screw Brake-Line Spool Actuator Controller
// ============================================================================
// Physical Mechanism:
//   - Two continuous-rotation servos (Left on Pin 2, Right on Pin 3).
//   - Each brake line winds directly around an M4 screw attached to its servo.
//   - Rotating in WIND_BRAKE winds the line onto the M4 screw (applying brake).
//   - Rotating in RELEASE_BRAKE unwinds the line from the M4 screw (releasing
//     brake, requiring aerodynamic canopy line tension).
//   - NO encoders, potentiometers, or limit switches exist on the mechanism.
//   - Actual displacement is ALWAYS unknown in flight (actual_position_known = false).
//   - Returning estimated_line_pull_mm to 0.0 after open-loop motion does NOT
//     guarantee physical neutral restoration (neutral_position_verified = false).
// ============================================================================

enum class SpoolDirection : uint8_t {
    STOP          = 0, // Calibrated neutral stop pulse (or 0 us when detached)
    WIND_BRAKE    = 1, // Shortens brake line by winding onto M4 screw
    RELEASE_BRAKE = 2  // Lengthens brake line by unwinding from M4 screw
};

enum class ActuatorFaultCode : uint8_t {
    NONE                    = 0,
    UNCALIBRATED            = 1, // Default safe lockout until bench calibration
    INVALID_CALIBRATION     = 2, // Calibration bounds/polarity/kinematics invalid
    NON_FINITE_INPUT        = 3, // NaN or Inf steering input rejected
    STALE_COMMAND           = 4, // Guidance command watchdog timeout exceeded
    MAX_BURST_EXCEEDED      = 5, // Single continuous rotation burst hit time cap
    TRAVEL_LIMIT_REACHED    = 6, // Open-loop estimated pull reached max_pull_mm
    OPEN_LOOP_DRIFT_LIMIT   = 7, // Cumulative open-loop travel uncertainty exceeded safe limit
    PROHIBITED_FLIGHT_STATE = 8, // Steering requested in non-guidance flight state
    LOOP_STALL_ISR_CUTOFF   = 9  // Hardware timer ISR stopped motors during main-loop stall
};

inline const char* getSpoolDirectionName(SpoolDirection dir) {
    switch (dir) {
        case SpoolDirection::STOP:          return "STOP";
        case SpoolDirection::WIND_BRAKE:    return "WIND";
        case SpoolDirection::RELEASE_BRAKE: return "RELEASE";
        default:                            return "UNKNOWN";
    }
}

inline const char* getActuatorFaultName(ActuatorFaultCode fault) {
    switch (fault) {
        case ActuatorFaultCode::NONE:                    return "NONE";
        case ActuatorFaultCode::UNCALIBRATED:            return "UNCALIBRATED";
        case ActuatorFaultCode::INVALID_CALIBRATION:     return "INVALID_CALIBRATION";
        case ActuatorFaultCode::NON_FINITE_INPUT:        return "NON_FINITE_INPUT";
        case ActuatorFaultCode::STALE_COMMAND:           return "STALE_COMMAND";
        case ActuatorFaultCode::MAX_BURST_EXCEEDED:      return "MAX_BURST_EXCEEDED";
        case ActuatorFaultCode::TRAVEL_LIMIT_REACHED:    return "TRAVEL_LIMIT_REACHED";
        case ActuatorFaultCode::OPEN_LOOP_DRIFT_LIMIT:   return "OPEN_LOOP_DRIFT_LIMIT";
        case ActuatorFaultCode::PROHIBITED_FLIGHT_STATE: return "PROHIBITED_FLIGHT_STATE";
        case ActuatorFaultCode::LOOP_STALL_ISR_CUTOFF:   return "LOOP_STALL_ISR_CUTOFF";
        default:                                         return "UNKNOWN";
    }
}

// ----------------------------------------------------------------------------
// Normalized Differential Brake Request (Output of Steering Split)
// ----------------------------------------------------------------------------
struct BrakeCommand {
    float left;   // [0.0, 1.0]: 0.0 = no brake, 1.0 = full brake effort
    float right;  // [0.0, 1.0]: 0.0 = no brake, 1.0 = full brake effort
};

// ----------------------------------------------------------------------------
// Per-Channel Continuous-Rotation Servo & M4 Screw Calibration
// ----------------------------------------------------------------------------
struct ContinuousServoChannelCal {
    uint16_t neutral_pulse_us;      // Bench-measured stop pulse width (us)
    uint16_t neutral_deadband_us;   // Bench-measured stop deadband half-width (us)
    uint16_t min_pulse_us;          // Verified electrical lower pulse bound (us)
    uint16_t max_pulse_us;          // Verified electrical upper pulse bound (us)
    uint16_t max_wind_delta_us;     // Max pulse offset from neutral when winding (us)
    uint16_t max_release_delta_us;  // Max pulse offset from neutral when releasing (us)
    int8_t   wind_polarity;         // +1 if pulse > neutral winds line; -1 if pulse < neutral winds line
    float    rev_per_sec_at_max;    // Measured loaded rotational rate at max delta (rev/s)
    float    mm_per_revolution;     // Measured effective brake-line travel per M4 screw rev (mm/rev)
    float    max_pull_mm;           // Maximum safe brake-line pull for 1.0 brake (mm)
};

struct ContinuousServoSystemCal {
    bool                      calibrated;              // Must be true AND pass validation to enable motion
    ContinuousServoChannelCal left;
    ContinuousServoChannelCal right;
    float                     position_deadband_mm;    // Open-loop travel deadband inside which motor stops (mm)
    float                     drift_uncertainty_ratio; // Uncertainty growth per mm of open-loop travel (e.g. 0.15)
    float                     max_uncertainty_mm;      // Max accumulated open-loop drift uncertainty before hold (mm)
    uint32_t                  max_burst_duration_ms;   // Hard cap on any single continuous rotation burst (ms)
    uint32_t                  min_cooldown_ms;         // Mandatory pause after hitting burst cap (ms)
    uint32_t                  command_timeout_ms;      // Stale guidance command timeout (ms)
};

// ----------------------------------------------------------------------------
// Per-Channel Runtime Actuator State (Separates Commanded, Estimated & Known)
// ----------------------------------------------------------------------------
struct ContinuousServoChannelState {
    float          target_brake_norm;         // Requested brake fraction [0.0, 1.0]
    SpoolDirection direction;                 // STOP, WIND_BRAKE, or RELEASE_BRAKE
    float          commanded_speed_norm;      // Normalized motor speed command [0.0, 1.0]
    uint16_t       commanded_pulse_us;        // Active PWM pulse width (us); 0 when detached
    uint32_t       planned_burst_ms;          // Planned duration for current step to reach target_pull_mm (ms)
    uint32_t       current_burst_ms;          // Continuous active rotation duration in current burst (ms)
    uint32_t       cooldown_remaining_ms;     // Remaining cooldown time after burst cap (ms)
    uint32_t       cumulative_wind_ms;        // Cumulative time spent winding (ms)
    uint32_t       cumulative_release_ms;     // Cumulative time spent releasing (ms)
    float          cumulative_travel_mm;      // Total open-loop line movement (wind + release) (mm)
    float          estimated_line_pull_mm;    // Open-loop dead-reckoned pull (mm); NAN if uncalibrated
    float          estimated_uncertainty_mm;  // Open-loop dead-reckoned uncertainty (mm); NAN if uncalibrated
    bool           actual_position_known;     // ALWAYS false without physical position sensor
    bool           neutral_position_verified; // False once any open-loop motion occurs without home sensor
    bool           burst_limit_latched;       // True while paused due to max_burst_duration_ms
    bool           drift_limit_latched;       // True if estimated_uncertainty_mm >= max_uncertainty_mm
    SpoolDirection latched_burst_direction;   // Direction that triggered burst cap
};

// Optional hardware PWM sink callback for deterministic host testing
typedef void (*ServoPwmWriteCallback)(uint8_t pin, uint16_t pulse_us);

// ============================================================================
// Actuator Manager Class
// ============================================================================
class ActuatorManager {
public:
    ActuatorManager();

    void init();
    void setPwmWriteCallback(ServoPwmWriteCallback cb) { pwm_write_cb = cb; }

    // Calibration management & validation
    static ContinuousServoSystemCal getDefaultConfigCalibration();
    static bool validateChannelCalibration(const ContinuousServoChannelCal& ch);
    static bool validateSystemCalibration(const ContinuousServoSystemCal& cal);
    bool configureCalibration(const ContinuousServoSystemCal& cal);
#if SERVO_USE_STANDARD_POSITIONAL
    bool isCalibrated() const { return true; }
#else
    bool isCalibrated() const { return calibration_valid; }
#endif
    const ContinuousServoSystemCal& getCalibration() const { return calibration; }

    // Flight-state steering permission gate
    static bool isFlightStateSteeringPermitted(FlightState state);

    // Primary steering & time-stepped spool actuation interface
    void applySteering(
        float steering,
        bool enabled,
        uint32_t now_ms,
        FlightState flight_state = FlightState::PRECISION_GUIDANCE
    );

    // Backward-compatible 2-argument overload
    void applySteering(float steering, bool enabled);

    // Periodic watchdog & open-loop spool travel integration update (called in loop())
    void update(uint32_t now_ms);

    // Hardware timer ISR safety handler (runs independently of loop() via IntervalTimer on Teensy 4.1)
    void onHardwareTimerTick(uint32_t isr_now_ms);

    // Emergency / immediate stop of both brake-line servos
    void stopServosImmediate(bool detach_pwm);

    // Reset open-loop line pull estimate on the ground after manual line re-tensioning
    void resetEstimatedLinePosition();

    // Format continuous-rotation actuator telemetry fields (KEY=VALUE) into buffer
    size_t formatTelemetryKeyValue(char* out_buf, size_t buf_size) const;

    // Format continuous-rotation actuator CSV columns into buffer
    size_t formatTelemetryCsvColumns(char* out_buf, size_t buf_size) const;

    // Auxiliary actuators (preserved intact)
    void setBurnwire(bool active);
    void updateBuzzer(bool active, uint32_t now_ms);
    void setFlywheel(bool active);

    // Continuous-rotation servo state accessors
    float getRequestedSteeringEffort() const { return requested_steering_effort; }
    float getLeftBrake() const { return left_state.target_brake_norm; }
    float getRightBrake() const { return right_state.target_brake_norm; }
    SpoolDirection getLeftSpoolDirection() const { return left_state.direction; }
    SpoolDirection getRightSpoolDirection() const { return right_state.direction; }
    float getLeftSpeedNorm() const { return left_state.commanded_speed_norm; }
    float getRightSpeedNorm() const { return right_state.commanded_speed_norm; }
    int getLeftServoPulseUs() const { return (int)left_state.commanded_pulse_us; }
    int getRightServoPulseUs() const { return (int)right_state.commanded_pulse_us; }
    uint32_t getLeftBurstDurationMs() const { return left_state.current_burst_ms; }
    uint32_t getRightBurstDurationMs() const { return right_state.current_burst_ms; }
    uint32_t getLeftPlannedBurstMs() const { return left_state.planned_burst_ms; }
    uint32_t getRightPlannedBurstMs() const { return right_state.planned_burst_ms; }
    float getLeftEstimatedPullMm() const { return left_state.estimated_line_pull_mm; }
    float getRightEstimatedPullMm() const { return right_state.estimated_line_pull_mm; }
    float getLeftEstimatedUncertaintyMm() const { return left_state.estimated_uncertainty_mm; }
    float getRightEstimatedUncertaintyMm() const { return right_state.estimated_uncertainty_mm; }
    bool isActualPositionKnown() const { return false; }
    bool isLeftNeutralVerified() const { return left_state.neutral_position_verified; }
    bool isRightNeutralVerified() const { return right_state.neutral_position_verified; }
    bool areServosAttached() const { return servos_attached; }
    ActuatorFaultCode getFaultCode() const { return last_fault; }
    const ContinuousServoChannelState& getLeftChannelState() const { return left_state; }
    const ContinuousServoChannelState& getRightChannelState() const { return right_state; }

    // Direct raw PWM pulse control (for bench test, full unwind, manual positioning)
    void setDirectPulse(uint16_t left_us, uint16_t right_us);

    // Backward-compatibility angle accessors (90 deg = neutral / 0 brake, 0 deg = full brake)
    float getLeftServoAngle() const { return (1.0f - left_state.target_brake_norm) * 90.0f; }
    float getRightServoAngle() const { return (1.0f - right_state.target_brake_norm) * 90.0f; }

    // Auxiliary actuator getters (preserved intact)
    bool isBurnwirePinHigh() const { return burnwire_state; }
    bool isBuzzerPinHigh() const { return buzzer_state; }
    bool isFlywheelActive() const { return flywheel_state; }
    float getFlywheelRPM() const { return current_flywheel_rpm; }
    int getFlywheelPulseUs() const { return current_flywheel_pulse_us; }

private:
    ContinuousServoSystemCal    calibration;
    bool                        calibration_valid;
    ContinuousServoChannelState left_state;
    ContinuousServoChannelState right_state;
    float                       requested_steering_effort;
    bool                        steering_enabled;
    bool                        servos_attached;
    bool                        has_command_timestamp;
    uint32_t                    last_command_ms;
    uint32_t                    last_update_ms;
    uint32_t                    left_step_start_ms;
    uint32_t                    right_step_start_ms;
    ActuatorFaultCode           last_fault;
    ServoPwmWriteCallback       pwm_write_cb;

    bool     burnwire_state;
    bool     buzzer_state;
    bool     flywheel_state;
    float    current_flywheel_rpm;
    int      current_flywheel_pulse_us;
    uint32_t last_buzzer_toggle_ms;

    void initChannelState(ContinuousServoChannelState& ch, bool is_cal_valid);
    void advanceChannelEstimate(
        ContinuousServoChannelState& ch,
        const ContinuousServoChannelCal& cal,
        uint32_t dt_ms
    );
    void computeChannelCommand(
        ContinuousServoChannelState& ch,
        const ContinuousServoChannelCal& cal,
        uint32_t now_ms,
        uint32_t& step_start_ms
    );
    void writeServoOutputs();
};
