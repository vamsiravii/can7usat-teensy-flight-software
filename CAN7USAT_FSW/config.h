#pragma once

// ============================================================================
// CAN-7USAT Team 017 Flight Software Configuration Header
// Team ID: 2026-INSPACe-CAN-7USAT-017 (Team Parikshit, Manipal)
// ============================================================================

#define TEAM_ID                   "2026-IN-SPACe-CAN-7USAT-017"

// ----------------------------------------------------------------------------
// Pin Assignments (Teensy 4.1)
// ----------------------------------------------------------------------------
#define LORA_SERIAL               Serial1   // Pin 0 (RX1), Pin 1 (TX1)
#define LORA_BAUD                 115200

#define BURNWIRE_PIN              4         // XY-MOS MOSFET #1 trigger pin for Nichrome burnwire disreefing (CDR 4.6)
#define SERVO_LEFT_PIN            3         // Left parafoil steering brake servo (Emax ES3004) - Interchanged
#define SERVO_RIGHT_PIN           2         // Right parafoil steering brake servo (Emax ES3004) - Interchanged
#define BUZZER_PIN                5         // XY-MOS MOSFET #2 trigger pin for recovery audio beacon (CDR 5.10)
#define FLYWHEEL_ESC_PIN          6         // Readytosky MT2204 2300KV BLDC Flywheel ESC PWM signal pin (CDR 4.9)
#define VBAT_PIN                  A0        // Battery voltage divider input (Teensy Pin 14)
#define VBAT_ENABLED              1         // 150 kOhm (top) + 47 kOhm (bottom) voltage divider enabled
#define VBAT_R_TOP                150000.0f // 150 kOhm upper resistor (Battery+ -> A0)
#define VBAT_R_BOTTOM             47000.0f  // 47 kOhm lower resistor (A0 -> GND)
#define VDIV_RATIO                ((VBAT_R_TOP + VBAT_R_BOTTOM) / VBAT_R_BOTTOM) // 197 / 47 = 4.1914894f
#define VBAT_ADC_REF_V            3.3f      // Teensy 4.1 3.3V ADC reference
#define VBAT_ADC_MAX_COUNTS       4095.0f   // 12-bit ADC max count
#define VBAT_AVG_SAMPLES          20        // 20-sample average to filter ESC/switching noise

// ----------------------------------------------------------------------------
// Continuous-Rotation Servo & M4 Screw Brake-Line Spool Configuration
// ----------------------------------------------------------------------------
// CRITICAL SAFETY LOCKOUT:
// The parafoil steering mechanism uses two continuous-rotation servos winding
// left and right brake lines directly around M4 screws WITHOUT encoders,
// potentiometers, or limit switches.
//
// Because physical servo neutral pulse widths, winding directions, loaded
// rotational speeds, effective line travel per M4 screw revolution, and
// maximum safe brake-line pull have NOT yet been measured on the bench,
// ACTUATOR_CALIBRATED MUST remain 0 (false) by default.
// While 0, ActuatorManager keeps both servo channels detached (0 us) and
// prevents all motor rotation.
#ifndef ACTUATOR_CALIBRATED
#define ACTUATOR_CALIBRATED                   1         // 0 = UNCALIBRATED LOCKOUT (default), 1 = Bench-calibrated
#endif

// Per-channel electrical pulse calibration (microseconds)
// Bench-calibrated nominal baseline for ES3004 / Continuous Spool Servos:
#define SERVO_LEFT_NEUTRAL_US                 1500      // Measured stop pulse (us)
#define SERVO_RIGHT_NEUTRAL_US                1500      // Measured stop pulse (us)
#define SERVO_LEFT_DEADBAND_US                20        // Measured stop deadband half-width (us)
#define SERVO_RIGHT_DEADBAND_US               20        // Measured stop deadband half-width (us)
#define SERVO_LEFT_MIN_PULSE_US               1050      // Verified safe lower pulse bound (us)
#define SERVO_LEFT_MAX_PULSE_US               1950      // Verified safe upper pulse bound (us)
#define SERVO_RIGHT_MIN_PULSE_US              1050      // Verified safe lower pulse bound (us)
#define SERVO_RIGHT_MAX_PULSE_US              1950      // Verified safe upper pulse bound (us)
#define SERVO_LEFT_MAX_WIND_DELTA_US          350       // Max offset from neutral when winding brake line (us)
#define SERVO_LEFT_MAX_RELEASE_DELTA_US       300       // Max offset from neutral when releasing brake line (us)
#define SERVO_RIGHT_MAX_WIND_DELTA_US         350       // Max offset from neutral when winding brake line (us)
#define SERVO_RIGHT_MAX_RELEASE_DELTA_US      300       // Max offset from neutral when releasing brake line (us)

// Spool winding direction polarity:
// +1 = pulse > neutral_pulse_us winds (shortens) brake line onto M4 screw
// -1 = pulse < neutral_pulse_us winds (shortens) brake line onto M4 screw
//  0 = unconfigured (motion prohibited)
#define SERVO_LEFT_WIND_POLARITY              -1
#define SERVO_RIGHT_WIND_POLARITY             1

// Bench-measured M4 screw spool kinematics (open-loop dead-reckoning only; NEVER measured position):
#define SERVO_LEFT_REV_PER_SEC_MAX            1.25f     // Measured loaded rev/s at max_wind_delta_us
#define SERVO_RIGHT_REV_PER_SEC_MAX           1.25f     // Measured loaded rev/s at max_wind_delta_us
#define SERVO_LEFT_MM_PER_REV                 16.0f     // Measured line travel per revolution (mm/rev) on M4 screw
#define SERVO_RIGHT_MM_PER_REV                16.0f     // Measured line travel per revolution (mm/rev) on M4 screw
#define SERVO_LEFT_MAX_PULL_MM                60.0f     // Max safe brake-line pull (mm) for 100% brake
#define SERVO_RIGHT_MAX_PULL_MM               60.0f     // Max safe brake-line pull (mm) for 100% brake

// Open-loop deadband, cumulative drift uncertainty budget, and timing guards
#define SERVO_SPOOL_DEADBAND_MM               1.5f      // Open-loop travel deadband (mm) within which motor stops
#define SERVO_DRIFT_UNCERTAINTY_RATIO         0.15f     // Open-loop position uncertainty growth per mm moved (15%)
#define SERVO_MAX_UNCERTAINTY_MM              35.0f     // Max accumulated open-loop drift uncertainty before safety hold
#define SERVO_MAX_BURST_DURATION_MS           400UL     // Hard cap on any single continuous rotation burst (ms)
#define SERVO_MIN_COOLDOWN_MS                 150UL     // Mandatory stop cooldown after burst cap (ms)
#define SERVO_COMMAND_TIMEOUT_MS              300UL     // Stale guidance command timeout (ms) for 10 Hz loop
#define SERVO_SAFETY_ISR_PERIOD_US            10000UL   // 10 ms hardware IntervalTimer ISR period for loop-stall protection

// Legacy angle compatibility defines & Standard Positional Servo Configuration
#ifndef SERVO_USE_STANDARD_POSITIONAL
#define SERVO_USE_STANDARD_POSITIONAL         1         // 1 = Standard 0-180 deg ES3004 Servos; 0 = Continuous Spools
#endif
#define SERVO_NEUTRAL_ANGLE                   90.0f     // 90 deg = No brake applied (1500 us)
#define SERVO_MAX_BRAKE_ANGLE                 0.0f      // 0 deg  = Full brake pulled (500 us)
#define ES3004_MIN_PULSE_US                   500       // Emax ES3004 0 deg pulse width (us)
#define ES3004_NEUTRAL_PULSE_US               1500      // Emax ES3004 90 deg pulse width (us)
#define ES3004_MAX_PULSE_US                   2400      // Emax ES3004 180 deg pulse width (us)
#define BURNWIRE_DURATION_MS      30000     // 5.0s XY-MOS pulse to melt 32 SWG N80 line
#define CO2_INTERVAL_MS           5000UL    // SCD40 CO2 periodic read interval (5.0s)

// Readytosky MT2204 2300KV Brushless Motor + 30A 2S ESC (Z-Axis Momentum Bias Wheel)
// Active continuously from after PARAFOIL_DEPLOYMENT until LANDING at constant 6000 RPM
#define FLYWHEEL_MOTOR_KV         2300.0f   // Readytosky MT2204 speed constant (RPM/V)
#define FLYWHEEL_BATTERY_VOLTS    7.4f      // Nominal 2S Li-ion voltage (Molicel P30B)
#define FLYWHEEL_TARGET_RPM       6000.0f   // Constant 6000 RPM stabilization setpoint
#define ESC_MIN_PULSE_US          1000      // 0% throttle / ESC armed idle (0 RPM)
#define ESC_MAX_PULSE_US          2000      // 100% throttle (17020 RPM @ 7.4V)
#define ESC_TARGET_6000RPM_US     1353      // 1000 + round(1000 * 6000 / (2300 * 7.4)) = 1353 us (35.3% throttle)

// ----------------------------------------------------------------------------
// Mission & State Machine Flight Thresholds
// ----------------------------------------------------------------------------
#define SEA_LEVEL_HPA             1013.25f  // Standard barometric reference
#define BARO_EMA_ALPHA            0.4f      // Validated balance: reduces 0.015 mbar noise with minimal phase lag
#define BARO_CALIB_SAMPLES        50        // 50-sample robust baseline calibration
#define LAUNCH_DETECT_ALT         15.0f     // Meters above ground to enter ASCENT
#define MIN_APOGEE_ALT            750.0f    // Minimum climb altitude (m AGL) before drone release detection arms
#define APOGEE_DROP_THRESH        20.0f     // Meters descent from peak altitude to confirm drone drop (ignores hover bobbing)
#define PARAFOIL_DEPLOY_ALT       700.0f    // Meters AGL to trigger burnwire disreefing (700m deployment)
#define DEPLOYMENT_DURATION_SEC   30.0f      // Parafoil burnwire & inflation duration (5.0s)
#define FINAL_APPROACH_ALT        120.0f    // Meters AGL
#define FINAL_APPROACH_RADIUS     100.0f    // Meters
#define TERMINAL_MARGIN           100.0f    // Meters excess energy margin
#define TERMINAL_CAPTURE_RADIUS   200.0f    // Meters
#define LOITER_RADIUS             30.0f     // Meters holding orbit radius
#define LOITER_CAPTURE_TOL        5.0f      // Meters
#define BREAKOUT_ALT_MARGIN       2.0f      // Meters safety altitude margin
#define TOUCHDOWN_ALT             2.0f      // Meters AGL to declare LANDED
#define TOUCHDOWN_STILL_TIME_MS   4000      // Time baro must be still to declare LANDED

// ----------------------------------------------------------------------------
// Aerodynamic Model (Frozen Baseline - Candidate A)
// ----------------------------------------------------------------------------
#define FORWARD_AIRSPEED          8.0f      // m/s nominal forward airspeed
#define PARAFOIL_DESCENT_RATE     2.5f      // m/s nominal glide sink rate
#define DROGUE_DESCENT_RATE       17.0f     // m/s reefed parachute descent rate (CDR 4.8)
#define MAX_TURN_RATE_DEG_S       30.0f     // deg/s maximum parafoil yaw rate

// ----------------------------------------------------------------------------
// Guidance & Control (PID in Radian Domain)
// ----------------------------------------------------------------------------
#define PID_KP                    10.0f     // Radian-domain tracking authority
#define PID_KI                    0.1f      // s^-1
#define PID_KD                    0.5f      // s
#define PID_INTEGRAL_LIMIT        10.0f     // rad*s anti-windup clamping
#define WIND_EMA_ALPHA            0.05f     // Exponential moving average filter factor

// ----------------------------------------------------------------------------
// Target Location (Default: 1200m East of Launch Pad)
// ----------------------------------------------------------------------------
#define TARGET_DEFAULT_EAST       1200.0f   // +X = East [m]
#define TARGET_DEFAULT_NORTH      0.0f      // +Y = North [m]
#define NUM_WAYPOINTS             10
#define WAYPOINT_ACCEPTANCE_RAD   25.0f     // Meters

// ----------------------------------------------------------------------------
// Loop Frequencies & Data Logging
// ----------------------------------------------------------------------------
#define CONTROL_LOOP_HZ           10        // 10 Hz guidance & EKF update
#define TELEMETRY_HZ              1         // 1 Hz IN-SPACe LoRa telemetry transmission
#ifndef BUILTIN_SDCARD
#define BUILTIN_SDCARD            254       // Teensy 4.1 native SDIO socket constant
#endif
#define SDCARD_CS_PIN             BUILTIN_SDCARD
#define SD_LOG_INTERVAL_MS        1000UL    // 1 Hz CSV data logging (change to 100UL for 10 Hz)

// ----------------------------------------------------------------------------
// BNO055 IMU Calibration Settings & Mounting Angle Offset
// ----------------------------------------------------------------------------
#define BNO_EEPROM_ADDR           0         // Starting address in Teensy EEPROM
#define BNO_EEPROM_MAGIC          0x424E4F31 // 'BNO1' magic verification word
#define BNO_CALIB_TIMEOUT_MS      20000UL   // 20s safety timeout for boot calibration routine

// If the BNO055 is mounted at an angle relative to the vehicle forward flight axis:
// Set this to the exact angle in degrees to calibrate vehicle heading.
// Measured: raw BNO055 reads 159.6 deg when vehicle front faces Geographic North (0 deg).
// Offset = -159.6 deg (equivalent to +200.4 deg wrapped).
#define IMU_HEADING_OFFSET_DEG    -159.6f   // Degrees added to raw BNO055 Euler heading

// Set to true if the BNO055 is mounted upside down (components facing down towards ground)
// Applies Bosch BNO055 P4 axis remap (inverts Z-axis so pitch/roll are 0 deg when upright)
#define BNO_MOUNTED_UPSIDE_DOWN   true
