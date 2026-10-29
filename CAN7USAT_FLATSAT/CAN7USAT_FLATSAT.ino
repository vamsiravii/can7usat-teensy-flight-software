#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>
#include <math.h>

#include <MS5611.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#include "config.h"
#include "nav_types.h"
#include "nav_math.h"
#include "ekf.h"
#include "guidance.h"
#include "flight_sm.h"
#include "actuators.h"

// ============================================================================
// CAN-7USAT Team 017 | FlatSat Interactive State & Sensor Verification FSW
// ----------------------------------------------------------------------------
// Sensors:   1. MS5611 Barometer (0x77 / 0x76)
//            2. NEO-M9N GNSS     (0x42 UBX)
//            3. BNO055 IMU       (0x28 / 0x29)
// Actuators: 1. Burnwire XY-MOS MOSFET (Pin 4) -> Deploys Parafoil @ 700m (5.0s)
//            2. Buzzer   XY-MOS MOSFET (Pin 5) -> Activates Beacon @ Landing
//            3. Left & Right Continuous-Rotation Brake Spools (M4 screw, Pins 2 & 3) -> Guided Descent
// LoRa TX:   Transmits at 1 Hz on Serial1 silently (NO spam on USB Serial!)
// ============================================================================

// Sensors on Wire (SDA 18, SCL 19)
MS5611 baro(0x77, &Wire);
Adafruit_BNO055 bno(55, 0x28, &Wire);
static uint8_t bnoI2CAddr = 0x28;
SFE_UBLOX_GNSS gnss;

bool baroOK = false;
bool imuOK = false;
bool gnssOK = false;

// Navigation & Estimation
ExtendedKalmanFilter ekf;
WindEstimator windEstimator;
PIDController pid(PID_KP, PID_KI, PID_KD, -1.0f, 1.0f, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
WaypointManager waypointManager;
GuidanceController guidanceController(&waypointManager, &pid);
LoiterGuidance loiterGuidance;

// Flight Management & Actuators
FlightStateMachine flightSM;
ActuatorManager actuators;

// FlatSat Control Flags
bool manualStateHold = true;      // True = Hold user-selected state; False = Auto altitude transitions
bool manualSteeringOverride = false;
float overrideSteeringVal = 0.0f;
bool lastBurnwirePinState = false;

// Track runtime sensor activity in the current state
bool baroActiveInState = false;
bool gnssActiveInState = false;
bool imuActiveInState = false;
bool ekfActiveInState = false;
uint8_t currentGnssRateHz = 1;

// Barometer Calibration & Filtering State Variables
float launchPressure = 1013.25f;
float filteredPressure = 1013.25f;
const float EMA_ALPHA = BARO_EMA_ALPHA;
float groundAlt = 0.0f;

// Navigation & Timing State Variables
GPSPoint launchGPS(0.0, 0.0);
bool launchGPSFixed = false;
GPSPoint targetGPS(0.0, 0.0);
bool targetGPSConfigured = false;
LocalPoint targetLocal(TARGET_DEFAULT_EAST, TARGET_DEFAULT_NORTH);

// LoRa Telecommand & Telemetry Gating State
bool telemetryEnabled = false; // Starts automatically on ASCENT or via Ground Station 'START' command
bool telemAscentTriggered = false;
char loraCmdBuf[128];
uint8_t loraCmdLen = 0;
char usbCmdBuf[128];
uint8_t usbCmdLen = 0;

uint32_t packetCount = 0;
uint32_t missionStartMs = 0;
uint32_t lastControlUpdateMs = 0;
uint32_t lastTelemetryMs = 0;

// MicroSD Card Data Logging (Arms automatically as soon as ASCENT starts)
bool sdOK = false;
bool sdLoggingActive = false;
File csvFile;
char csvFileName[24] = "flight_001.csv";
uint32_t sdRecordCount = 0;
uint32_t lastSdLogMs = 0;
bool sdAscentTriggered = false;

float currentSteering = 0.0f;
float lastBaroAlt = 0.0f;
float lastPressurePa = 0.0f;
float lastTempC = 0.0f;

float ax = 0.0f, ay = 0.0f, az = 0.0f;
float heading_deg = 0.0f, roll = 0.0f, pitch = 0.0f, spin = 0.0f;
float activeHeadingOffsetDeg = IMU_HEADING_OFFSET_DEG;
char gnssTimeStr[16] = "UNAVAILABLE";
double gnssLat = 0.0, gnssLon = 0.0;
float gnssAltMSL = 0.0f;
int gnssSats = 0;

// ============================================================================
// Helper Functions
// ============================================================================

float pressureToAlt(float hPa, float referenceHpa) {
    if (!isfinite(hPa) || hPa <= 0.0f || !isfinite(referenceHpa) || referenceHpa <= 0.0f) {
        return 0.0f;
    }
    return 44330.0f * (1.0f - powf(hPa / referenceHpa, 0.190295f));
}

// Read battery voltage via 150k/47k voltage divider on A0 (12-bit ADC, 20-sample average)
float readBatteryVoltage(float* outPinVoltage = nullptr) {
#if VBAT_ENABLED
    long sum = 0;
    for (int i = 0; i < VBAT_AVG_SAMPLES; i++) {
        sum += analogRead(VBAT_PIN);
    }
    float adcCounts = (float)sum / (float)VBAT_AVG_SAMPLES;
    float pinVoltage = adcCounts * (VBAT_ADC_REF_V / VBAT_ADC_MAX_COUNTS);
    if (outPinVoltage != nullptr) {
        *outPinVoltage = pinVoltage;
    }
    return pinVoltage * VDIV_RATIO;
#else
    if (outPinVoltage != nullptr) *outPinVoltage = 0.0f;
    return 0.0f;
#endif
}

// ============================================================================
// BNO055 IMU Calibration & EEPROM Persistence Subsystem
// ============================================================================
struct BNO055_EEPROM_Data {
    uint32_t magic;
    adafruit_bno055_offsets_t offsets;
    uint8_t sys;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;
    uint32_t timestamp;
};

bool loadBNO055CalibFromEEPROM(adafruit_bno055_offsets_t& offsetsOut) {
    BNO055_EEPROM_Data data;
    EEPROM.get(BNO_EEPROM_ADDR, data);
    if (data.magic == BNO_EEPROM_MAGIC) {
        bool magAllZero = (data.offsets.mag_offset_x == 0 &&
                           data.offsets.mag_offset_y == 0 &&
                           data.offsets.mag_offset_z == 0);
        if (data.mag < 3 || magAllZero) {
            Serial.print("[BNO055] WARNING: Stored EEPROM profile does not have Mag 3/3 (found Mag=");
            Serial.print(data.mag);
            Serial.println("/3). Clearing profile to calibrate full Mag 3/3.");
            clearBNO055EEPROM();
            return false;
        }
        offsetsOut = data.offsets;
        return true;
    }
    return false;
}

bool readBNO055OffsetsDirect(adafruit_bno055_offsets_t &offsets) {
    // Read raw 22 offset registers directly from BNO055 hardware in CONFIG mode
    // (Bypasses Adafruit's strict isFullyCalibrated() check so Mag+Gyro offsets save reliably!)
    bno.setMode(OPERATION_MODE_CONFIG);
    delay(25);

    Wire.beginTransmission(bnoI2CAddr);
    Wire.write(0x55); // ACCEL_OFFSET_X_LSB_ADDR
    if (Wire.endTransmission() != 0) {
        bno.setMode(OPERATION_MODE_NDOF);
        delay(20);
        return false;
    }

    uint8_t bytesRead = Wire.requestFrom(bnoI2CAddr, (uint8_t)22);
    if (bytesRead < 22) {
        bno.setMode(OPERATION_MODE_NDOF);
        delay(20);
        return false;
    }

    uint8_t raw[22];
    for (int i = 0; i < 22; i++) {
        raw[i] = Wire.read();
    }

    offsets.accel_offset_x = (int16_t)((raw[1] << 8) | raw[0]);
    offsets.accel_offset_y = (int16_t)((raw[3] << 8) | raw[2]);
    offsets.accel_offset_z = (int16_t)((raw[5] << 8) | raw[4]);

    offsets.mag_offset_x   = (int16_t)((raw[7] << 8) | raw[6]);
    offsets.mag_offset_y   = (int16_t)((raw[9] << 8) | raw[8]);
    offsets.mag_offset_z   = (int16_t)((raw[11] << 8) | raw[10]);

    offsets.gyro_offset_x  = (int16_t)((raw[13] << 8) | raw[12]);
    offsets.gyro_offset_y  = (int16_t)((raw[15] << 8) | raw[14]);
    offsets.gyro_offset_z  = (int16_t)((raw[17] << 8) | raw[16]);

    offsets.accel_radius   = (int16_t)((raw[19] << 8) | raw[18]);
    offsets.mag_radius     = (int16_t)((raw[21] << 8) | raw[20]);

    bno.setMode(OPERATION_MODE_NDOF);
    delay(20);
    return true;
}

void saveBNO055CalibToEEPROM(const adafruit_bno055_offsets_t& offsets, uint8_t sys, uint8_t gyro, uint8_t accel, uint8_t mag) {
    BNO055_EEPROM_Data data;
    data.magic = BNO_EEPROM_MAGIC;
    data.offsets = offsets;
    data.sys = sys;
    data.gyro = gyro;
    data.accel = accel;
    data.mag = mag;
    data.timestamp = millis();
    EEPROM.put(BNO_EEPROM_ADDR, data);
}

void clearBNO055EEPROM() {
    uint32_t zeroMagic = 0x00000000;
    EEPROM.put(BNO_EEPROM_ADDR, zeroMagic);
    Serial.println("[BNO055] EEPROM calibration profile cleared.");
}

void calibrateBNO055OnBoot(uint32_t timeoutMs = BNO_CALIB_TIMEOUT_MS) {
    if (!imuOK) {
        Serial.println("[BNO055] Hardware offline. Skipping IMU calibration.");
        return;
    }

    Serial.println();
    Serial.println("==================================================================");
    Serial.println(">>> BNO055 9-DOF IMU BOOT CALIBRATION & EEPROM RESTORE <<<");
    Serial.println("==================================================================");

    adafruit_bno055_offsets_t savedOffsets;
    bool hasSavedOffsets = loadBNO055CalibFromEEPROM(savedOffsets);

    if (hasSavedOffsets) {
        Serial.println("[BNO055] Found verified Mag 3/3 profile in EEPROM. Loading offsets...");
        bno.setSensorOffsets(savedOffsets);
        delay(100);
        Serial.println("[BNO055] Stored profile applied.");
        Serial.print("[BNO055] Loaded Offsets -> Mag: (X=");
        Serial.print(savedOffsets.mag_offset_x);
        Serial.print(", Y=");
        Serial.print(savedOffsets.mag_offset_y);
        Serial.print(", Z=");
        Serial.print(savedOffsets.mag_offset_z);
        Serial.print(") | Gyro: (X=");
        Serial.print(savedOffsets.gyro_offset_x);
        Serial.print(", Y=");
        Serial.print(savedOffsets.gyro_offset_y);
        Serial.print(", Z=");
        Serial.print(savedOffsets.gyro_offset_z);
        Serial.println(")");
        Serial.println("[BNO055] Profile active. Keeping CanSat still for 2s to zero gyro drift...");
    } else {
        Serial.println("[BNO055] No verified Mag 3/3 profile found. Preparing calibration routine.");
        Serial.println("[BNO055] Step 1: Keep CanSat STILL for 3s (Zeroes Gyroscope -> Gyro: 3/3).");
        Serial.println("[BNO055] Step 2: Rotate CanSat slowly in figure-8 (until Mag: 3/3).");
        Serial.println("[BNO055] >>> WAITING FOR MAG 3/3 BEFORE SAVING PROFILE TO EEPROM... <<<");
        Serial.println("[BNO055] (Send any key via Serial/LoRa to skip if already mounted on launch pad)");
    }

    uint32_t activeTimeoutMs = 3000UL;
    uint32_t startMs = millis();
    uint32_t lastPrintMs = 0;
    uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;
    bool calibComplete = false;

    while (true) {
        bno.getCalibration(&sys, &gyro, &accel, &mag);

        // Check for manual skip from USB Serial or LoRa
        if (Serial.available() > 0 || LORA_SERIAL.available() > 0) {
            Serial.println("[BNO055] User skip signal received. Continuing boot sequence.");
            break;
        }

        // 1. If EEPROM profile was loaded, Gyro=3 (stationary zero-bias) is all we need!
        if (hasSavedOffsets) {
            if (gyro == 3 || (millis() - startMs >= activeTimeoutMs)) {
                calibComplete = true;
                Serial.println("[BNO055] Gyroscope zero-bias established. Verified Mag 3/3 profile ready for flight!");
                break;
            }
        } else {
            // 2. Fresh calibration: MUST wait until Mag reaches 3/3 AND Gyro reaches 3/3!
            if (mag == 3 && gyro == 3) {
                calibComplete = true;
                Serial.println("\n[BNO055] >>> SUCCESS! Magnetometer 3/3 and Gyroscope 3/3 achieved! <<<");
                break;
            }
        }

        if (millis() - lastPrintMs >= 500) {
            lastPrintMs = millis();
            uint32_t elapsedSec = (millis() - startMs) / 1000;

            Serial.print("[IMU CALIB] Status -> Sys: ");
            Serial.print(sys);
            Serial.print("/3 | Gyro: ");
            Serial.print(gyro);
            Serial.print("/3 | Accel: ");
            Serial.print(accel);
            Serial.print("/3 | Mag: ");
            Serial.print(mag);
            if (hasSavedOffsets) {
                uint32_t remainingSec = (activeTimeoutMs > (millis() - startMs)) ? (activeTimeoutMs - (millis() - startMs)) / 1000 : 0;
                Serial.print("/3  [Auto-advances in ");
                Serial.print(remainingSec);
                Serial.println("s]");
            } else {
                Serial.print("/3  [WAITING FOR MAG 3/3... Rotate CanSat in figure-8 | ");
                Serial.print(elapsedSec);
                Serial.println("s elapsed]");
            }

            // Broadcast calibration status to Ground Station
            LORA_SERIAL.print("CALIB:SYS=");
            LORA_SERIAL.print(sys);
            LORA_SERIAL.print(",GYRO=");
            LORA_SERIAL.print(gyro);
            LORA_SERIAL.print(",ACCEL=");
            LORA_SERIAL.print(accel);
            LORA_SERIAL.print(",MAG=");
            LORA_SERIAL.println(mag);
        }

        delay(40);
    }

    bno.getCalibration(&sys, &gyro, &accel, &mag);

    // Save only when fresh calibration achieved Mag 3/3 AND Gyro 3/3!
    if (!hasSavedOffsets && (mag == 3 && gyro == 3)) {
        adafruit_bno055_offsets_t newOffsets;
        if (readBNO055OffsetsDirect(newOffsets) || bno.getSensorOffsets(newOffsets)) {
            saveBNO055CalibToEEPROM(newOffsets, sys, gyro, accel, mag);
            Serial.println("[BNO055] >>> CALIBRATION COMPLETE! Mag 3/3 profile saved to EEPROM. <<<");
            Serial.print("[BNO055] Stored Offsets -> Mag: (X=");
            Serial.print(newOffsets.mag_offset_x);
            Serial.print(", Y=");
            Serial.print(newOffsets.mag_offset_y);
            Serial.print(", Z=");
            Serial.print(newOffsets.mag_offset_z);
            Serial.print(") | Gyro: (X=");
            Serial.print(newOffsets.gyro_offset_x);
            Serial.print(", Y=");
            Serial.print(newOffsets.gyro_offset_y);
            Serial.print(", Z=");
            Serial.print(newOffsets.gyro_offset_z);
            Serial.println(")");
            LORA_SERIAL.println("ACK:CALIB=SAVED_EEPROM");
        } else {
            Serial.println("[BNO055] Warning: Could not read offset registers.");
        }
    } else if (hasSavedOffsets) {
        Serial.println("[BNO055] Retaining verified Mag 3/3 EEPROM calibration profile.");
        LORA_SERIAL.println("ACK:CALIB=RESTORED_EEPROM");
    } else {
        Serial.print("[BNO055] Calibration finished without full Mag 3/3 (Sys: ");
        Serial.print(sys);
        Serial.print(" Gyro: ");
        Serial.print(gyro);
        Serial.print(" Accel: ");
        Serial.print(accel);
        Serial.print(" Mag: ");
        Serial.print(mag);
        Serial.println("/3). Profile NOT saved to EEPROM.");
        LORA_SERIAL.println("ACK:CALIB=INCOMPLETE");
    }
    Serial.println("==================================================================\n");
}

void startSDLogging() {
    if (!sdOK) {
        sdOK = SD.begin(SDCARD_CS_PIN);
    }
    if (!sdOK) {
        Serial.println("[SD] Failed to start logging: MicroSD card not detected or init failed.");
        return;
    }
    if (sdLoggingActive) return;

    for (int i = 1; i <= 999; i++) {
        snprintf(csvFileName, sizeof(csvFileName), "flight_%03d.csv", i);
        if (!SD.exists(csvFileName)) {
            break;
        }
    }

    csvFile = SD.open(csvFileName, FILE_WRITE);
    if (csvFile) {
        sdLoggingActive = true;
        sdRecordCount = 0;
        csvFile.println(
            "TEAM_ID,TIME_STAMPING,PACKET_COUNT,ALTITUDE,PRESSURE,TEMP,VOLTAGE,"
            "GNSS_TIME,GNSS_LATITUDE,GNSS_LONGITUDE,GNSS_ALTITUDE,GNSS_SATS,"
            "ACCELEROMETER_DATA,GYRO_SPIN_RATE,FLIGHT_SOFTWARE_STATE,OPTIONAL_DATA"
        );
        csvFile.flush();
        Serial.print(">>> [SD CARD] Logging STARTED -> Created onboard CSV: ");
        Serial.print(csvFileName);
        Serial.println(" <<<");
    } else {
        Serial.print("[SD] Error: Could not create CSV file: ");
        Serial.println(csvFileName);
    }
}

void stopSDLogging() {
    if (sdLoggingActive && csvFile) {
        csvFile.flush();
        csvFile.close();
        sdLoggingActive = false;
        Serial.print(">>> [SD CARD] Logging STOPPED & Saved -> File: ");
        Serial.print(csvFileName);
        Serial.print(" (");
        Serial.print(sdRecordCount);
        Serial.println(" records) <<<");
    }
}

void logDataToSD(
    uint32_t missionTimeSeconds,
    uint32_t recordNumber,
    float altitudeM,
    float pressurePa,
    float baroTempC,
    float batteryV,
    const char* gpsTime,
    double latitudeDeg,
    double longitudeDeg,
    float gpsAltitudeM,
    int satellites,
    float accelX,
    float accelY,
    float accelZ,
    float gyroZ,
    const char* currentState,
    const char* optionalData = ""
) {
    if (!sdLoggingActive || !csvFile) return;

    float accelTotal = sqrtf(accelX * accelX + accelY * accelY + accelZ * accelZ);

    csvFile.print(TEAM_ID);
    csvFile.print(",");
    csvFile.print(missionTimeSeconds);
    csvFile.print(",");
    csvFile.print(recordNumber);
    csvFile.print(",");
    csvFile.print(altitudeM, 2);
    csvFile.print(",");
    csvFile.print(pressurePa, 2);
    csvFile.print(",");
    csvFile.print(baroTempC, 2);
    csvFile.print(",");
    csvFile.print(batteryV, 2);
    csvFile.print(",");
    csvFile.print(gpsTime);
    csvFile.print(",");
    csvFile.print(latitudeDeg, 7);
    csvFile.print(",");
    csvFile.print(longitudeDeg, 7);
    csvFile.print(",");
    csvFile.print(gpsAltitudeM, 2);
    csvFile.print(",");
    csvFile.print(satellites);
    csvFile.print(",");
    csvFile.print(accelTotal, 2);
    csvFile.print(",");
    csvFile.print(gyroZ, 4);
    csvFile.print(",");
    csvFile.print(currentState);
    csvFile.print(",");
    csvFile.println(optionalData ? optionalData : "");

    csvFile.flush();
}

void updateSensorActivityFlags(FlightState st) {
    switch (st) {
        case FlightState::BOOT:
        case FlightState::LAUNCH_PAD:
            // Launch pad power-saving standby:
            // MS5611 active (10 Hz to detect liftoff > 15m)
            // NEO-M9N active at 1 Hz low-power pad mode (locks pad origin)
            // BNO055 IMU & 10 Hz EKF/Guidance INACTIVE on pad
            baroActiveInState = baroOK;
            gnssActiveInState = gnssOK;
            imuActiveInState  = false;
            ekfActiveInState  = false;
            if (gnssOK && currentGnssRateHz != 1) {
                gnss.setNavigationFrequency(1);
                currentGnssRateHz = 1;
            }
            break;

        case FlightState::ASCENT:
        case FlightState::DROGUE_DESCENT:
        case FlightState::PARAFOIL_DEPLOYMENT:
        case FlightState::PRECISION_GUIDANCE:
        case FlightState::TERMINAL_GUIDANCE:
        case FlightState::FINAL_APPROACH:
            // Full active flight: all sensors active at full GNC rate
            baroActiveInState = baroOK;
            gnssActiveInState = gnssOK;
            imuActiveInState  = imuOK;
            ekfActiveInState  = true;
            if (gnssOK && currentGnssRateHz != 5) {
                gnss.setNavigationFrequency(5);
                currentGnssRateHz = 5;
            }
            break;

        case FlightState::LANDING:
        case FlightState::FAILSAFE:
            // Post-landing recovery mode:
            // BNO055 IMU & EKF/Guidance turned OFF, Servos detached (0 mA)
            // Buzzer XY-MOS MOSFET ACTIVE (pulsing beacon)
            // GNSS & Baro kept at 1 Hz for recovery coordinates in LoRa telemetry
            baroActiveInState = baroOK;
            gnssActiveInState = gnssOK;
            imuActiveInState  = false;
            ekfActiveInState  = false;
            if (gnssOK && currentGnssRateHz != 1) {
                gnss.setNavigationFrequency(1);
                currentGnssRateHz = 1;
            }
            break;
    }
}

void printFlatSatMenu() {
    Serial.println("\n======================================================================");
    Serial.println("  CAN-7USAT Team 017 | FlatSat State & Sensor Verification Console");
    Serial.println("======================================================================");
    Serial.println("  Send a key in Serial Monitor (115200 baud) to switch Flight State:");
    Serial.println("    '1' : LAUNCH_PAD          (Pad Standby: Baro ON, GNSS 1Hz, IMU OFF, Servos OFF)");
    Serial.println("    '2' : ASCENT              (Rocket Climb: All Sensors ON, GNSS 5Hz, Servos OFF)");
    Serial.println("    '3' : DROGUE_DESCENT      (>700m Descent: All Sensors ON, Wind Est ON, Servos OFF)");
    Serial.println("    '4' : PARAFOIL_DEPLOYMENT (700m Trigger: FIRES BURNWIRE XY-MOS Pin 4 for 30.0s!)");
    Serial.println("    '5' : PRECISION_GUIDANCE  (<700m Guided: MT2204 Flywheel 6000 RPM + Continuous Spools ON)");
    Serial.println("    '6' : TERMINAL_GUIDANCE   (30m Loiter Orbit: MT2204 Flywheel 6000 RPM + Continuous Spools ON)");
    Serial.println("    '7' : FINAL_APPROACH      (Landing Approach: MT2204 Flywheel 6000 RPM + Continuous Spools ON)");
    Serial.println("    '8' : LANDING             (Touchdown: BUZZER XY-MOS Pin 5 ON, Flywheel/Spools/IMU OFF)");
    Serial.println("----------------------------------------------------------------------");
    Serial.println("  Inspection & Override Commands:");
    Serial.println("    's' : Print ACTIVE / INACTIVE Sensor & Actuator Status Matrix");
    Serial.println("    'l' : Override Steering -> Full LEFT Brake  (-1.0) in guided states");
    Serial.println("    'n' : Override Steering -> Neutral Glide    ( 0.0) in guided states");
    Serial.println("    'r' : Override Steering -> Full RIGHT Brake (+1.0) in guided states");
    Serial.println("    'a' : Reset Open-Loop Line Pull Estimate to 0.0 mm (after manual re-tension)");
    Serial.println("    'g' : Resume Live EKF + PID Guidance Steering");
    Serial.println("    'm' : Toggle MANUAL State Hold vs AUTO Altitude State Machine");
    Serial.println("    'c' : Toggle MicroSD Card CSV Logging (Manual START / STOP)");
    Serial.println("    'b' : Run BNO055 9-DOF IMU Calibration (Loads/Saves EEPROM profile)");
    Serial.println("    'z' : Tare Heading to Geographic North (0.0 deg) in real time");
    Serial.println("    'u' : Full UNWIND Both Servos (180 deg / 2400 us)");
    Serial.println("    'x' : Full BRAKE Both Servos  (0 deg / 500 us)");
    Serial.println("    'd' : DETACH Servos (0 mA Standby)");
    Serial.println("    '?' : Show this command menu");
    Serial.println("  [Note: LoRa Telemetry is transmitting silently at 1 Hz on Serial1]");
    Serial.println("======================================================================\n");
}

void printActiveStatusReport() {
    FlightState st = flightSM.getState();
    updateSensorActivityFlags(st);

    Serial.println("\n+--------------------------------------------------------------------+");
    Serial.print("| FLATSAT STATUS REPORT | STATE: ");
    Serial.print(flightSM.getStateName());
    Serial.print(" | MODE: ");
    Serial.println(manualStateHold ? "MANUAL HOLD" : "AUTO FSM");
    Serial.println("+--------------------------------------------------------------------+");

    // 1. SENSORS ACTIVE / INACTIVE MATRIX
    Serial.println("| [SENSORS STATUS]");

    Serial.print("|   1. MS5611 Barometer : ");
    if (!baroOK) {
        Serial.println("HARDWARE OFFLINE (Check I2C 0x77/0x76)");
    } else {
        Serial.print(baroActiveInState ? "ACTIVE   (10 Hz EMA)   " : "INACTIVE (Standby)     ");
        Serial.print("| Alt: ");
        Serial.print(lastBaroAlt, 2);
        Serial.print(" m | P: ");
        Serial.print(lastPressurePa, 0);
        Serial.print(" Pa | T: ");
        Serial.print(lastTempC, 1);
        Serial.println(" C");
    }

    Serial.print("|   2. NEO-M9N GNSS     : ");
    if (!gnssOK) {
        Serial.println("HARDWARE OFFLINE (Check I2C 0x42)");
    } else {
        Serial.print(gnssActiveInState ? "ACTIVE   (" : "INACTIVE (");
        Serial.print(currentGnssRateHz);
        Serial.print(" Hz UBX)    | Sats: ");
        Serial.print(gnssSats);
        Serial.print(" | Lat: ");
        Serial.print(gnssLat, 5);
        Serial.print(" | Lon: ");
        Serial.print(gnssLon, 5);
        Serial.print(" | Time: ");
        Serial.println(gnssTimeStr);
    }

    Serial.print("|   3. BNO055 IMU       : ");
    if (!imuOK) {
        Serial.println("HARDWARE OFFLINE (Check I2C 0x28/0x29)");
    } else {
        uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;
        bno.getCalibration(&sys, &gyro, &accel, &mag);
        if (imuActiveInState) {
            Serial.print("ACTIVE   (10 Hz GNC | Calib: S=");
            Serial.print(sys);
            Serial.print(" G=");
            Serial.print(gyro);
            Serial.print(" A=");
            Serial.print(accel);
            Serial.print(" M=");
            Serial.print(mag);
            Serial.print(") | Hdg: ");
            Serial.print(heading_deg, 1);
            Serial.print(" deg | Roll: ");
            Serial.print(roll, 1);
            Serial.print(" | Pitch: ");
            Serial.print(pitch, 1);
            Serial.print(" | Gz: ");
            Serial.print(spin, 2);
            Serial.println(" rad/s");
        } else {
            Serial.print("STANDBY  (Calib: S=");
            Serial.print(sys);
            Serial.print(" G=");
            Serial.print(gyro);
            Serial.print(" A=");
            Serial.print(accel);
            Serial.print(" M=");
            Serial.print(mag);
            Serial.println(") - Powered Standby on Pad");
        }
    }

    Serial.println("+--------------------------------------------------------------------+");

    // 2. ACTUATORS & SUBSYSTEMS MATRIX
    Serial.println("| [ACTUATORS & GNC SUBSYSTEMS]");

    Serial.print("|   4. Burnwire XY-MOS (Pin 4, 700m Deploy) : ");
    if (actuators.isBurnwirePinHigh()) {
        Serial.println("ACTIVE   [HIGH - MELTING REEFING LINE (30.0s Pulse)]");
    } else {
        Serial.println("INACTIVE [LOW - OFF]");
    }

    Serial.print("|   5. Buzzer XY-MOS   (Pin 5, Landing)     : ");
    if (flightSM.isBuzzerActive()) {
        Serial.print("ACTIVE   [500ms Pulsed Beacon | Pin currently ");
        Serial.print(actuators.isBuzzerPinHigh() ? "HIGH]" : "LOW]");
        Serial.println();
    } else {
        Serial.println("INACTIVE [LOW - OFF]");
    }

    Serial.print("|   6. ES3004 Brake Servos (Pins 2 & 3)     : ");
#if SERVO_USE_STANDARD_POSITIONAL
    if (flightSM.isSteeringEnabled()) {
        float lAng = SERVO_NEUTRAL_ANGLE + actuators.getLeftBrake() * (SERVO_MAX_BRAKE_ANGLE - SERVO_NEUTRAL_ANGLE);
        float rAng = SERVO_NEUTRAL_ANGLE + actuators.getRightBrake() * (SERVO_MAX_BRAKE_ANGLE - SERVO_NEUTRAL_ANGLE);
        Serial.print("ACTIVE   [Steer: ");
        Serial.print(currentSteering, 2);
        Serial.print(" | Left (Pin 3): ");
        Serial.print(lAng, 1);
        Serial.print(" deg (");
        Serial.print(actuators.getLeftServoPulseUs());
        Serial.print(" us) | Right (Pin 2): ");
        Serial.print(rAng, 1);
        Serial.print(" deg (");
        Serial.print(actuators.getRightServoPulseUs());
        Serial.println(" us)]");
    } else {
        Serial.println("INACTIVE [DETACHED - 0 mA Draw]");
    }
#else
    if (!actuators.isCalibrated()) {
        Serial.print("LOCKED OUT [UNCALIBRATED - FAULT: ");
        Serial.print(getActuatorFaultName(actuators.getFaultCode()));
        Serial.println("]");
    } else if (flightSM.isSteeringEnabled()) {
        Serial.print("ACTIVE   [Steer: ");
        Serial.print(currentSteering, 2);
        Serial.print(" | L: ");
        Serial.print(getSpoolDirectionName(actuators.getLeftSpoolDirection()));
        Serial.print(" ");
        Serial.print(actuators.getLeftBrake() * 100.0f, 0);
        Serial.print("% (");
        Serial.print(actuators.getLeftServoPulseUs());
        Serial.print(" us, ");
        Serial.print(actuators.getLeftEstimatedPullMm(), 1);
        Serial.print(" mm) | R: ");
        Serial.print(getSpoolDirectionName(actuators.getRightSpoolDirection()));
        Serial.print(" ");
        Serial.print(actuators.getRightBrake() * 100.0f, 0);
        Serial.print("% (");
        Serial.print(actuators.getRightServoPulseUs());
        Serial.print(" us, ");
        Serial.print(actuators.getRightEstimatedPullMm(), 1);
        Serial.println(" mm)]");
    } else {
        Serial.println("INACTIVE [DETACHED - 0 mA Draw]");
    }
#endif

    Serial.print("|   7. MT2204 Flywheel (Pin 6, 2300KV ESC)    : ");
    if (actuators.isFlywheelActive()) {
        Serial.print("ACTIVE   [CONSTANT ");
        Serial.print((int)actuators.getFlywheelRPM());
        Serial.print(" RPM | PWM: ");
        Serial.print(actuators.getFlywheelPulseUs());
        Serial.println(" us]");
    } else {
        Serial.print("INACTIVE [0 RPM | PWM: ");
        Serial.print(actuators.getFlywheelPulseUs());
        Serial.println(" us Armed Idle]");
    }

    Serial.print("|   8. 5-State EKF & Guidance               : ");
    Serial.println(ekfActiveInState ? "ACTIVE   (10 Hz Sensor Fusion)" : "INACTIVE (Bypassed)");

    Serial.print("|   9. LoRa Radio TX   (Serial1 @ 115200)   : ");
    if (telemetryEnabled) {
        Serial.print("ACTIVE   (1 Hz Silent Background TX | Packets Sent: ");
        Serial.print(packetCount);
        Serial.println(")");
    } else {
        Serial.println("STANDBY  (Waiting for Ground Station GPS:<lat>,<lon> & START)");
    }
    Serial.print("|  10. Target Landing Coordinates           : ");
    if (targetGPSConfigured) {
        Serial.print("Lat ");
        Serial.print(targetGPS.lat, 6);
        Serial.print(", Lon ");
        Serial.print(targetGPS.lon, 6);
        Serial.print(" | Local (E, N): (");
        Serial.print(targetLocal.x, 1);
        Serial.print(" m, ");
        Serial.print(targetLocal.y, 1);
        Serial.println(" m)");
    } else {
        Serial.print("Default Local Offset (E: ");
        Serial.print(targetLocal.x, 1);
        Serial.print(" m, N: ");
        Serial.print(targetLocal.y, 1);
        Serial.println(" m)");
    }
    float pinV = 0.0f;
    float batV = readBatteryVoltage(&pinV);
    Serial.print("|  11. MicroSD CSV Logging (Onboard): ");
    if (sdLoggingActive) {
        Serial.print("ACTIVE   (File: ");
        Serial.print(csvFileName);
        Serial.print(" | Records: ");
        Serial.print(sdRecordCount);
        Serial.println(")");
    } else {
        Serial.println(sdOK ? "ARMED    (Automatic on ASCENT)" : "OFFLINE  (No MicroSD Card Detected)");
    }
    Serial.print("|  12. Battery Divider (150k/47k)   : ");
    Serial.print("A0 = ");
    Serial.print(pinV, 3);
    Serial.print(" V -> Battery: ");
    Serial.print(batV, 2);
    Serial.println(" V");
    Serial.println("+--------------------------------------------------------------------+\n");
}

void updateTargetFromGPS() {
    if (!targetGPSConfigured) return;
    if (launchGPSFixed) {
        targetLocal = gps_to_local(targetGPS, launchGPS);
        waypointManager.init(LocalPoint(0.0f, 0.0f), targetLocal, NUM_WAYPOINTS, WAYPOINT_ACCEPTANCE_RAD);
        loiterGuidance.setCenter(targetLocal);
    } else if (gnssLat != 0.0 || gnssLon != 0.0) {
        targetLocal = gps_to_local(targetGPS, GPSPoint(gnssLat, gnssLon));
        waypointManager.init(LocalPoint(0.0f, 0.0f), targetLocal, NUM_WAYPOINTS, WAYPOINT_ACCEPTANCE_RAD);
        loiterGuidance.setCenter(targetLocal);
    }
}

void setFlatSatState(FlightState newState);

void processLoRaTelecommand(char* rawCmd) {
    while (*rawCmd == ' ' || *rawCmd == '\t') rawCmd++;
    size_t len = strlen(rawCmd);
    while (len > 0 && (rawCmd[len - 1] == '\r' || rawCmd[len - 1] == '\n' || rawCmd[len - 1] == ' ')) {
        rawCmd[--len] = '\0';
    }
    if (len == 0) return;

    if (strncmp(rawCmd, "CanSat TX", 9) == 0) {
        Serial.print("\n>>> [LORA MODULE BOOT] ");
        Serial.print(rawCmd);
        Serial.println(" <<<");
        return;
    }

    if (strncasecmp(rawCmd, "CMD:", 4) == 0) {
        rawCmd += 4;
        while (*rawCmd == ' ') rawCmd++;
    }

    // Strip optional "STATE:" prefix
    const char* stateToken = rawCmd;
    if (strncasecmp(rawCmd, "STATE:", 6) == 0) {
        stateToken = rawCmd + 6;
        while (*stateToken == ' ') stateToken++;
    }

    // 1. State Change Commands (1-8 or State Names)
    if (strcasecmp(stateToken, "1") == 0 || strcasecmp(stateToken, "LAUNCH_PAD") == 0 || strcasecmp(stateToken, "PAD") == 0) {
        manualStateHold = true;
        setFlatSatState(FlightState::LAUNCH_PAD);
        return;
    }
    if (strcasecmp(stateToken, "2") == 0 || strcasecmp(stateToken, "ASCENT") == 0) {
        manualStateHold = true;
        setFlatSatState(FlightState::ASCENT);
        return;
    }
    if (strcasecmp(stateToken, "3") == 0 || strcasecmp(stateToken, "DROGUE_DESCENT") == 0 || strcasecmp(stateToken, "DROGUE") == 0) {
        manualStateHold = true;
        setFlatSatState(FlightState::DROGUE_DESCENT);
        return;
    }
    if (strcasecmp(stateToken, "4") == 0 || strcasecmp(stateToken, "PARAFOIL_DEPLOYMENT") == 0 || strcasecmp(stateToken, "DEPLOY") == 0 || strcasecmp(stateToken, "F") == 0 || strcasecmp(stateToken, "BURN") == 0 || strcasecmp(stateToken, "BURNWIRE") == 0 || strcasecmp(stateToken, "DROP") == 0) {
        setFlatSatState(FlightState::PARAFOIL_DEPLOYMENT);
        return;
    }
    if (strcasecmp(stateToken, "5") == 0 || strcasecmp(stateToken, "PRECISION_GUIDANCE") == 0 || strcasecmp(stateToken, "PRECISION") == 0) {
        manualStateHold = true;
        setFlatSatState(FlightState::PRECISION_GUIDANCE);
        return;
    }
    if (strcasecmp(stateToken, "6") == 0 || strcasecmp(stateToken, "TERMINAL_GUIDANCE") == 0 || strcasecmp(stateToken, "TERMINAL") == 0) {
        manualStateHold = true;
        setFlatSatState(FlightState::TERMINAL_GUIDANCE);
        return;
    }
    if (strcasecmp(stateToken, "7") == 0 || strcasecmp(stateToken, "FINAL_APPROACH") == 0 || strcasecmp(stateToken, "FINAL") == 0) {
        manualStateHold = true;
        setFlatSatState(FlightState::FINAL_APPROACH);
        return;
    }
    if (strcasecmp(stateToken, "8") == 0 || strcasecmp(stateToken, "LANDING") == 0 || strcasecmp(stateToken, "LAND") == 0) {
        manualStateHold = true;
        setFlatSatState(FlightState::LANDING);
        return;
    }
    if (strcasecmp(rawCmd, "AUTO") == 0) {
        manualStateHold = false;
        LORA_SERIAL.println("ACK:MODE=AUTO");
        Serial.println("\n>>> [LORA CMD] Switched to AUTO ALTITUDE STATE MACHINE <<<");
        return;
    }

    if (strcasecmp(rawCmd, "START") == 0 || strcasecmp(rawCmd, "TELEM_START") == 0 || strcasecmp(rawCmd, "TELEM:START") == 0) {
        telemetryEnabled = true;
        lastTelemetryMs = millis() - 1000UL;
        Serial.println("\n>>> [LORA CMD] Telemetry STARTED by Ground Station <<<");
        return;
    }

    if (strcasecmp(rawCmd, "STOP") == 0 || strcasecmp(rawCmd, "TELEM_STOP") == 0 || strcasecmp(rawCmd, "TELEM:STOP") == 0) {
        telemetryEnabled = false;
        Serial.println("\n>>> [LORA CMD] Telemetry STOPPED by Ground Station <<<");
        return;
    }

    // MicroSD Card Telecommands
    if (strcasecmp(rawCmd, "SD:START") == 0 || strcasecmp(rawCmd, "SD_START") == 0) {
        startSDLogging();
        LORA_SERIAL.print("ACK:SD=STARTED,FILE=");
        LORA_SERIAL.println(csvFileName);
        return;
    }
    if (strcasecmp(rawCmd, "SD:STOP") == 0 || strcasecmp(rawCmd, "SD_STOP") == 0) {
        stopSDLogging();
        LORA_SERIAL.println("ACK:SD=STOPPED");
        return;
    }

    // BNO055 IMU Calibration Commands
    if (strcasecmp(rawCmd, "CALIB") == 0 || strcasecmp(rawCmd, "CALIB:IMU") == 0 || strcasecmp(rawCmd, "CAL") == 0) {
        LORA_SERIAL.println("ACK:CALIB=STARTED");
        calibrateBNO055OnBoot(30000UL);
        return;
    }
    if (strcasecmp(rawCmd, "CALIB:CLEAR") == 0 || strcasecmp(rawCmd, "CALIB:RESET") == 0) {
        clearBNO055EEPROM();
        LORA_SERIAL.println("ACK:CALIB=CLEARED");
        return;
    }
    if (strcasecmp(rawCmd, "TARE") == 0 || strcasecmp(rawCmd, "ZERO") == 0 || strcasecmp(rawCmd, "TARE:HDG") == 0 || strcasecmp(rawCmd, "ZERO:HDG") == 0) {
        if (imuOK) {
            imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
            activeHeadingOffsetDeg = wrap_180(-euler.x());
            heading_deg = wrap_360(euler.x() + activeHeadingOffsetDeg);
            LORA_SERIAL.print("ACK:TARE_HDG=0.0,OFFSET=");
            LORA_SERIAL.println(activeHeadingOffsetDeg, 1);
            Serial.print("[IMU TARE] Nose calibrated to 0.0 deg North via command! Offset: ");
            Serial.print(activeHeadingOffsetDeg, 1);
            Serial.println(" deg");
        }
        return;
    }
    if (strcasecmp(rawCmd, "UNWIND") == 0 || strcasecmp(rawCmd, "UNWIND:FULL") == 0) {
        manualSteeringOverride = true;
        overrideSteeringVal = 0.0f;
        currentSteering = 0.0f;
        actuators.setDirectPulse(ES3004_MAX_PULSE_US, ES3004_MAX_PULSE_US);
        LORA_SERIAL.println("ACK:ACT=UNWOUND");
        Serial.println("[ACTUATOR] Full UNWIND commanded via telecommand (180 deg / 2400 us).");
        return;
    }

    if (strcasecmp(rawCmd, "RESET:ACT") == 0 || strcasecmp(rawCmd, "CLEAR:ACT") == 0) {
        actuators.resetEstimatedLinePosition();
        LORA_SERIAL.println("ACK:ACT=RESET");
        Serial.println("[ACTUATOR] Open-loop line pull estimate reset to 0.0 mm.");
        return;
    }

    const char* coords = rawCmd;
    if (strncasecmp(rawCmd, "GPS:", 4) == 0) {
        coords = rawCmd + 4;
    } else if (strncasecmp(rawCmd, "TARGET:", 7) == 0) {
        coords = rawCmd + 7;
    }
    while (*coords == ' ') coords++;

    const char* comma = strchr(coords, ',');
    if (comma != NULL) {
        double inLat = strtod(coords, NULL);
        double inLon = strtod(comma + 1, NULL);
        if (isfinite(inLat) && isfinite(inLon) && fabs(inLat) <= 90.0 && fabs(inLon) <= 180.0 && (inLat != 0.0 || inLon != 0.0)) {
            targetGPS = GPSPoint(inLat, inLon);
            targetGPSConfigured = true;
            updateTargetFromGPS();

            LORA_SERIAL.print("ACK:TARGET_GPS=");
            LORA_SERIAL.print(targetGPS.lat, 7);
            LORA_SERIAL.print(",");
            LORA_SERIAL.print(targetGPS.lon, 7);
            if (launchGPSFixed || gnssLat != 0.0 || gnssLon != 0.0) {
                LORA_SERIAL.print(",EAST_M=");
                LORA_SERIAL.print(targetLocal.x, 1);
                LORA_SERIAL.print(",NORTH_M=");
                LORA_SERIAL.println(targetLocal.y, 1);
            } else {
                LORA_SERIAL.println(",STATUS=SAVED_AWAITING_PAD_FIX");
            }

            Serial.print("\n>>> [LORA CMD] Precision Landing Target GPS Set: ");
            Serial.print(targetGPS.lat, 7);
            Serial.print(", ");
            Serial.print(targetGPS.lon, 7);
            Serial.println(" <<<");
        }
    }
}

void pollLoRaTelecommands() {
    while (LORA_SERIAL.available() > 0) {
        char c = (char)LORA_SERIAL.read();
        if (c == '\r' || c == '\n') {
            if (loraCmdLen > 0) {
                loraCmdBuf[loraCmdLen] = '\0';
                processLoRaTelecommand(loraCmdBuf);
                loraCmdLen = 0;
            }
        } else if (loraCmdLen < sizeof(loraCmdBuf) - 1) {
            loraCmdBuf[loraCmdLen++] = c;
        }
    }
}

void setFlatSatState(FlightState newState) {
    uint32_t now = millis();
    if (newState == FlightState::PARAFOIL_DEPLOYMENT) {
        manualStateHold = false;
    } else {
        manualStateHold = true;
    }
    flightSM.forceState(newState, now);
    updateSensorActivityFlags(newState);

    // Auto-arm LoRa telemetry and SD logging when switched to ASCENT or higher
    if (newState >= FlightState::ASCENT) {
        telemetryEnabled = true;
        telemAscentTriggered = true;
        if (!sdLoggingActive) {
            startSDLogging();
        }
        sdAscentTriggered = true;
    } else if (newState == FlightState::LAUNCH_PAD) {
        telemAscentTriggered = false;
        sdAscentTriggered = false;
    }

    // Immediately apply actuator outputs for the newly selected state
    if (!flightSM.isSteeringEnabled()) {
        currentSteering = 0.0f;
    } else if (manualSteeringOverride) {
        currentSteering = overrideSteeringVal;
    }
    actuators.applySteering(currentSteering, flightSM.isSteeringEnabled(), now, newState);
    actuators.setFlywheel(flightSM.isFlywheelActive());
    actuators.setBurnwire(flightSM.isBurnwireActive());
    actuators.updateBuzzer(flightSM.isBuzzerActive(), now);
    lastBurnwirePinState = actuators.isBurnwirePinHigh();

    LORA_SERIAL.print("ACK:STATE=");
    LORA_SERIAL.println(flightSM.getStateName());

    Serial.print("\n>>> [FLATSAT] Switched Flight State to: ");
    Serial.print(flightSM.getStateName());
    if (newState == FlightState::PARAFOIL_DEPLOYMENT) {
        Serial.print(" (700m Deployment -> Burnwire XY-MOS Pin 4 HIGH for 5000 ms!)");
    } else if (newState == FlightState::LANDING) {
        Serial.print(" (Touchdown -> Recovery Buzzer XY-MOS Pin 5 Beacon ON!)");
    }
    Serial.println(" <<<");

    printActiveStatusReport();
}

void handleSingleKeyInput(char c) {
    switch (c) {
        case '1':
            manualStateHold = true;
            setFlatSatState(FlightState::LAUNCH_PAD);
            break;
        case '2':
            manualStateHold = true;
            setFlatSatState(FlightState::ASCENT);
            break;
        case '3':
            manualStateHold = true;
            setFlatSatState(FlightState::DROGUE_DESCENT);
            break;
        case '4':
        case 'f':
        case 'F':
            manualStateHold = true;
            setFlatSatState(FlightState::PARAFOIL_DEPLOYMENT);
            break;
        case '5':
            manualStateHold = true;
            setFlatSatState(FlightState::PRECISION_GUIDANCE);
            break;
        case '6':
            manualStateHold = true;
            setFlatSatState(FlightState::TERMINAL_GUIDANCE);
            break;
        case '7':
            manualStateHold = true;
            setFlatSatState(FlightState::FINAL_APPROACH);
            break;
        case '8':
            manualStateHold = true;
            setFlatSatState(FlightState::LANDING);
            break;
        case 's':
        case 'S':
            printActiveStatusReport();
            break;
        case 't':
        case 'T':
            telemetryEnabled = !telemetryEnabled;
            Serial.print("\n>>> [TELEMETRY] Manually toggled to: ");
            Serial.println(telemetryEnabled ? "STARTED (1 Hz TX ON)" : "STOPPED (STANDBY)");
            printActiveStatusReport();
            break;
        case 'l':
        case 'L':
            manualSteeringOverride = true;
            overrideSteeringVal = -1.0f;
            currentSteering = -1.0f;
            actuators.setDirectPulse(ES3004_MIN_PULSE_US, ES3004_NEUTRAL_PULSE_US);
            Serial.println("[OVERRIDE] Steering set to -1.0 (Full LEFT Brake: Left Pin 3 -> 0 deg / 500 us, Right Pin 2 -> 90 deg / 1500 us).");
            printActiveStatusReport();
            break;
        case 'n':
        case 'N':
            manualSteeringOverride = true;
            overrideSteeringVal = 0.0f;
            currentSteering = 0.0f;
            actuators.setDirectPulse(ES3004_NEUTRAL_PULSE_US, ES3004_NEUTRAL_PULSE_US);
            Serial.println("[OVERRIDE] Steering set to 0.0 (Neutral Glide: Both Servos -> 90 deg / 1500 us).");
            printActiveStatusReport();
            break;
        case 'r':
        case 'R':
            manualSteeringOverride = true;
            overrideSteeringVal = 1.0f;
            currentSteering = 1.0f;
            actuators.setDirectPulse(ES3004_NEUTRAL_PULSE_US, ES3004_MIN_PULSE_US);
            Serial.println("[OVERRIDE] Steering set to +1.0 (Full RIGHT Brake: Left Pin 3 -> 90 deg / 1500 us, Right Pin 2 -> 0 deg / 500 us).");
            printActiveStatusReport();
            break;
        case 'a':
        case 'A':
            actuators.resetEstimatedLinePosition();
            Serial.println("[ACTUATOR] Open-loop line pull estimate reset to 0.0 mm.");
            printActiveStatusReport();
            break;
        case 'g':
        case 'G':
            manualSteeringOverride = false;
            Serial.println("[GNC] Resumed live EKF + PID closed-loop steering.");
            printActiveStatusReport();
            break;
        case 'm':
        case 'M':
            manualStateHold = !manualStateHold;
            Serial.print("[MODE] Switched to: ");
            Serial.println(manualStateHold ? "MANUAL STATE HOLD (FlatSat Bench Mode)" : "AUTO ALTITUDE STATE MACHINE");
            break;
        case 'c':
        case 'C':
            if (!sdLoggingActive) {
                startSDLogging();
            } else {
                stopSDLogging();
            }
            printActiveStatusReport();
            break;
        case 'b':
        case 'B':
            calibrateBNO055OnBoot(30000UL);
            printActiveStatusReport();
            break;
        case 'z':
        case 'Z':
            if (imuOK) {
                imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
                activeHeadingOffsetDeg = wrap_180(-euler.x());
                heading_deg = wrap_360(euler.x() + activeHeadingOffsetDeg);
                Serial.println("\n*******************************************************");
                Serial.print(">>> [IMU TARE] NOSE CALIBRATED TO GEOGRAPHIC NORTH (0.0 deg)! <<<\n");
                Serial.print("    Raw Sensor Heading: ");
                Serial.print(euler.x(), 1);
                Serial.print(" deg | New Active Offset: ");
                Serial.print(activeHeadingOffsetDeg, 1);
                Serial.println(" deg");
                Serial.println("*******************************************************\n");
                printActiveStatusReport();
            }
            break;
        case 'u':
        case 'U':
            manualSteeringOverride = true;
            overrideSteeringVal = 0.0f;
            currentSteering = 0.0f;
            actuators.setDirectPulse(ES3004_MAX_PULSE_US, ES3004_MAX_PULSE_US);
            Serial.println("[OVERRIDE] Full UNWIND commanded (Both Servos -> 180 deg / 2400 us).");
            printActiveStatusReport();
            break;
        case 'x':
        case 'X':
            manualSteeringOverride = true;
            overrideSteeringVal = 0.0f;
            currentSteering = 0.0f;
            actuators.setDirectPulse(ES3004_MIN_PULSE_US, ES3004_MIN_PULSE_US);
            Serial.println("[OVERRIDE] Full BRAKE / FLARE commanded (Both Servos -> 0 deg / 500 us).");
            printActiveStatusReport();
            break;
        case 'd':
        case 'D':
            manualSteeringOverride = true;
            overrideSteeringVal = 0.0f;
            currentSteering = 0.0f;
            actuators.setDirectPulse(0, 0);
            Serial.println("[STANDBY] Servos DETACHED (0 mA current draw).");
            printActiveStatusReport();
            break;
        case '?':
        case 'h':
        case 'H':
            printFlatSatMenu();
            break;
        default:
            break;
    }
}

void handleSerialInput() {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\r' || c == '\n') {
            if (usbCmdLen == 1) {
                handleSingleKeyInput(usbCmdBuf[0]);
            } else if (usbCmdLen > 1) {
                usbCmdBuf[usbCmdLen] = '\0';
                processLoRaTelecommand(usbCmdBuf);
                printActiveStatusReport();
            }
            usbCmdLen = 0;
        } else if (usbCmdLen < sizeof(usbCmdBuf) - 1) {
            usbCmdBuf[usbCmdLen++] = c;
        }
    }
}

void transmitLoRaTelemetry(
    uint32_t missionTimeSeconds,
    uint32_t packetNumber,
    float altitudeM,
    float pressurePa,
    float baroTempC,
    float batteryV,
    const char* gpsTime,
    double latitudeDeg,
    double longitudeDeg,
    float gpsAltitudeM,
    int satellites,
    float accelX,
    float accelY,
    float accelZ,
    float gyroZ,
    const char* currentState,
    const char* optionalData = ""
) {
    // Pure CSV Telemetry format (no labels) transmitted over LoRa (Serial1).
    // Conforms strictly to IN-SPACe CanSat India competition specification.
    float accelTotal = sqrtf(accelX * accelX + accelY * accelY + accelZ * accelZ);

    LORA_SERIAL.print(TEAM_ID);
    LORA_SERIAL.print(",");
    LORA_SERIAL.print(missionTimeSeconds);
    LORA_SERIAL.print(",");
    LORA_SERIAL.print(packetNumber);
    LORA_SERIAL.print(",");
    LORA_SERIAL.print(altitudeM, 2);
    LORA_SERIAL.print(",");
    LORA_SERIAL.print(pressurePa, 2);
    LORA_SERIAL.print(",");
    LORA_SERIAL.print(baroTempC, 2);
    LORA_SERIAL.print(",");
    LORA_SERIAL.print(batteryV, 2);
    LORA_SERIAL.print(",");
    LORA_SERIAL.print(gpsTime);
    LORA_SERIAL.print(",");
    LORA_SERIAL.print(latitudeDeg, 7);
    LORA_SERIAL.print(",");
    LORA_SERIAL.print(longitudeDeg, 7);
    LORA_SERIAL.print(",");
    LORA_SERIAL.print(gpsAltitudeM, 2);
    LORA_SERIAL.print(",");
    LORA_SERIAL.print(satellites);
    LORA_SERIAL.print(",");
    LORA_SERIAL.print(accelTotal, 2);
    LORA_SERIAL.print(",");
    LORA_SERIAL.print(gyroZ, 4);
    LORA_SERIAL.print(",");
    LORA_SERIAL.print(currentState);
    LORA_SERIAL.print(",");
    LORA_SERIAL.println(optionalData ? optionalData : "");
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
    Serial.begin(115200);         // Interactive FlatSat Console (USB Serial)
    LORA_SERIAL.begin(LORA_BAUD); // Ra-08H LoRa Transmitter (Serial1 @ 115200)
    pinMode(VBAT_PIN, INPUT);
    analogReadResolution(12);     // 12-bit ADC (0..4095) for 150k/47k battery divider on A0
    delay(1000);

    // Initialize Actuators (2x ES3004 Servos, Burnwire XY-MOS, Buzzer XY-MOS)
    actuators.init();

    // Start I2C
    Wire.begin();
    Wire.setClock(100000);

    // 1. MS5611 Barometer
    baroOK = baro.begin();
    if (!baroOK) {
        baro = MS5611(0x76, &Wire);
        baroOK = baro.begin();
    }

    // 2. BNO055 IMU
    imuOK = bno.begin();
    if (!imuOK) {
        bno = Adafruit_BNO055(55, 0x29, &Wire);
        imuOK = bno.begin();
    }
    if (imuOK) {
        bno.setExtCrystalUse(true);
        if (BNO_MOUNTED_UPSIDE_DOWN) {
            bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P4);
            bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P4);
            Serial.println("[BNO055] Axis Remap: INVERTED (P4 applied for upside-down mount -> Z inverted).");
        }
        Serial.print("[BNO055] Heading Offset: ");
        Serial.print(IMU_HEADING_OFFSET_DEG, 1);
        Serial.println(" deg (Calibrated to Geographic North 0 deg).");
        calibrateBNO055OnBoot();
    }

    // 3. NEO-M9N GNSS
    gnssOK = gnss.begin(Wire);
    if (gnssOK) {
        gnss.setI2COutput(COM_TYPE_UBX);
        gnss.setNavigationFrequency(1); // 1 Hz in Launch Pad mode
        gnss.setAutoPVT(true);
        currentGnssRateHz = 1;
    }

    // 4. MicroSD Card (Teensy 4.1 Native SDIO socket)
    sdOK = SD.begin(SDCARD_CS_PIN);

    // Calibrate Baseline Launch Pressure (50-sample average + 5-sample warm-up)
    if (baroOK) {
        baro.setOversampling(OSR_ULTRA_HIGH);
        Serial.println("Calibrating MS5611 baseline pressure (50 samples)...");

        for (int i = 0; i < 5; i++) {
            baro.read();
            delay(50);
        }

        float sumPressure = 0.0f;
        int validSamples = 0;
        for (int i = 0; i < 50; i++) {
            if (baro.read() == MS5611_READ_OK) {
                float currentP = baro.getPressure();
                if (isfinite(currentP) && currentP > 0.0f) {
                    sumPressure += currentP;
                    validSamples++;
                }
            }
            delay(50);
        }

        if (validSamples > 0) {
            launchPressure = sumPressure / (float)validSamples;
        } else {
            launchPressure = SEA_LEVEL_HPA;
        }
        filteredPressure = launchPressure;
        groundAlt = 0.0f;
    }

    // Initialize Waypoints & EKF
    LocalPoint startPt(0.0f, 0.0f);
    waypointManager.init(startPt, targetLocal, NUM_WAYPOINTS, WAYPOINT_ACCEPTANCE_RAD);
    loiterGuidance.setCenter(targetLocal);
    ekf.reset(0.0f, 0.0f, 0.0f, 0.0f);

    flightSM.reset();
    flightSM.forceState(FlightState::LAUNCH_PAD, millis());
    updateSensorActivityFlags(FlightState::LAUNCH_PAD);

    missionStartMs = millis();
    lastControlUpdateMs = missionStartMs;
    lastTelemetryMs = missionStartMs;

    printFlatSatMenu();
    printActiveStatusReport();
}

// ============================================================================
// Main Execution Loop
// ============================================================================
void loop() {
    uint32_t now = millis();
    actuators.update(now);

    // 1. Check for interactive FlatSat user commands on USB Serial & LoRa Telecommands on Serial1
    handleSerialInput();
    pollLoRaTelecommands();

    // 2. 10 Hz Control & Sensor Polling Loop
    if (now - lastControlUpdateMs >= (1000 / CONTROL_LOOP_HZ)) {
        float dt = (lastControlUpdateMs == 0) ? 0.1f : (float)(now - lastControlUpdateMs) / 1000.0f;
        lastControlUpdateMs = now;

        FlightState currentState = flightSM.getState();
        updateSensorActivityFlags(currentState);

        // --- A. Poll MS5611 Barometer (if active in current state) ---
        float alt_agl = lastBaroAlt;
        if (baroOK && baroActiveInState && baro.read() == MS5611_READ_OK) {
            float rawPressure = baro.getPressure();
            if (isfinite(rawPressure) && rawPressure > 0.0f) {
                lastPressurePa = rawPressure * 100.0f;
                lastTempC = baro.getTemperature();
                filteredPressure = (EMA_ALPHA * rawPressure) + ((1.0f - EMA_ALPHA) * filteredPressure);
                alt_agl = pressureToAlt(filteredPressure, launchPressure);
                lastBaroAlt = alt_agl;

                if (ekfActiveInState) {
                    ekf.correctBaro(alt_agl);
                }
            }
        }

        // --- B. Poll BNO055 IMU (ONLY when active in current state) ---
        if (imuOK && imuActiveInState) {
            imu::Vector<3> accVec = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
            imu::Vector<3> eulerVec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
            imu::Vector<3> gyroVec = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

            ax = accVec.x();
            ay = accVec.y();
            az = accVec.z();
            heading_deg = wrap_360(eulerVec.x() + activeHeadingOffsetDeg);
            roll = eulerVec.y();
            pitch = eulerVec.z();
            spin = gyroVec.z();

            if (ekfActiveInState) {
                ekf.correctHeading(heading_deg);
            }
        }

        // --- C. Poll NEO-M9N GNSS (if active in current state) ---
        LocalPoint posLocal(0.0f, 0.0f);
        if (gnssOK && gnssActiveInState) {
            gnss.checkUblox();
            if (gnss.getPVT()) {
                gnssSats = gnss.getSIV();
                gnssAltMSL = gnss.getAltitudeMSL() / 1000.0f;
                snprintf(gnssTimeStr, sizeof(gnssTimeStr), "%02d:%02d:%02d",
                         gnss.getHour(), gnss.getMinute(), gnss.getSecond());
                gnssLat = gnss.getLatitude() / 1e7;
                gnssLon = gnss.getLongitude() / 1e7;

                if (gnssSats >= 4) {
                    if (!launchGPSFixed) {
                        launchGPS = GPSPoint(gnssLat, gnssLon);
                        launchGPSFixed = true;
                        updateTargetFromGPS();
                    }
                    if (ekfActiveInState) {
                        GPSPoint currentGPS(gnssLat, gnssLon);
                        posLocal = gps_to_local(currentGPS, launchGPS);
                        float vx_meas = (float)(gnss.getNedEastVel()) / 1000.0f;
                        float vy_meas = (float)(gnss.getNedNorthVel()) / 1000.0f;
                        ekf.correctGPS(posLocal.x, posLocal.y, vx_meas, vy_meas, true);

                        if (currentState <= FlightState::DROGUE_DESCENT) {
                            windEstimator.update(vx_meas, vy_meas);
                        } else {
                            float psi_rad = deg2rad(heading_deg);
                            float air_vx = FORWARD_AIRSPEED * sinf(psi_rad);
                            float air_vy = FORWARD_AIRSPEED * cosf(psi_rad);
                            windEstimator.update(vx_meas - air_vx, vy_meas - air_vy);
                        }
                    }
                }
            }
        }

        // --- D. EKF Prediction & State Machine Update ---
        if (ekfActiveInState) {
            if (currentState == FlightState::DROGUE_DESCENT) {
                ekf.setKinematics(0.0f, DROGUE_DESCENT_RATE);
            } else {
                ekf.setKinematics(FORWARD_AIRSPEED, PARAFOIL_DESCENT_RATE);
            }
            ekf.setWindEstimate(windEstimator.getWindX(), windEstimator.getWindY());
            ekf.predict(dt, currentSteering);
        }

        EKFStateEstimate est = ekf.getEstimate();
        LocalPoint navPos(est.x, est.y);
        float navHeading = est.heading_deg;
        float navAlt = est.altitude;

        if (manualStateHold) {
            // In Manual FlatSat mode: keep the chosen state, but tick safety timers (e.g. 5.0s burnwire cutoff)
            flightSM.tickTimers(now);
        } else {
            FlightState prevState = currentState;
            flightSM.update(alt_agl, navHeading, navPos, targetLocal, dt, now);
            currentState = flightSM.getState();
            if (currentState != prevState) {
                Serial.print("\n>>> [AUTO FSM] State Transitioned to: ");
                Serial.print(flightSM.getStateName());
                Serial.println(" <<<");
                printActiveStatusReport();
            }
        }

        // --- E. Guidance & Steering Calculation ---
        if (manualSteeringOverride && flightSM.isSteeringEnabled()) {
            currentSteering = overrideSteeringVal;
        } else if (currentState == FlightState::PRECISION_GUIDANCE) {
            guidanceController.setWind(windEstimator.getWindX(), windEstimator.getWindY());
            GuidanceOutput gOut = guidanceController.update(navPos, navHeading, navAlt, dt);
            currentSteering = gOut.steering;
        } else if (currentState == FlightState::TERMINAL_GUIDANCE) {
            loiterGuidance.setWind(windEstimator.getWindX(), windEstimator.getWindY());
            currentSteering = loiterGuidance.update(navPos, navHeading);
        } else if (currentState == FlightState::FINAL_APPROACH) {
            float desired_course = bearing(navPos, targetLocal);
            WindCompResult comp = calculate_wind_corrected_heading(
                desired_course, FORWARD_AIRSPEED, windEstimator.getWindX(), windEstimator.getWindY()
            );
            float err_deg = heading_error(navHeading, comp.heading_deg);
            currentSteering = pid.update(deg2rad(err_deg), dt);
        } else {
            currentSteering = 0.0f;
            pid.reset();
        }

        // --- F. Update Actuators (Continuous Brake Spool Servos, MT2204 Flywheel ESC & Dual XY-MOS MOSFETs) ---
        if (!manualSteeringOverride) {
            actuators.applySteering(currentSteering, flightSM.isSteeringEnabled(), now, currentState);
        }
        actuators.setFlywheel(flightSM.isFlywheelActive());
        actuators.setBurnwire(flightSM.isBurnwireActive());
        actuators.updateBuzzer(flightSM.isBuzzerActive(), now);

        // Notify on USB Serial when the 5.0s Burnwire XY-MOS pulse finishes
        if (lastBurnwirePinState && !actuators.isBurnwirePinHigh()) {
            Serial.println(">>> [BURNWIRE XY-MOS] 30.0s deployment pulse complete. Pin 4 -> LOW (OFF). <<<");
        }
        lastBurnwirePinState = actuators.isBurnwirePinHigh();
    }

    // 3. 1 Hz Silent LoRa Telemetry Transmission (Auto-starts on ASCENT, or gated by 'START' / 'STOP')
    if (flightSM.getState() >= FlightState::ASCENT) {
        if (!telemAscentTriggered) {
            telemetryEnabled = true;
            telemAscentTriggered = true;
            Serial.println(">>> [LORA] ASCENT phase active -> 1 Hz Telemetry Transmission AUTO-STARTED! <<<");
        }
    } else if (flightSM.getState() == FlightState::LAUNCH_PAD) {
        telemAscentTriggered = false;
    }

    if (telemetryEnabled && (now - lastTelemetryMs >= 1000UL)) {
        lastTelemetryMs = now;
        packetCount++;

        float vbat = readBatteryVoltage();

        uint32_t missionTimeSec = (now - missionStartMs) / 1000UL;

        transmitLoRaTelemetry(
            missionTimeSec,
            packetCount,
            isfinite(lastBaroAlt) ? lastBaroAlt : 0.0f,
            isfinite(lastPressurePa) ? lastPressurePa : 0.0f,
            isfinite(lastTempC) ? lastTempC : 0.0f,
            vbat,
            gnssTimeStr,
            gnssLat,
            gnssLon,
            gnssAltMSL,
            gnssSats,
            ax,
            ay,
            az,
            spin,
            flightSM.getStateName()
        );
    }

    // 4. MicroSD Card Data Logging (Automatically starts as soon as ASCENT starts)
    if (flightSM.getState() >= FlightState::ASCENT) {
        if (!sdLoggingActive && !sdAscentTriggered) {
            startSDLogging();
            sdAscentTriggered = true;
        }
    } else if (flightSM.getState() == FlightState::LAUNCH_PAD) {
        sdAscentTriggered = false;
    }

    if (sdLoggingActive && (now - lastSdLogMs >= SD_LOG_INTERVAL_MS)) {
        lastSdLogMs = now;
        sdRecordCount++;

        float vbat = readBatteryVoltage();
        uint32_t missionTimeSec = (now - missionStartMs) / 1000UL;

        logDataToSD(
            missionTimeSec,
            sdRecordCount,
            isfinite(lastBaroAlt) ? lastBaroAlt : 0.0f,
            isfinite(lastPressurePa) ? lastPressurePa : 0.0f,
            isfinite(lastTempC) ? lastTempC : 0.0f,
            vbat,
            gnssTimeStr,
            gnssLat,
            gnssLon,
            gnssAltMSL,
            gnssSats,
            ax,
            ay,
            az,
            spin,
            flightSM.getStateName()
        );
    }
}
