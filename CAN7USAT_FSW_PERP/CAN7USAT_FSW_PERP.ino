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
// Global Subsystems & State
// Sensors:   MS5611 Barometer, NEO-M9N GNSS, BNO055 IMU
// Actuators: 2x XY-MOS MOSFETs (Pin 4 Burnwire @ 700m, Pin 5 Buzzer @ Landing)
//            2x Continuous-Rotation Servos on M4 Brake-Line Screws (Pins 2 & 3)
//            1x Readytosky MT2204 2300KV Momentum Bias Flywheel ESC (Pin 6)
// Telemetry: Transmitted at 1 Hz on Serial1 (LoRa) ONLY (Silent on USB Serial)
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

// Flight Management & Actuators (2x ES3004 Servos + 2x XY-MOS MOSFETs)
FlightStateMachine flightSM;
ActuatorManager actuators;

// Barometer Calibration & Filtering State Variables
float launchPressure = 1013.25f;
float filteredPressure = 1013.25f;
const float EMA_ALPHA = BARO_EMA_ALPHA; // 0.4f: reduces 0.015 mbar noise with minimal phase lag
float groundAlt = 0.0f;

// Navigation & Timing State Variables
GPSPoint launchGPS(0.0, 0.0);
bool launchGPSFixed = false;
GPSPoint targetGPS(0.0, 0.0);
bool targetGPSConfigured = false;
LocalPoint targetLocal(TARGET_DEFAULT_EAST, TARGET_DEFAULT_NORTH);

// LoRa Telecommand & Telemetry Gating State
bool telemetryEnabled = false; // Starts automatically on ASCENT or via Ground Station "start" command 
bool telemAscentTriggered = false;
bool manualStateHold = false;  // Set true when a state change is triggered by command (send AUTO to resume auto transitions)
bool manualSteeringOverride = false;
float overrideSteeringVal = 0.0f;
bool lastBurnwirePinState = false;
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
float roll = 0.0f, pitch = 0.0f, spin = 0.0f;
char gnssTimeStr[16] = "UNAVAILABLE";
double gnssLat = 0.0, gnssLon = 0.0;
float gnssAltMSL = 0.0f;
int gnssSats = 0;
bool gnssHighRateEnabled = false;

// ============================================================================
// Utilities & LoRa Telemetry Output (Silent on Transmitter USB Serial)
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

void printSensorStatus() {
    float pinV = 0.0f;
    float batV = readBatteryVoltage(&pinV);
    Serial.println();
    Serial.println("========== SENSOR & ACTUATOR INITIALIZATION ==========");
    Serial.print("MS5611 Barometer:          ");
    Serial.println(baroOK ? "OK" : "FAILED");
    Serial.print("BNO055 IMU:                ");
    if (imuOK) {
        uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;
        bno.getCalibration(&sys, &gyro, &accel, &mag);
        Serial.print("OK (Calib: Sys=");
        Serial.print(sys);
        Serial.print("/3, Gyr=");
        Serial.print(gyro);
        Serial.print("/3, Acc=");
        Serial.print(accel);
        Serial.print("/3, Mag=");
        Serial.print(mag);
        Serial.print("/3 | Heading Offset: ");
        Serial.print(IMU_HEADING_OFFSET_DEG, 1);
        Serial.println(" deg)");
    } else {
        Serial.println("FAILED");
    }
    Serial.print("NEO-M9N GNSS:              ");
    Serial.println(gnssOK ? "OK" : "FAILED");
    Serial.print("MicroSD Card (Built-in):   ");
    Serial.println(sdOK ? "READY (Arms on ASCENT)" : "NOT DETECTED (Check card)");
    Serial.print("Battery Divider (150k/47k):");
    Serial.print(" A0 = ");
    Serial.print(pinV, 3);
    Serial.print(" V -> Battery = ");
    Serial.print(batV, 2);
    Serial.println(" V");
    Serial.print("Brake Spool Servos (Pins 2,3):");
    if (actuators.isCalibrated()) {
        Serial.println(" READY (Continuous-rotation M4 spool guidance enabled)");
    } else {
        Serial.print(" UNCALIBRATED LOCKOUT (Motors detached/safe; Fault: ");
        Serial.print(getActuatorFaultName(actuators.getFaultCode()));
        Serial.println(")");
    }
    Serial.println("Burnwire XY-MOS MOSFET:    READY (Pin 4, Deploy @ 700m)");
    Serial.println("Buzzer XY-MOS MOSFET:      READY (Pin 5, Activate @ Landing)");
    Serial.println("MT2204 Flywheel ESC:       READY (Pin 6, 6000 RPM Post-Deploy -> Landing)");
    Serial.println("LoRa Telecommand/Telem:    STANDBY (Send GPS:<lat>,<lon> then START)");
    Serial.println("======================================================");
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

void printCurrentStateSummary() {
    float pinV = 0.0f;
    float batV = readBatteryVoltage(&pinV);
    Serial.println("+------------------------------------------------------------------------------+");
    Serial.print("| FSW STATUS | STATE: ");
    Serial.print(flightSM.getStateName());
    Serial.print(" | MODE: ");
    Serial.print(manualStateHold ? "MANUAL COMMAND HOLD" : "AUTO ALTITUDE SM");
    Serial.print(" | VBAT: ");
    Serial.print(batV, 2);
    Serial.print(" V (A0: ");
    Serial.print(pinV, 3);
    Serial.println(" V)");
    if (imuOK) {
        uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;
        bno.getCalibration(&sys, &gyro, &accel, &mag);
        Serial.print("| BNO055 Calib: Sys=");
        Serial.print(sys);
        Serial.print("/3, Gyr=");
        Serial.print(gyro);
        Serial.print("/3, Acc=");
        Serial.print(accel);
        Serial.print("/3, Mag=");
        Serial.print(mag);
        Serial.println("/3");
    }
    Serial.print("| SD Logging: ");
    if (sdLoggingActive) {
        Serial.print("ACTIVE (File: ");
        Serial.print(csvFileName);
        Serial.print(", Records: ");
        Serial.print(sdRecordCount);
        Serial.println(")");
    } else {
        Serial.println(sdOK ? "ARMED (Triggers on ASCENT)" : "SD CARD NOT DETECTED");
    }
    Serial.print("| Burnwire (Pin 4): ");
    Serial.print(actuators.isBurnwirePinHigh() ? "ACTIVE (HIGH 30.0s)" : "OFF (LOW)");
    Serial.print(" | Buzzer (Pin 5): ");
    Serial.println(flightSM.isBuzzerActive() ? "ACTIVE (BEACON)" : "OFF (LOW)");
    Serial.print("| Flywheel (Pin 6): ");
    Serial.print((int)actuators.getFlywheelRPM());
    Serial.print(" RPM (");
    Serial.print(actuators.getFlywheelPulseUs());
    Serial.println(" us)");
    Serial.print("| Brake Spools (Pins 2,3): ");
    if (!actuators.isCalibrated()) {
        Serial.print("LOCKED OUT [UNCALIBRATED - FAULT: ");
        Serial.print(getActuatorFaultName(actuators.getFaultCode()));
        Serial.println("]");
    } else if (flightSM.isSteeringEnabled()) {
        Serial.print("L=[");
        Serial.print(getSpoolDirectionName(actuators.getLeftSpoolDirection()));
        Serial.print(" ");
        Serial.print(actuators.getLeftBrake() * 100.0f, 0);
        Serial.print("% ");
        Serial.print(actuators.getLeftServoPulseUs());
        Serial.print("us ");
        Serial.print(actuators.getLeftEstimatedPullMm(), 1);
        Serial.print("mm] R=[");
        Serial.print(getSpoolDirectionName(actuators.getRightSpoolDirection()));
        Serial.print(" ");
        Serial.print(actuators.getRightBrake() * 100.0f, 0);
        Serial.print("% ");
        Serial.print(actuators.getRightServoPulseUs());
        Serial.print("us ");
        Serial.print(actuators.getRightEstimatedPullMm(), 1);
        Serial.println("mm]");
    } else {
        Serial.println("DETACHED (Flight state disables steering; 0 uA)");
    }
    Serial.println("+------------------------------------------------------------------------------+");
}

void setFlightState(FlightState newState) {
    uint32_t now = millis();
    if (newState == FlightState::PARAFOIL_DEPLOYMENT) {
        // Drone drop test: deployment triggers burnwire and auto-advances to GUIDANCE & LANDING
        manualStateHold = false;
    } else {
        manualStateHold = true;
    }
    flightSM.forceState(newState, now);

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

    if (!flightSM.isSteeringEnabled()) {
        currentSteering = 0.0f;
    } else if (manualSteeringOverride) {
        currentSteering = overrideSteeringVal;
    }
    actuators.applySteering(currentSteering, flightSM.isSteeringEnabled(), now, flightSM.getState());
    actuators.setFlywheel(flightSM.isFlywheelActive());
    actuators.setBurnwire(flightSM.isBurnwireActive());
    actuators.updateBuzzer(flightSM.isBuzzerActive(), now);
    lastBurnwirePinState = actuators.isBurnwirePinHigh();

    LORA_SERIAL.print("ACK:STATE=");
    LORA_SERIAL.println(flightSM.getStateName());

    Serial.print("\n>>> [STATE CMD] Switched Flight State to: ");
    Serial.print(flightSM.getStateName());
    Serial.println(" <<<");
    printCurrentStateSummary();
}

void processTelecommand(char* rawCmd) {
    while (*rawCmd == ' ' || *rawCmd == '\t') rawCmd++;
    size_t len = strlen(rawCmd);
    while (len > 0 && (rawCmd[len - 1] == '\r' || rawCmd[len - 1] == '\n' || rawCmd[len - 1] == ' ')) {
        rawCmd[--len] = '\0';
    }
    if (len == 0) return;

    // Ignore Ra-08H startup banner
    if (strncmp(rawCmd, "CanSat TX", 9) == 0) return;

    if (strncasecmp(rawCmd, "CMD:", 4) == 0) {
        rawCmd += 4;
        while (*rawCmd == ' ') rawCmd++;
    }

    // Strip optional "STATE:" prefix for state commands
    const char* stateToken = rawCmd;
    if (strncasecmp(rawCmd, "STATE:", 6) == 0) {
        stateToken = rawCmd + 6;
        while (*stateToken == ' ') stateToken++;
    }

    // 1. State Change Commands (1-8 or State Names)
    if (strcasecmp(stateToken, "1") == 0 || strcasecmp(stateToken, "LAUNCH_PAD") == 0 || strcasecmp(stateToken, "PAD") == 0) {
        setFlightState(FlightState::LAUNCH_PAD);
        return;
    }
    if (strcasecmp(stateToken, "2") == 0 || strcasecmp(stateToken, "ASCENT") == 0) {
        setFlightState(FlightState::ASCENT);
        return;
    }
    if (strcasecmp(stateToken, "3") == 0 || strcasecmp(stateToken, "DROGUE_DESCENT") == 0 || strcasecmp(stateToken, "DROGUE") == 0) {
        setFlightState(FlightState::DROGUE_DESCENT);
        return;
    }
    if (strcasecmp(stateToken, "4") == 0 || strcasecmp(stateToken, "PARAFOIL_DEPLOYMENT") == 0 || strcasecmp(stateToken, "DEPLOY") == 0 || strcasecmp(stateToken, "F") == 0 || strcasecmp(stateToken, "BURN") == 0 || strcasecmp(stateToken, "BURNWIRE") == 0 || strcasecmp(stateToken, "DROP") == 0) {
        setFlightState(FlightState::PARAFOIL_DEPLOYMENT);
        return;
    }
    if (strcasecmp(stateToken, "5") == 0 || strcasecmp(stateToken, "PRECISION_GUIDANCE") == 0 || strcasecmp(stateToken, "PRECISION") == 0) {
        setFlightState(FlightState::PRECISION_GUIDANCE);
        return;
    }
    if (strcasecmp(stateToken, "6") == 0 || strcasecmp(stateToken, "TERMINAL_GUIDANCE") == 0 || strcasecmp(stateToken, "TERMINAL") == 0) {
        setFlightState(FlightState::TERMINAL_GUIDANCE);
        return;
    }
    if (strcasecmp(stateToken, "7") == 0 || strcasecmp(stateToken, "FINAL_APPROACH") == 0 || strcasecmp(stateToken, "FINAL") == 0) {
        setFlightState(FlightState::FINAL_APPROACH);
        return;
    }
    if (strcasecmp(stateToken, "8") == 0 || strcasecmp(stateToken, "LANDING") == 0 || strcasecmp(stateToken, "LAND") == 0) {
        setFlightState(FlightState::LANDING);
        return;
    }

    // 2. Mode & Steering Override Commands
    if (strcasecmp(rawCmd, "AUTO") == 0 || strcasecmp(rawCmd, "M") == 0) {
        manualStateHold = !manualStateHold;
        LORA_SERIAL.print("ACK:MODE=");
        LORA_SERIAL.println(manualStateHold ? "MANUAL_HOLD" : "AUTO");
        Serial.print("[MODE CMD] Switched to: ");
        Serial.println(manualStateHold ? "MANUAL COMMAND HOLD" : "AUTO ALTITUDE STATE MACHINE");
        printCurrentStateSummary();
        return;
    }
    if (strcasecmp(rawCmd, "L") == 0 || strcasecmp(rawCmd, "LEFT") == 0) {
        manualSteeringOverride = true;
        overrideSteeringVal = -1.0f;
        currentSteering = -1.0f;
        actuators.applySteering(currentSteering, flightSM.isSteeringEnabled(), millis(), flightSM.getState());
        LORA_SERIAL.println("ACK:STEER=LEFT");
        Serial.println("[STEER CMD] Full LEFT Brake (-1.0 -> Left continuous spool winding).");
        printCurrentStateSummary();
        return;
    }
    if (strcasecmp(rawCmd, "N") == 0 || strcasecmp(rawCmd, "NEUTRAL") == 0) {
        manualSteeringOverride = true;
        overrideSteeringVal = 0.0f;
        currentSteering = 0.0f;
        actuators.applySteering(currentSteering, flightSM.isSteeringEnabled(), millis(), flightSM.getState());
        LORA_SERIAL.println("ACK:STEER=NEUTRAL");
        Serial.println("[STEER CMD] NEUTRAL (0.0 -> Both brake spools stopped).");
        printCurrentStateSummary();
        return;
    }
    if (strcasecmp(rawCmd, "R") == 0 || strcasecmp(rawCmd, "RIGHT") == 0) {
        manualSteeringOverride = true;
        overrideSteeringVal = 1.0f;
        currentSteering = 1.0f;
        actuators.applySteering(currentSteering, flightSM.isSteeringEnabled(), millis(), flightSM.getState());
        LORA_SERIAL.println("ACK:STEER=RIGHT");
        Serial.println("[STEER CMD] Full RIGHT Brake (+1.0 -> Right continuous spool winding).");
        printCurrentStateSummary();
        return;
    }
    if (strcasecmp(rawCmd, "G") == 0 || strcasecmp(rawCmd, "GNC") == 0) {
        manualSteeringOverride = false;
        LORA_SERIAL.println("ACK:STEER=GNC");
        Serial.println("[STEER CMD] Resumed live EKF + PID closed-loop steering.");
        printCurrentStateSummary();
        return;
    }
    if (strcasecmp(rawCmd, "RESET:ACT") == 0 || strcasecmp(rawCmd, "CLEAR:ACT") == 0) {
        actuators.resetEstimatedLinePosition();
        LORA_SERIAL.println("ACK:ACT=RESET");
        Serial.println("[ACTUATOR] Open-loop line pull estimate reset to 0.0 mm.");
        printCurrentStateSummary();
        return;
    }
    if (strcasecmp(rawCmd, "S") == 0 || strcasecmp(rawCmd, "STATUS") == 0) {
        printCurrentStateSummary();
        return;
    }

    // 3. Telemetry START / STOP Commands
    if (strcasecmp(rawCmd, "START") == 0 || strcasecmp(rawCmd, "TELEM_START") == 0 || strcasecmp(rawCmd, "TELEM:START") == 0) {
        telemetryEnabled = true;
        lastTelemetryMs = millis() - 1000UL;
        Serial.println("[LORA CMD] Telemetry STARTED.");
        return;
    }

    if (strcasecmp(rawCmd, "STOP") == 0 || strcasecmp(rawCmd, "TELEM_STOP") == 0 || strcasecmp(rawCmd, "TELEM:STOP") == 0) {
        telemetryEnabled = false;
        Serial.println("[LORA CMD] Telemetry STOPPED.");
        return;
    }
    if (strcasecmp(rawCmd, "T") == 0) {
        telemetryEnabled = !telemetryEnabled;
        Serial.print("[TELEMETRY] Toggled to: ");
        Serial.println(telemetryEnabled ? "STARTED" : "STOPPED");
        return;
    }

    // 4. MicroSD Card Telecommands (Manual bench testing & verification)
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

    // 5. BNO055 IMU Calibration Commands
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

    // 6. Precision Landing Target GPS Coordinates (GPS:<lat>,<lon>)
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

            Serial.print("[LORA CMD] Precision Landing Target GPS Set: ");
            Serial.print(targetGPS.lat, 7);
            Serial.print(", ");
            Serial.print(targetGPS.lon, 7);
            Serial.print(" | Local (E, N): (");
            Serial.print(targetLocal.x, 1);
            Serial.print("m, ");
            Serial.print(targetLocal.y, 1);
            Serial.println("m)");
        }
    }
}

void pollTelecommands() {
    while (LORA_SERIAL.available() > 0) {
        char c = (char)LORA_SERIAL.read();
        if (c == '\r' || c == '\n') {
            if (loraCmdLen > 0) {
                loraCmdBuf[loraCmdLen] = '\0';
                processTelecommand(loraCmdBuf);
                loraCmdLen = 0;
            }
        } else if (loraCmdLen < sizeof(loraCmdBuf) - 1) {
            loraCmdBuf[loraCmdLen++] = c;
        }
    }
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\r' || c == '\n') {
            if (usbCmdLen > 0) {
                usbCmdBuf[usbCmdLen] = '\0';
                processTelecommand(usbCmdBuf);
                usbCmdLen = 0;
            }
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
    Serial.begin(115200);         // USB status output (no 1 Hz telemetry spam)
    LORA_SERIAL.begin(LORA_BAUD); // Ra-08H LoRa module (Serial1 @ 115200)
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
    if (imuOK) {
        bnoI2CAddr = 0x28;
    } else {
        bno = Adafruit_BNO055(55, 0x29, &Wire);
        imuOK = bno.begin();
        if (imuOK) bnoI2CAddr = 0x29;
    }
    if (imuOK) {
        bno.setExtCrystalUse(true);
        if (BNO_MOUNTED_UPSIDE_DOWN) {
            bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P4);
            bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P4);
            Serial.println("[BNO055] Axis Remap: INVERTED (P4 applied for upside-down mount -> Z inverted).");
        }
        calibrateBNO055OnBoot();
    }

    // 3. NEO-M9N GNSS
    gnssOK = gnss.begin(Wire);
    if (gnssOK) {
        gnss.setI2COutput(COM_TYPE_UBX);
        gnss.setNavigationFrequency(1); // 1 Hz on launch pad to conserve power
        gnss.setAutoPVT(true);
    }

    // 4. MicroSD Card (Teensy 4.1 Native SDIO socket)
    sdOK = SD.begin(SDCARD_CS_PIN);

    printSensorStatus();

    // Calibrate Baseline Launch Pressure (50-sample average + 5-sample warm-up)
    if (baroOK) {
        baro.setOversampling(OSR_ULTRA_HIGH);
        Serial.println("Calibrating baseline pressure...");

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

        Serial.print("Ground Reference Pressure (hPa): ");
        Serial.println(launchPressure, 2);
    }

    // Initialize Waypoints from Start (0,0) to Target (1200,0)
    LocalPoint startPt(0.0f, 0.0f);
    waypointManager.init(startPt, targetLocal, NUM_WAYPOINTS, WAYPOINT_ACCEPTANCE_RAD);
    loiterGuidance.setCenter(targetLocal);

    // Initialize EKF
    ekf.reset(0.0f, 0.0f, 0.0f, 0.0f);

    flightSM.reset();
    flightSM.update(0.0f, 0.0f, startPt, targetLocal, 0.1f, millis());

    missionStartMs = millis();
    lastControlUpdateMs = missionStartMs;
    lastTelemetryMs = missionStartMs;

    Serial.println("Flight State: LAUNCH_PAD (Waiting for Ground Station GPS:<lat>,<lon> & START)");
}

// ============================================================================
// Main Execution Loop
// ============================================================================
void loop() {
    uint32_t now = millis();

    // ------------------------------------------------------------------------
    // Task 0: Poll Incoming LoRa & USB Telecommands (GPS Target, START, STOP)
    // ------------------------------------------------------------------------
    pollTelecommands();
    actuators.update(now);

    // ------------------------------------------------------------------------
    // Task 1: 10 Hz Control Loop (EKF, Guidance, State Machine, Spools & XY-MOS)
    // ------------------------------------------------------------------------
    if (now - lastControlUpdateMs >= (1000 / CONTROL_LOOP_HZ)) {
        float dt = (lastControlUpdateMs == 0) ? 0.1f : (float)(now - lastControlUpdateMs) / 1000.0f;
        lastControlUpdateMs = now;

        FlightState prevState = flightSM.getState();
        FlightState currentState = prevState;

        // 1. Read MS5611 Barometer (Required to detect launch: alt > 15m)
        float alt_agl = lastBaroAlt;
        if (baroOK && baro.read() == MS5611_READ_OK) {
            float rawPressure = baro.getPressure();
            if (isfinite(rawPressure) && rawPressure > 0.0f) {
                lastPressurePa = rawPressure * 100.0f;
                lastTempC = baro.getTemperature();

                // EMA filter on pressure: balances noise reduction with minimal phase lag
                filteredPressure = (EMA_ALPHA * rawPressure) + ((1.0f - EMA_ALPHA) * filteredPressure);

                // Relative hypsometric altitude equation referenced to calibrated launch pressure
                alt_agl = pressureToAlt(filteredPressure, launchPressure);
                lastBaroAlt = alt_agl;

                if (currentState > FlightState::LAUNCH_PAD && currentState < FlightState::LANDING) {
                    ekf.correctBaro(alt_agl);
                }
            }
        }

        // ====================================================================
        // LAUNCH PAD STANDBY MODE (Saves battery during extended holds)
        // ====================================================================
        if (currentState == FlightState::LAUNCH_PAD) {
            // Check GNSS at 1 Hz to acquire satellites and lock launch coordinates
            if (gnssOK) {
                gnss.checkUblox();
                if (gnss.getPVT()) {
                    gnssSats = gnss.getSIV();
                    gnssAltMSL = gnss.getAltitudeMSL() / 1000.0f;
                    snprintf(gnssTimeStr, sizeof(gnssTimeStr), "%02d:%02d:%02d",
                             gnss.getHour(), gnss.getMinute(), gnss.getSecond());

                    gnssLat = gnss.getLatitude() / 1e7;
                    gnssLon = gnss.getLongitude() / 1e7;

                    if (gnssSats >= 4 && !launchGPSFixed) {
                        launchGPS = GPSPoint(gnssLat, gnssLon);
                        launchGPSFixed = true;
                        updateTargetFromGPS();
                        Serial.println("[PAD] 3D GNSS Fix acquired. Launch coordinates locked.");
                    }
                }
            }

            // Update flight state machine to check for lift-off (alt_agl > 15m) unless in manual command hold
            if (!manualStateHold) {
                flightSM.update(alt_agl, 0.0f, LocalPoint(0.0f, 0.0f), targetLocal, dt, now);
            } else {
                flightSM.tickTimers(now);
            }

            // Keep continuous-rotation servos detached (0 mA), Flywheel OFF (1000 us), and XY-MOS MOSFETs OFF on pad
            currentSteering = 0.0f;
            actuators.applySteering(0.0f, false, now, FlightState::LAUNCH_PAD);
            actuators.setFlywheel(false);
            actuators.setBurnwire(false);
            actuators.updateBuzzer(false, now);
        } else {
            // ================================================================
            // ACTIVE FLIGHT MODE (ASCENT, DROGUE, DEPLOY @ 700m, GUIDANCE, LANDING)
            // ================================================================

            // Boost GNSS navigation rate to 5 Hz for precision flight
            if (gnssOK && !gnssHighRateEnabled && currentState < FlightState::LANDING) {
                gnss.setNavigationFrequency(5);
                gnssHighRateEnabled = true;
            }

            // 2. Read BNO055 IMU (Active during flight prior to landing)
            float heading_deg = 0.0f;
            if (imuOK && currentState < FlightState::LANDING) {
                imu::Vector<3> accVec = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
                imu::Vector<3> eulerVec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
                imu::Vector<3> gyroVec = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

                ax = accVec.x();
                ay = accVec.y();
                az = accVec.z();

                // Adafruit Euler convention: x = heading, y = roll, z = pitch
                // Apply mounting offset to align BNO055 perpendicular heading with vehicle forward axis
                heading_deg = wrap_360(eulerVec.x() + IMU_HEADING_OFFSET_DEG);
                roll = eulerVec.y();
                pitch = eulerVec.z();
                spin = gyroVec.z();

                ekf.correctHeading(heading_deg);
            }

            // 3. Read NEO-M9N GNSS
            LocalPoint posLocal(0.0f, 0.0f);
            if (gnssOK) {
                gnss.checkUblox();
                if (gnss.getPVT()) {
                    gnssSats = gnss.getSIV();
                    gnssAltMSL = gnss.getAltitudeMSL() / 1000.0f;

                    snprintf(gnssTimeStr, sizeof(gnssTimeStr), "%02d:%02d:%02d",
                             gnss.getHour(), gnss.getMinute(), gnss.getSecond());

                    gnssLat = gnss.getLatitude() / 1e7;
                    gnssLon = gnss.getLongitude() / 1e7;

                    if (gnssSats >= 4 && currentState < FlightState::LANDING) {
                        if (!launchGPSFixed) {
                            launchGPS = GPSPoint(gnssLat, gnssLon);
                            launchGPSFixed = true;
                            updateTargetFromGPS();
                        }

                        GPSPoint currentGPS(gnssLat, gnssLon);
                        posLocal = gps_to_local(currentGPS, launchGPS);

                        float vx_meas = (float)(gnss.getNedEastVel()) / 1000.0f;
                        float vy_meas = (float)(gnss.getNedNorthVel()) / 1000.0f;

                        ekf.correctGPS(posLocal.x, posLocal.y, vx_meas, vy_meas, true);

                        // Update Wind Estimator
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

            // 4. EKF Time Prediction
            if (currentState < FlightState::LANDING) {
                ekf.setWindEstimate(windEstimator.getWindX(), windEstimator.getWindY());
                ekf.predict(dt, currentSteering);
            }

            EKFStateEstimate est = ekf.getEstimate();
            LocalPoint navPos(est.x, est.y);
            float navHeading = est.heading_deg;
            float navAlt = est.altitude;

            // 5. Update Flight State Machine (or tick burnwire timer if in manual command hold)
            if (!manualStateHold) {
                flightSM.update(alt_agl, navHeading, navPos, targetLocal, dt, now);
            } else {
                flightSM.tickTimers(now);
                // Safety: even if manual hold is set on bench, detect touchdown to prevent motor burnout
                if (currentState >= FlightState::PRECISION_GUIDANCE && currentState < FlightState::LANDING) {
                    if (alt_agl <= TOUCHDOWN_ALT) {
                        flightSM.forceState(FlightState::LANDING, now);
                        Serial.println("[AUTO TOUCHDOWN] Ground contact detected (< 2.0m AGL) -> LANDING beacon activated!");
                    }
                }
            }
            currentState = flightSM.getState();

            // Kinematics adjustment for EKF based on flight phase
            if (currentState == FlightState::DROGUE_DESCENT) {
                ekf.setKinematics(0.0f, DROGUE_DESCENT_RATE);
            } else {
                ekf.setKinematics(FORWARD_AIRSPEED, PARAFOIL_DESCENT_RATE);
            }

            // 6. GNC Guidance & Control Logic for Parafoil Steering
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

            // 7. Update Actuators:
            //    - Continuous-Rotation Brake Spools (Pins 2 & 3): Bounded open-loop differential brake actuation
            //    - XY-MOS MOSFET #1 (Pin 4): Burnwire active for 5.0s at 700m parafoil deployment
            //    - XY-MOS MOSFET #2 (Pin 5): Recovery buzzer pulsed after touchdown
            //    - Readytosky MT2204 Flywheel ESC (Pin 6): Constant 6000 RPM (1353 us) from post-deployment to landing
            actuators.applySteering(currentSteering, flightSM.isSteeringEnabled(), now, currentState);
            actuators.setFlywheel(flightSM.isFlywheelActive());
            actuators.setBurnwire(flightSM.isBurnwireActive());
            actuators.updateBuzzer(flightSM.isBuzzerActive(), now);

            bool currentBurnwirePinState = actuators.isBurnwirePinHigh();
            if (lastBurnwirePinState && !currentBurnwirePinState) {
                Serial.println("[SAFETY CUTOFF] Burnwire XY-MOS Pin 4 turned LOW after 30.0s duration.");
                printCurrentStateSummary();
            }
            lastBurnwirePinState = currentBurnwirePinState;
        }

        if (flightSM.getState() != prevState) {
            Serial.print("[STATE TRANSITION] -> ");
            Serial.println(flightSM.getStateName());
        }
    }

    // ------------------------------------------------------------------------
    // Task 2: 1 Hz LoRa Telemetry Transmission
    // (Automatically begins as soon as ASCENT starts, or via 'START' telecommand)
    // ------------------------------------------------------------------------
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

        // If on launch pad, sample IMU at 1 Hz for ground station health packet
        if (flightSM.getState() == FlightState::LAUNCH_PAD && imuOK) {
            imu::Vector<3> accVec = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
            imu::Vector<3> eulerVec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
            imu::Vector<3> gyroVec = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

            ax = accVec.x();
            ay = accVec.y();
            az = accVec.z();
            roll = eulerVec.y();
            pitch = eulerVec.z();
            spin = gyroVec.z();
        }

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

    // ------------------------------------------------------------------------
    // Task 3: MicroSD Card Data Logging (Automatically starts as soon as ASCENT starts)
    // ------------------------------------------------------------------------
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
