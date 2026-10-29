/**
 * @file i2c_scanner.ino
 * @brief CAN-7USAT | Teensy 4.1 I2C Bus Scanner & Sensor Checklist
 *
 * Scans Wire (SDA = Pin 18, SCL = Pin 19) for all 7-bit I2C addresses
 * (0x01 to 0x7E), identifies known CAN-7USAT sensors, and prints a
 * diagnostic checklist of detected vs. missing hardware.
 *
 * Serial Monitor (115200 baud) Commands:
 *   's' or Enter -> Run an immediate I2C scan
 *   'a'          -> Toggle auto-scan every 5 seconds (ON by default)
 *   '1'          -> Set I2C clock to 100 kHz (Standard Mode)
 *   '4'          -> Set I2C clock to 400 kHz (Fast Mode)
 */

#include <Arduino.h>
#include <Wire.h>

static bool autoScan = true;
static uint32_t lastScanMs = 0;
static uint32_t currentClockHz = 100000;
static uint32_t scanCount = 0;

// Helper to identify known CAN-7USAT I2C addresses
const char* identifyDevice(uint8_t address) {
    switch (address) {
        case 0x28: return "Adafruit BNO055 9-DOF IMU (Primary ADR=LOW)";
        case 0x29: return "Adafruit BNO055 9-DOF IMU (Alt ADR=HIGH)";
        case 0x42: return "u-blox NEO-M9N GNSS (DDC/I2C UBX)";
        case 0x62: return "Sensirion SCD40 / SCD41 CO2 Sensor";
        case 0x76: return "MS5611 Barometer (CSB=HIGH)";
        case 0x77: return "MS5611 Barometer (CSB=LOW)";
        case 0x68: return "MPU6050 / DS3231 RTC / ICM-20948";
        case 0x75: return "BMP280 / BME280 (Alt)";
        default:   return "Unknown I2C Device";
    }
}

void runI2CScan() {
    scanCount++;
    int foundCount = 0;

    bool foundMS5611 = false;
    uint8_t ms5611Addr = 0;
    bool foundBNO055 = false;
    uint8_t bno055Addr = 0;
    bool foundGNSS = false;
    bool foundSCD40 = false;

    Serial.println();
    Serial.println("============================================================");
    Serial.print("  CAN-7USAT I2C SCAN #");
    Serial.print(scanCount);
    Serial.print(" | Bus: Wire (SDA=18, SCL=19) @ ");
    Serial.print(currentClockHz / 1000);
    Serial.println(" kHz");
    Serial.println("============================================================");

    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();

        if (error == 0) {
            foundCount++;
            Serial.print("  [FOUND] 0x");
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);
            Serial.print("  ->  ");
            Serial.println(identifyDevice(address));

            if (address == 0x76 || address == 0x77) {
                foundMS5611 = true;
                ms5611Addr = address;
            }
            if (address == 0x28 || address == 0x29) {
                foundBNO055 = true;
                bno055Addr = address;
            }
            if (address == 0x42) {
                foundGNSS = true;
            }
            if (address == 0x62) {
                foundSCD40 = true;
            }
        } else if (error == 4) {
            Serial.print("  [ERROR] 0x");
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);
            Serial.println("  ->  Bus error / stuck SDA-SCL line");
        }
    }

    if (foundCount == 0) {
        Serial.println("  [!] NO I2C DEVICES FOUND! Check 3.3V power, GND, SDA (Pin 18), and SCL (Pin 19).");
    } else {
        Serial.print("\n  Total I2C devices responding: ");
        Serial.println(foundCount);
    }

    Serial.println("\n---------------- CAN-7USAT SENSOR CHECKLIST ----------------");
    Serial.print("  1. MS5611 Barometer (0x77/0x76) : ");
    if (foundMS5611) {
        Serial.print("DETECTED (0x");
        Serial.print(ms5611Addr, HEX);
        Serial.println(")");
    } else {
        Serial.println("MISSING");
    }

    Serial.print("  2. BNO055 9-DOF IMU (0x28/0x29) : ");
    if (foundBNO055) {
        Serial.print("DETECTED (0x");
        Serial.print(bno055Addr, HEX);
        Serial.println(")");
    } else {
        Serial.println("MISSING");
    }

    Serial.print("  3. NEO-M9N GNSS     (0x42)      : ");
    Serial.println(foundGNSS ? "DETECTED (0x42)" : "MISSING");

    Serial.print("  4. SCD40 CO2 Sensor (0x62)      : ");
    Serial.println(foundSCD40 ? "DETECTED (0x62)" : "MISSING (Optional)");

    Serial.println("------------------------------------------------------------");
    Serial.println("  Commands: [s] Scan Now | [a] Toggle Auto-Scan | [1] 100kHz | [4] 400kHz");
}

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0 < 3000)) {
        delay(10);
    }

    Wire.begin();
    Wire.setClock(currentClockHz);

    Serial.println("\n============================================================");
    Serial.println("  CAN-7USAT | Teensy 4.1 I2C Scanner Initialized");
    Serial.println("============================================================");

    runI2CScan();
    lastScanMs = millis();
}

void loop() {
    if (Serial.available() > 0) {
        char c = Serial.read();
        if (c == 's' || c == 'S' || c == '\n' || c == '\r') {
            if (c == 's' || c == 'S') {
                runI2CScan();
                lastScanMs = millis();
            }
        } else if (c == 'a' || c == 'A') {
            autoScan = !autoScan;
            Serial.print("\n[CONFIG] Auto-scan every 5s: ");
            Serial.println(autoScan ? "ENABLED" : "PAUSED (press 's' for manual scan)");
        } else if (c == '1') {
            currentClockHz = 100000;
            Wire.setClock(currentClockHz);
            Serial.println("\n[CONFIG] I2C Clock set to 100 kHz (Standard Mode)");
            runI2CScan();
            lastScanMs = millis();
        } else if (c == '4') {
            currentClockHz = 400000;
            Wire.setClock(currentClockHz);
            Serial.println("\n[CONFIG] I2C Clock set to 400 kHz (Fast Mode)");
            runI2CScan();
            lastScanMs = millis();
        }
    }

    if (autoScan && (millis() - lastScanMs >= 5000UL)) {
        lastScanMs = millis();
        runI2CScan();
    }
}
