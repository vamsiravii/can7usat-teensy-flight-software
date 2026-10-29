#include <Arduino.h>

// ============================================================================
// CAN-7USAT Team 017 | Standalone Readytosky MT2204 2300KV Flywheel Test
// Board:  Teensy 4.1 (Uses Hardware FlexPWM2_2_A on Pin 6 — NO buggy Servo.h!)
// Motor:  Readytosky MT2204 2300KV CW Brushless Outrunner
// ESC:    Simplifly 30A 2S-6S BLDC ESC (Signal on Pin 6)
// Power:  2S Li-ion Battery (7.4V Nominal -> 17020 Max RPM)
// Target: Constant 6000 RPM (1353 us PWM pulse width, 35.3% throttle)
// ============================================================================

#define FLYWHEEL_ESC_PIN          6
#define FLYWHEEL_MOTOR_KV         2300.0f
#define FLYWHEEL_BATTERY_VOLTS    7.4f
#define FLYWHEEL_TARGET_RPM       6000.0f

#define ESC_MIN_PULSE_US          1000      // 0 RPM (Armed Idle)
#define ESC_MAX_PULSE_US          1653      // 100% Throttle (17020 RPM @ 7.4V)
#define ESC_TARGET_6000RPM_US     1353      // 1000 + round(1000 * 6000 / (2300 * 7.4)) = 1353 us

int currentPulseUs = ESC_MIN_PULSE_US;
int targetPulseUs = ESC_MIN_PULSE_US;
float currentRpm = 0.0f;
uint32_t lastRampMs = 0;

static inline void writeHardwarePwmUs(uint8_t pin, int pulse_us) {
    if (pulse_us <= 0) {
        analogWrite(pin, 0);
        return;
    }
    // 50 Hz period = 20,000 us; 16-bit resolution = 65536 counts per 20,000 us
    uint32_t duty = ((uint32_t)pulse_us * 65536UL + 10000UL) / 20000UL;
    if (duty > 65535UL) duty = 65535UL;
    analogWrite(pin, (int)duty);
}

int rpmToPulseUs(float targetRpm) {
    float maxRpm = FLYWHEEL_MOTOR_KV * FLYWHEEL_BATTERY_VOLTS;
    float throttle = targetRpm / maxRpm;
    if (throttle < 0.0f) throttle = 0.0f;
    if (throttle > 1.0f) throttle = 1.0f;
    return (int)(ESC_MIN_PULSE_US + throttle * (ESC_MAX_PULSE_US - ESC_MIN_PULSE_US) + 0.5f);
}

void setFlywheelRPM(float targetRpm) {
    currentRpm = targetRpm;
    targetPulseUs = rpmToPulseUs(targetRpm);
    if (targetPulseUs <= ESC_MIN_PULSE_US) {
        currentPulseUs = ESC_MIN_PULSE_US;
        writeHardwarePwmUs(FLYWHEEL_ESC_PIN, ESC_MIN_PULSE_US);
        Serial.println("[MT2204 FLYWHEEL] STOPPED -> 0 RPM (1000 us Hardware FlexPWM)");
    } else {
        Serial.print("[MT2204 FLYWHEEL] Soft-starting ramp to ");
        Serial.print((int)currentRpm);
        Serial.print(" RPM (Target PWM: ");
        Serial.print(targetPulseUs);
        Serial.println(" us)...");
    }
}

void printMenu() {
    Serial.println("\n==============================================================");
    Serial.println("  CAN-7USAT Readytosky MT2204 2300KV Flywheel Bench Test");
    Serial.println("==============================================================");
    Serial.println("  Pin 6 -> BLDC ESC Signal Wire (Hardware 50 Hz FlexPWM)");
    Serial.println("--------------------------------------------------------------");
    Serial.println("  Serial Commands (Send via Serial Monitor @ 115200):");
    Serial.println("    'r' or '6' : Run Constant 6000 RPM (1353 us Flight Speed)");
    Serial.println("    's' or '0' : STOP Motor (1000 us Armed Idle / 0 RPM)");
    Serial.println("    '+'        : Nudge PWM +10 us");
    Serial.println("    '-'        : Nudge PWM -10 us");
    Serial.println("    '?'        : Print this menu and current ESC output");
    Serial.println("==============================================================\n");
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    // Configure 16-bit 50 Hz Hardware FlexPWM on Pin 6 and hold 1000 us to arm ESC
    analogWriteResolution(16);
    analogWriteFrequency(FLYWHEEL_ESC_PIN, 50.0f);
    writeHardwarePwmUs(FLYWHEEL_ESC_PIN, ESC_MIN_PULSE_US);

    Serial.println("[INIT] Arming Simplifly 30A ESC at exact 1000 us (3.0s)...");
    delay(3000);
    Serial.println("[READY] ESC Armed at 1000 us (0 RPM).");

    printMenu();
}

void loop() {
    uint32_t now = millis();

    // Non-blocking soft-start ramp to prevent BLDC inrush current spike / brownout reset
    if (currentPulseUs < targetPulseUs && (now - lastRampMs >= 80)) {
        lastRampMs = now;
        if (currentPulseUs < 1060) {
            currentPulseUs = 1060;
        } else {
            currentPulseUs += 8;
            if (currentPulseUs >= targetPulseUs) {
                currentPulseUs = targetPulseUs;
                Serial.print("[MT2204 FLYWHEEL] Reached steady target (");
                Serial.print(currentPulseUs);
                Serial.println(" us).");
            }
        }
        writeHardwarePwmUs(FLYWHEEL_ESC_PIN, currentPulseUs);
    }

    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'r' || cmd == 'R' || cmd == '6') {
            setFlywheelRPM(FLYWHEEL_TARGET_RPM);
        } else if (cmd == 's' || cmd == 'S' || cmd == '0' || cmd == ' ') {
            setFlywheelRPM(0.0f);
        } else if (cmd == '+') {
            targetPulseUs += 10;
            if (targetPulseUs > ESC_MAX_PULSE_US) targetPulseUs = ESC_MAX_PULSE_US;
            currentPulseUs = targetPulseUs;
            writeHardwarePwmUs(FLYWHEEL_ESC_PIN, currentPulseUs);
            Serial.print("[TUNE] ESC PWM = ");
            Serial.print(currentPulseUs);
            Serial.println(" us");
        } else if (cmd == '-') {
            targetPulseUs -= 10;
            if (targetPulseUs < ESC_MIN_PULSE_US) targetPulseUs = ESC_MIN_PULSE_US;
            currentPulseUs = targetPulseUs;
            writeHardwarePwmUs(FLYWHEEL_ESC_PIN, currentPulseUs);
            Serial.print("[TUNE] ESC PWM = ");
            Serial.print(currentPulseUs);
            Serial.println(" us");
        } else if (cmd == '?') {
            printMenu();
            Serial.print("Status -> RPM: ");
            Serial.print((int)currentRpm);
            Serial.print(" RPM | ESC Pulse: ");
            Serial.print(currentPulseUs);
            Serial.println(" us");
        }
    }
}
