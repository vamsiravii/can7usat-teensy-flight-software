#include <Arduino.h>
#include <math.h>

// ============================================================================
// CAN-7USAT Team 017 | Standalone Dual Emax ES3004 Parafoil Servo Test
// Board: Teensy 4.1 (Uses Hardware 50 Hz FlexPWM on Pins 2 & 3 — NO Servo.h!)
// Left  ES3004 Servo Pin: Pin 3 (FlexPWM4_2_B) - Interchanged
// Right ES3004 Servo Pin: Pin 2 (FlexPWM4_2_A) - Interchanged
// ============================================================================

#define SERVO_LEFT_PIN            3
#define SERVO_RIGHT_PIN           2

#define SERVO_NEUTRAL_ANGLE       90.0f     // 90 deg = No brake applied (1500 us)
#define SERVO_MAX_BRAKE_ANGLE     0.0f      // 0 deg  = Full brake pulled / Wind (500 us)
#define SERVO_MAX_UNWIND_ANGLE    180.0f    // 180 deg = Full line release / Unwind (2400 us)

#define ES3004_MIN_PULSE_US       500       // Emax ES3004 0 deg pulse width (us) - Wind/Brake
#define ES3004_NEUTRAL_PULSE_US   1500      // Emax ES3004 90 deg pulse width (us) - Neutral Stop
#define ES3004_MAX_PULSE_US       2400      // Emax ES3004 180 deg pulse width (us) - Unwind/Release

bool servosAttached = false;
float leftAngle = SERVO_NEUTRAL_ANGLE;
float rightAngle = SERVO_NEUTRAL_ANGLE;
int leftPulseUs = ES3004_NEUTRAL_PULSE_US;
int rightPulseUs = ES3004_NEUTRAL_PULSE_US;

static inline void writeHardwarePwmUs(uint8_t pin, int pulse_us) {
    if (pulse_us <= 0) {
        analogWrite(pin, 0); // 0% duty -> Pin held LOW (0 mA standby)
        return;
    }
    // 50 Hz period = 20,000 us; 16-bit resolution = 65536 counts per 20,000 us
    uint32_t duty = ((uint32_t)pulse_us * 65536UL + 10000UL) / 20000UL;
    if (duty > 65535UL) duty = 65535UL;
    analogWrite(pin, (int)duty);
}

int angleToPulseUs(float angleDeg) {
    if (angleDeg < 0.0f) angleDeg = 0.0f;
    if (angleDeg > 180.0f) angleDeg = 180.0f;
    if (angleDeg <= 90.0f) {
        // [0, 90] deg -> [500, 1500] us (Winding / Braking span)
        float ratio = angleDeg / 90.0f;
        return (int)(ES3004_MIN_PULSE_US + ratio * (ES3004_NEUTRAL_PULSE_US - ES3004_MIN_PULSE_US));
    } else {
        // [90, 180] deg -> [1500, 2400] us (Unwinding / Release span)
        float ratio = (angleDeg - 90.0f) / 90.0f;
        return (int)(ES3004_NEUTRAL_PULSE_US + ratio * (ES3004_MAX_PULSE_US - ES3004_NEUTRAL_PULSE_US));
    }
}

void setServoAngles(float leftDeg, float rightDeg, const char* label) {
    leftAngle = leftDeg;
    rightAngle = rightDeg;
    leftPulseUs = angleToPulseUs(leftAngle);
    rightPulseUs = angleToPulseUs(rightAngle);

    if (!servosAttached) {
        writeHardwarePwmUs(SERVO_LEFT_PIN, leftPulseUs);
        delay(25);
        writeHardwarePwmUs(SERVO_RIGHT_PIN, rightPulseUs);
        servosAttached = true;
        Serial.println("[ES3004] Servos ENABLED on Pin 3 (Left) & Pin 2 (Right) via Hardware FlexPWM.");
    } else {
        writeHardwarePwmUs(SERVO_LEFT_PIN, leftPulseUs);
        writeHardwarePwmUs(SERVO_RIGHT_PIN, rightPulseUs);
    }

    Serial.print("[");
    Serial.print(label);
    Serial.print("] Left ES3004 (Pin 3): ");
    Serial.print(leftAngle, 1);
    Serial.print(" deg (");
    Serial.print(leftPulseUs);
    Serial.print(" us) | Right ES3004 (Pin 2): ");
    Serial.print(rightAngle, 1);
    Serial.print(" deg (");
    Serial.print(rightPulseUs);
    Serial.println(" us)");
}

// Same differential steering mixer used in CAN7USAT_FSW/actuators.cpp
void applySteeringCommand(float steering) {
    if (steering < -1.0f) steering = -1.0f;
    if (steering > 1.0f) steering = 1.0f;

    float leftBrake = (steering < 0.0f) ? fabsf(steering) : 0.0f;
    float rightBrake = (steering > 0.0f) ? steering : 0.0f;

    float lAng = SERVO_NEUTRAL_ANGLE + leftBrake * (SERVO_MAX_BRAKE_ANGLE - SERVO_NEUTRAL_ANGLE);
    float rAng = SERVO_NEUTRAL_ANGLE + rightBrake * (SERVO_MAX_BRAKE_ANGLE - SERVO_NEUTRAL_ANGLE);

    char buf[32];
    snprintf(buf, sizeof(buf), "STEER %+.2f", steering);
    setServoAngles(lAng, rAng, buf);
}

void jogUnwindBoth(uint32_t durationMs = 1500) {
    Serial.println("\n>>> Jogging BOTH Servos UNWIND (180 deg / 2400 us) for 1.5s... <<<");
    setServoAngles(SERVO_MAX_UNWIND_ANGLE, SERVO_MAX_UNWIND_ANGLE, "JOG UNWIND");
    delay(durationMs);
    setServoAngles(SERVO_NEUTRAL_ANGLE, SERVO_NEUTRAL_ANGLE, "JOG COMPLETE -> NEUTRAL");
    Serial.println(">>> Unwind Jog Complete -> Stopped at Neutral (1500 us) <<<\n");
}

void jogWindBoth(uint32_t durationMs = 1500) {
    Serial.println("\n>>> Jogging BOTH Servos WIND / BRAKE (0 deg / 500 us) for 1.5s... <<<");
    setServoAngles(SERVO_MAX_BRAKE_ANGLE, SERVO_MAX_BRAKE_ANGLE, "JOG WIND");
    delay(durationMs);
    setServoAngles(SERVO_NEUTRAL_ANGLE, SERVO_NEUTRAL_ANGLE, "JOG COMPLETE -> NEUTRAL");
    Serial.println(">>> Wind Jog Complete -> Stopped at Neutral (1500 us) <<<\n");
}

void runAutomatedSweep() {
    Serial.println("\n>>> Starting Automated ES3004 Parafoil Steering Sweep <<<");

    Serial.println("1. Neutral (0% brake)...");
    applySteeringCommand(0.0f);
    delay(1000);

    Serial.println("2. Sweeping to Full LEFT Turn (steering = -1.0)...");
    for (int i = 0; i <= 10; i++) {
        applySteeringCommand(-0.1f * i);
        delay(150);
    }
    delay(800);

    Serial.println("3. Returning to Neutral...");
    for (int i = 10; i >= 0; i--) {
        applySteeringCommand(-0.1f * i);
        delay(100);
    }
    delay(600);

    Serial.println("4. Sweeping to Full RIGHT Turn (steering = +1.0)...");
    for (int i = 0; i <= 10; i++) {
        applySteeringCommand(0.1f * i);
        delay(150);
    }
    delay(800);

    Serial.println("5. Returning to Neutral...");
    for (int i = 10; i >= 0; i--) {
        applySteeringCommand(0.1f * i);
        delay(100);
    }
    delay(600);

    Serial.println("6. Sweeping to Full UNWIND (180 deg / 2400 us)...");
    for (int i = 0; i <= 10; i++) {
        float ang = 90.0f + 9.0f * i;
        setServoAngles(ang, ang, "SWEEP UNWIND");
        delay(100);
    }
    delay(800);

    Serial.println("7. Returning to Neutral (90 deg / 1500 us)...");
    for (int i = 10; i >= 0; i--) {
        float ang = 90.0f + 9.0f * i;
        setServoAngles(ang, ang, "SWEEP NEUTRAL");
        delay(100);
    }

    Serial.println(">>> Sweep Complete (Neutral 90 deg) <<<\n");
}

void printMenu() {
    Serial.println("\n=======================================================");
    Serial.println("  CAN-7USAT Dual Emax ES3004 Parafoil Servo Bench Test");
    Serial.println("=======================================================");
    Serial.println("  Pin 3 -> Left  ES3004 Servo (0 deg Brake/Wind, 90 deg Neutral, 180 deg Unwind)");
    Serial.println("  Pin 2 -> Right ES3004 Servo (0 deg Brake/Wind, 90 deg Neutral, 180 deg Unwind)");
    Serial.println("-------------------------------------------------------");
    Serial.println("  Serial Commands (Send via Serial Monitor @ 115200):");
    Serial.println("    'n' or '0' : Neutral Glide      (Left=90 deg,  Right=90 deg  -> 1500 us)");
    Serial.println("    'l'        : Full LEFT Turn     (Left=0 deg,   Right=90 deg  -> Steer -1.0)");
    Serial.println("    'r'        : Full RIGHT Turn    (Left=90 deg,  Right=0 deg   -> Steer +1.0)");
    Serial.println("    '1'        : Half LEFT Turn     (Left=45 deg,  Right=90 deg  -> Steer -0.5)");
    Serial.println("    '2'        : Half RIGHT Turn    (Left=90 deg,  Right=45 deg  -> Steer +0.5)");
    Serial.println("    'b'        : Full Both BRAKES   (Left=0 deg,   Right=0 deg   -> 500 us)");
    Serial.println("    'u'        : Full Both UNWIND   (Left=180 deg, Right=180 deg -> 2400 us)");
    Serial.println("    '3'        : Unwind LEFT only   (Left=180 deg, Right=90 deg)");
    Serial.println("    '4'        : Unwind RIGHT only  (Left=90 deg,  Right=180 deg)");
    Serial.println("    'j'        : Jog UNWIND (1.5s @ 180 deg, then auto-returns to 90 deg neutral)");
    Serial.println("    'k'        : Jog WIND   (1.5s @ 0 deg,   then auto-returns to 90 deg neutral)");
    Serial.println("    'w'        : Run Automated Full Steering & Unwind Sweep");
    Serial.println("    'd'        : DETACH Servos (0 mA Launch Pad Standby Mode)");
    Serial.println("    'a'        : ATTACH Servos & Center at Neutral (90 deg)");
    Serial.println("    '?'        : Print this menu and current servo status");
    Serial.println("=======================================================\n");
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    analogWriteResolution(16);
    analogWriteFrequency(SERVO_LEFT_PIN, 50.0f);
    analogWriteFrequency(SERVO_RIGHT_PIN, 50.0f);

    printMenu();
    // Attach and center at neutral on startup for immediate bench verification
    setServoAngles(SERVO_NEUTRAL_ANGLE, SERVO_NEUTRAL_ANGLE, "INIT NEUTRAL");
}

void loop() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();

        if (cmd == 'n' || cmd == 'N' || cmd == '0') {
            applySteeringCommand(0.0f);
        } else if (cmd == 'l' || cmd == 'L') {
            applySteeringCommand(-1.0f);
        } else if (cmd == 'r' || cmd == 'R') {
            applySteeringCommand(1.0f);
        } else if (cmd == '1') {
            applySteeringCommand(-0.5f);
        } else if (cmd == '2') {
            applySteeringCommand(0.5f);
        } else if (cmd == 'b' || cmd == 'B') {
            setServoAngles(SERVO_MAX_BRAKE_ANGLE, SERVO_MAX_BRAKE_ANGLE, "FULL FLARE / BOTH BRAKES (WIND)");
        } else if (cmd == 'u' || cmd == 'U') {
            setServoAngles(SERVO_MAX_UNWIND_ANGLE, SERVO_MAX_UNWIND_ANGLE, "FULL UNWIND / BOTH SIDES");
        } else if (cmd == '3') {
            setServoAngles(SERVO_MAX_UNWIND_ANGLE, SERVO_NEUTRAL_ANGLE, "UNWIND LEFT ONLY");
        } else if (cmd == '4') {
            setServoAngles(SERVO_NEUTRAL_ANGLE, SERVO_MAX_UNWIND_ANGLE, "UNWIND RIGHT ONLY");
        } else if (cmd == 'j' || cmd == 'J') {
            jogUnwindBoth(1500);
        } else if (cmd == 'k' || cmd == 'K') {
            jogWindBoth(1500);
        } else if (cmd == 'w' || cmd == 'W') {
            runAutomatedSweep();
        } else if (cmd == 'd' || cmd == 'D') {
            writeHardwarePwmUs(SERVO_LEFT_PIN, 0);
            writeHardwarePwmUs(SERVO_RIGHT_PIN, 0);
            servosAttached = false;
            Serial.println("[STANDBY] ES3004 Servos DETACHED (0 mA current draw).");
        } else if (cmd == 'a' || cmd == 'A') {
            setServoAngles(SERVO_NEUTRAL_ANGLE, SERVO_NEUTRAL_ANGLE, "ATTACH NEUTRAL");
        } else if (cmd == '?') {
            printMenu();
            Serial.print("Status -> Attached: ");
            Serial.print(servosAttached ? "YES" : "NO (0 mA)");
            Serial.print(" | Left: ");
            Serial.print(leftAngle, 1);
            Serial.print(" deg (");
            Serial.print(leftPulseUs);
            Serial.print(" us) | Right: ");
            Serial.print(rightAngle, 1);
            Serial.print(" deg (");
            Serial.print(rightPulseUs);
            Serial.println(" us)");
        }
    }
}
