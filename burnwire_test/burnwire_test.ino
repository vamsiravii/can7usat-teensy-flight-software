#include <Arduino.h>

// ============================================================================
// CAN-7USAT Team 017 | Standalone XY-MOS Burnwire & Buzzer Hardware Test
// Board: Teensy 4.1
// Burnwire XY-MOS MOSFET #1 Trigger: Pin 4
// Buzzer   XY-MOS MOSFET #2 Trigger: Pin 5
// ============================================================================

#define BURNWIRE_PIN          4
#define BUZZER_PIN            5
#define BURNWIRE_DURATION_MS  30000UL  // 30.0s full deployment burn duration
#define SHORT_PULSE_MS        500UL    // 0.5s quick LED / MOSFET gate check

bool burnwireActive = false;
uint32_t burnwireStartMs = 0;
uint32_t activeBurnDurationMs = BURNWIRE_DURATION_MS;
uint32_t lastPrintMs = 0;

bool buzzerBeaconEnabled = false;
bool buzzerPinState = false;
uint32_t lastBuzzerToggleMs = 0;

void printMenu() {
    Serial.println("\n=======================================================");
    Serial.println("  CAN-7USAT XY-MOS Burnwire & Buzzer Bench Test");
    Serial.println("=======================================================");
    Serial.println("  Pin 4 -> Burnwire XY-MOS MOSFET #1 (Active HIGH)");
    Serial.println("  Pin 5 -> Buzzer   XY-MOS MOSFET #2 (Active HIGH)");
    Serial.println("-------------------------------------------------------");
    Serial.println("  Serial Commands (Send via Serial Monitor @ 115200):");
    Serial.println("    'f' : FIRE Burnwire for 30.0 seconds (Full Deploy)");
    Serial.println("    't' : TEST Burnwire for 0.5 seconds (Gate/LED Check)");
    Serial.println("    'b' : Toggle Recovery Buzzer Beacon (500ms pulse)");
    Serial.println("    's' : EMERGENCY STOP (Force both XY-MOS LOW)");
    Serial.println("    '?' : Print this menu and current pin states");
    Serial.println("=======================================================\n");
}

void stopAll() {
    burnwireActive = false;
    buzzerBeaconEnabled = false;
    buzzerPinState = false;
    digitalWrite(BURNWIRE_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("[SAFE] Both XY-MOS MOSFET outputs forced LOW (OFF).");
}

void setup() {
    // Immediately configure pins LOW before anything else for safety
    pinMode(BURNWIRE_PIN, OUTPUT);
    digitalWrite(BURNWIRE_PIN, LOW);

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    Serial.begin(115200);
    delay(1000);

    printMenu();
    stopAll();
}

void loop() {
    uint32_t now = millis();

    // 1. Handle Serial Commands
    if (Serial.available() > 0) {
        char cmd = Serial.read();

        if (cmd == 'f' || cmd == 'F') {
            burnwireActive = true;
            activeBurnDurationMs = BURNWIRE_DURATION_MS;
            burnwireStartMs = now;
            lastPrintMs = now;
            digitalWrite(BURNWIRE_PIN, HIGH);
            Serial.println("[BURNWIRE] XY-MOS Pin 4 -> HIGH (Firing for 30000 ms)...");
        } else if (cmd == 't' || cmd == 'T') {
            burnwireActive = true;
            activeBurnDurationMs = SHORT_PULSE_MS;
            burnwireStartMs = now;
            lastPrintMs = now;
            digitalWrite(BURNWIRE_PIN, HIGH);
            Serial.println("[BURNWIRE] XY-MOS Pin 4 -> HIGH (Short 500 ms Gate/LED Test)...");
        } else if (cmd == 'b' || cmd == 'B') {
            buzzerBeaconEnabled = !buzzerBeaconEnabled;
            if (!buzzerBeaconEnabled) {
                buzzerPinState = false;
                digitalWrite(BUZZER_PIN, LOW);
                Serial.println("[BUZZER] XY-MOS Pin 5 -> LOW (Beacon OFF)");
            } else {
                lastBuzzerToggleMs = now;
                buzzerPinState = true;
                digitalWrite(BUZZER_PIN, HIGH);
                Serial.println("[BUZZER] XY-MOS Pin 5 -> Beacon ENABLED (500ms ON / 500ms OFF)");
            }
        } else if (cmd == 's' || cmd == 'S' || cmd == ' ') {
            stopAll();
        } else if (cmd == '?') {
            printMenu();
            Serial.print("Status -> Burnwire Pin 4: ");
            Serial.print(burnwireActive ? "HIGH (ACTIVE)" : "LOW (OFF)");
            Serial.print(" | Buzzer Pin 5: ");
            Serial.println(buzzerBeaconEnabled ? "BEACON ACTIVE" : "LOW (OFF)");
        }
    }

    // 2. Non-Blocking Burnwire Auto-Shutoff Timer
    if (burnwireActive) {
        uint32_t elapsed = now - burnwireStartMs;

        if (elapsed >= activeBurnDurationMs) {
            digitalWrite(BURNWIRE_PIN, LOW);
            burnwireActive = false;
            Serial.println("[BURNWIRE] Auto-shutoff complete. XY-MOS Pin 4 -> LOW (OFF).");
        } else if (now - lastPrintMs >= 250) {
            lastPrintMs = now;
            Serial.print("  Burnwire active... ");
            Serial.print(activeBurnDurationMs - elapsed);
            Serial.println(" ms remaining");
        }
    }

    // 3. Non-Blocking Buzzer XY-MOS Beacon (500ms ON / 500ms OFF)
    if (buzzerBeaconEnabled) {
        if (now - lastBuzzerToggleMs >= 500) {
            lastBuzzerToggleMs = now;
            buzzerPinState = !buzzerPinState;
            digitalWrite(BUZZER_PIN, buzzerPinState ? HIGH : LOW);
        }
    }
}
