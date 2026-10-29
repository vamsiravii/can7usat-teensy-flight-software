# CAN-7USAT Team 017 | Resilient Lovelace Flight Software & GNC

This repository contains the complete Flight Software (FSW), FlatSat verification firmware, Software-in-the-Loop (SIL) simulation framework, and hardware actuator test benches for the **CAN-7USAT Team 017 Guided Parafoil Precision Landing Mission** (Teensy 4.1 flight computer).

---

## Repository Structure

```text
resilient-lovelace/
├── CAN7USAT_FSW/                # Canonical Teensy 4.1 Flight Software (GNC, State Machine, Telemetry)
├── CAN7USAT_FLATSAT/            # FlatSat flight firmware with interactive USB bench control & diagnostics
├── CAN7USAT_FSW_PERP/           # Perpendicular breakout guidance variant
├── sil_test/                    # SIL high-fidelity flight dynamics simulation & C++ unit test harness
│   ├── test_all.sh              # 1-click test runner (98 unit tests + 19 actuator tests + Monte Carlo)
│   ├── unit_tests.cpp           # C++ unit tests for EKF, geodesy, PID, hypsometry & FSM
│   └── test_integrated_fsw_actuators.cpp # 19-stage actuator and spool safety tests
├── burnwire_test/               # Standalone XY-MOS Nichrome burnwire & buzzer hardware test
├── servo_test/                  # Hardware FlexPWM dual ES3004 brake servo bench test
├── flywheel_test/               # MT2204 2300KV ESC / reaction wheel momentum bias test
├── groundstation/               # Ground station telemetry logging and parsing utilities
├── i2c_scanner/                 # Teensy 4.1 I2C bus scanner (Wire & Wire1)
├── lora_flasher_passthrough/    # UART passthrough flasher for Ai-Thinker Ra-08 LoRa module
├── CAN-7USAT-Precision-Landing/ # Python 6-DoF flight dynamics, wind modeling & trajectory optimizer
├── Ai-Thinker-LoRaWAN-Ra-08/    # Ai-Thinker Ra-08 (ASR6601CB) LoRa SDK and drivers
└── Parikshit - 2026-INSPACe-CAN-7USAT-017_CDR (1).pdf # Critical Design Review (CDR) documentation
```

---

## Hardware Architecture & Pin Map (Teensy 4.1)

| Subsystem | Hardware Component | Teensy 4.1 Pin | Interface / Notes |
| :--- | :--- | :--- | :--- |
| **Left Brake Servo** | EMAX ES3004 (Metal Gear) | **Pin 3** | FlexPWM4_2_B (500 µs - 1500 µs) |
| **Right Brake Servo** | EMAX ES3004 (Metal Gear) | **Pin 2** | FlexPWM4_2_A (500 µs - 1500 µs) |
| **Burnwire** | XY-MOS MOSFET #1 (32 SWG N80) | **Pin 4** | Active HIGH, 30.0s deployment pulse |
| **Recovery Buzzer** | XY-MOS MOSFET #2 (Active Piezo) | **Pin 5** | Active HIGH, landing beacon |
| **Flywheel ESC** | ReadyToSky 30A 2S ESC + MT2204 | **Pin 9** | Hardware PWM (1353 µs setpoint @ 6000 RPM) |
| **IMU / Heading** | Bosch BNO055 (NDOF Mode) | **Pins 18/19** | I2C (Wire) @ 100 kHz, 0x28 |
| **Altimeter** | MS5611 Barometric Sensor | **Pins 18/19** | I2C (Wire) @ 100 kHz, 0x77 |
| **CO2 Sensor** | Sensirion SCD40 NDIR | **Pins 18/19** | I2C (Wire) @ 100 kHz, 0x62 |
| **GNSS Receiver** | u-blox MAX-M10S (UBX Protocol) | **Serial1 (Pins 0/1)**| Hardware UART @ 115200 baud, 10 Hz |
| **Telemetry Comms**| Ai-Thinker Ra-08 LoRa (433 MHz) | **Serial8 (Pins 34/35)**| Hardware UART @ 115200 baud |

---

## Verification & SIL Testing

To compile and run the full 98-case unit test suite, 19-stage actuator safety harness, and 100-run Monte Carlo flight dynamics simulation:

```bash
cd sil_test
bash test_all.sh
```

---

## Compiling Flight Firmware

Using `arduino-cli`:

```bash
# Flight Software (FSW)
arduino-cli compile --fqbn teensy:avr:teensy41 CAN7USAT_FSW/CAN7USAT_FSW.ino

# FlatSat Firmware
arduino-cli compile --fqbn teensy:avr:teensy41 CAN7USAT_FLATSAT/CAN7USAT_FLATSAT.ino
```
