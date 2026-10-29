// Teensy 4.1 USB-to-UART0 Passthrough for Flashing Ra-08H (ASR6601)
// Wiring:
//   Teensy 3.3V -> Ra-08H 3.3V
//   Teensy GND  -> Ra-08H GND
//   Teensy Pin 1 (TX1) -> Ra-08H RXD
//   Teensy Pin 0 (RX1) -> Ra-08H TXD
//
// Steps:
//   1. Upload this sketch to Teensy 4.1, then CLOSE the Arduino Serial Monitor.
//   2. Put Ra-08H switch in BOOT position.
//   3. Hold down the tiny BOOT button on the Ra-08H while pressing & releasing RST (or unplug/replug 3.3V), then release BOOT.
//   4. Run the tremo_loader.py command in Terminal.

void setup() {
  Serial.begin(921600);
  Serial1.begin(921600); // ASR6601 ROM Bootloader default baud rate is 921600
}

void loop() {
  int n = Serial.available();
  if (n > 0) {
    uint8_t buf[256];
    if (n > 256) n = 256;
    Serial.readBytes((char*)buf, n);
    Serial1.write(buf, n);
  }

  int m = Serial1.available();
  if (m > 0) {
    uint8_t buf[256];
    if (m > 256) m = 256;
    Serial1.readBytes((char*)buf, m);
    Serial.write(buf, m);
  }
}
