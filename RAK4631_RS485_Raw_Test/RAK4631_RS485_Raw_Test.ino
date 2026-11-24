/* Send a known Modbus RTU request without a Modbus library.
 * RAK4631 + RAK5802; USB monitor 115200; RS485 9600 8N1.
 * Pair with RS485_Receive_Diagnostic. This does not read replies.
 */
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>

// Unit 1, function 04, start 0, quantity 121, CRC (low byte first).
const uint8_t request[] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x79, 0x31, 0xE8};
unsigned long lastSend = 0;

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 5000) delay(10);
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, HIGH);
  delay(500);
  Serial1.begin(9600, SERIAL_8N1);
  Serial.println("RAK raw RS485 test: sending every 5 seconds");
  lastSend = millis() - 5000;
}

void loop() {
  if (millis() - lastSend < 5000) return;
  lastSend = millis();
  // Diagnostic only: wake the interface, then leave an idle gap before
  // the actual Modbus frame. Do not use this preamble in production.
  Serial1.write((uint8_t)0x00);
  Serial1.flush();
  delay(30);
  Serial1.write(request, sizeof(request));
  Serial1.flush();
  Serial.println("Sent wake byte, then: 01 04 00 00 00 79 31 E8");
}
