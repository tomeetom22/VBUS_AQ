/* Receive-only RS485 diagnostic for XIAO RP2040 + V1.0 breakout.
 * Upload to the XIAO; leave the RAK master test running.
 * USB monitor: 115200. RS485: 9600 8N1, RX D5, TX D4, DE D2.
 * This sketch does not reply, so the RAK will still report timeouts.
 */
#include <Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial rs485(D5, D4);
unsigned long lastStatus = 0;
unsigned long byteCount = 0;
unsigned long lastByteMs = 0;
uint8_t burst[256];
uint16_t burstLength = 0;
bool overflow = false;

void setup() {
  pinMode(D2, OUTPUT);
  digitalWrite(D2, LOW); // Receive mode.
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 5000) delay(10);
  rs485.begin(9600, SERIAL_8N1);
  Serial.println("RS485 receive diagnostic ready; listening at 9600 8N1.");
}

void loop() {
  while (rs485.available()) {
    uint8_t value = rs485.read();
    ++byteCount;
    lastByteMs = millis();
    if (burstLength < sizeof(burst)) burst[burstLength++] = value;
    else overflow = true;
  }
  if (burstLength && millis() - lastByteMs >= 20) {
    Serial.print("Burst: ");
    for (uint16_t i = 0; i < burstLength; ++i) {
      if (burst[i] < 16) Serial.print('0');
      Serial.print(burst[i], HEX);
      Serial.print(' ');
    }
    if (overflow) Serial.print("(buffer overflow)");
    Serial.println();
    burstLength = 0;
    overflow = false;
  }
  if (millis() - lastStatus >= 5000) {
    lastStatus = millis();
    Serial.print("Listening; bytes received: ");
    Serial.println(byteCount);
  }
}
