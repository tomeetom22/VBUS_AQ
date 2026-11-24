/* Diagnostic master for RS485_Slave_Test. RAK4631 + RAK5802.
 * Uses the known-working raw write, then checks the Modbus response.
 * RS485 9600 8N1; USB Serial Monitor 115200. Not production firmware.
 */
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>

const uint8_t request[] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x79, 0x31, 0xE8};
const unsigned long POLL_MS = 5000;
const unsigned long TIMEOUT_MS = 2000;
const unsigned long FRAME_GAP_MS = 10;
uint8_t reply[256];
unsigned long lastPoll = 0;

uint16_t crc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

void pollNode() {
  // Discard stale bytes, including any local echo of the diagnostic preamble.
  while (Serial1.available()) Serial1.read();
  Serial1.write((uint8_t)0x00);
  Serial1.flush();
  delay(30);
  while (Serial1.available()) Serial1.read();
  Serial1.write(request, sizeof(request));
  Serial1.flush();

  size_t count = 0;
  bool overflow = false;
  unsigned long start = millis(), lastByte = start;
  while (millis() - start < TIMEOUT_MS) {
    while (Serial1.available()) {
      int value = Serial1.read();
      if (count < sizeof(reply)) reply[count++] = (uint8_t)value;
      else overflow = true;
      lastByte = millis();
    }
    if (count && millis() - lastByte >= FRAME_GAP_MS) break;
  }

  Serial.print("Received "); Serial.print(count); Serial.println(" bytes");
  if (overflow) { Serial.println("ERROR: response buffer overflow"); return; }
  if (count == 0) { Serial.println("ERROR: response timeout"); return; }
  // Full response: id, function, byte count, 121 registers, CRC.
  if (count != 247 || reply[0] != 1 || reply[1] != 4 || reply[2] != 242) {
    Serial.print("ERROR: unexpected frame; first bytes:");
    for (size_t i = 0; i < count && i < 16; ++i) {
      Serial.print(' '); if (reply[i] < 16) Serial.print('0');
      Serial.print(reply[i], HEX);
    }
    Serial.println();
    return;
  }
  uint16_t expected = (uint16_t)reply[count - 2] | ((uint16_t)reply[count - 1] << 8);
  if (crc16(reply, count - 2) != expected) { Serial.println("ERROR: response CRC mismatch"); return; }
  uint16_t length = ((uint16_t)reply[3] << 8) | reply[4];
  if (length > 240) { Serial.println("ERROR: invalid message length"); return; }
  Serial.print("Node 1: ");
  for (uint16_t i = 0; i < length; ++i) Serial.write(reply[5 + i]);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 5000) delay(10);
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, HIGH);
  delay(500);
  Serial1.begin(9600, SERIAL_8N1);
  Serial.println("Raw Modbus request/response diagnostic");
  lastPoll = millis() - POLL_MS;
}

void loop() {
  if (millis() - lastPoll < POLL_MS) return;
  lastPoll = millis();
  pollNode();
}
