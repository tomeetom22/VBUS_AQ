/* RAK4631 + RAK5802 master for RS485_Slave_Test.
 * USB Serial Monitor: 115200; RS485: 9600 8N1.
 * Install ModbusRTUMaster 2.x and its dependencies.
 */
#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // RAK4631 USB Serial implementation
#include "src/ModbusRTUMaster.h" // Project-local diagnostic copy.

const unsigned long BUS_BAUD = 9600;
const unsigned long POLL_INTERVAL_MS = 5000;
const uint8_t NODE_IDS[] = {1}; // Add 2 when the second XIAO is connected.
const uint16_t REGISTER_COUNT = 121;
const uint16_t MAX_BYTES = 240;
uint16_t registers[REGISTER_COUNT];
ModbusRTUMaster modbus(Serial1); // RAK5802 switches direction automatically.
unsigned long lastPoll = 0;

void readNode(uint8_t id) {
  // Diagnostic wake byte, separated from the request by an idle gap.
  Serial1.write((uint8_t)0x00);
  Serial1.flush();
  delay(30);
  uint8_t error = modbus.readInputRegisters(id, 0, registers, REGISTER_COUNT);
  if (error != 0) {
    Serial.print("Node "); Serial.print(id);
    Serial.print(": Modbus error "); Serial.println(error);
    return;
  }
  uint16_t length = registers[0];
  if (length > MAX_BYTES) {
    Serial.print("Node "); Serial.print(id);
    Serial.println(": invalid message length");
    return;
  }
  Serial.print("Node "); Serial.print(id); Serial.print(": ");
  for (uint16_t i = 0; i < length; ++i) {
    uint16_t word = registers[1 + i / 2];
    Serial.write(static_cast<uint8_t>((i & 1) ? word : word >> 8));
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 5000) delay(10);
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, HIGH); // Enable switched 3.3 V for the RAK5802.
  delay(500);
  Serial1.begin(BUS_BAUD, SERIAL_8N1);
  modbus.begin(BUS_BAUD, SERIAL_8N1);
  modbus.setTimeout(1000);
  Serial.println("RAK4631 Modbus message test");
  lastPoll = millis() - POLL_INTERVAL_MS;
}

void loop() {
  if (millis() - lastPoll < POLL_INTERVAL_MS) return;
  lastPoll = millis();
  for (uint8_t id : NODE_IDS) readNode(id);
}
