#include <Arduino.h>
#include <SoftwareSerial.h>
#include "MessageSlave.h"

// Set a different address on each XIAO: bottom=1, top=2.
const uint8_t NODE_ID = 1;
const unsigned long BAUD = 9600;
SoftwareSerial rs485(D5, D4); // RX, TX (PIO UART)
MessageSlave node(rs485, D2); // HIGH while transmitting
unsigned long lastUpdate = 0;

void setup() {
  Serial.begin(115200);
  rs485.begin(BAUD, SERIAL_8N1);
  node.begin(NODE_ID, BAUD);
  node.setMessage("Hello from XIAO");
}

void loop() {
  node.poll(); // Call frequently; the node replies only when addressed.
  if (millis() - lastUpdate >= 10000) {
    lastUpdate = millis();
    node.setMessage(String("uptime_ms=") + lastUpdate);
  }
}
