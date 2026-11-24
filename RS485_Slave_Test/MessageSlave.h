#pragma once
#include <Arduino.h>
#include <ModbusRTUSlave.h>

// Input registers: length in bytes, then 120 registers of packed text.
// The Modbus unit address identifies the node.
class MessageSlave {
public:
  static const uint16_t MAX_BYTES = 240;
  static const uint16_t REGISTER_COUNT = 121;

  MessageSlave(Stream &serial, int directionPin) : modbus(serial, directionPin) {}
  void begin(uint8_t unitId, unsigned long baud) {
    modbus.configureInputRegisters(registers, REGISTER_COUNT);
    modbus.begin(unitId, baud, SERIAL_8N1);
  }
  void poll() { modbus.poll(); }

  // Returns false and leaves the old message intact if it does not fit.
  bool setMessage(const String &message) {
    if (message.length() > MAX_BYTES) return false;
    for (uint16_t i = 0; i < REGISTER_COUNT; ++i) registers[i] = 0;
    registers[0] = message.length();
    for (uint16_t i = 0; i < message.length(); ++i) {
      uint8_t byte = static_cast<uint8_t>(message[i]);
      if ((i & 1) == 0) registers[1 + i / 2] = uint16_t(byte) << 8;
      else registers[1 + i / 2] |= byte;
    }
    return true;
  }
private:
  uint16_t registers[REGISTER_COUNT] = {};
  ModbusRTUSlave modbus;
};
