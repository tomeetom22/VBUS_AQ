#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <string.h>

struct OpcHistogram {
  uint16_t bins[24];
  float binsPerMl[24];
  uint8_t bin1Mtof;
  uint8_t bin3Mtof;
  uint8_t bin5Mtof;
  uint8_t bin7Mtof;
  float samplingPeriodS;
  float sampleFlowRate;
  float temperatureC;
  float humidityPct;
  float pm1;
  float pm25;
  float pm10;
  uint16_t rejectGlitch;
  uint16_t rejectLongTof;
  uint16_t rejectRatio;
  uint16_t rejectOutOfRange;
  uint16_t fanRevCount;
  uint16_t laserStatus;
  uint16_t checksum;
  uint16_t calculatedChecksum;
  bool checksumOk;
};

enum OpcError : uint8_t {
  OPC_ERROR_NONE = 0,
  OPC_ERROR_UNEXPECTED_RESPONSE,
  OPC_ERROR_READY_TIMEOUT,
  OPC_ERROR_CHECKSUM
};

class OPCDevice {
public:
  OPCDevice(uint8_t cs):
      csPin(cs), lastStatusResponse(0x00), lastErrorCode(OPC_ERROR_NONE) {}

  void begin() {
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    // Keep the first OPC command separate from any earlier SPI activity.
    delay(OPC_INTER_COMMAND_MS);
  }

  bool readHistogram(OpcHistogram &hist) {
    const uint8_t CMD_READ_HISTOGRAM = 0x30;
    uint8_t raw[86];
    if (!readBytes(CMD_READ_HISTOGRAM, raw, sizeof(raw))) return false;
    if (!decodeHistogram(raw, hist)) {
      lastErrorCode = OPC_ERROR_CHECKSUM;
      return false;
    }
    return true;
  }

  bool turnOn() {
    const uint8_t CMD_WRITE_POWER_STATE = 0x03;
    // N3 power options: laser switch ON = 0x07, fan pot ON = 0x03.
    // Each option must be sent as a separate 0x03 command sequence.
    if (!writeByte(CMD_WRITE_POWER_STATE, 0x07)) return false;
    if (!writeByte(CMD_WRITE_POWER_STATE, 0x03)) return false;
    // Alphasense specifies >600 ms after fan-on (and <2 s).
    delay(1000);
    return true;
  }

  uint8_t lastResponse() const {
    return lastStatusResponse;
  }

  OpcError lastError() const {
    return lastErrorCode;
  }

  const char *lastErrorText() const {
    switch (lastErrorCode) {
      case OPC_ERROR_UNEXPECTED_RESPONSE:
        return "unexpected SPI response";
      case OPC_ERROR_READY_TIMEOUT:
        return "ready timeout";
      case OPC_ERROR_CHECKSUM:
        return "histogram checksum mismatch";
      default:
        return "none";
    }
  }

private:
  static const uint16_t OPC_INTER_BYTE_US = 20;
  static const uint16_t OPC_COMMAND_POLL_MS = 20;
  static const uint16_t OPC_INTER_COMMAND_MS = 20;
  static const uint16_t OPC_ERROR_RECOVERY_MS = 2500;

  uint8_t csPin;
  uint8_t lastStatusResponse;
  OpcError lastErrorCode;

  uint8_t opcTransferByte(uint8_t value) {
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1));
    digitalWrite(csPin, LOW);
    delayMicroseconds(2);
    uint8_t resp = SPI.transfer(value);
    delayMicroseconds(2);
    digitalWrite(csPin, HIGH);
    SPI.endTransaction();
    return resp;
  }

  bool waitForReady(uint8_t command) {
    const uint8_t OPC_READY = 0xF3;
    const uint8_t OPC_BUSY = 0x31;
    uint8_t response = OPC_BUSY;

    lastErrorCode = OPC_ERROR_NONE;
    for (uint8_t attempt = 0; attempt < 26; attempt++) {
      // The N3 requires the original command byte for every ready poll.
      response = opcTransferByte(command);
      lastStatusResponse = response;

      if (response == OPC_READY) {
        // Data may follow after >10 us once the OPC reports ready.
        delayMicroseconds(OPC_INTER_BYTE_US);
        return true;
      }
      if (response == OPC_BUSY) {
        // Command-byte polling must be separated by >10 ms.
        delay(OPC_COMMAND_POLL_MS);
        continue;
      }

      // Any response other than BUSY/READY is a protocol error. Stop clocking
      // the OPC and give its SPI buffer the manufacturer-required >2 s reset.
      lastErrorCode = OPC_ERROR_UNEXPECTED_RESPONSE;
      delay(OPC_ERROR_RECOVERY_MS);
      return false;
    }

    lastErrorCode = OPC_ERROR_READY_TIMEOUT;
    delay(OPC_ERROR_RECOVERY_MS);
    return false;
  }

  bool readBytes(uint8_t command, uint8_t *buffer, size_t length) {
    if (!waitForReady(command)) return false;
    for (size_t i = 0; i < length; i++) {
      buffer[i] = opcTransferByte(command);
      if (i + 1 < length) delayMicroseconds(OPC_INTER_BYTE_US);
    }
    // Separate this completed communication from the next OPC command.
    delay(OPC_INTER_COMMAND_MS);
    return true;
  }

  bool writeByte(uint8_t command, uint8_t value) {
    if (!waitForReady(command)) return false;
    opcTransferByte(value);
    // Each power option is a separate communication sequence.
    delay(OPC_INTER_COMMAND_MS);
    return true;
  }

  static uint16_t le16(const uint8_t *buffer, size_t offset) {
    return (uint16_t)buffer[offset] | ((uint16_t)buffer[offset + 1] << 8);
  }

  static float leFloat(const uint8_t *buffer, size_t offset) {
    float v;
    memcpy(&v, buffer + offset, sizeof(v));
    return v;
  }

  static uint16_t opcCrc16(const uint8_t *buffer, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
      crc ^= buffer[i];
      for (uint8_t bit = 0; bit < 8; bit++) {
        if (crc & 0x0001) {
          crc >>= 1;
          crc ^= 0xA001;
        } else {
          crc >>= 1;
        }
      }
    }
    return crc;
  }

  bool decodeHistogram(const uint8_t *buffer, OpcHistogram &histogram) {
    for (uint8_t i = 0; i < 24; i++) {
      histogram.bins[i] = le16(buffer, i * 2);
      histogram.binsPerMl[i] = 0.0f;
    }
    histogram.bin1Mtof = buffer[48];
    histogram.bin3Mtof = buffer[49];
    histogram.bin5Mtof = buffer[50];
    histogram.bin7Mtof = buffer[51];
    histogram.samplingPeriodS = le16(buffer, 52) / 100.0f;
    histogram.sampleFlowRate = le16(buffer, 54) / 100.0f;
    histogram.temperatureC = -45.0f + (175.0f * le16(buffer, 56) / 65535.0f);
    histogram.humidityPct = 100.0f * le16(buffer, 58) / 65535.0f;
    histogram.pm1 = leFloat(buffer, 60);
    histogram.pm25 = leFloat(buffer, 64);
    histogram.pm10 = leFloat(buffer, 68);
    histogram.rejectGlitch = le16(buffer, 72);
    histogram.rejectLongTof = le16(buffer, 74);
    histogram.rejectRatio = le16(buffer, 76);
    histogram.rejectOutOfRange = le16(buffer, 78);
    histogram.fanRevCount = le16(buffer, 80);
    histogram.laserStatus = le16(buffer, 82);
    histogram.checksum = le16(buffer, 84);
    histogram.calculatedChecksum = opcCrc16(buffer, 84);
    histogram.checksumOk = (histogram.checksum == histogram.calculatedChecksum);

    float mlPerPeriod = histogram.sampleFlowRate * histogram.samplingPeriodS;
    if (mlPerPeriod > 0.0f) {
      for (uint8_t i = 0; i < 24; i++) {
        histogram.binsPerMl[i] = histogram.bins[i] / mlPerPeriod;
      }
    }
    return histogram.checksumOk;
  }
};
