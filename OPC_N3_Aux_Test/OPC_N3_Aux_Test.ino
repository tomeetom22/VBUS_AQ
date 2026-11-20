#include <Arduino.h>
#include <SPI.h>

// Select which auxiliary-board OPC connector to test.
//   1 = OPC1 CS on WB_IO1 / GPIO17
//   2 = OPC2 CS on WB_IO2 / GPIO34
#define TEST_OPC 2

static const uint8_t OPC_CS_PIN =
#if TEST_OPC == 1
    WB_IO1;
#else
    WB_IO2;
#endif

static const uint32_t OPC_SPI_HZ = 500000;
static const uint8_t OPC_READY = 0xF3;
static const uint8_t OPC_BUSY = 0x31;
static const uint8_t CMD_WRITE_POWER_STATE = 0x03;
static const uint8_t CMD_READ_POWER_STATE = 0x13;
static const uint8_t CMD_READ_HISTOGRAM = 0x30;

static SPISettings opcSpiSettings(OPC_SPI_HZ, MSBFIRST, SPI_MODE1);

static void printHex(uint8_t value) {
  if (value < 0x10) Serial.print('0');
  Serial.print(value, HEX);
}

static uint8_t transferByte(uint8_t value, uint16_t delayUs = 10) {
  SPI.beginTransaction(opcSpiSettings);
  digitalWrite(OPC_CS_PIN, LOW);
  delay(10);
  uint8_t response = SPI.transfer(value);
  delayMicroseconds(10);
  digitalWrite(OPC_CS_PIN, HIGH);
  SPI.endTransaction();
  if (delayUs) delayMicroseconds(delayUs);
  return response;
}

static bool waitForReady(uint8_t command) {
  Serial.print("  command 0x");
  printHex(command);
  Serial.println();

  for (uint8_t attempt = 0; attempt < 30; attempt++) {
    uint8_t response = transferByte(command, 0);

    Serial.print("    response ");
    Serial.print(attempt + 1);
    Serial.print(": 0x");
    printHex(response);
    Serial.println();

    if (response == OPC_READY) return true;
    if (response != OPC_BUSY) {
      Serial.println("    Unexpected response; stopping this command.");
      return false;
    }
    delay(20);
  }

  Serial.println("    Timed out waiting for OPC ready.");
  return false;
}

static bool writeByte(uint8_t command, uint8_t value) {
  if (!waitForReady(command)) return false;

  uint8_t response = transferByte(value);
  Serial.print("    payload response: 0x");
  printHex(response);
  Serial.println();
  return true;
}

static bool readBytes(uint8_t command, uint8_t *buffer, size_t length) {
  if (!waitForReady(command)) return false;

  for (size_t i = 0; i < length; i++) {
    buffer[i] = transferByte(command);
  }
  return true;
}

static uint16_t readLe16(const uint8_t *buffer, size_t offset) {
  return (uint16_t)buffer[offset] |
         ((uint16_t)buffer[offset + 1] << 8);
}

static uint16_t opcCrc16(const uint8_t *buffer, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= buffer[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 1) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

static void readPowerState() {
  uint8_t state[6];
  Serial.println("Reading power state...");
  if (!readBytes(CMD_READ_POWER_STATE, state, sizeof(state))) {
    Serial.println("POWER STATE: failed");
    return;
  }

  Serial.print("POWER STATE: fan=");
  Serial.print(state[0]);
  Serial.print(" laser=");
  Serial.print(state[1]);
  Serial.print(" fanDAC=");
  Serial.print(state[2]);
  Serial.print(" laserDAC=");
  Serial.print(state[3]);
  Serial.print(" laserSwitch=");
  Serial.print(state[4]);
  Serial.print(" gain=");
  Serial.println(state[5]);
}

static void readHistogram() {
  uint8_t histogram[86];
  Serial.println("Reading one 86-byte histogram...");
  if (!readBytes(CMD_READ_HISTOGRAM, histogram, sizeof(histogram))) {
    Serial.println("HISTOGRAM: failed");
    return;
  }

  uint16_t receivedCrc = readLe16(histogram, 84);
  uint16_t calculatedCrc = opcCrc16(histogram, 84);

  Serial.print("HISTOGRAM: received CRC=0x");
  printHex(receivedCrc >> 8);
  printHex(receivedCrc & 0xFF);
  Serial.print(" calculated CRC=0x");
  printHex(calculatedCrc >> 8);
  printHex(calculatedCrc & 0xFF);
  Serial.println(receivedCrc == calculatedCrc ? " OK" : " BAD");

  Serial.print("  sample period raw: ");
  Serial.println(readLe16(histogram, 52));
  Serial.print("  flow raw: ");
  Serial.println(readLe16(histogram, 54));
}

void setup() {
  Serial.begin(115200);
  uint32_t serialStart = millis();
  while (!Serial && millis() - serialStart < 5000) delay(10);

  Serial.println();
  Serial.println("=====================================");
  Serial.println("OPC-N3 auxiliary-board SPI-only test");
  Serial.println("=====================================");
  Serial.print("Testing OPC");
  Serial.println(TEST_OPC);
  Serial.print("CS GPIO: ");
  Serial.println(OPC_CS_PIN);
  Serial.println("SPI: SCK=GPIO3, MOSI/SDI=GPIO30, MISO/SDO=GPIO29");
  Serial.println("No SD library or SD initialization is used.");
  Serial.println("OPC must have external 5 V and common ground.");

  pinMode(OPC_CS_PIN, OUTPUT);
  digitalWrite(OPC_CS_PIN, HIGH);

  SPI.setPins(WB_SPI_MISO, WB_SPI_CLK, WB_SPI_MOSI);
  SPI.begin();

  Serial.println("CS held high; waiting 3 seconds for OPC startup...");
  delay(3000);

  Serial.println("Checking OPC with documented histogram command 0x30...");
  uint8_t startupHistogram[86];
  if (!readBytes(CMD_READ_HISTOGRAM, startupHistogram, sizeof(startupHistogram))) {
    Serial.println("STATUS: no valid OPC response to command 0x30.");
    Serial.println("Check OPC pin order, CS routing, MISO/SDO, and common ground.");
    return;
  }
  Serial.println("STATUS: OPC responded with READY and returned a histogram.");

  Serial.println("Sending fan/laser startup commands...");
  bool laserCommandOk = writeByte(CMD_WRITE_POWER_STATE, 0x07);
  delayMicroseconds(2);
  bool fanCommandOk = writeByte(CMD_WRITE_POWER_STATE, 0x03);
  delay(1000);
  Serial.print("POWER COMMANDS: ");
  Serial.println(laserCommandOk && fanCommandOk ? "sent" : "failed");

  readPowerState();
  readHistogram();
  Serial.println("Test complete. Reset to repeat.");
}

void loop() {
  // This is intentionally a one-shot diagnostic. It does not touch the SD card.
}
