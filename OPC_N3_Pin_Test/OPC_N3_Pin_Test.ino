#include <Arduino.h>
#include <SPI.h>

// Test one OPC connector at a time.
//   1 = OPC1 CS on WB_IO1 / GPIO17
//   2 = OPC2 CS on WB_IO2 / GPIO34
#define TEST_OPC 2

static const uint8_t SD_CS_PIN = WB_SPI_CS;
static const uint8_t OPC1_CS_PIN = WB_IO1;
static const uint8_t OPC2_CS_PIN = WB_IO2;
static const uint8_t OPC_CS_PIN =
#if TEST_OPC == 1
    OPC1_CS_PIN;
#else
    OPC2_CS_PIN;
#endif

static const uint8_t OPC_SCK_PIN = WB_SPI_CLK;
static const uint8_t OPC_MOSI_PIN = WB_SPI_MOSI;
static const uint8_t OPC_MISO_PIN = WB_SPI_MISO;
static const uint32_t OPC_SPI_HZ = 300000;

static SPISettings opcSpiSettings(OPC_SPI_HZ, MSBFIRST, SPI_MODE1);

static void printHex(uint8_t value) {
  if (value < 0x10) Serial.print('0');
  Serial.print(value, HEX);
}

static void printLevels(const char *label) {
  Serial.print(label);
  Serial.print(" CS=");
  Serial.print(digitalRead(OPC_CS_PIN));
  Serial.print(" SCK=");
  Serial.print(digitalRead(OPC_SCK_PIN));
  Serial.print(" MOSI=");
  Serial.print(digitalRead(OPC_MOSI_PIN));
  Serial.print(" MISO=");
  Serial.println(digitalRead(OPC_MISO_PIN));
}

void setup() {
  Serial.begin(115200);
  uint32_t serialStart = millis();
  while (!Serial && millis() - serialStart < 5000) delay(10);

  Serial.println("Beginning Test!");

  pinMode(OPC1_CS_PIN, OUTPUT);
  pinMode(OPC2_CS_PIN, OUTPUT);
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(OPC1_CS_PIN, HIGH);
  digitalWrite(OPC2_CS_PIN, HIGH);
  digitalWrite(SD_CS_PIN, HIGH);
  
  

  pinMode(OPC_SCK_PIN, OUTPUT);
  pinMode(OPC_MOSI_PIN, OUTPUT);
  pinMode(OPC_MISO_PIN, OUTPUT);

  digitalWrite(OPC_SCK_PIN, LOW);
  digitalWrite(OPC_MOSI_PIN, LOW);
  digitalWrite(OPC_MISO_PIN, LOW);

}

void loop() {
  // One-shot test.
  digitalWrite(OPC1_CS_PIN, HIGH);
  delay(1000);

}
