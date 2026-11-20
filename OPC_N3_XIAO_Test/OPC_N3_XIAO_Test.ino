/*
 * Alphasense OPC-N3 test for Seeed Studio XIAO RP2040.
 * Based on OPC_N3_DataLogger_V1 by Alexei Perelet, University of Utah,
 * Environmental Fluid Dynamics Lab (AQ_Sense/reference).
 * Requires the OPC-N3 Library (OPCN3.h); Serial Monitor: 115200 baud.
 *
 * OPC pin   Function     XIAO pin     RP2040 GPIO
 *    1      +5 V         External regulated 5 V
 *    2      SCK          D8           GPIO2
 *    3      SDO/MISO     D9           GPIO4
 *    4      SDI/MOSI     D10          GPIO3
 *    5      /SS          D7           GPIO1
 *    6      GND          GND (also connect external supply ground)
 *
 * SPI uses 3.3 V logic. Power the OPC from 5 V, not the XIAO 3V3 pin.
 * Facing the upright sensor's connector, left to right is pins 6..1.
 * XIAO pin names and GPIO numbers are different: use D7, not literal 7.
 */

#include <Arduino.h>
#include <OPCN3.h>
#include <SPI.h>

static const uint8_t OPCN3_CS = D7;
static const unsigned long READ_INTERVAL_MS = 10000;

// Use the same library and calls as the reference logger. The installed
// OPCN3 library handles its own SPI mode-1 transactions at 500 kHz.
OPCN3 myOPCN3(OPCN3_CS);
String OPCSerial;
unsigned long tmpTime = 0;
bool opcReady = false;

void setup() {
  digitalWrite(OPCN3_CS, HIGH);
  pinMode(OPCN3_CS, OUTPUT);

  Serial.begin(115200);
  unsigned long serialStart = millis();
  while (!Serial && millis() - serialStart < 5000) delay(10);
  delay(1000);

  Serial.println("# XIAO RP2040 / OPC-N3 reference logger test");
  Serial.println("# CS=D7, SCK=D8, MISO=D9, MOSI=D10");
  Serial.println("# OPC needs external 5 V and shared ground; SPI logic is 3.3 V.");
  Serial.println("# Initializing OPC using OPCN3.initialize()...");

  // The board variant supplies the default XIAO hardware SPI pins.
  SPI.begin();
  if (!myOPCN3.initialize()) {
    Serial.print("# ERROR: OPC initialization failed; last SPI response=0x");
    Serial.println(myOPCN3.lastResponse(), HEX);
    Serial.println("# Check wiring and power; power-cycle the OPC and reset the XIAO to retry.");
    return;
  }

  OPCSerial = myOPCN3.getSerialNumber();
  OPCSerial.trim();
  Serial.print("# OPC serial number: ");
  Serial.println(OPCSerial);
  Serial.println("# CSV rows: elapsed_ms,OPC_serial,<HistogramData.toString()>");
  Serial.println("# Histogram fields and their order match the reference logger's SD output.");

  tmpTime = millis();
  opcReady = true;
}

void loop() {
  if (!opcReady) {
    delay(100);
    return;
  }

  unsigned long tmpTime_0 = millis();
  if (tmpTime_0 - tmpTime > READ_INTERVAL_MS) {
    // Keep the reference's one-second scheduling and histogram library call.
    tmpTime = millis();
    HistogramData hist = myOPCN3.readHistogramData();

    // The installed library exposes a validity flag for communication/CRC
    // failures. Avoid emitting failed reads as measurement rows.
    if (!hist.valid) {
      Serial.print("# ERROR: invalid histogram (SPI read or checksum failure); last SPI response=0x");
      Serial.println(myOPCN3.lastResponse(), HEX);
      return;
    }

    // Same serial number and histogram serialization as writeToCard() in
    // the reference; elapsed milliseconds substitute for the missing RTC.
    Serial.print(OPCSerial);
    Serial.print(",");
    Serial.println(hist.toString());
  }
}

