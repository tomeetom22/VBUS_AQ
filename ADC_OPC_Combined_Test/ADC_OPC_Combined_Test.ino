/*
 * Combined ADS1115 and OPC-N3 test for XIAO RP2040.
 * Install Adafruit ADS1X15, Adafruit BusIO, and the OPCN3 library.
 * Serial Monitor: 115200 baud. One CSV measurement row every 10 seconds.
 *
 * ADS1115: 3V3, GND, SDA=D0, SCL=D1; default address 0x48.
 * Keep analog inputs between GND and 3V3.
 * OPC-N3: external regulated 5V, shared GND, CS=D7, SCK=D8,
 * MISO=D9, MOSI=D10. SPI logic is 3.3V.
 */
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
#include <OPCN3.h>
#include <ModBusRTUSlave.h>

const uint8_t ADS_ADDRESS = 0x48;
const uint8_t OPC_CS = D7;
const unsigned long READ_INTERVAL_MS = 10000;
Adafruit_ADS1115 ads;
OPCN3 opc(OPC_CS);
String opcSerial;
unsigned long lastReadMs = 0;
bool ready = false;

void setup() {
  digitalWrite(OPC_CS, HIGH);
  pinMode(OPC_CS, OUTPUT);
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 5000) delay(10);

  if (!Wire.setSDA(D0) || !Wire.setSCL(D1)) {
    Serial.println("# ERROR: failed to select I2C pins; reset to retry.");
    return;
  }
  Wire.begin();
  if (!ads.begin(ADS_ADDRESS, &Wire)) {
    Serial.println("# ERROR: ADS1115 not found at 0x48; check wiring and reset.");
    return;
  }
  ads.setGain(GAIN_ONE);

  SPI.begin();
  if (!opc.initialize()) {
    Serial.print("# ERROR: OPC initialization failed; last SPI response=0x");
    Serial.println(opc.lastResponse(), HEX);
    return;
  }
  opcSerial = opc.getSerialNumber();
  opcSerial.trim();
  Serial.println("# CSV: scan_interval_ms,ADC_A0_V,ADC_A1_V,ADC_A2_V,ADC_A3_V,OPC_serial,<HistogramData.toString() fields>");
  lastReadMs = millis();
  ready = true;
}

void loop() {
  if (!ready) { delay(100); return; }
  unsigned long now = millis();
  unsigned long scanIntervalMs = now - lastReadMs;
  if (scanIntervalMs < READ_INTERVAL_MS) return;
  lastReadMs = now;

  HistogramData hist = opc.readHistogramData();
  if (!hist.valid) {
    Serial.print("# ERROR: invalid OPC histogram; last SPI response=0x");
    Serial.println(opc.lastResponse(), HEX);
    return;
  }
  Serial.print(scanIntervalMs);
  for (uint8_t channel = 0; channel < 4; channel++) {
    int16_t raw = ads.readADC_SingleEnded(channel);
    Serial.print(',');
    Serial.print(ads.computeVolts(raw), 4);
  }
  Serial.print(',');
  Serial.print(opcSerial);
  Serial.print(',');
  Serial.println(hist.toString());
}
