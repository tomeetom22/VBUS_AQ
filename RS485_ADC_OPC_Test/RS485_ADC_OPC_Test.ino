/* XIAO RP2040 ADC + OPC Modbus test. USB monitor: 115200.
 * ADS1115: SDA D0, SCL D1, 3V3, GND; inputs within 0-3.3 V.
 * RS485: RX D5, TX D4, direction D2; 9600 8N1.
 * OPC: CS D7, SCK D8, MISO D9, MOSI D10; external 5V and shared GND.
 */
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Adafruit_ADS1X15.h>
#include <OPCN3.h>
#include "MessageSlave.h"

const uint8_t NODE_ID = 1;
const unsigned long BAUD = 9600;
const unsigned long SCAN_MS = 10000;
Adafruit_ADS1115 ads;
OPCN3 opc(D7);
SoftwareSerial rs485(D5, D4);
MessageSlave node(rs485, D2);
String opcSerial;
unsigned long lastScan = 0;
bool sensorsReady = false;
bool headerPending = false;
String pendingMessage = "WAIT: first sensor scan";
// Compact names keep all 45 columns within the 240-byte message limit.
const char HEADER[] = "interval_ms,A0_V,A1_V,A2_V,A3_V,OPC_serial,"
  "b0,b1,b2,b3,b4,b5,b6,b7,b8,b9,b10,b11,b12,b13,b14,b15,b16,b17,b18,b19,"
  "b20,b21,b22,b23,tof1,tof3,tof5,tof7,period_s,flow,temp_C,rh,"
  "rej_glitch,rej_tof,rej_ratio,rej_range,fan,laser,crc,PM1,PM2_5,PM10";
static_assert(sizeof(HEADER) - 1 <= MessageSlave::MAX_BYTES, "Header exceeds Modbus message limit");

void publish(const String &message) {
  if (headerPending) pendingMessage = message;
  else node.setMessage(message);
}

void publishError(const String &error) {
  publish(error);
  Serial.println(error);
}

void setup() {
  digitalWrite(D7, HIGH);
  pinMode(D7, OUTPUT);
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 5000) delay(10);
  rs485.begin(BAUD, SERIAL_8N1);
  node.begin(NODE_ID, BAUD);
  node.setMessage("WAIT: initializing sensors");

  if (!Wire.setSDA(D0) || !Wire.setSCL(D1)) {
    publishError("ERROR: I2C pin setup failed; reset to retry");
    return;
  }
  Wire.begin();
  if (!ads.begin(0x48, &Wire)) {
    publishError("ERROR: ADS1115 not found; reset to retry");
    return;
  }
  ads.setGain(GAIN_ONE);
  SPI.begin();
  if (!opc.initialize()) {
    publishError("ERROR: OPC initialization failed; reset to retry");
    return;
  }
  opcSerial = opc.getSerialNumber();
  opcSerial.trim();
  lastScan = millis();
  sensorsReady = true;
  if (node.setMessage(HEADER)) {
    headerPending = true;
    Serial.println(HEADER);
  } else {
    publishError("ERROR: header exceeds 240-byte Modbus limit");
  }
}

void loop() {
  bool replied = node.poll();
  if (headerPending && replied) {
    headerPending = false;
    node.setMessage(pendingMessage);
  }
  if (!sensorsReady) return;
  unsigned long now = millis();
  unsigned long interval = now - lastScan;
  if (interval < SCAN_MS) return;
  lastScan = now;

  HistogramData hist = opc.readHistogramData();
  if (!hist.valid) {
    publishError("ERROR: invalid OPC histogram");
    return;
  }
  String row = String(interval);
  for (uint8_t channel = 0; channel < 4; ++channel) {
    int16_t raw = ads.readADC_SingleEnded(channel);
    row += ',';
    row += String(ads.computeVolts(raw), 4);
  }
  row += ',';
  row += opcSerial;
  row += ',';
  row += hist.toString();
  if (row.length() > MessageSlave::MAX_BYTES) {
    publishError("ERROR: sensor row exceeds 240-byte Modbus limit");
    Serial.print("# Full row (bytes "); Serial.print(row.length()); Serial.print("): ");
    Serial.println(row);
    return;
  }
  publish(row);
  Serial.println(row);
}
