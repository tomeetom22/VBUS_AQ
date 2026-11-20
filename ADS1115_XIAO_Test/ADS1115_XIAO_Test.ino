/*
 * ADS1115 input test for Seeed XIAO RP2040, with no AFE connected.
 * Install Adafruit ADS1X15 and Adafruit BusIO. Serial Monitor: 115200 baud.
 *
 * Standard Qwiic cable: black=GND, red=3V3, blue=SDA (D0), yellow=SCL (D1).
 * Power the ADS1115 from 3.3 V for this test. The four analog inputs are
 * on the ADC board, not in the Qwiic connector. Keep each between GND and 3V3.
 * Disconnect power before moving wires. Floating inputs have undefined values.
 */
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;
const uint8_t ADS_ADDRESS = 0x48; // Default address on Adafruit breakout.
const unsigned long READ_INTERVAL_MS = 5000;
unsigned long lastReadMs = 0;
bool adcReady = false;

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 5000) delay(10);
  Serial.println("\nXIAO RP2040 / ADS1115 analog input test");
  Serial.println("Qwiic: GND=black, 3V3=red, SDA=blue/D0, SCL=yellow/D1");
  Serial.println("ADS1115 address 0x48; inputs must stay between GND and 3V3.");

  // Keep the default D4/D5 free for the RS485 breakout.
  if (!Wire.setSDA(D0) || !Wire.setSCL(D1)) {
    Serial.println("ERROR: failed to select I2C pins.");
    return;
  }
  Wire.begin();
  if (!ads.begin(ADS_ADDRESS, &Wire)) {
    Serial.println("ERROR: ADS1115 not found at 0x48. Check Qwiic wiring, power, and ADDR; reset to retry.");
    return;
  }
  // +/-4.096 V range covers a 0-3.3 V single-ended test. The gain setting
  // does not allow an input voltage above the ADC's 3.3 V supply.
  ads.setGain(GAIN_ONE);
  adcReady = true;
  Serial.println("ADS1115 detected. Floating inputs may show arbitrary voltages.");
  Serial.println("Test with an input connected to GND or 3V3; remove power before changing connections.");
}

void loop() {
  if (!adcReady) {
    delay(100);
    return;
  }
  if (millis() - lastReadMs < READ_INTERVAL_MS) return;
  lastReadMs = millis();

  Serial.println("\nSingle-ended (relative to GND):");
  for (uint8_t channel = 0; channel < 4; channel++) {
    int16_t raw = ads.readADC_SingleEnded(channel);
    Serial.print("A"); Serial.print(channel);
    Serial.print(": raw="); Serial.print(raw);
    Serial.print("  V="); Serial.println(ads.computeVolts(raw), 4);
  }

  // Separate conversions from the single-ended readings above.
  int16_t diff01 = ads.readADC_Differential_0_1();
  int16_t diff23 = ads.readADC_Differential_2_3();
  Serial.print("A0-A1: raw="); Serial.print(diff01);
  Serial.print("  V="); Serial.println(ads.computeVolts(diff01), 4);
  Serial.print("A2-A3: raw="); Serial.print(diff23);
  Serial.print("  V="); Serial.println(ads.computeVolts(diff23), 4);
}
