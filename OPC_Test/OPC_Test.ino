#include <OPCN3.h>
#include "SPI.h"
#include <Arduino.h>

#define OPC_CS WB_IO2
#define OPC_MOSI WB_SPI_MOSI
#define OPC_MISO WB_SPI_MISO
#define OPC_CLK WB_SPI_CLK

OPCN3 my_OPC(OPC_CS);

unsigned long start_time;

void setup() {
  pinMode(OPC_CS, OUTPUT);
  digitalWrite(OPC_CS, HIGH);
  Serial.begin(115200);
  SPI.setPins(OPC_MISO, OPC_CLK, OPC_MOSI);

  if (!my_OPC.initialize()) {
    Serial.print("OPC initialization failed; last response 0x");
    Serial.println(my_OPC.lastResponse(), HEX);
    while (true) { delay(1000); }
  }

  Serial.print("Serial number: ");
  Serial.println(my_OPC.getSerialNumber());

  DACandPowerStatus power = my_OPC.readDACandPowerStatus();
  Serial.print("Power valid/fan/laserDAC/laserSwitch: ");
  Serial.print(power.valid ? 1 : 0);
  Serial.print('/');
  Serial.print(power.fanOn);
  Serial.print('/');
  Serial.print(power.laserDACOn);
  Serial.print('/');
  Serial.println(power.laserSwitch);

  start_time = millis();

}

void loop() {
  unsigned long time_now = millis();
  if (time_now - start_time > 5000) {
    HistogramData hist = my_OPC.readHistogramData();
    my_OPC.printHistogram(Serial, hist);
    start_time = millis();
  }
}
