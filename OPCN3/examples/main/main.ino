#include <OPCN3.h>

// create new instance (specify SS pin in constructor)
OPCN3 myOPCN3(D10);

void setup() {

  Serial.begin(9600);

  // turn on laser, fan and set high gain
  myOPCN3.initialize();
  delay(1000);

}

void loop() {

  delay(5000);
  HistogramData hist = myOPCN3.readHistogramData();

  // Get Temperature
  Serial.print("Temperature: ");
  Serial.println(hist.getTempC());

  // Get PM values
  Serial.print("PM 10: ");
  Serial.println(hist.pm10);

  // Get PM values
  Serial.print("PM 10: ");
  Serial.println(hist.pm10);


}