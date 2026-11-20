/*
AlphaSense OPC-N3
with RTC and SDCard
Arduino Nano BLE 33
written by Alexei Perelet
University of Utah
Environmental Fluid Dynamics Lab
Initial test for Fatima IF @ Utqiagvik Alaska
Code not fully cleaned up
*/
#include <OPCN3.h>
#include <SD.h>
#include <RTClib.h>
#include <SPI.h>

// Define specific pins
#define OPCN3_CS 9
#define SD_CS 8

// Timing variables
RTC_DS3231 rtc;
DateTime now;
unsigned long tmpTime;
uint8_t currDay;

// Adjust clock 
bool adjustClockNow = 0;
DateTime adjustedTime(2025, 10, 21, 20, 35, 00); // Year, Month, Day, Hour (24), Minute, Second

// create new instance of OPCN3(specify SS pin in constructor)
OPCN3 myOPCN3(OPCN3_CS);
// Serial output string for backup
String OPCSerial;

// SD Card file object
File logfile; 

////////////////////////
////////////////////////
// FUNCTION DECLARATIONS
////////////////////////
////////////////////////

// Check if SD Card is plugged in and active
void SDCheck(){
    bool ready = false;
    while (!ready)
    {
        if (SD.begin(500000, SD_CS))
        {
            ready = true;
            Serial.println("SD is Good...");
            return;
        }
        // Flash LED Light if issue
        //SDIssueLED();
        Serial.println("Waiting on SD...");
        
    }
}

// Initialize RTC
void RTCcheck(){
    bool ready = false;

    while (!ready)
    {
        if (rtc.begin())
        {
            delay(1000);
            ready = true;

            if (rtc.lostPower()) {
              Serial.println("RTC lost power, Time set to last compile time.");
              rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
            }
            if (adjustClockNow) {
              rtc.adjust(adjustedTime);
            }
            now = rtc.now();
            Serial.println("RTC Good. Time is: ");
            Serial.print(now.year());
            Serial.print("/");
            Serial.print(now.month());
            Serial.print("/");
            Serial.print(now.day());
            Serial.print(" ");
            Serial.print(now.hour());
            Serial.print(":");
            Serial.print(now.minute());
            Serial.print(":");
            Serial.println(now.second());

            return;
        }
        // Flash LED Light if issue
        //RTCIssueLED();
        Serial.println("Waiting on RTC...");
    }
}

// Check files on card and create new file with increasing counter at end of file name
// Opens file
void mkFile(){
    const int dayCheck = now.day();
    const int monthCheck = now.month();
    const int hourCheck = now.hour();
    const int minuteCheck = now.minute();
    String day = "";
    String month = "";
    String year = "";
    String hour = "";
    String minute = "";

    if (dayCheck<10) {
        day = "0"+String(now.day());
    }
    else {
        day = String(now.day());
    }
    if (monthCheck<10) {
        month = "0"+String(now.month());
    }
    else {
        month = String(now.month());
    }
    if (hourCheck<10) {
        hour = "0"+String(now.hour());
    }
    else {
        hour = String(now.hour());
    }
    if (minuteCheck<10) {
        minute = "0"+String(now.minute());
    }
    else {
        minute = String(now.minute());
    }
    year = String(now.year());

    const String filepath = "/" + year + "/" + month + "/" + day + "_" + hour + minute + ".dat";

    // Check if Year folder exists
    if (!SD.exists(year)){
        // Make dir
        SD.mkdir(year);
    }

    if (!SD.exists(year+"/" + month)) {
      //
      SD.mkdir(year+"/" + month);
    }

    // File with index i not found. Creating file with filepath and opening
    logfile = SD.open(filepath, FILE_WRITE);
    if (!logfile) {
      // if cannot open file for some reason flash SD card issue LED sequence
      Serial.println("Cannot Open File...");
      while(1);
    }

    // Print Header information
    ///////////////////
    // Does not work correctly, partially due to how OPC-N3 library is written
    ///////////////////
    //ConfigurationVariables __configurationVariables = myOPCN3.readConfigurationVariables();
    //logfile.println(__configurationVariables.toString());
}

// Write data to SD card
void writeToCard(HistogramData _hist) {
  SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));
  delay(10);
  if (now.day() != currDay) {
    logfile.close();
    mkFile();
    Serial.println("Making new daily file...");
    currDay = now.day();
  }
  logfile.print(now.year());
  logfile.print(",");
  logfile.print(now.month());
  logfile.print(",");
  logfile.print(now.day());
  logfile.print(",");
  logfile.print(now.hour());
  logfile.print(",");
  logfile.print(now.minute());
  logfile.print(",");
  logfile.print(now.second());
  logfile.print(",");
  logfile.print(OPCSerial);
  logfile.print(",");
  size_t bytesWritten = logfile.println( _hist.toString());

  if (bytesWritten <5) {
    Serial.println("Error writing to SD Card...");
  }
  logfile.flush();
  
  delay(10);
}

void setup() {
  // I don't think these are necessary but the OPC-N3 library is written like crap so confirming both OPC-N3 and SD Card Chip Select is off
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  pinMode(OPCN3_CS, OUTPUT);
  digitalWrite(OPCN3_CS, HIGH);

  // Start serial connection for data backup
  Serial.begin(9600);
  delay(1000);

  currDay = now.day();
  // Perform checks of clock and SD card
  RTCcheck();
  SDCheck();

  // turn on laser, fan and set high gain
  myOPCN3.initialize();

  OPCSerial = myOPCN3.getSerialNumber();
  OPCSerial.trim();

  mkFile();
}

////////////////////////
////////////////////////
// MAIN LOOP
////////////////////////
////////////////////////

void loop() {
  unsigned long tmpTime_0 = millis();

  if (tmpTime_0-tmpTime > 1000) {
    now = rtc.now();
    tmpTime = millis();

    // Get OPC-N3 Data
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1));
    HistogramData hist = myOPCN3.readHistogramData();
    
    // Write data to card
    writeToCard(hist);

    // Write data to serial
    Serial.print(now.year());
    Serial.print(",");
    Serial.print(now.month());
    Serial.print(",");
    Serial.print(now.day());
    Serial.print(",");
    Serial.print(now.hour());
    Serial.print(",");
    Serial.print(now.minute());
    Serial.print(",");
    Serial.print(now.second());
    Serial.print(",");
    Serial.print(OPCSerial);
    Serial.print(",");
    Serial.println(hist.toString());
  }
}