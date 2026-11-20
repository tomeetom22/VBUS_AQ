#include <Arduino.h>
#include <ArduinoJson.h>        // Click to install library: http://librarymanager/All#ArduinoJson

#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_ADS1X15.h>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>  //http://librarymanager/All#SparkFun_u-blox_GNSS

#include "SPI.h"
#include "SD.h"  //http://librarymanager/All#SD
#include "OPC_N3.h"

// Other Defines ----------------------------------------------------------------------------------
// !! ALERT !! When DEBUG is defined as 0, the LEMSv3 will only work properly under battery or
// external power, NOT under USB power. If you would like to power the LEMS via USB but have DEBUG
// set to 0, uncomment the three lines centered around the line that has USBDevice.detach(); This
// should be near line 207 in the code.
#define DEBUG 1
// Set to 1 for indoor/SD testing without a GNSS module or sky view.
#define GPS_OFF 1
// output frequency in milliseconds
#define OUTPUT_FREQ 1000
#define OPC_OUTPUT_FREQ 1000

// GPS initialization time in milliseconds ([default] 1 min)
#define GPS_INITTIME 60000
// GPS timeout in milliseconds ([default] 1 hour)
#define GPS_TIMEOUT 3600000

// AQ sensor configuration. Keep this file focused on sensor presence only.
struct Config {
  bool opc1 = true;
  bool opc2 = true;
  bool gas1 = false;
  bool gas2 = false;
};
// global configuration object
Config config;

// GPS --------------------------------------------------------------------------------------------
SFE_UBLOX_GNSS g_GNSS;
char timestamp[20];
bool gpsIsActive;
bool geofixed = false;
bool gpsTimeValid = false;
// Cache the last valid GNSS time so SD callbacks never perform I2C traffic.
uint16_t cachedGnssYear = 0;
uint8_t cachedGnssMonth = 0;
uint8_t cachedGnssDay = 0;
uint8_t cachedGnssHour = 0;
uint8_t cachedGnssMinute = 0;
uint8_t cachedGnssSecond = 0;
uint32_t gpsStaleCount = 0;
uint32_t timestampBaseEpoch = 0;
unsigned long timestampBaseMillis = 0;
unsigned long noGpsLastMillis = 0;
uint32_t noGpsElapsedSeconds = 0;
uint16_t noGpsSubsecondMillis = 0;
int LTzone = 0;
bool gnss_init();
bool gnss_check_geofix();
bool gnss_get_geofix();
bool gnss_get_time();
void set_no_gps_timestamp();
void update_no_gps_timestamp();
uint32_t gnss_date_to_epoch(uint16_t year, uint8_t month, uint8_t day,
                            uint8_t hour, uint8_t minute, uint8_t second);
void gnss_epoch_to_date(uint32_t epoch, uint16_t &year, uint8_t &month,
                        uint8_t &day, uint8_t &hour, uint8_t &minute,
                        uint8_t &second);
void update_timestamp_from_last_gnss();

// ADS1115 ----------------------------------------------------------------------------------------
Adafruit_ADS1115 ads; /* Use this for the 16-bit version */
bool adsIsActive;
double vBat;

bool ads_init();
void ads_get();

// SD Card ----------------------------------------------------------------------------------------
bool sdIsActive;
File errfile;                       // File object (err file)
File logfile;                       // File object (log file)
File datafile;                      // File object (data file)
File gnssfile;                      // File object (gnss file)
String datafname = "AQSN00.CSV";  // Data filename
String gnssfname = "AQSN_G.CSV";  // GNSS filename
const char *configfname = "AQCONFIG.TXT";
bool sd_init();
void sd_init_datafile();
void sd_dump_datafile();
void sd_dump_gnssfile();
void sd_dump_error(const char *);
void sd_log_header();
void sd_log_geofix();
void sd_dump_log(const char *);
void sd_date_time(uint16_t *date, uint16_t *time);
bool sd_is_leap_year(uint16_t year);
uint8_t sd_days_in_month(uint16_t year, uint8_t month);

// Auxiliary-board SPI wiring:
//   IO-B 25  SPI_CS   -> RAK4631 GPIO26 (WB_SPI_CS)
//   IO-B 26  SPI_CLK  -> RAK4631 GPIO3  (WB_SPI_CLK)
//   IO-B 27  SPI_MISO -> RAK4631 GPIO29 (WB_SPI_MISO / SD SDO)
//   IO-B 28  SPI_MOSI -> RAK4631 GPIO30 (WB_SPI_MOSI)
//   IO-B 18  3v3      -> direct 3.3 V (not a GPIO-controlled power pin)
static const uint8_t SD_CS_PIN = WB_SPI_CS;      // IO-B pin 25, GPIO 26
static const uint8_t SD_SCK_PIN = WB_SPI_CLK;    // IO-B pin 26, GPIO 3
static const uint8_t SD_MISO_PIN = WB_SPI_MISO;  // IO-B pin 27, GPIO 29; SD SDO
static const uint8_t SD_MOSI_PIN = WB_SPI_MOSI;  // IO-B pin 28, GPIO 30
static const uint32_t SD_SPI_CLOCK_HZ = 250000;

// Auxiliary-board OPC wiring:
//   IO-B 29  IO1 -> OPC1 SS -> RAK4631 GPIO17 (WB_IO1)
//   IO-B 30  IO2 -> OPC2 SS -> RAK4631 GPIO34 (WB_IO2)
// Both OPCs share SPI_CLK, SPI_MOSI, and SPI_MISO with the SD card.
// The OPC 5 V supply is external and must share ground with the RAK board.
static const uint8_t OPC1_CS_PIN = WB_IO1;
static const uint8_t OPC2_CS_PIN = WB_IO2;
OPCDevice opc1(OPC1_CS_PIN);
OPCDevice opc2(OPC2_CS_PIN);
bool opc1IsActive = false;
bool opc2IsActive = false;
unsigned long lastOpcMs = 0;
OpcHistogram opcHist1;
OpcHistogram opcHist2;
float opc1Pm1 = NAN;
float opc1Pm25 = NAN;
float opc1Pm10 = NAN;
float opc2Pm1 = NAN;
float opc2Pm25 = NAN;
float opc2Pm10 = NAN;
bool opc1LastReadOk = false;
bool opc2LastReadOk = false;

bool opc_init(OPCDevice &opc, const char *sensorName);
void opc_get();
void spi_deselect_all();


// FUNCTIONS --------------------------------------------------------------------------------------
void error(const char *);
void led_warning();
void serial_info();
void serial_geofix();
void serial_dump();


// ------------------------------------------------------------------------------------------------
// SETUP
void setup() {
  // Always have a valid CSV timestamp, even if GNSS is disabled or unavailable.
  set_no_gps_timestamp();

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  pinMode(OPC1_CS_PIN, OUTPUT);
  digitalWrite(OPC1_CS_PIN, HIGH);
  pinMode(OPC2_CS_PIN, OUTPUT);
  digitalWrite(OPC2_CS_PIN, HIGH);
  spi_deselect_all();

  // Initialize the builtin LEDs
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);

  // SD power is direct 3v3 on IO-B connector pin 18; only cycle the LEDs.
  digitalWrite(PIN_LED1, LOW);
  digitalWrite(PIN_LED2, LOW);
  delay(1000);
  digitalWrite(PIN_LED1, HIGH);
  digitalWrite(PIN_LED2, HIGH);
  delay(500);
  digitalWrite(PIN_LED1, LOW);
  digitalWrite(PIN_LED2, LOW);
  delay(500);
  digitalWrite(PIN_LED1, HIGH);

#if DEBUG
  // Initialize Serial for debug output
  Serial.begin(115200);

  // Initialize Serial for debug output.
  time_t timeout = millis();
  while (!Serial) {
    if ((millis() - timeout) < 5000) {
      delay(100);
      digitalWrite(PIN_LED1, !digitalRead(PIN_LED1));
    } else {
      break;
    }
  }
  Serial.println("=====================================");
  Serial.println("UofU LEMS DEBUG");
  Serial.println("=====================================");
  Serial.println("Auxiliary-board SPI wiring:");
  Serial.print("  SD CS/MISO/SCK/MOSI = ");
  Serial.print(SD_CS_PIN);
  Serial.print("/");
  Serial.print(SD_MISO_PIN);
  Serial.print("/");
  Serial.print(SD_SCK_PIN);
  Serial.print("/");
  Serial.println(SD_MOSI_PIN);
  Serial.print("  OPC1 CS / OPC2 CS = ");
  Serial.print(OPC1_CS_PIN);
  Serial.print(" / ");
  Serial.println(OPC2_CS_PIN);
  Serial.println("  OPC power: external 5 V with common ground");
#else
  // Allows to unplug before main code starts
  delay(5000);
#endif
  digitalWrite(PIN_LED1, HIGH);
  digitalWrite(PIN_LED2, LOW);

  // initialize I2C library
  Wire.begin();

  // Initialize SPI and hold every OPC CS high before the SD card uses SPI.
  SPI.setPins(SD_MISO_PIN, SD_SCK_PIN, SD_MOSI_PIN);
  SPI.begin();
  // Use the GNSS clock for SD timestamps only after a valid fix is acquired.
  SdFile::dateTimeCallback(sd_date_time);

  // initialize components
  sdIsActive = sd_init();
  Serial.flush();
#if GPS_OFF
  // Skip all GNSS communication during lab/SD testing.
  gpsIsActive = false;
  geofixed = false;
  gpsTimeValid = false;
  set_no_gps_timestamp();
#else
  gpsIsActive = gnss_init();
  if (!gpsIsActive) {
    set_no_gps_timestamp();
    gpsStaleCount = 1;
  }
#endif
  Serial.flush();
  adsIsActive = ads_init();

  opc1.begin();
  opc2.begin();

  opc1IsActive = config.opc1 && opc_init(opc1, "OPC1");
  opc2IsActive = config.opc2 && opc_init(opc2, "OPC2");

  // create datafile
  sd_init_datafile();

#if DEBUG
  serial_info();
#endif

  if (gpsIsActive) {
    geofixed = gnss_check_geofix();
    if (geofixed) {
      // Populate the cached time before SD metadata is written.
      if (gnss_get_time()) {
        sd_log_geofix();
#if DEBUG
        serial_geofix();
#endif
      }
    }
  }

  // Create the data file after the initial GPS check so its timestamp is valid
  // when a fix was available during startup.
  sd_init_datafile();

  // LED SIGNAL -> ALL GOOD TO GO!
  digitalWrite(PIN_LED1, LOW);
  digitalWrite(PIN_LED2, LOW);
  delay(500);
  for (uint8_t i = 0; i < 5; i++) {
    digitalWrite(PIN_LED1, !digitalRead(PIN_LED1));
    digitalWrite(PIN_LED2, !digitalRead(PIN_LED2));
    delay(200);
  }
  digitalWrite(PIN_LED1, LOW);
  digitalWrite(PIN_LED2, LOW);
  delay(500);
}
// END OF SETUP
// ------------------------------------------------------------------------------------------------

// ------------------------------------------------------------------------------------------------
// LOOP
void loop() {
  digitalWrite(PIN_LED1, HIGH);  // Turn LED on
  // time at start of loop
  unsigned long lastTime = millis();

  // data aquisition (based on active sensors)
  if (gpsIsActive) {
    // Keep the synthetic clock running until a valid GNSS time is available.
    // After a valid fix, extrapolate from the last GNSS time if a poll fails.
    if (!gnss_get_time()) {
      if (gpsTimeValid) {
        update_timestamp_from_last_gnss();
      } else {
        update_no_gps_timestamp();
      }
    }
  } else {
    update_no_gps_timestamp();
  }
  if (adsIsActive) { ads_get(); }
  if (millis() - lastOpcMs >= OPC_OUTPUT_FREQ) {
    lastOpcMs = millis();
    opc_get();

#if DEBUG
    serial_dump();
#endif

    sd_dump_datafile();
  }

  // log GPS COORD
  if (gpsIsActive && !geofixed) {
    geofixed = gnss_get_geofix();
    if (geofixed) {
      if (gnss_get_time()) {
        // dump to log/serial
        sd_log_geofix();
#if DEBUG
        serial_geofix();
#endif
      }
    }
  }

  // if GPS fixed -> log GPS once a day  (noon LT)
  if (geofixed && gpsTimeValid &&
      cachedGnssHour + LTzone == 12 && cachedGnssMinute == 0) {
    sd_dump_gnssfile();
  }

  digitalWrite(PIN_LED1, LOW);  // Turn LED off

  // delay to next aquisition (max OUTPUT_FREQ)
  unsigned long elapsedTime = millis() - lastTime;
  if (elapsedTime < OUTPUT_FREQ) {
    delay(OUTPUT_FREQ - elapsedTime);
  }
}
// END OF MAIN LOOP
// ------------------------------------------------------------------------------------------------ 


void spi_deselect_all() {
  digitalWrite(SD_CS_PIN, HIGH);
  digitalWrite(OPC1_CS_PIN, HIGH);
  digitalWrite(OPC2_CS_PIN, HIGH);
}

// ------------------------------------------------------------------------------------------------


// ------------------------------------------------------------------------------------------------ 
// OPC
bool opc_init(OPCDevice &opc, const char *sensorName) {
  char log_message[96];
  snprintf(log_message, sizeof(log_message), "Starting %s", sensorName);
  sd_dump_log(log_message);

  spi_deselect_all();
  if (!opc.turnOn()) {
    spi_deselect_all();
    snprintf(log_message, sizeof(log_message),
             "[WARNING] %s turn-on failed: %s (SPI status 0x%02X)",
             sensorName, opc.lastErrorText(), opc.lastResponse());
    sd_dump_log(log_message);
    led_warning();
    return false;
  }
  spi_deselect_all();

  OpcHistogram discard;
  spi_deselect_all();
  if (!opc.readHistogram(discard)) {
    spi_deselect_all();
    snprintf(log_message, sizeof(log_message),
             "[WARNING] %s first histogram failed: %s (SPI status 0x%02X)",
             sensorName, opc.lastErrorText(), opc.lastResponse());
    sd_dump_log(log_message);
    led_warning();
    return false;
  }
  spi_deselect_all();

  snprintf(log_message, sizeof(log_message), "| %s detected", sensorName);
  sd_dump_log(log_message);
  return true;
}

// ------------------------------------------
void opc_get() {
  opc1Pm1 = NAN;
  opc1Pm25 = NAN;
  opc1Pm10 = NAN;
  opc2Pm1 = NAN;
  opc2Pm25 = NAN;
  opc2Pm10 = NAN;
  opc1LastReadOk = false;
  opc2LastReadOk = false;

  if (opc1IsActive) {
    spi_deselect_all();
    opc1LastReadOk = opc1.readHistogram(opcHist1);
    spi_deselect_all();
    if (opc1LastReadOk) {
      opc1Pm1 = opcHist1.pm1;
      opc1Pm25 = opcHist1.pm25;
      opc1Pm10 = opcHist1.pm10;
    } else {
      char log_message[96];
      snprintf(log_message, sizeof(log_message),
               "[WARNING] OPC1 histogram failed: %s (SPI status 0x%02X)",
               opc1.lastErrorText(), opc1.lastResponse());
      sd_dump_log(log_message);
    }
  }

  if (opc2IsActive) {
    spi_deselect_all();
    opc2LastReadOk = opc2.readHistogram(opcHist2);
    spi_deselect_all();
    if (opc2LastReadOk) {
      opc2Pm1 = opcHist2.pm1;
      opc2Pm25 = opcHist2.pm25;
      opc2Pm10 = opcHist2.pm10;
    } else {
      char log_message[96];
      snprintf(log_message, sizeof(log_message),
               "[WARNING] OPC2 histogram failed: %s (SPI status 0x%02X)",
               opc2.lastErrorText(), opc2.lastResponse());
      sd_dump_log(log_message);
    }
  }
}

// ------------------------------------------------------------------------------------------------

// ------------------------------------------------------------------------------------------------
// GPS GNSS
void set_no_gps_timestamp() {
  // Start a synthetic CSV clock when no real-time clock is available.
  gpsTimeValid = false;
  gpsStaleCount = 0;
  noGpsLastMillis = millis();
  noGpsElapsedSeconds = 0;
  noGpsSubsecondMillis = 0;
  snprintf(timestamp, sizeof(timestamp), "2000-05-01 00:00:00");
}

bool gnss_init() {
  sd_dump_log("Starting GNSS (GPS)");
  unsigned long gnss_timeout = millis() + 5000;  // 5 second timeout
  
  // Try to initialize GNSS with timeout
  while (!g_GNSS.begin() && millis() < gnss_timeout) {
    delay(100);
  }
  
  if (millis() >= gnss_timeout) {
    sd_dump_log("[WARNING] GNSS (GPS) init timeout -> not detected");
    led_warning();
    return false;
  }
  
  sd_dump_log("| GNSS (GPS) detected");
  g_GNSS.setI2COutput(COM_TYPE_UBX);                  //Set the I2C port to output UBX only (turn off NMEA noise)
  g_GNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);  //Save (only) the communications port settings to flash and BBR
  return true;
}
// ------------------------------------------
bool gnss_check_geofix() {

  digitalWrite(PIN_LED1, HIGH);  // turn green LED on

  time_t startTime = millis();  // timer
  time_t fixTime = millis();    // timer
  time_t ledTime = millis();    // timer

  byte fixType = g_GNSS.getFixType();  // Get the fix type
  while (fixType == 0) {
    //Query module every 10 seconds
    if (millis() - fixTime > 10000) {
      fixTime = millis();             //Update the timer
      fixType = g_GNSS.getFixType();  // Get the fix type
    }

    // flash blue LED
    if (millis() - ledTime > 1000) {
      ledTime = millis();  //Update the timer
      digitalWrite(PIN_LED2, !digitalRead(PIN_LED2));
    }

    if (millis() - startTime > GPS_INITTIME) {
      g_GNSS.powerSaveMode();
      return false;
    }
  }
  digitalWrite(PIN_LED2, LOW);  // turn blue LED off

  // get time zone (approx.)
  long longitude = g_GNSS.getLongitude();
  LTzone = round(longitude / 10000000.0 / 15.0);
  if (LTzone < -12 || LTzone > 12) { LTzone = 0; }

  return true;
}
// ------------------------------------------
bool gnss_get_geofix() {

  // return false if GPS time out....
  if (millis() > GPS_TIMEOUT && !geofixed) {
    return false;
  }
  digitalWrite(PIN_LED2, HIGH);        // turn bluw LED on
  byte fixType = g_GNSS.getFixType();  // Get the fix type
  bool test = false;
  if (fixType > 0) {
    test = true;
    // get time zone (approx.)
    long longitude = g_GNSS.getLongitude();
    LTzone = round(longitude / 10000000.0 / 15.0);
    if (LTzone < -12 || LTzone > 12) { LTzone = 0; }
  }
  digitalWrite(PIN_LED2, LOW);  // turn blue LED off
  return test;
}
// ------------------------------------------
bool gnss_get_time() {
  // Poll PVT once. The individual getters can each initiate another poll if
  // their cached field is stale, so only read them after a successful PVT.
  static bool firstTimePoll = true;
  uint16_t pvtTimeout = firstTimePoll ? GPS_INITTIME : defaultMaxWait;
  firstTimePoll = false;

  if (!g_GNSS.getPVT(pvtTimeout)) {
    gpsStaleCount++;
    return false;
  }

  uint16_t year = g_GNSS.getYear();
  uint8_t month = g_GNSS.getMonth();
  uint8_t day = g_GNSS.getDay();
  uint8_t hour = g_GNSS.getHour();
  uint8_t minute = g_GNSS.getMinute();
  uint8_t second = g_GNSS.getSecond();

  if (year < 1980 || month < 1 || month > 12 ||
      day < 1 || day > sd_days_in_month(year, month) ||
      hour > 23 || minute > 59 || second > 59) {
    gpsStaleCount++;
    return false;
  }

  cachedGnssYear = year;
  cachedGnssMonth = month;
  cachedGnssDay = day;
  cachedGnssHour = hour;
  cachedGnssMinute = minute;
  cachedGnssSecond = second;
  gpsTimeValid = true;
  gpsStaleCount = 0;
  timestampBaseEpoch = gnss_date_to_epoch(
      year, month, day, hour, minute, second);
  timestampBaseMillis = millis();

  snprintf(timestamp,
           sizeof(timestamp),
           "%04u-%02u-%02u %02u:%02u:%02u",
           year, month, day, hour, minute, second);
  return true;
}

uint32_t gnss_date_to_epoch(uint16_t year, uint8_t month, uint8_t day,
                            uint8_t hour, uint8_t minute, uint8_t second) {
  uint32_t days = 0;
  for (uint16_t y = 1970; y < year; y++) {
    days += sd_is_leap_year(y) ? 366UL : 365UL;
  }
  for (uint8_t m = 1; m < month; m++) {
    days += sd_days_in_month(year, m);
  }
  days += day - 1;

  return days * 86400UL + (uint32_t)hour * 3600UL +
         (uint32_t)minute * 60UL + second;
}

void gnss_epoch_to_date(uint32_t epoch, uint16_t &year, uint8_t &month,
                        uint8_t &day, uint8_t &hour, uint8_t &minute,
                        uint8_t &second) {
  uint32_t remainingDays = epoch / 86400UL;
  uint32_t secondsToday = epoch % 86400UL;

  year = 1970;
  while (remainingDays >= (sd_is_leap_year(year) ? 366UL : 365UL)) {
    remainingDays -= sd_is_leap_year(year) ? 366UL : 365UL;
    year++;
  }

  month = 1;
  while (remainingDays >= sd_days_in_month(year, month)) {
    remainingDays -= sd_days_in_month(year, month);
    month++;
  }
  day = remainingDays + 1;
  hour = secondsToday / 3600UL;
  minute = (secondsToday % 3600UL) / 60UL;
  second = secondsToday % 60UL;
}

void update_timestamp_from_last_gnss() {
  if (!gpsTimeValid) {
    return;
  }

  uint32_t elapsedSeconds =
      (uint32_t)(millis() - timestampBaseMillis) / 1000UL;
  uint32_t currentEpoch = timestampBaseEpoch + elapsedSeconds;
  uint16_t year;
  uint8_t month, day, hour, minute, second;
  gnss_epoch_to_date(currentEpoch, year, month, day, hour, minute, second);

  snprintf(timestamp, sizeof(timestamp), "%04u-%02u-%02u %02u:%02u:%02u",
           year, month, day, hour, minute, second);
}

void update_no_gps_timestamp() {
  // Accumulate elapsed time using unsigned subtraction so millis() rollover
  // does not reset the synthetic clock.
  unsigned long nowMillis = millis();
  unsigned long deltaMillis = nowMillis - noGpsLastMillis;
  noGpsLastMillis = nowMillis;

  uint32_t totalMillis = (uint32_t)noGpsSubsecondMillis + deltaMillis;
  noGpsElapsedSeconds += totalMillis / 1000UL;
  noGpsSubsecondMillis = totalMillis % 1000UL;

  uint16_t year = 2000;
  uint8_t month = 1;
  uint8_t day = 1;
  uint32_t remainingDays = noGpsElapsedSeconds / 86400UL;
  uint32_t secondsToday = noGpsElapsedSeconds % 86400UL;

  while (remainingDays >= sd_days_in_month(year, month)) {
    remainingDays -= sd_days_in_month(year, month);
    if (month == 12) {
      month = 1;
      year++;
    } else {
      month++;
    }
  }
  day += remainingDays;

  uint8_t hour = secondsToday / 3600UL;
  uint8_t minute = (secondsToday % 3600UL) / 60UL;
  uint8_t second = secondsToday % 60UL;

  snprintf(timestamp, sizeof(timestamp), "%04u-%02u-%02u %02u:%02u:%02u",
           year, month, day, hour, minute, second);
}


// ------------------------------------------------------------------------------------------------
// ADS
bool ads_init() {
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads.setGain(GAIN_ONE);  // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  sd_dump_log("Searching for ADC");
  bool IsActive;

  if (ads.begin()) {
    IsActive = true;
    sd_dump_log("| ADC detected");
  } else {
    IsActive = false;
    if (config.gas1 || config.gas2) {
      sd_dump_log("[WARNING] ADS sensor NOT detected ->check wiring!");
      led_warning();
    }
  }
  return IsActive;
}
// ------------------------------------------
void ads_get() {
  int16_t adc1 = ads.readADC_Differential_2_3();
  vBat = 5.0 * ads.computeVolts(adc1);
}

// ------------------------------------------------------------------------------------------------
// SD CARD
bool sd_is_leap_year(uint16_t year) {
  return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

uint8_t sd_days_in_month(uint16_t year, uint8_t month) {
  static const uint8_t days[] = {
    31, 28, 31, 30, 31, 30,
    31, 31, 30, 31, 30, 31
  };

  if (month == 2 && sd_is_leap_year(year)) {
    return 29;
  }
  return days[month - 1];
}

// Return the weekday for a Gregorian date: Sunday = 0, Saturday = 6.
uint8_t sd_day_of_week(uint16_t year, uint8_t month, uint8_t day) {
  static const uint8_t monthOffsets[] = {
    0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4
  };

  if (month < 3) {
    year--;
  }
  return (year + year / 4 - year / 100 + year / 400 +
          monthOffsets[month - 1] + day) % 7;
}

bool sd_is_mountain_daylight_time(uint16_t year, uint8_t month,
                                  uint8_t day, uint8_t utcHour) {
  // US Mountain Time rules: second Sunday in March through first Sunday
  // in November. The transitions occur at 09:00 UTC and 08:00 UTC.
  if (month < 3 || month > 11) {
    return false;
  }
  if (month > 3 && month < 11) {
    return true;
  }

  uint8_t transitionDay = 1;
  if (month == 3) {
    transitionDay = 8;
    while (sd_day_of_week(year, month, transitionDay) != 0) {
      transitionDay++;
    }
    if (day > transitionDay) {
      return true;
    }
    if (day < transitionDay) {
      return false;
    }
    return utcHour >= 9;
  }

  while (sd_day_of_week(year, month, transitionDay) != 0) {
    transitionDay++;
  }
  if (day < transitionDay) {
    return true;
  }
  if (day > transitionDay) {
    return false;
  }
  return utcHour < 8;
}

void sd_date_time(uint16_t *date, uint16_t *time) {
  // Keep the SD library default until GNSS has supplied a valid clock.
  *date = FAT_DEFAULT_DATE;
  *time = FAT_DEFAULT_TIME;

  if (!gpsTimeValid) {
    return;
  }

  uint16_t year = cachedGnssYear;
  uint8_t month = cachedGnssMonth;
  uint8_t day = cachedGnssDay;
  uint8_t hour = cachedGnssHour;
  uint8_t minute = cachedGnssMinute;
  uint8_t second = cachedGnssSecond;

  // FAT dates begin at 1980; reject an invalid GNSS response safely.
  if (year < 1980 || month < 1 || month > 12 ||
      day < 1 || day > sd_days_in_month(year, month)) {
    return;
  }

  // Keep measurements in UTC, but use seasonal Mountain Time for SD metadata.
  int8_t mountainOffsetHours = sd_is_mountain_daylight_time(
      year, month, day, hour) ? -6 : -7;
  int32_t localSeconds = (int32_t)hour * 3600 +
                         (int32_t)minute * 60 + second +
                         (int32_t)mountainOffsetHours * 3600;
  while (localSeconds < 0) {
    localSeconds += 24L * 3600L;
    if (day == 1) {
      if (month == 1) {
        year--;
        month = 12;
      } else {
        month--;
      }
      day = sd_days_in_month(year, month);
    } else {
      day--;
    }
  }
  while (localSeconds >= 24L * 3600L) {
    localSeconds -= 24L * 3600L;
    day++;
    if (day > sd_days_in_month(year, month)) {
      day = 1;
      if (month == 12) {
        month = 1;
        year++;
      } else {
        month++;
      }
    }
  }

  hour = localSeconds / 3600L;
  minute = (localSeconds % 3600L) / 60L;
  second = localSeconds % 60L;

  *date = FAT_DATE(year, month, day);
  *time = FAT_TIME(hour, minute, second);
}

bool sd_init() {
    spi_deselect_all();
    bool isActive = SD.begin(SD_SPI_CLOCK_HZ, SD_CS_PIN);

  // if initialization fails, retry 10x with delays
  if (!isActive) {
    for (int k = 0; k < 10; k++) {
      delay(500);  // Wait before retry
      spi_deselect_all();
      SD.end();
      delay(200);  // Wait after SD.end()
      spi_deselect_all();
      SPI.begin();  // Re-initialize SPI
      delay(200);
      spi_deselect_all();
      isActive = SD.begin(SD_SPI_CLOCK_HZ, SD_CS_PIN);
      if (isActive) {
        break;
      }
    }
    if (!isActive) {
      error("Card Initialization Failed!");
      return false;
    }
  }

  // reading config file
  spi_deselect_all();
  File json_file = SD.open(configfname);
    if (!json_file) {
      error("AQCONFIG.TXT not found!");
      return false;
    }
  
    String cfg = "";
    while (json_file.available()) {
      char c = (char)json_file.read();
      cfg += c;
    }
    json_file.close();
    spi_deselect_all();
  
    StaticJsonDocument<1024> json_config;
    DeserializationError json_error = deserializeJson(json_config, cfg.c_str());

    if (json_error) {
      // Proceed with defaults
    }

  if (json_config.containsKey("opc1")) { config.opc1 = json_config["opc1"]; }
  if (json_config.containsKey("opc2")) { config.opc2 = json_config["opc2"]; }
  if (json_config.containsKey("gas1")) { config.gas1 = json_config["gas1"]; }
  if (json_config.containsKey("gas2")) { config.gas2 = json_config["gas2"]; }

  spi_deselect_all();
  logfile = SD.open("LEMS_LOG.TXT", FILE_WRITE);
  if (logfile) {
    logfile.println("=====================================");
    logfile.println("UofU AQ LOG");
    logfile.println("=====================================");
    logfile.print("Config: ");
    logfile.println(configfname);
    logfile.print("OPC1: ");
    logfile.println(config.opc1 ? "enabled" : "disabled");
    logfile.print("OPC2: ");
    logfile.println(config.opc2 ? "enabled" : "disabled");
    logfile.print("Gas1: ");
    logfile.println(config.gas1 ? "enabled" : "disabled");
    logfile.print("Gas2: ");
    logfile.println(config.gas2 ? "enabled" : "disabled");
    logfile.close();
    spi_deselect_all();
  }

  return true;
}

// ------------------------------------------------------------------------------------------------
void sd_init_datafile() {

  // Check for new datafname
  // example: "AQSN00.CSV"
  for (uint8_t i = 0; i < 100; i++) {
    datafname[4] = '0' + i / 10;
    datafname[5] = '0' + i % 10;
    spi_deselect_all();
    if (!SD.exists(datafname)) {
      break;
    }
  }

  spi_deselect_all();
  datafile = SD.open(datafname, FILE_WRITE);
  if (!datafile) {
    error("Couldn't Create Data File!");
  } else {
    sd_dump_log("Creating data file");
  }

  char log_message[64];
  snprintf(log_message, sizeof(log_message),
           "| File name: %s",
           datafname.c_str());
  sd_dump_log(log_message);

  datafile.print("#timestamp,GPS_STALE");
  if (adsIsActive) {
    datafile.print(",Bat_V");
  }
  if (opc1IsActive) {
    datafile.print(",OPC1_PM1_ugm3,OPC1_PM25_ugm3,OPC1_PM10_ugm3,OPC1_SamplePeriod_s,OPC1_ChecksumOK");
  }
  if (opc2IsActive) {
    datafile.print(",OPC2_PM1_ugm3,OPC2_PM25_ugm3,OPC2_PM10_ugm3,OPC2_SamplePeriod_s,OPC2_ChecksumOK");
  }

  datafile.println();
  datafile.flush();
  spi_deselect_all();
}
// ------------------------------------------
void sd_dump_datafile() {

  // Log Data
  spi_deselect_all();
  datafile.print(timestamp);
  datafile.print(", ");
  datafile.print(gpsStaleCount);
  if (adsIsActive) {
    datafile.print(", ");
    datafile.print(vBat);
  }
  if (opc1IsActive) {
    datafile.print(", ");
    if (!isnan(opc1Pm1)) datafile.print(opc1Pm1, 3);
    datafile.print(", ");
    if (!isnan(opc1Pm25)) datafile.print(opc1Pm25, 3);
    datafile.print(", ");
    if (!isnan(opc1Pm10)) datafile.print(opc1Pm10, 3);
    datafile.print(", ");
    if (opc1LastReadOk) datafile.print(opcHist1.samplingPeriodS, 2);
    datafile.print(", ");
    datafile.print(opc1LastReadOk && opcHist1.checksumOk ? 1 : 0);
  }
  if (opc2IsActive) {
    datafile.print(", ");
    if (!isnan(opc2Pm1)) datafile.print(opc2Pm1, 3);
    datafile.print(", ");
    if (!isnan(opc2Pm25)) datafile.print(opc2Pm25, 3);
    datafile.print(", ");
    if (!isnan(opc2Pm10)) datafile.print(opc2Pm10, 3);
    datafile.print(", ");
    if (opc2LastReadOk) datafile.print(opcHist2.samplingPeriodS, 2);
    datafile.print(", ");
    datafile.print(opc2LastReadOk && opcHist2.checksumOk ? 1 : 0);
  }
  datafile.println();
  datafile.flush();
  spi_deselect_all();

  delay(50);
}
// ------------------------------------------
void sd_init_gnssfile() {
  gnssfile.print("#timestamp,FixType,Lat_deg,Long_deg,Alt_m");
  gnssfile.println();
  gnssfile.flush();
}
// ------------------------------------------
void sd_dump_gnssfile() {

  spi_deselect_all();
  gnssfile = SD.open(gnssfname, FILE_WRITE);
  if (gnssfile) {
    byte fixType = g_GNSS.getFixType();  // Get the fix type
    long latitude = g_GNSS.getLatitude();
    long longitude = g_GNSS.getLongitude();
    long altitude = g_GNSS.getAltitude();

    // Log Data
    gnssfile.print(timestamp);
    gnssfile.print(", ");
    gnssfile.print(fixType);
    gnssfile.print(", ");
    gnssfile.print(latitude / 10000000.0, 7);
    gnssfile.print(", ");
    gnssfile.print(longitude / 10000000.0, 7);
    gnssfile.print(", ");
    gnssfile.print(altitude / 1000);

    gnssfile.println();
    gnssfile.flush();
    gnssfile.close();
    spi_deselect_all();
  }
}
// ------------------------------------------
void sd_dump_error(const char *errorMsg) {
  spi_deselect_all();
  errfile = SD.open("LEMS_ERR.TXT", FILE_WRITE);
  if (errfile) {
    errfile.print("[ERROR] ");
    errfile.print(errorMsg);
    errfile.println();
    errfile.flush();
    errfile.close();
    spi_deselect_all();
  }
}
// ------------------------------------------
void sd_dump_log(const char *logMsg) {
  spi_deselect_all();
  logfile = SD.open("LEMS_LOG.TXT", FILE_WRITE);
  if (logfile) {
    logfile.print(logMsg);
    logfile.println();
    logfile.flush();
    logfile.close();
    spi_deselect_all();
  }
#if DEBUG
  Serial.print(logMsg);
  Serial.println();
#endif
}
// ------------------------------------------
void sd_log_header() {
  spi_deselect_all();
  logfile = SD.open("LEMS_LOG.TXT", FILE_WRITE);
  if (logfile) {
    logfile.println("=====================================");
    logfile.println("UofU AQ LOG");
    logfile.println("=====================================");
    logfile.println("GNSS (GPS) detected");
    logfile.println("=====================================");
    logfile.print("Writing to ");
    logfile.println(datafname);
    logfile.println("=====================================");
    logfile.close();
    spi_deselect_all();
  }
}
// ------------------------------------------
void sd_log_geofix() {
  spi_deselect_all();
  logfile = SD.open("LEMS_LOG.TXT", FILE_WRITE);
  if (logfile) {

    logfile.println("GNSS (GPS) geofix info:");

    logfile.print("| Datetime: ");
    logfile.println(timestamp);

    byte fixType = g_GNSS.getFixType();  // Get the fix type
    logfile.print(F("| Fix: "));
    logfile.print(fixType);
    if (fixType == 0)
      logfile.print(F("(No fix)"));
    else if (fixType == 1)
      logfile.print(F("(Dead reckoning)"));
    else if (fixType == 2)
      logfile.print(F("(2D)"));
    else if (fixType == 3)
      logfile.print(F("(3D)"));
    else if (fixType == 4)
      logfile.print(F("(GNSS + Dead reckoning)"));
    logfile.println();

    long latitude = g_GNSS.getLatitude();
    long longitude = g_GNSS.getLongitude();
    long altitude = g_GNSS.getAltitude();

    logfile.print(F("| Lat,Long: ("));
    logfile.print(latitude / 10000000.0, 7);
    logfile.print(F(","));
    logfile.print(longitude / 10000000.0, 7);
    logfile.println(F(") (degrees)"));

    logfile.print(F("| Alt: "));
    logfile.print(altitude / 1000);
    logfile.println(F(" (m)"));

    logfile.print(F("| Time Zone: "));
    logfile.print(LTzone);
    logfile.println(F(" (h)"));

    //logfile.println("=====================================");
    logfile.close();
    spi_deselect_all();
  }
}
// ------------------------------------------------------------------------------------------------
// Turn red LED on and loop. Used in case of errors
void error(const char *errorMsg) {
#if DEBUG
  Serial.print("[ERROR] ");
  Serial.println(errorMsg);
#endif
  if (sdIsActive) { sd_dump_error(errorMsg); }
  digitalWrite(PIN_LED1, LOW);  // Ensure green LED is off
  while (true) {
    digitalWrite(PIN_LED2, HIGH);
    digitalWrite(PIN_LED1, LOW);
    delay(300);
    digitalWrite(PIN_LED2, LOW);
    digitalWrite(PIN_LED1, HIGH);
    delay(300);
  }
}

void led_warning() {
  // LED SIGNAL -> WARNING
  digitalWrite(PIN_LED1, LOW);
  digitalWrite(PIN_LED2, HIGH);
  delay(200);
  for (uint8_t i = 0; i < 5; i++) {
    digitalWrite(PIN_LED1, !digitalRead(PIN_LED1));
    delay(50);
  }
  delay(200);
  digitalWrite(PIN_LED1, HIGH);
  digitalWrite(PIN_LED2, LOW);
}
// ------------------------------------------
void serial_info() {
  Serial.println("=====================================");

  Serial.print("#timestamp,GPS_STALE");
  if (adsIsActive) {
    Serial.print(",Bat_V");
  }
  if (opc1IsActive) {
    Serial.print(",OPC1_PM1_ugm3,OPC1_PM25_ugm3,OPC1_PM10_ugm3,OPC1_SamplePeriod_s,OPC1_ChecksumOK");
  }
  if (opc2IsActive) {
    Serial.print(",OPC2_PM1_ugm3,OPC2_PM25_ugm3,OPC2_PM10_ugm3,OPC2_SamplePeriod_s,OPC2_ChecksumOK");
  }
  Serial.println();

  Serial.println("=====================================");
}
// ------------------------------------------
void serial_geofix() {
  Serial.println("=====================================");

  Serial.println("GNSS (GPS) geofix info:");
  byte fixType = g_GNSS.getFixType();  // Get the fix type

  Serial.print("Datetime: ");
  Serial.println(timestamp);


  Serial.print(F("| Fix: "));
  Serial.print(fixType);
  if (fixType == 0)
    Serial.print(F("(No fix)"));
  else if (fixType == 1)
    Serial.print(F("(Dead reckoning)"));
  else if (fixType == 2)
    Serial.print(F("(2D)"));
  else if (fixType == 3)
    Serial.print(F("(3D)"));
  else if (fixType == 4)
    Serial.print(F("(GNSS + Dead reckoning)"));
  Serial.println();

  long latitude = g_GNSS.getLatitude();
  long longitude = g_GNSS.getLongitude();
  long altitude = g_GNSS.getAltitude();

  Serial.print(F("| Lat,Long: ("));
  Serial.print(latitude / 10000000.0, 7);
  Serial.print(F(","));
  Serial.print(longitude / 10000000.0, 7);
  Serial.println(F(") (degrees)"));

  Serial.print(F("| Alt: "));
  Serial.print(altitude / 1000);
  Serial.println(F(" (m)"));

  Serial.print(F("| Time Zone: "));
  Serial.print(LTzone);
  Serial.println(F(" (h)"));

  Serial.println("=====================================");
}
// ------------------------------------------
void serial_dump() {
  Serial.print(timestamp);
  Serial.print(", ");
  Serial.print(gpsStaleCount);
  if (adsIsActive) {
    Serial.print(", ");
    Serial.print(vBat);
  }
  if (opc1IsActive) {
    Serial.print(", ");
    if (!isnan(opc1Pm1)) Serial.print(opc1Pm1, 3);
    Serial.print(", ");
    if (!isnan(opc1Pm25)) Serial.print(opc1Pm25, 3);
    Serial.print(", ");
    if (!isnan(opc1Pm10)) Serial.print(opc1Pm10, 3);
    Serial.print(", ");
    if (opc1LastReadOk) Serial.print(opcHist1.samplingPeriodS, 2);
    Serial.print(", ");
    Serial.print(opc1LastReadOk && opcHist1.checksumOk ? 1 : 0);
  }
  if (opc2IsActive) {
    Serial.print(", ");
    if (!isnan(opc2Pm1)) Serial.print(opc2Pm1, 3);
    Serial.print(", ");
    if (!isnan(opc2Pm25)) Serial.print(opc2Pm25, 3);
    Serial.print(", ");
    if (!isnan(opc2Pm10)) Serial.print(opc2Pm10, 3);
    Serial.print(", ");
    if (opc2LastReadOk) Serial.print(opcHist2.samplingPeriodS, 2);
    Serial.print(", ");
    Serial.print(opc2LastReadOk && opcHist2.checksumOk ? 1 : 0);
  }

  Serial.println("");
}
