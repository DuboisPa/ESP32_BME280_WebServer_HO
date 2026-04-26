/*********
  
   target : uPesy ESP32 Wroom Low Power      
            https://www.upesy.fr/blogs/tutorials/upesy-esp32-wroom-low-power-devkit-board-documentation-version-latest#
            LCD2004 display on I2C
            transducer BME280 on I2C
            transducer DS18B20 oneWire
            resistor 4.7 kOhm
            temporary push-button to set on/off the display backlight
            temporary push-button to power on/off
            voltage regulator LM2596D
            battery Liion 3.7 volts 1000 mAh or more
            connectors

   Partition Scheme : Minimal SPIFFS(1.9MB APP with OTA/190KB SPIFFS)
      don't forget to mount SPIFFS

*********/

// Import required libraries
// if the library is global   use <>
// if the library is local    use ""

#include <algorithm>          
#include <deque>
//#include <iostream>
#include <time.h>
#include <sys/time.h>

#include <Adafruit_BME280.h>  // https://www.waveshare.com/wiki/BME280_Environmental_Sensor
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>      // https://arduinojson.org/
#include <AsyncTCP.h>
#include <DallasTemperature.h>
#include <ElegantOTA.h>       // warning if library is updated https://docs.elegantota.pro/getting-started/async-mode
#include <ESPAsyncWebServer.h>   
#include <ESPmDNS.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <Wire.h>

#define HOSTNAME "ESP32_Weather_Nmea0183"

// Declaration for Wire and/or Wire1 to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// Hitlego Esp32S : SCL on GPIO22, SDA on GPIO21
#define SCL0_GPIO          22       // BME280 SCK  pin 33
#define SDA0_GPIO          21       // BME280 SDI  pin 36
#define SCL1_GPIO          33       // LCD
#define SDA1_GPIO          32       // LCD

#define buttonResetPin      0       // (pin 25)
#define buttonDisplayPin   17       // (pin 28)
#define buttonSleepPin     13       // GPIO 13 and 3.3 volts
#define batteryPin         35       // read battery tension 35
#define batteryCoeff    1.435       // or 1.37
#define BuiltInLed          2       // GPIO 02

// HardwareSerial on passe les GPIO !
#define rx0GPIO              3     // pin 34 
#define tx0GPIO              1                  // pin 35 
//#define rx1GPIO 14               // pin 12 mapped, as native rx1GPIO  9 is used to flash
//#define tx1GPIO 12               // pin 13 mapped, as native tx1GPIO 10 is used to flash
//#define rx2GPIO 16               // pin 27 
//#define tx2GPIO 17               // pin 28 
//HardwareSerial SerialPort1(1);
//HardwareSerial SerialPort2(2);

#define baudrateSerial  38400       // vitesse communication Arduino - PC

// https://raw.githubusercontent.com/nayarsystems/posix_tz_db/master/zones.csv
//#define ntpServer "europe.pool.ntp.org"
//#define My_Time_Zone "GMT0BST,M3.5.0/1,M10.5.0"                       // "Europe/London"
// #define My_Time_Zone "CET-1CEST,M3.5.0,M10.5.0/3"                    // "Europe/Paris"

//#define OneDay                        86400UL
//#define ThirtyDaysInSeconds         2592000UL

#define readBMEDuration               60000UL                           // 60 secondes
#define _1HOUR                         3600UL
//#define logDataInMemoryDuration      900000UL                           // 15 minutes
//#define MAX_MEASURES                      672                           // 7 days : 4 per hour * 24 * 3 
#define logDataInMemoryDuration     1800000UL                           // 30 minutes
#define MAX_MEASURES                      336                           // 7 days : 2 per hour * 24 * 3 
//#define MAX_MEASURES                      672                           // 14 days : 2 per hour * 24 * 3 

struct oneRecord { 
  uint32_t seconds;
  float water_temperature;
  float air_temperature;
  float humidity;
  float pressure;
}; 

std::deque<oneRecord> weatherHistory;

#define wSettings_FILE "weatherSettings.json"                           // SPIFFS files are case-sensitive

#define CONFIGRESET     0
#define CONFIGREAD      1
#define CONFIGWRITE     2
JsonDocument jConfig;
String jsonReplyString;

bool bDisplay = false, bDisplayOn = true;
volatile bool bButtonDisplayPressed = false, bButtonResetPressed = false, bButtonSleepPressed = false;

String sInternetAddress;
float fWTemp, fATemp, fAHumi, fAP, fAPressure;

uint32_t newMillis, oldBMEMillis, oldHistoryMillis, oldDisplayMillis, oldSleepMillis, oldWeatherMillis;

bool bSerial0Port, bSerial1Port, bSerial2Port; 
uint32_t baudrateSerial0, baudrateSerial1, baudrateSerial2;  
bool bNMEA2K, bWWeather, bAWeather, bWIXDR, bIIMDA;
uint32_t tempoWeather;

struct tm timeinfo, startTimeInfo;

String APSTAHostName;
uint8_t nbAPclients;
bool bAPSTAmode, bSTAmode, bAPmode, bUDP, bDHCP, bStatic, bBattery;
bool b2ndUDP = false;
IPAddress STAlocalIP, STAsubnetMaskIP, STAgatewayIP, STAbroadcastIP,
         APlocalIP, APsubnetMaskIP, APgatewayIP, APbroadcastIP, APdhcp_startIP,
         localIP, subnetMaskIP, gatewayIP;
uint16_t UDPPort, delayLCD;
String Global_SSID_NAME, Global_PASSWORD;

//The udp library class
WiFiUDP STAudp, APudp;                                                  // WiFiUDP est un typedef de NetworkUDP

bool bPrepareData = false;
bool bDataReady = false;
uint8_t nbOfHours;

// Initialize I2C BME280 sensor.
#define SEALEVELPRESSURE_HPA 1013.25f
Adafruit_BME280 bme;

// --- Configuration pour la sonde de température ---
#define ONE_WIRE_BUS 4                       // GPIO 4 pin 26                 
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// use this for I2C LCD.
#define Lcd_I2C_Addr    0x27
#define numCols         20
#define numRows         4
LiquidCrystal_I2C display(Lcd_I2C_Addr, numCols, numRows);                  // only Wire, not Wire1

// Create AsyncWebServer object on port 8091 instead of 80 already used on the network
#define Web_Server_Port       80
AsyncWebServer server(Web_Server_Port);

/* function prototype declarations */
void AddChecksum(char* msg);
void Display_Init(void);
void DisplayIMessage(String sMessage, bool bClear, bool bCRLN);
void AirData();
void GetWeatherData();
void WaterData();
void readWeatherData(int16_t idx);
int16_t searchIdxWeatherHistory(uint32_t nValue);
void Storage_Init(void);
void WiFi_Init(void);
void WSettingsRW(JsonDocument& jnewConfig, uint8_t configValue);
/* end declarations */

void IRAM_ATTR onButtonSleepEvent() {
   bButtonSleepPressed = millis() > oldSleepMillis + 250;
}

void IRAM_ATTR onButtonResetEvent() {                                        // if button pressed, reset settings to default                        
   bButtonResetPressed = true;
}

void IRAM_ATTR onButtonDisplayEvent() {
   static uint32_t tempMillis;
   tempMillis = millis();
   if (tempMillis > oldDisplayMillis + 250) {
      oldDisplayMillis = tempMillis;
      bButtonDisplayPressed = true;
   }
}

void convertFromJson(JsonVariantConst source, IPAddress& dest) {
  dest.fromString(source.as<const char*>());
}

String readHistoryPressure(uint32_t nSeconds) {
   char buffer[8];
   uint16_t idx = searchIdxWeatherHistory(nSeconds);
   if (idx > 0) {
      snprintf(buffer, sizeof(buffer), "%+6.1f", fAP - weatherHistory[idx].pressure);
      return String(buffer);
   }
   return "------";
}

void setup() {
   btStop();
   Serial.begin(baudrateSerial);
   
   pinMode(buttonSleepPin, INPUT_PULLDOWN);                                   // GPIO13
   attachInterrupt(digitalPinToInterrupt(buttonSleepPin), onButtonSleepEvent, FALLING); 

   pinMode(buttonResetPin, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(buttonResetPin), onButtonResetEvent, RISING);
   
   pinMode(buttonDisplayPin, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(buttonDisplayPin), onButtonDisplayEvent, RISING);
   
   Display_Init();
   Storage_Init();
   WSettingsRW(jConfig, CONFIGREAD);
   WiFi_Init();

   pinMode(batteryPin, INPUT);
   //pinMode(BuiltInLed, OUTPUT);                                         // this led is use to monitor battery charge
   //digitalWrite(BuiltInLed, LOW);                                       // ON after Wifi connection so I set it OFF
   
   if (bWWeather) { 
      // Init DS18B20 water temp
      sensors.begin();                       // OneWire
   }
   if (bAWeather) {
      bool wire1Status = Wire1.begin(SDA0_GPIO, SCL0_GPIO);             // SDA, SCL   
      if (! bme.begin(BME280_ADDRESS, &Wire1)) {                        // 0x77 BlueDot / WaveShare capteur temperature pression humidité
         bme.begin(BME280_ADDRESS_ALTERNATE, &Wire1);                   // 0x76 Gybmep BME-BMP280 capteur temperature pression humidité
      }
   }
   if (bSerial0Port) { Serial.begin(baudrateSerial0, SERIAL_8N1, rx0GPIO, tx0GPIO); } 
   //if (bSerial1Port) { SerialPort1.begin(baudrateSerial1, SERIAL_8N1, rx1GPIO, tx1GPIO); } 
   //if (bSerial2Port) { SerialPort2.begin(baudrateSerial2, SERIAL_8N1, rx2GPIO, tx2GPIO); } 

   // home page with weather values
   server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(SPIFFS, "/index.html", "text/html");
   });
   // read weather values and send them to index.html
   server.on("/loadValues", HTTP_GET, [](AsyncWebServerRequest *request) {
      String jsonString;
      JsonDocument jConfig;
      char tTime[10];
      //strftime(tTime, sizeof(tTime), "J%d %H:%M", &timeinfo);
      snprintf(tTime, sizeof(tTime), "J%02d %02d:%02d", timeinfo.tm_mday -1, timeinfo.tm_hour, timeinfo.tm_min);
      jConfig["date_time"] = tTime; 
      jConfig["water_temperature"] = String(fWTemp,2);
      jConfig["air_temperature"] = String(fATemp,2); 
      jConfig["humidity"] = String(fAHumi,2);
      jConfig["pressure"] = String(fAP,2);
      serializeJson(jConfig, jsonString);  // Convertit JSON en string
      //Serial.println(jsonString);
      request->send(200, "application/json", jsonString);  // Envoi JSON
   }); 

   // historic page
   server.on("/histo.html", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(SPIFFS, "/histo.html", "text/html");
   });
   // return data memory to histo.html
   server.on("/replyData", HTTP_GET, [](AsyncWebServerRequest *request) {
      if (bDataReady) {
         bDataReady = false;
         request->send(200, "application/json", jsonReplyString);  // Envoi JSON
      }
      else {
         JsonDocument jMessage;
         jMessage["message"] = "Data not ready";
         serializeJson(jMessage, jsonReplyString);
         request->send(413, "application/json", jsonReplyString);  // Envoi JSON Insufficient storage
      }
   }); 

   // configuration page
   server.on("/config.html", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(SPIFFS, "/config.html", "text/html");
   });
   // load parameters from JSON in-memory
   server.on("/loadConfig", HTTP_GET, [](AsyncWebServerRequest *request) {
      String jsonString;
      serializeJson(jConfig, jsonString);  // Convertit JSON en string
      request->send(200, "application/json", jsonString);  // Envoi JSON
   }); 

   server.onRequestBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      if (request->url() == "/requestData") {
            // receive request to load data from SPIFFS storage
            JsonDocument jRequestData;
            if (!bPrepareData) {             // only one request at once
               DeserializationError error = deserializeJson(jRequestData, data, len);
               //request->send(200, "text/plain", "Config reçue");
               nbOfHours = jRequestData["nbOfHours"];
               bPrepareData = true;
            }
         }
         else if (request->url() ==  "/saveConfig") {
            // save parameters to JSON file 
            JsonDocument jnewConfig;
            DeserializationError error = deserializeJson(jnewConfig, data, len);
            WSettingsRW(jnewConfig, CONFIGWRITE);
            //request->send(200, "text/plain", "Config reçue");
         }   
   });  

   ElegantOTA.begin(&server);  
   server.begin();
  
   struct timeval now = { .tv_sec = 0, .tv_usec = 0 };                  // 1970-01-01 00:00:00
   settimeofday(&now, NULL);
   getLocalTime(&startTimeInfo);
   oldBMEMillis = oldHistoryMillis = oldDisplayMillis = oldSleepMillis = oldWeatherMillis = millis();

   GetWeatherData();
   oneRecord record = {0, fWTemp, fATemp, fAHumi, fAP};
   weatherHistory.push_back(record);                                    // add time zero first record to end
}

void loop() {
   static uint32_t nSeconds;
   time_t now;
   char nmea0183string[128]; 

   if (bButtonDisplayPressed) {
      bButtonDisplayPressed = false;
      bDisplayOn = ! bDisplayOn;                                        // if true then false; if false then true
      display.setBacklight(bDisplayOn);
   }

   if (bButtonSleepPressed) {
      bButtonSleepPressed = false;
      display.setBacklight(false);
      Wire.end();
      detachInterrupt(digitalPinToInterrupt(buttonSleepPin));
      // Configure le GPIO x comme source de réveil quand la tension vaut 3.3V
      esp_sleep_enable_ext0_wakeup((gpio_num_t)buttonSleepPin, HIGH);         // alternative, un seul GPIO
      // esp_sleep_enable_ext1_wakeup(0x1000, ESP_EXT1_WAKEUP_ANY_HIGH);      // 2^12 = 0x1000 . si plusieurs GPIO 2^GPIOn1 + 2*^GPIOn1 + 2^GPIOnx >> hex
      esp_deep_sleep_start();  
   }

   if (bButtonResetPressed) { WSettingsRW(jConfig, CONFIGRESET);}       // reset ESP32 to default settings

   newMillis = millis();
   if (newMillis > (oldBMEMillis + readBMEDuration)) {                  // refresh Display
      oldBMEMillis = newMillis;
      GetWeatherData();
      time(&now);
      //localtime_r(&now,&timeinfo);
      gmtime_r(&now,&timeinfo);
      nSeconds = difftime(mktime(&timeinfo), mktime(&startTimeInfo)); 
      
      snprintf(nmea0183string, sizeof(nmea0183string), "%7.2fhPa   %6.2f%c", fAP, fATemp, byte(223));
      DisplayIMessage(nmea0183string, true, true);
      snprintf(nmea0183string, sizeof(nmea0183string), "03h%s  06h%s", readHistoryPressure(nSeconds - 3 * _1HOUR), readHistoryPressure(nSeconds - 6 * _1HOUR));
      DisplayIMessage(nmea0183string, false, true);
      snprintf(nmea0183string, sizeof(nmea0183string), "12h%s  24h%s", readHistoryPressure(nSeconds - 12 * _1HOUR), readHistoryPressure(nSeconds - 24 * _1HOUR));
      DisplayIMessage(nmea0183string, false, true);
      snprintf(nmea0183string, sizeof(nmea0183string), "48h%s  J%02d %02d:%02d", readHistoryPressure(nSeconds - 48 * _1HOUR), timeinfo.tm_mday -1, timeinfo.tm_hour, timeinfo.tm_min);
      DisplayIMessage(nmea0183string, false, true);
      
      // read battery tension
      if (bBattery) {
         float fBatteryVoltage = batteryCoeff * (analogRead(batteryPin) / 4095)*3.3;
         Serial.println(fBatteryVoltage);
      }
   }

   if (newMillis > (oldWeatherMillis + tempoWeather)) {                 // send nmea0183 if enabled, at planned interval
      oldWeatherMillis = newMillis;
      if (bWIXDR && bWWeather) { WaterData();}
      if (bWIXDR && bAWeather) { AirData();}
   }   

   if (newMillis > (oldHistoryMillis + logDataInMemoryDuration)) {      // store history in memory, for histo.html and Display
      oldHistoryMillis = newMillis;
      if (weatherHistory.size() == MAX_MEASURES) {
         weatherHistory.pop_front();         // remove first record
      }
      oneRecord record = {nSeconds+59, fWTemp, fATemp, fAHumi, fAP};
      weatherHistory.push_back(record);      // add record to end
   }

   if (bPrepareData) {                                                  // histo.html page
      bPrepareData = false;
      readWeatherData(searchIdxWeatherHistory(nSeconds - nbOfHours * _1HOUR));
   }
   ElegantOTA.loop();
}

int16_t searchIdxWeatherHistory(uint32_t nValue) {                      // search a record in history
   int w = weatherHistory.size();
   for (int16_t x = 0; x < w; x++){
      if (weatherHistory[x].seconds >= nValue) return x;
   }
   return 0;
}

void readWeatherData(int16_t idx){                                      // read records in history to send to histo.hml page
   JsonDocument jReplyData;

   for (uint16_t i = idx; i < weatherHistory.size(); i++) {
      JsonObject obj = jReplyData.add<JsonObject>();
      obj["s"] = weatherHistory[i].seconds;                     
      obj["w"] = weatherHistory[i].water_temperature; 
      obj["a"] = weatherHistory[i].air_temperature; 
      obj["h"] = weatherHistory[i].humidity;    
      obj["p"] = weatherHistory[i].pressure;    
   }
   serializeJson(jReplyData, jsonReplyString);     
   //Serial.println(jsonReplyString);
   bDataReady = true;
}

void AddChecksum(char* msg) {                                           // Nmea0183 checksum
   unsigned int i=1;        // First character not included in checksum
   uint8_t tmp, chkSum = 0;
   char ascChkSum[5];       // 5 instead of 4

   while (msg[i] != '\0') {
      chkSum ^= msg[i++];
   }
   ascChkSum[0] = '*';
   ascChkSum[3] = '\r';     // added, else problem in qtVlm
   ascChkSum[4] = '\0';
   tmp = chkSum / 16;
   ascChkSum[1] = tmp > 9 ? 'A' + tmp-10 : '0' + tmp;
   tmp = chkSum % 16;
   ascChkSum[2] = tmp > 9 ? 'A' + tmp-10 : '0' + tmp;
   strcat(msg, ascChkSum);
}

/**************************************
   temperature pressure humidity 
       
   $II                                       // Integrated Instrumentation
   $WI                                       // Weather Instrument
   $..MTW                                    // Mean Temperature of Water
   $..XDR                                    // Transducer Values
   $..MDA                                    // Meteorological Composite, obsolete as of 2009
                                             // https://gpsd.gitlab.io/gpsd/NMEA.html#_mda_meteorological_composite

   $WIMTW,19.52,C*checksum                   // Mean Temperature of Water
   $WIXDR,C,19.52,C,TempAir*checksum         // air temperature
   $WIXDR,C,19.52,C,ENV_OUTAIR_T*checksum    // air temperature
   $WIXDR,C,19.52,C,ENV_WATER_T*checksum     // water temperature
   $WIXDR,P,1.02481,B,Barometer*checksum     // pressure
   $WIXDR,H,66.66,P,ENV_OUTAIR_T*checksum    // humididity

   $WIXDR,C,%.2f,C,ENV_OUTAIR_T,H,%.2f,P,ENV_OUTAIR_T,P,%.4f,B,Barometer*checksum  // 3 values
   
**************************************/

void AirData() {
   char nmea0183string[128]; 
    
   snprintf(nmea0183string, sizeof(nmea0183string), "$WIXDR,C,%.2f,C,ENV_OUTAIR_T,H,%.2f,P,ENV_OUTAIR_T,P,%.4f,B,Barometer", fATemp, fAHumi, fAPressure);
   AddChecksum(nmea0183string);
   if (bSerial0Port) { Serial.println(nmea0183string); }
   if (bUDP && bSTAmode) {
      STAudp.beginPacket(STAbroadcastIP, UDPPort);  // broadcast_IP
      STAudp.print(nmea0183string);
      STAudp.endPacket();      
   }
   if (bUDP && b2ndUDP && bSTAmode) {
      STAudp.beginPacket(STAbroadcastIP, UDPPort+1);  // broadcast_IP + 1
      STAudp.print(nmea0183string);
      STAudp.endPacket();      
   }
   if (bUDP && bAPmode) {
      APudp.beginPacket(APbroadcastIP, UDPPort);  // broadcast_IP
      APudp.print(nmea0183string);
      APudp.endPacket();      
   }
   if (bUDP && b2ndUDP && bAPmode) {
      APudp.beginPacket(APbroadcastIP, UDPPort+1);  // broadcast_IP + 1
      APudp.print(nmea0183string);
      APudp.endPacket();      
   }
}

void WaterData() {
   char nmea0183string[48];

   snprintf(nmea0183string, sizeof(nmea0183string), "$WIXDR,C,%.2f,C,ENV_WATER_T", fWTemp);
   AddChecksum(nmea0183string);
   if (bSerial0Port) { Serial.println(nmea0183string); }
   if (bUDP && bSTAmode) {
      STAudp.beginPacket(STAbroadcastIP, UDPPort);  // broadcast_IP
      STAudp.print(nmea0183string);
      STAudp.endPacket();      
   }
   if (bUDP && b2ndUDP && bSTAmode) {
      STAudp.beginPacket(STAbroadcastIP, UDPPort+1);  // broadcast_IP + 1
      STAudp.print(nmea0183string);
      STAudp.endPacket();      
   }
   if (bUDP && bAPmode) {
      APudp.beginPacket(APbroadcastIP, UDPPort);  // broadcast_IP
      APudp.print(nmea0183string);
      APudp.endPacket();      
   }
   if (bUDP && b2ndUDP && bAPmode) {
      APudp.beginPacket(APbroadcastIP, UDPPort+1);  // broadcast_IP + 1
      APudp.print(nmea0183string);
      APudp.endPacket();      
   }
}

void GetWeatherData() {                                                 // read BME0280 sensor
   //fAalt  = -bme.readAltitude(SEALEVELPRESSURE_HPA);
   if (bWWeather) {
      bool bOK = sensors.requestTemperatures();
      fWTemp = sensors.getTempCByIndex(0);
   }
   else
      fWTemp = -127.0;
   fATemp = bme.readTemperature();
   fAHumi = round(bme.readHumidity()*100)/100;
   fAP    = bme.readPressure() / 100.0F;
   fAPressure = fAP / 1000.0F;
}

void WSettingsRW(JsonDocument& jnewConfig, uint8_t configValue) {       // read/write/reset settings in/from json file
   
   if (!SPIFFS.exists("/" wSettings_FILE)) { 
      // Serial.println(String(wSettings_FILE) + " doesn't exist");
      configValue = CONFIGRESET;
   }
   
   switch (configValue) {
      case CONFIGRESET : {             // 0
         File file = SPIFFS.open("/" wSettings_FILE, FILE_WRITE);
         jnewConfig["APSTAHostName"] = "ESP32_Weather_Nmea0183";
         jnewConfig["APmode"] = true; 
         jnewConfig["APClients"] = 4; 
         jnewConfig["APPassword"] = "12345678"; 
         jnewConfig["APlocalIP"] = "192.168.4.2";
         jnewConfig["APsubnetMaskIP"] = "255.255.255.0"; 
         jnewConfig["APgatewayIP"] = "192.168.4.99"; 
         jnewConfig["APdhcp_startIP"] = "192.168.4.11";
         jnewConfig["STAmode"] = false; 
         jnewConfig["DHCP"] = true; 
         jnewConfig["Static"] = false; 
         jnewConfig["STASSID"] = "FreeOrangeSFRBouygues"; 
         jnewConfig["STAPassword"] = "12345678";
         jnewConfig["STAlocalIP"] = "192.168.1.64"; 
         jnewConfig["STAsubnetMaskIP"] = "255.255.255.0"; 
         jnewConfig["STAgatewayIP"] = "192.168.1.254"; 
         jnewConfig["WaterSensor"] = true; 
         jnewConfig["AirSensor"] = true; 
         jnewConfig["WIXDR"] = true; 
         jnewConfig["IIMDA"] = false;
         jnewConfig["Battery"] = false; 
         jnewConfig["DelayTime"] = 60;
         jnewConfig["DelayTimeLCD"] = 5;
         jnewConfig["UDPPort"] = 2010; 
         jnewConfig["USBPort"] = 38400; 
         jnewConfig["Serial1"] = 0; 
         jnewConfig["Serial2"] = 0;
         jnewConfig["NMEA2K"] = false;

         // jnewConfig.shrinkToFit();  // optional
         serializeJsonPretty(jnewConfig, file);       // indent 2 spaces
         file.close();
         ESP.restart();
         break;
      }
      case CONFIGREAD : {              // 1
         File file = SPIFFS.open("/" wSettings_FILE, FILE_READ);
         DeserializationError error = deserializeJson(jnewConfig, file);
         /* if (error) {
            Serial.print("deserializeJson() failed: ");
            Serial.println(error.c_str());
            //return;
         } */
         APSTAHostName = jConfig["APSTAHostName"].as<String>(); 
         bAPmode = jConfig["APmode"];
         nbAPclients = jConfig["APClients"];
         bSTAmode = jConfig["STAmode"]; 
         bAPSTAmode = bAPmode && bSTAmode;
         bDHCP = jConfig["DHCP"]; 
         bStatic = jConfig["Static"]; 
         
         convertFromJson(jConfig["APlocalIP"], APlocalIP);
         convertFromJson(jConfig["APsubnetMaskIP"], APsubnetMaskIP);
         convertFromJson(jConfig["APgatewayIP"], APgatewayIP);
         convertFromJson(jConfig["APdhcp_startIP"], APdhcp_startIP);
         convertFromJson(jConfig["STAlocalIP"], STAlocalIP);
         convertFromJson(jConfig["STAsubnetMaskIP"], STAsubnetMaskIP);
         convertFromJson(jConfig["STAgatewayIP"], STAgatewayIP);
         bWWeather = jConfig["WaterSensor"]; 
         bAWeather = jConfig["AirSensor"]; 
         bWIXDR = jConfig["WIXDR"]; 
         bIIMDA = jConfig["IIMDA"]; 
         bBattery    = jConfig["Battery"];
         tempoWeather = jConfig["DelayTime"]; 
         tempoWeather *= 1000; // milliseconds 
         delayLCD    = jConfig["DelayTimeLCD"];
         delayLCD    *= 1000;
         UDPPort = jConfig["UDPPort"];
         bUDP = (UDPPort > 0);
         baudrateSerial0 = jConfig["USBPort"];
         baudrateSerial1 = jConfig["Serial1"];
         baudrateSerial2 = jConfig["Serial2"];
         bSerial0Port = (baudrateSerial0 > 0);
         bSerial1Port = (baudrateSerial1 > 0);
         bSerial2Port = (baudrateSerial2 > 0);
         bNMEA2K = jConfig["NMEA2K"];  

         file.close();
         break;
      }
      case CONFIGWRITE : {             // 2
         File file = SPIFFS.open("/" wSettings_FILE, FILE_WRITE);
         // Serial.println(jnewConfig.as<String>());
         file.print(jnewConfig.as<String>()); 
         file.close();
         ESP.restart();
         break;
      }
   }
}

void WiFi_Init() {                                                      // start wifi in STA and/or AP mode, depending settings
   String strUDP;

   if (bAPSTAmode) { WiFi.mode(WIFI_AP_STA);}
   else if (bSTAmode && !bAPmode) { WiFi.mode(WIFI_STA);}
   else if (bAPmode && !bSTAmode ) { WiFi.mode(WIFI_AP);}
   WiFi.setHostname(APSTAHostName.c_str());
   
   if (bSTAmode) {
      uint32_t oldMillis = millis();
      if (bStatic) {
         WiFi.config(STAlocalIP, STAgatewayIP, STAsubnetMaskIP); // primaryDNS, secondaryDNS))
      }
      Global_SSID_NAME = jConfig["STASSID"].as<String>(); 
      Global_PASSWORD = jConfig["STAPassword"].as<String>();
      WiFi.begin(Global_SSID_NAME, Global_PASSWORD);
      DisplayIMessage("Connecting to ", true, true);
      DisplayIMessage(String(Global_SSID_NAME), false, true);
      do {
         delay(500);
         Serial.print(".");
      }
      while ((WiFi.status() != WL_CONNECTED) && (millis() < oldMillis + 10000));     // 10 seconds
      jConfig["STAlocalIP"] = STAlocalIP = WiFi.localIP();
      jConfig["STAsubnetMaskIP"] = STAsubnetMaskIP = WiFi.subnetMask();
      jConfig["STAgatewayIP"] = STAgatewayIP = WiFi.gatewayIP();
      STAbroadcastIP = WiFi.broadcastIP();
      if (WiFi.status() != WL_CONNECTED) {
         DisplayIMessage("No Station connection", true, true);
         bAPmode = true;
         bSTAmode = false;
         WiFi.mode(WIFI_AP);
      }
      else {
         if (bUDP) {
            STAudp.begin(UDPPort);
            strUDP = String(UDPPort);
         }
         else strUDP = "none";
         DisplayIMessage("IP    " + STAlocalIP.toString(), true, true);
         DisplayIMessage("Mask  " + STAsubnetMaskIP.toString(), false, true);
         //DisplayIMessage("GW    " + STAgatewayIP.toString(), false, true);
         DisplayIMessage("Broad." + STAbroadcastIP.toString(), false, true);
         DisplayIMessage("Port  " + strUDP, false, true);
      }
      delay(2000);
   }

   if (bAPmode) {
      // bool softAPConfig(IPAddress local_ip, IPAddress gateway, IPAddress subnet, IPAddress dhcp_lease_start = (uint32_t) 0);
      // bool softAP(const char* ssid, const char* passphrase = NULL, int channel = 1, int ssid_hidden = 0, int max_connection = 4, bool ftm_responder = false);
      Global_SSID_NAME = APSTAHostName.c_str();
      Global_PASSWORD = jConfig["APPassword"].as<String>();
      DisplayIMessage("Starting Autonomous", true, true);
      delay(1000);
      WiFi.softAPConfig(APlocalIP, APgatewayIP, APsubnetMaskIP, APdhcp_startIP);
      WiFi.softAP(Global_SSID_NAME, Global_PASSWORD, 1, 0, nbAPclients, false); 
      APlocalIP = WiFi.softAPIP();
      APsubnetMaskIP = WiFi.softAPSubnetMask();
      APbroadcastIP = WiFi.softAPBroadcastIP();
      if (bUDP) {
         APudp.begin(UDPPort);
         strUDP = String(UDPPort);
      }
      else strUDP = "none";
      DisplayIMessage("IP    " + APlocalIP.toString(), true, true);
      DisplayIMessage("Mask  " + APsubnetMaskIP.toString(), false, true);
      //DisplayIMessage("GW    " + APgatewayIP.toString(), false, true);
      DisplayIMessage("Broad." + APbroadcastIP.toString(), false, true);
      DisplayIMessage("Port  " + strUDP, false, true);
      delay(2000);
   }
   MDNS.begin(APSTAHostName.c_str());
}

void Storage_Init() {                                                   // check if an existing SPIFFS partition containang html pages and json file settings
   if(!SPIFFS.begin(true))
      DisplayIMessage("Storage Mount Failed", true, true);
   else {
      DisplayIMessage("Storage Mount OK", true, true);
   }
   delay(1500);
}

void Display_Init() {                                                   // start LCD
  // You can also pass in a Wire library object like &Wire1
   bool wire0Status = Wire.begin(SDA1_GPIO, SCL1_GPIO);                 // SDA, SCL for LCD2004, kows only Wire, not Wire1
   display.init();                                                      // I2C LCD init command
   bDisplay = true;
   display.setBacklight(bDisplayOn);
   display.setCursor(0, 0); display.print(HOSTNAME);   
   display.setCursor(1, 2); display.print("version 2026-04-18");
   display.setCursor(3, 3); display.print("by Patrick D.");   
   delay(1500);
   display.clear();
}

void DisplayIMessage(String sMessage, bool bClear, bool bCRLN) {        // display messages on LCD
   static int nRow = 0;

   if (bDisplay) {
      if (bClear) {
         display.clear();
         display.setCursor(0, nRow=0); 
      }
      if (bCRLN) { 
         display.setCursor(0, nRow++); 
      }
      display.print(sMessage); 
   } 
   else
      Serial.println(sMessage); 
}