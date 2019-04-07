/*
 * AUTHOR DEAN GREENHOUGH  07/09/2019
 * INITAL WATER SOFTENER PROJECT
 * ESP32, VL53, I2C, SPI, ePAPER DISPLAY, ESP DEEP SLEEP, NTP, WDT, WIFI MANAGER, PROWL NOTIFICATION, JSON, MQTT, AVERAGING SAMPLES, CALIBRATION, SPIFFS 
 * LESSONS LEARNED 
 * ABANDONED DUE TO POWER DEEP SLEEP CURRENT BEING 4.8uA  AND NOT SUITABLE FOR PROPOSED PROJECT.
 * THE VL53'S HAVE PROVED TROUBLESOME WITH ALTERATIONS TO THE CORE CODE TO PREVENT SYSTEM CRASH ON I2C BUS AND THE PROBLEMS ASSOSCIATED WITH MODIFYING THE CORE HAL CODE.
 * THE PROBLEM CAN BE SOLVED USING A I2C MULTIPLIER AND WILL OVERCOME MODIFICATION OF THE CORE CODE.......MODULE PURCHASED TO BE TESTED
 * ePAPER DISPLAY REQUIRES FURTHER EXPLORATION AND PARTIAL UPDATES WILL PROVE TO BE IMPORTANT AS FULL DISPLAY UPDATE TAKES 10Secs 
 * THIS CODE IS WORKING STABLE AND JUST DEMONSTRATES A WORK IN PROGRESS AND IS NOT ELEGANT BY ANY STANDARD
 * THIS WAS ABANDONED FOR A FRESH START IN THE HOPE OF CONDENSING THE CODE, USING NEWLY DISCOVERED HARDWARE TO SOLVE THE VL53 CONFLICT 
 * THE ABILITY TO CARRY OUT OTA UPDATES AND FIRMWARE UPGRADES TO uCONTROLERS IN THE FIELD AUTOMATICALY  REQUIRES A DIFFERENT APPROACH.
 * 
 */

#include <FS.h>                   //this needs to be first, or it all crashes and burns...
#include <SPIFFS.h> 
#if defined(ESP8266)
#include <ESP8266WiFi.h>          
#else
#include <WiFi.h>                 
#endif

//needed for library
#include <DNSServer.h>
#if defined(ESP8266)
#include <ESP8266WebServer.h>
#else
#include <WebServer.h>
#endif
#include <WiFiManager.h>     
#include <ArduinoJson.h>   
#include <EspProwl.h>
//WIFI & BT SHUTDOWN
#include <esp_wifi.h>
//#include <esp_bt.h>
//WDT
#include "esp_system.h"
const int wdtTimeout = 30000;  //time in mS to trigger the watchdog = 30SECS
hw_timer_t *timer = NULL;

//NTP
#include "time.h"
//const char* ntpServer = "pool.ntp.org";
//const long  gmtOffset_sec = 0;
//const int   daylightOffset_sec = 3600;
String time_str; //GLOBAL TIME STRING
//RTC MEMORY
void IRAM_ATTR resetModule() {
  ets_printf("reboot\n");
  esp_restart();
}

//DEBUG
#define DEBUG 1
// ADC SETUP
#include <driver/adc.h>
//DISPLAY 
//BUSY -> 4, RST -> 16, DC -> 17, CS -> SS(5), CLK -> SCK(18), DIN -> MOSI(23), GND -> GND, 3.3V -> 3.3V

#include <GxEPD.h>
//#include <GxGDEP015OC1/GxGDEP015OC1.cpp>    // 1.54" b/w
#include <GxGDEW0154Z04/GxGDEW0154Z04.cpp>  // 1.54" b/w/r
#define HAS_RED_COLOR
// DISPLAY FONTS Adafruit_GFX
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
//#include <Fonts/FreeMonoBold18pt7b.h>
//#include <Fonts/FreeMonoBold24pt7b.h>
#include <GxIO/GxIO_SPI/GxIO_SPI.cpp>
#include <GxIO/GxIO.cpp>
//////////////////////////////////////////////////
#include <Wire.h>
#include <VL53L0X.h>
//CREATE INSTANCES VL53
VL53L0X LEFT_BLOCK;
VL53L0X RIGHT_BLOCK;
//////////////////////////////////////////////////
#if defined(ESP8266)
GxIO_Class io(SPI, SS, 0, 2); 
GxEPD_Class display(io); 

#elif defined(ESP32)
GxIO_Class io(SPI, SS, 17, 16);
GxEPD_Class display(io, 16, 4); 

#else
GxIO_Class io(SPI, SS, 8, 9); 
GxEPD_Class display(io);
#endif  

//SET DEEP SLEEP TIMER
#define uS_TO_S_FACTOR 1000000LL    /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10 //60*60*24         //60*60*24    /* Time ESP32 will go to sleep (in seconds) */
RTC_DATA_ATTR int bootCount = 1;

//define your default values here, if there are different values in config.json, they are overwritten.
//char mqtt_server[40];
//char mqtt_port[6] = "8080";
char blynk_token[44] = "YOUR_PROWL_TOKEN";

//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}
#define TRIGGER_PIN 32
//#define DEBUG 1

//WIFI ATTEMPT CONNECT TIMER
//int counter      = 1;
//AVERAGING
int averageLeft  = 0;
int averageRight = 0;
//CALIBRATION
int calibrateLeft  = 0;
int calibrateRight = 0;

void setup() {
 
  Serial.begin(115200);  
  Serial.println("SKETCH WS_Prototype_1.5.2 ADDED:alter display, remove battery, show last update, added NTP function WITH MODIFIED HAL FILES");
  
  //ADC NEED TO BUILD RESISTOR DIVIDER OR INA219
  /*
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_11db);
  int val = adc1_get_voltage(ADC1_CHANNEL_0);
  delay(500);
  int VCC = map (val, 0, 4095, 0,3300);
  
  Serial.print  ("val ");
  Serial.println (val);
  Serial.print   ("VCC ");
  Serial.println (VCC);
  Serial.println ();
  */
  display.init();
  print_wakeup_reason();

  esp_sleep_enable_timer_wakeup((uint64_t)(TIME_TO_SLEEP) * uS_TO_S_FACTOR); //MODIFIED DUE TO DEEP SLEEP ISSUES
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");
  pinMode(TRIGGER_PIN, INPUT);

  if (digitalRead(TRIGGER_PIN) == HIGH) {  OnDemand();   }
  //WDT
  timer = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(timer, &resetModule, true);  //attach callback
  timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
  timerAlarmEnable(timer);							            //enable interupt
  timerWrite(timer, 0);                             //reset timer (feed watchdog)
  long loopTime = millis();
  Serial.println("BEGIN WDT TIMEOUT");
  
  //clean FS, for testing
  //SPIFFS.format();
  
  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {//ADDED true as had error 10025 mounting issue, this forces formatOnFAil
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          //strcpy(mqtt_server, json["mqtt_server"]);
          //strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(blynk_token, json["blynk_token"]);

        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read



  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  //WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  //WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_blynk_token("blynk", "blynk token", blynk_token, 44);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  wifiManager.setDebugOutput(true);

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
  //wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
  
  //add all your parameters here
  //wifiManager.addParameter(&custom_mqtt_server);
  //wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_blynk_token);

  //reset settings - for testing
  //wifiManager.resetSettings();

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();
  
  

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration

  
  //NEED TO WRITE ERROR HANDLING ROUTINE
  //******************************************************************************************
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
    wifiManager.setTimeout(300);
  
  if (!wifiManager.autoConnect("AutoConnectAP", "password")) {
    Serial.println("failed to connect and hit timeout");
    delay(500);
    //reset and try again, or maybe put it to deep sleep
    ESP.restart();
    delay(500);
  //******************************************************************************************
  }

  //if you get here you have connected to the WiFi
  //Serial.println("connected to WiFi");

  //read updated parameters
  //strcpy(mqtt_server, custom_mqtt_server.getValue());
  //strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(blynk_token, custom_blynk_token.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    //json["mqtt_server"] = mqtt_server;
    //json["mqtt_port"] = mqtt_port;
    json["blynk_token"] = blynk_token;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }
  //Serial.print("AutoConnect ip ");
  //Serial.println(WiFi.localIP());
  //MOSFET LOW POWER
  pinMode(27, OUTPUT);
  digitalWrite(27, HIGH);////////////////////////////////////////////////MOSFET
  delay(500);
  //SET XSHUT PINS
  pinMode(33, OUTPUT);
  pinMode(19, OUTPUT);
  digitalWrite(33, LOW);
  digitalWrite(19, LOW);  
  
  Wire.begin();
  delay(500);
 
  //SET PIN MODES & i2c
  pinMode(33, INPUT);
  delay(150); //DO NOT CHANGE TIMINGS CRITICAL
  //LEFT BLOCK INIT
  LEFT_BLOCK.init(true);
  delay(150); //DO NOT CHANGE TIMINGS CRITICAL
  LEFT_BLOCK.setAddress((uint8_t)22);
  pinMode(19, INPUT);
  delay(150); //DO NOT CHANGE TIMINGS CRITICAL
  //RIGHT BLOCK INIT
  RIGHT_BLOCK.init(true);
  delay(150); //DO NOT CHANGE TIMINGS CRITICAL
  RIGHT_BLOCK.setAddress((uint8_t)25);
  delay(150); //DO NOT CHANGE TIMINGS CRITICAL
    
  LEFT_BLOCK.setTimeout (500);
  RIGHT_BLOCK.setTimeout(500);
  LEFT_BLOCK.setMeasurementTimingBudget (20000);
  RIGHT_BLOCK.setMeasurementTimingBudget(20000);
  
  //BEGIN MEASUREMENTS 
     //AVERAGING ROUTINE
     averageLeft  = average_left  (5,20);
     averageRight = average_right (5,20);

     //****************************************CALIBRATION ROUTINE****************************************
     calibrateLeft  = ( averageLeft -11);
     calibrateRight = (averageRight -23);

     Serial.print ("calibrated Left   = ");     
     Serial.println(calibrateLeft);
     Serial.print ("calibrated Right  = ");
     Serial.println(calibrateRight);
      
       
  //************PROWL PUSH NOTIFICATION************
  EspProwl.begin();
  // For Prowl, go to
  //   https://www.prowlapp.com/api_settings.php
  // to create an API key.
  // If you don't, the server will return a 401 error code.
  
  EspProwl.setApiKey("************************************");
  
  //EspProwl.setApiKey(blynk_token);
  EspProwl.setApplicationName("WATER SOFTENER");
  //DEBUG
  Serial.print("PROWL_TOKEN = ");
  Serial.println(blynk_token);
  

  

  if (calibrateLeft >350  && calibrateLeft <400)
     {
     if (DEBUG) Serial.print("Sending push notification  LEFT BLOCK: RESULT =  ");
      int returnCode = EspProwl.push("LEFT BLOCK LOW", "", 0);
    if (returnCode == 200) {
    if (DEBUG) Serial.println("OK.");
    } else {
      if (DEBUG) Serial.print("Error. Server returned: ");
      if (DEBUG) Serial.println(returnCode);
    }         
     }
 
  if (calibrateRight >350 && calibrateRight <400)
     {     
      if (DEBUG) Serial.print("Sending push notification RIGHT BLOCK: RESULT =  ");
      int returnCode = EspProwl.push("RIGHT BLOCK LOW", "", 0);
    if (returnCode == 200) {
    if (DEBUG) Serial.println("OK.");
    } else {
      if (DEBUG) Serial.print("Error. Server returned: ");
      if (DEBUG) Serial.println(returnCode);
    }               
     }
  //************PROWL PUSH NOTIFICATION END*********
  
  
    
   // Serial.println();
   
  
  // SHUTDOWN WIFI BT
  //.println(" WIFI & BT DISABLED");
  //esp_wifi_stop();
  //esp_bt_controller_disable();
  //BOOT COUNT
  Serial.println();
  Serial.print("***** Boot number: " + String(bootCount));
  Serial.println(" *****");
  Serial.println();
  ++bootCount;
  //Serial.println();
  
  //GLOBAL TIME
  configTime(0, 0, "pool.ntp.org");
  //Serial.println(printLocalTime());
  //NTP
  
  setenv("TZ", "GMT0BST,M3.4.0/01,M10.4.0/02", 1); // You must include '0' after first designator e.g. GMT0GMT-1, ',1' is true or ON
												   // Serial.println("UK Time = "+printLocalTime());


  //CALL FUNCTIONS    
  SALT_BLOCK_READ();
  UPDATE_BLOCK_LEVELS();

  // Serial.println(printLocalTime());
  Serial.println();
  Serial.print("time_str = ");
  Serial.println(time_str);
  Serial.println();
  //MOSFET LOW POWER
  digitalWrite(27, LOW);

  //WDT
  loopTime = millis() - loopTime;
  Serial.print("***** LOOPTIME = ");
  Serial.print(loopTime); //should be under 30000
  Serial.println(" *****");
  Serial.println();

  //ENTER DEEP SLEEP
    Serial.println("ENTERING DEEP SLEEP");
    esp_deep_sleep_start();
    Serial.println("This will never be printed");

}

void loop() {
  // LOW POWER, SO LOOP UNUSED


}

//GET GLOBAL_DATE_TIME
String printLocalTime() {
	struct tm timeinfo;
	if (!getLocalTime(&timeinfo)) {
		Serial.println("Failed to obtain time");
		return "NTPtime Error";
	}
	//See http://www.cplusplus.com/reference/ctime/strftime/
	char output[80];
	strftime(output, 80, "%d %b   %H:%M", &timeinfo);
	time_str = String(output);
	return String(time_str);
}


//READ SALT BLOCKS, SET UP 3 BOXES
void SALT_BLOCK_READ()
{
  const GFXfont* s = &FreeMonoBold9pt7b;
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);
  display.setFont(s);
  //LEFT BLOCK
  display.setCursor(20, 30);
  display.print(printLocalTime());
  //display.print(averageLeft);
  //RIGHT BLOCK
  display.setCursor(140, 30);
  //display.print(averageRight);
 
  
  //DRAW OUTLINE LEFT SALT BLOCK
  display.drawRect(5,   50, 90, 145, GxEPD_BLACK);
  //DRAW OUTLINE RIGHT SALT BLOCK
  display.drawRect(105, 50, 90, 145, GxEPD_BLACK);
  //TOP OUTLINE BOX
  display.drawRect(5,   5, 190, 40, GxEPD_BLACK);  
  ////////////////////////////////////////////////////////////////////////
  /*
  //BATTERY OUTLINE 
  display.drawRect(10, 10, 80, 30, GxEPD_BLACK);
  //BOTTOM 0
  display.drawRect(12, 12, 10, 26, GxEPD_BLACK);
  display.fillRect(12, 12, 10, 26, GxEPD_BLACK);
  //LEVEL 1
  display.drawRect(25, 12, 10, 26, GxEPD_BLACK);
  display.fillRect(25, 12, 10, 26, GxEPD_BLACK);
  //LEVEL 2
  display.drawRect(38, 12, 10, 26, GxEPD_BLACK);
  display.fillRect(38, 12, 10, 26, GxEPD_BLACK);
  //LEVEL 3
  display.drawRect(51, 12, 10, 26, GxEPD_BLACK);
  display.fillRect(51, 12, 10, 26, GxEPD_BLACK);
  //LEVEL 4
  display.drawRect(64, 12, 10, 26, GxEPD_BLACK);
  display.fillRect(64, 12, 10, 26, GxEPD_BLACK);
  //LEVEL 5
  display.drawRect(77, 12, 10, 26, GxEPD_BLACK);
  display.fillRect(77, 12, 10, 26, GxEPD_BLACK);

  //BATTERY TOP
  display.drawRect(90, 17,  5, 15, GxEPD_BLACK);
  display.fillRect(90, 17,  5, 15, GxEPD_BLACK);
  */
  ////////////////////////////////////////////////////////////////////////
}

  void UPDATE_BLOCK_LEVELS() 
 {
//LEFT SALT BLOCK DISPLAY LEVELS  

 if (calibrateLeft <=70)
      { //100% 
        display.drawRect(10,  84, 80, 24, GxEPD_BLACK);
        display.drawRect(10, 113, 80, 24, GxEPD_BLACK);
        display.drawRect(10, 142, 80, 24, GxEPD_BLACK);
        display.drawRect(10, 171, 80, 19, GxEPD_BLACK);      
        
        display.fillRect(10, 55, 80, 24,  GxEPD_BLACK);
        display.fillRect(10, 84, 80, 24,  GxEPD_BLACK);
        display.fillRect(10,113, 80, 24,  GxEPD_BLACK);
        display.fillRect(10,142, 80, 24,  GxEPD_BLACK);
        display.fillRect(10,171, 80, 19,  GxEPD_BLACK); 
        
      }
    
 else if (calibrateLeft <=115)
      { //80%
        display.drawRect(10, 55, 80, 24,  GxEPD_BLACK);
        display.drawRect(10, 84, 80, 24,  GxEPD_BLACK);
        display.drawRect(10,113, 80, 24, GxEPD_BLACK);
        display.drawRect(10,142, 80, 24, GxEPD_BLACK);
        display.drawRect(10,171, 80, 19, GxEPD_BLACK);
        
        display.fillRect(10, 84, 80, 24, GxEPD_BLACK);
        display.fillRect(10,113, 80, 24, GxEPD_BLACK);
        display.fillRect(10,142, 80, 24, GxEPD_BLACK);
        display.fillRect(10,171, 80, 19, GxEPD_BLACK);        
        
      }

 else if (calibrateLeft <=160)
      { //60%
        display.drawRect(10,  55, 80, 24, GxEPD_BLACK);
        display.drawRect(10,  84, 80, 24, GxEPD_BLACK);
        display.drawRect(10, 113, 80, 24, GxEPD_BLACK);
        display.drawRect(10, 142, 80, 24, GxEPD_BLACK);
        display.drawRect(10, 171, 80, 19, GxEPD_BLACK);
              
        display.fillRect(10, 113, 80, 24, GxEPD_BLACK);
        display.fillRect(10, 142, 80, 24, GxEPD_BLACK);
        display.fillRect(10, 171, 80, 19, GxEPD_BLACK);
        
      }
 else if (calibrateLeft <=205)
      { //40% 
        display.drawRect(10,  55, 80, 24, GxEPD_BLACK);
        display.drawRect(10,  84, 80, 24, GxEPD_BLACK);
        display.drawRect(10, 113, 80, 24, GxEPD_BLACK);
        display.drawRect(10, 142, 80, 24, GxEPD_BLACK);
        display.drawRect(10, 171, 80, 19, GxEPD_BLACK);
        
        display.fillRect(10, 142, 80, 24, GxEPD_BLACK);
        display.fillRect(10, 171, 80, 19, GxEPD_BLACK);
       
      }

 else if (calibrateLeft <=245)
      { //20% 
        display.drawRect(10, 55,  80, 24, GxEPD_BLACK);
        display.drawRect(10, 84,  80, 24, GxEPD_BLACK);
        display.drawRect(10, 113, 80, 24, GxEPD_BLACK);
        display.drawRect(10, 142, 80, 24, GxEPD_BLACK);
        display.drawRect(10, 171, 80, 19, GxEPD_BLACK);
        
        display.fillRect(10, 171, 80, 19, GxEPD_BLACK);
        
      }

 else if (calibrateLeft <=300)
      {
        //Serial.println ("LEFT EMPTY"); //FILL RECT WITH RED    
    
        display.drawRect(10,  55, 80, 24, GxEPD_BLACK);
        display.drawRect(10,  84, 80, 24, GxEPD_BLACK);
        display.drawRect(10, 113, 80, 24, GxEPD_BLACK);
        display.drawRect(10, 142, 80, 24, GxEPD_BLACK);
        display.drawRect(10, 171, 80, 19, GxEPD_BLACK); 
        display.fillRect(10, 171, 80, 19, GxEPD_RED);
      }

 else if (calibrateLeft >300)
       {
        //Serial.println ("LEFT EMPTY_2");
        //display.drawRect(10,  55, 80, 24, GxEPD_BLACK);
        //display.drawRect(10,  84, 80, 24, GxEPD_BLACK);
        //display.drawRect(10, 113, 80, 24, GxEPD_BLACK);
        //display.drawRect(10, 142, 80, 24, GxEPD_BLACK);
        //display.drawRect(10, 171, 80, 19, GxEPD_BLACK); 

        display.fillRect(5, 50, 90, 145, GxEPD_RED);
  
        /*
        display.fillRect(10, 55,  80, 24, GxEPD_RED);
        display.fillRect(10, 84,  80, 24, GxEPD_RED);
        display.fillRect(10, 113, 80, 24, GxEPD_RED);
        display.fillRect(10, 142, 80, 24, GxEPD_RED);
        display.fillRect(10, 171, 80, 19, GxEPD_RED);
        */
       }

       
       delay(500);                                        //REQUIRED OR FAILS TO GET VL53 MEASUREMENT
  
 //RIGHT SALT BLOCK DISPLAY LEVELS  
 
  if (calibrateRight <=70)
      { //100% 
       // display.fillRect(110,  55, 80, 24, GxEPD_BLACK);
        display.drawRect(110,  84, 80, 24, GxEPD_BLACK);
        display.drawRect(110, 113, 80, 24, GxEPD_BLACK);
        display.drawRect(110, 142, 80, 24, GxEPD_BLACK);
        display.drawRect(110, 171, 80, 19, GxEPD_BLACK);      
        
        display.fillRect(110,  55, 80, 24, GxEPD_BLACK);
        display.fillRect(110,  84, 80, 24, GxEPD_BLACK);
        display.fillRect(110, 113, 80, 24, GxEPD_BLACK);
        display.fillRect(110, 142, 80, 24, GxEPD_BLACK);
        display.fillRect(110, 171, 80, 19, GxEPD_BLACK); 
        
      }

  else if (calibrateRight <=115)
      { //80%
        display.drawRect(110,  55, 80, 24, GxEPD_BLACK);
        display.drawRect(110,  84, 80, 24, GxEPD_BLACK);
        display.drawRect(110, 113, 80, 24, GxEPD_BLACK);
        display.drawRect(110, 142, 80, 24, GxEPD_BLACK);
        display.drawRect(110, 171, 80, 19, GxEPD_BLACK);
        
        display.fillRect(110,  84, 80, 24, GxEPD_BLACK);
        display.fillRect(110, 113, 80, 24, GxEPD_BLACK);
        display.fillRect(110, 142, 80, 24, GxEPD_BLACK);
        display.fillRect(110, 171, 80, 19, GxEPD_BLACK);        
        
      }

 else if (calibrateRight <=160)
      { //60%
        display.drawRect(110,  55, 80, 24, GxEPD_BLACK);
        display.drawRect(110,  84, 80, 24, GxEPD_BLACK);
        display.drawRect(110, 113, 80, 24, GxEPD_BLACK);
        display.drawRect(110, 142, 80, 24, GxEPD_BLACK);
        display.drawRect(110, 171, 80, 19, GxEPD_BLACK);
              
        display.fillRect(110, 113, 80, 24, GxEPD_BLACK);
        display.fillRect(110, 142, 80, 24, GxEPD_BLACK);
        display.fillRect(110, 171, 80, 19, GxEPD_BLACK);
        
      }
 else if (calibrateRight <=205)
      { //40% 
        display.drawRect(110,  55, 80, 24, GxEPD_BLACK);
        display.drawRect(110,  84, 80, 24, GxEPD_BLACK);
        display.drawRect(110, 113, 80, 24, GxEPD_BLACK);
        display.drawRect(110, 142, 80, 24, GxEPD_BLACK);
        display.drawRect(110, 171, 80, 19, GxEPD_BLACK);
        
        display.fillRect(110, 142, 80, 24, GxEPD_BLACK);
        display.fillRect(110, 171, 80, 19, GxEPD_BLACK);
        
      }

 else if (calibrateRight <=245)
      { //20% 
        display.drawRect(110,  55, 80, 24, GxEPD_BLACK);
        display.drawRect(110,  84, 80, 24, GxEPD_BLACK);
        display.drawRect(110, 113, 80, 24, GxEPD_BLACK);
        display.drawRect(110, 142, 80, 24, GxEPD_BLACK);
        display.drawRect(110, 171, 80, 19, GxEPD_BLACK);
        
        display.fillRect(110, 171, 80, 19, GxEPD_BLACK);
        
      }

 else if (calibrateRight <=300)
      {
        //Serial.println ("RIGHT EMPTY");                       //TODO FILL RECT WITH RED
        display.drawRect(110,  55, 80, 24, GxEPD_BLACK);
        display.drawRect(110,  84, 80, 24, GxEPD_BLACK);
        display.drawRect(110, 113, 80, 24, GxEPD_BLACK);
        display.drawRect(110, 142, 80, 24, GxEPD_BLACK);
        display.drawRect(110, 171, 80, 19, GxEPD_BLACK); 
        display.fillRect(110, 171, 80, 19, GxEPD_RED);
      }

 else if (calibrateRight >300)
       {
        //Serial.println ("RIGHT EMPTY_2");
        //display.drawRect(110,  55, 80,24,  GxEPD_BLACK);      //TODO FILL RECT WITH RED
        //display.drawRect(110,  84, 80, 24, GxEPD_BLACK);
        //display.drawRect(110, 113, 80, 24, GxEPD_BLACK);
        //display.drawRect(110, 142, 80, 24, GxEPD_BLACK);
        //display.drawRect(110, 171, 80, 19, GxEPD_BLACK);
        
        display.fillRect(105, 50, 90, 145, GxEPD_RED);


        /*
        display.fillRect(110, 55,  80, 24, GxEPD_RED);
        display.fillRect(110, 84,  80, 24, GxEPD_RED);
        display.fillRect(110, 113, 80, 24, GxEPD_RED);
        display.fillRect(110, 142, 80, 24, GxEPD_RED);
        display.fillRect(110, 171, 80, 19, GxEPD_RED);
        */
       }  
  
        delay (500); //REQUIRED OR FAILS TO GET VL53 MEASUREMENT
        display.update();
  
  
 }

//DEEP SLEEP WAKEUP REASON
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case 1  : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case 2  : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case 3  : Serial.println("Wakeup caused by timer"); break;
    case 4  : Serial.println("Wakeup caused by touchpad"); break;
    case 5  : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.println("Wakeup was not caused by deep sleep"); break;
  }
}

//AVERAGING
int average_left(int samples, int delayTime){
int averageLeft = 0; 

for(int i=0; i< samples; i++){
  averageLeft = averageLeft + LEFT_BLOCK.readRangeSingleMillimeters();
  delay(delayTime);  
 }
  return (averageLeft/samples);
}

int average_right(int samples, int delayTime){
int averageRight = 0; 

for(int i=0; i< samples; i++){
  averageRight = averageRight + RIGHT_BLOCK.readRangeSingleMillimeters();
  delay(delayTime);  
  }
  return (averageRight/samples);
}
