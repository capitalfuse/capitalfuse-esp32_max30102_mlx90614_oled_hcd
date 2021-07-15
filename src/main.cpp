/****************************************************************************************************************************
  WiFi Manager For ESP32 boards

  <Reference>
  https://github.com/tzapu/WiFiManager
  https://github.com/kentaylor
  
  Modified from Khoi Hoang https://github.com/khoih-prog/ESP_WiFiManager
  Takanobu Fuse, FICUSONLINE, F9E 
  ficus.online@gmail.com
  Licensed under MIT license
 *****************************************************************************************************************************/
/****************************************************************************************************************************
   This example will open a configuration portal when no WiFi configuration has been previously entered or when a button is pushed.

   Also in this example a password is required to connect to the configuration portal
   network.

   The Credentials, being input via Config Portal, will then be saved into LittleFS / SPIFFS file,
   and be used to connect to Adafruit MQTT Server

   Based on original sketch posted by user "wackoo" on https://forum.arduino.cc/index.php?topic=692108
 *****************************************************************************************************************************/
#include <Arduino.h>            // for button
#include <OneButton.h>          // for button

#include <FS.h>

// Now support ArduinoJson 6.0.0+ ( tested with v6.15.2 to v6.16.1 )
#include <ArduinoJson.h>        // get it from https://arduinojson.org/ or install via Arduino library manager

//For ESP32, To use ESP32 Dev Module, QIO, Flash 4MB/80MHz, Upload 921600
//Ported to ESP32

#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>

// From v1.1.0
#include <WiFiMulti.h>
WiFiMulti wifiMulti;

// LittleFS has higher priority than SPIFFS
#if ( ARDUINO_ESP32C3_DEV )
  // Currently, ESP32-C3 only supporting SPIFFS and EEPROM. Will fix to support LittleFS
  #define USE_LITTLEFS          false
  #define USE_SPIFFS            true
#else
  #define USE_LITTLEFS    true
  #define USE_SPIFFS      false
#endif

#if USE_LITTLEFS
  // Use LittleFS
  #include "FS.h"

  // The library has been merged into esp32 core release 1.0.6
  #include <LITTLEFS.h>             // https://github.com/lorol/LITTLEFS
    
  FS* filesystem =      &LITTLEFS;
  #define FileFS        LITTLEFS
  #define FS_Name       "LittleFS"
  
#elif USE_SPIFFS
  #include <SPIFFS.h>
  FS* filesystem =      &SPIFFS;
  #define FileFS        SPIFFS
  #define FS_Name       "SPIFFS"
  
#else
  // Use FFat
  #include <FFat.h>
  FS* filesystem =      &FFat;
  #define FileFS        FFat
  #define FS_Name       "FFat"
#endif

#define ESP_getChipId()   ((uint32_t)ESP.getEfuseMac())

#include <Adafruit_NeoPixel.h>
int pin = 12; // Which pin on the Arduino is connected to the NeoPixels?
int numPixels = 1; // How many NeoPixels are attached to the Arduino?
int pixelFormat = NEO_GRB + NEO_KHZ800; // NeoPixel color format & data rate.
#define lED_LEVEL 50
bool toggleFlag = false;
// Rather than declaring the whole NeoPixel object here, we just create
// a pointer for one, which we'll then allocate later...
Adafruit_NeoPixel *pixels;

//#define LED_ON            HIGH
//#define LED_OFF           LOW

#include "Adafruit_MQTT.h"                //https://github.com/adafruit/Adafruit_MQTT_Library
#include "Adafruit_MQTT_Client.h"         //https://github.com/adafruit/Adafruit_MQTT_Library

//See file .../hardware/espressif/esp32/variants/(esp32|doitESP32devkitV1)/pins_arduino.h
const int BUTTON_PIN  = 15;
//const int BLUE_LED    = 12;
//const int GREEN_LED   = 13;

#define CHECK_INTERVAL 60000L // 60 sec
ulong currentTime;
ulong lastTime = 0;

const char* CONFIG_FILE = "/ConfigMQTT.json";

// Indicates whether ESP has WiFi credentials saved from previous session
bool initialConfig = false; //default false

// include library to read and write from flash memory
#include <EEPROM.h>
// define the number of bytes you want to access
#define EEPROM_SIZE 1

// Json format size
const size_t SENSORDATA_JSON_SIZE0 = JSON_OBJECT_SIZE(6)+150;
const size_t SENSORDATA_JSON_SIZE1 = JSON_OBJECT_SIZE(6);
uint8_t user_id = 1;

// Default configuration values for MQTT
// This actually works
#define MQTT_SERVER              "myhome.gw"
#define MQTT_SERVERPORT          "1883" //1883, or 8883 for SSL
#define MQTT_USERNAME            "myhome"
#define MQTT_KEY                 "myhome" //key or password
#define MQTT_TOPIC               "hass/sensor/hcd/ESP32_xxxxx/state" // MQTT TOPIC

// Labels for custom parameters in WiFi manager
#define MQTT_SERVER_Label             "MQTT_SERVER_Label"
#define MQTT_SERVERPORT_Label         "MQTT_SERVERPORT_Label"
#define MQTT_USERNAME_Label           "MQTT_USERNAME_Label"
#define MQTT_KEY_Label                "MQTT_KEY_Label"
#define MQTT_TOPIC_Label              "MQTT_TOPIC_Label"

// Variables to save custom parameters to...
// I would like to use these instead of #defines
#define custom_MQTT_SERVER_LEN       20
#define custom_MQTT_PORT_LEN          5
#define custom_MQTT_USERNAME_LEN     20
#define custom_MQTT_KEY_LEN          40
#define custom_MQTT_TOPIC_LEN        60

char custom_MQTT_SERVER[custom_MQTT_SERVER_LEN];
char custom_MQTT_SERVERPORT[custom_MQTT_PORT_LEN];
char custom_MQTT_USERNAME[custom_MQTT_USERNAME_LEN];
char custom_MQTT_KEY[custom_MQTT_KEY_LEN];
char custom_MQTT_TOPIC[custom_MQTT_TOPIC_LEN];
char sensor_payload[50] = "payload";

// Function Prototypes
void MQTT_connect();
bool readConfigFile();
bool writeConfigFile();

// For Config Portal
// SSID and PW for Config Portal
String ssid = "ESP32_" + String(ESP_getChipId(), HEX);
String password;

// Just dummy topics. To be updated later when got valid data from FS or Config Portal
String MQTT_Pub_Topic   = "hass/sensor/hcd/" + ssid +"/state";

// SSID and PW for your Router
String Router_SSID;
String Router_Pass;

// You only need to format the filesystem once
//#define FORMAT_FILESYSTEM       true
#define FORMAT_FILESYSTEM         false

#define MIN_AP_PASSWORD_SIZE    6

#define SSID_MAX_LEN            32
//From v1.0.10, WPA2 passwords can be up to 63 characters long.
#define PASS_MAX_LEN            64

typedef struct
{
  char wifi_ssid[SSID_MAX_LEN];
  char wifi_pw[PASS_MAX_LEN];
}  WiFi_Credentials;

typedef struct
{
  String wifi_ssid;
  String wifi_pw;
}  WiFi_Credentials_String;

#define NUM_WIFI_CREDENTIALS      1

typedef struct
{
  WiFi_Credentials  WiFi_Creds [NUM_WIFI_CREDENTIALS];
  uint16_t checksum;
} WM_Config;

WM_Config         WM_config;

#define  CONFIG_FILENAME              F("/wifi_cred.dat")

// Use false if you don't like to display Available Pages in Information Page of Config Portal
// Comment out or use true to display Available Pages in Information Page of Config Portal
// Must be placed before #include <ESP_WiFiManager.h>
#define USE_AVAILABLE_PAGES     true

// From v1.0.10 to permit disable/enable StaticIP configuration in Config Portal from sketch. Valid only if DHCP is used.
// You'll loose the feature of dynamically changing from DHCP to static IP, or vice versa
// You have to explicitly specify false to disable the feature.
#define USE_STATIC_IP_CONFIG_IN_CP          false

// New in v1.0.11
#define USING_CORS_FEATURE          true
//////

// Use USE_DHCP_IP == true for dynamic DHCP IP, false to use static IP which you have to change accordingly to your network
#if (defined(USE_STATIC_IP_CONFIG_IN_CP) && !USE_STATIC_IP_CONFIG_IN_CP)
// Force DHCP to be true
  #if defined(USE_DHCP_IP)
    #undef USE_DHCP_IP
  #endif
  #define USE_DHCP_IP     true
#else
  // You can select DHCP or Static IP here
  #define USE_DHCP_IP     true
  //#define USE_DHCP_IP     false
#endif

#if ( USE_DHCP_IP )
  // Use DHCP
  #warning Using DHCP IP
  IPAddress stationIP   = IPAddress(0, 0, 0, 0);
  IPAddress gatewayIP   = IPAddress(192, 168, 2, 1);
  IPAddress netMask     = IPAddress(255, 255, 255, 0);
#else
  // Use static IP
  #warning Using static IP
  #ifdef ESP32
    IPAddress stationIP   = IPAddress(192, 168, 2, 232);
  #else
    IPAddress stationIP   = IPAddress(192, 168, 2, 186);
  #endif
  
  IPAddress gatewayIP   = IPAddress(192, 168, 2, 1);
  IPAddress netMask     = IPAddress(255, 255, 255, 0);
#endif

#define USE_CONFIGURABLE_DNS      true

IPAddress dns1IP      = gatewayIP;
IPAddress dns2IP      = IPAddress(8, 8, 8, 8);

#define USE_CUSTOM_AP_IP          false

// New in v1.4.0
IPAddress APStaticIP  = IPAddress(192, 168, 100, 1);
IPAddress APStaticGW  = IPAddress(192, 168, 100, 1);
IPAddress APStaticSN  = IPAddress(255, 255, 255, 0);

#include <ESP_WiFiManager.h>              //https://github.com/khoih-prog/ESP_WiFiManager

//Button config
OneButton btn = OneButton(
                  BUTTON_PIN,  // Input pin for the button
                  true,        // Button is active LOW
                  true         // Enable internal pull-up resistor
                );

// Create an ESP32 WiFiClient class to connect to the MQTT server
WiFiClient *client                    = NULL;

Adafruit_MQTT_Client    *mqtt         = NULL;
Adafruit_MQTT_Publish   *pub_sensor_values  = NULL;

///////////////////////////////////////////
// New in v1.4.0
/******************************************
 * // Defined in ESPAsync_WiFiManager.h
typedef struct
{
  IPAddress _ap_static_ip;
  IPAddress _ap_static_gw;
  IPAddress _ap_static_sn;

}  WiFi_AP_IPConfig;

typedef struct
{
  IPAddress _sta_static_ip;
  IPAddress _sta_static_gw;
  IPAddress _sta_static_sn;
#if USE_CONFIGURABLE_DNS  
  IPAddress _sta_static_dns1;
  IPAddress _sta_static_dns2;
#endif
}  WiFi_STA_IPConfig;
******************************************/

WiFi_AP_IPConfig  WM_AP_IPconfig;
WiFi_STA_IPConfig WM_STA_IPconfig;

// For MAX30102, MLX90614, OLED header files
//**************************************************************************
//**************************************************************************

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MLX90614.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h" 

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 100000UL, 100000UL);
bool oledToggleFlag = false;

MAX30105 particleSensor;

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

#define MAX_BRIGHTNESS 255

// For MAX30102, MLX90614, OLED Valiables, FreeRTOS Semaphore
//**************************************************************************
//**************************************************************************

void preCheck(void *pvParameters);
void hrMax30102(void *pvParameters);
void spo2Max30102(void *pvParameters);
void tempMlx90614(void *pvParameters);

// MAX30102 Check Presence variables
bool checkFlag = true; // Flag for finding error
long unblockedValue; //Average IR at power up

// MAX30102 Heart Rate variables
uint32_t chkCount;
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

int32_t beatsPerMinute;
int32_t beatAvg;
long redValue;
long delta;

// MAX30102 Spo2 variables
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

// MAX30102 Config
byte ledBrightness = 0x1F; //Options: 0=Off to 255=50mA
byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32
byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
int pulseWidth = 411; //Options: 69, 118, 215, 411
int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

// MLX90614 variables
float objectTemp;
bool finalFlag = false;

// Final Outputs for Display
typedef struct {
  char const *title1;
  char const *title2;
  char const *value1;
  char const *value2;
  char const *base_unit1;
  char const *base_unit2;
} finalData;

int str_len;
char char_array1[10];
char char_array2[10];

SemaphoreHandle_t  xMutex;

//**************************************************************************
//**************************************************************************
void initAPIPConfigStruct(WiFi_AP_IPConfig &in_WM_AP_IPconfig)
{
  in_WM_AP_IPconfig._ap_static_ip   = APStaticIP;
  in_WM_AP_IPconfig._ap_static_gw   = APStaticGW;
  in_WM_AP_IPconfig._ap_static_sn   = APStaticSN;
}

//**************************************************************************
//**************************************************************************
void initSTAIPConfigStruct(WiFi_STA_IPConfig &in_WM_STA_IPconfig)
{
  in_WM_STA_IPconfig._sta_static_ip   = stationIP;
  in_WM_STA_IPconfig._sta_static_gw   = gatewayIP;
  in_WM_STA_IPconfig._sta_static_sn   = netMask;
#if USE_CONFIGURABLE_DNS  
  in_WM_STA_IPconfig._sta_static_dns1 = dns1IP;
  in_WM_STA_IPconfig._sta_static_dns2 = dns2IP;
#endif
}

//**************************************************************************
//**************************************************************************
void displayIPConfigStruct(WiFi_STA_IPConfig in_WM_STA_IPconfig)
{
  LOGERROR3(F("stationIP ="), in_WM_STA_IPconfig._sta_static_ip, ", gatewayIP =", in_WM_STA_IPconfig._sta_static_gw);
  LOGERROR1(F("netMask ="), in_WM_STA_IPconfig._sta_static_sn);
#if USE_CONFIGURABLE_DNS
  LOGERROR3(F("dns1IP ="), in_WM_STA_IPconfig._sta_static_dns1, ", dns2IP =", in_WM_STA_IPconfig._sta_static_dns2);
#endif
}

//**************************************************************************
//**************************************************************************
void configWiFi(WiFi_STA_IPConfig in_WM_STA_IPconfig)
{
  #if USE_CONFIGURABLE_DNS  
    // Set static IP, Gateway, Subnetmask, DNS1 and DNS2. New in v1.0.5
    WiFi.config(in_WM_STA_IPconfig._sta_static_ip, in_WM_STA_IPconfig._sta_static_gw, in_WM_STA_IPconfig._sta_static_sn, in_WM_STA_IPconfig._sta_static_dns1, in_WM_STA_IPconfig._sta_static_dns2);  
  #else
    // Set static IP, Gateway, Subnetmask, Use auto DNS1 and DNS2.
    WiFi.config(in_WM_STA_IPconfig._sta_static_ip, in_WM_STA_IPconfig._sta_static_gw, in_WM_STA_IPconfig._sta_static_sn);
  #endif 
}

//**************************************************************************
//**************************************************************************
uint8_t connectMultiWiFi()
{
  
#define WIFI_MULTI_1ST_CONNECT_WAITING_MS           800L

#define WIFI_MULTI_CONNECT_WAITING_MS               500L

  uint8_t status;

  WiFi.mode(WIFI_STA);

  LOGERROR(F("ConnectMultiWiFi with :"));

  if ( (Router_SSID != "") && (Router_Pass != "") )
  {
    LOGERROR3(F("* Flash-stored Router_SSID = "), Router_SSID, F(", Router_Pass = "), Router_Pass );
    LOGERROR3(F("* Add SSID = "), Router_SSID, F(", PW = "), Router_Pass );
    wifiMulti.addAP(Router_SSID.c_str(), Router_Pass.c_str());
  }

  for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
  {
    // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
    if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) )
    {
      LOGERROR3(F("* Additional SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
    }
  }

  LOGERROR(F("Connecting MultiWifi..."));

  //WiFi.mode(WIFI_STA);

#if !USE_DHCP_IP
  configWiFi(WM_STA_IPconfig);
#endif

  int i = 0;
  status = wifiMulti.run();
  delay(WIFI_MULTI_1ST_CONNECT_WAITING_MS);

  while ( ( i++ < 20 ) && ( status != WL_CONNECTED ) )
  {
    status = wifiMulti.run();

    if ( status == WL_CONNECTED )
      break;
    else
      delay(WIFI_MULTI_CONNECT_WAITING_MS);
  }

  if ( status == WL_CONNECTED )
  {
    LOGERROR1(F("WiFi connected after time: "), i);
    LOGERROR3(F("SSID:"), WiFi.SSID(), F(",RSSI="), WiFi.RSSI());
    LOGERROR3(F("Channel:"), WiFi.channel(), F(",IP address:"), WiFi.localIP() );
  }
  else
  {
    LOGERROR(F("WiFi not connected"));
    //ESP.restart();
    return status;
  }

  return status;
}

//**************************************************************************
//**************************************************************************
void toggleLED()
{
  //toggle state
  //digitalWrite(BLUE_LED, !digitalRead(BLUE_LED));
  toggleFlag = !toggleFlag;
  pixels->clear(); // Set all pixel colors to 'off'
  if (toggleFlag == true) {
    pixels->setPixelColor(0, pixels->Color(0, 0, lED_LEVEL));
  }
  pixels->show();
}

//**************************************************************************
//**************************************************************************
void check_WiFi()
{
  if ( (WiFi.status() != WL_CONNECTED) )
  {
    Serial.println(F("\nWiFi lost. Call connectMultiWiFi in loop"));
    connectMultiWiFi();
  }
}

//**************************************************************************
//**************************************************************************
void data_publish()
{
  check_WiFi(); // check wifi status and reconnect

  StaticJsonDocument<SENSORDATA_JSON_SIZE1> json1;
  JsonObject userID = json1.createNestedObject("id" + String(user_id));
  userID["hr"] = beatAvg; //Heart Rate Average
  userID["spo2"] = spo2; // SpO2
  userID["temp"] = objectTemp; // Body Temperature
  serializeJson(json1, Serial);
  serializeJson(json1, sensor_payload, sizeof(sensor_payload));
  //pub_sensor_values.publish(sensor_payload);
    
  MQTT_connect();
  
  if (pub_sensor_values->publish(sensor_payload)) 
  {
    //Serial.println(F("Failed to send value to pub_sensor_values feed!"));
    Serial.print(F("Published "));
  }
  else 
  {
    //Serial.println(F("Value to pub_sensor_values feed sucessfully sent!"));
    Serial.print(F("Publish Error "));
  }
}

//**************************************************************************
//**************************************************************************
void check_interval()
{
  lastTime = millis();

  // Check Sleep Mode Interval
  if (lastTime - currentTime >= CHECK_INTERVAL )
  {
    esp_deep_sleep_start();
  }

}

//**************************************************************************
//**************************************************************************
int calcChecksum(uint8_t* address, uint16_t sizeToCalc)
{
  uint16_t checkSum = 0;
  
  for (uint16_t index = 0; index < sizeToCalc; index++)
  {
    checkSum += * ( ( (byte*) address ) + index);
  }

  return checkSum;
}

//**************************************************************************
//**************************************************************************
bool loadConfigData()
{
  File file = FileFS.open(CONFIG_FILENAME, "r");
  LOGERROR(F("LoadWiFiCfgFile "));

  memset((void *) &WM_config,       0, sizeof(WM_config));

  // New in v1.4.0
  memset((void *) &WM_STA_IPconfig, 0, sizeof(WM_STA_IPconfig));
  //////

  if (file)
  {
    file.readBytes((char *) &WM_config,   sizeof(WM_config));

    // New in v1.4.0
    file.readBytes((char *) &WM_STA_IPconfig, sizeof(WM_STA_IPconfig));
    //////

    file.close();
    LOGERROR(F("OK"));

    if ( WM_config.checksum != calcChecksum( (uint8_t*) &WM_config, sizeof(WM_config) - sizeof(WM_config.checksum) ) )
    {
      LOGERROR(F("WM_config checksum wrong"));
      
      return false;
    }
    
    // New in v1.4.0
    displayIPConfigStruct(WM_STA_IPconfig);
    //////

    return true;
  }
  else
  {
    LOGERROR(F("failed"));

    return false;
  }
}

//**************************************************************************
//**************************************************************************
void saveConfigData()
{
  File file = FileFS.open(CONFIG_FILENAME, "w");
  LOGERROR(F("SaveWiFiCfgFile "));

  if (file)
  {
    WM_config.checksum = calcChecksum( (uint8_t*) &WM_config, sizeof(WM_config) - sizeof(WM_config.checksum) );
    
    file.write((uint8_t*) &WM_config, sizeof(WM_config));

    displayIPConfigStruct(WM_STA_IPconfig);

    // New in v1.4.0
    file.write((uint8_t*) &WM_STA_IPconfig, sizeof(WM_STA_IPconfig));
    //////

    file.close();
    LOGERROR(F("OK"));
  }
  else
  {
    LOGERROR(F("failed"));
  }
}

//**************************************************************************
//**************************************************************************
void deleteOldInstances()
{
  // Delete previous instances
  if (mqtt)
  {
    delete mqtt;
    mqtt = NULL;
    
    Serial.println(F("Deleting old MQTT object"));
  }

  if (pub_sensor_values)
  {
    delete pub_sensor_values;
    pub_sensor_values = NULL;
    
    Serial.println(F("Deleting old pub_sensor_values object"));
  }
}

//**************************************************************************
//**************************************************************************
void createNewInstances()
{
  if (!client)
  {
    client = new WiFiClient;
    
    Serial.print(F("\nCreating new WiFi client object : "));
    Serial.println(client? F("OK") : F("failed"));
  }
  
  // Create new instances from new data
  if (!mqtt)
  {
    // Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
    mqtt = new Adafruit_MQTT_Client(client, custom_MQTT_SERVER, atoi(custom_MQTT_SERVERPORT), custom_MQTT_USERNAME, custom_MQTT_KEY);
    
    Serial.print(F("Creating new MQTT object : "));
    
    if (mqtt)
    {
      Serial.println(F("OK"));
      Serial.println(String("MQTT_SERVER = ")    + custom_MQTT_SERVER    + ", MQTT_SERVERPORT = "  + custom_MQTT_SERVERPORT);
      Serial.println(String("MQTT_USERNAME = ")  + custom_MQTT_USERNAME  + ", MQTT_KEY = "         + custom_MQTT_KEY);
      Serial.println(String("MQTT_Pub_Topic = ")  + MQTT_Pub_Topic);
    }
    else
      Serial.println(F("Failed"));
  }
  
  if (!pub_sensor_values)
  {
    Serial.print(F("Creating new MQTT_Pub_Topic,  pub_sensor_values = "));
    Serial.println(MQTT_Pub_Topic);
    
    pub_sensor_values = new Adafruit_MQTT_Publish(mqtt, MQTT_Pub_Topic.c_str());
 
    Serial.print(F("Creating new pub_sensor_values object : "));
    
    if (pub_sensor_values)
    {
      Serial.println(F("OK"));
      Serial.println(String("pub_sensor_values MQTT_Pub_Topic = ")  + MQTT_Pub_Topic);
    }
    else
      Serial.println(F("Failed"));
    }
}

//**************************************************************************
//**************************************************************************
void wifi_manager()
{
  Serial.println(F("\nConfig Portal requested."));
  //digitalWrite(GREEN_LED, LED_ON); // turn the LED on by making the voltage LOW to tell us we are in configuration mode.
  pixels->clear(); // Set all pixel colors to 'off'
  pixels->setPixelColor(0, pixels->Color(0, lED_LEVEL, 0));
  pixels->show();

  //Local intialization. Once its business is done, there is no need to keep it around
  ESP_WiFiManager ESP_wifiManager("ConfigOnSwitchFS-MQTT");

  //Check if there is stored WiFi router/password credentials.
  //If not found, device will remain in configuration mode until switched off via webserver.
  Serial.print(F("Opening Configuration Portal. "));
  
  Router_SSID = ESP_wifiManager.WiFi_SSID();
  Router_Pass = ESP_wifiManager.WiFi_Pass();

  // From v1.1.1, Don't permit NULL password
  if ( !initialConfig && (Router_SSID != "") && (Router_Pass != "") )
  {
    //If valid AP credential and not DRD, set timeout 120s.
    ESP_wifiManager.setConfigPortalTimeout(120);
    Serial.println(F("Got stored Credentials. Timeout 120s"));
  }
  else
  {
    ESP_wifiManager.setConfigPortalTimeout(0);

    Serial.print(F("No timeout : "));
    
    if (initialConfig)
    {
      Serial.println(F("DRD or No stored Credentials.."));
    }
    else
    {
      Serial.println(F("No stored Credentials."));
    }
  }
  
  //Local intialization. Once its business is done, there is no need to keep it around

  // Extra parameters to be configured
  // After connecting, parameter.getValue() will get you the configured value
  // Format: <ID> <Placeholder text> <default value> <length> <custom HTML> <label placement>
  // (*** we are not using <custom HTML> and <label placement> ***)

  // MQTT_SERVER
  ESP_WMParameter MQTT_SERVER_FIELD(MQTT_SERVER_Label, "MQTT SERVER", custom_MQTT_SERVER, custom_MQTT_SERVER_LEN /*20*/);

  // MQTT_SERVERPORT
  ESP_WMParameter MQTT_SERVERPORT_FIELD(MQTT_SERVERPORT_Label, "MQTT SERVER PORT", custom_MQTT_SERVERPORT, custom_MQTT_PORT_LEN + 1);

  // MQTT_USERNAME
  ESP_WMParameter MQTT_USERNAME_FIELD(MQTT_USERNAME_Label, "MQTT USERNAME", custom_MQTT_USERNAME, custom_MQTT_USERNAME_LEN /*20*/);

  // MQTT_KEY
  ESP_WMParameter MQTT_KEY_FIELD(MQTT_KEY_Label, "MQTT KEY", custom_MQTT_KEY, custom_MQTT_KEY_LEN /*40*/);

  // MQTT_TOPIC
  ESP_WMParameter MQTT_TOPIC_FIELD(MQTT_TOPIC_Label, "MQTT TOPIC", custom_MQTT_TOPIC, custom_MQTT_TOPIC_LEN /*60*/);

  ESP_wifiManager.addParameter(&MQTT_SERVER_FIELD);
  ESP_wifiManager.addParameter(&MQTT_SERVERPORT_FIELD);
  ESP_wifiManager.addParameter(&MQTT_USERNAME_FIELD);
  ESP_wifiManager.addParameter(&MQTT_KEY_FIELD);
  ESP_wifiManager.addParameter(&MQTT_TOPIC_FIELD);

  // Sets timeout in seconds until configuration portal gets turned off.
  // If not specified device will remain in configuration mode until
  // switched off via webserver or device is restarted.
  //ESP_wifiManager.setConfigPortalTimeout(120);

  ESP_wifiManager.setMinimumSignalQuality(-1);

  // From v1.0.10 only
  // Set config portal channel, default = 1. Use 0 => random channel from 1-13
  ESP_wifiManager.setConfigPortalChannel(0);
  //////
  
#if USE_CUSTOM_AP_IP
  //set custom ip for portal
  // New in v1.4.0
  ESP_wifiManager.setAPStaticIPConfig(WM_AP_IPconfig);
  //////
#endif
  
#if !USE_DHCP_IP    
  #if USE_CONFIGURABLE_DNS
    // Set static IP, Gateway, Subnetmask, DNS1 and DNS2. New in v1.0.5
    ESP_wifiManager.setSTAStaticIPConfig(stationIP, gatewayIP, netMask, dns1IP, dns2IP);
  #else
    // Set static IP, Gateway, Subnetmask, Use auto DNS1 and DNS2.
    ESP_wifiManager.setSTAStaticIPConfig(stationIP, gatewayIP, netMask);
  #endif 
#endif  

  // New from v1.1.1
#if USING_CORS_FEATURE
  ESP_wifiManager.setCORSHeader("Your Access-Control-Allow-Origin");
#endif

  // SSID to uppercase
  ssid.toUpperCase();
  password = "My" + ssid;
  
  // Start an access point
  // and goes into a blocking loop awaiting configuration.
  // Once the user leaves the portal with the exit button
  // processing will continue
  if (!ESP_wifiManager.startConfigPortal((const char *) ssid.c_str(), password.c_str()))
  {
    Serial.println(F("Not connected to WiFi but continuing anyway."));
  }
  else
  {
    // If you get here you have connected to the WiFi
    Serial.println(F("Connected...yeey :)"));
    Serial.print(F("Local IP: "));
    Serial.println(WiFi.localIP());
  }

  // Only clear then save data if CP entered and with new valid Credentials
  // No CP => stored getSSID() = ""
  if ( String(ESP_wifiManager.getSSID(0)) != "" && String(ESP_wifiManager.getSSID(1)) != "" )
  {
    // Stored  for later usage, from v1.1.0, but clear first
    memset(&WM_config, 0, sizeof(WM_config));
    
    for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
    {
      String tempSSID = ESP_wifiManager.getSSID(i);
      String tempPW   = ESP_wifiManager.getPW(i);
  
      if (strlen(tempSSID.c_str()) < sizeof(WM_config.WiFi_Creds[i].wifi_ssid) - 1)
        strcpy(WM_config.WiFi_Creds[i].wifi_ssid, tempSSID.c_str());
      else
        strncpy(WM_config.WiFi_Creds[i].wifi_ssid, tempSSID.c_str(), sizeof(WM_config.WiFi_Creds[i].wifi_ssid) - 1);
  
      if (strlen(tempPW.c_str()) < sizeof(WM_config.WiFi_Creds[i].wifi_pw) - 1)
        strcpy(WM_config.WiFi_Creds[i].wifi_pw, tempPW.c_str());
      else
        strncpy(WM_config.WiFi_Creds[i].wifi_pw, tempPW.c_str(), sizeof(WM_config.WiFi_Creds[i].wifi_pw) - 1);  
  
      // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
      if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) )
      {
        LOGERROR3(F("* Add SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
        wifiMulti.addAP(WM_config.WiFi_Creds[i].wifi_ssid, WM_config.WiFi_Creds[i].wifi_pw);
      }
    }

    // New in v1.4.0
    ESP_wifiManager.getSTAStaticIPConfig(WM_STA_IPconfig);
    //////
    
    saveConfigData();
  }

  // Getting posted form values and overriding local variables parameters
  // Config file is written regardless the connection state
  strcpy(custom_MQTT_SERVER, MQTT_SERVER_FIELD.getValue());
  strcpy(custom_MQTT_SERVERPORT, MQTT_SERVERPORT_FIELD.getValue());
  strcpy(custom_MQTT_USERNAME, MQTT_USERNAME_FIELD.getValue());
  strcpy(custom_MQTT_KEY, MQTT_KEY_FIELD.getValue());
  strcpy(custom_MQTT_TOPIC, MQTT_TOPIC_FIELD.getValue());

  // Writing JSON config file to flash for next boot
  writeConfigFile();

  //digitalWrite(GREEN_LED, LED_OFF); // Turn LED off as we are not in configuration mode.
  pixels->clear(); // Set all pixel colors to 'off'
  pixels->show();

  deleteOldInstances();

  MQTT_Pub_Topic = String(custom_MQTT_TOPIC);
  //createNewInstances();
  WiFi.mode(WIFI_OFF);
  currentTime = millis(); // Reset start time for sleep mode
}

// this function is just to display newly saved data,
// it is not necessary though, because data is displayed
// after WiFi manager resets ESP32
//**************************************************************************
//**************************************************************************
void newConfigData() 
{
  Serial.println();
  Serial.print(F("custom_MQTT_SERVER: ")); 
  Serial.println(custom_MQTT_SERVER);
  Serial.print(F("custom_SERVERPORT: ")); 
  Serial.println(custom_MQTT_SERVERPORT);
  Serial.print(F("custom_USERNAME: ")); 
  Serial.println(custom_MQTT_USERNAME);
  Serial.print(F("custom_KEY: ")); 
  Serial.println(custom_MQTT_KEY);
  Serial.print(F("custom_TOPIC: ")); 
  Serial.println(custom_MQTT_TOPIC);
  Serial.println();
}

//**************************************************************************
//**************************************************************************
bool readConfigFile() 
{
  // this opens the config file in read-mode
  File f = FileFS.open(CONFIG_FILE, "r");

  if (!f)
  {
    Serial.println(F("Config File not found"));
    return false;
  }
  else
  {
    // we could open the file
    size_t size = f.size();
    // Allocate a buffer to store contents of the file.
    std::unique_ptr<char[]> buf(new char[size + 1]);

    // Read and store file contents in buf
    f.readBytes(buf.get(), size);
    // Closing file
    f.close();
    // Using dynamic JSON buffer which is not the recommended memory model, but anyway
    // See https://github.com/bblanchon/ArduinoJson/wiki/Memory%20model

#if (ARDUINOJSON_VERSION_MAJOR >= 6)

    DynamicJsonDocument json0(SENSORDATA_JSON_SIZE0);
    auto deserializeError = deserializeJson(json0, buf.get());
    
    if ( deserializeError )
    {
      Serial.println(F("JSON parseObject() failed"));
      return false;
    }
    
    serializeJson(json0, Serial);
    
#else

    DynamicJsonBuffer jsonBuffer;
    // Parse JSON string
    JsonObject& json0 = jsonBuffer.parseObject(buf.get());
    
    // Test if parsing succeeds.
    if (!json0.success())
    {
      Serial.println(F("JSON parseObject() failed"));
      return false;
    }
    
    json0.printTo(Serial);
    
#endif

    // Parse all config file parameters, override
    // local config variables with parsed values
    if (json0.containsKey(MQTT_SERVER_Label))
    {
      strcpy(custom_MQTT_SERVER, json0[MQTT_SERVER_Label]);
    }

    if (json0.containsKey(MQTT_SERVERPORT_Label))
    {
      strcpy(custom_MQTT_SERVERPORT, json0[MQTT_SERVERPORT_Label]);
    }

    if (json0.containsKey(MQTT_USERNAME_Label))
    {
      strcpy(custom_MQTT_USERNAME, json0[MQTT_USERNAME_Label]);
    }

    if (json0.containsKey(MQTT_KEY_Label))
    {
      strcpy(custom_MQTT_KEY, json0[MQTT_KEY_Label]);
    }

    if (json0.containsKey(MQTT_TOPIC_Label))
    {
      strcpy(custom_MQTT_TOPIC, json0[MQTT_TOPIC_Label]);
    }
  }
  
  Serial.println(F("\nConfig File successfully parsed"));
  
  return true;
}

//**************************************************************************
//**************************************************************************
bool writeConfigFile() 
{
  Serial.println(F("Saving Config File"));

#if (ARDUINOJSON_VERSION_MAJOR >= 6)
  DynamicJsonDocument json0(SENSORDATA_JSON_SIZE0);
#else
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json0 = jsonBuffer.createObject();
#endif

  // JSONify local configuration parameters
  json0[MQTT_SERVER_Label]      = custom_MQTT_SERVER;
  json0[MQTT_SERVERPORT_Label]  = custom_MQTT_SERVERPORT;
  json0[MQTT_USERNAME_Label]    = custom_MQTT_USERNAME;
  json0[MQTT_KEY_Label]         = custom_MQTT_KEY;
  json0[MQTT_TOPIC_Label]       = custom_MQTT_TOPIC;

  // Open file for writing
  File f = FileFS.open(CONFIG_FILE, "w");

  if (!f)
  {
    Serial.println(F("Failed to open Config File for writing"));
    return false;
  }

#if (ARDUINOJSON_VERSION_MAJOR >= 6)
  serializeJsonPretty(json0, Serial);
  // Write data to file and close it
  serializeJson(json0, f);
#else
  json0.prettyPrintTo(Serial);
  // Write data to file and close it
  json0.printTo(f);
#endif

  f.close();

  Serial.println(F("\nConfig File successfully saved"));
  return true;
}

//**************************************************************************
//**************************************************************************
void MQTT_connect() 
{
  int8_t ret;

  MQTT_Pub_Topic = String(custom_MQTT_TOPIC);

  createNewInstances();

  // Return if already connected
  if (mqtt->connected()) 
  {
    return;
  }

  Serial.println(F("Connecting to WiFi MQTT (3 attempts)..."));

  uint8_t attempt = 3;
  
  while ((ret = mqtt->connect()) != 0) 
  { 
    // connect will return 0 for connected
    Serial.println(mqtt->connectErrorString(ret));
    Serial.println(F("Another attemtpt to connect to MQTT in 2 seconds..."));
    mqtt->disconnect();
    delay(2000);  // wait 2 seconds
    attempt--;
    
    if (attempt == 0) 
    {
      Serial.println(F("WiFi MQTT connection failed. Continuing with program..."));
      return;
    }
  }
  
  Serial.println(F("WiFi MQTT connection successful!"));
}

// ************************************************************************************************************
// Define reference handlers to these two tasks. Use these TaskHandle_t type variables to change task priority.
TaskHandle_t TaskHandle_0;
TaskHandle_t TaskHandle_1; // handler for Task1
TaskHandle_t TaskHandle_2; // handler for Task2
TaskHandle_t TaskHandle_3; // handler for Task3

//event handler functions for button
//**************************************************************************
//**************************************************************************
static void handleClick() 
{
  Serial.println(F("Button clicked!"));
  currentTime = millis(); // Reset start time for sleep mode
  user_id = user_id + 1;
  if (user_id > 10) {
    user_id = 1;
  }
  EEPROM.write(0, user_id);
  EEPROM.commit();
}

//**************************************************************************
//**************************************************************************
static void handleDoubleClick() 
{
  currentTime = millis(); // Reset start time for sleep mode
  Serial.println(F("Button double clicked!"));
  //newConfigData();
  // Reset User ID
  user_id = 1;
  EEPROM.write(0, user_id);
  EEPROM.commit();
}

//**************************************************************************
//**************************************************************************
static void handleLongPressStop() 
{
  Serial.println(F("Button pressed for long time and then released!"));
  vTaskDelay(10);
  vTaskSuspend(TaskHandle_0);
  wifi_manager();
  vTaskResume(TaskHandle_0);
}

// *********************************** Setup function ********************************************************************************
// ***********************************************************************************************************************************
void setup()
{
  // Put your setup code here, to run once
  Serial.begin(115200);
  while (!Serial);

  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);
  // read the last LED state from flash memory
  if (EEPROM.read(0) > 10) {
    EEPROM.write(0, 1); // set default user_id:1
    EEPROM.commit();
  }
  user_id = EEPROM.read(0);
  
  // Wake up from Deep Sleep Mode : ext0 uses RTC_IO to wakeup
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_15,0); //1 = High, 0 = Low, GPIO15

  delay(200);

  Serial.print(F("\nStarting ConfigOnSwichFS_MQTT_Ptr using ")); Serial.print(FS_Name);
  Serial.print(F(" on ")); Serial.println(ARDUINO_BOARD);
  Serial.println(ESP_WIFIMANAGER_VERSION);

  btn.attachClick(handleClick);
  btn.attachDoubleClick(handleDoubleClick);
  btn.attachLongPressStop(handleLongPressStop);

  // Initialize the LED digital pin as an output.
  // pinMode(BLUE_LED, OUTPUT);
  // pinMode(GREEN_LED, OUTPUT);

  // create a new NeoPixel object dynamically with these values:
  pixels = new Adafruit_NeoPixel(numPixels, pin, pixelFormat);
  pixels->begin(); // INITIALIZE NeoPixel strip object (REQUIRED)

  // Mount the filesystem
  if (FORMAT_FILESYSTEM)
  {
    Serial.println(F("Forced Formatting."));
    FileFS.format();
  }

  // Format FileFS if not yet
  if (!FileFS.begin(true))
  {
    Serial.println(F("SPIFFS/LittleFS failed! Already tried formatting."));
  
    if (!FileFS.begin())
    {     
      // prevents debug info from the library to hide err message.
      delay(100);
      
#if USE_LITTLEFS
      Serial.println(F("LittleFS failed!. Please use SPIFFS or EEPROM. Stay forever"));
#else
      Serial.println(F("SPIFFS failed!. Please use LittleFS or EEPROM. Stay forever"));
#endif

      while (true)
      {
        delay(1);
      }
    }
  }

  if (!readConfigFile())
  {
    Serial.println(F("Failed to read configuration file, using default values"));
  }

  // New in v1.4.0
  initAPIPConfigStruct(WM_AP_IPconfig);
  initSTAIPConfigStruct(WM_STA_IPconfig);
  //////

  if (!readConfigFile())
  {
    Serial.println(F("Can't read Config File, using default values"));
  }

  // Load stored data, the addAP ready for MultiWiFi reconnection
  bool configDataLoaded = loadConfigData();

  // Pretend CP is necessary as we have no AP Credentials
  initialConfig = true;

  if (configDataLoaded)
  {
    for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
    {
      // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
      if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) )
      {
        LOGERROR3(F("* Add SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
        wifiMulti.addAP(WM_config.WiFi_Creds[i].wifi_ssid, WM_config.WiFi_Creds[i].wifi_pw);
        initialConfig = false;
      }
    }
  }

  if (initialConfig)
  {
    Serial.println(F("Open Config Portal without Timeout: No stored WiFi Credentials"));
  
    wifi_manager();
  }
/*   else if ( WiFi.status() != WL_CONNECTED ) 
  {
    Serial.println(F("ConnectMultiWiFi in setup"));
   
    connectMultiWiFi();
  } */

  //digitalWrite(BLUE_LED, LED_OFF); // Turn led off as we are not in configuration mode.
  pixels->clear();
  pixels->show(); 

// ************************MAX30102, MLX90614, OLED********************************
// ********************************************************************************  
  
  // create mutex and assign it a already create handler 
  xMutex = xSemaphoreCreateMutex();
  
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  delay(1000);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setRotation(2);

  // MAX30102
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  // MLX90614
  mlx.begin();

  // set up 4 tasks to run independently.
  xTaskCreatePinnedToCore(
    preCheck
    ,  "preCheck"   // A name just for humans
    ,  4000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &TaskHandle_0
    ,  0);
    
  xTaskCreatePinnedToCore(
    hrMax30102
    ,  "hrMax30102"   // A name just for humans
    ,  3000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &TaskHandle_1
    ,  0); 
  
  xTaskCreatePinnedToCore(
    spo2Max30102
    ,  "spo2Max30102"   // A name just for humans
    ,  4000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &TaskHandle_2
    ,  0); 

  xTaskCreatePinnedToCore(
    tempMlx90614
    ,  "tempMlx90614"   // A name just for humans
    ,  4000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  0  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &TaskHandle_3
    ,  0);

}

// ********************************************* Loop function ******************************************************************
// ******************************************************************************************************************************
void loop()
{ 
  btn.tick(); //loop function runs on cpu core 1 by default 
  if ( checkFlag == false ){
    // loop() is also the one of FreeRTOS Tasks, so delete own after running task1.
    vTaskDelete(NULL); 
  }
}

//************************OLED Display Functions*********************************
//*****************************************************************************

void displayNotice() {
  oledToggleFlag = !oledToggleFlag;
  if (oledToggleFlag == false){
    display.clearDisplay();
    delay(2000);
  } else {
    display.setTextSize(2);
    display.setCursor(8,0);
    display.print(F("ID :"));
    display.setCursor(64,0);
    display.print(user_id);
    display.setTextSize(1);
    display.setCursor(8,32);
    display.print(F("Put Your Finger ON"));
    display.setTextSize(2);
    display.setCursor(8,48);
    display.print(F("START ^^^"));
    delay(500);
  }
  //yield();
  display.display();
}

void displayFinal(finalData *p) {
  display.clearDisplay();
  delay(10);
  //yield();
  display.display();

  display.setTextSize(1);
  display.setCursor(0,4);
  display.print(p->title2);
  
  display.setTextSize(2);
  display.setCursor(8,16); 
  display.print(p->value2);
  display.print(" ");
  display.print(p->base_unit2);
  
  display.setTextSize(1);
  display.setCursor(0,36);
  display.print(p->title1);

  display.setTextSize(2);
  display.setCursor(8,48); 
  display.print(p->value1);
  display.print(" ");
  display.print(p->base_unit1);
  
  delay(10);
  //yield();
  display.display();
  delay(3000);

  if (finalFlag == false) {
    xSemaphoreGive(xMutex); // release mutex
  }

}

void displayError() {
  display.clearDisplay();
  delay(10);
  //yield();
  display.display();

  display.setTextSize(1);
  display.setCursor(20,24);
  display.print(F("Read Error..."));
  display.setCursor(20,40);
  display.print(F("Try Again!"));
  
  delay(10);
  //yield();
  display.display();
  delay(2000);
  
  xSemaphoreGive(xMutex); // release mutex
  //tempMlx90614(NULL);
}

void displayChecking() {
  display.clearDisplay();
  delay(10);
  //yield();
  display.display();

  display.setTextSize(1);
  display.setCursor(8,16);
  display.print(F("Keep Your Finger"));
  display.setCursor(8,30);
  display.print(F("putting on"));
  display.setTextSize(2);
  display.setCursor(8,48);
  display.print(F("Checking.."));

  delay(10);
  //yield();
  display.display();
}

//****************************************************************TASKS***************************************************************
//************************************************************************************************************************************

// Output data will be transformed to char from string
char const *strToChar1(String str) {
  str_len=str.length()+1;
  str.toCharArray(char_array1, str_len);
  return char_array1;
}
char const *strToChar2(String str) {
  str_len=str.length()+1;
  str.toCharArray(char_array2, str_len);
  return char_array2;
}

// ********************************Each task runs on CPU Core 0****************************************************
// MAX30102:Checking Presence
//*******************************************************************************************************
void preCheck(void *pvParameters)
{
  (void) pvParameters;

  xSemaphoreTake(xMutex, portMAX_DELAY);

  currentTime = millis();
  
  for (;;) 
  {
    // this is just for checking go into sleep mode
    check_interval();

    displayNotice();

    particleSensor.setPulseAmplitudeRed(0x00); //Red Led off
  
    //Take an average of IR readings at power up
    unblockedValue = 0;
    
    for (byte x = 0 ; x < 32 ; x++)
    {
      unblockedValue += particleSensor.getIR(); //Read the IR value
    }
    unblockedValue /= 32;
  
  
    Serial.print("IR ");
    Serial.print(particleSensor.getIR());
  
    long currentDelta = particleSensor.getIR() - unblockedValue;
  
    Serial.print(", delta ");
    Serial.println(currentDelta);
  
    if (currentDelta > (long)500)
    {
      Serial.println(" Something is there!");
      particleSensor.setPulseAmplitudeRed(ledBrightness);

      xSemaphoreGive(xMutex); // release mutex
      vTaskDelay(pdMS_TO_TICKS(100));
      vTaskDelete(NULL);
    }
    //displayNotice();
  }
}

// MAX30102:Mesuring Heart Beat
//************************************************************************************************************
void hrMax30102(void *pvParameters)
{
  (void) pvParameters;

  xSemaphoreTake(xMutex, portMAX_DELAY);

  checkFlag = false;
  //digitalWrite(BLUE_LED, 1);
  pixels->clear(); // Set all pixel colors to 'off'
  pixels->setPixelColor(0, pixels->Color(0, 0, lED_LEVEL));
  pixels->show();
  
  displayChecking();
  
  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeIR(0); //Turn off IR LED
  
  for(int i=0; i<1500; i++) {
    
    //digitalWrite(BLUE_LED, !digitalRead(BLUE_LED)); //Blink onboard LED with every data read
    toggleFlag = !toggleFlag;
    pixels->clear(); // Set all pixel colors to 'off'
    if (toggleFlag == true) {
      pixels->setPixelColor(0, pixels->Color(0, 0, lED_LEVEL));
    }
    pixels->show();

    redValue = particleSensor.getRed();
  
    if (checkForBeat(redValue) == true)
    {
      //We sensed a beat!
      delta = millis() - lastBeat;
      lastBeat = millis();
  
      beatsPerMinute = 60 / (delta / 1000.0);
  
      if (beatsPerMinute < 255 && beatsPerMinute > 20)
      {
        rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
        rateSpot %= RATE_SIZE; //Wrap variable
  
        //Take average of readings
        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }
  
    Serial.print("Red=");
    Serial.print(redValue);
    Serial.print(", BPM=");
    Serial.print(beatsPerMinute);
    Serial.print(", Avg BPM=");
    Serial.print(beatAvg);

    if (redValue < 10000) {
      chkCount++;
      Serial.print(" No finger?");
      if (chkCount > 100) {
        break;
      }
    }  
  
    Serial.println();
    if(i > 500) {
      if((beatAvg >30) && (beatsPerMinute > 0.95*beatAvg && beatsPerMinute < 1.05*beatAvg)) {
        checkFlag=true;
        break;
      }
    }
  }

  //digitalWrite(BLUE_LED, 0);
  pixels->clear();
  pixels->show();
  
  if(checkFlag==true) {
    finalData *p = (finalData *)malloc(sizeof(finalData));
    
    p->title1 = "Average BPM";
    p->title2 = "BPM";

    p->value1 = strToChar1(String(beatAvg));
    p->value2 = strToChar2(String(beatsPerMinute));
       
    p->base_unit1 = "bpm";
    p->base_unit2 = "bpm";
    
    displayFinal(p);
    
  }else {
    vTaskSuspend(TaskHandle_2); //Suspend spo2Max30102 Task
    beatAvg = 0;
    spo2 = 0;
    displayError();
  }
  
  vTaskDelay(pdMS_TO_TICKS(100));
  vTaskDelete(NULL);
}

// MAX30102:Mesuring SpO2 Pulse Oximeter
//*******************************************************************************************************
void spo2Max30102(void *pvParameters)
{
  (void) pvParameters;

  xSemaphoreTake(xMutex, portMAX_DELAY);

  checkFlag=false;
  //digitalWrite(GREEN_LED, 1);
  pixels->clear(); // Set all pixel colors to 'off'
  pixels->setPixelColor(0, pixels->Color(0, lED_LEVEL, 0));
  pixels->show();

  displayChecking();

  // max30102
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  for (int x=0; x< 20; x++)
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      //digitalWrite(GREEN_LED, !digitalRead(GREEN_LED)); //Blink onboard LED with every data read
      toggleFlag = !toggleFlag;
      pixels->clear(); // Set all pixel colors to 'off'
      if (toggleFlag == true) {
        pixels->setPixelColor(0, pixels->Color(0, lED_LEVEL, 0));
      }
      pixels->show();

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART

      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);

      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);

      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);

      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);

      if (validHeartRate==0 || validSPO2==0) {
        chkCount++;
      }
    }
    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    if((heartRate > (beatAvg - 10) && heartRate < (beatAvg + 10)) && (validHeartRate==1 && validSPO2==1)) {
      checkFlag=true;
      break;
    }
    
    if (chkCount > 100) {
      Serial.print(" No finger?");
      break;
    }

  }

  //digitalWrite(GREEN_LED, 0);
  pixels->clear();
  pixels->show();
  
  if(checkFlag==true) {
    finalData *p = (finalData *)malloc(sizeof(finalData));
    
    p->title1 = "SpO2";
    p->title2 = "Heart Rate(BPM)";

    p->value1 = strToChar1(String(spo2));
    p->value2 = strToChar2(String(heartRate));
    
    p->base_unit1 = "%";
    p->base_unit2 = "bpm";
    
    displayFinal(p);
    
  } else {
    spo2 = 0;
    displayError();
  }
  
  vTaskDelay(pdMS_TO_TICKS(100));
  vTaskDelete(NULL);
}

// MLX90614:Measuring Object Temp and Ambient Temp
//**********************************************************************************************************************
void tempMlx90614(void *pvParameters)
{
  (void) pvParameters;
  
  xSemaphoreTake(xMutex, portMAX_DELAY);
  
  finalData *p = (finalData *)malloc(sizeof(finalData));
  
  p->title1 = "TEMPERATURE:";
  p->title2 = "AMBIENT:";
  objectTemp = round(mlx.readObjectTempC() * 100) / 100.00f; 
  p->value1 = strToChar1(String(objectTemp));
  p->value2 = strToChar2(String(mlx.readAmbientTempC()));

  p->base_unit1 = "`C";
  p->base_unit2 = "`C";
  
  finalFlag = true;
  displayFinal(p);

  data_publish();
  vTaskDelay(pdMS_TO_TICKS(200));

  deleteOldInstances();
  WiFi.mode(WIFI_OFF);

  ESP.restart();
}