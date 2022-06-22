// Spartfun Libraries
#include "SparkFun_VL53L1X.h"

// WebServer
#include <ESPAsyncTCP.h>
//#include <FS.h>
//#include <Wire.h>
//#include <vector>
//#include "config.h"

// static std::vector<AsyncClient *> clients; // a list to hold all clients

#include <WiFiManager.h>
WiFiManager wifiManager;

#define WEBSERVER_H
#include "ESPAsyncWebServer.h"

//#include <ESP8266WiFi.h>

#define DNS_PORT 53
AsyncWebServer server(80);
DNSServer DNS;

float threshold_percentage = 80;

// if "true", the raw measurements are sent via MQTT during runtime (for debugging) - I'd recommend setting it to "false" to save traffic and system resources.
// in the calibration phase the raw measurements will still be sent through MQTT
static bool update_raw_measurements = false;

// this value has to be true if the sensor is oriented as in Duthdeffy's picture
static bool advised_orientation_of_the_sensor = true;

char peopleCounterArray[50];
SFEVL53L1X distanceSensor(Wire);

static int NOBODY = 0;
static int SOMEONE = 1;
static int LEFT = 0;
static int RIGHT = 1;

static int DIST_THRESHOLD_MAX[] = {0, 0}; // treshold of the two zones

static int PathTrack[] = {0, 0, 0, 0};
static int PathTrackFillingSize = 1; // init this to 1 as we start from state where nobody is any of the zones
static int LeftPreviousStatus = NOBODY;
static int RightPreviousStatus = NOBODY;

static int center[2] = {0, 0}; /* center of the two zones */
static int Zone = 0;
static int PplCounter = 0;

static int ROI_height = 0;
static int ROI_width = 0;

int cnt;
int limit = 5;
int newMinDistance;

float sum_zone_0;
float sum_zone_1;

float number_attempts;

float average_zone_0 = sum_zone_0 / number_attempts;
float average_zone_1 = sum_zone_1 / number_attempts;

// declaring PIN for button purpose
int inPin = 0;
int val = 0;
int timeout = 120; // seconds to run for AP portal

// Flag for communication between 2 devices
int Flag;
// unsigned long lastChange;
int FlagExternal;
// unsigned long lastChangeExternal;
int FlagLastChangeInLoop;
// unsigned long lastChangeInLoop;

// Async Delay
#include <AsyncDelay.h>
AsyncDelay samplingInterval;

//
bool IsAdded;
uint16_t distance;

// EEPROM
#include <EEPROM.h>
int addr = 0;
byte value;
//
uint8_t adres[2] = {0, 2};
//
#define EEPROM_SIZE 8

// String IPAdressOfExternalDevice;

bool IsConnected;
bool IsResetDevice;
bool IsEEPROMWrite;

//#include <ESP8266HTTPClient.h>

// ESPNOW
#include <espnow.h>

typedef struct struct_message
{
  int cnt_espNow;
} struct_message;

struct_message incomingReadings;

// Insert your SSID
// constexpr char WIFI_SSID[] = "ESP-7D82999";

// MAC Address of the receiver
// uint8_t broadcastAddress[] = {0x5c, 0xcf, 0x7f, 0x6d, 0x1f, 0xe7};
// uint8_t broadcastAddress[] = {0x68, 0xC6, 0x3A, 0xA5, 0xB5, 0xB3};

// new mac after 3stawy
// ip:135
// uint8_t broadcastAddress[] = {0x68, 0xC6, 0x3A, 0xA5, 0x98, 0x15};
// ip:136
uint8_t broadcastAddress[] = {0x5C, 0xCF, 0x7F, 0x6D, 0x1F, 0xd7};

// mac address for opener_main: 5c:cf:7f:6d:1f:e2
// mac address for opener_support: 5c:cf:7f:6d:1f:d7

// Create a struct_message called myData
struct_message myData;

// set 5 trials for send message to external device
int j = 15;
int i = 0;

#include "arduino_secrets.h"
#include "thingProperties.h"

bool flagToNotReadyCntChangedOnStart;

void setup()
{

  Wire.begin();
  Serial.begin(115200);
  // EEPROM
  EEPROM.begin(4096);
  // Read EEPROM
  // if (EEPROM.read(0) == 1)
  //{
  byte data = EEPROM.read(1);
  cnt = (int8_t)data;
  byte dataNewMinDistance = EEPROM.read(2);
  newMinDistance = (int8_t)dataNewMinDistance;
  // IPAdressOfExternalDevice = readStringFromEEPROM(3);
  // }
  delay(100);

  // Timer set to 4 hours - to restart device and calculate evry 6 hours
  samplingInterval.start(14400000, AsyncDelay::MILLIS);

  // pinmode for button purpose
  pinMode(inPin, INPUT);

  wifiManager.autoConnect("Counting_Wizard");
  delay(100);

  // AsyncServer *serverAA = new AsyncServer(TCP_PORT); // start listening on tcp port 7050
  // serverAA->onClient(&handleNewClient, serverAA);
  // serverAA->begin();

  // Serial.println("VL53L1X Qwiic Test");
  if (distanceSensor.init() == false)
    // Serial.println("Sensor online!");
    distanceSensor.setIntermeasurementPeriod(100);
  distanceSensor.setDistanceModeLong();

  delay(1000);
  zones_calibration();

  // Serial.println("Thresold0:");
  // Serial.println(DIST_THRESHOLD_MAX[0]);
  // Serial.println("Thresold1:");
  // Serial.println(DIST_THRESHOLD_MAX[1]);
  // Serial.println();

  // if (!SPIFFS.begin())
  //{
  // Serial.println("An Error has occurred while mounting SPIFFS");
  //  return;
  //}

  server.on("/getADC", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", String(handleADC())); });

  server.on("/add", HTTP_POST, [](AsyncWebServerRequest *request)
            {
              //Serial.print("Adding one person");
              cnt++;
              request->send(200); });

  server.on("/subtract", HTTP_POST, [](AsyncWebServerRequest *request)
            {
              //Serial.print("Subtract one person");
              cnt--;
              request->send(200); });

  server.on("/set", HTTP_POST, [](AsyncWebServerRequest *request)
            {
              String arg = request->arg("number");
              //Serial.print("New people value is: ");
              //Serial.println(arg.toInt());
              cnt = arg.toInt();
              IsEEPROMWrite = true;
              request->send(200); });

  server.on("/setNewMinDistance", HTTP_POST, [](AsyncWebServerRequest *request)
            {
              String arg = request->arg("number");
              //Serial.print("New MinDistance is: ");
              //Serial.println(newMinDistance);
              newMinDistance = arg.toInt();
              IsEEPROMWrite = true;
              request->send(200); });

  server.on("/getNewMinDistance", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", String(getNewMinDistance())); });

  server.on("/getDistance", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", String(getDistance())); });

  server.on("/setExternalIPAdress", HTTP_POST, [](AsyncWebServerRequest *request)
            {
              String arg = request->arg("string");
              //Serial.print("New External IP adress is: ");
              //Serial.println(IPAdressOfExternalDevice);
              //IPAdressOfExternalDevice = arg;
              request->send(200);
              IsEEPROMWrite = true;
              IsResetDevice = true; });

  // server.on("/getExternalIPAdress", HTTP_GET, [](AsyncWebServerRequest *request)
  //           { request->send(200, "text/plain", String(IPAdressOfExternalDevice)); });

  server.on("/ExternalDeviceConnectionStatus", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", String(IsConnected)); });

  server.on("/resetFlags", HTTP_POST, [](AsyncWebServerRequest *request)
            {
             Flag = 0;
             FlagExternal = 0;
             FlagLastChangeInLoop = 0;
             IsAdded = 0;
              request->send(200); });

  server.on("/getFlags", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", String(FlagsToStrings())); });

  // String IPTrimmed = IPAdressOfExternalDevice;
  // IPTrimmed.trim();

  // Serial.print("Wi-Fi Channel: ");
  // Serial.println(WiFi.channel());

  // Set device as a Wi-Fi Station and set channel
  WiFi.mode(WIFI_AP_STA);

  // int32_t channel = getWiFiChannel(WIFI_SSID);

  // WiFi.printDiag(Serial); // Uncomment to verify channel number before
  // wifi_promiscuous_enable(1);
  // wifi_set_channel(channel);
  // wifi_promiscuous_enable(0);

  // Defined in thingProperties.h
  initProperties();

  WiFi.printDiag(Serial); // Uncomment to verify channel change after
  Serial.println(String(WiFi.macAddress()));
  // Init ESP-NOW
  if (esp_now_init() != 0)
  {
    // Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);

  // Connect to Arduino IoT Clou

  ArduinoCloud.begin(ArduinoIoTPreferredConnection, false);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
  //  Start server
  server.begin();
}

String getNewMinDistance()
{
  String newMinDistanceAsString = String(newMinDistance);
  return newMinDistanceAsString;
}

String getDistance()
{
  String newDistanceAsString = String(distance);
  return newDistanceAsString;
}

void loop()
{
  if (IsEEPROMWrite == true)
  {
    // EEPROM.put(0, 1);
    Serial.println("Writing to eeprom");
    EEPROM.put(1, cnt);
    EEPROM.put(2, newMinDistance);
    // writeStringToEEPROM(3, IPAdressOfExternalDevice);
    EEPROM.commit();
    // Serial.println("writed to eeprom");
    IsEEPROMWrite = false;
  }

  if (IsResetDevice == true)
  {
    EEPROM.put(1, cnt);
    EEPROM.commit();
    ESP.restart();
  }

  // reset device evry 4 hours
  if (samplingInterval.isExpired())
  {
    EEPROM.put(1, cnt);
    // Serial.println("writing:");
    // Serial.println(cnt);
    EEPROM.commit();
    samplingInterval.repeat();
    ESP.restart();
  }

  // check button status
  val = digitalRead(inPin); // read input value
  if (val != HIGH)
  {
    // set configportal timeout
    // set againg wifi configurator
    server.end();
    wifiManager.setConfigPortalTimeout(timeout);
    wifiManager.startConfigPortal("Counting_Wizard");
    ESP.restart();
  }

  distanceSensor.setROI(ROI_height, ROI_width, center[Zone]); // first value: height of the zone, second value: width of the zone
  delay(50);
  distanceSensor.setTimingBudgetInMs(50);
  distanceSensor.startRanging();           // Write configuration bytes to initiate measurement
  distance = distanceSensor.getDistance(); // Get the result of the measurement from the sensor
  distanceSensor.stopRanging();

  // Serial.println(distance);
  //   inject the new ranged distance in the people counting algorithm
  processPeopleCountingData(distance, Zone);
  ProcessData();

  Zone++;
  Zone = Zone % 2;

  cntForCloud = cnt;
  ArduinoCloud.update();

  DNS.processNextRequest();
  // String test;
  //  test = WiFi.BSSIDstr().c_str();
  //   strcpy(SSID, test);

  // test = wifiManager.getSSID();
  // Serial.println(WiFi.SSID());
  // Serial.println(WiFi.psk());
}

void zones_calibration()
{
  // the sensor does 100 measurements for each zone (zones are predefined)
  // each measurements is done with a timing budget of 100 ms, to increase the precision
  center[0] = 167;
  center[1] = 231;
  ROI_height = 8;
  ROI_width = 8;
  delay(500);
  Zone = 0;
  sum_zone_0 = 0;
  sum_zone_1 = 0;
  uint16_t distance;
  number_attempts = 20;

  // Preheating Sensor
  distanceSensor.startRanging();
  distanceSensor.setROI(ROI_height, ROI_width, center[Zone]); // first value: height of the zone, second value: width of the zone
  delay(50);
  distanceSensor.setTimingBudgetInMs(50);
  distanceSensor.startRanging(); // Write configuration bytes to initiate measurement
  // Serial.println("Preheating Sensor - 3 seconds");
  delay(3000);
  distance = distanceSensor.getDistance(); // Get the result of the measurement from the sensor
  distanceSensor.stopRanging();
  // Serial.println("Preheating Finished");

  for (int i = 0; i < number_attempts; i++)
  {
    // increase sum of values in Zone 0
    distanceSensor.setROI(ROI_height, ROI_width, center[Zone]); // first value: height of the zone, second value: width of the zone
    delay(50);
    distanceSensor.setTimingBudgetInMs(50);
    distanceSensor.startRanging();           // Write configuration bytes to initiate measurement
    distance = distanceSensor.getDistance(); // Get the result of the measurement from the sensor
    distanceSensor.stopRanging();
    sum_zone_0 = sum_zone_0 + distance;
    Zone++;
    Zone = Zone % 2;

    // increase sum of values in Zone 1
    distanceSensor.setROI(ROI_height, ROI_width, center[Zone]); // first value: height of the zone, second value: width of the zone
    delay(50);
    distanceSensor.setTimingBudgetInMs(50);
    distanceSensor.startRanging();           // Write configuration bytes to initiate measurement
    distance = distanceSensor.getDistance(); // Get the result of the measurement from the sensor
    distanceSensor.stopRanging();
    sum_zone_1 = sum_zone_1 + distance;
    Zone++;
    Zone = Zone % 2;
  }

  // after we have computed the sum for each zone, we can compute the average distance of each zone
  average_zone_0 = sum_zone_0 / number_attempts;
  average_zone_1 = sum_zone_1 / number_attempts;

  CalculateThresoldZonePercentage();

  // the value of the average distance is used for computing the optimal size of the ROI and consequently also the center of the two zones
  int function_of_the_distance = 16 * (1 - (0.15 * 2) / (0.34 * (min(average_zone_0, average_zone_1) / 1000)));
  delay(1000);
  int ROI_size = min(8, max(4, function_of_the_distance));
  ROI_width = ROI_size;
  ROI_height = ROI_size;
  if (advised_orientation_of_the_sensor)
  {

    switch (ROI_size)
    {
    case 4:
      center[0] = 150;
      center[1] = 247;
      break;
    case 5:
      center[0] = 150;
      center[1] = 247;
      break;
    case 6:
      center[0] = 159;
      center[1] = 239;
      break;
    case 7:
      center[0] = 159;
      center[1] = 239;
      break;
    case 8:
      center[0] = 167;
      center[1] = 231;
      break;
    }
  }
  else
  {
    switch (ROI_size)
    {
    case 4:
      center[0] = 193;
      center[1] = 58;
      break;
    case 5:
      center[0] = 194;
      center[1] = 59;
      break;
    case 6:
      center[0] = 194;
      center[1] = 59;
      break;
    case 7:
      center[0] = 195;
      center[1] = 60;
      break;
    case 8:
      center[0] = 195;
      center[1] = 60;
      break;
    }
  }

  // Serial.println("ROI size:");
  // Serial.println(ROI_size);
  // Serial.println("centers of the ROIs defined");
  // Serial.println(center[0]);
  // Serial.println(center[1]);
  // Serial.println("Setting new ROIs");

  delay(1000);
  // we will now repeat the calculations necessary to define the thresholds with the updated zones
  Zone = 0;
  sum_zone_0 = 0;
  sum_zone_1 = 0;
  for (int i = 0; i < number_attempts; i++)
  {
    // increase sum of values in Zone 0
    distanceSensor.setROI(ROI_height, ROI_width, center[Zone]); // first value: height of the zone, second value: width of the zone
    delay(50);
    distanceSensor.setTimingBudgetInMs(50);
    distanceSensor.startRanging();           // Write configuration bytes to initiate measurement
    distance = distanceSensor.getDistance(); // Get the result of the measurement from the sensor
    distanceSensor.stopRanging();
    sum_zone_0 = sum_zone_0 + distance;
    Zone++;
    Zone = Zone % 2;

    // increase sum of values in Zone 1
    distanceSensor.setROI(ROI_height, ROI_width, center[Zone]); // first value: height of the zone, second value: width of the zone
    delay(50);
    distanceSensor.setTimingBudgetInMs(50);
    distanceSensor.startRanging();           // Write configuration bytes to initiate measurement
    distance = distanceSensor.getDistance(); // Get the result of the measurement from the sensor
    distanceSensor.stopRanging();
    sum_zone_1 = sum_zone_1 + distance;
    Zone++;
    Zone = Zone % 2;
  }
  average_zone_0 = sum_zone_0 / number_attempts;
  average_zone_1 = sum_zone_1 / number_attempts;

  CalculateThresoldZonePercentage();

  float threshold_zone_0 = average_zone_0 * threshold_percentage / 100; // they can be int values, as we are not interested in the decimal part when defining the threshold
  float threshold_zone_1 = average_zone_1 * threshold_percentage / 100;

  DIST_THRESHOLD_MAX[0] = threshold_zone_0;
  DIST_THRESHOLD_MAX[1] = threshold_zone_1;
  delay(2000);

  // we now save the values into the EEPROM memory
  int hundred_threshold_zone_0 = threshold_zone_0 / 100;
  int hundred_threshold_zone_1 = threshold_zone_1 / 100;
  int unit_threshold_zone_0 = threshold_zone_0 - 100 * hundred_threshold_zone_0;
  int unit_threshold_zone_1 = threshold_zone_1 - 100 * hundred_threshold_zone_1;
}

void CalculateThresoldZonePercentage()
// calculatiing thersold zone
{
  // Serial.println("Average Zone 0:");
  // Serial.println(average_zone_0);
  // Serial.println("Average Zone 1:");
  // Serial.println(average_zone_1);
  if (min(average_zone_0, average_zone_1) <= 130)
  {
    // Serial.println("small distance");
    threshold_percentage = 80;
    distanceSensor.setDistanceModeShort();
    // Serial.println(threshold_percentage);
  }
  else
  {
    // 70 suppose to be ambient light immune
    // Serial.println("long distance");
    threshold_percentage = (70 / min(average_zone_0, average_zone_1)) * 1000;
    distanceSensor.setDistanceModeLong();
    // Serial.println(threshold_percentage);
  }
}

// NOBODY = 0, SOMEONE = 1, LEFT = 0, RIGHT = 1

void processPeopleCountingData(int16_t Distance, uint8_t zone)
{

  int CurrentZoneStatus = NOBODY;
  int AllZonesCurrentStatus = 0;
  int AnEventHasOccured = 0;

  // minium distance set to 120 to limit door opening and closing
  if (Distance < DIST_THRESHOLD_MAX[Zone] && Distance > newMinDistance)
  {
    CurrentZoneStatus = SOMEONE;
  }

  // left zone
  if (zone == LEFT)
  {

    if (CurrentZoneStatus != LeftPreviousStatus)
    {
      // event in left zone has occured
      AnEventHasOccured = 1;

      if (CurrentZoneStatus == SOMEONE)
      {
        AllZonesCurrentStatus += 1;
      }
      // need to check right zone as well ...
      if (RightPreviousStatus == SOMEONE)
      {
        // event in left zone has occured
        AllZonesCurrentStatus += 2;
      }
      // remember for next time
      LeftPreviousStatus = CurrentZoneStatus;
    }
  }
  // right zone
  else
  {

    if (CurrentZoneStatus != RightPreviousStatus)
    {

      // event in left zone has occured
      AnEventHasOccured = 1;
      if (CurrentZoneStatus == SOMEONE)
      {
        AllZonesCurrentStatus += 2;
      }
      // need to left right zone as well ...
      if (LeftPreviousStatus == SOMEONE)
      {
        // event in left zone has occured
        AllZonesCurrentStatus += 1;
      }
      // remember for next time
      RightPreviousStatus = CurrentZoneStatus;
    }
  }

  // if an event has occured
  if (AnEventHasOccured)
  {
    if (PathTrackFillingSize < 4)
    {
      PathTrackFillingSize++;
    }

    // if nobody anywhere lets check if an exit or entry has happened
    if ((LeftPreviousStatus == NOBODY) && (RightPreviousStatus == NOBODY))
    {

      // check exit or entry only if PathTrackFillingSize is 4 (for example 0 1 3 2) and last event is 0 (nobobdy anywhere)
      if (PathTrackFillingSize == 4)
      {
        // check exit or entry. no need to check PathTrack[0] == 0 , it is always the case
        // Serial.println("false positive?");
        if ((PathTrack[1] == 1) && (PathTrack[2] == 3) && (PathTrack[3] == 2))
        {
          // this is an entry
          // Serial.println("Entering");
          FlagForFlow(1);
          // ws.printfAll("1");
          PostMessageToExternalDevice(1);
        }
        else if ((PathTrack[1] == 2) && (PathTrack[2] == 3) && (PathTrack[3] == 1))
        {
          // This an exit
          // Serial.println("Exiting");
          FlagForFlow(2);
          // ws.printfAll("2");
          PostMessageToExternalDevice(2);
        }
      }
      for (int i = 0; i < 4; i++)
      {
        PathTrack[i] = 0;
      }
      PathTrackFillingSize = 1;
    }
    else
    {
      // update PathTrack
      // example of PathTrack update
      // 0
      // 0 1
      // 0 1 3
      // 0 1 3 1
      // 0 1 3 3
      // 0 1 3 2 ==> if next is 0 : check if exit
      PathTrack[PathTrackFillingSize - 1] = AllZonesCurrentStatus;
    }
  }
}

String handleADC()
{
  String adc = String(cnt);
  return adc;
}

String FlagsToStrings()
{
  String FlagsSumAsString;
  FlagsSumAsString = "Flags " + String(Flag) + ";" + "EnumFlagExternal " + String(FlagExternal) + ";" + "EnumFlagLastChangeInLoop " + String(FlagLastChangeInLoop) + ";" + "IsAdded " + String(IsAdded);
  return FlagsSumAsString;
}

extern "C"
{
#include "user_interface.h"
}

// Set your Board ID (ESP32 Sender #1 = BOARD_ID 1, ESP32 Sender #2 = BOARD_ID 2, etc)
//#define BOARD_ID 2

/* clients events */
// static void handleError(void *arg, AsyncClient *client, int8_t error)
// {
//   // Serial.printf("\n connection error %s from client %s \n", client->errorToString(error), client->remoteIP().toString().c_str());
// }

// static void handleData(void *arg, AsyncClient *client, void *data, size_t len)
// {
//   // Serial.printf("\n data received from client %s \n", client->remoteIP().toString().c_str());
//   Serial.write((uint8_t *)data, len);

//   // reply to client
//   if (client->space() > 32 && client->canSend())
//   {
//     char reply[32];
//     sprintf(reply, "this is from %s", SERVER_HOST_NAME);
//     client->add(reply, strlen(reply));
//     client->send();
//   }
// }

// static void handleDisconnect(void *arg, AsyncClient *client)
// {
//   // Serial.printf("\n client %s disconnected \n", client->remoteIP().toString().c_str());
// }

// static void handleTimeOut(void *arg, AsyncClient *client, uint32_t time)
// {
//   // Serial.printf("\n client ACK timeout ip: %s \n", client->remoteIP().toString().c_str());
// }

/* server events */
// static void handleNewClient(void *arg, AsyncClient *client)
// {
//   // Serial.printf("\n new client has been connected to server, ip: %s", client->remoteIP().toString().c_str());

//   // add to list
//   clients.push_back(client);

//   // register events
//   client->onData(&handleData, NULL);
//   client->onError(&handleError, NULL);
//   client->onDisconnect(&handleDisconnect, NULL);
//   client->onTimeout(&handleTimeOut, NULL);
// }

// 0-null; 1-entered; 2-exited;
void FlagForFlow(int flag)
{
  Flag = flag;
}

void FlagForFlowExternalDevice(int flag)
{
  FlagExternal = flag;
}

void writeStringToEEPROM(int addrOffset, const String &strToWrite)
{
  byte len = strToWrite.length();
  EEPROM.put(addrOffset, len);
  for (int i = 0; i < len; i++)
  {
    EEPROM.put(addrOffset + 1 + i, strToWrite[i]);
  }
}

String readStringFromEEPROM(int addrOffset)
{
  int newStrLen = EEPROM.read(addrOffset);
  char data[newStrLen + 1];
  for (int i = 0; i < newStrLen; i++)
  {
    data[i] = EEPROM.read(addrOffset + 1 + i);
  }
  data[newStrLen] = '\0'; // !!! NOTE !!! Remove the space between the slash "/" and "0" (I've added a space because otherwise there is a display bug)
  return String(data);
}

void ProcessData()
{
  if (Flag == 1)
  {
    switch (FlagExternal)
    {
    case 0:
    case 4:
    case 1:
      cnt++;
      IsEEPROMWrite = true;
      Flag = 3;
      IsAdded = true;
      // Serial.println("Internal Flag:");
      // Serial.println(Flag);
      // Serial.println("Is added?");
      // Serial.println(IsAdded);
      // Serial.println("Flag External:");
      // Serial.println(FlagExternal);
      break;
    case 3:
      if (IsAdded)
      {
        Flag = 0;
        FlagExternal = 0;
        // Serial.println("Internal Flag:");
        // Serial.println(Flag);
        // Serial.println("Is added?");
        // Serial.println(IsAdded);
        // Serial.println("Flag External:");
        // Serial.println(FlagExternal);
        break;
      }
      else
      {
        cnt++;
        IsEEPROMWrite = true;
        Flag = 0;
        FlagExternal = 0;
        IsAdded = true;
        // Serial.println("Internal Flag:");
        // Serial.println(Flag);
        // Serial.println("Is added?");
        // Serial.println(IsAdded);
        // Serial.println("Flag External:");
        // Serial.println(FlagExternal);
        break;
      }
    }
  }

  if (FlagExternal == 1)
  {
    switch (Flag)
    {
    case 0:
    case 4:
    case 1:
      cnt++;
      IsEEPROMWrite = true;
      FlagExternal = 3;
      IsAdded = true;
      // Serial.println("Internal Flag:");
      // Serial.println(Flag);
      // Serial.println("Is added?");
      // Serial.println(IsAdded);
      // Serial.println("Flag External:");
      // Serial.println(FlagExternal);
      break;
    case 3:
      if (IsAdded)
      {
        Flag = 0;
        FlagExternal = 0;
        // Serial.println("Internal Flag:");
        // Serial.println(Flag);
        // Serial.println("Is added?");
        // Serial.println(IsAdded);
        // Serial.println("Flag External:");
        // Serial.println(FlagExternal);
        break;
      }
      else
      {
        cnt++;
        IsEEPROMWrite = true;
        Flag = 0;
        FlagExternal = 0;
        IsAdded = true;
        // Serial.println("Internal Flag:");
        // Serial.println(Flag);
        // Serial.println("Is added?");
        // Serial.println(IsAdded);
        // Serial.println("Flag External:");
        // Serial.println(FlagExternal);
        break;
      }
    }
  }

  if (Flag == 2)
  {
    switch (FlagExternal)
    {
    case 0:
    case 3:
    case 2:
      cnt--;
      IsEEPROMWrite = true;
      Flag = 4;
      IsAdded = false;
      // Serial.println("Internal Flag:");
      // Serial.println(Flag);
      // Serial.println("Is added?");
      // Serial.println(IsAdded);
      // Serial.println("Flag External:");
      // Serial.println(FlagExternal);
      break;

    case 4:
      if (IsAdded)
      {
        cnt--;
        IsEEPROMWrite = true;
        Flag = 0;
        FlagExternal = 0;
        IsAdded = false;
        // Serial.println("Internal Flag:");
        // Serial.println(Flag);
        // Serial.println("Is added?");
        // Serial.println(IsAdded);
        // Serial.println("Flag External:");
        // Serial.println(FlagExternal);
        break;
      }
      else
      {
        Flag = 0;
        FlagExternal = 0;
        // Serial.println("Internal Flag:");
        // Serial.println(Flag);
        // Serial.println("Is added?");
        // Serial.println(IsAdded);
        // Serial.println("Flag External:");
        // Serial.println(FlagExternal);
        break;
      }
    }
  }

  if (FlagExternal == 2)
  {
    switch (Flag)
    {
    case 0:
    case 3:
    case 2:
      cnt--;
      IsEEPROMWrite = true;
      FlagExternal = 4;
      IsAdded = false;
      // Serial.println("Internal Flag:");
      // Serial.println(Flag);
      // Serial.println("Is added?");
      // Serial.println(IsAdded);
      // Serial.println("Flag External:");
      // Serial.println(FlagExternal);
      break;
    case 4:
      if (IsAdded)
      {
        cnt--;
        IsEEPROMWrite = true;
        Flag = 0;
        FlagExternal = 0;
        IsAdded = false;
        // Serial.println("Internal Flag:");
        // Serial.println(Flag);
        // Serial.println("Is added?");
        // Serial.println(IsAdded);
        // Serial.println("Flag External:");
        // Serial.println(FlagExternal);
        break;
      }
      else
      {
        Flag = 0;
        FlagExternal = 0;
        // Serial.println("Internal Flag:");
        // Serial.println(Flag);
        // Serial.println("Is added?");
        // Serial.println(IsAdded);
        // Serial.println("Flag External:");
        // Serial.println(FlagExternal);
        break;
      }
    }
  }
}

void PostMessageToExternalDevice(int value)
{
  // HTTPClient http; // Declare object of class HTTPClient
  // Serial.println("Posting to external device...");

  myData.cnt_espNow = value;
  esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  i = 0;
  // String messageToPost = "http://192.168.4.1:80/PostMessageToExternalDevice?number=" + String(value);
  //  http.begin(messageToPost);                    // Specify request destination
  //  http.addHeader("Content-Type", "text/plain"); // Specify content-type header
  // String httpRequestData = ""; //+ value;
  // Serial.println(httpRequestData);
  //  int httpCode = http.POST(httpRequestData); // Send the request
  //   String payload = http.getString();         // Get the response payload
  //   Serial.println(httpCode); // Print HTTP return code
  //   Serial.println(payload);  // Print request response payload
  //  http.end(); // Close connection
}

// callback function that will be executed when data is received
void OnDataRecv(uint8_t *mac_addr, uint8_t *incomingData, uint8_t len)
{
  // Copies the sender mac address to a string
  // char macStr[18];
  // Serial.print("Packet received from: ");
  // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //         mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // Serial.println(macStr);
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));

  FlagForFlowExternalDevice(incomingReadings.cnt_espNow);

  // Serial.printf("Board ID %u: %u bytes\n", incomingReadings.cnt_espNow, len);
  // Serial.println();
}

// uint8_t status_OnDataSent;
//  Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus)
{
  // status_OnDataSent = sendStatus;
  //  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0)
  {
    // Serial.println("Delivery success");
    //  i = 0;
  }
  else
  {
    if (i < j)
    {
      i++;
      esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
      // delay(1000);
    }
    // i = 0;
    // Serial.println(String(i));
    // Serial.println("Delivery fail");
  }
}

void onCntChangedChange()
{
  if (flagToNotReadyCntChangedOnStart == true)
  {
    IsEEPROMWrite = true;
    cnt = cntChanged;
  }
  flagToNotReadyCntChangedOnStart = true;
}
