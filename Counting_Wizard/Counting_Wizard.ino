// Spartfun Libraries
#include "SparkFun_VL53L1X.h"

// WebServer
#include <ESPAsyncTCP.h>
#include <FS.h>
#include <Wire.h>
#include <vector>
#include "config.h"

static std::vector<AsyncClient *> clients; // a list to hold all clients

// AsyncWebSocketClient
#include <WebSocketsClient.h>
#include <Hash.h>
WebSocketsClient webSocket;
#define USE_SERIAL Serial

#include "WiFiManager.h"
WiFiManager wifiManager;

#define WEBSERVER_H
#include "ESPAsyncWebServer.h"

#define DNS_PORT 53
AsyncWebServer server(80);
DNSServer DNS;

//
AsyncWebSocket ws("/ws"); // access at ws://[esp ip]/ws
AsyncEventSource events("/events");
//

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
int newMinDistance = 30;

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
unsigned long lastChange;
int FlagExternal;
unsigned long lastChangeExternal;
int FlagLastChangeInLoop;
unsigned long lastChangeInLoop;

// Async Delay
//#include <AsyncDelay.h>
// AsyncDelay samplingInterval;

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

String IPAdressOfExternalDevice;

bool IsConnected;

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  // EEPROM
  EEPROM.begin(4096);
  delay(100);
  // samplingInterval.start(375, AsyncDelay::MILLIS);
  for (uint8_t t = 4; t > 0; t--)
  {
    USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
    USE_SERIAL.flush();
    delay(1000);
  }

  // pinmode for button purpose
  pinMode(inPin, INPUT);

  wifiManager.autoConnect("Counting_Wizard");
  delay(100);

  AsyncServer *serverAA = new AsyncServer(TCP_PORT); // start listening on tcp port 7050
  serverAA->onClient(&handleNewClient, serverAA);
  serverAA->begin();

  Serial.println("VL53L1X Qwiic Test");
  if (distanceSensor.init() == false)
    Serial.println("Sensor online!");
  distanceSensor.setIntermeasurementPeriod(100);
  distanceSensor.setDistanceModeLong();

  delay(1000);
  zones_calibration();

  Serial.println("Thresold0:");
  Serial.println(DIST_THRESHOLD_MAX[0]);
  Serial.println("Thresold1:");
  Serial.println(DIST_THRESHOLD_MAX[1]);
  Serial.println();

  if (!SPIFFS.begin())
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Configure Webserver
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index.html", String(), false); });

  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/style.css", "text/css"); });

  server.on("/GaugeMeter.js", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/GaugeMeter.js", "text/css"); });

  server.on("/jquery.min.js", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/jquery.min.js", "text/css"); });

  server.on("/jquery-3.3.1.min.js", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/jquery-3.3.1.min.js", "text/css"); });

  server.on("/knockout-min.js", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/knockout-min.js", "text/css"); });

  server.on("/getADC", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", String(handleADC())); });

  server.on("/ControlPanel", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/ControlPanel.html", String(), false); });

  server.on("/add", HTTP_POST, [](AsyncWebServerRequest *request)
            {
              Serial.print("Adding one person");
              cnt++;
              request->send(200); });

  server.on("/subtract", HTTP_POST, [](AsyncWebServerRequest *request)
            {
              Serial.print("Subtract one person");
              cnt--;
              request->send(200); });

  server.on("/set", HTTP_POST, [](AsyncWebServerRequest *request)
            {
              String arg = request->arg("number");
              Serial.print("New people value is: ");
              Serial.println(arg);
              cnt = arg.toInt();
              request->send(200); });

  server.on("/setNewLimit", HTTP_POST, [](AsyncWebServerRequest *request)
            {
              String arg = request->arg("number");
              Serial.print("New limit is: ");
              Serial.println(arg);
              limit = arg.toInt();
              request->send(200); });

  server.on("/getNewLimit", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", String(getLimit())); });

  server.on("/setNewMinDistance", HTTP_POST, [](AsyncWebServerRequest *request)
            {
              String arg = request->arg("number");
              Serial.print("New MinDistance is: ");
              Serial.println(newMinDistance);
              newMinDistance = arg.toInt();
              request->send(200); });

  server.on("/getNewMinDistance", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", String(getNewMinDistance())); });

  server.on("/getDistance", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", String(getDistance())); });

  server.on("/setExternalIPAdress", HTTP_POST, [](AsyncWebServerRequest *request)
            {
              String arg = request->arg("string");
              Serial.print("New External IP adress is: ");
              Serial.println(IPAdressOfExternalDevice);
              IPAdressOfExternalDevice = arg;
              request->send(200); });

  server.on("/getExternalIPAdress", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", String(IPAdressOfExternalDevice)); });

  server.on("/ExternalDeviceConnectionStatus", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", String(IsConnected)); });

  // attach AsyncWebSocket
  ws.onEvent(onEvent);
  server.addHandler(&ws);

  // Start server
  server.begin();
  Serial.println("HTTP server started");

  // Read EEPROM
  if (EEPROM.read(0) == 1)
  {
    cnt = EEPROM.read(1);
    newMinDistance = EEPROM.read(2);
    IPAdressOfExternalDevice = readStringFromEEPROM(3);
  }

  String IPTrimmed = IPAdressOfExternalDevice;
  IPTrimmed.trim();
  // server address, port and URL
  webSocket.begin(IPTrimmed, 80, "/ws");

  // event handler
  webSocket.onEvent(webSocketEvent);
  // try ever 5000 again if connection has failed
  webSocket.setReconnectInterval(5000);

  // start heartbeat (optional)
  // ping server every 15000 ms
  // expect pong from server within 3000 ms
  // consider connection disconnected if pong is not received 2 times
  webSocket.enableHeartbeat(15000, 3000, 2);
  Serial.println("webSocket Client started");
}

String getLimit()
{
  String limitAsString = String(limit);
  return limitAsString;
}

String getNewMinDistance()
{
  String newMinDistanceAsString = String(newMinDistance);
  return newMinDistanceAsString;
}

String getDistance()
{
  String newMinDistanceAsString = String(distance);
  return newMinDistanceAsString;
}

void loop()
{
  ProcessData();

  // EEPROM to save counting and min Distance values
  EEPROM.write(0, 1);
  EEPROM.write(1, cnt);
  EEPROM.write(2, newMinDistance);
  writeStringToEEPROM(3, IPAdressOfExternalDevice);
  EEPROM.commit();

  webSocket.loop();
  DNS.processNextRequest();
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

  // ProcessData();

  distanceSensor.setROI(ROI_height, ROI_width, center[Zone]); // first value: height of the zone, second value: width of the zone
  delay(50);
  distanceSensor.setTimingBudgetInMs(50);
  distanceSensor.startRanging();           // Write configuration bytes to initiate measurement
  distance = distanceSensor.getDistance(); // Get the result of the measurement from the sensor
  distanceSensor.stopRanging();

  // Serial.println(distance);
  //  inject the new ranged distance in the people counting algorithm
  processPeopleCountingData(distance, Zone);

  Zone++;
  Zone = Zone % 2;

  // Clean flag evry 375ms to cover example when one person enter and after long time second enter and only second counter catch that enter
  // if (samplingInterval.isExpired())
  //{
  //  Flag = 0;
  //  FlagExternal = 0;
  //  samplingInterval.repeat();
  //}
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
  Serial.println("Preheating Sensor - 10 seconds");
  delay(10000);
  distance = distanceSensor.getDistance(); // Get the result of the measurement from the sensor
  distanceSensor.stopRanging();
  Serial.println("Preheating Finished");

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

  Serial.println("ROI size:");
  Serial.println(ROI_size);
  Serial.println("centers of the ROIs defined");
  Serial.println(center[0]);
  Serial.println(center[1]);
  Serial.println("Setting new ROIs");

  delay(2000);
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
  Serial.println("Average Zone 0:");
  Serial.println(average_zone_0);
  Serial.println("Average Zone 1:");
  Serial.println(average_zone_1);
  if (min(average_zone_0, average_zone_1) <= 130)
  {
    Serial.println("small distance");
    threshold_percentage = 80;
    distanceSensor.setDistanceModeShort();
    Serial.println(threshold_percentage);
  }
  else
  {
    // 70 suppose to be ambient light immune
    Serial.println("long distance");
    threshold_percentage = (70 / min(average_zone_0, average_zone_1)) * 1000;
    distanceSensor.setDistanceModeLong();
    Serial.println(threshold_percentage);
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
        Serial.println("false positive?");
        if ((PathTrack[1] == 1) && (PathTrack[2] == 3) && (PathTrack[3] == 2))
        {
          // this is an entry
          Serial.println("Entering");
          FlagForFlow(1);
          ws.printfAll("1");
        }
        else if ((PathTrack[1] == 2) && (PathTrack[2] == 3) && (PathTrack[3] == 1))
        {
          // This an exit
          Serial.println("Exiting");
          FlagForFlow(2);
          ws.printfAll("2");
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

extern "C"
{
#include "user_interface.h"
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  if (type == WS_EVT_CONNECT)
  {
    // client connected
    os_printf("ws[%s][%u] connect\n", server->url(), client->id());
    client->printf("Hello Client %u :)", client->id());
    client->ping();
  }
  else if (type == WS_EVT_DISCONNECT)
  {
    // client disconnected
    os_printf("ws[%s][%u] disconnect: %u\n", server->url(), client->id());
  }
  else if (type == WS_EVT_ERROR)
  {
    // error was received from the other end
    os_printf("ws[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t *)arg), (char *)data);
  }
  else if (type == WS_EVT_PONG)
  {
    // pong message was received (in response to a ping request maybe)
    os_printf("ws[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len) ? (char *)data : "");
  }
  else if (type == WS_EVT_DATA)
  {
    // data packet
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len)
    {
      // the whole message is in a single frame and we got all of it's data
      os_printf("ws[%s][%u] %s-message[%llu]: ", server->url(), client->id(), (info->opcode == WS_TEXT) ? "text" : "binary", info->len);
      if (info->opcode == WS_TEXT)
      {
        data[len] = 0;
        os_printf("%s\n", (char *)data);
      }
      else
      {
        for (size_t i = 0; i < info->len; i++)
        {
          os_printf("%02x ", data[i]);
        }
        os_printf("\n");
      }
      if (info->opcode == WS_TEXT)
        client->text("I have got your text message");
      else
        client->binary("I got your binary message");
    }
    else
    {
      // message is comprised of multiple frames or the frame is split into multiple packets
      if (info->index == 0)
      {
        if (info->num == 0)
          os_printf("ws[%s][%u] %s-message start\n", server->url(), client->id(), (info->message_opcode == WS_TEXT) ? "text" : "binary");
        os_printf("ws[%s][%u] frame[%u] start[%llu]\n", server->url(), client->id(), info->num, info->len);
      }

      os_printf("ws[%s][%u] frame[%u] %s[%llu - %llu]: ", server->url(), client->id(), info->num, (info->message_opcode == WS_TEXT) ? "text" : "binary", info->index, info->index + len);
      if (info->message_opcode == WS_TEXT)
      {
        data[len] = 0;
        os_printf("%s\n", (char *)data);
      }
      else
      {
        for (size_t i = 0; i < len; i++)
        {
          os_printf("%02x ", data[i]);
        }
        os_printf("\n");
      }

      if ((info->index + len) == info->len)
      {
        os_printf("ws[%s][%u] frame[%u] end[%llu]\n", server->url(), client->id(), info->num, info->len);
        if (info->final)
        {
          os_printf("ws[%s][%u] %s-message end\n", server->url(), client->id(), (info->message_opcode == WS_TEXT) ? "text" : "binary");
          if (info->message_opcode == WS_TEXT)
            client->text("I got your text message");
          else
            client->binary("I got your binary message");
        }
      }
    }
  }
}

/* clients events */
static void handleError(void *arg, AsyncClient *client, int8_t error)
{
  Serial.printf("\n connection error %s from client %s \n", client->errorToString(error), client->remoteIP().toString().c_str());
}

static void handleData(void *arg, AsyncClient *client, void *data, size_t len)
{
  Serial.printf("\n data received from client %s \n", client->remoteIP().toString().c_str());
  Serial.write((uint8_t *)data, len);

  // reply to client
  if (client->space() > 32 && client->canSend())
  {
    char reply[32];
    sprintf(reply, "this is from %s", SERVER_HOST_NAME);
    client->add(reply, strlen(reply));
    client->send();
  }
}

static void handleDisconnect(void *arg, AsyncClient *client)
{
  Serial.printf("\n client %s disconnected \n", client->remoteIP().toString().c_str());
}

static void handleTimeOut(void *arg, AsyncClient *client, uint32_t time)
{
  Serial.printf("\n client ACK timeout ip: %s \n", client->remoteIP().toString().c_str());
}

/* server events */
static void handleNewClient(void *arg, AsyncClient *client)
{
  Serial.printf("\n new client has been connected to server, ip: %s", client->remoteIP().toString().c_str());

  // add to list
  clients.push_back(client);

  // register events
  client->onData(&handleData, NULL);
  client->onError(&handleError, NULL);
  client->onDisconnect(&handleDisconnect, NULL);
  client->onTimeout(&handleTimeOut, NULL);
}

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{

  switch (type)
  {
  case WStype_DISCONNECTED:
    USE_SERIAL.printf("[WSc] Disconnected!\n");
    IsConnected = false;
    break;
  case WStype_CONNECTED:
  {
    USE_SERIAL.printf("[WSc] Connected to url: %s\n", payload);
    IsConnected = true;
    // send message to server when Connected
    webSocket.sendTXT("Connected");
  }
  break;
  case WStype_TEXT:
    // 1 IS SOMEONE ENETERD
    if (payload[0] == '1')
    {
      Serial.println("External Device Sent: Entered");
      FlagForFlowExternalDevice(1);
    }
    // 0 IS NOONE ENETERED
    if (payload[0] == '2')
    {
      Serial.println("External Device Sent: Exit");
      FlagForFlowExternalDevice(2);
    }

    USE_SERIAL.printf("[WSc] get text: %s\n", payload);
    // String customEnter = "Entering " + String(FlagEnter);
    Serial.println(String(Flag));
    // String customExit = "Exiting " + String(FlagExit);

    // send message to server
    // webSocket.sendTXT("sample message here");
    break;
  case WStype_BIN:
    USE_SERIAL.printf("[WSc] get binary length: %u\n", length);
    hexdump(payload, length);

    // send data to server
    // webSocket.sendBIN(payload, length);
    break;
  case WStype_PING:
    // pong will be send automatically
    USE_SERIAL.printf("[WSc] get ping\n");
    break;
  case WStype_PONG:
    // answer to a ping we send
    USE_SERIAL.printf("[WSc] get pong\n");
    break;
  }
}

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
  EEPROM.write(addrOffset, len);
  for (int i = 0; i < len; i++)
  {
    EEPROM.write(addrOffset + 1 + i, strToWrite[i]);
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
      cnt++;
      Flag = 3;
      IsAdded = true;
      Serial.println("Internal Flag:");
      Serial.println(Flag);
      Serial.println("Is added?");
      Serial.println(IsAdded);
      Serial.println("Flag External:");
      Serial.println(FlagExternal);
      break;
    case 3:
      if (IsAdded)
      {
        Flag = 0;
        FlagExternal = 0;
        Serial.println("Internal Flag:");
        Serial.println(Flag);
        Serial.println("Is added?");
        Serial.println(IsAdded);
        Serial.println("Flag External:");
        Serial.println(FlagExternal);
        break;
      }
      else
      {
        cnt++;
        Flag = 0;
        FlagExternal = 0;
        IsAdded = true;
        Serial.println("Internal Flag:");
        Serial.println(Flag);
        Serial.println("Is added?");
        Serial.println(IsAdded);
        Serial.println("Flag External:");
        Serial.println(FlagExternal);
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
      cnt++;
      FlagExternal = 3;
      IsAdded = true;
      Serial.println("Internal Flag:");
      Serial.println(Flag);
      Serial.println("Is added?");
      Serial.println(IsAdded);
      Serial.println("Flag External:");
      Serial.println(FlagExternal);
      break;
    case 3:
      if (IsAdded)
      {
        Flag = 0;
        FlagExternal = 0;
        Serial.println("Internal Flag:");
        Serial.println(Flag);
        Serial.println("Is added?");
        Serial.println(IsAdded);
        Serial.println("Flag External:");
        Serial.println(FlagExternal);
        break;
      }
      else
      {
        cnt++;
        Flag = 0;
        FlagExternal = 0;
        IsAdded = true;
        Serial.println("Internal Flag:");
        Serial.println(Flag);
        Serial.println("Is added?");
        Serial.println(IsAdded);
        Serial.println("Flag External:");
        Serial.println(FlagExternal);
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
      cnt--;
      Flag = 4;
      IsAdded = false;
      Serial.println("Internal Flag:");
      Serial.println(Flag);
      Serial.println("Is added?");
      Serial.println(IsAdded);
      Serial.println("Flag External:");
      Serial.println(FlagExternal);
      break;

    case 4:
      if (IsAdded)
      {
        cnt--;
        Flag = 0;
        FlagExternal = 0;
        IsAdded = false;
        Serial.println("Internal Flag:");
        Serial.println(Flag);
        Serial.println("Is added?");
        Serial.println(IsAdded);
        Serial.println("Flag External:");
        Serial.println(FlagExternal);
        break;
      }
      else
      {
        Flag = 0;
        FlagExternal = 0;
        Serial.println("Internal Flag:");
        Serial.println(Flag);
        Serial.println("Is added?");
        Serial.println(IsAdded);
        Serial.println("Flag External:");
        Serial.println(FlagExternal);
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
      cnt--;
      FlagExternal = 4;
      IsAdded = false;
      Serial.println("Internal Flag:");
      Serial.println(Flag);
      Serial.println("Is added?");
      Serial.println(IsAdded);
      Serial.println("Flag External:");
      Serial.println(FlagExternal);
      break;
    case 4:
      if (IsAdded)
      {
        cnt--;
        Flag = 0;
        FlagExternal = 0;
        IsAdded = false;
        Serial.println("Internal Flag:");
        Serial.println(Flag);
        Serial.println("Is added?");
        Serial.println(IsAdded);
        Serial.println("Flag External:");
        Serial.println(FlagExternal);
        break;
      }
      else
      {
        Flag = 0;
        FlagExternal = 0;
        Serial.println("Internal Flag:");
        Serial.println(Flag);
        Serial.println("Is added?");
        Serial.println(IsAdded);
        Serial.println("Flag External:");
        Serial.println(FlagExternal);
        break;
      }
    }
  }
}
