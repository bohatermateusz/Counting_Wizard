//Spartfun Libraries
#include "SparkFun_VL53L1X.h"

// Acess point
//#include <ESP8266WiFi.h>
//#include <WiFiClient.h>

// WebServer
#include <ESPAsyncTCP.h>
//#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <Wire.h>

#include "WiFiManager.h"
WiFiManager wifiManager;

#define WEBSERVER_H
#include "ESPAsyncWebServer.h"

//SSID and Password to your ESP Access Point
//const char *ssid = "Counting-Wizard";
//const char *password = "pass-123456";

AsyncWebServer server(80);
DNSServer dns;

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

int cnt = 0;

float sum_zone_0;
float sum_zone_1;

float number_attempts;

float average_zone_0 = sum_zone_0 / number_attempts;
float average_zone_1 = sum_zone_1 / number_attempts;

void setup()
{
    Wire.begin();
    Serial.begin(115200);

    wifiManager.autoConnect("AP-NAME");

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

    // Create Access Point
    //WiFi.mode(WIFI_AP);
    delay(100);

    //IPAddress localIp(192, 168, 1, 1);
    //IPAddress gateway(192, 168, 1, 1);
    //IPAddress subnet(255, 255, 255, 0);

    //WiFi.softAPConfig(localIp, gateway, subnet);

    // boolean result = WiFi.softAP(ssid, password, 5);
    // if (result == true)
    // {
    //     Serial.println("WIFI AP is Ready");
    //     //Get IP address
    //     IPAddress myIP = WiFi.softAPIP();
    //     Serial.print("HotSpt IP:");
    //     Serial.println(myIP);
    // }
    // else
    // {
    //     Serial.println("Failed to start WIFI AP");
    // }

    if (!SPIFFS.begin())
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    //Configure Webserver
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

    // Start server
    server.begin();
    Serial.println("HTTP server started");
}

void loop()
{
    uint16_t distance;

    distanceSensor.setROI(ROI_height, ROI_width, center[Zone]); // first value: height of the zone, second value: width of the zone
    delay(50);
    distanceSensor.setTimingBudgetInMs(50);
    distanceSensor.startRanging();           //Write configuration bytes to initiate measurement
    distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
    distanceSensor.stopRanging();

    //Serial.println(distance);
    // inject the new ranged distance in the people counting algorithm
    processPeopleCountingData(distance, Zone);

    Zone++;
    Zone = Zone % 2;
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
    distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
    Serial.println("Preheating Sensor - 10 seconds");
    delay(10000);
    distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
    distanceSensor.stopRanging();
    Serial.println("Preheating Finished");

    for (int i = 0; i < number_attempts; i++)
    {
        // increase sum of values in Zone 0
        distanceSensor.setROI(ROI_height, ROI_width, center[Zone]); // first value: height of the zone, second value: width of the zone
        delay(50);
        distanceSensor.setTimingBudgetInMs(50);
        distanceSensor.startRanging();           //Write configuration bytes to initiate measurement
        distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
        distanceSensor.stopRanging();
        sum_zone_0 = sum_zone_0 + distance;
        Zone++;
        Zone = Zone % 2;

        // increase sum of values in Zone 1
        distanceSensor.setROI(ROI_height, ROI_width, center[Zone]); // first value: height of the zone, second value: width of the zone
        delay(50);
        distanceSensor.setTimingBudgetInMs(50);
        distanceSensor.startRanging();           //Write configuration bytes to initiate measurement
        distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
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
        distanceSensor.startRanging();           //Write configuration bytes to initiate measurement
        distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
        distanceSensor.stopRanging();
        sum_zone_0 = sum_zone_0 + distance;
        Zone++;
        Zone = Zone % 2;

        // increase sum of values in Zone 1
        distanceSensor.setROI(ROI_height, ROI_width, center[Zone]); // first value: height of the zone, second value: width of the zone
        delay(50);
        distanceSensor.setTimingBudgetInMs(50);
        distanceSensor.startRanging();           //Write configuration bytes to initiate measurement
        distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
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
//calculatiing thersold zone
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
        //70 suppose to be ambient light immune
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
    if (Distance < DIST_THRESHOLD_MAX[Zone] && Distance > 120)
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
                Serial.println();
                if ((PathTrack[1] == 1) && (PathTrack[2] == 3) && (PathTrack[3] == 2))
                {
                    // this is an entry
                    Serial.println("Entering");
                    cnt++;
                    Serial.println(handleADC());
                }
                else if ((PathTrack[1] == 2) && (PathTrack[2] == 3) && (PathTrack[3] == 1))
                {
                    // This an exit
                    Serial.println("Exiting");
                    cnt--;
                    Serial.println(handleADC());
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
