// Code generated by Arduino IoT Cloud, DO NOT EDIT.

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>

const char DEVICE_LOGIN_NAME[] = "46c6bd53-278f-4d20-b9ad-1e35ea7a8ee8";

char SSID[20];                               // = SECRET_SSID;                   // Network SSID (name)
char PASS[20]; // = SECRET_PASS;             // Network password (use for WPA, or use as key for WEP)
const char DEVICE_KEY[] = SECRET_DEVICE_KEY; // Secret device password

int cntForCloud;

void initProperties()
{
  char Buf[20];
  String SSIDasString;
  SSIDasString = WiFi.SSID();
  SSIDasString.toCharArray(Buf, 20);
  strcpy(SSID, Buf);

  String PASSWORDasString;
  PASSWORDasString = WiFi.psk();
  PASSWORDasString.toCharArray(Buf, 20);
  strcpy(PASS, Buf);

  // SSID = WiFi.SSID();

  // SSID = SSIDasString //.toCharArray(Buf, 50);

  ArduinoCloud.setBoardId(DEVICE_LOGIN_NAME);
  ArduinoCloud.setSecretDeviceKey(DEVICE_KEY);
  ArduinoCloud.addProperty(cntForCloud, READ, 5 * SECONDS, NULL);
}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);
