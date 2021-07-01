#ifndef _CONFIG
#define _CONFIG

#define DEBOUNCE 50
#define LOCALUDPPORT 8888
#define DNSNAME "CreepyEyes"
#define MQTT_SERVER "pi4"
#define MQTT_PORT 1883
#define MQTT_CHANNEL_PUB "home/" DNSNAME "/state"
#define MQTT_CHANNEL_SUB "home/" DNSNAME "/control"
#define MQTT_CHANNEL_LOG "home/" DNSNAME "/log"
#define MQTT_USER "clockuser"
#define MQTT_PASSWORD "clockuser"
#define UPDATE_URL "http://pi4/cgi-bin/test.rb"

IPAddress timeServer(224, 0, 1, 1);
const char* ntpServerName = "us.pool.ntp.org";

const char* ssids[] = {"WiFi", "Info"};
const char* passs[] = {"Goes", "Here"};
const int wifiCount = 2;
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message

#endif
