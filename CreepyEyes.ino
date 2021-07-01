#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <Time.h>
#include <TimeLib.h>
#include <TimeAlarms.h>
#include <Timezone.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266httpUpdate.h>
#include <WiFiUdp.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include <stdint.h>
#include "Config.h"
#include "Formatting.h"
#include "NeoPatterns.h"

void mqttPublish(const char* status);
void mqttCallback(char* topic, byte* payload, unsigned int length);
void BigEyes1Complete();
void BigEyes2Complete();
void SmallEyes1Complete();
void SmallEyes2Complete();
void SmallEyes3Complete();


// Define some NeoPatterns for the two rings and the stick
//  as well as some completion routines
NeoPatterns BigEyes1(24, D2, NEO_GRB + NEO_KHZ800, &BigEyes1Complete);
NeoPatterns BigEyes2(24, D3, NEO_GRB + NEO_KHZ800, &BigEyes2Complete);
NeoPatterns SmallEyes1(16, D5, NEO_GRB + NEO_KHZ800, &SmallEyes1Complete);
NeoPatterns SmallEyes2(16, D6, NEO_GRB + NEO_KHZ800, &SmallEyes2Complete);
NeoPatterns SmallEyes3(16, D7, NEO_GRB + NEO_KHZ800, &SmallEyes3Complete);


TimeChangeRule myDST = {"PDT", Second, Sun, Mar, 2, -420};    //Daylight time = UTC - 7 hours
TimeChangeRule mySTD = {"PST", First, Sun, Nov, 2, -480};     //Standard time = UTC - 8 hours
Timezone myTZ(myDST, mySTD);
void setupAlarms();
ESP8266WiFiMulti wifiMulti;

long lastTime = 0;
long lastTimeClock = 0;
time_t utc, local;

int morningIndex = -1;
int eveningIndex = -1;

bool connectedOnce = false;

DynamicJsonDocument mqttDoc(1024);
WiFiClient mqttWiFiClient;
String mqttClientId; 
long lastReconnectAttempt = 0; 

PubSubClient mqttClient(MQTT_SERVER, MQTT_PORT, mqttCallback, mqttWiFiClient);

// NTP 
WiFiUDP Udp;

byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets
int ntpRetryCount = 3;
int ntpRetry = 0;


TimeChangeRule *tcr;        //pointer to the time change rule, use to get TZ abbrev
void MorningAlarm() {
  Serial.println("Good morning!");
  time_t utc = now();
  time_t local = myTZ.toLocal(utc, &tcr);
  printTime(local, tcr -> abbrev);
  StopAllEyes();
}

void EveningAlarm() {
  Serial.println("Good evening!");
  time_t utc = now();
  time_t local = myTZ.toLocal(utc, &tcr);
  printTime(local, tcr -> abbrev);
  InitiateAllEyes();
}

void StopAllEyes(){
  // kill the running patterns
  BigEyes1.Stop();
  BigEyes1.clear();
  BigEyes1.show();

  BigEyes2.Stop();
  BigEyes2.clear();
  BigEyes2.show();

  SmallEyes1.Stop();
  SmallEyes1.clear();
  SmallEyes1.show();

  SmallEyes2.Stop();
  SmallEyes2.clear();
  SmallEyes2.show();

  SmallEyes3.Stop();
  SmallEyes3.clear();
  SmallEyes3.show();
}

void InitiateAllEyes() {
  // Kick off a pattern
  BigEyes1.Fade(BigEyes1.Color(0,0,0), BigEyes1.ColorHSV(random(65535)), 1000, 10 + random(10));
  BigEyes2.Fade(BigEyes2.Color(0,0,0), BigEyes2.ColorHSV(random(65535)), 1000, 10 + random(10));
  SmallEyes1.Fade(SmallEyes1.Color(0,0,0), SmallEyes1.ColorHSV(random(65535)), 1000, 10 + random(10));
  SmallEyes2.Fade(SmallEyes2.Color(0,0,0), SmallEyes2.ColorHSV(random(65535)), 1000, 10 + random(10));
  SmallEyes3.Fade(SmallEyes3.Color(0,0,0), SmallEyes3.ColorHSV(random(65535)), 1000, 10 + random(10));
}

int createAlarmUTC(int h, int m, OnTick_t onTickHandler) {
  TimeElements t;
  t.Second = 0;
  t.Minute = m;
  t.Hour = h;

  // Unused
  t.Day = 18;
  t.Month = 11;
  t.Year = year(now()) - 1970;

  time_t utc = makeTime(t);
  mqttLog(("UTC Alarm set for " + String(h) + ":" + String(m)));
  return Alarm.alarmOnce(hour(utc), m, 0, onTickHandler);
}

int createAlarm(int h, int m, OnTick_t onTickHandler) {
  TimeElements t;
  t.Second = 0;
  t.Minute = m;
  t.Hour = h;
  time_t holding = now();
  t.Day = day(holding);
  t.Month = month(holding);
  t.Year = year(holding) - 1970;

  time_t localTime = makeTime(t);
  time_t utc = myTZ.toUTC(localTime);
  Serial.print("createAlarm: ");
  Serial.print(hour(utc));
  Serial.print(":");
  Serial.println(m);
  return Alarm.alarmOnce(hour(utc), m, 0, onTickHandler);
}

void setupAlarms() {
  mqttLog("setupAlarms - Enter");
  for (int i = 0; i < dtNBR_ALARMS; i++) {
    Alarm.free(i);
  }

  getSunriseSunsetTimes();
  mqttLog("setupAlarms - Exit");
}

void getSunriseSunsetTimes() {
  mqttLog("getSunriseSunsetTimes - Enter");
  WiFiClient client;
  const int httpPort = 80;
  const char* host = "api.sunrise-sunset.org";
  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    mqttLog("connection failed");
    return;
  }

  // We now create a URI for the request
  String url = "/json?lat=38.581572&lng=-121.494400&formatted=0";
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
      "Host: " + host + "\r\n" +
      "Connection: close\r\n\r\n");
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }

  while (client.available()) {
    String line = client.readStringUntil('\r');
    line = line.substring(1);
    if (line[0] == '{') {
      mqttLog("Received sunrise");
      StaticJsonDocument<1000> doc;
      DeserializationError err = deserializeJson(doc, line.substring(line.indexOf(':') + 1, line.lastIndexOf(',')));
      String sunrise = doc["sunrise"];
      String sunset = doc["sunset"];

      Serial.print("Sunrise response: ");
      Serial.println(sunrise);

      String sunriseHour = sunrise.substring(sunrise.indexOf('T') + 1, sunrise.indexOf(':'));
      String sunriseMin = sunrise.substring(sunrise.indexOf(':') + 1, sunrise.indexOf(':') + 3);
      String sunsetHour = sunset.substring(sunset.indexOf('T') + 1, sunset.indexOf(':'));
      String sunsetMin = sunset.substring(sunset.indexOf(':') + 1, sunset.indexOf(':') + 3);
      Serial.print("Sunrise UTC: ");
      Serial.println(sunriseHour + ":" + sunriseMin);
      Alarm.free(morningIndex);
      Alarm.free(eveningIndex);
      bool useSunrise;
      morningIndex = createAlarmUTC(sunriseHour.toInt(), sunriseMin.toInt(), MorningAlarm);
      Serial.println("Sunrise alarm:" + sunriseHour + ":" + sunriseMin);

      int sunsetMinInt = sunsetMin.toInt();
      int sunsetHourInt = sunsetHour.toInt();
      Serial.print("Sunset UTC: ");
      Serial.println(String(sunsetHourInt) + ":" + String(sunsetMinInt));
      eveningIndex = createAlarmUTC(sunsetHourInt, sunsetMinInt, EveningAlarm);
      // Convert UTC sunset to local
      TimeElements t;
      t.Second = 0;
      t.Minute = sunsetMinInt;
      t.Hour = sunsetHourInt;

      // Unused
      t.Day = 18;
      t.Month = 11;
      t.Year = year(now()) - 1970;

      time_t localSunset = myTZ.toLocal(makeTime(t), &tcr);

      t.Minute = sunriseMin.toInt();
      t.Hour = sunriseHour.toInt();
      time_t localSunrise = myTZ.toLocal(makeTime(t), &tcr);

      time_t utc = now();
      time_t local = myTZ.toLocal(utc, &tcr);
      Serial.print("Time: ");
      printTime(local, tcr -> abbrev);

      if ((hour(localSunset) > hour(local) && minute(localSunset) > minute(local)) || 
        (hour(localSunrise) < hour(local) && minute(localSunrise) < minute(local))) {
        Serial.println("Started up after sunset so starting");
        InitiateAllEyes();
      }

      break;
    }
  }

  Serial.println();
  Serial.println("closing connection");
  mqttLog("getSunriseSunsetTimes - Exit");
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  //Udp.beginMulticast(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

String IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") +\
    String(ipAddress[1]) + String(".") +\
    String(ipAddress[2]) + String(".") +\
    String(ipAddress[3])  ;
}

time_t getNtpTime()
{
  if (wifiMulti.run() == WL_CONNECTED) {
    while (Udp.parsePacket() > 0) ; // discard any previously received packets
    // Fall back to using the global NTP pool server in case we cannot connect to internal NTP server
    if (ntpRetry > 1)
      WiFi.hostByName(ntpServerName, timeServer);
    Serial.print("Transmit NTP Request to ");
    Serial.println(IpAddress2String(timeServer));
    sendNTPpacket(timeServer);
    uint32_t beginWait = millis();
    while (millis() - beginWait < 1500) {
      int size = Udp.parsePacket();
      if (size >= NTP_PACKET_SIZE) {
        Serial.println("Receive NTP Response");
        Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
        unsigned long secsSince1900;
        // convert four bytes starting at location 40 to a long integer
        secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
        secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
        secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
        secsSince1900 |= (unsigned long)packetBuffer[43];
        randomSeed(secsSince1900);
        // Make sure we chill a little
        setSyncInterval(86400);
        unsigned long calcTime = secsSince1900 - 2208988800UL;// + timeZone * SECS_PER_HOUR;
        Serial.print("ntp: ");
        Serial.println(calcTime);
        return calcTime;
      }
    }
  }
  Serial.println("No NTP Response :-(");
  if (ntpRetry < ntpRetryCount) 
  {
    ntpRetry++;
    return getNtpTime();
  }

  // We couldn't connect so we are gonna try harder!
  Serial.println("NTP n");
  setSyncInterval(5);
  return now(); // return now if unable to get the time so we just get our drifted internal time instead of wrong time.
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.printf("Inside mqtt callback: %s\n", topic);
  Serial.println(length);

  String topicString = (char*)topic;
  topicString = topicString.substring(topicString.lastIndexOf('/')+1);
  Serial.print("Topic: ");
  Serial.print(topicString);

  String action = (char*)payload;
  action = action.substring(0, length);
  Serial.println(action);

  if (action == "On") InitiateAllEyes();
  if (action == "Off") StopAllEyes();
  if (action == "Update") {
    WiFiClient updateWiFiClient;
    t_httpUpdate_return ret = ESPhttpUpdate.update(updateWiFiClient, UPDATE_URL);
    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("HTTP_UPDATE_NO_UPDATES");
        break;

      case HTTP_UPDATE_OK:
        Serial.println("HTTP_UPDATE_OK");
        break;
    }
  }
}

void mqttPublish(const char* status) {
  if (mqttClient.connected()) {
    mqttDoc["Status"] = status;
    time_t utc = now();
    time_t local = myTZ.toLocal(utc, &tcr);
    mqttDoc["Date"] = formatTime(local, tcr -> abbrev);
    char buffer[512];
    size_t n = serializeJson(mqttDoc, buffer);
    mqttClient.publish(MQTT_CHANNEL_PUB, buffer, true);
  }
}

void mqttLog(const char* status) {
  Serial.println(status);
  if (mqttClient.connected()) {
    mqttClient.publish(MQTT_CHANNEL_LOG, status, true);
  }
}

void mqttLog(String status) {
  char buf[256];
  status.toCharArray(buf, 256);
  mqttLog(buf);
}

boolean mqttReconnect() {
  char buf[100];
  mqttClientId.toCharArray(buf, 100);
  if (mqttClient.connect(buf, MQTT_USER, MQTT_PASSWORD)) {
    mqttClient.subscribe(MQTT_CHANNEL_SUB);
    Serial.println("MQTT connected");
  }

  return mqttClient.connected();
}

String generateMqttClientId() {
  char buffer[4];
  uint8_t macAddr[6];
  WiFi.macAddress(macAddr);
  sprintf(buffer, "%02x%02x", macAddr[4], macAddr[5]);
  return "CreepyEyes" + String(buffer);
}

void updateStarted() {
  Serial.println("CALLBACK:  HTTP update process started");
}

void updateFinished() {
  Serial.println("CALLBACK:  HTTP update process finished");
}

void updateProgress(int cur, int total) {
  Serial.printf("CALLBACK:  HTTP update process at %d of %d bytes...\n", cur, total);
}

void updateError(int err) {
  Serial.printf("CALLBACK:  HTTP update fatal error code %d\n", err);
}

void onConnect() {
  Serial.print("Connected: ");
  Serial.println(WiFi.localIP());
  connectedOnce = true;
  Udp.begin(LOCALUDPPORT);
  setSyncProvider(getNtpTime);
}

void setupWiFi(){
#ifdef DNSNAME
  WiFi.hostname(DNSNAME);
#else
  char buffer[4];
  uint8_t macAddr[6];
  WiFi.macAddress(macAddr);
  sprintf(buffer, "%02x%02x", macAddr[4], macAddr[5]);
  WiFi.hostname("CreepyEyes" + String(buffer));
#endif

  for (int i=0; i < wifiCount; i++) {
    wifiMulti.addAP(ssids[i], passs[i]);
  }
  Serial.println("Connecting");
  if (wifiMulti.run() == WL_CONNECTED) {
    onConnect();
  }
}

bool validateWiFi(long milliseconds) {
  // Update WiFi status. Take care of rollover
  if (milliseconds >= lastTimeClock + 1000 || milliseconds < lastTimeClock) {
    if (wifiMulti.run() != WL_CONNECTED) {
      Serial.println("Disconnected");
      connectedOnce = false;
      return false;
    } else {
      if (!connectedOnce) {
        Serial.print("Connected late to ");
        Serial.println(WiFi.SSID());
        onConnect();
      }

      connectedOnce = true;
    }
  }

  return connectedOnce;
}

void validateMqtt(long milliseconds) {
  if (!mqttClient.connected()) {
    if (milliseconds - lastReconnectAttempt > 5000 || lastReconnectAttempt == 0 || milliseconds < lastReconnectAttempt) {
      Serial.println("MQTT not connected");
      lastReconnectAttempt = milliseconds;
      Serial.println("MQTT reconnecting");
      // Attempt to reconnect
      if (mqttReconnect()) {
        Serial.println("MQTT reconnected");
      }
    }

    if (milliseconds - lastReconnectAttempt > 60000) {
      Serial.println("MQTT disconnecting WiFi");
      WiFi.disconnect();
      delay(500);
    }
  } else {
    mqttClient.loop();
  }
}

//------------------------------------------------------------
//Completion Routines - get called on completion of a pattern
//------------------------------------------------------------

void Blink(NeoPatterns *eye)
{
  eye->clear();
  eye->show();
  delay(100);
  eye->Fade(eye->Color2, eye->ColorHSV(random(65535)), 1000, 5 + random(10));
}

// BigEyes Completion Callback
void BigEyes1Complete()
{
  Serial.println("blink big 1");
  Blink(&BigEyes1);
}

void BigEyes2Complete()
{
  Serial.println("blink big 2");
  Blink(&BigEyes2);
}

void SmallEyes1Complete()
{
  Serial.println("blink small 1");
  Blink(&SmallEyes1);
}

void SmallEyes2Complete()
{
  Serial.println("blink small 2");
  Blink(&SmallEyes2);
}

void SmallEyes3Complete()
{
  Serial.println("blink small 3");
  Blink(&SmallEyes3);
}
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println();

  setupWiFi();

  mqttClientId = generateMqttClientId();

  ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW);
  ESPhttpUpdate.onStart(updateStarted);
  ESPhttpUpdate.onEnd(updateFinished);
  ESPhttpUpdate.onProgress(updateProgress);
  ESPhttpUpdate.onError(updateError);

  // Initialize all the pixelStrips
  BigEyes1.begin();
  BigEyes2.begin();
  SmallEyes1.begin();
  SmallEyes2.begin();
  SmallEyes3.begin();

}

void loop() {
  long milliseconds = millis();

  Alarm.delay(0);

  // Check if connected then handle the connected magic
  if (validateWiFi(milliseconds)) {
    validateMqtt(milliseconds);

    // Check the time. Set alarms. Take care of rollover
    if (milliseconds >= lastTime + 7200000 || milliseconds < lastTime || lastTime == 0 ) {
      lastTime = milliseconds;
      setupAlarms();
    }

  }
  if (milliseconds >= lastTimeClock + 1000 || milliseconds < lastTimeClock) {
    lastTimeClock = milliseconds;
  }

  // Convert to local time what was gathered above
  utc = now();
  local = myTZ.toLocal(utc, &tcr);

  BigEyes1.Update();
  BigEyes2.Update();
  SmallEyes1.Update();
  SmallEyes2.Update();
  SmallEyes3.Update();
}
