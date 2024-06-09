// sim7600G + ESP32

// IMEI: 868822046804710
// Barra Ruta 68

/*
CSQ
Value RSSI dBm  Condition
2 -109  Marginal
3 -107  Marginal
4 -105  Marginal
5 -103  Marginal
6 -101  Marginal
7 -99 Marginal
8 -97 Marginal
9 -95 Marginal
10  -93 OK
11  -91 OK
12  -89 OK
13  -87 OK
14  -85 OK
15  -83 Good
16  -81 Good
17  -79 Good
18  -77 Good
19  -75 Good
20  -73 Excellent
21  -71 Excellent
22  -69 Excellent
23  -67 Excellent
24  -65 Excellent
25  -63 Excellent
26  -61 Excellent
27  -59 Excellent
28  -57 Excellent
29  -55 Excellent
30  -53 Excellent


RSSI = (56*(CSQ - 2) - 3052)/28

 */
 
#include <ArduinoJson.h>
#include "time.h"
#include <esp_task_wdt.h>
#include "alarma.h"
#include "ZMPT101B.h"

// Scaffolding for Web Services
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
// Allow OTA software updates
#include <AsyncElegantOTA.h>

// Use local Wi-Fi
#include <WiFi.h>

#define TINY_GSM_MODEM_SIM7600

#define GUARDTIME  15000

#define ALARM      13
#define RELAY      14         // Relay Baliza
#define GREENLED   32
#define REDLED     33
#define BATT       35
#define VAC        34
#define SMAG       5

#define FIXEDSIZE  48
#define DOCSIZE    192
#define SENDINTERVAL  60000*15  // envío KeepAlive c/ 15 min
#define POWERREAD  15000    //Lectura energía cada 15 seg
#define DETECTSPAN  5000
#define REFVALADC  3170
#define MQTTLOOP   5000

#define WDT_TIMEOUT 60     // set watchdog a 60 seg

#define servername    "copecCarDefense" //Define the name to your server...
#define ID         50       // Identificador único para cada nodo

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
#define SerialAT Serial2

// See all AT commands, if wanted
//#define DUMP_AT_COMMANDS

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

byte numAlarma;

char buffer[256];

// Your GPRS credentials, if any
//const char apn[]      = "m2m.movistar.cl";
//const char gprsUser[] = "";
//const char gprsPass[] = "";
const char apn[]      = "m2m.entel.cl";
const char gprsUser[] = "entelpcs";
const char gprsPass[] = "entelpcs";


// MQTT details
const char* broker = "mqtt.tago.io";
const char* mqtt_pass = "c9e52dc2-614d-4d6b-9520-4946f81ff2b5";
const char* mqtt_user = "Token"; 
const char* topicAlarmas = "Copec/alarmas";
const char* topicKeepAlive = "Copec/keepalive";
const char* topicAvisos = "Copec/avisos";
String cmd = "Copec/cmd";
const char* topicCmd = cmd.c_str();    

#include <TinyGsmClient.h>
#include <PubSubClient.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif

TinyGsmClient client(modem);
PubSubClient  mqtt(client);

#define UART_BAUD   115200

#define MODEM_TX      17
#define MODEM_RX      16
#define MODEM_PWRKEY  4
#define MODEM_SLEEP   

unsigned long timerAL;
unsigned long timer;
unsigned long powTimer;
unsigned long alarmTimer;

boolean reboot;
bool netOK;
bool gprsOK;
bool rtcOK;

uint32_t lastReconnectAttempt = 0;

Alarma curAlarm;
bool alarmActive;

const char* ntpServer = "pool.ntp.org";
const int TZ = -16;    // Timezone Santiago, Chile

// Standard web server, on port 80. Must be global. Obvs.
AsyncWebServer server(80); 

// Forward declaration: convert Wi-Fi connection response to meaningful message
const char *wl_status_to_string(wl_status_t status);

const String Apname = "CarDefenseCopec ID" + String(ID);
const String Appass = "12345678";

const String OTAuser = "CarDefenseCopec";
const String OTApass = "Tank2024";

const String clientID = "CopecID" + String(ID);

String   webpage, MessageLine;
String    Version = "1.1";   // Programme version, see change log at end

float latitude;
float longitude;

const float vLimit = 150.0;
const float refVoltage = 13.6;

enum power
{
  AC,
  DC
};

enum baliza
{
  DES,
  ACT
};

enum lock
{
  CLOSE,
  OPEN
};

power pwr;
baliza bal;
lock mag;

int rssi;
bool acOk;

ZMPT101B voltageSensor(VAC, 50.0);

float battLevel;
float vrms;

void setup()
{
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  
  pinMode(RELAY, OUTPUT);          // Relay ON/OFF baliza
  pinMode(ALARM, INPUT_PULLUP);
  pinMode(SMAG, INPUT_PULLUP);     // Sensor magnético apertura gabinete

  pinMode(GREENLED, OUTPUT);
  pinMode(REDLED, OUTPUT);

  digitalWrite(RELAY, LOW);

  modemPowerOn();

  SerialMon.println("Wait...");

  Serial.println("SIM7600 serial initialize");
  delay(1000);

    // Restart takes quite some time
  // To skip it, call init() instead of restart()
  DBG("Initializing modem...");
  while (!modem.init()) {
      DBG("Failed to restart modem, delaying 10s and retrying");
      delay(10000);
  }

  /*  Preferred mode selection : AT+CNMP
        2 – Automatic
        13 – GSM Only
        14 – WCDMA Only
        38 – LTE Only
        59 – TDS-CDMA Only
        9 – CDMA Only
        10 – EVDO Only
        19 – GSM+WCDMA Only
        22 – CDMA+EVDO Only
        48 – Any but LTE
        60 – GSM+TDSCDMA Only
        63 – GSM+WCDMA+TDSCDMA Only
        67 – CDMA+EVDO+GSM+WCDMA+TDSCDMA Only
        39 – GSM+WCDMA+LTE Only
        51 – GSM+LTE Only
        54 – WCDMA+LTE Only
  */
  String ret;
  ret = modem.setNetworkMode(2);
  DBG("setNetworkMode:", ret);

  String name = modem.getModemName();
  DBG("Modem Name:", name);

  String modemInfo = modem.getModemInfo();
  DBG("Modem Info:", modemInfo);

  netOK = false;
  rtcOK = false;

  SerialMon.print("Waiting for network...");
  if(!modem.waitForNetwork()) {
      SerialMon.println(" fail");
  }

  else
    SerialMon.println(" success");

  if (modem.isNetworkConnected()) {
      SerialMon.println("Network connected");
      netOK = true;
  }

  if(netOK)
  {
    // GPRS connection parameters are usually set after network registration
    SerialMon.print(F("Connecting to "));
    SerialMon.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        SerialMon.println(" fail");
    }
    SerialMon.println(" success");
  
    if (modem.isGprsConnected()) {
        SerialMon.println("GPRS connected");
        initRTC();
        rtcOK = true;
    }
  }

  // MQTT Broker setup
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);

  clearCurAlarm();
  
  enableGPS();

  initWiFi();

   /*********  Server Commands  **********/
  // ##################### HOMEPAGE HANDLER ###########################
  //server.on("/",         SD_dir);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    Serial.println("Home Page...");
    Home(); // Build webpage ready for display
    request->send(200, "text/html", webpage);
  });

  server.onNotFound(notFound);

  // Starting Async OTA web server AFTER all the server.on requests registered
  AsyncElegantOTA.begin(&server,OTAuser.c_str(),OTApass.c_str());
  
  server.begin();
 
  pwr = AC;
  bal = ACT;
  mag = CLOSE;

  acOk = true;
  powerLed(acOk);

  //Check power AC
  vrms = voltageSensor.getRmsVoltage();
  if(vrms <= vLimit)
    acOk = false;
  
  powTimer = millis();
  timerAL = millis();
  timer = millis();

  checkBattery();
  sendKeepAlive();  // Para chequear conectividad con broker MQTT

  alarmActive = false;
  
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
}

void loop()
{
  if(checkAlarms())
  {
    sendAlarm();  
  }
      
  else if((millis() - timer) > SENDINTERVAL)
  {
    checkBattery();
    sendKeepAlive();
    timer = millis();
  }

  else if((millis() - powTimer) > POWERREAD)
  {
    //checkBattery();
    checkLock();
    checkPower();
    powTimer = millis();
  }
  
  esp_task_wdt_reset();    // Reset watchdog
}


void modemPowerOn()
{
  pinMode(MODEM_PWRKEY, OUTPUT);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(1000);
  digitalWrite(MODEM_PWRKEY, LOW);
}

bool checkAlarms()
{
  bool stat = false;
  if(digitalRead(ALARM) == LOW)
  {
    delay(50);
    if(digitalRead(ALARM) == LOW)
    {
      if((millis() - timerAL) > GUARDTIME)    // tiempo de guarda de 15 s entre detecciones
      {
        if(!alarmActive)
        {
          alarmTimer = millis();
          alarmActive = true;
        }
        
        else
        {
          if((millis() - alarmTimer) > DETECTSPAN)    //Alarma sigue activa luego de 5 seg?
          { 
              Serial.println("*****************Nueva alarma detectada*******************");
              Serial.print(getCurrentDateTime());
              Serial.print(" ");
              Serial.println("Alarma 2");
              curAlarm.numAlarma = 2;
              curAlarm.timestamp = getEpochTime();
              curAlarm.sendStatus = false; 
              timerAL = millis();
              stat = true;
              alarmActive = false;
          }
        }
      }

      if(bal == ACT)
        digitalWrite(RELAY, HIGH);
    }    
  }

  else
  {
    digitalWrite(RELAY, LOW);
    alarmActive = false;
  }
    
  return stat;
}

void initRTC()
{
  int   year3    = 0;
  int   month3   = 0;
  int   day3     = 0;
  int   hour3    = 0;
  int   min3     = 0;
  int   sec3     = 0;
  float timezone = 0;
  
  DBG("Asking modem to sync with NTP");
  modem.NTPServerSync(ntpServer, TZ);

  for (int8_t i = 5; i; i--) 
  {
    DBG("Requesting current network time");
    if (modem.getNetworkTime(&year3, &month3, &day3, &hour3, &min3, &sec3,&timezone)) 
    {
      DBG("Year:", year3, "\tMonth:", month3, "\tDay:", day3);
      DBG("Hour:", hour3, "\tMinute:", min3, "\tSecond:", sec3);
      DBG("Timezone:", timezone);
      break;
    } 
    else 
    {
      DBG("Couldn't get network time, retrying in 15s.");
      delay(15000L);
    }
  }
  
  DBG("Retrieving time again as a string");
  String timeStr = modem.getGSMDateTime(DATE_FULL);
  DBG("Current Network Time:", timeStr);  

  struct tm time = {0};
  time_t epoch = 0;
  int hh, mm, ss, yy, mon, day, tz;
  sscanf(timeStr.c_str(), "%d/%d/%d,%d:%d:%d-%d", &yy, &mon, &day, &hh, &mm, &ss, &tz);

  time.tm_hour = hh;
  time.tm_min = mm;
  time.tm_sec = ss;
  time.tm_year = 2000 + yy - 1900;
  
  time.tm_mon = mon - 1;
  time.tm_mday = day;
  time.tm_isdst = -1;

  epoch = mktime(&time);
  struct timeval tv;
  tv.tv_sec = epoch+1;
  tv.tv_usec = 0;
  
  settimeofday(&tv, NULL);

  setenv("TZ", "CLT,M4.1.0/0,M9.1.0/0", 1);
  tzset();
  printLocalTime();
}

void enableGPS(void)
{
    // Set SIM7000G GPIO4 LOW ,turn on GPS power
    // CMD:AT+SGPIO=0,4,1,1
    // Only in version 20200415 is there a function to control GPS power
    modem.sendAT("+SGPIO=0,4,1,1");
    if (modem.waitResponse(10000L) != 1) 
    {
        DBG(" SGPIO=0,4,1,1 false ");
    }
    modem.enableGPS();
}

void disableGPS(void)
{
    // Set SIM7000G GPIO4 LOW ,turn off GPS power
    // CMD:AT+SGPIO=0,4,1,0
    // Only in version 20200415 is there a function to control GPS power
    modem.sendAT("+SGPIO=0,4,1,0");
    if (modem.waitResponse(10000L) != 1) {
        DBG(" SGPIO=0,4,1,0 false ");
    }
    modem.disableGPS();
}

void printLocalTime()
{
  String cDT = getCurrentDateTime();
  Serial.println(cDT); 
}

unsigned long getEpochTime() 
{
  time_t now;
  struct tm timeinfo;
  for(int i = 0; i < 10; i++)
  {
    if(getLocalTime(&timeinfo))
      break;
    delay(100);
  }
  time(&now);
  return now-3600*(TZ/4);
}

bool getGPS()
{
  bool gpsOk;
  gpsOk = modem.getGPS(&latitude, &longitude);
  if(gpsOk)
  {

    Serial.print("latitude:"); Serial.print(latitude);
    Serial.print(", longitude:"); Serial.println(longitude);
      
  }
  else
  {
    Serial.println("Error getting GPS data");
    latitude = 0.0;
    longitude = 0.0;
  }
  
  return gpsOk;
}

String getCurrentDateTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return String("");
  }

  String curDay;
  byte d = timeinfo.tm_mday;
  if(d < 10)
    curDay = "0" + String(d);
  else
    curDay = String(d);
    
  String curMonth;

  byte m = timeinfo.tm_mon + 1;

  if(m < 10)
    curMonth = "0" + String(m);
  else
    curMonth = String(m);     
  
  String curYear = String(timeinfo.tm_year +1900);
  
  String curHour;

  byte hh = timeinfo.tm_hour;
 
  if(hh < 10)
    curHour = "0" + String(hh);
  else
    curHour = String(hh); 

  String curMin;
  
  byte mm = timeinfo.tm_min;
 
  if(mm < 10)
    curMin = "0" + String(mm);
  else
    curMin = String(mm); 

  String curSec;
   
  byte ss = timeinfo.tm_sec;
 
  if(ss < 10)
    curSec = "0" + String(ss);
  else
    curSec = String(ss);
       
  String curDateTime = curDay + "/" + curMonth + "/" + curYear + " " + curHour + ":" + curMin + ":" + curSec;
  return curDateTime; 
}

void checkNetwork()
{
    // Make sure we're still registered on the network
  if (!modem.isNetworkConnected()) 
  {
      SerialMon.println("Network disconnected");
      if (!modem.waitForNetwork(180000L, true)) 
      {
          SerialMon.println(" fail");
          delay(10000);
          return;
      }
      if (modem.isNetworkConnected()) 
      {
          SerialMon.println("Network re-connected");
      }
      // and make sure GPRS/EPS is still connected
      if (!modem.isGprsConnected()) 
      {
          SerialMon.println("GPRS disconnected!");
          SerialMon.print(F("Connecting to "));
          SerialMon.print(apn);
          if (!modem.gprsConnect(apn, gprsUser, gprsPass)) 
          {
              SerialMon.println(" fail");
              delay(10000);
              return;
          }
          if (modem.isGprsConnected()) 
          {
              SerialMon.println("GPRS reconnected");
              if(!rtcOK)
              {
                initRTC();
                rtcOK = true;
              }
                 
          }
      }
  }
}

void initWiFi()
{
  WiFi.softAP(Apname.c_str(), Appass.c_str()); //Network and password for the access point genereted by ESP32
  
  //Set your preferred server name, if you use "cardefense" the address would be http://cardefensepilot.local/
  if (!MDNS.begin(servername)) 
  {          
    Serial.println(F("Error setting up MDNS responder!")); 
    ESP.restart(); 
  } 

  IPAddress Ap_Ip = WiFi.softAPIP();
  Serial.print("Access Point IP address: ");
  Serial.println(Ap_Ip);
}

// Translates the Wi-Fi connect response to English
const char *wl_status_to_string(wl_status_t status)
{
  switch (status) {
    case WL_NO_SHIELD:
      return "WL_NO_SHIELD";
    case WL_IDLE_STATUS:
      return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL:
      return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED:
      return "WL_SCAN_COMPLETED";
    case WL_CONNECTED:
      return "WL_CONNECTED";
    case WL_CONNECT_FAILED:
      return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST:
      return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED:
      return "WL_DISCONNECTED";
    default:
      return "UNKNOWN";
  }
}

void Home() {
  webpage = HTML_Header();
  webpage += "<h1>Home Page</h1>";
  webpage += "<h2>ESP Asychronous WebServer Example</h2>";
  webpage += "<img src = 'icon' alt='icon'>";
  webpage += "<h3>File Management - Firmware Update</h3>";
  webpage += HTML_Footer();
}
//#############################################################################################
String HTML_Header() {
  String page;
  page  = "<!DOCTYPE html>";
  page += "<html lang = 'en'>";
  page += "<head>";
  page += "<title>Web Server</title>";
  page += "<meta charset='UTF-8'>"; // Needed if you want to display special characters like the ° symbol
  page += "<style>";
  page += "body {width:75em;margin-left:auto;margin-right:auto;font-family:Arial,Helvetica,sans-serif;font-size:16px;color:blue;background-color:#e1e1ff;text-align:center;}";
  page += "footer {padding:0.08em;background-color:cyan;font-size:1.1em;}";
  page += "table {font-family:arial,sans-serif;border-collapse:collapse;width:70%;}"; // 70% of 75em!
  page += "table.center {margin-left:auto;margin-right:auto;}";
  page += "td, th {border:1px solid #dddddd;text-align:left;padding:8px;}";
  page += "tr:nth-child(even) {background-color:#dddddd;}";
  page += "h4 {color:slateblue;font:0.8em;text-align:left;font-style:oblique;text-align:center;}";
  page += ".center {margin-left:auto;margin-right:auto;}";
  page += ".topnav {overflow: hidden;background-color:lightcyan;}";
  page += ".topnav a {float:left;color:blue;text-align:center;padding:0.6em 0.6em;text-decoration:none;font-size:1.3em;}";
  page += ".topnav a:hover {background-color:deepskyblue;color:white;}";
  page += ".topnav a.active {background-color:lightblue;color:blue;}";
  page += ".notfound {padding:0.8em;text-align:center;font-size:1.5em;}";
  page += ".left {text-align:left;}";
  page += ".medium {font-size:1.4em;padding:0;margin:0}";
  page += ".ps {font-size:0.7em;padding:0;margin:0}";
  page += ".sp {background-color:silver;white-space:nowrap;width:2%;}";
  page += "</style>";
  page += "</head>";
  page += "<body>";
  page += "<div class = 'topnav'>";
  page += "<a href='/'>Home</a>";
  page += "<a href='/update'>Update Firmware</a>";
  //page += "<a href='/logout'>[Log-out]</a>";
  page += "</div>";
  return page;
}
//#############################################################################################
String HTML_Footer() {
  String page;
  page += "<br><br><footer>";
  page += "<p class='medium'>ESP Asynchronous WebServer Example</p>";
  page += "<p class='ps'><i>Copyright &copy;&nbsp;Tank SpA 2023 Version " +  Version + "</i></p>";
  page += "</footer>";
  page += "</body>";
  page += "</html>";
  return page;
}

//#############################################################################################
void Page_Not_Found() {
  webpage = HTML_Header();
  webpage += "<div class='notfound'>";
  webpage += "<h1>Sorry</h1>";
  webpage += "<p>Error 404 - Page Not Found</p>";
  webpage += "</div><div class='left'>";
  webpage += "<p>The page you were looking for was not found, it may have been moved or is currently unavailable.</p>";
  webpage += "<p>Please check the address is spelt correctly and try again.</p>";
  webpage += "<p>Or click <b><a href='/'>[Here]</a></b> for the home page.</p></div>";
  webpage += HTML_Footer();
}
//#############################################################################################
// Not found handler is also the handler for 'delete', 'download' and 'stream' functions
void notFound(AsyncWebServerRequest *request) { // Process selected file types
  Page_Not_Found();
  request->send(200, "text/html", webpage);
}

void sendAlarm()
{
  // Check conectividad a red celular y GPRS
  checkNetwork();
  // Conectar a broker MQTT
  connectMqtt();
  //getGPS();

  StaticJsonDocument<FIXEDSIZE> doc;

  doc["id"] = ID;
  doc["ts"] = curAlarm.timestamp;
  doc["nA"] = curAlarm.numAlarma;

  String buf;
  size_t n = serializeJson(doc, buf);

  Serial.print(n);
  Serial.print("/");
  Serial.println(buf);
  

  if (mqtt.publish(topicAlarmas, buf.c_str(),n)) 
  {
     Serial.println("Success sending message");
     clearCurAlarm();
  } 
  else 
  {
    
     Serial.println("Error sending message");
  }

  delay(500);
  disconnectMqtt();
}

void sendKeepAlive()
{
   // Check conectividad a red celular y GPRS
  checkNetwork();

  // Conectar a broker MQTT
  connectMqtt();

  getGPS();
  int csq = modem.getSignalQuality();
  rssi = (56*(csq - 2) - 3052)/28;

  String txt;
  if(bal == ACT)
     txt+="ON";
  else
     txt+="OFF";

  String txt2;
  if(acOk)
     txt2+="ON";
  else
     txt2+="OFF";

  StaticJsonDocument<DOCSIZE> doc;

  doc["id"] = ID;
  doc["ts"] = getEpochTime();
  doc["bal"] = txt;
  doc["powerAc"] = txt2;
  doc["rssi"] = rssi;
  doc["batt"] = battLevel;
  
  JsonObject GPS = doc.createNestedObject("gps");
  GPS["lat"] = latitude;
  GPS["lon"] = longitude;
  

  String buf;
  size_t n = serializeJson(doc, buf);

  Serial.print(n);
  Serial.print("/");
  Serial.println(buf);
  

  if (mqtt.publish(topicKeepAlive, buf.c_str(),n)) 
  {
     Serial.println("Success sending message");
  } 
  else 
  {
    
     Serial.println("Error sending message");
  }

  checkMqttLoop(); 
  
  disconnectMqtt();
}

void clearCurAlarm()
{
    //Borrar current alarm
    curAlarm.numAlarma = 0;
    curAlarm.timestamp = 0;
    curAlarm.sendStatus = true;
}

void powerLed(bool ok)
{
  if(ok)
  {
     digitalWrite(GREENLED, LOW);
     digitalWrite(REDLED, HIGH);
  }
  else
  {
     digitalWrite(GREENLED, HIGH);
     digitalWrite(REDLED, LOW);
  }
}

void checkPower()
{
  // Revisar status de alimentación principal (AC) con sensor de corriente
  vrms = voltageSensor.getRmsVoltage();
//  Serial.print("Power: ");
//  Serial.println(vrms);
  
  if(vrms <= vLimit)
  {
    if(pwr == AC)
    {
       //Enviar alarma de corte de energía 
       unsigned long dt = getEpochTime();  
       Serial.println("Corte Energia");     
       checkNetwork();
       connectMqtt();
       StaticJsonDocument <64> doc;
       doc["Id"] = ID;
       doc["Datetime"] = dt;
       doc["Tipo"] = "Corte Energia";
       char txt[100];
       size_t n = serializeJson(doc, txt);
       mqtt.publish(topicAvisos, txt, n);
       acOk = false;
       pwr = DC;
       delay(500);
       disconnectMqtt();
    }
  }
  else
  {
    if(pwr == DC)
    {
      //Enviar aviso de restitucion de energía
      unsigned long dt = getEpochTime();
      Serial.println("Restitucion Energia");
      checkNetwork();
      connectMqtt();
      StaticJsonDocument <64> doc;
      doc["Id"] = ID;
      doc["Datetime"] = dt;
      doc["Tipo"] = "Restitucion Energia";
      char txt[100];
      size_t n = serializeJson(doc, txt);
      mqtt.publish(topicAvisos, txt, n);     
      pwr = AC;
      acOk = true;
      delay(500);
      disconnectMqtt();
    }
  }

  powerLed(acOk);
}

void checkLock()
{
  // Revisar apertura de gabinete
  if(digitalRead(SMAG) == HIGH)
  {
    delay(150);
    if(digitalRead(SMAG) == HIGH)
    {
      if(mag == CLOSE)
      {
        mag = OPEN;
        unsigned long dt = getEpochTime();
        Serial.println("Apertura Gabinete");
        //Enviar mensaje de alerta de apertura de gabinete !!!
        checkNetwork();
        connectMqtt();
        StaticJsonDocument <64> doc;
        doc["Id"] = ID;
        doc["Datetime"] = dt;
        doc["Tipo"] = "Apertura Gabinete";
        char txt[100];
        size_t n = serializeJson(doc, txt);
        mqtt.publish(topicAvisos, txt, n);
        delay(500);
        disconnectMqtt();
      }
    }
  }
  else
  {
    if(mag == OPEN)
    {
       //Enviar aviso de cierre de gabinete !!!
      unsigned long dt = getEpochTime();
      Serial.println("Cierre Gabinete");
      checkNetwork();
      connectMqtt();
      StaticJsonDocument <64> doc;
      doc["Id"] = ID;
      doc["Datetime"] = dt;
      doc["Tipo"] = "Cierre Gabinete";
      char txt[100];
      size_t n = serializeJson(doc, txt);
      mqtt.publish(topicAvisos, txt, n);
      delay(500);
      disconnectMqtt();
    }
    
    mag = CLOSE;
  }
}

void checkBattery()
{
  int suma = 0;
  int val = 0;
  
  for(int i=0; i<10; i++)
  {
    suma += analogRead(BATT);
  }

  val = suma/10;

  // Convertir de valor 0-4095 a volts 
  
  battLevel = float(val) * refVoltage/float(REFVALADC);
//  Serial.println(battLevel);
//  Serial.println(val);
}

void connectMqtt()
{
  // Conectar a broker MQTT
  while (!mqtt.connected()) 
  {
    Serial.println("Connecting to MQTT...");
 
    if (mqtt.connect(clientID.c_str(), mqtt_user, mqtt_pass)) 
    {
      Serial.println("connected");
      mqtt.subscribe(topicCmd);
    } 
    else 
    {
      Serial.print("failed with state ");
      Serial.print(mqtt.state());
      delay(2000);
    }
  }
}

void disconnectMqtt()
{
  mqtt.disconnect();

  Serial.println("Disconnecting from MQTT broker");
  Serial.println("******************************************");  
}

void checkMqttLoop()
{
  unsigned long t = millis();
  Serial.println("Checking mqtt loop");
  while((millis() - t) < MQTTLOOP)
  {
    mqtt.loop();
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int len)
{
  Serial.println("******************************");
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  analyzeCmd(payload,len);
}

void analyzeCmd(byte* payload, unsigned int len)
{
  SerialMon.write(payload, len);
  SerialMon.println();
  DynamicJsonDocument doc(64);
  deserializeJson(doc, payload);

  String id = doc["id"];
  String val = doc["baliza"];

  //Serial.println(val);  

  if(id.toInt() == ID)
  {
    if(val != "null")
    {
      if(val == "ON")
      {
        Serial.println("Activando baliza");
        bal = ACT;
      }
      else if(val == "OFF")
      {
        Serial.println("Desactivando baliza");
        bal = DES;
      }
  
      return;
    }
  }
}
