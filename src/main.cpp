#define TINY_GSM_MODEM_BG96 // EC25

#include <Arduino.h>
// #include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <EEPROM.h>
// #include "BluetoothSerial.h"
#include "REG_SDM120.h"
#include <ModbusMaster.h>
#include <HardwareSerial.h>
#include <MAGELLAN_MQTT.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <SSLClient.h>
#include <HTTPClientESP32.h>
#include <ArduinoOTA.h>

#define trigWDTPin    32
#define ledHeartPIN   0
#define REGEN_BY_CONDITION false
#define UART_BAUD 115200
#define WDTPin 33 // Watch Dog pin for Trig

#define MODEM_TX 27
#define MODEM_RX 14
#define GSM_RESET 25

#define SerialMon Serial

const char serverOTA[] = "raw.githubusercontent.com";
const int port = 443;

HardwareSerial modbus(2);
HardwareSerial SerialAT(1);
// GSM Object
TinyGsm modem(SerialAT);

ModbusMaster node;
// StaticJsonDocument<400> doc;

// HTTPS Transport MQTT
TinyGsmClient gsm_mqtt_client(modem, 0);
// PubSubClient GSMmqtt(gsm_mqtt_client);

WiFiManager wifiManager;
WiFiClient WiFi_client;
MAGELLAN_MQTT magelWiFi(WiFi_client);
MAGELLAN_MQTT magelGSM(gsm_mqtt_client);

// HTTPS Transport OTA
TinyGsmClient base_client(modem, 1);
SSLClient secure_layer(&base_client);
HttpClient GSMclient = HttpClient(secure_layer, serverOTA, port);

String json, deviceToken;
unsigned long previousMillis = 0;
unsigned long periodOTA = 0;

boolean GSMnetwork = false;
boolean GSMgprs = false;
bool connectWifi = false;
bool resultSub = false;

// Your GPRS credentials
const char apn[] = "internet";
const char user[] = "";
const char pass[] = "";

String new_version;
const char version_url[] = "";//"/Vichayasan/BMA/refs/heads/main/TX/bin_version.txt"; // "/IndustrialArduino/OTA-on-ESP/release/version.txt";  https://raw.githubusercontent.com/:owner/:repo/master/:path
const char* version_url_WiFiOTA = "";//"https://raw.githubusercontent.com/Vichayasan/BMA/refs/heads/main/TX/bin_version.txt"; // "/IndustrialArduino/OTA-on-ESP/release/version.txt";  https://raw.githubusercontent.com/:owner/:repo/master/:path

String firmware_url;
String current_version = "0.0.1";

void t1CallgetMeter();
void t2CallsendViaNBIOT();
void t3CallHeartbeat();
void HeartBeat();
void readMeter();
 
struct Meter
{

  String vAB;
  String vBC;
  String vCA;
  String vAN;
  String vBN;
  String vCN;

 
  String freq;

 
  String vuAN;
  String vuBN;
  String vuCN;
  String vuLNW; // Voltage Unbalance L-N Worst


  ////////////////////////

  String Temp1;
  String Temp2;
  String Temp3;
  String Temp4;

  String sdmVolt;
  String sdmCurrent;
  String sdmPF; //Power factor

};
Meter meter;



void writeString(char add, String data)
{
  int _size = data.length();
  int i;
  for (i = 0; i < _size; i++)
  {
    EEPROM.write(add + i, data[i]);
  }
  EEPROM.write(add + _size, '\0'); //Add termination null character for String Data
  EEPROM.commit();
}


String read_String(char add)
{
  int i;
  char data[100]; //Max 100 Bytes
  int len = 0;
  unsigned char k;
  k = EEPROM.read(add);
  while (k != '\0' && len < 500) //Read until null character
  {
    k = EEPROM.read(add + len);
    data[len] = k;
    len++;
  }
  data[len] = '\0';

  return String(data);
}
  

void _writeEEPROM(String data) {
  Serial.print("Writing Data:");
  Serial.println(data);

  writeString(10, data);  //Address 10 and String type data
  delay(10);
}


void t1CallgetMeter() {     // Update read all data
  HeartBeat();
  readMeter();

}

void t3CallHeartbeat() {
  HeartBeat();
}
//void t4Restart() {     // Update read all data
//  Serial.println("Restart");
//  ESP.restart();
//}

 


char  char_to_byte(char c)
{
  if ((c >= '0') && (c <= '9'))
  {
    return (c - 0x30);
  }
  if ((c >= 'A') && (c <= 'F'))
  {
    return (c - 55);
  }
}
 

float HexTofloat(uint32_t x) {
  return (*(float*)&x);
}

uint32_t FloatTohex(float x) {
  return (*(uint32_t*)&x);
}
//------------------------------------------------

float Read_Meter_float(char addr , uint16_t  REG) {
  float i = 0;
  uint8_t j, result;
  uint16_t data[2];
  uint32_t value = 0;
  node.begin(addr, modbus);
  result = node.readInputRegisters (REG, 2); ///< Modbus function 0x04 Read Input Registers
  delay(500);
  if (result == node.ku8MBSuccess) {
    for (j = 0; j < 2; j++)
    {
      data[j] = node.getResponseBuffer(j);
    }
    value = data[0];
    value = value << 16;
    value = value + data[1];
    i = HexTofloat(value);
    //Serial.println("Connec modbus Ok.");
    return i;
  } else {
    Serial.print("Connec modbus fail. REG >>> "); Serial.println(REG, HEX); // Debug
    delay(1000);
    return 0;
  }
}

void readMeter() {     // Update read all data
  delay(1000);                              // เคลียบัสว่าง
  Serial.println("debug get meter 01");

    meter.sdmVolt = Read_Meter_float(ID_SDM, Reg_addr[0]);//แสกนหลายตัวตามค่า ID_METER_ALL=X
    meter.sdmCurrent = Read_Meter_float(ID_SDM, Reg_addr[1]);//แสกนหลายตัวตามค่า ID_METER_ALL=X
    meter.sdmPF = Read_Meter_float(ID_SDM, Reg_addr[3]);
    meter.freq = Read_Meter_float(ID_SDM, Reg_addr[4]);

    // Serial.println("sdmVolt: " + meter.sdmVolt);
    // Serial.println("sdmCurrent: " + meter.sdmCurrent);
    Serial.println("debug get meter 02");

}

void configModeCallback(WiFiManager *myWiFiManager)
{
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  // if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

void _initWiFi(){
  // WiFi.mode(WIFI_AP_STA); // Correct
  String host = "Mcgellan-" + deviceToken;
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setConfigPortalTimeout(60); // auto close configportal after n seconds
  wifiManager.setAPClientCheck(true);     // avoid timeout if client connected to softap
  wifiManager.setBreakAfterConfig(true);  // always exit configportal even if wifi save fails
  if (!wifiManager.autoConnect(host.c_str()))
    {
      Serial.println("failed to connect and hit timeout");
      delay(1000);
    }
}

void getMac()
{
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.println("OK");
  Serial.print("+deviceToken: ");
  Serial.println(WiFi.macAddress());
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 0x10) {
      deviceToken += "0"; // Add leading zero if needed
    }
    deviceToken += String(mac[i], HEX); // Convert byte to hex
  }
  deviceToken.toUpperCase();
  // ESPUI.updateLabel(GUI.idLabel, String(deviceToken));

}

void _initMagel(){
  Serial.println();
  Serial.println("debug magel 01");
  setting.ThingIdentifier = "67103330522117731051068";
  setting.ThingSecret = "18810218412214560";
  setting.endpoint = "magellan.ais.co.th"; //if not set *default: magellan.ais.co.th

  HeartBeat();

  if (connectWifi){

     magelWiFi.begin(setting);
     Serial.println();
     Serial.println("debug magel 02");
     magelWiFi.getControlJSON([](String controls){ 
      Serial.print("# Control incoming JSON: ");
      Serial.println(controls);
      String control = magelWiFi.deserializeControl(controls);
      magelWiFi.control.ACK(control); //ACKNOWLEDGE control to magellan ⚠️ important to Acknowledge control value to platform
    });
    Serial.println("Thing Token: " + magelWiFi.info.getThingToken());

  }else{

    // boolean status = GSMmqtt.connect(user_mqtt.c_str(), key.c_str(), secret.c_str());
    // if (status == false){
    //   Serial.println("magel: fail");
    //   }
    // magelGSM.setServer("magellan.ais.co.th", 1883);
    magelGSM.begin(setting);

    magelGSM.getControlJSON([](String controls){ 
      Serial.print("# Control incoming JSON: ");
      Serial.println(controls);
      String control = magelGSM.deserializeControl(controls);
      magelGSM.control.ACK(control); //ACKNOWLEDGE control to magellan ⚠️ important to Acknowledge control value to platform
      
    });
    Serial.println("Thing Token: " + magelGSM.info.getThingToken());
    }

  // setting.clientBufferSize = defaultOTAbuffer; // if not set *default: 1024
  HeartBeat();
  Serial.println();
  Serial.println("debug magel 03");
  
  // Serial.println("# Read Credential 1st time");
  // Serial.println("ThingIdentifier: "+ magel.credential.getThingIdentifier());
  // Serial.println("ThingSecret: "+ magel.credential.getThingSecret());
  // Serial.println("# Regenerate Credential");
  // magel.credential.regenerate(REGEN_BY_CONDITION); // if set true == will regenerate only credential activated, false regenerate without condition
  // Serial.println("# Read Credential 2nd time (After Regenerate)");
  // Serial.println("ThingIdentifier: "+ magel.credential.getThingIdentifier());
  // Serial.println("ThingSecret: "+ magel.credential.getThingSecret());
  // Serial.println("PreviousThingIdentifier: "+ magel.credential.getPreviousThingIdentifier());
  // Serial.println("PreviousThingSecret: "+ magel.credential.getPreviousThingSecret());
  // Serial.println("# Recovery Credential(swarp credential old to current)");
  // magel.credential.recovery();
  // Serial.println("ThingIdentifier: "+ magel.credential.getThingIdentifier());
  // Serial.println("ThingSecret: "+ magel.credential.getThingSecret());
  // Serial.println("PreviousThingIdentifier: "+ magel.credential.getPreviousThingIdentifier());
  // Serial.println("PreviousThingSecret: "+ magel.credential.getPreviousThingSecret());
  Serial.println();
}

bool checkForUpdate(String &firmware_url)
{
  HeartBeat();

  Serial.println("Making GSM GET request securely...");
  GSMclient.get(version_url);
  int status_code = GSMclient.responseStatusCode();
  delay(1000);
  String response_body = GSMclient.responseBody();
  delay(1000);

  Serial.print("Status code: ");
  Serial.println(status_code);
  Serial.print("Response: ");
  Serial.println(response_body);

  response_body.trim();
  response_body.replace("\r", ""); // Remove carriage returns
  response_body.replace("\n", ""); // Remove newlines

  // Extract the version number from the response
  new_version = response_body;

  Serial.println("Current version: " + current_version);
  Serial.println("Available version: " + new_version);
  GSMclient.stop();

  if (new_version != current_version)
  {
    Serial.println("New version available. Updating...");
    firmware_url = String("/Vichayasan/BMA/refs/heads/main/TX/firmware") + new_version + ".bin";
    Serial.println("Firmware URL: " + firmware_url);
    return true;
  }
  else
  {
    Serial.println("Already on the latest version");
  }

  return false;
}

// Update the latest firmware which has uploaded to Github
void performOTA(const char *firmware_url)
{
  HeartBeat();

  // Initialize HTTP
  Serial.println("Making GSM GET firmware OTA request securely...");
  GSMclient.get(firmware_url);
  int status_code = GSMclient.responseStatusCode();
  delay(1000);
  long contentlength = GSMclient.contentLength();
  delay(1000);

  Serial.print("Contentlength: ");
  Serial.println(contentlength);

  if (status_code == 200)
  {

    if (contentlength <= 0)
    {
      SerialMon.println("Failed to get content length");
      GSMclient.stop();
      return;
    }

    // Begin OTA update
    bool canBegin = Update.begin(contentlength);
    size_t written;
    long totalBytesWritten = 0;
    uint8_t buffer[1024];
    int bytesRead;
    long contentlength_real = contentlength;

    if (canBegin)
    {
      HeartBeat();

      while (contentlength > 0)
      {
        HeartBeat();

        bytesRead = GSMclient.readBytes(buffer, sizeof(buffer));
        if (bytesRead > 0)
        {
          written = Update.write(buffer, bytesRead);
          if (written != bytesRead)
          {
            Serial.println("Error: written bytes do not match read bytes");
            Update.abort();
            return;
          }
          totalBytesWritten += written; // Track total bytes written

          Serial.printf("Write %.02f%% (%ld/%ld)\n", (float)totalBytesWritten / (float)contentlength_real * 100.0, totalBytesWritten, contentlength_real);

          String OtaStat = "OTA Updating: " + String((float)totalBytesWritten / (float)contentlength_real * 100.0) + " % ";
          

          contentlength -= bytesRead; // Reduce remaining content length
        }
        else
        {
          Serial.println("Error: Timeout or no data received");
          break;
        }
      }

      if (totalBytesWritten == contentlength_real)
      {
        Serial.println("Written : " + String(totalBytesWritten) + " successfully");
      }
      else
      {
        Serial.println("Written only : " + String(written) + "/" + String(contentlength_real) + ". Retry?");
      }

      if (Update.end())
      {
        SerialMon.println("OTA done!");
        if (Update.isFinished())
        {
          SerialMon.println("Update successfully completed. Rebooting.");
          delay(300);
          ESP.restart();
        }
        else
        {
          SerialMon.println("Update not finished? Something went wrong!");
        }
      }
      else
      {
        SerialMon.println("Error Occurred. Error #: " + String(Update.getError()));
      }
    }
    else
    {
      Serial.println("Not enough space to begin OTA");
    }
  }
  else
  {
    Serial.println("Cannot download firmware. HTTP code: " + String(status_code));
  }

  GSMclient.stop();
}

void GSM_OTA()
{
  Serial.println("---- GSM OTA Check version before update ----");

  if (checkForUpdate(firmware_url))
  {
    performOTA(firmware_url.c_str());
  }
}

bool WiFicheckForUpdate(String &firmware_url)
{
  HeartBeat();
  Serial.println("Making WiFi GET request securely...");

  HTTPClient http;
  http.begin(version_url_WiFiOTA);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK)
  {
    new_version = http.getString();
    new_version.trim();

    Serial.print("Response: ");
    Serial.println(new_version);
    Serial.println("Current version: " + current_version);
    Serial.println("Available version: " + new_version);

    if (new_version != current_version)
    {
      Serial.println("New version available. Updating...");
      firmware_url = String("https://raw.githubusercontent.com/Vichayasan/BMA/refs/heads/main/TX/firmware") + new_version + ".bin";
      Serial.println("Firmware URL: " + firmware_url);
      return true;
    }
    else
    {
      Serial.println("Already on the latest version");
    }
  }
  else
  {
    Serial.println("Failed to check for update, HTTP code: " + String(httpCode));
  }

  http.end();
  return false;
}

void WiFiperformOTA(const char *firmware_url)
{
  HeartBeat();

  // Initialize HTTP
  Serial.println("Making WiFi GET fiemware OTA request securely...");

  HTTPClient http;

  http.begin(firmware_url);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK)
  {
    int contentLength = http.getSize();
    bool canBegin = Update.begin(contentLength);
    long contentlength_real = contentLength;

    Serial.print("Contentlength: ");
    Serial.println(contentLength);

    if (canBegin)
    {
      HeartBeat();

      Serial.println("WiFi OTA Updating..");
      String OtaStat = "WiFi OTA Updating...";

      size_t written = Update.writeStream(http.getStream());

      if (written == contentLength)
      {
        Serial.println("Written : " + String(written) + " successfully");
      }
      else
      {
        Serial.println("Written only : " + String(written) + "/" + String(contentLength) + ". Retry?");
      }

      if (Update.end())
      {
        Serial.println("OTA done!");
        if (Update.isFinished())
        {
          Serial.println("Update successfully completed. Rebooting.");
          delay(300);
          ESP.restart();
        }
        else
        {
          Serial.println("Update not finished? Something went wrong!");
        }
      }
      else
      {
        Serial.println("Error Occurred. Error #: " + String(Update.getError()));
      }
    }
    else
    {
      Serial.println("Not enough space to begin OTA");
    }
  }
  else
  {
    Serial.println("Cannot download firmware. HTTP code: " + String(httpCode));
  }

  http.end();
}

void WiFi_OTA()
{
  Serial.println("---- WiFi OTA Check version before update ----");

  if (WiFicheckForUpdate(firmware_url))
  {
    WiFiperformOTA(firmware_url.c_str());
  }
}


//**************************************************************************************************************
void setup() {

 pinMode(ledHeartPIN, OUTPUT);

  // Sink current to drain charge from watchdog circuit
  pinMode(trigWDTPin, OUTPUT);
  digitalWrite(trigWDTPin, LOW);

  pinMode(GSM_RESET, OUTPUT);
  digitalWrite(GSM_RESET, HIGH);

  Serial.begin(115200);
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  modem.sendAT("+QCFG=\"roamservice\",2");
  modem.testAT();

  Serial.println();
  // Serial.println("debug 01");

  modem.init();
  delay(30000);

  SerialMon.println("Initializing modem...");

  String modemInfo = modem.getModemInfo();
  Serial.print("Modem: ");
  Serial.println(modemInfo);

  Serial.println();
  Serial.println("debug 02");
  Serial.println();
  HeartBeat();

  delay(30000);

  Serial.print("Waiting for network...");
  if (!modem.waitForNetwork())
  {
    GSMnetwork = false;
    Serial.println(" fail");
    delay(10000);
    ESP.restart();
    
  }
  else
  {
    GSMnetwork = true;
    Serial.println(" OK");


  }
  // delay(2000);

  Serial.println();
  Serial.print("GSM RSSI: ");
  Serial.println(modem.getSignalQuality());
  Serial.print("SimCCID: ");
  Serial.println(modem.getSimCCID());
  Serial.print("SimStatus: ");
  Serial.println(modem.getSimStatus());
  Serial.print("RegistrationStatus: ");
  Serial.println(modem.getRegistrationStatus());
  Serial.println();
  delay(30000);
  HeartBeat();

  if (GSMnetwork)
  {
    String showText = "Connecting to ";
    showText += apn;
    showText += " ...";
    
    Serial.print("Connecting to ");
    Serial.print(apn);
    if (!modem.gprsConnect(apn, user, pass))
    {
      GSMgprs = false;
      Serial.println(" fail");
      delay(10000);
    }
    else
    {
      GSMgprs = true;
      Serial.println(" OK");
    }
    delay(3000);
  }

  if ((GSMnetwork == true) && (GSMgprs == true))
  {
    connectWifi = false;
  }

  if ((GSMnetwork == false) || (GSMgprs == false))
  {
    connectWifi = true;
  }

  Serial.println();
  delay(2000);

  if (connectWifi){
    _initWiFi();
  }

  Serial.printf("Wi-Fi RSSI: %d \n", WiFi.RSSI() );

  getMac();

  modbus.begin(2400, SERIAL_8N1, 16, 17);
  // communicate with Modbus slave ID 1 over Serial (port 2)
  node.begin(1, modbus);

  _initMagel();
  Serial.println();

  Serial.println();
  Serial.println(F("***********************************"));

  HeartBeat();
//  runner.init();

//  runner.addTask(t1);
//  Serial.println("added t1");
//  runner.addTask(t2);
//  Serial.println("added t2");
//  runner.addTask(t3);
//  Serial.println("added t3");
  //  runner.addTask(t4);
  //  Serial.println("added t4");

//  t1.enable();  Serial.println("Enabled t1");
//  t2.enable();  Serial.println("Enabled t2");
//  t3.enable();  Serial.println("Enabled t3");
  //  t4.enable();  Serial.println("Enabled t4");
  secure_layer.setInsecure();

}

void loop() {
  unsigned long currentMillis = millis();
  //float x = Read_Meter_float(ID_meter,Reg_Volt);
  //runner.execute();
  if(connectWifi){

    magelWiFi.loop();

    if(!magelWiFi.isConnected()) //*
      {
        magelWiFi.reconnect();
      }
    
    // magelWiFi.interval(10, [](){

    // }
  
  // );

  }else{

    magelGSM.loop();

    if(!magelGSM.isConnected()) //*
      {
        magelGSM.reconnect();
      }

  }



  if (currentMillis - previousMillis >= 30000){
    previousMillis = currentMillis;

    Serial.println("debug loop sender 01");
    readMeter() ;
 
  //  GET_METER();
    Serial.println();
  
  //   String json = "";
  //   json.concat("{\"Tn\":\"");
  //   json.concat(deviceToken);
  //   json.concat("\",\"vol\":");
  //   json.concat(meter.sdmVolt);
  //   json.concat(",\"cur\":");
  //   json.concat(meter.sdmCurrent);
  // //  json.concat(",\"ap\":");
  // //  json.concat(DATA_METER[2]);
  //  json.concat(",\"pf\":");
  //  json.concat(meter.sdmPF);
  //  json.concat(",\"f\":");
  //  json.concat(meter.freq);
  // //  json.concat(",\"TAE\":");
  // //  json.concat(DATA_METER[5]);
  //   json.concat("}");
  //   Serial.println(json);

    // int str_len = json.length() + 1;
    // char char_array[str_len];
    // json.toCharArray(char_array, str_len);
    // String msg = json.toJSONString();

    if (connectWifi)
    {

      Serial.println("debug loop sender wifi 02");

      // magelWiFi.loop();
      magelWiFi.subscribes([](){ //*lambda function
      magelWiFi.subscribe.control();
      magelWiFi.subscribe.report.response();
      Serial.println("Subscribe list!!!");
      /* subscribe something or doing something at once after connect/reconnect */
      // magelWiFi.report.send(json.toJSONString());

      // magelWiFi.sensor.add("volt", meter.sdmVolt);
      // magelWiFi.sensor.add("current", meter.sdmCurrent);
      // magelWiFi.sensor.add("powerFactor", meter.sdmPF);
      // magelWiFi.sensor.add("frequency", meter.freq);
      // magelWiFi.sensor.report();

      Serial.println("debug loop sender wifi 02");

     });
      resultSub = magelWiFi.report.send("{\"DeviceToken\":\"" + String(deviceToken) + "\",\"Network\":\"Wi-Fi\",\"volt\":" + String(meter.sdmVolt) + ",\"current\":" + String(meter.sdmCurrent) + ",\"PowerFactor\":" + String(meter.sdmPF) + ",\"frequency\":" + String(meter.freq) + "}"); //send data sensor with manual json format
      Serial.print("[Status report]: ");
      Serial.println((resultSub)? "Sending via Wi-Fi SUCCESS" : "Sending via Wi-Fi FAIL");

      Serial.println("debug loop sender wifi 03");

    }else{

      Serial.println("debug loop sender GSM 01");
      // magelGSM.loop();
      magelGSM.subscribes([](){ //*lambda function
      magelGSM.subscribe.control();
      magelGSM.subscribe.report.response();
      Serial.println("Subscribe list!!!");
      /* subscribe something or doing something at once after connect/reconnect */
      // magelGSM.report.send(json.toJSONString());

      // magelGSM.sensor.add("volt", meter.sdmVolt);
      // magelGSM.sensor.add("current", meter.sdmCurrent);
      // magelGSM.sensor.add("powerFactor", meter.sdmPF);
      // magelGSM.sensor.add("frequency", meter.freq);
      // magelGSM.sensor.report();

     Serial.println("debug loop sender GSM 02");

     
      });
      resultSub = magelGSM.report.send("{\"DeviceToken\":\"" + String(deviceToken) + "\",\"Network\":\"GSM\",\"volt\":" + String(meter.sdmVolt) + ",\"current\":" + String(meter.sdmCurrent) + ",\"PowerFactor\":" + String(meter.sdmPF) + ",\"frequency\":" + String(meter.freq) + "}"); //send data sensor with manual json format
      Serial.print("[Status report]: ");
      Serial.println((resultSub)? "Sending via GSM SUCCESS" : "Sending via GSM FAIL");
      Serial.println("debug loop sender GSM 03");

    }

  }

  if (currentMillis - periodOTA >= 60000){
    periodOTA = currentMillis;
    if (connectWifi){
        WiFi_OTA();
      }else{
        GSM_OTA();
      }
  }
  
}


void HeartBeat() {
  // Sink current to drain charge from watchdog circuit
  pinMode(trigWDTPin, OUTPUT);
  digitalWrite(trigWDTPin, LOW);

  // Led monitor for Heartbeat
  digitalWrite(ledHeartPIN, LOW);
  delay(300);
  digitalWrite(ledHeartPIN, HIGH);

  // Return to high-Z
  pinMode(trigWDTPin, INPUT);

  Serial.println("Heartbeat sent");
  
}