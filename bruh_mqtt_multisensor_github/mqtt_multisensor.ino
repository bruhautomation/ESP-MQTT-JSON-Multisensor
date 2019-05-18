/*
  CHANGE LOG
  ----------
  UPDATE 16 MAY 2017 by Knutella - Fixed MQTT disconnects when wifi drops by moving around Reconnect and adding a software reset of MCU

  UPDATE 23 MAY 2017 - The MQTT_MAX_PACKET_SIZE parameter may not be setting appropriately due to a bug in the PubSub library. If the MQTT messages are not being transmitted as expected you may need to change the MQTT_MAX_PACKET_SIZE parameter in "PubSubClient.h" directly.

  UPDATE 27 NOV 2017 - Changed HeatIndex to built in function of DHT library. Added definition for fahrenheit or celsius

  UPDATE 20 APR 2019 - Removed the need for hardcoded network credentials by using the WifiManager library. On first boot the nodemcu will create an access point which you can connect to for configuration. If every the nodemcu can't connect to the last known wifi network it will go into AP mode again.

  UPDATE 20 APR 2019 - Fixed LED flash as 'flash' is no longer a seperate attribute in homeassistant, it's now within the effects list

  UPDATE 21 APR 2019 - Added MQTT settings as custom parameters (Excl. port) which can be set when the device is in AP mode

  UPDATE 18 MAY 2019 - Removed LED and associated code, serves limited use

  UPDATE 18 MAY 2019 - Upgraded code to support ArduinoJson 6

  REQUIRED LIBRARIES
  ------------------
  - DHT sensor library
  - Adafruit unified sensor
  - PubSubClient
  - ArduinoJSON
  - WifiManager

  SUPPORTED BOARDS
  ----------------
  - ESP8266 (You can add it to the board manager by going to File -> Preference and pasting http://arduino.esp8266.com/stable/package_esp8266com_index.json
             into the Additional Board Managers URL field. Next, download the ESP8266 dependancies by going to Tools -> Board -> Board Manager
             and searching for ESP8266 and installing it.)
*/


#include <FS.h>
#include <ESP8266WiFi.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>

#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic



//flag for saving data
bool shouldSaveConfig = false;

/************ WIFI and MQTT INFORMATION (CHANGE THESE FOR YOUR SETUP) ******************/
char mqtt_server[40];
char mqtt_user[40];
char mqtt_password[40];
#define mqtt_port 1883

/************* MQTT TOPICS (change these topics as you wish)  **************************/
#define light_state_topic "home/sensornode/bedroomMultiSensor"
#define light_set_topic "home/sensornode/bedroomMultiSensor/set"


/************ TEMP SETTINGS (CHANGE THIS FOR YOUR SETUP) *******************************/
#define IsFahrenheit false //to use celsius change to false


/**************************** FOR OTA **************************************************/
#define SENSORNAME "MultiSensor_1"
#define OTApassword "SENOR_OTA_PASSWORD" // change this to whatever password you want to use when you upload OTA
int OTAport = 8266;

/**************************** PIN DEFINITIONS ********************************************/
#define PIRPIN    D5
#define DHTPIN    D7
#define DHTTYPE   DHT22
#define LDRPIN    A0



/**************************** SENSOR DEFINITIONS *******************************************/
float ldrValue;
int LDR;
float calcLDR;
float diffLDR = 10;

float diffTEMP = 0.2;
float tempValue;

float diffHUM = 1;
float humValue;

int pirValue;
int pirStatus;
String motionStatus;

char message_buff[100];

int calibrationTime = 5;

const int BUFFER_SIZE = 300;

#define MQTT_MAX_PACKET_SIZE 512

WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE);

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

/********************************** START SETUP*****************************************/
void setup() {

  Serial.begin(115200);

  pinMode(PIRPIN, INPUT);
  pinMode(DHTPIN, INPUT);
  pinMode(LDRPIN, INPUT);

  Serial.begin(115200);
  delay(5);

  //read configuration from FS json
  Serial.println("Mounting file system...");

  if (SPIFFS.begin()) {
    Serial.println("file system successfully mounted");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("\nReading config file...");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("config file opened successfully");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, buf.get());
        serializeJson(doc, Serial);
        if (error) {
          Serial.println("\nFailed to load json config with error:");
          Serial.println(error.c_str());
        } else {
          Serial.println("\nJSON parsed successfully");
          strcpy(mqtt_server, doc["mqtt_server"]);
          strcpy(mqtt_user, doc["mqtt_user"]);
          strcpy(mqtt_password, doc["mqtt_password"]);
        }
      }
    }
  } else {
    Serial.println("\nFailed to mount FS");
  }
  //end read

  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_user("user", "mqtt user", mqtt_user, 40);
  WiFiManagerParameter custom_mqtt_password("passsord", "mqtt password", mqtt_password, 40);

  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_password);

  wifiManager.setAPCallback(configModeCallback);

  wifiManager.autoConnect(SENSORNAME, "SENSOR_AP_PASSWORD");

  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_password, custom_mqtt_password.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("\nSaving config...");
    DynamicJsonDocument doc(1024);
    doc["mqtt_server"] = mqtt_server;
    doc["mqtt_user"] = mqtt_user;
    doc["mqtt_password"] = mqtt_password;
    Serial.println("\nconfig saves successfully...");

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("\nFailed to open config file for writing");
    }

    serializeJson(doc, Serial);
    serializeJson(doc, configFile);
    configFile.close();
    //end save
  }

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  ArduinoOTA.setPort(OTAport);
  ArduinoOTA.setHostname(SENSORNAME);
  ArduinoOTA.setPassword((const char *)OTApassword);

  ArduinoOTA.onStart([]() {
    Serial.println("\nOTA server starting");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA server ended");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("\nOTA Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("\nOTA Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("\nOTA Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("\nOTA Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("\nOTA End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("\nOTA server ready");

  Serial.println("\nStarting multisensor node:" + String(SENSORNAME));
  reconnect();
}


/********************************** WIFI MANAGER CALLBACK ****************************************/
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("\nEntered config mode");
  Serial.println(WiFi.softAPIP());
  Serial.println(myWiFiManager->getConfigPortalSSID());
}



/********************************** START CALLBACK*****************************************/
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  char message[length + 1];
  for (int i = 0; i < length; i++) {
    message[i] = (char)payload[i];
  }
  message[length] = '\0';
  Serial.println(message);

  if (!processJson(message)) {
    return;
  }

  sendState();
}



/********************************** START PROCESS JSON*****************************************/
bool processJson(char* message) {
  StaticJsonDocument<BUFFER_SIZE> doc;

  DeserializationError error = deserializeJson(doc, message);
  if (error) {
    Serial.println("parseObject() failed");
    return false;
  }

  return true;
}



/********************************** START SEND STATE*****************************************/
void sendState() {
  StaticJsonDocument<BUFFER_SIZE> doc;

  doc["humidity"] = (String)humValue;
  doc["motion"] = (String)motionStatus;
  doc["ldr"] = (String)LDR;
  doc["temperature"] = (String)tempValue;
  doc["heatIndex"] = (String)dht.computeHeatIndex(tempValue, humValue, IsFahrenheit);

  char buffer[measureJson(doc) + 1];
  size_t buffSize = measureJson(doc) + 1;
  serializeJson(doc, buffer, buffSize);

  Serial.println(buffer);
  client.publish(light_state_topic, buffer, true);
}


/********************************** START RECONNECT*****************************************/
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(SENSORNAME, mqtt_user, mqtt_password)) {
      Serial.println("Connected to MQTT server");
      client.subscribe(light_set_topic);
      sendState();
    } else {
      Serial.print("\nMQTT connection failed (Error code:");
      Serial.print(client.state());
      Serial.println(") - Trying again in 5 seconds...\n");
      delay(5000);
    }
  }
}



/********************************** START CHECK SENSOR **********************************/
bool checkBoundSensor(float newValue, float prevValue, float maxDiff) {
  return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}


/********************************** START MAIN LOOP***************************************/
void loop() {

  ArduinoOTA.handle();

  if (!client.connected()) {
    // reconnect();
    software_Reset();
  }
  client.loop();

  float newTempValue = dht.readTemperature(IsFahrenheit);
  float newHumValue = dht.readHumidity();

  //PIR CODE
  pirValue = digitalRead(PIRPIN); //read state of the

  if (pirValue == LOW && pirStatus != 1) {
    motionStatus = "standby";
    sendState();
    pirStatus = 1;
  }

  else if (pirValue == HIGH && pirStatus != 2) {
    motionStatus = "motion detected";
    sendState();
    pirStatus = 2;
  }

  delay(100);

  if (checkBoundSensor(newTempValue, tempValue, diffTEMP)) {
    tempValue = newTempValue;
    sendState();
  }

  if (checkBoundSensor(newHumValue, humValue, diffHUM)) {
    humValue = newHumValue;
    sendState();
  }


  int newLDR = analogRead(LDRPIN);

  if (checkBoundSensor(newLDR, LDR, diffLDR)) {
    LDR = newLDR;
    sendState();
  }
}


/****reset***/
void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
  Serial.print("resetting");
  ESP.reset();
}
