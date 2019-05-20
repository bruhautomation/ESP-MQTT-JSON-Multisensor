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

  UPDATE 20 MAY 2019 - Upgraded code to support latest DHT library

  UPDATE 20 MAY 2019 - Updated reconnect() function to reset board if MQTT connection fails 5 or more times (So you can re-enter the MQTT details using the AP code configuration portal)

  UPDATE 20 MAY 2019 - Added Port, Sensor Name, State Topic and Set Topic as custom parameters which can be set using the configuration portal

  UPDATE 20 MAY 2019 - Added onDemandPortal access in case of incorrect MQTT settings. Previously went into an infinite loop trying to connect using incorrect credentials

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
#include <ArduinoJson.h>

#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic


/************ PIN DEFINITIONS ************/
#define PIRPIN    D5
#define DHTPIN    D7
#define DHTTYPE   DHT22
#define LDRPIN    A0


/************ SENSOR VARIABLES ************/
char sensor_name[50] = "new_MultisensorDevice";
bool shouldSaveConfig = false;

int calibrationTime = 5;
const int BUFFER_SIZE = 300;

float ldrValue;
int LDR;
float calcLDR;
float diffLDR = 10;

#define IsFahrenheit false //to use celsius change to false
float diffTEMP = 0.2;
float tempValue;

float diffHUM = 1;
float humValue;

#define PIR_ON_STRING "motion_detected"
#define PIR_OFF_STRING "standby"
int pirValue;
int pirStatus;
String motionStatus;

WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE);


/************ MQTT VARIABLES ************/
char mqtt_server[40];
char mqtt_user[40];
char mqtt_password[40];
char mqtt_port[6];
char multisensor_state_topic[100] = "sensornodes/new_MultisensorDevice";
char multisensor_set_topic[100] = "sensornodes/new_MultisensorDevice/set";
#define MQTT_MAX_PACKET_SIZE 512




 
bool readConfig() {

  Serial.print("Mounting file system");
  if (SPIFFS.begin()) {
    Serial.println("..........file system mounted successfully");
    Serial.print("Searching for existing configuration file");
    if (SPIFFS.exists("/config.json")) {
      Serial.println("..........file found");
      Serial.print("Opening configuration file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("..........file opened successfully");
        Serial.print("Reading configuration file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, buf.get());
        if (error) {
          Serial.print("..........failed to read file with error code: ");
          Serial.println(error.c_str());
          Serial.println("Starting access point");
          return false;
        } else {
          Serial.println("..........file read successfully");
          Serial.print("File contents: ");
          serializeJson(doc, Serial);
          Serial.println();

          strcpy(sensor_name, doc["sensor_name"]);
          strcpy(mqtt_server, doc["mqtt_server"]);
          strcpy(mqtt_port, doc["mqtt_port"]);
          strcpy(mqtt_user, doc["mqtt_user"]);
          strcpy(mqtt_password, doc["mqtt_password"]);
          strcpy(multisensor_state_topic, doc["multisensor_state_topic"]);
          strcpy(multisensor_set_topic, doc["multisensor_set_topic"]);
        }
      } else {
        Serial.println("..........failed to open file");
        Serial.println("Starting access point");
        return false;

      }
    } else {
      Serial.println("..........file not found");
      Serial.println("Starting access point");
      return false;
    }
  } else {
    Serial.println("..........failed to mount file system");
    Serial.println("Starting access point");
    return false;
  }
  return true;
}

void saveConfig() {

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("..........failed to open configuration file for writing");
  } else {
    DynamicJsonDocument doc(1024);
    doc["sensor_name"] = sensor_name;
    doc["mqtt_server"] = mqtt_server;
    doc["mqtt_port"] = mqtt_port;
    doc["mqtt_user"] = mqtt_user;
    doc["mqtt_password"] = mqtt_password;
    doc["multisensor_state_topic"] = multisensor_state_topic;
    doc["multisensor_set_topic"] = multisensor_set_topic;
    serializeJson(doc, configFile);
    Serial.println("..........configuration saved successfully");
    Serial.print("New file contents: ");
    serializeJson(doc, Serial);
    Serial.println();
    configFile.close();
  }
}

void initWifiManager() {

  WiFiManagerParameter custom_sensor_name("sensorname", "sensor_name", sensor_name, 100);
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_user("user", "mqtt user", mqtt_user, 40);
  WiFiManagerParameter custom_mqtt_password("password", "mqtt password", mqtt_password, 40);
  WiFiManagerParameter custom_multisensor_state_topic("statetopic", "multisensor_state_topic", multisensor_state_topic, 100);
  WiFiManagerParameter custom_multisensor_set_topic("settopic", "multisensor_set_topic", multisensor_set_topic, 100);

  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  wifiManager.addParameter(&custom_sensor_name);
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_password);
  wifiManager.addParameter(&custom_multisensor_state_topic);
  wifiManager.addParameter(&custom_multisensor_set_topic);

  wifiManager.setAPCallback(configModeCallback);

  wifiManager.setDebugOutput(false);

  if (!wifiManager.autoConnect(sensor_name, "SSEJO_12412!")) {
    Serial.println("Failed to: connect to known access point / Start AP mode - Resetting board");
    software_Reset();
  }

  strcpy(sensor_name, custom_sensor_name.getValue());
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_password, custom_mqtt_password.getValue());
  strcpy(multisensor_state_topic, custom_multisensor_state_topic.getValue());
  strcpy(multisensor_set_topic, custom_multisensor_set_topic.getValue());

}

void initMQTT() {
  client.setServer(mqtt_server, atoi(mqtt_port));
  client.setCallback(mqttCallback);
}


void onDemandPortal() { //For if you get the MQTT settings wrong

  WiFiManagerParameter custom_sensor_name("sensorname", "sensor_name", sensor_name, 100);
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_user("user", "mqtt user", mqtt_user, 40);
  WiFiManagerParameter custom_mqtt_password("password", "mqtt password", mqtt_password, 40);
  WiFiManagerParameter custom_multisensor_state_topic("statetopic", "multisensor_state_topic", multisensor_state_topic, 100);
  WiFiManagerParameter custom_multisensor_set_topic("settopic", "multisensor_set_topic", multisensor_set_topic, 100);

  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  wifiManager.addParameter(&custom_sensor_name);
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_password);
  wifiManager.addParameter(&custom_multisensor_state_topic);
  wifiManager.addParameter(&custom_multisensor_set_topic);

  wifiManager.setAPCallback(configModeCallback);

  wifiManager.setDebugOutput(false);

  if (!wifiManager.startConfigPortal(sensor_name, "SSEJO_12412!")) {
    Serial.println("failed to connect and hit timeout");
    delay(1000);
    //reset and try again, or maybe put it to deep sleep
    software_Reset();
  }

  strcpy(sensor_name, custom_sensor_name.getValue());
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_password, custom_mqtt_password.getValue());
  strcpy(multisensor_state_topic, custom_multisensor_state_topic.getValue());
  strcpy(multisensor_set_topic, custom_multisensor_set_topic.getValue());

  if (shouldSaveConfig) {
    saveConfig();
  }

}


/************ SETUP ************/
void setup() {

  pinMode(PIRPIN, INPUT);
  pinMode(DHTPIN, INPUT);
  pinMode(LDRPIN, INPUT);

  Serial.begin(115200);

  delay(5000);

  bool existingConfig = readConfig();

  initWifiManager();

  if (shouldSaveConfig) {
    saveConfig();

  }

  initMQTT();

  Serial.println("Set-up Finished.");

  Serial.println();
  Serial.println();
  Serial.println("#########################################");
  Serial.print("Sensor Name: ");
  Serial.println(sensor_name);
  Serial.print("Device local ip: ");
  Serial.println(WiFi.localIP());
  Serial.println("#########################################");
  Serial.println();
  Serial.println();

  dht.begin();
  reconnect();
}


/************ CONFIG SAVE CALLBACK ************/
void saveConfigCallback () {
  Serial.print("Configuration needs saving");
  shouldSaveConfig = true;
}


/************ WIFI MANAGER CALLBACK ************/
void configModeCallback (WiFiManager * myWiFiManager) {
  Serial.println("Device has entered configuration mode..........connect to the AP point to complete set-up");
  Serial.println(WiFi.softAPIP());
  Serial.println(myWiFiManager->getConfigPortalSSID());
}


/************ MQTT NEW MESSAGE CALLBACK ************/
void mqttCallback(char* topic, byte * payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.println("] ");

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


/************ PROCESS JSON ************/
bool processJson(char* message) {
  StaticJsonDocument<BUFFER_SIZE> doc;

  DeserializationError error = deserializeJson(doc, message);
  if (error) {
    Serial.println("Failed to deserialize message");
    return false;
  }

  return true;
}



/************ SEND CURRENT STATES ************/
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

  Serial.print("OUTBOUND MESSAGE -->: ");
  Serial.println(buffer);
  client.publish(multisensor_state_topic, buffer, true);
}


/************ RECONNECT ************/
void reconnect() {

  int attempts = 1;

  while (!client.connected()) {
    Serial.print("Attempting to connect to MQTT server");

    if (client.connect(sensor_name, mqtt_user, mqtt_password)) {
      Serial.println("..........connected successfully");
      client.subscribe(multisensor_set_topic);
      sendState();
    } else {
      Serial.print("..........connection failed with error code: ");
      Serial.println(client.state());
      Serial.print("Retrying in 5 seconds");

      int reconDelay = 1;
      while (reconDelay < 6) {
        Serial.print("..");
        reconDelay++;
      }

      Serial.println("..........retrying now");

      attempts++;
    }
    if (attempts > 4) {
      Serial.println("Max attempts reached");
      bool configFileRM = SPIFFS.remove("/config.json");
      if (!configFileRM) {
        Serial.println("Failed to remove configuration file, this may not work. You may have to reflash the board");
      } else {
        Serial.println("Successfully removed incorrect configuration file");
        onDemandPortal();
        initMQTT();
        attempts = 1;
        delay(1000);
      }

    }
  }
}


/************ START CHECK SENSOR ************/
bool checkBoundSensor(float newValue, float prevValue, float maxDiff) {
  return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}


/************ MAIN LOOP ************/
void loop() {

  if (!client.connected()) {
    software_Reset();
  }

  client.loop();

  float newTempValue = dht.readTemperature();
  float newHumValue = dht.readHumidity();

  if (isnan(newHumValue) || isnan(newTempValue)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  delay(250);

  //PIR CODE
  pirValue = digitalRead(PIRPIN); //read state of the

  if (pirValue == LOW && pirStatus != 1) {
    motionStatus = PIR_OFF_STRING;
    sendState();
    pirStatus = 1;
  } else if (pirValue == HIGH && pirStatus != 2) {
    motionStatus = PIR_ON_STRING;
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


/************ RESET ESP ************/
void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
  Serial.println("..........resetting board now\n\n\n");
  delay(2000);
  ESP.reset();
  delay(2000);
}
