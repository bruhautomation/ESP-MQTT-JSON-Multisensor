# ESP MQTT JSON Multisensor
This project is based on the Multisensor project by bruhAutomation
https://github.com/bruhautomation/ESP-MQTT-JSON-Multisensor


### Supported Features Include
- **DHT22** temperature sensor
- **DHT22** humidity sensor
- **AM312** PIR motion sensor 
- **Photoresistor** or **TEMT600** light sensor
- **RGB led** with support for color, flash, fade, and transition
- **Over-the-Air (OTA)** upload from the ArudioIDE
- **Wifi configuration** using a temporary access point


### Upgrades
- **Removed LED** and associated code (Served limited purpose)
- **ArduinoJson6** compatible code
- **Implemented WifiManager library** to set WiFi and MQTT settings using an **access point**, removing hardcoded values
- Update code to support **latest DHT library**


### To Do List
- TBC


#### OTA Uploading
This code also supports remote uploading to the ESP8266 using Arduino's OTA library. To utilize this, you'll need to first upload the sketch using the traditional USB method. However, if you need to update your code after that, your WIFI-connected ESP chip should show up as an option under Tools -> Port -> Porch at your.ip.address.xxx. More information on OTA uploading can be found [here](http://esp8266.github.io/Arduino/versions/2.0.0/doc/ota_updates/ota_updates.html). Note: You cannot access the serial monitor over WIFI at this point.  


#### Parts List
- [TEMT6000](http://geni.us/aRYe)
- [Power Supply](http://geni.us/ZZ1r)
- [Common Cathode RGB Led](http://geni.us/nFcB)
- [Header Wires](http://geni.us/pvFNG)
- [AM312 Mini PIR Sensor](http://geni.us/dbGQ)


### Wiring Diagram
![alt text](https://github.com/bruhautomation/ESP-MQTT-JSON-Multisensor/blob/master/wiring_diagram_v2.png?raw=true "Wiring Diagram")
