# ESP MQTT JSON Multisensor

This project is based on the Multisensor project by bruhAutomation
https://github.com/bruhautomation/ESP-MQTT-JSON-


### Updates


### Supported Features Include
- **DHT22** temperature sensor
- **DHT22** humidity sensor
- **AM312** PIR motion sensor 
- **photoresistor** or **TEMT600** light sensor
- **RGB led** with support for color, flash, fade, and transition
- **Over-the-Air (OTA)** upload from the ArudioIDE
- **Wifi configuration** using a temporary access point



#### OTA Uploading
This code also supports remote uploading to the ESP8266 using Arduino's OTA library. To utilize this, you'll need to first upload the sketch using the traditional USB method. However, if you need to update your code after that, your WIFI-connected ESP chip should show up as an option under Tools -> Port -> Porch at your.ip.address.xxx. More information on OTA uploading can be found [here](http://esp8266.github.io/Arduino/versions/2.0.0/doc/ota_updates/ota_updates.html). Note: You cannot access the serial monitor over WIFI at this point.  



- [TEMT6000](http://geni.us/aRYe)
- [Power Supply](http://geni.us/ZZ1r)
- [Common Cathode RGB Led](http://geni.us/nFcB)
- [Header Wires](http://geni.us/pvFNG)
- [AM312 Mini PIR Sensor](http://geni.us/dbGQ)


### Wiring Diagram
![alt text](https://github.com/bruhautomation/ESP-MQTT-JSON-Multisensor/blob/master/wiring_diagram_v2.png?raw=true "Wiring Diagram")


### Home Assistant Service Examples
Besides using the card in Home Assistant's user interface, you can also use the Services tool to control the light using the light.turn_on and light.turn_off services. This will let you play with the parameters you can call later in automations or scripts. 

Fade the Light On Over 5 Seconds - light.turn_on
```
{"entity_id":"light.sn1_led",
"brightness":150,
"color_name":"blue",
"transition":"5"
}
```

Flash The Light - light.turn_on
```
{"entity_id":"light.sn1_led",
"color_name":"green",
"brightness":255,
"flash":"short"
}
```

Fade the Light Off Over 5 Seconds - light.turn_off
```
{"entity_id":"light.sn1_led",
"transition":"5"
}
```
