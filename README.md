# ESP MQTT JSON Digital LEDs

This project shows a super easy way to get started with your own DIY Multisensor to use with [Home Assistant](https://home-assistant.io/), a sick, open-source Home Automation platform that can do just about anything. 

The code covered in this repository utilizies Home Assistant's [MQTT JSON Light Component](https://home-assistant.io/components/light.mqtt_json/), [MQTT Sensor Component](https://home-assistant.io/components/sensor.mqtt/), and a [NodeMCU ESP8266](http://geni.us/cpmi) development board. 


### Supported Features Include
- **DHT22** temperature sensor
- **DHT 22** humdidity sensor
- **AM312** PIR motion sensor 
- **photoresistor** or **TEMT600** light sensor
- **RGB led* with support for color, flash, fade, and transition
- **Over-the-Air (OTA)** ppload from the ArduinoID


#### OTA Uploading
This code also supports remote uploading to the ESP8266 using Arduino's OTA library. To utilize this, you'll need to first upload the sketch using the traditional USB method. However, if you need to update your code after that, your WIFI-connected ESP chip should show up as an option under Tools -> Port -> Porch at your.ip.address.xxx. More information on OTA uploading can be found [here](http://esp8266.github.io/Arduino/versions/2.0.0/doc/ota_updates/ota_updates.html). Note: You cannot access the serial monitor over WIFI at this point.  


### Parts List

**Amazon Prime (fast shipping)**
- [NodeMCU 1.0](http://geni.us/cpmi)
- [DHT22 Module](http://geni.us/vAJWMXo)
- [LDR Photoresistor Module](http://geni.us/O0AO0)
    OR
- [TEMT6000](http://geni.us/aRYe)
- [Power Supply](http://geni.us/ZZ1r)
- [Common Cathode RGB Led](http://geni.us/nFcB)
- [Header Wires](http://geni.us/pvFNG)
- [Mini PIR Sensor](http://geni.us/dbGQ)

**Aliexpress (long shipping = cheap prices)**
- [NodeMCU 1.0](http://geni.us/EfYA)
- [DHT22 Module](http://geni.us/35Np8H)
- [LDR Photoresistor Module](http://geni.us/O5iv)
    OR
- [TEMT6000](http://geni.us/xAuLoy)
- [Power Supply](http://geni.us/NSYjvb)
- [Common Cathode RGB Led](http://geni.us/OfHbhZb)
- [Header Wires](http://geni.us/Iv6p9)
- [Mini PIR Sensor](http://geni.us/WBKyxhx)


### Wiring Diagram
![alt text](https://github.com/bruhautomation/ESP-MQTT-JSON-Multisensor/blob/master/wiring_diagram.png?raw=true "Wiring Diagram")


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
"transition":"50"
}
```
