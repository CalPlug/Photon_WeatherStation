# Photon_WeatherStation
This project is designed for a local weather station using a Particle Photon microcontroller which has WiFi capabilities.  The developed project posts sensor values to a MQTT server.  

Description:
Photon weather station includes temperature, humidity, pressure, rain in inches,
wind speed, direction of wind, and ionizing radiation data from UV and geiger counter sensors.
The Geiger Counter that was interfaced via UART is by Mighty Ohm:  http://mightyohm.com/blog/products/geiger-counter/
Based on the example code from Haodong Liang who used ThingSpeak, but modified for it also to be 
used for CloudMQTT. Link to his project: https://www.hackster.io/hliang/thingspeak-weather-station-data-analysis-2877b0.
The SI1145 driver was developed by Limor Fried of Adafruit:  https://learn.adafruit.com/adafruit-si1145-breakout-board-uv-ir-visible-sensor/
The TCA9548a driver was developed by Limor Fried of Adafruit:  https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout/wiring-and-test?view=all with porting to Photon by Rickkas7: https://github.com/rickkas7/TCA9548A-RK  
The Sparkfun Weather Shield base project was developed by N. Seidle of SparkFun.
brownout protection by JVanier: https://community.particle.io/t/eeprom-persistence-issue/16514/39

Authors:

Sid Kasat, CS Junior @ UC Irvine

Mindy Saylors, EE Junior @ UC Irvine

Hamed Ghafarshad, CS Senior @ UC Irvine

Alex Ramirez, CSE Junior @ UC Irvine

Project Managers: Dr. Michael Klopfer, Prof. GP Li.
California Institute for Telecommunications and Information Technology (Calit2), 2017
University of California, Irvine
Extended components of project copyright Regents of the Univeristy of California and released into the public domain.

Released 12/5/2017
