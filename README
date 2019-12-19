# ESP-IDF component to publish BME680 data via MQTT

## What is this?
This is a component to monitor the air quality with a BME680 sensor. The sensor is connected with I2C to the ESP32, which in turn publishes the data to a MQTT broker.

## Warning
I made this code to get some sensor values to my home mqtt server, so I can check how bad the air currently is.
This code is best described as "messy proof of concept", so please do not expect fancy things like error checking :-)

## Contents / Installation
This uses the binary Bosch Sensortec library which processes the raw data. I changed the sensor parameters that the library reports, so I could f.e. get the sIAQ (IAQ index optimised for static use). Please take a look at README-bsec and LICENSE-bsec.

Large parts of this are taken from the ESP-IDF examples, but I further simplified a lot of things. Those examples state that they should not be used in production, and I simplified them even further. See the warning above.

To use this, you need to follow a few steps:
 - Create an Esp-Idf project like the HelloWorld example, and replace the main component with this
 - set your WLAN name / password in firmware_connection.c, line 74
 - adapt the absolute path in CMakeLists.txt, relative path did not work for me
