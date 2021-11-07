# Twomes Room Monitor Module firmware
Firmware for the Twomes Room Monitor Module. 

## Table of contents
* [General info](#general-info)
* [Pairing](#pairing)
* [Deploying](#deploying)
* [Developing](#developing) 
* [Features](#features)
* [Status](#status)
* [License](#license)
* [Credits](#credits)

## General info
This firmware is designed to run on the ESP32 of the [Twomes Temperature Monitor hardware](https://github.com/energietransitie/twomes-temperature-monitor-hardware), connected with the [Twomes CO₂ Meter Shield hardware](https://github.com/energietransitie/twomes-co2-meter-hardware) which contains two primary sensors:
* Sensirion [SCD41](https://www.sensirion.com/en/environmental-sensors/carbon-dioxide-sensors/carbon-dioxide-sensor-scd4x/) CO₂ sensor
* [Si7051](https://www.silabs.com/sensors/temperature/si705x/device.si7051) temperature sensor

The ESP32 reads various properties of these sensors, and stores them in [RTC fast memory](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/memory-types.html#rtc-fast-memory), to preserve them during the deep sleep interval between brief moments when the ESP32 needs to be awake for data acquisition. After a series of 20 measurements at fixed time intervals, the ESP32 will send the series of measurements to the [Twomes P1 Gateway measurement device](https://github.com/energietransitie/twomes-p1-gateway-firmware) using the ESP-NOW protocol.

Via the [Twomes P1 Gateway measurement device](https://github.com/energietransitie/twomes-p1-gateway-firmware), the Twomes Room Monitor Module sends the following properties to the Twomes server:
| Sensor | Property           | Unit | Data type  | Format | Interval [h:mm:ss]| Description                            |
|--------|--------------------|------|------------|--------|-------------------|----------------------------------------|
| SCD41  | `CO2concentration` | ppm  | float      | f.0    | 0:05:00           | CO₂ concentration                      |
| SCD41  | `humidity`         | %RH  | float      | f.0    | 0:05:00           | relative humidity                      |
| SCD41  | `roomTempCO2`      | °C   | float      | f.1    | 0:05:00           | air temperature                        |
| Si7051 | `roomTemp`         | °C   | float      | f.1    | 0:05:00           | air temperature                        |

## Pairing
After [deploying](#deploying) and installation in the home, the Twomes Room Monitor Module should be [paired as a satellite to the Twomes P1 Gateway measurement device](https://github.com/energietransitie/twomes-p1-gateway-firmware/blob/main/README.md#pairing-satellites).

## Deploying
This section describes how you can deploy binary releases of the firmware, i.e. without changing the source code, without a development environment and without needing to compile the source code.

### Prerequisites
To deploy the firmware, in addition to the [generic prerequisites for deploying Twomes firmware](https://github.com/energietransitie/twomes-generic-esp-firmware#prerequisites), you need:
* a 3.3V TTL-USB Serial Port Adapter (e.g. [FT232RL](https://www.tinytronics.nl/shop/en/communication-and-signals/usb/ft232rl-3.3v-5v-ttl-usb-serial-port-adapter), CP210x, etc..), including the cable to connect ths adapter to a free USB port on your computer (a USB to miniUSB cable in the case of a [FT232RL](https://www.tinytronics.nl/shop/en/communication-and-signals/usb/ft232rl-3.3v-5v-ttl-usb-serial-port-adapter));
* You need to place a 3.6V Lithium AA-battery (e.g. SAFT LS14500) in its holder to power the board, as the Twomes Room Monitor Module is not powered via the 6 pin programming connector.
* Find a row of 6 holes holes (J1 in the corner of the PCB of the Toom Monitor Module), find the `GND` pin (see  bottom of the PCB), alighn the 6 pins of the serial port adapter such that `GND` and other pins match; then connect the serial port adapter to your computer and connect the 6 pins of the serial port adapter to the 6 holes on the PCB.

### Uploading firmware
* Download the [binary release for your device](https://github.com/energietransitie/twomes-p1-room-monitor-firmware/releases) and extract it to a directory of your choice.
* Follow the [generic Twomes firmware upload instructions ](https://github.com/energietransitie/twomes-generic-esp-firmware#device-preparation-step-1a-uploading-firmware-to-esp32), with the exceptions mentioned below:
	* When you see the beginning of the sequence `Connecting ......_____......`, press and hold the button labeled `GPIO0 (SW2)` on the PCB, then briefly press the button labeled `RESET (SW1)`. 
	* You should see an indication that the firmware is being written to the device.
	* When the upload is finished, view the serial output with a serial monitor tool like PuTTY or the utility of your IDE (115200 baud). Press `RESET (SW1)` shortly to  make sure the firmware boots. 
	* Remove the battery or insert a battery insulating pull tab when you do not plan to use the room monitor module immediately, to prevent unnecessary battery drain.

## Developing 
This section describes how you can change the source code using a development environment and compile the source code into a binary release of the firmware that can be depoyed, either via the development environment, or via the method described in the section [Deploying](#deploying).

Please see the [developing section of the generig Twomes firmware](https://github.com/energietransitie/twomes-generic-esp-firmware#developing) first. Reember to presse buttons to upload the firmware: 
* When you see the beginning of the sequence `conecting ......_____......`, press and hold the button labeled `GPIO0 (SW2)` on the PCB, then briefly press the button labeled `RESET (SW1)` shortly. 
* You should see an indication that the firmware is being written to the device.

## Features
List of features ready and TODOs for future development. 

Ready:
* Pairing with [Twomes P1 Gateway measurement device](https://github.com/energietransitie/twomes-p1-gateway-firmware)
* Measure temperature using the [Si7051](https://www.silabs.com/sensors/temperature/si705x/device.si7051) sensor
* Measure CO₂ concentration, relative humidity and temperature using the [SCD41](https://www.sensirion.com/en/environmental-sensors/carbon-dioxide-sensors/carbon-dioxide-sensor-scd4x/) sensor
* Deep sleep between data acquisition for ultra-low power consumption
* Send measurement data to the paired [Twomes P1 Gateway measurement device](https://github.com/energietransitie/twomes-p1-gateway-firmware) using ESP-NOW

To-do:
* Implement more error indicators using LEDs

## Status
Project is: in Progress.

## License
This software is available under the [Apache 2.0 license](./LICENSE.md), Copyright 2021 [Research group Energy Transition, Windesheim University of Applied Sciences](https://windesheim.nl/energietransitie) 

## Credits
This software is a collaborative effort of:
* Sjors Smit ·  [@Shorts1999](https://github.com/Shorts1999)
* Fredrik-Otto Lautenbag ·  [@Fredrik1997](https://github.com/Fredrik1997)
* Gerwin Buma ·  [@GerwinBuma](https://github.com/GerwinBuma) 
* Werner Heetebrij ·  [@Werner-Heetebrij](https://github.com/Werner-Heetebrij)
* Henri ter Hofte · [@henriterhofte](https://github.com/henriterhofte) · Twitter [@HeNRGi](https://twitter.com/HeNRGi)

Product owner:
* Marco Winkelman · [@MarcoW71](https://github.com/MarcoW71)

We use and gratefully aknowlegde the efforts of the makers of the following source code and libraries:
* [ESP-IDF](https://github.com/espressif/esp-idf), by Espressif Systems, licensed under [Apache 2.0 License](https://github.com/espressif/esp-idf/blob/master/LICENSE)
