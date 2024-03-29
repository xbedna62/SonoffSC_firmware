# SonoffSC

Custom firmware for the Sonoff SC (both for the ATMega328P and the ESP8266).

[![version](https://badge.fury.io/gh/xoseperez%2Fsonoffsc.svg)](CHANGELOG.md)
[![travis](https://travis-ci.org/xoseperez/sonoffsc.svg?branch=master)](https://travis-ci.org/xoseperez/sonoffsc)
[![license](https://img.shields.io/github/license/xoseperez/sonoffsc.svg)](LICENSE)

## Features

* Up to 5 different **configurable WiFi networks** with static IP support
* **MQTT** enabled, send sensor data to your local/cloud broker
* Support for all the sensors in the board
    * **DHT11** but also for the **DHT22** (upgrading hardware)
    * **Sharp GP2Y1010AU0F**
    * **GM55 LDR**
    * **Electret microphone**
* Fast asynchronous **HTTP Server**
    * Basic authentication
    * Web-based configuration
    * Websockets-based communication between the device and the browser
    * Web server code cleaned, merged and embedded into firmware for fastest access
* **NTP** synchronisation
* **Over-The-Air** (OTA) updates (only for the ESP8266)
* **Remote OTA** updates using the [NoFuss library](https://bitbucket.org/xoseperez/nofuss) (only for the ESP8266)
* [**Domoticz**](https://domoticz.com/) integration via MQTT
* Enable/disable clap mode from **Alexa**
* **Debug** messages via UDP & Telnet

## Documentation

*TODO*

Both projects (for the atmega328p and the esp8266) are ready to be built with [PlatformIO](http://platformio.org/).
If you do not use PlatformIO check the platformio.ini files for dependencies (in the "lib_deps" parameter).

Know more here:
* [http://tinkerman.cat/sonoff-sc-with-mqtt-and-domoticz-support](http://tinkerman.cat/sonoff-sc-with-mqtt-and-domoticz-support)
* [http://tinkerman.cat/itead-studio-sonoff-sc-revisited/](http://tinkerman.cat/itead-studio-sonoff-sc-revisited/)

## License

Copyright (C) 2017-2018 by Xose Pérez (@xoseperez)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
