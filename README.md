# ESP32-Peripheral-Drivers
ESP32 Peripheral Drivers built with ESP-IDF

A collection of driver classes for GPIO connected peripherals.

## The Drivers

In-depth Driver documentation is located in the [main code directory](main). The base Dalals 1Wire bus driver is located the [onewire directory under main](main/onewire).

### DHT22

The _DHT22_ is a temperature and humidity sensor that runs off 3V-5V and uses single-wire signaling to initiate and transmit its readings along with a checksum, with a new reading being available up to one every two seconds. 

### DS18S20

The _DS18B20_ is a temperature sensor that runs off 3V-5V and uses the _Dallas 1Wire_ bus for signaling and communication. It has configurable accuracy. 


## License

ESP32-Peripheral-Drivers - Copyright 2019  technosf  [http://github.com/technosf]

Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 or greater (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
http://www.gnu.org/licenses/gpl-3.0.en.html
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
![glp3 logo](http://www.gnu.org/graphics/gplv3-88x31.png)

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
