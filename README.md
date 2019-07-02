# ESP32-Peripheral-Drivers
ESP32 Peripheral Drivers built with ESP-IDF

A collection of driver classes for GPIO connected peripherals.

## The Drivers

### DHT22

The _DHT22_ is a temperature and humidity sensor that runs off 3V-5V and uses single-wire signaling to initiate and transmit its readings, with one reading being available up to one every two seconds..

This driver uses the ESP32 _RMT_ device to capture the input from the DHT22 and then process the data into five bytes. This data can be read raw, or as temperature (Celsius) and humidity (percent). Looking for prior art for the _RMT_ approach, [jcollie's repo](https://github.com/jcollie/esp32DHT) provided proof of concept.


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
