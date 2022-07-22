# ESP32 Peripheral Drivers
ESP32 Peripheral Drivers built with ESP-IDF

This top-level driver collection provide high-level functions to operate the peripherals. If you are using *ESP-IDF* and *cmake*, clone it into the *components* directory of your project.


## The Drivers

### Released

#### MAX6675

The _MAX6675_ is a SPI temperature sensor interface for K-type thermocouples. It measures from 0°C to 1023°C in 0.25°C increments. This driver can read and return Celsius, Fahrenheit and the raw data produced by the sensor.


### In Progress

#### DHT22

The _DHT22_ is a temperature and humidity sensor that runs off 3V-5V and uses single-wire signaling to initiate and transmit its readings along with a checksum, with one reading being available up to one every two seconds. 

This driver uses the ESP32 _RMT_ device to capture the input from the DHT22. It uses a single _RMT_ device and one _RMT_ memory block only (i.e. it can use any one RMT device, even #7). By pulling the GPIO low this driver causes the DHT22 to signal a start pulse and following 40 data bits which are captured by the RMT and processed into five bytes. The start pulse is typically low 80µs + high 80µs, while each data bit pulse is low 50-70µs followed by a high of 25-30µs for 0 or a high of 70-80µs for 1. 

The processed data status, the  temperature (Celsius) and humidity (percent) can be read from the API, as can the raw 40 bit signal data. Looking for prior art for the _RMT_ approach, [jcollie's repo](https://github.com/jcollie/esp32DHT) provided proof of concept.

#### DS18B20

The _DS18B20_ is a temperature sensor that uses the Dallas 1Wire bus for signaling and communication, and either runs off a 3V-5V supply or off power from the signaling wire parasistically. It measures from -55°C to +125°C and has selectable accuracy, from 9 to 12 bits. Multiple devices can run off the Dallas 1Wire bus as each is uniquely addressed. Upper and Lower temperature limits can be configured in each _DS18B20_ which if met will set the devices alarm state: A controller on the Dallas 1Wire bus can search the bus for devices with alarm states using an Alarm Search command, and without having to interogate each device on the bus.


### Examples

#### MAX6675
```C++
    #include <esp_system.h>
    #include <freertos/FreeRTOS.h>
    #include <freertos/task.h>
    #include <math.h>

    #include "MAX6675.h"

    void app_main()
    {
        epd::MAX6675* max6675 = new epd::DS18B20(epd::MAX6675::SPI2);

        while (true)
        {
            printf("temp R=%u\n",  max6675->getRaw().temperature_reading);
            printf("temp C=%f\n",  max6675->getCelsius(false));
            printf("temp F=%f\n",  max6675->getFahrenheit(false));
            vTaskDelay(20);
        }
    }
```

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
