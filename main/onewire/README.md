# OneWire -  Dallas 1Wire Driver

ESP32 Peripheral Driver for Dallas 1Wire devices built with ESP-IDF

## Overview & Goals

The 1Wire protocol is a simple topology that can support multiple heterogeneous device over a single data data wire -
The genesis of this codebase is my attempt at using multiple DS18B20 temperature sensors. The 1Wire protocol physical layer is bases on wire timing signals lasting from 1us to 600us (plus _Over-Drive_ timings) and an open-drain on the wire voltage so that anyone device can indicate its presence. The network layer is a set of signals that affect the bus and all devices on it. Subsequent levels target families of devices with operational commands and data.

I've segmented the code into the bus specification, bus implementation (currently just one using the RMT peripheral), device basics and a overall class to cover common use cases that require organizing several commands.


## Components

### OneWireBus

_OneWireBus_ is an abstract class that defines the operating state for a logical _bus_.

It defines:
* Standard timings for the bus
* _Over Drive_ timings for the bus
* Physical layer commands
* State of the bus, including command ordering dependencies
* Enforcing exclusive access to the bus  during command execution
* Lower level functions for marshalling data to and from the bus

#### OneWireBusRMT

_OneWireBusRMT_ is an implementation of _OneWireBus_ that uses the ESP32 RMT peripheral. Using two GPIO pins, two RMT Channels, and two RMT Channel memory blocks,  _OneWireBusRMT_ writes and reads from the same bus wire at the same time.  

There is an RX RMT Channel that uses 2 RMT memory blocks connected to one pin, and a TX RMT Channel that uses 1 RMT memory block connected to another ping. These two pins are connected together and to the physical bus wire. TX writes to the bus from heap memory rather than via the RMT Channel buffer, so by using consecutive channels, RX can 'share' TX the memory block. 

### OneWireDevice

_OneWireDevice_ is the basic building block for actual devices. It contains commonalities such as deriving device family, address and registration code validation.

### OneWire

_OneWire_ is a high-level helper, implementing high-level use cases by combining the operations needed to execute scenarios into a single function. 


## Use

```
#include <vector>

#include <OneWireBus.h>
#include <OneWireBusRMT.h>
#include <DS18B20.h>

void main()
{
	// Create a OneWireBus using RMT peripheral on pins 18, 19 using RMT Channels 6,7
	epd::OneWireBusRMT* owb = new epd::OneWireBusRMT( 18, 19 );
	
	// Initialize the bus and scan the registration codes of the devices on it
	owb->initialize();	
	owb->scanRegistrationCodes();
	printf( "Found %d devices\n", owb->getRegistrationCodes().size() );
	
	
	std::vector< epd::DS18B20* > ds18b20 = epd::DS18B20::getDevices( owb );
	printf( "Found %d DS18B20 sensors\n", ds18b20.size() );
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
