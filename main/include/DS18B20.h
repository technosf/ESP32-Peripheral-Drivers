/*
 * DS18B20.h
 *
 * Driver for DS18B20 Temperature sensor
 *
 *  Created on: Jul 1, 2019
 *      Author: technosf <github.10.technomation@xoxy.net>
 *      See: https://github.com/technosf/ESP32-Peripheral-Drivers
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef EPD_DS18B20_H_
#define EPD_DS18B20_H_

#include <esp_err.h>
#include <esp_log.h>
#include <driver/gpio.h>

#include <unordered_map>

#include "OneWireBus.h"
#include "OneWireDevice.h"

namespace epd
{

    /**
     * @class DS18B20
     * @ingroup epd
     *
     * @brief Communicates with DS18B20 temperature sensor using the ESP32's RMT interface
     *
     * This driver uses the ESP32 RMT peripheral to capture the DHT22 signaling in the background
     * while it waits to process the checksum and calculate the temperature and humidity.
     *
     * Most of the timing can be modified via the configuration constants.
     *
     * @author technosf <github.10.technomation@xoxy.net>
     * @see https://github.com/technosf/ESP32-Peripheral-Drivers
     */
    class DS18B20 : public OneWireDevice
    {
            static const constexpr char* TAG { "DS18B20" };

            enum class PWR_SRC
            {
                UNKNOWN, BUS, PARASITIC,
            };

        public:

            /**
             * The family code for DS18B20
             */
            static const uint8_t FAMILY { 0x28 };

            /**
             *
             */
            static const uint8_t STRONG_PULLUP_US { 12 };

            /**
             *
             */
            static const constexpr float TEMP_UNDEFINED { 999.99 };

            /**
             * @brief Identify all family devices on the given bus.
             * @param bus the bus to scan
             * @return true if scan occurred
             */
            static bool identify( OneWireBus* bus );

            /**
             * @brief Returns a vector of all identified DS18B20's on any bus
             * @return DS18B20 devices
             */
            static std::vector< DS18B20* > getDevices();

            /**
             * @brief Returns a vector of all identified DS18B20's on this bus
             * @param bus the bus to get DS18B20 devices on
             * @return DS18B20 devices
             */
            static std::vector< DS18B20* > getDevices( OneWireBus* bus );

            /* --------------------------------------------------------
             *
             * High Level Functions
             *
             * --------------------------------------------------------
             */

            /**
             * @brief What power source Is this device using parasitic power?
             * @return true if it is, false if not or unknown.
             */
            PWR_SRC getPowerSource();

            /**
             *
             * @param bus
             * @return
             */
            PWR_SRC getPowerSource( OneWireBus* bus );

            /**
             *
             */
            virtual ~DS18B20();

            /* --------------------------------------------------------
             *
             * OneWire DS18B20 Commands
             *
             * --------------------------------------------------------
             */

        protected:
            /**
             * @brief Issue a Convert T command on the given bus
             *
             * @return true is sent
             */
            static bool convert_t( OneWireBus* bus );

            /**
             * @brief reads from the scratch pad and interprets the value as Celsius temperature
             * @param bus the bus to issue the command on
             * @return the value converted into Celsius
             */
            static float read_scratchpad( OneWireBus* bus );

            /**
             * @brief
             * @param bus
             * @return
             */
            static bool write_scratchpad( OneWireBus* bus );

            /**
             * @brief
             * @param bus
             * @return
             */
            static bool copy_scratchpad( OneWireBus* bus );

            /**
             * @brief
             * @param bus
             * @return
             */
            static bool recall_e2( OneWireBus* bus );

            /**
             * @brief Issue a Read Power Supply command on the bus and store the result
             *
             * read_power_supply is followed by a read slot where a zero value indicates
             * parasitic device response, one bus power, or unknown if the command
             * was not successfully transmitted.
             *
             * @return the power source, if known
             */
            static PWR_SRC read_power_supply( OneWireBus* bus );

        private:

            /*
             * Busses with identified DS18B20 devices, and if they are know and parasitic
             */
            static std::unordered_map< OneWireBus*, PWR_SRC > BUS_POWER;

            /*
             * DS18B20 Reg codes and their devices
             */
            static std::unordered_map< uint64_t, DS18B20* > DEVICE_CODES;

            /*
             * DS18B20 reg codes and their busses
             */
            static std::unordered_map< DS18B20*, OneWireBus* > DEVICE_BUSSES;

            OneWireBus* m_bus;    // Bus this device is connected to
            PWR_SRC m_parasitic_power { PWR_SRC::UNKNOWN };    // Is this device using parasitic power
            float last_celsius_reading { TEMP_UNDEFINED };

            /**
             * @brief Instantiate new DHT22
             * @param pin the GPIO pin connected to the DHT22 data line
             * @param channel the RMT channel to use to capture data
             */
            DS18B20( uint64_t reg, OneWireBus* bus );

    };
// DS18B20

}// namespace

#endif /* EPD_DS18B20_H_ */
