/**
 *  @file       DS18B20.h
 *  @version    0.1.0
 *
 *  @brief      Driver for DS18B20 Temperature sensor
 *
 *  @date       Jul 1, 2019
 *  @author     technosf <github.10.technomation@xoxy.net>
 *  @see        https://github.com/technosf/ESP32-Peripheral-Drivers
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
#include <cstring>

#include "OneWireBus.h"
#include "OneWireDevice.h"


//TODO configure the device with write scratch
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
            static const constexpr char* TAG { "DS18B20" };     //!< ESP_LOG Tag 


        public:

            DS18B20( onewire_rom_code_t reg, OneWireBus* bus );


            /**
             * @enum PWR_SPLY
             * 
             */
            enum PWR_SPLY
            {
                UNKNOWN = 0,            //!< Device power ssupplyource unknown
                BUS = 1,                //!< Bus powered devices only
                PARASITIC = 2,          //!< One or more parasiticly powered devices
            };


            /**
             * @brief DS18B20 Temperature resolutions
             * 
             * The resolution of the temperature sensor is user-configurable to 9, 10, 11, or 12 bits,
             * corresponding to increments of 0.5°C, 0.25°C, 0.125°C, and 0.0625°C, respectively.
             */
            static const constexpr float RESOLUTION [4] { 0.5, 0.25, 0.125, 0.0625 };

            struct scratchpad_t
            {
                 union {
                    uint8_t raw [9];
                    struct {   
                        uint8_t TEMP_LSB;
                        uint8_t TEMP_MSB;
                        uint8_t T_H_REGISTER;
                        uint8_t T_L_REGISTER;
                        uint8_t CONFIG;
                        uint8_t RESERVED_FFH;
                        uint8_t RESERVED;
                        uint8_t RESERVED_10H;
                        uint8_t CRC;
                    } eeprom;
                 };
                 
                void copy(onewire_data_t& data);

                uint8_t getResolution();

                float getTemperature();

                bool isValid();
            };


/* --------------------------------------------------------
*
*   Statics
*
*  --------------------------------------------------------
*/

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
            static const constexpr float TEMP_UNDEFINED { 99999.99 };

            /**
             * @brief Identify all family devices on the given bus.
             * @param bus the bus to scan
             * @return true if scan occurred
             */
            //static bool identify( OneWireBus* bus );

            /**
             * @brief Returns a vector of all identified DS18B20's on any bus
             * @return DS18B20 devices
             */
           // static std::vector< DS18B20* > getDevices();

            /**
             * @brief Returns a vector of all identified DS18B20's on this bus
             * @param bus the bus to get DS18B20 devices on
             * @return DS18B20 devices
             */
            static std::vector< DS18B20* > getDevices( OneWireBus* bus, bool autosearch = true );


            /**
             *
             */
            virtual ~DS18B20();

            /* --------------------------------------------------------
             *
             * High Level Functions
             *
             * --------------------------------------------------------
             */

            /**
             * @brief Issues a Convert T command on the attached bus to this device or all devices
             * 
             * @param thisdevice true if address this device only
             * @return true command completed successfully
             */
            virtual bool cmdConvertT( bool thisdevice = true );

            /**
             * @brief Issues a Convert T command on the attached bus to this device or all devices
             * 
             * @param thisdevice true if address this device only
             * @return true command completed successfully
             */
            virtual bool cmdReadScratchpad();


            /**
             * @brief Issues a Read Power Supply command if power supply not already known
             * 
             * If the power supply for the bus or device is unknown, queires the bus and device
             * 
             * @return the power supply for the device
             */
            virtual PWR_SPLY cmdReadPowerSupply();


            /**
             * @brief 
             * 
             * @param rom_codes 
             * @param search_start 
             * @return true 
             * @return false 
             */
            virtual bool cmdAlarmSearch( std::vector< onewire_rom_code_t >& rom_codes );


            /* --------------------------------------------------------
             *
             * Helpers
             *
             * --------------------------------------------------------
             */

            scratchpad_t getScratchpad();

            /**
             * @brief Get the Resolution object
             * 
             * @return uint8_t 
             */
            uint8_t getResolution();

            /**
             * @brief Get the Temperature reading from the last scratchpad reading
             * 
             * @return float temperture in Celsius
             */
            float getTemperature();

            /* --------------------------------------------------------
             *
             * OneWire DS18B20 Commands
             *
             * --------------------------------------------------------
             */

            /**
             * @brief Issue a Convert T command on the given bus
             *
             * @return true is sent
             */
            static bool cmd_convert_t( OneWireBus* bus );

            /**
             * @brief reads from the scratch pad and interprets the value as Celsius temperature
             * @param bus the bus to issue the command on
             * @return the value converted into Celsius
             */
            static bool cmd_read_scratchpad( OneWireBus* bus, scratchpad_t& scratchpad );

            /**
             * @brief
             * @param bus
             * @return
             */
            static bool cmd_write_scratchpad( OneWireBus* bus, uint8_t th, uint8_t tl, uint8_t config );

            /**
             * @brief
             * @param bus
             * @return
             */
            static bool cmd_copy_scratchpad( OneWireBus* bus );

            /**
             * @brief
             * @param bus
             * @return
             */
            static bool cmd_recall_e2( OneWireBus* bus );

            /**
             * @brief Issue a Read Power Supply command on the bus and store the result
             *
             * read_power_supply is followed by a read slot where a zero value indicates
             * parasitic device response, one bus power, or unknown if the command
             * was not successfully transmitted.
             *
             * @return the power source, if known
             */
            static PWR_SPLY cmd_read_power_supply( OneWireBus* bus );


            /**
             * @brief Issues a Alarm Search command. Requires reset first.
             * 
             * Implementation of the Maxim search algorithm, using a struct for the
             * search state initial and ending status. Searches from the given state
             * and returns when a device, or no more devices are found.
             *
             * @param search_state the state to search and returning state
             * @param autoreset Allows bus reset to bypassed
             * @return true if the command was issued
             */
            virtual bool cmd_alarm_search( OneWireBus* bus, OneWireBus::onewire_search_state_t& search_state, bool autoreset = true ); 

        private:

            /**
             * @brief Busses with identified DS18B20 devices, and if they are know and parasitic
             */
            static std::unordered_map< OneWireBus*, PWR_SPLY > BUS_POWER;

            /**
             * @brief All known DS18B20 ROM codes and their devices
             */
            static std::unordered_map< onewire_rom_code_t, DS18B20* > DEVICE_ROM_CODES_BUS;


            OneWireBus* p_bus;    // Bus this device is connected to
            //PWR_SPLY m_parasitic_power { PWR_SPLY::UNKNOWN };    // Is this device using parasitic power
            //float last_celsius_reading { TEMP_UNDEFINED };

            scratchpad_t m_scratchpad;
            PWR_SPLY m_power_supply { PWR_SPLY::UNKNOWN };

    }; // DS18B20

}// namespace

#endif /* EPD_DS18B20_H_ */
