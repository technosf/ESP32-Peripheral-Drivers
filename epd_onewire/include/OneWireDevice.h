/**
 *  @file       OneWireDevice.h
 *  @version    0.1.0
 *
 *  @brief      Driver for Dallas 1-Wire protocol operations
 *
 *  @date       Jul 22, 2019
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

#ifndef EPD_ONEWIREDEVICE_H_
#define EPD_ONEWIREDEVICE_H_

#include <esp_log.h>

#include <stdint.h>
#include <string.h>
#include <array>

#include "OneWireBus.h"


namespace epd
{

    /**
     * @class OneWireDevice
     * @ingroup epd
     *
     * @brief Base class for Dallas 1-wire devices
     *
     * Instantiates with a 1-Wire bus and raw registration code
     *
     * @author technosf <github.10.technomation@xoxy.net>
     * @see https://github.com/technosf/ESP32-Peripheral-Drivers
     */
    class OneWireDevice
    {
            static const constexpr char* TAG { "OneWireDevice" };   //!< ESP LOG Tag

        public:

            /**
             * @brief Validates a reg code
             * 
             * @param reg the reg code
             * @return true if valid
             */
            static bool validate( onewire_rom_code_t reg );
            static bool validate( uint8_t* regbytes, uint8_t bytes );


            /**
             * @brief Identifies the family from the reg code
             * 
             * @param reg the reg code
             * @return the family
             */
            static uint8_t family( onewire_rom_code_t reg );


            /**
             * @brief Constructor generates and validates the device registration code
             *
             * @param reg raw registration data from the device
             */
            OneWireDevice( onewire_rom_code_t reg );


            /**
             * @brief Destroy the One Wire Device object
             * 
             */
            virtual ~OneWireDevice()
            {
            }

            /**
             * @brief Information about this device
             */
            const virtual char* info();


            /**
             * @brief Returns the device ROM code
             * 
             * @return  rom code
             */
             onewire_rom_code_t getRomCode();


            /**
             * @brief The device CRC
             * 
             * @return the CRC8
             */
            uint8_t getCrc();


            /**
             * @brief The device serial numberess
             * 
             * @return Six byte serial number
             */
            onewire_rom_code_t getSerialNumber();


            /**
             * @brief Returns the family code for the device
             * 
             * @return the device family code
             */
             uint8_t getFamily();

            /**
             * @brief Indicates if the ROM code for this device is valid
             * 
             * @return true if valid
             */
             bool isValid();


            /**
             * @brief Is this device is attached to a bus, or the given OneWireBus
             * 
             * If no bus is supplied, will return the current state. If a bus is
             * supplied the device bus , that bus will be interogated, and will be set as the 
             * device bus if found to contain this device.
             * 
             * 
             * @param bus the specific OneWireBus to check
             * @param clearBus if true will clear the device current bus info before interogation
             * @return epd::OneWireBus* the bus device is attached to, or nullptr if not attached
             */
            epd::OneWireBus* isBusAttached(epd::OneWireBus* bus = nullptr, bool clearBus = false );

        protected:

            OneWireBus* m_one_wire_bus { nullptr };         //!< OneWireBus the device is attached to, if known

        private:

            onewire_rom_code_t m_rom_code;                  //!< Device ROM code
            uint8_t m_crc;                                  //!< Device CRC
            onewire_rom_code_t m_serial_number {0};         //!< Device serial number
            uint8_t m_family;                               //!< Device family
            bool m_valid { false };                         //!< Device valid flag
            std::string m_info;                             //!< Device info
    };
// OneWireDevice

}// epd

#endif /* EPD_ONEWIREDEVICE_H_ */
