/*
 * OneWireDevice.h
 *
 * Core definition for Dallas 1-Wire Bus devices
 *
 *  Created on: Jul 20, 2019
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

#ifndef EPD_ONEWIREDEVICE_H_
#define EPD_ONEWIREDEVICE_H_

#include <esp_log.h>

#include <stdint.h>
#include <string.h>
#include <array>

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
            static const constexpr char* TAG { "OneWireDevice" };

        private:

            uint64_t m_registration_code;
            uint8_t m_crc;
            std::array< uint8_t, 6 > m_address;
            uint8_t m_family;
            bool m_valid { false };
            std::string m_info;

        public:

            /**
             * @brief Validates a reg code
             * @param reg the reg code
             * @return true if valid
             */
            static bool validate( uint64_t reg );

            /**
             * @brief Identifies the family from the reg code
             * @param reg the reg code
             * @return the family
             */
            static uint8_t family( uint64_t reg );

            /**
             * @brief Constructor generates and validates the device registration code
             *
             * @param bus the OneWireBus that the device was discovered on
             * @param reg raw registration data from the device
             */
            OneWireDevice( uint64_t reg );

            virtual ~OneWireDevice()
            {
            }

            /**
             * @brief Information about this device
             */
            const virtual char* info();

            /**
             * @brief Returns the registration code
             * @return  registration code
             */
            uint64_t getRegistrationCode();

            /**
             * @brief The device CRC
             * @return the CRC8
             */
            uint8_t getCrc();

            /**
             * @brief The device address
             * @return Six byte device address
             */
            std::array< uint8_t, 6 > getAddress();

            /**
             * @brief Returns the family code for the device
             * @return the device family code
             */
            uint8_t getFamily();

            /**
             * @brief Returns the given byte of the registration code
             * @param byte 0-7 for the 8 bytes
             * @return the byte
             */
            uint8_t getByte( uint8_t byte );

            /**
             * @brief indicates if the reg code for this device is valid
             * @return true if valid
             */
            bool isValid();
    };
// OneWireDevice

}// epd

#endif /* EPD_ONEWIREDEVICE_H_ */
