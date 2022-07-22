/**
 *  @file       OneWire.h
 *  @version    0.1.0
 *
 *  @brief      Driver for Dallas 1-Wire protocol operations -- core definitions
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

#ifndef EPD_ONEWIRE_H_
#define EPD_ONEWIRE_H_

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <freertos/semphr.h>

#include <stdint.h>
#include <string.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "OneWireBus.h"
#include "OneWireDevice.h"

namespace epd
{

    /**
     * @class OneWire
     * @ingroup epd
     *
     * @brief Core Dallas 1-Wire bus protocol definition
     */
    class OneWire
    {
            static const constexpr char* TAG { "OneWire" };

        public:

            /**
             * @brief Default constructor
             */
            OneWire();

            /**
             * @brief Constructor with single bus
             * @param bus
             */
            OneWire( OneWireBus* bus );

            /**
             *
             */
            virtual ~OneWire()
            {
            }

            /**
             * @brief Add a OneWire bus to the object
             * 
             * @param bus the bus to add
             * @return True if bus added
             */
            virtual bool addBus( OneWireBus* bus );


            /**
             * @brief Remove a OneWire bus from this object
             * 
             * @param bus the bus to remove
             * @return True if bus removed
             */
            virtual bool removeBus( OneWireBus* bus );


            /**
             * @brief Scan for and identify all devices connected to busses associated with this object
             * 
             * @return True if the scan completed without issue
             */
            virtual bool scanDevices();


            /**
             * @brief Verify that the given device is valid and connected to a bus
             * @param device the device to verify
             * @return true if the device was verified attached
             */
            //    virtual bool verifyDevice( OneWireDevice device );

        private:

            std::unordered_set< OneWireBus* > m_busses;                         //!<  OneWire Busses

            std::unordered_map< uint64_t, OneWireBus* > m_rom_codes;   //!<  Device registration codes and associated bus
            
            // std::unordered_map< uint8_t, OneWireDevice* > m_family_devices;    // Valid devices by Family Code
            //     std::unordered_map< uint64_t, OneWireDevice* > m_valid_devices;    // Valid devices by Registration Cod

            //OneWireDevice find_next_device( OneWireDevice from_device );

            /**
             * @brief search all busses from the search point returning first device found
             *
             * @param search_point the search point and configuration
             * @return false if search failed to execute
             */
            //  bool _searchAll( OneWireBus::onewire_search_state_t& search_point );
            /**
             * @brief search given bus from the search point returning first device found
             *
             * @param bus the bus to search
             * @param search_point the search point and configuration
             * @return false if search failed to execute
             */
            //    bool _search( OneWireBus* bus, OneWireBus::onewire_search_state_t& search_point );
            /**
             * @brief Validate a given registration code and if valid add it to the database of devices
             *
             * If the device already exists, return NULL
             *
             * @param registration_code the registration code
             * @param bus the bus the registration code is on
             * @return the device if new and valid, or nullptr if not
             */
            //   OneWireDevice* _validate_and_add_new_device( uint64_t registration_code, OneWireBus* bus );
e

    };
// OneWire
}// epd
#endif /* EPD_ONEWIRE_H_ */
