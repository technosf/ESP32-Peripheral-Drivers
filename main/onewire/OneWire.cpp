/*
 * OneWire.cpp
 *
 * Driver for Dallas 1-Wire protocol operations
 *
 *  Created on: Jul 22, 2019
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

#include "OneWire.h"

using namespace epd;

OneWire::OneWire()
{
}

OneWire::OneWire( OneWireBus* bus )
{
    m_busses.insert( bus );
}

bool OneWire::addBus( OneWireBus* bus )
{
    if ( m_busses.find( bus ) == m_busses.end() )    // Bus already known
    return false;

    m_busses.insert( bus );    // Add new bus
    return true;
}

bool OneWire::removeBus( OneWireBus* bus )
{
    return ( m_busses.erase( bus ) == 1 );
}

bool OneWire::scanDevices()
{
    ESP_LOGD( TAG, "::scanDevices - Start" );

    bool result { true };
    std::unordered_map< uint64_t, OneWireBus* > valid_registration_codes;
    std::unordered_map< uint64_t, OneWireDevice* > valid_devices;
    std::unordered_map< uint8_t, OneWireDevice* > family_devices;

    for ( auto bus : m_busses )
    /*
     * Loop around all busses
     */
    {

        ESP_LOGD( TAG, "::search_rom - Scanning bus %s", bus->info() );
        bus->scanRegistrationCodes();
        ESP_LOGD( TAG, "::scanDevices - found %d devices, validating...", bus->getRegistrationCodes().size() );

        for ( auto reg_code : bus->getRegistrationCodes() )
        /*
         * process each found reg code into a device, storing the valid ones
         */
        {
            // OneWireDevice* device = new OneWireDevice( reg_code );

            if ( OneWireDevice::validate( reg_code ) )
            {
                valid_registration_codes.emplace( reg_code, bus );
                // valid_devices.emplace( reg_code, device );
                // family_devices.emplace( OneWireDevice::family( reg_code ), device );

                ESP_LOGD( TAG, "::scanDevices - Found valid device on bus: %s", bus->info() );    //, device->info() );
            }
//            else
//            {
//                ESP_LOGW( TAG,
//                        "::scanDevices - Invalid device registration code 0x%02X%02X%02X%02X%02X%02X%02X%02X found on bus: %s",
//                        device->getByte( 0 ), device->getByte( 1 ), device->getByte( 2 ), device->getByte( 3 ),
//                        device->getByte( 4 ), device->getByte( 5 ), device->getByte( 6 ), device->getByte( 7 ),
//                        bus->info() );
//            }
        }        // for ( auto code : codes )
    }    // for ( auto bus : m_busses )

    ESP_LOGD( TAG, "::scanDevices - found %d valid devices", valid_devices.size() );

    m_registration_codes.swap( valid_registration_codes );    // Device registration codes and their bus
    //  m_valid_devices.swap( valid_devices );    // Valid devices by Registration Code
    //   m_family_devices.swap( family_devices );    // Valid devices by Family Code

    ESP_LOGD( TAG, "::scanDevices - End" );
    return result;
}

//bool OneWire::verifyDevice( OneWireDevice device )
//{
//    if ( !device.isValid() ) return false;
//    onewire_search_state_t search_point( device.getRegistrationCode() );
//    if ( !_searchAll( search_point ) ) return false;
//
//    return ( device.getRegistrationCode() == search_point.getRegistrationCode() );
//}

//bool OneWire::_searchAll( onewire_search_state_t& search_point )
//{
//    for ( auto bus : m_busses )
//    {
//        if ( !_search( bus, search_point ) )
//        {
//            return false;
//        }
//    }
//    return true;
//}
//
//bool OneWire::_search( OneWireBus* bus, onewire_search_state_t& search_point )
//{
//    if ( !bus->bus_reset() )
//    /*
//     * Cant reset the bus so break out from this bus
//     */
//    {
//        ESP_LOGW( TAG, "::_search - Failed to reset bus: %s", bus->info() );
//        return false;
//    }
//
//    if ( !bus->search_rom( search_point ) )
//    /*
//     * Search this bus from the given location
//     */
//    {
//        ESP_LOGW( TAG, "::_search - Failed to search bus: %s", bus->info() );
//        return false;
//    }
//
//    return true;
//}

//OneWireDevice* OneWire::_validate_and_add_new_device( uint64_t registration_code, OneWireBus* bus )
//{
//    OneWireDevice* device = new OneWireDevice( registration_code );
//
//    if ( !device->isValid()    // Not a valid reg code
//    || !m_registration_codes.emplace( registration_code, bus ).second )    // Already known
//        return nullptr;
//
//    /*
//     * Valid and not known, so add to remaining stores
//     */
//    m_valid_devices.emplace( registration_code, device );
//    m_family_devices.emplace( device->getFamily(), device );
//
//    return device;
//}

