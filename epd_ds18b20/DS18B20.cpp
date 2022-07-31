/**
 *  @file       DS18B20.cpp
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

#include "DS18B20.h"

using namespace epd;

// std::unordered_map< OneWireBus*, DS18B20::PWR_SRC > DS18B20::BUS_POWER;
std::unordered_map< onewire_rom_code_t, DS18B20 > DS18B20::DEVICE_ROM_CODES_BUS;
//std::unordered_map< DS18B20*, OneWireBus* > DS18B20::DEVICE_BUSSES;
//std::vector< DS18B20 > DEVICE_ROM_CODES_BUS;


DS18B20::DS18B20( onewire_rom_code_t reg, OneWireBus* bus ) :
        OneWireDevice::OneWireDevice( reg )
{
    if ( FAMILY == getFamily() )
    {
        m_bus = bus;
    }
    else
    {
        ESP_LOGE( TAG, "Non-DB18B20 instantiated - %s", info() );
    }

}

DS18B20::~DS18B20()
{
}


// bool DS18B20::identify( OneWireBus* bus )
// {
//     ESP_LOGD( TAG, "::identify - Start" );

//     for ( auto& code : bus->getRomCodes() )
//     {
//         if ( !OneWireDevice::validate( code ) )
//         {
//             ESP_LOGD( TAG, "::identify - Invalid Reg Code" );
//             ESP_LOG_BUFFER_HEX_LEVEL( TAG, &code, 8, ESP_LOG_DEBUG );
//             break;
//         }

//         DS18B20* ds18b20 = DS18B20::DEVICE_CODES [ code ];
//         if ( ds18b20 != nullptr )
//         /*
//          * Clear previous entry for this reg code
//          */
//         {
//             DS18B20::DEVICE_CODES.erase( code );
//             DS18B20::DEVICE_BUSSES.erase( ds18b20 );
//            // DS18B20::DEVICE_POWER.erase( ds18b20 );
//         }
//         //  OneWireDevice device( code );

//         if ( FAMILY == OneWireDevice::family( code ) )
//         /*
//          * Only collate devices in our family
//          */
//         {
//             ds18b20 = new DS18B20( code, bus );
//             DS18B20::DEVICE_CODES.emplace( code, ds18b20 );
//             DS18B20::DEVICE_BUSSES.emplace( ds18b20, bus );
//            // DS18B20::BUS_POWER.emplace( ds18b20, PWR_SRC::UNKNOWN );
//             //DS18B20::BUSSES.emplace( bus );
//             ESP_LOGD( TAG, "::identify - Found %s", ds18b20->info() );
//         }
//     }    // for

//     ESP_LOGD( TAG, "::identify - End" );
//     return true;
// }    // identify


// std::vector< DS18B20* > DS18B20::getDevices()
// {
//     std::vector< DS18B20* > devices;
//     for ( auto& codedevice : DS18B20::DEVICE_CODES )
//     {
//         devices.push_back( codedevice.second );
//     }
//     return devices;
// }    // getDevices


std::vector< DS18B20* > DS18B20::getDevices( OneWireBus* bus, bool autosearch )
{
    std::vector< DS18B20* > devices;
    if ( bus && ( bus->isSearched() || ( !bus->isSearched() && autosearch && bus->searchRomCodes() ) ) )
    {
        ESP_LOGD( TAG, "::getDevices - Start" );
        
        for ( onewire_rom_code_t romcode : bus->getRomCodes() )
        {
            if ( FAMILY == family(romcode) ) 
            {  
                DEVICE_ROM_CODES_BUS.insert( std::pair<onewire_rom_code_t, DS18B20>(romcode, DS18B20 { romcode, bus } ) );
                devices.push_back(&DEVICE_ROM_CODES_BUS.at(romcode));
            }
        }
        ESP_LOGD( TAG, "::getDevices - End. Count %d", devices.size() );
    }
    return devices;
}    // getDevices


/* --------------------------------------------------------
 *
 * OneWire DS18B20 Commands
 *
 * --------------------------------------------------------
 */

bool DS18B20::convert_t( OneWireBus* bus )
{
    ESP_LOGI( TAG, "::convert_t - Start" );

    //onewire_data_t ConvertT { 0x44 };
    //onewire_data_t data { 0 };

    //if ( !bus->writeAndRead( ConvertT, 8, data ) ) return false;
    if ( !bus->write( 0x44 ) ) return false;
    bus->readUntilOne(128);

    ESP_LOGD( TAG, "::convert_t - End" );
    return true;

}    // convert_t


bool DS18B20::read_scratchpad( OneWireBus* bus, scratchpad_t& scratchpad )
{
    ESP_LOGD( TAG, "::read_scratchpad - Start" );
    //onewire_data_t ReadScratchpad { 0xBE };
    onewire_data_t data { 0 };

    if ( !bus->writeAndRead( 0xBE, 9*8, data ) ) return false;
   std::copy(data.begin(), data.end(), scratchpad.raw);
    //scratchpad = (scratchpad_t)data.data();

    ESP_LOG_BUFFER_HEX_LEVEL( TAG, &scratchpad, 9 , ESP_LOG_VERBOSE );
	//printf( "\nRS\n\t%02X %02X %02X %02X\n", data.data()[0], data.data()[1], data.data()[8],data.data()[9] );

    ESP_LOGD( TAG, "::read_scratchpad - End" );
    return true;    // in 0.0625C increments
}    // read_scratchpad


// // TODO
// bool DS18B20::write_scratchpad( OneWireBus* bus )
// {
//     ESP_LOGD( TAG, "::write_scratchpad - Start" );
//     ESP_LOGD( TAG, "::write_scratchpad - End" );
//     return true;    // in 0.0625C increments
// }    // write_scratchpad


// // TODO
// bool DS18B20::copy_scratchpad( OneWireBus* bus )
// {
//     ESP_LOGD( TAG, "::copy_scratchpad - Start" );
//     ESP_LOGD( TAG, "::copy_scratchpad - End" );
//     return true;    // in 0.0625C increments
// }    // copy_scratchpad


// // TODO
// bool DS18B20::recall_e2( OneWireBus* bus )
// {
//     ESP_LOGD( TAG, "::recall_e2 - Start" );
//     ESP_LOGD( TAG, "::recall_e2 - End" );
//     return true;    // in 0.0625C increments
// }    // recall_e2


// /*
//  * Start with unknown power state and determine bus or parasitic
//  */
// DS18B20::PWR_SRC DS18B20::read_power_supply( OneWireBus* bus )
// {
//     static onewire_data_t ReadPowerSupply = { 0xB4 };

//     ESP_LOGD( TAG, "::read_power_supply - Start" );

//     onewire_data_t data { 0 };
//     PWR_SRC result { PWR_SRC::UNKNOWN };

//     if ( bus->writeAndRead( ReadPowerSupply, 1, data ) )
//     {
//         result = PWR_SRC::BUS;
//         if ( data [ 0 ] == 0 ) result = PWR_SRC::PARASITIC;
//     }

//     ESP_LOGD( TAG, "::read_power_supply - End" );
//     return result;
// }    // read_power_supply
