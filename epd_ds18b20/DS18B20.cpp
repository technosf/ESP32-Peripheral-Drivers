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

std::unordered_map< OneWireBus*, DS18B20::PWR_SPLY > DS18B20::BUS_POWER; 
std::unordered_map< onewire_rom_code_t, DS18B20* > DS18B20::DEVICE_ROM_CODES_BUS;



DS18B20::DS18B20( onewire_rom_code_t romcode, OneWireBus* bus ) :
        OneWireDevice::OneWireDevice( romcode )
{
    if ( FAMILY == getFamily() )
    {
        p_bus = bus;
    }
    else
    {
        ESP_LOGE( TAG, "Non-DB18B20 instantiated - %s", info() );
    }

}

DS18B20::~DS18B20()
{
    DEVICE_ROM_CODES_BUS.erase(getRomCode());
}


std::vector< DS18B20* > DS18B20::getDevices( OneWireBus* bus, bool autosearch )
{
    ESP_LOGD( TAG, "::getDevices - Start" );

    std::vector< DS18B20* > devices;
    if ( bus && ( bus->isSearched() || ( !bus->isSearched() && autosearch && bus->cmdSearchRom() ) ) )
    {
        
        for ( onewire_rom_code_t romcode : bus->getRomCodes() )
        {
            if ( FAMILY == family(romcode) ) 
            {  
                DS18B20* obj = new DS18B20( romcode, bus );;
                DEVICE_ROM_CODES_BUS.emplace( romcode, obj );
                devices.push_back(obj);
            }
        }
    }

    ESP_LOGD( TAG, "::getDevices - End. Count %d", devices.size() );

    return devices;
}    // getDevices


/* --------------------------------------------------------
*
* High Level Functions
*
* --------------------------------------------------------
*/


DS18B20::scratchpad_t DS18B20::getScratchpad()
{
    return m_scratchpad;
}

bool DS18B20::cmdConvertT( bool thisdevice )
{
    ESP_LOGD( TAG, "::cmdConvertT - Start. This device %d", thisdevice );

    if ( ( thisdevice && !p_bus->cmdAddressDevice( getRomCode() )  )   // this device and cannot address it
        ||  ( !thisdevice && !p_bus->cmd_skip_rom() )                  // not this device and cannot skip rom
        ) return false;

    bool result = cmd_convert_t( p_bus );

    ESP_LOGD( TAG, "::cmdConvertT - End. " );

    return result;    
} // cmdConvertT


bool DS18B20::cmdReadScratchpad()
{
    return ( p_bus->cmdAddressDevice( getRomCode() ) && cmd_read_scratchpad( p_bus, m_scratchpad ) );
}

DS18B20::PWR_SPLY DS18B20::cmdReadPowerSupply()     
{
    ESP_LOGD( TAG, "::cmdReadPowerSupply - Start. " );

    PWR_SPLY result { PWR_SPLY::UNKNOWN };

    if ( !DS18B20::BUS_POWER[p_bus] && p_bus->cmd_skip_rom() )
    // Determine nominal bus power supply
    {
        result = cmd_read_power_supply( p_bus );
        DS18B20::BUS_POWER.emplace( p_bus, result);
    }
    else
    {
        result = DS18B20::BUS_POWER[p_bus];
    }

    if ( !m_power_supply && result ) 
    // Do not know device PS, but know nominal bus PS
    {
        if ( PWR_SPLY::PARASITIC == result && p_bus->cmdAddressDevice( getRomCode() ) )
        // Parasitic devices on bus, is this device one?
        {
            result = cmd_read_power_supply(p_bus);
        } 
        else if ( PWR_SPLY::PARASITIC == result )
        {
              result = PWR_SPLY::UNKNOWN;     // Could not address device
        }
        m_power_supply = result;
    }

    ESP_LOGD( TAG, "::cmdReadPowerSupply - End. " );

    return result;
} // cmdReadPowerSupply


bool DS18B20::cmdAlarmSearch( std::vector< onewire_rom_code_t >& rom_codes )
{
    ESP_LOGD( TAG, "::cmdAlarmSearch - Start\n\tBus %s", info() );

    OneWireBus::onewire_search_state_t search_point;

    do
    {
        if ( cmd_alarm_search( p_bus, search_point ) == false )
        /*
         * Search ROM command failed, note failure, go to next bus
         */
        {
            ESP_LOGD( TAG, "::cmdAlarmSearch - Alarm Search failed" );
            return false;
        }

        if ( search_point.found ) rom_codes.push_back( search_point.getRegistrationCode());
    }
    while ( !search_point.LastDeviceFlag );

    ESP_LOGD( TAG, "::cmdAlarmSearch - End\n\tFound %d devices.", rom_codes.size() );

    return true;
} // cmdAlarmSearch

/* --------------------------------------------------------
*
* Helpers
*
* --------------------------------------------------------
*/

uint8_t DS18B20::getResolution()
{
    return m_scratchpad.getResolution();
}

float DS18B20::getTemperature()
{
    return m_scratchpad.getTemperature();
}

/* --------------------------------------------------------
 *
 * OneWire DS18B20 Commands
 *
 * --------------------------------------------------------
 */

bool DS18B20::cmd_convert_t( OneWireBus* bus )
{
    ESP_LOGI( TAG, "::cmd_convert_t - Start" );

    if ( !bus->wireWrite( 0x44 ) ) return false;
    bus->wireReadUntilOne(32);                      // Read 32 slots at interval until 1 or end
    
    // TODO Parasitic

    ESP_LOGD( TAG, "::cmd_convert_t - End" );
    return true;

}    // cmd_convert_t


bool DS18B20::cmd_read_scratchpad( OneWireBus* bus, scratchpad_t& scratchpad )
{
    ESP_LOGD( TAG, "::cmd_read_scratchpad - Start" );
    
    onewire_data_t data;

    if ( !bus->wireWriteAndRead( 0xBE, 9*8, data ) ) return false;

    scratchpad.copy(data);

    ESP_LOGD( TAG, "::cmd_read_scratchpad - End" );
    return true;  
}    // cmd_read_scratchpad



bool DS18B20::cmd_write_scratchpad( OneWireBus* bus, uint8_t th, uint8_t tl, uint8_t config )
{
    ESP_LOGD( TAG, "::cmd_write_scratchpad" );

    onewire_data_t data { 0x4E, th, tl, config };
    return bus->wireWriteAndRead( data, 0, data );

}    // cmd_write_scratchpad



bool DS18B20::cmd_copy_scratchpad( OneWireBus* bus )
{
    ESP_LOGD( TAG, "::cmd_copy_scratchpad" );

    return  ( bus->wireWrite( 0x4E ) &&  bus->wireReadUntilOne(32) );

}    // cmd_copy_scratchpad



bool DS18B20::cmd_recall_e2( OneWireBus* bus )
{
    ESP_LOGD( TAG, "::cmd_recall_e2" );

    return  ( bus->wireWrite( 0xB8 ) &&  bus->wireReadUntilOne(32) );

}    // cmd_recall_e2



/*
 * Start with unknown power state and determine bus or parasitic
 */
DS18B20::PWR_SPLY DS18B20::cmd_read_power_supply( OneWireBus* bus )
{
    ESP_LOGD( TAG, "::read_power_supply - Start" );

    onewire_data_t data { 0 };
    PWR_SPLY result { PWR_SPLY::UNKNOWN };

    if ( bus->wireWriteAndRead( 0xB4, 1, data ) )
    {
        result = PWR_SPLY::BUS;
        if ( data [ 0 ] == 0 ) result = PWR_SPLY::PARASITIC;
    }

    ESP_LOGD( TAG, "::read_power_supply - End" );
    return result;
}    // read_power_supply


bool DS18B20::cmd_alarm_search( OneWireBus* bus, OneWireBus::onewire_search_state_t& search_state, bool autoreset )
{
    ESP_LOGD( TAG, "::cmd_alarm_search" );
    return bus->busSearch( 0xEC, search_state, autoreset );
}    // cmd_search_rom


/* --------------------------------------------------------
 *
 * scratchpad_t
 *
 * --------------------------------------------------------
 */


void DS18B20::scratchpad_t::copy(onewire_data_t& data)
{
    memcpy( &raw, data.data(), 9 );
}

uint8_t DS18B20::scratchpad_t::getResolution()
{
    return eeprom.CONFIG >> 5;
}

float DS18B20::scratchpad_t::getTemperature()
{
    return RESOLUTION[ eeprom.CONFIG >> 5 ] * ( eeprom.TEMP_LSB + ( 256 * eeprom.TEMP_MSB ) );
}

bool DS18B20::scratchpad_t::isValid()
{
    return validate( raw, 9 );
}