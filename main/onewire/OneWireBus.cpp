/*
 * OneWireBus.cpp
 *
 * Driver definition for Dallas 1-Wire Bus protocol
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

#include "OneWireBus.h"

using namespace epd;

bool OneWireBus::bus_reset()
{
    ESP_LOGD( TAG, "::bus_reset - Start" );
    if ( xSemaphoreTake(m_aquire_bus_mutex, 0) == pdFALSE ) return false;

    m_reset = true;
    m_presence = false;
    onewire_pulses_t pulses;
    bool reset { _bus_reset( pulses ) };

    ESP_LOGD( TAG, "::bus_reset - Pulses: %d ( expecting 4 )", pulses.size() );
    if ( reset && pulses.size() > 0 )
    /*
     * Examine pulses. Optionally may start with the presence pulse.
     * Should be two at least (high, low) if devices are present.
     */
    {
        uint8_t pulse_offset { 0 };

        if ( pulses [ 0 ] > ( OW_RESET_TIME_US - 25 ) && pulses [ 0 ] < ( OW_RESET_TIME_US + 25 ) )    // Master reset found
        pulse_offset++;

        if ( ( 0 - pulses [ pulse_offset ] ) >= OW_PRESENCE_PULSE_HIGH_MIN_US    // Pulled high (will be a -ve #)
        && ( 0 - pulses [ pulse_offset ] ) <= OW_PRESENCE_PULSE_HIGH_MAX_US    // Pulled high (will be a -ve #)
        && pulses [ pulse_offset + 1 ] >= OW_PRESENCE_PULSE_LOW_MIN_US
                && pulses [ pulse_offset + 1 ] <= OW_PRESENCE_PULSE_LOW_MAX_US )    // Presence pulse - low
        /*
         * Presence pulses found
         */
        {
            m_presence = true;
            if ( pulse_offset )
            /*
             * Master reset signal found, along with a presence pulses
             * so we can calculate adaptive timing
             */
            {
                m_adaptive_min = pulses [ 1 ];
                m_adaptive_max = ( pulses [ 1 ] + pulses [ 2 ] ) / 5;
            }
        }
    }

    xSemaphoreGive( m_aquire_bus_mutex );
    ESP_LOGD( TAG, "::bus_reset - End. Presence: %d", m_presence );
    return m_presence;
}    // bus_reset

bool OneWireBus::match_rom( uint64_t address, uint8_t func )
{
    ESP_LOGD( TAG, "::match_rom - Start" );

    if ( !m_reset ) return false;
    if ( xSemaphoreTake(m_aquire_bus_mutex, 0) == pdFALSE ) return false;

    m_reset = false;

    onewire_data_t MatchRom = { 0x55 };
    MatchRom.insert( MatchRom.end(), address );
    onewire_data_t data;

    _write_slots( MatchRom );
    _read_slots( 1, data );

    xSemaphoreGive( m_aquire_bus_mutex );
    ESP_LOGD( TAG, "::match_rom - End" );
    return true;
}    // match_rom

bool OneWireBus::read_rom()
{
    ESP_LOGD( TAG, "::read_rom - Start" );

    if ( !m_reset ) return false;
    if ( xSemaphoreTake(m_aquire_bus_mutex, 0) == pdFALSE ) return false;

    static onewire_data_t ReadRom = { 0x33 };

    m_reset = false;
    onewire_data_t data;

    _write_slots( ReadRom );
    _read_slots( 64, data );
    if ( data.size() == 8 )
    /*
     * 64 bits returned
     */
    {
        // registerDevice(*(uint64_t*) data.data());    // FIXME
    }
    //OneWireDevice d( this, data );

//    ESP_LOGI( TAG, "CRC 0x%02X  ADDR 0x%02X%02X%02X%02X%02X%02X  FAM 0x%02X  V %d", d.crc, d.address [ 0 ],
//            d.address [ 1 ], d.address [ 2 ], d.address [ 3 ], d.address [ 4 ], d.address [ 5 ], d.family, d.valid );

    ESP_LOGI( TAG, "RR Adr 0x%02X 0x%02X%02X%02X%02X%02X%02X 0x%02X", data [ 0 ], data [ 1 ], data [ 2 ], data [ 3 ],
            data [ 4 ], data [ 5 ], data [ 6 ], data [ 7 ] );

    xSemaphoreGive( m_aquire_bus_mutex );
    ESP_LOGD( TAG, "::read_rom - End" );
    return true;
}    // read_rom

bool OneWireBus::search_rom( onewire_search_state_t& search_state )
{
    ESP_LOGD( TAG, "::search_rom - Start" );

    if ( !m_reset ) return false;
    if ( xSemaphoreTake(m_aquire_bus_mutex, 0) == pdFALSE ) return false;

    static onewire_data_t SearchRom = { 0xF0 };

    m_reset = false;
    onewire_data_t data { 0 };
    uint8_t id_bit_number { 1 }, last_zero { 0 }, rom_byte_mask { 1 }, rom_byte_number { 0 };
    bool search_direction;
    search_state.found = false;

    _write_slots( SearchRom );

    do
    /*
     * Loop through ROM bytes
     */
    {
        _read_slots( 2, data );    //  id_bit = 0 position, cmp_bit = 1 position

        if ( data [ 0 ] == 3 )
        /*
         * ( id_bit == 1 ) && ( cmp_id_bit == 1 )
         * No devices on the bus, break out of the loop
         */
        {
            ESP_LOGW( TAG, "::search_rom - No devices found on bus %s = %d", info(), data [ 0 ] );
            search_state.LastDeviceFlag = true;
            break;
        }

        /*
         * Default to all devices coupled have 0 or 1
         * and set search direction to id_bit
         */
        search_direction = data [ 0 ] & 0x01;

        if ( data [ 0 ] == 0 )
        /*
         * ( id_bit == cmp_id_bit ) - Discrepancy
         *
         * Recalculate the search direction
         */
        {
            if ( id_bit_number < search_state.LastDiscrepancy )
            /*
             * if this discrepancy is before the Last Discrepancy
             * on a previous next then pick the same as last time
             */
            {
                search_direction = ( ( search_state.ROM_NO [ rom_byte_number ] & rom_byte_mask ) > 0 );
            }
            else
            /*
             * if equal to last pick 1, if not then pick 0
             */
            {
                search_direction = ( id_bit_number == search_state.LastDiscrepancy );
            }

            if ( !search_direction )
            /*
             * if 0 (false) was picked then record its position in LastZero
             */
            {
                last_zero = id_bit_number;
                if ( last_zero < 9 )
                /*
                 * check for Last discrepancy in family
                 */
                {
                    search_state.LastFamilyDiscrepancy = last_zero;
                }
            }    //if ( !search_direction )
        }    // if ( data [ 0 ] == 3 )

        if ( search_direction )
        /*
         * set or clear the bit in the ROM byte rom_byte_number
         * with mask rom_byte_mask
         */
        {
            search_state.ROM_NO [ rom_byte_number ] |= rom_byte_mask;
        }
        else
        {
            search_state.ROM_NO [ rom_byte_number ] &= ~rom_byte_mask;
        }

        /*
         * increment the byte counter id_bit_number
         * and shift the mask rom_byte_mask
         */
        id_bit_number++;
        rom_byte_mask <<= 1;

        if ( rom_byte_mask == 0 )
        /*
         * if the mask is 0 then go to new SerialNum byte rom_byte_number
         * and reset mask
         */
        {
            rom_byte_number++;
            rom_byte_mask = 1;
        }

        ESP_LOGV( TAG, "::search_rom - Search direction: %d", search_direction );
        data [ 0 ] = 0;
        _write_slots( search_direction );    // serial number search direction write bit
    }
    while ( rom_byte_number < 8 );    // loop until through all ROM bytes 0-7

    if ( id_bit_number > 64 )
    /*
     * Search was successful
     * set  LastDiscrepancy, LastDeviceFlag, search_result
     * and check for last device
     */
    {
        ESP_LOGD( TAG, "::search_rom - Found device" );
        ESP_LOG_BUFFER_HEX_LEVEL( TAG, &search_state.ROM_NO, 8, ESP_LOG_DEBUG );
        search_state.LastDiscrepancy = last_zero;
        if ( search_state.LastDiscrepancy == 0 ) search_state.LastDeviceFlag = true;
        search_state.found = true;
    }

    xSemaphoreGive( m_aquire_bus_mutex );

    ESP_LOGD( TAG, "::search_rom - End" );
    return true;

}    // search_rom

bool OneWireBus::skip_rom()
{
    ESP_LOGD( TAG, "::skip_rom - Start" );

    if ( !m_reset ) return false;
    if ( xSemaphoreTake(m_aquire_bus_mutex, 0) == pdFALSE ) return false;

    m_reset = false;
    onewire_data_t cmd { 0xCC };
    // onewire_data_t data;

    _write_slots( cmd );
    // _read_slots( 1, data );

    xSemaphoreGive( m_aquire_bus_mutex );

    ESP_LOGD( TAG, "::skip_rom - End" );
    return true;
}    // skip_rom

bool OneWireBus::_write_slots( const onewire_data_t& data )
{
    return _write_slots( data.size() * 8, data );    // Write to implementation
}    // _write_slots

bool OneWireBus::_write_slots( bool data )
{
    if ( data ) return _write_slots( OW_DATA_ONE.size(), OW_DATA_ONE );    // Write to implementation

    return _write_slots( OW_DATA_ZERO.size(), OW_DATA_ZERO );    // Write to implementation
}    // _write_slots

void OneWireBus::_process_pulse( onewire_pulses_t& pulses, bool level_high, uint16_t duration )
{
    if ( ( level_high && pulses.back() > 0 )    // High going into a low duration,
    || ( !level_high && pulses.back() < 0 ) )    // Low going into high duration
        /*
         * Check for change of level, create new pulse if so, otherwise
         * the timing will accumulate
         */
        pulses.push_back( 0 );    // Add new pulse

    if ( level_high )
    /*
     * Line pulled high is indicated by a negative duration
     */
    {
        pulses.back() -= duration;
    }
    else
    /*
     * Line pulled low is indicated by a positive duration
     */
    {
        pulses.back() += duration;
    }
}    // _process_pulse

uint16_t OneWireBus::_unmarshal_pulses( onewire_pulses_t& pulses, onewire_data_t& data, unmarshal_behavior_t behavior )
{

    if ( pulses [ 0 ] < 0 )
    /*
     * First pulse is line high.
     */
    {
        switch ( behavior )
        {
            case DISCARD_HIGH:
                /*
                 * Discard as we want to start with line-low (+ve) followed by line-high (-ve) durations
                 */
                ESP_LOGW( TAG, "::_unmarshal_pulses - Discarding first pulse - High." );
                pulses.erase( pulses.begin() );
                break;
            case MAKE_ZERO:
                /*
                 * Insert nominal low vale, meaning the pulse will read as a zero
                 */
                ESP_LOGW( TAG, "::_unmarshal_pulses - Augmenting to 0 - first pulse High." );
                pulses.insert( pulses.begin(), OW_READ_SLOT_LOW_US );
                break;
            case MAKE_ONE:
                /*
                 * Insert read low vale, meaning the pulse will read as a one
                 */
                ESP_LOGW( TAG, "::_unmarshal_pulses - Augmenting to 1 - first pulse High." );
                pulses.insert( pulses.begin(), OW_READ_DATA_US );
                break;
            case FAIL:
            default:
                ESP_LOGE( TAG, "::_unmarshal_pulses - Failing on first pulse High." );
                return 0;
        }
    }

    if ( ( pulses.size() % 2 ) == 1 )
    /*
     * Odd # of pulses. Note and pad with last pulse being high
     */
    {
        ESP_LOGW( TAG, "::_unmarshal_pulses - Odd number of pulses present for unmarshalling." );
        pulses.insert( pulses.end(), -9999 );
    }

    uint8_t bit { 0 }, byte { 0 };

    for ( uint16_t i = 0; i < pulses.size(); i += 2 )
    /*
     * Cycle through pulse pairs
     */
    {
        bit = ( i / 2 ) % 8;
        if ( bit == 0 )
        /*
         * New byte
         */
        {
            byte = ( i / 16 );
            data.push_back( 0 );
        }

        if ( pulses [ i ] <= OW_READ_DATA_US )
        /*
         * One
         */
        {
            data [ byte ] |= ( 0x01 << bit );    // LSB-MSB bit translation
        }

        if ( pulses [ i + 1 ] >= 0    // Check for line low, or no more pulses
        || ( OW_SLOT_TIME_US + OW_RECOVERY_TIME_US ) < ( pulses [ i ] - pulses [ i + 1 ] ) )    // total duration
        /*
         * Indicates no more level changes or an pulse error as expecting
         * a -ve duration indicating line gone high
         * or total low+high time > slot time
         */
        {
            break;
        }
    }    // for

    return ( ( ( bit + 1 ) % 8 ) + ( byte * 8 ) );

}    // _unmarshal_pulses

/* -----------------------------------------------------------------------------
 * Helpers
 * -----------------------------------------------------------------------------
 */

bool OneWireBus::getPresence()
{
    return m_presence;
}    // getPresence

uint16_t OneWireBus::getAdaptiveMin()
{
    return m_adaptive_min;
}    // getAdaptiveMin

uint16_t OneWireBus::getAdaptiveMax()
{
    return m_adaptive_max;
}    // getAdaptiveMax

bool OneWireBus::scanRegistrationCodes()
{
    ESP_LOGD( TAG, "::scanRegistrationCodes - Start\n\tBus %s", info() );

    std::vector< uint64_t > registration_codes;
    OneWireBus::onewire_search_state_t search_point;

    do
    {
        if ( !bus_reset() )
        /*
         * Could not reset the bus
         */
        {
            ESP_LOGW( TAG, "::scanRegistrationCodes - Reset Bus failed" );
            return false;
        }

        if ( search_rom( search_point ) == false )
        /*
         * Search ROM command failed, note failure, go to next bus
         */
        {
            ESP_LOGD( TAG, "::scanDevices - Search ROM failed" );
            return false;
        }

        if ( search_point.found ) registration_codes.push_back( search_point.getRegistrationCode() );
    }
    while ( !search_point.LastDeviceFlag );

    m_registration_codes.swap( registration_codes );

    ESP_LOGD( TAG, "::scanRegistrationCodes - End\n\tBus %s", info() );
    return true;
}    // scanRegistrationCodes

const std::vector< uint64_t >& OneWireBus::getRegistrationCodes()
{
    return m_registration_codes;
}    // getRegistrationCodes

bool OneWireBus::writeAndRead( onewire_data_t& to_wire, uint16_t bits_to_read, onewire_data_t& from_wire )
{
    ESP_LOGD( TAG, "::writeAndRead - Start" );

    if ( xSemaphoreTake(m_aquire_bus_mutex, 0) == pdFALSE ) return false;

    m_reset = false;

    _write_slots( to_wire );
    _read_slots( bits_to_read, from_wire );

    xSemaphoreGive( m_aquire_bus_mutex );

    ESP_LOGD( TAG, "::writeAndRead - End" );
    return true;
}
// writeAndRead

