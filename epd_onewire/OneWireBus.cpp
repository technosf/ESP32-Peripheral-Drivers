/**
 *  @file       OneWireBus.cpp
 *  @version    0.1.0
 *
 *  @brief      Driver definition for Dallas 1-Wire Bus protocol
 *
 *  @date       Jul 20, 2019
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


#include "OneWireBus.h"

using namespace epd;


bool OneWireBus::initialize()
{
    if ( _initialize() ) return searchRomCodes();
    return false;
}


bool OneWireBus::bus_reset()
{    
    if ( !_bus_guard() ) return false;

    ESP_LOGD( TAG, "::bus_reset - Start" );

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
            if ( pulse_offset 
            &&  ( m_adaptive_fastest_read != -pulses [ 1 ] ) 
            &&  ( m_adaptive_slowest_write != ( ( m_adaptive_fastest_read + pulses [ 2 ] ) / 5 ) )
            )
            /*
             * Master reset signal found, along with a presence pulses
             * so we can calculate adaptive timing
             */
            {
                m_adaptive_fastest_read = -pulses [ 1 ];  // Tpdh (Pin is high, so represented as a -ve value in pulses)
                m_adaptive_slowest_write = ( m_adaptive_fastest_read + pulses [ 2 ] ) / 5;
                if ( NONE != m_adaptive_timing )_set_adaptive_timing( m_adaptive_timing );
                ESP_LOGD( TAG, "::bus_reset - Adaptive timing fastest-read/Tpdh: %u, Tpdl: %u  slowest write: %u", m_adaptive_fastest_read, pulses [ 2 ], m_adaptive_slowest_write );
            }
        }
    }

    _bus_release( STATUS::RESET );

    ESP_LOGD( TAG, "::bus_reset - End. Presence: %d", m_presence );

    return m_presence;
}    // bus_reset


bool OneWireBus::match_rom( uint64_t address, bool autoreset )
{
    if ( !_bus_guard_and_reset() ) return false;

    ESP_LOGD( TAG, "::match_rom - Start" );

    onewire_data_t cmd = { 0x55 };
    cmd.insert( cmd.end(), address );
    onewire_data_t data;

    _write_slots( cmd );
    _read_slots( 1, data );

    _bus_release( STATUS::ADDRESSED );

    ESP_LOGD( TAG, "::match_rom - End" );

    return true;
}    // match_rom


bool OneWireBus::read_rom( onewire_rom_code_t& romcode, bool autoreset )
{
    static onewire_data_t cmd = { 0x33 };

    if ( !_bus_guard_and_reset() ) return false;

    ESP_LOGD( TAG, "::read_rom - Start" );

    onewire_data_t data;

    _write_slots( cmd );
    _read_slots( 64, data );

    if ( data.size() == 8 )     // 64 bits returned
        romcode = (*(onewire_rom_code_t*) data.data());

    //bool valid = OneWireDevice::validate( romcode );    
    
    //OneWireDevice d( this, data );

//    ESP_LOGV( TAG, "CRC 0x%02X  ADDR 0x%02X%02X%02X%02X%02X%02X  FAM 0x%02X  V %d", d.crc, d.address [ 0 ],
//            d.address [ 1 ], d.address [ 2 ], d.address [ 3 ], d.address [ 4 ], d.address [ 5 ], d.family, d.valid );

    ESP_LOGI( TAG, "ROM Code: 0x%02X 0x%02X%02X%02X%02X%02X%02X 0x%02X", data [ 0 ], data [ 1 ], data [ 2 ], data [ 3 ],
            data [ 4 ], data [ 5 ], data [ 6 ], data [ 7 ] );

    _bus_release( STATUS::ADDRESSED );

    ESP_LOGD( TAG, "::read_rom - End" );

    return true;
}    // read_rom


bool OneWireBus::search_rom( onewire_search_state_t& search_state, bool autoreset )
{
    static onewire_data_t cmd = { 0xF0 };

    if ( !_bus_guard_and_reset( autoreset ) ) return false;

    ESP_LOGD( TAG, "::search_rom - Start" );

    onewire_data_t data { 0 };
    uint8_t id_bit_number { 1 }, last_zero { 0 }, rom_byte_mask { 1 }, rom_byte_number { 0 };
    bool search_direction;
    search_state.found = false;

    _write_slots( cmd );

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

    _bus_release( STATUS::ADDRESSED );

    ESP_LOGD( TAG, "::search_rom - End" );

    return true;

}    // search_rom


bool OneWireBus::skip_rom( bool autoreset )
{
    static onewire_data_t cmd { 0xCC };

    if ( !_bus_guard_and_reset( autoreset ) ) return false;

    ESP_LOGD( TAG, "::skip_rom - Start" );

    _write_slots( cmd );

    _bus_release( STATUS::ADDRESSED );

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
    if ( ( level_high && ( 0 < pulses.back() ) )        // High going into a low duration,
    || (  !level_high && ( 0 > pulses.back() ) ) )      // Low going into high duration
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

   // ESP_LOG_BUFFER_HEX_LEVEL( TAG, &pulses, pulses.size(),ESP_LOG_VERBOSE);
    //ESP_LOG_BUFFER_HEX_LEVEL( TAG, &data, data.size(),ESP_LOG_VERBOSE);

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
        || OW_SLOT_RECOVERY_US < ( pulses [ i ] - pulses [ i + 1 ] ) )    // total duration
        /*
         * Indicates no more level changes or an pulse error as expecting
         * a -ve duration indicating line gone high
         * or total low+high time > slot time
         */
        {
            break;
        }
    }    // for

    if ( 8 == ++bit ) byte++;
    return ( ( bit % 8 ) + ( byte * 8 ) );

}    // _unmarshal_pulses


/* -----------------------------------------------------------------------------
 * Helpers
 * -----------------------------------------------------------------------------
 */

bool OneWireBus::getPresence()
{
    return m_presence;
}    // getPresence


uint16_t OneWireBus::getAdaptiveFastestRead()
{
    return m_adaptive_fastest_read;
}    // getAdaptiveFastestRead


uint16_t OneWireBus::getAdaptiveSlowestWrite()
{
    return m_adaptive_slowest_write;
}    // getAdaptiveSlowestWrite


ADAPTIVE_TIMING OneWireBus::useAdaptive( bool setadaptive, ADAPTIVE_TIMING timing )
{
    if ( setadaptive )
    {
        m_adaptive_timing = timing;
        _set_adaptive_timing( timing );
    }

    return m_adaptive_timing;
} // useAdaptive

bool OneWireBus::searchRomCodes()
{
    ESP_LOGD( TAG, "::searchRomCodes - Start\n\tBus %s", info() );

    std::vector< uint64_t > rom_codes;
    OneWireBus::onewire_search_state_t search_point;

    do
    {
        if ( !bus_reset() )
        /*
         * No presence response from bus
         */
        {
            ESP_LOGW( TAG, "::searchRomCodes - Reset Bus failed" );
            return false;
        }

        if ( search_rom( search_point ) == false )
        /*
         * Search ROM command failed, note failure, go to next bus
         */
        {
            ESP_LOGD( TAG, "::searchRomCodes - Search ROM failed" );
            return false;
        }

        if ( search_point.found ) rom_codes.push_back( search_point.getRegistrationCode());
    }
    while ( !search_point.LastDeviceFlag );

    m_rom_codes.swap( rom_codes );

    m_scanned = true;

    ESP_LOGD( TAG, "::searchRomCodes - End\n\tBus type: %s   Found %d devices.", info(), m_rom_codes.size() );

    return true;
} // searchRomCodes


bool OneWireBus::readRomCode()
{
    ESP_LOGD( TAG, "::readRomCode - Start\n\tBus %s", info() );

    onewire_rom_code_t rom_code;

    if ( !bus_reset() )
    /*
        * No presence response from bus
        */
    {
        ESP_LOGW( TAG, "::readRomCode - Reset Bus failed" );
        return false;
    }

    bool result = read_rom(rom_code);

    if (result)
    {
        m_rom_codes.clear();
        m_rom_codes.push_back( rom_code );
        ESP_LOGD( TAG, "::readRomCode - found %lld", rom_code );
    }

    ESP_LOGD( TAG, "::readRomCode - End");

    return result;
}    // readRomCode


const std::vector< uint64_t >& OneWireBus::getRomCodes()
{
    return m_rom_codes;
} // getRegistrationCodes


bool OneWireBus::writeAndRead( onewire_data_t& to_wire, uint16_t bits_to_read, onewire_data_t& from_wire )
{
    bool result { _write_slots( to_wire ) };
    if ( result && bits_to_read ) return _read_slots( bits_to_read, from_wire );
    return result;
} // writeAndRead


bool OneWireBus::writeAndRead( uint8_t to_wire, uint16_t bits_to_read, onewire_data_t& from_wire )
{
    bool result { _write_slots( to_wire ) };
    if ( result && bits_to_read ) return _read_slots( bits_to_read, from_wire );
    return result;
} // writeAndRead


bool OneWireBus::write( uint8_t to_wire )
{
    ESP_LOGD( TAG, "::write - to_wire: %u",to_wire);
    return _write_slots( to_wire );
} // write


bool OneWireBus::readUntilOne( uint8_t maxSlots )
{
    onewire_data_t data;
    for ( uint8_t i = 0; i < maxSlots; i++ )
    {
	    vTaskDelay(100 / portTICK_PERIOD_MS);
        data.clear();
        if ( _read_slots( 1, data ) && data.front() == 1 ) return true;
    }
    return false;
}

bool OneWireBus::isSearched()
{
    return m_scanned;
} // isSearched 


epd::STATUS OneWireBus::getStatus()
{
    return m_status;
} // getStatus


/* -----------------------------------------------------------------------------
 * Private
 * -----------------------------------------------------------------------------
 */

inline bool OneWireBus::_bus_guard_and_reset( bool autoreset, TickType_t xBlockTime ) 
{ 
    return  (   m_status == STATUS::RESET
            ||  (   autoreset 
                &&  bus_reset() 
                )   
            ) 
#ifdef OW_THREADSAFE            
            && ( xSemaphoreTakeRecursive(m_aquire_bus_mutex, xBlockTime) == pdTRUE );
#else
            && true;
#endif             
};


inline bool OneWireBus::_bus_guard( TickType_t xBlockTime) 
{ 
#ifdef OW_THREADSAFE
    return  ( xSemaphoreTakeRecursive(m_aquire_bus_mutex, xBlockTime) == pdTRUE ); 
#else
    return true;
#endif
};


inline void OneWireBus::_bus_release( STATUS state )
{
    m_status = state;

#ifdef OW_THREADSAFE
    xSemaphoreGiveRecursive( m_aquire_bus_mutex );
#endif
}