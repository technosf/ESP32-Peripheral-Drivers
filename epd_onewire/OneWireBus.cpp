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


bool OneWireBus::initialize( bool search )
{
    if ( _initialize() 
        && (    !search 
                || cmdSearchRom() 
        ) 
    ) return true;
    return false;
}


/* --------------------------------------------------------
*
* One Wire Network Layer Commands
*
* --------------------------------------------------------
*/

bool OneWireBus::cmd_bus_reset()
{    
    if ( !busGuard() ) return false;

    ESP_LOGD( TAG, "::cmd_bus_reset - Start" );

    m_presence = false;
    onewire_pulses_t pulses;
    bool reset { _bus_reset( pulses ) };

    ESP_LOGD( TAG, "::cmd_bus_reset - Pulses: %d ( expecting 4 )", pulses.size() );
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
                ESP_LOGD( TAG, "::cmd_bus_reset - Adaptive timing fastest-read/Tpdh: %u, Tpdl: %u  slowest write: %u", m_adaptive_fastest_read, pulses [ 2 ], m_adaptive_slowest_write );
            }
        }
    }

    busRelease( STATUS::RESET );

    ESP_LOGD( TAG, "::cmd_bus_reset - End. Presence: %d", m_presence );

    return m_presence;
}    // cmd_bus_reset


bool OneWireBus::cmd_match_rom( const onewire_rom_code_t& address, bool autoreset )
{
    if ( !_bus_guard_and_reset() ) return false;

    ESP_LOGD( TAG, "::cmd_match_rom - Start" );

    onewire_data_t cmd = { 0x55 };
    cmd.insert( cmd.end(), address );
    onewire_data_t data;

    _write_slots( cmd );
    _read_slots( 1, data );

    busRelease( STATUS::ADDRESSED );

    ESP_LOGD( TAG, "::cmd_match_rom - End" );

    return true;
}    // cmd_match_rom


bool OneWireBus::cmd_read_rom( onewire_rom_code_t& romcode, bool autoreset )
{
    if ( !_bus_guard_and_reset() ) return false;

    ESP_LOGD( TAG, "::cmd_read_rom - Start" );

    onewire_data_t data;

    _write_slots( static_cast<uint8_t>(0x33) );
    _read_slots( 64, data );

    if ( data.size() == 8 )     // 64 bits returned
        romcode = (*(onewire_rom_code_t*) data.data());

    ESP_LOGD( TAG, "ROM Code: 0x%02X 0x%02X%02X%02X%02X%02X%02X 0x%02X", data [ 0 ], data [ 1 ], data [ 2 ], data [ 3 ],
            data [ 4 ], data [ 5 ], data [ 6 ], data [ 7 ] );

    busRelease( STATUS::ADDRESSED );

    ESP_LOGD( TAG, "::cmd_read_rom - End" );

    return true;
}    // cmd_read_rom


bool OneWireBus::cmd_search_rom( onewire_search_state_t& search_state, bool autoreset )
{
    ESP_LOGD( TAG, "::search_rom" );
    return busSearch( 0xF0, search_state, autoreset );
}    // cmd_search_rom


bool OneWireBus::cmd_skip_rom( bool autoreset )
{
    if ( !_bus_guard_and_reset( autoreset ) ) return false;

    ESP_LOGD( TAG, "::cmd_skip_rom - Start" );

    _write_slots( static_cast<uint8_t>(0xCC) );

    busRelease( STATUS::ADDRESSED );

    ESP_LOGD( TAG, "::cmd_skip_rom - End" );

    return true;
}    // cmd_skip_rom




/* --------------------------------------------------------
*
* Command Helper functions
*
* --------------------------------------------------------
*/

inline bool OneWireBus::busGuard( TickType_t xBlockTime) 
{ 
#ifdef OW_THREADSAFE
    return  ( xSemaphoreTakeRecursive(m_aquire_bus_mutex, xBlockTime) == pdTRUE ); 
#else
    return true;
#endif
};


inline void OneWireBus::busRelease( STATUS state )
{
    m_status = state;

#ifdef OW_THREADSAFE
    xSemaphoreGiveRecursive( m_aquire_bus_mutex );
#endif
}


bool OneWireBus::busSearch( uint8_t cmd, OneWireBus::onewire_search_state_t& search_state, bool autoreset )
{
    if ( !_bus_guard_and_reset( autoreset ) ) return false;

    ESP_LOGD( TAG, "::busSearch - Start" );

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
            ESP_LOGW( TAG, "::busSearch - No devices found on bus %s = %d", info(), data [ 0 ] );
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

        ESP_LOGV( TAG, "::busSearch - Search direction: %d", search_direction );
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
        ESP_LOGD( TAG, "::busSearch - Found device" );
        ESP_LOG_BUFFER_HEX_LEVEL( TAG, &search_state.ROM_NO, 8, ESP_LOG_DEBUG );
        search_state.LastDiscrepancy = last_zero;
        if ( search_state.LastDiscrepancy == 0 ) search_state.LastDeviceFlag = true;
        search_state.found = true;
    }

    busRelease( STATUS::ADDRESSED );

    ESP_LOGD( TAG, "::busSearch - End" );

    return true;

}    // busSearch


/* --------------------------------------------------------
*
* Value Added Bus Commands
*
* --------------------------------------------------------
*/


bool OneWireBus::cmdReadRom()
{
    ESP_LOGD( TAG, "::readRomCode - Start\n\tBus %s", info() );

    onewire_rom_code_t rom_code;

    if ( !cmd_bus_reset() )
    /*
    * No presence response from bus
    */
    {
        ESP_LOGW( TAG, "::readRomCode - Reset Bus failed" );
        return false;
    }

    bool result = cmd_read_rom(rom_code);

    if (result)
    {
        m_rom_codes.clear();
        m_rom_codes.push_back( rom_code );
        ESP_LOGD( TAG, "::readRomCode - found %lld", rom_code );
    }

    ESP_LOGD( TAG, "::readRomCode - End");

    return result;
}    // cmdReadRom


bool OneWireBus::cmdSearchRom()
{
    ESP_LOGD( TAG, "::cmdSearchRom - Start\n\tBus %s", info() );

    std::vector< onewire_rom_code_t > rom_codes;
    OneWireBus::onewire_search_state_t search_point;

    do
    {
        if ( !cmd_bus_reset() ) 
        /*
         * Poss first reset, so directly assess presence response from bus
         */
        {
            ESP_LOGW( TAG, "::cmdSearchRom - Reset Bus failed" );
            return false;
        }

        if ( cmd_search_rom( search_point ) == false )
        /*
         * Search ROM command failed, note failure, go to next bus
         */
        {
            ESP_LOGD( TAG, "::cmdSearchRom - Search ROM failed" );
            return false;
        }

        if ( search_point.found ) rom_codes.push_back( search_point.getRegistrationCode());
    }
    while ( !search_point.LastDeviceFlag );

    m_rom_codes.swap( rom_codes );

    m_scanned = true;

    ESP_LOGD( TAG, "::cmdSearchRom - End\n\tBus type: %s   Found %d devices.", info(), m_rom_codes.size() );

    return true;
} // cmdSearchRom


bool OneWireBus::cmdAddressDevice( const onewire_rom_code_t& device_rom_code )
{
    ESP_LOGD( TAG, "::cmdAddressDevice - Start" );

    const auto& it = std::find( m_rom_codes.begin(), m_rom_codes.end(), device_rom_code);
    if ( it == m_rom_codes.end() ) return false;
    if ( 1 == m_rom_codes.size() )
    // One device, Skip Rom
    {
        cmd_skip_rom();
    }
    else
    // Multiple device, Match Rom
    {
        cmd_match_rom( device_rom_code );
    }

    ESP_LOGD( TAG, "::cmdAddressDevice - End" );

    return true;
} // cmdAddressDevice



/* --------------------------------------------------------
*
* Value Added Wire Operations
*
* --------------------------------------------------------
*/


bool OneWireBus::wireWriteAndRead( const onewire_data_t& to_wire, uint16_t bits_to_read, onewire_data_t& from_wire )
{
    ESP_LOGD( TAG, "::wireWriteAndRead - onewire_data_t" );

    bool result { _write_slots( to_wire ) };
    if ( result && bits_to_read ) return _read_slots( bits_to_read, from_wire );
    return result;
} // writeAnwireWriteAndReaddRead


bool OneWireBus::wireWriteAndRead( uint8_t to_wire, uint16_t bits_to_read, onewire_data_t& from_wire )
{
    ESP_LOGD( TAG, "::wireWriteAndRead - uint8_t" );

    bool result { _write_slots( to_wire ) };
    if ( result && bits_to_read ) return _read_slots( bits_to_read, from_wire );
    return result;
} // wireWriteAndRead


bool OneWireBus::wireWrite( uint8_t to_wire )
{
    ESP_LOGD( TAG, "::write - to_wire: %u",to_wire);

    return _write_slots( to_wire );
} // wireWrite


bool OneWireBus::wireReadUntilOne( uint8_t maxSlots )
{
    ESP_LOGD( TAG, "::wireReadUntilOne" );
    
    static onewire_data_t data(1);

    for ( uint8_t i = 0; i <= maxSlots; i++ )
    {
        if ( _read_slots( 1, data ) && data.front() == 1 ) return true;
	    vTaskDelay(100 / portTICK_PERIOD_MS);
        data.clear();
    }
    return false;
} // wireReadUntilOne


/* --------------------------------------------------------
*
* Bus State functions
*
* --------------------------------------------------------
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


OneWireBus::ADAPTIVE_TIMING OneWireBus::useAdaptive( bool setadaptive, ADAPTIVE_TIMING timing )
{
    if ( setadaptive )
    {
        m_adaptive_timing = timing;
        _set_adaptive_timing( timing );
    }

    return m_adaptive_timing;
} // useAdaptive


const std::vector< onewire_rom_code_t >& OneWireBus::getRomCodes()
{
    return m_rom_codes;
} // getRegistrationCodes



bool OneWireBus::isSearched()
{
    return m_scanned;
} // isSearched 


epd::OneWireBus::STATUS OneWireBus::getStatus()
{
    return m_status;
} // getStatus


/* --------------------------------------------------------
*
* Implementations
*
* --------------------------------------------------------
*/

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


uint16_t OneWireBus::_unmarshal_pulses( onewire_pulses_t& pulses, onewire_data_t& data, UNMARSHAL_BEHAVIOR behavior )
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
 *
 * Private
 * 
 * -----------------------------------------------------------------------------
 */

inline bool OneWireBus::_bus_guard_and_reset( bool autoreset, TickType_t xBlockTime ) 
{ 
    return  (   m_status == STATUS::RESET
            ||  (   autoreset 
                &&  cmd_bus_reset() 
                )   
            ) 
#ifdef OW_THREADSAFE            
            && ( xSemaphoreTakeRecursive(m_aquire_bus_mutex, xBlockTime) == pdTRUE );
#else
            && true;
#endif             
}



/* -----------------------------------------------------------------------------
 *
 * onewire_search_state_t
 *
 * -----------------------------------------------------------------------------
 */

OneWireBus::onewire_search_state_t::onewire_search_state_t()
{
}

/**
 * @struct onewire_search_state_t
 * 
 * @brief Primes state to search from a given registration code
 * 
 * @param registration_code
 */
OneWireBus::onewire_search_state_t::onewire_search_state_t( onewire_rom_code_t registration_code )
{
    *(onewire_rom_code_t*) ROM_NO = registration_code;
    LastDiscrepancy = 64;
    found = false;
}

onewire_rom_code_t OneWireBus::onewire_search_state_t::getRegistrationCode()
{
    return *(onewire_rom_code_t*) ROM_NO;
}

void OneWireBus::onewire_search_state_t::reset()
{
    ROM_NO [ 8 ] = 0;
    LastDiscrepancy = 0;
    LastFamilyDiscrepancy = 0;
    LastDeviceFlag = false;
    found = false;
}

/**
 * @brief Creates a copy of current state
 * 
 * @return
 */
OneWireBus::onewire_search_state_t OneWireBus::onewire_search_state_t::copy()
{
    onewire_search_state_t copy( getRegistrationCode() );
    return copy;
}

