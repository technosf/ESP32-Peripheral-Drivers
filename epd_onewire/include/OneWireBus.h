/**
 *  @file       OneWireBus.h
 *  @version    0.1.0
 *
 *  @brief      Driver definition for Dallas 1-Wire Bus protocol
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

#ifndef EPD_ONEWIREBUS_H_
#define EPD_ONEWIREBUS_H_


/*  Define OW_THREADSAFE to enable threadsafe operation */
//#define OW_THREADSAFE 
#undef OW_THREADSAFE 



/*  Define OW_PARASITIC to incorporate parasitic power management operations */
//#define OW_PARASITIC 
#undef OW_PARASITIC 


/* One Wire Over-Drive Timings in Micro Seconds (US) */
/* Timings from https://pdfserv.maximintegrated.com/en/an/AN126.pdf */

//#define OW_OVERDRIVE 
#undef OW_OVERDRIVE 


#ifndef OW_OVERDRIVE
    #pragma message "OWR Standard timings used."
// --------------------------------------------------------------
// One Wire Standard Timings in Micro Seconds (US)
//  --------------------------------------------------------------

    #define OW_RESET_TIME_US                    480 
    #define OW_SLOT_TIME_US                     80
    #define OW_RECOVERY_TIME_US                 1

    #define OW_PRESENCE_PULSE_HIGH_MIN_US       15
    #define OW_PRESENCE_PULSE_HIGH_MAX_US       60
    #define OW_PRESENCE_PULSE_LOW_MIN_US        60
    #define OW_PRESENCE_PULSE_LOW_MAX_US        240

    #define OW_READ_SLOT_LOW_US                 2
    #define OW_READ_DATA_US                     12

    #define OW_WRITE_SLOT_0_LOW_US              74
    #define OW_WRITE_SLOT_1_LOW_US              7
#endif


#ifdef OW_OVERDRIVE
    #pragma message "OWR Over-Drive timings used."
// --------------------------------------------------------------
// One Wire Over-Drive Timings in Micro Seconds (US)
//  --------------------------------------------------------------

    #define OW_RESET_TIME_US                    70 
    #define OW_SLOT_TIME_US                     15
    #define OW_RECOVERY_TIME_US                 1

    #define OW_PRESENCE_PULSE_HIGH_MIN_US       2
    #define OW_PRESENCE_PULSE_HIGH_MAX_US       6
    #define OW_PRESENCE_PULSE_LOW_MIN_US        8
    #define OW_PRESENCE_PULSE_LOW_MAX_US        24

    #define OW_READ_SLOT_LOW_US                 1.25
    #define OW_READ_DATA_US                     12

    #define OW_WRITE_SLOT_0_LOW_US              14
    #define OW_WRITE_SLOT_1_LOW_US              1.5
#endif


#define OW_SLOT_RECOVERY_US                 ( OW_RECOVERY_TIME_US + OW_SLOT_TIME_US )
#define OW_WRITE_SLOT_0_HIGH_US             ( OW_SLOT_RECOVERY_US - OW_WRITE_SLOT_0_LOW_US )    //!< Minimum = recovery time
#define OW_WRITE_SLOT_1_HIGH_US             ( OW_SLOT_RECOVERY_US - OW_WRITE_SLOT_1_LOW_US )    //!< Minimum = recovery time


#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <freertos/semphr.h>

#include <stdint.h>
#include <string.h>
#include <algorithm>
#include <unordered_set>
#include <vector>


namespace epd
{
    /**
     * @typedef onewire_pulses_t
     * @ingroup epd
     *
     * @brief pulse durations off the wire in us
     *
     * Positive numbers are durations that the line is actively pulled low.
     * Negative numbers are durations that the line is pulled high by pull-up resistor.
     *
     * @author technosf <github.10.technomation@xoxy.net>
     * @see https://github.com/technosf/ESP32-Peripheral-Drivers
     */
    using onewire_pulses_t = std::vector<int>;

    /**
     * @typedef onewire_data_t
     * @ingroup epd
     *
     * @brief data for/from the wire
     */
    using onewire_data_t = std::vector<uint8_t>;

    /**
     * @typedef onewire_rom_code_t
     * @ingroup epd
     * 
     * @brief ROM Code datatype
     * 
     */
    using onewire_rom_code_t = uint64_t;



    /**
     * @class OneWireBus
     * @ingroup epd
     *
     * @brief Core Dallas 1-Wire bus protocol definition
     * @see https://www.maximintegrated.com/en/app-notes/index.mvp/id/126
     */
    class OneWireBus
    {
            static const constexpr char* TAG { "OneWireBus" };                  //!< ESP LOG Tg

        public:

            /**
             * @enum OneWireBus power source
             * 
             */
            
            enum ADAPTIVE_TIMING
            {
                NONE = 0,               //!< Standard timings
                READ = 1,               //!< Adapt slot reads
                WRITE = 2,              //!< Adapt slot writes
                READ_WRITE = 3,         //!< Adapt slot reads and writes
            };
            
            
            /**
             * @enum OneWireBus status
             * 
             */
            enum STATUS
            {
                UNRESET = 0,            //!< Bus requires a reset before any new command can be sent
                RESET = 1,              //!< Bus is reset and awaiting a command
                ADDRESSED = 2,          //!< Bus has had devices addressed
            };


            /**
             * @enum UNMARSHAL_BEHAVIOR
             *
             * @brief brief description to handle unmarshalling when first pulse is high
             */
            enum UNMARSHAL_BEHAVIOR
            {
                FAIL,                   //!< FAIL
                DISCARD_HIGH,           //!< DISCARD_HIGH Discard the first high pulse
                MAKE_ZERO,              //!< MAKE_ZERO Insert a read slot low to make this pulse zero
                MAKE_ONE                //!< MAKE_ONE Insert a read slot low to make this pulse one
            };

            /**
             * @struct onewire_search_state_t
             * 
             * @brief Structure that encapsulates Search ROM state
             */
            struct onewire_search_state_t
            {
                    uint8_t ROM_NO [ 8 ] { 0 };
                    int LastDiscrepancy { 0 };
                    int LastFamilyDiscrepancy { 0 };
                    bool LastDeviceFlag { false };
                    bool found { false };

                    onewire_search_state_t();

                    /**
                     * @struct onewire_search_state_t
                     * 
                     * @brief Primes state to search from a given registration code
                     * 
                     * @param registration_code
                     */
                    onewire_search_state_t( onewire_rom_code_t registration_code );

                    onewire_rom_code_t getRegistrationCode();

                    void reset();

                    /**
                     * @brief Creates a copy of current state
                     * 
                     * @return
                     */
                    onewire_search_state_t copy();
            };

            /**
             *
             */
            virtual ~OneWireBus()
            {
            }


            /**
             * @brief Information about this implementation of the bus protocol
             */
            const virtual char* info() = 0;


            /**
             * @brief Initialize the bus for operation
             * 
             * Do wire initialization, search for ROM codes
             * 
             * @return true is initialization was successful
             */
            bool initialize( bool search = true );


            /* --------------------------------------------------------
             *
             * One Wire Network Layer Commands
             *
             * --------------------------------------------------------
             */

            /**
             * @brief Performs a 1-Wire reset cycle.
             *
             * Returns true if any device responds with a presence pulse.
             * Returns false if there is no device or the bus is shorted
             * or otherwise held low for more than 480us.
             *
             * Also calculates the time base for the bus from the presence pulse
             * timings (time base = (PDh+PDl)/5)us) if the master pull down found.
             *
             * @return true if reset and presence reponse seen
             */
            virtual bool cmd_bus_reset();


            /**
             * @brief Address one specific iButton. Requires reset first.
             *
             * Address one specific iButton among several connected to the
             * 1–Wire bus. If only one iButton can be connected, the Skip ROM
             * command can be used instead to get the registration number.
             *
             * @return true if command sent
             */
            virtual bool cmd_match_rom( const onewire_rom_code_t& rom_code, bool autoreset = true );


            /**
             * @brief  Issues a 1-Wire ROM Read command. Requires reset first.
             *
             * Identify a device or determine if several devices are connected in parallel.
             *
             * @param[out] romcode the found romcode
             * @return true if command issued
             */
            virtual bool cmd_read_rom( onewire_rom_code_t& romcode, bool autoreset = true );


            /**
             * @brief Issues a 1-Wire ROM Search command. Requires reset first.
             *
             * Implementation of the Maxim search algorithm, using a struct for the
             * search state initial and ending status. Searches from the given state
             * and returns when a device, or no more devices are found.
             *
             * @param search_state the state to search and returning state
             * @param autoreset Allows bus reset to bypassed
             * @return true if the command was issued
             */
            virtual bool cmd_search_rom( onewire_search_state_t& search_state, bool autoreset = true );


            /**
             * @brief Issues a 1-Wire ROM Skip command, to address all devices on bus. Requires reset first.
             *
             * Omit addressing if only one device can be connected -OR-
             * Broadcast data to all devices on the bus, e.g., to format many
             * devices or to copy the contents of one device to many others. This
             * application usually requires all iButtons to be of the same type and to
             * be connected properly.
             */
            virtual bool cmd_skip_rom( bool autoreset = true );


            /* --------------------------------------------------------
             *
             * Command Wrapper functions
             *
             * --------------------------------------------------------
             */
            
            /**
             * @brief Attempt to guard the bus
             * 
             * @param xBlockTime time to wait for the guard
             * @return true if the bus is guarder
             * @return false if the bus guard failed
             */
            inline bool busGuard( TickType_t xBlockTime = portMAX_DELAY );

            /**
             * @brief Release the bus guard and set the bus state
             * 
             * The bus is either RESET (Bus Reset was last command),
             * ADDRESSED (devices were selected with an addressing command),
             * or UNRESET (not reset nor addressed).
             * 
             * @param busreset sets the bus state
             */
            inline void busRelease( STATUS state = STATUS::UNRESET );

            /**
             * @brief 
             * 
             * @param cmd 
             * @param search_state 
             * @param autoreset 
             * @return true 
             * @return false 
             */
            bool busSearch( uint8_t cmd, OneWireBus::onewire_search_state_t& search_state, bool autoreset );

            /* --------------------------------------------------------
             *
             * Value Add Bus Commands
             *
             * --------------------------------------------------------
             */

            /**
             * @brief Read the ROM Code of the single device attached to the bus
             * 
             * @return true if read completed.
             */
            virtual bool cmdReadRom();


            /**
             * @brief Search the bus for any attached devices and collates their ROM codes.
             *
             * Updates the object state.
             * 
             * @return true if scan completed.
             */
            virtual bool cmdSearchRom();

            
            /**
             * @brief Address the bus for the given device using the most efficient command
             * 
             * If the device is the only one on the bus, use Skip ROM, otherwise use Match ROM.
             * 
             * @param device the device ROM Code
             * @return true the bus is addressed
             */
            virtual bool cmdAddressDevice( const onewire_rom_code_t& device );


            /* --------------------------------------------------------
             *
             * Value Add Wire Operations
             *
             * --------------------------------------------------------
             */

            /**
             * @brief Writes out byte (command) data and reads a given number of slots,
             * returning the unmarshalled data.
             * 
             * @param to_wire data to be written to the bus
             * @param bits_to_read read slots to read and unmarshal
             * @param[out] from_wire data unmarshalled from the bus
             * @return true if data written
             */
            virtual bool wireWriteAndRead( const onewire_data_t& to_wire, uint16_t bits_to_read, onewire_data_t& from_wire );
            virtual bool wireWriteAndRead( uint8_t to_wire, uint16_t bits_to_read, onewire_data_t& from_wire );
            virtual bool wireWrite( uint8_t to_wire );

            /**
             * @brief Reads bus every 100ms until a 1 is read
             * 
             * @param maxSlots manimum number of slots
             * @return true 
             * @return false 
             */
            virtual bool wireReadUntilOne( uint8_t maxSlots );


            /* --------------------------------------------------------
             *
             * Bus State functions
             *
             * --------------------------------------------------------
             */

            /**
             * @brief Was a presence pulse seen on last reset
             * 
             * @return true if presence seen
             */
            virtual bool getPresence();

            /**
             * @brief Has the bus been searched for all attached devices
             * 
             * @return true bus has been successfully searched
             * @return false bus has not been successfully searched
             */
            virtual bool isSearched();

            /**
             * @brief Get the bus Status
             * 
             * The bus status is one of:
             *  - Reset (where it is ready for a ROM command)
             *  - Addressed (where devices have been addressed and are ready for a function command)
             *  - Unknown (where the state is unknow or a function command has been issued)
             * 
             * @return STATUS the current status
             */
             STATUS getStatus();
             
            /**
             * @brief adaptive shortest time to begin read
             * 
             * @details 
             * "The captured value in timer TO 19 is the intrinsic period
             * of the fastest 1-Wire slave, and will be used to set up the
             * timing of a 1-Wire read, to increase reliability of reading a
             * correct “1” on the line. To provide timing margin the value
             * in TO 19 is decremented by 02 Hex (approximately 1 us at
             * an 8 MHZ Xtal 1 clock). This value is then subtracted from
             * FF Hex and reloaded back into TO 19 to provide the required
             * count down value to be used in the read/write communications."
             * 
             * "When issuing reads, the user should enable the Timer TO 19
             * interrupt, as this interrupt will indicate when data is ready to
             * be read from the port. A read is initiated by writing a 1 to
             * Port 0.010 and waiting for a Timer TO 19 interrupt. "
             * 
             * 
             * @return the minimum time to read 
             * @see https://patents.google.com/patent/US5978927A/en
             * @todo Implement adaptive timing on a switch
             */
            virtual uint16_t getAdaptiveFastestRead(); 


            /**
             * @brief adaptive timing maximum
             * 
             * @details 
             * "This value will be used to control the speed of
             * 1-Wire communications, the slowest slave being the bottle
             * neck. When writing a “O'” on the line, the line must be held
             * low for at least the period of the slowest slave. Similar to the
             * value in timer TO 19, the result of the divide by five of the
             * T1 value, is then incremented by 01 Hex (approximately 1.5
             * us at an 8 MH Xtal 1 clock) to provide timing margin. This
             * value is Subtracted from FF Hex and reloaded back into
             * timer T116 to provide the required count up value to be used
             * in the read/write communications."
             * 
             * @return the maximum time to write a 0
             * @see https://patents.google.com/patent/US5978927A/en
             * @todo Implement adaptive timing on a switch
             */
            virtual uint16_t getAdaptiveSlowestWrite(); 

            /**
             * @brief Inspect or set adaptive timing use
             * 
             * @param use true if setting timing, false if inquiry
             * @param timing the timing to set if setting
             * @return ADAPTIVE_TIMING the current timing
             */
            virtual ADAPTIVE_TIMING useAdaptive( bool setadaptive = false, ADAPTIVE_TIMING timing = NONE );


            /**
             * @brief Returns registration codes of known devices attached to the bus.
             * 
             * @return the registration codes
             */
            const virtual std::vector< onewire_rom_code_t >& getRomCodes();


        protected:

            /* --------------------------------------------------------
             *
             * Implementation
             *
             * --------------------------------------------------------
             */

            /**
             * @brief Initialize the bus
             * 
             * Check for presence, devices and parasistic power use.
             * 
             * @return true if the bus was initialized
             */
            virtual bool _initialize( ) = 0;

            /**
             * @brief Implementation of 1Wire bus line level reset
             *
             * Sends a bus reset out on the wire and captures
             * the responses of the bus. Returned data is by sign:
             * +ve = low, -ve = high and is the duration in microseconds.
             *
             * The first datum should correspond to the master pulling the bus low
             * (OW_RESET_TIME_US, or zero if the master is not captured). If the
             * master is captured, the adaptive timing is calculated.
             *
             * @param[out] pulses the pulse durations off the wire
             * @return true if the reset was sent
             */
            virtual bool _bus_reset( onewire_pulses_t& pulses ) = 0;


            /**
             * @brief Write the given bits of data to the bus
             *
             * @param bits the number of bits to write
             * @param data the data to write
             * @return true if written
             */
            virtual bool _write_slots( uint16_t bits, const onewire_data_t& data ) = 0;


            /**
             * @brief Write the given whole byte of data to the bus
             *
             * @param data the data byte to write
             * @return true if written
             */
            virtual bool _write_slots( uint8_t data ) = 0;


            /**
             * @brief Write the given whole bytes of data to the bus
             *
             * @param data the data to write
             * @return true if written
             */
            virtual bool _write_slots( const onewire_data_t& data ) final;


            /**
             * @brief Write the given bit of data to the bus
             *
             * @param data the bit to write
             * @return true if written
             */
            virtual bool _write_slots( bool data ) final;


            /**
             * @brief Signal read slots and retrieve the data, converting from wire-format to machine format
             *
             * 1Wire data is signaled by devices LSB-MSB. This function should convert that to MSB-LSB at
             * the per-byte level.
             *
             * @param bits the number of bits to read
             * @param[out] data the data converted from pulses off the wire
             * @return true if data retrieved
             */
            virtual bool _read_slots( uint16_t bits, onewire_data_t& data ) = 0;


            /**
             * @brief Processes individual pulse data and integrates it into a set of pulses
             *
             * This helper takes a raw pulse and encodes and incorporates it into the set of pulses.
             * It aggregates consecutive pulses with the same line level.
             *
             * @param[out] pulses the set of pulses being generated
             * @param level_high true if pulse line high, or false for pulse line low
             * @param duration the duration in us of the pulse
             */
            virtual void _process_pulse( onewire_pulses_t& pulses, bool level_high, uint16_t duration );


            /**
             * @brief Unmarshal pulses into big-endian byte data
             *
             * This helper is implemented using the defined bus timings to differentiate
             * between Zero and One pulses.
             * Expect even number of pulses and alternating line levels.
             * Starts on first low pulse.
             *
             * @param pulses the set of pulses off the wire
             * @param[out] data the unmarshalled data
             * @param behavior Behavior if first pulse is high
             * @return the number of bits that were unmarshalled
             */
            virtual uint16_t _unmarshal_pulses( onewire_pulses_t& pulses, onewire_data_t& data, UNMARSHAL_BEHAVIOR behavior =
                    FAIL );


            /**
             * @brief Put a strong pull up on the bus for the given duration
             * @param pullup_duration_us pullup duration in us
             * @return true if transmitted
             */
            virtual bool _strong_pullup( uint8_t pullup_duration_us ) = 0;


            /**
             * @brief Implement the adaptive timing
             * 
             * @param timing the timings to adapt
             */
            virtual void _set_adaptive_timing( ADAPTIVE_TIMING timing ) = 0;


        private:

            /*
             * Constant Values
             */
            const onewire_data_t OW_DATA_ZERO { 0 };
            const onewire_data_t OW_DATA_ONE { 1 };

#ifdef OW_THREADSAFE
            const SemaphoreHandle_t m_aquire_bus_mutex { xSemaphoreCreateRecursiveMutex() };    // Coordinate commandeering the bus
#endif

            /*
            *   Members
            */
            std::vector< onewire_rom_code_t > m_rom_codes;  // Known ROM Codes on this bus
            bool m_presence { false };                      // Presence pulse seen
            bool m_scanned { false };                       // Bus has been scanned for devices and devices are known
            STATUS m_status { UNRESET };                    //!< Bus status
            uint16_t m_adaptive_fastest_read { 0 };         // From Tpdh, the fastest device, min time needed for read-slot
            uint16_t m_adaptive_slowest_write { 0 };        // (Tpdh+Tpdl)/5 -  the slowest device
            ADAPTIVE_TIMING m_adaptive_timing { NONE };     //

            /**
             * @brief Check bus is reset/autoreset and guard the bus
             * 
             * For Bus level (network) commands that require a reset bus
             * 
             * @param autoreset reset the bus if not reset
             * @param xBlockTime time to wait for the guard
             * @return true if bus is guarded and in reset state
             * @return false if the bus is not guarded and reset
             */
            inline bool _bus_guard_and_reset( bool autoreset = true, TickType_t xBlockTime = portMAX_DELAY );


    }; // OneWireBus

} // epd

#endif /* EPD_ONEWIREBUS_H_ */
