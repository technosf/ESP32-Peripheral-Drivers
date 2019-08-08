/*
 * OneWireBus.h
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

#ifndef EPD_ONEWIREBUS_H_
#define EPD_ONEWIREBUS_H_

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
     * @class OneWireBus
     * @ingroup epd
     *
     * @brief Core Dallas 1-Wire bus protocol definition
     * @see https://www.maximintegrated.com/en/app-notes/index.mvp/id/126
     */
    class OneWireBus
    {
            static const constexpr char* TAG { "OneWireBus" };

        public:

            /* --------------------------------------------------------------
             * One Wire Standard Timings in Micro seconds (US)
             * --------------------------------------------------------------
             */
            static const constexpr float OW_RESET_TIME_US { 650 };
            static const constexpr float OW_SLOT_TIME_US { 80 };
            static const constexpr float OW_RECOVERY_TIME_US { 1 };

            static const constexpr float OW_PRESENCE_PULSE_HIGH_MIN_US { 15 };
            static const constexpr float OW_PRESENCE_PULSE_HIGH_MAX_US { 60 };
            static const constexpr float OW_PRESENCE_PULSE_LOW_MIN_US { 60 };
            static const constexpr float OW_PRESENCE_PULSE_LOW_MAX_US { 240 };

            static const constexpr float OW_READ_SLOT_LOW_US { 2 };
            static const constexpr float OW_READ_DATA_US { 12 };

            static const constexpr float OW_WRITE_SLOT_0_LOW_US { 74 };
            static const constexpr float OW_WRITE_SLOT_0_HIGH_US { OW_RECOVERY_TIME_US + OW_SLOT_TIME_US
                    - OW_WRITE_SLOT_0_LOW_US };    // Minimum = recovery time

            static const constexpr float OW_WRITE_SLOT_1_LOW_US { 7 };
            static const constexpr float OW_WRITE_SLOT_1_HIGH_US { OW_RECOVERY_TIME_US + OW_SLOT_TIME_US
                    - OW_WRITE_SLOT_1_LOW_US };    // Minimum = recovery time

            /* --------------------------------------------------------------
             * One Wire Over Drive Timings in Nano Seconds (NS)
             * --------------------------------------------------------------
             */
            static const constexpr float OW_OD_RESET_TIME_US { 70 };
            static const constexpr float OW_OD_SLOT_TIME_US { 15 };
            static const constexpr float OW_OD_RECOVERY_TIME_US { 1 };

            static const constexpr float OW_OD_PRESENCE_PULSE_HIGH_MIN_US { 2 };
            static const constexpr float OW_OD_PRESENCE_PULSE_HIGH_MAX_US { 6 };
            static const constexpr float OW_OD_PRESENCE_PULSE_LOW_MIN_US { 8 };
            static const constexpr float OW_OD_PRESENCE_PULSE_LOW_MAX_US { 24 };

            static const constexpr float OW_OD_READ_SLOT_LOW_US { 1.25 };
            static const constexpr float OW_OD_READ_DATA_US { 12 };

            static const constexpr float OW_OD_WRITE_SLOT_0_LOW_US { 14 };
            static const constexpr float OW_OD_WRITE_SLOT_0_HIGH_US { OW_OD_RECOVERY_TIME_US + OW_OD_SLOT_TIME_US
                    - OW_OD_WRITE_SLOT_0_LOW_US };    // Minimum = recovery time

            static const constexpr float OW_OD_WRITE_SLOT_1_LOW_US { 1.5 };
            static const constexpr float OW_OD_WRITE_SLOT_1_HIGH_US { OW_OD_RECOVERY_TIME_US + OW_OD_SLOT_TIME_US
                    - OW_OD_WRITE_SLOT_1_LOW_US };    // Minimum = recovery time

            /*
             * Constant Values
             */
            const onewire_data_t OW_DATA_ZERO { 0 };
            const onewire_data_t OW_DATA_ONE { 1 };

            /**
             * @typedef onewire_search_state_t
             * @brief Structure that encapsulates Search ROM state
             */
            struct onewire_search_state_t
            {
                    uint8_t ROM_NO [ 8 ] { 0 };
                    int LastDiscrepancy { 0 };
                    int LastFamilyDiscrepancy { 0 };
                    bool LastDeviceFlag { false };
                    bool found { false };

                    onewire_search_state_t()
                    {
                    }

                    /**
                     * @brief Primes state to search from a given registration code
                     * @param registration_code
                     */
                    onewire_search_state_t( uint64_t registration_code )
                    {
                        *(uint64_t*) ROM_NO = registration_code;
                        LastDiscrepancy = 64;
                        found = false;
                    }

                    uint64_t getRegistrationCode()
                    {
                        return *(uint64_t*) ROM_NO;
                    }

                    void reset()
                    {
                        ROM_NO [ 8 ] = 0;
                        LastDiscrepancy = 0;
                        LastFamilyDiscrepancy = 0;
                        LastDeviceFlag = false;
                        found = false;
                    }

                    /**
                     * @brief Creates a copy of current state
                     * @return
                     */
                    onewire_search_state_t copy()
                    {
                        onewire_search_state_t copy( getRegistrationCode() );
                        return copy;
                    }
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

            /*
             * One Wire Network Layer Commands
             */

            /**
             * @brief Performs a 1-Wire reset cycle.
             *
             * Returns true if a device responds with a presence pulse.
             * Returns false if there is no device or the bus is shorted
             * or otherwise held low for more than 480us.
             *
             * Also calculates the time base for the bus from the presence pulse
             * timings (time base = (PDh+PDl)/5)us) if the master pull down found.
             *
             * @return true if the bus is in reset state
             */
            virtual bool bus_reset();

            /**
             * @brief Address one specific iButton. Requires reset first.
             *
             * Address one specific iButton among several connected to the
             * 1–Wire bus. If only one iButton can be connected, the Skip ROM
             * command can be used instead to get the registration number.
             *
             * @return true if command sent
             */
            virtual bool match_rom( uint64_t registration_code, uint8_t func );

            /**
             * @brief  Issues a 1-Wire ROM Read command. Requires reset first.
             *
             * Identify a device or determine if several devices are connected in parallel.
             *
             * @return true if command sent
             */
            virtual bool read_rom();

            /**
             * @brief Issues a 1-Wire ROM Search command. Requires reset first.
             *
             * Implementation of the Maxim search algorithm, using a struct for the
             * search state initial and ending status. Searches from the given state
             * and returns when a device, or no more devices are found.
             *
             * @param [in/out] search_state the state to search and returning state
             * @return true if the command was issued
             */
            virtual bool search_rom( onewire_search_state_t& search_state );

            /**
             * @brief Issues a 1-Wire ROM Skip command, to address all devices on bus. Requires reset first.
             *
             * Omit addressing if only one device can be connected -OR-
             * Broadcast data to all devices on the bus, e.g., to format many
             * devices or to copy the contents of one device to many others. This
             * application usually requires all iButtons to be of the same type and to
             * be connected properly.
             */
            virtual bool skip_rom();

//            /**
//             * @brief the number of devices found after search/read ROM
//             * @return the number of devices found
//             */
//            uint8_t getDeviceCount();
//
//            /**
//             * @brief Is the given device on the bus?
//             * @param registration_code device registration code
//             * @return true if found on bus
//             */
//            bool isDevice( uint64_t registration_code );
//
//            /**
//             * @brief Return all the devices found on the bus as an unordered set
//             * @return the devices
//             */
//            const std::unordered_set< uint64_t > getDevices();

            /**
             * @brief Issues a 1-Wire ROM Overdrive Match command for one device. Requires reset first.
             *
             * Address one specific iButton among several connected to the 1–Wire bus
             * and set it into overdrive mode for subsequent communication at overdrive speed.
             *
             * @return
             */
            //  virtual bool overdrive_match_rom();
            /**
             * @brief Issues a 1-Wire rom skip command, to address all on bus.
             *
             * Set all overdrive–capable devices into overdrive mode and to subsequently
             * broadcast data at overdrive speed.
             */
            //  virtual bool overdrive_skip_rom();
//
//            /**
//             * @brief Issues a 1-Wire rom skip command, to address all on bus.
//             *
//             * Read one or several consecutive bytes of one or consecutive pages
//             * starting at any valid address.
//             */
//            virtual bool read_memory();
//
//
//            /**
//             * @brief Issues a 1-Wire rom skip command, to address all on bus.
//             *
//             * EPROM devices only: to read the redirection byte followed by an
//             * inverted CRC16, then read consecutive data bytes starting at any
//             * valid data address and obtain an inverted CRC16 of the previous data
//             * bytes at the end of the page; continued reading delivers the same
//             * sequence of information for the next pages.
//             */
//            virtual bool extended_read_memory();
//
//
//            /**
//             * @brief Issues a 1-Wire rom skip command, to address all on bus.
//             *
//             * Read one or several consecutive bytes of one or consecutive pages
//             * starting at any valid address.
//             */
//            virtual bool read_subkey();
//
//
//            /**
//             * @brief Issues a 1-Wire rom skip command, to address all on bus.
//             *
//             * Read one or several consecutive bytes of one or consecutive pages
//             * starting at any valid address.
//             */
//            virtual bool extended_read_memory();
//
//
//
//            /**
//             * @brief Issues a 1-Wire rom skip command, to address all on bus.
//             *
//             * Read one or several consecutive bytes of one or consecutive pages
//             * starting at any valid address.
//             */
//            virtual bool write_scratchpad();
            /*
             * Helper functions
             */

            /**
             * @brief Was a presence pulse seen on last reset
             * @return true if presence seen
             */
            virtual bool getPresence();

            /**
             * @brief adaptive timing minimum
             * @return the minimum time base in us, or zero if not known
             */
            virtual uint16_t getAdaptiveMin();

            /**
             * @brief adaptive timing maximum
             * @return the maximum time base in us, or zero if not known
             */
            virtual uint16_t getAdaptiveMax();

            /**
             * @brief Search the bus and and collates found registration codes.
             *
             * @return true if scan completed.
             */
            virtual bool scanRegistrationCodes();

            /**
             * @brief Returns registration codes of devices attached to the bus.
             * @return the registration codes
             */
            const virtual std::vector< uint64_t >& getRegistrationCodes();

            /**
             * @brief Writes out byte (command) data and reads a given number of slots,
             * returning the unmarshalled data.
             * @param to_wire data to be written to the bus
             * @param bits_to_read read slots to read and unmarshal
             * @param from_wire data unmarshalled from the bus
             * @return true if data written
             */
            virtual bool writeAndRead( onewire_data_t& to_wire, uint16_t bits_to_read, onewire_data_t& from_wire );

        protected:

            /**
             * @brief Implementation of 1Wire bus line level reset
             *
             * Sends a bus reset out on the wire and captures (possible the reset)
             * the responses of the bus. Returned data is by sign:
             * +ve = low, -ve = high and is the duration in microseconds.
             *
             * The first datum should correspond to the master pulling the bus low
             * (OW_RESET_TIME_US, or zero if the master is not captured). If the
             * master is captured, the adaptive timing is calculated.
             *
             * @param pulses [in/out] the pulse durations off the wire
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
             * @param data [in/out] the data converted from pulses off the wire
             * @return true if data retrieved
             */
            virtual bool _read_slots( uint16_t bits, onewire_data_t& data ) = 0;

            /**
             * @brief Processes individual pulse data and integrates it into a set of pulses
             *
             * This helper takes a raw pulse and encodes and incorporates it into the set of pulses.
             * It aggregates consecutive pulses with the same line level.
             *
             * @param pulses [in/out] the set of pulses being generated
             * @param level_high true if pulse line high, or false for pulse line low
             * @param duration the duration in us of the pulse
             */
            void _process_pulse( onewire_pulses_t& pulses, bool level_high, uint16_t duration );

            /**
             * @brief Unmarshal pulses into big-endian byte data
             *
             * This helper is implemented using the defined bus timings to differentiate
             * between Zero and One pulses.
             * Expect even number of pulses and alternating line levels.
             * Starts on first low pulse.
             *
             * @param pulses [in] the set of pulses off the wire
             * @param data [in/out] the unmarshalled data
             * @return the number of bits that were unmarshalled
             */
            uint16_t _unmarshal_pulses( onewire_pulses_t& pulses, onewire_data_t& data );

        private:

            const SemaphoreHandle_t m_aquire_bus_mutex { xSemaphoreCreateMutex() };    // Coordinate commandeering the bus

            bool m_presence { false };    // Presence pulse seen
            bool m_scanned { false };    // Bus has been scanned for devices and devices are known
            bool m_reset { false };     // Bus is in reset state
            uint16_t m_adaptive_min { 0 };
            uint16_t m_adaptive_max { 0 };

            std::vector< uint64_t > m_registration_codes;

    };
// OneWireBus
}// epd
#endif /* EPD_ONEWIREBUS_H_ */
