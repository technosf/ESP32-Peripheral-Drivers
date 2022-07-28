/**
 *  @file       OneWireBusRMT.h
 *  @version    0.1.0
 *
 *  @brief      ESP32 RMT implementation of Dallas 1-Wire Bus protocol driver
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

#ifndef EPD_ONEWIREBUSRMT_H_
#define EPD_ONEWIREBUSRMT_H_

#include <esp_system.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/rmt.h>

#include <stdlib.h>

#include "OneWireBus.h"

namespace epd
{
    /**
     * @class OneWireBusRMT
     * @ingroup epd
     *
     * @brief ESP32 RMT implementation of OneWireBus
     *
     * Uses RMT device to read/write pulses over the one wire bus using one channel/one pin
     * each for TX and RX where both are connected at the same time to the same wire. The
     * RX channel requires two memory blocks, while the TX channel one. It is possible
     * for the TX channel memory to be shared with the RX channel so that only two
     * channels are used, as long as the RX and TX channels are consecutive and the
     * RX channel is the lower channel.
     *
     * @author technosf <github.10.technomation@xoxy.net>
     * @see https://github.com/technosf/ESP32-Peripheral-Drivers
     */
    class OneWireBusRMT : public OneWireBus
    {
            static const constexpr char* TAG { "OneWireBusRMT" };

        public:

            /**
             * @brief Constructor using integer values to define RMT channels and pins
             *
             * Pins are required, but RMT channels default to 6 and 7.
             *
             * @param rx_pin the GPIO pin number for RX
             * @param tx_pin the GPIO pin number for TX
             * @param rx_channel the RMT channel number for RX (should be 0-6). Defaults to 6
             * @param tx_channel the RMT channel number for TX (should be 1-7) and one greater than RX. Defaults to 7
             */
            OneWireBusRMT( uint8_t rx_pin, uint8_t tx_pin, uint8_t rx_channel = 6, uint8_t tx_channel = 7 );


            /**
             * @brief Constructor using RMT & GPIO units
             *
             * Pins are required, but RMT channels default to 6 and 7.
             *
             * @param rx_pin the GPIO pin for RX
             * @param tx_pin the GPIO pin for TX
             * @param rx_channel the RMT channel for RX (should be 0-6). Defaults to 6
             * @param tx_channel the RMT channel for TX (should be 1-7) and one greater than RX. Defaults to 7
             */
            OneWireBusRMT( gpio_num_t rx_pin, gpio_num_t tx_pin, rmt_channel_t rx_channel = RMT_CHANNEL_6,
                    rmt_channel_t tx_channel = RMT_CHANNEL_7 );


            /**
             * @brief Destroy the OneWireBusRMT object
             * 
             */
            virtual ~OneWireBusRMT();


            /**
             * @brief Info on the implementation
             * 
             * @return
             */
            const char* info() final
            {
                return "OneWireBusRMT";
            }


            /**
             * @brief Initialize the ESP32 peripherals ready for bus operations
             */
            void initialize();


        private:

            /**
             * @see OneWireBus
             */
            bool _bus_reset( onewire_pulses_t& pulses );


            /**
             * @brief Strong pull-up on the bus
             *
             * This is a special case - both tx and rx are put into transmit
             *
             * @see OneWireBus
             */
            virtual bool _strong_pullup( uint8_t pullup_duration_us );


            /**
             * @see OneWireBus
             */
            bool _write_slots( uint16_t bits, const onewire_data_t& data );


            /**
             * @see OneWireBus
             */
            bool _read_slots( uint16_t bits, onewire_data_t& data );

            /**
             * @see OneWireBus
             */
            virtual void _set_adaptive_timing( ADAPTIVE_TIMING timing );

            /* --------------------------------------------------------------
             * Implementation Specifics
             * -------------------------------------------------------------- */

            /**
             * @brief Probe the bus, returning pulse timings on the line
             *
             * Starts with first low pulse found.
             *
             * @param [in/out] pulses the pointer for the returned data
             * @return the number of items returned
             */
            size_t _probe( onewire_pulses_t& pulses );


            
            /* --------------------------------------------------------------
             * Members
             * -------------------------------------------------------------- */

            gpio_num_t m_rx_pin, m_tx_pin;                  //!< RMT RX and TX pins
            rmt_channel_t m_rx_channel, m_tx_channel;       //!< RMT RX and TX channels
            RingbufHandle_t m_rx_ringBuf { nullptr };       //!< RMT RX ringbuffer

    };
// OneWireBusRMT
}// epd
#endif /* EPD_ONEWIREBUSRMT_H_ */
