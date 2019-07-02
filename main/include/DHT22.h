/*
 * DHT22.h
 *
 * Driver for DHT22 Temperature and Humidity sensor
 *
 *  Created on: Jul 1, 2019
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

#ifndef EPD_DHT22_H_
#define EPD_DHT22_H_

#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/rmt.h>

namespace epd
{

    /**
     * \class DHT22
     * \ingroup epd
     *
     * \brief Communicates with DHT22 temperature and humidity sensor using the ESP32's RMT interface
     *
     * This driver uses the ESP32 RMT peripheral to capture the DHT22 signaling in the background
     * while it waits to process the checksum and calculate the temperature and humidity.
     *
     * Most of the timing can be modified via the configuration constants.
     *
     * \author technosf <github.10.technomation@xoxy.net>
     * \see https://github.com/technosf/ESP32-Peripheral-Drivers
     */
    class DHT22
    {
            static const constexpr char* TAG { "DHT22" };

            /*
             * Configuration constants
             *
             */
            const uint16_t DHT22_REREAD_INTERVAL_MS { 2000 };                   // Interval between the DHT22 can be read
            const uint8_t DHT22_PULLDOWN_PERIOD { 10 / portTICK_PERIOD_MS };    // DHT22 data line pulldown period to initiate data send
            const uint8_t DHT22_READ_PERIOD { 10 / portTICK_PERIOD_MS };        // Period required for the DHT22 to send its data
            const uint8_t DHT22_INTERVAL_MIN { 140 };
            const uint8_t DHT22_INTERVAL_MAX { 180 };
            const uint8_t DHT22_PULSE_MIN { 55 };
            const uint8_t DHT22_PULSE_MAX { 145 };
            const uint8_t DHT22_ON_OFF_PULSE_THRESHOLD { 110 };                 // Time threshold to descriminate between on/off pulses

            /**
             *  \enum dht22_status_t
             *  \brief Status from reading and processing data from the DHT22
             */
            enum dht22_status_t
            {
                DHT22_STATUS_UNSET = -1,        //!< DHT22_STATUS_UNSET
                DHT22_STATUS_OK = 0,            //!< DHT22_STATUS_OK
                DHT22_STATUS_TIMEOUT = 1,       //!< DHT22_STATUS_TIMEOUT
                DHT22_STATUS_NACK = 2,          //!< DHT22_STATUS_NACK
                DHT22_STATUS_DATA = 3,          //!< DHT22_STATUS_DATA
                DHT22_STATUS_CHECKSUM = 4,      //!< DHT22_STATUS_CHECKSUM
                DHT22_STATUS_UNDERFLOW = 5,     //!< DHT22_STATUS_UNDERFLOW
                DHT22_STATUS_OVERFLOW = 6,      //!< DHT22_STATUS_OVERFLOW
            };

        public:

            /**
             * \brief Instantiate new DHT22
             * \param pin the GPIO pin connected to the DHT22 data line
             * \param channel the RMT channel to use to capture data
             */
            DHT22( int pin, int channel );

            /**
             * \brief Instantiate new DHT22
             * \param pin the GPIO pin connected to the DHT22 data line
             * \param channel the RMT channel to use to capture data
             */
            DHT22( gpio_num_t pin, rmt_channel_t channel );

            virtual ~DHT22();

            /**
             * \brief Initialize the peripheral
             */
            virtual void initialize();

            /**
             * \brief Update the temperature and humidity readings
             * \return true if the readings were updated
             */
            virtual bool update();

            /**
             * \brief Get the last raw data read from the DHT22
             * \return the raw data
             */
            virtual uint8_t * getRaw();

            /**
             * \brief Get the status of the last data read
             * \return the status
             */
            virtual dht22_status_t getStatus();

            /**
             * \brief Get the temperature in Celsius
             * \return temperature in degrees Celsius
             */
            virtual float getTemperatureCelsius();

            /**
             * \brief Get the temperature in Fahrenheit
             * \return temperature in degrees Fahrenheit
             */
            virtual float getTemperatureFahrenheit()
            {
                return C2F( getTemperatureCelsius() );
            }

            /**
             * \brief Get the percent humidity
             * \return humidity percent
             */
            virtual float getHumidityPercent();

            /**
             * \brief Convert Celsius to Fahrenheit
             *
             *  (0°C × 9/5) + 32 = 32°F
             *
             * \param celsius
             * \return fahrenheit
             */
            static float C2F( float celsius )
            {
                return ( 1.8 * celsius ) + 32;
            }

            /**
             * \brief Convert Fahrenheit to Celsius
             *
             * (32°F − 32) × 5/9 = 0°C
             *
             * \param fahrenheit
             * \return celsius
             */
            static float F2C( float fahrenheit )
            {
                return ( fahrenheit - 32 ) * 0.555555556;
            }

        private:

            gpio_num_t m_pin;
            rmt_channel_t m_channel;

            SemaphoreHandle_t m_read_exclusion { xSemaphoreCreateMutex() };
            RingbufHandle_t m_ringBuf { nullptr };
            uint32_t m_next_read_timestamp { 0 };

            dht22_status_t m_status { DHT22_STATUS_UNSET };
            uint8_t * m_raw { nullptr };
            float m_last_temperature_celcius { 0 };
            float m_last_humidity_percent { 0 };


            /**
             * \brief Twiddle the DH22 and read the pin for any DHT22 output
             */
            void _enquire();


            /**
             * \brief Process and RMT data read from the DHT22 and stored in the ring buffer
             * \return the status of the processed data
             */
            dht22_status_t _process();


            /**
             * \brief Decode the signals received by RMT and set the status
             *
             * \param data the RMT data
             * \param number_of_pulses the number of pulses collected
             * \return the status of the decode
             */
            dht22_status_t _decode( rmt_item32_t* data, int number_of_pulses );

    }; // DHT22

} // namespace

#endif /* EPD_DHT22_H_ */
