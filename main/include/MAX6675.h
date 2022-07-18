/**
 *  @file       MAX6675.h
 *  @version    1.0.0
 *
 *  @brief      Driver for MAX6675 Temperature sensor
 *
 *  @date       Jul 8, 2022
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

#ifndef EPD_MAX6675_H_
#define EPD_MAX6675_H_

#define MAX6675_SO_SIZE_BITS    16
#define MAX6675_SPI_MODE        1
#define MAX6675_TCSS_NS         150

#include <cstring>

#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>

/**
 * 
 * 
 */
namespace epd
{
    /**
     * @class MAX6675
     * @ingroup epd
     *
     * @brief Reads MAX6675 temperature sensors using the ESP32's SPI interface
     *
     * @details 
     * This driver uses the ESP32 SPI peripheral to request and read the temerature 
     * from a MAX6675 and its connected thermocouple. The MAX6675 has a resolution
     * of 12 bits, a range of 0°C to 1023°C, with 0.25°C per bit. 
     *
     * This driver allows the resolution to be changed so that certain reading errors
     * (say the thermocouple isn't a K type) can still be compensated for.
     *
     * @author technosf <github.10.technomation@xoxy.net>
     * @see https://github.com/technosf/ESP32-Peripheral-Drivers
     */
    class MAX6675 
    {
        public:

            static const constexpr char* TAG { "MAX6675" };


            /**
             * @brief Data output format for raw MAX6675 readings
             * 
             */ 
            struct max6675_raw_t {
                uint8_t three_state : 1;
                uint8_t device_id : 1;
                uint8_t thermocouple_input : 1;
                uint16_t temperature_reading : 12;
                uint8_t dummy_sign_bit : 1;
            };  

         
            /**
             * @brief SPI Pins
             * 
             */
            struct spi_pins_t
            {
                spi_host_device_t device;
                gpio_num_t cs_pin;
                gpio_num_t clk_pin;
                gpio_num_t miso_pin;
            } ;


            static const constexpr spi_pins_t SPI2 {.device = SPI2_HOST, .cs_pin = GPIO_NUM_15, .clk_pin = GPIO_NUM_14, .miso_pin = GPIO_NUM_12}; //!< SPI2 device definition
            static const constexpr spi_pins_t SPI3 {.device = SPI3_HOST, .cs_pin = GPIO_NUM_5,  .clk_pin = GPIO_NUM_18, .miso_pin = GPIO_NUM_19}; //!< SPI3 device definition

            /**
             * @brief Construct a new MAX6675 object
             * 
             * @param SPI_HOST_DEVICE 
             * @param CS CS GPIO pin
             * @param CLK CLK GPIO pin
             * @param MISO MISO GPIO pin
             * @param resolutionCelsius reading to celsius resolution
             */
            MAX6675(spi_host_device_t SPI_HOST_DEVICE, int8_t CS, int8_t CLK, int8_t MISO, float resolutionCelsius = 0.25);

            /**
             * @brief Construct a new MAX6675 object
             * 
             * @param SPI_HOST_DEVICE 
             * @param CS CS GPIO pin
             * @param CLK CLK GPIO pin
             * @param MISO MISO GPIO pin
             * @param resolutionCelsius reading to celsius resolution
             */
            MAX6675(spi_host_device_t SPI_HOST_DEVICE, gpio_num_t CS, gpio_num_t CLK, gpio_num_t MISO, float resolutionCelsius = 0.25);

            /**
             * @brief Construct a new MAX6675 object
             * 
             * @param SPI_PINS 
             * @param resolutionCelsius reading to celsius resolution
             */
            MAX6675(spi_pins_t SPI_PINS, float resolutionCelsius = 0.25);

            virtual ~MAX6675();

            /**
             * @brief Initialize the SPI device and bus
             */
            virtual void initialize();


            /**
             * @brief return the last raw data read from the MAX6675
             * 
             * 
             * @param live True, reads live sensor data. False returns the last value read. 
             * @return max6675_raw_t raw MAX6675 data
             */
            virtual max6675_raw_t getRaw(bool live = true);


            /**
             * @brief Retruns the last temperature in Celsius
             * 
             * Converts the raw temperature value to Celsius.
             * The raw temperature value from the MAX6675 is 12 bits, 
             * from 0-1023°C in 0.25°C increments.
             * 
             * °C =     raw temp/4
             * 
             * @param live True, reads live sensor data. False returns the last value read. 
             * @return temperature in degrees Celsius; -1 for an open thermocouple
             */
            virtual float getCelsius(bool live = true);


            /**
             * @brief Returns the last temperature in Fahrenheit
             * 
             * °F = 	(9/5)°C + 32
             * °F = 	(9/5)(raw temp/4) + 32
             * 
             * @param live True, reads live sensor data. False returns the last value read. 
             * @return temperature in degrees Fahrenheit; -1 for an open thermocouple
             */
            virtual float getFahrenheit(bool live = true);


        private:

            union  {
                uint16_t uint_value;
                struct max6675_raw_t value;
                } m_max6675_data;               //!< last read MAX6675 raw data

            spi_host_device_t m_spi_host;       //!< SPI Host  

            spi_device_handle_t m_spi;          //!< Handle to the SPI Device

            /**
             * @brief SPI device config structure, config for MAX6675
             * 
             */
            spi_device_interface_config_t m_devcfg  = {
                .command_bits = 0,
                .address_bits = 0,
                .dummy_bits = 0,
                .mode = MAX6675_SPI_MODE,                    // MAX6675 appears to be CPOL=0, CPHA=1
                .duty_cycle_pos = 0,
                .cs_ena_pretrans = 0,
                .cs_ena_posttrans = 0,
                .clock_speed_hz = SPI_MASTER_FREQ_8M/2,     // MAX6675 max clock is 4.3MHz
                .input_delay_ns = MAX6675_TCSS_NS,          // MAX6675 datasheet specifies a Tcss > 100ns
                .spics_io_num = 0,
                .flags = 0,
                .queue_size = 1,                            // One reading at a time         
                .pre_cb = nullptr, 
                .post_cb = nullptr,
            };

            /**
             * @brief SPI bus config structure, config for MAX6675
             * 
             */
            spi_bus_config_t m_buscfg  = {
                .mosi_io_num = -1,
                .miso_io_num = 0,
                .sclk_io_num = 0,
                .quadwp_io_num = -1,
                .quadhd_io_num = -1,
                .data4_io_num = -1,
                .data5_io_num = -1,
                .data6_io_num = -1,
                .data7_io_num = -1,
                .max_transfer_sz = MAX6675_SO_SIZE_BITS,
                .flags = 0,
                .intr_flags = 0
            };

            /**
             * @brief Read the MAX6675 sensor and populate m_max6675_data
             * 
             */
            void readSensor();

    }; // MAX6675

} // namespace

#endif /* EPD_MAX6675_H_ */
