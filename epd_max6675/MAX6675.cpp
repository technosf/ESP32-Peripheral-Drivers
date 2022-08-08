/**
 *  @file       MAX6675.cpp
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

#include "MAX6675.h"


#define MAX6675_CELSIUS_MULT        0.25
#define MAX6675_FAHRENHEIT_MULT     0.45


using namespace epd;



MAX6675::MAX6675(spi_host_device_t SPI_HOST_DEVICE, int8_t CS, int8_t CLK, int8_t MISO) :
    MAX6675( SPI_HOST_DEVICE, static_cast< gpio_num_t >( CS ), static_cast< gpio_num_t >( CLK ), static_cast< gpio_num_t >( MISO ) )
{
}


MAX6675::MAX6675(spi_host_device_t SPI_HOST_DEVICE, gpio_num_t CS, gpio_num_t CLK, gpio_num_t MISO)
{
     m_max6675_data.uint_value = 65535;                 // Initialize MAX6675 raw data with 1s

    ESP_LOGD(TAG, "::MAX6675 Constructing SPI for Device:%d CS:%d CLK:%d MISO:%d", SPI_HOST_DEVICE, CS, CLK, MISO);

    m_spi_host = SPI_HOST_DEVICE;
    m_devcfg.spics_io_num = CS;
    m_buscfg.sclk_io_num = CLK;
    m_buscfg.miso_io_num = MISO;
}


MAX6675::MAX6675(spi_pins_t SPI_PINS) :
    MAX6675( SPI_PINS.device, SPI_PINS.cs_pin, SPI_PINS.clk_pin, SPI_PINS.miso_pin)
{
} 


MAX6675::~MAX6675()
{
   spi_bus_remove_device(m_spi);
   spi_bus_free(m_spi_host);
}



void MAX6675::initialize()
{
    //return;
    esp_err_t ret;


    ESP_LOGD(TAG, "::initialize...");

    //Initialize the SPI bus
    ESP_LOGD(TAG, "spi_bus_initialize");
    ret = spi_bus_initialize(m_spi_host, &m_buscfg, SPI_DMA_DISABLED);
    ESP_ERROR_CHECK(ret);

    ESP_LOGD(TAG, "spi_bus_add_device");
    ret = spi_bus_add_device(m_spi_host, &m_devcfg, &m_spi);
    ESP_ERROR_CHECK(ret);    

    ESP_LOGD(TAG, "::initialize complete.");

} // initialize


/**
 * Uses a static transaction member that is reset before each read.
 * The SPI bus is aquired before a polling read and it is realeased
 * only after the recieved data as been copied to the raw data member.
 * Data issues and open thermocouple issues are logged but do not
 * throw errors.
 */
void MAX6675::_readSensor()
{
    static spi_transaction_t transaction;
	memset(&transaction, 0, sizeof(transaction));

    transaction.length      = MAX6675_SO_SIZE_BITS; 
    transaction.rxlength    = MAX6675_SO_SIZE_BITS; 
    transaction.flags       = SPI_TRANS_USE_RXDATA; // Read into the transactio rx_data field, do not use DMA.


    ESP_LOGD(TAG, "::_readSensor - polling sensor.");

    spi_device_acquire_bus(m_spi, portMAX_DELAY);   // Lock out bus use while reading and copying the data
	assert(spi_device_polling_transmit(m_spi, &transaction) == ESP_OK);
    m_max6675_data.uint_value = SPI_SWAP_DATA_RX(*(uint32_t*)transaction.rx_data, 16);    // Copy the raw data
    spi_device_release_bus(m_spi);                  // Unlock the bus


    ESP_LOG_BUFFER_HEX_LEVEL(TAG,transaction.rx_data,2,ESP_LOG_DEBUG);
    ESP_LOGD(TAG, "::_readSensor - sign bit: %u, temperature: %u, thermocouple_input: %u, device_id: %u, three_state: %u",
                m_max6675_data.value.dummy_sign_bit,
                m_max6675_data.value.temperature_reading,
                m_max6675_data.value.thermocouple_input,
                m_max6675_data.value.device_id,
                m_max6675_data.value.three_state);


    if (m_max6675_data.value.dummy_sign_bit == 1)
      ESP_LOGE(TAG, "::_readSensor - Dummy sign bit is high");

    if (m_max6675_data.value.thermocouple_input == 1)
      ESP_LOGE(TAG, "::_readSensor - Thermocouple is not connected\n");

} // _readSensor


MAX6675::max6675_raw_t MAX6675::getRaw(bool live)
{
    if (live) _readSensor();
    return m_max6675_data.value;
} // getRaw


float MAX6675::getCelsius(bool live)
{
    if (live) _readSensor();
    if (m_max6675_data.value.thermocouple_input) return -1;
    return (MAX6675_CELSIUS_MULT*m_max6675_data.value.temperature_reading);
} // getCelsius

/**
 * Read the sensor if live reading required, otherwise return the current reading.
 * If the thermocouple is open, return -1 as the temperature.
 */
float MAX6675::getFahrenheit(bool live)
{
    if (live) _readSensor();
    if (m_max6675_data.value.thermocouple_input) return -1;
    return (MAX6675_FAHRENHEIT_MULT*m_max6675_data.value.temperature_reading)+32;
} // getFahrenheit
