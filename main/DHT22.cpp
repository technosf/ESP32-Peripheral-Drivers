/*
 * DHT22.cpp
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

#include "DHT22.h"

using namespace epd;


DHT22::DHT22( int pin, int channel ) :
        DHT22( static_cast< gpio_num_t >( pin ), static_cast< rmt_channel_t >( channel ) )
{
}


DHT22::DHT22( gpio_num_t pin, rmt_channel_t channel )
{
    m_pin = pin;
    m_channel = channel;
}

DHT22::~DHT22()
{
    rmt_driver_uninstall( m_channel );
}


void DHT22::initialize()
{
    ESP_ERROR_CHECK( gpio_set_direction( m_pin, GPIO_MODE_INPUT_OUTPUT ) );    //GPIO_MODE_INPUT_OUTPUT);
    ESP_ERROR_CHECK( gpio_set_pull_mode( m_pin, GPIO_PULLDOWN_ONLY ) );

    rmt_config_t rmt_rx;
    rmt_rx.channel = m_channel;
    rmt_rx.gpio_num = m_pin;
    rmt_rx.clk_div = 80;                            // 1μs (80 Mhz / 80)
    rmt_rx.mem_block_num = 1;                       // Use 1 memory block
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.rx_config.filter_en = true;              // Filter out pulses below threshold
    rmt_rx.rx_config.filter_ticks_thresh = 50;      // Ignore pulses shorter than threshold
    rmt_rx.rx_config.idle_threshold = 500;          // 1 millis
    ESP_ERROR_CHECK(rmt_config( &rmt_rx ));

    ESP_ERROR_CHECK( rmt_driver_install( m_channel, 512, 0 ) );       // RMT data is cycled into a 512 byte RingBuffer
    ESP_ERROR_CHECK( rmt_get_ringbuf_handle( m_channel, &m_ringBuf ) );

    ESP_LOGD( TAG, "::initialize - RMT device on pin %d / channel %d initialized.", m_pin, m_channel );

}    // initialize


uint8_t * DHT22::getRaw()
{
    return m_raw;
}


DHT22::dht22_status_t DHT22::getStatus()
{
    return m_status;
}


bool DHT22::update()
{
    ESP_LOGD( TAG, "::update" );

    if ( m_read_exclusion == NULL )
    {
        ESP_LOGE( TAG, "::update - DHT22 not initialized." );
        return false;
    }

    if ( esp_log_timestamp() < m_next_read_timestamp || xSemaphoreTake( m_read_exclusion, 0 ) == pdFALSE )
    {
        ESP_LOGW( TAG, "::update - Read exclusion period breach." );
        return false;
    }

    _enquire();    // Read

    m_next_read_timestamp = DHT22_REREAD_INTERVAL_MS + esp_log_timestamp();
    m_status =  _process();

    xSemaphoreGive( m_read_exclusion );

    return (m_status == DHT22_STATUS_OK);
}    // update


float DHT22::getTemperatureCelsius()
{
    return m_last_temperature_celcius;
}


float DHT22::getHumidityPercent()
{
    return m_last_humidity_percent;
}


void DHT22::_enquire()
{
    ESP_LOGD( TAG, "::_enquire" );

    rmt_set_pin( m_channel, RMT_MODE_TX, m_pin );
    gpio_pulldown_en( m_pin );                      // Pull pin LOW
    vTaskDelay( DHT22_PULLDOWN_PERIOD );            // for required time to signal the DHT22
    gpio_pulldown_dis( m_pin );                     // Allow the pin to float so the DHT22 can signal

    rmt_set_pin( m_channel, RMT_MODE_RX, m_pin );    // Put the RMT channel into receive mode
    rmt_rx_start( m_channel, true );                // Start receiving
    vTaskDelay( DHT22_READ_PERIOD );                 // Wait for the DHT22 to complete signaling
    rmt_rx_stop( m_channel );                       // Stop receiving
}    // _read


DHT22::dht22_status_t DHT22::_process()
{
    ESP_LOGD( TAG, "::_process" );

    dht22_status_t status = DHT22_STATUS_UNSET;
    size_t rx_size = 0;
    void* items = static_cast< rmt_item32_t* >( xRingbufferReceive( m_ringBuf, &rx_size, 100 ) );

    if ( items )
    /*
     * Did not timeout, so decode the signals
     */
    {
        status = _decode( static_cast< rmt_item32_t* >( items ), rx_size / sizeof(rmt_item32_t) );
        vRingbufferReturnItem( m_ringBuf, items );

    }

    ESP_LOGD( TAG, "::_process - Status: %d", status );

    return status;
}    // _process


DHT22::dht22_status_t DHT22::_decode( rmt_item32_t* data, int number_of_pulses )
{
    ESP_LOGD( TAG, "::_decode - Pulses: %d", number_of_pulses );

    if ( number_of_pulses < 42 )
    {
        return DHT22_STATUS_UNDERFLOW;
    }

    if ( number_of_pulses > 42 )
    {
        return DHT22_STATUS_OVERFLOW;
    }

    uint32_t interval = data [ 0 ].duration0 + data [ 0 ].duration1;    // Total duration is in 1st position (index 0)

    ESP_LOGD( TAG, "::_decode - Signal interval: %d", interval );

    if ( interval < DHT22_INTERVAL_MIN || interval > DHT22_INTERVAL_MAX )
    /*
     * Should be ~160
     */
    {
        return DHT22_STATUS_NACK;
    }

    uint8_t raw [ 5 ] { 0 };    // New zero'ed result set

    for ( uint8_t i = 1; i <= 40; ++i )
    /*
     * Read 40 pulses corresponding to 40 bits (5 bytes), data is in 2nd position (index 1)
     */
    {
        uint8_t pulse = data [ i ].duration0 + data [ i ].duration1;    // Calculate the pulse length
        uint8_t bit = i - 1;    // Bit position 0 - 39
        uint8_t byte = bit / 8;    // Data byte 0 - 4

        if ( pulse < DHT22_PULSE_MIN || pulse > DHT22_PULSE_MAX ) return DHT22_STATUS_DATA;    // DATA error

        if ( pulse > DHT22_ON_OFF_PULSE_THRESHOLD )
        /*
         * 1 detected, set corresponding bit in raw value
         */
        {
            raw [ byte ] |= 0x80 >> ( bit % 8 );    // Set bit
        }
    }    // for

    if ( raw [ 4 ] != ( ( raw [ 0 ] + raw [ 1 ] + raw [ 2 ] + raw [ 3 ] ) & 0xFF ) )
    /*
     * Checksum error
     */
    {
        return DHT22_STATUS_CHECKSUM;
    }

    /*
     * Results are OK, so set new output values
     */
    m_raw = raw;
    m_last_temperature_celcius = ( ( ( raw [ 2 ] & 0x7F ) << 8 ) | raw [ 3 ] ) * ( ( raw [ 2 ] & 0x80 ) ? -0.1 : 0.1 );
    m_last_humidity_percent = ( ( raw [ 0 ] << 8 ) | raw [ 1 ] ) * 0.1;

    return DHT22_STATUS_OK;    // OK
}    // _decode

