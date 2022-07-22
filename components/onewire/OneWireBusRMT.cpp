/**
 *  @file       OneWireBusRMT.cpp
 *  @version    0.1.0
 *
 *  @brief      ESP32 RMT implementation of Dallas 1-Wire Bus protocol driver
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

#include "OneWireBusRMT.h"

using namespace epd;

// -- Helper forward declarations

static rmt_item32_t RMT_PULSE( uint16_t duration0, bool level0, uint16_t duration1, bool level1 );

static void U8_TO_RMT( const void* src, rmt_item32_t* dest, size_t src_size, size_t wanted_num, size_t* translated_size,
        size_t* item_num );

// -- 

static rmt_item32_t OWR_RESET =    //!< RMT Reset Pulse Definition
        RMT_PULSE( OneWireBusRMT::OWR_TICKS_PER_US * OneWireBus::OW_RESET_TIME_US, 0, 0, 1 );

static rmt_item32_t OWR_READ_SLOT =    //!< RMT Read Slot Pulse Definition
        RMT_PULSE( OneWireBusRMT::OWR_TICKS_PER_US * OneWireBus::OW_READ_SLOT_LOW_US, 0,
                OneWireBusRMT::OWR_TICKS_PER_US * ( OneWireBus::OW_SLOT_TIME_US - OneWireBus::OW_READ_SLOT_LOW_US ),
                1 );

static rmt_item32_t OWR_WRITE_SLOT_0 =    //!< RMT Write Zero Slot Pulse Definition
        RMT_PULSE( OneWireBusRMT::OWR_TICKS_PER_US * OneWireBus::OW_WRITE_SLOT_0_LOW_US, 0,
                OneWireBusRMT::OWR_TICKS_PER_US * OneWireBus::OW_WRITE_SLOT_0_HIGH_US, 1 );

static rmt_item32_t OWR_WRITE_SLOT_1 =    //!< RMT Write One Slot Pulse Definition
        RMT_PULSE( OneWireBusRMT::OWR_TICKS_PER_US * OneWireBus::OW_WRITE_SLOT_1_LOW_US, 0,
                OneWireBusRMT::OWR_TICKS_PER_US * OneWireBus::OW_WRITE_SLOT_1_HIGH_US, 1 );


// -- Constructors

OneWireBusRMT::OneWireBusRMT( uint8_t rx_pin, uint8_t tx_pin, uint8_t rx_channel, uint8_t tx_channel ) :
        OneWireBusRMT( static_cast< gpio_num_t >( rx_pin ), static_cast< gpio_num_t >( tx_pin ),
                static_cast< rmt_channel_t >( rx_channel ), static_cast< rmt_channel_t >( tx_channel ) )
{
}

OneWireBusRMT::OneWireBusRMT( gpio_num_t rx_pin, gpio_num_t tx_pin, rmt_channel_t rx_channel, rmt_channel_t tx_channel )
{
    m_rx_pin = rx_pin;
    m_rx_channel = rx_channel;
    m_tx_pin = tx_pin;
    m_tx_channel = tx_channel;
    if ( ( tx_channel - rx_channel ) != 1 )
    ESP_LOGE( TAG, "::OneWireBusRMT - RMT Receive channel and Transmit channel are not consecutive or in that order." );
    ESP_LOGI( TAG, "::OneWireBusRMT - Clock Divider:%d  Ticks per US:%d  Idle Threshold US:%d", OWR_CLK_DIV,
            OWR_TICKS_PER_US, OWR_RX_IDLE_THRESHOLD_US );

}

OneWireBusRMT::~OneWireBusRMT()
{
    rmt_driver_uninstall( m_tx_channel );
    rmt_driver_uninstall( m_rx_channel );
    vRingbufferDelete( m_rx_ringBuf );
}

// --

void OneWireBusRMT::initialize()
{
    ESP_ERROR_CHECK( gpio_set_direction( m_rx_pin, GPIO_MODE_INPUT_OUTPUT ) );      // RX
    ESP_ERROR_CHECK( gpio_set_pull_mode( m_rx_pin, GPIO_PULLUP_ONLY ) );            // Pullup only
    ESP_ERROR_CHECK( gpio_set_direction( m_tx_pin, GPIO_MODE_OUTPUT_OD ) );         // TX & Drain
    ESP_ERROR_CHECK( gpio_set_pull_mode( m_tx_pin, GPIO_PULLUP_ONLY ) );            // Drain only

    /* ----------------------------------------------------------------------
     * RX
     * ----------------------------------------------------------------------
     */
    rmt_config_t rmt_rx;
    rmt_rx.channel = m_rx_channel;
    rmt_rx.gpio_num = m_rx_pin;
    rmt_rx.clk_div = OWR_CLK_DIV;                               // Set interval time
    rmt_rx.mem_block_num = 2;                                   // Use 1 memory block
    rmt_rx.rmt_mode = RMT_MODE_RX;                              // Start off in Receive mode
    rmt_rx.rx_config.filter_en = true;                          // Filter out pulses below threshold
    rmt_rx.rx_config.filter_ticks_thresh = 2 + OWR_CLK_PWR;     // Ignore pulses shorter than threshold
    rmt_rx.rx_config.idle_threshold = OWR_RX_IDLE_THRESHOLD_US * OWR_TICKS_PER_US;
    ESP_ERROR_CHECK( rmt_config( &rmt_rx ) );

    ESP_ERROR_CHECK( rmt_driver_install( m_rx_channel, 1023, 0 ) );     // RMT data is cycled into a RingBuffer

    /*
     * Check the RX clock - Confirm we are using the APB clock
     */
    rmt_source_clk_t rmt_source_clk;
    ESP_ERROR_CHECK( rmt_get_source_clk( m_rx_channel, &rmt_source_clk ) );
    if ( rmt_source_clk != RMT_BASECLK_APB )
    {
        ESP_LOGE( TAG, "::initialize - RX RMT is not using APB clock. Clock # is: %d", rmt_source_clk );
    }

    ESP_ERROR_CHECK( rmt_get_ringbuf_handle( m_rx_channel, &m_rx_ringBuf ) );

    /* ----------------------------------------------------------------------
     * TX
     * ----------------------------------------------------------------------
     */
    rmt_config_t rmt_tx;
    rmt_tx.rmt_mode = RMT_MODE_TX;                  // Start off in Receive mode
    rmt_tx.channel = m_tx_channel;
    rmt_tx.gpio_num = m_tx_pin;
    rmt_tx.clk_div = OWR_CLK_DIV;                   // Set interval time
    rmt_tx.mem_block_num = 1;                       // Use 1 memory block
    rmt_tx.tx_config.loop_en = false;
    rmt_tx.tx_config.carrier_en = false;
    rmt_tx.tx_config.idle_level = RMT_IDLE_LEVEL_HIGH;
    rmt_tx.tx_config.idle_output_en = true;



    ESP_ERROR_CHECK( rmt_config( &rmt_tx ) );

    ESP_ERROR_CHECK( rmt_driver_install( m_tx_channel, 0, 0 ) );        // RMT data is cycled into a 512 byte RingBuffer
    ESP_ERROR_CHECK( rmt_translator_init( m_tx_channel, U8_TO_RMT ) );  // Set full-byte translator

    /*
     * Check the TX clock - Confirm we are using the APB clock
     */
    ESP_ERROR_CHECK( rmt_get_source_clk( m_tx_channel, &rmt_source_clk ) );
    if ( rmt_source_clk != RMT_BASECLK_APB )
    {
        ESP_LOGE( TAG, "::initialize - TX RMT is not using APB clock. Clock is: %d", rmt_source_clk );
    }

    ESP_LOGD( TAG, "::initialize - RMT device on pins %d, %d / channels %d, %d initialized.", m_rx_pin, m_tx_pin,
            m_rx_channel, m_tx_channel );

}    // initialize


bool OneWireBusRMT::_bus_reset( onewire_pulses_t& pulses )
{
    ESP_LOGD( TAG, "::_bus_reset" );

    ESP_ERROR_CHECK( rmt_rx_start( m_rx_channel, true ) );                      // Start reading - we want the reset pulse too
    ESP_ERROR_CHECK( rmt_write_items( m_tx_channel, &OWR_RESET, 1, true ) );    // Reset pulse
    _probe( pulses );                                                           // Take the pulses off the wire
    rmt_rx_stop( m_rx_channel );    // RX Stop has to be after we read from the wire to work
    return true;
}    // _bus_reset


bool OneWireBusRMT::_strong_pullup( uint8_t pullup_duration_us )
{
    ESP_LOGD( TAG, "::_strong_pullup" );

//    rmt_item32_t strong_pullup_pulse = RMT_PULSE( pullup_duration_us, true, OW_RECOVERY_TIME_US, true );
//
//    ESP_ERROR_CHECK( rmt_rx_start( m_rx_channel, true ) );                // Start reading - we want the reset pulse too
//    ESP_ERROR_CHECK( rmt_write_items( m_tx_channel, &strong_pullup_pulse, 1, true ) );    // Reset pulse
//    _probe (pulses);                                                           // Take the pulses off the wire
//    rmt_rx_stop( m_rx_channel );    // RX Stop has to be after we read from the wire to work
    return true;
}    // _strong_pullup


bool OneWireBusRMT::_write_slots( uint16_t bits, const onewire_data_t& data )
{
    ESP_LOGV( TAG, "::_write_slots - Start\n\t Writing %d bits, first byte %d", bits, data [ 0 ] );

    uint8_t wholebytes = bits / 8;
    uint8_t boundarybits = bits % 8;

    if ( wholebytes > 0 )
    /*
     * Write any whole whole bytes with rmt_write_sample blocking only if no boundary bits
     */
    {

        ESP_LOGD( TAG, "::_write_slots - Bytes - block:%d", ( boundarybits == 0 ) );
        ESP_ERROR_CHECK( rmt_write_sample( m_tx_channel, &data [ 0 ], wholebytes, ( boundarybits == 0 ) ) );
    }

    if ( boundarybits > 0 )
    /*
     * Write any boundary bits not associated with a whole byte. Block until complete.
     */
    {
        rmt_item32_t* rmt_items = (rmt_item32_t*) malloc( boundarybits * sizeof(rmt_item32_t) );
        for ( uint8_t i = 0; i < boundarybits; i++ )
        {
            if ( data [ wholebytes ] & ( 0x01 << i ) )
            {
                rmt_items [ i ].val = OWR_WRITE_SLOT_1.val;
            }
            else
            {
                rmt_items [ i ].val = OWR_WRITE_SLOT_0.val;
            }
        }
        ESP_ERROR_CHECK( rmt_write_items( m_tx_channel, rmt_items, boundarybits, true ) );
        free( rmt_items );
    }

    // probe();
    ESP_LOGV( TAG, "::_write_slots - End" );
    return true;
}    // _write_slots


bool OneWireBusRMT::_read_slots( uint16_t bits, onewire_data_t& data )
{
    ESP_LOGV( TAG, "::_read_slots - Start\n\tReading %d bits", bits );

    rmt_item32_t* rmt_items = (rmt_item32_t*) malloc( bits * sizeof(rmt_item32_t) );
    onewire_pulses_t pulses;

    for ( uint16_t i = 0; i < bits; i++ )
    /*
     * Generate the read slot waveform
     */
    {
        rmt_items [ i ].val = OWR_READ_SLOT.val;
    }

    ESP_ERROR_CHECK( rmt_rx_start( m_rx_channel, true ) );    // Read before writing as the device will modify the write slot
    ESP_ERROR_CHECK( rmt_write_items( m_tx_channel, rmt_items, bits, true ) );
    _probe( pulses );
    ESP_ERROR_CHECK( rmt_rx_stop( m_rx_channel ) );

    bool result = ( _unmarshal_pulses( pulses, data, FAIL ) == bits );

    ESP_LOG_BUFFER_HEX_LEVEL( TAG, &data, ( bits / 8 ) + 1, ESP_LOG_VERBOSE );

    free( rmt_items );
    ESP_LOGV( TAG, "::_read_slots - End" );
    return result;
}    // _read_slots


size_t OneWireBusRMT::_probe( onewire_pulses_t& pulses )
{
    ESP_LOGV( TAG, "::_probe - Start" );

    size_t rx_size = 0;      // Size of the items received
    size_t index { 0 };    // output index

    rmt_item32_t* items =    // The RMT RX data
            static_cast< rmt_item32_t* >( xRingbufferReceive( m_rx_ringBuf, &rx_size, 100 ) );

    if ( items )
    /*
     * Did not timeout, so decode the signals
     */
    {
        ESP_LOGV( TAG, "::_probe - Received %d items.", rx_size );

        pulses.push_back( 0 );

        for ( int i = 0; i < rx_size; i++ )
        {
            ESP_LOGV( TAG, "::_probe - Item:%d   %d:%d us\t%d:%d us", i, items [ i ].level0,
                    items [ i ].duration0 >> OWR_CLK_PWR, items [ i ].level1, items [ i ].duration1 >> OWR_CLK_PWR );

            _process_pulse( pulses, items [ i ].level0, items [ i ].duration0 >> OWR_CLK_PWR );    // Append pulse
            _process_pulse( pulses, items [ i ].level1, items [ i ].duration1 >> OWR_CLK_PWR );    // Append pulse
            index++;
            if ( items [ i ].level1 == 1 && items [ i ].duration1 == 0 ) break;    // Final value typically has a duration of 0 on second value
        }    // for

        vRingbufferReturnItem( m_rx_ringBuf, items );    // Release the ring buffer space
    }    // if
    
    ESP_LOGV( TAG, "::_probe - End" );

    return index;
}    // _probe


/* ---------------------------------------------------
 * Helpers
 * ---------------------------------------------------
 */


/**
 * @brief Creates a new rmt_item32_t
 * 
 * @param duration0 
 * @param level0 
 * @param duration1 
 * @param level1 
 * @return rmt_item32_t 
 */
static rmt_item32_t RMT_PULSE( uint16_t duration0, bool level0, uint16_t duration1, bool level1 )
{
    rmt_item32_t pulse;
    pulse.duration0 = duration0;
    pulse.level0 = level0;
    pulse.duration1 = duration1;
    pulse.level1 = level1;
    return pulse;
}    // _rmt_pulse


/**
 * @brief Translator for uint8_t type of data to rmt format data.
 *
 * Translates whole bytes only.
 * This is a choice as if src size exceeds wanted num, the system code may be unpredictable.
 * So sticking to the example methodology.
 *
 * @see ESP_IDF RMT examples
 *
 * @param src the source uint8_t bytes
 * @param dest the item32s to transmit
 * @param src_size the number of source bytes
 * @param wanted_num the buffer size in rmt_item32_t apparently
 * @param translated_size the number of bytes translated
 * @param item_num the number of translated rmt_item32_t
 */
static void U8_TO_RMT( const void* src, rmt_item32_t* dest, size_t src_size, size_t wanted_num, size_t* translated_size,
        size_t* item_num )
{
    ESP_LOGV( "OneWireBusRMT", "::U8_TO_RMT - Start\n\tSrc Size:%d   Wanted Num:%d", src_size, wanted_num );

    if ( src == nullptr || dest == nullptr )
    {
        *translated_size = 0;
        *item_num = 0;
        return;
    }

    size_t size = 0;
    size_t num = 0;
    uint8_t *psrc = (uint8_t *) src;
    rmt_item32_t* pdest = dest;

    while ( size < src_size && num < wanted_num )
    /*
     * Bytes and bits are under the two limits
     */
    {
        for ( int i = 0; i < 8; i++ )
        /*
         * Process each bit in the current byte
         */
        {
            if ( *psrc & ( 0x1 << i ) )
            {
                pdest->val = OWR_WRITE_SLOT_1.val;
            }
            else
            {
                pdest->val = OWR_WRITE_SLOT_0.val;
            }

            num++;
            pdest++;
        }    // for

        size++;
        psrc++;
    }    // while

    *translated_size = size;
    *item_num = num;
}    // u8_to_rmt

