/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * AP_IMET_Sensor.cpp
 *
 *      Author: Kyle Fruson
 */

#include "AP_IMET_Sensor.h"
#include "AP_MAX14830.h"

//------------------------------------------------------------------------------
// Static FIFO buffer to retrieve rx message from UART1.
//------------------------------------------------------------------------------
static const uint8_t MESSAGE_BUFFER_LENGTH = 255;
static uint8_t rx_fifo_buffer[MESSAGE_BUFFER_LENGTH];

//------------------------------------------------------------------------------
// Static pointer to control access to the contents of message_buffer.
//------------------------------------------------------------------------------

static uint8_t *rx_buffer_ptr;

//------------------------------------------------------------------------------
// Message buffer for accumulated data for partial storing.
//------------------------------------------------------------------------------
static char    _message_buffer[MESSAGE_BUFFER_LENGTH];
static uint8_t _message_buffer_length;

//------------------------------------------------------------------------------
// Constructor
//------------------------------------------------------------------------------
AP_IMET_Sensor::AP_IMET_Sensor(AP_HAL::OwnPtr<AP_MAX14830> max14830) :
    _max14830(std::move(max14830))
{
    rx_buffer_ptr = rx_fifo_buffer;
    _message_buffer_length = 0;
    parse_state = PARSE_STATE::NO_SYNC;
}

/* ************************************************************************* */

void AP_IMET_Sensor::handle_imet_uart2_interrupt()
{
    // Read Data out of FIFO when interrupt triggered, store current length of FIFO.
    _max14830->set_uart_address(UART::ADDR_2);
    rxbuf_fifo_len = _max14830->rx_read(rx_fifo_buffer, MESSAGE_BUFFER_LENGTH);
    // Clear the interrupt.
    _max14830->clear_interrupts();

    // Pointer to the start of FIFO buffer.
    rx_buffer_ptr = &rx_fifo_buffer[0];

    // For loop to iterate over all the recieve data
    for(int i=0; i<rxbuf_fifo_len; i++)
    {
        // Temporary copy for readability.
        char character = *rx_buffer_ptr;

        // Accumulate data into the message buffer for partial storage.
        _message_buffer[_message_buffer_length++] = character;

        // Wait for a stop sync to be received.
        switch(parse_state)
        {
            case(PARSE_STATE::NO_SYNC):
            {
                // Carriage Return '\r' (0x0D).
                if('0' == character)
                {
                    parse_state = PARSE_STATE::START_SYNC_0;
                }
                // This if statement has no else, as this will be normal while the
                //  contents of the message are being copied to the message buffer.
                break;
            }
            case(PARSE_STATE::START_SYNC_0):
            {
                if(',' == character)
                {
                    parse_state = PARSE_STATE::START_SYNC_1;
                } 
                else
                {
                    // Reset as there was no sync
                    parse_state = PARSE_STATE::NO_SYNC;
                }
                break;
            }
            case(PARSE_STATE::START_SYNC_1):
            {
                if('+' == character)
                {
                    parse_state = PARSE_STATE::WAIT_FOR_STOP_SYNC_1;
                }
                else
                {
                    // Reset as there was no sync
                    parse_state = PARSE_STATE::NO_SYNC;
                }
                break;
            }
            case(PARSE_STATE::WAIT_FOR_STOP_SYNC_1):
            {
                // Carriage Return '\r' (0x0D).
                if('\r' == character)
                {
                    parse_state = PARSE_STATE::WAIT_FOR_STOP_SYNC_2;
                }
                // This if statement has no else, as this will be normal while the
                //  contents of the message are being copied to the message buffer.
                break;
            }
            case(PARSE_STATE::WAIT_FOR_STOP_SYNC_2):
            {
                // Line Feed '\n' (0x0A).
                if('\n' == character)
                {
                    // Reset as the last stop sync byte has been received and message
                    //  has ended.
                    parse_state = PARSE_STATE::WAIT_FOR_STOP_SYNC_1;
                    // Full message received... Process it.
                    handle_complete_imet_msg(_message_buffer, _message_buffer_length);
                    // Reset buffer length
                    _message_buffer_length = 0;
                }
                break;
            }
            default:
            {
                // This should never happen.
                parse_state = PARSE_STATE::NO_SYNC;
                break;
            }
        }

        // increment our byte
        ++rx_buffer_ptr;
    }

    return;
}



void AP_IMET_Sensor::handle_complete_imet_msg(const char* message, const uint8_t message_length)
{
    // Mavlink packet for sending out data.
    mavlink_imet_sensor_raw_t imet_pkt{};
    
    // Set the time since boot in milliseconds.
    imet_pkt.time_boot_ms = AP_HAL::millis();

    // Clear the imet_pkt.data buffer before copying.
    memset(imet_pkt.data, 0, sizeof(imet_pkt.data));

    // Index for filtered data.
    uint8_t data_index = 0;

    // Filter out '\n' and '\r' characters.
    for (uint8_t i = 0; i < message_length; i++) {
        if (message[i] != '\n' && message[i] != '\r') {
            if (data_index < sizeof(imet_pkt.data)) {
                imet_pkt.data[data_index++] = message[i];
            } else {
                // If we exceed the buffer size, break to prevent overflow.
                break;
            }
        }
    }

    // Set the data length after filtering.
    imet_pkt.data_length = data_index;

    // Send out Mavlink Message.
    gcs().send_to_active_channels(MAVLINK_MSG_ID_IMET_SENSOR_RAW, (const char *)&imet_pkt);

    return;
}