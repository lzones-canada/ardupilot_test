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
// Static buffer to store the message to be converted in.
//------------------------------------------------------------------------------

static uint8_t rx_buffer[MESSAGE_BUFFER_LENGTH];


//------------------------------------------------------------------------------
// Static pointer to control access to the contents of message_buffer.
//------------------------------------------------------------------------------

static uint8_t *rx_buffer_ptr = rx_buffer;
static uint8_t  rx_buffer_len = 0;

//------------------------------------------------------------------------------
// Constructor
//------------------------------------------------------------------------------
AP_IMET_Sensor::AP_IMET_Sensor(AP_MAX14830* max14830)
    : _max14830(max14830)
{
}

/* ************************************************************************* */

void AP_IMET_Sensor::handle_imet_uart2_interrupt()
{
    //---------------------------------------------------------------------------
	// Enumerated data type to define the states of parsing buffer and copying message
	//---------------------------------------------------------------------------
    static enum
	{
		WAIT_FOR_STOP_SYNC_1,		// Carriage return (0x0D).
		WAIT_FOR_STOP_SYNC_2		// Line feed (0x0A).
	} parse_state = WAIT_FOR_STOP_SYNC_1;

    // Read Data out of FIFO when interrupt triggered, store current length of FIFO.
    _max14830->set_uart_address(UART::ADDR_2);
    rxbuf_fifo_len = _max14830->rx_read(rx_fifo_buffer, MESSAGE_BUFFER_LENGTH);
    // Clear the interrupt.
    _max14830->clear_interrupts();

    // Pointer to the start of FIFO buffer.
    const uint8_t *byte_ptr = &rx_fifo_buffer[0];
    // Byte length to track the number of bytes parsed.
    uint8_t byte_count = 0;

    // Parse the data in buffer.
    while(true)
    {
        if((rx_buffer + MESSAGE_BUFFER_LENGTH) <= rx_buffer_ptr)
		{
			// If the message won't all fit in the message buffer, it indicates
			//  an error.  Reset and start looking for a new message.
			rx_buffer_ptr = rx_buffer;
			rx_buffer_len = 0;
			parse_state = WAIT_FOR_STOP_SYNC_1;
		}

        // Parse all data until the end of the fifo buffer.
        if(rxbuf_fifo_len == byte_count) {
            // Finished converting all new data.
            break;
        }

        // Copy data over to rx buffer.
        *rx_buffer_ptr = *byte_ptr;
        // Increment length of our buffer.
        rx_buffer_len++;

        // Track length of total message.
        byte_count++;

        // Wait for a stop sync to be received.
        switch(parse_state)
		{
			case(WAIT_FOR_STOP_SYNC_1):
			{
                // Carriage Return '\r' (0x0D).
				if('\r' == *byte_ptr)
				{
					parse_state = WAIT_FOR_STOP_SYNC_2;
				}
				// This if statement has no else, as this will be normal while the
				//  contents of the message are being copied to the message buffer.
				break;
			}
			case(WAIT_FOR_STOP_SYNC_2):
			{
                // Line Feed '\n' (0x0A).
				if('\n' == *byte_ptr)
				{
                    // Full messaged received... Process it.
                    handle_complete_imet_msg(rx_buffer_len);
				}
				// Reset to the start of the message buffer and start looking for a new message.
                rx_buffer_ptr = rx_buffer;
                rx_buffer_len = 0;
				parse_state = WAIT_FOR_STOP_SYNC_1;
				break;
			}
			default:
			{
				// This should never happen.
				// Reset to the start of the message buffer and start looking for a new message.
				rx_buffer_ptr = rx_buffer;
                rx_buffer_len = 0;
				parse_state = WAIT_FOR_STOP_SYNC_1;
				break;
			}
		}

        // Increment our buffer indices.
        byte_ptr++;
        rx_buffer_ptr++;
    }

    return;
}

/* ************************************************************************* */

void AP_IMET_Sensor::handle_complete_imet_msg(const uint8_t ptr_len)
{
    // Copy over into Working Buffer
    //  point buffer back to start of message. 
    const uint8_t *rx_work_buffer = (rx_buffer_ptr - (ptr_len - 1));

    // Mavlink packet for sending out data.
    mavlink_imet_sensor_raw_t imet_pkt{};

    // for loop to iterate over complete message.
    for(int i=0; i<ptr_len; i++)
    {
        uint8_t curr_char = *rx_work_buffer;
        // Only add data to the packet if it is not a carriage return or line feed.
        if(curr_char != '\r' && curr_char != '\n') {
            // Copy data into mavlink packet, Save the length.
            imet_pkt.data[imet_pkt.data_length++] = curr_char;
        }
        rx_work_buffer++;
    }

    // Send out Mavlink Message
    gcs().send_to_active_channels(MAVLINK_MSG_ID_IMET_SENSOR_RAW, (const char *)&imet_pkt);

    return;
}