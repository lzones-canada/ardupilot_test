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
 * AP_IMET.cpp
 *
 *      Author: Kyle Fruson
 */

#include "AP_IMET.h"
#include <utility>
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

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


//------------------------------------------------------------------------------
// Static index to control previous message length for iterations.
//------------------------------------------------------------------------------

static uint8_t prev_len = 0;


// Constructor
AP_IMET::AP_IMET() 
{
}

void AP_IMET::init()
{
    // Init UART-SPI Max Driver
    _driver.init();

    /* Request 2Hz update (500ms) */
    _driver.register_periodic_callback(500 * AP_USEC_PER_MSEC, FUNCTOR_BIND_MEMBER(&AP_IMET::_timer, void));

    // _drdy_pin = hal.gpio->channel(HAL_GPIO_PIN_DRDY1_BMP388);
    // _drdy_pin->mode(HAL_GPIO_INPUT);
    // _drdy_pin->attach_interrupt(HAL_GPIO_PIN_DRDY1_EXT, trigger_drdy_interrupt, AP_HAL::GPIO::INTERRUPT_RISING);

    return;
}

/* ************************************************************************* */
void AP_IMET::_timer(void)
{
    if(!_driver.poll_global_isr()) {
        return;
    }

    // Handle UART1 Interrupt for IMET data.
    handle_imet_uart1_interrupt();

    // Parse data to readable field for mavlink message??

    // Handle UART2 Interrupt

    return;
}
/* ************************************************************************* */


void AP_IMET::handle_imet_uart1_interrupt()
{
    //---------------------------------------------------------------------------
	// Enumerated data type to define the states of parsing buffer and copying message
	//---------------------------------------------------------------------------
    static enum
	{
		WAIT_FOR_STOP_SYNC_1,		// Carriage return.
		WAIT_FOR_STOP_SYNC_2		// Line feed.
	} parse_state = WAIT_FOR_STOP_SYNC_1;


    // Read Data out of FIFO when interrupt triggered, store current length of FIFO.
    uint8_t rxbuf_fifo_len = _driver.rx_fifo_read(rx_fifo_buffer, MESSAGE_BUFFER_LENGTH);

    // Initialize pointer to the start of FIFO buffer.
    const uint8_t *byte_ptr = &rx_fifo_buffer[0];
    // Initialize buffer pointer.
    rx_buffer_ptr = rx_buffer;
    // Initialize buffer pointer length index.
    uint8_t byte_ptr_len = 0;
    // Initialize previous message length index.
    prev_len = 0;

    // Parse the data in buffer.
    while(true)
    {
        if((rx_buffer + MESSAGE_BUFFER_LENGTH) <= rx_buffer_ptr)
		{
			// If the message won't all fit in the message buffer, it indicates
			//  an error.  Reset and start looking for a new message.
            prev_len = 0;
			rx_buffer_ptr = rx_buffer;
			parse_state = WAIT_FOR_STOP_SYNC_1;
		}

        // Parse all data until the end of the fifo buffer.
        if(rxbuf_fifo_len == byte_ptr_len) {
            // Finished converting all new data, Clear the interrupt and reset buffer.
            _driver.clear_interrupts();
            break;
        }

        // Copy data over to rx buffer.
        *rx_buffer_ptr = *byte_ptr;
        ++rx_buffer_ptr;
        // Track length of buffer pointer.
        ++byte_ptr_len;

        // Wait for a stop sync to be received.
        switch(parse_state)
		{
			case(WAIT_FOR_STOP_SYNC_1):
			{
                // Carriage Return '\r' (0x0D).
				if(0x0D == *byte_ptr)
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
				if(0x0A == *byte_ptr)
				{
                    handle_complete_imet_msg(byte_ptr_len);
				    break;
				}
				// Reset to the start of the message buffer and start looking for a new message.
				parse_state = WAIT_FOR_STOP_SYNC_1;
				break;
			}
			default:
			{
				// This should never happen.
				// Reset to the start of the message buffer and start looking for a new message.
				rx_buffer_ptr = rx_buffer;
                prev_len = 0;
				parse_state = WAIT_FOR_STOP_SYNC_1;
				break;
			}
		}

        // Increment byte index.
        ++byte_ptr;
    }

    return;
}

/* ************************************************************************* */

void AP_IMET::handle_complete_imet_msg(const uint8_t ptr_len)
{
    // Calculate the maximum number of iterations needed.
    uint8_t max_iterations = ptr_len - prev_len;

    // Working buffer point back to start of message. 
    const uint8_t *rx_work_buffer = rx_buffer_ptr - max_iterations;

    // Mavlink packet for sending out data.
    mavlink_imet_sensor_raw_t imet_pkt{};
    uint8_t length = 0;

    // for loop to iterate over complete message.
    for(int i=0; i<max_iterations; i++){
        uint8_t curr_char = *rx_work_buffer;
        // Only add data to the packet if it is not a carriage return or line feed.
        if(curr_char != '\r' && curr_char != '\n'){
            // Copy data into mavlink packet.
            imet_pkt.data[length] = curr_char;
            length++;
        }
        rx_work_buffer++;
    }

     // Save the length
    imet_pkt.data_length = length;

    // Save the index of our latest complete message.
    prev_len = ptr_len;

    // Send out Mavlink Message
    gcs().send_to_active_channels(MAVLINK_MSG_ID_IMET_SENSOR_RAW, (const char *)&imet_pkt);

    return;
}
