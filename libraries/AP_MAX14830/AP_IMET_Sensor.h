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
 * AP_IMET_Sensor.h
 *
 *      Author: Kyle Fruson
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS_MAVLink.h>



// Forward declaration
class AP_MAX14830;

//------------------------------------------------------------------------------
// IMET Class - UART1
//------------------------------------------------------------------------------

class AP_IMET_Sensor
{
public:
    // Constructor
    AP_IMET_Sensor(AP_HAL::OwnPtr<AP_MAX14830> max14830);

    // Handle UART1 Interrupt
    void handle_imet_uart2_interrupt(void);

    // Handle complete IMET message.
    void handle_complete_imet_msg(const char* message, const uint8_t message_length);

    // Length of bytes to read - returned from Max14830 FIFO.
    uint8_t rxbuf_fifo_len;

private:
    // Pointer to MAX14830 object
    AP_HAL::OwnPtr<AP_MAX14830> _max14830;
    
    //------------------------------------------------------------------------------
    // Parser state for the IMET Messages
    //------------------------------------------------------------------------------
    struct PARSE_STATE
    {
        enum value
        {
            NO_SYNC,                    // Waiting for '0'
            START_SYNC_0,               // Reserved for ','
            START_SYNC_1,               // Reserved for '+'
            WAIT_FOR_STOP_SYNC_1,       // Carriage return (0x0D).
            WAIT_FOR_STOP_SYNC_2        // Line feed (0x0A).
        };
    };

    // Keeps track of the parsing state of the start sync and stop sync
    PARSE_STATE::value parse_state;

};
