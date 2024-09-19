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
 * AP_VOLZ_Wing.h
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
#include <Filter/LimitedValueFilter.h>
#include "AP_VOLZ_State.h"


#define VOLZ_DATA_FRAME_SIZE		 		6
#define VOLZ_ADDR                       	0x01
#define VOLZ_PWR_CTRL_CMD               	0x9A
#define VOLZ_PWR_CTRL_STAT               	0x4A
#define VOLZ_PWR_FD_CMD                 	0xFD
#define VOLZ_PWR_BD_CMD                 	0xBD
#define VOLZ_PWR_ID_CMD                 	0x00
#define VOLZ_MAX_PWR_CMD                 	0xFF
#define VOLZ_POS_RAW_CMD             	    0xE5
#define VOLZ_POS_RAW_STAT             	    0x45

// Initialize the state machine state
enum State {
    CALIBRATE,
    REQUEST_POSITION,
    WING_LIMIT_HIT,
    CALIBRATE_COMPLETE,
    ACTIVE,
    ACTIVE_REQUEST,
    INIT_REQUEST,
    DEADBAND,
};


// Forward declaration
class AP_MAX14830;

//------------------------------------------------------------------------------
// Volz Class - UART1
//------------------------------------------------------------------------------

class AP_VOLZ_Wing
{
public:
    // Constructor
    AP_VOLZ_Wing(AP_HAL::OwnPtr<AP_MAX14830> max14830);

    // Initialize volz sweep wing object
    void init(void);

    // Update function for Volz Wing
    void update(void);

    // Handle VOLZ-UART1 Interrupt
    void handle_volz_uart1_interrupt(void);

private:
    // Pointer to MAX14830 object
    AP_HAL::OwnPtr<AP_MAX14830> _max14830;

    // Structure for Volz State as need frequent access
    AP_VOLZ_State& volz_state;

    // Private functors
    void send_command(uint8_t data[VOLZ_DATA_FRAME_SIZE]);
    void handle_volz_message(uint8_t* rx_work_buffer);
    void handle_pos_msg(uint8_t data[VOLZ_DATA_FRAME_SIZE]);
    void set_servo_command(int16_t command);
    void send_idle(void);
    void send_fwd(uint8_t command);
    void send_rev(uint8_t command);
    void request_position(void);
    void update_position(void);
    float wing_status_percent(int32_t position);
    float wing_status_degree(int32_t position);
    int16_t calc_servo_command(int32_t current_pos, uint16_t target_pos);
    uint16_t calc_target_ticks(uint8_t value);
    uint16_t decode_position(uint8_t arg1, uint8_t arg2);
    uint16_t calc_volz_crc(uint8_t data[VOLZ_DATA_FRAME_SIZE]);
    // Write Function for Volz Wing
    bool tx_write(uint8_t *buffer, uint16_t length);

    // Access and setup for Wing Limit Switch.
    AP_HAL::DigitalSource *_wing_limit;

    // Limit the sweep angle
    LimitedValueFilter<float> sweep_angle_limit;

    // wing limit switch pin state
    bool _wing_limit_state;
    // store previous wing limit switch state
    bool _prev_wing_limit_state;

    // initialize the state machine
    State machine_state;

    // Length of bytes to read - returned from Max14830 FIFO.
    uint8_t rxbuf_fifo_len;

    // variable to store our current offset from the init position
    uint16_t offset;
    // variable to track total position of the wing 
    int32_t total_position;
    // variable to store target position
    uint16_t target_position;
    // variable to store the raw position value from the servo
    uint16_t raw_position;
    // variable to store the previous raw position
    uint16_t prev_raw_position;
    // current servo command
    int16_t servo_cmd;

    uint32_t last_wing_log_ms;

    // variable to store current command in degrees
    uint8_t target_command;
    // variable to store the previous position
    uint8_t prev_target_command;
};
