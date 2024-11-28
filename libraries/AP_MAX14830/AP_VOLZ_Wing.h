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
 * AP_VOLZ_PROTOCOL.h
 *
 *  Author: Kyle Fruson
 *
 * Baud-Rate: 115.200 bits per second
 * Number of Data bits: 8
 * Number of Stop bits: 1
 * Parity: None
 * Half/Full Duplex: Half Duplex
 *
 * Volz Command and Response are all 6 bytes
 *
 * Command
 * byte	|	Communication Type
 * 1		Command Code
 * 2		Actuator ID
 * 3		Argument 1
 * 4		Argument 2
 * 5		CRC High-byte
 * 6		CRC	Low-Byte
 *
 * byte	|	Communication Type
 * 1		Response Code
 * 2		Actuator ID
 * 3		Argument 1
 * 4		Argument 2
 * 5		CRC High-byte
 * 6		CRC	Low-Byte
 *
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

/*=========================================================================*/
// VOLZ Definitions
/*=========================================================================*/

#define VOLZ_DATA_FRAME_SIZE 6
#define VOLZ_ID              1
#define VOLZ_UART_ADDR       (UART::ADDR_4)
#define VOLZ_NUM_TELEM_TYPES 3

/*=========================================================================*/
// ENUMS
/*=========================================================================*/

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

// Command and response IDs
enum class CMD_ID : uint8_t {
    MOTOR_POWER_CONTROL = 0x9A,
    MOTOR_POWER_CONTROL_RESPONSE = 0x4A,
    READ_POSITION_RAW = 0xE5,
    POSITION_RAW_RESPONSE = 0x45,
    READ_CURRENT = 0xB0,
    CURRENT_RESPONSE = 0x30,
    READ_VOLTAGE = 0xB1,
    VOLTAGE_RESPONSE = 0x31,
    READ_TEMPERATURE = 0xC0,
    TEMPERATURE_RESPONSE = 0x10,
};

// Motor Power Control Arguments
enum PWR_ARG : uint8_t {
    FORWARD_DIRECTION  = 0xFD,
    BACKWARD_DIRECTION = 0xBD,
    IDLE               = 0x00,
};

/*=========================================================================*/


// Forward declaration
class AP_MAX14830;

/*=========================================================================*/
// Volz Class - UART4
/*=========================================================================*/

class AP_VOLZ_Wing
{
public:
    // Constructor
    AP_VOLZ_Wing(AP_HAL::OwnPtr<AP_MAX14830> max14830);

    // Initialize volz sweep wing object
    void init(void);

    // Update function for Volz Wing
    void update(void);

    // Handle VOLZ Interrupt
    void handle_volz_interrupt(void);

private:
    // Pointer to MAX14830 object
    AP_HAL::OwnPtr<AP_MAX14830> _max14830;

    // Structure for Volz State as need frequent access
    AP_VOLZ_State& volz_state;

    // Command frame
    union CMD {
        struct PACKED {
            CMD_ID ID;
            uint8_t actuator_id; // actuator send to or receiving from
            uint8_t arg1; // CMD dependant argument 1
            uint8_t arg2; // CMD dependant argument 2
            uint8_t crc1;
            uint8_t crc2;
        };
        uint8_t data[VOLZ_DATA_FRAME_SIZE];
    };

    // Telemetry structure
    struct {
        CMD_ID types[VOLZ_NUM_TELEM_TYPES] {
            CMD_ID::READ_TEMPERATURE,
            CMD_ID::READ_VOLTAGE,
            CMD_ID::READ_CURRENT,
        };
        uint8_t actuator_id;
        uint8_t request_type;
        CMD cmd_buffer;
        struct {
            // Last response time in milliseconds
            uint32_t last_response_ms;
            // Sensor on main PCB - degrees
            float pcb_temp_raw;
            // Input Voltage
            float input_voltage;
            // Current Consumption
            float current_consump;
        } data;
    } telem;

    // Private functors
    void send_command(CMD &cmd);
    void handle_volz_message(const CMD &cmd);
    void handle_pos_msg(uint8_t arg1, uint8_t arg2);
    void set_servo_command(int16_t power);
    void send_idle(void);
    void send_fwd(uint8_t motor_power);
    void send_rev(uint8_t motor_power);
    void request_position(void);
    void request_current(void);
    void request_telem(void);
    void update_position(void);
    float wing_status_degree(int32_t position);
    int16_t calc_servo_power(int32_t current_pos, uint16_t target_pos);
    uint16_t calc_target_ticks(uint8_t value);
    uint16_t decode_position(uint8_t arg1, uint8_t arg2) const;
    // Return the crc for a given command packet
    uint16_t calculate_volz_crc(const CMD &cmd) const;
    // Write Function for Volz Wing
    bool tx_write(uint8_t *buff, uint16_t len);
    // Return true if the given ID is a valid response
    bool is_response(uint8_t ID) const;

    // Access and setup for Wing Limit Switch.
    AP_HAL::DigitalSource *_wing_limit;

    // Limit the sweep angle
    LimitedValueFilter<float> sweep_angle_limit;

    // initialize the state machine
    State machine_state;

    // wing limit switch pin state
    bool _wing_limit_state;
    // store previous wing limit switch state
    bool _prev_wing_limit_state;

    // variable to track total position of the wing 
    int32_t total_position;
    // variable to store target position
    uint16_t target_position;
    // variable to store the raw position value from the servo
    uint16_t raw_position;
    // variable to store the previous raw position
    uint16_t prev_raw_position;
    // servo power control
    int16_t servo_power;
    // Last wing log time in milliseconds
    uint32_t last_wing_log_ms;
    // Message counter for current request
    uint8_t msg_counter;
    // variable to store current command in degrees
    uint8_t target_command;
    // variable to store the previous position
    uint8_t prev_target_command;
};
