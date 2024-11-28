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
 * AP_VOLZ_Wing.cpp
 *
 *      Author: Kyle Fruson
 */

#include "AP_VOLZ_Wing.h"
#include "AP_MAX14830.h"


//------------------------------------------------------------------------------
// Static FIFO buffer to retrieve rx message from Volz Wing UART1 FIFO
//------------------------------------------------------------------------------
static const uint8_t MESSAGE_BUFFER_LENGTH = 255;
static uint8_t rx_fifo_buffer[MESSAGE_BUFFER_LENGTH];


#define SCALING_FACTOR     (360.0 / 4096.0)
#define TOTAL_REVS         (8.6)
#define TICKS_PER_REV      (4096)
#define TOTAL_TICKS        (TICKS_PER_REV * TOTAL_REVS)
// 0.5 Degree Threshold (460 Ticks / Degree)
 //  460 / 2 = 230 Ticks * 0.1 = 23 Ticks
#define THRESHOLD_POSITION (TICKS_PER_REV / 9)
#define MAX_POWER          (0xFF)


//------------------------------------------------------------------------------
// Constructor
//------------------------------------------------------------------------------
AP_VOLZ_Wing::AP_VOLZ_Wing(AP_HAL::OwnPtr<AP_MAX14830> max14830) :
    _max14830(std::move(max14830)),
    sweep_angle_limit(WING_MAX_DEGREES, WING_MIN_DEGREES),
    volz_state(AP_VOLZ_State::get_singleton())
{
}

//------------------------------------------------------------------------------

void AP_VOLZ_Wing::init()
{
    // wing limit servo pin
    _wing_limit = hal.gpio->channel(HAL_GPIO_PIN_WING_LIMIT);
    _wing_limit->mode(HAL_GPIO_INPUT);

    // init states
    _wing_limit_state = false;
    _prev_wing_limit_state = false;
    // init decoded pos
    raw_position = 0;
    prev_raw_position = 0;
    total_position = 0;
    target_position = 0;
    servo_power = 0;
    msg_counter = 0;

    // Init target command to min degrees (fully open)
    volz_state.set_target_command(WING_MIN_DEGREES);
    prev_target_command = WING_MIN_DEGREES;
    target_command = 0;

    // Init state machine.
    machine_state = INIT_REQUEST;

    return;
}


//------------------------------------------------------------------------------
// Periodic update to handle volz sweep wing logic.
//------------------------------------------------------------------------------
void AP_VOLZ_Wing::update()
{
    // Update the sweep wing angle every iteration.
    update_position();


    // --------------------------------------------------------------
    // Wing limit switch logic
    // --------------------------------------------------------------
    _wing_limit_state = _wing_limit->read();

    // Transition to the next state after a certain condition (e.g., after a number of iterations)
    if (_prev_wing_limit_state != _wing_limit_state) {
        // Send out idle command from false to true transition
        if(_wing_limit_state) {
            machine_state = WING_LIMIT_HIT;
        }
    }
    // store previous wing limit switch state
    _prev_wing_limit_state = _wing_limit_state;

    // -----------------------------------------------

	if(volz_state.get_calibrate())
	{
        // Transition to the CALIBRATE state
        machine_state = CALIBRATE;
        // Manually flip back the state to false to allow for re-triggering.
        volz_state.set_calibrate(false);
	}

    // -----------------------------------------------

    // Retrieve the target command from the ground station
    target_command = volz_state.get_target_command();
    // Check if the target deviates more than 1% from our previous target 
    if (prev_target_command != target_command) {
        // Calculate the target position in ticks
        target_position = calc_target_ticks(target_command);
        // Scale the target with threshold which increases our accuracy
        if(target_position - total_position >= THRESHOLD_POSITION && target_command < WING_MAX_DEGREES) {
            target_position += (THRESHOLD_POSITION / 3);
        }
        else if(target_position - total_position < -THRESHOLD_POSITION && target_command > WING_MIN_DEGREES) {
            target_position -= (THRESHOLD_POSITION / 3);
        }

        // Fully open commanded, just treat as a CALIBRATE to reset all values and counters.
        if(target_command == static_cast<uint8_t>(WING_MIN_DEGREES)){
            machine_state = CALIBRATE;
        }
        else {
            // Transition to the ACTIVE_REQUEST state
            machine_state = ACTIVE_REQUEST;
        }
    }
    // Update the previous commanded target
    prev_target_command = target_command;

    // -----------------------------------------------

    // State machine to handle the wing limit switch
    switch (machine_state) 
    {
        case CALIBRATE:
            // Begin rotation of our Servo
            set_servo_command(MAX_POWER);
            break;

        case REQUEST_POSITION:
            // Sit patiently and wait for the wing limit switch to be triggered..
            // Request Position from the servo to update position.
            request_position();
            break;

        case WING_LIMIT_HIT:
            // Immediately send idle command once the wing limit switch is triggered
            send_idle();
            break;

        case CALIBRATE_COMPLETE:
            // Send out a position request
            request_position();
            break;

        case ACTIVE:
            // This calculates our servo direction command based on the current and target positions
            servo_power = calc_servo_power(total_position, target_position);
            // Send out the servo command
            set_servo_command(servo_power);
            break;

        case ACTIVE_REQUEST:
            // Request Position from the servo to update the current position
            request_position();
            break;

        case INIT_REQUEST:
            // Request our current position while we are doing nothing to init an offset..
            request_position();
            break;

        case DEADBAND: {
            // Request telem data while doing nothing...
            uint32_t tnow = AP_HAL::millis();
            if (tnow - telem.data.last_response_ms > 300) {
                // Every 300ms
                request_telem();
                // Update timestamp for the next request cycle
                telem.data.last_response_ms = tnow;
            }
            break;
        }

        default:
            break;
    }
    
    return;
}

/* ************************************************************************* */

void AP_VOLZ_Wing::request_telem() {
    // Assemble the command based on the current telemetry request type
    CMD cmd {};
    cmd.ID = telem.types[telem.request_type];
    cmd.actuator_id = VOLZ_ID;

    // Send the command
    send_command(cmd);

    // Increment the request type
    telem.request_type++;
    // Cycle back to the first telemetry type
    if (telem.request_type >= VOLZ_NUM_TELEM_TYPES) {
        telem.request_type = 0;
    }

    return;
}

/* ************************************************************************* */

// Return true if the given ID is a valid response
bool AP_VOLZ_Wing::is_response(uint8_t ID) const
{
    switch(ID) {
        case (uint8_t)CMD_ID::MOTOR_POWER_CONTROL_RESPONSE:
        case (uint8_t)CMD_ID::POSITION_RAW_RESPONSE:
        case (uint8_t)CMD_ID::CURRENT_RESPONSE:
        case (uint8_t)CMD_ID::VOLTAGE_RESPONSE:
        case (uint8_t)CMD_ID::TEMPERATURE_RESPONSE:
            return true;

        default:
            break;
    }

    return false;
}

/* ************************************************************************* */

void AP_VOLZ_Wing::handle_volz_interrupt()
{
    // Read FIFO data
    _max14830->set_uart_address(VOLZ_UART_ADDR);
    const uint16_t bytes_read = _max14830->rx_read(rx_fifo_buffer, MESSAGE_BUFFER_LENGTH);
    // Clear the interrupt.
    _max14830->clear_interrupts();

    // Validate received data length
    if (bytes_read < VOLZ_DATA_FRAME_SIZE) {
        return;  // Remove unnecessary buffer reset
    }

     // Copy data into telem.cmd_buffer
    memcpy(telem.cmd_buffer.data, rx_fifo_buffer, bytes_read);
    //memmove(telem.cmd_buffer.data, rx_fifo_buffer, bytes_read);  // Works but slower

     // Check for valid response start byte
    if (is_response(telem.cmd_buffer.data[0])) {
        // Calc CRC and compare with received CRC
        uint16_t received_crc = UINT16_VALUE(telem.cmd_buffer.crc1, telem.cmd_buffer.crc2);
        uint16_t calculated_crc = calculate_volz_crc(telem.cmd_buffer);
        // Process message only on CRC match
         if(received_crc == calculated_crc) {
            // Process message directly from rx_buffer
            handle_volz_message(telem.cmd_buffer);
        }
    }
    return;
}

/* ************************************************************************* */

void AP_VOLZ_Wing::handle_volz_message(const CMD &cmd)
{
    // Confirm our received address is for the Volz Wing
    if(cmd.actuator_id == VOLZ_ID) {
        // Handle message
        switch (cmd.ID) {
            case CMD_ID::POSITION_RAW_RESPONSE:
                // Handle position message
                handle_pos_msg(cmd.arg1, cmd.arg2);
                // Flip flop between the command and request state.
                if(machine_state == ACTIVE_REQUEST) {
                    // Transition to the next state after the wing limit state transitions back to false
                    machine_state = ACTIVE;
                }
                if(machine_state == CALIBRATE_COMPLETE) {
                    // Finished Calibrating and acknowledge, set our position and transition state.
                    // Set our position to 0 as we are at the wing limit
                    total_position = 0;
                    // Set prev to current on postion.
                    prev_raw_position = raw_position;
                    // Set our internal states to the min value
                    volz_state.set_target_command(WING_MIN_DEGREES);
                    prev_target_command = WING_MIN_DEGREES;
                    // Transition to the next state after the wing limit state transitions back to false
                    machine_state = DEADBAND;
                }
                if(machine_state == INIT_REQUEST) {
                    // Transition out
                    machine_state = DEADBAND;
                }
                break;
            
            case CMD_ID::MOTOR_POWER_CONTROL_RESPONSE:
                // FD Acknowledgement
                if(cmd.arg1 == PWR_ARG::FORWARD_DIRECTION) {
                    if(machine_state == CALIBRATE) {
                        // Acknowledgement for power control command, move ahead state machine
                        machine_state = REQUEST_POSITION;
                    }
                    // Power control with valid motor power
                    if(machine_state == ACTIVE && cmd.arg2 != 0) {
                        // Acknowledgement for power control command, move ahead state machine
                        machine_state = ACTIVE_REQUEST;
                    }
                    // Power control with no motor power being sent
                    if(machine_state == ACTIVE && cmd.arg2 == 0) {
                        // Transition into deadband state with no motor power being sent
                        machine_state = DEADBAND;
                    }
                }
                // BD Acknowledgement
                if(cmd.arg1 == PWR_ARG::BACKWARD_DIRECTION) {
                    // Power control with valid motor power
                    if(machine_state == ACTIVE && cmd.arg2 != 0) {
                        // Acknowledgement for power control command, move ahead state machine
                        machine_state = ACTIVE_REQUEST;
                    }
                    // Power control with no motor power being sent
                    if(machine_state == ACTIVE && cmd.arg2 == 0) {
                        // Transition into deadband state with no motor power being sent
                        machine_state = DEADBAND;
                    }
                }
                // Idle Acknowledgement
                if(cmd.arg1 == PWR_ARG::IDLE) {
                    if(machine_state == WING_LIMIT_HIT) {
                        // Acknowledgement for power control command, move ahead state machine
                        machine_state = CALIBRATE_COMPLETE;
                    }
                }
                break;

            case CMD_ID::TEMPERATURE_RESPONSE:
                // Temperature is reported relative to -50 deg C
                telem.data.pcb_temp_raw = -50.0 + cmd.arg1;
                volz_state.set_pcb_temp(telem.data.pcb_temp_raw);
                break;

            case CMD_ID::VOLTAGE_RESPONSE:
                // Voltage is reported in 200mv increments (0.2v)
                telem.data.input_voltage = 0.2 * cmd.arg1;
                volz_state.set_voltage(telem.data.input_voltage);
                break;

            case CMD_ID::CURRENT_RESPONSE:
                // Current is reported in 20mA increments (0.02A)
                telem.data.current_consump = 0.02 * cmd.arg1;
                volz_state.set_current(telem.data.current_consump);
                break;

            default:
                // This should never happen
                break;
        }
    }

    return;
}

/* ************************************************************************* */

void AP_VOLZ_Wing::handle_pos_msg(uint8_t arg1, uint8_t arg2)
{
    // Parse the data & extract/decode position data
    raw_position = decode_position(arg1, arg2);

    // Calculate the angle difference between current and previous positions
    int16_t angle_diff = raw_position - prev_raw_position;

    // Account for full rotations
    if (angle_diff < -TICKS_PER_REV / 2) {
        // Decrement spin count for full rotations in the forward direction
        total_position -= TICKS_PER_REV;
    } else if (angle_diff > TICKS_PER_REV / 2) {
        // Increment spin count for full rotations in the backward direction
        total_position += TICKS_PER_REV;
    }
    // Increment spin count for forward rotation or decrement for backward rotation
    total_position -= angle_diff;

    // Update previous position for the next iteration
    prev_raw_position = raw_position;

    return;
}

/* ************************************************************************* */

uint16_t AP_VOLZ_Wing::decode_position(uint8_t arg1, uint8_t arg2) const
{
    // Extracting individual nibbles from the arguments
    uint8_t A = HIGH_NIBBLE(arg1);
    uint8_t B = LOW_NIBBLE(arg1);
    uint8_t C = HIGH_NIBBLE(arg2);
    uint8_t D = LOW_NIBBLE(arg2);

    // Combining nibbles to form the actual position
    uint16_t decoded_pos = (A << 12) | (B << 8) | (C << 4) | D;

    // Handling negative values using two's complement representation
    if (A & 0x8) {
        decoded_pos |= 0xFFFFF000;
    }
    
    return decoded_pos;
}

/* ************************************************************************* */

int16_t AP_VOLZ_Wing::calc_servo_power(int32_t current_pos, uint16_t target_pos)
{
    // calculate the error between the current and target position    
    int error = target_pos - current_pos;
    // Initialize power to zero
    int16_t power = 0;

    // apply saturation filter to clamp the max power.
    if (error > THRESHOLD_POSITION)
    {
        power = -MAX_POWER;
    }
    else if (error < -THRESHOLD_POSITION)
    {
        power = MAX_POWER;
    }
    // Check if the current position is within the tolerance range of the target position
    else if (fabs(error) <= THRESHOLD_POSITION)
    {
        power = 0;
    }

    return power;
}

/* ************************************************************************* */

void AP_VOLZ_Wing::set_servo_command(int16_t power)
{
    // Command Forward when positive power
    if (power >= 0) {
        send_fwd(power);
    }
    // Command Reverse when negative
    else {
        // Remove our negation by taking absolute value.
        send_rev(abs(power));
    }
    return;
}

/* ************************************************************************* */

float AP_VOLZ_Wing::wing_status_degree(int32_t position)
{
    const float MIN_DEGREES = sweep_angle_limit.get_lower_limit();
    const float MAX_DEGREES = sweep_angle_limit.get_upper_limit();
    // Convert position to degrees in respect to our specified range
    float degrees = (position / static_cast<float>(TOTAL_TICKS)) * (MAX_DEGREES - MIN_DEGREES) + MIN_DEGREES;

    return degrees;
}

/* ************************************************************************* */

uint16_t AP_VOLZ_Wing::calc_target_ticks(uint8_t value)
{
    const uint8_t MIN_DEGREES = sweep_angle_limit.get_lower_limit();
    const uint8_t MAX_DEGREES = sweep_angle_limit.get_upper_limit();
    // Convert commanded position to tick counter in respect to our specified range
    uint16_t ticks = static_cast<uint16_t>((static_cast<int>(value) - MIN_DEGREES) * TOTAL_TICKS / static_cast<int>(MAX_DEGREES - MIN_DEGREES));

    return ticks;
}

/* ************************************************************************* */

void AP_VOLZ_Wing::send_idle()
{
    // Idle command message
    CMD cmd {};
    cmd.ID = CMD_ID::MOTOR_POWER_CONTROL;
    cmd.actuator_id = VOLZ_ID;
    cmd.arg1 = PWR_ARG::IDLE;

    // Send out the idle command
    send_command(cmd);
    return;
}

/* ************************************************************************* */

void AP_VOLZ_Wing::send_fwd(uint8_t motor_power)
{
    // Forward command message
    CMD cmd;
    cmd.ID = CMD_ID::MOTOR_POWER_CONTROL;
    cmd.actuator_id = VOLZ_ID;
    cmd.arg1 = PWR_ARG::FORWARD_DIRECTION;
    cmd.arg2 = motor_power;

    // Send out the forward command
    send_command(cmd);
    return;
}


/* ************************************************************************* */

void AP_VOLZ_Wing::send_rev(uint8_t motor_power)
{
    // Reverse command message
    CMD cmd;
    cmd.ID = CMD_ID::MOTOR_POWER_CONTROL;
    cmd.actuator_id = VOLZ_ID;
    cmd.arg1 = PWR_ARG::BACKWARD_DIRECTION;
    cmd.arg2 = motor_power;
    
    // Send out the reverse command
    send_command(cmd);
    return;
}

/* ************************************************************************* */

void AP_VOLZ_Wing::request_current()
{
    // Request Current message
    CMD cmd {};
    cmd.ID = CMD_ID::READ_CURRENT;
    cmd.actuator_id = VOLZ_ID;

    // Send out the Current request
    send_command(cmd);
    return;
}

/* ************************************************************************* */

void AP_VOLZ_Wing::request_position()
{
    // Request position message
    CMD cmd {};
    cmd.ID = CMD_ID::READ_POSITION_RAW;
    cmd.actuator_id = VOLZ_ID;

    // Sneak in a current request every 8th request
    if (msg_counter <= 8) {
        // Send out the position request
        send_command(cmd);
        msg_counter++;
    } else {
        // Send out the current request
        request_current();
        msg_counter = 0;
    }

    return;
}

/* ************************************************************************* */

void AP_VOLZ_Wing::update_position()
{
    sweep_angle_limit.new_value(wing_status_degree(total_position));
    volz_state.set_sweep_angle(sweep_angle_limit.get());

#if HAL_LOGGING_ENABLED
    // Current Timestamps
    uint32_t now = AP_HAL::millis();
    // Log the wing position every 500ms
    float sweep_temp = floorf(volz_state.get_sweep_angle() + 0.5);
    if (now - last_wing_log_ms > 500) {
        // @LoggerMessage: VOLZ
        // @Description: Volz servo data
        // @Field: TimeUS: Time since system startup
        // @Field: DesPos: desired position
        // @Field: Pos: reported position
        // @Field: volt: supply voltage
        // @Field: curr: supply current
        // @Field: temp: temperature
        AP::logger().WriteStreaming("VOLZ",
            "TimeUS,DesPos,Pos,Volt,Curr,Temp",
            "sddvAO",
            "F00000",
            "QBBfff",
            AP_HAL::micros64(),
            volz_state.get_target_command(),
            float_to_uint8(sweep_temp),
            volz_state.get_voltage(),
            volz_state.get_current(),
            volz_state.get_pcb_temp()
            );
        last_wing_log_ms = now;
    }
#endif

    return;
}

/* ************************************************************************* */

// calculate CRC for volz serial protocol and send the data.
void AP_VOLZ_Wing::send_command(CMD &cmd)
{
    const uint16_t crc = calculate_volz_crc(cmd);

    // add CRC result to the message
    cmd.crc1 = HIGHBYTE(crc);
    cmd.crc2 = LOWBYTE(crc);

    tx_write(cmd.data, sizeof(cmd));

    return;
}

/* ************************************************************************* */

// Return the crc for a given command packet
uint16_t AP_VOLZ_Wing::calculate_volz_crc(const CMD &cmd) const
{
    uint16_t crc = 0xFFFF;

    // calculate Volz CRC value according to protocol definition
    for(uint8_t i=0; i<4; i++) {
        // take input data into message that will be transmitted.
        crc = (cmd.data[i] << 8) ^ crc;

        for(uint8_t j=0; j<8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x8005;
            } else {
                crc = crc << 1;
            }
        }
    }

    return crc;
}

/* ************************************************************************* */

bool AP_VOLZ_Wing::tx_write(uint8_t *buff, uint16_t len)
{
    _max14830->set_uart_address(VOLZ_UART_ADDR);
    _max14830->tx_write(buff, len);

    return true;
}

/* ************************************************************************* */