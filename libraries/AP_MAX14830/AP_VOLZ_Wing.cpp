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
#include <cstdio> // Add this line to include the required header


//------------------------------------------------------------------------------
// Static FIFO buffer to retrieve rx message from Volz Wing UART1 FIFO
//------------------------------------------------------------------------------

static const uint8_t MESSAGE_BUFFER_LENGTH = 128;
static uint8_t rx_fifo_buffer[MESSAGE_BUFFER_LENGTH];

//------------------------------------------------------------------------------
// Static buffer to store the message to be converted in.
//------------------------------------------------------------------------------

static uint8_t rx_buffer[MESSAGE_BUFFER_LENGTH];


//------------------------------------------------------------------------------
// Static pointer to control access to the contents of message_buffer.
//------------------------------------------------------------------------------

static uint8_t *rx_buffer_ptr = rx_buffer;



extern const AP_HAL::HAL& hal;
// Reference to the global instance of Volz_State
Volz_State volz_state;

#define SCALING_FACTOR    (360.0 / 4096.0)
#define TOTAL_REVS        (9)
#define TICKS_PER_REV     (4096)
#define TOTAL_TICKS       (TICKS_PER_REV * TOTAL_REVS)
#define WING_MIN_DEGREES  (8.0)
#define WING_MAX_DEGREES  (88.0)

#define MAX_POWER         (0xFF)
#define KP                (0.1)


//------------------------------------------------------------------------------
// Constructor
//------------------------------------------------------------------------------
AP_VOLZ_Wing::AP_VOLZ_Wing(AP_MAX14830* max14830) : 
    _max14830(max14830),
    sweep_angle_limit(WING_MAX_DEGREES, WING_MIN_DEGREES)
{
}

// Function to convert enum to string
const char* AP_VOLZ_Wing::stateToString(State state) {
    switch (state) {
        case CALIBRATE: return "CALIBRATE";
        case REQUEST_POSITION: return "REQUEST_POSITION";
        case WING_LIMIT_HIT: return "WING_LIMIT_HIT";
        case CALIBRATE_COMPLETE: return "CALIBRATE_COMPLETE";
        case ACTIVE: return "ACTIVE";
        case ACTIVE_REQUEST: return "ACTIVE_REQUEST";
        case INIT_REQUEST: return "INIT_REQUEST";
        case DEADBAND: return "DEADBAND";
        default: return "UNKNOWN_STATE";
    }
}

void AP_VOLZ_Wing::init(void)
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
    offset = 0;
    
    curr_percent = 0.0;
    servo_cmd = 0;

    // target percent to achieve from GCS
    volz_state.set_target_command(WING_MAX_DEGREES);
    prev_target_command = WING_MAX_DEGREES;
    target_percent = 0.0;
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
    if (!initialised) {
        initialised = true;
        init();
    }

    // Update the sweep wing angle every iteration.
    update_position();

    // ---------------------------------------
    // read any available data on serial port
    // ---------------------------------------
    // Done through polling interrupt now!


    // ---------------------------------------
    // Wing limit switch logic
    // ---------------------------------------
    // Read the limit switch state
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

    // Retrieve the target percentage from the ground station
    target_command = volz_state.get_target_command();
    // Check if the target deviates more than 1% from our previous target 
    if (prev_target_command != target_command) {
        printf("target_command: %d\n", target_command);
        // Calculate the target position in ticks
        target_position = calc_target_ticks(target_command);
        // Scale the target with threshold which increases our accuracy
        if(target_position - total_position >= 460 && target_command < WING_MAX_DEGREES) {
            target_position += 150;
        }
        else if(target_position - total_position < -460 && target_command > WING_MIN_DEGREES) {
            target_position -= 150;
        }
        // Transition to the ACTIVE_REQUEST state
        machine_state = ACTIVE_REQUEST;

        // // Fully open commanded, lets go slightly past to ensure wing limit contact.
        // if(target_command == static_cast<uint8_t>(WING_MAX_DEGREES)){
        //     machine_state = CALIBRATE;
        // }
        // else {
        //     // Transition to the ACTIVE_REQUEST state
        //     machine_state = ACTIVE_REQUEST;
        // }
    }
    // Update the previous commanded target
    prev_target_command = target_command;

    // static uint8_t counter = 0;
    // static State prev_state = machine_state;
    // counter++;
    // if (counter >= 25 || prev_state != machine_state) {
    //     counter = 0;
    //     DEV_PRINTF("%s\n", stateToString(machine_state));
    // }
    // prev_state = machine_state;

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
            servo_cmd = calc_servo_command(total_position, target_position);
            // Send out the servo command
            set_servo_command(servo_cmd);
            break;

        case ACTIVE_REQUEST:
            // Request Position from the servo to update the current position
            request_position();
            break;

        case INIT_REQUEST:
            // Request our current position while we are doing nothing to init an offset..
            request_position();
            break;

        case DEADBAND:
            // Do nothing..
            break;

        default:
            break;
    }
    
    return;
}

/* ************************************************************************* */

void AP_VOLZ_Wing::handle_volz_uart1_interrupt()
{
    //uint8_t rx_data[VOLZ_DATA_FRAME_SIZE] = {0}; 

    // Read Data out of FIFO when interrupt triggered, store current length of FIFO.
    _max14830->set_uart_address(UART::ADDR_1);
    rxbuf_fifo_len = _max14830->rx_read(rx_fifo_buffer, MESSAGE_BUFFER_LENGTH);
    // Clear the interrupt on this UART Address.
    _max14830->clear_interrupts();

    // Pointer to the start of FIFO buffer.
    const uint8_t *byte_ptr = &rx_fifo_buffer[0];

    // Check if we have enough data to process.
    if(rxbuf_fifo_len < VOLZ_DATA_FRAME_SIZE) {
        // Not enough data to process. Reset our buffer pointer and return.
        rx_buffer_ptr = rx_buffer;
        return;
    }

    // Copy over the data from the FIFO buffer to our rx buffer.
    while (rxbuf_fifo_len--) {
        // Copy data over to rx buffer.
        *rx_buffer_ptr = *byte_ptr;
        // Increment our buffer indices.
        byte_ptr++;
        rx_buffer_ptr++;
    }

    // Copy over into a Working Buffer for further processing.
    uint8_t *rx_work_buffer = rx_buffer;

    // Reset to the start of the message buffer and start looking for a new message.
    rx_buffer_ptr = rx_buffer;

    handle_volz_message(rx_work_buffer);

    return;
}

/* ************************************************************************* */

void AP_VOLZ_Wing::handle_volz_message(uint8_t* rx_work_buffer)
{
    // Confirm our received address is for the Volz Wing
    if (rx_work_buffer[1] == VOLZ_ADDR) 
    {
        /// Extract msgid
        uint8_t msgid = rx_work_buffer[0];
        // Calc CRC and compare with received CRC
        uint16_t received_crc = UINT16_VALUE(rx_work_buffer[4], rx_work_buffer[5]);
        uint16_t calculated_crc = calc_volz_crc(rx_work_buffer);
        // Process message only on CRC match
         if(received_crc == calculated_crc) {
            // Handle message
            switch (msgid) 
            {
                case VOLZ_POS_RAW_STAT:
                    // Handle position message
                    handle_pos_msg(rx_work_buffer);
                    // Flip flop between the command and request state.
                    if(machine_state == ACTIVE_REQUEST) {
                        // Transition to the next state after the wing limit state transitions back to false
                        machine_state = ACTIVE;
                    }
                    if(machine_state == CALIBRATE_COMPLETE) {
                        // Finished Calibrating and acknowledge, set our position and transition state.
                        // Set our position to max value as we are at the wing limit
                        total_position = TOTAL_TICKS;
                        // Set prev to current.
                        prev_raw_position = raw_position;
                        // Transition to the next state after the wing limit state transitions back to false
                        machine_state = DEADBAND;
                    }
                    if(machine_state == INIT_REQUEST) {
                        // capture our offset
                        offset = total_position;
                        // Transition out
                        machine_state = DEADBAND;
                    }
                    break;
                
                case VOLZ_PWR_CTRL_STAT:
                    // FD Acknowledgement
                    if(rx_work_buffer[2] == VOLZ_PWR_FD_CMD) {
                        if(machine_state == CALIBRATE) {
                            // Acknowledgement for power control command, move ahead state machine
                            machine_state = REQUEST_POSITION;
                        }
                        // Power control with valid motor power
                        if(machine_state == ACTIVE && rx_work_buffer[3] != 0) {
                            // Acknowledgement for power control command, move ahead state machine
                            machine_state = ACTIVE_REQUEST;
                        }
                        // Power control with no motor power being sent
                        if(machine_state == ACTIVE && rx_work_buffer[3] == 0) {
                            // Transition into deadband state with no motor power being sent
                            machine_state = DEADBAND;
                        }
                    }
                    // BD Acknowledgement
                    if(rx_work_buffer[2] == VOLZ_PWR_BD_CMD) {
                        // Power control with valid motor power
                        if(machine_state == ACTIVE && rx_work_buffer[3] != 0) {
                            // Acknowledgement for power control command, move ahead state machine
                            machine_state = ACTIVE_REQUEST;
                        }
                        // Power control with no motor power being sent
                        if(machine_state == ACTIVE && rx_work_buffer[3] == 0) {
                            // Transition into deadband state with no motor power being sent
                            machine_state = DEADBAND;
                        }
                    }
                    // Idle Acknowledgement
                    if(rx_work_buffer[2] == VOLZ_PWR_ID_CMD) {
                        if(machine_state == WING_LIMIT_HIT) {
                            // Acknowledgement for power control command, move ahead state machine
                            machine_state = CALIBRATE_COMPLETE;
                        }
                    }

                default:
                    break;
            }
        }
    }

    return;
}

/* ************************************************************************* */

void AP_VOLZ_Wing::handle_pos_msg(uint8_t data[VOLZ_DATA_FRAME_SIZE])
{
    // Parse the data & extract/decode position data
    raw_position = decode_position(data[2], data[3]);

    // Calculate the angle difference between current and previous positions
    int16_t angle_diff = raw_position - prev_raw_position;

    // Account for full rotations
    if (angle_diff < -TICKS_PER_REV / 2) {
        // Increment spin count for full rotations in the forward direction
        total_position += TICKS_PER_REV;
    } else if (angle_diff > TICKS_PER_REV / 2) {
        // Decrement spin count for full rotations in the backward direction
        total_position -= TICKS_PER_REV;
    }
    // Increment spin count for forward rotation or decrement for backward rotation
    total_position += angle_diff;

    // Update previous position for the next iteration
    prev_raw_position = raw_position;

    return;
}

/* ************************************************************************* */

uint16_t AP_VOLZ_Wing::decode_position(uint8_t arg1, uint8_t arg2)
{
    // Extracting individual nibbles from the arguments
    uint8_t A = (arg1 >> 4) & 0xF;
    uint8_t B = (arg1 & 0xF);
    uint8_t C = (arg2 >> 4) & 0xF;
    uint8_t D = (arg2 & 0xF);

    // Combining nibbles to form the actual position
    uint16_t decoded_pos = (A << 12) | (B << 8) | (C << 4) | D;

    // Handling negative values using two's complement representation
    if (A & 0x8) {
        decoded_pos |= 0xFFFFF000;
    }
    
    return decoded_pos;
}

/* ************************************************************************* */

int16_t AP_VOLZ_Wing::calc_servo_command(int32_t current_pos, uint16_t target_pos)
{
    // calculate the error between the current and target position    
    int error = target_pos - current_pos;
    float command = KP * error;

    // Current Position in a Percentage
    curr_percent = wing_status_percent(current_pos);
    // Target Position in a Percentage
    target_percent = wing_status_percent(target_pos);

    DEV_PRINTF("curr_percent: %f\n", curr_percent);
    DEV_PRINTF("target_percent: %f\n", target_percent);

    // 0.5 Degree Threshold (460 Ticks / Degree)
    //  460 / 2 = 230 Ticks * 0.1 = 23 Ticks
    const float THRESHOLD_POSITION = 230.0 * KP;

    // apply saturation filter to clamp the max command speed
    if (command > MAX_POWER)
    {
        command = MAX_POWER;
    }
    else if (command < -MAX_POWER)
    {
        command = -MAX_POWER;
    }

     // Check if the current position is within the tolerance range of the target position
    // if (fabs(current_pos - target_pos) <= THRESHOLD_POSITION)
    // {
    //     command = 0;
    // }

    // // Max power in reverse direction until we achieve the target position.
    // command = (curr_percent > (target_percent + 3)) ? -MAX_POWER : 0;

    // Edge case - fully closed
    if (target_command == static_cast<uint8_t>(WING_MIN_DEGREES))
    {
        // Move at full power in reverse direction until we are still 5% out.
        if(curr_percent > (target_percent + 5)) {
            command = -MAX_POWER;
        }
        // Slow down when within 5% to 1.5% of target.
        else if(curr_percent <= (target_percent + 5) && curr_percent > 1.5) {
            command = (-MAX_POWER / 2);
        }
        // Slow down further within 1.5% to 0.5% of target.
        else if(curr_percent <= (target_percent + 1.5) && curr_percent >= 0.5) {
            command = (-MAX_POWER / 4);
        }
        // Stop when within 0.5% to 0% of the target.
        else {
            command = 0;
        }
    }
    // Edge case - fully open
    else if (target_command == static_cast<uint8_t>(WING_MAX_DEGREES))
    {
        // Move at full power in reverse direction until we are still 5% out.
        if(curr_percent < (target_percent - 5)) {
            command = MAX_POWER;
        }
        // Slow down when within 95% to 98.5% of target.
        else if(curr_percent >= (target_percent - 5) && curr_percent < 98.5) {
            command = (MAX_POWER / 2);
        }
        // Slow down further within 98.5% to 99.5% of target.
        else if(curr_percent >= (target_percent - 1.5) && curr_percent <= 99.5) {
            command = (MAX_POWER / 4);
        }
        // Stop when within 99.5% to 100% of the target.
        else {
            command = 0;
        }
    }
    // Check if the current position is within the tolerance range of the target position
    else if (fabs(command) <= THRESHOLD_POSITION)
    {
        command = 0;
    }

    // Handle abnormality Edge Case of minor movement where the servo appears to move but not register position..
    // Counter and threshold for repeated low positive commands
    static int repeated_low_positive_counter = 0;
    const int REPEAT_THRESHOLD = 25;
    const float REPEAT_COMMAND_THRESHOLD = 35.0;

    // Check if the command is repeatedly low and positive
    if (fabs(command) > 0 && fabs(command) <= REPEAT_COMMAND_THRESHOLD)
    {
        repeated_low_positive_counter++;
        // Check if we have reached our threshold and force stopping of commands.
        if (repeated_low_positive_counter > REPEAT_THRESHOLD)
        {
            command = 0;
            repeated_low_positive_counter = 0;  // Reset the counter
        }
    }
    else
    {
        repeated_low_positive_counter = 0;  // Reset the counter
    }

    return command;
}

/* ************************************************************************* */

void AP_VOLZ_Wing::set_servo_command(int16_t command)
{

    DEV_PRINTF("command: %d\n", command);
    DEV_PRINTF("\n");

    // Command Forward when positive
    if (command >= 0) {
        send_fwd(command);
    }
    // Command Reverse when negative
    else {
        // Remove our negation by taking absolute value.
        send_rev(abs(command));
    }
    return;
}

/* ************************************************************************* */

float AP_VOLZ_Wing::wing_status_percent(int32_t position)
{
    float percent = (position / static_cast<float>(TOTAL_TICKS)) * 100.0f;
    return percent;
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
    uint8_t idle[VOLZ_DATA_FRAME_SIZE] = { VOLZ_PWR_CTRL_CMD, VOLZ_ADDR, 0x00, 0x00, 0x00, 0x00};
    // Send out the idle command
    send_command(idle);
    return;
}

/* ************************************************************************* */

void AP_VOLZ_Wing::send_fwd(uint8_t command)
{
    // Forward command message
    uint8_t fwd[VOLZ_DATA_FRAME_SIZE]  = { VOLZ_PWR_CTRL_CMD, VOLZ_ADDR, VOLZ_PWR_FD_CMD, 0x00, 0x00, 0x00};
    // Set the command value
    fwd[3] = command;
    // Send out the forward command
    send_command(fwd);
    return;
}


/* ************************************************************************* */

void AP_VOLZ_Wing::send_rev(uint8_t command)
{
    // Reverse command message
    uint8_t rev[VOLZ_DATA_FRAME_SIZE]  = { VOLZ_PWR_CTRL_CMD, VOLZ_ADDR, VOLZ_PWR_BD_CMD, 0x00, 0x00, 0x00};
    // Set the command value
    rev[3] = command;
    // Send out the reverse command
    send_command(rev);
    return;
}

/* ************************************************************************* */

void AP_VOLZ_Wing::request_position()
{
    // Request position message
    uint8_t pos_req[VOLZ_DATA_FRAME_SIZE]  = { VOLZ_POS_RAW_CMD, VOLZ_ADDR, 0x00, 0x00, 0x00, 0x00};
    // Send out the position request
    send_command(pos_req);
    return;
}

/* ************************************************************************* */

void AP_VOLZ_Wing::update_position()
{
    sweep_angle_limit.new_value(wing_status_degree(total_position));
    volz_state.set_sweep_angle(sweep_angle_limit.get());

    return;
}

/* ************************************************************************* */

// calculate CRC for volz serial protocol and send the data.
void AP_VOLZ_Wing::send_command(uint8_t tx_data[VOLZ_DATA_FRAME_SIZE])
{
    // calculate CRC
    uint16_t crc = calc_volz_crc(tx_data);

    // add CRC result to the message
    tx_data[4] = HIGHBYTE(crc);
    tx_data[5] = LOWBYTE(crc);

    tx_write(tx_data, VOLZ_DATA_FRAME_SIZE);

    return;
}

/* ************************************************************************* */

uint16_t AP_VOLZ_Wing::calc_volz_crc(uint8_t data[VOLZ_DATA_FRAME_SIZE])
{
    uint8_t i,j;
    uint16_t crc = 0xFFFF;

    // calculate Volz CRC value according to protocol definition
    for(i=0; i<4; i++) {
        // take input data into message that will be transmitted.
        crc = ((data[i] << 8) ^ crc);

        for(j=0; j<8; j++) {
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

bool AP_VOLZ_Wing::tx_write(uint8_t *buffer, uint16_t length)
{
    _max14830->set_uart_address(UART::ADDR_1);
    _max14830->tx_write(buffer, length);

    return true;
}

/* ************************************************************************* */