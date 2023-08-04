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
 * AP_SAEJ1939.h
 *
 *      Author: Kyle Fruson
 */
 
#pragma once

#include <AP_SAEJ1939/AP_SAEJ1939_config.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANSensor.h>

#if AP_SAEJ1939_ENABLED

// structure for data field frames 
struct frame_data_angles_t {
    uint16_t angle_1;
    uint16_t angle_2;
};

class AP_SAEJ1939 : public CANSensor 
{
public:
    AP_SAEJ1939() : CANSensor("SAE_J1939") {}

    // handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;
    
    // handler for sending frames
    bool send_frame(const uint32_t pgn, const uint8_t dest_address, const uint8_t priority, const uint8_t *data, const uint8_t data_len);
    
    // handler for Amphenol checksum calculations
    uint8_t get_CheckSum(const AP_HAL::CANFrame frame);

    // Getter function to access the working_data_buffer
     const frame_data_angles_t* get_working_data_buffer() const { return &working_data_buffer; }

private:

    // amphenol servo constants
    static const uint8_t SOURCE_ADDRESS = 0x80;
    static const uint16_t J1939_SERVO_PGN = 0xFF0B;
    static const uint8_t BROADCAST_ADDRESS = 0xFF;

    // Declare working_data_buffer as a static member variable
    static frame_data_angles_t working_data_buffer;

    // structure for CAN ID frames
    union frame_id_t {
        struct PACKED {
            uint8_t source_address:8;
            uint8_t pdu_dest:8;
            uint8_t pdu_format:8;
            uint8_t res_dp:2;
            uint8_t priority:3;
        };
        uint32_t value;
    };

    // structure for sending data frames 
    union send_frame_data_t {
        struct PACKED {
            uint8_t data0;
            uint8_t data1;
            uint8_t data2;
            uint8_t data3;
            uint8_t data4;
            uint8_t data5;
            uint8_t data6;
            uint8_t data7;
        };
    };

    // structure for data field frames 
    union frame_data_t {
        struct PACKED {
            uint16_t angle_1;
            uint16_t angle_2;
            uint16_t reserved;
            uint8_t  error;
            uint8_t  chksum;
        };
    };

};

#endif  // AP_SAEJ1939_ENABLED