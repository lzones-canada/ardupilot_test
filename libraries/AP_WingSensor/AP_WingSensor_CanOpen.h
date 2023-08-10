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
 * AP_WingSensor_CanOpen.h
 *
 *      Author: Kyle Fruson
 */
 
#pragma once

#include "AP_WingSensor_config.h"

#if AP_WINGSENSOR_CANOPEN_ENABLED

#include "AP_WingSensor.h"
#include "AP_WingSensor_Backend.h"

class AP_WingSensor_CanOpen : public CANSensor, public AP_WingSensor_Backend {
public:
    AP_WingSensor_CanOpen(AP_WingSensor &_frontend);
    
    void update() override;

private:

    // handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;

    // handler for sending frames
    bool send_frame(const uint32_t pgn, const uint8_t dest_address, const uint8_t priority, const uint8_t *data, const uint8_t data_len);
    
    // handler for Amphenol checksum calculations
    uint8_t get_CheckSum(const AP_HAL::CANFrame frame);

    // error handler
    bool error_handling(const uint8_t error_code);

    // amphenol servo constants
    static const uint8_t SOURCE_ADDRESS = 0x80;
    static const uint16_t J1939_SERVO_PGN = 0xFF0B;
    static const uint8_t BROADCAST_ADDRESS = 0xFF;

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

#endif // AP_WINGSENSOR_CANOPEN_ENABLED

