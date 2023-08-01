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
 * AP_J1939CAN.h
 *
 *      Author: Kyle Fruson
 */
 
#pragma once

#include <AP_J1939CAN/AP_J1939CAN_config.h>

#if AP_J1939CAN_ENABLED
#include <AP_HAL/AP_HAL.h>

#include <AP_CANManager/AP_CANSensor.h>
#include <AP_Param/AP_Param.h>

#define AP_J1939CAN_USE_EVENTS (defined(CH_CFG_USE_EVENTS) && CH_CFG_USE_EVENTS == TRUE)

#if AP_J1939CAN_USE_EVENTS
#include <ch.h>
#endif

class AP_J1939CAN_Driver : public CANSensor
{
public:
    AP_J1939CAN_Driver();

private:

    // handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;
    
    // handler for sending frames
    bool send_frame(const uint32_t pgn, const uint8_t dest_address, const uint8_t priority, const uint8_t *data, const uint8_t data_len);
    
    // handler for Amphenol checksum calculations
    uint8_t get_CheckSum(uint8_t data[]);

    void loop();

    struct {
        uint32_t detected_bitmask;
        uint32_t detected_bitmask_ms;
    } _init;

    struct {
        HAL_Semaphore sem;
        bool is_new;
        uint32_t last_new_ms;
#if AP_J1939CAN_USE_EVENTS
        thread_t *thread_ctx;
#endif
    } _output;

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

    static const uint8_t SOURCE_ADDRESS = 0x80;
    static const uint16_t J1939_SERVO_PGN = 0xFF0B;
    static const uint8_t BROADCAST_ADDRESS = 0xFF;
};

class AP_J1939CAN {
public:
    AP_J1939CAN();

    void init();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_J1939CAN);

    static const struct AP_Param::GroupInfo var_info[];
    static AP_J1939CAN *get_singleton() { return _singleton; }

private:
    static AP_J1939CAN *_singleton;

    AP_J1939CAN_Driver *_driver;
};
namespace AP {
    AP_J1939CAN *j1939can();
};

#endif // AP_J1939CAN_ENABLED
