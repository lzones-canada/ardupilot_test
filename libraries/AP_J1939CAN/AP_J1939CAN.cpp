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
 * AP_J1939CAN.cpp
 *
 *      Author: Kyle Fruson
 */

#include "AP_J1939CAN.h"

#if AP_J1939CAN_ENABLED
#include <stdio.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>    // for MIN,MAX

extern const AP_HAL::HAL& hal;

#if HAL_CANMANAGER_ENABLED
#define debug_can(level_debug, fmt, args...) do { AP::can().log_text(level_debug, "J1939CAN", fmt, ##args); } while (0)
#else
#define debug_can(level_debug, fmt, args...)
#endif

#define AP_J1939CAN_DEBUG 1

AP_J1939CAN::AP_J1939CAN()
{
    debug_can(AP_CANManager::LOG_INFO, "J1939CAN: constructed\n\r");
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "J1939CAN: constructed");
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_J1939CAN must be singleton");
    }
#endif
    _singleton = this;
}

void AP_J1939CAN::init()
{
    if (_driver != nullptr) {
        // only allow one instance
        return;
    }

    // Instantiate driver when set (CAN_D1_PROTOCOL)
    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        if (CANSensor::get_driver_type(i) == AP_CAN::Protocol::J1939CAN) {
            _driver = new AP_J1939CAN_Driver();
            return;
        }
    }
}

AP_J1939CAN_Driver::AP_J1939CAN_Driver() : CANSensor("J1939CAN")
{
    register_driver(AP_CAN::Protocol::J1939CAN);

    // start thread for receiving and sending CAN frames. Tests show we use about 640 bytes of stack
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_J1939CAN_Driver::loop, void), "j1939can", 2048, AP_HAL::Scheduler::PRIORITY_CAN, 0);
}


// parse inbound frames
void AP_J1939CAN_Driver::handle_frame(AP_HAL::CANFrame &frame)
{
    // check for extended frame
    if (!frame.isExtended()) {
        return;
    }

    const frame_id_t id { .value = frame.id & AP_HAL::CANFrame::MaskExtID };

#if AP_J1939CAN_DEBUG
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"J1939CAN[%lu]: pdu_fmt:%X, sa:%X, pdu_dest:%X, pri:%X, dlen:%d", id.value, (int)id.pdu_format, (int)id.source_address, (int)id.pdu_dest, (int)id.priority, (int)frame.dlc);
#endif
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"J1939CAN[%lu]: pdu_fmt:%X, sa:%X, pdu_dest:%X, pri:%X, dlen:%d", id.value, (int)id.pdu_format, (int)id.source_address, (int)id.pdu_dest, (int)id.priority, (int)frame.dlc);

    if (id.source_address != SOURCE_ADDRESS) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"J1939CAN[%lu]: pdu_fmt:%X, sa:%X, pdu_dest:%X, pri:%X, dlen:%d", id.value, (int)id.pdu_format, (int)id.source_address, (int)id.pdu_dest, (int)id.priority, (int)frame.dlc);
        // not for us or invalid id (0 and 1 are invalid)
        return;
    }

    // static variables for checksum
    static uint8_t received_CRC;
    static uint8_t calculated_CRC;

    // get the checksum from frame. Last byte is the checksum
    received_CRC = frame.data[frame.dlc-1];

    // calculate the checksum for integrity purposes.
    calculated_CRC = get_CheckSum(frame.data);

    // calculate checksum
    if(received_CRC != calculated_CRC) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"J1939CAN[%lu]: checksum failed, calc:%X vs. rec:%X", id.value, calculated_CRC, received_CRC);
        return;
    }

    //GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"J1939CAN: rx id:%d, src:%d, dest:%d, len:%d", (int)id.pdu_format, (int)id.source_address, (int)id.pdu_dest, (int)frame.dlc);
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"CAN msg from:%lu data: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X", frame.id, frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4], frame.data[5], frame.data[6], frame.data[7]);

    // parse the frame
    frame_data_t data {
        {
            //.angle_1    = static_cast<uint16_t>(((frame.data[0] << 0) | (frame.data[1] << 8)) * 0.1f),
            //.angle_1    = static_cast<uint16_t>(((frame.data[0] << 8) | frame.data[1]) * 0.1f),
            .angle_1    = le16toh_ptr(&frame.data[0]),// * SCALE_FACTOR,
            .angle_2    = le16toh_ptr(&frame.data[2]),// * SCALE_FACTOR,
            .reserved   = le16toh_ptr(&frame.data[4]),
            .error      = frame.data[6],
            .chksum     = frame.data[7],
        }
    };

    //memcopy to a shared working buffer??

    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"DATA_PARSED:  Angle1:%d, Angle2:%d, Reserved:%d, Error:%d, ChkSum:%d", data.angle_1, data.angle_2, data.reserved, data.error, data.chksum);

    return;
}

// uint16_t littleEndianToUint16(const uint8_t* data)
// {
//     return static_cast<uint16_t>(data[0] << 0) | static_cast<uint16_t>(data[1] << 8);
// }

// uint32_t littleEndianToUint32(const uint8_t* data)
// {
//     return static_cast<uint32_t>(data[0] << 0) | static_cast<uint32_t>(data[1] << 8) |
//            static_cast<uint32_t>(data[2] << 16) | static_cast<uint32_t>(data[3] << 24);
// }

void AP_J1939CAN_Driver::loop()
{
    AP_HAL::CANFrame txFrame {};
    AP_HAL::CANFrame rxFrame {};

#if AP_J1939CAN_USE_EVENTS
    _output.thread_ctx = chThdGetSelfX();
#endif

    while (true) {
#if AP_J1939CAN_USE_EVENTS
        // sleep until we get new data, but also wake up at 400Hz to send the old data again
        chEvtWaitAnyTimeout(ALL_EVENTS, chTimeUS2I(2500));
 #else
        hal.scheduler->delay_microseconds(2500); // 400Hz
#endif
    } // while true
}


bool AP_J1939CAN_Driver::send_frame(const uint32_t pgn, const uint8_t dest_address, const uint8_t priority, const uint8_t *data, const uint8_t data_len)
{
    // broadcast telemetry request frame
    const frame_id_t id {
        {
            .source_address =  dest_address,
          //.pdu_dest       =  static_cast<uint8_t>(pgn & 0xFF),
            .pdu_dest       =  static_cast<uint8_t>(pgn | 0xFF),
            .pdu_format     =  static_cast<uint8_t>(pgn >> 8),
            .res_dp         =  0,
            .priority       =  priority,
        }
    };

    AP_HAL::CANFrame frame = AP_HAL::CANFrame((id.value | AP_HAL::CANFrame::FlagEFF), data, data_len, false);

    //const uint64_t timeout_us = uint64_t(timeout_ms) * 1000UL;
    const uint64_t timeout_us = uint64_t(10) * 1000UL;
    return write_frame(frame, timeout_us);
}

// Checksum calculation for Amphenol Wing Sensor.
uint8_t AP_J1939CAN_Driver::get_CheckSum(uint8_t data[])
{
    uint8_t XOR = 0x00;
    uint8_t c;
    for (uint8_t i = 0; i < 8; i++) {
        c = data[i];
        XOR ^= c;
    }
    return XOR;
}


// singleton instance
AP_J1939CAN *AP_J1939CAN::_singleton;

namespace AP {
AP_J1939CAN *j1939can()
{
    return AP_J1939CAN::get_singleton();
}
};

#endif // AP_J1939CAN_ENABLED