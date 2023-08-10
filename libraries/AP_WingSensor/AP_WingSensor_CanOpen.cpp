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
 * AP_WingSensor_CanOpen.cpp
 *
 *      Author: Kyle Fruson
 */

#include "AP_WingSensor_config.h"

#if AP_WINGSENSOR_CANOPEN_ENABLED

#include <AP_HAL/AP_HAL.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_WingSensor_CanOpen.h"

// static variables
static const double WING_RESOLUTION = 0.1;

extern const AP_HAL::HAL& hal;


#if HAL_CANMANAGER_ENABLED
#define debug_can(level_debug, fmt, args...) do { AP::can().log_text(level_debug, "CAN_OPEN", fmt, ##args); } while (0)
#else
#define debug_can(level_debug, fmt, args...)
#endif


// constructor
AP_WingSensor_CanOpen::AP_WingSensor_CanOpen(AP_WingSensor &_frontend) :
    CANSensor("CAN_OPEN"),
    AP_WingSensor_Backend(_frontend)
{
    register_driver(AP_CAN::Protocol::CAN_OPEN);
}

// parse inbound frames
void AP_WingSensor_CanOpen::handle_frame(AP_HAL::CANFrame &frame)
{
    // check for extended frame
    if (!frame.isExtended()) {
        return;
    }

    // parse the frame id
    const frame_id_t id { .value = frame.id & AP_HAL::CANFrame::MaskExtID };

    // confirm message is for us
    if (id.source_address != SOURCE_ADDRESS) {
#if AP_CANOPEN_DEBUG        
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"J1939CAN[0x%lX]: pdu_fmt:%02X, sa:%02X, pdu_dest:%02X, pri:%d, dlen:%d", id.value, (int)id.pdu_format, (int)id.source_address, (int)id.pdu_dest, (int)id.priority, (int)frame.dlc);
#endif        
        // not for us or invalid id (0 and 1 are invalid)
        return;
    }

    // static variables for checksum
    static uint8_t received_CRC;
    static uint8_t calculated_CRC;

    // get the checksum from frame. Last byte is the checksum
    received_CRC = frame.data[frame.dlc-1];

    // calculate the checksum for integrity purposes.
    calculated_CRC = get_CheckSum(frame);

    // check for good checksum
    if(received_CRC == calculated_CRC) {
    // parse raw frame data
#if AP_CANOPEN_DEBUG
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"CAN[0x%lX] raw msg data: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X", id.value, frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
#endif

        // parse out and save frame data to internal structure.
        frame_data_t data {
            {
                .angle_1    = le16toh_ptr(&frame.data[0]),
                .angle_2    = le16toh_ptr(&frame.data[2]),
                .reserved   = le16toh_ptr(&frame.data[4]),
                .error      = frame.data[6],
                .chksum     = frame.data[7],
            }
        };

        // save data to frontend on success
        if(!error_handling(data.error)) {
            // Apply resolution.
            internal_state.angle_1_deg = data.angle_1 * WING_RESOLUTION;
            internal_state.angle_2_deg = data.angle_2 * WING_RESOLUTION;
        }
    }
    // checksum failed
    else {
        debug_can(AP_CANManager::LOG_ERROR, "Checksum error");
#if AP_CANOPEN_DEBUG
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"J1939CAN[0x%lX]: checksum failed, calc:%X vs. rec:%X", id.value, calculated_CRC, received_CRC);
#endif
    }

    // send data to the AP_WingSensor.
    send_frame_data_t data_frame {
        {
            .data0 = 0x00,
            .data1 = 0xFE,
            .data2 = 0xD8,
            .data3 = 0xFF,
            .data4 = 0x02,
            .data5 = 0x00,
            .data6 = 0x09,
            .data7 = 0x20,
        }
    };

    send_frame(0xFED8, 0x81, 7, (uint8_t*)&data_frame, sizeof(data_frame));

    return;
}

// construct and send a frame
bool AP_WingSensor_CanOpen::send_frame(const uint32_t pgn, const uint8_t dest_address, const uint8_t priority, const uint8_t *data, const uint8_t data_len)
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
uint8_t AP_WingSensor_CanOpen::get_CheckSum(const AP_HAL::CANFrame frame)
{
    uint8_t XOR = 0x00;
    uint8_t c;
    uint8_t size = frame.dlc-1;
    for (uint8_t i = 0; i < size; i++) {
        c = frame.data[i];
        XOR ^= c;
    }
    return XOR;
}

// error handling for Amphenol Wing Sensor.
bool AP_WingSensor_CanOpen::error_handling(const uint8_t error_code)
{
    // error handling from sensor
    switch(error_code) {
        case 0x00:
            // no error
            break;
        case 0x01:
            // hall sensor comms error
            debug_can(AP_CANManager::LOG_ERROR, "Error Hall sensor communication");
            return true;
        case 0x02:
            // hall sensor in error status
            debug_can(AP_CANManager::LOG_ERROR, "Hall sensor in error status");
            return true;
        case 0x03:
            // initialize error
            debug_can(AP_CANManager::LOG_ERROR, "Initialization error");
            return true;
        case 0x04:
            // angle out of range
            debug_can(AP_CANManager::LOG_ERROR, "Angle out of range");
            return true;
        case 0x05:
            // checksum error
            debug_can(AP_CANManager::LOG_ERROR, "Checksum error");
            return true;
        default:
            // unknown error
            debug_can(AP_CANManager::LOG_ERROR, "Unknown error");
            break;
    }
    return false;
}


void AP_WingSensor_CanOpen::update()
{
    // copy the data to the front end
    copy_to_frontend();
}

#endif // AP_WINGSENSOR_CANOPEN_ENABLED
