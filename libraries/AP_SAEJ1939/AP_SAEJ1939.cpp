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

#include "AP_SAEJ1939.h"

#if AP_SAEJ1939_ENABLED
#include <stdio.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>    // for MIN,MAX

extern const AP_HAL::HAL& hal;
// Define the static member variable
frame_data_t AP_SAEJ1939::working_data_buffer;

#define AP_SAEJ1939_DEBUG 1

// parse inbound frames
void AP_SAEJ1939::handle_frame(AP_HAL::CANFrame &frame)
{
    // check for extended frame
    if (!frame.isExtended()) {
        return;
    }

    const frame_id_t id { .value = frame.id & AP_HAL::CANFrame::MaskExtID };

    if (id.source_address != SOURCE_ADDRESS) {
#if AP_SAEJ1939_DEBUG        
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

    // calculate checksum
    if(received_CRC != calculated_CRC) {
#if AP_SAEJ1939_DEBUG            
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"J1939CAN[0x%lX]: checksum failed, calc:%X vs. rec:%X", id.value, calculated_CRC, received_CRC);
#endif
        return;
    }

#if AP_SAEJ1939_DEBUG
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"CAN[0x%lX] raw msg data: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X", id.value, frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
#endif

    // parse the frame
    frame_data_t data {
        {
            .angle_1    = le16toh_ptr(&frame.data[0]),// * SCALE_FACTOR,
            .angle_2    = le16toh_ptr(&frame.data[2]),// * SCALE_FACTOR,
            .reserved   = le16toh_ptr(&frame.data[4]),
            .error      = frame.data[6],
            .chksum     = frame.data[7],
        }
    };

    //memcopy to a shared working buffer for usage.
    memcpy(&working_data_buffer, &data, sizeof(data));

#if AP_SAEJ1939_DEBUG
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"CAN[0x%lX] data  parsed: Angle1:%d, Angle2:%d, Reserved:0x%02X, Error:0x%02X, ChkSum:0x%02X", id.value, data.angle_1, data.angle_2, data.reserved, data.error, data.chksum);
#endif

    // send the data to the AP_SAEJ1939 object
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

    send_frame(0xFED8, 0x81, 7, (uint8_t*)&data_frame, sizeof(data));

    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"CAN Working data buffer: Angle1:%d, Angle2:%d, Reserved:0x%02X, Error:0x%02X, ChkSum:0x%02X", working_data_buffer.angle_1, working_data_buffer.angle_2, working_data_buffer.reserved, working_data_buffer.error, working_data_buffer.chksum);

    return;
}

// construct and send a frame
bool AP_SAEJ1939::send_frame(const uint32_t pgn, const uint8_t dest_address, const uint8_t priority, const uint8_t *data, const uint8_t data_len)
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
uint8_t AP_SAEJ1939::get_CheckSum(const AP_HAL::CANFrame frame)
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

#endif // AP_SAEJ1939_ENABLED
