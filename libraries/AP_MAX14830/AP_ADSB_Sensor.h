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
 * AP_ADSB_Sensor.h
 *
 *      Author: Kyle Fruson
 */

#pragma once

#ifndef AP_ADSB_SENSOR_H
#define AP_ADSB_SENSOR_H

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_ADSB/GDL90_protocol/GDL90_Message_Structs.h>
#include <AP_ADSB/GDL90_protocol/hostGDL90Support.h>

// Time in milliseconds before we declare the Wing Sensor to be "unhealthy"
#define HEALTHY_LAST_RECEIVED_MS    3000
#define ADSB_SQUAWK_OCTAL_DEFAULT   1200


/*=========================================================================*/
// Mode Message Enums
/*=========================================================================*/

struct MODE
{
    enum
    {
        OFF  = 0x4F,    // 'O'
        STBY = 0x41,    // 'A'
        ON   = 0x43,    // 'C'
        ALT  = 0x53     // 'S'
    };
};

struct IDENT
{
    enum
    {
        ACTVE   = 0x49,  // 'I'
        INACTVE = 0x2D   // '-'
    };
};

struct EMERG
{
    enum
    {
        NONE = 0x30,    // hex ASCII '0'
        GEN  = 0x31,    // hex ASCII '1'
        MED  = 0x32,    // hex ASCII '2'
        FUEL = 0x33,    // hex ASCII '3'
        COM  = 0x34,    // hex ASCII '4'
        HJCK = 0x35,    // hex ASCII '5'
        DWN  = 0x36,    // hex ASCII '6'
        LOL  = 0x37,    // hex ASCII '7'
    };
};

/*=========================================================================*/

//------------------------------------------------------------------------------
// Struct Pair for Hex to ASCII Chksum conversion.
//------------------------------------------------------------------------------
struct CharPair {
    char highChar;
    char lowChar;
};

// Forward declaration
class AP_MAX14830;

//------------------------------------------------------------------------------
// ADSB Class - UART2
//------------------------------------------------------------------------------

class AP_ADSB_Sensor
{
public:
    // Constructor
    AP_ADSB_Sensor(AP_MAX14830* max14830);

    // Update function for ADSB out.
    void update(void);

    // Call Sign Message - 1 min interval or on change
    void send_cs_msg(void);

    // Operating Mode Message - 1 Second interval (nominal)
    void send_op_mode_msg(void);

    // VFR Code Message - 1 min interval or on change
    void send_vfr_msg(void);

    // Handle ADSB-UART2 Interrupt
    void handle_adsb_uart2_interrupt(void);

    // Handle complete ABSB message.
    void handle_complete_adsb_msg(const GDL90_RX_MESSAGE &msg);

    // ADSB Sensor Hardware name from ID
    const char* get_hardware_name(const uint8_t hwId);

    // Expose adbs out function for gcs usage.
    void send_adsb_out_status(const mavlink_channel_t chan);

    // CRC Function for ADSB - ASCII Hex Representation.
    CharPair calc_hex_to_ascii_crc(uint8_t *buf, uint8_t len);

    // Mavlink ADSB out status Packet
    mavlink_uavionix_adsb_out_status_t tx_status;

    // ADSB Squak Code
    AP_Int16   *squawk_octal_param;
    uint16_t    squawk_octal;
    enum ap_var_type ptype;

    // Track last timing of cs msg.
    uint32_t last_cs_msg;

    // Track last timing of op mode msg.
    uint32_t last_mode_msg;

    // Length of bytes to read - returned from Max14830 FIFO.
    uint8_t rxbuf_fifo_len;

    // Flag representing change in callsign. for Message.
    bool cs_flag_change;

    // Custom init flag for message request on startup.
    bool init_flag;

    // ADSB Data
    struct 
    {
        uint32_t last_msg_ms;
        GDL90_RX_MESSAGE msg;
        GDL90_RX_STATUS status;

        // cache local copies so we always have the latest info of everything.
        struct 
        {
            GDL90_IDENTIFICATION_V3 identification;
            GDL90_TRANSPONDER_CONFIG_MSG_V4_V5 transponder_config;
            GDL90_HEARTBEAT heartbeat;
            GDL90_TRANSPONDER_STATUS_MSG transponder_status;
            GDL90_OWNSHIP_REPORT ownship_report;
            GDL90_OWNSHIP_GEO_ALTITUDE ownship_geometric_altitude;
            GDL90_SENSOR_BARO_MESSAGE sensor_message;
        } decoded;

    } rx;

    struct 
    {
        uint32_t last_packet_GPS_ms;
        uint32_t last_packet_Transponder_Control_ms;
        uint32_t last_packet_Transponder_Status_ms;
        uint32_t last_packet_Transponder_Heartbeat_ms;
        uint32_t last_packet_Transponder_Ownship_ms;
    } run_state;

    struct 
        {
            bool                          baroCrossChecked;
            uint8_t                       airGroundState;
            bool                          identActive;
            bool                          modeAEnabled;
            bool                          modeCEnabled;
            bool                          modeSEnabled;
            bool                          es1090TxEnabled;
            int32_t                       externalBaroAltitude_mm;
            uint16_t                      squawkCode;
            uint8_t                       emergencyState;
            uint8_t                       callsign[8];
            bool                          x_bit;
        } ctrl;

private:
    // Pointer to MAX14830 object
    AP_MAX14830* _max14830;
    HAL_Semaphore _sem; // Protect _max14830 from concurrent access

    // structure for sending Op Mode Message
    // union send_mode_data_t {
    //     struct PACKED {
    //         uint8_t  data0;
    //         uint8_t  data1;
    //         uint8_t  data2;
    //         uint8_t  data3;
    //         uint8_t  mode;
    //         uint8_t  data5;
    //         uint8_t  ident;
    //         uint8_t  data7;
    //         uint16_t squawk;
    //         uint8_t  emerg;
    //         uint8_t  health;
    //         uint16_t chksum;
    //         uint8_t  data16;
    //         uint8_t  data17;
    //     };
    // };

    // // Calculate the individual bytes for .squawk
    // uint8_t squawk_byte1 = squawk_octal / 1000;         // Thousands digit
    // uint8_t squawk_byte2 = (squawk_octal / 100) % 10;   // Hundreds digit
    // uint8_t squawk_byte3 = (squawk_octal / 10) % 10;    // Tens digit
    // uint8_t squawk_byte4 = squawk_octal % 10;          // Ones digit

    // // send data to the AP_WingSensor.
    // send_mode_data_t tx_frame {
    //     {
    //         .data0  = 0x5E,          // '^'
    //         .data1  = 0x4D,          // 'M'
    //         .data2  = 0x44,          // 'D'
    //         .data3  = 0x20,          // ' '
    //         .mode   = MODE::STBY,    // 'A'
    //         .data5  = 0x2C,          // ','
    //         .ident   = IDENT::ACTVE,  // 'I'
    //         .data7  = 0x2C,          // ','
    //         .squawk = static_cast<uint16_t>((squawk_byte1 << 12) |
    //                             (squawk_byte2 << 8)  |
    //                             (squawk_byte3 << 4)  |
    //                             squawk_byte4),
    //         .emerg  = EMERG::NONE,
    //         .health = 0x01,          // '1' - Healthy
    //         .chksum = 0xFA,          // '00' - Chcksum
    //         .data16 = 0x0D,          // '\r'
    //         .data17 = 0x0A,          // '\n'
    //     }
    // };

    bool request_msg(const GDL90_MESSAGE_ID msg_id);
    uint16_t gdl90Transmit(GDL90_TX_MESSAGE &message, const uint16_t length);
    bool hostTransmit(uint8_t *buffer, uint16_t length);
};

#endif // AP_ADSB_SENSOR_H
