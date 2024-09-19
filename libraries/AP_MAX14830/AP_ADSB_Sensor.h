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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_ADSB/GDL90_protocol/GDL90_Message_Structs.h>
#include <AP_ADSB/GDL90_protocol/hostGDL90Support.h>
#include "AP_ADSB/sagetech-sdk/sagetech_mxs.h"
#include "AP_ADSB_State.h"


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
    AP_ADSB_Sensor(AP_HAL::OwnPtr<AP_MAX14830> max14830);

    // Initialize ADSB object
    void init(void);

    // Update function for ADSB out.
    void update(void);

    // Call Sign Message - 1 min interval or on change
    void send_cs_msg(void);

    // Operating Mode Message - 1 Second interval (nominal)
    void send_op_mode_msg(void);

    // VFR Code Message - 1 min interval or on change
    void send_vfr_msg(void);

    // Handle ADSB-UART2 Interrupt
    void handle_adsb_uart3_interrupt(void);

    // Handle complete ABSB message.
    void handle_complete_adsb_msg(const GDL90_RX_MESSAGE &msg);

    // mavlink message handler
    void handle_message(const mavlink_channel_t chan, const mavlink_message_t &msg);

    // control ADSB-out transcievers
    void handle_out_control(const mavlink_uavionix_adsb_out_control_t &packet);

    // Helper function to return operating mode.
    uint8_t get_mode(void);

    // CRC Function for ADSB - ASCII Hex Representation.
    CharPair calc_hex_to_ascii_crc(uint8_t *buf, uint8_t len);

    // Mavlink ADSB out status Packet
    mavlink_uavionix_adsb_out_status_t tx_status;

    // Mavlink ADSB out dynamic Packet
    mavlink_uavionix_adsb_out_dynamic_t tx_dynamic;

    // ADSB Squak Code
    enum ap_var_type ptype;
    AP_Int16   *squawk_octal_param;

    // Timer for last Call Sign Message.
    uint32_t last_cs_msg;

    // Timer for last Op Mode Message.
    uint32_t last_mode_msg;

    // Timer for last Heartbeat Message.
    uint32_t last_Heartbeat_ms;

    // Timer for last Ownship Message.
    uint32_t last_Ownship_ms;

    // Length of bytes to read - returned from Max14830 FIFO.
    uint8_t rxbuf_fifo_len;

    // Array for holding our callsign
    uint8_t callsign[8] = {0};

    // Flag to signal change in Callsign.
    bool cs_flag_change;


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
        bool baroCrossChecked;
        uint8_t airGroundState;
        bool identActive;
        bool modeAEnabled;
        bool modeCEnabled;
        bool modeSEnabled;
        bool es1090TxEnabled;
        int32_t externalBaroAltitude_mm;
        uint16_t squawkCode;
        uint8_t emergencyState;
        uint8_t callsign[8];
        uint8_t mode = MODE::OFF;
        bool x_bit;
    } ctrl; // Declare ctrl as a member variable

private:
    // Pointer to MAX14830 object
    AP_HAL::OwnPtr<AP_MAX14830> _max14830;

    // Structure for ADSB State as need frequent access
    AP_ADSB_State& adsb_state;

    // Write Function for ADSB out.
    bool _tx_write(uint8_t *buffer, uint16_t length);
};
