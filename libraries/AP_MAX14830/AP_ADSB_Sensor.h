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
// ADSB Definitions
/*=========================================================================*/

#define ADSB_UART_ADDR        (UART::ADDR_3)
#define ADSB_MODE_FRAME_SIZE  18
#define ADSB_CS_FRAME_SIZE	  16
#define ADSB_VFR_FRAME_SIZE	  12


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
        ALT  = 0x53,    // 'S'
    };
};

struct IDENT
{
    enum
    {
        ACTVE   = 0x49,  // 'I'
        INACTVE = 0x2D,  // '-'
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

// Forward declaration
class AP_MAX14830;

/*=========================================================================*/
// ADSB Class - UART3
/*=========================================================================*/

class AP_ADSB_Sensor
{
public:
    // Constructor
    AP_ADSB_Sensor(AP_HAL::OwnPtr<AP_MAX14830> max14830);

    void init(void); // Initialize ADSB object
    void update(void); // Update function for ADSB out.
    void handle_adsb_interrupt(void); // Handle ADSB Interrupt
    void handle_message(const mavlink_channel_t chan, const mavlink_message_t &msg); // mavlink message handler

private:
    // Pointer to MAX14830 object
    AP_HAL::OwnPtr<AP_MAX14830> _max14830;

    // Structure for ADSB State as need frequent access
    AP_ADSB_State& adsb_state;

    // Message Handlers
    void send_cs_msg(void);      // Call Sign Message - 1 min interval or on change
    void send_op_mode_msg(void); // Operating Mode Message - 1 Second interval (nominal)
    void send_vfr_msg(void);     // VFR Code Message - 1 min interval or on change
    void handle_complete_adsb_msg(const GDL90_RX_MESSAGE &msg); // Handle complete ABSB message.
    void handle_out_control(const mavlink_uavionix_adsb_out_control_t &packet); // control ADSB-out transcievers
    uint8_t get_mode(void); // Helper function to return operating mode.
    uint8_t uint8_to_hex(uint8_t a); // return the ascii hex character of an uint8 value
    bool _tx_write(uint8_t *buffer, uint16_t length); // Write Function for ADSB out.

    mavlink_uavionix_adsb_out_status_t tx_status; // Mavlink ADSB out status Packet
    mavlink_uavionix_adsb_out_dynamic_t tx_dynamic; // Mavlink ADSB out dynamic Packet
    // ADSB Squak Code
    enum ap_var_type ptype;
    AP_Int16   *squawk_octal_param;

    // Timing and State
    uint32_t last_cs_msg;
    uint32_t last_mode_msg;
    uint32_t last_vfr_msg;
    uint32_t last_Heartbeat_ms;
    uint32_t last_Ownship_ms;
    bool cs_flag_change;
    bool trig_mav_sending;

// Receiver State
    struct {
        uint32_t last_msg_ms;
        GDL90_RX_MESSAGE msg;
        GDL90_RX_STATUS status;
        struct {
            GDL90_HEARTBEAT heartbeat;
            GDL90_OWNSHIP_REPORT ownship_report;
            GDL90_OWNSHIP_GEO_ALTITUDE ownship_geometric_altitude;
        } decoded;
    } rx;

    // Control State
    struct {
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
    } ctrl;

    // Call Sign Command frame
    union CS_CMD {
        struct PACKED {
            // Header
            uint8_t sync;        // '^'
            uint8_t type1;       // 'C'
            uint8_t type2;       // 'S'
            uint8_t space;       // ' '
            // Payload
            uint8_t callsign[8];
            // Footer
            uint8_t crc1;
            uint8_t crc2;
            uint8_t terminator;  // '\r'
        };
        uint8_t data[ADSB_CS_FRAME_SIZE];
    };

    // Operating Mode Command frame
    union MODE_CMD {
        struct PACKED {
            // Header (4 bytes)
            uint8_t sync;        // '^'
            uint8_t type1;       // 'M'
            uint8_t type2;       // 'D'
            uint8_t space;       // ' '
            
            // Payload (9 bytes)
            uint8_t mode;        // Operating mode
            uint8_t comma1;      // ','
            uint8_t ident;       // Ident state
            uint8_t comma2;      // ','
            uint8_t squawk[4];   // Squawk code in ASCII
            uint8_t emergency;   // Emergency state
            uint8_t health;      // Health status '1'
            
            // Footer (3 bytes)
            uint8_t crc1;        // High nibble CRC
            uint8_t crc2;        // Low nibble CRC
            uint8_t terminator;  // '\r'
        };
        uint8_t data[ADSB_MODE_FRAME_SIZE];
    };

    // VFR Command frame
    union VFR_CMD {
        struct PACKED {
            // Header (4 bytes)
            uint8_t sync;        // '^'
            uint8_t type1;       // 'V'
            uint8_t type2;       // 'C'
            uint8_t space;       // ' '
            
            // Payload (4 bytes)
            uint8_t squawk[4];   // Squawk code in ASCII
            
            // Footer (3 bytes)
            uint8_t crc1;        // High nibble CRC
            uint8_t crc2;        // Low nibble CRC
            uint8_t terminator;  // '\r'
        };
        uint8_t data[ADSB_VFR_FRAME_SIZE];
    };
};
