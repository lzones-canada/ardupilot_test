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
 * AP_ADSB_Sensor.cpp
 *
 *      Author: Kyle Fruson
 */

#include "AP_ADSB_Sensor.h"
#include "AP_MAX14830.h"
#include <cstdio> // Add this line to include the required header



//------------------------------------------------------------------------------
// Time Interval/Timeout Constants.
//------------------------------------------------------------------------------
#define HEALTHY_LAST_RECEIVED_MSG       10000     // 10sec Timeout.
#define VEHICLE_OP_MODE                 1000      // 1sec Interval.
#define VEHICLE_CS_MSG                  60 *1000  // 1min Interval.
// GDL90 Heading Resolution
#define GDL_RES_HEAD                    (256.0 / 360.0)
// GDL90 Vertical Velocity Resolution
#define GDL_RES_VERTVEL                 64.0
// GDL90 GNNS Altitude Resolution
#define GDL_RES_ALT                     5.0


//------------------------------------------------------------------------------
// Static FIFO buffer to retrieve rx message from ABSB UART2 FIFO
//------------------------------------------------------------------------------
static const uint8_t MESSAGE_BUFFER_LENGTH = 255;
static uint8_t rx_fifo_buffer[MESSAGE_BUFFER_LENGTH];

//------------------------------------------------------------------------------
// Static buffer to store/send messages.
//------------------------------------------------------------------------------

// Tx Buffer for Operating Mode Message.
static const uint8_t MODE_MSG_LENGTH = 18;
static uint8_t tx_mode_msg[MODE_MSG_LENGTH];

// Tx Buffer for Call Sign Message.
static const uint8_t CS_MSG_LENGTH = 16;
static uint8_t tx_cs_msg[CS_MSG_LENGTH];

// Tx Buffer for VFR Code Message.
static const uint8_t VFR_MSG_LENGTH = 12;
static uint8_t tx_vfr_msg[VFR_MSG_LENGTH];



//------------------------------------------------------------------------------
// Constructor
//------------------------------------------------------------------------------
AP_ADSB_Sensor::AP_ADSB_Sensor(AP_HAL::OwnPtr<AP_MAX14830> max14830) :
    _max14830(std::move(max14830)),
    adsb_state(AP_ADSB_State::get_singleton())
{
}

//------------------------------------------------------------------------------

void AP_ADSB_Sensor::init()
{
    // Retrieve Squawk Code from ADSB_SQUAWK parameter and Init value from Parameter.
    squawk_octal_param = (AP_Int16*)AP_Param::find("ADSB_SQUAWK", &ptype);
    ctrl.squawkCode = static_cast<uint16_t>(squawk_octal_param->get());

    return;
}

//------------------------------------------------------------------------------
// Periodic update to handle vehicle message timeouts
//------------------------------------------------------------------------------
void AP_ADSB_Sensor::update()
{
    const uint32_t now_ms = AP_HAL::millis();

    // Call Sign Message - 1 min interval or on change
    if(now_ms - last_cs_msg >= VEHICLE_CS_MSG || cs_flag_change) 
    {
        send_cs_msg();
        last_cs_msg = now_ms;
        cs_flag_change = false;
    }

    // Operating Mode Message - 1 Second Interval (nominal)
    if(now_ms - last_mode_msg >= VEHICLE_OP_MODE) 
    {
        send_op_mode_msg();
        last_mode_msg = now_ms;
    }

    // if the transponder has stopped giving us the data needed to fill the transponder status mavlink message, reset that data.
    if ((now_ms - last_Heartbeat_ms >= HEALTHY_LAST_RECEIVED_MSG && last_Heartbeat_ms != 0) &&
        (now_ms - last_Ownship_ms   >= HEALTHY_LAST_RECEIVED_MSG && last_Ownship_ms   != 0))
    {
        tx_status.fault |= UAVIONIX_ADSB_OUT_STATUS_FAULT_STATUS_MESSAGE_UNAVAIL;
    }

    return;
}

/* ************************************************************************* */

void AP_ADSB_Sensor::send_cs_msg()
{
    // Header (idx: 0-3)
    tx_cs_msg[0] = 0x5E;          // '^'
    tx_cs_msg[1] = 0x43;          // 'C'
    tx_cs_msg[2] = 0x53;          // 'S'
    tx_cs_msg[3] = 0x20;          // ' '
    // Update ASCII Flight ID (idx: 4)
    // TODO: Update to use MAVLink Callsign
    tx_cs_msg[4]  = ctrl.callsign[0];
    tx_cs_msg[5]  = ctrl.callsign[1];
    tx_cs_msg[6]  = ctrl.callsign[2];
    tx_cs_msg[7]  = ctrl.callsign[3];
    tx_cs_msg[8]  = ctrl.callsign[4];
    tx_cs_msg[9]  = ctrl.callsign[5];
    tx_cs_msg[10] = ctrl.callsign[6];
    tx_cs_msg[11] = ctrl.callsign[7];

    // Calculate Checksum - Checksum of bytes 1 through 14. In hex ASCII i.e. “FA”
    chksum = crc_sum_of_bytes(tx_cs_msg, CS_MSG_LENGTH-4);
    // Extract the high and low nibbles and convert to ASCII
    highChar = uint8_to_hex(HIGH_NIBBLE(chksum));
    lowChar  = uint8_to_hex(LOW_NIBBLE(chksum));
    // Extract the two ASCII hex digits from chksum
    tx_cs_msg[12] = highChar;       // Extract the first hex digit
    tx_cs_msg[13] = lowChar;        // Extract the second hex digit

    // Append New Line Field (idx: 16)
    tx_cs_msg[14] = 0x0D;  // '\r'

    //GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"tx_cs_msg: %s", tx_cs_msg);

    // Send out Message.
    _tx_write(tx_cs_msg, CS_MSG_LENGTH);


    return;
}

/* ************************************************************************* */

void AP_ADSB_Sensor::send_op_mode_msg()
{
    // Header (idx: 0-3)
    tx_mode_msg[0] = 0x5E;          // '^'
    tx_mode_msg[1] = 0x4D;          // 'M'
    tx_mode_msg[2] = 0x44;          // 'D'
    tx_mode_msg[3] = 0x20;          // ' '

    // Update Mode Field (idx: 4)
    tx_mode_msg[4] = get_mode();

    // Comma Field (idx: 5)
    tx_mode_msg[5] = 0x2C;          // ','

    // Update Ident Field (idx: 6)
    tx_mode_msg[6] = ctrl.identActive ? IDENT::ACTVE : IDENT::INACTVE;
    // only send identButtonActive once per request
    ctrl.identActive = false;

    // Comma Field (idx: 7)
    tx_mode_msg[7] = 0x2C;          // ','

    // Update Squawk Code in Tx Buffer [idx: 8-11]
    //  Convert to ASCII Hex
    tx_mode_msg[8]  = (ctrl.squawkCode / 1000)     + '0';   // Thousands digit
    tx_mode_msg[9]  = (ctrl.squawkCode / 100) % 10 + '0';   // Hundreds digit
    tx_mode_msg[10] = (ctrl.squawkCode / 10) % 10  + '0';   // Tens digit
    tx_mode_msg[11] = (ctrl.squawkCode % 10)       + '0';   // Ones digit

    // Update Emergency Field (idx: 12)
    tx_mode_msg[12] = (sg_emergc_t) ctrl.emergencyState + '0'; // In hex ASCII

    // Health bit (idx: 13)
    tx_mode_msg[13] = 0x31;          // In hex ASCII '1'

    // Calculate Checksum - Checksum of bytes 1 through 14. In hex ASCII i.e. “FA”
    chksum = crc_sum_of_bytes(tx_mode_msg, MODE_MSG_LENGTH-4);
    // Extract the high and low nibbles and convert to ASCII
    highChar = uint8_to_hex(HIGH_NIBBLE(chksum));
    lowChar  = uint8_to_hex(LOW_NIBBLE(chksum));
    // Extract the two ASCII hex digits from chksum
    tx_mode_msg[14] = highChar;        // Extract the first hex digit
    tx_mode_msg[15] = lowChar;         // Extract the second hex digit

    // Append New Line Field (idx: 16)
    tx_mode_msg[16] = 0x0D;  // '\r'

    //GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"tx_mode_msg: %s", tx_mode_msg);

    // Send out Message.
    _tx_write(tx_mode_msg, MODE_MSG_LENGTH);

    return;
}

/* ************************************************************************* */

void AP_ADSB_Sensor::send_vfr_msg()
{
    // Header (idx: 0-3)
    tx_vfr_msg[0] = 0x5E;          // '^'
    tx_vfr_msg[1] = 0x56;          // 'V' 
    tx_vfr_msg[2] = 0x43;          // 'C'
    tx_vfr_msg[3] = 0x20;          // ' '
    // Update Squawk Code in Tx Buffer [idx: 8-11]
    tx_vfr_msg[4] = (ctrl.squawkCode / 1000)     + '0';   // Thousands digit
    tx_vfr_msg[5] = (ctrl.squawkCode / 100) % 10 + '0';   // Hundreds digit
    tx_vfr_msg[6] = (ctrl.squawkCode / 10) % 10  + '0';   // Tens digit
    tx_vfr_msg[7] = (ctrl.squawkCode % 10)       + '0';   // Ones digit

    // Calculate Checksum - Checksum of bytes 1 through 14. In hex ASCII i.e. “FA”
    chksum = crc_sum_of_bytes(tx_mode_msg, MODE_MSG_LENGTH-4);
    // Extract the high and low nibbles and convert to ASCII
    highChar = uint8_to_hex(HIGH_NIBBLE(chksum));
    lowChar  = uint8_to_hex(LOW_NIBBLE(chksum));
    // Extract the two ASCII hex digits from chksum
    tx_vfr_msg[8] = highChar;       // Extract the first hex digit
    tx_vfr_msg[9] = lowChar;        // Extract the second hex digit


    // Append New Line Field (idx: 16)
    tx_vfr_msg[10] = 0x0D;  // '\r'

    // Send out Message.
    _tx_write(tx_vfr_msg, VFR_MSG_LENGTH);

    return;
}

/* ************************************************************************* */

void AP_ADSB_Sensor::handle_adsb_uart3_interrupt()
{
    rx.status.prev_data = GDL90_FLAG_BYTE;
    //---------------------------------------------------------------------------
    // Enumerated data type to define the states of parsing buffer and copying message
    //---------------------------------------------------------------------------
    rx.status.state = GDL90_RX_IDLE;

    // Read Data out of FIFO when interrupt triggered, store current length of FIFO.
    _max14830->set_uart_address(ADSB_UART_ADDR);
    const uint16_t bytes_read = _max14830->rx_read(rx_fifo_buffer, MESSAGE_BUFFER_LENGTH);
    // Clear the interrupt.
    _max14830->clear_interrupts();

    // Pointer to the start of FIFO buffer.
    const uint8_t *byte_ptr = &rx_fifo_buffer[0];
    // Byte length to track the number of bytes parsed.
    uint8_t byte_count = 0;

    // Parse the data in buffer.
    while(true)
    {
        // Parse all data until the end of the fifo buffer.
        if(bytes_read == byte_count) {
            // Finished converting all new data.
            break;
        }

        // Track length of buffer pointer.
        byte_count++;

        // Message parsing state machine.
        switch(rx.status.state)
        {
            case(GDL90_RX_IDLE):
            {
                // 0x7E is the flag byte. If we see it, we're in a new message.
                if(GDL90_FLAG_BYTE == *byte_ptr && GDL90_FLAG_BYTE == rx.status.prev_data)
                {
                    rx.status.length = 0;
                    rx.status.state = GDL90_RX_IN_PACKET;
                }
                // This if statement has no else, as this will be normal while the
                //  contents of the message are being copied to the message buffer.
                break;
            }
            case(GDL90_RX_IN_PACKET):
            {
                if(GDL90_CONTROL_ESCAPE_BYTE == *byte_ptr)
                {
                    rx.status.state = GDL90_RX_UNSTUFF;
                }
                else if (GDL90_FLAG_BYTE == *byte_ptr) 
                {
                    // packet complete! Check CRC and restart packet cycle on all pass or fail scenarios
                    rx.status.state = GDL90_RX_IDLE;

                    if (rx.status.length < GDL90_OVERHEAD_LENGTH) 
                    {
                        // something is wrong, there's no actual data
                        return;
                    }

                    const uint8_t crc_LSB = rx.msg.raw[rx.status.length - 2];
                    const uint8_t crc_MSB = rx.msg.raw[rx.status.length - 1];

                    // NOTE: status.length contains messageId, payload and CRC16. So status.length-3 is effective payload length
                    rx.msg.crc = (uint16_t)crc_LSB | ((uint16_t)crc_MSB << 8);
                    const uint16_t crc = crc16_ccitt_GDL90((uint8_t*)&rx.msg.raw, rx.status.length-2, 0);
                    //GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"CRC: 0x%02X, msgCRC: 0x%02X", crc, rx.msg.crc);
                    if (crc == rx.msg.crc) {
                        rx.status.prev_data = *byte_ptr;
                        // NOTE: this is the only path that returns true ie. HANDLE COMPLETE MESSAGE HERE.
                        handle_complete_adsb_msg(rx.msg);
                    }
                }
                else if (rx.status.length < GDL90_RX_MAX_PACKET_LENGTH) 
                {
                    rx.msg.raw[rx.status.length++] = *byte_ptr;
                } 
                else 
                {
                    rx.status.state = GDL90_RX_IDLE;
                }
                break;
            }
            case(GDL90_RX_UNSTUFF):
            {
                rx.msg.raw[rx.status.length++] = *byte_ptr ^ GDL90_STUFF_BYTE;
                rx.status.state = GDL90_RX_IN_PACKET;
                break;
            }
            default:
            {
                // This should never happen.
                break;
            }
        }
        // Store previous byte.
        rx.status.prev_data = *byte_ptr;
        // Increment byte index.
        byte_ptr++;
    }

    return;
}

/* ************************************************************************* */

void AP_ADSB_Sensor::handle_complete_adsb_msg(const GDL90_RX_MESSAGE &msg)
{
    // Handle the message based on the message ID.
    switch(msg.messageId)
    {
        case(GDL90_ID_HEARTBEAT):
        {
            // The Heartbeat message provides real-time indications of the status and operation of the
            // transponder. The message will be transmitted with a period of one second for the UCP
            // protocol.
            memcpy(&rx.decoded.heartbeat, msg.raw, sizeof(rx.decoded.heartbeat));
            last_Heartbeat_ms = AP_HAL::millis();

            // this is always true. The "ground/air bit place" is set meaning we're always in the air
            tx_status.state  |= UAVIONIX_ADSB_OUT_STATUS_STATE_ON_GROUND;
            tx_dynamic.state |= UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND;
            // Faults
            if (rx.decoded.heartbeat.status.one.maintenanceRequired) {
                tx_status.fault |= UAVIONIX_ADSB_OUT_STATUS_FAULT_MAINT_REQ;
            } else {
                tx_status.fault &= ~UAVIONIX_ADSB_OUT_STATUS_FAULT_MAINT_REQ;
            }
            // Faults
            if (rx.decoded.heartbeat.status.two.functionFailureGnssUnavailable) {
                tx_status.fault |= UAVIONIX_ADSB_OUT_STATUS_FAULT_GPS_UNAVAIL;
            } else {
                tx_status.fault &= ~UAVIONIX_ADSB_OUT_STATUS_FAULT_GPS_UNAVAIL;
            }
            // Faults
            if (rx.decoded.heartbeat.status.two.functionFailureGnssNo3dFix) {
                tx_status.fault |= UAVIONIX_ADSB_OUT_STATUS_FAULT_GPS_NO_POS;
            } else {
                tx_status.fault &= ~UAVIONIX_ADSB_OUT_STATUS_FAULT_GPS_NO_POS;
            }
            // Faults
            if (rx.decoded.heartbeat.status.two.functionFailureTransmitSystem) {
                tx_status.fault |= UAVIONIX_ADSB_OUT_STATUS_FAULT_TX_SYSTEM_FAIL;
            } else {
                tx_status.fault &= ~UAVIONIX_ADSB_OUT_STATUS_FAULT_TX_SYSTEM_FAIL;
            }
            // Faults
            tx_status.fault &= ~UAVIONIX_ADSB_OUT_STATUS_FAULT_STATUS_MESSAGE_UNAVAIL;

            // Identifier
            if (rx.decoded.heartbeat.status.one.ident) {
                tx_status.state  |= UAVIONIX_ADSB_OUT_STATUS_STATE_IDENT_ACTIVE;
                tx_dynamic.state |= UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT;
            } else {
                tx_status.state  &= ~UAVIONIX_ADSB_OUT_STATUS_STATE_IDENT_ACTIVE;
                tx_dynamic.state &= ~UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT;
            }

            // GPS Fix
            if (rx.decoded.heartbeat.status.one.gpsPositionValid) {
                tx_dynamic.gpsFix = GPS_FIX_TYPE_3D_FIX;
            } else {
                tx_dynamic.gpsFix = GPS_FIX_TYPE_NO_GPS;
            }
            
            // UTC Time
            tx_dynamic.utcTime = rx.decoded.heartbeat.timestamp;

            //GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"GDL90_ID_HEARTBEAT");
            break;
            
        }
        case(GDL90_ID_OWNSHIP_REPORT):
        {
            // The Ownship message contains information on the GNSS position. If the Ownship GNSS
            // position fix is invalid, the Latitude, Longitude, and NIC fields will all have the ZERO value. The
            // Ownship message will be transmitted with a period of one second regardless of data status or
            // update for the UCP protocol. All fields in the ownship message are transmitted MSB first
            memcpy(&rx.decoded.ownship_report, rx.msg.raw, sizeof(rx.decoded.ownship_report));
            last_Ownship_ms = AP_HAL::millis();
            // Integrity (NIC) & Accuracy (NACp)
            tx_status.NIC_NACp = rx.decoded.ownship_report.report.NIC | (rx.decoded.ownship_report.report.NACp << 4);
            // Flight ID
            memcpy(tx_status.flight_id, rx.decoded.ownship_report.report.callsign, sizeof(tx_status.flight_id));
            // there is no message in the vocabulary of the 200x that has board temperature
            //  tx_status.temperature = rx.decoded.ownship_report.report.temperature;

            // Latitude
            double latitude = toLatLon(rx.decoded.ownship_report.report.latitude);
            tx_dynamic.gpsLat = static_cast<int32_t>(latitude * 1e7);

            // Longitude
            double longitude = toLatLon(rx.decoded.ownship_report.report.longitude);
            tx_dynamic.gpsLon = static_cast<int32_t>(longitude * 1e7);

            // Extract the altitudeMisc field
            uint16_t altitudeMiscValue = rx.decoded.ownship_report.report.altitudeMisc;
            // Flip byte order as per the GDL90 Message Struct
            rx.decoded.ownship_report.report.altitudeMisc = htobe16(altitudeMiscValue);

            // GPS/GNNS Altitude False (INT32_MAX) when invalid GPS Fix
            //  Ownship Geometric Alt message fills in data when GPS Fix is valid
            if(!tx_dynamic.gpsFix)
                tx_dynamic.gpsAlt = INT32_MAX;

            // Barometric ALtitude (referenced to 29.92 inches Hg)
            tx_dynamic.baroAltMSL = toAlt2(rx.decoded.ownship_report.report.altitude);

            // Extract the velocities field
            uint32_t velocitiesValue = rx.decoded.ownship_report.report.velocities;
            // Flip byte order as per the GDL90 Message Struct
            rx.decoded.ownship_report.report.velocities = htobe32(velocitiesValue);

            // Heading (Repurposed in Vertical Accuracy)
            tx_dynamic.accuracyVert = rx.decoded.ownship_report.report.heading * GDL_RES_HEAD;

            // Vertical Velocity - 64 Feet per Minute
            tx_dynamic.velVert = rx.decoded.ownship_report.report.verticalVelocity * GDL_RES_VERTVEL;

            // Horizontal Velocity - Knots
            tx_dynamic.accuracyHor = rx.decoded.ownship_report.report.horizontalVelocity;

            // Emergency Status
            tx_dynamic.emergencyStatus = rx.decoded.ownship_report.report.emergencyCode;

            // Unused Fields ***
            tx_dynamic.accuracyVel = UINT16_MAX;
            tx_dynamic.velNS = INT16_MAX;
            tx_dynamic.VelEW = INT16_MAX;
            tx_dynamic.numSats = UINT8_MAX;


            // Send out Mavlink ADSB status messages
            gcs().send_to_active_channels(MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC, (const char *)&tx_dynamic);
            gcs().send_to_active_channels(MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS, (const char *)&tx_status);

            //GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"GDL90_ID_OWNSHIP_REPORT");
            break;
        }
        case(GDL90_ID_OWNSHIP_GEOMETRIC_ALTITUDE):
        {
            // An Ownship Geometric Altitude message will be transmitted with a period of one second when
            // the GNSS fix is valid for the UCP protocol. All fields in the Geometric Ownship Altitude
            // message are transmitted MSB first.
            memcpy(&rx.decoded.ownship_geometric_altitude, rx.msg.raw, sizeof(rx.decoded.ownship_geometric_altitude));
            // GNSS Altitude
            tx_dynamic.gpsAlt = htobe16(rx.decoded.ownship_geometric_altitude.geometricAltitude) * GDL_RES_ALT;

            break;
    
        }
        default:
        {
            // This should never happen.
            break;
        }
    }

    // Squawk Code
    tx_status.squawk  = ctrl.squawkCode;
    tx_dynamic.squawk = ctrl.squawkCode;

    // Operating Mode        
    if (ctrl.modeAEnabled) {
        tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_MODE_A_ENABLED;
    } else {
        tx_status.state &= ~UAVIONIX_ADSB_OUT_STATUS_STATE_MODE_A_ENABLED;
    }

    if (ctrl.modeCEnabled) {
        tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_MODE_C_ENABLED;
    } else {
        tx_status.state &= ~UAVIONIX_ADSB_OUT_STATUS_STATE_MODE_C_ENABLED;
    }

    if (ctrl.modeSEnabled) {
        tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_MODE_S_ENABLED;
    } else {
        tx_status.state &= ~UAVIONIX_ADSB_OUT_STATUS_STATE_MODE_S_ENABLED;
    }

    if (ctrl.es1090TxEnabled) {
        tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_1090ES_TX_ENABLED;
    } else {
        tx_status.state &= ~UAVIONIX_ADSB_OUT_STATUS_STATE_1090ES_TX_ENABLED;
    }

    if (ctrl.x_bit) {
        tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_XBIT_ENABLED;
    } else {
        tx_status.state &= ~UAVIONIX_ADSB_OUT_STATUS_STATE_XBIT_ENABLED;
    }

    return;
}

/* ************************************************************************* */

void AP_ADSB_Sensor::handle_message(const mavlink_channel_t chan, const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL)
    {
        mavlink_uavionix_adsb_out_control_t packet {};
        mavlink_msg_uavionix_adsb_out_control_decode(&msg, &packet);
        handle_out_control(packet);
    }
    
    return;
}

/* ************************************************************************* */

/*
 * handle incoming packet UAVIONIX_ADSB_OUT_CONTROL
 * allows a GCS to set the contents of the control message sent by ardupilot to the transponder
 */
void AP_ADSB_Sensor::handle_out_control(const mavlink_uavionix_adsb_out_control_t &packet)
{
    ctrl.baroCrossChecked = packet.state & UAVIONIX_ADSB_OUT_CONTROL_STATE::UAVIONIX_ADSB_OUT_CONTROL_STATE_EXTERNAL_BARO_CROSSCHECKED;
    ctrl.airGroundState = packet.state & UAVIONIX_ADSB_OUT_CONTROL_STATE::UAVIONIX_ADSB_OUT_CONTROL_STATE_ON_GROUND;
    ctrl.identActive = packet.state & UAVIONIX_ADSB_OUT_CONTROL_STATE::UAVIONIX_ADSB_OUT_CONTROL_STATE_IDENT_BUTTON_ACTIVE;
    ctrl.modeAEnabled = packet.state & UAVIONIX_ADSB_OUT_CONTROL_STATE::UAVIONIX_ADSB_OUT_CONTROL_STATE_MODE_A_ENABLED;
    ctrl.modeCEnabled = packet.state & UAVIONIX_ADSB_OUT_CONTROL_STATE::UAVIONIX_ADSB_OUT_CONTROL_STATE_MODE_C_ENABLED;
    ctrl.modeSEnabled = packet.state & UAVIONIX_ADSB_OUT_CONTROL_STATE::UAVIONIX_ADSB_OUT_CONTROL_STATE_MODE_S_ENABLED;
    ctrl.es1090TxEnabled = packet.state & UAVIONIX_ADSB_OUT_CONTROL_STATE::UAVIONIX_ADSB_OUT_CONTROL_STATE_1090ES_TX_ENABLED;
    ctrl.externalBaroAltitude_mm = packet.baroAltMSL;
    ctrl.squawkCode = packet.squawk;
    ctrl.emergencyState = packet.emergencyStatus;
    memcpy(ctrl.callsign, packet.flight_id, sizeof(ctrl.callsign));
    ctrl.x_bit = packet.x_bit;

    // Signal change of callsign flag.
    cs_flag_change = true;

    return;
}

/* ************************************************************************* */

/*
 * helper function for converting an uint8 value to its ASCII hex character.
 */

uint8_t AP_ADSB_Sensor::uint8_to_hex(uint8_t val)
{
    uint8_t res = 0;

    if (val <= 9) {
        res = val + 0x30;  // 0-9 -> '0'-'9'
    } else if (val >= 10 && val <= 15) {
        res = val + 0x41 - 10;  // 10-15 -> 'A'-'F'
    } else {
        return res;  // Input is out of valid range (0-15)
    }
    return res;
}

/* ************************************************************************* */

bool AP_ADSB_Sensor::_tx_write(uint8_t *buffer, uint16_t length)
{
    _max14830->set_uart_address(ADSB_UART_ADDR);
    _max14830->tx_write(buffer, length);

    return true;
}

/* ************************************************************************* */

uint8_t AP_ADSB_Sensor::get_mode()
{
    if (!ctrl.modeAEnabled && !ctrl.modeCEnabled && !ctrl.modeSEnabled && !ctrl.es1090TxEnabled) {
        ctrl.mode = MODE::STBY;
    }
    if (ctrl.modeAEnabled && !ctrl.modeCEnabled && ctrl.modeSEnabled && ctrl.es1090TxEnabled) {
        ctrl.mode = MODE::ON;
    }
    if (ctrl.modeAEnabled && ctrl.modeCEnabled &&  ctrl.modeSEnabled && ctrl.es1090TxEnabled) {
        ctrl.mode = MODE::ALT;
    }
    // Turn off the transponder if we are in failsafe
    if (adsb_state.get_adsb_failsafe()) {
        ctrl.mode = MODE::OFF;
    }

    return ctrl.mode;
}

/* ************************************************************************* */
