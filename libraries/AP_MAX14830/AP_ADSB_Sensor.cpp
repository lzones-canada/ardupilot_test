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
AP_ADSB_Sensor::AP_ADSB_Sensor(AP_MAX14830* max14830)
    : _max14830(max14830)
{
}


//------------------------------------------------------------------------------
// Periodic update to handle vehicle message timeouts
//------------------------------------------------------------------------------
void AP_ADSB_Sensor::update()
{
    const uint32_t now_ms = AP_HAL::millis();

    // Retrieve Squawk Code from ADSB_SQUAWK parameter.
    squawk_octal_param = (AP_Int16*)AP_Param::find("ADSB_SQUAWK", &ptype);
    squawk_octal = (uint16_t)squawk_octal_param->get();

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
    tx_cs_msg[4]  = callsign[0];
    tx_cs_msg[5]  = callsign[1];
    tx_cs_msg[6]  = callsign[2];
    tx_cs_msg[7]  = callsign[3];
    tx_cs_msg[8]  = callsign[4];
    tx_cs_msg[9]  = callsign[5];
    tx_cs_msg[10] = callsign[6];
    tx_cs_msg[11] = callsign[7];

    // Calculate Checksum - Checksum of bytes 1 through 14. In hex ASCII i.e. “FA”
    CharPair chksum = calc_hex_to_ascii_crc(tx_cs_msg, CS_MSG_LENGTH-4);
    // Extract the two ASCII hex digits from chksum
    tx_cs_msg[12] = chksum.highChar;       // Extract the first hex digit
    tx_cs_msg[13] = chksum.lowChar;        // Extract the second hex digit

    // Append New Line Field (idx: 16)
    tx_cs_msg[14] = 0x0D;  // '\r'

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
    tx_mode_msg[4] = MODE::STBY;

    // Comma Field (idx: 5)
    tx_mode_msg[5] = 0x2C;          // ','

    // Update Ident Field (idx: 6)
    tx_mode_msg[6] = IDENT::ACTVE;

    // Comma Field (idx: 7)
    tx_mode_msg[7] = 0x2C;          // ','

    // Update Squawk Code in Tx Buffer [idx: 8-11]
    //  Convert to ASCII Hex
    tx_mode_msg[8]  = (squawk_octal / 1000)     + '0';   // Thousands digit
    tx_mode_msg[9]  = (squawk_octal / 100) % 10 + '0';   // Hundreds digit
    tx_mode_msg[10] = (squawk_octal / 10) % 10  + '0';   // Tens digit
    tx_mode_msg[11] = (squawk_octal % 10)       + '0';   // Ones digit

    // Update Emergency Field (idx: 12)
    tx_mode_msg[12] = EMERG::NONE;

    // Health bit (idx: 13)
    tx_mode_msg[13] = 0x31;          // In hex ASCII '1'

    // Calculate Checksum - Checksum of bytes 1 through 14. In hex ASCII i.e. “FA”
    CharPair chksum = calc_hex_to_ascii_crc(tx_mode_msg, MODE_MSG_LENGTH-4);
    // Extract the two ASCII hex digits from chksum
    tx_mode_msg[14] = chksum.highChar;        // Extract the first hex digit
    tx_mode_msg[15] = chksum.lowChar;         // Extract the second hex digit

    // Append New Line Field (idx: 16)
    tx_mode_msg[16] = 0x0D;  // '\r'

    // Send out Message.
    _tx_write(tx_mode_msg, MODE_MSG_LENGTH);

    // Save the most recent squawk for mavlink transmission.
    tx_status.squawk = squawk_octal;

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
    tx_vfr_msg[4]  = (squawk_octal / 1000)     + '0';   // Thousands digit
    tx_vfr_msg[5]  = (squawk_octal / 100) % 10 + '0';   // Hundreds digit
    tx_vfr_msg[6] = (squawk_octal / 10) % 10  + '0';   // Tens digit
    tx_vfr_msg[7] = (squawk_octal % 10)       + '0';   // Ones digit

    // Calculate Checksum - Checksum of bytes 1 through 14. In hex ASCII i.e. “FA”
    CharPair chksum = calc_hex_to_ascii_crc(tx_vfr_msg, VFR_MSG_LENGTH-4);
    // Extract the two ASCII hex digits from chksum
    tx_vfr_msg[8] = chksum.highChar;       // Extract the first hex digit
    tx_vfr_msg[9] = chksum.lowChar;        // Extract the second hex digit


    // Append New Line Field (idx: 16)
    tx_vfr_msg[10] = 0x0D;  // '\r'

    // Send out Message.
    _tx_write(tx_vfr_msg, VFR_MSG_LENGTH);

    return;
}

/* ************************************************************************* */

void AP_ADSB_Sensor::handle_adsb_uart2_interrupt()
{
    rx.status.prev_data = GDL90_FLAG_BYTE;
    //---------------------------------------------------------------------------
	// Enumerated data type to define the states of parsing buffer and copying message
	//---------------------------------------------------------------------------
    rx.status.state = GDL90_RX_IDLE;

    // Read Data out of FIFO when interrupt triggered, store current length of FIFO.
    // FIXME: UPDATE TO UART_ADDR_2!
    if (_max14830) {
        rxbuf_fifo_len = _max14830->rx_read(rx_fifo_buffer, MESSAGE_BUFFER_LENGTH, UART_ADDR_1);
    }

    // Pointer to the start of FIFO buffer.
    const uint8_t *byte_ptr = &rx_fifo_buffer[0];
    // Byte length to track the number of bytes parsed.
    uint8_t byte_count = 0;

    // Parse the data in buffer.
    while(true)
    {
        // Parse all data until the end of the fifo buffer.
        if(rxbuf_fifo_len == byte_count) {
            // Finished converting all new data, Clear the interrupt and reset buffer.
            if (_max14830) {
                _max14830->clear_interrupts();
            }
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
            tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_ON_GROUND;

            if (rx.decoded.heartbeat.status.one.maintenanceRequired) {
                tx_status.fault |= UAVIONIX_ADSB_OUT_STATUS_FAULT_MAINT_REQ;
            } else {
                tx_status.fault &= ~UAVIONIX_ADSB_OUT_STATUS_FAULT_MAINT_REQ;
            }

            if (rx.decoded.heartbeat.status.two.functionFailureGnssUnavailable) {
                tx_status.fault |= UAVIONIX_ADSB_OUT_STATUS_FAULT_GPS_UNAVAIL;
            } else {
                tx_status.fault &= ~UAVIONIX_ADSB_OUT_STATUS_FAULT_GPS_UNAVAIL;
            }

            if (rx.decoded.heartbeat.status.two.functionFailureGnssNo3dFix) {
                tx_status.fault |= UAVIONIX_ADSB_OUT_STATUS_FAULT_GPS_NO_POS;
            } else {
                tx_status.fault &= ~UAVIONIX_ADSB_OUT_STATUS_FAULT_GPS_NO_POS;
            }

            if (rx.decoded.heartbeat.status.two.functionFailureTransmitSystem) {
                tx_status.fault |= UAVIONIX_ADSB_OUT_STATUS_FAULT_TX_SYSTEM_FAIL;
            } else {
                tx_status.fault &= ~UAVIONIX_ADSB_OUT_STATUS_FAULT_TX_SYSTEM_FAIL;
            }

            tx_status.fault &= ~UAVIONIX_ADSB_OUT_STATUS_FAULT_STATUS_MESSAGE_UNAVAIL;

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
            tx_status.NIC_NACp = rx.decoded.ownship_report.report.NIC | (rx.decoded.ownship_report.report.NACp << 4);
            memcpy(tx_status.flight_id, rx.decoded.ownship_report.report.callsign, sizeof(tx_status.flight_id));
            // there is no message in the vocabulary of the 200x that has board temperature
            //  tx_status.temperature = rx.decoded.ownship_report.report.temperature;

            //GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"GDL90_ID_OWNSHIP_REPORT");
            break;
        }
        case(GDL90_ID_OWNSHIP_GEOMETRIC_ALTITUDE):
        {
            // An Ownship Geometric Altitude message will be transmitted with a period of one second when
            // the GNSS fix is valid for the UCP protocol. All fields in the Geometric Ownship Altitude
            // message are transmitted MSB first.
            memcpy(&rx.decoded.ownship_geometric_altitude, rx.msg.raw, sizeof(rx.decoded.ownship_geometric_altitude));

            //GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"GDL90_ID_OWNSHIP_GEOMETRIC_ALTITUDE");
            break;
    
        }
        default:
        {
            // This should never happen.
            break;
        }
    }

    return;
}

/* ************************************************************************* */

// send a periodic report of the ADSB out status
void AP_ADSB_Sensor::send_adsb_out_status(const mavlink_channel_t chan)
{
    // Send out ADSB out status
    mavlink_msg_uavionix_adsb_out_status_send_struct(chan, &tx_status);

    return;
}

/* ************************************************************************* */

// DATA16 Msg leveraged as a conduit message for Call Sign.
void AP_ADSB_Sensor::handle_data16_packet(mavlink_channel_t chan, const mavlink_data16_t &m)
{
    // Extract Call Sign from packet.
    memcpy(&callsign, &m.data[0], sizeof(callsign));
    // Signal change of callsign flag.
    cs_flag_change = true;

    return;
}

/* ************************************************************************* */

CharPair AP_ADSB_Sensor::calc_hex_to_ascii_crc(uint8_t *buf, uint8_t len)
{
    // The checksum is algebraic sum of the message byte values.
    uint8_t chksum = 0;

    for(int i=0; i<len; i++) {
        chksum += buf[i];
    }

    // Extract the high and low nibbles
    unsigned char highNibble = (chksum >> 4) & 0xF;
    unsigned char lowNibble = chksum & 0xF;

    // Convert the high and low nibbles to ASCII characters
    char highChar = (highNibble < 10) ? ('0' + highNibble) : ('A' + (highNibble - 10));
    char lowChar = (lowNibble < 10) ? ('0' + lowNibble) : ('A' + (lowNibble - 10));

    CharPair charPair;
    charPair.highChar = highChar;
    charPair.lowChar = lowChar;

    return charPair;
}

/* ************************************************************************* */

bool AP_ADSB_Sensor::_tx_write(uint8_t *buffer, uint16_t length)
{
    if (_max14830 == nullptr) {
      return false;
    }

    // FIXME: Update to use UART_ADDR_2!!
    _max14830->tx_write(buffer, length, UART_ADDR_1);
    return true;
}

/* ************************************************************************* */