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


#define VEHICLE_OP_MODE           800   // 1sec Timeout. Use 800ms to be safe.
#define VEHICLE_CS_MSG            48000 // 0.8 minute interval

//------------------------------------------------------------------------------
// Static FIFO buffer to retrieve rx message from ABSB UART2 FIFO
//------------------------------------------------------------------------------
static const uint8_t MESSAGE_BUFFER_LENGTH = 255;
static uint8_t rx_fifo_buffer[MESSAGE_BUFFER_LENGTH];

//------------------------------------------------------------------------------
// Static buffer to store/send messages.
//------------------------------------------------------------------------------

static uint8_t rx_buffer[MESSAGE_BUFFER_LENGTH];

// Tx Buffer for Operating Mode Message.
// FIXME: Minus 1 Length when '\n' DELETED! */
static const uint8_t MODE_MSG_LENGTH = 18;
static uint8_t tx_mode_msg[MODE_MSG_LENGTH];

// Tx Buffer for Call Sign Message.
// FIXME: Minus 1 Length when '\n' DELETED! */
static const uint8_t CS_MSG_LENGTH = 16;
static uint8_t tx_cs_msg[CS_MSG_LENGTH];


//------------------------------------------------------------------------------
// Static pointer to control access to the contents of message_buffer.
//------------------------------------------------------------------------------

static uint8_t *rx_buffer_ptr = rx_buffer;

/* ************************************************************************* */


//------------------------------------------------------------------------------
// Constructor
//------------------------------------------------------------------------------
AP_ADSB_Sensor::AP_ADSB_Sensor(AP_MAX14830* max14830)
    : _max14830(max14830)
{
}


/*
 * periodic update to handle vehicle message timeouts
 */
void AP_ADSB_Sensor::update()
{
    const uint32_t now = AP_HAL::millis();

    // Custom ADSB request message to be sent out on first update.
    // TODO: Evaluate if Necessary..
    if(!init_flag) {
        request_msg(GDL90_ID_IDENTIFICATION);
        request_msg(GDL90_ID_TRANSPONDER_CONFIG);
        // Latch
        init_flag = true;
    }

    // Retrieve Squawk Code from ADSB_SQUAWK parameter.
    squawk_octal_param = (AP_Int16*)AP_Param::find("ADSB_SQUAWK", &ptype);
    squawk_octal = (uint16_t)squawk_octal_param->get();

    // TODO: Check if callsign has changed. If so, set flag to true.

    // Call Sign Message - 1 min interval or on change
    // FIXME: Update Timer OR CHANGE
    if(now - last_cs_msg > 10000) {
        send_cs_msg();
        last_cs_msg = now;
    }

    // Operating Mode Message - 1 Second Interval (nominal)
    if(now - last_mode_msg > VEHICLE_OP_MODE) {
        send_op_mode_msg();
        last_mode_msg = now;
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
    tx_cs_msg[4]  = 'U';
    tx_cs_msg[5]  = 'A';
    tx_cs_msg[6]  = 'V';
    tx_cs_msg[7]  = 'I';
    tx_cs_msg[8]  = 'O';
    tx_cs_msg[9]  = 'N';
    tx_cs_msg[10] = 'I';
    tx_cs_msg[11] = 'X';

    // Calculate Checksum - Checksum of bytes 1 through 14. In hex ASCII i.e. “FA”
    uint8_t chksum = calc_hex_crc(tx_cs_msg, CS_MSG_LENGTH-4);
    // Update Checksum Field (idx: 14-15)
    // Extract the two hex digits from chksum
    tx_cs_msg[12] = ((chksum >> 4) & 0xF) +  '0';  // Extract the first hex digit
    tx_cs_msg[13] = (chksum & 0xF)        +  '0';  // Extract the second hex digit

    // Append New Line Field (idx: 16)
    tx_cs_msg[14] = 0x0D;  // '\r'



    // FIXME: TEMPORARY FOR PRINTING!!!************************************************ */
    tx_cs_msg[15] = 0x0A;  // '\n'

    // Send out Message.
    hostTransmit(tx_cs_msg, CS_MSG_LENGTH);

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
    uint8_t chksum = calc_hex_crc(tx_mode_msg, MODE_MSG_LENGTH-4);
    // Update Checksum Field (idx: 14-15)
    // Extract the two hex digits from chksum
    tx_mode_msg[14] = ((chksum >> 4) & 0xF) + '0';  // Extract the first hex digit
    tx_mode_msg[15] = (chksum & 0xF)        +  '0'; // Extract the second hex digit

    // Append New Line Field (idx: 16)
    tx_mode_msg[16] = 0x0D;  // '\r'


    // FIXME: TEMPORARY FOR PRINTING!!!************************************************ */
    tx_mode_msg[17] = 0x0A;  // '\n'

    // Send out Message.
    hostTransmit(tx_mode_msg, MODE_MSG_LENGTH);

    return;
}

/* ************************************************************************* */

void AP_ADSB_Sensor::handle_adsb_uart2_interrupt()
{    
    // Fill the array with the specified values

    // Flag Byte
    rx_fifo_buffer[0] = GDL90_FLAG_BYTE;
    // Msg ID - Heartbeat
    rx_fifo_buffer[1] = GDL90_ID_HEARTBEAT;
    // Data - 7
    rx_fifo_buffer[2] = 0x81;
    rx_fifo_buffer[3] = 0x41;
    rx_fifo_buffer[4] = 0xDB;
    rx_fifo_buffer[5] = 0xD0;
    rx_fifo_buffer[6] = 0x08;
    rx_fifo_buffer[7] = 0x02;
    // CRC
    rx_fifo_buffer[8] = 0xB3;
    rx_fifo_buffer[9] = 0x8B;
    // Flag Byte
    rx_fifo_buffer[10] = GDL90_FLAG_BYTE;


    // Flag Byte
    rx_fifo_buffer[11] = GDL90_FLAG_BYTE;
    // Msg ID - Ownship Report
    rx_fifo_buffer[12] = GDL90_ID_OWNSHIP_REPORT;
    // Data - 27
    rx_fifo_buffer[13] = 0x00;
    rx_fifo_buffer[14] = 0xAB;
    rx_fifo_buffer[15] = 0x45;
    rx_fifo_buffer[16] = 0x49;
    rx_fifo_buffer[17] = 0x1F;
    rx_fifo_buffer[18] = 0xEF;
    rx_fifo_buffer[19] = 0x15;
    rx_fifo_buffer[20] = 0xA8;
    rx_fifo_buffer[21] = 0x89;
    rx_fifo_buffer[22] = 0x78;
    rx_fifo_buffer[23] = 0x0F;
    rx_fifo_buffer[24] = 0x09;
    rx_fifo_buffer[25] = 0xA9;

    // Continue filling the array with the values at indices 26 and onwards
    rx_fifo_buffer[26] = 0x07;
    rx_fifo_buffer[27] = 0xB0;
    rx_fifo_buffer[28] = 0x01;
    rx_fifo_buffer[29] = 0x20;
    rx_fifo_buffer[30] = 0x01;
    rx_fifo_buffer[31] = 0x4E;
    rx_fifo_buffer[32] = 0x38;
    rx_fifo_buffer[33] = 0x32;
    rx_fifo_buffer[34] = 0x35;
    rx_fifo_buffer[35] = 0x56;
    rx_fifo_buffer[36] = 0x20;
    rx_fifo_buffer[37] = 0x20;
    rx_fifo_buffer[38] = 0x20;
    rx_fifo_buffer[39] = 0x00;
    // CRC
    rx_fifo_buffer[40] = 0x85;
    rx_fifo_buffer[41] = 0x5B;
    // Flag Byte
    rx_fifo_buffer[42] = GDL90_FLAG_BYTE;



    // Flag Byte
    rx_fifo_buffer[43] = GDL90_FLAG_BYTE;
    // Msg ID - Ownship Geometric
    rx_fifo_buffer[44] = GDL90_ID_OWNSHIP_GEOMETRIC_ALTITUDE;
    // Data - 4
    rx_fifo_buffer[45] = 0x00;
    rx_fifo_buffer[46] = 0xC8;
    rx_fifo_buffer[47] = 0x7F;
    rx_fifo_buffer[48] = 0xFE;
    // CRC
    rx_fifo_buffer[49] = 0x4B;
    rx_fifo_buffer[50] = 0xD7;
    // Flag Byte
    rx_fifo_buffer[51] = GDL90_FLAG_BYTE;


    // Init
    rxbuf_fifo_len = 53;
    rx.status.prev_data = GDL90_FLAG_BYTE;

    //---------------------------------------------------------------------------
	// Enumerated data type to define the states of parsing buffer and copying message
	//---------------------------------------------------------------------------
    rx.status.state = GDL90_RX_IDLE;

    // FIXME: Read Data out of FIFO when interrupt triggered, store current length of FIFO.
    // if (_max14830) {
    //     rxbuf_fifo_len = _max14830->rx_read(rx_fifo_buffer, MESSAGE_BUFFER_LENGTH);
    // }

    // Initialize pointer to the start of FIFO buffer.
    const uint8_t *byte_ptr = &rx_fifo_buffer[0];
    // Initialize buffer pointer.
    rx_buffer_ptr = rx_buffer;
    // Initialize buffer pointer length index.
    uint8_t buffer_ptr_len = 0;

    // Parse the data in buffer.
    while(true)
    {
        if((rx_buffer + MESSAGE_BUFFER_LENGTH) <= rx_buffer_ptr)
		{
			// If the message won't all fit in the message buffer, it indicates
			//  an error.  Reset and start looking for a new message.
			rx_buffer_ptr = rx_buffer;
			rx.status.state = GDL90_RX_IDLE;
		}

        // Parse all data until the end of the fifo buffer.
        if(rxbuf_fifo_len == buffer_ptr_len) 
        {
            // Finished converting all new data, Clear the interrupt and reset buffer.
            if (_max14830) {
                _max14830->clear_interrupts();
            }
            break;
        }

        // Copy data over to rx buffer.
        *rx_buffer_ptr = *byte_ptr;
        ++rx_buffer_ptr;
        // Track length of buffer pointer.
        ++buffer_ptr_len;

        // Message parsing state machine.
        switch(rx.status.state)
		{

			case(GDL90_RX_IDLE):
			{
                // Carriage Return '\r' (0x0D).
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
                    // if(rx.msg.messageId == GDL90_ID_OWNSHIP_GEOMETRIC_ALTITUDE) {
                    //     GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"CRC: 0x%02X, msgCRC: 0x%02X", crc, rx.msg.crc);
                    // }
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
				// Reset to the start of the message buffer and start looking for a new message.
				rx_buffer_ptr = rx_buffer;
				rx.status.state = GDL90_RX_IDLE;
				break;
			}
		}
        // Store previous byte.
        rx.status.prev_data = *byte_ptr;
        // Increment byte index.
        ++byte_ptr;
    }

    return;
}

/* ************************************************************************* */

void AP_ADSB_Sensor::handle_complete_adsb_msg(const GDL90_RX_MESSAGE &msg)
{
    
    //TODO: TEST WHICH MESSAGES WE ARE RECEIVING!!!!
    //GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"msg_id: %d", msg.messageId);

    switch(msg.messageId)
    {
        case(GDL90_ID_HEARTBEAT):
        {
            // The Heartbeat message provides real-time indications of the status and operation of the
            // transponder. The message will be transmitted with a period of one second for the UCP
            // protocol.
            memcpy(&rx.decoded.heartbeat, msg.raw, sizeof(rx.decoded.heartbeat));
            run_state.last_packet_Transponder_Heartbeat_ms = AP_HAL::millis();

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
        case(GDL90_ID_IDENTIFICATION):
        {
            // The Identification message contains information used to identify the connected device. The
            // Identification message will be transmitted with a period of one second regardless of data status
            // or update for the UCP protocol and will be transmitted upon request for the UCP-HD protocol.
            if (memcmp(&rx.decoded.identification, rx.msg.raw, sizeof(rx.decoded.identification)) != 0) {
                memcpy(&rx.decoded.identification, rx.msg.raw, sizeof(rx.decoded.identification));

                // Firmware Part Number (not null terminated, but null padded if part number is less than 15 characters).
                // Copy into a temporary string that is 1 char longer so we ensure it's null terminated
                const uint8_t str_len = sizeof(rx.decoded.identification.primaryFwPartNumber);
                char primaryFwPartNumber[str_len+1];
                memcpy(&primaryFwPartNumber, rx.decoded.identification.primaryFwPartNumber, str_len);
                primaryFwPartNumber[str_len] = 0;
                
                GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"ADSB:Detected %s v%u.%u.%u SN:%u %s",
                    get_hardware_name(rx.decoded.identification.primary.hwId),
                    (unsigned)rx.decoded.identification.primary.fwMajorVersion,
                    (unsigned)rx.decoded.identification.primary.fwMinorVersion,
                    (unsigned)rx.decoded.identification.primary.fwBuildVersion,
                    (unsigned)rx.decoded.identification.primary.serialNumber,
                    primaryFwPartNumber);
            }
            break;
        }
        case(GDL90_ID_TRANSPONDER_CONFIG):
        {
            memcpy(&rx.decoded.transponder_config, msg.raw, sizeof(rx.decoded.transponder_config));
            break;
        }
        case(GDL90_ID_OWNSHIP_REPORT):
        {
            // The Ownship message contains information on the GNSS position. If the Ownship GNSS
            // position fix is invalid, the Latitude, Longitude, and NIC fields will all have the ZERO value. The
            // Ownship message will be transmitted with a period of one second regardless of data status or
            // update for the UCP protocol. All fields in the ownship message are transmitted MSB first
            memcpy(&rx.decoded.ownship_report, rx.msg.raw, sizeof(rx.decoded.ownship_report));
            run_state.last_packet_Transponder_Ownship_ms = AP_HAL::millis();
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
        case(GDL90_ID_TRANSPONDER_STATUS):
        {
            memcpy(&rx.decoded.transponder_status, msg.raw, sizeof(rx.decoded.transponder_status));
            if (rx.decoded.transponder_status.identActive) {
                tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_IDENT_ACTIVE;
            } else {
               tx_status.state &= ~UAVIONIX_ADSB_OUT_STATUS_STATE_IDENT_ACTIVE;
            }
            
            if (rx.decoded.transponder_status.modeAEnabled) {
                tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_MODE_A_ENABLED;
            } else {
               tx_status.state &= ~UAVIONIX_ADSB_OUT_STATUS_STATE_MODE_A_ENABLED;
            }

            if (rx.decoded.transponder_status.modeCEnabled) {
                tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_MODE_C_ENABLED;
            } else {
                tx_status.state &= ~UAVIONIX_ADSB_OUT_STATUS_STATE_MODE_C_ENABLED;
            }

            if (rx.decoded.transponder_status.modeSEnabled) {
                tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_MODE_S_ENABLED;
            } else {
                tx_status.state &= ~UAVIONIX_ADSB_OUT_STATUS_STATE_MODE_S_ENABLED;
            }

            if (rx.decoded.transponder_status.es1090TxEnabled) {
                tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_1090ES_TX_ENABLED;
            } else {
                tx_status.state &= ~UAVIONIX_ADSB_OUT_STATUS_STATE_1090ES_TX_ENABLED;
            }

            if (rx.decoded.transponder_status.x_bit) {
                tx_status.state |= UAVIONIX_ADSB_OUT_STATUS_STATE_XBIT_ENABLED;
            } else {
                tx_status.state &= ~UAVIONIX_ADSB_OUT_STATUS_STATE_XBIT_ENABLED;
            }

            tx_status.squawk = rx.decoded.transponder_status.squawkCode;

            tx_status.fault &= ~UAVIONIX_ADSB_OUT_STATUS_FAULT_STATUS_MESSAGE_UNAVAIL;

            if (run_state.last_packet_Transponder_Status_ms == 0) {
                // set initial control message contents to transponder defaults
                ctrl.modeAEnabled = rx.decoded.transponder_status.modeAEnabled;
                ctrl.modeCEnabled = rx.decoded.transponder_status.modeCEnabled;
                ctrl.modeSEnabled = rx.decoded.transponder_status.modeSEnabled;
                ctrl.es1090TxEnabled = rx.decoded.transponder_status.es1090TxEnabled;
                ctrl.squawkCode = rx.decoded.transponder_status.squawkCode;
                ctrl.x_bit = rx.decoded.transponder_status.x_bit;
            }
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"state2: 0x%02X", tx_status.fault);
            run_state.last_packet_Transponder_Status_ms = AP_HAL::millis();
            gcs().send_message(MSG_UAVIONIX_ADSB_OUT_STATUS);
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

const char* AP_ADSB_Sensor::get_hardware_name(const uint8_t hwId)
{
    switch(hwId) {
        case 0x09: return "Ping200s";
        case 0x0A: return "Ping20s";
        case 0x18: return "Ping200C";
        case 0x27: return "Ping20Z";
        case 0x2D: return "SkyBeaconX";             // (certified)
        case 0x26: //return "Ping200Z/Ping200X";    // (uncertified). Let's fallthrough and use Ping200X
        case 0x2F: return "Ping200X";               // (certified)
        case 0x30: return "TailBeaconX";            // (certified)
    } // switch hwId
    return "Unknown HW";
}

/* ************************************************************************* */

uint8_t AP_ADSB_Sensor::calc_hex_crc(uint8_t *buf, uint8_t len)
{
    // The checksum is algebraic sum of the message byte values.
    uint8_t sum = 0;

    for(int i=0; i<len; i++) {
        sum += buf[i];
    }

    return sum;
}

/* ************************************************************************* */

bool AP_ADSB_Sensor::request_msg(const GDL90_MESSAGE_ID msg_id)
{
    const GDL90_TRANSPONDER_MESSAGE_REQUEST_V2 msg = {
      messageId : GDL90_ID_MESSAGE_REQUEST,
      version   : 2,
      reqMsgId  : msg_id
    };
    return gdl90Transmit((GDL90_TX_MESSAGE&)msg, sizeof(msg)) != 0;
}

/* ************************************************************************* */

uint16_t AP_ADSB_Sensor::gdl90Transmit(GDL90_TX_MESSAGE &message, const uint16_t length)
{
    uint8_t gdl90FrameBuffer[GDL90_TX_MAX_FRAME_LENGTH] {};

    const uint16_t frameCrc = crc16_ccitt_GDL90((uint8_t*)&message.raw, length, 0);

    // Set flag byte in frame buffer
    gdl90FrameBuffer[0] = GDL90_FLAG_BYTE;
    uint16_t frameIndex = 1;
    
    // Copy and stuff all payload bytes into frame buffer
    for (uint16_t i = 0; i < length+2; i++) {
        // Check for overflow of frame buffer
        if (frameIndex >= GDL90_TX_MAX_FRAME_LENGTH) {
            return 0;
        }
        
        uint8_t data;
        // Append CRC to payload
        if (i == length) {
            data = LOWBYTE(frameCrc);
        } else if (i == length+1) {
            data = HIGHBYTE(frameCrc);
        } else {
            data = message.raw[i];    
        }

        if (data == GDL90_FLAG_BYTE || data == GDL90_CONTROL_ESCAPE_BYTE) {
            // Check for frame buffer overflow on stuffed byte
            if (frameIndex + 2 > GDL90_TX_MAX_FRAME_LENGTH) {
              return 0;
            }
            
            // Set control break and stuff this byte
            gdl90FrameBuffer[frameIndex++] = GDL90_CONTROL_ESCAPE_BYTE;
            gdl90FrameBuffer[frameIndex++] = data ^ GDL90_STUFF_BYTE;
        } else {
            gdl90FrameBuffer[frameIndex++] = data;
        }
    }
    
    // Add end of frame indication
    gdl90FrameBuffer[frameIndex++] = GDL90_FLAG_BYTE;

    // Push packet to UART
    if (hostTransmit(gdl90FrameBuffer, frameIndex)) {
        return frameIndex;
    }
    
    return 0;
}

/* ************************************************************************* */

bool AP_ADSB_Sensor::hostTransmit(uint8_t *buffer, uint16_t length)
{
    if (_max14830 == nullptr) {
      return false;
    }

    //_max14830->tx_write(buffer, length);
    return true;
}

/* ************************************************************************* */