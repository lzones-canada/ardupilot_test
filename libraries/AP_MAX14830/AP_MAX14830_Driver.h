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
 * AP_MAX14830_Driver.h
 *
 *      Author: Kyle Fruson
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/AP_HAL_Namespace.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/GPIO.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>
#include <utility>
#include <stdio.h>


/*=========================================================================*/
// MAX14830 Enums
/*=========================================================================*/
// UART Address for SPI Command Byte
struct UART
{
    enum value
    {
        ADDR_1  = (0x00 << 5),  // 0x00
        ADDR_2  = (0x01 << 5),  // 0x20
        ADDR_3  = (0x02 << 5),  // 0x40
        ADDR_4  = (0x03 << 5)   // 0x60
    };
};
// Baudrate
struct BAUD
{
    enum value
    {
        RATE_300      = 300,        //
        RATE_450      = 450,        // 
        RATE_600      = 600,        // 
        RATE_900      = 900,        // 
        RATE_1200     = 1200,       // 
        RATE_1800     = 1800,       // 
        RATE_2400     = 2400,       //
        RATE_3600     = 3600,       //
        RATE_4800     = 4800,       //
        RATE_7200     = 7200,       //
        RATE_9600     = 9600,       // *#
        RATE_14400    = 14400,      //
        RATE_19200    = 19200,      // *#
        RATE_28800    = 28800,      //
        RATE_38400    = 38400,      // *#
        RATE_57600    = 57600,      // *#
        RATE_76800    = 76800,      //
        RATE_115200   = 115200,     // *#
        RATE_230400   = 230400,     // *#
        RATE_460800   = 460800,     // *#
        RATE_921600   = 921600,     // *#
        RATE_4000000  = 4000000,    //  #
        RATE_10000000 = 10000000,   //  #
        RATE_13500000 = 13500000,   // *#
        RATE_27000000 = 27000000    // * 
    };
};
// Fifo Trigger Level
struct FIFO_TRIG
{
    enum value
    {
        LEVEL_1  = 8,
        LEVEL_2  = 16,
        LEVEL_3  = 24,
        LEVEL_4  = 32,
        LEVEL_5  = 40,
        LEVEL_6  = 48,
        LEVEL_7  = 56,
        LEVEL_8  = 64,
        LEVEL_9  = 72,
        LEVEL_10 = 80,
        LEVEL_11 = 88,
        LEVEL_12 = 96,
        LEVEL_13 = 104,
        LEVEL_14 = 112,
        LEVEL_15 = 120 
    };
};
// Word Length
struct WORDLEN
{
    enum value
    {
        WORD_5,
        WORD_6,
        WORD_7,
        WORD_8,
        WORD_MAX,
    };
};


// Global IRQ Register
struct GLOBALIRQ
{
	enum
	{
		BLANK4 = (0x01 << 7),
		BLANK3 = (0x01 << 6),
		BLANK2 = (0x01 << 5),
		BLANK1 = (0x01 << 4),
		IRQ4   = (0x01 << 3),
		IRQ3   = (0x01 << 2),
		IRQ2   = (0x01 << 1),
		IRQ1   = (0x01 << 0)
	};
};

//------------------------------------------------------------------------------
// Enumerated data type for controlling the baud rate of the MAX14830 UARTS.
//  The bit rate is a combination of 2 bytes:
//	 Baud-Rate Generator LSB Divisor Register
//	 Baud-Rate Generator MSB Divisor Register
//
// These values are calculated from a spreadsheet obtained from:
//   https://www.analog.com/en/app-notes/programming-baud-rates-of-the-max3108-uart.html
//------------------------------------------------------------------------------

enum MAX14830_bit_rate
{
	bps_230400	= 0x0100,
	bps_115200	= 0x0200,
	bps_57600	= 0x0400,
	bps_28800	= 0x0800,
	bps_14400	= 0x1000,
	bps_7200	= 0x2000,
	bps_3600	= 0x4000,
	bps_1800	= 0x8000,
	bps_900		= 0x0001,
	bps_450		= 0x0002,
	bps_76800	= 0x0300,
	bps_38400	= 0x0600,
	bps_19200	= 0x0C00,
	bps_9600	= 0x1800,
	bps_4800	= 0x3000,
	bps_2400	= 0x6000,
	bps_1200	= 0xC000,
	bps_600		= 0x8001,
	bps_300		= 0x0003
};

/*=========================================================================*/



class AP_MAX14830_Driver {
public:
    AP_MAX14830_Driver(void);
    ~AP_MAX14830_Driver(void){}

    // initialize sensor object
    bool init(void);

    // Break out Chip Initialization for reattempts
    bool max14830_chip_init(void);

    // Sets UART address as MAX14830 has 4 UART Channels to write too.
    void set_uart_address(UART::value uart_addr);

    // Middle layer read function for Max Rx Buffer Read
    uint8_t fifo_rx_read(uint8_t *rxdata, uint8_t len);

	// Middle layer write function for Max Tx Buffer Write
    uint8_t fifo_tx_write(uint8_t *txdata, uint8_t len);

    // Global IRQ to determine UART Channel Interrupt Source
    uint8_t global_interrupt_source();

	// Clear Interrupts for Max chip by reading the Interrupt Status Register
    void clear_interrupts(void);

    // Set RTS pin state needed for Diagnostic Port on Modem.
    void set_RTS_state(bool set_bit);

	// Clear FIFOs, both TX and RX cleared/flushed
    void fifo_reset(void);

    // Getter for signal_ready flag
    bool get_signal_ready(void) { return signal_ready;}

    // Register Periodic Callback for higher level functions to use
    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb);

protected:
    uint8_t _uart_address = 0;

private:
    // SPI object for communication management
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    // Perform Software Reset
    void _max_soft_reset(void);

    // Middle layer write function for Max chip Register Write
    bool _write_register(uint8_t reg, uint8_t data);

    // Middle layer read function for Max chip Register Read
    int _read_register(uint8_t reg);

    // Configure rx fifo interrupt (0=disable 1=enable)
    void _set_rx_interrupt(bool enable);

    // Configure rx timeout interrupt (0=disable 1=enable)
    void _set_rx_timeout_interrupt(bool enable);

    // Configure rx byte timeout register (default to 2 byte timeout)
    void _set_rx_byte_timeout(bool enable);

    // Set Hardware IRQ Interrupt to outside world, Auto Transceiver Direction Control (Auto RTS)
    void _set_irq_trans_ctrl(bool enable, bool auto_enable);

    // Set a delay for RTS pin on the setup and hold time to be safe
    void _set_rts_delay(bool delay);

    // Set baud rate
    void _set_baud(BAUD::value baud);

    // Set line control, parity, stop bits, word length
    void _set_line(bool parity, bool stop);

    // Set Fifo Trigger Level on the RX Fifo
    void _set_fifo_trg_lvl(FIFO_TRIG::value trg_level);

    // Read known software Register signalling the MAX14830 is ready for commands.
    bool _read_ready(void);

    // Flag set when MAX14830 is ready for commands
    bool signal_ready = false;

    enum MAX14830_bit_rate _get_bit_rate_enum(BAUD::value bit_rate);

    /*=========================================================================*/
    // MAX14830R Register map Definitions.
    /*=========================================================================*/

    // IRQ Enable Register
    struct IRQEN
	{
		enum
		{
			CTSIEn      = (0x01 << 7),
			RXEMPTYIEN  = (0x01 << 6),
			TXEMPTYIEN  = (0x01 << 5),
			TFIFOTRGIEN = (0x01 << 4),
			RFIFOTRGIEN = (0x01 << 3),
			STSIEN      = (0x01 << 2),
			SPCLCHRIEN  = (0x01 << 1),
			LSRERRIEN   = (0x01 << 0)
		};
	};

    // Interrupt Status Register
    struct ISR
	{
		enum
		{
			CTSINT       = (0x01 << 7),
			RXEMPTYINT   = (0x01 << 6),
			TXEMPTYINT   = (0x01 << 5),
			TFIFOTRIGINT = (0x01 << 4),
			RFIFOTRIGINT = (0x01 << 3),
			STSINT       = (0x01 << 2),
			SPCHARINT    = (0x01 << 1),
			LSRERRINT    = (0x01 << 0)
		};
	};

    // Line Status Interrupt Enable Register
    struct LSRINTEN
	{
		enum
		{
			NOFUN1      = (0x01 << 7),
			NOFUN0      = (0x01 << 6),
			NOISEINTEN  = (0x01 << 5),
			RBREAKIEN   = (0x01 << 4),
			FRAMEERRIEN = (0x01 << 3),
			PARITYIEN   = (0x01 << 2),
			ROVERRIEN   = (0x01 << 1),
			RTIMOUTIEN  = (0x01 << 0)
		};
	};

    // MODE1
    struct MODE1
	{
		enum
		{
			IRQSEL      = (0x01 << 7),
			AUTOSLEEP   = (0x01 << 6),
			FORCEDSLEEP = (0x01 << 5),
			TRNSCVCTRL  = (0x01 << 4),
			RTSHIZ      = (0x01 << 3),
			TXHIZ       = (0x01 << 2),
			TXDISABLE   = (0x01 << 1),
			RXDISABLE   = (0x01 << 0)
		};
	};

    // MODE2
    struct MODE2
	{
		enum
		{
			ECHOSUPRS  = (0x01 << 7),
			MULTIDROP  = (0x01 << 6),
			LOOPBACK   = (0x01 << 5),
			SPECIALCHR = (0x01 << 4),
			RXEMPTYINV = (0x01 << 3),
			RXTRIGINV  = (0x01 << 2),
			FIFORST    = (0x01 << 1),
			RST        = (0x01 << 0)
		};
	};

    // Line Control Register
    struct LCR
	{
		enum
		{
			RTSBIT      = (0x01 << 7),
			TXBREAK     = (0x01 << 6),
			FORCEPARITY = (0x01 << 5),
			EVENPARITY  = (0x01 << 4),
			PARITYEN    = (0x01 << 3),
			STOPBITS    = (0x01 << 2),
			LENGTHMSB   = (0x01 << 1),
			LENGTHLSB   = (0x01 << 0)
		};
	};

    // Receiver Timeout Register
    struct RXTIMEOUT
	{
		enum
		{
			TIMOUT7 = (0x01 << 7),
			TIMOUT6 = (0x01 << 6),
			TIMOUT5 = (0x01 << 5),
			TIMOUT4 = (0x01 << 4),
			TIMOUT3 = (0x01 << 3),
			TIMOUT2 = (0x01 << 2),
			TIMOUT1 = (0x01 << 1),
			TIMOUT0 = (0x01 << 0)
		};
	};

     // FIFO Interrupt Trigger Level Register
    struct FIFOTRGLVL
	{
		enum
		{
			RXTRIG3 = (0x01 << 7),
			RXTRIG2 = (0x01 << 6),
			RXTRIG1 = (0x01 << 5),
			RXTRIG0 = (0x01 << 4),
			TXTRIG3 = (0x01 << 3),
			TXTRIG2 = (0x01 << 2),
			TXTRIG1 = (0x01 << 1),
			TXTRIG0 = (0x01 << 0)
		};
	};

    struct HDPLXDELAY
	{
		enum
		{
			SETUP3 = (0x01 << 7),
			SETUP2 = (0x01 << 6),
			SETUP1 = (0x01 << 5),
			SETUP0 = (0x01 << 4),
			HOLD3  = (0x01 << 3),
			HOLD2  = (0x01 << 2),
			HOLD1  = (0x01 << 1),
			HOLD0  = (0x01 << 0)
		};
	};

    // Clock Source Register
    struct CLKSRC
	{
		enum
		{
			CLKTORTS  = (0x01 << 7),
			NOUSE3    = (0x01 << 6),
			NOUSE2    = (0x01 << 5),
			CLOCKEN   = (0x01 << 4),
			PLLBYPASS = (0x01 << 3),
			PLLEN     = (0x01 << 2),
			CRYSTALEN = (0x01 << 1),
			NOUSE1    = (0x01 << 0)
		};
	};

    /*=========================================================================*/
};