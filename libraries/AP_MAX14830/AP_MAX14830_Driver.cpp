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
 * AP_MAX14830_Driver.cpp
 *
 *      Author: Kyle Fruson
 */


#include "AP_MAX14830_Driver.h"
/*=========================================================================*/
#define HAL_MAX14830_SPI_NAME "max14830"


/*=========================================================================*/
// MAX14830R Register map defines
/*=========================================================================*/
#define MAX14830R_RHR            (0x00)
#define MAX14830R_THR            (0x00)
#define MAX14830R_IRQEN          (0x01)
#define MAX14830R_ISR            (0x02)
#define MAX14830R_LSRINTEN       (0x03)
#define MAX14830R_LSR            (0x04)
#define MAX14830R_SPCLCHRINTEN   (0x05)
#define MAX14830R_SPCLCHARINT    (0x06)
#define MAX14830R_STSINTEN       (0x07)
#define MAX14830R_STSINT         (0x08)
#define MAX14830R_MODE1          (0x09)
#define MAX14830R_MODE2          (0x0A)
#define MAX14830R_LCR            (0x0B)
#define MAX14830R_RXTIMEOUT      (0x0C)
#define MAX14830R_HDPLXDELAY     (0x0D)
#define MAX14830R_IRDA           (0x0E)
#define MAX14830R_FLOWLVL        (0x0F)
#define MAX14830R_FIFOTRGLVL     (0x10)
#define MAX14830R_TXFIFOLVL      (0x11)
#define MAX14830R_RXFIFOLVL      (0x12)
#define MAX14830R_FLOWCTRL       (0x13)
#define MAX14830R_XON1           (0x14)
#define MAX14830R_XON2           (0x15)
#define MAX14830R_XOFF1          (0x16)
#define MAX14830R_XOFF2          (0x17)
#define MAX14830R_GPIOCONFG      (0x18)
#define MAX14830R_GPIODATA       (0x19)
#define MAX14830R_PLLCONFIG      (0x1A)
#define MAX14830R_BRGCONFIG      (0x1B)
#define MAX14830R_DIVLSB         (0x1C)
#define MAX14830R_DIVMSB         (0x1D)
#define MAX14830R_CLKSOURCE      (0x1E)
#define MAX14830R_GLOBALIRQ      (0x1F)
#define MAX14830R_GLOBALCOMND    (0x1F)

#define MAX14830R_TRGT_DIVLSB    (0x01)
#define MAX14830_WRITE_FLAG      (0x80)
#define MAX14830_READ_FLAG       (0x7F)
/*=========================================================================*/

AP_MAX14830_Driver::AP_MAX14830_Driver() :
    _dev(nullptr)
{
}

/* ************************************************************************* */

// MAX14830 initialization
bool AP_MAX14830_Driver::init()
{
    // Check if already initialized / exists.
    if (_dev) {
        return true;
    }

    // Look for "max chip" device
    _dev = std::move(hal.spi->get_device(HAL_MAX14830_SPI_NAME));
    if (!_dev) {
        return false;
    }

    max14830_chip_init();

    return true;
}

/* ************************************************************************* */

// MAX14830 initialization - Split out for re-attempts.
bool AP_MAX14830_Driver::max14830_chip_init() {

    WITH_SEMAPHORE(_dev->get_semaphore());

    /* Soft-reset the MAX14830
        Return value of 'write_register()' is not checked.
        This commands has the tendency to fail upon soft-reset.
    */
    _max_soft_reset();
    hal.scheduler->delay(10);

    /*=======================================================================*/

    // For loop to allow for startup attempts.
    for (unsigned i = 0; i < 8; i++) 
    {
        // Waiting for board reset.. read known Register DIVLSB.
        if(_read_ready()) {
            signal_ready = true;
        }
        else {
            continue;
        }

        // Setup UART 1 - Reserved (HSTM)------------------------------------------

        // Setup UART 2 - IMET ----------------------------------------------------
        set_uart_address(UART::ADDR_2);

        // Set baud rate
        _set_baud(BAUD::RATE_57600);

        // Enable Rx Interrupt
        _set_rx_interrupt(true);
        
        // Set Rx Timeout Enable Register
        _set_rx_timeout_interrupt(true);

        // No parity, StopBit, 8 Data Bits
        _set_line(false, false);

        // Rx Timeout (default 2 byte timeout)
        _set_rx_byte_timeout(true);
        // Set FIFO Interrupt Trigger Level at 3/4 full?
        // Actual FIFO trigger level is 8 times RxTrig[7:4], hence, selectable threshold granularity is eight.
        _set_fifo_trg_lvl(FIFO_TRIG::LEVEL_12);

        // Set IRQ Interrupt Enable, Auto Transceiver Direction Control Disabled
        _set_irq_trans_ctrl(true, false);

        // Setup UART 3 - ADSB ----------------------------------------------------
        set_uart_address(UART::ADDR_3);

        // Set baud rate
        _set_baud(BAUD::RATE_57600);

        // Enable Rx Interrupt
        _set_rx_interrupt(true);
        
        // Set Rx Timeout Enable Register
        _set_rx_timeout_interrupt(true);

        // No parity, StopBit, 8 Data Bits
        _set_line(false, false);

        // Rx Timeout (default 2 byte timeout)
        _set_rx_byte_timeout(true);

        // Set FIFO Interrupt Trigger Level at 3/4 full?
        // Actual FIFO trigger level is 8 times RxTrig[7:4], hence, selectable threshold granularity is eight.
        _set_fifo_trg_lvl(FIFO_TRIG::LEVEL_12);

        // Set IRQ Interrupt Enable, Auto Transceiver Direction Control Disabled
        _set_irq_trans_ctrl(true, false);

        // Setup UART 4 - Sweep Wing Servo  ---------------------------------------
        set_uart_address(UART::ADDR_4);

        // Set baud rate
        _set_baud(BAUD::RATE_115200);

        // Enable Rx Interrupt
        _set_rx_interrupt(true);
        
        // Set Rx Timeout Enable Register
        _set_rx_timeout_interrupt(true);

        // No parity, StopBit, 8 Data Bits
        // TODO: TWO STOP BTIS?!
        _set_line(false, false);

        // Rx Timeout (default 2 byte timeout)
        _set_rx_byte_timeout(true);

        // Set FIFO Interrupt Trigger Level at 3/4 full?
        // Actual FIFO trigger level is 8 times RxTrig[7:4], hence, selectable threshold granularity is eight.
        _set_fifo_trg_lvl(FIFO_TRIG::LEVEL_12);

        // Set IRQ Interrupt Enable, Auto Transceiver Direction Control Disabled
        _set_irq_trans_ctrl(true, true);
        // Set Half duplex delay register for RTS line buffer
        _set_rts_delay(true);


        // Dev Setup --------------------------------------------------------------
        // FIXME: Enable CLK Source DEV BOARD - **** NOT NEEDED ON CHIP CHANGE ****
        //set_uart_address(UART::ADDR_1);
        //_write_register(MAX14830R_CLKSOURCE, 0x0A);
        // ------------------------------------------------------------------------

        // Read known Register break out once we get known value confirming reset.
        if(signal_ready) {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE,"MaxQuart Clock[0x%02X] Enabled", _read_register(MAX14830R_CLKSOURCE));
            break;
        }
    }
    
    // Setup UART 4 - Future ------------------------------------------------------

    /*=======================================================================*/

    return true;
}

/* ************************************************************************* */

// Set UART address for UART Selection
void AP_MAX14830_Driver::set_uart_address(UART::value uart_addr)
{
    //hal.console->printf("Setting UART Address: 0x%02X\n", uart_addr);
    _uart_address = uart_addr;
    return;
}

/* ************************************************************************* */

// Software reset of the MAX Chip.
void AP_MAX14830_Driver::_max_soft_reset()
{
    _dev->write_register(MAX14830R_MODE2 | UART::ADDR_1 | MAX14830_WRITE_FLAG, 0x01);
    _dev->write_register(MAX14830R_MODE2 | UART::ADDR_2 | MAX14830_WRITE_FLAG, 0x01);
    _dev->write_register(MAX14830R_MODE2 | UART::ADDR_3 | MAX14830_WRITE_FLAG, 0x01);
    _dev->write_register(MAX14830R_MODE2 | UART::ADDR_4 | MAX14830_WRITE_FLAG, 0x01);

    _dev->write_register(MAX14830R_MODE2 | UART::ADDR_1 | MAX14830_WRITE_FLAG, 0x00);
    _dev->write_register(MAX14830R_MODE2 | UART::ADDR_2 | MAX14830_WRITE_FLAG, 0x00);
    _dev->write_register(MAX14830R_MODE2 | UART::ADDR_3 | MAX14830_WRITE_FLAG, 0x00);
    _dev->write_register(MAX14830R_MODE2 | UART::ADDR_4 | MAX14830_WRITE_FLAG, 0x00);

    return;
}

/* ************************************************************************* */

// Check if Chip has completed reset and ready to receive commands.
bool AP_MAX14830_Driver::_read_ready()
{
    //bool ready = false;
    uint8_t v;

    // Read Device ID Register until reset is complete.
    for (unsigned i = 0; i < 10; i++)
    {

        if (!_dev->read_registers(MAX14830R_DIVLSB, &v, 1) || v != MAX14830R_TRGT_DIVLSB) {
            continue;
        }
        else {
            return true;
            // Exit loop as UART is ready.
            break;
        }
    }

    return false;
}
/* ************************************************************************* */

// Clear FIFOs, both TX and RX cleared/flushed
void AP_MAX14830_Driver::fifo_reset()
{
    uint8_t mode2_state;
    // Capture State of MODE2 Register
    mode2_state = _read_register(MAX14830R_MODE2);
    // Set the FIFORst bit high to clear both Rx / Tx FIFOs of all data.
    mode2_state |= MODE2::FIFORST;
    _write_register(MAX14830R_MODE2, mode2_state);
    // FIFORst bit must then be set back to 0 to continue normal operation.
    mode2_state &= ~(MODE2::FIFORST);
    _write_register(MAX14830R_MODE2, mode2_state);

    return;
}

/* ************************************************************************* */

// Register location to write data
bool AP_MAX14830_Driver::_write_register(uint8_t reg, uint8_t data)
{
    // Set address of MAX UART to write to.
    reg |= _uart_address;
    // Write transaction is indicated by MSbit of the MAX3108 register address byte = 1
    reg |= MAX14830_WRITE_FLAG;

    // Write to SPI Device.
    for (uint8_t i=0; i<8; i++) {
        _dev->write_register(reg, data);
        uint8_t result = _read_register(reg);
        if (result == data) {
            return true;
        }
    }

    // Discard the write flag when sending to understand the Register Clearly.
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"BAD_WRITE_REG: 0x%02X", (reg &= MAX14830_READ_FLAG));
    return false;
}

/* ************************************************************************* */

// Software write for MAX Chip (Data stuffed in Transmit Hold Register).
uint8_t AP_MAX14830_Driver::fifo_tx_write(uint8_t *txdata, uint8_t len)
{
    // Init Tx Buffer for SPI Write.
    uint8_t txbuf[len+1];
    memset(txbuf, 0x00, len+1);
    
    // Write transaction is indicated by MSbit of the MAX3108 register address byte = 1
    txbuf[0] = MAX14830R_THR | MAX14830_WRITE_FLAG | _uart_address;
    // Copy rest of data over for transmission.
    memcpy(txbuf+1, txdata, len);

    // Write to SPI Device.
    return _dev->transfer(txbuf, len+1, nullptr, 0);
}

/* ************************************************************************* */

// Byte of data from register
int AP_MAX14830_Driver::_read_register(uint8_t reg)
{
    // Init Rx Buffer for SPI Read.
    int len = 1;
    // Buffer to hold read data (1 byte + dummy byte + register data byte)
    uint8_t rxbuf[len+2];
    // Clear rxbuf buffer
    memset(rxbuf, 0x00, len+2);

    // Set address of MAX UART to read from.
    reg |= _uart_address;
    // Read transaction is indicated by MSbit of the MAX3108 register address byte = 0
    reg &= MAX14830_READ_FLAG;
    // Set address of MAX UART to read from.
    rxbuf[0] = reg;

    // Read from SPI Device.
    if (!_dev->transfer(rxbuf, len+2, rxbuf, len+2)) {
        return false;
    }

    return rxbuf[1]; // Return the first byte of data received
}

/* ************************************************************************* */

// Software read for MAX Chip (Data in Receiver Hold Register).
uint8_t AP_MAX14830_Driver::fifo_rx_read(uint8_t *rxdata, uint8_t len)
{
    // Length of RX FIFO Chars to read
    uint8_t fifoLen = _read_register(MAX14830R_RXFIFOLVL);
    // Receiver Hold Register
    uint8_t recv_hold_reg = MAX14830R_RHR;
    // Set address of MAX UART to read from.
    recv_hold_reg |= _uart_address;
    // Read transaction is indicated by MSbit of the MAX3108 register address byte = 0
    recv_hold_reg &= MAX14830_READ_FLAG;

    // Clear rxdata buffer
    memset(rxdata, 0x00, len+1);

    // Read from SPI Device
    _dev->read_registers(recv_hold_reg, rxdata, fifoLen);

    return fifoLen;
}

/* ************************************************************************* */

// Configure rx fifo interrupt (0=disable 1=enable)
void AP_MAX14830_Driver::_set_rx_interrupt(bool enable) 
{
    // Peserve State of IRQEN Register
    uint8_t irqen_state;
    irqen_state = _read_register(MAX14830R_IRQEN);

    // Enable RX Interrupt
    if(enable) {
        // Receive Fifo Interrupt
        irqen_state |= IRQEN::RFIFOTRGIEN;
        // Line Status Interrupt (used for parity errors and RX timeout)
        irqen_state |= IRQEN::LSRERRIEN;

    }
    // Disable RX Interrupt
    else {
        // Receive Fifo Interrupt
        irqen_state &= ~(IRQEN::RFIFOTRGIEN);
        // Line Status Interrupt (used for parity errors and RX timeout)
        irqen_state &= ~(IRQEN::LSRERRIEN);
    }
    // Write back to IRQEN Register
    _write_register(MAX14830R_IRQEN, irqen_state);

    return;
}

/* ************************************************************************* */

// Configure the Rx timeout line status interrupt register (0=disable 1=enable)
void AP_MAX14830_Driver::_set_rx_timeout_interrupt(bool enable) 
{
    // Peserve State of LSR Register
    uint8_t lsr_state;
    lsr_state = _read_register(MAX14830R_LSRINTEN);

    // Enable RX Timeout Interrupt
    if(enable) {
        lsr_state |= LSRINTEN::RTIMOUTIEN;

    }
    // Disable RX Timeout Interrupt
    else {
        lsr_state &= ~(LSRINTEN::RTIMOUTIEN);
    }

    // Write status back to LSR Register
    _write_register(MAX14830R_LSRINTEN, lsr_state);

    return;
}

/* ************************************************************************* */

// Configure rx byte timeout value for interrupt(default to 2 byte timeout)
void AP_MAX14830_Driver::_set_rx_byte_timeout(bool enable)
{
    // Peserve State of Rx Timeout Register
    uint8_t rxto_state;
    rxto_state = _read_register(MAX14830R_RXTIMEOUT);
    // Enable Rx Timeout interrupt
    if(enable) {
        rxto_state |= RXTIMEOUT::TIMOUT1;
    }
    // Disable Rx Timeout interrupt
    else {
        rxto_state &= ~(RXTIMEOUT::TIMOUT1);
    }
    // Write back to Rx Timeout Register
    _write_register(MAX14830R_RXTIMEOUT, rxto_state);

    return;
}

/* ************************************************************************* */

// Clear Interrupts - Read ISR Register
void AP_MAX14830_Driver::clear_interrupts() 
{
    // Read Interrupt Status Register to clear interrupts
    _read_register(MAX14830R_ISR); //COR
    return;
}

/* ************************************************************************* */

// Configure auto tx/rx control (0=disable 1=enable)
void AP_MAX14830_Driver::_set_irq_trans_ctrl(bool irq_enable, bool auto_enable)
{
    // Peserve State of MODE1 Register
    uint8_t mode1_state;
    mode1_state = _read_register(MAX14830R_MODE1);

    if(irq_enable) {
        mode1_state |= MODE1::IRQSEL;
    }
    // Disable Auto Tx/Rx Control 
    else {
        mode1_state &= ~(MODE1::IRQSEL);
    }

    // Enable Auto Tx/Rx Control
    if(auto_enable) {
        mode1_state |= MODE1::TRNSCVCTRL;
    }
    // Disable Auto Tx/Rx Control 
    else {
        mode1_state &= ~(MODE1::TRNSCVCTRL);
    }
    
    // Write back to MODE1 Register
    _write_register(MAX14830R_MODE1, mode1_state);

    return;
}

/* ************************************************************************* */

// Configure Setup and Hold time for the auto RTS pin Control
void AP_MAX14830_Driver::_set_rts_delay(bool delay)
{
    // Peserve State of HDplxDelay Register
    uint8_t delay_state;
    delay_state = _read_register(MAX14830R_HDPLXDELAY);

    // Enable
    if(delay) {
        delay_state |= HDPLXDELAY::HOLD0;
        delay_state |= HDPLXDELAY::HOLD1;
        delay_state |= HDPLXDELAY::SETUP0;
        delay_state |= HDPLXDELAY::SETUP1;
    }
    // Disable Auto Tx/Rx Control 
    else {
        delay_state &= ~(HDPLXDELAY::HOLD0);
        delay_state &= ~(HDPLXDELAY::HOLD1);
        delay_state &= ~(HDPLXDELAY::SETUP0);
        delay_state &= ~(HDPLXDELAY::SETUP1);
    }

    // Write back to MODE1 Register
    _write_register(MAX14830R_HDPLXDELAY, delay_state);

    return;
}

/* ************************************************************************* */

// Configure baud rate for the UART line
void AP_MAX14830_Driver::_set_baud(BAUD::value baud)
{
    uint16_t baud_div = _get_bit_rate_enum(baud);
    
    // Extract baud_div into MSB and LSB
    uint8_t div_lsb = baud_div >> 8;
    uint8_t div_msb = baud_div;

    // Sets DIV LSB
    _write_register(MAX14830R_DIVLSB, div_lsb);

    // Set DIV MSB
    _write_register(MAX14830R_DIVMSB, div_msb);

    return;
}

/* ************************************************************************* */

// Parity bit (0=disable, 1=enable), 2 StopBits (0=disable, 1=enable), Word length (5, 6, 7, 8)
void AP_MAX14830_Driver::_set_line(bool parity, bool stop)
{
    // Peserve State of LCR Register
    uint8_t lcr_state;
    lcr_state = _read_register(MAX14830R_LCR);

    //Set parity
    if(parity) {
        lcr_state |= LCR::PARITYEN;
    }
    // Disable parity
    else {
        lcr_state &= ~(LCR::PARITYEN);
    }
    // Set 2 StopBits
    if(stop) {
        lcr_state |= LCR::STOPBITS;
    }
    // Disable 2 StopBits
    else {
        lcr_state &= ~(LCR::STOPBITS);
    }

    // Set word length to 8 bits
    lcr_state |= WORDLEN::WORD_8;

    // Write back to LCR Register
    _write_register(MAX14830R_LCR, lcr_state);

    return;
}

/*******************************************************************************
* This function sets the discrete bit used for the RTS line.
*  TODO: Set the UART ADDRESS FOR MODEM PRIOR!!! _max14830->set_uart_address(UART::ADDR_3);
*  Will need to look at locking the bus?
*******************************************************************************/
void AP_MAX14830_Driver::set_RTS_state(bool set_bit)
{
	// Peserve State of LCR Register
    uint8_t lcr_state;
    lcr_state = _read_register(MAX14830R_LCR);

    // If RTSbit is set to 1, then RTS_ is set to logic-high
    if(set_bit) {
        lcr_state |= LCR::RTSBIT;
    }
    // If RTSbit is set to 0, then RTS_ is set to logic-low
    else {
        lcr_state &= ~(LCR::RTSBIT);
    }

    // Write back to LCR Register
    _write_register(MAX14830R_LCR, lcr_state);

    return;

    //     0C  // Size (Mac Addr - End Payload, no CRC included)
    
    //     00 00 00 00 00 00  // Mac Addr (0 for local)
    
    //     01 23  // Magic Number
    
    //     02  // Control
        
    //     04  // Cmd (Write 4, Read 0, Special Fn 8) 
    
    //     1D  // Pwr ID, 03-> effective after reset, 29 -> Immediate
    //     30  // 0x14 -> 20 dDm, 0x1E -> 30 dBm
    
    //     AB CD // CRC, dummy values if S163=0
}


/* ************************************************************************* */

// Trigger Level to set on the RX Fifo with 8 times granularity.
void AP_MAX14830_Driver::_set_fifo_trg_lvl(FIFO_TRIG::value trg_level)
{
    // Peserve State of Trger Level Register
    uint8_t trglevel_state;
    trglevel_state = _read_register(MAX14830R_FIFOTRGLVL);

    // Granularity 8 Times.
    switch (trg_level) {
        case FIFO_TRIG::LEVEL_15:
            // All bits set, break out and leave as is.
            break;
        case FIFO_TRIG::LEVEL_14:
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG0);
            break;
        case FIFO_TRIG::LEVEL_13:
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG1);
            break;
        case FIFO_TRIG::LEVEL_12:
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG0);
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG1);
            trglevel_state |=  (FIFOTRGLVL::RXTRIG2);
            trglevel_state |=  (FIFOTRGLVL::RXTRIG3);
            break;
        case FIFO_TRIG::LEVEL_11:
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG2);
            break;
        case FIFO_TRIG::LEVEL_10:
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG0);
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG2);
            break;
        case FIFO_TRIG::LEVEL_9:
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG1);
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG2);
            break;
        case FIFO_TRIG::LEVEL_8:
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG0);
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG1);
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG2);
            break;
        case FIFO_TRIG::LEVEL_7:
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG3);
            break;
        case FIFO_TRIG::LEVEL_6:
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG0);
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG3);
            break;
        case FIFO_TRIG::LEVEL_5:
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG1);
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG3);
            break;
        case FIFO_TRIG::LEVEL_4:
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG0);
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG1);
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG3);
            break;
        case FIFO_TRIG::LEVEL_3:
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG2);
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG3);
            break;
        case FIFO_TRIG::LEVEL_2:
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG0);
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG2);
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG3);
            break;
        case FIFO_TRIG::LEVEL_1:
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG1);
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG2);
            trglevel_state &= ~(FIFOTRGLVL::RXTRIG3);
            
            break;
        default:
            trglevel_state = trglevel_state;
            break;
    }

    // Clear TxTrigger Levels as Default..
    trglevel_state &= ~(FIFOTRGLVL::TXTRIG0);
    trglevel_state &= ~(FIFOTRGLVL::TXTRIG1);
    trglevel_state &= ~(FIFOTRGLVL::TXTRIG2);
    trglevel_state &= ~(FIFOTRGLVL::TXTRIG3);

    // Write back to FIFO Interrupt Trigger Level Register
    _write_register(MAX14830R_FIFOTRGLVL, trglevel_state);

    return;
}

/*******************************************************************************
* This function accepts the bit rate requested in the hardware structure and
*	returns the corresponding enum value required to program an SCI to that
*	bit rate.  If the bit rate requested is not supported, a default rate of
*	9600 bps is returned.
*******************************************************************************/

enum MAX14830_bit_rate AP_MAX14830_Driver::_get_bit_rate_enum(BAUD::value bit_rate)
{
	enum MAX14830_bit_rate result;
	switch(bit_rate)
	{
		case(BAUD::RATE_230400):
		{
			result = bps_230400;
			break;
		}
		case(BAUD::RATE_115200):
		{
			result = bps_115200;
			break;
		}
		case(BAUD::RATE_76800):
		{
			result = bps_76800;
			break;
		}
		case(BAUD::RATE_57600):
		{
			result = bps_57600;
			break;
		}
		case(BAUD::RATE_38400):
		{
			result = bps_38400;
			break;
		}
		case(BAUD::RATE_28800):
		{
			result = bps_28800;
			break;
		}
		case(BAUD::RATE_19200):
		{
			result = bps_19200;
			break;
		}
		case(BAUD::RATE_14400):
		{
			result = bps_14400;
			break;
		}
		case(BAUD::RATE_9600):
		{
			result = bps_9600;
			break;
		}
		case(BAUD::RATE_7200):
		{
			result = bps_7200;
			break;
		}
		case(BAUD::RATE_4800):
		{
			result = bps_4800;
			break;
		}
		case(BAUD::RATE_3600):
		{
			result = bps_3600;
			break;
		}
		case(BAUD::RATE_2400):
		{
			result = bps_2400;
			break;
		}
		case(BAUD::RATE_1800):
		{
			result = bps_1800;
			break;
		}
		case(BAUD::RATE_1200):
		{
			result = bps_1200;
			break;
		}
		case(BAUD::RATE_900):
		{
			result = bps_900;
			break;
		}
		case(BAUD::RATE_600):
		{
			result = bps_600;
			break;
		}
		case(BAUD::RATE_450):
		{
			result = bps_450;
			break;
		}
		case(BAUD::RATE_300):
		{
			result = bps_300;
			break;
		}
		default:
		{
			result = bps_9600;
			break;
		}
	}
	return(result);
}

/* ************************************************************************* */

// exposed polling function for ISR interrupt.
uint8_t AP_MAX14830_Driver::global_interrupt_source()
{
    // Retrieve and store state from Global IRQ Register
    uint8_t global_irq = _read_register(MAX14830R_GLOBALIRQ);

    // Flip & Mask to keep only lower 4 bits of interest.
    //  0x0F represents binary 00001111
    uint8_t masked_irq = (~global_irq) & 0x0F;

    return masked_irq;
}

/* ************************************************************************* */

// Register periodic callback for device
AP_HAL::Device::PeriodicHandle AP_MAX14830_Driver::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return _dev->register_periodic_callback(period_usec, cb);
}

/* ************************************************************************* */