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


extern const AP_HAL::HAL& hal;


/*=========================================================================*/
// Some static member constants for the comms with the MAX14830 chip
/*=========================================================================*/

static const uint32_t RS232_EXT_CLK =                3686400;
//static const uint32_t RS232_EXT_CLK =                8000000;
static const uint32_t RS232_PLL_PREDIV =             1;
static const uint32_t RS232_PLL_CLK = (RS232_EXT_CLK / RS232_PLL_PREDIV);

//static const uint32_t RS232_PLL_CLK = 6 * (RS232_EXT_CLK / RS232_PLL_PREDIV);
//static const uint32_t RS232_PLL_CLK = 6*(25000000/42);



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
#define MAX14830R_TXSYNCH        (0x20)
#define MAX14830R_SYNCHDELAY1    (0x21)
#define MAX14830R_SYNCHDELAY2    (0x22)
#define MAX14830R_TIMER1         (0x23)
#define MAX14830R_TIMER2         (0x24)
#define MAX14830R_REVID          (0x25)

#define MAX14830_WRITE_FLAG      (0x80)
#define MAX14830_READ_FLAG       (0x7F)
/*=========================================================================*/

extern const AP_HAL::HAL &hal;

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
    _dev = std::move(hal.spi->get_device(name));
    if (!_dev) {
        return false;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    for (unsigned i = 0; i < 5; i++) {

        // write software reset
        _max_soft_reset();
        // delay to allow for reset
        hal.scheduler->delay(2);

        // Waiting for board reset.. read known Register RevID..
        uint8_t rev_id = 0;
        rev_id = _read_register(0x1F);
        hal.scheduler->delay(1);

        if (rev_id != 0xA1) {
            continue;
        }

        // Setup UART 1 - IMET ----------------------------------------------------

        // Read Interrupt Status Register to clear interrupts.
        _read_register(MAX14830R_ISR);

        // Set baud rate
        _set_baud(BAUD::RATE_57600);
        // delay to allow for reset
        hal.scheduler->delay(1);

        // Enable Rx Interrupt
        _set_rx_interrupt(true);
        // delay to allow for reset
        hal.scheduler->delay(1);
        
        // Set Rx Timeout Enable Register
        _set_rx_timeout_interrupt(true);
        // delay to allow for reset
        hal.scheduler->delay(1);

        // No parity, StopBit, 8 Data Bits
        _set_line(false, false);
        // delay to allow for reset
        hal.scheduler->delay(1);

        // Rx Timeout (default 2 byte timeout)
        _set_rx_byte_timeout(true);
        // delay to allow for reset
        hal.scheduler->delay(1);

        // Set FIFO Interrupt Trigger Level at 3/4 full?
        // Actual FIFO trigger level is 8 times RxTrig[7:4], hence, selectable threshold granularity is eight.
        _set_fifo_trg_lvl(FIFO_TRIG::LEVEL_12);
        // delay to allow for reset
        hal.scheduler->delay(1);



        // Setup UART 2 - ADSB ----------------------------------------------------



        // Read known Register (RevID) - break out once we get known value
        rev_id = _read_register(0x1F);
        hal.scheduler->delay(1);
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"RevID: 0x%02X", rev_id);
        // as per datasheet: it is recommended to only check for most significant 4 bits: Ah.
        rev_id >>= 4;
        if (rev_id == 0xA) {
            break;
        }
    }

    // Setup UART 3 - Future --------------------------------------------------
    
    // Setup UART 4 - Future --------------------------------------------------

    /*=======================================================================*/
    

    // Enable Auto Transceiver Control ??
    //_set_autotrans(true);
    // delay to allow for reset
    //hal.scheduler->delay(1);

    // fifo reset always ?? Clearing all above settings??
    //_fifo_reset();
    // delay to allow for reset
    //hal.scheduler->delay(1);

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    return true;
}

/* ************************************************************************* */

// Set UART address for UART Selection
void AP_MAX14830_Driver::set_uart_address(uint8_t addr)
{
    _uart_address = addr;
    return;
}

/* ************************************************************************* */

// Software reset of the MAX Chip.
void AP_MAX14830_Driver::_max_soft_reset()
{
    _write_register(MAX14830R_MODE2, 0x01);
    _write_register(MAX14830R_MODE2, 0x00);
    return;
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
    // delay to allow for reset
    hal.scheduler->delay(1);
    // FIFORst bit must then be set back to 0 to continue normal operation.
    mode2_state &= ~(MODE2::FIFORST);
    _write_register(MAX14830R_MODE2, mode2_state);

    return;
}

/* ************************************************************************* */

// Register location to write data
bool AP_MAX14830_Driver::_write_register(uint8_t reg, uint8_t data)
{
    // Write transaction is indicated by MSbit of the MAX3108 register address byte = 1
    reg |= MAX14830_WRITE_FLAG; // | uart_address;

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
uint8_t AP_MAX14830_Driver::fifo_tx_write(uint8_t *txdata, uint8_t len, uint8_t uart_addr)
{
    // Init Tx Buffer for SPI Write.
    uint8_t txbuf[len+1];
    memset(txbuf, 0x00, len+1);
    
    // Write transaction is indicated by MSbit of the MAX3108 register address byte = 1
    txbuf[0] = MAX14830R_THR | MAX14830_WRITE_FLAG | uart_addr;
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
    uint8_t rxbuf[2];
    memset(rxbuf, 0x00, 2);

    // Read transaction is indicated by MSbit of the MAX3108 register address byte = 0
    reg &= MAX14830_READ_FLAG;

    // Read from SPI Device.
    if (!_dev->read_registers(reg, rxbuf, sizeof(rxbuf))) {
        return false;
    }

    return rxbuf[0];
}

/* ************************************************************************* */

// Software read for MAX Chip (Data in Receiver Hold Register).
uint8_t AP_MAX14830_Driver::fifo_rx_read(uint8_t *rxdata, uint8_t len, uint8_t uart_addr)
{
    // Length of RX FIFO Chars to read
    uint8_t fifoLen = _read_register(MAX14830R_RXFIFOLVL);
    // Receiver Hold Register
    uint8_t recv_hold_reg = (MAX14830R_RHR & MAX14830_READ_FLAG) | uart_addr;

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
    //uint8_t mode2_state;
    irqen_state = _read_register(MAX14830R_IRQEN);
    //mode2_state = _read_register(MAX14830R_MODE2);

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
void AP_MAX14830_Driver::_set_autotrans(bool enable)
{
    // Peserve State of MODE1 Register
    uint8_t mode1_state;
    mode1_state = _read_register(MAX14830R_MODE1);
    // Enable Auto Tx/Rx Control
    if(enable) {
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

// Configure baud rate for the UART line
void AP_MAX14830_Driver::_set_baud(BAUD::value baud)
{
    // Peserve State of Clock Source Register
    //uint8_t clksrc_state;
    //clksrc_state = _read_register(MAX14830R_CLKSOURCE);

    // Disable Clock Source
    //clksrc_state &= ~(CLKSRC::CLOCKEN);
    //_write_register(MAX14830R_CLKSOURCE, ~(CLKSRC::CLOCKEN));


    // Calculate Divisor for Baud Rate
    float x = 0.0;
    float d = 0.0;
    float f = 0.0;
    uint8_t div = 0;
    uint8_t fract = 0;
    
    x = (float)RS232_PLL_CLK / (float)(16 * baud);
    f = modff(x, &d);
    div = d;
    fract = roundf(16*f);

    // Set PLLConfig
    _write_register(MAX14830R_PLLCONFIG, RS232_PLL_PREDIV); //0x01

    // Set BRGConfig
    _write_register(MAX14830R_BRGCONFIG, fract); //0x00

    // Set DIV LSB
    _write_register(MAX14830R_DIVLSB, div); //0x04
    
    // Set DIV MSB
    _write_register(MAX14830R_DIVMSB, 0x00); //0x00
    
    // Re-enable CLK Source - !!!!!WATCH OUT HERE ON CHIP CHANGE!!!!
    _write_register(MAX14830R_CLKSOURCE, 0x18); //0x18

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

/* ************************************************************************* */

// exposed polling function for ISR interrupt.
uint8_t AP_MAX14830_Driver::poll_global_isr()
{
    // Retrieve and store state from Global IRQ Register
    uint8_t isr_state;
    isr_state = _read_register(MAX14830R_ISR);

    //GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"isr_state: 0x%02X", isr_state);

    // Interrupt trigger on Rx Fifo Fill OR Line Status Error (Rx Timeout)
    if((isr_state & ISR::RFIFOTRIGINT) || (isr_state & ISR::LSRERRINT)) {
        return 1;
    }
    else {
        return 0;
    }


    // FIXME: See below!!
    // uint8_t global_irq;
    // global_irq = _read_register(MAX14830R_GLOBALIRQ);

    // return global_irq;
}

/* ************************************************************************* */

// Register periodic callback for device
AP_HAL::Device::PeriodicHandle AP_MAX14830_Driver::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return _dev->register_periodic_callback(period_usec, cb);
}

/* ************************************************************************* */
