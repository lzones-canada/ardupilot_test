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
 * AP_MAX14830.cpp
 *
 *      Author: Kyle Fruson
 */

#include "AP_MAX14830.h"

extern const AP_HAL::HAL &hal;

/* ************************************************************************* */

AP_MAX14830 *AP_MAX14830::_singleton;       // Singleton Pattern


// Assume these constants represent the individual interrupt flags
#define UART1_INTERRUPT  (1 << 0)
#define UART2_INTERRUPT  (1 << 1)
#define UART3_INTERRUPT  (1 << 2)
#define UART4_INTERRUPT  (1 << 3)

/* ************************************************************************* */

// Constructor
AP_MAX14830::AP_MAX14830() :
    driver(new AP_MAX14830_Driver()), // Initialize _driver with a new instance of AP_MAX14830_Driver
    adsb(this),
    imet(this),
    volz(this)
{
    // Singleton Pattern
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_MAX14830 must be singleton");
    }

    _singleton = this;
}

/* ************************************************************************* */

void AP_MAX14830::init()
{
    // Init UART-SPI Max Driver
    driver->init();

    // Init ADSB
    adsb.init();

    // Init Volz Wing
    volz.init();

    /* Request 50Hz update (20ms) */
    driver->register_periodic_callback(20 * AP_USEC_PER_MSEC, FUNCTOR_BIND_MEMBER(&AP_MAX14830::_timer, void));

    return;
}

// ----------------------------------------------------------------------------
/*
  IRQ handler
 */
void AP_MAX14830::_timer(void)
{
    // Check if the signal is ready.
    if(!driver->get_signal_ready()) {
        // If not, keep attempting to initialize the chip.
        driver->max14830_chip_init();

        // Clear all interrupt sources.
        set_uart_address(UART::ADDR_1);
        clear_interrupts();
        set_uart_address(UART::ADDR_2);
        clear_interrupts();
        set_uart_address(UART::ADDR_3);
        clear_interrupts();
        set_uart_address(UART::ADDR_4);
        clear_interrupts();
        return;
    }

    // Read GlobalIRQ register to determine which UART is source of interrupt.
    uint8_t global_irq = driver->global_interrupt_source();
    //DEV_PRINTF("IRQ: %d\n", global_irq);

    // Handle UART1 Interrupt - HSTM data.
    if(global_irq &  UART1_INTERRUPT) {
        //DEV_PRINTF("RES-INTERUPT1\n");
    }
    
    // Handle UART2 Interrupt - IMET data.
    if(global_irq &  UART2_INTERRUPT) {
        imet.handle_imet_uart2_interrupt();
        //DEV_PRINTF("IMET-INTERUPT2\n");
    }

    // Handle UART3 Interrupt - ADSB data.
    if(global_irq &  UART3_INTERRUPT) {
        adsb.handle_adsb_uart3_interrupt();
        //DEV_PRINTF("ADSB-INTERUPT3\n");
    }

    // Handle UART4 Interrupt - Reserved.
    if(global_irq &  UART4_INTERRUPT) {
        volz.handle_volz_uart1_interrupt();
        //DEV_PRINTF("VOLZ-WING-INTERUPT1\n");
    }

    // Handle Volz Update Loop.
    volz.update();

    // Handle ADSB Update Loop.
    adsb.update();

    return;
}

/* ************************************************************************* */

//---------------------------------------------------------------------------
// Exposed Helper Functions for outside Sensor Usage.
//---------------------------------------------------------------------------

uint8_t AP_MAX14830::rx_read(uint8_t *buf, uint8_t len)
{
    return driver->fifo_rx_read(buf, len);
}

/* ************************************************************************* */

void AP_MAX14830::clear_interrupts()
{
    driver->clear_interrupts();
    return;
}

/* ************************************************************************* */

void AP_MAX14830::set_uart_address(UART::value uart_addr)
{
    driver->set_uart_address(uart_addr);
    return;
}

/* ************************************************************************* */

void AP_MAX14830::tx_write(uint8_t *buf, uint8_t len)
{
    driver->fifo_tx_write(buf, len);
    return;
}

/* ************************************************************************* */

void AP_MAX14830::fifo_reset()
{
    driver->fifo_reset();
    return;
}

/* ************************************************************************* */

void AP_MAX14830::set_RTS_state(bool state)
{
    driver->set_RTS_state(state);
    return;
}

/* ************************************************************************* */

// Singleton Pattern
AP_MAX14830 *AP::MAX14830()
{
    return AP_MAX14830::get_singleton();
}

/* ************************************************************************* */