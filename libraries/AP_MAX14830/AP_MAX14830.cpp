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

// Singleton Pattern
AP_MAX14830 *AP_MAX14830::_singleton;

// Constructor
AP_MAX14830::AP_MAX14830() :
    adsb(this),
    imet(this)
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
    _driver.init();

    /* Request 5Hz update (200ms) */
    _driver.register_periodic_callback(500 * AP_USEC_PER_MSEC, FUNCTOR_BIND_MEMBER(&AP_MAX14830::_timer, void));

    hal.gpio->pinMode(HAL_GPIO_DRDY1_EXT_IRQ, HAL_GPIO_INPUT);
    //hal.gpio->attach_interrupt(HAL_GPIO_DRDY1_EXT_IRQ, trigger_irq_event , AP_HAL::GPIO::INTERRUPT_FALLING);


    return;
}

/* ************************************************************************* */

void AP_MAX14830::_timer(void)
{
    // Poll MAX14830 for global interrupt.
    uint8_t global_isr = _driver.poll_global_isr();

    // Switch on global interrupt per UART/Address basis.
    // TODO: READ OUT THE INTERRUPT STATUS REGISTER TO DETERMINE WHICH UART TRIGGERED THE INTERRUPT.
    //   CAN BOTH HAPPEN AT SAME TIME?? IE. HANDLE CASE WHERE BOTH UARTS HAVE DATA READY.
    //GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"global_isr: %d", global_isr);

    //uint8_t global_irq = hal.gpio->read(HAL_GPIO_DRDY1_EXT_IRQ);
    //GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"global_isr: %d %d", global_isr, !global_irq);

    switch(global_isr)
    {
        case(GLOBALIRQ::IRQ1):
        {
            // Handle UART1 Interrupt - IMET data.
            imet.handle_imet_uart1_interrupt();
            break;
        }
        case(GLOBALIRQ::IRQ2):
        {
            // Handle UART2 Interrupt - ADSB data.
            adsb.handle_adsb_uart2_interrupt();
            break;
        }
        // Future Use
        case(GLOBALIRQ::IRQ3):
        {
            break;
        }
        case(GLOBALIRQ::IRQ4):
        {
            break;
        }
        default:
        {
            // No interrupt, just break out to return.
            break;
        }
    }
    
    // ---------------------------------------
    //adsb.update();

    return;
}


//---------------------------------------------------------------------------
// Exposed Helper Functions for outside Sensor Usage.
//---------------------------------------------------------------------------

uint8_t AP_MAX14830::rx_read(uint8_t *buf, uint8_t len)
{
    WITH_SEMAPHORE(_sem);
    return _driver.fifo_rx_read(buf, len);
}

/* ************************************************************************* */

void AP_MAX14830::tx_write(uint8_t *buf, uint8_t len)
{
    WITH_SEMAPHORE(_sem);
    _driver.fifo_tx_write(buf, len);
    return;
}

/* ************************************************************************* */

void AP_MAX14830::clear_interrupts()
{
    WITH_SEMAPHORE(_sem);
    _driver.clear_interrupts();
    return;
}

/* ************************************************************************* */

void AP_MAX14830::set_uart_address(UART::value uart_addr)
{
    WITH_SEMAPHORE(_sem);
    _driver.set_uart_address(uart_addr);
    return;
}

/* ************************************************************************* */

void AP_MAX14830::set_RTS_state(bool state)
{
    WITH_SEMAPHORE(_sem);
    _driver.set_RTS_state(state);
    return;
}

/* ************************************************************************* */

// Singleton Pattern
AP_MAX14830 *AP::MAX14830()
{
    return AP_MAX14830::get_singleton();
}

/* ************************************************************************* */
