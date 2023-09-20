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
AP_MAX14830::AP_MAX14830()
    : adsb(this), 
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

    // /* Request 2Hz update (500ms) */
    // _driver.register_periodic_callback(500 * AP_USEC_PER_MSEC, FUNCTOR_BIND_MEMBER(&AP_MAX14830::_timer, void));

    /* Request 5Hz update (200ms) */
    _driver.register_periodic_callback(200 * AP_USEC_PER_MSEC, FUNCTOR_BIND_MEMBER(&AP_MAX14830::_timer, void));

    // _drdy_pin = hal.gpio->channel(HAL_GPIO_PIN_DRDY1_BMP388);
    // _drdy_pin->mode(HAL_GPIO_INPUT);
    // _drdy_pin->attach_interrupt(HAL_GPIO_PIN_DRDY1_EXT, trigger_drdy_interrupt, AP_HAL::GPIO::INTERRUPT_RISING);

    return;
}

/* ************************************************************************* */
void AP_MAX14830::_timer(void)
{
    // Poll for global interrupt.
    uint8_t global_isr = _driver.poll_global_isr();

    // ---------------------------------------
    // read any data available on serial port
    // ---------------------------------------

    // Switch on global interrupt per UART/Address basis.
    switch(global_isr)
    {
        case(GLOBALIRQ::IRQ0):
        {
            // Handle UART1 Interrupt - IMET data.
            imet.handle_imet_uart1_interrupt();
            break;
        }
        case(GLOBALIRQ::IRQ1):
        {
            // Handle UART2 Interrupt - ADSB data.
            adsb.handle_adsb_uart2_interrupt();
            break;
        }
        // Future Use
        // case(GLOBALIRQ::IRQ2):
        // {
        //     break;
        // }
        // case(GLOBALIRQ::IRQ3):
        // {
        //     break;
        // }
        default:
        {
            // No interrupt, just break out to return.
            break;
        }
    }


    // SANDBOX
    // static uint8_t counter = 0;
    // counter++;
    // if (counter > 15) {
    //     counter = 0;
    //     adsb.handle_adsb_uart2_interrupt();
    // }
    
    adsb.update();


    return;
}


//---------------------------------------------------------------------------
// Exposed Helper Functions for outside Sensors.
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


/* ************************************************************************* */}


// Singleton Pattern
AP_MAX14830 *AP::MAX14830()
{
    return AP_MAX14830::get_singleton();
}

/* ************************************************************************* */
