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
thread_t *AP_MAX14830::_irq_handler_ctx;


#define TIMEOUT_PRIORITY 250      // Right above timer thread
#define EVT_TIMEOUT EVENT_MASK(0) // Event in the irq handler thread triggered by a timeout interrupt
#define EVT_IRQ     EVENT_MASK(1) // Event in the irq handler thread triggered by a radio IRQ (Tx finished, Rx finished, MaxRetries limit)

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
    if (_irq_handler_ctx != nullptr) {
        AP_HAL::panic("AP_MAX14830: double instantiation of irq_handler\n");
    }

    _irq_handler_ctx = chThdCreateFromHeap(NULL,
                                           THD_WORKING_AREA_SIZE(2048),
                                           "thread_max_14830",
                                           TIMEOUT_PRIORITY,          /* Initial priority.    */
                                           irq_handler_thd,           /* Thread function.     */
                                           NULL);                     /* Thread parameter.    */

    /* Request 5Hz update (200ms) */
    //_driver.register_periodic_callback(200 * AP_USEC_PER_MSEC, FUNCTOR_BIND_MEMBER(&AP_MAX14830::_timer, void));
    
    // Register ADSB update function with scheduler
    hal.scheduler->register_timer_process(FUNCTOR_BIND(&adsb, &AP_ADSB_Sensor::update, void));

    // Interrupt pin Setup for MAX14830
    hal.gpio->pinMode(HAL_GPIO_DRDY1_EXT_IRQ, HAL_GPIO_INPUT);
    hal.gpio->attach_interrupt(HAL_GPIO_DRDY1_EXT_IRQ, trigger_irq_event , AP_HAL::GPIO::INTERRUPT_FALLING);

    // Init UART-SPI Max Driver
    _driver.init();

    return;
}

// ----------------------------------------------------------------------------
/*
    Trigger IRQ event
*/
void AP_MAX14830::trigger_irq_event()
{
    //we are called from ISR context
    chSysLockFromISR();
    if (_irq_handler_ctx) {
        chEvtSignalI(_irq_handler_ctx, EVT_IRQ);
    }
    chSysUnlockFromISR();
}


// ----------------------------------------------------------------------------
/*
  IRQ handler thread
 */
void AP_MAX14830::irq_handler_thd(void *arg)
{
    _irq_handler_ctx = chThdGetSelfX();

    (void) arg;
    while (true) {
        eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
        switch (evt) {
        case EVT_IRQ:
            _singleton->irq_handler();
            break;
        // case EVT_TIMEOUT:
        //     radio_singleton->irq_timeout(isr_timeout_time_us);
        //     break;
        default:
            break;
        }
    }

    return;
}

// ----------------------------------------------------------------------------
/*
  IRQ handler
 */
void AP_MAX14830::irq_handler(void)
{
    // Lock the bus to prevent other threads from accessing the MAX14830.
    _driver.lock_bus();

    // Poll MAX14830 for global interrupt.
    uint8_t global_isr = _driver.poll_global_isr();

    // Switch on global interrupt per UART/Address basis.
    // TODO: READ OUT THE INTERRUPT STATUS REGISTER TO DETERMINE WHICH UART TRIGGERED THE INTERRUPT.
    //   CAN BOTH HAPPEN AT SAME TIME?? IE. HANDLE CASE WHERE BOTH UARTS HAVE DATA READY.

    switch(global_isr)
    {
        case(GLOBALIRQ::IRQ1):
        {
            // Handle UART1 Interrupt - IMET data.
            //imet.handle_imet_uart1_interrupt();
            adsb.handle_adsb_uart2_interrupt();
            break;
        }
        case(GLOBALIRQ::IRQ2):
        {
            // Handle UART2 Interrupt - ADSB data.
            adsb.handle_adsb_uart2_interrupt();
            break;
        }
        // Future Use
        // case(GLOBALIRQ::IRQ3):
        // {
        //     break;
        // }
        // case(GLOBALIRQ::IRQ4):
        // {
        //     break;
        // }
        default:
        {
            // No interrupt, just break out to return.
            break;
        }
    }


    // Unlock the bus to allow other threads to access the MAX14830.
    _driver.unlock_bus();

    return;
}

/* ************************************************************************* */

//---------------------------------------------------------------------------
// Exposed Helper Functions for outside Sensor Usage.
//---------------------------------------------------------------------------

uint8_t AP_MAX14830::rx_read(uint8_t *buf, uint8_t len)
{
    return _driver.fifo_rx_read(buf, len);
}

/* ************************************************************************* */

void AP_MAX14830::clear_interrupts()
{
    _driver.clear_interrupts();
    return;
}

/* ************************************************************************* */

void AP_MAX14830::set_uart_address(UART::value uart_addr)
{
    _driver.set_uart_address(uart_addr);
    return;
}

/* ************************************************************************* */

void AP_MAX14830::tx_write(uint8_t *buf, uint8_t len)
{
    _driver.lock_bus();
    _driver.fifo_tx_write(buf, len);
    _driver.unlock_bus();
    return;
}

/* ************************************************************************* */

void AP_MAX14830::set_RTS_state(bool state)
{
    _driver.lock_bus();
    _driver.set_RTS_state(state);
    _driver.unlock_bus();
    return;
}

/* ************************************************************************* */

// Singleton Pattern
AP_MAX14830 *AP::MAX14830()
{
    return AP_MAX14830::get_singleton();
}

/* ************************************************************************* */
