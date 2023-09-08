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
 * AP_IMET.h
 *
 *      Author: Kyle Fruson
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS_config.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/GPIO.h>
#include "AP_IMET_State.h"
#include "AP_MAX14830.h"
#include <hal.h>



//------------------------------------------------------------------------------
// AP_IMET Class
//------------------------------------------------------------------------------
class AP_IMET {
public:
    AP_IMET(void);
    ~AP_IMET(void){}

    // initialize sensor object
    void init();

    // Handle UART1 Interrupt
    void handle_imet_uart1_interrupt(void);

    //void handle_adsb_uart2_interrupt(void);

    // Handle complete IMET message.
    void handle_complete_imet_msg(const uint8_t len);

    // Handle complete ABSB message.
    //void handle_complete_adsb_msg();

    // Update function for data transmission and saving internal state.
    void update(void);

protected:
    // Structure for Sensor data
    IMET_state state;

    // Data Ready Interrupt
    bool data_ready();

private:
    // SPI object for communication management through MAX14830 Chip.
    AP_MAX14830 _driver;

    // External Interrupt Pin for Data Ready
    AP_HAL::DigitalSource *_drdy_pin;

    // iterruption object for data logging management
    HAL_Semaphore _sem;

    // update the temperature, called at 20Hz
    void _timer(void);
};
