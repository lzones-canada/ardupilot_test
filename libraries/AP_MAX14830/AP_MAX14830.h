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
 * AP_MAX14830.h
 *
 *      Author: Kyle Fruson
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Math/AP_Math.h>
#include <AP_ADSB/AP_ADSB.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS_config.h>
#include <GCS_MAVLink/GCS.h>
#include <utility>
#include <stdio.h>
#include "AP_IMET_Sensor.h"
#include "AP_ADSB_Sensor.h"
#include "AP_VOLZ_Wing.h"
#include "AP_MAX14830_Driver.h"


//------------------------------------------------------------------------------
// AP_MAX14830 Interface Class
//------------------------------------------------------------------------------
class AP_MAX14830 {
public:
    // Constructor
    AP_MAX14830();

    // initialize sensor object
    void init();

    // Expose ability to clear interrupt in Driver.
    void clear_interrupts(void);

    // Expose ability to reset FIFOs in Driver.
    void fifo_reset(void);

    // Expose write function to our attached sensors
    void tx_write(uint8_t *buf, uint8_t len);

    // Expose read function to our attached sensors
    uint8_t rx_read(uint8_t *buf, uint8_t len);

    // Expose ability to set UART address in Driver.
    void set_uart_address(UART::value addr);

    // Expose ability to set RTS pin in Driver.
    void set_RTS_state(bool state);

    // Structure for ADSB data
    AP_ADSB_Sensor adsb;

    // Structure for Sensor data
    AP_IMET_Sensor imet;

    // Structure for Volz data
    AP_VOLZ_Wing volz;

    // get singleton instance
    static AP_MAX14830 *get_singleton(void) {
        return _singleton;
    }

protected:
    // Data Ready Interrupt
    bool data_ready();

private:
    // SPI object for communication management through MAX14830 Chip.
    AP_HAL::OwnPtr<AP_MAX14830_Driver> driver;

    // External Interrupt Pin for Data Ready
    AP_HAL::DigitalSource *_drdy_pin;

    // iterruption object for data logging management
    HAL_Semaphore _sem;

    // update the temperature, called at 20Hz
    void _timer(void);

    static AP_MAX14830 *_singleton;
};

namespace AP {
    AP_MAX14830 *MAX14830();
};