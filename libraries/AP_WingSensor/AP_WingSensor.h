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
 * AP_WingSensor.h
 *
 *      Author: Kyle Fruson
 */

#pragma once

#include "AP_WingSensor_config.h"

#if HAL_WINGSENSOR_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#include "AP_WingSensor_Backend.h"
#include "AP_WingSensor_State.h"

class AP_WingSensor {
public:
    friend class AP_WingSensor_Backend;

    // For parameter initialization
    AP_WingSensor();

    // Initializes backend
    void init(void);

    // Requests backend to update the frontend.
    void update();
    
    // Return Angle 1 wing sensor reading in degrees
    float get_angle_1() const { return state.angle_1_deg; }

    // Return Angle 2 wing sensor reading in degrees
    float get_angle_2() const { return state.angle_2_deg; }

    bool is_healthy() const;

    // return timestamp of last update
    uint32_t get_last_update_ms(void) const {
        return state.last_updated_ms;
    }

    // get a copy of state structure
    void get_state(WingSensor_State &state);

    static AP_WingSensor *get_singleton(void) {
        return singleton;
    }

protected:
    // Back end Parameters
    WingSensor_State state;

private:
    // Tracking backends
    AP_WingSensor_Backend *backend;
    static AP_WingSensor *singleton;

    // Semaphore for access to shared frontend data
    HAL_Semaphore sem;

    // write to log
    void log_status();
};

namespace AP {
    AP_WingSensor *WingSensor();
};

#endif // HAL_WINGSENSOR_ENABLED
