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
 * AP_WingSensor_State.h
 *
 *      Author: Kyle Fruson
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

// Time in milliseconds before we declare the Wing Sensor to be "unhealthy"
#define HEALTHY_LAST_RECEIVED_MS 3000

// Stores the current state read by system
// All backends are required to fill in this state structure
struct WingSensor_State {
    // When this structure was last updated (milliseconds)
    uint32_t last_updated_ms;

    // Angle 1 (degrees)
    float angle_1_deg;

    // Angle 2 (degrees)
    float angle_2_deg;
};
