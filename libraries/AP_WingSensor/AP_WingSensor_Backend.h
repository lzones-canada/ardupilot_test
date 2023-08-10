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
 * AP_WingSensor_Backend.h
 *
 *      Author: Kyle Fruson
 */

#pragma once

#include "AP_WingSensor.h"
#include "AP_WingSensor_State.h"
#include <AP_HAL/Semaphores.h>

class AP_WingSensor; //forward declaration

class AP_WingSensor_Backend {
public:    
    // Constructor with initialization
    AP_WingSensor_Backend(AP_WingSensor &_frontend);

    // Virtual destructor that wing sensor backend can override 
    virtual ~AP_WingSensor_Backend(void) {}

    // Update the state structure
    virtual void update() = 0;

protected:
    // Copies internal state to the frontend state
    void copy_to_frontend();

    // Internal state for this driver (before copying to frontend)
    WingSensor_State internal_state;

    HAL_Semaphore &get_sem(void);

private:
    AP_WingSensor &frontend;
};
