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
 * AP_WingSensor_Backend.cpp
 *
 *      Author: Kyle Fruson
 */

#include "AP_WingSensor.h"

#if HAL_WINGSENSOR_ENABLED

#include "AP_WingSensor_Backend.h"

extern const AP_HAL::HAL &hal;

AP_WingSensor_Backend::AP_WingSensor_Backend(AP_WingSensor &_frontend) :
    frontend(_frontend)
{
}

void AP_WingSensor_Backend::copy_to_frontend() 
{
    WITH_SEMAPHORE(frontend.sem);
    frontend.state = internal_state;
}

HAL_Semaphore &AP_WingSensor_Backend::get_sem(void)
{
    return frontend.sem;
}
#endif // HAL_WINGSENSOR_ENABLED
