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
 * AP_WingSensor.cpp
 *
 *      Author: Kyle Fruson
 */

#include "AP_WingSensor.h"

#if HAL_WINGSENSOR_ENABLED
#include "AP_WingSensor_CanOpen.h"
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
#include <AP_CANManager/AP_CANManager.h>
#endif

extern const AP_HAL::HAL& hal;

AP_WingSensor *AP_WingSensor::singleton;

// Initialize parameters
AP_WingSensor::AP_WingSensor()
{
    singleton = this;
    return;
}

// Initialize backends based on existing params
void AP_WingSensor::init(void)
{
    if (backend != nullptr) {
        // Init called twice, perhaps
        return;
    }

    // create backend object
    backend = new AP_WingSensor_CanOpen(*this);

    return;
}

// Ask all backends to update the frontend
void AP_WingSensor::update()
{
    if (backend) {
        backend->update();
#if HAL_LOGGING_ENABLED
        log_status();
#endif
    }
}

bool AP_WingSensor::is_healthy(void) const
{
    return (backend && (AP_HAL::millis() - state.last_updated_ms) < HEALTHY_LAST_RECEIVED_MS);
}

#if HAL_LOGGING_ENABLED
/*
  write status to log
 */
void AP_WingSensor::log_status(void)
{
// @LoggerMessage: AP_WingSensor
// @Description: Piher Amphenol PST-360 Wing Sensor system data.
// @Field: TimeUS: Time since system startup
// @Field: Angle1: Reported angle 1 in degrees
// @Field: Angle2: Reported angle 2 in degrees
    AP::logger().WriteStreaming("WingSensor",
                       "TimeUS,Angle1,Angle2",
                       "s#-----------",
                       "F------------",
                       "Qff",
                       AP_HAL::micros64(),
                       state.angle_1_deg,
                       state.angle_2_deg);
}
#endif // LOGGING_ENABLED

// get a copy of state structure
void AP_WingSensor::get_state(WingSensor_State &_state)
{
    WITH_SEMAPHORE(sem);
    _state = state;
}

namespace AP {
AP_WingSensor *WingSensor()
{
    return AP_WingSensor::get_singleton();
}
}

#endif // HAL_WINGSENSOR_ENABLED

