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
 * AP_Volz_State.h
 *
 *      Author: Kyle Fruson
 */

#pragma once

#ifndef AP_VOLZ_STATE_H
#define AP_VOLZ_STATE_H

// Stores the current state read by system
// All backends are required to fill in this state structure
struct Volz_State {

public:
    // Setter for target percent command.
    void set_target_percent(uint8_t value) { _target_percent = value;}

    // Getter for target percent command.
    uint8_t get_target_percent() { return _target_percent;}

    // Flag to calibrate wing.
    bool  wing_calibrate;

    // Setter for sweep wing angle.
    void set_sweep_angle(float value) { _sweep_angle = value;}

    // Getter for sweep wing angle.
    float get_sweep_angle() { return _sweep_angle; }

private:
    // Target percent command.
    uint8_t _target_percent;

    // Sweep wing angle in Degrees.
    float _sweep_angle;
};

// Define the global instance of Volz_State
extern Volz_State volz_state;

#endif // AP_VOLZ_STATE_H