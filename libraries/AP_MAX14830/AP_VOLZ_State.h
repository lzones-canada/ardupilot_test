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
 * AP_VOLZ_State.h
 *
 *      Author: Kyle Fruson
 */

#pragma once

#ifndef AP_VOLZ_STATE_H
#define AP_VOLZ_STATE_H

// Define the wing limits
#define WING_MIN_DEGREES   (8.0)  // Fully Open (angled measured from vertical, dont' ask me why)
#define WING_MAX_DEGREES   (88.0) // Fully Closed (angled measured from vertical, dont' ask me why)

// Stores the current state read by system
// All backends are required to fill in this state structure
struct Volz_State {

public:
    // Setter for target percent command.
    void set_target_command(uint8_t value) { _target_command = value;}

    // Getter for target percent command.
    uint8_t get_target_command() { return _target_command;}

    // Setter for wing calibrate flag.
    void set_calibrate(bool value) { _wing_calibrate = value; }

    // Getter for wing calibrate flag.
    bool get_calibrate() { return _wing_calibrate; }

    // Setter for sweep wing angle.
    void set_sweep_angle(float value) { _sweep_angle = value;}

    // Getter for sweep wing angle.
    float get_sweep_angle() { return _sweep_angle; }

private:
    // Target percent command.
    uint8_t _target_command;

    // Sweep wing angle in Degrees.
    float _sweep_angle;

    // Flag to calibrate wing.
    bool  _wing_calibrate;
};

// Define the global instance of Volz_State
extern Volz_State volz_state;

#endif // AP_VOLZ_STATE_H