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
 * AP_ADSB_State.h
 *
 *      Author: Kyle Fruson
 */

#pragma once

#ifndef AP_ADSB_STATE_H
#define AP_ADSB_STATE_H

// Stores the current state read by system
// All backends are required to fill in this state structure
struct ADSB_State {

public:
    // Setter for wing calibrate flag.
    void set_adsb_failsafe(bool value) { _adsb_failsafe = value; }

    // Getter for wing calibrate flag.
    bool get_adsb_failsafe() { return _adsb_failsafe; }

private:
    // Flag to indicate ADSB failsafe.
    bool _adsb_failsafe;
};

// Define the global instance of ADSB_State
extern ADSB_State adsb_state;

#endif // AP_ADSB_STATE_H