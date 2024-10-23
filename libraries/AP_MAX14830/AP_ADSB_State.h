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

// get high or low nibble from byte
#define LOW_NIBBLE(x)	((uint8_t) (x & 0xf))
#define HIGH_NIBBLE(x)	((uint8_t) ((x >> 4) & 0xf))

// Stores the current state read by system
// All backends are required to fill in this state structure
class AP_ADSB_State {
public:
    // Getter for the singleton instance
    static AP_ADSB_State& get_singleton() {
        static AP_ADSB_State _singleton;  // Guaranteed to be lazy-initialized and thread-safe
        return _singleton;
    }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_ADSB_State);

    // Setter for ADS-B failsafe flag.
    void set_adsb_failsafe(bool value) {
        WITH_SEMAPHORE(_sem);
        _adsb_failsafe = value;
    }

    // Getter for ADS-B failsafe flag.
    bool get_adsb_failsafe() {
        WITH_SEMAPHORE(_sem);
        return _adsb_failsafe;
    }

private:
    // Private constructor to prevent direct instantiation
    AP_ADSB_State() = default; 
    
    // Flag to indicate ADSB failsafe
    bool _adsb_failsafe = false;

    // Semaphore for access to shared frontend data
    HAL_Semaphore _sem;
};
