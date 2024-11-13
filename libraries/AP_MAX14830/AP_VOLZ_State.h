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

// Define the wing limits
#define WING_MIN_DEGREES   (8.0)  // Fully Open (angled measured from vertical, dont' ask me why)
#define WING_MAX_DEGREES   (88.0) // Fully Closed (angled measured from vertical, dont' ask me why)

// Stores the current state read by system
// All backends are required to fill in this state structure
class AP_VOLZ_State {
public:
    // Getter for the singleton instance
    static AP_VOLZ_State& get_singleton() {
        static AP_VOLZ_State _singleton; // Guaranteed to be lazy-initialized and thread-safe
        return _singleton;
    }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_VOLZ_State);

    // Setter for target command.
    void set_target_command(uint8_t value) { 
        WITH_SEMAPHORE(_sem);
        _target_command = value;
    }

    // Getter for target command.
    uint8_t get_target_command() { 
        WITH_SEMAPHORE(_sem);
        return _target_command;
    }

    // Setter for wing calibrate flag.
    void set_calibrate(bool value) { 
        WITH_SEMAPHORE(_sem);
        _wing_calibrate = value;
    }

    // Getter for wing calibrate flag.
    bool get_calibrate() { 
        WITH_SEMAPHORE(_sem);
        return _wing_calibrate;
    }

    // Setter for sweep wing angle.
    void set_sweep_angle(float value) { 
        WITH_SEMAPHORE(_sem);
        _sweep_angle = value;
    }

    // Getter for sweep wing angle.
    float get_sweep_angle() { 
        WITH_SEMAPHORE(_sem);
        return _sweep_angle;
    }

    // Setter for PCB temperature.
    void set_pcb_temp(float value) { 
        WITH_SEMAPHORE(_sem);
        _pcb_temp_scaled = value;
    }

    // Getter for PCB temperature.
    float get_pcb_temp() { 
        WITH_SEMAPHORE(_sem);
        return _pcb_temp_scaled;
    }

    // Setter for voltage.
    void set_voltage(float value) { 
        WITH_SEMAPHORE(_sem);
        _voltage_raw = value;
    }

    // Getter for voltage.
    float get_voltage() { 
        WITH_SEMAPHORE(_sem);
        return _voltage_raw;
    }

    // Setter for current.
    void set_current(float value) { 
        WITH_SEMAPHORE(_sem);
        _current_raw = value;
    }

    // Getter for current.
    float get_current() { 
        WITH_SEMAPHORE(_sem);
        return _current_raw;
    }

private:
    // Private constructor to prevent direct instantiation
    AP_VOLZ_State() = default; 

    // Target percent command.
    uint8_t _target_command = 0;

    // Sweep wing angle in Degrees.
    float _sweep_angle = 0.0;

    // PCB Temperature - raw value.
    float _pcb_temp_scaled = 0;

    // Voltage Input - raw value.
    float _voltage_raw = 0;

    // Current Consumption - raw value.
    float _current_raw = 0;

    // Flag to calibrate wing.
    bool _wing_calibrate = false;

    // Semaphore for access to shared frontend data
    HAL_Semaphore _sem;
};
