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
 * AP_IMET_State.h
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
struct IMET_Main_State {

    // Sensor ID for package
    int sensorID;

    // Pressure (mbar)
    double pressure;

    // Pressure Sensor Substrate Temperature (Celsius)
    double pressureTemp;

    // Humidity (% RH)
    double humidity;

    // Humidity Sensor Substrate Temperature (Celsius)
    double humidityTemp;
};


// GPS Sensor State
struct IMET_GPS_State {

    // Sensor ID for package (03)
    int sensorID;

    // Date (YYYY/MM/DD), Time (HH:MM:SS UTC)
    time_t datetime;

    // Latitude (degrees)
    double latitude;

    // Longitude (degrees)
    double longitude;

    // Number of satellites visible and count.
    double satelliteCount;
};


// Air Temperature Sensor State
struct IMET_Temp_State {

    // Sensor ID for package (05) - A Configuration
    int sensorID;

    // Glass Bead NTC Thermistor
    double airTemp;

    // Resistance of Thermistor (Ohms)
    double resistance;
};

// Main Sensor Structure for IMET
struct IMET_Sensor_State {

    // Main Board Sensor ID (0)
    IMET_Main_State mainSensor;

    // GPS Sensor ID (03)
    IMET_GPS_State gpsSensor;

    // Air Temperature Sensor ID (05)
    IMET_Temp_State airTempSensor;
};