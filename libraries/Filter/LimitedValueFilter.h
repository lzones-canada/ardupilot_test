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

//
/// @file	LimitedValueFilter.h
/// @brief	A class to implement a limited value with upper and lower bounds
/// @author Kyle Fruson

#pragma once

#include <AP_Math/AP_Math.h>
#include "FilterClass.h"

// LimitedValueFilter implements the limited value math
template <typename T>
class LimitedValueFilter {
public:
    // Constructors
    LimitedValueFilter(const T& upper_limit);
    LimitedValueFilter(const T& upper_limit, const T& lower_limit);

    // Function to change the upper limit of the value
    void change_upper_limit(const T& upper_limit);

    // Function to change the lower limit of the value
    void change_lower_limit(const T& lower_limit);

    // Function to set a new value (clipped within limits)
    void new_value(const T& new_nonlimited_value);

    // Function to get the current limited value
    const T& get() const;

    // Function to get the lower limit
    const T& get_lower_limit() const;

    // Function to get the upper limit
    const T& get_upper_limit() const;

    // Function to reset the limited value
    void reset();

private:
    // Declare private copy constructor and copy assignment operator to prevent
    // the compiler from generating default ones.
    LimitedValueFilter(const LimitedValueFilter&);
    LimitedValueFilter& operator=(const LimitedValueFilter&);

    // Private data members
    T maximum_value;
    T minimum_value;
    T current_value;
};