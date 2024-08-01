//
/// @file LimitedValueFilter.cpp
/// @brief	A class to implement a limited value with upper and lower bounds
/// @author Kyle Fruson

#include "LimitedValueFilter.h"


////////////////////////////////////////////////////////////////////////////////////////////
// Limited Value Filter
////////////////////////////////////////////////////////////////////////////////////////////

template <typename T>
LimitedValueFilter<T>::LimitedValueFilter(const T& upper_limit)
    : current_value(), maximum_value(upper_limit), minimum_value(-upper_limit) {}

template <typename T>
LimitedValueFilter<T>::LimitedValueFilter(const T& upper_limit, const T& lower_limit)
    : current_value(), maximum_value(upper_limit), minimum_value(lower_limit) {}

template <typename T>
void LimitedValueFilter<T>::change_upper_limit(const T& upper_limit) {
    maximum_value = upper_limit;
}

template <typename T>
void LimitedValueFilter<T>::change_lower_limit(const T& lower_limit) {
    minimum_value = lower_limit;
}

template <typename T>
void LimitedValueFilter<T>::new_value(const T& new_nonlimited_value) {
    if (new_nonlimited_value > maximum_value) {
        current_value = maximum_value;
    } else if (new_nonlimited_value < minimum_value) {
        current_value = minimum_value;
    } else {
        current_value = new_nonlimited_value;
    }
}

template <typename T>
const T& LimitedValueFilter<T>::get() const {
    return current_value;
}

template <typename T>
const T& LimitedValueFilter<T>::get_lower_limit() const {
    return minimum_value;
}

template <typename T>
const T& LimitedValueFilter<T>::get_upper_limit() const {
    return maximum_value;
}

template <typename T>
void LimitedValueFilter<T>::reset() {
    // Reset implementation here
}

/* 
  instantiate template classes
 */
template class LimitedValueFilter<int8_t>;
template class LimitedValueFilter<int16_t>;
template class LimitedValueFilter<int32_t>;
template class LimitedValueFilter<uint8_t>;
template class LimitedValueFilter<uint16_t>;
template class LimitedValueFilter<uint32_t>;
template class LimitedValueFilter<double>;
template class LimitedValueFilter<float>;