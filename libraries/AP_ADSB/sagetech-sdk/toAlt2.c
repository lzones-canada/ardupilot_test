/**
 *
 * @file toAlt.c
 * @author Kyle Fruson
 *
 * @date Oct 18, 2023
 *
 */

#include "sgUtil.h"


/*
 * Documented in the header file.
    Altitude: 12-bit offset integer. Resolution = 25 feet.
    Altitude (ft) = ("ddd" * 25) - 1,000
 */
double toAlt2(const double alt)
{
    double value = alt;
    value = (value * 25.0) - 1000.0;

    return value;
}