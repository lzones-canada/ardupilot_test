/**
 *
 * @file toAlt.c
 * @author Kyle.Fruson
 *
 * @date Oct 18, 2023
 *
 */

#include "sgUtil.h"


/*
 * Documented in the header file.
 */
double toAlt2(const double alt)
{
    double value = alt;
    value = (value * 25.0) - 1000.0;

    return value;
}
