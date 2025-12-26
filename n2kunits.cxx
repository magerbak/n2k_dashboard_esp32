#include <cmath>

#include "n2kunits.h"

double rad2Deg(double rad)
{
    if (std::isnan(rad)) {
        return rad;
    }
    return rad * (180.0 / M_PI);
}

double deg2Rad(double deg)
{
    if (std::isnan(deg)) {
        return deg;
    }
    return deg * (M_PI / 180.0);
}

// Converts a bearing of 0...360 into +-180
double getSignedBearing(double bearing)
{
    return bearing > 180.0 ? (bearing - 360.0) : bearing;
}

// Normalizes bearing between 0...360.
double normalizeBearing(double bearing)
{
    return bearing - (trunc(bearing / 360.0) * 360.0);
}

double fixedFrac2Double(uint16_t x, double precision)
{
    return (double)x * precision;
}

double meters2Ft(double m)
{
    if (std::isnan(m)) {
        return m;
    }

    return m * 3.28084;
}

double meters2Nm(double m)
{
    if (std::isnan(m)) {
        return m;
    }
    // 1852m = 1nm
    return m / 1852.0;
}

double metersPerSec2Kts(double spd)
{
    if (std::isnan(spd)) {
        return spd;
    }
    // kts = nm/hr
    // 1852m = 1nm
    // 3600s = 1hr
    return spd * (3600.0 / 1852.0);
}

bool areFloatsEqual(double a, double b, double epsilon)
{
    return std::fabs(a - b) < epsilon;
}


