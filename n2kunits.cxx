#include <cmath>
#include <assert.h>

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

// Converts a bearing of 0...2PI into +-PI
double getSignedBearing(double bearing) {
    return bearing > M_PI ? (bearing - 2 * M_PI) : bearing;
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


