#include <cmath>

#include "n2kvector.h"
#include "n2kunits.h"

N2kVector::N2kVector(double mag, double bearing)
{
    set(mag, bearing);
}

void N2kVector::set(double mag, double bearing)
{
    double rad = deg2Rad(bearing);

    m_mag = mag;
    m_bearing = bearing;

    m_x = mag * std::sin(rad);
    m_y = mag * std::cos(rad);
}

void N2kVector::setXY(double x, double y)
{
    double angle = rad2Deg(std::atan2(x, y));

    m_bearing = angle < 0 ? angle + 360 : angle;
    m_mag = std::sqrt(x * x + y * y);

    m_x = x;
    m_y = y;
};



