#include <cmath>

#include "n2kvector.h"
#include "n2kunits.h"

N2kVector::N2kVector(double mag, double bearing)
{
    double rad = deg2Rad(bearing);

    m_x = mag * std::sin(rad);
    m_y = mag * std::cos(rad);
}

void N2kVector::set(double mag, double bearing)
{
    double rad = deg2Rad(bearing);

    m_x = mag * std::sin(rad);
    m_y = mag * std::cos(rad);
}

double N2kVector::getMagnitude() const
{
    return std::sqrt(m_x * m_x + m_y * m_y);
}

double N2kVector::getBearing() const
{
    double angle = rad2Deg(std::atan2(m_x, m_y));

    return angle < 0 ? angle + 360 : angle;
}


