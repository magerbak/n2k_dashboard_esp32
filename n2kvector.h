#pragma once

class N2kVector
{
public:
    N2kVector() = default;
    // Bearing in degrees relative to 0' North.
    N2kVector(double mag, double bearing);
    virtual ~N2kVector() = default;

    void set(double mag, double bearing);
    double getMagnitude() const { return m_mag; }
    double getBearing() const { return m_bearing; }
    double getSignedBearing() const { return m_bearing > 180.0 ? (m_bearing - 360.0) : m_bearing; }

    void setXY(double x, double y);
    double getX() const { return m_x; }
    double getY() const { return m_y; }

private:
    // Cartesian components
    double m_x = 0.0;
    double m_y = 0.0;

    // Polar components. Bearing in degrees relative to 0' North.
    double m_mag = 0.0;
    double m_bearing = 0.0;
};

