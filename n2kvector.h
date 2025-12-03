#pragma once

class N2kVector
{
public:
    N2kVector() = default;
    // Bearing in degrees relative to 0' North.
    N2kVector(double mag, double bearing);
    virtual ~N2kVector() = default;

    void set(double mag, double bearing);
    void setXY(double x, double y) { m_x = x; m_y = y; };

    double getX() const { return m_x;}
    double getY() const {return m_y;}
    double getMagnitude() const;

    // Bearing in degrees relative to 0' North.
    double getBearing() const;

private:
    double m_x = 0.0;
    double m_y = 0.0;
};

