#include <cmath>
#include <stdio.h>
#include <assert.h>

#include "n2kpos.h"
#include "n2kunits.h"

N2kPos::N2kPos(double Lat, double Lon) :
    m_lat(Lat),
    m_lon(Lon)
{
    assert(m_lat <= 90.0 && m_lat >= -90.0);
    assert(m_lon <= 180.0 && m_lon > -180.0);
}

const N2kVector N2kPos::getRelDistance(const N2kPos &Ref) const
{
    double avgLat = (m_lat - Ref.m_lat) / 2.0 + Ref.m_lat;
    double y = (m_lat - Ref.m_lat) * 60.0;
    double Delta = (m_lon - Ref.m_lon);

    if (Delta > 180.0) {
        Delta = -360.0 + Delta;
    }
    else if (Delta < -180.0) {
        Delta = 360 + Delta;
    }

    double x = Delta * std::cos(deg2Rad(avgLat)) * 60.0;

    N2kVector v;
    v.setXY(x, y);

    return v;
}

const N2kPos N2kPos::getAdjPosition(const N2kVector &Adj) const
{
    double Lat = m_lat + Adj.getY() /  60.0;
    double Lon = m_lon;

    // Handle wrap scenarios
    if (Lat > 90.0) {
        Lat = 180.0 - Lat;
    }
    else if (Lat < -90.0) {
        Lat = -180.0 - Lat;
    }

    double avgLat = (Lat - m_lat) / 2.0 + m_lat;
    if (m_lat != 90.0 && m_lat != -90.0) {
        double ScaledAdj = Adj.getX() / (60.0 * std::cos(deg2Rad(avgLat)));
        if (ScaledAdj >= -180.0 && ScaledAdj <= 180.0) {
            Lon = m_lon + ScaledAdj;
        }
    }

    if (Lon > 180.0) {
        Lon = Lon - 360.0;
    }
    else if (Lon <= -180.0) {
        Lon = Lon + 360.0;
    }

    return N2kPos(Lat, Lon);
}

bool N2kPos::toString(char* buf, size_t maxlen) const
{
    double absLat = fabs(m_lat);
    double degsLat = floor(absLat);
    double minsLat = (absLat - degsLat) * 60.0;
    double absLon = fabs(m_lon);
    double degsLon = floor(absLon);
    double minsLon = (absLon - degsLon) * 60.0;

    int rc = snprintf(buf, maxlen,
                      "%.0f°%.3f'%c %03.0f°%.3f'%c",
                      degsLat, minsLat, m_lat >= 0 ? 'N' : 'S',
                      degsLon, minsLon, m_lon >= 0 ? 'E' : 'W');

    if (rc < 0 || rc == (int)maxlen) {
        // Truncation or error
        return false;
    }
    return true;
}




