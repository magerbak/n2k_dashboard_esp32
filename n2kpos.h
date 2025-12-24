#pragma once

#include "n2kvector.h"

class N2kPos
{
public:
    // Latitude and longitude in degrees.
    N2kPos() = default;
    N2kPos(double Lat, double Lon);
    ~N2kPos() = default;

    void set(double Lat, double Lon) { m_lat = Lat; m_lon = Lon; }

    double getLatitude() const { return m_lat; }
    double getLongitude() const { return m_lon; }

    // Returns a new position, adjusted by a relative position in nautical miles.
    const N2kPos getAdjPosition(const N2kVector &Adj) const;

    // Returns non-geodesic relative distance in nautical miles (not accurate
    // over large distances or close to poles).
    const N2kVector getRelDistance(const N2kPos &Ref) const;

    enum FormatOption {
        FMT_LAT_ONLY,
        FMT_LON_ONLY,
        FMT_LAT_AND_LON,
    };
    bool toString(char* buf, size_t maxlen, FormatOption = FMT_LAT_AND_LON) const;

private:
    double m_lat = 0.0;
    double m_lon = 0.0;
};



