#pragma once

#include <time.h>
#include "n2kpos.h"
#include "n2kvector.h"

// Represents the closest point of approach of vessel A to vessel B.
//
// To calculate, we need the position and velocity of both vessels and this
// yields a range, bearing and time (stored in epoch time).
class Cpa
{
public:
    Cpa() : m_t(0) {}
    Cpa(const N2kPos& a, const N2kVector& va, const N2kPos& b, const N2kVector& vb);
    ~Cpa() = default;

    void update(const N2kPos &a, const N2kVector &va, const N2kPos &b, const N2kVector &vb);

    double getDistance() const { return m_v.getMagnitude(); }
    double getBearing() const { return m_v.getBearing(); }

    const time_t* getTime() const { return &m_t; }
    double getRelTime(time_t Ref) const { return difftime(m_t, Ref); }

private:
    N2kVector m_v;
    time_t m_t;
};
