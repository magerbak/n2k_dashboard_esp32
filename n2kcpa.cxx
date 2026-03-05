#include <stdio.h>
#include <cmath>
#include "n2kcpa.h"

Cpa::Cpa(const N2kPos &a, const N2kVector &va, const N2kPos &b, const N2kVector &vb)
{
    update(a, va, b, vb);
}


void Cpa::update(const N2kPos &a, const N2kVector &va, const N2kPos &b, const N2kVector &vb)
{
    N2kVector DeltaPos = a.getRelDistance(b);
    N2kVector DeltaV;
    time_t now = time(nullptr);

    DeltaV.setXY(va.getX() - vb.getX(), va.getY() - vb.getY());

    // If AIS position updates are infrequent compared to local position updates,
    // it may be useful to store the timestamp of the AIS position report so that
    // it can be brought forward to the current time before calculating the CPA.

    // Denominator for CPA time calc below.
    double dv2 = DeltaV.getY() * DeltaV.getY() + DeltaV.getX() * DeltaV.getX();

    // If both vessels are effectively moving in parallel then CPA time calculation
    // will involve a divide by (almost) zero.
    if (dv2 < 1e-10) {
        // Vessels moving in parallel — CPA is current distance, time is now
        m_t = now;
        m_v = DeltaPos;
        return;
    }

    // Time of CPA in hours (assuming velocity is in knots).
    // See https://pierdusud.com/en/on-the-closest-point-of-approach/ for
    // a simple derivation.
    double t = -(DeltaPos.getY() * DeltaV.getY() + DeltaPos.getX() * DeltaV.getX()) / dv2;

    // Store the absolute time in seconds.
    m_t = now + round(t * 3600);

    // Calculate relative position at time of CPA
    N2kVector CpaDeltaPos;
    CpaDeltaPos.setXY(DeltaPos.getX() + DeltaV.getX() * t,
                      DeltaPos.getY() + DeltaV.getY() * t);
    m_v = CpaDeltaPos;
}

