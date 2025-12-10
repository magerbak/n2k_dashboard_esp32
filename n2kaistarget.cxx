#include <stdio.h>
#include <time.h>
#include <string.h>
#include <cmath>

#include "strlcpy.h"
#include "n2kaistarget.h"

N2kAISTarget::N2kAISTarget(uint32_t mmsi) :
    m_mmsi(mmsi)
{
    // Default name is MMSI
    snprintf(m_name, sizeof(m_name), "%u", mmsi);
}

void N2kAISTarget::update(const N2kPos &pos, const N2kVector &vel)
{
    m_timestamp = time(nullptr);

    m_pos = pos;
    m_velocity = vel;
}

void N2kAISTarget::update(const char* name)
{
    if (name) {
        copyTrimmed(m_name, name, sizeof(m_name));
    }
}

void N2kAISTarget::update(uint8_t type, double length, double beam, double draft,
                          const char* callsign, const char* name, const char* dest)
{
    m_type = type;
    m_length = length;
    m_beam = beam;
    m_draft = draft;

    if (callsign) {
        copyTrimmed(m_callsign, callsign, sizeof(m_callsign));
    }
    if (name) {
        copyTrimmed(m_name, name, sizeof(m_name));
    }
    if (dest) {
        copyTrimmed(m_dest, dest, sizeof(m_dest));
    }
}

bool N2kAISTarget::copyTrimmed(char* dest, const char* src, size_t size) {
    bool bIsTruncated = strlcpy(dest, src, size) >= size;

    if (!bIsTruncated) {
        size_t l = strlen(dest);
        while (l > 0 && dest[l - 1] == ' ') {
            dest[--l] = '\0';
        }
    }
    return bIsTruncated;
}

void N2kAISTarget::calcCpa(const N2kPos &refPos, const N2kVector &refVelocity)
{
    m_relDistance = m_pos.getRelDistance(refPos);
    m_cpa.update(m_pos, m_velocity, refPos, refVelocity);
}

bool N2kAISTarget::toString(char* buffer, size_t len) const
{
    time_t now = time(nullptr);

    //char pos[32];
    //m_pos.toString(pos, sizeof(pos));

    int rc;
    if (m_cpa.getRelTime(now) >= 0.0 && !std::isnan(m_cpa.getDistance())) {
        rc = snprintf(buffer, len,
                      "%s, Range %.1fnm, Bearing %.0f, COG %.0f, SOG %.1fkts, CPA %.2fnm, TCPA %.1fmins",
                      m_name, m_relDistance.getMagnitude(), m_relDistance.getBearing(),
                      m_velocity.getBearing(), m_velocity.getMagnitude(),
                      m_cpa.getDistance(), m_cpa.getRelTime(now) / 60);
    }
    else {
        rc = snprintf(buffer, len,
                      "%s, Range %.1fnm, Bearing %.0f, COG %.0f, SOG %.1fkts",
                      m_name, m_relDistance.getMagnitude(), m_relDistance.getBearing(),
                      m_velocity.getBearing(), m_velocity.getMagnitude());
    }

    if (rc < 0 || rc == (int)len) {
        // Truncation or error
        return false;
    }
    return true;
}

bool N2kAISTarget::toStringVesselInfo(char* buffer, size_t len) const
{
    int rc = -1;

    if (m_callsign[0] || m_length > 0.0 || m_beam > 0.0 || m_draft > 0.0) {
        rc = snprintf(buffer, len,
                      "MMSI %u, %s, LOA %.1fft, Beam %.1fft, Draft %.1fft, Type %u",
                      m_mmsi, m_callsign, m_length, m_beam, m_draft, m_type);
    }

    if (rc < 0 || rc == (int)len) {
        // Truncation or error
        return false;
    }
    return true;
}



