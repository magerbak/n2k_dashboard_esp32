#pragma once

#include <stdint.h>
#include <stddef.h>
#include <time.h>

#include "n2kpos.h"
#include "n2kvector.h"
#include "n2kcpa.h"

// Represents an AIS target.
//
// MMSI is used as the unique identifier for each object and is required at
// construction time.
//
// Depending on which messages have been subsequently received, only a subset
// of information may be populated.
//
// In particular, static vessel info (length, name, etc) is generally received
// separately from dynamic info (position, speed, etc). Class B reports contain
// a subset of class A information.
//
// getTimestamp will return 0 until dynamic data has been set.
//
// getName will return the MMSI in the absence of static vessel name info.
//
// calcCPA will be ignored until dynamic position info has been received.
class N2kAISTarget
{
public:
    N2kAISTarget(uint32_t mmsi);
    ~N2kAISTarget() = default;

    enum AISClass {
        CLASS_B,
        CLASS_A,
    };

    enum NavStatus {
        NAV_STATUS_UNDER_WAY_MOTORING = 0,
        NAV_STATUS_AT_ANCHOR = 1,
        NAV_STATUS_NOT_UNDER_COMMAND = 2,
        NAV_STATUS_RESTRICTED_MANOEUVERABILITY = 3,
        NAV_STATUS_CONSTRAINED_BY_DRAUGHT = 4,
        NAV_STATUS_MOORED = 5,
        NAV_STATUS_AGROUND = 6,
        NAV_STATUS_FISHING = 7,
        NAV_STATUS_UNDER_WAY_SAILING = 8,
        NAV_STATUS_HAZARDOUS_MATERIAL_HIGH_SPEED = 9,
        NAV_STATUS_HAZARDOUS_MATERIAL_WING_IN_GROUND = 10,
        NAV_STATUS_AIS_SART = 14,
    };

    const time_t getTimestamp() const { return m_timestamp; }
    uint32_t getMmsi() const { return m_mmsi; }
    const char* getName() const { return m_name; }
    AISClass getClass() const { return m_class; }

    const char* getDest() const { return m_dest; }

    NavStatus getNavStatus() const { return m_navStatus; }
    const char* getNavStatusStr() const;

    uint8_t getVesselType() const { return m_type; }
    const char* getVesselTypeStr() const;

    double getLength() const { return m_length; }
    double getBeam() const { return m_beam; }
    double getDraft() const { return m_draft; }

    void update(const N2kPos &pos, const N2kVector &vel, NavStatus status);
    void update(const N2kPos& pos, const N2kVector& vel);
    void update(const char* name);
    void update(uint8_t type, double length, double beam, double draft,
                const char* callsign, const char* name, const char* dest);

    void calcCpa(const N2kPos &ref, const N2kVector &refVelocity);
    const N2kVector& getRelDistance() const { return m_relDistance; }
    const N2kVector& getVelocity() const { return m_velocity; }
    const Cpa* getCpa() const { return &m_cpa; }

    bool toString(char* buffer, size_t len) const;
    bool toStringVesselInfo(char* buffer, size_t len) const;

private:
    static const char* s_navStatusTable[NAV_STATUS_AIS_SART + 1];
    static bool copyTrimmed(char* dest, const char* src, size_t size);

    // Time of last position update
    time_t m_timestamp = 0;

    uint32_t m_mmsi = 0;
    AISClass m_class = CLASS_B;

    N2kPos m_pos;
    N2kVector m_velocity;
    NavStatus m_navStatus = NAV_STATUS_UNDER_WAY_MOTORING;

    uint8_t m_type = 0;
    double m_length = 0;
    double m_beam = 0;
    double m_draft = 0;
    char m_callsign[7 + 1] = { };
    char m_name[20 + 1] = { };
    char m_dest[20 + 1] = { };

    N2kVector m_relDistance;
    Cpa m_cpa;
};


