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

    const time_t getTimestamp() const { return m_timestamp; }
    uint32_t getMmsi() const { return m_mmsi; }
    const char* getName() const { return m_name; }

    double getLength() const { return m_length; }
    double getBeam() const { return m_beam; }
    double getDraft() const { return m_draft; }

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
    static bool copyTrimmed(char* dest, const char* src, size_t size);

    // Time of last position update
    time_t m_timestamp = 0;

    uint32_t m_mmsi = 0;

    N2kPos m_pos;
    N2kVector m_velocity;

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


