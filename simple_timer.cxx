#include <Arduino.h>

#include "simple_timer.h"

void SimpleTimer::begin(void* user,  uint32_t interval, SimpleTimerCallback callback)
{
    m_user = user;
    m_interval = interval;
    m_start = millis();

    m_callback = callback;
}


void SimpleTimer::cancel()
{
    m_callback = nullptr;

    m_user = nullptr;
    m_interval = 0;
    m_start = 0;
}

void SimpleTimer::setInterval(uint32_t interval) {
    m_interval = interval;
}

uint32_t SimpleTimer::getRemaining() const {
    uint32_t t = millis();

    if (t < m_start + m_interval) {
        return m_start + m_interval - t;
    }

    return 0;
}


void SimpleTimer::tick(uint32_t millis)
{
    uint32_t next_start = m_start + m_interval;

    if (m_callback && millis - m_start >= m_interval) {
        bool bContinue = (*m_callback)(m_user);

        if (bContinue) {
            m_start = next_start;
        }
        else {
            cancel();
        }
    }
}

