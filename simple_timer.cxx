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


void SimpleTimer::tick(uint32_t millis)
{
    if (m_callback && millis - m_start >= m_interval) {
        bool bContinue = (*m_callback)(m_user);

        if (bContinue) {
            m_start = millis;
        }
        else {
            cancel();
        }
    }
}

