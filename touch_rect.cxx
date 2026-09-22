#include <Arduino.h>
#include "touch_rect.h"

void TouchRect::init(int16_t x, int16_t y, uint16_t w, uint16_t h)
{
    m_x0 = x;
    m_y0 = y;
    m_w = w;
    m_h = h;

    m_releaseCallback = nullptr;
    m_releaseUserData = nullptr;

    m_currState = false;
    m_lastState = false;

    m_pressTimestamp = 0;
}

void TouchRect::setReleaseCallback(pfCallback callback, void* user)
{
    m_releaseCallback = callback;
    m_releaseUserData = user;
}

void TouchRect::update(bool pressed,  int16_t x, int16_t y)
{
    if (pressed & contains(x, y)) {
        press(true);
    }
    else {
        press(false);
    }
}

bool TouchRect::contains(int16_t x, int16_t y) const
{
    return (x >= m_x0 && x < m_x0 + m_w) && (y >= m_y0 && y < m_y0 + m_h);
}

void TouchRect::press(bool pressed)
{
    m_lastState = m_currState;
    m_currState = pressed;

    if (justPressed()) {
        m_pressTimestamp = millis();
    }
    else if (justReleased() && m_releaseCallback) {
        m_releaseCallback(m_releaseUserData);
    }
}

uint32_t TouchRect::getHoldTime() const
{
    return millis() - m_pressTimestamp;
}

