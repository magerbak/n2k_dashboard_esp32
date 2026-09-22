#pragma once

#include <stdint.h>

class TouchRect
{
public:
    typedef void (*pfCallback)(void* user);

    TouchRect() = default;
    ~TouchRect() = default;

    void init(int16_t x, int16_t y, uint16_t w, uint16_t h);
    void setReleaseCallback(pfCallback callback, void* user);

    // Helper that calls contains() and press()
    void update(bool press,  int16_t x, int16_t y);
    bool contains(int16_t x, int16_t y) const;
    void press(bool pressed);

    bool isPressed() const { return m_currState; }
    bool justPressed() const { return m_currState && !m_lastState; }
    bool justReleased() const { return !m_currState && m_lastState; }

    // Only valid while isPressed() or justReleased() return true
    uint32_t getHoldTime() const;

    void getBoundingRect(int16_t* px,  int16_t* py, uint16_t* pw, uint16_t* ph) const {
        *px = m_x0;
        *py = m_y0;
        *pw = m_w;
        *ph = m_h;
    }

private:
    int16_t m_x0 = 0;
    int16_t m_y0 = 0;
    uint16_t m_w = 0;
    uint16_t m_h = 0;

    pfCallback m_releaseCallback = nullptr;
    void* m_releaseUserData = nullptr;

    bool m_currState = false;
    bool m_lastState = false;
    uint32_t m_pressTimestamp = 0;
};


