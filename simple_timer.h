#pragma once

#include <stdint.h>

typedef bool (*SimpleTimerCallback)(void*);

class SimpleTimer
{
public:
    SimpleTimer() = default;
    ~SimpleTimer() = default;

    void begin(void* user,  uint32_t interval, SimpleTimerCallback callback);
    void cancel();

    bool isEnabled() const { return m_callback != nullptr; }
    void tick(uint32_t millis);

private:
    SimpleTimerCallback m_callback = nullptr;
    void* m_user = nullptr;
    uint32_t m_interval = 0;
    uint32_t m_start = 0;
};
