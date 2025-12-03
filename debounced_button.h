#include <stdint.h>

class DebouncedButton
{
public:
    DebouncedButton(int pin, int activeState = HIGH);
    ~DebouncedButton() = default;

    void begin();

    bool wasPressed();
    bool wasReleased();

private:
    static const uint16_t DEBOUNCE_INTERVAL_MS = 50;

    bool updateState();

    int m_pin = 0;
    int m_activeState = HIGH;

    uint16_t m_lastChange = 0;
    int m_lastValue = LOW;
    int m_buttonValue = LOW;
};


