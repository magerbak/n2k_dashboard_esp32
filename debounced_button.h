#include <stdint.h>
#include <Arduino.h>

class DebouncedButton
{
public:
    DebouncedButton(int pin, int activeState = HIGH);
    ~DebouncedButton() = default;

    void begin();

    // Update button state. Returns true if state changed.
    bool updateState();
    // Query the current button state.
    bool isPressed() const { return m_buttonValue == m_activeState; }

    // Update button state and return whether indicated state change occured
    // since last update.
    bool wasPressed();
    bool wasReleased();

private:
    static const uint16_t DEBOUNCE_INTERVAL_MS = 50;

    int m_pin = 0;
    int m_activeState = HIGH;

    uint16_t m_lastChange = 0;
    int m_lastValue = LOW;
    int m_buttonValue = LOW;
};


