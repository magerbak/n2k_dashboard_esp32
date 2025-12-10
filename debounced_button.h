#include <stdint.h>

class DebouncedButton
{
public:
    DebouncedButton(int pin, int activeState = HIGH);
    ~DebouncedButton() = default;

    void begin();

    // Updates button state and returns whether indicated state change occured.
    bool wasPressed();
    bool wasReleased();

    // Lower level methods to independently update button state and query the new state.
    bool updateState();
    bool isPressed() const { return m_buttonValue == m_activeState; }

private:
    static const uint16_t DEBOUNCE_INTERVAL_MS = 50;



    int m_pin = 0;
    int m_activeState = HIGH;

    uint16_t m_lastChange = 0;
    int m_lastValue = LOW;
    int m_buttonValue = LOW;
};


