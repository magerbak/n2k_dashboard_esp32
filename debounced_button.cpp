#include <Arduino.h>

#include "debounced_button.h"

DebouncedButton::DebouncedButton(int pin, int activeState) :
    m_pin(pin),
    m_activeState(activeState)
{
    m_buttonValue = m_activeState == HIGH ? LOW : HIGH;
}

void DebouncedButton::begin() {
    pinMode(m_pin, m_activeState == HIGH ? INPUT : INPUT_PULLUP);
}

bool DebouncedButton::updateState() {
    bool bChanged = false;
    uint16_t now = millis();
    int val = digitalRead(m_pin);

    if (val != m_lastValue) {
        m_lastChange = now;
    }

    if (now - m_lastChange > DEBOUNCE_INTERVAL_MS) {
        if (val != m_buttonValue) {
            bChanged = true;
            m_buttonValue = val;
        }
    }

    m_lastValue = val;
    return bChanged;
}

bool DebouncedButton::wasPressed() {
    return updateState() && m_buttonValue == m_activeState;
}

bool DebouncedButton::wasReleased() {
    return updateState() && m_buttonValue != m_activeState;
}




