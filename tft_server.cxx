#include <Arduino.h>
#include <TFT_eSPI.h>
#include "tft_server.h"

TFTServer::TFTServer(TFT_eSPI* tft, uint8_t* data, size_t maxlen) :
    m_tft(tft), m_data(data), m_maxlen(maxlen)
{
}

TFTServer::~TFTServer()
{
}

bool TFTServer::run()
{
    uint8_t cmd = 0;


    switch (m_state) {
    case STATE_INIT:
        setTimeout(50);
        m_state = STATE_FLUSH;
        break;
    case STATE_FLUSH:
        if (!checkTimeout()) {
            if (Serial.available()) {
               cmd = Serial.read();
            }
        }
        else {
            m_state = STATE_READY;
        }
        break;
    case STATE_READY:
        if (Serial.available()) {
            cmd = Serial.read();

            switch (cmd) {
            case 'H':
                // Hello. For testing.
                Serial.println("Hello");
                Serial.flush();
                break;
            case 'S':
                if (screenshot()) {
                    setTimeout(50);
                    m_state = STATE_HDR;
                }
                else {
                    Serial.write('X');
                    Serial.flush();
                }
                break;
            default:
                Serial.write('?');
                Serial.flush();
                break;
            }
        }
        break;
    case STATE_HDR:
        if (!checkTimeout()) {
            break;
        }
        sendHeader();

        setTimeout(IMAGE_XFER_PERIOD_MS);
        m_state = STATE_IMG;
        break;
    case STATE_IMG:
        if (!checkTimeout()) {
            if (Serial.available()) {
                cmd = Serial.read();

                switch (cmd) {
                case 'X':
                    setTimeout(50);
                    m_state = STATE_FLUSH;
                    break;
                default:
                    // Ignore other commands
                    break;
                }
            }
            break;
        }
        setTimeout(IMAGE_XFER_PERIOD_MS);

        int n = m_tft->width() * m_tft->height() * 3 - m_offset;
        if (n == 0) {
            // Complete
            setTimeout(0);
            m_offset = 0;
            m_state = STATE_READY;
            break;
        }

        if (n > IMAGE_XFER_BYTES) {
            n = IMAGE_XFER_BYTES;
        }
        Serial.write(&m_data[m_offset], n);
        Serial.flush();
        m_offset += n;
        break;
    }
    return true;
}

void TFTServer::setTimeout(uint32_t timeout)
{
    m_start = millis();
    m_timeout = timeout;
}

bool TFTServer::checkTimeout() const
{
    return (millis() - m_start >= m_timeout);
}

bool TFTServer::screenshot()
{
    if (m_data == nullptr || m_tft->width() * m_tft->height() * 3 > m_maxlen) {
        return false;
    }
    m_offset = 0;

    m_tft->readRectRGB(0, 0, m_tft->width(), m_tft->height(), m_data);

    return true;
}

void TFTServer::sendHeader()
{
    Serial.write("V");
    Serial.write(1);        // Version 1
    Serial.write("W");
    Serial.write((uint16_t)m_tft->width() >> 8);
    Serial.write((uint16_t)m_tft->width() & 0xff);
    Serial.write("H");
    Serial.write((uint16_t)m_tft->height() >> 8);
    Serial.write((uint16_t)m_tft->height() & 0xff);
    Serial.write("Y");
    Serial.write(24);       // 24-bit pixels (RGB).
    Serial.write("D");      // W * H * 3 bytes of data follow
    Serial.flush();
}