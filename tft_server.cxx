#include <Arduino.h>
#include <TFT_eSPI.h>
#include "tft_server.h"

TFTServer::TFTServer(TFT_eSPI* tft) :
    m_tft(tft), m_state(STATE_INIT)
{
}

TFTServer::~TFTServer()
{
    delete [] m_data;
}

bool TFTServer::run()
{
    uint8_t cmd = 0;


    switch (m_state) {
    case STATE_INIT:
        m_start = millis();
        m_delay = 50;
        m_state = STATE_FLUSH;
        break;
    case STATE_FLUSH:
        if (millis() - m_start < m_delay) {
            if (Serial.available()) {
               cmd = Serial.read();
            }
        }
        else {
            m_start = millis();
            m_delay = 50;
            m_state = STATE_READY;
        }
        break;
    case STATE_READY:
        if (Serial.available()) {
            cmd = Serial.read();

            switch (cmd) {
            case 'E':
                // Echo
                Serial.println("Hello");
                Serial.flush();
                break;
            case 'S':
                if (screenshot()) {
                    m_start = millis();
                    m_delay = 50;
                    m_state = STATE_HDR;
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
        if (millis() - m_start < m_delay) {
            break;
        }
        sendHeader();

        m_start = millis();
        m_delay = IMAGE_XFER_PERIOD_MS;
        m_state = STATE_IMG;
        break;
    case STATE_IMG:
        if (millis() - m_start < m_delay) {
            if (Serial.available()) {
                cmd = Serial.read();

                switch (cmd) {
                case 'X':
                    m_state = STATE_FLUSH;
                    break;
                default:
                    // Ignore other commands
                    break;
                }
            }
            break;
        }
        m_start = millis();
        m_delay = IMAGE_XFER_PERIOD_MS;

        int n = m_tft->width() * m_tft->height() * 3 - m_offset;
        if (n == 0) {
            delete [] m_data;
            m_offset = 0;
            m_start = 0;
            m_delay = 0;
            m_state = STATE_READY;
            break;
        }

        if (n > IMAGE_XFER_BYTES) {
            n = IMAGE_XFER_BYTES;
        }
        Serial.write(&m_data[m_offset], n);
        m_offset += n;
        Serial.flush();
        break;
    }
    return true;
}

bool TFTServer::screenshot()
{
    if (m_data == nullptr) {
        m_data = new uint8_t[m_tft->width() * m_tft->height() * 3];
    }
    m_offset = 0;

    if (m_data) {
        m_tft->readRectRGB(0, 0, m_tft->width(), m_tft->height(), m_data);
    }

    return m_data != nullptr;
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