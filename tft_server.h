#pragma once
#include <stdint.h>

class TFT_eSPI;

class TFTServer
{
public:
    TFTServer(TFT_eSPI* tft, uint8_t* buf, size_t bufLen);
    ~TFTServer();

    bool run();

private:
    static constexpr uint32_t IMAGE_XFER_PERIOD_MS = 1;
    static constexpr uint32_t IMAGE_XFER_BYTES = 32;
    bool screenshot();
    void sendHeader();

    enum State {
        STATE_INIT,
        STATE_FLUSH,
        STATE_READY,
        STATE_HDR,
        STATE_IMG,
    };
    State m_state = STATE_INIT;
    uint32_t m_start = 0;
    uint32_t m_delay = 0;

    TFT_eSPI* m_tft = nullptr;
    uint8_t* m_data = nullptr;
    size_t m_maxlen = 0;
    int m_offset = 0;
};
