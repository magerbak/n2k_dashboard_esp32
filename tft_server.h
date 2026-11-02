#pragma once
#include <stdint.h>

class TFT_eSPI;

class TFTServer
{
public:
    // buf should be big enough for entire screenshot bitmap.
    TFTServer(TFT_eSPI* tft, uint8_t* buf, size_t bufLen);
    ~TFTServer();

    bool run();

private:
    // Rate control. Transfer IMAGE_XFER_BYTES every IMAGE_XFER_PERIOD_MS.
    // Modify these params (as well as Serial baudrate) to control transfer
    // rate, efficiency and CPU utilization.
    static constexpr uint32_t IMAGE_XFER_PERIOD_MS = 1;
    static constexpr uint32_t IMAGE_XFER_BYTES = 32;

    void setTimeout(uint32_t timeout);
    bool checkTimeout() const;
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

    // Timer state
    uint32_t m_start = 0;
    uint32_t m_timeout = 0;

    // Pointer to display driver
    TFT_eSPI* m_tft = nullptr;

    // Screenshot state
    uint8_t* m_data = nullptr;
    size_t m_maxlen = 0;
    int m_offset = 0;
};
