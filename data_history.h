#pragma once

// Stores a rolling history of minimum and maximum data values of type T.
//
// To be memory efficient, the design anticipates a use-case where we track
// the minimum and maximum value for some interval and then record a history of
// N values of this tuple.
//
// Min/max data is updated every time updateData is called.
// History is advanced each time updateHistory is called.
// History data is retrieved by passing a callback to forEachData. Callback will
//   be called once or twice (after buffer has wrapped) with a range of data (oldest
//   to newest) for min and max values.
//   Final call will be when offset + len == getLength()
template <typename T>
class MinMaxDataHistory
{
public:
    // Passes min and max data of length len starting from offset in the history.
    typedef void (*Callback)(void* user, const T* dataMin, const T* dataMax, size_t len, size_t offset);

    MinMaxDataHistory() = default;
    ~MinMaxDataHistory()
    {
        if (m_dataMin) {
            delete [] m_dataMin;
            m_dataMin = nullptr;
        }
        if (m_dataMax) {
            delete [] m_dataMax;
            m_dataMax = nullptr;
        }
        m_len = 0;

        reset();
    }

    void begin(size_t length)
    {
        if (m_dataMin){
            delete [] m_dataMin;
        }
        if (m_dataMax){
            delete [] m_dataMax;
        }
        m_dataMin = new T[length];
        m_dataMax = new T[length];
        m_len = length;

        reset();
    }

    void reset()
    {
        m_offset = 0;
        m_bWrapped = false;

        m_bFirst = true;
        m_min = 0;
        m_max = 0;
    }

    void updateData(T t)
    {
        if (m_bFirst || t < m_min) {
            m_min = t;
        }
        if (m_bFirst || t > m_max) {
            m_max = t;
        }

        m_bFirst = false;
    }

    void updateHistory() {
        m_dataMin[m_offset] = m_min;
        m_dataMax[m_offset] = m_max;

        if (++m_offset == m_len) {
            m_bWrapped = true;
            m_offset = 0;
        }

        m_bFirst = true;
    }

    T getMin() const { return m_min; }
    T getMax() const { return m_max; }

    // Pass data history to callback from oldest to latest.
    void forEachData(Callback callback, void* user) const
    {
        size_t offset = 0;
        if (m_bWrapped) {
            (*callback)(user, &m_dataMin[m_offset], &m_dataMax[m_offset],
                        m_len - m_offset, offset);
            offset += m_len - m_offset;
        }
        (*callback)(user, &m_dataMin[0], &m_dataMax[0], m_offset, offset);
    }

    // Max length of history (fixed)
    size_t getSize() const { return m_len; }

    // Current length of history
    size_t getLength() const { return m_bWrapped ? m_len : m_offset; }

private:
    T* m_dataMin = nullptr;
    T* m_dataMax = nullptr;
    size_t m_len = 0;

    size_t m_offset = 0;
    bool m_bWrapped = false;

    // Stats for each update period in the history.
    bool m_bFirst = true;
    T m_min = 0;
    T m_max = 0;
};
