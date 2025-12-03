#include "strlcpy.h"

size_t strlcpy(char* dest, const char* src, size_t size)
{
    if (!dest || !src || !size) {
        return 0;
    }

    size_t len;
    size_t ix;

    for (len = 0, ix = 0; src[len]; len++) {
        if (ix + 1 < size) {
            dest[ix] = src[len];
            ix++;
        }
    }
    dest[ix] = '\0';

    // ix is the number of characters copied.
    // len is the number of characters we would have written.

    return len;
}

