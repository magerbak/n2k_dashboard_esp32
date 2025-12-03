#ifndef STRLCPY_H_
#define STRLCPY_H_

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

size_t strlcpy(char* dest, const char* src, size_t size);

#ifdef __cplusplus
}
#endif

#endif
