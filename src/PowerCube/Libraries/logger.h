// logger.h
//

#ifndef __CUBE_LOGGER_H__
#define __CUBE_LOGGER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//
//
//

int     trace_putc(uint8_t c);
int     trace_puts(const char* str);
int     trace_printf(const char* format, ...);

int     _log_printf(int level, const char* format, ...);

//
//
//

#define LOG_LEVEL_ERROR     (0)
#define LOG_LEVEL_WARNING   (1)
#define LOG_LEVEL_INFO      (2)
#define LOG_LEVEL_VERBOSE   (3)
#define LOG_LEVEL_DEBUG     (4)

#ifndef LOG_LEVEL
#define LOG_LEVEL           -1 // LOG_LEVEL_INFO
#endif

// ERROR
#if LOG_LEVEL >= LOG_LEVEL_ERROR
#define LOGe(x, ...)        _log_printf(LOG_LEVEL_ERROR, x, ##__VA_ARGS__)
#else
#define LOGe(x, ...)
#endif

// WARNING
#if LOG_LEVEL >= LOG_LEVEL_WARNING
#define LOGw(x, ...)        _log_printf(LOG_LEVEL_WARNING, x, ##__VA_ARGS__)
#else
#define LOGw(x, ...)
#endif

// INFO
#if LOG_LEVEL >= LOG_LEVEL_INFO
#define LOGi(x, ...)        _log_printf(LOG_LEVEL_INFO, x, ##__VA_ARGS__)
#else
#define LOGi(x, ...)
#endif

// VERBOSE
#if LOG_LEVEL >= LOG_LEVEL_VERBOSE
#define LOGv(x, ...)        _log_printf(LOG_LEVEL_VERBOSE, x, ##__VA_ARGS__)
#else
#define LOGv(x, ...)
#endif

// DEBUG
#if LOG_LEVEL >= LOG_LEVEL_DEBUG
#define LOGd(x, ...)        _log_printf(LOG_LEVEL_DEBUG, x, ##__VA_ARGS__)
#else
#define LOGd(x, ...)
#endif


#ifdef __cplusplus
}
#endif

#endif // __CUBE_LOGGER_H__
