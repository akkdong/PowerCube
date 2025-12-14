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

#if OBSOLETE

int     trace_putc(uint8_t c);
int     trace_puts(const char* str);
int     trace_printf(const char* format, ...);

int     _log_printf(int level, const char* format, ...);

#else

extern int Trace(const char* format, ...);

#define _log_printf(level, format, ...)	Trace(format, ##__VA_ARGS__);

#endif

//
//
//

#define LOG_LEVEL_ERROR     (1)
#define LOG_LEVEL_WARNING   (2)
#define LOG_LEVEL_INFO      (3)
#define LOG_LEVEL_VERBOSE   (4)
#define LOG_LEVEL_DEBUG     (5)

#ifndef LOG_LEVEL
#define LOG_LEVEL           (0) // LOG_LEVEL_INFO
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
