/**
 * @file log.h
 * @author Ethan J. Musser (emusser@seas.upenn.edu)
 * @brief Macros for logging to console
 * @date 2022-07-13
 *
 * @copyright 2022 The Trustees of the University of Pennsylvania. All rights reserved.
 * 
 * @todo(ethanmusser) Implement stack trace.
 *
 * This header provides a set of debug logging macros with adjustable
 * logging severity levels.  In order of increasing severity, the levels are
 *  - 'DEBUG' (0)
 *  - 'INFO' (1)
 *  - 'WARN' (2)
 *  - 'ERROR' (3)
 *  - 'FATAL' (4)
 *  - 'NONE' (5)
 *  .
 * The minimum level for console output is set by defining `LOG_MIN_SEVERITY`
 * (default is `DEBUG`).
 *
 * Usage of the `LOG_XXXX` logging macros (where `XXXX` is `DEBUG`, `INFO`,
 * etc.) is akin to using [`std::fprintf`](https://en.cppreference.com/w/cpp/io/c/fprintf).
 * For example,
 *
 * ```cpp
 * LOG_WARN("This is a warning message.");
 * LOG_ERROR("%s", "This is an error message.");
 * ```
 *
 * Conditional logging macros are also provided, which take a leading
 * conditional argument before the standard `std::fprintf` input.  For example
 *
 * ```cpp
 * LOG_IF_INFO(false, "%s", "This info message will not be logged.");
 * LOG_IF_FATAL(true, "This fatal message will be logged.");
 * ```
 *
 * Colored terminal output is provided by default via
 * [ANSI escape codes](https://gist.github.com/fnky/458719343aabd01cfb17a3a4f7296797).
 * This can be disabled by defining the `NO_COLOR` macro.
 *
 */

#pragma once

#include <iostream>

///////////////////////////////////////////////////////////////////////////////
// Console Printing                                                          //
///////////////////////////////////////////////////////////////////////////////

// String Formatting
#define NEWLINE "\n"

// OStreams
#define STDERR stderr
#define STDOUT stdout

// Colors
#define CONSOLE_SEQ_ESC "\x1b"
#define CONSOLE_SEQ_BEG "["
#define CONSOLE_SEQ_SEP ";"
#define CONSOLE_SEQ_END "m"
#define FONT_REG "0"
#define FONT_BOLD "1"
#define COLOR_RESET "0"
#define COLOR_BLACK "30"
#define COLOR_RED "31"
#define COLOR_GREEN "32"
#define COLOR_YELLOW "33"
#define COLOR_BLUE "34"
#define COLOR_MAGENTA "35"
#define COLOR_CYAN "36"
#define COLOR_WHITE "37"

// Color Convenience
#define FMT_TEXT(font, color) \
  CONSOLE_SEQ_ESC CONSOLE_SEQ_BEG font CONSOLE_SEQ_SEP color CONSOLE_SEQ_END
#define RESET_COLOR FMT_TEXT(FONT_REG, COLOR_RESET)

// Console printing via ostreams
#define PRINT(ostream, ...) std::fprintf(ostream, __VA_ARGS__)

// Color printing
#ifndef NO_COLOR
#define PRINT_COLOR(ostream, color, str, ...) std::fprintf(ostream, str, __VA_ARGS__)
#else
#define PRINT_COLOR(ostream, color, str, ...) PRINT(ostream, __VA_ARGS__)
#endif

///////////////////////////////////////////////////////////////////////////////
// Logging                                                                   //
///////////////////////////////////////////////////////////////////////////////

// Log Severity Levels
#define SEVERITY_DEBUG 0
#define SEVERITY_INFO 1
#define SEVERITY_WARN 2
#define SEVERITY_ERROR 3
#define SEVERITY_FATAL 4
#define SEVERITY_NONE 5

// Log Severity Tags
#define TAG_DEBUG "DEBUG"
#define TAG_INFO "INFO"
#define TAG_WARN "WARN"
#define TAG_ERROR "ERROR"
#define TAG_FATAL "FATAL"
#define TAG_NONE "NONE"

// Log Severity Tags
#define COLOR_DEBUG FMT_TEXT(FONT_REG, COLOR_BLUE)
#define COLOR_INFO FMT_TEXT(FONT_REG, COLOR_WHITE)
#define COLOR_WARN FMT_TEXT(FONT_REG, COLOR_YELLOW)
#define COLOR_ERROR FMT_TEXT(FONT_REG, COLOR_RED)
#define COLOR_FATAL FMT_TEXT(FONT_BOLD, COLOR_RED)

// Log arguments & formatting flags
// TODO(ethanmusser): Add time to log output.
#define LOG_ARGS(tag) tag, __FILE__, __func__, __LINE__
#define LOG_FORMAT "[%-5s][%-15s | %s:%d] "

// Default ostream (default STDERR)
#ifndef LOG_OSTREAM
#define LOG_OSTREAM STDERR
#endif

// Log to console
// TODO(ethanmusser): Implement logging to file.
#define LOG(msg, tag, args...) PRINT(LOG_OSTREAM, LOG_FORMAT msg NEWLINE, LOG_ARGS(tag), ##args)
#ifndef NO_COLOR
#define LOG_COLOR(msg, tag, color, args...) PRINT(LOG_OSTREAM, color LOG_FORMAT msg NEWLINE RESET_COLOR, LOG_ARGS(tag), ##args)
#else
#define LOG_COLOR(msg, tag, color, args...) LOG(msg, tag, args)
#endif

// Minimum log severity (default DEBUG)
#ifndef LOG_MIN_SEVERITY
#define LOG_MIN_SEVERITY SEVERITY_DEBUG
#endif

// Debug logging
#if LOG_MIN_SEVERITY <= DEBUG_LEVEL
#define LOG_DEBUG(message, args...) LOG_COLOR(message, TAG_DEBUG, COLOR_DEBUG, ##args)
#else
#define LOG_DEBUG(message, args...)
#endif

// Info logging
#if LOG_MIN_SEVERITY <= INFO_LEVEL
#define LOG_INFO(message, args...) LOG_COLOR(message, TAG_INFO, COLOR_INFO, ##args)
#else
#define LOG_INFO(message, args...)
#endif

// Warn logging
#if LOG_MIN_SEVERITY <= WARN_LEVEL
#define LOG_WARN(message, args...) LOG_COLOR(message, TAG_WARN, COLOR_WARN, ##args)
#else
#define LOG_WARN(message, args...)
#endif

// Error logging
#if LOG_MIN_SEVERITY <= ERROR_LEVEL
#define LOG_ERROR(message, args...) LOG_COLOR(message, TAG_ERROR, COLOR_ERROR, ##args)
#else
#define LOG_ERROR(message, args...)
#endif

// Fatal logging
#if LOG_MIN_SEVERITY <= FATAL_LEVEL
#define LOG_FATAL(message, args...) LOG_COLOR(message, TAG_FATAL, COLOR_FATAL, ##args)
#else
#define LOG_FATAL(message, args...)
#endif

// Conditional logs (always defined)
#if LOG_MIN_SEVERITY <= NONE_LEVEL
#define LOG_IF_DEBUG(condition, message, args...) \
    if (condition)                                \
    LOG_COLOR(message, TAG_DEBUG, COLOR_DEBUG, ##args)
#define LOG_IF_INFO(condition, message, args...) \
    if (condition)                               \
    LOG_COLOR(message, TAG_INFO, COLOR_INFO, ##args)
#define LOG_IF_WARN(condition, message, args...) \
    if (condition)                               \
    LOG_COLOR(message, TAG_WARN, COLOR_WARN, ##args)
#define LOG_IF_ERROR(condition, message, args...) \
    if (condition)                                \
    LOG_COLOR(message, TAG_ERROR, COLOR_ERROR, ##args)
#define LOG_IF_FATAL(condition, message, args...) \
    if (condition)                                \
    LOG_COLOR(message, TAG_FATAL, COLOR_FATAL, ##args)
#else
#define LOG_IF_INFO(condition, message, args...)
#define LOG_IF_INFO(condition, message, args...)
#define LOG_IF_WARN(condition, message, args...)
#define LOG_IF_ERROR(condition, message, args...)
#define LOG_IF_FATAL(condition, message, args...)
#endif
