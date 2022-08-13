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

// C Escapes
#define BELL "\a"  ///< Terminal bell character
#define BACKSPACE "\b"  ///< Backspace character
#define TAB "\t"  ///< Horizontal tab character
#define NEWLINE "\n"  ///< Newline character
#define RETURN "\r"  ///< Carriage return character

// OStreams
#define STDERR stderr  ///< Standard error stream
#define STDOUT stdout  ///< Standard output stream

// ANSI Escape Sequences
#define CONSOLE_SEQ_ESC "\x1b"  ///< ANSI escape sequence
#define CONSOLE_SEQ_BEG "["  ///< ANSI begin sequence
#define CONSOLE_SEQ_SEP ";"  ///< ANSI separation sequence
#define CONSOLE_SEQ_END "m"  ///< ANSI end sequence

// ANSI Graphics Modes
#define FONT_REGULAR "0"  ///< ANSI reset mode sequence
#define FONT_BOLD "1"  ///< ANSI bold mode sequence
#define FONT_DIM "2"  ///< ANSI dim/faint mode sequence
#define FONT_ITALIC "3"  ///< ANSI italic mode sequence
#define FONT_UNDERLINE "4"  ///< ANSI underline mode sequence
#define FONT_BLINK "5"  ///< ANSI blinking mode sequence
#define FONT_INVERSE "7"  ///< ANSI inverse/reverse mode sequence
#define FONT_HIDDEN "8"  ///< ANSI hidden/invisible mode sequence
#define FONT_STRIKETHROUGH "9"  ///< ANSI strikethrough mode sequence

// ANSI Colors
#define COLOR_RESET "0"  ///< ANSI reset color and mode sequence
#define COLOR_BLACK_FG "30"  ///< ANSI black foreground sequence
#define COLOR_RED_FG "31"  ///< ANSI red foreground sequence
#define COLOR_GREEN_FG "32"  ///< ANSI green foreground sequence
#define COLOR_YELLOW_FG "33"  ///< ANSI yellow foreground sequence
#define COLOR_BLUE_FG "34"  ///< ANSI blue foreground sequence
#define COLOR_MAGENTA_FG "35"  ///< ANSI magenta foreground sequence
#define COLOR_CYAN_FG "36"  ///< ANSI cyan foreground sequence
#define COLOR_WHITE_FG "37"  ///< ANSI white foreground sequence
#define COLOR_DEFAULT_FG "39"  ///< ANSI default foreground sequence
#define COLOR_BLACK_BG "40"  ///< ANSI black background sequence
#define COLOR_RED_BG "41"  ///< ANSI red background sequence
#define COLOR_GREEN_BG "42"  ///< ANSI green background sequence
#define COLOR_YELLOW_BG "43"  ///< ANSI yellow background sequence
#define COLOR_BLUE_BG "44"  ///< ANSI blue background sequence
#define COLOR_MAGENTA_BG "45"  ///< ANSI magenta background sequence
#define COLOR_CYAN_BG "46"  ///< ANSI cyan background sequence
#define COLOR_WHITE_BG "47"  ///< ANSI white background sequence
#define COLOR_DEFAULT_BG "49"  ///< ANSI default background sequence
#define COLOR_BRIGHT_BLACK_FG "90"  ///< ANSI BRIGHT black foreground sequence
#define COLOR_BRIGHT_RED_FG "91"  ///< ANSI BRIGHT red foreground sequence
#define COLOR_BRIGHT_GREEN_FG "92"  ///< ANSI BRIGHT green foreground sequence
#define COLOR_BRIGHT_YELLOW_FG "93"  ///< ANSI BRIGHT yellow foreground sequence
#define COLOR_BRIGHT_BLUE_FG "94"  ///< ANSI BRIGHT blue foreground sequence
#define COLOR_BRIGHT_MAGENTA_FG "95"  ///< ANSI BRIGHT magenta foreground sequence
#define COLOR_BRIGHT_CYAN_FG "96"  ///< ANSI BRIGHT cyan foreground sequence
#define COLOR_BRIGHT_WHITE_FG "97"  ///< ANSI BRIGHT white foreground sequence
#define COLOR_BRIGHT_BLACK_BG "100"  ///< ANSI BRIGHT black background sequence
#define COLOR_BRIGHT_RED_BG "101"  ///< ANSI BRIGHT red background sequence
#define COLOR_BRIGHT_GREEN_BG "102"  ///< ANSI BRIGHT green background sequence
#define COLOR_BRIGHT_YELLOW_BG "103"  ///< ANSI BRIGHT yellow background sequence
#define COLOR_BRIGHT_BLUE_BG "104"  ///< ANSI BRIGHT blue background sequence
#define COLOR_BRIGHT_MAGENTA_BG "105"  ///< ANSI BRIGHT magenta background sequence
#define COLOR_BRIGHT_CYAN_BG "106"  ///< ANSI BRIGHT cyan background sequence
#define COLOR_BRIGHT_WHITE_BG "107"  ///< ANSI BRIGHT white background sequence

// Color Convenience
#define FMT_TEXT(font, color) \
  CONSOLE_SEQ_ESC CONSOLE_SEQ_BEG font CONSOLE_SEQ_SEP color CONSOLE_SEQ_END
#define FMT_TEXT_BG(font, fg_color, bg_color) \
  CONSOLE_SEQ_ESC CONSOLE_SEQ_BEG font CONSOLE_SEQ_SEP bg_color CONSOLE_SEQ_SEP fg_color CONSOLE_SEQ_END
#define RESET_COLOR FMT_TEXT(FONT_REGULAR, COLOR_RESET)

// Console printing via ostreams
#define PRINT(ostream, str, args...) std::fprintf(ostream, str NEWLINE, ##args)

// Color printing
#ifndef NO_COLOR
#define PRINT_FMT(ostream, ansi_fmt, str, args...) \
  std::fprintf(ostream, ansi_fmt str RESET_COLOR NEWLINE, ##args)
#else
#define PRINT_FMT(ostream, color, str, args...) PRINT(ostream, ##args)
#endif

///////////////////////////////////////////////////////////////////////////////
// Logging                                                                   //
///////////////////////////////////////////////////////////////////////////////

// Log Severity Levels
#define SEVERITY_ALL 0
#define SEVERITY_TRACE 1
#define SEVERITY_DEBUG 2
#define SEVERITY_INFO 3
#define SEVERITY_NOTICE 4
#define SEVERITY_WARN 5
#define SEVERITY_ERROR 6
#define SEVERITY_FATAL 7
#define SEVERITY_NONE 8

// Log Severity Tags
#define TAG_TRACE "TRACE"
#define TAG_DEBUG "DEBUG"
#define TAG_INFO "INFO"
#define TAG_NOTICE "NOTICE"
#define TAG_WARN "WARN"
#define TAG_ERROR "ERROR"
#define TAG_FATAL "FATAL"

// Log Severity Tags
#define COLOR_TRACE FMT_TEXT(FONT_REGULAR, COLOR_MAGENTA_FG)
#define COLOR_DEBUG FMT_TEXT(FONT_REGULAR, COLOR_BLUE_FG)
#define COLOR_INFO FMT_TEXT(FONT_REGULAR, COLOR_WHITE_FG)
#define COLOR_NOTICE FMT_TEXT(FONT_REGULAR, COLOR_GREEN_FG)
#define COLOR_WARN FMT_TEXT(FONT_REGULAR, COLOR_YELLOW_FG)
#define COLOR_ERROR FMT_TEXT(FONT_REGULAR, COLOR_RED_FG)
#define COLOR_FATAL FMT_TEXT(FONT_BOLD, COLOR_RED_FG)

// Log arguments & formatting flags
// TODO(ethanmusser): Add time to log output.
#ifndef LOG_ARGS
#define LOG_ARGS(tag) tag
#endif
#ifndef LOG_FORMAT
#define LOG_FORMAT "[%-6s] "
#endif

// Verbose log arguments & formatting flags
#ifndef VLOG_ARGS
#define VLOG_ARGS(tag) tag, __FILE__, __func__, __LINE__
#endif
#ifndef VLOG_FORMAT
#define VLOG_FORMAT "[%-6s][%-15s | %s:%d] "
#endif

// Default ostream (default STDERR)
#ifndef LOG_OSTREAM
#define LOG_OSTREAM STDERR
#endif

// Log to console
// TODO(ethanmusser): Implement logging to file.
#define LOG(msg, tag, args...) PRINT(LOG_OSTREAM, LOG_FORMAT msg, LOG_ARGS(tag), ##args)
#define VLOG(msg, tag, args...) PRINT(LOG_OSTREAM, VLOG_FORMAT msg, VLOG_ARGS(tag), ##args)
#ifndef NO_COLOR
#define LOG_COLOR(msg, tag, color, args...) PRINT_FMT(LOG_OSTREAM, color, LOG_FORMAT msg RESET_COLOR, LOG_ARGS(tag), ##args)
#define VLOG_COLOR(msg, tag, color, args...) PRINT_FMT(LOG_OSTREAM, color, VLOG_FORMAT msg RESET_COLOR, VLOG_ARGS(tag), ##args)
#else
#define LOG_COLOR(msg, tag, color, args...) LOG(msg, tag, args)
#define VLOG_COLOR(msg, tag, color, args...) VLOG(msg, tag, args)
#endif

// Minimum log severity (default ALL)
#ifndef LOG_MIN_SEVERITY
#define LOG_MIN_SEVERITY SEVERITY_ALL
#endif

// Trace logging
#if LOG_MIN_SEVERITY <= SEVERITY_TRACE
#define LOG_TRACE(message, args...) LOG_COLOR(message, TAG_TRACE, COLOR_TRACE, ##args)
#define VLOG_TRACE(message, args...) VLOG_COLOR(message, TAG_TRACE, COLOR_TRACE, ##args)
#define LOG_IF_TRACE(condition, message, args...) \
    if (condition)                                \
    LOG_TRACE(message, ##args)
#define VLOG_IF_TRACE(condition, message, args...) \
    if (condition)                                \
    VLOG_TRACE(message, ##args)
#else
#define LOG_TRACE(message, args...)
#define VLOG_TRACE(message, args...)
#define LOG_IF_TRACE(condition, message, args...)
#define VLOG_IF_TRACE(condition, message, args...)
#endif

// Debug logging
#if LOG_MIN_SEVERITY <= SEVERITY_DEBUG
#define LOG_DEBUG(message, args...) LOG_COLOR(message, TAG_DEBUG, COLOR_DEBUG, ##args)
#define VLOG_DEBUG(message, args...) VLOG_COLOR(message, TAG_DEBUG, COLOR_DEBUG, ##args)
#define LOG_IF_DEBUG(condition, message, args...) \
    if (condition)                                \
    LOG_DEBUG(message, ##args)
#define VLOG_IF_DEBUG(condition, message, args...) \
    if (condition)                                \
    VLOG_DEBUG(message, ##args)
#else
#define LOG_DEBUG(message, args...)
#define VLOG_DEBUG(message, args...)
#define LOG_IF_DEBUG(condition, message, args...)
#define VLOG_IF_DEBUG(condition, message, args...)
#endif

// Info logging
#if LOG_MIN_SEVERITY <= SEVERITY_INFO
#define LOG_INFO(message, args...) LOG_COLOR(message, TAG_INFO, COLOR_INFO, ##args)
#define VLOG_INFO(message, args...) VLOG_COLOR(message, TAG_INFO, COLOR_INFO, ##args)
#define LOG_IF_INFO(condition, message, args...) \
    if (condition)                               \
    LOG_INFO(message, ##args)
#define VLOG_IF_INFO(condition, message, args...) \
    if (condition)                               \
    VLOG_INFO(message, ##args)
#else
#define LOG_INFO(message, args...)
#define VLOG_INFO(message, args...)
#define LOG_IF_INFO(condition, message, args...)
#define VLOG_IF_INFO(condition, message, args...)
#endif

// Notice logging
#if LOG_MIN_SEVERITY <= SEVERITY_NOTICE
#define LOG_NOTICE(message, args...) LOG_COLOR(message, TAG_NOTICE, COLOR_NOTICE, ##args)
#define VLOG_NOTICE(message, args...) VLOG_COLOR(message, TAG_NOTICE, COLOR_NOTICE, ##args)
#define LOG_IF_NOTICE(condition, message, args...) \
    if (condition)                                \
    LOG_NOTICE(message, ##args)
#define VLOG_IF_NOTICE(condition, message, args...) \
    if (condition)                                \
    VLOG_NOTICE(message, ##args)
#else
#define LOG_NOTICE(message, args...)
#define VLOG_NOTICE(message, args...)
#define LOG_IF_NOTICE(condition, message, args...)
#define VLOG_IF_NOTICE(condition, message, args...)
#endif

// Warn logging
#if LOG_MIN_SEVERITY <= SEVERITY_WARN
#define LOG_WARN(message, args...) LOG_COLOR(message, TAG_WARN, COLOR_WARN, ##args)
#define VLOG_WARN(message, args...) VLOG_COLOR(message, TAG_WARN, COLOR_WARN, ##args)
#define LOG_IF_WARN(condition, message, args...) \
    if (condition)                               \
    LOG_WARN(message, ##args)
#define VLOG_IF_WARN(condition, message, args...) \
    if (condition)                               \
    VLOG_WARN(message, ##args)
#else
#define LOG_WARN(message, args...)
#define VLOG_WARN(message, args...)
#define LOG_IF_WARN(condition, message, args...)
#define VLOG_IF_WARN(condition, message, args...)
#endif

// Error logging
#if LOG_MIN_SEVERITY <= SEVERITY_ERROR
#define LOG_ERROR(message, args...) LOG_COLOR(message, TAG_ERROR, COLOR_ERROR, ##args)
#define VLOG_ERROR(message, args...) VLOG_COLOR(message, TAG_ERROR, COLOR_ERROR, ##args)
#define LOG_IF_ERROR(condition, message, args...) \
    if (condition)                                \
    LOG_ERROR(message, ##args)
#define VLOG_IF_ERROR(condition, message, args...) \
    if (condition)                                \
    VLOG_ERROR(message, ##args)
#else
#define LOG_ERROR(message, args...)
#define VLOG_ERROR(message, args...)
#define LOG_IF_ERROR(condition, message, args...)
#define VLOG_IF_ERROR(condition, message, args...)
#endif

// Fatal logging
#if LOG_MIN_SEVERITY <= SEVERITY_FATAL
#define LOG_FATAL(message, args...) LOG_COLOR(message, TAG_FATAL, COLOR_FATAL, ##args)
#define VLOG_FATAL(message, args...) VLOG_COLOR(message, TAG_FATAL, COLOR_FATAL, ##args)
#define LOG_IF_FATAL(condition, message, args...) \
    if (condition)                                \
    LOG_FATAL(message, ##args)
#define VLOG_IF_FATAL(condition, message, args...) \
    if (condition)                                \
    VLOG_FATAL(message, ##args)
#else
#define LOG_FATAL(message, args...)
#define VLOG_FATAL(message, args...)
#define LOG_IF_FATAL(condition, message, args...)
#define VLOG_IF_FATAL(condition, message, args...)
#endif

