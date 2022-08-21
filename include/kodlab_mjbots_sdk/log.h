/**
 * @file log.h
 * @author Ethan J. Musser (emusser@seas.upenn.edu)
 * @brief Macros for logging to console
 * @date 2022-07-13
 *
 * @copyright (c) Copyright 2022 The Trustees of the University of Pennsylvania.
 * All rights reserved.
 *
 * The `log.h` header provides a set of debug logging macros with adjustable
 * logging severity levels.  In order of increasing severity, the levels are
 * `TRACE`, `DEBUG`, `INFO`, `NOTICE`, `WARN`, `ERROR`, and `FATAL`.
 *
 * The minimum severity level for console output is set by defining the
 * `LOG_MIN_SEVERITY` macro (default is `SEVERITY_ALL`).  Setting
 * `LOG_MIN_SEVERITY` to `SEVERITY_NONE` will disable macro console output.
 *
 * Usage of the `LOG_XXXX` logging macros (where `XXXX` is `DEBUG`, `INFO`,
 * etc.) is akin to using [`std::fprintf`](https://en.cppreference.com/w/cpp/io/c/fprintf).
 * For example,
 * ```cpp
 * LOG_WARN("This is a warning message.");
 * LOG_ERROR("%s", "This is an error message.");
 * ```
 *
 * Conditional logging macros `LOG_IF_XXXX` are also provided, which take a
 * leading conditional argument before the standard `std::fprintf` input.  For
 * example,
 * ```cpp
 * LOG_IF_INFO(false, "%s", "This info message will not be logged.");
 * LOG_IF_FATAL(true, "This fatal message will be logged.");
 * ```
 *
 * Verbose logging commands are provided for all logging macros, and take the
 * form `VLOG_XXXX` or `VLOG_IF_XXXX`.  The default output of the log and
 * verbose log macros is as follows.
 * ```
 * [SEVERITY] Log message
 * [SEVERITY][path/to/file | function:line_no] Verbose log message
 * ```
 * The output behavior can be changed by redefining the `LOG_ARGS`,
 * `LOG_FORMAT`, `VLOG_ARGS`, and `VLOG_FORMAT` macros.  For example, to produce
 * verbose output of the form
 * ```
 * [SEVERITY][path/to/file][line_no][function] Verbose log message
 * ```
 * the verbose macros would be redefined as follows
 * ```cpp
 * #define VLOG_ARGS(tag) tag, __FILE__, __LINE__, __func__
 * #define VLOG_FORMAT "[%-6s][%-15s][%d][%s] "
 * ```
 *
 * Colored terminal output is provided by default via
 * [ANSI escape codes](https://gist.github.com/fnky/458719343aabd01cfb17a3a4f7296797).
 * This can be disabled by defining the `NO_COLOR` macro.
 *
 *
 * @todo Implement stack trace.
 * @todo Add time to log output.
 * @todo Implement logging to file.
 *
 */

#pragma once

#include <iostream>  // std::fprintf

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

/**
 * @brief Format and color foreground of text
 * @param font ANSI graphics mode specifier
 * @param color ANSI foreground color specifier
 */
#define FMT_TEXT(font, color) \
  CONSOLE_SEQ_ESC CONSOLE_SEQ_BEG font CONSOLE_SEQ_SEP color CONSOLE_SEQ_END

/**
 * @brief Format and color both foreground and background of text
 * @param font ANSI graphics mode specifier
 * @param fg_color ANSI foreground color specifier
 * @param bg_color ANSI background color specifier
 */
#define FMT_TEXT_BG(font, fg_color, bg_color) \
  CONSOLE_SEQ_ESC CONSOLE_SEQ_BEG font CONSOLE_SEQ_SEP bg_color CONSOLE_SEQ_SEP fg_color CONSOLE_SEQ_END

/**
 * @brief ANSI sequence to reset graphics and color
 */
#define RESET_COLOR FMT_TEXT(FONT_REGULAR, COLOR_RESET)

/**
 * @brief Print to console via an output stream
 * @param ostream output stream
 * @param str string to be formatted and printed
 * @param args string format arguments
 */
#define PRINT(ostream, str, args...) std::fprintf(ostream, str NEWLINE, ##args)

#ifndef NO_COLOR
/**
 * @brief Print to console in color via an output stream
 * @param ostream output stream
 * @param ansi_fmt ANSI format sequence beginning with `CONSOLE_SEQ_ESC` and
 * `CONSOLE_SEQ_BEG`, and ending with `CONSOLE_SEQ_END`.  Use `FMT_TEXT` or
 * `FMT_TEXT_BG` macros to formulate this argument.
 * @param str string to be formatted and printed
 * @param args string format arguments
 */
#define PRINT_FMT(ostream, ansi_fmt, str, args...) \
  std::fprintf(ostream, ansi_fmt str RESET_COLOR NEWLINE, ##args)
#else
#define PRINT_FMT(ostream, color, str, args...) PRINT(ostream, ##args)
#endif

///////////////////////////////////////////////////////////////////////////////
// Logging                                                                   //
///////////////////////////////////////////////////////////////////////////////

// Log Severity Levels
#define SEVERITY_ALL 0  ///< Minimum logging severity level
#define SEVERITY_TRACE 1  ///< Trace logging severity level
#define SEVERITY_DEBUG 2  ///< Debug logging severity level
#define SEVERITY_INFO 3  ///< Info logging severity level
#define SEVERITY_NOTICE 4  ///< Notice logging severity level
#define SEVERITY_WARN 5  ///< Warn logging severity level
#define SEVERITY_ERROR 6  ///< Error logging severity level
#define SEVERITY_FATAL 7  ///< Fatal logging severity level
#define SEVERITY_NONE 8  ///< Maximum logging severity level

// Log Severity Tags
#define TAG_TRACE "TRACE"  ///< Trace logging tag
#define TAG_DEBUG "DEBUG"  ///< Debug logging tag
#define TAG_INFO "INFO"  ///< Info logging tag
#define TAG_NOTICE "NOTICE"  ///< Notice logging tag
#define TAG_WARN "WARN"  ///< Warn logging tag
#define TAG_ERROR "ERROR"  ///< Error logging tag
#define TAG_FATAL "FATAL"  ///< Fatal logging tag

// Log Severity Tags
#define COLOR_TRACE FMT_TEXT(FONT_REGULAR, COLOR_MAGENTA_FG)  ///< Trace ANSI font/color sequence
#define COLOR_DEBUG FMT_TEXT(FONT_REGULAR, COLOR_BLUE_FG)  ///< Debug ANSI font/color sequence
#define COLOR_INFO FMT_TEXT(FONT_REGULAR, COLOR_WHITE_FG)  ///< Info ANSI font/color sequence
#define COLOR_NOTICE FMT_TEXT(FONT_REGULAR, COLOR_GREEN_FG)  ///< Notice ANSI font/color sequence
#define COLOR_WARN FMT_TEXT(FONT_REGULAR, COLOR_YELLOW_FG)  ///< Warn ANSI font/color sequence
#define COLOR_ERROR FMT_TEXT(FONT_REGULAR, COLOR_RED_FG)  ///< Error ANSI font/color sequence
#define COLOR_FATAL FMT_TEXT(FONT_BOLD, COLOR_RED_FG)  ///< Fatal ANSI font/color sequence

#ifndef LOG_ARGS
/**
 * @brief Log arguments
 * @param tag log severity tag
 */
#define LOG_ARGS(tag) tag
#endif
#ifndef LOG_FORMAT
#define LOG_FORMAT "[%-6s] " ///< Log format string
#endif

#ifndef VLOG_ARGS
/**
 * @brief Verbose log arguments
 * @param tag log severity tag
 */
#define VLOG_ARGS(tag) tag, __FILE__, __func__, __LINE__
#endif
#ifndef VLOG_FORMAT
#define VLOG_FORMAT "[%-6s][%-15s | %s:%d] "  ///< Verbose log format string
#endif

#ifndef LOG_OSTREAM
#define LOG_OSTREAM STDERR  ///< Logging output stream (default STDERR)
#endif

/**
 * @brief Log message with severity tag to console
 * @param msg string to be formatted and logged
 * @param tag log severity tag
 * @param args string format arguments
 */
#define LOG(msg, tag, args...) PRINT(LOG_OSTREAM, LOG_FORMAT msg, LOG_ARGS(tag), ##args)

/**
 * @brief Verbosely log message with severity tag to console
 * @param msg string to be formatted and logged
 * @param tag log severity tag
 * @param args string format arguments
 */
#define VLOG(msg, tag, args...) PRINT(LOG_OSTREAM, VLOG_FORMAT msg, VLOG_ARGS(tag), ##args)

#ifndef NO_COLOR
/**
 * @brief Log formatted message with severity tag to console
 * @param msg string to be formatted and logged
 * @param tag log severity tag
 * @param fmt_seq ANSI format sequence
 * @param args string format arguments
 */
#define LOG_COLOR(msg, tag, fmt_seq, args...) PRINT_FMT(LOG_OSTREAM, fmt_seq, LOG_FORMAT msg RESET_COLOR, LOG_ARGS(tag), ##args)

/**
 * @brief Verbosely log formatted message with severity tag to console
 * @param msg string to be formatted and logged
 * @param tag log severity tag
 * @param fmt_seq ANSI format sequence
 * @param args string format arguments
 */
#define VLOG_COLOR(msg, tag, fmt_seq, args...) PRINT_FMT(LOG_OSTREAM, fmt_seq, VLOG_FORMAT msg RESET_COLOR, VLOG_ARGS(tag), ##args)
#else
#define LOG_COLOR(msg, tag, color, args...) LOG(msg, tag, args)
#define VLOG_COLOR(msg, tag, color, args...) VLOG(msg, tag, args)
#endif

#ifndef LOG_MIN_SEVERITY
#define LOG_MIN_SEVERITY SEVERITY_ALL  ///< Minimum log severity (default SEVERITY_ALL)
#endif

#if LOG_MIN_SEVERITY <= SEVERITY_TRACE
/**
 * @brief Log message to console with `TRACE` severity level
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define LOG_TRACE(message, args...) LOG_COLOR(message, TAG_TRACE, COLOR_TRACE, ##args)

/**
 * @brief Log verbose message to console with `TRACE` severity level
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define VLOG_TRACE(message, args...) VLOG_COLOR(message, TAG_TRACE, COLOR_TRACE, ##args)

/**
 * @brief Conditionally log message to console with `TRACE` severity level
 * @param condition condition dictating whether to log
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define LOG_IF_TRACE(condition, message, args...) \
    if (condition)                                \
    LOG_TRACE(message, ##args)

/**
 * @brief Conditionally log verbose message to console with `TRACE` severity
 * level
 * @param condition condition dictating whether to log
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define VLOG_IF_TRACE(condition, message, args...) \
    if (condition)                                \
    VLOG_TRACE(message, ##args)
#else
#define LOG_TRACE(message, args...)
#define VLOG_TRACE(message, args...)
#define LOG_IF_TRACE(condition, message, args...)
#define VLOG_IF_TRACE(condition, message, args...)
#endif

#if LOG_MIN_SEVERITY <= SEVERITY_DEBUG
/**
 * @brief Log message to console with `DEBUG` severity level
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define LOG_DEBUG(message, args...) LOG_COLOR(message, TAG_DEBUG, COLOR_DEBUG, ##args)

/**
 * @brief Log verbose message to console with `DEBUG` severity level
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define VLOG_DEBUG(message, args...) VLOG_COLOR(message, TAG_DEBUG, COLOR_DEBUG, ##args)

/**
 * @brief Conditionally log message to console with `DEBUG` severity level
 * @param condition condition dictating whether to log
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define LOG_IF_DEBUG(condition, message, args...) \
    if (condition)                                \
    LOG_DEBUG(message, ##args)

/**
 * @brief Conditionally log verbose message to console with `DEBUG` severity
 * level
 * @param condition condition dictating whether to log
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define VLOG_IF_DEBUG(condition, message, args...) \
    if (condition)                                \
    VLOG_DEBUG(message, ##args)
#else
#define LOG_DEBUG(message, args...)
#define VLOG_DEBUG(message, args...)
#define LOG_IF_DEBUG(condition, message, args...)
#define VLOG_IF_DEBUG(condition, message, args...)
#endif

#if LOG_MIN_SEVERITY <= SEVERITY_INFO
/**
 * @brief Log message to console with `INFO` severity level
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define LOG_INFO(message, args...) LOG_COLOR(message, TAG_INFO, COLOR_INFO, ##args)

/**
 * @brief Log verbose message to console with `INFO` severity level
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define VLOG_INFO(message, args...) VLOG_COLOR(message, TAG_INFO, COLOR_INFO, ##args)

/**
 * @brief Conditionally log message to console with `INFO` severity level
 * @param condition condition dictating whether to log
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define LOG_IF_INFO(condition, message, args...) \
    if (condition)                               \
    LOG_INFO(message, ##args)

/**
 * @brief Conditionally log verbose message to console with `INFO` severity
 * level
 * @param condition condition dictating whether to log
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define VLOG_IF_INFO(condition, message, args...) \
    if (condition)                               \
    VLOG_INFO(message, ##args)
#else
#define LOG_INFO(message, args...)
#define VLOG_INFO(message, args...)
#define LOG_IF_INFO(condition, message, args...)
#define VLOG_IF_INFO(condition, message, args...)
#endif

#if LOG_MIN_SEVERITY <= SEVERITY_NOTICE
/**
 * @brief Log message to console with `NOTICE` severity level
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define LOG_NOTICE(message, args...) LOG_COLOR(message, TAG_NOTICE, COLOR_NOTICE, ##args)

/**
 * @brief Log verbose message to console with `NOTICE` severity level
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define VLOG_NOTICE(message, args...) VLOG_COLOR(message, TAG_NOTICE, COLOR_NOTICE, ##args)

/**
 * @brief Conditionally log message to console with `NOTICE` severity level
 * @param condition condition dictating whether to log
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define LOG_IF_NOTICE(condition, message, args...) \
    if (condition)                                \
    LOG_NOTICE(message, ##args)

/**
 * @brief Conditionally log verbose message to console with `NOTICE` severity
 * level
 * @param condition condition dictating whether to log
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define VLOG_IF_NOTICE(condition, message, args...) \
    if (condition)                                \
    VLOG_NOTICE(message, ##args)
#else
#define LOG_NOTICE(message, args...)
#define VLOG_NOTICE(message, args...)
#define LOG_IF_NOTICE(condition, message, args...)
#define VLOG_IF_NOTICE(condition, message, args...)
#endif

#if LOG_MIN_SEVERITY <= SEVERITY_WARN
/**
 * @brief Log message to console with `WARN` severity level
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define LOG_WARN(message, args...) LOG_COLOR(message, TAG_WARN, COLOR_WARN, ##args)

/**
 * @brief Log verbose message to console with `WARN` severity level
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define VLOG_WARN(message, args...) VLOG_COLOR(message, TAG_WARN, COLOR_WARN, ##args)

/**
 * @brief Conditionally log message to console with `WARN` severity level
 * @param condition condition dictating whether to log
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define LOG_IF_WARN(condition, message, args...) \
    if (condition)                               \
    LOG_WARN(message, ##args)

/**
 * @brief Conditionally log verbose message to console with `WARN` severity
 * level
 * @param condition condition dictating whether to log
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define VLOG_IF_WARN(condition, message, args...) \
    if (condition)                               \
    VLOG_WARN(message, ##args)
#else
#define LOG_WARN(message, args...)
#define VLOG_WARN(message, args...)
#define LOG_IF_WARN(condition, message, args...)
#define VLOG_IF_WARN(condition, message, args...)
#endif

#if LOG_MIN_SEVERITY <= SEVERITY_ERROR
/**
 * @brief Log message to console with `ERROR` severity level
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define LOG_ERROR(message, args...) LOG_COLOR(message, TAG_ERROR, COLOR_ERROR, ##args)

/**
 * @brief Log verbose message to console with `ERROR` severity level
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define VLOG_ERROR(message, args...) VLOG_COLOR(message, TAG_ERROR, COLOR_ERROR, ##args)

/**
 * @brief Conditionally log message to console with `ERROR` severity level
 * @param condition condition dictating whether to log
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define LOG_IF_ERROR(condition, message, args...) \
    if (condition)                                \
    LOG_ERROR(message, ##args)

/**
 * @brief Conditionally log verbose message to console with `ERROR` severity
 * level
 * @param condition condition dictating whether to log
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define VLOG_IF_ERROR(condition, message, args...) \
    if (condition)                                \
    VLOG_ERROR(message, ##args)
#else
#define LOG_ERROR(message, args...)
#define VLOG_ERROR(message, args...)
#define LOG_IF_ERROR(condition, message, args...)
#define VLOG_IF_ERROR(condition, message, args...)
#endif

#if LOG_MIN_SEVERITY <= SEVERITY_FATAL
/**
 * @brief Log message to console with `FATAL` severity level
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define LOG_FATAL(message, args...) LOG_COLOR(message, TAG_FATAL, COLOR_FATAL, ##args)

/**
 * @brief Log verbose message to console with `FATAL` severity level
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define VLOG_FATAL(message, args...) VLOG_COLOR(message, TAG_FATAL, COLOR_FATAL, ##args)

/**
 * @brief Conditionally log message to console with `FATAL` severity level
 * @param condition condition dictating whether to log
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define LOG_IF_FATAL(condition, message, args...) \
    if (condition)                                \
    LOG_FATAL(message, ##args)

/**
 * @brief Conditionally log verbose message to console with `FATAL` severity
 * level
 * @param condition condition dictating whether to log
 * @param message string to be formatted and logged
 * @param args string format arguments
 */
#define VLOG_IF_FATAL(condition, message, args...) \
    if (condition)                                \
    VLOG_FATAL(message, ##args)
#else
#define LOG_FATAL(message, args...)
#define VLOG_FATAL(message, args...)
#define LOG_IF_FATAL(condition, message, args...)
#define VLOG_IF_FATAL(condition, message, args...)
#endif

