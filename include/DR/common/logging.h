#pragma once

/**
@file
Once you've included this file, the suggested ways you should write log messages include:
<pre>
  DR::log()->trace("Some trace message: {} {}", something, some_other);
</pre>
Similarly, it provides:
<pre>
  DR::log()->debug(...);
  DR::log()->info(...);
  DR::log()->warn(...);
  DR::log()->error(...);
  DR::log()->critical(...);
</pre>
If you want to log objects that are expensive to serialize, these macros will not be compiled if debugging is turned off
(-DNDEBUG is set):
<pre>
  DR_LOG_TRACE(DR::log(), "message: {}", something_conditionally_compiled);
  DR_LOG_DEBUG(DR::log(), "message: {}", something_conditionally_compiled);
</pre>

The format string syntax is fmtlib; see https://fmt.dev/6.0.0/syntax.html. In particular, any class that overloads
`operator<<` for `ostream` can be printed without any special handling.
*/

#include <string>

#ifndef DR_DOXYGEN_CXX
// Before including spdlog, activate the SPDLOG_DEBUG and SPDLOG_TRACE macros
// if and only if DR is being compiled in debug mode.  When not in debug
// mode, they are no-ops and their arguments are not evaluated.
#ifndef NDEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#define DR_LOG_TRACE(logger, ...)      \
  do {                                 \
    DR_LOG_TRACE(logger, __VA_ARGS__); \
  } while (0)

#define DR_LOG_DEBUG(logger, ...)             \
  do {                                        \
    SPDLOG_LOGGER_DEBUG(logger, __VA_ARGS__); \
  } while (0)
#else
#define DR_LOG_TRACE(logger, ...)
#define DR_LOG_DEBUG(logger, ...)
#endif

/* clang-format off */
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>
/* clang-format on */

#undef SPDLOG_DEBUG
#undef SPDLOG_TRACE
#define SPDLOG_DEBUG SPDLOG_LOGGER_DEBUG
#define SPDLOG_TRACE SPDLOG_LOGGER_TRACE

#endif  // DR_DOXYGEN_CXX

namespace DR {

namespace logging {

/// The DR::logging::logger class provides text logging methods. See the logging.h documentation for a short tutorial.
using logger = spdlog::logger;

/// DR::logging::sink is an alias for spdlog::sinks::sink.
using spdlog::sinks::sink;

}  // namespace logging

/// Retrieve an instance of a logger to use for logging; for example:
/// <pre>
///   DR::log()->info("potato!")
/// </pre>
///
/// See the logging.h documentation for a short tutorial.
logging::logger* log();

namespace logging {

/// (Advanced) Retrieves the default sink for all DR logs. The return value can be cast to spdlog::sinks::dist_sink_mt
/// and thus allows DR consumers to redirect DR's text logs to locations other than the default of stderr.
sink* get_dist_sink();

/// When constructed, logs a message (at "warn" severity); the destructor is guaranteed to be trivial.  This is useful
/// for declaring an instance of this class as a function-static global, so that a warning is logged the first time the
/// program encounters some code, but does not repeat the warning on subsequent encounters within the same process.
///
/// For example:
/// <pre>
/// double* SanityCheck(double* data) {
///   if (!data) {
///     static const logging::Warn log_once("Bad data!");
///     return alternative_data();
///   }
///   return data;
/// }
/// </pre>
struct Warn {
  template <typename... Args>
  Warn(const char* a, const Args&... b) {
    DR::log()->warn(a, b...);
  }
};

/// Invokes `DR::log()->set_level(level)`.
/// @param level Must be a string from spdlog enumerations: `trace`, `debug`, `info`, `warn`, `err`, `critical`, `off`,
///              or `unchanged` (not an enum, but useful for command-line).
/// @return The string value of the previous log level.
std::string set_log_level(const std::string& level);

}  // namespace logging
}  // namespace DR
