#pragma once
#include <Arduino.h>

// Compile-time log levels, set with build_flags = -D MC_LOG_LEVEL=...; messages
// above the level are compiled out. See docs/logging.md.
#define MC_LOG_NONE  0
#define MC_LOG_ERROR 1   // something failed and the caller should know
#define MC_LOG_INFO  2   // start-up and one-off events
#define MC_LOG_DEBUG 3   // per-command chatter, also gated per motor by toggleDebug()
#define MC_LOG_TRACE 4   // per-step chatter, also gated per motor by toggleDebug()

#ifndef MC_LOG_LEVEL
#define MC_LOG_LEVEL MC_LOG_INFO
#endif

// Output stream (any Print). The sketch must begin() it.
#ifndef MC_LOG_STREAM
#define MC_LOG_STREAM Serial
#endif

namespace mc_log {

inline void printAll() {
    MC_LOG_STREAM.println();
}

// Prints each argument, then a newline. Pass F("...") text, not Strings, to
// avoid heap fragmentation on AVR.
template <typename T, typename... Rest>
inline void printAll(const T& first, const Rest&... rest) {
    MC_LOG_STREAM.print(first);
    printAll(rest...);
}

}

#if MC_LOG_LEVEL >= MC_LOG_ERROR
#define MC_ERROR(...) mc_log::printAll(__VA_ARGS__)
#else
#define MC_ERROR(...) do {} while (0)
#endif

#if MC_LOG_LEVEL >= MC_LOG_INFO
#define MC_INFO(...) mc_log::printAll(__VA_ARGS__)
#else
#define MC_INFO(...) do {} while (0)
#endif

#if MC_LOG_LEVEL >= MC_LOG_DEBUG
#define MC_DEBUG(...) mc_log::printAll(__VA_ARGS__)
#else
#define MC_DEBUG(...) do {} while (0)
#endif

#if MC_LOG_LEVEL >= MC_LOG_TRACE
#define MC_TRACE(...) mc_log::printAll(__VA_ARGS__)
#else
#define MC_TRACE(...) do {} while (0)
#endif
