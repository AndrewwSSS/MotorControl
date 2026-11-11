# Logging

Everything the library prints goes through `Log.h`. Each project picks a level
in its `platformio.ini`:

```ini
build_flags = -D MC_LOG_LEVEL=MC_LOG_DEBUG
```

| Level | Prints |
| --- | --- |
| `MC_LOG_NONE` | nothing |
| `MC_LOG_ERROR` | failures, e.g. a homing search that did not find its switch |
| `MC_LOG_INFO` (default) | start-up: servos driven to their default angle, homing progress, a skipped seek pass |
| `MC_LOG_DEBUG` | every commanded target, for motors with `toggleDebug()` on |
| `MC_LOG_TRACE` | every step or degree moved, for motors with `toggleDebug()` on |

Messages above the chosen level are removed by the preprocessor, text and all.
`toggleDebug()` switches the DEBUG and TRACE messages per motor at runtime, but
only when the level compiles them in. TRACE prints once per pulse, which slows
the step loop down - keep it for the bench.

Output goes to `Serial` unless the project points it elsewhere. The stream is
not begun by the library; the sketch still calls `begin()` on it:

```ini
build_flags = -D MC_LOG_LEVEL=MC_LOG_INFO -D MC_LOG_STREAM=Serial1
```

Set both through `build_flags` rather than a `#define` in `main.cpp`, so every
translation unit sees the same values. Inside the library, log with
`MC_ERROR(F("text "), value, ...)` and its siblings: arguments are printed in
turn with `F()` text kept in flash, and no `String` is built.
