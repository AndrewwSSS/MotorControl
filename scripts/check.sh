#!/usr/bin/env bash
# Everything MotorControl can check on its own: the native unit tests, and the
# example sketches compiled for each board they claim to support. Knows
# nothing about the projects that use the library - each of those has its own
# scripts/check.sh.
#
#   scripts/check.sh            from anywhere inside the repo
#   PIO=/path/to/pio scripts/check.sh

set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

# pio is often not on PATH when installed by the VS Code extension.
PIO="${PIO:-$(command -v pio || echo "$HOME/.platformio/penv/bin/pio")}"

# Every log line compiled in, so the TRACE paths are built on the boards too.
ci_opts=(
    --lib "$root/src"
    --project-option "build_flags = -D MC_LOG_LEVEL=MC_LOG_TRACE"
    --project-option "lib_ldf_mode = deep+"
)

step() { printf '\n=== %s ===\n' "$1"; }

step "Native tests"
"$PIO" test -e native

step "Example: Steppers (uno, esp32)"
"$PIO" ci examples/Steppers/Steppers.ino "${ci_opts[@]}" \
    --board uno --board esp32dev

step "Example: Servos (uno)"
"$PIO" ci examples/Servos/Servos.ino "${ci_opts[@]}" \
    --project-option "lib_deps = arduino-libraries/Servo@^1.3.0, adafruit/Adafruit PWM Servo Driver Library@^3.0.3" \
    --board uno

step "Example: Servos (esp32)"
"$PIO" ci examples/Servos/Servos.ino "${ci_opts[@]}" \
    --project-option "platform = espressif32@7.0.1" \
    --project-option "lib_deps = madhephaestus/ESP32Servo@^3.2.1, adafruit/Adafruit PWM Servo Driver Library@^3.0.3" \
    --board esp32dev

step "All green"
