# Changelog

All notable changes to this library. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow
[Semantic Versioning](https://semver.org/) (before 1.0, a minor version may
change the API).

## [0.3.0] - 2026-11-11

### Added

- `FasEngine`: a `StepEngine` on FastAccelStepper, with pulses and ramps in
  hardware on the ESP32.
- `EngineStepper<Engine>`: a `Stepper` that owns its engine, so one `new` builds
  a whole axis on any engine.
- `StepProfile::accelerationFor()`, so an engine that ramps in hardware can use
  the same profile.
- Per-axis soft limits (`setLimits()`), armed once the position is known.
- Hobby servos on the ESP32 through `ESP32Servo`.
- MIT license, GitHub Actions CI (native tests and example builds for Uno and
  ESP32), and guides in `docs/`.

### Fixed

- An endstop pressed right after a resync is now seen at once. Homing used to
  overshoot by up to 5 ms of travel when it started close to the switch.

## [0.2.0] - 2026-10-29

### Added

- `StepEngine` interface: a `Stepper` decides what the axis does, an engine makes
  the pulses. `PolledEngine` times them against `micros()` on any board.
- `Endstop` slots at each end of an axis, and two-pass homing (`homeAxis()`,
  `StepperManager::home()`).
- `StepProfile` speed profiles, with `LinearRamp`.
- `MotorManager<T>`, shared by `ServoManager` and the new `StepperManager`.
- Idle power release on a shared enable line.
- Compile-time log levels and a configurable output stream (`Log.h`).
- Native unit tests against a fake Arduino core, example sketches, and
  `scripts/check.sh`.

### Changed

- `Motor` is renamed `AngularMotor`, and its angle fields use camelCase.
- `StepMotor` and `HomedStepper` are replaced by the `Stepper` hierarchy and
  `Endstop`.

## [0.1.0] - 2026-09-03

- First version, extracted from the Ruka arm: servo motors (`Servo` and PCA9685)
  and a stepper, driven through a manager.

[0.3.0]: https://github.com/AndrewwSSS/MotorControl/releases/tag/v0.3.0
[0.2.0]: https://github.com/AndrewwSSS/MotorControl/compare/add262a...fd8cedc
[0.1.0]: https://github.com/AndrewwSSS/MotorControl/commit/add262a
