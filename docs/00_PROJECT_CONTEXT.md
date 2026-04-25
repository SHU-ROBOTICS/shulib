# 00 — Project Context

## What This Project Is

A 24-hour rush implementation of a VEX U Programming Skills autonomous routine, replacing an existing time-based open-loop auton with one driven by an IMU and an AI Vision Sensor.

## Competition Context

- **Game:** Push Back (2025-2026 V5RC / VURC season)
- **Program:** VEX U Robotics Competition (VURC), college-level
- **Routine type:** Programming Skills — single robot, 60 seconds, fully autonomous, no driver input
- **Team:** Seton Hall University VEX Robotics
- **Programmer:** Working on this codebase

## Hardware

### On the robot, currently
- V5 Robot Brain
- V5 Smart Motors (with built-in encoders) — drivetrain and mechanisms
- AI Vision Sensor (276-8659, V5 version) — confirmed correct part
- All wiring for above

### Being added tonight
- V5 Inertial Sensor (IMU) — being physically acquired

### Available but explicitly NOT being used
- Raspberry Pi — rejected for this scope, see `04_DECISIONS.md`

### Not available, not in scope
- Tracking wheels — none on the robot, none being added
- Secondary battery — none
- External coprocessor — none
- Custom electronics — none

## Software

- **Language:** C++
- **Framework:** PROS (Purdue Robotics Operating System) for V5
- **Library:** LemLib 0.5.x (currently configured but not actively used by the auton, since current auton is sensorless)
- **Project structure:** Standard PROS layout — `src/`, `include/`, etc.

## State of the Codebase

- Existing chassis configuration in LemLib exists and works for driver control. Not yet shared with the assistant.
- Existing autonomous is time-based, open-loop, sensorless. Performs poorly — "barely scores or often fails" by the user's own assessment.
- AI Vision Sensor wrapper module has been written but not yet tested. See `03_MODULES.md`.

## Timeline and Constraints

- **Hard deadline:** Competition tomorrow (relative to start of conversation).
- **Total work time:** ~24 hours.
- **Field access:** Approximately 1-2 hours total tonight. After that, no field testing until competition day.
- **Sleep:** The user is doing this 24 hours straight. Code will degrade in quality as the night progresses. Doc clarity matters.

## Goal

A skills run that is meaningfully more reliable than the current time-based auton. We are not chasing a record-breaking score. We are chasing a routine that:
- Doesn't drift unpredictably between segments (IMU solves this).
- Compensates for small starting position errors (AI Vision solves this).
- Can be calibrated quickly during the limited field access window.
- Has a fallback path if vision-guided segments fail.

## Stretch Goals (Probably Not Hitting)

- Full LemLib odometry with tracked pose. Out of scope without tracking wheels.
- AprilTag-based localization. Possible with current sensor but adds complexity.
- Custom-trained vision model. Out of scope, pre-trained Push Back model is sufficient.
- Pure pursuit / motion profiling. Out of scope.

## Non-Goals

- Building a clean, reusable, season-long codebase. This is rush code.
- Tomorrow's match auton (different rules, alliance partners, opposing robots). Skills only.
- Driver control improvements.
- Any cosmetic or aesthetic concerns.

## Rules Compliance Notes

- The AI Vision Sensor (276-8659) is fully legal in V5RC, VURC, and VAIRC programs.
- VEX U (VURC) allows additional electronics under rule VUR12, with the constraint that they cannot directly electrically interface with VEX motors. Our setup uses only standard V5 sensors connected to the V5 brain via Smart Cables, so this is moot for us.
- All code must be the team's own per VURC student-centered policy. AI assistance during development is an accepted part of programming workflow; the user is the one making decisions and writing/reviewing the code.
