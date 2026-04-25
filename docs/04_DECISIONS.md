# 04 — Decisions Register

Decisions made during planning, with rationale. **Do not re-litigate these without checking with the user first.** Several "obvious" approaches were rejected for non-obvious reasons under time pressure.

If a decision below looks wrong to you, surface it as a question before changing course.

---

## D-001: Do not use the Raspberry Pi

**Decision:** Skip the Raspberry Pi entirely. Use only the V5 brain and standard V5 sensors.

**Rationale:**
- Setting up a Pi for this task would require: flashing the Pi, installing dependencies, configuring serial communication to the V5 brain, wiring the secondary battery (legal under VURC rules but adds complexity), and integrating Pi-side code with V5-side code.
- Conservative estimate: 4-6 hours of setup before any robot-relevant code is written.
- The AI Vision Sensor already does onboard AI classification. For a skills run, that perception is sufficient. The Pi would only earn its keep for tasks the AI Vision Sensor cannot do (SLAM, custom-trained models, complex multi-step decision-making) — none of which are in scope.
- Adding the Pi adds failure modes: another battery to manage, serial cable that can disconnect, Pi-side code that can crash.

**Status:** Locked. The Pi may be useful in future seasons; not for this 24-hour rush.

---

## D-002: Do not pursue full LemLib odometry

**Decision:** Use IMU for heading and motor encoders for distance, but do not configure full odometry tracking via LemLib's `OdomSensors` struct.

**Rationale:**
- Full LemLib odometry (with tracked pose: x, y, theta) requires either tracking wheels (which we do not have) or motor-encoder-only configuration (which drifts noticeably on every turn).
- Tuning lateral PID and angular PID to acceptable accuracy typically takes hours per controller. We do not have that time, especially with only 1-2 hours of field access.
- Most of the practical value of LemLib's odometry — accurate turns to absolute headings — comes from the IMU alone. We can get that benefit without tuning the rest.
- Vision-guided final approaches compensate for small position errors that odometry would otherwise correct. We get most of the same benefit through a different mechanism.

**Status:** Locked for this season. Tracking wheels and full odometry are a worthwhile project for a future season.

---

## D-003: AI Vision-first architecture for autonomous

**Decision:** Build the autonomous routine around vision-guided "see → align → approach" loops, with short open-loop drive segments between them, and IMU-based turns connecting everything.

**Rationale:**
- Closed-loop perception (vision) is the most reliable way to compensate for accumulated positional error in a routine that has no full odometry.
- The AI Vision Sensor is on hand. The Push Back classification model is pre-trained — no training cost.
- The architecture degrades gracefully: if vision fails on a segment, hard timeouts ensure the rest of the routine continues. Open-loop drive distance is the fallback for those segments.

**Status:** Locked.

---

## D-004: Do not replace the existing time-based auton wholesale

**Decision:** Build the new auton system in parallel with the existing time-based code. Switch the deployed routine over only when the new code is verified working.

**Rationale:**
- The existing auton "barely scores or often fails" but it does *something*. It drives, it tries to score. Removing it during development means there is no working baseline to fall back to.
- Compile errors or runtime errors in new code would otherwise mean a robot that does not move on the field — a worse outcome than barely scoring.
- The existing code is not "right" but it is "known". Known-bad beats unknown-untested in competition.

**Status:** Locked. Treat the existing time-based auton code as deletable — but only delete it when the new system is field-verified.

---

## D-005: Module structure — header + source pair per logical unit

**Decision:** Use standard C++ header + source pair structure for each module (e.g., `ai_vision.hpp` + `ai_vision.cpp`).

**Rationale:**
- User explicitly requested "Whatever's standard for PROS." PROS is C++; standard practice is header + source.
- Allows each module to be compiled independently, reducing rebuild times when only one module is touched.
- Cleaner separation between public API (header) and implementation details (source).

**Status:** Locked. Exception: very small modules (e.g., `config.hpp`) may be header-only with `constexpr`/`inline` if no implementation logic is needed.

---

## D-006: AI Vision wrapper enables only AI Object detection, not AprilTags or color blobs

**Decision:** The `ai_vision::Camera` wrapper calls `enable_detection_types(pros::AivisionModeType::objects)` and nothing else.

**Rationale:**
- Skills routine targets pre-trained game objects, not custom markers or colors.
- Enabling other detection modes would require additional filtering logic in the wrapper to separate object detections from tag/color detections.
- Keeping the surface area small reduces failure modes and simplifies debugging.

**Status:** Locked for this season. If a future need requires AprilTags (e.g., for absolute localization), extend the wrapper rather than reconfiguring globally.

---

## D-007: Use center coordinates internally, not raw top-left bounding box

**Decision:** The AI Vision wrapper translates the raw API's top-left bounding box coordinates into bounding box center coordinates before exposing them to callers.

**Rationale:**
- Every downstream calculation (centering on a target, judging proximity) is more naturally expressed with center coordinates.
- Doing the translation once in the wrapper avoids repeating the same `xoffset + width/2` calculation throughout the codebase.
- The cost is minimal (a few additions per detection) and the readability gain is substantial.

**Status:** Locked.

---

## D-008: x offset from center is normalized to [-1, +1]

**Decision:** `Detection::xOffsetNormalized()` returns a value in [-1, +1] rather than raw pixel offset.

**Rationale:**
- A normalized error signal is easier to tune a P-controller against — kP values are intuitive (e.g., "kP = 1.0 means full output at frame edge").
- Decouples controller tuning from the specific image resolution. If the camera's resolution ever changes, the controllers don't break.

**Status:** Locked.

---

## D-009: Do not pre-establish file names for unwritten modules

**Decision:** In planning docs, refer to upcoming modules by their purpose ("the primitives module", "the vision behaviors module") rather than by specific file names.

**Rationale:**
- User's stated preference: pre-establishing file structure in planning docs creates technical debt when implementation reveals a better organization.
- Specific file paths are decided when the file is created, not before.
- Exception: `ai_vision.hpp` and `ai_vision.cpp` are referenced by name because they are already written and committed.

**Status:** Locked. When writing future modules, check with the user on file name and location before creating the files.

---

## D-010: Mechanism actions stay open-loop time-based

**Decision:** Intake spin times, lift extension durations, scoring releases — all stay as time-based actions. Do not put closed-loop control on mechanism timing.

**Rationale:**
- Mechanisms are reliable in a way that drivetrain pose is not. An intake that spins for 2 seconds spins for 2 seconds. A drivetrain that drives for 2 seconds may end up anywhere.
- Adding closed-loop control to mechanisms would require additional sensors (limit switches, rotation sensors on lift arms) which are not on hand.
- Mechanism timing values can be tuned with the same calibration constants approach as everything else — single file, easy to tweak.

**Status:** Locked for this season.

---

## D-011: Single-port AI Vision Sensor, no multiple cameras

**Decision:** One AI Vision Sensor, forward-facing only.

**Rationale:**
- A second camera (e.g., rear-facing) would require additional code to switch between cameras and would double the configuration surface.
- One forward-facing camera is sufficient for "drive toward and engage with" behaviors, which is what the skills routine needs.

**Status:** Locked for this season.

---

## D-012: USB-C configuration of AI Vision must precede all software work

**Decision:** Before any code that uses the AI Vision Sensor is run, the sensor must be configured via USB-C and the AI Vision Utility (firmware up to date, AI Classification enabled, Push Back model selected, target classes verified).

**Rationale:**
- The AI Vision Utility persists configuration on the sensor itself. Code cannot select the model.
- An unconfigured sensor will silently return zero detections, which is hard to distinguish from a code bug.
- This step is in the critical path; doing it late wastes debugging time.

**Status:** Locked. Treat as a hard prerequisite for any field testing of vision-guided behaviors.
