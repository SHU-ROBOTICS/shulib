# 03 — Modules

Documentation for code modules in this project. Each module is described with its public API, file locations, dependencies, and current status.

## Module Status Overview

| Module | Files | Status |
|--------|-------|--------|
| AI Vision Wrapper | `include/ai_vision.hpp`, `src/ai_vision.cpp` | DONE — written, not yet tested |
| Config | TBD | NOT STARTED |
| Chassis Integration (IMU) | (existing chassis config file, edit pending) | NOT STARTED — blocked on user sharing existing config |
| Drive Primitives | TBD | NOT STARTED |
| Vision-Guided Behaviors | TBD | NOT STARTED |
| Skills Auton Routine | TBD | NOT STARTED |

## AI Vision Wrapper — DONE

### Files
- `include/ai_vision.hpp` — public interface
- `src/ai_vision.cpp` — implementation

### Purpose
A clean, typed wrapper around the raw `pros::AIVision` API. Provides convenient access to AI-classified detections with derived fields (center coordinates, normalized offsets, area).

### Why It Exists
The raw PROS API for the AI Vision Sensor is verbose: tagged-union-style structs, top-left bounding box coordinates, manual type-checking for object vs. tag vs. color blob. The wrapper handles all of that and presents a simpler, more direct interface for the rest of the autonomous code.

### Public API

```cpp
namespace ai_vision {

constexpr int IMAGE_WIDTH = 320;
constexpr int IMAGE_HEIGHT = 240;
constexpr int IMAGE_CENTER_X = 160;
constexpr int IMAGE_CENTER_Y = 120;

struct Detection {
    int classId;      // AI Classification class ID
    int xCenter;      // Center X of bounding box, in pixels
    int yCenter;      // Center Y of bounding box, in pixels
    int width;
    int height;
    int score;        // Confidence, 0..100

    double xOffsetNormalized() const;   // -1.0..+1.0, useful as P-controller input
    int area() const;                   // pixels, useful as distance proxy
};

class Camera {
public:
    explicit Camera(int port);
    bool initialize();
    std::optional<Detection> getLargestOfClass(int classId);
    std::vector<Detection> getAllOfClass(int classId);
    std::vector<Detection> getAll();
};

}
```

### Dependencies
- PROS C++ API (`pros/ai_vision.hpp`)
- Standard library: `<optional>`, `<vector>`, `<algorithm>` (in impl)
- **Does not depend on:** chassis, IMU, LemLib, or any other project module. Safe to test in isolation.

### Design Decisions Embedded
- Only enables `AivisionModeType::objects` — not AprilTags, not color blobs. If those are needed later, the wrapper must be extended.
- Translates raw top-left bounding box coordinates to center coordinates. All downstream math works with centers.
- `xOffsetNormalized()` divides by half-width so the result is `[-1, +1]`, which is more natural to tune a P-controller against than raw pixel offsets.
- Sorts results by area (largest first) because callers usually want "the most prominent detection".
- Returns `std::optional` for the "find one" case rather than a sentinel value or thrown exception. Callers must explicitly check.

### Usage Pattern

```cpp
ai_vision::Camera cam(VISION_PORT);
cam.initialize();   // call once during robot init

// In autonomous:
auto detection = cam.getLargestOfClass(TARGET_CLASS_ID);
if (detection) {
    double xErr = detection->xOffsetNormalized();
    int width = detection->width;
    // use xErr to drive turning, width to gauge distance
}
```

### Standalone Test (verifies wrapper without the rest of the system)

Drop into `opcontrol()` temporarily, plug camera in, point at a target:

```cpp
void opcontrol() {
    ai_vision::Camera cam(VISION_PORT);
    cam.initialize();

    while (true) {
        pros::screen::erase();
        auto detections = cam.getAll();
        pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Detections: %d", (int)detections.size());
        for (size_t i = 0; i < detections.size() && i < 4; i++) {
            const auto& d = detections[i];
            pros::screen::print(pros::E_TEXT_MEDIUM, i + 1,
                "id=%d w=%d off=%.2f", d.classId, d.width, d.xOffsetNormalized());
        }
        pros::delay(100);
    }
}
```

This test is intended to:
1. Confirm the wrapper compiles in the user's PROS setup.
2. Identify what class IDs target objects report as (write these down).
3. Identify what bounding box widths correspond to "good scoring distance" (write these down).
4. Catch lighting issues before competition.

### Known Constraints / Limitations
- Does not currently support AprilTag detection. Would require extending the wrapper.
- Does not currently support color blob detection. Would require extending the wrapper.
- Sensor configuration (which AI model is loaded) cannot be set from code; must be set via the AI Vision Utility on a computer.

---

## Config — NOT STARTED

### Intended Purpose
Centralize port assignments and tuning constants in one place. Includes:
- Port numbers for IMU, AI Vision, drive motors, mechanism motors.
- AI Vision class IDs for target objects (discovered in AI Vision Utility).
- Tuning constants: drive scale factor, turn kP, turn tolerance, vision approach width thresholds, default action timeouts.

### Why It Exists
Calibration at the field is fast when constants are in one file. Hunting for magic numbers across multiple files is slow and error-prone under time pressure.

### Open Questions
- Single header (`config.hpp`) with `constexpr` declarations only, or header + source pair? Recommendation: single header, `constexpr` and `inline constexpr` for all values, no source file needed.
- Does this file also include the `extern Camera cam;` global, or is that in a separate "globals" file? Recommendation: keep config purely declarative (constants only), put globals elsewhere.

### Decisions Pending
- File name and exact path. To be confirmed with user.
- Naming convention for constants (SCREAMING_SNAKE_CASE conventional in C++, follow whatever the existing codebase uses).

---

## Chassis Integration (IMU) — NOT STARTED

### Intended Purpose
Modify the existing LemLib chassis configuration to include the V5 Inertial Sensor.

### Why Not Yet Started
**Blocked on the user sharing the existing chassis configuration.** Without seeing the existing code, we cannot write the correct edit — LemLib chassis configurations vary in style (whether using `OdomSensors`, whether using `Drivetrain` directly, etc.).

### Anticipated Edit
Adding an IMU to a LemLib 0.5.x chassis typically means:
1. Declaring a `pros::Imu` (or `pros::IMU`) on the IMU's port.
2. Adding it to the `lemlib::OdomSensors` struct (or equivalent in the user's setup).
3. Calling `chassis.calibrate()` during robot initialization to zero the IMU.

But the exact code depends on the existing structure.

### Open Questions
- Does the existing chassis config use `lemlib::OdomSensors`?
- Where is `chassis.calibrate()` called currently (or is it called at all)?

---

## Drive Primitives — NOT STARTED

### Intended Purpose
Functions that replace the time-based driving and turning in the current auton:
- `turnToHeading(double headingDegrees, int timeoutMs)` — turn until IMU heading matches target.
- `driveDistance(double inches, int maxVoltage, int timeoutMs)` — drive forward/backward N inches using motor encoder average, with optional heading correction.
- `brake()` — stop and hold.

### Why It Exists
LemLib's full `moveToPoint` / `moveToPose` require accurate odometry, which requires either tracking wheels or careful tuning of motor-encoder-only odom. We have neither. Custom primitives that use the IMU directly for turning, and motor encoders directly for distance, are simpler and more reliable for this scope.

### Anticipated Implementation Approach
- `turnToHeading`: simple proportional controller on `target - currentHeading`, output to chassis tank drive (left = +output, right = -output), exits when error within tolerance for a few consecutive ticks OR timeout.
- `driveDistance`: motor encoder average compared to target encoder distance (converted from inches via wheel circumference). Optional heading correction term: `(startHeading - currentHeading) * kCorrection` applied as differential.
- `brake`: chassis tank with both sides at 0 voltage and brake mode set on motors.

### Decisions Pending
- Whether to use LemLib's existing `turnToHeading` (if its angular PID tunes well) or write our own.
- Whether to include heading correction in `driveDistance` from the start, or add it only if straight-line drift is observed.

---

## Vision-Guided Behaviors — NOT STARTED

### Intended Purpose
Compose the AI Vision wrapper and drive primitives into autonomous actions:
- `visionCenter(int classId, int timeoutMs)` — turn until target object is centered in frame.
- `visionApproach(int classId, int targetWidth, int timeoutMs)` — drive forward until target object's bounding box width >= threshold.
- `visionAcquire(int classId, int searchTimeoutMs, int approachTimeoutMs)` — combined: search by rotating, then center, then approach.

### Dependencies
- AI Vision Wrapper (DONE).
- Drive Primitives (NOT STARTED).
- Config (NOT STARTED) — for tuning constants.

### Critical Behavior Requirements
- All three behaviors MUST have hard timeouts. No infinite loops.
- All three behaviors MUST leave the robot stopped on exit, regardless of exit cause.
- All three behaviors should print to brain screen what they're doing for debugging.

---

## Skills Auton Routine — NOT STARTED

### Intended Purpose
The actual 60-second autonomous routine, composed of named segments that call primitives and behaviors.

### Dependencies
- All preceding modules.
- User's specific scoring strategy (which target objects, in which order, with which mechanism actions).

### Anticipated Structure
```cpp
void runSkills() {
    log_segment("init");
    chassis.calibrate();  // zero IMU
    // ... segments ...
    log_segment("done");
    brake();
}

void scoreFirstBlock() { ... }
void returnToStart() { ... }
// etc.
```

### Decisions Pending
- Final ordering of segments (depends on scoring strategy, which the user has confirmed exists but has not yet shared in detail).
- Which segments are vision-guided vs. open-loop.
- Mechanism action timing.
