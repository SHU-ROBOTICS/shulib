# Architecture Overview

This document explains the high-level design of shulib and the reasoning behind its organization.

---

## Table of Contents

- [Design Goals](#design-goals)
- [Core Principles](#core-principles)
- [Layer Architecture](#layer-architecture)
- [Key Components](#key-components)
- [How Components Interact](#how-components-interact)
- [Why This Design](#why-this-design)

---

## Design Goals

When designing shulib, we optimized for:

### 1. Multi-Robot Support

At competitions, teams often have multiple robots. We needed to:
- Switch between robots by changing **one line of code**
- Keep robot-specific details **isolated from core logic**
- Support robots with **completely different configurations**

### 2. Season Separation

VEX games change every year. We wanted to:
- Keep the **core library reusable** across seasons
- Isolate game-specific code so it can be **archived or replaced**
- Let next year's team **start fresh** without losing the foundation

### 3. Maintainability

Robotics teams have high turnover. We designed for:
- Code that's **readable by newcomers**
- **Clear boundaries** between components
- **Extensive documentation** at every level

### 4. Reliability

Competition code must work. We prioritized:
- **Simple, proven algorithms** over complex ones
- **Explicit configuration** over magic auto-detection
- **Fail-safe defaults** and clear error messages

---

## Core Principles

### Configuration as Single Source of Truth

Every robot-specific value lives in ONE place: the robot config file.
```cpp
// All of TestBot's details in one file
inline const RobotConfig TESTBOT = {
    .name = "TestBot",
    .drivetrain = {
        .left_ports = {-16, 17, -18, 19, -20},
        .right_ports = {11, -12, 13, -14, 15},
        // ...
    },
    // ...
};
```

**No magic numbers scattered in code.** If you need to change a port, you change it in one place.

### Layers of Abstraction

Each layer knows as little as possible about layers below it:

- **Autonomous code** calls `chassis.drive()` – doesn't know about motors
- **Chassis** calls `drivetrain.drive()` – doesn't know about specific motor groups
- **Drivetrain** calls `motors.move()` – doesn't know about PID or odometry

This means you can change lower layers without breaking upper layers.

### Compile-Time Configuration

We use `#define` for robot selection instead of runtime switches:
```cpp
#define ROBOT_TESTBOT  // Selected at compile time
```

**Benefits:**
- No code for unused robots in the binary
- Errors caught at compile time, not runtime
- Zero runtime overhead

**Tradeoff:** Must recompile to switch robots (but compilation is fast).

---

## Layer Architecture
```
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│                        config.hpp                           │
│                                                             │
│   The "control panel" - select robot and autonomous here    │
│                                                             │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│                         main.cpp                            │
│                                                             │
│   The "wiring closet" - creates objects, connects things    │
│                                                             │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│                   seasons/pushback_2026/                    │
│                                                             │
│   The "game plan" - autonomous routines, driver controls    │
│                                                             │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│                         robots/                             │
│                                                             │
│   The "spec sheets" - what ports, dimensions each robot has │
│                                                             │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│                          core/                              │
│                                                             │
│   The "engine" - chassis, odometry, PID, motion control     │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Layer Rules

1. **Data flows down** - upper layers pass config/commands to lower layers
2. **Upper layers use lower layers**, never the reverse
3. **Core knows nothing about seasons** - it's game-agnostic
4. **Seasons know nothing about specific robots** - they work with any robot

---

## Key Components

### Chassis

The main interface for controlling the robot.
```cpp
class Chassis {
public:
    void calibrate();                    // Initialize sensors
    void drive(int h, int v, int turn);  // Move the robot
    Pose getPose();                      // Get current position
    void setPose(float x, float y, float theta);  // Set position
};
```

**Chassis is the abstraction layer.** Upper code talks to Chassis, not directly to motors.

### Drivetrain

Handles motor control for specific drive configurations.
```cpp
class Drivetrain {
public:
    virtual void drive(int h, int v, int turn);
    virtual void setBrakeMode(pros::motor_brake_mode_e mode);
};

class TankDrive : public Drivetrain {
    // Left motors: vertical + turn
    // Right motors: vertical - turn
};

class XDrive : public Drivetrain {
    // All four motors with different coefficients
};
```

**Drivetrain is pluggable.** Swap TankDrive for XDrive without changing upper layers.

### Odometry

Tracks robot position using wheel encoders.
```cpp
// Runs in background task, updates continuously
Pose getPose();  // Returns current X, Y, Theta

// Position is calculated from:
// - Left tracking wheel (forward/back movement)
// - Right tracking wheel (forward/back movement)
// - Back tracking wheel (sideways drift)
```

**Odometry is independent of drive motors.** Uses dedicated tracking wheels for accuracy.

### PID Controller

Generic proportional-integral-derivative controller.
```cpp
class PID {
public:
    PID(double kP, double kI, double kD);
    double update(double error, double deltaTime);
    void reset();
};
```

**PID is reusable.** Same class used for turns, driving straight, heading correction.

### RobotConfig

Struct containing all robot-specific values.
```cpp
struct RobotConfig {
    std::string name;
    DrivetrainConfig drivetrain;  // Motor ports, dimensions
    TrackingConfig tracking;      // Tracking wheel setup
    MechanismsConfig mechanisms;  // Intake, conveyor, etc.
};
```

**RobotConfig is the single source of truth.** Everything about a robot in one place.

### Mechanisms

Season-specific subsystems (intake, conveyor, etc.).
```cpp
class Mechanisms {
public:
    Mechanisms(const MechanismsConfig& config);
    
    void intakeIn();
    void intakeOut();
    void intakeStop();
    
    void conveyorUp();
    void conveyorDown();
    void conveyorStop();
    
    void toggleArm();
    // ...
};
```

**Mechanisms are configured, not hardcoded.** Ports come from RobotConfig.

---

## How Components Interact

### Initialization Flow
```
config.hpp
    │
    │  #define ROBOT_TESTBOT
    ▼
main.cpp
    │
    │  #include "shulib/robots/testbot.hpp"
    │  const auto& ROBOT = shulib::robots::TESTBOT;
    │
    │  // Create motor groups from config
    │  pros::MotorGroup leftMotors(ROBOT.drivetrain.left_ports);
    │  pros::MotorGroup rightMotors(ROBOT.drivetrain.right_ports);
    │
    │  // Create drivetrain from motor groups
    │  shulib::TankDrive drivetrain(leftMotors, rightMotors, ...);
    │
    │  // Create chassis from drivetrain + sensors
    │  shulib::Chassis chassis(drivetrain, sensors);
    ▼
Chassis is ready to use
```

### Motion Command Flow
```
auton.cpp: moveVertical(chassis, 24)
    │
    │  // Get current position
    │  Pose current = chassis.getPose();
    │
    │  // Calculate error
    │  double error = target - traveled;
    │
    │  // PID calculates power
    │  double power = pid.update(error, dt);
    │
    │  // Send to chassis
    │  chassis.drive(0, power, correction);
    ▼
chassis.cpp: drive(h, v, turn)
    │
    │  // Forward to drivetrain
    │  drivetrain.drive(h, v, turn);
    ▼
drivetrain.cpp: drive(h, v, turn)
    │
    │  // Apply coefficients for each motor group
    │  // Left: h*0 + v*1 + turn*1
    │  // Right: h*0 + v*1 + turn*(-1)
    │
    │  leftMotors.move(v + turn);
    │  rightMotors.move(v - turn);
    ▼
Motors move
```

### Position Update Flow (Background)
```
Odometry task (runs every 10ms)
    │
    │  // Read tracking wheels
    │  double leftDelta = leftWheel.getDistanceDelta();
    │  double rightDelta = rightWheel.getDistanceDelta();
    │  double backDelta = backWheel.getDistanceDelta();
    │
    │  // Calculate movement
    │  double forward = (leftDelta + rightDelta) / 2;
    │  double strafe = backDelta;
    │  double turn = (rightDelta - leftDelta) / trackWidth;
    │
    │  // Apply rotation matrix for field-centric position
    │  // Update global X, Y, Theta
    ▼
chassis.getPose() returns updated position
```

---

## Why This Design

### Why Separate Configs from Code?

**Problem:** Magic numbers scattered everywhere.
```cpp
// Bad: port numbers all over the code
pros::Motor leftFront(1);
pros::Motor leftBack(-2);  // Why negative? Who knows!
```

**Solution:** Centralized configuration.
```cpp
// Good: all in one place, documented
.left_ports = {1, -2, 3}  // Negative = reversed motor
```

### Why Compile-Time Robot Selection?

**Problem:** Runtime selection adds complexity and failure modes.
```cpp
// Bad: runtime selection
if (robotName == "TestBot") { ... }
else if (robotName == "XEBEC") { ... }  // Typo = runtime crash
```

**Solution:** Compile-time selection with `#define`.
```cpp
// Good: compiler catches errors
#define ROBOT_TESTBOT  // Typo = compile error
```

### Why No IMU?

**Problem:** IMU calibration is slow and sometimes fails.

**Solution:** 3-wheel odometry is:
- More reliable (no calibration drift)
- Faster to initialize
- Simpler to debug

The architecture supports adding IMU later if needed.

### Why JSON Logging?

**Problem:** Debug output is hard to parse and analyze.
```
X: 12.34, Y: 56.78, Theta: 90.12  // Hard to parse programmatically
```

**Solution:** JSON format.
```json
{"odometry": {"x": 12.34, "y": 56.78, "theta": 90.12}}
```

- Easy to parse with any language
- Self-documenting (field names)
- Tools can visualize it

---

## Related Documents

- [Layer Diagram](LAYER_DIAGRAM.md) – Visual component diagram
- [Data Flow](DATA_FLOW.md) – Detailed data flow diagrams
- [Design Decisions](DESIGN_DECISIONS.md) – More "why" explanations
- [Project Structure](../getting-started/PROJECT_STRUCTURE.md) – File layout