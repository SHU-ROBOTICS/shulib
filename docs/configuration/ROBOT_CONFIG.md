# Robot Configuration

**Location:** `include/shulib/robots/robot_config.hpp` and individual robot files

---

## Table of Contents

1. [Overview](#overview)
2. [Configuration Architecture](#configuration-architecture)
3. [The Config Structs](#the-config-structs)
4. [Robot Selection](#robot-selection)
5. [DrivetrainConfig](#drivetrainconfig)
6. [TrackingConfig](#trackingconfig)
7. [MechanismConfig](#mechanismconfig)
8. [PneumaticConfig](#pneumaticconfig)
9. [Complete Robot Configuration](#complete-robotconfig)
10. [Our Robots](#our-robots)
11. [Adding a New Robot](#adding-a-new-robot)
12. [Configuration Best Practices](#configuration-best-practices)
13. [Troubleshooting Configuration](#troubleshooting-configuration)
14. [Key Takeaways](#key-takeaways)
15. [Where to Go Next](#where-to-go-next)

---

## Overview

### What is Robot Configuration?

Robot configuration is how we tell the code about the physical robot - what motors are where, how big the wheels are, where the sensors are mounted. Instead of hardcoding these values throughout the codebase, we centralize them in configuration files.

### Why Configuration Matters

**The Problem:**
```cpp
// BAD: Hardcoded values scattered everywhere
pros::Motor leftMotor1(12);
pros::Motor leftMotor2(-14);  // Why negative? Who knows!
// ... in another file ...
float wheelDiameter = 3.25;   // Inches? Centimeters? 
// ... in yet another file ...
float trackWidth = 15.0;      // Is this still accurate?
```

**The Solution:**
```cpp
// GOOD: All config in one place
const RobotConfig XEBEC = {
    .name = "XEBEC",
    .drivetrain = {
        .left_ports = {12, -14, 16, -18, 20},
        .right_ports = {11, -13, 15, -17, 19},
        .wheel_diameter = 3.25,  // inches
        .track_width = 15.0,     // inches
        // ...
    },
    // ...
};

// Usage is clean and self-documenting
chassis.setWheelDiameter(ROBOT.drivetrain.wheel_diameter);
```

### Benefits

| Benefit | Description |
|---------|-------------|
| **Single source of truth** | Change a value once, it updates everywhere |
| **Self-documenting** | Field names explain what values mean |
| **Easy robot switching** | Change one `#define` to switch robots |
| **Reduced errors** | No copy-paste mistakes |
| **Team-friendly** | Non-coders can understand and modify |

---

## Configuration Architecture

### The Layer Cake

```
┌─────────────────────────────────────────────────────────────────┐
│                        config.hpp                               │
│                   (Robot/Auton selection)                       │
│                                                                 │
│    #define ROBOT_XEBEC                                         │
│    // #define ROBOT_QUEENS_REVENGE                             │
├─────────────────────────────────────────────────────────────────┤
│                        main.cpp                                 │
│                   (Loads selected config)                       │
│                                                                 │
│    #if defined(ROBOT_XEBEC)                                    │
│        #include "shulib/robots/xebec.hpp"                      │
│        const auto& ROBOT = shulib::robots::XEBEC;              │
│    #elif defined(ROBOT_QUEENS_REVENGE)                         │
│        ...                                                      │
│    #endif                                                       │
├─────────────────────────────────────────────────────────────────┤
│                     Robot Config Files                          │
│          (xebec.hpp, queens_revenge.hpp, etc.)                  │
│                                                                 │
│    namespace shulib::robots {                                   │
│        inline const RobotConfig XEBEC = { ... };               │
│    }                                                            │
├─────────────────────────────────────────────────────────────────┤
│                     robot_config.hpp                            │
│                   (Base struct definitions)                     │
│                                                                 │
│    struct DrivetrainConfig { ... };                            │
│    struct TrackingConfig { ... };                              │
│    struct RobotConfig { ... };                                 │
└─────────────────────────────────────────────────────────────────┘
```

### File Locations

```
include/
└── shulib/
    └── robots/
        ├── robot_config.hpp      ← Base structs (don't edit often)
        ├── xebec.hpp             ← XEBEC's configuration
        └── queens_revenge.hpp    ← Queens Revenge's configuration

config.hpp                        ← Robot/auton selection (edit this!)
```

### How It Flows

1. **You edit `config.hpp`** to select which robot to build for
2. **main.cpp reads `config.hpp`** and includes the right robot file
3. **Robot file provides `ROBOT` constant** with all settings
4. **Code uses `ROBOT.xxx`** to access configuration values

---

## The Config Structs

### Struct Hierarchy

```cpp
RobotConfig
├── name: string
├── drivetrain: DrivetrainConfig
│   ├── left_ports: vector<int>
│   ├── right_ports: vector<int>
│   ├── wheel_diameter: float
│   ├── track_width: float
│   └── motor_rpm: int
├── tracking: TrackingConfig
│   ├── left_port: int
│   ├── right_port: int
│   ├── back_port: int
│   ├── wheel_diameter: float
│   ├── left_offset: float
│   ├── right_offset: float
│   └── back_offset: float
├── mechanisms: MechanismConfig
│   ├── intake_ports: vector<int>
│   ├── conveyor_ports: vector<int>
│   └── releaser_ports: vector<int>
└── pneumatics: PneumaticConfig
    ├── arm_port: char
    └── lever_port: char
```

### Base Definitions

```cpp
// robot_config.hpp

#pragma once
#include <string>
#include <vector>

namespace shulib::robots {

struct DrivetrainConfig {
    std::vector<int> left_ports;
    std::vector<int> right_ports;
    float wheel_diameter;  // inches
    float track_width;     // inches
    int motor_rpm;         // 100, 200, or 600
};

struct TrackingConfig {
    int left_port;
    int right_port;
    int back_port;
    float wheel_diameter;  // inches
    float left_offset;     // inches (negative = left of center)
    float right_offset;    // inches (positive = right of center)
    float back_offset;     // inches
};

struct MechanismConfig {
    std::vector<int> intake_ports;
    std::vector<int> conveyor_ports;
    std::vector<int> releaser_ports;
};

struct PneumaticConfig {
    char arm_port;    // ADI port letter
    char lever_port;  // ADI port letter
};

struct RobotConfig {
    std::string name;
    DrivetrainConfig drivetrain;
    TrackingConfig tracking;
    MechanismConfig mechanisms;
    PneumaticConfig pneumatics;
};

} // namespace shulib::robots
```

---

## Robot Selection

### The config.hpp File

```cpp
// config.hpp
#pragma once

// ╔═══════════════════════════════════════════════════════════════╗
// ║                     ROBOT SELECTION                           ║
// ║         Uncomment ONE robot to build for that robot           ║
// ╚═══════════════════════════════════════════════════════════════╝

#define ROBOT_XEBEC
// #define ROBOT_QUEENS_REVENGE
// #define ROBOT_TESTBOT

// ╔═══════════════════════════════════════════════════════════════╗
// ║                   AUTONOMOUS SELECTION                        ║
// ║        Uncomment ONE autonomous routine to use                ║
// ╚═══════════════════════════════════════════════════════════════╝

#define AUTON_SKILLS
// #define AUTON_RED_LEFT
// #define AUTON_RED_RIGHT
// #define AUTON_BLUE_LEFT
// #define AUTON_BLUE_RIGHT
// #define AUTON_TEST

// ╔═══════════════════════════════════════════════════════════════╗
// ║                     DEBUG OPTIONS                             ║
// ╚═══════════════════════════════════════════════════════════════╝

// #define DEBUG_PID
// #define DEBUG_ODOMETRY
#define ODOM_DISPLAY
```

### How main.cpp Uses It

```cpp
// main.cpp
#include "config.hpp"

// Load the selected robot configuration
#if defined(ROBOT_XEBEC)
    #include "shulib/robots/xebec.hpp"
    const auto& ROBOT = shulib::robots::XEBEC;
#elif defined(ROBOT_QUEENS_REVENGE)
    #include "shulib/robots/queens_revenge.hpp"
    const auto& ROBOT = shulib::robots::QUEENS_REVENGE;
#elif defined(ROBOT_TESTBOT)
    #include "shulib/robots/testbot.hpp"
    const auto& ROBOT = shulib::robots::TESTBOT;
#else
    #error "No robot selected! Define ROBOT_XEBEC or ROBOT_QUEENS_REVENGE in config.hpp"
#endif
```

### Switching Robots

To switch from XEBEC to Queens Revenge:

```cpp
// Before:
#define ROBOT_XEBEC
// #define ROBOT_QUEENS_REVENGE

// After:
// #define ROBOT_XEBEC
#define ROBOT_QUEENS_REVENGE
```

Then rebuild:
```bash
pros make clean && pros make
```

**That's it!** The entire codebase now uses Queens Revenge's configuration.

---

## DrivetrainConfig

### Purpose

Describes the drive motors - what ports they're on, wheel size, track width.

### Fields

```cpp
struct DrivetrainConfig {
    std::vector<int> left_ports;   // Left drive motor ports
    std::vector<int> right_ports;  // Right drive motor ports
    float wheel_diameter;          // Drive wheel diameter (inches)
    float track_width;             // Distance between left and right wheels (inches)
    int motor_rpm;                 // Motor cartridge RPM (100, 200, or 600)
};
```

### Field Details

#### `left_ports` / `right_ports`

**Type:** `std::vector<int>`

**What it is:** List of motor port numbers for each side of the drivetrain.

**Port number conventions:**
- **Positive** = Normal direction
- **Negative** = Reversed direction

```cpp
// Example: 5-motor left side
.left_ports = {12, -14, 16, -18, 20}
//             │    │   │    │   │
//             │    │   │    │   └── Port 20, normal
//             │    │   │    └── Port 18, reversed
//             │    │   └── Port 16, normal
//             │    └── Port 14, reversed
//             └── Port 12, normal
```

**Why reverse some motors?**

Motors on opposite sides of the robot face opposite directions. Motors on the same side might also face different directions depending on mounting:

```
    LEFT SIDE                RIGHT SIDE
    
    ┌─────┐                  ┌─────┐
    │ →── │  (normal)        │ ──← │  (normal for right)
    └─────┘                  └─────┘
    ┌─────┐                  ┌─────┐
    │ ──← │  (reversed)      │ →── │  (reversed for right)
    └─────┘                  └─────┘
```

**How to determine direction:**
1. Spin each motor forward (+100 power)
2. If wheel spins the wrong way, add negative to that port

---

#### `wheel_diameter`

**Type:** `float`

**Unit:** Inches

**What it is:** The diameter of your drive wheels.

**Common VEX wheel sizes:**
| Wheel | Diameter |
|-------|----------|
| 4" Omni | 4.0" |
| 3.25" Omni | 3.25" |
| 2.75" Omni | 2.75" |
| 4" Traction | 4.0" |

**How to measure:**
1. Remove a wheel from the robot
2. Measure across the widest part (including tread)
3. Use calipers for precision

**⚠️ Important:** Measure the ACTUAL diameter, not the nominal size. Wheels wear down!

---

#### `track_width`

**Type:** `float`

**Unit:** Inches

**What it is:** The distance between the left and right wheel centers.

```
    ◄─────────── track_width ───────────►
    
    ┌─────┐                      ┌─────┐
    │  L  │                      │  R  │
    │  ●  │◄────────────────────►│  ●  │
    │     │                      │     │
    └─────┘                      └─────┘
         center                 center
```

**How to measure:**
1. Measure from the center of the left wheel to the center of the right wheel
2. Measure at the axle level
3. If wheels are different widths on each side, use the average

**Why it matters:** Used for calculating turning radius and arc-based movements.

---

#### `motor_rpm`

**Type:** `int`

**What it is:** The RPM rating of your motor cartridges.

**Options:**
| Cartridge | Color | RPM | Torque |
|-----------|-------|-----|--------|
| 36:1 | Red | 100 | High |
| 18:1 | Green | 200 | Medium |
| 6:1 | Blue | 600 | Low |

**How to check:** Look at the cartridge color in your motors.

**Why it matters:** The code uses this to calculate expected speeds and for velocity control.

---

### Example DrivetrainConfig

```cpp
.drivetrain = {
    .left_ports = {12, -14, 16, -18, 20},  // 5 motors, alternating direction
    .right_ports = {11, -13, 15, -17, 19}, // 5 motors, alternating direction
    .wheel_diameter = 3.25,                 // 3.25" omni wheels
    .track_width = 15.0,                    // 15" between wheel centers
    .motor_rpm = 400                        // 400 RPM with gearing
}
```

---

## TrackingConfig

### Purpose

Describes the tracking wheels (dead wheels) used for odometry.

### Fields

```cpp
struct TrackingConfig {
    int left_port;        // Left tracking wheel rotation sensor port
    int right_port;       // Right tracking wheel rotation sensor port
    int back_port;        // Back tracking wheel rotation sensor port
    float wheel_diameter; // Tracking wheel diameter (inches)
    float left_offset;    // Left wheel distance from center (inches)
    float right_offset;   // Right wheel distance from center (inches)
    float back_offset;    // Back wheel distance from center (inches)
};
```

### Field Details

#### `left_port` / `right_port` / `back_port`

**Type:** `int`

**What it is:** The V5 port number for each rotation sensor.

**Port number conventions:**
- **Positive** = Sensor reads positive when robot moves forward
- **Negative** = Sensor is reversed

```cpp
.left_port = -8,   // Port 8, reversed
.right_port = 10,  // Port 10, normal
.back_port = 9,    // Port 9, normal
```

**How to determine direction:**
1. Push the robot forward
2. Read each sensor value
3. If value goes negative, add a minus sign to the port

---

#### `wheel_diameter` (tracking)

**Type:** `float`

**Unit:** Inches

**What it is:** The diameter of your tracking wheels.

**⚠️ Not the same as drive wheel diameter!** Tracking wheels are usually smaller.

**Common tracking wheel sizes:**
| Wheel | Diameter |
|-------|----------|
| 2.75" Omni | 2.75" |
| 2" Omni | 2.0" |

**Precision matters!** Tracking wheel diameter directly affects odometry accuracy. Measure carefully.

---

#### `left_offset` / `right_offset` / `back_offset`

**Type:** `float`

**Unit:** Inches

**What it is:** The distance from each tracking wheel to the robot's center of rotation.

```
                    Center of
                    Rotation
                       ↓
    ┌──────────────────●──────────────────┐
    │                  │                  │
    ├────┐             │             ┌────┤
    │ L  │◄─── left ───┼─── right ──►│  R │
    │    │   offset    │   offset    │    │
    ├────┘  (negative) │  (positive) └────┤
    │                  │                  │
    │            ┌─────┴─────┐            │
    │            │     B     │            │
    │            └─────┬─────┘            │
    │                  │                  │
    │            back_offset              │
    │            (+ if behind center)     │
    └─────────────────────────────────────┘
```

**Sign conventions:**
- `left_offset`: **Negative** (wheel is to the left of center)
- `right_offset`: **Positive** (wheel is to the right of center)
- `back_offset`: **Positive** if behind center, **Negative** if in front

**How to measure:**

1. **Find the center of rotation:**
   - Usually the geometric center of the drivetrain
   - If you spin the robot in place, this point doesn't move

2. **Measure perpendicular distance:**
   - From center to the tracking wheel's contact point
   - Measure perpendicular to the wheel's rolling direction

```
For left/right wheels:
                    Center
                       │
    ◄──────────────────┼──────────────────►
    │                  │                  │
    L                  │                  R
    wheel              │              wheel
                       │
    ◄───── 6.5" ──────►◄────── 6.5" ─────►
    
    left_offset = -6.5    right_offset = +6.5
```

---

### Example TrackingConfig

```cpp
.tracking = {
    .left_port = -8,          // Port 8, reversed
    .right_port = 10,         // Port 10, normal
    .back_port = 9,           // Port 9, normal
    .wheel_diameter = 2.75,   // 2.75" tracking wheels
    .left_offset = -6.5,      // 6.5" left of center
    .right_offset = 6.5,      // 6.5" right of center
    .back_offset = 0.0        // Centered (or adjust if not)
}
```

---

## MechanismConfig

### Purpose

Describes the motors for game-specific mechanisms (intake, conveyor, etc.).

### Fields

```cpp
struct MechanismConfig {
    std::vector<int> intake_ports;    // Intake motor(s)
    std::vector<int> conveyor_ports;  // Conveyor/indexer motor(s)
    std::vector<int> releaser_ports;  // Releaser/scorer motor(s)
};
```

### Field Details

#### `intake_ports`

**Type:** `std::vector<int>`

**What it is:** Motor port(s) for the intake mechanism.

```cpp
// Single motor intake
.intake_ports = {6}

// Dual motor intake (one reversed)
.intake_ports = {-6, 7}
```

#### `conveyor_ports`

**Type:** `std::vector<int>`

**What it is:** Motor port(s) for the conveyor/indexer.

```cpp
// Four motor conveyor
.conveyor_ports = {2, -3, -4, 5}
```

#### `releaser_ports`

**Type:** `std::vector<int>`

**What it is:** Motor port(s) for the releaser/scoring mechanism.

```cpp
// Single motor releaser
.releaser_ports = {1}

// Dual motor releaser
.releaser_ports = {-6, 7}
```

### Season-Specific

These fields are designed for the Push Back 2025-2026 game. Future seasons might need different mechanisms. The struct can be extended:

```cpp
struct MechanismConfig {
    // Push Back mechanisms
    std::vector<int> intake_ports;
    std::vector<int> conveyor_ports;
    std::vector<int> releaser_ports;
    
    // Future: add new mechanism fields here
    // std::vector<int> catapult_ports;
    // std::vector<int> lift_ports;
};
```

---

## PneumaticConfig

### Purpose

Describes pneumatic solenoid ports for mechanisms like arms and levers.

### Fields

```cpp
struct PneumaticConfig {
    char arm_port;    // ADI port for arm solenoid
    char lever_port;  // ADI port for lever solenoid
};
```

### Field Details

#### `arm_port` / `lever_port`

**Type:** `char`

**What it is:** The ADI (3-wire) port letter for each solenoid.

**Valid values:** `'A'` through `'H'`

```cpp
.pneumatics = {
    .arm_port = 'B',    // Arm solenoid on ADI port B
    .lever_port = 'C'   // Lever solenoid on ADI port C
}
```

### Using in Code

```cpp
// Create pneumatic objects from config
pros::ADIDigitalOut arm(ROBOT.pneumatics.arm_port);
pros::ADIDigitalOut lever(ROBOT.pneumatics.lever_port);

// Toggle
arm.set_value(true);   // Extend
arm.set_value(false);  // Retract
```

---

## Complete RobotConfig

### The Full Structure

```cpp
struct RobotConfig {
    std::string name;              // Human-readable robot name
    DrivetrainConfig drivetrain;   // Drive motor configuration
    TrackingConfig tracking;       // Odometry sensor configuration
    MechanismConfig mechanisms;    // Game mechanism configuration
    PneumaticConfig pneumatics;    // Pneumatic configuration
};
```

### Example: Complete XEBEC Configuration

```cpp
// xebec.hpp
#pragma once
#include "robot_config.hpp"

namespace shulib::robots {

inline const RobotConfig XEBEC = {
    .name = "XEBEC",
    
    .drivetrain = {
        .left_ports = {12, -14, 16, -18, 20},
        .right_ports = {11, -13, 15, -17, 19},
        .wheel_diameter = 3.25,
        .track_width = 15.0,
        .motor_rpm = 400
    },
    
    .tracking = {
        .left_port = -8,
        .right_port = 10,
        .back_port = 9,
        .wheel_diameter = 2.75,
        .left_offset = -6.5,
        .right_offset = 6.5,
        .back_offset = 0.0
    },
    
    .mechanisms = {
        .intake_ports = {-6, 7},
        .conveyor_ports = {2, -3, -4, 5},
        .releaser_ports = {1}
    },
    
    .pneumatics = {
        .arm_port = 'B',
        .lever_port = 'C'
    }
};

} // namespace shulib::robots
```

---

## Our Robots

### XEBEC

Our primary competition robot.

| Component | Configuration |
|-----------|---------------|
| **Drivetrain** | 10-motor tank (5L + 5R) |
| **Drive Ports** | L: 12, -14, 16, -18, 20 / R: 11, -13, 15, -17, 19 |
| **Drive Wheels** | 3.25" omnis |
| **Track Width** | 15.0" |
| **Motor RPM** | 400 |
| **Tracking Wheels** | 2.75" omnis |
| **Tracking Ports** | L: -8, R: 10, B: 9 |
| **Tracking Offsets** | L: -6.5", R: 6.5", B: 0" |
| **Intake** | Ports -6, 7 |
| **Conveyor** | Ports 2, -3, -4, 5 |
| **Releaser** | Port 1 |
| **Arm** | ADI B |
| **Lever** | ADI C |

### Queens Revenge

Our secondary competition robot.

| Component | Configuration |
|-----------|---------------|
| **Drivetrain** | 10-motor tank (5L + 5R) |
| **Drive Ports** | L: 11, -12, 13, -14, -15 / R: 16, -17, 18, -19, 20 |
| **Drive Wheels** | 3.25" omnis |
| **Track Width** | 15.0" |
| **Motor RPM** | 400 |
| **Tracking Wheels** | 2.75" omnis |
| **Tracking Ports** | L: -8, R: 10, B: 9 |
| **Tracking Offsets** | L: -6.5", R: 6.5", B: 2.5" |
| **Intake** | Ports 2, -3 |
| **Conveyor** | Ports 4, -5 |
| **Releaser** | Ports -6, 7 |
| **Arm** | ADI B |
| **Lever** | ADI C |

### Key Differences

| Aspect | XEBEC | Queens Revenge |
|--------|-------|----------------|
| Back offset | 0" | 2.5" |
| Intake ports | -6, 7 | 2, -3 |
| Conveyor ports | 2, -3, -4, 5 | 4, -5 |
| Releaser ports | 1 | -6, 7 |

---

## Adding a New Robot

### Step 1: Create the Config File

Create `include/shulib/robots/my_robot.hpp`:

```cpp
#pragma once
#include "robot_config.hpp"

namespace shulib::robots {

inline const RobotConfig MY_ROBOT = {
    .name = "My Robot",
    
    .drivetrain = {
        .left_ports = { /* your ports */ },
        .right_ports = { /* your ports */ },
        .wheel_diameter = 3.25,
        .track_width = 15.0,
        .motor_rpm = 200
    },
    
    .tracking = {
        .left_port = /* port */,
        .right_port = /* port */,
        .back_port = /* port */,
        .wheel_diameter = 2.75,
        .left_offset = -6.5,
        .right_offset = 6.5,
        .back_offset = 0.0
    },
    
    .mechanisms = {
        .intake_ports = { /* ports */ },
        .conveyor_ports = { /* ports */ },
        .releaser_ports = { /* ports */ }
    },
    
    .pneumatics = {
        .arm_port = 'A',
        .lever_port = 'B'
    }
};

} // namespace shulib::robots
```

### Step 2: Add to main.cpp

```cpp
// In the robot selection section of main.cpp:

#if defined(ROBOT_XEBEC)
    #include "shulib/robots/xebec.hpp"
    const auto& ROBOT = shulib::robots::XEBEC;
#elif defined(ROBOT_QUEENS_REVENGE)
    #include "shulib/robots/queens_revenge.hpp"
    const auto& ROBOT = shulib::robots::QUEENS_REVENGE;
#elif defined(ROBOT_MY_ROBOT)                           // ADD THIS
    #include "shulib/robots/my_robot.hpp"               // ADD THIS
    const auto& ROBOT = shulib::robots::MY_ROBOT;       // ADD THIS
#else
    #error "No robot selected!"
#endif
```

### Step 3: Add to config.hpp

```cpp
// In config.hpp:

// #define ROBOT_XEBEC
// #define ROBOT_QUEENS_REVENGE
// #define ROBOT_MY_ROBOT          // ADD THIS LINE
```

### Step 4: Test

1. Uncomment `#define ROBOT_MY_ROBOT` in config.hpp
2. Build: `pros make clean && pros make`
3. Upload and test each system

### Checklist for New Robot

```
□ All motor ports correct?
□ Motor directions correct? (test each one!)
□ Tracking sensor ports correct?
□ Tracking sensor directions correct?
□ Wheel diameters measured accurately?
□ Track width measured accurately?
□ Tracking offsets measured accurately?
□ Pneumatic ports correct?
□ Robot name set?
```

---

## Configuration Best Practices

### 1. Measure Precisely

```
Good:  wheel_diameter = 2.73   (measured with calipers)
Bad:   wheel_diameter = 2.75   (assumed from product name)
```

Odometry accuracy depends on correct measurements!

### 2. Document Your Ports

```cpp
.left_ports = {
    12,   // Front left - faces inward
    -14,  // Mid-front left - faces outward
    16,   // Center left - faces inward
    -18,  // Mid-back left - faces outward
    20    // Back left - faces inward
},
```

### 3. Test Each Motor Individually

Before trusting your config, verify each motor:

```cpp
void testMotors() {
    for (int port : ROBOT.drivetrain.left_ports) {
        printf("Testing port %d\n", abs(port));
        pros::Motor motor(port);
        motor.move(50);
        pros::delay(1000);
        motor.move(0);
        pros::delay(500);
    }
}
```

### 4. Version Your Configs

When you change something, note why:

```cpp
// v1.0 - Initial config
// v1.1 - Fixed left motor 3 direction (was backwards)
// v1.2 - Updated wheel diameter after wheel change (was 3.25, now 3.22)
```

### 5. Keep a Physical Reference

Document port assignments physically on the robot:

```
┌──────────────────────────────────┐
│  Motor Port Reference - XEBEC    │
├──────────────────────────────────┤
│  LEFT DRIVE    │  RIGHT DRIVE   │
│  12  (front)   │  11  (front)   │
│  14R (mid-f)   │  13R (mid-f)   │
│  16  (center)  │  15  (center)  │
│  18R (mid-b)   │  17R (mid-b)   │
│  20  (back)    │  19  (back)    │
├──────────────────────────────────┤
│  R = Reversed in software        │
└──────────────────────────────────┘
```

---

## Troubleshooting Configuration

### Problem: Robot Drives Wrong Direction

**Symptom:** Robot goes backward when commanded forward

**Fix:** All drive motors are probably reversed. Flip all the signs:

```cpp
// Before
.left_ports = {12, -14, 16}

// After  
.left_ports = {-12, 14, -16}
```

### Problem: Robot Spins Instead of Driving Straight

**Symptom:** Commanding forward makes robot spin in place

**Fix:** One side's motors are all reversed. Flip that side:

```cpp
// If left side is wrong:
.left_ports = {-12, 14, -16}  // Flip all signs
```

### Problem: Odometry Reads Backwards

**Symptom:** Robot moves forward, Y decreases

**Fix:** Tracking sensors need reversing:

```cpp
// Before
.left_port = 8

// After
.left_port = -8
```

### Problem: Odometry Drift on Turns

**Symptom:** Robot drifts when turning, not when straight

**Fix:** Tracking wheel offsets are probably wrong. Re-measure.

### Problem: Distances Are Off by a Constant Factor

**Symptom:** Robot goes 10% too far consistently

**Fix:** Wheel diameter is wrong. Recalibrate:

```cpp
// If going 10% too far:
// new_diameter = old_diameter * (expected / actual)
// new_diameter = 2.75 * (24 / 26.4) = 2.5
```

### Problem: Build Error "No robot selected"

**Symptom:** Compilation fails with error about robot selection

**Fix:** Uncomment exactly ONE robot in config.hpp:

```cpp
#define ROBOT_XEBEC           // Uncomment ONE
// #define ROBOT_QUEENS_REVENGE  // Leave others commented
```

---

## Key Takeaways

### The Essentials

1. **One file to change robots:** Edit `config.hpp`, rebuild
2. **All settings centralized:** No hunting through code
3. **Structs are self-documenting:** Field names explain everything
4. **Negative ports = reversed:** Simple direction control
5. **Measure precisely:** Especially wheel diameters and offsets

### Quick Reference

| Config | What It Controls |
|--------|------------------|
| `ROBOT.name` | Display name |
| `ROBOT.drivetrain.*` | Drive motors, wheel size |
| `ROBOT.tracking.*` | Odometry sensors |
| `ROBOT.mechanisms.*` | Game mechanisms |
| `ROBOT.pneumatics.*` | Solenoids |

### The One-Sentence Summary

> **Robot configuration centralizes all hardware-specific settings in one place, making it trivial to switch between robots or update port assignments.**

---

## Where to Go Next

| Topic | Document | What You'll Learn |
|-------|----------|-------------------|
| Tracking wheel setup | [TRACKING_WHEELS.md](./TRACKING_WHEELS.md) | Physical installation |
| Motor setup | [MOTOR_CONFIGURATION.md](./MOTOR_CONFIGURATION.md) | Motor details |
| Testing config | [CALIBRATION.md](./CALIBRATION.md) | Verifying your config |
| Using config | [CHASSIS.md](../core-library/CHASSIS.md) | How code uses config |

### Related Code Files

```
include/shulib/robots/
├── robot_config.hpp      ← Struct definitions
├── xebec.hpp             ← XEBEC config
└── queens_revenge.hpp    ← Queens Revenge config

config.hpp                ← Robot selection
src/main.cpp              ← Config loading
```

---

*Document last updated: January 2026*