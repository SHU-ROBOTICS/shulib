# Project Structure

This document explains what's in each folder and file, and why it's organized this way.

---

## Table of Contents

- [Design Philosophy](#design-philosophy)
- [Directory Overview](#directory-overview)
- [Key Files](#key-files)
- [Folders Explained](#folders-explained)
- [Layer Diagram](#layer-diagram)
- [Adding New Things](#adding-new-things)
- [File Naming Conventions](#file-naming-conventions)

---

## Design Philosophy

The project is organized around three core principles:

### 1. Separation of Concerns

Each folder has one job:
- `core/` – Reusable library code
- `robots/` – Robot-specific configurations
- `seasons/` – Game-specific code

### 2. Configuration Over Code

Change behavior by editing configs, not core code:
- Switch robots by editing `config.hpp`
- Switch autonomous by editing `config.hpp`
- Add robots by creating config files, not modifying library code

### 3. Season Isolation

Game-specific code doesn't pollute the reusable library:
- Core library works for any game
- Season code can be archived or replaced each year
- Next year's team starts with a working foundation

---

## Directory Overview
```
shulib/
├── config.hpp                     # ← EDIT THIS to switch robots/autos
│
├── include/                       # Header files (.hpp)
│   ├── main.h                     # PROS main header
│   ├── api.h                      # PROS API
│   └── shulib/                    # Our library headers
│       ├── core/                  # Reusable library
│       │   ├── chassis.hpp
│       │   ├── drivetrain.hpp
│       │   ├── drivetrain/
│       │   │   ├── tankdrive.hpp
│       │   │   └── xdrive.hpp
│       │   ├── odometry.hpp
│       │   ├── odomUnit.hpp
│       │   ├── pid.hpp
│       │   ├── pose.hpp
│       │   ├── logger.hpp
│       │   └── util.hpp
│       │
│       ├── robots/                # Robot configurations
│       │   ├── robot_config.hpp   # Config struct definitions
│       │   ├── xebec.hpp
│       │   ├── queens_revenge.hpp
│       │   └── testbot.hpp
│       │
│       └── seasons/               # Season-specific headers
│           └── pushback_2026/
│               ├── auton.hpp
│               ├── mechanisms.hpp
│               └── opcontrol.hpp
│
├── src/                           # Implementation files (.cpp)
│   ├── main.cpp                   # Entry point
│   │
│   ├── core/                      # Core library implementations
│   │   ├── chassis.cpp
│   │   ├── drivetrain.cpp
│   │   ├── odometry.cpp
│   │   ├── odomUnit.cpp
│   │   ├── pid.cpp
│   │   ├── pose.cpp
│   │   ├── logger.cpp
│   │   └── util.cpp
│   │
│   └── seasons/                   # Season implementations
│       └── pushback_2026/
│           ├── auton.cpp
│           ├── mechanisms.cpp
│           └── opcontrol.cpp
│
├── docs/                          # Documentation (you are here!)
│   ├── README.md
│   ├── getting-started/
│   ├── architecture/
│   ├── core-library/
│   ├── configuration/
│   ├── motion/
│   ├── seasons/
│   ├── troubleshooting/
│   ├── reference/
│   ├── competition/
│   └── contributing/
│
├── firmware/                      # PROS firmware (don't edit!)
├── Makefile                       # Build configuration
├── project.pros                   # PROS project file
├── common.mk                      # Shared make settings
├── LICENSE                        # MIT License
└── README.md                      # Project README (links to docs/)
```

---

## Key Files

### `config.hpp`

**Location:** Project root

**Purpose:** Single place to select robot and autonomous.
```cpp
// ═══════════════════════════════════════════════════════════
// ROBOT SELECTION - Uncomment ONE
// ═══════════════════════════════════════════════════════════
// #define ROBOT_XEBEC
// #define ROBOT_QUEENS_REVENGE
#define ROBOT_TESTBOT

// ═══════════════════════════════════════════════════════════
// AUTONOMOUS SELECTION - Uncomment ONE
// ═══════════════════════════════════════════════════════════
// #define AUTON_SKILLS
#define AUTON_TEST
```

**When to edit:** Every time you switch robots or change autonomous.

---

### `src/main.cpp`

**Location:** `src/main.cpp`

**Purpose:** Entry point, initialization, global objects.

**What it does:**
1. Includes the selected robot config based on `config.hpp`
2. Creates motor groups, sensors, drivetrain, chassis
3. Implements PROS callbacks: `initialize()`, `autonomous()`, `opcontrol()`

**When to edit:** When adding a new robot to the selection logic.
```cpp
// Robot selection in main.cpp
#if defined(ROBOT_XEBEC)
    #include "shulib/robots/xebec.hpp"
    const auto& ROBOT = shulib::robots::XEBEC;
#elif defined(ROBOT_TESTBOT)
    #include "shulib/robots/testbot.hpp"
    const auto& ROBOT = shulib::robots::TESTBOT;
// Add new robots here...
#endif
```

---

### `include/shulib/robots/robot_config.hpp`

**Location:** `include/shulib/robots/robot_config.hpp`

**Purpose:** Defines the configuration structs.
```cpp
struct DrivetrainConfig {
    std::vector<int> left_ports;
    std::vector<int> right_ports;
    float track_width;
    float wheel_diameter;
    float rpm;
};

struct TrackingConfig {
    int left_port;
    int right_port;
    int back_port;
    float wheel_diameter;
    float left_offset;
    float right_offset;
    float back_offset;
};

struct MechanismsConfig {
    std::vector<int> intake_ports;
    std::vector<int> conveyor_ports;
    std::vector<int> releaser_ports;
    char arm_adi_port;
    char lever_adi_port;
};

struct RobotConfig {
    std::string name;
    DrivetrainConfig drivetrain;
    TrackingConfig tracking;
    MechanismsConfig mechanisms;
};
```

**When to edit:** When you need new config options for all robots.

---

## Folders Explained

### `include/shulib/core/`

**Purpose:** Reusable library code that works year-to-year, game-to-game.

| File | Description |
|------|-------------|
| `chassis.hpp` | Main robot interface (drive, getPose, setPose) |
| `drivetrain.hpp` | Base drivetrain class |
| `drivetrain/tankdrive.hpp` | Tank drive implementation |
| `drivetrain/xdrive.hpp` | X-drive implementation |
| `odometry.hpp` | Position tracking system |
| `odomUnit.hpp` | Single tracking wheel class |
| `pid.hpp` | PID controller |
| `pose.hpp` | Position (X, Y, Theta) class |
| `logger.hpp` | Telemetry and debugging |
| `util.hpp` | Utility functions |

**When to edit:** Rarely. Only when improving core functionality.

---

### `include/shulib/robots/`

**Purpose:** Robot-specific configurations.

| File | Description |
|------|-------------|
| `robot_config.hpp` | Struct definitions for configs |
| `xebec.hpp` | XEBEC robot configuration |
| `queens_revenge.hpp` | Queens Revenge configuration |
| `testbot.hpp` | TestBot configuration |

**When to edit:** When adding a new robot or changing port assignments.

**Example robot config:**
```cpp
// testbot.hpp
inline const RobotConfig TESTBOT = {
    .name = "TestBot",
    .drivetrain = {
        .left_ports = {-16, 17, -18, 19, -20},
        .right_ports = {11, -12, 13, -14, 15},
        .track_width = 15.0,
        .wheel_diameter = 3.25,
        .rpm = 400
    },
    // ...
};
```

---

### `include/shulib/seasons/`

**Purpose:** Season-specific (game-specific) code headers.
```
seasons/
└── pushback_2026/
    ├── auton.hpp        # Autonomous function declarations
    ├── mechanisms.hpp   # Mechanism class definition
    └── opcontrol.hpp    # Driver control declarations
```

**When to edit:** Every season when the game changes.

---

### `src/core/`

**Purpose:** Implementation of core library classes.

| File | Description |
|------|-------------|
| `chassis.cpp` | Chassis method implementations |
| `drivetrain.cpp` | Drivetrain base class methods |
| `odometry.cpp` | Position tracking math |
| `odomUnit.cpp` | Tracking wheel handling |
| `pid.cpp` | PID algorithm |
| `pose.cpp` | Pose math operations |
| `logger.cpp` | JSON logging implementation |
| `util.cpp` | Utility function implementations |

**When to edit:** When fixing bugs or improving core algorithms.

---

### `src/seasons/`

**Purpose:** Implementation of season-specific code.
```
seasons/
└── pushback_2026/
    ├── auton.cpp        # Autonomous routines
    ├── mechanisms.cpp   # Mechanism control
    └── opcontrol.cpp    # Driver control logic
```

**When to edit:** When writing autonomous routines or changing controls.

---

### `docs/`

**Purpose:** All documentation (you're reading it!).

See the [main README](../README.md) for the full documentation index.

**When to edit:** When adding features, fixing issues, or clarifying existing docs.

---

### `firmware/`

**Purpose:** PROS firmware files.

⚠️ **Never edit these files!** They're managed by PROS.

---

## Layer Diagram
```
┌─────────────────────────────────────────────────────────────┐
│                        config.hpp                           │
│                  (Robot & Autonomous Selection)             │
│                                                             │
│   "I want to use TestBot with the test autonomous"          │
├─────────────────────────────────────────────────────────────┤
│                         main.cpp                            │
│                    (Initialization)                         │
│                                                             │
│   "Create motors, sensors, chassis based on config"         │
├─────────────────────────────────────────────────────────────┤
│                     seasons/pushback_2026/                  │
│              (auton.cpp, opcontrol.cpp, mechanisms.cpp)     │
│                                                             │
│   "Here's how to play this year's game"                     │
├─────────────────────────────────────────────────────────────┤
│                          robots/                            │
│         (xebec.hpp, queens_revenge.hpp, testbot.hpp)        │
│                                                             │
│   "Here's what ports and dimensions each robot has"         │
├─────────────────────────────────────────────────────────────┤
│                           core/                             │
│       (chassis, drivetrain, odometry, pid, logger)          │
│                                                             │
│   "Here's how to track position and control motors"         │
└─────────────────────────────────────────────────────────────┘

        Data flows DOWN. Upper layers use lower layers.
              Lower layers know nothing about upper.
```

---

## Adding New Things

| To Add... | Create/Edit These Files |
|-----------|-------------------------|
| New robot | 1. Create `include/shulib/robots/myrobot.hpp`<br>2. Add to `main.cpp` selection<br>3. Add to `config.hpp` options |
| New autonomous routine | 1. Add declaration to `auton.hpp`<br>2. Add implementation to `auton.cpp`<br>3. Add to `config.hpp` options |
| New mechanism type | 1. Add to `MechanismsConfig` struct<br>2. Add to `Mechanisms` class<br>3. Update robot configs |
| New season | 1. Create `include/shulib/seasons/newgame_2027/`<br>2. Create `src/seasons/newgame_2027/`<br>3. Update `main.cpp` includes |
| New drivetrain type | 1. Create `include/shulib/core/drivetrain/mecanum.hpp`<br>2. Inherit from `Drivetrain`<br>3. Update `main.cpp` to use it |

See specific guides:
- [Adding a Robot](../configuration/ADDING_A_ROBOT.md)
- [Adding a Season](../seasons/ADDING_A_SEASON.md)
- [Writing Autonomous](../seasons/AUTONOMOUS.md)

---

## File Naming Conventions

### Files

| Type | Convention | Example |
|------|------------|---------|
| Headers | lowercase, underscores | `robot_config.hpp` |
| Sources | lowercase, underscores | `auton.cpp` |
| Docs | UPPERCASE, underscores | `MOTOR_SIGNS.md` |

### Code

| Type | Convention | Example |
|------|------------|---------|
| Classes | PascalCase | `TankDrive`, `OdomUnit` |
| Functions | camelCase | `rotateTo()`, `moveVertical()` |
| Variables | camelCase | `leftMotors`, `trackWidth` |
| Constants | UPPER_SNAKE_CASE | `MAX_POWER`, `TIMEOUT` |
| Namespaces | lowercase | `shulib::core`, `shulib::seasons::pushback` |
| Config structs | PascalCase | `RobotConfig`, `DrivetrainConfig` |
| Config instances | UPPER_SNAKE_CASE | `TESTBOT`, `XEBEC` |

### Directories

| Type | Convention | Example |
|------|------------|---------|
| Code folders | lowercase | `core/`, `robots/`, `seasons/` |
| Doc folders | lowercase, hyphens | `getting-started/`, `core-library/` |
| Season folders | lowercase, underscore, year | `pushback_2026/` |