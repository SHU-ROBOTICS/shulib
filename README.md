<img src="assets/banner.png">

<p>
<img src="https://img.shields.io/github/v/tag/n0es/shulib?label=shulib&color=%23004488">
<img src="https://img.shields.io/github/contributors/n0es/shulib">
</p>

**Seton Hall University Library for VEX Robotics**

---

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Documentation](#documentation)
- [Quick Start](#quick-start)
- [Current Season](#current-season)
- [Robots](#robots)
- [To-Do](#to-do)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgments](#acknowledgments)

---

## Introduction

shulib is a PROS template heavily inspired by [LemLib](https://github.com/LemLib/LemLib), designed to make implementing new developments simple and seamless.

Built to last across seasons and team generations, shulib provides:

- **Multi-robot support** – Switch between robots by changing one line
- **Season separation** – Game-specific code is isolated; core library is reusable year-to-year
- **Odometry & motion control** – Accurate autonomous movement using tracking wheels
- **Clean abstractions** – Simple APIs hide complex functionality
- **Extensive documentation** – Everything you need to understand, use, and extend the library

---

## Features

### Current Features (Stable)

- **Chassis Constructors**
  - Easily build and customize your robot's chassis with predefined templates and flexible parameters
  - Supports Tank Drive and X-Drive configurations
  - Configuration-driven: define once, use everywhere

- **Odometry / Absolute Positioning System**
  - Accurate tracking of the robot's position on the field
  - 3-wheel tracking (left, right, back) without IMU dependency
  - Real-time pose updates (X, Y, Theta)
  - Correction factors for fine-tuning accuracy

- **Motion Control**
  - `rotateTo(angle)` – PID-controlled rotation to absolute heading
  - `moveVertical(distance)` – Drive forward/backward with heading correction
  - `moveToPose(x, y, theta)` – Navigate to coordinates
  - Stall detection and timeout handling

- **Multi-Robot Configuration**
  - Define robots in separate config files
  - Switch robots with a single `#define`
  - Each robot has its own motor ports, tracking wheels, mechanisms

- **Telemetry & Logging**
  - JSON-formatted output for data analysis
  - Real-time odometry, battery, temperature monitoring
  - Debug, log, success, and error message levels

### Planned Features

- **Path Planning GUI** – Visual autonomous path creation with waypoint export
- **Communication Protocols** – Enhanced communication between multiple robots
- **User Interface Tools** – Brain screen autonomous selector, dashboard integrations
- **Pure Pursuit** – Smooth curved path following

---

## Documentation

### Getting Started

| Document | Description |
|----------|-------------|
| [Installation](getting-started/INSTALLATION.md) | Set up PROS, toolchain, VS Code |
| [Quick Start](getting-started/QUICK_START.md) | Get running in 10 minutes |
| [Project Structure](getting-started/PROJECT_STRUCTURE.md) | Understand what's where and why |
| [First Build](getting-started/FIRST_BUILD.md) | Building and uploading to robot |

### Architecture

| Document | Description |
|----------|-------------|
| [Overview](architecture/OVERVIEW.md) | High-level design philosophy |
| [Layer Diagram](architecture/LAYER_DIAGRAM.md) | How components interact |
| [Data Flow](architecture/DATA_FLOW.md) | How data moves through the system |
| [Design Decisions](architecture/DESIGN_DECISIONS.md) | Why we made certain choices |

### Core Library

| Document | Description |
|----------|-------------|
| [Chassis](core-library/CHASSIS.md) | Chassis class API & usage |
| [Drivetrain](core-library/DRIVETRAIN.md) | TankDrive, XDrive explained |
| [Odometry](core-library/ODOMETRY.md) | How position tracking works |
| [PID](core-library/PID.md) | PID controller theory & usage |
| [Pose](core-library/POSE.md) | Coordinate system and units |
| [OdomUnit](core-library/ODOM_UNIT.md) | Tracking wheel class |
| [Logger](core-library/LOGGER.md) | Telemetry and debugging |

### Configuration

| Document | Description |
|----------|-------------|
| [Robot Config](configuration/ROBOT_CONFIG.md) | All config options explained |
| [Adding a Robot](configuration/ADDING_A_ROBOT.md) | Step-by-step new robot guide |
| [Motor Signs](configuration/MOTOR_SIGNS.md) | How to determine port polarities |
| [Tracking Wheels](configuration/TRACKING_WHEELS.md) | Setting up odometry sensors |

### Motion

| Document | Description |
|----------|-------------|
| [Motion Overview](motion/MOTION_OVERVIEW.md) | How autonomous motion works |
| [moveVertical](motion/MOVE_VERTICAL.md) | Driving straight |
| [rotateTo](motion/ROTATE_TO.md) | Turning to angle |
| [moveToPose](motion/MOVE_TO_POSE.md) | Navigating to coordinates |
| [PID Tuning](motion/PID_TUNING.md) | Tuning guide with examples |

### Seasons

| Document | Description |
|----------|-------------|
| [Adding a Season](seasons/ADDING_A_SEASON.md) | How to set up next year's game |
| [Mechanisms](seasons/MECHANISMS.md) | Defining intakes, conveyors, etc. |
| [Autonomous](seasons/AUTONOMOUS.md) | Writing competition routines |
| [OpControl](seasons/OPCONTROL.md) | Driver control setup |

### Troubleshooting

| Document | Description |
|----------|-------------|
| [Common Issues](troubleshooting/COMMON_ISSUES.md) | Start here for problems |
| [Motors Fighting](troubleshooting/MOTORS_FIGHTING.md) | High current, no movement |
| [Odometry Drift](troubleshooting/ODOMETRY_DRIFT.md) | Position tracking problems |
| [PID Oscillation](troubleshooting/PID_OSCILLATION.md) | Robot shaking/vibrating |
| [Build Errors](troubleshooting/BUILD_ERRORS.md) | Compilation issues |

### Reference

| Document | Description |
|----------|-------------|
| [API Reference](reference/API_REFERENCE.md) | Full API documentation |
| [Glossary](reference/GLOSSARY.md) | Terms and definitions |
| [Units](reference/UNITS.md) | Inches, degrees, radians |
| [Coordinate System](reference/COORDINATE_SYSTEM.md) | X, Y, Theta conventions |

### Competition

| Document | Description |
|----------|-------------|
| [Checklist](competition/CHECKLIST.md) | Pre-match checklist |
| [Quick Reference](competition/QUICK_REFERENCE.md) | Pit cheat sheet |
| [Switching Robots](competition/SWITCHING_ROBOTS.md) | How to swap configs fast |

### Contributing

| Document | Description |
|----------|-------------|
| [Code Style](contributing/CODE_STYLE.md) | Formatting conventions |
| [Git Workflow](contributing/GIT_WORKFLOW.md) | Branching strategy |
| [Pull Requests](contributing/PULL_REQUESTS.md) | How to submit changes |

---

## Quick Start
```bash
# Clone the repository
git clone https://github.com/n0es/shulib.git
cd shulib

# Select your robot in config.hpp
# Uncomment ONE of:
#   #define ROBOT_XEBEC
#   #define ROBOT_QUEENS_REVENGE
#   #define ROBOT_TESTBOT

# Build and upload
pros make
pros upload

# View output
pros terminal
```

For detailed instructions, see [Quick Start Guide](getting-started/QUICK_START.md).

---

## Current Season

### Push Back (2025-2026)

| Detail | Info |
|--------|------|
| **Field** | 12ft × 12ft |
| **Objects** | 88 blocks (44 red, 44 blue) |
| **Goals** | 2 long goals, 2 center goals |
| **Scoring** | 3 pts/block, 10 pt control bonus |
| **Match** | 15 sec auto + 1:45 driver |
| **Parking** | 8 pts (1 robot), 30 pts (2 robots) |

See [Push Back Docs](seasons/pushback_2026/) for strategies and controls.

---

## Robots

| Robot | Left Motors | Right Motors | Notes |
|-------|-------------|--------------|-------|
| XEBEC | 12, -14, 16, -18, 20 | 11, -13, 15, -17, 19 | Competition robot |
| Queens Revenge | 11, -12, 13, -14, -15 | 16, -17, 18, -19, 20 | Competition robot |
| TestBot | -16, 17, -18, 19, -20 | 11, -12, 13, -14, 15 | Development robot |

Negative port = reversed motor. See [Motor Signs](configuration/MOTOR_SIGNS.md).

---

## To-Do

### Upcoming Features
- [ ] Path Planning GUI integration
- [ ] Brain screen autonomous selector
- [ ] Communication protocols
- [ ] Pure pursuit path following

### Ongoing Improvements
- [x] Odometry system
- [x] Motion functions
- [x] Multi-robot configuration
- [x] Comprehensive documentation

### Bug Fixes
- [x] Motor fighting (motor sign configuration)
- [x] PID oscillation at end of turns
- [ ] Fine-tune turn accuracy

---

## Contributing

1. Read [Code Style](contributing/CODE_STYLE.md)
2. Follow [Git Workflow](contributing/GIT_WORKFLOW.md)
3. Submit via [Pull Request](contributing/PULL_REQUESTS.md)
---

## License

shulib is released under the [MIT License](../LICENSE).

---

## Acknowledgments

- **[@n0es](https://github.com/n0es)** – Original creator, architecture, and odometry system
- **[LemLib](https://github.com/LemLib/LemLib)** – Inspiration for the library design
- **[PROS](https://pros.cs.purdue.edu/)** – The underlying robotics framework
- **Purdue ACM SIGBots** – For creating and maintaining PROS

---

<p align="center">
  <b>Built by Seton Hall University Robotics</b><br>
  <i>For the next generation of builders</i>
</p>