# Layer Diagram

This document provides visual diagrams showing how shulib's components are organized and interact.

---

## Table of Contents

- [Layer Diagram](#layer-diagram)
  - [Table of Contents](#table-of-contents)
  - [High-Level Architecture](#high-level-architecture)
    - [The Stack](#the-stack)
  - [File Organization](#file-organization)
    - [Directory to Layer Mapping](#directory-to-layer-mapping)
  - [Component Relationships](#component-relationships)
    - [Chassis and Its Dependencies](#chassis-and-its-dependencies)
    - [Motion Function Dependencies](#motion-function-dependencies)
  - [Dependency Graph](#dependency-graph)
    - [What Depends on What](#what-depends-on-what)
    - [Import Rules](#import-rules)
  - [Runtime Object Structure](#runtime-object-structure)
    - [Objects Created at Startup](#objects-created-at-startup)
    - [Object Lifecycle](#object-lifecycle)
  - [Related Documents](#related-documents)

---

## High-Level Architecture

### The Stack
```
╔═════════════════════════════════════════════════════════════════════════════╗
║                              APPLICATION LAYER                               ║
╠═════════════════════════════════════════════════════════════════════════════╣
║                                                                              ║
║    ┌─────────────────────────────────────────────────────────────────────┐  ║
║    │                          config.hpp                                  │  ║
║    │                                                                      │  ║
║    │   #define ROBOT_TESTBOT          ← Which robot to use               │  ║
║    │   #define AUTON_TEST             ← Which autonomous to run          │  ║
║    └─────────────────────────────────────────────────────────────────────┘  ║
║                                      │                                       ║
║                                      ▼                                       ║
║    ┌─────────────────────────────────────────────────────────────────────┐  ║
║    │                           main.cpp                                   │  ║
║    │                                                                      │  ║
║    │   • Reads config.hpp                                                │  ║
║    │   • Creates global objects (motors, sensors, chassis)               │  ║
║    │   • Implements PROS callbacks (initialize, autonomous, opcontrol)   │  ║
║    └─────────────────────────────────────────────────────────────────────┘  ║
║                                                                              ║
╠═════════════════════════════════════════════════════════════════════════════╣
║                               SEASON LAYER                                   ║
╠═════════════════════════════════════════════════════════════════════════════╣
║                                                                              ║
║    ┌───────────────────────┐ ┌───────────────────────┐ ┌─────────────────┐  ║
║    │      auton.cpp        │ │    opcontrol.cpp      │ │  mechanisms.cpp │  ║
║    │                       │ │                       │ │                 │  ║
║    │ • rotateTo()          │ │ • Driver controls     │ │ • Intake        │  ║
║    │ • moveVertical()      │ │ • Button mappings     │ │ • Conveyor      │  ║
║    │ • moveToPose()        │ │ • Joystick handling   │ │ • Releaser      │  ║
║    │ • Competition autos   │ │                       │ │ • Pneumatics    │  ║
║    └───────────────────────┘ └───────────────────────┘ └─────────────────┘  ║
║                                                                              ║
╠═════════════════════════════════════════════════════════════════════════════╣
║                            CONFIGURATION LAYER                               ║
╠═════════════════════════════════════════════════════════════════════════════╣
║                                                                              ║
║    ┌───────────────────┐ ┌───────────────────┐ ┌───────────────────────┐    ║
║    │    testbot.hpp    │ │     xebec.hpp     │ │  queens_revenge.hpp   │    ║
║    │                   │ │                   │ │                       │    ║
║    │ • Motor ports     │ │ • Motor ports     │ │ • Motor ports         │    ║
║    │ • Tracking setup  │ │ • Tracking setup  │ │ • Tracking setup      │    ║
║    │ • Dimensions      │ │ • Dimensions      │ │ • Dimensions          │    ║
║    │ • Mechanisms      │ │ • Mechanisms      │ │ • Mechanisms          │    ║
║    └───────────────────┘ └───────────────────┘ └───────────────────────┘    ║
║                                      │                                       ║
║                                      ▼                                       ║
║                        ┌─────────────────────────┐                           ║
║                        │    robot_config.hpp     │                           ║
║                        │                         │                           ║
║                        │  • RobotConfig struct   │                           ║
║                        │  • DrivetrainConfig     │                           ║
║                        │  • TrackingConfig       │                           ║
║                        │  • MechanismsConfig     │                           ║
║                        └─────────────────────────┘                           ║
║                                                                              ║
╠═════════════════════════════════════════════════════════════════════════════╣
║                               CORE LIBRARY                                   ║
╠═════════════════════════════════════════════════════════════════════════════╣
║                                                                              ║
║    ┌─────────────────────────────────────────────────────────────────────┐  ║
║    │                           Chassis                                    │  ║
║    │                                                                      │  ║
║    │   drive() │ getPose() │ setPose() │ calibrate() │ setBrakeMode()    │  ║
║    └─────────────────────────────────────────────────────────────────────┘  ║
║           │                        │                                         ║
║           ▼                        ▼                                         ║
║    ┌─────────────────┐     ┌─────────────────┐                              ║
║    │   Drivetrain    │     │    Odometry     │                              ║
║    │                 │     │                 │                              ║
║    │ • TankDrive     │     │ • Position X,Y  │                              ║
║    │ • XDrive        │     │ • Heading Theta │                              ║
║    │ • Motor configs │     │ • Background    │                              ║
║    └─────────────────┘     └─────────────────┘                              ║
║           │                        │                                         ║
║           ▼                        ▼                                         ║
║    ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐      ║
║    │  MotorGroup     │     │    OdomUnit     │     │      PID        │      ║
║    │  (PROS)         │     │                 │     │                 │      ║
║    │                 │     │ • Rotation      │     │ • kP, kI, kD    │      ║
║    │ • move()        │     │   sensor        │     │ • update()      │      ║
║    │ • set_brake()   │     │ • Distance      │     │ • reset()       │      ║
║    └─────────────────┘     └─────────────────┘     └─────────────────┘      ║
║                                                                              ║
║    ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐      ║
║    │      Pose       │     │     Logger      │     │      Util       │      ║
║    │                 │     │                 │     │                 │      ║
║    │ • x, y, theta   │     │ • JSON output   │     │ • radToDeg()    │      ║
║    │ • distance()    │     │ • Telemetry     │     │ • degToRad()    │      ║
║    │ • angle()       │     │ • Debug levels  │     │ • slew()        │      ║
║    └─────────────────┘     └─────────────────┘     └─────────────────┘      ║
║                                                                              ║
╚═════════════════════════════════════════════════════════════════════════════╝
```

---

## File Organization

### Directory to Layer Mapping
```
shulib/
│
├── config.hpp ─────────────────────────────────────┐
│                                                   │  APPLICATION
├── src/main.cpp ───────────────────────────────────┘
│
├── src/seasons/pushback_2026/ ─────────────────────┐
│   ├── auton.cpp                                   │
│   ├── opcontrol.cpp                               │  SEASON
│   └── mechanisms.cpp                              │
├── include/shulib/seasons/pushback_2026/ ──────────┘
│
├── include/shulib/robots/ ─────────────────────────┐
│   ├── robot_config.hpp                            │  CONFIGURATION
│   ├── testbot.hpp                                 │
│   ├── xebec.hpp                                   │
│   └── queens_revenge.hpp ─────────────────────────┘
│
├── src/core/ ──────────────────────────────────────┐
│   ├── chassis.cpp                                 │
│   ├── drivetrain.cpp                              │
│   ├── odometry.cpp                                │
│   ├── odomUnit.cpp                                │  CORE LIBRARY
│   ├── pid.cpp                                     │
│   ├── pose.cpp                                    │
│   ├── logger.cpp                                  │
│   └── util.cpp                                    │
├── include/shulib/core/ ───────────────────────────┘
```

---

## Component Relationships

### Chassis and Its Dependencies
```
                              ┌─────────────────┐
                              │     Chassis     │
                              │                 │
                              │ The main robot  │
                              │ interface       │
                              └────────┬────────┘
                                       │
                 ┌─────────────────────┼─────────────────────┐
                 │                     │                     │
                 ▼                     ▼                     ▼
        ┌─────────────────┐   ┌─────────────────┐   ┌─────────────────┐
        │   Drivetrain    │   │   OdomSensors   │   │   (Settings)    │
        │                 │   │                 │   │                 │
        │ Controls motors │   │ Tracks position │   │ Future: PID     │
        └────────┬────────┘   └────────┬────────┘   │ gains, etc.     │
                 │                     │            └─────────────────┘
                 │            ┌────────┼────────┐
                 │            │        │        │
                 ▼            ▼        ▼        ▼
        ┌─────────────┐  ┌────────┐ ┌────────┐ ┌────────┐
        │ MotorGroups │  │ Left   │ │ Right  │ │ Back   │
        │             │  │ Odom   │ │ Odom   │ │ Odom   │
        │ • Left      │  │ Unit   │ │ Unit   │ │ Unit   │
        │ • Right     │  └───┬────┘ └───┬────┘ └───┬────┘
        └─────────────┘      │          │          │
                             ▼          ▼          ▼
                        ┌────────────────────────────────┐
                        │      Rotation Sensors          │
                        │           (PROS)               │
                        └────────────────────────────────┘
```

### Motion Function Dependencies
```
┌─────────────────────────────────────────────────────────────────────────────┐
│                            MOTION FUNCTIONS                                  │
│                                                                              │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐             │
│  │   rotateTo()    │  │  moveVertical() │  │  moveToPose()   │             │
│  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘             │
│           │                    │                    │                       │
│           │                    │           ┌───────┴───────┐               │
│           │                    │           │               │               │
│           │                    │           ▼               ▼               │
│           │                    │    ┌─────────────┐ ┌─────────────┐        │
│           │                    │    │ rotateTo()  │ │moveVertical()│       │
│           │                    │    └─────────────┘ └─────────────┘        │
│           │                    │                                            │
└───────────┼────────────────────┼────────────────────────────────────────────┘
            │                    │
            ▼                    ▼
    ┌───────────────────────────────────────┐
    │              USES                      │
    │                                        │
    │   ┌─────────┐  ┌─────────┐  ┌──────┐  │
    │   │ Chassis │  │   PID   │  │ Pose │  │
    │   │         │  │         │  │      │  │
    │   │ drive() │  │ update()│  │ x,y,θ│  │
    │   │getPose()│  │         │  │      │  │
    │   └─────────┘  └─────────┘  └──────┘  │
    │                                        │
    └────────────────────────────────────────┘
```

---

## Dependency Graph

### What Depends on What
```
                            ┌─────────────┐
                            │ config.hpp  │
                            └──────┬──────┘
                                   │
                                   ▼
                            ┌─────────────┐
                            │  main.cpp   │
                            └──────┬──────┘
                                   │
              ┌────────────────────┼────────────────────┐
              │                    │                    │
              ▼                    ▼                    ▼
     ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
     │ seasons/auton   │  │ seasons/opctrl  │  │ robots/testbot  │
     └────────┬────────┘  └────────┬────────┘  └────────┬────────┘
              │                    │                    │
              │                    │                    ▼
              │                    │           ┌─────────────────┐
              │                    │           │  robot_config   │
              │                    │           └─────────────────┘
              │                    │
              └─────────┬──────────┘
                        │
                        ▼
               ┌─────────────────┐
               │ seasons/mechs   │
               └────────┬────────┘
                        │
                        ▼
               ┌─────────────────┐
               │ core/chassis    │
               └────────┬────────┘
                        │
         ┌──────────────┼──────────────┐
         │              │              │
         ▼              ▼              ▼
┌─────────────┐ ┌─────────────┐ ┌─────────────┐
│  drivetrain │ │  odometry   │ │   logger    │
└──────┬──────┘ └──────┬──────┘ └─────────────┘
       │               │
       ▼               ▼
┌─────────────┐ ┌─────────────┐
│  tankdrive  │ │  odomUnit   │
└─────────────┘ └──────┬──────┘
                       │
                       ▼
               ┌─────────────┐
               │    pose     │
               └─────────────┘
                       │
                       ▼
               ┌─────────────┐
               │    util     │
               └─────────────┘
```

### Import Rules

| Layer | Can Import From |
|-------|-----------------|
| config.hpp | Nothing (just #defines) |
| main.cpp | Everything |
| seasons/* | core/*, robot configs |
| robots/* | robot_config.hpp only |
| core/* | Other core/*, PROS API |

**Key Rule:** Lower layers NEVER import from upper layers.

---

## Runtime Object Structure

### Objects Created at Startup
```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           GLOBAL OBJECTS                                     │
│                         (created in main.cpp)                                │
│                                                                              │
│  ┌───────────────────────────────────────────────────────────────────────┐  │
│  │ const RobotConfig& ROBOT = shulib::robots::TESTBOT                    │  │
│  │                                                                        │  │
│  │ Contains all configuration for the selected robot                     │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
│                                      │                                       │
│                    ┌─────────────────┼─────────────────┐                    │
│                    │                 │                 │                    │
│                    ▼                 ▼                 ▼                    │
│  ┌─────────────────────┐ ┌─────────────────┐ ┌─────────────────────────┐   │
│  │ pros::MotorGroup    │ │ pros::MotorGroup│ │ pros::Rotation (x3)     │   │
│  │ leftMotors          │ │ rightMotors     │ │ leftRot, rightRot,      │   │
│  │                     │ │                 │ │ backRot                 │   │
│  │ ports from config   │ │ ports from cfg  │ │ ports from config       │   │
│  └──────────┬──────────┘ └────────┬────────┘ └────────────┬────────────┘   │
│             │                     │                       │                 │
│             └──────────┬──────────┘                       │                 │
│                        │                                  │                 │
│                        ▼                                  ▼                 │
│            ┌───────────────────────┐        ┌─────────────────────────┐    │
│            │ shulib::TankDrive     │        │ shulib::OdomUnit (x3)   │    │
│            │ drivetrain            │        │ leftOdom, rightOdom,    │    │
│            │                       │        │ backOdom                │    │
│            └───────────┬───────────┘        └────────────┬────────────┘    │
│                        │                                 │                 │
│                        │            ┌────────────────────┘                 │
│                        │            │                                      │
│                        │            ▼                                      │
│                        │  ┌─────────────────────────┐                      │
│                        │  │ shulib::OdomSensors     │                      │
│                        │  │ sensors                 │                      │
│                        │  │                         │                      │
│                        │  │ Pointers to odom units  │                      │
│                        │  └────────────┬────────────┘                      │
│                        │               │                                   │
│                        └───────┬───────┘                                   │
│                                │                                           │
│                                ▼                                           │
│                    ┌───────────────────────┐                               │
│                    │ shulib::Chassis       │                               │
│                    │ chassis               │                               │
│                    │                       │                               │
│                    │ The main interface    │                               │
│                    │ used by auton/opctrl  │                               │
│                    └───────────────────────┘                               │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Object Lifecycle
```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              PROGRAM LIFECYCLE                               │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  POWER ON                                                                    │
│      │                                                                       │
│      ▼                                                                       │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │ Global constructors run                                              │    │
│  │ • MotorGroups created with ports from ROBOT config                  │    │
│  │ • Rotation sensors created                                          │    │
│  │ • OdomUnits created                                                 │    │
│  │ • TankDrive created                                                 │    │
│  │ • Chassis created                                                   │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│      │                                                                       │
│      ▼                                                                       │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │ initialize() called by PROS                                          │    │
│  │ • Logger initialized                                                │    │
│  │ • Chassis calibrated (resets sensors, starts odometry task)         │    │
│  │ • Initial pose set to (0, 0, 0)                                     │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│      │                                                                       │
│      ▼                                                                       │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │ opcontrol() called by PROS                                           │    │
│  │ • Runs in loop                                                      │    │
│  │ • Handles driver input                                              │    │
│  │ • Can trigger autonomous via button press                           │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│      │                                                                       │
│      │ (when A button pressed)                                              │
│      ▼                                                                       │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │ autonomous() called                                                  │    │
│  │ • Runs selected autonomous routine                                  │    │
│  │ • Uses motion functions (rotateTo, moveVertical, etc.)              │    │
│  │ • Returns when complete                                             │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│      │                                                                       │
│      ▼                                                                       │
│  Back to opcontrol()                                                         │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Related Documents

- [Architecture Overview](OVERVIEW.md) – High-level design philosophy
- [Data Flow](DATA_FLOW.md) – How data moves through the system
- [Design Decisions](DESIGN_DECISIONS.md) – Why we made certain choices
- [Project Structure](../getting-started/PROJECT_STRUCTURE.md) – File layout