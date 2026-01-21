# Data Flow

This document explains how data moves through shulib during different operations.

---

## Table of Contents

- [Data Flow](#data-flow)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Initialization Flow](#initialization-flow)
    - [Sequence Diagram](#sequence-diagram)
    - [Data Transformations](#data-transformations)
  - [Odometry Flow](#odometry-flow)
    - [Background Task Loop](#background-task-loop)
    - [Coordinate System Reference](#coordinate-system-reference)
  - [Motion Command Flow](#motion-command-flow)
    - [rotateTo() Flow](#rotateto-flow)
  - [Driver Control Flow](#driver-control-flow)
  - [Telemetry Flow](#telemetry-flow)
    - [Sample Terminal Output](#sample-terminal-output)
  - [Configuration Flow](#configuration-flow)
  - [Related Documents](#related-documents)

---

## Overview

Data in shulib flows through several distinct paths:

| Flow Type | Direction | Purpose |
|-----------|-----------|---------|
| Configuration | Config → Objects | Set up robot at startup |
| Commands | User/Auto → Motors | Control robot movement |
| Odometry | Sensors → Pose | Track robot position |
| Telemetry | System → Terminal | Debug and monitoring |

---

## Initialization Flow

How configuration data flows from files to runtime objects.

### Sequence Diagram
```
┌──────────┐     ┌──────────┐     ┌──────────┐     ┌──────────┐     ┌──────────┐
│config.hpp│     │ main.cpp │     │testbot.hpp│    │  PROS    │     │ Hardware │
└────┬─────┘     └────┬─────┘     └────┬─────┘     └────┬─────┘     └────┬─────┘
     │                │                │                │                │
     │ #define        │                │                │                │
     │ ROBOT_TESTBOT  │                │                │                │
     │───────────────>│                │                │                │
     │                │                │                │                │
     │                │ #include       │                │                │
     │                │ testbot.hpp    │                │                │
     │                │───────────────>│                │                │
     │                │                │                │                │
     │                │ ROBOT =        │                │                │
     │                │ TESTBOT config │                │                │
     │                │<───────────────│                │                │
     │                │                │                │                │
     │                │ Create MotorGroup               │                │
     │                │ with ROBOT.drivetrain.left_ports│                │
     │                │────────────────────────────────>│                │
     │                │                │                │                │
     │                │                │                │ Initialize     │
     │                │                │                │ motor ports    │
     │                │                │                │───────────────>│
     │                │                │                │                │
     │                │ Create TankDrive                │                │
     │                │ with MotorGroups                │                │
     │                │────────────────────────────────>│                │
     │                │                │                │                │
     │                │ Create Chassis                  │                │
     │                │ with Drivetrain + Sensors       │                │
     │                │────────────────────────────────>│                │
     │                │                │                │                │
     │                │ chassis.calibrate()             │                │
     │                │────────────────────────────────>│                │
     │                │                │                │                │
     │                │                │                │ Reset encoders │
     │                │                │                │───────────────>│
     │                │                │                │                │
     │                │                │                │ Start odom task│
     │                │                │                │───────────────>│
     │                │                │                │                │
```

### Data Transformations
```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         CONFIGURATION DATA FLOW                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  config.hpp                                                                  │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │ #define ROBOT_TESTBOT                                                │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                      │                                       │
│                                      ▼                                       │
│  testbot.hpp                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │ RobotConfig TESTBOT = {                                              │    │
│  │     .name = "TestBot",                                               │    │
│  │     .drivetrain = {                                                  │    │
│  │         .left_ports = {-16, 17, -18, 19, -20},                       │    │
│  │         .right_ports = {11, -12, 13, -14, 15},                       │    │
│  │         .track_width = 15.0,                                         │    │
│  │         .wheel_diameter = 3.25,                                      │    │
│  │         .rpm = 400                                                   │    │
│  │     },                                                               │    │
│  │     ...                                                              │    │
│  │ }                                                                    │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                      │                                       │
│                                      ▼                                       │
│  main.cpp - Object Creation                                                  │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │ // Port numbers extracted from config                                │    │
│  │ pros::MotorGroup leftMotors({-16, 17, -18, 19, -20});                │    │
│  │ pros::MotorGroup rightMotors({11, -12, 13, -14, 15});                │    │
│  │                                                                      │    │
│  │ // Dimensions extracted from config                                  │    │
│  │ TankDrive drivetrain(leftMotors, rightMotors,                        │    │
│  │                      15.0,   // track_width                          │    │
│  │                      3.25,   // wheel_diameter                       │    │
│  │                      400);   // rpm                                  │    │
│  │                                                                      │    │
│  │ // Chassis wraps everything                                          │    │
│  │ Chassis chassis(drivetrain, sensors);                                │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                      │                                       │
│                                      ▼                                       │
│  Runtime Objects                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │ chassis ─┬─> drivetrain ─┬─> leftMotors ─> Physical Motors          │    │
│  │          │               └─> rightMotors ─> Physical Motors         │    │
│  │          │                                                          │    │
│  │          └─> sensors ─┬─> leftOdom ─> Rotation Sensor               │    │
│  │                       ├─> rightOdom ─> Rotation Sensor              │    │
│  │                       └─> backOdom ─> Rotation Sensor               │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Odometry Flow

How position data flows from sensors to the Pose.

### Background Task Loop
```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      ODOMETRY UPDATE LOOP (every 10ms)                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                        STEP 1: Read Sensors                          │    │
│  │                                                                      │    │
│  │    ┌───────────┐     ┌───────────┐     ┌───────────┐                │    │
│  │    │   Left    │     │   Right   │     │   Back    │                │    │
│  │    │  Encoder  │     │  Encoder  │     │  Encoder  │                │    │
│  │    │           │     │           │     │           │                │    │
│  │    │  Raw: 1847│     │  Raw: 1852│     │  Raw: 23  │                │    │
│  │    └─────┬─────┘     └─────┬─────┘     └─────┬─────┘                │    │
│  │          │                 │                 │                       │    │
│  └──────────┼─────────────────┼─────────────────┼───────────────────────┘    │
│             │                 │                 │                            │
│             ▼                 ▼                 ▼                            │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                     STEP 2: Convert to Distance                      │    │
│  │                                                                      │    │
│  │    distance = (raw_ticks / ticks_per_rev) * wheel_circumference     │    │
│  │                                                                      │    │
│  │    ┌───────────┐     ┌───────────┐     ┌───────────┐                │    │
│  │    │   Left    │     │   Right   │     │   Back    │                │    │
│  │    │  Distance │     │  Distance │     │  Distance │                │    │
│  │    │           │     │           │     │           │    (inches)    │    │
│  │    │  15.234"  │     │  15.276"  │     │  0.189"   │                │    │
│  │    └─────┬─────┘     └─────┬─────┘     └─────┬─────┘                │    │
│  │          │                 │                 │                       │    │
│  └──────────┼─────────────────┼─────────────────┼───────────────────────┘    │
│             │                 │                 │                            │
│             ▼                 ▼                 ▼                            │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                      STEP 3: Calculate Deltas                        │    │
│  │                                                                      │    │
│  │    deltaLeft  = currentLeft  - previousLeft   =  0.042"             │    │
│  │    deltaRight = currentRight - previousRight  =  0.044"             │    │
│  │    deltaBack  = currentBack  - previousBack   =  0.002"             │    │
│  │                                                                      │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                      │                                       │
│                                      ▼                                       │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                    STEP 4: Calculate Movement                        │    │
│  │                                                                      │    │
│  │    // Forward/backward movement (average of left and right)         │    │
│  │    deltaForward = (deltaLeft + deltaRight) / 2 = 0.043"             │    │
│  │                                                                      │    │
│  │    // Rotation (difference between sides / track width)             │    │
│  │    deltaTheta = (deltaRight - deltaLeft) / trackWidth = 0.00013 rad │    │
│  │                                                                      │    │
│  │    // Strafe (back wheel minus rotation component)                  │    │
│  │    deltaStrafe = deltaBack - (backOffset * deltaTheta) = 0.002"     │    │
│  │                                                                      │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                      │                                       │
│                                      ▼                                       │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                STEP 5: Apply Rotation Matrix                         │    │
│  │                                                                      │    │
│  │    // Convert local movement to global coordinates                  │    │
│  │    avgTheta = currentTheta + (deltaTheta / 2)                       │    │
│  │                                                                      │    │
│  │    deltaX = deltaForward * sin(avgTheta) + deltaStrafe * cos(avgTheta)│   │
│  │    deltaY = deltaForward * cos(avgTheta) - deltaStrafe * sin(avgTheta)│   │
│  │                                                                      │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                      │                                       │
│                                      ▼                                       │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                    STEP 6: Update Global Pose                        │    │
│  │                                                                      │    │
│  │    globalX     += deltaX     →  X = 24.532"                         │    │
│  │    globalY     += deltaY     →  Y = 36.891"                         │    │
│  │    globalTheta += deltaTheta →  θ = 1.571 rad (90°)                 │    │
│  │                                                                      │    │
│  │    ┌─────────────────────────────────────────────────────────────┐  │    │
│  │    │                    CURRENT POSE                              │  │    │
│  │    │                                                              │  │    │
│  │    │                X = 24.532 inches                             │  │    │
│  │    │                Y = 36.891 inches                             │  │    │
│  │    │                θ = 90.0 degrees                              │  │    │
│  │    │                                                              │  │    │
│  │    └─────────────────────────────────────────────────────────────┘  │    │
│  │                                                                      │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Coordinate System Reference
```
                    +Y (forward)
                       ↑
                       │
                       │
         ┌─────────────┼─────────────┐
         │             │             │
         │    ROBOT    │             │
-X ←─────┤      ●──────┼────────────>│──────→ +X
(left)   │    (center) │             │        (right)
         │             │             │
         │             │             │
         └─────────────┼─────────────┘
                       │
                       │
                       ↓
                    -Y (backward)

        Theta (θ):
        • 0° = facing +Y (forward)
        • 90° = facing +X (right)
        • 180° = facing -Y (backward)
        • -90° = facing -X (left)
```

---

## Motion Command Flow

How a motion command travels from autonomous code to the motors.

### rotateTo() Flow
```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         rotateTo(chassis, 90)                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  auton.cpp                                                                   │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │ rotateTo(chassis, 90);   // Turn to face 90 degrees                  │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                      │                                       │
│                                      ▼                                       │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                         CONTROL LOOP                                 │    │
│  │                                                                      │    │
│  │   while (not at target) {                                           │    │
│  │                                                                      │    │
│  │       ┌─────────────────────────────────────────────────────────┐   │    │
│  │       │ 1. GET CURRENT ANGLE                                     │   │    │
│  │       │                                                          │   │    │
│  │       │    Pose current = chassis.getPose();                     │   │    │
│  │       │    double currentAngle = current.theta;  // e.g., 45°    │   │    │
│  │       └─────────────────────────────────────────────────────────┘   │    │
│  │                              │                                       │    │
│  │                              ▼                                       │    │
│  │       ┌─────────────────────────────────────────────────────────┐   │    │
│  │       │ 2. CALCULATE ERROR                                       │   │    │
│  │       │                                                          │   │    │
│  │       │    double error = target - current;  // 90 - 45 = 45°    │   │    │
│  │       │                                                          │   │    │
│  │       │    // Normalize to [-180, 180]                           │   │    │
│  │       │    while (error > 180) error -= 360;                     │   │    │
│  │       │    while (error < -180) error += 360;                    │   │    │
│  │       └─────────────────────────────────────────────────────────┘   │    │
│  │                              │                                       │    │
│  │                              ▼                                       │    │
│  │       ┌─────────────────────────────────────────────────────────┐   │    │
│  │       │ 3. PID CALCULATION                                       │   │    │
│  │       │                                                          │   │    │
│  │       │    power = pid.update(error, deltaTime);                 │   │    │
│  │       │                                                          │   │    │
│  │       │    // PID internals:                                     │   │    │
│  │       │    // P = kP * error           = 1.5 * 45 = 67.5         │   │    │
│  │       │    // I = kI * totalError      = 0.01 * 100 = 1.0        │   │    │
│  │       │    // D = kD * (error - prev)  = 0.15 * 5 = 0.75         │   │    │
│  │       │    // power = P + I + D        = 69.25                   │   │    │
│  │       │                                                          │   │    │
│  │       │    power = clamp(power, -50, 50);  // Limit to max       │   │    │
│  │       │    // power = 50                                         │   │    │
│  │       └─────────────────────────────────────────────────────────┘   │    │
│  │                              │                                       │    │
│  │                              ▼                                       │    │
│  │       ┌─────────────────────────────────────────────────────────┐   │    │
│  │       │ 4. SEND COMMAND                                          │   │    │
│  │       │                                                          │   │    │
│  │       │    chassis.drive(0, 0, power);  // (h=0, v=0, turn=50)   │   │    │
│  │       └─────────────────────────────────────────────────────────┘   │    │
│  │                              │                                       │    │
│  │       pros::delay(10);      │                                       │    │
│  │   }                         │                                       │    │
│  │                             │                                       │    │
│  └─────────────────────────────┼───────────────────────────────────────┘    │
│                                │                                            │
│                                ▼                                            │
│  chassis.cpp                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │ void Chassis::drive(int h, int v, int turn) {                        │    │
│  │     drivetrain.drive(h, v, turn);                                    │    │
│  │ }                                                                    │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                │                                            │
│                                ▼                                            │
│  drivetrain.cpp                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │ void Drivetrain::drive(int h, int v, int turn) {                     │    │
│  │     for (auto& config : motorConfigs) {                              │    │
│  │         int output = h * config.hCoef +                              │    │
│  │                      v * config.vCoef +                              │    │
│  │                      turn * config.turnCoef;                         │    │
│  │         config.motors->move(output);                                 │    │
│  │     }                                                                │    │
│  │ }                                                                    │    │
│  │                                                                      │    │
│  │ // For TankDrive:                                                    │    │
│  │ // Left:  h*0 + v*1 + turn*1  = 0 + 0 + 50 = 50                     │    │
│  │ // Right: h*0 + v*1 + turn*(-1) = 0 + 0 - 50 = -50                  │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                │                                            │
│                                ▼                                            │
│  PROS Motor API                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │ leftMotors.move(50);   // All left motors spin forward               │    │
│  │ rightMotors.move(-50); // All right motors spin backward             │    │
│  │                                                                      │    │
│  │ Result: Robot rotates clockwise (turning right toward 90°)          │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Driver Control Flow

How joystick input becomes motor movement.
```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           DRIVER CONTROL LOOP                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  opcontrol.cpp                                                               │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │ void run(Chassis& chassis, const RobotConfig& config) {              │    │
│  │     pros::Controller controller(pros::E_CONTROLLER_MASTER);          │    │
│  │     Mechanisms mech(config.mechanisms);                              │    │
│  │                                                                      │    │
│  │     while (true) {                                                   │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                      │                                       │
│                                      ▼                                       │
│       ┌─────────────────────────────────────────────────────────────────┐   │
│       │                     READ JOYSTICKS                               │   │
│       │                                                                  │   │
│       │  int leftY = controller.get_analog(ANALOG_LEFT_Y);   // -127~127│   │
│       │  int rightX = controller.get_analog(ANALOG_RIGHT_X); // -127~127│   │
│       │                                                                  │   │
│       │  Example: leftY = 100 (pushing forward)                         │   │
│       │           rightX = 30 (slight right turn)                       │   │
│       └─────────────────────────────────────────────────────────────────┘   │
│                                      │                                       │
│                                      ▼                                       │
│       ┌─────────────────────────────────────────────────────────────────┐   │
│       │                    APPLY DEADBAND                                │   │
│       │                                                                  │   │
│       │  // Ignore small inputs (joystick drift)                        │   │
│       │  if (abs(leftY) < 10) leftY = 0;                                │   │
│       │  if (abs(rightX) < 10) rightX = 0;                              │   │
│       │                                                                  │   │
│       │  // leftY = 100 (unchanged, above deadband)                     │   │
│       │  // rightX = 30 (unchanged, above deadband)                     │   │
│       └─────────────────────────────────────────────────────────────────┘   │
│                                      │                                       │
│                                      ▼                                       │
│       ┌─────────────────────────────────────────────────────────────────┐   │
│       │                   SEND TO CHASSIS                                │   │
│       │                                                                  │   │
│       │  chassis.drive(0, leftY, rightX);                               │   │
│       │  // chassis.drive(0, 100, 30)                                   │   │
│       └─────────────────────────────────────────────────────────────────┘   │
│                                      │                                       │
│                                      ▼                                       │
│       ┌─────────────────────────────────────────────────────────────────┐   │
│       │                   DRIVETRAIN CALCULATION                         │   │
│       │                                                                  │   │
│       │  // TankDrive coefficients:                                     │   │
│       │  // Left:  hCoef=0, vCoef=1, turnCoef=1                         │   │
│       │  // Right: hCoef=0, vCoef=1, turnCoef=-1                        │   │
│       │                                                                  │   │
│       │  leftPower  = 0*0 + 100*1 + 30*1  = 130 → clamped to 127       │   │
│       │  rightPower = 0*0 + 100*1 + 30*(-1) = 70                        │   │
│       │                                                                  │   │
│       │  leftMotors.move(127);                                          │   │
│       │  rightMotors.move(70);                                          │   │
│       │                                                                  │   │
│       │  Result: Robot drives forward while curving right               │   │
│       └─────────────────────────────────────────────────────────────────┘   │
│                                      │                                       │
│                                      ▼                                       │
│       ┌─────────────────────────────────────────────────────────────────┐   │
│       │                   HANDLE BUTTONS                                 │   │
│       │                                                                  │   │
│       │  if (controller.get_digital(DIGITAL_R1)) {                      │   │
│       │      mech.intakeIn();                                           │   │
│       │      mech.conveyorUp();                                         │   │
│       │  }                                                              │   │
│       │                                                                  │   │
│       │  if (controller.get_digital_new_press(DIGITAL_Y)) {             │   │
│       │      mech.toggleArm();                                          │   │
│       │  }                                                              │   │
│       └─────────────────────────────────────────────────────────────────┘   │
│                                      │                                       │
│       pros::delay(10);               │                                       │
│   }  // end while loop               │                                       │
│ }                                    │                                       │
│                                      │                                       │
└──────────────────────────────────────┴───────────────────────────────────────┘
```

---

## Telemetry Flow

How debug data flows from the robot to the terminal.
```
┌─────────────────────────────────────────────────────────────────────────────┐
│                            TELEMETRY DATA FLOW                               │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  Various Sources                                                             │
│  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐  ┌─────────────┐   │
│  │   Odometry    │  │    Battery    │  │    Motors     │  │   Events    │   │
│  │               │  │               │  │               │  │             │   │
│  │ x, y, theta   │  │ voltage       │  │ temperatures  │  │ log msgs    │   │
│  │               │  │ current       │  │               │  │ errors      │   │
│  └───────┬───────┘  └───────┬───────┘  └───────┬───────┘  └──────┬──────┘   │
│          │                  │                  │                 │          │
│          └──────────────────┼──────────────────┼─────────────────┘          │
│                             │                  │                            │
│                             ▼                  ▼                            │
│                    ┌─────────────────────────────────────┐                  │
│                    │              Logger                  │                  │
│                    │                                      │                  │
│                    │  Collects data from all sources     │                  │
│                    │  Formats as JSON                    │                  │
│                    │  Outputs periodically               │                  │
│                    └──────────────────┬──────────────────┘                  │
│                                       │                                     │
│                                       ▼                                     │
│                    ┌─────────────────────────────────────┐                  │
│                    │          JSON Formatting            │                  │
│                    │                                      │                  │
│                    │  {"odometry": {"x":24.5, "y":36.8,  │                  │
│                    │               "theta":1.57},        │                  │
│                    │   "battery": {"voltage":12950,      │                  │
│                    │               "current":2340}}      │                  │
│                    └──────────────────┬──────────────────┘                  │
│                                       │                                     │
│                                       ▼                                     │
│                    ┌─────────────────────────────────────┐                  │
│                    │         Serial Output (USB)         │                  │
│                    │                                      │                  │
│                    │  std::cout << jsonString << endl;   │                  │
│                    └──────────────────┬──────────────────┘                  │
│                                       │                                     │
│                                       │  USB Cable                          │
│                                       │                                     │
│                                       ▼                                     │
│                    ┌─────────────────────────────────────┐                  │
│                    │      Computer (pros terminal)       │                  │
│                    │                                      │                  │
│                    │  Displays JSON in terminal          │                  │
│                    │  Can be piped to analysis tools     │                  │
│                    └─────────────────────────────────────┘                  │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Sample Terminal Output
```json
{"messages": [{ "message": "Logger initialized!", "type": "success" }]}
{"messages": [{ "message": "Initializing TestBot", "type": "log" }]}
{"messages": [{ "message": "Chassis calibrated!", "type": "success" }]}
{"odometry": {"x":0.00,"y":0.00,"theta":0.00}}
{"battery": {"voltage":12950, "current":793, "temperature":28.0}}
{"odometry": {"x":0.05,"y":2.34,"theta":0.01}}
{"odometry": {"x":0.12,"y":5.67,"theta":0.02}}
{"controller": {"capacity":100, "level":4230}}
{"temps": {"_0":35,"_1":35,"_2":40,"_3":35,"_4":35}}
```

---

## Configuration Flow

How configuration values propagate through the system.
```
┌─────────────────────────────────────────────────────────────────────────────┐
│                       CONFIGURATION PROPAGATION                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  testbot.hpp                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │  .left_ports = {-16, 17, -18, 19, -20}                               │    │
│  └───────────────────────────────┬─────────────────────────────────────┘    │
│                                  │                                          │
│         ┌────────────────────────┼────────────────────────┐                 │
│         │                        │                        │                 │
│         ▼                        ▼                        ▼                 │
│  ┌─────────────┐          ┌─────────────┐          ┌─────────────┐         │
│  │  main.cpp   │          │  auton.cpp  │          │ opcontrol   │         │
│  │             │          │             │          │             │         │
│  │ Creates     │          │ Uses ROBOT  │          │ Uses ROBOT  │         │
│  │ MotorGroup  │          │ .name for   │          │ .name for   │         │
│  │ with ports  │          │ logging     │          │ display     │         │
│  └──────┬──────┘          └─────────────┘          └─────────────┘         │
│         │                                                                   │
│         ▼                                                                   │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  pros::MotorGroup leftMotors({-16, 17, -18, 19, -20})                │   │
│  │                                                                      │   │
│  │  Negative ports automatically reversed by PROS                      │   │
│  │  Port 16: reversed    (physical motor spins opposite)               │   │
│  │  Port 17: normal                                                    │   │
│  │  Port 18: reversed                                                  │   │
│  │  Port 19: normal                                                    │   │
│  │  Port 20: reversed                                                  │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│         │                                                                   │
│         ▼                                                                   │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  When leftMotors.move(100) is called:                                │   │
│  │                                                                      │   │
│  │  Port 16: receives -100 (reversed)  → spins "backward"              │   │
│  │  Port 17: receives +100 (normal)    → spins "forward"               │   │
│  │  Port 18: receives -100 (reversed)  → spins "backward"              │   │
│  │  Port 19: receives +100 (normal)    → spins "forward"               │   │
│  │  Port 20: receives -100 (reversed)  → spins "backward"              │   │
│  │                                                                      │   │
│  │  Because of physical mounting, all wheels spin the same direction!  │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Related Documents

- [Architecture Overview](OVERVIEW.md) – High-level design philosophy
- [Layer Diagram](LAYER_DIAGRAM.md) – Visual component diagram
- [Design Decisions](DESIGN_DECISIONS.md) – Why we made certain choices
- [Motor Signs](../configuration/MOTOR_SIGNS.md) – Understanding port polarities