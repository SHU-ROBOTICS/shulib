# Drivetrain

The Drivetrain system handles motor control for different drive configurations. It provides a unified interface that works with both Tank Drive and X-Drive robots.

---

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Base Drivetrain Class](#base-drivetrain-class)
- [TankDrive](#tankdrive)
- [XDrive](#xdrive)
- [MotorConfig System](#motorconfig-system)
- [How Drive Commands Work](#how-drive-commands-work)
- [Adding a New Drivetrain Type](#adding-a-new-drivetrain-type)

---

## Overview

### What is a Drivetrain?

A Drivetrain is the system that converts high-level movement commands (`drive forward`, `turn right`) into individual motor power values. Different robot designs need different motor configurations:

| Type | Motors | Movement | Use Case |
|------|--------|----------|----------|
| **Tank Drive** | 2 groups (left/right) | Forward, backward, turn | Most common, simple, reliable |
| **X-Drive** | 4 groups (corners) | Forward, backward, strafe, turn | Omnidirectional movement |

### Design Philosophy

The Drivetrain system uses a **coefficient-based approach**:

1. Each motor group has coefficients for horizontal, vertical, and turn
2. The `drive()` function multiplies inputs by coefficients
3. Different drivetrain types just use different coefficient values

This means the same `drive(h, v, turn)` interface works for any drivetrain type.

---

## Architecture
```
┌─────────────────────────────────────────────────────────────┐
│                        Chassis                               │
│                                                              │
│   chassis.drive(horizontal, vertical, turn)                  │
└──────────────────────────┬──────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                    Drivetrain (base)                         │
│                                                              │
│   • Stores vector of MotorConfig                            │
│   • drive() applies coefficients to each motor group        │
│   • setBrakeMode() sets mode for all motors                 │
└──────────────────────────┬──────────────────────────────────┘
                           │
              ┌────────────┴────────────┐
              │                         │
              ▼                         ▼
┌─────────────────────────┐   ┌─────────────────────────┐
│       TankDrive         │   │        XDrive           │
│                         │   │                         │
│ • 2 motor groups        │   │ • 4 motor groups        │
│ • Left: h=0, v=1, t=1   │   │ • FL: h=1, v=1, t=1     │
│ • Right: h=0, v=1, t=-1 │   │ • FR: h=-1, v=1, t=-1   │
│                         │   │ • BL: h=-1, v=1, t=1    │
│                         │   │ • BR: h=1, v=1, t=-1    │
└─────────────────────────┘   └─────────────────────────┘
```

---

## Base Drivetrain Class

From `include/shulib/core/drivetrain.hpp`:
```cpp
namespace shulib {

class Drivetrain {
public:
    Drivetrain(float wheelDiameter, float rpm, float horizontalDrift)
        : wheelDiameter(wheelDiameter), rpm(rpm), horizontalDrift(horizontalDrift) {}
    
    virtual ~Drivetrain() = default;
    
    // Main drive method - applies coefficients to all motor groups
    virtual void drive(int horizontal, int vertical, int turn, bool fieldCentric);
    
    // Set brake mode for all motors
    virtual void setBrakeMode(pros::motor_brake_mode_e mode);
    
    // Get wheel diameter (used for distance calculations)
    float getWheelDiameter() { return wheelDiameter; }
    
    // Get all motor configurations
    virtual std::vector<MotorConfig> getMotorConfigs() { return motorConfigs; }
    
    // Get temperatures of all motors
    std::map<std::string, double> getTemps();
    
    // Configuration for each motor group
    struct MotorConfig {
        pros::MotorGroup* motors;
        float horizontalCoefficient;
        float verticalCoefficient;
        float turnCoefficient;
        std::string name;
    };

protected:
    float wheelDiameter;
    float rpm;
    float horizontalDrift;
    std::vector<MotorConfig> motorConfigs;
};

}  // namespace shulib
```

### Constructor Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `wheelDiameter` | `float` | Diameter of drive wheels in inches |
| `rpm` | `float` | Motor cartridge RPM (100, 200, or 600) |
| `horizontalDrift` | `float` | Drift compensation for strafing (X-Drive only) |

---

## TankDrive

Tank Drive is the most common drivetrain type. It has two motor groups (left and right) that work together for forward/backward movement and opposite for turning.

### Class Definition

From `include/shulib/core/drivetrain/tankdrive.hpp`:
```cpp
namespace shulib {

class TankDrive : public Drivetrain {
public:
    TankDrive(pros::MotorGroup& leftMotors,
              pros::MotorGroup& rightMotors, 
              float trackWidth,
              float wheelDiameter, 
              float rpm)
        : Drivetrain(wheelDiameter, rpm, 0),
          trackWidth(trackWidth) 
    {
        // Left motors: no strafe, forward, turn adds power
        MotorConfig leftConfig = {&leftMotors, 0, 1, 1};
        motorConfigs.push_back(leftConfig);
        
        // Right motors: no strafe, forward, turn subtracts power
        MotorConfig rightConfig = {&rightMotors, 0, 1, -1};
        motorConfigs.push_back(rightConfig);
    }

private:
    float trackWidth;
};

}  // namespace shulib
```

### Constructor Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `leftMotors` | `pros::MotorGroup&` | Reference to left side motor group |
| `rightMotors` | `pros::MotorGroup&` | Reference to right side motor group |
| `trackWidth` | `float` | Distance between left and right wheels (inches) |
| `wheelDiameter` | `float` | Diameter of drive wheels (inches) |
| `rpm` | `float` | Motor cartridge RPM |

### Motor Coefficients

| Motor Group | Horizontal | Vertical | Turn |
|-------------|------------|----------|------|
| Left | 0 | 1 | 1 |
| Right | 0 | 1 | -1 |

### How It Works

For `drive(h, v, turn)`:
```
Left Power  = h*0 + v*1 + turn*1  = v + turn
Right Power = h*0 + v*1 + turn*(-1) = v - turn
```

| Command | Left | Right | Result |
|---------|------|-------|--------|
| `drive(0, 100, 0)` | 100 | 100 | Forward |
| `drive(0, -100, 0)` | -100 | -100 | Backward |
| `drive(0, 0, 50)` | 50 | -50 | Turn right (clockwise) |
| `drive(0, 0, -50)` | -50 | 50 | Turn left (counter-clockwise) |
| `drive(0, 80, 30)` | 110→127 | 50 | Forward + curve right |

### Usage Example
```cpp
// Create motor groups (port signs handle motor direction)
pros::MotorGroup leftMotors({-16, 17, -18, 19, -20});
pros::MotorGroup rightMotors({11, -12, 13, -14, 15});

// Create tank drive
shulib::TankDrive drivetrain(
    leftMotors, 
    rightMotors,
    15.0,   // track width (inches)
    3.25,   // wheel diameter (inches)
    400     // motor RPM
);
```

---

## XDrive

X-Drive (also called Holonomic or Mecanum-style) has four motor groups at 45-degree angles, allowing the robot to strafe (move sideways) without turning.

### Class Definition

From `include/shulib/core/drivetrain/xdrive.hpp`:
```cpp
namespace shulib {

class XDrive : public Drivetrain {
public:
    XDrive(pros::MotorGroup& frontLeft, 
           pros::MotorGroup& frontRight,
           pros::MotorGroup& backLeft, 
           pros::MotorGroup& backRight,
           float wheelDiameter, 
           float rpm, 
           float horizontalDrift)
        : Drivetrain(wheelDiameter, rpm, horizontalDrift) 
    {
        // Front Left: strafe right, forward, turn right
        MotorConfig flConfig = {&frontLeft, 1, 1, 1};
        motorConfigs.push_back(flConfig);
        
        // Front Right: strafe left, forward, turn left
        MotorConfig frConfig = {&frontRight, -1, 1, -1};
        motorConfigs.push_back(frConfig);
        
        // Back Left: strafe left, forward, turn right
        MotorConfig blConfig = {&backLeft, -1, 1, 1};
        motorConfigs.push_back(blConfig);
        
        // Back Right: strafe right, forward, turn left
        MotorConfig brConfig = {&backRight, 1, 1, -1};
        motorConfigs.push_back(brConfig);
    }
    
    std::string toString() { return "XDrive"; }
};

}  // namespace shulib
```

### Motor Coefficients

| Motor Group | Horizontal | Vertical | Turn |
|-------------|------------|----------|------|
| Front Left | 1 | 1 | 1 |
| Front Right | -1 | 1 | -1 |
| Back Left | -1 | 1 | 1 |
| Back Right | 1 | 1 | -1 |

### How It Works

For `drive(h, v, turn)`:
```
FL = h*1  + v*1 + turn*1  = h + v + turn
FR = h*(-1) + v*1 + turn*(-1) = -h + v - turn
BL = h*(-1) + v*1 + turn*1  = -h + v + turn
BR = h*1  + v*1 + turn*(-1) = h + v - turn
```

| Command | FL | FR | BL | BR | Result |
|---------|----|----|----|----|--------|
| `drive(0, 100, 0)` | 100 | 100 | 100 | 100 | Forward |
| `drive(100, 0, 0)` | 100 | -100 | -100 | 100 | Strafe right |
| `drive(-100, 0, 0)` | -100 | 100 | 100 | -100 | Strafe left |
| `drive(0, 0, 50)` | 50 | -50 | 50 | -50 | Turn right |
| `drive(50, 50, 0)` | 100 | 0 | 0 | 100 | Diagonal (forward-right) |

### Usage Example
```cpp
// Create motor groups for each corner
pros::MotorGroup frontLeft({1});
pros::MotorGroup frontRight({-2});
pros::MotorGroup backLeft({3});
pros::MotorGroup backRight({-4});

// Create X-drive
shulib::XDrive drivetrain(
    frontLeft, frontRight,
    backLeft, backRight,
    4.0,    // wheel diameter (inches)
    600,    // motor RPM
    1.0     // horizontal drift compensation
);
```

---

## MotorConfig System

The `MotorConfig` struct is the heart of the coefficient-based system:
```cpp
struct MotorConfig {
    pros::MotorGroup* motors;      // Pointer to the motor group
    float horizontalCoefficient;   // Multiplier for strafe input
    float verticalCoefficient;     // Multiplier for forward/back input
    float turnCoefficient;         // Multiplier for rotation input
    std::string name;              // Optional name for debugging
};
```

### Why Coefficients?

Instead of hardcoding motor math for each drivetrain type:
```cpp
// BAD: Hardcoded for tank drive
void TankDrive::drive(int v, int turn) {
    leftMotors.move(v + turn);
    rightMotors.move(v - turn);
}

// BAD: Hardcoded for X-drive
void XDrive::drive(int h, int v, int turn) {
    frontLeft.move(h + v + turn);
    frontRight.move(-h + v - turn);
    // etc.
}
```

We use a generic loop with coefficients:
```cpp
// GOOD: Works for any drivetrain
void Drivetrain::drive(int h, int v, int turn, bool fieldCentric) {
    for (const auto& config : motorConfigs) {
        int output = h * config.horizontalCoefficient +
                     v * config.verticalCoefficient +
                     turn * config.turnCoefficient;
        config.motors->move(output);
    }
}
```

This makes it easy to add new drivetrain types - just define the right coefficients!

---

## How Drive Commands Work

### The drive() Implementation

From `src/core/drivetrain.cpp`:
```cpp
void shulib::Drivetrain::drive(int horizontal, int vertical, int turn,
                               bool fieldCentric) {
    // Field-centric transformation (optional)
    if (fieldCentric) {
        double angle = shulib::getPose().theta;
        double cosA = cos(angle);
        double sinA = sin(angle);
        horizontal = horizontal * cosA - vertical * sinA;
        vertical = horizontal * sinA + vertical * cosA;
    }

    // Apply coefficients to each motor group
    for (const auto& config : motorConfigs) {
        int motorOutput = horizontal * config.horizontalCoefficient +
                          vertical * config.verticalCoefficient +
                          turn * config.turnCoefficient;
        config.motors->move(motorOutput);
    }
}
```

### Step-by-Step Example

Given a TankDrive with:
- Left motors: `{h=0, v=1, turn=1}`
- Right motors: `{h=0, v=1, turn=-1}`

When you call `chassis.drive(0, 80, 30)`:
```
1. Field centric is false, so inputs unchanged
   horizontal = 0, vertical = 80, turn = 30

2. Process LEFT motors (h=0, v=1, turn=1):
   output = 0*0 + 80*1 + 30*1 = 110
   → Clamped to 127 by PROS
   leftMotors.move(110)

3. Process RIGHT motors (h=0, v=1, turn=-1):
   output = 0*0 + 80*1 + 30*(-1) = 50
   rightMotors.move(50)

4. Result: Robot moves forward while curving right
   (left side faster than right)
```

---

## Additional Methods

### setBrakeMode()

Sets the brake mode for all motors in all motor groups:
```cpp
void shulib::Drivetrain::setBrakeMode(pros::motor_brake_mode_e mode) {
    for (const auto& config : motorConfigs) {
        config.motors->set_brake_mode_all(mode);
    }
}
```

### getTemps()

Returns a map of motor temperatures for monitoring:
```cpp
std::map<std::string, double> shulib::Drivetrain::getTemps() {
    std::map<std::string, double> temps;
    for (const auto& config : motorConfigs) {
        int i = 0;
        for (const auto& temp : config.motors->get_temperature_all()) {
            temps[config.name + "_" + std::to_string(i)] = temp;
            i++;
        }
    }
    return temps;
}
```

Usage:
```cpp
auto temps = drivetrain.getTemps();
for (auto& [name, temp] : temps) {
    printf("%s: %.1f°C\n", name.c_str(), temp);
}
```

---

## Adding a New Drivetrain Type

To add a new drivetrain type (e.g., Mecanum, Swerve):

### Step 1: Create the Header

Create `include/shulib/core/drivetrain/mecanum.hpp`:
```cpp
#pragma once
#include "pros/motor_group.hpp"
#include "shulib/core/drivetrain.hpp"

namespace shulib {

class MecanumDrive : public Drivetrain {
public:
    MecanumDrive(pros::MotorGroup& frontLeft,
                 pros::MotorGroup& frontRight,
                 pros::MotorGroup& backLeft,
                 pros::MotorGroup& backRight,
                 float wheelDiameter,
                 float rpm)
        : Drivetrain(wheelDiameter, rpm, 0)
    {
        // Define coefficients for mecanum wheels
        // (these would need to be tuned for your specific setup)
        MotorConfig flConfig = {&frontLeft, 1, 1, 1};
        motorConfigs.push_back(flConfig);
        
        MotorConfig frConfig = {&frontRight, -1, 1, -1};
        motorConfigs.push_back(frConfig);
        
        MotorConfig blConfig = {&backLeft, -1, 1, 1};
        motorConfigs.push_back(blConfig);
        
        MotorConfig brConfig = {&backRight, 1, 1, -1};
        motorConfigs.push_back(brConfig);
    }
    
    std::string toString() override { return "MecanumDrive"; }
};

}  // namespace shulib
```

### Step 2: Use It
```cpp
#include "shulib/core/drivetrain/mecanum.hpp"

// In main.cpp
shulib::MecanumDrive drivetrain(fl, fr, bl, br, 4.0, 600);
shulib::Chassis chassis(drivetrain, sensors);
```

The existing `drive()` method will automatically work with your new drivetrain type!

---

## Related Documents

- [Chassis](CHASSIS.md) – The main robot interface
- [Motor Signs](../configuration/MOTOR_SIGNS.md) – How to configure motor directions
- [Adding a Robot](../configuration/ADDING_A_ROBOT.md) – Setting up robot configs

---