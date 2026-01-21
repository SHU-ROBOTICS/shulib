# Chassis

The Chassis class is the main interface for controlling your robot. It wraps the drivetrain, odometry, and sensors into a single, easy-to-use object.

---

## Table of Contents

- [Overview](#overview)
- [Class Definition](#class-definition)
- [Creating a Chassis](#creating-a-chassis)
- [API Reference](#api-reference)
- [Usage Examples](#usage-examples)
- [How It Works Internally](#how-it-works-internally)
- [Related Classes](#related-classes)

---

## Overview

### What is Chassis?

Chassis is the abstraction layer between your code and the physical robot. Instead of directly controlling motors and reading sensors, you interact with Chassis methods:
```cpp
// Instead of this (low-level):
leftMotors.move(50);
rightMotors.move(50);
double x = calculateXFromEncoders();

// You do this (high-level):
chassis.drive(0, 50, 0);
Pose pos = chassis.getPose();
```

### What Chassis Manages

| Component | Description |
|-----------|-------------|
| **Drivetrain** | Motor control (TankDrive or XDrive) |
| **OdomSensors** | Tracking wheels and optional IMU |
| **Position** | X, Y, Theta via odometry system |
| **Calibration** | Sensor initialization and reset |

### Key Benefits

- **Simplified API** – One object to control everything
- **Abstraction** – Don't need to know motor details
- **Consistency** – Same interface regardless of drivetrain type
- **Background Tracking** – Odometry runs automatically in a separate task

---

## Class Definition

From `include/shulib/core/chassis.hpp`:
```cpp
namespace shulib {

class Chassis {
public:
    Chassis(Drivetrain drivetrain, OdomSensors sensors);
    
    void calibrate(bool calibrateImu = true);
    
    void setPose(float x, float y, float theta, bool radians = false);
    void setPose(Pose pose, bool radians = false);
    Pose getPose(bool radians = false);
    
    void setBrakeMode(pros::motor_brake_mode_e mode);
    void drive(int horizontal, int vertical, int turn, bool fieldCentric = false);
    
    void moveToLocalPose(Pose p, bool async = true);
    void resetLocalPosition();
    void init();

protected:
    bool motionRunning = false;
    bool motionQueued = false;
    float distTraveled = 0;
    Drivetrain drivetrain;
    OdomSensors sensors;

private:
    pros::Mutex mutex;
};

}  // namespace shulib
```

---

## Creating a Chassis

Chassis is typically created in `main.cpp` using values from the robot config.

### Step-by-Step Creation
```cpp
#include "shulib/core/chassis.hpp"
#include "shulib/core/drivetrain/tankdrive.hpp"
#include "shulib/core/odomUnit.hpp"

// 1. Create motor groups from config
pros::MotorGroup leftMotors(ROBOT.drivetrain.left_ports);
pros::MotorGroup rightMotors(ROBOT.drivetrain.right_ports);

// 2. Create rotation sensors for tracking wheels
pros::Rotation leftRotation(ROBOT.tracking.left_port);
pros::Rotation rightRotation(ROBOT.tracking.right_port);
pros::Rotation backRotation(ROBOT.tracking.back_port);

// 3. Create odometry units (tracking wheels)
shulib::OdomUnit leftOdom(&leftRotation, 
                          ROBOT.tracking.wheel_diameter, 
                          ROBOT.tracking.left_offset);
shulib::OdomUnit rightOdom(&rightRotation, 
                           ROBOT.tracking.wheel_diameter, 
                           ROBOT.tracking.right_offset);
shulib::OdomUnit backOdom(&backRotation, 
                          ROBOT.tracking.wheel_diameter, 
                          ROBOT.tracking.back_offset);

// 4. Create drivetrain
shulib::TankDrive drivetrain(leftMotors, rightMotors,
                              ROBOT.drivetrain.track_width,
                              ROBOT.drivetrain.wheel_diameter,
                              ROBOT.drivetrain.rpm);

// 5. Create sensor bundle (nullptr = no IMU)
shulib::OdomSensors sensors(&leftOdom, &rightOdom, &backOdom, nullptr);

// 6. Create chassis
shulib::Chassis chassis(drivetrain, sensors);
```

### With IMU (Optional)
```cpp
// If using an IMU for heading
pros::Imu imu(IMU_PORT);
shulib::OdomSensors sensors(&leftOdom, &rightOdom, &backOdom, &imu);
shulib::Chassis chassis(drivetrain, sensors);
```

---

## API Reference

### Constructor
```cpp
Chassis(Drivetrain drivetrain, OdomSensors sensors);
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `drivetrain` | `Drivetrain` | TankDrive or XDrive instance |
| `sensors` | `OdomSensors` | Bundle of tracking wheels and optional IMU |

---

### calibrate()

Initialize sensors and start the odometry background task.
```cpp
void calibrate(bool calibrateImu = true);
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `calibrateImu` | `bool` | `true` | Whether to calibrate IMU (if present) |

**What it does:**
1. Calibrates IMU if present and `calibrateImu` is true (takes ~2 seconds)
2. Verifies all tracking wheels are initialized (throws error if not)
3. Resets all tracking wheel encoders to zero
4. Sets initial pose to (0, 0, 0)
5. Starts the background odometry task
6. Rumbles the controller to indicate success (single pulse `.`)

**Example:**
```cpp
void initialize() {
    chassis.calibrate(false);  // false = skip IMU calibration
}
```

**Error Handling:**
- Throws `std::runtime_error` if any tracking wheel is nullptr
- If IMU calibration fails after 5 attempts, IMU is disabled and tracking wheels are used instead
- Controller rumbles `---` on IMU calibration failure

---

### drive()

Move the robot with horizontal, vertical, and turn components.
```cpp
void drive(int horizontal, int vertical, int turn, bool fieldCentric = false);
```

| Parameter | Type | Range | Description |
|-----------|------|-------|-------------|
| `horizontal` | `int` | -127 to 127 | Strafe power (ignored by TankDrive) |
| `vertical` | `int` | -127 to 127 | Forward/backward power |
| `turn` | `int` | -127 to 127 | Rotation power |
| `fieldCentric` | `bool` | true/false | If true, movement relative to field not robot |

**Tank Drive Behavior:**
```cpp
// Left motor:  vertical + turn
// Right motor: vertical - turn
```

| Example Call | Left Power | Right Power | Result |
|--------------|------------|-------------|--------|
| `drive(0, 50, 0)` | 50 | 50 | Forward |
| `drive(0, -50, 0)` | -50 | -50 | Backward |
| `drive(0, 0, 50)` | 50 | -50 | Turn right |
| `drive(0, 0, -50)` | -50 | 50 | Turn left |
| `drive(0, 50, 20)` | 70 | 30 | Forward + curve right |

**Field Centric Mode:**
When `fieldCentric = true`, the movement direction is relative to the field, not the robot. The robot uses its current heading to transform the input.
```cpp
// Field centric transformation (from drivetrain.cpp):
double angle = shulib::getPose().theta;
double cosA = cos(angle);
double sinA = sin(angle);
horizontal = horizontal * cosA - vertical * sinA;
vertical = horizontal * sinA + vertical * cosA;
```

---

### getPose()

Get the current position and heading.
```cpp
Pose getPose(bool radians = false);
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `radians` | `bool` | `false` | If true, theta in radians; otherwise degrees |

**Returns:** `Pose` object with `x`, `y`, `theta`.

**Important:** Internally, odometry stores theta in **radians**. This function converts to degrees by default.
```cpp
// From chassis.cpp:
Pose shulib::Chassis::getPose(bool radians) {
    Pose pose = shulib::getPose(true);  // Get in radians
    if (!radians) pose.theta = radToDeg(pose.theta);  // Convert if needed
    return pose;
}
```

**Examples:**
```cpp
// Get position in degrees (default)
Pose pos = chassis.getPose();
printf("X: %.1f, Y: %.1f, Theta: %.1f°\n", pos.x, pos.y, pos.theta);

// Get position in radians
Pose posRad = chassis.getPose(true);
printf("Theta: %.3f rad\n", posRad.theta);
```

---

### setPose()

Set the current position and heading (for resetting odometry).
```cpp
void setPose(float x, float y, float theta, bool radians = false);
void setPose(Pose pose, bool radians = false);
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `x` | `float` | X position in inches |
| `y` | `float` | Y position in inches |
| `theta` | `float` | Heading (degrees by default, radians if specified) |
| `radians` | `bool` | Whether theta is in radians |

**Examples:**
```cpp
// Reset to origin
chassis.setPose(0, 0, 0);

// Set to specific starting position
chassis.setPose(24, 12, 90);  // X=24", Y=12", facing 90°

// Set using Pose object
Pose startPos(24, 12, 90);
chassis.setPose(startPos);
```

---

### setBrakeMode()

Set the motor brake mode for all drive motors.
```cpp
void setBrakeMode(pros::motor_brake_mode_e mode);
```

| Mode | Constant | Description |
|------|----------|-------------|
| Coast | `pros::E_MOTOR_BRAKE_COAST` | Motors spin freely when stopped |
| Brake | `pros::E_MOTOR_BRAKE_BRAKE` | Motors resist movement when stopped |
| Hold | `pros::E_MOTOR_BRAKE_HOLD` | Motors actively hold position when stopped |

**Recommendation:**
- Use **Hold** for autonomous (more accurate stops)
- Use **Coast** for driver control (smoother feel)
```cpp
void autonomous() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    // ...
}

void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // ...
}
```

---

### resetLocalPosition()

Reset X and Y to zero while keeping the current heading.
```cpp
void resetLocalPosition();
```

**What it does:**
```cpp
void shulib::Chassis::resetLocalPosition() {
    float theta = this->getPose().theta;  // Preserve heading
    shulib::setPose(shulib::Pose(0, 0, theta), false);  // Reset X, Y
}
```

**Use case:** When you want to measure relative movement from the current position without losing heading information.

---

## Usage Examples

### Basic Autonomous Movement
```cpp
void autonomous() {
    chassis.setPose(0, 0, 0);  // Start at origin
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    
    // Drive forward for 2 seconds
    chassis.drive(0, 50, 0);
    pros::delay(2000);
    chassis.drive(0, 0, 0);  // Stop
    
    // Turn right for 1 second
    chassis.drive(0, 0, 50);
    pros::delay(1000);
    chassis.drive(0, 0, 0);  // Stop
}
```

### Position-Based Movement
```cpp
void autonomous() {
    chassis.setPose(0, 0, 0);
    
    // Drive forward until Y reaches 24 inches
    while (chassis.getPose().y < 24) {
        chassis.drive(0, 50, 0);
        pros::delay(10);
    }
    chassis.drive(0, 0, 0);
    
    printf("Final Y: %.1f\n", chassis.getPose().y);
}
```

### Driver Control
```cpp
void opcontrol() {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    
    while (true) {
        int forward = controller.get_analog(ANALOG_LEFT_Y);
        int turn = controller.get_analog(ANALOG_RIGHT_X);
        
        // Apply deadband
        if (abs(forward) < 10) forward = 0;
        if (abs(turn) < 10) turn = 0;
        
        chassis.drive(0, forward, turn);
        
        pros::delay(10);
    }
}
```

### Print Position on Button Press
```cpp
void opcontrol() {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    
    while (true) {
        // ... drive code ...
        
        if (controller.get_digital_new_press(DIGITAL_B)) {
            Pose pos = chassis.getPose();
            printf("Position: (%.1f, %.1f) @ %.1f°\n", 
                   pos.x, pos.y, pos.theta);
        }
        
        pros::delay(10);
    }
}
```

---

## How It Works Internally

### Initialization Flow
```
calibrate(false)
    │
    ├── Check sensors.left != nullptr (throw error if null)
    ├── Check sensors.right != nullptr
    ├── Check sensors.back != nullptr
    │
    ├── Reset all tracking wheels to 0
    │   ├── sensors.left->reset()
    │   ├── sensors.right->reset()
    │   └── sensors.back->reset()
    │
    ├── Set pose to (0, 0, 0)
    │
    ├── Call setSensors(sensors, drivetrain)
    │   └── Stores references for odometry system
    │
    ├── Call init_odometry()
    │   └── Starts background task that:
    │       • Runs update() every 10ms
    │       • Logs telemetry (position, battery, temps)
    │
    └── Rumble controller "." to indicate success
```

### Drive Command Flow
```
chassis.drive(0, 50, 30)
    │
    └── drivetrain.drive(0, 50, 30, false)
        │
        │  // For each motor config in motorConfigs:
        │
        ├── Left motors (coefficients: h=0, v=1, turn=1)
        │   output = 0*0 + 50*1 + 30*1 = 80
        │   leftMotors.move(80)
        │
        └── Right motors (coefficients: h=0, v=1, turn=-1)
            output = 0*0 + 50*1 + 30*(-1) = 20
            rightMotors.move(20)
```

### Odometry Background Task
```
Every 10ms (in background task):
    │
    ├── Call update()
    │   ├── Read tracking wheel deltas
    │   ├── Calculate rotation from left/right difference
    │   ├── Calculate forward movement
    │   ├── Calculate strafe (back wheel)
    │   └── Update global pose using rotation matrix
    │
    ├── Log odometry telemetry (if changed)
    ├── Log motor temperatures
    ├── Log battery status
    └── Log controller status
```

---

## Related Classes

| Class | Description | Documentation |
|-------|-------------|---------------|
| `Drivetrain` | Base class for motor control | [Drivetrain](DRIVETRAIN.md) |
| `TankDrive` | Tank drive implementation | [Drivetrain](DRIVETRAIN.md) |
| `XDrive` | X-drive implementation | [Drivetrain](DRIVETRAIN.md) |
| `OdomSensors` | Bundle of tracking sensors | [Odometry](ODOMETRY.md) |
| `OdomUnit` | Single tracking wheel | [OdomUnit](ODOM_UNIT.md) |
| `Pose` | Position and heading | [Pose](POSE.md) |

---