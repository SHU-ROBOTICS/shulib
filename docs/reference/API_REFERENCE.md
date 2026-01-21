# API Reference

**Quick reference for all shulib classes and functions**

---

## Table of Contents

1. [Chassis](#chassis)
2. [Pose](#pose)
3. [PID](#pid)
4. [OdomUnit](#odomunit)
5. [Odometry Functions](#odometry-functions)
6. [Motion Functions](#motion-functions)
7. [Logger](#logger)
8. [Mechanisms](#mechanisms)

---

## Chassis

**Header:** `shulib/core/chassis.hpp`

The main robot control class. Handles drivetrain, odometry, and movement.

### Constructor

```cpp
Chassis(Drivetrain* drivetrain, OdomSensors sensors);
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `drivetrain` | `Drivetrain*` | Pointer to drivetrain (TankDrive or XDrive) |
| `sensors` | `OdomSensors` | Struct containing tracking wheel OdomUnits |

### Methods

#### calibrate()
```cpp
void calibrate();
```
Initialize sensors and reset odometry. Call once in `initialize()`.

#### setPose()
```cpp
void setPose(float x, float y, float theta);
void setPose(Pose pose);
```
Set the current position. Use at start of autonomous.

| Parameter | Type | Description |
|-----------|------|-------------|
| `x` | `float` | X position in inches |
| `y` | `float` | Y position in inches |
| `theta` | `float` | Heading in degrees |

#### getPose()
```cpp
Pose getPose();
```
**Returns:** Current `Pose` (x, y, theta).

#### drive()
```cpp
void drive(float strafe, float forward, float turn);
```
Direct drive control.

| Parameter | Type | Range | Description |
|-----------|------|-------|-------------|
| `strafe` | `float` | -127 to 127 | Left/right (X-drive only) |
| `forward` | `float` | -127 to 127 | Forward/backward |
| `turn` | `float` | -127 to 127 | Rotation |

---

## Pose

**Header:** `shulib/core/pose.hpp`

Represents robot position and heading.

### Constructor

```cpp
Pose(float x = 0, float y = 0, float theta = 0);
```

### Members

| Member | Type | Description |
|--------|------|-------------|
| `x` | `float` | X position (inches) |
| `y` | `float` | Y position (inches) |
| `theta` | `float` | Heading (degrees) |

### Operators

| Operator | Returns | Description |
|----------|---------|-------------|
| `Pose + Pose` | `Pose` | Add positions (theta from left operand) |
| `Pose - Pose` | `Pose` | Subtract positions |
| `Pose * float` | `Pose` | Scale position |
| `Pose / float` | `Pose` | Scale position |
| `Pose * Pose` | `float` | Dot product |

### Methods

#### distance()
```cpp
float distance(Pose other);
```
**Returns:** Distance in inches to `other` pose.

#### angle()
```cpp
float angle(Pose other);
```
**Returns:** Angle in **radians** to `other` pose.

#### rotate()
```cpp
Pose rotate(float angle);
```
**Returns:** New pose rotated by `angle` **radians**.

#### lerp()
```cpp
Pose lerp(Pose other, float t);
```
**Returns:** Pose interpolated between `this` and `other` by factor `t` (0-1).

---

## PID

**Header:** `shulib/core/pid.hpp`

PID controller for motion control.

### Constructor

```cpp
PID(float kP, float kI = 0, float kD = 0);
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `kP` | `float` | Proportional gain |
| `kI` | `float` | Integral gain |
| `kD` | `float` | Derivative gain |

### Methods

#### update()
```cpp
float update(float error, float dt);
```
Calculate PID output.

| Parameter | Type | Description |
|-----------|------|-------------|
| `error` | `float` | Current error (target - actual) |
| `dt` | `float` | Time step in seconds |

**Returns:** Motor output value.

#### reset()
```cpp
void reset();
```
Reset integral and derivative state. Call before starting new movement.

### Members

| Member | Type | Description |
|--------|------|-------------|
| `kP` | `float` | Proportional gain (public) |
| `kI` | `float` | Integral gain (public) |
| `kD` | `float` | Derivative gain (public) |

---

## OdomUnit

**Header:** `shulib/core/odomUnit.hpp`

Wraps a tracking wheel sensor.

### Constructor

```cpp
OdomUnit(pros::Rotation* sensor, float diameter, float offset);
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `sensor` | `pros::Rotation*` | Pointer to rotation sensor |
| `diameter` | `float` | Wheel diameter in inches |
| `offset` | `float` | Distance from center in inches |

### Methods

#### reset()
```cpp
void reset();
```
Zero the sensor and internal state.

#### get_travel()
```cpp
float get_travel();
```
**Returns:** Total distance traveled since reset (inches).

#### get_travel_delta()
```cpp
float get_travel_delta();
```
**Returns:** Distance since last call (inches). **Stateful!**

#### get_offset()
```cpp
float get_offset();
```
**Returns:** Offset value (inches).

---

## Odometry Functions

**Header:** `shulib/core/odometry.hpp`

Global functions for odometry control.

### init()
```cpp
void shulib::init(OdomSensors sensors, Drivetrain* drivetrain);
```
Initialize odometry system. Called by `Chassis::calibrate()`.

### update()
```cpp
void shulib::update();
```
Update position from sensors. Called automatically by background task.

### getPose()
```cpp
Pose shulib::getPose();
```
**Returns:** Current pose.

### setPose()
```cpp
void shulib::setPose(Pose pose);
```
Set current pose.

### Correction Factors

```cpp
void shulib::setXCorrectionFactor(float factor);
void shulib::setYCorrectionFactor(float factor);
void shulib::setThetaCorrectionFactor(float factor);
```
Fine-tune odometry scaling (default: 1.0).

---

## Motion Functions

**Header:** `shulib/seasons/pushback_2026/auton.hpp`

Autonomous movement functions.

### moveVertical()
```cpp
void moveVertical(shulib::Chassis& chassis, double distance);
```
Drive straight forward or backward.

| Parameter | Type | Description |
|-----------|------|-------------|
| `chassis` | `Chassis&` | Reference to chassis |
| `distance` | `double` | Distance in inches (+ = forward) |

**Blocking:** Yes

### rotateTo()
```cpp
void rotateTo(shulib::Chassis& chassis, double targetAngle);
```
Turn to face a specific heading.

| Parameter | Type | Description |
|-----------|------|-------------|
| `chassis` | `Chassis&` | Reference to chassis |
| `targetAngle` | `double` | Target heading in degrees |

**Blocking:** Yes

### moveToPose()
```cpp
void moveToPose(shulib::Chassis& chassis, Pose target);
```
Drive to specific coordinates.

| Parameter | Type | Description |
|-----------|------|-------------|
| `chassis` | `Chassis&` | Reference to chassis |
| `target` | `Pose` | Target position and heading |

**Blocking:** Yes

---

## Logger

**Header:** `shulib/core/logger.hpp`

Telemetry and debug logging.

### Access

```cpp
Logger& shulib::logger();
```
**Returns:** Reference to singleton logger instance.

### Log Methods

```cpp
void log(Args... args);
void error(Args... args);
void warning(Args... args);
void success(Args... args);
void announce(Args... args);
void debug(Args... args);
```

All methods accept variadic arguments:
```cpp
logger().log("X position: ", pose.x, " inches");
```

### Telemetry

```cpp
void updateTelemetry(std::string key, T value);
```
Update a telemetry value. Sends only on change.

```cpp
void setTelemetryInterval(int ms);
```
Set telemetry update interval (default: 100ms).

---

## Mechanisms

**Header:** `shulib/seasons/pushback_2026/mechanisms.hpp`

Game-specific mechanism control (Push Back 2026).

### Constructor

```cpp
Mechanisms(const shulib::robots::RobotConfig& config);
```

### Methods

#### init()
```cpp
void init();
```
Initialize motors and pneumatics. Call in `initialize()`.

#### setIntake()
```cpp
void setIntake(int power);
```
Set intake motor power (-127 to 127).

#### setConveyor()
```cpp
void setConveyor(int power);
```
Set conveyor motor power (-127 to 127).

#### setReleaser()
```cpp
void setReleaser(int power);
```
Set releaser motor power (-127 to 127).

#### intakeAndConvey()
```cpp
void intakeAndConvey(int power);
```
Run intake and conveyor together.

#### toggleArm()
```cpp
void toggleArm();
```
Toggle arm pneumatic state.

#### toggleLever()
```cpp
void toggleLever();
```
Toggle lever pneumatic state.

---

## Configuration Structs

**Header:** `shulib/robots/robot_config.hpp`

### RobotConfig

```cpp
struct RobotConfig {
    std::string name;
    DrivetrainConfig drivetrain;
    TrackingConfig tracking;
    MechanismConfig mechanisms;
    PneumaticConfig pneumatics;
};
```

### DrivetrainConfig

```cpp
struct DrivetrainConfig {
    std::vector<int> left_ports;   // Negative = reversed
    std::vector<int> right_ports;
    float wheel_diameter;          // Inches
    float track_width;             // Inches
    int motor_rpm;                 // 100, 200, or 600
};
```

### TrackingConfig

```cpp
struct TrackingConfig {
    int left_port;                 // Negative = reversed
    int right_port;
    int back_port;
    float wheel_diameter;          // Inches
    float left_offset;             // Inches (negative = left)
    float right_offset;            // Inches (positive = right)
    float back_offset;             // Inches
};
```

### MechanismConfig

```cpp
struct MechanismConfig {
    std::vector<int> intake_ports;
    std::vector<int> conveyor_ports;
    std::vector<int> releaser_ports;
};
```

### PneumaticConfig

```cpp
struct PneumaticConfig {
    char arm_port;                 // 'A' through 'H'
    char lever_port;
};
```

---

## OdomSensors

**Header:** `shulib/core/odometry.hpp`

```cpp
struct OdomSensors {
    OdomUnit* left;
    OdomUnit* right;
    OdomUnit* back;
    pros::IMU* imu;               // Optional, can be nullptr
};
```

---

## Quick Type Reference

| Type | File | Description |
|------|------|-------------|
| `Pose` | pose.hpp | Position + heading |
| `PID` | pid.hpp | PID controller |
| `Chassis` | chassis.hpp | Main robot class |
| `OdomUnit` | odomUnit.hpp | Tracking wheel wrapper |
| `OdomSensors` | odometry.hpp | Sensor collection |
| `Logger` | logger.hpp | Telemetry/debug |
| `Mechanisms` | mechanisms.hpp | Game mechanisms |
| `RobotConfig` | robot_config.hpp | Robot settings |

---

*For detailed explanations, see the core-library documentation.*