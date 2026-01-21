# Glossary

**Terms and definitions for shulib and VEX robotics**

---

## A

**ADI (Analog/Digital Interface)**
The 3-wire ports on the V5 Brain (ports A-H). Used for legacy sensors and pneumatic solenoids.

**Arcade Drive**
A control scheme where one joystick controls both forward/backward and turning.

**Autonomous**
The 15-second period at the start of a match where robots operate without driver input, following pre-programmed routines.

---

## B

**Blocking Function**
A function that doesn't return until its action is complete. Motion functions like `moveVertical()` are blocking.

**Brake Mode**
How a motor behaves when power is set to zero: Coast (free spin), Brake (resist motion), or Hold (actively maintain position).

---

## C

**Cartridge**
The replaceable gear insert in V5 motors that determines speed/torque ratio. Colors: Red (100 RPM), Green (200 RPM), Blue (600 RPM).

**Centidegrees**
1/100th of a degree. V5 rotation sensors report position in centidegrees (36,000 per revolution).

**Chassis**
The main robot control class in shulib. Manages drivetrain and odometry.

**Correction Factor**
A multiplier applied to odometry calculations to compensate for systematic errors.

---

## D

**Dead Wheel**
See: Tracking Wheel.

**Deadzone**
A range near zero where joystick input is ignored to prevent drift from imperfect centering.

**Derivative (D)**
The PID term that responds to how fast the error is changing. Helps prevent overshoot.

**Drivetrain**
The motors and wheels that move the robot. Common types: Tank Drive, X-Drive, Mecanum.

---

## E

**Encoder**
A sensor that measures rotation. V5 motors have built-in encoders; external rotation sensors are also encoders.

**Error**
In PID, the difference between the target and current value. `error = target - current`

---

## F

**Feedback Loop**
A control system where output affects input. Odometry provides feedback for motion control.

---

## G

**Gain**
A multiplier in a control system. PID has three gains: kP, kI, kD.

---

## H

**Heading**
The direction the robot is facing, typically measured in degrees from the starting orientation.

---

## I

**IMU (Inertial Measurement Unit)**
A sensor that measures rotation using gyroscopes. The V5 IMU can provide heading information.

**Integral (I)**
The PID term that accumulates error over time. Helps eliminate steady-state error.

**Integral Windup**
When the integral term accumulates too much while the robot is stuck, causing overshoot when it finally moves.

---

## L

**Lerp (Linear Interpolation)**
A method to find a point between two values. `lerp(a, b, 0.5)` returns the midpoint.

---

## M

**Mechanism**
Any non-drivetrain system on the robot (intake, conveyor, lift, etc.).

**Motor Group**
A PROS class that controls multiple motors as a single unit.

---

## N

**Normalization**
Converting a value to a standard range. Angles are often normalized to [-180°, 180°].

---

## O

**Odometry**
The process of tracking robot position by measuring wheel rotations. Uses tracking wheels in shulib.

**Offset**
The distance from the robot's center of rotation to a tracking wheel.

**Opcontrol (Operator Control)**
The driver-controlled period of a match (1:45 after autonomous).

**Oscillation**
When a system repeatedly overshoots and undershoots the target. Usually indicates PID tuning issues.

**Overshoot**
When the robot passes the target position before settling.

---

## P

**PID (Proportional-Integral-Derivative)**
A control algorithm that calculates output based on error (P), accumulated error (I), and rate of change (D).

**Port**
A connection point on the V5 Brain. Smart ports (1-21) for V5 devices, ADI ports (A-H) for 3-wire devices.

**Pose**
A complete description of robot state: position (x, y) and heading (theta).

**PROS**
Purdue Robotics Operating System. The open-source development environment used for shulib.

**Proportional (P)**
The PID term that responds directly to error magnitude. Larger error = larger output.

---

## R

**Radians**
A unit of angle measurement. 2π radians = 360°. Many math functions use radians.

**Reversed**
A motor or sensor configured to report/move opposite to its default direction. Indicated by negative port numbers in shulib.

**Rotation Sensor**
A V5 sensor that measures rotation. Used for tracking wheels in odometry.

---

## S

**Season**
A competition year with a specific game. shulib organizes game-specific code by season.

**Setpoint**
The target value in a control system. Same as "target."

**Slew Rate**
The maximum rate of change for a value. Used to limit acceleration.

**Solenoid**
An electromagnetic valve that controls pneumatic pistons.

**Steady-State Error**
The persistent difference between target and actual position after the system has settled.

---

## T

**Tank Drive**
A drivetrain with wheels on left and right sides. Turns by running sides at different speeds.

**Telemetry**
Data sent from the robot for monitoring and debugging.

**Theta (θ)**
The heading angle in a pose. Measured in degrees in shulib.

**Tolerance**
How close is "close enough." Movement functions exit when error is less than tolerance.

**Tracking Wheel**
An unpowered wheel connected to a rotation sensor for odometry. Also called "dead wheel."

**Track Width**
The distance between the left and right wheels of a drivetrain.

---

## U

**Undershoot**
When the robot doesn't quite reach the target position.

---

## V

**V5**
The current VEX robotics platform (V5 Brain, V5 Motors, V5 Sensors).

**VEX**
The company that produces the robotics competition platform and hardware.

---

## W

**Windup**
See: Integral Windup.

---

## X

**X-Drive**
A drivetrain with four wheels mounted at 45° angles. Can move in any direction (omnidirectional).

---

## Symbols

**kP, kI, kD**
The gain constants for Proportional, Integral, and Derivative terms in PID.

**θ (Theta)**
Greek letter commonly used for angles. Represents heading in Pose.

**Δ (Delta)**
Greek letter meaning "change in." ΔY means change in Y.

---

## Units Quick Reference

| Quantity | Unit | Notes |
|----------|------|-------|
| Distance | Inches | All shulib distances |
| Angle (user-facing) | Degrees | Pose.theta, rotateTo() |
| Angle (math functions) | Radians | atan2(), rotate() |
| Motor power | -127 to 127 | Percentage of max |
| Sensor position | Centidegrees | 36,000 per revolution |
| Time | Milliseconds | pros::delay(), timeouts |

---

*For unit conversions, see [UNITS.md](./UNITS.md)*
*For coordinate system details, see [COORDINATE_SYSTEM.md](./COORDINATE_SYSTEM.md)*