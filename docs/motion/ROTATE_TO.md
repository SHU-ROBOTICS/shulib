# rotateTo

**Turning to face a specific angle**

**Location:** `src/seasons/pushback_2026/auton.cpp`

---

## Table of Contents

1. [Overview](#overview)
2. [Function Signature](#function-signature)
3. [Understanding Angles](#understanding-angles)
4. [How It Works](#how-it-works)
5. [The Algorithm](#the-algorithm)
6. [Shortest Path Turning](#shortest-path-turning)
7. [Parameters and Constants](#parameters-and-constants)
8. [Usage Examples](#usage-examples)
9. [What Can Go Wrong](#what-can-go-wrong)
10. [Tuning rotateTo](#tuning-rotateto)
11. [Key Takeaways](#key-takeaways)

---

## Overview

### What It Does

`rotateTo()` turns the robot in place to face a specific heading angle.

```cpp
rotateTo(chassis, 90);   // Turn to face 90° (east)
rotateTo(chassis, 0);    // Turn to face 0° (forward/north)
rotateTo(chassis, -45);  // Turn to face -45° (northwest)
```

### When to Use It

| Use Case | Example |
|----------|---------|
| Face a scoring goal | `rotateTo(chassis, 45)` |
| Turn around | `rotateTo(chassis, 180)` |
| Align with wall | `rotateTo(chassis, 0)` |
| Face pickup zone | `rotateTo(chassis, -90)` |

### When NOT to Use It

- **Turning while driving:** Use `moveToPose()` instead
- **Small corrections:** Heading correction in `moveVertical()` handles this
- **Arc movements:** Not what this function does

---

## Function Signature

```cpp
void rotateTo(Chassis& chassis, double targetAngle);
```

### Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `chassis` | `Chassis&` | Reference to the chassis object |
| `targetAngle` | `double` | Target heading in **degrees** |

### Returns

Nothing (`void`). The function **blocks** until the turn is complete or times out.

---

## Understanding Angles

### Our Coordinate System

```
                    0° (forward)
                       ↑
                       │
                       │
     -90° (left) ←─────┼─────→ 90° (right)
                       │
                       │
                       ↓
                ±180° (backward)
```

### Angle Reference

| Angle | Direction | Description |
|-------|-----------|-------------|
| 0° | Forward | Initial robot heading |
| 90° | Right | Perpendicular right |
| -90° or 270° | Left | Perpendicular left |
| 180° or -180° | Backward | Facing opposite of start |
| 45° | Front-right | Diagonal |
| -45° | Front-left | Diagonal |

### Equivalent Angles

These are the same direction:
- 90° = -270°
- -90° = 270°
- 180° = -180°
- 360° = 0°

The function handles this automatically - you can use any representation.

---

## How It Works

### The Big Picture

```
┌─────────────────────────────────────────────────────────────────┐
│                      rotateTo FLOW                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   START                                                         │
│     │                                                           │
│     ▼                                                           │
│   Calculate shortest turn direction                             │
│     │                                                           │
│     ▼                                                           │
│   ┌─────────────────────────────────────────┐                  │
│   │           CONTROL LOOP                   │                  │
│   │                                          │                  │
│   │  1. Get current heading                  │                  │
│   │  2. Calculate angle error                │                  │
│   │  3. Normalize to [-180°, 180°]          │                  │
│   │  4. Check if done → exit if yes         │                  │
│   │  5. Calculate turn output (PID)          │                  │
│   │  6. Apply to motors (spin in place)      │                  │
│   │  7. Wait 10ms                            │                  │
│   │                                          │                  │
│   └─────────────────────────────────────────┘                  │
│     │                                                           │
│     ▼                                                           │
│   Stop motors                                                   │
│     │                                                           │
│     ▼                                                           │
│   END                                                           │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Key Concept: Spin In Place

Unlike `moveVertical()`, we want ZERO forward motion. We spin the left and right sides in opposite directions:

```
      ROTATE LEFT (counter-clockwise):
      
      ┌─────────────────────┐
      │ ↓               ↑   │
      │ LEFT         RIGHT  │
      │ backward    forward │
      └─────────────────────┘
              ↺
              
      ROTATE RIGHT (clockwise):
      
      ┌─────────────────────┐
      │ ↑               ↓   │
      │ LEFT         RIGHT  │
      │ forward    backward │
      └─────────────────────┘
              ↻
```

---

## The Algorithm

### Step-by-Step

```cpp
void rotateTo(Chassis& chassis, double targetAngle) {
    // ══════════════════════════════════════════════════════════════
    // SETUP
    // ══════════════════════════════════════════════════════════════
    
    // Create PID controller for rotation
    PID rotationPID(1.5, 0.01, 0.15);
    rotationPID.reset();
    
    // Timing
    int elapsed = 0;
    const int TIMEOUT = 2000;  // 2 seconds max
    
    // ══════════════════════════════════════════════════════════════
    // CONTROL LOOP
    // ══════════════════════════════════════════════════════════════
    
    while (elapsed < TIMEOUT) {
        // Get current heading
        float currentAngle = chassis.getPose().theta;
        
        // Calculate error
        float error = targetAngle - currentAngle;
        
        // Normalize to [-180, 180] (shortest path)
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        
        // Are we there yet?
        if (fabs(error) < 2.0) {  // Within 2 degrees
            break;
        }
        
        // Calculate motor output
        float output = rotationPID.update(error, 0.01);
        
        // Clamp to limits
        output = std::clamp(output, -MAX_ROTATION, MAX_ROTATION);
        
        // Apply minimum power if needed
        if (fabs(output) < MIN_ROTATION && fabs(error) > 2.0) {
            output = (error > 0) ? MIN_ROTATION : -MIN_ROTATION;
        }
        
        // Spin in place (no forward/strafe, only turn)
        chassis.drive(0, 0, output);
        
        // Wait and track time
        pros::delay(10);
        elapsed += 10;
    }
    
    // ══════════════════════════════════════════════════════════════
    // CLEANUP
    // ══════════════════════════════════════════════════════════════
    
    chassis.drive(0, 0, 0);  // Stop
}
```

### Visual: What Happens Over Time

```
Target: 90° (turning right from 0°)

Time     Current    Error    Output    Rotation
────────────────────────────────────────────────
0ms      0°         90°      MAX       FAST →→→→→
100ms    15°        75°      HIGH      FAST →→→→
200ms    35°        55°      MED       MED  →→→
300ms    55°        35°      MED       MED  →→
400ms    72°        18°      LOW       SLOW →
500ms    84°        6°       MIN       SLOW →
600ms    89°        1°       DONE      ▓
────────────────────────────────────────────────
                                       ↑
                                   Reached target!
```

---

## Shortest Path Turning

### The Problem

Without normalization, turning from 10° to 350° would go:
- The "long way": 10° → 350° (340° turn!)
- When the "short way" is: 10° → -10° (only 20° turn)

### The Solution

We normalize the error to always be between -180° and 180°:

```cpp
// Normalize to [-180, 180]
while (error > 180) error -= 360;
while (error < -180) error += 360;
```

### Examples

| From | To | Raw Error | Normalized | Turn Direction |
|------|-----|-----------|------------|----------------|
| 0° | 90° | 90° | 90° | Right |
| 0° | -90° | -90° | -90° | Left |
| 0° | 270° | 270° | -90° | Left (shorter!) |
| 350° | 10° | -340° | 20° | Right (shorter!) |
| 10° | 350° | 340° | -20° | Left (shorter!) |

### Visual

```
    Target: 270° from current 0°
    
    THE LONG WAY (270°):        THE SHORT WAY (-90°):
    
         ↑ 0°                         ↑ 0°
         │                            │
    ┌────┤                       ┌────┤
    │    │                       │    │
    │    ● → → →                 │    ●
    │         ↓                  │    ↑
    │         ↓                  │    │
    └──── 270° ←                 └────┴──── 270°
    
    Turns 270° right           Turns 90° left
    (BAD - too far!)           (GOOD - shortest path!)
```

---

## Parameters and Constants

### Our Tuned Values

```cpp
// Rotation PID (more conservative than linear)
PID rotationPID(1.5, 0.01, 0.15);

// Output limits
const double MAX_ROTATION = 50.0;   // Don't spin too fast
const double MIN_ROTATION = 20.0;   // Overcome static friction

// Exit conditions
const double TOLERANCE = 2.0;       // degrees
const int TIMEOUT = 2000;           // milliseconds
```

### What Each Does

| Constant | Purpose | If Too Low | If Too High |
|----------|---------|------------|-------------|
| `kP` (1.5) | Response speed | Sluggish | Overshoots/oscillates |
| `kI` (0.01) | Eliminate small error | Stops slightly off | Oscillates |
| `kD` (0.15) | Damping | Overshoots | Jittery |
| `MAX_ROTATION` | Top turn speed | Slow turns | Overshoots |
| `MIN_ROTATION` | Floor power | Won't turn | Jerky |
| `TOLERANCE` | "Close enough" | Never exits | Inaccurate |

### Why Different from Linear?

Rotation is more sensitive than linear motion:
- Small angle errors are very visible
- Momentum carries through turns easily
- Overshooting looks bad and wastes time

So we use:
- Lower kP (1.5 vs 10)
- Lower MAX_OUTPUT (50 vs 60)
- More kD relative to kP

---

## Usage Examples

### Basic Turns

```cpp
// Face right
rotateTo(chassis, 90);

// Face left
rotateTo(chassis, -90);

// Face backward
rotateTo(chassis, 180);

// Face forward (reset)
rotateTo(chassis, 0);
```

### Turn and Drive Pattern

```cpp
void navigateSquare() {
    // Start facing forward
    
    moveVertical(chassis, 24);  // Forward
    rotateTo(chassis, 90);      // Turn right
    
    moveVertical(chassis, 24);  // Forward (now going right)
    rotateTo(chassis, 180);     // Turn right
    
    moveVertical(chassis, 24);  // Forward (now going back)
    rotateTo(chassis, -90);     // Turn right
    
    moveVertical(chassis, 24);  // Forward (now going left)
    rotateTo(chassis, 0);       // Turn right (back to start heading)
}
```

### Face Target Before Moving

```cpp
void approachTarget(Pose target) {
    Pose current = chassis.getPose();
    
    // Calculate angle to target
    float angleToTarget = current.angle(target) * 180 / M_PI;
    
    // Turn to face it
    rotateTo(chassis, angleToTarget);
    
    // Now drive straight toward it
    float distance = current.distance(target);
    moveVertical(chassis, distance);
}
```

### Align with Field Elements

```cpp
void alignToGoal() {
    // Goal is at 45° from our starting position
    rotateTo(chassis, 45);
    
    // Now facing the goal, can drive or shoot
}
```

### Turn Around

```cpp
void turnAround() {
    Pose current = chassis.getPose();
    float newHeading = current.theta + 180;
    
    // Normalize (rotateTo handles this, but for clarity)
    if (newHeading > 180) newHeading -= 360;
    
    rotateTo(chassis, newHeading);
}
```

---

## What Can Go Wrong

### Problem: Robot Doesn't Turn

**Symptoms:** Motors don't move, robot sits still

**Causes & Fixes:**
| Cause | Fix |
|-------|-----|
| MIN_ROTATION too low | Increase to 25-30 |
| Motor ports wrong | Check ROBOT config |
| PID output is 0 | Check error calculation |
| Already at target | Verify current vs target angle |

### Problem: Robot Oscillates

**Symptoms:** Turns past target, then back, then past again...

**Causes & Fixes:**
| Cause | Fix |
|-------|-----|
| kP too high | Reduce kP |
| kD too low | Increase kD |
| TOLERANCE too tight | Increase to 3-5° |
| Robot too fast | Lower MAX_ROTATION |

### Problem: Robot Stops Short

**Symptoms:** Turns most of the way but stops a few degrees off

**Causes & Fixes:**
| Cause | Fix |
|-------|-----|
| kI too low | Increase kI slightly |
| MIN_ROTATION too low | Increase MIN_ROTATION |
| TOLERANCE too large | Decrease tolerance |
| Friction | Check for mechanical binding |

### Problem: Turns the Wrong Way

**Symptoms:** Goes the long way around

**Causes & Fixes:**
| Cause | Fix |
|-------|-----|
| Angle normalization broken | Check while loops |
| Odometry heading wrong | Verify tracking sensors |
| Sign error in code | Check error calculation |

### Problem: Inconsistent Accuracy

**Symptoms:** Sometimes accurate, sometimes off

**Causes & Fixes:**
| Cause | Fix |
|-------|-----|
| Odometry drift | Check tracking wheel mounting |
| Wheel slippage | Check for debris on wheels |
| Inconsistent friction | Clean the field/wheels |

---

## Tuning rotateTo

### Quick Tuning Process

1. **Start with P only:**
   ```cpp
   PID rotationPID(1, 0, 0);
   MAX_ROTATION = 40;
   MIN_ROTATION = 20;
   ```

2. **Increase kP** until robot turns crisply
   - Stop when it starts to overshoot

3. **Add kD** to eliminate overshoot
   - Start with kD = kP / 10
   - Increase until overshoot stops

4. **Add kI** only if needed
   - Only if robot consistently stops a few degrees off
   - Keep very small (0.01-0.05)

5. **Adjust TOLERANCE** for accuracy needs
   - Tighter = more accurate but slower
   - 2-3° is usually good enough

### Testing Procedure

```cpp
void testRotation() {
    chassis.setPose(0, 0, 0);
    
    // Test 90° turn
    rotateTo(chassis, 90);
    Pose end = chassis.getPose();
    printf("Target: 90°  Actual: %.1f°\n", end.theta);
    pros::delay(500);
    
    // Test 180° turn
    rotateTo(chassis, 180);
    end = chassis.getPose();
    printf("Target: 180°  Actual: %.1f°\n", end.theta);
    pros::delay(500);
    
    // Test turn back to 0°
    rotateTo(chassis, 0);
    end = chassis.getPose();
    printf("Target: 0°  Actual: %.1f°\n", end.theta);
    
    // Check position drift (should still be at origin)
    printf("Position drift: (%.1f, %.1f)\n", end.x, end.y);
}
```

### Multiple Angle Test

```cpp
void testMultipleAngles() {
    chassis.setPose(0, 0, 0);
    
    int angles[] = {45, 90, 135, 180, -135, -90, -45, 0};
    
    for (int angle : angles) {
        rotateTo(chassis, angle);
        
        Pose end = chassis.getPose();
        float error = angle - end.theta;
        printf("Target: %d°  Actual: %.1f°  Error: %.1f°\n", 
               angle, end.theta, error);
        
        pros::delay(300);
    }
}
```

> **For detailed PID tuning theory, see [PID_TUNING.md](./PID_TUNING.md)**

---

## Key Takeaways

1. **Specify target angle in degrees:** 0° = forward, 90° = right, etc.

2. **Shortest path automatic:** Function always turns the short way

3. **Spins in place:** No forward/backward motion

4. **More sensitive than linear:** Use lower PID gains

5. **Blocking function:** Waits until complete or timeout

### Quick Reference

```cpp
// Turn to specific angles
rotateTo(chassis, 90);    // Face right
rotateTo(chassis, -90);   // Face left
rotateTo(chassis, 180);   // Face backward
rotateTo(chassis, 0);     // Face forward

// Key constants to tune:
// - rotationPID gains (kP, kI, kD)
// - MAX_ROTATION, MIN_ROTATION
// - TOLERANCE
```

---

*For the big picture, see [MOTION_OVERVIEW.md](./MOTION_OVERVIEW.md)*
*For tuning help, see [PID_TUNING.md](./PID_TUNING.md)*