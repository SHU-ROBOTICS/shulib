# moveToPose

**Driving to specific coordinates**

**Location:** `src/seasons/pushback_2026/auton.cpp`

---

## Table of Contents

1. [Overview](#overview)
2. [Function Signature](#function-signature)
3. [How It Works](#how-it-works)
4. [The Algorithm](#the-algorithm)
5. [Two Approaches](#two-approaches)
6. [Parameters and Constants](#parameters-and-constants)
7. [Usage Examples](#usage-examples)
8. [Defining Target Poses](#defining-target-poses)
9. [What Can Go Wrong](#what-can-go-wrong)
10. [Tuning moveToPose](#tuning-movetoppose)
11. [Key Takeaways](#key-takeaways)

---

## Overview

### What It Does

`moveToPose()` drives the robot from its current position to a target (x, y) coordinate, optionally ending at a specific heading.

```cpp
// Drive to coordinates (24, 36)
moveToPose(chassis, Pose(24, 36, 0));

// Drive to coordinates and end facing 90°
moveToPose(chassis, Pose(48, 72, 90));
```

### When to Use It

| Use Case | Example |
|----------|---------|
| Navigate across field | `moveToPose(chassis, SCORING_ZONE)` |
| Go to specific location | `moveToPose(chassis, Pose(24, 36, 0))` |
| Multi-waypoint paths | Series of `moveToPose()` calls |
| Return to starting position | `moveToPose(chassis, START)` |

### When NOT to Use It

- **Pure straight line:** `moveVertical()` is more accurate
- **Pure rotation:** `rotateTo()` is more appropriate
- **Very short distances:** Overhead not worth it

### Comparison with Other Functions

| Function | Movement Type | Best For |
|----------|---------------|----------|
| `moveVertical()` | Straight only | Short precise movements |
| `rotateTo()` | Rotation only | Turning in place |
| `moveToPose()` | Any direction | Navigating to locations |

---

## Function Signature

```cpp
void moveToPose(Chassis& chassis, Pose target);
```

### Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `chassis` | `Chassis&` | Reference to the chassis object |
| `target` | `Pose` | Target position and heading |

### Returns

Nothing (`void`). The function **blocks** until the movement is complete or times out.

---

## How It Works

### The Big Picture

```
┌─────────────────────────────────────────────────────────────────┐
│                      moveToPose FLOW                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   Current Position: (10, 20, 0°)                               │
│   Target Position:  (50, 60, 90°)                              │
│                                                                 │
│         Target                                                  │
│           ●  (50, 60)                                          │
│          ╱                                                      │
│         ╱  Robot needs to:                                     │
│        ╱   1. Turn toward target                               │
│       ╱    2. Drive to target                                  │
│      ╱     3. Turn to final heading                            │
│     ╱                                                           │
│    ●                                                            │
│  Start (10, 20)                                                │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Key Concepts

1. **Calculate angle to target** - Which way to face
2. **Calculate distance to target** - How far to go
3. **Control both simultaneously or sequentially**

---

## The Algorithm

### Approach 1: Turn-Then-Drive (Simpler)

```cpp
void moveToPose(Chassis& chassis, Pose target) {
    Pose current = chassis.getPose();
    
    // Step 1: Calculate angle from current to target
    float dx = target.x - current.x;
    float dy = target.y - current.y;
    float angleToTarget = atan2(dx, dy) * 180.0 / M_PI;
    
    // Step 2: Turn to face the target
    rotateTo(chassis, angleToTarget);
    
    // Step 3: Calculate distance
    float distance = current.distance(target);
    
    // Step 4: Drive straight to target
    moveVertical(chassis, distance);
    
    // Step 5: Turn to final heading (if different)
    if (fabs(target.theta - angleToTarget) > 2.0) {
        rotateTo(chassis, target.theta);
    }
}
```

**Pros:**
- Simple and reliable
- Reuses existing functions
- Easy to debug

**Cons:**
- Three-phase motion (turn, drive, turn)
- Slower than curved path
- Stops between phases

### Approach 2: Continuous Control (More Advanced)

```cpp
void moveToPose(Chassis& chassis, Pose target) {
    PID linearPID(12, 0.03, 0);
    PID headingPID(10, 0.005, 0.25);
    linearPID.reset();
    headingPID.reset();
    
    int elapsed = 0;
    const int TIMEOUT = 5000;
    float currentPower = 0;  // For acceleration control
    
    while (elapsed < TIMEOUT) {
        Pose current = chassis.getPose();
        
        // Calculate distance to target
        float dx = target.x - current.x;
        float dy = target.y - current.y;
        float distance = sqrt(dx*dx + dy*dy);
        
        // Check if arrived
        if (distance < 1.0) {
            break;
        }
        
        // Calculate angle to target
        float angleToTarget = atan2(dx, dy) * 180.0 / M_PI;
        
        // Calculate heading error
        float headingError = angleToTarget - current.theta;
        
        // Normalize heading error to [-180, 180]
        while (headingError > 180) headingError -= 360;
        while (headingError < -180) headingError += 360;
        
        // PID outputs
        float linearOutput = linearPID.update(distance, 0.01);
        float turnOutput = headingPID.update(headingError, 0.01);
        
        // Clamp outputs
        linearOutput = std::clamp(linearOutput, -MAX_OUTPUT, MAX_OUTPUT);
        turnOutput = std::clamp(turnOutput, -MAX_ROTATION, MAX_ROTATION);
        
        // Acceleration control (smooth starts)
        float targetPower = linearOutput;
        if (targetPower > currentPower + ACCEL_RATE) {
            currentPower += ACCEL_RATE;
        } else if (targetPower < currentPower - ACCEL_RATE) {
            currentPower -= ACCEL_RATE;
        } else {
            currentPower = targetPower;
        }
        
        // Deceleration zone (smooth stops)
        if (distance < DECEL_ZONE) {
            currentPower *= (distance / DECEL_ZONE);
        }
        
        // Minimum power
        if (fabs(currentPower) < MIN_OUTPUT && distance > 1.0) {
            currentPower = (currentPower > 0) ? MIN_OUTPUT : -MIN_OUTPUT;
        }
        
        // Drive
        chassis.drive(0, currentPower, turnOutput);
        
        pros::delay(10);
        elapsed += 10;
    }
    
    // Final heading adjustment
    if (fabs(target.theta - chassis.getPose().theta) > 2.0) {
        rotateTo(chassis, target.theta);
    }
    
    chassis.drive(0, 0, 0);
}
```

**Pros:**
- Smoother motion
- Faster (curves toward target)
- Continuous correction

**Cons:**
- More complex
- Harder to tune
- May not end precisely on target

---

## Two Approaches

### Comparing the Methods

```
TURN-THEN-DRIVE:

    Start           Phase 1          Phase 2          Phase 3
      ●      →     ● (turn)    →    ────────●   →    ● (turn)
                      ↻                              ↻
    
    Robot stops between each phase.
    Simple but slower.


CONTINUOUS CONTROL:

    Start                                            End
      ●─────────────────────────────────────────────►●
         ↘                                       ↗
           ↘ Curves smoothly toward target     ↗
             ↘                               ↗
               ↘───────────────────────────↗
    
    Robot moves and turns simultaneously.
    Faster but more complex.
```

### Which to Use?

| Situation | Recommended Approach |
|-----------|---------------------|
| Short distances (<12") | Turn-then-drive |
| Precision required | Turn-then-drive |
| Long distances | Continuous |
| Speed critical | Continuous |
| Easy debugging | Turn-then-drive |
| Curved paths | Continuous |

### Our Implementation

We use a **hybrid approach**:
1. Continuous control for most of the movement
2. Final `rotateTo()` for precise heading

---

## Parameters and Constants

### Our Tuned Values

```cpp
// Linear PID
PID linearPID(12, 0.03, 0);

// Heading PID
PID headingPID(10, 0.005, 0.25);

// Output limits
const double MAX_OUTPUT = 70.0;
const double MIN_OUTPUT = 20.0;
const double MAX_ROTATION = 30.0;

// Motion control
const double ACCEL_RATE = 6.0;    // Power increase per cycle
const double DECEL_ZONE = 6.0;    // Inches before target to slow down

// Exit conditions
const double POSITION_TOLERANCE = 1.0;  // inches
const double ANGLE_TOLERANCE = 2.0;     // degrees
const int TIMEOUT = 5000;               // milliseconds
```

### What Each Does

| Constant | Purpose | Effect |
|----------|---------|--------|
| `linearPID` | Controls forward speed | Higher = faster approach |
| `headingPID` | Controls turning | Higher = more aggressive correction |
| `MAX_OUTPUT` | Speed limit | Caps maximum speed |
| `MIN_OUTPUT` | Friction override | Ensures movement |
| `MAX_ROTATION` | Turn rate limit | Prevents over-steering |
| `ACCEL_RATE` | Smooths acceleration | Higher = faster accel |
| `DECEL_ZONE` | Approach distance | Larger = smoother stop |

---

## Usage Examples

### Basic Navigation

```cpp
// Drive to a specific point
moveToPose(chassis, Pose(24, 36, 0));

// Drive to point and face a direction
moveToPose(chassis, Pose(48, 72, 90));

// Return to origin
moveToPose(chassis, Pose(0, 0, 0));
```

### Using Named Locations

```cpp
// Define key locations
const Pose START(0, 0, 0);
const Pose SCORING_ZONE(48, 24, 45);
const Pose LOADING_ZONE(12, 60, -90);
const Pose PARKING(60, 12, 0);

void autonomous() {
    chassis.setPose(START);
    
    // Navigate through locations
    moveToPose(chassis, SCORING_ZONE);
    score();
    
    moveToPose(chassis, LOADING_ZONE);
    pickup();
    
    moveToPose(chassis, SCORING_ZONE);
    score();
    
    moveToPose(chassis, PARKING);
}
```

### Multi-Waypoint Path

```cpp
void navigateAroundObstacle() {
    // Can't go straight - obstacle in the way
    // Use waypoints to go around
    
    Pose waypoint1(12, 36, 0);   // Go left first
    Pose waypoint2(36, 48, 45);  // Then diagonal
    Pose destination(60, 48, 90);
    
    moveToPose(chassis, waypoint1);
    moveToPose(chassis, waypoint2);
    moveToPose(chassis, destination);
}
```

### Relative Movement

```cpp
void moveRelative(float dx, float dy, float dtheta) {
    Pose current = chassis.getPose();
    
    // Calculate new position relative to current
    Pose target(
        current.x + dx,
        current.y + dy,
        current.theta + dtheta
    );
    
    moveToPose(chassis, target);
}

// Usage:
moveRelative(12, 24, 0);  // Move 12" right, 24" forward
```

---

## Defining Target Poses

### Coordinate System Reminder

```
                    +Y (forward)
                       ↑
                       │
                       │
     -X (left) ←───────┼───────→ +X (right)
                       │
                       │
                       ↓
                    -Y (backward)
```

### Finding Coordinates

**Method 1: Measure from field**
```cpp
// Scoring zone is 48" forward, 24" right
const Pose SCORING(24, 48, 45);
```

**Method 2: Use odometry readout**
```cpp
// Drive robot manually to position
// Read coordinates from brain/terminal
// (36.5, 42.3, 0) - use these!
const Pose PICKUP(36.5, 42.3, 0);
```

**Method 3: Calculate from field layout**
```cpp
// Field is 144" × 144"
// Center is at (72, 72)
const Pose FIELD_CENTER(72, 72, 0);
```

### Best Practices

```cpp
// Group related poses
namespace Poses {
    // Starting positions
    const Pose RED_START(12, 12, 0);
    const Pose BLUE_START(132, 12, 180);
    
    // Scoring zones
    const Pose GOAL_LEFT(24, 72, 90);
    const Pose GOAL_RIGHT(120, 72, -90);
    
    // Pickup locations
    const Pose PICKUP_1(48, 36, 0);
    const Pose PICKUP_2(96, 36, 0);
}

// Use like:
moveToPose(chassis, Poses::GOAL_LEFT);
```

---

## What Can Go Wrong

### Problem: Robot Goes Wrong Direction

**Symptoms:** Drives away from target instead of toward it

**Causes & Fixes:**
| Cause | Fix |
|-------|-----|
| Coordinate system confusion | Verify X/Y orientation |
| Starting pose wrong | Check `setPose()` call |
| Angle calculation flipped | Check `atan2` argument order |

### Problem: Robot Curves Wildly

**Symptoms:** Takes strange looping path

**Causes & Fixes:**
| Cause | Fix |
|-------|-----|
| Heading PID too aggressive | Lower headingPID kP |
| Angle normalization broken | Check [-180, 180] wrap |
| Odometry heading wrong | Verify tracking wheels |

### Problem: Stops Short of Target

**Symptoms:** Gets close but not quite there

**Causes & Fixes:**
| Cause | Fix |
|-------|-----|
| TOLERANCE too large | Reduce to 0.5" |
| DECEL_ZONE too large | Reduce decel zone |
| MIN_OUTPUT too low | Increase MIN_OUTPUT |

### Problem: Overshoots Target

**Symptoms:** Goes past, then hunts back and forth

**Causes & Fixes:**
| Cause | Fix |
|-------|-----|
| linearPID kP too high | Reduce kP |
| DECEL_ZONE too small | Increase decel zone |
| MAX_OUTPUT too high | Reduce MAX_OUTPUT |
| Momentum too high | Slow down approach |

### Problem: Final Heading Wrong

**Symptoms:** Position correct but facing wrong way

**Causes & Fixes:**
| Cause | Fix |
|-------|-----|
| Final rotateTo skipped | Check angle tolerance |
| Pose.theta not set | Verify target pose has heading |
| Angle tolerance too large | Reduce tolerance |

### Problem: Takes Forever

**Symptoms:** Very slow or never finishes

**Causes & Fixes:**
| Cause | Fix |
|-------|-----|
| Hunting at target | Increase tolerance |
| PID gains too low | Increase kP values |
| MIN_OUTPUT too low | Increase MIN_OUTPUT |
| MAX_OUTPUT too low | Increase MAX_OUTPUT |

---

## Tuning moveToPose

### Quick Tuning Process

1. **Start with turn-then-drive approach**
   - Easier to debug
   - Uses already-tuned functions

2. **For continuous control, tune linear first:**
   ```cpp
   PID linearPID(5, 0, 0);
   // Increase kP until approach is responsive
   ```

3. **Then tune heading:**
   ```cpp
   PID headingPID(5, 0, 0);
   // Increase kP until robot tracks toward target
   // Add kD if it oscillates
   ```

4. **Adjust acceleration:**
   ```cpp
   ACCEL_RATE = 4;  // Start low
   // Increase for faster acceleration
   ```

5. **Adjust deceleration zone:**
   ```cpp
   DECEL_ZONE = 8;  // Start high
   // Decrease for faster stops (may overshoot)
   ```

### Testing Procedure

```cpp
void testMoveToPose() {
    // Start at origin
    chassis.setPose(0, 0, 0);
    
    // Test simple forward
    moveToPose(chassis, Pose(0, 24, 0));
    printPose("Forward 24", chassis.getPose());
    pros::delay(500);
    
    // Test diagonal
    moveToPose(chassis, Pose(24, 48, 0));
    printPose("Diagonal", chassis.getPose());
    pros::delay(500);
    
    // Test with heading change
    moveToPose(chassis, Pose(24, 48, 90));
    printPose("With heading", chassis.getPose());
    pros::delay(500);
    
    // Return home
    moveToPose(chassis, Pose(0, 0, 0));
    printPose("Home", chassis.getPose());
}

void printPose(const char* label, Pose p) {
    printf("%s: (%.1f, %.1f, %.1f°)\n", 
           label, p.x, p.y, p.theta);
}
```

### Square Test

```cpp
void testSquare() {
    chassis.setPose(0, 0, 0);
    
    // Drive a 24" square
    moveToPose(chassis, Pose(0, 24, 0));
    moveToPose(chassis, Pose(24, 24, 90));
    moveToPose(chassis, Pose(24, 0, 180));
    moveToPose(chassis, Pose(0, 0, 270));
    
    // Should end at (0, 0)
    Pose end = chassis.getPose();
    printf("Error: (%.1f, %.1f, %.1f°)\n", 
           end.x, end.y, end.theta);
}
```

> **For detailed PID tuning theory, see [PID_TUNING.md](./PID_TUNING.md)**

---

## Key Takeaways

1. **Most versatile motion function:** Handles any point-to-point navigation

2. **Specify full pose:** Position (x, y) AND heading (theta)

3. **Two approaches:** Turn-then-drive (simple) or continuous (smooth)

4. **Uses both PIDs:** Linear for distance, heading for direction

5. **Final heading:** Usually a separate `rotateTo()` at the end

### Quick Reference

```cpp
// Basic usage
moveToPose(chassis, Pose(x, y, theta));

// Examples
moveToPose(chassis, Pose(24, 36, 0));    // Go to (24, 36) facing forward
moveToPose(chassis, Pose(48, 72, 90));   // Go to (48, 72) facing right

// Key things to tune:
// - linearPID (approach speed)
// - headingPID (course correction)
// - ACCEL_RATE, DECEL_ZONE (smoothness)
// - POSITION_TOLERANCE (accuracy)
```

---

*For the big picture, see [MOTION_OVERVIEW.md](./MOTION_OVERVIEW.md)*
*For tuning help, see [PID_TUNING.md](./PID_TUNING.md)*