# moveVertical

**Driving straight forward or backward**

**Location:** `src/seasons/pushback_2026/auton.cpp`

---

## Table of Contents

1. [Overview](#overview)
2. [Function Signature](#function-signature)
3. [How It Works](#how-it-works)
4. [The Algorithm](#the-algorithm)
5. [Heading Correction](#heading-correction)
6. [Parameters and Constants](#parameters-and-constants)
7. [Usage Examples](#usage-examples)
8. [What Can Go Wrong](#what-can-go-wrong)
9. [Tuning moveVertical](#tuning-movevertical)
10. [Key Takeaways](#key-takeaways)

---

## Overview

### What It Does

`moveVertical()` drives the robot in a straight line for a specified distance.

- **Positive distance** = Forward
- **Negative distance** = Backward

```cpp
moveVertical(chassis, 24);   // Drive 24 inches forward
moveVertical(chassis, -12);  // Drive 12 inches backward
```

### When to Use It

| Use Case | Example |
|----------|---------|
| Approach game element | `moveVertical(chassis, 6)` |
| Back away from wall | `moveVertical(chassis, -10)` |
| Cross the field | `moveVertical(chassis, 48)` |
| Precise positioning | After `rotateTo()` to fine-tune position |

### When NOT to Use It

- **Diagonal movement:** Use `moveToPose()` instead
- **Curved paths:** Use multiple `moveToPose()` calls
- **While turning:** This function only goes straight

---

## Function Signature

```cpp
void moveVertical(Chassis& chassis, double distance);
```

### Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `chassis` | `Chassis&` | Reference to the chassis object |
| `distance` | `double` | Distance to travel in inches (+ = forward, - = backward) |

### Returns

Nothing (`void`). The function **blocks** until the movement is complete or times out.

---

## How It Works

### The Big Picture

```
┌─────────────────────────────────────────────────────────────────┐
│                    moveVertical FLOW                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   START                                                         │
│     │                                                           │
│     ▼                                                           │
│   Record starting pose                                          │
│     │                                                           │
│     ▼                                                           │
│   ┌─────────────────────────────────────────┐                  │
│   │           CONTROL LOOP                   │                  │
│   │                                          │                  │
│   │  1. Get current pose                     │                  │
│   │  2. Calculate distance traveled          │                  │
│   │  3. Calculate remaining distance         │                  │
│   │  4. Check if done → exit if yes         │                  │
│   │  5. Calculate linear output (PID)        │                  │
│   │  6. Calculate heading correction         │                  │
│   │  7. Apply to motors                      │                  │
│   │  8. Wait 10ms                            │                  │
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

### Key Concept: Distance Traveled

We don't just track Y position - we track **total distance traveled** from the start:

```cpp
// Distance is calculated from starting position
float dx = current.x - start.x;
float dy = current.y - start.y;
float distanceTraveled = sqrt(dx*dx + dy*dy);

// Account for direction
if (distance < 0) {
    distanceTraveled = -distanceTraveled;  // Backward
}

// Error is how much we have left to go
float error = distance - distanceTraveled;
```

**Why not just use Y?** If the robot drifts slightly sideways, pure Y tracking would be inaccurate. Distance calculation handles any drift.

---

## The Algorithm

### Step-by-Step

```cpp
void moveVertical(Chassis& chassis, double distance) {
    // ══════════════════════════════════════════════════════════════
    // SETUP
    // ══════════════════════════════════════════════════════════════
    
    // Record where we started
    Pose start = chassis.getPose();
    float startHeading = start.theta;
    
    // Create PID controller for linear motion
    PID linearPID(10, 2.5, 0.3);
    linearPID.reset();
    
    // Timing
    int elapsed = 0;
    const int TIMEOUT = 3000;  // 3 seconds max
    
    // ══════════════════════════════════════════════════════════════
    // CONTROL LOOP
    // ══════════════════════════════════════════════════════════════
    
    while (elapsed < TIMEOUT) {
        // Get current position
        Pose current = chassis.getPose();
        
        // Calculate how far we've gone
        float dx = current.x - start.x;
        float dy = current.y - start.y;
        float distanceTraveled = sqrt(dx*dx + dy*dy);
        
        // Handle backward movement
        if (distance < 0) {
            distanceTraveled = -distanceTraveled;
        }
        
        // How much is left?
        float error = distance - distanceTraveled;
        
        // Are we there yet?
        if (fabs(error) < 1.0) {  // Within 1 inch
            break;
        }
        
        // Calculate motor output
        float output = linearPID.update(error, 0.01);
        output = std::clamp(output, -MAX_OUTPUT, MAX_OUTPUT);
        
        // Apply minimum power to overcome friction
        if (fabs(output) < MIN_OUTPUT && fabs(error) > 1.0) {
            output = (error > 0) ? MIN_OUTPUT : -MIN_OUTPUT;
        }
        
        // Heading correction (keep driving straight)
        float headingError = startHeading - current.theta;
        float correction = headingError * HEADING_KP;
        correction = std::clamp(correction, -MAX_CORRECTION, MAX_CORRECTION);
        
        // Drive with correction
        chassis.drive(0, output, correction);
        
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
Distance: 24 inches

Time     Traveled    Error    Output    Speed
────────────────────────────────────────────────
0ms      0.0"        24.0"    100%      FAST ████████████
100ms    3.2"        20.8"    87%       FAST ██████████
200ms    7.1"        16.9"    71%       MED  ████████
300ms    11.8"       12.2"    51%       MED  ██████
400ms    16.2"       7.8"     33%       MED  ████
500ms    19.8"       4.2"     18%       SLOW ███
600ms    22.1"       1.9"     MIN%      SLOW ██
700ms    23.5"       0.5"     DONE      ▓
────────────────────────────────────────────────
                                        ↑
                                    Reached target!
```

---

## Heading Correction

### The Problem

When driving "straight," robots drift. Friction differs between sides, wheels aren't perfectly aligned, etc.

```
    Intended path:          Actual path (without correction):
    
         │                           │
         │                          ╱
         │                         ╱
         │                        ╱
         ●                       ●
       Start                   Start
```

### The Solution

We continuously correct to maintain our starting heading:

```cpp
// Calculate how far off our heading is
float headingError = startHeading - currentHeading;

// Proportional correction
float correction = headingError * HEADING_KP;

// Apply to steering
chassis.drive(0, forwardPower, correction);
```

### How It Looks

```
    With heading correction:
    
         │ ← slight turn left
         │
         │ → slight turn right  
         │
         │ ← slight turn left
         │
         ● (stays on path)
       Start
```

The correction is small and continuous, keeping the robot on track.

---

## Parameters and Constants

### Our Tuned Values

```cpp
// Linear PID
PID linearPID(10, 2.5, 0.3);

// Output limits
const double MAX_OUTPUT = 60.0;   // Max motor power
const double MIN_OUTPUT = 20.0;   // Overcome friction

// Heading correction
const double HEADING_KP = 1.0;    // Proportional gain
const double MAX_CORRECTION = 10.0;  // Don't over-correct

// Exit conditions
const double TOLERANCE = 1.0;     // inches
const int TIMEOUT = 3000;         // milliseconds
```

### What Each Does

| Constant | Purpose | If Too Low | If Too High |
|----------|---------|------------|-------------|
| `kP` (10) | Speed of response | Sluggish | Overshoots |
| `kI` (2.5) | Eliminate steady error | Stops short | Overshoots |
| `kD` (0.3) | Damping | Overshoots | Jittery |
| `MAX_OUTPUT` | Top speed | Slow | Less control |
| `MIN_OUTPUT` | Floor power | Can't move | Jerky starts |
| `HEADING_KP` | Correction strength | Drifts | Oscillates |
| `TOLERANCE` | "Close enough" | Never exits | Inaccurate |

---

## Usage Examples

### Basic Forward/Backward

```cpp
// Drive forward 24 inches
moveVertical(chassis, 24);

// Drive backward 12 inches
moveVertical(chassis, -12);
```

### Approach and Score

```cpp
void scoreBlock() {
    // Drive up to scoring zone
    moveVertical(chassis, 18);
    
    // Release block
    mech.releaser(127);
    pros::delay(300);
    mech.releaser(0);
    
    // Back away
    moveVertical(chassis, -6);
}
```

### Combined with Rotation

```cpp
void navigateCorner() {
    // Drive along wall
    moveVertical(chassis, 36);
    
    // Turn corner
    rotateTo(chassis, 90);
    
    // Continue along new wall
    moveVertical(chassis, 24);
}
```

### Multiple Segments

```cpp
void zigZag() {
    // Forward
    moveVertical(chassis, 24);
    rotateTo(chassis, 45);
    
    // Diagonal segment (better with moveToPose, but works)
    moveVertical(chassis, 17);  // ~12" at 45°
    rotateTo(chassis, 0);
    
    // Forward again
    moveVertical(chassis, 24);
}
```

---

## What Can Go Wrong

### Problem: Robot Doesn't Move

**Symptoms:** Motors don't spin, robot sits still

**Causes & Fixes:**
| Cause | Fix |
|-------|-----|
| MIN_OUTPUT too low | Increase to 25-30 |
| Motor ports wrong | Check ROBOT config |
| Motors disconnected | Check cables |
| PID output is 0 | Check error calculation |

### Problem: Robot Overshoots

**Symptoms:** Goes past target, then backs up

**Causes & Fixes:**
| Cause | Fix |
|-------|-----|
| kP too high | Reduce kP |
| kD too low | Increase kD |
| MAX_OUTPUT too high | Reduce MAX_OUTPUT |
| Robot too fast | Lower gear ratio |

### Problem: Robot Stops Short

**Symptoms:** Stops before reaching target

**Causes & Fixes:**
| Cause | Fix |
|-------|-----|
| TOLERANCE too large | Reduce to 0.5" |
| kI too low | Increase kI slightly |
| MIN_OUTPUT too low | Increase MIN_OUTPUT |
| Friction too high | Check for mechanical binding |

### Problem: Robot Curves Instead of Straight

**Symptoms:** Ends up off to one side

**Causes & Fixes:**
| Cause | Fix |
|-------|-----|
| Heading correction off | Increase HEADING_KP |
| One side's motors weak | Check motor health |
| Tracking wheels wrong | Verify tracking config |
| Uneven weight | Balance the robot |

### Problem: Timeout Before Reaching Target

**Symptoms:** Function exits but robot didn't arrive

**Causes & Fixes:**
| Cause | Fix |
|-------|-----|
| TIMEOUT too short | Increase to 5000ms |
| Robot stuck | Check for obstacles |
| Odometry broken | Verify tracking wheels |
| Distance too far | Break into segments |

---

## Tuning moveVertical

### Quick Tuning Process

1. **Start with these values:**
   ```cpp
   PID linearPID(5, 0, 0);  // P only first
   MAX_OUTPUT = 50;
   MIN_OUTPUT = 20;
   ```

2. **Increase kP** until robot responds quickly
   - If it overshoots, you've gone too far

3. **Add kD** to reduce overshoot
   - Start with kD = kP / 30
   - Increase until overshoot is minimal

4. **Add kI** only if needed
   - Only if robot consistently stops short
   - Keep it small (kI < 1)

5. **Adjust MIN_OUTPUT** if robot struggles to start moving

6. **Adjust TOLERANCE** for your precision needs

### Testing Procedure

```cpp
void testMoveVertical() {
    // Reset position
    chassis.setPose(0, 0, 0);
    
    // Test forward
    moveVertical(chassis, 24);
    
    // Check result
    Pose end = chassis.getPose();
    printf("Target: 24.0\"  Actual: %.1f\"\n", end.y);
    printf("X drift: %.1f\"\n", end.x);
    printf("Heading drift: %.1f°\n", end.theta);
    
    // Wait, then test backward
    pros::delay(1000);
    moveVertical(chassis, -24);
    
    // Should be back at origin
    end = chassis.getPose();
    printf("Back at: (%.1f, %.1f, %.1f)\n", end.x, end.y, end.theta);
}
```

> **For detailed PID tuning theory, see [PID_TUNING.md](./PID_TUNING.md)**

---

## Key Takeaways

1. **Simple interface:** Just pass distance (positive = forward, negative = backward)

2. **Uses distance, not just Y:** Handles drift correctly

3. **Heading correction:** Keeps robot driving straight

4. **Blocking function:** Waits until complete or timeout

5. **Tunable:** Adjust PID and constants for your robot

### Quick Reference

```cpp
// Forward
moveVertical(chassis, 24);

// Backward  
moveVertical(chassis, -24);

// Key constants to tune:
// - linearPID gains (kP, kI, kD)
// - MAX_OUTPUT, MIN_OUTPUT
// - HEADING_KP
// - TOLERANCE
```

---

*For the big picture, see [MOTION_OVERVIEW.md](./MOTION_OVERVIEW.md)*
*For tuning help, see [PID_TUNING.md](./PID_TUNING.md)*