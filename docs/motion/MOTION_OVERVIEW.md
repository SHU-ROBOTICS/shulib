# Motion Overview

**How autonomous motion works in shulib**

---

## Table of Contents

1. [Introduction](#introduction)
2. [The Motion Functions](#the-motion-functions)
3. [Choosing the Right Function](#choosing-the-right-function)
4. [The Common Pattern](#the-common-pattern)
5. [How Motion Uses Odometry](#how-motion-uses-odometry)
6. [How Motion Uses PID](#how-motion-uses-pid)
7. [Motion Parameters](#motion-parameters)
8. [Combining Motions](#combining-motions)
9. [Building an Autonomous Routine](#building-an-autonomous-routine)
10. [Timeouts and Safety](#timeouts-and-safety)
11. [Debugging Motion](#debugging-motion)
12. [Key Takeaways](#key-takeaways)
13. [Where to Go Next](#where-to-go-next)

---

## Introduction

### What is Autonomous Motion?

Autonomous motion is how your robot moves **without driver input**. During the 15-second autonomous period, the robot must:

1. Know where it is (odometry)
2. Know where it wants to go (target)
3. Calculate how to get there (PID)
4. Execute the movement (motors)
5. Know when it's arrived (feedback)

Our motion system handles all of this with simple function calls.

### The Goal

Turn this thinking:
> "Drive forward 24 inches, then turn right 90 degrees, then drive to the scoring zone"

Into this code:
```cpp
moveVertical(chassis, 24);
rotateTo(chassis, 90);
moveToPose(chassis, Pose(48, 72, 90));
```

Clean, readable, reliable.

---

## The Motion Functions

### Overview

| Function | What It Does | When to Use |
|----------|--------------|-------------|
| `moveVertical()` | Drive forward/backward a distance | Straight-line movements |
| `rotateTo()` | Turn to face a specific angle | Point turns, reorienting |
| `moveToPose()` | Drive to an (x, y) coordinate | Navigating to specific locations |

### Quick Examples

```cpp
// Drive forward 24 inches
moveVertical(chassis, 24);

// Drive backward 12 inches
moveVertical(chassis, -12);

// Turn to face 90° (east)
rotateTo(chassis, 90);

// Turn to face -45° (northwest)
rotateTo(chassis, -45);

// Drive to coordinates (24, 36) 
moveToPose(chassis, Pose(24, 36, 0));

// Drive to coordinates and end facing 90°
moveToPose(chassis, Pose(48, 72, 90));
```

### Function Signatures

```cpp
void moveVertical(Chassis& chassis, double distance);

void rotateTo(Chassis& chassis, double targetAngle);

void moveToPose(Chassis& chassis, Pose target);
```

All functions:
- Take a `Chassis&` reference as the first parameter
- Are **blocking** - they don't return until the movement is complete
- Use odometry for feedback
- Use PID for smooth control

---

## Choosing the Right Function

### Decision Tree

```
What do you need to do?
│
├─► Move in a straight line?
│   │
│   ├─► Yes ──► moveVertical()
│   │           • Forward: positive distance
│   │           • Backward: negative distance
│   │
│   └─► No ──► Continue below
│
├─► Turn in place (no translation)?
│   │
│   ├─► Yes ──► rotateTo()
│   │           • Specify target angle
│   │           • Robot will take shortest path
│   │
│   └─► No ──► Continue below
│
└─► Move to a specific location?
    │
    └─► Yes ──► moveToPose()
                • Handles both translation and rotation
                • Can end at specific heading
```

### Common Scenarios

| Scenario | Best Function | Example |
|----------|---------------|---------|
| Score in front of robot | `moveVertical()` | `moveVertical(chassis, 6)` |
| Face the goal | `rotateTo()` | `rotateTo(chassis, 45)` |
| Drive to scoring zone | `moveToPose()` | `moveToPose(chassis, SCORING_ZONE)` |
| Back away from wall | `moveVertical()` | `moveVertical(chassis, -12)` |
| Turn around | `rotateTo()` | `rotateTo(chassis, 180)` |
| Navigate around obstacle | Multiple `moveToPose()` | See below |

### When NOT to Use Each

| Function | Don't Use When... |
|----------|-------------------|
| `moveVertical()` | You need to move diagonally or curve |
| `rotateTo()` | You need to move while turning |
| `moveToPose()` | Simple straight line is faster/more accurate |

---

## The Common Pattern

All motion functions follow the same basic pattern:

```
┌─────────────────────────────────────────────────────────────────────┐
│                    THE MOTION CONTROL LOOP                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   1. INITIALIZE                                                     │
│      • Reset PID controllers                                        │
│      • Record starting position                                     │
│      • Calculate target                                             │
│                                                                     │
│   2. LOOP (every 10ms)                                             │
│      ┌──────────────────────────────────────────────────────────┐  │
│      │ a. Get current position from odometry                     │  │
│      │ b. Calculate error (target - current)                     │  │
│      │ c. Check if within tolerance → EXIT if yes               │  │
│      │ d. Check timeout → EXIT if exceeded                      │  │
│      │ e. Run error through PID → get motor output              │  │
│      │ f. Apply output to motors                                 │  │
│      │ g. Wait 10ms                                              │  │
│      └──────────────────────────────────────────────────────────┘  │
│                              │                                      │
│                              ▼                                      │
│   3. CLEANUP                                                        │
│      • Stop motors                                                  │
│      • Log result                                                   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### Code Template

```cpp
void motionFunction(Chassis& chassis, /* parameters */) {
    // 1. INITIALIZE
    PID pid(kP, kI, kD);
    pid.reset();
    Pose start = chassis.getPose();
    int elapsed = 0;
    
    // 2. LOOP
    while (elapsed < TIMEOUT) {
        // a. Current position
        Pose current = chassis.getPose();
        
        // b. Calculate error
        float error = /* target - current */;
        
        // c. Check tolerance
        if (fabs(error) < TOLERANCE) {
            break;  // Success!
        }
        
        // d. Timeout handled by while condition
        
        // e. PID calculation
        float output = pid.update(error, 0.01);
        output = std::clamp(output, -MAX_OUTPUT, MAX_OUTPUT);
        
        // f. Apply to motors
        chassis.drive(/* appropriate values */);
        
        // g. Wait
        pros::delay(10);
        elapsed += 10;
    }
    
    // 3. CLEANUP
    chassis.drive(0, 0, 0);
    logger().log("Motion complete");
}
```

---

## How Motion Uses Odometry

### The Feedback Loop

Motion functions don't just blindly spin motors. They continuously check position and adjust:

```
                    ┌─────────────┐
                    │   TARGET    │
                    │  (24, 36)   │
                    └──────┬──────┘
                           │
                           ▼
              ┌────────────────────────┐
              │  error = target - current
              └────────────┬───────────┘
                           │
           ┌───────────────┴───────────────┐
           │                               │
           ▼                               │
    ┌─────────────┐                        │
    │     PID     │                        │
    │ Controller  │                        │
    └──────┬──────┘                        │
           │                               │
           ▼                               │
    ┌─────────────┐      ┌─────────────┐   │
    │   Motors    │ ───► │   Robot     │   │
    │             │      │   Moves     │   │
    └─────────────┘      └──────┬──────┘   │
                                │          │
                                ▼          │
                         ┌─────────────┐   │
                         │  Odometry   │   │
                         │  Updates    │ ──┘
                         │  Position   │
                         └─────────────┘
                         
    This loop runs 100 times per second!
```

### What Each Function Reads

| Function | Odometry Values Used |
|----------|---------------------|
| `moveVertical()` | x, y (for distance), theta (for heading correction) |
| `rotateTo()` | theta only |
| `moveToPose()` | x, y, theta (all three) |

### Why This Matters

Without odometry feedback, you'd have to guess timing:
```cpp
// BAD: Time-based (unreliable)
motors.move(100);
pros::delay(1000);  // Hope this is 24 inches...
motors.stop();
```

With odometry feedback, you go until you arrive:
```cpp
// GOOD: Position-based (reliable)
while (distance_to_target > tolerance) {
    // Keep going until we're actually there
}
```

---

## How Motion Uses PID

### The Role of PID

PID controllers convert position error into motor power:

```
ERROR                    PID                     OUTPUT
(how far off)    ───►   (magic math)    ───►   (motor power)

"12 inches away"  ───►  PID(12, ...)   ───►    "70% power"
"2 inches away"   ───►  PID(2, ...)    ───►    "25% power"
"0.5 inches away" ───►  PID(0.5, ...)  ───►    "10% power"
```

### Different PIDs for Different Motions

Each motion function uses PIDs tuned for its specific purpose:

| Function | PID(s) Used | Tuning Priority |
|----------|-------------|-----------------|
| `moveVertical()` | Linear PID, Heading PID | Smooth stops, straight driving |
| `rotateTo()` | Rotation PID | No overshoot, accurate angles |
| `moveToPose()` | Linear PID, Heading PID | Balance speed and accuracy |

### PID Values in Our Code

```cpp
// Rotation (tuned for accuracy)
PID rotationPID(1.5, 0.01, 0.15);

// Linear motion (tuned for smooth stops)
PID linearPID(5.0, 0.05, 0.2);

// Heading correction (light touch)
PID headingPID(1.0, 0, 0);
```

> **For tuning details, see [PID_TUNING.md](./PID_TUNING.md)**

---

## Motion Parameters

### Tolerances

How close is "close enough"?

```cpp
const double POSITION_TOLERANCE = 1.0;   // inches
const double ANGLE_TOLERANCE = 2.0;      // degrees
```

**Trade-off:**
- Tighter tolerance = more accurate, but slower and might oscillate
- Looser tolerance = faster, but less precise

### Timeouts

Maximum time to spend on a movement:

```cpp
const int MOVE_TIMEOUT = 3000;   // 3 seconds
const int TURN_TIMEOUT = 2000;   // 2 seconds
```

**Why timeouts?**
- Prevent infinite loops if something goes wrong
- Keep autonomous moving if one action fails
- Safety net for competition

### Output Limits

```cpp
const double MIN_OUTPUT = 20.0;   // Overcome static friction
const double MAX_OUTPUT = 70.0;   // Don't go too fast
```

**MIN_OUTPUT:** Below this, motors might not move (friction). We clamp up to this.

**MAX_OUTPUT:** Above this, control becomes less precise. We clamp down to this.

---

## Combining Motions

### Sequential Movements

Most autonomous routines are sequences of motions:

```cpp
void autonomous() {
    // Set starting position
    chassis.setPose(0, 0, 0);
    
    // Execute sequence
    moveVertical(chassis, 24);      // Drive forward
    rotateTo(chassis, 90);          // Turn right
    moveVertical(chassis, 12);      // Drive forward
    rotateTo(chassis, 0);           // Turn back
    moveToPose(chassis, Pose(0, 0, 0));  // Return home
}
```

### Adding Mechanism Actions

Interleave mechanism control with motion:

```cpp
void scoringSequence() {
    // Drive to scoring position
    moveToPose(chassis, SCORING_ZONE);
    
    // Activate mechanism
    mech.setIntake(127);
    pros::delay(500);
    mech.setIntake(0);
    
    // Back away
    moveVertical(chassis, -12);
}
```

### Parallel Actions (Advanced)

For actions during motion, use tasks:

```cpp
void intakeWhileDriving() {
    // Start intake in background
    pros::Task intakeTask([]() {
        mech.setIntake(127);
        pros::delay(2000);
        mech.setIntake(0);
    });
    
    // Drive while intake runs
    moveVertical(chassis, 36);
    
    // Intake task continues/finishes on its own
}
```

---

## Building an Autonomous Routine

### Step 1: Plan the Route

Sketch on paper or use the field diagram:

```
    Start ──► Pickup ──► Score ──► Park
      ●         ●          ●        ●
    (0,0)    (24,12)    (48,36)  (60,0)
```

### Step 2: Identify Key Positions

Define named poses:

```cpp
const Pose START(0, 0, 0);
const Pose PICKUP(24, 12, 45);
const Pose SCORING(48, 36, 90);
const Pose PARKING(60, 0, 0);
```

### Step 3: Write the Sequence

```cpp
void myAutonomous() {
    chassis.setPose(START);
    
    // Go to pickup
    moveToPose(chassis, PICKUP);
    mech.intake(true);
    pros::delay(500);
    
    // Go to scoring
    moveToPose(chassis, SCORING);
    mech.release();
    pros::delay(300);
    
    // Park
    moveToPose(chassis, PARKING);
}
```

### Step 4: Test and Refine

1. Run the routine
2. Watch what goes wrong
3. Adjust positions, add delays, tweak tolerances
4. Repeat

---

## Timeouts and Safety

### Why Timeouts Matter

Without timeouts, a stuck robot loops forever:

```cpp
// DANGEROUS: No timeout
while (error > tolerance) {
    // If we never reach tolerance, infinite loop!
}
```

With timeouts, we eventually move on:

```cpp
// SAFE: Timeout protection
int elapsed = 0;
while (error > tolerance && elapsed < TIMEOUT) {
    elapsed += 10;
    pros::delay(10);
}
if (elapsed >= TIMEOUT) {
    logger().warning("Movement timed out!");
}
```

### Handling Timeout

When a movement times out, you have options:

```cpp
// Option 1: Log and continue (most common)
if (timedOut) {
    logger().warning("Didn't reach target, continuing...");
}
// Next movement starts

// Option 2: Abort autonomous
if (timedOut) {
    logger().error("Critical movement failed, aborting");
    return;  // Exit autonomous
}

// Option 3: Retry
int attempts = 0;
while (timedOut && attempts < 3) {
    // Try again
    attempts++;
}
```

### Competition Consideration

In a match, it's usually better to:
- **Continue** after non-critical timeouts
- **Abort** only if continuing would be dangerous

A partially-completed autonomous is better than a stuck robot.

---

## Debugging Motion

### Enable Logging

All motion functions log their progress:

```
[LOG] moveVertical: target=24.0"
[LOG] moveVertical: current=0.0", error=24.0"
[LOG] moveVertical: current=12.5", error=11.5"
[LOG] moveVertical: current=23.2", error=0.8"
[SUCCESS] moveVertical: reached target in 1250ms
```

### Common Debug Prints

```cpp
void debugMovement() {
    Pose p = chassis.getPose();
    printf("Pose: (%.1f, %.1f, %.1f°)\n", p.x, p.y, p.theta);
}
```

### Watch for These Signs

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| Robot doesn't move | Motor config wrong | Check ports and directions |
| Robot oscillates | PID too aggressive | Lower kP, increase kD |
| Robot stops short | Tolerance too large | Decrease tolerance |
| Robot overshoots | PID needs tuning | Increase kD, lower kP |
| Movement takes forever | MIN_OUTPUT too low | Increase MIN_OUTPUT |
| Timeout every time | Odometry broken | Check tracking wheels |

### Test Each Function Independently

Before building complex routines, verify basics:

```cpp
void testMoveVertical() {
    chassis.setPose(0, 0, 0);
    moveVertical(chassis, 24);
    Pose end = chassis.getPose();
    printf("Expected: (0, 24, 0)\n");
    printf("Actual: (%.1f, %.1f, %.1f)\n", end.x, end.y, end.theta);
}

void testRotateTo() {
    chassis.setPose(0, 0, 0);
    rotateTo(chassis, 90);
    Pose end = chassis.getPose();
    printf("Expected theta: 90\n");
    printf("Actual theta: %.1f\n", end.theta);
}
```

---

## Key Takeaways

### The Essentials

1. **Three main functions:** `moveVertical()`, `rotateTo()`, `moveToPose()`

2. **All use the same pattern:** Initialize → Loop (read, calculate, apply) → Cleanup

3. **Odometry provides feedback:** We know where we are at all times

4. **PID provides smooth control:** Error becomes appropriate motor power

5. **Timeouts prevent disasters:** Never loop forever

6. **Start simple:** Test each function before combining

### Quick Reference

| Want to... | Use... |
|------------|--------|
| Drive straight | `moveVertical(chassis, distance)` |
| Turn in place | `rotateTo(chassis, angle)` |
| Go to coordinates | `moveToPose(chassis, pose)` |

### The One-Sentence Summary

> **Motion functions combine odometry feedback with PID control to reliably move the robot to target positions during autonomous.**

---

## Where to Go Next

| Topic | Document | What You'll Learn |
|-------|----------|-------------------|
| Driving straight | [MOVE_VERTICAL.md](./MOVE_VERTICAL.md) | moveVertical() in detail |
| Turning | [ROTATE_TO.md](./ROTATE_TO.md) | rotateTo() in detail |
| Going to coordinates | [MOVE_TO_POSE.md](./MOVE_TO_POSE.md) | moveToPose() in detail |
| Tuning for your robot | [PID_TUNING.md](./PID_TUNING.md) | Practical tuning steps |
| PID theory | [PID.md](../core-library/PID.md) | Understanding PID math |
| Position tracking | [ODOMETRY.md](../core-library/ODOMETRY.md) | How we know where we are |

---

*Document last updated: January 2026*