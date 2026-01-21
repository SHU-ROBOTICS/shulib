# PID Tuning Guide

**Practical steps for tuning your motion functions**

---

## Table of Contents

1. [Before You Start](#before-you-start)
2. [The Tuning Process](#the-tuning-process)
3. [Tuning moveVertical](#tuning-movevertical)
4. [Tuning rotateTo](#tuning-rotateto)
5. [Tuning moveToPose](#tuning-movetoppose)
6. [Reading Robot Behavior](#reading-robot-behavior)
7. [Quick Fixes](#quick-fixes)
8. [Recording and Comparing](#recording-and-comparing)
9. [Our Tuned Values](#our-tuned-values)
10. [Key Takeaways](#key-takeaways)

---

## Before You Start

### Prerequisites

Before tuning PID, make sure:

- [ ] **Odometry is working** - Position updates correctly when you push the robot
- [ ] **Motors are configured correctly** - Robot drives in expected direction
- [ ] **Tracking wheels are calibrated** - Distances are accurate
- [ ] **Robot is consistent** - Batteries charged, wheels clean, no loose parts

**If odometry is broken, PID tuning is pointless.** Fix odometry first!

### What You'll Need

- Your robot
- A tape measure
- Something to mark distances (tape on floor)
- Terminal access for logging
- 15-30 minutes per function

### Starting Values

If you have no idea where to start:

```cpp
// Linear motion (moveVertical, moveToPose linear)
PID linearPID(5, 0, 0);

// Rotation (rotateTo)
PID rotationPID(1, 0, 0);

// Heading correction
PID headingPID(1, 0, 0);
```

These are conservative. Robot will be slow but shouldn't do anything crazy.

---

## The Tuning Process

### The Universal Method

For ALL PID tuning, follow this sequence:

```
┌─────────────────────────────────────────────────────────────────┐
│                    PID TUNING SEQUENCE                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   Step 1: Set I and D to zero                                  │
│           Start with P only                                     │
│                                                                 │
│   Step 2: Increase P until robot responds quickly              │
│           Stop when you see overshoot                          │
│                                                                 │
│   Step 3: Add D to reduce overshoot                            │
│           Start at P/10, adjust as needed                       │
│                                                                 │
│   Step 4: Add I only if there's steady-state error             │
│           Keep it small!                                        │
│                                                                 │
│   Step 5: Fine-tune all three                                  │
│           Small adjustments based on behavior                   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Why This Order?

- **P first:** It does most of the work
- **D second:** Controls what P causes (overshoot)
- **I last:** Only needed if P+D can't reach target exactly

### The Golden Rule

**Change ONE thing at a time.** If you change P and D together, you won't know which caused the change in behavior.

---

## Tuning moveVertical

### Setup

Mark a distance on the floor (24" is good):

```
    START                           END
      │                              │
      ▼                              ▼
    ──●──────────────────────────────●──
      └──────────── 24" ────────────┘
```

### Test Code

```cpp
void testMoveVertical() {
    chassis.setPose(0, 0, 0);
    
    moveVertical(chassis, 24);
    
    Pose end = chassis.getPose();
    printf("Target: 24.0  Actual Y: %.2f  Error: %.2f\n", 
           end.y, 24.0 - end.y);
    printf("X drift: %.2f  Heading drift: %.2f\n", 
           end.x, end.theta);
}
```

### Step 1: Tune P

```cpp
PID linearPID(5, 0, 0);  // Start here
```

Run test. Observe:
- **Too slow?** Increase P → try 8, then 10, then 12
- **Overshoots?** You've gone too far. Back off.
- **Just right?** Robot approaches quickly, minor overshoot OK

**Target behavior:** Robot accelerates, slows as it approaches, stops near target.

### Step 2: Add D

Once P gives fast response (with some overshoot):

```cpp
PID linearPID(10, 0, 1);  // Added D
```

Run test. Observe:
- **Still overshoots?** Increase D → try 1.5, then 2
- **Jittery/vibrating?** D too high. Reduce.
- **Stops smoothly?** Good!

**Target behavior:** Smooth approach, no overshoot, stops on target.

### Step 3: Add I (Maybe)

Only if robot consistently stops short:

```cpp
PID linearPID(10, 0.5, 1);  // Added I
```

Keep I small. If robot oscillates, reduce I.

### Step 4: Tune Heading Correction

For straight driving, we also need heading correction:

```cpp
const double HEADING_KP = 1.0;
```

Test: Robot should drive straight, not curve.
- **Curves?** Increase HEADING_KP
- **Oscillates side-to-side?** Decrease HEADING_KP

### Final Check

Run the test 5 times. You should get consistent results (within 0.5").

---

## Tuning rotateTo

### Setup

Clear space around robot. Mark the floor at 0°, 90°, 180°, 270°.

### Test Code

```cpp
void testRotateTo() {
    chassis.setPose(0, 0, 0);
    
    printf("Turning to 90 degrees...\n");
    rotateTo(chassis, 90);
    
    Pose end = chassis.getPose();
    printf("Target: 90.0  Actual: %.2f  Error: %.2f\n", 
           end.theta, 90.0 - end.theta);
}
```

### Step 1: Tune P

```cpp
PID rotationPID(1, 0, 0);  // Start conservative
```

Run test. Observe:
- **Turns slowly?** Increase P → try 1.5, then 2
- **Overshoots badly?** P too high. Reduce.
- **Quick turn with small overshoot?** Perfect for now.

**Note:** Rotation is more sensitive than linear. Use smaller P values.

### Step 2: Add D

```cpp
PID rotationPID(1.5, 0, 0.1);  // Added D
```

Run test. Observe:
- **Still overshoots?** Increase D → try 0.15, 0.2
- **Doesn't quite reach target?** D might be too high
- **Stops precisely?** Good!

### Step 3: Add I (Rarely Needed)

Only if robot consistently stops a few degrees off:

```cpp
PID rotationPID(1.5, 0.01, 0.15);  // Tiny I
```

### Test Multiple Angles

```cpp
void testRotationMultiple() {
    chassis.setPose(0, 0, 0);
    
    int targets[] = {90, 180, -90, 0, 45, -45, 135, -135};
    
    for (int target : targets) {
        rotateTo(chassis, target);
        
        float actual = chassis.getPose().theta;
        float error = target - actual;
        printf("%d° → %.1f° (error: %.1f°)\n", target, actual, error);
        
        pros::delay(500);
    }
}
```

All errors should be < 3°.

---

## Tuning moveToPose

### Setup

Mark a grid on the floor if possible, or at least a few key points.

### Test Code

```cpp
void testMoveToPose() {
    chassis.setPose(0, 0, 0);
    
    Pose target(24, 36, 45);
    moveToPose(chassis, target);
    
    Pose end = chassis.getPose();
    float posError = end.distance(target);
    float angError = target.theta - end.theta;
    
    printf("Position error: %.2f inches\n", posError);
    printf("Angle error: %.2f degrees\n", angError);
}
```

### If Using Turn-Then-Drive

You're mostly using `moveVertical` and `rotateTo` - those should already be tuned. Just verify the combination works.

### If Using Continuous Control

Tune two PIDs:

**Linear PID (distance):**
```cpp
PID linearPID(12, 0, 0);  // Start here
```

Same process as moveVertical - increase P until responsive, add D for smoothness.

**Heading PID (steering):**
```cpp
PID headingPID(10, 0, 0);  // Start here
```

- **Robot doesn't curve toward target?** Increase P
- **Robot swerves too much?** Decrease P
- **Robot oscillates?** Add D

### Tune Acceleration/Deceleration

```cpp
const double ACCEL_RATE = 6.0;  // Power increase per cycle
const double DECEL_ZONE = 6.0;  // Inches before target
```

- **Jerky start?** Decrease ACCEL_RATE
- **Overshoots?** Increase DECEL_ZONE
- **Too slow?** Increase ACCEL_RATE, decrease DECEL_ZONE

### The Square Test

Ultimate validation:

```cpp
void testSquare() {
    chassis.setPose(0, 0, 0);
    
    moveToPose(chassis, Pose(0, 24, 0));
    moveToPose(chassis, Pose(24, 24, 90));
    moveToPose(chassis, Pose(24, 0, 180));
    moveToPose(chassis, Pose(0, 0, 270));
    
    Pose end = chassis.getPose();
    printf("Final error: (%.2f, %.2f, %.2f°)\n", 
           end.x, end.y, end.theta);
}
```

Good tuning: Final position within 2" of origin.

---

## Reading Robot Behavior

### Visual Guide

| What You See | What It Means | Adjust |
|--------------|---------------|--------|
| **Slow, sluggish** | P too low | ↑ P |
| **Fast then oscillates** | P too high, D too low | ↓ P or ↑ D |
| **Overshoots once, settles** | D too low | ↑ D |
| **Jittery, vibrating** | D too high | ↓ D |
| **Stops short consistently** | Needs I, or MIN_OUTPUT too low | Add I or ↑ MIN |
| **Creeps forever** | I too high, or tolerance too tight | ↓ I or ↑ tolerance |
| **Violent oscillation** | P way too high | ↓↓ P significantly |

### Response Curves

```
GOOD RESPONSE:
                    ┌────── target
    Position    ────┘
                   ╱
                  ╱
                 ╱
                ╱
    ───────────╱
    0               Time →
    
    Fast rise, smooth approach, no overshoot.


P TOO HIGH (oscillation):
                    ┌────── target
    Position   ╱╲  ╱
              ╱  ╲╱
             ╱
            ╱
    ───────╱
    0               Time →
    
    Goes past, comes back, repeats.


P TOO LOW (sluggish):
                    ┌────── target
    Position        │
                   ╱╱╱
                 ╱╱
              ╱╱
    ─────────╱
    0               Time →
    
    Takes forever to get there.


NEEDS I (steady-state error):
                    ┌────── target
    Position    ────┼─── stops here (short)
                   ╱
                  ╱
                 ╱
    ───────────╱
    0               Time →
    
    Gets close but can't quite reach.
```

---

## Quick Fixes

### "Robot won't move"
```cpp
// Check MIN_OUTPUT
const double MIN_OUTPUT = 25;  // Increase if stuck
```

### "Robot overshoots"
```cpp
// Reduce P, increase D
PID pid(8, 0, 2);  // Was (12, 0, 0.5)
```

### "Robot oscillates forever"
```cpp
// Reduce P significantly, add D
PID pid(5, 0, 1);  // Was (15, 0, 0)

// Or increase tolerance
const double TOLERANCE = 2.0;  // Was 0.5
```

### "Robot stops short"
```cpp
// Add small I
PID pid(10, 0.5, 1);  // Added I

// Or increase MIN_OUTPUT
const double MIN_OUTPUT = 25;  // Was 15
```

### "Rotation keeps going"
```cpp
// Reduce rotation P, add D
PID rotationPID(1.0, 0, 0.2);  // Was (2, 0, 0)
```

### "Drives crooked"
```cpp
// Increase heading correction
const double HEADING_KP = 2.0;  // Was 1.0
```

---

## Recording and Comparing

### Log Your Trials

Keep a tuning log:

```
Date: 2026-01-20
Function: moveVertical

Trial 1: P=5, I=0, D=0
  Result: Slow, takes 3 seconds for 24"

Trial 2: P=10, I=0, D=0
  Result: Fast, overshoots by 3"

Trial 3: P=10, I=0, D=1
  Result: Fast, overshoots by 1"

Trial 4: P=10, I=0, D=2
  Result: Smooth, stops within 0.5"
  ✓ KEEPER

Final values: P=10, I=0, D=2
```

### Telemetry Recording

Enable detailed logging during tuning:

```cpp
void moveVerticalDebug(Chassis& chassis, double distance) {
    // ... setup ...
    
    while (/* running */) {
        // ... calculations ...
        
        // Log every cycle
        printf("%.3f,%.3f,%.3f,%.3f\n", 
               elapsed/1000.0, error, output, current.y);
        
        // ... rest of loop ...
    }
}
```

Copy output to spreadsheet, plot for visualization.

---

## Our Tuned Values

### moveVertical

```cpp
PID linearPID(10, 2.5, 0.3);
const double MAX_OUTPUT = 60.0;
const double MIN_OUTPUT = 20.0;
const double HEADING_KP = 1.0;
const double MAX_CORRECTION = 10.0;
const double TOLERANCE = 1.0;
```

### rotateTo

```cpp
PID rotationPID(1.5, 0.01, 0.15);
const double MAX_ROTATION = 50.0;
const double MIN_ROTATION = 20.0;
const double TOLERANCE = 2.0;
```

### moveToPose

```cpp
PID linearPID(12, 0.03, 0);
PID headingPID(10, 0.005, 0.25);
const double MAX_OUTPUT = 70.0;
const double MIN_OUTPUT = 20.0;
const double MAX_ROTATION = 30.0;
const double ACCEL_RATE = 6.0;
const double DECEL_ZONE = 6.0;
const double POSITION_TOLERANCE = 1.0;
```

**These work for OUR robots.** Your robot may need different values!

---

## Key Takeaways

### The Process

1. **P first** - Get quick response
2. **D second** - Eliminate overshoot
3. **I last** - Only if needed for steady-state
4. **One change at a time** - Know what worked

### Signs of Good Tuning

- Robot responds quickly (not sluggish)
- Stops on target (not short, not over)
- No oscillation
- Consistent results
- Works across different distances/angles

### Signs of Bad Tuning

- Robot oscillates around target
- Takes forever to settle
- Inconsistent results
- Different behavior at different speeds

### Remember

- **Rotation is more sensitive** than linear
- **Every robot is different** - tune for yours
- **Battery level matters** - tune with typical charge
- **Test multiple scenarios** - not just one distance

---

*For PID theory, see [PID.md](../core-library/PID.md)*
*For function details, see [MOVE_VERTICAL.md](./MOVE_VERTICAL.md), [ROTATE_TO.md](./ROTATE_TO.md), [MOVE_TO_POSE.md](./MOVE_TO_POSE.md)*