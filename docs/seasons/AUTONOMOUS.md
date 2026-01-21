# Autonomous

**Writing autonomous routines**

---

## Table of Contents

1. [Overview](#overview)
2. [Autonomous Structure](#autonomous-structure)
3. [The run() Pattern](#the-run-pattern)
4. [Writing a Routine](#writing-a-routine)
5. [Starting Position](#starting-position)
6. [Motion + Mechanisms](#motion--mechanisms)
7. [Timing and Delays](#timing-and-delays)
8. [Multiple Routines](#multiple-routines)
9. [Testing and Debugging](#testing-and-debugging)
10. [Competition Considerations](#competition-considerations)
11. [Key Takeaways](#key-takeaways)

---

## Overview

### What is Autonomous?

The **autonomous period** is 15 seconds at the start of each match where the robot runs pre-programmed routines with no driver input. Good autonomous can:

- Score early points
- Secure strategic positions
- Set up for driver control

### Our Autonomous System

```
config.hpp                 auton.cpp
┌────────────────┐        ┌─────────────────────────┐
│ #define        │        │ void run() {            │
│ AUTON_RED_LEFT │───────►│   #if AUTON_RED_LEFT    │
└────────────────┘        │     redLeft(chassis);   │
                          │   #endif                │
                          │ }                       │
                          └─────────────────────────┘
```

Selection happens at compile time via `config.hpp`.

---

## Autonomous Structure

### File Organization

```
include/shulib/seasons/pushback_2026/
└── auton.hpp        ← Declarations

src/seasons/pushback_2026/
└── auton.cpp        ← Implementations
```

### auton.hpp

```cpp
#pragma once
#include "shulib/core/chassis.hpp"
#include "mechanisms.hpp"

namespace pushback_2026 {
namespace auton {

// Main entry point
void run(shulib::Chassis& chassis, Mechanisms& mech);

// Individual routines
void skills(shulib::Chassis& chassis, Mechanisms& mech);
void redLeft(shulib::Chassis& chassis, Mechanisms& mech);
void redRight(shulib::Chassis& chassis, Mechanisms& mech);
void blueLeft(shulib::Chassis& chassis, Mechanisms& mech);
void blueRight(shulib::Chassis& chassis, Mechanisms& mech);
void test(shulib::Chassis& chassis, Mechanisms& mech);

} // namespace auton
} // namespace pushback_2026
```

### auton.cpp

```cpp
#include "shulib/seasons/pushback_2026/auton.hpp"
#include "config.hpp"

namespace pushback_2026 {
namespace auton {

void run(shulib::Chassis& chassis, Mechanisms& mech) {
    #if defined(AUTON_SKILLS)
        skills(chassis, mech);
    #elif defined(AUTON_RED_LEFT)
        redLeft(chassis, mech);
    // ... etc
    #endif
}

void skills(shulib::Chassis& chassis, Mechanisms& mech) {
    // Implementation
}

// ... other routines

} // namespace auton
} // namespace pushback_2026
```

---

## The run() Pattern

### How It Works

```cpp
void run(shulib::Chassis& chassis, Mechanisms& mech) {
    // Preprocessor selects which routine to run
    // based on config.hpp settings
    
    #if defined(AUTON_SKILLS)
        skills(chassis, mech);
        
    #elif defined(AUTON_RED_LEFT)
        redLeft(chassis, mech);
        
    #elif defined(AUTON_RED_RIGHT)
        redRight(chassis, mech);
        
    #elif defined(AUTON_BLUE_LEFT)
        blueLeft(chassis, mech);
        
    #elif defined(AUTON_BLUE_RIGHT)
        blueRight(chassis, mech);
        
    #elif defined(AUTON_TEST)
        test(chassis, mech);
        
    #else
        // No autonomous selected - do nothing
        
    #endif
}
```

### Changing Autonomous

Edit `config.hpp`:

```cpp
// Uncomment ONE:
// #define AUTON_SKILLS
#define AUTON_RED_LEFT      // ← This one is active
// #define AUTON_RED_RIGHT
// #define AUTON_BLUE_LEFT
// #define AUTON_BLUE_RIGHT
// #define AUTON_TEST
```

Then rebuild:
```bash
pros make clean && pros make
```

### Why Compile-Time Selection?

| Approach | Pros | Cons |
|----------|------|------|
| **Compile-time** (#define) | No code bloat, simple | Must rebuild to change |
| **Runtime** (selector) | Change without rebuild | More code, UI needed |

We use compile-time for simplicity. Future: add brain screen selector.

---

## Writing a Routine

### Basic Template

```cpp
void myRoutine(shulib::Chassis& chassis, Mechanisms& mech) {
    // 1. Set starting position
    chassis.setPose(0, 0, 0);
    
    // 2. Execute sequence
    moveVertical(chassis, 24);
    mech.setIntake(127);
    pros::delay(500);
    mech.setIntake(0);
    
    // 3. Return to safe position (optional)
    moveToPose(chassis, Pose(0, 0, 0));
}
```

### Step-by-Step Process

**Step 1: Define starting position**

Where does the robot start? Set the pose accordingly:

```cpp
// Starting at origin, facing forward
chassis.setPose(0, 0, 0);

// Starting at specific field position
chassis.setPose(12, 24, 45);
```

**Step 2: Plan the route**

Sketch on paper or field diagram:
1. Where do we need to go?
2. What do we need to do at each location?
3. What's the time budget?

**Step 3: Write motion commands**

```cpp
moveVertical(chassis, 24);       // Drive forward 24"
rotateTo(chassis, 90);           // Turn to face right
moveToPose(chassis, Pose(48, 36, 0));  // Go to coordinates
```

**Step 4: Add mechanism actions**

```cpp
mech.setIntake(127);   // Start intake
pros::delay(500);      // Wait for action
mech.setIntake(0);     // Stop intake
```

**Step 5: Test and refine**

Run it, see what breaks, adjust.

---

## Starting Position

### Setting the Pose

The robot needs to know where it starts:

```cpp
// Tell odometry where we are
chassis.setPose(x, y, theta);
```

### Common Starting Positions

```
Field Layout (simplified):
┌─────────────────────────────────────┐
│                                     │
│   BLUE                       RED    │
│   LEFT                       LEFT   │
│    ●                           ●    │
│                                     │
│                                     │
│                                     │
│   BLUE                       RED    │
│   RIGHT                      RIGHT  │
│    ●                           ●    │
│                                     │
└─────────────────────────────────────┘
```

### Origin Strategies

**Strategy 1: Origin at start position**
```cpp
// Robot starts at (0, 0)
chassis.setPose(0, 0, 0);

// All targets are relative to start
moveToPose(chassis, Pose(24, 36, 0));  // 24" right, 36" forward
```

**Strategy 2: Origin at field center**
```cpp
// Robot starts at actual field position
chassis.setPose(-60, -60, 0);  // Bottom-left of field

// Targets are absolute field coordinates
moveToPose(chassis, Pose(0, 0, 0));  // Go to field center
```

**Recommendation:** Use origin at start position. It's simpler.

---

## Motion + Mechanisms

### Interleaving Actions

Most autonomous routines interleave motion and mechanism actions:

```cpp
void scoringRoutine(shulib::Chassis& chassis, Mechanisms& mech) {
    chassis.setPose(0, 0, 0);
    
    // Drive to pickup
    moveToPose(chassis, Pose(24, 12, 0));
    
    // Intake
    mech.intakeAndConvey(127);
    pros::delay(500);
    mech.intakeAndConvey(0);
    
    // Drive to goal
    moveToPose(chassis, Pose(48, 36, 90));
    
    // Score
    mech.setReleaser(127);
    pros::delay(300);
    mech.setReleaser(0);
    
    // Back away
    moveVertical(chassis, -12);
}
```

### Parallel Actions (Advanced)

Run mechanisms while driving:

```cpp
void efficientRoutine(shulib::Chassis& chassis, Mechanisms& mech) {
    chassis.setPose(0, 0, 0);
    
    // Start intake BEFORE driving
    mech.setIntake(127);
    
    // Drive while intaking
    moveToPose(chassis, Pose(24, 12, 0));
    
    // Stop intake when arrived
    mech.setIntake(0);
}
```

Or use tasks for more complex parallelism:

```cpp
void parallelRoutine(shulib::Chassis& chassis, Mechanisms& mech) {
    // Start conveyor in background
    pros::Task conveyorTask([&mech]() {
        mech.setConveyor(127);
        pros::delay(2000);
        mech.setConveyor(0);
    });
    
    // Drive while conveyor runs
    moveToPose(chassis, Pose(48, 36, 0));
    
    // Task finishes on its own
}
```

---

## Timing and Delays

### The 15-Second Budget

Autonomous is only 15 seconds. Budget your time:

```cpp
void timedRoutine(shulib::Chassis& chassis, Mechanisms& mech) {
    chassis.setPose(0, 0, 0);
    
    // Action 1: ~3 seconds
    moveToPose(chassis, Pose(24, 24, 0));  // ~2s
    mech.score();                           // ~1s
    
    // Action 2: ~4 seconds
    moveToPose(chassis, Pose(48, 36, 90)); // ~3s
    mech.score();                           // ~1s
    
    // Action 3: ~3 seconds
    moveToPose(chassis, Pose(60, 12, 0));  // ~2s
    mech.park();                            // ~1s
    
    // Total: ~10 seconds, leaving buffer
}
```

### Using Delays

```cpp
// Fixed delay
pros::delay(500);  // Wait 500ms

// Delay for mechanism action
mech.setIntake(127);
pros::delay(1000);  // Run intake for 1 second
mech.setIntake(0);
```

### Avoiding Wasted Time

```cpp
// BAD: Sequential delays
mech.setIntake(127);
pros::delay(500);   // Waiting...
mech.setIntake(0);
moveVertical(chassis, 24);

// GOOD: Action during motion
mech.setIntake(127);
moveVertical(chassis, 24);  // Intake runs while driving
mech.setIntake(0);
```

---

## Multiple Routines

### Why Multiple Routines?

Different starting positions require different paths:

```
RED LEFT start → Different path than → RED RIGHT start
     ↓                                      ↓
   Goes to left goal              Goes to right goal
```

### Shared Code

Extract common sequences into helpers:

```cpp
// Helper functions
void scoreBlock(Mechanisms& mech) {
    mech.setConveyor(127);
    mech.setReleaser(127);
    pros::delay(500);
    mech.setReleaser(0);
    mech.setConveyor(0);
}

void pickupBlock(Mechanisms& mech) {
    mech.intakeAndConvey(127);
    pros::delay(600);
    mech.intakeAndConvey(0);
}

// Routines use helpers
void redLeft(shulib::Chassis& chassis, Mechanisms& mech) {
    chassis.setPose(0, 0, 0);
    
    moveToPose(chassis, PICKUP_1);
    pickupBlock(mech);
    
    moveToPose(chassis, LEFT_GOAL);
    scoreBlock(mech);
}

void redRight(shulib::Chassis& chassis, Mechanisms& mech) {
    chassis.setPose(0, 0, 0);
    
    moveToPose(chassis, PICKUP_2);
    pickupBlock(mech);
    
    moveToPose(chassis, RIGHT_GOAL);
    scoreBlock(mech);
}
```

### Named Positions

Define key positions as constants:

```cpp
// In auton.cpp or a separate file
namespace Positions {
    const Pose RED_LEFT_START(0, 0, 0);
    const Pose RED_RIGHT_START(0, 0, 0);
    
    const Pose LEFT_GOAL(24, 48, 90);
    const Pose RIGHT_GOAL(120, 48, -90);
    
    const Pose PICKUP_1(36, 24, 0);
    const Pose PICKUP_2(108, 24, 0);
    
    const Pose PARKING_ZONE(72, 12, 0);
}

// Usage
moveToPose(chassis, Positions::LEFT_GOAL);
```

---

## Testing and Debugging

### Test Routine

Always have a test routine:

```cpp
void test(shulib::Chassis& chassis, Mechanisms& mech) {
    chassis.setPose(0, 0, 0);
    
    // Test what you're working on
    moveVertical(chassis, 24);
    
    // Print result
    Pose end = chassis.getPose();
    printf("Ended at: (%.1f, %.1f, %.1f)\n", 
           end.x, end.y, end.theta);
}
```

### Incremental Testing

Don't write the whole routine then test. Test incrementally:

```cpp
void redLeft(shulib::Chassis& chassis, Mechanisms& mech) {
    chassis.setPose(0, 0, 0);
    
    // Step 1: Test this first
    moveToPose(chassis, PICKUP_1);
    return;  // STOP HERE, test
    
    // Step 2: Uncomment when step 1 works
    // pickupBlock(mech);
    // return;  // STOP HERE, test
    
    // Step 3: Continue...
    // moveToPose(chassis, LEFT_GOAL);
}
```

### Logging

Add logging to understand what's happening:

```cpp
void redLeft(shulib::Chassis& chassis, Mechanisms& mech) {
    logger().log("=== RED LEFT AUTO START ===");
    chassis.setPose(0, 0, 0);
    
    logger().log("Moving to pickup");
    moveToPose(chassis, PICKUP_1);
    logger().log("At pickup: ", chassis.getPose().x, ", ", chassis.getPose().y);
    
    logger().log("Picking up block");
    pickupBlock(mech);
    
    logger().log("Moving to goal");
    moveToPose(chassis, LEFT_GOAL);
    
    logger().log("Scoring");
    scoreBlock(mech);
    
    logger().success("=== RED LEFT AUTO COMPLETE ===");
}
```

### Watch the Timeout

Motion functions have timeouts. If something takes too long:

```
[WARNING] moveVertical timeout - didn't reach target
```

This means either:
- Distance was too far
- Robot got stuck
- Odometry is broken

---

## Competition Considerations

### Reliability Over Complexity

A simple routine that works > complex routine that fails.

```cpp
// GOOD: Simple, reliable
void conservative(shulib::Chassis& chassis, Mechanisms& mech) {
    chassis.setPose(0, 0, 0);
    
    moveVertical(chassis, 24);
    mech.score();
    moveVertical(chassis, -12);
}

// RISKY: Complex, might fail
void ambitious(shulib::Chassis& chassis, Mechanisms& mech) {
    chassis.setPose(0, 0, 0);
    
    moveToPose(chassis, POS_1);
    mech.pickup();
    moveToPose(chassis, POS_2);
    mech.score();
    moveToPose(chassis, POS_3);
    mech.pickup();
    moveToPose(chassis, POS_4);
    mech.score();
    moveToPose(chassis, PARK);
}
```

### Fallback Routines

Have simpler fallbacks ready:

```cpp
// config.hpp before competition:
// #define AUTON_RED_LEFT       // Main routine
// #define AUTON_RED_LEFT_SAFE  // Fallback if main fails
```

### Pre-Match Checklist

Before each match:
- [ ] Correct autonomous selected in config.hpp?
- [ ] Robot starting in correct position?
- [ ] Mechanisms in correct starting state?
- [ ] Code uploaded is the right version?

---

## Key Takeaways

### The Essentials

1. **run() selects the routine** based on config.hpp
2. **Set starting pose** at the beginning of every routine
3. **Interleave motion and mechanisms** for efficiency
4. **Budget your 15 seconds** wisely
5. **Test incrementally** - one step at a time
6. **Simple and reliable** beats complex and risky

### Routine Template

```cpp
void myRoutine(shulib::Chassis& chassis, Mechanisms& mech) {
    // 1. Set start position
    chassis.setPose(0, 0, 0);
    
    // 2. Actions
    moveToPose(chassis, TARGET);
    mech.doSomething();
    
    // 3. Continue until done or time runs out
}
```

### Quick Reference

```cpp
// Motion
moveVertical(chassis, 24);           // Forward 24"
rotateTo(chassis, 90);               // Turn to 90°
moveToPose(chassis, Pose(x, y, θ));  // Go to coordinates

// Mechanisms
mech.setIntake(127);                 // Run intake
pros::delay(500);                    // Wait
mech.setIntake(0);                   // Stop

// Timing
pros::delay(ms);                     // Wait ms milliseconds
```

---

*For season structure, see [ADDING_A_SEASON.md](./ADDING_A_SEASON.md)*
*For mechanisms, see [MECHANISMS.md](./MECHANISMS.md)*
*For motion functions, see [motion/MOTION_OVERVIEW.md](../motion/MOTION_OVERVIEW.md)*