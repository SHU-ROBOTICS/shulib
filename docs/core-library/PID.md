# PID Controller

**Location:** `include/shulib/core/pid.hpp` and `src/core/pid.cpp`

---

## Table of Contents

1. [What is PID?](#what-is-pid)
2. [The Control Problem](#the-control-problem)
3. [The Three Terms Explained](#the-three-terms-explained)
4. [The Mathematics](#the-mathematics)
5. [Visualizing PID Behavior](#visualizing-pid-behavior)
6. [Our Implementation](#our-implementation)
7. [API Reference](#api-reference)
8. [Using PID in Practice](#using-pid-in-practice)
9. [Common PID Configurations](#common-pid-configurations)
10. [The Art of Gain Selection](#the-art-of-gain-selection)
11. [When PID Isn't Enough](#when-pid-isnt-enough)
12. [Debugging PID Issues](#debugging-pid-issues)
13. [Key Takeaways](#key-takeaways)
14. [Where to Go Next](#where-to-go-next)

---

## What is PID?

### The Simple Explanation

**PID** stands for **Proportional-Integral-Derivative**. It's a control algorithm that helps your robot get to where it wants to be - smoothly, quickly, and accurately.

Think of it like this: You're in a car trying to maintain exactly 60 mph.

- **Without PID**: You floor it until you hit 60, then let off completely. You overshoot to 65, slow down to 55, speed back up to 63... you're constantly oscillating.

- **With PID**: You apply gas proportionally to how far you are from 60. As you get close, you ease off. If you've been under 60 for a while, you push a little harder. If you're approaching 60 quickly, you let off early. Smooth and stable.

PID is the mathematical way to describe "smart" control that humans do intuitively.

### The Formal Definition

A **PID controller** continuously calculates an error value (the difference between where you want to be and where you are) and applies a correction based on:

- **P**roportional: How far are you from the target right now?
- **I**ntegral: How long have you been away from the target?
- **D**erivative: How fast are you approaching (or leaving) the target?

The output is a weighted sum of these three terms, and that output drives your motors.

### Why It Matters for Robotics

In VEX, we use PID for virtually everything:

| Application | What's Being Controlled | Target |
|-------------|------------------------|--------|
| Driving straight | Motor power | Specific distance |
| Turning | Motor power | Specific angle |
| Arm position | Motor power | Specific height |
| Flywheel speed | Motor power | Specific RPM |
| Heading correction | Turn rate | Maintain angle |

Without PID, movements are jerky, inaccurate, and unreliable. With PID, they're smooth, precise, and repeatable.

---

## The Control Problem

### The Setup

Let's define the basic control problem:

```
┌─────────────────────────────────────────────────────────────────┐
│                     THE CONTROL LOOP                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│    ┌──────────┐     ┌────────────┐     ┌──────────┐            │
│    │  TARGET  │     │    PID     │     │  ROBOT   │            │
│    │ (setpoint)│────►│ CONTROLLER │────►│ (plant)  │            │
│    └──────────┘     └────────────┘     └────┬─────┘            │
│         │                                    │                  │
│         │           ┌────────────┐           │                  │
│         │           │   ERROR    │           │                  │
│         └──────────►│  = target  │◄──────────┘                  │
│                     │  - actual  │    (feedback)                │
│                     └────────────┘                              │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**Key terms:**
- **Setpoint (target)**: Where you want to be (e.g., 24 inches forward)
- **Process variable (actual)**: Where you actually are (from sensors)
- **Error**: The difference between target and actual
- **Controller output**: The command sent to motors
- **Plant**: The thing being controlled (your robot)

### The Feedback Loop

PID is a **closed-loop** controller, meaning it constantly checks the result of its actions:

```
Time 0ms:   Target = 24"    Actual = 0"     Error = 24"   → High power
Time 100ms: Target = 24"    Actual = 8"     Error = 16"   → Medium-high power
Time 200ms: Target = 24"    Actual = 18"    Error = 6"    → Medium power
Time 300ms: Target = 24"    Actual = 23"    Error = 1"    → Low power
Time 400ms: Target = 24"    Actual = 24"    Error = 0"    → Stop
```

The controller continuously adjusts based on feedback. This is fundamentally different from "open-loop" control where you just send commands and hope for the best.

### What Makes Control Hard?

If it were easy, we'd just say "error × constant = output." But real systems have complications:

1. **Inertia**: The robot doesn't stop instantly when you cut power
2. **Friction**: Static friction requires extra force to overcome
3. **Delays**: There's time between commanding and seeing results
4. **Disturbances**: Getting bumped, battery voltage changes, etc.
5. **Non-linearity**: Behavior changes at different speeds/positions

PID addresses these challenges through its three terms working together.

---

## The Three Terms Explained

### P - Proportional (The "Right Now" Term)

**What it does:** Produces output proportional to the current error.

**The math:** `P_output = kP × error`

**Analogy:** You're walking toward a wall. The closer you get, the slower you walk. The farther away, the faster you go. Your speed is *proportional* to your distance from the wall.

```
ERROR vs P OUTPUT (assuming kP = 5):

Error:     24"    18"    12"    6"     3"     1"     0"
           │      │      │      │      │      │      │
P Output: 120    90     60     30     15     5      0
           ▼      ▼      ▼      ▼      ▼      ▼      ▼
          MAX    ███    ██     █      ▪      ·      
```

**Characteristics:**
- ✅ Simple and intuitive
- ✅ Responds immediately to error
- ✅ Larger error = larger response
- ❌ Can't eliminate steady-state error alone (more on this later)
- ❌ Too high → oscillation

**In robotics:** P is usually the dominant term. Start tuning here.

---

### I - Integral (The "History" Term)

**What it does:** Accumulates past errors over time. If you've been off-target for a while, it pushes harder.

**The math:** `I_output = kI × ∫error dt` (sum of all past errors × time)

**Analogy:** You're trying to push a heavy box. You push, but it doesn't move (friction). You keep pushing. The longer you push without movement, the harder you push, until finally you overcome the friction and it moves.

```
ACCUMULATED ERROR OVER TIME:

Time:    0    100ms  200ms  300ms  400ms  500ms
Error:   5"    5"     5"     5"     4"     3"
         │     │      │      │      │      │
         ▼     ▼      ▼      ▼      ▼      ▼
Integral: 0   0.5    1.0    1.5    1.9    2.2
(sum)     ─────────────────────────────────────►
                    Keeps growing!
```

**Characteristics:**
- ✅ Eliminates steady-state error
- ✅ Overcomes constant disturbances (friction, gravity)
- ✅ Ensures you eventually reach target exactly
- ❌ Can cause overshoot (accumulated push continues past target)
- ❌ "Wind-up" problem if error persists too long
- ❌ Responds slowly

**In robotics:** I is often small or zero. Use it when P alone can't reach the target exactly.

---

### D - Derivative (The "Future" Term)

**What it does:** Responds to the *rate of change* of error. If you're approaching the target quickly, it slows you down early.

**The math:** `D_output = kD × (d/dt)error` (how fast error is changing)

**Analogy:** You're driving toward a stop sign. Even though you're still 100 feet away, you start braking because you're going 60 mph. D responds to *velocity* toward the target, not just distance.

```
ERROR RATE OF CHANGE:

Time:     0ms    100ms   200ms   300ms   400ms
Error:    24"     18"     12"     8"      6"
Change:    -      -6"     -6"     -4"     -2"
          │       │       │       │       │
          ▼       ▼       ▼       ▼       ▼
D output:  0     HIGH    HIGH    MED     LOW
                (braking) (braking)
                
Error is DECREASING, so derivative is NEGATIVE,
which REDUCES output (acts as a brake).
```

**Characteristics:**
- ✅ Reduces overshoot
- ✅ Dampens oscillations
- ✅ Anticipates future behavior
- ❌ Amplifies sensor noise (noisy derivative)
- ❌ Can cause jitter if too high
- ❌ Doesn't help with steady-state error

**In robotics:** D provides damping. Use it to smooth out oscillations from P.

---

### How They Work Together

The three terms complement each other:

```
┌────────────────────────────────────────────────────────────────────────┐
│                    THE PID ORCHESTRA                                    │
├────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  P (Proportional)     "We're far from target - GO!"                    │
│  ████████████████     Main driving force                               │
│                       Gets you most of the way there                    │
│                                                                         │
│  I (Integral)         "We've been stuck - push HARDER!"                │
│  ████                 Overcomes obstacles                               │
│                       Eliminates lingering error                        │
│                                                                         │
│  D (Derivative)       "We're approaching fast - SLOW DOWN!"            │
│  ██████               Prevents overshoot                                │
│                       Smooths the arrival                               │
│                                                                         │
│  ───────────────────────────────────────────────────────────           │
│  Combined Output = P + I + D                                            │
│                                                                         │
└────────────────────────────────────────────────────────────────────────┘
```

**A typical movement:**

```
Phase 1: FAR FROM TARGET
├── P: Large (main thrust)
├── I: Small (just started accumulating)
├── D: Moderate (accelerating toward target)
└── Result: Strong forward power

Phase 2: APPROACHING TARGET
├── P: Decreasing (getting closer)
├── I: Growing (been away from target for a while)
├── D: Negative/braking (approaching quickly)
└── Result: Moderate power, starting to slow

Phase 3: NEAR TARGET
├── P: Small (almost there)
├── I: Moderate (accumulated history)
├── D: Near zero (speed is low)
└── Result: Fine adjustments

Phase 4: AT TARGET
├── P: Zero (no error)
├── I: Decaying (no new error to add)
├── D: Zero (no change)
└── Result: Hold position
```

---

## The Mathematics

### The Continuous Form

In calculus notation, the ideal PID controller is:

```
output(t) = kP·e(t) + kI·∫e(τ)dτ + kD·(de/dt)
```

Where:
- `e(t)` = error at time t
- `kP, kI, kD` = the "gains" (tuning constants)
- `∫e(τ)dτ` = integral of error over time
- `de/dt` = derivative of error with respect to time

### The Discrete Form (What We Actually Use)

Computers can't do true calculus - they work in discrete time steps. We approximate:

```
output[n] = kP·e[n] + kI·Σe[i]·Δt + kD·(e[n] - e[n-1])/Δt
```

Where:
- `n` = current time step
- `n-1` = previous time step
- `Δt` = time between updates
- `Σe[i]·Δt` = running sum of (error × time)

### Breaking Down Each Term

**Proportional:**
```
P = kP × error

Example: kP = 5, error = 10"
P = 5 × 10 = 50
```

**Integral:**
```
I = kI × (accumulated_error)
accumulated_error += error × Δt

Example: kI = 0.5, error = 10" for 0.1 seconds
accumulated_error = 10 × 0.1 = 1.0 inch-seconds
I = 0.5 × 1.0 = 0.5

After another 0.1 seconds still at error = 10":
accumulated_error = 1.0 + (10 × 0.1) = 2.0 inch-seconds
I = 0.5 × 2.0 = 1.0
```

**Derivative:**
```
D = kD × (error - previous_error) / Δt

Example: kD = 0.1, current_error = 10", previous_error = 12", Δt = 0.01s
rate = (10 - 12) / 0.01 = -200 inches/second
D = 0.1 × (-200) = -20

(Negative because error is DECREASING - this acts as a brake)
```

### Putting It Together

```cpp
float PID::update(float error, float time) {
    // Derivative: rate of change of error
    float derivative = (error - prevError) / time;
    
    // Integral: accumulated error over time
    integral += error * time;
    
    // Save for next iteration
    prevError = error;
    
    // Combine all three terms
    return (kP * error) + (kI * integral) + (kD * derivative);
}
```

---

## Visualizing PID Behavior

### Response to a Step Change

Imagine the robot is at position 0 and suddenly needs to be at position 100:

```
                    TARGET = 100
                    ─────────────────────────────────────────
                    
     P ONLY (kP too low):                P ONLY (kP too high):
     ─────────────────────               ─────────────────────
100 │                    ___            │    ╱╲   ╱╲
    │               ____╱               │   ╱  ╲_╱  ╲___
 75 │          ____╱                    │  ╱
    │     ____╱                         │ ╱
 50 │ ___╱                              │╱
    │╱        Never quite               │     Oscillates!
  0 └───────────────────────            └───────────────────────
         Time →                               Time →
    "Steady-state error"                "Underdamped"


     P + I (eliminates error):          P + D (smooth arrival):
     ─────────────────────               ─────────────────────
100 │                ________           │            __________
    │           ____╱                   │        ___╱
 75 │      ____╱                        │    ___╱
    │  ___╱                             │  _╱
 50 │_╱                                 │_╱
    │                                   │       No overshoot!
  0 └───────────────────────            └───────────────────────
         Time →                               Time →
    "I pushes to target"                "D prevents overshoot"


     P + I + D (well-tuned):
     ───────────────────────
100 │           ____________
    │       ___╱
 75 │    __╱
    │  _╱
 50 │_╱    Fast rise, no overshoot,
    │      reaches target exactly!
  0 └───────────────────────
         Time →
    "The goal!"
```

### Terminology for Response Characteristics

| Term | Meaning |
|------|---------|
| **Rise time** | How fast you get close to target |
| **Overshoot** | How much you go past the target |
| **Settling time** | How long until you stay at target |
| **Steady-state error** | Remaining error after settling |
| **Oscillation** | Bouncing back and forth around target |
| **Damping** | How quickly oscillations die out |

### Effect of Each Gain

| Gain | Rise Time | Overshoot | Settling Time | Steady-State Error |
|------|-----------|-----------|---------------|-------------------|
| ↑ kP | Faster | Increases | Mixed | Decreases (but never zero) |
| ↑ kI | Faster | Increases | Increases | Eliminates |
| ↑ kD | Mixed | Decreases | Decreases | No effect |

---

## Our Implementation

### The PID Class

```cpp
// pid.hpp
#pragma once

namespace shulib {

class PID {
public:
    // Constructor - set gains at creation
    PID(float kP, float kI, float kD) : kP(kP), kI(kI), kD(kD) {}

    // Calculate output given current error and time step
    float update(float error, float time);

    // Modify gains on the fly
    void setKP(float newKP);
    void setKI(float newKI);
    void setKD(float newKD);

    // Reset accumulated state (important when starting new movement)
    void reset();

protected:
    float kP;           // Proportional gain
    float kI;           // Integral gain
    float kD;           // Derivative gain

    float integral = 0;     // Running sum of error×time
    float prevError = 0;    // Previous error for derivative
};

} // namespace shulib
```

### The Update Function

```cpp
// pid.cpp
#include "shulib/core/pid.hpp"

float shulib::PID::update(float error, float time) {
    // ═══════════════════════════════════════════════════════════════
    // DERIVATIVE TERM
    // ═══════════════════════════════════════════════════════════════
    // Rate of change of error.
    // (current_error - previous_error) / time_elapsed
    //
    // If error is DECREASING (approaching target), this is NEGATIVE,
    // which reduces output (braking effect).
    
    float derivative = (error - prevError) / time;
    
    // ═══════════════════════════════════════════════════════════════
    // INTEGRAL TERM
    // ═══════════════════════════════════════════════════════════════
    // Accumulated error over time.
    // Each cycle, we add (error × time) to the running total.
    //
    // This grows as long as there's error, pushing harder and harder
    // until the error is eliminated.
    
    integral += error * time;
    
    // ═══════════════════════════════════════════════════════════════
    // SAVE STATE FOR NEXT ITERATION
    // ═══════════════════════════════════════════════════════════════
    // We need the current error for next cycle's derivative calculation.
    
    prevError = error;
    
    // ═══════════════════════════════════════════════════════════════
    // COMBINE ALL TERMS
    // ═══════════════════════════════════════════════════════════════
    // The magic formula: weighted sum of P, I, and D terms.
    
    return (kP * error) + (kI * integral) + (kD * derivative);
}
```

### Reset Function

```cpp
void shulib::PID::reset() {
    integral = 0;      // Clear accumulated error
    prevError = 0;     // Clear previous error
}
```

**When to call reset():**
- Before starting a new movement
- When switching targets
- After the robot has been manually moved
- Any time the controller state is "stale"

```cpp
// Example: Starting a new movement
pid.reset();  // Clear old state!
while (!atTarget) {
    float error = target - current;
    float output = pid.update(error, 0.01);
    motor.move(output);
    pros::delay(10);
}
```

### Setter Functions

```cpp
void shulib::PID::setKP(float newKP) {
    this->kP = newKP;
}

void shulib::PID::setKI(float newKI) {
    this->kI = newKI;
}

void shulib::PID::setKD(float newKD) {
    this->kD = newKD;
}
```

These allow runtime gain adjustment, useful for:
- Different conditions (loaded vs unloaded)
- Adaptive control
- Debugging/tuning

---

## API Reference

### Constructor

```cpp
PID(float kP, float kI, float kD)
```

Creates a new PID controller with the specified gains.

**Parameters:**
- `kP`: Proportional gain (typically 1-20 for position, 0.1-2 for velocity)
- `kI`: Integral gain (typically 0-1, often very small)
- `kD`: Derivative gain (typically 0-1)

**Example:**
```cpp
// Create a PID controller for turning
shulib::PID turnPID(1.5, 0.01, 0.15);

// Create a PID controller for driving straight
shulib::PID drivePID(5.0, 0.05, 0.2);
```

---

### update()

```cpp
float update(float error, float time)
```

Calculates the control output based on current error.

**Parameters:**
- `error`: Current error (target - actual). Positive = below target.
- `time`: Time since last update in seconds (e.g., 0.01 for 10ms)

**Returns:** Control output value (not clamped - you may need to limit it)

**Example:**
```cpp
while (!atTarget) {
    float error = targetAngle - currentAngle;  // e.g., 90° - 45° = 45°
    float output = turnPID.update(error, 0.01);  // 0.01 = 10ms
    
    // Clamp output to motor range
    output = std::clamp(output, -127.0f, 127.0f);
    
    drivetrain.turn(output);
    pros::delay(10);
}
```

---

### reset()

```cpp
void reset()
```

Clears the integral accumulator and previous error. **Call this before starting a new movement.**

**Example:**
```cpp
void moveToAngle(float targetAngle) {
    turnPID.reset();  // IMPORTANT: Clear old state
    
    while (!atTarget) {
        // ... control loop
    }
}
```

---

### setKP(), setKI(), setKD()

```cpp
void setKP(float newKP)
void setKI(float newKI)
void setKD(float newKD)
```

Modify gains at runtime.

**Example:**
```cpp
// Increase P gain if we're moving slowly
if (speed < threshold) {
    pid.setKP(originalKP * 1.5);
}
```

---

## Using PID in Practice

### Basic Pattern

```cpp
void moveToTarget(float target) {
    shulib::PID pid(5.0, 0.1, 0.2);  // Create controller
    pid.reset();                      // Clear any old state
    
    while (true) {
        float current = getSensorValue();
        float error = target - current;
        
        // Check if we're done
        if (fabs(error) < TOLERANCE) {
            break;
        }
        
        // Get PID output
        float output = pid.update(error, 0.01);
        
        // Clamp to valid range
        output = std::clamp(output, -MAX_OUTPUT, MAX_OUTPUT);
        
        // Apply to motors
        motor.move(output);
        
        // Wait for next cycle
        pros::delay(10);
    }
    
    motor.move(0);  // Stop
}
```

### Adding Minimum Output

Sometimes friction prevents movement at low power:

```cpp
float output = pid.update(error, 0.01);

// Apply minimum output to overcome static friction
if (fabs(output) < MIN_OUTPUT && fabs(error) > TOLERANCE) {
    output = (error > 0) ? MIN_OUTPUT : -MIN_OUTPUT;
}
```

### Adding Timeout

Prevent infinite loops if something goes wrong:

```cpp
int elapsed = 0;
const int TIMEOUT = 3000;  // 3 seconds

while (elapsed < TIMEOUT) {
    // ... PID loop ...
    
    if (fabs(error) < TOLERANCE) {
        break;  // Success!
    }
    
    pros::delay(10);
    elapsed += 10;
}

// If we get here via timeout, something went wrong
if (elapsed >= TIMEOUT) {
    logger().warning("PID timeout! Did not reach target.");
}
```

### Adding Settle Time

Ensure we're actually stable, not just passing through:

```cpp
int settleCount = 0;
const int SETTLE_CYCLES = 5;  // 50ms at 10ms/cycle

while (true) {
    float error = target - current;
    
    if (fabs(error) < TOLERANCE) {
        settleCount++;
        if (settleCount >= SETTLE_CYCLES) {
            break;  // Settled!
        }
    } else {
        settleCount = 0;  // Reset if we leave tolerance
    }
    
    // ... rest of PID loop ...
}
```

### Multiple PIDs Working Together

Sometimes you need separate controllers for different aspects:

```cpp
void driveStraight(float distance) {
    shulib::PID drivePID(5.0, 0.05, 0.2);    // For forward motion
    shulib::PID headingPID(1.0, 0, 0.05);    // For keeping straight
    
    float startHeading = getHeading();
    float startPos = getPosition();
    
    while (true) {
        float distError = distance - (getPosition() - startPos);
        float headingError = startHeading - getHeading();
        
        float driveOutput = drivePID.update(distError, 0.01);
        float turnOutput = headingPID.update(headingError, 0.01);
        
        // Combine: drive forward + correct heading
        leftMotor.move(driveOutput + turnOutput);
        rightMotor.move(driveOutput - turnOutput);
        
        pros::delay(10);
    }
}
```

---

## Common PID Configurations

### Our Tuned Values

From our codebase, here are configurations that work for our robots:

#### Rotation (Turning in Place)

```cpp
PID turnPID(1.5, 0.01, 0.15);
const double MIN_POWER = 20.0;
const double MAX_POWER = 50.0;
const double TOLERANCE = 2.0;  // degrees
```

**Why these values:**
- Moderate P (1.5): Turns are sensitive, don't want to overshoot
- Very small I (0.01): Just enough to eliminate small steady-state error
- Small D (0.15): Dampens oscillation at target

#### Linear Movement (Driving Forward/Back)

```cpp
PID drivePID(5.0, 0.05, 0.2);
const double MIN_POWER = 20.0;
const double MAX_POWER = 60.0;
const double TOLERANCE = 1.0;  // inches
```

**Why these values:**
- Higher P (5.0): Position error in inches needs larger response
- Small I (0.05): Helps overcome friction
- Moderate D (0.2): Prevents overshoot when stopping

#### Heading Correction (While Driving)

```cpp
PID headingPID(1.0, 0, 0);
const double MAX_CORRECTION = 10.0;  // limit turn adjustment
```

**Why these values:**
- P only: Just need proportional correction
- No I: Don't want accumulated heading error causing drift
- No D: Heading correction should be responsive, not damped

### Starting Points for New Applications

| Application | Starting kP | Starting kI | Starting kD |
|-------------|-------------|-------------|-------------|
| Position (inches) | 5.0 | 0.05 | 0.2 |
| Angle (degrees) | 1.5 | 0.01 | 0.15 |
| Velocity (RPM) | 0.5 | 0.1 | 0.01 |
| Arm position | 3.0 | 0.1 | 0.3 |

**These are starting points only!** Every robot is different. You must tune for your specific mechanism.

---

## The Art of Gain Selection

### The Tuning Philosophy

Tuning PID is part science, part art. Here's the general approach:

```
┌─────────────────────────────────────────────────────────────────┐
│                    THE TUNING PROCESS                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  1. START WITH P ONLY                                          │
│     Set I = 0, D = 0                                           │
│     Increase P until system responds quickly                    │
│     Accept some overshoot for now                              │
│                                                                 │
│  2. ADD D TO REDUCE OVERSHOOT                                  │
│     Increase D until overshoot is acceptable                   │
│     Don't add too much - causes jitter                         │
│                                                                 │
│  3. ADD I IF NEEDED                                            │
│     Only if there's steady-state error                         │
│     Keep it small - too much causes overshoot                  │
│                                                                 │
│  4. FINE-TUNE                                                   │
│     Small adjustments to balance speed vs stability            │
│     Test across different conditions                           │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Reading the Behavior

| What You See | Probable Cause | What to Adjust |
|--------------|----------------|----------------|
| Sluggish response | P too low | ↑ Increase P |
| Oscillating wildly | P too high | ↓ Decrease P |
| Overshoot then settle | D too low | ↑ Increase D |
| Jittery/noisy | D too high | ↓ Decrease D |
| Never quite reaches target | Need I | Add small I |
| Overshoot gets worse over time | I too high | ↓ Decrease I |
| Robot doesn't move at all | P way too low OR output not reaching motors | Check wiring, ↑ increase P significantly |

### Gain Interaction

The gains affect each other:

```
If you increase P:
├── Response gets faster
├── Overshoot increases
└── May need more D to compensate

If you increase D:
├── Overshoot decreases
├── Response may slow slightly
└── Can reduce P slightly if too aggressive

If you increase I:
├── Steady-state error decreases
├── Overshoot increases
├── Settling time increases
└── May need more D to compensate
```

> **Detailed step-by-step tuning procedures are in [PID_TUNING.md](../motion/PID_TUNING.md)**

---

## When PID Isn't Enough

### Limitations of Basic PID

PID is powerful but not perfect. It struggles with:

1. **Large disturbances**: Getting rammed by another robot
2. **Highly non-linear systems**: Behavior changes dramatically at different operating points
3. **Systems with significant delay**: Long lag between command and response
4. **Constraints**: Motor limits, physical stops

### Enhancements We Use

#### Output Clamping

```cpp
float output = pid.update(error, 0.01);
output = std::clamp(output, -MAX_OUTPUT, MAX_OUTPUT);
```

Motors can only do -127 to +127. Clamping prevents the math from asking for impossible values.

#### Minimum Output (Deadband)

```cpp
if (fabs(output) < MIN_OUTPUT && fabs(error) > TOLERANCE) {
    output = (error > 0) ? MIN_OUTPUT : -MIN_OUTPUT;
}
```

Overcomes static friction that prevents movement at low power.

#### Integral Anti-Windup

When the robot can't move (hitting a wall, motor maxed out), the integral keeps growing ("winding up"). When it finally can move, there's a huge accumulated value that causes massive overshoot.

```cpp
// Simple anti-windup: limit integral
const float MAX_INTEGRAL = 1000;
integral = std::clamp(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

// Or: only accumulate when output isn't saturated
if (fabs(output) < MAX_OUTPUT) {
    integral += error * time;
}
```

#### Slew Rate Limiting

Prevent sudden large changes in output (smoother acceleration):

```cpp
float output = pid.update(error, 0.01);

// Limit how fast output can change
const float MAX_CHANGE = 5.0;  // per cycle
float change = output - lastOutput;
change = std::clamp(change, -MAX_CHANGE, MAX_CHANGE);
output = lastOutput + change;

lastOutput = output;
```

### Alternative Approaches

For some applications, PID alternatives work better:

| Approach | When to Use |
|----------|-------------|
| **Bang-bang** | Simple on/off control (flywheel at speed) |
| **Feedforward** | When you know the system model well |
| **Motion profiling** | When you want specific velocity/acceleration curves |
| **State machines** | When behavior should be discrete (open/closed) |

---

## Debugging PID Issues

### Problem: Robot Doesn't Move

```
Checklist:
□ Is error being calculated correctly? (print it)
□ Is PID output non-zero? (print it)
□ Is output reaching motors? (check wiring, print motor values)
□ Is kP high enough? (try kP = 10 or higher temporarily)
□ Is there a sign error? (output positive when it should be negative)
□ Are motors configured correctly? (port numbers, reversal)
```

### Problem: Robot Oscillates

```
Symptoms: Robot bounces back and forth around target

Solutions:
1. Decrease kP (too much response to error)
2. Increase kD (add damping)
3. Add deadband near target (stop correcting for tiny errors)
4. Check for mechanical issues (loose parts, backlash)
```

### Problem: Robot Overshoots Then Settles

```
Symptoms: Goes past target, then comes back

Solutions:
1. Increase kD (more braking as you approach)
2. Decrease kP (less aggressive approach)
3. If I is non-zero, decrease it
4. Add slew rate limiting
```

### Problem: Robot Never Quite Reaches Target

```
Symptoms: Stops a bit short/long consistently

Solutions:
1. Add small kI (to eliminate steady-state error)
2. Add minimum output (to overcome friction)
3. Decrease tolerance (if it's too generous)
4. Check for mechanical binding
```

### Debugging Print Statements

```cpp
void debugPID(float target, float current, float output) {
    static int counter = 0;
    if (counter++ % 10 == 0) {  // Print every 100ms
        printf("Target: %.2f  Current: %.2f  Error: %.2f  Output: %.2f\n",
               target, current, target - current, output);
    }
}
```

---

## Key Takeaways

### The Essentials

1. **PID = Proportional + Integral + Derivative**
   - P responds to current error
   - I responds to accumulated error
   - D responds to rate of change of error

2. **Start with P, add D, then I if needed**
   - P is the main driver
   - D adds stability
   - I eliminates steady-state error (but can cause overshoot)

3. **Always reset() before new movements**
   - Old integral values will corrupt new movements
   - This is a common bug!

4. **Clamp your outputs**
   - Motors have limits
   - Don't ask for what you can't have

5. **Tuning is iterative**
   - Start with known-good values
   - Adjust based on observed behavior
   - Test across different conditions

### The One-Sentence Summary

> **PID calculates motor output by combining how far you are from the target (P), how long you've been away (I), and how fast you're approaching (D), enabling smooth and accurate robot movements.**

---

## Where to Go Next

| Topic | Document | What You'll Learn |
|-------|----------|-------------------|
| Practical tuning steps | [PID_TUNING.md](../motion/PID_TUNING.md) | Step-by-step tuning process |
| Motion functions | [MOTION_OVERVIEW.md](../motion/MOTION_OVERVIEW.md) | How PID is used in autonomous |
| Specific movements | [MOVE_VERTICAL.md](../motion/MOVE_VERTICAL.md), [ROTATE_TO.md](../motion/ROTATE_TO.md) | PID in action |
| Oscillation problems | [PID_OSCILLATION.md](../troubleshooting/PID_OSCILLATION.md) | Debugging oscillation |
| The Chassis class | [CHASSIS.md](./CHASSIS.md) | How PID integrates with Chassis |

### External Resources

- 📄 **[Wikipedia: PID Controller](https://en.wikipedia.org/wiki/PID_controller)** - Comprehensive theory
- 🎥 **[Brian Douglas - PID Control](https://www.youtube.com/watch?v=UR0hOmjaHp0)** - Excellent video series
- 📚 **[PID Without a PhD](https://www.wescottdesign.com/articles/pid/pidWithoutAPhd.pdf)** - Practical guide

---

*Document last updated: January 2026*