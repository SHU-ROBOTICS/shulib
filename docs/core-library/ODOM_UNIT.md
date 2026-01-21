# OdomUnit

**Location:** `include/shulib/core/odomUnit.hpp` and `src/core/odomUnit.cpp`

---

## Table of Contents

1. [What is an OdomUnit?](#what-is-an-odomunit)
2. [The Role in Odometry](#the-role-in-odometry)
3. [How It Works](#how-it-works)
4. [The Conversion Math](#the-conversion-math)
5. [Delta Tracking](#delta-tracking)
6. [The OdomUnit Class](#the-odomunit-class)
7. [API Reference](#api-reference)
8. [Creating OdomUnits](#creating-odomunits)
9. [Common Patterns](#common-patterns)
10. [Debugging OdomUnits](#debugging-odomunits)
11. [Key Takeaways](#key-takeaways)
12. [Where to Go Next](#where-to-go-next)

---

## What is an OdomUnit?

### The Simple Explanation

An **OdomUnit** is a wrapper around a single tracking wheel. It takes the raw sensor readings (encoder ticks) and converts them into meaningful distances (inches).

Think of it as a translator:

```
┌─────────────────┐      ┌─────────────────┐      ┌─────────────────┐
│  Rotation       │      │    OdomUnit     │      │    Odometry     │
│  Sensor         │ ───► │    (translator) │ ───► │    System       │
│                 │      │                 │      │                 │
│  "36000 ticks"  │      │  "10.0 inches"  │      │  "Robot moved!" │
└─────────────────┘      └─────────────────┘      └─────────────────┘
```

Without OdomUnit, the odometry system would have to deal with raw sensor values and know the wheel diameter for every calculation. OdomUnit encapsulates this complexity.

### What It Encapsulates

Each OdomUnit knows three things about its tracking wheel:

| Property | What It Is | Example |
|----------|------------|---------|
| **Sensor** | Which rotation sensor to read | Port 8 |
| **Diameter** | The wheel's diameter in inches | 2.75" |
| **Offset** | Distance from robot's center | -6.5" (left of center) |

With this information, it can convert any sensor reading to a distance traveled.

### Why Not Just Use the Sensor Directly?

You could read the rotation sensor directly:

```cpp
// Without OdomUnit (messy)
pros::Rotation sensor(8);
float ticks = sensor.get_position();  // Centidegrees
float rotations = ticks / 36000.0;    // Convert to rotations
float distance = rotations * 2.75 * M_PI;  // Convert to inches

// You'd repeat this calculation everywhere!
```

With OdomUnit, it's clean:

```cpp
// With OdomUnit (clean)
OdomUnit wheel(&sensor, 2.75, -6.5);
float distance = wheel.get_travel();  // Just works!
```

The conversion logic is written once and reused everywhere.

---

## The Role in Odometry

### The Three OdomUnits

The odometry system uses three OdomUnits:

```
                    FRONT OF ROBOT
                         ↑
              ┌─────────────────────┐
              │                     │
         ┌────┤                     ├────┐
         │ L  │                     │  R │
         │    │                     │    │
         │    │                     │    │
         └────┤                     ├────┘
              │       ════         │
              │        B           │
              │                     │
              └─────────────────────┘
              
    L = Left OdomUnit   (measures Y movement on left side)
    R = Right OdomUnit  (measures Y movement on right side)
    B = Back OdomUnit   (measures X movement / sideways drift)
```

### How They Feed Into Odometry

```
┌─────────────────────────────────────────────────────────────────────┐
│                    DATA FLOW                                        │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Left OdomUnit ──────► get_travel_delta() ──────┐                 │
│                                                   │                 │
│   Right OdomUnit ─────► get_travel_delta() ──────┼──► Odometry    │
│                                                   │     update()    │
│   Back OdomUnit ──────► get_travel_delta() ──────┘                 │
│                                                                     │
│                                                   │                 │
│                                                   ▼                 │
│                                              New Pose               │
│                                             (x, y, θ)               │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

Every 10ms, the odometry system:
1. Calls `get_travel_delta()` on each OdomUnit
2. Gets back how far each wheel moved since the last call
3. Uses those distances to calculate the new robot position

### The Offset Property

Each OdomUnit stores an **offset** - its distance from the robot's center of rotation:

```
                     Center of
                     Rotation
                        ↓
         ┌──────────────●──────────────┐
         │              │              │
    ┌────┤              │              ├────┐
    │ L  │◄─── -6.5" ───┼─── +6.5" ───►│  R │
    │    │    (offset)  │   (offset)   │    │
    └────┤              │              ├────┘
         │              │              │
         │              │              │
         │         ┌────┴────┐         │
         │         │    B    │         │
         │         └────┬────┘         │
         │              │              │
         │        offset = 0           │
         │    (centered in this case)  │
         └─────────────────────────────┘
```

**Offset conventions:**
- **Left wheel**: Negative (e.g., -6.5")
- **Right wheel**: Positive (e.g., +6.5")
- **Back wheel**: Positive if behind center, negative if in front

The offset is critical for calculating rotation - the odometry math uses it to determine how much the robot turned based on the difference in wheel travel.

---

## How It Works

### The Rotation Sensor

VEX V5 Rotation Sensors measure angular position with very high precision:

| Specification | Value |
|---------------|-------|
| Resolution | 0.01° (centidegrees) |
| Ticks per revolution | 36,000 |
| Range | Continuous (no wrap-around in position mode) |
| Update rate | 10ms |

The sensor returns position in **centidegrees** - hundredths of a degree. So 36,000 centidegrees = 360° = 1 full rotation.

### From Ticks to Inches

The conversion chain:

```
Centidegrees → Degrees → Rotations → Distance

Example: Sensor reads 18,000 centidegrees

Step 1: Centidegrees to degrees
        18,000 / 100 = 180°

Step 2: Degrees to rotations
        180 / 360 = 0.5 rotations

Step 3: Rotations to distance
        0.5 × π × diameter = 0.5 × π × 2.75" = 4.32"

Or combined:
        distance = (centidegrees / 36,000) × π × diameter
        distance = (18,000 / 36,000) × π × 2.75
        distance = 0.5 × 3.14159 × 2.75
        distance = 4.32 inches
```

### The Code

```cpp
double OdomUnit::get_travel() {
    if (this->sensor != nullptr) {
        // sensor->get_position() returns centidegrees
        // Divide by 36000 to get rotations
        // Multiply by π × diameter to get distance
        return (float(this->sensor->get_position()) * this->diameter * M_PI / 36000);
    } else {
        return 0;
    }
}
```

Let's break this down:

```
get_position()     Returns centidegrees (e.g., 18000)
× diameter         Multiply by wheel diameter (e.g., 2.75)
× M_PI             Multiply by π (3.14159...)
÷ 36000            Divide by centidegrees per rotation

= (18000 × 2.75 × 3.14159) / 36000
= 155,509 / 36000
= 4.32 inches
```

---

## The Conversion Math

### Why This Formula Works

The distance traveled by a wheel is:

```
distance = rotations × circumference
distance = rotations × (π × diameter)
```

And rotations from sensor reading:

```
rotations = centidegrees / 36000
```

Combining:

```
distance = (centidegrees / 36000) × π × diameter
```

Rearranging for code efficiency:

```
distance = centidegrees × diameter × π / 36000
```

### Dimensional Analysis

Let's verify the units work out:

```
centidegrees × inches × (unitless) / (centidegrees/rotation)

= centidegrees × inches × rotation / centidegrees

= inches × rotation

Wait, that gives us inches × rotations, not just inches!
```

Actually, let's be more careful:

```
centidegrees × (inches) × (unitless) / (centidegrees per rotation)

The π is actually: inches of circumference per inch of diameter
So π has units of: (circumference inches) / (diameter inches) = unitless

And 36000 is: centidegrees per rotation

So:
centidegrees × inches × (unitless) / (centidegrees/rotation)
= inches × rotation × (rotation⁻¹)
= inches ✓
```

The units work out to inches, which is what we want!

### Precision Considerations

The rotation sensor has 0.01° precision. What does that mean in distance?

```
Minimum detectable rotation: 0.01° = 1 centidegree

With a 2.75" wheel:
minimum_distance = (1 / 36000) × π × 2.75
                 = 0.00024 inches
                 ≈ 0.006 mm

That's incredibly precise! Much finer than we need.
```

In practice, mechanical slop and wheel imperfections limit real-world accuracy far more than sensor precision.

---

## Delta Tracking

### The Problem

The odometry system doesn't want to know the *total* distance traveled since power-on. It wants to know how far the wheel moved *since the last check*.

```
Time 0ms:    Total = 0.00"    
Time 10ms:   Total = 0.50"    Δ = 0.50"  ← We want this!
Time 20ms:   Total = 1.05"    Δ = 0.55"  ← And this!
Time 30ms:   Total = 1.58"    Δ = 0.53"  ← And this!
```

### The Solution: `get_travel_delta()`

OdomUnit tracks the last position and returns only the change:

```cpp
double OdomUnit::get_travel_delta() {
    double current = this->get_travel();      // Get current total
    double delta = current - this->lastPosition;  // Calculate change
    this->lastPosition = current;             // Save for next time
    return delta;                             // Return just the change
}
```

**Visual:**

```
         lastPosition          current
              │                    │
              ▼                    ▼
    ──────────●════════════════════●────────────────►
              │◄─────── delta ────►│
              │                    │
         (stored)              (measured)
```

### Why Delta Matters

The odometry update equation needs delta values:

```cpp
void shulib::update() {
    // Get how far each wheel moved SINCE LAST UPDATE
    float dL = odomSensors.left->get_travel_delta();   // e.g., 0.05"
    float dR = odomSensors.right->get_travel_delta();  // e.g., 0.06"
    float dS = odomSensors.back->get_travel_delta();   // e.g., -0.01"
    
    // Calculate rotation from difference
    float deltaTheta = (dR - dL) / (sL - sR);
    
    // ... rest of odometry math
}
```

If we used total travel instead of delta, the first update would calculate the robot's position since power-on, which would be wrong.

### The Importance of Calling Order

**⚠️ Warning:** `get_travel_delta()` modifies internal state!

```cpp
// First call: returns 0.50"
float delta1 = odomUnit.get_travel_delta();

// Second call immediately after: returns ~0" (no time has passed!)
float delta2 = odomUnit.get_travel_delta();  // This is nearly zero!
```

Each call "consumes" the delta. The odometry system calls it exactly once per update cycle, which is correct. Don't call it extra times!

---

## The OdomUnit Class

### Class Definition

```cpp
namespace shulib {

class OdomUnit {
public:
    /**
     * Create a new OdomUnit
     * @param sensor  Pointer to the rotation sensor
     * @param diameter  Wheel diameter in inches
     * @param offset  Distance from tracking center in inches
     */
    OdomUnit(pros::Rotation* sensor, float diameter, float offset);

    /**
     * Reset the tracking wheel's position to 0
     */
    void reset();

    /**
     * Get total distance traveled since last reset
     * @return Distance in inches
     */
    double get_travel();

    /**
     * Get distance traveled since last call to this function
     * @return Change in distance in inches
     */
    double get_travel_delta();

    /**
     * Get the offset from tracking center
     * @return Offset in inches
     */
    double get_offset();

private:
    pros::Rotation* sensor = nullptr;  // The rotation sensor
    float diameter;                     // Wheel diameter in inches
    float offset;                       // Distance from center in inches
    float lastPosition;                 // For delta calculation
};

} // namespace shulib
```

### Member Variables

| Variable | Type | Description |
|----------|------|-------------|
| `sensor` | `pros::Rotation*` | Pointer to the V5 Rotation Sensor |
| `diameter` | `float` | Wheel diameter in inches |
| `offset` | `float` | Distance from tracking center (negative = left) |
| `lastPosition` | `float` | Last known travel distance (for delta) |

### Implementation Details

**Constructor:**
```cpp
OdomUnit::OdomUnit(pros::Rotation* sensor, float diameter, float offset) {
    this->sensor = sensor;
    this->diameter = diameter;
    this->offset = offset;
    this->lastPosition = 0;
}
```

**Reset:**
```cpp
void OdomUnit::reset() {
    if (this->sensor != nullptr) {
        this->sensor->reset_position();  // Zero the sensor
    }
    this->lastPosition = 0;  // Zero our tracking too
}
```

**Get Travel:**
```cpp
double OdomUnit::get_travel() {
    if (this->sensor != nullptr) {
        return (float(this->sensor->get_position()) * this->diameter * M_PI / 36000);
    } else {
        return 0;
    }
}
```

**Get Travel Delta:**
```cpp
double OdomUnit::get_travel_delta() {
    double current = this->get_travel();
    double delta = current - this->lastPosition;
    this->lastPosition = current;
    return delta;
}
```

**Get Offset:**
```cpp
double OdomUnit::get_offset() {
    return this->offset;
}
```

---

## API Reference

### Constructor

```cpp
OdomUnit(pros::Rotation* sensor, float diameter, float offset)
```

Creates a new OdomUnit wrapping a rotation sensor.

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `sensor` | `pros::Rotation*` | Pointer to the rotation sensor object |
| `diameter` | `float` | Tracking wheel diameter in inches |
| `offset` | `float` | Distance from robot's center of rotation (inches) |

**Example:**
```cpp
pros::Rotation leftSensor(-8);  // Port 8, reversed
shulib::OdomUnit leftWheel(&leftSensor, 2.75, -6.5);
```

---

### reset()

```cpp
void reset()
```

Resets the sensor position to zero and clears internal tracking state.

**When to call:**
- During robot initialization
- After manually moving the robot
- When recalibrating

**Example:**
```cpp
leftWheel.reset();
rightWheel.reset();
backWheel.reset();
// All wheels now read 0 travel
```

---

### get_travel()

```cpp
double get_travel()
```

Returns the total distance traveled since the last reset.

**Returns:** Distance in inches (positive = forward/right rotation)

**Example:**
```cpp
// After robot has moved
double totalDistance = leftWheel.get_travel();
printf("Left wheel traveled: %.2f inches\n", totalDistance);
```

**Note:** This does NOT consume the delta. Safe to call multiple times.

---

### get_travel_delta()

```cpp
double get_travel_delta()
```

Returns the distance traveled since the **last time this function was called**.

**Returns:** Change in distance in inches

**Example:**
```cpp
// In odometry update loop (every 10ms)
double dL = leftWheel.get_travel_delta();   // e.g., 0.05"
double dR = rightWheel.get_travel_delta();  // e.g., 0.06"
double dS = backWheel.get_travel_delta();   // e.g., -0.01"
```

**⚠️ Warning:** This function modifies internal state! Each call updates `lastPosition`. Only call once per update cycle.

---

### get_offset()

```cpp
double get_offset()
```

Returns the wheel's offset from the robot's center of rotation.

**Returns:** Offset in inches (negative = left of center, positive = right of center)

**Example:**
```cpp
double leftOffset = leftWheel.get_offset();   // e.g., -6.5
double rightOffset = rightWheel.get_offset(); // e.g., +6.5
```

---

## Creating OdomUnits

### Basic Creation Pattern

```cpp
// 1. Create the rotation sensor
pros::Rotation sensor(8);  // Port 8

// 2. Create the OdomUnit wrapping it
shulib::OdomUnit wheel(&sensor, 2.75, -6.5);
//                      │       │      │
//                      │       │      └── Offset: 6.5" left of center
//                      │       └── Diameter: 2.75 inches
//                      └── Pointer to sensor
```

### Handling Reversed Sensors

If pushing the robot forward makes the sensor read negative:

```cpp
// Method 1: Negative port number (preferred)
pros::Rotation sensor(-8);  // Port 8, reversed
shulib::OdomUnit wheel(&sensor, 2.75, -6.5);

// Method 2: Reverse in sensor API
pros::Rotation sensor(8);
sensor.set_reversed(true);
shulib::OdomUnit wheel(&sensor, 2.75, -6.5);
```

### From Robot Configuration

In our codebase, OdomUnits are created from the robot config:

```cpp
// In main.cpp:

// Create rotation sensors from config
pros::Rotation leftRotation(ROBOT.tracking.left_port);
pros::Rotation rightRotation(ROBOT.tracking.right_port);
pros::Rotation backRotation(ROBOT.tracking.back_port);

// Create OdomUnits
shulib::OdomUnit leftOdom(&leftRotation, 
                          ROBOT.tracking.wheel_diameter, 
                          ROBOT.tracking.left_offset);

shulib::OdomUnit rightOdom(&rightRotation, 
                           ROBOT.tracking.wheel_diameter, 
                           ROBOT.tracking.right_offset);

shulib::OdomUnit backOdom(&backRotation, 
                          ROBOT.tracking.wheel_diameter, 
                          ROBOT.tracking.back_offset);
```

### The OdomSensors Bundle

After creating individual OdomUnits, they're bundled into an OdomSensors object:

```cpp
// Bundle the three OdomUnits (plus optional IMU)
shulib::OdomSensors sensors(&leftOdom, &rightOdom, &backOdom, nullptr);
//                           │          │          │          │
//                           │          │          │          └── IMU (nullptr = none)
//                           │          │          └── Back wheel
//                           │          └── Right wheel
//                           └── Left wheel
```

This OdomSensors object is then passed to the Chassis.

---

## Common Patterns

### Pattern 1: Initialization Sequence

```cpp
void initialize() {
    // 1. Create sensors
    pros::Rotation leftSensor(ROBOT.tracking.left_port);
    pros::Rotation rightSensor(ROBOT.tracking.right_port);
    pros::Rotation backSensor(ROBOT.tracking.back_port);
    
    // 2. Create OdomUnits
    shulib::OdomUnit left(&leftSensor, 
                          ROBOT.tracking.wheel_diameter,
                          ROBOT.tracking.left_offset);
    shulib::OdomUnit right(&rightSensor,
                           ROBOT.tracking.wheel_diameter,
                           ROBOT.tracking.right_offset);
    shulib::OdomUnit back(&backSensor,
                          ROBOT.tracking.wheel_diameter,
                          ROBOT.tracking.back_offset);
    
    // 3. Bundle into OdomSensors
    shulib::OdomSensors sensors(&left, &right, &back, nullptr);
    
    // 4. Create Chassis
    shulib::Chassis chassis(drivetrain, sensors);
    
    // 5. Calibrate (this resets the OdomUnits internally)
    chassis.calibrate(false);
}
```

### Pattern 2: Debug Raw Values

```cpp
// Useful for testing/calibration
void debugOdomUnits() {
    while (true) {
        printf("L: %7.2f  R: %7.2f  B: %7.2f\n",
               leftOdom.get_travel(),
               rightOdom.get_travel(),
               backOdom.get_travel());
        pros::delay(100);
    }
}
```

### Pattern 3: Verify Sensor Directions

```cpp
void testSensorDirections() {
    printf("Push robot FORWARD. All values should INCREASE.\n");
    
    for (int i = 0; i < 50; i++) {
        printf("L: %+.2f  R: %+.2f  B: %+.2f\n",
               leftOdom.get_travel(),
               rightOdom.get_travel(),
               backOdom.get_travel());
        pros::delay(100);
    }
    
    printf("\nPush robot RIGHT. Back value should INCREASE.\n");
    // ... similar loop
}
```

### Pattern 4: Measure Wheel Diameter

```cpp
void measureWheelDiameter() {
    // Push robot exactly 24 inches (use tape measure)
    // Compare expected vs actual
    
    leftOdom.reset();
    printf("Push robot exactly 24 inches, then press A.\n");
    
    while (!controller.get_digital_new_press(DIGITAL_A)) {
        pros::delay(10);
    }
    
    double measured = leftOdom.get_travel();
    printf("Measured: %.2f inches\n", measured);
    printf("Expected: 24.00 inches\n");
    
    if (fabs(measured - 24.0) > 0.5) {
        double correctionFactor = 24.0 / measured;
        printf("Wheel diameter might be off. Try: %.3f\n",
               ROBOT.tracking.wheel_diameter * correctionFactor);
    }
}
```

---

## Debugging OdomUnits

### Problem: Wheel Reads Zero

**Symptoms:** `get_travel()` always returns 0

**Checklist:**
```
□ Is the sensor plugged into the correct port?
□ Is the port number correct in config?
□ Is the sensor detected? (Check devices in brain menu)
□ Is the sensor pointer valid (not nullptr)?
□ Is the wheel actually spinning when you push the robot?
```

**Test:**
```cpp
// Check if sensor is working at all
if (sensor.get_position() == PROS_ERR) {
    printf("Sensor error! Check connection.\n");
} else {
    printf("Sensor position: %d\n", sensor.get_position());
}
```

### Problem: Wheel Reads Backwards

**Symptoms:** Value decreases when pushing robot forward

**Solution:** Reverse the sensor port number:
```cpp
// Change this:
pros::Rotation sensor(8);

// To this:
pros::Rotation sensor(-8);  // Note the negative sign
```

### Problem: Values Seem Wrong Scale

**Symptoms:** Travel values much too large or small

**Possible causes:**
1. Wrong wheel diameter in config
2. Diameter in wrong units (mm instead of inches?)
3. Math error somewhere

**Debug:**
```cpp
// Push robot exactly 1 wheel circumference
// Should read: π × diameter = ~8.64" for 2.75" wheel

leftOdom.reset();
printf("Rotate left wheel exactly 1 full turn.\n");
// ... wait for user ...
printf("Travel: %.2f (should be ~8.64 for 2.75\" wheel)\n", 
       leftOdom.get_travel());
```

### Problem: Delta Returns Zero After First Call

**Symptoms:** First `get_travel_delta()` works, subsequent calls return ~0

**This is expected behavior!** Delta returns the change *since the last call*. If you call it twice in a row with no wheel movement, the second call returns zero.

**Make sure:** Only call `get_travel_delta()` once per update cycle.

### Problem: Values Jump Erratically

**Symptoms:** Travel values suddenly jump by large amounts

**Possible causes:**
1. Loose sensor connection
2. Electrical interference
3. Wheel slipping badly
4. Code calling reset unexpectedly

**Debug:**
```cpp
// Log values to find the jump
double lastValue = 0;
while (true) {
    double current = leftOdom.get_travel();
    double delta = current - lastValue;
    
    if (fabs(delta) > 1.0) {  // Jumped more than 1 inch
        printf("JUMP! %.2f → %.2f (delta: %.2f)\n", 
               lastValue, current, delta);
    }
    
    lastValue = current;
    pros::delay(10);
}
```

### Diagnostic Print Function

```cpp
void printOdomDiagnostics() {
    printf("\n=== ODOM UNIT DIAGNOSTICS ===\n");
    
    // Left wheel
    printf("Left:  port=%d, dia=%.2f, off=%.2f, travel=%.2f\n",
           ROBOT.tracking.left_port,
           ROBOT.tracking.wheel_diameter,
           ROBOT.tracking.left_offset,
           leftOdom.get_travel());
    
    // Right wheel
    printf("Right: port=%d, dia=%.2f, off=%.2f, travel=%.2f\n",
           ROBOT.tracking.right_port,
           ROBOT.tracking.wheel_diameter,
           ROBOT.tracking.right_offset,
           rightOdom.get_travel());
    
    // Back wheel
    printf("Back:  port=%d, dia=%.2f, off=%.2f, travel=%.2f\n",
           ROBOT.tracking.back_port,
           ROBOT.tracking.wheel_diameter,
           ROBOT.tracking.back_offset,
           backOdom.get_travel());
    
    printf("==============================\n\n");
}
```

---

## Key Takeaways

### The Essentials

1. **OdomUnit wraps a tracking wheel**
   - Sensor + diameter + offset = complete wheel description
   - Converts raw sensor ticks to inches automatically

2. **The conversion formula:**
   ```
   distance = (centidegrees × diameter × π) / 36000
   ```

3. **Delta tracking is stateful**
   - `get_travel_delta()` returns change since last call
   - Only call once per update cycle!
   - It updates internal state

4. **Offsets matter for rotation**
   - Left wheel: negative offset (e.g., -6.5")
   - Right wheel: positive offset (e.g., +6.5")
   - Used by odometry to calculate robot rotation

5. **Sensor direction must be correct**
   - Forward movement = positive values
   - Use negative port number to reverse if needed

### The One-Sentence Summary

> **OdomUnit converts raw rotation sensor readings into inches of wheel travel, handling the math and delta tracking so the odometry system gets clean distance values.**

---

## Where to Go Next

| Topic | Document | What You'll Learn |
|-------|----------|-------------------|
| The full odometry system | [ODOMETRY.md](./ODOMETRY.md) | How OdomUnits feed into position tracking |
| Physical wheel setup | [TRACKING_WHEELS.md](../configuration/TRACKING_WHEELS.md) | Mounting, wiring, calibration |
| Sensor troubleshooting | [ODOMETRY_DRIFT.md](../troubleshooting/ODOMETRY_DRIFT.md) | When tracking goes wrong |
| Robot configuration | [ROBOT_CONFIG.md](../configuration/ROBOT_CONFIG.md) | Where wheel specs are defined |
| The OdomSensors bundle | [CHASSIS.md](./CHASSIS.md) | How OdomUnits combine into OdomSensors |

### Related Code Files

```
include/shulib/core/odomUnit.hpp  ← Class definition
src/core/odomUnit.cpp             ← Implementation
include/shulib/core/chassis.hpp   ← OdomSensors class
src/core/odometry.cpp             ← Where OdomUnits are used
```

---

*Document last updated: January 2026*
*May your tracking wheels spin true! 🎡*