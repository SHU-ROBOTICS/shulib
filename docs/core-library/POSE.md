# Pose

**Location:** `include/shulib/core/pose.hpp` and `src/core/pose.cpp`

---

## Table of Contents

1. [What is a Pose?](#what-is-a-pose)
2. [Why Pose Matters](#why-pose-matters)
3. [The Pose Class](#the-pose-class)
4. [Creating Poses](#creating-poses)
5. [Pose Arithmetic](#pose-arithmetic)
6. [Pose Methods](#pose-methods)
7. [Common Use Cases](#common-use-cases)
8. [The Math Behind the Methods](#the-math-behind-the-methods)
9. [Poses Throughout shulib](#poses-throughout-shulib)
10. [Tips and Gotchas](#tips-and-gotchas)
11. [API Reference](#api-reference)
12. [Key Takeaways](#key-takeaways)
13. [Where to Go Next](#where-to-go-next)

---

## What is a Pose?

### The Simple Explanation

A **Pose** is a complete description of where something is and which way it's facing.

Think about giving someone directions to meet you:
- "I'm at the coffee shop" ← **Position** (where)
- "I'm facing the entrance" ← **Orientation** (which way)

Together, position + orientation = **Pose**.

### The Three Components

```
┌─────────────────────────────────────────────────────────────────┐
│                         A POSE                                  │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│    x ────────► Horizontal position (left/right)                │
│                Units: inches                                    │
│                                                                 │
│    y ────────► Vertical position (forward/backward)            │
│                Units: inches                                    │
│                                                                 │
│    theta ───► Heading/orientation (which way you're facing)    │
│        (θ)    Units: degrees (or radians)                      │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Visual Representation

```
                    +Y
                     ↑
                     │
                     │    Robot at Pose(10, 15, 45°)
                     │         ↗
                     │        🤖  ← facing 45° (northeast)
              15" ───┼─────────●
                     │         │
                     │         │
    ─────────────────┼─────────┼─────────────► +X
                     │        10"
                     │
                     │
                     
    The pose (10, 15, 45°) means:
    - 10 inches to the right of origin
    - 15 inches forward from origin  
    - Facing 45° (between forward and right)
```

### Pose vs Position vs Point

| Term | What It Contains | Example |
|------|------------------|---------|
| **Point** | Just x, y | "The goal is at (24, 36)" |
| **Position** | Just x, y | Same as point |
| **Pose** | x, y, AND theta | "Robot is at (24, 36) facing 90°" |

A Pose is more than a point - it includes direction. This matters because robots aren't just *at* a location, they're *facing* a direction at that location.

---

## Why Pose Matters

### Everything Uses Poses

In shulib, Poses are everywhere:

```cpp
// Odometry returns a Pose
Pose current = chassis.getPose();

// Motion targets are Poses
moveToPose(chassis, Pose(24, 36, 90));

// Starting position is a Pose
chassis.setPose(0, 0, 0);

// Path planning uses Poses
std::vector<Pose> path = {
    Pose(0, 0, 0),
    Pose(12, 24, 45),
    Pose(24, 48, 90)
};
```

### The Alternative is Messy

Without a Pose class, you'd pass around separate values:

```cpp
// Without Pose (ugly, error-prone)
void moveTo(float x, float y, float theta);
float getCurrentX();
float getCurrentY();
float getCurrentTheta();

// Which parameter is which? Easy to mix up!
moveTo(90, 24, 36);  // Oops, put theta first by mistake!
```

```cpp
// With Pose (clean, self-documenting)
void moveTo(Pose target);
Pose getCurrent();

// Clear and hard to mess up
moveTo(Pose(24, 36, 90));
```

### Poses Enable Math

The Pose class overloads operators, letting you do intuitive math:

```cpp
Pose start(0, 0, 0);
Pose offset(10, 5, 0);

// Add poses to get new position
Pose end = start + offset;  // (10, 5, 0)

// Find distance between poses
float dist = start.distance(end);  // 11.18 inches

// Interpolate between poses
Pose midpoint = start.lerp(end, 0.5);  // (5, 2.5, 0)
```

---

## The Pose Class

### Class Definition

```cpp
namespace shulib {

class Pose {
public:
    // The three components
    float x;      // Horizontal position (inches)
    float y;      // Vertical position (inches)
    float theta;  // Heading (degrees or radians, depending on context)

    // Constructor
    Pose(float x, float y, float theta = 0);

    // Operators
    Pose operator+(const Pose& other) const;
    Pose operator-(const Pose& other) const;
    float operator*(const Pose& other) const;  // Dot product
    Pose operator*(const float& scalar) const;
    Pose operator/(const float& scalar) const;

    // Methods
    Pose lerp(Pose other, float t) const;
    float distance(Pose other) const;
    float angle(Pose other) const;
    Pose rotate(float angle) const;

    // String conversion
    operator std::string() const;
};

} // namespace shulib
```

### Member Variables

| Field | Type | Description |
|-------|------|-------------|
| `x` | `float` | X-coordinate in inches (positive = right) |
| `y` | `float` | Y-coordinate in inches (positive = forward) |
| `theta` | `float` | Heading angle (positive = counter-clockwise) |

**Note on theta units:** The Pose class itself is **unit-agnostic** for theta. It stores whatever you give it. Most shulib functions use degrees by default, with a `radians` parameter to switch. Always check the context!

---

## Creating Poses

### Basic Construction

```cpp
// Full pose: position and heading
Pose robotPose(24, 36, 90);  // x=24", y=36", facing 90°

// Position only (theta defaults to 0)
Pose point(24, 36);  // x=24", y=36", theta=0°

// Origin
Pose origin(0, 0, 0);
```

### Common Patterns

```cpp
// Get current pose from odometry
Pose current = chassis.getPose();

// Create a target pose
Pose target(48, 72, 180);

// Create a relative offset
Pose offset(10, 0, 0);  // 10 inches to the right

// Copy a pose
Pose copy = current;  // Makes a copy, not a reference
```

### From Coordinates

```cpp
// From field coordinates
float fieldX = 72;   // Center of field
float fieldY = 72;
float heading = 45;
Pose centerPose(fieldX, fieldY, heading);

// From sensor readings
float odomX = odometry.getX();
float odomY = odometry.getY();
float odomTheta = odometry.getTheta();
Pose sensorPose(odomX, odomY, odomTheta);
```

---

## Pose Arithmetic

The Pose class overloads standard operators for intuitive math operations.

### Addition: `Pose + Pose`

Adds the x and y components. **Theta comes from the LEFT operand.**

```cpp
Pose a(10, 20, 45);
Pose b(5, 10, 90);

Pose c = a + b;
// c.x = 10 + 5 = 15
// c.y = 20 + 10 = 30
// c.theta = 45 (from 'a', NOT summed!)
```

**Use case:** Applying an offset to a position.

```cpp
Pose robot(50, 50, 0);
Pose offset(10, 0, 0);  // 10 inches in X

Pose newPos = robot + offset;  // (60, 50, 0)
```

**⚠️ Important:** Addition does NOT add theta values. The result keeps the left operand's theta. This is intentional - adding angles rarely makes sense.

---

### Subtraction: `Pose - Pose`

Subtracts x and y components. **Theta comes from the LEFT operand.**

```cpp
Pose a(30, 40, 90);
Pose b(10, 15, 45);

Pose c = a - b;
// c.x = 30 - 10 = 20
// c.y = 40 - 15 = 25
// c.theta = 90 (from 'a')
```

**Use case:** Finding the vector from one position to another.

```cpp
Pose target(100, 100, 0);
Pose current(60, 80, 0);

Pose difference = target - current;  // (40, 20, 0)
// We need to go 40" in X and 20" in Y to reach target
```

---

### Multiplication: `Pose * Pose` (Dot Product)

Returns a **float** (not a Pose!) - the dot product of the x,y vectors.

```cpp
Pose a(3, 4, 0);
Pose b(2, 1, 0);

float dot = a * b;
// dot = (3 * 2) + (4 * 1) = 6 + 4 = 10
```

**What's a dot product?**

The dot product tells you how much two vectors "agree" in direction:
- Positive: Vectors point roughly the same way
- Zero: Vectors are perpendicular (90° apart)
- Negative: Vectors point roughly opposite ways

```
    a · b > 0          a · b = 0          a · b < 0
    
      ↗ a               ↑ a                 ↑ a
     ↗                  │                   │
    ↗ b                 └──→ b              ↓ b
    
   Same-ish           Perpendicular       Opposite-ish
   direction                              directions
```

**Use case:** Determining if the robot is facing toward or away from a target.

---

### Scalar Multiplication: `Pose * float`

Multiplies x and y by the scalar. **Theta is unchanged.**

```cpp
Pose a(10, 20, 45);

Pose b = a * 2.0;
// b.x = 10 * 2 = 20
// b.y = 20 * 2 = 40
// b.theta = 45 (unchanged)
```

**Use case:** Scaling a vector.

```cpp
Pose direction(1, 0, 0);  // Unit vector pointing right
Pose movement = direction * 24;  // Move 24 inches right
```

---

### Scalar Division: `Pose / float`

Divides x and y by the scalar. **Theta is unchanged.**

```cpp
Pose a(20, 40, 90);

Pose b = a / 2.0;
// b.x = 20 / 2 = 10
// b.y = 40 / 2 = 20
// b.theta = 90 (unchanged)
```

**Use case:** Normalizing or finding midpoints.

```cpp
Pose fullDistance(100, 50, 0);
Pose halfway = fullDistance / 2;  // (50, 25, 0)
```

---

## Pose Methods

### `lerp(Pose other, float t)` - Linear Interpolation

Returns a pose between `this` and `other`, based on parameter `t`.

```cpp
Pose lerp(Pose other, float t) const;
```

**Parameters:**
- `other`: The target pose
- `t`: Interpolation factor (0.0 = this pose, 1.0 = other pose)

**Returns:** A new Pose between the two

```cpp
Pose start(0, 0, 0);
Pose end(100, 50, 0);

Pose quarter = start.lerp(end, 0.25);   // (25, 12.5, 0)
Pose half = start.lerp(end, 0.5);       // (50, 25, 0)
Pose threeQuarter = start.lerp(end, 0.75); // (75, 37.5, 0)
```

**Visual:**
```
start                                    end
  ●────────●────────●────────●────────●
  │       t=0.25   t=0.5   t=0.75     │
t=0.0                               t=1.0
```

**Use case:** Path smoothing, animation, gradual movement.

```cpp
// Move toward target gradually
Pose current = chassis.getPose();
Pose target(100, 100, 0);

// Get a point 10% of the way toward target
Pose nextStep = current.lerp(target, 0.1);
```

**⚠️ Note:** `lerp` does NOT interpolate theta. Only x and y are interpolated.

---

### `distance(Pose other)` - Euclidean Distance

Returns the straight-line distance between two poses.

```cpp
float distance(Pose other) const;
```

**Parameters:**
- `other`: The other pose

**Returns:** Distance in inches (always positive)

```cpp
Pose a(0, 0, 0);
Pose b(3, 4, 0);

float dist = a.distance(b);  // 5.0 (it's a 3-4-5 triangle!)
```

**The math:**
```
distance = √[(x₂-x₁)² + (y₂-y₁)²]
distance = √[(3-0)² + (4-0)²]
distance = √[9 + 16]
distance = √25 = 5
```

**Visual:**
```
         b (3, 4)
         ●
        /│
       / │ 4
      /  │
     /   │
    ●────┘
   a     3
   (0,0)
   
   Distance = √(3² + 4²) = 5
```

**Use case:** Checking if you've arrived, calculating travel distance.

```cpp
Pose current = chassis.getPose();
Pose target(24, 36, 0);

if (current.distance(target) < 1.0) {
    // Within 1 inch of target - we're there!
}
```

**⚠️ Note:** `distance` ignores theta. It's purely positional distance.

---

### `angle(Pose other)` - Angle to Another Pose

Returns the angle from this pose TO the other pose.

```cpp
float angle(Pose other) const;
```

**Parameters:**
- `other`: The target pose

**Returns:** Angle in **radians** (not degrees!)

```cpp
Pose a(0, 0, 0);
Pose b(1, 1, 0);

float ang = a.angle(b);  // 0.785 radians = 45°
```

**The math:**
```
angle = atan2(other.y - this.y, other.x - this.x)
angle = atan2(1 - 0, 1 - 0)
angle = atan2(1, 1)
angle = π/4 ≈ 0.785 radians = 45°
```

**Visual:**
```
         b (1, 1)
         ●
        /
       / 45°
      /
     ●─────────→ X-axis
    a (0, 0)
```

**Use case:** Finding which direction to face to reach a target.

```cpp
Pose current = chassis.getPose();
Pose target(24, 36, 0);

float angleToTarget = current.angle(target);  // In radians!
float angleDegrees = angleToTarget * 180 / M_PI;  // Convert to degrees
```

**⚠️ Warning:** Returns **radians**, not degrees! Convert if needed.

**⚠️ Note:** This returns the angle FROM this pose TO the other, measured from the positive X-axis. It does NOT consider the current theta of either pose.

---

### `rotate(float angle)` - Rotate the Position Vector

Rotates the (x, y) position around the origin by the given angle.

```cpp
Pose rotate(float angle) const;
```

**Parameters:**
- `angle`: Rotation angle in **radians**

**Returns:** New Pose with rotated x, y (theta unchanged)

```cpp
Pose a(1, 0, 0);  // Point on the X-axis

Pose b = a.rotate(M_PI / 2);  // Rotate 90° counter-clockwise
// b.x ≈ 0
// b.y ≈ 1
// b.theta = 0 (unchanged)
```

**Visual:**
```
Before rotation:        After rotate(π/2):
                        
     +Y                      +Y
      │                       │
      │                       ● b (0, 1)
      │                       │
   ───●───► +X             ───┼───► +X
     a (1, 0)                 │
```

**The math:**
```
new_x = x × cos(angle) - y × sin(angle)
new_y = x × sin(angle) + y × cos(angle)
```

This is the standard 2D rotation matrix applied to the point.

**Use case:** Transforming coordinates between reference frames.

```cpp
// Convert a local offset to global coordinates
Pose localOffset(10, 0, 0);  // 10 inches to robot's right
float robotHeading = chassis.getPose().theta * M_PI / 180;  // Convert to radians

Pose globalOffset = localOffset.rotate(robotHeading);
```

**⚠️ Warning:** Angle must be in **radians**!

**⚠️ Note:** This rotates the (x, y) position vector, NOT the theta heading.

---

### `operator std::string()` - String Conversion

Converts the Pose to a human-readable string.

```cpp
operator std::string() const;
```

**Returns:** String in format `{ 'x':'...', 'y':'...', 't':'...' }`

```cpp
Pose p(12.5, 24.7, 45.0);
std::string s = p;  // "{ 'x':'12.500000', 'y':'24.700000', 't':'45.000000' }"

// Or explicitly
std::string str = static_cast<std::string>(p);
```

**Use case:** Debugging and logging.

```cpp
Pose current = chassis.getPose();
printf("Current pose: %s\n", static_cast<std::string>(current).c_str());
```

---

## Common Use Cases

### 1. Tracking Robot Position

```cpp
// Get where the robot is
Pose current = chassis.getPose();
printf("Robot at (%.1f, %.1f) facing %.1f°\n", 
       current.x, current.y, current.theta);
```

### 2. Setting Targets

```cpp
// Define where to go
Pose scoringPosition(48, 72, 90);
moveToPose(chassis, scoringPosition);
```

### 3. Calculating Travel Distance

```cpp
Pose start = chassis.getPose();
// ... robot moves ...
Pose end = chassis.getPose();

float traveled = start.distance(end);
printf("Traveled %.1f inches\n", traveled);
```

### 4. Finding Direction to Target

```cpp
Pose current = chassis.getPose();
Pose target(100, 50, 0);

// Get angle from current position to target
float angleRad = current.angle(target);
float angleDeg = angleRad * 180 / M_PI;

printf("Target is at %.1f° from current position\n", angleDeg);
```

### 5. Calculating Relative Position

```cpp
Pose target(100, 100, 0);
Pose current(60, 80, 0);

Pose difference = target - current;
// difference.x = 40 (need to go 40" in X)
// difference.y = 20 (need to go 20" in Y)
```

### 6. Path Interpolation

```cpp
Pose start(0, 0, 0);
Pose end(100, 100, 0);

// Generate 10 points along the path
for (float t = 0; t <= 1.0; t += 0.1) {
    Pose waypoint = start.lerp(end, t);
    printf("Waypoint: (%.1f, %.1f)\n", waypoint.x, waypoint.y);
}
```

### 7. Coordinate Transformation

```cpp
// Robot is at (50, 50) facing 45°
Pose robot(50, 50, 45);

// There's an object 20" in front of the robot (in robot's local frame)
Pose localObject(0, 20, 0);  // 0 right, 20 forward, in local coords

// Convert to global coordinates
float headingRad = robot.theta * M_PI / 180;
Pose rotatedOffset = localObject.rotate(headingRad);
Pose globalObject = robot + rotatedOffset;

// globalObject is now the object's position in field coordinates
```

### 8. Checking Arrival

```cpp
Pose target(24, 36, 90);
const float POSITION_TOLERANCE = 1.0;  // inches
const float ANGLE_TOLERANCE = 2.0;     // degrees

Pose current = chassis.getPose();

bool positionReached = current.distance(target) < POSITION_TOLERANCE;
bool angleReached = fabs(current.theta - target.theta) < ANGLE_TOLERANCE;

if (positionReached && angleReached) {
    printf("Arrived at target!\n");
}
```

---

## The Math Behind the Methods

### Distance Formula (Pythagorean Theorem)

```
                    b (x₂, y₂)
                    ●
                   /│
                  / │
           d    /  │ Δy = y₂ - y₁
              /    │
             /     │
            ●──────┘
           a       Δx = x₂ - x₁
         (x₁, y₁)

d = √(Δx² + Δy²)
d = √((x₂-x₁)² + (y₂-y₁)²)
```

This is the Euclidean distance - the straight line "as the crow flies."

### Angle Formula (Arctangent)

```
                    b (x₂, y₂)
                    ●
                   /
                  /
                 / θ
                /
               ●─────────────────► X
              a (x₁, y₁)

θ = atan2(y₂ - y₁, x₂ - x₁)
```

We use `atan2` instead of `atan` because:
- `atan` only gives angles in the range (-π/2, π/2)
- `atan2` gives angles in the full range (-π, π)
- `atan2` handles the case when Δx = 0 (division by zero)

### Rotation Matrix

Rotating a point (x, y) by angle θ around the origin:

```
┌    ┐   ┌              ┐ ┌   ┐
│ x' │   │ cos(θ) -sin(θ) │ │ x │
│    │ = │                │ │   │
│ y' │   │ sin(θ)  cos(θ) │ │ y │
└    ┘   └              ┘ └   ┘

x' = x·cos(θ) - y·sin(θ)
y' = x·sin(θ) + y·cos(θ)
```

**Visual intuition:**
```
Before (θ=0):     After (θ=45°):    After (θ=90°):

    +Y                 +Y                 +Y
     │                  │  ●               │
     │                  │ /                ●
     │                  │/                 │
     ●────► +X        ──●────► +X       ───┼───► +X
   (1,0)                                   │
```

### Linear Interpolation (Lerp)

```
lerp(a, b, t) = a + t × (b - a)
             = a × (1 - t) + b × t
```

When t = 0: result = a
When t = 1: result = b
When t = 0.5: result = midpoint

```
t=0      t=0.25    t=0.5     t=0.75    t=1
 a─────────●─────────●─────────●─────────b
```

### Dot Product

```
a · b = ax × bx + ay × by
      = |a| × |b| × cos(θ)
```

Where θ is the angle between vectors a and b.

**Geometric interpretation:**
- The dot product equals the length of the projection of one vector onto the other, scaled by the length of the other vector.
- If vectors are perpendicular, dot product = 0
- If vectors point the same direction, dot product is positive
- If vectors point opposite directions, dot product is negative

---

## Poses Throughout shulib

### In Odometry

```cpp
// odometry.cpp stores the current pose
shulib::Pose odomPose(0, 0, 0);

// getPose() returns it
shulib::Pose shulib::getPose(bool radians) {
    if (radians) return odomPose;
    else return Pose(odomPose.x, odomPose.y, radToDeg(odomPose.theta));
}

// setPose() sets it
void shulib::setPose(Pose pose, bool radians) {
    if (radians) odomPose = pose;
    else odomPose = Pose(pose.x, pose.y, degToRad(pose.theta));
}
```

### In Motion Functions

```cpp
void moveToPose(Chassis& chassis, Pose target, ...) {
    Pose current = chassis.getPose();
    
    // Calculate distance to target
    float dx = target.x - current.x;
    float dy = target.y - current.y;
    
    // Calculate angle to target
    float angleToTarget = atan2(dx, dy) * 180.0 / M_PI;
    
    // Calculate distance
    float distance = current.distance(target);
    
    // Move there
    rotateTo(chassis, angleToTarget);
    moveVertical(chassis, distance);
    rotateTo(chassis, target.theta);
}
```

### In the Chassis Class

```cpp
class Chassis {
    // ...
    
    void setPose(float x, float y, float theta, bool radians = false) {
        shulib::setPose(Pose(x, y, theta), radians);
    }
    
    void setPose(Pose pose, bool radians = false) {
        shulib::setPose(pose, radians);
    }
    
    Pose getPose(bool radians = false) {
        return shulib::getPose(radians);
    }
};
```

### In Logger/Telemetry

```cpp
// Logger has special handling for Pose
void Logger::updateTelemetry(const std::string& key, const Pose& pose) {
    std::stringstream ss;
    ss << "{\"x\":" << pose.x 
       << ",\"y\":" << pose.y 
       << ",\"theta\":" << pose.theta << "}";
    telemetryData[key] = ss.str();
}

// Usage:
logger().updateTelemetry("odometry", odomPose);
// Outputs: {"odometry": {"x": 12.5, "y": 24.3, "theta": 0.789}}
```

---

## Tips and Gotchas

### Gotcha 1: Theta Doesn't Add in `+` Operator

```cpp
Pose a(0, 0, 45);
Pose b(0, 0, 45);

Pose c = a + b;
// c.theta is 45, NOT 90!
```

Addition preserves the left operand's theta. If you want to add angles, do it explicitly:

```cpp
float newTheta = a.theta + b.theta;
```

### Gotcha 2: `angle()` Returns Radians

```cpp
Pose a(0, 0, 0);
Pose b(1, 1, 0);

float ang = a.angle(b);  // Returns ~0.785 (radians), not 45°!

// Convert to degrees if needed:
float angDeg = ang * 180 / M_PI;  // Now it's 45°
```

### Gotcha 3: `rotate()` Uses Radians Too

```cpp
Pose p(1, 0, 0);

// WRONG: This rotates by 90 radians, not 90 degrees!
Pose wrong = p.rotate(90);

// RIGHT: Convert degrees to radians first
Pose right = p.rotate(90 * M_PI / 180);  // or M_PI / 2
```

### Gotcha 4: Pose is Copied, Not Referenced

```cpp
Pose original(10, 20, 30);
Pose copy = original;  // This is a COPY

copy.x = 999;
// original.x is still 10!
```

This is usually what you want, but be aware of it.

### Gotcha 5: distance() Ignores Theta

```cpp
Pose a(0, 0, 0);
Pose b(0, 0, 180);

float dist = a.distance(b);  // Returns 0!
// They're at the same position, just facing opposite directions
```

If you need to consider heading difference, calculate it separately:

```cpp
float posDist = a.distance(b);
float angDist = fabs(a.theta - b.theta);
```

### Tip: Use Named Poses for Clarity

```cpp
// Instead of magic numbers everywhere:
moveToPose(chassis, Pose(24, 36, 90));
moveToPose(chassis, Pose(48, 72, 180));

// Use named constants:
const Pose SCORING_ZONE(24, 36, 90);
const Pose LOADING_ZONE(48, 72, 180);

moveToPose(chassis, SCORING_ZONE);
moveToPose(chassis, LOADING_ZONE);
```

### Tip: Create Helper Poses

```cpp
// Useful constant poses
const Pose ORIGIN(0, 0, 0);
const Pose FORWARD_UNIT(0, 1, 0);   // Unit vector pointing forward
const Pose RIGHT_UNIT(1, 0, 0);     // Unit vector pointing right
```

---

## API Reference

### Constructor

```cpp
Pose(float x, float y, float theta = 0)
```

Creates a new Pose.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `x` | `float` | (required) | X-coordinate in inches |
| `y` | `float` | (required) | Y-coordinate in inches |
| `theta` | `float` | `0` | Heading angle |

---

### Operators

| Operator | Signature | Returns | Description |
|----------|-----------|---------|-------------|
| `+` | `Pose + Pose` | `Pose` | Add x,y components (theta from left) |
| `-` | `Pose - Pose` | `Pose` | Subtract x,y components (theta from left) |
| `*` | `Pose * Pose` | `float` | Dot product of x,y vectors |
| `*` | `Pose * float` | `Pose` | Scale x,y by scalar (theta unchanged) |
| `/` | `Pose / float` | `Pose` | Divide x,y by scalar (theta unchanged) |

---

### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `lerp` | `lerp(Pose other, float t)` | `Pose` | Linear interpolation (0≤t≤1) |
| `distance` | `distance(Pose other)` | `float` | Euclidean distance (inches) |
| `angle` | `angle(Pose other)` | `float` | Angle to other pose (**radians**) |
| `rotate` | `rotate(float angle)` | `Pose` | Rotate x,y by angle (**radians**) |

---

### String Conversion

```cpp
operator std::string() const
```

Returns: `"{ 'x':'...', 'y':'...', 't':'...' }"`

---

## Key Takeaways

### The Essentials

1. **A Pose is (x, y, theta)** - position plus heading
   - x: left/right (inches)
   - y: forward/back (inches)
   - theta: heading (degrees or radians, context-dependent)

2. **Operators work on x,y only** (mostly)
   - Addition/subtraction: combine positions
   - Theta comes from the LEFT operand
   - Multiplication is dot product

3. **Method angle units matter**
   - `angle()` returns **radians**
   - `rotate()` takes **radians**
   - Always convert if working with degrees!

4. **Pose is used everywhere**
   - Odometry position
   - Motion targets
   - Path waypoints
   - Coordinate transforms

### The One-Sentence Summary

> **A Pose bundles x, y, and theta into a single object with convenient math operations, making it easy to work with robot positions and orientations throughout the codebase.**

---

## Where to Go Next

| Topic | Document | What You'll Learn |
|-------|----------|-------------------|
| Coordinate conventions | [COORDINATE_SYSTEM.md](../reference/COORDINATE_SYSTEM.md) | Field coordinates, units, conventions |
| Using poses in odometry | [ODOMETRY.md](./ODOMETRY.md) | How poses are tracked |
| Motion with poses | [MOVE_TO_POSE.md](../motion/MOVE_TO_POSE.md) | Moving to target poses |
| Unit conversions | [UNITS.md](../reference/UNITS.md) | Degrees, radians, inches |
| Utility functions | [util.hpp](../../include/shulib/core/util.hpp) | Related math utilities |

---

*Document last updated: January 2026*