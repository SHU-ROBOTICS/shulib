# Units

**Unit conventions and conversions in shulib**

---

## Quick Reference

| Quantity | shulib Unit | Notes |
|----------|-------------|-------|
| Distance | **Inches** | All distances, offsets, diameters |
| Angles (API) | **Degrees** | Pose.theta, rotateTo(), moveToPose() |
| Angles (math) | **Radians** | atan2(), sin(), cos(), rotate() |
| Motor power | **-127 to 127** | Percentage of maximum |
| Time | **Milliseconds** | pros::delay(), timeouts |
| Sensor rotation | **Centidegrees** | 36,000 per revolution |

---

## Distance: Inches

### Convention

All distances in shulib are in **inches**:

```cpp
// Wheel diameter
.wheel_diameter = 3.25  // 3.25 inches

// Tracking wheel offset
.left_offset = -6.5  // 6.5 inches left of center

// Movement
moveVertical(chassis, 24);  // 24 inches

// Pose
Pose target(48, 72, 0);  // x=48", y=72"
```

### Why Inches?

- VEX field is 12 feet = 144 inches
- Easy mental math (field is 144 × 144)
- Common in US robotics

### Converting from Other Units

| From | To Inches | Formula |
|------|-----------|---------|
| Feet | Inches | `inches = feet × 12` |
| Centimeters | Inches | `inches = cm / 2.54` |
| Meters | Inches | `inches = meters × 39.37` |
| Millimeters | Inches | `inches = mm / 25.4` |

```cpp
// Conversion functions
float feetToInches(float feet) {
    return feet * 12.0;
}

float cmToInches(float cm) {
    return cm / 2.54;
}
```

### Common Measurements

| Object | Size |
|--------|------|
| Field | 144" × 144" (12' × 12') |
| Tile | 24" × 24" (2' × 2') |
| Robot max starting | 18" × 18" × 18" |
| 3.25" omni wheel | 3.25" diameter |
| 2.75" tracking wheel | 2.75" diameter |

---

## Angles: Degrees vs Radians

### The Rule

**User-facing functions use DEGREES:**
```cpp
Pose pose(0, 0, 90);        // 90 degrees
rotateTo(chassis, 45);      // 45 degrees
chassis.setPose(0, 0, 180); // 180 degrees
```

**Math functions use RADIANS:**
```cpp
float rad = atan2(dy, dx);  // Returns radians
Pose rotated = p.rotate(rad);  // Expects radians
float sine = sin(rad);      // Expects radians
```

### Converting

```cpp
// Degrees to Radians
float degToRad(float degrees) {
    return degrees * M_PI / 180.0;
}

// Radians to Degrees
float radToDeg(float radians) {
    return radians * 180.0 / M_PI;
}
```

**Common conversions:**

| Degrees | Radians | Expression |
|---------|---------|------------|
| 0° | 0 | 0 |
| 30° | π/6 | 0.524 |
| 45° | π/4 | 0.785 |
| 60° | π/3 | 1.047 |
| 90° | π/2 | 1.571 |
| 180° | π | 3.142 |
| 270° | 3π/2 | 4.712 |
| 360° | 2π | 6.283 |

### Why Both?

- **Degrees** are human-friendly (we think in degrees)
- **Radians** are math-friendly (calculus, trig functions)

### Gotcha: Pose.angle() Returns Radians!

```cpp
Pose current(0, 0, 0);
Pose target(10, 10, 0);

// angle() returns RADIANS
float angleRad = current.angle(target);  // ~0.785 radians

// Convert to degrees for display or rotateTo()
float angleDeg = radToDeg(angleRad);     // 45 degrees
rotateTo(chassis, angleDeg);
```

---

## Motor Power: -127 to 127

### Convention

Motor power is specified as a percentage of maximum:

| Value | Meaning |
|-------|---------|
| 127 | Full power forward |
| 64 | Half power forward |
| 0 | Stop |
| -64 | Half power reverse |
| -127 | Full power reverse |

```cpp
motor.move(127);   // Full forward
motor.move(-127);  // Full reverse
motor.move(50);    // ~40% forward
```

### Why 127?

- V5 motors use signed 8-bit values internally
- Range: -128 to 127
- We use -127 to 127 for symmetry

### Converting from Percentage

```cpp
// If you think in 0-100%:
int power = percentage * 127 / 100;

// Examples:
// 50% = 63 (actually 64 for 50.4%)
// 75% = 95
// 100% = 127
```

---

## Time: Milliseconds

### Convention

All time values in PROS/shulib are in **milliseconds**:

```cpp
pros::delay(1000);  // Wait 1 second (1000 ms)
pros::delay(10);    // Wait 10 ms

const int TIMEOUT = 3000;  // 3 second timeout
```

### Common Values

| Duration | Milliseconds |
|----------|--------------|
| 10 ms | 10 | Loop iteration |
| 100 ms | 100 | Telemetry interval |
| 500 ms | 500 | Half second |
| 1 second | 1000 | |
| 2 seconds | 2000 | Typical turn timeout |
| 3 seconds | 3000 | Typical move timeout |
| 15 seconds | 15000 | Autonomous period |
| 1:45 | 105000 | Driver control period |

### Converting

```cpp
// Seconds to milliseconds
int ms = seconds * 1000;

// Milliseconds to seconds
float seconds = ms / 1000.0;
```

### Getting Current Time

```cpp
uint32_t now = pros::millis();  // Milliseconds since program start
```

---

## Sensor Values: Centidegrees

### Rotation Sensor

V5 Rotation Sensors report position in **centidegrees** (1/100 of a degree):

| Rotation | Centidegrees |
|----------|--------------|
| 1° | 100 |
| 90° | 9,000 |
| 180° | 18,000 |
| 360° (1 rev) | 36,000 |

```cpp
pros::Rotation sensor(8);
int pos = sensor.get_position();  // Returns centidegrees
```

### Converting to Distance

OdomUnit handles this automatically, but the formula is:

```cpp
// Centidegrees to inches
float revolutions = centidegrees / 36000.0;
float circumference = diameter * M_PI;
float distance = revolutions * circumference;

// Simplified:
float distance = centidegrees * diameter * M_PI / 36000.0;
```

### Motor Encoders

V5 Motor encoders also use centidegrees:

```cpp
pros::Motor motor(1);
double pos = motor.get_position();  // Centidegrees
```

---

## Angle Normalization

### The Problem

Angles can be represented multiple ways:
- 90° = -270° = 450° = -630°

### Normalizing to [-180°, 180°]

```cpp
float normalizeAngle(float angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}
```

### Normalizing to [0°, 360°]

```cpp
float normalizeAngle360(float angle) {
    while (angle >= 360) angle -= 360;
    while (angle < 0) angle += 360;
    return angle;
}
```

### When to Normalize

- **Finding shortest turn:** Use [-180°, 180°]
- **Display/logging:** Either works
- **Comparing angles:** Always normalize first

---

## Unit Summary Table

| Context | Unit | Example |
|---------|------|---------|
| `Pose.x`, `Pose.y` | Inches | `Pose(24, 36, 0)` |
| `Pose.theta` | Degrees | `Pose(0, 0, 90)` |
| `moveVertical()` | Inches | `moveVertical(chassis, 24)` |
| `rotateTo()` | Degrees | `rotateTo(chassis, 90)` |
| `Pose.angle()` | **Radians** | Returns ~1.57 for 90° |
| `Pose.rotate()` | **Radians** | `pose.rotate(M_PI/2)` |
| `motor.move()` | -127 to 127 | `motor.move(100)` |
| `pros::delay()` | Milliseconds | `pros::delay(1000)` |
| `sensor.get_position()` | Centidegrees | Returns 36000 per rev |
| Wheel diameter | Inches | `.wheel_diameter = 2.75` |
| Offsets | Inches | `.left_offset = -6.5` |

---

## Conversion Cheat Sheet

```cpp
// Distance
inches = feet * 12;
inches = cm / 2.54;

// Angle
radians = degrees * M_PI / 180.0;
degrees = radians * 180.0 / M_PI;

// Time
milliseconds = seconds * 1000;
seconds = milliseconds / 1000.0;

// Sensor
degrees = centidegrees / 100.0;
revolutions = centidegrees / 36000.0;
inches = centidegrees * wheelDiameter * M_PI / 36000.0;

// Motor (percentage to power)
power = percentage * 127 / 100;
```

---

*For coordinate system, see [COORDINATE_SYSTEM.md](./COORDINATE_SYSTEM.md)*
*For term definitions, see [GLOSSARY.md](./GLOSSARY.md)*