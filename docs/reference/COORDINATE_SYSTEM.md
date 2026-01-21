# Coordinate System

**X, Y, and Theta conventions in shulib**

---

## The Basics

### Our Coordinate System

```
                      +Y (forward)
                         ↑
                         │
                         │
                         │
          -X ←───────────┼───────────→ +X
          (left)         │              (right)
                         │
                         │
                         ↓
                      -Y (backward)


                    Heading (θ):
                    
                       0° (forward)
                         ↑
                         │
                         │
        -90° (left) ←────┼────→ +90° (right)
                         │
                         │
                         ↓
                    ±180° (backward)
```

### Summary

| Direction | Coordinate |
|-----------|------------|
| Forward | +Y |
| Backward | -Y |
| Right | +X |
| Left | -X |
| Turn right (clockwise) | +θ |
| Turn left (counter-clockwise) | -θ |

---

## Position (X, Y)

### The X Axis

**X measures left/right position:**

```
    ← Negative X          Positive X →
    
    X = -24              X = 0              X = +24
       │                   │                   │
       ▼                   ▼                   ▼
    ───●───────────────────●───────────────────●───
    (left)              (center)            (right)
```

### The Y Axis

**Y measures forward/backward position:**

```
              Y = +24   ← Forward
                 ●
                 │
                 │
    Y = 0   ────●──── (starting position)
                 │
                 │
                 ●
              Y = -24  ← Backward
```

### Combined X and Y

```
                    +Y
                     ↑
            (-12,24) │ (12,24)
               ●     │     ●
                     │
    -X ←─────────────●─────────────→ +X
                  (0,0)
               ●     │     ●
           (-12,-24) │ (12,-24)
                     ↓
                    -Y
```

---

## Heading (Theta, θ)

### Angle Reference

```
                    0°
                    ↑
                    │
                    │
                    │
    -90° ←──────────┼──────────→ 90°
    (270°)          │           
                    │
                    │
                    ↓
                ±180°
```

### Angle Values

| Heading | Direction | Description |
|---------|-----------|-------------|
| 0° | ↑ | Facing forward (default) |
| 45° | ↗ | Facing front-right |
| 90° | → | Facing right |
| 135° | ↘ | Facing back-right |
| 180° / -180° | ↓ | Facing backward |
| -135° (225°) | ↙ | Facing back-left |
| -90° (270°) | ← | Facing left |
| -45° (315°) | ↖ | Facing front-left |

### Positive = Clockwise

Turning **right** (clockwise) **increases** theta:

```
Start: 0°     After right turn: 90°
   ↑                    →
   │                    │
   ●                    ●
   
   θ = 0°               θ = 90° (+90°)
```

Turning **left** (counter-clockwise) **decreases** theta:

```
Start: 0°     After left turn: -90°
   ↑                    ←
   │                    │
   ●                    ●
   
   θ = 0°               θ = -90° (-90°)
```

---

## The Origin

### Where is (0, 0, 0)?

The origin is wherever you **set it** at the start of autonomous:

```cpp
chassis.setPose(0, 0, 0);  // "Right here is the origin, facing forward"
```

### Common Origin Strategies

**Strategy 1: Robot starts at origin**
```cpp
// Robot's starting position is (0, 0)
// Robot's starting heading is 0°
chassis.setPose(0, 0, 0);

// All movements are relative to start
moveToPose(chassis, Pose(24, 36, 0));  // 24" right, 36" forward from start
```

**Strategy 2: Field-centric coordinates**
```cpp
// Robot starts at actual field position
// Field center would be (72, 72)
chassis.setPose(12, 12, 0);  // Starting in corner

// Movements are to absolute field positions
moveToPose(chassis, Pose(72, 72, 0));  // Go to field center
```

**Recommendation:** Use Strategy 1 (origin at start) for simplicity.

---

## Pose Representation

### The Pose Class

```cpp
Pose(x, y, theta)
```

| Component | Unit | Description |
|-----------|------|-------------|
| `x` | Inches | Left/right position |
| `y` | Inches | Forward/backward position |
| `theta` | Degrees | Heading angle |

### Examples

```cpp
Pose(0, 0, 0)       // At origin, facing forward
Pose(24, 0, 0)      // 24" right of origin, facing forward
Pose(0, 36, 0)      // 36" forward of origin, facing forward
Pose(24, 36, 90)    // 24" right, 36" forward, facing right
Pose(-12, 24, -45)  // 12" left, 24" forward, facing front-left
```

### Visualizing Poses

```
           Pose(0, 36, 0)
                ↑
                ●
                
    Pose(-24, 24, -90)              Pose(24, 24, 90)
         ←●                              ●→
                
                ●
           Pose(0, 0, 0)
           (origin, facing forward)
```

---

## Movement Directions

### moveVertical()

Positive distance = forward (in robot's facing direction):

```cpp
moveVertical(chassis, 24);   // Forward 24"
moveVertical(chassis, -24);  // Backward 24"
```

**Note:** "Forward" is relative to robot's current heading, not the Y axis!

```
If robot is facing 0° (forward):
  moveVertical(24) → +Y direction

If robot is facing 90° (right):
  moveVertical(24) → +X direction
  
If robot is facing 180° (backward):
  moveVertical(24) → -Y direction
```

### rotateTo()

Specifies **absolute** heading:

```cpp
rotateTo(chassis, 90);   // Turn to face 90° (right)
rotateTo(chassis, 0);    // Turn to face 0° (forward)
rotateTo(chassis, -90);  // Turn to face -90° (left)
```

Not relative! If robot is at 45° and you call `rotateTo(90)`, it turns 45° right.

### moveToPose()

Specifies **absolute** position:

```cpp
moveToPose(chassis, Pose(24, 36, 0));  // Go to (24, 36) facing 0°
```

---

## Angle Math

### Calculating Angle to Target

```cpp
Pose current = chassis.getPose();
Pose target(24, 36, 0);

// Difference
float dx = target.x - current.x;
float dy = target.y - current.y;

// Angle (returns RADIANS!)
float angleRad = atan2(dx, dy);  // Note: dx first for our coordinate system

// Convert to degrees
float angleDeg = angleRad * 180.0 / M_PI;
```

**Why atan2(dx, dy) not atan2(dy, dx)?**

Standard math uses atan2(y, x) where 0° is along +X axis.
Our system has 0° along +Y axis, so we swap: atan2(dx, dy).

### The Pose.angle() Method

```cpp
float angleToTarget = current.angle(target);  // Returns RADIANS
```

---

## Common Mistakes

### Mistake 1: Confusing X and Y

```cpp
// WRONG: Thinking X is forward
moveToPose(chassis, Pose(24, 0, 0));  // This goes RIGHT, not forward!

// RIGHT: Y is forward
moveToPose(chassis, Pose(0, 24, 0));  // This goes forward
```

### Mistake 2: Forgetting Theta is Degrees

```cpp
// WRONG: Using radians for Pose.theta
Pose target(24, 36, M_PI/2);  // 1.57 degrees, not 90°!

// RIGHT: Using degrees
Pose target(24, 36, 90);  // 90 degrees
```

### Mistake 3: Confusing Relative vs Absolute

```cpp
// rotateTo is ABSOLUTE
rotateTo(chassis, 90);  // Turn to 90°, regardless of current heading

// To turn a relative amount:
Pose current = chassis.getPose();
rotateTo(chassis, current.theta + 45);  // Turn 45° from current
```

### Mistake 4: moveVertical Direction

```cpp
// moveVertical moves in ROBOT's forward direction
// If robot faces 90° (right), moveVertical goes in +X, not +Y

// To guarantee movement along Y axis, face 0° first:
rotateTo(chassis, 0);
moveVertical(chassis, 24);  // Now definitely +Y
```

---

## Field Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                                                                 │
│     (0, 144)                                      (144, 144)    │
│         ●───────────────────────────────────────────●           │
│         │                                           │           │
│         │                    ↑                      │           │
│         │                   +Y                      │           │
│         │                    │                      │           │
│         │                    │                      │           │
│         │                    │                      │           │
│         │       ←───────────(72,72)──────────→      │           │
│         │       -X           │           +X         │           │
│         │                    │                      │           │
│         │                    │                      │           │
│         │                   -Y                      │           │
│         │                    ↓                      │           │
│         │                                           │           │
│         ●───────────────────────────────────────────●           │
│     (0, 0)                                        (144, 0)      │
│                                                                 │
│     Field is 144" × 144" (12' × 12')                           │
│     Each tile is 24" × 24" (6 × 6 tiles)                       │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Quick Reference

| Question | Answer |
|----------|--------|
| Which way is +X? | Right |
| Which way is +Y? | Forward |
| Which way is +θ? | Clockwise (turning right) |
| What unit is X, Y? | Inches |
| What unit is θ? | Degrees |
| Where is (0,0)? | Where you call setPose(0,0,0) |
| What's 90°? | Facing right |
| What's -90°? | Facing left |

---

*For unit details, see [UNITS.md](./UNITS.md)*
*For term definitions, see [GLOSSARY.md](./GLOSSARY.md)*