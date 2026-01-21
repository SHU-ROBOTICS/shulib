# Odometry

**Location:** `include/shulib/core/odometry.hpp` and `src/core/odometry.cpp`

---

## Table of Contents

1. [What is Odometry?](#what-is-odometry)
2. [Why Should You Care?](#why-should-you-care)
3. [The Fundamental Problem](#the-fundamental-problem)
4. [Our Solution: Three-Wheel Odometry](#our-solution-three-wheel-odometry)
5. [The Coordinate System](#the-coordinate-system)
6. [The Mathematics of Position Tracking](#the-mathematics-of-position-tracking)
7. [From Theory to Code](#from-theory-to-code)
8. [The Update Loop](#the-update-loop)
9. [API Reference](#api-reference)
10. [Integration with the Chassis](#integration-with-the-chassis)
11. [Telemetry and Debugging](#telemetry-and-debugging)
12. [Correction Factors](#correction-factors)
13. [Testing Your Understanding](#testing-your-understanding)
14. [Historical Context](#historical-context)
15. [Key Takeaways](#key-takeaways)
16. [Where to Go Next](#where-to-go-next)

---

## What is Odometry?

### The Simple Explanation

**Odometry** is how your robot knows where it is.

Imagine you're blindfolded and dropped somewhere in a gymnasium. Someone gives you instructions:
- "Walk forward 10 steps"
- "Turn right 90 degrees"
- "Walk forward 5 steps"

Even blindfolded, you could point back to where you started. You kept track of your movements in your head. That's odometry - **using movement measurements to figure out position**.

### The Technical Definition

Odometry (from Greek *odos* = "path" and *metron* = "measure") is the use of motion sensors to estimate change in position over time. In robotics, we continuously measure small movements and accumulate them to maintain a running estimate of where we are.

### What We Actually Track

Our odometry system maintains three values, updated 100 times per second:

```
┌─────────────────────────────────────────────────────────────┐
│                     THE ROBOT'S POSE                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│    X Position ────► How far left or right (inches)          │
│                                                             │
│    Y Position ────► How far forward or backward (inches)    │
│                                                             │
│    Theta (θ) ─────► Which direction we're facing (degrees)  │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

Together, these three numbers completely describe where the robot is and which way it's pointing. We call this a **Pose**.

### An Intuition for Pose

Think of giving someone directions to a specific spot in a room:

- **Bad directions**: "Go over there" ❌
- **Better directions**: "Go to the corner" (position only) ⚠️
- **Complete directions**: "Stand in the corner, facing the door" ✓

The complete directions are a Pose - position AND orientation.

---

## Why Should You Care?

### The Autonomous Challenge

In VEX competition, you get **15 seconds of autonomous** where the robot runs completely on its own. No joystick. No driver. Just code.

In those 15 seconds, your robot might need to:
1. Drive to a scoring zone
2. Pick up game pieces
3. Navigate around obstacles
4. Score in multiple locations
5. Return to a parking zone

**How does the robot know where to go?**

### Approach 1: Time-Based Movement (The Bad Way)

```cpp
void autonomous_bad() {
    // "Drive forward for 1 second, that should be about 24 inches..."
    drivetrain.move(100);
    pros::delay(1000);
    drivetrain.stop();
    
    // "Turn for half a second, that's probably 90 degrees..."
    drivetrain.turn(100);
    pros::delay(500);
    drivetrain.stop();
}
```

**Why this fails:**
| Problem | What Happens |
|---------|--------------|
| Battery at 100% vs 80% | Robot goes different distances |
| Tile friction varies | Some spots are slippery |
| Robot weight changes | With/without game pieces |
| Wheels wear down | Less grip over time |
| Getting bumped | No way to recover |
| Different robot | Completely different timing |

You'd have to re-tune EVERYTHING constantly. And it'll never be reliable.

### Approach 2: Position-Based Movement (The Good Way)

```cpp
void autonomous_good() {
    // "Go to position (0, 24) - exactly 24 inches forward"
    while (getDistanceToTarget() > 0.5) {
        Pose current = odometry.getPose();
        // Calculate how to get there from HERE
        // Adjust motors based on CURRENT position
    }
    
    // "Turn to face 90 degrees"
    while (abs(odometry.getPose().theta - 90) > 1) {
        // Keep turning until we're actually at 90°
    }
}
```

**Why this works:**
- Battery low? Motors just run longer until you arrive.
- Got bumped? Odometry sees the error and corrects.
- Different robot? Same coordinates work (with proper calibration).
- Consistent results match after match.

### Real Competition Impact

| Metric | Without Odometry | With Odometry |
|--------|------------------|---------------|
| Reliable scoring positions | 1-2 | 4-6 |
| Recovery from bumps | None | Automatic |
| Code reusability | Low | High |
| Tuning time | Hours per match | Once per robot |
| Consistency | ~60% | ~95% |

**Odometry is often the difference between winning and losing close matches.**

---

## The Fundamental Problem

### The Challenge of Knowing Where You Are

Consider this: your robot has no GPS, no cameras watching from above, no beacons on the walls. It only knows what its own sensors tell it.

The sensors we have:
- **Motor encoders**: Count motor rotations
- **Rotation sensors**: Count wheel rotations
- **IMU (Inertial Measurement Unit)**: Measures rotation rate

None of these directly tell us position. They tell us about *movement*. Our job is to convert movement measurements into position estimates.

### Why Motor Encoders Aren't Enough

Your first thought might be: "We have 10 motors with encoders. Can't we use those?"

**The problem: Wheels slip.**

```
What the motor encoder sees:      What actually happens:
        
     ─────────────►                ────────►
     "Rotated 360°!                "Only went 80% as far
      Should be 10 inches!"         because wheels slipped
                                    during acceleration"
```

Motor encoders measure *motor rotation*, not *ground truth*. When you accelerate hard, turn sharply, or get pushed, the drive wheels slip. The encoder happily counts rotations that didn't translate to actual movement.

### The Solution: Dedicated Tracking Wheels

Instead of using drive wheels (which slip), we use **unpowered wheels** that just roll along with the robot:

```
        DRIVE WHEELS                    TRACKING WHEELS
        
    ┌────────────────┐              ┌────────────────┐
    │   ◄── Motor    │              │   No motor     │
    │       power    │              │   Just rolls   │
    │                │              │   freely       │
    │   Can slip!    │              │   Can't slip!  │
    │   (torque)     │              │   (no torque)  │
    └────────────────┘              └────────────────┘
```

Tracking wheels:
- Have no motor, so they can't slip from torque
- Only rotate when the robot actually moves
- Give us "ground truth" about actual motion

---

## Our Solution: Three-Wheel Odometry

### Why Three Wheels?

Think about what movements a robot on a flat surface can make:

```
MOVEMENT TYPE           HOW TO DETECT IT
─────────────────────────────────────────────────────────

Forward/Backward        Left and right wheels both rotate
                        the same direction
    ↑
    │                   dL ≈ dR (both positive or both negative)
    ↓

Rotation (turning)      Left and right wheels rotate
                        opposite amounts
    ↺                   
                        dL ≠ dR (one more than other, or opposite)

Sideways (strafing)     Left and right wheels don't detect this!
    ←→                  A horizontal wheel is needed.
                        
                        dL ≈ dR ≈ 0, but dBack ≠ 0
```

**Two wheels** (left + right) can detect forward/back and rotation, but NOT sideways drift.

**Three wheels** (left + right + back horizontal) can detect ALL planar motion!

### Our Physical Configuration

```
                    FRONT OF ROBOT
                         ↑
              ┌─────────────────────┐
              │                     │
              │    ┌─────────┐      │
         ┌────┤    │  BRAIN  │      ├────┐
         │ L  │    └─────────┘      │  R │
         │ ║  │                     │  ║ │
         │ ║  │                     │  ║ │
         └────┤                     ├────┘
              │       ════         │
              │        B           │
              │                     │
              └─────────────────────┘
                    BACK OF ROBOT
                    
    L = Left tracking wheel (vertical - measures Y movement)
    R = Right tracking wheel (vertical - measures Y movement)
    B = Back tracking wheel (horizontal - measures X movement)
    
    ║ = Wheel rolls this direction (perpendicular to axle)
    ═ = Wheel rolls this direction (perpendicular to axle)
```

### What Each Wheel Tells Us

| Wheel | Orientation | Measures | Used For |
|-------|-------------|----------|----------|
| Left | Vertical | Forward/back on left side | Y position, rotation |
| Right | Vertical | Forward/back on right side | Y position, rotation |
| Back | Horizontal | Left/right movement | X position (drift) |

### The Key Insight: Rotation from Wheel Difference

When the robot turns, the left and right wheels travel different distances:

```
             Turning RIGHT (clockwise):
             
                    ┌─────────┐
                    │    ↻    │
                    │         │
           travels  │  PIVOT  │  travels
           LESS     │  POINT  │  MORE
             │      │         │      │
             ▼      │         │      ▼
            dL=2"   │         │   dR=4"
                    │         │
                    └─────────┘
                    
    The RIGHT wheel is on the OUTSIDE of the turn,
    so it travels a longer arc.
    
    The DIFFERENCE tells us how much we rotated!
```

**Rotation = (dR - dL) / track_width**

This is the fundamental equation that makes it all work.

---

## The Coordinate System

### Defining Our World

Before we can track position, we need to agree on what position means. We use a standard 2D coordinate system:

```
                              +Y
                               ↑
                               │
                               │
                               │
              Quadrant II      │      Quadrant I
              (-X, +Y)         │       (+X, +Y)
                               │
         ─────────────────────0,0─────────────────────► +X
                               │
              Quadrant III     │      Quadrant IV
              (-X, -Y)         │       (+X, -Y)
                               │
                               │
                               ↓
                              -Y
```

### Our Conventions

| Aspect | Convention | Notes |
|--------|------------|-------|
| X-axis | Positive = Right | From robot's starting perspective |
| Y-axis | Positive = Forward | The direction robot initially faces |
| Origin (0,0) | Robot's starting position | You can change this! |
| Units | Inches | All distances in inches |
| Theta = 0° | Facing +Y (forward) | Robot's initial heading |
| Positive rotation | Counter-clockwise | Math standard (not intuitive!) |

### Heading (Theta) Reference

```
                         0° 
                         ↑
                         │
                         │
            315° ────────┼──────── 45°
            (-45°)       │
                         │
     270° ───────────────┼───────────────── 90°
    (-90°)               │
                         │
            225° ────────┼──────── 135°
           (-135°)       │
                         │
                         ↓
                       180°
                      (-180°)
```

**Common angles:**
- 0° = Forward
- 90° = Facing right
- -90° (or 270°) = Facing left  
- 180° (or -180°) = Facing backward

> **⚠️ NOTE:** Positive rotation is COUNTER-CLOCKWISE. This is standard in mathematics but feels backwards to many people. When in doubt, test with the actual robot!

### Why Starting Position Matters

When you call `setPose(0, 0, 0)`, you're telling the robot "this spot is the origin, and you're facing forward."

All future positions are **relative to this starting point**.

```cpp
// Example: Robot starts against the left wall, 12" from corner
chassis.setPose(0, 0, 0);  // This corner is now (0,0)

// Later, the robot is at (24, 36)
// That means: 24" to the right, 36" forward from where it started
```

> **Detailed coordinate conventions are in [COORDINATE_SYSTEM.md](../reference/COORDINATE_SYSTEM.md)**

---

## The Mathematics of Position Tracking

This is the heart of odometry. We'll build up the math step by step, from simple to complete.

### The Reference: 5225A Pilons Document

Our implementation is based on the legendary **5225A Pilons Tracking Document**:

📄 **[http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf](http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf)**

This document is considered THE definitive resource for VEX odometry. If you want to truly understand the math, read it. We'll cover the key concepts here.

### Starting Simple: One Dimension

Let's start with the simplest case - a robot that can only move forward and backward in a straight line.

```
    Start                    End
      ●────────────────────────●
      0"                      24"
      
    Position = Previous Position + Distance Traveled
    
    new_y = old_y + distance
    new_y = 0 + 24 = 24"
```

That's trivial. Just add up distances.

### Adding Rotation: Two Dimensions

Now let's add turning. This is where it gets interesting.

If the robot moves forward while also turning, it travels along an **arc**, not a straight line:

```
                        ╭───────╮  End
                       ╱         ╲
                      ╱           ●
                     ╱
                    ╱
                   ╱   The robot traveled
                  ╱    along this curved path
                 ╱
                ╱
               ●
             Start
```

We can't just add X and Y separately anymore. We need to account for the changing direction.

### The Core Equations

Here's the key insight: even though the robot travels along an arc, we can calculate where it ends up by thinking about two things separately:

1. **How much did it rotate?** (Δθ)
2. **How far did it move in its local frame?** (local ΔX and ΔY)

Then we rotate that local movement by the robot's heading to get global coordinates.

#### Step 1: Calculate Rotation (Δθ)

```
             LEFT WHEEL                    RIGHT WHEEL
                │                              │
                │         ┌────────┐          │
                │         │        │          │
                ●─────────┤ center ├──────────●
               dL         │        │          dR
                │         └────────┘          │
                │                              │
                ▼                              ▼
            traveled                       traveled
             less                           more
```

When the robot turns, the left and right wheels trace different arc lengths:

```
Δθ = (dR - dL) / (sL - sR)
```

Where:
- `dR` = distance traveled by right wheel (inches)
- `dL` = distance traveled by left wheel (inches)
- `sR` = right wheel's offset from center (positive, e.g., +6.5")
- `sL` = left wheel's offset from center (negative, e.g., -6.5")
- `(sL - sR)` = total track width (e.g., -6.5 - 6.5 = -13")

**Example calculation:**
```
Robot turns right. Right wheel goes farther than left.
dR = 0.10"
dL = 0.05"
sL = -6.5"
sR = +6.5"

Δθ = (0.10 - 0.05) / (-6.5 - 6.5)
Δθ = 0.05 / -13
Δθ = -0.00385 radians
Δθ ≈ -0.22°

Negative = clockwise = turning right ✓
```

#### Step 2: Calculate Local Displacement

Now we need to figure out how far the robot moved in its **own reference frame** (relative to itself, not the field).

##### Case A: Robot Went Straight (Δθ ≈ 0)

If there's barely any rotation:

```
local_ΔY = (dL + dR) / 2     // Average of left and right
local_ΔX = dS                 // Back wheel measures strafe directly
```

##### Case B: Robot Turned While Moving

When the robot turns and moves simultaneously, it traces an arc. The math is more complex:

```
We need the CHORD length (straight line from start to end of arc):

    chord = 2 × radius × sin(θ/2)

For forward/back:
    radius = (dR / Δθ) + sR
    local_ΔY = 2 × sin(Δθ/2) × radius

For sideways:
    radius = (dS / Δθ) + sS
    local_ΔX = 2 × sin(Δθ/2) × radius
```

**Why chord length?**

```
            We want THIS distance
                    │
                    ▼
               ●────────●
              ╱          ╲  End
             ╱            ╲
            ╱   NOT the    ╲
           ╱   arc length   ╲
          ╱                  ╲
         ●                    
       Start          
```

The chord formula `2r×sin(θ/2)` gives us the straight-line distance between start and end of an arc.

#### Step 3: Convert to Global Coordinates

We now have how far the robot moved **relative to itself** (local_ΔX, local_ΔY). But the field doesn't rotate with the robot! We need to rotate these into **field coordinates**.

```
             LOCAL FRAME                    GLOBAL FRAME
        (moves with robot)               (fixed to field)
        
              ↑ forward                        ↑ +Y
              │                                │
              │                                │
         ←────┼────→                      ←────┼────→ +X
         left │ right                          │
              │                                │
              
    If robot is facing 45° right, its "forward" is not 
    the same as the field's +Y. We must rotate.
```

The rotation equations:

```
// Update heading first
global_θ += Δθ

// Rotate forward/back movement into global frame
global_Y += local_ΔY × cos(global_θ)
global_X += local_ΔY × sin(global_θ)

// Rotate sideways movement into global frame
global_Y += local_ΔX × sin(global_θ)
global_X += local_ΔX × (-cos(global_θ))
```

### Visual: The Complete Process

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        ONE ODOMETRY UPDATE CYCLE                         │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌──────────────┐                                                        │
│  │ READ SENSORS │                                                        │
│  └──────┬───────┘                                                        │
│         │                                                                │
│         ▼                                                                │
│   dL = 0.05"   dR = 0.06"   dS = -0.01"                                 │
│   (left)       (right)      (back/strafe)                               │
│         │                                                                │
│         ▼                                                                │
│  ┌──────────────────────────────────────────────────────────────┐       │
│  │ CALCULATE ROTATION                                            │       │
│  │                                                               │       │
│  │   Δθ = (dR - dL) / (sL - sR)                                 │       │
│  │   Δθ = (0.06 - 0.05) / (-6.5 - 6.5)                          │       │
│  │   Δθ = 0.01 / -13 = -0.00077 rad ≈ -0.044°                   │       │
│  └──────────────────────────────────────────────────────────────┘       │
│         │                                                                │
│         ▼                                                                │
│  ┌──────────────────────────────────────────────────────────────┐       │
│  │ CALCULATE LOCAL DISPLACEMENT                                  │       │
│  │                                                               │       │
│  │   Since |Δθ| < 0.0001, use straight-line approximation:      │       │
│  │   local_ΔY = (dL + dR) / 2 = (0.05 + 0.06) / 2 = 0.055"     │       │
│  │   local_ΔX = dS = -0.01"                                     │       │
│  └──────────────────────────────────────────────────────────────┘       │
│         │                                                                │
│         ▼                                                                │
│  ┌──────────────────────────────────────────────────────────────┐       │
│  │ ROTATE TO GLOBAL FRAME                                        │       │
│  │                                                               │       │
│  │   Current heading: θ = 45° = 0.785 rad                       │       │
│  │                                                               │       │
│  │   global_X += local_ΔY × sin(θ) + local_ΔX × (-cos(θ))       │       │
│  │            += 0.055 × 0.707 + (-0.01) × (-0.707)             │       │
│  │            += 0.039 + 0.007 = 0.046"                         │       │
│  │                                                               │       │
│  │   global_Y += local_ΔY × cos(θ) + local_ΔX × sin(θ)          │       │
│  │            += 0.055 × 0.707 + (-0.01) × 0.707                │       │
│  │            += 0.039 - 0.007 = 0.032"                         │       │
│  │                                                               │       │
│  │   global_θ += Δθ = 45° + (-0.044°) = 44.956°                 │       │
│  └──────────────────────────────────────────────────────────────┘       │
│         │                                                                │
│         ▼                                                                │
│  ┌──────────────────────────────────────────────────────────────┐       │
│  │ UPDATED POSE                                                  │       │
│  │                                                               │       │
│  │   Previous: (12.000, 24.000, 45.000°)                        │       │
│  │   New:      (12.046, 24.032, 44.956°)                        │       │
│  └──────────────────────────────────────────────────────────────┘       │
│                                                                          │
│   This happens 100 times per second. Small updates accumulate            │
│   into accurate position tracking over time.                             │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## From Theory to Code

Now let's see how this math translates into actual C++ code.

### The Global State

In `odometry.cpp`, we maintain global variables that store the current state:

```cpp
// The robot's current pose (what we're tracking)
shulib::Pose odomPose(0, 0, 0);

// The robot's current velocity (for advanced features)
shulib::Pose odomSpeed(0, 0, 0);

// The sensors we're reading from
shulib::OdomSensors odomSensors(nullptr, nullptr, nullptr, nullptr);

// Correction factors for calibration
double xCorrectionFactor = 1.0;
double yCorrectionFactor = 1.0;
double thetaCorrectionFactor = 1.0;
```

### The Update Function

This is the core algorithm, running 100 times per second:

```cpp
void shulib::update() {
    // ═══════════════════════════════════════════════════════════════
    // STEP 1: GET WHEEL OFFSETS
    // ═══════════════════════════════════════════════════════════════
    // These are the distances from each tracking wheel to the robot's
    // center of rotation. They're set during configuration.
    
    float sL = odomSensors.left->get_offset();   // e.g., -6.5 inches
    float sR = odomSensors.right->get_offset();  // e.g., +6.5 inches
    float sS = odomSensors.back->get_offset();   // e.g., 0 inches

    // ═══════════════════════════════════════════════════════════════
    // STEP 2: READ SENSOR DELTAS
    // ═══════════════════════════════════════════════════════════════
    // get_travel_delta() returns how far each wheel has moved since
    // the LAST time we called it. This is typically a small value
    // like 0.01 to 0.1 inches.
    
    float dL = odomSensors.left->get_travel_delta();
    float dR = odomSensors.right->get_travel_delta();
    float dS = odomSensors.back->get_travel_delta();

    // ═══════════════════════════════════════════════════════════════
    // STEP 3: CALCULATE ROTATION
    // ═══════════════════════════════════════════════════════════════
    // The difference between left and right wheel travel tells us
    // how much the robot has rotated.
    
    Pose localPose(0, 0, 0);
    localPose.theta = (dR - dL) / (sL - sR) * thetaCorrectionFactor;

    // ═══════════════════════════════════════════════════════════════
    // STEP 4: UPDATE GLOBAL HEADING
    // ═══════════════════════════════════════════════════════════════
    // Add the rotation to our running total
    
    odomPose.theta += localPose.theta;

    // ═══════════════════════════════════════════════════════════════
    // STEP 5: CALCULATE LOCAL DISPLACEMENT
    // ═══════════════════════════════════════════════════════════════
    
    float deltaX = 0;
    float deltaY = 0;
    float rC = 0;  // radius of curvature

    if (abs(localPose.theta) < 0.0001) {
        // ─────────────────────────────────────────────────────────────
        // CASE A: Essentially went straight (negligible rotation)
        // ─────────────────────────────────────────────────────────────
        // When rotation is tiny, the arc approximation has numerical
        // issues (division by near-zero). Use simple average instead.
        
        deltaY = (dL + dR) / 2;  // Average of left and right
        deltaX = dS;              // Back wheel directly measures strafe
    } else {
        // ─────────────────────────────────────────────────────────────
        // CASE B: Turned while moving (arc motion)
        // ─────────────────────────────────────────────────────────────
        // Use the chord length formula to find actual displacement
        
        // For forward/backward movement:
        rC = (dR / localPose.theta) + sR;
        deltaY = 2 * sin(localPose.theta / 2) * rC;

        // For sideways movement:
        rC = (dS / localPose.theta) + sS;
        deltaX = 2 * sin(localPose.theta / 2) * rC;
    }

    // ═══════════════════════════════════════════════════════════════
    // STEP 6: CONVERT TO GLOBAL COORDINATES
    // ═══════════════════════════════════════════════════════════════
    // Rotate local displacement by current heading to get field coords
    
    // Forward/backward component
    odomPose.y += deltaY * cos(odomPose.theta) * yCorrectionFactor;
    odomPose.x += deltaY * sin(odomPose.theta) * yCorrectionFactor;

    // Sideways component  
    odomPose.y += deltaX * sin(odomPose.theta) * xCorrectionFactor;
    odomPose.x += deltaX * -cos(odomPose.theta) * xCorrectionFactor;
}
```

### Why the Threshold Check?

```cpp
if (abs(localPose.theta) < 0.0001) {
```

When Δθ is very close to zero, the formula `dR / Δθ` approaches infinity. This causes numerical instability. For tiny rotations, the straight-line approximation is more accurate anyway.

**Think of it this way:** If you turn 0.001°, you went basically straight. The "arc" is indistinguishable from a line.

---

## The Update Loop

### The Background Task

Odometry runs in a dedicated background task, independent of your main code:

```cpp
void shulib::init_odometry() {
    shulib::logger().log("Initializing odometry...");
    
    if (trackingTask == nullptr) {
        trackingTask = new pros::Task{[=] {
            // This lambda runs forever in the background
            while (true) {
                // Update position
                update();
                
                // Log telemetry (if position changed enough to matter)
                if (position_changed_significantly) {
                    shulib::logger().updateTelemetry("odometry", odomPose);
                }
                
                // Wait 10ms before next update (100 Hz)
                pros::delay(10);
            }
        }};
        shulib::logger().success("Odometry initialized!");
    }
}
```

### Why 100 Hz?

We update 100 times per second (every 10ms). Why this specific rate?

| Update Rate | Pros | Cons |
|-------------|------|------|
| 10 Hz (100ms) | Low CPU | Misses fast movements, choppy |
| 50 Hz (20ms) | Balanced | Might miss rapid changes |
| **100 Hz (10ms)** | **Catches fast motion** | **Good balance** |
| 200 Hz (5ms) | Very smooth | Higher CPU, diminishing returns |
| 1000 Hz (1ms) | Overkill | Wastes CPU, sensor can't keep up |

100 Hz is the sweet spot for VEX V5 - fast enough to catch any realistic robot motion, slow enough to leave CPU for other tasks.

### What Happens Each Cycle

```
Time: 0ms
├── Read sensors: L=100.00, R=100.00, B=0.00
├── Calculate: Δθ=0, ΔY=0, ΔX=0
└── Pose: (0.00, 0.00, 0.00°)

Time: 10ms
├── Read sensors: L=100.05, R=100.05, B=0.00
├── Delta: dL=0.05", dR=0.05", dS=0.00"
├── Calculate: Δθ=0, ΔY=0.05", ΔX=0
└── Pose: (0.00, 0.05, 0.00°)

Time: 20ms
├── Read sensors: L=100.11, R=100.09, B=0.01
├── Delta: dL=0.06", dR=0.04", dS=0.01"
├── Calculate: Δθ=-0.15°, ΔY=0.05", ΔX=0.01"
└── Pose: (0.01, 0.10, -0.15°)

... continues forever ...
```

---

## API Reference

### Getting Position

#### `shulib::getPose(bool radians = false)`

Returns the robot's current position as a Pose object.

```cpp
// Get position with heading in DEGREES (default)
shulib::Pose pos = shulib::getPose();
printf("At (%.1f, %.1f) facing %.1f°\n", pos.x, pos.y, pos.theta);

// Get position with heading in RADIANS
shulib::Pose posRad = shulib::getPose(true);
printf("Heading: %.3f radians\n", posRad.theta);
```

**Returns:** A `Pose` object with fields:
- `x`: X position in inches
- `y`: Y position in inches  
- `theta`: Heading in degrees (or radians if `radians=true`)

### Setting Position

#### `shulib::setPose(Pose pose, bool radians = false)`

Sets the robot's current position. Use this to establish your starting point or correct drift.

```cpp
// Set to origin, facing forward
shulib::setPose(shulib::Pose(0, 0, 0));

// Set to specific field position
shulib::setPose(shulib::Pose(24, 36, 90));  // (24", 36", facing right)

// Set using radians for heading
shulib::setPose(shulib::Pose(24, 36, M_PI/2), true);  // Same as above
```

**Parameters:**
- `pose`: The new pose to set
- `radians`: If `true`, interpret `pose.theta` as radians

**Common uses:**
```cpp
// At the start of autonomous
void autonomous() {
    chassis.setPose(0, 0, 0);  // We're at the origin now
    // ... rest of autonomous
}

// After aligning against a wall
void alignToWall() {
    // Robot is now against the wall, facing forward
    // Keep X, reset Y and heading
    Pose current = chassis.getPose();
    chassis.setPose(current.x, 0, 0);
}
```

### Velocity Functions

#### `shulib::getSpeed(bool radians = false)`

Returns current velocity in field coordinates.

```cpp
shulib::Pose vel = shulib::getSpeed();
printf("Velocity: (%.1f, %.1f) in/s, rotating %.1f°/s\n", 
       vel.x, vel.y, vel.theta);
```

#### `shulib::getLocalSpeed(bool radians = false)`

Returns current velocity in robot coordinates (forward/back, left/right).

```cpp
shulib::Pose localVel = shulib::getLocalSpeed();
// localVel.y = forward speed (positive = forward)
// localVel.x = strafe speed (positive = right)
// localVel.theta = rotation speed
```

#### `shulib::estimatePose(float time, bool radians = false)`

Predicts where the robot will be after `time` seconds, assuming constant velocity.

```cpp
// Where will we be in 0.2 seconds?
shulib::Pose future = shulib::estimatePose(0.2);

// Useful for "lead" targeting - aim where target WILL be
```

### Correction Factors

#### `shulib::setXCorrectionFactor(double factor)`
#### `shulib::setYCorrectionFactor(double factor)`
#### `shulib::setThetaCorrectionFactor(double factor)`

Adjust these if your odometry has consistent systematic errors.

```cpp
// If robot consistently reports 5% less distance in Y
shulib::setYCorrectionFactor(1.05);  // Multiply all Y by 1.05

// If robot consistently over-reports rotation by 2%
shulib::setThetaCorrectionFactor(0.98);  // Multiply all theta by 0.98
```

**When to use:**
- After verifying hardware is correct (no loose wheels, correct diameters)
- Only for consistent, repeatable errors
- Not a substitute for proper calibration!

#### `shulib::getThetaCorrectionFactor()`

Returns the current theta correction factor.

---

## Integration with the Chassis

Odometry doesn't exist in isolation - it's part of the Chassis system.

### Initialization Flow

```cpp
// In main.cpp:

// 1. Create sensor objects
pros::Rotation leftRotation(ROBOT.tracking.left_port);
pros::Rotation rightRotation(ROBOT.tracking.right_port);
pros::Rotation backRotation(ROBOT.tracking.back_port);

// 2. Wrap in OdomUnit objects (handles unit conversion)
shulib::OdomUnit leftOdom(&leftRotation, 
                          ROBOT.tracking.wheel_diameter, 
                          ROBOT.tracking.left_offset);
shulib::OdomUnit rightOdom(&rightRotation, 
                           ROBOT.tracking.wheel_diameter, 
                           ROBOT.tracking.right_offset);
shulib::OdomUnit backOdom(&backRotation, 
                          ROBOT.tracking.wheel_diameter, 
                          ROBOT.tracking.back_offset);

// 3. Bundle into OdomSensors
shulib::OdomSensors sensors(&leftOdom, &rightOdom, &backOdom, nullptr);

// 4. Create Chassis (connects drivetrain + sensors)
shulib::Chassis chassis(drivetrain, sensors);

// 5. Calibrate (resets sensors, starts odometry task)
chassis.calibrate(false);  // false = no IMU

// 6. Set starting position
chassis.setPose(0, 0, 0);
```

### Chassis Methods That Use Odometry

The Chassis class provides convenient wrappers:

```cpp
// These all use the odometry system internally:

chassis.getPose();           // Same as shulib::getPose()
chassis.setPose(0, 0, 0);    // Same as shulib::setPose()
chassis.resetLocalPosition(); // Keeps heading, resets X/Y to 0
```

### How Motion Functions Use Odometry

All autonomous motion depends on odometry feedback:

```cpp
void moveVertical(Chassis& chassis, double distance) {
    Pose start = chassis.getPose();  // Where are we now?
    
    while (true) {
        Pose current = chassis.getPose();  // Where are we now?
        
        // Calculate how far we've gone
        double dx = current.x - start.x;
        double dy = current.y - start.y;
        double traveled = sqrt(dx*dx + dy*dy);
        
        // Are we there yet?
        if (traveled >= distance) break;
        
        // Keep driving...
        chassis.drive(0, power, correction);
    }
}
```

Without odometry, we couldn't know how far we've traveled!

---

## Telemetry and Debugging

### Automatic Telemetry

The odometry system automatically sends position data to the logger:

```cpp
// In the update loop:
if (abs(odomPose.x - lastLoggedPose.x) > 0.1 ||
    abs(odomPose.y - lastLoggedPose.y) > 0.1 ||
    abs(odomPose.theta - lastLoggedPose.theta) > 0.1) {
    shulib::logger().updateTelemetry("odometry", odomPose);
    lastLoggedPose = odomPose;
}
```

This only logs when position changes by more than 0.1 units, avoiding spam.

### Viewing Telemetry

Connect to the robot and run:
```bash
pros terminal
```

You'll see JSON output like:
```json
{"odometry": {"x": 12.45, "y": 24.82, "theta": 0.789}}
```

### Brain Screen Display

If `ODOM_DISPLAY` is defined, position shows on the brain:

```cpp
#ifdef ODOM_DISPLAY
pros::Task screenTask([&]() {
    while (true) {
        pros::lcd::print(0, "%s", ROBOT.name.c_str());
        pros::lcd::print(1, "X: %.2f", chassis.getPose().x);
        pros::lcd::print(2, "Y: %.2f", chassis.getPose().y);
        pros::lcd::print(3, "Theta: %.2f", chassis.getPose().theta);
        pros::delay(50);
    }
});
#endif
```

### Debug Print Statements

For quick debugging:

```cpp
// Print once
Pose p = chassis.getPose();
printf("Position: (%.2f, %.2f, %.2f)\n", p.x, p.y, p.theta);

// Print in a loop
while (true) {
    Pose p = chassis.getPose();
    printf("X=%.1f Y=%.1f θ=%.1f\n", p.x, p.y, p.theta);
    pros::delay(100);  // Don't spam too fast
}
```

---

## Correction Factors

### What They Are

Correction factors are multipliers applied to odometry calculations to fix systematic errors:

```cpp
actual_value = measured_value × correction_factor
```

### When You Need Them

| Symptom | Possible Cause | Which Factor |
|---------|---------------|--------------|
| Robot goes 5% too far in Y | Wheel diameter slightly wrong | Y: 0.95 |
| Robot goes 3% too short in Y | Wheel slipping | Y: 1.03 |
| Turns are 2% too far | Track width slightly wrong | Theta: 0.98 |
| X movement reads wrong | Back wheel issue | X: varies |

### How to Calculate

1. **For Y (forward/back):**
   ```
   Command robot to go exactly 24"
   Measure actual distance traveled: 25.2"
   Odometry reported: 24"
   
   Correction = expected / actual = 24 / 25.2 = 0.952
   
   shulib::setYCorrectionFactor(0.952);
   ```

2. **For Theta (rotation):**
   ```
   Command robot to turn exactly 360°
   Measure actual rotation: 355°
   Odometry reported: 360°
   
   Correction = expected / reported = 360 / 360 = 1.0
   But actual was 355°, so reported was wrong.
   Correction = actual / target = 355 / 360 = 0.986
   
   Wait - we want to FIX it:
   If turning 360° actually turns 355°, and we report 360°,
   then our reported value is too HIGH.
   To fix: multiply by 355/360 = 0.986
   
   shulib::setThetaCorrectionFactor(0.986);
   ```

### Important Notes

⚠️ **Correction factors are a LAST RESORT.**

Before using them:
1. Verify wheel diameters are correct
2. Verify offsets are measured correctly  
3. Check for loose mounts or worn wheels
4. Ensure wheels maintain ground contact

Correction factors hide problems. Fix the root cause if possible.

---

## Testing Your Understanding

### Concept Check Questions

Try to answer these before looking at the answers:

1. **Why do we use three tracking wheels instead of two?**

2. **If the right wheel travels farther than the left, which way did the robot turn?**

3. **Why can't we use motor encoders for odometry?**

4. **What does a correction factor of 1.05 mean?**

5. **Why do we update position 100 times per second instead of 10?**

<details>
<summary>Click to reveal answers</summary>

1. **Three wheels** detect all 2D motion: forward/back (left+right wheels), rotation (difference between left and right), AND sideways drift (back wheel). Two wheels can't detect sideways motion.

2. **Right** - When turning right (clockwise), the right wheel is on the outside of the turn and travels a longer arc.

3. **Motor encoders** measure motor rotation, not ground truth. Drive wheels slip during acceleration, turning, and when pushed. Tracking wheels don't slip because they have no torque.

4. **1.05 correction** means multiply all values by 1.05 - the system was reading 5% low, so we scale up to compensate.

5. **100 Hz** catches fast movements that 10 Hz would miss. If the robot moves quickly, a lot can happen in 100ms. 10ms updates ensure smooth, accurate tracking.

</details>

### Mental Simulation Exercise

Work through this by hand:

```
Initial pose: (0, 0, 0°) - at origin, facing +Y

Update 1:
- dL = 0.10"
- dR = 0.10"  
- dS = 0.00"
- Offsets: sL = -6.5", sR = +6.5", sS = 0"

Calculate new pose:
- Δθ = ?
- local_ΔY = ?
- local_ΔX = ?
- New (x, y, θ) = ?
```

<details>
<summary>Click to reveal answer</summary>

```
Δθ = (dR - dL) / (sL - sR)
   = (0.10 - 0.10) / (-6.5 - 6.5)
   = 0 / -13
   = 0  (no rotation)

Since Δθ ≈ 0:
  local_ΔY = (dL + dR) / 2 = (0.10 + 0.10) / 2 = 0.10"
  local_ΔX = dS = 0.00"

Convert to global (θ = 0°, so sin(0)=0, cos(0)=1):
  global_Y += 0.10 × cos(0°) = 0.10 × 1 = 0.10"
  global_X += 0.10 × sin(0°) = 0.10 × 0 = 0.00"

New pose: (0.00, 0.10, 0°)

Robot moved 0.10" forward (in +Y), no rotation.
```

</details>

---

## Historical Context

### The Evolution of VEX Odometry

**Early Days (Pre-2015):**
- Teams used motor encoders directly
- Time-based autonomous routines
- Very inconsistent results

**The Pilons Document (2018):**
- Team 5225A published their tracking document
- First widely-shared mathematical framework for VEX odometry
- Became the gold standard

**LemLib Era (2020+):**
- Open-source library implementing advanced odometry
- Made proper odometry accessible to all teams
- Inspired our shulib implementation

**Today:**
- Three-wheel odometry is standard for competitive teams
- Some teams adding vision systems for absolute positioning
- Pure odometry still highly effective

### Why We Built shulib

LemLib is excellent, but we wanted:
- Code we fully understand
- Easier customization for our specific needs
- Learning experience for team members
- Documentation tailored to our robots

Our odometry is heavily inspired by LemLib and the Pilons document.

---

## Key Takeaways

### The Essential Points

1. **Odometry = tracking position using motion measurements**
   - We measure wheel rotations
   - Convert to distances
   - Accumulate into (x, y, θ) position

2. **Three wheels detect all 2D motion**
   - Left + Right: forward/back and rotation
   - Back: sideways drift
   - Unpowered wheels for accuracy

3. **The math has two cases**
   - Straight motion: simple averaging
   - Arc motion: chord length formula

4. **Updates happen constantly**
   - 100 Hz (every 10ms)
   - Small changes accumulate
   - Background task, always running

5. **Calibration matters**
   - Measure wheel diameters precisely
   - Measure offsets precisely
   - Use correction factors sparingly

### The One-Sentence Summary

> **Odometry measures how much each tracking wheel rotates, uses geometry to calculate how that translates to position changes, and accumulates those changes to know where we are at all times.**

---

## Where to Go Next

Now that you understand odometry conceptually, explore these related topics:

| Topic | Document | What You'll Learn |
|-------|----------|-------------------|
| Tracking wheel details | [ODOM_UNIT.md](./ODOM_UNIT.md) | The OdomUnit class, sensor conversion |
| The Pose class | [POSE.md](./POSE.md) | Pose operations and math |
| Hardware setup | [TRACKING_WHEELS.md](../configuration/TRACKING_WHEELS.md) | Physical installation and wiring |
| Coordinate details | [COORDINATE_SYSTEM.md](../reference/COORDINATE_SYSTEM.md) | Full coordinate conventions |
| When things go wrong | [ODOMETRY_DRIFT.md](../troubleshooting/ODOMETRY_DRIFT.md) | Debugging drift issues |
| The Chassis class | [CHASSIS.md](./CHASSIS.md) | How Chassis uses odometry |

### External Resources

- 📄 **[5225A Pilons Tracking Document](http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf)** - The original mathematical reference
- 📚 **[LemLib Documentation](https://lemlib.github.io/LemLib/)** - The library that inspired us
- 🎥 **Search "VEX odometry" on YouTube** - Visual explanations and implementations

---

*Document last updated: January 2026*
*For the next generation of SHU Robotics - good luck at competition! 🤖*