# Odometry Drift

**When position tracking goes wrong**

---

## Symptoms

You might have odometry drift if:

- ⚠️ Robot thinks it's somewhere it's not
- ⚠️ Autonomous works once then fails on repeat
- ⚠️ Position error accumulates over time
- ⚠️ Straight driving shows X drift
- ⚠️ Turns show position drift
- ⚠️ Return-to-start doesn't return to start

---

## Types of Drift

### Linear Drift

Position drifts during straight-line driving:

```
Intended:           Actual:
    ↑                  ╱
    │                 ╱
    │                ╱
    │               ╱
    ●              ●
  Start          Start

Odometry shows straight, but robot curves
OR robot goes straight but odometry shows curve
```

### Rotational Drift

Heading drifts during turns:

```
After 360° turn:
                    
Expected:  ●→       Actual: ●↗
          Back at             Off by 5-10°
          0°                  
```

### Accumulated Drift

Error builds up over time:

```
Position error over time:

Error
  │
  │                    ╱
  │                 ╱╱
  │              ╱╱
  │          ╱╱╱
  │      ╱╱╱
  │  ╱╱╱
  └─────────────────────── Time
     Small errors compound!
```

---

## Common Causes

### 1. Wrong Wheel Diameter

**The #1 cause of linear drift.**

If configured diameter ≠ actual diameter:
- Too large → Robot thinks it went farther than it did
- Too small → Robot thinks it went shorter than it did

```cpp
// Config says 2.75"
.wheel_diameter = 2.75

// But actual wheel is worn to 2.70"
// Result: 2% error on all distances!
```

### 2. Wrong Tracking Wheel Offsets

**The #1 cause of rotational drift.**

If offsets don't match physical mounting:
- Turns calculate wrong
- Heading drifts

```cpp
// Config says 6.5" from center
.left_offset = -6.5

// But actual measurement is 6.25"
// Result: Heading error on every turn!
```

### 3. Wheel Slippage

Tracking wheel slips instead of rolling:
- Debris on wheel or floor
- Wheel not making good contact
- Wheel bouncing

### 4. Mechanical Issues

- Loose tracking wheel mounting
- Bent axle
- Worn bearings
- Wheel not perpendicular to ground

### 5. Sensor Issues

- Loose cable connection
- Intermittent sensor readings
- Wrong sensor port in config

### 6. Wrong Sensor Direction

Sensor reads negative when should be positive:

```cpp
// Robot moves forward, sensor reads negative
// Odometry thinks robot is going backward!
```

---

## Diagnosis

### Test 1: Push Test

Push robot by hand, watch odometry:

```cpp
void pushTest() {
    chassis.setPose(0, 0, 0);
    
    while (true) {
        Pose p = chassis.getPose();
        printf("X: %.2f  Y: %.2f  θ: %.2f\n", p.x, p.y, p.theta);
        pros::delay(100);
    }
}
```

1. Push robot forward → Y should increase
2. Push robot right → X should increase
3. Turn robot clockwise → θ should increase

If any are backwards, fix sensor port signs.

### Test 2: Straight Line Test

```cpp
void straightLineTest() {
    chassis.setPose(0, 0, 0);
    
    // Drive exactly 24" (measure with tape!)
    moveVertical(chassis, 24);
    
    Pose end = chassis.getPose();
    printf("Expected: Y=24.0\n");
    printf("Actual:   Y=%.2f\n", end.y);
    printf("Error:    %.2f (%.1f%%)\n", 
           24.0 - end.y, 
           (24.0 - end.y) / 24.0 * 100);
}
```

Run multiple times. Consistent error = wheel diameter wrong.

### Test 3: Square Test

```cpp
void squareTest() {
    chassis.setPose(0, 0, 0);
    
    // Drive a 24" square
    for (int i = 0; i < 4; i++) {
        moveVertical(chassis, 24);
        rotateTo(chassis, (i + 1) * 90);
    }
    
    Pose end = chassis.getPose();
    printf("Should be back at (0, 0, 0)\n");
    printf("Actual: (%.2f, %.2f, %.2f)\n", end.x, end.y, end.theta);
}
```

This tests both linear and rotational accuracy.

### Test 4: Spin Test

```cpp
void spinTest() {
    chassis.setPose(0, 0, 0);
    
    // Spin 10 full rotations
    for (int i = 0; i < 10; i++) {
        rotateTo(chassis, (i + 1) * 360);
    }
    
    // Should be at 3600° (or 0° wrapped)
    Pose end = chassis.getPose();
    printf("Heading after 10 spins: %.2f\n", end.theta);
    printf("Position drift: (%.2f, %.2f)\n", end.x, end.y);
}
```

Position should stay near (0, 0). If it drifts, offsets are wrong.

---

## Fixes

### Calibrating Wheel Diameter

**Procedure:**

1. Mark a starting line on the floor
2. Mark a line exactly 48" away (use tape measure!)
3. Reset odometry
4. Push robot from start to end line (by hand, carefully)
5. Read Y value from odometry
6. Calculate correction factor

```cpp
// If odometry shows 50" but you pushed 48":
correction = actual_distance / odometry_distance
correction = 48.0 / 50.0 = 0.96

// New diameter:
new_diameter = old_diameter * correction
new_diameter = 2.75 * 0.96 = 2.64
```

Update config and repeat until accurate.

### Calibrating Offsets

**For left/right offsets:**

1. Mark robot's center position
2. Spin robot 360° in place
3. Robot should end at same position
4. If it drifts forward/backward, offsets are wrong

```cpp
// If robot drifts FORWARD after spinning:
// Tracking wheels are CLOSER to center than configured
// DECREASE offset values (make closer to 0)

// If robot drifts BACKWARD:
// Tracking wheels are FARTHER from center
// INCREASE offset values (farther from 0)
```

**For back offset:**

1. Spin robot 360°
2. If heading drifts, back offset is wrong
3. Adjust and repeat

### Fixing Wheel Slippage

**Clean the wheel:**
```
- Remove debris
- Clean with rubbing alcohol
- Check for flat spots
```

**Improve contact:**
```
- Add spring tension
- Check wheel alignment
- Ensure wheel is perpendicular to floor
```

**Check mounting:**
```
- Tighten all screws
- Check for play in bearings
- Verify axle isn't bent
```

### Fixing Sensor Direction

If sensor reads backwards:

```cpp
// Before: sensor reads negative going forward
.left_port = 8

// After: reversed
.left_port = -8
```

---

## Measurement Tips

### Measuring Wheel Diameter

1. Remove wheel from robot
2. Use calipers if available
3. Measure at multiple points (wheels wear unevenly)
4. Measure INCLUDING any rubber coating
5. Use average of measurements

**Don't trust nominal size!**
- "2.75 inch wheel" might actually be 2.73" or 2.78"
- Worn wheels are smaller

### Measuring Offsets

1. Find the robot's center of rotation
   - Usually the center of the drivetrain
   - Where the robot pivots when spinning
   
2. Measure perpendicular distance from center to wheel contact point

3. Be precise - 0.25" error causes significant drift!

```
           Center of
           Rotation
              │
    ◄─────────┼─────────►
    │         │         │
    L         │         R
   wheel      │       wheel
              │
    ◄───6.5"──┼──6.5"───►
              │
         left_offset = -6.5
        right_offset = +6.5
```

---

## Prevention

### Regular Maintenance

- [ ] Check tracking wheel tension weekly
- [ ] Clean wheels before each competition
- [ ] Inspect for wear monthly
- [ ] Re-verify calibration after any mechanical changes

### Pre-Match Checks

- [ ] Push test - directions correct?
- [ ] Quick straight line - distance correct?
- [ ] Quick spin - heading holds?

### During Development

- [ ] Re-calibrate after changing wheels
- [ ] Re-calibrate after changing mounting
- [ ] Log odometry during testing
- [ ] Compare expected vs actual regularly

---

## Calibration Values

Keep a record of calibrated values:

```
╔═══════════════════════════════════════════════════════╗
║               XEBEC CALIBRATION LOG                   ║
╠═══════════════════════════════════════════════════════╣
║ Date       │ Parameter      │ Value    │ Notes        ║
╠────────────┼────────────────┼──────────┼──────────────╣
║ 2026-01-15 │ wheel_diameter │ 2.73     │ Measured     ║
║ 2026-01-15 │ left_offset    │ -6.45    │ Calibrated   ║
║ 2026-01-15 │ right_offset   │ 6.52     │ Calibrated   ║
║ 2026-01-15 │ back_offset    │ 0.0      │ Centered     ║
║ 2026-01-20 │ wheel_diameter │ 2.71     │ After wear   ║
╚═══════════════════════════════════════════════════════╝
```

---

## Summary

| Problem | Test | Fix |
|---------|------|-----|
| Linear drift | Straight line test | Calibrate wheel diameter |
| Rotational drift | Spin test | Calibrate offsets |
| Random jumps | Push test | Check connections |
| Direction wrong | Push test | Fix port signs |
| Accumulated error | Square test | All of the above |

---

*For odometry theory, see [ODOMETRY.md](../core-library/ODOMETRY.md)*
*For tracking wheel setup, see [TRACKING_WHEELS.md](../configuration/TRACKING_WHEELS.md)*
*For other issues, see [COMMON_ISSUES.md](./COMMON_ISSUES.md)*