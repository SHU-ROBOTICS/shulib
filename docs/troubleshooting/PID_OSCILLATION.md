# PID Oscillation

**When the robot can't settle on target**

---

## Symptoms

You might have PID oscillation if:

- ⚠️ Robot overshoots target repeatedly
- ⚠️ Robot rocks back and forth at target
- ⚠️ Robot vibrates or jitters
- ⚠️ Robot never stops moving
- ⚠️ Movement takes much longer than expected
- ⚠️ Robot "hunts" around the target position

---

## Types of Oscillation

### Overshoot Oscillation

Robot passes target, comes back, passes again:

```
Position over time:
              
    Target ─ ─ ─┬─────┬─────┬─────┬─────
                │     │     │     │
              ╱ │ ╲ ╱ │ ╲ ╱ │ ╲ ╱ │
             ╱  │  ╳  │  ╳  │  ╳  │
            ╱   │ ╱ ╲ │ ╱ ╲ │ ╱ ╲ │
           ╱    │╱   ╲│╱   ╲│╱   ╲│
    ──────╱─────┴─────┴─────┴─────┴───── Time
    
    Robot crosses target repeatedly
```

**Cause:** P too high, D too low

### High-Frequency Jitter

Rapid small vibrations:

```
Position over time:

    Target ─────∼∼∼∼∼∼∼∼∼∼∼∼∼∼∼∼∼∼∼∼∼─────
              ╱                         
             ╱  Tiny rapid oscillations
            ╱   at high frequency
    ───────╱──────────────────────────── Time
```

**Cause:** D too high

### Integral Windup

Slowly increasing oscillation:

```
Position over time:

    Target ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─
                              ╱╲
                            ╱    ╲
                          ╱        ╲
                       ╱            ╲
                    ╱╲                ╲
                  ╱    ╲                ╲
               ╱╱        ╲╲              ╲
    ──────────╱────────────╲───────────── Time
    
    Oscillations grow larger over time!
```

**Cause:** I too high, or I accumulated while stuck

### Sluggish + Overshoot

Slow approach, then overshoot:

```
Position over time:

    Target ─ ─ ─ ─ ─ ─ ─ ─ ─ ─┬─────┬───
                              │     │
                             ╱│╲   ╱│
                           ╱╱ │ ╲ ╱ │
                        ╱╱╱   │  ╳  │
                    ╱╱╱╱      │ ╱ ╲ │
    ───────────────╱──────────┴─────┴─── Time
    
    Takes forever, then overshoots anyway
```

**Cause:** P too low, then overcorrects; or I building up

---

## Diagnosing the Cause

### Step 1: Identify the Pattern

| Pattern | Most Likely Cause |
|---------|-------------------|
| Large overshoot, damped oscillations | P too high |
| Large overshoot, sustained oscillations | P too high, D too low |
| Small rapid jitter | D too high |
| Growing oscillations | I too high |
| Slow then overshoot | I accumulating |

### Step 2: Test with P Only

Temporarily remove I and D:

```cpp
PID testPID(currentP, 0, 0);  // I and D = 0
```

Run the motion:
- If it still oscillates: **P is too high**
- If it's sluggish but stable: **P is okay, D/I were the problem**

### Step 3: Log the Values

Add logging to see what PID is doing:

```cpp
float error = target - current;
float output = pid.update(error, dt);

printf("err=%.2f out=%.2f\n", error, output);
```

Look for:
- Output jumping around wildly → P or D too high
- Output growing over time → I accumulating
- Output never settling → Tolerance too tight

---

## Fixes by Symptom

### Fix: Large Overshoot

**The problem:** P too high, robot builds too much momentum

**The fix:**
1. Reduce P by 25-50%
2. Increase D
3. Reduce max output

```cpp
// Before
PID pid(10, 0, 0.5);

// After
PID pid(7, 0, 1.5);  // Lower P, higher D
```

### Fix: Sustained Oscillation

**The problem:** P drives past target, not enough D to stop

**The fix:**
1. Increase D significantly
2. Maybe reduce P slightly

```cpp
// Before
PID pid(8, 0, 0.2);

// After
PID pid(7, 0, 1.0);  // More D
```

### Fix: High-Frequency Jitter

**The problem:** D reacting to noise or minor changes

**The fix:**
1. Reduce D
2. Consider filtering sensor input

```cpp
// Before
PID pid(5, 0, 3.0);

// After
PID pid(5, 0, 1.0);  // Less D
```

### Fix: Growing Oscillations (Integral Windup)

**The problem:** I accumulated too much while stuck or overshooting

**The fix:**
1. Reduce I
2. Add integral clamping
3. Reset I on overshoot

```cpp
// Before
PID pid(5, 2.0, 0.5);

// After
PID pid(5, 0.5, 0.5);  // Much less I
```

Better solution - clamp integral:

```cpp
// In PID class
if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
if (integral < -MAX_INTEGRAL) integral = -MAX_INTEGRAL;
```

### Fix: Slow Then Overshoot

**The problem:** I builds up during slow approach, then overshoots

**The fix:**
1. Increase P (faster initial response)
2. Reduce I
3. Add D for braking

```cpp
// Before
PID pid(3, 1.5, 0);

// After
PID pid(6, 0.5, 1.0);  // Higher P, lower I, add D
```

---

## Tuning Procedure

### Step 1: Start Fresh

```cpp
PID pid(1, 0, 0);  // Very low P, no I or D
```

### Step 2: Increase P Until Responsive

- Double P each time
- Stop when you see any overshoot
- Back off slightly

```cpp
PID pid(1, 0, 0);   // Too slow
PID pid(2, 0, 0);   // Still slow
PID pid(4, 0, 0);   // Better
PID pid(8, 0, 0);   // Overshoots! 
PID pid(6, 0, 0);   // ← Use this P
```

### Step 3: Add D to Eliminate Overshoot

- Start with D = P / 10
- Increase until overshoot stops
- Stop if you see jitter

```cpp
PID pid(6, 0, 0.6);  // Start
PID pid(6, 0, 1.0);  // Less overshoot
PID pid(6, 0, 1.5);  // No overshoot ← Good!
```

### Step 4: Add I Only If Needed

Only if robot stops short of target:
- Start very small (0.01 - 0.1)
- Increase slowly

```cpp
PID pid(6, 0.1, 1.5);  // Try small I
```

If it causes problems, remove I and increase P instead.

---

## Quick Fixes

### "It's oscillating and I don't have time to tune"

**Nuclear option:** Reduce P dramatically

```cpp
// Before
PID pid(10, 0.5, 0.3);

// Quick fix
PID pid(4, 0, 0);  // Just P, very low
```

It'll be slow but stable.

### "It overshoots once then settles"

That's usually acceptable! But if you want to fix it:

```cpp
// Increase D
pid.kD *= 1.5;
```

### "It jitters at the target"

```cpp
// Reduce D and/or increase tolerance
pid.kD *= 0.5;
TOLERANCE = 2.0;  // Was 1.0
```

### "It never quite reaches target"

```cpp
// Increase P or add small I
pid.kP *= 1.2;
// OR
pid.kI = 0.1;
```

---

## PID Values Reference

### Our Working Values

**moveVertical (linear motion):**
```cpp
PID linearPID(10, 2.5, 0.3);
// Relatively high P - needs to move mass
// Some I - helps reach exact target
// Low D - not super sensitive
```

**rotateTo (rotation):**
```cpp
PID rotationPID(1.5, 0.01, 0.15);
// Lower P - rotation is more sensitive
// Very low I - almost none
// Moderate D relative to P
```

**moveToPose heading correction:**
```cpp
PID headingPID(10, 0.005, 0.25);
// Moderate P
// Very low I
// Some D for stability
```

### General Guidelines

| Motion Type | P Range | I Range | D Range |
|-------------|---------|---------|---------|
| Linear (heavy) | 5-15 | 0-3 | 0-2 |
| Linear (light) | 3-10 | 0-2 | 0-1 |
| Rotation | 1-3 | 0-0.1 | 0.1-0.5 |
| Fine positioning | 2-8 | 0-1 | 0.5-2 |

---

## Oscillation vs Other Problems

**It's NOT a PID problem if:**

| Symptom | Actual Problem |
|---------|----------------|
| Always goes to wrong position | Odometry issue |
| Drifts sideways | Motor direction issue |
| One side faster than other | Motor config issue |
| Works once, fails next time | Inconsistent starting pose |
| Random behavior | Sensor connection issue |

**It IS a PID problem if:**

- Robot reaches approximately right area but can't settle
- Behavior is consistent and repeatable
- Reducing P makes it calmer (just slower)
- Pattern matches oscillation types above

---

## Advanced: Manual Analysis

### Plotting Response

Log data and plot it:

```cpp
void loggedMove(float target) {
    PID pid(10, 0, 1);
    float current = 0;
    
    printf("time,target,current,error,output\n");
    
    for (int t = 0; t < 200; t++) {
        current = chassis.getPose().y;
        float error = target - current;
        float output = pid.update(error, 0.01);
        
        printf("%d,%.2f,%.2f,%.2f,%.2f\n", 
               t * 10, target, current, error, output);
        
        chassis.drive(0, output, 0);
        pros::delay(10);
    }
}
```

Copy output to spreadsheet, create chart. Look for:
- Overshoot: Current exceeds target
- Oscillation: Multiple zero crossings of error
- Settling time: How long until stable

---

## Summary

| Problem | Primary Fix | Secondary Fix |
|---------|-------------|---------------|
| Large overshoot | ↓ P | ↑ D |
| Sustained oscillation | ↑ D | ↓ P |
| High-frequency jitter | ↓ D | Filter input |
| Growing oscillations | ↓ I | Add I clamping |
| Slow + overshoot | ↑ P, ↓ I | Add D |
| Won't reach target | ↑ P | Add small I |

**The golden rule:** When in doubt, reduce gains. A slow stable system is better than a fast unstable one.

---

*For PID tuning procedure, see [PID_TUNING.md](../motion/PID_TUNING.md)*
*For PID theory, see [PID.md](../core-library/PID.md)*
*For other issues, see [COMMON_ISSUES.md](./COMMON_ISSUES.md)*