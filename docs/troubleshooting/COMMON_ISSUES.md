# Common Issues

**Quick reference for frequent problems**

---

## Quick Diagnosis

### "It won't build"
→ See [BUILD_ERRORS.md](./BUILD_ERRORS.md)

### "Motors are fighting each other"
→ See [MOTORS_FIGHTING.md](./MOTORS_FIGHTING.md)

### "Robot position drifts over time"
→ See [ODOMETRY_DRIFT.md](./ODOMETRY_DRIFT.md)

### "Robot oscillates / won't settle"
→ See [PID_OSCILLATION.md](./PID_OSCILLATION.md)

---

## Issue Index

### Build & Code Issues

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| `fatal error: file not found` | Wrong include path | [BUILD_ERRORS.md](./BUILD_ERRORS.md#include-not-found) |
| `undefined reference` | Missing implementation | [BUILD_ERRORS.md](./BUILD_ERRORS.md#undefined-reference) |
| `No robot selected` | Config not set | Uncomment robot in `config.hpp` |
| Code uploads but nothing happens | Wrong `main.cpp` | Check entry points are correct |

### Motor Issues

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| Robot spins instead of driving straight | One side reversed | [MOTORS_FIGHTING.md](./MOTORS_FIGHTING.md) |
| Motors stall/overheat quickly | Fighting each other | [MOTORS_FIGHTING.md](./MOTORS_FIGHTING.md) |
| Robot drives backward when commanded forward | All motors reversed | Flip all port signs |
| One motor doesn't spin | Bad port or cable | Check connections, try different port |
| Motor makes noise but doesn't move | Mechanical jam | Check for obstructions |

### Odometry Issues

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| Position drifts during straight driving | Wheel diameter wrong | [ODOMETRY_DRIFT.md](./ODOMETRY_DRIFT.md#calibrating-wheel-diameter) |
| Position drifts during turns | Offsets wrong | [ODOMETRY_DRIFT.md](./ODOMETRY_DRIFT.md#calibrating-offsets) |
| Position jumps randomly | Loose connection | Check tracking wheel wiring |
| Position doesn't change | Sensor not reading | Verify sensor port and direction |
| Heading drifts over time | Back wheel offset wrong | Recalibrate back offset |

### Motion Issues

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| Robot oscillates at target | PID gains too high | [PID_OSCILLATION.md](./PID_OSCILLATION.md) |
| Robot stops short | Gains too low or tolerance too high | Increase kP or reduce tolerance |
| Robot overshoots once then settles | kD too low | Increase kD |
| Robot moves very slowly | kP too low | Increase kP |
| Robot jerks/vibrates | kD too high | Reduce kD |
| Motion times out | Distance too far or robot stuck | Check odometry, reduce distance |

### Controller Issues

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| No response to controller | Controller not paired | Re-pair controller |
| Buttons don't work | Wrong button constant | Verify `DIGITAL_XX` names |
| Stick drift | Controller needs deadzone | Add deadzone code |
| Toggle doesn't work | Using `get_digital` instead of `get_digital_new_press` | Use `new_press` for toggles |

### Pneumatic Issues

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| Pneumatic doesn't fire | No air pressure | Pump up air tank |
| Pneumatic fires but weak | Low pressure | Add more air |
| Pneumatic stuck | Mechanical jam | Check piston physically |
| Toggle state wrong | State variable not tracked | Check toggle logic |

---

## First Steps for Any Issue

### 1. Check the Obvious

- [ ] Is it plugged in?
- [ ] Is the battery charged?
- [ ] Is the code uploaded?
- [ ] Is the right robot selected in config?

### 2. Isolate the Problem

- [ ] Does the issue happen every time?
- [ ] Does it happen with one motor/sensor or all?
- [ ] Did it work before? What changed?

### 3. Check Logs

```bash
pros terminal
```

Look for:
- Error messages
- Unexpected values
- Warnings

### 4. Test Components Individually

```cpp
// Test single motor
pros::Motor test(1);
test.move(50);

// Test single sensor
pros::Rotation sensor(8);
printf("Position: %d\n", sensor.get_position());
```

---

## Emergency Fixes

### "Competition is in 5 minutes and nothing works"

**Option 1: Hard reset**
```bash
pros make clean
pros make
pros upload
```

**Option 2: Fallback to known-good code**
```bash
git stash          # Save current changes
git checkout main  # Go to stable branch
pros make && pros upload
```

**Option 3: Disable the broken part**
```cpp
// Comment out problematic code
// moveToPose(chassis, target);  // BROKEN
moveVertical(chassis, 24);  // Simple fallback
```

**Option 4: Simple autonomous**
```cpp
void autonomous() {
    // Just do something simple that works
    chassis.drive(0, 50, 0);
    pros::delay(2000);
    chassis.drive(0, 0, 0);
}
```

---

## When to Ask for Help

### Before asking:
1. Read the error message carefully
2. Check this troubleshooting guide
3. Search the error message online
4. Try the suggested fixes

### When asking, include:
1. Exact error message or symptom
2. What you expected to happen
3. What actually happened
4. What you've already tried
5. Relevant code snippets

### Good places to ask:
- Team leads
- VEX Forum (vexforum.com)
- PROS Discord

---

## Preventing Issues

### Before Competition

- [ ] Test all motors individually
- [ ] Test all sensors
- [ ] Run complete autonomous
- [ ] Test all driver controls
- [ ] Charge all batteries
- [ ] Check all connections
- [ ] Have backup code ready

### During Development

- [ ] Commit working code frequently
- [ ] Test after each change
- [ ] Document what you change
- [ ] Don't change multiple things at once

---

*Detailed guides:*
- [BUILD_ERRORS.md](./BUILD_ERRORS.md) - Compilation problems
- [MOTORS_FIGHTING.md](./MOTORS_FIGHTING.md) - Motor direction issues
- [ODOMETRY_DRIFT.md](./ODOMETRY_DRIFT.md) - Position tracking problems
- [PID_OSCILLATION.md](./PID_OSCILLATION.md) - Motion tuning issues