# Motors Fighting

**When motors work against each other**

---

## Symptoms

You might have motors fighting if:

- ⚠️ Motors get hot very quickly (within seconds)
- ⚠️ Motors stall under no load
- ⚠️ Robot barely moves despite full power
- ⚠️ You hear grinding or straining sounds
- ⚠️ Current draw is very high
- ⚠️ Robot spins in place when trying to drive straight

---

## What's Happening

Motors "fight" when they're trying to spin in opposite directions while mechanically connected:

```
NORMAL:                          FIGHTING:
                                 
Motor A → → →                    Motor A → → →
           ↓                                ↓
        [Wheel]                          [Wheel]
           ↓                                ↑
Motor B → → →                    Motor B ← ← ←

Both push same direction         They work AGAINST each other!
= Wheel spins freely             = Wheel can't move, motors strain
```

---

## Common Causes

### 1. Wrong Port Sign in Config

The most common cause. One motor's direction is wrong:

```cpp
// WRONG: Motor 14 should be reversed but isn't
.left_ports = {12, 14, 16}  

// RIGHT: Motor 14 is reversed (negative)
.left_ports = {12, -14, 16}
```

### 2. Wrong Port Number

Motor is on a different port than the code thinks:

```cpp
// Code says port 12
.left_ports = {12, ...}

// But motor is actually plugged into port 11!
```

### 3. Mechanical Reversal

Motor is physically mounted backward but code doesn't account for it:

```
Motor facing IN:     Motor facing OUT:
                     
  ┌─────┐              ┌─────┐
  │ →── │ Shaft       │ ──← │ Shaft
  └─────┘              └─────┘
  
Spins one way        Spins opposite way!
```

### 4. Mixed Motor Groups

Accidentally putting a right-side motor in the left-side group:

```cpp
// WRONG: Port 11 is a RIGHT motor!
.left_ports = {12, 14, 11, 18, 20}
                      ↑
                   WRONG!

// RIGHT: Port 16 belongs on left
.left_ports = {12, 14, 16, 18, 20}
```

---

## How to Diagnose

### Step 1: Identify Which Motors

Disconnect motors one at a time. When the problem goes away, you found the fighting pair.

### Step 2: Test Each Motor Individually

```cpp
void testMotor(int port) {
    pros::Motor motor(port);  // No reversal
    
    printf("Testing port %d - should spin FORWARD\n", port);
    motor.move(50);
    pros::delay(2000);
    motor.move(0);
    
    printf("Testing port %d - should spin BACKWARD\n", port);
    motor.move(-50);
    pros::delay(2000);
    motor.move(0);
}

// Test each motor:
testMotor(12);
testMotor(14);
testMotor(16);
// etc.
```

### Step 3: Check Direction Physically

For each motor:
1. Run it at +50 power
2. Watch which way the wheel spins
3. Is that the direction you want?

**For left side:** Wheels should spin to move robot FORWARD
**For right side:** Wheels should spin to move robot FORWARD

### Step 4: Create Direction Map

| Port | When Positive (+) | Should Be | Config |
|------|-------------------|-----------|--------|
| 12 | Spins forward | Forward | `12` (normal) |
| 14 | Spins backward | Forward | `-14` (reversed) |
| 16 | Spins forward | Forward | `16` (normal) |

---

## How to Fix

### Fix 1: Correct the Port Sign

If motor spins wrong way, add or remove the negative sign:

```cpp
// Before: Motor 14 spins wrong way
.left_ports = {12, 14, 16}

// After: Reverse motor 14
.left_ports = {12, -14, 16}
```

### Fix 2: Fix Port Assignment

If motor is on wrong port:

```cpp
// Before: Wrong port number
.left_ports = {12, 13, 16}  // 13 is wrong

// After: Correct port
.left_ports = {12, 14, 16}  // 14 is correct
```

### Fix 3: Verify All Ports

Double-check every port against physical wiring:

```
Physical Robot:          Code Config:
─────────────────        ─────────────
Left motor 1: Port 12    .left_ports = {12, ...} ✓
Left motor 2: Port 14    .left_ports = {..., -14, ...} ✓
...                      ...
```

---

## Prevention

### Label Your Wires

Put labels on motor cables:
- "L1" = Left motor 1
- "R3" = Right motor 3

### Document Motor Mounting

Note which motors face which way:

```
XEBEC Motor Mounting:
┌─────────────────────────────────────────┐
│ Position      │ Port │ Faces   │ Config │
├───────────────┼──────┼─────────┼────────┤
│ Left front    │ 12   │ Inward  │ 12     │
│ Left mid-f    │ 14   │ Outward │ -14    │
│ Left center   │ 16   │ Inward  │ 16     │
│ Left mid-b    │ 18   │ Outward │ -18    │
│ Left back     │ 20   │ Inward  │ 20     │
└───────────────┴──────┴─────────┴────────┘
```

### Test After Any Change

After ANY wiring change:
1. Run individual motor test
2. Verify no fighting
3. Then test together

---

## Quick Test Routine

Run this to verify all drive motors:

```cpp
void verifyDriveMotors() {
    printf("=== DRIVE MOTOR VERIFICATION ===\n");
    
    // Test left side
    printf("\nLEFT SIDE - All should move robot FORWARD:\n");
    for (int port : {12, -14, 16, -18, 20}) {
        int absPort = abs(port);
        pros::Motor motor(port);
        
        printf("Port %d: ", absPort);
        motor.move(40);
        pros::delay(500);
        motor.move(0);
        printf("Check wheel direction!\n");
        pros::delay(500);
    }
    
    // Test right side
    printf("\nRIGHT SIDE - All should move robot FORWARD:\n");
    for (int port : {11, -13, 15, -17, 19}) {
        int absPort = abs(port);
        pros::Motor motor(port);
        
        printf("Port %d: ", absPort);
        motor.move(40);
        pros::delay(500);
        motor.move(0);
        printf("Check wheel direction!\n");
        pros::delay(500);
    }
    
    printf("\n=== TEST COMPLETE ===\n");
    printf("All wheels on each side should spin same direction!\n");
}
```

---

## Mechanism Motors Fighting

The same problem can happen with mechanism motors:

### Intake Example

```cpp
// WRONG: Both motors spin same direction, but one is mounted opposite
.intake_ports = {6, 7}

// RIGHT: One motor reversed to match mounting
.intake_ports = {-6, 7}
```

### Conveyor Example

```cpp
// Four-motor conveyor with alternating mounting
.conveyor_ports = {2, -3, -4, 5}
//                    ↑   ↑
//                Motors mounted opposite way
```

### How to Check Mechanisms

1. Remove game pieces
2. Run mechanism at low power
3. Watch ALL rollers/wheels
4. They should all move the same direction
5. If not, one motor sign is wrong

---

## Summary Checklist

When motors are fighting:

- [ ] Identify which motors are fighting (disconnect to isolate)
- [ ] Test each motor individually
- [ ] Check physical direction vs expected direction
- [ ] Verify port numbers match physical wiring
- [ ] Add/remove negative signs to fix direction
- [ ] Test again to confirm fix
- [ ] Update documentation

---

*For other motor issues, see [COMMON_ISSUES.md](./COMMON_ISSUES.md)*
*For motor configuration, see [ROBOT_CONFIG.md](../configuration/ROBOT_CONFIG.md)*