# Operator Control

**Driver control setup and button mapping**

---

## Table of Contents

1. [Overview](#overview)
2. [The Opcontrol Loop](#the-opcontrol-loop)
3. [Reading the Controller](#reading-the-controller)
4. [Drivetrain Control](#drivetrain-control)
5. [Button Mapping Patterns](#button-mapping-patterns)
6. [Mechanism Control](#mechanism-control)
7. [State Management](#state-management)
8. [Driver Feedback](#driver-feedback)
9. [Testing Controls](#testing-controls)
10. [Key Takeaways](#key-takeaways)

---

## Overview

### What is Opcontrol?

**Operator control** (opcontrol) is the driver-controlled period of the match (1 minute 45 seconds). Your code reads controller inputs and translates them into robot actions.

### The Goal

```
Driver Input              Code                   Robot Action
─────────────            ──────                  ────────────
Push left stick    →    Read analog value    →  Motors spin
Press R1           →    Read digital value   →  Intake runs
Press Y            →    Detect new press     →  Toggle arm
```

### File Structure

```
include/shulib/seasons/pushback_2026/
└── opcontrol.hpp    ← Declaration

src/seasons/pushback_2026/
└── opcontrol.cpp    ← Implementation
```

---

## The Opcontrol Loop

### Basic Structure

```cpp
void run(shulib::Chassis& chassis, Mechanisms& mech) {
    // Create controller object
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    
    // Main loop - runs forever during opcontrol
    while (true) {
        // ════════════════════════════════════════
        // 1. DRIVETRAIN
        // ════════════════════════════════════════
        int leftY = controller.get_analog(ANALOG_LEFT_Y);
        int rightX = controller.get_analog(ANALOG_RIGHT_X);
        chassis.drive(0, leftY, rightX);
        
        // ════════════════════════════════════════
        // 2. MECHANISMS
        // ════════════════════════════════════════
        // ... button mappings here ...
        
        // ════════════════════════════════════════
        // 3. LOOP DELAY
        // ════════════════════════════════════════
        pros::delay(10);  // 10ms = 100Hz loop
    }
}
```

### Why the Delay?

Without delay, the loop runs as fast as possible:
- Wastes CPU
- May cause issues with motor commands
- No benefit (motors only update every 10ms anyway)

10ms delay = 100 iterations per second = plenty fast.

### The Controller Object

```cpp
// Master controller (driver 1)
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Partner controller (driver 2, if using)
pros::Controller partner(pros::E_CONTROLLER_PARTNER);
```

---

## Reading the Controller

### Analog Sticks

```cpp
// Get stick position (-127 to 127)
int leftY = controller.get_analog(ANALOG_LEFT_Y);   // Forward/back
int leftX = controller.get_analog(ANALOG_LEFT_X);   // Left/right
int rightY = controller.get_analog(ANALOG_RIGHT_Y); // Forward/back
int rightX = controller.get_analog(ANALOG_RIGHT_X); // Left/right
```

**Stick Layout:**
```
      LEFT STICK              RIGHT STICK
          ↑                       ↑
      +Y (127)                +Y (127)
          │                       │
  -X ←────┼────→ +X      -X ←────┼────→ +X
 (-127)   │    (127)    (-127)   │    (127)
          │                       │
      -Y (-127)              -Y (-127)
          ↓                       ↓
```

### Digital Buttons

```cpp
// Is button currently pressed?
bool pressed = controller.get_digital(DIGITAL_R1);

// Was button JUST pressed (this frame only)?
bool newPress = controller.get_digital_new_press(DIGITAL_R1);
```

**Button Names:**
```
┌─────────────────────────────────────────┐
│  L1  L2                      R1  R2     │
│  ┌─────┐    [UP]  [DOWN]    ┌─────┐     │
│  │     │    [LEFT][RIGHT]   │     │     │
│  │     │                    │     │     │
│  └─────┘   [X] [Y] [A] [B]  └─────┘     │
│    LEFT       BUTTONS        RIGHT      │
│    STICK                     STICK      │
└─────────────────────────────────────────┘

DIGITAL_L1, DIGITAL_L2
DIGITAL_R1, DIGITAL_R2
DIGITAL_UP, DIGITAL_DOWN, DIGITAL_LEFT, DIGITAL_RIGHT
DIGITAL_X, DIGITAL_Y, DIGITAL_A, DIGITAL_B
```

### get_digital vs get_digital_new_press

| Function | Returns true... |
|----------|-----------------|
| `get_digital()` | Every loop while button is held |
| `get_digital_new_press()` | Only the first loop when pressed |

**Use `get_digital()` for:**
- Hold-to-run actions (intake, drive)
- Continuous control

**Use `get_digital_new_press()` for:**
- Toggle actions (pneumatics)
- One-shot actions (shoot once)

---

## Drivetrain Control

### Tank Drive (Split Stick)

Left stick = forward/back, Right stick = turning:

```cpp
int forward = controller.get_analog(ANALOG_LEFT_Y);
int turn = controller.get_analog(ANALOG_RIGHT_X);

chassis.drive(0, forward, turn);
```

**This is what we use.** Good for:
- Precise forward/backward control
- Fine turning control
- Most VEX games

### Arcade Drive (Single Stick)

One stick controls both:

```cpp
int forward = controller.get_analog(ANALOG_LEFT_Y);
int turn = controller.get_analog(ANALOG_LEFT_X);

chassis.drive(0, forward, turn);
```

Good for:
- Simple control
- Casual driving

### Tank Drive (Two Sticks)

Each stick controls one side:

```cpp
int left = controller.get_analog(ANALOG_LEFT_Y);
int right = controller.get_analog(ANALOG_RIGHT_Y);

// Convert to chassis.drive format
int forward = (left + right) / 2;
int turn = (right - left) / 2;
chassis.drive(0, forward, turn);

// Or direct motor control
leftMotors.move(left);
rightMotors.move(right);
```

Good for:
- Maximum control
- Experienced drivers

### Deadzone

Sticks rarely return exactly 0 when centered. Add a deadzone:

```cpp
int applyDeadzone(int value, int deadzone = 10) {
    if (abs(value) < deadzone) return 0;
    return value;
}

int forward = applyDeadzone(controller.get_analog(ANALOG_LEFT_Y));
int turn = applyDeadzone(controller.get_analog(ANALOG_RIGHT_X));
```

---

## Button Mapping Patterns

### Pattern 1: Hold to Run

Motor runs while button is held:

```cpp
if (controller.get_digital(DIGITAL_R1)) {
    mech.setIntake(127);
} else {
    mech.setIntake(0);
}
```

### Pattern 2: Two-Button Control

Different buttons for forward/reverse:

```cpp
if (controller.get_digital(DIGITAL_R1)) {
    mech.setIntake(127);   // Forward
} else if (controller.get_digital(DIGITAL_L1)) {
    mech.setIntake(-127);  // Reverse
} else {
    mech.setIntake(0);     // Stop
}
```

### Pattern 3: Toggle On/Off

Press to start, press again to stop:

```cpp
// Need static variable to remember state
static bool intakeRunning = false;

if (controller.get_digital_new_press(DIGITAL_R1)) {
    intakeRunning = !intakeRunning;
}

mech.setIntake(intakeRunning ? 127 : 0);
```

### Pattern 4: Pneumatic Toggle

```cpp
if (controller.get_digital_new_press(DIGITAL_Y)) {
    mech.toggleArm();
}
```

### Pattern 5: One-Shot Action

Runs once per press, doesn't repeat:

```cpp
if (controller.get_digital_new_press(DIGITAL_A)) {
    mech.shoot();  // Runs once
}
```

### Pattern 6: Variable Speed

Analog control for variable power:

```cpp
// Right trigger (L2/R2 are analog!)
int triggerValue = controller.get_analog(ANALOG_RIGHT_TRIGGER);
mech.setLift(triggerValue);
```

Wait, VEX controllers don't have analog triggers. Use stick instead:

```cpp
int liftPower = controller.get_analog(ANALOG_RIGHT_Y);
mech.setLift(liftPower);
```

### Pattern 7: Combined Buttons

Multiple mechanisms with one button:

```cpp
if (controller.get_digital(DIGITAL_R1)) {
    mech.setIntake(127);
    mech.setConveyor(127);
} else if (controller.get_digital(DIGITAL_L1)) {
    mech.setIntake(-127);
    mech.setConveyor(-127);
} else {
    mech.setIntake(0);
    mech.setConveyor(0);
}
```

---

## Mechanism Control

### Our Push Back Controls

```cpp
void run(shulib::Chassis& chassis, Mechanisms& mech) {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    
    while (true) {
        // ════════════════════════════════════════
        // DRIVETRAIN
        // ════════════════════════════════════════
        int leftY = controller.get_analog(ANALOG_LEFT_Y);
        int rightX = controller.get_analog(ANALOG_RIGHT_X);
        chassis.drive(0, leftY, rightX);
        
        // ════════════════════════════════════════
        // INTAKE + CONVEYOR (R1 in, L1 out)
        // ════════════════════════════════════════
        if (controller.get_digital(DIGITAL_R1)) {
            mech.intakeAndConvey(127);
        } else if (controller.get_digital(DIGITAL_L1)) {
            mech.intakeAndConvey(-127);
        } else {
            mech.intakeAndConvey(0);
        }
        
        // ════════════════════════════════════════
        // RELEASER (R2 forward, L2 back)
        // ════════════════════════════════════════
        if (controller.get_digital(DIGITAL_R2)) {
            mech.setReleaser(127);
        } else if (controller.get_digital(DIGITAL_L2)) {
            mech.setReleaser(-127);
        } else {
            mech.setReleaser(0);
        }
        
        // ════════════════════════════════════════
        // PNEUMATICS (toggle)
        // ════════════════════════════════════════
        if (controller.get_digital_new_press(DIGITAL_Y)) {
            mech.toggleArm();
        }
        
        if (controller.get_digital_new_press(DIGITAL_LEFT)) {
            mech.toggleLever();
        }
        
        // ════════════════════════════════════════
        // LOOP DELAY
        // ════════════════════════════════════════
        pros::delay(10);
    }
}
```

### Organizing Controls

Group related controls together with comments:

```cpp
// ══════════════════════════════════════════════════════════════
// INTAKE SYSTEM
// ══════════════════════════════════════════════════════════════
// R1: Intake + Conveyor IN
// L1: Intake + Conveyor OUT

if (controller.get_digital(DIGITAL_R1)) {
    mech.intakeAndConvey(127);
} else if (controller.get_digital(DIGITAL_L1)) {
    mech.intakeAndConvey(-127);
} else {
    mech.intakeAndConvey(0);
}
```

---

## State Management

### Static Variables

For toggle states that persist across loop iterations:

```cpp
void run(shulib::Chassis& chassis, Mechanisms& mech) {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    
    // Static = keeps value between function calls
    static bool armExtended = false;
    static bool slowMode = false;
    
    while (true) {
        // Toggle arm
        if (controller.get_digital_new_press(DIGITAL_Y)) {
            armExtended = !armExtended;
            mech.setArm(armExtended);
        }
        
        // Toggle slow mode
        if (controller.get_digital_new_press(DIGITAL_X)) {
            slowMode = !slowMode;
        }
        
        // Apply slow mode to driving
        int forward = controller.get_analog(ANALOG_LEFT_Y);
        int turn = controller.get_analog(ANALOG_RIGHT_X);
        
        if (slowMode) {
            forward /= 2;
            turn /= 2;
        }
        
        chassis.drive(0, forward, turn);
        
        pros::delay(10);
    }
}
```

### Mode Switching

Switch between different control modes:

```cpp
enum class Mode { NORMAL, SCORING, CLIMBING };
static Mode currentMode = Mode::NORMAL;

// Mode buttons
if (controller.get_digital_new_press(DIGITAL_UP)) {
    currentMode = Mode::NORMAL;
}
if (controller.get_digital_new_press(DIGITAL_DOWN)) {
    currentMode = Mode::SCORING;
}
if (controller.get_digital_new_press(DIGITAL_A)) {
    currentMode = Mode::CLIMBING;
}

// Different controls per mode
switch (currentMode) {
    case Mode::NORMAL:
        // Normal driving controls
        break;
    case Mode::SCORING:
        // Scoring-focused controls
        break;
    case Mode::CLIMBING:
        // Climbing controls
        break;
}
```

---

## Driver Feedback

### Controller Rumble

Give tactile feedback:

```cpp
// Short rumble
controller.rumble(".");

// Long rumble
controller.rumble("-");

// Pattern
controller.rumble("- . -");  // Long, short, long
```

Use for:
- Confirming toggle state
- Warning (low battery, motor hot)
- Game state changes

```cpp
if (controller.get_digital_new_press(DIGITAL_Y)) {
    mech.toggleArm();
    controller.rumble(".");  // Confirm toggle
}
```

### Controller Screen

Display status on controller:

```cpp
// Print to controller screen
controller.print(0, 0, "Mode: NORMAL");
controller.print(1, 0, "Arm: %s", armExtended ? "OUT" : "IN");

// Clear line
controller.clear_line(0);

// Clear all
controller.clear();
```

**Note:** Don't print every loop - too slow. Print only on change:

```cpp
static bool lastArmState = false;

if (armExtended != lastArmState) {
    controller.print(1, 0, "Arm: %s", armExtended ? "OUT" : "IN");
    lastArmState = armExtended;
}
```

---

## Testing Controls

### Button Test

```cpp
void testButtons() {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    
    while (true) {
        printf("L1:%d L2:%d R1:%d R2:%d ", 
               controller.get_digital(DIGITAL_L1),
               controller.get_digital(DIGITAL_L2),
               controller.get_digital(DIGITAL_R1),
               controller.get_digital(DIGITAL_R2));
        
        printf("X:%d Y:%d A:%d B:%d\n",
               controller.get_digital(DIGITAL_X),
               controller.get_digital(DIGITAL_Y),
               controller.get_digital(DIGITAL_A),
               controller.get_digital(DIGITAL_B));
        
        pros::delay(100);
    }
}
```

### Stick Test

```cpp
void testSticks() {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    
    while (true) {
        printf("LX:%4d LY:%4d RX:%4d RY:%4d\n",
               controller.get_analog(ANALOG_LEFT_X),
               controller.get_analog(ANALOG_LEFT_Y),
               controller.get_analog(ANALOG_RIGHT_X),
               controller.get_analog(ANALOG_RIGHT_Y));
        
        pros::delay(100);
    }
}
```

### Full Control Test

```cpp
void testControls() {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    
    printf("Press each button to test...\n");
    
    while (true) {
        if (controller.get_digital_new_press(DIGITAL_R1)) {
            printf("R1 pressed!\n");
            mech.setIntake(127);
            pros::delay(500);
            mech.setIntake(0);
        }
        
        if (controller.get_digital_new_press(DIGITAL_Y)) {
            printf("Y pressed - toggling arm\n");
            mech.toggleArm();
        }
        
        // ... test other buttons
        
        pros::delay(10);
    }
}
```

---

## Key Takeaways

### The Essentials

1. **The opcontrol loop runs continuously**
   - Read controller
   - Apply to robot
   - Delay 10ms
   - Repeat

2. **Analog sticks: -127 to 127**
   - Use for drive control
   - Apply deadzone if needed

3. **Digital buttons: two modes**
   - `get_digital()` - every frame while held
   - `get_digital_new_press()` - once per press

4. **Common patterns**
   - Hold to run
   - Two-button forward/reverse
   - Toggle on/off
   - Combined mechanisms

5. **Static variables for state**
   - Remember toggle states
   - Track modes

### Quick Reference

```cpp
// Controller
pros::Controller ctrl(pros::E_CONTROLLER_MASTER);

// Sticks
int value = ctrl.get_analog(ANALOG_LEFT_Y);  // -127 to 127

// Buttons
bool held = ctrl.get_digital(DIGITAL_R1);      // True while held
bool pressed = ctrl.get_digital_new_press(DIGITAL_R1);  // True once

// Feedback
ctrl.rumble(".");                    // Vibrate
ctrl.print(0, 0, "Hello");          // Display text
```

---

*For season structure, see [ADDING_A_SEASON.md](./ADDING_A_SEASON.md)*
*For mechanisms, see [MECHANISMS.md](./MECHANISMS.md)*
*For specific button mappings, see [pushback_2026/CONTROLS.md](./pushback_2026/CONTROLS.md)*