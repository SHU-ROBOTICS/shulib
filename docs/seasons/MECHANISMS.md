# Mechanisms

**Defining game-specific motors and pneumatics**

---

## Table of Contents

1. [Overview](#overview)
2. [The Mechanisms Class](#the-mechanisms-class)
3. [Motor Groups](#motor-groups)
4. [Pneumatics](#pneumatics)
5. [Writing Mechanism Methods](#writing-mechanism-methods)
6. [Common Patterns](#common-patterns)
7. [Our Push Back Mechanisms](#our-push-back-mechanisms)
8. [Adding New Mechanisms](#adding-new-mechanisms)
9. [Testing Mechanisms](#testing-mechanisms)
10. [Key Takeaways](#key-takeaways)

---

## Overview

### What Are Mechanisms?

**Mechanisms** are the game-specific parts of your robot - everything except the drivetrain. They change every year based on the game:

| Game | Typical Mechanisms |
|------|-------------------|
| Push Back 2026 | Intake, Conveyor, Releaser, Arm |
| Previous years | Catapults, lifts, claws, flywheels |
| Future games | Who knows! |

### Why a Mechanisms Class?

Instead of scattered motor objects everywhere:

```cpp
// BAD: Motors scattered throughout code
pros::Motor intake1(6);
pros::Motor intake2(7);

void runIntake() {
    intake1.move(127);
    intake2.move(-127);  // Reversed? Who remembers?
}
```

We centralize everything:

```cpp
// GOOD: Organized in Mechanisms class
class Mechanisms {
    void setIntake(int power);  // Clean interface
};

mech.setIntake(127);  // Just works
```

---

## The Mechanisms Class

### Structure

```cpp
class Mechanisms {
public:
    // Constructor - receives robot config
    Mechanisms(const RobotConfig& config);
    
    // Initialize hardware
    void init();
    
    // Public methods for each mechanism
    void setIntake(int power);
    void setConveyor(int power);
    // ...
    
private:
    // Motor groups
    pros::MotorGroup* intake;
    pros::MotorGroup* conveyor;
    
    // Pneumatics
    pros::ADIDigitalOut* arm;
    
    // State tracking
    bool armExtended = false;
    
    // Config reference
    const RobotConfig& config;
};
```

### Why This Design?

| Design Choice | Reason |
|---------------|--------|
| Takes config in constructor | Ports come from robot config, not hardcoded |
| Separate `init()` method | Can delay hardware creation until needed |
| Private motor groups | External code can't directly access motors |
| Public methods | Clean interface, hides complexity |
| State tracking | Remember toggle states, positions, etc. |

---

## Motor Groups

### What's a Motor Group?

A `pros::MotorGroup` controls multiple motors as one unit. Perfect for mechanisms with 2+ motors.

```cpp
// Create from vector of ports
std::vector<int> ports = {6, -7};  // -7 means reversed
pros::MotorGroup* intake = new pros::MotorGroup(ports);

// Control as one
intake->move(127);  // Both motors run together
```

### Creating from Config

```cpp
void Mechanisms::init() {
    // Config has: .intake_ports = {-6, 7}
    intake = new pros::MotorGroup(config.mechanisms.intake_ports);
    
    // Config has: .conveyor_ports = {2, -3, -4, 5}
    conveyor = new pros::MotorGroup(config.mechanisms.conveyor_ports);
}
```

### Motor Directions

Negative port number = reversed motor:

```cpp
.intake_ports = {-6, 7}
//               │   │
//               │   └── Port 7, normal direction
//               └── Port 6, REVERSED
```

This is set in the robot config file, not in the Mechanisms class.

### Brake Modes

Set appropriate brake mode for each mechanism:

```cpp
void Mechanisms::init() {
    // Intake: coast when stopped (don't fight game pieces)
    intake->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    
    // Lift: hold position when stopped
    lift->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    
    // Flywheel: coast (momentum matters)
    flywheel->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}
```

| Brake Mode | Behavior | Use For |
|------------|----------|---------|
| `COAST` | Free spin when stopped | Intakes, flywheels |
| `BRAKE` | Resist motion when stopped | General mechanisms |
| `HOLD` | Actively hold position | Lifts, arms |

---

## Pneumatics

### Single-Acting vs Double-Acting

- **Single-acting:** One solenoid, spring return (most common in VEX)
- **Double-acting:** Two solenoids, explicit extend/retract

We typically use single-acting with `ADIDigitalOut`:

```cpp
pros::ADIDigitalOut* piston;

piston->set_value(true);   // Extend
piston->set_value(false);  // Retract (spring return)
```

### Creating Pneumatics

```cpp
void Mechanisms::init() {
    // Config has: .arm_port = 'B'
    arm = new pros::ADIDigitalOut(config.pneumatics.arm_port);
    
    // Config has: .lever_port = 'C'
    lever = new pros::ADIDigitalOut(config.pneumatics.lever_port);
}
```

### Toggle Pattern

Most pneumatics are toggled (press to extend, press again to retract):

```cpp
class Mechanisms {
private:
    bool armExtended = false;
    
public:
    void toggleArm() {
        armExtended = !armExtended;
        arm->set_value(armExtended);
    }
    
    // Or explicit control
    void extendArm() {
        armExtended = true;
        arm->set_value(true);
    }
    
    void retractArm() {
        armExtended = false;
        arm->set_value(false);
    }
};
```

---

## Writing Mechanism Methods

### Simple Power Control

For mechanisms that just need on/off or variable power:

```cpp
void Mechanisms::setIntake(int power) {
    intake->move(power);
}

// Usage:
mech.setIntake(127);   // Full forward
mech.setIntake(-127);  // Full reverse
mech.setIntake(0);     // Stop
```

### Combined Actions

For actions that involve multiple mechanisms:

```cpp
void Mechanisms::intakeAndConvey(int power) {
    intake->move(power);
    conveyor->move(power);
}

// Usage:
mech.intakeAndConvey(127);  // Intake and move through conveyor
```

### Timed Actions

For actions that run for a specific duration:

```cpp
void Mechanisms::shoot() {
    releaser->move(127);
    pros::delay(500);    // Run for 500ms
    releaser->move(0);
}
```

**⚠️ Warning:** This blocks! Only use in autonomous or be careful in opcontrol.

### Non-Blocking Timed Actions

For opcontrol, use state machines instead:

```cpp
class Mechanisms {
private:
    bool shooting = false;
    uint32_t shootStartTime = 0;
    const int SHOOT_DURATION = 500;  // ms
    
public:
    void startShoot() {
        shooting = true;
        shootStartTime = pros::millis();
        releaser->move(127);
    }
    
    void update() {
        // Call this every loop iteration
        if (shooting && pros::millis() - shootStartTime > SHOOT_DURATION) {
            releaser->move(0);
            shooting = false;
        }
    }
};
```

### Position Control

For mechanisms that move to specific positions (lifts, arms):

```cpp
void Mechanisms::liftToPosition(int targetPosition) {
    // Simple P control
    while (true) {
        int current = lift->get_position();
        int error = targetPosition - current;
        
        if (abs(error) < 10) break;  // Close enough
        
        int power = error * 0.5;  // P gain
        power = std::clamp(power, -127, 127);
        
        lift->move(power);
        pros::delay(10);
    }
    lift->move(0);
}
```

---

## Common Patterns

### Pattern 1: Run While Held

Motor runs while button is held, stops when released:

```cpp
// In opcontrol:
if (controller.get_digital(DIGITAL_R1)) {
    mech.setIntake(127);
} else if (controller.get_digital(DIGITAL_L1)) {
    mech.setIntake(-127);
} else {
    mech.setIntake(0);
}
```

### Pattern 2: Toggle On/Off

Button press toggles between running and stopped:

```cpp
// In Mechanisms class:
bool intakeRunning = false;

void toggleIntake() {
    intakeRunning = !intakeRunning;
    intake->move(intakeRunning ? 127 : 0);
}

// In opcontrol:
if (controller.get_digital_new_press(DIGITAL_R1)) {
    mech.toggleIntake();
}
```

### Pattern 3: Pneumatic Toggle

```cpp
void toggleArm() {
    armExtended = !armExtended;
    arm->set_value(armExtended);
}

// In opcontrol:
if (controller.get_digital_new_press(DIGITAL_Y)) {
    mech.toggleArm();
}
```

### Pattern 4: Multiple Speeds

```cpp
void setIntake(int power) {
    intake->move(power);
}

// In opcontrol:
if (controller.get_digital(DIGITAL_R1)) {
    mech.setIntake(127);      // Full speed
} else if (controller.get_digital(DIGITAL_R2)) {
    mech.setIntake(60);       // Half speed
} else {
    mech.setIntake(0);
}
```

### Pattern 5: Linked Mechanisms

```cpp
void score() {
    // Coordinated action
    conveyor->move(127);
    pros::delay(200);
    releaser->move(127);
    pros::delay(300);
    releaser->move(0);
    conveyor->move(0);
}
```

---

## Our Push Back Mechanisms

### Current Configuration

| Mechanism | Purpose | Motors | Control |
|-----------|---------|--------|---------|
| Intake | Pick up blocks | 2 motors | R1 in, L1 out |
| Conveyor | Move blocks through robot | 4 motors | With intake |
| Releaser | Score blocks | 1-2 motors | R2 forward, L2 back |
| Arm | Pneumatic positioning | Pneumatic | Y toggle |
| Lever | Goal manipulation | Pneumatic | LEFT toggle |

### XEBEC Implementation

```cpp
// From xebec.hpp config:
.mechanisms = {
    .intake_ports = {-6, 7},
    .conveyor_ports = {2, -3, -4, 5},
    .releaser_ports = {1}
},
.pneumatics = {
    .arm_port = 'B',
    .lever_port = 'C'
}
```

### Methods

```cpp
class Mechanisms {
public:
    void setIntake(int power);         // Set intake speed
    void setConveyor(int power);       // Set conveyor speed
    void setReleaser(int power);       // Set releaser speed
    void intakeAndConvey(int power);   // Both together
    void toggleArm();                  // Toggle arm pneumatic
    void toggleLever();                // Toggle lever pneumatic
};
```

---

## Adding New Mechanisms

### Step 1: Update Config Struct

In `robot_config.hpp`:

```cpp
struct MechanismConfig {
    std::vector<int> intake_ports;
    std::vector<int> conveyor_ports;
    std::vector<int> releaser_ports;
    std::vector<int> new_mechanism_ports;  // ADD THIS
};
```

### Step 2: Update Robot Configs

In `xebec.hpp` and `queens_revenge.hpp`:

```cpp
.mechanisms = {
    .intake_ports = {...},
    .conveyor_ports = {...},
    .releaser_ports = {...},
    .new_mechanism_ports = {8, -9}  // ADD THIS
}
```

### Step 3: Add to Mechanisms Class

In `mechanisms.hpp`:

```cpp
class Mechanisms {
public:
    void setNewMechanism(int power);
    
private:
    pros::MotorGroup* newMechanism;
};
```

In `mechanisms.cpp`:

```cpp
void Mechanisms::init() {
    // ... existing code ...
    newMechanism = new pros::MotorGroup(config.mechanisms.new_mechanism_ports);
}

void Mechanisms::setNewMechanism(int power) {
    newMechanism->move(power);
}
```

### Step 4: Add Controls

In `opcontrol.cpp`:

```cpp
if (controller.get_digital(DIGITAL_X)) {
    mech.setNewMechanism(127);
} else {
    mech.setNewMechanism(0);
}
```

---

## Testing Mechanisms

### Individual Motor Test

```cpp
void testMotors() {
    printf("Testing intake...\n");
    mech.setIntake(60);
    pros::delay(1000);
    mech.setIntake(0);
    
    printf("Testing conveyor...\n");
    mech.setConveyor(60);
    pros::delay(1000);
    mech.setConveyor(0);
    
    // ... repeat for each mechanism
}
```

### Direction Test

```cpp
void testDirections() {
    printf("Intake FORWARD - should pull in\n");
    mech.setIntake(60);
    pros::delay(2000);
    mech.setIntake(0);
    
    printf("Intake REVERSE - should push out\n");
    mech.setIntake(-60);
    pros::delay(2000);
    mech.setIntake(0);
}
```

### Pneumatic Test

```cpp
void testPneumatics() {
    printf("Extending arm...\n");
    mech.extendArm();
    pros::delay(1000);
    
    printf("Retracting arm...\n");
    mech.retractArm();
    pros::delay(1000);
}
```

### Full System Test

```cpp
void testFullCycle() {
    printf("Full scoring cycle:\n");
    
    printf("1. Intake block\n");
    mech.intakeAndConvey(127);
    pros::delay(1500);
    mech.intakeAndConvey(0);
    
    printf("2. Score block\n");
    mech.setReleaser(127);
    pros::delay(500);
    mech.setReleaser(0);
    
    printf("Done!\n");
}
```

---

## Key Takeaways

### The Essentials

1. **Mechanisms class centralizes all game-specific hardware**
   - Motors, pneumatics, sensors for mechanisms
   - Clean interface for opcontrol and autonomous

2. **Config-driven**
   - Ports come from robot config, not hardcoded
   - Easy to update for different robots

3. **Motor groups simplify multi-motor mechanisms**
   - Control multiple motors as one
   - Direction set by port sign (negative = reversed)

4. **Common patterns**
   - Run while held
   - Toggle on/off
   - Pneumatic toggle
   - Combined actions

5. **Test individually**
   - Verify each mechanism works
   - Check directions are correct

### Quick Reference

```cpp
// Create motor group
intake = new pros::MotorGroup(config.mechanisms.intake_ports);

// Create pneumatic
arm = new pros::ADIDigitalOut(config.pneumatics.arm_port);

// Simple control
void setIntake(int power) { intake->move(power); }

// Toggle pneumatic
void toggleArm() {
    armExtended = !armExtended;
    arm->set_value(armExtended);
}
```

---

*For the overall season structure, see [ADDING_A_SEASON.md](./ADDING_A_SEASON.md)*
*For control mappings, see [OPCONTROL.md](./OPCONTROL.md)*
*For Push Back specifics, see [pushback_2026/CONTROLS.md](./pushback_2026/CONTROLS.md)*