# Adding a Season

**How to set up code for a new competition year**

---

## Table of Contents

1. [Overview](#overview)
2. [Season Architecture](#season-architecture)
3. [Creating a New Season](#creating-a-new-season)
4. [File Templates](#file-templates)
5. [Connecting to Main](#connecting-to-main)
6. [Season Checklist](#season-checklist)
7. [Example: Creating 2027 Season](#example-creating-2027-season)

---

## Overview

### Why Seasons?

Each year's VEX game is different. By separating game-specific code into "seasons," we:

- **Preserve history** - Past seasons remain intact for reference
- **Keep core clean** - Reusable library code stays separate
- **Start fresh** - New season, clean slate for mechanisms and autos
- **Learn from the past** - Can look at how previous years solved problems

### What's Season-Specific?

| Season-Specific | Core Library (Reusable) |
|-----------------|------------------------|
| Mechanisms (intake, lift, etc.) | Chassis / Drivetrain |
| Autonomous routines | Odometry |
| Operator control mappings | PID |
| Game strategy | Pose / Math |
| Button assignments | Logger |

---

## Season Architecture

### Folder Structure

```
shulib/
├── include/shulib/
│   ├── core/              ← REUSABLE (don't touch for new season)
│   │   ├── chassis.hpp
│   │   ├── odometry.hpp
│   │   └── ...
│   │
│   ├── robots/            ← Robot configs (may need updates)
│   │   ├── robot_config.hpp
│   │   └── ...
│   │
│   └── seasons/
│       ├── pushback_2026/    ← Current season
│       │   ├── auton.hpp
│       │   ├── mechanisms.hpp
│       │   └── opcontrol.hpp
│       │
│       └── newgame_2027/     ← NEW SEASON GOES HERE
│           ├── auton.hpp
│           ├── mechanisms.hpp
│           └── opcontrol.hpp
│
└── src/
    ├── core/              ← REUSABLE (don't touch)
    │
    └── seasons/
        ├── pushback_2026/
        │   ├── auton.cpp
        │   ├── mechanisms.cpp
        │   └── opcontrol.cpp
        │
        └── newgame_2027/     ← NEW SEASON IMPLEMENTATION
            ├── auton.cpp
            ├── mechanisms.cpp
            └── opcontrol.cpp
```

### The Three Season Files

Every season needs exactly three files (header + implementation):

| File | Purpose |
|------|---------|
| `mechanisms.hpp/.cpp` | Define game-specific mechanisms |
| `auton.hpp/.cpp` | Autonomous routines |
| `opcontrol.hpp/.cpp` | Driver control |

---

## Creating a New Season

### Step 1: Create Directories

```bash
# Create header directory
mkdir -p include/shulib/seasons/newgame_2027

# Create source directory
mkdir -p src/seasons/newgame_2027
```

### Step 2: Create Header Files

Create three files in `include/shulib/seasons/newgame_2027/`:

- `mechanisms.hpp`
- `auton.hpp`
- `opcontrol.hpp`

### Step 3: Create Source Files

Create three files in `src/seasons/newgame_2027/`:

- `mechanisms.cpp`
- `auton.cpp`
- `opcontrol.cpp`

### Step 4: Update main.cpp

Change the includes to point to the new season:

```cpp
// Change these lines:
#include "shulib/seasons/pushback_2026/mechanisms.hpp"
#include "shulib/seasons/pushback_2026/auton.hpp"
#include "shulib/seasons/pushback_2026/opcontrol.hpp"

// To these:
#include "shulib/seasons/newgame_2027/mechanisms.hpp"
#include "shulib/seasons/newgame_2027/auton.hpp"
#include "shulib/seasons/newgame_2027/opcontrol.hpp"
```

### Step 5: Update Robot Configs (If Needed)

If your robot has new mechanisms, update `robot_config.hpp`:

```cpp
struct MechanismConfig {
    // Old mechanisms might not apply
    // std::vector<int> intake_ports;
    
    // Add new ones
    std::vector<int> launcher_ports;
    std::vector<int> lift_ports;
};
```

---

## File Templates

### mechanisms.hpp

```cpp
#pragma once
#include "main.h"
#include "shulib/robots/robot_config.hpp"

namespace newgame_2027 {

class Mechanisms {
public:
    // Constructor - takes robot config
    Mechanisms(const shulib::robots::RobotConfig& config);
    
    // Initialize (call in initialize())
    void init();
    
    // ══════════════════════════════════════════════════════════
    // DEFINE YOUR MECHANISMS HERE
    // ══════════════════════════════════════════════════════════
    
    // Example: Launcher
    void setLauncher(int power);
    void launcherShoot();
    void launcherStop();
    
    // Example: Lift
    void setLift(int power);
    void liftToPosition(int position);
    
    // Example: Pneumatics
    void toggleClaw();
    
private:
    // Motor groups
    pros::MotorGroup* launcher;
    pros::MotorGroup* lift;
    
    // Pneumatics
    pros::ADIDigitalOut* claw;
    bool clawExtended = false;
    
    // Config reference
    const shulib::robots::RobotConfig& config;
};

} // namespace newgame_2027
```

### mechanisms.cpp

```cpp
#include "shulib/seasons/newgame_2027/mechanisms.hpp"

namespace newgame_2027 {

Mechanisms::Mechanisms(const shulib::robots::RobotConfig& config) 
    : config(config) {
    // Motors and pneumatics created in init()
}

void Mechanisms::init() {
    // Create motor groups from config
    launcher = new pros::MotorGroup(config.mechanisms.launcher_ports);
    lift = new pros::MotorGroup(config.mechanisms.lift_ports);
    
    // Create pneumatics
    claw = new pros::ADIDigitalOut(config.pneumatics.claw_port);
    
    // Set motor modes
    launcher->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    lift->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

// ══════════════════════════════════════════════════════════════
// IMPLEMENT YOUR MECHANISMS HERE
// ══════════════════════════════════════════════════════════════

void Mechanisms::setLauncher(int power) {
    launcher->move(power);
}

void Mechanisms::launcherShoot() {
    launcher->move(127);
}

void Mechanisms::launcherStop() {
    launcher->move(0);
}

void Mechanisms::setLift(int power) {
    lift->move(power);
}

void Mechanisms::liftToPosition(int position) {
    // Implement position control if needed
}

void Mechanisms::toggleClaw() {
    clawExtended = !clawExtended;
    claw->set_value(clawExtended);
}

} // namespace newgame_2027
```

### auton.hpp

```cpp
#pragma once
#include "shulib/core/chassis.hpp"
#include "mechanisms.hpp"

namespace newgame_2027 {
namespace auton {

// Main entry point - called from main.cpp autonomous()
void run(shulib::Chassis& chassis, Mechanisms& mech);

// ══════════════════════════════════════════════════════════════
// DECLARE YOUR AUTONOMOUS ROUTINES HERE
// ══════════════════════════════════════════════════════════════

void skills(shulib::Chassis& chassis, Mechanisms& mech);
void redLeft(shulib::Chassis& chassis, Mechanisms& mech);
void redRight(shulib::Chassis& chassis, Mechanisms& mech);
void blueLeft(shulib::Chassis& chassis, Mechanisms& mech);
void blueRight(shulib::Chassis& chassis, Mechanisms& mech);
void test(shulib::Chassis& chassis, Mechanisms& mech);

} // namespace auton
} // namespace newgame_2027
```

### auton.cpp

```cpp
#include "shulib/seasons/newgame_2027/auton.hpp"
#include "config.hpp"

namespace newgame_2027 {
namespace auton {

void run(shulib::Chassis& chassis, Mechanisms& mech) {
    // Select autonomous based on config.hpp setting
    #if defined(AUTON_SKILLS)
        skills(chassis, mech);
    #elif defined(AUTON_RED_LEFT)
        redLeft(chassis, mech);
    #elif defined(AUTON_RED_RIGHT)
        redRight(chassis, mech);
    #elif defined(AUTON_BLUE_LEFT)
        blueLeft(chassis, mech);
    #elif defined(AUTON_BLUE_RIGHT)
        blueRight(chassis, mech);
    #elif defined(AUTON_TEST)
        test(chassis, mech);
    #else
        // Default: do nothing
    #endif
}

// ══════════════════════════════════════════════════════════════
// IMPLEMENT YOUR AUTONOMOUS ROUTINES HERE
// ══════════════════════════════════════════════════════════════

void skills(shulib::Chassis& chassis, Mechanisms& mech) {
    chassis.setPose(0, 0, 0);
    
    // Your skills autonomous here
}

void redLeft(shulib::Chassis& chassis, Mechanisms& mech) {
    chassis.setPose(0, 0, 0);
    
    // Your red left autonomous here
}

void redRight(shulib::Chassis& chassis, Mechanisms& mech) {
    chassis.setPose(0, 0, 0);
    
    // Your red right autonomous here
}

void blueLeft(shulib::Chassis& chassis, Mechanisms& mech) {
    chassis.setPose(0, 0, 0);
    
    // Your blue left autonomous here
}

void blueRight(shulib::Chassis& chassis, Mechanisms& mech) {
    chassis.setPose(0, 0, 0);
    
    // Your blue right autonomous here
}

void test(shulib::Chassis& chassis, Mechanisms& mech) {
    chassis.setPose(0, 0, 0);
    
    // Test autonomous for development
}

} // namespace auton
} // namespace newgame_2027
```

### opcontrol.hpp

```cpp
#pragma once
#include "shulib/core/chassis.hpp"
#include "mechanisms.hpp"

namespace newgame_2027 {
namespace opcontrol {

// Main entry point - called from main.cpp opcontrol()
void run(shulib::Chassis& chassis, Mechanisms& mech);

} // namespace opcontrol
} // namespace newgame_2027
```

### opcontrol.cpp

```cpp
#include "shulib/seasons/newgame_2027/opcontrol.hpp"

namespace newgame_2027 {
namespace opcontrol {

void run(shulib::Chassis& chassis, Mechanisms& mech) {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    
    while (true) {
        // ══════════════════════════════════════════════════════
        // DRIVETRAIN
        // ══════════════════════════════════════════════════════
        
        int leftY = controller.get_analog(ANALOG_LEFT_Y);
        int rightX = controller.get_analog(ANALOG_RIGHT_X);
        
        chassis.drive(0, leftY, rightX);
        
        // ══════════════════════════════════════════════════════
        // MECHANISMS - DEFINE YOUR CONTROLS HERE
        // ══════════════════════════════════════════════════════
        
        // Example: R1 to shoot
        if (controller.get_digital(DIGITAL_R1)) {
            mech.launcherShoot();
        } else {
            mech.launcherStop();
        }
        
        // Example: L1/L2 for lift
        if (controller.get_digital(DIGITAL_L1)) {
            mech.setLift(127);
        } else if (controller.get_digital(DIGITAL_L2)) {
            mech.setLift(-127);
        } else {
            mech.setLift(0);
        }
        
        // Example: A to toggle claw
        if (controller.get_digital_new_press(DIGITAL_A)) {
            mech.toggleClaw();
        }
        
        // Loop delay
        pros::delay(10);
    }
}

} // namespace opcontrol
} // namespace newgame_2027
```

---

## Connecting to Main

### main.cpp Changes

```cpp
#include "main.h"
#include "config.hpp"
#include "shulib/core/chassis.hpp"

// ══════════════════════════════════════════════════════════════
// SEASON INCLUDES - CHANGE THESE FOR NEW SEASON
// ══════════════════════════════════════════════════════════════
#include "shulib/seasons/newgame_2027/mechanisms.hpp"
#include "shulib/seasons/newgame_2027/auton.hpp"
#include "shulib/seasons/newgame_2027/opcontrol.hpp"

// Use the season namespace
using namespace newgame_2027;

// Global objects
shulib::Chassis* chassis;
Mechanisms* mechanisms;

void initialize() {
    // ... chassis setup ...
    
    // Initialize mechanisms
    mechanisms = new Mechanisms(ROBOT);
    mechanisms->init();
}

void autonomous() {
    auton::run(*chassis, *mechanisms);
}

void opcontrol() {
    opcontrol::run(*chassis, *mechanisms);
}
```

---

## Season Checklist

### Before Season Starts

- [ ] Create season directories
- [ ] Copy templates to new files
- [ ] Update `main.cpp` includes
- [ ] Update namespace references
- [ ] Build and verify compiles

### Designing Mechanisms

- [ ] Identify all game mechanisms needed
- [ ] Update `robot_config.hpp` if new motor/pneumatic fields needed
- [ ] Update robot config files (xebec.hpp, etc.)
- [ ] Implement Mechanisms class
- [ ] Test each mechanism individually

### Setting Up Controls

- [ ] Decide button mappings
- [ ] Implement in `opcontrol.cpp`
- [ ] Document in controls doc
- [ ] Test with drivers

### Writing Autonomous

- [ ] Identify starting positions
- [ ] Plan routes for each position
- [ ] Implement each routine
- [ ] Test and tune

---

## Example: Creating 2027 Season

Let's say the 2027 game is called "Stack Attack" and involves:
- A launcher to shoot balls
- A lift to stack cones
- A pneumatic claw

### Step 1: Create Folders

```bash
mkdir -p include/shulib/seasons/stackattack_2027
mkdir -p src/seasons/stackattack_2027
```

### Step 2: Update robot_config.hpp

```cpp
struct MechanismConfig {
    std::vector<int> launcher_ports;
    std::vector<int> lift_ports;
};

struct PneumaticConfig {
    char claw_port;
};
```

### Step 3: Update Robot Config (xebec.hpp)

```cpp
.mechanisms = {
    .launcher_ports = {1, -2},
    .lift_ports = {3, -4}
},
.pneumatics = {
    .claw_port = 'A'
}
```

### Step 4: Create Season Files

Copy templates from above, customize for Stack Attack.

### Step 5: Update main.cpp

```cpp
#include "shulib/seasons/stackattack_2027/mechanisms.hpp"
#include "shulib/seasons/stackattack_2027/auton.hpp"
#include "shulib/seasons/stackattack_2027/opcontrol.hpp"

using namespace stackattack_2027;
```

### Step 6: Build and Test

```bash
pros make clean && pros make
```

---

*For mechanism details, see [MECHANISMS.md](./MECHANISMS.md)*
*For autonomous writing, see [AUTONOMOUS.md](./AUTONOMOUS.md)*
*For controls setup, see [OPCONTROL.md](./OPCONTROL.md)*