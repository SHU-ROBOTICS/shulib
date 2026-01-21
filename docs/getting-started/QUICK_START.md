# Quick Start Guide

Get shulib running on your robot in 10 minutes.

---

## Table of Contents

- [Quick Start Guide](#quick-start-guide)
  - [Table of Contents](#table-of-contents)
  - [Prerequisites](#prerequisites)
  - [Step 1: Select Your Robot](#step-1-select-your-robot)
    - [Don't see your robot?](#dont-see-your-robot)
  - [Step 2: Select Your Autonomous](#step-2-select-your-autonomous)
  - [Step 3: Build](#step-3-build)
    - [Build failed?](#build-failed)
  - [Step 4: Connect Your Robot](#step-4-connect-your-robot)
    - [Not showing up?](#not-showing-up)
  - [Step 5: Upload](#step-5-upload)
  - [Step 6: Test](#step-6-test)
    - [Start the Terminal](#start-the-terminal)
    - [Test Driver Control](#test-driver-control)
    - [Test Autonomous](#test-autonomous)
    - [Something Wrong?](#something-wrong)
  - [What's Next](#whats-next)
    - [Understand the Code](#understand-the-code)
    - [Configure Your Robot](#configure-your-robot)
    - [Write Autonomous Routines](#write-autonomous-routines)
    - [Prepare for Competition](#prepare-for-competition)
  - [Quick Reference](#quick-reference)
    - [Terminal Commands](#terminal-commands)
    - [Controller Buttons (Default)](#controller-buttons-default)
    - [Config File Location](#config-file-location)
    - [Key Files](#key-files)

---

## Prerequisites

Before starting, make sure you have:

- [x] Development environment set up ([Installation Guide](INSTALLATION.md))
- [x] A VEX V5 robot with brain, controller, and drive motors
- [x] USB cable to connect brain to computer
- [x] Charged robot battery

---

## Step 1: Select Your Robot

Open `config.hpp` in the project root directory.

Find the robot selection section and uncomment your robot:
```cpp
// ═══════════════════════════════════════════════════════════
// ROBOT SELECTION
// Uncomment ONE of the following:
// ═══════════════════════════════════════════════════════════

// #define ROBOT_XEBEC
// #define ROBOT_QUEENS_REVENGE
#define ROBOT_TESTBOT              // ← Uncomment your robot
```

**Important:** Only ONE robot should be uncommented at a time.

### Don't see your robot?

If your robot isn't listed, you'll need to create a configuration for it. See [Adding a Robot](../configuration/ADDING_A_ROBOT.md).

---

## Step 2: Select Your Autonomous

In the same `config.hpp` file, find the autonomous selection section:
```cpp
// ═══════════════════════════════════════════════════════════
// AUTONOMOUS SELECTION
// Uncomment ONE of the following:
// ═══════════════════════════════════════════════════════════

// #define AUTON_SKILLS
// #define AUTON_RED_LEFT
// #define AUTON_RED_RIGHT
// #define AUTON_BLUE_LEFT
// #define AUTON_BLUE_RIGHT
#define AUTON_TEST                 // ← Good for first-time testing
```

**For first-time setup**, use `AUTON_TEST`. It runs a simple motion test:
1. Drive forward 24 inches
2. Turn to 90 degrees
3. Drive forward 24 inches
4. Turn back to 0 degrees

---

## Step 3: Build

Open a terminal in the project directory and run:
```bash
pros make
```

**Expected output:**
```
Compiling src/main.cpp...
Compiling src/core/chassis.cpp...
...
Linking output.bin...
[OK] build/output.bin
```

### Build failed?

| Error | Solution |
|-------|----------|
| "No such file or directory" | Make sure you're in the project root directory |
| "config.hpp not found" | Check that config.hpp exists in the root |
| Undefined reference errors | Make sure only ONE robot is selected |
| See [Build Errors](../troubleshooting/BUILD_ERRORS.md) for more |

---

## Step 4: Connect Your Robot

1. **Turn on the V5 Brain** (press the power button)
2. **Connect USB cable** from brain to computer
3. **Verify connection:**
```bash
pros lsusb
```

You should see output like:
```
VEX V5 Brain (USB) - /dev/ttyACM0
```

### Not showing up?

- Try a different USB cable (some are charge-only)
- Try a different USB port
- On Linux, you may need udev rules (see [Installation](INSTALLATION.md#troubleshooting))

---

## Step 5: Upload

Upload the compiled code to the robot:
```bash
pros upload
```

**Expected output:**
```
Uploading bin/output.bin to V5 via /dev/ttyACM0
[████████████████████████████████] 100%
Upload complete.
```

The brain screen will show upload progress and then display your program.

---

## Step 6: Test

### Start the Terminal

To see debug output from the robot:
```bash
pros terminal
```

You'll see the PROS banner followed by initialization messages:
```
Initializing logger...
{"messages": [{ "message": "Logger initialized!", "type": "success" }]}
{"messages": [{ "message": "Initializing TestBot", "type": "log" }]}
...
=== OPCONTROL STARTED ===
Press A to run autonomous
Press B to print position
```

### Test Driver Control

1. **Disconnect the USB cable** (or keep it connected for terminal output)
2. **Turn on the V5 Controller**
3. **Wait for controller to connect** to the brain

Try the controls:

| Control | Action |
|---------|--------|
| Left Stick Y | Drive forward / backward |
| Right Stick X | Turn left / right |

**Verify:**
- Robot drives straight (doesn't curve)
- Robot turns in correct direction
- No grinding or fighting sounds from motors

### Test Autonomous

1. Press **A** on the controller to run the selected autonomous
2. Watch the robot perform the test routine:
   - Forward 24 inches
   - Turn to 90°
   - Forward 24 inches
   - Turn to 0°

3. Press **B** to print the current position

**Expected terminal output:**
```
>>> A PRESSED - Running Autonomous <

=== CHASSIS.DRIVE() TEST ===
Test 1: Forward 24 inches
=== MOVE 24.0 inches ===
=== MOVE DONE: 24.2 inches ===
Result: X=0.1 Y=24.2 Theta=0.5

Test 2: Turn to 90 degrees
=== ROTATE TO 90.0 deg ===
=== ROTATE DONE: 89.1 deg (target was 90.0) ===
...
```

### Something Wrong?

| Problem | See |
|---------|-----|
| Motors making grinding noise, not moving | [Motors Fighting](../troubleshooting/MOTORS_FIGHTING.md) |
| Robot curves instead of going straight | [Odometry Drift](../troubleshooting/ODOMETRY_DRIFT.md) |
| Robot shakes at end of movements | [PID Oscillation](../troubleshooting/PID_OSCILLATION.md) |
| High current (10+ amps) in terminal | [Motors Fighting](../troubleshooting/MOTORS_FIGHTING.md) |

---

## What's Next

### Understand the Code

- [Project Structure](PROJECT_STRUCTURE.md) – What's in each folder
- [Architecture Overview](../architecture/OVERVIEW.md) – How everything fits together

### Configure Your Robot

- [Adding a Robot](../configuration/ADDING_A_ROBOT.md) – Create a config for your robot
- [Motor Signs](../configuration/MOTOR_SIGNS.md) – Fix motor direction issues

### Write Autonomous Routines

- [Motion Overview](../motion/MOTION_OVERVIEW.md) – How motion functions work
- [Autonomous Guide](../seasons/AUTONOMOUS.md) – Writing competition routines
- [PID Tuning](../motion/PID_TUNING.md) – Get smooth, accurate movement

### Prepare for Competition

- [Pre-Match Checklist](../competition/CHECKLIST.md) – Don't forget anything
- [Switching Robots](../competition/SWITCHING_ROBOTS.md) – Quick config changes

---

## Quick Reference

### Terminal Commands

| Command | Description |
|---------|-------------|
| `pros make` | Build the project |
| `pros make clean` | Delete build files, then build |
| `pros upload` | Upload to robot |
| `pros terminal` | View serial output |
| `pros lsusb` | List connected V5 devices |
| `pros conduct info-project` | Show project info |

### Controller Buttons (Default)

| Button | Action |
|--------|--------|
| A | Run autonomous |
| B | Print current position |
| Left Stick Y | Forward / Backward |
| Right Stick X | Turn |
| R1 | Intake + Conveyor In |
| L1 | Intake + Conveyor Out |
| R2 | Releaser Forward |
| L2 | Releaser Backward |
| Y | Toggle Arm Pneumatic |
| Left Arrow | Toggle Lever Pneumatic |

### Config File Location
```
shulib/
└── config.hpp          ← Edit this to switch robots/autos
```

### Key Files

| File | Purpose |
|------|---------|
| `config.hpp` | Robot and autonomous selection |
| `src/main.cpp` | Initialization and wiring |
| `src/seasons/pushback_2026/auton.cpp` | Autonomous routines |
| `src/seasons/pushback_2026/opcontrol.cpp` | Driver control |
| `include/shulib/robots/*.hpp` | Robot configurations |