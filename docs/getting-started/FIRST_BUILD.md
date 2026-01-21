# First Build Guide

This guide walks you through building and uploading shulib to your robot for the first time.

---

## Table of Contents

- [Before You Start](#before-you-start)
- [Understanding the Build Process](#understanding-the-build-process)
- [Step-by-Step Build](#step-by-step-build)
- [Uploading to the Robot](#uploading-to-the-robot)
- [Verifying the Upload](#verifying-the-upload)
- [Build Commands Reference](#build-commands-reference)
- [Common Build Errors](#common-build-errors)

---

## Before You Start

### Checklist

- [ ] Development environment installed ([Installation Guide](INSTALLATION.md))
- [ ] Repository cloned and on `lodge` branch
- [ ] Robot selected in `config.hpp`
- [ ] Autonomous selected in `config.hpp`
- [ ] V5 Brain charged and powered on
- [ ] USB cable ready

### Verify Your Setup
```bash
# Check you're in the right directory
pwd
# Should show: /home/yourname/projects/shulib (or similar)

# Check you're on the right branch
git branch
# Should show: * lodge

# Check PROS is installed
pros --version
# Should show: 3.5.x or higher
```

---

## Understanding the Build Process

### What Happens When You Build
```
┌─────────────────────────────────────────────────────────────┐
│  1. PREPROCESSING                                           │
│     - Process #include directives                           │
│     - Expand #define macros (like ROBOT_TESTBOT)            │
│     - Handle conditional compilation (#if, #ifdef)          │
├─────────────────────────────────────────────────────────────┤
│  2. COMPILATION                                             │
│     - Convert each .cpp file to object code (.o)            │
│     - Check for syntax errors                               │
│     - Optimize code                                         │
├─────────────────────────────────────────────────────────────┤
│  3. LINKING                                                 │
│     - Combine all object files                              │
│     - Link with PROS libraries                              │
│     - Create final binary (output.bin)                      │
├─────────────────────────────────────────────────────────────┤
│  4. OUTPUT                                                  │
│     - bin/output.bin - The program for the V5 Brain         │
└─────────────────────────────────────────────────────────────┘
```

### Build Artifacts

After building, you'll have:
```
shulib/
├── bin/
│   └── output.bin          # Final binary for V5 Brain
└── build/
    ├── main.cpp.o          # Compiled main.cpp
    ├── chassis.cpp.o       # Compiled chassis.cpp
    └── ...                 # Other object files
```

---

## Step-by-Step Build

### Step 1: Navigate to Project Directory
```bash
cd ~/projects/shulib
```

### Step 2: Verify Configuration

Open `config.hpp` and confirm your selections:
```cpp
// Make sure ONE robot is uncommented
#define ROBOT_TESTBOT

// Make sure ONE autonomous is uncommented
#define AUTON_TEST
```

### Step 3: Clean Previous Build (Optional but Recommended)
```bash
pros make clean
```

This removes old build files. Recommended when:
- Switching robots
- Changing config.hpp
- Experiencing strange behavior
- First time building

### Step 4: Build
```bash
pros make
```

**Expected output:**
```
Compiling src/main.cpp...
Compiling src/core/chassis.cpp...
Compiling src/core/drivetrain.cpp...
Compiling src/core/odometry.cpp...
Compiling src/core/odomUnit.cpp...
Compiling src/core/pid.cpp...
Compiling src/core/pose.cpp...
Compiling src/core/logger.cpp...
Compiling src/core/util.cpp...
Compiling src/seasons/pushback_2026/auton.cpp...
Compiling src/seasons/pushback_2026/mechanisms.cpp...
Compiling src/seasons/pushback_2026/opcontrol.cpp...
Linking bin/output.bin...
Section sizes:
   text    data     bss     dec     hex filename
  xxxxx   xxxxx   xxxxx   xxxxx   xxxxx bin/output.bin
[OK] bin/output.bin
```

The `[OK]` at the end means success!

### Step 5: Note the Binary Size

The "Section sizes" line tells you how big your program is:

| Section | Description |
|---------|-------------|
| text | Code (instructions) |
| data | Initialized variables |
| bss | Uninitialized variables |
| dec | Total size in decimal bytes |

V5 Brain has ~32MB of storage, so you have plenty of room.

---

## Uploading to the Robot

### Step 1: Connect the Robot

1. Turn on the V5 Brain (press power button)
2. Wait for it to fully boot (shows home screen)
3. Connect USB cable from brain to computer

### Step 2: Verify Connection
```bash
pros lsusb
```

**Expected output:**
```
VEX V5 Brain (USB) - /dev/ttyACM0
```

If nothing shows up:
- Try a different USB cable
- Try a different USB port
- Check that brain is fully powered on
- On Linux, check udev rules (see [Installation](INSTALLATION.md))

### Step 3: Upload
```bash
pros upload
```

**Expected output:**
```
Uploading bin/output.bin to V5...
[████████████████████████████████] 100%
Upload successful.
```

### Step 4: Watch the Brain Screen

During upload, the brain screen shows:
1. "Receiving..." with progress bar
2. "Installing..."
3. Returns to program list

Your program should now appear in the program list on the brain.

---

## Verifying the Upload

### Step 1: Start Terminal
```bash
pros terminal
```

This opens a serial connection to see robot output.

**Expected output:**
```
         _+=+_
      .-`  .  `-.          PROS
   ...
Version:        4.1.0     Platform:  V1.1.5

Initializing logger...
{"messages": [{ "message": "Logger initialized!", "type": "success" }]}
{"messages": [{ "message": "Initializing TestBot", "type": "log" }]}
...
{"messages": [{ "message": "Ready!", "type": "log" }]}

=== OPCONTROL STARTED ===
Press A to run autonomous
Press B to print position
```

### Step 2: Test Basic Functionality

1. **Turn on the controller** and wait for it to connect
2. **Move the joysticks** - robot should respond
3. **Press B** - should print position to terminal
4. **Press A** - should run autonomous

### Step 3: Check for Issues

| Symptom | Likely Cause | See |
|---------|--------------|-----|
| No terminal output | Wrong baud rate or connection issue | Reconnect USB |
| "Logger initialized" but nothing else | Crash during init | Check motor ports |
| Motors make noise but don't move | Motor signs wrong | [Motors Fighting](../troubleshooting/MOTORS_FIGHTING.md) |
| Robot drives but curves | Odometry misconfigured | [Odometry Drift](../troubleshooting/ODOMETRY_DRIFT.md) |

### Step 4: Exit Terminal

Press `Ctrl+C` to exit the terminal and return to command prompt.

---

## Build Commands Reference

### Basic Commands

| Command | Description |
|---------|-------------|
| `pros make` | Build the project |
| `pros make clean` | Delete all build files |
| `pros make clean all` | Clean then build |
| `pros upload` | Upload to connected V5 Brain |
| `pros terminal` | Open serial terminal |
| `pros lsusb` | List connected V5 devices |

### Combined Commands
```bash
# Clean, build, and upload in one line
pros make clean && pros make && pros upload

# Build and upload
pros make && pros upload

# Upload and immediately open terminal
pros upload && pros terminal
```

### Useful Flags
```bash
# Upload to specific slot (1-8)
pros upload --slot 2

# Upload with specific program name
pros upload --name "TestBot Auton"

# Build with verbose output (shows all compiler commands)
pros make --verbose

# Build specific target
pros make quick    # Faster build, less optimization
```

---

## Common Build Errors

### "No rule to make target"

**Cause:** A source file is missing or misnamed.

**Fix:** Check that all files in `src/` exist and are spelled correctly.
```bash
# List all source files
find src -name "*.cpp"
```

---

### "undefined reference to..."

**Cause:** A function is declared but not implemented, or not linked.

**Example:**
```
undefined reference to `shulib::Chassis::drive(int, int, int, bool)'
```

**Fix:** 
1. Check that the function is implemented in a `.cpp` file
2. Check that the `.cpp` file is being compiled (in `src/` directory)
3. Check spelling matches exactly

---

### "no matching function for call to..."

**Cause:** Function call doesn't match any declaration.

**Example:**
```
no matching function for call to 'rotateTo(Chassis&, int, int)'
```

**Fix:** Check the function signature matches what you're calling:
```cpp
// Declaration (in .hpp)
void rotateTo(Chassis& chassis, double angle);

// Call (must match)
rotateTo(chassis, 90);      // ✓ Correct
rotateTo(chassis, 90, 50);  // ✗ Too many arguments
```

---

### "config.hpp: No such file or directory"

**Cause:** Can't find the config file.

**Fix:** 
1. Make sure `config.hpp` exists in project root
2. Make sure you're building from the project root directory
3. Check the include path in the file that's failing

---

### "'ROBOT_TESTBOT' was not declared"

**Cause:** Robot isn't defined in `config.hpp`.

**Fix:** Make sure `config.hpp` has your robot uncommented:
```cpp
#define ROBOT_TESTBOT    // Not commented out
```

---

### "multiple definition of..."

**Cause:** Same function or variable defined in multiple files.

**Fix:**
1. Functions should only be defined in ONE `.cpp` file
2. Use `inline` for functions in headers
3. Use `extern` for global variables in headers

---

### Build Succeeds but Upload Fails

| Error | Cause | Fix |
|-------|-------|-----|
| "No V5 devices found" | Brain not connected | Check USB cable and connection |
| "Permission denied" | No access to USB device | Add udev rules (Linux) |
| "Device busy" | Another program using port | Close other terminals/programs |
| "Upload failed" | Communication error | Retry, try different USB port |

---

## Next Steps

Now that you've built and uploaded successfully:

- [Quick Start](QUICK_START.md) – Test driving and autonomous
- [Project Structure](PROJECT_STRUCTURE.md) – Understand the code layout
- [Adding a Robot](../configuration/ADDING_A_ROBOT.md) – Configure your specific robot