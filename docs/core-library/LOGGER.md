# Logger

**Location:** `include/shulib/core/logger.hpp` and `src/core/logger.cpp`

---

## Table of Contents

1. [What is the Logger?](#what-is-the-logger)
2. [Why Logging Matters](#why-logging-matters)
3. [Two Types of Output](#two-types-of-output)
4. [Architecture](#architecture)
5. [The Logger Class](#the-logger-class)
6. [Debug Messages](#debug-messages)
7. [Telemetry](#telemetry)
8. [Viewing Output](#viewing-output)
9. [API Reference](#api-reference)
10. [Common Patterns](#common-patterns)
11. [Best Practices](#best-practices)
12. [Debugging the Debugger](#debugging-the-debugger)
13. [Key Takeaways](#key-takeaways)
14. [Where to Go Next](#where-to-go-next)

---

## What is the Logger?

### The Simple Explanation

The **Logger** is shulib's system for getting information out of the robot. When your robot is running autonomously, you can't see what's happening inside the code. The Logger lets you:

1. **Print debug messages** - "Starting autonomous", "Error: motor disconnected"
2. **Stream telemetry data** - Position, battery voltage, motor temperatures

Think of it as the robot's way of talking to you.

### The Problem It Solves

Without logging:
```cpp
// Robot does something wrong...
// You have no idea what happened.
// Was it the sensors? The motors? The logic?
// 🤷
```

With logging:
```cpp
logger().log("Starting movement to (24, 36)");
logger().log("Current pose: " + std::to_string(pose.x) + ", " + std::to_string(pose.y));
logger().error("Timeout! Only reached " + std::to_string(distance) + " inches");
// Now you know exactly what went wrong!
```

### What Makes It Special

Our Logger isn't just `printf()`. It has:

| Feature | Benefit |
|---------|---------|
| **Message types** | Color-coded errors, warnings, successes |
| **Telemetry streaming** | Continuous data for real-time monitoring |
| **Background task** | Non-blocking, won't slow your code |
| **JSON output** | Easy to parse with external tools |
| **Chunked transmission** | Handles large data without overflow |
| **Singleton pattern** | One global logger, accessible anywhere |

---

## Why Logging Matters

### During Development

Logging helps you understand what your code is doing:

```cpp
void moveToTarget(Pose target) {
    logger().log("moveToTarget called with target: ", target.x, ", ", target.y);
    
    Pose current = chassis.getPose();
    logger().log("Current position: ", current.x, ", ", current.y);
    
    float distance = current.distance(target);
    logger().log("Distance to target: ", distance);
    
    // ... movement code ...
    
    logger().success("Reached target!");
}
```

Output:
```
[LOG] moveToTarget called with target: 24, 36
[LOG] Current position: 0.5, 0.3
[LOG] Distance to target: 42.7
[SUCCESS] Reached target!
```

### During Competition

At competition, you can't modify code between matches. But you CAN read logs to understand what happened:

```
[LOG] Autonomous started
[LOG] Moving to scoring zone
[WARNING] Motor 3 temperature high: 52°C
[ERROR] Timeout waiting for position!
[LOG] Autonomous ended
```

Now you know: Motor 3 was overheating, which slowed the robot, causing a timeout.

### For Data Analysis

Telemetry data can be captured and analyzed:

```json
{"odometry": {"x": 12.5, "y": 24.3, "theta": 0.789}}
{"odometry": {"x": 12.8, "y": 24.9, "theta": 0.791}}
{"odometry": {"x": 13.1, "y": 25.5, "theta": 0.793}}
```

Plot this data to visualize robot paths, identify drift, tune PID, etc.

---

## Two Types of Output

### 1. Debug Messages

One-time messages about events:

```cpp
logger().log("Starting calibration");
logger().success("Calibration complete!");
logger().warning("Battery below 50%");
logger().error("Sensor disconnected!");
```

**Characteristics:**
- Triggered by specific events
- Different severity levels (log, warning, error, success)
- Printed immediately (well, next update cycle)
- Human-readable

### 2. Telemetry Data

Continuous streams of values:

```cpp
logger().updateTelemetry("odometry", pose);
logger().updateTelemetry("battery_voltage", voltage);
logger().updateTelemetry("motor_temps", tempMap);
```

**Characteristics:**
- Updated continuously (every 10ms typically)
- Only sent when values change
- JSON format for machine parsing
- Used for monitoring and analysis

### When to Use Which

| Situation | Use |
|-----------|-----|
| Something happened (event) | Debug message |
| Continuous state monitoring | Telemetry |
| Errors and warnings | Debug message |
| Position tracking | Telemetry |
| "I'm here in the code" | Debug message |
| Sensor values over time | Telemetry |

---

## Architecture

### The Singleton Pattern

There's only ONE Logger instance in the entire program:

```cpp
class Logger {
public:
    // Get the one and only instance
    static Logger& getInstance() {
        static Logger instance;  // Created once, lives forever
        return instance;
    }
    
private:
    // Private constructor - can't create more instances
    Logger() { /* ... */ }
    
    // Disable copying
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
};
```

Access it with the global helper:
```cpp
shulib::logger()  // Returns the singleton instance
```

### The Background Task

The Logger runs its own PROS task for sending data:

```cpp
void Logger::init() {
    if (telemetryTask == nullptr) {
        telemetryTask = new pros::Task([this] {
            while (true) {
                this->sendTelemetry();  // Send pending data
                pros::delay(100);        // Every 100ms
            }
        });
    }
}
```

This means logging is **non-blocking** - your code doesn't wait for data to transmit.

### Data Flow

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         LOGGER DATA FLOW                                │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   Your Code                                                             │
│      │                                                                  │
│      │  logger().log("Hello")                                          │
│      │  logger().updateTelemetry("x", 12.5)                            │
│      ▼                                                                  │
│   ┌─────────────────┐                                                   │
│   │  Debug Messages │──────┐                                            │
│   │  Queue          │      │                                            │
│   └─────────────────┘      │                                            │
│                            │                                            │
│   ┌─────────────────┐      │      ┌─────────────────┐                  │
│   │  Telemetry Data │──────┼─────►│  Background     │                  │
│   │  Map            │      │      │  Task           │                  │
│   └─────────────────┘      │      │  (every 100ms)  │                  │
│                            │      └────────┬────────┘                  │
│                            │               │                            │
│                            │               ▼                            │
│                            │      ┌─────────────────┐                  │
│                            └─────►│  printf()       │                  │
│                                   │  (serial out)   │                  │
│                                   └────────┬────────┘                  │
│                                            │                            │
│                                            ▼                            │
│                                   ┌─────────────────┐                  │
│                                   │  pros terminal  │                  │
│                                   │  (your laptop)  │                  │
│                                   └─────────────────┘                  │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### Thread Safety

Multiple tasks might log simultaneously. The Logger uses a mutex to prevent corruption:

```cpp
void Logger::updateTelemetry(const std::string& key, const std::string& value) {
    mutex.take();              // Lock
    telemetryData[key] = value;
    mutex.give();              // Unlock
}
```

---

## The Logger Class

### Key Components

```cpp
class Logger {
public:
    // Singleton access
    static Logger& getInstance();
    
    // Initialization
    void init();
    
    // Debug messages
    void log(const std::string& message);
    void error(const std::string& message);
    void warning(const std::string& message);
    void success(const std::string& message);
    void announce(const std::string& message);
    void debug(const std::string& message);
    
    // Telemetry
    template<typename T>
    void updateTelemetry(const std::string& key, const T& value);
    
    // Configuration
    void setTelemetryInterval(int ms);

private:
    // Message types
    enum class MessageType { LOG, ERROR, WARNING, SUCCESS, ANNOUNCE, DEBUG };
    
    // Storage
    std::unordered_map<std::string, std::string> telemetryData;
    std::vector<std::pair<std::string, MessageType>> debugMessages;
    
    // Synchronization
    pros::Mutex mutex;
    
    // Background task
    pros::Task* telemetryTask;
    
    // Timing
    uint32_t lastTelemetryTime;
    int telemetryInterval;
};
```

### Message Types

| Type | Method | Use Case | Visual |
|------|--------|----------|--------|
| LOG | `log()` | General information | `[LOG]` |
| ERROR | `error()` | Something went wrong | `[ERROR]` |
| WARNING | `warning()` | Potential problem | `[WARNING]` |
| SUCCESS | `success()` | Something worked | `[SUCCESS]` |
| ANNOUNCE | `announce()` | Important milestone | `[ANNOUNCE]` |
| DEBUG | `debug()` | Detailed debugging info | `[DEBUG]` |

---

## Debug Messages

### Basic Usage

```cpp
// Simple messages
logger().log("Robot initialized");
logger().error("Motor 5 disconnected!");
logger().warning("Battery at 30%");
logger().success("Autonomous complete");
```

### With Variables (Variadic Templates)

The Logger supports multiple arguments that get concatenated:

```cpp
// Multiple arguments
logger().log("Position: ", x, ", ", y);
logger().error("Timeout after ", elapsed, "ms");
logger().warning("Motor ", motorId, " temp: ", temp, "°C");
```

This works because of variadic templates:

```cpp
template<typename... Args>
void log(const Args&... args) {
    std::stringstream ss;
    (ss << ... << args);  // Fold expression - concatenates all args
    log(ss.str());
}
```

### When Messages Are Sent

Messages aren't sent immediately. They're queued and sent in batches:

```cpp
logger().log("Message 1");  // Queued
logger().log("Message 2");  // Queued
logger().log("Message 3");  // Queued
// ... 100ms later, background task sends all three
```

This batching improves performance and prevents serial buffer overflow.

### Output Format

Debug messages are output as JSON:

```json
{"messages": [
    {"message": "Robot initialized", "type": "log"},
    {"message": "Calibration complete!", "type": "success"}
]}
```

---

## Telemetry

### Basic Usage

```cpp
// Update a single value
logger().updateTelemetry("x_position", 12.5);
logger().updateTelemetry("battery", 85);
logger().updateTelemetry("state", "autonomous");
```

### Pose Telemetry

Special handling for Pose objects:

```cpp
Pose robotPose(12.5, 24.3, 45.0);
logger().updateTelemetry("odometry", robotPose);
// Output: {"odometry": {"x": 12.5, "y": 24.3, "theta": 45.0}}
```

### Map Telemetry

For motor temperatures and similar data:

```cpp
std::map<std::string, double> temps = {
    {"motor_1", 45.2},
    {"motor_2", 48.7},
    {"motor_3", 52.1}
};
logger().updateTelemetry("temps", temps);
// Output: {"temps": {"motor_1": 45.2, "motor_2": 48.7, "motor_3": 52.1}}
```

### Change Detection

Telemetry is only sent when values actually change:

```cpp
// These only send once:
logger().updateTelemetry("x", 10.0);
logger().updateTelemetry("x", 10.0);  // Same value - not sent again
logger().updateTelemetry("x", 10.0);  // Same value - not sent again
logger().updateTelemetry("x", 10.5);  // Changed! This one is sent.
```

This reduces bandwidth and makes output cleaner.

### Automatic Telemetry

The odometry system automatically logs certain telemetry:

```cpp
// In odometry.cpp - happens automatically every update
logger().updateTelemetry("odometry", odomPose);
logger().updateTelemetry("temps", drive.getTemps());
logger().updateTelemetry("battery", batteryTelemetry);
logger().updateTelemetry("controller", controllerTelemetry);
```

You get this for free without writing any logging code!

---

## Viewing Output

### Using PROS Terminal

The primary way to view logs:

```bash
# In your terminal, with robot connected via USB or WiFi
pros terminal
```

You'll see output like:
```
{"messages": [{"message": "Logger initialized!", "type": "success"}]}
{"odometry": {"x": 0.00, "y": 0.00, "theta": 0.00}}
{"battery": {"voltage": 12500, "current": 0, "temperature": 25, "capacity": 100}}
```

### Filtering Output

The output can be verbose. Use grep to filter:

```bash
# Only errors
pros terminal | grep "error"

# Only odometry
pros terminal | grep "odometry"

# Only messages (not telemetry)
pros terminal | grep "messages"
```

### Pretty Printing JSON

For readability:

```bash
# Using jq (install with: sudo apt install jq)
pros terminal | jq .

# Output becomes:
{
  "odometry": {
    "x": 12.5,
    "y": 24.3,
    "theta": 0.789
  }
}
```

### Logging to File

Capture logs for later analysis:

```bash
# Save to file
pros terminal > match_log.txt

# Save with timestamp
pros terminal > "log_$(date +%Y%m%d_%H%M%S).txt"
```

### Brain Screen (Alternative)

For quick checks without a laptop:

```cpp
// In your code
pros::lcd::initialize();
pros::lcd::print(0, "X: %.2f", chassis.getPose().x);
pros::lcd::print(1, "Y: %.2f", chassis.getPose().y);
```

But this is limited to 8 lines and no scrolling. Logger is better for detailed debugging.

---

## API Reference

### Initialization

#### `init()`

```cpp
void init()
```

Initializes the Logger and starts the background telemetry task.

**Call once** at the start of your program:

```cpp
void initialize() {
    logger().init();  // Start the logger
    // ... rest of initialization
}
```

---

### Debug Message Methods

#### `log(message)` / `log(args...)`

```cpp
void log(const std::string& message)

template<typename... Args>
void log(const Args&... args)
```

Logs a general informational message.

```cpp
logger().log("Starting autonomous");
logger().log("Position: ", x, ", ", y);
```

---

#### `error(message)` / `error(args...)`

```cpp
void error(const std::string& message)

template<typename... Args>
void error(const Args&... args)
```

Logs an error message. Use for things that went wrong.

```cpp
logger().error("Motor disconnected!");
logger().error("Timeout after ", elapsed, "ms");
```

---

#### `warning(message)` / `warning(args...)`

```cpp
void warning(const std::string& message)

template<typename... Args>
void warning(const Args&... args)
```

Logs a warning message. Use for potential problems.

```cpp
logger().warning("Battery below 50%");
logger().warning("Motor ", id, " temperature high: ", temp);
```

---

#### `success(message)` / `success(args...)`

```cpp
void success(const std::string& message)

template<typename... Args>
void success(const Args&... args)
```

Logs a success message. Use when something completed successfully.

```cpp
logger().success("Calibration complete!");
logger().success("Reached target in ", time, "ms");
```

---

#### `announce(message)` / `announce(args...)`

```cpp
void announce(const std::string& message)

template<typename... Args>
void announce(const Args&... args)
```

Logs an announcement. Use for important milestones.

```cpp
logger().announce("=== AUTONOMOUS STARTED ===");
logger().announce("Match ", matchNum, " beginning");
```

---

#### `debug(message)` / `debug(args...)`

```cpp
void debug(const std::string& message)

template<typename... Args>
void debug(const Args&... args)
```

Logs a debug message. Use for detailed debugging information.

```cpp
logger().debug("PID output: ", output);
logger().debug("Loop iteration ", i, ", error = ", error);
```

---

### Telemetry Methods

#### `updateTelemetry(key, value)`

```cpp
template<typename T>
void updateTelemetry(const std::string& key, const T& value)
```

Updates a telemetry value. Specialized for different types.

**For numbers:**
```cpp
logger().updateTelemetry("speed", 45.5);
logger().updateTelemetry("count", 10);
```

**For strings:**
```cpp
logger().updateTelemetry("state", "autonomous");
```

**For Pose:**
```cpp
logger().updateTelemetry("position", pose);
// Output: {"position": {"x": ..., "y": ..., "theta": ...}}
```

**For maps:**
```cpp
std::map<std::string, double> data = {{"a", 1.0}, {"b", 2.0}};
logger().updateTelemetry("data", data);
// Output: {"data": {"a": 1.0, "b": 2.0}}
```

---

### Configuration

#### `setTelemetryInterval(ms)`

```cpp
void setTelemetryInterval(int ms)
```

Sets how often telemetry is sent (default: 250ms).

```cpp
logger().setTelemetryInterval(100);  // Send every 100ms (faster)
logger().setTelemetryInterval(500);  // Send every 500ms (slower)
```

**Trade-offs:**
- Faster = more data, higher bandwidth, more responsive
- Slower = less data, lower bandwidth, less responsive

---

### Global Accessor

#### `logger()`

```cpp
inline Logger& logger() { return Logger::getInstance(); }
```

Returns the singleton Logger instance. Use this everywhere:

```cpp
shulib::logger().log("Hello!");
// Or with 'using namespace shulib':
logger().log("Hello!");
```

---

## Common Patterns

### Pattern 1: Function Entry/Exit Logging

```cpp
void complexFunction() {
    logger().debug("Entering complexFunction");
    
    // ... do stuff ...
    
    logger().debug("Exiting complexFunction");
}
```

### Pattern 2: Conditional Logging

```cpp
void checkMotorHealth(int motorId, double temp) {
    if (temp > 55) {
        logger().error("Motor ", motorId, " OVERHEATING: ", temp, "°C");
    } else if (temp > 45) {
        logger().warning("Motor ", motorId, " warm: ", temp, "°C");
    } else {
        logger().debug("Motor ", motorId, " temp OK: ", temp, "°C");
    }
}
```

### Pattern 3: Periodic Status Updates

```cpp
void autonomousTask() {
    int loopCount = 0;
    
    while (running) {
        // Log status every 50 iterations (500ms at 10ms loop)
        if (loopCount % 50 == 0) {
            Pose p = chassis.getPose();
            logger().log("Status: pos=(", p.x, ",", p.y, ") θ=", p.theta);
        }
        
        loopCount++;
        pros::delay(10);
    }
}
```

### Pattern 4: Error Context

```cpp
void moveToPosition(Pose target) {
    logger().log("Attempting move to (", target.x, ", ", target.y, ")");
    
    Pose start = chassis.getPose();
    int elapsed = 0;
    
    while (elapsed < TIMEOUT) {
        Pose current = chassis.getPose();
        
        if (current.distance(target) < TOLERANCE) {
            logger().success("Reached target in ", elapsed, "ms");
            return;
        }
        
        elapsed += 10;
        pros::delay(10);
    }
    
    // If we get here, we timed out
    Pose final = chassis.getPose();
    logger().error("TIMEOUT! Started at (", start.x, ",", start.y, 
                   "), ended at (", final.x, ",", final.y,
                   "), target was (", target.x, ",", target.y, ")");
}
```

### Pattern 5: State Machine Logging

```cpp
enum class RobotState { IDLE, MOVING, SCORING, RETURNING };

void setState(RobotState newState) {
    static RobotState currentState = RobotState::IDLE;
    
    if (newState != currentState) {
        logger().announce("State change: ", 
                         stateToString(currentState), " → ", 
                         stateToString(newState));
        currentState = newState;
    }
}
```

### Pattern 6: Telemetry Dashboard Data

```cpp
void updateDashboard() {
    // Position
    Pose p = chassis.getPose();
    logger().updateTelemetry("pose", p);
    
    // Battery
    logger().updateTelemetry("battery_pct", pros::battery::get_capacity());
    logger().updateTelemetry("battery_v", pros::battery::get_voltage());
    
    // Motors
    logger().updateTelemetry("motor_temps", drivetrain.getTemps());
    
    // Match info
    logger().updateTelemetry("match_time", pros::competition::get_time());
    logger().updateTelemetry("is_autonomous", pros::competition::is_autonomous());
}
```

---

## Best Practices

### DO: Use Appropriate Message Types

```cpp
// Good - clear severity levels
logger().error("Critical failure!");
logger().warning("Something might be wrong");
logger().log("Normal operation");
logger().success("Task completed");

// Bad - everything is log()
logger().log("ERROR: Critical failure!");  // Use error() instead!
```

### DO: Include Context

```cpp
// Good - actionable information
logger().error("Motor ", motorId, " on port ", port, 
               " disconnected during ", currentOperation);

// Bad - vague
logger().error("Motor error");
```

### DO: Log State Changes

```cpp
// Good - know what's happening
logger().log("Autonomous phase: ", phase);
logger().log("Target changed to: ", newTarget.x, ", ", newTarget.y);

// Bad - silent operation
// (nothing logged, no idea what robot is doing)
```

### DON'T: Log Too Much

```cpp
// Bad - logs every 10ms loop iteration
while (running) {
    logger().log("Loop running...");  // 100 messages per second!
    pros::delay(10);
}

// Good - log periodically or on events
int count = 0;
while (running) {
    if (count++ % 100 == 0) {  // Every 1 second
        logger().log("Still running, iteration ", count);
    }
    pros::delay(10);
}
```

### DON'T: Log Sensitive Computations in Tight Loops

```cpp
// Bad - string formatting in hot loop
for (int i = 0; i < 1000000; i++) {
    logger().debug("i = ", i);  // Slow!
}

// Good - log summary
logger().log("Starting 1M iterations");
for (int i = 0; i < 1000000; i++) {
    // ... work ...
}
logger().log("Completed 1M iterations");
```

### DON'T: Forget to Initialize

```cpp
// Bad - logging before init
void initialize() {
    logger().log("Starting...");  // Might not work!
    logger().init();              // Too late!
}

// Good - init first
void initialize() {
    logger().init();              // Initialize first
    logger().log("Starting...");  // Now it works
}
```

---

## Debugging the Debugger

### Problem: No Output

**Symptoms:** `pros terminal` shows nothing

**Checklist:**
```
□ Did you call logger().init()?
□ Is the robot connected? (pros terminal should say "Connected")
□ Is something else using the serial port?
□ Try: pros terminal --raw
```

### Problem: Garbled Output

**Symptoms:** Random characters, incomplete JSON

**Possible causes:**
1. Baud rate mismatch (shouldn't happen with PROS)
2. Buffer overflow from too much data
3. Multiple things printing simultaneously

**Solutions:**
```cpp
// Reduce telemetry rate
logger().setTelemetryInterval(500);  // Slower

// Reduce logging verbosity
// Remove debug() calls in hot paths
```

### Problem: Messages Delayed

**Symptoms:** Messages appear long after the event

**This is normal!** Messages are batched and sent every 100ms. For time-critical debugging:

```cpp
// Force immediate output (bypasses logger)
printf("IMMEDIATE: %f\n", value);
fflush(stdout);
```

### Problem: Telemetry Not Updating

**Symptoms:** Same values shown repeatedly

**Remember:** Telemetry only sends when values CHANGE. If position isn't moving, it won't re-send.

**To verify it's working:**
```cpp
// Add a counter that always changes
static int counter = 0;
logger().updateTelemetry("heartbeat", counter++);
```

---

## Key Takeaways

### The Essentials

1. **Initialize first:** Call `logger().init()` at the start of your program

2. **Two types of output:**
   - Debug messages: Events, errors, warnings
   - Telemetry: Continuous data streams

3. **Use appropriate severity levels:**
   - `error()` - Something broke
   - `warning()` - Something might be wrong
   - `log()` - General information
   - `success()` - Something worked
   - `debug()` - Detailed debugging

4. **Telemetry is smart:**
   - Only sends when values change
   - Handles Pose and map types automatically
   - JSON format for easy parsing

5. **Non-blocking:** Logger runs in background, won't slow your code

6. **View with:** `pros terminal`

### The One-Sentence Summary

> **The Logger provides thread-safe debug messaging and telemetry streaming, letting you see what's happening inside your robot through the PROS terminal.**

---

## Where to Go Next

| Topic | Document | What You'll Learn |
|-------|----------|-------------------|
| Telemetry in odometry | [ODOMETRY.md](./ODOMETRY.md) | What's automatically logged |
| Debugging motion | [PID_TUNING.md](../motion/PID_TUNING.md) | Using logs to tune PID |
| Competition debugging | [QUICK_REFERENCE.md](../competition/QUICK_REFERENCE.md) | Logging at competition |
| Common issues | [COMMON_ISSUES.md](../troubleshooting/COMMON_ISSUES.md) | Using logs to diagnose problems |

### Related Code Files

```
include/shulib/core/logger.hpp  ← Class definition
src/core/logger.cpp             ← Implementation
src/core/odometry.cpp           ← Example telemetry usage
src/main.cpp                    ← Logger initialization
```

### External Tools

- **PROS Terminal:** Built-in, just run `pros terminal`
- **jq:** JSON processor for filtering logs (`apt install jq`)
- **Custom dashboards:** Parse JSON telemetry with Python, Node.js, etc.

---

*Document last updated: January 2026*