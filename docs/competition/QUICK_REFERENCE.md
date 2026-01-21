# Quick Reference

**Cheat sheet for the pits**

---

## Commands

```bash
# Build
pros make

# Clean build
pros make clean && pros make

# Upload to robot
pros upload

# Build and upload
pros mu

# Terminal output
pros terminal

# Check PROS version
pros --version
```

---

## Switching Robots

Edit `config.hpp`:
```cpp
#define ROBOT_XEBEC
// #define ROBOT_QUEENS_REVENGE
```

Then: `pros make clean && pros make`

---

## Switching Autonomous

Edit `config.hpp`:
```cpp
// #define AUTON_SKILLS
#define AUTON_RED_LEFT
// #define AUTON_RED_RIGHT
// #define AUTON_BLUE_LEFT
// #define AUTON_BLUE_RIGHT
// #define AUTON_TEST
```

Then: `pros make clean && pros make`

---

## Controls (Push Back)

```
┌───────────────────────────────────────┐
│  L1: Intake OUT    R1: Intake IN      │
│  L2: Release BACK  R2: Release FWD    │
│                                       │
│  LEFT STICK Y: Drive fwd/back         │
│  RIGHT STICK X: Turn                  │
│                                       │
│  Y: Toggle Arm                        │
│  LEFT: Toggle Lever                   │
└───────────────────────────────────────┘
```

---

## Port Reference

### XEBEC

| System | Ports |
|--------|-------|
| Left Drive | 12, -14, 16, -18, 20 |
| Right Drive | 11, -13, 15, -17, 19 |
| Tracking L/R/B | -8, 10, 9 |
| Intake | -6, 7 |
| Conveyor | 2, -3, -4, 5 |
| Releaser | 1 |
| Arm | ADI B |
| Lever | ADI C |

### Queens Revenge

| System | Ports |
|--------|-------|
| Left Drive | 11, -12, 13, -14, -15 |
| Right Drive | 16, -17, 18, -19, 20 |
| Tracking L/R/B | -8, 10, 9 |
| Intake | 2, -3 |
| Conveyor | 4, -5 |
| Releaser | -6, 7 |
| Arm | ADI B |
| Lever | ADI C |

---

## Scoring (Push Back)

| Action | Points |
|--------|--------|
| Block in goal | 3 |
| Goal control | 10 |
| 1 robot parked | 8 |
| 2 robots parked | **30** |

**Remember:** 2 robots parked (30 pts) = 10 blocks!

---

## Match Timing

| Period | Duration |
|--------|----------|
| Autonomous | 0:15 |
| Driver | 1:45 |
| **Total** | **2:00** |

### Parking Timeline
- 20 sec left → Think about parking
- 15 sec left → Move to parking
- 10 sec left → First robot parked
- 5 sec left → Both parked
- 0 sec → **DON'T MOVE**

---

## Coordinate System

```
     +Y (forward)
        ↑
        │
  -X ←──┼──→ +X
        │
        ↓
     -Y (back)

Heading: 0° = forward, 90° = right
```

---

## Motion Functions

```cpp
// Drive straight
moveVertical(chassis, 24);     // 24" forward
moveVertical(chassis, -12);    // 12" backward

// Turn to angle
rotateTo(chassis, 90);         // Face right
rotateTo(chassis, -90);        // Face left

// Go to position
moveToPose(chassis, Pose(24, 36, 90));
```

---

## Mechanism Control

```cpp
mech.setIntake(127);       // Intake full forward
mech.setIntake(-127);      // Intake reverse
mech.setIntake(0);         // Stop

mech.intakeAndConvey(127); // Both together

mech.setReleaser(127);     // Score
mech.toggleArm();          // Toggle arm
mech.toggleLever();        // Toggle lever
```

---

## Common Issues

| Problem | Quick Fix |
|---------|-----------|
| Won't build | `pros make clean && pros make` |
| Won't upload | Restart brain, check USB |
| Motors fight | Check port signs (- for reversed) |
| Robot drifts | Check tracking wheel connections |
| Oscillates | Lower PID P value |
| No autonomous | Check competition mode |

---

## Emergency Code Reset

```bash
# Discard all changes
git checkout .

# Or revert specific file
git checkout HEAD -- src/main.cpp
```

---

## Motor Test

```cpp
void testMotor(int port) {
    pros::Motor m(port);
    m.move(50);
    pros::delay(1000);
    m.move(0);
}
```

---

## Quick Debug

```cpp
// Print pose
Pose p = chassis.getPose();
printf("(%.1f, %.1f, %.1f)\n", p.x, p.y, p.theta);

// Print to controller
controller.print(0, 0, "X: %.1f", p.x);
```

---

## Tuned PID Values

### moveVertical
```cpp
PID(10, 2.5, 0.3)
MAX_OUTPUT = 60
```

### rotateTo
```cpp
PID(1.5, 0.01, 0.15)
MAX_ROTATION = 50
```

---

## Files to Know

| File | Purpose |
|------|---------|
| `config.hpp` | Robot & auton selection |
| `src/main.cpp` | Entry points |
| `src/seasons/pushback_2026/auton.cpp` | Autonomous routines |
| `src/seasons/pushback_2026/opcontrol.cpp` | Driver controls |
| `include/shulib/robots/xebec.hpp` | XEBEC config |
| `include/shulib/robots/queens_revenge.hpp` | QR config |

---

## Phone Numbers

*(Fill in before competition)*

```
Team Lead: _______________
Mentor: __________________
Emergency: _______________
```

---

## Notes

*(Use for match notes)*

```
________________________________________
________________________________________
________________________________________
________________________________________
________________________________________
________________________________________
________________________________________
________________________________________
```

---

*Full documentation: See project docs folder*