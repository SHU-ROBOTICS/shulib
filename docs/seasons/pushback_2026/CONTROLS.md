# Push Back Controls

**Button mappings for the 2025-2026 season**

---

## Controller Layout

```
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│      L1                                               R1            │
│   ┌──────┐                                         ┌──────┐         │
│   │INTAKE│                                         │INTAKE│         │
│   │ OUT  │                                         │  IN  │         │
│   └──────┘                                         └──────┘         │
│      L2                                               R2            │
│   ┌──────┐                                         ┌──────┐         │
│   │RELEAS│                                         │RELEAS│         │
│   │ BACK │                                         │ FWD  │         │
│   └──────┘                                         └──────┘         │
│                                                                     │
│                                                                     │
│   ┌───────────┐        ┌───┐ ┌───┐        ┌───────────┐            │
│   │           │        │UP │ │ Y │        │           │            │
│   │   LEFT    │        ├───┤ ├───┤        │   RIGHT   │            │
│   │   STICK   │   ┌───┐│LFT│ │ A │┌───┐   │   STICK   │            │
│   │           │   │LVR││───│ │───││   │   │           │            │
│   │  DRIVE    │   └───┘│DWN│ │ B │└───┘   │   TURN    │            │
│   │           │        └───┘ └───┘        │           │            │
│   └───────────┘          ↑                └───────────┘            │
│                        LEVER                                        │
│                        TOGGLE    ARM                                │
│                                TOGGLE                               │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Control Summary

| Button | Action | Type |
|--------|--------|------|
| **Left Stick Y** | Drive forward/backward | Analog |
| **Right Stick X** | Turn left/right | Analog |
| **R1** | Intake + Conveyor IN | Hold |
| **L1** | Intake + Conveyor OUT | Hold |
| **R2** | Releaser FORWARD | Hold |
| **L2** | Releaser BACKWARD | Hold |
| **Y** | Toggle Arm | Press |
| **LEFT** | Toggle Lever | Press |

---

## Detailed Controls

### Drivetrain

| Input | Action | Notes |
|-------|--------|-------|
| Left Stick Y | Forward/Backward | Push up = forward |
| Right Stick X | Turn | Push right = turn right |

**Drive Style:** Split arcade (tank-style with arcade turn)

### Intake System

| Button | Action | Behavior |
|--------|--------|----------|
| R1 (hold) | Intake + Conveyor IN | Runs while held |
| L1 (hold) | Intake + Conveyor OUT | Reverses while held |
| Neither | Stop | Motors stop |

**Note:** R1/L1 control BOTH intake and conveyor together.

### Releaser

| Button | Action | Behavior |
|--------|--------|----------|
| R2 (hold) | Releaser Forward | Scores blocks |
| L2 (hold) | Releaser Backward | Reverses |
| Neither | Stop | Motor stops |

### Pneumatics

| Button | Action | Behavior |
|--------|--------|----------|
| Y (press) | Toggle Arm | Extends/retracts arm |
| LEFT (press) | Toggle Lever | Extends/retracts lever |

**Note:** Press once to extend, press again to retract.

---

## Robot-Specific Notes

### XEBEC

Standard controls as documented above.

### Queens Revenge

Same control scheme, but:
- Lever behavior may differ slightly
- Test before competition

---

## Control Rationale

### Why This Layout?

**R1/L1 for Intake:**
- Primary action, easy access
- Trigger-style hold makes sense
- Reversible for jams

**R2/L2 for Releaser:**
- Secondary action
- Below intake triggers
- Natural finger position

**Y for Arm:**
- Not frequently used
- Single press toggle
- Out of the way

**LEFT for Lever:**
- Rarely used
- D-pad accessible
- Won't accidentally press

### Driver Preferences

Current layout optimized for:
- Right-hand dominant driver
- Primary focus on intake/score cycle
- Quick access to most-used controls

---

## Quick Reference Card

Print this and tape to controller:

```
╔═══════════════════════════════════════╗
║          PUSH BACK CONTROLS           ║
╠═══════════════════════════════════════╣
║  LEFT STICK Y    Drive fwd/back       ║
║  RIGHT STICK X   Turn                 ║
╠═══════════════════════════════════════╣
║  R1 (hold)       Intake IN            ║
║  L1 (hold)       Intake OUT           ║
║  R2 (hold)       Releaser FWD         ║
║  L2 (hold)       Releaser BACK        ║
╠═══════════════════════════════════════╣
║  Y (press)       Toggle Arm           ║
║  LEFT (press)    Toggle Lever         ║
╚═══════════════════════════════════════╝
```

---

## Changing Controls

To modify controls, edit `src/seasons/pushback_2026/opcontrol.cpp`:

```cpp
// Change R1 to X for intake
if (controller.get_digital(DIGITAL_X)) {  // was DIGITAL_R1
    mech.intakeAndConvey(127);
}
```

Then rebuild:
```bash
pros make clean && pros make
```

---

*For control implementation, see [OPCONTROL.md](../OPCONTROL.md)*
*For game rules, see [GAME_RULES.md](./GAME_RULES.md)*
*For strategy, see [STRATEGY.md](./STRATEGY.md)*