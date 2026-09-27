# `src/team/` — the team's code

This folder is where the team's own robot code lives: the code students write, change between
matches and must be able to explain and defend — the driver's controls and the autonomous routines
first, as the programs that run them are built. Its first file is robot two's driver file,
`robot_two_driver.hpp` (chunk T4): which button does what, the feel, and the call into the
library's driver loop — the competition program's driver control will run it, and so will the
driver program; the generic half it uses (the binder, the checks, the one heading rule) is in
`src/common/driver_controls.hpp`, never copied here. What does **not** belong
here: what a robot *is* (its ports, motor directions and mechanisms live in `src/robots/`), a
program's plumbing (booting, screens, the PROS entry points — `src/programs/`, `src/common/` and
`src/main.cpp`), and anything the library already provides (`include/shulib/`, never copied in).
Include a file from here the way every file under `src/` includes another, written from `src/`:
`#include "team/<file>.hpp"`.
