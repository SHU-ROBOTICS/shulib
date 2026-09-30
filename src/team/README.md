# `src/team/` — the team's code

This folder is where the team's own robot code lives: the code students write, change between
matches and must be able to explain and defend — the driver's controls and the autonomous routines
first, as the programs that run them are built. Its first file is robot two's driver file,
`robot_two_driver.hpp` (chunk T4): which button does what, the feel, and the call into the
library's driver loop — the competition program's driver control will run it, and so will the
driver program; the generic half it uses (the binder, the checks, the one heading rule) is in
`src/common/driver_controls.hpp`, never copied here. Its second is the team's AUTONS,
`autons.hpp` / `autons.cpp` (chunk COMP-t): every autonomous routine the robot can run, and
`registerAutons()` at the bottom of `autons.cpp`, the one list both auton programs read — "shulib
Auton Tester" runs the one picked on the brain's screen when RUN is touched, and the competition
program will run the one picked for a match, through the same library call. It is the one `.cpp`
under `src/` the host test suite compiles (it is PROS-free): the suite runs every auton in it on the
host test plant, so write a new auton there, run the tests, then try it on the bench (guide
chapter 16, "Testing an auton on the bench"). What does **not** belong
here: what a robot *is* (its ports, motor directions and mechanisms live in `src/robots/`), a
program's plumbing (booting, screens, the PROS entry points — `src/programs/`, `src/common/` and
`src/main.cpp`), and anything the library already provides (`include/shulib/`, never copied in).
Include a file from here the way every file under `src/` includes another, written from `src/`:
`#include "team/<file>.hpp"`.
