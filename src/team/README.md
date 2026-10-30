# `src/team/` — the team's code

This folder is where the team's own robot code lives: the code students write, change between
matches and must be able to explain and defend — the driver's controls and the autonomous routines
first, as the programs that run them are built. Its first file is robot two's driver file,
`robot_two_driver.hpp` (chunk T4): which button does what and the feel (until the mechanism core's
fifth part it held the call into the library's driver loop too — below) — "shulib Drive" runs it on
its own (since T4 Part 3), and the competition
program's driver control runs it ("shulib Competition", since chunk COMP-b); neither has run on a
robot yet. Since the mechanism core's fifth part (MECH-A5, 2026-10-29) it holds only the team's
decisions — its claw controls (`whileHeld`, `toggle`, named by robot two's mechanism declarations),
the feel and the loop's two numbers; the plumbing every driver file shares (binding the controls to
the program's mechanism service, the controller, the call into the library's driver loop) is
`src/common/driver_file.hpp`, and the one heading rule `src/common/driver_heading.hpp` — never copied
here (`src/common/driver_controls.hpp` held both until then). Its second is the team's AUTONS,
`autons.hpp` / `autons.cpp` (chunk COMP-t): every autonomous routine the robot can run, and
`registerAutons()` at the bottom of `autons.cpp`, the one list both auton programs read — "shulib
Auton Tester" runs the one picked on the brain's screen when RUN is touched, and the competition
program ("shulib Competition") runs the one picked for a match, through the same library call
(guide chapter 17, "The competition program"). It is the one `.cpp`
under `src/` the host test suite compiles (it is PROS-free): the suite runs every auton in it on the
host test plant, so write a new auton there, run the tests, then try it on the bench (guide
chapter 16, "Testing an auton on the bench"). Both files are robot two's: they may name its
mechanisms (`autons.cpp` says `using namespace robot_two;`), which the src build gate allows only in
a robot's own team files — `src/team/robot_two_*`, and `autons.{hpp,cpp}` by its reviewed map: the
name `robot_two` written anywhere else under `src/` — split over lines or not — fails the build's
gate. (The gate reads the text as the compiler does; a name a macro pastes together with `##` is
never written out, so the gate cannot see it — never build a robot's name that way.) What
does **not** belong
here: what a robot *is* (its ports, motor directions and mechanisms live in `src/robots/`), a
program's plumbing (booting, screens, the PROS entry points — `src/programs/`, `src/common/` and
`src/main.cpp`), and anything the library already provides (`include/shulib/`, never copied in).
Include a file from here the way every file under `src/` includes another, written from `src/`:
`#include "team/<file>.hpp"`.
