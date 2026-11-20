# `src/team/` — the team's code

This folder is where the team's own robot code lives: the code students write, change between
matches and must be able to explain and defend — the driver's controls and the autonomous routines
first, as the programs that run them are built. Its first file is robot two's driver file,
`robot_two_driver.hpp` (chunk T4): which button does what and the feel (until the mechanism core's
fifth part it held the call into the library's driver loop too — below) — "shulib Drive" runs it on
its own (since T4 Part 3), and the competition
program's driver control runs it ("shulib Competition", since chunk COMP-b); neither has run on a
robot yet. Since the mechanism core's fifth part (MECH-A5, 2026-10-29) it holds only the team's
decisions — its mechanism controls (`whileHeld`, `toggle`, named by the robot's mechanism declarations),
the feel and the loop's two numbers; the plumbing every driver file shares (binding the controls to
the program's mechanism service, the controller, the call into the library's driver loop) is
`src/common/driver_file.hpp`, and the one heading rule `src/common/driver_heading.hpp` — never copied
here (`src/common/driver_controls.hpp` held both until then). Its second is the team's AUTONS,
`autons.hpp` / `autons.cpp` (chunk COMP-t): every autonomous routine the robot can run, and
`registerAutons()` at the bottom of `autons.cpp`, the one list both auton programs read — "shulib
Auton Tester" runs the one picked on the brain's screen when RUN is touched, and the competition
program ("shulib Competition") runs the one picked for a match, through the same library call
(guide chapter 17, "The competition program"). The host test suite compiles it — with robot one's
`robot_one_autons.cpp` and the shared `skills_route.cpp`, the three `.cpp` files under `src/` it
compiles (all PROS-free) — and runs every auton in them on the host test plant, so write a new auton
there, run the tests, then try it on the bench (guide chapter 16, "Testing an auton on the bench"). Both files are robot two's: they may name its
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

**Robot one's files (chunk ROB1-A, 2026-11-11).** `robot_one_driver.hpp` is robot one's driver file
(`RobotOneDriver`) — the BUILDER'S map since 2026-11-15 (chunk ROB1-C2; ROB1-A's was a default written
only because nothing builds without one): R1 / R2 the cascade while held, A the claw, B the front toggle, X
the ramp, L1 / L2 the right / left side toggle while held, Y the re-zero, the d-pad free; **the team's to
change** (guide chapter 21, "Adding your robot by hand"). `robot_one_autons.hpp` / `.cpp` hold `registerRobotOneAutons()` — the
40 in and the acceptance run as SKILLS autons, and the team's skills route.

**Robot two is robot one's TWIN (chunk ROB1-BT, 2026-11-16).** Robot two was rebuilt as robot one's build
(its claws retired), so its files started as robot one's with robot two's name. `robot_two_driver.hpp`'s
buttons, feel and map are **robot two's own** (the team lead, 2026-11-20): change them for robot two alone.
`autons.cpp` holds robot one's list with ONE difference, the team lead's: robot two runs the skills route
MIRRORED (`registry.add(competition::mirrored(skillsRoute(), …kSkillsMirror));`, cookbook chapter 9).
`test/robot_twin_test.cpp` fails the host suite when the autons lists, the mechanisms or the driver files'
plumbing differ, so **change an auton in BOTH robots' files**. **`skills_route.hpp` / `.cpp`** hold the
team's skills route, written ONCE in robot one's numbers and compiled into both robots: its names
(`cascade`, `claw`, …) are the BUILDING robot's (`using namespace this_robot;`). The route maker writes
only between its `ROUTE MAKER — BEGIN` and `END` lines (delete those two lines to write the route by
hand), and in each driver file only between the `[control words …]` and `[feel …]` markers. Robot one's files say `using namespace robot_one;` the way robot two's say `robot_two`, and the
same gate rule holds for both names. A mirror swaps the robot's sides, so a side-toggle step in the
shared route picks the side: `competition::isMirrored(ctx) ? leftToggle : rightToggle` (robot one's
right toggle is robot two's left one). **Which file is which robot's:** `robot_two_driver.hpp` and
`autons.{hpp,cpp}` are robot two's; `robot_one_*` are robot one's. A program never names either robot's:
it runs `src/common/this_robot.hpp`'s `ThisRobotDriver` and `registerThisRobotsAutons()`, which pick the
build's robot (`make ROBOT=tank …` or `make ROBOT=one …`), and the src build gate fails a program that
names `RobotTwoDriver`, `RobotOneDriver`, `registerAutons` or `registerRobotOneAutons` itself. Robot
one's numbers are not here: they live in `src/robots/robot_one_facts.hpp` — typed from the builder's answers
(2026-11-15), OWED where nobody has said yet.
