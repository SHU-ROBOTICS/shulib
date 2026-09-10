# DANIEL — READ THIS RIGHT NOW TO MAKE IT DRIVEABLE (2026-09-10)

From jal, who is not in the lab. Everything you need is pushed on branch `shulib-v2`. This note is
temporary and gets deleted once the chassis has driven; the permanent procedure is Station D of
`docs/internal/chunks/R3a-BENCH-WORKSHEET.md`.

## Why the motors "moved but cancelled out" — do NOT drive it that way again

Each side has five motors on ONE gear train. Some are mounted facing the other way, which is normal.
Any program that sends the same voltage to all five makes the reversed ones push backwards against
the others: the train locks, every motor sits at stall current, you get humming and twitching and no
motion. That overheats the motors and strips gears. **Do not use the brain's built-in drive program
or any other quick program on this chassis.** The bench program below measures which motors are
reversed and drives them correctly, and it refuses to power anything until it has that measurement.

## Step 0 — before any code (5 minutes)

1. **Charged battery in the brain.** It died this afternoon. The drive test pulls hard.
2. **Port 19's motor cable, both ends.** The brain saw only 9 of the 10 drive motors: ports 11–15
   and 16, 17, 18, 20 answered; **19 did not**. Re-seat that cable at the motor and at the brain.
3. **Controller:** turn it on and leave it on its **home screen** (the one showing the brain's name).
   **Never start the program from the controller's own menu** — that put the brain into a pretend
   match this afternoon (screen goes yellow, "FIELD CONTROL CONNECTED", nothing works). If you ever
   see that yellow screen: controller off, wait ten seconds, the menu comes back by itself,
   controller on again, leave it alone.

## Step 1 — build and upload the program (lab computer)

**Everything is on the `shulib-v2` branch.** `main` is the August release and has none of this, so
if you clone the repo fresh or are on another machine, switch branches first.

On the lab computer, open a terminal:

```sh
cd ~/projects/shulib
git checkout shulib-v2
git pull
make ROBOT=tank
pros upload --slot 3 --after run
```

Brain powered on and plugged into the computer by micro-USB first. The upload prints
`Finished uploading "Bench Tests"`. On the brain's boot splash, check the BUILD stamp shows today's
date and the time you just built. If it does not, the upload did not land; run it again.

If you are on your own laptop instead: you need git, the PROS CLI, and `arm-none-eabi-g++`. If you
do not have those, use the lab computer.

If the program is already on the brain and you just power-cycled, start it from the **brain**:
Programs → "Bench Tests" (slot 3) → **Run**. Not Timed Run, not Match.

## Step 2 — the buttons, in this order

The program shows a touch menu with ten buttons. Press firmly with a fingernail, straight down and
up — the screen is resistive, not a phone. After each button, touch the screen to return to the menu.

1. **4 BATT/CTRL** — must say **CONNECTED**. If NOT CONNECTED, pair the controller: smart cable
   from the top port of the controller into any brain smart port, wait for the brain's name on the
   controller's screen, unplug the cable, go back to its home screen.
2. **1 DEVICE CENSUS** — must say **motors found: 10**. If port 19 is still missing, fix the cable
   and run it again. Do not go on with nine.
3. **3 MOTOR WATCH (live)** — robot on the floor, all wheels down. After you tap it, **push the whole
   robot forward about a foot by hand.** Forward = the end with the motors on ports **15 and 16**
   leading (ports 11 and 20 are the back). Then touch the screen. It asks which way you pushed:
   tap **FRONT FIRST**. It is done when it prints **SIGNS CAPTURED** and a line like
   `signed ports for DRIVE: LEFT +11 -12 +13 ... | RIGHT ...`. **Write that line down or photograph
   it.** UP and DOWN mixed within a side is normal and is exactly what it is measuring.
   If it says PARTIAL, push again, firmer, all wheels on the floor.
4. **Lift the robot onto blocks** so no wheel touches anything. Second person at the battery.
5. **10 DRIVE (POWERS)** — it prints the cartridge it will use: must say **BLUE**. It asks ARE THE
   WHEELS OFF THE GROUND: tap **YES, WHEELS UP** only if step 4 is true.
6. **Hold L1** on the controller and ease the **left stick forward**. Every wheel on both sides turns
   toward the front. Let go of L1: everything stops. Right stick turns the sides opposite ways.
   Press **R1** once to allow 6 V, again for 9 V, again for 12 V. Watch the screen: every port row
   green with the same sign on a side. A red row or a **CUT** message means the program protected
   the drivetrain from a fight — **copy the message word for word** and stop.
7. **Touch the screen to exit.** Everything goes to 0 V. The summary shows max current and
   temperature per port.
8. **Ground run:** put the robot on the floor with 3 m clear all round, tap **10 DRIVE** again, answer
   **NO** to wheels up, then **YES, GROUND MODE**. Drive at 3 V, then press R1 once for 6 V. Stop
   there. Higher than 6 V on the ground only with jal present.

## Send back to jal

- the `signed ports for DRIVE` line from step 3, exactly
- what the census said (10 motors or not)
- anything red or any CUT message, word for word
- did all wheels turn toward the front on "forward"? if not, which one

That line of signed ports is what goes into the library so the real driver code drives it right.

## What jal is building, and where things live (so you know where to look)

**The goal.** `shulib` is the team's own robot library: driver control, autonomous motion, and
position tracking, written so the same code runs on both of this season's robots and so that
routines can be written without C++. It is built in pieces, each tested on a computer against a
simulated robot before it ever touches a real one, and every fact about a robot is *measured*, never
guessed. Until this week no real robot had run any of it. Your chassis is the first.

**Two different programs, and which one you are using today.**

- **The bench tester** — `src/bench_r3a.cpp`, the program in slot 3 called "Bench Tests". It is a
  measuring tool with a touch menu. It does not use the library's driving code at all; its DRIVE
  station sends plain volts to the motors through the library's motor and controller *adapters*,
  behind safety checks. **This is the only thing that should drive the chassis today.**
- **The real driver code** — the library's own teleop loop in `src/main.cpp`, which drives through
  the library's `Chassis` object. It cannot run on your chassis yet, on purpose: the library still
  needs two pieces for a tank drive with five motors per side and no tracking wheels (a "motor
  group" that fans one command to five motors with the right signs, and odometry from the drive
  motors' own encoders), plus an IMU on the robot. jal is building those next. **The sign line
  from MOTOR WATCH is one of the inputs to that work — that is why it matters that you send it.**

**The one place you might need to edit.** Near the top of `src/bench_r3a.cpp`, the block that starts
`#if defined(SHULIB_ROBOT_TANK_2026)` is the chassis table for this robot: the left ports
`{11, 12, 13, 14, 15}`, the right ports `{20, 19, 18, 17, 16}`, the cartridge `Blue`, and
`imuPort = 0` (none mounted yet). If a motor is moved to a different port, change the number there,
rebuild, upload. **If you mount an IMU, put its port number in `imuPort`.** Nothing else in that file
needs touching to drive. Do not type motor signs into the table: MOTOR WATCH measures them every
power cycle, and the permanent version saves them to the SD card automatically (that is the next
chunk of work, called R3d).

**How to build for this robot.** `make ROBOT=tank` — the `ROBOT=` switch in the `Makefile` picks the
robot; `bench` is the old practice bot, `tank` is yours. Building the wrong one boots a table for the
wrong robot and refuses loudly, so it is hard to get wrong.

**Where the procedure and the findings live.** `docs/internal/chunks/R3a-BENCH-WORKSHEET.md`, Station D,
is the full version of the steps above with blanks to fill in. Everything measured on this chassis
is being written into `docs/internal/chunks/R3b-PROGRESS.md` (section 9 is today's session). Anything
you find — a port that will not answer, a CUT message, a motor that runs hot — belongs there, so
send it to jal in words and it gets recorded with the evidence.

**What not to touch.** Everything under `include/shulib/` is the library proper, with frozen public
interfaces and a test suite of over a million assertions; nothing in there is the reason the chassis
is not driving, and editing it will not make it drive. The fix for "the motors fight each other" is
the sign measurement, not code.
