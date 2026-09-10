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

On the lab computer, open a terminal:

```
cd ~/projects/shulib
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
