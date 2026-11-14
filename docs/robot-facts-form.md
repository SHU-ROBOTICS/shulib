# Robot facts form — robot one (the cascade robot)

> **For the team's builder.** One page: fill it in at the robot and hand it to whoever types the facts.
> Most answers here become one fact in `src/robots/robot_one_facts.hpp` (2 and 4 go into the drive table; the
> footer says which answer stops which build), and until a fact is typed, every robot-one program that needs it
> refuses to build, naming it ([Chapter 21](guide/21-adding-your-robot-by-hand.md)).
> **Write "don't know" rather than guess.** For each answer write **who**, **how** (read off / counted /
> tape / station 13 RECORD) and **the date**. "The robot's centre" = midway between the left and right
> wheels, and midway along the wheels.

## A. What the robot is (before any program)

1. Is the drive a **tank** — wheels on two sides, it cannot drive sideways? *(If NO: stop and tell the
   coordinator — every robot-one program supports only a tank.)*
2. How many drive motors per side; their **cartridge colour** (read the insert on ONE motor: red / green /
   blue).
3. Wheel size (inches, across the tread); motor-to-wheel gearing (tooth counts, or "direct").
4. **Is there an inertial sensor (IMU)? Which port?** *(The library cannot drive a tank without one.)*
5. Which end is the front (where is the AI Vision camera)?

## B. What is plugged where — read off the brain's own Devices screen (no program needed)

6. Every port 1–21: the device (motor / rotation / vision / IMU / radio / empty) and what it is for.
   **Mark every motor that is NOT a drive motor** — each one must be declared before Calibrate runs, or
   Calibrate would take it for a drive motor and power it.
7. Every 3-wire port A–H: what is plugged in (a solenoid? nothing?).

## C. The cascade

*C1–C5 and C6 (i) come first; C6 (ii), C7 and C8 need Bench Tests' station 13 on robot one, which builds
once 6, 8, 9, 13 (i) and 16 are typed (the footer).*

8. **(C1)** Its motor ports; their cartridge colour; do two motors face each other (mirrored)? *(As you see
   them — station 13's direction check confirms each.)*
9. **(C2)** Gearing from motor to spool: count the teeth (motor gear, spool gear).
10. **(C3)** A rotation sensor on the cascade? Its port; on which shaft.
11. **(C4)** The point on the top stage the cascade's HEIGHT is measured from (pick one — say which), to the
    floor.
12. **(C5)** Its levels, by stack height: a NAME (15 letters at most) and the height of each — **four to
    six**. *(The default controls name the first four on the d-pad; fewer than four: tell the coordinator,
    who changes the controls first.)*
13. **(C6)** Two steps. (i) A **starting** current cap and stall signature, typed BEFORE Bench Tests is
    built — **chosen, not measured**, with the coordinator, from chapter 18's order rule (the heaviest hold
    < the signature < the cap − 0.2 A; the cap no more than the string can take); written "chosen, not
    measured". (ii) Later, the **measured** pair from station 13's RECORD while the cascade presses gently
    down on its bottom stop at 3 V.
14. **(C7 — station 13)** With the cascade on its bottom stop at power-up: RECORD. Then jog up and stop at
    each level: tape the height (item 11's point) and RECORD. **Read the RECORD's `output … deg`, not its
    inches** — until the shape is typed, station 13's inches come from placeholders. Three to eight rows.
    **The top stop:** station 13 does **not** apply the cap yet, so keep the ceiling at 3 V on the cascade
    and never power it into the top stop — jog up at 3 V to just short of it and stop; close the last bit
    by hand only if the gearing turns freely by hand, then tape and RECORD. If it does not, write "top
    stop not reached" and tell the coordinator.
15. **(C8)** Three voltages, each measured or **chosen with the coordinator** (then written "chosen, not
    measured"): (i) the smallest volts that hold the empty cascade halfway without sagging; (ii) the
    **homing push** — how hard it pushes DOWN into its bottom stop to find zero (gentle; written negative,
    e.g. −3 V); (iii) the most a held button may give it **before** it has been homed.

## D. The flipper

16. Its kind: **motor arm / pneumatic / passive bar** — or "not on the robot yet".
17. Motor arm: port, cartridge, gear teeth; at station 13, RECORD stowed and at full flip (the angles).
    Pneumatic: the 3-wire letter; one cable or two from the valve; which way is "out" (station 13's fire);
    how long a full stroke takes. Passive: nothing.

## E. Sensors

18. The AI Vision camera: its port; how far forward and left of the centre, how high, tilted how much.
19. Each rotation sensor not on the cascade: where, and what it measures.

## F. The twin

20. Is robot two being rebuilt like robot one? When? *(Then this form again for robot two.)*

---

*Which programs wait for which answers:* 2 and 4 go into the drive table, typed before Calibrate runs —
Calibrate refuses an unset cartridge, and the boot refuses a tank with no IMU, by name — and 4 is typed
in the same edit as 6's and 8's motor ports (until those are typed, placeholder ports stand in, and an IMU on
one of them fails every build by name); **every program's
build** waits for 6's non-drive motors, 8, 9, 13 (i) and 16; **Drive, the Auton Tester, Competition and
shulib Tune** (every program but Bench Tests and Calibrate) also wait for 12, 14 and all three of 15 (11 is
how 14's heights are taped; 15's voltages may first be "chosen, not measured" and then tuned with shulib
Tune). 17 stops no build yet: the flipper's numbers become facts — and start to stop builds
— only when its block is added ([Chapter 21](guide/21-adding-your-robot-by-hand.md), the flipper). 13 (ii)
re-types facts already typed, and should land before the skills runs. 18 and 19 are written down and stop
no build.
