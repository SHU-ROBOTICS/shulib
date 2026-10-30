# R3a — bench worksheet (no program required)

> **For a helper at the robot. Nothing here needs code uploaded.** The build this worksheet was first
> written against — the invented X-drive, `make ROBOT=xdrive` — **cannot boot on this robot**: its port
> map is invented and wrong (R3a brief §2; since G1 Part 2c it refuses at boot, naming the first port,
> with nothing powered). Do not try to run it. Everything below uses the brain's built-in **Devices** screen, a tape measure, and hands.
>
> Answers get pasted **raw, uninterpreted** into `R3a-PROGRESS.md` Batch 1 before any analysis.

---

## Station A — RUN THE PROGRAM (no laptop needed)

**The program is already on the brain: slot 3, named "Bench Tests".** It is **read-only and commands
no motion — EXCEPT button 10 DRIVE**, which powers the drive motors and has its own station (D)
below. Nothing else on the menu can move the robot. *(Button **12 AI VISION** writes the AI Vision
CAMERA's own settings — tag and object detection on, its video overlay on — and says so on its
screen; it powers nothing. Station I.)* *(Robot two has FOUR more programs, five in all (since T4
Part 3, 2026-10-04). **"shulib Drive"** in **slot 1** — robot two's DRIVER FILE on the library: the
sticks drive and the claw buttons work from the moment it starts, built from the calibration record
(Station F; the claws, Station J). **"shulib Auton Tester"** in **slot 2** runs one of the team's
AUTONS from RUN on its screen after a 3 s countdown (Station M). **"shulib Calibrate"** in **slot 4**
MEASURES THE DRIVETRAIN ITSELF and writes the answer to the SD card (Station G) — "shulib Drive",
the tester and the competition program boot from that answer. **"shulib Competition"** in **slot 6**
is THE program for a MATCH: the auton chosen on its brain-screen menu runs in `autonomous()`, then
driver control runs the same driver file (Station K, after M). Do not confuse them: "Bench Tests"
measures by hand, "shulib Calibrate" measures the drivetrain, "shulib Drive" is driving practice,
the tester runs one auton on the bench, and the competition program is the match. *Retired at T4
Part 3:* the OLD "shulib Drive" (the typed table's open-loop drive — Station E is now a pointer) and
"shulib Teleop" (slot 2, its diagnostics now "shulib Drive"'s DETAILS page).)*

> **The slot map — FINAL (the team lead's rulings, 2026-09-27 — T4 R6; in force since T4 Part 3).**
> **1** "shulib Drive" (the NEW one) · **2** "shulib Auton Tester" · **3** "Bench Tests" · **4** "shulib
> Calibrate" · **5 HELD for the team's own old-code fallback: never upload a shulib program over it** ·
> **6** "shulib Competition" · 7–8 free. The NEW "shulib Drive" has the OLD one's name and slot, but drives
> toward the calibration record's front (the camera end) where the old one drove toward the 15/16 end:
> **Station F.0 tells them apart before any wheel moves.** A brain uploaded before T4 Part 3 may still hold
> "shulib Teleop" in slot 2 and the tester in slot 7: re-upload the tester to slot 2 (Station M) — it
> overwrites Teleop — and delete slot 7 from the brain's Programs menu. **Before any upload session,
> photograph the brain's Programs list**, so what each slot held is on record.

1. Power the brain on. Open **Programs** and run **slot 3 — "Bench Tests"** with **Run** — not
   Timed Run, not Match. **Start it from the BRAIN's screen or from the laptop, NEVER from the
   controller's own menu.** On 2026-09-10 (robot two's first session) launching it from the
   controller put the brain into a pretend match — the program restarted, then showed a yellow
   **FIELD CONTROL CONNECTED / DISABLED / AUTONOMOUS** screen and no menu, because the tester
   lives in driver control and VEXos was reporting a field. If you ever see that yellow screen:
   turn the controller off, wait ten seconds while the menu comes back on its own, turn the
   controller on again and leave it on its home screen. A linked controller sitting on its home
   screen is fine; the same session proved the program boots straight into driver control that way.
   **One exception, and only one (COMP-b, 2026-09-30):** "shulib Competition" in **slot 6** is
   started from the controller's **Timed Run** at **Station K** — the team owns no competition switch,
   and a pretend match is what K tests (A4 register HA-162). Afterwards turn the controller off, wait
   ten seconds, and see the menu return before starting any other slot.
   The header bar also shows **`T x,y r# h#`** — the last touch the program saw, the release count,
   and which button it hit (`h-1` = none). If a tap seems to do nothing, read that line to whoever
   is helping: it says whether the program saw the tap and where it thinks it landed.

> ⚠ **Slot 1 held an OLD build called "QueensRevenge". Do not run it.** Its port map is invented and
> it will fault at boot on this robot. It can only be deleted from the brain's own Programs menu —
> the PROS CLI has no remove command. *(Since R3b Part 0b, slot 1 is where **"shulib Drive"** goes
> — Station F.0's upload command overwrites it. Until that upload has been done on this brain, the
> warning stands: if slot 1 still says "QueensRevenge", do not run it.)*
2. A **boot splash** shows a **BUILD** stamp for two seconds, and the same stamp sits on the menu.
   **If that stamp is not the build you just uploaded, the upload did not land** — re-upload before
   trusting anything on screen. (A silent upload failure cost a debugging cycle on 2026-08-18.)
3. A **touch menu** appears: **twelve buttons, three across and four down**, numbered like a
   phone keypad (since R2a, 2026-09-24 — it was two columns of five):

   ```text
   [ 1 DEVICE   ] [ 2 IMU +    ] [ 3 MOTOR    ]
   [   CENSUS   ] [   ROTATE   ] [   WATCH    ]
   [ 4 BATT/CTRL] [ 5 SD CARD  ] [ 6 LOOP RATE]
   [            ] [   PROBE    ] [            ]
   [ 7 RUN ALL  ] [ 8 MOTORS   ] [ 9 SCREEN   ]
   [            ] [   STATIC   ] [   RULER    ]
   [ 10 DRIVE   ] [ 11 DISTANCE] [ 12 AI      ]
   [   POWERS   ] [            ] [   VISION   ]
   ```

   Each button shows its number and name on two lines (shortened to fit; the log still writes
   the full name, e.g. `>>>>>> 3  MOTOR WATCH (live)`). Tap any test:

   **An amber stripe down a button's left edge means that test needs your hands.**
   Each test also opens with two amber lines — `DO NOW` and `DONE IF` — so you never
   have to guess what you are supposed to do or when you are finished.

   | Button (as the log names it) | Hands? | What YOU do | Finished when |
   |---|---|---|---|
   | **1 DEVICE CENSUS** | — | nothing, just read it | you have noted every port and what is in it |
   | **2 IMU + ROTATE** | **yes** | turn the whole robot counter-clockwise | the big number **ROSE** while you turned — if it fell, say so |
   | **3 MOTOR WATCH (live)** | **yes** | push the robot forward, OR spin one wheel — it records itself | the table showed UP/DOWN per port and you touched to save it |
   | **4 BATT/CTRL** | **yes** | pair a controller if it reads NOT CONNECTED | it reads CONNECTED (unblocks 4 register entries) |
   | **5 SD CARD PROBE** | — | nothing, unless it fails — then reformat FAT32 | all three steps pass, header chip reads PASS |
   | **6 LOOP RATE** | — | nothing, leave the robot still | min/max/mean are printed |
   | **7 RUN ALL** | **yes** | be ready to rotate when test 2's readout appears | every test above has run once |
   | **8 MOTORS (static)** | — | nothing — a one-shot snapshot; use 3 to capture movement | you have seen each motor's raw and canonical position |
   | **9 SCREEN RULER** | **yes** | answer the four numbered questions on screen | all four answered — they fix the layout constants |
   | **10 DRIVE (POWERS)** | **yes — RED stripe** | **POWERS THE MOTORS.** Wheels off the ground; hold **L1** to drive | you drove at 3 V wheels-up, raised the ceiling with **R1**, touched to stop — see **Station D** |
   | **11 DISTANCE** | **yes** | hold the robot square to a wall at 6, 12 and 24 in (tape) and press **RECORD** at each; then point it at open space — see **Station H** | a RECORD number for 6, 12, 24 in and open space, per sensor |
   | **12 AI VISION** | **yes** | a game piece about 1 ft then 2 ft ahead, moved left then right, **RECORD** each; **FAMILY** for a printed tag — see **Station I** | a RECORD number per spot, and which FAMILY sees your tag (if you have one) |

   Each button carries a **status dot** at its bottom right: hollow until run, then filled —
   green PASS, amber CHECK, red FAIL, **white DONE** (the test ran and judges nothing — 11 and 12
   end that way). That is how you tell what is left without keeping a list.
   **Button 10 has a RED stripe and a red edge instead of amber: it is the only one that
   powers motors.** *(This table was corrected at R3b Session 2 — it had listed the menu as it
   was before MOTOR WATCH existed — and grew to twelve at R2a.)*

4. Results print on the screen. When it fills, tap to continue. When a test ends, tap
   **TOUCH TO RETURN TO MENU**.
5. **Everything is also written to `/usd/r3a_log.txt` on the SD card.** That file has the full
   detail — the screen truncates long lines. **Do not lose the card.**

> Re-running a test is free. Run anything as many times as you like — especially **3 MOTORS** after
> turning each wheel, which is how the port→wheel map gets built.
>
> One catch: the log file opens once per boot, so **power-cycling starts a fresh file**. Copy the
> card off before rebooting if a session matters.

---

> **The validation binary does Stations 1, 2 and 4 better than hands can — but run BOTH.** It
> prints a full device census, raw-vs-canonical values, and live IMU heading, and it writes a copy to
> `/usd/r3a_log.txt`. The hand measurements below are then an **independent second method**, and
> two methods agreeing is worth far more than either alone. Where they disagree, that is the finding.

## Station 0 — setup (2 min)

- [ ] Charged battery in, brain powered on.
- [ ] From the home screen open **Devices**. Leave it open — most of this session lives there.
- [ ] Phone out. **Photograph rather than transcribe** wherever a photo is possible; a photo can be
      re-read, a transcription cannot.

---

## Station 1 — the port map  *(settles HA-111, HA-120)*

- [ ] **1.1** Photograph the **full Devices list**. Scroll and take a second photo if it runs long.
- [ ] **1.2** Is there an **ADI expander** in the list? **YES / NO** — either answer is useful.
      *(A previous report claimed one, read from registry index 21 — outside the documented 0–20
      range — so its absence would CORRECT that report.)*
- [ ] **1.3** Note any port that shows a device **the list cannot identify**, or that flickers
      between present and absent.

> Expected from the last session, with port 13 now repaired: drive on **11, 12, 13, 14** and
> **15, 16, 17, 18**, IMU on **4**, **no** rotation sensors, **no** GPS. **Contradicting this is a
> finding, not a mistake** — write down what you see, not what this line says.

---

## Station 2 — which motor is which, and which way is positive

Program-free port identification, done from the Devices screen:

> **"Spin the wheel forward" is ambiguous and we are not going to say it.** Forward for the
> *wheel*, or for the *robot*? Seen from which side? A wrong reading here **mirrors every turn the
> library will ever command**, so these are two separate steps, in this order.

**STEP 1 — which way is positive**

- [ ] **2.1** Decide which end of the robot is its **FRONT**, and **photograph it**. The library's
      frame is +X forward, so this choice is part of the measurement, not an obvious fact.
- [ ] **2.2** Note each drive port's position, then **push the WHOLE ROBOT forward** about a foot
      along the floor, front end leading.
- [ ] **2.3** Read the ports again. For **each** one, write down **UP or DOWN**.
      *(Pushing the robot removes the ambiguity: a robot has exactly one forward, and no near side
      or far side to argue about.)*

**STEP 2 — which port is which wheel**

- [ ] **2.4** Lift the robot so the wheels are off the ground.
- [ ] **2.5** Spin **ONE** wheel — **any direction, it does not matter for this step**.
- [ ] **2.6** Read the ports. Exactly one changed: that port drives that wheel. Repeat per wheel and
      fill in the table below.

| Port | Which wheel (front-left / back-right / …) | STEP 1: pushing robot FORWARD → |
|---|---|---|
| 11 | | UP / DOWN |
| 12 | | UP / DOWN |
| 13 | | UP / DOWN |
| 14 | | UP / DOWN |
| 15 | | UP / DOWN |
| 16 | | UP / DOWN |
| 17 | | UP / DOWN |
| 18 | | UP / DOWN |

- [ ] **2.4** Any **non-drive** motors: port → what it drives (intake / lift / clamp / …).
      *(Settles HA-92 and scopes what mechanisms exist.)*

---

## Station 3 — drivetrain geometry and gearing  *(HA-14, HA-15, HA-17 — the highest value here)*

- [ ] **3.1 Wheel diameter.** Easiest on a mounted wheel: measure **floor → center of the axle**,
      then **double it**. Expect **2.75″ / 3.25″ / 4.00″**. Anything between means the center or the
      hub was caught — remeasure.
      → **________ in**

- [ ] **3.2 GEAR RATIO — PER SIDE. The single highest-value measurement in this session.**
      The two sides may be geared **differently** (a prior robot's source says the right side was
      *"geared down a touch for traction"*). **Do not average, do not give one number for both.**
      - Photograph each gear **straight-on** and count teeth in the photo, zoomed — far easier than
        counting on the robot, and re-countable.
      - If the motor shaft goes straight into the wheel with no gears: write **"direct drive"**.

      → **LEFT side:  ______ (motor gear teeth) : ______ (wheel gear teeth)**
      → **RIGHT side: ______ (motor gear teeth) : ______ (wheel gear teeth)**
      → **Do the two sides differ? YES / NO**

- [ ] **3.3 Motor cartridge colour — YOU MUST LOOK. Software cannot answer this.**
      The colour insert is visible through the motor housing: **RED / GREEN / BLUE**.
      Check **every drive motor**, or at minimum two per side, and note any that differ.
      → LEFT: ________  RIGHT: ________

      > The brain reports a cartridge, but that is a **software setting** a program wrote
      > (`motor_set_gearing`), not a reading of the hardware — a V5 motor cannot sense its own
      > cartridge. On 2026-08-19 the brain said GREEN while the build team recalled BLUE.
      > **Your eyes are the only authority here**, and a mismatch is a real finding: it means
      > velocity and position are scaled wrong by up to 3x.

- [ ] **3.4 Track width.** Measure **outside face of left wheel → outside face of right wheel**,
      then **subtract one wheel's width**. That is centre-to-centre of the contact patches without
      eyeballing the middle of a tyre.
      → **________ in**

- [ ] **3.5** Photograph the drivetrain from **above** and from **one side**.

---

## Station 4 — the IMU, live on the brain screen  *(HA-02, HA-110, HA-23)*

The IMU convention is a **sign** question, and a wrong sign mirrors the whole world. Devices shows
it live, so this needs no code.

- [ ] **4.1** Tap the **IMU (port 4)** in Devices so its live heading shows.
- [ ] **4.2** Rotate the whole robot **COUNTER-CLOCKWISE** (viewed from above — i.e. to its left).
      Does the heading number go **UP or DOWN**?
      → **UP / DOWN**  ← *write this down carefully; it is the one that mirrors everything*
- [ ] **4.3** Does the heading **wrap** at 360 → 0, or keep counting past 360? → **WRAPS / KEEPS COUNTING**
- [ ] **4.4** If pitch/roll are shown: **tilt the nose UP** — does pitch go **UP or DOWN**?
      **Roll to the RIGHT** — does roll go **UP or DOWN**? *(If not shown, write "not shown".)*
      → pitch: ________  roll: ________
- [ ] **4.5** Power-cycle the brain and watch the IMU value. **Roughly how many seconds** until it
      settles instead of drifting/jumping? → **________ s** *(expected ≈ 2 s; a very different
      number matters.)*
- [ ] **4.6** With the robot **completely still**, watch the heading for **60 seconds**.
      Start: ________  After 60 s: ________  → drift = ________

---

## Station 5 — inventory questions  *(may delete a week of work)*

- [ ] **5.1** Are there **two VEX Rotation sensors** loose in a parts bin? **YES / NO / how many**
      *(Not mounted — just whether we own them. A prior robot's code declared two that were never
      wired. If they exist, a whole odometry workaround may be unnecessary.)*
- [ ] **5.2** Are there **tracking-wheel omni wheels** (~1.5–2″) available? **YES / NO**
- [ ] **5.3** Is there a **VEX GPS sensor** anywhere? **YES / NO**
- [ ] **5.4** Is there an **AI Vision camera** anywhere? **YES / NO**
- [ ] **5.5** Is a **second controller** (partner) available and pairable? **YES / NO**

---

## Station 5.5 — PAIR THE CONTROLLER  *(2 minutes, unblocks four register entries)*

**Do this even if nothing else gets done.** No controller has been paired in either previous bench
session (`master=0 partner=0`), and that one fact has kept **HA-57, HA-103, HA-104 and HA-107**
unreachable both times. It also enables **wireless program upload and a wireless terminal**, which
moves the whole develop-upload-read loop off the robot and onto a desk.

- [ ] **5.5.1** Pair a **master** controller to the brain, using VEX's documented pairing procedure
      (tether it to the brain, then it holds the pairing). *Confirm the OUTCOME rather than the
      steps:* the brain should report a **connected master controller**.
      → paired? **YES / NO**
- [ ] **5.5.2** Battery level on the controller: ________ %  *(a flat controller looks like a
      pairing failure)*
- [ ] **5.5.3** If a **partner** controller is available, pair it too — VEX U runs two drivers.
      → partner paired? **YES / NO / none available**
- [ ] **5.5.4** With the controller paired, does the brain's Devices/status list show it? Photograph.

> If pairing does not take: note **exactly what the screen said** and move on. A failure here is a
> finding — it has silently blocked two sessions already.

## Station 6 — SD card prep  *(HA-122 — physical half only)*

- [ ] **6.1** Format the microSD card **FAT32**. *(exFAT is NOT read by the brain.)*
- [ ] **6.2** Card size: ________ GB. *(Note it — very large cards are the usual failure.)*
- [ ] **6.3** Insert it into the brain's slot. Does it **seat and click**? **YES / NO**
- [ ] **6.4** Power-cycle with the card in. Does the brain boot **normally**? **YES / NO**
- [ ] Leave the card **in**.

> The rest of HA-122 — `usd_is_installed()`, the `/usd/` path convention, and the
> yank-the-card-after-a-flush durability test — needs the R3a validation binary and is **not**
> today's work.

---

## Station D — DRIVE  *(the 2026 tank chassis, robot two — added R3b Session 2, 2026-09-10)*

> ⚠ **This is the ONE station that powers motors.** Button **10 DRIVE** has a **RED** stripe. It
> drives the robot through the library's motor and controller *adapters* with open-loop volts — it
> is **not** the library's motion stack, and it commands nothing unless a button is **held**.
> Everything else on the menu is still read-only.
>
> **Six things must be true before it will drive, and the program checks each one and refuses
> loudly if it is not.** The refusal text names what is missing. A refusal is a finding — write
> down what it said.

**D.0 — before any code runs (the build team, at the robot)**

- [ ] **D.0.1 Mount an IMU** on any smart port. Note the port: → **________**
      *(The tester's IMU button and the library both need one; until it is mounted, the IMU
      station refuses and says so. No other sensor is needed for this station.)*
- [ ] **D.0.2 Pair the controller** (Station 5.5). Button 4 must read **CONNECTED**.
- [ ] **D.0.3 Read the cartridge colour off a drive motor** — the insert is visible through the
      housing. → **RED / GREEN / BLUE** *(reported BLUE on 2026-09-10; confirm with your own eyes,
      on at least two motors per side. Software cannot answer this.)*
- [ ] **D.0.4 Ruler the wheel**: floor → axle centre, doubled. → **________ in** *(reported 2.75)*
- [ ] **D.0.5 Tape the track width**: outside face of a left wheel → outside face of the right,
      minus one wheel width. → **________ in**
- [ ] **D.0.6 Decide which end is the FRONT and photograph it.** Every sign below is relative to
      this choice. → front = **the end where ports 15 (left) and 16 (right) sit** *(implied by
      the 2026-09-10 port report, which was read standing behind the robot; describe or photograph
      that end here so nobody has to re-derive it: ________)*
- [ ] **D.0.7 Report the drive ports, PER SIDE**, from the Devices screen or button 1:
      → **LEFT: 11 12 13 14 15**  **RIGHT: 20 19 18 17 16** *(each back → front; reported by the
      team lead 2026-09-10 from the robot and typed into the table the same day — D.1.1's census
      confirms them and is what flips the table's `measured` flag)*
      *(Left and right as seen from behind the robot, looking toward the front you chose.)*
- [ ] **D.0.8 External gearing motor → wheel**: tooth counts, or "direct". → **________**
      *("600 rpm" was reported, which reads as direct drive; the count settles it.)*

> **STOP here and send D.0.3–D.0.8 in.** The chassis table (`src/robots/chassis_table.hpp` since R3b
> Part 0b — it used to sit at the top of `src/programs/bench_r3a.cpp`) shipped with the ports **UNSET** on
> purpose — nothing is guessed — and until they were typed in, button 10 refused with
> `chassis table UNSET — missing: LEFT ports, RIGHT ports`. A rebuilt program with the ports filled
> in is what the rest of this station runs on. *(Robot two's table is filled and MEASURED as of
> 2026-09-10 evening, signs included; the steps below are kept for the next chassis.)*

**D.1 — census → push → drive (with the rebuilt program on the brain)**

- [ ] **D.1.1** Run **1 DEVICE CENSUS**. Every reported port shows **MOTOR** with its side from the
      table beside it (`LEFT(table)` / `RIGHT(table)`). A port with **`?`** or a missing port is a
      finding — write it down, do not continue to D.1.3.
      *(2026-09-10, first census on robot two: **9 motors, not 10** — 11 12 13 14 15 and 16 17 18 20
      answered, **port 19 did not**; the RADIO is on port 1. That evening, after re-seating cables on
      11, 15 and 19, the census showed **all ten**, plus an **IMU on port 2** — the table's
      `measured` flag was flipped on that and `imuPort = 2` typed in. If the count ever drops below
      ten again, it is a cable: re-seat and re-run before anything else.)*
- [ ] **D.1.2** Put the robot on the floor, wheels down. Run **3 MOTOR WATCH**. **Push the WHOLE
      robot, FRONT END LEADING, about a foot.** Watch the panel: every drive port should show
      **UP** or **DOWN**. *(UP and DOWN mixed **within** a side is normal — adjacent motors on one
      gear train spin opposite ways. It is not a wiring fault.)* Touch the screen.
      → It asks **WHICH WAY DID YOU PUSH IT?** — answer honestly (**FRONT FIRST** if the end you
      chose as the front led). If you are not sure, re-run and push again, one way only.
      **DONE IF** it prints **`WHOLE-ROBOT PUSH … SIGNS CAPTURED`** and a line like
      `signed ports for DRIVE: LEFT +1 -2 +3 … | RIGHT …`. Copy that line here:
      → **LEFT: ______________________  RIGHT: ______________________**
      *(Since R3b Part 0b robot two's table CARRIES its measured signs, so after this line the
      program prints one row per port — **`table + push + AGREES`** or **`DISAGREES`** — and a
      summary. Every row AGREES = the robot still matches what was measured on 2026-09-10. Any
      **DISAGREES** is a finding — a motor re-wired, a cable moved, or a wrong table entry — and
      DRIVE here will refuse to power that robot until the table and the robot agree (so did the
      OLD "shulib Drive"; the new one, since T4 Part 3, boots from the calibration RECORD and checks
      the record against the ports instead — Stations F and G). Copy the DISAGREES rows word for
      word.)*
      If it says **PARTIAL** (a port did not move) or **single-wheel spin**, push again, firmer,
      all wheels on the floor. A port that never moves is a finding.
- [ ] **D.1.3** **Lift the robot onto blocks so NO wheel touches anything.** Check it cannot rock
      off. Have a second person ready at the battery.
- [ ] **D.1.4** Tap **10 DRIVE (POWERS)**. Read what it prints — it states the **cartridge belief**
      it is about to write to every motor and the **signed ports** from D.1.2. **If the cartridge
      it names is not the colour you read in D.0.3, STOP** and report it; do not tap YES.
      Then it asks **ARE THE WHEELS OFF THE GROUND?** — tap **YES, WHEELS UP** only if D.1.3 is true.
- [ ] **D.1.5 Drive at 3 V, wheels up.** Hold **L1** (left index finger — the UPPER button on the
      controller's top-left edge, not L2 below it) and, **while still holding it**, push the
      **LEFT stick straight up** with your left thumb. Two things at once. Watch the panel's second
      line: **L and R must read above 0.0 V** while the stick is pushed — if the footer says
      DRIVING but L and R stay at 0.0 V, the stick is not being deflected (or not being read),
      and no motor has been commanded; say so. *(2026-09-10 evening: the station ran with ten
      motors constructed, L1 registered, R1 stepped the ceiling, and L/R stayed 0.0 V — no stick
      input ever reached the mapping, so nothing turned.)* Do not press R1 until wheels turn. All
      wheels on both sides should turn the **same way, toward the front**. Release L1: everything
      stops. Try the **right stick** left and right: the sides should turn opposite ways.
      **DONE IF** the panel's footer says **DRIVING (L1 held)**, every port row reads green with a
      velocity of the same sign on a side, and nothing reads red.
      → Did all wheels turn toward the front on "forward"? **YES / NO** — if NO, which side/wheel?
      → Any port shown **RED** or a **CUT** message? Copy it **word for word**: ______________________
      *(A CUT names the port and the reason. It is the program protecting the drivetrain: five
      motors on one gear train, and a wrong sign on one fights the other four. The drive stays cut
      until you touch to exit and re-enter. A cut is a finding, not a failure of yours.)*
- [ ] **D.1.6 Raise the ceiling.** Tap **R1** once: the panel and the controller LCD both read
      **6 V**. Drive again as in D.1.5. Repeat for **9 V** and **12 V** only if 6 V was clean.
      → highest ceiling driven clean, wheels up: **3 / 6 / 9 / 12 V**
      → highest current shown on any port (the `A` column): **________ A** on port **____**
      → highest temperature shown (the `C` column): **________ °C**
- [ ] **D.1.7 Touch the screen to exit.** Every motor goes to 0 V and coasts. The summary prints
      per-port max current and temperature and whether a cut happened. It is all in
      `/usd/r3a_log.txt`.
- [ ] **D.1.8 The ground run — only after a CLEAN wheels-up run at 6 V** (D.1.5–D.1.6 with no
      CUT, the motors actually driven for at least 3 s, and the ceiling raised to 6 V; the exit
      summary prints **`ground mode is UNLOCKED`** when that is true). Then:
      1. Put the robot **on the ground**. **Clear 3 m in every direction.** A **second person at
         the battery**, ready to pull it.
      2. Re-enter **10 DRIVE**. It asks "wheels off the ground?" — tap **NO**. It then asks
         **ON THE GROUND? clear 3 m all round, a second person at the battery** — tap
         **YES, GROUND MODE** only if both are true. *(If instead it says
         `REFUSED: ground driving needs a clean wheels-up run at >= 6 V this power cycle first`,
         go back to D.1.5–D.1.6 and make one clean run at 6 V; that message is the program, not
         you.)* The panel header and the controller LCD now say **GROUND**; the ceiling is back
         at **3 V**.
      3. **3 V first.** Hold **L1**, push the left stick forward a little, **release** — it should
         roll forward a short way and stop. Then the right stick a little: it should turn on the
         spot. → forward on "forward"? **YES / NO**   turned the expected way? **YES / NO**
      4. **Then 6 V.** Tap **R1** once (panel and LCD read **6 V**); repeat step 3.
      5. **Then stop:** touch the screen. **Do not go above 6 V on the ground unless the team lead
         is present** — R1 will step to 9 and 12 V, and the program will not stop you.
      → any CUT text, word for word: ______________________
      → does the robot pull to one side on "forward"? **NO / pulls LEFT / pulls RIGHT**

> **What to send back:** D.0.3–D.0.8, the signed-ports line from D.1.2, the YES/NO and any
> CUT text from D.1.5, the numbers from D.1.6, the ground-run answers from D.1.8, and the SD
> card's `/usd/r3a_log.txt` (the exit summary of every run names its mode — WHEELS UP or
> GROUND — and whether it was clean). Results go into `R3a-PROGRESS.md` as a new phase, by the
> coordinator, from the log.

---

## Station E — the OLD "shulib Drive"  *(RETIRED at T4 Part 3, 2026-10-04 — a pointer)*

The program this station ran — the typed table's open-loop drive in slot 1 (R3b Part 0b) — was replaced
IN PLACE at T4 Part 3 by the new "shulib Drive": same name, same slot, **opposite forward**. Run
**Station F** for slot 1 now; its F.0 tells the two apart, and its WARNING and CUT paragraphs (F.1) are
this station's, reworded for the library's protections.

---

## Station F — "shulib Drive", THE DRIVER PROGRAM  *(slot 1, robot two — REWRITTEN from the code at T4 Part 3, 2026-10-04; it was "LIBRARY TELEOP", slot 2, from R3b Parts 1–3 to T4 Part 3)*

> ⚠ **"shulib Drive" is robot two's DRIVER FILE (`src/team/robot_two_driver.hpp`) on the library.**
> It builds the library's own robot — `robot::Robot`: two motor groups, the drive motors' own
> encoders as odometry, the IMU for heading, the claws — from the **calibration record** on the SD
> card, and drives it through the library's driver-control loop with the library's drive
> protections. The competition program (slot 6, Station K) runs the SAME driver file in its driver
> control. **THIS VERSION has not run on hardware.** The program it replaced in slot 1 drove robot
> two open-loop on 2026-09-17; the library program it retired ("shulib Teleop", slot 2) drove it from
> the calibration record on 2026-09-25 — eight starts ([R3d-PROGRESS.md](R3d-PROGRESS.md), "2026-09-25
> evening"); no library-commanded motion has run (R3b open). So its first run is **WHEELS UP, then the
> ground**. It POWERS THE MOTORS from the moment driver control starts, up to 12 V, and the claw
> buttons work (Station J). Bench Tests (slot 3) stays the diagnosis tool.

> ⚠ **WHICH END IS FORWARD — read this before any stick moves.**
> 1. Forward (stick up, `est x` rising) is the end that **LED the Calibrate push** saved in the
>    record. On robot two since 2026-09-25 that is the **AI Vision (camera, port 3) end — the 11/20
>    end.**
> 2. Bench Tests' **MOTOR WATCH** and its **10 DRIVE** station use the **typed table**, whose front is
>    the **15/16 end** — the other end. MOTOR WATCH's `AGREES` / `DISAGREES` rows compare against the
>    TABLE, so in **F.2.1** push the TABLE's front (15/16) first; a push led by the camera end and
>    answered FRONT FIRST shows every recorded port DISAGREES — the 180° turn, not a wiring fault.
>    (Station G.2.2 uses MOTOR WATCH differently: it records UP/DOWN per port for the front chosen in
>    G.0.4.)
> 3. The boot says `CROSS-CHECK: the MEASURED record and the typed chassis table disagree about 10
>    port(s)`. That is mostly this 180° turn (every sign flips) plus ports 14 and 19 (only the table
>    lists them). **Do not edit the table on that warning** — there is no ruling to, and the table
>    drives nothing but Bench Tests' stations now.

**F.0 — upload, and REQUIRED before any wheel moves: is this the OLD program or the NEW one?**

Build and upload from a clean, committed tree (Station G.1 says how; never from a tree an executor
is working in):

```sh
rm -rf bin .d && make ROBOT=tank PROGRAM=drive && pros upload --slot 1 --name "shulib Drive"
```

Start it from the **brain** or the laptop, never from the controller's own menu (Station A's
pretend-match trap).

- [ ] **F.0.1 OLD or NEW — required.** The NEW program and the OLD one have the same name, the same
      slot, the same `make` switch and the same beacon, and **drive toward OPPOSITE ends on the same
      stick**. A brain that still holds the old package (uploaded before T4 Part 3) drives toward the
      15/16 end. Tell them apart before the wheels touch anything:
      - the OLD one's top bar's second item reads **`adapters, NOT the motion stack`**, and its serial
        banner is **`[R3B] booted: shulib DRIVE`** — **STOP and re-upload**;
      - the NEW one's top bar reads `shulib DRIVE` and `BUILD hh:mm:ss` only, its first row is the
        drive protections' (`drive OK`), it has a **DETAILS** button, and its serial lines carry
        **`[DRV]`** (`[DRV] booted: shulib DRIVE build [shulib-robot-variant=tank-drive] ...`, then
        `[DRV] ... This is NOT the old shulib Drive (its tag was [R3B]).`);
      - the BUILD time is the build you just uploaded. (*Not* the `[BLD] robot tables: ...` line:
        every build since G1 Part 2c prints it, the old one included.)
      → OLD or NEW: **______** · BUILD time on the bar: **________**
- [ ] **F.0.2** It reads `/usd/shulib_drivetrain.bin`, the record "shulib Calibrate" (slot 4,
      Station G) writes, and checks it against the ports BEFORE anything is powered. **No record on
      the card means refused: the typed table cannot describe robot two** (its track width and
      motor→wheel ratio are UNSET in `src/robots/chassis_table.hpp`, and they are never guessed). A
      refusal paints the red refusal screen — bar `DRIVE REFUSED AT BOOT` and the BUILD time, the
      headline, the reason (with `(the full text is on the serial log)` when it did not fit), then
      `Nothing is powered. Run shulib Calibrate (slot 4) to save the record this program boots
      from.` — and serial prints `[DRV] REFUSED: <headline> -- <reason>` (since SER1 in pieces when the
      line passes 255 characters, the later ones starting `[DRV] ... `; for a refusal decided once
      the brain's devices exist — the last two headlines below — it comes through the log as
      `[ERROR][DRV] REFUSED: <headline> -- <reason>`, perhaps in two or three pieces, the later ones
      starting `... `, and the card's `drive_log.txt` holds it too); driver control paints it again
      and does nothing. The headlines:

| Headline | Why | Do |
|---|---|---|
| `NO USABLE DRIVETRAIN DESCRIPTION` | no record on the card (or one that failed its format checks), and the table cannot describe a drivetrain | run Calibrate (Station G) |
| `NO USABLE DRIVETRAIN DESCRIPTION`, reason `\| REFUSE: port(s) <n>(<kind>) carry a DIFFERENT device than the calibration recorded -- this robot was rewired; run shulib Calibrate` (or the IMU moved) — the WHOLE sentence is also on serial, in the earlier line `calibration record: DECODED, port check REFUSE -- ...`, then `boot source: REFUSE -- the calibration record does not match this robot's ports ...` | the record was taken on a different wiring. A record that fails its port check NEVER falls back to the table | re-calibrate; do not "fix" the record |
| `the library graph REFUSED to build` | the library would not build the robot — e.g. `a RIGHT drive motor cannot be made: port 13: the motor rejected its configuration: ...` | Bench Tests (slot 3) on that port |
| `the driver file REFUSED this robot` | the driver file's controls do not fit the robot built (a control naming a mechanism the robot does not declare, or of the wrong kind, or a declared mechanism no control names) — the reason names it (since MECH-A5 `src/common/driver_file.hpp` says it, in T4's words; a control whose preset, position or homing is not this robot's mechanism's is said in the library's own sentence, `DriverInputs::bind: …`) | send the line; the driver file and the mechanism table disagree |

**F.1 — what the screen shows: MAIN, then DETAILS and MORE (touch the buttons)**

Three pages. Only the visible one is painted; a row is repainted only when it changed, five times a
second at most. A touch on a button flips the page when the finger LIFTS, and only once the page has
been up half a second (its button is grey until then); a finger already down when the page changed
does nothing. Every row is at most 50 characters (the SMALL budget — an ESTIMATE, HA-136); if one runs
off the right edge, write down which, word for word.

**MAIN** (the driver's page — on the field the driver reads the controller; this page explains a stop):

| Row | What it says | What it means |
|---|---|---|
| top bar (blue) | `shulib DRIVE` · `BUILD hh:mm:ss` | which build — the time must be the build you just uploaded |
| 1 PROTECTION | green `drive OK` · amber `drive OK -- frozen 13` · amber `DEGRADED dead 18 13` · red `CUT 1 s #2: FIGHT on LEFT -- port(s) 18` · red `REFUSED: 3 drive motors dead -- 0 V for the run` (and amber `drive OK -- 2 over-current warnings` only if the over-current cut were switched off — it is ON) | the library's drive protections, now: every drive motor answering; a reading that froze this tick (dropped 250 ms later if it stays frozen — or a train stopping); ports dead (at boot, or dropped mid-run) while the stick still drives; a 1 s cut in force (both sides 0 V, then it re-arms by itself; the words are the serial `CUT:` line's); more than two dead — 0 V until the program is restarted |
| 2 controller | `controller: CONNECTED` · red `NO CONTROLLER -- the drive gets 0 V` | a controller that is not linked reads 0 on every stick |
| button | `DETAILS` (bottom right) | the second page |

**DETAILS** (`shulib DRIVE  DETAILS` — "shulib Teleop"'s diagnostics, the team lead's R5):

| Row | What it says | What it means |
|---|---|---|
| 1 (amber) | `THIS VERSION has not run on hardware (T4 3)` | this program version's run status (the whole sentence is on serial) |
| 2–3 source | green `MEASURED record from the SD card, verified against` / `the ports` — or amber `MEASURED record \| DEGRADED: dead 13 -- driving` / `without them (left dead 0, right dead 1). ...` — or amber `TYPED TABLE -- unverified since the rebuild; run` / `shulib Calibrate` | where the drivetrain numbers came from, and which ports were dead AT BOOT. A reason longer than two rows ends at its last whole sentence with ` ...`; the whole reason is the serial `port check:` line |
| 4 claws | `CLAW 6 HOLD 34C \| 7 HOLD 33C \| CLAW CLOSED` — amber when a claw is `ABSENT` or the row ends `FREED` | each pivot by its port — `FWD` / `REV` while its button is held, `HOLD` when stopped, `ABSENT` when not built — and its temperature (its last good reading: it freezes, unmarked, if the motor stops answering — J.3.3); then the pneumatic claw, `OPEN` / `CLOSED` / `ABSENT`. Station J reads this row |
| 5 faults | `faults: none`, or (red) `FAULT n=1 last MOTOR_GROUP_DISAGREE` — `first` and `last` both when they fit 50 characters | the fault latch, by name (serial has every fault). `MOTOR_GROUP_DISAGREE` = a motor fought its group for 250 ms — the protections CUT the drive for 1 s on the same verdict (MAIN's row says so) |
| 6 cmd | `cmd vx  +20.0 in/s  w +0.50 rad/s` · `  NO CONTROLLER` (amber) | what the sticks ask for, in SPEEDS (not volts) |
| 7 est | `est x   +12.3  y    -0.4  hdg  +45.2 DEADRECKON` | the library's **estimate of where the robot is**, from the drive encoders and the IMU, from where it booted; `UNINIT` (the IMU still calibrating, ~2 s), `DEADRECKON` (normal here), `DEGRADED` (amber: read the fault row) |
| 8 driver | `driver ARCADE BODY  field-centric OFF` | the stick mapping — left stick drives (up = forward, WHICH END), right stick turns — from the driver file's `kFeel`; field-centric is OFF there |
| 9 xcheck | `hdg xcheck +0.0003 rad/tick  batt 12.90 V` | the heading the wheels imply vs the IMU's, per tick — near 0 rolling cleanly; then the battery |
| buttons | `MORE` (bottom left) · `BACK` (bottom right) | |

**MORE** (`shulib DRIVE  MORE`):

| Row | What it says | What it means |
|---|---|---|
| 1 L | `L  +6.2 V  4/4 present  0 disagree` (`  dead 18` when one is) | the LEFT group: volts sent, motors answering / motors in the record, motors disagreeing now, and its DEAD ports (at boot or dropped mid-run). Green all present · amber one missing · red one disagreeing |
| 2 R | the same for RIGHT | |
| 3 travel | `L/R travel  +0.20  +0.20 in (last tick)` (grey) | how far each side rolled in the last tick |
| 4 protections | `protections DRIVING  cuts 0` (`  dead 18 13` when any) | the protections' mode and the cuts this start of driver control |
| 5 last cut | `last cut: none this entry` · amber `last CUT: FIGHT on LEFT -- port(s) 18` | the last cut's words, whole |
| 6 peaks | `peak A this entry: L 2.31  R 1.80` | each side's highest motor current since driver control started — F.6 (ii) reads it |
| button | `BACK` | MAIN |

**The controller** (3 lines of 19 characters): row 0 `LIB 12V B12.9V` (the library's 12 V cap and
the battery); row 1 `hdg  +45.2 deg` — or `dead 18 13` while any drive port is dead; row 2 the FIRST that
holds of `REFUSED see brain`, `CUT see brain`, the last fault by name, `no controller 0V`, `DEGRADED see
brain`, `CLAW ABSENT`, `ok` — a state in force outranks an old fault. ONE line is written at a time,
row 2 first, at most every 0.1 s (PROS does not apply faster controller text), so the three appear over
about 0.3 s.

**On serial only:** the banner (`[DRV] booted: shulib DRIVE ...`, the run status — two lines since SER1,
the second beginning `[DRV] ... `, one sentence wrapped — and `BUILD STAMP`), the
boot source's lines, then everything from the brain's devices on — also in the SD copy (F.7):
`log copy: /usd/drive_log.txt (one file per boot) -- ...` (or `... did NOT open: NO SD CARD --
serial only ...` with no card, `... did NOT open, though an SD card is in (full? write-protected? a
folder of that name?) -- serial only ...` with one), `library graph BUILT from the CALIBRATION RECORD ...`, the port check, the
speed caps and gains (`facade alive: ... budgets lin 60 in/s ang 6.00 rad/s wheel 60 in/s; ff kS 1.00
kV 0.1714 ...` on the 2026-09-25 record — **placeholders**, R5 measures them), the stall note, the two
dead-port lines (`dead-port tolerance at BOOT ...`, `dead-port tolerance MID-RUN ...`), `driver
control: robot two's driver file ... re-zero button Y; mode ARCADE BODY`, `drive protections (the
library's, in every program): ...` (its numbers generated from the driver file's loop config), the
mechanism lines (`[MECH]`, Station J.3.1 — since MECH-A5, 2026-10-29), and `opcontrol(): robot two's
driver file is live -- ...`.

**What "feels different" means.** The stick becomes a **speed request** through the library's
pipeline (speed caps → wheel speeds → a feedforward guessing the volts → the battery ceiling), and the
guesses are placeholders, so full stick may not be full volts and a turn may feel capped. **That is
expected** — it is what R5 measures. The driver-feel layer's defaults: a SMOOTH deadband around the
centre, no expo curve, no rate limit (the driver file's `kFeel`, HA-112). Write down what it felt like;
do not retune anything on the spot.

**What the estimate on screen means.** Nothing on this robot tells the library where it is on the
field: no GPS, no tracking wheels, and the camera is not read. `est` is dead reckoning from where it
booted; it drifts as the wheels slip. **On the 2026-09-25 record the scale read about 8 % short** (a
taped 48 in read `est x` 44), so one tile reads about 22, not 24, until a new calibration.

**What "stall check: NO independent source" means (serial).** On this robot the odometry IS the
wheels, so the library cannot see a stuck robot whose wheels spin. **If the robot pushes against
something and the wheels spin, the library will not know — you will.** Let go of the stick.

**What a WARNING means — a dead drive port.** A recorded drive port that is EMPTY at boot is
**tolerated** under the boot check's rule (a side below three answering, or more than two dead in
total, refuses the boot): DETAILS' source rows say `MEASURED record | DEGRADED: dead <ports> --
driving without them ...`, MAIN says `DEGRADED dead <ports>`, MORE's side row `3/4 present  dead
<port>`, and the controller `DEGRADED see brain` with `dead <ports>` on row 1. **A port that dies
AFTER boot**: a drive motor whose readings freeze while its side turns — MAIN `drive OK -- frozen
<n>` for up to 250 ms — is dropped (serial `[WARN][TEL] WARNING: port <n> (LEFT|RIGHT) went ABSENT at
runtime: reads frozen for 250 ms while its side answered -- ...`, then `degradation re-evaluated:
...`), MAIN turns to `DEGRADED dead <n>`, and the robot drives on. It stays dead until the program is
restarted. **More than two dead in total** stops the drive — MAIN red `REFUSED: 3 drive motors dead
-- 0 V for the run`, serial `[ERROR][TEL] REFUSED: more than 2 motors dead in total -- 0 V for the
rest of the run ...` — and every later start of driver control is refused too until the program is
restarted. Nothing here has run on a robot: this is the host-tested rule (A4 register HA-175). A
warning is a **finding**: write down which port, and re-seat its cable before the next run.

**What a CUT means.** The protections cut **both sides to 0 V for 1 s**, then driving resumes by
itself, when a drive motor fights its side for 250 ms (the verdict that latches
`MOTOR_GROUP_DISAGREE`), or draws more than **2.4 A** for 250 ms while its side is driven above 1 V
(pushing a wall is expected to do that, and a full-stick reversal at speed or a launch from rest might
— A4 register HA-176, invented: a `CUT:` line in ordinary driving is exactly what to send back). MAIN
shows it red for that second (`CUT 1 s #<n>: ...`), the controller `CUT see brain`; serial `[WARN][TEL]
CUT: FIGHT on LEFT -- port(s) <n> -- both sides 0 V for 1 s, cut #1, then re-arm` (or `CUT: port <n>
(LEFT) over 2.4 A for 250 ms ...`), one `  at the cut:` line per motor, then `re-armed after cut #1 (1
s at 0 V)`. When the stick is let go after a drive, serial may add `drive: LEFT peak <A> A (port <n>)
over a <s> s drive, no cut`. **A cut that repeats is damage in progress: STOP, read MORE's `last`
row, and send the serial lines word for word.**

**F.2 — the first run of THIS version: WHEELS UP**

- [ ] **F.2.1** Bench Tests first, once: **1 DEVICE CENSUS** shows the record's eight drive ports
      (**11 12 13 15 16 17 18 20** on the 2026-09-25 record) as MOTOR and the IMU on port 2 (14 is
      the known extra motor the record does not name). Then **3 MOTOR WATCH**: push the whole robot
      with the **TABLE's front — the 15/16 end — leading**, and answer **FRONT FIRST** (WHICH END IS
      FORWARD, item 2). If the record's 180° turn is the whole story (item 3), the eight recorded
      ports read **AGREES** and 19 shows no motor; anything else is a finding.
      → every DISAGREES row, and what it shows for 14, word for word: ______________
- [ ] **F.2.2** Robot on blocks, no wheel touching anything, 3 m clear, second person at the
      battery. F.0.1 says NEW. Start **"shulib Drive"** (slot 1) from the brain. Serial (if a laptop
      is capturing, G.7): `calibration record: DECODED, port check OK` and `boot source: RECORD`.
      MAIN reads green `drive OK`; DETAILS' `est` leaves `UNINIT` after ~2 s (the sticks work during
      it); MORE's L/R rows read **`4/4 present  0 disagree`** on the 2026-09-25 record.
- [ ] **F.2.3** Ease the **left stick forward** — a little first: near the centre it eases in
      smoothly. All wheels turn **toward the record's front — the camera end (11/20) on the
      2026-09-25 record** (after a new calibration, the end pushed first in G.4); on MORE, `L` and `R`
      volts rise together and `L/R travel` is positive; on DETAILS `est x` climbs and `hdg xcheck`
      stays near 0. Release: 0 V, and the drive **COASTS** to a stop (the driver file's
      `releaseBrakeMode`; read from the code, not yet seen on the robot). **Right stick** left/right:
      the sides turn opposite ways and `hdg` moves the way the robot turns (push RIGHT = clockwise =
      `hdg` going DOWN).
      → all wheels toward the camera end on "forward"? **YES / NO** — if NO, which side:
      → `hdg` goes DOWN on a right-stick RIGHT push? **YES / NO**
      → MAIN anything but `drive OK`, a `disagree` above 0, or a fault — word for word: __________
- [ ] **F.2.4** Full stick for a few seconds, wheels up: the highest `L`/`R` volts (MORE) and the
      battery (DETAILS), and MORE's `peak A` row after you let go.
      → **L ______ V  R ______ V  at B ______ V** · **peak A: L ______ R ______** (the first
      current trace of robot two's drive: HA-176's evidence)

**F.3 — the claws:** Station J, in this program.

**F.4 — the ground run — the rule:** 3 m clear in every direction, a second person at the battery,
stop on ANY red.

- [ ] **F.4.1** Robot on the floor, start the program again, **small stick inputs first**. Forward one
      tile (toward the camera end), stop: read `est x` on DETAILS (about 22 on the 2026-09-25 record's
      scale). Back to the start: `est x` near 0? Spin once around: `hdg` back near where it started?
      → forward on "forward"? **YES / NO** · turned the expected way? **YES / NO**
      → `est x` after one tile forward **______** · after returning **______** · `hdg` after one
        full spin **______ deg** · pulls to one side? **NO / LEFT / RIGHT**

**F.5 — the feel, against Bench Tests' 10 DRIVE** (Station D — ruling M4's raw-volts baseline, the one
left now that the old "shulib Drive" and "shulib Teleop" are gone). Its differences, so the comparison
is fair: it runs **WHEELS UP only**; it needs **L1 held** (the dead-man); it starts at a **3 V ceiling**
that **R1** steps to 12 V; and it drives toward the **TYPED TABLE's front, the 15/16 end** — the
opposite end. So F.5 compares the FEEL, wheels up, at the 12 V ceiling, **never the direction**.

- [ ] **F.5.1** Wheels up: 10 DRIVE at its 12 V ceiling, then this program: small stick movements
      around the centre — **smoother here / the same / worse**? Full stick — **more / the same /
      less**? Anything else different? ______________________________________________

**F.6 — the protections** (nothing here has run on a robot; host-tested only)

- [ ] **F.6.1 (i) — ONLY with the team lead's OK.** Wheels up, half stick forward, **pull one drive
      motor's cable**. Expect MAIN `drive OK -- frozen <port>` (amber) within about 50 ms, then `DEGRADED
      dead <port>` about 250 ms later, the serial `went ABSENT at runtime` line ONCE, the robot driving
      on, and **no cut**. Plug it back in: it stays DEAD until the program is restarted (latched). This
      is A4 register HA-175's first evidence.
      → frozen shown? **YES / NO** · `DEGRADED dead ____` · any `CUT:` line, word for word: __________
- [ ] **F.6.2 (ii) — THE PUSH CHECK, REQUIRED before the competition program drives a match** (the
      team lead's N1; Station K.8 runs it in that program). On the ground, full stick into a wall for
      3 s, then let go. Record whether it CUT (MAIN red `CUT 1 s #1: ...`, serial `CUT: port <n> (...)
      over 2.4 A for 250 ms`), and the highest current: MORE's `peak A` row, and serial's `drive: LEFT
      peak <A> A (port <n>) over a <s> s drive, ...` line. A V5 motor pushing is believed to sit near
      its 2.5 A limit (the PROS default; A4 register HA-154, not measured), above the 2.4 A cut — so a
      CUT here is the expected answer, and it means a push in a match cuts the drive 1 s in every
      1.25 s. This is A4 register HA-176.
      → CUT? **YES / NO** · `peak A: L ______ R ______` · every `CUT:` line word for word: __________
- [ ] **F.6.3 (iii)** **No fight is staged on purpose.** The host test plant covers it.

**F.7 — the disable, and what to send back**

- [ ] **F.7.1** Turn the program off from the brain (or the field disables it: the loop exits cleanly,
      the drive 0 V + Brake, and serial says `opcontrol(): field DISABLED -- the driver file's loop
      exited cleanly after cancel() (0 V + Brake); each claw pivot made safe, the pneumatic claw left as
      it was; N screen prints so far`, preceded by one line a side, `drive at the disable: LEFT, cuts 0
      | port 16 <A> A <C> C | ...` — printed only if the loop sees the disable before PROS ends the
      task). Whether VEXos is cutting motor output by the time a disable's stop goes out is UNKNOWN
      (HA-145): a disable is no test of Brake.
- [ ] **F.7.2** Copy the SD card's **`/usd/drive_log.txt`** off before power-cycling (one file per
      boot: the next start replaces it). It holds this program's log lines and summaries from the
      brain's devices on — the boot lines, every dead port, every cut, the peaks, the disable lines,
      and, for a boot refused once the devices existed, the `REFUSED:` line — but never the per-tick
      records serial carries; the banner and the boot source's lines (and a refusal decided there,
      before the devices: the card then keeps the previous boot's file) are on serial only. **One
      case ends the card's copy** (`diag/log_copy_sink.hpp`, "A writer that never comes back"): the
      disable lines are logged just as the loop sees the disable, the moment PROS is believed to
      delete the task (HA-161 (a), reasoned). If the deletion lands while one of them is being
      written to the card, the card's copy ENDS there for the rest of that boot, with no line
      saying so — a second Timed Run in the same boot then leaves nothing on the card — and serial
      keeps every line. Compare the card's last lines with serial's at each disable.
      → a card copy that stopped at a disable while serial ran on? **NO / YES (at which line:
      ______)**

> **What to send back:** F.0.1's answer, the YES/NO answers and numbers from F.2–F.6, every MAIN row
> that was not `drive OK` and every serial `CUT:`, `went ABSENT` or `REFUSED` line word for word, any
> screen row that ran off the edge, the card's `drive_log.txt`, and the serial capture. The coordinator
> records them in the development log; they are the first hardware evidence for THIS "shulib Drive",
> for the driver-feel layer, and for the drive protections (HA-175, HA-176).

---

## Station G — CALIBRATE, then DRIVE FROM THE RECORD  *("shulib Calibrate" + "shulib Drive", robot two — added R3d Part 2, 2026-09-18; REWRITTEN from the code at R3d Part 2F-2, 2026-09-24; its driving half "shulib Teleop"'s until T4 Part 3)*

> ⚠ **Written for you to run ALONE.** Two things happen here. First, **the robot measures its own
> drivetrain**: which ports drive, which way each motor is wired, which side each is on, how far
> each side travels per radian of motor shaft, and how wide its track effectively is. Second,
> **"shulib Drive" boots from that measurement** (so do the Auton Tester and the competition program;
> the typed table cannot describe robot two). Both programs POWER
> THE MOTORS. Work through this in order; where a box asks for a number, write the number down — a
> run nobody recorded did not happen.
>
> **Why this exists:** robot two's drivetrain was REBUILT, so the ports and signs typed into
> `src/robots/chassis_table.hpp` are no longer trustworthy, and nobody wants to re-measure them by hand.
>
> **What has run, and what has not.** The FIRST version of "shulib Calibrate" ran five starts on
> robot two on 2026-09-24 and finished none: its one 3 V motor pulse was too weak for a five-motor
> side, two starts ended on a DONE touched before any push, and its screen could not be followed.
> **The version this station describes (R3d Part 2F: one push, a ramped pulse, a screen rebuilt
> for reading at arm's length) FIRST RAN on robot two the evening of 2026-09-24:** run 1 got
> through the push and every pulse and STOPPED on `SPIN_DISAGREE` (the two spin directions' track
> widths 20.3 % apart; nothing saved); run 2 finished by the team lead's report, with no serial
> capture — [R3d-PROGRESS.md](R3d-PROGRESS.md), RUN 1 / RUN 2. Every threshold in it is still
> invented for a first run (A4 register HA-136). **The screen's text sizes are DERIVED, not measured:** the
> character budgets (18 LARGE, 36 MEDIUM, 50 SMALL per line) come from the longest lines earlier
> programs showed legibly on this brain; Bench Tests button **9 SCREEN RULER** is what would
> measure them. If a line on the calibrate screen runs off the right edge, write down which one,
> word for word — that is a finding, not your mistake.
>
> **Robot two, as of 2026-09-24:** its wheels are **OMNI on both sides** (team lead). **Port 19 is
> UNPLUGGED** for the session (it was dropping off the bus), so the drivetrain is nine motors:
> **LEFT 11 12 13 14 15, RIGHT 16 17 18 20.** Port 18 travels 22–33 % short of its side-mates
> under a push (HA-130) and may have a slipping gear.

**G.0 — before you touch a program**

- [ ] **G.0.1** Charged battery in. **A FAT32-formatted SD card in the brain's card slot** — the
      calibration is written to it; without a card the run still happens but the answer is on
      screen and in the log only. (Bench Tests button **5 SD CARD PROBE** checks a card.)
      **Before any Calibrate run on a card that already holds a record, copy
      `shulib_drivetrain.bin` off the card on a laptop, and date the copy.** Until the SAFE-SAVE fix
      to Calibrate's save is in the build, a NOT SAVED run can erase it, and that copy is the only way
      back (added 2026-09-26).
- [ ] **G.0.2** **3 m clear in every direction** and a **second person at the battery** for
      everything from G.4 on. The calibration JERKS each drive motor on its own and then SPINS THE
      ROBOT IN PLACE, twice, under power.
- [ ] **G.0.3** **A tape line on the floor, straight, at least 48 in long** (two field tiles),
      with a START mark and a mark **48 in** further on — and room past it for the robot's own
      length. There is ONE push, and it is always 48 in: the program has no distance question.
- [ ] **G.0.4** **Decide which end is the FRONT, before you push anything.** Whatever end you push
      first and call FRONT becomes the front: "shulib Drive" will drive "forward" toward it. (The
      typed table calls the end with ports **15 and 16** the front; the rebuild may have changed
      that — the 2026-09-24 census pushes came out with every sign inverted. The record on the card
      since 2026-09-25 took the OTHER end — the camera (AI Vision, port 3) end, 11/20 — as its
      front, so a new record pushed from the 15/16 end reverses "shulib Drive"'s forward: Station F, WHICH
      END IS FORWARD. There is no ruling on which end; write down the one you chose.) → the FRONT is
      the end with ports **______**.
- [ ] **G.0.5** Read the **DO NOT** list at the end of this sheet. The two that bite here:
      **never start a program from the CONTROLLER's own menu** (HA-128 — it arms a pretend match
      and the program never gets driver control; its one exception, slot 6 at Station K, is not
      this station's), and **do not "correct" a reading** to what this sheet expects.

**G.1 — upload the programs (lab computer, brain on micro-USB)**

`rm -rf bin .d` before each `make` is not optional: `make` is incremental, and a stale object file
is how a build wears the wrong label. Run these in order, one at a time (the tester and the
competition program have their own stations' lines, M.1 and K.1 — the same form).

**Build on the laptop that holds the committed tree, and do not `git pull`** (the remote is behind
this laptop). Start from a CLEAN tree (`git status` prints nothing). If an executor is working in the
main tree, build in a separate worktree at the commit you mean instead (`git worktree add --detach
../hw-build <commit>`), so its in-progress edits cannot reach a package.

```sh
cd ~/projects/shulib && git status --short    # must print nothing

rm -rf bin .d && make ROBOT=tank                     && pros upload --slot 3 --name "Bench Tests"
rm -rf bin .d && make ROBOT=tank PROGRAM=drive       && pros upload --slot 1 --name "shulib Drive"
rm -rf bin .d && make ROBOT=tank PROGRAM=calibrate   && pros upload --slot 4 --name "shulib Calibrate"
```

*(`PROGRAM=library` — "shulib Teleop", slot 2 — was RETIRED at T4 Part 3: `make` stops with an error
naming the retirement. Slot 2 is the Auton Tester's now, Station M.)*

- [ ] **G.1.1** All three uploads reported success. Every program shows its **BUILD** stamp; on
      "shulib Calibrate" it is the time at the right end of the coloured bar across the top —
      `BUILD hh:mm:ss` — and the serial banner has the whole stamp (`BUILD STAMP: <date> <time>`).
      **If a stamp is not the build you just uploaded, the upload did not land** — re-upload
      before trusting anything. → time seen on Calibrate's top bar: **________________**

**G.2 — the census and MOTOR WATCH first**

Do this BEFORE calibrating. It takes three minutes and it is how you tell a calibration that
refused from a robot that is unplugged.

- [ ] **G.2.1** Run **slot 3 "Bench Tests"**, button **1 DEVICE CENSUS**. Expected for robot two
      tonight: **nine MOTORs (11–18 and 20)**, the IMU on port 2, the radio on port 1, the AI
      Vision on 3 and the distance sensors on 4 and 5. *(Once the claw pivots are plugged in
      (T3, 2026-09-26): also MOTOR on **6 and 7** — eleven motors. Calibrate leaves those two
      alone; Station J.)*
      → motors found **____** · IMU on port **____** · the full list, as printed:
        ______________________________________________
- [ ] **G.2.2** Button **3 MOTOR WATCH (live)**: push the whole robot **FRONT-FIRST** (the front
      you chose in G.0.4) along its length, then touch to save. Write down UP/DOWN per port — it is
      the independent check on the signs the calibration will capture.
      → per port: ______________________________________________
- [ ] **G.2.3** If the census shows a drive motor missing, **re-seat that cable and re-run the
      census** before going on.

**G.3 — OPTIONAL: a wheels-up run (it is SUPPOSED to stop)**

The team lead skipped this on 2026-09-24 ("it gets confusing"), and it is optional: it proves only
that the program stops when the robot cannot turn. If you do it: robot **on blocks, no wheel
touching anything**, start **slot 4**, and turn every wheel by hand, both sides, the same way,
several turns, in place of the push (DONE appears only once the shafts have turned at least one
wheel turn and everything is still again). Answer FRONT/BACK as if you had pushed it. At the first
drive port that motor's wheels jerk three times (3, 6, 9 V) and — because the robot cannot turn
on blocks — it **stops**, red, `STOPPED -- MOTORS OFF`, usually with `IT ROLLED` /
`it moved but did not turn` / `on the FLOOR? then report the port` (the gear train turned but the
robot did not), or `NOTHING MOVED` / `even the top pulse moved nothing` / `on the FLOOR? a wheel
stuck?`. **That is the program working.** → skipped / stopped with: ______________________

**G.4 — the run, on the floor**

Robot on the floor, **front at the tape's START mark, facing along the tape**, 3 m clear, second
person at the battery. Start **slot 4 "shulib Calibrate"** from the **brain's** screen.

**Reading the screen.** The coloured **bar across the top** is the state: **BLUE** = nothing is
powered, **AMBER** = motors are powered — **stand clear**, **GREEN** = saved, **RED** = stopped.
Its left end names the step (`1/5 IMU` … `5/5 SPIN`). Under it: the instruction in LARGE letters,
then ONE big live number, then a line that says what happens next. **Buttons:** **ABORT** is the red
button at the **bottom right** on every running screen, and in the two AMBER steps it widens to the
**whole bottom row**. **DONE** is the green button at the **bottom left**, and it only appears when
the push is good. **FRONT FIRST / BACK FIRST** and **RUN AGAIN** are big boxes in the **middle** of
the screen — never where DONE was, so a second tap on DONE cannot answer them. **Every new button
except ABORT is GREY for half a second before it works** — wait for its colour; ABORT always
works. The screen is resistive: a
**firm press**, and it reads the moment you **LIFT** your finger.

| Top bar | The screen says | You do |
|---|---|---|
| (blue) `CALIBRATE` | `HANDS OFF` · `STARTING UP` · `finding the motors and the IMU` | nothing — a second or two |
| (blue) `1/5 IMU` | `HANDS OFF` · `IMU WARMING UP` · the seconds counting · `NEXT: push it 48 in along the tape` · ABORT | **do not touch the robot**, about 2 s |
| (blue) `2/5 PUSH` | `PUSH IT FORWARD` · `48 IN ALONG TAPE` · big amber `0 of 9 turned` · `drift +0.0 deg` · `DONE shows once the motors turn` · ABORT | **Push the whole robot FORWARD, front first, straight along the tape, from START to the 48 in mark. Then LET GO and step back.** The big number counts drive motors whose shafts have turned — **9 of 9** tonight (a motor that is not on the drivetrain never counts, and does not need to). The bottom line walks you through it: `keep pushing -- the full 48 in` (not far enough yet) → `let go -- DONE shows when it stops` (far enough; the robot or your hand is still moving) → green `touch DONE -- then say which end`, and the big number turns **GREEN**. **Straight matters:** if the drift line turns **red** — `drift +3.4 deg -- TOO CURVED` — the push turned the robot more than 3 degrees, and DONE will stop the run (`PUSH CURVED`); touch RUN AGAIN and push again, straighter |
| (blue) `2/5 PUSH`, DONE shown | the same, with a green **DONE** bottom left (grey for half a second first) | touch **DONE** once. **DONE appears only when the push is good:** at least 2 motors turned, their shafts turned at least one wheel turn (6.28 rad), and the robot and every motor are STILL. If it never appears: push the full 48 in; take your hands off the robot |
| (blue) `3/5 WHICH END` | `WHICH END FIRST?` · two big boxes in the middle, `FRONT FIRST` and `BACK FIRST` (grey for half a second) · amber `NEXT: motors twitch -- stand clear` · ABORT | touch **FRONT FIRST** if the front led (as G.0.4 and the push asked), **BACK FIRST** if the back led. Answer what you actually did — a wrong answer reverses every motor and nothing later can catch it. Then **step back: the next screen powers the motors** |
| **(AMBER)** `4/5 PULSES` | `STAND CLEAR` · `TWITCHING MOTORS` · `PORT 12  6 V` (which motor, at which level) · `settling -- hands off` between jerks, `3 of 9  yaw -1.2 of 2.0 deg` during one (green once past 2 degrees) · the sides as they are decided, e.g. `L 11 12  R 16` (`OUT 18` for a port left out) · **ABORT is the whole bottom row** | **hands off, stand clear, touch nothing but ABORT.** One drive motor at a time is pulsed for 0.4 s — first at **3 V**; if the robot turned less than **2 degrees**, again at **6 V**, then **9 V**. Two degrees is small: a point 7 to 10 in from where the robot turns moves about **a quarter to a third of an inch**. At 9 V it is a visible **jerk** of one corner, not a drive. Between jerks it waits for the robot and every motor to be still — **any touch on the robot delays it**. The side of each motor is read from which way the robot turned. Nine motors take about a minute |
| **(AMBER)** `5/5 SPIN` | `STAND CLEAR` · `SPINNING` · `CCW 0.8/2 TURNS` (then `CW ...`) · `at 3 V`, or amber `at 6 V -- stepped up` · `NEXT: it saves -- stay clear` · between the two directions: `SETTLING` · `hands off -- it spins next` / `hands off -- it spins again` · **ABORT is the whole bottom row** | ⚠ **STAND WELL CLEAR.** It spins the robot in place **two full turns counter-clockwise, then two clockwise**, at 3 V. **Two safety stops:** if it spins the **WRONG way** by more than **15 degrees** it stops at once (`SPUN WRONG WAY` / `the sides came out swapped`); if it has not turned **5 degrees within 2 s**, it steps up to **6 V** once, and if it still does not turn within 2 s it stops (`COULD NOT SPIN`). A spin that starts but takes over 20 s also stops (`SPIN TOO SLOW`). 3 m clear, second person at the battery, finger ready |
| **(GREEN)** `SAVED` / **(RED)** `NOT SAVED` | the numbers (G.5) · RUN AGAIN, the big box middle right | read G.5 |
| **(RED)** `STOPPED -- MOTORS OFF` | a big red headline, what happened, what to do, `PORT n` when a port is to blame, the first lines of the reason in small grey, `the full reason is on the serial log` if it is longer · RUN AGAIN | every motor is at 0 V and coasting — the robot can be pushed. **Copy the headline and the PORT line.** Fix what it says, then touch **RUN AGAIN** |

**LEFT OUT — a motor that turns but moves nothing.** If a motor's own shaft turns (20 degrees or
more) at 9 V while no other drive motor moves more than 5 degrees, its gear is **slipping or stripped** — port
18's suspected fault. The run does **not** stop: the port is **LEFT OUT** — never powered again in
this run, in no side of the description — and the run goes on with the others. The pulses screen
shows it as `OUT 18`; the finished screen as amber `LEFT OUT 18` and the sentence
`port 18 turns but does not move the robot -- its` / `gear is slipping; it is left out`. The run
still SAVES, WITHOUT that port; "shulib Drive" then drives without it (a motor on a port the record
does not name is NAMED at its boot — `NEW motors not in the calibration: 14` in the port-check line,
as on 2026-09-25 — never a refusal; since T3 Part 2, with the claw motors plugged in, the same
line goes on `; claw pivots (declared, not drive motors): 6 7` — they are expected, not NEW). **Fix the gear, then calibrate again** —
the robot has one motor less on that side until you do.

**What a stop means — the words on the red screen.** `IT ROLLED` (`it moved but did not turn`):
at 9 V one motor moved its gear train but the robot did not turn — on the floor? A robot that
rolls instead of turning is a finding; report the port. `NOTHING MOVED`: even 9 V moved nothing —
on the floor? a wheel jammed? `SIGN MISMATCH`: the push and the pulse disagree about which way
that motor turns — check its cable and gears. `NOT SETTLING` (`something kept moving before a
pulse`): the robot or a motor never came to rest — hands off. `WRONG MOTOR MOVED`: a motor that did
not move on the push moved during a pulse. `PUSH NOT SEEN` / `PUSH CURVED` / `PUSH TOO SHORT`: the
push. `ONE SIDE ONLY`: every motor turned it the same way — check the wiring. `MOTOR FIGHTING`: one
motor fought its side during the spin — check that port before driving. `YOU STOPPED IT`: ABORT.

**RUN AGAIN.** On the finished screen and on the stopped screen, **RUN AGAIN** is the big blue box
in the middle, on the right (grey for half a second first). It starts a **completely new run** in
the same program — a new calibrator, nothing carried over from the last run; the IMU does not warm
up again. Put the robot back at the START mark first. **Every run that says SAVED REPLACES the
record on the card**. **A run that STOPS writes nothing. A run that says NOT SAVED may already have
ERASED the record that was on the card** (found 2026-09-26; the program's own NOT SAVED text says the
opposite and is wrong until the SAFE-SAVE fix to Calibrate's save is in the build). Copy the record off
the card first (G.0.1).

**G.5 — how to know it calibrated correctly**

- [ ] **G.5.1 SAVED.** The top bar is **GREEN** and says `SAVED`, and the small line reads
      `NEXT: copy these, then RUN AGAIN to check`. On serial: `SAVED -- /usd/shulib_drivetrain.bin:
      written, flushed, RE-READ and compared byte for byte`. **RED `NOT SAVED`** with
      `NOT SAVED -- copy these numbers NOW` means the numbers are on screen and in the log only — the
      serial log's `NOT SAVED -- <reason>` line says why (`no SD card in the brain`, `the card refused
      the write`, …). → **SAVED / NOT SAVED** · reason if not: ______________________
- [ ] **G.5.2 The numbers.** The screen shows the ports per side (`LEFT 5  RIGHT 4`), both scales
      (`ipr L 1.37500 R 1.37500`), the track width (`track 13.700 in`) and any `LEFT OUT` port. The
      serial log's MEASURED line has them in one line — **copy it EXACTLY**:
      `MEASURED tank BLUE 600 rpm: L5/R4 ports, ipr L1.37500 R1.37500, W 13.700 in` — and under it
      the signed ports of each side (`LEFT  (signed, AS MEASURED): ...`). → the MEASURED line:
      ______________________________________________
      Check each against what the robot IS:
      - **Ports per side — which side is LEFT depends on the front you chose (G.0.4).** Robot two
        tonight, with the **15/16 end** first: **LEFT 11 12 13 14 15** (five), **RIGHT 16 17 18 20**
        (four; port 19 unplugged). With the **camera (11/20) end** first the sides swap: **LEFT 16
        17 18 20**, **RIGHT 11 12 13 14 15**. A motor on the wrong side, or a split other than 5/4
        (4/5 camera end first), is a finding — write it down. (A LEFT OUT port makes that side one
        shorter; see above.)
      - **`ipr` near 1.375, and L ≈ R.** A 2.75 in wheel driven 1:1 by its motor moves 1.375 in
        (its radius) per radian of motor shaft. **A different value means gearing** and tells you the
        ratio: wheel turns per motor turn = ipr ÷ 1.375 (e.g. 2.29 → the wheel turns 5/3 as fast
        as the motor). L and R should agree closely; a big L/R difference is a finding (a wheel of a
        different size, or a slipping side). The wheel size is itself only REPORTED (2.75 in, not yet
        put to a ruler on this robot).
      - **The track width** should be about the **tape-measured distance between the left and right
        wheels' centres**, or **somewhat larger** — a skid-steer scrubs when it turns, so the width it
        acts like is wider than the one a tape sees (HA-135); omni wheels scrub less, so expect it
        close. → tape centre-to-centre **______ in** · calibrated **______ in**
- [ ] **G.5.3 RUN IT A SECOND TIME** — touch **RUN AGAIN** (robot back at START) and do G.4 again.
      **A number that does not reproduce is not a measurement.**
      → run 1 ipr L **______** R **______** W **______**
      → run 2 ipr L **______** R **______** W **______**
      The library CLAIMS the two scales agree within **2 %** and the widths within **3 %** (both
      invented, HA-136; this is the first evidence either way). **If they disagree by more than
      that, that is the finding — write it down; do not re-run until it looks better.**
- [ ] **G.5.4** The USB serial log has the whole record (G.7): the push table, every pulse, every
      spin direction, and the MEASURED line.

**G.6 — "shulib Drive" from the record**

- [ ] **G.6.1** Start **slot 1 "shulib Drive"** (Station F.0.1: the NEW one). At boot the serial log
      says `calibration record: DECODED, port check OK` and `boot source: RECORD`, and DETAILS' two
      source rows (rows 2 and 3) read **`MEASURED record from the SD card, verified against`** /
      **`the ports`**, in green.
      → which did it say: **RECORD / TYPED TABLE / REFUSE** ______________________
      → if it refused, the headline and the reason, word for word: ______________________
- [ ] **G.6.2** If a recorded drive port is dead the source rows read (amber)
      `MEASURED record | DEGRADED: dead <ports> -- driving` / `without them (left dead <n>, right
      dead <n>). ...` — e.g. port 13 dead on the 2026-09-25 record: `MEASURED record | DEGRADED: dead
      13 -- driving` / `without them (left dead 0, right dead 1). ...` (the ` ...`: the NEW-motors
      list follows on serial, in the `port check:` line) — MAIN says `DEGRADED dead 13`, and MORE's
      side row shows e.g. `3/4 present  dead 13` in amber. That is the program driving without that
      motor, on purpose, under the boot check's rule (a side below three answering, or more than two
      dead in total, refuses).
- [ ] **G.6.3** Now run **Station F's F.2 and F.4 checks** against this boot — F.2.2 (wheels up,
      DETAILS' `est` leaves `UNINIT`, MORE's L and R rows read `4/4 present  0 disagree` on the
      2026-09-25 record — `4+4 PRESENT` in the serial boot lines), F.2.3 (forward is toward the end you
      pushed first in G.4 — the front you chose in G.0.4; `hdg` goes DOWN on a right-stick RIGHT push),
      F.2.4 (full-stick volts wheels up), F.4.1 (the ground run, `est` on DETAILS), F.5 (the feel).
      Fill in their boxes on that page. The serial boot
      lines carry the rest: the record's ports (`[SES] ports MEASURED L -16 +17 -18 -20 R +11 -12
      +13 +15 ...`), the budgets `lin 60 in/s ang 6.00 rad/s wheel 60 in/s` and the gains `ff kS
      1.00 kV 0.1714`.
- [ ] **G.6.4 THE SCALE, CHECKED BY DRIVING.** On the floor, robot at the tape's START, drive
      straight along the tape to the **48 in** mark and stop. `est x` (DETAILS) should read **about 48**. A
      scale that is off by some percent shows as `est x` off by the same percent. (On the
      2026-09-25 record it read 44 — about 8 % short, carried by the team lead's decision.)
      → `est x` at the 48 in mark: **______**
- [ ] **G.6.5** *(Retired at G1 Part 2b. It asked whether Teleop drives better from the record than
      from the typed table; robot two's table cannot describe a drivetrain — no track width, no
      ratio — so it has only ever driven from a record, and that comparison cannot be made. The
      comparison that exists now is Station F's F.5: the feel against Bench Tests' 10 DRIVE.)*

**G.7 — reading the user port raw (the full record)**

The brain's serial output carries everything the screen has no room for. On the lab computer:

```sh
pros lsusb                      # the brain shows TWO ports; the USER one is the second
stty -F /dev/ttyACM1 raw -echo  # substitute the port lsusb showed
timeout 300 cat /dev/ttyACM1 > ~/calibrate-$(date +%H%M).log
strings ~/calibrate-*.log | grep R3D     # every line this program printed
```

**The port numbers RENUMBER between plug-ins** (HA-129), so re-run `pros lsusb` every single time.
Start the `cat` BEFORE you start the program, or you will miss the banner. What a run prints, in
order, every line starting `[R3D]`:
- the banner and `BUILD STAMP:`, the census (`STAGE 0`) — and, when a claw pivot is plugged in,
  `port 6: claw pivot (declared) -- not a candidate, not touched` (and 7; Station J.2) —
  `CARTRIDGE BELIEF`, the IMU port, three `config:` lines (every threshold the run used),
  `===== RUN 1 =====`;
- `phase: ...` at each step; `screen: <top bar>: <first lines> | [buttons]` whenever the layout
  changes (what you were looking at); `touch: x=.. y=.. release=.. on <button> -> ...` for EVERY
  touch, including why one did nothing (`the button was still GREYED (arming)`, `no button there`);
  `push live: ...` as the moved count changes;
- at DONE, the **PUSH TABLE**: one `push port N travel +1692.4 deg MOVED sign +1` line per motor,
  then `push: 9 of 9 candidates moved, drift ..., median travel ...`;
- `front: FRONT FIRST -- ...` and `drive ports, signed: -11 +12 ...`;
- **one line per pulse**: `pulse port 12 (2 of 10) 9.0 V yaw -6.47 deg own +49.8 deg others 49.8
  deg: LEFT` (the verdict is `under the floor -- next level`, `LEFT`, `RIGHT`, `LEFT OUT
  (decoupled)`, `ROLLED`, `STILL` or `CONTRADICTION`); `LEFT OUT: port 18 ...` when one is;
- **one line per spin direction**: `spin CCW 3.0 V yaw +720.2 deg L -78.56 in R +78.56 in W 12.500
  in 7.43 s`;
- `===== RUN 1 ENDED: DONE =====`, the **MEASURED** line, the signed ports per side, `LEFT OUT`,
  both scales, the width, the push's spread, both spin widths, then `SAVED -- ...` or
  `NOT SAVED -- <reason>` — or `===== RUN 1 ENDED: STOPPED =====` and
  `STOPPED [PULSE_ROLLED port 11]: <the whole reason>`;
- `loop: N ticks, worst gap between two ticks N ms ...` (a powered step stops itself past 100 ms);
- after RUN AGAIN: `===== RUN 2 -- RUN AGAIN: a NEW calibrator, nothing carried over =====`.
(The numbers in the push, pulse and spin examples are from the host simulation's ten-motor
robot, not from robot two.)

**What to send back**

1. The census from **G.2.1** and the per-port UP/DOWN from **G.2.2**, and which end you called
   the FRONT (G.0.4).
2. **G.3** if you did it.
3. Every **stop**, word for word: the red headline, the `PORT` line — and the log.
4. Both `MEASURED` lines (**G.5.2**, **G.5.3**), whether each said SAVED, and the tape width.
5. `est x` at the 48 in mark (**G.6.4**), the `boot source:` line (**G.6.1**) and Station F's F.2,
   F.4 and F.5 answers.
6. *(G.6.5 is retired — nothing to send.)*
7. The log files from **G.7** (the calibrate runs and the "shulib Drive" run), and any line on the
   calibrate screen that ran off its edge.

> **THIS VERSION OF "shulib Calibrate" (R3d Part 2F) HAS RUN — AND ITS RECORDED REPRODUCTION RUN
> IS STILL OWED.** It first ran on robot two on 2026-09-24 (run 1 STOPPED on `SPIN_DISAGREE`, run 2
> finished, unrecorded — [R3d-PROGRESS.md](R3d-PROGRESS.md)). Every threshold is still
> invented for a first run (HA-136); the screen's text budgets and positions are derived from earlier screens,
> not measured. **Expect it to stop more than it succeeds the first time, and treat every stop as
> information** — each one names its cause, and the serial record now carries the numbers behind
> it.

---

## Station H — THE DISTANCE SENSORS  *(Bench Tests button 11, robot two — added R2a, 2026-09-24)*

> **Read-only.** Nothing is powered and nothing is written to a sensor. **This station has NEVER
> RUN ON A ROBOT**; its text sizes are the same derived estimates as Station G's (if a line runs
> off the right edge, write down which one, word for word).
>
> **What it answers:** the team lead's *"how do we know the distance sensors ... are working?"* — and
> three beliefs the library's distance code is built on that no robot has checked yet: the reading
> is in **millimetres** (HA-113), **9999 means "nothing there"** (HA-114), and the confidence is
> **0–63 and only meaningful above 200 mm** (HA-115; PROS's C header says it returns 10 below
> that, its C++ header does not say). Your RECORD lines settle them. The station **interprets
> nothing**: it shows what PROS returns and what the library's adapter makes of it, side by side.
>
> **Robot two, 2026-09-24:** distance sensors on **port 4 (facing LEFT)** and **port 5 (facing
> RIGHT)**. The station finds them itself, in the brain's own device list — nothing is typed.

**H.0 — before**

- [ ] **H.0.1** "Bench Tests" uploaded from the committed tree (G.1's first line, `make ROBOT=tank`,
      slot 3); its BUILD stamp is the build you just made.
- [ ] **H.0.2** A **tape measure**, a **flat wall** (or a board held still and square), a pen. A card
      in the SD slot (every RECORD also goes to `/usd/r3a_log.txt`), and the laptop on the USER port
      if you can (G.7 — every line of this station starts `[R3A]`).
- [ ] **H.0.3** Find each sensor's **front face**. Every distance below is measured **from that face**,
      not from the robot's edge.

**H.1 — reading the screen** (every string copied from the code; the numbers are made up)

| Where | It says | What it is |
|---|---|---|
| top bar | `11 DISTANCE -- read-only` | |
| per sensor, line 1 (white; **red** on an error) | `PORT 4 RAW   305 mm conf 61` | exactly what PROS returned: `distance_get` and `distance_get_confidence`. `ERR` = PROS's error value |
| line 2 (light blue) | `  LIB   12.01 in conf 0.97` | the same sensor through **shulib's adapter**: inches, and a confidence from 0 to 1. A separate read, so it can be one sample newer than line 1. `  LIB -- adapter refused (log)` if the adapter would not start |
| line 3 (grey; **amber** once any error was counted) | `  size 75  vel +0.02m/s  9999 x0  ERR x0/0` | PROS's object size and object velocity, raw; **how many reads since you pressed 11 were exactly 9999**; how many were errors (raw / through the adapter) |
| the line above the buttons (amber) | `hold square to a wall at 6/12/24 in; RECORD each` | what to do. With more than two sensors: `+1 more DISTANCE sensor(s): RECORD logs every one` |
| after a RECORD (green) | `RECORD 3: 2 line(s) -> serial + SD card` | written — or amber `RECORD 3: 2 line(s) -> serial only (SD OFF)` |
| buttons | **RECORD** (bottom middle), **DONE** (bottom right) | **grey for half a second** when the screen appears — a tap then is ignored (and logged as ignored) |

The numbers change up to five times a second. Two sensors fit on the screen, port 4 above port 5.
If it says `no DISTANCE sensor in the registry -- nothing to show.`, the brain does not see one:
check both ends of the cable and run **1 DEVICE CENSUS**.

**H.2 — the distances.** Hold the robot **square** to the wall — the sensor's face parallel to it —
**still**, then press **RECORD**. Each RECORD writes BOTH sensors; the one facing away is your
open-space reading for free.

| Which side faces the wall | Tape (sensor face to wall) | = mm | RECORD # | port 4 RAW mm / conf | port 4 LIB in / conf | port 5 RAW mm / conf | port 5 LIB in / conf |
|---|---|---|---|---|---|---|---|
| LEFT (port 4) | 6 in | 152 | | | | | |
| LEFT (port 4) | 12 in | 305 | | | | | |
| LEFT (port 4) | 24 in | 610 | | | | | |
| RIGHT (port 5) | 6 in | 152 | | | | | |
| RIGHT (port 5) | 12 in | 305 | | | | | |
| RIGHT (port 5) | 24 in | 610 | | | | | |
| neither — open space, nothing within 3 m either side | — | — | | | | | |

- [ ] **H.2.1** 6 in is **152 mm — below the 200 mm** where PROS says confidence is not available.
      Write down what `conf` shows there, whatever it is.
- [ ] **H.2.2** **Open space:** leave it pointing at nothing for **30 seconds**, then RECORD. Does
      RAW sit at **exactly 9999** the whole time (the `9999 x` count climbs by about five a second
      if it does)? What does LIB say — `conf 0.00`?
- [ ] **H.2.3** Any `ERR` count above 0, or a red line 1 → write down when it happened.

**What each answer means — for whoever reads your numbers, NOT for you to act on.** RAW should be
about the "= mm" column (HA-113: millimetres); if it reads about the tape in inches or centimetres
instead, that is the finding. LIB should be about the tape in inches. Open space should read 9999
(HA-114). **Do not "correct" anything**: write what the screen showed.

**H.3 — what to send back**

1. The H.2 table, with the RECORD numbers.
2. The `[R3A] dist` lines — from `/usd/r3a_log.txt` on the card, or the serial capture (G.7;
   `strings <log> | grep '\[R3A\] dist'`). One RECORD line looks like this (numbers made up):
   `[R3A] dist REC 3 port 4 raw 305 mm conf 61 size 75 vel +0.021 m/s -> lib 12.008 in conf 0.968 | reads 57: 9999 x0, PROS_ERR x0, lib faulted 0 | t 123456 ms`
   — the station's `REC` count, the port, the raw PROS values, the adapter's inches and confidence,
   how many reads since you entered the station and how many of them were 9999 / an error, and the
   brain's clock in ms. Every touch is logged too (`[R3A] dist touch x=.. y=.. -> RECORD`).
3. H.2.1–H.2.3's answers, and anything surprising, as observed.

---

## Station I — THE AI VISION CAMERA  *(Bench Tests button 12, robot two — added R2a, 2026-09-24)*

> **It changes the CAMERA's settings and powers nothing.** When you press 12 it turns ON the
> camera's **tag detection** and **object detection** (the camera's built-in game-element model),
> and its **USB bounding-box overlay**; **FAMILY** sets which AprilTag family it looks for. The
> screen says so in amber. **Whether the camera keeps these settings after Bench Tests ends is
> UNKNOWN** — the serial log records every setting it wrote and what it read back (I.4 finds out).
> **This station has NEVER RUN ON A ROBOT.** It **interprets nothing**: no bearing, no distance —
> the numbers are the camera's own; the AI Vision code in the library (R2b) is written afterwards
> from what you record here.
>
> **"Can I see what the camera sees?"** Yes, but not through the brain: the camera's **own USB-C
> port** streams its video to **VEX's AI Vision Utility** on a computer. That is VEX's program, not
> ours — follow VEX's instructions to connect it. With the overlay on, the video draws a box around
> each thing the camera detected, so the video and the station's list should show the SAME
> detections.
>
> **COLORS** (colour signatures) are left alone: the camera only detects a colour after it has
> been taught one (a colour descriptor), and this station teaches none. The screen says
> `COLORS need color descriptors - none set here`.
>
> **Robot two, 2026-09-24:** AI Vision on **port 3**, at the front, level, roughly centred. The
> station finds it itself (the brain's device list must say **AI-VISION** for it).

**I.0 — before**

- [ ] **I.0.1** "Bench Tests" uploaded (as H.0.1); SD card in; the serial capture running if you can
      (G.7).
- [ ] **I.0.2** **A game piece** (this season's), a **tape measure**, a clear stretch of floor in front
      of the robot. **An AprilTag** if the team has one — best of all one of the FIELD's: the
      Override field carries a tag on each of its nine goals (game manual, read 2026-09-24), but
      the manual's text does not name their family, so the family that sees a field tag in I.4 IS
      the field's family. Any other printed tag still exercises the station.
- [ ] **I.0.3** Optional: a computer with **VEX's AI Vision Utility** and a USB-C cable for the
      camera's own port (I.3).

**I.1 — reading the screen** (every string copied from the code; the numbers are made up)

| Where | It says | What it is |
|---|---|---|
| top bar | `12 AI VISION -- port 3` | the camera the screen shows |
| amber | `THIS STATION TURNED ON, on port 3:` / `tag + object detection and the USB box overlay` | what it changed on the camera. **Red** `A SETTING FAILED on port 3 -- see serial / SD` / `tag + object detection or the overlay may be OFF` if the camera refused a setting |
| grey | `COLORS need color descriptors - none set here` | see above |
| big line | `OBJECTS 2  TEMP 41.3 C` | how many things the camera reports right now, and its temperature. `ERR` = PROS's error value |
| grey | `data changes every ~33 ms (min 20, n 45)` | how often the camera's reported data CHANGES, timed by the brain checking every 10 ms — a first hint at how fresh its answers are. Before two changes: `data changed 1 time(s); timing ...`; when nothing is moving: `no change in the data for 5 s` (a still scene may not change at all — move the piece) |
| one row per detection | `0 OBJECT 1 <class> x120 y80 w40 h38 s92` | row number; **OBJECT** = the game-element model; its id; the class name the camera reports for that id; then the box as PROS reports it — `x` (PROS: "left edge, from camera's view"), `y` ("top edge"), `w`, `h` — and `s`, the score. **PROS does not state the units**; do not assume pixels |
| | `1 TAG id 5  centre 160,120` then `  corners 150,110 170,110 170,130 150,130` (light blue) | an AprilTag: its id, the average of its four corners, and the four corners |
| | `2 COLOR 1 x120 y80 w40 h38 a125` | a colour or code detection (only if colours were set up elsewhere); `a` is the angle exactly as PROS gives it (its header says tenths of a degree) |
| | `3 TYPE 0x00 id 0 (raw on RECORD)` (amber) | something that is none of the four kinds — a finding |
| | `+3 more -- RECORD logs all 7` | more than four rows' worth; RECORD writes every one |
| above the buttons | `move a piece left, then right; RECORD each spot` | after RECORD: `RECORD 2 -> serial + SD card` (green) or `RECORD 2 -> serial only (SD OFF)`; after FAMILY: `tag family now 16H5 -- hold the tag in view` |
| buttons | **FAMILY** (bottom left) / **RECORD** (middle) / **DONE** (right) | FAMILY's second line is the tag family: `as found` (never set — the camera's own setting) until you press it, then `CIRCLE21H7` → `16H5` → `25H9` → `61H11` → round again (PROS's own names for the four families it knows). All three are **grey for half a second** first |

If it says `no AI-VISION sensor in the registry: nothing to show.`, the brain does not see the
camera as an AI Vision sensor: check the cable and run **1 DEVICE CENSUS** — it must say
`AI-VISION` for port 3. (If the census says `VISION` instead, that is the older Vision sensor, and
this station will not drive it.)

**I.2 — a game piece, WITHOUT the video.** Piece on the floor straight in front of the camera; a
tape from the camera lens forward. Hold each spot still for two seconds, then **RECORD**.

| Spot | RECORD # | OBJECTS | the piece's row, as shown (type, id, class, x, y, w, h, s) |
|---|---|---|---|
| ~1 ft, centred | | | |
| ~1 ft, moved LEFT (the robot's left) | | | |
| ~1 ft, moved RIGHT | | | |
| ~2 ft, centred | | | |
| ~2 ft, LEFT | | | |
| ~2 ft, RIGHT | | | |
| nothing in view | | | |

- [ ] **I.2.1** Moving the piece to the robot's **RIGHT** — did `x` go **UP** or **DOWN**? → ______
- [ ] **I.2.2** From 1 ft to 2 ft — did `w` and `h` get **smaller**? → ______
- [ ] **I.2.3** Did the class name match what the piece is? → the name shown: ______
- [ ] **I.2.4** Move the piece steadily across the view for ten seconds and read the grey line:
      `data changes every ~___ ms (min ___)`. RECORD once while it moves.

**I.3 — the same, WITH the video.** Connect the camera's USB-C to the computer and open VEX's AI
Vision Utility (VEX's instructions). If the camera cannot stream while it is plugged into the brain,
skip I.3 and say so. Repeat I.2's six spots and RECORD each.

- [ ] **I.3.1** Does the video draw a box around the piece at each spot? → yes / no
- [ ] **I.3.2** Does the number of boxes in the video match `OBJECTS` on the brain? → ______
- [ ] **I.3.3** Screenshot the video at the 1 ft RIGHT spot, with the brain's screen in the photo if
      you can.

**I.4 — an AprilTag (only if the team has one; a field tag if possible — I.0.2).** Hold the tag flat, facing the camera,
about 1 ft away, still.

- [ ] **I.4.1** With FAMILY showing `as found`: does a `TAG` row appear? → yes / no. RECORD.
- [ ] **I.4.2** If not: press **FAMILY** once, wait **three seconds**, look for a `TAG` row; again
      until one appears or all four have been tried. RECORD when it appears.
      → the family that saw it: **______** (`CIRCLE21H7` / `16H5` / `25H9` / `61H11` / none)
- [ ] **I.4.3** Rotate the tag a quarter turn — do the corners move round? RECORD.

**I.5 — does the camera keep the settings?** *(two minutes; the answer tells R2b whether a program
must set them every time.)* Press DONE, **power the brain off and on**, run Bench Tests, press 12
again, and before touching anything read the serial or SD log's line
`[R3A] aiv port 3 detection modes AS FOUND: ...` — what does it say? → ______ Copy it next to the
AS FOUND line from your FIRST press of 12 today, before this station had written anything. If the
first one lacked `TAGS` or `OBJECTS` and this one has both, the camera kept the setting across a
power cycle; if this one reads the same as the first, it did not. If the first already had both,
this test cannot tell — write that. (Nothing in the library assumes either answer yet.)

**What to send back**

1. The I.2 table (and I.3's if you did it), with the RECORD numbers; I.2.1–I.2.4; I.3's answers and
   screenshot; the I.4 family; the I.5 AS FOUND line.
2. The `[R3A] aiv` lines from `/usd/r3a_log.txt` or the serial capture
   (`strings <log> | grep '\[R3A\] aiv'`). At entry the station logs every setting it wrote and
   what came back: `[R3A] aiv port 3 detection modes AS FOUND: ...`, `SET tag + object detection
   ON (colours left as found) -> PROS_SUCCESS`, `SET USB bounding-box overlay ON -> ...`,
   `tag family NOT set: the camera's own (as found) until FAMILY is pressed`, `detection modes
   NOW: ...`, and the camera's list of class names (`class names: get_class_name(-1) -> ...`, then
   `class 0 '...'` per name). Each RECORD writes one summary line and one line per detection
   (numbers made up):
   `[R3A] aiv REC 2 port 3 count 1 read 1 temp 41.25 C family as-found modes 0x05 TAGS+OBJECTS | data changed every ~33 ms (min 20, n 45) | t 123456 ms`
   `[R3A] aiv REC 2 port 3 obj 0 OBJECT id 1 class '...' x 120 y 80 w 40 h 38 score 92 | raw 0104...`
   — a TAG line carries `centre` and `corners`, a COLOR/CODE line `angle`; every detection line ends
   with the object's 18 raw bytes in hex, so nothing is lost to how the station decoded it.
3. Anything surprising, as observed — a row that flickers, a count that never settles, a class
   name that is wrong.

---

## Station J — THE CLAWS: the two pivots and the pneumatic claw  *("shulib Drive", slot 1, robot two — added T3, 2026-09-26, for "shulib Teleop" in slot 2; the pneumatic claw, J.9, by T3's addendum the same day; moved to "shulib Drive" at T4 Part 3, 2026-10-04)*

> ⚠ **The claws' first run. It POWERS TWO MORE MOTORS, at full power, from the controller's
> shoulder buttons.** Robot two's two claw PIVOTS, as the team lead stated them on 2026-09-26:
> **port 6** moves one pivot **forward while R1 is held, back while R2 is held**; **port 7** moves
> the other **forward on L1, back on L2**. RED cartridge each, **12 V while a button is held**, and
> on release each pivot is meant to **hold its angle**.
>
> **And the PNEUMATIC claw (J.9)**, as the team lead stated it the same day: its solenoid is on the
> brain's **3-wire port A**; **press A ⇒ the claw opens, press A again ⇒ it closes**; it is
> **CLOSED when the program starts**; and when the robot is disabled it **keeps its state**. Which
> line level opens it, and whether its valve is single or double, are **not known** (A4 register
> HA-159). **Because A is the claw's, the field-centric re-zero button is now Y** (the team lead's
> ruling, 2026-09-26).
>
> **Which programs have the claws.** The two that run robot two's DRIVER FILE (`src/team/robot_two_driver.hpp`
> — which button does what, stated there and nowhere else since T4 Part 3): **slot 1, "shulib Drive"**
> (this station), and **slot 6, "shulib Competition"**'s driver control (Station K). The Auton Tester
> (slot 2) builds the claws so an auton can drive them, and binds no button. **Slot 4, "shulib
> Calibrate", never touches them** (J.2 checks it). Slot 3, Bench Tests, lists them in its census;
> its DRIVE station commands only the chassis table's drive ports. The pneumatic claw is the same: in
> slots 3 and 4 nothing in shulib drives 3-wire port A, so the claw sits wherever its valve leaves it
> — nobody has looked at what that is. (No census lists it: Bench Tests' DEVICE CENSUS reads the 21
> smart ports; a 3-wire port is not one of them.)
>
> **What has run: nothing.** No census or capture has ever shown a motor on port 6 or 7 (the
> 2026-09-24/25 captures list motors on 11–18 and 20 only — A4 register HA-153), and no program has
> commanded one; nothing has ever driven 3-wire port A. **"Holds" is claimed nowhere until J.5 is
> logged**, and it depends on the separate DEFECTS2 fix (a program-commanded Hold actually reaching
> the motor) being in the build you uploaded — ask the coordinator which commit carries both.
>
> **The claw lines are quoted below from the code** (T3 Part 2, 2026-09-26, for "shulib Teleop"; since
> T4 Part 3 "shulib Drive" prints them — its boot lines and DETAILS' claw row; there is NO banner claw
> sentence any more — and `src/common/driver_panel.hpp` words them for both driver programs, from the
> mechanism table AND the driver file's controls; `test/driver_panel_claws_test.cpp` pins every one
> word for word).
> Serial lines are quoted without their `[t=   0.00] ` time prefix; a log line longer than 196
> characters continues on the next line after `... `. **A line on the robot that differs from the
> quote is a finding — write down what it said.** The re-zero button is **Y** in that build (the
> boot's `driver control:` line ends `re-zero button Y; field-centric: OFF (selectable)`).

**J.0 — before**

- [ ] **J.0.1 Station F passes first** — "shulib Drive" (F.0.1: the NEW one) drives, wheels up and
      then on the ground, from THIS build (F.2 and F.4; or G.6 when it boots from the calibration
      record). The claws are tried on a program whose driving is already known to be right, never the
      other way round.
- [ ] **J.0.2** Slots 1, 3 and 4 uploaded from the commit the coordinator names (G.1's commands,
      `rm -rf bin .d` before each `make`), and every BUILD stamp is the build you just made.
- [ ] **J.0.3** The serial capture running before you start anything (G.7) — J.2's lines and
      "shulib Drive"'s boot lines are on it.
- [ ] **J.0.4 Label both ends of both claw cables** (the brain end and the motor end: "6 CLAW",
      "7 CLAW"). The brain cannot tell a claw cable from a drive cable (A4 register HA-155); only a
      label can.
- [ ] **J.0.5** Both claws clear of everything, **no hand in a claw while slot 1 runs**, a second
      person at the battery. 12 V into a claw that is already at its stop is the motor's full stall
      current (HA-154).

**J.1 — the census sees them** (Bench Tests, slot 3, button **1 DEVICE CENSUS**)

- [ ] **J.1.1** MOTOR on **6** and **7**, besides the drive motors (11–18 and 20; 19 if it is
      plugged in again) and the IMU on 2. → MOTOR on 6 **YES / NO** · on 7 **YES / NO** · the full
      list as printed: ______________________________________________
      *(Station G's G.2.1 says "nine MOTORs"; with the claws plugged in the census shows ELEVEN.
      That is expected — the two extra are 6 and 7.)*
- [ ] **J.1.2** Read the **cartridge insert colour** off each claw motor (A4 register HA-153 says
      RED). → port 6 **RED / GREEN / BLUE** · port 7 **RED / GREEN / BLUE**

**J.2 — Calibrate leaves the claws alone** (slot 4 — part of Station G's run, or a start you
abort at the push screen)

Right after the census (`STAGE 0 ...`, one line per device, then `11 motor(s), 1 IMU(s)` with port
19 unplugged — the census still COUNTS the claws), the serial log must show exactly these two
lines — copied from the code, final:

```
[R3D] port 6: claw pivot (declared) -- not a candidate, not touched
[R3D] port 7: claw pivot (declared) -- not a candidate, not touched
```

- [ ] **J.2.1** Both lines present. → **YES / NO**. If the census printed `MOTOR` on 6 or 7 and its
      line is MISSING, **stop and send the log** — Calibrate would then have written BLUE gearing to
      that claw and set it to coast.
- [ ] **J.2.2** The push screen's big number counts the **drive motors only**: `0 of 9 turned` with
      port 19 unplugged, `0 of 10` with it — never "of 11" or "of 12". → it said: ______
- [ ] **J.2.3** Through the whole Calibrate run neither claw moves under power. *(Calibrate does not
      HOLD them either — "not touched" means no command at all — so a loaded claw may sag under its
      own weight while Calibrate runs. That is the motor's unpowered state, not a command; write it
      down if it happens.)* → a claw moved under power? **NO / YES** (which, when): ______

**J.3 — "shulib Drive"'s boot** (slot 1, started from the brain; the robot on blocks)

- [ ] **J.3.1** The mechanism boot lines (`[MECH]`), four per pivot — copy all eight word for word.
      The first says what the robot file DECLARES the pivot to be; the second, where its motor is
      plugged in and whether the brain found it there; the third, what robot two's driver file does
      with it (the buttons and volts from the driver file's controls, found by the declaration); the
      fourth, its only cut-off:
      `[MECH] claw pivot 6: POWER -- RED, motor 1 forward; let go: HOLD`
      `[MECH] claw pivot 6: motor 1 -> port 6 ok`
      `[MECH] claw pivot 6: hold-to-run R1 / R2, 12 V`
      `[MECH] 'claw pivot 6': no stall signature -- held against its stop it is cut only by the driver letting go (HA-154)`
      and the same four for `claw pivot 7` (`port 7`, `L1 / L2`). The claw's three lines are J.9.1's.
      *(Until MECH-A5, 2026-10-29, ONE line per pivot: `[DRV] claw pivot 6: BUILT -- port 6 (typed
      +6), RED, R1 forward / R2 reverse at 12 V, HOLD on release -- buttons: src/team/robot_two_driver.hpp`;
      before that its end read `-- in shulib Teleop only` (T3), then `-- in shulib Teleop and shulib
      Competition` (COMP-b). Station K reads the same lines, the same `[MECH]` tag; the Auton Tester
      prints them without the driver-control lines.)*
      ABSENT (a warning; the drive still runs): the role line carries the device provider's own
      sentence for the port, and the control line says the buttons do nothing — e.g. for an empty
      port 7:
      `[WARN][MECH] claw pivot 7: motor 1 -> port 7 ABSENT: port 7: nothing plugged in (expected a motor)`
      `[WARN][MECH] claw pivot 7: hold-to-run L1 / L2, 12 V -- ABSENT: does nothing; the drive still runs`
      And the port check (the `calibration record: DECODED, port check OK -- ...` line and the
      later `port check:` line) names them as claw pivots, never as NEW motors:
      `... NEW motors not in the calibration: 14; claw pivots (declared, not drive motors): 6 7. NOT CHECKED: ...`
      (with 19 unplugged, 14 is still the one NEW motor). If the record lists 6 or 7 as a DRIVE
      port, "shulib Drive" REFUSES: `a CLAW port is a DRIVE port in the record` / `port 6 is both a claw
      (claw pivot 6) and a drive motor in the record -- recalibrate: shulib Calibrate (slot 4)
      never takes a declared claw port` — run Station G again.
      → ______________________________________________
- [ ] **J.3.2** *(RETIRED at T4 Part 3. It checked "shulib Teleop"'s banner sentence `THE CLAWS are in
      THIS program (slot 2), NOT shulib Drive (slot 1): ...`; "shulib Drive" prints no banner claw
      sentence — the claws are in both driver programs now, and the boot lines (J.3.1) and the
      driver line's `re-zero button Y` say what each button does. Its run-status line still ends `Its
      claws (the pivots on 6 and 7, the pneumatic claw on 3-wire A) have never been driven.`)*
- [ ] **J.3.3** DETAILS' claw row — row 4, above the ONE fault row (touch **DETAILS** on MAIN): `CLAW 6
      HOLD 34C | 7 HOLD 33C | CLAW CLOSED`. Each pivot by its port: `FWD` / `REV` (the volts its motor was last
      commanded), `HOLD` when stopped, `ABSENT` when it was not built — then its temperature in
      whole °C; then the pneumatic claw, `CLAW OPEN` / `CLAW CLOSED` / `CLAW ABSENT`. The
      temperature is the motor's LAST GOOD reading: if a pivot stops answering after boot (its
      cable worked loose), its number FREEZES and nothing on the row says so, and a motor that has
      never answered a read shows `20C` — the program cannot tell (the library's motor seam carries
      no "stale" mark); `--` never appears on the brain. The row is
      AMBER when a claw is ABSENT, or when it ends `FREED` (driver control released a claim an
      operation had left on a claw; serial names which, `[WARN][MECH] '...': a claim left by another
      owner RELEASED as driver control began (...)` since MECH-A5 — not expected on this station,
      which runs no autonomous). The controller's third line says `CLAW ABSENT` when a claw is absent and nothing
      more urgent is showing. → as shown: ______________
- [ ] **J.3.4** If a claw says ABSENT, **"shulib Drive" still drives** — by design, a missing claw motor never
      stops the drive (run F.2.3 for a few seconds to see it). → drove with a claw ABSENT? **YES / NO
      / not tried**

**J.4 — which way each button moves each claw** (robot on blocks; short taps first, half a second)

- [ ] **J.4.1** Tap **R1**: the **port-6** claw moves; the port-7 claw and every wheel do NOT.
      → it moved **OPEN / CLOSED / UP / DOWN** (or: ______) · anything else moved? ______
- [ ] **J.4.2** Tap **R2**: the port-6 claw moves the OTHER way. → **YES / NO**
- [ ] **J.4.3** Tap **L1**: the **port-7** claw moves; the port-6 claw and every wheel do NOT.
      → it moved ______ · anything else moved? ______
- [ ] **J.4.4** Tap **L2**: the port-7 claw moves the other way. → **YES / NO**
- [ ] **J.4.5** Hold **R1 and L2 together**: port 6 goes forward and port 7 goes back at the same
      time — the two pivots are independent. → **YES / NO**
- [ ] **J.4.6** While each button is held, DETAILS' claw row reads FWD / REV for that pivot, and HOLD after
      release — R1 held: `CLAW 6 FWD nnC | 7 HOLD nnC | CLAW ...`; R2 held: `CLAW 6 REV nnC | ...`;
      let go: `CLAW 6 HOLD nnC | ...` (the row repaints at most five times a second, so a tap
      shorter than 0.2 s may never show FWD). → **YES / NO**

**If R1 moves a claw the way the team calls "back",** that is a finding, not a fault: the fix is ONE
word in `src/robots/mechanism_table.hpp` (the coordinator changes that pivot's declaration from
`.motors = {Forward}` to `.motors = {Reversed}` — since MECH-A5; until then its `+6` to `-6`), **never
swapping R1 and R2** — the buttons are the driver's, the direction is the motor's. (The
buttons live in the driver file, `src/team/robot_two_driver.hpp`, and nowhere else since T4 Part 3:
changing which button moves a pivot is a change THERE, by the team — never a table edit.) **If R1
moves the WRONG claw, or a wheel, stop:** a swapped cable (HA-155) or a wrong port (HA-153) — send
J.1's census and what moved.

**J.5 — the load test: does it HOLD?** *(settles A4 register HA-156; needs DEFECTS2 in this build)*

For each pivot in turn: hold its forward button until the claw is at a **mid angle — not against
its stop** — with a game piece in it if the claw carries one; **release**; put a mark (tape, a
pencil line on a card behind it) where the claw's tip is, or read its angle with a phone level.

| Pivot | Loaded with | sag at 0 s | at 10 s | at 30 s | a gentle push by hand while released: springs BACK / stays PUSHED / flops FREE | temperature after (claw row) |
|---|---|---|---|---|---|---|
| port 6 (R1/R2) | | | | | | |
| port 7 (L1/L2) | | | | | | |

**"Holds" is written nowhere — not in the code, not in a commit — until this table is filled in
and sent.** Springs BACK is what Hold should do; stays PUSHED is Brake; flops FREE is Coast (the
release did not reach the motor as a hold — send it at once).

**J.6 — both buttons of one pivot** (ruling R2)

- [ ] **J.6.1** Hold **R1 and R2 together**: the port-6 claw does NOT move (it holds; neither button
      wins). Then let go of R2 while still holding R1: it moves forward. Let go of R1 while holding
      R2: it moves back. → **YES / NO**
- [ ] **J.6.2** The same with **L1 and L2** for port 7. → **YES / NO**

**J.7 — the controller goes away**

A claw's travel is short: at 12 V it may reach its stop before anyone can cut the link, and a
claw that has "stopped" at its stop shows nothing about the disconnect. So this test is passed on
**what "shulib Drive" commands**, read off the claw row and the serial log — not on whether the claw
moved. And the link is cut by **pulling a cable**, not with the controller's power button, so the
moment of the disconnect is the moment you choose: with the controller **tethered** (a smart cable
from the controller to the brain, as for pairing in Station 5.5), pull the tether; on the radio,
pull the **radio's cable from brain port 1**. The USB serial capture (G.7) is on its own cable and
keeps running either way.

- [ ] **J.7.1** Put both claws at **mid travel**. Count out loud "three, two, one, GO": on GO the
      driver presses **R1 and L1 together and keeps holding them**; about one second after GO the
      second person pulls the cable. **The pass:** the claw row shows each pivot go from FWD to
      HOLD when the link goes — `CLAW 6 FWD nnC | 7 FWD nnC | CLAW ...` becomes `CLAW 6 HOLD nnC |
      7 HOLD nnC | CLAW ...`, and row 2 (`cmd`) ends `NO CONTROLLER` — while the driver is STILL
      holding both buttons. ("shulib Drive" prints NO serial line when the controller drops: the panel is
      the evidence, so photograph it.) → FWD then HOLD, both pivots? **YES / NO** · what the row
      showed: ______________________________________________
      *If a claw reaches its stop before the pull,* the test still counts (the row reads FWD until
      the link goes), but a pivot pressed against its stop at 12 V heats (A4 register HA-154):
      **if the pull has not happened within about three seconds of a claw reaching its stop, the
      driver lets go and you start again from mid travel.** **If the row still reads FWD after the
      pull, or a claw keeps moving or keeps pushing against its stop: that is a FAIL** — the
      second person pulls the battery at once (the driver can no longer stop it), and you send the
      log.
- [ ] **J.7.2** Plug the cable back in. Nothing moves until a button is pressed (a button STILL held
      when it reconnects drives again — by design: a disconnect counts as a release, not a
      lock-out). → **YES / NO**

**J.8 — heat: holding against the stop** *(A4 register HA-154; every timing is invented until this)*

Read both claws' temperatures on the claw row first. Then close the **port-6** claw against its
stop and **keep holding R1**. Read the temperature every 10 s. **Stop at 50 °C, or the moment the
claw goes weak** (the motor is believed to start derating at 55 °C — A4 register HA-44, whose
onset nobody has measured; 50 °C leaves room for that belief to be wrong). Then the same for port
7 on L1. The row shows the motor's LAST GOOD reading, and it FREEZES without saying so if the
motor stops answering (J.3.3) — so a number that has not moved at all for 30 s of holding is not
proof the motor is cool: let go, check that pivot's cable, and write down what you found.

| Pivot | before | 10 s | 20 s | 30 s | 60 s (only if under 50 °C) | went weak at |
|---|---|---|---|---|---|---|
| port 6 | | | | | | |
| port 7 | | | | | | |

**J.9 — the PNEUMATIC claw on A** *(A4 register HA-159: its polarity, and one wire or two; T3's
addendum, 2026-09-26)*

Everything here is by eye: no census lists a 3-wire device, and the program cannot tell whether
the air moved — it knows only what it COMMANDED. The table's polarity is a guess, written on the
claw's `.positions` line in `src/robots/mechanism_table.hpp` — `{{"closed", {Low}, grabs}, {"open",
{High}}}`, "line ON = open" (until MECH-A5 the constant `kClawOpenCommand = true`). What can show whether
it is right is J.9.2 (where the claw is after the start) TOGETHER WITH J.9.3's first press — and
only when J.9.0 found the valve's one cable in A: a claw that does not move when A is pressed says
nothing about the polarity. The program commands the claw once at the start and then ONLY when A
is pressed: never on a disconnect, never on a disable (J.9.5, J.9.6).

- [ ] **J.9.0 Before.** Air in the system, charged the way the team always charges it (the gauge
      reading, if there is a gauge: ______). The claw empty and **no hand in it** — it can move the
      moment slot 1 starts. **Look at the valve:** how many 3-wire cables run from the claw's
      valve into the brain, and into which lettered ports? → **ONE / TWO** · into: ______
      *If ONE, but NOT in A:* stop — do not run J.9 — and send the letter. The table says A (A4
      register HA-159) and the program drives only A, so the claw would sit wherever its valve
      rests, and J.9.2 would look like a polarity answer when it is not one.
      *If TWO:* write down both letters, run only J.9.1 and J.9.2, and send them — the table
      declares ONE line (A); a second line is a fact nobody has stated, and the program does not
      drive it. (No polarity answer comes from this case: that needs J.9.3 and one cable.)
- [ ] **J.9.1** "shulib Drive"'s boot lines for the claw, as the code words them (three, since
      MECH-A5 — what it is declared, where its line is, what A does):
      `[MECH] claw: PNEUMATIC -- starts 'closed', safe 'closed'; closed = A low, open = A high (levels UNVERIFIED until a bench station sees them); a disable leaves it where it is`
      `[MECH] claw: line 1 -> 3-wire A not checkable (the brain cannot see a 3-wire device)`
      `[MECH] claw: toggle on A ('closed' / 'open')`
      (the ABSENT form — `[WARN][MECH] claw: line 1 -> 3-wire A ABSENT: <reason>` and `[WARN][MECH]
      claw: toggle on A ('closed' / 'open') -- ABSENT: does nothing; the drive still runs` — cannot
      happen on the brain: its provider refuses only a 3-wire port outside A..H, and the table's A
      is checked before the build. Until MECH-A5 one line: `[DRV] claw: BUILT -- 3-wire port A,
      CLOSED at the start (line low; line high opens it -- polarity UNVERIFIED, HA-159); A toggles it;
      a disable leaves it where it is`.) → ______________
- [ ] **J.9.2 At the start the claw should CLOSE.** Note where the claw is before you start slot 1
      (wherever the valve left it). Start slot 1 and watch it. → before the start: **OPEN /
      CLOSED** · after the start: **CLOSED / OPEN** · it moved: **at the start / when driver
      control began / not at all**. **Do not call the polarity yet** — a claw that is OPEN after
      the start may have a backwards polarity, OR a valve that is not being driven at all (no air,
      a loose cable) resting open. J.9.3's first press tells them apart.
- [ ] **J.9.3 Press A ⇒ the claw OPENS; press A again ⇒ it CLOSES.** Short, separate presses: open,
      closed, open, closed. → the FIRST press: the claw **OPENED / CLOSED / did NOTHING** · each
      press flipped it exactly once? **YES / NO** · anything else moved (a pivot, a wheel)? ______
      · the panel's claw word after each press (the claw row's last word: `CLAW OPEN` after an
      opening press, `CLAW CLOSED` after a closing one — what the program COMMANDED, read through
      the polarity guess): ______ · **serial, about half a second after each press** *(since MECH-A5,
      2026-10-29: a press of A is a mechanism STEP, and the service says its verdict when the claw's
      move time has passed — expected, not a fault)*: after an opening press `[MECH] 'claw': SUCCEEDED
      -- time in 0.50s`, after a closing one `[WARN][MECH] 'claw': UNCONFIRMED -- time in 0.50s (no
      piece sensor to confirm it; counted done)` — ONE line per press? **YES / NO** (copy any other
      `MECH` line) · pressing
      **Y** (the re-zero button now) leaves the claw alone? **YES / NO**
      **The polarity, from J.9.2 and this FIRST press** (only with J.9.0's one cable in A):
      - CLOSED after the start, and the first press OPENED it: **the polarity is RIGHT.**
      - OPEN after the start, and the first press CLOSED it: **the claw's levels are BACKWARDS.**
        Write down "OPENED AT THE START". The coordinator swaps `Low` and `High` on the claw's
        `.positions` line in `src/robots/mechanism_table.hpp` (since MECH-A5; until then the one
        constant `kClawOpenCommand`) — never the button, never the start position, which is a NAME
        and follows the levels — and records the flip where the guess is written (the host test's
        pin, HA-159, this sheet). You may go on
        with this build (every press still flips the claw), but read every "opens" below as
        "closes" and the other way round, and say so in what you send back.
      - The first press did NOTHING: **not a polarity answer** — the valve is not being driven
        (the air, the cable, the port, or a second cable J.9.0 missed). Stop J.9, write down what
        you saw, and send it.
      → the polarity: **RIGHT / BACKWARDS ("OPENED AT THE START") / not called**
- [ ] **J.9.4 Holding A does nothing more.** Hold A down for about three seconds, then let go: the
      claw flips ONCE, at the press — nothing while A is held, nothing at the release; serial shows ONE
      verdict line (J.9.3's), half a second after the press. Then two presses in quick succession (well
      under half a second apart): the claw flips twice, and serial shows `[WARN][MECH] 'claw': CANCELLED
      in 0.<n>s (replaced by a newer command)` for the first, then the second's verdict — expected.
      → **YES / NO** *(if it flips again, chatters, or flips on the release: stop and send it)*
- [ ] **J.9.5 The controller goes away while the claw is OPEN ⇒ it STAYS OPEN.** Open it with A.
      Pull the cable as in J.7 (the tether, or the radio's cable from brain port 1): the claw stays
      open — the program commands nothing on a disconnect. Press A a few times on the unplugged
      controller: nothing moves. Plug the cable back in: still open, nothing moves until A is
      pressed, and one press closes it. → stayed OPEN when the link went? **YES / NO** · nothing
      moved at the reconnect? **YES / NO** · one press closed it? **YES / NO**
      **Then, HOLDING A across the reconnect** *(A4 register HA-147 — added at T3's merge,
      2026-09-26)*: open the claw with A. Hold A down. Pull the cable; plug it back in STILL
      holding A; keep holding for two seconds, then let go. Nothing may move until A is released
      and pressed again; then one press closes it. → moved while A was held across the reconnect?
      **YES / NO** *(YES = the controller's link and its buttons do not come back together: stop
      and send it — until it is answered, the driver's rule is to let go of A after a controller
      drop before pressing again)*
- [ ] **J.9.6 The disable ⇒ the claw KEEPS its state** *(only when a disable is available that
      arrives AFTER slot 1 is already in driver control, in the SAME run — the program not
      restarted: a field controller or competition switch that disables the running program, or
      the end of a Timed Run started from the BRAIN's screen if that ends in a disable of the same
      run (nobody here has seen what one does at its end). **A launch from the CONTROLLER's own
      menu does NOT count and is forbidden** (Station A step 1, G.0.5, the DO NOT list; A4
      register HA-128 — the rule's one exception is slot 6 at Station K, never slot 1): it RESTARTS the program and leaves it disabled before driver control ever
      runs, so A can never open the claw first, and the restart commands CLOSED again. **The
      serial log decides:** "shulib Drive"'s boot lines printed a second time mean the program restarted,
      and the check does not count. If no such disable is available, write "not available": this
      check waits for the first session that has one.)* Open the claw with A and let the robot be
      disabled: it stays OPEN through the disable. Enable again: still open, and one press of A
      closes it. Then the same starting CLOSED: it stays closed.
      The program commands nothing on a disable; whether the BRAIN itself keeps a 3-wire output's
      level while disabled is UNMEASURED — this is the check. → OPEN through the disable? **YES /
      NO / not available** · CLOSED through it? **YES / NO / not available**
- [ ] **J.9.7 With the valve's cable PULLED, nothing can tell** *(A4 register HA-158 — added at
      T3's merge, 2026-09-26; no hand in the claw)*. Stop the program. Pull the valve's 3-wire
      cable out of port A. Start slot 1 as in J.9.2: the boot line for the claw must be the SAME
      as J.9.1's (no refusal, no "absent"), and the screen the same. Look at the brain's own
      Devices screen for any entry for the valve, with the cable plugged and unplugged. **Plug the
      cable back into A before anything else.** → any difference anywhere (boot line, screen,
      Devices)? **YES / NO** *(NO is the expected answer: nothing the program can read sees a
      solenoid — an unplugged valve shows only as a claw that does not move)*
- [ ] **J.9.8 How long the claw takes to move** *(A4 register HA-185 — added at MECH-A5, 2026-10-29;
      no hand in the claw; only after J.9.3 called the polarity)*. An auton's claw step counts the
      move done when the claw's declared move time has passed — `.moveTime = 0.5_s`, a GUESS —
      because nothing senses the claw. Film it with a phone's SLOW-MOTION video, the press of A in
      the frame: three presses that open it and three that close it, with the tank charged as in
      J.9.0; then cycle the claw (A, again and again) until the gauge reads the lowest a match
      leaves it (or 20 cycles, with no gauge) and film one open and one close more. From each clip,
      the time from the press to the claw at rest. → the longest OPEN: ______ s · the longest CLOSE:
      ______ s · the low-air pair: ______ s / ______ s · the gauge at the low-air pair: ______
      *(Over 0.5 s, any of them: the guess is too short — an auton would drive away from a claw still
      closing. Send the clips' times; the coordinator changes the number, its pin and HA-185
      together.)*

**What to send back**

1. J.1: the census list and both cartridge colours.
2. J.2: the two Calibrate lines as they appeared (or that one was missing), the push count, and
   whether a claw moved under power.
3. J.3: the eight pivot boot lines word for word, the claw row as shown, and J.3.4.
4. J.4: which way R1, R2, L1 and L2 moved which claw — and anything else that moved.
5. J.5's table and the hand-push answers. J.6's answers. J.7: which cable you pulled, and what the
   claw row and the serial log showed at the disconnect.
6. J.8's temperature table.
7. J.9: the valve's cable count and letters; the claw before and after the start; the first
   press, and the polarity answer (RIGHT / BACKWARDS, "OPENED AT THE START" / not called); the
   boot lines; each press, the hold, the disconnect (and A held across the reconnect) and the
   disable (which disable, or "not available"); J.9.7's pulled-cable answer; J.9.8's times (and the
   clips, if they can be sent) — as observed.
8. The serial logs (G.7) of the Calibrate start and the "shulib Drive" run, and anything surprising, as
   observed.

---

## Station M — THE AUTON TESTER: run the 40 in auton on the bench, then on the tape  *("shulib Auton Tester", slot 2 — slot 7 from COMP-t until T4 Part 3 retired "shulib Teleop" from slot 2, T4 N4; robot two — added COMP-t, 2026-09-28)*

> ⚠ **This program RUNS AN AUTON: the library drives the robot on its own, under its own control, for
> the first time.** You touch **RUN** on the brain's screen, a **3 s countdown** runs (nothing moves),
> and then the chosen auton runs through `competition::runAuton` — the same call the competition
> program's `autonomous()` makes (Station K), on the same entry from the team's file `src/team/autons.cpp`.
> **ANY controller button, or a touch of the screen, aborts** — in the countdown (then nothing ever
> moves) or in the run (the robot stops within one more 10 ms tick and brakes). So does switching the
> controller off, or VEXos disabling the robot. **The sticks do NOT abort.** RUN is refused without a
> connected controller, from a boot that did not use the calibration record, and in any competition
> mode.
>
> **What has run: nothing.** No part of this program has run on a brain. Every expectation below is
> from the host test plant (instant motors, a tank that never skids) and is marked so. K (the
> competition program — written at COMP-b, below, and run after this one) and L (the driver program)
> are other chunks' stations; this one comes after J.
>
> **K still owns** what the tester cannot show: a field disable mid-auton, the auton → driver-control
> hand-over, whether the controller's Timed Run enters `autonomous()` (HA-162; HA-128 measured only how
> it starts) and PROS deleting its competition tasks (HA-161). A route auton is neither station's yet:
> since MECH-A5 (2026-10-29) both programs register robot two's six mechanism route ids at boot (the
> `ROUTES:` line, M.2.2), but the team's file holds no route auton.

**M.0 — before you touch a program**

- [ ] **M.0.1** A charged battery and a **FAT32 card** in the brain (the run log goes there — M.7).
- [ ] **M.0.2** **If you re-run Calibrate first** (for a truer 40 in — M.6), **G.0.1's rule applies:
      copy `shulib_drivetrain.bin` off the card on a laptop, dated, first** — and do it only after
      SAFE-SAVE B1 is in the build (until then a NOT SAVED run can erase the record).
- [ ] **M.0.3** **3 m clear ahead** of the robot, and a second person at the battery.
- [ ] **M.0.4** A **straight tape at least 60 in long** on the floor, with a **START** mark, a **40 in**
      mark and a **43.6 in** mark. *Why 43.6:* on 2026-09-25 the saved record read `est x` 44 at a taped
      48 in — it under-reads distance by about 8 % — so an auton that drives 40 in BY THE ESTIMATE
      should travel about 40 × 48 / 44 ≈ **43.6 in** on the tape, until a careful Calibrate rerun fixes
      the record. **The number in the auton stays 40. Never change it to make the tape agree.**
- [ ] **M.0.5** The controller **paired** (Station 5.5) and **ON, on its home screen** (never started
      from its menu — HA-128).
- [ ] **M.0.6** Do not touch the robot for **3 s after power-on** (the IMU calibrates; READY's estimate
      line says `estimate NOT LIVE yet (the IMU is calibrating)` until it is done).

**M.1 — upload**

```sh
rm -rf bin .d && make ROBOT=tank PROGRAM=autontest && pros upload --slot 2 --name "shulib Auton Tester"
```

Start it from the **BRAIN's** screen (Programs → slot 2 → Run). *(A brain uploaded before T4 Part 3 may
still hold an older tester in slot 7 and "shulib Teleop" in slot 2: this upload overwrites slot 2;
delete slot 7 from the brain's Programs menu, so nobody runs the stale one.)* READY's last line ends with a
`BUILD hh:mm:ss` stamp: it must be the time you just built — if not, the upload did not land.

**M.2 — boot** (serial, `pros terminal`, if a laptop is on the micro-USB)

- [ ] **M.2.1** The banner: `[ATS] booted: shulib AUTON TESTER build [shulib-robot-variant=tank-autontest]
      -- runs ONE of the team's autons` … `NOT RUN ON A ROBOT YET.`, and `[ATS] competition status at
      boot: 0x00 field=no disabled=no auton=no`.
- [ ] **M.2.2** The three boot lines (one line once, split by the chunk's review so a long build hash
      can never cut the SD path off):
      `boot: shulib AUTON TESTER [shulib-robot-variant=tank-autontest] hash …`, then
      `boot source RECORD, 2 autons, 3 of 3 mechanisms present`, then the SD line. **This is M.7's
      CREATED check:** on the first boot on a card without the file it reads
      `SD run log ON, CREATED /usd/shulib_auton_runs.txt`; if the file was already there,
      `SD run log ON, APPENDED to /usd/shulib_auton_runs.txt`; with no card, `SD run log OFF (no card, or
      the log would not open)` — write down which. `3 of 3` is the pneumatic claw and both pivots (the
      ones the run guard safes — M.9): fewer is a mechanism not built, and its `mechanism '…' is
      ABSENT` line says which — copy it. Then *(since MECH-A5, 2026-10-29)* the mechanism lines,
      `[MECH]` — J.3.1's and J.9.1's without the driver-control lines (this program has none): three
      per pivot and two for the claw, eight in all — and one line for the route ids (this program's
      `[ATS]` tag):
      `ROUTES: 6 mechanism route ids published (claw_pivot_6_run .. claw_open); 26 of the registry's 32 left for the team's own`.
- [ ] **M.2.3** The first `competition status 0x00 field=no disabled=no auton=no -- RUN is allowed`
      line (HA-178 (c)). If it reads anything else, RUN will be refused: copy the line.
- [ ] **M.2.4** The boot is **not** refused with `the tester's pacer chain is WRONG` — the boot's chain
      proof passed on the real provider (R13). If it is, stop: that is a build error.
- [ ] **M.2.5** PICK shows `AUTON: NONE` in the bar, the two autons (`40in forward MATCH`, `TEST 24in
      turn back MATCH`) and `NONE` bottom-centre. Write down any line that runs off the screen's edge
      (HA-136).

**M.3 — READY, and THE FORWARD CHECK**

Touch `40in forward`. READY shows eight lines:

```text
MATCH park T+27.0s stop T+29.0s RELATIVE
BOOT RECORD: forward = the end that led Calibrate
EST x 0.0 y 0.0 in, facing 0.0 deg LIVE
READY: touch RUN, then step back
RUN counts 3 s, then it moves. Step back.
ANY BUTTON or a touch of the screen aborts.
SD: runs appended to shulib_auton_runs.txt
faults: none  BUILD hh:mm:ss
```

- [ ] **M.3.1** The EST line reads `LIVE`, and the second line reads **`BOOT RECORD: forward = the end
      that led Calibrate`**. A red `BOOT TYPED TABLE: its front may be the OTHER end` line, or RUN grey
      with `RUN needs the calibration RECORD: run Calibrate`, is a **STOP**: do not go on — fix the
      record first (D9).
- [ ] **M.3.2** **THE FORWARD CHECK — a STOP condition.** Push the robot slowly about 6 in toward the
      end you believe is forward (the **camera end** on today's record). **EST x must RISE.** If it
      FALLS, **stop**: the record's forward is the other end, and M.6 would drive toward you. Write
      down which end made x rise — **that end is "forward" for M.5–M.8.** Put the robot back.
      → the end that made x rise: ______________
- [ ] **M.3.3** Turn the controller **OFF**: the fourth line reads `NO CONTROLLER: RUN needs it (the
      abort)` and RUN goes grey. Turn it back **ON**: RUN green again.

**M.4 — the aborts, WHEELS UP** (the robot on a stand, wheels clear of everything)

**Every touch in M.4 goes on the TOP HALF of the screen** — no button on the countdown, running or
RESULT page is there, so the finger that aborts cannot also press something on RESULT. (PICK does
have one there, its first auton; you are not on PICK when you abort.)

- [ ] **M.4.1** RUN, then press **B** when the countdown shows `2` ⇒ RESULT `ABORTED`, `aborted in the
      countdown by: button B`, `nothing moved: runAuton was never called`. Nothing turned (HA-178 (b)).
- [ ] **M.4.2** RUN, then **touch** the screen in the countdown ⇒ `ABORTED`, `… by: touch`. Then again,
      and this time **press and HOLD**: it must abort at the touch-DOWN (HA-178 (a)). Keep holding until
      RESULT is up, then lift: `< PICK` and `AGAIN` must stay **GREY until half a second after the
      lift** (R7). → did they? **YES / NO**
- [ ] **M.4.3** RUN, then switch the **controller OFF** in the countdown ⇒ `ABORTED`, `… by: controller
      lost`, nothing moved. Switch it back on. Note how long after the switch-off the screen changed
      (HA-178 (d)): ______ s
- [ ] **M.4.4** RUN, let it **start** (the wheels turn), then press any button ⇒ the wheels stop within
      a blink; RESULT reads `ABORTED`, `aborted by: button …`, `end action skipped; …` (a press that
      lands while the brake is already running reads `end action CUT; …` instead — the abort cut the
      brake short; nothing is wrong with it). Copy the `[ATS]` lines from serial. **A red `ABORT
      IGNORED` is a STOP**, whenever it appears: the button was seen but never reached the run — a
      build error. Do not run on the floor; copy the lines and send them.
- [ ] **M.4.5** RUN, let it start, then switch the **controller OFF** ⇒ the wheels stop; RESULT `aborted
      by: controller lost`.

**M.5 — a wheels-up full run** (still on the stand)

- [ ] **M.5.1** RUN and let it finish. **Expected: RESULT `FINISHED`**, `END` x ≈ 40 by the encoders,
      `faults this run: none`. On a stand the robot cannot turn, so a weak side has nothing to pull off
      its line. **If it is NOT `FINISHED` wheels up, stop and copy the lines:** that is not a weak side's
      drift.

**M.6 — the floor run on the tape**

> **M.6 needs an IMU1 Part A build** *(added by IMU1 Part A, 2026-10-29 — the team lead's D4 (a), 2026-10-22;
> the check corrected at Part A's review, the same day)*. Before IMU1 Part A, the tester's motion read the IMU's
> turn rate twice in every tick, and on this robot the second read could spoil the next tick's first — the one
> TANK1's steering correction uses (derived from the code and PROS's documented 10 ms copy; never seen on a
> robot). **A floor run counts as evidence for HA-179 and HA-180 only from a tester built from a tree with IMU1
> Part A's hold in it:** before M.1's build, in the laptop's shulib folder,
> `grep -c kRateHoldUs include/shulib/hal/pros/imu.hpp` must print **1 or more**; `0` (or "No such file") is a
> pre-IMU1 tree. (Not `git log | grep IMU1`: two older plan commits carry "IMU1" in their subjects and hold no
> fix.) Write M.2.2's boot `hash` in M.6.4's table — for a run written down earlier, the hash's `g…` part names
> the commit (`…-g2ef1f3f` → `2ef1f3f`), and
> `git show 2ef1f3f:include/shulib/hal/pros/imu.hpp | grep -c kRateHoldUs`
> answers the same question. A run on an older build (or a hash ending `-dirty` you cannot account for) is
> still recorded, marked **pre-IMU1**, and a TIMEOUT on it — TSTR lit, the correction pinned at its limit —
> is NOT evidence against HA-179. M.0–M.5 (the boot, the forward check, the aborts, the wheels-up run) need
> no IMU1 build.

- [ ] **M.6.1** **Capture serial BEFORE RUN** (it is the only place the tick lines are; the SD file gets
      each run's summary lines, not its ticks). Use ONE of: (a) the brain's micro-USB to a laptop running
      `pros terminal`, on a cable long enough for ~45 in of travel, held **slack and to the SIDE** so it
      neither snags a wheel nor pulls the robot off its line; or (b) the wireless terminal — the laptop's
      USB into the paired CONTROLLER, `pros terminal` (Station 5.5) — whose rate at 100 lines a second is
      unmeasured, so it may drop lines. Save the whole transcript. The `[t=…]` stamps step about 0.01 s
      per tick: a bigger jump between `[MOT]` lines means lines were lost — write the count as "N of M,
      with gaps". **With neither, write the TSTR count as "not captured" — never a guessed number.**
- [ ] **M.6.2** The robot on START, **the end that made EST x RISE in M.3 leading**, facing along the
      tape. RUN, and step back — **to the SIDE, never behind or in front**.
- [ ] **M.6.3** **The expected result (stated BEFORE the run so nobody is surprised; re-predicted after
      TANK1 Part A, 2026-09-28):** `FINISHED`, `faults this run: none`, END x near 40 and END |y| within
      0.5 in, the brake run. On the host test plant a 9 % weak side finishes this way (T+1.89 s, END
      (39.64, 0.02, 0.10°), TSTR on 116 of the run's 203 `[MOT]` tick lines); an even drive shows TSTR on
      few or none (noise lights about 1 % — TANK1's P7). `ROUTINE STOPPED` (`faults this run: 1, last
      MOTION_TIMEOUT`, the brake still run) stays possible — HA-179 (TANK1's constants against this
      robot's unmeasured motor lag) being wrong; that is a result, not a failure of the tester. **A
      `FINISHED` is checked against the tape, not taken as a pass:** a sideways skid (HA-180) is
      invisible to the robot's sensors, so it never turns the word red — **a `FINISHED` whose tape offset
      from the line differs from END y by more than 0.5 in is HA-180 failing, not a pass.** The `MOT`
      Warn `tank moveTo not steered … turn first: face()` must NOT appear (the 40 in starts on its own
      heading): if it does, stop and copy the lines.
- [ ] **M.6.4** Record — **the numbers, never "about right"**:

      | | run 1 | run 2 | run 3 |
      |---|---|---|---|
      | build hash (M.2.2), and the `kRateHoldUs` grep's count (1 or more = IMU1 Part A) | | | |
      | RESULT word | | | |
      | END x, END y (estimate, from RESULT or serial line 4) | | | |
      | the faults line | | | |
      | `[MOT]` tick lines showing TSTR, out of how many (or "not captured") | | | |
      | TAPE along the line, in (expect ~43.6 today, ~40 after a good Calibrate rerun) | | | |
      | TAPE off the line to the SIDE, in, and WHICH side looking the way it drove (END y is + to the LEFT) | | | |
      | the veer you saw; did it stop firmly? | | | |

      **HA-180 settles on three runs whose tape sideways offset agrees with END y within 0.5 in.**
- [ ] **M.6.5** **Between runs, LIFT the robot back onto START** — the end that made EST x rise
      leading, facing along the tape, as in M.6.2. After a run the drive is held under **Brake**, so
      do not shove it back along the floor: that strains the motors against the brake. Lift it, set it
      down, then touch AGAIN (RELATIVE start: where it is set down is the new (0, 0)).

**M.7 — the SD log (HA-177)**

The file M.2 found or created is the one this station checks — M.2.2's first boot was the CREATED
check, and every run of M.4–M.6 has been appended to it since. **Never format the card and never
swap in a blank one to get a "fresh" file:** `shulib_drivetrain.bin` (the calibration record) is on
it, and without the record the tester refuses to boot (D9; robot two cannot boot from its typed
table).

- [ ] **M.7.1** Power-cycle the brain and start the tester again ⇒ the SD boot line reads `SD run log ON,
      APPENDED to /usd/shulib_auton_runs.txt`. Do one more run (wheels up is fine). Then copy
      `shulib_auton_runs.txt` (the card's root) to the laptop. **Every run since M.2 must be there, in
      order, across the reboot** — M.4's aborts, M.5, M.6's floor runs, and this last one.
      → M.2.2's SD word: __________ ; M.7.1's: __________ ; every run present in order? **YES / NO**
- [ ] **M.7.2** *(Only if M.2.2 read APPENDED and a CREATED is still wanted.)* Copy
      `shulib_auton_runs.txt` off first; then, on a laptop, delete **only that one file** from the card —
      never format it, and leave `shulib_drivetrain.bin` where it is. Boot ⇒ `SD run log ON, CREATED
      /usd/shulib_auton_runs.txt`. → CREATED seen? **YES / NO**

**M.8 — (optional) the acceptance run**

- [ ] **M.8.1** From PICK, `TEST 24in turn back`, on the tape, forward as in M.3. **Expected
      (re-predicted after TANK1, 2026-09-28):** back on the START mark facing the other way — on the host
      test plant a 9 % weak side ends at (0.37, 0.06) facing 177.3° after 6.14 s (its last leg faces its
      own bearing back to the mark, so not exactly 180°); or, if M.6 stopped for a weak side, `ROUTINE
      STOPPED` at step 1 (moveTo) with steps 2–4 skipped. Record which, and the TSTR count (serial
      captured as in M.6, or "not captured"). Before it, and between any repeats, LIFT the robot onto
      START as in M.6.5 — never shove a braked drive.

**M.9 — the claws**

- [ ] **M.9.1** Through every run above: the pneumatic claw stays **CLOSED** (its safe state and its
      start level), and both pivots **HOLD** at the cut and at the end (the guard safes every PRESENT
      mechanism; the tester never commands them otherwise). → did anything on a claw move? **YES / NO**
      *(Today's autons never command a claw, so this station shows only that nothing moved one. That
      the guard SAFES every present mechanism, at a run's end and on an abort, is shown by the host
      suite — test (18b) — not here; since MECH-A5 the claws run on the mechanism service under the
      guard, and an auton step on a claw is host-tested (`test/robot_two_claws_example_test.cpp`).)*

**What to send back**

1. M.2: the banner, the three boot lines, the first status line — word for word; and whether the
   boot was refused.
2. M.3: the BOOT line, and **which end made EST x rise**.
3. M.4: each abort's RESULT lines, M.4.2's grey answer, M.4.3's seconds, and the `[ATS]` lines.
4. M.5: the RESULT.
5. M.6: the table, and the whole serial transcript (or "not captured").
6. M.7: the SD words (M.2.2's and M.7.1's, and M.7.2's if run) and the file itself.
7. M.8 (if run) and M.9.
8. Anything surprising, as observed.

---

## Station K — THE PRACTICE MATCH: "shulib Competition" through the controller's Timed Run  *("shulib Competition", slot 6 — slot 5 stays the team's old-code fallback; robot two — added COMP-b, 2026-09-30)*

> ⚠ **This is the program for a MATCH.** Before a match you choose an auton on the brain's menu
> (**NONE**, the default, runs nothing); when autonomous starts, the chosen auton runs on the program's
> OWN task through `competition::runAuton` — Station M's call, on the same entry — and a **disable**, or
> the end of the autonomous period, ends it at once (one more 10 ms tick at most, then 0 V + Brake);
> then driver control runs the team's driver file (Station J's buttons). **No button aborts an auton
> here** — that is the tester's rule, never the match's.
>
> **What has run: nothing.** No part of this program has run on a brain. Every expectation below is
> from the host test plant (instant motors, a tank that never skids) or from Station M's measured runs,
> and is marked so. **The team owns no competition switch**, so the match here is the controller's
> **Timed Run** — and what a Timed Run does after it starts the program disabled is itself UNMEASURED
> (A4 register **HA-162**); K.3 measures it before anything else depends on it. A field or a switch
> stays UNRUN at the bench.
>
> **Run K only after F, J and M have passed on the same build line** (the same commit's builds):
> **M.3's forward check passed** (the only station that proves "forward" before an auton drives) and
> **M.6's floor result recorded**, whatever it was. **Driver control here runs the library's drive
> protections** (since T4 Part 2: a frozen drive motor dropped, more than two dead stop the drive for
> the run, a fight or an over-current cuts both sides for 1 s — Station F's WARNING and CUT paragraphs)
> — none has run on a robot, so drive as gently as Station F, and **THE PUSH CHECK (K.8) is REQUIRED
> before this program drives a match** (the team lead's N1).

**K.0 — before you touch a program**

- [ ] **K.0.1** F, J and M passed on this build line (above) — write the commit: __________
- [ ] **K.0.2** A charged battery; the **FAT32 card** in the brain with `shulib_drivetrain.bin` (the
      calibration record) and Station M's `shulib_auton_runs.txt` on it. Never format it.
- [ ] **K.0.3** **A laptop on serial for the WHOLE station** (`pros terminal`, micro-USB to the brain, or
      the wireless terminal through the controller — Station 5.5): K's measurements ARE the program's
      lines. Save every transcript. With no serial, K.3 and K.6 cannot be settled — write "no serial".
- [ ] **K.0.4** The robot **WHEELS UP** on a stand for K.2–K.4 and K.6; on the tape (M.0.4's) for K.5;
      on the ground, square to a wall, for K.8. A second person at the battery.
- [ ] **K.0.5** **The claw at the OTHER level from its start level** for K.4: the start level is CLOSED,
      so leave it **OPEN** — open it with **A** in the brain start below (K.2.5), then end that run by
      stopping the program from the brain, so it stays open. → claw left OPEN? **YES / NO**

**K.1 — upload**

```sh
rm -rf bin .d && make ROBOT=tank PROGRAM=competition && pros upload --slot 6 --name "shulib Competition"
```

**Slot 6. Never slot 5** (the team's own old code, held as the fallback — photograph the Programs list
first, Station A's slot map).

**K.2 — the brain start: the boot, the menu, driver control** (started from the **BRAIN's** screen, the
controller on its home screen — nothing competition-like is connected, so PROS goes straight to driver
control after the boot)

- [ ] **K.2.1** The banner: `[CMP] booted: shulib COMPETITION build [shulib-robot-variant=tank-competition]
      -- the auton chosen on the brain's` … `NOT RUN ON A ROBOT YET.`, and `[CMP] competition status at
      boot: 0x00 field=no disabled=no auton=no`.
- [ ] **K.2.2** The three boot lines: `boot: shulib COMPETITION [shulib-robot-variant=tank-competition] hash
      …`, then `boot source RECORD, 2 autons, 3 of 3 mechanisms present; autons ALLOWED`, then `SD run log
      ON, APPENDED to /usd/shulib_auton_runs.txt` (Station M created the file — the same one). **A boot line
      reading `autons REFUSED (a typed-table boot: run shulib Calibrate)` is a STOP** (the record did not
      load); so is any refusal screen — copy its words. The boot is **not** refused with `the competition
      program's pacer chain is WRONG` (the chain proof passed on the real provider). After a refused
      boot no auton task exists, so serial carries **no** `competition status … at <ms>` line at all —
      only the four phase functions' `entry` and `normal return` lines (`autonomous()`'s reads `the boot
      was REFUSED: no auton runs`). Then **the log copy**: `log copy: /usd/comp_log.txt (CREATED, kept
      across boots) -- its events and summaries, ...` on the first boot (`APPENDED` after), or `log
      copy: /usd/comp_log.txt did NOT open: NO SD CARD -- serial only; ...` (with a card in: `... did
      NOT open, though an SD card is in (full? write-protected? a folder of that name?) -- ...`) —
      never a refusal. **An auton the registry refused is DROPPED, not the boot** (the team lead's
      ruling, 2026-10-02): `1 auton(s) REFUSED at registration and DROPPED -- the dropped ones are NOT
      on the menu (each named below); every other auton and driver control still run -- fix
      src/team/autons.cpp`, then `DROPPED: auton '<name>': <why>` (past eight drops: `(the first 8
      named below)`, and a last line counting the rest), and the menu names it in red; none is
      expected with the team's file as it is — copy any word for word.
- [ ] **K.2.3** The mechanism lines, word for word: J.3.1's eight for the pivots and J.9.1's three for
      the claw — "shulib Drive" prints the same, the same `[MECH]` tag (since MECH-A5, 2026-10-29; until
      then each pivot's `[CMP]` BUILT line ended `-- buttons: src/team/robot_two_driver.hpp`) — and
      the route ids' line (this program's `[CMP]` tag), `ROUTES: 6 mechanism route ids published
      (claw_pivot_6_run .. claw_open); 26 of the registry's 32 left for the team's own`.
- [ ] **K.2.4** The phase lines: `opcontrol() entry at <ms>, status 0x00 field=no disabled=no auton=no`,
      then `opcontrol(): driver control starts -- the driver file, after 0.000 s waiting for the auton
      task`. The screen: the **driver page** — `shulib COMPETITION` and the BUILD time in the bar (the time
      you just built: else the upload did not land), then the drive protections' row **`drive OK`**
      (green), `auton: none requested on this boot`, `controller: CONNECTED`, the fault row, the claw row.
      (The menu shows only for a moment at boot: with nothing connected, driver control takes the screen
      at once.) **What a CUT, a FROZEN port or a dead port looks like here:** the protections' row turns
      red `CUT 1 s #<n>: <the serial CUT line's words>` for that second (the controller `CUT see brain`),
      amber `drive OK -- frozen <port>` while a reading is frozen, amber `DEGRADED dead <ports>` once one is
      dropped (the controller `DEGRADED see brain`, with `dead <ports>` on its row 1), red `REFUSED: <n>
      drive motors dead -- 0 V for the run` past two dead — the same rows and serial lines as Station
      F's, in this program's driver page.
- [ ] **K.2.5** Wheels up, drive briefly: the sticks drive as at Station F (forward = the end M.3 found);
      R1/R2 and L1/L2 move the pivots as at J; **A opens the claw — leave it OPEN** (K.0.5). The
      controller's lines: `LIB 12V B…V`, `hdg …`, `ok`; the protections' row stays `drive OK`. Serial
      (and `comp_log.txt`, K.7.2) carries one `[MECH] 'claw': …` verdict line per press of A (J.9.3's —
      since MECH-A5, expected), and none for the pivots' buttons. Then stop the program **from the
      brain**.

**K.3 — THE TIMED RUN, MEASURED (HA-162 — the FIRST match step; everything after it depends on it)**

The one exception to "never start from the controller's menu": **this program, slot 6, at this station.**
(K.2's brain start comes before it only to read the boot and to leave the claw OPEN for K.4, which
shares this start; nothing in K.2 is a match.)

- [ ] **K.3.1** The robot wheels up, the claw OPEN (K.0.5), **touch nothing on the brain**: NONE stays
      the choice, so nothing will drive. On the CONTROLLER: Programs → slot 6 "shulib Competition" →
      **Timed Run**. Watch the claw from the moment you start it — that is K.4, the same start.
- [ ] **K.3.2** Copy, from serial, every phase line and every status line, in order, with their
      milliseconds — the boot, then e.g. `disabled() entry at <ms>, status 0x… …`, `competition status 0x…
      field=… disabled=… auton=… at <ms>`, `autonomous() entry at <ms>, …`, the request's lines
      (`request 1: NONE -- no auton was chosen …`), `autonomous() normal return at <ms>, … -- its request
      FINISHED`, and whatever follows (`opcontrol() entry …`, or `disabled() entry …`). From them:
      → the disabled window before autonomous: ______ s (`0` if none) ; autonomous enabled for: ______ s ;
      then: ______________________ (driver control for ____ s? a disable? the end?) ; the status bytes seen:
      ______
- [ ] **K.3.3** **If no `autonomous() entry` line ever appears**, write "Timed Run never entered
      autonomous()" and try **Match** from the same controller menu, the same way; record its lines
      instead. If neither enters autonomous, **stop here**: K.5–K.6 need it.
- [ ] **K.3.4** **If there is NO disabled window before autonomous** (autonomous starts at once), the
      auton cannot be chosen on the bench before it starts — write that down and **stop**: how to choose
      one then is the team lead's call, after this measurement (nothing is built for it).
- [ ] **K.3.5** **Afterwards: turn the controller OFF, wait ten seconds, and see the brain's menu return**
      before starting any other slot. (Every later start in K is the same Timed Run.)

**K.4 — the claw at a disabled start (HA-157's second half — the COMP brief's B28)**

From K.3.1's start (the claw left OPEN, the brain coming up DISABLED, so `initialize()` writes the claw's
start level — CLOSED — while disabled):
- [ ] **K.4.1** Did the claw move **at the start** (the write reached the valve while disabled), **only when
      the robot was enabled**, **only when driver control began** (the driver file's first entry tells it
      CLOSED, whatever autonomous left), or **never**? → ______________ (and at which line's milliseconds)

**K.5 — a practice match on the tape** (only if K.3 found a disabled window before autonomous)

> **K.5 counts with M.6's rule** *(added at IMU1 Part A's review, 2026-10-29 — the IMU1 brief's §6, D4 (a))*:
> K.5 runs the same 40 in through the same call, so a K.5 run is evidence for HA-179 and HA-180 only on an IMU1
> Part A build, checked the same way — the `grep -c kRateHoldUs include/shulib/hal/pros/imu.hpp` before K.1's
> build (1 or more), and K.2.2's boot `hash` in K.5.3's table. A run on an older build is recorded, marked
> **pre-IMU1**; a TIMEOUT on it is not evidence against HA-179. K's other steps need no IMU1 build.

- [ ] **K.5.1** The robot on START, **the end that made EST x rise in M.3 leading**, facing along the tape.
      Start the Timed Run (K.3.1's way). **In the disabled window**, touch `40in forward MATCH` on the
      brain: the bar reads `AUTON: 40in forward` and serial logs `menu: the auton is now '40in forward'`.
      Step back — to the SIDE.
- [ ] **K.5.2** When autonomous starts: the screen shows **AUTON RUNNING** (an amber bar, `AUTON: 40in
      forward`), the robot drives, then the **result page**. **Expected (stated BEFORE the run):** the
      same as **M.6 measured** on this robot — Station M's table is the prediction here, because the run
      is the same call on the same entry. On the host test plant (re-predicted after TANK1): `FINISHED`,
      END x near 40 and END |y| within 0.5 in, `faults this run: none`, the brake run (a 9 % weak side
      finishes at T+1.89 s, END (39.64, 0.02, 0.10°), TSTR on 116 of the run's 203 `[MOT]` tick lines);
      `ROUTINE STOPPED` with `faults this run: 1, last MOTION_TIMEOUT` stays possible (HA-179), and if M.6
      read that, expect it here too. **COMP-t's thresholds, through THIS program's request** (host test
      plant; `test/competition_run_test.cpp` (R24) reproduces them): the 40 in FINISHES down to a weak
      side at ×0.723 of the other and stops (`motion TIMEOUT`) at ×0.722; the acceptance run (K.6's)
      FINISHES down to ×0.724 — at ~9 % weak, `FINISHED` at T+6.14 s, END (0.37, 0.06, 177.28°) — and
      stops at ×0.723. A motion the odometry calls stuck is ended by the library (`ODO_STUCK`, the
      scheduler's abort): for the 40 in, line 3 then reads `routine STOPPED, 0 of 1 steps done, stopped at
      moveTo (motion CANCELLED); faults this run: 1, last ODO_STUCK -- see the [ERROR] line` — record it,
      never re-run blind.
- [ ] **K.5.3** Copy the four `[CMP] request 1: …` lines — `… '40in forward' FINISHED -- ROUTINE_RAN; in
      time, end action succeeded`, `… hand-off 0.0__s after autonomous(), live after …`, `… routine OK, 1 of
      1 steps done; faults this run: none`, `… seated (0.00, 0.00) facing 0.0 deg (the IMU read …); END
      estimate (…) … LIVE` — and the `[COMP]`, `[SEQ]` and `[MOT]` lines. The moveTo's `[MOT]` result line
      ends with the cross-track column, printed as `xt` and the distance in inches with a `"` (e.g.
      ` xt   0.02"`); the tick lines carry ` TSTR` where the library was steering. Record — **the numbers,
      never "about right"**:

      | | run 1 | run 2 |
      |---|---|---|
      | build hash (K.2.2), and the `kRateHoldUs` grep's count (1 or more = IMU1 Part A) | | |
      | RESULT word | | |
      | line 3: `routine OK`, or `stopped at <step> (<cause>)` and the last fault by name (e.g. `ODO_STUCK`) | | |
      | the hand-off (line 2), s | | |
      | END x, END y (line 4) | | |
      | the `xt` value on the `[MOT]` result line | | |
      | `[MOT]` tick lines with TSTR, of how many (or "not captured") | | |
      | TAPE along the line, in (expect M.6's) | | |
      | TAPE off the line to the side, in, and which side | | |

- [ ] **K.5.4** **Driver control after the auton** (if K.3 found one): the driver page's auton row reads
      `auton FINISHED: 40in forward` (or the word the result page showed). **`AUTON ENDING` must NOT
      appear** — the auton ended long before; if it does, copy its `opcontrol(): AUTON ENDING -- …` line and
      stop. Drive gently, wheels down only if the space allows (the protections are host-tested only).
- [ ] **K.5.5** Between runs, LIFT the robot back onto START (M.6.5's rule — never shove a braked drive).
      Turn the controller off, ten seconds, the menu back (K.3.5) before each new Timed Run.

**K.6 — THE MID-AUTON DISABLE (HA-161; written from K.3's measurement)** — wheels up

The auton must still be RUNNING when the mode changes, so this uses the acceptance run, `TEST 24in turn
back` (about 6 s on the host test plant, its legs 24 in, a turn, back).
- [ ] **K.6.1** Choose `TEST 24in turn back MATCH` in the disabled window and let autonomous start it.
      **If K.3 found the autonomous period SHORTER than ~6 s**, the period's own end lands mid-auton: wait
      for it. **Otherwise** switch the **controller OFF** about 2 s into the run (what that does to a Timed
      Run is itself unmeasured — the status lines will say).
- [ ] **K.6.2** **Expected:** the wheels stop within a blink. **Read the result on SERIAL, not on the
      brain:** the change that ends the run also starts the next phase function, which hands the screen
      over at once — `disabled()` to the **menu**, `opcontrol()` to the **driver page** — while the auton
      task is still unwinding, so the result page of an ended run is NOT painted, or shows only for a
      moment (the brain shows the menu after a disable). The request's first `[CMP]` line reads `… ABANDONED -- ROUTINE_RAN; ABANDONED: the
      robot was disabled (status 0x…), end action …` (or `the match left autonomous`), the end action
      `skipped` (or `CUT by the abandon` if the brake had begun — nothing is wrong then); when driver
      control follows, the
      driver page's auton row reads `auton ABANDONED: …`. **A first line reading `ABANDON IGNORED` is a
      STOP** (on serial — no page shows it here): the change was seen and the run went on — copy
      everything, never run it on the floor.
- [ ] **K.6.3** **HA-161, from the program's own lines** (copy them with their milliseconds, IN THEIR
      SERIAL ORDER). The change's own mark is the **next phase function's `entry` line** (`disabled() entry
      at <ms>` or `opcontrol() entry at <ms>`): PROS starts that function at the change, and it logs that
      line first. (The auton task's `competition status … at <ms>` line cannot mark the change: it cannot
      log while it serves a run, so a change during a run is logged only after the run's lines, and only
      when the status then differs from the last one it logged.)
      **(a)** PROS deleted `autonomous()` at the change if its **`autonomous() normal return` line NEVER
      appears** for this run; a `normal return` logged **after** the next phase function's `entry` line
      **refutes** (a) (`autonomous()` was still running when the next phase began); a `normal return`
      logged **before** it is **AMBIGUOUS** — the watch's abandon finished the run before PROS acted on the
      change — record it as such; it settles nothing. **(b)** the program's own task survived the change if
      the run's **`[CMP] request <n>: …` lines appear at all** — the auton task writes them only after the run
      has ended, so after the change — and its `competition status` lines continue at later changes.
      **The limit:** an auton that had already finished logs a normal return legitimately — this settles
      nothing unless the run's first line says `ABANDONED`. (A request line `a driver loop still held the
      robot when it was served -- it LEFT` appears only after a change straight from driver control to
      autonomous; if one appears anywhere in K, copy it: PROS did not delete `opcontrol()` at that change.)
      → `normal return` for this run: **NEVER / at ____ ms, BEFORE / AFTER the next `entry` line** ; the
      request's lines present: **YES / NO**

**K.7 — the SD logs and the serial**

- [ ] **K.7.1** Copy `shulib_auton_runs.txt` off the card: after Station M's runs it must carry K's boot
      lines and every K request's lines, in order (the competition program appends to the tester's file).
      → present, in order? **YES / NO**
- [ ] **K.7.2** Copy **`comp_log.txt`** off the card too (since T4 Part 3): this program's own log
      lines and summaries, APPENDED across boots — every boot's session header separates them — with
      the dead ports, every cut and the peaks, never a per-tick record (a slow loop's
      `fault=LOOP_OVERRUN` comes once per slow stretch, not per tick). Each line is written when it is
      logged (a card write in that tick; what that costs on the brain is unmeasured, HA-60). A line two
      parts of the program logged at the same instant may be on serial only (the card takes one
      writer at a time), and then the card's next line says so: `[WARN][DIA] copy: <n> line(s) left
      off here while another task was writing it -- serial has them` — copy any such line. **One
      case ends the copy instead** (`diag/log_copy_sink.hpp`, "A writer that never comes back"): if
      PROS deletes the driver-control task at a disable while it is writing a line to the card
      (HA-161 (a), reasoned — `opcontrol()` logs two lines right as its loop sees the disable), the
      card's copy ENDS for the rest of that boot, with no `copy:` line, and serial keeps every line.
      So compare the card with the serial capture at each disable of this station: a boot whose
      card copy stops at a disable while serial runs on is that case — note at which line (the first
      sighting of how often it happens; nothing has measured it).
      → present, one session header per boot of this station? **YES / NO** → a card copy that
      stopped at a disable while serial ran on? **NO / YES (at which line: ______)**
- [ ] **K.7.3** **The serial log's lines WHOLE** (since SER1, 2026-10-08 — A4 register HA-189's ROBOT TEST, accepted
      as such by the team lead 2026-10-18 in place of the dedicated settle (the note after K.7.4); and HA-190).
      Every line the program prints is now ONE write to the USB, which PROS keeps in one piece; before, two
      tasks printing at once — the auton task and PROS's phase tasks, at every mode change
      — could mix or lose bytes. Read K.0.3's saved transcripts around EVERY Timed-Run mode change of K.3–K.6
      (each phase's `entry`/`return` lines, the `competition status` lines, the auton task's `request` lines):
      every line starts with its tag (`[CMP]`, `[t=`, ...) and ends before the next one starts — no line
      spliced into another, none broken off mid-word, none printed twice. A line longer than one write
      arrives as pieces on purpose, the later ones beginning `[CMP] ... ` — that is whole.
      → every line whole at every change? **YES / NO** (copy any broken line word for word, with the lines
      around it). **HA-190:** a transcript that STOPS at a mode change while the brain still shows the program
      running — and the robot does not answer the sticks after the next enable, because every phase function
      logs its `entry` line first and blocks on it — is PROS's own serial lock left held by a task PROS
      deleted in the middle of a write (believed, never seen). `comp_log.txt` stops at the same line (the card
      copy is written after serial, so it waits too). Note the last line, then restart the program.
      → serial stopped at a mode change? **NO / YES (last line: ______)**
- [ ] **K.7.4** **`pros terminal` connected to a RUNNING program, once** (SER1): start the program from the
      brain with the USB cable unplugged, wait for the menu, then plug in and start `pros terminal`. PROS
      prints its banner, and the program keeps running — no halt, no red data-abort screen. (The first fix
      considered for HA-189, unbuffered stdout, was reasoned to crash exactly here — SER1's brief.)
      → banner printed and the program still running? **YES / NO**
      *(HA-189's dedicated settle — two tasks printing numbered lines through `pros terminal` for 60 s, with
      Timed-Run mode changes while they print, every line whole and every number once (SER1's brief, §1.5) —
      is NOT built: no build carries such a program (the bench program's menu is full, twelve stations in its
      3 × 4 grid). **The team lead ruled on 2026-10-18 ("Use Station K's K.7.3"): K.7.3 is accepted, in
      writing, as HA-189's robot test — no new Bench Tests station.** Its known weakness, accepted with it: it
      reads the competition program's own lines, which carry no sequence numbers, so a line LOST between two
      whole ones shows only as an expected line missing. HA-189 stays unsettled until K.7.3 runs.)*

**K.8 — THE PUSH CHECK (REQUIRED before this program drives a match — the team lead's N1; HA-176)** —
on the ground

A V5 motor pushing a wall sits near its current limit (the PROS default, 2.5 A — HA-154), above the
protections' **2.4 A for 250 ms** over-current cut. So a push may CUT the drive for 1 s in every 1.25 s
— in a match. Nothing of robot two's current under load has ever been measured; this is the
measurement.

- [ ] **K.8.1** Robot on the ground, square to a wall, a hand's width from it; brain-started (K.2's way),
      driver control live, the protections' row `drive OK`. **Full stick into the wall for 3 s, then let
      go.** Then read: did it CUT (the row red `CUT 1 s #1: ...`, serial `[WARN][TEL] CUT: port <n> (...)
      over 2.4 A for 250 ms ...` with its `at the cut:` lines), and the highest current — serial's `drive:
      LEFT peak <A> A (port <n>) over a <s> s drive, ...` line (printed when the stick is let go; the
      disable line may never print), and the `at the cut:` currents.
      → CUT? **YES / NO** · peak **L ______ A (port ____)  R ______ A (port ____)** · every `CUT:` line
        word for word: ______________________
- [ ] **K.8.2** Then, on the ground and **clear of everything** (3 m all round, a second person at the
      battery): a full-stick **LAUNCH from rest**, let go; then a full-stick **REVERSAL at top speed**
      (full forward until it is at speed, then full back at once), let go. Read the same peak line after
      each, and look for any `CUT:` line — HA-176 believes neither holds the current limit for 250 ms;
      a cut in either is a HEALTHY drive cut mid-manoeuvre.
      → launch: CUT? **YES / NO** peak ______ · reversal: CUT? **YES / NO** peak ______
- [ ] **K.8.3** If anything CUT: the team lead decides before any match whether the over-current cut
      stays on (`DriveProtectionConfig::overCurrentCuts`, one field) or its number changes (N1's
      answer) — send K.8.1 and K.8.2's lines. Read the push's log for a `went ABSENT` line too (HA-175:
      a live motor wrongly dropped at a wall).

**What to send back**

1. K.2: the banner, the three boot lines, the claw lines, the phase lines — word for word.
2. **K.3: every phase and status line with its milliseconds, and the three blanks** (or K.3.3/K.3.4's
   stop).
3. K.4's answer.
4. K.5: the table, the four `[CMP]` lines per run, and the transcript (or "not captured").
5. K.6: what the brain showed after the change (the menu, or the driver page's auton row), the request's
   lines (its first line's word is the result), the next phase function's `entry` line, and K.6.3's two
   answers.
6. K.7's two files, K.7.3's answers (and any broken or last line, word for word) and K.7.4's.
7. **K.8's answers and lines — the push check is required before a match.**
8. Anything surprising, as observed.

---

## DO NOT

- ❌ **Do not upload an old build.** The default build is the **bench tester**
      (`src/programs/bench_r3a.cpp`) — read-only except button 10 DRIVE, which powers nothing unless L1 is
      held and its six checks pass (button 12 AI VISION writes the camera's own detection
      settings, never a motor). For the 2026 tank chassis the build is `make ROBOT=tank`, and
      its driver program is `make ROBOT=tank PROGRAM=drive` (Station F — check F.0.1 before any wheel
      moves: an OLD "shulib Drive" package drives toward the other end). The invented X-drive
      is behind a build flag (`make ROBOT=xdrive`) and refuses at boot on either robot, naming
      port 1 and powering nothing (G1 Part 2c).
      *(Brain's programming port is **micro-USB**, not USB-C.)*
- ❌ **Do not run 10 DRIVE with a wheel touching anything** until Station D says to, and never
      without a second person at the battery.
- ❌ **Do not start "shulib Drive" with the robot on the ground** until Station F's wheels-up run
      (F.2) was clean, and **never before F.0.1 says it is the NEW program** — an OLD package in slot 1
      drives toward the OTHER end on the same stick; it powers the motors from the moment driver
      control starts.
- ❌ **Do not start a program from the CONTROLLER's own menu** (Run, Timed Run or Match) — start it
      from the brain's screen or the laptop. From the controller it arms a pretend match: the
      program restarts and sits DISABLED, never reaching driver control (Station A, step 1; A4
      register HA-128). *(Added 2026-09-26 by T3's addendum: G.0.5 and J.9.6 point here.)* **The
      one exception (COMP-b, 2026-09-30): slot 6, "shulib Competition", started with Timed Run at
      Station K** — then turn the controller off, wait ten seconds, and see the menu return before
      starting any other slot.
- ❌ **Do not disassemble anything** to read a gear — photograph it in place.
- ❌ **Do not "correct" a reading to what this sheet expects.** A contradiction is the most valuable
      output of the session.
- ❌ **Do not run "shulib Auton Tester" on a field, a competition switch or a Timed Run** (RUN is
      refused there anyway), and **never "correct" an auton's 40 to what the tape says** — the
      calibration record is what is wrong; re-run Calibrate (Station M, M.0.4). *(Added COMP-t,
      2026-09-28.)*
- ❌ **Do not rename or reconfigure ports** to match anything. The code moves to the robot, never the
      reverse.

---

## Before the helper leaves

- [ ] Every photo transferred off the phone.
- [ ] Station 2's table filled in completely — it is the one that cannot be reconstructed later.
- [ ] **3.2 answered PER SIDE**, not once.
- [ ] **Station 5.5 done** — it is the cheapest item here and the one that has been missed twice.
- [ ] **If Station M ran: `shulib_auton_runs.txt` copied off the SD card** (M.7), with the serial
      transcripts of M.6.
- [ ] **If Station K ran: every K serial transcript saved** (K.3's milliseconds are the measurement),
      `shulib_auton_runs.txt` and `comp_log.txt` copied off (K.7), and K.8's push-check lines.
- [ ] Anything surprising written down **as observed**, before anyone interprets it.

