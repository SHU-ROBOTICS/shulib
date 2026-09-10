# R3a — bench worksheet (no program required)

> **For a helper at the robot. Nothing here needs code uploaded.** The shipped `src/main.cpp`
> **cannot boot on this robot** — its port map is invented and wrong (R3a brief §2) — so do not try
> to run it. Everything below uses the brain's built-in **Devices** screen, a tape measure, and hands.
>
> Answers get pasted **raw, uninterpreted** into `R3a-PROGRESS.md` Batch 1 before any analysis.

---

## Station A — RUN THE PROGRAM (no laptop needed)

**The program is already on the brain: slot 3, named "Bench Tests".** It is **read-only and commands
no motion — EXCEPT button 10 DRIVE**, which powers the drive motors and has its own station (D)
below. Nothing else on the menu can move the robot.

1. Power the brain on. Open **Programs** and run **slot 3 — "Bench Tests"**.

> ⚠ **Slot 1 holds an OLD build called "QueensRevenge". Do not run it.** Its port map is invented and
> it will fault at boot on this robot. It can only be deleted from the brain's own Programs menu —
> the PROS CLI has no remove command. Deleting it is the safest move so nobody picks it by mistake.
2. A **boot splash** shows a **BUILD** stamp for two seconds, and the same stamp sits on the menu.
   **If that stamp is not the build you just uploaded, the upload did not land** — re-upload before
   trusting anything on screen. (A silent upload failure cost a debugging cycle on 2026-08-18.)
3. A **touch menu** appears. Tap any test:

   **An amber stripe down a button's left edge means that test needs your hands.**
   Each test also opens with two amber lines — `DO NOW` and `DONE IF` — so you never
   have to guess what you are supposed to do or when you are finished.

   | Button | Hands? | What YOU do | Finished when |
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

   Each button carries a **status dot**: hollow until run, then filled — green PASS,
   amber CHECK, red FAIL. That is how you tell what is left without keeping a list.
   **Button 10 has a RED stripe and a red edge instead of amber: it is the only one that
   powers motors.** *(This table was corrected at R3b Session 2 — it had listed the menu as it
   was before MOTOR WATCH existed.)*

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
      this choice. → front = **________** (describe it: "the intake end", "the battery end", …)
- [ ] **D.0.7 Report the drive ports, PER SIDE**, from the Devices screen or button 1:
      → **LEFT:  ____ ____ ____ ____ ____**  **RIGHT: ____ ____ ____ ____ ____**
      *(Left and right as seen from behind the robot, looking toward the front you chose.)*
- [ ] **D.0.8 External gearing motor → wheel**: tooth counts, or "direct". → **________**
      *("600 rpm" was reported, which reads as direct drive; the count settles it.)*

> **STOP here and send D.0.3–D.0.8 in.** The chassis table in `src/bench_r3a.cpp` ships with
> the ports **UNSET** on purpose — nothing is guessed — and until they are typed in, button 10
> refuses with `chassis table UNSET — missing: LEFT ports, RIGHT ports`. A rebuilt program with
> the ports filled in is what the rest of this station runs on.

**D.1 — census → push → drive (with the rebuilt program on the brain)**

- [ ] **D.1.1** Run **1 DEVICE CENSUS**. Every reported port shows **MOTOR** with its side from the
      table beside it (`LEFT(table)` / `RIGHT(table)`). A port with **`?`** or a missing port is a
      finding — write it down, do not continue to D.1.3.
- [ ] **D.1.2** Put the robot on the floor, wheels down. Run **3 MOTOR WATCH**. **Push the WHOLE
      robot, FRONT END LEADING, about a foot.** Watch the panel: every drive port should show
      **UP** or **DOWN**. *(UP and DOWN mixed **within** a side is normal — adjacent motors on one
      gear train spin opposite ways. It is not a wiring fault.)* Touch the screen.
      → It asks **WHICH WAY DID YOU PUSH IT?** — answer honestly (**FRONT FIRST** if the end you
      chose as the front led). If you are not sure, re-run and push again, one way only.
      **DONE IF** it prints **`WHOLE-ROBOT PUSH … SIGNS CAPTURED`** and a line like
      `signed ports for DRIVE: LEFT +1 -2 +3 … | RIGHT …`. Copy that line here:
      → **LEFT: ______________________  RIGHT: ______________________**
      If it says **PARTIAL** (a port did not move) or **single-wheel spin**, push again, firmer,
      all wheels on the floor. A port that never moves is a finding.
- [ ] **D.1.3** **Lift the robot onto blocks so NO wheel touches anything.** Check it cannot rock
      off. Have a second person ready at the battery.
- [ ] **D.1.4** Tap **10 DRIVE (POWERS)**. Read what it prints — it states the **cartridge belief**
      it is about to write to every motor and the **signed ports** from D.1.2. **If the cartridge
      it names is not the colour you read in D.0.3, STOP** and report it; do not tap YES.
      Then it asks **ARE THE WHEELS OFF THE GROUND?** — tap **YES, WHEELS UP** only if D.1.3 is true.
- [ ] **D.1.5 Drive at 3 V, wheels up.** Hold **L1** (left index finger — the button on the
      controller's top edge, not the sticks) and push the **left stick forward** a little. All
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

## DO NOT

- ❌ **Do not upload an old build.** The default build is the **bench tester**
      (`src/bench_r3a.cpp`) — read-only except button 10 DRIVE, which powers nothing unless L1 is
      held and its six checks pass. For the 2026 tank chassis the build is `make ROBOT=tank`. The
      invented X-drive wiring is behind a build flag and would fault at boot on either robot.
      *(Brain's programming port is **micro-USB**, not USB-C.)*
- ❌ **Do not run 10 DRIVE with a wheel touching anything** until Station D says to, and never
      without a second person at the battery.
- ❌ **Do not disassemble anything** to read a gear — photograph it in place.
- ❌ **Do not "correct" a reading to what this sheet expects.** A contradiction is the most valuable
      output of the session.
- ❌ **Do not rename or reconfigure ports** to match anything. The code moves to the robot, never the
      reverse.

---

## Before the helper leaves

- [ ] Every photo transferred off the phone.
- [ ] Station 2's table filled in completely — it is the one that cannot be reconstructed later.
- [ ] **3.2 answered PER SIDE**, not once.
- [ ] **Station 5.5 done** — it is the cheapest item here and the one that has been missed twice.
- [ ] Anything surprising written down **as observed**, before anyone interprets it.

