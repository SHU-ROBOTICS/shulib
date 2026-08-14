# R1a Bench Runbook — first contact between shulib and a V5, step by step

> **Who this is for:** the team lead, standing at a table with the robot (the OLD COMPETITION BOT —
> see `R1a-BENCH-SESSION.md` for its measured device map), a V5
> brain, a charged battery, a USB cable, and a laptop with `pros terminal`.
> **What it is:** the ordered checklist R1a's brief demanded instead of an unrun bench session.
> Each step names **what it measures**, **which `HA-nn` it settles**, and **what a wrong answer
> looks like** — so a surprise is recognized at the table, not explained away later.
>
> **Ground rules (from the brief, §6.3):**
> - **Record everything** — keep the `pros terminal` transcript; label every number
>   **measured-on-tank-bot**. One measurement on one unit is not proof of portability.
> - **This is a smoke session, NOT validation.** R3 owns validation. Do not tune a gain, do
>   not claim an accuracy number, do not mark a register entry settled without the logged
>   evidence.
> - **The tank bot validates the platform layer and the tank kinematics path only.** It
>   validates NONE of the holonomic thesis — no strafe authority, no pseudo-inverse, no
>   per-axis decoupling, no H-drive geometry. Those register entries STAY OPEN, and the
>   session record must say so out loud.
> - Steps are ordered so that **nothing drives before its prerequisites are believed**: the
>   battery scale is checked before any motion (it scales every motor command), signs before
>   any closed loop.

---

## Step 0 — Device inventory and the real port map

**Measures:** the actual port of every motor and sensor, and each drive motor's cartridge color.
**Settles:** HA-111 (the invented port map in `src/main.cpp`); the gearset half of HA-15's
stand-in (`kDriveGearset`).
**Do:** on the brain's screen open Devices; write down port ↔ device type for every plugged
device. Pop the cap color off one drive motor if unsure (red 100/green 200/blue 600 RPM).
Edit the `k*Port` constants and `kDriveGearset` in `src/main.cpp`, rebuild (`make`), upload.
**A wrong answer looks like:** step 1's boot faults with a ProsMotor read-back precondition —
that IS the port map being wrong, doing its job. Fix the constant, not the check.

## Step 1 — Boot behaviour, session header, and the T8 question

**Measures:** the full boot path with real adapters; **what an uncaught precondition throw out
of `initialize()` actually does on the brain** — the T8 open question, which C2 answered only
on the host.
**Settles:** the T8 behaviour record (no HA number — it becomes one if the answer is scary);
exercises HA-98's read-back path.
**Do:** (a) with everything plugged and mapped, boot with `pros terminal` attached. Expect:
the §18.5 session header FIRST, with a real build hash (the word MISSING = the Makefile
injection failed — stop and fix), the port map line matching step 0, a battery line, the
banner, `strafeAuthority=1.00`. (b) Then unplug ONE drive motor, reboot, and watch: the
ProsMotor read-back precondition must fire. **Record exactly what the brain shows** — error
screen? task death? anything over serial first?
**A wrong answer looks like:** (a) a healthy boot with a motor unplugged — the read-back
check is not reaching the device; (b) a silent hang with nothing over serial — the throw is
dying invisibly, and the precondition policy needs a serial print BEFORE the throw (report
back to the register; that is a design change, not a bench fix).

## Step 2 — Battery raw units, BEFORE anything drives

**Measures:** the raw integer `battery_get_voltage()` returns, and capacity's range.
**Settles:** HA-99 (mV/mA — the WEAKEST belief in R1a: the vendored source documents NO unit;
the mV belief is from PROS's website), HA-100 (percent).
**Do:** read the session header's `batt` field and the fault display's battery row against
the brain's own battery screen.
**A wrong answer looks like:** `batt 12600.00V` (raw mV passed as V — the ÷1000 is missing or
the unit is not mV) or `batt 0.01V` (double-applied). Either way STOP — brownout compensation
scales every motor command by this number; nothing drives until it reads ~12.x V.

## Step 3 — Clock rate

**Measures:** `pros::micros()` against a stopwatch.
**Settles:** HA-101.
**Do:** watch the `[t= ...]` column in the terminal for 60 wall-clock seconds.
**A wrong answer looks like:** the column gaining ~0.06 s (µs belief wrong by 1000×) or
~60000 s per minute. Every dt in every controller divides by this — a wrong scale here is
every derivative term wrong.

## Step 4 — One motor, open loop: command scale, encoder units, direction signs

**Measures:** `move_voltage`'s scale, `get_position`'s units after explicit configuration,
`get_actual_velocity`'s RPM, and each drive motor's physical direction.
**Settles:** HA-94, HA-95, HA-96, HA-98; the sign half of HA-111.
**Do:** wheels OFF the ground. In opcontrol, ease the left stick forward a little and watch
the per-wheel telemetry while looking at the wheels.
**A wrong answer looks like:**
- robot hums, wheels barely creep at full stick → the mV scale is off (HA-94);
- position creeps at 1/360 of the wheel's obvious rotation → encoder units were not accepted
  (HA-95/98 — the read-back should have caught it at boot; if it did not, that is a FINDING);
- velocity ~9.5× larger than plausible → RPM passed through as rad/s (HA-96);
- a wheel spins backward relative to its three siblings → flip that port's sign in the map
  (HA-111) — do NOT touch the kinematics.

## Step 5 — Motor current and temperature

**Measures:** `get_current_draw`'s mA belief at stall-ish load.
**Settles:** HA-97.
**Do:** at LOW voltage (≤ 3 V), briefly pinch one wheel by hand; watch current.
**A wrong answer looks like:** thousands (mA passed as A — stall detection would fire on a
free wheel) or milliamp-looking decimals under load. Temperature should read ~20–40 °C.

## Step 6 — Rotation sensors, by hand

**Measures:** centidegree scale, cumulative (non-wrapping) position, reversal-once.
**Settles:** HA-11, HA-16, HA-105.
**Do:** roll each tracking wheel exactly 5 marked revolutions forward by hand, slowly; then 2
backward. Watch `position()` in telemetry.
**A wrong answer looks like:** a value that jumps back at each full turn (bound to a wrapping
angle — a mis-binding, adapter bug); ~100× too large or small (centidegree scale); the
forward roll DEcreasing (port sign — fix the map, never add a negate in code).

## Step 7 — IMU: calibration, heading sign, cumulative continuity

**Measures:** calibration time and readiness; the as-mounted CW convention; get_rotation's
unboundedness.
**Settles:** HA-02, HA-03, HA-23, HA-108.
**Do:** boot; time `isReady()` false→true (expect ~2 s, HA-23). Then rotate the robot
**+90° clockwise** against a wall/protractor: canonical heading must **DECREASE** by 90°
(imu_conversion.hpp:28-31). Then spin it a slow full 450°: the canonical heading must move
smoothly through the wrap with no 360° step.
**A wrong answer looks like:** heading INCREASING on a CW spin — the documented line to flip
is the subtraction in `imuHeadingToCanonical` (flip it in the CONVERSION, with the register
updated, never in the adapter); a step at 360° — the adapter is somehow reading a bounded
source (adapter bug, stop).

## Step 8 — IMU yaw-rate: the gyro z sign (T4's mandated measurement)

**Measures:** `get_gyro_rate().z`'s sign and unit under a known physical rotation — the
UNDOCUMENTED quantity both briefs flagged.
**Settles:** HA-04 (sign), HA-109 (deg/s unit, z = yaw axis).
**Do:** rebuild with `YawRateSource::GyroRateZ` in main.cpp (one ctor argument). Spin the
robot CW at a steady slow rate (~90°/s by metronome/count). Canonical yawRate must read
**negative**, magnitude ≈ 1.57 rad/s for 90°/s. Repeat CCW → positive. Then rebuild with the
default differentiation source, same spin, compare magnitudes.
**A wrong answer looks like:** positive on CW → z is CCW-positive on this unit: record it,
flip the assumption in the ADAPTER's gyro branch (one negate), and keep the register entry
labelled measured-on-tank-bot — one unit is not all units.

## Step 9 — IMU pitch/roll signs

**Measures:** the as-mounted sign convention of `get_pitch()`/`get_roll()`.
**Settles:** HA-110.
**Do:** tip the robot nose-up ~10° on a book: record pitch's sign. Tip its left side up:
record roll's sign.
**A wrong answer looks like:** there is no wrong answer yet — the adapter passes through
unnegated BY DESIGN until this measurement exists. Whatever you record becomes the convention
the tip detector is written against.

## Step 10 — Controller: axes, buttons, the disconnect signal

**Measures:** axis range and mapping, level reads, `is_connected` behaviour.
**Settles:** HA-103; exercises HA-104's design (level reads + ButtonEdge).
**Do:** wheels OFF the ground. Full left-stick up → telemetry axis +1.00 and wheels forward;
half stick ≈ +0.50. Hold a button; confirm it reads held (level), not one-shot. Then **pull
the controller's cable / power it off mid-stick**: the robot must stop within a tick (zero
twist commanded), and reconnect must resume.
**A wrong answer looks like:** the robot continuing at the last stick value while
disconnected — is_connected is not being honoured (adapter or loop bug: stop).

## Step 11 — Controller LCD: the 15-vs-19 column conflict

**Measures:** how many characters one LCD row actually shows.
**Settles:** HA-107 (the conflict R1a found: the vendored set_text doc says col [0-14] ⇒ 15
columns; HA-57 says 19; neither is a measurement).
**Do:** make the fault display (or a scratch line) show the 19-char ruler
`0123456789ABCDEFGHI`. Count what is visible on the physical LCD.
**A wrong answer looks like:** 15 visible → HA-57 is FALSE: shrink `ILineDisplay::kCols` to
the measured value (one constant, C5's content layer already truncates through the seam) and
record HA-57 settled-false, HA-107 settled. 19 visible → HA-57 confirmed, HA-107 records the
doc comment as wrong.

## Step 12 — Loop cadence under the real pacer

**Measures:** the 100 Hz tick under ProsTickPacer with the full stack idling.
**Settles:** evidence toward HA-32 (full validation is R3's, under load).
**Do:** idle in opcontrol 60 s; read LoopMonitor's dt stats from telemetry.
**A wrong answer looks like:** mean dt ≈ 12 ms (pacing drifting with work — delay_until not
doing its job) or huge jitter spikes (record for HA-34).

## Step 13 — GPS without (and with) a strip

**Measures:** the no-fix path, the boot offset check on a factory device, get_error's raw
magnitude; the position-axis mapping IF a strip is available.
**Settles:** HA-06 (factory offset reads (0,0)), HA-08 (sentinel screening), HA-07 (raw
error ~0.01–0.05 = meters), HA-106 (yaw convention); HA-01 + HA-09 ONLY with a strip hung.
**Do:** with no strip: confirm `hasFix()` false continuously, no crash, faultedReads
climbing. With a strip: place the SENSOR at a known point at a known heading; log raw
`get_position()`/status.yaw; run the field-cal oracle procedure (gps_conversion_test.cpp:164
— replace the skipped oracle's expected values with the measured mapping and unskip it).
**A wrong answer looks like:** a crash off-strip (screening failed — adapter bug, stop);
boot precondition on a factory-fresh device (offset not actually (0,0) from factory —
a REAL finding, record it); get_error ~1–2 on-strip (it is returning inches/other, not
meters — HA-07 false, fix the conversion).

## Step 14 — The unplug matrix

**Measures:** live-disconnect behaviour of every sensor read path (the T7 screen).
**Settles:** HA-08's live half.
**Do:** while idling in opcontrol with telemetry visible, unplug then replug, one at a time:
IMU (expect IMU_LOST fault + heading held), one rotation sensor (position held, faultedReads
climbing), GPS (hasFix false), one drive motor (position/velocity/current held).
**A wrong answer looks like:** any crash, any NaN in telemetry, any value snapping to zero
(the T7 design forbids exactly that — a zeroed reading is the invisible-runaway shape).

## Step 15 — Write the session record

Into `docs/internal/chunks/R1a-BENCH-SESSION.md`: date, robot (name the ACTUAL robot — the first session found it was the old competition bot, not the practice bot these documents assumed), firmware
versions, the transcript file, and for each step above: the measured value, the HA-nn it
settles, and settled-true / settled-false / still-open. Update
`docs/hardware-assumptions.md` per its own rules (measured value recorded next to the guess
it replaces). Do NOT flip anything the tank bot cannot see — the holonomic entries stay open
and the record says so. *(Steps 16–20 below were added by R1b — fold their results into the
same record; this step applies after whichever steps a session actually ran.)*

---

## Steps 16–20 — the mechanism-sensor beliefs (added by chunk R1b, HA-113…122)

> None of these sensors is known to be ON the available robot. Every step below works with
> the sensor on a loose cable at the table — say in the record which port it was plugged
> into. Step 17 comes first if session time is short: it is the one that un-blocks planning.

## Step 16 — SD card: the blackbox's device

**Measures:** the card probe, the `/usd/` write path end to end, and what a flush is worth.
**Settles:** HA-122 (all three halves).
**Do:** (a) NO card: boot — the robot must run normally; log `ProsBlockSink::isOpen()` =
false, reported once. (b) Card in: boot, let the blackbox write, power off, read the file on
a laptop (its existence at the expected name settles the prefix belief; its bytes should
decode with the blackbox reader up to the cut). (c) Yank-after-flush: write, flush, wait 2 s,
pull the card mid-run; count surviving bytes against the write log.
**A wrong answer looks like:** (a) a boot hang or throw with no card — the T5 ruling failed,
STOP, that is an adapter bug; (b) no file — the fopen prefix belief is wrong: log errno,
record HA-122 settled-false, fix the ONE join in the adapter; (c) heavy loss after a clean
flush — record the measured loss window; E1's format tolerates it but R4's write-budget
numbers must then absorb it.

## Step 17 — ADI inventory: does this robot actually have an expander?

**Measures:** the real 3-wire capacity of this robot; the truth behind R1a's unverified
expander report (it came from registry index 21 — OUTSIDE the documented 0–20 range — and
was never re-checked).
**Settles:** HA-120's open question; informs every mechanism port plan.
**Do:** brain screen → Devices, photograph it; then run a registry scan over the CORRECT
0-indexed 0–20 range and compare. Plug a switch into brain ADI 'a' and read it via port
spellings 'a', 1, and 'A' — all three must read the same line (the addressing half).
**A wrong answer looks like:** the three spellings reading different lines (adapter or
belief bug — STOP); an expander appearing only in the out-of-range read (it does not exist;
correct the R1a session's inventory in place, with a note).

## Step 18 — Distance sensor: mm, the 9999 no-object value, confidence's two ranges

**Measures:** the raw mm scale; whether "no object" is exactly the constant 9999; what
`get_confidence()` actually returns at and below 200 mm (currently a stated UNKNOWN).
**Settles:** HA-113, HA-114, HA-115.
**Do:** sensor on a cable, facing across the table. (a) Tape-measure an object at 500 mm —
raw ≈ 500, canonical ≈ 19.7 in. (b) Point it at open air ≥ 30 s, log raw + adapter values
every tick: raw must sit at 9999 with adapter confidence() 0.0 throughout. (c) Walk an
object from 400 mm to 50 mm, logging raw distance AND raw confidence: record the raw
confidence behaviour below 200 mm — whatever it does, write it into HA-115 (that settles the
unknown), and confirm the ADAPTER's confidence() reads 1.0 through the close range.
**A wrong answer looks like:** (a) raw ≈ 19.7 (it is already inches?! HA-113 false — fix the
conversion, not the adapter); (b) raw flickering 9998/10000 — the EXACT-9999 belief is too
narrow: widen the screen to the measured band, register the correction; (c) adapter
confidence dipping to 0 with an object at 100 mm — the close-range rule is not being reached
(adapter bug, STOP).

## Step 19 — Optical sensor: ranges, and proximity's UNMEASURED polarity

**Measures:** hue/saturation ranges on known colors; proximity's direction — the belief the
vendored doc does not state (larger = closer is currently INVENTED, HA-117).
**Settles:** HA-116, HA-117, HA-118 (with step 14's unplug matrix).
**Do:** (a) hold a known-red and a known-blue game piece at ~30 mm: hue ≈ 0–20° / 210–230°,
saturation > 0.5. (b) THE POLARITY PAIR, before anything thresholds on proximity(): log raw
`get_proximity()` with the lens covered by a hand, then facing open air. (c) unplug mid-read:
adapter values hold, faultedReads climbs, no infinity anywhere.
**A wrong answer looks like:** (b) covered ≪ open — the polarity is INVERTED: put the
documented 1−x flip in `opticalProximityToCanonical` (the conversion, never the adapter),
correct HA-117, re-run the conversion tests; (a) hue off by a constant offset — record it;
color WINDOWS are a consumer decision and the window author needs the measured centers.

## Step 20 — ADI digital lines: construction actuation, levels, and the dead-port value

**Measures:** whether constructing a DigitalOut really drives the line AT construction (vs at
the first tick); level semantics of DigitalIn; what `get_value()` actually returns from a
dead/misconfigured port (PROS_ERR is the belief; 0 would blind the screen).
**Settles:** HA-119, HA-121.
**Do:** **AIR DISCONNECTED FIRST.** (a) Construct a ProsDigitalOut with initialState=false,
then =true (two boots): watch/listen for the solenoid click at program start — record WHEN it
fires. (b) With air, construct with the mechanism's declared safe state: there must be NO
motion at boot. (c) Wire a limit switch: held closed, `state()` must read a steady level (not
a one-shot); release/press ten times and confirm ten clean edges through ButtonEdge. (d)
Unplug the switch's wire mid-run and log the RAW `get_value()` — that number settles
HA-121's refusal-value half.
**A wrong answer looks like:** (a) the click at the first loop tick rather than construction
— HA-119's timing half is wrong: the boot-glitch window is different, re-reason the T3 note
(the required-argument design still stands); (b) ANY motion at boot with matched safe states
— STOP, that is the exact defect the agreement test exists to prevent; (d) raw 0 — the
screen never fires on a dead port: record it, and note that faultedReads() cannot see this
failure mode on this firmware.
