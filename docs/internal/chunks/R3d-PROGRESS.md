# R3d — drivetrain self-calibration — PROGRESS (live log)

| | |
|---|---|
| **Chunk** | R3d — drivetrain self-calibration |
| **Part** | Part 1 (host-only: description + calibrator + plant extensions) |
| **Date** | 2026-09-17 |
| **Tree at start** | `76a5819` (`docs(internal): R3d brief -- drivetrain self-calibration, Part 1 (host-only) ready for the go`), clean |
| **Executor model** | opus (Claude Opus 5, 1M context) |
| **Commit policy** | **The executor commits NOTHING.** No `git add`, no `git commit`, no `git push`, no worktree. The coordinator verifies independently and commits. |

> This log is appended **in real time**, after every meaningful step. If the work is interrupted,
> this file is the honest record of exactly how far it got. Rulings made beyond the brief are
> recorded with the alternative considered and the reason. Every mutation is recorded with the
> **observed** failure message, not a claim that it would fail.

---

## Order of work

Following the brief's landmine 4 (land the plant extensions first, so every later test can lean on
them) and the coordinator's ordering:

1. **PROGRESS log created** (this file). Record the clean-tree suite count *before* touching anything.
2. **`include/shulib/sim/drive_plant.hpp`** — the two additive extensions of brief T6
   (`MemberFault::Reversed`; the coupled-train command model) + their tests (brief §5 tests 18–20)
   + the whole-suite bit-identity pin recorded (count before == count after, pre-existing tests
   untouched).
3. **`include/shulib/hal/drive_geometry.hpp`** — `fromInchesPerRadian` factory + test 3;
   **`include/shulib/hal/drivetrain_description.hpp`** (new) + test 1;
   **`src/chassis_table.hpp`** — `tableToDescription` + test 2.
4. **`include/shulib/calibration/drivetrain_calibrator.hpp`** (new directory) — phase by phase,
   each with its plant tests and mutations as they land (tests 4–17).
5. **Reproducibility** (test 13) last, with the observed spread recorded.
6. **All verification commands**; the documentation contract (brief §7); the prose review, logged
   per file; the DoD checklist with evidence per line; the final clean-tree suite count.

---

## Log

### Step 0 — required reading + clean-tree baseline

*(in progress — appended below as it happens)*

**Clean-tree baseline, measured BEFORE touching anything** (`cmake --build build/test -j$(nproc) && ./build/test/shulib_tests | tail -6`, tree `76a5819`, clean):

```
[doctest] test cases:    1208 |    1208 passed | 0 failed | 3 skipped
[doctest] assertions: 1563156 | 1563156 passed | 0 failed |
[doctest] Status: SUCCESS!
```

This is the number the brief's test 18 (T6 bit identity) is pinned against: the pre-existing suite
must still report **1208 cases / 1563156 assertions** from the files that existed at `76a5819` after
the two `sim/drive_plant.hpp` extensions land. (New test files add to the totals; the check is that
no pre-existing file's contribution changes, verified by re-running with the new test files
temporarily excluded as well as by the per-file counts.)

### Step 0 — required reading, done

Read in full: `RESUMING.md`; the brief `R3d-drivetrain-self-calibration.md` (§0–9 = Part 1, §10 =
Parts 2–3, NOT built); `hal/motor_group.hpp`, `hal/drive_geometry.hpp`,
`localization/drive_encoder_odometry.hpp`, `hal/motor.hpp`, `hal/imu.hpp`, `hal/clock.hpp`,
`teleop/coupled_side_monitor.hpp`, `sim/drive_plant.hpp` (whole), `sim/scenario.hpp`,
`sim/hostile/{slip,encoder,imu}_hostility.hpp`, `sim/motor_model.hpp`, `src/chassis_table.hpp`,
`src/bench_r3a.cpp:900-1150` (MOTOR WATCH), `test/motor_group_plant_test.cpp`,
`test/drive_encoder_odometry_test.cpp`, `test/chassis_table_test.cpp`, `test/motion_test_rig.hpp`,
`test/README.md`, `core/check.hpp`, `units/quantity.hpp`, `math/angle.hpp`,
`hal/fake/{fake_motor,fake_imu,fake_clock}.hpp`, `kinematics/tank.hpp`,
`docs/hardware-assumptions.md` (Robot 2 section HA-124–133 + the entry format),
`.github/workflows/ci.yml` (the exact guard greps), `tools/api_doc_tool.py`'s header,
`docs/roadmap.md`'s "You are here", `docs/internal/build-order.md`'s R3d entry + first `Next:`.

Survey of who uses the pieces R3d extends (so the bit-identity pin has a scope):
`attachCoupledMembers` / `setMemberFault` / `MemberFault` appear in exactly ONE place outside the
plant — `test/motor_group_plant_test.cpp` — and there every member of a wheel is always commanded
through a `hal::MotorGroup`, i.e. the SAME volts. That is what makes the T6(b) equal-command
shortcut bit-identical for the whole pre-existing suite rather than merely "probably".

---

### Step 1 — `sim/drive_plant.hpp`: the two T6 extensions (landed FIRST, per landmine 4)

**What changed** (`include/shulib/sim/drive_plant.hpp`):

1. `MemberFault::Reversed`, **appended** to the enum (`SignFlipped`'s semantics untouched —
   landmine 8). Read side: falls through with `SignFlipped` to the same negation. Command side:
   a new private `effectiveMemberVolts(commanded, fault)` negates a `Reversed` member's volts.
2. The coupled-train command model, a new private `wheelCommand(w)` called from `step()` step 1
   in place of `motors_[idx]->commandedVoltage()`. Equal raw commands (EXACT `!=` comparison)
   → the primary's value, the pre-R3d line bit for bit; otherwise the mean of the members'
   *effective* volts.
3. Header banner step 1, `attachCoupledMembers`'s doc and the `DebugRecord` line updated to say
   what is now true. The record still logs the PRIMARY's commanded volts, with a comment saying
   why: it is telemetry of what the stack asked for, and a `MotorGroup` fan-out makes the two
   the same value anyway.

**Bit-identity pin (brief test 18's wider half):**

| | cases | assertions |
|---|---|---|
| clean tree `76a5819`, before any edit | 1208 | 1,563,156 |
| after the two `sim/` extensions, before any new test file | **1208** | **1,563,156** |

Identical. Then `test/drive_plant_coupled_test.cpp` (3 cases, +4,031 assertions) takes the tree
to 1211 / 1,567,187.

**RULING R1 — `Reversed` negates the command for EVERY member, not only the primary.**
The brief's T6(a) says "*if it is the PRIMARY* its commanded volts are negated". That sentence
was written when the plant read a wheel's command from the primary alone; T6(b) generalises the
command read to every member, and the negation has to generalise with it or the model becomes
incoherent. Worked through: the calibrator pulses ONE member with `sign[i] × V`. For a `Reversed`
EXTRA the captured sign is −1, so it commands −3 V. Without the generalisation the mean is
−0.6 V, the wheel rolls BACKWARD, the member's (negated) reading rises, and `sign × reading` comes
out NEGATIVE — the calibrator would refuse a correctly-signed member as a sign contradiction
(T3's own-encoder check), on the plant, forever. *Alternative considered:* keep the negation on
the primary only and have the calibrator special-case extras — rejected: it puts a fault model's
consequence inside the code under test, which is exactly backwards. The negation belongs to the
member's wiring, not to its index.

**RULING R2 — what a `Reversed` member READS, spelled out, because the brief's test-19 prose is
ambiguous.** Test 19's row says "the plant drives BACKWARD under +V and the member reads
negative". Taken literally with T6(a)'s "*reports negated position*" the two halves contradict
each other: if the wheel is driven backward its synthesized shaft is already negative, so the
NEGATED reading is POSITIVE. The mechanism in T6(a) is the operative statement and it is also the
physically right one for a V5: a motor mounted the other way round on the gear train still counts
its OWN rotation as positive under +V, while the WHEEL turns the other way. So: commanded +V →
wheel backward → member's encoder counts UP; hand-pushed forward → member's encoder counts DOWN,
which is the −1 the push capture must record. The test asserts both directions explicitly
(`DrivePlant 19`), and the end-to-end consequence (a `Reversed` member's own encoder advances
POSITIVE under `sign × V`) is what makes T3's cross-check pass rather than fire.

**Mutations run — every one rebuilt, run, and the failure OBSERVED:**

| # | Mutation | Observed |
|---|---|---|
| M18 | `if (false && allSame)` — the equal-command shortcut made an unconditional mean | **RED**, three cases: `drive_plant_coupled_test.cpp:140 REQUIRE( five.member(0, m).position().value() == one.member(0, 0).position().value() ) … values: REQUIRE( -0.240444 == -0.240444 )` (bit-level, prints equal), and the PRE-EXISTING pin twice — `motor_group_plant_test.cpp:189 FATAL ERROR: REQUIRE( sameTruth(base.truth, group.truth) ) … REQUIRE( false ) logged: seed := 1`, on both the clean and the FullHostility tank sweeps |
| M19a | `effectiveMemberVolts` returns `commanded` always (drop the command negation) | **RED**, 9 assertions, e.g. `CHECK( reversed.member(0, 0).velocity().value() == healthy.member(0, 0).velocity().value() ) … values: CHECK( -14.4796 == 14.4796 )`; `CHECK( pushed.h.truePose().x().value() > 2.0 ) … values: CHECK( 0 > 2 )`; `CHECK( rev.h.plant().trueWheelSpin()[0].value() == Approx(3.52941) ) … values: CHECK( -3.52941 == Approx( 3.52941 ) )` |
| M19b | `Reversed` given its own `writeMember` case that does NOT negate the reading | **RED**, 8 assertions, e.g. `CHECK( faultedFive.member(0, 3).position().value() == -cleanFive.member(0, 3).position().value() ) … values: CHECK( 1.97464 == -1.97464 )`; `CHECK( flippedFive.member(0, 3).position().value() == faultedFive.member(0, 3).position().value() ) … values: CHECK( -1.97464 == 1.97464 )` |
| M20 | `wheelCommand` returns the primary's effective volts always (the pre-R3d line) | **RED**, all 6 assertions of `DrivePlant 20`, e.g. `CHECK( train.h.plant().trueWheelSpin()[0].value() == Approx(3.52941) ) … values: CHECK( 0 == Approx( 3.52941 ) )` — the train sat still, which is exactly the "passes for the wrong reason" trap of landmine 4 |

Restored after each; full suite re-run green (1211 / 1,567,187).

**A trap worth recording for whoever runs this next.** The `-tc="…"` runs above first reported
GREEN under mutation M18. The cause was not the mutation: `cmake --build build/test` runs
`shulib_doc_gates` BEFORE compiling, the briefing gate had gone stale the moment the suite count
moved, and the failing gate stopped the build — so the *previous* binary was re-run and passed.
A mutation "observed green" here would have been a lie told by a stale executable. Every mutation
above was therefore re-run after `python3 tools/briefing_status.py generate`, with the build's
tail inspected for `Linking CXX executable shulib_tests` before trusting any result.

**A finding about the test, not the code.** The first draft of test 18 asserted
`((((v+v)+v)+v)+v)/5 != v` on a table of "irregular-looking" voltages and went RED — for *most*
doubles that identity HOLDS exactly, so the equal-command shortcut would have been unfalsifiable
and M18 a silent no-op. The table is now six values SEARCHED FOR (2 M uniform draws in ±12 V) as
ones where the identity fails, and the test asserts that at least five of the six still do.

---

### Step 2 — the description, the calibrated geometry, the table's second source

**`include/shulib/hal/drive_geometry.hpp`** — `fromInchesPerRadian(k)` (constexpr, `{k, 1.0}`),
plus a new header section "TWO CONSTRUCTORS, TWO PROVENANCES" and a rewritten `wheelRadius` field
comment that says in words that a CALIBRATED geometry's `wheelRadius` carries the PRODUCT and a
panel must print "ipr", never "radius" (brief T5).

**`include/shulib/hal/drivetrain_description.hpp`** — NEW. `kMaxSmartPorts`, `DrivetrainKind`,
`MotorCartridge`, `DescriptionSource`, `cartridgeWord()`, `DriveSideDescription`,
`DrivetrainDescription` with `valid(why, cap)`, `leftGeometry()`, `rightGeometry()`, `sideOf()`,
`signedPortOf()`, `describe()`, `portMagnitude()`. Fixed-width, no pointers, no `std::string`,
allocation-free, PROS-free, sim-free.

**`src/chassis_table.hpp`** — `describeMissingForDescription()` and `tableToDescription()`;
`kMaxPort` now CITES `hal::kMaxSmartPorts`.

**RULING R3 — `kMaxSmartPorts` has ONE home; the table cites it, the monitor is pinned to it.**
The brief left this to the executor. `hal::kMaxSmartPorts` is the library's home.
`src/chassis_table.hpp`'s `kMaxPort` became `static_cast<int>(hal::kMaxSmartPorts)` — the table
already includes `hal/drive_geometry.hpp`, so the citation costs nothing and the compiler now
enforces the agreement. `teleop::kMaxSideMembers` deliberately keeps its own constant:
`hal::MotorGroup` already includes `teleop/coupled_side_monitor.hpp`, so a citation the other way
would close a **directory-level include cycle** (hal → teleop → hal) and would make a pure fight
detector depend on a drivetrain description it has no business knowing about. *Alternative
considered:* a `static_assert` inside `hal/drivetrain_description.hpp` — rejected for the same
reason (it needs the include). The equality is instead pinned by two `static_assert`s at the top
of `test/drivetrain_description_test.cpp`, which is the same trade landmine 6 offers for the
median. Consistency check run: nothing already written depends on `kMaxPort` being a distinct
constant (it is only ever used as an array bound and a range check, both with the value 21), and
nothing downstream is constrained by the choice.

**RULING R4 — `valid()`, `describe()` and the accessors are MEMBERS, not free functions.**
Brief §4.1's sketch comment writes them as free functions taking the description first
(`valid(desc, why, cap)`); brief §5 test 12 writes
`result().leftGeometry().inchesPerRadian()` — a member call. The two cannot both be satisfied by
one spelling. Members win, because the test is the executable specification and because the struct
stays an aggregate either way (designated initializers still work; `DriveGeometry` already has
member functions). *Alternative:* provide both — rejected, two spellings of one operation is the
thing this chunk exists to avoid.

**RULING R5 — `valid()` also refuses an unknown KIND byte and an unknown CARTRIDGE byte.**
Brief §5 test 1's list names "kind ≠ Tank" but not the cartridge. The cartridge check was added
because the enum deliberately has no `Unset` member (per §4.1), so a default-constructed or
corrupted description carries the byte 0 — and Part 2 will read this struct back from an SD card,
where a flipped byte in a one-byte field is precisely the corruption the CRC is being added for.
Without the check, a description nobody filled in would look filled in. *Alternative:* add an
`Unset = 0` enumerator — rejected: it makes "no cartridge stated" a representable, seemingly-legal
value, which is HA-111's defect class again.

**RULING R6 — an UNSIGNED table produces an INVALID description, and a new
`describeMissingForDescription()` names why.** T9 says the table is the second source and that
"the table's UNSET fields still refuse". The dangerous case it does not name is the bench bot's
table: `signsMeasured = false`, every entry positive, geometry hypothetically filled — a naive
mapping would yield a description passing every check in `valid()` while half of robot two's
motors fought their side-mates on the first command. `tableToDescription()` therefore copies ports
ONLY when `signsMeasured`, leaving both sides empty otherwise, and the new companion
`describeMissingForDescription()` adds "measured signs" to `describeMissingForLibrary()`'s list so
the cause is named rather than inferred. *Alternatives:* (a) a `SHULIB_PRECONDITION` inside
`tableToDescription` — rejected: a throw at boot where the tree's convention is "paint what is
missing and refuse"; (b) extending `describeMissingForLibrary` itself — rejected: `src/main.cpp`
already checks "signed" as its own gate and would print the reason twice.

**Mutations run — rebuilt, run, failure OBSERVED, restored:**

| # | Mutation | Observed |
|---|---|---|
| M1a | `valid()`: the BOTH-sides duplicate check deleted | **RED** `drivetrain_description_test.cpp:97 ERROR: CHECK_FALSE( d.valid(why, sizeof why) ) is NOT correct!` + `:98 CHECK( std::strstr(why, "BOTH sides") != nullptr )` |
| M1b | `valid()`: the per-side `inchesPerRadian > 0 / finite` check deleted | **RED** `:162 CHECK_FALSE( l.valid(...) )`, `:163 CHECK( strstr(why, "LEFT inches per radian") )`, and the RIGHT pair — 4 assertions per bad value across `{0, −1.375, NaN, ∞}` |
| M1c | `valid()`: the cartridge check deleted | **RED** in TWO files — `drivetrain_description_test.cpp:196/197` and `chassis_table_test.cpp:462/463 CHECK_FALSE( c.valid(why, sizeof why) )` (the UNSET-cartridge table would have described a drivetrain) |
| M2a | `tableToDescription`: `absPort()` applied on the way across (the sign dropped) | **RED** `chassis_table_test.cpp:374 CHECK( d.left.signedPorts[i] == expectedLeft[i] )`, `:377 CHECK( d.signedPortOf(11) == -11 )` |
| M2b | `tableToDescription`: the wheel DIAMETER used where the radius belongs | **RED** `chassis_table_test.cpp:389-394`, incl. `CHECK( d.left.inchesPerRadian.value() != 2.75 * 0.6 )` — the deliberate "if anyone swapped it" line fired |
| M3 | `fromInchesPerRadian` multiplies by 2 | **RED TWICE.** First at COMPILE time: `test/drivetrain_description_test.cpp:327:52: error: static assertion failed — static_assert(kCompileTime.wheelRadius.value() == 1.375);`. Then, with that one line temporarily relaxed to a `CHECK` so the runtime half could be seen too: 30+ assertions red across three test cases, incl. `drivetrain_description_test.cpp:296 CHECK( g.inchesPerRadian().value() == k )` and `chassis_table_test.cpp:392` |

Suite after step 2: **1216 cases / 1,567,485 assertions, green.** Doc coverage gate reports
**1807 public entities across 126 headers, all documented** — the new header included.

---

### Step 3 — the calibrator

**`include/shulib/hal/motor_group.hpp`** — `hal::medianInPlace(double*, std::size_t)` extracted as
a free function; `MotorGroup::medianOf` now delegates to it, body moved verbatim (landmine 6's
STRONG option: **shared**, not merely pinned equal). The calibrator calls the same function for
every median it takes, so a slow member cancels exactly between calibration and run time rather
than approximately.

**`include/shulib/calibration/drivetrain_calibrator.hpp`** — NEW directory, NEW file.
`Phase`, `Answer`, `HumanFacts`, `CalibratorConfig`, `PortObservation`, `CalibrationReport`,
`CalibratorStatus`, `DrivetrainCalibrator`, plus `kDegreesToRadians`, `kReproScaleTolerance`,
`kReproWidthTolerance`. All six phases of brief §4.2, every refusal naming its cause in a bounded
`char reason_[160]`, allocation-free, PROS-free, sim-free, no faults raised.

**Rulings beyond the brief, each with the alternative and the reason:**

**R7 — the reproducibility tolerances are library CONSTANTS, not config fields.** Brief §4.4 names
`reproScale` (2 %) and `reproWidth` (3 %); §4.3's config list does not contain them, and the
calibrator never reads them — they are a claim ABOUT it. They are therefore
`calibration::kReproScaleTolerance` / `kReproWidthTolerance`, documented as the claim, cited by the
test. *Alternative:* add them to `CalibratorConfig` — rejected: a config field nothing reads is a
knob that silently does nothing, which is worse than a magic number because it looks live.

**R8 — one config field beyond §4.3: `restTimeout` (3 s).** The rest windows between pulses and
around the spin wait for every candidate to fall below `restFloor`. §4.3 bounds every powered
wait but gives that unpowered wait no bound, and a member that never settles (a frozen encoder
holding its last velocity — which is exactly what `MemberFault::Frozen` does, and what test 10
found) would hang the procedure for ever with a person standing over the robot. *Alternative:*
reuse `spinTimeout` (20 s) — rejected: twenty seconds of waiting for a robot to stop is not the
same decision as twenty seconds of spinning, and one number serving two unrelated waits is how a
threshold ends up wrong for both. Covered by the HA-136 register entry with the others.

**R9 — `commandTimeout` is enforced across the WHOLE of a powered phase, including its rest
windows.** §4.2 attaches it to "any powered phase"; the rest windows inside `SideAssign` and
`SpinTest` command nothing. Enforced there anyway: a caller that has stopped ticking is a stalled
caller either way, everything is already at 0 V so refusing costs nothing, and the alternative
(enforce only while volts are non-zero) means the guard is off during exactly the window in which
a stalled caller is most likely to be noticed late.

**R10 — 0 V + `BrakeMode::Coast` everywhere, including the brief's "brake, rest" between the two
spin directions.** §4.2's SpinTest line says "brake, rest". Landmine 7 says Coast on every
refusal/abort. Using `BrakeMode::Brake` between the spins would leave a moment in the middle of a
powered procedure where the robot resists being pushed clear, for no gain: the plant's (and a real
drivetrain's) coast-down from 3 V is short. So `safeState()` — 0 V + Coast — is the ONLY stopping
action in the class, which also makes "every refusal leaves Coast" a property of one line rather
than of a dozen call sites.

**R11 — an out-of-order `answer()` REFUSES the run.** §4.2 does not say. A touch that silently did
nothing would leave the operator believing a step had been taken (and, on a resistive screen, a
stray touch and a deliberate one look alike afterwards). The refusal names both the answer and
what the run was waiting for. *Alternative:* ignore it — rejected for the reason above.
`Answer::Abort` is accepted in every phase; `abort()` is accepted from every phase INCLUDING
`Done`, which is the literal reading of brief test 17 and is documented as discarding an uncopied
result.

**R12 — the unpowered phases command the safe state ON ENTRY, not every tick.** Re-commanding
0 V to 21 ports at 100 Hz is device traffic bought for nothing, and — the reason that decided it —
`PushCapture` and `ScalePush` are phases in which something ELSE (a person) is moving the robot;
a calibrator writing 0 V every tick there is asserting control it does not have. The powered
phases DO re-command every tick (rule 7).

**R13 — the idle-candidate rest check during a pulse covers only NON-DRIVE candidates.** §4.2
says "any candidate outside the pulsed one moving above `restFloor`". Taken literally that refuses
every run on a coupled train: the pulsed member's four side-mates are BACK-DRIVEN through the gear
train and MUST turn — that is the whole reason the brief's own T3 says the plant's coupled-train
model is what lets the host prove this. The check is therefore on candidates that did not move on
the push, which is also exactly what brief test 8 injects ("a non-drive fake set moving during a
pulse").

**R14 — `finishDone()` validates its own output.** Whatever the procedure concluded, the
description is run through `DrivetrainDescription::valid()` before `phase_` becomes `Done`, and a
failure refuses with the validity message. A last line of defence, and free.

**R15 — the rest refusals NAME the port that is preventing rest.** Found by test 10: a
plant-frozen member holds its last VELOCITY as well as its last position, so it reports the speed
it had when it died, for ever — the drivetrain never comes to rest, and the original message
("the drivetrain never came to rest within 3.0 s") left a person to guess which of twelve motors
was at fault. Now: `port 13 is still turning at 7.2 rad/s after 3.0 s of waiting for rest …`.

**Calibrator mutations — every one applied, rebuilt, run, failure OBSERVED, restored.** Each row's
message is pasted from the run. (The build was verified to RELINK before every result was
trusted — see the stale-binary trap recorded under step 1.)

| # | Mutation | Observed |
|---|---|---|
| M4 | `movedThreshold` → 0 (everything counts as a drive port) | **RED** `:433 FATAL ERROR: REQUIRE( end == Phase::Done )` (the two idle motors were captured and then contradicted themselves) + test 4b's `CHECK( reasonMentions(rig.cal, "the push moved 0 port(s)") )` |
| M5 | the BACK-FIRST inversion deleted | **RED** ×8 `:516 CHECK( back.cal.report().ports[i].sign == -front.cal.report().ports[i].sign )` |
| M6 | the yaw-sign → side rule swapped | **RED** `:538 FATAL ERROR: REQUIRE( runCalibration(rig, RunOptions{}) == Phase::Done )` and again in test 12 — every side flips, so the spin commands both sides the same way and the run collapses |
| M7 | the own-encoder cross-check dropped, with a wrong sign injected | **RED** `:583 CHECK( reasonMentions(rig.cal, "contradict") )` plus `:592/:593/:595` — the contradiction IS still caught, but a whole phase later by the fight detector and only AFTER a wrong scale was taken. Test 7 was SHARPENED mid-mutation to assert that the refusal happens at the pulse (`leftInchesPerRadian == 0`), which is the property the check actually buys |
| M8 | the idle-candidate rest check during a pulse dropped | **RED** `:619 CHECK( reasonMentions(rig.cal, "not a drive motor") )` |
| M9 | `pulseYawFloor` → 0 | **RED** ×5 incl. `:640 CHECK( reasonMentions(rig.cal, "FLOOR") )` and the both-sides bracket `:661` |
| M10 | the MEAN where the median belongs | **RED** `:702 CHECK( rep.leftInchesPerRadian == Approx(cleanRep.leftInchesPerRadian).epsilon(1e-9) )` — one frozen member of five dragged the scale |
| M11 | `straightBound` → ∞ | **RED** `:732 CHECK( end == Phase::Aborted )`, `:733/:734` |
| M12a | `W = (dL − dR)/Δθ` — the formula's sign | **RED** `:750 FATAL ERROR: REQUIRE( … == Phase::Done )` (a negative width, refused by the calibrator's own W ≤ 0 guard) and again in test 13 |
| M12b | `W = (dR + dL)/Δθ` — the sides summed | **RED** `:750 FATAL ERROR: REQUIRE( … == Phase::Done )` |
| M12c | the second spin direction dropped (`widthCw := widthCcw`) | **RED** `:1065 CHECK( widthCwIn > widthCcwIn )`, `:1067 CHECK( end == Phase::Aborted )`, `:1068` — under asymmetric scrub the disagreement becomes invisible |
| M14 | the coupled-side monitor detached from the spin | **RED** ×6 incl. `:874 CHECK( end == Phase::Aborted )` and `:880 CHECK_THROWS_AS( (void)rig.cal.result(), PreconditionError ) did NOT throw at all!` — the fight was calibrated straight into the description |
| M15a | the IMU-ready bound removed | **RED** `:896 CHECK( rig.cal.phase() == Phase::Aborted )` |
| M15b | the spin bound removed | **RED** `:943 CHECK( rig.cal.phase() == Phase::Aborted )`, `:944 CHECK( safeStateHolds(rig) )` |
| M16 | the missed-tick check removed | **RED** `:972 CHECK( end == Phase::Aborted )`, `:973/:974` |
| M17 | the safe state skipped in `abort()` | **GREEN ON THE FIRST PASS — a real hole in the test, closed and re-run RED.** See below |
| M17b | the safe state skipped on a REFUSAL | **RED** `:878 CHECK( safeStateHolds(rig) )` (test 14, mid-spin, ±3 V on ten motors) and `:1082 CHECK( safeStateHolds(fight) )` |

**The green hole, and what was wrong with the test.** M17 — deleting `safeState()` from `abort()` —
left brief test 17 GREEN. The mutation was not weak; the test was. At the instant each phase
begins, the calibrator has usually just commanded the safe state itself for its own reasons
(`enter()`/`enterStep()` do, and `finishDone()` does), so aborting at the first tick of a phase
finds every motor already at 0 V + Coast and an `abort()` that did nothing at all still leaves the
motors safe. The test was asserting a state, not an ACTION. Fixed by putting every motor into the
worst state `abort()` could find it in — 7 V and `BrakeMode::Hold` — immediately before each abort,
in all six phases plus `Idle` and the `Answer::Abort` route; and by adding a case that lets a
REFUSAL fire while the calibrator itself is driving (the fight detector, mid-spin), with an
assertion that it really was commanding ±3 V at the time. Re-run: **RED, 8 assertions** —
`:1043 CHECK( safeStateHolds(rig) )` six times (one per phase), `:1054` (Idle), `:1064`
(`Answer::Abort`). This is the one mutation of the chunk that found a defect in the TEST rather
than confirming one in the code, and it is the reason the rule "a mutation not actually run does
not count" is worth the time it costs.

**Two more findings the tests produced, recorded because they are facts about the design:**

1. **A BACK-FIRST sign capture makes the robot's "forward" the other direction, so the TAPE push
   must go the other way too.** Test 5 first failed with "the LEFT side's median travel over the
   push is −29.5 rad — the push must roll BOTH sides forward". That is correct behaviour, not a
   bug: once the operator has said which end is the front, "forward" means that end, and the
   panel's instruction for the tape push ("PUSH THE ROBOT STRAIGHT along the tape") means forward.
   The test now pushes the other way after answering BACK FIRST, and says why.
2. **A plant-`Frozen` member holds its last VELOCITY as well as its last position**, so it reports
   the speed it had when it died, for ever. The drivetrain therefore never reaches rest before the
   spin and the REST check fires before the fight detector gets its chance. That is the safer of
   the two orders (nothing is powered while the rest check fires), and it is why ruling R15 exists:
   the rest refusal now names the port and its velocity instead of leaving a person to guess which
   of twelve motors is at fault.

**Reproducibility, measured (brief §4.4, test 13).** Four complete runs under composed encoder +
IMU hostility at seeds 1–4:

```
reproducibility over 4 hostile seeds: scale spread 0 %, width spread 0.010989 % (bounds 2 / 3 %)
effective width under 5 % turn scrub: 13.1572 in (true 12.5)
figure-eight worst error under the CALIBRATED geometry: 1.06667e-12 in
```

The scale spread is exactly 0 % and that is the honest reading, not a test that forgot to vary
anything: encoder quantization is a deterministic tick GRID and draws no random numbers, so four
seeds give four identical scales. What the seed does move is the IMU's per-boot rate-bias draw,
which is why the WIDTH spread is non-zero. The test asserts both — and separately that the hostile
scale is NOT the pristine one (the grid moved it off the plant's exact radius), so "0 % spread"
cannot be produced by hostility that was never switched on.

Suite after step 3: **1233 cases / 1,568,972 assertions, green.**

---

### Step 4 — verification, and one finding the gates produced

**A DEFECT THE HOST BUILD COULD NOT SEE, caught by the src build gate.** Both new headers used a
variadic-template funnel to format their messages (`fail(why, cap, format, …)` in the description,
`refuse(format, …)` in the calibrator). The host test build's flag set does not include
`-Wformat-nonliteral`, so it was clean; **`tools/src_build_gate.py`'s policy does**, and the gate
failed all five builds with 12–18 disallowed warnings at
`include/shulib/hal/drivetrain_description.hpp:290 — format not a string literal, argument types
not checked`. That warning is not pedantry here: a funnel taking `const char* format` switches OFF
the compiler's printf argument checking for EVERY message in the class, which is exactly where a
wrong specifier costs a person the one sentence they get when the robot refuses.

Fixed properly rather than suppressed: `valid()`'s eleven checks each write their own reason with a
LITERAL format, and the calibrator's `refuse()` now takes an already-formatted string while each of
its nineteen call sites formats with a literal into a local buffer. The funnel's load-bearing
property — every refusal goes through ONE line that commands 0 V + Coast — is unchanged.

That change then exposed a second thing the compiler could now see: with literal formats GCC could
compute truncation, and `-Wformat-truncation` failed three messages that did not fit 160 bytes. The
reason buffer is now `kReasonCap = 256`, and the compiler is checking that no refusal is cut off.

Because the refusal plumbing changed AFTER the first mutation pass, **every calibrator mutation was
re-run against the final code** (M4–M17b, 19 runs) and every one was observed red again; the
messages above are from those re-runs. The snprintf calls were then reflowed to the tree's ~100-column
style and the 15 redundant `msg`-scoping blocks the mechanical rewrite left were removed, after
which M6 and M12a (the coordinator's two named spot-checks) were run a THIRD time and observed red.

**Gate results, all run on the final tree:**

| Gate | Command | Result |
|---|---|---|
| Host suite | `cmake --build build/test -j$(nproc) && ./build/test/shulib_tests` | **1233 cases / 1,568,972 assertions, 0 failed, 3 skipped — green** |
| src BUILD gate, self-test | `python3 tools/src_build_gate.py self-test` | `OK (28 detector cases, each against a real build)` |
| src BUILD gate | `python3 tools/src_build_gate.py check` | `PASS (all 5 builds link…)` — bench, xdrive, tank, tank-drive, tank-library, **0 disallowed warnings** |
| ARM header gate | the RESUMING recipe | `TU includes 160 headers` → `ARM compile gate: CLEAN` |
| CI guard 1 (PROS-free) | the exact `ci.yml` grep | `library is PROS-free (all of include/shulib except hal/pros/)` |
| CI guard 2 (layering) | the exact `ci.yml` grep | `layering holds: core is sim-free` |
| Doc coverage | `api_doc_tool.py check-coverage` | `1890 public entities across 127 headers, all documented` |
| Doc freshness | `api_doc_tool.py check-fresh` | clean after `generate` |
| Doc examples | `api_doc_tool.py check-examples` | `387 quoted lines, all verbatim (4 source files)` |
| C7 removability | `api_doc_tool.py check-removability` + the brief's grep | `no public doc references docs/internal/`; the grep returns nothing |
| Briefing | `briefing_status.py generate && check` | `generated block current` |
| Staleness audit | `doc_staleness_audit.py self-test` + run | `OK (10 detector/filter cases)`; `clean (26 live docs)` |
| A4 reconciliation, direction 1 | `grep -rn "PROVISIONAL (A4" include/ test/ \| grep -v "HA-[0-9]"` | empty |
| A4 reconciliation, direction 2 | per new ID | HA-134 → the calibrator; HA-135 → the calibrator + the description; HA-136 → the calibrator; HA-137 → the plant. All four found where the register claims them (the tags were ADDED when this check showed three of the four missing) |

**Trap for the next session, recorded because it cost time twice here.** `cmake --build build/test`
runs `shulib_doc_gates` BEFORE compiling anything. The briefing gate goes stale the moment the
suite's case count changes, so after adding or mutating a test the build STOPS at the gate and the
previous binary is still sitting there — and re-running it looks like a green result. Two mutation
runs read GREEN this way before the cause was found. Every mutation in this log was therefore run
through a wrapper that regenerates the briefing and then requires `Linking CXX executable
shulib_tests` in the build output before any test result is believed.

---

### Step 5 — the documentation pass, and the prose review (RESUMING rule 4)

**Touched, exactly the five the brief names:**

1. **`docs/api/`** — regenerated (`python3 tools/api_doc_tool.py generate`). New page
   `docs/api/drivetrain_description.md`; `drive_geometry.md`, `motor_group.md`, `drive_plant.md`
   and the new `drivetrain_calibrator.md` updated; `README.md`, `all-entities.md` and `mkdocs.yml`
   re-emitted. Coverage 1890 entities across 127 headers, all documented.
2. **`docs/hardware-assumptions.md`** — four new entries under a new sub-heading in the Robot 2
   section, **HA-134** (the T3 yaw-sign belief, reasoned — with the honest note that the host plant
   CANNOT be an independent check of it, because the plant turns the robot using the same frozen F5
   contract the calibrator reads the sign against), **HA-135** (the T8 effective-vs-taped width,
   reasoned; direction reasoned, ratio unmeasured), **HA-136** (ONE entry naming all sixteen
   `CalibratorConfig` numbers plus the two reproducibility tolerances, invented), **HA-137** (the T6
   coupled-train command model as a SIM belief, settled by R6's back-fit, with the admission that
   the mean model is OPTIMISTIC because a real motor back-drives four gearboxes). Four index rows
   added. HA-124/125/126's *Settle* lines now name **R3d Part 3** and say what Part 1 already did
   and did not do. HA-131 **re-homed to R4** with the sentence that says why (a detector is not a
   description, and it needs real pushes). HA-130's *Source* and *Settle* grew the calibrator (the
   shared median makes port 18 cancel exactly; the report's spread reads it in one run). The status
   note's total moved 133 → 137 and the section intro's counts are restated.
3. **`docs/roadmap.md`** — the "You are here" R3d paragraph gains the landing sentence: what Part 1
   is, what it was proven against, and — in the same breath — that it has never moved a physical
   drivetrain, nothing consumes a description yet, and R3b is still open.
4. **`docs/internal/build-order.md`** — the R3d entry's status line now opens **"PART 1 LANDED
   (host-only); Parts 2–3 next"** and carries the register's four new entries and HA-131's move; the
   FIRST bold `Next:` block now points at Part 2's brief (owed, awaiting the team lead's go) and its
   closing sentence says what is in the tree and what still is not. The block's history paragraph
   and the 2026-09-14 "Previously" block are left exactly as they were.
5. **`docs/internal/PROJECT-BRIEFING.md`** — regenerated by the tool; gate green.

**Deliberately NOT touched, each with the reason, each re-read to confirm it is still true:**

- **`docs/guide/14-what-it-cannot-do-yet.md`** — read end to end looking for a sentence Part 1
  falsifies; there is none. Its claims are about the bench bot's one 2026-08-13 session, the port
  map being a labelled guess, and every physical constant being a labelled guess — all still true,
  and Part 1 ADDED four labelled guesses rather than settling any. The calibration chapter is Part
  3's, written from a procedure somebody has actually followed.
- **`README.md`** — no user-facing behaviour exists yet: nothing reads a description, no program
  runs the calibrator. Its one "calibration" hit is the IMU's boot calibration in the hostility
  list, untouched by this chunk.
- **`docs/shulib-v2-master-plan.md`** — the capability catalog's "online wheel-radius calibration"
  row stays in the not-yet column until it has RUN (Part 3). Checked its other two hits:
  §Robots (2)'s "R3d's calibration must measure the drive-encoder distance scale per side (a push
  along a tape)" is still a correct forward statement, and the `filters/` EMA/median rows are about
  a different median entirely.
- **The bench worksheet** — Station G is Part 3's, and writing a station for a procedure nobody has
  followed is how a worksheet comes to describe a robot that does not exist.
- **`docs/faq.md`** — greped for calibration, track width, wheel diameter, port map, chassis table:
  no hits. No question exists yet.
- **`hal/motor_group.hpp:73`'s "until R3d" comment and `src/main.cpp:461/:889`** — the dead-port
  promise is **Part 2's** (an ABSENT member, the boot check). Still open, so the comments are still
  accurate. Left exactly as they are.
- **`docs/internal/ORIENTATION.md:115`** ("Step 1 — measure the robot. Wheel diameter, gear ratio,
  track width…") — checked because Part 1 is the thing that will eventually falsify it. It is still
  true TODAY: no program reads a description, so a human still measures and types. It becomes stale
  at Part 2, and that is where it should be fixed.
- **`docs/guide/06-how-things-fail.md:189`** ("the calibration awaits hardware") — checked; the word
  there means threshold tuning, not drivetrain calibration. Still true.
- **`docs/vexbuilder-integration.md:137`, `docs/legacy-command-vocabulary.md:106`** — both use
  "calibration" for the `corrections{}` slice and M3's legacy routines. Neither is about R3d.

**Prose review, per changed source file** (grep `docs/` and `README.md` for the file's name and its
concepts — `DriveGeometry`, `fromDiameter`, `MOTOR WATCH`, `chassis table`, `track width`,
`external ratio`, `MemberFault`, `attachCoupledMembers`, `SignFlipped`, `MotorGroup`, `median`,
`calibrat` — read every hit outside `docs/api/` and `docs/internal/chunks|transcripts`):

| File | Checked | Changed |
|---|---|---|
| `include/shulib/sim/drive_plant.hpp` | 5 hits in `hardware-assumptions.md` (HA-13/14/45/46/48 — the plant's own provisional constants, none of them about the command path), 1 in `build-order.md` (A2's entry, describing the pipeline at the level of "voltage → wheel velocity", still accurate). `MemberFault` has exactly one prose hit (build-order's R3b entry, listing the two faults the group's tests use — it names Frozen and SignFlipped, both unchanged, and does not claim the list is exhaustive). `attachCoupledMembers` and `SignFlipped` have NO prose hits outside the chunk logs. | **Added HA-137** for the new command model; added the `A4 register HA-137` tag in the header. Nothing found stale. |
| `include/shulib/hal/drive_geometry.hpp` | `DriveGeometry`: 5 hits in `hardware-assumptions.md` (HA-124/125/126 and the R3b geometry note), 3 in `build-order.md`, 2 in `roadmap.md`. All describe it as "one object per side, both consumers", which the new constructor does not change. `fromDiameter` has NO prose hits. | HA-124/126's *Settle* lines rewritten to say the calibration measures the PRODUCT and cannot separate the factors — which is what `fromInchesPerRadian` exists for. |
| `include/shulib/hal/drivetrain_description.hpp` (new) | Nothing could reference it before today. Checked `track width` (9 hits in the register, 8 in build-order, 3 in roadmap, 1 in the master plan, 1 in ORIENTATION) for anything that now conflicts with "effective ≠ taped". | **Added HA-135** and cited it from the `trackWidth` field comment; HA-125's *Settle* line now warns that the spin's number is the EFFECTIVE width and is not the tape's. |
| `include/shulib/hal/motor_group.hpp` | `MotorGroup`: 3 hits in roadmap, 3 in build-order, 2 in the register (HA-130/133). `median`: 3 in the master plan (the `filters/` EMA/median rows — a different median), 3 in roadmap, 3 in the register. Every one describes the median's BEHAVIOUR, which the extraction preserved verbatim (and the whole suite pins bit-identically). | **HA-130's Source and Settle** grew the calibrator: the shared median is now the reason port 18 cancels exactly, and the report's spread reads it in one run instead of a live panel. |
| `include/shulib/calibration/drivetrain_calibrator.hpp` (new) | `calibrat*`: 46 hits in the register, 18 in build-order, 15 in roadmap, 8 in the master plan, 4 in `guide/06`, 1 each in README, vexbuilder-integration, legacy-command-vocabulary. Every one read. | **Added HA-134/135/136**; roadmap and build-order updated as above. The four non-R3d senses of the word (IMU boot calibration, threshold tuning, the `corrections{}` slice, M3's legacy routines) are recorded above as checked and correct. |
| `src/chassis_table.hpp` | `chassis table` / `chassis_table`: 6 hits in the register, 2 in roadmap, 2+1 in build-order. All describe it as the ONE typed input with signs living in it, which `tableToDescription()` preserves rather than replaces. `external ratio`: 3 hits, all HA-126. `MOTOR WATCH`: 3 in build-order, 2 in roadmap, 1 in the register — all about the tester's capture, which is untouched. | Nothing stale. The table is now a SOURCE of the description, and the register's HA-124/125/126 settle lines say so. |
| `test/drive_plant_coupled_test.cpp`, `test/drivetrain_description_test.cpp`, `test/drivetrain_calibrator_test.cpp`, `test/chassis_table_test.cpp` | Test files are referenced from the register's *Source* fields only. | `test/motor_group_test.cpp`'s citation in HA-130/133 is unchanged and still accurate; the new files are cited from HA-134/135/136/137. |

---

### Step 6 — two late notes, the DoD, and the close

**RULING R16 — the provenance string is a HUMAN FACT, not a config field.** §4.2 says the Done
phase's description carries "a provenance line the caller supplies (the program's build stamp)"
but does not say through which door. It is `HumanFacts::provenance`, beside the kind, the cartridge
and the IMU port: those four are exactly the things a PERSON contributes to a run, and a provenance
line is a statement about who ran it and when. The calibrator copies it into the description's fixed
96-byte buffer at `Done` (truncated to 40 characters and followed by the measured essentials —
port count, tape distance, width — so a banner carries both halves) and keeps no pointer to it past
`start()`. *Alternative:* a `CalibratorConfig` field — rejected: that struct is the sixteen
THRESHOLDS, every one of them a register entry, and a string among them would be the odd one out.

**Parallel work in the tree, seen and NOT touched.** At the close, `git status` showed three
untracked files this executor did not create:
`docs/internal/chunks/R3d-part2-persistence-boot-programs.md`,
`docs/internal/chunks/SESSION-2026-09-18-autonomous.md` and
`docs/internal/chunks/T2-input-shaping-drive-modes.md` — the coordinator's own work in an
autonomous overnight session, written at ~05:35–06:10 on 2026-09-18. None of them was modified.
Two consequences were acted on:

1. The `**Next:**` block in `build-order.md` had been written to say Part 2's brief was "owed and
   waiting to be written". That is FALSE — the brief exists. The block now links it. A status
   pointer that is wrong about the very next step is the failure mode RESUMING rule 5 exists for,
   and it would have been introduced by this chunk.
2. Part 2's brief §1 states Part 1's API from the working tree as it stood that morning and says
   the coordinator will re-check it. It AGREES with what shipped, name for name: the description's
   fields and helpers, `kMaxSmartPorts = 21`, the calibrator's constructor,
   `start(HumanFacts{kind, cartridge, imuPort, provenance})`, `tick`/`answer`/`abort`/`phase`/
   `status`/`reason`/`result`/`report`, the full `Answer` set, `tableToDescription()`, and
   `medianInPlace` shared with the calibrator. **Two things for the coordinator to resolve when
   amending it:** (a) §2.3 says the record serialises the description's fields "in declaration
   order" and then lists `kind, cartridge, imuPort, source, left, right, trackWidth, provenance` —
   the actual declaration order is `kind, cartridge, imuPort, left, right, trackWidth, source,
   provenance`, with `source` seventh, not fourth. Either wording is a fine record layout; they are
   not the same order, and a byte-exact golden will be written from whichever is picked. (b) §1
   does not mention `bench::describeMissingForDescription()`, which Part 1 added beside
   `tableToDescription()` and which Part 2's composition root will want: it is what names "measured
   signs" as missing when an UNSIGNED table cannot describe a drivetrain (ruling R6).

T2's brief says it runs in an ISOLATED WORKTREE touching `include/shulib/teleop/` only. Nothing in
this chunk modified anything under `teleop/`. Note for the integration: this chunk added a
`static_assert` in `test/drivetrain_description_test.cpp` pinning `teleop::kMaxSideMembers ==
hal::kMaxSmartPorts` (ruling R3). If T2 changes that constant the assert fires at compile time,
which is the pin doing its job.

---

## Definition of Done — Part 1, with evidence per line

- [x] **`hal/drivetrain_description.hpp` with `valid()`, `describe()`, the geometry accessors;
      every public member documented; ARM gate green with the new headers.**
      `include/shulib/hal/drivetrain_description.hpp` (336 lines) — `kMaxSmartPorts`,
      `DrivetrainKind`, `MotorCartridge`, `cartridgeWord()`, `DescriptionSource`,
      `DriveSideDescription`, `DrivetrainDescription` with `valid(why, cap)`, `leftGeometry()`,
      `rightGeometry()`, `sideOf()`, `signedPortOf()`, `describe()`, `portMagnitude()`.
      Tests: `test/drivetrain_description_test.cpp` (3 cases). Doc coverage gate: 1890 entities
      across 127 headers, ALL documented. ARM: `TU includes 160 headers` → CLEAN.
- [x] **`hal/drive_geometry.hpp` gains `fromInchesPerRadian`; the header says what `wheelRadius`
      then carries.** The new "TWO CONSTRUCTORS, TWO PROVENANCES" section and the rewritten
      `wheelRadius` field comment both say it in words; `DriveGeometry 3` pins exactness with `==`
      (including a `static_assert`, which is how mutation M3 was first observed red — at COMPILE
      time).
- [x] **`calibration/drivetrain_calibrator.hpp`: all six phases as §4.2, every refusal naming its
      cause, allocation-free, PROS-free, sim-free.** 1217 lines. Phases `WaitImu`, `PushCapture`,
      `SideAssign`, `ScalePush`, `SpinTest`, `Done` (+ `Idle`/`Aborted`). Nineteen refusal sites,
      each formatted with a literal and each naming its cause — and the ones that CAN name a port
      do (tests 7, 8, 9, 14 and both rest refusals assert the port number is in the message). Every
      array sized by `hal::kMaxSmartPorts`; no `new`, no container, no `std::string`. CI guard 1
      (PROS-free) and guard 2 (sim-free) both run and both clean, with the new directory in scope
      because both guards are globs over `include/shulib`.
- [x] **`sim/drive_plant.hpp`: the two T6 extensions; the whole pre-existing suite bit-identical
      (recorded before and after).** 1208 cases / 1,563,156 assertions before the edit; 1208 /
      1,563,156 after it, before any new test file was added. Recorded under step 1.
- [x] **`src/chassis_table.hpp`: `tableToDescription()`; `test/chassis_table_test.cpp` extended;
      the src build gate's five builds green.** Plus `describeMissingForDescription()` (ruling R6)
      and `kMaxPort` now citing `hal::kMaxSmartPorts` (ruling R3). Two new cases in
      `chassis_table_test.cpp`. `src_build_gate.py self-test` → `OK (28 detector cases)`;
      `check` → `PASS (all 5 builds link)`, **0 disallowed warnings** — which is a stronger
      statement than it was this morning, because the gate CAUGHT a real defect here first.
- [x] **Tests 1–20 green; every mutation RUN, observed red with the message logged, restored.**
      Brief §5's twenty rows are covered by 23 test cases across four files (the numbering is kept;
      4b, 1b, 18 and 19 are additions). **27 distinct mutations**, every one applied to the real
      tree, rebuilt, run and OBSERVED — 26 red on the first attempt, **one GREEN** (M17), which was
      a hole in the TEST; the test was fixed and the mutation re-run red. Every calibrator mutation
      was then RE-RUN against the final code after the format-string refactor, and M6/M12a a third
      time after the reflow.
- [x] **Host suite green; PROS-free guard, layering guard, ARM header gate, the doc gates green.**
      The table under step 4. Final suite: **1233 cases / 1,568,972 assertions, 0 failed, 3
      skipped.**
- [x] **`R3d-PROGRESS.md` complete: the order of work, every ruling beyond §3 with its alternative,
      the prose-review log, the clean-tree suite count.** This file. Sixteen rulings (R1–R16), the
      prose review as a per-file table with what was checked and what changed, and the clean-tree
      count at the top.
- [x] **Nothing here claims a hardware result.** The calibrator header opens with a section headed
      "PROVEN ON THE PLANT; NOT YET RUN ON A ROBOT"; the description header says no description has
      ever been produced on a robot; the calibrator test file's first line says so; all four new
      register entries state what the host CANNOT settle (HA-134 says explicitly that the plant
      cannot be an independent check of it); the roadmap and build-order sentences both end on it.

**Nothing was committed.** No `git add`, no `git commit`, no `git push`, no worktree, no branch
change. The tree is dirty with exactly the files this chunk touched, plus the three untracked
coordinator files listed above that were not touched.

**Suite counts, end to end:** clean tree `76a5819` — 1208 cases / 1,563,156 assertions ·
after the `sim/` extensions alone — 1208 / 1,563,156 (bit-identical) · final — **1233 cases /
1,568,972 assertions, green, 3 skipped.**

*Log closed 2026-09-18 by the R3d Part 1 executor (opus).*

---
---

# Part 2 — persistence, the registry, boot verification, and the programs

| | |
|---|---|
| **Chunk** | R3d — drivetrain self-calibration |
| **Part** | Part 2 (persistence + registry + boot verification + the two programs) |
| **Date** | 2026-09-18 |
| **Tree at start** | `17aa4eb` (`feat(calibration): R3d Part 1 -- the drivetrain description and the self-calibrator, host-only`), clean |
| **Executor model** | opus (Claude Opus 5, 1M context) |
| **Commit policy** | **The executor commits NOTHING.** No `git add`, no `git commit`, no `git push`, no worktree, no branch change. The coordinator verifies independently and commits. |
| **Parallel work** | Chunk T2 runs in a SEPARATE worktree on `include/shulib/teleop/`. Nothing here modifies that directory. |

> Appended in real time. Rulings beyond the brief carry the alternative considered and the reason.
> Every mutation carries the **observed** failure message, not a claim that it would fail.

---

## Order of work

Following the brief's §2 order with landmine 4's constraint (the `MotorGroup` absent member LANDS
LAST among the library items, because it changes a load-bearing class the whole suite leans on):

1. **This log section**; the clean-tree suite count re-measured at `17aa4eb` before touching anything.
2. **`calibration/calibration_record.hpp`** — the on-card format, the CRC, the byte-exact golden,
   the decode refusals (§4 tests 1–3).
3. **`hal/block_source.hpp`** + `hal/fake/fake_block_source.hpp` + `hal/pros/block_source.hpp` +
   the shim's `fopen` path + the adapter test through a real temp dir (§4 test 4).
4. **`hal/device_registry.hpp`** + the fake + `hal/pros/device_registry.hpp` + the shim's
   `pros/apix.h` and `shim::registryState()` + the adapter test (§4 test 5).
5. **`calibration/boot_check.hpp`** — `verifyAgainstRegistry` + `chooseBootSource` (§4 tests 6–7).
6. **The programs.** `Makefile` + `tools/src_build_gate.py` FIRST (so the sixth build exists and
   fails honestly until the program does), then `src/calibrate_program.{hpp,cpp}`, then the
   `src/main.cpp` `tanklib` refactor onto the description. `src_build_gate check` after each.
7. **`hal::MotorGroup` absent member** LAST (§4 tests 8–9), with the plant pin re-run immediately.
8. All verification; the documentation contract (§6); the prose review per changed file; the DoD
   with evidence; the final suite count.

---

## Log

### Step 0 — required reading + clean-tree baseline

**Clean-tree baseline, re-measured BEFORE touching anything** (tree `17aa4eb`, clean):

```
[doctest] test cases:    1233 |    1233 passed | 0 failed | 3 skipped
[doctest] assertions: 1568972 | 1568972 passed | 0 failed |
[doctest] Status: SUCCESS!
```

Matches the number Part 1 closed on, so the tree is the one the brief was written against.

**Read in full, in the brief's order:** `RESUMING.md` ("Standards that are not negotiable", "Pace
and documentation rules"); the brief `R3d-part2-persistence-boot-programs.md` §0–§8; the parent
spec's §3 T1/T9/T10 and §10; Part 1's `hal/drivetrain_description.hpp` and
`calibration/drivetrain_calibrator.hpp` (the public surface, lines 95–480), `src/chassis_table.hpp`,
and Part 1's whole PROGRESS section (rulings R1–R16, the stale-binary trap note at step 1);
`hal/block_sink.hpp`, `hal/pros/block_sink.hpp`, `hal/fake/fake_block_sink.hpp`,
`test/pros_block_sink_adapter_test.cpp`; `diag/blackbox_format.hpp` (whole — the `ByteWriter`/
`ByteReader` pattern and the "golden, not a round trip" reasoning); `src/bench_r3a.cpp` (the
zero-index note at :112–122 and the census at :573–607, plus the touch grid/`hitTest` at
:1851–1955); `include/pros/apix.h` :620–662 and `include/pros/device.h` (the `v5_device_e_t`
values); `test/pros_shim/pros/` (`shim_control.hpp`, `misc.hpp`, `rtos.hpp`);
`hal/motor_group.hpp` (whole); `teleop/drivetrain_degradation.hpp` (whole); `src/main.cpp`
(whole); `src/drive_program.cpp` (whole) + `.hpp`; `Makefile` :1–140; `tools/src_build_gate.py`
(whole); `R3a-BENCH-WORKSHEET.md` Station A and Station F; `.github/workflows/ci.yml` guards;
`tools/api_doc_tool.py`'s header; `test/CMakeLists.txt`; `docs/hardware-assumptions.md`
(HA-120/122, the Robot 2 section HA-124–137, and `grep registry`).

**What the `registry` grep found, so the register entry is a citation and not a duplicate**
(brief §6): three hits, all in HA-120 and HA-122 — HA-120 records that the R1a expander sighting
came from *index 21* and calls for "a CORRECTED 0–20 registry scan"; HA-122 calls the /usd/ path
split "the registry-indexing shape R1a met". Neither states the zero-index mapping ITSELF as a
belief with a settle step, and nothing in the register mentions `registry_get_plugged_type`. So
the mapping gets its own NAMED entry (below) that CITES HA-120 rather than repeating it.

---

### Step 1 — the record format (`calibration/calibration_record.hpp`), §4 tests 1–3

**What landed:** `include/shulib/calibration/calibration_record.hpp` — `kRecordMagic` "SHDC",
`kRecordVersion = 1`, `kRecordFileName = "shulib_drivetrain.bin"` (ruling P6's one spelling),
`kProvenanceBytes`, `kRecordPayloadBytes = 168`, `kRecordPrefixBytes = 8`, `kRecordBytes = 180`
(both pinned by `static_assert`), `crc32Ieee()`, the LAYOUT TABLE as a comment block, and
`encodeRecord` / `decodeRecord`. `test/calibration_record_test.cpp` (5 cases).

The five refusals, in this order and no other: **magic → version → payload length → total length
→ CRC → `valid()`**. The ordering is a ruling, not an accident (below).

**RULING P-R1 — the golden array's bound is DEDUCED (`const std::uint8_t kGolden[]`), not written.**
The first draft wrote `std::array<std::uint8_t, 180>`, which SILENTLY ZERO-FILLS a short
initializer list. A miscounted NUL run in the provenance field (79 pad bytes, typed as 63) produced
a legal 180-byte array whose content was shifted sixteen bytes and whose tail was zeros — the test
failed at "byte 160", which reads like an encoder bug and is really a typo in the *expectation*.
With the bound deduced, `static_assert(sizeof kGolden == 180)` turns that whole class into a
COMPILE error. *Alternative considered:* generating the array from the Python deriver into the test
file — rejected: a generated expectation is one edit away from being regenerated from the encoder,
which is exactly the independence the golden exists for.

**RULING P-R2 — the golden's expected bytes come from a SECOND IMPLEMENTATION, written from the
layout table, and the file says so.** The brief says "hand-derived, not captured from the encoder".
180 bytes of hand-typed hex is a transcription exercise with its own error rate (see P-R1), so the
values were derived by `struct.pack` + `zlib.crc32` — an implementation of the *table*, not of the
*encoder*, and one this repository has never seen. Every group of bytes in the array carries the
field name it spells, so the array is checkable by eye against the table; the f64 values were chosen
to have hand-checkable bit patterns (1.375 = `0x3FF6…`, 1.5 = `0x3FF8…`, 13.25 = `0x402A8…`). The
property that matters is preserved and was **measured**: under mutation M-P1 the round-trip test
stayed GREEN and the golden went RED.

**RULING P-R3 — the refusal ORDER is LENGTH before CRC, and `valid()` last.** A CRC computed over
the wrong number of bytes says "corrupt" about a file that is really "from another layout", and the
operator's next move differs (recalibrate on this build vs. rebuild and recalibrate). `valid()` last
so that an intact record spelling a contradictory description is reported as *"intact (CRC ok) but
the description it spells is self-contradictory: …"* — which separates a producer bug from a card
problem at 8 a.m. on a bench. *Alternative:* CRC first (cheapest total refusal) — rejected: cheapness
is not the scarce resource here, the operator's next move is.

**RULING P-R4 — `encodeRecord` does NOT check `valid()`; `decodeRecord` does.** One gate, at the
point where bytes become a belief. A caller may legitimately want to persist whatever the calibrator
produced; a caller must never boot from a description nobody checked. The calibrate program checks
`valid()` itself before writing, because a record nobody can boot from is not worth writing — and
that is a program policy, not a format rule.

**RULING P-R5 — the provenance length is computed BOUNDED, not with `std::strlen`.** A description
built by hand (or decoded from a record whose provenance filled all 96 bytes) need not be
NUL-terminated, and `strlen` would walk off the struct. The field is fixed width; so is the read of it.

**A trap met for real, and the recipe that closes it** (Part 1 recorded the first half; this is the
second): `cmake --build build/test` runs the doc gates BEFORE the link, and **any suite-count change
makes the briefing gate fail**, which blocks the relink and re-runs the OLD binary — mutation M-P1
first reported GREEN for exactly that reason. `--target shulib_tests/fast` skips the gates, but it
ALSO skips CMake's `depend` step, so a HEADER edited since the last depend run is invisible to make
and the build silently does nothing — mutation M-P5 first reported GREEN for *that* reason, with no
"Linking" line to give it away. Both are lies told by a stale executable. The recipe used from here
on, and worth writing into RESUMING one day:

```sh
make -C build/test -f CMakeFiles/shulib_tests.dir/build.make CMakeFiles/shulib_tests.dir/depend
cmake --build build/test --target shulib_tests/fast -j"$(nproc)"   # REQUIRE the Linking line
```

(`depend` must run AFTER a new TU has been compiled once, because the depfile the compiler emits is
what `depend` converts. A new test file therefore needs: configure → build → depend → build.)

**Mutations run — every one rebuilt with the recipe above, the Linking line seen, run, failure
OBSERVED, restored:**

| # | Mutation | Observed |
|---|---|---|
| M-P1 | `source` and `trackWidth` swapped in BOTH `encodeRecord` and `decodeRecord` | **RED, and exactly where it should be:** `calibration_record_test.cpp:160 CHECK( static_cast<std::uint8_t>(out[i]) == kGolden[i] ) … values: CHECK( 2 == 0 ) logged: byte 71 of the record` (and bytes 76–79, 176–179), `:165 CHECK( crc32Ieee(...) == 0x45C46E98U ) … values: CHECK( 2173237730 == 1170501272 )`, `:183 CHECK( back.trackWidth.value() == 13.25 )`. **`CalibrationRecord 3` (the round trip through both seams) stayed GREEN — 1 passed / 0 failed — which is the E1 lesson demonstrated rather than asserted.** |
| M-P2 | the CRC check dropped (`if (false && stored != want)`) | **RED**, 8 assertions in the flipped-bit sweep, e.g. `:272 CHECK( decodeRecord(bytes, decoded, why, sizeof why) == false ) … values: CHECK( true == false )` and `:274 … did not name the CRC: record is intact (CRC ok) but the description it spells is self-contradictory: LEFT lists port 12 twice` (the sign-byte flip that `valid()` happens to catch anyway — the other seven are caught by nothing) |
| M-P3 | the record LENGTH check dropped | **RED**, 7 assertions: `:238 CHECK_FALSE( decodeRecord(...) ) … values: CHECK_FALSE( true )` for the 179-byte case, `:247 … find("181")` for the padded one, and the two truncation subcases of test 3. (It also makes `in.subspan(176, 4)` read past the end — so the length check is load-bearing for memory safety, not only for the message.) |
| M-P4 | the final `valid()` check dropped | **RED**: `:286 CHECK_FALSE( decodeRecord(...) ) … CHECK_FALSE( true )`, `:287 find("CRC ok")`, `:288 find("LEFT side has no ports")` |

**A finding about the TEST, not the code.** `CHECK_MESSAGE(..., s.what, ...)` with a `const char*`
struct member printed the POINTER (`one flipped bit in 0x562cd8730244 was NOT refused`). A failure
message nobody can read is not a message; the loop now wraps it in `std::string`.

---

### Step 2 — the block SOURCE seam (§4 test 4)

**What landed:** `include/shulib/hal/block_source.hpp` (`IBlockSource::readAll(std::span<std::byte>)
noexcept → std::size_t`, `isOpen()`), `include/shulib/hal/fake/fake_block_source.hpp`
(`setBytes`, `setShortRead`, `setOpen`, `readCalls`), `include/shulib/hal/pros/block_source.hpp`
(`ProsBlockSource(fileName, mountRoot = "/usd/")`, fopen `"rb"`, the bare-name and
mount-root-ends-in-'/' preconditions, owns the `FILE*`, non-copyable/non-movable), and
`test/pros_block_source_adapter_test.cpp` (8 cases, through the shim + a REAL temp directory).

**RULING P-R6 — the seam is WHOLE-FILE, not a stream, and the header says why.** Everything it
carries is a small fixed-width self-checking record read once at boot. open/read/seek/tell/eof would
be five more ways for an adapter to be subtly wrong about a file that fits in a stack buffer, each
needing its own host test against a real filesystem. *Alternative:* a streaming sibling now —
rejected, deferred with a named trigger in the header (the day something wants to replay a blackbox
on the brain).

**RULING P-R7 — `readAll` returns a COUNT, and a SHORT read is not an error.** A card yanked
mid-write leaves a prefix; the seam hands back what there was and the FORMAT refuses it by name
("truncated"). A bool would have thrown away the one word the operator needs. The count is
`[[nodiscard]]` for the same reason `IBlockSink::write` is: a caller that ignores it cannot tell a
full record from an empty file and would decode whatever its buffer happened to hold.

**RULING P-R8 — `readAll` SEEKS TO 0 first.** The seam's contract says "from the start"; without the
rewind a second call returns 0, and a boot path that retried would see an empty file and report "no
calibration" on a card that has one. Mutation M-P6 pins it.

**RULING P-R9 — three outcomes are kept apart: NO CARD, NO FILE, EMPTY FILE.** All three read 0
bytes; `isOpen()` separates the third from the first two, and the composition root prints which. They
send an operator to three different places ("put a card in", "run shulib Calibrate", "the file is
zero bytes — the write failed"), and a seam that collapsed them would make the boot message useless.

**Mutations run (recipe above; Linking line seen each time):**

| # | Mutation | Observed |
|---|---|---|
| M-P5 | the `::pros::usd::is_installed()` probe dropped (`if (true)`) | **RED** (after the stale-binary correction above): `pros_block_source_adapter_test.cpp:166 CHECK_FALSE( source.isOpen() ) … CHECK_FALSE( true )`, `:168 CHECK( source.readAll(into) == 0U ) … values: CHECK( 4 == 0 )`, plus four bytes copied into a buffer that should have stayed untouched |
| M-P6 | the `std::fseek(file_, 0, SEEK_SET)` rewind dropped | **RED**: `:86 CHECK( source.readAll(again) == payload.size() ) … values: CHECK( 0 == 10 )`, `:87 CHECK( std::memcmp(...) == 0 ) … values: CHECK( -10 == 0 )` |

**An honest limit, stated rather than tested into:** the host CANNOT distinguish `"rb"` from `"r"`
— POSIX does not translate — so no host test can fail if the mode is wrong, and the mode matters on
a V5 whose newlib might. It is a BELIEF, and it goes into the register as one (below) rather than
being claimed by a test that could not see it.

---

### Step 3 — the device REGISTRY seam (§4 test 5)

**What landed:** `include/shulib/hal/device_registry.hpp` (`DeviceKind`, `deviceKindWord`,
`IDeviceRegistry::pluggedKind(int port)`, `isSmartPort`),
`include/shulib/hal/fake/fake_device_registry.hpp` (`setKind`, `setKinds`, `clear`, `queries`),
`include/shulib/hal/pros/device_registry.hpp` (`ProsDeviceRegistry`, `kindOf`),
`test/pros_shim/pros/apix.h` (NEW — `v5_device_e_t` transcribed verbatim from vendored
`pros/device.h:48-63`, a ZERO-INDEXED `registry_get_plugged_type`, `shim::registryState()` with
`queries`/`lastIndex`, and `shim::setPluggedAtPort()` which does the physical→index arithmetic ON
THE TEST SIDE), `resetRegistry()` wired into `shim::resetAll()`, and
`test/pros_device_registry_adapter_test.cpp` (5 cases).

**RULING P-R10 — the seam speaks PHYSICAL ports; the `port − 1` lives on one line of the adapter.**
Landmine 3 and `src/bench_r3a.cpp:112-122`. Everything else in PROS (and every human) says 1..21.
The interface therefore says 1..21, and `hal/pros/device_registry.hpp`'s `pluggedKind()` carries the
single marked conversion line. *Alternative considered:* an index-taking seam with the conversion at
each call site — rejected, that is literally the 2026-08-18 defect generalised.

**RULING P-R11 — the SHIM is indexed like PROS, and the test helper does the arithmetic.**
`shim::registryState().kinds[i]` is what index `i` answers; `shim::setPluggedAtPort(p, …)` writes
slot `p − 1`. If the shim had been indexed by physical port, the shim and the adapter would have
SHARED the convention and the test could not see an off-by-one at all — the shared-model
cancellation `shim_control.hpp` warns about, in the one place it would have mattered most. The shim
also records `lastIndex`, so the conversion is asserted **directly** (`pluggedKind(1)` ⇒
`lastIndex == 0`; `pluggedKind(21)` ⇒ `lastIndex == 20`) and not only through its consequences.

**RULING P-R12 — out-of-range answers `None` WITHOUT calling PROS, and the test asserts the call
never happened.** `registry_get_plugged_type` takes a `uint8_t`: a caller's 22 would pass straight
through and read index 22 — the very read that invented the phantom ADI expander (HA-120) — and a
negative would convert to a large index. Asserting only "the answer was None" would pass with the
screen missing, because index 22 is empty in the shim; asserting `queries` did not move is what
makes the screen the thing under test.

**RULING P-R13 — `DeviceKind` is OURS; an unknown PROS device kind becomes `Other`, never `Motor`.**
A re-export of `v5_device_e_t` would put a PROS type in the library's vocabulary (the PROS-free
rule). And the direction of the default matters: "shulib Calibrate" builds its candidate list from
the ports that answer `Motor`, so a device kind PROS adds next season landing in `Motor` would be
POWERED as a drive motor. `E_DEVICE_AIVISION` maps to `Vision` (it is a camera); `E_DEVICE_SERIAL`
and `E_DEVICE_UNDEFINED` to `Other`. The test walks the whole enum and two undefined values.

**Mutations run (recipe above; Linking line seen each time):**

| # | Mutation | Observed |
|---|---|---|
| M-P7 | `static_cast<std::uint8_t>(port)` instead of `port − 1` | **RED**, 20+ assertions across two cases, e.g. `pros_device_registry_adapter_test.cpp:48 CHECK( registry.pluggedKind(1) == DeviceKind::Motor ) … values: CHECK( 0 == 1 )`, `:49 CHECK( ...lastIndex == 0 ) … values: CHECK( 1 == 0 )`, `:52 CHECK( ...lastIndex == 20 ) … values: CHECK( 21 == 20 )`, and every row of the asymmetric census |
| M-P8 | the `isSmartPort` out-of-range screen dropped | **RED**: `:115 CHECK( pros::shim::registryState().queries == before ) … values: CHECK( 5 == 0 )`, `:116 CHECK( ...lastIndex == -1 ) … values: CHECK( 23 == -1 )` — PROS was asked about index 23 |
| M-P9 | the `kindOf` fallback returns `Motor` instead of `Other` | **RED**: `:152 CHECK( kindOf(v5_device_e_t(33)) == DeviceKind::Other ) … values: CHECK( 1 == 10 )` and the same for value 1 |

---

### Step 4 — the BOOT CHECK (`calibration/boot_check.hpp`), §4 tests 6 and 7

**What landed:** `include/shulib/calibration/boot_check.hpp` — `PortClass{Present, Missing,
Swapped}` + `portClassWord`, `BootDecision{Ok, Degraded, Refuse}` + `bootDecisionWord`,
`BootPortStatus`, `BootVerdict` (rows, counts, `newPorts`, the IMU's own row, the
`teleop::DegradationVerdict` it was taken from, a bounded `reason`, `usable()`, `isMissing(port)`),
`verifyAgainstRegistry()`, and `BootSource`/`BootSourceChoice`/`chooseBootSource()`.
`test/boot_check_test.cpp` (11 cases).

**RULING P-R14 — the refusal PRECEDENCE is Swapped → IMU → policy, and the test pins the order.**
All three can be true at once, and only one sentence fits on a refusal screen. A rewired drive port
is the most serious finding (it invalidates the whole description, not one member), the IMU is next
(every number was measured against it), the dead-port policy last (it is the only one the team lead
has already ruled is survivable). *Alternative:* concatenating all three — rejected: a 240-char
screen line that says three things says none of them, and the operator's first move differs per
cause.

**RULING P-R15 — a SWAPPED port is NOT counted as ANSWERING for the degradation policy.** There is
no motor on that port to drive, so the census must not pretend there is. Consequence, asserted in
the test rather than left implicit: on a one-swap robot the shared policy on its own would have said
DRIVE_DEGRADED ("carry on without it"), and the swap check OVERRIDES that. "A motor is missing" and
"this robot was rewired" are different findings with different right answers, and the more serious
one wins.

**RULING P-R16 — an INVALID description REFUSES here rather than tripping a precondition.** Every
caller checks `valid()` first, so this path should be unreachable; it runs at BOOT on a robot, where
a `SHULIB_PRECONDITION` throw is a dead robot and a refusal is a screen that says what to do. The
test asserts the registry was never read (`queries() == 0`) on that path, so the refusal really is
first.

**RULING P-R17 — `reason` is 320 bytes and ASCII-only.** 320 because the Ok line carries the honest
limit as well as the counts, and the host build compiles this header under `-Wformat-truncation`,
which turned a 240-byte buffer into a COMPILE ERROR — the right place to find that out rather than
on a brain's screen. ASCII-only because the V5's screen font and the USB serial log both render a
UTF-8 em dash as mojibake; every runtime string in these headers now uses `--`, as `src/` already
does. (`static_assert` messages keep their em dashes: those are compiler output, not a V5 screen.)

**RULING P-R18 — `chooseBootSource` is `constexpr`, and the test `static_assert`s four rows.**
A boot decision that can be evaluated at compile time cannot have reached for a clock, a global or
an adapter. It also made mutations M-P12 and M-P13 fail at COMPILE time, naming the row — a stronger
red than a runtime one, and one nobody can mistake for a flaky test.

**The pinned limit (brief §4 test 6's last row), and why it is written the way it is.** `BootCheck
6h` asserts that two drive ports EXCHANGED still read `Ok`, and that the Ok reason SAYS SO
("NOT CHECKED: two drive cables swapped WITH EACH OTHER look identical here -- the first drive
catches that"). The same case covers a RE-GEARING that keeps the ports. The test's comment is
addressed to whoever later tries to "tighten" the check: this is the boundary of what a port census
can know, and `teleop::CoupledSideMonitor` is what closes it, at the first drive, within 250 ms.

**Mutations run (depend-then-fast recipe; Linking line seen; restored):**

| # | Mutation | Observed |
|---|---|---|
| M-P10 | the `swappedCount > 0` refusal dropped | **RED**, 8 assertions across all three subcases of `BootCheck 6d`, e.g. `boot_check_test.cpp:189 CHECK( v.decision == BootDecision::Refuse ) … values: CHECK( 1 == 2 )` (Degraded instead of Refuse), `:194 find("14(ROTATION)")`, `:195 find("rewired")` |
| M-P11 | a Swapped port classed as Missing instead | **RED**: `:190 CHECK( swappedCount == 1 ) … CHECK( 0 == 1 )`, `:191 CHECK( missingCount == 0 ) … CHECK( 1 == 0 )`, `:192 CHECK( v.ports[3].state == PortClass::Swapped ) … CHECK( 1 == 2 )`, and the decision itself |
| M-P12 | a REFUSED record falls back to the typed table | **RED AT COMPILE TIME**: `boot_check_test.cpp:400:19: error: static assertion failed — static_assert(chooseBootSource(true, BootDecision::Refuse, true).source == BootSource::Refuse);` |
| M-P13 | the record/table preference swapped (`recordDecoded && !tableValid`) | **RED AT COMPILE TIME** on the same `static_assert`; re-run with the four `static_assert`s temporarily relaxed so the RUNTIME half could be seen too — **RED**, `:382 CHECK( choice.source == r.want ) … values: CHECK( 1 == 0 ) logged: record=true verdict=0 table=true` and the same for `verdict=1`, with `why was: no calibration record on the card -- falling back to the TYPED TABLE…` |

---

### Step 5 — the build axis and the gate's SIXTH build (landed BEFORE the program, on purpose)

`Makefile`: `PROGRAM=calibrate` (tank only, `$(error)` on any other ROBOT and on any other
PROGRAM value), with the comment block rewritten for four programs in four slots.
`tools/src_build_gate.py`: `tank-calibrate` added to `VARIANTS`, `BUILD_ARGS`, `VARIANT_DEFINES`,
`VARIANT_BEACONS` and `XDRIVE_WIRING_DEAD_IN`, the header's four narrative passages updated (the
builds table, "WHAT check DOES", the beacon step's "the other FIVE", and the variant-differential
coupling note), and TWO new self-test cases: **14** (the `PROGRAM=calibrate` Makefile append
DROPPED ⇒ FAIL on the token-exact define AND on both halves of the beacon check) and **15** (the
real `tank-calibrate` build alone ⇒ PASS, which is also the NUL-terminated-match test for
`…=tank` inside `…=tank-calibrate`).

**This was landed before `src/calibrate_program.cpp` existed, and the gate was RUN in that state.**
It failed exactly as it should:

```
src build gate: FAIL
  [tank-calibrate] build tank-calibrate but the beacon 'shulib-robot-variant=tank-calibrate' is NOT
    in bin/hot.package.elf — src/main.cpp's #if chain did not take the tank-calibrate branch
  [tank-calibrate] build tank-calibrate but bin/hot.package.elf carries the tank beacon
    'shulib-robot-variant=tank' — this is a tank build wearing a tank-calibrate label
```

That is the brief's "so the sixth build exists and fails honestly until the program does",
observed rather than assumed — and it is the same failure the self-test's case 14 now plants
deliberately, so the detector that caught it is itself pinned.

---

### Step 6 — "shulib Calibrate" (`src/calibrate_program.{hpp,cpp}`) and the `main.cpp` chain

**What landed:** `src/calibrate_program.hpp` (one function, `shulib::bench::runCalibrate()`),
`src/calibrate_program.cpp` (~700 lines), and `src/main.cpp`'s preprocessor chain: the two-PROGRAM
guard generalised to a three-way count, `SHULIB_RUNS_CALIBRATE`, the beacon
`shulib-robot-variant=tank-calibrate`, `kCalibrateBuildStamp`, `SHULIB_STATE_SCREENS` widened, and
a branch in `initialize()`, `autonomous()` and `opcontrol()`.

The program: registry census → candidates with POSITIVE ports and the table's cartridge → the IMU
port DISCOVERED (ruling P1; the table's value printed as a cross-check, a mismatch a WARNING) →
`ProsImu` + `calibrate()` once → `DrivetrainCalibrator` → a 10 ms loop with a release-edge touch
row → on `Done`, encode → write → flush → CLOSE → re-open → read back → byte compare → `SAVED`.

**RULING P-R19 — a Part 1 API GAP was found and FIXED IN PART 1 (landmine 8 / RESUMING's "a chunk
that finds a flaw in an earlier chunk fixes it there").** `answer()` REFUSES an out-of-order answer
and ENDS THE RUN — which is right, and which makes a wrong BUTTON destructive. But `Phase` alone
cannot tell a panel which answers are live: `PushCapture` is two steps (watching, then awaiting
FRONT/BACK) and so is `ScalePush`. The only public signal was the PROSE of `status().prompt`, and
prose is not an API. So `DrivetrainCalibrator::accepts(Answer)` was added — a PURE QUERY over the
same private step `answer()` switches on, changing no logic and moving no state. *Alternatives
considered:* (a) a caller-side mirror of the step in the program (derivable, and correct — the
program sends the answers) — rejected because every future consumer would re-derive it and one
that got it wrong would abort a two-minute calibration on a mis-tap; (b) `strstr` on the prompt —
rejected, that is coupling a program to wording. Because `accepts()` DUPLICATES `answer()`'s
switch, the anti-drift device is a test rather than a comment: **`DrivetrainCalibrator 21`** drives
a FRESH rig to each of the four waiting steps and asserts, for EVERY `Answer`, that `accepts()`
agrees with what `answer()` actually does — it never asserts a hard-coded expectation, so a
divergence in either direction is red. 52,952 assertions.

**RULING P-R20 — Part 1's on-screen strings were converted to ASCII, in Part 1's files.** Found
while writing Station G against the program's actual strings: `status_.prompt` reads
`"WAIT — the IMU is calibrating…"` with a U+2014 EM DASH, and so do 23 other runtime strings in the
calibrator plus 2 in `drivetrain_description.hpp`'s `valid()`. LVGL's built-in fonts carry ASCII
and nothing else, so on the brain that renders as a missing-glyph box, and in a raw `cat` of the
user port as mojibake — in the ONE sentence a person gets when the robot stops. `src/` has used
`--` for exactly this reason since R3a (measured: ZERO em dashes inside string literals anywhere in
`src/`, against 58 uses of `--`). Part 1 wrote those strings before they had a screen to land on;
Part 2 built the screen and converted them. **Only string literals changed — comments keep their
typography, and no logic moved**; the suite is green on the same assertion count. A note in the
calibrator's header records the rule so it does not regress. This is a Part 1 file edit, declared
here under landmine 8.

**RULING P-R21 — the program IDLES on its result screen instead of returning.** `opcontrol()`
returning ends the program and the brain clears the screen — which would throw away the ONLY copy
of an UNSAVED result (no card, a card that refused). It therefore loops on `pros::delay(250)` until
the field disables it. *Alternative:* returning and letting the operator re-read the serial log —
rejected: the person running this is alone in a workshop with a brain, and the screen is what they
have.

**RULING P-R22 — a port the REGISTRY calls a motor whose ADAPTER refuses is DROPPED from the
candidates with a named warning, not a refusal.** `ProsMotor`'s constructor writes the gearset and
reads it back (HA-98), so it can refuse a port the census listed. A calibration of nine motors is
still a calibration; a calibration that refuses to start because one cable is loose is a session
lost. The port is NAMED on serial, because a silently missing motor is how a side comes up short.
Below two usable candidates the program refuses (the calibrator's own precondition).

**RULING P-R23 — the button row draws only the LIVE answers, and ABORT is always last.** Drawn from
`accepts()`, so a button that would end the run is never under a thumb; ABORT keeps the rightmost
slot in every phase so its position never moves. Touches are release-edge with the gap-belongs-left
hit test `bench_r3a.cpp:1935-1942` records (robot two's first recorded tap released one pixel into
a gap and did nothing).

---

### Step 7 — `hal::MotorGroup`'s ABSENT member (§4 tests 8–9), and the bit-identity pin

**Order note (a deviation from the prompt's step list, with the reason).** The prompt ordered the
programs (6) before `MotorGroup` (7); the brief's landmine 4 says "land it LAST among the LIBRARY
items", i.e. after §2 items 1–4 and before item 6. Both are satisfiable except for one thing:
`src/main.cpp`'s DEGRADED path CONSUMES the absent member (it hands the group a span with holes and
reads `presentCount()`), so the `tanklib` refactor cannot compile before it. The order actually run
was therefore: items 1–4 → the Makefile/gate → `calibrate_program` (which does not touch
`MotorGroup` at all) → **`MotorGroup`** → the `main.cpp` refactor. That keeps every gate honest at
every step and preserves what landmine 4 protects — the plant pin re-run immediately after the
change, with the suite counted before and after.

**What changed** (`include/shulib/hal/motor_group.hpp`): a `nullptr` slot is ABSENT.
`setVoltage`/`setBrakeMode` skip it; `position()`/`velocity()` take the median over the PRESENT
members; `current()` divides by `presentCount()`; `temperature()` and `brakeMode()` read the FIRST
PRESENT member; `evaluateDisagreement()` reports `present = false` with no reading taken; new
`presentCount()`, `absentMask()`, `isPresent(i)`; `member(i)` on an absent slot is a precondition
violation; the constructor requires at least ONE present member. The header's "every member is
PRESENT by contract … no dead-port tolerance (that is shulib Drive's job until R3d)" paragraph is
replaced by the new contract, with the `NullMotor`-rejected and decided-at-boot reasoning.

**THE BIT-IDENTITY PIN, measured:**

| | cases | assertions |
|---|---|---|
| before the `MotorGroup` edit | 1263 | 1,622,729 |
| after the edit, before any new test | 1263 | **1,622,738** |

**+9 assertions, and every one of them is accounted for.** They are in the ONE pre-existing case
that asserted the OLD contract — `MotorGroup: construction refuses an empty span and a null
member`, which required a `nullptr` member to THROW. That requirement is exactly what ruling P4
changes, so the case was rewritten (a null member now constructs and is reported absent; what still
refuses is an empty span, too many members, and an ALL-absent group): one `CHECK_THROWS_AS`
removed, ten `CHECK`s added, net +9. **No other test file's contribution changed by a single
assertion**, which is the honest form of "all-present groups are bit-identical" — and it is stated
that way rather than as a round number, because the round number would have been a lie.

**RULING P-R24 — `memberCount()` still reports SLOTS, and `presentCount()` is the new question.**
A panel walks a fixed array of ports and indexes it by slot; if `memberCount()` shrank when a
member died, every index after the hole would shift and the panel would name the wrong port. The
disagreement bit masks index slots for the same reason.

**RULING P-R25 — `current()` divides by PRESENT, not by SLOTS.** An absent member is not a member
drawing 0 A. Dividing by the slot count would make a side with a dead port read COOLER than it is,
which is the direction that hides a problem. Mutation M-P17 pins it.

**A MEASURED finding, recorded rather than asserted away (plant case 9).** With one of five members
absent, the A2 plant's coupled-train model drops that side to 80 % of its command (the plant takes
the MEAN of the members' effective volts and the library commands nothing to the hole). `MoveToPose`
then **TIMED OUT on all four seeds** inside its 8 s watchdog, having closed 21.20 in → 2.21,
37.09 → 5.86, 12.04 → 0.86 and 7.53 → 0.48. So a degraded drivetrain DRIVES, and drives SHORT.
The case asserts what this chunk is responsible for (it moves, toward the target; the hole is never
flagged as a fight; the reads are over the present members, pinned tick by tick against a group
built over exactly those four) and DOES NOT assert that it settles — that is a property of a
degraded drivetrain under provisional gains (HA-45/50/51/52), it is R5's to measure, and raising a
budget until a green tick appeared would have been inventing a hardware claim. The numbers are in
the test's comment and in a `MESSAGE` the suite prints.

**Mutations run (depend-then-fast recipe; Linking line seen; restored):**

| # | Mutation | Observed |
|---|---|---|
| M-P15 | the absent member counted in the median (as 0) | **RED**: `motor_group_test.cpp:467 CHECK( g.position().value() == 2.5 ) … values: CHECK( 2 == 2.5 )`, `:468 CHECK( g.velocity().value() == 25.0 ) … CHECK( 20 == 25 )`, and both halves of the "equals a group over exactly those four" comparison |
| M-P16 | the hole reported `present = true` to the monitor | **RED**, 5 assertions: `:515 CHECK( g.disagreeingMembers() == 0 ) … CHECK( 1 == 0 )` (the DEAD PORT flagged as a fighting one), `:516 CHECK( g.disagreeingMask() == 0u ) … CHECK( 4 == 0 )`, `:517 CHECK( g.lastVerdict().presentCount == 4 ) … CHECK( 5 == 4 )`, and the real fight then reported as TWO |
| M-P17 | `current()` divided by `members_.size()` instead of `present_` | **RED**: `:486 CHECK( g.current().value() == 10.0 / 4.0 ) … values: CHECK( 2 == 2.5 )` |

---

### Step 8 — "shulib Teleop" rebuilt on the DESCRIPTION

`src/main.cpp`'s whole `tanklib` namespace was rewritten. `TankRobot` now takes
`const hal::DrivetrainDescription&`; `TankMotors` builds from `signedPorts` AS STORED and puts a
`nullptr` where the boot check found a port MISSING; `TankKinematics{desc.trackWidth}`; TWO
`DriveGeometry`s from `leftGeometry()`/`rightGeometry()`; `ProsImu` on `desc.imuPort`;
`tankPortMapString()` renders the DESCRIPTION with its source word first; the panel's bottom rows
say where the numbers came from and which ports are dead.

The boot sequence in `initialize()`, in order and BEFORE any adapter exists: read the card →
`decodeRecord` → `verifyAgainstRegistry` → `chooseBootSource` → use the record, or the table with a
WARNING and its own registry verification, or refuse naming both.

**Landmine 1, checked by grep after the refactor.** `kChassis` appears in `main.cpp` exactly three
times, all inside the boot sequence and all legitimate: `tableToDescription(kChassis)` as the
second source, `describeMissingForDescription(kChassis, …)` for the refusal message, and
`tableToDescription(kChassis)` again for the CROSS-CHECK warning. **No field of the table is read
for a number anywhere in the graph** — no `leftCount`, no `wheelDiameterIn`, no `imuPort`,
no `tableTrackWidth`, no `tableDriveGeometry`.

**RULING P-R26 — the cross-check between the record and the table is a WARNING that counts
disagreeing ports.** When the record is in use, the table is still mapped and compared port by
port through `signedPortOf()`; a disagreement logs "the record wins (it moved THIS robot); the
table is stale and should be updated or deleted". *Alternative:* saying nothing — rejected: a
stale table is what the NEXT boot falls back to if the card is missing, so a person should know
it is wrong before that boot happens.

**RULING P-R27 — the stall check takes the LEFT geometry, and the boot log prints both.** The
config wants one `DriveGeometry` per wheel slot; the description carries two, because a
calibration measures each side separately. They agree on any symmetric robot. Taking the left one
and PRINTING both makes a disagreement visible rather than averaged away. *Alternative:* the mean
— rejected: a number nobody measured, standing where two measured ones disagree.

**Three `-Wformat-truncation` findings, all caught by the src build gate, all real.** The gate
treats it as a disallowed warning, and it was right three times: (1) `decodeRecord`'s valid()
refusal could overrun a caller's buffer with a 159-byte inner reason (bounded to `%.100s`);
(2) the port-map string could overrun the session header's 96-byte field on a 21-port-per-side
description (now assembled in a 200-byte local and copied in bounded, with the manual append loop
replacing a moving `snprintf` the compiler could not bound); (3) two boot-log lines (split in two,
and a `%.140s` reduced to `%.80s`). None was reachable on robot two's ten ports — and all three
are the kind of thing that becomes reachable the day somebody adds a port.

---

### Step 9 — verification

| Gate | Result |
|---|---|
| Host suite | **1267 cases / 1,624,433 assertions, 0 failed, 3 skipped** (from 1233 / 1,568,972 at `17aa4eb`) |
| `src_build_gate.py check` | **PASS — all 6 builds link** (`bench`, `xdrive`, `tank`, `tank-drive`, `tank-library`, `tank-calibrate`), 0 disallowed warnings, defines token-exact, beacons asserted |
| `src_build_gate.py self-test` | **OK (33 detector cases, each against a real build)** — was 28 before the two new cases |
| ARM header gate | `TU includes 168 headers` → **clean** at `-Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror -Os -mcpu=cortex-a9` |
| CI guard 1 (PROS-free) | empty — the only `pros/` includes under `include/shulib` are in `hal/pros/` |
| CI guard 2 (sim-free) | empty |
| Doc gates | self-test OK; coverage **1989 public entities across 133 headers, ALL documented**; freshness OK; examples 387 quoted lines verbatim; removability clean; briefing current; staleness audit clean |
| Removability grep | empty |

**The adapter FENCE test caught one of my own edits within the hour** (`fence: every adapter's
suppression set is EXACTLY -Wshadow + -Wsign-conversion`): `hal/pros/device_registry.hpp` had been
written with a speculative third `-Wconversion`, and the R1a guard refused it —
`device_registry.hpp: suppression set must be exactly two flags, found 3`. Removing it, the ARM
gate then failed on VENDORED code: `pros/apix.h` pulls `api.h`, which pulls `pros/llemu.hpp`, whose
`pros::lcd` namespace is `[[deprecated]]` unless liblvgl's own llemu is in the include path.

**RULING P-R28 — the fence is WIDENED by exactly one flag, in exactly one file, and the guard
enforces the exception.** `-Wdeprecated-declarations` is permitted only in an adapter that includes
`pros/apix.h`, which is only `device_registry.hpp`, because `registry_get_plugged_type` is declared
nowhere else. This does NOT reopen R1a's mutation M11: M11 was "-Wconversion added so OUR sloppy
adapter compiles" — a diagnostic about the adapter's own arithmetic — and a deprecation warning
about a vendored namespace cannot hide one. It is the same category the src build gate already
ignores BY PATH (`include/pros/`); the header gate has no path filter, so it is done by flag. Both
mandatory flags are still demanded, and `pros_adapter_fence_test.cpp` now fails any other third
flag and any `-Wdeprecated-declarations` in a file that does not include apix.h. *Alternatives
rejected, in the header:* `pros::c::get_plugged_type` from the narrow `pros/device.h` (a DIFFERENT
indexing convention, and the zero-indexed call is the one with the hardware history);
re-declaring the vendored function ourselves (a second source of truth that would drift SILENTLY);
`#define`-ing PROS's `_PROS_KERNEL_SUPPRESS_LLEMU_WARNING` from a library header (reaching into
another project's internals, and order-dependent on what a TU includes first).

---

### Step 10 — the documentation pass and the PROSE REVIEW (RESUMING rule 4)

**Touched, in the same pass as the code:**

| Document | What changed |
|---|---|
| `docs/api/` + `mkdocs.yml` | regenerated: **1989 public entities across 133 headers, ALL documented** (was 1890 / 127 at Part 1). Six new pages: `block_source`, `pros-block_source`, `device_registry`, `pros-device_registry`, `calibration_record`, `boot_check` |
| `docs/hardware-assumptions.md` | **HA-138** (the registry's zero-indexing, as a NAMED entry — see the grep note under step 0: HA-120 and HA-122 only ALLUDE to it, so this cites them rather than duplicating), **HA-139** (fopen `"rb"` under `/usd/`, flagged because the host CANNOT test the mode), **HA-140** (the CRC choice, as a design note under HA-122, saying why the blackbox deliberately has none). Index rows, the status line (7 of **140**), the narrative count, and the Robot-2 preamble (now seventeen entries, six reasoned). **HA-124/125/126 AMENDED**: they described a library program that reads the table's geometry and refuses while it is UNSET, which is no longer the whole truth — a record on the card supplies both and the table is the fallback |
| `docs/internal/chunks/R3a-BENCH-WORKSHEET.md` | **Station G — CALIBRATE, then TELEOP FROM THE RECORD**, in Station F's shape, written for the team lead ALONE: G.0 (card, clearance, tape, the two DO-NOTs), G.1 (the four upload commands with `rm -rf bin .d` before each `make`), G.2 (Bench Tests census + MOTOR WATCH FIRST, with "DISAGREES is expected after a rebuild"), G.3 (wheels-up, **expect the yaw-floor refusal — that is the program working**), G.4 (a screen-by-screen table: what it says, what you do), G.5 (the `MEASURED` line to copy, SAVED vs NOT SAVED, and a SECOND run for reproduction), G.6 (Teleop from the record, with Station F's F.2 checks and one new question), G.7 (`pros lsusb` + `stty` + `cat` + `strings`, with HA-129's renumbering), and "what to send back". **Every screen string in it was copied from the program's source, not paraphrased.** Station A's "three programs" list is now four |
| `docs/roadmap.md` | the "You are here" gains an R3d Part 2 paragraph (one paragraph: what landed, six builds, host-only, unrun, Part 3 is the morning); the Parts 1–3 paragraph's "proves five builds" corrected to "proved five builds (six since R3d Part 2…)" with a note that the graph is now built from a description |
| `docs/internal/build-order.md` | the R3d status line (`Status 2026-09-18: PARTS 1 AND 2 LANDED (host-only); Part 3 next`), the Part 2 sentence expanded into what actually landed, and the FIRST bold `**Next:**` block rewritten to **Part 3 = the bench session, the team lead's morning** — keeping the T2-in-parallel sentence and the coders-wait sentence verbatim |
| `README.md` | one bullet in "What it is NOT yet": a program exists in which the robot measures its own drivetrain, and **it has never been run** |
| `docs/internal/PROJECT-BRIEFING.md` | regenerated |
| `.github/workflows/ci.yml` | the arm-compile-gate comment: five builds → **six**, naming `PROGRAM=calibrate` |
| `docs/internal/RESUMING.md` | §3's build-gate paragraph and the retired-blocker note: five builds → six, and `PROGRAM=calibrate` named beside `drive` and `library` |
| `docs/faq.md` | **TOUCHED, against the brief's "NOT touched" list — see the deviation below** |

**DEVIATION FROM THE BRIEF, declared: `docs/faq.md` WAS touched.** §6 lists it as not-touched. The
prose-review grep for `registry` found this, in a published FAQ:

> **Why doesn't shulib use PROS's `registry_get_plugged_type()` to find devices?**
> … **No shulib adapter calls the registry.**

That is now FALSE — `hal::pros::ProsDeviceRegistry` is exactly that adapter, and two programs
depend on it. RESUMING's rule 4 ("read the hits, fix what is stale") and its standard "a chunk that
finds a flaw in an earlier chunk fixes it there" both point one way, and a published document that
contradicts the library is worse than a document the brief forgot to list. The answer is rewritten
to keep everything that is still true (the two conventions, the silent failure, the bench probe
that reported two healthy devices dead) and to say what the library now does instead: the seam is
stated in physical ports, the conversion is one line in one adapter, tested from both ends, and the
two things that cannot be written without it are named.

**NOT touched, with the reason:**

- **the guide (`docs/guide/`)** — the "Calibrating a robot" chapter is **Part 3's**, by the brief
  and by C8's rule that a guide chapter is written from a procedure somebody has FOLLOWED. Nothing
  here has been followed.
- **`docs/shulib-v2-master-plan.md`** — it moves when the thing has RUN, not when it compiles.
- **`docs/changelog.md`** — measured, not assumed: it carries no entry for R3b, R3d Part 1 or
  anything since the 2026-08-15 release (`grep '2026-09\|R3b\|R3d'` → empty). It is release-scoped,
  and adding a lone entry for an unrun program would be the only such entry in the file.
- **`include/shulib/teleop/`** — chunk T2's, in a separate worktree. `git status` confirms not one
  file under it is modified.
- **`docs/diagnostics-plan.md`**, **`docs/legacy-command-vocabulary.md`** — read via the grep; both
  hits are about `IBlockSink`/an unrelated "registry" of command ids, both still true.

**PROSE REVIEW, per changed source file** (grep `docs/` and `README.md` for the file's name and its
concepts; read every hit; fix what is stale). The greps run were the brief's twelve terms: `every
member present`, `dead-port tolerance`, `until R3d`, `chassis table`, `tableTrackWidth`,
`tableDriveGeometry`, `refuses at boot`, `shulib Teleop`, `three programs`, `five builds`,
`IBlockSink`, `block_sink`, `registry`.

| Changed source file | Checked | Changed |
|---|---|---|
| `include/shulib/hal/motor_group.hpp` | `every member present` (no public-doc hit — only the two internal briefs, which are historical records); `dead-port tolerance` (same); `until R3d` (`R3b-PROGRESS`, `R3b-parts123`, the R3d briefs — all historical); the header's own promise | the HEADER's own "no dead-port tolerance (that is shulib Drive's job until R3d)" paragraph — the sentence R3d was named in — replaced by the new contract. `docs/api/motor_group.md` regenerated |
| `include/shulib/hal/block_source.hpp`, `hal/pros/block_source.hpp`, `hal/fake/fake_block_source.hpp` | `IBlockSink`, `block_sink` — 9 hits (diagnostics-plan, changelog, briefing, roadmap ×3, R1a/R1b/DEFECTS1 logs) | none stale: every one describes the WRITE seam, which is unchanged. The read sibling is additive and appears in the regenerated reference |
| `include/shulib/hal/device_registry.hpp`, `hal/pros/device_registry.hpp`, `hal/fake/fake_device_registry.hpp` | `registry` — 7 hits: `docs/faq.md` (the whole answer), `docs/hardware-assumptions.md` HA-120 + HA-122, `docs/legacy-command-vocabulary.md` ×4 | **faq.md rewritten** (above). HA-120/HA-122 CITED by the new HA-138 rather than duplicated. legacy-command-vocabulary's "registry" is the command-id registry, unrelated |
| `include/shulib/calibration/calibration_record.hpp`, `boot_check.hpp` | `refuses at boot` (one public hit, the build-order Next block — rewritten anyway); `IBlockSink` (above) | the register's HA-122 family now carries HA-139 and HA-140 beside it |
| `include/shulib/calibration/drivetrain_calibrator.hpp` (Part 1's, amended) | the roadmap's and build-order's Part 1 paragraphs; the register's HA-136 | the roadmap's Part 1 paragraph kept (still true) with the Part 2 paragraph after it; a new header section records the ASCII rule |
| `include/shulib/hal/drivetrain_description.hpp` (Part 1's, amended) | as above | 2 runtime strings to ASCII; nothing in prose depended on their typography |
| `src/main.cpp` (`tanklib`) | `shulib Teleop` — 9 hits (roadmap, worksheet Station A + F, hardware-assumptions, session log, build-order); `chassis table` — 8 hits; `tableTrackWidth` / `tableDriveGeometry` — the register's HA-124, and internal logs | roadmap's Parts 1–3 paragraph amended; worksheet Station A's program list now four; **HA-124/125/126 amended** (they described the table as the library's only geometry source); Station F left as the historical record of the table-driven boot, with Station G carrying the record-driven one |
| `src/calibrate_program.{hpp,cpp}` (new) | `three programs` — 1 public-ish hit (the R3b brief, historical); `five builds` — RESUMING §3, `ci.yml`, roadmap | RESUMING and `ci.yml` corrected to six builds and `PROGRAM=calibrate`; roadmap's is past tense and now carries the six-build note |
| `Makefile`, `tools/src_build_gate.py` | `five builds` (above); the Makefile's own PROGRAM comment block | both rewritten for four programs / six builds |
| `test/pros_shim/pros/apix.h` (new), `shim_control.hpp` | the shim's honest-limit statement in `shim_control.hpp` | none: the new shim header carries its own copy of the belief list and points at `shim_control.hpp` for the full statement, as every other shim header does |

---

## Definition of Done — Part 2, with evidence per line

- [x] **Items 1–6 of §2 landed; every public member documented; ARM gate green with the new
      headers.**
      **1.** `include/shulib/hal/block_source.hpp` (`IBlockSource::readAll`, `isOpen`),
      `hal/fake/fake_block_source.hpp` (`setBytes`, `setShortRead`, `setOpen`, `readCalls`),
      `hal/pros/block_source.hpp` (`ProsBlockSource(fileName, mountRoot = "/usd/")`, fopen `"rb"`,
      owns the `FILE*`, non-copyable/non-movable, the bare-name and trailing-'/' preconditions);
      `test/pros_block_source_adapter_test.cpp` (8 cases, shim + a REAL temp directory: written by
      `ProsBlockSink` and read back byte-identical, no card ⇒ refusing, no file ⇒ refusing and
      DISTINCT from an empty file, a truncated file ⇒ a short COUNT).
      **2.** `hal/device_registry.hpp` (`DeviceKind`, `deviceKindWord`, `IDeviceRegistry`,
      `isSmartPort`), `hal/fake/fake_device_registry.hpp`, `hal/pros/device_registry.hpp`
      (`ProsDeviceRegistry` over `registry_get_plugged_type(port − 1)`, the conversion on ONE
      marked line with the R1a-shape trap cited); `test/pros_shim/pros/apix.h` NEW
      (`v5_device_e_t` transcribed from the vendored `pros/device.h`, a zero-indexed
      `registry_get_plugged_type`, `shim::registryState()` with `queries`/`lastIndex`, and
      `setPluggedAtPort()` doing the arithmetic on the TEST side); `resetRegistry()` in
      `shim::resetAll()`; `test/pros_device_registry_adapter_test.cpp` (5 cases: port 1 → index 0,
      port 21 → index 20, 0/22/negatives ⇒ None with PROS never called, every `v5_device_e_t`
      value mapped plus two undefined ones).
      **3.** `calibration/calibration_record.hpp` — "SHDC", version 1, `payloadBytes` 168,
      `kRecordBytes` 180, the description's fields fixed-width little-endian in DECLARATION order,
      a bitwise CRC-32/IEEE, `kRecordFileName`, `encodeRecord`/`decodeRecord`;
      `test/calibration_record_test.cpp` (5 cases).
      **4.** `calibration/boot_check.hpp` — `verifyAgainstRegistry` + `chooseBootSource`;
      `test/boot_check_test.cpp` (11 cases).
      **5.** `hal::MotorGroup`'s ABSENT member; `test/motor_group_test.cpp` cases 8 and 8b,
      `test/motor_group_plant_test.cpp` cases 9 and 9b.
      **6.** `src/calibrate_program.{hpp,cpp}`, `src/main.cpp`'s chain and the `tanklib` rewrite,
      `Makefile`'s `PROGRAM=calibrate`, `tools/src_build_gate.py`'s sixth build.
      **Documentation:** doc coverage gate reports **1989 public entities across 133 headers, ALL
      documented**. **ARM:** `TU includes 168 headers` → clean.
- [x] **`python3 tools/src_build_gate.py self-test && check` — SIX builds green.**
      `self-test: OK (33 detector cases, each against a real build)` (was 28; +5 assertions across
      the two new cases 14 and 15). `check: PASS (all 6 builds link)` — `bench`, `xdrive`, `tank`,
      `tank-drive`, `tank-library`, `tank-calibrate`; **0 disallowed warnings** on every one, and
      that is a stronger statement than it was this morning because the gate CAUGHT four real
      defects here first (three `-Wformat-truncation` sites and the sixth build's missing beacon).
- [x] **Host suite green; PROS-free and layering guards green.**
      **1267 cases / 1,624,433 assertions, 0 failed, 3 skipped.** Guard 1 (`#include <pros/>`
      outside `hal/pros/`) — empty. Guard 2 (`shulib/sim/` outside `sim/`) — empty:
      `calibration/` includes no `sim/`.
- [x] **Tests 1–10 green; every §4 mutation observed red and logged.** The brief's ten rows are
      covered by **31 test cases across six files** (5 record + 8 block source + 5 registry +
      11 boot check + the 4 MotorGroup cases, plus calibrator case 21). **Seventeen distinct
      mutations** (M-P1…M-P17), every one applied to the real tree, rebuilt with the depend-then-
      fast recipe, the `Linking` line seen, run and OBSERVED — with the two stale-binary GREENs
      recorded honestly (M-P1 and M-P5 each reported green once from a binary that had not been
      relinked, and each was re-run correctly). Two of them (M-P12, M-P13) went red at COMPILE
      time on a `static_assert`, and M-P13 was additionally re-run with those asserts relaxed so
      the runtime half was observed too.
- [x] **"shulib Calibrate" and "shulib Teleop" both say UNRUN on their boot banner and their
      screens.** Calibrate: the banner block prints `THIS PROGRAM HAS NEVER RUN ON HARDWARE. It
      POWERS THE MOTORS.` and `Every threshold in it is INVENTED for a first run (A4 register
      HA-136).`; the screen header's subtitle is
      `the robot measures its OWN drivetrain -- NEVER RUN BEFORE`; `initialize()` prints the same
      before anything is constructed. Teleop: the banner says `Not yet run on hardware before this
      boot: the first run is wheels-up`, the telemetry carries `HARDWARE-UNVALIDATED: …`, and the
      screen header's subtitle now reads
      `THE LIBRARY drives: Chassis::drive() -- NEVER RUN BEFORE`.
- [x] **Station G written; the PROGRESS log complete with the prose-review record.** Station G is
      ~150 lines in Station F's shape, with every screen string copied from the program's source.
      This log carries the order of work, **28 rulings beyond the brief (P-R1…P-R28)** each with
      its alternative and reason, every mutation with its OBSERVED message, the two stale-binary
      traps and the recipe that closes them, the prose review as a per-file table, and the
      declared deviations (faq.md touched; the MotorGroup/main.cpp ordering).
- [x] **No hardware claim anywhere.** Both programs say so at boot and on screen (above). The
      three new register entries are all UNSETTLED and each names what only a brain can settle —
      HA-139 says explicitly that **no host test can fail if the fopen mode is wrong**. The
      roadmap paragraph, the build-order Next block and the README bullet all end on "has not
      run". Station G's closing note says "NOTHING IN THIS STATION HAS BEEN RUN" and tells the
      team lead to expect refusals. The one place a number is claimed — plant case 9's degraded
      run — states it as a PLANT measurement and says R5 owns the real one.

**Nothing was committed.** No `git add`, no `git commit`, no `git push`, no worktree, no branch
change. `git status` is dirty with exactly the files this part touched, plus three files that were
already dirty or untracked when this executor started and were NOT touched by it:
`docs/internal/chunks/SESSION-2026-09-18-autonomous.md` and
`docs/internal/chunks/T2-input-shaping-drive-modes.md` (both the coordinator's, edited before the
launch) and the untracked `docs/internal/chunks/G1-robot-config-and-builder.md`.

**`include/shulib/teleop/` was NOT modified** — `git status --short include/shulib/teleop/` is
empty. Chunk T2 owns that directory in its own worktree. Two notes for the integration: (1) this
part READS `teleop/drivetrain_degradation.hpp` (the boot check derives its verdict from
`evaluateDegradation`) and `teleop/coupled_side_monitor.hpp` (through `hal::MotorGroup`), so a
change to either's semantics changes the boot check's verdicts — the dead-port policy is now
shared by THREE programs, not two; (2) Part 1's `static_assert` pinning
`teleop::kMaxSideMembers == hal::kMaxSmartPorts` is still in
`test/drivetrain_description_test.cpp` and still fires at compile time if T2 moves that constant.

**Suite counts, end to end:** clean tree `17aa4eb` — 1233 cases / 1,568,972 assertions · before the
`MotorGroup` edit — 1263 / 1,622,729 · after the `MotorGroup` edit, before any new test — 1263 /
**1,622,738** (+9, all in the one rewritten case that asserted the old contract; no other file
moved) · final — **1267 cases / 1,624,433 assertions, green, 3 skipped.**

*Log closed 2026-09-18 by the R3d Part 2 executor (opus).*

---

### Addendum — one claim made precise, after the DoD was written

**RULING P-R29 — "the ONE conversion" is scoped to the LIBRARY, not to the repository.**
`hal/pros/device_registry.hpp` originally read "It exists nowhere else in this tree". A grep for
`registry_get_plugged_type` proved that false: `src/bench_r3a.cpp:581` keeps its own
`const int port = idx + 1;`, and the brief EXPLICITLY rules the tester's census out of scope
("a measuring tool with a hardware history; leave it"). A header that overstates its own invariant
is worse than one that states a smaller true one, because the next person greps for the second copy
and finds it. The header now says "the ONE conversion IN THE LIBRARY", names the tester's copy as a
deliberate exception, and says that if either moves the other must too — and the tester's own
comment at `bench_r3a.cpp:112-122` already says the same thing from its side.

Re-verified after that edit and after the `docs/api/` regeneration it forced: suite **1267 /
1,624,433 green**; doc gates green; ARM `TU includes 168 headers` clean; both CI guards and the
removability grep empty; `src_build_gate.py check` **PASS (all 6 builds link)**.

## Part 3 — THE BENCH SESSION (worksheet Station G), coordinator's live log — started 2026-09-24 01:36:21 EDT

> Written by the coordinator in real time, from what the brain printed and what the team lead
> reported. Every hardware claim below cites its log file or says "team lead's report".
> Packages: built 2026-09-24 from the COMMITTED tree `018424c` in a clean detached worktree
> (`.claude/worktrees/hw-018424c`) — the G1 Part 2a executor was working in the main tree at the
> time and cannot have reached them. Beacons and build stamps checked per package:
> Bench Tests `shulib-robot-variant=tank`, shulib Drive `tank-drive`, shulib Teleop
> `tank-library`, shulib Calibrate `tank-calibrate`; all `v0.1.1-313-g018424c` (not dirty).

- **01:36:21** — **The brain ENUMERATED on this laptop's USB** — the first time since
  2026-09-18 (the team lead brought a new data cable). `pros lsusb`:
  `/dev/ttyACM0 - VEX Robotics V5 Brain - 1CA1A500 - VEX Robotics Communications Port` (system),
  `/dev/ttyACM1 - VEX Robotics V5 Brain - 1CA1A500 - VEX Robotics User Port` (user).
- **01:37:09** — **G.1: all four programs UPLOADED** with `pros upload --slot N --name … --after none`
  (never auto-run): slot 3 "Bench Tests", slot 1 "shulib Drive", slot 2 "shulib Teleop", slot 4
  "shulib Calibrate" — each `Finished uploading … to V5`, exit 0; the cold library package was
  already on the brain. G.1.1's on-screen stamp check is the team lead's to read at each start.
- **01:39:55 → ~01:48** — **G.2: Bench Tests (slot 3) run from the brain's screen**; user port captured
  raw to `docs/internal/evidence/r3d-session-2026-09-24-census.log` (8,115 bytes; read with
  `strings`). What the brain printed, verbatim in the log:
  - **SD CARD PROBE (HA-122):** `usd_is_installed() : 1`, `fopen(/usd/probe.txt,wb) : OK`,
    `fwrite/fflush : 17 bytes, fflush=0`, `CARD IS FULLY WORKING`.
  - **DEVICE CENSUS:** port 1 RADIO; port 2 IMU; MOTOR on **11 12 13 14 15** (left) and on **18 19
    20** (right) — `motors found: 8 (chassis table expects 10)`. **Ports 16 and 17 are EMPTY.**
  - **MOTOR WATCH, one front-first push:** L11 −373 DOWN · L12 +377 UP · L13 −290 DOWN ·
    **L14 0 "did not move"** · L15 −380 DOWN · R18 +371 UP · R19 −371 DOWN · R20 +372 UP;
    `PARTIAL: 7 of 10 table ports moved; silent: 14 17 16`. Every sign that moved AGREES with the
    typed table's (−11 +12 −13 −15 | +20 −19 +18). **Port 18 travelled the same as its side-mates
    (+371 vs 372/371)** — the ~20 % short travel of 2026-09-10 (HA-130) did not appear on this push.
    **Port 13 travelled ~23 % less than its side-mates (−290 vs −373/−377/−380).**
  - Findings for the team lead before any calibration: 16 and 17 absent; 14 answers the registry
    but did not turn under a push (not meshed? a slipping gear? a dead encoder?); 13 short.
- **~01:48** — **the brain dropped off USB** (`pros lsusb`: no ports) — the capture ended with it.
- **G.1.1 confirmed from the log:** the running Bench Tests printed `build hash : v0.1.1-313-g018424c`
  and `BUILD STAMP: Sep 24 2026 01:34:38` — the package uploaded at G.1. PROS 4.2.2, VEXos
  `V1.1.5 (b18)`. The brain printed its table header as `LEFT -11 +12 -13 +14 -15 | RIGHT +20 -19
  +18 -17 +16 ... MEASURED` — the typed table's claim, which this census contradicts (16, 17 absent).
- **back on USB** after the drop (same brain `1CA1A500`, ttyACM0/1); a second capture started.
- **01:52 → 01:58** — **Bench Tests again, after the team lead re-seated 16 and 17**; capture
  `docs/internal/evidence/r3d-session-2026-09-24-bench2.log`. Three census + MOTOR WATCH rounds:
  - census 1 and 2: **all TEN motors** (11–20), IMU 2, radio 1. **Census 3: port 19 ABSENT**
    (`motors found: 9`). Port 19 moved normally in the 01:4x push (−371) and is silent (0 deg) in
    every push here — **port 19 is dropping off the bus: a connection fault (the cable, a cable
    end, or brain port 19), not only a stuck encoder.** Team lead: "i cant tell if the port is bad".
  - pushes: L 382/−384/386/−389/390 · R −388/390/**−303**/0/−394; then 328/−328/328/−331/329 ·
    −328/327/**−248**/0/−329; then 115/−115/114/−114/117 · −116/117/**−78**/—/−117. **Port 14 now
    moves** with its side. **Port 18 travels 22–33 % short in all three** (HA-130's pattern, which
    the 01:4x push did not show; port 13 was the short one then).
  - **Every sign here is the INVERSE of the 01:4x push** (L11 UP here, DOWN there) and consistent
    across all ten ports with the table's signs under a BACK-first push — either these pushes went
    back-first or the rebuild moved the front. To be answered by the team lead; Calibrate asks it.
  - Checked in the source before advising: the calibrator never drives a candidate that did not
    move in its push (`drivetrain_calibrator.hpp`:25-28), and the boot check treats a motor on a
    port the record does not name as a WARNING, never a refusal (`boot_check.hpp`:28).
- **02:03** — **G.3 (the wheels-up Calibrate run) SKIPPED — the team lead's decision** ("I don't want
  to turn the wheels by hand because it gets confusing"). What is given up, stated: the first
  powered calibrator run with the robot unable to travel, and the evidence that side-assignment
  REFUSES when nothing turns (that refusal stays host-proven only). Mitigations for G.4 on the
  floor: its first powered action is 0.4 s single-motor pulses; ABORT (0 V + coast) is always on
  screen; 3 m clear and a second person at the battery per G.0.2. Recommended to the team lead:
  UNPLUG port 19 for the session so the drivetrain is the same 9-motor robot on every run.
- **02:51** — **Sensors mounted (team lead's report):** distance sensor facing RIGHT on **port 5**,
  distance sensor facing LEFT on **port 4**, **AI Vision** at the FRONT on **port 3**. Offsets from
  the turning centre not yet measured. To be confirmed by the next census (capture `session-3`).
- **02:53** — Team lead: all three sensors are **level** and "pretty much in the center" (read as:
  centred front-to-back for the side sensors, centred left-right for the front camera). The
  side sensors' lateral offsets and the camera's forward offset and height are still UNMEASURED
  — owed before L6 (wall reset) and L3 (vision) use them.
- **~03:00 → 02:59** — **shulib Calibrate (slot 4), three starts** — capture `session-3` (to be saved as
  evidence at session end). Build stamp `Sep 24 2026 01:34:54`, hash `v0.1.1-313-g018424c` (the
  uploaded package). Its STAGE 0 census, from the registry: RADIO 1 · IMU 2 · **VISION 3 ·
  DISTANCE 4 · DISTANCE 5** · MOTOR 11–18 and 20 — **9 motors, port 19 absent** (unplugged, as
  recommended); the sensors the team lead mounted are CONFIRMED on those ports.
  IMU port DISCOVERED from the registry (2, matching the table). Cartridge stated BLUE 600 rpm.
  - **Runs 1 and 2:** `PUSH THE WHOLE ROBOT ... | 0 of 9 motor(s) have moved`, then DONE touched
    → `STOPPED: the push moved 0 port(s); a drivetrain needs at least 2`. All motors 0 V + coast;
    nothing written. (The count only takes a push that happens BEFORE DONE.)
  - **Run 3:** DONE → `ANSWER: FRONT 1st` → `STAND CLEAR -- pulsing one motor at a time. | port 11:
    settling, yaw +0.8 deg` → **`STOPPED: port 11: the robot turned only 0.17 deg (floor 2.00 deg)
    while it was driven -- is the robot on the FLOOR, not up on blocks?`** Nothing written.
    Whether the robot was on the floor is being asked. **If it was:** one motor at 3.0 V for 0.4 s,
    back-driving four coasting side-mates through a coupled train, turned the robot 0.17° — the
    invented pulse numbers (HA-136: `pulseVolts` 3.0 V, `pulseTime` 0.4 s, `pulseYawFloor` 2.0°)
    would be too weak for a five-motor coupled side, and side assignment cannot pass on this robot
    as built. **If it was on blocks:** this is the G.3 refusal, the program working.
  - **The team lead's report on the screen, verbatim:** "THE UI ON THE ROBOT IS TERRIBLE WHILE
    RUNNING CALIBRATE I CANT READ IT AND THE BUTTONS DONT WORK I DONT KNOW AFTER I PUSH IT WHAT IM
    SUPPOSED TO DO". The log shows every touch registering as an ANSWER (DONE, FRONT 1st), so the
    touches worked; what failed is that the screen did not make the next action clear or legible.
    A usability defect in `src/calibrate_program.cpp`'s panel — owed a fix.
- **CORRECTION (written 2026-09-24, after a read-only investigation re-read the whole capture):** the
  calibrate evidence log holds **FIVE** Calibrate starts, not three — the coordinator read only the
  capture's last 120 lines when it wrote the entry above. From
  `strings docs/internal/evidence/r3d-session-2026-09-24-calibrate.log`:
  - **A** (lines ~483-518): DONE → FRONT 1st → `STOPPED: port 11: the robot turned only 0.14 deg`.
  - **B** (~617-652): DONE → FRONT 1st → the push had yawed the robot `+10.5 deg` → pulses ran and
    **STOPPED AT PORT 18**: `the robot turned only 0.09 deg (floor 2.00 deg)`. Reaching port 18 means
    every drive port before it in candidate order (11–17) CLEARED the 2° floor — **the 3 V / 0.4 s
    single-motor pulse worked for seven motors in this start** and failed on port 18, the member
    that travels short (HA-130).
  - **C, D** (~751-783, ~882-914): DONE before any push → `the push moved 0 port(s)`.
  - **E** (~1013-1048): the start recorded above (port 11, 0.17°).
  So the single-motor pulse is INTERMITTENT on this robot, not impossible, and the program printed
  no per-pulse numbers that would say why. The port-11 refusals carry POSITIVE yaw (+0.14, +0.17)
  where HA-134 predicts negative for a left member driven forward — noise at this level, or pushes
  made with the other end first while FRONT 1st was answered (open).
- **2026-09-24 04:19** — **Robot two's wheels are OMNI on both sides** (team lead's report, answering the
  physics investigation's open question). Consequence for R3d Part 2F: a one-sided push meets little
  lateral scrub, so a pulse that breaks the coupled train's static friction should turn the robot;
  the plant's coast-follow fraction for this robot is near FREE rolling (κ close to 1), which is the
  regime where a one-sided push both turns AND translates. The team lead gave the **go on Part 2F-1**.

## Part 2F-1 — the library: one push, a ramped pulse, the spin guards, a plant that can fail, the panel model — executor log, started 2026-09-24 04:21:08 EDT

> Executor: Claude Opus 5.5 (1M context), launched by the coordinator on the team lead's go (2026-09-24).
> Brief: [`R3d-part2f-calibrate-on-the-robot.md`](R3d-part2f-calibrate-on-the-robot.md) §1 (F1), §3, §4 F1, §5 F1, §6.
> Tree at start: `76323f2`, clean. **Nothing here is committed by the executor; nothing here has run on a robot.**

### Log

- **2026-09-24 04:21:08 EDT** — Log created before any code (rule 2 of the launch prompt). Required reading begun: RESUMING.md, the Part 2F brief (all of it).
- **2026-09-24 04:35:59 EDT** — Required reading done: RESUMING.md; the Part 2F brief in full; both investigations
  (`calib_invest.json`, physics + screen); this log's Part 3; the R3d spec (T3, §4.2 rest definition)
  and the Part 2 brief; `drivetrain_calibrator.hpp` (all), `drive_plant.hpp` (all),
  `motor_model.hpp`, `motor.hpp`, `fake_motor.hpp`, `scenario.hpp`, `motor_group.hpp`
  (`medianInPlace`); `test/drivetrain_calibrator_test.cpp` (all), `test/drive_plant_coupled_test.cpp`
  (all); `src/calibrate_program.cpp` READ ONLY, for what it needs from the calibrator.
  **What the old program uses** (the shim list is decided from this, not guessed): `Answer::{Done,
  FrontFirst, BackFirst, Distance24, Distance48, Distance72, Abort}`; a `switch` over EVERY `Phase`
  enumerator with no `default` (so removing `Phase::ScalePush` OR adding a new `Phase` would be a
  `-Wswitch` error in the src build gate); `accepts()`, `status().{phase,prompt,progress}`,
  `report().{portCount, ports[].{port,sign,side,pushDeltaRad,tapeDeltaRad}, leftSpreadRad,
  rightSpreadRad, widthCcwIn, widthCwIn, widthDisagreement}`, `result()`, `reason()`, `tick()`,
  `answer()`, `abort(const char*)`, `start()`, the constructor, `HumanFacts`. It does NOT use any
  `CalibratorConfig` field.
- **2026-09-24 04:35:59 EDT** — **Baseline suite (clean tree `76323f2`, freshly built):** `test cases: 1386 | 1386 passed |
  3 skipped`, `assertions: 1638673 | 1638673 passed`. Per-case XML saved to the session scratchpad
  (`f1/baseline.xml`, 1389 `TestCase` elements incl. the skipped) with a comparison script
  (`f1/cases.py`) that diffs every case's assertion count AND its printed MESSAGE text, so "every
  other suite bit-identical" is checked on the numbers the suites print, not only on counts.
- **2026-09-24 04:38:19 EDT** — **Design read-out before any code (rulings beyond the brief, each with its alternative):**
  - **R-F1a — the backward-compatible surface for `src/calibrate_program.cpp` is THREE retired names,
    nothing more** (from the use list above): `Answer::Distance24/48/72` stay as enumerators that
    `accepts()` NEVER accepts (so the old program's `buttonsFor()` never draws them) and that
    `answer()` refuses as out of order; `Phase::ScalePush` stays as an enumerator the calibrator never
    enters (the old program's `switch` has no `default`, so removing it is a `-Wswitch` build error in
    the gate); `PortObservation::tapeDeltaRad` stays, always 0 (there is no tape push). Each is marked
    RETIRED in its `///` and names F2 as the part that removes it. With these the OLD program still
    runs, honestly, on the NEW calibrator: its prompts come from `status().prompt` (rewritten below
    to fit its 44-character cut), its DONE is still accepted at any time (the gate is a readiness
    query — F-R3), and it goes push → FRONT/BACK → ramped pulses → guarded spin → Done → save.
    Alternative rejected: keeping the whole old surface (`pulseVolts`, `spinVolts`, the distance
    step) — it would be a second calibrator inside the first, and the program uses none of it.
  - **R-F1b — `Phase` gains NO enumerator.** The scale is computed at the end of `SideAssign`, once the
    sides are known (brief F1.1); a new phase would break the old program's `switch` just as removing
    one would.
  - **R-F1c — one push gives signs AND scale, so each drive port's push-direction travel is |Δ| by
    construction** (its raw sign IS the direction it moved). The old tape push's "a side's median
    travel ≤ 0 — the push must roll BOTH sides forward" refusal therefore cannot fire any more and is
    REMOVED rather than kept as dead code; the straightness bound (now on this push) is what guards a
    twist that would invert one side's signs. The scale divides `pushDistance` by the side's median
    |travel|, so a BACK-first push (the robot moved backward in its own frame) gives the same scale.
  - **R-F1d — "far enough" = the median |travel| of the moved candidates ≥ `minPushRad`**, the SAME
    threshold the per-side check uses; `finishPushCapture` refuses a push that fails it with the
    existing too-short refusal (`PushTooShort`) at DONE, and the per-side check still runs after the
    sides are known (as today). Alternative rejected: only the per-side check — it would run nine
    powered pulses on a push already known too short to give a scale. "At rest" is in `pushReady()`
    but is NOT a finish refusal (brief F1.1/F-R3: "a readiness query, not a new refusal"); the gate is
    therefore STRICTER than the finish, never looser — no DONE the panel offers can be refused for a
    predicate the gate checked.
  - **R-F1e — `restYawRateFloor` = 2 deg/s, with a precondition `restYawRateFloor x pulseTime <
    pulseYawFloor`.** Why that relation: a drift slow enough to pass the rest check and continue
    through the pulse adds at most 0.8 deg, and a side can only come out WRONG if a leak opposing the
    true yaw is itself ≥ the 2 deg floor — so the relation makes "a slow drift flips a side"
    impossible by construction, not merely unlikely. Why not lower: the host's hostile IMU (HA-22,
    invented) has 0.5 deg/s white rate noise per sample; 2 deg/s is 4 sigma, so a 50-tick settle
    window is broken by noise ~0.3 % of the time. A real V5's rest noise is unmeasured.
  - **R-F1f — RUN AGAIN needs NO calibrator API:** the program rebuilds the calibrator in place
    (`std::optional<DrivetrainCalibrator>::emplace`), which carries nothing over BY CONSTRUCTION. A
    `reset()` was the alternative — rejected: every field added later would have to remember to be
    reset, and one that did not would leak a sign or a side between runs. The header says exactly
    what F2 must do; a host case proves two back-to-back runs on one plant are independent.
  - **A physics note for the coordinator (a FINDING, not a stop — the plant change is still built as
    specified):** this log's 04:19 entry reads "omni ⇒ κ close to 1 (free rolling)". Rigid-body
    dynamics say the opposite for a free-rolling OMNI side: one side pushed forward, the other side
    exerting no longitudinal and no lateral force, a uniform square robot gets a = F/m and
    α = F(W/2)/I with I = m(L²+W²)/12, so the idle side's speed is v_com − ωW/2 = −0.2 × the driven
    side's — it runs slightly BACKWARD and the robot turns MORE than a pinned pivot. κ → 1 (rolling
    instead of turning) is the TRACTION-scrub regime the physics investigation described. The brief's
    κ (a kinematic stand-in: the coasting wheel's speed = κ × the driven wheel's, κ ∈ [0, 1]) is
    therefore built as specified and its κ-near-1 cases prove the ROLLED branch — but which κ robot
    two shows is a number only the serial record can settle, and the physics above suggests the
    pulses should turn an all-omni robot readily once the train breaks free (consistent with start B,
    where 3 V cleared the floor on seven motors). Recorded for HA-137; nothing in the tree depends on it.
- **2026-09-24 04:47:04 EDT** — **F1.4 plant: LANDED in `include/shulib/sim/drive_plant.hpp`, ADDITIVE.**
  `DrivePlantConfig::coastFollow` (κ ∈ [0, 1], default 0; κ > 0 only on a two-wheel kinematics): a
  wheel whose every COUPLED member is at exactly 0 V AND reports `BrakeMode::Coast` turns at κ × the
  driven wheels' mean spin — computed on a SEPARATE three-pass path taken only when κ > 0, because it
  draws the degradation seams in a different order from the original interleaved loop, which runs
  untouched at κ = 0. `MemberFault::Decoupled` (appended): its command is left out of the train's
  command (`coupledOnlyCommand()`, reached only when a wheel HAS a decoupled member), its encoder
  reports where it stood when the fault was set + its own free spin through the plant's motor model
  + `DrivePlantConfig::decoupledDrag` (default 0, the brief's literal fault) of the train's travel.
  Ruling beyond the brief: **`decoupledDrag`** — without it a slipping member reads NOTHING on a push
  and is never a drive port, so the calibrator's DECOUPLED branch could only be reached by setting the
  fault after the push; with it the plant models port 18 as the evidence shows it (captured by the
  push, 22–33 % short — HA-130) and the "a left-out port dragged by its mates is not a stray mover"
  logic becomes testable. Default 0 keeps the brief's definition.
  **Bit-identity, measured:** after the plant change (calibrator untouched) the fresh binary ran the
  whole suite and `f1/cases.py baseline.xml after_plant.xml` → `compared 1389 cases outside []: 1389
  identical, 0 differ` (assertion counts AND every MESSAGE text).
  **Plant cases 21–26** added to `test/drive_plant_coupled_test.cpp` (coast-follow never touches a
  driven wheel; its exact shape ω = (1 − κ)v/W, vx = (1 + κ)v/2, κ = 1 exactly zero yaw; it reads the
  brake mode, one held member is enough; Decoupled; **the regression that documents 2026-09-24**: at
  kS 1.2 one member of five at 3 V for 0.4 s turns the plant EXACTLY 0.0 rad, 6 V also exactly 0, 9 V
  −6.47° pinned and 0.0 rolling; the preconditions). First run: one red of MY OWN expectation (case
  22 asserted x > 5 in at κ = 1; the robot rolls v × 0.4 s = 1.41 in — the assertion was wrong, not
  the plant; replaced with the exact expected value). Then `DrivePlant*`: 9 cases, 4097 assertions,
  all green. Mutations for these are run with the rest, below.
- **2026-09-24 05:02:12 EDT** — **F1.1–F1.3 + F1.5 calibrator: REWRITTEN (`include/shulib/calibration/drivetrain_calibrator.hpp`,
  1296 → 1904 lines — most of the growth is the header's explanation and the per-field `///`).**
  One push (`PushCapture` now carries signs AND travel; `finishPushCapture` refuses PushTooFew →
  PushCurved → PushTooShort in that order; the scale is computed at the end of `SideAssign` from the
  travel CAPTURED AT DONE, since the pulses move every shaft afterwards); the ramp (`pulseLevels` 3/6/9
  + `pulseLevelCount`, stop at the first level at/over `pulseYawFloor`; a contradiction is checked at the
  deciding level (own ≤ 0, as before) AND at any level where the member's own shaft went back by
  `movedThreshold` or more; at the top level ROLLED (others ≥ movedThreshold) → DECOUPLED (own ≥
  `decoupledOwnMin`, left out, warning, never commanded again) → STILL); rest = motors AND
  `|yawRate| ≤ restYawRateFloor` (one `atRest()` shared by the pulses, the spin rests and
  `pushReady()`); the spin's wrong-way guard, start check and step-up (`spinLevels` 3/6; the start
  check measures |yaw| gained SINCE THE LEVEL BEGAN — ruling: a cumulative check would let 4.9° crept
  at 3 V plus 0.2° at 6 V pass as "started" and then stall for the whole timeout; and |yaw|, not the
  directional yaw, so a spin already going the wrong way is never stepped UP — the wrong-way guard ends
  it instead); `Refusal` (21 codes) + `refusalPort()` on every refusal; `warning()`; the status
  numbers; `PulseRecord`/`SpinRecord` + `formatPushRecord/formatPulseRecord/formatSpinRecord`;
  `refusalName()`/`pulseVerdictWord()`; the percent signs gone ("pct"). **Removed:** `pulseVolts`,
  `spinVolts` (replaced by the level arrays), `Step::ScaleWatching/ScaleAwaitingDistance`, the
  distance branches of `answer()/accepts()`, the "median travel ≤ 0" refusal (R-F1c). **Retired but
  kept for the old program:** `Phase::ScalePush`, `Answer::Distance24/48/72`,
  `PortObservation::tapeDeltaRad` (R-F1a). Copy construction/assignment are now `= delete`d
  (they were already ill-formed — reference members — this states it). `status().prompt` now points
  into a buffer the calibrator owns for the push step (it names `pushDistance`), documented as valid
  until the next tick/answer/abort; every prompt is ≤ 44 characters so the OLD program's `%.44s`
  shows it whole.
  Rulings found while writing: **(i)** the harness pushes BY MOTOR COMMAND, so a wheel with a
  Decoupled member got a 5/4 larger mean and the push curved 95° (case 24's first red) — fixed IN THE
  HARNESS (`CalibRig::coupled[]`, documented there as an artifact of pushing by command, not a
  property of the robot); **(ii)** the record formatters add 0.0 to every signed number, because a
  zero travel times a captured sign of −1 printed "own -0.0 deg" (case 31's first red — a reader would
  take it for a motor that moved backward); **(iii)** `status()` is the snapshot of the last tick,
  `pushReady()` reads live — case 29 had to tick once before comparing them (documented in the test).
  **Calibrator cases: 30, all green** (18 before → 30: 4, 4b, 5–19, 21 rewritten; 22–33 new).
  **Whole suite, fresh binary:** `test cases: 1404 | 1404 passed | 3 skipped`, `assertions: 1600178`.
  `f1/cases.py baseline.xml after_cal.xml drivetrain_calibrator_test.cpp` → `1377 cases compared: 1371
  identical, 6 differ` — the six are `DrivePlant 21–26`, NEW (absent before); plant cases 18–20 and
  every other suite are identical in counts AND printed values. The assertion total FELL (1638673 →
  1600178) because the old case 21 ran a `REQUIRE` on every tick of every scripted drive (52952 of its
  assertions); the new one stops the script with `stopWhen` and asserts the same agreement per
  (step, answer) pair — the property is unchanged, the per-tick bookkeeping is gone. Measured timings
  worth keeping: the spin's step-up fired at 2.01 s and its refusal at 4.02 s (the FakeClock sums
  0.01 s steps, so tick 200 reads 1.99999… and the check fires one tick later — the bound is "the first
  tick at or after 2 s"); the wrong-way stop came at 15.53° of TRUE rotation (the 15° bound plus at most
  one tick of spin).
- **2026-09-24 10:51:03 EDT** — **F1.6 panel: LANDED — `include/shulib/calibration/calibrate_panel.hpp` (new) and
  `test/calibrate_panel_test.cpp` (new, cases P1–P12).** PROS-free, allocation-free, ASCII-only.
  `viewFor(cal, secondsSinceLayoutChange, SaveOutcome = NotAttempted)` (ruling: the third parameter,
  defaulted, because only the program knows whether the write + read-back succeeded — SAVED / NOT SAVED
  / MEASURED — and the brief's two-argument call still compiles); `hitTest`; `layoutKey` (changes with
  the screen and the buttons' ids + rectangles, NOT with arming); `TouchGate` (the release edge; an
  error read ignored and never moving the count, so it cannot replay; `view()` owns the arming clock so
  the program never tracks it; `touch()` re-checks `accepts()` — and `pushReady()` for DONE — against
  the calibrator NOW, since the view a finger landed on may be a frame old); `stoppedWords(Refusal)`;
  `wrapWords<Cols, Lines>` (constexpr; never splits a word; says when text did not fit);
  `screenText`, `panelLiteralsFit()` and `panelLayoutHolds()` checked by `static_assert` (every named
  literal and every STOPPED wording against its budget; every fixed rectangle on screen; ABORT's slot
  inside the powered row; no middle box whose HIT box reaches the button row's). Rulings: **ABORT is
  not on the finished screens** (Saved / Stopped): the run is over and abort() from Done would discard
  a result — RUN AGAIN is the one button there; **RUN AGAIN is a middle-right box**, never in the
  button row, so a reflexive tap after DONE-refused or ABORT cannot restart the run; **the pulses
  screen's NEXT line carries the L/R/OUT chips** (the sides as decided — the diagnostic the screen
  investigation asked for; the brief's "it spins -- stand clear" is the Spin screen's own instruction,
  and ABORT is the whole row) — alternative: a sixth zone, rejected for lack of rows under the
  estimated LARGE height; **two sub-budgets** besides the brief's three: `kHeaderCols` 24 (the header
  bar is shared with the build time) and `kNoteCols` 18 (the note left of RUN AGAIN) — both
  constants, both checked. The rig was lifted into `test/calibration_test_rig.hpp` so both files
  drive the SAME scripted operator (a pure move: the calibrator cases' counts stayed 30 / 15869).
  First run: 11 of 12 green; the red was my own sanity bound (P1 asserted > 20000 views; the seven
  runs give 17504 — the bound, not the panel, was wrong; now > 10000 with the measured number in a
  comment). Whole suite: `test cases: 1416 | 1416 passed | 3 skipped`, `assertions: 2386630`.
- **2026-09-24 13:27:02 EDT** — **The brief's nine required mutations (§4 F1), each RUN on a fresh binary** by
  a driver (`f1/mutate.py` in the session scratchpad): apply the edit (the old text must be unique) →
  touch the edited header and both test files that include it → `cmake --build build/test --target
  shulib_tests/fast` and REQUIRE `Linking CXX executable shulib_tests` → run the named case → keep the
  red verbatim → restore the saved bytes (`filecmp` byte-identical: TRUE for all nine) → relink → the
  same case green (TRUE for all nine). Observed reds:
  1. **Flip the side rule** (`yaw_ < 0.0 ? Left` → `yaw_ > 0.0 ? Left`) → case 6: `FATAL ERROR:
     REQUIRE( runCalibration(rig, RunOptions{}) == Phase::Done ) is NOT correct! values: REQUIRE( 7 == 6 )
     logged: the robot spun the WRONG way: 15.5 deg against the CCW spin it was driven for (bound 15.0
     deg) -- the sides are swapped`. (Worth noting: a flipped side rule is now caught ON THE ROBOT by the
     new wrong-way guard, where under Part 1 it would have spun the wrong way for 20 s. The first run of
     this mutation printed no reason; case 6's REQUIRE now carries the reason and it was re-run.)
  2. **Drop the yaw-rate half of rest** (`atRest()` → motors only) → case 26: `REQUIRE( 7 == 6 ) logged:
     port 11: driven forward at its captured sign +1 at 3.0 V, its OWN encoder went 0.000 rad -- the push
     and the pulse contradict each other` — the hand's turn leaked into the 3 V pulse (which moves
     nothing on this plant), crossed the 2° floor, and the run then read its own still shaft as a
     contradiction: exactly the leak the rest check exists to stop.
  3. **Drop the decoupled classification** (`own >= decoupledOwnMin` → `own >= 1e300`) → case 24:
     `REQUIRE( end == Phase::Done ) ... values: REQUIRE( 7 == 6 ) logged: port 18 could not move the
     robot at 9.0 V: it turned 0.00 deg (floor 2.00 deg), its own shaft 630.9 deg, no other drive port
     moved -- is it on the FLOOR? is a wheel stuck?`
  4. **Drop the wrong-way guard** (`< -spinWrongWay` → `< -1e300`) → case 27: `CHECK( cal.refusal() ==
     code ) ... values: CHECK( 14 == 12 ) logged: refusal SPIN_TIMEOUT, expected SPIN_WRONG_WAY: the
     CCW spin reached only 5.39 of 2.00 turns in 20.0 s` (+ `reasonMentions(... "WRONG way")` false).
     **Finding from this red:** the timeout message printed |yaw| — 5.39 turns of the WRONG way reported
     as progress. Fixed: it now prints the DIRECTIONAL turns (unreachable behind the guard, and the
     honest number either way).
  5. **Drop the spin-start check** (`if (!spinStarted_)` → never) → case 28: `CHECK( 14 == 13 ) logged:
     refusal SPIN_TIMEOUT, expected SPIN_NO_START: the CCW spin reached only 0.00 of 2.00 turns in 20.0 s`
     — the 20 s stall the check exists to prevent.
  6. **Draw DONE before ready** (`if (ready)` → `if (cal.accepts(Answer::Done))`) → P1 and P2: `CHECK(
     buttonMeansSomething(b, rig.cal) ) is NOT correct! ... logged: static_cast<int>(v.screen) := 2,
     static_cast<int>(b.id) := 0` (the Push screen, the DONE button) — 5070 failed assertions across the
     two cases.
  7. **Move ABORT on one screen** (the Push screen's ABORT at x0 320) → P3: `CHECK( r == kAbortSlot ) is
     NOT correct! values: CHECK( {320, 180, 474, 234} == {330, 180, 474, 234} ) logged:
     static_cast<int>(s) := 2` (first run printed `{?}`; a `doctest::StringMaker<PanelRect>` was added so
     the red names the corners, and the mutation re-run).
  8. **Put FRONT where DONE was** (the robot's own hazard) → P4: `CHECK_FALSE( overlaps(hitBox(after.
     buttons[i].rect), hitBox(causing)) ) ... logged: DONE -> WHICH END, static_cast<int>(after.buttons
     [i].id) := 1` AND `CHECK( late.action == TouchAction::None ) ... values: CHECK( 1 == 0 )` — the
     reflexive second tap, 0.6 s later, ANSWERED FRONT FIRST.
  9. **Drop the arming delay** (`secondsSinceLayoutChange >= kArmingDelaySeconds` → `>= 0.0`) → P5:
     `CHECK_FALSE( done->armed ) is NOT correct!` and `CHECK( gate.touch(tapAt(releases, kPrimarySlot),
     v, rig.cal).action == TouchAction::None ) ... values: CHECK( 1 == 0 )`.
  None came back green on its first run.
- **2026-09-24 13:52:17 EDT** — **CORRECTION to my own procedure, found by a mutation that came back GREEN.** The first
  runs of four PLANT mutations (below, M14–M17) came back green. Run down before anything else: the
  `shulib_tests/fast` target does NOT track header dependencies in this build — touching
  `include/shulib/sim/drive_plant.hpp` rebuilt **0** objects, and `compiler_depend.make` lists no
  header at all — so objects compiled against an older header survive, and the linker keeps ONE copy
  of each inline function from whichever object it meets first. My driver had touched only the test
  files that include the edited header DIRECTLY; the sim suites' stale objects supplied the
  un-mutated plant. (The launch prompt's "touch EVERY test including the edited header" means exactly
  this, and I had read it narrowly.) Consequences, and what was redone:
  - The driver now touches EVERY `test/*.cpp` (156 objects rebuilt per mutation and per restore — the
    result JSON records the count; a full rebuild is ~11 s here).
  - **Every mutation above and below was RE-RUN that way**; all nine required ones are red again with
    the same messages (M4's message now reads `-5.39 of 2.00 turns`, the directional fix).
  - **My earlier "bit-identical" comparisons were built with the same `/fast` target**, so the sim
    suites' objects may not have been recompiled against the new plant; those comparisons are
    WITHDRAWN. Redone on a binary with every object rebuilt (`touch test/*.cpp`, 156 objects):
    `f1/cases.py baseline.xml after_full.xml` (excluding the calibrator and panel files) → **1377
    compared: 1370 identical, 7 differ** — `DrivePlant 21–26` (new) and `pros_adapter_fence_test`'s
    "only hal/pros includes <pros/>" (**495 → 498 assertions**: it walks every header under
    `include/shulib/` and there is one more — `calibrate_panel.hpp`; same pass/fail, no printed values).
    Plant cases 18–20 and every other suite: identical in assertion count AND every printed MESSAGE.
    Full suite on that binary: `test cases: 1416 | 1416 passed | 3 skipped`, `assertions: 2386635`.
- **2026-09-24 13:52:17 EDT** — **Mutations beyond the brief's list, all on fresh binaries (every object rebuilt), all
  restored byte-identical and relinked green:**
  - M10 `pushReady()` without "far enough" → case 29 (`REQUIRE( 7 == 2 )`: the scripted operator took
    the DONE that appeared and the finish refused it) and P2. **P2 first came back GREEN** under this
    mutation (run before the driver fix, but the reason was real and independent): its not-far-enough
    moment was always MID-push, where the at-rest clause hid the dropped one. Strengthened: a new
    subcase with a short push that has STOPPED; re-run → `CHECK( findButton(v, PanelButtonId::Done) ==
    nullptr ) is NOT correct!`.
  - M11 the ramp never stops early (floor only honoured at the top level) → case 23: `CHECK(
    levelsOf(rig.cal, 11) == std::string{row.levels} ) ... values: CHECK( 3V:under the floor -- next
    level 6V:under the floor -- next level 9V:LEFT  == 3V:LEFT  )`.
  - M12 median → mean for the scale → case 10: `CHECK( 1.83911 == Approx( 1.625 ) )`.
  - M13 the "%%" put back in the disagreement reason → cases 18 and 30: `CHECK( screenSafe(cal.reason())
    ) ... the two spin directions disagree about the track width by 7.3 % (...)` and `CHECK(
    std::strchr(rig.cal.reason(), '%') == nullptr )`.
  - M14 plant ignores the brake mode → DrivePlant 23: `CHECK( 3.52941 == 0 )` (a HELD idle side rolled).
  - M15 plant follows on every wheel → DrivePlant 21: `CHECK( sameTruth(a, b) )` false.
  - M16 a Decoupled member's volts back in the mean → DrivePlant 24: `CHECK( t.h.sample().x == xBefore
    ) ... CHECK( 5.64556 == 4.94118 )` (its 9 V moved the robot).
  - M17 the follow's shape halved → DrivePlant 22: `CHECK( 0.441176 == Approx( 0.882353 ) ) logged:
    kappa := 0.25`.
  - M18 drop the "own shaft clearly backward" contradiction → case 7: `CHECK( 9 == 8 ) logged: refusal
    PULSE_ROLLED, expected PULSE_CONTRADICTION: port 11 moved its gear train (others 49.8 deg) but the
    robot turned only 0.00 deg even at 9.0 V ...`.
  - M19 `TouchGate` without its re-check (first written so it did not compile — `-Werror`
    unused-parameter — which is not a red of the property; rewritten as `alsoReady || !cal.accepts(a)
    || true`) → P6: `CHECK( stale.action == TouchAction::None ) ... CHECK( 1 == 0 )`.
  - M20 an error read moves the release count → P6: `CHECK( 1 == 0 )` (the next good read REPLAYED).
  - M21 `kFrontBox` stretched into reach of the button row → **compile-time red**: `static assertion
    failed: calibrate_panel: the fixed layout breaks a promise (off screen, ABORT moved out of its row,
    or a middle box within reach of the button row)`.
  - M22 `kLargeCols` 18 → 15 (as if the ruler said so) → **compile-time red**: `static assertion
    failed: calibrate_panel: a literal does not fit its line budget, ...` — "changing one constant is
    all it takes", demonstrated.
  - M23 the scale re-reads the shafts after the pulses instead of the travel captured at DONE (first
    written with an unused variable — `-Werror`; rewritten) → case 10: `REQUIRE( runCalibration(clean,
    RunOptions{}) == Phase::Done ) ... REQUIRE( 7 == 6 )`.
  - M24 no too-short refusal at DONE → case 29: `CHECK( 2 == 7 ) ... logged: refusal NONE, expected
    PUSH_TOO_SHORT`.
  - M25 the layout clock never re-greys → P5 and P12: `CHECK_FALSE( true )` on `armed`, and a tap that
    should land on nothing answered (`CHECK( 1 == 0 )`).
- **2026-09-24 14:02:11 EDT** — **`src/` BUILD, checked before any doc work:** `python3 tools/src_build_gate.py self-test`
  → `OK (33 detector cases, each against a real build)`; `check` → `src build gate: PASS (all 6 builds
  link ...)`; `[tank-calibrate] exit 0, 4/4 TUs compiled, warnings: 2 allowed (-Wunused-function), 3
  vendor-ignored, 0 disallowed` — the SAME profile every build had in the pre-change run kept in the
  session scratchpad (`gate_check.log`, tree `018424c`). So the OLD `src/calibrate_program.cpp`
  compiles and links against the NEW calibrator, under `-Wformat-nonliteral`/`-Wformat-truncation`,
  through exactly the three retired names (R-F1a). `git diff --stat -- src Makefile tools .github`:
  empty. ARM all-headers TU (184 headers): compiles clean under the gate flags; PROS-free and sim
  layering greps: empty.
- **2026-09-24 14:02:11 EDT** — **Documentation, in this change** (the brief's §5 F1 list):
  - `docs/api/` regenerated: `drivetrain_calibrator.md` (changed), `calibrate_panel.md` (NEW),
    `README.md` + `all-entities.md` (indexes), `mkdocs.yml` (+1 nav line, written by the generator).
    **A brief assumption that is FALSE in the tree:** the brief lists `docs/api/drive_plant.md` —
    there is no such page, by design: `tools/api_doc_tool.py`'s `TARGET_EXCLUSIONS` excludes
    `include/shulib/sim/` ("test-only — enforced by the sim-layering CI guard"). The plant's new rules
    are documented in its header and in HA-137. Not a stop: nothing to generate. Coverage gate:
    `2597 public entities across 148 headers, all documented`.
  - `docs/hardware-assumptions.md`: the status note (2F-1 added no entry, rewrote three); the table
    rows of HA-134/136/137; the entries HA-134 (the ramp and the classification; the first bench
    session's evidence stated as NOT settling it; **and a correction found on the way: the entry's
    heading said one side forward "turns the robot toward THAT side" while its own claim — LEFT
    forward → clockwise — turns it toward the OTHER side; the table row said "about the other"; the
    heading now matches**), HA-136 (every number added, removed and changed, the new precondition,
    and the screen model's estimates), HA-137 (coast-follow, the slipping member, why the calibrator
    no longer rests on the pinned wheel, and the physics note that an ideal free-rolling omni side
    would turn the robot MORE than a pivot — a negative κ the field does not allow); HA-130's settle
    line (the spread is over the one push now). Two phrases that named this log were caught by
    `check-removability` and reworded (the register is public).
  - `docs/changelog.md`: a "Changed since 2.2" section — unfrozen, no bump; what a caller must change.
  - `docs/internal/chunks/R3d-drivetrain-self-calibration.md`: the AMENDMENT at T3, one-line pointers at
    T4, T7, §4.2 and §4.3.
  - `docs/internal/build-order.md`: the R3d status line (2026-09-24) and the FIRST bold `**Next:**`
    (2F-2 waits for the go; the bench session resumes after F2; G1 Part 2b after that) — the previous
    Next kept as history under "Previously", its stray closing `**` removed so the bold still pairs.
  - `docs/roadmap.md`: the "Now (2026-09-24)" pointer and the bench-session paragraph (the library
    half landed host-only; the program half waits for the go).
  - `docs/internal/HANDOFF-remaining-work.md` S1: one superseded-in-part note (it described a tape push
    and a yaw-floor refusal on a wheels-up run — both false after 2F-1).
  - The briefing: regenerated LAST (below).
  **Deliberately NOT touched, with why:** `src/` and the worksheet (F2's); `README.md` (its line
  "calibration/ — the drivetrain self-calibrator, its SD-card record, the boot check" is incomplete now
  — no screen model — but not false; F2 is when a program uses the panel); `docs/guide/` (no chapter
  describes the push/pulse procedure — the calibration chapter is Part 3's, written from a followed
  procedure); the roadmap's "R3d Part 1 LANDED 2026-09-18" paragraph (a dated landing record, true for
  its date); `calibration_record.hpp`, `boot_check.hpp` (unchanged — `kRecordVersion` still 1).
- **2026-09-24 14:02:11 EDT** — **Prose review (rule 4), grep RUN before this entry**: `grep -rnE
  'SideAssign|ScalePush|Distance24|Distance48|Distance72|pulseVolts|spinVolts|tape push|twitch|STAND
  CLEAR|PushCapture'` over `docs/`, `README.md`, `include/`, `src/` (90 hits before the doc edits),
  plus `%.44s|44 char` for the brief's "44". Every hit read. **Fixed** (include/ and docs/): the HA-136
  field list and its "pulseVolts/spinVolts inherit" sentence; HA-130's "SPREAD over the tape push";
  the handoff's S1 line; the R3d spec (amendment + pointers, not a rewrite of history);
  `calibrate_panel.hpp`'s ScalePush comment (it said "drawn as the pulses would be" — it is grouped
  with the spin). **Unchanged, true:** `twitch`/`twitchy` in the stick-shaping and monitor comments,
  `docs/guide/03`, `roadmap.md:1635`, `build-order.md:1408`, T2's brief (driver feel, unrelated); the
  Part 2F brief itself; the historical Part 2 brief (line 249, "the tape push distances" — a record of
  that part's scope) and the 2026-09-18 session log. **LISTED, NOT EDITED — false now, owned by F2:**
  `src/calibrate_program.cpp` :11-12 (claims every screen line goes to serial), :18-19 and :663-664
  ("the first run is WHEELS UP … expected to refuse at the yaw floor" — a wheels-up run now refuses
  ROLLED/STILL after the ramp), :185-219 (the distance buttons, now never offered; the button row
  itself), :415-418 (the "1/4 … 4/4" phase words — there is no TAPE phase), :430 (the `%.44s` cut —
  every prompt now fits it), :441-455 (the table's `tapeDeltaRad` column, always 0), :500-504 ("Wheels
  up, a yaw-floor refusal is EXPECTED"), :753 ("spread over the tape push"); the worksheet
  `R3a-BENCH-WORKSHEET.md` Station G :688, :706, :709, :728 and every quoted prompt (the one push, the
  ramp, LEFT OUT, RUN AGAIN, the wheels-up expectation).
- **2026-09-24 14:07:06 EDT** — **Final verification, run in this order on the finished tree (every test object rebuilt
  first, then the briefing regenerated LAST, then the launch prompt's command set):**
  `cmake --build build/test -j` → exit 0, doc gates: `doc coverage: 2597 public entities across 148
  headers, all documented`, `doc example scan: 408 quoted lines, all verbatim`, `removability: no
  public doc references docs/internal/`, `briefing status: generated block current`, staleness audit
  clean; suite `test cases: 1416 | 1416 passed | 0 failed | 3 skipped`, `assertions: 2386636 |
  2386636 passed`. `src_build_gate.py self-test` → OK (33 cases); `check` → PASS, all six builds,
  0 disallowed warnings, `tank-calibrate` included. ARM TU over all 184 headers → exit 0. PROS-free
  grep → empty; sim-layering grep → empty. `api_doc_tool.py generate && check-fresh && briefing_status.py
  check && check-removability` → exit 0. `git diff --stat -- src Makefile tools .github` → empty. C7
  removability grep over the public docs → 0 hits. **Against the baseline, on this binary:** every
  pre-existing case outside the calibrator/panel/plant-coupled files identical in assertion count and
  printed values (1367 of 1368), the one difference `pros_adapter_fence_test`'s header walk (495 →
  498: one more header); plant cases 18–20 identical; calibrator file 18 cases / 54436 assertions →
  30 / 15869 (rewritten; the old case 21's per-tick REQUIRE was most of the old count); panel file new,
  12 cases / 786455 assertions (P1 alone checks every view of seven runs); plant-coupled file 3 / 4025 →
  9 / 4097.

### Definition of Done — Part 2F-1 (the brief's §4 F1), with evidence per line

- [x] **§1 in full.** F1.1 one push (`finishPushCapture`, cases 4, 4b, 5, 10, 11, 29); F1.2 the ramp and
      the classification (`finishPulse`, cases 6, 7, 9, 22–26); F1.3 the spin's two guards and step-up
      (`tickSpinTest`, cases 27, 28, 15); F1.4 the plant (DrivePlant 21–26; the calibrator rig at kS 1.2
      with κ stated per case, κ = 0.9 and 1 in case 25); F1.5 the numbers, `pushReady()`, `Refusal` +
      `refusalPort()` on every refusal (`requireRefusal` asserts code, port, safe state and ASCII/no-'%'
      in every refusal case), no '%' (case 30 reads the source), the serial record (case 31); F1.6 the
      panel (P1–P12). **With two measured caveats:** the spin's "≤ 2 s at each level" fires on the first
      tick AT OR AFTER 2 s — 2.01 s / 4.02 s on the FakeClock, whose summed 0.01 s steps put tick 200 at
      1.99999…; and the wrong-way stop lands at 15° plus at most one tick of spin (15.53° measured).
- [x] **Every public member documented** — the coverage gate, 2597 entities / 148 headers.
- [x] **ARM gate** — 184 headers, exit 0.
- [x] **PROS-free / sim-free guards** — both greps empty; `calibrate_panel.hpp` includes only the
      standard library and the calibrator.
- [x] **Every required case with its mutation RUN on a fresh binary and observed red** — the nine
      (flip the side rule; drop the yaw-rate rest; drop the decoupled classification; drop the
      wrong-way guard; drop the spin-start check; draw DONE before ready; move ABORT; FRONT where DONE
      was; drop the arming delay) and sixteen more, every one with all 156 objects rebuilt, restored
      byte-identical and relinked green (the reds are quoted above). Two mutations were first WRITTEN
      wrong (they tripped `-Werror`, not the property) and were rewritten; one test (P2) was
      STRENGTHENED after a mutation came back green; four plant mutations came back green through a
      stale-object artifact of MY driver, which was then fixed and every mutation re-run.
- [x] **Every pre-existing suite bit-identical except the calibrator's** — above; the calibrator's
      rewritten cases: 4, 4b (bracketing now on a full 48 in push), 5 (BACK-first now keeps the scale),
      6, 7 (+ the rolling contradiction), 8, 9 (wheels-up now refuses ROLLED naming the FLOOR — the train
      moves, the robot does not), 10 (one push; a port dropping off the bus replaces the frozen member),
      11 (refused at DONE, before anything powered), 12, 13, 14, 15 (the timeout case now a spin that
      starts then jams; the never-starting spin is case 28's), 16, 17 (`ScalePush` out of the phase
      list), 18 (+ "pct"), 19 (+ the new preconditions; the old kS 1.2 full run is now the default rig),
      21 (two waiting steps; the retired distances refused everywhere) — every one at kS 1.2 (why: the
      file header). New: 22–33.
- [x] **`src/` byte-identical** — `git diff --stat -- src` empty; the old program builds through three
      retired names (`Phase::ScalePush`, `Answer::Distance24/48/72`, `PortObservation::tapeDeltaRad`),
      which **F2 removes**.
- [x] **The documentation** — the list above; the briefing regenerated last.
- [x] **The log** — this section.

**NOT done, and not claimed:** F2 (the program as a renderer of the panel, RUN AGAIN's rebuild, the
serial lines actually printed, Station G rewritten); any hardware claim — nothing here has run on a
robot, no screen text has been seen on a V5, and every budget, height and pad in the panel is an
estimate until the SCREEN RULER is read; a missed tick is tested during the SPIN (case 16) and holds for
the ramped pulses by construction (the check precedes the phase dispatch), not by its own case; which κ
robot two shows (the physics note above) is for its serial record to say. **For the coordinator:** the
four "GREEN" plant mutations were a stale-object artifact of `/fast` (it does not track header
dependencies here) — any verification that relinks with `/fast` after a header edit must touch every
`test/*.cpp` first, or it can run a binary that mixes old and new inline code.

## Part 2F-2 — 2026-09-24 15:23 EDT

The executor for Part F2 (the brief `R3d-part2f-calibrate-on-the-robot.md` §2): `src/calibrate_program.cpp`
becomes a renderer of F1's `viewFor()` through `TouchGate`; RUN AGAIN; the serial record printed; F1's three
retired names removed; Station G rewritten from the new code. Go given by the team lead on 2026-09-24. Nothing
committed, nothing uploaded, no `pros` command run.

- **15:23** — log opened before any code (HEAD dc26b8d, tree clean). Reading next: RESUMING, the brief,
  the Part 3 and Part 2F-1 logs, the screen investigation, the code.
- **15:43** — **Required reading done:** RESUMING.md; the Part 2F brief (§0, §1 for the API, §2, §3,
  §4 F2, §5 F2, §6, §7, §8); this log's Part 3 and Part 2F-1 (its rulings R-F1a..f, the retired
  names, the stale src/ and worksheet lines it listed at 14:02); the screen investigation
  (`calib_invest.json`, second entry, every fact); `calibrate_panel.hpp` (all), `drivetrain_calibrator.hpp`
  (all), `src/calibrate_program.{hpp,cpp}` (all), `src/main.cpp` (the calibrate `#if` chain, its boot
  banner, `opcontrol()`, the precondition handler — it throws `PreconditionError`), `src/bench_r3a.cpp`
  (the screen helpers, the ruler, MOTOR WATCH, DRIVE, the IMU two-box question), `include/pros/screen.h`
  (no format attribute on `screen_print_at`; enums in `pros::`, functions in `pros::c::`;
  `E_TOUCH_ERROR` reads carry −1 in every field), the Makefile's `WARNFLAGS` (`-Wall -Wextra
  -Wformat=2`, no `-Werror`), `tools/src_build_gate.py` (ALLOWED = `{-Wunused-function}` only),
  `test/calibration_test_rig.hpp`, `test/calibrate_panel_test.cpp` (P1–P12's helpers), the calibrator
  cases that name the retired names (≈:374, :920, :1091, :1110, :1137, :1147), and worksheet Station G
  (:597–:790).
- **15:43** — **Baseline, clean tree `dc26b8d`, every object rebuilt** (`cmake --build build/test`,
  `Linking CXX executable shulib_tests` seen): `test cases: 1416 | 1416 passed | 0 failed | 3 skipped`,
  `assertions: 2386636 | 2386636 passed` — F1's final numbers. Per-case XML kept in the scratchpad
  (`f2/baseline.xml`) for F1's `cases.py` comparison at the end.
- **15:43** — **Findings from the reading that shape the work (none a stop):**
  1. `calibrate_panel.hpp:771` names `Phase::ScalePush` (`case Phase::ScalePush: // RETIRED`). Removing
     the enumerator (deliverable 4) forces deleting that one case label — an edit to the panel header
     BEYOND its budget constants. Forced, not chosen; logged as ruling F2-R1 below.
  2. The pre-run refusal path puts `e.what()` of a `PreconditionError` on the SCREEN, and the hal/pros
     adapters' precondition messages contain UTF-8 em dashes (`hal/pros/motor.hpp:102,105`,
     `imu.hpp:108`, `block_sink.hpp:83`, `block_source.hpp:75`) — non-ASCII that VEXos's fonts cannot
     draw. The panel's own lines are proven ASCII by the host tests; text the PROGRAM adds is not. So
     the renderer gets ONE print function that sanitizes (non-ASCII → `?`, an em/en dash → `-`, `%` →
     `?`) and cuts at the size's budget before the literal `"%s"` reaches `screen_print_at`.
  3. `print_at` leaks its buffer per call (investigation), and a diffing renderer at the 10 ms tick
     would redraw a live number up to 100 times a second — ~18 KB/s of leak with three live lines.
     So an IN-PLACE text change of an unchanged-colour line is limited to 5 Hz per line; a layout,
     colour, header or button change is drawn on the same tick. (Numbers: below, with the renderer.)
  4. `src/calibrate_program.hpp` says "It has NEVER RUN ON HARDWARE" — false since the first bench
     night (the Part 2 program ran five starts); `src/main.cpp`'s calibrate boot banner says "THIS
     PROGRAM HAS NEVER RUN ON HARDWARE" — true of THIS build (the rewrite has not run) but misleading.
     The `.hpp` is mine to fix; `main.cpp` is not in scope beyond the entry — listed, not edited (below).
- **15:43–15:50** *(appended between two `date` reads)* — **THE CHARACTER BUDGETS — DERIVED FROM THE TREE, NOT MEASURED.** The team lead cannot run
  the SCREEN RULER (Bench Tests button 9: *"i cant do 9 so just assume based on the other tests"*), so
  the budgets come from what the tree records being READ on a V5 screen. The evidence gives LOWER
  bounds only — text that was seen whole — and no measured upper bound; no record anywhere says a line
  ran off the right edge (the one "cut" line, the old prompt at 44, was our own `%.44s`, not the font).
  Lengths computed, not counted by eye:

  | Size | The widest line the tree records as READ on a V5 | Chars, from x | What it proves | Other evidence | Budget |
  |---|---|---|---|---|---|
  | SMALL | Bench Tests' scrolling log: every station's lines cut at `kScreenCols = 54` (`bench_r3a.cpp:206`) and drawn from x = 8 — the census, SD probe and MOTOR WATCH screens read on robot two (Part 3; the coordinator's statement) and on the bench bot every R3a session | 54, x 8 | ≤ 8.7 px/char; no complaint of a cut right edge in any session | the old calibrate header ran two SMALL strings into each other at y = 20 (the investigation) — an overlap of OUR placement, not a width fact | **50 (F1's, kept)** — 4 under the evidence |
  | MEDIUM | Bench Tests' station 10 DRIVE row `CEILING 12 V   L  +0.0 V   R  +0.0 V` (`bench_r3a.cpp:1601`, printed at :1606), read to its LAST field (`R +0.0 V`) in photographs of the panel (R3b-PROGRESS §10.4, "at any screenshot"; the controller LCD's copy has no `V`, so the quote is the brain's) | 36, x 8 | ≤ 13.1 px/char | the Bench Tests splash `BUILD Aug 19 2026 13:28:57` (26, x 10) read to its seconds (R3a-PROGRESS §20.3; that the splash, not the header's SMALL copy at x 250, was the one read is the likelier reading, not certain); MOTOR WATCH's right-column cells `L11 DOWN    -373` (16, x 248); the old calibrate prompt, 44 MEDIUM chars from x 8 — the one long MEDIUM line with a complaint ("I CANT READ IT"), cause unknown | **36 (F1's, kept)** — exactly the widest line proven whole; not raised toward 44, which is the line that drew the complaint |
  | LARGE | the IMU rotate test's live number `  +87.24 deg` (`bench_r3a.cpp:703`), read on the bench bot ("80 plus", R3a-PROGRESS §18.5) | 12, x 8 | ≤ 39 px/char — weak | `FIELD CONTROL CONNECTED` (23, x 10) "showed, on screen" on robot two (R3b-PROGRESS §9.3) — the record does not say it was whole; `Bench Tests` (11) on the splash | **18 (F1's, kept)** — the LEAST supported budget: above the 12 proven whole, below the 23 reported shown |

  **Ruling F2-R2 — keep all three of F1's constants** (`kLargeCols` 18, `kMediumCols` 36, `kSmallCols`
  50). The evidence supports each (SMALL and MEDIUM sit at or under the widest line proven whole), and
  none of it supports a different number. Alternatives rejected: **LARGE 12** (the only width PROVEN
  whole) — the panel's own literals need 17 (`WRONG MOTOR MOVED`) and its live numbers 16, so 12 would
  fail `panelLiteralsFit()` and force rewording `calibrate_panel.hpp`'s text, which is F1's and not
  this part's to change on no new evidence; **MEDIUM 44** (what the old prompt drew) — that is the
  line the team lead could not read. **Not evidence, recorded as such:** if VEXos's text sizes are its
  usual mono fonts (the investigation's unverified inference: MEDIUM ≈ 10 px wide, LARGE ≈ 20 px),
  every budget has 25–40 % of margin; if LARGE were a 60 px font, 18 would run off — nothing in the tree
  rules that out except the 36 px erase band `screenBig()` gives a LARGE line, which is a guess of the
  same kind. **Bench Tests button 9 (SCREEN RULER) settles all three in one minute; until it is read,
  every budget, line height and position in `calibrate_panel.hpp` is an estimate** — the worksheet says
  so in Station G.
- **15:50** — **Design read-out before any code (rulings beyond the brief, each with its alternative):**
  - **F2-R1 — the retired `case Phase::ScalePush:` label in `calibrate_panel.hpp:771` is deleted with
    the enumerator.** Forced by deliverable 4, not chosen; the panel's text, layout and logic are
    otherwise untouched. Alternative: keep `Phase::ScalePush` — rejected, the brief says F2 removes it.
  - **F2-R3 — no new `src/` file.** The renderer, the program's own screens and the serial record stay
    in `src/calibrate_program.cpp`. A new TU would be compiled into ALL SIX builds (the Makefile builds
    every `src/*.cpp`; the gate reports "4/4 TUs") for a renderer only one program uses, and would move
    the gate's per-build profile for no reader's benefit. Alternative: `src/calibrate_screen.cpp` —
    rejected on those grounds; the file is organised in sections instead.
  - **F2-R4 — the renderer diffs against what is PAINTED.** A change of screen or of any line's place or
    size is a full repaint (one erase of the content area, then everything). Otherwise: the header when
    its colour or word changes; a line whose COLOUR changed at once; a line whose TEXT changed at most
    every 0.2 s (5 Hz); a button that changed (appeared, armed, moved) at once; a button that vanished
    is erased. An in-place text change never erases: the new text is printed padded with spaces to the
    old text's length, which covers the old cells because VEXos draws text pen-on-ERASER (an opaque cell
    — `bench_r3a.cpp:1912`, a hardware finding) — so a live number never flickers. The leak arithmetic
    that sets 5 Hz: `print_at` leaks roughly its string plus a heap header per call (the investigation);
    three live lines at the 10 ms tick would be ~300 prints/s, at 5 Hz 15/s — a three-minute run leaks
    on the order of 150 KB instead of ~3 MB, against a ~48 MB heap, and RUN AGAIN repeats runs in one
    program. The program counts its prints and reports the count at every outcome. Alternatives: a full
    10 Hz repaint (the old program; flicker + the leak) — rejected; no rate limit — rejected (the leak).
  - **F2-R5 — one print function.** Every character reaches `pros::c::screen_print_at(size, x, y, "%s",
    buf)` through `printText()`, which copies into a buffer of the size's budget + 1, replacing `%` and
    control bytes with `?`, a UTF-8 em/en dash with `-`, any other non-ASCII character with ONE `?`, and
    cutting at the size's budget. The panel's lines are already proven clean by the host tests; the
    program's own text (a precondition's `e.what()`, which carries em dashes) is not, and this is the
    one place that makes it so.
  - **F2-R6 — the touch baseline must come from a GOOD read.** `TouchGate` is constructed with the
    release count of the first read that is not `E_TOUCH_ERROR` (up to 20 tries at start); if none is
    good, the first good read inside the loop rebuilds the gate and is never acted on. Why: a gate
    built from an error read's −1 would treat the first good read as a NEW release at stale x/y.
  - **F2-R7 — RUN AGAIN rebuilds three things, and only these:** the calibrator (`std::optional::
    emplace` + `start()`, as F1's header prescribes), the `TouchGate` (a new one, based on the count of
    the very release that asked for RUN AGAIN, so that release cannot replay), and the program's per-run
    record (printed counts, the save outcome, the outcome flag, the loop timing) — a value-initialised
    struct, so a field added later is reset by construction too. What survives is the program's, not
    the run's: the motor adapters, the IMU adapter (already calibrated), the clock, the pacer, the three
    stated facts, the painter (invalidated, so the next frame is a full repaint).
  - **F2-R8 — a `PreconditionError` inside the loop is caught ONCE at the loop's edge**: every motor to
    0 V + Coast, a PROGRAM FAULT screen with the message, the serial line, then the same idle-until-
    disabled as before. Before, it unwound out of `opcontrol()` (the guard still stopped the motors, but
    the screen said nothing). Alternative: let it propagate — rejected, the operator would see a frozen
    screen.
  - **F2-R9 — a field disable on a FINISHED screen exits without `abort()`.** `abort()` from Done
    discards the result (F1's contract); the run is over, the motors are already at 0 V. A disable while
    running aborts as before.
  - **F2-R10 — serial additions beyond the brief's list, each for reading a capture without guessing:**
    the config the run used (one line, at start); a `screen:` line whenever the LAYOUT changes (what the
    operator was looking at); every touch with the button it landed on and why it did or did not act
    (greyed / not accepted now / nothing); the push's live count on change (≤ 4 lines/s); the loop's
    worst gap between two ticks at every outcome (rule 7 refuses past 100 ms while powered — this is
    the number that would say whether a MISSED_TICK was the program's fault).
- **16:25** — **The three RETIRED names REMOVED** (deliverable 4), with every reference:
  `drivetrain_calibrator.hpp` — `Phase::ScalePush` (and its `updateStatus()` case and the header's
  phase-list line), `Answer::Distance24/48/72` (and their `answerWord()` cases and the `accepts()`
  doc's "retired" clause), `PortObservation::tapeDeltaRad`; the header's 2F-1 section gains one
  line saying 2F-2 removed them; `CalibratorStatus::prompt`'s doc no longer says "the width the Part 2
  program prints" as if that program still printed it (the 44 bound is kept — case 33 pins it — and
  now says why it exists). `calibrate_panel.hpp:771` — the `case Phase::ScalePush:` label (F2-R1).
  Tests: `drivetrain_calibrator_test.cpp` case 4 (the ten `tapeDeltaRad == 0` checks dropped), case
  17 (comment), case 19 (the out-of-order touch is now FRONT FIRST while the push is watched — the
  same refusal on the live surface, with a REQUIRE that the push IS being watched, and the reason
  checked for "FRONT FIRST", "waiting for DONE" and "nothing was recorded"), case 21 (the sweep is
  every Answer there is; the idle check asks FRONT and BACK). `docs/api` regenerated
  (`drivetrain_calibrator.md`, `calibrate_panel.md`, `README.md`, `all-entities.md`: 2597 → 2592
  entities — the five names).
- **16:25** — **Host suite after the removal, every object rebuilt** (`touch test/*.cpp` +
  `shulib_tests/fast`, `Linking CXX executable shulib_tests` seen; the full target stops at the doc
  gates until the briefing is regenerated — last): `test cases: 1416 | 1416 passed | 0 failed | 3
  skipped`, `assertions: 2386604`. F1's `cases.py` against the baseline: **1415 identical, 4
  differ, each explained to the assertion:** case 4 85 → 75 (ten `tapeDeltaRad` checks), case 19
  25 → 27 (+ the REQUIRE, + one reason check), case 21 52 → 29 (2 steps × 3 retired answers × 3
  assertions, the 6 "never offered" checks, one idle check → two), P9 3982 → 3981 (P9 reads the
  panel header's SOURCE at run time and it has one line fewer). Total −32 = 2386636 − 2386604.
  Two back-to-back runs of the new binary: 1419 of 1419 identical (deterministic). *(An oddity run
  down on the way: the STALE binary, run after the header edit, printed 2386635 — P9 reads the
  header from disk at run time, so a stale binary sees the new source. Not a flake.)*
- **16:25** — **`src/calibrate_program.cpp` REWRITTEN** (795 → ~1370 lines; most of the growth is
  the header's explanation, the program's own checked literals, and the serial record) and
  `calibrate_program.hpp` (its hardware claim corrected: the Part 2 version ran five starts and
  finished none; THIS version has never run; the function's contract restated — it now returns only
  on a field disable or a pre-run refusal). One `make ROBOT=tank PROGRAM=calibrate`: exit 0, the
  only warnings the vendored `lv_obj_style.h` one and main.cpp's two allowed `-Wunused-function`;
  `calibrate_program.cpp` — which now includes `calibrate_panel.hpp` and so compiles `viewFor()`,
  `TouchGate` and `wrapWords` under src/'s `-Wall -Wextra -Wformat=2` for the first time — adds
  NONE. The full six-build gate is running.
- **16:25** — **A FINDING about F1's panel (not a stop, not changed here):** compiled on the HOST at
  `-O1` with glibc's `_FORTIFY_SOURCE` (my throwaway walk, below), `calibrate_panel.hpp:644`
  (`panelChips`: `std::snprintf(out, cap, "%s", buf)`) draws `-Wformat-truncation`: "output between
  1 and 268 bytes into a destination of size 51". A FALSE positive — the line is reached only when
  `used <= kMediumCols` (36) and `cap` is 51 — and the ARM src build at `-Os` (what the gate
  compiles) does not report it; nor does a probe TU calling `viewFor()` under `arm-none-eabi-g++
  -Wall -Wextra -Wformat=2` at `-Os`, `-O2` or `-O3` (newlib has no fortify object sizes, so the
  source array's bound never reaches the check). But the gate FAILS on that category, so a compiler, flag or
  inlining change could turn it into a gate failure with no code change. The fix, if it ever fires,
  is a bounded precision (`"%.*s"` with the known length) or a `memcpy`; it is F1's header and
  this part's brief confines it to its budget constants, so it is recorded for the coordinator.
- **16:27** — **Six-build gate on the rewritten program:** `src_build_gate.py self-test` → `OK (33
  detector cases, each against a real build)`; `check` → `src build gate: PASS (all 6 builds link
  ...)`, every build `exit 0, 4/4 TUs compiled, warnings: 2 allowed (-Wunused-function), 3
  vendor-ignored, 0 disallowed` (xdrive 0 allowed), beacons checked — the SAME profile as F1's
  pre-change run (`f1/gate_check.log`). `bin/` and `.d/` are the gate's (gitignored).
- **16:48** — **Station G REWRITTEN** (`R3a-BENCH-WORKSHEET.md` :597 → the DO NOT list; 225 lines in, 140
  out): the purpose; what has run and what has not (the Part 2 program's five starts; THIS version
  never; the budgets DERIVED, not measured, and what to write down if a line runs off the edge);
  robot two tonight (OMNI wheels both sides; port 19 unplugged → LEFT 11–15, RIGHT 16 17 18 20;
  port 18 short); G.0 the one 48 in tape and deciding the FRONT before any push; G.1 the build
  (the BUILD time on the top bar); G.2 census/MOTOR WATCH for nine motors; **G.3 the wheels-up run
  OPTIONAL** (and what it now stops with: `IT ROLLED` or `NOTHING MOVED` after 3/6/9 V, not a
  yaw-floor refusal); **G.4 the run**, a table of every screen with its top-bar word, its exact text
  and what to do — the ONE push, **the DONE rule** (appears only when ≥ 2 turned, one wheel turn of
  median travel, and everything still), the drift going red `-- TOO CURVED`, the pulse levels and
  what a jerk looks like (2° ≈ a quarter to a third of an inch 7–10 in from the turning point; the
  arithmetic: 0.0349 in per inch), **the spin and its two safety stops** (15° wrong way; 5° in 2 s at
  3 V then 6 V) and the 20 s timeout; **LEFT OUT and what it means** (port 18's slipping gear; the run
  saves without it; Teleop's boot calls the extra motor a warning); what each red headline means;
  **RUN AGAIN** (and that every SAVED run replaces the record, a stopped one does not); **G.5 "how to
  know it calibrated correctly"** — SAVED; the MEASURED line; ports per side against LEFT 11–15 /
  RIGHT 16 17 18 20; `ipr` ≈ 1.375 for a 2.75 in wheel 1:1 with L ≈ R, and a different value read
  as a ratio (ipr ÷ 1.375); the track width ≈ the taped wheel centre-to-centre or somewhat larger
  (scrub; omni scrubs less); a second run within 2 % / 3 %; G.6 Teleop from the record with the new
  **G.6.4: `est x` ≈ 48 after a 48 in drive along the tape**; G.7 every serial line a run prints, in
  order, with host-plant examples labelled as such. **Every backtick-quoted string in the station
  was checked by a script against the source** (the panel, the calibrator, the program, main.cpp,
  the boot check, the description): 86 found verbatim; the other 51 are printf-formatted lines,
  each checked by hand against its format (e.g. `PORT 12  6 V` ← `"PORT %d  %.0f V"`, `3 of 9  yaw
  -1.2 of 2.0 deg` ← `"%u of %u  yaw %+.1f of %.1f deg"`, the MEASURED line ← `describe()`) or shell
  commands. Two stale Teleop quotes inside Station G fixed on the way: G.6.3's `5 members 0
  disagree` (the row now reads `5/5 present 0 disagree`, main.cpp:826) and G.6.2's source row
  (`MEASURED record | DEGRADED: dead ...`, main.cpp:868 + boot_check.hpp:300).
- **16:48** — **The rest of the documentation:** `docs/changelog.md` — a "Removed since 2.2" section
  above F1's (the three names, a migration each; unfrozen, no bump); `docs/internal/build-order.md` —
  the R3d status line "2026-09-24 (later)" and **the FIRST bold `**Next:**`: the bench session
  resumes — rebuild and upload shulib Calibrate from the committed tree and run Station G; G1 Part 2b
  follows, its own go** (the old Next kept under "Previously", its bold markers re-paired: 6/6/8 per
  line, even); `docs/roadmap.md` — the "Now" pointer and the bench-session paragraph (the program
  half landed, build-checked, not run; budgets derived); `docs/hardware-assumptions.md` — HA-136 one
  sentence on how the budgets were checked (derived lower-bound evidence, still settled by the
  ruler) and the register's status note; `README.md` — `calibration/` now lists its screen model (F1
  had listed this as incomplete-not-false, to fix when a program used the panel — one does now);
  `docs/internal/HANDOFF-remaining-work.md` S1 — F1's "Station G is rewritten in 2F-2" now past
  tense, the wheels-up run optional; `docs/api` regenerated (above). The briefing: regenerated LAST.
- **16:48** — **Prose review (RESUMING rule 4) — grep RUN before this entry:** `grep -rnE
  'SideAssign|ScalePush|Distance24|Distance48|Distance72|tapeDeltaRad|\bTAPE\b|twitch|STAND
  CLEAR|wheels up|wheels-up|WHEELS UP|yaw floor|yaw-floor'` over `docs/ README.md include/ src/`
  (203 hits, saved in the scratchpad `f2/prose_hits.txt`), plus `'%\.44s|44 char|at most 44|44
  characters'` for the brief's "44", plus `'shulib Calibrate|calibrate_program|PROGRAM=calibrate'`
  over README, `docs/*.md`, `docs/guide`, `docs/api`. Every hit outside the historical logs read.
  **Fixed:** the HANDOFF S1 note (tense; wheels-up optional); README's `calibration/` line;
  `CalibratorStatus::prompt`'s doc (the 44 bound's reason); the calibrator header's phase list and
  its 2F-2 line; F1's src list at its 14:02 entry — every line it named is gone with the rewrite
  (`:11-12` the serial claim is now true and itemised; `:18-19`/`:663-664`/`:500-504` the
  wheels-up yaw-floor expectation — gone; `:185-219` the button row — gone; `:415-418` the 1/4..4/4
  words — gone; `:430` the `%.44s` — gone; `:441-455` the `tapeDeltaRad` column — gone; `:753` the
  "tape push" spread — now "push"); F1's worksheet list (:688, :706, :709, :728, the quoted prompts) —
  the station is rewritten. **Unchanged, true:** `SideAssign` (a live enumerator) everywhere;
  `STAND CLEAR` and `twitch` in the panel's literals, the calibrator's prompts, the worksheet and the
  generated API pages (the words on the screen); `twitch`/`twitchy` in the stick-shaping, input-shaping
  and coupled-monitor comments and `docs/guide/03` (driver feel — unrelated); `wheels up` in Stations
  D/E/F, `src/bench_r3a.cpp`, `src/drive_program.hpp`, `src/main.cpp:971` (Teleop's boot text),
  `docs/roadmap.md:195-196,250` and `hardware-assumptions.md:1096` (the DRIVE station, Teleop and the
  coupled monitor — not Calibrate);
  `roadmap.md:275` (R3d Part 1's dated landing record); "yaw floor" in the calibrator's 2F-1 notes
  (true); the changelog's F1 section naming the retired names as retired (a dated record, with the
  new "Removed" section above it saying they are gone); `docs/api`'s banner copy of the calibrator
  header's 2F-2 line (generated). The historical logs and briefs (R3d-PROGRESS, the Part 2F brief, the
  Part 2 brief, R3b logs, the 2026-09-18 session log) are records and were not edited.
  **Listed, NOT edited (outside this part's files):** `src/main.cpp`'s calibrate boot banner says
  "THIS PROGRAM HAS NEVER RUN ON HARDWARE" (true of this build, but the Part 2 program ran — the
  program's own banner now says both), and the AUTONOMOUS state screen `main.cpp` paints for the
  calibrate build prints a 120-character `why` in SMALL from x 10, which will run off the right edge
  — both `main.cpp`'s, which this part may touch only for the calibrate entry.
- **17:04** — **EVERY STRING THE PROGRAM CAN DRAW, per screen, with its size, length and budget.** Sources:
  the panel's named literals (`panel_text::kAll`, printed with their budgets by a throwaway), the 21
  STOPPED wordings (`stoppedWords()`, same throwaway), the program's own literals (`text::kAll`, each
  `static_assert`ed), and — for the FORMATTED lines — the longest text each line slot actually showed
  over seven scripted runs on the host plant (the throwaway walk below; every slot's maximum is within
  its budget, checked). Three slots sit exactly AT their budget and are the first to watch on the
  robot: the pulses' side chips (36/36 MEDIUM), the longest STOPPED "why" (36/36 MEDIUM) and a wrapped
  reason line (50/50 SMALL); and one of the program's own, `NOT DRIVER CONTROL` (18/18 LARGE).
  Every pixel row: the text lines' estimated bottoms end at 227 (a SMALL line at 212 + 16), the button
  row at 234, the program's own screens at 224 — all under 236.

  | Screen | String (live ones: an example) | Size | Len | Budget | Note |
  |---|---|---|---|---|---|
  | every screen (top bar, right) | `BUILD hh:mm:ss` | SMALL | 14 | 50 | program; x 340 |
  | Starting (program) | `CALIBRATE` | MEDIUM (bar) | 9 | 24 |  |
  | Starting (program) | `HANDS OFF` | LARGE | 9 | 18 |  |
  | Starting (program) | `STARTING UP` | LARGE | 11 | 18 |  |
  | Starting (program) | `finding the motors and the IMU` | MEDIUM | 30 | 36 |  |
  | ImuWait | `1/5 IMU` | MEDIUM (bar) | 7 | 24 |  |
  | ImuWait | `HANDS OFF` | LARGE | 9 | 18 |  |
  | ImuWait | `IMU WARMING UP` | LARGE | 14 | 18 |  |
  | ImuWait | `0.4 s  (live, '%.1f s')` | LARGE | 5 | 18 | max seen 5 |
  | ImuWait | `NEXT: push it 48 in along the tape` | MEDIUM | 34 | 36 | formatted from pushDistance |
  | Push | `2/5 PUSH` | MEDIUM (bar) | 8 | 24 |  |
  | Push | `PUSH IT FORWARD` | LARGE | 15 | 18 |  |
  | Push | `48 IN ALONG TAPE` | LARGE | 16 | 18 | formatted |
  | Push | `9 of 9 turned  (live, '%u of %u turned')` | LARGE | 15 | 18 | max seen `10 of 12 turned`; worst `21 of 21 turned` = 15 |
  | Push | `drift +0.3 deg  /  drift -10.0 deg -- TOO CURVED` | MEDIUM | 29 | 36 | max seen 29 |
  | Push | `DONE shows once the motors turn` | MEDIUM | 31 | 36 |  |
  | Push | `keep pushing -- the full 48 in` | MEDIUM | 30 | 36 | formatted |
  | Push | `let go -- DONE shows when it stops` | MEDIUM | 34 | 36 |  |
  | Push | `touch DONE -- then say which end` | MEDIUM | 32 | 36 | green |
  | WhichEnd | `3/5 WHICH END` | MEDIUM (bar) | 13 | 24 |  |
  | WhichEnd | `WHICH END FIRST?` | LARGE | 16 | 18 |  |
  | WhichEnd | `NEXT: motors twitch -- stand clear` | MEDIUM | 34 | 36 | amber |
  | Pulses | `4/5 PULSES` | MEDIUM (bar) | 10 | 24 | amber bar |
  | Pulses | `STAND CLEAR` | LARGE | 11 | 18 |  |
  | Pulses | `TWITCHING MOTORS` | LARGE | 16 | 18 |  |
  | Pulses | `PORT 12  6 V  (live, 'PORT %d  %.0f V')` | LARGE | 12 | 18 | max seen 12 |
  | Pulses | `settling -- hands off` | MEDIUM | 21 | 36 |  |
  | Pulses | `3 of 9  yaw -1.2 of 2.0 deg  (live)` | MEDIUM | 29 | 36 | max seen 29 |
  | Pulses | `L 11 12 13 14 15  R 16 17 20  OUT 18  (chips)` | MEDIUM | 36 | 36 | AT the budget (seen 36); longer falls back to counts `L 5  R 4  OUT 1` |
  | Spin | `5/5 SPIN` | MEDIUM (bar) | 8 | 24 | amber bar |
  | Spin | `STAND CLEAR` | LARGE | 11 | 18 |  |
  | Spin | `SPINNING` | LARGE | 8 | 18 |  |
  | Spin | `CCW 0.8/2 TURNS  (live)  /  SETTLING` | LARGE | 15 | 18 | max seen 15 |
  | Spin | `at 3 V  /  at 6 V -- stepped up` | MEDIUM | 20 | 36 |  |
  | Spin | `hands off -- it spins next / hands off -- it spins again` | MEDIUM | 27 | 36 |  |
  | Spin | `NEXT: it saves -- stay clear` | MEDIUM | 28 | 36 |  |
  | Saved | `SAVED / NOT SAVED / MEASURED` | MEDIUM (bar) | 9 | 24 | green / red / blue |
  | Saved | `LEFT 5  RIGHT 4` | LARGE | 15 | 18 | max seen 15 |
  | Saved | `ipr L 1.37500 R 1.37500` | MEDIUM | 23 | 36 | `%.5f` each |
  | Saved | `track 13.700 in` | MEDIUM | 15 | 36 |  |
  | Saved | `LEFT OUT 18  /  LEFT OUT: 3 PORTS` | MEDIUM | 11 | 18 | the note beside RUN AGAIN (kNoteCols) |
  | Saved | `NEXT: copy these, then RUN AGAIN to check` | SMALL | 41 | 50 |  |
  | Saved | `NOT SAVED -- copy these numbers NOW` | SMALL | 35 | 50 | red |
  | Saved | `the warning, wrapped (2 lines)` | SMALL | 48 | 50 | max seen 48 |
  | Stopped | `STOPPED -- MOTORS OFF` | MEDIUM (bar) | 21 | 24 | red bar |
  | Stopped | `the headline per code (21 codes)` | LARGE | 17 | 18 | longest `WRONG MOTOR MOVED` |
  | Stopped | `the why per code` | MEDIUM | 36 | 36 | longest `something kept moving before a pulse` — AT the budget |
  | Stopped | `the do per code` | MEDIUM | 35 | 36 | longest `push it again, STRAIGHT on the tape` |
  | Stopped | `PORT 11` | MEDIUM | 7 | 36 |  |
  | Stopped | `the reason, wrapped (≤ 3 lines)` | SMALL | 50 | 50 | AT the budget (seen 50) |
  | Stopped | `the full reason is on the serial log` | SMALL | 36 | 50 |  |
  | buttons (MEDIUM, centred) | `DONE / FRONT FIRST / BACK FIRST / ABORT / RUN AGAIN` | MEDIUM | 11 | 12 | label budget 12 |
  | Cannot start (program) | `CANNOT START` | MEDIUM (bar) | 12 | 24 | red bar |
  | Cannot start (program) | `CARTRIDGE UNSET / IMU COUNT NOT 1 / TOO FEW MOTORS / MOTORS REFUSED / IMU REFUSED` | LARGE | 15 | 18 |  |
  | Cannot start (program) | `the why, wrapped (≤ 3 lines)` | MEDIUM | 36 | 36 | wrapWords<36,3>; runtime text sanitised |
  | Cannot start (program) | `the fix, wrapped (≤ 2 lines)` | SMALL | 50 | 50 | wrapWords<50,2> |
  | Cannot start (program) | `nothing is powered -- fix it, then restart slot 4` | SMALL | 49 | 50 |  |
  | Cannot start (program) | `Bench Tests (slot 3) is the diagnosis tool / the full text is on the serial log` | SMALL | 42 | 50 |  |
  | Program fault (program) | `STOPPED -- MOTORS OFF / PROGRAM FAULT / every motor is at 0 V, coasting` | MEDIUM/LARGE/SMALL | 31 | 50 | + the message wrapped as above |
  | Not driver control (program) | `NOT DRIVER CONTROL` | LARGE | 18 | 18 | AT the budget |
  | Not driver control (program) | `It powers the motors, so it runs ONLY in` | SMALL | 40 | 50 |  |
  | Not driver control (program) | `DRIVER CONTROL, enabled. Motors: 0 V, coasting.` | SMALL | 47 | 50 |  |
  | Not driver control (program) | `status bits 0x0f field=yes disabled=yes auton=yes` | SMALL | 49 | 50 | worst case; buffer kSmallCols+1 |
  | Not driver control (program) | `Unplug any competition switch or field cable,` | SMALL | 45 | 50 |  |
  | Not driver control (program) | `and NEVER start it from the CONTROLLER's menu.` | SMALL | 46 | 50 |  |
  | Not driver control (program) | `Any result is on the serial log too ([R3D]).` | SMALL | 44 | 50 |  |

- **17:04** — **A full run as the operator would see it, generated from `viewFor()`** by a throwaway host
  program (`f2/walk.cpp` in the session scratchpad, not committed; it drives the SAME scripted operator
  and plant as the panel's host cases through a `TouchGate`, and prints every layout change, the frame
  before it, and the finished screen 0.6 s later). The clean run, condensed (host plant: ten drive
  motors + two idle, so `of 12`; robot two will show `of 9`):
  1. blue `1/5 IMU`: `HANDS OFF` / `IMU WARMING UP` / amber `0.0 s` / `NEXT: push it 48 in along the
     tape` — ABORT {330,180,474,234}, armed.
  2. blue `2/5 PUSH`: `PUSH IT FORWARD` / `48 IN ALONG TAPE` / amber `0 of 12 turned` / `drift +0.0
     deg` / `DONE shows once the motors turn` — ABORT only. Mid-push the NEXT line reads `let go --
     DONE shows when it stops`; at rest after 48 in (t = 8.18 s) the number turns GREEN `10 of 12
     turned`, the NEXT line GREEN `touch DONE -- then say which end`, and DONE appears at
     {6,180,300,234} **GREYED** (the arming delay), then green.
  3. blue `3/5 WHICH END`: `WHICH END FIRST?` / amber `NEXT: motors twitch -- stand clear` — FRONT
     FIRST {6,72,234,158} and BACK FIRST {246,72,474,158} both GREYED at first, ABORT armed. Neither
     box overlaps DONE's rectangle (the double-tap rule).
  4. AMBER `4/5 PULSES`: `STAND CLEAR` / `TWITCHING MOTORS` / `PORT 11  3 V` / `settling -- hands
     off` / (chips, empty until the first side is decided) — ABORT the whole row {6,180,474,234}. At
     the last port: `PORT 20  9 V` / GREEN `10 of 10  yaw +6.3 of 2.0 deg` / `L 11 12 13 14 15  R 16 17
     18 19`.
  5. AMBER `5/5 SPIN`: `STAND CLEAR` / `SPINNING` / `SETTLING` / `hands off -- it spins next` /
     `NEXT: it saves -- stay clear`, then `CCW 1.2/2 TURNS` / `at 3 V` while turning, then `hands
     off -- it spins again` before the CW half — ABORT the whole row.
  6. GREEN `SAVED`: GREEN `LEFT 5  RIGHT 5` / `ipr L 1.62500 R 1.62500` / `track 12.500 in` / SMALL
     `NEXT: copy these, then RUN AGAIN to check` — RUN AGAIN {246,112,474,158} GREYED, armed 0.6 s later.
  The other runs: **port 18 slipping** ends RED `NOT SAVED` (as told) with `LEFT 5  RIGHT 4`, amber
  `LEFT OUT 18` beside RUN AGAIN, red `NOT SAVED -- copy these numbers NOW`, and the warning wrapped
  whole: `port 18 turns but does not move the robot -- its` / `gear is slipping; it is left out`;
  the pulses screen showed `L 11 12 13 14 15  R 16 17 19  OUT 18` (36/36). **κ = 1 (rolls)** stops at
  port 11 after 3/6/9 V: RED `STOPPED -- MOTORS OFF`, red `IT ROLLED` / `it moved but did not turn`
  / `on the FLOOR? then report the port` / red `PORT 11` / grey `port 11 moved its gear train
  (others 49.8 deg) but` / `the robot turned only 0.00 deg even at 9.0 V` / `the full reason is on
  the serial log`. The curved push, DONE-before-any-push, BACK-first and spin-fight runs were walked
  too (their lines are in the table's maxima). **Prints, by the painter's rules** (a COPY of
  `PanelPainter`'s logic in the throwaway, PROS calls replaced by a counter — it measures the rules,
  not the program): the clean 52 s run **217 prints** (6 full repaints, 36 padded in-place updates),
  port 18's 48 s run 206, the rolled run 54 — about 4 prints a second, against ~300/s for a
  per-tick repaint of three live lines. On the robot the program reports its own count at every
  outcome (`loop: ... N screen prints so far`).
- **17:04** — **THE SERIAL RECORD, old vs new.** The banner (a script diffed every `emit` literal
  before the loop, `dc26b8d` → now):
  ```
  -#  THIS PROGRAM HAS NEVER RUN ON HARDWARE. It POWERS THE MOTORS.
  +#  THIS BUILD HAS NEVER RUN ON HARDWARE. It POWERS THE MOTORS. (The Part 2
  +#  program ran five starts on 2026-09-24 and finished none; this is Part 2F.)
  -EVERY THRESHOLD BELOW IS INVENTED (HA-136). Wheels UP on the first run: the pulses
  -turn nothing, and the run REFUSES at the yaw floor. That is the program WORKING.
  +config: push 48 in along a tape; a port MOVED past 5.0 deg of shaft; the push straight within ...
  +config: pulses ramp 3.0/6.0/9.0 V for 0.40 s each, a side needs 2.0 deg of yaw; ...
  +config: spin 2 turns each way, levels 3.0 then 6.0 V; must turn 5.0 deg within 2.0 s ...
  +ONE PUSH, 48 in, forward, along a tape; then FRONT or BACK; then STAND CLEAR: one
  +motor at a time jerks (3, then 6, then 9 V), then it spins both ways. INVENTED numbers.
  +WARNING: every touch read at start was an ERROR read -- ...   (only if it happens)
  +===== RUN 1 =====
  ```
  (unchanged: the stamp, the hash, the census, the cartridge belief, the IMU lines). **Added in the
  run** — the lines start B of the first night lacked: `phase: <STEP> | <progress>`; `screen: <top bar>:
  <first three lines> | [buttons]` on every layout change; `touch: x=.. y=.. release=.. on <button> ->
  ANSWER <x> | RUN AGAIN | ignored -- the button was still GREYED (arming) | ignored -- the calibrator
  no longer accepts it | ignored -- no button there`; `push live: <progress>` on change (≤ 4/s, a 5 s
  keepalive); at DONE the **push table** and a summary; `front: ...` and `drive ports, signed: ...`;
  **one line per pulse**; `LEFT OUT: port N ...`; **one line per spin direction**; `===== RUN n
  ENDED: DONE|STOPPED =====`; at Done the MEASURED line (now printed bare, not as `MEASURED: MEASURED
  ...`), the signed ports per side on one line each, `LEFT OUT: none | <ports> -- (<warning>)`, the
  scales with their L/R ratio, the width, the push's drift/medians/spreads, both spin widths, SAVED /
  NOT SAVED (and that an earlier record stays on the card if this run did not replace it); at a stop
  `STOPPED [<CODE> port N]: <reason>`; at every outcome `loop: N ticks, worst gap between two ticks
  N ms ...; N screen prints so far`; at RUN AGAIN `===== RUN n -- RUN AGAIN: a NEW calibrator, nothing
  carried over =====`. Examples from the calibrator's own formatters on the host plant (port 18
  slipping; a throwaway, host numbers, NOT robot output):
  ```
  [R3D]   push port 18 travel +1184.7 deg MOVED sign +1
  [R3D] pulse port 11 (1 of 10) 3.0 V yaw +0.00 deg own +0.0 deg others 0.0 deg: under the floor -- next level
  [R3D] pulse port 11 (1 of 10) 9.0 V yaw -6.47 deg own +49.8 deg others 49.8 deg: LEFT
  [R3D] pulse port 16 (6 of 10) 6.0 V yaw +3.15 deg own +24.3 deg others 24.3 deg: RIGHT
  [R3D] pulse port 18 (8 of 10) 9.0 V yaw +0.00 deg own +630.9 deg others 0.0 deg: LEFT OUT (decoupled)
  [R3D] spin CCW 3.0 V yaw +720.2 deg L -78.56 in R +78.56 in W 12.500 in 7.43 s
  [R3D] MEASURED tank BLUE 600 rpm: L5/R4 ports, ipr L1.62500 R1.62500, W 12.500 in
  ```
- **17:04** — **RUN AGAIN's reset, exactly** (ruling F2-R7, in `runCalibrate()`): on
  `TouchAction::RunAgain` — which `TouchGate` returns only when the phase is Done or Aborted — the
  program prints the RUN banner, commands 0 V + Coast on every candidate, `cal.emplace(motorSpan,
  portSpan, *imu, clock)` (the old calibrator is DESTROYED first — `optional::emplace` — so no sign,
  side, report row, refusal, warning, status or timer survives; F1's case 32 proves two back-to-back
  runs on one plant are independent), `cal->start(facts)`, `gate.emplace(sample.releaseCount)` (a new
  `TouchGate`, based on the release that asked, so it cannot replay; its layout clock starts empty,
  so the first frame is greyed), `run = RunRecord{}` (printed counts, save outcome, outcome flag, push
  log state, layout key, loop timing — value-initialised) and `painter.invalidate()` (the next frame
  is a full repaint). **Carried: nothing of the run.** Kept, because they are the program's: the motor
  adapters (built once, positive ports), the IMU adapter (calibrated once — HA-05 forbids a second
  `calibrate()`), the clock, the pacer, the three stated facts, and `g_screenPrints` (a PROGRAM-wide
  leak count, deliberately not reset). Not host-testable (it is `src/`); checked by reading, and by
  F1's case 32 for the calibrator half.
- **17:24** — **One more check, by reading, of what the rewrite puts on the opcontrol task's STACK** (the
  old program already held the calibrator there): `sizeof` measured with `arm-none-eabi-g++` —
  `DrivetrainCalibrator` 7712 B (F1's pulse records are most of it); on the host (32-bit): `PanelView`
  704 B, `TouchGate` 24 B. The loop's frame adds one painter (~0.7 KB: ten lines, three buttons) and
  one view; `TouchGate::view()` builds at most two more views transiently. PROS's default task stack
  is `TASK_STACK_DEPTH_DEFAULT 0x2000` words (32 KB, `include/pros/rtos.h:72`); the peak (the frame,
  a `viewFor()` call, newlib's `printf`) is estimated at about half of that — an estimate from sizes,
  not a measurement on the robot.
- **17:24** — **FINAL VERIFICATION, in this order, on the finished tree** (every test object rebuilt with
  `touch test/*.cpp` + `shulib_tests/fast`, `Linking CXX executable shulib_tests` seen; the briefing
  regenerated LAST on that green binary; then the full target and the launch prompt's command set):
  - `./build/test/shulib_tests` → `test cases: 1416 | 1416 passed | 0 failed | 3 skipped`,
    `assertions: 2386604 | 2386604 passed | 0 failed`. Per case against the baseline (`cases.py`,
    assertion counts AND printed messages): **1415 identical, 4 differ** — cases 4, 19, 21 and P9,
    each explained above; the plant, the panel's other eleven cases and every other suite identical.
  - `cmake --build build/test` (the full target, doc gates included) → `api_doc_tool self-test: OK`,
    `doc coverage: 2592 public entities across 148 headers, all documented`, `doc example scan: 408
    quoted lines, all verbatim`, `removability: no public doc references docs/internal/`, `briefing
    status: generated block current`, `doc staleness audit: clean`; `Built target shulib_tests`.
  - `python3 tools/src_build_gate.py self-test` → `OK (33 detector cases, each against a real
    build)`; `check` → `src build gate: PASS (all 6 builds link ...)` — bench, tank, tank-drive,
    tank-library, tank-calibrate each `exit 0, 4/4 TUs compiled, warnings: 2 allowed
    (-Wunused-function), 3 vendor-ignored, 0 disallowed`, xdrive `0 allowed`; hashes asserted; beacons
    checked (unchanged).
  - ARM TU over every header (184 + `main`) under `-Wall -Wextra -Wconversion -Wsign-conversion
    -Wshadow -Werror -Os` → exit 0. PROS-free grep → empty. Sim-layering grep → empty.
  - `api_doc_tool.py generate && check-fresh && briefing_status.py check && check-removability` → exit 0.
  - `git diff --stat -- Makefile tools .github src/main.cpp src/bench_r3a.cpp src/drive_program.cpp` →
    empty. Nothing staged, nothing committed, nothing uploaded, no `pros` command run.
  - Files changed (`git diff --numstat`, + / −): `src/calibrate_program.cpp` 1038/453,
    `src/calibrate_program.hpp` 12/8, `include/shulib/calibration/drivetrain_calibrator.hpp` 9/24,
    `include/shulib/calibration/calibrate_panel.hpp` 0/1, `test/drivetrain_calibrator_test.cpp`
    12/13, `docs/internal/chunks/R3a-BENCH-WORKSHEET.md` 225/140, `docs/changelog.md` 20/0,
    `docs/roadmap.md` 8/4, `docs/hardware-assumptions.md` 10/2, `README.md` 2/1,
    `docs/internal/build-order.md` 5/1, `docs/internal/HANDOFF-remaining-work.md` 3/2,
    `docs/internal/PROJECT-BRIEFING.md` 2/2 (generated), `docs/api/` four pages (generated), this log.

### Definition of Done — Part 2F-2 (the brief's §4 F2), with evidence per line

- [x] **§2 in full.** F2.1 — `src/calibrate_program.cpp` renders `viewFor()` through `TouchGate`
      (`PanelPainter`: changed zones only; the view recomputed and painted right after every answer and
      on the tick of every step change; every touch through `TouchGate::touch()` against the view just
      painted); only `"%s"` reaches `screen_print_at` (`printText()`, the one call site); every buffer
      sized from its budget; no '%', ASCII only (the panel's lines proven by F1's host tests, the
      program's own literals by `static_assert`, runtime text sanitised); every pixel y < 236 (the
      string table's arithmetic). Kept: the census, the IMU discovery, the cartridge belief,
      `AllStopGuard`, the per-tick `competition_is_disabled()` exit, write → flush → re-read →
      compare → SAVED, now handed to the panel as `SaveOutcome`. **RUN AGAIN:** rebuilt in place, the
      gate and the per-run record new, nothing carried (the entry above). **The serial record:** every
      line F1 made available, printed — the push table at DONE, one line per pulse, the spin per
      direction, the excluded ports, the MEASURED line and SAVED / NOT SAVED — plus the additions listed
      (F2-R10). F2.2 — the layout is the panel's (F1); the program's own screens follow it. F2.3 —
      Station G rewritten from the code (the entry above).
- [x] **The three retired names removed**, every reference updated (`drivetrain_calibrator.hpp`, the
      panel's one case label, the calibrator's cases 4/17/19/21), `docs/api` regenerated, the changelog.
- [x] **Six builds link, zero disallowed warnings** (`-Wformat-nonliteral`, `-Wformat-truncation`
      included); beacons unchanged — the gate output above.
- [x] **The worksheet rewritten from the code** — every quoted string checked against the source
      (86 verbatim; 51 formatted lines or commands checked by hand).
- [x] **A list of every on-screen string with its size and budget** — the table above.
- [x] **No hardware claim.** The program's header, its `.hpp`, its banner, Station G, the changelog,
      the roadmap and build-order each say the rewritten program has not run.
- [x] **Budgets** derived from the tree's evidence and logged, kept at F1's values (ruling F2-R2),
      stated as derived-not-measured in the log, the worksheet and HA-136.
- [x] **Prose review** — grep run and every hit read before the entry that says so.

**NOT done, and not claimed:** nothing here has run on a robot — no screen of it has been seen on a V5,
no touch of it read, no record written by it; the character budgets, line heights and positions are
derived estimates (Bench Tests button 9 measures them in a minute); the renderer and RUN AGAIN are
checked by the build gate and by reading only (`src/` has no host tests), and the walk-through's print
counts come from a COPY of the painter's rules, not the program; `src/main.cpp`'s calibrate boot banner
and its AUTONOMOUS state screen's over-long SMALL line are listed, not fixed (out of this part's
files); the host-only `-Wformat-truncation` false positive at `calibrate_panel.hpp:644` is recorded,
not fixed (F1's header; the ARM gate does not see it); whether the DONE button flickers if
`pushReady()` toggles near its thresholds on the robot (it re-greys on every reappearance) is for the
serial record's `push live:` lines to show — the panel decides DONE, not the program. **For the
coordinator (brief §8):** rebuild the calibrate package from the COMMITTED tree, confirm the beacon,
and read every screen string against Station G.
- **2026-09-24 20:42** — **Second bench session begins:** the brain enumerated again (`/dev/ttyACM0`/`ttyACM1`, brain `1CA1A500`). Uploading the four packages built from the COMMITTED `2780462` (R3d Part 2F complete) in the clean worktree `.claude/worktrees/hw-2780462`, beacons and stamps checked at build.
- **20:42** — all four UPLOADED (`--after none`): slot 4 shulib Calibrate (`tank-calibrate`), slot 3 Bench Tests (`tank`), slot 1 shulib Drive (`tank-drive`), slot 2 shulib Teleop (`tank-library`), each stamped `v0.1.1-319-g2780462`. Recording the user port from here (capture `session4-calibrate`).
- **~20:45 — shulib Calibrate (R3d Part 2F) RUN 1 on robot two, on the floor — got through the push and
  every pulse; STOPPED at the spin's agreement check.** Evidence:
  `docs/internal/evidence/r3d-session-2026-09-24b-calibrate-run1.log` (read with `strings`). Build stamp
  `shulib Calibrate Sep 24 2026 17:42:16` (the 2780462 package). Census: RADIO 1, IMU 2, VISION 3,
  DISTANCE 4, DISTANCE 5, MOTOR 11–18 and 20 (19 unplugged) — 9 candidates.
  - **The one push** (DONE touched after it; on the WHICH END screen one touch landed on no button and was
    ignored — `touch: x=185 y=209 release=3 on nothing -> ignored` — before FRONT FIRST): 8 of 9 MOVED, drift **+0.70°** (bound 3.0), median **36.78 rad** (~2100° of shaft
    for 48 in). Per port: 11 +2091.6 · 12 −2094.0 · 13 +2092.8 · **14 +0.0 (did NOT move — as in the
    `-census.log` push and both pushes of `-calibrate.log`; `-bench2.log`'s motor watch DID see it move,
    −389 and −331: INTERMITTENT)** · 15 +2120.4 · 16 −2151.6 · 17 +2150.4 · **18 −1545.6 (73 % of its
    mates — HA-130's short travel, again)** · 20 −2156.4. `FRONT FIRST` answered. **Every captured sign is
    the INVERSE of the typed table's** (+11 −12 +13 +15 | −16 +17 −18 −20 vs the table's −11 +12 −13 −15 |
    +16 −17 +18 +20): the end that led this push is the end the table calls the BACK (the 11/20 end), or the
    motors were rewired in the rebuild — either way the record's FRONT is the end that led the push.
  - **The ramped pulses WORKED** (the team lead's choice): 3 V moved nothing enough on most ports; **6 V
    turned the robot 6–14.5°** on ports 11, 12, 13, 16, 17, 20; port 18 cleared the floor at 3 V
    (−4.34°). Sides: 11, 12, 13 on one side; 16, 17, 18, 20 on the other (labelled RIGHT / LEFT relative to
    the end that led the push). **Port 15 was LEFT OUT (DECOUPLED)**: at 6 V its own shaft 60°, the others
    13°, yaw 1.6°; at 9 V its own 24°, the others 2.4°, yaw 0.43° — its motor turns but does not drive the
    train (a slipping or disengaged gear). Port 14 was never a drive candidate (it did not move in the push).
  - **The spin:** CCW 3.0 V, +720.8° in 11.2 s, sides −109.35 / +82.02 in ⇒ W 15.212 in; CW 3.0 V,
    −720.3° in 8.2 s, sides +136.22 / **−19.79** in ⇒ W 12.410 in. **STOPPED [SPIN_DISAGREE]: 20.3 %**
    (bound 5 %). Nothing written to the card ("Every motor is at 0 V and coasting. Nothing was written to the
    card by this run."). The side of 11, 12, 13 barely rolled in the CW spin. **Best explanation, NOT
    proven:** that side had three working motors and two that do not drive (14 not turning, 15 slipping)
    against four on the other side (19 unplugged), so the robot pivoted about the weak side in one direction
    and not the other and the two directions measured different effective widths. Nothing in the log points
    at the software (the yaw is `get_rotation()` summed per tick at ~0.6°/tick; both spins stopped within
    1° of 720°), but that is an absence of evidence, not a proof. Loop worst gap 15 ms; 253 screen prints.
  - **The team lead asked whether the spin was meant to be 90°** — no: two full turns each way (the config
    line says so). He first counted "maybe 3-4 times per each direction" by eye on this run; the IMU read
    +720.8° / −720.3°, and the wheel travel (191.4 in and 156.0 in of side travel summed) would need an
    effective track width of 6.2–10.2 in (the two directions) for 3–4 turns. **Settled on run 2**, below: he counted it again —
    "it spun twice per".
- **~21:00 — Calibrate RUN 2 (RUN AGAIN on the same boot, or a fresh start — not recorded which): the team
  lead reports "it worked" and "it spun twice per"; the finished screen read "right 4 left 4"** (his words;
  the panel's line is `LEFT %u  RIGHT %u`, from `r.left.count` / `r.right.count`). **NO serial capture:** the
  brain left USB at 20:53:51 (`session4-204340.log.events`: `closed`, then only `no user port` until the
  recorder's last line at 21:20:29), so every serial line of run 2 is lost. What is NOT known: whether the
  header said SAVED, the `ipr L/R` and `track` numbers, the LEFT OUT line, and which motor made the 11/12/13
  side four this time (run 1 had 14 not moving and 15 LEFT OUT as DECOUPLED; 14 has moved in one earlier
  capture, so either could be intermittent). **If the
  header said SAVED, `/usd/shulib_drivetrain.bin` now exists, and "shulib Teleop" (slot 2) will load it on
  its next boot in preference to the typed table (`src/main.cpp`, WHERE THE DESCRIPTION COMES FROM),
  with FRONT = the end that led the push.** Next: read the record back (its numbers) before anything
  drives from it.

## Part 3 continued — 2026-09-25 evening: shulib Teleop from the saved record (Station G.6) — the library's graph DROVE robot two

Evidence: `docs/internal/evidence/r3d-session-2026-09-25-teleop.log.gz` (gzip -9 of the raw 17.25 MB
capture, verified byte-identical on decompress; read with `zcat … | strings`; the reconnecting recorder,
`session5`, opened the user port at 22:04:17, lost it 22:15:07 when the team lead switched the robot
off, re-opened 22:15:45, closed 23:07:33).

- **Before the capture: a Calibrate run, UNRECORDED.** The team lead ran shulib Calibrate (slot 4, the
  `2780462` build) before plugging in: "calibration ran it seemingly worked". No serial record exists.
  The record Teleop then booted from says `description provenance: shulib Calibrate Sep 24 2026 17:42:16 |
  ROBOT-MEASURED: 8 ports, push 48 in, W 12.006 in` — that build stamp is shared by 2026-09-24's run 2 and
  tonight's run, and the record carries no date, so WHICH run wrote it is not provable from the tree
  (tonight's, by the team lead's account).
- **shulib Teleop (slot 2, `v0.1.1-319-g2780462`, stamp `Sep 24 2026 17:42:12`) — EIGHT starts, all
  booted from the RECORD:** `calibration record: DECODED, port check OK -- OK: all 8 recorded drive ports
  carry motors, IMU on port 2. NEW motors not in the calibration: 14.` · `boot source: RECORD` ·
  `[SES] ports MEASURED L -16 +17 -18 -20 R +11 -12 +13 +15 IMU2 ipr 1.2544/1.3670 TW12.01` · `4+4 slots
  behind two MotorGroups, 4+4 PRESENT` · `ff kS 1.00 kV 0.1714 kA 0.0000`, budgets 60 in/s · battery
  13.0-13.1 V at boot. **The record's L/R scales differ by 9.0 %** (1.2544 vs 1.3670 in/rad), against
  run 1's push at ~2.7 %. **This is the first time the library's own object graph (MotorGroups,
  DriveEncoderOdometry, Localizer, Chassis::drive through feedforward) drove robot two** — open-loop, under
  driver control. It is NOT a library-commanded motion (no moveTo ran; R3b stays open).
  | start | length | faults (the log's own lines) |
  |---|---|---|
  | 1 | 306.8 s | none (before the worksheet run: the team lead called start 2 "the real one ... where I actually followed the rules") |
  | 2 | 93.6 s | `fault=ODO_STUCK n=1 FIRST implausible odometry tick` at t=34.40, `n=2` at t=60.32 — **the team lead's "real" G.6 run** (wheels up, then floor, per the worksheet) |
  | 3 | 78.7 s | `fault=MOTOR_GROUP_DISAGREE n=1 FIRST 1 member(s) disagree with their coupled group` at t=10.09 (commanded `v(9.2, 2.0, 0.00)`); the log does not name the member |
  | 4-8 | 228.6 / 18.1 / 16.4 / 13.4 / 671.6 s | none |
  **Both ODO_STUCK ticks came with commanded speed 0** (`v(0.0, 0.0, 0.00)` on the fault tick's own
  record) — but only episode 1 was at rest. **CORRECTED 2026-09-26 (DIAG1; this entry first read "Both
  ... came with commanded speed 0 ... i.e. not from driving" and "a reading glitch at rest"):** in
  `zcat r3d-session-2026-09-25-teleop.log.gz | strings`, episode 1 is strings line **102381** (t=34.40):
  the last non-zero command is t=34.01 (a gentle forward, ~5.7 in/s), then zero for 0.39 s — at rest.
  Episode 2 is strings line **110158** (t=60.32) and fired on the FIRST zero-command record after
  **~0.35 s of hard reversing** (t=59.97-60.31, strings lines 110053-110155, |v| up to 34.0 in/s). A
  record's `v(...)` is the COMMANDED speed in the FIELD frame (`robotToField(body, published heading)`),
  so on this tank the published heading can be reconstructed as `atan2(-vy, -vx)` while reversing:
  over the last nine 10 ms intervals (t=60.22→60.31) it swung +16.9, -19.7, +17.6, -17.1, +18.5,
  -1.4 (t=60.27→60.28, strings lines 110143→110146), -18.2, +24.1 and **-36.9°** — eight of the nine
  at 16.9° or more, those eight alternating in sign, up to 37° — and no robot turns 37° in 10 ms.
  *(Re-checked by DIAG1's review, 2026-09-26: this correction first listed the eight large swings as if
  they were consecutive, dropping the -1.4.)* That makes the IMU heading path the prime
  suspect for episode 2; it is an INFERENCE (it assumes zero body lateral and that the stamped heading is
  the published one), not a finding, and it is registered as HA-160. `DriveEncoderOdometry` calls a tick
  implausible on a non-finite delta, |Δθ| > π/2 in one tick, or |side travel| > 36 in in one tick
  (`drive_encoder_odometry.hpp` ~:164-166 as of `ea58fbc`), and that day's fault line recorded none of the values —
  **cause still UNKNOWN for both**; since DIAG1 the line names the check and its values
  (`why=ROT dth=… h0=… h1=… L=… R=…`), so the next one will say. The fault then stays latched on every
  record (`flt=ODO_STUCK`), q 0.34 → 0.20.
- **The team lead's screen readings (Station G.6.3/F.2 and G.6.4), taken across starts 3-8** — `est x`
  and `hdg` exist only on the panel, not on serial:
  - **forward drove toward the CAMERA end** (AI Vision, port 3). With the record's signs identical to
    2026-09-24 run 1's (the inverse of `src/chassis_table.hpp`'s), the camera end is the end the typed
    table calls the BACK (the 11/20 end): **the table's "front = 15/16 end" is the opposite of the robot's
    front as the team uses it.**
  - one tile (24 in) forward by stick: `est x` "about 22-24"; driven back: "near 0"; one full spin:
    `hdg` back within "<0.3"; everything else "working as it should".
  - **G.6.4, the 48 in taped drive: `est x` = 44 (8.3 % short), ending at heading −11.2° (veered RIGHT).**
    A gradual 11° veer shortens the x-projection by ~0.5 %, so it does not explain the 8 %. **The saved
    scale under-reads distance by ~8 %**; a 40 in library drive would travel ~43.6 in. Best explanation
    (NOT proven): the push that made the record went past 48 in. The veer: open-loop teleop gives both
    sides equal volts; the right side is the weak one (14 not in the record, 15 slipping) — the team lead
    leans "build", and that fits.
- **The team lead's decision: NO Calibrate rerun tonight.** R3d stays OPEN (its close is the recorded
  reproduction run), and the ~8 % scale shortfall and the 9 % L/R gap are carried, not fixed.
- **Bench Tests `6b074ae`** (beacon `tank`, stations 11 DISTANCE / 12 AI VISION) uploaded to slot 3 at
  ~22:17; not run yet.
- **Rulings made this session (the doc pass that carries them into build-order / roadmap / handoff waits,
  by the team lead's choice, until the auton is done):** (1) an auton hook inside shulib Teleop was
  briefed, launched and **stopped before any code** — "I want to eventually use the actual autonomous not
  put it in teleop. i want to do things genuinely correct"; autons go only through the real
  `autonomous()` (G1 2b → 2c → COMP). (2) After G1 Part 2b: **2c, then COMP**. (3) Whether the team owns a
  competition switch: unknown (the team lead is asking).
