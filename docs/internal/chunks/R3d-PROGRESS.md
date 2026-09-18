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
