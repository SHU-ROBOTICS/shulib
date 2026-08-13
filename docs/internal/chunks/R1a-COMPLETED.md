# R1a — COMPLETED (2026-08-13, in the working tree pending review/commit)

> `hal/pros` adapters: the drivetrain, the driver, and the seam that makes them testable.
> Written FROM `R1a-PROGRESS.md` (the live log), not instead of it. Nothing committed,
> nothing pushed — verification is the reviewer's, per the standing process.
> Predecessor: F2. Successor: **R1b** (mechanism-sensor adapters), then R3.

## The one-sentence honest status

The library now has its first hardware-binding code — nine PROS-backed adapters plus the real
tick pacer, host-tested and mutation-proven against a programmable shim of PROS — **and it has
still never driven a robot**: the shim tests the adapters against our beliefs about PROS
(HA-94…112, all registered), only a bench tests the beliefs, and the bench session is
deliberately unrun (it needs the team lead at a table; `R1a-BENCH-RUNBOOK.md` is its
replacement deliverable, walking every belief in dependency order).

## Built, file by file

**Adapters — `include/shulib/hal/pros/` (header-only; the ONLY tree permitted to include
PROS, by path-anchored guard):**
- `clock.hpp` — `ProsClock` over `micros()` (T6 ruling: not `millis()` — 1 ms quantization is
  10% of the tick, a 10% error in every derivative term). HA-101.
- `motor.hpp` — `ProsMotor`: mV command via `motorVoltageToMillivolts` (clamp + round),
  non-finite REJECTED (L4 — never coerced), ctor demands the gearset, sets degrees+gearing
  explicitly AND reads both back (trap A), sentinel screen → hold-last-good + `faultedReads()`
  (T7), brake modes mapped both directions, reversal by port sign only (once, in PROS).
- `rotation.hpp` — `ProsRotation`: centidegree conversions, PROS_ERR (in-band INT32_MAX)
  screened, no reset/re-zero calls by design.
- `imu.hpp` — `ProsImu`: binds `get_rotation()` (HA-03), bootHeading applied exactly once
  (HA-05), NEVER tares (guard-pinned), `calibrate()` = the one sanctioned reset (double-call
  is a precondition violation), BOTH yaw-rate sources behind `YawRateSource` (T4 ruling;
  differentiation default, gyro-z labelled HA-04/HA-109), isReady = calibrated AND not
  calibrating, pitch/roll unnegated pending HA-110.
- `gps.hpp` — `ProsGps`: PORT-ONLY construction, `get_offset()==(0,0)` boot check with a
  deferred path that can never throw from `pose()`/`hasFix()` (a bug in my own first design,
  caught before tests — see Findings), sentinel → no-fix BEFORE conversion (HA-08), calls
  `gpsToRobotPose()` and `gpsRmsErrorToCanonical()` (E2's function, finally called by the
  adapter it was built for), hasFix = device validity only (error-magnitude gating stays E2's).
- `battery.hpp` — `ProsBattery`: mV/mA/percent conversions; header carries the loud warning
  that the unit belief is website-only (HA-99, the weakest in the chunk); holds last-good,
  never zero (a 0 V read would floor every motor command).
- `char_sink.hpp` — `ProsCharSink` (main.cpp's StdoutCharSink promoted; injectable FILE* so
  it is testable; deliberately includes no PROS header at all).
- `line_display.hpp` — `ProsLineDisplay` over Controller::set_text: truncate at kCols, NEVER
  wrap, pad to a true overwrite; records the HA-107 15-vs-19 column conflict R1a found.
- `controller.hpp` — `ProsController`: ÷127 via the conversion, `get_digital()` LEVELS only
  (never the consuming new_press — guard-pinned), positive `isConnected()`, master/partner by
  construction.
- `tick_pacer.hpp` — `ProsTickPacer` over `Task::delay_until` (anchored cadence; lazy first
  anchor so a late start cannot replay phantom catch-up ticks). Replaces V5DelayPacer.

**New seam (F4-additive, register row F13, NOT frozen):** `hal/controller.hpp` —
`IController` + `ControllerAxis`/`ControllerButton` + `ButtonEdge`;
`hal/fake/fake_controller.hpp` (disconnected-reads-zero mirrored from HA-103).

**New pure conversions (PROS-free):** `hal/motor_conversion.hpp`,
`hal/rotation_conversion.hpp`, `hal/controller_conversion.hpp` — the imu/gps_conversion
pattern, each carrying its binding contract and HA citations.

**The host shim:** `test/pros_shim/pros/{error.h, rtos.hpp, motors.hpp, rotation.hpp,
imu.hpp, gps.hpp, misc.hpp, shim_control.hpp}` — every semantic transcribed from the vendored
headers' doc comments (file:line cited in each), every header `#error`s without
`SHULIB_HOST_PROS_SHIM` (defined ONLY by test/CMakeLists.txt, wired BEFORE so it shadows the
real SDK in the host build only), the honest-limit statement in the shim's own words
(shim_control.hpp carries the full version), and ADVERSARIAL defaults: motor ports boot in
rotations/red (the leave-device-as-is trap made real), get_heading wraps while get_rotation
accumulates, new_press really consumes.

**Tests (9 new files, +65 cases / +~1,650 assertions):** `motor_conversion_test`,
`rotation_conversion_test`, `controller_conversion_test` (hand-computed literals, never the
constant under test), `pros_motor_adapter_test`, `pros_rotation_adapter_test`,
`pros_imu_adapter_test`, `pros_gps_adapter_test`, `pros_controller_adapter_test`,
`pros_platform_adapter_test` (clock/pacer/battery/char-sink/line-display),
`pros_adapter_fence_test` (the structural guard: exact two-flag suppression set, pop before
shulib code, forbidden calls textually absent, shim #error presence, in-tree path-anchored
PROS scan). Every test names the bug it would catch.

**Wiring and build:** `src/main.cpp` rewritten (all 14 R1 markers resolved — none re-homed:
adapters wired, precondition policy installed FIRST in initialize() before the graph
constructs, session header emitted with a live battery read, teleop loop at the tick cadence
with isConnected gating and HA-112's labelled deadband-only mapping, autonomous still
deliberately motionless until R3); `Makefile` injects `SHULIB_BUILD_HASH` per build (`--dirty`
load-bearing); `.github/workflows/ci.yml` PROS-free guard amended PATH-ANCHORED;
`test/CMakeLists.txt` wires the shim.

**Docs:** `docs/changelog.md` NEW + nav (deliverable #7 now standing); `docs/faq.md` NEW +
nav; guide ch. 07/08/13/14 updated (14 with the most care — the adapter bullet now says
exactly what host-testing can and cannot claim); `docs/hardware-assumptions.md` HA-94…112 +
corrected status line; `docs/roadmap.md` WS2 checkbox `[~]` with cited evidence, you-are-here
R1a entry (not-done-first), F13 row, corrected Next pointer. Internal: this file, the live
PROGRESS log, `R1a-BENCH-RUNBOOK.md` (15 ordered steps, each naming what it measures, which
HA-nn it settles, and what a wrong answer looks like).

## Numbers

| Measure | Before | After |
|---|---|---|
| Suite | 1018 cases / 1,522,327 asserts / 3 skipped | **1083 / 1,523,069 / 3** (green) |
| ARM gate | 124 headers | **139 headers**, UNAMENDED, clean |
| PROS-free guard | broad, no exemption | path-anchored, proven live (plant→caught→clean) |
| Layering guard | pass | pass (unchanged) |
| `make` | package (fake-backed) | package (adapter-backed), hash injected |
| Doc gates | all green | all green (changelog/faq in scope) |

## The mutation table — every one EXECUTED and OBSERVED

Runner: exact-match exactly-once edits, **build-gated** (a failed build can never read
green), full suite per mutation, restore from scratchpad copies (never `git checkout`),
SIGPIPE ignored. M13 was executed live during the guard proof.

| # | Mutation | Result (observed) |
|---|---|---|
| M1 | delete the mV scale | **RED** (7 cases / 18 asserts) |
| M2 | delete the mA ÷1000 | **RED** (2 cases) |
| M3 | delete the centidegree scale | **RED** (5 cases) |
| M4 | get_heading() for get_rotation() in the differentiator | **RED** (1 case — the phantom-rate seam test) |
| M5 | remove units/gearing set + read-back | **RED** (4 cases — the shim's adversarial defaults bite) |
| M6 | remove the GPS offset boot-check | **RED** (2 cases) |
| M7 | drop gpsRmsErrorToCanonical | **RED** (1 case) |
| M8 | remove rotation sentinel screening | **RED** (1 case) |
| M9 | non-finite → silent coerce-to-zero | **RED** (1 case) |
| M10 | bind get_digital_new_press | **RED** (3 cases — two-consumer starvation + count pin) |
| M11 | fence pop moved to end-of-file | **RED** (fence-scope guard test) |
| M12 | remove a shim header's #error | **RED** (shim guard test) |
| M13 | guard widened to --exclude-dir + smuggled localization/pros/ | **EXECUTED LIVE**: exclude-dir form MISSED the plant (the measured hole); the shipped path-anchored form CAUGHT it |
| M14 | conversion computed then discarded, raw returned | **RED** (4 cases) |

**Self-added M4b — the honest green:** swapping `get_heading()` at the **heading()** site
(not the differentiator) stayed **GREEN**, and the record says why rather than hiding it:
through the wrapping `math::Angle`, the two bindings are OBSERVATIONALLY EQUIVALENT at that
one seam — no behaviour test can distinguish semantically identical mutants, so this is an
equivalence, not a suite hole. Because HA-03's binding contract bans the call in writing
regardless, it was then closed the way the tare-family ban is closed — a textual pin in the
fence guard test — re-run, and **observed RED**. Final tally: 15 mutations executed, 15
observed at their expected/recorded outcome, zero unexplained greens.

## HA register entries added (HA-94…112)

94 move_voltage mV (reasoned) · 95 position=output-shaft degrees (reasoned) · 96 RPM
(reasoned) · 97 mA (reasoned) · 98 device-state persistence + read-back (reasoned) ·
99 battery mV/mA — **website-only, the weakest** (reasoned, flagged) · 100 capacity percent
(reasoned) · 101 micros (reasoned) · 102 delay_until cadence (reasoned) · 103 axes ±127 +
zero-on-disconnect (reasoned) · 104 new_press consumes (reasoned) · 105 rotation velocity
centideg/s (reasoned) · 106 gps yaw = CW-from-North (reasoned-weak) · 107 LCD 15-vs-19 column
**CONFLICT found by reading the vendored doc** (invented) · 108 reset()=calibration
(reasoned) · 109 gyro deg/s + z=yaw (invented) · 110 pitch/roll signs (invented) · 111 port
map (invented, boot-loud by design) · 112 teleop mapping + deadband (invented, T2's).

## Decisions where a viable alternative existed

1. **Quoted `"pros/*.hpp"` includes over amending the Makefile** — forced by a FINDING (below);
   quoted is PROS's own internal convention and resolves in all three builds; adding `-I` to
   the kernel build changes include semantics for every future file.
2. **T7 confirmed with one refinement:** screen → hold-last-good → *expose* (`faultedReads()`);
   the RAISE stays with the loop layer (HealthMonitor), because hal/ sits below diag/ and
   raising is policy by that file's own charter — and hold-last-good is precisely what the
   existing ODO_STUCK cross-check is designed to see. Rejected: adapters taking a FaultLatch
   (layering inversion); silent zero (the invisible-runaway shape).
3. **ProsGps deferred-offset policy:** boot-phase nonzero → precondition (loud); deferred
   nonzero → permanently no-fix, never a throw from a read path. Rejected: throwing from
   pose() (breaks gps.hpp:27-29), trusting an unverifiable device.
4. **`ProsMotor` ctor takes the gearset with NO default** — the cartridge is a physical fact;
   rejected: defaulting to green (a silent wrong-velocity-scale on a red/blue drive).
5. **isReady() = calibrateStarted && !is_calibrating** — an IMU nobody calibrated must read
   not-ready; rejected: negating is_calibrating alone (ready-by-default garbage window).
6. **Line display pads to kCols** — set_text leaves the old tail otherwise ("ARM OKFAULT");
   rejected: raw truncate-only writes.
7. **Pacer anchors lazily on first pace()** — rejected: anchoring at construction (a late
   first pace would replay phantom catch-up ticks through FreeRTOS's semantics).
8. **hasFix() = device validity only** — error-magnitude gating stays in E2's corrector where
   its thresholds are already registered; rejected: a second HAL-level threshold (two gates on
   one number fight invisibly).
9. **M4b closed with a textual pin, not a behaviour test** — a behaviour test cannot
   distinguish observationally equivalent mutants; inventing one would be theater.
10. **Changelog reconstructs 2.0/2.1 history and marks it** *(reconstructed)* — rejected:
    pretending the file always existed.

## Findings in EARLIER chunks (Rule 4: fixed there), and build-system findings

1. **PROS's `common.mk` resolves includes with `-iquote` ONLY** — angle-bracket `<pros/*>`
   includes can never compile in the robot build. The brief's M5/M7 measurements used `-I`
   flags, so this was invisible until `make` ran with a real adapter. Fixed in the adapters
   (quoted form); fence test updated to match.
2. **The assumptions register's status line was stale since E4** — "0 of 82" over 93 entries;
   E4's nine and F1's two never updated the narrative. Fixed (now 0 of 112, with the counts
   recomputed: 77 invented / 32 reasoned / 2 measured-elsewhere / 1 mixed) and the staleness
   recorded in place.
3. **README carried TWO different suite counts** (915 headline, 659 in the tree diagram) —
   the exact trap the briefs have twice recorded being found by reading. Fixed; the tree-block
   count is now non-numeric so it cannot re-diverge.
4. **roadmap's "Next:" pointer said Phase T** — stale against the recorded R1-split ruling
   that folded T1 into R1a. Fixed, with the correction noted in place.
5. **The build order doc's "Every chunk closes with all six"** over a seven-row table
   (deliverable #7 added at R1a briefing without updating the prose). Fixed, noted in place.
6. **HA-107 (new):** the vendored `set_text` doc's col range [0-14] contradicts HA-57's
   19-column claim — registered as a conflict for the bench, not silently resolved either way.
7. *(In this chunk, caught before tests:)* my own ProsGps first design could throw from
   pose()/hasFix() via the deferred offset check — a MUST-NOT-THROW violation. Fixed, and the
   test "a bad offset found later = dead GPS, no throw" pins it.

## Not finished, named honestly

- **`[~]` upload-and-boot of THIS binary:** `make` produces the package (verified, exit 0,
  hot+cold binaries present); uploading and booting it needs the physical brain — runbook
  step 1, owner: team lead. The 2026-08-12 boot evidence is for the C7-era fake-backed binary,
  and nothing here claims otherwise.
- **The bench smoke session: NOT RUN, by instruction** — replaced by `R1a-BENCH-RUNBOOK.md`.
  Every HA-94…112 entry remains UNSETTLED until it runs. T8's task-boundary question is
  likewise recorded as a bench observation, not answered.
- **R1b's adapters untouched** (IDistance/IOptical/IDigitalOut/IDigitalIn/IBlockSink) — per
  the split; trap B (the distance sensor's in-band 9999) stays owned there.
- **Teleop is deadband-only** — T2 owns driver feel; HA-112 labels the mapping.
- **The `Next:` pointer in the build order doc still reads R1a** — deliberately: it flips when
  the reviewer verifies and commits this chunk, not before.

## Handoffs

**To R1b:** the fence pattern, the shim framework + `resetAll()`, the amended guard, the
adapter idiom (screen→hold→expose), the QUOTED-include lesson (finding 1 — R1b must not
rediscover it), and trap B. **To R3:** HA-94…112 with the runbook that walks them; the
port-map constants block in main.cpp; the field-cal oracle pointer for HA-01. **To T2:** the
IController seam (row F13 — its freeze trigger), ButtonEdge, and HA-112's mapping to replace.
