# T2 — Input shaping, drive modes, and the feel properties — PROGRESS

<!-- release: between parts — no part is building now; its landed parts live in 'shulib Drive'; its open parts are in build-order's T2 section (declared by the coordinator 2026-11-13 for the first release since August) -->

> **Live log.** Appended as the work happens, not written afterwards.
>
> | | |
> |---|---|
> | Chunk | T2 — Input shaping, drive modes, and the feel properties |
> | Brief | `docs/internal/chunks/T2-input-shaping-drive-modes.md` (main tree at execution time; uncommitted) |
> | Date | 2026-09-18 |
> | Tree at start | `76a5819` |
> | Executor model | `opus` |
> | Mode | **worktree, parallel with R3d** (`.claude/worktrees/agent-aef02a3f7fb230a9d`) |
> | Commits | **none** — the coordinator integrates (brief §7) |

---

## 0. Setup note (before any work)

The worktree was handed over checked out at **`c778c11`** ("the release protocol is a script now"),
which is **not** an ancestor of `76a5819` — it sits 30+ commits behind on a different tip, with no
`docs/internal/RESUMING.md` and no `include/shulib/teleop/stick_mapping.hpp`. The tree was clean
(`git status --porcelain --untracked-files=all` empty), so the worktree branch
`worktree-agent-aef02a3f7fb230a9d` was moved with `git reset --hard 76a5819`. Nothing was stashed,
nothing was committed, and the main checkout at `/home/gonzei/projects/shulib` was not touched.

**Clean-tree suite BEFORE any change** (`cmake -S test -B build/test && cmake --build build/test -j`):

```
[doctest] test cases:    1208 |    1208 passed | 0 failed | 3 skipped
[doctest] assertions: 1563156 | 1563156 passed | 0 failed |
[doctest] Status: SUCCESS!
```

Build emitted no warnings under the project's `-Werror` policy.

---

## Order of work

1. [x] PROGRESS log; clean-tree suite count.
2. [x] `input_shaping.hpp` + `slew_limiter.hpp` + tests 1–3 (+ mutations).
3. [x] `drive_modes.hpp` + tests 4, 9, 10.
4. [x] `field_centric.hpp` + test 5.
5. [x] `driver_control.hpp` + tests 6, 7, 8.
6. [x] `stick_mapping.hpp` header comment; docs; ARM gate; CI greps; whole suite; DoD with evidence;
       final suite count. **Chunk complete in this worktree; nothing committed.**

---

## 1. Log

### 2026-09-18 — step 1 done

Read in order: `RESUMING.md`, the brief §0–§8, `teleop/stick_mapping.hpp`,
`test/stick_mapping_test.cpp`, `teleop/coupled_side_monitor.hpp` (`SideVolts`, `tankSideVolts`),
`chassis/chassis.hpp` (`drive()`), `hal/controller.hpp`, `math/twist2d.hpp`, `math/angle.hpp`,
`math/frame.hpp`, `test/chassis_drive_test.cpp`, `test/motion_test_rig.hpp`,
`sim/scenario.hpp`, `build-order.md` Phase T + T2, `roadmap.md`'s Phase T rows,
HA-112's register entry and settle block, `docs/guide/14-what-it-cannot-do-yet.md:306-330`,
`.github/workflows/ci.yml`'s two greps, `tools/api_doc_tool.py`'s header, `test/README.md`,
`test/CMakeLists.txt`.

### 2026-09-18 — step 2 done: input_shaping.hpp + slew_limiter.hpp + tests 1–3

Landed `include/shulib/teleop/slew_limiter.hpp`, `include/shulib/teleop/input_shaping.hpp`
and `test/input_shaping_test.cpp` (16 cases, 202 assertions). `StickInput` gained `rightY`
(additive, defaulted, declared before `connected` so every designated initializer in the tree
still compiles in order — checked: the only aggregate initializations anywhere are designated).

A fast mutation harness was used for the cycle
(`g++ … test/test_main.cpp <one TU>`, ~1.4 s per run) instead of the full suite; every mutation
below was then confirmed against the real build at the end of the chunk.

#### Mutations RUN for tests 1–3 (each applied, built, run, observed, restored)

| # | Mutation | Where | Result | Observed message (first failing assertion) |
|---|---|---|---|---|
| M1 | the naive cut — `return axis` outside the band instead of the rescale | `input_shaping.hpp continuousDeadband` | **RED** (3 cases, 11 assertions) | `CHECK( continuousDeadband(std::nextafter(d, 1.0), d) < 1e-12 ) is NOT correct!  values: CHECK( 0.05 < 1e-12 )` |
| M2 | the sign of zero — `return -axis * 0.0` inside the band (landmine 1) | `input_shaping.hpp continuousDeadband` | **RED** (1 case, 2 assertions) | `CHECK( sameBits(continuousDeadband(0.0, d), 0.0) ) is NOT correct!  values: CHECK( false )` |
| M3 | `axis * (expo*axis*axis + 1.0)` — the brief's named curve mutation, leaves (1,1) | `input_shaping.hpp expoCurve` | **RED** (1 case, 14 assertions) | `CHECK( expoCurve(1.0, e) == 1.0 ) is NOT correct!  values: CHECK( 1.1 == 1 )  logged: e := 0.1` |
| M4 | dt ignored — `step = rate_` | `slew_limiter.hpp update` | **RED** | `CHECK( moved <= rate * dt * (1.0 + 1e-12) ) is NOT correct!  values: CHECK( 0.5 <= 0.0005 )  logged: rate := 0.5, dt := 0.001, target := 1, y := 0.5, previous := 0` |
| M5 | the negative-side clamp skipped — `else if (false)` | `slew_limiter.hpp update` | **RED** | `CHECK( moved <= rate * dt * (1.0 + 1e-12) ) is NOT correct!  values: CHECK( 1.02 <= 0.0005 )  logged: target := -1, y := -1, previous := 0.02` |
| M6 | OFF written as arithmetic — `last_ = target * 1.0` | `slew_limiter.hpp update` | **GREEN — see below** | — |
| M6b | OFF written as arithmetic — `last_ = 0.0 + target` | `slew_limiter.hpp update` | **RED** | `CHECK( sameBits(off.update(-0.0, 0.01), -0.0) ) is NOT correct!  values: CHECK( false )` |
| M6c | the OFF branch removed entirely — rate 0 falls through the general path | `slew_limiter.hpp update` | **RED** (the whole bit sweep) | `CHECK( sameBits(off.update(x, 0.01), x) ) is NOT correct!  values: CHECK( false )  logged: x := -1` |

**M6 stayed green, and the honest reading is that it is not a defect.** `target * 1.0` is
bit-identical to `target` for every IEEE-754 double — sign of zero, infinities and quiet-NaN
payloads all survive a multiply by one. So there was no bug for a test to catch; the mutation
was a badly chosen one, not a gap in the suite. It was replaced by two mutations that ARE
defects and both went red: M6b (`0.0 + target`, which turns `-0.0` into `+0.0` — the exact
sign-of-zero class landmine 1 names) and M6c (the OFF branch deleted, which is the realistic
"the default just falls through the general path" bug and leaves the limiter stuck at 0).
Nothing was weakened to make M6 red.

**A test-design defect found and fixed while running M3.** The file originally opened with
`static_assert(expoCurve(1.0, 1.0) == 1.0)` and three siblings pinning the ends at compile
time. Under M3 the build stopped at that `static_assert` and no runtime case ran at all — the
exact failure `test/stick_mapping_test.cpp` warns about in its own header ("a flipped sign
observed as a failing TEST with its inputs in the message rather than as a compile error that
hides every other case"). The compile-time pins were narrowed to inputs no plausible arithmetic
mutation changes (`continuousDeadband(0.0, 0.05) == 0.0`, `expoCurve(0.0, 0.5) == 0.0`,
`shapeAxis(0.0, AxisShaping{}) == 0.0`, `AxisShaping{}.valid()`, `SlewLimiter{}.isOff()`), which
still prove constexpr-evaluability; the ends are pinned by runtime cases. M3 was then re-run and
produced the message in the table.

**Sweep counters added.** Four sweeps only `CHECK` on failure (the house pattern from
`stick_mapping_test.cpp`, so a passing sweep does not inflate the assertion count). Each now
carries a `CHECK(swept > 4000)` so a loop broken to zero iterations cannot pass vacuously.

### 2026-09-18 — step 3 done: drive_modes.hpp + tests 4, 9, 10

Landed `include/shulib/teleop/drive_modes.hpp` (`DriveMode`, `arcadeRequest`, `tankRequest`
in two overloads, `modeRequest`) and `test/drive_modes_test.cpp`.

#### Mutations RUN for tests 4, 9, 10

| # | Mutation | Where | Result | Observed message |
|---|---|---|---|---|
| M7 | the halves swapped — `forward = (r-l)/2`, `yaw = (l+r)/2` (the brief's named mutation) | `drive_modes.hpp tankRequest` | **RED** | `CHECK( got.left == maxVolts * l ) is NOT correct!  values: CHECK( 12 == -12 )  logged: l := -1, r := -1, got.left := 12, got.right := -12` |
| M8 | the yaw sign flipped — `yaw = (l-r)/2` | `drive_modes.hpp tankRequest` | **RED** | `CHECK( got.left == maxVolts * l ) is NOT correct!  values: CHECK( -11.25 == -12 )  logged: l := -1, r := -0.9375` |
| M9 | the `isConnected()` guard dropped in ONE mode (tank only) | `drive_modes.hpp tankRequest(StickInput)` | **RED**, and it NAMES the mode | `CHECK( sameBits(r.yawCcw, 0.0) ) is NOT correct!  values: CHECK( false )  logged: static_cast<int>(mode) := 2, a := -1` |
| M10 | the strafe sign not negated — `left = leftX` | `drive_modes.hpp arcadeRequest` | **RED** | `CHECK( arcadeRequest(pushedLeft).left == 0.5 ) is NOT correct!  values: CHECK( -0.5 == 0.5 )` |

### 2026-09-18 — step 4 done: field_centric.hpp + test 5

Landed `include/shulib/teleop/field_centric.hpp` (`FieldCentricReason`, `FieldCentricState`,
`FieldCentric` with `rezero`/`request`/`state`/`reset`) and appended test 5's six cases to
`test/drive_modes_test.cpp` (the mode's file — the brief's three-file list, and field-centric is
`ArcadeField`'s behaviour).

#### Mutations RUN for test 5

| # | Mutation | Where | Result | Observed message |
|---|---|---|---|---|
| M11 | the latch dropped (the brief's named mutation) — trusted ticks rotate again | `field_centric.hpp request` | **RED** | `CHECK( still.request.forward == 1.0 ) is NOT correct!  values: CHECK( 6.12323e-17 == 1 )  logged: tick := 0` (and `still.state.fellBack` false, `reason` 0 vs 2) |
| M12 | rotate by **+**(heading − zero) instead of **−** (the brief's named mutation, landmine 3) | `field_centric.hpp request` | **RED at 90, 270, 37 and −53 degrees; GREEN at 0 and 180** | `CHECK( got.request.left == doctest::Approx(c.wantLeft)… )  values: CHECK( 1 == Approx( -1 ) )  logged: c.headingDeg := 90` — and the same failure against the independent `math::fieldToRobot` oracle |
| M13 | the driver's zero dropped from the rotation | `field_centric.hpp request` | **RED** | `CHECK( got.request.forward == doctest::Approx(1.0)… )  values: CHECK( 0.798636 == Approx( 1 ) )  logged: zeroDeg := 37` |

M12's green-at-0-and-180 result is landmine 3 measured: a cardinal-only test would have let the
sign through. The odd angles (37, −53) are what close it.

### 2026-09-18 — step 5 done: driver_control.hpp + tests 6, 7, 8

Landed `include/shulib/teleop/driver_control.hpp` (`DriveShaperState`, `DriveShaper`,
`shapeToChassisSpeeds`) and `test/driver_control_test.cpp`.

#### Mutations RUN for tests 6, 7, 8

| # | Mutation | Where | Result | Observed message |
|---|---|---|---|---|
| M14 | slew BEFORE shaping — the two steps swapped (the brief's named mutation for test 6) | `driver_control.hpp shape` | **RED** | `CHECK( got.forward == doctest::Approx(0.1)… )  values: CHECK( 0.000145794 == Approx( 0.1 ) )` — and 0.000145794 is exactly the hand-computed swapped-order value `((0.1−0.05)/0.95)^3` |
| M15 | rotation BEFORE slew — the limiter sees the robot's frame | `driver_control.hpp shape` | **RED** | `CHECK( shaper.state().slewOutput.forward == doctest::Approx(1.0)… )  values: CHECK( 0.9 == Approx( 1 ) )  logged: deg := 45` — the limiter visibly chasing the rotating target |
| M16 | a one-tick buffer between the sticks and the command (the brief's named mutation for test 8) | `driver_control.hpp shape` | **RED**, through the plant | `CHECK( firstMoved == kStepTick ) is NOT correct!  values: CHECK( 6 == 5 )` and, on the release, `CHECK( volts[kReleaseTick] == doctest::Approx(0.0)… )  values: CHECK( 11.4 == Approx( 0 ) )` |
| M17 | the limiters ramp down instead of being reset on a disconnect | `driver_control.hpp shape` | **RED** (the stale-history half only) | `CHECK( back.forward == doctest::Approx(0.1)… )  values: CHECK( 1 == Approx( 0.1 ) )` |
| M17b | the disconnected short-circuit removed entirely — the all-zero request RAMPS DOWN | `driver_control.hpp shape` | **RED** (4 assertions) | `CHECK( sameBits(out.forward, 0.0) ) is NOT correct!  values: CHECK( false )` |

M17 is recorded as a partial catch and M17b was added because of it: M17 left the `return
DriveRequest{}` in place, so only the limiter's stale history was observable. M17b is the
realistic shape of the bug (no short-circuit at all) and the test sees the robot still rolling.

**17 mutations applied and run in total; 16 observed red, 1 (M6) observed green and shown not to
be a defect.** Every one was restored, and the full suite is green at the end of the chunk.

---

## 2. Rulings beyond the brief's §3

Each names the alternative that was rejected and why.

**T2a — `continuousDeadband` uses a CLOSED test (`|x| <= d`), so the threshold value itself is
inside the dead zone.** The brief says "0 inside `(−d, +d)`" and "the value there is exactly 0";
both spellings satisfy that, but the open form reaches ±d through the rescale branch and returns
`-(0.0)` = **negative zero** at `-d`. *Alternative:* keep the strict interior and special-case the
sign. *Reason:* the library has one rule about the sign of zero at the stick mapping (landmine 1,
`stick_mapping.hpp`'s negate-then-deadband) and a closed comparison keeps it with no special case.
Note this deliberately differs from `deadbanded()`, where the threshold PASSES — the two functions
have different semantics and `deadbanded()` was not touched.

**T2b — a setting outside its documented range disables its feature rather than being clamped or
rejected.** `continuousDeadband` with `d` outside `(0, 1)` is the identity; `expoCurve` with
`expo == 0` is the identity; `SlewLimiter` with `rate <= 0` is OFF. *Alternatives:* a precondition
(rejected — a teleop tick must not be able to abort the program, and the finiteness contract's home
is `Chassis::drive`); a silent clamp (rejected — it invents a value the caller never asked for).
*Reason:* these are `constexpr` pure functions with nowhere to report an error to, so they do the
least destructive thing; `AxisShaping::valid()` and `DriverProfile::valid()` are the loud
counterpart, for configuration time rather than tick time. Without this, `d >= 1` divides by zero or
by a negative and hands the driver a silently INVERTED axis.

**T2c — a NEGATIVE slew rate is read as OFF, not as a backwards limit.** *Alternative:* take
`|rate|`. *Reason:* a limiter that ran the axis away from its target is an undebuggable control
inversion; OFF is a failure mode a driver can feel and name.

**T2d — `expoCurve` short-circuits on `expo == 0` rather than multiplying by one.** *Alternative:*
rely on `x * (0*x² + 1)` being the identity. *Reason:* it is not — at `x = inf` the `0 * inf` term
is NaN, so an axis that was merely out of range becomes un-diagnosable garbage. (The mirror of this
is M6: `target * 1.0` IS bit-identical, which is why that mutation stayed green.)

**T2e — a NON-FINITE heading is treated as untrusted and latches the fallback.** The brief names
two inputs (the boot window and IMU_LOST) and says nothing about a NaN heading. *Alternative:*
propagate it, so the request becomes NaN and `Chassis::drive`'s precondition fires. *Reason:*
"trusted" is exactly the claim such a number fails; propagating it aborts the program with nothing
on the panel to explain why, while falling back uses the honest failure path the mode already has
and shows a reason. This is a branch, not a precondition, so the function stays total. It does mean
`FieldCentric::request()` is not `constexpr` (it calls `std::isfinite`/`cos`/`sin`), which the brief
does not require of it.

**T2f — `rezero(heading)` clears the latch unconditionally and takes no trust argument.**
*Alternative:* refuse the press while the heading is untrusted. *Reason:* the brief pins the
signature at one argument, and a refused press leaves the driver pressing a dead button with no way
to tell why. Clearing it means the very next `request()` re-latches if trust is still absent — the
same message, delivered through the indication that is already on the panel.

**T2g — a DISCONNECTED controller short-circuits the whole pipeline and RESETS the three
limiters.** *Alternative:* let the all-zero request ramp down through the slew limiter (the
"natural" implementation). *Reason:* a controller that drops mid-match must stop the robot in the
same tick; a rate limit exists to protect the drivetrain from a driver, not to keep a robot rolling
after its driver is gone. Resetting also means the tick after a reconnect starts from rest rather
than from the command the lost driver last held. Mutation M17b pins it.

**T2h — a controller drop does NOT disturb field-centric's latch.** *Alternative:* clear it (a
reconnect is a fresh start). *Reason:* a lost radio link says nothing about the heading, and
silently restoring field-centric behind the driver's back is exactly the frame flicker the latch
exists to prevent.

**T2i — `setProfile()` rebuilds and RESETS the limiters; field-centric's zero and latch survive.**
*Reason:* carrying a limiter's history across a rate change makes the first tick after a driver swap
obey neither rate. The driver's notion of "away from me" is not a tuning parameter, so it survives.

**T2j — ONE slew rate governs all three axes.** The brief writes `slew : SlewConfig` (singular).
*Reason:* all three axes are dimensionless request fractions in [-1, 1], so one number is
meaningful across them; `SlewConfig` is a struct rather than a bare double precisely so per-axis
rates can be added later without changing a caller's spelling.

**T2k — the compile-time pins in the new test files are narrowed to mutation-insensitive inputs.**
See the M3 note in §1. This follows `stick_mapping_test.cpp`'s own stated precedent.

**T2l — the tank round trip is pinned EXACTLY on a dyadic grid and to 1e-14 off it.** The brief
says "exactly" for "a grid of (l, r)". *Reason:* exactness is not a property of the formula in
general — `(l+r)/2` and `(r−l)/2` recombined are correct to a couple of ulps for arbitrary doubles.
On multiples of 1/16 in [-1, 1] every operation in the round trip is representable, so `==` is the
right comparison and a tolerance there would be hiding something. Both grids are swept, and both
catch the swapped-halves and flipped-sign mutations.

---

## 3. A CORRECTION to the brief (for the coordinator)

**The brief's §4 test 5 sketch — "heading 90° CCW, stick UP ⇒ body request LEFT…" — is wrong. It
is body RIGHT.**

Derivation, twice over:

1. *Physically.* At heading 90° the robot's nose points along field +Y. The driver pushes "away
   from me", which is field +X. Field +X is 90° **clockwise** from the nose, i.e. the robot's
   RIGHT. The body frame's +Y is LEFT, so the request is `left = −1`.
2. *Against the frozen F1 rotation.* `math::fieldToRobot` applies R(−θ): with field (1, 0) and
   θ = π/2 it returns body (0, −1). The test cross-checks every case against it.

The brief's own landmine 3 ("rotated by MINUS the (heading − zero)") is what the implementation
follows, and it is consistent with RIGHT, so this looks like a slip in the illustrative sentence
rather than a design disagreement. **Nothing was implemented from the sketch.** If the brief is
kept as a record, the sentence is worth correcting there.

---

## 4. Documentation pass

### Touched

| File | What changed |
|---|---|
| `docs/api/*.md` + `mkdocs.yml` | **Regenerated** by `tools/api_doc_tool.py generate`. Five new pages (`input_shaping`, `slew_limiter`, `drive_modes`, `field_centric`, `driver_control`), `stick_mapping.md` rewritten from its edited header, and `README.md` / `all-entities.md` re-indexed. **`mkdocs.yml` is GENERATED by that tool, not hand-edited** — its Teleop nav block gained the five pages, and the freshness gate fails without it. See the integration note below. |
| `docs/hardware-assumptions.md` | HA-112 only: the register ROW and the settle ENTRY. The claim widens from "the mapping" to "the mapping AND the feel layer's defaults", the source lines move to the `teleop/` headers, and the entry now says out loud that no claim about how any of it feels is made and that the library has never been driven by a person. **No new HA number minted.** |
| `docs/guide/14-what-it-cannot-do-yet.md` | The "`drive()` is a primitive, not a driver-control product" bullet, rewritten honestly: the shaping now exists and is host-tested, `src/` still calls the raw mapping, and **no driver has felt any of it**. Carries a "this bullet used to say" note, as the file's own convention requires. |
| `include/shulib/teleop/stick_mapping.hpp` | Header comment: T2 has landed alongside rather than replacing; `deadbanded()` and `mapSticks()` marked SUPERSEDED with the two reasons they are kept verbatim; `kStickDeadband`'s and `deadbanded()`'s `///` comments updated. `StickInput` gained `rightY`. **Bodies UNCHANGED.** |
| `docs/internal/PROJECT-BRIEFING.md` | **Generated block only** (`tools/briefing_status.py generate`, run because the build gate reads it). Three lines: the interrupted-chunk list, the suite count, the header count. The coordinator regenerates this at integration. |

### Prose review — every hit read

Grepped `docs/` and `README.md` for `stick_mapping`, `deadband`, `HA-112`, `driver feel`, `T2`,
`field-centric`.

- **`README.md` — no hits at all** for any of the six terms. Nothing stale.
- **`docs/guide/14-what-it-cannot-do-yet.md`** — one bullet, stale, **FIXED** (above). No other hit.
- **`docs/hardware-assumptions.md`** — HA-112's row and entry, stale, **FIXED**. No other hit.
- **`docs/shulib-v2-master-plan.md`** — two hits, both read, both **still true**: line ~84 lists
  "driver feel" among the fall's work (it is), and line ~440 names "field-centric overwrite" as one
  of the legacy bug classes Phase 0 made structurally impossible (it does, and T2's field-centric
  never touches the estimate, so the claim is not weakened). **Nothing to fix**, and the file is
  outside this chunk's §6 list anyway.
- **`docs/roadmap.md` — STALE, and deliberately LEFT for the coordinator** (§6 forbids touching it):
  - `:1545-1549` "Input shaping + drive modes" is `[ ]` and describes this chunk in the future
    tense. It is now host-complete; it should become `[~]` (not `[x]` — `src/` is not re-pointed and
    nothing has been driven), naming T2.
  - `:1550-1553` "Field-centric's honest failure mode" is `[ ]`. The fallback, the re-zero and the
    visible indication all exist and are tested; same `[~]` reasoning.
  - `:194` and `:619-620` are historical session records that say "T2 owns driver feel" / "chunk
    T2's seam" — accurate as history, no change needed.
  - `:1554` "Button bindings / driver macros" is untouched by T2 (that is T3) — correctly still `[ ]`.
- **`docs/internal/build-order.md` — STALE, LEFT for the coordinator** (§6 forbids touching it): the
  T2 entry and the `**Next:**` pointer block. The `Next:` pointer currently names R3d Part 1, which
  is still true for the main tree.
- **`docs/internal/chunks/*`** — historical logs (`R1a-*`, `R3a-*`, `R3b-*`, `C4-*`, `C6-*`,
  `E*`, `F*`, `DOCS2-*`, `DEFECTS1-*`). Read the T2 / HA-112 / deadband hits; all are records of
  what was true when written, and all say "T2 owns…" in the future tense, which is exactly what a
  historical log should say. **Nothing to fix.**
- `docs/internal/verify/verify-*.sh` — the `T2` matches are unrelated (test-case ids). No change.

### Left for integration (the coordinator's files, §7)

1. `docs/roadmap.md` — the two Phase T rows above, to `[~]` with honest wording.
2. `docs/internal/build-order.md` — the T2 entry and the `Next:` pointer.
3. `README.md` — its "what it is not yet" section (no term hits today, so check by eye).
4. `docs/guide/10-the-api.md` — untouched by this chunk, per §6.
5. `PROJECT-BRIEFING.md` — regenerate after integration; the block here carries THIS worktree's
   numbers.
6. **`mkdocs.yml` will conflict with R3d.** R3d is editing it in the main tree at the same time.
   It is a GENERATED file for the API `nav:` block, so the resolution is mechanical: take either
   side and re-run `python3 tools/api_doc_tool.py generate`, which is required at integration
   anyway. The five new Teleop entries are `api/drive_modes.md`, `api/driver_control.md`,
   `api/field_centric.md`, `api/input_shaping.md`, `api/slew_limiter.md`.
7. A `T2-COMPLETED.md` record, and this log's retirement.

---

## 5. Register entries to add (NOT numbered here — the coordinator mints the numbers)

No new hardware BELIEF was created by this chunk: nothing here reads a device, and HA-112's
existing entry was widened to cover the defaults (§4). Two candidates are recorded for the
coordinator to judge, both INVENTED and both currently living only inside HA-112's widened text:

1. **"A continuous (rescaled) deadband is preferable to a hard cut for a human driver."** This is
   the load-bearing FEEL claim of the whole chunk and it is not a measurement — the argument for it
   is analytic (the naive form jumps 0 → 0.05 in one stick count), and nobody has driven either
   form. It is arguably HA-112's business, and arguably its own row because it is the one belief
   whose falsification would change the SHAPE of the code rather than a number in it.
2. **"A driver prefers controls that stay in one frame over controls that silently return to
   field-centric."** The latch's whole justification. Also invented; also falsifiable only by a
   driver.

If the coordinator would rather not grow the register, both are already stated in HA-112's entry
and in the headers' prose, and nothing in the code depends on them being numbered.

---

## 6. DoD (brief §5), with evidence

- [x] **Items 1–7 of §2.**
  1. `include/shulib/teleop/input_shaping.hpp` — `continuousDeadband`, `expoCurve`, `shapeAxis`,
     `shapeRequest`, `AxisShaping`, `DriverProfile`, `kDefaultDeadband` (0.05), `kDefaultExpo` (0).
     Every function `constexpr`.
  2. `include/shulib/teleop/slew_limiter.hpp` — `SlewConfig`, `SlewLimiter` (`update`, `reset`,
     `value`, `ratePerSecond`, `isOff`). Rate 0 is the default and is a bit-identical pass-through
     (pinned over a 4001-point sweep plus `-0.0` and `inf`).
  3. `include/shulib/teleop/drive_modes.hpp` — `DriveMode{ArcadeBody, ArcadeField, TankSticks}`,
     `arcadeRequest`, `tankRequest` (two overloads), `modeRequest`. `StickInput::rightY` added
     additively; `mapSticks` ignores it and the src build gate proves no caller broke.
  4. `include/shulib/teleop/field_centric.hpp` — `FieldCentricReason`, `FieldCentricState`,
     `FieldCentric` with `rezero(heading)`, `request(...)`, `state()`, `reset()`, the LATCHED
     fallback, and no reference to any IMU, localizer or chassis.
  5. `include/shulib/teleop/driver_control.hpp` — `DriveShaperState`, `DriveShaper::shape(...)` in
     the order mode → shaping → slew → rotation, `state()`, `rezero()`, `setProfile()`, `reset()`,
     `fieldCentric()`, plus `shapeToChassisSpeeds()`.
  6. One-tick latency pinned through the A2 plant: `test/driver_control_test.cpp`, two cases (the
     step and the release), mutation M16 red.
  7. `stick_mapping.hpp`'s header updated; `deadbanded()` and `mapSticks()` BODIES untouched.
- [x] **Every public member `///`-documented** — `api_doc_tool.py check-coverage` reports
      `doc coverage: 1846 public entities across 130 headers, all documented`, with the five new
      headers in the target set (the generator emitted a page for each). *The BEFORE figure was
      not captured — the first gate run of the chunk already had the new headers on disk, so only
      the after-value is measured and only it is stated here.* No `docs/internal/` path in any
      header comment
      (`check-removability` clean; two prose references to the build order were reworded when the
      gate caught them — recorded here because that gate reads the GENERATED page, so a `//` file
      header is in scope too, not only `///`).
- [x] **ARM gate green with the new headers** — 164 headers in the generated TU (163 headers +
      `main`), `arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion
      -Wshadow -Werror -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp`, exit 0.
- [x] **Tests 1–10 green; mutations logged red.** 39 new cases / 713 assertions across the three
      files; 17 mutations run, 16 red with the message pasted, 1 (M6) green and explained.
- [x] **`test/stick_mapping_test.cpp` still green and UNCHANGED** — `git diff HEAD --` on that path
      is empty; its 6 cases / 349 assertions pass.
- [x] **The whole suite green** — 1247 cases / 1,563,890 assertions / 0 failed / 3 skipped
      (baseline 1208 / 1,563,156). Host build under
      `-Wall -Wextra -Wpedantic -Werror -Wshadow -Wconversion -Wsign-conversion -Wdouble-promotion`
      with no warnings.
- [x] **PROS-free and layering guards** — both CI greps run verbatim, both clean. `include/shulib/`
      outside `hal/pros/` has no `pros/` include; nothing outside `sim/` includes `shulib/sim/`.
      `include/shulib/teleop/` has no `std::string`, no `std::function`, no `new`/`malloc`, no
      `<vector>`/`<memory>` (the only grep hits are the words in prose comments).
- [x] **`docs/api/` regenerated** — freshness gate green.
- [x] **No `src/` file touched** — `git status --porcelain src/` is empty; and
      `python3 tools/src_build_gate.py check` **PASSES all five builds** (bench, xdrive, tank,
      tank-drive, tank-library), which is the evidence that the `StickInput::rightY` addition broke
      no existing caller.
- [x] **No plan document touched** — `docs/roadmap.md`, `docs/internal/build-order.md`,
      `README.md`, `docs/guide/10-the-api.md` all unmodified. `PROJECT-BRIEFING.md` carries the
      generated block only (§4).
- [x] **This log complete** — rulings §2, the brief correction §3, the prose review §4, the
      register candidates §5, the mutations in §1, the counts here.

### Counts

| | Before | After |
|---|---|---|
| Suite cases | 1208 | **1247** |
| Suite assertions | 1,563,156 | **1,563,890** |
| Skipped | 3 | 3 |
| Failed | 0 | **0** |
| Public headers | 158 | **163** |
| Documented public entities | *(not captured)* | **1846 across 130 target headers, all documented** |

### Not done / partial — stated before anything is claimed

- **`src/` is NOT re-pointed.** `src/main.cpp`'s `runTeleopLoop`, `src/drive_program.cpp` and
  `src/bench_r3a.cpp` still call `mapSticks` / `mapSticksToChassisSpeeds`. That is brief §7's
  integration work and deliberately out of scope here, but it means **nothing in this chunk runs on
  a robot yet** and the roadmap rows must be `[~]`, never `[x]`.
- **No driver has felt any of it.** Every default is invented. The only comparison made anywhere is
  to the OLD MAPPING's numbers, which is a compatibility argument, not a feel argument.
- **Perceived latency is not claimed.** The one-tick property is pinned through the host plant;
  real input-to-motion latency is PROS call cost plus loop rate, both unmeasured.
- **No panel or button binding exists.** `FieldCentricState` is produced but nothing displays it,
  and `rezero()` is not bound to a button — both are integration (§7) and T3.
- **`mkdocs.yml` is expected to conflict with R3d** (§4, item 6). It is machine-generated; the
  resolution is to re-run the generator.
- **The worktree's base commit had to be corrected** before any work started (§0).
