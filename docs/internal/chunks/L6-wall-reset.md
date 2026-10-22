# L6 — the WALL RESET with the side distance sensors: one pose axis re-seated against a field wall from a trustworthy side-sensor reading — the geometry from a measured mount, the gates that REFUSE and REPORT, a one-axis seat in the Localizer, the auton's call, the host test plant's field and sensor model, and Step 0 at the robot

> **THE TEAM LEAD'S ANSWERS (2026-10-22)** to "Decisions for the team lead": **"Accept all (Recommended)."**
> - **D1 (a) — DECIDED, as recommended:** L6-A1 (the library core) on his go; L6-A2 (the robot's side sensors and the
>   everyday call) when VIS1-A1 merges; L6-B (robot two, the WALL bench page, the tester's wall line) before MECH-A5
>   launches; L6-C (a `Routine` member) optional, after MECH-A4.
> - **D2 (a) — DECIDED, as recommended, AMENDING locked decision #4:** a ONE-AXIS seat (`Localizer::seatAxis`) — that
>   axis only; the other axis, the heading and the drift term untouched; never `setPose`. Decision #4 now reads "…
>   `relocalize()` feeds this corrector — it does not snap, **except a caller-invoked ONE-AXIS wall reset (L6), which seats
>   that axis only while no motion is active, inside gates whose largest jump is the author's `maxCorrection`; no sensor
>   ever seats the heading**" (recorded beside #4 in the master plan, 2026-10-22).
> - **D3 (a) — DECIDED:** a refused reset leaves the estimate untouched and the auton goes on, unless the call is `required`.
> - **D4 (a) — DECIDED:** an auton calls `competition::resetFromWall(ctx, …)` through `then()` now (FIELD starts only).
> - **D5 (a) — DECIDED:** the side sensors' facts live in the robot file (VIS1's template); a refused sensor never refuses the robot.
> - **D6 (a) — DECIDED:** each mount MEASURED and typed once per robot (unmeasured ⇒ every reset refuses `MOUNT UNSET`).
> - **D7 (a) — DECIDED:** the gates are provisional library defaults; `maxCorrection` has NO default.
> - **D8 (a) — DECIDED:** the season's keep-outs by default (the Toggle and its brackets, the Loaders, the corners, the
>   VEX U Load Zones, the start pieces until the beam's cone is measured).
> - **D9 (a) — DECIDED:** no heading correction; no background "correct when it can" mode.
> - **D10 (a) — DECIDED:** mounting advice — lens 10–10.5 in up, at the fore-aft centre, ≥ 2.5 in inboard, square, clear.
> - **D11 — DECIDED:** cookbook 05's re-seed recipe (it overwrites the other axis) is fixed (in L6-A1).
> - **D12 — DECIDED:** REMAINING-WORK row 25 reworded (a robot parked beside a wall re-seats against it).
> - **D13 — DECIDED:** when job slots are short, the order the brief gives.
> - **Parts and sizes:** L6-A1 ~2¾–3½, L6-A2 ~1¾–2, L6-B ~1½–2, L6-C ~½–¾ builder-days (~6–7½ in all); the useful
>   first landing is A1 + A2 + B. HA-204..HA-212 (L6 was accepted before IMU1, so it keeps them).

> **Status 2026-10-18, drafted by 2026-10-22; RE-CHECKED 2026-10-22 in two passes against three independent checkers
> (tree, safety, team use) — "## Re-check — amendments" below, RC1–RC48, every body change marked where it sits:
> DRAFT for the team lead's go — WAITS for his answers to D1–D13. Nothing is built.** The re-check's file reads are at
> the tip **`71b4764`** (the VIS1 plan commit; `ceb17c0` + docs only); `c09e022` (tooling — the bounded suite runner)
> landed during the re-check and moves no line this brief cites outside `tools/` and four internal documents. Written by the coordinator's brief writer
> at tree **`ceb17c0`** (branch `shulib-v2`; the main worktree carries the coordinator's uncommitted plan edits, §6.1). In flight while this was written, NOT edited (every worktree
> read only): **MECH-A3** (`.claude/worktrees/mech-a3`, on `fbaf729`, building — its changed files listed in §6.1),
> **CAL1 Part A2a** (`.claude/worktrees/cal1-a2a`, on `ceb17c0`, just started — only its two logs changed). **The
> VIS1 brief got the team lead's "Accept all" on 2026-10-18** (the coordinator's plan commit was being written in the
> main worktree while this brief was — uncommitted edits to `build-order.md`, `REMAINING-WORK.md`, the HA register's
> Held line now carrying HA-192..HA-203, the worksheet; read, not touched): **VIS1-A1 is the next run, beside
> MECH-A3**, and it edits `config/robot_config.hpp` and `robot/robot.hpp` — which is why L6's library half is split
> in two (D1). VIS1's §6.7 already reserves a template for L6's sensors (D5, §6). **Line numbers in this brief are
> `ceb17c0`'s**; the plan commit moves some of `build-order.md`'s and `REMAINING-WORK.md`'s — re-check at the tip.
>
> **The team lead's words this brief holds (BINDING).**
> - 2026-10-08 (Q2 answered): **one V5 Distance Sensor on each SIDE of the robot (left and right), facing straight
>   out** (`docs/internal/REMAINING-WORK.md` :104 at `ceb17c0`, **:107 at `71b4764`** — row 25; *re-stamped by RC32*).
> - 2026-10-14: "we definitely need distance sensors working and we need ai vision camera, objects" — **L6 BEFORE
>   THE PANEL, beside the MECH core, before the first competition (2026-11-23) if it can** (`build-order.md` :2188 at
>   `ceb17c0`, **:2224 at `71b4764`**, the deviations table; `REMAINING-WORK.md` :107 at the tip — *RC32*).
> - Both robots are 15" this season (2026-10-08); robot two (the tank) carries distance sensors on ports **4 (left)
>   and 5 (right)** (`R2a-sensor-stations.md` :14-17, the census of 2026-09-24); **robot one has no build yet**.
> - **L6 must NOT depend on the AI Vision camera** (VIS1, awaiting his go). It does not: nothing here reads a camera.
>
> **Executor model: `opus`. Do not commit. Live log `docs/internal/chunks/L6-PROGRESS.md`, created FIRST** (a heading
> per run). Say "host test plant", never "simulator". **No hardware claim anywhere:** nothing here has run on a
> robot; Bench Tests station 11 (worksheet Station H) was uploaded and **never run** (`REMAINING-WORK.md` :166-167; :169-170 at `71b4764`),
> so **no distance-sensor reading of this robot exists**. **Hardware Assumptions: HA-204..HA-212 ONLY** (§4; the
> register's Held line gains them at the plan commit — HA-192..HA-203 are VIS1's; a tenth belief ⇒ stop and tell
> the coordinator). Every `file:line` below was OPENED at `ceb17c0` (`~:N` = within a few lines). **Memory rule
> (2026-10-15):** every binary built — probes, the suite, any test — runs as `(ulimit -v 4194304; timeout 300 …)`.
>
> **The manual.** VEX V5RC *Override* Game Manual **v2.0**, local text `.claude/coordinator/scratch/autoflip/v20.txt`
> and its Appendix A drawings rendered as `scratch/autoflip/img/a-103.png` (A8, Toggle), `a-104.png` (A9, Loader),
> `a-105.png` (A10, field element locations), `fo-019.png` (Figure FO-1). Whether v2.1 changed a dimension is
> UNVERIFIED (VIS1's header records the 404s of 2026-10-15); **the executor re-checks A8–A10 against the newest
> manual before L6-A copies a number** and says which version it used.
>
> **Continuation 2026-10-22 (the same writer's task, resumed after the 2026-10-18 ~23:30 stop; main still `ceb17c0`).**
> §0–§4 were re-read against the tree and the worktrees; §5–§9 were written. Seven things in §0–§4 were found wrong
> or incomplete and are **corrected in place** (each correction says so where it sits):
> 1. **The API version (§1.8, §2.1, §1.11).** The draft bumped `kApiMinor` at every landing. The project's practice is a
>    bump only for growth of a FROZEN surface (`version.hpp`'s `kApiMinor` doc: "additive extensions of a frozen
>    surface"; every "since 2.4" changelog entry says "No version bump" for unfrozen surfaces, e.g. COMP-a's
>    `Localizer::reseatHeading`, `changelog.md` :1086-1090). **L6-A1, A2 and B bump nothing; only L6-C (F10's
>    `Routine`) bumps.**
> 2. **The wall height IS printed** (D10, §0.3, HA-208): A13 (`img/a-108.png`, portable perimeter) **11.54 in**, A14
>    (`img/a-109.png`, metal perimeter) **11.50 in** — viewed 2026-10-22; VIS1-A1's executor read the same
>    (`vis1-a1/docs/internal/chunks/VIS1-PROGRESS.md`). Only the Toggle's overhang is still read by eye.
> 3. **The two perimeters differ** (§0.3, §1.1, HA-207): inside wall to wall **140.40 in** (A13, 3566.4 mm — A10's
>    140.41) on the portable, **140.50 in** (A14, 3568.7 mm) on the metal — walls at ±70.20 vs ±70.25, 0.045 in apart.
> 4. **D2 (b)'s quality claim is now MEASURED** (probe P4, §0.4) — it was read from `refreshQuality()` only.
> 5. **D8's file name** said `field/override_2026.hpp`; §1.1's `localization/override_walls.hpp` is the one.
> 6. **The boot line and the provider order after VIS1-A1** (§1.5, §1.9): VIS1-A1 (worktree `vis1-a1`, building)
>    replaces "vision ABSENT" by camera words printed LAST and builds the camera in the constructor BODY, last; L6-A2
>    follows both, and the pinned boot-line tests (not listed in the draft) change — *the set corrected by RC26*.
> 7. **Pace rule 7** (§2.1, §3): the draft's L6-A1 carried ~10 public documents under a "≤ ~5" heading. Re-allocated:
>    the narrative guide section moves to L6-A2 (where a robot can first use the reset), the master plan's two
>    sentences move to the plan commit, guide 02's "about −70 to +70" is NOT wrong ("about") and is dropped.


## Re-check — amendments (2026-10-22, the same writer, after three independent checkers: tree, safety, team use)

Each checker finding was re-verified by this writer (files re-opened at `71b4764`; probes re-run under
`(ulimit -v 4194304; timeout 300 …)`). Each entry says REAL / PARTLY / NOT REAL, the evidence, and where the body
changed; every body change is marked *(RCn)* where it sits. Overlapping findings are merged into one entry.
- **First pass (RC1–RC5, 01:28–01:38):** probes in `scratch/brief-L6/recheck/`; the brief before it is
  `recheck/L6-wall-reset.before-recheck.md`. Its list reached this writer CUT OFF inside the team lens's sixth finding
  (RC22); the pass was STOPPED (usage limits) after RC5 — **RC4 and RC6–RC22 of that list were never written**, and
  their text is not on disk. Its probes show three of them in progress (`recheck/r3.txt`'s "RC13" height-gate rows,
  "RC14" clear-panel rows, "RC17" along-error rows; `recheck/r4_required.cpp`, a compile-time `maxCorrection`).
- **Second pass (RC23–RC48, from 09:10):** the checkers ran again over the half-amended brief and re-raised every one
  of those still open, with new ones; their list also arrived cut off (inside the team lens's sixth finding, the
  RELATIVE-start one — RC48 dispositions it from what arrived). The first list's open numbers are CLOSED by the second
  list's entries, not re-numbered: r3's RC13 → **RC34**, RC14 → **RC37**, RC17 → **RC33**; r4's compile-time field →
  considered and rejected in **RC45**. Probes in `scratch/brief-L6/recheck2/` (`q1_budget`, `q2_gates`, `q3/` the
  amended header layout); the brief before this pass is `recheck2/L6-wall-reset.before-pass2.md`. **The first pass's
  finished entries were checked here:** every body change RC1, RC2, RC3 and RC5 name had landed (D2 (a), §0.2, §1.3, T6,
  landmine 14, §7 — RC1; the `MotionActive` verdict, rule 1, §1.3's contract, the plain words, T4 and M32 — RC2; D8,
  §1.1, §6.1, §6.4, §7, landmines 8 and 11 — RC3; D4's qualified listing — RC5), except RC5's own stray reference
  (corrected, RC23); D4's listing has since been rewritten (RC45) and still qualifies every L6 name as RC5 requires.

**RC1 — M18 is an equivalent mutant (tree, major). REAL — fixed at the root by DROPPING the odometry shift (the
checker's option (b)); and re-probing found a SECOND equivalent mutant (M19 under the complementary tier).**
Re-verified: the Localizer moves its position by odometry DELTAS only (`localizer.hpp` :266-281); nothing in
`include/` or `src/` reads an odometry's ABSOLUTE pose (grep `odom…\.pose()`: only the Localizer's :235-236, :267,
:526-527, and tests); the drive program's cross-check reads `lastHeadingDisagreement()` and `lastSideTravel()`
(`drive_program.cpp` :378-397), per-tick values. Probe R1 (`recheck/r1_seat_variants.cpp` → `recheck/r1.txt`: a
Localizer over `PilonsOdometry` and fakes, seat x by −2 in, then still ticks and a further drive, under BOTH fusion
tiers): the draft's seat, M18 (no odometry shift) and the adopted form (no shift, no odometry re-take) give
**identical Localizer observables in both tiers** — only the odometry's own pose differs (8.0 vs 10.0). **M19
(`hasLast_` left true) is ALSO invisible under `ComplementaryFusion`** (the twist is differenced against `lastFused`,
which the seat re-takes) — but under **`EkfFusion`**, which keeps its own position state, the seat reads as ONE
TICK'S ODOMETRY INCREMENT (`ekf_fusion.hpp` :450, `ux = px − lastX_`): **1.68 in of a 2-in seat bounces back and the
filter's velocity reads 167.6 in/s.** So `hasLast_ = false` (next `dt` 0 → `resync`, :441-444) is load-bearing and
only an EKF-tier test sees it. Why (b), not (a): the shift is not free — `IOdometry::setPose` re-takes the heading
reference (`prevHeading_ = imu_.heading()`, `drive_encoder_odometry.hpp` :219, `pilons_odometry.hpp` :203), a side
effect a one-axis seat has no reason to cause; `reseatHeading` already leaves the odometry untouched (:544-547); and
an applied correction already lets odometry and estimate diverge by design (STEP 2's note, :246-249), so `setPose`'s
"never diverge" is not an invariant the Localizer keeps elsewhere. **Changed:** D2 (a); §0.2's odometry row; §1.3
(no shift; `hasLast_` named load-bearing with R1's numbers; the doc's reason); T6 (BOTH tiers; M18 deleted; M19
restated as RED under the EKF only; a new M18b — the dropped shift restored — red on "the odometry's pose
bit-identical"); landmine 14; §7's L6-A1 spot check (M19 under the EKF).

**RC2 — No "no motion active" guard (tree, major). REAL — fixed.** Re-verified: `reseatHeading`'s contract
(`localizer.hpp` :549-553) requires that NO motion be active, and `runAuton` enforces it for the start seed
(`match_runner.hpp` :147-149, "the start pose cannot be re-seated under it"); the draft's rule 1 checked only
liveness and finiteness; `Chassis::waitUntil` ticks any armed motion (`motion_scheduler.hpp` :833-848). Also checked
the other side: every `Routine` motion step is blocking and detaches before it returns (`chassis.hpp` :556-566,
`runBlocking`), so a team's ordinary `driveTo → then(reset)` never trips the new gate — it bites a Tier-3 `async()`
motion, exactly the case that would move the estimate under a controller. **Changed:** the verdict enum gains
`MotionActive` (rule order; the enum is new in L6-A1, so "append-only" starts at its landing); rule 1 refuses it
before any sample; §1.3's `seatAxis` doc carries `reseatHeading`'s no-motion contract; the plain-words list; T4
gains the armed-hold case (no time, nothing read, the hold still armed) and mutation **M32** (the check dropped).

**RC3 — VIS1 does not use `shulib::override2026` (tree, major). REAL — corrected.** Re-verified: VIS1-A1's working
header opens and closes `namespace shulib::vision::override2026` (`vis1-a1/include/shulib/vision/override_elements.hpp`
:33, :123); VIS1's brief §1.10 (:1102-1112) names none. The draft's shared-namespace premise, the redefinition rule
built on it (§6.4, landmine 11) and the summary's line were wrong. **Chosen on its merits:**
`shulib::localization::override2026` — season data beside the subsystem that consumes it (VIS1's pattern), so the
two headers can never collide. The one hazard that remains is lookup: inside `namespace shulib::vision` (AF1's code),
an unqualified `override2026::` finds VIS1's — L6's listings always write `localization::override2026::`. §6.4 keeps
only the agreement test, still valid: VIS1-A1's `kTogglePlaces` use 70.2 (:76), L6's 1783.2/25.4 = 70.2047, inside
0.01 in. **Changed:** D4's example, D8, §1.1 (the namespace and why), §6.1's VIS1 row, §6.4 (rewritten), §7's ARM-gate
comment, landmines 8 and 11, §9.

**RC5 — The D4 example does not compile in a student's auton file (tree, minor). REAL — fixed.** Re-verified:
`src/team/autons.cpp` is `namespace shulib::bench` (:30); the checker's transcription (`check-tree/out/d4.cpp`)
fails on the unqualified `WallSide` and compiles once it is `shulib::localization::WallSide`. **Changed:** D4's
example qualifies every L6 name (and is re-shaped by RC45 and RC48 — the parking and the square-up, and the context
form; *reference corrected in the second pass: this entry first named "RC18 and RC26", entries the stopped first pass
never wrote*); the
rule for every listing derived from it (guide 03's section, cookbook 05's recipe — `check-examples` compiles them)
is the same: every L6 name qualified as the auton file would need it, compiled inside `namespace shulib::bench`.

**RC23 — The brief was half re-checked, yet its header and the relayed summary said it was finished; RC5 cited entries
that did not exist; an internal tag sat in a student-facing listing (tree, major; safety, major — first half; team,
major). REAL — fixed by this pass.** Re-verified: the section held RC1, RC2, RC3, RC5 only (`grep -n '^\*\*RC'`), the
brief's mtime was 01:38 and `recheck/rc/` held rc01–rc03 and rc05; RC5 said D4 "is re-shaped by RC18 and RC26" while
D4 (then :188-192) was the draft's form, and "RC26" contradicted the preamble's "cut off … (RC22)"; the D4 listing
carried `(RC5)` in a code comment, which a cookbook copy would leak and §7's C7 grep (`internal/|chunks/|RESUMING|
build-order`) does not catch; the summary relayed with the task still said both season headers use
`shulib::override2026`, which RC3 had withdrawn (VIS1-A1's header is `shulib::vision::override2026`,
`vis1-a1/include/shulib/vision/override_elements.hpp` :33, :123 — re-read today). **Changed:** the header's status (two
passes, RC1–RC48, the tip `71b4764`); this section's preamble (what each pass did, and that the first list's RC4 and
RC6–RC22 are CLOSED by the second list's entries, with the mapping the probe files prove); RC5's stray reference
(→ RC45, RC48, marked as corrected); the `(RC5)` tag removed from the listing; §7 gains a grep that no `RCn` tag
reaches a guide, cookbook, header or compiled listing; **§9 is regenerated from the amended body at the end of this
pass (after RC48)**, and the summary returned to the coordinator is built from §9, not from the draft's.

**RC24 — §1.2/§1.3 as written did not compile (an include cycle), a `localization/` header reached up into the facade,
and the listing was not C++ (tree, major). REAL — fixed at the root (a leaf `axis.hpp`, a PURE `wall_reset.hpp`, the
calls at the layers that own their arguments), and re-probed.** Re-verified: `chassis.hpp` :184 and :187 include
`motion/command_pipeline.hpp` and `motion/motion.hpp`; `motion.hpp` :140 includes `localization/localizer.hpp` and uses
`localization::Localizer` at :187 and :260 — so `localizer.hpp` (needing `Axis` for `seatAxis`) → `wall_reset.hpp`
(declaring `resetFromWall(chassis::Chassis&, …)`) → `chassis.hpp` → … → `localizer.hpp` is a cycle; the checker's three
include orders all failed (`check-tree/round2/tu/axis_variant.txt`, read). Master plan §5 (:197) forbids the upward
include; no `localization/` header includes `chassis/`, `motion/`, `sequence/` or `competition/` today (grep). The
listing's `{2 deg}` is not C++ (`_deg` builds a non-`constexpr` `math::Angle`, `units/literals.hpp` :64-72);
`WallSample`/`kMaxWallSamples` were used, never declared; D4 (a) gave the explicit form 4 parameters, §1.2 five; D6
said `facingDeg`, §1.2 `facing` (an `AngleDim`). **Fixed:** `Axis` in the NEW leaf `localization/axis.hpp`;
`wall_reset.hpp` PURE; the explicit call in the NEW `chassis/wall_reset_call.hpp` (namespace `chassis`, as
`routine.hpp` sits over `chassis.hpp`); the everyday call in the NEW `competition/wall_reset.hpp` (RC48 — it takes the
auton's context); the listing rewritten in real C++ (`N * kWallDegToRad` defaults, `WallSample` and `kMaxWallSamples`
declared, the explicit form's 4 parameters everywhere — the heading tolerance is now DERIVED inside the call, RC38;
`facing` an angle in D6). **Probe Q3** (`recheck2/q3/`; `q3_layout.txt`): the amended layout — leaf, pure header,
season walls as `constexpr` data with their keep-out kinds, the two calls, the robot file's block in a `constexpr`
table, and D4's amended listing inside `namespace shulib::bench` — compiles with the suite's flags in both include
orders (rc 0 ×4), and every v2 header plus these in ONE unit compiles with the ARM gate's flags (rc 0). P3 is restated
as covering the prototype only. **Changed:** §1.2's heading, its note and listing; D4 (a)'s first sentence; §0.4's P3;
§2.1's L6-A1 file list; §3.1's changelog line; §7 gains the "PURE" include grep. (§1.3's `#include` of the leaf: RC33.)

**RC25 — D2 changes a LOCKED decision but presented itself as a scope note outside it, leaving out the sentence that
rules on a caller-invoked relocalize (tree, major). REAL — fixed.** Re-verified: master plan :547 (decision #4,
LOCKED 2026-06-19) ends "`relocalize()` feeds this corrector — it does not snap"; :409 sends a tag fix "into the gated
corrector (… nudge … *never a snap*)"; :639 lists `relocalize(featureSource)` among the alignment primitives. A wall
reset is a caller-invoked relocalization against a feature, and D2 (a) snaps one axis, so the locked text rules against
(a) and for (c); the draft's "governs correctors that run every tick unasked … outside this decision" contradicted it
(grep of the draft: no "relocalize"). **Changed:** D2 (a) quotes decision #4 in full, says plainly that (a) AMENDS it,
states the amendment sentence the team lead would accept (an exception for a caller-invoked one-axis wall reset with no
motion active, bounded by `maxCorrection`, never the heading), and why the hazard never-snap guards (risk 5, a pose jump
under a running motion) cannot arise under rule 1's `MotionActive` gate (RC2); D2 (c) is labelled as the form the locked
text prescribes; §3.3's plan-commit item becomes the amendment, applied ONLY on the team lead's acceptance of D2 (a),
recorded in the decision's row; §0.2's row and §3.4's prose grep (`relocalize` added). The recommendation stays (a) —
on its stated reasons — and the choice is visibly his.

**RC26 — The boot-line pins L6-A2 breaks: VIS1-A1's NEW `robot_camera_test.cpp` was missing, and `robot_builder_test`
survives (tree, minor). REAL — fixed.** Re-verified in VIS1-A1's working tree (`.claude/worktrees/vis1-a1`, read only):
`test/robot_camera_test.cpp` :344 and :366 use `endsWith(banner(robot), "tags ABSENT camera …")`, :401 builds `head = "…
GPS ABSENT tags ABSENT "` and :404-429 compare `banner == head + "camera port 3 …"` (eight times) — every one turns red
when `walls %s ` is inserted between "tags ABSENT" and the camera's words (§1.5); `boot_source_test.cpp` :392 ends its
head with "tags ABSENT " and `xdrive_config_test.cpp` :234 is exact ("… tags ABSENT camera ABSENT") — both change;
`robot_builder_test.cpp` :882-887 uses `find("tags ABSENT")` and ends-with "camera ABSENT" — both still hold. VIS1-A1's
guide 10 :579-582 shows "… tags ABSENT camera …" lines. **Changed:** §1.5 (the exact set, ~10 pins, and the guide's
lines), §1.9, §2.1's L6-A2 row, §6.1's VIS1-A1 row, §6.2's row, the header's continuation note 6. (§9 at the end.)

**RC27 — L6-B's boot plumbing ran through a call that does not exist; the call sites and the shared-file row were
wrong (tree, minor). REAL — fixed.** Re-verified at `71b4764`: `decideBootSource()` (`src/common/boot_source.cpp`
:35-56) calls `decideBootSourceFrom(record, registry, fromTable, missing, kMechanisms, kChassis.robot, <printer>)`;
the template (`boot_source.hpp` :468-475) ends in a forwarding `PrintLine&&` — a defaulted parameter cannot follow it —
and calls `bootConfigFrom` itself (:587); `bootConfigFrom` has a forward declaration (:251) and a friend (:281);
`drive_program_source_test.cpp` :163 pins the call's text; `decideBootSourceFrom` has seven test call sites (listed in
§1.10); `bootConfigFrom` 15 in `boot_source_test.cpp` (grep: 15, the checker's 14 + the comment's). VIS1 §1.15
(:1240-1241) names no mechanism for the camera's copy; the MECH brief (:1913-1914) has L9 Part B moving
`decideBootSourceFrom` into the library with "the sensor ports". **Chosen:** a NEW overload of `decideBootSourceFrom`
with the walls before the printer (the old signature forwards with no walls — zero call-site churn), the copy into
`out.built.config.sensors.walls` after `bootConfigFrom`, and a named refusal for a colliding port in the claws' style
(the boot names the fix, not just the collision) — over a post-patch in `boot_source.cpp`, which would leave the host
decision (what tests reach) without the walls and skip the named refusal. **Changed:** §1.10's plumbing and tests; §2.1's
L6-B files (+ `drive_program_source_test.cpp`); §2.2's L6-B line; §6.2's boot_source row (VIS1-B's real claim, L9 Part B).

**RC28 — Coordination gaps: MECH-A5 on L6-C's files, the launch templates' "next free HA-204", incomplete in-flight
file lists (tree, minor — parts (a), (c), (d); part (b), D13's order, is RC29). REAL — fixed.** Re-verified: the MECH
brief's §2 table (:2134) has MECH-A5 writing `src/programs/{competition_program,competition_run,auton_tester_program,
auton_tester_run,…}`, two of which L6-C edits, and §6.2's L6-C row named only MECH-A4; `grep -o 'next free HA-[0-9]*'
.claude/coordinator/drafts/*.json` → `mech-a4-args.template.json` and `vis1-a2-args.template.json` say HA-204 (every
older args file HA-192); `git status` in `mech-a3` (2026-10-22 ~09:20) also lists `control/trapezoid_profile.hpp`,
`test/mechanism_fixtures.hpp`, `test/mechanism_kinds_test.cpp` and the `Makefile`; in `cal1-a2a` the changelog,
README, HANDOFF, ORIENTATION, REMAINING-WORK, build-order, the roadmap, the briefing, `docs/api/` and `mkdocs.yml`.
**Changed:** §6.1's MECH-A3 row (its real file list and state: paused mid-fix) and CAL1-A2a row ("no shared CODE file",
its documents named); §6.2's L6-C row (MECH-A5 added, the order stated — L6-C itself is now optional, RC48); §6.6 (the
plan commit moves the templates' "next free" id). The `Makefile` in MECH-A3's status is reported, not judged — it is
the coordinator's resume audit's to read (the "leftover test code" check).

**RC29 — D13 understated what L6 costs the 11-23 critical path, left VIS1-A1 and CAL1 out of its one-slot order, and
the "~2026-11-01 under D13 (a)" date was the parallel estimate (tree, minor — part (b); team, major). REAL — D13
rewritten with the slot arithmetic.** Re-verified: the brief's own sizes (A2 ~1¼–1½ + B ~1–1½, each + ½–1 of
verification) made "A5 starts ~2½–3½ builder-days later" too small, and L6-A1 (2–2½ + ½–1) sat before MECH-A4 unmentioned;
L6-A2 waits on VIS1-A1's merge (§6.2) but D13 (a) never placed VIS1-A1, and VIS1's accepted rule is "if a slot is short,
VIS1 waits, never MECH" (`VIS1-ai-vision-colours.md` :1690-1691); the MECH brief's §2 title: "CAL1 before A5 (N9)"; VIS1-B's
accepted slot is after A5 and before A6 (build-order :1693), not "after L6-B"; one-slot mode is in force (VIS1-A1's log,
"the team lead dropped to one job near a usage limit", ~01:45 today). The sizes are re-taken from the MECH brief's §2
table (A4 ~3¾–4¾, A5 ~3¾–5, A6 ~2–3), CAL1's (A2a ~1¾–1⅞, A2b ~¾–1, B ~1⅞–2⅛) and L6's as amended (A1 ~2¾–3½, A2
~1¾–2, B ~1½–2 — each entry's share named in §2.1). **Changed:** D13 — (a) a second slot (MECH untouched, L6-B ~Oct 30 – Nov
3), (b) one slot with L6 right after MECH-A4 (L6-B ~Nov 5–10; MECH-A5 ~8–12 slot-days later, ~Nov 14–22, at or past
the freeze; VIS1-A1's rest ahead of MECH amends VIS1's rule), (c) MECH first (L6-B ~Nov 12–22 or later); recommended (a),
and (b) with one slot, citing the team lead's own 10-14 ranking ("definitely" the distance sensors; the MECH core "only if
frozen ~Nov 16"); D1 (a)'s sizes; §2.1's size cells and the 11-23 paragraph; §6.2's "Order, all told". The dates assume
one slot-day per calendar day — an assumption, said so.

**RC30 — P4's "ONE tick later" was two ticks; the first tick after either seat reads Degraded (tree, minor). REAL —
reworded; D2 (b)'s conclusion stands.** Re-verified: `probes/p4_quality.cpp` :33 waits `waitUntil(false, 0.02 s)`, two
10-ms ticks; this writer re-ran the checker's per-tick binary under the memory cap (`check-tree/round2/tu/p4pt`): seatAxis
— tick 1 Degraded q 0.100, tick 2 Degraded 0.200; setPose — tick 1 Degraded 0.500 (the dt-0 re-base: `hasLast_` false,
`localizer.hpp` :244, :588), tick 2 DeadReckon 1.000. **Changed:** D2 (b), §0.4's P4 line, landmine 2 ("from the second
tick"), T6 (the quality's bit-identity scoped to the instant `seatAxis` returns; the per-tick values pinned); §9's P4 line
at the end.

**RC31 — The scheduler's pose-plausibility guard was never mentioned; a seat over 2.25 in raises IMPLAUSIBLE on a
caller-paced tick, and a timed-out frozen wait would add a second log line (tree, minor). REAL — landmine 23, two
test assertions, a guide sentence.** Re-verified: `diag/plausibility_guard.hpp` :63 and :69 (maxSpeed 150 in/s, margin
1.5) and :116 (`maxDist = maxSpeed × margin × dt`); `motion_scheduler.hpp` :1106-1111 checks the pose every tick unless
Uninitialized; the loop monitor returns dt 0 on its first tick after a reset (`loop_monitor.hpp` :84-90), and the
scheduler resets it at `async()` (:761) and at both waits (:798, :830); :838-844 log "waitUntil timed out" as an `SCH`
Warn. This writer re-ran the checker's probe under the memory cap (`check-tree/round2/tu/sg`): a 2.5-in seat then a
blocking `moveTo` → 0 IMPLAUSIBLE; then a caller-paced `tick()` → 1. IMPLAUSIBLE is advisory (guide 11 :342, "no"
abort). **Changed:** T8 (a seat over 2.25 in followed by a Routine step raises none; the Tier-3 case exactly one, pinned
as documented); T9 (no `SCH` timeout line — the predicate ends the wait); landmine 23; §3.1's guide 10 sentence for the
Tier-3 case; §1.2's call doc already states that the predicate ends the wait (RC24).

**RC32 — Documents: a stale HANDOFF entry, a false "chunk count unchanged", the robotProfile that cannot hold a
measured mount, `ceb17c0` line numbers (tree, minor). REAL — fixed.** Re-verified at `71b4764`:
`HANDOFF-remaining-work.md` :489 opens "L6. [NEW-WALLRESET] Re-seat the position estimate against a wall or goal" with the
"drive slowly toward a known wall … ~1 day" plan, and §3's plan commit reworded only REMAINING-WORK row 25; build-order's
phase table ("The order at a glance", :833-851) has no L6 (grep: L6 only at :632, :1713, :2224, :2227) and :854 counts
"70 chunks", REMAINING-WORK :16 "of the phase table's 70"; `docs/vexbuilder-integration.md` :115 is `sensors: [ { port,
type } ]`; REMAINING-WORK row 25 is :107, the deviations row :2224, the roadmap's F1 row :135, station H's "any time"
line :169-170. **Changed:** the header's citations re-stamped (both numbers given); D12's and §0.3/§1.1's `roadmap.md`
:134 → :135; §3.3 — L6 joins the phase table (70 → 71, its own row recommended), HANDOFF's L6 entry becomes a pointer,
REMAINING-WORK's size → ~6–7½ (RC29); §6.7 — the robotProfile gap recorded beside G3a's. Other `ceb17c0` line numbers
in the body stay as cited (landmine 18: "re-read; never paste a line number from this brief").

**RC33 — The keep-outs were tested at the ESTIMATE's along-wall position with a fixed, underived 3-in margin, so an
along-axis drift beside a Loader seated a wrong axis with the keep-outs ON (safety, blocker). REAL — fixed at the root:
the keep-out widening carries the estimate's along-wall uncertainty, derived from the Localizer.** Re-verified: rule 3
widened each span by range × tan(cone) + 3 in only, and the along coordinate it tests is the axis the reset does not
measure; HA-211's 3 in had no derivation; robot two's saved distance scale reads ~8 % short (`roadmap.md` ~:411,
REMAINING-WORK :167, "the ~8 % distance error"); HA-83 believes σ grows 2 % of travel. The checker's `f1_static.txt` case
A (red wall, sensor at x 55, estimate along −4/−5 in, range 8) → SEATED, −2.33 in, with `maxCorrection` 3 and 4; the
first pass's r3 rows ("RC17") showed −1.50…−3.73 and tried a flat +6 in, which never reached the body. **Re-probed (Q2,
`recheck2/q2_gates.txt`, the F1 field, the tank rig, the independent cone-of-rays sensor, lens 10.5 in):** the draft's
widening SEATED the case A beam 1.57 in wrong; with the drift term after 24 or 48 in of travel at 10 % → `KeepOut`, both
estimate errors. Of the checker's two options, (b) — derive it — is taken: an author-stated `alongTolerance` would be a
number a student cannot know; the Localizer CAN count the travel since the along axis was last set. Why not the existing
drift term (`distanceSinceCorrection`): it is ONE scalar for both axes and D2 deliberately leaves it unreset by a
one-axis seat — so a reset against a perpendicular wall (which DOES fix the along axis) would not shrink it; a per-axis
counter does. **Changed:** rule 3; §1.3 (the two counters — the heading one serves RC38); D8 (a); the plain words' item
3; HA-211 (the margin's derivation; the 10 % rate and why); T16 (the counters, Q2's case A as a test, M33–M35). Cost: the
clean wall shrinks as the robot drives — Q1 (C): on a left/right wall the beam centre's band is 2 × 34.7 in at 7 in
range right after a seed, 2 × 25.1 in after 48 in, 2 × 15.5 in after 96 in (piece zones aside, RC34) — the guide says
"reset early in a leg, or reset the perpendicular axis first".

**RC34 — Start-of-match pieces stand against EVERY wall in the claimed "clean face", D10's 8–10-in mount does not
clear them, and nothing gated the beam's height (safety, blocker). REAL — fixed at the root: a BEAM-HEIGHT gate from
the typed height and cone (before sampling and on the median), the start pieces as PIECE-ZONE keep-outs while the cone
is unmeasured, D10 restated.** Re-verified: A12 (`img/a-107.png`, viewed) — Cup-Pin-Cup clusters at 43.50/46.66/49.82 and
90.59/93.75/96.91 in from the corner, 1.58 in off EACH of the four walls; Figure VEXU-1 (`check-field-safety/vexu1-083.png`,
viewed) shows them on all four walls in the VEX U setup. §1.1 had no span there; D10 and the plain words claimed 8–10 in
"clears Pins and Cups"; `valid()` accepted any height > 0. The checker's `f1_static.txt` case B (seated 1.19–2.70 in
wrong at 8–9 in up) and case F (a 3-in lens, the floor, −2.36), and the first pass's r3 "RC13" rows, agree. **Re-probed
(Q2, `recheck2/q2_gates.txt`)** with the amended rules on the F1 field: at 8 in up every cluster case (walls 6–12 in) the
draft SEATED 1.16–2.86 in wrong is refused `BeamLow`; at 9 in up the draft's wrong seats at 10 and 12 in (−1.87, −1.80)
are refused and the correct 6-in one stays SEATED (+0.008); at 10.5 and 11 in up every reading within the ceiling SEATS
within 0.06 in; a 3-/4-in lens over the floor → `BeamLow`; with a REAL cone of 24° under the typed 18°, the gate alone
still seated within 0.19 in, and with the cone UNMEASURED the piece zone refused `KeepOut`. **Why both mechanisms:** the
gate is the general guarantee (a single piece ANYWHERE against a wall — the start pieces, a knocked Pin, a VEX U match
load) but rests on the cone, which is unofficial (HA-205); the piece zones cover the places pieces are KNOWN to stand
until Step 0.5 measures the cone, and lift then — "a keep-out is lifted only by a measurement", the Toggle's rule. The
cost is real and stated (Q1 (C)): with the cone unmeasured the beam centre has ~2 × 8 in per wall right after a seed
and none after 48 in of travel; measured, ~2 × 35 in → 2 × 15 in. **Changed:** §1.2 — rule 3's keep-out KINDS, the NEW
rule 4 (beam height, before sampling), rule 7 (the median under the ceiling), the renumbering (§0.2's and landmine 3's
"rule 5" → 6; the listing's "rules 5-9"); the plain words' checks (a new item 4), "What it cannot catch" and "Where it can
be used" (the honest spans); D10 (a) (10–10.5 in, the ceiling table, the wall top, the strip) and (b) (robot two's height
decides whether the reset works at all); §0.3 (the clusters); §1.1 (the start pieces as `PieceZone`); §1.4 (`valid()`:
height ≤ 11.5 in, a measured cone 3°–30°); §1.7 (the plant's clusters and floor); §0.4's S5d row; D7 (b), D9, §5's
passive-mode line; T3 (+M36–M38); T8 (Q2's cluster cases; S5d no longer a known limit); landmine 1; HA-212 (the start
pieces, the 6.5-in piece top), HA-205's code sites. Residual, said plainly: a nested stack or a robot is taller than the
ceiling assumes (landmine 1).

**RC35 — VEX U fields MUST carry the GPS code strip, and it sits where D10 aims the beam — unmentioned, unregistered,
untested (safety, major). REAL — registered (folded into HA-206, no new id), measured in Step 0, modelled in the
host plant.** Re-verified: v20.txt :3454 (VURC field setup: "The VEX GPS code strip must be installed on the Field") and
:2061 (`<RSC4>` a: required in Autonomous Coding Skills); A8 (`check-field-safety/a8_wall_crop.png`, viewed): two rows of
black/white squares in the wall's upper band; at the printed 13.20-in dimension's 48.1 px/in the band spans ≈ 9.06–11.44
in above the floor (re-measured by this writer from the image's own pixels: the printed 13.20-in dimension spans rows
147–782, the squares between the brackets rows 235–345 → 9.09–11.37 in; the checker read 9.06–11.44 — the body keeps
"≈ 9.06–11.44"). Every D10 height
(10–10.5 in after RC34) puts the beam's centre ON the strip, and its cone over it at any range. grep of the draft: no
"strip", "VEX U", "VURC". The checker's point that confidence is synthetic below 200 mm (landmine 6) stands: a dark-square
bias there would meet only the error-bound and disagreement gates. **Why HA-206 and not a new id:** it is the same belief
— "the wall's face at the sensor's height returns a true reading" — about a different surface; a tenth id would collide
with IMU1's recommended HA-213 (§6.6). **Changed:** HA-206 (the strip, its band, its own "invented" confidence and its
settles); Step 0.3 (black and white squares at 4/8/12 in — a printed sample if the field lacks a strip) and Step 0.6 (on a
real strip); the plain words' clear-panel bullet; §1.7 (a strip band with a per-colour bias, default none) and T14 (the
band biases only its rays); §3.1's guide 14 item. D10 (RC34) already says the strip is under the beam.

**RC36 — VEX U loads through the Loaders DURING the autonomous period, and the Load Zones reach past the 12-in
corner keep-out (safety, major). REAL — the Load Zones are FIXED keep-outs on every wall, D8 (c) states the
alternative, the guide says why.** Re-verified: v20.txt :3511-3512 (`<VUG4>`: Match Loads "during both the Autonomous
Period and Driver Controlled Period"); :1346-1351 (`<SG11>` b, d: placed or dropped through a Loader's top or back,
"separately or nested together", removed only through its bottom opening); :4425-4427 (the Load Zone, bounded by the
perimeter and the tape); A10 (`img/a-105.png`, viewed): the tape at 587.1 mm = 23.11 in from each corner along both walls
— a square in all four corners. The draft's keep-outs reached only |along| ≥ 58.2 (corners) and the Loaders' ±2.01 in.
**Chosen — FIXED, not a piece zone:** the beam-height gate (RC34) keeps out a SINGLE piece, but a VEX U Load Zone is the
one place on the field where taller objects are EXPECTED during the auton — a robot collecting loads, a nested stack
it drops — so a measured cone must not lift it; D8 (c) gives the alternative (lift it with the cone) and its cost.
Cost of the choice (Q1b, `recheck2/q1b_spans.txt`): once the cone is measured the beam centre has 2 × 23.6 in per wall
right after a seed (18.3–41.8 in either side of the centre at a 7-in reading), 2 × 14 in after 48 in of travel, 2 × 4.4
in after 96 in. **Changed:** D8 (a) (the Load Zones, why) and (c); §0.3 (the Load Zones, `<VUG4>`, `<SG11>`); §1.1 (the
Load Zones as `Fixed` spans on all four walls); rule 3's piece-zone kinds (the start pieces only); the plain words'
spans (Q1b's numbers, "reset early, or reset the perpendicular axis first"); HA-212 (the VEX U loads); §3.1's guide 14
item; T8 (a Load Zone refuses with the cone measured or not).

**RC37 — "On a clear panel the reset will REFUSE … never seat wrong from it" is false in the brief's own model (safety,
major). REAL — reworded everywhere it appears, pinned as a known limit.** Re-verified: the checker's `f1_static.txt` case
D (see-through red-wall panel, an opaque object behind it, `maxCorrection` 3: 1 in → SEATED +1.01, 2 in → +1.94, 3–4 in →
`Disagrees`) and the first pass's r3 "RC14" rows (+0.96, +2.02, +3.03, then `Disagrees`); this pass's Q2 (the amended
gates, lens 10.5 in, the F1 field): +1.09 and +1.99 in SEATED, 3 and 4 in refused — the beam-height and other gates do
not touch it (the object is behind the wall's face and as tall as a person). A14 (`check-field-safety/a14_small.png`,
viewed by the checker) draws the metal wall as a 1.27-in section, so the wall's own frame can sit within ~1.3 in behind
the inner face. **Changed:** the plain words' "Clear wall panels" (usually refuses; seats wrong by up to `maxCorrection`
with something close behind; a tight `maxCorrection` caps it); HA-206 (that consequence in the belief's row); §3.1's
guide 14 item; T8 (the see-through-plus-object case pinned as a KNOWN LIMIT beside S4b/S7b, with its numbers).

**RC38 — The accuracy headline (0.74–0.92 in) and the precision gate rested on VEX's 200-mm step read as a sweet spot
and on a fixed 2° heading tolerance that ignored the register's own HA-84 (safety, major). REAL — fixed at the root:
max(15 mm, 5 %) or a measured accuracy, a heading tolerance that grows with turning (the Localizer counts it), the
budget restated, the fore-aft centre made load-bearing.** Re-verified: VEX's wording (api.vex.com, quoted in §0.3: "Below
200mm, accuracy is approximately ±15mm. Above 200mm, accuracy improves to about 5%") — two approximations; P1's `acc()`
(`probes/p1_geometry.py`) is a step, and the bound drops 0.898 → 0.772 between 7.80 and 7.90 in (Q1 (A2), f = 0) — a
cliff in a gate; HA-84 (`docs/hardware-assumptions.md` :277, :3847-3852) believes heading σ grows 1 % of rotation; HA-211's
2° was HA-20 alone. **Re-probed (Q1, `recheck2/q1_budget.txt`; P1's own geometry, the worst corner over believed skew ≤
5°, heading error ±tol, mount ±0.25 in / ±2°, accuracy ±):** with max(15 mm, 5 %) — f = 0: 0.91–0.98 in at 2–10 in before
turning, 1.02–1.19 after 180° (tolerance 5.6°), 1.25–1.63 after 450° (11°); f = 2.5: 1.00–1.07 from the start; f = 5:
1.09–1.15 from the start. With a mount measured to ±0.1 in and a measured accuracy ±0.3 in, f = 0: 0.46–0.52 before
turning, 0.78–1.16 after 450°. The checker's own points (`f2_budget.txt`: f = 5 at 8 in, 0.918 → 1.229 at 2° → 4.5°)
agree in direction (its 4.5° is the 1-σ figure; this brief uses 2σ, the floor's own doubling). **Changed:** rule 8
(the accuracy model, the derived heading tolerance — its counter is §1.3's, RC33); `SideMount::rangeAccuracy`;
D6 (the optional measured fields); the plain words' "How accurate it is" (restated: ~0.91–0.98 in centred before any
turning, refused after ~180° or 5 in forward, ~0.5 in with Step 0); §0.4's P1a/P1f notes (Q1 re-does them); §2.1's
"what an auton can do"; D10's fore-aft centre (load-bearing); HA-204 (the max model), HA-211 (the heading tolerance
and its code sites); §1.4's config struct (no `headingTolerance` field) and `valid()`'s new bounds; §6.7's G3a
vocabulary; T2 (M6 is now "the step restored", plus M6b/M6c). The 1.0-in `errorBoundMax` stays (F2's pose target);
with VEX's published numbers it is tight on purpose — Step 0.1 and 0.3 are what relax it, by measurement.

**RC39 — D10's "at least 1 in inboard … a robot touching the wall must still read" is always refused by the 2.0-in
`rangeMin`, which nothing derived (safety, major). REAL — fixed by the checker's option (a), with the 2.0 derived and
tested.** Re-verified: D10 (a) said "at least 1 in inboard"; §1.2/D7 default `rangeMin` 2.0 in; §1.4's `valid()` allows it
down to 0.79 in (VEX's 20 mm); grep: no derivation of 2.0 anywhere in the draft. The checker's `f1_static.txt` case C: a
robot flush with the sensor face 1.0 or 1.9 in from the wall → `REFUSED out-of-range` both times. **Chosen (a):** advise
≥ `rangeMin` + 0.5 in = 2.5 in inboard, keep the 2.0-in default (the close-range behaviour is the least known regime —
VEX publishes a floor, not a behaviour), and make Step 0.3 measure flush and 1 in so the team can lower it on evidence.
(b) — lower `rangeMin` now — would rest on nothing measured. **Changed:** D10 (a) (2.5 in inboard, why); Step 0.1 (e) (the
inboard distance recorded; under 2.5 in "never park flush to reset"); Step 0.3 (flush and 1-in readings); HA-211 (both
range ends derived and marked invented).

**RC40 — P2 put the Loaders on the wrong walls, yet S4a/S4b were D8's and §5's Loader evidence (safety, major — second
half). REAL — re-based on the F1 re-run; the conclusion stands at 3.69 in.** Re-verified: `probes/proto/field_model.hpp`
:73 and :87-90 push the Loaders onto the x = ±W walls (A10's drawing frame), while §0.3/§1.1 place them on the red/blue
walls in F1, which FO-1 and VEX U's Figure VEXU-1 (viewed) confirm; P2's S4a/S4b reset x against the LEFT wall beside a
"Loader" that F1 does not have there. The first pass's `recheck/r3.txt` (re-read; its harness `r3_f1_field.cpp` builds the
F1 field) has S4a' → `KeepOut` and S4b' → SEATED −3.69 in (a y reset against the red wall), and the old S4 script on the
F1 field seating correctly (+0.009). **Changed:** §0.4's S4a/S4b rows and a note after the table (what was wrong, the F1
rows); T8 (an F1-frame field, S4a/S4b as S4a'/S4b'); §1.7 (the plant's Loaders on the red/blue walls, said); D8 (b)
(RC36 already cites 3.69 in F1).

**RC41 — HA-208 is false at the Toggle's brackets: they hang ~0.9 in below the wall top, and the flag that lifts the
Toggle keep-out was set without reading there (safety, minor). REAL — the Toggle keep-out split: its body liftable by
the measured flag, its brackets never; HA-208 restated; Step 0.6 reads at the brackets.** Re-verified on A8
(`check-field-safety/a8_wall_crop.png`, viewed; this writer's own pixel reading): the 13.20-in printed dimension spans
pixel rows 147 → 782 (48.1 px/in); a column profile through each bracket over a white square (x 440 and 1540) is dark
to row 271 and white from 272 → (782 − 271)/48.1 ≈ 10.6 in above the floor; the brackets span pixel columns ≈ 360–460 and 1515–1615 of the Toggle's 360–1613 (25.99 in) → |along| ≈ 10.9–13.0
in at each end. **Re-probed (Q2's T rows):** with the whole Toggle keep-out lifted and brackets modelled 1 in deep, a beam
at along 11.9 SEATED 0.98 / 0.96 in wrong at 10 / 10.5 in up; at mid-Toggle 0.015 / 0.157 (the latter the cone's top
grazing the Toggle's underside — Step 0.6 at the real height is what may lift it). **Changed:** HA-208 (body vs
brackets, the measured 10.6 in, the invented depth, what the flag lifts); §0.3's Toggle bullet; §1.1 (three spans: body
`Toggle`, brackets `Fixed`); D8 (a) (the flag per side, `SideMount::toggleChecked`, lifts the body only — the draft's
per-robot `beamUnderToggleChecked` is gone, RC38's struct note); Step 0.6 (mid-Toggle and both bracket ends; the
brackets' depth); T8 (+M39).

**RC42 — The Pin's size was wrong in §0.3 and in P2's lying-Pin model (safety, minor). REAL — corrected and
relabelled.** Re-verified: v20.txt :4546 ("Pin … approximately 1.6" (40mm) in diameter and 6.5" (165mm) tall") and :4270
("Cup … approximately 3.15" (80mm) in diameter and 6.5" (164.5mm) tall"); `probes/p2_scenarios.cpp` :134 models the "lying
Pin" as a box 6.5 in out from the wall × 3.2 in along it × 3.16 in tall — a Cup's cross-section, end-on. The standing Pin
(:139, 1.6 × 1.6 × 6.5) was right. **Changed:** §0.3 (each piece's diameter); §0.4's S5a/S5b labels (a lying Cup); T8
(S5b relabelled, a lying-Pin case added); §1.7's plant pieces already carry 1.6 / 3.15 (RC34). Not re-run: a lying piece
under a sensor 9–10.5 in up is far below the beam-height ceiling's 6.5 in either way, so the label, not the verdict,
was wrong (S5a's 4-in sensor is refused `BeamLow` under RC34 regardless).

**RC43 — The sensor's multi-object behaviour (the nearest return in the cone) was hard-coded into the host plant and
the known-limit pins but registered nowhere (safety, minor). REAL — HA-205 extended (no new id), Step 0.5 records the
behaviour, the plant gains a `blend` mode.** Re-verified: §1.7 ("returning the NEAREST opaque hit"), T8's KNOWN-LIMIT
magnitudes and rule 3's footprint widening assume it; HA-205 covered the cone angle only; VEX's distance-sensor page says
object size comes from "how much of the sensor's view is taken up" — an integrating sensor, which may blend. **What
depends on it, sorted:** the beam-height gate (RC34) does NOT — at or under the ceiling no single piece is in the cone at
all, whatever the sensor does with two surfaces; the known-limit magnitudes (S4b, S7b, the clear-panel case) and the
claim that the cone's top edge passing over the wall top is harmless (D10, RC34) DO. **Changed:** HA-205 (the belief, its
dependants, its settles); Step 0.5 (how the reading changes, not only where) and Step 0.6 (a person behind an alliance
wall); §1.7 and T14 (the plant's `blend` mode, so a test can run a rule under both behaviours).

**RC44 — Step 0.8's refusal drills mislabelled one outcome and gave no safe sign or stop plan; Step 0.4's "far" was
ambiguous at a bench (safety, minor). REAL — rewritten.** Re-verified: Step 0.8 listed "a Pin against the wall in the
beam (low sensor only)" among "each refusal on purpose", but P2's S5d SEATED 2.59 in wrong and S5a refused only through
`maxCorrection`; "an estimate pushed 6 in off by a deliberate wrong start pose" gave no sign — an estimate 6 in FARTHER
from the wall than the robot makes the next leg steer it ~6 in toward the wall from an 8-in gap; nobody was told to
stand ready to disable; Step 0.4's "port 5 far" can be a room object at a bench. Bench Tests station 11 itself is
read-only (`src/programs/bench_r3a.cpp` :2487-2489, "Nothing is written to a sensor"), so Steps 0.1–0.6 are safe as
written. **Changed:** Step 0.8 — the shared safety (a student on DISABLE, each drill's auton ends at the reset, a
`brake()` end action), the capture (the competition program's Timed Run and `comp_log.txt`, a serial cable; the tester's
wall line from L6-B, RC47), and six drills each with its expected verdict or a KNOWN-LIMIT label (the Pin drill now
expects SEATED under the ceiling — RC34 — and records which; the wrong start pose typed 6 in TOWARD the wall);
Step 0.4 ("port 5's reading must not change when the board moves").

**RC45 — D4's example — the only student-facing call shape — parked at a 6-in reading and never squared up; refused
IMPRECISE or NOT SQUARE on first try; the "~2 × 40 in" spans were too generous (team, major). REAL — D4 rewritten as a
FIELD-start sequence (face, a leg along the wall, `turnTo` square-up, a ~7-in reading under the ceiling), the bands per
wall stated.** Re-verified: D4 parked at x = −56.7 → a 6.0-in reading for a sensor 7.5 in from the centre; the checker's
`r2/d4_parking.py` bound (rule 7, VEX accuracy, mount ±0.25 in / ±2°, heading 2°) 1.019 in at 5 in forward — over 1.0;
this pass's Q1 (D) (max(15 mm, 5 %), the corner sweep): 0.946 centred and 1.120 at 5 in forward for that reading; P2's
own rows and the log lines park at −54.7 (an 8-in reading); `Routine::driveTo` arrives facing the bearing from the step's
start (`routine.hpp` :241-256), so "nose toward +Y" held only after a leg along x = −56.7 — a FIELD start at (−60, −60,
90°) ends 7.8° off, refused by rule 2; guide 16 :257-258 ("remember a tank turns first (`face()`)"). After RC33/RC34/RC36
the bands are narrower still (Q1b). **Changed:** D4 — (a) the everyday form over the auton's context (RC48), the example
(face, leg along the left wall to y = −38, `turnTo(90_deg)`, a ~7-in reading: x = −70.205 + 7.5 + 7 = −55.7), compiled by
Q3 inside `namespace shulib::bench`; where to park (under the ceiling, inside 2–16 in, room for `maxCorrection`); where
the beam may land (the bands, the leg's own travel); the first pass's compile-time `maxCorrection` (r4) considered and
rejected on Q3's evidence (the error names neither field nor fix); (b) L6-C not recommended (RC48). §2.1's "what an auton
can do" (the "5–12 in" replaced by the sequence); §3.1's guide 03 and cookbook 05 items (the recipe, the bands).

**RC46 — No refusal told the student what to do; the verdict words and remedies were unspecified (team, major). REAL —
a verdict table (word, numbers, what to do) in §1.2, carried verbatim by guide 11, with a guide 12 symptom entry and a
procedure for `maxCorrection`.** Re-verified: §1.2 listed the verdicts but specified words for only a few; P2's output
used other words ("not-parallel", "keep-out", "no-return"); only `BadRequest` named a fix; the log spec carried numbers
but no remedy, and "by LEFT" read as a direction; guide 11 promises "every fault code: what it means, what causes it, what
to do about it" (`11-reading-the-diagnostics.md` :4-5) with a "What to do" column (:331); guide 12 (symptom-first) was in
no run's document list. The checker's two confusing first-try cases are now covered: a RELATIVE start is refused
`not a field start` by the everyday form before any aim check (RC48), and a beam facing away reads `not square` near 90°
or 180° with that row's remedy (wrong side, wrong wall, nose not along it). **Changed:** §1.2 — the line's "(LEFT sensor)"
form, a keep-out line that prints where the beam landed and the nearest clean span, the table (21 verdicts: the words are
`wallResetVerdictWord`'s, the q3 transcription's), "Choosing `maxCorrection`" (the procedure the first pass's RC22
asked for — run without the reset, tape the axis error, largest + ~1 in, never near the gap to a lookalike); §3.1 —
guide 11 carries the table, guide 12's symptom entry in L6-B (L6-A2's list stays at five).

**RC47 — A refused reset was invisible where students test: the tester's RESULT page, its SD log and the host autons
test all missed it (team, major). REAL — the reset records its outcome where the robot's code can read it; the Auton
Tester shows it (L6-B); guide 16 says what the host run does.** Re-verified: `TesterRunRecord` (`src/programs/
auton_tester_run.hpp` :68-83) holds the word, the END estimate and the faults, no reset outcome; guide 16 :241-251: the SD
file gets run summary lines only, "Nothing is written to the card during a run"; only the competition program copies log
lines to `comp_log.txt` (guide 17 :295-302); the host test "autons (1b)" (`test/autons_test.cpp` :143-172) runs every
registered auton on `TesterStack` (a rig, not a Robot — `test/auton_tester_rig.hpp`), whose context has no walls slot, so
a student's reset refuses `NO SENSOR` there and the test passes. **Changed:** §1.6 (the slot non-const — the reset writes
its log), the listing's `WallResetLog` (RC24); §1.10 (the tester's record, RESULT line and SD words, with their exact
form); §2.1's L6-B files (+ `auton_tester_{run,panel}.hpp` and their tests); §2.2's L6-B tests (+MB4); §3.1 (guide 16:
the host run refuses `NO SENSOR`; only the robot tests the reset). Step 0.8 names the capture (RC44). L6-B's slot rule
already covers MECH-A5's rewrite of the tester files (before A5 launches, or after it merges — §6.2); its size carries
the extra (RC29).

**RC48 — RELATIVE-start autons — all of the team's today — got a confusing refusal or a wall with NO keep-outs; the
"library cannot see the start mode" claim was wrong for an auton (team, major; the finding arrived cut off at "Either
give `line()` a keep-out argume…", dispositioned from what arrived). REAL — the everyday call takes the auton's CONTEXT:
it refuses any non-FIELD start, samples through the guard's deadline-aware wait, and records its outcome; `line()` keeps
an optional keep-out list for the explicit form; L6-C becomes optional and not recommended.** Re-verified:
`src/team/autons.cpp` :10-13 and :66 — the 40 in auton is `StartPose::relative()`, the only entry; `FieldWall::line(name,
axis, coord, inward)` took no keep-outs, so a RELATIVE auton's reset had none (D8 (b) rejects exactly that); landmine 8
said the library cannot see the start mode "(the call takes a `Chassis`, not the auton entry)" — but `AutonContext`
(`competition/auton_registry.hpp` :215-229) carries `entry` (`effectiveStart()`, :266-275) and `guard`, whose
`waitFor()`/`pause()` are "the only waits the deadline (and a mid-auton disable) can cut" (:218-219; `run_guard.hpp`
:95-109, `waitFor` :437). Placing the call in `competition/` also settles RC24's layering (it includes the layer below).
**Chosen:** "a wall reset needs a FIELD start" as the RULE, enforced (`NotFieldStart`) rather than only documented —
under a RELATIVE start no keep-out, season or own, can be trusted; `line()` keeps an optional keep-out list (§1.2's
listing) for Tier-3 code that knows its own frame. A `Routine` member (L6-C) could see neither the guard nor the start
mode, so it would be strictly weaker than `then()` over the context form: optional, not recommended. **Changed:** D1
(three runs and an optional fourth; L6-A2 builds the context form), D3 (L6-C's stop cause only if taken), D4 (a)/(b)
(RC45 carries the listing); the NEW §1.12 (the context form, its order, the guard's cut, the log); §1.1's banner; §1.2's
sampling paragraph (two waits, one sampler); §1.8 (no bump for the new header; L6-C optional); §1.11 and §2.2/§3.1/§7's
L6-C entries marked optional; §2.1's L6-A2 row (the new header, three test files; the `localization/` overload gone);
§1.9 (+`competition_wall_reset_test.cpp`); T11 (the explicit form's own wall) and the NEW T17 (+M40, M41); landmines 5 and
8; §5's route-command line; §6.2 (the tester files' row, RC47); §3.3 (the 11-23 plan's line: no FIELD-start auton yet);
the plain words ("a wall reset needs a FIELD start").

---

## DECISIONS FOR THE TEAM LEAD (a recommendation for each; nothing below decides them silently)

**D1 — Three runs (and an optional fourth), and when each starts. RECOMMENDED (a).**
- **(a)** **L6-A1, the library core** (the season's walls, the geometry, the gates, the verdict, `Localizer::seatAxis`,
  the call over EXPLICIT sensor handles, the host test plant's field and side-sensor model, D11's recipe fix)
  **starts on your go**, beside MECH-A3 and VIS1-A1: it shares **no code file** with either (§6.2 — only documents:
  the changelog, guides 10/11/14, the HA register and the generated pages, merged serially; *corrected 2026-10-22:
  the draft named `version.hpp` too — A1 bumps nothing, §1.8*). **L6-A2, the robot's
  side sensors** (the robot file's block and `valid()`, the Robot building them, the context's optional slot, the everyday
  call over the auton's context, §1.12 — *RC48*) starts **when VIS1-A1 merges**: both edit `config/robot_config.hpp` and
  `robot/robot.hpp` (VIS1 §6.7's rule: never two builders in one file; the second re-reads the tip). **L6-B, robot
  two** (its side sensors in `src/robots/chassis_table.hpp`, `src/common/boot_source.{hpp,cpp}`'s copy, Bench Tests
  station 11's WALL page, the Auton Tester's wall line — *RC47* —, the worksheet) starts **when L6-A2 merges, if MECH-A5 has not launched** (A5 rewrites
  `boot_source` and re-reads its tip anyway — MECH brief §2); if A5 has launched, B waits for A5 to merge and goes
  before A6 launches (VIS1-B's slot — whichever of the two goes second re-reads the other). **L6-C, the Routine
  step** (`r.resetFromWall(…)`, F10-additive) is **OPTIONAL and NOT recommended** (*RC48*, D4 (b): a `Routine` sees
  neither the run guard nor the start mode); if taken, after MECH-A4 merges and outside MECH-A5's run (§6.2). An auton
  resets through `then()` over the everyday form from the day A2 lands. **What this buys before 2026-11-23:** with
  A1, A2 and B landed (~6–7½ builder-days, §2.1 as amended — *RC29*; ~7½–10½ slot-days with verification) and Step
  0.1's mount typed, any FIELD-start auton in `src/team/autons.cpp` can re-seat x or y against a wall it has parked
  beside (§"The design in plain words"; a RELATIVE start is refused — RC48). When it lands depends on D13.
- **(b)** One run after MECH-A5. **Cost:** nothing of L6 on the robot before mid-November at the earliest (A3 →
  A4 → A5 are ~11–15 builder-days, MECH brief §2's table) — too late to test before the 23rd.
- **(c)** L6-A as ONE run, after VIS1-A1 merges. **Cost:** ~3 days of L6 idle while VIS1-A1 builds and is reviewed;
  nothing gained (A1 shares no file with VIS1-A1).
- **(d)** B always after A5 (VIS1-B's rule). **Cost:** the date risk of (b) for B; it saves A5 one re-read of a
  ~20-line block in `boot_source.hpp` (§6.2).

**D2 — How a reset changes the estimate: a ONE-AXIS SEAT. RECOMMENDED (a).**
- **(a)** A new `Localizer::seatAxis(Axis, Length)` (the Localizer is not in the Freeze Register): that axis jumps to
  the measured value; **the other axis, the heading, the heading bias and the drift term (`distanceSinceCorrection`)
  are untouched**; *(RC1)* the odometry is NOT touched (only the Localizer reads it, and only its per-tick deltas)
  and the next tick re-bases exactly as after `setPose` (`hasLast_ = false`, `localizer.hpp` :519-532 — load-bearing
  under the EKF tier, measured in RC1: without it the filter reads the seat as one tick's travel). It is a **caller act** gated by the reset's
  checks, like `setPose`/`reseatHeading` (`localizer.hpp` :509-564). **(a) AMENDS a LOCKED decision** *(RC25,
  corrected: the draft called this "a wording change, not in force")*. Decision #4 (master plan :547, locked
  2026-06-19, "GPS/AprilTag aggressiveness") reads in full: "Innovation-bounded, covariance-weighted, per-tick-clamped
  **gated nudge (never snap)** + hard dead-reckon-only flag. **`relocalize()` feeds this corrector — it does not
  snap.** Clamp/gate/gain are M3-tuned (mechanism locked, numbers open); complementary tier realizes the same contract
  heuristically." A wall reset IS a caller-invoked relocalization against a field feature (`relocalize(featureSource)`
  is an alignment primitive, :639; :409 sends a tag fix "into the gated corrector … *never a snap*"), and (a) snaps
  one axis — so the locked text rules AGAINST (a) and FOR (c). **The amendment you would be accepting:** "…
  `relocalize()` feeds this corrector — it does not snap, **except a caller-invoked ONE-AXIS wall reset (L6), which
  seats that axis only while no motion is active, inside gates whose largest jump is the author's `maxCorrection`; no
  sensor ever seats the heading**." Why amend rather than take (c): the hazard never-snap guards — master plan risk 5
  (:528-532), "corrector pose-jump mid-motion" — cannot arise, because the reset refuses while any motion is active
  (rule 1, `MotionActive`, RC2) and the next motion starts from the seated pose; and (c)'s own costs (below) include the
  very overclaim (b) has. Everything else decision #4 covers — every corrector, every other relocalize — is
  unchanged. The Localizer's banner (:43-48, "nothing that reads a sensor can reach reseatHeading()") stays TRUE and
  gains one sentence. Heading stays untouchable by any sensor (D9).
- **(b)** Use today's full `setPose` with the other axis copied. **Cost, measured (probes P2 S5b and P4):** the drift
  term drops 23.77 in → 0, and **from the second tick after the seat** the quality class reads **DeadReckon (quality
  1.000)** where the one-axis seat keeps **Degraded (0.200)** — the first tick is the dt-0 re-base tick, Degraded in
  BOTH (setPose 0.500, seatAxis 0.100) *(RC30, corrected: the draft said "ONE tick later")* (P4, `out/p4_quality.txt`;
  per tick: `check-tree/round2/tu/p4_per_tick.txt`, re-run by this writer; the rule is `refreshQuality`, `localizer.hpp`
  :579-600: Degraded past the 12-in `driftHorizon`, :155) — **a one-axis measurement passes as a two-axis one**.
- **(c)** A corrector NUDGE — *what locked decision #4's text prescribes for a relocalize (RC25)* — (an `ICorrector`
  proposing (measured x, predicted y), `correction.hpp` :46-53).
  **Cost:** a corrector slot armed and disarmed by the step (the Robot holds at most `kMaxCorrectors` = 4,
  `localizer.hpp` :204), ~0.3 s standing still to converge (gain 0.15 per tick, 12 in/s budget,
  `complementary_fusion.hpp` :63-70), a 12-in innovation gate the author does not set — and an applied fix STILL
  clears the drift term in proportion to its confidence (`localizer.hpp` :378-381): the same overclaim as (b).

**D3 — What a REFUSED reset does to the routine: continue, unless the author says it is required. RECOMMENDED (a).**
- **(a)** A refusal leaves the estimate **exactly as it was**, writes ONE Warn line naming the gate and its numbers
  (`[WALL] left wall x REFUSED disagrees: wall says -54.7, estimate -48.7, limit 4.0 in`), and the auton
  **continues** — the robot is then exactly where it would have been without L6. A request marked `required = true`
  turns a refusal into a FAILED step: through `then()` the chain stops like any failed action (`routine.hpp`
  ~:426-462, `ActionFailed`) — and, only if the optional L6-C is taken, as an appended `RoutineStopCause::WallResetRefused`.
- **(b)** Always stop. **Cost:** a bumped Pin or a partner in the beam parks the robot for the rest of the period.
- **(c)** Always continue. **Cost:** an author who built the next 30 in of the route on the reset cannot say so.

**D4 — The call an auton makes now: the everyday function over the auton's CONTEXT, reached through `then()`; the
Routine member NOT recommended. RECOMMENDED (a).**
- **(a)** L6-A1 ships `chassis::resetFromWall(Chassis&, const localization::WallSensor&, const localization::WallResetRequest&,
  const localization::WallResetLimits&) → localization::WallResetReport` in `chassis/wall_reset_call.hpp` (explicit
  handles — a hand-wired Tier-3 robot's form, and the host tests'; *RC24: a `localization/` header cannot include the
  facade*); L6-A2 adds the everyday form **`competition::resetFromWall(AutonContext& ctx, const
  localization::WallResetRequest&)`** in `competition/wall_reset.hpp` (*RC48*): it finds the named side's sensor, mount
  and limits through `ctx.chassis.deps().ctx->wallSensors()` (an OPTIONAL slot, D5), **refuses a start that is not a
  FIELD start** (`ctx.entry.effectiveStart()` — the season's walls are F1 coordinates), samples through **the run
  guard's deadline-aware wait** (`ctx.guard.waitFor` — a deadline or a disable mid-window CUTS the reset, `REFUSED cut`,
  instead of running out a frozen wait), and records the outcome where the Auton Tester reads it (RC47). Neither form
  needs **any change to `Routine` (F10), `Chassis` (F6) or `AutonContext`**, and both land beside MECH-A4. In an auton
  whose entry declares a FIELD start (for example `StartPose::field(-55.7_in, -50_in, 90_deg)` — where the drivers
  place the robot is the team's choice):
  ```cpp
  r.face(-55.7_in, -38_in);      // a tank turns first (guide 16)
  r.driveTo(-55.7_in, -38_in);   // a leg ALONG the left wall: a centred left sensor 10.5 in up reads ~7 in
  r.turnTo(90_deg);              // square to the wall — a tank turns in place, so x holds
  r.then([&] { return competition::resetFromWall(ctx,
                   {.wall = localization::override2026::kLeftWall,
                    .sensor = localization::WallSide::Left,
                    .maxCorrection = 3_in}).continues(); }, "left wall x");
  ```
  (written as `src/team/autons.cpp` needs it — inside `namespace shulib::bench`, so `competition::` and
  `localization::` resolve from there; Q3 compiles it, `recheck2/q3/tu/student_auton.cpp`. `continues()` is true when
  SEATED, or when REFUSED and not `required`.) *(RC45, rewritten: the draft parked at x = −56.7 — a 6.0-in reading
  under a sensor assumed 7.5 in from the centre — and never squared up: `driveTo` arrives facing the bearing from
  wherever the step started, `routine.hpp` :241-256, so from a FIELD start at (−60, −60, 90°) it ended 7.8° off
  parallel — refused NOT SQUARE — and guide 16 :257-258 asks a tank to `face()` before it drives.)* **Where to park:**
  the reading must sit under the beam-height ceiling for the sensor's height (rule 4: ~10.8 in at 10.5 in up with the
  unmeasured cone, ~9.2 at 10 in) and inside 2–16 in; ~7 in leaves room for a 3-in `maxCorrection` either way. The
  robot centre's coordinate is the wall + inward × (the mount's `left` + the reading): −70.205 + 7.5 + 7 = −55.7. **Where
  the beam may land** (the guide gives the bands per wall — Q1b, `recheck2/q1b_spans.txt`): with the cone unmeasured,
  |along| 33.6–41.8 in from the wall's centre right after a seed, narrowing by the leg's own travel (the 12-in leg above
  leaves 34.8–40.6, so y = −38 lands at 38); with the cone measured, 18.3–41.8 right after a seed, 23.1–37.0 after 48 in.
  *Considered and rejected (the first pass's `recheck/r4_required.cpp`):* making `maxCorrection` a type with no
  default so an omitted one does not compile — Q3 shows the error a student then gets ("invalid initialization of
  reference of type 'const WallResetRequest&' from expression of type '<brace-enclosed initializer list>'") names
  neither the field nor the fix; the run-time refusal by name (D7) says both.
- **(b)** **L6-C — the `Routine` member `r.resetFromWall(…)`, after MECH-A4. NOT recommended** *(RC48)*: a `Routine`
  holds only the `Chassis`, so it can see neither the run guard (its wait is frozen — up to `windowMax` late past a
  deadline, landmine 5) nor the start mode (it cannot refuse a RELATIVE start); what it adds over (a) — a
  `WallResetRefused` stop cause instead of `ActionFailed`, and `lastWallReset()` — the `[WALL]` line and the outcome
  log (RC47) already give. **Cost if taken:** an F10 growth (API 2.6), MECH-A4's files and MECH-A5's (§6.2).

**D5 — Where the side sensors' facts live: the robot file, VIS1's template. RECOMMENDED (a).**
- **(a)** A NEW `config/wall_sensor_config.hpp` (`WallSensorsConfig`: per side, the port and the MEASURED mount),
  with only hooks in `config/robot_config.hpp` (`SensorsConfig::walls`, an APPENDED `PortUse::WallSensor` and its
  case in the exhaustive `portUseWord`, `valid()`'s claims and mount checks); the Robot builds each sensor through the
  provider's EXISTING `distance(port)` (MECH-A1, `hal/device_provider.hpp` :178 — no provider change) and its
  read-fault count through `readFaults(port)` (`hal/pros/device_provider.hpp` :516-528); **a sensor the provider
  will not make never refuses the robot** — `robot.wallSensor(side)` is nullptr and `wallSensorAbsence(side)` says
  why (the mechanism accessors' contract, `robot.hpp` ~:646-696); the boot line gains ASCII words beside "GPS … tags
  ABSENT vision ABSENT" (`robot.hpp` :770, :787, :798). This is exactly VIS1's §6.7 template.
- **(b)** The auton builds them. **Cost:** a second place a robot's facts live; G3a's panel could not write it; the
  boot line could not report it.

**D6 — The mount is MEASURED, typed once per robot; a measured range BIAS is optional. RECOMMENDED (a).**
- **(a)** Per side: `forward`, `left` (inches from the robot's centre — the turning centre on a tank), `height`
  (inches above the floor), `facing` (an angle, CCW from forward: +90° = straight out the left side, −90° the right;
  typed `90.0 * kWallDegToRad` — *RC24: the draft said `facingDeg`*), **each with its tolerance
  and a provenance string** (who measured, how, when); `measured = false` (the default) makes every reset with that
  sensor REFUSE `MOUNT UNSET` — never the robot. Optional: `rangeBias` (inches subtracted from every reading),
  typed ONLY from Step 0.3's tape readings when they show a consistent offset; 0 = none measured. *(RC38, RC34, RC41)*
  Three more optional MEASURED fields, each 0/false until its Step 0 exists: `rangeAccuracy` (Step 0.3's spread — else
  VEX's max(15 mm, 5 %)), `coneHalfAngle` (Step 0.5 — else the unofficial 18° and the start pieces kept out),
  `toggleChecked` (Step 0.6 at this sensor's height).
- **(b)** No bias field. **Cost:** a sensor that reads 0.3 in long at every distance (VEX publishes ±15 mm, HA-204)
  costs the reset that 0.3 in forever.

**D7 — Which numbers are the library's and which are the team's. RECOMMENDED (a).**
- **(a)** The **sensor-physics and geometry gates** ship as named PROVISIONAL defaults (HA-211), overridable per robot
  in the robot file: range window **2–16 in**; beam within **5°** of square to the wall; window spread **≤ 0.5 in**;
  **≥ 5 samples over ≥ 0.15 s**; confidence **≥ 0.5** (above 200 mm — below it the adapter reports 1.0,
  `hal/pros/distance.hpp` :24-31); still: **≤ 1 in/s and ≤ 3°/s**; the reading's computed error bound **≤ 1.0 in**
  (F2's ~1.0″ pose target, `roadmap.md` :136; its accuracy term max(15 mm, 5 %) or the measured one, its heading
  tolerance 2° + 2 % of the rotation since the heading was set — *RC38*); keep-out margin **3 in** plus **10 % of the
  travel since the along axis was set** (*RC33*); the beam-height ceiling from the typed height, a 6.5-in piece + 0.5 in
  and an **18°** cone while the mount's is unmeasured (*RC34*). The ONE policy number, **`maxCorrection`**
  (how far the wall may move the estimate), has **no default**: the author states it in every request, and an unset
  one is refused at the call by name (`maxCorrection UNSET: say how far this wall may move x`). This is the MECH
  core's rule (a team threshold is never the library's guess) applied to the one threshold that is the team's.
- **(b)** Everything defaulted. **Cost:** a 4-in default bound silently accepts a 3-in error (P2 S7b; S5d before RC34's gate).
- **(c)** Everything typed. **Cost:** ten numbers a student cannot derive, typed by guess.

**D8 — The keep-outs: the season's elements, applied by default. RECOMMENDED (a).**
- **(a)** `localization::override2026` (*RC3*; a NEW `localization/override_walls.hpp`, §1.1 — *corrected 2026-10-22: the draft named
  `field/override_2026.hpp` here*; season data with provenance) names the four walls (F1 frame)
  and, along each, the spans a beam must not land on: the **Toggle** above each wall's centre (26 in long, A8/A10),
  the **Loaders** on the red and blue walls (4.02 in wide × 3.74 in deep, centred 58.77 in either side of centre,
  A9/A10), **each corner** (12 in, where the cone meets the other wall), *(RC36)* **the VEX U Load Zones** — every
  corner's 23.11-in square, on BOTH walls it touches, always: in VEX U, Match Loads come in through the Loaders DURING
  the autonomous period (`<VUG4>`, v20.txt :3511-3512) and leave only through a Loader's bottom opening into its Load
  Zone (`<SG11>` d, :1351; "separately or nested together", :1347) — so a robot taking loads, or a dropped nested stack,
  may stand there, both taller than the beam-height gate assumes — and *(RC34)* A12's **start pieces**, until the
  mount's cone is measured. Each span is widened by the beam's
  footprint at the expected range, the margin, **and how far the estimate may have drifted ALONG the wall since that
  axis was last set** (`alongDriftRate` × the travel since — *RC33*: the keep-outs are tested at the estimate's along
  coordinate, the one axis the reset does not measure). A request may carry its OWN keep-outs (a wall line the author
  defines, e.g. for a RELATIVE-start auton). The Toggle BODY's keep-out is lifted per SIDE only by a MEASURED flag
  (`SideMount::toggleChecked`, Step 0.6: that sensor, at its height, reads the wall beneath a real Toggle's body);
  *(RC41)* the Toggle's two **brackets** — which hang to ≈ 10.6 in, into the beam's band (A8) — are `Fixed` and never
  lifted (Q2: with the whole keep-out lifted, a beam at a bracket SEATED 0.96–0.98 in wrong at 10–10.5 in up).
- **(b)** No keep-outs (the bound catches it). **Cost, measured (P2 S4b):** beside a Loader the reset SEATED x
  **3.71 in wrong** inside a 4-in bound (3.69 in on the F1 field — RC40).
- **(c)** *(added by RC36)* The Load Zones as PIECE zones, lifted with a measured cone like the start pieces. **Gain:**
  ~2 × 5 in more wall near each corner once the cone is measured. **Cost:** a robot collecting VEX U loads in a Load
  Zone during the auton, or a nested stack it drops, is taller than the beam-height gate assumes — the one place on the
  field where such objects are EXPECTED during the auton.

**D9 — Heading correction and a passive "correct when it can" mode: both OUT. RECOMMENDED (a).**
- **(a)** **Heading is never corrected.** One sensor per side gives ONE range — two unknowns (the axis and the angle)
  cannot come out of it; and the P1 numbers show the axis barely needs the heading: at a 2° heading error the axis
  moves 0.009 in with the sensor at the fore-aft centre (0.17 in with it 5 in forward). **No passive mode:** every
  wrong SEATED answer the probes found (S4b, S7b, S10b — and, before RC34's beam-height gate, S5d and A12's start
  pieces) passes every reading gate; what makes a reset safe is the
  author's assertion "I have parked beside THIS wall, HERE" plus the keep-outs and the bound. A background corrector
  has no such assertion and runs where nobody chose (a partner beside the robot, a Pin against the wall) — OUT until
  a chunk proves otherwise (§5).
- **(b)** A passive corrector behind the 12-in innovation gate. **Cost:** a robot parked 5 in from our side reads as a
  wall 5 in away — P2 S7b's 2.96-in wrong seat, every tick, nudged in.

**D10 — Mounting advice for the builders (no code; robot one now, robot two if a sensor can move). RECOMMENDED (a).**
- **(a)** Pass to the build team: each side sensor's lens **10–10.5 in above the floor** *(RC34, corrected: the draft
  said 8–10 in "clears Pins and Cups" — it does not: the beam spreads, and A12's start pieces stand against every wall)*.
  The reset reads only as far as the beam's lower edge stays above a standing piece (6.5 in, v20.txt :4270, :4546) +
  0.5 in — the beam-height ceiling (§1.2 rule 4; Q1 (B)): with the unofficial ±18° cone, **≤ 9.2 in of reading at 10 in
  up, ≤ 10.8 at 10.5, but ≤ 6.2 at 9 in and ≤ 3.1 at 8 in**; with a measured ±12° cone ≤ 14.1 / 16.5 / 9.4 / 4.7.
  Below the wall top, **11.50 in (metal, A14) / 11.54 in (portable, A13)** (*corrected 2026-10-22: the draft read ~11.5
  by eye from A8*): the cone's TOP edge then rises over the wall at most readings (a ±18° cone 10.5 in up passes 11.5 in
  beyond ~3 in of range) and sees the field's outside or a Toggle's underside — farther than the wall, so harmless
  under the nearest-surface belief (HA-205, RC43), and inside the Toggle keep-out otherwise. The code strip (9.06–11.44 in,
  RC35) is under the beam at every useful height; **at the robot's fore-aft centre** — *load-bearing (RC38)*
  (the turning centre on a tank: the axis error from a heading error is the forward offset × the heading error — P2
  S11a 0.25 in at 5 in forward, S11b 0.006 in at 0; with the heading tolerance growing as the robot turns, a sensor 5 in
  forward is refused IMPRECISE from the first reset under VEX's published accuracy, Q1 (A)); **at least 2.5 in inboard** of anything on that side —
  *RC39, corrected: the draft said 1 in, which the reset's own 2.0-in `rangeMin` refuses (`OUT OF RANGE`) whenever the
  robot touches the wall* — `rangeMin` + 0.5 in, so a robot flush against the wall still reads inside the window; **facing straight out, rigid**,
  with nothing of the robot's own (an arm, a claw at any position) inside a ±18° cone.
- **(b)** Leave robot two's sensors where they are. Step 0.1 measures them and the gates use what is measured — *but
  (RC34) the height decides whether the reset works at all:* a lens below ~9 in leaves a ceiling under ~6 in (±18°) —
  inside the 2-in `rangeMin`… 6-in band only, below 200 mm — and below ~7.6 in none at all: every reset refuses `BEAM
  LOW` until the sensor moves up or Step 0.5 measures a narrower cone. Robot two's heights are unmeasured (HA-210).

**D11 — A flaw found in an earlier chunk: the cookbook's re-seed recipe overwrites the OTHER axis. RECOMMENDED fix
it in L6-A.** `docs/cookbook/05-mixing-tiers.md` :101-104 (and its compiled twin `test/cookbook_examples_test.cpp`
:646-649) teach `r.startAt(Pose2d{36_in, 0_in, c.chassis.pose().heading()})` "squared against the wall at x = 36 in"
— a wall measures x and says nothing about y, yet the recipe writes y = 0 (the test passes only because its robot
drove along y = 0). L6-A rewrites the recipe to keep y (`c.chassis.pose().y()`), states that `startAt` also clears
the drift term for both axes (D2 (b)), and points at the wall reset for a sensor-measured axis. (RESUMING's standard:
"a chunk that finds a flaw in an earlier chunk fixes it there.")

**D12 — REMAINING-WORK row 25's wording is wrong for SIDE sensors on a tank. RECOMMENDED: reword at the plan
commit.** Row 25 (:104; :107 at the tip) says "drive toward a wall until the sensor reads a set distance". A tank cannot drive
sideways, so a SIDE sensor never faces the wall the robot drives toward; robot two has no front sensor. What a side
sensor can do — and what L6 builds — is **re-seat the axis across a wall the robot has parked beside**, nose along
the wall. "Drive until a FRONT sensor reads X" is a different feature for a robot with a front sensor (OUT, §5).

**D13 — When job slots are short (your one-at-a-time rule of 2026-10-01, in force again since 2026-10-22 ~01:45),
where L6 sits beside the MECH core, CAL1 and VIS1. RECOMMENDED (a); with one slot, (b).** *(Added 2026-10-22 with §6;
REWRITTEN by RC29: the draft's (a) left out VIS1-A1 — L6-A2's prerequisite — and CAL1 A2a/A2b/B, which MECH-A5 waits on
(the MECH brief's N9), and understated the slip.)* The arithmetic, in SLOT-days (builder-days with ~½–1 of
verification each, from the briefs' ESTIMATES — none measured — and read as calendar days only as the brief's working
assumption): MECH-A3's rest ~½–1; MECH-A4 ~4¼–5¾; CAL1's rest (A2a's rest, A2b, B) ~4½–6; MECH-A5 ~4¼–6; MECH-A6
~2½–3½; VIS1-A1's rest ~½–1½; **L6-A1 + A2 + B ~7½–10½** (§2.1, as amended).
- **(a)** **Two slots for this stretch** (your budget call): MECH and CAL1 in one (A3 → A4 → CAL1 → A5 → A6, exactly as
  without L6: A5 ends ~Nov 5–10), VIS1-A1's rest → **L6-A1 → L6-A2 → L6-B** in the other (L6-B ~Oct 30 – Nov 3). Nothing
  MECH waits; Step 0.6–0.8 get ~2 weeks before the ~Nov 16 freeze. **Cost:** the second slot's budget.
- **(b)** **One slot, L6 right after MECH-A4:** MECH-A3 → MECH-A4 → VIS1-A1's rest → L6-A1 → L6-A2 → L6-B → CAL1 → MECH-A5
  → MECH-A6. MECH-A4 — the auton mechanism steps, the key 11-23 piece — is NOT delayed (L6-A1 shares no file with it,
  so nothing is gained by putting L6-A1 first). **L6-B ~Nov 5–10.** **Cost:** MECH-A5 slips by the whole L6 line plus
  VIS1-A1's rest, **~8–12 slot-days, to ~Nov 14–22** — at or past the ~Nov 16 freeze, so robot two's programs would go to
  11-23 without the mechanism service in its autons (driver control keeps T3's mechanisms on buttons); and VIS1-A1's
  rest goes ahead of MECH, which **amends VIS1's accepted rule** ("if a slot is short, VIS1 waits, never MECH",
  `VIS1-ai-vision-colours.md` :1690-1691) for that one run. VIS1-A2 and VIS1-B keep their accepted places (VIS1-B
  after A5, before A6; build-order :1693).
- **(c)** **One slot, MECH first** (VIS1's rule applied to L6): A3 → A4 → CAL1 → A5 → (A6) → VIS1-A1's rest → L6. **L6-B
  ~Nov 12–22 with A6 deferred, later with it** — at or past the freeze, little or no time for Step 0.6–0.8 on a field:
  "we definitely need distance sensors working" would not hold for 11-23.
- Your 10-14 words weigh (b) over (c) when only one slot is affordable: the distance sensors "definitely", the MECH core
  "only if frozen ~Nov 16" (the first-competition plan). Under every option **Step 0.1–0.5 need no code** and should run
  at the next bench session — they decide what the reset can do on robot two at all (RC34, RC38).

---

## The design in plain words (for the team lead)

- **What it does.** Robot two's odometry is its drive motors and its IMU — no tracking wheels, no GPS — so every
  skid adds a little position error that nothing removes (HANDOFF :133-137; guide 14's "a skid it cannot see",
  `docs/guide/14-what-it-cannot-do-yet.md` ~:362-367). A field wall is the one thing on the field that never moves.
  When the robot has **parked beside a wall, nose along it**, the side sensor facing that wall measures how far the
  wall is; from the sensor's measured position on the robot, that gives **the robot's distance from that wall —
  ONE coordinate** (x for the left and right walls, y for the red and blue walls). The library replaces that one
  coordinate of its estimate and touches nothing else: not the other coordinate, not the heading.
- **What the auton says.** "Reset x against the left wall with the left sensor, and do not move x by more than
  3 inches" — one line (D4). The answer is one word, **SEATED** or **REFUSED** with the reason, and one log line
  either way, with the numbers.
- **When it refuses — and it refuses rather than guess.** The worst thing this feature could do is quietly re-zero
  an auton WRONG in the middle of a match. So before any reading is used, the library checks, in this order, and
  refuses at the first failure:
  1. **the setup**: the auton started at a FIELD pose (*RC48*); the sensor exists and its mount was measured; the estimate is live; no motion is running (the
     estimate is never moved under a motion — *RC2*); the request is sane;
  2. **the aim**: by the estimate's heading, the sensor's beam points at THAT wall within 5° of square (a tank that
     parked crooked, or a sensor typed on the wrong side, fails here);
  3. **the place**: where the beam lands on the wall is not beside a Loader, under a Toggle, or in a corner (the
     season's keep-outs, widened by the beam's spread, a margin, and how far the robot may have drifted ALONG the wall
     since that coordinate was last set — 10 % of the distance driven, by default — *RC33*);
  4. **the beam's height** *(RC34)*: from the sensor's measured height and the beam's spread, the reading must be short
     enough that the beam's lower edge stays above a standing Pin or Cup (6.5 in) all the way to the wall — so a reading
     can never be a single game piece standing in front of the wall. That sets how far the reset can read: about
     10.8 in for a sensor 10.5 in up, 6.2 in at 9 in up, 3 in at 8 in up (with the unofficial ±18° beam; more once
     Step 0.5 measures the real one);
  5. **stillness**: the robot is stopped (≤ 1 in/s, ≤ 3°/s) for the whole sampling window (~0.15–0.3 s);
  6. **every reading**: no read fault (MECH-A1's channel: a held value is never a reading), a real return (not
     PROS's "no object"), enough confidence, inside 2–16 in;
  7. **the readings agree with each other**: at least 5, spread ≤ 0.5 in (an edge, a flickering see-through panel or
     a moving object spreads them), and their middle value is still within the beam's height limit;
  8. **the answer is precise enough**: the error bound computed from the sensor's published accuracy, the mount's
     tolerances and the heading's is ≤ 1 in;
  9. **the answer agrees with the estimate**: the wall may move the coordinate by at most the author's
     `maxCorrection`. A bigger disagreement means either the odometry is badly off OR something other than the wall
     is in the beam — the library cannot tell which, so it refuses and says so.
  If all pass, the coordinate is SEATED to the wall's answer.
- **What it cannot catch — said plainly.** Something **taller than a game piece, at a plausible distance** in the beam
  passes every check: a robot parked beside ours, a stack (a Pin nested in a Cup) against the wall, a sensor wired to
  the other side's port, something close behind a CLEAR wall panel (RC37). The probes show it (P2: a backwards sensor
  with a robot 5 in away → 2.96 in wrong). *A single standing Pin or Cup no longer does (RC34):* the beam-height check
  refuses any reading the beam could take from one — before it, the start pieces A12 puts against every wall seated
  1.2–2.9 in wrong under an 8–9-in sensor, and a Pin under a 4-in sensor 2.59 in (Q2, P2 S5d). Three things shrink the
  rest: **a tight
  `maxCorrection`** (the author knows how far the robot can have drifted since the last reset), and **Step 0's
  side check** (a board held to ONE side; that side's port must read it). The guide says this in so many words.
- **How accurate it is** *(RC38, corrected: the draft's "best just past 8 in … ~0.74–0.92 in" read VEX's two
  approximations as a measured sweet spot and held the heading's error to 2°)*. VEX publishes "approximately ±15 mm"
  below 200 mm and "about 5 %" above — two approximations that meet at 200 mm, not an optimum — so the reset assumes the
  LARGER of the two at every range (±0.59 in out to ~11.8 in, 5 % beyond) until Step 0.3 measures the real spread. The
  heading's error GROWS with turning (the register's HA-84: 1 % of the rotation, 1σ), so the reset assumes 2° + 2 % of
  every degree turned since the heading was last set. Worst case all together (Q1 (A)), with a mount measured to
  ±0.25 in and ±2°: the sensor at the fore-aft centre **~0.91–0.98 in** at 2–10 in of reading before any turning (just
  inside the 1.0-in limit) and **~1.02–1.19 in after 180° of turning — refused IMPRECISE**; 5 in forward of centre
  **~1.09–1.15 in from the start — refused**. What makes it usable is Step 0: the mount measured to ±0.1 in and the
  sensor's accuracy measured at ±0.3 in bring the centred sensor to **~0.46–0.52 in** before turning (0.78–1.16 after
  450°). So the fore-aft centre (D10) is load-bearing, and Step 0.1 and 0.3 are what let the reset pass its own
  precision check after a turn.
- **Where it can be used on the Override field.** Four walls, inner faces 70.2 in from the centre (A10). The
  **left and right walls** (as seen from the red station: x = ∓70.2) have only a Toggle at their centre; the **red
  and blue walls** (y = ∓70.2) also carry two Loaders each, near the corners (A10; FO-1). **Every wall also starts the
  match with pieces against it** — A12's Cup-Pin-Cup clusters, 1.58 in off the wall, 18.8–28.3 in either side of its
  centre (VEX U's Figures VEXU-1 and VURS1 show the same) — and VEX U teams load pieces through the Loaders during the
  auton too (RC36). Where the beam may land *(RC34, corrected: the draft said "~2 × 40 in of clean face")*, with every
  keep-out widened by the beam's spread, the 3-in margin and the drift (RC33): the **VEX U Load Zones** — the 23.11-in
  squares in every corner, where loads come in DURING the auton and robots gather to take them (RC36) — are always kept
  out; **until Step 0.5 measures the cone**, the start pieces are too — about **2 × 8 in per wall** right after a seed
  (the beam centre 33.6–41.8 in either side of the wall's centre at a 7-in reading), 2 × 3.5 in after 24 in of driving,
  none after 48 in; **once the cone is measured** the start pieces lift (the beam-height check alone keeps single
  pieces out) — **2 × 23.6 in** per wall right after a seed (18.3–41.8 in either side of the centre), 2 × 14 in after 48
  in, 2 × 4.4 in after 96 in (Q1b) — **so reset early in a leg, or reset the perpendicular axis first** (its travel
  counter restarts, RC33). **A wall reset needs a FIELD start** *(RC48, corrected: the draft let a RELATIVE-start auton
  "carry its own keep-outs" — with no way to make them)*: the season's walls and every keep-out are field coordinates,
  and a RELATIVE-start auton (the team's 40 in auton is one, `src/team/autons.cpp` :52-56, :66) does not know where they
  are in its own frame — the everyday call refuses it, `REFUSED not a field start`. **The team has no FIELD-start auton
  yet**; its first wall-reset auton will be its first.
- **Clear wall panels.** A VEX Forum thread ("V5 Distance sensor passes through field walls", vexforum.com/t/111715 — read only through search-result summaries: the forum answers 403 to a fetch) says the
  sensor sees THROUGH the metal field perimeter's clear panels or drops out beyond ~700 mm, and works on the portable
  perimeter's opaque walls. If that is true at our events, the reset will USUALLY refuse there (no return / out of
  range / disagrees) — **but not always** *(RC37, corrected: the draft said "never seat wrong from it")*: something
  opaque standing within `maxCorrection` BEHIND the clear face — the metal wall's own structure (A14 draws a 1.27-in
  section), a drive-team member's hand or controller resting on the wall — reads as the wall and SEATS the axis wrong
  by up to that distance (Q2: an object 1 in behind → 1.09 in wrong, 2 in → 1.99 in; 3 and 4 in → refused `disagrees`).
  A tight `maxCorrection` caps it; nothing in the sensor's reading can tell. **Step 0.6 tests it on whatever field the team
  can reach**, and the guide tells teams to check their event's perimeter (HA-206). *(RC35)* **The GPS code strip** —
  black and white squares along every wall at 9.1–11.4 in, required on VEX U and skills fields — is where the beam
  lands at every useful mount height; whether a black square reads like a white one is not published, so Step 0.3
  and 0.6 RECORD both.
- **What it does not do.** It never turns or moves the robot; it never corrects heading (D9); it does not run in the
  background (D9); it does not use the camera; it does not stop the drive at a distance (D12). It does not use the
  sensor's "object size" or "object velocity" outputs (`hal/pros/distance.hpp` :40-44 leaves them unbound) — Step 0.3
  records object size against the wall and against a Pin, so a later chunk can judge whether it would separate them.

---

## 0. Why — what exists, what the sensor and the field offer, and the probes (every line cited was OPENED at `ceb17c0`)

### 0.1 Why this chunk, and why now
- The team lead's ruling of 2026-10-14 (header) puts L6 before the panel and, if it can, before 2026-11-23. The
  first competition's plan is robots verified, mechanisms on buttons, student autons (memory: first competition) —
  L6 is the one sensor feature that makes a student auton's LATER legs land where the earlier ones intended.
- **No absolute position source exists on either robot:** no GPS (`robot.hpp` :537-541 builds `AbsentGps` unless
  the config says present; robot two's boot config sets `c.sensors.gps.present = false`,
  `src/common/boot_source.hpp` :310), no tags (`AbsentTagSource`, `robot.hpp` :542), no tracking wheels (HANDOFF
  :133-137). The drift is real and unmeasured: the saved calibration's scale read ~8 % short (`roadmap.md` ~:411),
  and a sideways skid is invisible (HA-180).
- The handoff's L6 entry (`HANDOFF-remaining-work.md` :485-500) named three gaps; their state at `ceb17c0`: (1) **no
  one-axis setter** — still true (`localizer.hpp` :519-532 `setPose` moves both and clears the drift term); (2)
  **nothing detects a drive stall** — still true, and irrelevant to a SIDE-sensor reset (D12: the robot parks
  first; it never drives into the wall the sensor faces); (3) **no `distance()` on the provider** — closed by
  MECH-A1 (`hal/device_provider.hpp` :174-178).

### 0.2 What exists in the tree
| Piece | Where (opened) | What it gives L6 / what it lacks |
|---|---|---|
| The frozen distance seam | `include/shulib/hal/distance.hpp` :15-38 — `distance()` (inches), `confidence()` ([0, 1]); "read confidence() FIRST" (:16-19) | F4, LOCKED 2026-06-19 (`roadmap.md` :138). Untouched by L6 |
| The PROS adapter | `hal/pros/distance.hpp` :11-22 (raw 9999 → confidence 0, a READING, not a fault), :24-31 (at ≤ 200 mm confidence is reported 1.0 — the raw channel is undefined there), :33-38 (PROS_ERR → hold the last good value, count it), :40-44 (object size and velocity DELIBERATELY NOT BOUND), :75-76 ("distance() and confidence() are two samples, not one atomic snapshot"), :82 (`ProsDistance` is also an `IReadFaults`), :90-121 | Every gate L6 needs on a single reading exists; the two-read non-atomicity is a gate rule (§1.2 rule 6) |
| Its conversions | `hal/distance_conversion.hpp` :47 (`kDistanceNoObjectMm` 9999), :52 (`kDistanceConfidenceAvailableAboveMm` 200), :55 (full scale 63), :61-66 (÷25.4) | HA-113/114/115 (`hardware-assumptions.md` :959-990) — all unsettled |
| The read-fault channel | `hal/read_faults.hpp` :12-23 (a held value is byte-for-byte a reading; "the core reads this count BESIDE the value"), :56-58 (monotonic); `hal/fake/fake_read_faults.hpp` (`failing(true)` counts every read) | L6 reads the count before and after each sample; a move refuses the window (MECH-A2's rule, `mechanism/service.hpp` :232, :382) |
| The provider | `hal/device_provider.hpp` :174-178 (`distance(port)`, idempotent, nullptr + `lastRefusal()`), :192-196 (`readFaults(port)`); `hal/pros/device_provider.hpp` :441-463 (`distance()`: range, `expectKind(Distance)`, construct in a try), :516-528 (`readFaults` answers the `ProsDistance` it made) | **No provider change.** One port can serve one role: `valid()`'s one-owner claim (below) keeps a wall sensor and a mechanism's piece sensor apart |
| The robot file | `config/robot_config.hpp` :325-332 (`SensorsConfig { GpsConfig gps; }`, "TAGS AND VISION ARE DELIBERATELY ABSENT"), :524-536 (`PortUse`, MECH-A1 appended `MechanismSensor`), :539-553 (`portUseWord`, an exhaustive switch), :619-640 (`valid()` and its `claim` lambda) | L6 appends `SensorsConfig::walls` and `PortUse::WallSensor` (D5) |
| The Robot | `robot/robot.hpp` :537-581 (the constructor: GPS, tags, vision, the Localizer, `RobotContext`, deps, the chassis), :597 (`chassis()`), :603 (`localizer()`), :770, :787, :798 (the boot line's "GPS %s tags ABSENT vision ABSENT") | L6 builds the two sensors and puts an OPTIONAL pointer in the context (D5) |
| The context | `chassis/robot_context.hpp` :9-11 ("NOT part of the F4 freeze — it grows additively as M3/M4 consumers need more sensors (distance / optical …)"), :31-46 (the config's named pointers), :65-77 (every slot must be non-null) | L6's slot is the first OPTIONAL one: nullptr = "this robot has no wall sensors", checked by the reset, never by the context's constructor (landmine 4) |
| The estimate | `localization/localizer.hpp` :43-48 (the caller seed and "nothing that reads a sensor can reach reseatHeading()"), :86-126 (the boot guard), :208-236 (the constructor), :378-384 (an applied fix clears the drift term by its confidence), :509-532 (`setPose`: both axes, drift term → 0, `hasLast_ = false`), :534-564 (`reseatHeading`, precondition: estimate live), :579-610 (`refreshQuality`) | L6 adds `seatAxis` beside them (D2) |
| The odometry seam | `localization/odometry.hpp` :79-81 (`setPose` teleports position, "must re-baseline"), `drive_encoder_odometry.hpp` :214-220 and `pilons_odometry.hpp` :198-204 (`setPose` keeps the encoder baselines, re-takes the heading reference) | *(RC1)* `seatAxis` does NOT touch the odometry: its `setPose` would re-take the heading reference (`prevHeading_ = imu_.heading()`, drive :219, pilons :203), and nothing but the Localizer's deltas reads an odometry's position |
| Never-snap | master plan :547 (decision #4, "gated nudge (never snap)" … "`relocalize()` feeds this corrector — it does not snap"), :409 and :639 (`relocalize(featureSource)`), :528-532 (risk 5, "corrector pose-jump mid-motion"); `complementary_fusion.hpp` :63-70 (12 in gate, 12 in/s budget, gain 0.15) | D2 (a) AMENDS the locked decision for the wall reset alone (*RC25*); the fusion path is untouched |
| The facade | `chassis/chassis.hpp` :5-21 (F6 FROZEN), :480-485 (`pose()`, `setPose()`), :472-475 (`waitUntil(pred, Time)`) | L6 changes nothing in F6; the reset samples inside `chassis.waitUntil` (it ticks the Localizer every pace — `motion_scheduler.hpp` :819-849) |
| The recipe layer | `chassis/routine.hpp` :5-22 (F10 FROZEN; additive growth legal), :227-235 (`startAt` = `setPose`), :426-462 (`then()`: void / bool / ExitReason / MechanismOutcome) | L6-A uses `then()` (bool); L6-C adds a member after MECH-A4 (D1, D4) |
| The run guard's waits | `sequence/run_guard.hpp` :95-109 — `Chassis::waitUntil` is a FROZEN wait the guard cannot cut; lateness = its own remaining timeout | The reset's sampling window is bounded (≤ 0.5 s, landmine 5) |
| The auton's world | `competition/auton_registry.hpp` :154-199 (`StartPose`: FIELD or RELATIVE), :215-229 (`AutonContext`: chassis, guard, telemetry, mechanisms, entry) | The everyday reset reads `ctx.chassis`, `ctx.guard` (its deadline-aware wait) and `ctx.entry` (the start mode) — §1.12, *RC48*; a FIELD start is what makes the season's walls meaningful, and the everyday call requires one |
| The host test plant | `sim/drive_plant.hpp` :62-80 (sensors synthesized FROM TRUTH through the A3 seams), :384-386 (`truePose()` for the harness only); `hal/fake/fake_distance.hpp` :11-22 (settable distance and confidence); NO field, NO wall, NO distance model anywhere in `include/shulib/sim/` (grep, §0.4) | L6 adds `sim/field_plant.hpp` (§1.7) |
| Robot two | `R2a-sensor-stations.md` :14-17 (distance 4 left, 5 right, census 2026-09-24); `src/robots/chassis_table.hpp` :69-91 (`ChassisTable`: no sensor field but the IMU's), :124-147 (robot two's table) | The side sensors are NOT in any table today; their mount is unmeasured (HA-210) |
| Bench Tests station 11 | `src/programs/bench_r3a.cpp` :2051-2081 (`readDistance`: raw mm, confidence, size, velocity + the adapter), :2118-2136 (the RECORD line), :2487-2617 (the station; :2553 "hold square to a wall at 6/12/24 in; RECORD each") | **Step 0.2–0.6 need NO new code** (§2.0); L6-B adds the WALL page |
| The re-seed recipe | `docs/cookbook/05-mixing-tiers.md` :94-127; `test/cookbook_examples_test.cpp` :636-655 | The y-overwrite flaw (D11) |

### 0.3 What the sensor, the field and the rules give (read 2026-10-18)
- **The V5 Distance Sensor** (api.vex.com's V5 Python "Distance Sensor" page, fetched 2026-10-18): range **20–2000
  mm**; "Below 200mm, accuracy is approximately ±15mm. Above 200mm, accuracy improves to about 5%"; a Class 1 laser,
  "detection is directed straight ahead"; object size as NONE/SMALL/MEDIUM/LARGE (PROS: 0–400, "A 18" x 30" grey
  card will return a value of approximately 75", vendored `include/pros/distance.hpp` :160-188). **Not published:**
  the field of view, the update rate, the latency, behaviour on clear or shiny surfaces. A community post (VEX Forum
  "V5 Distance Sensor Technical Details", t/88482, seen only through search summaries — 403 to a fetch) gives a cone
  of **±12°** at long range and **~±18°** in a short-range mode — **unofficial** (HA-205).
- **The perimeter** (A10, `img/a-105.png`): inner faces **3566.4 mm (140.41 in) apart**, the centre at 1783.2 mm —
  walls at **±70.205 in** in the F1 frame. Two perimeter types are legal (`<T9>`, v20.txt :3057-3059), and **their
  drawings differ** (*corrected 2026-10-22, viewed*): **A13** (`img/a-108.png`, the PORTABLE perimeter) — 140.40 in
  (3566.4 mm) inside wall to wall, **wall height 11.54 in**, walls 2.00 in thick; **A14** (`img/a-109.png`, the METAL
  perimeter) — **140.50 in (3568.7 mm)**, **wall height 11.50 in**, 1.27 in thick. So the walls sit at ±70.20 or ±70.25
  in — 0.045 in apart per wall, inside HA-207's tolerance; L6 uses A10's 1783.2 mm (the drawing the field elements are
  placed on) and HA-207 names the difference. The portable one's walls are opaque (v20.txt :3007: replacing them
  "with transparent panels" is not allowed); the metal one's panels are polycarbonate — the manual says so only in
  passing (`<T7>`, v20.txt :3024: "sponsor decals on polycarbonate panels", which also means a DECAL may sit where a beam
  lands, event by event) and never says "clear"; that they are clear is the forum report and common knowledge
  (HA-206). "Field components may vary slightly from event to event" (A3 text, v20.txt :4076) — HA-207.
- **The Toggle** (A8, `img/a-103.png`; glossary v20.txt :4666-4668): 25.99 in long, sits on brackets ON TOP of the
  wall, axle 13.20 in up, faces ~2.05 in; one centred on each wall (A10; FO-1). **So a sensor whose beam stays below
  the wall top reads the WALL beneath a Toggle's body, not the Toggle** *(RC41: except at its two brackets, which hang to
  ≈ 10.6 in at |along| ≈ 10.9–13.0 — A8 at 300 dpi; they stay kept out)* — the task's premise that a side sensor near a
  wall's centre "will see the Toggle instead of the wall" holds only for a beam that reaches above the wall top (a
  high mount, a far reading, the short-range cone): P2's S10b — a sensor 11.5 in up, cone ±18° — read the Toggle and
  seated 0.75 in wrong. The keep-out stays on by default (D8) because the Toggle's overhang past the wall's inner
  face is read by eye from the drawings, and the cone that decides whether a beam reaches above the 11.50/11.54-in
  wall top is unofficial (HA-205, HA-208).
- **The Loaders** (A9 `img/a-104.png`, A10): 4.02 in wide, protruding **3.74 in** from the wall, 22.97 in tall —
  two on each ALLIANCE wall, centred 290.6 / 3275.8 mm from the audience-side wall in A10's figure, i.e. **58.77 in
  either side of centre**. FO-1 (`img/fo-019.png`) puts the Alliance Stations (and so the Loaders) on the walls A10
  draws left and right; F1 ("+Y away from red", `roadmap.md` :134; :135 at the tip) makes those the **red wall y = −70.205** and the
  **blue wall y = +70.205**, the Loaders at **x = ±58.77** on each, and the two other walls **x = ∓70.205** —
  "left" and "right" as seen from the red station — with no Loader. *(RC36)* **The Load Zones** (glossary v20.txt
  :4425-4427: bounded by the perimeter and the tape around each Loader; A10 draws that tape 587.1 mm = 23.11 in from the
  corner along BOTH walls) — a 23.11-in square in each of the four corners; **in VEX U, Match Loads are introduced
  through the Loaders during the AUTONOMOUS period too** (`<VUG4>`, :3511-3512), placed or dropped through a Loader's
  top or back, "separately or nested together", and removed only through its bottom opening (`<SG11>` b, d, :1346-1351).
- **Goals** are 23.11 in from the nearest wall at their closest (A10: the nine goal centres at 23.11 / 46.66 /
  70.20 / 93.75 / 117.30 in from a wall) and at most 8.7 in tall (v20.txt :4377-4380) — not in a side beam of a
  robot parked against a wall, but a robot parked with its OTHER side toward a goal reads the goal from the other
  sensor; L6 uses only the sensor the request names.
- **Pins and Cups** are both 6.5 in tall standing; a **Pin is ~1.6 in (40 mm) across, a Cup ~3.15 in (80 mm)**
  (v20.txt :4546, :4270; A5 — *RC42, corrected: the draft said "3.16 in across lying" for both*), so a Pin lying on the
  floor is ~1.6 in tall and a Cup ~3.15 in; **at the match start a Cup-Pin-Cup
  cluster stands against EVERY wall on both sides of its centre** *(RC34)* — A12 (`img/a-107.png`): centred 1.58 in off
  the wall at 43.50 / 46.66 / 49.82 and 90.59 / 93.75 / 96.91 in from the corner, i.e. |along| 18.81–28.28 in; VEX U's
  Figure VEXU-1 (manual p.83) and the VEX U Skills Figure VURS1 (p.87) show the same — and pieces stand anywhere after. **Robots** start within 18 in
  (v20.txt :1064); both of ours are 15 in.

### 0.4 The probes (`.claude/coordinator/scratch/brief-L6/`, run by this writer 2026-10-18; every binary under `ulimit -v 4 GiB` + `timeout`)
- **Tree:** `tree/` = `git archive ceb17c0 include test src tools`, with ONE prototype edit:
  `tree/include/shulib/localization/localizer.hpp` gains `seatAxisProto` (D2 (a), ~20 lines before `reseatHeading`).
- **P1 — the geometry and the error budget** (`probes/p1_geometry.py` → `out/p1_geometry.txt`), worked for a 15-in
  robot (the sensor's left offset 7.5 in), the left wall, the robot nose along +Y:
  - the sensor's accuracy by range (VEX): ±0.591 in up to 7.87 in; ±0.400 at 8 in; ±0.600 at 12; ±0.800 at 16;
    ±1.200 at 24 (P1a) — *(RC38, withdrawn: "the sweet spot is just past 200 mm" read two approximations as an optimum —
    the reset uses max(15 mm, 5 %), flat at ±0.59 in to ~11.8 in)*;
  - geometry alone (exact sensor), heading error 0–2°: sensor at the fore-aft centre — ≤ 0.012 in square to the wall,
    ≤ 0.071 in at 5° skew (d 12 in); 5 in forward — 0.18–0.25 in (P1b); the first-order term is
    **forward offset × heading error** (P1c: f 5 in, 2° → 0.165 in; f 0 → 0.009);
  - a mount typed wrong passes through ~1:1 in the LEFT offset (0.25 in → 0.25 in), not at all in the forward
    offset, and only as 1 − cos in the facing (5° → 0.031 in) (P1d);
  - the "which distance does a cone report on an oblique wall" question (beam centre d/cos ε vs perpendicular d):
    **≤ 0.046 in at 12 in inside the 5° gate**, 0.42 in at 15° (P1e) — the 5° gate makes it moot;
  - the full budget (skew ≤ 5°, heading ≤ 2°, mount ±0.25 in and ±2°, the sensor's published accuracy), worst case:
    **f = 0: 0.909 / 0.744 / 0.968 / 1.192 / 1.644 in at 4 / 8 / 12 / 16 / 24 in; f = 5: 1.082 / 0.918 / 1.141 /
    1.366 / 1.818** (P1f) — the 1.0-in bound (D7) admits ~5–12 in with the sensor at the centre; *(RC38: P1f used the
    200-mm step and a fixed 2°; **Q1 (A)** re-does it with max(15 mm, 5 %) and the growing heading tolerance — f = 0:
    0.91–0.98 in at 2–10 in before any turning, 1.02–1.19 after 180°; f = 5: 1.09–1.15 from the start; with Step 0's
    ±0.1-in mount and ±0.3-in measured accuracy, f = 0: 0.46–0.52)*;
  - the footprint (P1g): ±1.70 in at 8 in (±12°), ±2.60 (±18°); at 12 in ±2.55 / ±3.90 — at 9 in up, the ±18° cone
    spans 6.4–11.6 in at 8 in range.
- **P2 — the rules on the host test plant** (`probes/p2_scenarios.cpp` + `probes/proto/{wall_proto,field_model}.hpp`
  → `out/p2_scenarios.txt`; built with the suite's flags `-std=c++20 -Wall -Wextra -Wpedantic -Werror -Wshadow
  -Wconversion -Wsign-conversion -Wdouble-promotion`, against `tree/include` + `tree/test`). A tank `ChassisRig`
  (`test/motion_test_rig.hpp` :230-245) drives a 24-in leg nose-first along the left wall, parks, and the prototype
  evaluates 10 samples from an **independent** cone-of-rays model (7 × 7 rays, nearest hit, boxes with heights:
  walls 11.5 in tall, Toggles above them, Loaders, Pins, a robot) from the TRUE pose and TRUE mount, ±0.15 in noise:
  | Scenario | Verdict | Estimate error after |
  |---|---|---|
  | S1 clean, estimate drifted +2.5 in | SEATED (median 8.050) | **+0.050 in** |
  | S2 drifted +6 in, `maxCorrection` 4 | REFUSED disagrees | +6.000 (untouched) |
  | S3 parked 8° off parallel | REFUSED not-parallel | — |
  | S4a beside a Loader, keep-outs ON *(RC40: re-run in F1 as S4a', the RED wall)* | REFUSED keep-out | — |
  | **S4b beside a Loader, keep-outs OFF** *(RC40: in F1, S4b' — median 4.309)* | **SEATED** (median 4.286) | **−3.714; −3.69 in F1 — WRONG, inside the bound** |
  | S5a lying Pin at the wall, sensor 4 in up *(RC42: the box was 3.16 in tall, end-on to the wall — a lying CUP, not a Pin; `p2_scenarios.cpp` :134)* | REFUSED disagrees (median 2.729) | — |
  | S5b lying Pin, sensor 9 in up *(RC42: likewise a lying Cup)* | SEATED | −0.005 |
  | S5c standing Pin 1 in off the wall, sensor 9 in up | SEATED | +0.027 |
  | **S5d standing Pin 1 in off the wall, sensor 4 in up** | **SEATED** (median 5.409) | **−2.591 — WRONG** *(RC34: now refused `BeamLow` — a 4-in lens has no ceiling)* |
  | S6 a read fault at sample 4 | REFUSED read-fault | — |
  | S7a sensor mounted BACKWARDS, open field | REFUSED no-return | — |
  | **S7b BACKWARDS, a robot 5 in to its right** | **SEATED** (median 5.038) | **−2.962 — WRONG** |
  | S8 see-through (clear) wall panels | REFUSED no-return | — |
  | S9 sampled while driving 12 in/s | REFUSED moving | — |
  | S10a sensor 11.5 in up at the wall's centre, keep-outs ON | REFUSED keep-out | — |
  | **S10b the same, keep-outs OFF** | **SEATED** (median 7.251, the Toggle) | **−0.749 — WRONG** |
  | S11a sensor 5 in forward, estimate's heading +2° off | SEATED | +0.254 |
  | S11b sensor at the centre, heading +2° off | SEATED | +0.006 |
  | S12 mount typed 0.5 in too far out | SEATED | +0.499 |
  - **S5b's seat through `setPose` instead** (D2 (b)): the drift term **23.77 → 0.00** (the one-axis seat keeps
    23.77) — the next tick's quality class would read DeadReckon, not Degraded.
  - *(RC40, corrected:)* P2's field model put the Loaders on the LEFT/RIGHT walls (`probes/proto/field_model.hpp` :73,
    :87-90 — A10's drawing frame, not F1's); §0.3 and §1.1's F1 placement — the red/blue walls, y = ∓70.205 — is the
    right one (FO-1; VEX U's Figure VEXU-1). The first pass re-ran S4 on an F1-frame field (`recheck/r3_f1_field.cpp` →
    `recheck/r3.txt`, re-read): **S4a'** (a y reset against the RED wall beside the Loader at x 58.8, keep-outs ON) →
    `KeepOut`; **S4b'** (OFF) → SEATED, **−3.69 in** — and the old S4 script (LEFT wall) on the F1 field seats correctly
    (+0.009), as it must with no Loader there. D8 (b)'s evidence stands at 3.69 in. No other P2 row involves a Loader.
  - Every WRONG row is a real object at a plausible distance; each is what D8 (keep-outs), D10 (mount height) and
    Step 0.4 (the side check) exist for. None of them is caught by a reading gate, and the brief does not pretend
    one could be (§8, landmine 1).
- *(RC24: P3 compiled the PROTOTYPE — plain doubles, `std::vector`, no `Chassis`, `seatAxisProto(bool, …)` — not §1.2's
  surface; the draft's §1.2 did NOT compile (an include cycle). §1.2's amended layout is compiled by **Q3**,
  `recheck2/q3_layout.txt`: host, both include orders, and the ARM gate over every header in one unit.)*
- **P3 — the prototype and the seat compile for the brain** (`probes/p3_arm.cpp` → `out/p3_arm.txt`): the ARM gate's
  flags (`-std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror -Os -mcpu=cortex-a9
  -mfpu=neon-fp16 -mfloat-abi=softfp`) over `wall_proto.hpp` + the edited `localizer.hpp`: **compiles clean** (only
  GCC's psABI `note:` lines for `std::vector` iterators — the real code uses a fixed-capacity sample buffer and
  allocates nothing, §1.2).
- **P4 — the quality class after a seat, MEASURED** (added 2026-10-22 at the continuation; `probes/p4_quality.cpp` →
  `out/p4_quality.txt`; the same tree, rig and flags as P2, `-O1`, under `ulimit -v 4 GiB` + `timeout 300`): a 24-in
  drive leaves the drift term at 23.77 in, class Degraded, quality 0.200; seat x 0.3 in over, let TWO ticks run
  (`waitUntil(false, 0.02 s)`, `p4_quality.cpp` :33 — *RC30, corrected: the draft said ONE; per tick, the first is the dt-0
  re-base tick, Degraded in both — seatAxis 0.100, setPose 0.500 — and the second is the line below*) —
  **(a) the one-axis seat: Degraded, 23.77, 0.200, y moved 0.0000; (b) `setPose` with y copied: DeadReckon, 0.00,
  1.000, y moved 0.0000.** D2 (b)'s cost is a measurement, not a reading of the code.
- **Found while reading:** (1) the cookbook's re-seed recipe writes y = 0 (D11); (2) REMAINING-WORK row 25's
  "drive toward a wall" cannot apply to side sensors on a tank (D12); (3) `RobotContext` documents itself as
  growing "additively as M3/M4 consumers need more sensors (distance …)" (:9-11) but every slot today is a required
  non-null — L6's is the first optional one (landmine 4); (4) the guide says the field runs "about −70 to +70"
  (`docs/guide/02-the-field-and-coordinates.md` :17-18) — *withdrawn 2026-10-22: "about" makes it true; L6's guide
  section states ±70.2 (A10/A13) and ±70.25 (A14) itself, and guide 02 is not touched*; (5) *added 2026-10-22* cookbook
  05's re-seed paragraph says "the simulator's ground truth" — the project's word is the host test plant; L6-A1
  rewrites that paragraph for D11 and fixes the word with it.

---

## 1. Scope — IN (every new and changed file, by run)

### 1.1 `include/shulib/localization/override_walls.hpp` — NEW: the season's walls (L6-A1)
- Namespace **`shulib::localization::override2026`** — *(RC3, corrected 2026-10-22: the draft said
  `shulib::override2026` "like VIS1's"; VIS1-A1's real header is `shulib::vision::override2026`,
  `vis1-a1/include/shulib/vision/override_elements.hpp` :33, :123)*. The reason on its merits: season data lives
  beside the subsystem that consumes it (VIS1's pattern), so the two seasons' headers never share a namespace and a
  name in both can never collide; next season adds `localization::override2027` beside it.
- `inline constexpr units::Length kWallInnerFace{1783.2 / 25.4}` (70.205 in, A10 — the drawing the elements are
  placed on; A13's portable perimeter agrees, A14's METAL one is 1784.35 mm, 70.25 in: the 0.045-in difference is
  stated beside the constant and in HA-207, never silently averaged — *added 2026-10-22*) and the four walls as
  `localization::FieldWall` values (type in §1.2): **`kLeftWall`** (x = −70.205, inward +1), **`kRightWall`**
  (x = +70.205, inward −1), **`kRedWall`** (y = −70.205, inward +1), **`kBlueWall`** (y = +70.205, inward −1) —
  "left"/"right" as seen from the red station, F1 (`roadmap.md` :135 at `71b4764`). Each carries its keep-out spans ALONG the wall
  (§0.3): the Toggle in three spans (*RC41*): its BODY, |along| ≤ 10.9 in (`Toggle` kind — lifted per side by the mount's
  measured `toggleChecked`), and its two BRACKETS, 10.9 ≤ |along| ≤ 12.995 in (25.99/2, A8 — `Fixed`: they hang to ≈ 10.6
  in, HA-208), the corners (|along| ≥ 70.205 − 12), and on the red and blue
  walls the Loaders (|along| ∈ [56.76, 60.78]: 58.77 ± 4.02/2, A9/A10) — all `Fixed`; *(RC34)* on all four walls the
  start pieces (|along| ∈ [18.81, 28.28]: A12's clusters, the Cups' outer edges) as `PieceZone` spans; *(RC36)* on all
  four walls the VEX U **Load Zones** (|along| ≥ 70.205 − 23.11 = 47.095: A10's tape) as `Fixed` spans (they contain the
  corners and the Loaders, which stay listed for their own `why`). Every number carries its source line in a
  comment (`A4 register HA-207` / `HA-208` / `HA-212`), and the executor re-checks A8–A10 in the newest manual (header).
- A file-banner paragraph: what the field frame is here (F1, red at −y), that a FIELD start pose is what makes these
  walls meaningful (`StartPose::field`, `auton_registry.hpp` :154-199) — *RC48: the everyday call REFUSES any other
  start (§1.12)* — and that `FieldWall::line` (frame `Own`, with the author's keep-outs or none) serves the explicit,
  Tier-3 form only.

### 1.2 `include/shulib/localization/{axis,wall_reset}.hpp` — NEW, PURE: the geometry, the gates, the verdict; `chassis/wall_reset_call.hpp` — NEW: the explicit call (L6-A1); `competition/wall_reset.hpp` — NEW: the everyday call (L6-A2, §1.12)
*Rewritten as real C++ in the second re-check (RC24): the draft's listing put `Axis` in `wall_reset.hpp`, which also
declared `resetFromWall(chassis::Chassis&, …)` and so had to include `chassis/chassis.hpp`; with `seatAxis(Axis, …)` in
`localizer.hpp` that closes the cycle chassis.hpp → motion/command_pipeline.hpp → motion/motion.hpp → localizer.hpp
(`motion.hpp` :140 includes it, :187 and :260 use `Localizer`) — the checker compiled the draft's form in three include
orders and all three failed (`check-tree/round2/tu/axis_variant.txt`). It also broke master plan §5's rule (:197,
"no layer includes a header from a layer above it"): no `localization/` header includes `chassis/` or `motion/` today.
So `Axis` lives in a LEAF header; `wall_reset.hpp` is PURE (no `chassis/`, `motion/`, `sequence/` or `competition/`
include — §7 greps it); the explicit call sits at the facade's layer, as `chassis/routine.hpp` does over `chassis.hpp`;
the everyday call sits in `competition/`, where the auton's context lives (RC48). Probe **Q3**
(`recheck2/q3/`, `q3_layout.txt`): this layout — the leaf, the pure header, the season walls as `constexpr` data, the
two calls, the robot file's block in a `constexpr` table, D4's listing inside `namespace shulib::bench` — compiles
with the suite's flags in both include orders, and every v2 header plus these in ONE translation unit compiles with the
ARM gate's flags. The members later entries add are named in each entry (keep-out kinds RC34/RC41, the measured
accuracy RC38, the cone RC34, the drift term RC33, the heading tolerance RC38, `NotFieldStart`/`Cut`/`BeamLow` RC34/RC48,
the outcome log RC47).*
PROS-free, `sim/`-free, allocation-free (a fixed `std::array<WallSample, kMaxWallSamples>`), no exceptions on a refusal
(a refusal is a VALUE); preconditions only for programming errors (a non-finite argument). The angle defaults are
spelled `N * kWallDegToRad` because the `_deg` literal is not `constexpr` (`units/literals.hpp` :64-72) and the robot
file's block must sit in the `constexpr` `ChassisTable` (`src/robots/chassis_table.hpp` :124) — the tree's own
pattern (`calibration/drivetrain_calibrator.hpp` :274). Its surface (the executor may rename only with the
coordinator's agreement; every member documented with `///` — the API doc gate):
```cpp
// include/shulib/localization/axis.hpp — the LEAF that localizer.hpp and wall_reset.hpp both include
namespace shulib::localization {
enum class Axis : std::uint8_t { X, Y };   // the field axis a wall measures
}

// include/shulib/localization/wall_reset.hpp — PURE (master plan §5: no header from a layer above)
namespace shulib::hal { class IDistance; class IReadFaults; }
namespace shulib::localization {
inline constexpr double kWallDegToRad = math::Angle::kPi / 180.0;
enum class WallSide : std::uint8_t { Left, Right };                 // which of the robot's two side sensors
enum class WallFrame : std::uint8_t { Field, Own };                  // a season wall (F1) / an author's own line
enum class KeepOutKind : std::uint8_t { Fixed, Toggle, PieceZone };  // never lifted / toggleChecked / a measured cone

struct WallKeepOut { units::Length from{}; units::Length to{}; KeepOutKind kind = KeepOutKind::Fixed; const char* why = ""; };
inline constexpr std::size_t kMaxWallKeepOuts = 12;

struct FieldWall {                       // the line `axis = coord`, the field on the `inward` (+1 / -1) side
    const char* name = "";               // "left wall", printed in the log line
    Axis axis = Axis::X;
    units::Length coord{};
    std::int8_t inward = +1;
    WallFrame frame = WallFrame::Field;
    std::array<WallKeepOut, kMaxWallKeepOuts> keepOut{};
    std::uint8_t keepOutCount = 0;
    [[nodiscard]] static constexpr FieldWall line(const char* name, Axis axis, units::Length coord, std::int8_t inward,
                                                  std::span<const WallKeepOut> keepOuts = {});   // frame = Own
};

struct SideMount {                                          // one side sensor, body frame, MEASURED (Step 0)
    bool measured = false;                                  // false = UNSET: every reset with it REFUSES
    units::Length forward{};                                // from the robot centre (the turning centre on a tank)
    units::Length left{};
    units::Length height{};                                 // the lens centre above the floor
    units::AngleDim facing{};                               // CCW from forward: 90 * kWallDegToRad = out the left side
    units::Length positionTolerance{0.25};                  // how well forward/left were measured (Step 0.1)
    units::AngleDim facingTolerance{2.0 * kWallDegToRad};   // how square it was found (Step 0.1)
    units::Length rangeBias{};                              // subtracted from every reading; 0 = none measured (D6)
    units::Length rangeAccuracy{};                          // Step 0.3's measured +/-; 0 = VEX's max(15 mm, 5 %)
    units::AngleDim coneHalfAngle{};                        // Step 0.5's measured; 0 = UNMEASURED: piece zones on
    bool toggleChecked = false;                             // Step 0.6 at this height: the Toggle BODY lifts
    const char* provenance = "";                            // who measured what, how, when
};

struct WallResetLimits {                                    // D7 (a): PROVISIONAL (A4: HA-211), per robot
    units::Length rangeMin{2.0};
    units::Length rangeMax{16.0};
    units::AngleDim squareWithin{5.0 * kWallDegToRad};
    units::Length spreadMax{0.5};
    int minSamples = 5;
    units::Time window{0.15};
    units::Time windowMax{0.5};
    double minConfidence = 0.5;
    units::Velocity stillSpeed{1.0};
    units::AngularVelocity stillTurn{3.0 * kWallDegToRad};
    units::Length errorBoundMax{1.0};
    units::Length keepOutMargin{3.0};
    double alongDriftRate = 0.10;                           // x the travel since the along axis was seeded
    units::AngleDim coneHalfAngle{18.0 * kWallDegToRad};    // while the mount's cone is unmeasured
    units::Length pieceTop{6.5};                            // the beam-height ceiling's piece
    units::Length beamMargin{0.5};
    units::AngleDim headingFloor{2.0 * kWallDegToRad};      // the heading tolerance = floor + perTurn x turned
    double headingPerTurn = 0.02;
};

struct WallResetRequest {                                   // D3, D4, D7
    FieldWall wall{};
    WallSide sensor = WallSide::Left;
    units::Length maxCorrection{0.0};                       // NO default: 0 is UNSET, refused by name (D7)
    bool required = false;
    const char* name = nullptr;
};

struct WallSample { units::Length distance{}; double confidence = 0.0; bool faultMoved = false; };
inline constexpr std::size_t kMaxWallSamples = 64;          // landmine 12

enum class WallResetVerdict : std::uint8_t {                // APPEND-ONLY from L6-A1's landing; the words: §1.2's verdict table
    Seated, NoSensor, SensorAbsent, MountUnset, NotLive, MotionActive, BadRequest, NotFieldStart, NotSquare,
    KeepOut, BeamLow, Moving, ReadFault, NoReturn, LowConfidence, OutOfRange, TooFewSamples, Spread, Imprecise,
    Disagrees, Cut };
[[nodiscard]] constexpr const char* wallResetVerdictWord(WallResetVerdict v) noexcept;   // exhaustive, no default:

struct WallResetReport {
    WallResetVerdict verdict = WallResetVerdict::NoSensor;
    Axis axis = Axis::X;
    units::Length before{}, measured{}, applied{};          // applied = 0 unless SEATED
    units::Length median{}, spread{}, expectedRange{}, errorBound{}, ceiling{}, along{};
    int samples = 0;
    bool required = false;
    [[nodiscard]] constexpr bool seated() const noexcept;
    [[nodiscard]] constexpr bool continues() const noexcept;   // seated() || !required (D3)
};

struct WallSensor {                                         // one side as the reset reads it (built by robot::Robot, §1.5)
    hal::IDistance* sensor = nullptr;
    const hal::IReadFaults* faults = nullptr;
    SideMount mount{};
    const char* absence = "not declared";
};
struct WallResetLog { int seated = 0; int refused = 0; WallResetReport last{}; };   // the last outcome (the tester reads it)
struct WallSensors { WallSensor left{}; WallSensor right{}; WallResetLimits limits{}; WallResetLog log{}; };

/// PURE: the robot centre's coordinate a reading implies (beam-centre model, the mount's bias removed).
[[nodiscard]] units::Length axisFromReading(units::Length reading, const math::Pose2d& pose, const SideMount& mount,
                                            const FieldWall& wall);
/// PURE: the worst-case error of that coordinate — the accuracy (mount.rangeAccuracy, else max(15 mm, 5 %) of the
/// reading: A4 register HA-204), the mount's tolerances and `headingTolerance`, at the corners of the tolerance box
/// (not linearised; the Q1/P1 sweep is its oracle).
[[nodiscard]] units::Length axisErrorBound(units::Length reading, const math::Pose2d& pose, const SideMount& mount,
                                           const FieldWall& wall, units::AngleDim headingTolerance);
/// PURE: headingFloor + headingPerTurn x the |rotation| since the heading was last seeded.
[[nodiscard]] units::AngleDim headingTolerance(units::AngleDim turnedSinceHeadingSeed, const WallResetLimits& limits);
/// PURE: the farthest reading whose cone stays above pieceTop + beamMargin, from the TYPED height and cone.
[[nodiscard]] units::Length beamCeiling(const SideMount& mount, const WallResetLimits& limits);
/// PURE: the gates that need no reading (§ the rules 2-4), in order; Seated means "go on and sample".
[[nodiscard]] WallResetVerdict preCheck(const math::Pose2d& pose, const SideMount& mount, const WallResetRequest& request,
                                        const WallResetLimits& limits, units::Length alongUncertainty);
/// PURE: the whole verdict from a window of samples (the rules 5-9).
[[nodiscard]] WallResetReport evaluateWallReset(std::span<const WallSample> samples, const math::Pose2d& pose,
                                                const math::Twist2d& maxTwist, const SideMount& mount,
                                                const WallResetRequest& request, const WallResetLimits& limits,
                                                units::AngleDim headingTolerance, units::Length alongUncertainty);
}  // namespace shulib::localization

// include/shulib/chassis/wall_reset_call.hpp — the facade's layer (chassis/routine.hpp's pattern)
namespace shulib::chassis {
/// THE EXPLICIT CALL (L6-A1 — a hand-wired Tier-3 robot's form, and the host tests'): blocking, at most
/// limits.windowMax + two ticks; the setup and pre-checks, then samples inside chassis.waitUntil (its predicate ENDS the
/// wait itself — the wait never times out, so no SCH line), evaluates, SEATS on success through
/// chassis.deps().localizer->seatAxis, ONE log line either way. Commands nothing to the drive. The heading tolerance
/// and the along uncertainty come from the Localizer's counters (§1.3).
[[nodiscard]] localization::WallResetReport resetFromWall(Chassis& chassis, const localization::WallSensor& sensor,
                                                          const localization::WallResetRequest& request,
                                                          const localization::WallResetLimits& limits);
}  // namespace shulib::chassis

// include/shulib/competition/wall_reset.hpp — L6-A2: THE EVERYDAY CALL (§1.12)
namespace shulib::competition {
[[nodiscard]] localization::WallResetReport resetFromWall(AutonContext& ctx, const localization::WallResetRequest& request);
}
```
**The rules, in the order they run** (the verdict names the FIRST that fails; the log line carries its numbers):
1. **Setup:** *(RC48)* in the everyday form, first the start — not FIELD → `NotFieldStart` (§1.12); the context has a `WallSensors` (else `NoSensor`); the named side's sensor exists (else
   `SensorAbsent`, with the Robot's absence sentence); its mount is measured (else `MountUnset`); the estimate is live
   (`qualityClass() != Uninitialized`, else `NotLive`); *(RC2)* **no motion is active**
   (`chassis.scheduler().hasActiveMotion()`, else `MotionActive` — checked BEFORE any sample: the sampling wait
   keeps an armed motion ticking, `motion_scheduler.hpp` :833-848, and a seat under it is master-plan risk 5's
   "corrector pose-jump mid-motion"; `reseatHeading`'s contract, `localizer.hpp` :549-553, and `runAuton`'s check,
   `match_runner.hpp` :147-149, are the precedent — every `Routine` motion step is blocking and detaches before it
   returns, `chassis.hpp` :556-566 ("waitUntilSettled cannot return while it is active"), so an auton's ordinary reset never meets it; a Tier-3 `async()` hold does); the
   request is sane — `maxCorrection` > 0 and finite, the wall's `inward` ±1, keep-outs ordered (else `BadRequest`,
   naming the field).
2. **Square:** by the ESTIMATE's heading and the typed facing, the beam's angle off the wall's outward normal ≤
   `squareWithin` (else `NotSquare`, with the angle). A beam pointing AWAY from the wall is `NotSquare` (cos < 0).
3. **Keep-out:** the beam centre's along-wall coordinate ± (expected range × tan(cone) + `keepOutMargin` + **the along
   uncertainty**) overlaps no keep-out (else `KeepOut`, naming the span's `why` and printing the beam's along
   coordinate). *(RC33)* The along uncertainty is `alongDriftRate` × `localizer.travelSinceSeat(the wall's ALONG axis)`
   — the path length since that axis was last seeded (`setPose` / `startAt`, the runner's start seat) or seated (a reset
   against a perpendicular wall): the keep-out is tested at the ESTIMATE's along coordinate, which is exactly the axis
   this reset does not measure, so it must be widened by how far that axis may have drifted. **Keep-out KINDS**
   *(RC34)*: `Fixed` spans always apply; a `Toggle` span is lifted only by the mount's measured `toggleChecked` (RC41);
   a `PieceZone` span — where single pieces are KNOWN to stand: A12's start pieces — applies while
   the mount's cone is UNMEASURED (`coneHalfAngle` 0) and lifts once Step 0.5's measured half-angle is typed, because
   rule 4 then rests on a measured cone.
4. **Beam height** *(RC34)*: the expected range ≤ `beamCeiling(mount, limits)` = (mount.height − `pieceTop` −
   `beamMargin`) / tan(cone), the cone being the mount's MEASURED half-angle or, unmeasured, `limits.coneHalfAngle` (else
   `BeamLow`, printing the ceiling and the height). At or under the ceiling the cone's lower edge stays above a standing
   6.5-in piece + 0.5 in at every range up to the reading, so the nearest surface cannot be a single Pin or Cup — nor
   the floor, which a cone meets only beyond height / tan(cone). Rules 1–4 run BEFORE any sample: a refused aim costs no
   time (test T4).
5. **Stillness**, over the whole window: the estimate's |v| ≤ `stillSpeed` and |ω| ≤ `stillTurn` on every tick (else
   `Moving`).
6. **Each sample** is `{distance, confidence, faultMoved}` taken as: count c0 = `faults->faultedReads()`; d =
   `distance()`; c = `confidence()`; c1 = count again; `faultMoved = (c1 != c0)` (the two reads are two device reads,
   `hal/pros/distance.hpp` :75-76 — both must be fault-free). Refuse the window at the first sample with `faultMoved`
   (`ReadFault`); confidence 0 (`NoReturn` — PROS's 9999 and a held value both land here); confidence <
   `minConfidence` (`LowConfidence`); `distance − rangeBias` outside [`rangeMin`, `rangeMax`] (`OutOfRange`). A sensor
   with NO read-fault count (`faults == nullptr` — a fake, or a provider that never screens) is accepted, and the log
   line says `faults n/a` (landmine 7).
7. **Enough, consistent, and under the ceiling:** ≥ `minSamples` samples over ≥ `window` (else `TooFewSamples`); max −
   min ≤ `spreadMax` (else `Spread`). The value used is the MEDIAN, and *(RC34)* the median ≤ the beam ceiling (else
   `BeamLow`): rule 4 judged the ESTIMATE's expected range; this judges what the sensor saw — an estimate that put the
   wall nearer than it is cannot let a piece past the ceiling through (M37).
8. **Precise enough:** `axisErrorBound(median, pose, mount, wall, headingTolerance)` ≤ `errorBoundMax` (else
   `Imprecise`, with the bound). *(RC38)* The accuracy term is the mount's measured `rangeAccuracy` (Step 0.3) or,
   unmeasured, **max(15 mm, 5 % of the reading)** — never a step at 200 mm (the draft's model dropped the bound by
   0.13–0.20 in across 200 mm — Q1 (A2): 0.898 at a 7.80-in reading → 0.772 at 7.90; the checker's f2: 0.932 → 0.737 —
   a cliff in a gate); `headingTolerance` = `headingFloor` + `headingPerTurn` ×
   `localizer.turnedSinceHeadingSeed()` — 2° + 2 % of the |rotation| since `setPose` / `reseatHeading` (HA-20's floor
   as before; HA-84's 1-σ 1 % of rotation, doubled the floor's way).
9. **Agrees:** |`axisFromReading(median, …)` − the estimate's coordinate| ≤ `maxCorrection` (else `Disagrees`, with both
   numbers and the limit). Then **SEAT** (§1.3) and report `applied`.

**The log line** (one per call, `hal::LogLevel::Info` when SEATED, `Warn` when REFUSED, subsystem `"WALL"`, built with
`diag::lineformat::Line` — ASCII, within its 384 bytes, `diag/line_format.hpp` :49):
`left wall x SEATED (LEFT sensor): -52.21 -> -54.66 (moved -2.45, limit 3.00) | median 8.05 in spread 0.26 n 10 | bound 0.62 | faults ok`
`left wall x REFUSED disagrees (LEFT sensor): wall says -54.71, estimate -48.71, limit 4.00 | median 7.98 in spread 0.28 n 10`
`left wall x REFUSED keep-out (LEFT sensor): Load Zone at along -45.2 (widened 6.5 = beam 2.3 + margin 3.0 + drift 1.2) | clean -40.6..-34.8`
(the request's `name`, when set, leads the line; *RC46: "by LEFT" became "(LEFT sensor)" — the draft's form read as a
direction*).

**Every verdict, its word, its numbers and what to do** *(RC46 — the draft specified words for a few verdicts and a
remedy for none but `BadRequest`; guide 11 promises "what it means, what causes it, what to do about it",
`docs/guide/11-reading-the-diagnostics.md` :4-5, with a "What to do" column, :331)*. `wallResetVerdictWord` returns the
word; the line carries the numbers; **guide 11 carries this table verbatim** and guide 12 a symptom entry ("my wall reset
keeps refusing" → read the word, then this row):

| Verdict | Word | The line also prints | What to do |
|---|---|---|---|
| Seated | `SEATED` | before → after, moved, the limit, median, spread, n, the bound, faults | nothing; now and then compare the next leg's end with the tape |
| NoSensor | `REFUSED no sensor` | — | this robot file declares no wall sensors: add the `walls` block (guide 10). On the host test plant's tester rig it is expected — that rig has none (guide 16) |
| SensorAbsent | `REFUSED sensor absent` | the side, its port, the provider's reason | check the cable and the port; the boot line's `walls L4 R5(ABSENT)` says which |
| MountUnset | `REFUSED mount unset` | the side | measure the mount (Step 0.1), type it with `measured = true` |
| NotLive | `REFUSED not live` | — | the estimate never went live (the IMU still calibrating?) — read the boot lines; a program error |
| MotionActive | `REFUSED motion active` | — | a motion is still armed (a Tier-3 `async()`): let it finish or cancel it first |
| BadRequest | `REFUSED bad request` | the field, e.g. `maxCorrection UNSET: say how far this wall may move x` | fix that field in the auton |
| NotFieldStart | `REFUSED not a field start` | the start mode | the auton starts RELATIVE: declare a FIELD start (`StartPose::field(…)`) — the season's walls are field coordinates (RC48) |
| NotSquare | `REFUSED not square` | the beam's angle off the wall's normal | 5°–20°: `turnTo` square before the reset. Near 90° or 180°: this beam faces away from that wall — the wrong side named (LEFT/RIGHT), the wrong wall, or the nose not along the wall |
| KeepOut | `REFUSED keep-out` | the span's name (Toggle, Toggle bracket, Loader, Load Zone, start pieces, corner), the beam's along-wall coordinate, the widening, the nearest clean span | park so the beam lands in the wall's clean band (guide 03's table); after a long drive the band narrows — reset earlier in the leg, or the other axis first |
| BeamLow | `REFUSED beam low` | the expected range or the median, the ceiling, the lens height and cone | park nearer the wall, under the ceiling; a ceiling near zero means the sensor is too low (D10) or its cone unmeasured (Step 0.5) |
| Moving | `REFUSED moving` | the speed or turn rate seen | brake and let the leg settle first (`r.brake()`, a short `pause`) |
| ReadFault | `REFUSED read fault` | the sample number | the port faulted mid-window — check the cable; Bench Tests station 11 |
| NoReturn | `REFUSED no return` | the sample | nothing within ~78 in — or a CLEAR wall panel (HA-206): see what the beam faces |
| LowConfidence | `REFUSED low confidence` | the confidence | a dark, shiny or slanted surface, or the code strip (HA-206) — Step 0.3/0.6's readings say which |
| OutOfRange | `REFUSED out of range` | the reading and the window | park so the reading is ~6–8 in; never flush (D10's 2.5 in) |
| TooFewSamples | `REFUSED too few samples` | n and the window | a pacing problem in the program — report it |
| Spread | `REFUSED spread` | max − min | the beam is on an edge, a seam or something moving — move along the wall |
| Imprecise | `REFUSED imprecise` | the bound and its limit | the sensor is far from the fore-aft centre, the robot has turned a lot since its heading was set, or the mount and accuracy are unmeasured — Step 0.1 / 0.3, typed (RC38) |
| Disagrees | `REFUSED disagrees` | the wall's answer, the estimate, the limit | EITHER the robot drifted more than `maxCorrection` OR something other than the wall is in the beam (a robot, a stack): look first; raise the limit only from Step 0.8's taped legs |
| Cut | `REFUSED cut` | — | the run's deadline or a disable landed in the window; nothing changed |

**Choosing `maxCorrection`** *(RC46; the procedure the first pass's RC22 asked for — its text is not on disk)*: run the
auton up to the reset WITHOUT it, several times, and tape the error on that axis at the reset point; the largest seen plus
~1 in is the drift this wall may correct. Never set it near the gap to the nearest thing a wrong seat could mistake for
the wall (landmine 1) — a tight limit is the defence the gates cannot give.

**Sampling** in the EXPLICIT form uses the chassis's own wait, so the Localizer and every health check keep ticking;
the window is bounded by `windowMax` (a frozen wait the run guard cannot cut — `run_guard.hpp` :95-109 — so its lateness
past a deadline is at most `windowMax`; the guide says so). *(RC48)* The EVERYDAY form (§1.12) samples through the run
guard's deadline-aware `waitFor` instead: a deadline or a disable mid-window returns at once, refused `cut`, nothing
seated. One sampler (a template over the wait) serves both — no gate is written twice.

### 1.3 `include/shulib/localization/localizer.hpp` — CHANGED: `seatAxis` (L6-A1, D2)
- `void seatAxis(Axis axis, units::Length value)`: PRECONDITIONS — the estimate is live (`reseatHeading`'s rule and
  message shape, :556-558) and `value` finite; and *(RC2)*, **by contract since the Localizer cannot see the
  scheduler, NO MOTION is active** — `reseatHeading`'s own words (:549-553), stated in `seatAxis`'s doc; the wall
  reset checks it (rule 1, `MotionActive`) before it samples. Moves `fusedX_` OR `fusedY_` to
  `value`; re-takes `lastFused…` (as `setPose` does — redundant while `hasLast_` is false, so no mutation is claimed
  for it); `pose_` recomposed with the CURRENT heading; `twist_ = {}`; and **`hasLast_ = false` — LOAD-BEARING**: the
  next tick's `dt` is 0, so `EkfFusion` re-bases onto the seated prediction (`ekf_fusion.hpp` :441-444, `resync`)
  instead of reading the seat as one tick's odometry increment (:450, `ux = px − lastX_`) — RC1's probe R1: without
  it 1.68 in of a 2-in seat bounces back and the filter's velocity reads 167.6 in/s. *(RC1, corrected 2026-10-22: the
  draft also shifted the odometry through `odom_.setPose`; that is DROPPED.)* **The odometry is NOT touched:**
  nothing reads an odometry's absolute position but the Localizer, which reads only its per-tick deltas (:266-281);
  `IOdometry::setPose` would re-take the heading reference (`prevHeading_ = imu_.heading()`, drive_encoder :219,
  pilons :203) — a side effect a one-axis seat has no reason to cause; `reseatHeading` leaves the odometry untouched
  too (:544-547). The member's doc says why `setPose`'s "the predictor and the fused belief never diverge" (:509-511)
  does not apply here: an applied correction already lets them diverge, by design (STEP 2's note, :246-249).
  **Leaves** `distanceSinceCorrection_`, `headingBias_`, `qualityClass_`/`quality_`, `lastCorrection_` and the
  odometry untouched (D2 — and the member's doc says why: one axis measured is not a fix).
- *(RC33, RC38)* `#include "shulib/localization/axis.hpp"` (the leaf, RC24), and **two read-only counters** the reset's
  gates need, kept beside the drift term and touched only by bookkeeping: `[[nodiscard]] units::Length
  travelSinceSeat(Axis) const noexcept` — the path length (the same `hypot(odx, ody)` the drift term adds, :382-383) since
  that axis was last seeded by `setPose` (both axes → 0) or seated by `seatAxis` (that axis → 0); and `[[nodiscard]]
  units::AngleDim turnedSinceHeadingSeed() const noexcept` — the sum of |wrapped heading change| per tick since `setPose`
  or `reseatHeading`. Neither is changed by a corrector's applied fix (conservative: a partial GPS fix does not make the
  wall reset trust the other axis; no corrector is fitted on our robots). Neither changes any existing observable (T16). Prototype:
  `seatAxisProto` in `scratch/brief-L6/tree/…` (it still has the dropped shift — landmine 15).
- The banner's caller-seed paragraph (:43-48) and decision-#4 echoes gain one sentence each: a wall reset (L6) is a
  caller step that MAY write one position axis from a sensor reading, inside gates the author bounds; no sensor can
  reach the HEADING (unchanged, D9). The executor greps `never snap|never-snap|nothing that reads a sensor` over
  `include/` and the public docs and words each hit (the COMP-a lesson recorded at :54-63).

### 1.4 `include/shulib/config/wall_sensor_config.hpp` — NEW, and its hooks in `config/robot_config.hpp` (L6-A2, D5)
- `struct WallSensorConfig { std::uint8_t port = 0; localization::SideMount mount{}; }` (0 = no sensor on that side);
  `struct WallSensorsConfig { WallSensorConfig left{}; WallSensorConfig right{}; localization::WallResetLimits limits{};
  const char* note = ""; }` (*RC38, RC41: the heading tolerance is now the limits' floor + per-turn growth, and the
  Toggle flag is per side, in the mount — the draft's `headingTolerance` and `beamUnderToggleChecked` fields are gone*).
- `robot_config.hpp`: the include; `SensorsConfig` APPENDS `WallSensorsConfig walls{};` (after `gps`, :329-332 — the
  "TAGS AND VISION" sentence is not touched; a line is added saying the side sensors are declared here since L6);
  `PortUse` APPENDS `WallSensor` (:524-536) with its case in `portUseWord` ("a wall sensor") — the switch is
  exhaustive (`-Wswitch -Werror`); at `ceb17c0` it is the ONLY switch on `PortUse` in `include/`, `src/` and `test/` (grep `case PortUse::` — re-check at the tip); `valid()`
  claims each non-zero wall port through the existing `claim` (:626-640) and refuses, by name: a port outside 1..21;
  the same port on both sides; a MEASURED mount with a non-finite number, |forward| or |left| > 24 in, a height ≤ 0 or
  > 11.5 in (the wall top, A14 — a lens above it cannot read the wall; *RC34*), a measured cone half-angle outside
  3°–30° (0, unmeasured, is valid), a tolerance ≤ 0 or > 2 in (position) / > 10° (facing), a bias with |bias| > 2 in; limits out of order
  (`rangeMin` ≥ `rangeMax`, `rangeMin` < 0.79 in — VEX's 20 mm —, `minSamples` < 3 or > 64, `window` > `windowMax`,
  `windowMax` > 1 s; *RC38:* a measured `rangeAccuracy` outside 0.05–2 in, `headingFloor` outside 0.5°–10°,
  `headingPerTurn` outside 0–0.1, `alongDriftRate` outside 0–0.5, `pieceTop` or `beamMargin` < 0). An UNMEASURED mount is NOT a config error (the robot boots; the reset refuses `MountUnset`).
- **Nothing in this header names a PROS type** (the PROS-free guard covers `config/`).

### 1.5 `include/shulib/robot/robot.hpp` — CHANGED: the Robot builds the side sensors (L6-A2, D5)
- A member `walls_` (a `localization::WallSensors`, declared BEFORE the context so `&walls_` can be handed to it)
  whose sensors are filled **in the constructor BODY, after VIS1-A1's `buildCamera(devices)`** — *corrected
  2026-10-22: the draft built them in the initializer list after `vision_{}`; VIS1-A1 (worktree `vis1-a1`) builds its
  camera LAST in the body "so every provider request before it … is exactly what it was", and a fake provider's
  request log is pinned by tests — L6 keeps every earlier request where it was the same way*. From
  `cfg_.sensors.walls`: for each side with a port, `devices.distance(port)` and `devices.readFaults(port)` (MECH-A1's
  accessors); a nullptr sensor leaves that side absent with the provider's `lastRefusal()` sentence COPIED into a
  member buffer (the provider's pointer is valid only until its next refusal — `device_provider.hpp` :217-222; VIS1-A1's
  `copyCameraReason` is the model); a side with port 0 reads "not declared". The context never reads the slot during
  construction (nothing dereferences it before the first reset), so filling it in the body is safe — a test pins it.
- `RobotContextConfig` gains `.walls = &walls_` (§1.6). Accessors: `const localization::WallSensors& wallSensors()
  const noexcept`, `hal::IDistance* wallSensor(localization::WallSide) noexcept` (nullptr when absent),
  `const char* wallSensorAbsence(localization::WallSide) const noexcept` ("" exactly when present) — the mechanism
  accessors' contract (:646-696).
- `describe()` (:740-806): *corrected 2026-10-22 against VIS1-A1's working tree* — after VIS1-A1 the three format
  strings end `"GPS %s tags ABSENT %s"` with the CAMERA's words last ("camera ABSENT", "camera port 3 ABSENT (the provider's reason)",
  "camera port 3 set up, model OK" — LAST "so a short buffer cuts the camera's reason, never the drive"). L6-A2 inserts
  **`walls %s `** between `tags ABSENT` and the camera's words, from one helper, short and fixed-width so the camera's
  reason stays the part a short buffer cuts: `walls L4 R5`, `walls L4 R5(ABSENT)` (a refused side: its port and
  "ABSENT"; the reason is in `wallSensorAbsence()` and in the reset's refusal line, not on the boot line),
  `walls L4 R5 MOUNT UNSET` when either declared mount is unmeasured, **`walls ABSENT`** with no side declared (the
  line's own word for an optional device not fitted — "GPS ABSENT", "tags ABSENT", "camera ABSENT"; the draft said
  `NONE`). ASCII only; the line still truncates rather than overflows (its contract, :744-750). **The tests that pin
  the "tags ABSENT" → camera boundary EXACTLY change with it** — L6-A2 edits them at VIS1-A1's merged form *(RC26,
  corrected against VIS1-A1's working tree, 2026-10-22: the draft named robot_builder_test and missed VIS1-A1's NEW
  file)*: `test/boot_source_test.cpp` :392-393 (the line's head ending "… GPS ABSENT tags ABSENT "),
  `test/xdrive_config_test.cpp` :234 (`"… GPS present tags ABSENT camera ABSENT"` after VIS1-A1), and **VIS1-A1's NEW
  `test/robot_camera_test.cpp`** — :344 and :366 (`endsWith(banner, "tags ABSENT camera …")`) and :401-429 (`head =
  "… GPS ABSENT tags ABSENT "` then eight `banner == head + "camera port 3 …"` checks): about ten pins. NOT changed:
  `test/robot_builder_test.cpp` :882-887 (`find("tags ABSENT")` and ends-with `"camera ABSENT"` still hold after the
  insertion). VIS1-A1's guide 10 example lines (:579-582, "… tags ABSENT camera …") gain the `walls …` words in L6-A2's
  guide 10 edit.
- `validate()` (the config + devices check) is NOT changed: a refused wall sensor never refuses the robot.

### 1.6 `include/shulib/chassis/robot_context.hpp` — CHANGED: the first OPTIONAL slot (L6-A2)
- `RobotContextConfig` APPENDS `localization::WallSensors* walls = nullptr;` (*RC47: non-const — the everyday reset
  records its outcome in the slot's `log`*; a forward declaration — the
  context includes no localization header: it is "L3, PROS-free: it sees only the L0 interfaces", :9) with the doc
  "OPTIONAL: nullptr = this robot declares no wall sensors; the wall reset refuses NO SENSOR — the one slot the
  constructor does NOT require". Accessor `[[nodiscard]] localization::WallSensors* wallSensors() const
  noexcept` (the pointer, as the other slots hand theirs out). The constructor's preconditions (:65-77) are unchanged; a test pins that a context built WITHOUT the slot
  still constructs (every existing caller — the host harness `sim/scenario.hpp`, the Robot, the rigs — is unchanged).
- **AND `include/shulib/motion/motion_scheduler.hpp` :660-667 — CHANGED, one line (*found 2026-10-22; the draft
  missed it*).** `Chassis::deps()` is `sched_.deps()` (`chassis.hpp` :514), and the scheduler's `deps()` returns
  `schedDeps_` (`motion_scheduler.hpp` :730), whose `ctx` is a **SHADOW context assembled field by field** from the
  real one (:660-667: clock, drive motors, IMU, GPS, battery, the STAMPED telemetry, tags, vision). A slot appended to
  `RobotContextConfig` and not carried there is **nullptr in every `chassis.deps().ctx`** — the everyday call would
  refuse `NO SENSOR` on every robot while every unit test of the Robot passed. The file's own comment records the
  same trap already sprung once ("motorGroups (R3b Part 1) was found dropped by exactly this line", :668-671). L6-A2
  adds `.walls = deps.ctx->wallSensors()` to the shadow, and T13 drives the everyday call through a real `Chassis`
  built by the Robot (mutation M31, §2.2).

### 1.7 `include/shulib/sim/field_plant.hpp` — NEW: the host test plant's field and side sensors (L6-A1)
- `sim::FieldModel`: axis-aligned boxes with heights (walls 11.5 in tall — HA-208 —, the Toggles above them, the
  Loaders — *RC40: on the red and blue walls, y = ∓70.205, the F1 frame; P2's prototype had them on the left/right* —, and whatever a test adds: a Pin standing or lying, a Cup, another robot), each opaque or SEE-THROUGH (the
  clear-panel case, HA-206), a floor, and *(RC34)* A12's start clusters on every wall (a Cup 3.15 in across, a Pin 1.6 in —
  RC42 — both 6.5 in tall), with `override2026()` building the field from ITS OWN copy of the drawing numbers (not
  from `localization/override_walls.hpp` — an independent model, the A2 rule; a test asserts the two agree to 0.01 in).
- `sim::SideSensorPlant`: a TRUE mount (forward, left, height, facing — set separately from the robot file's believed
  one: every adversarial case mismatches them), a cone of rays (default ±18°, 7 × 7) returning the NEAREST opaque hit — HA-205's belief; *(RC43)* a `blend` mode (the
  mean range over the cone's hits) lets a test show which rules hold either way: the beam-height gate does (no piece is
  in the cone at all at or under the ceiling), the known-limit magnitudes and the top edge's harmlessness may not —
  a code-strip band on every wall (9.06–11.44 in, A8 — *RC35*) with a per-colour range bias and confidence scale, default 0 /
  1 so the plant's walls read true until Step 0.3 gives numbers; `> 78.74 in` → PROS's 9999 (published into the fake as confidence 0); seeded noise (`sim/rng.hpp`), a bias, an
  update interval with a hold between updates and a latency (a k-tick delay line — HA-209's two unknowns), drop-outs,
  a FROZEN mode, and read faults through a `hal::fake::FakeReadFaults`. Each `step()` reads the harness's `truePose()`
  (ground truth for SYNTHESIS only — the plant's own rule, `drive_plant.hpp` :77-80) and publishes into a
  `hal::fake::FakeDistance` (`fake_distance.hpp`, unmodified). Allocation-free after construction is NOT required here
  (host only), but the model is deterministic.
- `shulib/sim/` stays unreachable from the library (the CI layering guard).

### 1.8 `include/shulib/version.hpp` — bumped ONLY by L6-C (*corrected 2026-10-22*)
The draft bumped `kApiMinor` at every landing. The project bumps it only for growth of a FROZEN surface
(`version.hpp`'s `kApiMinor` doc: "Bumped for additive extensions of a frozen surface"), and every landing since 2.4 on
unfrozen surfaces says "No version bump" in its changelog entry — COMP-a's `Localizer::reseatHeading` is the exact
precedent ("`Localizer` and `robot::BuildOptions` are outside every freeze", `changelog.md` :1086-1090). So: **L6-A1**
(new headers, `Localizer::seatAxis`), **L6-A2** (`RobotConfig`, `Robot`, `RobotContext` — "NOT part of the F4
freeze", `robot_context.hpp` :9-11 — none in a Freeze Register row) and **L6-B** (`src/`) bump NOTHING; their
changelog sections are "Added since 2.x … No version bump" with that reason (`competition/wall_reset.hpp` included —
a new header, no frozen surface grows — *RC48*). **L6-C** (OPTIONAL, not recommended — D4 (b)) appends to `Routine` (F10,
FROZEN) and a `RoutineStopCause` enumerator: `kApiMinor` + 1 with `kApiVersionString` and the comment block's
paragraph (L9 Part A1's 2.4 is the model). MECH-A4 takes 2.5 (MECH brief §1.9); whichever F10-growing run lands after
it takes the next number (§6.3).

### 1.9 Tests — NEW files: L6-A1 `test/wall_reset_geometry_test.cpp`, `test/wall_reset_gates_test.cpp`,
`test/wall_reset_plant_test.cpp` (the P2 scenarios on the host test plant, through the explicit call),
`test/localizer_seat_axis_test.cpp`, `test/sim_field_plant_test.cpp`; CHANGED `test/cookbook_examples_test.cpp`
(D11's recipe). L6-A2 `test/wall_sensor_config_test.cpp`, `test/robot_wall_sensors_test.cpp` (the Robot, the context
slot, the everyday call end to end on the host test plant), *(RC48)* `test/competition_wall_reset_test.cpp` (the context
form: the start check, the guard's cut, the log); CHANGED (*added 2026-10-22; corrected by RC26*) the
boot-line pins of §1.5 — `test/boot_source_test.cpp`, `test/xdrive_config_test.cpp` and VIS1-A1's `test/robot_camera_test.cpp`
— at VIS1-A1's merged form. `test/CMakeLists.txt` only if the suite lists its files (the executor checks).

### 1.10 L6-B — robot two (`src/`; after L6-A2 merges, before MECH-A5 launches — D1)
- `src/robots/chassis_table.hpp`: `ChassisTable` APPENDS `config::WallSensorsConfig wallSensors{};` (VIS1's camera
  pattern; the bench bot's and the X-drive's tables keep the default — no side declared); robot two's block: **left
  port 4, right port 5, `mount.measured = false`** with a provenance line ("ports: census 2026-09-24; facing straight
  out: the team lead 2026-10-08; mount UNSET — Step 0.1 owed") — the coordinator types the measured mount from Step
  0.1's sheet in a separate small commit (or the executor, if Step 0.1's numbers exist when B runs).
- `src/common/boot_source.{hpp,cpp}` *(RC27, corrected: the draft's route — "a trailing `bootConfigFrom` parameter that
  `decideBootSource()` passes" — does not exist: `decideBootSource()` (`boot_source.cpp` :35-56) never calls
  `bootConfigFrom`; it calls the template `decideBootSourceFrom(record, registry, fromTable, missing, kMechanisms,
  kChassis.robot, printer)` (`boot_source.hpp` :468-475), whose LAST parameter is a forwarding `PrintLine&&`, so no
  defaulted parameter can follow it; the template calls `bootConfigFrom` at :587)*: a NEW overload
  `decideBootSourceFrom(record, registry, fromTable, missing, mechanisms, robotName, const config::WallSensorsConfig&
  walls, PrintLine&& printLine)`, to which the existing signature forwards with `walls = {}` — so its seven test call
  sites (`boot_source_test.cpp` :614; `driver_panel_claws_test.cpp` :177, :484, :821; `driver_controls_test.cpp` :150;
  `robot_two_driver_test.cpp` :169, :226) compile and behave unchanged, and `bootConfigFrom` (:289-325, its forward
  declaration :251 and friend :281, its 15 calls in `boot_source_test.cpp`) is NOT touched. After `bootConfigFrom` the
  overload copies `walls` into `out.built.config.sensors.walls` and checks it the way the claws are checked (:557-583 —
  "RobotConfig::valid() would refuse that too, but its words name a collision, not the fix"): a wall port that is a
  DRIVE port of the chosen description, a claw's port, or the other side's refuses the boot with a line naming the fix
  ("port 4 is both a wall sensor and a drive motor in the typed table -- src/robots/chassis_table.hpp"); `valid()` then
  agrees. `boot_source.cpp`'s `decideBootSource()` calls the new overload with `kChassis.wallSensors` — which changes
  the call text `test/drive_program_source_test.cpp` :163 pins (`"decideBootSourceFrom( record, registry, fromTable,
  missing, kMechanisms, kChassis.robot,"`): L6-B re-pins it to the new call (the pin's purpose — this build's tables
  reach the boot decision — unchanged). **No program file changes** (the programs call `decideBootSource(kTag)` —
  `competition_program.cpp` :648, `drive_program.cpp` :619, `auton_tester_program.cpp` :451) and the boot line comes
  from `Robot::describe()`.
- **Bench Tests station 11 gains a WALL page** (a second state on a page button, inside `distanceStation`'s own
  functions, `bench_r3a.cpp` :2487-2617, plus a NEW PROS-free `src/programs/bench_wall.hpp` holding its pure parts —
  never `bench_r3a.hpp`, `kMenu`, `menu_grid` or the station row, which MECH-A6 owns): for each declared side, the
  robot file's mount (or `MOUNT UNSET -- Step 0.1`), the reading minus the bias, **the computed distance from the
  robot's CENTRE LINE to the surface** (`reading − bias` projected through the mount — what Step 0.7 compares with the
  tape), the error bound at this reading, the update interval (a change timer like station 12's, HA-209), and the
  object size raw (Step 0.3's question). RECORD writes `[R3A] wall …` lines. Numbers, never a verdict needing a
  threshold the team has not typed.
- *(RC47)* **The Auton Tester shows the reset's outcome.** The everyday call (§1.12) records every outcome in the
  Robot's `WallSensors::log` (seated and refused counts, the last report). L6-B adds to `src/programs/auton_tester_run.hpp`'s
  `TesterRunRecord` the log's counts before and after the run and the last report, read through
  `chassis.deps().ctx->wallSensors()` (nullptr → `WALL n/a`); the RESULT page (`src/programs/auton_tester_panel.hpp`)
  gains ONE line and the SD summary (`shulib_auton_runs.txt`) the same words: `WALL 1 seated 0 refused | last: left wall
  x SEATED -2.45` / `WALL 0 seated 1 refused | last: left wall x REFUSED disagrees` / `WALL n/a`. Before it, a refusal
  reached only USB serial on the bench (guide 16 :241-251: "Nothing is written to the card during a run"; only the
  competition program copies log lines to `comp_log.txt`, guide 17 :295-302).
- `docs/internal/chunks/R3a-BENCH-WORKSHEET.md`: Station H gains **H.4–H.8** (Step 0.3–0.7, §2.0) — H.4–H.6 at the
  plan commit (no code), H.7 with B.
- Tests: `test/chassis_table_test.cpp` (robot two's wall block: ports 4/5, mount unset; the other tables declare
  none), `test/boot_source_test.cpp` (*RC27*: through the new overload the block reaches `out.built.config.sensors.walls`;
  a wall port that is a drive port, a claw's port or the other side's refuses the boot with the fix named; the old
  signature boots with no walls), `test/drive_program_source_test.cpp` (the :163 pin re-pinned),
  `test/bench_wall_test.cpp` (the page's pure parts).

### 1.11 L6-C — the Routine step — OPTIONAL, NOT recommended (D4 (b), *RC48*); if taken, after MECH-A4 merges
- `chassis/routine.hpp` (F10, additive): `Routine& resetFromWall(const localization::WallResetRequest&)` (skipped
  when stopped; SEATED and non-required REFUSED continue; a required REFUSED stops the chain as an APPENDED
  `RoutineStopCause::WallResetRefused` with the verdict word as the stop line's detail) and `const
  localization::WallResetReport& lastWallReset() const noexcept` (a "no reset yet" report before the first);
  `test/routine_signature_pin_test.cpp` appends its pins (the enumerator's value pinned after MECH-A4's
  `MechanismStepFailed`); the two exhaustive switches on `RoutineStopCause` in `src/programs/competition_run.hpp` and
  `auton_tester_run.hpp` gain the word (MECH-A4 adds one the same way — MECH brief §1.9 — and
  `test/competition_run_test.cpp` gains it); `version.hpp` + 1; the changelog; the guide and cookbook switch their
  listings from `then()` to the step (the `then()` form stays documented as the general idiom).

### 1.12 `include/shulib/competition/wall_reset.hpp` — NEW: THE EVERYDAY CALL over the auton's context (L6-A2, *RC48*)
- `[[nodiscard]] localization::WallResetReport competition::resetFromWall(AutonContext& ctx, const
  localization::WallResetRequest& request)` — the form every team auton uses (D4 (a)). It sits in `competition/`
  because its argument does (`AutonContext`, `competition/auton_registry.hpp` :215-229); it includes the explicit call
  (§1.2) and `auton_registry.hpp`. In order:
  1. **the start** — `ctx.entry.effectiveStart().mode() != StartPose::Mode::Field` → `NotFieldStart`, before anything
     is read (the season's walls and every keep-out are F1 coordinates; under a RELATIVE start the robot's place
     relative to every field element is unknown, so no keep-out could be trusted — for an `Own` wall too);
  2. **the sensor** — `ctx.chassis.deps().ctx->wallSensors()` (nullptr → `NoSensor`; the shadow context carries it,
     §1.6), the named side (absent → `SensorAbsent` with the Robot's sentence), its mount and the robot's limits;
  3. **everything else exactly as the explicit form** (§1.2's rules 1–9), EXCEPT the wait: it samples through
     `ctx.guard.waitFor(pred, windowMax + two ticks)` — the guard's deadline-aware wait (`sequence/run_guard.hpp`
     :95-109, :437): `RunExpired` → `Cut`, the Localizer untouched — so landmine 5's lateness does not apply to it;
  4. the outcome recorded in the slot's `WallResetLog` (RC47), and ONE `[WALL]` line through `ctx.telemetry`.
- Nothing in `AutonContext`, `Routine` or `Chassis` changes; MECH-A4's `AutonContext::mechanismService` (its brief §1.9)
  is an unrelated member it never reads.

---

## 2. Step 0, the runs, their tests and mutations, and the definition of done

### 2.0 STEP 0 at the robot — what a student measures (0.1–0.6 need NO new code: run them at the next bench session)
Robot two, "Bench Tests" (slot 3) for the readings. A tape measure, a carpenter's square, a flat board ~2 ft × 1 ft
(or a field wall panel), one Pin, a pen, the SD card in. Every reading is RECORDed (Station H's RECORD line carries
raw mm, confidence, object size, velocity and the adapter's inches). Results go into the worksheet's Station H.4–H.8
(added at the plan commit) and the coordinator types them; nobody "corrects" a number to match a belief.
- **0.1 The mounts (HA-210; D6).** For EACH side sensor: (a) **the robot's centre** = midway between the left and
  right wheels' contact lines, and midway along the wheelbase — the point the auton's start poses name (say this to
  the team: start poses and the mount must use the SAME point); (b) **forward** and **left** from that point to the
  centre of the sensor's front face (± how well, e.g. ±0.25 in), **height** of the lens centre above the floor; (c) the
  **facing**: a carpenter's square against the robot's side and the sensor's face — square, or by how many degrees
  not; (d) who measured, the date; *(RC39)* (e) how far the sensor's face sits INBOARD of the robot's outermost part on
  that side (under 2.5 in: the guide says never park flush to reset — the reading would fall under `rangeMin`). Also:
  is anything of the robot's own (an arm, a claw, at ANY position) within a
  ±18° cone in front of the sensor? A photo of each sensor from the side.
- **0.2 They are seen** (Station H.1, no change): the census lists DISTANCE on 4 and 5; station 11 shows both.
- **0.3 Against the tape (HA-204, HA-209).** The board square to ONE sensor, its distance measured from the sensor's
  FRONT FACE (Station H.0.3): **flush (the board against the face), 1, 2, 3, 4, 6, 8, 10, 12, 16, 20, 24 in** (*RC39*: the flush and 1-in readings
show what the sensor does inside `rangeMin` — they settle the 2.0 in) — five RECORDs at each, robot and board still.
  This settles the accuracy and any constant BIAS (D6), the spread of a still reading (the 0.5-in spread gate), and
  where the reading's zero is (the face? the lens? HA-204). At 8 and 16 in also RECORD with a **Pin** held where the
  board was (object size: wall-sized vs Pin-sized — §0.3's open question). *(RC35)* **The code strip:** with the board
  faced by a strip's BLACK square, then a WHITE one (a printed full-scale sample if the team's field has no strip —
  the squares as A8 draws them), at the mount height, RECORD at 4, 8 and 12 in: reading, confidence, object size — a
  range difference between the two is a per-colour bias the reset cannot see (HA-206).
- **0.4 The side check (the defence against S7b).** The board at 8 in on the LEFT only: port 4 must read ~8 in and
  **port 5's reading must NOT change when the board moves** (*RC44, corrected: "port 5 far" is ambiguous at a bench —
  port 5 may see a room object well inside 2 m*); then the RIGHT only. A swap means the wiring or the table is wrong — STOP and report.
- **0.5 Angles (HA-205; the 5° gate).** The board at 8 and at 12 in, turned **0, 5, 10, 15, 20°** about a vertical
  line through the beam's spot — RECORD each. Then the FOOTPRINT: slide a Pin in from the side along the board,
  and up from the floor, and note where the reading starts to change (the cone's real half-angle at that range) *(RC43)*
  **and HOW it changes**: a jump straight to the Pin's range (the sensor reports the nearest surface) or a gradual slide
  between the board's and the Pin's (a blend) — RECORD at several Pin positions so the coordinator can tell (HA-205).
- **0.6 A real field wall (HA-206, HA-207, HA-208), if one is reachable.** On whatever perimeter the team can reach
  (say WHICH: the metal one with clear panels, or the portable one): the sensor at its mount height, square to the
  wall at 4, 8, 12 in — mid-panel, at a panel seam, beneath a Toggle (*RC41*: at mid-Toggle AND at each bracket end,
  |along| ≈ 11–13 in — and measure how far the brackets reach into the field), beside a Loader; on the clear panel especially
  (does it read the panel, something behind it, or 9999?); *(RC35)* **on the code strip** where the field carries one
  (VURC and skills fields must — v20.txt :3454, :2061), the beam centred on a black square and on a white one;
  *(RC43)* at the alliance walls, once with a person standing just behind the wall while the reading is taken (the
  cone's top passes over the wall top at most mount heights — does the reading move?). Measure the wall-top height and the Toggle's overhang past
  the wall's inner face. Measure the field across, wall face to wall face, at three places (HA-207: 140.41 in?).
  **If the event's field type is known, say which.**
- **0.7 End to end (after L6-B, the WALL page).** The robot parallel to the board, the board at a taped distance from
  the robot's CENTRE LINE (not the sensor): 12, 15, 18 in per side — the page's computed distance against the tape.
  The update interval the page shows (HA-209).
- **0.8 On a field (after L6-B; autons the STUDENTS write — G4).** *(RC44, rewritten: each drill now states its
  expected verdict, or says it is a KNOWN LIMIT, and how it is run safely.)* **For every drill:** the path clear, one
  student holding the controller ready to DISABLE for the whole run, and each drill's auton ENDING right after its
  reset line (its end action a `brake()`, no leg after the reset) — so a wrong seat can never steer the robot anywhere.
  **Capture:** run it through the competition program's Timed Run (log lines are copied to `comp_log.txt` on the card —
  guide 17) or with a serial cable; from L6-B on, the Auton Tester's RESULT page and its SD summary also carry the
  reset's word (RC47). The drills:
  1. **the main one** — a FIELD start; a leg nose-along the left wall to a reading of ~7 in; square up; reset x →
     expected `SEATED`; then a second auton, the same plus one known leg after the reset, the robot taped where it ended
     → expected within the reset's error bound of the tape;
  2. parked crooked (≥ 6° by the estimate) → expected `REFUSED not square`;
  3. a start pose typed **6 in TOWARD the wall** from where the robot really stands (so any later leg would steer it
     AWAY from the wall, never into it), parked so the TRUE reading is ~7 in, `maxCorrection` 4 → expected `REFUSED
     disagrees` (a true reading past the beam-height ceiling would refuse `beam low` first — rule 7 runs before rule 9);
  4. a Pin standing against the wall in the beam → with the reading under the beam-height ceiling expected `SEATED`
     within the bound (the beam passes over a single piece — RC34), past it `REFUSED beam low` — RECORD which;
  5. **KNOWN LIMIT — may SEAT wrong by up to ~3 in (landmine 1):** a robot, or a board ≥ 12 in tall, standing 3 in from
     our side in the beam — RECORD the line; this is the drill that shows why `maxCorrection` must be tight;
  6. beside a Loader, inside a Load Zone, at a Toggle bracket → expected `REFUSED keep-out`, naming the span.
  Every `[WALL]` line captured with the tape's numbers.

### 2.1 The runs — four, each its own go, each verified and committed on its own (D1)
| Run | What (§1) | Files it writes | Waits on | Size (builder-days) | Public docs |
|---|---|---|---|---|---|
| **L6-A1 — the library core** | §1.1–§1.3, §1.7–§1.9: the season's walls; the geometry, gates, verdict and the EXPLICIT `resetFromWall`; `Localizer::seatAxis`; the host test plant's field and side sensors; D11's recipe fix | NEW `localization/{axis,override_walls,wall_reset}.hpp`, `chassis/wall_reset_call.hpp` (*RC24*), `sim/field_plant.hpp`, 5 test files; CHANGED `localization/localizer.hpp`, `hal/distance.hpp` (ONE comment sentence — §3.4, checked comment-only in §7), `test/cookbook_examples_test.cpp` — **no `version.hpp`** (§1.8) | the team lead's go | **~2¾–3½** (*RC29: +¾–1 — the beam-height gate and piece zones RC34/RC36, the drift term and the Localizer's two counters RC33/RC38, the accuracy and heading model RC38, the Toggle split RC41, the verdict table RC46, the plant's clusters / strip / brackets / see-through cases*) | (*re-allocated 2026-10-22, pace rule 7*) changelog; guide 10 (the call, request, report, verdict words, `seatAxis`); guide 11 (the `[WALL]` line); guide 14 (what it cannot catch; no heading; no passive mode; clear panels; the frozen wait's lateness); cookbook 05 (D11's fix ONLY); the HA register (HA-204..HA-212) — and the generated `docs/api/` with its nav, and `test/README.md` (the host field plant, for test authors: `sim/` is outside the reference by `api_doc_tool.py`'s rule, :95-103) |
| **L6-A2 — the robot's side sensors** | §1.4–§1.6 and §1.12 (the everyday call over the auton's context — *RC48*) | NEW `config/wall_sensor_config.hpp`, NEW `competition/wall_reset.hpp`, 3 test files; CHANGED `config/robot_config.hpp`, `robot/robot.hpp`, `chassis/robot_context.hpp`, **`motion/motion_scheduler.hpp` (the shadow context's one carried line — §1.6, found 2026-10-22)**, (*RC48: no `localization/wall_reset.hpp` overload — the everyday form is §1.12's*), the boot-line pins (§1.5: `boot_source_test`, `xdrive_config_test`, VIS1-A1's `robot_camera_test` — *RC26*) — **no `version.hpp`** (§1.8) | **VIS1-A1 merged** (the same two config/robot files and the pins) and L6-A1 merged | **~1¾–2** (*+¼ on 2026-10-22: the shadow context's line, its end-to-end test and M31; the pins. RC29: +½ — the everyday call over the auton's context, RC48; the outcome log, RC47*) | changelog; **guide 03 (the NEW section "Re-seating one axis against a wall", moved here from A1 — the everyday call a team writes)**; guide 10 (the robot file's block, the accessors, the boot line's `walls` words); guide 15 (glossary: wall reset, seat, keep-out, side sensor, footprint); cookbook 05 (a NEW recipe "Re-seat x against a wall", the everyday form through `then()`, compiled) — and `docs/api/` |
| **L6-B — robot two** | §1.10: the table's block, the boot copy, station 11's WALL page, worksheet H.7 | `src/robots/chassis_table.hpp`, `src/common/boot_source.{hpp,cpp}` (the new `decideBootSourceFrom` overload — *RC27*), `src/programs/bench_r3a.cpp` (station 11's functions only), NEW `src/programs/bench_wall.hpp`; *(RC47)* `src/programs/{auton_tester_run,auton_tester_panel}.hpp` (the record's wall counts, the RESULT line, the SD summary words); `test/{chassis_table,boot_source,drive_program_source,auton_tester_run,auton_tester_panel}_test.cpp`, NEW `test/bench_wall_test.cpp` | L6-A2 merged; **MECH-A5 not launched** (else after A5 merges and before A6 launches) | **~1½–2** (*RC29: +½ — the boot overload and its named refusal, RC27; the tester's wall line, RC47*) | guide 16 or 17 (the boot line's `walls` words); worksheet H.4–H.8; changelog |
| **L6-C — the Routine step** | §1.11 | `chassis/routine.hpp`, `test/routine_signature_pin_test.cpp`, `src/programs/{competition_run,auton_tester_run}.hpp` (one `case` each), `test/competition_run_test.cpp`, `version.hpp` | MECH-A4 merged | **~½–¾** (OPTIONAL — not recommended, D4 / RC48) | changelog; guide 09/10; cookbook 05 |
| **L6: A1 + A2 + B** (C optional) | | | | **~6–7½** (*RC29*; verification ~½–1 per run on top: ~7½–10½ slot-days) | |

Sized against comparable runs: VIS1-A1 (a device behind the provider, a robot-file block, geometry with error bounds, a
host model) is ~2½–3½ (VIS1 brief §9); MECH-A2 (a service, a plant, the guard's cut) ~2¾–3¾; L6-A1+A2 have less device
work than VIS1-A1 (no adapter, no shim: MECH-A1 built the accessor) and add the seat and the gate ladder.
REMAINING-WORK row 25's "~1 day (rough)" predates the side-sensor ruling, the read-fault rule, the keep-outs and the
host field model.

**What an auton can do after A1 + A2 + B (and Step 0.1's mount typed):** *(RC45)* in a FIELD-start auton, `face` and
drive a leg nose-along a wall, square up with `turnTo`, parked so the reading is ~6–8 in — under the beam-height
ceiling for the sensor's height — with the beam centre inside that wall's clean band (the guide's table, Q1b); call
`competition::resetFromWall(ctx, …)` through `then()`, and continue with that coordinate
corrected to within its error bound — ≤ 1.0 in by the gate: ~0.9–1.0 in with VEX's published accuracy and a ±0.25-in
mount, ~0.5 in once Step 0.1 and 0.3 measure them (Q1 (A); *RC38*, the draft said "~0.7–0.9 in (P1f)") — or, refused, continue exactly as it would have without L6 (or
stop, if it said `required`). **After A1 alone** a hand-wired host test can already do it with explicit handles; the
robot cannot (nothing builds its sensors until A2, nothing declares robot two's until B). **Before 2026-11-23**
*(RC29, re-done: the draft's "~2026-11-01 under D13 (a)" was the parallel-slot estimate presented as the one-slot
one)*: A1 ~2¾–3½, A2 ~1¾–2 (after VIS1-A1 merges — VIS1-A1 resumed 2026-10-22 08:44, its mutations M1–M9 red, M10
and its documents owed), B ~1½–2, each with ~½–1 of verification — **L6-B ~Oct 30 – Nov 3 with a second slot (D13
(a)); ~Nov 5–10 with one slot and L6 after MECH-A4 (D13 (b)); ~Nov 12–22 with MECH first (D13 (c))**. The freeze is
~Nov 16. So **Step 0.1–0.5 (no code) should run at the NEXT bench session**, before any L6 code lands: the mount's
height decides whether robot two can use the reset at all (the beam-height ceiling, RC34), and the measured accuracy
and mount tolerance whether it passes the precision gate after any turning (RC38).

### 2.2 Tests (ADVERSARIAL — each tries to break ONE rule) and the mutations each must turn red
Every mutation is RUN and seen RED, then restored (RESUMING's standard); the executor logs each in `L6-PROGRESS.md`.
Tests on the host test plant use `sim/field_plant.hpp` — never the library's own geometry — as the oracle.

**L6-A1** (T1–T11, T14–T16 — the explicit call) **and L6-A2** (T12, T13, T17, and T8's S1 repeated through the everyday call on a `robot::Robot` built over the fake provider)
| # | Test (the rule it tries to break) | Mutations it must turn RED |
|---|---|---|
| T1 | **The inversion is exact**: for every wall × every side × forward offset {0, 5} × skew {0, ±4.9°} × bias {0, 0.3}, with a NOISELESS ray-centre sensor on the field plant, `axisFromReading` returns the true coordinate within 1e-9 in | M1 a wall's `inward` sign ignored; M2 the range used without the cos (beam off normal); M3 the lever arm rotated by −heading; M4 the bias added, not subtracted |
| T2 | **The error bound bounds**: 20 000 seeded draws inside the stated tolerances (heading, mount position and facing, the published accuracy at the reading) — the true error NEVER exceeds `axisErrorBound`; and the bound is not loose (worst draw ≥ 0.5 × bound) | M5 the mount-position term dropped; *(RC38)* M6 the accuracy model replaced by the 200-mm STEP (T2 asserts the bound is continuous in the reading and never below max(15 mm, 5 %) — the step turns it red at 7.87–11.8 in); M6b a measured `rangeAccuracy` ignored; M6c the heading tolerance held at the floor (the turn counter ignored — a robot that turned 450° seats as if it had not) |
| T3 | **Each gate refuses its own case, and only it** — for every verdict: a case just inside passes and just outside refuses with exactly that verdict (square 4.9° / 5.1°; keep-out: a beam centre 1 in outside a Loader whose FOOTPRINT overlaps → KeepOut; speed 0.99 / 1.01 in/s; confidence 0.50 / 0.49; range 2.0 / 1.99, 16.0 / 16.01 after bias; 5 / 4 samples; spread 0.50 / 0.51; bound 1.00 / 1.01; correction = limit / limit + 0.001; `maxCorrection` 0 → BadRequest naming it) | M7 the square gate's comparison flipped; M8 the footprint widening dropped; M9 the speed read once at the start of the window, not every tick (a push mid-window); M10 a 9999 sample skipped instead of refusing; M11 the spread gate removed; M12 the disagreement compared on the OTHER axis (a robot whose y is far off); M13 an UNSET `maxCorrection` taken as infinite; *(RC34)* — and the beam height: an expected range at the ceiling / +0.01 in → `BeamLow` before any sample; an expected range under the ceiling with a MEDIAN just past it → `BeamLow`; a measured cone used when typed, the limits' 18° when not; a `PieceZone` span refusing with an unmeasured cone and not with a measured one — M36 the beam-height gate removed; M37 the median's ceiling check removed (the estimate-nearer case lets a piece through); M38 the piece zones' rule inverted |
| T4 | **A refused aim costs no time and reads nothing**: for NotSquare / KeepOut / MountUnset / NoSensor / *(RC2)* **MotionActive** (a hold armed through `chassis.scheduler().async()`, the robot still) the clock has not advanced and the fake distance was never read (a counting fake); for MotionActive the Localizer is bit-identical and the armed hold is still armed (the reset cancels nothing) | M14 the pre-check run after sampling; *(RC2)* M32 the motion check dropped (the hold's case samples, seats, and the hold drives the robot to cancel the jump) |
| T5 | **The read-fault rule**: the count moves between `distance()` and `confidence()` of the LAST sample → ReadFault; moves before the window and not during → passes; `faults == nullptr` → passes and the line says `faults n/a` | M15 the count read once per window; M16 compared only around `distance()` |
| T6 | **The seat moves one axis and nothing else**: after `seatAxis(X, v)` — x == v exactly; y, heading, heading bias, `distanceSinceCorrection`, `lastCorrection` bit-identical, and the quality class and scalar bit-identical AT THE INSTANT `seatAxis` returns — *RC30*: the next tick is the dt-0 re-base tick (Degraded in both seats: `seatAxis` 0.100, `setPose` 0.500) and from the second tick the one-axis seat is back at Degraded 0.200 while `setPose` reads DeadReckon 1.000 — T6 pins those per-tick values; the next tick's twist has no spike; a following 10-in drive moves the estimate by the odometry's 10 in only; seating Y likewise; the odometry's own pose bit-identical across the seat. *(RC1)* **Run under BOTH fusion tiers** — `ComplementaryFusion` (the Robot's, `robot.hpp` :1164) and `EkfFusion`: only the EKF tier keeps its own position state, so only it sees M19. **A REFUSED reset leaves the Localizer bit-identical** (pose, bias, drift term, the odometry's pose) | M17 the seat clears the drift term (setPose's behaviour); M19 `hasLast_` left true — **RED only under `EkfFusion`** (RC1's R1: the filter reads the seat as one tick's travel, 1.68 in of a 2-in seat bounces back, a 167.6 in/s velocity); *(RC1: the draft's M18, "the odometry not shifted", is DELETED — the shift is dropped, and R1 showed it was an equivalent mutant)*; M18b the seat calls `odom_.setPose` (the dropped shift restored — the odometry's pose changes: the bit-identical check above) |
| T7 | **seatAxis's preconditions**: not live → `PreconditionError` naming "not live yet"; non-finite → error | M20 the live check removed |
| T8 | **End to end on the host test plant — P2's scenarios as tests** (tank `ChassisRig`, a FIELD start, the independent field model, its seeded noise — *RC40:* an F1-frame field, the Loaders on the red/blue walls; S4a/S4b as R3's S4a'/S4b', a y reset against the RED wall): S1 SEATED within 0.1 in of truth; S2 Disagrees and untouched; S3 NotSquare; S4a KeepOut; S5b (a lying CUP, relabelled — RC42; plus the same with a lying PIN, 1.6 in) / S5c SEATED within 0.1; S6 ReadFault; S7a NoReturn; S8 NoReturn; *(RC37)* **KNOWN LIMIT**: a see-through red wall with an opaque object 1 / 2 in behind its face SEATS ~1.1 / ~2.0 in wrong with `maxCorrection` 3 (Q2's D rows), 3 and 4 in → `Disagrees`; S9 Moving; S10a KeepOut; S11b within 0.05; S12 off by the typed error (pinned 0.5 ± 0.05). **The KNOWN LIMITS pinned as such** (named "known limit — an object at a plausible distance", so a change either way is seen): S4b with keep-outs OFF seats ~3.7 in wrong; S5d (sensor 4 in up, a standing Pin) now refuses `BeamLow` (*RC34* — no longer a known limit); *(RC34)* Q2's start-cluster cases on the F1 field: a lens 8 or 9 in up with the wall past its ceiling → `BeamLow` (the draft SEATED 1.2–2.9 in wrong), 10.5 and 11 in up within it → SEATED within 0.1 in, a REAL cone of 24° under a typed 18° with the cone UNMEASURED → `KeepOut` (the piece zone), a 3- or 4-in lens over the floor → `BeamLow`; *(RC41)* with `toggleChecked` the BODY's span lifts and the brackets' does not — a beam at a bracket (along 11.9) → `KeepOut` (Q2's T rows: with the whole keep-out lifted, SEATED 0.96 in wrong), a beam at mid-Toggle with the flag → SEATED; *(RC36)* a beam centre inside a Load Zone → `KeepOut` with the cone measured or not (Q2's LZ case: a Cup standing in front of the red wall at x 50); S7b (backwards, a robot 5 in away) seats ~3.0 in wrong. *(RC31)* **A SEATED correction over 2.25 in followed by a Routine step raises NO `IMPLAUSIBLE`** (the scheduler's pose guard, `diag/plausibility_guard.hpp` 150 in/s × 1.5 × 10 ms = 2.25 in, `motion_scheduler.hpp` :1106-1111, forgives a jump only on a dt-0 tick, which every verb and wait gives — :761, :798, :830; COMP-a's `checkNoSeatFaults`, `competition_runner_test.cpp` :214-218, is the model); the Tier-3 case — the same seat followed by a caller-paced `scheduler().tick()` — raises exactly ONE advisory `IMPLAUSIBLE`, pinned as documented | M8, M12, M17 again; *(RC41)* M39 `toggleChecked` lifting the brackets too (the bracket case SEATS wrong); M21 the median replaced by the mean (a test with one in-spread outlier: the reported `median` must be the median) |
| T9 | **The call's contract**: ONE log line per call — and *(RC31)* no `SCH` "waitUntil timed out" line: the sampling predicate ENDS the wait itself (at `windowMax` by the clock at the latest), so the frozen wait never reaches its own timeout (:838-844) — ASCII, ≤ 384 bytes, Info/Warn by verdict, subsystem `WALL`, carrying wall, axis, side, verdict and its numbers (a recording sink); the drive is NEVER commanded (the fake motors' last command unchanged across a call); the call returns within `windowMax` + one tick even when every sample is 9999 | M22 a second line per sample; M23 the window bound ignored (a sensor stuck at 9999 waits for the wait's whole timeout) |
| T10 | **`then()` and D3**: SEATED → `continues()`; REFUSED + not required → `continues()` and the chain goes on; REFUSED + required → the chain stops `ActionFailed`, the drive safed by the Routine (its policy) | M24 `continues()` ignoring `required` |
| T11 | **The explicit form's own wall** (*RC48*: frame `Own`): `FieldWall::line` in a frame rotated 90° from F1 gives the same seat as the field preset in F1; its keep-outs, when given, apply as given; with none, none | M1 again |
| T12 | **The robot file**: `valid()` refuses — a wall port that is a drive port, a mechanism sensor's port, or the other side's port (naming both claimants: "port 4 is claimed by the LEFT drive and by a wall sensor"); a port of 22; each mount and limit bound of §1.4 just outside; an UNMEASURED mount is VALID | M25 the claim removed; M26 an unmeasured mount refused at boot |
| T13 | **The Robot**: both sides built through the fake provider; a refused port leaves that side absent with the provider's sentence (copied — it survives a later refusal), the robot still builds and drives; the boot line's `walls` words for every case (both, one absent, mount unset, none); the context slot non-null and a context built without it still constructs; **the slot reaches `robot.chassis().deps().ctx`** (the scheduler's SHADOW context — §1.6) and the everyday call SEATS through the Robot's own `Chassis` on the host test plant | M27 a refused sensor refuses the robot; M28 the absence pointer kept, not copied (a later refusal rewrites it); **M31 the shadow context's `.walls` line dropped (`motion_scheduler.hpp` — every everyday call refuses NO SENSOR)** |
| T14 | **The field plant itself** (`sim_field_plant_test.cpp`): its ray-cast against closed forms (a square wall at 8 in reads 8; a 10° wall reads 8/cos 10° on the centre ray); the cone picks the nearest (and, in `blend` mode, the hits' mean — *RC43*); see-through boxes are passed; the update hold and the latency delay; seeded determinism; *(RC35)* the code-strip band (9.06–11.44 in) applies its bias only to rays that hit it; its Override walls agree with `override_walls.hpp` to 0.01 in | — (the plant's own tests; a mutation of the plant must turn T1/T8 red — M29 the plant's wall at 70.0 instead of 70.205 → T14's agreement case) |
| T15 | **The cookbook recipe (D11)**: the re-seed keeps y for a robot whose y is NOT 0 | M30 the recipe's `0_in` restored |
| T16 | *(RC33, RC38)* **The along uncertainty and the Localizer's counters**: `travelSinceSeat(X/Y)` grows by the path length, `setPose` zeroes both, `seatAxis(X)` zeroes X only, a corrector's fix zeroes neither; `turnedSinceHeadingSeed` sums \|Δθ\| through a ±π wrap (a 350° spin reads 350°, not 10°), zeroed by `setPose` and `reseatHeading`; no existing observable changes (every Localizer test green). **On the host test plant (F1 field, Q2's case A):** a beam beside a Loader whose ESTIMATE is 4–5 in off along the wall — the draft's widening SEATED it 1.57 in wrong; after 24 or 48 in of travel the drift term refuses `KeepOut`; the same case beside a Toggle body; and a reset right after a perpendicular wall's seat (travel ≈ 0) is NOT over-refused | M33 the drift term dropped (the Loader case SEATS wrong); M34 `seatAxis` zeroing BOTH counters (a reset against the left wall stops the red wall's keep-outs from widening); M35 the turn counter summing signed Δθ (a left-right wiggle reads 0) |
| T17 | *(RC48)* **The everyday call over the auton's context** (L6-A2): a RELATIVE start and an UNSET one (a route whose Start states no heading) → `NotFieldStart` with the clock unmoved and nothing read; a FIELD start SEATS (S1 through a real `Robot`'s `Chassis` and a running `RunGuard`); a deadline landing mid-window → `Cut` at once (zero lateness — the guard's `waitFor`), the Localizer bit-identical; every outcome counted in the slot's log | M40 the start check dropped (a RELATIVE auton SEATS a field number in its own frame); M41 sampling through the frozen `chassis.waitUntil` (the mid-window deadline is late by up to `windowMax`) |

**L6-B**: `chassis_table_test` (robot two declares 4/5, mount unset; the bench bot and X-drive declare none);
`boot_source_test` (the block reaches `out.built.config.sensors.walls` through the new overload; a table whose wall port
is a drive port is refused by a line naming the fix — *RC27*);
`bench_wall_test` (the page's centre-line distance against hand-computed cases; the change timer). Mutations: the copy
dropping the right side (MB1); the page's distance missing the left offset (MB2); the bias ignored on the page (MB3).
*(RC47)* `auton_tester_run_test` / `auton_tester_panel_test`: a run whose auton seats once → `WALL 1 seated 0 refused |
last: … SEATED …` on the RESULT page and in the SD summary line; a refused one → its word; a robot with no slot →
`WALL n/a`; the counts are the run's own (a second run starts from the log's new totals). Mutation MB4: the log read only
before the run (a reset during it never shows).

**L6-C** (*only if taken — RC48*): the F10 pins appended and compiled; `resetFromWall` step: skipped when stopped (counted), SEATED continues,
non-required REFUSED continues, required REFUSED stops as `WallResetRefused` with the verdict word in the stop line;
`lastWallReset()` before any call reads "no reset yet". Mutations: the step not skipped when stopped (MC1); a
non-required refusal stopping the chain (MC2); the enumerator inserted rather than appended (MC3 — the pin).

### 2.3 Definition of done — every run (the MECH core's §2.1, adapted)
- [ ] `L6-PROGRESS.md` created FIRST and appended as the run goes (a heading per run); nothing committed.
- [ ] Every new public member documented (the API doc gate); every number that is a belief carries `A4 register HA-nn`
      (HA-204..HA-212 only).
- [ ] The host suite green under `(ulimit -v 4194304; timeout 300 …)`; every listed mutation RUN and seen RED, logged
      with the failing test's name, then restored.
- [ ] The ARM header gate clean; the CI PROS-free and sim-layering guards PASS; `python3 tools/src_build_gate.py check`
      (all seven builds, zero disallowed warnings — L6-A changes headers `src/` includes) and, for B and C,
      `self-test`.
- [ ] The doc gates: `api_doc_tool.py` check-fresh / coverage / examples / removability; `briefing_status.py check`
      (regenerated, never by hand, never against a stale build); `doc_staleness_audit.py`.
- [ ] §3's documents changed IN THE SAME CHANGE, and the prose review logged (§3.3).
- [ ] The roadmap and REMAINING-WORK under-claim: `[~]` for L6 until a robot has seated a wall (Step 0.8); "host-
      and build-gated, NOT run on a robot" in every status line.

---

## 3. Documents — in the same change as the code (pace rule 3), the prose review logged (rule 4), ≤ ~5 public each (rule 7)

### 3.1 Per run (public)
*Re-allocated 2026-10-22 (pace rule 7: the draft gave L6-A1 ten public documents under this heading's "≤ ~5"). The
changelog and the HA register count; the generated `docs/api/` and its nav do not (a tool writes them).*
- **L6-A1 (six — the changelog, four guide/cookbook pages, the register):** `docs/changelog.md` ("Added since 2.x
  (no version bump — §1.8): the wall reset's library — `chassis::resetFromWall` (explicit handles),
  `Localizer::seatAxis`, the Override walls; Fixed: cookbook 05's re-seed recipe kept y"); **guide 10** (the API: the
  call, the request, the report and its verdict words, `seatAxis` and why it is not `setPose`); **guide 11** (the
  `[WALL]` line, word by word, both forms, and *(RC46)* §1.2's verdict table verbatim — word, numbers, what to do) — *and (RC31) beside the explicit call in guide 10: a Tier-3 loop that ticks
the scheduler itself sees one advisory `IMPLAUSIBLE` after a seat over ~2.25 in*; **guide 14** (what it CANNOT catch — an object at a plausible distance,
  with P2's three examples; no heading correction; no passive mode; the clear-panel risk — it REFUSES usually, but SEATS wrong by up to `maxCorrection` when something
opaque stands close behind the panel (*RC37*) — and the code-strip risk (*RC35*); VEX U loads during the auton, so the Load Zones are
always kept out (*RC36*); the frozen wait's lateness ≤
  `windowMax`); **cookbook 05** (D11's fix of "Re-seed the estimate mid-routine" ONLY: keep y, `startAt` clears the
  drift term for both axes, a pointer to guide 10's wall reset — and the paragraph's "the simulator's ground truth"
  becomes "the host test plant's ground truth", since the paragraph is rewritten anyway); the HA register (§4).
  Beside them, not counted: `docs/api/` regenerated (`override_walls`, `wall_reset` pages + the nav the tool writes)
  and `test/README.md` (the host field plant and its side sensors, for test authors — `api_doc_tool.py` :95-103 keeps
  `sim/` out of the reference and sends test authors there).
- **L6-A2 (five):** changelog; **guide 03** (`docs/guide/03-knowing-where-you-are.md`) — the NEW section "Re-seating
  one axis against a wall", moved here from A1 because this is the run that lets a team's robot use it: what a side
  sensor measures, the nine checks in plain words (*RC34*: the beam's height added) and the FIELD-start rule (*RC48*), what it cannot catch (pointing at guide 14), the mount advice
  (D10), clear wall panels (HA-206), the walls at ±70.2 (A10/A13/A14), the everyday call — *(RC45)* with the FIELD-start
  rule, the parking recipe (face, a leg along the wall, `turnTo` square, a reading of ~7 in under the ceiling) and the
  beam-centre bands per wall type and per distance driven (Q1b); **guide 10** (the robot
  file's `walls` block, `robot.wallSensor()` / `wallSensorAbsence()`, the boot line's `walls` words — where VIS1-A1
  documents the camera's); **guide 15** (glossary: wall reset, seat, keep-out, side sensor, footprint); **cookbook 05**
  (a NEW recipe "Re-seat x against a wall" — D4's sequence, the everyday form through `then()` — compiled by
  `test/cookbook_examples_test.cpp` and quoted under `check-examples`); `docs/api/` regenerated.
- **L6-B:** changelog; guide 16 / 17 (the boot line's `walls` words on robot two; the WALL page; the tester's wall line —
  RC47 — and, in guide 16's "What the host tests do with your new entry", *(RC47)* that an auton's wall reset refuses
  `NO SENSOR` on the host test plant's tester rig — it declares no wall sensors — and the auton continues unless
  `required`, so only the robot tests the reset); *(RC46)* **guide 12** (a symptom entry, "my wall reset keeps refusing": read the word, then §1.2's row, and how
  to choose `maxCorrection`); the worksheet's Station H.7 (the WALL page, from the code — every string copied).
- **L6-C (five; only if taken — RC48):** changelog (THE API bump — §1.8); guide 09 (the recipe API's new step) and 10; cookbook 05's
  recipe and guide 03's listing switched to `r.resetFromWall(…)` (the `then()` form stays documented as the general
  idiom).

### 3.2 Internal, every run
`docs/internal/chunks/L6-PROGRESS.md` (created FIRST); `build-order.md` (L6's entry and "Next"); `REMAINING-WORK.md`
(row 25's status and size); `docs/roadmap.md` (an L6 item in the M3 localization list beside "Calibration routines +
persistence", ~:1666 — `[~]` "host- and build-gated, NOT run on a robot" until Step 0.8 seats a real wall);
`PROJECT-BRIEFING.md` regenerated (`briefing_status.py generate`, against a FRESH build only); at the last run,
`L6-COMPLETED.md` and the pointer moves (RESUMING rule 5).

### 3.3 The plan commit (the coordinator's, on the team lead's go, BEFORE L6-A1)
This brief → `docs/internal/chunks/L6-wall-reset.md`; the HA register's Held line gains **HA-204..HA-212 (L6's)**;
`REMAINING-WORK.md` row 25 reworded (D12: "re-seat ONE axis against a wall the robot has parked beside, from the side
sensor facing it — never heading, never in the background") and re-sized (~6–7½, three runs + the optional L6-C — RC29);
`build-order.md` gains an "### L6" entry (the runs, their waits; *RC48:* and the first-competition plan's line for
student autons gains "a wall reset needs a FIELD-start auton — none exists yet, `autons.cpp` :66") **and L6 JOINS the phase table** *(RC32, corrected: the
draft said "the chunk count is unchanged (L6 is already a row)" — it is not: build-order names L6 only at :632, :1713,
:2224 and :2227, never in "The order at a glance")* — recommended as its own row ("SENS — the distance sensors before the
panel", the team lead's 2026-10-14 ruling), the count line (:854, "70 chunks") gaining "(L6 added <date>, +1 …)" → **71**,
and REMAINING-WORK :16's "70" with it; **`HANDOFF-remaining-work.md`'s L6 entry** (:489-500 at the tip: "drive slowly
toward a known wall until the distance sensor reads X … or as a runner built-in … Size ~1 day") is rewritten to a
pointer at this brief (D12's wording; VIS1's plan commit `71b4764` edited HANDOFF the same way); the worksheet's
Station H gains **H.4–H.6** (Step 0.3–0.6 — no code) and a pointer to Step 0.1's mount sheet; the D10 mounting advice
goes to the build team (the team lead's channel, not a file). **And (*moved here from L6-A1, 2026-10-22* — a change to
the plan is the plan commit's, pace rule 2):** `docs/shulib-v2-master-plan.md` :412 — the Distance row's "wall
cross-check" becomes "wall reset (L6): one axis against a field wall, from a side sensor"; and decision #4 (:547,
LOCKED 2026-06-19, titled "GPS/AprilTag aggressiveness") is **AMENDED — only if the team lead accepts D2 (a)** — by
D2's sentence: an EXCEPTION for a caller-invoked one-axis wall reset (no motion active, bounded by `maxCorrection`,
never the heading), the rest of the row — "`relocalize()` feeds this corrector — it does not snap" for every other
relocalize and every corrector — unchanged; the row's status cell records "amended <date> (L6, D2 (a)), the team
lead's words", as the Freeze Register records its amendments. *(RC25, corrected: the draft proposed a "scope note,
not a change" claiming the decision governs only per-tick correctors — its own relocalize sentence says otherwise.)*

### 3.4 The prose review's greps (each run runs them and logs the hits it read)
`never snap|never-snap|nothing that reads a sensor|caller seed|relocalize` (D2's wording — the amended decision #4,
*RC25* — `include/` + public docs);
`wall reset|re-seed|reseat|startAt` (the recipe and the guide); `distance sensor|side sensor|IDistance` (stale claims
that nothing uses the distance sensors for localization — e.g. `hal/distance.hpp` :7-9 "Used for manipulation
sensor-confirm and the docking distance-fallback" and :18-19 "Consumers are the manipulation sensor-confirm and the
docking distance-fallback": L6-A1 adds "and the wall reset (L6)" to those two COMMENT sentences and changes no other
byte of the F4-frozen header — §7's check proves the diff is comment lines only); `about −70|70 in|140` (the field's size); `HA-20[4-9]|HA-21[0-2]` (every code site has its row and
every row a code site — the register's reconciliation, both directions).

---

## 4. Hardware Assumptions — HA-204..HA-212 ONLY (the register's Held line gains them at the plan commit)
> **The ids COLLIDE with the IMU1 brief's HA-204..HA-206** (found 2026-10-22, §6.6). Recommended: L6 keeps
> HA-204..HA-212 and IMU1 takes HA-213..HA-215 (IMU1's own recommendation) — unless the team lead accepts IMU1 first,
> in which case every id below shifts by three (HA-207..HA-215), mechanically, at the plan commit (landmine 20).

Each entry in the register's form: *Claim · Source (`A4 register HA-nn` beside the code) · Confidence · Settle · Blast
radius if wrong*. **None is measured.** A tenth belief ⇒ stop and tell the coordinator.

| # | Belief | Confidence | Cited in code by | Settled by |
|---|---|---|---|---|
| **HA-204** | The V5 Distance Sensor reads a flat, wall-sized target to **±15 mm below 200 mm and ~5 % above**, over **20–2000 mm**, measured from its **front face** (the zero point is not published) — *RC38:* two approximations meeting at 200 mm, so the library assumes **max(15 mm, 5 % of the reading)** at every range until Step 0.3's measured `rangeAccuracy` replaces it | **reasoned** — VEX's published figures (api.vex.com, fetched 2026-10-18); the zero point and our sensors' real spread unknown | L6-A1 (`axisErrorBound`'s accuracy term; `rangeMin`'s 0.79-in floor) | Step 0.3 (ten distances × five RECORDs per side; the bias and the spread); Step 0.7 |
| **HA-205** | The beam is a cone of about **±12°** (long range) to **±18°** (short-range mode) — and within **5° of square** to a flat wall the reading is the beam-centre range to within HA-204 (P1e: the beam-centre vs perpendicular difference is ≤ 0.046 in at 12 in there) — *and (RC43)* **with two surfaces in the cone the sensor reports the NEAREST one** (the host plant's model, T8's known-limit magnitudes, rule 3's footprint widening and the harmlessness of the cone's top edge over the wall top all rest on this); VEX says object size is estimated from "how much of the sensor's view is taken up" (api.vex.com), so the sensor integrates over its view and may report a BLEND instead | **invented** — a community post seen only through search summaries; VEX publishes no angle and no multi-target behaviour | L6-A1 (`coneHalfAngle` widening the keep-outs AND setting the beam-height ceiling while the mount's cone is unmeasured — *RC34*; the 5° gate's reason; the plant's nearest-hit default and its `blend` mode — *RC43*) | Step 0.5 (angles 0–20° at 8 and 12 in; the Pin slid in from the side and from below — WHERE the reading starts to change and HOW: a jump to the Pin's range, or a gradual slide — *RC43*); Step 0.6 (a person standing just behind an alliance wall while the cone's top passes over it) |
| **HA-206** | The field wall's inner face returns a reading at the sensor's height on the perimeter the team will meet — **the portable perimeter's opaque walls AND the metal perimeter's clear panels** (*RC37 — if wrong on a clear panel: the reset usually REFUSES, but SEATS wrong by up to `maxCorrection` when something opaque stands within that distance behind the panel*) — *and (RC35)* **the VEX GPS code strip**, which VURC fields and Autonomous Coding Skills fields MUST carry (v20.txt :3454; `<RSC4>` a, :2061): two rows of black and white squares on the wall's face at **9.06–11.44 in** above the floor (A8 at 300 dpi, `check-field-safety/a8_wall_crop.png`, 48.1 px/in from the printed 13.20 in) — where every useful mount's beam lands (D10); a black square is believed to read the same range as a white one, within HA-204, with a usable confidence | **invented, with CONTRARY evidence** (the panels: a VEX Forum thread reports the sensor seeing through the metal field's clear panels or dropping out beyond ~700 mm — seen through search summaries only) · **invented** (the strip: no published behaviour on dark or patterned targets) | L6-A1 (the guide's warning; the host plant's see-through case and its code-strip band — *RC35*) | Step 0.6 on each perimeter type the team can reach, ON the strip where the field has one; Step 0.3's black/white squares (a printed full-scale sample if the team's field has no strip — a practice field may not); the event's field type |
| **HA-207** | The perimeter's inner faces are **±70.205 in** from the field's centre (A10: 3566.4 mm apart; A13's portable perimeter the same, 140.40 in; **A14's metal perimeter 140.50 in, ±70.25** — *added 2026-10-22*) and a real field is within **±0.5 in** of its drawing | **reasoned** (the drawings, which differ by 0.045 in per wall between the two perimeter types) · **invented** (the tolerance — the manual says only that fields "vary slightly", A3) | L6-A1 (`override_walls.hpp`) | Step 0.6: the field measured wall face to wall face at three places, and WHICH perimeter type it was |
| **HA-208** | Each Toggle's BODY sits ON TOP of its wall, overhanging the inner face by ≤ ~1 in — so a beam that stays below the wall top (**11.50 in metal, A14; 11.54 in portable, A13 — printed**) reads the wall beneath the Toggle's body; *(RC41, corrected: the draft said "beneath a Toggle")* **its two BRACKETS, at \|along\| ≈ 10.9–13.0 in (both ends), hang down to ≈ 10.6 in above the floor** (A8 at 300 dpi, `check-field-safety/a8_wall_crop.png`, 48.1 px/in — in front of the code strip, inside D10's mount band); how far they reach INTO the field is not drawn | **reasoned** — the wall heights are PRINTED (A13/A14; *corrected 2026-10-22: the draft said no dimension was printed*); the bracket's bottom measured off A8's printed scale · **invented** — the Toggle's overhang past the inner face (read by eye from A8/A10) and the brackets' depth: no dimension is printed | L6-A1 (`override_walls.hpp`'s Toggle BODY span — `Toggle` kind — and the two BRACKET spans — `Fixed`; the host plant's wall height and brackets) | Step 0.6 (the overhang and the brackets' depth measured; readings at the mount's height at mid-Toggle AND at each bracket end) → `SideMount::toggleChecked` (D8) lifts the BODY only |
| **HA-209** | The sensor's **update interval and latency are UNKNOWN**; PROS returns its latest value, so repeated reads within one update repeat it (N samples are fewer independent readings), and one `distance()` / `confidence()` pair can straddle an update (`hal/pros/distance.hpp` :75-76) | **invented** (the interval, the latency) · **reasoned** (the two reads, from the adapter's source) | L6-A1 (`minSamples` with `window`; the per-sample fault rule) | Step 0.7 (the WALL page's change timer, a board moved slowly) |
| **HA-210** | Robot two's side sensors are on **port 4 (left) and port 5 (right)**, facing straight out, with nothing of the robot's own in the cone at any mechanism position; their mounts are **UNMEASURED** — the HEIGHT included, which decides whether the beam-height gate lets the reset work on robot two at all (*RC34*) | **reported** (the census of 2026-09-24; the team lead's facing, 2026-10-08) | L6-B (`src/robots/chassis_table.hpp`'s provenance) | Step 0.1 (the mounts, the cone clear), 0.2, 0.4 (the side check) |
| **HA-211** | The gate defaults: range **2–16 in** (*RC39, the derivation the draft omitted:* 2 in = VEX's 20-mm floor, 0.79 in, with room for the unpublished close-range behaviour — invented, settled by Step 0.3's flush / 1 / 2 / 3-in readings; 16 in = where the published accuracy alone, 5 % = 0.8 in, leaves no room under the 1.0-in bound); square within **5°**; spread **≤ 0.5 in**; **≥ 5 samples over ≥ 0.15 s** (window ≤ 0.5 s); confidence **≥ 0.5**; still **≤ 1 in/s, ≤ 3°/s**; error bound **≤ 1.0 in**; keep-out margin **3 in** (*RC33, derived:* the start pose's placement ~1 in + a field element's place, HA-207's ±0.5 in + the beam centre's aim, the facing tolerance 2° × 16 in ≈ 0.56 in — summed ≈ 2.1, rounded up; invented as a sum, settled by Step 0.6/0.8) **plus the along drift: 10 % of the travel since the along axis was last seeded or seated** (robot two's saved distance scale reads ~8 % short — `roadmap.md` ~:411, REMAINING-WORK :167, until the Calibrate rerun — rounded up; HA-83's 2 % σ is the register's belief for a calibrated robot; Step 0.8's taped legs settle it per robot); heading tolerance **2° + 2 % of the \|rotation\| since the heading was last seeded** (*RC38*: the floor is HA-20's ~1°/min over a 60-s run, doubled, as before; the growth is HA-84's 1-σ 1 % of rotation, doubled the same way — the draft's fixed 2° ignored HA-84: ~4.5° after 450° of turning at 1σ) | **invented** — each derived on paper (P1) from HA-204/205/209 and HA-20, none observed | L6-A1 (`WallResetLimits`, incl. `headingFloor` / `headingPerTurn`, `alongDriftRate`, `pieceTop` / `beamMargin`, `coneHalfAngle`) | Step 0.3–0.8; the first field session's `[WALL]` lines |
| **HA-212** | The **Loaders** are 4.02 in wide, protrude **3.74 in**, and are centred **58.77 in** either side of centre on the red and blue walls; *(RC34)* **the start pieces** — A12's Cup-Pin-Cup clusters, 6.5 in tall, centred 1.58 in off EVERY wall at \|along\| 18.81–28.28 in (VEX U's Figures VEXU-1 and VURS1 the same) — stand there until moved; *(RC36)* **in VEX U, Match Loads enter through the Loaders during the AUTONOMOUS period** (`<VUG4>`, v20.txt :3511-3512), into the 23.11-in Load Zones (A10's tape) — a robot taking them, and pieces (nested or not) it drops, may be there at any time; **no single Override piece is taller than 6.5 in** (v20.txt :4270, :4546 — a nested stack is, landmine 1); no other field element is fixed against a wall below its top (the nearest Goal is 23.11 in off a wall) | **reasoned** — A9/A10/A12 drawings, FO-1, VEXU-1, VURS1 | L6-A1 (`override_walls.hpp`'s Loader keep-outs and `PieceZone` spans; `WallResetLimits::pieceTop`) | Step 0.6 (a Loader's position and depth, and the start pieces' places, on a real field) |

Every entry's code site carries `A4 register HA-nn`; the register's direction-1 grep (no `PROVISIONAL (A4` without an HA
id) stays empty. HA-113/114/115 (the adapter's beliefs) are NOT changed; their rows gain "L6's wall reset also rests on
this" in their blast radius. HA-180 (a sideways skid invisible) gains "the wall reset (L6) sees it on the axis across
the wall it resets against".

---

## 5. OUT, and explicitly rejected

**OUT (with the chunk or the decision that owns it):**
- **Heading from a wall** (D9). One side sensor gives ONE range; the axis and the angle are two unknowns. Two sensors
  on ONE side, fore and aft, could measure the angle to the wall — hardware neither robot has (Q2's answer: one per
  side). A later chunk, if robot one is built with a pair; until then the IMU owns heading (master plan decision #4's
  neighbour, the caller seed `reseatHeading`, `localizer.hpp` :534-564, is untouched).
- **A passive "correct when it can" corrector** (D9). Every WRONG seat the probes found (S4b, S7b, S10b; S5d and the start pieces before RC34's gate) passed
  every reading gate; what makes a reset safe is the author's "I parked beside THIS wall, HERE". OUT until a chunk
  PROVES a background use safe — on robot evidence (Step 0.8's `[WALL]` lines from real fields), not on the host test
  plant's.
- **Drive-until-a-distance with a FRONT sensor** (D12; the handoff's original L6 text, `HANDOFF-remaining-work.md`
  :487-500, "drive slowly toward a known wall until the distance sensor reads X"). Robot two has no front sensor; its
  side sensors cannot face the wall a tank drives toward. That feature — and the drive-stall detector the handoff's
  gap (2) asks for, which only matters when driving INTO a wall — belongs to a robot with a front sensor; no chunk
  owns it yet (the coordinator places it if robot one carries one).
- **A reset against a GOAL** (the handoff's "a wall or goal"). Goals are octagonal and 3.25 / 5.8 / 8.7 in tall
  (v20.txt :4377-4380): a beam at D10's 8–10 in passes over all but the centre Goal, an octagon's face changes with the
  bearing, and whether a Goal stays put when bumped is not established. No owner.
- **The sensor's object size and object velocity** — unbound in the adapter on purpose (`hal/pros/distance.hpp`
  :40-44). Step 0.3 RECORDs object size against a wall and a Pin; a later chunk judges whether it separates them (it
  would attack landmine 1's class of wrong seat — the one thing no gate here can).
- **A route command for the path runner** (`runner::BuiltinCommand`, F8; the handoff's "or as a runner built-in —
  adding a built-in id is additive under F8"). A route step needs the wall, the side and `maxCorrection` as data in
  the route — G3a's panel and the route schema's business. OUT until G3a asks; the auton's form is the everyday call
  through `then()` (D4 (a), *RC48*).
- **Robot one's sensors** — no build exists. L6-B types robot two's block; robot one's is typed when it is built
  (Step 0.1 for its mounts), in the same `ChassisTable` field.
- **Squaring the robot to the wall automatically** (turn until the reading is least, or two sensors equal) — it
  would move the robot on a sensor's word; OUT, no owner. The author's route parks the robot square; the 5° gate
  refuses when it did not.
- **Telling the perimeter type at run time** (A13 portable vs A14 metal, 0.045 in apart) — OUT; HA-207 carries the
  difference inside its tolerance, and Step 0.6 records which type the team met.
- **The AI Vision camera** (VIS1) — L6 reads no camera, and needs nothing VIS1 builds (the team lead's 2026-10-14
  constraint). **What the auto-flipper (AF1) may take from L6:** the robot's side sensors through L6-A2's accessors,
  for REACH and PRESENCE at a Toggle only (VIS1's brief §6.5 item 3 — never as colour evidence); L6 builds none of
  AF1's use, and AF1 must know that L6's Toggle keep-out makes a wall reset REFUSE exactly where AF1 parks.
- **GPS and AprilTags** — after G3 (the team lead, 2026-10-14); a wall reset and a GPS corrector would then both
  write position, and how they interact is that chunk's question (the seat is a caller act; a corrector nudges every
  tick — D2).
- **Driver control** — no button resets against a wall; the driver's estimate is not used to score.

**Explicitly rejected (each with the decision that weighed it):** the full `setPose` for the seat (D2 (b), P2/P4:
the drift term and the quality class overclaim a one-axis measurement); a corrector NUDGE (D2 (c)); a library default
for `maxCorrection` (D7 (b): a 4-in default silently accepts a 3-in error); no keep-outs (D8 (b), S4b: 3.71 in wrong);
always stopping on a refusal (D3 (b)) or always continuing (D3 (c)); the auton building the sensors (D5 (b));
refusing the whole robot for a refused side sensor (the mechanism accessors' rule, VIS1's D5); a single reading
instead of a window (one sample cannot show a spread, an edge or a moving object); the MEAN instead of the median
(M21: one outlier inside the spread moves a mean); trusting confidence inside 200 mm (the adapter REPORTS 1.0 there —
landmine 6); a held value as a reading (the read-fault rule — landmine 3); the host field plant sharing the
library's geometry (landmine 9); widening the 5° square gate to "get coverage" (P1e: the beam-centre vs perpendicular
question stays under 0.05 in only inside it).

---

## 6. Coordination — shared files, who waits for whom, and what the consumers get

### 6.1 In flight while this was written (re-checked 2026-10-22 ~01:00; NOT edited — every worktree read only)
| Job | Worktree, base | State (its own log) | Its files | L6's relation |
|---|---|---|---|---|
| **MECH-A3** (position and speed) | `mech-a3`, on `fbaf729` (main has moved three commits since: `fd55ffc`, `ceb17c0`, `71b4764`) | built and reviewed; PAUSED partway through its review fixes (the coordinator's resume list, 2026-10-22: audit, finish, merge, then MECH-A4) | *RC28 — its `git status`, 2026-10-22 ~09:20:* `mechanism/{kinds,service,spec,wiring}.hpp`, `control/trapezoid_profile.hpp`, `sim/mechanism_plant.hpp`, the `Makefile` (the coordinator's resume audit reads why); tests (`mechanism_fixtures.hpp`, `mechanism_kinds_test.cpp`, `mechanism_{service,spec,position,speed}_test.cpp`, NEW `mechanism_position_rig.hpp`, `mechanism_chapter_example_test.cpp`, `compile_refusals/s70_…`, `sim_mechanism_plant_test.cpp`); docs: README, faq, guide 10/11/14/15/README, NEW guide 18 (mechanisms), the HA register, HANDOFF, REMAINING-WORK, build-order, roadmap, `mkdocs.yml`, `docs/api/`, the briefing | **no shared code file.** Shared DOCUMENTS with L6-A1: the changelog, guides 10/11/14, the HA register, the plan files, `mkdocs.yml` (generated nav), `docs/api/` — §6.8 |
| **CAL1 Part A2a** (the standard Calibrate's program) | `cal1-a2a`, on `ceb17c0` | mid-build, PAUSED (2026-10-22) | `calibration/calibrate_panel.hpp`, NEW `calibration/calibrate_program.hpp`, NEW `hal/pros/calibrate.hpp`, NEW `test/calibrate_program_{rig.hpp,test.cpp}`, NEW `test/{calibrate_panel_additions,pros_calibrate_adapter}_test.cpp`; *RC28 — and documents:* `docs/changelog.md`, `README.md`, `HANDOFF-remaining-work.md`, `ORIENTATION.md`, `REMAINING-WORK.md`, `build-order.md`, `docs/roadmap.md`, the briefing, `docs/api/`, `mkdocs.yml` | **no shared CODE file** (*RC28, corrected: the draft said "no shared file" — the changelog, REMAINING-WORK, build-order, the roadmap, HANDOFF, `docs/api/`, `mkdocs.yml` and the briefing are L6's documents too, merged serially, §6.8*). Its rig forwards `IDeviceProvider::distance()` (`test/calibrate_program_rig.hpp` :257); L6 adds NO provider member (D5), so the rig needs nothing from L6 |
| **VIS1-A1** (the camera as a device) | `vis1-a1`, on `ceb17c0` | resumed builder (its log, "RESUMED 2026-10-22 00:44"): the device half written (its §1.1–§1.8); its geometry, season header, host camera model, tests, the three "vision ABSENT" pins, documents and mutations NOT yet. *RC28, re-read 2026-10-22 ~09:20:* stopped a second time ~01:45 and RESUMED 08:44 (its log): the suite green at 2,551 cases, mutations M1–M9, M6b, M7b RED, M10 to re-run, its documents in progress | `config/robot_config.hpp` (+58 lines: `SensorsConfig::camera`, `PortUse::Camera`, `valid()`'s camera block after every older claim), NEW `config/camera_config.hpp`, `hal/{device_provider,device_registry}.hpp`, `hal/fake/fake_device_provider.hpp`, `hal/pros/{device_provider,device_registry}.hpp`, NEW `hal/{vision_frames,pros/ai_vision,fake/fake_vision_frames}.hpp`, `robot/robot.hpp` (+133: `buildCamera` in the BODY, last; the boot line's camera words LAST), tests (`boot_source_test.cpp` :159, `mechanism_devices_test.cpp`, `pros_device_registry_adapter_test.cpp`, `robot_builder_test.cpp`, the shim); to come: `vision/{camera_geometry,override_elements}.hpp`, `sim/camera_plant.hpp` | **L6-A2 waits for its MERGE** (`robot_config.hpp`, `robot.hpp`, the boot-line pins incl. its NEW `robot_camera_test.cpp` — *RC26*). **L6-A1 shares no code file** (and no namespace — *RC3*: VIS1's is `vision::override2026`, §6.4) |
| *(the coordinator, main worktree)* | `shulib-v2` — *RC28, re-read 2026-10-22 ~09:35: both items have LANDED* | the VIS1 plan commit `71b4764` (the Held line HA-192..HA-203, build-order, REMAINING-WORK, the worksheet) and the bounded suite runner `c09e022` (`tools/suite_run.py`; `briefing_status.py`, `doc_staleness_audit.py` and `release.py` run the suite only through it); the main worktree is clean | `docs/…`, `tools/…` | L6's plan commit (§3.3) goes on that clean tree; §7's suite line is `suite_run.py` |

### 6.2 Shared files — who writes them, who waits, who re-reads
| File | Written by (in flight / planned) | L6's run | Rule |
|---|---|---|---|
| `config/robot_config.hpp`, `robot/robot.hpp` | VIS1-A1 (now) | L6-A2 | **L6-A2 starts from the tip AFTER VIS1-A1 merges** and re-reads both — `PortUse::WallSensor` appended AFTER `Camera`; the wall block's `valid()` checks AFTER the camera's ("after every older claim, so each keeps its message" — VIS1-A1's own rule); the sensors built in the body AFTER `buildCamera`; `walls %s ` inserted before the camera's words (§1.5). Never a hand-merge of a copy taken before VIS1-A1 merged (the MECH-A2 port rule's lesson) |
| `test/boot_source_test.cpp`, `test/xdrive_config_test.cpp`, `test/robot_camera_test.cpp` (NEW in VIS1-A1) — the boot-line pins; and guide 10's example lines (:579-582 in VIS1-A1's tree) | VIS1-A1 (all of them, now) | L6-A2 | the same: L6-A2 edits them at VIS1-A1's merged form (*RC26: `robot_builder_test.cpp` survives the insertion and is not edited*) |
| `chassis/robot_context.hpp`, `motion/motion_scheduler.hpp` | nobody in flight (MECH-A4's list — MECH brief §2 — names neither; IMU1 only cites the scheduler) | L6-A2 | re-read at the tip; MECH-A4 builds Routines over `Chassis`, so a merged A4 changes nothing L6-A2 edits |
| `localization/localizer.hpp` | nobody (IMU1 changes `hal/pros/imu.hpp` and two comments elsewhere, not the Localizer — IMU1 brief §1) | L6-A1 | — |
| `hal/distance.hpp` (F4, FROZEN) | nobody | L6-A1, two COMMENT sentences | §7's comment-only check |
| `chassis/routine.hpp`, `test/routine_signature_pin_test.cpp`, `src/programs/{competition_run,auton_tester_run}.hpp`, `test/competition_run_test.cpp`, `version.hpp` | **MECH-A4** (next after A3: the Routine's mechanism steps, `MechanismStepFailed`, API 2.5); **MECH-A5** (rewrites `src/programs/{competition_program,competition_run,auton_tester_program,auton_tester_run,…}` — MECH brief §2's table, :2134) | L6-C (optional — D4, RC48) | **L6-C, if taken, after MECH-A4 merges AND either before MECH-A5 launches or after A5 merges** (*RC28: the draft named only A4*); §6.3 |
| `src/robots/chassis_table.hpp`, `src/common/boot_source.{hpp,cpp}`, `test/boot_source_test.cpp`, `test/chassis_table_test.cpp` | **MECH-A5** (rewrites `boot_source`, the programs, `autons.cpp`, the gate); **VIS1-B** (`ChassisTable::camera{}`, the copy — after A5, before A6) | L6-B | **L6-B before MECH-A5 launches; if A5 has launched, after A5 merges and before A6 launches**, beside VIS1-B: whichever of L6-B and VIS1-B goes second re-reads the other's merged form. *RC27, corrected:* both append a `ChassisTable` field; VIS1-B's brief says only "the camera block copied into `RobotConfig::sensors.camera`" (VIS1 §1.15, :1240-1241) — it names no route, so whichever goes second adds its block through the same overload L6-B introduces (or, if VIS1-B is first, L6-B follows VIS1-B's route). **L9 Part B** (after MECH-A5) moves `decideBootSourceFrom` into the library and derives the declared ports; its derivation "must include the sensor ports" (MECH brief :1913-1914) — L9-B re-reads L6-B's overload and carries the wall ports |
| *(RC47)* `src/programs/{auton_tester_run,auton_tester_panel}.hpp` and their tests | **MECH-A5** (rewrites the tester's programs — MECH brief §2's table) | L6-B | the boot_source row's rule: L6-B before MECH-A5 launches, or after A5 merges |
| `src/programs/bench_r3a.cpp` (station 11's functions only) | **MECH-A6** (`bench_r3a.{hpp,cpp}`: the 3 × 5 menu, station 13); **VIS1-B** (station 12's functions) | L6-B | L6-B never touches `bench_r3a.hpp`, `kMenu`, `menu_grid` or the station row; its pure parts go in the NEW `src/programs/bench_wall.hpp` (VIS1-B's `bench_camera.hpp` pattern). **Before A6 launches** — if A6 has launched, after A6 merges (A6 re-reads its tip anyway, MECH brief §2) |
| `R3a-BENCH-WORKSHEET.md` (Station H) | the plan commits (VIS1's: Station I; MECH-A5/A6: J.9, station 13; CAL1-B: Station G) | the L6 plan commit (H.4–H.6), L6-B (H.7) | different stations; serial merges |

**Order, all told:** the team lead's go → the plan commit (§3.3) → **L6-A1** (beside MECH-A3's merge and VIS1-A1's
build when slots allow) → **L6-A2** when VIS1-A1 merges (and L6-A1 has) → **L6-B** before MECH-A5 launches (else between A5
and A6, beside VIS1-B) → (**L6-C**, optional, after MECH-A4 and outside MECH-A5's run — §6.2). CAL1 (A2a's rest, A2b, B)
precedes MECH-A5 (the MECH brief's N9) under every order. With two or more slots nothing MECH waits; with ONE (the team
lead's rule when the budget runs low — in force since 2026-10-22 ~01:45) the ranking is the team lead's: **D13**
*(rewritten by RC29)* gives the slot arithmetic — (a) a second slot; (b) L6 right after MECH-A4, MECH-A5 slipping
~8–12 slot-days to about the freeze, VIS1-A1's rest ahead of MECH (an amendment of VIS1's accepted rule); (c) MECH first,
L6 at or past the freeze.

### 6.3 MECH-A4 and L6-C — the Routine, and the API number
MECH-A4 rewrites `chassis/routine.hpp` (mechanism steps, `MechanismStepFailed`, `RoutineResult::mechanismVerdict`),
appends its F10 pins, adds one `case` to each of the two exhaustive `RoutineStopCause` switches in `src/programs/`,
and takes **API 2.5** (MECH brief §1.9, §2's table). L6-C starts from A4's merged tip and appends AFTER it:
`RoutineStopCause::WallResetRefused` after `MechanismStepFailed` (its value pinned there), the step and
`lastWallReset()` after A4's members, its own `case` in each switch, and **API 2.6** — or the next number, if any
other run that grows a frozen surface lands between (none is planned between A4 and L6-C as this is written; the
coordinator checks `version.hpp` at L6-C's launch); whichever lands second takes the next number and re-reads the
other's pins. L6-C is OPTIONAL and not recommended (D4 (b), *RC48*); an auton resets through `then()` over the
everyday form (D4 (a)), which needs nothing from A4.

### 6.4 The season's facts in two headers, two namespaces (*RC3, corrected 2026-10-22*)
VIS1-A1 adds `vision/override_elements.hpp` in **`shulib::vision::override2026`** (`vis1-a1` working tree :33,
:123; its brief §1.10: `ToggleFace`, `kTogglePlaces[4]`, `PieceSize`, `kPinSize`, `kCupSize`, `ModelClass`,
`kOverrideClassNames`, `modelClassSize`, the wall top 11.50/11.54 in); L6-A1 adds `localization/override_walls.hpp`
in **`shulib::localization::override2026`** (§1.1). *The draft said both used `shulib::override2026` and built a
redefinition rule on it — wrong; there is no redefinition risk, and the rule is withdrawn.* One naming hazard stays,
for the guide and any code inside `namespace shulib::vision` (AF1's): there an UNQUALIFIED `override2026::` finds
VIS1's namespace, not L6's — L6's listings always spell `localization::override2026::`. Rules:
- **No shared declaration and no include either way** — L6 must not depend on VIS1 (the team lead's constraint), and
  VIS1's header is DATA for the camera. Where the two state the same fact (the Toggle centred on each wall; its
  length — L6's keep-out uses A8's 25.99 in WITH the caps, VIS1 the glossary's 25.8 without), **a host test in the
  second-merged run includes both and asserts they agree** (each Toggle centre within 0.01 in; L6's keep-out ⊇ VIS1's
  Toggle length). The wall top is VIS1's datum only; L6 does not need it in the library (§1.1 — the Toggle keep-out
  is lifted only by the MEASURED flag, never computed from the wall top and an unofficial cone).

### 6.5 CAL1 (A2a now; A2b, B next — B before MECH-A5, the MECH brief's N9)
No shared code file in any CAL1 part (CAL1 Part B's list — CAL1 brief §9.2 — names `src/programs/calibrate_program.cpp`,
NEW `src/robots/calibrate_facts.hpp`, `src/main.cpp`, `src/common/screen_text.hpp`, `src/build_selection.hpp`,
`Makefile`, `tools/src_build_gate.py`, `src/robots/mechanism_table.hpp`, three calibration headers and tests). CAL1-B
READS `chassis_table.hpp` (`withTypedDriveFrom(kCalibrateFacts, kChassis)`); L6-B's appended, defaulted
`ChassisTable::wallSensors` changes nothing it reads. Calibrate never builds a `robot::Robot`'s wall sensors and
never needs them; if CAL1's census lists port 4/5 as "DISTANCE", that is the registry's word, untouched by L6.

### 6.6 IMU1 (a brief, awaiting the team lead) — the HA numbers COLLIDE
The IMU1 brief (`briefs/IMU1-imu-rate-once-per-tick.md`, written beside this one) ALSO reserves **HA-204..HA-206**; its
continuation note of 2026-10-22 names the collision and recommends **IMU1 → HA-213..HA-215 unless IMU1 is accepted
first**. This brief agrees: L6 cites its nine ids through its whole body; IMU1's three are a mechanical rename. **If the
team lead accepts IMU1 first, L6 shifts to HA-207..HA-215** — a mechanical rename of every `HA-20[4-9]|HA-21[0-2]` in
this file at the plan commit, with §4's table re-checked line by line. The coordinator records whichever ids are given
in the register's Held line in the same plan commit — **and moves the "next free" id in the coordinator's launch
templates that predate it** (*RC28*: `.claude/coordinator/drafts/mech-a4-args.template.json` and
`vis1-a2-args.template.json` both say "The HA register: next free HA-204" today; after L6's plan commit the next free is
HA-213, or HA-216 if IMU1 was accepted first), so MECH-A4 — which launches after that commit under every D13 option —
never mints an id L6 holds. IMU1 shares no code file with L6 (`hal/pros/imu.hpp`, two
comments, the shim); its fix makes the Localizer's turn rate — which L6's stillness gate reads through `twist()` —
one value per tick, which only helps.

### 6.7 What the other chunks get from L6
- **G3a (the robot panel)** — the robot file's `walls` block is DATA it must write: `WallSensorsConfig {left, right:
  WallSensorConfig {port, mount: SideMount {measured, forward, left, height, facing, positionTolerance,
  facingTolerance, rangeBias, rangeAccuracy, coneHalfAngle, toggleChecked, provenance}}, limits: WallResetLimits {…},
  note}` (*as amended by RC34/RC38/RC41*) — vocabulary fixed by L6-A2; the coordinator tells G3a's brief at the plan commit (VIS1 §6.6's practice).
  Every measured number carries its provenance string. *(RC32)* **The VexBuilder exchange cannot carry it yet:**
  `docs/vexbuilder-integration.md` §3's `robotProfile` (F7, NOT frozen) has `sensors: [ { port, type } ]` only (:115) —
  no mount, tolerance or provenance; the plan commit records that gap beside G3a's note (the profile grows when G3a
  writes the block), and no L6 run edits that document.
- **AF1 (the auto-flipper)** — the side sensors through `robot.wallSensor(side)` (L6-A2), for reach and presence at a
  Toggle (VIS1 §6.5 item 3); L6's Toggle keep-out means a wall reset REFUSES at the place AF1 parks — AF1 must not
  count on one there.
- **L4 (drive to a piece)** — nothing. **ALLY1** — nothing (the walls are colour-blind; "red wall" is F1 geometry).
- **The student autons (G4 — the students author them)** — the call, its words, the guide's recipe, and Step 0.8's
  field drill. L6 writes no auton.

### 6.8 Documents every run merges serially
`docs/changelog.md` (sections by landing; a "No version bump" header for A1/A2/B — §1.8), `docs/hardware-assumptions.md`
(rows merge by number: MECH holds 182..186, VIS1 192..203, L6 204..212 — or its shifted ids, §6.6), guides 10/11/14/15
(MECH-A3 edits 10/11/14/15; VIS1-A1 10 and 15; L6-A1 10/11/14; L6-A2 03/10/15; L6-B 12/16/17 — *RC46, RC47*), `mkdocs.yml` and `docs/api/` and
`PROJECT-BRIEFING.md` (GENERATED — regenerated after each merge, never hand-merged: `api_doc_tool.py generate`;
`briefing_status.py generate` only against a FRESH build — `rm -f build/test/shulib_tests` and rebuild first, the
memory rule's lesson), `build-order.md`'s first bold `Next:` (one long line every run edits — MECH-A3's merge notes
already flag the certain conflict with CAL1's sentence), `REMAINING-WORK.md`, `docs/roadmap.md`, the worksheet.

---

## 7. Checks — the coordinator re-runs every one after each run (RESUMING's step 3: never the report's word)
Every binary under the memory rule (2026-10-15). `<scratch>` is the coordinator's scratch folder — never `/tmp`.
```sh
cmake --build build/test -j"$(nproc)"
python3 tools/suite_run.py green   # the bounded run, landed at c09e022 (4 GiB, 300 s, output to a file; BROKEN is not red) — RC28
#   (the bare form, if ever needed: (ulimit -v 4194304; timeout 300 ./build/test/shulib_tests) | tail -6)
ctest --test-dir build/test -R compile_refusal --no-tests=error --output-on-failure | tail -3
# the ARM header gate (RESUMING :105-112) — every v2 header in ONE TU (L6's and VIS1's season headers side by side, §6.4)
find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort | awk '{print "#include \""$0"\""}' > <scratch>/all.cpp
echo "int main(){return 0;}" >> <scratch>/all.cpp
arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror \
  -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -c <scratch>/all.cpp -o /dev/null -Iinclude
# the CI guards (ci.yml ~:59-63, ~:75-80)
grep -rnE '#[[:space:]]*include[[:space:]]*[<"]pros/' include/shulib | grep -v '^include/shulib/hal/pros/'   # empty
grep -rnE --exclude-dir=sim '#[[:space:]]*include[[:space:]]*[<"]shulib/sim/' include/shulib              # empty
python3 tools/src_build_gate.py check            # every run (L6-A changes headers src/ includes); B and C also: self-test
python3 tools/api_doc_tool.py self-test && python3 tools/api_doc_tool.py check-coverage \
  && python3 tools/api_doc_tool.py check-fresh && python3 tools/api_doc_tool.py check-examples \
  && python3 tools/api_doc_tool.py check-removability
python3 tools/doc_staleness_audit.py && python3 tools/briefing_status.py check   # the briefing: against a FRESH build only
# ── the brief's own ──
# F4 stays frozen: hal/distance.hpp may differ ONLY in comment lines (§3.4) — prints nothing
git diff -U0 HEAD -- include/shulib/hal/distance.hpp | grep -E '^[+-][^+-]' | grep -vE '^[+-][[:space:]]*//'
git diff --stat -- src/ tools/ Makefile           # L6-A1, L6-A2: empty
grep -rn 'seatAxisProto\|l6proto\|l6field\|PROBE_' include/shulib test   # empty (no scratch code leaked; bare "PROBE"/"prototype" already occur 10 times at ceb17c0 in old comments)
grep -n 'setPose\|reseatHeading\|headingBias' include/shulib/localization/wall_reset.hpp include/shulib/chassis/wall_reset_call.hpp include/shulib/competition/wall_reset.hpp | grep -vE '^[^:]+:[0-9]+:[[:space:]]*//'   # empty: the seat is seatAxis, heading untouched (doc comments may name them)
grep -nE 'override_walls|wall_reset|localization/' include/shulib/sim/field_plant.hpp       # empty: the plant stays independent
grep -nE '#include.*(chassis|motion|sequence|competition)/' include/shulib/localization/{axis,wall_reset,override_walls}.hpp   # empty: PURE (RC24)
grep -n 'maxCorrection' include/shulib/localization/wall_reset.hpp   # the {0.0} UNSET default and its refusal — no other default
grep -n 'walls' include/shulib/motion/motion_scheduler.hpp           # L6-A2: exactly the shadow context's one line (§1.6)
grep -rn 'case PortUse::' include src test | wc -l                   # L6-A2: one more than at VIS1-A1's merge (12 if VIS1 added only Camera)
grep -rn 'PROVISIONAL (A4' include/shulib | grep -v 'HA-[0-9]'       # empty (the register's direction-1 grep)
grep -rnoE 'HA-(20[4-9]|21[0-2])' include src test | sort -u          # every id has its register row, and every row a code site (or its shifted ids, §6.6)
grep -rln 'internal/\|chunks/\|RESUMING\|build-order' docs/*.md docs/guide docs/cookbook README.md   # empty (C7)
grep -rnE 'RC[0-9]+\b' docs/guide docs/cookbook include/shulib test/cookbook_examples_test.cpp test/guide_examples_test.cpp   # empty: no re-check tag leaks into a listing (RC23)
grep -rn -i 'simulator' docs/guide/03-knowing-where-you-are.md docs/cookbook/05-mixing-tiers.md      # after L6-A1: not the re-seed paragraph's (05 :111 at ceb17c0); no new hit
```
Plus the spot checks — the run's single most load-bearing rule, in the source, and two of its mutations re-run by
hand (seen RED, restored, the suite green again):
- **L6-A1:** a REFUSED reset leaves the Localizer bit-identical (T6's second half) and a SEATED one writes ONE axis
  (`seatAxis`'s body: no `distanceSinceCorrection_`, no `headingBias_`, no `odom_`); re-run **M17** (the seat clears
  the drift term) and **M19 under `EkfFusion`** (*RC1*), and **M8** (the footprint widening dropped).
- **L6-A2:** the shadow context carries the slot (re-run **M31**) and a refused side sensor never refuses the robot
  (re-run **M27**).
- **L6-B:** robot two's block says `measured = false` unless Step 0.1's sheet exists and is cited (no invented mount);
  `git diff -- src/programs/bench_r3a.cpp` touches station 11's functions only (no `kMenu`, no `menu_grid`, no station
  row); re-run **MB2**.
- **L6-C** (only if taken): `WallResetRefused` is APPENDED after MECH-A4's `MechanismStepFailed` and pinned (re-run **MC3**).
- **L6-A2** (*RC48*): re-run **M40** (the start check dropped) and **M41** (the frozen wait) by hand.

---

## 8. Landmines
1. **An object taller than a game piece, at a plausible distance, passes every gate** — a robot beside a backwards
   sensor (S7b, 2.96 in), a Loader with the keep-outs off (S4b, 3.69 in in the F1 frame — RC40), a Toggle under a high
   beam (S10b, 0.75 in), something close behind a clear panel (RC37), a nested stack against the wall. No reading gate
   can tell them from the wall, and **no document may say one can.** *(RC34)* A SINGLE standing Pin or Cup (S5d, A12's
   start pieces) is no longer in this class — the beam-height gate refuses any reading its cone could take from one —
   so long as the typed height and cone are true (HA-205, HA-210): never type a height "so the reset works". The
   defences are the author's (`maxCorrection` tight; parked where the route says), the keep-outs (D8), the beam-height
   gate and the mount (D10) and Step 0.4's side check. T8 pins S4b/S7b and the clear-panel case as KNOWN LIMITS so a
   change either way is seen.
2. **Never `setPose` for a seat, never the heading.** `setPose` clears the drift term and, from the second tick after
   it (the first is the dt-0 re-base tick, Degraded either way — RC30), lifts the quality from Degraded to DeadReckon (P4) — a one-axis measurement passing as a two-axis one. Nothing in L6 calls
   `reseatHeading` or touches `headingBias_` (§7 greps it).
3. **A held value is a reading.** `ProsDistance` answers PROS_ERR with the last good distance and confidence 0, and
   counts it (`hal/pros/distance.hpp` :70-76); a held value is byte for byte a reading (`hal/read_faults.hpp` :12-23).
   And `distance()` and `confidence()` are TWO device reads (:75-76): one can fail while the other succeeds. So the
   count is read before the first and after the second read of EVERY sample (§1.2 rule 6) — never once per window
   (M15), never around `distance()` alone (M16).
4. **The context's first OPTIONAL slot — and the scheduler's SHADOW context.** (a) `RobotContext`'s constructor
   requires every slot non-null (`robot_context.hpp` :65-77); the walls slot must NOT gain a precondition — nullptr
   means "this robot declares no wall sensors", the reset refuses NO SENSOR, and every existing caller (ten construction
   sites: `sim/scenario.hpp` :142, `robot/robot.hpp` :563, `motion/motion_scheduler.hpp` :660, and seven in six test
   files — `grep -rn 'RobotContextConfig{' include src test`) compiles and runs unchanged. The context forward-declares `localization::WallSensors` and includes no localization header (it is L3,
   "it sees only the L0 interfaces", :9); it never dereferences the pointer. (b) `Chassis::deps().ctx` is the
   scheduler's SHADOW context, copied field by field (`motion_scheduler.hpp` :660-667, §1.6): a slot not carried there
   is nullptr in every chassis — and the file's comment records the same trap sprung once before (motorGroups). M31.
5. **The EXPLICIT form's sampling window is a FROZEN wait** (*RC48: the everyday form samples through the guard's
   deadline-aware `waitFor` and refuses `cut` — T17*). It runs inside `Chassis::waitUntil`, which the run guard cannot cut
   (`sequence/run_guard.hpp` :95-109): a deadline or a disable that lands mid-reset is acted on at most `windowMax`
   later (0.5 s default; `valid()` refuses more than 1 s). Never sample through an unbounded wait; T9 pins the bound
   with a sensor stuck at 9999 (M23). The guide says the lateness in so many words.
6. **Confidence inside 200 mm is synthetic.** The adapter REPORTS 1.0 for every valid reading at or below 200 mm
   (7.87 in), because PROS documents the raw channel as undefined there (`hal/pros/distance.hpp` :24-31, HA-115). So
   between `rangeMin` (2 in) and 7.87 in the confidence gate checks nothing; the range, spread, error-bound and
   disagreement gates carry those readings. Never "fix" this by passing the raw channel through.
7. **No fault channel is not "no faults".** A sensor with `faults == nullptr` (a fake, or a provider that never
   screens) is accepted — a PROS_ERR still reads confidence 0 and refuses NO RETURN — and the log line says
   `faults n/a`, never `faults ok` (T5).
8. **A RELATIVE start is not the field frame.** The season's walls (`localization::override2026::kLeftWall`…) are F1 coordinates; a
   RELATIVE-start auton (the team's 40 in auton is one, `src/team/autons.cpp` :52-56, :66) has its origin wherever the
   robot started, so `kLeftWall` means nothing there and the reset would SEAT x to a field number in a robot frame —
   inside a generous `maxCorrection` that is a silent wrong seat. *(RC48, corrected: the draft said "the library
   cannot see the start mode" — true only of a `Chassis`-taking call; the auton's context carries `entry`, whose
   `effectiveStart()` is the mode.)* The everyday call REFUSES any start that is not FIELD (`NotFieldStart`, T17). The
   explicit form (Tier 3, tests) still takes a `Chassis` and cannot see it — there `FieldWall::line` carries the
   author's own keep-outs or none, documented (T11). **No FIELD-start auton exists in `src/team/autons.cpp` today**
   (every entry is RELATIVE, :66): the team's first wall-reset auton is also its first FIELD start.
9. **The host field plant must stay INDEPENDENT** (the A2 rule): `sim/field_plant.hpp` has its own copy of the drawing
   numbers and its own ray cast and includes nothing from `localization/` (§7 greps it). A shared helper "to avoid
   duplication" would hide a sign error in both — T1's M1–M4 and T14's agreement case are what see one.
10. **`-Wswitch -Werror`.** Appending `PortUse::WallSensor` breaks `portUseWord` until it gains its case — at
    `ceb17c0` the only switch on `PortUse` in `include/`, `src/` and `test/` (10 `case` lines); VIS1-A1 appends
    `Camera` first — re-grep at its merged tip. `WallResetVerdict` is APPEND-ONLY and `wallResetVerdictWord` is an
    exhaustive switch with no `default:` (a new verdict without a word must not compile).
11. **Two season headers, two namespaces** *(RC3, corrected 2026-10-22: the draft said one shared namespace)*:
    `localization::override2026` (L6) and `vision::override2026` (VIS1). Never "unify" them into one (the camera's
    data and the reset's would then share a scope and could collide in the ARM gate's one translation unit), and
    inside `namespace shulib::vision` qualify L6's as `localization::override2026::` (§6.4).
12. **The sample buffer is fixed** (`kMaxWallSamples` = 64; allocation-free). At the 10-ms tick
    (`hal/pros/tick_pacer.hpp` :51) the 0.5-s `windowMax` is 50 samples; a faster pacer, or a host test with a short
    period, would fill it — the sampler STOPS at capacity and evaluates what it has; it never wraps or overruns.
    `valid()` caps `minSamples` at 64.
13. **N samples are not N readings** (HA-209): PROS returns the sensor's latest value, so reads inside one update
    repeat it — ten identical samples have a spread of 0 and prove less than ten. The window's TIME (≥ 0.15 s) is the
    real guard, not the count; never shorten the window to "speed the auton up" without Step 0.7's update interval.
14. **Leave the drift term alone after a seat.** A reviewer may "fix" `seatAxis` to reset `distanceSinceCorrection`
    — don't: the Localizer has ONE drift scalar for two axes, and the other axis was not measured (D2; T6, M17).
    *(RC1)* Nor "complete" it by shifting the odometry the way `setPose` does (re-taking its heading reference for
    nothing), nor drop `hasLast_ = false` because a complementary-tier test stays green — the EKF tier needs it (M19).
15. **The prototypes are not code to copy blind.** `seatAxisProto` has NO live precondition (the real `seatAxis` must
    — T7, M20); `proto/wall_proto.hpp` uses `std::vector` and plain `//` comments with several declarators per line —
    `api_doc_tool` fails the BUILD on an undocumented public entity (one `///` per member), and the library allocates
    nothing.
16. **No invented value, ever** (HA-111's lesson): robot two's mounts stay `measured = false` until Step 0.1's sheet
    exists and is cited; `maxCorrection` has no default; `rangeBias` stays 0 until Step 0.3 shows one; a placeholder
    "so the tests pass" is the defect class this project has paid for before.
17. **The reset does not stop the robot.** It commands nothing to the drive (T9); a robot still coasting from the last
    leg refuses MOVING. The recipe brakes (or lets the leg settle) first, then resets.
18. **Line numbers drift.** Every `:N` here is `ceb17c0`'s; VIS1-A1 moves `robot_config.hpp` and `robot.hpp` (by ~60
    and ~130 lines), MECH-A3 and A4 move others. Re-read; never paste a line number from this brief into code or a doc.
19. **Season data is dated, and the two perimeters differ.** `override_walls.hpp` is manual v2.0's (or a newer one's,
    if the executor finds it — and says which); A13 and A14 disagree by 0.045 in per wall (HA-207) — state it, never
    average it silently. A manual change is a one-header change with its citation.
20. **The HA ids may shift** (§6.6, the IMU1 collision): if the team lead accepts IMU1 first, every
    `HA-20[4-9]|HA-21[0-2]` here moves by three — a whole-file mechanical rename at the plan commit, never a partial one
    (a code site citing a row that belongs to another chunk is the register's worst failure).
21. **"Simulator" is not this project's word** — the host test plant; the host field plant (cookbook 05's paragraph is
    fixed on the way, §3.1).
22. **The memory rule** (2026-10-15): every binary — probes, the suite, each mutant — under `ulimit -v 4194304` and
    `timeout 300`. M23 (the window bound ignored) makes a stuck-at-9999 test wait out the whole `waitUntil` timeout —
    bounded, but a mutation driver that ran it without `timeout` would hang; and a mutant that keeps appending samples
    past the buffer is exactly the runaway that froze the laptop at 58 GB.
23. **A seat is a deliberate jump, and the pose guard knows only dt** *(RC31)*. The scheduler's `PoseDeltaGuard`
    raises `IMPLAUSIBLE` (advisory — guide 11 :342) when a tick's pose delta exceeds 150 in/s × 1.5 × dt — 2.25 in at the
    10-ms tick — and forgives a jump only on a dt-0 tick, which the loop monitor gives on the first tick after `async()`
    or any blocking wait (`motion_scheduler.hpp` :761, :798, :830; `loop_monitor.hpp` :84-90). `maxCorrection` 3 in (D4's
    example) and P2 S1's 2.5 in are both over 2.25. Measured (the checker's `shadow_and_guard.txt`, re-run by this
    writer): a 2.5-in seat then a blocking `moveTo` raised 0; then a caller-paced `scheduler().tick()` raised 1. So: the
    seat lands AFTER the sampling wait returns; an auton's next step is a verb or a wait (dt 0 — nothing raised, T8); a
    Tier-3 loop that ticks the scheduler itself sees ONE advisory `IMPLAUSIBLE` per seat over 2.25 in — guide 10 says so
    beside the explicit call. Never "fix" it by clamping the seat to 2.25 in (the reset would then lie about what it
    applied) or by silencing the guard. And the sampling predicate always ends the wait itself, so the frozen wait never
    writes its own "waitUntil timed out" line (T9: ONE line per call).

---

## 9. Summary
*(Regenerated from the amended body at the end of the second re-check, 2026-10-22 — RC23.)*

**What L6 is:** the wall reset as library code. When a FIELD-start auton has parked its robot nose-along a field wall
and squared up, the side distance sensor facing that wall measures how far the wall is; from the sensor's MEASURED
place on the robot that gives ONE coordinate — x against the left or right wall, y against the red or blue wall — and
the library replaces that one coordinate of its estimate (`Localizer::seatAxis`) and nothing else: not the other
coordinate, not the heading, not the estimate's trust in itself. Before it uses a reading it checks, in order — the
setup (a FIELD start, a sensor, a measured mount, no motion running), the aim (square within 5°), the place (outside the
keep-outs: the Toggle, its brackets, the Loaders, the corners, the VEX U Load Zones, and the start pieces until the
beam's cone is measured — each widened by the beam, a margin and the drift along the wall), the beam's height (no
reading a single Pin or Cup could give), stillness, every reading (no read fault, a real return, in range), their
agreement, the precision (≤ 1 in, from the accuracy, the mount and a heading tolerance that grows with turning), and
agreement with the estimate (≤ the author's `maxCorrection`) — and REFUSES at the first failure with one log line that
names it, its numbers and, in the guide, what to do. A refusal leaves the estimate exactly as it was; a deadline or a
disable mid-reset cuts it. It never moves the robot, never runs in the background, never reads the camera.

**Decisions for the team lead (each with a recommendation):** **D1** three runs — L6-A1 (the library core) on your go,
L6-A2 (the robot's side sensors and the everyday call) when VIS1-A1 merges, L6-B (robot two, the WALL bench page, the
tester's wall line) before MECH-A5 launches; L6-C (a `Routine` member) optional, not recommended · **D2** a ONE-AXIS seat
(`seatAxis`) — which **AMENDS locked decision #4** ("`relocalize()` … does not snap") for a caller-invoked one-axis wall
reset with no motion running; never `setPose` (it overclaims) · **D3** a refusal continues the auton unless the author
marks the reset `required` · **D4** the auton calls `competition::resetFromWall(ctx, …)` through `then()` — it refuses a
RELATIVE start and uses the run guard's deadline-aware wait; the example is a FIELD-start sequence (face, a leg along the
wall, square up, a ~7-in reading) · **D5** the sensors' facts in the robot file, VIS1's template · **D6** the mount
MEASURED per robot (unmeasured ⇒ `MOUNT UNSET`), with optional measured bias, accuracy, cone and Toggle check · **D7**
the gates are PROVISIONAL library defaults; `maxCorrection` has NO default · **D8** the season's keep-outs by default:
the Toggle body (lifted only by a measured check), its brackets, the Loaders, the corners and the VEX U Load Zones
(always), A12's start pieces (until the cone is measured) · **D9** heading correction and a passive mode both OUT ·
**D10** mounting advice: lens 10–10.5 in up (the beam-height ceiling decides how far the reset can read), at the
fore-aft centre (load-bearing for precision), ≥ 2.5 in inboard, square, clear of the robot's own parts — robot two's
sensors, if low, refuse until moved up · **D11** fix cookbook 05's re-seed recipe, which overwrites the OTHER axis ·
**D12** reword REMAINING-WORK row 25 (re-seat against a wall the robot parked beside — no "drive toward") · **D13** with
two slots, MECH untouched and L6-B ~Oct 30 – Nov 3; with ONE slot, L6 right after MECH-A4 (L6-B ~Nov 5–10; MECH-A5 slips
~8–12 slot-days, to about the ~Nov 16 freeze; VIS1-A1's rest ahead of MECH amends VIS1's rule) or MECH first (L6-B ~Nov
12–22).

**Sizes (builder-days, against other briefs' ESTIMATES — none measured; ~½–1 of verification per run on top):** **L6-A1
~2¾–3½ · L6-A2 ~1¾–2 · L6-B ~1½–2 — L6 ~6–7½** (~7½–10½ slot-days); the optional L6-C ~½–¾. **The useful first landing
before 2026-11-23 is A1 + A2 + B:** then a student's FIELD-start auton can re-seat x or y against a wall it parks beside,
to within ~0.9–1.0 in with VEX's published accuracy and a ±0.25-in mount — ~0.5 in once Step 0.1 and 0.3 measure them —
or carry on unchanged when refused (no FIELD-start auton exists yet). **Step 0.1–0.5 need no code and should run at the
next bench session**: the mount's height decides whether robot two can use the reset at all, and the measured accuracy
and cone how much of the wall it can use.

**Files:** NEW `localization/{axis,override_walls,wall_reset}.hpp` (pure), `chassis/wall_reset_call.hpp` (the explicit
call), `competition/wall_reset.hpp` (the everyday call), `config/wall_sensor_config.hpp`, `sim/field_plant.hpp`,
`src/programs/bench_wall.hpp`, ~11 test files; CHANGED `localization/localizer.hpp` (`seatAxis`, two counters),
`hal/distance.hpp` (two comment sentences only), `config/robot_config.hpp`, `robot/robot.hpp`,
`chassis/robot_context.hpp` (the first OPTIONAL slot), `motion/motion_scheduler.hpp` (one line — the shadow context),
`src/robots/chassis_table.hpp`, `src/common/boot_source.{hpp,cpp}` (a new decision overload),
`src/programs/{bench_r3a.cpp (station 11 only), auton_tester_run.hpp, auton_tester_panel.hpp}`, the boot-line pins
(`boot_source_test`, `xdrive_config_test`, VIS1-A1's `robot_camera_test`), `test/cookbook_examples_test.cpp`,
`test/drive_program_source_test.cpp`; (L6-C, if taken: `chassis/routine.hpp` + its pin, two `src/` switches,
`version.hpp`); documents per §3.

**Probes (`.claude/coordinator/scratch/brief-L6/`, every binary under `ulimit -v 4 GiB` + `timeout 300`):**
- **P1** (`p1_geometry.py`): the geometry — the axis error from a heading error ≈ the sensor's FORWARD offset × the
  heading error; a mount typed wrong passes ~1:1 in the left offset only; the beam-centre vs perpendicular question is
  ≤ 0.046 in inside the 5° gate. (Its accuracy model and budget are superseded by Q1 — RC38.)
- **P2** (`p2_scenarios.cpp` + an INDEPENDENT cone-of-rays field model, a tank `ChassisRig`): 19 scenarios — each
  refusal caught by its own check; real objects at plausible distances SEAT WRONG with no gate seeing them (S4b a
  Loader with keep-outs off −3.71 in, −3.69 in the F1 frame; S7b a backwards sensor beside a robot −2.96; S10b the
  Toggle with keep-outs off −0.75; S5d a standing Pin under a 4-in sensor −2.59, now refused by the beam-height gate).
- **P3** (`p3_arm.cpp`): the PROTOTYPE compiles for the brain. **P4** (`p4_quality.cpp`): from the second tick after a
  seat, `setPose` reads DeadReckon 1.000 while the one-axis seat keeps Degraded 0.200 (the first tick is the dt-0
  re-base, Degraded in both).
- **Re-check, first pass** (`recheck/`): R1 (the odometry shift an equivalent mutant; `hasLast_` load-bearing under the
  EKF only), R3 (S4 re-run in the F1 frame; the cluster, along-error and clear-panel rows).
- **Re-check, second pass** (`recheck2/`): **Q1** — the budget with max(15 mm, 5 %) and the growing heading tolerance
  (centred: 0.91–0.98 in before turning, 1.02–1.19 after 180°; 5 in forward: 1.09–1.15; with Step 0: 0.46–0.52), the
  beam-height ceilings, the clean spans; **Q1b** — the spans with the Load Zones fixed (2 × 23.6 in per wall after a
  seed once the cone is measured, 2 × 8 before); **Q2** — the amended gates on the F1 field: the along-drift term refuses
  the Loader case the draft seated 1.57 in wrong; the beam-height gate refuses every start-cluster case the draft
  seated 1.16–2.86 in wrong and keeps every correct one within 0.06 in; a low lens over the floor refused; a clear panel
  with an object 1–2 in behind still seats 1.1–2.0 in wrong (a known limit); a Toggle bracket seats 0.96 in wrong if the
  whole Toggle keep-out is lifted (so the brackets never are); **Q3** — the amended header layout compiles (host, both
  include orders; ARM, every header in one unit) with D4's listing inside `namespace shulib::bench`.

**Found while writing and re-checking (each handled):** cookbook 05's recipe writes y = 0 (D11) and says "simulator";
row 25's "drive toward a wall" (D12); the context's first optional slot (landmine 4); the scheduler's SHADOW context
(§1.6, M31); the API-version policy (§1.8); the wall heights printed in A13/A14 and the perimeters' 0.045-in difference
(HA-207/208); the HA-id collision with IMU1 (§6.6); the after-VIS1-A1 boot line (§1.5); and from the re-check: the
include cycle (RC24), decision #4's relocalize sentence (RC25), the start pieces on every wall (RC34), the GPS code strip
(RC35), VEX U loading during the auton (RC36), the Toggle's brackets (RC41), the pose guard's 2.25 in (RC31), the
invisible refusal in the tester (RC47), the RELATIVE start (RC48).

**UNVERIFIED (each a §4 entry, a Step 0 question, or a coordination fact to re-check):**
- whether manual **v2.1** exists and changed A8–A10, A12, A13 or A14 (404 on 2026-10-18 per VIS1-A1's log); every field
  number here is v2.0's;
- **the sensor on our robot** — no distance reading of robot two exists (station 11 never run): its accuracy, zero
  point, bias and still spread (HA-204); its cone and what it reports with two surfaces in it (HA-205 — a community post
  seen through search summaries; the beam-height gate and the piece zones rest on the cone); its update interval and
  latency (HA-209);
- **whether the beam reads the metal perimeter's clear panels** (HA-206 — contrary evidence; something close behind one
  can seat wrong), **and the GPS code strip's black squares like its white ones** (HA-206), and which perimeter type the
  team's events use;
- a real field's size and tolerance (HA-207); the Toggle's overhang and its brackets' depth (HA-208); a Loader's place
  and depth and the start pieces' places (HA-212);
- **robot two's side-sensor mounts** (HA-210: ports from the 2026-09-24 census, facing from the team lead — nothing
  measured, HEIGHT included, which decides whether the reset can work on robot two at all), and whether anything of the
  robot's own enters the cone;
- **every gate default** (HA-211 — derived on paper, none observed: the 10 % along drift rests on robot two's ~8 %-short
  saved scale; the 2 % per turn on HA-84);
- the P2/Q2 results come from host models this writer built (rays, boxes, ±0.15 in noise, the nearest-surface belief) —
  they show which checks catch which cases, not how often a field produces them;
- the slot-day dates in D13 assume one builder-day per calendar day — none is a measured duration;
- VIS1-A1's FINAL boot-line form (an unfinished working tree, re-read 2026-10-22) — L6-A2 re-reads it at the merge;
- the HA ids (204..212, or 207..215 if IMU1 is accepted first);
- **nothing in L6 has run on a robot.**
