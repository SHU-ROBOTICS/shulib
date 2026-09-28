# TANK1 — progress log (live)

- **Chunk:** TANK1 — `moveTo` on a tank steers back onto its line (brief:
  [`TANK1-tank-moveto-steers.md`](TANK1-tank-moveto-steers.md))
- **Executor model:** opus
- Appended as the work goes, with a heading per part. Every time in a section header is a `date`
  reading taken when the section was written, not an estimate written afterwards. **Nothing in this
  chunk has run on a robot.** Every performance number here is from the host test plant.

---

## Part A — the steering, the flag, the Warn, the tests, its documents

- **Started:** Sun Sep 27 08:23:45 AM EDT 2026 (`date`).
- **Tree at start:** `a62067b`, clean, in the git worktree `.claude/worktrees/tank1` (branch `tank1`) —
  NOT the main working tree.
- **Go:** the coordinator's workflow run, 2026-09-27, on the team lead's go ("Go, all recommended":
  D1 (a) 2.3, D2 (a) the 15° window, D3 (a) Part A now / Part B later, D4 (a) IMU1 separate). Do not
  commit; no `pros` command; no `/dev/ttyACM*`. Mutation backups under
  `.claude/coordinator/scratch/tank1-a/` (never `/tmp`).
- **Scope:** Part A ONLY — `motion/tank_steer.hpp` (new leaf), `MotionConfig::tankSteer`, `MoveToPose`'s tank
  path, the record flag (TSTR), the one Warn, the tests of §2.1–§2.3, §2.5's Part A mutations, §2.6's pin
  edit, every Part A document of §3, the changelog BEHAVIOUR CHANGE entry, version 2.3, HA-179 and HA-180.
  NOT Part B (the ` xt` column). No `src/` file changes.

### 08:23 — start
- Read `docs/internal/RESUMING.md` (the chunk loop, the standards, pace rules 1–8) and the brief in full
  (the go and the answers at the top, the Re-check amendments, § Decisions, §0–§8). This log created
  before any code, and before the P10 (a) pre-check (next).

### 08:24–08:27 — P10 (a) pre-check: the golden hashes on the PRISTINE tree *(`date` 08:27:28 at this entry)*
- **Before any code edit** (landmine 9). Probe `.claude/coordinator/scratch/tank1-a/p10_hash.cpp` (evidence,
  not project code): the brief's `probe_proposed.cpp` `holonomicScript` and `OFFSWITCH` copied verbatim, plus
  tank OFF-switch scripts that hash records AND log lines AND truth (Part A adds no ` xt` column, so with
  `maxSteer = 0` the log must match too): the ×0.95 40 in with a `RunReporter`; a tank script of every verb
  (faced `moveTo`, a leg 31° off its heading, a lateral `strafeTo`, `turnTo`, a reverse leg, `followTrajectory`,
  a speed-capped `moveTo`, `hold`, `brake`) at ×1.00 clean, ×0.91 clean and ×0.91 under composed hostility
  (seed 7); the acceptance run (via `Routine`) at R ×0.95, R ×0.91 and L ×0.91. Hash: FNV-1a over every
  record's `blackbox::encodeTick` bytes, every log line (level, tag, text) and the final truth.
- Built with `.claude/coordinator/scratch/tank1-a/b.sh` (the brief's `recheck/b.sh` flags: g++ 13.3.0
  `-std=c++20 -O2`, x86-64, Ubuntu 24.04 host `lodgepc`), against the worktree at `a62067b` with ONLY this log
  added (`git status`: `?? docs/internal/chunks/TANK1-PROGRESS.md`). Output kept as `p10_pre.txt`:

```
HOLO X clean                      records  2376 lines   8 hash 94744b94c963692e
HOLO X hostile s7                 records  2978 lines   8 hash 266ba26177532bdb
HOLO H clean                      records  2444 lines   8 hash 204ca897f71b7c33
HOLO H hostile s7                 records  3056 lines   8 hash 0920bd182607e36c
OFFSWITCH tank x0.95 maxSteer0 records 1003 hash a98d456dd75f0497
OFFSWITCH+log tank x0.95 records 1003 lines 1 hash e51558fe8e0d02ff
TANKOFF R x1.00 clean             records  5807 lines   9 hash 26657896bd82a7f9
TANKOFF R x0.91 clean             records  6515 lines   9 hash 8841616b0b4dc341
TANKOFF R x0.91 hostile s7        records  6515 lines   9 hash 1e337a62acc55cfb
ACCEPTOFF R x0.95 ok=0 0/4 records  1003 lines   5 hash a65a5f914c97789f
ACCEPTOFF R x0.91 ok=0 0/4 records  1003 lines   5 hash f69a2b5445e2ccf4
ACCEPTOFF L x0.91 ok=0 0/4 records  1003 lines   5 hash e8eba48d093fdea4
```
- The first five lines are byte-identical to the brief's `recheck/pp_tree.txt` (captured at `9e86a3c`): the
  two commits since touched no file these scripts reach.

### after 08:27 — baseline suite, then the library code *(`date` 12:46:29 at this entry — the wall clock jumped
  ~4 h between two consecutive tool calls with no work in between that this log knows of; recorded as read)*
- **Baseline host suite** (fresh `cmake -S test -B build/test` at `a62067b` + this log; the briefing regenerated
  first, since this log opens TANK1): `[doctest] test cases: 1685 | 1685 passed | 0 failed | 3 skipped`,
  `assertions: 2469415 | 2469415 passed | 0 failed`. (The brief's 1659 was measured at `9e86a3c`; T4 Part 1a's
  tests landed since.)
- **Code, as the brief's amended overlay (`proposed/`) writes it, read line by line and re-typed into the tree:**
  - `include/shulib/motion/tank_steer.hpp` — NEW leaf: `TankLineError`, `tankLineError`, `tankSteerAngle`,
    `tankLegEngages`, `TankSteerStatus`, `kTankSteerNoiseFraction` (header: the line, the bend, the settle bound).
  - `motion/motion_config.hpp` — `TankSteerConfig` (five fields + `valid()`), `MotionConfig::tankSteer` appended
    LAST, `validate()` refuses `!tankSteer.valid()`; `validate()`'s doc now says it descends into `tankSteer` (and
    why, unlike SettleConfig/OdoStallCheckConfig); the file header names HA-179.
  - `motion/move_to_pose.hpp` — `PoseMotionOptions::tankSteer` (default false, only the public constructor sets
    it); the gate in the protected constructor (`strafeAuthority() == 0.0` exactly); the engage decision + one
    Warn at the first live tick; the bent heading loop + the yaw observer reading `loc.twist().omega()`;
    `lastOmegaCmd_ = cmd.body.omega()` after the pipeline, engaged only; `r.tankSteerActive`;
    `tankSteerStatus()`; `start()` re-arms all of it. **Comments rewritten (§1.3 item 8):** the thesis block
    (two sanctioned exceptions, what TANK1 is not), pipeline step 3 (the bent setpoint on an engaged tank leg),
    the pipeline comment (SFB on an engaged tank leg).
  - `diag/debug_record.hpp` — `tankSteerActive` appended after `tickPhase`.
  - `diag/term_sink.hpp` — ` TSTR` after ` SFB`; the header's flag list and `emit()`'s doc name it and the order.
  - `diag/blackbox_format.hpp` — tick flags bit 2 encoded/decoded; the layout comment says what bit 2 is and why
    `kFormatVersion` stays 1; `kFormatVersion`'s doc says a meaning given to a 0-written bit is not a bump.
  - `version.hpp` — `kApiMinor` 3, `"2.3"` (D1 (a)), the comment giving the ruling and the three effects.
- **First evidence on the built code** (probes under `.claude/coordinator/scratch/tank1-a/`):
  - P10 (a) re-run on the change (`p10_post.txt`): **all twelve hashes identical to `p10_pre.txt`** (X/H clean and
    hostile; the tank OFF switch with and without log lines; the every-verb tank script at `maxSteer = 0`, clean
    and hostile; the acceptance run at `maxSteer = 0`, three sides/scales). Re-run again at the end (below).
  - The brief's `probe_proposed.cpp` rebuilt against the worktree with `-DTANK1` (`pp_built.txt`): byte-identical
    to the brief's `recheck/pp_fix.txt` once Part B's ` xt` column is stripped from the brief's copy.

### 22:07 — RESUMED after the session was cut off *(`date`: Sun Sep 27 10:07:56 PM EDT 2026)*
- The previous run stopped after the 12:46 entry above (the session ended ~12:52). Nothing between 12:46 and
  now is known to this log; the team lead said not to worry about the gap ("dont worry about the timeskip").
- **Checked what the tree holds, not trusted:** `git status` — 12 modified files + 3 untracked
  (`tank_steer.hpp`, `tank_steer_test.cpp`, this log); `git diff --stat -- src/` empty. The six Part A library
  headers diffed against the brief's `proposed/` overlay **with comments stripped**: code identical except two
  trailing-whitespace/comment positions (`move_to_pose.hpp`); Part B's pieces (`motion_result.hpp`,
  `motion_scheduler.hpp`, `run_reporter.hpp`) correctly NOT applied. `p10_pre.txt` vs `p10_post.txt`: identical.
- **Already in the tree (kept):** the library code (entry above); `test/tank_steer_test.cpp` (U1–U7, complete);
  §2.3's record tests (`debug_record_test.cpp` default + last-field, `blackbox_format_test.cpp` per-field case +
  round trip + bit-2 case, `term_sink_test.cpp` order golden + TSTR alone); §2.6's pin edit
  (`boot_source_test.cpp`: 13 members, the `TankSteerConfig` 5-member pin, 408 B / 456 B, five `REQUIRE`s); the
  `robot_builder_test.cpp` comment; `version.hpp` 2.3; the briefing regenerated (the TANK1 open-chunk line).
- **Not yet done (this run):** `test/motion_tank_steer_test.cpp` (P1–P14, §2.2), the suite on the change,
  §2.5's mutations, every document of §3, HA-179/180, the changelog, the prose review, the gates, the
  final P10 (a) re-run.

### 22:08–22:30 — the suite on the change, the plant tests, one finding *(`date` 10:30:44 PM at this entry)*
- **Suite on the resumed tree** (briefing regenerated, fresh `cmake -S test -B build/test`): the first build
  STOPPED at the doc gates — `docs/api` was stale (the previous run never regenerated it). `python3
  tools/api_doc_tool.py generate` (new `docs/api/tank_steer.md`; `motion_config`, `move_to_pose`, `debug_record`,
  `term_sink`, `blackbox_format`, `version` pages, `all-entities.md`, `README.md`) → rebuilt, `Linking CXX
  executable shulib_tests` → `1699 | 1699 passed | 0 failed | 3 skipped`, `2549953` assertions: the baseline's
  1685 + U1–U7 (10) + §2.3's four; **no existing test's code needed a change beyond §2.6's pins** (the brief's
  prediction holds on the tree).
- **Measured the built code first** (`.claude/coordinator/scratch/tank1-a/measure.cpp`, `measure` output): every
  number the brief quotes reproduced — P1 1.83/1.85/1.89/2.04/2.57 s (L ×0.91 1.89), OFF y −0.93/−1.56/−2.86/−4.87/
  −8.31; P2 1.89 s xt ±0.02, x monotone; P3 R ×0.95 5.78 s END (0.301, 0.074), R ×0.91 6.14 s (0.369, 0.080), L ×0.95
  5.78 s (0.301, 0.048), L ×0.91 6.15 s (0.356, 0.046), heading vs the leg ≤ 0.0033 rad, ×0.65 both sides stop at
  step 1, END (23.370, ∓0.560), ONE MotionTimeout fault; P4 lat 2/6 1.75/1.80 s, lat 12 TIMEOUT xt −5.10 peak
  heading 14.26°; P5 ±14.9° 1.81 s (clean) / 1.88, 1.93 s (×0.91), ±15.1° = OFF + 1 Warn; P6 (0,24,0) engaged, 0°,
  no motion; P7 hostility (even) 14 of 2 125 records (0.66 %, worst single leg 1.46 %); P8 (a) peak 1.500, (c) 30
  dropout ticks, change 0, held −0.500, (d) peak 0.028, (e) 2.39 s peak 0.755 rad/s 2.68°; P11 authority 0,
  lat 2 1.75 s; P12 1.85/1.89 s, |xt| ≤ 0.019; P13 kA 0.02: ×0.91 1.45 s xt 0.22, 2 flips; lat 2 1.33 s; lat 6
  TIMEOUT along 0.00 xt +0.56 peak 15.13°, 8 flips; kA 0.03: ×0.91 2.09 s, 3 flips.
- **`test/motion_tank_steer_test.cpp` written** — 20 cases, P1–P14 (P10 (b) in CI; P10 (a) stays the executor
  check). The oracle for P1's off switch is the pre-TANK1 ENGINE itself: a test subclass reaching `MoveToPose`'s
  protected constructor with the default `PoseMotionOptions` (tankSteer false, the path StrafeTo/HoldPose use) —
  maxSteer = 0 must equal it in every record byte (via `blackbox::encodeTick`), log line, truth sample and volt.
  Suite: **`1719 | 1719 passed | 0 failed | 3 skipped`, `2646593` assertions.**
- **FINDING (for the coordinator; pre-existing, D4/IMU1's): the LOCALIZER itself reads the IMU rate twice at every
  motion boundary.** P14's acceptance-run case first failed with 3 second reads where the brief (and its probe,
  which printed second reads only for single legs) expected 0. `gdb` on a scratch probe
  (`.claude/coordinator/scratch/tank1-a/second_reads.cpp`): all three are `Localizer::update` (`localizer.hpp`
  ~:364) called from `MotionScheduler::tickBody` (~:1016) on the FIRST tick of the next blocking verb, at the same
  clock instant as the previous motion's exit tick (no pace between two blocking verbs). On a robot (µs clock) that
  read is ~0 or a spike, so `loc.twist()` on the first tick of every chained motion is wrong — a seventh reader for
  IMU1's list. **TANK1 is immune by construction**: the observer runs only once `hasOmegaCmd_` is set, i.e. never
  on a motion's first tick; P14 now pins exactly that (the three second reads happen AT the three boundary
  instants and nowhere else; per-tick truth and volts bit-identical to the plain IMU in zero and spike modes).

### 22:31–22:50 — the mutations (first pass 22:34–22:44), first pass THROWN OUT (a stale-binary defect in my own driver) *(`date` 10:50:35 PM at this entry)*
- Driver: `.claude/coordinator/scratch/tank1-a/mutate.py` (evidence tooling) — per mutation: back the file up under
  `scratch/tank1-a/mut/<id>/`, apply an exact one-match replacement, touch every test TU whose depfile names the
  mutated header, `cmake --build build/test --target shulib_tests/fast -j8`, REQUIRE `Linking CXX executable
  shulib_tests`, run the WHOLE suite, list the failing cases (a first parser counted MESSAGE blocks as failures —
  89 "red" for M1 against doctest's 25 — fixed to count only blocks with an ERROR), restore, sha256-check.
- **The first pass (30 mutations) is NOT evidence and is kept only as a record** (`scratch/tank1-a/mut_pass1_contaminated/`):
  the restore used `shutil.copy2`, which puts the backup's OLD mtime back on the header, so every object compiled
  from the mutant looked up to date to make, and the NEXT mutation of a different header linked the previous
  mutant's inline functions (ODR — the test build is `-O0`, every TU emits its own copy). Seen in the output, not
  guessed: M9 and M24 red on U6 exactly as M8 is (`tankLegEngages(maxSteer)` false — M8's `<`); M21a and M22a red on
  P1/P7 with `tstr == 0` — M20's dropped record write; M23/M24/M26/M28 red on the blackbox round trips — M22b's
  missing decode. HANDOFF's stale-binary recipe warned of exactly this; the depfile touch did not cover a
  header RESTORED with an old timestamp. **Fix:** the restore now bumps the header's mtime (`os.utime`), so the
  next build recompiles every dependent of the restored file; before the second pass all five mutated headers
  were touched, rebuilt (`Linking …`) and the suite re-run green (1719/1719). Second pass running.

### 22:50–22:56 — the mutations, SECOND pass (the evidence) *(`date` 10:55:40 PM when it finished)*
- Every run: a FRESH binary (`Linking CXX executable shulib_tests` required and seen, all 30), the whole suite,
  the file restored and sha256-checked, its mtime bumped. Output: `.claude/coordinator/scratch/tank1-a/mut/<id>/`
  (`result.txt`, `suite.txt`), summary `mut/all.txt`. Baseline green before (1719/1719) and after (below).
  Cases named are the TANK1 ones plus any existing test that also went red; counts are doctest's failed cases.

| # | mutation | red (failed cases) — the brief's target first |
|---|---|---|
| M1 | bend sign flipped | 25: **U2**, **P1** (both), U3, P2–P5, P7, P8 (b)–(e), P11–P14; + existing D1 routine/sugar, C4 tank routine/auton, cookbook-04a, C3 floor, G2 runner 4, C5 truth tank |
| M2 | direction from `abs(along)` | 2: **U3**, **P2** |
| M3 | bend clamp removed | 3: **U4**, **P4**, P13 |
| M4 | fade removed | 5: **U5** (both), **P4**, P6, P13 |
| M5 | direction unclamped | 13: **U4**, U2, U3, U5, P1 (both), P2–P4, P11–P13, C3 floor |
| M6 | eligibility `< 0.5` | 2: **P10 (b)**, P11 |
| M7 | eligibility by `dynamic_cast<TankKinematics>` | 1: **P11** |
| M8 | window `<` | 1: **U6** |
| M9 | latch removed | 3: **P5**, **P6**, G2 runner 4 (the path runner's un-faced tank leg now engages — D2 (b)'s predicted change) |
| M10a | Warn every tick | 2: **P5**, P6 |
| M10b | Warn never | 2: **P5**, P6 |
| M11 | observer sign flipped | 14: **P1** (both), P2–P5, P7, P8 (b)–(e), P12–P14 |
| M12 | observer limit clamp removed | 2: **P8 (a)**, P1 |
| M13 | observer not reset in `start()` | 1: **P8 (b)** |
| M14 | observer ignores `isReady()` | 1: **P8 (c)** |
| M15a | reference = last tick's PRE-clamp ω | 1: **P8 (d)** only — as the brief predicted (equivalent wherever the 6 rad/s clamp does not bind) |
| M15b | reference = this tick's pre-pipeline ω | 12: **P1** (both), P2, P4, P5, P8 (b)–(e), P12–P14 |
| M16 | `lastOmegaCmd_` written on every motion | **0 — GREEN, an equivalent mutant, as the brief predicted**: the observer runs only on an ENGAGED leg, whose ticks already write it; on a holonomic or un-engaged leg nothing ever reads it (`hasOmegaCmd_`/`lastOmegaCmd_` are consumed only inside `if (tankEngaged_)`), so writing it there changes no output. P10 (b) stays green, which is the point. |
| M17 | `PoseMotionOptions::tankSteer` default true | 2: **P9**, P1 off switch (the UnsteeredMoveToPose oracle steers too) |
| M18 | flag floor removed (`> 0.0`) | 2: **P7** (the HOSTILITY clause: `lit·50 ≤ records` → 105 450 ≤ 2 125, i.e. 2 109 of 2 125 ticks lit against 14), P8 (b) (the second run's first record lights) |
| M19 | flag never set | 2: **P7**, P1 |
| M20 | record field not written | 2: **P7**, P1 |
| M21a | TermSink token missing | 3: **both §2.3 goldens**, P7 |
| M21b | TermSink token after CLMP | 1: **the order golden** |
| M22a | blackbox bit 2 not encoded | 5: **the per-field case**, **round trip**, the bit-2 case, the file round trip, the triage frame |
| M22b | blackbox bit 2 not decoded | 4: **round trip**, the bit-2 case, the file round trip, the triage frame |
| M23 | the bend aims at the POINT | 26: **P6**, P1–P5, P7, P8 (d)/(e), P11, P13, P14; + existing D1 sugar, C4 tank, B12/B13, C1/B18, cookbook-04a, guide-09c, guide-10b, C1 tank along-axis + sweep, MotorGroup 2/7, C5 truth tank |
| M24 | `maxSteer = 0` does not disable the observer | 3: **P1 off switch**, P5, P8 (e) |
| M26 | (existing) C3's SFB floor removed, TANK1 in | 2: **`tank: sub-perceptible lateral demand does NOT light the fallback flag`** (still red with TANK1 in — §0.5 item 4 holds on the tree), X-drive never falls back |
| M28 | observer reads `ctx.imu().yawRate()` | 1: **P14** |

- M25/M27/M29 are Part B's — not run (Part B not built).
- **After the last restore:** headers re-touched by the driver, the full build WITH the doc gates relinked
  (`[ 30%] Linking CXX executable shulib_tests`) and the suite is **1719 | 1719 passed | 0 failed | 3 skipped,
  2646593 assertions** (run at 22:58).

### 22:45–23:00 — second measurement probe; documents; gates *(`date` 11:01 PM at this entry)*
- **Probes on a SNAPSHOT of the clean headers** while pass 2 ran (`scratch/tank1-a/snap/`, `diff -r` identical
  to the worktree's `include/` when taken; `bsnap.sh`): `measure2` — lag: kA 0.02 (τ ≈ 0.12 s) R ×0.91/0.85/0.80/
  0.75 all SETTLE (1.45/1.54/1.67/1.97 s); kA 0.03 (τ ≈ 0.18 s) ×0.91 SETTLES 2.09 s, **×0.85 TIMES OUT (xt +0.60)**,
  ×0.80 and ×0.75 SETTLE (2.53/2.34 s) — not monotone, said so in HA-179 and guide 14; pinned 0.3–1.3 s: 3.27 s,
  correction at its 1.500 limit, 3.67°; lat 6 pinned 0.3–0.8 s: TIMEOUT, peak heading 17.42°, bend-only SETTLES
  2.29 s; the acceptance run at ×1.00 with `maxSteer = 0`: END (0.244, 0.123, 180.00°) 5.42 s — exactly the
  brief's pre-TANK1 numbers — and with TANK1 (0.229, 0.050, −179.97°) 5.44 s. The brief's `recheck/probe_imu2.cpp`
  rebuilt on the snapshot: output **byte-identical to the brief's `imu2_fix.txt`** (plain/zero/spike 15/15; with
  the rate-reading corrector: zeros 15/15, zeros+spikes 8/15). D2 (b)'s cliff re-measured on a no-window copy of
  the snapshot (`snap_nolatch/`, the latch line replaced): un-faced 40 in arrives at 20°/30° (1.82/1.97 s), times out
  from 40°; the built code (window) times out at every angle 20°–120°, today's behaviour.
- **Documents** (Part A, §3): see the file list in the final entry. Header comments: `chassis.hpp` (the
  TURN-WHILE-DRIVE item's tank sentence replaced by a pointer, a new TANK STEERING item; `moveTo`'s and
  `strafeTo`'s docs), `routine.hpp` (the field-vocabulary paragraph; `driveTo`'s doc), `strafe_to.hpp` (the
  drivetrain-honesty paragraph; the class doc), `tank_steer.hpp` (the floor's doc now quotes P7's measured counts:
  14 of 2 125, worst leg 1.5 %, 2 109 without the floor, 82 of 186 at ×0.95). `command_pipeline.hpp` READ (its SFB
  notes ~:25-33, ~:71-81 are still true on a tank) — not edited. `docs/api` regenerated (`chassis`, `routine`,
  `strafe_to` pages join the earlier ones).

### 23:04 — prose review (pace rule 4), run BEFORE this entry was written *(`date` 11:04:31 PM just before)*
- **The brief's greps**, over `README.md`, `docs/*.md`, `docs/guide/`, `docs/cookbook/` (and the live internal docs
  `build-order.md`, `HANDOFF`, `PROJECT-BRIEFING.md`, `ORIENTATION.md`, `RESUMING.md` for the tank terms; generated
  `docs/api/` and chunk records excluded), every hit read:
  - `laterally-offset`, `honestly exit` — 2 hits: the changelog QUOTING the replaced sentence; build-order's TANK1
    background (history). The sentence itself is gone from `chassis.hpp` (replaced by TANK STEERING).
  - `sideways target` — guide 04 :91 (still true — a pure sideways target is unreachable; the TANK1 bullet follows
    it), guide 10 :103 (the compiled `guide-10b` listing's comment — still true, not editable without its test),
    guide 10 :343 and guide 12 :74 (edited — the three tank cases told apart).
  - `cannot strafe`, `turn-then-drive` — guide 04 :73, glossary, cookbook 04 :15-16, master plan (LemLib's model;
    C3's H-drive fallback), `legacy-command-vocabulary.md` :101, roadmap/build-order C3 rows: all still true.
  - `plans turns`, `plan a turn`, `insert a turn`, `silently turn`, `the H-drive is running`, `setpoint 0` — 0 hits
    left in live docs (cookbook 04's "never insert a turn … silently turns" and guide 11's "the H-drive is running"
    were rewritten; move_to_pose.hpp's "setpoint 0" paragraph was rewritten in the earlier run).
  - `never plans` — my new sentences (guide 04, changelog), the changelog's 2.2 route note, HANDOFF L2u: consistent.
  - `sideways of you` — guide 01 :44 (drift illustration, unrelated), cookbook 04 :52 (rewritten: no 6 in promise).
  - `simulator` — 30-odd pre-existing hits (README, guides 05/07/08/09/13/14, cookbook 01/05, roadmap, master plan,
    changelog history); **none in anything TANK1 wrote** (`git diff | grep '^+' | grep -i simulat` empty). Not
    rewritten — outside this chunk's scope; noted for the coordinator.
  - `6 in` — every added line checked: the fade distance, or a statement that a 6 in offset is NOT removed with lag
    (guide 14, HA-179), or HA-179's instant-plant measurement labelled as such. No promise.
  - `yawRate()` — HA-179's IMU paragraph and `move_to_pose.hpp`'s comment (both say NEVER a second read).
  - `368`, `416` — 0 hits in docs (the sizes live only in `boot_source_test.cpp`, edited).
  - `MotionConfig` — guide 10 :284 listed the config's contents: **edited** (the `tankSteer` knobs, `maxSteer = 0`).
  - `SFB` — guide 04 :89, guide 10 :93, cookbook 04 :119 (H-drive — still true), guide 11 :159 (**rewritten**),
    roadmap/build-order C3 history.
  - `D12` — build-order :1581 (G2's data layer, history); C1-COMPLETED D12 (the dated amendment note appended).
  - `face(`, `TimedOut`, `strafeAuthority`, `tank` (+ timeout/sideways/lateral/plan/face) — read: guides 05/06 ("tank
    asked to strafe" — still true), README :40 (still true), guide 04 :15-18 (the parallel-parking paragraph —
    still true), guide 09's compiled `guide-09c` listing comments (still true), everything else mine or unrelated.
- **The changed sources' names** (`tank_steer`, `motion_config`, `move_to_pose`, `debug_record`, `term_sink`,
  `blackbox_format`, `strafe_to`, `chassis`, `routine`, `version`): hits read; two more stale spots found and fixed —
  **guide 11's blackbox section** (the reader "refuses a file whose format version it does not recognise": added
  that the format grew bit 2 WITHOUT a version bump, and why an older file reads it as off) and
  **`docs/diagnostics-plan.md`'s F9 field table** (added the pre-freeze `tankSteerActive` append, and that H1's
  decoder should expose bit 2). Glossary: **"tank steering"** added (guide-maintenance: a new term).
- Links in every changed document checked mechanically (relative paths + anchors): 0 bad.

### 23:07 — Part A: final gates, and where it stands *(`date` 11:07:54 PM at this entry)*
- **Host suite** (fresh configure earlier this run; final full build WITH the doc gates relinked `shulib_tests`):
  `[doctest] test cases: 1719 | 1719 passed | 0 failed | 3 skipped`, `assertions: 2646593 | 2646593 passed`.
  Baseline 1685 → 1719: +34 (U1–U7 10, §2.3 4, P1–P14 20). No existing test's CODE changed beyond §2.6's pins
  (`boot_source_test.cpp`); one test COMMENT (`robot_builder_test.cpp` ~:471).
- **P10 (a), the executor pre/post check, re-run on the FINISHED tree** (`scratch/tank1-a/p10_final.txt`, same
  probe, same flags, g++ 13.3.0 `-O2`, x86-64, host `lodgepc`, base `a62067b`): **all twelve hashes identical to
  `p10_pre.txt`** — X/H clean and hostile (s7) every verb with a RunReporter (records + log lines + truth); the
  tank ×0.95 40 in with `maxSteer = 0` (records + truth, and records + log + truth); the every-verb tank script at
  `maxSteer = 0` (×1.00 clean, ×0.91 clean, ×0.91 hostile s7); the acceptance run at `maxSteer = 0` (R ×0.95,
  R ×0.91, L ×0.91). The brief's `probe_proposed.cpp` on the finished tree (`pp_final.txt`) == `pp_built.txt`.
- **`python3 tools/src_build_gate.py self-test`**: `src build gate self-test: OK (189 detector cases, each against a
  real build)`. **`check`**: `src build gate: PASS (all 6 builds link; …)` — bench, xdrive, tank, tank-drive,
  tank-library, tank-calibrate: exit 0, 8/8 TUs, 0 disallowed warnings. `git diff --stat -- src/`: empty.
- **ARM header gate** (RESUMING's command, 192 headers incl. `motion/tank_steer.hpp`): rc 0; each changed header
  alone for the V5 (tank_steer, motion_config, move_to_pose, debug_record, term_sink, blackbox_format, chassis,
  routine, strafe_to, version): all OK.
- **CI greps:** `library is PROS-free (all of include/shulib except hal/pros/)`; `layering holds: core is sim-free`.
- **api_doc_tool:** `self-test: OK`; `doc coverage: 2932 public entities across 156 headers, all documented`;
  `check-fresh` rc 0 (silent); `doc example scan: 467 quoted lines, all verbatim (5 source files); 3 listings of
  whole-run files, each one unbroken run`; `removability: no public doc references docs/internal/`.
- **`doc_staleness_audit.py`**: `clean (27 live docs, 0 numeric claims checked)`.
- **Briefing** regenerated LAST, then `briefing_status.py check`: `generated block current`.
- **Not committed. No `pros` command. No `/dev/ttyACM*`.** Part B (the ` xt` column; M25/M27/M29) NOT started.
  No `TANK1-COMPLETED.md` (the coordinator's, at the merge); `roadmap.md`'s "you are here" and `build-order.md`'s
  `Next:` pointer NOT moved (the coordinator's reconciliation) — only the TANK1 entry's status line was added.
- **For the coordinator:** (1) the Localizer's second IMU read at every chained-motion boundary (the 22:30 finding)
  belongs on IMU1's list; (2) pass 1 of the mutations was contaminated by my driver's restore (22:50 entry) and
  re-run; (3) HA-179's "about 0.18 s: 15 % times out while 20/25 % finish" is non-monotone — worth a look when R5
  measures kA; (4) HA-162..HA-178 are other chunks' numbers — the register's status line now says 158 entries and
  names that gap; reconcile at merge with whichever of T4/COMP-t/2c/SAFE-SAVE lands HA entries; (5) the COMP-t
  re-predictions (§6) can be taken from P1/P3's measured numbers above, unchanged from the brief's table.

---

## Part A review findings and their disposition

- **Started:** Mon Sep 28 01:33:20 AM EDT 2026 (`date`). Same worktree, same uncommitted tree as the 23:07 entry
  (`git status`: 47 paths; `git diff --stat -- src/` empty). The reviewer's copy
  (`.claude/coordinator/scratch/tank1-a/review/wt/`) was diffed against the worktree's `include/`, `test/` and
  `docs/`: identical — its findings are about this tree.
- Ten findings (1 blocker, 2 major, 7 minor). Each is re-checked below on the host test plant before anything is
  changed; probes and backups under `.claude/coordinator/scratch/tank1-a/` (never `/tmp`).

### 01:33 — Finding 1 (blocker) reproduced first
- The reviewer's `rv_arc_pre` / `rv_arc_post` binaries re-run: pristine SETTLED 1.64/1.61/1.59/1.48 s, TANK1
  TIMEDOUT 5.01 s on all four (END y −2.954/−2.335/−3.813/−2.322, along ≈ 0) — as reported.
- **Root cause, traced tick by tick** (`scratch/tank1-a/fix/trace.cpp`, start −12°, target (29.9075, −2.3538, 0°)):
  at the first ticks the bend (−10.6°) almost equals the start heading (−12°), so the heading loop barely turns
  and the robot runs at the line at ~12° and 59 in/s; the heading loop (kP 4: a 0.25 s lag, ~15 in at full speed)
  turns too late, the robot crosses the line at along −16 with ~7° still on, peaks 0.78 in past it, and the
  fade then zeroes the bend while the robot slows: END cross −0.60, along 0.00, TIMEOUT. Not a sign error, not
  an unbounded correction: the **line loop is under-damped** — the bend is a pure P on the cross-track error with
  nothing that sees the robot already closing on the line. (First-order model: ζ = 1/(2·√(crossGain·v·τ_h))
  ≈ 0.41 at 60 in/s.) The same root makes other holes the review did not name — **measured:** faced legs with a
  plain lateral offset (the "lat" grid, `WHAT=lat`) settle at 50 in for 1–3 in and 7–8 in but TIME OUT for 4, 5
  and 6 in; at 60 in, 5–8 in time out while 10 in settles. The reach is not monotone in the offset: the 40 in /
  6 in case P4 pins settles on the right phase of an oscillation.
- **Re-probed (design error ⇒ re-probe):** an overlay of the worktree's headers with a probe switch
  (`scratch/tank1-a/fix/ov/`, `eval.cpp`; evidence only) — four damping forms: (1) steer on the cross-track error
  predicted `T` seconds ahead from the COMMANDED forward speed (last tick's `cmd.body.vx`) and the heading
  relative to the target heading, `cross + T·v·sin(θ − θt)`; (2) the same from the Localizer's measured cross
  rate; (4) from its measured speed; (3) the brief's fixed look-ahead `L` in inches. Graded on: the review's grid
  (`sweep2`: start 0–14° off, BOTH signs, distance 12–60 in, bearing −15…+15° by 0.25°, 24 563 legs — regressions
  = legs that settle with maxSteer = 0 but time out steered), the review's coarse grid (`sweep1`, 2 450 legs incl.
  reverse, target headings ±14°, a 3 % weak side), the brief's core/stretch set, the lateral reach grid, P-case
  numbers, hostility — at plant kA 0 / 0.01 / 0.02 / 0.03 / 0.04.

| law | sweep2 regressions kA 0 / 0.02 / 0.03 | sweep1 regressions | lat 6 @40 kA 0 / 0.02 / 0.03 |
|---|---|---|---|
| as built (no lead) | **178** / 122 / 16 (worst \|xt\| 0.70) | 15 | S / T / T |
| (1) T 0.03 | 44 / 0 / 0 | — | — |
| (1) T 0.05 | 0 / 0 / 0 | 0 | S −0.33 / S / S |
| (1) T 0.07 | 0 / 0 / 0 | 0 | S −0.40 / S / S |
| (1) T 0.10 | 0 / 0 / 0 | 0 | **T −0.52** / S / S |
| (1) T 0.15 | 0 / 0 / 0 | 0 | T / S / T |
| (1) T 0.25 | 0 / 72 / 102 | — | — |
| (2) measured cross rate, T 0.15 | 0 / 0 / 0 | — | — |
| (3) look-ahead L 2 / 4 / 6 in | 20 / 0 / 0 ; 0 / 0 / 0 ; 0 / 4 / 20 | L 4: 0 | L 4: T / S / S |
- **Chosen: form (1), the lead on the COMMANDED speed, `leadTime` 0.06 s** — a new sixth `TankSteerConfig`
  field (PROVISIONAL, HA-179), default 0.06. Why this form: it reads no sensor (last tick's `cmd.body.vx`, the
  same command the observer compares against, and this tick's estimate heading), so P14's one-IMU-read
  property holds and a GPS/tag nudge cannot spike it (the measured forms (2)/(4) difference the fused pose);
  a time, not an inch look-ahead, because the heading lag it compensates is a time (form (3) is too weak at
  full speed, too strong slow). Why 0.06: zero regressions on the review's grid at every lead measured from
  0.05 to 0.10 s at plant kA 0, 0.01, 0.02 and 0.03 (0.05/0.06/0.07/0.085/0.10 each run; 0.04 s: 4 at kA 0;
  0.15 s: 0 at kA 0–0.03; 0.20: 32 at kA 0.03; 0.25: 72/102 at kA 0.02/0.03); inside that band 0.06 balances
  the two stretch margins that pull opposite ways (lateral 6 in at 40 in with a 9 % weak side on the lag-free
  plant: xt −0.41/−0.44/−0.47 at 0.05/0.06/0.07; the 15 % weak 40 in at kA 0.03: xt 0.49/0.47/0.44).
  **Nothing ruled moved:** the 15° cap and window (D2), the observer, the Warn, the arrival rule, the off
  switch; nothing is computed from the target point; X/H untouched (engaged tank legs only). `leadTime = 0`
  is the Part A law bit for bit (the review grid at lead 0 reproduces the 178 regressions exactly).
- **Measured on the WORKTREE build** (`fix/measure3.cpp`, identical row for row to the overlay model at kA 0 and
  0.02 — `fix/out/wt_vs_ov_*.txt`, 0 rows differ): review grid (`sweep2`) regressions **0 / 0 / 0 / 0** at kA
  0 / 0.01 / 0.02 / 0.03 (improvements 9 658 / 11 960 / 14 742 / 17 226 against 8 510 / 9 684 / 11 690 /
  13 916 with no lead); at kA 0.04 (τ ≈ 0.24 s, where the brief already said the heading loop's own kP 4 is the
  limit) 24 legs regress with the lead against 18 without — all at 48 in, starts 10–14° off (listed,
  `fix/out/s2list_*`); coarse grid (`sweep1`) regressions 0 (15 without), slower legs 0, improvements 550
  (493); lateral reach (`lat`, faced, clean): 40 in settles 1–6 in, 50 in 1–8 in, 60 in 1–10 in — monotone,
  the holes gone. Core cases: P1's drift legs unchanged to 0.01 s (1.83/1.85/1.89/2.04/2.57 s), peak heading
  excursion down (×0.91: 1.88° → 1.55°); lateral 6 in at 40 in 1.81 s xt −0.36 (was 1.80 s −0.16); with lag
  kA 0.02 lateral 6 in now SETTLES (1.45 s — it timed out 0.56 in beside the target), and kA 0.03 the 15 %
  weak 40 in SETTLES (2.42 s — it timed out); the pinned lateral-6 leg (HA-179's 17.4° overshoot) now settles
  (2.31 s, peak 16.5°); composed hostility even drive TSTR 20 of 2 125 ticks (was 14), 10/10 settle.
- **What moved in the tests because of it:** P13 (c) — its own comment said "if (c) ever SETTLES, update guide
  14, the cookbook and HA-179, then flip it": flipped (settles 1.45 s, 2 yaw flips near the target, was 8), and
  a new measured limit (e) pinned (lateral 10 in at 40 in with lag: TimedOut along 0.00, 1.59 in off, heading
  within 15.09°). P8 (d) moved to 50 in (with a 0.6 rad/s turn budget a 6 in offset over 40 in now ends 0.63 in
  short — it settled only on the right phase of the old oscillation); M15a still separates it (peak 0.029 vs
  0.482 rad/s, `fix/p8d.cpp` on an M15a copy). Every quoted "measured" number re-measured on an instrumented
  copy of the test file (`fix/tmeas/inst.py` → `mts`, 22/22 pass) and updated. `boot_source_test.cpp`: 6
  `TankSteerConfig` members, `MotionConfig` 416 B, `ChassisConfig` 464 B, the `leadTime` REQUIRE.
- **Suite** (briefing + docs/api regenerated, full build with the doc gates, `Linking CXX executable
  shulib_tests`): **1724 | 1724 passed | 0 failed | 3 skipped, 2 648 009 assertions** (1719 + U8 ×3 + P15 +
  P16).

### Findings 2–5 (tests) — each re-checked, each fixed in the suite
- **2 (major) P8 (a) one-sided — REAL.** `rv_cap_orig`/`rv_cap_s17b` re-run: L ×0.50 peaks 4.493 rad/s under
  S17b, 1.500 on the real code. P8 (a) now runs BOTH weak sides and requires each bound reached by its own
  sign (R: −1.500, L: +1.500).
- **3 (major) TSTR's bend share untested — REAL.** P4's clean ±2/±6 in legs now require TSTR lit (measured 37
  and 91 of 181 ticks; bound half that) AND the peak correction under half the floor (0.022 / 0.043 rad/s
  against 0.12) — so only the bend's share can light it there.
- **4 (minor) authority threshold — REAL.** P11 adds H-drives with strafe derate 0.01, 1e-9 and
  `denorm_min()`: authority > 0, not eligible, not engaged.
- **5 (minor) status cross/bend never read — REAL.** New P16: on every commanding tick of a clean lateral 2 in
  leg, `cross` equals the line error of the pose the motion read, bit for bit, and `bend` equals
  `tankSteerAngle` of the predicted cross, bit for bit; the first tick is exactly cross −2, crossAhead −2,
  bend +0.2 rad. (Found writing it: on the EXIT tick the status still describes the tick before — the exit
  verdict returns before the heading loop; the doc now says so.) `TankSteerStatus` gained `crossAhead` (the
  error the bend steered on) — Tier-3, uncommitted, additive.
- Mutation pass 3 (every Part A mutation re-run on the changed code + the review's S1–S7/S17a/S17b + the
  lead's M30–M34) started 10:19:45 — results in the next entry.

### Mutation pass 3 (10:19–10:31) and what it found in my own tests
- `scratch/tank1-a/mutate2.py` (the second-pass driver, same mechanics: backup, one exact replacement, touch
  every dependent TU, `--target shulib_tests/fast`, REQUIRE `Linking CXX executable shulib_tests`, whole suite,
  restore + sha256 check + mtime bump), 44 mutations, output `scratch/tank1-a/mut2/`: **44/44 linked fresh,
  44/44 restored sha256-ok.** Every Part A mutation of the table above still red (M16 still GREEN — equivalent,
  now also writing `lastVxCmd_` on motions that never read it); M30 (lead removed) red on P13/P15/P16; M31
  (lead sign) red on U8 ×2, P4, P8 (d), P13, P15; M32 (lead memory not re-armed) red on P8 (b); M33 (heading the
  wrong way round) red on P4, P8 (d), P13, P15; the review's S1 red on P4, S2 on P11, S3 on P5/P8 (b), S4 on
  P16, S5 on P1, S7 on P1, S17a on P1/P3/P8 (a), S17b on P3/P8 (a).
- **Two survivors, both real gaps, both closed:**
  - **M34** (the lead's speed taken unsigned: a reverse leg predicted as if it drove forward) — GREEN: no test
    backed up with an offset. Probed (`fix/rev.cpp` on an M34 copy): reverse lateral 4 in at −40 in and 6 in
    at −50 in time out. **P15 (c) now runs the 50 in reach both ways** (forward and backing up, 1..8 in —
    measured identical, mirror images); under M34 3–7 in time out 0.61–0.90 in off. Re-run on a fresh binary:
    RED (P15).
  - **S6** (the observer runs before the first command, `hasOmegaCmd_` ignored — the review ran it, it
    survived there too, and it was not among the findings) — GREEN. Traced: a motion's very first tick has dt 0,
    which already stops the observer (that covers chained motions); but a fresh Localizer is live only from its
    SECOND update (`fix/boot.cpp`: quality 0 then 1), so a motion that waited reaches its first live tick with
    dt > 0 and nothing commanded — where S6 learns whatever rate the robot has. Every other case starts at rest
    (rate 0), so it was equivalent there. **New P8 (f):** the robot pushed round (±2 V, a `TankWorld` spin
    window) from the start; at the first live tick the Localizer reads 0.78 rad/s after 1 waiting tick, and the
    correction must be exactly 0 (S6: 0.071). First attempt used an IMU-boot dropout to force waiting ticks —
    the window, set after the harness had already published "ready", never took effect (waited 1, not ≥ 5);
    rewritten on the Localizer's own first-update wait. Re-run on a fresh binary: RED (P8 (f)).
- Suite after both: **1725 | 1725 passed** (P8 (f) added).

### Findings 6–10 (documents, source label, register) — each re-checked, each fixed
- **6 (minor) overstatements — REAL, all three.** guide 10 :92-94 said a tank "simply cannot reach a target that
  requires net sideways motion" (grep for "net sideways motion": that one hit, now gone) → a target it cannot
  reach by driving along its heading (beside it, or farther off its line than the steering removes over the
  leg) times out. guide 04 :91-94 "a sideways target is physically unreachable … honestly reports TimedOut" →
  the same wording. cookbook 04 :41-43 "bend its heading up to 15° … and never more" → the heading it AIMS for,
  with a parenthesis that the robot's actual heading can pass 15° by a degree or two (measured: 15.1–17.0° with
  lag, 16.5° after a pin). Also re-worded on the way (the lead changed the numbers): guide 10's strafeTo
  paragraph and cookbook's "Watch out for" ("a couple of inches … motor lag shrinks that" → a few inches over
  a 40 in leg, less over a short one, on a robot it depends on unmeasured lag); the changelog's "how motor lag
  shrinks it" → "what motor lag does to it". The compiled `guide-10b` listing comment ("a sideways target is
  physically unreachable") is still true of a point beside the robot and is verbatim from its test — kept.
- **7 (minor) the derate-0 H-drive exception — REAL.** Added to the changelog headline, guide 04's tank
  bullet, `version.hpp`'s 2.3 note and `chassis.hpp`'s TANK STEERING item.
- **8 (minor) guide 11's SFB text — REAL.** Re-measured on the final tree (`fix/sign_final.cpp`, the review's
  `rv_sign.cpp` re-run): a leg ON its line from 10° off lights SFB on 37 records, from −14° on 77, the first
  at record 2 with cross 0.000; faced, 0. Rewritten: on a tank SFB lights whenever the target POINT is not
  straight ahead by more than a hair — off its line, or started turned off its heading (even on its line), or
  the steering turning it. The ×0.91 count re-measured with the lead: 39 SFB records (498 with TANK1 off).
- **9 (minor) HA-179's source had no HA-179 label — REAL.** `grep -rn HA-179 include` found only
  `motion_config.hpp`. The tank-path comment in `move_to_pose.hpp` now says "Every constant this path reads is
  MotionConfig::tankSteer's, PROVISIONAL (A4: HA-179)" (the register's direction-1 grep form), and HA-179's
  Source says "(the tank path, labelled HA-179)". Direction 1 re-run: `grep -rn "PROVISIONAL (A4" include/
  test/ | grep -v "HA-[0-9]"` — empty.
- **10 (minor) the rate's sign — REAL.** Re-measured on the final tree: a negated rate on the ×0.91 40 in,
  TANK1 on: TIMEOUT, peak heading 15.68°, correction pinned at 1.500 rad/s; off: the usual TIMEOUT at y −2.86.
  HA-179's blast radius gained "The rate's SIGN" (the first control law that closes a loop on it; HA-04's
  `get_gyro_rate().z` sign is undocumented; robot two's differentiated rotation has it by construction); HA-04's
  blast radius gained a pointer to it. `move_to_pose.hpp`'s tank-path comment names the dependency too.

### 11:04–11:15 — mutation pass 4, the FINAL code and tests (the evidence for this round)
- `scratch/tank1-a/mutate3.py` (= `mutate2.py`, output `scratch/tank1-a/mut3/`), started on the finished code
  after the suite was green (1725/1725): **44 mutations, 44 fresh links (`Linking CXX executable
  shulib_tests`), 44 restores sha256-ok; 43 RED, 1 GREEN (M16, the documented equivalent mutant).** Red cases
  (TANK1 ones; existing suites that also went red are in `mut3/all.txt`):

| # | mutation | red |
|---|---|---|
| M1 | bend sign | 27 incl. U2, U3, P1–P5, P7, P8 ×4, P11–P16, D1/C4/C5/G2/cookbook-04a/C3-floor |
| M2 | direction from \|along\| | U3, P2, P15 |
| M3 | bend clamp removed | U4, P4, P13, P15 |
| M4 | fade removed | U5 ×2, U8, P4, P6, P15 |
| M5 | direction unclamped | 13 incl. U2–U5, P1–P4, P7, P8, P13, P15, P16 |
| M6 / M7 | eligibility `< 0.5` / by type | P10 (b), P11 / P11 |
| M8 | window `<` | U6 |
| M9 | latch removed | P5, P6, G2 runner 4 |
| M10a / M10b | Warn every tick / never | P5, P6 / P5, P6 |
| M11 | observer sign | 13 incl. P1 ×2, P2–P5, P7, P8 ×3, P12–P14 |
| M12 | observer clamp removed | P1, P3, P8 (a) |
| M13 / M14 | not reset / ignores isReady | P8 (b) / P8 (c) |
| M15a / M15b | pre-clamp ω / this tick's ω | P8 (d) / 10 incl. P1 ×2, P2, P4, P5, P8 ×2, P12–P14 |
| M16 | `lastOmegaCmd_`/`lastVxCmd_` on every motion | **GREEN — equivalent** (read only on engaged legs) |
| M17 | `tankSteer` default true | P1, P9 |
| M18 / M19 / M20 | floor removed / never set / not written | P7, P8 / P1, P4, P7 / P1, P4, P7 |
| M21a / M21b | TermSink token missing / order | P7 + 2 TermSink goldens / the order golden |
| M22a / M22b | blackbox bit 2 not encoded / decoded | 5 / 4 blackbox cases |
| M23 | bend aims at the POINT | 28 incl. P1–P8, P11, P13–P16 |
| M24 | maxSteer 0 keeps the observer | P1, P5, P8 |
| M26 | (existing) C3 SFB floor removed | the C3 floor case, X-drive never falls back |
| M28 | observer reads `ctx.imu().yawRate()` | P14 |
| M30 | the lead removed (Part A as first built) | P13, P15, P16 |
| M31 | the lead's sign | U8 ×2, P4, P8 (d), P13, P15 |
| M32 | the lead's memory not re-armed | P8 (b) |
| M33 | the lead's heading the wrong way round | P4, P8 (d), P13, P15 |
| M34 | the lead's speed unsigned | P15 |
| S1 (review) | TSTR bend share dropped | P4 |
| S2 (review) | eligibility `< 0.3` | P11 |
| S3 (review) | engage decided in `start()` | P5, P8 (b) |
| S4 (review) | status cross/bend not written | P16 |
| S5 / S7 (review) | floor ×2 / observer τ ×2 | P1 / P1 |
| S6 (review) | observer before the first command | P8 (f) |
| S17a / S17b (review) | one-sided observer clamp | P1, P3, P8 (a) / P3, P8 (a) |

### 11:15–11:28 — the final tree: suite, hashes, gates, prose review *(`date` 11:28:53 AM at this entry)*
- **Suite** (the final comment-only reflows in `tank_steer.hpp` and the two test files made after pass 4; then
  `--target shulib_tests/fast` → `Linking CXX executable shulib_tests`, docs/api + briefing regenerated, the
  full build WITH the doc gates): **`test cases: 1725 | 1725 passed | 0 failed | 3 skipped`, `assertions:
  2648032 | 2648032 passed`.** Baseline 1685 → 1725: +40 (U1–U8 13, §2.3's 4, P1–P16 23). No existing test's
  code changed beyond §2.6's pins (`boot_source_test.cpp`: 6 `TankSteerConfig` members, 416 B / 464 B, six
  `REQUIRE`s).
- **P10 (a) on the final tree** (`scratch/tank1-a/p10_final2.txt`, `p10_hash.cpp` built with `-DTANK1`, g++ 13.3.0
  `-O2`, x86-64, host `lodgepc`, base `a62067b`): **all twelve hashes identical to `p10_pre.txt`** (X/H clean
  and hostile; the tank ×0.95 off switch with and without the log; the every-verb tank script at `maxSteer = 0`,
  clean and hostile; the acceptance run at `maxSteer = 0`, three sides/scales). (A first run this round was
  built WITHOUT `-DTANK1` by my mistake — so its tank scripts ran steered and differed, as they must; not
  evidence, re-run with the flag.) `leadTime` is read only on an engaged leg, so none of the twelve can see it.
- **`src_build_gate.py self-test`**: OK (189 detector cases). **`check`**: PASS, all 6 builds link, 8/8 TUs each,
  0 disallowed warnings. `git diff -- src/`: 0 lines.
- **ARM header gate** (192 headers, RESUMING's flags): rc 0; `tank_steer.hpp`, `motion_config.hpp`,
  `move_to_pose.hpp`, `chassis.hpp`, `version.hpp` each alone: OK.
- **CI greps:** PROS-free; core sim-free. **api_doc_tool** self-test / check-coverage (2 935 entities, 156
  headers) / check-fresh / check-examples (467 lines) / check-removability: all rc 0. **doc_staleness_audit**:
  clean, rc 0. **Briefing** regenerated, then `briefing_status.py check`: current, rc 0.
- **Register:** direction 1 (`grep -rn "PROVISIONAL (A4" include/ test/ | grep -v "HA-[0-9]"`) empty; HA-179's
  two Source files both carry "HA-179" now.
- **Prose review (pace rule 4), grepped and read BEFORE this entry:** over `README.md`, `docs/*.md`,
  `docs/guide/`, `docs/cookbook/` (+ build-order, HANDOFF, PROJECT-BRIEFING, C1-COMPLETED for the stale
  numbers): the review's phrases — `net sideways motion` (0 left), `physically unreachable` (guide 10 :104, the
  compiled `guide-10b` comment, still true of a point beside the robot; `strafe_to.hpp` :36, strafeTo — true),
  `and never more` (cookbook 43, now qualified), `byte for byte`/`byte-for-byte`/`are untouched` (the TANK1 ones
  now carry the derate-0 exception; the rest are other chunks'), `more than a hair` (guide 11, rewritten);
  the old TANK1 numbers — `0.56`, `half an inch`, `17.4`, `17°`, `15 % weak`, `monotone`/`orderly`, `41 ticks`,
  `14 of 2`, `82 of`, `0.66 %`, `0.7 %`, `1.5 %`, `(0.23, 0.05`, `0.755`, `2.68`, `3.27`, `1.33 s`, `couple of
  inches`, `motor lag shrinks` — every remaining hit is unrelated or re-worded (tank_steer.hpp keeps "14 of
  2 125 before it" as history); the new concepts — `leadTime`, `tankLeadCross`, `crossAhead`, `0.06 s`, `lead`,
  `TankSteerConfig`, `tankSteerStatus`, `strafeTractionDerate`, `dead strafe`, `15°`, `6 in` — consistent (no
  public 6 in promise: guide 14 says "do not plan on a sideways offset being removed"). No `simulat` in any
  added line; no `docs/internal` reference in a public document; relative links in the 17 changed documents: 0
  bad.
- **Documents this round:** `docs/changelog.md` (the lead; "a leg that arrived before still arrives" with the
  grid and its 0.24 s limit; the derate-0 exception; `leadTime`, `tankLeadCross`, `crossAhead`; acceptance END
  (0.23, 0.04)), guide 04 (overstatement; the lead in a clause; derate-0), guide 10 (two overstatements), guide
  11 (SFB on a tank; 39 SFB ticks), guide 14 (lag numbers re-measured; the residual 0.24 s regression; the pin),
  cookbook 04 (never more → the aim; the few-inches wording), `hardware-assumptions.md` (HA-179 index row, claim,
  source, lag, the lead, settle clause, pin numbers, the rate's sign; HA-04 blast-radius pointer; the TANK1 header
  paragraph's date), `chassis.hpp` (TANK STEERING: derate-0, the lead, "a leg that arrives …"), `version.hpp`
  (derate-0), `move_to_pose.hpp` (thesis + pipeline step 3 name the lead; HA-179/HA-04 label; the status's
  exit-tick doc), `motion_config.hpp` (`leadTime`, struct doc), `tank_steer.hpp` (the lead section, the settle
  bound with the lead, `tankLeadCross`, `crossAhead`, the floor's re-measured counts), `docs/api` regenerated,
  `build-order.md` / HANDOFF status lines. **Deliberately not touched:** the brief (`TANK1-tank-moveto-steers.md`
  — the coordinator's record; its §1.1/§1.2/§1.3 item 4 formula, its P-numbers and §6's table predate the lead:
  superseded by this log), `C1-COMPLETED.md`'s D12 note (still true: a bend ≤ 15° toward the line, faded to 0),
  the roadmap F6 parenthetical (unchanged), guide 09/12/15 (re-read: nothing the lead changed), `routine.hpp`
  and `strafe_to.hpp` (re-read: true).

### Part A review — disposition summary
| # | finding | verdict | disposition |
|---|---|---|---|
| 1 | BLOCKER — legs that finished before time out (10–15° off, pointed at the line) | **REAL** — reproduced exactly; root cause an under-damped line loop (also holes in the lateral reach the review did not name) | **Fixed at the root:** the bend steers on the cross-track error 0.06 s ahead (`leadTime`, commanded speed, no sensor read). Review grid: 178 → **0** regressions at plant kA 0/0.01/0.02/0.03; coarse grid 15 → 0; reach monotone. Residual at kA 0.04 (τ ≈ 0.24 s): 24 of 1 867 (18 without the lead) — disclosed in the changelog, guide 14, HA-179, chassis.hpp. Nothing ruled moved (15° cap = window, observer, Warn, arrival rule, off switch, X/H). Pinned by P15 (the review's four legs, the 144-leg family, the reach both ways), U8, P16; M30–M34 red. **For the team lead via the coordinator:** a control-law change inside the ruled envelope, made on the "fix at the root" instruction — the alternative the review offered (disclose it as a fourth behaviour change) was not needed |
| 2 | P8 (a) one-sided | REAL | both sides, each bound reached by its own sign; S17a/S17b red |
| 3 | TSTR bend share untested | REAL | P4 requires TSTR on clean lateral legs with the correction under half the floor; S1 red |
| 4 | authority `< 0.3` survives | REAL | P11: derate 0.01, 1e-9, `denorm_min` not eligible; S2 red |
| 5 | status cross/bend unread | REAL | P16 (bit-exact every commanding tick); `crossAhead` added; S4 red |
| 6 | overstating sentences | REAL | guide 10, guide 04, cookbook 04 re-worded |
| 7 | derate-0 H-drive left out of "X/H unchanged" | REAL | changelog, guide 04, version.hpp, chassis.hpp |
| 8 | guide 11 SFB text wrong | REAL (re-measured: SFB lights on a leg ON its line started 10° off) | rewritten; count re-measured (39) |
| 9 | HA-179 label missing in move_to_pose.hpp | REAL | label added; register Source updated; direction 1 clean |
| 10 | yaw-rate sign dependence unregistered | REAL (re-measured: 15.68°, TIMEOUT, correction pinned at 1.5) | HA-179 blast radius + HA-04 pointer + source comment |
| — | (found here) M34 and S6 survived | REAL gaps in the tests | P15 (c) both ways; P8 (f); both red |

- **Not committed. No `pros` command. No `/dev/ttyACM*`.** Part B not started. The tree: 42 tracked files
  changed + 5 untracked (`tank_steer.hpp`, `tank_steer_test.cpp`, `motion_tank_steer_test.cpp`,
  `docs/api/tank_steer.md`, this log).
- **For the coordinator:** (1) the lead is a design change after the go — say so to the team lead with the
  numbers above; (2) COMP-t's §6 re-predictions: the 40 in times are unchanged to 0.01 s (1.83/1.85/1.89/2.04/
  2.57 s), END y within 0.02 in; the acceptance run ENDs move by ≤ 0.02 in (R ×0.95 (0.30, 0.06), R ×0.91
  (0.37, 0.06), clean (0.23, 0.04, −179.97°) 5.44 s); the ×0.65 honest stop is END (23.37, ∓0.49) (was ∓0.56) —
  re-derive from P1/P3 as the brief says; (3) the brief's §1.1–§1.3 and §2 numbers predate the lead — this
  log supersedes them; (4) the earlier items (the Localizer's boundary re-read for IMU1; HA-162..178's gap)
  stand.

## Part A — the coordinator's merge and verification (2026-09-28, 11:31–11:42)

**The merge:** the tank1 worktree (the build, cut off once by a session end and resumed at 22:07; the review
round; the fix, which added the lead after the review's blocker) as the transport commit `681ee1e`,
squash-merged onto `a62067b` with no conflicts. The brief gained a dated header note pointing at the lead and
at this log, since its formula, P-numbers and §6 table predate it. The team lead was told of the design change
the same morning; everything he ruled is unchanged.

**Verification on the merged tree:** suite **1725 | 1725 passed | 0 failed | 3 skipped**, 2,648,032 assertions;
`src_build_gate.py` self-test **OK (189 detector cases)** and check **PASS** on all six builds (8/8 TUs each,
0 disallowed, beacons and programs checked); the ARM header gate clean; both CI greps empty; `check-fresh` 0;
the briefing current (regenerated before the build and after the suite binary); removability empty; the
staleness audit clean.

**Bit-identity, re-run by the coordinator against the pristine `a62067b` goldens on the merged tree:** the
review's `rv_hash` (X and H drives clean and hostile, every verb, a tank running only strafeTo / turnTo / hold /
brake / drive() / face / HoldPose, and `maxSteer = 0` on every verb) — **17 of 17 lines identical**; the
executor's P10 (a) — **12 of 12 identical**.

**The review's blocker legs, re-run on the merged tree** (`rv_arc`; pristine settled 1.64 / 1.61 / 1.59 /
1.48 s; the first build of TANK1 timed out on all four): **SETTLED 1.60 / 1.57 / 1.62 / 1.50 s**, each END inside
the unchanged 0.5 in / 0.02 rad rule.

**Seven coordinator mutations, each on a fresh binary, restored and hash-checked — all RED:**

| # | Mutation | Observed |
|---|---|---|
| TM1 | the lead dropped (the bend steers on the cross-track error NOW — the blocker's law) | RED |
| TM2 | the lead's heading sign flipped | RED |
| TM3 | the observer takes a SECOND IMU read in the tick | RED |
| TM4 | the engage decision re-made every tick (a leg the heading loop turns into the window gets steered) | RED |
| TM5 | TSTR lit by the observer's share only (a bend-only leg dark in the log) | RED |
| TM6 | an H-drive with strafe authority 0.3 treated as a tank | RED |
| TM7 | the observer's cap removed on the positive side | RED (`motion_tank_steer_test.cpp:754`, the cap) |

**Not done:** Part B (the ` xt` column on tank result lines) follows, alongside COMP-t. Nothing of TANK1 has run
on a robot; COMP-t's floor run is its first.
