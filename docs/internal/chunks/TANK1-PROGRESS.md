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

---

## Part B — the result line's cross-track column

- **Started:** Mon Sep 28 12:18:53 PM EDT 2026 (`date`).
- **Tree at start:** `b81a590` (TANK1 Part A committed as `3965791`), clean, in the git worktree
  `.claude/worktrees/tank1b` (branch `tank1b`) — NOT the main working tree.
- **Go:** the coordinator's workflow run, 2026-09-28, on the team lead's go for TANK1 (D3 (a): Part B after
  Part A, alongside COMP-t). Do not commit; no `pros` command; no `/dev/ttyACM*`. Mutation backups under
  `.claude/coordinator/scratch/tank1-b/` (never `/tmp`), each restored file's mtime bumped.
- **Scope:** Part B ONLY — every TRANSLATING tank result line gains ` xt  <signed in>"` (brief §1.5, re-check
  item 3: `hadDirection()` judged on the FIRST live record's target); the stats sink, `CompletedMotion`,
  `diag::MotionResult`, `emitResultLine`, `RunReporter`; §2.4's tests; M25/M27/M29; the Part B documents (§3).
  X/H drives byte-identical including their log lines. No `src/` change, no HA entry, no version change (2.3
  already names `xt`). COMP-t is being built in parallel from the same base (it touches `src/`, `competition/`,
  `hal/pros/block_sink`, `sequence/run_guard`, `api_doc_tool`); any file of its brief this part touches is
  logged here.

### 12:18 — start
- Read `docs/internal/RESUMING.md` (standing rules), the brief's header notes (the AMENDMENT: the lead), the
  Re-check amendments, § Decisions (D3), §1.5, §1.6, §2.4, §2.5 (M25/M27/M29), §3's Part B documents, and this
  log's Part A entries (as built, the review, the merge). This heading written before any code or probe.

### 12:20–12:27 — baseline suite, then the pre-change golden (BEFORE any code edit) *(`date` 12:26:28 PM at the capture)*
- **Baseline host suite** (briefing regenerated, fresh `cmake -S test -B build/test`, `Linking CXX executable
  shulib_tests`): `[doctest] test cases: 1725 | 1725 passed | 0 failed | 3 skipped`, `assertions: 2648032 |
  2648032 passed` — the merge record's numbers.
- **The pre/post probe** `.claude/coordinator/scratch/tank1-b/pb_hash.cpp` (evidence, not project code; built by
  `tank1-b/b.sh` with Part A's flags: g++ 13.3.0 `-std=c++20 -O2`, x86-64, host `lodgepc`), compiled against the
  worktree while its code was still `b81a590`'s (`git diff --stat -- include test src` empty; only this log and
  the regenerated briefing modified). Per script it prints the records+truth hash (`motion` — Part B must not
  move it), the log-line hash (`log`), and the log-line hash with a trailing ` xt <n>"` stripped (`log-xt` — Part
  B may add ONLY that), and writes every log line to `tank1-b/pre/<script>.log` for a line-by-line diff. Scripts:
  X and H-bot every verb with a RunReporter (P10 (b)'s holonomicScript), clean and hostile s7; a tank every-verb
  script with TANK1 ON and with `maxSteer = 0`, at ×1.00 clean, ×0.91 clean, ×0.91 hostile s7; the acceptance run
  (Routine) at R ×1.00, R ×0.91, L ×0.91; a lone tank 40 in and a lone tank turnTo at ×1.00 and ×0.91; a
  derate-0 H-drive (a moveTo and a turn). Output `tank1-b/pb_pre.txt`:

```
HOLO      X clean                rec  2671 motion 85461744ffad5c49 | lines   9 log d95cec78110e8a73 log-xt d95cec78110e8a73 | xt 0
HOLO      X hostile s7           rec  3289 motion f9e919ccaac760fc | lines   9 log b30625d93ec6909f log-xt b30625d93ec6909f | xt 0
HOLO      H clean                rec  2739 motion 6d876399a495c60f | lines   9 log a020b15c01f87d0a log-xt a020b15c01f87d0a | xt 0
HOLO      H hostile s7           rec  3387 motion 2c72f7898f285d63 | lines   9 log 7a75994317bb5f36 log-xt 7a75994317bb5f36 | xt 0
TANKON    x1.00 clean            rec  5382 motion 64fa0283ae6c4442 | lines  11 log b069841ea79baf15 log-xt b069841ea79baf15 | xt 0
TANKON    x0.91 clean            rec  5359 motion 17d8e101c41d0dbb | lines  10 log 791b4057584f04d0 log-xt 791b4057584f04d0 | xt 0
TANKON    x0.91 hostile s7       rec  5779 motion 71e64341a417fe1a | lines  10 log 6dcbb5b469ca50ef log-xt 6dcbb5b469ca50ef | xt 0
TANKOFF   x1.00 clean            rec  5807 motion b85b92f876f17bec | lines   9 log 87f0422c4e61d5a6 log-xt 87f0422c4e61d5a6 | xt 0
TANKOFF   x0.91 clean            rec  6515 motion 28278f847b3d85b5 | lines   9 log 140e0625d8c072b7 log-xt 140e0625d8c072b7 | xt 0
TANKOFF   x0.91 hostile s7       rec  6515 motion 3b96e25324d28029 | lines   9 log d7fdb41ec08fc69b log-xt d7fdb41ec08fc69b | xt 0
ACCEPT    accept R x1.00         rec  1092 motion 415cf0666529dba7 | lines   4 log 8ebe3e89f24727da log-xt 8ebe3e89f24727da | xt 0
ACCEPT    accept R x0.91         rec  1232 motion be1d01203d5f35df | lines   4 log ade919d674d58f02 log-xt ade919d674d58f02 | xt 0
ACCEPT    accept L x0.91         rec  1234 motion 845529d84c23748e | lines   4 log 4cc93c85c4ad2204 log-xt 4cc93c85c4ad2204 | xt 0
SINGLE    moveTo40 x1.00         rec   361 motion 3121bdf45d6982b3 | lines   1 log 03cc448228e37e5c log-xt 03cc448228e37e5c | xt 0
SINGLE    moveTo40 x0.91         rec   379 motion 03112ba93201a4c4 | lines   1 log fd5af9b5415a1657 log-xt fd5af9b5415a1657 | xt 0
SINGLE    turnTo90 x1.00         rec   237 motion 3c4a7d9e1378ddd1 | lines   1 log 97b1d7eda4158e53 log-xt 97b1d7eda4158e53 | xt 0
SINGLE    turnTo90 x0.91         rec   299 motion 043cf0023360bd41 | lines   1 log c9d10b7543865a88 log-xt c9d10b7543865a88 | xt 0
HDEAD     H derate-0             rec   598 motion 28a1dcb5401fa0c9 | lines   2 log ebcbc039a5e87efa log-xt ebcbc039a5e87efa | xt 0
```
- Seen in `pre/SINGLE_turnTo90_x0.91.log`: `TurnTo#1 ✓SETTLED final(  -0.4,  -0.7,  89.1°) … 1.49s` — the weak side drags
  the turn ~0.8 in (at ×1.00 it ends at (−0.0, −0.0)): the case where a LATEST-target direction test (re-check
  item 3, M29) would print `xt` on a turn. Part B must keep that line byte-identical.

### 12:27–12:37 — the code *(the suite binary's link time 12:37:02, its first green run — a file time)*
- `diag/motion_result.hpp`: `MotionResult` appends `hasCrossTrack` (default false) and `crossTrack`;
  `emitResultLine` appends ` xt ` + `appendNum(crossTrack, 6, 2)` + `"` AFTER the duration, only when
  `hasCrossTrack && hasPathData` (a line whose over/drift say n/a never carries a number from the same stream —
  the brief gated on the flag alone; the extra term is the formatter's own guard, pinned by a test, B1 below);
  the header's target-shape comment shows the tank line and defines xt (signed, + left of the target heading).
- `motion/motion_scheduler.hpp`: includes `tank_steer.hpp`; `MotionStatsSink` keeps `firstTarget_` (the FIRST
  live record's target, captured beside `startPose_`) and `lastMeasured_`, both reset in `beginMotion()`;
  `hadDirection()` (first target vs start ≥ `kHoldEpsilonIn`, re-check item 3) and `crossTrack()`
  (`tankLineError(lastMeasured_, target_).cross`, computed when asked); the class doc's aggregation rules name the
  one first-target question. `CompletedMotion` appends `hasCrossTrack` / `crossTrack`; `finalize()` sets
  `hasCrossTrack = hasData && stats.hadDirection() && strafeAuthority() == 0.0` and `crossTrack` only under it
  (0 otherwise — the brief filled it whenever there was data; "valid iff the flag, 0 otherwise" is the
  `overshoot`/`drift` pattern, pinned by XT4 and B9). `overshoot()` untouched (its bytes are pinned).
- `motion/run_reporter.hpp`: copies the two fields. `motion/tank_steer.hpp`: its "What uses this … and nothing
  else" sentence was made false by this change — now names the stats sink as the one reader that does not steer.
- `docs/api` regenerated (`motion_result`, `motion_scheduler`, `run_reporter`, `tank_steer` pages,
  `all-entities.md` 2 935 → 2 941, `README.md`).
- **The existing suite on the change, before any new test: 1725 | 1725 passed** — no existing test pinned a tank
  result line (grep: the only byte-pinned result lines are the X-shaped goldens in `motion_result_test.cpp` and
  `run_report_e2e_test.cpp`, both built by hand with the column off).

### 12:37 — the post probe: X/H log lines byte-identical, tank lines gain ONLY the column *(`pb_post.txt`'s mtime 12:37:44 — a file time, not a `date` reading)*
- `pb_hash.cpp` rebuilt unchanged against the change (`tank1-b/pb_post.txt`, logs `tank1-b/post/`); compared by
  `tank1-b/cmp.py` and `diff` per script with ` xt …"` stripped from the post lines:
  - **every `motion` hash (records + truth) identical to `pb_pre.txt`, all 18 scripts** — Part B moves nothing;
  - **X and H-bot, clean and hostile: `log` hash identical** (d95cec78110e8a73, b30625d93ec6909f,
    a020b15c01f87d0a, 7a75994317bb5f36) and the four `.log` files byte-identical (`cmp`);
  - **every tank script (TANK1 on and `maxSteer = 0`), the acceptance runs, the lone 40 in legs and the derate-0
    H-drive: `log-xt` (post, column stripped) == `log` (pre)**, and the per-line diff after stripping is empty —
    the only change anywhere is the appended column;
  - the column appears on exactly the translating lines: every MoveToPose (incl. SUPERSEDED/TIMEOUT, the un-faced
    Warned leg, the trajectory legs), every tank StrafeTo; never on TurnTo, a capture-here HoldPose, DriveBrake;
    the lone tank turn at ×0.91 (`TurnTo#1 … 1.49s`) stays byte-identical.
- Examples (post): `MoveToPose#1 ✓SETTLED final(  39.8,   0.0,   0.0°) over  0.00" drift  0.0°   1.80s xt   0.00"`
  (×1.00); `… final(  39.6,   0.0,   0.1°) … 1.89s xt   0.02"` (×0.91); `maxSteer = 0` ×0.91:
  `MoveToPose#1 ✗TIMEOUT final(  39.8,  -2.9,  -0.8°) over  0.00" drift  0.8°   5.01s xt  -2.86"`; the un-faced leg
  `✗TIMEOUT final(  62.8,   6.9,  30.1°) … 5.01s xt  -5.83"`.

### 12:55–14:04 — measured, then the tests *(`pb_measure.txt`'s mtime 12:55:49; the tests were written between then and the mutation pass's first `date`, 14:05:02 — file times, not `date` readings; the test files' own mtimes were bumped by the mutation driver)*
- On the built change: `crossTrack == tankLineError(finalPose, targetPose).cross` and `== ` the last record's,
  **exactly (diff 0)** in all nine cases; vs truth ≤ 2.4e-13 (clean sensors). 40 in: ×1.00 0.000000, R ×0.91
  +0.017206, L ×0.91 −0.017206, `maxSteer = 0` ×0.91 −2.861415 (TimedOut); lateral 12 in: +12 → −5.096176, −12 →
  +5.096176; heading 90 to (12, 40) → +5.096176; backing to (−40, 12) → −5.096176; un-faced (40, 20) → −5.916509.
  The tank turn: latest target 0.0000 in from its start at ×1.00, **0.8074 in at ×0.91** (M29's discriminator);
  a brake pre-empting a moveTo 0.5 s in: 0.0000 on the memoryless plant, **6.6099 in with plant kA 0.02**
  (the brake's target coasts with the robot — a second M29 discriminator); the explicit hold 3 in ahead / 1 in
  aside −1.000000 (TimedOut), the lateral strafeTo −6.000000; the Routine auton shape at ×0.91: xt −0.08 on both
  drives, none on the turns.
- **Tests added (10):** `test/motion_result_test.cpp` — 4 formatter cases (absent unless the flag, incl. a
  nonzero value with the flag off; the golden bytes and widths, any outcome; no path data ⇒ no column; NaN/Inf/
  1e300 and the longest possible line, ~~179 bytes~~ *(wrong — the Part B review's finding 3: built from −1e300, not the widest column; the longest MESSAGE is 175 bytes, the whole line 197 — see "Part B review findings")*, still ends in the whole column under TermSink's 200-byte cut).
  `test/motion_tank_steer_test.cpp` — XT1 (the stats sink, hand-fed: first-target direction, sign in two frames,
  latest-target line, beginMotion forgets, the 0.1 in boundary), XT2 (tank moveTo: exact, true, signed; 7 legs),
  XT3 (turn ×1.00/×0.91, coasting brake, hold here — none; strafeTo, hold there — xt), XT4 (X and H every verb,
  clean and hostile — none; H derate 0.01/1e-9/denorm_min — none and `crossTrack == 0` though the leg ended off
  its line; derate 0 — xt), XT5 (NullSink — none; `maxSteer = 0` — xt −2.86 == END y), XT6 (Routine auton shape,
  both sides — xt on the drives only). One test bug of mine fixed on the way (XT3 held a REFERENCE to
  `lastCompleted()` across the next verb, which reuses the slot — now a copy). ChassisTank gained an optional
  plant kA (test helper).
- Suite: **1735 | 1735 passed | 0 failed | 3 skipped, 2 648 326 assertions** (1725 + 10).

### 14:05–14:09 — the mutations (one pass; each a FRESH binary) *(`date` 02:05:02 PM → 02:09:10 PM)*
- Driver `tank1-b/mutate_b.py` (Part A's `mutate3.py` mechanics: backup under `tank1-b/mut/<id>/`, one exact
  replacement, touch every TU whose depfile names the header — 54 for `motion_scheduler.hpp`, 5 for
  `motion_result.hpp`, 4 for `run_reporter.hpp` — `--target shulib_tests/fast`, REQUIRE `Linking CXX executable
  shulib_tests`, the WHOLE suite, restore + sha256 + mtime bump). Output `tank1-b/mut/<id>/result.txt` +
  `suite.txt`, summaries `tank1-b/mut_pass1_a.txt`, `mut_pass1_b.txt`. **15 mutations, 15 fresh links, 15
  restores sha256-ok; 14 RED, 1 GREEN (B11, predicted equivalent).**

| # | mutation | red (failed cases) |
|---|---|---|
| M25 | (brief) `hasCrossTrack` without the authority check | **XT4** (57 assertions: every X/H translation line) |
| M27 | (brief) cross-track sign flipped in the stats sink | **XT1**, **XT2**, XT3, XT5 |
| M29 | (brief) `hadDirection()` on the LATEST target (the draft) | **XT3** (the ×0.91 turn only — ×1.00 passes — and the coasting brake), XT1 (c), XT6 (both turns, both sides) |
| B1 | formatter drops the `hasPathData` gate | the no-path-data formatter case |
| B2 | formatter puts xt BEFORE the duration | both formatter goldens, XT2, XT3, XT5, XT6 |
| B3 | formatter decides by the value (`!= 0`), not the flag | "absent unless" (the nonzero value, flag off), the golden widths (0.00), XT2, XT3 |
| B4 | waiting records aggregated (the live gate loosened) | XT1 (d), and the existing C5 hand-fed stats case |
| B5 | `beginMotion()` keeps `firstTarget_` | XT1 (b) |
| B6 | `beginMotion()` keeps `lastMeasured_` | XT1 (b) |
| B7 | `crossTrack()` off the FIRST target's line | XT1 (f) |
| B8 | RunReporter does not copy the fields | XT2, XT3, XT4 (the derate-0 line), XT5, XT6 |
| B9 | `CompletedMotion::crossTrack` filled without the flag | XT4 (the authority > 0 H-drives' `crossTrack == 0`) |
| B10 | the 0.1 in boundary exclusive (`>`) | XT1 (g) |
| B11 | `hasCrossTrack` without the `hasData` term | **GREEN — equivalent, as predicted**: with no data both poses read their defaults (beginMotion's reset), so `hadDirection()` is already false; the term stays so the rule reads as documented and does not lean on that invariant |
| B12 | the authority test loosened to `< 1e-6` | XT4 (the 1e-9 and denorm_min H-drives) |

- **After the last restore:** the first full build stopped at the doc gates (the briefing's suite count), so the
  binary was still B12's — the briefing regenerated, rebuilt (`[ 33%] Linking CXX executable shulib_tests`):
  **1735 | 1735 passed**; the briefing generated AGAIN after that (the first regeneration had read the B12
  binary and said "RED — 1 case(s) failing"), `briefing_status.py check` current.

### 14:10–14:50 — documents; one test added for a guide sentence (B13) *(`date` 02:29:56 PM at B13, 02:49:16 PM at the final probe)*
- **Public:** `docs/guide/11-reading-the-diagnostics.md` "Per-motion result lines" — the column (the golden tank
  line, byte-pinned in `motion_result_test.cpp`; what it measures, its sign, why a tank gets it, where it never
  appears, that `maxSteer = 0` keeps it and no stream leaves it out). `docs/guide/12-when-things-go-wrong.md` — the
  tank TIMEOUT differential's TSTR bullet reads `xt`, with a MEASURED sign rule: `tank1-b/pb_side.cpp` →
  `pb_side.txt` — a weaker RIGHT side gives a negative `xt` forward, backing, in L9's reverseTo shape and at
  heading 90, with TANK1 off (−2.86 at ×0.91, −10.03 at ×0.70) and on (×0.70 times out at −0.41); pinned by
  XT5's new backing/reverseTo checks *(only with the steering OFF — the review's finding 9; and the sentence was too strong with motor lag — finding 1; both fixed, XT8, in "Part B review findings")*, which the new mutation **B13** (the sign taken off the TRAVEL line,
  start→target, instead of the target heading — a faced `driveTo`'s line, so a plausible bug) turns red
  (XT5 ×2 at `motion_tank_steer_test.cpp:1888`, plus XT1/XT2/XT3; fresh binary, restored, `mut/B13/`).
  `docs/guide/15-glossary.md` — **cross-track error** (a new term, guide-maintenance's rule).
  `docs/changelog.md` 2.3 — change 3 now true (was "not yet built"), the header note and the derate-0 sentence,
  ADDED (the five fields/functions), Migration (the ` xt` exception stated as fact, with its −2.86 reading).
  `docs/diagnostics-plan.md` — the C5 result-line row names the column.
- **Headers:** `motion_result.hpp` (the target-shape comment, xt's semantics), `motion_scheduler.hpp` (the stats
  sink's aggregation rules, the two functions, `CompletedMotion`'s fields, `finalize()`'s rule), `run_reporter.hpp`,
  `tank_steer.hpp` ("What uses this"), `version.hpp` (the 2.3 note's Added list — a comment, no version change).
  `docs/api` regenerated after each `///` change (`motion_result`, `motion_scheduler`, `run_reporter`,
  `tank_steer`, `version`, `all-entities`, `README`).
- **Internal:** `build-order.md` TANK1 entry (a Part B executor status line; Part A's earlier lines left as the
  coordinator's record), `HANDOFF-remaining-work.md` L2u (one sentence), this log, the briefing (regenerated).
- **Deliberately not touched:** the brief (the coordinator's record); `docs/shulib-v2-master-plan.md` §18.3 (the
  design sketch of the line, read — it was never the byte format, guide 11 is); `docs/guide/08` (its transcripts
  are an X-drive's, and its `n/a` lines carry no column on any drive); guides 04/09/10/14 and cookbook 04 (read
  for tank-log wording: nothing false — they send the reader to guide 11/12 for the log);
  `docs/hardware-assumptions.md` (no new entry; HA-179/180's settle clauses read "END y by the estimate" — still
  true, and the result line's `xt` is now the same number on a tank's 40 in leg: noted for the coordinator);
  `docs/roadmap.md` (the C5 rows are history; F6 does not freeze `CompletedMotion`'s field set — its three public
  types are `ChassisConfig`, `MotionOptions` and the trajectory result — so the appended fields are additive).

### 14:50 — prose review (pace rule 4), grepped and read BEFORE this entry
- Scope: `README.md`, `docs/*.md`, `docs/guide/`, `docs/cookbook/` (generated `docs/api/` excluded); every hit read.
  - **Forward references to Part B** (`Part B`, `xt column`, `` ` xt``, `cross-track column`, `when it lands`,
    `not yet built`), also over `include/`, `test/`, `src/`: the changelog's two "(when it lands)" sentences were
    the only stale ones — rewritten; the rest are mine or other chunks' (DIAG1/COMP-b/…, unrelated).
  - **Changed files' names** (`motion_result`, `motion_scheduler`, `run_reporter`, `tank_steer`): diagnostics-plan
    :168/:275-276, guide 05 :132, guide 11 :16/:285/:518, guide 12 :225, roadmap :921/:1189-1190/:1355/:1358,
    HA-179's source line, changelog — all still true (C5 history, reference links, the golden-file list).
  - **Types** (`MotionResult`, `emitResultLine`, `CompletedMotion`, `MotionStatsSink`, `RunReporter`,
    `hasPathData`, `lastCompleted`): guide 08 :325-331 (RunReporter wiring, X-drive), guide 09 :147, guide 10
    :83/:325, roadmap F6 row :140 and :1306 — true; F6 freezes `lastCompleted()`'s signature, not
    `CompletedMotion`'s field set.
  - **Concepts** (`result line` 26 hits, `§18.3` 3, `overshoot` 14, `stationary` 9, `n/a` 27, `hadDirection` 1,
    `kHoldEpsilon` 0, `cross-track` 8, `` xt`` 9, `sideways offset` 3, `END y` 2): guide 12 :201 ("Result lines
    say `n/a`") still true (the column is absent then, as guide 11 says); guide 08's transcripts are an X-drive's;
    master plan :848/:858 the §18.3 sketch (not edited, above); HA-179 :2029-2030 / HA-180 :2073 (not edited,
    above); roadmap :628 is fusion's cross-track drift, unrelated. No stale sentence left.
  - `simulat` in added lines: 0. Internal references from the changed public documents: 0. Links in every changed
    `.md` (relative paths + anchors, mechanically): 246 checked, 0 bad.

### 15:07 — Part B: final gates, and where it stands *(`date` 03:07:28 PM at this entry)*
- **Host suite** (briefing regenerated, full build WITH the doc gates, `[100%] Linking CXX executable
  shulib_tests`): **`test cases: 1735 | 1735 passed | 0 failed | 3 skipped`, `assertions: 2648332 | 2648332
  passed`.** Baseline 1725 → 1735: +10 (4 formatter, XT1–XT6). No existing test's code changed; one test helper
  gained a parameter (`ChassisTank`'s optional plant kA). The briefing regenerated after the suite binary, the
  build re-run with the doc gates: rc 0.
- **The pre/post probe on the FINAL tree** (`pb_final.txt`): byte-identical to `pb_post.txt`; vs `pb_pre.txt`
  (`b81a590`'s code): all 18 `motion` hashes equal; the four X/H `log` hashes equal and their `.log` files
  byte-identical; every tank/derate-0 log equals the pre log once the column is stripped.
- **`src_build_gate.py self-test`**: `src build gate self-test: OK (189 detector cases, each against a real
  build)`. **`check`** (re-run after the last header edit): `src build gate: PASS (all 6 builds link; …)` —
  bench, xdrive, tank, tank-drive, tank-library, tank-calibrate: exit 0, 8/8 TUs, 0 disallowed warnings.
  `git diff -- src/`: 0 lines.
- **ARM header gate** (RESUMING's command, 192 headers): rc 0; `motion_result.hpp`, `motion_scheduler.hpp`,
  `run_reporter.hpp`, `tank_steer.hpp`, `version.hpp` each alone: rc 0.
- **CI greps:** `library is PROS-free (all of include/shulib except hal/pros/)`; `layering holds: core is sim-free`.
- **api_doc_tool:** `self-test: OK`; `doc coverage: 2941 public entities across 156 headers, all documented`;
  `check-fresh` rc 0; `doc example scan: 467 quoted lines, all verbatim (5 source files); 3 listings of whole-run
  files, each one unbroken run`; `removability: no public doc references docs/internal/`.
- **`doc_staleness_audit.py`**: `clean (27 live docs, 0 numeric claims checked)`; its self-test OK (10 cases).
  **`briefing_status.py check`**: `generated block current`.
- **Mutations:** 16 run (M25, M27, M29, B1–B13), 16 fresh links, 16 restores sha256-ok; 15 RED, 1 GREEN (B11,
  the predicted equivalent). Table in the 14:05 entry; B13 above.
- **Not committed. No `pros` command. No `/dev/ttyACM*`.** No new HA entry, no version change, no `src/` change.
  No `TANK1-COMPLETED.md` (the coordinator's, at the merge); `roadmap.md`'s "you are here" and `build-order.md`'s
  `Next:` pointer NOT moved (the coordinator's reconciliation).
- **Shared with COMP-t (built in parallel from the same base):** `docs/changelog.md` (COMP-t's brief adds
  Fixed/Added entries — mine touch only the TANK1 2.3 entry's lines 21-31, 64-74, 96-100, 105-107),
  `docs/api/` (both regenerate — regenerate once more after the second merge), `docs/internal/PROJECT-BRIEFING.md`
  (regenerate after the merge), `build-order.md`/HANDOFF status lines. No file under `src/`, `competition/`,
  `hal/pros/block_sink`, `sequence/run_guard` or `tools/` was touched.
- **For the coordinator:** (1) `MotionResult`'s formatter renders the column only with `hasPathData` as well as
  the flag — one term beyond the brief's §1.5 (pinned, B1); `CompletedMotion::crossTrack` is 0 unless the flag
  (the brief filled it whenever there was data; pinned, B9). (2) `build-order.md`'s TANK1 entry still carries
  Part A's pre-merge lines ("Part A is being built", "BUILT … NOT committed", "Part B not started") — I added a
  Part B line and left those to your reconciliation, as with HANDOFF L2u's Part A sentence. (3) HA-179/180's
  settle clauses and COMP-t's Station M.6 read "END y by the estimate"; on the 40 in leg (line y = 0 along 0°) the
  tank's result line now prints that number as `xt` — M.6 can read it off the result line. (4) Part A's P2
  (`motion_tank_steer_test.cpp:468`) does `CAPTURE(c.name)` on a `const char*`, which doctest prints as a
  pointer; mine print the name (a `std::string` copy) — P2 left as it is (not Part B's). (5) The IDE's clangd
  resolves the MAIN tree's headers, so it flags the new members as missing in this worktree — noise, the build is
  the evidence.

### 15:39 — RESUMED by a second executor run (same workflow, same go) *(`date` 03:39:22 PM at this entry)*
- Found the worktree dirty with the whole of Part B as the 15:07 entry records it (every changed file's mtime
  ≤ 15:05:35 except this log, 15:20:06; the suite binary linked 15:05:21) and no final report delivered. Per
  RESUMING ("if `git status` is dirty … verify and finish"), NOT rebuilding Part B: re-reading the brief's Part B
  sections against the diff, then re-running the suite, the pre/post probe, the brief's three mutations and every
  gate on the tree as it stands, and logging what I see below before reporting.

### 15:39–15:55 — the second run's verification of the tree as it stands *(`date` 03:54:55 PM at the last check)*
- **Read against the brief** (§1.5, §2.4, §2.5 M25/M27/M29, §3's Part B list, re-check item 3): every item is
  in the diff; the ×0.91 turn, the coasting brake and the capture-here hold carry no column; the capture-here
  hold's first live target equals that record's estimate by construction (`move_to_pose.hpp` ~:199-201 captures
  `loc.pose()`, `baseRecord` writes the same `pose`); guide 11's "a ✓SETTLED leg's xt is inside the arrival
  tolerance" holds (`SettledUtil::update` needs |hypot(errX, errY)| ≤ maxError on the exit tick, and the column
  reads that exit record).
- **Suite** (briefing regenerated, `cmake -S test -B build/test`, full build with the doc gates): `1735 | 1735
  passed | 0 failed | 3 skipped`, `assertions: 2648332 | 2648332 passed`. The ten Part B cases alone: 10/10, 300
  assertions.
- **All 16 mutations RE-RUN** (`tank1-b/mutate_b_rerun.py` — `mutate_b.py` with its output under
  `tank1-b/mut_rerun/`, so the first pass's evidence is kept; summary `tank1-b/mut_rerun.txt`; `date` 03:40:35 →
  03:45:28 PM): 16 fresh links, 16 restores sha256-ok with mtimes bumped; **the same verdicts as the 14:05 pass
  and B13** — M25 → XT4 (57 assertions); M27 → XT1, XT2, XT3, XT5; M29 → XT1, XT3, XT6; B1 → the no-path-data
  formatter case; B2 → both formatter goldens + XT2/3/5/6; B3 → "absent unless", the goldens, XT2, XT3; B4 → XT1 +
  C5's hand-fed stats case; B5/B6/B7/B10 → XT1; B8 → XT2–XT6; B9/B12 → XT4; B11 GREEN (the predicted
  equivalent); B13 → XT1, XT2, XT3, XT5. Then the briefing regenerated, the suite relinked
  (`[ 33%] Linking CXX executable shulib_tests`): 1735/1735; the briefing generated AGAIN on that fresh binary,
  `check` current.
- **The pre/post probe re-built and re-run** (`tank1-b/pb_rerun`, `pb_rerun.txt`, logs `tank1-b/rerun/`):
  `pb_rerun.txt` byte-identical to `pb_final.txt` and `pb_post.txt`; against `pb_pre.txt` (`b81a590`'s code): all
  18 `motion` hashes equal; X/H clean and hostile `log` hashes equal and the four `.log` files `cmp`-identical;
  both lone tank turns (×1.00, ×0.91) `cmp`-identical; every other tank line differs only by the appended column.
- **Gates:** `src_build_gate.py self-test` OK (189 cases), `check` PASS (6 builds, `v0.1.1-349-gb81a590-dirty`),
  `git diff -- src/` 0 lines after it; ARM header gate (192 headers, the all-headers TU in `tank1-b/`) rc 0 and
  the five changed headers alone rc 0; both CI greps; `api_doc_tool` self-test / check-coverage (2941 across 156) /
  check-fresh / check-examples / check-removability; `doc_staleness_audit.py` self-test + clean;
  `briefing_status.py check` current.
- **One stale sentence found and fixed:** `build-order.md`'s Part B status line (written 14:24, before B13 and
  the `version.hpp` comment) said "15 mutations … 14 red" and "four header comments" — now 16 / 15 red and five
  header comments. Prose greps re-run over `README.md`, `docs/*.md`, `docs/guide/`, `docs/cookbook/` (`xt`,
  `cross-track`, `result line` — 27 hits read, `Part B`/`when it lands`/`not yet built`, the four changed headers'
  names, `simulat` in added lines — the one hit is this log's own "`simulat` … 0"): nothing else stale.
- **For the coordinator (new in this pass):** my `motion_scheduler.hpp` edits add 52 lines above ~:700 and 7
  more after ~:1090, so COMP-t's brief's line citations into that file (:704-705, :763-792, :765-775,
  :1010-1017) move by +52 once both land — only a problem if COMP-t writes them into a document or comment.

### Part B review findings and their disposition *(started `date` 04:21:45 PM)*
- Three reviewers (correctness, identity, documents) returned 12 findings, all minor; deduped to 10 (the
  changelog :70 H-drive exception is in two of them; the same code rule sits under three statements). Each is
  re-checked against the tree and the reviewers' own probes (`tank1-b/review-correctness/`, `review-bitid/`)
  BEFORE any edit; the disposition of each follows as it is done.
- *(`date` 04:48 PM)* **Resumed by a fresh executor run** (the entry above was started at 04:21:45 PM and stopped
  before any edit: every tracked file's mtime ≤ 16:21:50, this log's; the one thing it left is a measurement probe,
  `tank1-b/review-fix/rf_measure.cpp` → `rf_measure.txt`, 16:31:06 — read, and used below as evidence where it
  answers a finding; everything else re-checked from here).

#### 16:48–17:06 — every finding re-checked, the edits made *(`date` 05:07 PM at this entry)*
- **F1 (guide 12's sign rule fails with motor lag) — REAL, and wider than reported.** Re-checked with a sweep of
  my own (`tank1-b/review-fix2/rf_sweep_fine.cpp` → `rf_sweep_fine.txt`: steering ON, plant lag kA 0–0.06 in 0.005
  steps (τ up to ≈ 0.36 s), one side ×0.40–×0.95 in 0.01 steps, both sides, five legs that start ON their line —
  7 280 legs): 6 706 timed out; **776 ended on the STRONG side, the largest |xt| among them 0.648 in** (0.415 at
  kA 0.02 — the reviewer's leg, reproduced exactly by `review-fix/rf_measure.txt`; 0.445 at kA 0.03; 0.567 at
  0.045; 0.648 at 0.06), and none at kA 0. So the sign names the weak side only for an `xt` of an inch or more.
  **And a second hole the review did not name:** a leg that STARTS beside its line (`rf_offset.cpp` →
  `rf_offset.txt`, targets 1–12 in aside, ×0.60/×0.75/×0.91): of 82 timeouts, 24 crossed to the weak side and
  the rest ended on their START side — e.g. the target 12 in to the right (the robot starts left of the line)
  with the RIGHT side 9 % weak ends at xt **+4.52** (lag-free). A first draft of my guide fix said "a leg that
  started beside its line ends on the side it started from, whichever side is weak"; the probe showed that
  false (24 crossings) BEFORE it was kept — reworded to "can end on the side it started from".
  **Fix:** guide 12's TSTR bullet now says: on a leg that started on its line (faced at its point), an `xt` of an
  inch or more names the weak side; a leg that started beside its line can end on its start side; a smaller `xt`
  names no side (the reviewer's 60 in, ×0.75, kA 0.02 leg quoted: `xt   0.42"`, to the left). **Pinned by a new
  case XT8** (steering ON): (a) a 324-leg grid — kA 0/0.02/0.03, either side ×0.50–×0.90, six legs (forward
  24/40/60, backing 40, the reverseTo shape, heading 90) — every TIMEOUT with |xt| ≥ 1 names the weak side
  (measured: 180 timed out, 130 of them ≥ 1 in; non-vacuity `>= 100`); (b) the guide's 0.42 leg — TIMEOUT, xt
  +0.415, TSTR on its last live record, the line ending `s xt   0.42"`; (c) the 12 in-aside leg ends LEFT (> +1).
- **F2 + F6 (the derate-0 H-drive exception missing from three statements) — REAL (documents only).** Fixed:
  `motion_result.hpp`'s xt paragraph (names the exception, and that every outcome carries it),
  `CompletedMotion::hasCrossTrack`'s doc ("False on every drivetrain with any strafe authority (every X-drive,
  every H-drive with a live strafe wheel)"), changelog change 3 ("— except an H-drive with no strafe authority,
  which is treated as a tank (above)"), and the test file's Part B section comment. `version.hpp`'s 2.3 note
  already named it (read). `docs/api` regenerated.
- **F3 (the formatter's "longest possible line" is not the longest) — REAL.** Confirmed by construction:
  appendNum keeps a %f rendering up to 10 bytes and compacts a longer one to %.3g, whose longest is
  `-1.23e+300` (10 bytes); `-1e300` gives 7. The case now tries four values of both kinds (−1.23e300, −1e300,
  −999999.99, −123456.7) × every fault code 0–255 (a longer fault name added later is covered), measures the
  MESSAGE (the bytes after `[MOT] `), and checks that EVERY one of the 1 024 lines ends with its whole column
  (a message over 200 would be cut by TermSink and end in "…" — so a length bound read after the cut could
  not see it; the per-line ending check can). Measured: **175 bytes** (the review's number), asserted `>= 175`
  (non-vacuity) and `<= 200`. The 14:05 entry's "179 bytes" is corrected in place, marked.
- **F4 (xt on a CANCELLED / FAULT_ABORT line unpinned) — REAL.** New case **XT7**: a tank moveTo (40, 5, 0)
  stopped 0.7 s in by `sched.cancel()` and by an ODO_STUCK raised on the rig's latch — `exit == Cancelled`,
  `!preempted`, the right `abortFault`, `hasCrossTrack`, xt == `tankLineError(final, target).cross` (measured
  −0.4772 / −0.4536 — `review-fix/rf_measure.txt` §B), the line starts `MoveToPose#1 ✗CANCELLED final(` /
  `MoveToPose#1 ✗FAULT_ABORT=ODO_STUCK final(` and ends in the column. The formatter's any-outcome golden
  gained a CANCELLED and a SUPERSEDED line with the flag set.
- **F5 (`CompletedMotion`'s default unpinned) — REAL.** XT7 checks `CompletedMotion{}` AND
  `chassis.lastCompleted()` before any verb (exit Running): no flag, crossTrack 0. The field's doc now says why
  the default matters (the "none yet" record claims no number).
- **F7 (changelog: strafeTo "unchanged on a tank") — REAL.** Now: those verbs "move exactly as before on a
  tank; of their result lines, only `strafeTo`'s changes — it gains ` xt` (change 3)".
- **F8 (changelog: "one side 9 % weak" with a signed value) — REAL.** Both places now say the RIGHT side, with
  the left side's value beside it (−0.02; +2.86 — `pb_side.txt`, `pb_measure.txt`), and change 3 says a leg the
  steering brings home can finish a hair to either side of its line.
- **F9 (the test "pinning" guide 12 runs with the steering off) — REAL.** XT8 above is the steering-ON pin; the
  15:07-era claim in this log ("pinned by XT5's new backing/reverseTo checks") is corrected in place, marked.
- **F10 (guide 11: "a small xt means it stopped short along its line") — REAL.** A TIMEOUT needs translation AND
  heading settled (`move_to_pose.hpp`'s exit rule), and the reviewer's 0.42 leg is a third case (at the point,
  0.3 in short and 0.42 aside: hypot 0.51 > 0.5 — neither error alone would stop it). Reworded: with a small
  `xt`, being beside the line is not the story — compare `final(…)` with the target: short of it along the
  line, or at it but never settled (`drift`, or hovering just outside the tolerance — Chapter 12's "almost
  settled, never quite").
- **F11 ("a hold of a spot away" — no public verb does that) — REAL (wording).** `Chassis::hold` and
  `Routine::hold` take only a duration (`chassis.hpp:382`, `routine.hpp:306`). Guide 11, changelog change 3 and
  the stats sink's `hadDirection()` doc now say "a Tier-3 `HoldPose` built with a spot away from the robot" and
  "the `hold` verb (it holds where the robot is)". Guide 11 also says "whatever the outcome" (XT7 pins it).
- **F12 (internal status text; no `TANK1-COMPLETED.md`) — PARTLY mine.** The status lines false at the base are
  fixed where they sit (below, after the gates); the completion record, the roadmap's "you are here", the
  build-order `Next:` pointer and the briefing's OPEN-chunk line stay the coordinator's at the merge — the
  project's convention (SRC1-PROGRESS.md :288, "The -COMPLETED.md record, the commit and the merge verification
  are the coordinator's"; the brief §3: "the coordinator reconciles 'you are here'"), and writing a completion
  record before the coordinator's verification would claim a close that has not happened.
- **Suite on the change** (`--target shulib_tests/fast`, fresh link 17:05:26): **1737 | 1737 passed | 0 failed |
  3 skipped**, 2 650 734 assertions (1735 + XT7 + XT8). The Part B cases alone: 12/12.

#### 03:36 (2026-09-29) — RESUMED by a third executor run after the team lead's pause *(`date`: Tue Sep 29 03:36:32 AM EDT 2026)*
- The 16:48 run was stopped partway (the team lead paused all work, 2026-09-28 ~17:40). Found: every finding's
  edit in the tree (tracked files' mtimes 17:01–17:26; the suite binary linked 17:26:43; `docs/api` regenerated
  17:26:18; the briefing 17:36:53), a second mutation pass under `tank1-b/review-fix2/mut/` (`mut_rf.txt`, ended
  17:22:11; an earlier pass `mut_contaminated_1710/` set aside by that run's own name), and a `build-order.md`
  status line (17:25) that already claims that pass's verdicts — but NO log entry for the mutation pass, the
  gates or the prose review. Those claims are not taken as done: re-checked below from the tree.
- **One file the stopped run left wrong:** `src/build_selection.hpp` (mtime 17:47:00) carries ONE added line,
  `SelfTestUnhandled,  ///< self-test plant (case 18)` — `tools/src_build_gate.py`'s self-test case 18 plants
  exactly that line (`:1736-1738`) inside a `_replace` context and restores it on exit; the run was killed during
  the self-test (`review-fix2/srcgate_selftest.log`, 17:42:53, empty), so the plant was never restored. Not a
  Part B change (Part B has no `src/` change): restored to `HEAD` with `git checkout`, then `git diff -- src/`
  checked empty. `bin/` (gitignored) holds that interrupted build's `cold.package.elf`; the gate deletes and
  rebuilds `bin/`/`.d/` itself.
- Plan: rebuild and run the suite on the tree as it stands; re-read each finding's edit against the finding;
  re-run the review-round mutations whose evidence is not in this log (fresh binaries); every gate; the prose
  review; then the disposition summary.

#### 03:37–03:57 (2026-09-29) — the stopped run's work re-checked *(`date` 03:43:46 AM at the first suite run, 03:51:30 AM at the mutation pass's end)*
- **Each finding's edit re-read in the diff against the finding** (not against the 16:48 entry): F1/F9 guide 12's
  TSTR bullet + XT8; F2/F6 `motion_result.hpp` :54-58, `CompletedMotion::hasCrossTrack`'s doc, changelog change 3
  (:72-73) and the test section comment; F3 the formatter case (four values × 256 fault codes, message measured,
  every line's ending checked); F4/F5 XT7 and the formatter's CANCELLED/SUPERSEDED goldens; F7 changelog :80-81;
  F8 changelog :66-68 and :109-112; F10 guide 11 :114-120; F11 guide 11 :97-99, changelog :70-72,
  `hadDirection()`'s doc; F12 HANDOFF L2u and build-order's TANK1 entry. All present and each says what the
  finding asked. Evidence probes read: `review-fix2/rf_sweep_fine.txt` ("TOTAL: 6706 legs timed out … 776 ended on
  the STRONG side, largest such |xt| 0.648 in"), `rf_offset.txt` ("timeouts 82: ended on the start side 58,
  crossed 24"), `review-fix/rf_measure.txt`.
- **Suite on the tree as found** (briefing regenerated, `cmake -S test -B build/test`, full build with the doc
  gates — `[  8%] Linking CXX executable shulib_tests`, the restored `src/build_selection.hpp` recompiling 8 TUs):
  **`1737 | 1737 passed | 0 failed | 3 skipped`, `assertions: 2650734 | 2650734 passed`** — the 17:05 numbers.
  The twelve Part B cases alone: 12/12, 2 702 assertions; the formatter's MESSAGE line reads `longest result-line
  message: 175 bytes`, XT8's `180 legs timed out, 130 with |xt| >= 1 in`.
- **The stopped run's review-round mutation pass** (`review-fix2/mutate_rf.py` → `mut_rf.txt`, 17:15:08 →
  17:22:11; never logged before now): 23 mutants — M25, M27, M29, B1–B13, the reviewers' N1, N3, N8, N10, N11,
  N19, and TC (TermSink's message cap 200 → 170) — each `linked=True`, each restored `sha256 ok, mtime bumped`;
  **22 red, B11 green (the predicted equivalent)**. Is it evidence for THIS tree? Every one of its 23 backups
  (`mut/<id>/*.orig`) was compared by sha256 with the current file: **23/23 identical**; the test files' mtimes are
  the driver's last bump (17:22:11) and its unmutated-count runs read the same 1737 / 2 650 734 as today. (Its
  first attempt, `mut_contaminated_1710/`, was set aside by that run: the `/fast` target does not re-scan header
  dependencies, so B1–B3 linked M29's objects; the driver re-touches each mutation's TUs at restore since.)
- **Re-confirmed on fresh binaries by this run** (`review-fix3/mutate_rf3.py` — the same driver, output under
  `review-fix3/mut/`; `mut_rf3.txt`, 03:49:04 → 03:51:30 AM): the seven review-round mutants, M27 (the brief's
  sign mutation) and B13 (the build's travel-line sign, which reds XT8), **each `linked=True`, each restored sha256 ok with mtime bumped, each RED with the same cases as
  17:15**: N1 → XT7; N3 → XT3; N8 → XT3, XT7; N10 → the formatter golden, XT7; N11 → XT7; N19 → XT7; TC → the
  worst-case formatter case + TermSink's two truncation cases; M27 → XT1, XT2, XT3, XT5, XT7, XT8; B13 → XT1, XT2,
  XT3, XT5, XT7, XT8.
- **One sentence of the stopped run's tightened:** guide 12's lag example named no lag; it now says "with 0.12 s
  of lag" (plant kA 0.02 — the τ guide 14 quotes in the same words, and XT8 (b)'s), re-wrapped to the page's width.

#### 03:57–04:22 (2026-09-29) — gates, a prose review of the review round, five more stale sentences *(`date` 04:22:17 AM at the src gate's end; the next `date` read 10:38:31 AM — a wall-clock jump like Part A's, no work between)*
- **Gates on the tree as found** (after the `src/` restore, before this run's edits): `src_build_gate.py
  self-test` OK (189 cases, 03:55:30 → 04:01:20), `check` PASS (six builds, 04:02:27), `git diff -- src/` empty
  after both; ARM header gate (192 headers) rc 0 and the five Part B headers alone rc 0; both CI greps; every
  `api_doc_tool` check; `doc_staleness_audit.py` self-test + clean; `briefing_status.py check` current.
- **Prose review of the review round (pace rule 4), grepped and read BEFORE this entry** — the 14:50 and 15:39
  reviews predate the review fixes, so none covered them. Scope `README.md`, `docs/*.md`, `docs/guide/`,
  `docs/cookbook/`, and `include/` for the claims (hit counts re-counted on the FINAL tree, after this run's
  edits): `` ` xt``/`xt  `/`cross-track`/`crossTrack`/`hasCrossTrack` (51 hits, `RobotContext` noise among them); `one side (a few|9|3)`, `unchanged on a tank`, `stopped short`, `179 bytes`, `longest possible`,
  `weaker right`, `right side (9|25)`, `names the weak`, `no strafe authority`, `dead strafe` (20);
  `never carr`, `no x-drive or h-drive`, `H-drive line` (12); `hold of a spot`, `hold of here`, `spot away` (12);
  `almost settled`, `HoldPose`, `lastCompleted`, `✗CANCELLED`/`✗FAULT_ABORT`/`✗SUPERSEDED` (20); headers'
  `unchanged`/`bit for bit`/`byte for byte` near tank/strafe/hold/log (16); `maxSteer` (10); `log exactly`,
  `old tank exactly`, `pre-TANK1 tank` (5). Every hit read. **Five sentences Part B had made too strong — the
  same class as findings 2/6 and 7, which no reviewer listed:**
  - `docs/guide/14-what-it-cannot-do-yet.md` :363 "Setting `tankSteer.maxSteer` to 0 … restores the old tank
    exactly" → names the one exception, the `xt` column, linking Chapter 11;
  - `docs/hardware-assumptions.md` HA-179's **Contained:** clause, "`maxSteer = 0` restores the pre-TANK1 tank's
    motion, records and log exactly" → "— all but the result line's ` xt` column (TANK1 Part B, 2026-09-28), a
    report kept whatever `maxSteer` is" (an edit to an existing entry's clause; no new HA entry, the header count
    unchanged);
  - `motion_config.hpp` `TankSteerConfig::maxSteer`'s doc, "0 turns TANK1 off entirely: … moves, records and ends
    exactly as it did before TANK1" → "0 turns the steering off entirely … (its result line keeps TANK1 Part B's
    ` xt` column …)";
  - `strafe_to.hpp`'s file header, "is unchanged on a tank, bit for bit" → moves, records and ends exactly as
    before; only its result line changed (the scheduler's ` xt`);
  - `move_to_pose.hpp` `PoseMotionOptions::tankSteer`'s doc, StrafeTo and HoldPose "on a tank they are
    unchanged" → "move exactly as before", their result lines being another matter.
  Read and left true: `version.hpp` :84 ("motion, records and truth bit for bit"), `chassis.hpp` :125 ("motion
  exactly") and :304 (strafeTo's exit "unchanged by TANK1"), guide 10 :289 ("`maxSteer = 0` turns it off" — the
  steering), guide 12 :91 (a `strafeTo`'s "timeout is unchanged"), guide 04 :109 (the steering), guide 11 :199
  (`SFB` counts), guide 09 :147 and guide 10 :83/:325 (`lastCompleted()`), guide 12 :96-106 (the outcome
  bullets), the glossary's "tank only" (Ch. 11 carries the derate-0 exception), the header section comment at
  `motion_result_test.cpp` :187. No public doc cites a line of the three newly edited headers; COMP-t's brief
  cites none either (grep). The 14:10 entry's "deliberately not touched" list read guide 14 ("nothing false") and the HA register
  (only HA-179/180's settle clauses) — it missed these two sentences.
- `docs/api` regenerated (`motion_config.md`, `move_to_pose.md`, `strafe_to.md` join the Part B pages; the
  `move_to_pose.hpp` declarations below :98 move +2 lines). `simulat` in every added line: only this log's own
  mentions of the search itself (three, this one included). Relative links and anchors in the seven changed public documents: 124, 0 bad.
- **Final gates on the final tree** *(`date` 04:11:28 AM at the start)*: briefing regenerated; `cmake -S test -B
  build/test`; full build with the doc gates (176 objects, `[100%] Linking CXX executable shulib_tests`): **`test
  cases: 1737 | 1737 passed | 0 failed | 3 skipped`, `assertions: 2650734 | 2650734 passed`**; briefing
  regenerated after the suite binary, the build re-run with the doc gates: rc 0. ARM header gate: 192 headers rc 0;
  the eight changed headers alone rc 0. CI greps: `library is PROS-free (all of include/shulib except
  hal/pros/)`, `layering holds: core is sim-free`. `api_doc_tool` self-test OK / check-coverage "2941 public
  entities across 156 headers, all documented" / check-fresh rc 0 / check-examples "467 quoted lines, all
  verbatim" / check-removability "no public doc references docs/internal/". `doc_staleness_audit.py` self-test OK
  (10), "clean (27 live docs …)". `briefing_status.py check`: current. `src_build_gate.py self-test`: "OK (189
  detector cases, each against a real build)" (04:14:01 → 04:21:15); `check`: "PASS (all 6 builds link …)",
  bench/xdrive/tank/tank-drive/tank-library/tank-calibrate each exit 0, 8/8 TUs, 0 disallowed (04:22:17);
  `git diff -- src/`: 0 lines.
- **The pre/post probe on the final tree** (`pb_hash.cpp` rebuilt by `b.sh` → `review-fix3/pb_final3.txt`, logs
  `review-fix3/final/`): byte-identical to `pb_rerun.txt` (15:39); against `pb_pre.txt` (`b81a590`'s code) all 18
  `motion` hashes SAME; the four X/H `log` hashes SAME and their `.log` files `cmp`-identical; both lone tank turns
  identical; every other tank line and the derate-0 H-drive's differ only by the appended column.

#### Part B review — disposition summary *(`date` 10:38:31 AM)*
Twelve findings, all minor; ten distinct (F2 = F6, the H-drive exception's statements; F1 and F9 share one fix,
XT8). Each re-checked against the tree and the reviewers' probes before its edit.

| # | finding | verdict | fix (and its pin) |
|---|---|---|---|
| F1 | guide 12's sign rule fails with motor lag | REAL, wider (a leg started beside its line too) | guide 12 states the measured rule (≥ 1 in, started on its line; 0.42 with 0.12 s lag); XT8 |
| F2/F6 | "no X/H line carries xt" ×3 without the derate-0 exception | REAL (docs) | `motion_result.hpp`, `CompletedMotion` doc, changelog change 3 |
| F3 | the formatter's "longest line" was not the longest | REAL | worst case rebuilt (175-byte message, 1 024 lines each checked for the whole column); TC red |
| F4 | xt on CANCELLED / FAULT_ABORT unpinned | REAL | XT7 + formatter goldens; N1, N10, N11 red |
| F5 | `CompletedMotion`'s default unpinned | REAL | XT7; N19 red |
| F7 | changelog: strafeTo "unchanged on a tank" | REAL | changelog :80-81 |
| F8 | changelog: "one side 9 % weak" with a signed value | REAL | right side named, left value beside it |
| F9 | the "pin" of guide 12's rule ran with the steering off | REAL | XT8 (steering on); the 14:10 entry corrected in place |
| F10 | guide 11: "a small xt means it stopped short" | REAL | reworded: short along the line, or there but unsettled |
| F11 | "a hold of a spot away" — no public verb | REAL (wording) | "a Tier-3 `HoldPose` built with a spot away"; "the `hold` verb" |
| F12 | stale internal status; no `TANK1-COMPLETED.md` | PARTLY | status lines fixed (HANDOFF L2u, build-order); the record, pointers and briefing's OPEN line stay the coordinator's (convention: SRC1-PROGRESS :288, G1-PROGRESS :1614) |

Plus, from this run's prose review: five sentences in guide 14, HA-179, `motion_config.hpp`, `strafe_to.hpp` and
`move_to_pose.hpp` qualified (above). Mutations: 23 on fresh binaries at 17:15 (22 red, B11 the predicted
equivalent), nine of them re-run by this run with the same verdicts. Suite 1737/1737. **Not committed. No `pros`
command. No `/dev/ttyACM*`. No `src/` change, no new HA entry, no version change.**

**For the coordinator (new in this run):** (1) the stopped run's `src_build_gate.py` self-test plant in
`src/build_selection.hpp` — restored here; a gate self-test killed mid-run leaves one, so a paused tree is worth a
`git diff -- src/ Makefile` before its next build. (2) Files touched beyond the 15:07 list: `docs/guide/14`,
`docs/hardware-assumptions.md` (HA-179's clause only — COMP-t's brief :1988 adds HA-177/178 to the same file, and
its :1984 re-reads guide 14; merge both by hand), `motion_config.hpp`, `strafe_to.hpp`, `move_to_pose.hpp` (none of them COMP-t's files), and
`docs/api/{motion_config,move_to_pose,strafe_to}.md` (regenerate after the second merge). (3) At the merge:
`TANK1-COMPLETED.md`, the roadmap's "you are here", build-order's `Next:`, and the briefing last.

## Part B — the coordinator's merge and verification (2026-09-29, 10:50–11:01)

The tank1b worktree as the transport commit `920c704`, squash-merged onto `ee2e399`; one conflict (HANDOFF L2u's
status line) resolved to the merged state. Suite **1737 | 1737 passed | 0 failed | 3 skipped**, 2,650,734
assertions; `src_build_gate.py` self-test **OK (189)** and check **PASS** on all six builds; the ARM header gate,
both CI greps, `check-fresh`, removability, the staleness audit and the briefing clean (regenerated before the
build and after the suite binary).

**Identity** (the review's `rv_hash.cpp` built `-O2 -DTANK1` against the merged tree, diffed with the pristine
`a62067b` golden `hash_pre_O2.txt`): lines 1–6 (X and H drives, clean and hostile) identical; lines 7–17 (the tank
scripts: the siblings, `maxSteer = 0`, the WEIRD config, the acceptance runs) differ ONLY in the `sink` hash — the
log text, which now carries ` xt` — while `rec`, `lines`, `paces`, the pace hash and the end pose are identical.

| # | Coordinator mutation (fresh binary each; restored, hash-checked) | Observed |
|---|---|---|
| TB1 | the strafe-authority test dropped (`>= 0.0`): every drivetrain gets `xt` | RED |
| TB2 | `hadDirection()` dropped: turns and holds of here get `xt` | RED |
| TB3 | the cross-track sign flipped | RED |
| TB4 | the formatter prints the column without path data | RED (`motion_result_test.cpp:270`) |

TANK1 is complete: [`TANK1-COMPLETED.md`](TANK1-COMPLETED.md).
