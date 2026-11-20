# RMP1-K — see the mechanisms work: a bench check of every mechanism step, and the real run read back onto the steps

> Brief by the coordinator, 2026-11-20 ~09:30. **The team lead, 2026-11-20:** "the biggest thing really is the
> mechanisms we need to easily be able to make sure they work and are in the routes". One run, **RMP1-K-A**, built ON
> RMP1-I (the local helper `tools/route_builder.py`, `files_local.mjs`, Send to robot — read its brief and code
> first). Competition SATURDAY Nov 21. ONE library change: §2 D4's per-step log line (the team lead's freeze exception, 2026-11-20); nothing else in `include/`. The exported C++ of
> the team's routes stays byte-for-byte (the RMP1-C goldens).

## 1. What exists

- PREVIEW runs the real library on the host test plant and draws the mechanisms (RMP1-E) — a MODEL: it proves the
  order, timing and heights, never that the claw is plumbed or the cascade reaches 18 in.
- On the robot, the Auton Tester prints per run (real lines: `.claude/coordinator/robot_logs/robot_one_2026-11-19.log`):
  `[ATS] run 3: 'skills' (SKILLS, …) -- RUN touched; …`, `[WARN][RTN] routine 'skills' STOPPED at step 2 (driveTo):
  motion TIMEOUT — skipping the rest; …`, `[RTN] routine 'skills': step 3 (reverseTo) skipped — stopped at step 2`,
  `[ATS] run 3: routine STOPPED, 1 of 23 steps done, stopped at driveTo (motion TIMEOUT); faults this run: 6, …`,
  `[ATS] run 3: … END estimate (52.51, -13.32) facing -117.4 deg …`, and `[MECH]` / `[ERROR]` lines. A step that
  SUCCEEDS prints nothing of its own today. Find every format in the code (`include/shulib/chassis/routine.hpp`,
  `include/shulib/competition/`, `src/programs/auton_tester_program.cpp`, the mechanism service) — never guess one.
- The brain's USB: the SYSTEM port (Linux `/dev/ttyACM0`, `if00`) is where `pros upload` talks; the USER port
  (`ttyACM1`, `if02`) carries the program's output (`pros terminal <port>`). An open terminal must be closed before
  an upload and reopened after (the coordinator's practice: pause it, upload, resume).

## 2. Decisions

- **D1 — Check the mechanisms (bench):** on a local page (RMP1-I), the route's menu gets **Check the mechanisms**:
  the page builds, from the selected route, a CHECK route — every mechanism step in order with its own numbers and
  waits, every drive/turn/curve step left out, a home first when the route's first cascade move needs one — named
  `"<route> mechs"`, and sends it like Send to robot (the team's routes plus this one; the project file never
  holds it; the next ordinary Send leaves it out). One screen says what will move ("the cascade homes, goes to 6 in,
  then 18 in; the claw closes, then opens; the right toggle runs 0.8 s") and "the robot stays where it is — clear
  the mechanisms' paths". Its exported C++ is the ordinary export of those steps (the same generator, no new step
  kind); a test pins that the team's routes' C++ is unchanged by its presence.
- **D2 — Watch the run:** after a Send (or from the route's menu: **Watch the robot**), the helper opens the brain's
  USER port and streams its lines to the page; the step list shows the run as the robot reports it — each step done
  (green), the step that stopped it (red, with the robot's own words: "motion TIMEOUT", "STALLED", "NOT_HOMED", a
  carry-on failure in amber), the skipped ones (grey) — and one summary line ("1 of 23 steps done — stopped at step 2,
  driveTo: motion TIMEOUT"; the END estimate drawn on the field beside the plan). From today's lines alone the page
  ticks each step LIVE from D4's line as it ends; a run from an older build (no D4 lines) is read at its end from the
  "k of N steps done" count, and the UI says which it is doing. Raw lines in a "Show the robot's log" fold, and saved with the run to
  `build/robot_runs/<time>.log` for the coordinator.
- **D3 — The port, safely:** find the brain's user port the way `upload_auton.py` finds the system port (its
  `pros lsusb` parse; Windows: the second V5 COM port) — never a guess; one reader at a time; the reader closes
  before any upload (Send, Check) and reopens after; a controller-only USB is said plainly; the reader is the
  helper's child and dies with it.
- **D4 — Per-step live lines (GRANTED: the team lead's second freeze exception, 2026-11-20, "Yes, live ticks"):** `Routine` logs one `[RTN]` line as each step ends (its number, verb, outcome, seconds; a mechanism
  step's end position), so the page can tick steps live. Additive logging only, its format pinned by a test; bump kApiMinor (2.11) with the changelog, as 2.10 did; every routine golden that hashes log output re-checked (a log line must not change any motion).

- **D5 — What is on the robot (the team lead, 2026-11-20: "how do we check what we want and have on our robot?"):**
  a **Check robot one** (two) button (and `python3 tools/route_builder.py robot one`, a VS Code task "What is on
  robot one") reads the brain's program list over the SYSTEM port (`pros v5 ls-files` and whatever per-slot metadata
  pros-cli can read — find out which from pros-cli 3.5.6's own source, never guess its output) and shows the WANTED
  table beside what is there: slot 1 shulib Drive, 2 shulib Competition, 3 Bench Tests, 4 shulib Calibrate, 5 the
  team's old code (never touched), 6 shulib Auton Tester, 7 shulib Tune — each ✓, or ✗ with the plain fix ("slot 2
  holds shulib Auton Tester: upload Competition there"; "slot 6 is empty"). Where pros-cli can read it, each slot's
  upload time; and `tools/upload_auton.py` (and every upload command the docs print) passes `--description` with the
  build's git hash and the route names, so the check can say which build and routes each slot carries. Read-only: it
  never removes or uploads anything.

## 3. Tests (adversarial — each seen RED against a mutant)

Python self-test of the helper's new endpoints (D5's table against a fake `pros v5 ls-files` with every slot right, one wrong, one empty, the old-code slot left alone; a fake `pros terminal` replaying the real 11-19 log lines, a reader
closed during a Send, a port that vanishes mid-run, a second reader refused). Node: the CHECK route's steps from
routes with every mechanism step kind (and none — the button says there is nothing to check), starts/waits kept,
drives dropped, the home added only when needed; the line parser over the real logs (stopped, skipped, carry-on,
"k of N", END estimate, garbled and partial lines). Page (Firefox AND Chromium): Check sends the extra route and the
next Send leaves it out; a replayed run paints the steps and the summary; screenshots light/dark at 1440 and 1280.

## 4. Docs

Guide 22: a short "Check the mechanisms, then watch the run" section with screenshots (one idea per screen); the
changelog; RMP1-PROGRESS (`## RMP1-K`).

## 5. Landmines

- Never run the REAL pros or touch /dev/ttyACM* — fakes only; the coordinator tests on the robot with the team
  lead's OK.
- RMP1-J (mechanism steps made simple) may be merging beside you: keep to the route menu, the step list's run marks
  (one delimited block), new modules (`src/ui/run_view.mjs`, `src/model/check_route.mjs`, `src/model/run_log.mjs`),
  the helper, and ONE delimited block of `style.css` (`/* ── Mechanism check and run view (RMP1-K) ── */`).
