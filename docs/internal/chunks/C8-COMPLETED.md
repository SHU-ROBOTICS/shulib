# C8 — the manual: COMPLETED record

> Chunk C8 (WS12, M2/M7-adjacent), executed 2026-08-11. Brief:
> [`C8-user-guide.md`](C8-user-guide.md). Live log: [`C8-PROGRESS.md`](C8-PROGRESS.md).
> **Left in the working tree, uncommitted, for review** (per the brief).

---

## 1. What shipped — the file map

`docs/guide/` — 15 files, 2,359 lines / ~22,200 words of chapters + maintenance README, plus
one test file:

| File | Part of the brief | Lines |
|---|---|---|
| `docs/guide/README.md` | reader map + the maintenance note | 92 |
| `01-what-is-this.md` | Part 1 — orientation, VEX U, the autonomous problem, honest status | 100 |
| `02-the-field-and-coordinates.md` | Part 2 — coordinates, pose, **field- vs robot-relative** (the worked heading-flip example gets the chapter's second half) | 136 |
| `03-knowing-where-you-are.md` | Part 2 — odometry, why it drifts, per-sensor knowledge, fusion/gating/never-snap | 155 |
| `04-drivetrains.md` | Part 2 — holonomic vs tank vs H, kinematics, desaturation, strafe authority | 102 |
| `05-getting-there.md` | Part 2 — control loops, PID without anxiety, feedforward, settling's three conditions, the watchdog | 131 |
| `06-how-things-fail.md` | Part 2 — drift/slip/dead sensors/brownout/jams + the detect–name–contain–record–continue pattern | 138 |
| `07-getting-set-up.md` | Part 3 — prerequisites, build, what the test output means, layout | 121 |
| `08-your-first-routine.md` | Part 4 — the tutorial, every line explained, real transcripts | 369 |
| *(09 — deliberately vacant)* | reserved for D1's recipe-API chapter (see §5) | — |
| `10-the-api.md` | Part 5 — the API as prose, gotchas, **F6-not-frozen notice up top** | 242 |
| `11-reading-the-diagnostics.md` | Part 6 — every line type field-by-field; **all 10 raisable fault codes** with cause + action | 247 |
| `12-when-things-go-wrong.md` | Part 7 — symptom-first, organized around the motion-vs-estimate fork | 160 |
| `13-extending-the-library.md` | Part 8 — layers, testing bar, add-a-drivetrain (oracle warning), add-a-motion | 128 |
| `14-what-it-cannot-do-yet.md` | Part 9 — built from C1–C7's "NOT known" sections + roadmap/register links | 112 |
| `15-glossary.md` | Glossary — 71 terms, one sentence each, chapter-referenced | 126 |
| `test/guide_examples_test.cpp` | ⭐ the compiled examples (see §2) | 422 |

Also changed: `README.md` (the guide is now the first "Where to go next" entry; the stale
"a proper user guide is planned" bullet replaced) and `docs/roadmap.md` (you-are-here + an M7
checkbox added at C8 — see §6).

## 2. Where the examples live, and how they're verified

**`test/guide_examples_test.cpp` — 8 cases / 41 assertions, all green.** Mapping: `guide-08a`
(the tutorial's full wiring + routine, with the chapter's behavioral claims held as
assertions — settled exits, ground-truth landing within 1 in / ~2°, the exact landmark lines
in the transcript, the `n/a` honesty), `guide-08b` (same run with the tick stream tapped —
asserts >50 stamped tick lines and that `n/a` disappears), `guide-08c` (starved timeout →
`TimedOut`, motors at 0 V, `✗TIMEOUT` in the transcript), `guide-10a`–`10e` (options,
tank turn-then-drive idiom, `TrajectoryResult`, `drive()`+Frame, `waitUntil`).

The anti-rot mechanism, stated in all three places it needs stating (the test file header, the
guide README, chapter 8): **the chapters quote the test bodies verbatim**; a change on either
side must be mirrored, and CI compiles + runs the file, so an example that stops building or
stops behaving turns the suite red. Transcript excerpts in chapters 8/11 are pasted from real
runs (regenerate: `SHULIB_GUIDE_PRINT=1 ./build/test/shulib_tests -tc='guide-*'` — the print
hook is env-gated so CI stays quiet). Chapter 11's formats additionally lean on the existing
byte-pinned diagnostics tests, which the chapter names.

## 3. Verification actually performed

- **Suite:** `cmake --build build/test && ./build/test/shulib_tests` →
  **667 cases / 915,611 assertions / 3 deliberate skips, 0 failed** (was 659 / 915,570;
  +8 cases / +41 assertions are the guide examples). ⚠ Honest footnote: the *pre-existing*
  assertion total moves ±6 with the configure-time build hash (915,564 without the guide cases
  vs the documented 915,570) — some hash-length-sensitive assertion count, not a behavior
  change; noticed at C8, not investigated further.
- **Both CI guards** re-run locally: PROS-free (all `include/shulib/`) PASS; core-never-
  includes-sim PASS.
- **ARM gate:** all 102 headers as one TU, Cortex-A9, strict flags — CLEAN. (Note: RESUMING's
  one-liner needs `sed 's|^include/||'` when `find` is run from the repo root with a relative
  path; the `.*/include/` pattern only matches absolute paths.)
- **The tutorial was followed start to finish as written**: chapter 7's commands verbatim
  (fresh configure → build → run, green); chapter 8 path 1 (all three `guide-08*` print
  commands, output matches the chapter's transcripts); chapter 8 path 2 **actually typed in**
  as `test/my_first_auton_test.cpp` per the chapter's instructions — compiled first try,
  passed, then deleted. Found & fixed while doing it: the build auto-discovers new test files
  (`CONFIGURE_DEPENDS`), so "re-run configure" was wrong as stated; corrected in chapters 8
  and 12.
- **Link check, mechanical:** 164 relative links + anchors across the 15 guide files + the
  README's guide links — **all resolve** (script in the progress log; GitHub-style slug
  matching for anchors).
- **Removability (C7's property):** a full mv-`docs/internal`-aside pass shows every public link
  still resolves with internal absent — which is the property that matters, and it holds.
  ⚠ **Corrected at D1 (2026-08-11):** this bullet originally claimed the grep
  (`docs/internal|internal/|chunks/|RESUMING|build-order|-COMPLETED|-PROGRESS|guide-maintenance`
  over README + test/README + docs/*.md + docs/guide/*.md) was **empty**. It was not, and it was
  not empty at C8's own commit either (`3437d72`): it returns **5 hits** — two in `roadmap.md`,
  one each in `hardware-assumptions.md`, `diagnostics-plan.md`, `legacy-command-vocabulary.md`.
  All five are **prose mentions** of completion-record section names ("C2-COMPLETED §Mutations"),
  **not markdown links**, so nothing breaks when `docs/internal/` is dropped — the property was
  always sound; the stated evidence for it was wrong. The narrow four-term check
  (`internal/|chunks/|RESUMING|build-order`) *is* empty, and `docs/guide/` is clean under both;
  that is most likely the check actually run. Use the four-term grep as the gate and treat prose
  mentions as a release-review nit, not a breakage.

## 4. Decisions recorded (alternatives were viable)

1. **Part 2 split across five files (02–06)** instead of one concepts file — a person updating
   one topic edits one file, and field-vs-robot-relative gets the "real space" the brief
   ordered without burying the sensor material.
2. **Chapter slot 09 reserved (vacant)** for D1's recipe API, over contiguous numbering with
   future renumbering, and over decimal suffixes. Implements the brief's "D1 adds a file
   rather than forcing a rewrite" and `guide-maintenance.md`'s own "prefer inserting at a gap".
   ⚠ Small conflict left for the maintainer: `guide-maintenance.md`'s checklist line "chapter
   numbering is contiguous" — the reserved gap violates it literally. The guide README, ch. 8's
   footer, and the D1 sweep row all explain the gap; suggest amending that checklist line to
   "…contiguous except documented reserved slots" when convenient.
3. **Tutorial wiring is the longhand standalone recipe** (mirroring `chassis.hpp`'s §16.2
   promise and the C4 standalone test) rather than the test suite's `ChassisRig` helper —
   longer, but the wiring *is* the architecture lesson, and it's the same shape as
   `src/main.cpp`. The rig appears only in the later, non-tutorial cases.
4. **Two-stage diagnostics in the tutorial** (RunReporter direct to TermSink first — result
   lines honestly `n/a` — then the tick stream via a forwarding tap): turns the sim-only
   construction-order wart into a teaching beat about the library's no-invented-data rule,
   instead of hiding it.
5. **Env-gated transcript printing** (`SHULIB_GUIDE_PRINT=1`) over always-printing (CI noise)
   or doctest-flag plumbing — the reader gets a real, personal transcript; CI stays byte-quiet.
6. **`n/a`-first honesty kept in the base tutorial** rather than wiring full data immediately —
   deliberate: the reader's first contact with a surprising output is one the guide explains in
   the same breath.
7. **README "user guide is planned" bullet replaced, not appended to** — a stale promise
   sitting above a link to its fulfillment would be exactly the rot the guide preaches against.

## 5. Deferred, honestly

- **To D1:** chapter 09 (the recipe API), its `guide-09*` example cases, chapter 8's closing
  pointer update, guide README table row.
- **To D2:** removing the not-frozen notices (ch. 10 top, ch. 14) — they must stay until then.
- **To D3 (needs D1):** the recipe cookbook and the generated/published API reference; also any
  "10 lines to a routine" quick-start, which without the recipe API would today be a lie.
- **To G4:** the VexBuilder-based "first auton in 10 minutes" flow (build → export → drag →
  run) — D3's brief already excludes it for the same reason.
- **To R3:** the guide's many "never run on a robot" statements are a *sweep set* —
  `guide-maintenance.md`'s R3 row covers it; ch. 1, 3, 7, 12, 14 + README all carry instances.
- **Not done, no owner yet:** validation by an actual new reader. Chapter 14 says this about
  itself; M7's DoD line stays open in the roadmap checkbox added at C8.

## 6. The documentation contract (all six)

1. **Roadmap checkbox** — a user-guide item added under M7/WS12 (marked "*item added at C8*"),
   flipped `[x]` with evidence (file set, test case/assertion counts, date) and an honest scope
   note keeping M7's DoD open.
2. **"You are here"** — C8 paragraph appended to the roadmap blockquote (done + not-done,
   suite counts, the ±6 caveat), `Next: D1` unchanged.
3. **Design notes** — the guide README's maintenance section and the test file's header carry
   the why (anti-rot rule, reserved slot, quoting discipline); chapters carry per-chapter
   assumption headers.
4. **Test evidence** — §2/§3 above; each guide case's header comment names the chapter claim it
   holds. (No mutation checks: this chunk's "load-bearing logic" is the guide–test verbatim
   coupling, which has no mechanical checker yet — an honest gap; a future diff-based
   listing-extraction check would close it.)
5. **Decisions** — §4 above.
6. **Freeze Register** — untouched: C8 froze nothing. The guide *documents* F6's unfrozen
   status; the register rows stand as they were.

## 7. Interaction with the mid-chunk commit

`eded9fd` (`docs/internal/guide-maintenance.md`) landed while C8 ran. The guide as built
conforms to it (verbatim-example rule, link-don't-restate, no internal links, re-captured
transcripts, "run the commands as written" — done literally). The one flagged tension is §4
item 2. The public guide never references the internal file, by design; the two maintenance
documents split reader-facing (public, survives the squash) vs process (internal), per that
file's own header.
