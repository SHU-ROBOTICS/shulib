# RMP1-E — the mechanisms in PREVIEW (the cascade rising, the claw opening)

> Brief by the coordinator, 2026-11-19 ~19:20. **The team lead's go, 2026-11-19:** "yeah i would like that to be visible
> in the preview you can queue it" (asked: "in the preview can you see the cascade go up and down or the actual mechanism
> working?" — today it cannot: the step list and the timeline say a mechanism step was done or failed, the field shows
> only the robot from above). One run, **RMP1-E-A**. The route builder stays open to the 21st; NO library (`include/`)
> change. Base: main AFTER RMP1-C (named routes, the lift's up/down/to steps) merges — the page files are shared.

## 1. What exists

PREVIEW runs the real library on the host test plant in a Worker (the WASM engine, `tools/route_maker/wasm/`, RMP1-A2e)
and plays the run back (`src/model/playback.mjs`, `src/ui/preview.mjs`, `timeline.mjs`). The engine already runs robot
one's mechanisms on the suite's fakes (a mechanism step finishes, fails or stalls in PREVIEW), and the run record (RPV1)
names each pace's mechanism steps — but no mechanism's STATE (the lift's height, a piston's position, a motor running)
is recorded or drawn.

## 2. Decisions

- **D1 — Record the mechanisms' state per pace** in the engine's run record, from the library's simulated mechanisms
  (never the page's guess): each Lift's height (in its own units — inches for robot one's cascade) and its travel's
  bottom/top; each Arm's angle; each Pneumatic's position (its named position, e.g. claw `open`/`closed`); each Power
  mechanism running or not (and its direction). Additive to the record (an older page ignores it); the record's size
  stays bounded (sample at the playback's existing rate).
- **D2 — A mechanism strip beside the field during PREVIEW,** in step with the playback (scrubbing, play, pause, speed
  all drive it): a vertical **height bar** per lift (inches, its bottom and top stops marked, the current height as a
  number, the target of the step playing marked), a **piston** icon per pneumatic showing its named position, a
  **motor** chip per power mechanism lit while it runs (with its direction). One highlight at a time: the mechanism the
  playing step uses is the accented one; the rest are quiet (the team lead: dense screens are "overstimulating").
- **D3 — Honest about the plant:** the strip says, in one line in About this run, that heights and timings are the
  host test plant's (the fakes' gains), like the drive; a mechanism number that is a PLACEHOLDER in the robot's facts is
  marked as such (the existing PLACEHOLDER rule).
- **D4 — Robot two:** when PREVIEW plays robot two (if SM4 lands) its strip is its own; until then the strip is robot
  one's, saying so.
- **D5 — Nothing else changes:** the drive's playback, the timeline's colours, the steps list, the export, the project
  file.

## 3. Tests (adversarial — each seen RED against a mutant)

Engine (C++ / node): a route `home → up 6 in → down 2 in → back to the bottom → open claw` records heights that rise,
fall and return to the bottom row's height within the fake's tolerance, and the claw's position changes at that step;
a stalled lift (the fakes' stall) records the height where it stopped and the step failed. Page: the bar's height
follows the playback when scrubbed (seek to a time → the bar's value equals the record's sample), the piston icon
changes at the step, only one mechanism accented; light/dark screenshots at 1440 and 1280. The engine's existing
fixtures and native-twin records (CI's `ROUTE_MAKER_NATIVE_RECORDS`) re-made and compared.

## 4. Docs

Guide 22 step 6 (PREVIEW): one short paragraph + a screenshot of the strip; `docs/route-builder.md`; RMP1-PROGRESS
(`## RMP1-E`). Short and plain.

## 5. Landmines

- The engine is built by `tools/route_maker/build.py --engine build/wasm` (emscripten); CI builds the site twice
  (without, then with the engine) and its browser rows compare against the native twin's records — keep both in step.
- RMP1-D (the Controls tab layout) may be in flight on the same page: do not edit `controls_tab.mjs`, `feel_tab.mjs`;
  CSS in one delimited block of `style.css` (`/* ── PREVIEW mechanisms (RMP1-E) ── */`).
- Never run pros or touch /dev/ttyACM*.
