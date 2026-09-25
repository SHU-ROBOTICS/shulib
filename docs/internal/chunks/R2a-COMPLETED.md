# R2a — COMPLETED (landed 2026-09-24; build-gated, NOT run on a robot)

> Bench Tests' stations **11 DISTANCE** and **12 AI VISION**, and the menu that makes room for them.
> Written FROM [`R2a-PROGRESS.md`](R2a-PROGRESS.md) (the executor's live log, and the coordinator's
> verification entry at its end), not instead of it. Brief:
> [`R2a-sensor-stations.md`](R2a-sensor-stations.md) (committed `10de29f`). Executor model: `opus`.
> Opened by the team lead's question, 2026-09-24: *"how do we know the distance sensors and the ai
> visions sensors are working? is there logging or can I see the camera directly like what it
> sees?"* — and his "yeah add that".

## The one-sentence honest status

Bench Tests can now SHOW what robot two's distance sensors and AI Vision camera report — raw PROS
values beside the library adapter's, the camera's detections as PROS lays them out, a RECORD button
that writes them to serial and the SD card — and **neither station has ever run on a robot**: no
screen string has been seen on a V5, no RECORD line exists, and HA-113 / HA-114 / HA-115 have not
moved.

## Built, file by file

- **`src/bench_r3a.hpp`** — now PROS-free and carrying the menu's pure GEOMETRY (`menu_grid`):
  `Spec`, `Rect`, `buttonBox`, `extent`, `hitTest` generalized from two hard-coded columns to N.
  The rules kept and now tested: no dead gap inside the grid (a gap pixel belongs to the button
  above / to the left), dead header, margins and bottom strip (so (0,0) and an error read's −1
  launch nothing), the right edge from the RIGHTMOST column (not the last button — on a partial row
  that idiom kills the right column). `kMenu` = 12 buttons, 3 × 4, 46 px tall (the old were 36);
  `kStationRow` / `kStationRowHit` for the stations' RECORD / DONE / FAMILY row (the hit area 8 px
  taller, upward only).
- **`src/bench_r3a.cpp`** — stations 11 and 12 (read-only apart from the camera's own detection
  settings, which station 12 writes and says so); the menu drawn from `menu_grid` with two MEDIUM
  label lines per button and compile-time budget checks on every label; `panelSafe()` /
  `screenText()` as the one sanitising print path for new text and for the scrolling log and header
  (a `%` or a UTF-8 dash could reach the panel before — PROS re-reads screen text as a format);
  `E_TOUCH_ERROR` reads no longer taken as taps in the menu, `waitForTouch` or the new stations.
  Stations 1–10 keep their numbers and their logged names.
- **`test/bench_menu_grid_test.cpp`** — 9 cases, adversarial: every pixel of every button, every
  dead-zone pixel, the gap rule, row-major order, a pixel-for-pixel oracle against the shipped 2 × 5
  hit test, a partial last row, the stripe / label / dot layout inside a button, the station row.
- **Worksheet** — Station A shows the 3 × 4 menu; new **Station H** (distance: 6 / 12 / 24 in per
  side and open space, the screen table, what to send back) and **Station I** (AI Vision: a game
  piece at about 1 and 2 ft, left then right; VEX's AI Vision Utility for the video; a tag with the
  FAMILY cycle; a power-cycle check of whether the camera keeps its settings).

## Rulings beyond the brief (each with its alternative in the log)

- **FAMILY uses `aivision_set_tag_family_override`**, not the brief's `aivision_set_tag_family`:
  the vendored header says the latter "will allow multiple apriltags", so cycling would pile the
  families up. The station sets no family until FAMILY is first pressed ("as found" is evidence).
- **The change timer hashes every detection's bytes**, not just the count, so it times the
  camera, not the operator.
- **Buttons show shortened two-line text; the log keeps each station's full name** (the worksheet
  and every earlier log quote the names).
- **The stations' verdict is DONE (`Sev::Info`, a white dot) at best, never PASS**; CHECK on a
  missing sensor or an error.

## Verification

The executor's three gate passes, then the coordinator's own (R2a-PROGRESS, last entry): suite
1,425 cases / 2,386,871 assertions green; six builds PASS with 0 disallowed warnings; ARM header
gate, CI greps, API freshness, briefing, removability all clean; four hand mutations of the grid
(gap rule, extent idiom, bottom bound, station-row hit area), each red with a confirmed relink,
each restored with `cmp`. The PROS shim has none of the screen, AI Vision or distance C calls, so
the stations' own logic is compile- and review-checked only — the grid is the only host-tested
piece.

**Coordinator deviation, logged:** R2a moved the tester's registry comment (lines 112–122 → 135–145),
which four comments in files outside the brief cited by line; they now cite it by its anchor (the
comment just above `kMaxRegistryIndex`) — comment-only edits in
`include/shulib/hal/pros/device_registry.hpp` and `test/pros_device_registry_adapter_test.cpp`, and
the regenerated `docs/api/pros-device_registry.md`.

## What is NOT done

- **Nothing here has run on a robot.** The stations run at the next bench session (worksheet
  Stations H and I; `build-order.md`'s first `Next:`).
- The AI Vision adapter and any interpretation (bearing, range, frame) — **R2b**, written from the
  RECORD lines. `localization::TagMap`'s refusal of duplicate ids (the Override field repeats ids
  1–4) is R2b's / E3's (handoff).
- Colour signatures, the wall reset (L6), the distance beliefs HA-113…115 (the bench session that
  reads the RECORD lines settles them).
