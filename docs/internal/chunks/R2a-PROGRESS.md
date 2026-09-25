# R2a — progress log (live)

- **Chunk:** R2a — Bench Tests stations 11 DISTANCE and 12 AI VISION
- **Brief:** [`R2a-sensor-stations.md`](R2a-sensor-stations.md)
- **Started:** Thu Sep 24 06:10:28 PM EDT 2026
- **Tree at start:** `10de29f` on `shulib-v2`, clean
- **Executor model:** opus
- **Go:** the team lead, 2026-09-24. Do not commit; no robot upload; no `pros` commands.

Appended in real time. Nothing in this chunk has run on a robot.

---

## Log

### 18:10 — start
- `date` → Thu Sep 24 06:10:28 PM EDT 2026; `git status --short` empty; HEAD `10de29f`.
- Read `docs/internal/RESUMING.md` and the brief in full. This log created before any code.

### 18:10–18:35 — required reading, done
- `src/bench_r3a.cpp` (all 2101 lines) and `src/bench_r3a.hpp`: the station pattern (`rule()` →
  `drawHeader` + `screenClear`, `emit`/`emitf`/`emitS` to serial + SD + the scrolling panel,
  `g_screenActive = false` for a fixed live layout, a touch on `release_count`), `MenuItem`/`kMenu`,
  `kMaxMenu = 10`/`g_verdict[10]`, `buttonBox`/`hitTest` (2 × 5, `kBtnW 232 kBtnH 36 kBtnX0 6
  kBtnY0 38 kGap 4`, last row ends at y = 234), `drawMenu` (label MEDIUM at x0+10,y0+10; status dot
  at x1-16), `touchReadout`, the census kind words (`E_DEVICE_AIVISION` → `AI-VISION`).
- `include/shulib/hal/pros/distance.hpp` + `hal/distance.hpp`; HA-113/114/115
  (`docs/hardware-assumptions.md:200-202, 765-804`).
- Vendored PROS: `ai_vision.h` (all), `ai_vision.hpp` (all), `distance.h`, `screen.h`, `apix.h`
  (registry), `error.h`.
- `src/calibrate_program.cpp` header (:1-70) and `printText` (:254-296): the ONE path to the
  screen — `%` / control → `?`, a UTF-8 em/en dash → `-`, other non-ASCII → one `?`, cut at the
  size's budget, padded, printed through a literal `"%s"`; PROS 4.2.2's `print_at` LEAKS each
  print (redraw on change, ≤ 5 Hz).
- `include/shulib/calibration/calibrate_panel.hpp:43-107`: the budgets LARGE 18 / MEDIUM 36 /
  SMALL 50 per 472 px (ESTIMATES, HA-136) — and R3d-PROGRESS:1953-1978, which derives them from
  lines READ WHOLE on a V5: MEDIUM 36 from x 8 is the widest proven line, so **≤ 13.1 px per MEDIUM
  character is a proven upper bound**; SMALL 54 from x 8 was read with no complaint.
- `test/pros_shim/` (listed; `distance.hpp`, `apix.h`, `shim_control.hpp` read).
- Worksheet: Station A's menu table (:46-80), Station G (:597-875), DO NOT (:878-895).
- R3d-PROGRESS Part 3 (:1380-1491): sensors on 3/4/5 (the calibrate census printed `VISION 3 ·
  DISTANCE 4 · DISTANCE 5` — its library-side kind word folds both vision sensors into VISION,
  landmine 3), omni wheels, the screen is resistive and reads on RELEASE.
- `build-order.md` (the first `**Next:**` at :632, R2's entry at :1668-1674), `roadmap.md` (the R2
  row at :1024, "You are here" at :154), `HANDOFF-remaining-work.md` L3 (:324-342).

### 18:23–18:34 — baseline, clean tree `10de29f`
- The first `cmake --build build/test` stopped at the DOC GATE: `PROJECT-BRIEFING IS STALE` — caused
  by THIS log (a `-PROGRESS.md` makes R2a an open chunk in the briefing's generated block).
  Regenerated with `python3 tools/briefing_status.py generate` so the suite could build (it is
  regenerated again LAST). Then `cmake --build build/test` (every object rebuilt) →
  **`test cases: 1416 | 1416 passed | 0 failed | 3 skipped`, `assertions: 2386604 | 2386604 passed`.**
- `python3 tools/src_build_gate.py check` → **PASS**, all six builds: bench / tank / tank-drive /
  tank-library / tank-calibrate `2 allowed (-Wunused-function), 3 vendor-ignored, 0 disallowed`,
  xdrive `0 allowed`; `4/4 TUs compiled`; hash asserted; beacon checked. (Output kept in the
  scratchpad.)

### 18:35 — findings from the reading that shape the work (none a stop)
1. **The shim cannot exercise either station.** `test/pros_shim/` has `pros::v5::Distance`
   (`get_distance`, `get_confidence` only) and the registry; it has NO screen API, NO AI Vision API,
   NO distance C API (`distance_get`, `_get_confidence`, `_get_object_size`,
   `_get_object_velocity`), and none of the other C APIs `bench_r3a.cpp` calls (`imu.h`, `motors.h`,
   `misc.h`). The stations live in that PROS-bound TU; compiling it on the host would need the shim
   extended far beyond "what these stations need" (brief §4). NOT extended. The one pure piece —
   the menu grid — IS pulled out and host-tested (below).
2. **A latent defect in the tester, found while reading: `screenEmit()` prints through `"%s"` a
   line that can itself contain `%`.** Station 10's line `"  <25% of side max for 250ms; ..."`
   (`bench_r3a.cpp:1385`) reaches the panel; PROS hands the formatted text to VEXos AS A FORMAT
   (R3d F2's finding), so `"% o"` is re-read as a conversion with no argument. Adapter refusal texts
   (`e.what()`, `emitS(... %s, e.what())`) carry UTF-8 em dashes that VEXos's fonts cannot draw.
   Ruling below (R2a-R3).
3. **A second latent defect: an `E_TOUCH_ERROR` read carries −1 in every field** (vendored
   `screen.h:726-729`). The menu loop treats `release_count != seen` as a new tap, so an error read
   sets `seen = -1` and the NEXT good read is taken as a new tap at the STALE x/y — a station can
   launch without a tap. Ruling below (R2a-R4).
4. **`aivision_set_tag_family` ADDS a family; `aivision_set_tag_family_override` REPLACES**
   (`ai_vision.h:305-335`; `ai_vision.hpp:316-344`: "If override is true, the AI vision sensor will
   only look for the given family. Otherwise, it will add the given tag to the list"). A cycle built
   on the brief's `aivision_set_tag_family` would ACCUMULATE families and could never say which one
   sees a tag. Ruling R2a-R6.
5. **The tester's own lines are NOT `[R3A]`-prefixed.** Only `src/main.cpp`'s boot lines are
   (`:928-937`); `emit()` prints raw. The brief's "like the tester's other lines" is therefore
   met by prefixing the new RECORD lines explicitly, not by an existing convention. Ruling R2a-R7.
6. **PROS's C doc says what confidence returns at or below 200 mm**: `distance.h:69-71` — "Confidence
   is only available when distance is > 200mm (the value 10 is returned in this scenario)". The C++
   doc (and HA-115) call that value UNKNOWN. Not acted on (no HA moves in R2a — S1); recorded here
   and in the worksheet so the RECORD lines settle it.

### 18:37 — design, and every ruling beyond the brief (each with its alternative), BEFORE code

**R2a-R1 — the menu is 3 columns × 4 rows of 46 px buttons.** `kBtnX0 6, kBtnY0 38, w 153
(x1 = x0 + 153), h 46 (y1 = y0 + 46), gap 4`: columns 6–159 / 163–316 / 320–473 (6 px margins both
sides), rows 38–84 / 88–134 / 138–184 / 188–234 (the last row ends at y = 234, like today's). Row
major, like a phone keypad: `1 2 3 / 4 5 6 / 7 8 9 / 10 11 12`. The buttons get TALLER (46 vs 36): the
space under a 3 × 4 grid is there, and a resistive screen whose release point wanders wants it.
Rejected: 2 × 6 (29 px tall — below the brief's 36); 4 × 3 (114 px wide — 8 MEDIUM characters); 3 × 4
at 36 px (ends at y 194 and wastes 40 px of target).

**R2a-R2 — the button shows TWO MEDIUM lines; the LOG keeps the full label.** `MenuItem` gains
`btn1`/`btn2`; `label` stays exactly as it was for 1–10 (it is what `>>>>>> <label>` writes to
serial and the SD log, and what the worksheet and every earlier log quote). Budgets from the
geometry and calibrate's proven-upper-bound width (472 px / 36 MEDIUM characters): line 1 from
x0+8 to x1−1 = 145 px = **11 chars**; line 2 from x0+8 to 3 px short of the status dot = 123 px =
**9 chars**. Both `static_assert`ed, with ASCII-only and no-`%`. Two MEDIUM lines at 22 px pitch is
hardware-proven legible (the IMU rotate screen: y 44 and 66). Rejected: (a) one shortened label for
both screen and log — it would break `grep` across every earlier session's log for no gain; (b)
MEDIUM + SMALL second line — SMALL at arm's length. Shortenings (1–10 keep their numbers):

| # | log `label` (unchanged for 1–10) | button line 1 | button line 2 |
|---|---|---|---|
| 1 | `1  DEVICE CENSUS` | `1 DEVICE` | `CENSUS` |
| 2 | `2  IMU + ROTATE` | `2 IMU +` | `ROTATE` |
| 3 | `3  MOTOR WATCH (live)` | `3 MOTOR` | `WATCH` |
| 4 | `4  BATT/CTRL` | `4 BATT/CTRL` | — |
| 5 | `5  SD CARD PROBE` | `5 SD CARD` | `PROBE` |
| 6 | `6  LOOP RATE` | `6 LOOP RATE` | — |
| 7 | `7  RUN ALL` | `7 RUN ALL` | — |
| 8 | `8  MOTORS (static)` | `8 MOTORS` | `STATIC` |
| 9 | `9  SCREEN RULER` | `9 SCREEN` | `RULER` |
| 10 | `10 DRIVE (POWERS)` | `10 DRIVE` | `POWERS` |
| 11 | `11 DISTANCE` (new) | `11 DISTANCE` | — |
| 12 | `12 AI VISION` (new) | `12 AI` | `VISION` |

**R2a-R3 — `screenEmit()` and `drawHeader()` print through ONE sanitising printer** (finding 2):
`%`/control → `?`, em/en dash → `-`, other non-ASCII → `?`, cut at the budget, padded, literal
`"%s"` — calibrate's `printText`, re-implemented here (the two programs share no TU). Same cut
widths as before (54 SMALL for the scrolling log, 30 MEDIUM for the header), so nothing that
printed legibly changes. Rejected: leave the defect for a later chunk — RESUMING's "a chunk that
finds a flaw fixes it there", and the new stations print through the same two functions.
**The other existing prints are NOT converted** (the ruler, the touch readout, the splash, the
verdict chip, MOTOR WATCH/DRIVE cells, the IMU screens): each prints only literals and numbers, so
no `%` or non-ASCII can reach the panel; they have run on hardware; converting them is churn with
no defect behind it.

**R2a-R4 — an `E_TOUCH_ERROR` read is ignored** (finding 3) in the menu loop's tap logic (it is
still shown by the touch readout — it is evidence), in `waitForTouch()`, and in both new stations.
NOT changed: the two-button prompts (they already require `y >= 60`, which −1 fails); DRIVE's
exit-on-any-release (an error read ending DRIVE is the SAFE direction); MOTOR WATCH's
exit-on-release (an early end records what it has — benign). Listed, not touched.

**R2a-R5 — the grid lives in `src/bench_r3a.hpp` as PROS-free constexpr geometry**
(`shulib::bench::menu_grid`), host-tested by `test/bench_menu_grid_test.cpp` through a relative
include — the `chassis_table.hpp` precedent. The brief's rule: touch only `bench_r3a.{cpp,hpp}` in
`src/`, and a host test only if it needs nothing in `include/`. Rejected: a new `src/` header
(outside the brief's file list); a written walk only (the brief prefers the test when possible).
The generalization carries one trap the old 2-column code could not show: the grid's right edge
must come from the RIGHTMOST COLUMN, not the last button — on a partial last row the last button
is not in the rightmost column, and the old `buttonBox(kMenuCount - 1)` idiom would make the whole
right column dead. Tested with an 11-button grid.

**R2a-R6 — TAG FAMILY uses `aivision_set_tag_family_override`** (finding 4) so exactly ONE family is
active after each press. And the station does NOT set a family at entry: until FAMILY is first
pressed the button reads `as found`, so a tag seen before any press is evidence of the SENSOR'S own
setting. A family chosen earlier in this power cycle is RE-APPLIED at entry (and said), so the
button never disagrees with the sensor. First press → CIRCLE21H7, then 16H5, 25H9, 61H11 (PROS's
names), round. Rejected: `aivision_set_tag_family` (accumulates); setting 21H7 at entry (destroys
the as-found evidence and writes a setting nobody asked for).

**R2a-R7 — the `[R3A]` prefix** (finding 5) goes on every line these stations print that is a FACT a
later reader will grep for: the RECORD lines, the discovery line, each settings write and read-back,
the class-name table, every touch, the exit summary. Prose lines stay unprefixed like the rest of
the tester. Rejected: prefixing every `emit()` — it changes every line of every existing log and
costs the panel mirror 6 of its 54 characters.

**R2a-R8 — the read period is timed on the WHOLE detection set, not the count.** A signature
(FNV-1a) over the count and every object's 18 bytes, polled every 10 ms; each change records the
interval since the last; the panel shows the mean of the last 16 intervals, the minimum since
entry and how many changes. Rejected: the count alone (the brief's parenthesis) — it changes only
when a thing enters or leaves the view, which times the OPERATOR, not the sensor. The panel says
"no change for N s" when nothing changes (a still scene may never change); the worksheet says to
move the piece.

**R2a-R9 — every AI Vision RECORD line carries the object's 18 raw bytes in hex and a `t <ms>`
timestamp.** The header does not say which union member a CODE uses, and R2b must not need a second
bench session to find out; the timestamp lines a RECORD up with the USB video. The screen shows
COLOR/CODE with the color layout, OBJECT with the element layout, TAG with its corners — each the
header's own field names, nothing converted (a COLOR's `angle` is printed raw: the header says
"tenths of a degree", and dividing by ten would be a belief).

**R2a-R10 — the stations' buttons ARM after 0.5 s** (drawn grey and dead until then; a touch in that
time is logged `ignored (arming)`) — the calibrate panel's rule (HA-136's `kArmingDelaySeconds`).
Why here: menu button 12 is exactly where station 12's DONE is, and menu button 11 exactly where the
stations' RECORD is (bottom row, same three columns), so a reflexive double tap on the menu would
otherwise exit or record at once.

**R2a-R11 — the verdict is DONE at best, never PASS**: the stations judge nothing. CHECK (amber) when
no sensor of the kind is in the registry, any read returned `PROS_ERR`, or a settings write failed.

**R2a-R12 — ports are discovered at station ENTRY** (a fresh 0–20 registry scan, index + 1), not
from the boot census's `g_found` — a sensor plugged in after boot is found, and the census's array
is not touched. Station 12 drives `E_DEVICE_AIVISION` ports ONLY; an `E_DEVICE_VISION` port (the
legacy sensor, code 11) is reported as "not driven by this station" (landmine 3). With more than one
sensor of a kind, every one is set up and RECORDed; the panel shows two distance sensors and the
first AI Vision port, and says how many more there are.

**R2a-R13 — the menu header keeps `READ-ONLY except 10 DRIVE (powers motors)`.** Station 12 writes
the CAMERA's settings but powers nothing; the file header, `bench_r3a.hpp`, the serial banner and
station 12's own screen say so. Rejected: rewording the SMALL header — it is hardware-seen text.
(Observed, not changed: at the proven SMALL width, 41 characters from x 8 reach past the
`BUILD <stamp>` printed at x 250 on the same row; whether they collide on the panel is for the
SCREEN RULER.)

**Screen budgets used by the new screens:** calibrate's `kSmallCols 50` / `kMediumCols 36` (included
from `calibration/calibrate_panel.hpp`, so the ruler's answer changes one constant for both
programs). Live lines are re-formatted every 200 ms and PRINTED only when their text or colour
changed, padded to the previous length (VEXos draws text pen-on-eraser, so the old characters are
covered without an erase).

### 18:38–18:45 — step 1: the grid geometry (`src/bench_r3a.hpp`) and its host test

- `src/bench_r3a.hpp` gains `namespace shulib::bench::menu_grid` (PROS-free, `<cstdint>` only):
  `Rect`, `Spec{count, cols, x0, y0, w, h, gap}`, `rowsOf`, `colsUsed`, `buttonBox`, `extent`
  (right edge from the RIGHTMOST COLUMN), `hitTest`; `kMenu{12, 3, 6, 38, 153, 46, 4}`; the
  in-button offsets (stripe 6, label inset 8, line tops +3 / +25, dot centre x1−14 / y0+34 r 6,
  line 2 stops 3 px short of the dot) with `line1Px` (145) / `line2Px` (123); the stations' button
  row `kStationRow{3, 3, 6, 186, 153, 48, 4}` and its hit spec `kStationRowHit{3, 3, 6, 178, 153,
  56, 4}` (8 px of pad above only). The header's own doc says why it is PROS-free.
- `test/bench_menu_grid_test.cpp`, NEW, 9 cases: the twelve rectangles (≥ 36 and = 46 tall, inside
  480 × 236, below the header, pairwise disjoint, the exact first/last corners, symmetric
  margins); every pixel of every button hits it and the whole extent has no dead pixel; gap pixels
  go up/left (and the gap above 10 DRIVE is 7's, never DRIVE's); dead header / margins / bottom,
  (0,0), (−1,−1), (−32768,−32768); row-major; **the 2 × 5 menu reproduced pixel for pixel against
  the shipped `hitTest` re-typed as an oracle** over x −2..481, y −2..241; a PARTIAL last row (11
  in 3 columns: the right column alive, the empty slot dead; a single partial row; count 0 and
  cols 0 hit nothing); inside a button (stripe < label, line 1 to x1−1, line 2 clear of the dot, the
  dot inside the edge, 22 px line pitch, line 2's cell above the bottom edge at
  `kMediumHeight`, budgets 11 and 9 at 472/36); the stations' row (inside the panel, the menu's
  columns, the hit area = the drawn buttons + 8 px above and nothing else; menu 11/12 overlap
  RECORD/DONE — the reason for R2a-R10).
- Run: `9 passed`, 267 assertions (the exhaustive pixel loops report only on failure).
- **Mutation campaign (scripted: each mutation applied to `bench_r3a.hpp`, the suite rebuilt, the
  grid cases run, the header restored — `cmp` against the saved copy: identical):**

  | # | mutation | result | the case that went red |
  |---|---|---|---|
  | M1 | extent's right edge from the LAST button's column | **RED** (5 failed) | partial last row |
  | M2 | column-major numbering | **RED** (5 cases) | pixel-exact, gap, row-major, partial, station row |
  | M3 | an empty slot returns its index | **RED** (7238 failed) | partial last row |
  | M4 | buttons 47 tall (last row ends at y 238) | **RED** (18 failed) | twelve buttons … inside the panel |
  | M5 | the extent check deleted | **RED** (4 cases) | dead zones, old-menu oracle, partial, station row |
  | M6 | gap pixels go right/below | **RED** (2 cases) | gap rule, old-menu oracle |
  | M7 | the two defensive clamps deleted | **GREEN — expected** | none: inside the extent the division already lands in range (the last column's/row's gap lies outside the extent), so the clamps are unreachable; the header comment says "defensive only" |
  | M8 | no top pad on the station row's hit spec | **RED** (3 failed) | station row |
  | M9 | label inset 5 (over the stripe) | **RED** | inside a button |
  | M10 | line 2 runs to the dot's edge | **RED** | inside a button |

### 18:46–20:29 — step 2: `src/bench_r3a.cpp`

- **Includes:** `pros/ai_vision.h`, `pros/distance.h`, `pros/error.h`,
  `shulib/calibration/calibrate_panel.hpp` (the budgets), `shulib/hal/distance_conversion.hpp`
  (`kDistanceNoObjectMm`), `shulib/hal/pros/distance.hpp` (`ProsDistance`).
- **The sanitising path (R2a-R3):** `panelSafe()` + `screenText()` (literal `"%s"`, `g_screenPrints`
  counter) + `panelLiteralFits()` (constexpr: printable ASCII, no `%`, within budget);
  `screenEmit()` and `drawHeader()` now print through it with their old cuts (54 SMALL, 30 MEDIUM).
- **Touch (R2a-R4):** `releaseBaseline()` (the first NON-error read's `release_count`);
  `waitForTouch()` and the menu loop skip `E_TOUCH_ERROR` reads for the tap logic (the readout still
  prints them).
- **Menu (R2a-R1/R2):** `kMaxMenu` 12, `g_verdict[12]`; `MenuItem` gains `btn1`/`btn2`; entries 11
  and 12; the old `kBtnW/kBtnH/kBtnX0/kBtnY0/kGap`, `buttonBox()` and the 2-column `hitTest()`
  REMOVED in favour of `menu_grid::` (`kGrid = menu_grid::kMenu`, `static_assert(kGrid.count ==
  kMenuCount)`); `kBtnLine1Cols`/`kBtnLine2Cols` from the pixels at 472/`kMediumCols`;
  `static_assert(menuLiteralsFit())` over every button line (budget) and every label / `doThis` /
  `doneWhen` (ASCII, no `%`); `drawMenu()` draws two MEDIUM lines and the dot from the header's
  offsets.
- **`namespace sensors`** (shared): `LiveLine` (print only on change, pad to the old length),
  `drawRowButton()` (grey while arming), `TapWatch`, `rowTouchWord()`, `discover()` (registry
  0–20 → port = index + 1; `static_assert(kMaxRegistryIndex + 1 <= kMaxPort)`), `portList()`,
  `rawInt()`/`signedOrErr()`/`retWord()`; distance: `DistRead`, `readDistance()` (RAW C API then the
  adapter), `DistTally`, `paintDistance()`, `recordDistance()`; AI Vision: `kFamilies[4]`, the type
  flags, `typeWord()`, `modesWord()`, `className()` (64 zeroed bytes, printable ASCII out),
  `CamSnap`/`snapCamera()`, `signature()` (FNV-1a), `Det`/`decode()` (fields COPIED out by value, the
  18 bytes by `memcpy` of the whole struct — no pointer to a packed member),
  `tagCentreX/Y()`, `detRows()`, `recordDetection()`, `ChangeTimer`, `setUpCamera()`,
  `logClassNames()`, `recordCamera()`.
- **`distanceStation()`** and **`aiVisionStation()`**, `g_tagFamily` (−1 = as found).
- File header, `bench_r3a.hpp`'s `runR3a()` doc and the serial banner say station 12 writes the
  CAMERA's settings, never a motor (R2a-R13).
- **First ARM compile (`make ROBOT=tank`)** found ONE `-Wformat-truncation` in `modesWord()`: the
  bit-name buffer was 64 bytes, so GCC proved `"0x%02lx %s"` could overflow the callers' 64-byte
  buffers. Fixed at the root — the name buffer sized to its content (the longest name list,
  `TAGS+COLORS+OBJECTS+COLOR_MERGE`, is 31), not the caller's buffer grown. Recompiled: only the
  two allowed `-Wunused-function` from `src/main.cpp`.
- Scan of every string literal in the file for a non-ASCII byte outside comments: **0**.
- Two small corrections while re-reading: `discover()`'s capacity is now asserted; the AI Vision
  RECORD summary said `every ~0 ms (min 0, n 1)` before two changes had been seen — it now says
  `changes not timed yet (N change(s) seen; 2 needed)`.

### 20:29 — the tree moved under this chunk (not by this executor)

`git log` now shows two commits on top of `10de29f`, both docs-only and both outside this chunk's
files: `8ac243f` (18:46, the G1 Part 2b brief re-checked) and `f8542ab` (20:26, the COMP brief
revised). They touch `docs/internal/chunks/G1-part2-pros-provider-and-main.md` and
`COMP-competition-program.md` only; this chunk's working changes sit on `f8542ab` without conflict.
Read both briefs' lines that name the tester: G1 Part 2b's table (`:109`, `:113`) still reads true
(`runR3a` is still in `namespace shulib::bench`; `bench_r3a.hpp` now ALSO carries the PROS-free
`menu_grid` — worth knowing for Part 2c's `main.cpp` work and for COMP's `include/shulib/ui/` panel
pieces).

### 20:29–20:37 — step 3: the second gate run, the screen lines tightened, the worksheet

- `python3 tools/src_build_gate.py self-test` → `OK (33 detector cases, each against a real build)`;
  `check` → **PASS**, all six: bench / tank / tank-drive / tank-library / tank-calibrate `4/4 TUs,
  2 allowed (-Wunused-function), 3 vendor-ignored, 0 disallowed`, xdrive `0 allowed`; hash
  asserted; beacons checked — the same profile as the baseline.
- **Screen lines re-cut to the scrolling log's 54 columns.** The stations' exit and no-sensor
  summaries are printed through `screenEmit()` (cut at 54); several were longer and would have lost
  their tails on the panel. Now the `[R3A]` fact lines (per-port tallies, what was LEFT on the
  camera, screen-print counts) are emitted BEFORE the panel is re-enabled — serial + SD only, full
  width — and the panel summary is short lines, each ≤ 54. Recompiled (`make ROBOT=tank`): clean.
- **Every new panel string measured against its budget** (a script renders each format with
  C-printf semantics at typical and worst values — `scratchpad/lengths.py`): 97 strings, **0 over
  budget** (menu line 1 ≤ 11 / line 2 ≤ 9 / station buttons ≤ 11 MEDIUM; live lines ≤ 36 MEDIUM or
  ≤ 50 SMALL; scrolling-log lines ≤ 54). The tightest: `+19 more DISTANCE sensor(s): RECORD logs
  every one` 50/50 (only with 21 sensors), `no DISTANCE sensor in the registry -- nothing to show.`
  54/54, `11 DISTANCE` and `4 BATT/CTRL` and `6 LOOP RATE` 11/11.
- **Worksheet** (`R3a-BENCH-WORKSHEET.md`): Station A — button 12 named as the one that writes the
  camera's settings; the menu drawn as the 3 × 4 grid it now is, the table grown to 12 rows, the
  status dot's place and the white DONE verdict; **Station H — THE DISTANCE SENSORS** and **Station
  I — THE AI VISION CAMERA** after Station G (before, H.1/I.1 screen tables, H.2 the 6/12/24 in and
  open-space table per side, I.2 the piece at ~1 and ~2 ft centred/left/right, I.3 with VEX's AI
  Vision Utility video, I.4 a printed tag with the FAMILY cycle, I.5 whether the camera keeps its
  settings across a power cycle — landmine 1 made into a two-minute test — and what to send back,
  with a RECORD line of each kind); DO NOT's tester bullet names button 12. **Every quoted string
  checked against the source**: a script lists each backticked string in H/I; the literal ones are
  found verbatim in `bench_r3a.cpp`, and each formatted one was compared with its format by eye
  (formats, spacing and the `%5s`/`%7.2f` padding included).
- `docs/roadmap.md`: the R2 row → `[~]` with what exists and what does not; the "Now" paragraph of
  the pointer gains one sentence (R2a landed; the same bench session checks the sensors).
- `docs/internal/build-order.md`: the first `**Next:**` keeps its order (the bench session, R2a's
  bench check folded into it — rebuild Bench Tests, run Stations H and I — then G1 Part 2b, then
  COMP), with the R2a LANDED paragraph after it; R2's entry says LANDED. Edited in place, like the
  commit that briefed it (`10de29f`); no "Previously" paragraph, because the order did not change.
  The Next bold span has no nested bold (the briefing tool's regex stops at the first `**`).

### 20:37 — the prose review (RESUMING rule 4), grep run and every hit read BEFORE this entry

- **The brief's terms** — `grep -rn -i "ten stations\|1-10\b\|1–10\b\|kMaxMenu\|AI-VISION\|no AI
  Vision\|10 DRIVE" docs/ README.md src/` (transcripts excluded): 22 files. Read each: `Tests 1–10`
  in T2/G1/R3d briefs and logs and `seeds 1–10` in A3 are other numbering (not stations);
  `roadmap.md:210`, `build-order.md:661`, `R3b-part0b:22`, R3b-PROGRESS and R3d-PROGRESS are
  HISTORY (station 10 DRIVE on 2026-09-10; `kMaxMenu 9 → 10`) and stay; the worksheet's `10 DRIVE`
  lines are still true (station 10 kept its number); `HANDOFF:251` "no AI Vision adapter" is still
  true (R2b's); `absent_tag_source.md` / `api/README.md` describe `AbsentTagSource`, not the robot;
  `localizer.md` and the master plan use "AI-Vision" as a sensor name; `bench_r3a.cpp`/`.hpp` hits
  are the new code. **No stale "ten stations" anywhere.**
- **The changed files' names and concepts** — `grep -rn -i "bench_r3a\|buttonBox\|hitTest\|2 × 5\|2x5\|2
  x 5\|two columns\|five rows\|two-column\|kBtnH\|kBtnW" docs/ README.md`, then the same for
  `bench_r3a.cpp:[0-9]` across `src/ include/ tools/ test/ .github/`. **Finding 7 — a line
  citation this chunk moves:** the tester's zero-indexed-registry comment moved from lines 112–122
  to **135–145** (6 new includes and the 17-line header note above it). It is cited BY LINE in:
  `docs/hardware-assumptions.md:1309` (HA-138) — **fixed here**, now cited by what it is ("the
  registry comment just above `kMaxRegistryIndex`; lines 135-145 since R2a, 112-122 before it");
  and in three places this chunk may NOT touch — `include/shulib/hal/pros/device_registry.hpp:14`,
  `:66`, `:105`, `test/pros_device_registry_adapter_test.cpp:5`, and the generated
  `docs/api/pros-device_registry.md:90`. **Left for the coordinator, to go in the SAME commit**:
  change those to the symbol (or `:135-145`) and regenerate `docs/api` (`api_doc_tool.py`).
  Rejected: holding the comment at 112–122 by moving the includes below the namespace opening or
  deleting old comments above it — both game a citation instead of fixing it. The historical logs'
  line numbers (R3d-PROGRESS `bench_r3a.cpp:1601/1606/1912/1935-1942`, R3d briefs, R3b-PROGRESS)
  are records of their day and stay. `calibrate_program.cpp:37` ("pen-on-ERASER ... src/bench_r3a.cpp's
  menu") is still true: the menu still prints its labels with the eraser set to the button fill.
- README.md: one hit (`src/ ... the bench tester`, `:269`) — still true.

### 20:40 — verification pass 1 (tree as of 20:38), and one defect found re-reading station 12

- `cmake --build build/test` → the suite builds and runs: **`test cases: 1425 | 1425 passed | 0
  failed | 3 skipped`, `assertions: 2386871 | 2386871 passed`** (+9 cases / +267 assertions — the
  grid test's, exactly); the build's doc-gate step stops at `PROJECT-BRIEFING IS STALE` (the
  `Next:` pointer, the suite line and the `[~]` count moved — regenerated LAST, as the brief says).
  `api_doc_tool.py check-fresh` exit 0; `check-removability` → `no public doc references
  docs/internal/`; `git diff --stat -- include tools Makefile .github src/main.cpp
  src/calibrate_program.cpp src/drive_program.cpp` → **empty**.
- **Defect found re-reading `aiVisionStation()`'s exit:** the serial `LEFT` line and the panel
  summary said "tag + object detection ON, USB overlay ON" UNCONDITIONALLY — false when a setting
  write had returned PROS_ERR at entry (the live panel already said so in red; the exit did not).
  The same for a FAMILY write that failed. Fixed after the running gate finished (never edit `src/`
  while the gate compiles it): the exit now says what was set only when every write succeeded, and
  otherwise names the failure; the gate re-run follows.

### 20:44–20:51 — the exit fix, the Override tags, the last source edits, gate pass 3

- **HEAD moved again** (not this executor): `af36b19` (20:37) — the Override game facts in
  `HANDOFF-remaining-work.md` and the COMP brief. It ANSWERS the handoff's Q4: **the Override field
  HAS AprilTags** (one per goal, ids 0–4 with 1–4 duplicated); **the family is still unknown** ("R2a's
  station finds it"). Aligned: the `kFamilies` comment in `bench_r3a.cpp` no longer says "which one
  a VEX field uses is UNKNOWN (handoff Q4)" but that the field has tags and the manual's text does
  not name their family; worksheet I.0.2 / I.4 now ask for a FIELD tag first (the family that sees
  it IS the field's). Also in the working tree now, and NOT this executor's:
  `docs/internal/chunks/R3d-PROGRESS.md` (+30 lines — the coordinator's second bench session log,
  20:42 on, uploading packages built from `2780462` in a separate worktree). Left untouched.
- **The exit defect fixed** (logged at 20:40): `aiVisionStation()` keeps `setOk` (every entry write
  succeeded) and a new `familyFailed`; the serial `LEFT` line and the panel summary claim "tag +
  object detection ON, USB overlay ON, tag family X" only when `setOk`, else `a SETTING write FAILED
  at entry ... what the camera has is UNKNOWN`; a failed FAMILY write is named in both. New panel
  lines measured: 50, 52, 45 of 54.
- **A doc comment put back where it belongs:** the `waitForTouch()` doc block had ended up above the
  new `releaseBaseline()`; `waitForTouch` now keeps its block (via a declaration, the file's own
  `// fwd` idiom) and `releaseBaseline()` has its own.
- The registry comment is still at **lines 136–145** (135 blank), the range cited in
  `hardware-assumptions.md` (checked with `grep -n` after the last edit).
- `make ROBOT=tank` clean; then **`src_build_gate.py self-test` → OK (33 detector cases);
  `check` → PASS, all six builds, the baseline profile exactly** (bench / tank / tank-drive /
  tank-library / tank-calibrate: `4/4 TUs, 2 allowed (-Wunused-function), 3 vendor-ignored,
  0 disallowed`; xdrive `0 allowed`; hash asserted; beacons checked).

## Definition of Done (brief §4), with evidence per line

- [x] **Six builds link, zero disallowed warnings; beacons unchanged; bench and tank both compile
      `bench_r3a.cpp`.** Gate pass 3 above (20:51, the final source): six PASS, `0 disallowed`,
      every beacon checked, warning profile identical to the 18:34 baseline. Every screen print in
      new code goes through `screenText()` (literal `"%s"`); every displayed string is sanitised
      (`%` → `?`, non-ASCII → `?`/`-`); every string literal in the file is ASCII (scripted scan: 0
      non-ASCII outside comments); every pixel is at y ≤ 234 (menu last row 188–234, station row
      186–234, status line 171 + 13) — `static_assert`s on the station layouts, the host test on the
      grids. `-Wformat-truncation` fired once (`modesWord`) and was fixed at the root (18:46–20:29).
- [x] **The menu: a table of every button's rectangle and label length vs budget** — below; **a host
      check of the generalized `hitTest`** — `test/bench_menu_grid_test.cpp`, 9 cases, green, and a
      10-mutation campaign (9 red, 1 expected green with its reason) at 18:38–18:45.
- [x] **The stations through the PROS shim, if it supports the calls** — it does NOT: no screen,
      no AI Vision, no distance C API (finding 1). Not extended (brief §4). What IS host-tested is the
      pure geometry; the stations' logic is compile-checked (six builds) and review-checked only.
- [x] **Worksheet stations; `build-order.md`; `roadmap.md` R2 `[~]`; this log; no hardware claim.**
      Worksheet Station A + H + I + DO NOT (20:29–20:37, 20:44); build-order's first `Next:` and R2's
      entry; roadmap's R2 row `[~]` and the pointer's "Now" sentence; `hardware-assumptions.md`'s
      HA-138 citation (finding 7). Every document says "never run on a robot".
- [x] **Prose review, grep run before claiming** — 20:37 entry (both greps, every hit read).
- [x] Beyond the list: `PROJECT-BRIEFING.md` regenerated LAST (next entry).

### The menu, button by button (R2a-R1/R2) — rectangles inclusive, budgets at 472 px / 36 MEDIUM

| # | rect (x0,y0)–(x1,y1) | line 1 (≤ 11) | len | line 2 (≤ 9) | len | stripe |
|---|---|---|---|---|---|---|
| 1 | (6,38)–(159,84) | `1 DEVICE` | 8 | `CENSUS` | 6 | — |
| 2 | (163,38)–(316,84) | `2 IMU +` | 7 | `ROTATE` | 6 | amber |
| 3 | (320,38)–(473,84) | `3 MOTOR` | 7 | `WATCH` | 5 | amber |
| 4 | (6,88)–(159,134) | `4 BATT/CTRL` | 11 | — | 0 | amber |
| 5 | (163,88)–(316,134) | `5 SD CARD` | 9 | `PROBE` | 5 | — |
| 6 | (320,88)–(473,134) | `6 LOOP RATE` | 11 | — | 0 | — |
| 7 | (6,138)–(159,184) | `7 RUN ALL` | 9 | — | 0 | amber |
| 8 | (163,138)–(316,184) | `8 MOTORS` | 8 | `STATIC` | 6 | — |
| 9 | (320,138)–(473,184) | `9 SCREEN` | 8 | `RULER` | 5 | amber |
| 10 | (6,188)–(159,234) | `10 DRIVE` | 8 | `POWERS` | 6 | RED + red edge |
| 11 | (163,188)–(316,234) | `11 DISTANCE` | 11 | — | 0 | amber |
| 12 | (320,188)–(473,234) | `12 AI` | 5 | `VISION` | 6 | amber |

Inside each: stripe x0..x0+5; line 1 at (x0+8, y0+3), line 2 at (x0+8, y0+25); status dot centre
(x1−14, y0+34) r 6. Station row: (6,186)–(159,234) FAMILY [12 only], (163,186)–(316,234) RECORD,
(320,186)–(473,234) DONE; hit area the same columns from y 178. Station buttons' lines ≤ 11
(`CIRCLE21H7` 10 is the longest).

## What is NOT done (under-claimed)

- **Nothing here has run on a robot.** No screen string has been seen on a V5; every budget is the
  derived estimate (HA-136); no RECORD line exists; HA-113/114/115 do not move.
- **The shim exercises neither station** (it has none of the calls); the stations' behaviour is
  compile- and review-checked only. The grid is the only host-tested piece.
- **Line citations this chunk moved, in files it may not touch** (finding 7): `include/shulib/hal/
  pros/device_registry.hpp:14,:66,:105`, `test/pros_device_registry_adapter_test.cpp:5`, and the
  generated `docs/api/pros-device_registry.md:90` say `bench_r3a.cpp:112-122`; the comment is now
  at 135–145. For the coordinator, in the same commit.
- Not converted to the sanitising path: the file's other prints of fixed literals and numbers
  (R2a-R3's list) — no defect behind them.
- Not changed: DRIVE's and MOTOR WATCH's exit-on-any-release (R2a-R4 — DRIVE's is the safe
  direction).
- The AI Vision adapter (R2b), any interpretation, colour signatures, the wall reset (L6).

### 20:53 — the briefing regenerated LAST, then the whole verification list on the final tree

- `python3 tools/briefing_status.py generate` → regenerated (Next up = the new first `Next:`; open
  chunks now list R2a; suite 1,425 / 2,386,871; 10 `[~]` roadmap items).
- `cmake --build build/test` → built, doc gates green; `./build/test/shulib_tests` → **`test
  cases: 1425 | 1425 passed | 0 failed | 3 skipped`, `assertions: 2386871 | 2386871 passed`**.
- `src_build_gate.py self-test` OK / `check` PASS — gate pass 3 (20:51) is on this exact source.
- `api_doc_tool.py check-fresh` exit 0; `briefing_status.py check` → `generated block current`;
  `api_doc_tool.py check-removability` → `no public doc references docs/internal/`.
- `git diff --stat -- include tools Makefile .github src/main.cpp src/calibrate_program.cpp
  src/drive_program.cpp` → **empty**.
- Nothing committed, staged, pushed or uploaded; no `pros` command run. Files of this chunk:
  `src/bench_r3a.cpp`, `src/bench_r3a.hpp`, `test/bench_menu_grid_test.cpp` (new),
  `docs/internal/chunks/R3a-BENCH-WORKSHEET.md`, `docs/internal/build-order.md`, `docs/roadmap.md`,
  `docs/hardware-assumptions.md`, `docs/internal/PROJECT-BRIEFING.md` (generated), this log (new).
  `docs/internal/chunks/R3d-PROGRESS.md` is modified in the tree by the coordinator's bench session,
  not by this chunk. No `R2a-COMPLETED.md` written (the coordinator's, after verification).

### 23:38–23:50 — the COORDINATOR's verification (not the executor's), before the commit

Done in the main tree after the executor reported, with HEAD at `af3c43f` (the coordinator's
bench-session log commit landed first, docs only, and touched none of this chunk's files except
two unrelated sentences of `build-order.md` / `roadmap.md`, staged separately from this chunk's).

- **The whole gate set, re-run** (scratchpad `verify.sh`, 23:38–23:41): host build with the doc
  gates inside it, then the suite — `1425 | 1425 passed | 0 failed | 3 skipped`,
  `2386871 | 2386871 passed` (the executor's numbers exactly); `src_build_gate.py self-test` OK (33)
  and `check` PASS on all six builds, `0 disallowed` each, hashes asserted, beacons checked; the
  ARM header gate over 184 headers, exit 0; both CI greps empty; `check-fresh` 0; briefing
  `generated block current`; the removability grep empty.
- **Four hand mutations of `src/bench_r3a.hpp`**, each with `touch test/*.cpp` and the `/fast`
  target (it does not track header dependencies), the `Linking CXX executable shulib_tests` line
  required, the grid cases run, then the original restored with `cp` and checked with `cmp`:
  M1 a gap pixel goes to the RIGHT (`+ g.gap` in the column division) → 3 of 8 `menu grid` cases
  red, 381 assertions; M2 the extent's right edge from the LAST button (the 2-column idiom) → 1 red
  (the partial-row case), 5 assertions; M3 the hit test accepts one row below the extent → 2 red,
  469 assertions; M4 the station row's hit area 48 px instead of 56 (no longer covers the drawn
  buttons' bottom 8 rows) → the station-row case red, 3699 assertions. Restored and relinked: the
  full suite green again at the same counts.
- **Read, against the vendored headers:** `aivision_enable_detection_types` ADDS (ai_vision.h:
  "Enabling any given type of object will not disable the detection of other objects") — so
  "colours left as found" is true; `aivision_set_tag_family_override` REPLACES while
  `aivision_set_tag_family` "will allow multiple apriltags" — the executor's deviation from the
  brief's call is right; `aivision_get_class_name(port, -1, …)` returns the count; the detected
  type flags are 1/2/4/8 as the record line says; an `E_TOUCH_ERROR` read carries −1 in every field
  (screen.h:729); PROS's C `distance_get_confidence` doc says 10 is returned at ≤ 200 mm and the C++
  doc does not (the worksheet's H.2.1 note). `panelSafe`'s UTF-8 look-ahead never reads past the
  terminator; `LiveLine` pads a shorter redraw over the old text; every station button arms after
  0.5 s from a baseline taken after setup; a white filled dot is `Sev::Info`, which predates R2a
  (the worksheet's "white DONE" describes existing behaviour).
- **The four stale line citations (finding 7) fixed by the coordinator in this commit — a
  deviation from the brief's OUT ("any change to the library (`include/`)"), COMMENT-ONLY:**
  `include/shulib/hal/pros/device_registry.hpp` (three sites) and
  `test/pros_device_registry_adapter_test.cpp` (one) now name the comment by its anchor — "the
  registry comment just above `kMaxRegistryIndex` in `src/bench_r3a.cpp`" — instead of a line range,
  so the next edit above it cannot stale them again; `docs/api/pros-device_registry.md` regenerated
  (`api_doc_tool.py generate`; only that page changed; `check-fresh` 0). No code line changed.
  `hardware-assumptions.md` keeps the executor's anchor-plus-history wording. The older briefs and
  logs that cite `bench_r3a.cpp` line ranges are records of their own tree and are left as written.
- **Prose review, re-run by the coordinator** (grep run first, hits read): "ten stations", "1-10",
  "1–10", "kMaxMenu", "no AI Vision", "two columns", "2 x 5"/"2 × 5" across `docs/*.md`,
  `docs/guide`, `docs/cookbook`, `README.md`, RESUMING, HANDOFF, ORIENTATION and `src/` — two hits
  outside this file: `src/drive_program.cpp:386` (the DRIVE panel's two columns, unrelated) and
  HANDOFF :251 "no AI Vision adapter" (still true: R2b). "112-122": only this log, HA-138's history
  clause and two dated R3d-PROGRESS entries.
- **Pointers moved in this commit:** `build-order.md`'s first `Next:` now says the bench session
  CONTINUES — Calibrate has run twice (run 1 STOPPED, run 2 unrecorded), what is left is reading
  run 2's screen, a recorded reproduction run, Teleop from the record, and R2a's Stations H and I;
  `roadmap.md`'s "Now" sentence likewise; `R2a-COMPLETED.md` written; the briefing regenerated last.
