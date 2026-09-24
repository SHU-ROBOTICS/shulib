# R2a — Bench Tests learns to SHOW the distance sensors and the AI Vision camera

> **Execution brief.** Written 2026-09-24 at tree `6456beb`, from the team lead's question — *"how do we
> know the distance sensors and the ai visions sensors are working? is there logging or can I see the
> camera directly like what it sees?"* — and his "yeah add that". **R2 (the AI Vision adapter) is split:**
> **R2a** (this) adds two read-only stations to the bench tester so the team can SEE what the sensors
> report on robot two, and the adapter is written afterwards from those readings (R2b) instead of from
> beliefs. The distance sensors ride along: the wall reset (handoff L6) needs the same evidence.
> **One executor run. Executor model: `opus`. Do not commit.** Live log: `R2a-PROGRESS.md`, created FIRST.

---

## 0. Why, and what exists

Robot two now carries an **AI Vision sensor on port 3** (front, level, roughly centred) and **two
distance sensors, port 4 (left) and port 5 (right)** (team lead, 2026-09-24; the calibrate program's
own census confirmed VISION 3, DISTANCE 4, DISTANCE 5). Today nothing in the tree shows what they read:
Bench Tests (`src/bench_r3a.cpp`) has ten stations — census, IMU, motor watch, battery/controller, SD
probe, loop rate, run all, motors, screen ruler, drive — and none for these sensors; the library's
`hal/pros/distance.hpp` adapter is host-tested only; there is no AI Vision adapter (R2).

| Piece | Where | What matters |
|---|---|---|
| The tester | `src/bench_r3a.cpp` | `MenuItem{label, run, handsOn, doThis, doneWhen, powersMotors}` table `kMenu` (~:1807), `kMaxMenu = 10` and `g_verdict[]` (~:220), the grid `buttonBox`/`hitTest` (~:1861, :1936 — 2 columns × 5 rows, `kBtnH 36`, the last row already ends at y = 234), `emit`/`emitf` (serial + the SD log), `drawHeader`, the census's `E_DEVICE_*` kind words (~:381, including `E_DEVICE_AIVISION` → "AI-VISION") |
| The distance adapter | `include/shulib/hal/pros/distance.hpp` (`ProsDistance(port)`: `distance()` in canonical inches, `confidence()` in [0, 1], sentinel-screened), beliefs HA-113 (mm), HA-114 (9999 = no object, in-band), HA-115 (confidence 0–63, only above 200 mm) | the station shows the RAW PROS reads next to the adapter's canonical ones, so HA-113…115 can be settled from one screen |
| The AI Vision API | vendored PROS 4.2.2 `include/pros/ai_vision.h` / `.hpp` | `aivision_get_object_count`, `aivision_get_object` → `aivision_object_s_t{id, type, union{color{xoffset,yoffset,width,height,angle}, tag{x0..y3 corners}, element{xoffset,yoffset,width,height,...}}}` (type flags COLOR/CODE/OBJECT/TAG), `aivision_enable_detection_types` (MODE_TAGS / COLORS / OBJECTS / ALL), `aivision_set_tag_family` (CIRCLE_21H7, 16H5, 25H9, 61H11), `aivision_get_class_name`, `aivision_get_temperature`, `aivision_set_usb_bounding_box_overlay`, `AIVISION_MAX_OBJECT_COUNT 24` |
| Screen facts | the R3d Part 2F investigations; `src/calibrate_program.cpp`'s `printText` | three text sizes; budgets derived (LARGE 18 / MEDIUM 36 / SMALL 50); PROS re-reads screen text as a FORMAT (no `%` on screen) and LEAKS each print (redraw on change); usable height 240 |

**Seeing the camera itself:** the AI Vision sensor's own USB-C port streams video to VEX's AI Vision
Utility on a computer. This chunk turns the sensor's bounding-box overlay ON for that stream
(`aivision_set_usb_bounding_box_overlay`), so what the station lists and what the video outlines are
the same detections. The utility is VEX's tool, not this library's; the worksheet says how it is used
alongside the station.

---

## 1. Scope — IN

1. **Station 11 — DISTANCE (read-only).** Every port the brain's REGISTRY reports as DISTANCE is shown
   (discovered, never typed — the runtime-discovery rule): per sensor, live at ≤ 5 Hz, the adapter's
   `distance()` in inches and `confidence()` next to the RAW PROS reads (`distance_get` mm, confidence
   0–63, object size, object velocity) and the sentinel count. A **RECORD** button writes one line per
   sensor to serial and the SD log (`[R3A] dist port 4 raw 305 mm conf 61 size ... -> 12.01 in ...`), so
   the operator can hold the robot at taped distances from a wall and capture each. Nothing is powered.
2. **Station 12 — AI VISION (read-only apart from the sensor's own detection settings).** For every port
   the registry reports as AI-VISION: enable detection of TAGS and OBJECTS (the built-in game-element
   model; COLORS need descriptors this chunk does not define — say so on screen), turn the USB
   bounding-box overlay ON, and show live, at ≤ 5 Hz: the object count; up to the screen's room of
   objects, each with its type word (COLOR/CODE/OBJECT/TAG), id or class name (`aivision_get_class_name`
   for OBJECT), and its box (x, y, w, h) or a tag's centre and corners; the sensor temperature; the read
   period (how often the count changes — a first latency hint). A **TAG FAMILY** button cycles the four
   families (the one a field uses is UNKNOWN — handoff Q4). A **RECORD** button writes every current
   detection to serial and the SD log. The station prints what the sensor says and interprets NOTHING
   (no bearing, no distance — that is R2b's, from this evidence).
3. **The menu grows to twelve without shrinking the targets.** The 2 × 5 grid cannot take a sixth row
   (it would end at y ≈ 274). Change to **3 columns × 4 rows** (or another layout that keeps every
   button at least as tall as today's 36 px and every label inside its budget) — generalize
   `buttonBox`/`hitTest` for N columns, keeping their no-dead-gap rule and the header/margin dead
   zones; `kMaxMenu` and `g_verdict[]` to 12. **Stations 1–10 keep their numbers** (the worksheet and
   every log refer to them); labels may shorten to fit, logged.
4. **Station H/I in the worksheet** (`docs/internal/chunks/R3a-BENCH-WORKSHEET.md`, after Station G):
   *Distance* — hold the robot square to a wall at 6, 12 and 24 in (tape), RECORD each, and read what
   "no object" looks like (point it at open space); *AI Vision* — with and without the USB video open:
   a game piece at about 1 and 2 ft, moved left then right (does x rise to the right?), a printed
   AprilTag if the team has one (cycle the families until it is seen), RECORD each; what to send back.
   Every quoted screen string copied from the code.

## 2. OUT
The AI Vision ADAPTER and any interpretation (bearing, range, frame) — R2b, after the evidence; the
wall reset (L6); colour signatures; any change to the library (`include/`); any powered action. The
calibrate, drive and teleop programs.

## 3. Rulings
**S1 — Raw first, interpretation later.** Every number on these screens is either the RAW PROS value
or the existing adapter's output labelled as such; no new belief is minted here, so no HA number moves
except where a RECORD line settles one (HA-113…115: the bench session that reads them settles them).
**S2 — Discovered ports only.** The stations never read the chassis table for sensor ports.
**S3 — Changing the sensor's detection settings is allowed; the station says so on screen** ("this
station turned on TAG + OBJECT detection and the USB overlay on port 3").

## 4. Checks and DoD
- [ ] Six builds link, zero disallowed warnings (`-Wformat-nonliteral`, `-Wformat-truncation`: only a
      literal `"%s"` reaches a screen print; no `%` in any displayed text; ASCII only; y < 236); beacons
      unchanged; `src/bench_r3a.cpp` is compiled by the bench and tank builds — both must pass.
- [ ] The menu: a TABLE in the log of every button's rectangle and label length vs budget; a host
      check of the generalized `hitTest` if it can be pulled into a testable helper without touching
      `include/` (else a written walk of the grid's edges, logged).
- [ ] The two stations exercised through the PROS shim if the shim already supports these calls (say
      which it does not; do NOT extend the shim beyond what these stations need).
- [ ] The worksheet stations; `build-order.md` (R2 → R2a/R2b, R2a's status, the first `Next:` only if
      it changes); `roadmap.md` R2 row (`[~]`: the stations exist, the adapter does not);
      `R2a-PROGRESS.md`; no hardware claim — nothing here has run.
- [ ] Prose review (grep docs/, README.md and src/ comments for "ten stations", "1-10", "kMaxMenu",
      "AI-VISION", "no AI Vision"), run before claiming it.

## 5. Landmines
1. The sensor's settings persist on the sensor between programs? UNKNOWN — log what the station set,
   so a later program is never surprised.
2. `aivision_get_object` returns a packed struct; copy fields out before formatting (no unaligned
   references).
3. The registry kind for the AI Vision sensor is `E_DEVICE_AIVISION`; the library's `DeviceKind::Vision`
   covers both vision sensors — do not conflate them in the station.
4. Redraw only on change (the print leak); RECORD must not flood the SD log (one line per sensor per
   press).
