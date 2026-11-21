# RMP1-N — the camera view: see what White Bot's AI Vision sensor sees, live, in the route builder

> Brief by the coordinator, 2026-11-20 ~16:00 EST. **The team lead, 2026-11-20:** "i wan to see it through the route
> builder btw if possible so i can use i to show you what its doing because its on the front so you can see"; again
> at ~16:00: "how is the ai vision sensor going can we see what it sees in the route builder". The camera is on
> **port 7** of White Bot. Route builder only: **no robot code, no library**. Released `shulib Drive` (slot 1 on
> White Bot, uploaded 13:50) ALREADY prints what the camera sees (`src/programs/drive_program.cpp`, `CameraTrace`):
> this chunk only READS and DRAWS those lines. ONE idea per screen.

## The lines (read the code for the exact format; never guess one)

`CameraTrace` (drive_program.cpp) logs through the telemetry sink with tag `CAM` (so on serial: the same prefix as
every other tag's line — read run_log.mjs / the 11-20 log for the prefix):
- at boot: `AI Vision on port <p>: tags, objects and colours on (<ret>); a line every 100 ms: ...` or `no AI Vision
  sensor on any port: nothing to trace`; then `class <id>: '<name>'` for each class name the model reports (0..7);
- every 100 ms while it sees something (every 1 s while it sees nothing): `n <count>` then, per detection, one of
  ` | tag <id> c(<cx>,<cy>) <w>x<h>` (an AprilTag: its centre and its corners' bounding box),
  ` | obj <class> (<x>,<y>) <w>x<h> <score>%` (an object the AI model found), ` | col <id> (<x>,<y>) <w>x<h>` (a colour
  blob or code), ` | type <t> id <id>` (anything else). Up to 8 per line. Whether `(x,y)` of obj/col is the box's
  corner or centre, and the image's size (320 x 240?), come from `include/pros/ai_vision.h` and the VEX/PROS docs —
  state where you found it; if no source says, say so in the code and in RMP1-PROGRESS and draw with the assumption
  named.

## Decisions

- **D1 — a Camera view inside Watch the robot.** When the watched robot prints `[CAM]` lines, a **Camera** card
  appears in the run view (a tab or a toggle beside the run's summary; the run marks stay where they are). It shows
  the camera's frame as a dark rectangle at the image's aspect, the latest detections drawn as boxes (a tag: its id
  big inside its box; an object: its CLASS NAME from the boot's class lines, else `class <id>`, and its score; a
  colour: `colour <id>`), and ONE plain line above: "Sees: AprilTag 3, Pin (87 %)" or "Sees nothing" or "No camera
  on this robot" (the boot line). It updates as lines arrive (≤ 10 a second); a line older than 1.5 s greys the boxes
  ("last seen 2 s ago").
- **D2 — no camera, no card noise:** a robot printing no `[CAM]` lines shows no Camera card; "no AI Vision sensor"
  shows the card with that one sentence.
- **D3 — the log keeps it:** the helper's saved watch log (build/robot_runs/) already has every line; nothing new to
  save. "Show the robot's log" unchanged.
- **D4 — not today:** steering, AprilTag positions on the field, intake checks, any robot-code change.

## Beside you

RMP1-L (White Bot / Black Bot names, through ONE name source) and RMP1-M (a Mechanisms tab, Test per mechanism via
RMP1-K's check, "find the bottom") are in flight on the same page. Keep your edits to run_view.mjs to one contiguous
hook, your parser in its own module (`src/model/camera_log.mjs`), your drawing in its own (`src/ui/camera_view.mjs`),
your CSS in one delimited block at the end of style.css. Name a robot only through one local helper.

## Acceptance

- Parser: node tests on lines built from the C++ format strings (copy them into the test with a comment naming
  their source line; a test reads drive_program.cpp and fails if those format strings changed); garbled, cut,
  interleaved and non-CAM lines; 0, 1 and 8 detections; class names; the boot's no-sensor line.
- Page: a Playwright test with the helper's fake terminal replaying a fixture with boot lines, detections, a quiet
  second and a stale gap; the card's sentence and boxes (count, labels, positions scaled) pinned; Firefox AND
  Chromium. Mutants (each RED): x/y swapped; class name not used; stale never greyed; the card shown for a robot with
  no camera lines.
- Screenshots light/dark at 1440 and 1280 of the card seeing a tag and a Pin; read each for one idea per screen.
- Docs: guide 22's Watch section, one sentence and one picture; the route builder README; RMP1-PROGRESS; changelog
  (2.11, what people see).
