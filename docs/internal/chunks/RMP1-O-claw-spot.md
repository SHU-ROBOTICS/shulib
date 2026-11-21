# RMP1-O — the claw's spot: place a grab or a drop by where the CLAW is, not by the robot's centre

> Brief by the coordinator, 2026-11-20 ~17:45 EST. **The team lead, 2026-11-20**, after his route came up ~7 in short
> of two pins: "our bot is 15x15 and the claw isnt beyond that its indented into our bot so ... it looks like its
> overlapping"; "the route builder should take that into consideration the center of the pin is about 1.86 inches in
> the bot in the bots center front", then, measured: **"1.232 inches actually"**. Route builder and its host tool only: **no robot code, no library**. The
> competition is SATURDAY Nov 21: fast, nothing that guards correctness skipped. ONE idea per screen.

> **Errata (the builder, at RMP1-O's review, 2026-11-20 ~18:50; the text below is the coordinator's, unchanged):**
> written ~17:27 (the file's own time), not ~17:45. With the measured 1.232 in, a pin at the square's front edge leaves
> the claw 1.232 in short, not ~2. D4's formula is a FORWARD drive's: backing up (reverseTo, a reverse curve) the claw
> trails, so the centre is PAST the point, C = Q + forward·(Q − P)/|Q − P| (the robot backs over the pin first: a drop,
> not a grab). The refusal is |Q − P| < forward + 1 in: under 1 in of travel, rounding the centre turns the leg.

## Today

A route's point is the robot's CENTRE; the page draws a 15 in square around it. A pin the claw grabs sits 1.232 in
INSIDE the front edge, centred — 6.268 in ahead of the centre. A student places the square's front edge at the pin
(no overlap "looks right") and the claw stops ~2 in short; nothing on the page shows where the claw is.

## Decisions

- **D1 — the fact.** `kClawGrabInsetIn` (robot one's facts, `Stage::Recorded`: gates no build) typed **1.232**, source
  "measured by the team lead, 2026-11-20: the pin's centre 1.232 in inside the front edge, centred"; robot two's =
  robot one's (as its other copied facts). Counts and summaries updated as `kCascadeHoldKp` was (11992a7).
- **D2 — robots.json** (`test/route_maker_main.cpp`, the goldens): each robot carries `clawSpot: {forwardIn, sideIn}`
  — forward = footprint / 2 − inset (6.268 for a 15 in robot), side 0 — absent for a robot with no claw.
- **D3 — draw it.** On every robot the field draws (the start, each step's end, the dragged ghost, PREVIEW's robot)
  a small claw marker at that spot; bigger on the step selected when it is a claw step or the drive just before
  one. The inspector of a drive step says, in one line: "The claw reaches (x, y)."
- **D4 — place by the claw.** A drive step's point can be dragged BY THE CLAW (a toggle in the inspector, "Place by
  the claw", and a held key while dragging — pick one that no current shortcut uses, said in the help): the marker
  follows the pointer and the step's point is computed back along the approach — for a drive from P whose claw must
  land on Q, the centre is C = Q − forward·(Q − P)/|Q − P| (refused in words when |Q − P| < forward). Typing the claw's
  (x, y) is the same. The exported C++ is still the centre's numbers (byte-for-byte for routes that do not use it).
- **D5 — no overlap warning for pins:** pins are not solid (only goals and Loaders are); no change to obstacles.

## Beside you

RMP1-M (in fix/verify) adds each mechanism's WIRING to robots.json in the same emitter — keep your change to
`route_maker_main.cpp` one contiguous block and the JSON key separate (`clawSpot`); RMP1-L (robot names through one
source) and RMP1-N (a camera card in Watch the robot) are in flight. Name a robot only through one local helper.

## Acceptance

- node tests: D4's math (straight approach, a curve's last leg, Q too close to P refused, both robots), the marker's
  position for every heading; robots.json pinned for both robots and a clawless robot; exported C++ byte-for-byte.
- Page tests (Firefox AND Chromium): the marker at the start and a step's end; drag by the claw lands the claw on the
  pointer (± 0.1 in); the inspector line. Mutants (each RED): forward = footprint/2 (inset ignored); side ≠ 0; the
  back-computation along the wrong direction; the marker drawn at the centre.
- Screenshots light/dark at 1440 and 1280; read each (one idea per screen).
- Docs: guide 22 (one sentence + one picture: "the cross is your claw — put it on the pin"), the route builder README,
  changelog (2.11, what people see), RMP1-PROGRESS.
