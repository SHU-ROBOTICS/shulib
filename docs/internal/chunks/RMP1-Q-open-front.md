# RMP1-Q — the open front: a goal centred on the claw sits INSIDE the robot, and the route builder agrees

> Brief by the coordinator, 2026-11-20 ~21:25 EST. **The teammate (Joseph), 2026-11-20**, looking at his working test
> route in the route builder: "you see the problem, it is like hella overlap but its actually working". Then, with the
> robot's CAD ("Tears Final.step"), he confirmed the yellow circle in the robot's front is the pin holder (a goal base):
> "its not a set scoring position because its just at the bottom but of course you stack the pins". Route builder
> and its host tool only: **no robot code, no library**. The competition is SATURDAY Nov 21: fast, nothing that
> guards correctness skipped. ONE idea per screen.

## Today

The route builder treats robot one as a solid 15 in square. Its claw grabs a pin 1.232 in inside the front edge (RMP1-O,
`kClawGrabInsetIn`, the cross). To stack a pin on a goal, the robot drives until the claw (the cross) is over the
goal's centre. The goal's base then sits INSIDE the square, about 4 in deep. The real robot has an open bay there and
scores. The page calls it RCO1 (red: "into a structure"), so Export and PREVIEW refuse the route that works.

## What the CAD says (measured by the coordinator, 2026-11-20 21:20; scripts and numbers in `scratch/cad/`)

Measured on the team's CAD, which has a goal (`276-9250-004`) placed in the bay. It is centred on the robot's centre
line, 33.8 mm (1.33 in) inside the front edge (the sizing plate `Part 009`: x −181..200, y −208..173, floor z −34).
The nearest ROBOT surface to that goal's vertical axis, by height above the floor:

| Height | Nearest robot surface | Parts |
|---|---|---|
| 0–10 mm (a goal's lip, full width) | **81.9 mm = 3.22 in** | Part 021 / Part 020 (the frame's gussets, diagonally behind); Part 017 straight behind at 88.7 mm |
| 10–64 mm (a black goal's full-width body; a red/blue goal narrows above its lip) | **71.2 mm = 2.80 in** (not counting the claw) | the two 1x2x1x27 C-channels, diagonally behind |
| claw | 29–33 mm | the claw and its mount: they ride the cascade, and the route lifts it before driving in (the CAD's cascade pose is NOT the robot's: the teammate, "the cascade goes higher than what you see in the cad") |

Across the goal's centre at floor level the bay is open from x −98.9 to +100.4 mm. The 3.22 in circle is the
conservative shape: the lip fits it exactly, its corners touching the gussets.

## Decisions

- **D1 — the facts.** Robot one's facts file, `Stage::Recorded` (gates no build):
  `kFrontBayLipRadiusIn` = 3.22 (the robot's floor-level clearance around a goal centred on the claw: Part 020/021,
  CAD "Tears Final.step", measured 2026-11-20) and `kFrontBayBodyRadiusIn` = 2.80 (the same, from 10 to 64 mm above the
  floor: the C-channels). The bay's centre is the claw's spot (`kClawGrabInsetIn`): one fact, never a second copy.
  Robot two = robot one's (same build), as its other copied facts. Counts and summaries updated as RMP1-O did.
- **D2 — robots.json**: `frontBay: {centreForwardIn, lipRadiusIn, bodyRadiusIn}` beside `clawSpot`, absent for a
  robot without one (x, y, z).
- **D3 — the rule.** Against a GOAL only (walls and Loaders stay solid), the robot's footprint is the square MINUS the
  bay's disc. The disc is centred at the claw's spot. Its radius is the lip radius for a goal whose base narrows above
  its lip (red and blue: `overhangIn > 0` today), and the BODY radius for a goal that does not (black and centre). A
  goal whose footprint lies inside the disc is in the bay: no RCO1, no amber. A goal that reaches past the disc
  overlaps the rest of the square, and that overlap is judged exactly as today (TOUCH_IN, on purpose, THROUGH_IN). The
  sweep stays the plan's own (`stepPoses`). The square minus a disc is not convex: build it from convex pieces, or test
  each goal point against "inside the square AND outside the disc". Prove the exact boundary with tests either way.
- **D4 — black goals: ANSWERED 21:52** (the teammate: "the preload goes into the blue goal and we focus on that for
  now"): the route scores a blue (colour) goal, so D3 stands as written: the lip radius for red and blue goals, the
  body radius (conservative) for black and centre goals until the team takes one into the bay.
- **D5 — PREVIEW agrees.** Wherever PREVIEW decides THROUGH for a goal (obstacles.mjs `runThrough`, and the C++ preview
  model if it judges goals, `route_preview_model.hpp kThroughAllowanceIn`: read both), the bay applies the same way.
  The page's check and PREVIEW must never disagree on the same pose.
- **D6 — draw it.** The robot's drawing shows the bay. A notch, or the disc's arc, at the front, in the same strokes
  as the square, so that what is drawn is what is checked. Keep the RMP1-O cross where it is.
- **D7 — nothing else changes.** A robot without `frontBay` is checked byte-for-byte as today. Every existing route,
  golden and RCO verdict stays the same except those where a goal sits in robot one's or two's bay.

## Acceptance

- node tests, the boundary both sides: a red goal centred on the claw spot is OK (end of a step, and the straight
  approach from 12 in out); 0.6 in behind the claw spot is RED (it hits the back of the bay); 0.6 in to the side,
  red or amber as the rule says, with the number pinned. A black goal centred: allowed only within the body radius.
  A Loader or a wall at the bay: still RED. Robot x (no bay): unchanged. The mirrored route on robot two (RCO3)
  behaves the same.
- The coordinator's test project (`scratch/route_maker.TEST-v10-2026-11-20.json`, White Bot's working route): RCO1
  gone; every other check unchanged. State any it still flags, and why.
- Mutants (each RED): the disc ignored; the disc centred at the robot's centre; the lip radius used for a black goal;
  the bay applied to a Loader; PREVIEW's THROUGH not using the bay.
- Page tests (Firefox AND Chromium): the bay drawn, and the test project exporting with no RCO1. Screenshots light/dark
  at 1440 and 1280, each read.
- Docs: guide 22 (one sentence + one picture: "a goal centred on your claw sits in the robot's open front"), the route
  builder README, changelog (2.11, what people see), RMP1-PROGRESS.

## Beside you

CAM2 (the camera pickup step: library, facts, route builder "Finish with the camera") is building in its own worktree:
keep the facts-file change one contiguous block next to `kClawGrabInsetIn`, and the robots.json key separate. CI1
(Windows self-tests) touches tools/upload_auton.py and tools/route_builder.py only. RMP1-L (robot names) waits.
