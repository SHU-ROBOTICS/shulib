# RMP1-H — the field's solid structures: drawn, and every step checked against them

> Brief by the coordinator, 2026-11-19 ~23:20. **The team lead, 2026-11-19:** "some obstacles on the field are solid —
> they are meant to place cones on them — but we can't roll through them and the route builder doesn't say if you are
> too close to hitting them, so if you try a manoeuvre like going past them, clipping them gets in the way." One run,
> **RMP1-H-A**. The route builder stays open to the 21st. NO `include/` change (the library is frozen). Base: main
> `518717a`.

## 1. What exists

The route builder's field numbers come from the C++ (`test/route_maker_describe.hpp` → `robots.json`'s `field`): the
walls' inner faces (`localization/override_walls.hpp`), the four Toggles and the centre goal's POSITION
(`vision/override_elements.hpp`). The page checks the robot's square against the walls (a step "through a wall" is red;
PREVIEW marks THROUGH). The goals and any other solid floor structure have NO footprint anywhere in the code, so nothing
checks them.

## 2. Decisions

- **D1 — The structures' geometry, from the game manual, cited.** Find the VEX V5RC Override game manual (v2.0 of
  2026-09-03, or v2.1 if published; VEX's site) and read Appendix A — pA10 "Field Element Locations" and each element's
  drawing — for EVERY solid element on the floor that a robot cannot drive through (the goals the cones are placed on,
  and anything else solid; NOT the game pieces, which move). For each: its centre in F1's field frame (the frame
  `override_walls.hpp` uses — check against the walls and the centre goal at (0, 0)), its footprint (a circle radius or
  a polygon), the manual's page and figure. Put them in a C++ data header OUTSIDE `include/` (e.g.
  `test/override_field_obstacles.hpp`, beside the describe tool) with one comment per element citing its source, and emit
  them through `route_maker_describe.hpp` into `robots.json`'s `field.obstacles` — never typed in JS. Cross-check every
  footprint against the field picture (`field.webp` + `field.json`'s pixel scale): each drawn outline must sit on the
  element in the picture (a test). If the manual cannot be fetched, say so and STOP before guessing numbers.
- **D2 — Drawn on the field:** each obstacle a solid, quiet outline labelled on hover ("goal — solid"), under the
  route, never clickable as a step.
- **D3 — Every step checked, like the walls:** the robot's square swept along the page's planned path (the same sweep
  the wall check uses) against every footprint. **Red** when it overlaps (the step and the moment, which structure, how
  far into it, and a plain fix: "move the point", "go around: add a point at (x, y)"). **Amber** "within N in of the
  goal" when it passes closer than a clearance (default 1 in; the robot square's own corners). PREVIEW marks a run that
  enters a structure the same way it marks a wall (THROUGH, faded after).
- **D4 — Touching on purpose:** a step whose END is at a structure (placing a cone: the robot's front against the goal)
  is allowed to TOUCH it — only real overlap past a small allowance (0.5 in) is red; and a step can be marked "touches
  this on purpose" (one checkbox in the inspector) to silence the amber warning for that structure.
- **D5 — Nothing else changes:** the exported C++ byte-for-byte (the RMP1-C goldens), the project file (D4's flag is
  additive, page-only), PREVIEW's engine.

## 3. Tests (adversarial — each seen RED against a mutant)

Node: each obstacle's footprint from robots.json; a path through a goal → red with the right structure and depth; a
path grazing it within 1 in → amber; a step ending against a goal's edge (touching) → no red; the on-purpose flag
silences amber only; the sweep at curve steps and turns-in-place (a turning square's corners). C++ (suite): the
describe output carries the obstacles with their source; the footprint-on-picture check. Page (Firefox + Chromium): the
outlines drawn, the red/amber issue with its fix, PREVIEW's THROUGH on a structure; screenshots light/dark.

## 4. Docs

Guide 22: one short section "Solid structures on the field" with a screenshot; guide 02 (the field) one line; the
changelog; RMP1-PROGRESS (`## RMP1-H`). Every number's source named in the data header.

## 5. Landmines

- On the same page, in flight: RMP1-F (`export_drawer.mjs`, `host/files.mjs`, `tools/upload_auton.py`), RMP1-G
  (`field_view.mjs`, `step_list.mjs`, `inspector.mjs`, `project.mjs` view state, keys in `app.mjs`). Put your drawing in
  a NEW module (`src/ui/field_obstacles.mjs`) with ONE hook line in `field_view.mjs`; your checks in `checks.mjs` (or a
  new `src/model/obstacles.mjs`); CSS in one delimited block of `style.css` (`/* ── Field obstacles (RMP1-H) ── */`).
- The game pieces (Pins, Cups, the Toggles' arms) are NOT obstacles.
- Never run pros or touch /dev/ttyACM*.
