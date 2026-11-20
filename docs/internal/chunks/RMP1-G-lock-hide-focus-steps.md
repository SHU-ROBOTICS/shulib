# RMP1-G — steps you can lock, hide and focus on (overlapping steps stop getting in the way)

> Brief by the coordinator, 2026-11-19 ~21:35. **The team lead, 2026-11-19:** "it is hard to work on individual steps
> because so many things overlap each other so i was asked to like lock steps or make them invisible or add layers or
> something so that it can be easier to interact with steps". One run, **RMP1-G-A**. The route builder stays open to the
> 21st. NO library (`include/`) change. Base: integrate `200f491` (main + RMP1-C named routes + S13UX + the tutorial).

## 1. Why

A real skills route has 15–25 steps around the same few spots (a claw stop, a back-up, a turn): their points, paths
and handles sit on top of each other, so clicking or dragging one grabs another. The step list on the left and the field
fight for attention (the team lead: dense screens are "overstimulating").

## 2. Decisions

- **D1 — Hide a step** (an eye icon on its row in the step list, and `H` with the step selected): a hidden step is not
  drawn on the field and cannot be clicked there; it stays in the route, its row is dimmed with "hidden", PREVIEW and
  Export are unchanged. **Show all** in one click.
- **D2 — Lock a step** (a lock icon, `L`): a locked step is drawn but cannot be dragged or nudged; clicks on it pass to
  what is under it. Editing it in the inspector stays possible (typed numbers), with one line saying it is locked.
- **D3 — Focus** (a target icon, `F`): the selected step — and its neighbours, the one before and the one after — drawn
  at full strength, every other step faded and not clickable, until focus is turned off (`F` again or Esc). ONE idea at a
  time: this is the default way to edit a crowded spot.
- **D4 — Click-through for what still overlaps:** clicking a spot where several unlocked, shown points overlap picks
  the topmost; clicking again at the same spot (without moving) selects the next one under it (cycling), and the
  inspector says "2 of 3 here". Hovering shows which step a point belongs to.
- **D5 — Sections (light "layers"):** the step list can fold a range of steps into a named section ("Start", "Claw 1",
  …; added from a step's menu); a folded section's steps can be hidden or locked together with one click. Optional in
  the project file; an older project opens with none.
- **D6 — Saved, never exported:** hide/lock/focus and sections are saved in the project file (`route_maker.json`) as the
  page's own view state; the exported C++ is byte-for-byte unchanged (the RMP1-C goldens); undo/redo covers hide, lock
  and sections (not focus).

## 3. Tests (adversarial — each seen RED against a mutant)

Node: hide/lock/sections in the project round-trip; an old project opens unchanged; the export ignores them
(byte-for-byte). Page (Playwright, Firefox + Chromium): a hidden step's point is not drawn and a click there selects
the step under it; a locked step does not move on drag or on the nudge keys and a click passes through; focus fades all
but the selected and its neighbours and Esc clears it; three points on one spot cycle on repeated clicks with "k of 3";
a section hides/locks its steps together; undo/redo; screenshots light/dark at 1440 and 1280 (a crowded route before and
with focus on).

## 4. Docs

Guide 22: one short section "Crowded spots: hide, lock, focus" with a screenshot, and the keys table (`H`, `L`, `F`);
RMP1-PROGRESS (`## RMP1-G`).

## 5. Landmines

- Beside you on the SAME page (do not edit their files): RMP1-D (`controls_tab.mjs`, `feel_tab.mjs`), RMP1-E
  (`preview.mjs`, `timeline.mjs`, `src/model/playback.mjs`, `wasm/`), RMP1-F (`export_drawer.mjs`, `host/files.mjs`,
  `tools/upload_auton.py`). Yours: `field_view.mjs`, `step_list.mjs`, `inspector.mjs`, `src/model/project.mjs` (view
  state), keys in `app.mjs` (one delimited block). CSS in ONE delimited block of `style.css`
  (`/* ── Hide / lock / focus (RMP1-G) ── */`).
- Never run pros or touch /dev/ttyACM*.
