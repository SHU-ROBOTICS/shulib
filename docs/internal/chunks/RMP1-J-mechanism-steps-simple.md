# RMP1-J — mechanism steps made simple: pick the mechanism, pick what it does, see where it happens

> Brief by the coordinator, 2026-11-20 ~07:30. **The team lead, 2026-11-20:** "using the mechanisms in the route
> builder is a bit convoluted" (and "how do we know the mechanisms work"). One run, **RMP1-J-A**. The route builder
> stays open to the 21st (competition: SATURDAY Nov 21). NO library (`include/`) change, NO `src/` change. The
> exported C++ byte-for-byte for the same steps (the RMP1-C goldens). Base: integrate `ae87076`.

## 1. What is convoluted today (`src/ui/app.mjs` `openMechanismMenu`, `src/ui/words.mjs` `mechanismActions`, `src/ui/inspector.mjs` `mechanismFields`)

- **M** opens ONE long menu of every mechanism's every action — robot one: the cascade's 7, each side toggle's 2,
  the claw's, flipper's and ramp's 2 each = 17 rows, every row the same gear icon, the last ones below the fold
  (`docs/assets/route-builder/mechanism-menu.png`). The team lead finds dense screens "overstimulating".
- A lift move is five modes (Go up / Go down / Move to / Back to the bottom / A level) chosen in the menu AND again
  in the inspector, then two typed numbers (how far, the time limit), then "Wait until it is done" as a toggle whose
  off-state is described as "started: the route keeps going (a 'wait until done' collects it)".
- A mechanism step has no place on the field: it is a row in the list after the selected step; nothing on the field
  says "the claw closes HERE".
- The time limit is always empty and red until typed, with no hint of a sensible number.

## 2. Decisions (ONE idea per screen)

- **D1 — The menu is two short steps.** **M** shows the robot's mechanisms only — one row each, its name and a
  simple kind icon (lift, motor, piston), at most ~6 rows. Click one: its actions replace the list (a "‹ Back" at
  the top), each a big button in plain words — the claw: **Open claw** / **Close claw**; a side toggle: **Run it**;
  the cascade: **Move the cascade** (one button for every height move, D2), **Home it**, and **Wait for it** only
  when the route has a step that starts it without waiting. A mechanism with ONE action (a piston with two
  positions shows both) is still two clicks at most. Keys: the arrows and Enter walk it; Esc goes back, then closes.
- **D2 — One height control for a lift (and an arm):** the inspector shows the mechanism's travel as a vertical bar
  (its bottom, its top, its named levels as marks, where it is BEFORE this step from `plannedHeights`), and ONE
  field "Height: [ 18 ] in" — click the bar or a level mark, or type. The page writes the same step kinds as today
  (`to`, `preset`, `bottom`; `up`/`down` stay readable and editable for old projects, shown as "up 6 in → 18 in" with
  a one-click "make it a height"), so the export is unchanged for the same intent. A level whose height is OWED is a
  mark with no number ("not measured yet"), never a placeholder shown as a fact.
- **D3 — "Wait for it?" in plain words:** two choices on the step, **Wait here until it is done** / **Keep driving
  while it moves**; the second says, in one line, where the route waits for it (the step that does, or "add a wait
  before it is needed" with a one-click fix — the existing check's fix).
- **D4 — The time limit, suggested from PREVIEW, never invented:** the field stays empty until the student fills
  it, but once PREVIEW has run, the field shows a one-click **Use 1.4 s** with the reason "PREVIEW took 0.9 s here,
  + 0.5 s" (the library's own run on the host test plant; a PLACEHOLDER-gain mechanism says so in the same line).
  Without a PREVIEW, the hint says "Run PREVIEW to get a suggestion". Same for drive steps' time limit (the same
  rule; the team lead hit a 1.5 s limit failing a 24 in drive). This AMENDS RMP1's "the page never invents one":
  the number is the library's measured run, shown with where it came from, and the student clicks it.
- **D5 — Where it happens, on the field:** each mechanism step is drawn as a small badge on the planned path at
  the robot's pose when it starts (the claw icon + "close"), numbered like its row; clicking it selects the step;
  focus/hide/lock (RMP1-G) apply to it. A started (keep-driving) move draws its badge where it starts and a thin
  bracket along the path to where the route waits for it.
- **D6 — Homing without a puzzle:** adding the FIRST height move of a stall-homed lift to a route with no home adds
  nothing silently; it shows ONE card: "The cascade must find its bottom first — **Home it at the start** (while the
  first leg drives)" — one click inserts the existing "home while the first leg drives" fix (its time limit then
  gets D4's suggestion).

## 3. Tests (adversarial — each seen RED against a mutant)

Node: the two-step menu's model (a robot's mechanisms → rows; each kind → its actions; no action duplicated or
lost compared with today's `mechanismActions`); D2's height choice writes exactly today's step for the same intent
(golden: the RMP1-C exported C++ byte-for-byte for every mechanism step kind); an old project's `up`/`down` steps
open, edit and export unchanged; D4's suggestion = PREVIEW's step time + 0.5 s, rounded up to 0.1 s, absent
without a run, never written without the click; D6's card appears once and inserts today's fix. Page (Firefox AND
Chromium): M → mechanism → action adds the right step at the right place; the bar click sets the height; the badge
sits at the step's start pose and selects its step; Esc/Back; screenshots light/dark at 1440 and 1280 (the menu's
two steps, the height bar, a route with three badges).

## 4. Docs (one idea per screen)

Guide 22 step 5 rewritten around the two-step menu, the height bar and the badges (screenshots replaced:
`mechanism-menu.png` and the inspector); the D4 suggestion in step 6 (PREVIEW) and the keys table; the
changelog; RMP1-PROGRESS (`## RMP1-J`). Re-record nothing (the coordinator re-records the make-a-route video
after the merge).

## 5. Landmines

- **In flight beside you on the SAME page: RMP1-I** (`export_drawer.mjs`, `to_robot.mjs`, a new `send_panel.mjs`,
  a new `host/files_local.mjs`, `host/capabilities.mjs`, the top bar's one new button and the host pick in
  `app.mjs`, `tools/route_builder.py`). Do not edit their files; in `app.mjs` touch only `openMechanismMenu` and
  what it calls (keep your edits there contiguous). Your CSS in ONE delimited block of `style.css`
  (`/* ── Mechanism steps (RMP1-J) ── */`); the badges in a NEW module (`src/ui/mech_badges.mjs`) with ONE hook
  line in `field_view.mjs`.
- `src/model/route.mjs`'s step shape and `src/model/export.mjs` do NOT change (the exported C++ is the
  contract); only how the page builds and shows the steps.
- Never run pros or touch /dev/ttyACM*.
