# RMP1-D — the Controls tab, one thing at a time (layout only)

> Brief by the coordinator, 2026-11-19 ~18:05. **The team lead's go, 2026-11-19:** "THE CONTROLS TAB … THAT IS AN
> INFORMATION OVERLOAD" and "the formatting of the controls tab needs to be worked on" (with "put all resources into
> getting everything done"). One run, **RMP1-D-A**. The route builder stays open to the 21st (RMP1's D19). LAYOUT AND
> WORDS ONLY: what the tab EXPORTS (the driver file's three parts) must stay byte-for-byte the same. Base: main `8b2d99e`.

## 1. Why — the tab today (screenshot: `.claude/coordinator/briefs/RMP1-D-controls-1440.png`, 1440×900)

- **Three columns do one job:** MECHANISMS (left: each mechanism's chips + "Add"), the controller (middle: click a
  button), CONTROL (right: mostly empty — a help sentence and "Ready to export"). A student does not know where to start.
- **The controller's labels are ~9 px** ("L2 · left toggle: hold to run", "Up · free") — unreadable; leader lines crowd.
- **Driving feel shows everything at once:** three styles (Arcade / Tank / Field arcade), two sub-tabs (Forward and back /
  Turning), three sliders each with a number box (Dead zone, Curve, Ramp), the graph, "When the sticks are let go", "Re-zero
  button" — and its bottom is **cut off** behind the footer at 1440×900 (and worse at 1280×800).
- "WITH NO BUTTON" is unexplained.

## 2. Decisions

- **D1 — Two steps, one shown at a time.** The tab's body is two sections the student moves between: **1 · Buttons**
  and **2 · Driving feel** (a segmented control at the top of the tab; Export stays in the top bar). Only one section is
  on screen at a time.
- **D2 — Buttons: the controller is the page.** The controller drawn LARGE and centered, every label ≥ 13 px and
  readable at 1280×800, a free button clearly marked (dashed + "free"). Clicking a button (or a label) opens ONE editor
  panel beside the controller: the button's name, its job now, and its choices as large buttons (the mechanisms' jobs,
  "free"), plus the existing hold/toggle choice. The MECHANISMS column becomes a compact list inside that panel's empty
  state ("Click a button to give it a job. Your mechanisms: cascade, right toggle, …"), never a third column. "WITH NO
  BUTTON" becomes a plain sentence in that empty state (e.g. "The cascade homes by itself when driver control starts —
  no button needed").
- **D3 — Driving feel: the essentials first.** Shown first: the style (Arcade / Tank / Field arcade, as three clear cards)
  and, for the chosen axis, **Dead zone** and **Curve** with the graph beside them. Everything else (Ramp, "When the
  sticks are let go", "Re-zero button", the Turning axis if it is a sub-tab) under one "More settings" disclosure, closed
  by default. Nothing cut off at 1280×800 or 1440×900 (the section scrolls inside the tab if it must).
- **D4 — One highlight at a time:** the selected button is the only accented element on the controller; hover and focus
  states visible; keyboard reachable (Tab/Enter/Escape close the editor).
- **D5 — Nothing else changes:** the project file format, the model (`src/model/`), `controls_export.mjs`'s output (the
  three parts, byte-for-byte), Undo/Redo, the robot switch (one / two), the Route tab, the top bar.

## 3. Tests (adversarial — each seen RED against a mutant)

Page (Playwright, Firefox + Chromium): the two sections switch and only one is visible; every controller label's computed
font-size ≥ 13 px and none overlaps another (bounding boxes) at 1280×800, 1366×768, 1440×900; clicking a button opens
the editor with its current job, choosing a job updates the controller and Undo restores it; More settings closed by
default and every feel control still reachable inside it; nothing in the tab is clipped (scroll extents) at those sizes;
light/dark screenshots at 1440 and 1280. Model/export: the existing controls export goldens unchanged (byte-for-byte) —
re-run them; a mutant that changes an export byte must fail them. Re-take guide 22's `controls.png` /
`controls-export.png` if the tab's look changed.

## 4. Docs

Guide 22's Controls sections: the new two steps in its words; RMP1-PROGRESS (`## RMP1-D`). Short and plain.

## 5. Landmines

- **In flight beside you, the SAME page:** RMP1-C (named routes) changes `src/ui/app.mjs`, `style.css`, `dom.mjs`,
  `words.mjs`, `help.mjs`, `export_drawer.mjs`, `inspector.mjs`, `preview.mjs`, `field_view.mjs`, `src/model/*`. Work in
  `src/ui/controls_tab.mjs` and `src/ui/feel_tab.mjs`; put new CSS in ONE clearly delimited block of `style.css`
  (`/* ── Controls tab (RMP1-D) ── */ … /* ── end Controls tab ── */`) so the coordinator's merge is mechanical; touch
  `app.mjs` only if there is no other way, and say exactly where.
- A tutorial video of the Controls tab is being recorded from TODAY's layout (`tools/docs_video/controls_tab/`); it is
  re-recorded after you land — keep element names/roles stable where you can (`getByRole('tab', {name: 'Controls'})`,
  each controller button's accessible name).
- Never run pros or touch /dev/ttyACM*.
