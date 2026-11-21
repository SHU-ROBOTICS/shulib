# RMP1-M — the Mechanisms tab: see every mechanism on the robot, test each one, and "find the bottom" instead of "home"

> Brief by the coordinator, 2026-11-20 ~15:45 EST. **The team lead, 2026-11-20:** "how simple is it to see what
> mechanisms are on the bot and how we can add and take away and easily test and add to the route builder" — then,
> on the coordinator's proposal (a Mechanisms tab today; adding and removing after the 21st with the one setup
> program, SETUP1): **"write it in and start today on the mechanisms"**; and "what does homing the cascade mean it
> just is a bit confusing". Route builder and its host tools only: **no robot code, no library** (the freeze holds).
> The competition is SATURDAY Nov 21 (tomorrow): fast, nothing that guards correctness skipped. He finds dense
> screens "overstimulating": ONE idea and ONE highlight per screen is acceptance.

## Today (what this fixes)

- What is on a robot is spread over `src/robots/mechanism_table.hpp`, both facts files and both driver files (one
  mechanism, the right toggle, is spelled in ~30 places across 6 files). The page's M menu lists the mechanisms, the
  Controls tab their buttons; nothing shows a mechanism whole.
- Testing one mechanism alone needs Drive or Bench Tests' station 13. RMP1-K's **Check the mechanisms** tests a
  ROUTE's mechanism steps, not a mechanism.
- "Home" / "start homing" is a word the team does not use.

## Decisions

- **D1 — a Mechanisms tab** beside Route and Controls (the top bar's switch). For the robot chosen (robot one / robot
  two, through ONE local name helper — see "Beside you"), one card per mechanism, in the table's order. Each card,
  plain words, one idea: **what it is** ("a lift: 0 to 20 in", "a motor that runs while a button is held", "a
  piston: closed / open"), **its ports** (motor ports and which way each turns; a piston's line), **its buttons**
  (from the robot's driver file, as the Controls tab reads it), **its levels** (a lift; "no levels yet" when there
  are none — robot one has none today), and **which routes use it** (route names; a click opens that route at its
  first step using it). A card whose robot fact is still OWED says so in one line ("port not known yet: the
  builders' answer"), never a guess.
- **D2 — the ports reach the page.** `robots.json` gains each mechanism's wiring (motor ports with direction; a
  piston's ADI line) from the robot's own table (`test/route_maker_main.cpp` — host tool, not robot code); an OWED
  fact is `null` and said as owed. The site build and the helper's local facts carry it; the facts-differ warning
  (route_builder.py) still guards a checkout whose robots differ.
- **D3 — Test, one mechanism.** Each card has **Test**: it builds a one-mechanism CHECK route with RMP1-K's machinery
  (`check_route.mjs`; the same `<route> mechs`-style name, the same "what will move" dialog and its safety line,
  the same Send, the same write-back so a Competition build never lists it) and opens **Watch the robot**. What it
  does, per kind: a **lift/arm** finds its bottom (D5), goes to each level (or, with none, to the middle of its
  range), then back to the bottom; a **power** mechanism runs one way for a short time, then the other way; a
  **piston** goes to each position, then back to its start. Every step carries on (`onFail: carryOn`), every step
  bounded. The card ticks green / amber / red from the robot's own lines (RMP1-K's run log), and keeps the last
  result with its time. **Test every mechanism** at the top does all of them in one check route.
- **D4 — not today:** adding or removing a mechanism from the page (after the 21st, with SETUP1's measurement); any
  change to `src/`, `include/` or the Makefile.
- **D5 — "find the bottom" for "home".** Wherever the page SHOWS a person the word home / homing for a lift or arm
  homed at its BOTTOM stop (by stall or a switch), it says **find the bottom** ("cascade: find the bottom", "Find the
  bottom first: it does not know its height until it has"); homed at the top, **find the top**. The help says once
  what it means: *the motors count turns from wherever they were at power-up, so the robot does not know the
  cascade's height until it pushes gently down onto its bottom stop and calls that 0 in.* NOT changed: the exported
  C++ (`home()` / `startHome()`; the goldens byte-for-byte), the robot's own quoted lines, data keys.

## Beside you

- **RMP1-L is in flight on the same page**: White Bot / Black Bot — every SHOWN robot name through ONE name source.
  So in your new files name a robot ONLY through one helper of your own (`robotWords(which)` returning today's "robot
  one" / "robot two"), so the merge maps it to RMP1-L's source in one line. Do not rename other screens' robot words.
- RMP1-K's code is yours to call, not to change, except where D3 needs a one-mechanism entry point (keep it small;
  its tests stay green).

## Acceptance

- Exported C++ byte-for-byte (the RMP1-C goldens); node, helper self-test, upload self-test, page suites in Firefox
  AND Chromium green; `robots.json` change pinned (both robots, an owed port as null, the site build).
- D3's check route per kind pinned by node tests (a lift with levels / none, homed by stall / switch / not at all;
  power; piston; a robot with an owed port: Test says why and sends nothing); the confirm dialog names exactly what
  will move; nothing is sent without it.
- Mutants (each RED): a card with the wrong port; a Test that skips the "what will move" dialog; a lift test that
  moves before finding its bottom; "home" shown somewhere a person reads it; an export golden touched.
- Screenshots light/dark at 1440 and 1280: the tab (robot one), one card's Test dialog, a card after a run; read each
  for one idea per screen and nothing crowded.
- Docs: guide 22 (one short section with one picture: the tab), the route builder README, the changelog (2.11, what
  people see), RMP1-PROGRESS.
