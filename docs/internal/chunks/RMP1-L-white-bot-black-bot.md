# RMP1-L — White Bot and Black Bot: the robots' names in what people see (route builder, VS Code tasks, upload words)

> Brief by the coordinator, 2026-11-20 ~14:50 EST. **The team lead, 2026-11-20 12:40:** "also we will name the bots
> White Bot and Black bot instead of 1 and two" — asked how far, he chose **"What people see, today"**: the route
> builder, the VS Code tasks and the upload words today; the brain's screens, the C++ names and the docs **after the
> 21st**. Robot one = **White Bot** (ROBOT=one, brain 2F007C00); robot two = **Black Bot** (ROBOT=tank).
> The competition is SATURDAY Nov 21: fast, nothing that guards correctness skipped.

## What changes (and what must NOT)

- **D1 — the page.** Every word the route builder SHOWS a person that names a robot says **White Bot** / **Black
  Bot** (buttons, panel headings, inspector labels, check and problem messages, toasts, the help, the Send/What-is-on
  dialogs, the timeline, PREVIEW's words, the controls tab). ONE place decides the names (e.g. `robotName('one')` in the
  page's words module); no other file spells "White Bot". Capitalised as a name ("White Bot's start", "Send to White
  Bot"), never "white bot". A heading in capitals stays CSS-uppercased from the same words.
- **NOT changed (acceptance):**
  - the **exported C++** — byte-for-byte (the RMP1-C goldens and every export test unchanged; any "robot one" inside
    generated code or its comments stays);
  - **lines quoted from the robot or the library** (a brain's log, `[ATS]`/`[RTN]` lines, the library's refusal
    text the page shows verbatim) — the robot still says "robot one" until after the 21st;
  - file names, `ROBOT=one|tank`, `src/team/robot_one_*`, data keys (`'one'`/`'two'`) and URLs;
  - the docs (after the 21st; the route builder README's words may name both: "White Bot (robot one)").
- **D2 — the bridge.** Where a person must match the page to a file or a doc that still says "robot one", say both
  once: the export drawer's file line ("White Bot (robot one): `src/team/robot_one_autons.cpp`") and the help's first
  mention. Nowhere else.
- **D3 — VS Code tasks** (`.vscode/tasks.json`): labels "Upload routes to White Bot (robot one)", "… Black Bot
  (robot two)", "Upload everything to White Bot (robot one)", …, "What is on White Bot (robot one)"; the picker's
  robot choices "White Bot (robot one)" / "Black Bot (robot two)" (values unchanged). The helper's self-test that pins
  the task labels follows; guide/README text naming a task by its old label is updated in the same commit (the
  task's name only — not a docs rewrite).
- **D4 — upload words.** `tools/upload_auton.py` and `tools/route_builder.py robot …` accept `white` / `black` as
  well as `one` / `two` (same meaning; case-insensitive), and their printed sentences say "White Bot" / "Black Bot"
  where they said "robot one" / "robot two" (e.g. "Building White Bot's Drive: make ROBOT=one PROGRAM=drive").
  Self-tests: the aliases, and one old-word case each still working.

## Acceptance

- Exported C++ byte-for-byte (goldens); node tests, helper self-test, upload self-test, page suites in Firefox AND
  Chromium green, every test that asserted "robot one" text updated to the new words — never deleted.
- A grep proving no shown string still says "robot one/two" (list each remaining hit and why it stays: C++, quoted
  robot line, key, file name).
- Mutants (each RED): the name source returning "robot one"; one dialog bypassing it; the alias `white` mapped to
  `two`; an export golden touched.
- Screenshots light/dark at 1440 and 1280 of: the top bar + Send button, the inspector's robot headings, What is on
  White Bot, the export drawer; ONE idea per screen still holds (the names must not make a label wrap badly — read
  every shot).
- Docs: changelog line (2.11 section, "What people see"), RMP1-PROGRESS entry, the route builder README.
