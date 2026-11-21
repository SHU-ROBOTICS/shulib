# RMP1-P — "Leave gently": a wait and a slow back-out after a drop, one clear option on the Back up step

> Brief by the coordinator, 2026-11-20 ~18:25 EST. **The team lead, 2026-11-20**, after his test route knocked the
> stack over backing away from it: "that backwards motion needs to be a lot slower are we able to do something like
> that where we can add delays and make actions go slower in the route builder?" — then: **"wait and a slow
> back-out after each drop make this a clear option under the back up in the route builder"**. Route builder only:
> no robot code, no library. The competition is SATURDAY Nov 21: fast, nothing that guards correctness skipped.
> ONE idea per screen.

## Today

A drop is a mechanism step that lets go (the claw to "open"; any pneumatic or arm step whose position is marked as
releasing). The page already has every piece — a **Wait** step, **Back up** (`reverseTo`, straight back with no
turn first when it is a straight back-up), a per-step **max speed** — but a student has to know to add a Wait and
dig the speed out of the step's settings; the test route that worked (the coordinator's, 18:20) used: wait 0.5 s,
then back up 4 in straight back at most 8 in/s, before any turn.

## Decisions

- **D1 — the option.** A Back up step's inspector shows, as its FIRST control under the step's title, one toggle:
  **"Leave gently"** — sub-line "waits ½ s, then backs out slowly (5 in/s)". On: the step exports as the Wait and the
  slow back-up together (one step on the page, two lines in the C++, in the words the page already uses for each);
  the two numbers are editable right under the toggle (wait 0–2 s, speed 2–20 in/s), defaulting to 0.5 s and 5 in/s (8 was too fast: the team lead; below ~4 in/s robot one creeps and stalls, 11-19).
- **D2 — offered where it matters.** A Back up that directly follows a drop (or follows it with only waits between)
  starts with "Leave gently" ON when it is added; the step card shows a small "gently" tag. Any other Back up starts
  OFF. Turning it off is one click; nothing is ever changed on an existing route by loading it.
- **D3 — PREVIEW and the checks** run the wait and the capped speed exactly as exported (the run is the library's
  own); the time it adds shows in the timeline like any step.
- **D4 — not today:** a gentle option on other verbs; anything automatic beyond D2's default.

## Beside you

RMP1-O (the claw's spot: a marker and "Place by the claw" in the inspector of a DRIVE step) is building — keep your
inspector change to the Back up step's block, contiguous; RMP1-M (Mechanisms tab), RMP1-N (camera card) and RMP1-L
(robot names through one source — name a robot only through one local helper) are in flight.

## Acceptance

- Exported C++ byte-for-byte for every route that never turns it on (the goldens); with it on, the exact two lines
  (pinned for 0.5 s / 8 in/s and an edited pair), and the route still exports the same after save and reload.
- node tests: D2's default (after a claw open; after a drop then a wait; after a drive: off), the bounds (refused in
  words outside them), the JSON round trip. Page tests (Firefox AND Chromium): the toggle, its sub-line, the tag, the
  numbers. Mutants (each RED): the wait dropped from the export; the speed cap dropped; D2 on after a non-drop;
  the toggle changing an existing route on load.
- Screenshots light/dark at 1440 and 1280 (the inspector with it on; the step card's tag); read each.
- Docs: guide 22, one sentence + one picture ("after a drop, Leave gently"); the route builder README; changelog
  (2.11, what people see); RMP1-PROGRESS.
