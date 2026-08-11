# Chunk A4 — Hardware Assumptions Register + ARM compile gate

> **Phase A, chunk 4 of 39 — the chunk that closes Phase A.**
> Predecessors: A1 ✅ (diagnostics), A2 ✅ (host plant), A3 ✅ (hostile fakes).

**Workstream:** WS11 (Tooling/CI) + WS2 · **Milestone:** M1/M2 · **Spec:** build-order Phase A

---

## Why this chunk exists

Three chunks of work now rest on claims about physical hardware that **cannot be checked without a
robot**. A3 alone produced **25 invented magnitudes**, each labelled `PROVISIONAL (A4)` in-header.
A2 declared its `DrivePlantConfig` gains placeholders. `gps_conversion.hpp` has carried a
"validate-on-field" note since June, with a *skipped* oracle at `test/gps_conversion_test.cpp:163`.

Right now that debt is **scattered across headers and completion records**. This chunk makes it
**inventoried** — one document, every claim falsifiable, each paired with the test that will settle it
and the chunk that owns settling it.

**What it buys:** when hardware finally arrives, R3 is a prepared checklist walked top to bottom, not
an open-ended exploration. And when a value turns out wrong — some will — the register localizes the
blast radius instead of leaving a mystery.

This is not paperwork. It is the artifact that converts "we built a lot without a robot" from a risk
into a plan.

---

## Scope

### In
1. **The Hardware Assumptions Register** — `docs/hardware-assumptions.md`
2. **Bidirectional reconciliation** — every `PROVISIONAL (A4)` label in the tree maps to a register
   entry, and every register entry that came from a header points back at it. **No orphans in either
   direction.**
3. **The ARM compile gate in CI** — a translation unit including every v2 header, cross-compiled by
   `arm-none-eabi-g++` in the workflow
4. **Close Phase A** — roadmap and build-order status updated to Phase C next

### Out
- Measuring anything (that is R3/R4/R5) · new library features · anything on-robot

---

## The register

### Where to harvest from — be exhaustive, this is the point of the chunk

- **Every `PROVISIONAL (A4)` label in the tree** — grep for it; A3 queued 25
- `A2-COMPLETED.md` and `A3-COMPLETED.md` honest-limits sections
- `hal/gps_conversion.hpp` — the axis-assumption / validate-on-field note, and the **skipped oracle**
  at `test/gps_conversion_test.cpp:163`
- `hal/imu_conversion.hpp` — the pinned contracts (get_rotation binding, no-post-cal-tare,
  bootHeading ownership, yaw-rate source)
- `sim/drive_plant.hpp` — `DrivePlantConfig` placeholder gains
- `localization/tracking_wheel.hpp`, `pilons_odometry.hpp` — offsets, direction signs, wheel diameters
- F5's on-V5 number-match (still unvalidated), motor cartridge/gearing ratios, PROS call latency,
  loop-rate assumptions, battery sag curve

### Entry shape

Each entry is a **falsifiable claim**, not a topic. Suggested columns:

| Field | Meaning |
|---|---|
| **ID** | stable (`HA-01`…) — cited from headers and from R3's checklist |
| **Claim** | stated so it can be proven *false* — a number with units, or a definite assertion |
| **Source** | file/line where it is currently assumed |
| **Confidence** | measured elsewhere / reasoned / **invented** |
| **How it gets settled** | the specific bench measurement or test |
| **Owning chunk** | R3 (conventions/geometry), R4 (noise/drift), R5 (gains), R6 (plant back-fit) |
| **Blast radius if wrong** | what breaks, and how far it propagates |

**Blast radius is the highest-value column.** It is what tells a future reader whether a wrong value
is a one-line constant fix behind the HAL seam or something that invalidates a phase of work. Where an
assumption is contained *by design* (e.g. behind `hal/pros`), say so — that containment is a real
result of the architecture and worth recording.

### Rules
- **Do not resolve anything.** No robot exists. An entry that *can* be settled on the host is a bug in
  the entry — settle it and note that instead.
- **Be honest about confidence.** "Invented" is a legitimate and useful value; a guess dressed as a
  measurement is the failure mode this document exists to prevent.
- Sort or group so R3 can walk it as a **checklist**, not prose.

---

## The ARM compile gate

The v2 core is verified to cross-compile clean (77 headers as of A3), but **nothing keeps it that
way** — CI builds host only, so a host-only assumption could enter the core unnoticed and surface at
R1 when it is expensive.

Add to `.github/workflows/ci.yml`: install `gcc-arm-none-eabi`, generate a TU including every header
under `include/shulib/`, compile it with the same strict flag set used for host, fail the job on any
error. **Generate the header list rather than hard-coding it**, so a new header is covered
automatically — a gate you must remember to update is a gate that rots.

Flags that match current local verification:
```
arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror \
  -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp
```

Note in the workflow *why* this exists: it is a **compile** gate, not a link or run gate — the on-robot
build still needs PROS's toolchain and ultimately the robot (R1/R3).

---

## Definition of Done

- [ ] `docs/hardware-assumptions.md` exists, every current assumption entered as a falsifiable
      claim with source, confidence, settling method, owning chunk, and blast radius
- [ ] **Bidirectional reconciliation complete** — every `PROVISIONAL (A4)` in the tree maps to an
      entry; every header-sourced entry points back. Zero orphans, verified by grep and stated
- [ ] Headers updated to cite register IDs where they currently say `PROVISIONAL (A4)`
- [ ] CI cross-compiles all v2 headers for ARM with a **generated** header list, and fails on error
- [ ] The gate is proven to work — demonstrate it catching something (a deliberate host-only
      construct), then restore
- [ ] Roadmap + build-order updated: **Phase A closed, Phase C (C1 — `IMotion` + motion primitives)
      next**; the ARM-gate roadmap item flipped with cited evidence
- [ ] Full suite green; both existing CI guards still pass

---

## Live progress log — required

Append to `docs/internal/chunks/A4-PROGRESS.md` as work happens (`date +%H:%M:%S`), same vocabulary:
`START` / `DONE` / `MUTATE` / `DECIDE` / `BLOCKED` / `FOUND`. Watched live with `tail -f`.

---

## Documentation contract

All six, plus **`docs/internal/chunks/A4-COMPLETED.md`**. A4 is smaller than A2/A3, so match their
*rigor* rather than their length — but the register itself must be exhaustive.

Also write a short **Phase A retrospective** into the completion record: what the phase set out to
build (the substitute for hardware), what it actually produced, and — most usefully — **what it
found**, since A3's three Localizer defects are the concrete evidence the approach paid off.

**Do not commit.** Leave everything in the working tree for review.

---

## Landmines

- **An incomplete register is worse than none** — it creates false confidence that the debt is known.
  Grep exhaustively; if you are unsure whether something is an assumption, enter it.
- **Don't let the gate hard-code a header list.** It will rot the first time someone adds a file.
- **Don't quietly resolve assumptions to shrink the list.** The list is *supposed* to be long; that is
  an honest measure of what building without hardware costs.
- **Prove the gate catches something.** An untested gate is a comment.
