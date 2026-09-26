# T3 addendum — the pneumatic claw on A (toggle), robot two

> **Execution brief (addendum to `T3-claw-pivots-bindings-and-mechanisms.md`).** Written 2026-09-26 ~03:30.
> The team lead: *"when you press A on the controller ... it opens the claw using pneumatics and when you press
> it again it closes the claw. THAT IS ALSO PART OF THE DRIVER CONTROLS"*; *"we cant move fast though and make
> mistakes we just need to move as fast as we can."* Everything in the T3 brief applies; this adds one binding
> type and one mechanism kind. **Two follow-up runs, each in the worktree that already owns the files:**
> **B2** (library half, `.claude/worktrees/t3-lib`, after the T3 library run finishes) and **E2** (program half,
> `.claude/worktrees/t3-prog-prep`, after the T3 prep run finishes). **Executor model: `opus`. Do not commit.**
> Log into the SAME `T3-PROGRESS.md` of that worktree under a heading "Addendum: the pneumatic claw".

## 0. The facts (the team lead, 2026-09-26)
- The claw's solenoid is on the brain's **3-wire port A**.
- Single or double solenoid, and whether ON opens or closes the claw: **NOT KNOWN.** Build it as **ONE line on
  port A** with the open polarity as **ONE named constant marked UNVERIFIED** ("ON = open" as the starting
  guess); Station J confirms it by pressing A. A double solenoid would use `MechanismConfig`'s second
  pneumatic port — say how, do not build it speculatively.
- **At program start the claw is CLOSED.**
- **When the robot is disabled the claw KEEPS its state** (it must not drop what it holds): the driver-control
  toggle commands NOTHING on a disable or a disconnect. Whether the brain holds a 3-wire output's level while
  disabled is UNMEASURED — register it; Station J checks it.
- **The controller button A toggles the claw.** A was G1 Part 2b's Teleop field-centric re-zero button
  (`kRezeroButton = A` in `src/teleop_program.cpp`); **the re-zero button moves to Y** (the team lead's ruling);
  the coordinator makes that one-line Teleop change at merge. Every binding button must stay unique and none may
  equal the re-zero button (Y).

## 1. What exists
`hal::PneumaticMechanism(span<IDigitalOut* const>, bool safe, name)` — `set(bool)`, `commanded()`,
`applySafeState()` = `set(safe)` (`hal/mechanism.hpp` ~:286-340); `hal::IDigitalOut` (`hal/digital_out.hpp`);
`hal::pros::ProsDigitalOut` (`hal/pros/digital_out.hpp`); `MechanismConfig.pneumaticPorts[]` /
`pneumaticCount` (ADI lines 1..8, `config/robot_config.hpp` ~:345-350); `hal::ButtonEdge` (per-consumer edge,
HA-104). Missing: a toggle binding; `IDeviceProvider` has no digital-out member (handoff L9); `robot::Robot`
builds no pneumatic mechanism.

## 2. B2 — the library half (worktree `t3-lib`)
1. **A toggle binding** beside the hold binding (same header or a sibling in `include/shulib/teleop/`), over one
   `hal::PneumaticMechanism&`: its OWN `ButtonEdge` on one button; **each rising edge flips the commanded
   state exactly once** (holding the button does nothing more); the initial state is STATED at construction
   (no default) and commanded once at loop entry; buttons are read only while connected, and a disconnect,
   a reconnect or a disable command NOTHING (the claw keeps its state); an edge that happened while
   disconnected is never replayed; it takes no claim and never commands a mechanism someone else has
   `claimed()`. Every rule in the header and pinned by a test.
2. **The loop carries it:** the bindings overload of `runDriverControl` (from the T3 run) accepts toggle
   bindings too (one span of a common binding interface, or a second span — your call, stated); on
   `disabled()` the loop makes the HOLD bindings safe and leaves TOGGLE bindings untouched.
3. **`IDeviceProvider` gains a digital-out member** (additive; the provider is unfrozen), `ProsDeviceProvider`
   implements it over `ProsDigitalOut` for ADI lines 1..8 (the 3-wire ports are not smart ports: the registry
   cannot confirm a solenoid — say so and register it), the fake provider too; same-line-twice returns the same
   object.
4. **`MechanismConfig` pneumatic mechanisms:** the pneumatic safe value and the START value, each with an UNSET
   representation (never an implicit `false`), refused by `valid()` when `pneumaticCount > 0` and unset;
   `robot::Robot` builds a `PneumaticMechanism` for a config with pneumatic lines (and a mixed mechanism is
   refused or handled explicitly — say which), names it, exposes it through the same `mechanism(name)` lookup
   and the RunGuard span. A line the provider refuses ⇒ that mechanism ABSENT, reported by name, the drive
   still built.
5. **Tests (adversarial):** one press ⇒ exactly one flip; a held button over 50 ticks ⇒ still one flip; press,
   release, press ⇒ two flips; the initial state commanded once at entry and not re-sent; a LYING controller
   (A pressed while `isConnected()` false) ⇒ no flip, and no flip replayed on reconnect; a disable while open ⇒
   nothing commanded (still open); the toggle never touches the hold bindings' mechanisms and vice versa;
   construction refusals (the toggle button equal to a hold binding's button or the re-zero button); the
   provider's digital out (same line ⇒ same object; out-of-range line refused); config refusals for unset
   start/safe values; Robot builds and names the pneumatic mechanism; a refused line ⇒ ABSENT, drive built.
   Mutations RED: flip on level instead of edge; replay an edge across a disconnect; command safe on disable;
   default the start value.
6. Docs you own: the header comments, `docs/changelog.md` ("Added since 2.2": the toggle binding, the provider's
   digital out, pneumatic mechanisms from config), `docs/guide/10-the-api.md`, the cookbook recipe extended (a
   compiled example), `docs/vexbuilder-integration.md` (pneumatic fields), `docs/api` regenerated, the
   briefing regenerated. HA numbers: use ONLY **HA-157, HA-158** (a 3-wire output's level while the brain is
   disabled — UNKNOWN; the registry cannot see a solenoid on a 3-wire port — reasoned).

## 3. E2 — the program half (worktree `t3-prog-prep`)
1. **`src/mechanism_table.hpp`** gains the pneumatic claw: name "claw", ADI line **A** (line 1), start **CLOSED**,
   the open polarity as ONE named constant `kClawOpenCommand` = `true` marked **UNVERIFIED — Station J confirms;
   flip here only**, toggle button **A**, provenance "team lead 2026-09-26 (port A; polarity unknown)". Its safe
   value for a library-forced safe state (COMP's end-of-run guard, later) = the CLOSED command, stated.
   Compile-time checks: every binding button unique across the pivots and the claw; none equal to
   `hal::ControllerButton::Y` (the new re-zero button — REPLACE the earlier "not A" assertion and its comment).
2. **Tests:** the table's claw entry exactly; the button-uniqueness and not-Y checks' mutations RED (put the claw
   on R1; put a pivot on Y).
3. **Station J** gains the claw: at boot the claw CLOSES (if it opens instead, `kClawOpenCommand` is backwards —
   write it down, it is flipped in the table); press A ⇒ opens, press A again ⇒ closes; holding A does nothing
   more; unplug the controller while open ⇒ it stays open; the disable check (the field/Timed Run disable, when
   available) ⇒ the claw keeps its state; one wire or two into the brain (look). Teleop's claw quotes stay
   `<<FROM TELEOP AT MERGE>>`.
4. HA: use ONLY **HA-159** (robot two's claw solenoid is on 3-wire port A; single/double and polarity UNKNOWN —
   reported by the team lead 2026-09-26).

## 4. OUT
`src/teleop_program.*`, `src/boot_source.*`, `src/main.cpp` (the coordinator wires Teleop and moves the re-zero
button to Y at merge); `hal/pros/motor.hpp` and the shim's brake behaviour (DEFECTS2); build-order, roadmap,
handoff, the COMP brief. Any hardware claim.
