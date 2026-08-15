<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/controller_display.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `controller_display.hpp`

ControllerFaultDisplay — the D-4 controller-screen content.

This header declares **1** type (2 members).

Extracted from [`include/shulib/diag/controller_display.hpp`](../../include/shulib/diag/controller_display.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ControllerFaultDisplay`](#class-controllerfaultdisplay)
  - [`ControllerFaultDisplay`](#controllerfaultdisplay-controllerfaultdisplay)
  - [`update`](#controllerfaultdisplay-update)

<a id="class-controllerfaultdisplay"></a>

## `class ControllerFaultDisplay`

```cpp
class ControllerFaultDisplay
```

The three rows the V5 controller's LCD shows when a run stops: a one-word state plus the run clock, the FIRST latched fault BY NAME, then battery and total fault count. Built for the student standing at the field with no laptop and a robot that just stopped — it converts "it died" into a fault name someone can act on. update() DIFFS: only rows whose text changed reach the device, because V5 text writes are slow and firmware-rate-limited, and the clock and battery quantize (0.1 s, 0.1 V) so jitter alone cannot force a repaint. The clock still ticks visibly, on purpose — a frozen screen and a crashed program must not look identical. Reads the latch and battery it is given; owns nothing, raises nothing, never throws.

*class, declared at [`include/shulib/diag/controller_display.hpp:110`](../../include/shulib/diag/controller_display.hpp#L110).*

<a id="controllerfaultdisplay-controllerfaultdisplay"></a>

### `ControllerFaultDisplay::ControllerFaultDisplay`

```cpp
ControllerFaultDisplay(hal::ILineDisplay& display, const FaultLatch& faults, const hal::IBattery& battery) noexcept
```

All three must outlive the display.

*function, declared at [`include/shulib/diag/controller_display.hpp:113`](../../include/shulib/diag/controller_display.hpp#L113).*

<a id="controllerfaultdisplay-update"></a>

### `ControllerFaultDisplay::update`

```cpp
void update(units::Time now)
```

Refresh the screen from current state; call at any convenient cadence (every loop tick is fine — unchanged rows cost no device writes).

*function, declared at [`include/shulib/diag/controller_display.hpp:119`](../../include/shulib/diag/controller_display.hpp#L119).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 41 lines</summary>

```text

 ControllerFaultDisplay — the D-4 controller-screen content (WS13, chunk C5).

 The use case (diagnostics-plan D-4): a student at the field, no laptop, robot
 stopped. The V5 controller's three-row LCD can say WHY — the latched first
 fault and a one-word state — which converts "it just stopped" into a fault
 name someone can act on. Disproportionately useful on competition day; nearly
 free to build.

 This class is the CONTENT side of the seam split (brief constraint 4): it
 renders rows and pushes them through hal::ILineDisplay; the PROS controller
 adapter is R1's glue. Content (each row pinned by test):

     row 0:  "OK    t   12.3s"      one-word state + run clock
             "FAULT t   12.3s"      …the word flips when anything has latched
     row 1:  "flt none"             …or the FIRST fault: "flt ODO_STUCK"
     row 2:  "batt 12.4V n 0"      battery + total fault count

 Row 1's budget for a fault name is kCols - 4 ("flt ") = 15 characters, and the
 longest spelling does NOT fit: MECHANISM_STALLED is 17, so it renders as
 "flt MECHANISM_STALL" — the seam truncates rather than wraps, which is defined
 behaviour, not a bug. This banner used to claim the longest were the two 15-char
 codes and that the fit was "checked by static math here". There was no such
 check, and the arithmetic went stale the day chunk F1 appended MECHANISM_STALLED.
 The static math now EXISTS (kNameBudget below) and asserts the property that
 actually matters: every fault name must stay DISTINGUISHABLE inside the budget,
 so a truncated row still names exactly one code. A future code that collides in
 its first 15 characters is a compile error here, at the line that renders it.

 ── The write discipline (why update() diffs) ──────────────────────────────────────
 V5 controller text writes are SLOW and firmware-rate-limited (~50 ms per line
 class of slow). A per-tick repaint would both starve the display and waste the
 loop budget it shares. So update() rewrites ONLY rows whose content CHANGED —
 steady state costs three string compares and zero device writes; the battery
 row quantizes to 0.1 V so millivolt jitter cannot repaint it. The run clock
 quantizes to 0.1 s: ~10 row-0 writes/second, inside any sane device budget,
 and the seconds display is exactly why a student trusts the screen is LIVE
 (a frozen screen and a crashed program must not look identical).

 Reads the FaultLatch and battery it is given; owns nothing; raises nothing;
 never throws. Single-task by contract, like the rest of diag/.
```

</details>
