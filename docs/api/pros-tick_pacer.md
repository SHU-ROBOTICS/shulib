<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/tick_pacer.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `tick_pacer.hpp`

ProsTickPacer — motion::ITickPacer over pros::Task::delay_until (chunk R1a): the ONLY seam that regains control mid-motion on the robot, replacing main.cpp's V5DelayPacer (which had to hand-advance a FakeClock).

This header declares **1** type (2 members).

Extracted from [`include/shulib/hal/pros/tick_pacer.hpp`](../../include/shulib/hal/pros/tick_pacer.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ProsTickPacer`](#class-prostickpacer)
  - [`kTickMs`](#prostickpacer-ktickms)
  - [`pace`](#prostickpacer-pace)

<a id="class-prostickpacer"></a>

## `class ProsTickPacer`

```cpp
class ProsTickPacer final : public motion::ITickPacer
```

ITickPacer on the robot: blocks until the next tick boundary via pros::Task::delay_until, so the tick body's own duration is ABSORBED by the wait instead of added to it. That is the whole reason it is not pros::delay(kTickMs), which sleeps from NOW and would turn a 2 ms tick body into a 12 ms loop — 20% slow, forever, with the motion profiles integrating the error. The cadence anchors on the FIRST pace(), not at construction, so an object built long before it is used does not try to catch up the ticks it "missed" while nothing was pacing.

*class, declared at [`include/shulib/hal/pros/tick_pacer.hpp:49`](../../include/shulib/hal/pros/tick_pacer.hpp#L49).*

<a id="prostickpacer-ktickms"></a>

### `ProsTickPacer::kTickMs`

```cpp
static constexpr std::uint32_t kTickMs = 10
```

the motion tick (HA-32's 100 Hz)

*field, declared at [`include/shulib/hal/pros/tick_pacer.hpp:51`](../../include/shulib/hal/pros/tick_pacer.hpp#L51).*

<a id="prostickpacer-pace"></a>

### `ProsTickPacer::pace`

```cpp
void pace() override
```

Block until the next tick boundary (header: anchored cadence, lazy first-call anchor).

*function, declared at [`include/shulib/hal/pros/tick_pacer.hpp:55`](../../include/shulib/hal/pros/tick_pacer.hpp#L55).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 28 lines</summary>

```text

 ProsTickPacer — motion::ITickPacer over pros::Task::delay_until (chunk R1a):
 the ONLY seam that regains control mid-motion on the robot, replacing
 main.cpp's V5DelayPacer (which had to hand-advance a FakeClock).

 BINDS: pros::Task::delay_until(&prev, kTickMs) — NOT pros::delay(kTickMs).
 The difference is drift: delay(10) sleeps 10 ms from NOW, so each tick's
 processing time ADDS to the period (a 2 ms tick body makes a 12 ms loop —
 20% slow, and the motion profiles integrate that error forever).
 delay_until wakes at prev + delta and updates prev to the WAKE instant
 (vendored rtos.hpp:742-747, HA-102), so the cadence is anchored to the
 timeline, not to the work: processing time is absorbed, and the loop runs
 at the true 100 Hz the motion layer assumes (HA-32).

 FIRST CALL: prev is initialized from millis() lazily on the first pace() —
 the pacer anchors to the moment pacing STARTS, not the moment the object
 was constructed (a Robot constructed at t=0 but first paced at t=3000 must
 not "catch up" 300 phantom ticks; FreeRTOS's catch-up semantics would run
 them back-to-back and the motion layer would see 300 zero-dt ticks).

 CADENCE: kTickMs = 10 (the 100 Hz motion tick, HA-32) — the same constant
 V5DelayPacer carried; one owner, here, until a config plumbs it.

 The real IClock (ProsClock) reads real time and needs no help — the
 fake-clock advance that V5DelayPacer had to do is gone, which is exactly
 the R1 note that pacer carried in main.cpp since C7.

 HA register: HA-102, HA-32.
```

</details>
