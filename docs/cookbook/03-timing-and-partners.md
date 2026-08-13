# 3 — Timing and teammates

Autonomous is fifteen seconds long and there is another robot on your alliance. These recipes are
about both facts.

Listings are compiled and run in
[`test/cookbook_examples_test.cpp`](../../test/cookbook_examples_test.cpp), cases `cookbook-03a`
and `cookbook-03b`.

---

## Wait for your alliance partner

**Use this when:** your path crosses your partner's and one of you has to go second.

```cpp
// 1. A fixed beat: sit still, motors idle, while your partner clears out.
const double beforePause = c.rig.h.clock().now().value();
r.pause(750_ms);
```

**Why `pause` and not `hold`.** They sound alike and do opposite things:

- `pause(750_ms)` leaves the drive exactly as the last step left it — after a settled motion,
  stopped and unpowered. Time passes; the robot does nothing.
- `hold(750_ms)` *actively drives* to stay where it is, fighting anything that pushes it. Use it
  when a defender is about to lean on you; do not use it to let time pass.

**Watch out for:** typing the duration. `pause(750)` does not compile — durations are typed, so
"750, meaning milliseconds" cannot silently become 750 seconds of a fifteen-second match. Write
`750_ms` or `0.75_s`.

---

## Wait for a condition, with a deadline

**Use this when:** the next step is meaningless until something is true, and continuing without
it would be acting on a state the field never reached.

```cpp
// 2. A condition, with a deadline. If the deadline passes first the chain
//    STOPS — waitFor() means "the next step assumes this happened".
r.waitFor([&lane] { return lane.clear(); }, 2_s, "partner-clear");
```

The condition is yours. shulib knows nothing about your partner, so in the compiled example it is
a stand-in struct:

```cpp
struct LaneSensor {
    int polls = 0;
    bool clear() { return ++polls > 30; }
};
```

On a real robot that becomes a distance sensor reading, a line sensor, or a button your driver
presses.

**Why the deadline is required.** An unbounded wait is a hang in a costume. Fifteen seconds is
the whole autonomous period; a wait with no deadline can consume all of it and produce nothing.

**Why timing out *stops the chain*.** `waitFor` means "the step after this one assumes the
condition holds". If the deadline passes with the condition still false, running that next step
would be acting out a plan against a world that never arrived. If the timeout is a legitimate
strategy branch rather than a failure, you want the next recipe instead.

---

## Wait, but go anyway

**Use this when:** the thing you are waiting for is nice to have, not required — "give my partner
half a second, then go regardless".

```cpp
// 3. "Wait, but go anyway": that is NOT waitFor. Drop one tier inside a
//    then() — waitUntil returns a verdict you deliberately discard.
const double beforeGiveUp = c.rig.h.clock().now().value();
r.then([&c] { (void)c.chassis.waitUntil([] { return false; }, 400_ms); },
       "partner-or-not");
```

**Why it is written this way.** `waitUntil` is the full API's wait: it returns whether the
condition became true, and it never stops anything. Wrapping it in a `void` action makes it a
chain step that always succeeds. The `(void)` cast is not decoration — `waitUntil`'s result is
marked must-use precisely so you cannot ignore it by accident, and this is the one place you are
ignoring it on purpose.

**Watch out for:** reaching for this when you actually meant `waitFor`. "Go anyway" is a real
strategy and a real bug, depending on what the next step assumes. Write down which one you meant
in the step name — the log will show it.

---

## Fit the match window

**Use this when:** your routine is longer than the autonomous period allows, or might become so
when one leg goes slowly.

**The problem, stated plainly: a `Routine` has no whole-chain deadline.** Every step has its own
timeout, and they are independent. If an early leg burns six seconds fighting a defender, every
later leg still gets its full budget, and the routine can run well past the buzzer. The library
cannot fix this for you, because "what to drop when time is short" is strategy.

What you can do is read the clock between phases and keep the parking leg out of the chain that
might stop:

```cpp
RoutineResult budgetedAuton(Chassis& chassis, Intake& intake, Time budget,
                            Time secondGoalCost) {
    const Time started = clockNow(chassis);
    Routine r{chassis, "budgeted"};
    r.startAt(Pose2d{-48_in, -24_in, 0_deg})
        .driveTo(-24_in, -24_in, {.timeout = 8_s, .maxLinearSpeed = Velocity{8.0}})
        .then([&intake] { intake.release(); }, "score-1");

    // The optional goal runs only if there is time for it AND for the park.
    const double spent = (clockNow(chassis) - started).value();
    if (r.ok() && spent + secondGoalCost.value() < budget.value()) {
        r.driveTo(-24_in, 24_in, {.timeout = 6_s})
            .then([&intake] { intake.release(); }, "score-2");
    }

    // Parking is unconditional, so it lives outside the chain that might stop.
    const RoutineResult verdict = r.result();
    Routine park{chassis, "budgeted/park"};
    park.moveTo(Pose2d{-48_in, -48_in, 0_deg}, {.timeout = 6_s})
        .brake({.timeout = 1.5_s});
    return verdict;
}
```

Reading the clock needs one helper, and it reaches past the recipe layer to get it:

```cpp
Time clockNow(Chassis& chassis) { return chassis.deps().ctx->clock().now(); }
```

**Why it is written this way.**

- `secondGoalCost` is a **parameter**, not a constant baked into the library or the recipe. How
  long your second goal takes is something your team measures on your robot. A number invented
  here would be a guess wearing the clothes of a measurement.
- The **park chain is separate**. A stopped chain skips everything after the stop, so anything
  that must happen regardless — parking, retracting a mechanism, ending in a legal position —
  cannot be the last step of the chain that might stop.
- The comparison uses `.value()` on both sides. Both are typed times; comparing the underlying
  seconds is explicit about the fact that this is arithmetic on durations.

**The honest note.** `clockNow` reaches through `deps()`, which is the advanced seam
([Chapter 10](../guide/10-the-api.md)) — it is documented, supported, and stable, but it is one
tier below where a recipe should have to go for something this ordinary. This is a known gap in
the recipe layer, not a clever trick, and it is written up as such in the library's development
record. If you are reading this and a `Routine`-level deadline exists by now, prefer it.

**Watch out for:** trusting per-step timeouts as a budget. They are worst-case bounds on
individual motions, not a plan. Six steps with 5-second timeouts is a thirty-second routine in
the worst case, in a fifteen-second period.

---

*Next: [4 — Drivetrains](04-drivetrains.md).*
