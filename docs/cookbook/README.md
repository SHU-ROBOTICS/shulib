# The shulib cookbook

> **What this is:** short, complete answers to "how do I write the routine I am trying to write
> right now." Every recipe is a real compiled program that runs against the simulator.
> **What this is not:** a course. The [user guide](../guide/README.md) teaches the ideas in order;
> this book assumes you already have a robot object and a deadline.

The guide and the cookbook are different documents for different moments. A guide is read once,
front to back, when you are learning. A cookbook is read in the middle of a task, out of order,
because you need one specific thing. If you have never written a shulib routine, start with
[Chapter 8 — your first routine](../guide/08-your-first-routine.md) and
[Chapter 9 — the recipe API](../guide/09-the-recipe-api.md); come back here when you are writing
your own.

---

## How to use a recipe

Every recipe has the same four parts:

1. **Use this when** — the situation, in one sentence. Skim these to find your recipe.
2. **The recipe** — code you can paste. It is quoted *verbatim* from
   [`test/cookbook_examples_test.cpp`](../../test/cookbook_examples_test.cpp), where it is
   compiled and run against the simulated robot on every build. If a recipe stopped working,
   that file would go red before you ever saw it.
3. **Why it is written this way** — the reasoning, so you can change it safely.
4. **Watch out for** — the mistake this recipe exists to prevent.

Recipes assume `using namespace shulib::units::literals;` and the `Chassis` and `Routine` names
in scope, exactly as the guide's chapters set up.

**Exact signatures are not repeated here.** When you need the precise type of an argument or a
return, use the [generated API reference](../api/README.md) — it is extracted from the headers,
so it cannot disagree with the code.

---

## The recipes

### [1 — Getting there](01-getting-there.md)

| Recipe | Use this when |
|---|---|
| [The skeleton](01-getting-there.md#the-skeleton) | You are starting a new routine from nothing |
| [A two-goal side run](01-getting-there.md#a-two-goal-side-run) | Your auton visits several places and does something at each |
| [A reusable scoring step](01-getting-there.md#a-reusable-scoring-step) | The same three-or-four-step sequence appears more than once |
| [A waypoint sweep](01-getting-there.md#a-waypoint-sweep) | You want one call to drive a whole path |

### [2 — When a step fails](02-when-a-step-fails.md)

| Recipe | Use this when |
|---|---|
| [Bail out and park somewhere useful](02-when-a-step-fails.md#bail-out-and-park-somewhere-useful) | A failed grab should abandon the plan, not carry on with empty jaws |
| [Attempt something and keep going](02-when-a-step-fails.md#attempt-something-and-keep-going) | One missed goal in a sweep should not end the routine |
| [Recover from a broken sweep](02-when-a-step-fails.md#recover-from-a-broken-sweep) | A trajectory stopped partway and you need to know how far it got |

### [3 — Timing and teammates](03-timing-and-partners.md)

| Recipe | Use this when |
|---|---|
| [Wait for your alliance partner](03-timing-and-partners.md#wait-for-your-alliance-partner) | Your path crosses your partner's and one of you has to go second |
| [Wait for a condition, with a deadline](03-timing-and-partners.md#wait-for-a-condition-with-a-deadline) | The next step is meaningless until something is true |
| [Wait, but go anyway](03-timing-and-partners.md#wait-but-go-anyway) | The thing you are waiting for is nice to have, not required |
| [Fit the match window](03-timing-and-partners.md#fit-the-match-window) | Your routine is longer than the autonomous period allows |

### [4 — Drivetrains](04-drivetrains.md)

| Recipe | Use this when |
|---|---|
| [A tank routine](04-drivetrains.md#a-tank-routine) | Your drivetrain cannot slide sideways |
| [Budget a sideways leg](04-drivetrains.md#budget-a-sideways-leg) | Your drivetrain strafes, but slowly (an H-drive) |

### [5 — Mixing tiers](05-mixing-tiers.md)

| Recipe | Use this when |
|---|---|
| [Call the full API mid-routine](05-mixing-tiers.md#call-the-full-api-mid-routine) | A recipe step cannot express what this one leg needs |
| [Make a direct call count as a step](05-mixing-tiers.md#make-a-direct-call-count-as-a-step) | You dropped a tier and still want the chain to stop if it fails |
| [Re-seed the estimate mid-routine](05-mixing-tiers.md#re-seed-the-estimate-mid-routine) | The robot has just squared itself against a wall |

---

## The three things every recipe assumes

**1. Steps run as you chain them.** A `Routine` is not a plan that executes later; each step
blocks until its motion ends. The routine happens in the order it reads. That is why you can put
a `printf`, a sensor read, or a whole `if` between two steps and nothing surprising happens.

**2. A failed step stops the chain and parks the robot.** Everything after it is skipped and
logged. You do not have to check anything for the robot to be safe — you check `r.ok()` when you
want to *do* something different, not to prevent a crash.

**3. Nothing here is a ceiling.** Every step is one call on the `Chassis` object underneath, and
that object is one line away (`r.chassis()`, or the reference you already have). When a recipe
cannot express something, dropping to the full API for one leg is the intended move, not a
defeat. [Chapter 10](../guide/10-the-api.md) is that API in prose.

---

## What is not in here

- **Mechanisms.** shulib has no intake, lift, or pneumatics yet. Recipes that need one use a
  small struct written by hand, and say so — that is exactly the shape a real mechanism will
  have when the library grows one ([Chapter 14](../guide/14-what-it-cannot-do-yet.md)).
- **Strategy.** Which goals to score, in which order, is your team's to decide and defend. The
  recipes use plausible field points as scaffolding; none of them is an argument about what a
  good auton does.
- **Anything untested.** If it is not in `test/cookbook_examples_test.cpp`, it is not in here.

---

*Maintained alongside the guide. The rules for keeping both true are in the guide's own
[README](../guide/README.md).*
