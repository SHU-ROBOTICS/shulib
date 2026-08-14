# shulib documentation

shulib is a holonomic-native autonomous library for VEX U robots, built by the Seton Hall
University robotics team. This is everything written about it.

> **Honest status.** shulib has **never driven a robot.** No control loop has ever closed on
> hardware, no wheel has ever turned under the library's own steering, and no path has ever been
> followed. What *has* happened on real hardware is narrower and worth stating exactly: the code
> booted on a V5 brain (2026-08-12) and built its whole object graph there against fakes, and
> then on 2026-08-13 the hardware adapters commanded eight real motors and read real sensors,
> proving their unit conversions correct against a turning wheel — on one robot, once.
> Everything above that seam — motion, accuracy, control — is verified only against a host
> simulator with deliberately hostile sensor models: the logic is proven, and nearly every
> physical constant is still an estimate.
> [What it cannot do yet](guide/14-what-it-cannot-do-yet.md) is specific about which is which,
> and the [hardware assumptions register](hardware-assumptions.md) lists every number still
> waiting on a measurement.

---

## What this is, in one minute

A robot in an autonomous period has to answer three questions continuously, and shulib is the
library that answers them: **where am I** (odometry fused with an inertial sensor, corrected by
a GPS and by AprilTags), **how do I get over there** (motion that translates and rotates at the
same time and independently — our robots slide sideways, so the library is built around that
rather than treating it as an afterthought), and **what just happened** (structured diagnostics
on every tick, plus a blackbox that survives without a laptop).

Most VEX teams use LemLib, which drives only tank robots. Ours are X-drive and H-drive, so
LemLib cannot drive them at all. That is why this exists.

The honest shape of the project: **the thinking half is done and heavily tested** — three
drivetrain types, sensor fusion, motion, sequencing, a guaranteed safe stop, and two ways to
write a routine. **The hardware half is new** — the adapters exist and have been checked once at
a bench. **The "usable without writing C++" promise is not built yet**; routines are C++ today.

---

## Start here

**New to this?** Read the [user guide](guide/README.md) in order. It assumes no robotics
background and defines every term at first use — the first four chapters explain the field,
coordinates, odometry, and drivetrains before any code appears.

**Writing a routine right now?** Go to the [cookbook](cookbook/README.md). It is meant to be read
out of order, mid-task: find the thing you are trying to do, take the recipe, adapt it.

**Need the exact signature?** The [API reference](api/README.md) is generated from the headers, so
it cannot disagree with the code.

**Hit something surprising?** The [FAQ](faq.md) covers the behaviour the reference cannot carry —
why a distance sensor reads 393 inches, why a pneumatic fired at boot, why `hasFix()` is false all
match in Driving Skills. The [changelog](changelog.md) says what changed and whether it breaks you.

---

## The three documents, and why there are three

They answer different questions, and each links to the others rather than repeating them.

| Document | Answers | Read it |
|---|---|---|
| [**User guide**](guide/README.md) | *How does any of this work?* | In order, once |
| [**Cookbook**](cookbook/README.md) | *How do I do the specific thing I need?* | Out of order, repeatedly |
| [**API reference**](api/README.md) | *What exactly exists, and what is its type?* | As a lookup |

The guide teaches; the reference enumerates. Where they overlap, the guide explains *why* and the
reference states *what* — if you find them disagreeing about a fact, the reference is right, because
it is generated from the source.

---

## How these stay true

Documentation that drifts is worse than no documentation, because people trust it. Four properties
are enforced by the build itself, not by anyone remembering:

- **Every code example compiles and runs.** Examples live in the test suite and the prose quotes
  them verbatim; a build fails if a listing drifts from the code that compiles it.
- **The API reference is regenerated from the headers**, and the build fails if the committed copy
  is out of date.
- **Every public member of the two frozen surfaces is documented**, or the build fails naming the
  member. (The coverage gate parses `Chassis` and `Routine` — the surfaces the reference is
  generated from. Seams that are deliberately not frozen yet are deliberately not gated yet
  either; the Freeze Register says which is which.)
- **The frozen surfaces are pinned at compile time** — a changed signature on `Chassis` or
  `Routine` fails the build with the register row it violates.

---

## Reference material

- [**Roadmap**](roadmap.md) — every milestone and task, the Freeze Register, and an honest
  "you are here"
- [**Architecture and design decisions**](shulib-v2-master-plan.md) — why the library is shaped
  the way it is, including the alternatives that were rejected
- [**Hardware assumptions register**](hardware-assumptions.md) — every physical constant that is
  currently an estimate, what it would take to measure it, and what breaks if it is wrong
- [**Diagnostics plan**](diagnostics-plan.md) — the telemetry and observability contract
- [**Legacy command vocabulary**](legacy-command-vocabulary.md) — what the previous codebase's
  routine format meant, preserved for the eventual importer

---

*Source: [github.com/SHU-ROBOTICS/shulib](https://github.com/SHU-ROBOTICS/shulib)*
