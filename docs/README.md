# shulib documentation

shulib is a holonomic-native autonomous library for VEX U robots, built by the Seton Hall
University robotics team. This is everything written about it.

> **Honest status.** shulib has **never driven a robot.** It has been booted on a V5 brain
> (2026-08-12) and constructs its whole object graph there, but every motor and sensor it talks
> to is still a fake — the adapters that would reach real hardware are unwritten. Everything
> here is verified against a host simulator with deliberately hostile sensor models: the logic
> is proven, the physical constants are still estimates. [What it cannot do yet](guide/14-what-it-cannot-do-yet.md)
> is specific about which is which, and the [hardware assumptions register](hardware-assumptions.md)
> lists every number waiting on a measurement.

---

## Start here

**New to this?** Read the [user guide](guide/README.md) in order. It assumes no robotics
background and defines every term at first use — the first four chapters explain the field,
coordinates, odometry, and drivetrains before any code appears.

**Writing a routine right now?** Go to the [cookbook](cookbook/README.md). It is meant to be read
out of order, mid-task: find the thing you are trying to do, take the recipe, adapt it.

**Need the exact signature?** The [API reference](api/README.md) is generated from the headers, so
it cannot disagree with the code.

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
- **Every public member is documented**, or the build fails naming the member.
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
