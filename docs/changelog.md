# Changelog

> **What this is.** Every user-visible change to shulib, newest first, one section per API
> version — each entry says **what changed**, **whether it is breaking or additive**, and
> **what you must do about it**. The version policy itself lives in
> [`include/shulib/version.hpp`](https://github.com/SHU-ROBOTICS/shulib/blob/main/include/shulib/version.hpp):
> the major number moves only for a breaking change to a frozen surface, always with a
> migration note; the minor number moves for additive growth. The
> [Freeze Register](roadmap.md#freeze-register) says which surfaces are frozen at all.
>
> This file starts existing partway through the 2.1 line — the project ran to API 2.1 with
> the version history recorded only in `version.hpp`'s comments, which an outside team on
> 2.0 had no reason to read. Entries below the line marked *(reconstructed)* were written
> after the fact from those comments and the project records; everything above it was
> written when the change landed.

## API 2.1

### 2026-08-15 — 59 defect fixes, mostly additive, with five surface changes to know about

An audit chunk (DEFECTS1) triaged the 84 API defects the documentation pass had reported and
left in place, and fixed 59 of them. **Most are invisible to a caller doing nothing wrong** —
added preconditions, corrected counters, corrected comments. Five change something a compiler
or a caller can see, and one of those is breaking under the version policy.

**BREAKING (unfrozen surfaces, no consumer in the tree):**

- **`control::TrapezoidProfile::isDone()` is no longer `noexcept`**, and both it and `sample()`
  now REJECT a non-finite `t`. `sample(NaN)` used to return a *partially* finite state —
  position and velocity NaN, acceleration a perfectly finite `-aMax` — so a caller screening
  only `acceleration` passed it and forwarded a plausible-looking down-ramp; `isDone(NaN)`
  returned false, so a follower loop terminating on it spun forever. The `noexcept` drop is
  required because the precondition handler throws and a `noexcept` frame would turn a caller
  bug into `std::terminate`. **Migration:** screen your clock, or catch `PreconditionError`.
  This class has no consumer in the library but its own test.
- **`hal::IMechanism` is now non-copyable and non-movable.** It holds the claim token as value
  state, so a copied mechanism arrived already `claimed()`, with `claimant()` pointing at an
  operation registered against the original. **Migration:** construct a mechanism where it lives
  and hand out `IMechanism&` / `IMechanism*` — which is what the header already told you to do.
- **`localization::EkfFusion::state()` and `covariance()` are no longer `noexcept`**; both now
  bounds-check. They are observability accessors, and an out-of-range index was silent UB.

**ADDITIVE, but you may notice:**

- **`diag::MotionOutcome::Unset = 5`**, appended and value-pinned, is the new default for
  `MotionResult::outcome`. An unpopulated result line used to render `✓ SETTLED` — the one
  value meaning success — for a motion that never happened; it now renders `✗ UNSET`.
- **`ProsDigitalOut(smartPort, adiPort)` with the boot state forgotten no longer compiles.**
  It used to select the brain-ADI constructor with `initialState = (bool)2 = true` and fire a
  solenoid HIGH at boot on the wrong port. **Migration:** state the boot level, which the
  required third argument always intended.

**Behaviour you should know changed, without a signature moving:** a `MotionScheduler` destroyed
with a motion still armed now forces the drive to its safe state instead of leaving the motors
at their last command; `MotionConfig` and `Watchdog` reject an *infinite* timeout (`> 0.0` is
satisfied by infinity, and an infinite watchdog can never expire); `Localizer` rejects a `minDt`
that is non-positive or not below `maxDt`; and a stale, finished mechanism operation can no
longer `cancel()` a mechanism a different live operation now owns.

Full triage with evidence for every item, including the 15 findings that did **not** hold and
the 6 that need a decision before they can be fixed, is in the development record.


### 2026-08-14 — the API reference now covers the whole public API — no API change

No library code changed: not a signature, not a default, not a behaviour. This entry is here
because what the documentation *promises* changed, and because one of those promises is
enforced by a build gate you will meet if you contribute.

**The reference went from 2 documented types to every public entity in every shipped header** —
117 pages covering **1,625** types, members, nested types, free functions, namespace-scope
constants and type aliases, plus an [A–Z index](api/all-entities.md) of all of them. The
generator's target list is now a glob over `include/shulib/`, so a header added later is covered
the moment it exists rather than when someone remembers to list it.

**The coverage gate grew with it.** A public entity anywhere under `include/shulib/` with no
`///` documentation now fails the build, naming the entity, its file and its line. If you are
adding to a shipped header, that is the one new obligation: say what it is *for*.

**Being documented does not freeze anything.** The distinction matters and is easy to
mis-read: a change to a *frozen* signature (`Chassis` = F6, `Routine` = F10) fails a
compile-time pin that names the register row, and costs a major version bump plus a migration
note. A change to a documented-but-unfrozen seam costs one comment edit and a regeneration.
Freeze Register rows F11–F14 previously said those seams were deliberately ungated for exactly
this fear; all four are **amended** rather than silently overridden, and their freeze triggers
are unchanged.

**Three locked contracts were not in the reference at all**, and nobody could have known from
reading it: the generator was structurally blind to types with a base-class list, enums with an
explicit underlying type, and everything declared at namespace scope — so the coordinate frame
(`math::Frame` and its two conversions), the accuracy targets (`spec/accuracy.hpp`) and the
units vocabulary (`units::Length`, `Time`, `Voltage`, …) produced no output whatsoever. They are
there now.

**The site is easier to move around**, which matters more at 117 pages than at 2: the top-level
areas are tabs, subsystem groups collapse, each section's overview is its own landing page, and
each header's design commentary folds when it runs long. The nav is generated from the same list
as the pages and byte-checked, because a page missing from the nav was measured to publish
*unreachable*, with exit code 0 and an INFO line.

**Breaking:** nothing. **What you must do:** nothing, unless you contribute to a header — then
write a `///` on anything public you add.

### 2026-08-14 — the mechanism-sensor adapters land (`hal/pros/`, second half) — additive

The other half of the hardware binding: PROS-backed adapters for the mechanism seams —
`ProsDistance`, `ProsOptical`, `ProsDigitalOut`, `ProsDigitalIn`, and `ProsBlockSink` (the
SD-card device behind the blackbox). Each applies its unit conversion exactly once at the
edge, through two new pure conversion headers (`distance_conversion.hpp`,
`optical_conversion.hpp`).

New HAL seam, **not frozen**: `IDigitalIn` (one member — the raw level of a digital input
line; no debouncing, no edge state, no validity channel, each by documented ruling) with
`hal::fake::FakeDigitalIn`. It is an additive sibling outside the F4 freeze, exactly as
`IDigitalOut` and `IController` were; the Freeze Register records the non-freeze out loud
(row F14). It was built ahead of its consumer — the lift-homing switch-or-stall question is
still open — and the register row says that too.

Three behaviours worth knowing before you use these:

- **The distance sensor's "no object" is 9999 mm, in-band.** PROS reports an empty field of
  view as a plain reading that converts to a plausible-looking 393.66 inches — not an error.
  `ProsDistance` maps it to `confidence() == 0.0` (the seam's documented "no usable return"
  channel) and keeps `distance()` finite and far. Threshold `confidence()`, always — the FAQ
  entry "Why does my distance sensor read 393 inches?" walks it.
- **Constructing a `ProsDigitalOut` PHYSICALLY DRIVES the line.** PROS actuates the port at
  construction and defaults it LOW; on a pneumatic that moves the cylinder at boot. The
  adapter refuses the default — the initial state is a required constructor argument, and it
  must agree with the owning `PneumaticMechanism`'s declared safe state (a test pins the
  pattern).
- **A missing SD card does not stop the robot.** `ProsBlockSink` constructs successfully with
  no card, refuses every write (`write()` returns false from the first call — the blackbox's
  drop-and-count design absorbs it), and reports the fact once through `isOpen()`.

**Breaking:** nothing. Every frozen surface (F3/F4/F6/F10) is untouched; the adapters are new
files implementing existing interfaces, and `IDigitalIn` is additive.

**What you must do:** nothing, unless a mechanism uses these sensors — then construct the
`Pros*` adapters and read the seam headers' design notes (each names which PROS call it
binds, which conversion it applies, and which Hardware-Assumptions entries its beliefs rest
on — HA-113 onward, none yet measured on hardware). Honest scope, unchanged from R1a: these
are host-tested against the programmable PROS stand-in, and **the library has still never
driven a robot.**

### 2026-08-13 — first hardware validation of the adapters — no API change

No code changed. Recorded here because it changes what the library's claims are worth.

The `hal/pros/` adapters were run against a physical V5 brain and a real robot for the first time.
Every unit conversion they perform was checked against a turning wheel:

| Conversion | Expected | Measured |
|---|---|---|
| motor degrees → radians | 57.2958 | 57.296 |
| motor RPM → rad/s | 9.5493 | 9.549 |
| motor mA → amps | 1000 | 1000.0 |
| battery raw → volts | — | 13039 → 13.04 V |
| battery capacity → [0,1] | — | 91.0 → 0.91 |
| `micros()` per 1000 ms | 1000000 | 999784 |

Seven previously-guessed hardware assumptions are now measured observations rather than reasoning.
The battery unit was the weakest of them — PROS's vendored headers document no unit for
`battery_get_voltage()` at all.

**What this does not mean:** the library still has never driven a robot. Nothing closed a control
loop, nothing followed a path, and no wheel turned under motion control. These measurements
establish that the platform layer reads and commands real hardware correctly, and nothing more.

### 2026-08-13 — the PROS hardware adapters land (`hal/pros/`) — additive

The library's first hardware binding: header-only adapters under
`include/shulib/hal/pros/` implement the frozen HAL interfaces over real V5 devices —
`ProsClock`, `ProsMotor`, `ProsRotation`, `ProsImu`, `ProsGps`, `ProsBattery`,
`ProsCharSink`, `ProsLineDisplay`, `ProsController`, plus `ProsTickPacer` (the real
10 ms tick over the PROS scheduler). Each adapter applies its unit conversion exactly
once, at the edge, through new pure conversion headers (`motor_conversion.hpp`,
`rotation_conversion.hpp`, `controller_conversion.hpp` — joining the existing IMU and
GPS ones).

New HAL seam, **not frozen**: `IController` (normalized [-1, 1] axes, button levels, a
positive `isConnected()` signal, master/partner support) with `ButtonEdge` for
per-consumer press detection and `hal::fake::FakeController` for tests. The Freeze
Register records the non-freeze out loud (row F13).

**Breaking:** nothing. Every frozen surface (F3/F4/F6/F10) is untouched; the adapters are
new files implementing existing interfaces.

**What you must do:** nothing, unless you want your robot code on real hardware — then
construct the `Pros*` adapters instead of fakes (the shipped `src/main.cpp` is the worked
example, and guide chapter 7 walks it). Honest scope, stated plainly: these adapters are
host-tested against a programmable stand-in for PROS, and **the library has still never
driven a robot** — the beliefs behind every conversion are catalogued in the
[Hardware Assumptions Register](hardware-assumptions.md) (HA-94 onward) and get their
first reality check on a bench, not in a test suite.

### 2026-08-13 — a guaranteed end of run *(reconstructed)* — additive

`sequence/run_guard.hpp` is new: `RunGuard`, plus `RunGuardConfig`, `RunGuardReport` and
`GuardedWaitResult`. It is an `ITickPacer` **decorator** — construct it around the real pacer
and hand it to the chassis — and it is completely **inert until `run()`**.

At the instant you name, it cuts the active motion with zero latency, then **refuses every
later motion**. The refusal is a *latch*, not a pause: cancel-only expiry was measured inert
and marginally counterproductive, so once the deadline passes nothing new starts. Cancel-all
runs strictly before your end action, and an unconditional `hardStopAt` floor fires regardless.

**The library refuses to know your strategy.** No field coordinate, no park pose, no default
lead time, no default match length — both instants and the end action are yours. A team that
ends somewhere else, or does not park at all, is not fighting the library.

**Four limits, stated because they bind:** it proves a *scheduling* property, not a timing
margin on a real brain (loop rate and PROS call latency are still invented register entries);
**nothing preempts pure user code** — there are no background tasks anywhere in this library, so
a `while (true)` loop that never lets a call return keeps the CPU and your end action waits;
the frozen `pause`/`waitFor`/`waitUntil` surfaces cannot see the deadline and pay their full
remainder (a tested limit, not a hidden one); and **"zero travel after the deadline" is a
simulator result** — the host plant is memoryless, so on a robot with mass the honest claim is
*"no new commanded motion"*. Budget lead time for a robot that is still moving when the command
stops.

**Breaking:** nothing. New header, new types; no frozen surface touched.

**What you must do:** nothing, unless you want the guarantee — then wrap your pacer. Note that
the routine-level watchdog (diagnostics item D-8) is the same primitive under a second policy.

### 2026-08-13 — mechanism seam growth *(reconstructed from `version.hpp`)* — additive, 2.0 → 2.1

The mechanism layer grew through the documented additive paths: `RoutineStopCause`
gained an appended `MechanismFailed` enumerator, `Routine::then()` accepts a fourth
return type (`manipulation::MechanismOutcome`), and `FaultCode` appended
`MechanismStalled`. No frozen member changed shape.

**What you must do:** nothing. If you `switch` exhaustively over `RoutineStopCause` or
`FaultCode`, add the new cases.

## API 2.0

> The four entries below all landed **after** the 2.0 freeze and **before** F1 opened 2.1. None
> of them moved the version number, and that is the policy working rather than an oversight:
> `version.hpp` tracks the growth of *frozen* surfaces, and these four added entirely new
> headers and types outside `Chassis` and `Routine`. Nothing previously compiling changed
> meaning. They are reconstructed here because they are user-visible surface an outside team
> would otherwise meet only by reading the source.

### 2026-08-13 — a Kalman tier, which is **not** the default *(reconstructed)* — additive

`localization/ekf_fusion.hpp` adds `EkfFusion` and `EkfFusionConfig` — a 5-state SE(2) extended
Kalman filter behind the unchanged `IFusionPolicy` seam. It can do three things the simpler
filter cannot: weigh two disagreeing correctors against each other by their stated accuracy
(matched against the inverse-variance weighted mean to better than 0.2%), recover from a
displacement the fixed gate makes permanent, and state its own uncertainty as a number.

**`ComplementaryFusion` remains the shipped default, and the measurement is why.** Raced over
eight seeded 60-second runs the EKF finished *less* accurate — 0.351″ vs 0.225″ mean final
error, losing on 7 of 8 seeds. It lost the comparison its own definition-of-done asked it to
win, and that result was kept rather than tuned away. In this simulation dead-reckoning is
already sub-inch and the modelled GPS is noisier than the drift it corrects, so the right move
is mostly to ignore the sensor — which a blunt fixed gain does slightly harder. **Anyone
quoting this as an accuracy improvement is quoting it wrong.** Every noise parameter in it
(HA-83…HA-91) is invented, so real sensor characterization could flip the result either way.

**Breaking:** nothing. **What you must do:** nothing — you get the complementary filter unless
you ask for the EKF.

### 2026-08-13 — absolute heading from AprilTags *(reconstructed)* — additive

`localization/apriltag_corrector.hpp` adds `AprilTagCorrector` and `AprilTagCorrectorConfig`;
`localization/tag_map.hpp` adds `TagMap`, `TagPlacement` and `TagProvenance`. This is the only
source of **absolute heading** in the library — a tag whose field position is known tells the
robot which way it is actually pointing, and it proposes with `providesHeading = true`.

**shulib ships no tag map, deliberately.** Nobody on the project can cite a table of AprilTag
field poses, so inventing one would be a confidently wrong answer rather than a missing one.
`TagMap::add()` refuses a placement whose provenance is `Unspecified` — `Invented` is a
perfectly legitimate answer, *unstated* is not — and an unknown tag id makes the corrector
decline with `RejectedNoTagMapEntry` rather than guess.

**Two things worth knowing before you trust it:** a map that is two inches off produces a
corrector that is *confidently* two inches wrong, and unlike noise that error does not average
out (HA-68). And a **reversed corner winding is catastrophic and silent** — it mirrors the tag,
puts the recovered heading 180° out, and leaves the solver's own reprojection error reading
machine zero. No software self-check can see it; only a physical tag can (HA-69).

**Breaking:** nothing. **What you must do:** nothing, unless you add a camera — then you must
supply the tag map, with provenance.

### 2026-08-12 — GPS position correction *(reconstructed)* — additive

`localization/gps_corrector.hpp` adds `GpsCorrector` and `GpsCorrectorConfig`, the first real
`ICorrector`: it bounds position drift while the field strip is in view. It corrects **position
only** — `providesHeading` stays false, so the IMU keeps owning heading.

The accuracy claim is sized to the evidence rather than rounded up: over eight seeded 60-second
runs the corrected estimate was better on 7 of 8 on final error and 6 of 8 on worst-case. A
shorter scenario that passed 8 of 8 was found and **deliberately not adopted**, as scenario
shopping.

**Driving Skills has no GPS strip**, so `hasFix()` is false for that entire event and the
estimator dead-reckons — a normal operating state, not a fault. The FAQ entry
"Why does `hasFix()` go false in Driving Skills?" is the full version.

**Breaking:** nothing. **What you must do:** nothing, unless you have a GPS.

### 2026-08-12 — the SD-card blackbox *(reconstructed)* — additive

`diag/sd_sink.hpp` adds `SdSink` (an `ITelemetrySink` writing through the `IBlockSink` device
seam) with `SdSinkConfig`, `SdSinkStorage` and `SdSinkBuffers`; `diag/blackbox_format.hpp` adds
the versioned, session-stamped, fixed-width binary format; `diag/blackbox_reader.hpp` adds
`BlackboxReader` and `ReadStatus`. **A run becomes recoverable after the fact, with no laptop.**

Two design points that are the whole value: the **decoder ships in the same chunk as the
encoder** — a format nothing can read is not a record — and it is held to the format by
byte-exact goldens rather than by agreeing with the encoder, so the two cannot drift into
agreeing on something wrong. And the default posture is a **RAM flight recorder** that costs the
card nothing until a fault fires, at which point it writes the triage block **first** and the
preceding ticks after it — in that order, because the fault may be the brownout that cuts the
write short.

**Breaking:** nothing. **What you must do:** nothing. If you want it, give `SdSink` a block
device; the V5 adapter for that arrived later (see the 2026-08-14 entry) and a missing card is
handled rather than fatal.

### 2026-08-12 — the v2 facade freezes *(reconstructed)* — the 2.0 baseline

The public `Chassis` API froze (Freeze Register row F6), followed the same day by the
`Routine` recipe layer (row F10). From this point, routines written against either tier
do not need rewriting: signatures and documented behaviour change only with a major
version bump and a migration note, enforced by compile-time signature pins that fail the
build if a frozen member drifts.

**Breaking, relative to everything before it:** shulib v2 is a ground-up rebuild; the
legacy v1 tree was deleted when `src/main.cpp` was rewired onto the v2 core. There is no
v1→v2 migration path — v1 was never released beyond the team.

**What you must do:** new users start at the [guide](guide/README.md); the
[cookbook](cookbook/README.md) and the [API reference](api/README.md) are the day-to-day
documents.
