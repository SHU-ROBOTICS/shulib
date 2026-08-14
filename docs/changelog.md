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

### 2026-08-13 — mechanism seam growth *(reconstructed from `version.hpp`)* — additive, 2.0 → 2.1

The mechanism layer grew through the documented additive paths: `RoutineStopCause`
gained an appended `MechanismFailed` enumerator, `Routine::then()` accepts a fourth
return type (`manipulation::MechanismOutcome`), and `FaultCode` appended
`MechanismStalled`. No frozen member changed shape.

**What you must do:** nothing. If you `switch` exhaustively over `RoutineStopCause` or
`FaultCode`, add the new cases.

## API 2.0

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
