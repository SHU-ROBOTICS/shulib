<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/control/trapezoid_profile.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `trapezoid_profile.hpp`

TrapezoidProfile — a trapezoidal motion profile.

This header declares **3** types (9 members).

Extracted from [`include/shulib/control/trapezoid_profile.hpp`](../../include/shulib/control/trapezoid_profile.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct ProfileConstraints`](#struct-profileconstraints)
  - [`maxVelocity`](#profileconstraints-maxvelocity)
  - [`maxAcceleration`](#profileconstraints-maxacceleration)
- [`struct ProfileState`](#struct-profilestate)
  - [`position`](#profilestate-position)
  - [`velocity`](#profilestate-velocity)
  - [`acceleration`](#profilestate-acceleration)
- [`class TrapezoidProfile`](#class-trapezoidprofile)
  - [`TrapezoidProfile`](#trapezoidprofile-trapezoidprofile)
  - [`sample`](#trapezoidprofile-sample)
  - [`duration`](#trapezoidprofile-duration)
  - [`isDone`](#trapezoidprofile-isdone)

<a id="struct-profileconstraints"></a>

## `struct ProfileConstraints`

```cpp
struct ProfileConstraints
```

The envelope a profile must stay inside. Bare doubles by design: the CALLER picks the distance unit and these are that unit per second and per second² — inches and radians are what a motion layer would use, though none instantiates one today (header note).

*struct, declared at [`include/shulib/control/trapezoid_profile.hpp:32`](../../include/shulib/control/trapezoid_profile.hpp#L32).*

<a id="profileconstraints-maxvelocity"></a>

### `ProfileConstraints::maxVelocity`

```cpp
double maxVelocity = 0.0
```

Cruise-speed cap; must be > 0 (the 0 default is unusable)

*field, declared at [`include/shulib/control/trapezoid_profile.hpp:33`](../../include/shulib/control/trapezoid_profile.hpp#L33).*

<a id="profileconstraints-maxacceleration"></a>

### `ProfileConstraints::maxAcceleration`

```cpp
double maxAcceleration = 0.0
```

Ramp rate, used for BOTH ramps (accel == decel); must be > 0

*field, declared at [`include/shulib/control/trapezoid_profile.hpp:34`](../../include/shulib/control/trapezoid_profile.hpp#L34).*

<a id="struct-profilestate"></a>

## `struct ProfileState`

```cpp
struct ProfileState
```

The profile's command at one instant. All three fields carry the SIGN of the move — a negative `distance` mirrors every one of them.

*struct, declared at [`include/shulib/control/trapezoid_profile.hpp:39`](../../include/shulib/control/trapezoid_profile.hpp#L39).*

<a id="profilestate-position"></a>

### `ProfileState::position`

```cpp
double position = 0.0
```

Displacement FROM THE START of the move, not a field coordinate: 0 at t <= 0 and exactly `distance` at t >= duration(). The caller adds its own origin.

*field, declared at [`include/shulib/control/trapezoid_profile.hpp:42`](../../include/shulib/control/trapezoid_profile.hpp#L42).*

<a id="profilestate-velocity"></a>

### `ProfileState::velocity`

```cpp
double velocity = 0.0
```

Signed speed; 0 at both ends, never exceeds maxVelocity

*field, declared at [`include/shulib/control/trapezoid_profile.hpp:43`](../../include/shulib/control/trapezoid_profile.hpp#L43).*

<a id="profilestate-acceleration"></a>

### `ProfileState::acceleration`

```cpp
double acceleration = 0.0
```

+aMax on the up-ramp, 0 while cruising, -aMax on the down-ramp

*field, declared at [`include/shulib/control/trapezoid_profile.hpp:44`](../../include/shulib/control/trapezoid_profile.hpp#L44).*

<a id="class-trapezoidprofile"></a>

## `class TrapezoidProfile`

```cpp
class TrapezoidProfile
```

A one-axis motion plan: ramp up at maxAcceleration, cruise, ramp down to rest exactly on target — degrading to a triangle when the move is too short to reach cruise speed. Built once per move and then IMMUTABLE: sample(t) is a pure function of t, so the same t always returns the same state, re-sampling is free, and nothing advances a baseline. **It has no consumer in this library yet** — the shipped motions servo the live error with Pid + Feedforward and generate no profile. Shipped, tested, and waiting for the profiled-motion work; the header note says why that is written down rather than implied.

*class, declared at [`include/shulib/control/trapezoid_profile.hpp:54`](../../include/shulib/control/trapezoid_profile.hpp#L54).*

<a id="trapezoidprofile-trapezoidprofile"></a>

### `TrapezoidProfile::TrapezoidProfile`

```cpp
TrapezoidProfile(double distance, const ProfileConstraints& c)
```

Plan a move of SIGNED `distance` under `c`. `distance` must be finite and both constraints strictly positive; a violation trips SHULIB_PRECONDITION rather than being clamped, because a silently corrected limit is a plan nobody asked for. If the move is too short to reach c.maxVelocity the plan degrades to a TRIANGLE (peak speed sqrt(|distance| * maxAcceleration), no cruise phase). A zero distance is legal and yields duration() == 0 — an already-finished plan, not an error.

*function, declared at [`include/shulib/control/trapezoid_profile.hpp:62`](../../include/shulib/control/trapezoid_profile.hpp#L62).*

<a id="trapezoidprofile-sample"></a>

### `TrapezoidProfile::sample`

```cpp
[[nodiscard]] ProfileState sample(double t) const
```

The target state at `t` SECONDS AFTER THE MOVE STARTED — the caller owns the clock and the elapsed-time subtraction. `t` is CLAMPED, never rejected: t <= 0 returns rest at the start with acceleration already at ±aMax (the next instant is the up-ramp; 0 for a zero-distance move), and t >= duration() returns rest exactly on target, forever. Const and side-effect-free. **A NaN `t` is the one input that is neither clamped nor rejected**: every comparison against NaN is false, so it falls through to the decelerate branch, which yields position and velocity NaN but acceleration a FINITE -aMax (mirrored for a negative move) — the down-ramp constant. A caller that screens only `acceleration` for finiteness will miss it. The constructor guards its own inputs with SHULIB_PRECONDITION; this one does not guard the clock, so a caller whose elapsed time can go non-finite must screen it before the call.

*function, declared at [`include/shulib/control/trapezoid_profile.hpp:98`](../../include/shulib/control/trapezoid_profile.hpp#L98).*

<a id="trapezoidprofile-duration"></a>

### `TrapezoidProfile::duration`

```cpp
[[nodiscard]] double duration() const noexcept
```

Total planned time in seconds, both ramps included (0 for a zero-distance move). This is the PLAN's time, not a promise the drivetrain tracks it — a follower's timeout must allow slack beyond this, not equal it.

*function, declared at [`include/shulib/control/trapezoid_profile.hpp:121`](../../include/shulib/control/trapezoid_profile.hpp#L121).*

<a id="trapezoidprofile-isdone"></a>

### `TrapezoidProfile::isDone`

```cpp
[[nodiscard]] bool isDone(double t) const noexcept
```

True once `t` has reached duration(), inclusive — i.e. sample(t) has stopped changing. True at t == 0 for a zero-distance move. A statement about the PLAN's clock only: it says nothing about whether the robot actually arrived, which is SettledUtil's question, measured against the real estimate.

*function, declared at [`include/shulib/control/trapezoid_profile.hpp:127`](../../include/shulib/control/trapezoid_profile.hpp#L127).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 20 lines</summary>

```text

 TrapezoidProfile — a trapezoidal motion profile (master plan §M2). For a move of signed
 `distance` under (maxVelocity, maxAcceleration), it gives the (position, velocity,
 acceleration) target at any time: ramp up at aMax to the cruise speed, cruise, ramp down
 to rest. If the move is too short to reach maxVelocity it degrades to a TRIANGLE (peak
 speed < maxVelocity, no cruise). (S-curve is a later sibling.)

 NOTHING IN THE TREE INSTANTIATES ONE YET, and this note is here because the header used
 to say the opposite. It claimed "the velocity target feeds Feedforward; the position
 target feeds the per-axis Pid" and "the motion layer instantiates one per axis" — both
 describe an INTENDED wiring, not a real one. The C1 motions run per-axis Pid plus
 Feedforward directly against the live error, with the speed cap applied in the command
 pipeline; no profile is generated anywhere, and the only consumer of this class in the
 repository is its own test. That is a gap, not a bug — profiled motion is future work —
 but a header that describes an integration it does not have is a lie a generated
 reference then publishes, so it says the true thing now.

 Bare doubles, like the rest of control: the CALLER picks the distance unit and supplies
 matching unit/s and unit/s² (inches and radians are what a motion layer would use).
 sample(t) clamps t to [0, duration] — see its own note for what NaN does.
```

</details>
