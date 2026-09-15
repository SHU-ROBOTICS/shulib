<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/localization/odometry.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `odometry.hpp`

IOdometry — the dead-reckoning seam the Localizer predicts from.

This header declares **1** type (10 members).

Extracted from [`include/shulib/localization/odometry.hpp`](../../include/shulib/localization/odometry.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class IOdometry`](#class-iodometry)
  - [`~IOdometry`](#iodometry-destructor-iodometry)
  - [`IOdometry`](#iodometry-iodometry)
  - [`IOdometry (overload 2)`](#iodometry-iodometry-2)
  - [`IOdometry (overload 3)`](#iodometry-iodometry-3)
  - [`operator=`](#iodometry-operator-eq)
  - [`operator= (overload 2)`](#iodometry-operator-eq-2)
  - [`update`](#iodometry-update)
  - [`pose`](#iodometry-pose)
  - [`setPose`](#iodometry-setpose)
  - [`lastDeltaImplausible`](#iodometry-lastdeltaimplausible)

<a id="class-iodometry"></a>

## `class IOdometry`

```cpp
class IOdometry
```

The dead-reckoning seam: the four members Localizer predicts from, and nothing else. Implementations integrate BODY travel from their own sensors into a field-frame Pose2d whose heading is the IMU's (never wheel-derived), advance only when update() is called, teleport position only on setPose(), and report — never withhold — an untrustworthy tick through lastDeltaImplausible(). PilonsOdometry (two tracking wheels) and DriveEncoderOdometry (the drive motors' own encoders) are the two implementations.

*class, declared at [`include/shulib/localization/odometry.hpp:42`](../../include/shulib/localization/odometry.hpp#L42).*

<a id="iodometry-destructor-iodometry"></a>

### `IOdometry::~IOdometry`

```cpp
virtual ~IOdometry() = default
```

Virtual so a polymorphic owner could destroy through the base; nothing in shulib does — the Localizer holds an IOdometry& and the caller owns the concrete object, which must outlive it. Copy/move re-defaulted so the base imposes no policy (the IMotor precedent, hal/motor.hpp).

*function, declared at [`include/shulib/localization/odometry.hpp:48`](../../include/shulib/localization/odometry.hpp#L48).*

<a id="iodometry-iodometry"></a>

### `IOdometry::IOdometry`

```cpp
IOdometry() = default
```

*Covered by the comment on [`~IOdometry`](#iodometry-destructor-iodometry) — one comment documents this run of special members.*

*function, declared at [`include/shulib/localization/odometry.hpp:49`](../../include/shulib/localization/odometry.hpp#L49).*

<a id="iodometry-iodometry-2"></a>

### `IOdometry::IOdometry (overload 2)`

```cpp
IOdometry(const IOdometry&) = default
```

*Covered by the comment on [`~IOdometry`](#iodometry-destructor-iodometry) — one comment documents this run of special members.*

*function, declared at [`include/shulib/localization/odometry.hpp:50`](../../include/shulib/localization/odometry.hpp#L50).*

<a id="iodometry-iodometry-3"></a>

### `IOdometry::IOdometry (overload 3)`

```cpp
IOdometry(IOdometry&&) = default
```

*Covered by the comment on [`~IOdometry`](#iodometry-destructor-iodometry) — one comment documents this run of special members.*

*function, declared at [`include/shulib/localization/odometry.hpp:51`](../../include/shulib/localization/odometry.hpp#L51).*

<a id="iodometry-operator-eq"></a>

### `IOdometry::operator=`

```cpp
IOdometry& operator=(const IOdometry&) = default
```

*Covered by the comment on [`~IOdometry`](#iodometry-destructor-iodometry) — one comment documents this run of special members.*

*function, declared at [`include/shulib/localization/odometry.hpp:52`](../../include/shulib/localization/odometry.hpp#L52).*

<a id="iodometry-operator-eq-2"></a>

### `IOdometry::operator= (overload 2)`

```cpp
IOdometry& operator=(IOdometry&&) = default
```

*Covered by the comment on [`~IOdometry`](#iodometry-destructor-iodometry) — one comment documents this run of special members.*

*function, declared at [`include/shulib/localization/odometry.hpp:53`](../../include/shulib/localization/odometry.hpp#L53).*

<a id="iodometry-update"></a>

### `IOdometry::update`

```cpp
virtual void update() = 0
```

One integration tick: read the sensors, integrate, accumulate. The caller calls it once per control tick (the Localizer does, inside its own update()).

*function, declared at [`include/shulib/localization/odometry.hpp:57`](../../include/shulib/localization/odometry.hpp#L57).*

<a id="iodometry-pose"></a>

### `IOdometry::pose`

```cpp
[[nodiscard]] virtual math::Pose2d pose() const noexcept = 0
```

The accumulated field-frame estimate — canonical inches, heading the IMU's. A pure read: unchanged between ticks.

*function, declared at [`include/shulib/localization/odometry.hpp:61`](../../include/shulib/localization/odometry.hpp#L61).*

<a id="iodometry-setpose"></a>

### `IOdometry::setPose`

```cpp
virtual void setPose(const math::Pose2d& p) = 0
```

Teleport the POSITION (x, y); heading stays IMU-owned. Must re-baseline whatever the implementation differences so the teleport itself injects no phantom motion.

*function, declared at [`include/shulib/localization/odometry.hpp:65`](../../include/shulib/localization/odometry.hpp#L65).*

<a id="iodometry-lastdeltaimplausible"></a>

### `IOdometry::lastDeltaImplausible`

```cpp
[[nodiscard]] virtual bool lastDeltaImplausible() const noexcept = 0
```

True iff the last update() was untrustworthy (the trust gate — header contract).

*function, declared at [`include/shulib/localization/odometry.hpp:68`](../../include/shulib/localization/odometry.hpp#L68).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 29 lines</summary>

```text

 IOdometry — the dead-reckoning seam the Localizer predicts from (chunk R3b Part 2,
 2026-09-14).

 ── Why a seam, and why exactly these four members ──────────────────────────────────
 Until this chunk the chain motion → IPoseSource → Localizer → PilonsOdometry → 2 ×
 IRotation was rigid: Localizer took a CONCRETE PilonsOdometry&, and PilonsOdometry
 precondition-requires two tracking wheels. A robot without tracking wheels — robot two,
 and most VEX robots — therefore could not have a Localizer at all, which is the gap the
 roadmap's "What R3b must BUILD" item 2 measured ("LemLib does drive-encoder odometry and
 shulib cannot"). The Localizer never needed PilonsOdometry specifically: it calls
 exactly four things on it — update(), pose(), setPose() and lastDeltaImplausible() —
 and this interface is those four and nothing else, so the seam is the Localizer's real
 dependency made explicit, not a new abstraction. PilonsOdometry implements it with NO
 behavioural change (the whole existing suite is the bit-identity pin), and
 DriveEncoderOdometry is the second implementation.

 The contract every implementation owes (PilonsOdometry's, generalized):
   * update() integrates one tick and advances the pose; the CALLER owns the cadence.
   * pose() is a pure read — the same value until the next update() or setPose().
   * HEADING IS IMU-OWNED: pose().heading() is the IMU's reading, never integrated from
     wheels (pilons_odometry.hpp decision #3; the < 1° heading spec depends on it).
     setPose() therefore teleports POSITION only.
   * lastDeltaImplausible() is the TRUST GATE: true when the last tick was not to be
     trusted (an oversized heading change, an oversized wheel delta, or a non-finite
     integration). An implausible-but-finite delta is REPORTED, NEVER WITHHELD
     (plausibility_guard.hpp principle 4); only a non-finite tick freezes position.
     HealthMonitor turns the flag into ODO_STUCK.
 Raises no faults (raising is the loop layer's policy). Single-task by contract.
```

</details>
