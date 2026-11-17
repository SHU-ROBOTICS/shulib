<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/odom_tick_audit.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `odom_tick_audit.hpp`

OdomTickAudit — the odometry trust gate's account of ONE tick: which of its checks tripped and the values it tripped on.

This header declares **2** types (12 members), **1** free function, and **2** constants.

Extracted from [`include/shulib/diag/odom_tick_audit.hpp`](../../include/shulib/diag/odom_tick_audit.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class OdomTravelLabels`](#enum-class-odomtravellabels)
  - [`DriveSides`](#odomtravellabels-drivesides)
  - [`TrackingWheels`](#odomtravellabels-trackingwheels)
- [`struct OdomTickAudit`](#struct-odomtickaudit)
  - [`nonFinite`](#odomtickaudit-nonfinite)
  - [`rotation`](#odomtickaudit-rotation)
  - [`travel`](#odomtickaudit-travel)
  - [`dTheta`](#odomtickaudit-dtheta)
  - [`headingBefore`](#odomtickaudit-headingbefore)
  - [`headingAfter`](#odomtickaudit-headingafter)
  - [`travelA`](#odomtickaudit-travela)
  - [`travelB`](#odomtickaudit-travelb)
  - [`labels`](#odomtickaudit-labels)
  - [`tripped`](#odomtickaudit-tripped)
- [`kOdomAuditDetailMaxBytes`](#kodomauditdetailmaxbytes) — *constant*
- [`kOdomAuditPrecision`](#kodomauditprecision) — *constant*
- [`appendOdomTickAudit`](#appendodomtickaudit) — *free function*

<a id="enum-class-odomtravellabels"></a>

## `enum class OdomTravelLabels`

```cpp
enum class OdomTravelLabels : std::uint8_t
```

Which pair of labels a rendering gives its two travel values: the drive-encoder odometry's two SIDES (`L=`, `R=`) or the tracking-wheel odometry's two PODS (`fwd=`, `lat=`). The same two numbers mean different things on the two robots, so the line says which — a "check the tracking wheels" reading of a drive-encoder robot's line is exactly the misdiagnosis this prevents.

*enum class, declared at [`include/shulib/diag/odom_tick_audit.hpp:96`](../../include/shulib/diag/odom_tick_audit.hpp#L96).*

<a id="odomtravellabels-drivesides"></a>

### `OdomTravelLabels::DriveSides`

```cpp
DriveSides
```

DriveEncoderOdometry: travelA = left side, travelB = right side

*enumerator, declared at [`include/shulib/diag/odom_tick_audit.hpp:97`](../../include/shulib/diag/odom_tick_audit.hpp#L97).*

<a id="odomtravellabels-trackingwheels"></a>

### `OdomTravelLabels::TrackingWheels`

```cpp
TrackingWheels
```

PilonsOdometry: travelA = forward pod (RAW), travelB = lateral pod

*enumerator, declared at [`include/shulib/diag/odom_tick_audit.hpp:98`](../../include/shulib/diag/odom_tick_audit.hpp#L98).*

<a id="struct-odomtickaudit"></a>

## `struct OdomTickAudit`

```cpp
struct OdomTickAudit
```

The trust gate's account of the LAST update(): which checks tripped and the values they were computed from — every tick, not sticky (a clean tick clears every flag). The flags are the gate's own sub-expressions (header), so `tripped()` IS the verdict lastDeltaImplausible() reports. Default-constructed = "no account": nothing tripped, every value zero — what an odometry reports before its first update().

*struct, declared at [`include/shulib/diag/odom_tick_audit.hpp:106`](../../include/shulib/diag/odom_tick_audit.hpp#L106).*

<a id="odomtickaudit-nonfinite"></a>

### `OdomTickAudit::nonFinite`

```cpp
bool nonFinite = false
```

The integrated field delta came out non-finite (NaN or +/-Inf), so position FROZE this tick (heading still advanced).

*field, declared at [`include/shulib/diag/odom_tick_audit.hpp:109`](../../include/shulib/diag/odom_tick_audit.hpp#L109).*

<a id="odomtickaudit-rotation"></a>

### `OdomTickAudit::rotation`

```cpp
bool rotation = false
```

|dTheta| exceeded maxTickRotation (strict: equality does not trip).

*field, declared at [`include/shulib/diag/odom_tick_audit.hpp:111`](../../include/shulib/diag/odom_tick_audit.hpp#L111).*

<a id="odomtickaudit-travel"></a>

### `OdomTickAudit::travel`

```cpp
bool travel = false
```

A travel value's magnitude was NOT within maxTickTravel — true for an oversized value AND for a NaN one (the comparison fails; header), never at exact equality.

*field, declared at [`include/shulib/diag/odom_tick_audit.hpp:114`](../../include/shulib/diag/odom_tick_audit.hpp#L114).*

<a id="odomtickaudit-dtheta"></a>

### `OdomTickAudit::dTheta`

```cpp
units::AngleDim dTheta{}
```

The tick's heading change, radians: shortest signed, in (-pi, pi].

*field, declared at [`include/shulib/diag/odom_tick_audit.hpp:116`](../../include/shulib/diag/odom_tick_audit.hpp#L116).*

<a id="odomtickaudit-headingbefore"></a>

### `OdomTickAudit::headingBefore`

```cpp
units::AngleDim headingBefore{}
```

The IMU heading the tick started from, radians in (-pi, pi].

*field, declared at [`include/shulib/diag/odom_tick_audit.hpp:118`](../../include/shulib/diag/odom_tick_audit.hpp#L118).*

<a id="odomtickaudit-headingafter"></a>

### `OdomTickAudit::headingAfter`

```cpp
units::AngleDim headingAfter{}
```

The IMU heading the tick ended on, radians in (-pi, pi].

*field, declared at [`include/shulib/diag/odom_tick_audit.hpp:120`](../../include/shulib/diag/odom_tick_audit.hpp#L120).*

<a id="odomtickaudit-travela"></a>

### `OdomTickAudit::travelA`

```cpp
units::Length travelA{}
```

First travel value the gate bounded, inches: the LEFT side, or the forward pod's RAW travel (before offset correction — what the gate actually compares).

*field, declared at [`include/shulib/diag/odom_tick_audit.hpp:123`](../../include/shulib/diag/odom_tick_audit.hpp#L123).*

<a id="odomtickaudit-travelb"></a>

### `OdomTickAudit::travelB`

```cpp
units::Length travelB{}
```

Second travel value the gate bounded, inches: the RIGHT side, or the lateral pod's.

*field, declared at [`include/shulib/diag/odom_tick_audit.hpp:125`](../../include/shulib/diag/odom_tick_audit.hpp#L125).*

<a id="odomtickaudit-labels"></a>

### `OdomTickAudit::labels`

```cpp
OdomTravelLabels labels = OdomTravelLabels::DriveSides
```

Which labels the two travel values render under.

*field, declared at [`include/shulib/diag/odom_tick_audit.hpp:127`](../../include/shulib/diag/odom_tick_audit.hpp#L127).*

<a id="odomtickaudit-tripped"></a>

### `OdomTickAudit::tripped`

```cpp
[[nodiscard]] constexpr bool tripped() const noexcept
```

True iff any check tripped — by construction the verdict lastDeltaImplausible() reports for the same tick (each implementation sets its verdict FROM this).

*function, declared at [`include/shulib/diag/odom_tick_audit.hpp:131`](../../include/shulib/diag/odom_tick_audit.hpp#L131).*

<a id="kodomauditdetailmaxbytes"></a>

## `kOdomAuditDetailMaxBytes`

```cpp
inline constexpr std::size_t kOdomAuditDetailMaxBytes = 140
```

The most bytes an ODO_STUCK detail may carry and still reach the log whole: FaultLatch's 176-byte line buffer, less its NUL and the 35-byte worst-case prefix (header).

*constant, declared at [`include/shulib/diag/odom_tick_audit.hpp:138`](../../include/shulib/diag/odom_tick_audit.hpp#L138).*

<a id="kodomauditprecision"></a>

## `kOdomAuditPrecision`

```cpp
inline constexpr int kOdomAuditPrecision = 3
```

Decimal places every audit number renders with — enough to tell 1.571 (pi/2, the rotation bound) from 1.570, and 36.000 in (the travel bound) from 36.001.

*constant, declared at [`include/shulib/diag/odom_tick_audit.hpp:142`](../../include/shulib/diag/odom_tick_audit.hpp#L142).*

<a id="appendodomtickaudit"></a>

## `appendOdomTickAudit`

```cpp
inline void appendOdomTickAudit(lineformat::Line& line, const OdomTickAudit& audit)
```

Append the audit's rendering to `line` (header: the exact text): ` why=` and the tripped checks' tokens, then the three angles and the two travel values under `audit.labels` — or ` why=UNREPORTED` alone when nothing tripped. Numbers via lineformat::appendNum only.

*free function, declared at [`include/shulib/diag/odom_tick_audit.hpp:147`](../../include/shulib/diag/odom_tick_audit.hpp#L147).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 80 lines, click to expand</summary>

```text

 OdomTickAudit — the odometry trust gate's account of ONE tick: which of its checks tripped
 and the values it tripped on (chunk DIAG1, 2026-09-26).

 ── Why this exists ─────────────────────────────────────────────────────────────────
 On 2026-09-25 robot two raised `fault=ODO_STUCK n=1 FIRST implausible odometry tick`
 twice in one start, and the line named neither the check that tripped nor a single value,
 so the cause could not be told from the log. That matters more than one bad line:
 ODO_STUCK is the scheduler's DEFAULT abort fault (motion_scheduler.hpp,
 MotionSchedulerConfig::abortFaultMask), so a repeat during an autonomous ENDS the drive.
 It has to be explainable from the log before the first auton, not reproduced on the bench.

 ── Why it lives in diag/, as a plain value ─────────────────────────────────────────
 The gate is computed inside each IOdometry implementation (localization/), and the ONLY
 place ODO_STUCK is raised is HealthMonitor (this directory). diag/ is a dependency LEAF —
 localization may include diag, never the reverse (debug_record.hpp, health_monitor.hpp) —
 so the monitor cannot name an odometry type. The account therefore travels the same way
 the verdict always did, as an OBSERVABLE: the odometry fills this struct every update(),
 the Localizer forwards it, motion::tickHealthObservables copies it into
 HealthMonitor::Observations, and the monitor renders it on the rising edge only.

 ── What it records, and what it does NOT decide ────────────────────────────────────
 The three flags are the gate's three sub-expressions, COPIED AS THEY STAND in each
 implementation — never re-derived — and each implementation then sets its verdict to
 tripped(). So the verdict (lastDeltaImplausible()) and this account cannot disagree: they
 are the same bits, by construction, and the existing trust-gate tests are the pin that the
 verdict did not move. One consequence of copying rather than re-deriving is documented,
 not "fixed": `travel` is `!(|a| <= max && |b| <= max)`, and a comparison with NaN is
 false, so a NaN side reports NONFINITE+TRAVEL together (its travel is not "within the
 bound", which is true). This struct decides nothing — HealthMonitor's trigger is still the
 verdict, never tripped() — so a bug here can garble a line but cannot raise or hide a
 fault.

 When is NONFINITE the ONLY check that trips? Written down so nobody re-derives it — DIAG1's
 first mutation run got it wrong (it called dropping `!finite` from the verdict an
 "equivalent change no test can see"), and review round 1 found the configurations that
 break that claim; the suite now pins them (DIAG1 B10, C9). The heading change is always
 finite (an Angle cannot hold NaN, and |dTheta| <= pi), and arcStep's chord factor and its
 rotation are at most 1. So once every travel value is finite and within maxTickTravel (M),
 the field delta is non-finite only through an OVERFLOW on the way to it:
  * drive encoders: the centre is 0.5 * (dL + dR), and the SUM overflows before the
    halving when M > DBL_MAX / 2 (both sides at 1e308 under M = DBL_MAX: +Inf, then arcStep
    gives dx = +Inf, dy = Inf * 0 = NaN);
  * tracking wheels: arcStep rotates (fwd, lat), whose length can reach sqrt(2) * M, so an
    M from about DBL_MAX / sqrt(2) up overflows at a 45-degree heading; and the offset
    correction dTheta * offset overflows once |offset| passes DBL_MAX / |dTheta| (so from
    DBL_MAX / pi up, on a big enough turn), and is non-finite on EVERY tick for a
    non-finite offset (0 * Inf is NaN) — TrackingWheel checks neither, only the diameter.
 With M at most DBL_MAX / 2 and offsets whose pi-multiple plus M stays under DBL_MAX / 2, a
 NONFINITE tick is always also a TRAVEL tick (a NaN or Inf travel value fails `<= M`). Every
 real robot is some 300 orders of magnitude inside that (36 in; offsets of inches), but the
 config checks ask only for a finite M > 0, so the breaking configurations are legal — and in
 them NONFINITE appears ALONE (`why=NONFINITE`), the only flag that says position FROZE.

 ── The rendering (the text a person reads) ─────────────────────────────────────────
   ` why=ROT+TRAVEL dth=<n>rad h0=<n> h1=<n> L=<n>in R=<n>in`   (drive-encoder sides)
   ` why=TRAVEL ... fwd=<n>in lat=<n>in`                         (tracking wheels)
 Tokens in the fixed order NONFINITE, ROT, TRAVEL, joined by '+'. dth, h0 and h1 are
 RADIANS (1.571 = 90 deg; the default rotation bound is pi/2 = 1.571, the travel bound
 36 in); h0/h1 are the IMU headings the tick started and ended on. An audit with no flag
 set renders ` why=UNREPORTED` and NO values: the verdict said implausible but the source
 did not say why (a test double, or a caller that never filled the field), and printing
 the values of an account nobody gave would invent one. Every number goes through
 lineformat::appendNum at precision 3 and width 0 — the ONE formatter (NaN, +Inf and -Inf
 as fixed tokens, anything longer than 10 bytes compacted to %.3g) — never `snprintf %f`,
 which prints libc's own NaN spellings and a 300-digit value that would blow the size
 bound below. ASCII only.

 ── The size bound ──────────────────────────────────────────────────────────────────
 FaultLatch::raise() formats into `char buf[176]` and TRUNCATES SILENTLY (fault.hpp). Its
 prefix for this code is at most 35 bytes — `fault=ODO_STUCK` (15) + ` n=` (3) + a 10-digit
 count + ` FIRST` (6) + the separating space (1) — so the whole DETAIL may be at most
 175 - 35 = 140 bytes: kOdomAuditDetailMaxBytes. Each <n> is at most 10 bytes (a %.3f
 rendering longer than that is compacted, and %.3g's widest is `-1.23e+308`), so the worst
 detail HealthMonitor can build — `implausible odometry tick` (25) + all three tokens (25)
 + five 10-byte numbers with their labels and units (80, tracking-wheel labels) + ` +STALL`
 (7) — is 137 bytes. The test suite pins it against adversarial values, not this sum.

 Stateless free function plus a plain aggregate; nothing allocates. Single-task by contract,
 like the rest of diag/.
```

</details>
