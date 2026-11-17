<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/health_monitor.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `health_monitor.hpp`

HealthMonitor — sensor/power pathology → FaultCode, edge-triggered.

This header declares **3** types (17 members).

Extracted from [`include/shulib/diag/health_monitor.hpp`](../../include/shulib/diag/health_monitor.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct HealthMonitorConfig`](#struct-healthmonitorconfig)
  - [`brownoutVolts`](#healthmonitorconfig-brownoutvolts)
  - [`brownoutRecoverVolts`](#healthmonitorconfig-brownoutrecovervolts)
  - [`maxMotorTempC`](#healthmonitorconfig-maxmotortempc)
- [`class HealthMonitor`](#class-healthmonitor)
  - [`HealthMonitor`](#healthmonitor-healthmonitor)
  - [`tick`](#healthmonitor-tick)
  - [`motorGroupDisagreeing`](#healthmonitor-motorgroupdisagreeing)
  - [`brownedOut`](#healthmonitor-brownedout)
  - [`imuLost`](#healthmonitor-imulost)
  - [`reset`](#healthmonitor-reset)
  - [`struct HealthMonitor::Observations`](#struct-healthmonitor-observations)
    - [`imuReady`](#healthmonitor-observations-imuready)
    - [`odomImplausible`](#healthmonitor-observations-odomimplausible)
    - [`odomStalled`](#healthmonitor-observations-odomstalled)
    - [`fixGated`](#healthmonitor-observations-fixgated)
    - [`batteryVolts`](#healthmonitor-observations-batteryvolts)
    - [`maxMotorTempC`](#healthmonitor-observations-maxmotortempc)
    - [`groupMembersDisagreeing`](#healthmonitor-observations-groupmembersdisagreeing)
    - [`odomAudit`](#healthmonitor-observations-odomaudit)

<a id="struct-healthmonitorconfig"></a>

## `struct HealthMonitorConfig`

```cpp
struct HealthMonitorConfig
```

The trip points HealthMonitor compares each tick's observables against. Every number here is PROVISIONAL hardware guesswork rather than measurement — the V5's real cutoff under load and the real thermal droop onset on our motors are both unmeasured until the on-robot phase (register HA-42, HA-44) — so treat the defaults as a starting point to tune, not calibration.

*struct, declared at [`include/shulib/diag/health_monitor.hpp:94`](../../include/shulib/diag/health_monitor.hpp#L94).*

<a id="healthmonitorconfig-brownoutvolts"></a>

### `HealthMonitorConfig::brownoutVolts`

```cpp
units::Voltage brownoutVolts{10.5}
```

Battery voltage at/below which a BROWNOUT episode trips. PROVISIONAL (A4: HA-42).

*field, declared at [`include/shulib/diag/health_monitor.hpp:96`](../../include/shulib/diag/health_monitor.hpp#L96).*

<a id="healthmonitorconfig-brownoutrecovervolts"></a>

### `HealthMonitorConfig::brownoutRecoverVolts`

```cpp
units::Voltage brownoutRecoverVolts{10.8}
```

Voltage the pack must RECOVER above before a new brownout episode can trip (hysteresis; must be >= brownoutVolts). PROVISIONAL (A4: HA-42).

*field, declared at [`include/shulib/diag/health_monitor.hpp:99`](../../include/shulib/diag/health_monitor.hpp#L99).*

<a id="healthmonitorconfig-maxmotortempc"></a>

### `HealthMonitorConfig::maxMotorTempC`

```cpp
double maxMotorTempC = 55.0
```

Motor temperature (°C) at/above which MOTOR_OVER_TEMP trips. PROVISIONAL (A4: HA-44).

*field, declared at [`include/shulib/diag/health_monitor.hpp:101`](../../include/shulib/diag/health_monitor.hpp#L101).*

<a id="class-healthmonitor"></a>

## `class HealthMonitor`

```cpp
class HealthMonitor
```

Turns per-tick sensor and power observables into FaultCode raises. EDGE-TRIGGERED per EPISODE: a pathology that persists for 500 ticks is ONE fault, not 500, and each condition re-arms only once it clears — brownout with hysteresis on top, so a pack sagging around the threshold under a pulsing load cannot chatter episodes. It takes plain VALUES rather than component references, because diag/ is a dependency leaf and may not name estimator types; the caller reads them from the components it already owns. Timing is deliberately not here — LoopMonitor owns overruns. Single-task by contract, like the rest of diag/.

*class, declared at [`include/shulib/diag/health_monitor.hpp:111`](../../include/shulib/diag/health_monitor.hpp#L111).*

<a id="healthmonitor-healthmonitor"></a>

### `HealthMonitor::HealthMonitor`

```cpp
explicit HealthMonitor(FaultLatch& faults, const HealthMonitorConfig& config = {})
```

EXPLICIT: the defaulted second parameter makes this a one-argument converting constructor, so without it `HealthMonitor m = someLatch;` compiled and a FaultLatch& silently converted at any call site taking a HealthMonitor by value or const&. Its diag/ siblings mark their single-argument constructors explicit (tick_attribution.hpp), which is what made this read as an oversight rather than a decision.  `faults` is borrowed, not owned, and must outlive the monitor — which only ever raises into it and never clears it. `config` is COPIED and checked here rather than at the first trip: all three thresholds must be finite, brownoutVolts and maxMotorTempC must be > 0, and brownoutRecoverVolts must be >= brownoutVolts so hysteresis cannot run backwards.  The finiteness of the recover level is load-bearing rather than tidiness. It used to be ORDERED but not checked for finiteness, so `+Inf` constructed — and `+Inf` passes the ordering. The re-arm test in tick() (`v >= brownoutRecoverVolts`) could then never be true, brownoutActive_ never cleared, and the whole run reported at most ONE brownout episode however many times the pack collapsed: the E1 anti-spam edge trigger silently became a permanent mute on the one signal it exists to report.

*function, declared at [`include/shulib/diag/health_monitor.hpp:160`](../../include/shulib/diag/health_monitor.hpp#L160).*

<a id="healthmonitor-tick"></a>

### `HealthMonitor::tick`

```cpp
void tick(const Observations& o)
```

Evaluate one tick's observables; raise one fault per NEW episode (header).

*function, declared at [`include/shulib/diag/health_monitor.hpp:174`](../../include/shulib/diag/health_monitor.hpp#L174).*

<a id="healthmonitor-motorgroupdisagreeing"></a>

### `HealthMonitor::motorGroupDisagreeing`

```cpp
[[nodiscard]] bool motorGroupDisagreeing() const noexcept
```

True while at least one coupled-group member is in a persisted-disagreement episode (as of the last tick) — for a panel that shows the live state beside the latched fault.

*function, declared at [`include/shulib/diag/health_monitor.hpp:259`](../../include/shulib/diag/health_monitor.hpp#L259).*

<a id="healthmonitor-brownedout"></a>

### `HealthMonitor::brownedOut`

```cpp
[[nodiscard]] bool brownedOut() const noexcept
```

True once ANY brownout episode has occurred this run (latched; header note).

*function, declared at [`include/shulib/diag/health_monitor.hpp:262`](../../include/shulib/diag/health_monitor.hpp#L262).*

<a id="healthmonitor-imulost"></a>

### `HealthMonitor::imuLost`

```cpp
[[nodiscard]] bool imuLost() const noexcept
```

True while the IMU is in a lost episode (seen ready, currently not).

*function, declared at [`include/shulib/diag/health_monitor.hpp:264`](../../include/shulib/diag/health_monitor.hpp#L264).*

<a id="healthmonitor-reset"></a>

### `HealthMonitor::reset`

```cpp
void reset() noexcept
```

New-run boundary (mirrors FaultLatch::clear()): forget episodes AND the brownout marker; the boot-window rule starts over (imuSeenReady resets).

*function, declared at [`include/shulib/diag/health_monitor.hpp:268`](../../include/shulib/diag/health_monitor.hpp#L268).*

<a id="struct-healthmonitor-observations"></a>

## `struct HealthMonitor::Observations`

```cpp
struct Observations
```

The per-tick observables. The caller reads these from the components it already owns; every default is the HEALTHY value, so a caller without some source (e.g. no motor temps wired yet) simply leaves the field alone.

*struct, declared at [`include/shulib/diag/health_monitor.hpp:116`](../../include/shulib/diag/health_monitor.hpp#L116).*

<a id="healthmonitor-observations-imuready"></a>

### `HealthMonitor::Observations::imuReady`

```cpp
bool imuReady = true
```

IImu::isReady()

*field, declared at [`include/shulib/diag/health_monitor.hpp:117`](../../include/shulib/diag/health_monitor.hpp#L117).*

<a id="healthmonitor-observations-odomimplausible"></a>

### `HealthMonitor::Observations::odomImplausible`

```cpp
bool odomImplausible = false
```

IOdometry::lastDeltaImplausible() (the trust gate)

*field, declared at [`include/shulib/diag/health_monitor.hpp:118`](../../include/shulib/diag/health_monitor.hpp#L118).*

<a id="healthmonitor-observations-odomstalled"></a>

### `HealthMonitor::Observations::odomStalled`

```cpp
bool odomStalled = false
```

caller-computed wheels-spin-but-no-motion cross-check (see note below)

*field, declared at [`include/shulib/diag/health_monitor.hpp:119`](../../include/shulib/diag/health_monitor.hpp#L119).*

<a id="healthmonitor-observations-fixgated"></a>

### `HealthMonitor::Observations::fixGated`

```cpp
bool fixGated = false
```

Localizer::lastCorrection().gated

*field, declared at [`include/shulib/diag/health_monitor.hpp:121`](../../include/shulib/diag/health_monitor.hpp#L121).*

<a id="healthmonitor-observations-batteryvolts"></a>

### `HealthMonitor::Observations::batteryVolts`

```cpp
units::Voltage batteryVolts{12.6}
```

IBattery::voltage()

*field, declared at [`include/shulib/diag/health_monitor.hpp:122`](../../include/shulib/diag/health_monitor.hpp#L122).*

<a id="healthmonitor-observations-maxmotortempc"></a>

### `HealthMonitor::Observations::maxMotorTempC`

```cpp
double maxMotorTempC = 0.0
```

max IMotor::temperature() over the drive

*field, declared at [`include/shulib/diag/health_monitor.hpp:123`](../../include/shulib/diag/health_monitor.hpp#L123).*

<a id="healthmonitor-observations-groupmembersdisagreeing"></a>

### `HealthMonitor::Observations::groupMembersDisagreeing`

```cpp
int groupMembersDisagreeing = 0
```

members of any hal::MotorGroup PERSISTENTLY disagreeing with their coupled group this tick (motion::tickHealthObservables sums every group's disagreeingMembers()); 0 = healthy, and the default for a robot with no groups

*field, declared at [`include/shulib/diag/health_monitor.hpp:124`](../../include/shulib/diag/health_monitor.hpp#L124).*

<a id="healthmonitor-observations-odomaudit"></a>

### `HealthMonitor::Observations::odomAudit`

```cpp
OdomTickAudit odomAudit{}
```

IOdometry::lastTickAudit() — the gate's account of the SAME tick odomImplausible judges. DETAIL ONLY: printed on the ODO_STUCK line, never a trigger. APPENDED last (DIAG1): callers use designated initializers, which must follow declaration order. Left default, the line reads `why=UNREPORTED`.

*field, declared at [`include/shulib/diag/health_monitor.hpp:129`](../../include/shulib/diag/health_monitor.hpp#L129).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 76 lines, click to expand</summary>

```text

 HealthMonitor — sensor/power pathology → FaultCode, edge-triggered (WS13, chunk A3).

 ── Why this exists ─────────────────────────────────────────────────────────────────
 A1 built the fault vocabulary (FaultCode) and the latch (FaultLatch); A3 is the
 chunk that must prove "every sensor pathology raises a fault code with a safe
 fallback" is REAL. The estimators deliberately do not raise faults themselves —
 PilonsOdometry exposes lastDeltaImplausible(), the Localizer exposes quality
 state, the HAL exposes isReady()/voltage()/temperature() — because raising is
 POLICY, and fault.hpp assigns that policy to the loop layer ("OdoStuck …
 raised by the C/E layers"). This class is that policy, factored so the A3 test
 loops and C1's real tick loop share ONE implementation instead of each
 hand-rolling edge detection.

 ── Why it takes raw OBSERVABLES, not component references ──────────────────────────
 tick(Observations) receives plain bools/values the caller already reads each tick
 (imu.isReady(), odom.lastDeltaImplausible(), localizer.lastCorrection().gated,
 battery.voltage(), max motor temperature). Rejected: holding IImu&/Localizer&
 references — diag/ is a dependency LEAF (debug_record.hpp: localization may
 include diag, NEVER the reverse), so the monitor cannot name estimator types; and
 raw values also keep it trivially testable and reusable for any future source of
 the same observables. LoopOverrun is deliberately NOT here — LoopMonitor (A1)
 already owns timing and its dt bookkeeping; two monitors, two concerns.

 ── Edge-triggered, per EPISODE (the anti-spam contract) ────────────────────────────
 A pathology that persists for 500 ticks is ONE episode, not 500 faults: each
 condition raises on its false→true transition and re-arms when the condition
 clears. FaultLatch already counts cascades; flooding it with one code per tick
 would bury the first-fault story that latch exists to tell (a 2am log with 500
 IMU_LOST lines is the legacy anti-pattern §18 bans). Brownout adds HYSTERESIS
 (recoverVolts above the trip point) so a pack sagging around the threshold under
 a pulsing load cannot chatter episodes.

 ── Semantics that are contracts, each pinned by test ───────────────────────────────
  * IMU_LOST fires only on a loss AFTER the IMU has been seen ready — the boot
    calibration window (isReady() false from t=0) is NORMAL, not a fault. Waiting
    out calibration is the loop's job (C1); reporting it as a loss would make every
    clean boot start with a spurious fault.
  * BROWNOUT is additionally LATCHED here (brownedOut() stays true for the run) —
    the E1 "latched brownout marker" semantics: a pack that collapsed and bounced
    back is still a collapsed pack, and the end-of-run summary must say so.
  * GPS no-fix is NOT a fault, by design: Driving Skills has no strip at all, so
    "no fix" is a normal operating state the Localizer already reports as quality
    decay. What IS raised is GPS_GATE_REJECT — a fix that arrived and was rejected
    by the fusion gate (the fixGated observable), i.e. a sensor actively lying.
  * MOTOR_OVER_TEMP threshold defaults to 55 °C — the V5's documented first
    throttle step. PROVISIONAL (A4 register HA-44): the exact droop onset on our
    motors is unmeasured until R4.
  * brownoutVolts default 10.5 V — PROVISIONAL (A4 register HA-42): the true V5
    cutoff behaviour under load is unmeasured until R3/R4.
    (Register: docs/hardware-assumptions.md.)

 ── The odometry line (chunk DIAG1, 2026-09-26) ─────────────────────────────────────
 ODO_STUCK is the scheduler's default ABORT fault, and on 2026-09-25 robot two raised it as
 `implausible odometry tick` with nothing else, so the cause could not be read off the log.
 The line now carries the trust gate's own account (Observations::odomAudit,
 diag/odom_tick_audit.hpp): `implausible odometry tick why=ROT dth=1.745rad h0=0.012
 h1=1.757 L=0.000in R=0.000in`. What did NOT change, each pinned by test (except where
 said):
  * The TRIGGER is still `odomImplausible || odomStalled`. The audit is detail only — a
    tripped audit with the verdict false raises nothing — so the fault cannot move.
  * The detail is RAISED only on the rising edge; a 500-tick episode is one line carrying
    the FIRST tick's values (pinned). It is also only FORMATTED there, so the ticks after it
    cost nothing — a property of the code, NOT pinned: building the text on every bad tick
    and raising only on the edge would print the same lines, so no output test can see it.
  * An implausible verdict with an empty audit (a source that never filled it) reads
    ` why=UNREPORTED` rather than inventing values.
  * When the stall verdict and the implausible verdict rise on the SAME tick, the valued
    implausible text wins and ` +STALL` is appended (the team lead's call, 2026-09-26): the
    values are what cannot be recovered afterwards, while the stall is a caller verdict with
    nothing more to say. A stall alone still reads `wheels spin but odometry reports no
    motion`, exactly as before.
 The detail is bounded by kOdomAuditDetailMaxBytes (140) so FaultLatch's 176-byte line
 never truncates it; the worst case is 137 bytes (odom_tick_audit.hpp).

 Single-task by contract, like the rest of diag/ (see fault.hpp).
```

</details>
