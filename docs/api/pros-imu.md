<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/imu.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `imu.hpp`

ProsImu — IImu over pros::Imu (chunk R1a): the load-bearing < 1° heading source behind the HAL.

This header declares **2** types (10 members).

Extracted from [`include/shulib/hal/pros/imu.hpp`](../../include/shulib/hal/pros/imu.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class YawRateSource`](#enum-class-yawratesource)
  - [`DifferentiateRotation`](#yawratesource-differentiaterotation)
  - [`GyroRateZ`](#yawratesource-gyroratez)
- [`class ProsImu`](#class-prosimu)
  - [`ProsImu`](#prosimu-prosimu)
  - [`calibrate`](#prosimu-calibrate)
  - [`heading`](#prosimu-heading)
  - [`yawRate`](#prosimu-yawrate)
  - [`isReady`](#prosimu-isready)
  - [`pitch`](#prosimu-pitch)
  - [`roll`](#prosimu-roll)
  - [`faultedReads`](#prosimu-faultedreads)

<a id="enum-class-yawratesource"></a>

## `enum class YawRateSource`

```cpp
enum class YawRateSource
```

Which hardware quantity yawRate() differentiates or reads (header: the T4 ruling and both costs).

*enum class, declared at [`include/shulib/hal/pros/imu.hpp:78`](../../include/shulib/hal/pros/imu.hpp#L78).*

<a id="yawratesource-differentiaterotation"></a>

### `YawRateSource::DifferentiateRotation`

```cpp
DifferentiateRotation
```

d/dt of get_rotation() — documented sign (default)

*enumerator, declared at [`include/shulib/hal/pros/imu.hpp:79`](../../include/shulib/hal/pros/imu.hpp#L79).*

<a id="yawratesource-gyroratez"></a>

### `YawRateSource::GyroRateZ`

```cpp
GyroRateZ
```

get_gyro_rate().z — real rate, UNDOCUMENTED sign (HA-04)

*enumerator, declared at [`include/shulib/hal/pros/imu.hpp:80`](../../include/shulib/hal/pros/imu.hpp#L80).*

<a id="class-prosimu"></a>

## `class ProsImu`

```cpp
class ProsImu final : public IImu
```

IImu over pros::Imu — the load-bearing heading source. Binds get_rotation() (cumulative CW degrees) and never get_heading(); applies bootHeading exactly ONCE, here at the edge; never tares, because a tare re-zeros the sensor underneath a live bootHeading offset. STATEFUL despite the const readers: each reader screens a non-finite value and returns the last good one, and the default yaw-rate path caches a sample to differentiate — which is why construction takes an IClock. One owner, the loop; not safe to read concurrently.

*class, declared at [`include/shulib/hal/pros/imu.hpp:89`](../../include/shulib/hal/pros/imu.hpp#L89).*

<a id="prosimu-prosimu"></a>

### `ProsImu::ProsImu`

```cpp
ProsImu(std::uint8_t port, math::Angle bootHeading, const IClock& clock, YawRateSource yawSource = YawRateSource::DifferentiateRotation)
```

`bootHeading`: the robot's canonical field heading AT calibration — ONE owner, the robot's start pose (HA-05). `clock`: required by the differentiation path; must outlive this adapter.

*function, declared at [`include/shulib/hal/pros/imu.hpp:94`](../../include/shulib/hal/pros/imu.hpp#L94).*

<a id="prosimu-calibrate"></a>

### `ProsImu::calibrate`

```cpp
void calibrate()
```

Start calibration (reset(false), HA-108). Call ONCE from initialize(); never blocks — the loop waits on isReady() (C1's wait-for-live). A second call is a precondition violation: re-calibrating mid-run re-zeros the sensor under a live bootHeading offset (HA-05's hazard).

*function, declared at [`include/shulib/hal/pros/imu.hpp:106`](../../include/shulib/hal/pros/imu.hpp#L106).*

<a id="prosimu-heading"></a>

### `ProsImu::heading`

```cpp
[[nodiscard]] math::Angle heading() const override
```

Canonical heading via imuHeadingToCanonical (bootHeading applied ONCE, here). Screened reads hold last-good; before any good read this is bootHeading itself (a robot that has not moved).

*function, declared at [`include/shulib/hal/pros/imu.hpp:117`](../../include/shulib/hal/pros/imu.hpp#L117).*

<a id="prosimu-yawrate"></a>

### `ProsImu::yawRate`

```cpp
[[nodiscard]] units::AngularVelocity yawRate() const override
```

Canonical yaw rate (CCW-positive rad/s) from the YawRateSource fixed at construction. DifferentiateRotation (the default) differentiates get_rotation() BETWEEN SUCCESSIVE CALLS, so the interval is however long since you last asked rather than a fixed dt: the first call after construction returns 0 (one sample is not a derivative), and a second call at the same clock instant repeats the previous answer instead of dividing by zero. Call it once per tick. GyroRateZ instead reads the hardware rate, converted on an UNMEASURED sign assumption (HA-04). Either way a screened read holds last-good.

*function, declared at [`include/shulib/hal/pros/imu.hpp:134`](../../include/shulib/hal/pros/imu.hpp#L134).*

<a id="prosimu-isready"></a>

### `ProsImu::isReady`

```cpp
[[nodiscard]] bool isReady() const override
```

True once calibrate() was called and calibration has finished (header: never-calibrated must read not-ready). POSITIVE polarity (imu.hpp:31-33).

*function, declared at [`include/shulib/hal/pros/imu.hpp:141`](../../include/shulib/hal/pros/imu.hpp#L141).*

<a id="prosimu-pitch"></a>

### `ProsImu::pitch`

```cpp
[[nodiscard]] math::Angle pitch() const override
```

Chassis pitch from get_pitch() (degrees, (-180, 180)) as a canonical Angle, UNNEGATED: the as-mounted sign convention is unmeasured (HA-110), so consume the MAGNITUDE until the bench settles it. Nothing reads this yet — the tip detection it exists for is a master-plan M2 item — so the first consumer is also the first chance to bake in a wrong sign. A non-finite read holds the last good value and counts in faultedReads(); before any good read, 0.

*function, declared at [`include/shulib/hal/pros/imu.hpp:151`](../../include/shulib/hal/pros/imu.hpp#L151).*

<a id="prosimu-roll"></a>

### `ProsImu::roll`

```cpp
[[nodiscard]] math::Angle roll() const override
```

Chassis roll from get_roll() (degrees, (-180, 180)) as a canonical Angle, on the same terms as pitch(): UNNEGATED because the mounting sign is unmeasured (HA-110), a non-finite read holds last-good and counts in faultedReads(), and 0 before any good read. Note that roll and pitch each hold their own last-good value, independently.

*function, declared at [`include/shulib/hal/pros/imu.hpp:165`](../../include/shulib/hal/pros/imu.hpp#L165).*

<a id="prosimu-faultedreads"></a>

### `ProsImu::faultedReads`

```cpp
[[nodiscard]] int faultedReads() const noexcept
```

How many reads were screened to last-good (T7 observability).

*function, declared at [`include/shulib/hal/pros/imu.hpp:176`](../../include/shulib/hal/pros/imu.hpp#L176).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 55 lines, click to expand</summary>

```text

 ProsImu — IImu over pros::Imu (chunk R1a): the load-bearing < 1° heading
 source behind the HAL.

 BINDS: pros::Imu::get_rotation() — CUMULATIVE CW degrees, "theoretically
 unbounded" — and NEVER get_heading(), which is bounded [0,360) and loses the
 revolution continuity the cumulative contract assumes (imu_conversion.hpp
 binding contract, clause 1; HA-03). Conversion: imuHeadingToCanonical(),
 which applies bootHeading exactly ONCE, here at the edge (HA-05) — no
 downstream consumer re-applies it.

 NEVER CALLS tare/tare_rotation/set_rotation/tare_heading/set_heading —
 each re-zeros the sensor independently of bootHeading and silently
 invalidates the additive offset (HA-05; the < 1° budget has no room for
 it). The ONE sanctioned reset() call is calibrate(), below, which IS the
 calibration — not a post-calibration tare. The fence guard test greps this
 file to keep all of that true structurally.

 YAW RATE — BOTH SOURCES SHIP, behind YawRateSource (team-lead ruling, brief
 T4; default = DifferentiateRotation):
  * DifferentiateRotation: d/dt of get_rotation(), whose CW-positive sign is
    DOCUMENTED — so the CW→CCW negate in imuYawRateToCanonical() is provably
    right. Costs: the adapter becomes STATEFUL (previous sample + timestamp,
    hence the injected IClock and the `mutable` members below — yawRate() is
    const, and the differentiation cache is part of the READ, not commanded
    state), and differentiating a quantized angle AMPLIFIES quantization
    noise — a real cost against the heading-owned < 1° budget, measured at
    the bench before trusting.
  * GyroRateZ: get_gyro_rate().z — a real hardware rate, but its SIGN is
    UNDOCUMENTED (HA-04) and its deg/s unit is itself a belief (HA-109).
    Assumed CW-positive until the bench measures it (runbook step: spin CW
    at a steady rate; canonical yaw rate must be negative).
  Rejected: shipping only one path (either loses a real hardware rate or
  defaults the load-bearing quantity to a sign nobody has measured).

 CALIBRATION: calibrate() starts it (reset(false), HA-108) — call it once
 from initialize(); it never blocks. isReady() is true only after
 calibrate() was called AND is_calibrating() has gone false: an
 never-calibrated IMU emits garbage-that-moves (HA-23), so "I was never
 calibrated" must read as not-ready, not as ready-by-default. POSITIVE
 polarity per imu.hpp:31-33.

 SENTINELS (T7): PROS_ERR_F reads (calibrating / unplugged) hold the last
 good value; heading() before any good read returns bootHeading (the best
 available estimate of a robot that has not moved). faultedReads() exposes
 the screen count; the loop's HealthMonitor owns raising IMU_LOST (its
 boot-window rule already handles the calibration phase).

 PITCH/ROLL: get_pitch()/get_roll() degrees (-180,180) → math::Angle,
 UNNEGATED — the as-mounted sign convention is unmeasured (HA-110). No
 consumer exists yet (the tip detection these exist for is a master-plan M2
 item); whoever writes the first one consumes MAGNITUDES until the bench
 settles the signs.

 HA register: HA-02..HA-05, HA-23, HA-108..HA-110.
```

</details>
