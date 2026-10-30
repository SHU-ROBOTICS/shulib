<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/imu.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `imu.hpp`

ProsImu — IImu over pros::Imu (chunk R1a): the load-bearing < 1° heading source behind the HAL.

This header declares **2** types (11 members).

Extracted from [`include/shulib/hal/pros/imu.hpp`](../../include/shulib/hal/pros/imu.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class YawRateSource`](#enum-class-yawratesource)
  - [`DifferentiateRotation`](#yawratesource-differentiaterotation)
  - [`GyroRateZ`](#yawratesource-gyroratez)
- [`class ProsImu`](#class-prosimu)
  - [`kRateHoldUs`](#prosimu-krateholdus)
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

*enum class, declared at [`include/shulib/hal/pros/imu.hpp:138`](../../include/shulib/hal/pros/imu.hpp#L138).*

<a id="yawratesource-differentiaterotation"></a>

### `YawRateSource::DifferentiateRotation`

```cpp
DifferentiateRotation
```

d/dt of get_rotation() — documented sign (default)

*enumerator, declared at [`include/shulib/hal/pros/imu.hpp:139`](../../include/shulib/hal/pros/imu.hpp#L139).*

<a id="yawratesource-gyroratez"></a>

### `YawRateSource::GyroRateZ`

```cpp
GyroRateZ
```

get_gyro_rate().z — real rate, UNDOCUMENTED sign (HA-04)

*enumerator, declared at [`include/shulib/hal/pros/imu.hpp:140`](../../include/shulib/hal/pros/imu.hpp#L140).*

<a id="class-prosimu"></a>

## `class ProsImu`

```cpp
class ProsImu final : public IImu
```

IImu over pros::Imu — the load-bearing heading source. Binds get_rotation() (cumulative CW degrees) and never get_heading(); applies bootHeading exactly ONCE, here at the edge; never tares, because a tare re-zeros the sensor underneath a live bootHeading offset. STATEFUL despite the const readers: each reader screens a non-finite value and returns the last good one, the default yaw-rate path caches a sample to differentiate, and yawRate() answers ONCE per control tick — a read within kRateHoldUs (5 ms) of the last fresh answer returns that answer (header: why, and the hold's three limits). Construction takes an IClock for both. One owner, the loop; not safe to read concurrently — the hold adds state under the same rule.

*class, declared at [`include/shulib/hal/pros/imu.hpp:152`](../../include/shulib/hal/pros/imu.hpp#L152).*

<a id="prosimu-krateholdus"></a>

### `ProsImu::kRateHoldUs`

```cpp
static constexpr std::uint32_t kRateHoldUs = 5000
```

How long yawRate() keeps an answer, in WHOLE MICROSECONDS (the robot clock's own resolution, so the 5 ms edge is exact): half of ProsTickPacer::kTickMs, so every read of one 10 ms tick gets the tick's first answer and the next tick's first read is fresh — pinned to the tick by test/imu_rate_hold_pin_test.cpp. It rests on every rate read of a tick falling within 5 ms of the tick's first and on no reader outside the tick (A4 register HA-215): a stall over 5 ms inside a tick, a tick that wakes under 5 ms after a late one, a tick whose first read moves with the IMU's copy between the two positions (HA-214), or a reader on its own rhythm 5-10 ms after a tick's first read each costs a tick or two (header: the three limits), and a loop whose period is not a multiple of 10 ms reads 0 on a large share of its reads (HA-213's 10 ms copy).

*field, declared at [`include/shulib/hal/pros/imu.hpp:164`](../../include/shulib/hal/pros/imu.hpp#L164).*

<a id="prosimu-prosimu"></a>

### `ProsImu::ProsImu`

```cpp
ProsImu(std::uint8_t port, math::Angle bootHeading, const IClock& clock, YawRateSource yawSource = YawRateSource::DifferentiateRotation)
```

`bootHeading`: the robot's canonical field heading AT calibration — ONE owner, the robot's start pose (HA-05). `clock`: read on EVERY yawRate() call, on either source (the one-answer-per-tick hold, IMU1), and by the differentiation path; must outlive this adapter.

*function, declared at [`include/shulib/hal/pros/imu.hpp:171`](../../include/shulib/hal/pros/imu.hpp#L171).*

<a id="prosimu-calibrate"></a>

### `ProsImu::calibrate`

```cpp
void calibrate()
```

Start calibration (reset(false), HA-108). Call ONCE from initialize(); never blocks — the loop waits on isReady() (C1's wait-for-live). A second call is a precondition violation: re-calibrating mid-run re-zeros the sensor under a live bootHeading offset (HA-05's hazard).

*function, declared at [`include/shulib/hal/pros/imu.hpp:183`](../../include/shulib/hal/pros/imu.hpp#L183).*

<a id="prosimu-heading"></a>

### `ProsImu::heading`

```cpp
[[nodiscard]] math::Angle heading() const override
```

Canonical heading via imuHeadingToCanonical (bootHeading applied ONCE, here). Screened reads hold last-good; before any good read this is bootHeading itself (a robot that has not moved).

*function, declared at [`include/shulib/hal/pros/imu.hpp:194`](../../include/shulib/hal/pros/imu.hpp#L194).*

<a id="prosimu-yawrate"></a>

### `ProsImu::yawRate`

```cpp
[[nodiscard]] units::AngularVelocity yawRate() const override
```

Canonical yaw rate (CCW-positive rad/s) from the YawRateSource fixed at construction, ONCE per control tick: a read less than kRateHoldUs (5 ms) after the last FRESH answer returns that answer — no device read, no faultedReads() count, the differentiation sample untouched — so ask as often as you like inside one tick (header: why, and the three limits). DifferentiateRotation (the default) differentiates get_rotation() between FRESH answers, so the interval is however long since the last one rather than a fixed dt: the first call after construction returns 0 (one sample is not a derivative), and so does a read held from it. GyroRateZ instead reads the hardware rate, converted on an UNMEASURED sign assumption (HA-04). Either way a screened read holds last-good, and is itself held for the rest of its tick.

*function, declared at [`include/shulib/hal/pros/imu.hpp:214`](../../include/shulib/hal/pros/imu.hpp#L214).*

<a id="prosimu-isready"></a>

### `ProsImu::isReady`

```cpp
[[nodiscard]] bool isReady() const override
```

True once calibrate() was called and calibration has finished (header: never-calibrated must read not-ready). POSITIVE polarity (imu.hpp:31-33).

*function, declared at [`include/shulib/hal/pros/imu.hpp:231`](../../include/shulib/hal/pros/imu.hpp#L231).*

<a id="prosimu-pitch"></a>

### `ProsImu::pitch`

```cpp
[[nodiscard]] math::Angle pitch() const override
```

Chassis pitch from get_pitch() (degrees, (-180, 180)) as a canonical Angle, UNNEGATED: the as-mounted sign convention is unmeasured (HA-110), so consume the MAGNITUDE until the bench settles it. Nothing reads this yet — the tip detection it exists for is a master-plan M2 item — so the first consumer is also the first chance to bake in a wrong sign. A non-finite read holds the last good value and counts in faultedReads(); before any good read, 0.

*function, declared at [`include/shulib/hal/pros/imu.hpp:241`](../../include/shulib/hal/pros/imu.hpp#L241).*

<a id="prosimu-roll"></a>

### `ProsImu::roll`

```cpp
[[nodiscard]] math::Angle roll() const override
```

Chassis roll from get_roll() (degrees, (-180, 180)) as a canonical Angle, on the same terms as pitch(): UNNEGATED because the mounting sign is unmeasured (HA-110), a non-finite read holds last-good and counts in faultedReads(), and 0 before any good read. Note that roll and pitch each hold their own last-good value, independently.

*function, declared at [`include/shulib/hal/pros/imu.hpp:255`](../../include/shulib/hal/pros/imu.hpp#L255).*

<a id="prosimu-faultedreads"></a>

### `ProsImu::faultedReads`

```cpp
[[nodiscard]] int faultedReads() const noexcept
```

How many reads were screened to last-good (T7 observability). A yawRate() read HELD inside a tick is not a device read and is not counted (IMU1), so a dead sensor counts one rate read per tick, plus its heading / pitch / roll reads.

*function, declared at [`include/shulib/hal/pros/imu.hpp:268`](../../include/shulib/hal/pros/imu.hpp#L268).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 115 lines, click to expand</summary>

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

 ONE ANSWER PER CONTROL TICK (chunk IMU1, 2026-10-29). The library reads the
 rate two or three times a tick — the Localizer; every motion's record when
 the sink wants records (robot two's serial sink wants every one); a GPS or
 tag corrector's gate; the calibration classes — and a team may call
 imu.yawRate() from an auton. On a robot those reads are MICROSECONDS apart
 (ProsClock is micros(), HA-101) while PROS copies the IMU's readings into
 the shared buffer only every 10 ms (the vendored pros/imu.hpp,
 set_data_rate's doc; A4 register HA-213). So before IMU1 a second read in a
 tick differentiated over microseconds — about 0, or a spike of one copy over
 the gap — AND moved the differentiation sample, so the NEXT tick's first
 read (the Localizer's, which TANK1's yaw observer and DriveBrake use) was
 about 3 % high, or 0 on every tick for one band of the copy's phase. That
 is DERIVED from this code and PROS's documented copy and reproduced on the
 host test plant (test/imu_rate_once_per_tick_test.cpp) — never measured on a
 robot. Now yawRate() keeps its answer for kRateHoldUs (5 ms): a read less
 than 5 ms after the last FRESH answer returns that answer and touches
 neither the device, nor the differentiation sample, nor faultedReads(). Ask
 as often as you like INSIDE one tick. Why 5 ms: half the 10 ms control tick
 (ProsTickPacer::kTickMs, hal/pros/tick_pacer.hpp), so every read of one tick
 falls inside it and the next tick's first read, 10 ms on, falls outside —
 pinned to the tick by test/imu_rate_hold_pin_test.cpp; compared in WHOLE
 microseconds, the robot clock's own resolution, so the edge is exact (in
 double seconds it would be decided by rounding). The hold covers BOTH
 sources: a record that differs from the Localizer's value is a lie in the
 evidence column whichever source made it. A held read never extends the
 hold (it runs from the last fresh answer), so a reader that asks every 4 ms
 still gets a fresh answer at least every second read — a hold restarted by
 every read would freeze the rate for ever.

 THE HOLD'S THREE LIMITS — it infers "the same tick" from the CLOCK, not from
 the loop (A4 register HA-215, HA-214; measured on the host test plant only, the
 costs below pinned by IMU1's L1-L3 in test/imu_rate_once_per_tick_test.cpp):
  (1) a stall or a late wake: a tick that stalls more than 5 ms between two
      of its reads gives its later readers a fresh answer, the next tick's
      first read is held from it and the one after reads low — two of the
      Localizer's ticks off; and a tick that wakes 5 ms or more LATE is
      followed by an on-time wake less than 5 ms after it (delay_until keeps
      the 10 ms grid), which gets the late tick's answer, one tick stale.
  (2) the tick's FIRST read moving: the first read keys the tick's answer, so
      when its position changes between ticks — a corrector's gate reads
      first only on a fresh-fix tick; Calibrate's REST capture reads the
      heading before its rate where its settle step did not — and the IMU's
      copy lands between the two positions, the pair of ticks around the
      change reads 0 and about double.
  (3) a reader OUTSIDE the tick: a read from team code between routine
      steps, a screen task, any reader on its own rhythm, 5-10 ms after a
      tick's first read makes the next tick's first read held (stale) and
      the one after skewed — the one case where the hold does worse than the
      adapter before it. And a loop of your own whose period is not a
      multiple of 10 ms reads 0 on a large share of its reads (HA-213's copy):
      differentiate the heading over 10 ms or more yourself, or use the gyro
      once HA-04 and HA-109 are measured.
 (1) and (2) cost a tick or two per cause and do not repeat without it; the
 adapter before IMU1 was off on EVERY tick a second reader read. NOT CURED by
 the hold — A4 register HA-214's residual: when the copy lands within a
 read's jitter of the tick's ONE rate read, that read alternates 0 / about
 double on the affected ticks, with or without a second reader. Its cure is
 the gyro's own rate (GyroRateZ) once the bench measures its sign (HA-04) and
 its units (HA-109).

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

 HA register: HA-02..HA-05, HA-23, HA-108..HA-110, HA-213..HA-215 (IMU1's hold).
```

</details>
