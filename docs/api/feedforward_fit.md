<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/calibration/feedforward_fit.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `feedforward_fit.hpp`

FeedforwardFit — kS, kV and kA of ONE drive side, fitted on the brain from that side's own run.

This header declares **6** types (50 members), **3** free functions, and **2** constants.

Extracted from [`include/shulib/calibration/feedforward_fit.hpp`](../../include/shulib/calibration/feedforward_fit.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kFitRankTolerance`](#kfitranktolerance) — *constant*
- [`enum class FitVerdict`](#enum-class-fitverdict)
  - [`Ok`](#fitverdict-ok)
  - [`TooFewSamples`](#fitverdict-toofewsamples)
  - [`NotExcited`](#fitverdict-notexcited)
  - [`SignMismatch`](#fitverdict-signmismatch)
  - [`NonPhysical`](#fitverdict-nonphysical)
  - [`Implausible`](#fitverdict-implausible)
  - [`PoorFit`](#fitverdict-poorfit)
- [`enum class FitExclusion`](#enum-class-fitexclusion)
  - [`None`](#fitexclusion-none)
  - [`NonFinite`](#fitexclusion-nonfinite)
  - [`OffNominalDt`](#fitexclusion-offnominaldt)
  - [`Stop`](#fitexclusion-stop)
  - [`DeadBand`](#fitexclusion-deadband)
  - [`OverCurrent`](#fitexclusion-overcurrent)
- [`kFitExclusionReasons`](#kfitexclusionreasons) — *constant*
- [`struct FitSample`](#struct-fitsample)
  - [`speedBefore`](#fitsample-speedbefore)
  - [`volts`](#fitsample-volts)
  - [`speedAfter`](#fitsample-speedafter)
  - [`dt`](#fitsample-dt)
  - [`maxCurrent`](#fitsample-maxcurrent)
- [`struct FitConfig`](#struct-fitconfig)
  - [`nominalDt`](#fitconfig-nominaldt)
  - [`dtTolerance`](#fitconfig-dttolerance)
  - [`minSpeed`](#fitconfig-minspeed)
  - [`currentGuard`](#fitconfig-currentguard)
  - [`tauMinFraction`](#fitconfig-tauminfraction)
  - [`minSamples`](#fitconfig-minsamples)
  - [`maxResidualRms`](#fitconfig-maxresidualrms)
  - [`kSMin`](#fitconfig-ksmin)
  - [`kSMax`](#fitconfig-ksmax)
  - [`freeSpeedMin`](#fitconfig-freespeedmin)
  - [`freeSpeedMax`](#fitconfig-freespeedmax)
  - [`kAMin`](#fitconfig-kamin)
  - [`kAMax`](#fitconfig-kamax)
  - [`alphaMin`](#fitconfig-alphamin)
  - [`valid`](#fitconfig-valid)
- [`struct FitResult`](#struct-fitresult)
  - [`verdict`](#fitresult-verdict)
  - [`gains`](#fitresult-gains)
  - [`kABelowResolution`](#fitresult-kabelowresolution)
  - [`samples`](#fitresult-samples)
  - [`excluded`](#fitresult-excluded)
  - [`residualRms`](#fitresult-residualrms)
  - [`alpha`](#fitresult-alpha)
  - [`beta`](#fitresult-beta)
  - [`gamma`](#fitresult-gamma)
  - [`voltsMin`](#fitresult-voltsmin)
  - [`voltsMax`](#fitresult-voltsmax)
- [`gainsPlausible`](#gainsplausible) — *free function*
- [`class FeedforwardFit`](#class-feedforwardfit)
  - [`FeedforwardFit`](#feedforwardfit-feedforwardfit)
  - [`add`](#feedforwardfit-add)
  - [`result`](#feedforwardfit-result)
  - [`admitted`](#feedforwardfit-admitted)
  - [`excluded`](#feedforwardfit-excluded)
  - [`config`](#feedforwardfit-config)
- [`fitVerdictName`](#fitverdictname) — *free function*
- [`fitExclusionName`](#fitexclusionname) — *free function*

<a id="kfitranktolerance"></a>

## `kFitRankTolerance`

```cpp
inline constexpr double kFitRankTolerance = 1e-9
```

The relative size under which a column of the fit carries no information of its own: R's diagonal entry for the column (the part the columns before it cannot explain) at or under this fraction of the column's own 2-norm is NotExcited. A NUMERICAL constant, not a guess about a robot: exactly collinear data leaves ~1e-15 here after rounding, and the characterizer's plant runs leave ~1e-2 (a ramp at kA > 0 without its steps, ~1e-3) — 1e-9 sits many decades from both.

*constant, declared at [`include/shulib/calibration/feedforward_fit.hpp:136`](../../include/shulib/calibration/feedforward_fit.hpp#L136).*

<a id="enum-class-fitverdict"></a>

## `enum class FitVerdict`

```cpp
enum class FitVerdict : std::uint8_t
```

What a fit concluded. APPEND-ONLY: the values are persisted and printed by name (fitVerdictName()); a new verdict goes at the end.

*enum class, declared at [`include/shulib/calibration/feedforward_fit.hpp:140`](../../include/shulib/calibration/feedforward_fit.hpp#L140).*

<a id="fitverdict-ok"></a>

### `FitVerdict::Ok`

```cpp
Ok = 0
```

the gains below are the fit's, inside every range

*enumerator, declared at [`include/shulib/calibration/feedforward_fit.hpp:141`](../../include/shulib/calibration/feedforward_fit.hpp#L141).*

<a id="fitverdict-toofewsamples"></a>

### `FitVerdict::TooFewSamples`

```cpp
TooFewSamples = 1
```

fewer admitted samples than FitConfig::minSamples

*enumerator, declared at [`include/shulib/calibration/feedforward_fit.hpp:142`](../../include/shulib/calibration/feedforward_fit.hpp#L142).*

<a id="fitverdict-notexcited"></a>

### `FitVerdict::NotExcited`

```cpp
NotExcited = 2
```

a column carries no information of its own (header: one level, a ramp alone at kA = 0)

*enumerator, declared at [`include/shulib/calibration/feedforward_fit.hpp:143`](../../include/shulib/calibration/feedforward_fit.hpp#L143).*

<a id="fitverdict-signmismatch"></a>

### `FitVerdict::SignMismatch`

```cpp
SignMismatch = 3
```

β <= 0: more volts made the side slower — the command's and the reading's signs disagree

*enumerator, declared at [`include/shulib/calibration/feedforward_fit.hpp:144`](../../include/shulib/calibration/feedforward_fit.hpp#L144).*

<a id="fitverdict-nonphysical"></a>

### `FitVerdict::NonPhysical`

```cpp
NonPhysical = 4
```

α >= 1 (the law's decay would be growth) or kS < 0

*enumerator, declared at [`include/shulib/calibration/feedforward_fit.hpp:145`](../../include/shulib/calibration/feedforward_fit.hpp#L145).*

<a id="fitverdict-implausible"></a>

### `FitVerdict::Implausible`

```cpp
Implausible = 5
```

the gains are outside FitConfig's ranges (kS, the implied free speed, kA)

*enumerator, declared at [`include/shulib/calibration/feedforward_fit.hpp:146`](../../include/shulib/calibration/feedforward_fit.hpp#L146).*

<a id="fitverdict-poorfit"></a>

### `FitVerdict::PoorFit`

```cpp
PoorFit = 6
```

the residual RMS is over FitConfig::maxResidualRms — the law does not describe the data

*enumerator, declared at [`include/shulib/calibration/feedforward_fit.hpp:147`](../../include/shulib/calibration/feedforward_fit.hpp#L147).*

<a id="enum-class-fitexclusion"></a>

## `enum class FitExclusion`

```cpp
enum class FitExclusion : std::uint8_t
```

Why a sample was NOT admitted, or None when it was. The order is the order add() judges in.

*enum class, declared at [`include/shulib/calibration/feedforward_fit.hpp:151`](../../include/shulib/calibration/feedforward_fit.hpp#L151).*

<a id="fitexclusion-none"></a>

### `FitExclusion::None`

```cpp
None = 0
```

admitted

*enumerator, declared at [`include/shulib/calibration/feedforward_fit.hpp:152`](../../include/shulib/calibration/feedforward_fit.hpp#L152).*

<a id="fitexclusion-nonfinite"></a>

### `FitExclusion::NonFinite`

```cpp
NonFinite = 1
```

a field that is not a number

*enumerator, declared at [`include/shulib/calibration/feedforward_fit.hpp:153`](../../include/shulib/calibration/feedforward_fit.hpp#L153).*

<a id="fitexclusion-offnominaldt"></a>

### `FitExclusion::OffNominalDt`

```cpp
OffNominalDt = 2
```

dt outside nominalDt·(1 ± dtTolerance)

*enumerator, declared at [`include/shulib/calibration/feedforward_fit.hpp:154`](../../include/shulib/calibration/feedforward_fit.hpp#L154).*

<a id="fitexclusion-stop"></a>

### `FitExclusion::Stop`

```cpp
Stop = 3
```

u = 0 exactly — a segment's stop (header: two reasons)

*enumerator, declared at [`include/shulib/calibration/feedforward_fit.hpp:155`](../../include/shulib/calibration/feedforward_fit.hpp#L155).*

<a id="fitexclusion-deadband"></a>

### `FitExclusion::DeadBand`

```cpp
DeadBand = 4
```

|v| before or after under minSpeed

*enumerator, declared at [`include/shulib/calibration/feedforward_fit.hpp:156`](../../include/shulib/calibration/feedforward_fit.hpp#L156).*

<a id="fitexclusion-overcurrent"></a>

### `FitExclusion::OverCurrent`

```cpp
OverCurrent = 5
```

a member at or over currentGuard

*enumerator, declared at [`include/shulib/calibration/feedforward_fit.hpp:157`](../../include/shulib/calibration/feedforward_fit.hpp#L157).*

<a id="kfitexclusionreasons"></a>

## `kFitExclusionReasons`

```cpp
inline constexpr std::size_t kFitExclusionReasons = 5
```

How many exclusion reasons there are (FitExclusion's values past None).

*constant, declared at [`include/shulib/calibration/feedforward_fit.hpp:161`](../../include/shulib/calibration/feedforward_fit.hpp#L161).*

<a id="struct-fitsample"></a>

## `struct FitSample`

```cpp
struct FitSample
```

One tick of one side, as the fit reads it: the side's speed at a tick, the volts commanded AT that tick, the side's speed at the NEXT tick, the time between the two readings, and the largest member current read at the next tick. Speeds are the side's median surface speed in in/s (the characterizer's reading: median over members of sign × velocity() × inches per radian).

*struct, declared at [`include/shulib/calibration/feedforward_fit.hpp:167`](../../include/shulib/calibration/feedforward_fit.hpp#L167).*

<a id="fitsample-speedbefore"></a>

### `FitSample::speedBefore`

```cpp
double speedBefore = 0.0
```

v[k], in/s

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:168`](../../include/shulib/calibration/feedforward_fit.hpp#L168).*

<a id="fitsample-volts"></a>

### `FitSample::volts`

```cpp
double volts = 0.0
```

u[k], volts, as commanded (sign included)

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:169`](../../include/shulib/calibration/feedforward_fit.hpp#L169).*

<a id="fitsample-speedafter"></a>

### `FitSample::speedAfter`

```cpp
double speedAfter = 0.0
```

v[k+1], in/s

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:170`](../../include/shulib/calibration/feedforward_fit.hpp#L170).*

<a id="fitsample-dt"></a>

### `FitSample::dt`

```cpp
double dt = 0.0
```

seconds from the v[k] reading to the v[k+1] reading

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:171`](../../include/shulib/calibration/feedforward_fit.hpp#L171).*

<a id="fitsample-maxcurrent"></a>

### `FitSample::maxCurrent`

```cpp
double maxCurrent = 0.0
```

amperes: the largest |current()| of any member, read with v[k+1]

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:172`](../../include/shulib/calibration/feedforward_fit.hpp#L172).*

<a id="struct-fitconfig"></a>

## `struct FitConfig`

```cpp
struct FitConfig
```

Every number the fit judges with. NOT ONE IS A MEASUREMENT — all are INVENTED for a first run and carried as fields so an S2 serial log can move them with a logged line. PROVISIONAL, every one (A4: HA-170).

*struct, declared at [`include/shulib/calibration/feedforward_fit.hpp:178`](../../include/shulib/calibration/feedforward_fit.hpp#L178).*

<a id="fitconfig-nominaldt"></a>

### `FitConfig::nominalDt`

```cpp
double nominalDt = 0.01
```

The tick the fitted α belongs to, seconds: shulib's 10 ms loop (HA-32). PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:180`](../../include/shulib/calibration/feedforward_fit.hpp#L180).*

<a id="fitconfig-dttolerance"></a>

### `FitConfig::dtTolerance`

```cpp
double dtTolerance = 0.10
```

A sample's dt must be within nominalDt·(1 ± this). PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:182`](../../include/shulib/calibration/feedforward_fit.hpp#L182).*

<a id="fitconfig-minspeed"></a>

### `FitConfig::minSpeed`

```cpp
double minSpeed = 1.0
```

|speed| under this (in/s) is the static-friction dead band. PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:184`](../../include/shulib/calibration/feedforward_fit.hpp#L184).*

<a id="fitconfig-currentguard"></a>

### `FitConfig::currentGuard`

```cpp
double currentGuard = 2.3
```

A member current at or over this (A) is the current limit: 2.3 A, under the PROS default limit of 2.5 A (HA-172). PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:187`](../../include/shulib/calibration/feedforward_fit.hpp#L187).*

<a id="fitconfig-tauminfraction"></a>

### `FitConfig::tauMinFraction`

```cpp
double tauMinFraction = 0.2
```

τMin as a fraction of nominalDt: a time constant under τMin is below the fit's resolution and kA is reported as exactly 0 (header). 0.2 = dt/5. PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:190`](../../include/shulib/calibration/feedforward_fit.hpp#L190).*

<a id="fitconfig-minsamples"></a>

### `FitConfig::minSamples`

```cpp
std::uint32_t minSamples = 100
```

Fewer admitted samples than this is TooFewSamples. PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:192`](../../include/shulib/calibration/feedforward_fit.hpp#L192).*

<a id="fitconfig-maxresidualrms"></a>

### `FitConfig::maxResidualRms`

```cpp
double maxResidualRms = 3.0
```

A residual RMS (in/s) over this is PoorFit. PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:194`](../../include/shulib/calibration/feedforward_fit.hpp#L194).*

<a id="fitconfig-ksmin"></a>

### `FitConfig::kSMin`

```cpp
double kSMin = 0.0
```

kS below this (V) is Implausible (below 0 it is NonPhysical first). PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:196`](../../include/shulib/calibration/feedforward_fit.hpp#L196).*

<a id="fitconfig-ksmax"></a>

### `FitConfig::kSMax`

```cpp
double kSMax = 4.0
```

kS above this (V) is Implausible. PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:198`](../../include/shulib/calibration/feedforward_fit.hpp#L198).*

<a id="fitconfig-freespeedmin"></a>

### `FitConfig::freeSpeedMin`

```cpp
double freeSpeedMin = 20.0
```

The implied free speed at 12 V, (12 − kS)/kV in in/s, below this is Implausible. PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:201`](../../include/shulib/calibration/feedforward_fit.hpp#L201).*

<a id="fitconfig-freespeedmax"></a>

### `FitConfig::freeSpeedMax`

```cpp
double freeSpeedMax = 200.0
```

The implied free speed above this (in/s) is Implausible. PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:203`](../../include/shulib/calibration/feedforward_fit.hpp#L203).*

<a id="fitconfig-kamin"></a>

### `FitConfig::kAMin`

```cpp
double kAMin = 0.0
```

kA below this (V·s²/in) is Implausible. PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:205`](../../include/shulib/calibration/feedforward_fit.hpp#L205).*

<a id="fitconfig-kamax"></a>

### `FitConfig::kAMax`

```cpp
double kAMax = 0.2
```

kA above this (V·s²/in) is Implausible. PROVISIONAL (A4: HA-170).

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:207`](../../include/shulib/calibration/feedforward_fit.hpp#L207).*

<a id="fitconfig-alphamin"></a>

### `FitConfig::alphaMin`

```cpp
[[nodiscard]] double alphaMin() const noexcept
```

The α under which kA is below resolution: e^(−1/tauMinFraction) (header).

*function, declared at [`include/shulib/calibration/feedforward_fit.hpp:210`](../../include/shulib/calibration/feedforward_fit.hpp#L210).*

<a id="fitconfig-valid"></a>

### `FitConfig::valid`

```cpp
[[nodiscard]] bool valid(char* why, std::size_t cap) const
```

True when every field is finite and in its domain: nominalDt, minSpeed, currentGuard, tauMinFraction and maxResidualRms > 0; dtTolerance in (0, 1); minSamples >= 3 (three unknowns); each range's low end at or under its high end. Otherwise false, with the first offending field named in `why` (NUL-terminated, cut to `cap`; `why` may be null).

*function, declared at [`include/shulib/calibration/feedforward_fit.hpp:216`](../../include/shulib/calibration/feedforward_fit.hpp#L216).*

<a id="struct-fitresult"></a>

## `struct FitResult`

```cpp
struct FitResult
```

Everything a fit concluded — filled as far as the verdict let it get, so a refused fit still shows its numbers on the serial line.

*struct, declared at [`include/shulib/calibration/feedforward_fit.hpp:259`](../../include/shulib/calibration/feedforward_fit.hpp#L259).*

<a id="fitresult-verdict"></a>

### `FitResult::verdict`

```cpp
FitVerdict verdict = FitVerdict::TooFewSamples
```

The verdict (FitVerdict). Only Ok means the gains may be used.

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:261`](../../include/shulib/calibration/feedforward_fit.hpp#L261).*

<a id="fitresult-gains"></a>

### `FitResult::gains`

```cpp
control::FeedforwardGains gains{}
```

kS (V), kV (V·s/in), kA (V·s²/in) — the fit's, when it got as far as computing them (every verdict from SignMismatch on; zeros before). kA is RECORDED, NOT APPLIED (header).

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:264`](../../include/shulib/calibration/feedforward_fit.hpp#L264).*

<a id="fitresult-kabelowresolution"></a>

### `FitResult::kABelowResolution`

```cpp
bool kABelowResolution = false
```

True when α was under FitConfig::alphaMin() and kA is therefore reported as exactly 0.

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:266`](../../include/shulib/calibration/feedforward_fit.hpp#L266).*

<a id="fitresult-samples"></a>

### `FitResult::samples`

```cpp
std::uint32_t samples = 0
```

How many samples were admitted.

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:268`](../../include/shulib/calibration/feedforward_fit.hpp#L268).*

<a id="fitresult-excluded"></a>

### `FitResult::excluded`

```cpp
std::array<std::uint32_t, kFitExclusionReasons> excluded{}
```

How many were excluded, by reason: index = FitExclusion value − 1.

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:270`](../../include/shulib/calibration/feedforward_fit.hpp#L270).*

<a id="fitresult-residualrms"></a>

### `FitResult::residualRms`

```cpp
double residualRms = 0.0
```

The root-mean-square of the admitted samples' residuals, in/s (divisor: the sample count).

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:272`](../../include/shulib/calibration/feedforward_fit.hpp#L272).*

<a id="fitresult-alpha"></a>

### `FitResult::alpha`

```cpp
double alpha = 0.0
```

The fitted α (0 until a solution exists).

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:274`](../../include/shulib/calibration/feedforward_fit.hpp#L274).*

<a id="fitresult-beta"></a>

### `FitResult::beta`

```cpp
double beta = 0.0
```

The fitted β.

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:276`](../../include/shulib/calibration/feedforward_fit.hpp#L276).*

<a id="fitresult-gamma"></a>

### `FitResult::gamma`

```cpp
double gamma = 0.0
```

The fitted γ.

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:278`](../../include/shulib/calibration/feedforward_fit.hpp#L278).*

<a id="fitresult-voltsmin"></a>

### `FitResult::voltsMin`

```cpp
double voltsMin = 0.0
```

The smallest |volts| of any admitted sample — the low end of the range the gains were fitted over (0 with none admitted).

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:281`](../../include/shulib/calibration/feedforward_fit.hpp#L281).*

<a id="fitresult-voltsmax"></a>

### `FitResult::voltsMax`

```cpp
double voltsMax = 0.0
```

The largest |volts| of any admitted sample — the high end of that range (HA-171: the gains must hold outside it).

*field, declared at [`include/shulib/calibration/feedforward_fit.hpp:284`](../../include/shulib/calibration/feedforward_fit.hpp#L284).*

<a id="gainsplausible"></a>

## `gainsPlausible`

```cpp
[[nodiscard]] inline bool gainsPlausible(const control::FeedforwardGains& gains, const FitConfig& cfg, char* why, std::size_t cap)
```

Is `gains` inside FitConfig's ranges — kS in [kSMin, kSMax], the implied free speed at 12 V (hal::kMaxMotorVoltage − kS)/kV in [freeSpeedMin, freeSpeedMax], kA in [kAMin, kAMax]? Every edge is INSIDE. False names the first range missed in `why` (NUL-terminated, cut to `cap`; may be null), with the value — ASCII, no '%'. Precondition: gains.kV > 0 (the fit's verdicts before this one guarantee it).

*free function, declared at [`include/shulib/calibration/feedforward_fit.hpp:292`](../../include/shulib/calibration/feedforward_fit.hpp#L292).*

<a id="class-feedforwardfit"></a>

## `class FeedforwardFit`

```cpp
class FeedforwardFit
```

The online least-squares fit of ONE drive side (header). Feed add() once per tick; read result() whenever — it does not consume anything. Copyable (it is plain numbers); reset() by assigning a new one.

*class, declared at [`include/shulib/calibration/feedforward_fit.hpp:317`](../../include/shulib/calibration/feedforward_fit.hpp#L317).*

<a id="feedforwardfit-feedforwardfit"></a>

### `FeedforwardFit::FeedforwardFit`

```cpp
explicit FeedforwardFit(const FitConfig& config = {})
```

Copies `config`. Precondition: config.valid().

*function, declared at [`include/shulib/calibration/feedforward_fit.hpp:320`](../../include/shulib/calibration/feedforward_fit.hpp#L320).*

<a id="feedforwardfit-add"></a>

### `FeedforwardFit::add`

```cpp
FitExclusion add(const FitSample& s)
```

Judge one sample and, when it is admitted, rotate it into the fit. Returns FitExclusion::None when admitted, else the reason it was not (and counts it). Never throws on a bad sample: a non-finite one is an exclusion, not a precondition, because it comes off a sensor.

*function, declared at [`include/shulib/calibration/feedforward_fit.hpp:327`](../../include/shulib/calibration/feedforward_fit.hpp#L327).*

<a id="feedforwardfit-result"></a>

### `FeedforwardFit::result`

```cpp
[[nodiscard]] FitResult result() const
```

The fit as it stands (header: the verdicts in order). Pure — callable any number of times.

*function, declared at [`include/shulib/calibration/feedforward_fit.hpp:372`](../../include/shulib/calibration/feedforward_fit.hpp#L372).*

<a id="feedforwardfit-admitted"></a>

### `FeedforwardFit::admitted`

```cpp
[[nodiscard]] std::uint32_t admitted() const noexcept
```

How many samples have been admitted.

*function, declared at [`include/shulib/calibration/feedforward_fit.hpp:437`](../../include/shulib/calibration/feedforward_fit.hpp#L437).*

<a id="feedforwardfit-excluded"></a>

### `FeedforwardFit::excluded`

```cpp
[[nodiscard]] std::uint32_t excluded(FitExclusion reason) const noexcept
```

How many samples were excluded for `reason` (0 for FitExclusion::None).

*function, declared at [`include/shulib/calibration/feedforward_fit.hpp:440`](../../include/shulib/calibration/feedforward_fit.hpp#L440).*

<a id="feedforwardfit-config"></a>

### `FeedforwardFit::config`

```cpp
[[nodiscard]] const FitConfig& config() const noexcept
```

The config in force.

*function, declared at [`include/shulib/calibration/feedforward_fit.hpp:445`](../../include/shulib/calibration/feedforward_fit.hpp#L445).*

<a id="fitverdictname"></a>

## `fitVerdictName`

```cpp
[[nodiscard]] constexpr const char* fitVerdictName(FitVerdict v) noexcept
```

A short, stable, ASCII name for a verdict — the serial record's word and a test's message.

*free function, declared at [`include/shulib/calibration/feedforward_fit.hpp:481`](../../include/shulib/calibration/feedforward_fit.hpp#L481).*

<a id="fitexclusionname"></a>

## `fitExclusionName`

```cpp
[[nodiscard]] constexpr const char* fitExclusionName(FitExclusion e) noexcept
```

A short, stable, ASCII name for an exclusion reason — a column of the per-segment serial line.

*free function, declared at [`include/shulib/calibration/feedforward_fit.hpp:495`](../../include/shulib/calibration/feedforward_fit.hpp#L495).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 115 lines, click to expand</summary>

```text

 FeedforwardFit — kS, kV and kA of ONE drive side, fitted on the brain from that side's own run
 (chunk R5a Part A, 2026-09-29; the team lead's Q7 ruling of 2026-09-26: gains are measured by the
 robot and kept on the SD card, never typed or committed as constants).

 ── PROVEN ON THE HOST TEST PLANT AND ON SYNTHETIC DATA; IT HAS NEVER FITTED A ROBOT ─────
 Every property below is shown on data the test generates or on the A2 host test plant — whose
 motor model IS the law fitted here (sim/motor_model.hpp's honesty boundary), so recovering the
 plant's gains proves the ARITHMETIC, not that a V5 drive side follows this law (A4 register
 HA-171). The model-mismatch cases in feedforward_fit_test.cpp exist for exactly that reason, and
 what they found is recorded rather than smoothed over: PoorFit is NOT a general mismatch detector
 (see "What PoorFit does not catch" below). Every number in FitConfig is INVENTED (HA-170).

 ── The model: the DISCRETE form of the feedforward law ────────────────────────────
 control::Feedforward commands V = kS·sgn(v) + kV·v + kA·a. Read as the first-order ODE
 kA·v̇ = u − kS·sgn(v) − kV·v and held at one voltage u for one tick of length dt, its exact
 solution is

     v[k+1] = α·v[k] + β·u[k] + γ·sgn(v[k]),   α = e^(−dt·kV/kA),
                                               β = (1 − α)/kV,
                                               γ = −(1 − α)·kS/kV,

 which is LINEAR in (α, β, γ). So the fit is a linear least-squares problem over three columns —
 the speed read at a tick, the volts commanded at that tick, the sign of that speed — against the
 speed read at the next tick, and the gains come back as

     kV = (1 − α)/β,    kS = −γ/β,    kA = −kV·dt / ln α.

 NO DIFFERENTIATED VELOCITY IS USED ANYWHERE. The obvious alternative — regress u against v and a
 numerically differentiated a — divides the noisiest number on the robot (an encoder's rate) by
 10 ms. The discrete form needs only two consecutive speed readings. It is also EXACTLY the update
 the A2 plant integrates (motor_model.hpp: v_ss + (v − v_ss)·e^(−dt/τ)), which is why the host can
 check the arithmetic to 1e-9 and why that check says nothing about physics.

 ── Numerics: incremental QR by Givens rotations, never the normal equations ──────────
 Each admitted sample is rotated into a 3×3 upper-triangular R and its right-hand side as it
 arrives (the sequential form of Householder/Givens least squares — Gentleman 1973), and the part
 of the sample the three columns cannot explain is squared into the residual sum on the spot. The
 normal equations — accumulate XᵀX and Xᵀy, solve, RSS = yᵀy − θᵀXᵀy — are REJECTED: over one run
 Σv² reaches ~10⁷ (in/s)², and the residual sum is then the difference of two numbers ~10⁹ times
 larger than itself, which leaves nothing of it in a double. The test forces the choice without
 dictating it: 10⁵ samples at ~80 in/s with a ~1e-3 in/s residual must report the residual RMS of a
 two-pass reference to 1e-9 relative (a normal-equations build of this class fails it — run as a
 mutation). Allocation-free and O(1) per sample: three rotations of a four-element row.

 ── Identifiability: why the STEP segments exist (R5a's re-check R2 and decision D10) ──────────────
 On a ramp, once the start transient has decayed, v[k] is an affine function of u[k] (the
 feedforward law is linear), so the three columns are COLLINEAR: at kA = 0 exactly — the fit says
 NotExcited — and at kA > 0 about 40× worse conditioned. A held STEP breaks the collinearity: the
 speed then climbs at constant volts. drive_characterizer.hpp therefore runs two step segments
 after the two ramps. "One voltage level only" is NotExcited too (the volts column is then the sign
 column times a constant). NotExcited is judged on R's diagonal: a column whose part NOT explained
 by the columns before it is under kRankTolerance of its own norm carries no information of its own.

 ── Admission: which samples the law describes, each exclusion counted by reason ─────────
   * NonFinite     a speed, volts, dt or current that is not a number. Never on a healthy robot;
                   refused rather than let one NaN poison R for the rest of the run.
   * OffNominalDt  dt outside nominalDt·(1 ± dtTolerance). α depends on dt; one α is fitted, so a
                   late tick is a sample of a DIFFERENT law.
   * Stop          u = 0 exactly: a segment's stop. For TWO reasons, and the exclusion stays if
                   either goes away (R5a's landmine 5): on a V5 a motor at 0 V under Coast may
                   free-wheel (A4 register HA-143) and a Brake stop is braking — the law describes
                   neither; and on the host test plant the friction term follows the STEADY-STATE
                   sign (motor_model.hpp), so at u = 0 a moving wheel decays with NO friction term at
                   all — admitting those samples moves the plant's kS from 1.00 to 0.53 (the
                   re-check's probe; a mutation here goes red on the plant).
   * DeadBand      |v[k]| or |v[k+1]| under minSpeed: static friction, where sgn(v) is not the law's
                   and a stuck wheel reads 0 whatever u is.
   * OverCurrent   any member of the side at or over currentGuard (read at the tick that closes the
                   sample): the motor is at its current limit, the one non-linearity the ramp's range
                   is believed to contain (A4 register HA-172), where torque — not the law — sets the
                   acceleration.
 What admission CANNOT exclude, because it does not know kS in advance: a sample with 0 < |u| <= kS
 and |v| >= minSpeed also violates the law on the plant (its friction follows the steady-state
 sign, which is 0 inside the dead band). The characterizer's segments never produce one — every
 segment starts from rest and never powers a deceleration. Do not add a powered deceleration
 without re-reading this.

 ── Verdicts, in the order they are judged ──────────────────────────────────────────
 TooFewSamples (under minSamples) → NotExcited (rank, above) → SignMismatch (β <= 0: more volts made
 the side SLOWER, so the command and the reading disagree in sign — the sign convention broke, rule
 1 of drivetrain_calibrator.hpp) → NonPhysical (α >= 1 — the law's decay would be growth — or kS < 0)
 → PoorFit (residual RMS over maxResidualRms) → Implausible (outside FitConfig's ranges) → Ok.
 PoorFit is judged BEFORE the ranges on purpose: when the law does not describe the data its
 numbers are meaningless, and "kS out of range" would send a person after the wrong thing.
 FitVerdict is append-only — its values are persisted and printed by name.

 ── kA below resolution is NOT an error ─────────────────────────────────────────────
 When the drive responds within one tick, α is ~0 and ln α is useless: a kA = 0 plant fits α to
 ±1e-13, whose positive side would report kA ≈ 5·10⁻⁵ and whose negative side a NaN. So when
 α < alphaMin = e^(−1/tauMinFraction) — a time constant under tauMinFraction of a tick (τMin = dt/5)
 — kA is reported as EXACTLY 0 with kABelowResolution set. That covers α <= 0 too.

 ── What PoorFit does not catch (A4 register HA-171) ────────────────────────────────
 PoorFit judges the residual RMS against maxResidualRms and nothing else. A mismatch whose residual
 stays under that bound is FITTED — a quadratic drag up to the size feedforward_fit_test.cpp's drag
 sweep pins, a velocity() that is low-pass filtered by the firmware, a one-tick command latency,
 and NOISE ON THE SPEED READING — and each leaves a kS/kV/kA bias that test pins and HA-171
 records. The noise is the one every robot has: a noisy speed is noise in a REGRESSOR (errors in
 variables), which pulls α toward 0 — kA reads LOW and kS HIGH — while the residual it leaves is
 only ~1.3·σ, so a side-median noise of σ up to ~2.2 in/s is still Ok. Over 40 seeds on the four
 segments' exact law (M7): σ 0.5 in/s → kS +3 %, kV +1 %, kA −18 % on average; σ 1 in/s → kS
 +11 %, kV +1 %, kA −46 %; σ 2 in/s → kS +6 %, kV +5 %, kA −72 % (single runs up to kS +33 %,
 kV +11 %); and when the current guard removes the step's current-limited opening, which carries
 most of the excitation, the bias grows three- to fourfold (σ 0.5 in/s, a 100 in/s² limit:
 kS +14 % against +3 %, kA −56 % against −18 %). kS and kV are applied; the REST step's encoder speed σ at rest is a floor on σ. The fit
 reports the RESIDUAL so a person can judge; it does not claim the law holds.

 kA IS FITTED, RECORDED AND NOT APPLIED: the command pipeline calls the cruise overload
 Feedforward::calculate(v) (command_pipeline.hpp step 7) and WheelSpeeds carries no acceleration, so
 no measured kA reaches a motor (R5a's decisions D2 and D10; A4 register HA-45).

 Tick-free and clock-free: a caller (drive_characterizer.hpp) hands in one sample per tick. PROS-free,
 sim-free, allocation-free, raises nothing, and every string it returns is ASCII with no '%' (a
 panel may paint it: PROS re-reads screen text as a format).
```

</details>
