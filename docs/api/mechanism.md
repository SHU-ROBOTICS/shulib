<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/mechanism.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `mechanism.hpp`

The mechanism device seam (chunk F1, WS7/M4): IMechanism + the two concrete compositions every VEX mechanism reduces to at the device level — a group of motors on one shaft (MotorMechanism) and a set of digital lines switching one pneumatic circuit (Pneumat…

This header declares **4** types (44 members).

Extracted from [`include/shulib/hal/mechanism.hpp`](../../include/shulib/hal/mechanism.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ICancellable`](#class-icancellable)
  - [`~ICancellable`](#icancellable-destructor-icancellable)
  - [`ICancellable`](#icancellable-icancellable)
  - [`ICancellable (overload 2)`](#icancellable-icancellable-2)
  - [`ICancellable (overload 3)`](#icancellable-icancellable-3)
  - [`operator=`](#icancellable-operator-eq)
  - [`operator= (overload 2)`](#icancellable-operator-eq-2)
  - [`cancel`](#icancellable-cancel)
- [`class IMechanism`](#class-imechanism)
  - [`~IMechanism`](#imechanism-destructor-imechanism)
  - [`IMechanism`](#imechanism-imechanism)
  - [`IMechanism (overload 2)`](#imechanism-imechanism-2)
  - [`IMechanism (overload 3)`](#imechanism-imechanism-3)
  - [`operator=`](#imechanism-operator-eq)
  - [`operator= (overload 2)`](#imechanism-operator-eq-2)
  - [`applySafeState`](#imechanism-applysafestate)
  - [`name`](#imechanism-name)
  - [`tryClaim`](#imechanism-tryclaim)
  - [`tryClaim (overload 2)`](#imechanism-tryclaim-2)
  - [`releaseClaim`](#imechanism-releaseclaim)
  - [`claimed`](#imechanism-claimed)
  - [`claimant`](#imechanism-claimant)
- [`class MotorMechanism`](#class-motormechanism)
  - [`MotorMechanism`](#motormechanism-motormechanism)
  - [`setVoltage`](#motormechanism-setvoltage)
  - [`commandedVoltage`](#motormechanism-commandedvoltage)
  - [`applySafeState`](#motormechanism-applysafestate)
  - [`name`](#motormechanism-name)
  - [`safeBrakeMode`](#motormechanism-safebrakemode)
  - [`maxCurrent`](#motormechanism-maxcurrent)
  - [`meanVelocity`](#motormechanism-meanvelocity)
  - [`meanPosition`](#motormechanism-meanposition)
  - [`motors`](#motormechanism-motors)
- [`class PneumaticMechanism`](#class-pneumaticmechanism)
  - [`PneumaticMechanism`](#pneumaticmechanism-pneumaticmechanism)
  - [`kMaxPerLineLevels`](#pneumaticmechanism-kmaxperlinelevels)
  - [`PneumaticMechanism (overload 2)`](#pneumaticmechanism-pneumaticmechanism-2)
  - [`PneumaticMechanism (overload 3)`](#pneumaticmechanism-pneumaticmechanism-3)
  - [`set`](#pneumaticmechanism-set)
  - [`setLevels`](#pneumaticmechanism-setlevels)
  - [`commandedLevel`](#pneumaticmechanism-commandedlevel)
  - [`commanded`](#pneumaticmechanism-commanded)
  - [`applySafeState`](#pneumaticmechanism-applysafestate)
  - [`name`](#pneumaticmechanism-name)
  - [`safeCommand`](#pneumaticmechanism-safecommand)
  - [`perLine`](#pneumaticmechanism-perline)
  - [`safeLevel`](#pneumaticmechanism-safelevel)
  - [`lines`](#pneumaticmechanism-lines)

<a id="class-icancellable"></a>

## `class ICancellable`

```cpp
class ICancellable
```

The hal-level face of "whatever is currently driving a mechanism" — exactly the ONE capability an end-of-run guard needs from a claimant it knows nothing about: stop, synchronously, into the mechanism's declared safe state, and become inert (further ticks are no-ops). Declared HERE, below the manipulation layer, so IMechanism's claim token can carry it without an upward include; manipulation::IMechanismOp implements it (its cancel() contract is already exactly this). See the file banner's claimant-hook section for the measured failure this closes.

*class, declared at [`include/shulib/hal/mechanism.hpp:101`](../../include/shulib/hal/mechanism.hpp#L101).*

<a id="icancellable-destructor-icancellable"></a>

### `ICancellable::~ICancellable`

```cpp
virtual ~ICancellable() = default
```

Re-declared only because the virtual destructor suppresses the implicit copy/move members; this seam holds no state of its own. The virtual destructor is what makes deleting through a stored ICancellable* well-defined. Note that a claimant is registered BY ADDRESS (IMechanism::tryClaim below), so an implementer that holds a claim should DELETE its own copy/move instead of inheriting these defaults — a copy would leave the mechanism's registration aimed at the original. Manipulation's operations do exactly that.

*function, declared at [`include/shulib/hal/mechanism.hpp:110`](../../include/shulib/hal/mechanism.hpp#L110).*

<a id="icancellable-icancellable"></a>

### `ICancellable::ICancellable`

```cpp
ICancellable() = default
```

*Covered by the comment on [`~ICancellable`](#icancellable-destructor-icancellable) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/mechanism.hpp:111`](../../include/shulib/hal/mechanism.hpp#L111).*

<a id="icancellable-icancellable-2"></a>

### `ICancellable::ICancellable (overload 2)`

```cpp
ICancellable(const ICancellable&) = default
```

*Covered by the comment on [`~ICancellable`](#icancellable-destructor-icancellable) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/mechanism.hpp:112`](../../include/shulib/hal/mechanism.hpp#L112).*

<a id="icancellable-icancellable-3"></a>

### `ICancellable::ICancellable (overload 3)`

```cpp
ICancellable(ICancellable&&) = default
```

*Covered by the comment on [`~ICancellable`](#icancellable-destructor-icancellable) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/mechanism.hpp:113`](../../include/shulib/hal/mechanism.hpp#L113).*

<a id="icancellable-operator-eq"></a>

### `ICancellable::operator=`

```cpp
ICancellable& operator=(const ICancellable&) = default
```

*Covered by the comment on [`~ICancellable`](#icancellable-destructor-icancellable) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/mechanism.hpp:114`](../../include/shulib/hal/mechanism.hpp#L114).*

<a id="icancellable-operator-eq-2"></a>

### `ICancellable::operator= (overload 2)`

```cpp
ICancellable& operator=(ICancellable&&) = default
```

*Covered by the comment on [`~ICancellable`](#icancellable-destructor-icancellable) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/mechanism.hpp:115`](../../include/shulib/hal/mechanism.hpp#L115).*

<a id="icancellable-cancel"></a>

### `ICancellable::cancel`

```cpp
virtual void cancel() = 0
```

Render the claimant inert and its mechanism safe, now. Idempotent; never raises (the IMechanismOp cancel contract).

*function, declared at [`include/shulib/hal/mechanism.hpp:119`](../../include/shulib/hal/mechanism.hpp#L119).*

<a id="class-imechanism"></a>

## `class IMechanism`

```cpp
class IMechanism
```

The minimal common surface of every mechanism: a declared safe state that can be forced from outside, a stable name for logs, and the one-operation claim token. See the file banner for why nothing else is unified.

*class, declared at [`include/shulib/hal/mechanism.hpp:125`](../../include/shulib/hal/mechanism.hpp#L125).*

<a id="imechanism-destructor-imechanism"></a>

### `IMechanism::~IMechanism`

```cpp
virtual ~IMechanism() = default
```

NON-COPYABLE AND NON-MOVABLE, and unlike the other HAL seams that is about state rather than style: this base HOLDS the claim token. While copy/move were defaulted, a copied mechanism arrived already claimed(), with claimant() aimed at an operation registered against the ORIGINAL — so a legitimate tryClaim(copy) failed for no reason the caller could see, and F2's end-of-run guard walking a span containing the copy reached claimant() and cancelled an operation driving the original, whose own claim was never released. That is exactly the unreleased-claim failure the claimant hook exists to close, reintroduced by a defaulted special member.  manipulation/mechanism_op.hpp already deletes copy/move on both operations for the mirror-image reason ("the claim is a resource and the mechanism's registered claimant points at THIS object"); the mechanism side simply never got the same treatment. Construct a mechanism once where it lives and hand out IMechanism&/IMechanism*.

*function, declared at [`include/shulib/hal/mechanism.hpp:140`](../../include/shulib/hal/mechanism.hpp#L140).*

<a id="imechanism-imechanism"></a>

### `IMechanism::IMechanism`

```cpp
IMechanism() = default
```

*Covered by the comment on [`~IMechanism`](#imechanism-destructor-imechanism) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/mechanism.hpp:141`](../../include/shulib/hal/mechanism.hpp#L141).*

<a id="imechanism-imechanism-2"></a>

### `IMechanism::IMechanism (overload 2)`

```cpp
IMechanism(const IMechanism&) = delete
```

*Covered by the comment on [`~IMechanism`](#imechanism-destructor-imechanism) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/mechanism.hpp:142`](../../include/shulib/hal/mechanism.hpp#L142).*

<a id="imechanism-imechanism-3"></a>

### `IMechanism::IMechanism (overload 3)`

```cpp
IMechanism(IMechanism&&) = delete
```

*Covered by the comment on [`~IMechanism`](#imechanism-destructor-imechanism) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/mechanism.hpp:143`](../../include/shulib/hal/mechanism.hpp#L143).*

<a id="imechanism-operator-eq"></a>

### `IMechanism::operator=`

```cpp
IMechanism& operator=(const IMechanism&) = delete
```

*Covered by the comment on [`~IMechanism`](#imechanism-destructor-imechanism) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/mechanism.hpp:144`](../../include/shulib/hal/mechanism.hpp#L144).*

<a id="imechanism-operator-eq-2"></a>

### `IMechanism::operator= (overload 2)`

```cpp
IMechanism& operator=(IMechanism&&) = delete
```

*Covered by the comment on [`~IMechanism`](#imechanism-destructor-imechanism) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/mechanism.hpp:145`](../../include/shulib/hal/mechanism.hpp#L145).*

<a id="imechanism-applysafestate"></a>

### `IMechanism::applySafeState`

```cpp
virtual void applySafeState() = 0
```

Command the DECLARED safe state, synchronously — safe when the call returns, no further tick required (the same synchronous rule as the scheduler's cancel path: a safe state that depends on someone continuing to tick can leave things energized). Idempotent; callable at any time, including while an operation is running (F2's park guard does exactly that — it does not ask permission at the buzzer).

*function, declared at [`include/shulib/hal/mechanism.hpp:153`](../../include/shulib/hal/mechanism.hpp#L153).*

<a id="imechanism-name"></a>

### `IMechanism::name`

```cpp
[[nodiscard]] virtual const char* name() const noexcept = 0
```

Stable short name for logs / fault details (a stable literal — stored, not copied, like Routine's name).

*function, declared at [`include/shulib/hal/mechanism.hpp:157`](../../include/shulib/hal/mechanism.hpp#L157).*

<a id="imechanism-tryclaim"></a>

### `IMechanism::tryClaim`

```cpp
[[nodiscard]] bool tryClaim() noexcept
```

Take the claim ANONYMOUSLY. False if another operation already holds it. An anonymous claim is invisible to F2's end-of-run cancel-all (banner: the claimant hook) — prefer the registering overload.

*function, declared at [`include/shulib/hal/mechanism.hpp:165`](../../include/shulib/hal/mechanism.hpp#L165).*

<a id="imechanism-tryclaim-2"></a>

### `IMechanism::tryClaim (overload 2)`

```cpp
[[nodiscard]] bool tryClaim(ICancellable& claimant) noexcept
```

Take the claim AND register the claimant, so an end-of-run guard holding only IMechanism* can reach the operation and cancel it (chunk F2). `claimant` must stay valid until the claim is released — every operation exit path releases, and since F2 the library operations also cancel-on-destruction, so a registered pointer cannot dangle.

*function, declared at [`include/shulib/hal/mechanism.hpp:178`](../../include/shulib/hal/mechanism.hpp#L178).*

<a id="imechanism-releaseclaim"></a>

### `IMechanism::releaseClaim`

```cpp
void releaseClaim() noexcept
```

Release the claim (no-op if not held — release is always safe).

*function, declared at [`include/shulib/hal/mechanism.hpp:187`](../../include/shulib/hal/mechanism.hpp#L187).*

<a id="imechanism-claimed"></a>

### `IMechanism::claimed`

```cpp
[[nodiscard]] bool claimed() const noexcept
```

True while an operation holds the claim.

*function, declared at [`include/shulib/hal/mechanism.hpp:193`](../../include/shulib/hal/mechanism.hpp#L193).*

<a id="imechanism-claimant"></a>

### `IMechanism::claimant`

```cpp
[[nodiscard]] ICancellable* claimant() const noexcept
```

The registered claimant, or nullptr (unclaimed, or claimed anonymously via the parameterless tryClaim). The end-of-run guard's reach.

*function, declared at [`include/shulib/hal/mechanism.hpp:197`](../../include/shulib/hal/mechanism.hpp#L197).*

<a id="class-motormechanism"></a>

## `class MotorMechanism`

```cpp
class MotorMechanism : public IMechanism
```

N motors on one mechanically coupled shaft (an intake's two motors, a lift's pair), commanded as one. The SAME voltage goes to every motor: direction reversal is a device-level fact (the pros adapter owns it, exactly as it owns mA→A), so by the time a motor reaches this seam "+V" already means "forward" for that motor.

*class, declared at [`include/shulib/hal/mechanism.hpp:209`](../../include/shulib/hal/mechanism.hpp#L209).*

<a id="motormechanism-motormechanism"></a>

### `MotorMechanism::MotorMechanism`

```cpp
MotorMechanism(std::span<IMotor* const> motors, BrakeMode safe, const char* mechName)
```

`motors` (non-empty, all non-null) must outlive the mechanism; `safe` is the DECLARED safe brake mode (banner: Hold for a loaded lift, Coast or Brake for an intake — there is no correct default, so there is no default). `mechName` must be a stable literal.

*function, declared at [`include/shulib/hal/mechanism.hpp:215`](../../include/shulib/hal/mechanism.hpp#L215).*

<a id="motormechanism-setvoltage"></a>

### `MotorMechanism::setVoltage`

```cpp
void setVoltage(units::Voltage volts)
```

Command every motor (clamped/validated by the IMotor contract).

*function, declared at [`include/shulib/hal/mechanism.hpp:225`](../../include/shulib/hal/mechanism.hpp#L225).*

<a id="motormechanism-commandedvoltage"></a>

### `MotorMechanism::commandedVoltage`

```cpp
[[nodiscard]] units::Voltage commandedVoltage() const
```

The voltage the DEVICE actually got, read back from the first motor — never this object's own record of what it thinks it commanded (the bottom-of-the-stack rule every F1 test also follows). All motors are commanded identically through this seam, so one readback speaks for the group.

*function, declared at [`include/shulib/hal/mechanism.hpp:236`](../../include/shulib/hal/mechanism.hpp#L236).*

<a id="motormechanism-applysafestate"></a>

### `MotorMechanism::applySafeState`

```cpp
void applySafeState() override
```

The declared safe state: safe brake mode on every motor, THEN zero volts, so the stop lands under the declared semantics and never a momentary coast — the same ordering applyCancelSafeState() documents.

*function, declared at [`include/shulib/hal/mechanism.hpp:243`](../../include/shulib/hal/mechanism.hpp#L243).*

<a id="motormechanism-name"></a>

### `MotorMechanism::name`

```cpp
[[nodiscard]] const char* name() const noexcept override
```

The `mechName` pointer given at construction, returned verbatim — this class BORROWS the string and never copies it, so the literal must outlive the mechanism.

*function, declared at [`include/shulib/hal/mechanism.hpp:252`](../../include/shulib/hal/mechanism.hpp#L252).*

<a id="motormechanism-safebrakemode"></a>

### `MotorMechanism::safeBrakeMode`

```cpp
[[nodiscard]] BrakeMode safeBrakeMode() const noexcept
```

The declared safe brake mode (construction-time fact, for tests/logs).

*function, declared at [`include/shulib/hal/mechanism.hpp:255`](../../include/shulib/hal/mechanism.hpp#L255).*

<a id="motormechanism-maxcurrent"></a>

### `MotorMechanism::maxCurrent`

```cpp
[[nodiscard]] units::Current maxCurrent() const
```

Highest per-motor current draw — the jam/stall signal (motor.hpp calls current() "the PRIMARY capture/stall signal for manipulation sensor-confirm"). Max, not mean: a jam shows on the most loaded motor.

*function, declared at [`include/shulib/hal/mechanism.hpp:260`](../../include/shulib/hal/mechanism.hpp#L260).*

<a id="motormechanism-meanvelocity"></a>

### `MotorMechanism::meanVelocity`

```cpp
[[nodiscard]] units::AngularVelocity meanVelocity() const
```

Mean output-shaft angular velocity across the group (one coupled shaft, so the mean IS the shaft; signed, so a direction fact survives).

*function, declared at [`include/shulib/hal/mechanism.hpp:270`](../../include/shulib/hal/mechanism.hpp#L270).*

<a id="motormechanism-meanposition"></a>

### `MotorMechanism::meanPosition`

```cpp
[[nodiscard]] units::AngleDim meanPosition() const
```

Mean output-shaft ANGLE across the group, radians, CUMULATIVE (each motor's position() never wraps — motor.hpp). One coupled shaft, so the mean IS the shaft, exactly as meanVelocity() argues; each motor's own direction is already applied by its adapter (the signed port), so a mirrored motor reads the same way as its mate. The position reader a position controller needs (the mechanism core, MECH-A1; read by MECH-A3's behaviours).

*function, declared at [`include/shulib/hal/mechanism.hpp:283`](../../include/shulib/hal/mechanism.hpp#L283).*

<a id="motormechanism-motors"></a>

### `MotorMechanism::motors`

```cpp
[[nodiscard]] std::span<IMotor* const> motors() const noexcept
```

The devices themselves — for readers this grammar does not cover (per-motor position for F3's liftToLevel homing, temperatures). Handing out the seam rather than wrapping every reader keeps this class honest about what it is: a command fan-out with a declared safe state.

*function, declared at [`include/shulib/hal/mechanism.hpp:295`](../../include/shulib/hal/mechanism.hpp#L295).*

<a id="class-pneumaticmechanism"></a>

## `class PneumaticMechanism`

```cpp
class PneumaticMechanism : public IMechanism
```

One pneumatic circuit behind N digital lines (a clamp's solenoid, a pair of deploy cylinders fired together), commanded as one. The declared safe value is per-mechanism for the same reason the brake mode is (T4): whether "safe at the buzzer" means clamp-closed (keep the goal) or cylinder-retracted (inside expansion limits) is a fact about the robot, not about the library. Since the mechanism core (MECH-A1, 2026-10-08) it has a second, PER-LINE form as well: one declared safe level per line and `setLevels()`, for a mechanism whose named positions are different level patterns across its lines (a three-position hood on two). The fan-out form and its constructor are unchanged; the two forms differ only in what applySafeState() drives.

*class, declared at [`include/shulib/hal/mechanism.hpp:313`](../../include/shulib/hal/mechanism.hpp#L313).*

<a id="pneumaticmechanism-pneumaticmechanism"></a>

### `PneumaticMechanism::PneumaticMechanism`

```cpp
PneumaticMechanism(std::span<IDigitalOut* const> lines, bool safe, const char* mechName)
```

`lines` (non-empty, all non-null) must outlive the mechanism; `safe` is the DECLARED safe command. `mechName` must be a stable literal.

*function, declared at [`include/shulib/hal/mechanism.hpp:317`](../../include/shulib/hal/mechanism.hpp#L317).*

<a id="pneumaticmechanism-kmaxperlinelevels"></a>

### `PneumaticMechanism::kMaxPerLineLevels`

```cpp
static constexpr std::size_t kMaxPerLineLevels = 4
```

The most lines the PER-LINE form below stores a safe level for (fixed storage; the robot config allows two per mechanism, config::kMaxPneumaticsPerMechanism).

*field, declared at [`include/shulib/hal/mechanism.hpp:328`](../../include/shulib/hal/mechanism.hpp#L328).*

<a id="pneumaticmechanism-pneumaticmechanism-2"></a>

### `PneumaticMechanism::PneumaticMechanism (overload 2)`

```cpp
PneumaticMechanism(std::span<IDigitalOut* const> lines, std::span<const bool> safeLevels, const char* mechName)
```

The PER-LINE form (the mechanism core, MECH-A1): each line has its OWN declared safe level, `safeLevels[i]` for `lines[i]`. A mechanism whose positions need different levels on different lines — a three-position hood on two lines: down = low/low, middle = high/low, up = high/high — then has a declared safe POSITION that applySafeState() applies, so the end-of-run guard needs no knowledge of the mechanism. The single-level constructor above is unchanged and is what a mechanism whose safe position is one level on every line uses. PRECONDITIONS: `lines` non-empty, all non-null, at most kMaxPerLineLevels; one safe level per line; `mechName` non-null and a stable literal.

*function, declared at [`include/shulib/hal/mechanism.hpp:338`](../../include/shulib/hal/mechanism.hpp#L338).*

<a id="pneumaticmechanism-pneumaticmechanism-3"></a>

### `PneumaticMechanism::PneumaticMechanism (overload 3)`

```cpp
template <std::size_t N> PneumaticMechanism(std::span<IDigitalOut* const> lines, const bool (&safeLevels)[N], const char* mechName)
```

The per-line form from a C ARRAY of safe levels — `const bool safe[2]{true, false}; PneumaticMechanism hood{lines, safe, "hood"};`. Needed, not sugar: without it an array argument DECAYS to a pointer and converts to `bool` (a standard conversion, which overload resolution prefers to the span's user-defined one), so the call would silently pick the single-level constructor with `safe = true` (found by MECH-A1's own test, whose braces turned it into a narrowing error). An array binds here exactly and forwards to the span form.

*function, declared at [`include/shulib/hal/mechanism.hpp:362`](../../include/shulib/hal/mechanism.hpp#L362).*

<a id="pneumaticmechanism-set"></a>

### `PneumaticMechanism::set`

```cpp
void set(bool value)
```

Command every line.

*function, declared at [`include/shulib/hal/mechanism.hpp:366`](../../include/shulib/hal/mechanism.hpp#L366).*

<a id="pneumaticmechanism-setlevels"></a>

### `PneumaticMechanism::setLevels`

```cpp
void setLevels(std::span<const bool> levels)
```

Command EACH line its own level — `levels[i]` to line i, in the construction order (the mechanism core's positions, MECH-A1). PRECONDITION: one level per line.

*function, declared at [`include/shulib/hal/mechanism.hpp:374`](../../include/shulib/hal/mechanism.hpp#L374).*

<a id="pneumaticmechanism-commandedlevel"></a>

### `PneumaticMechanism::commandedLevel`

```cpp
[[nodiscard]] bool commandedLevel(std::size_t i) const
```

What line `i` was last commanded — its DEVICE readback, never this object's record (the bottom-of-stack rule commanded() follows). PRECONDITION: `i` is below the line count.

*function, declared at [`include/shulib/hal/mechanism.hpp:384`](../../include/shulib/hal/mechanism.hpp#L384).*

<a id="pneumaticmechanism-commanded"></a>

### `PneumaticMechanism::commanded`

```cpp
[[nodiscard]] bool commanded() const
```

The command the DEVICE actually got (first line's readback — the same bottom-of-stack rule as MotorMechanism::commandedVoltage). Remember what this is NOT (digital_out.hpp): evidence that anything moved.

*function, declared at [`include/shulib/hal/mechanism.hpp:392`](../../include/shulib/hal/mechanism.hpp#L392).*

<a id="pneumaticmechanism-applysafestate"></a>

### `PneumaticMechanism::applySafeState`

```cpp
void applySafeState() override
```

The declared safe state: every line driven to `safe`. ONE command, not the motor version's brake-then-zero two-step — a solenoid has no coast phase to slip through. Still only a command: nothing here is evidence the air actually moved. The PER-LINE form drives each line to ITS declared safe level instead — the declared safe POSITION.

*function, declared at [`include/shulib/hal/mechanism.hpp:398`](../../include/shulib/hal/mechanism.hpp#L398).*

<a id="pneumaticmechanism-name"></a>

### `PneumaticMechanism::name`

```cpp
[[nodiscard]] const char* name() const noexcept override
```

The `mechName` pointer given at construction, returned verbatim — BORROWED, never copied, so the literal must outlive the mechanism.

*function, declared at [`include/shulib/hal/mechanism.hpp:410`](../../include/shulib/hal/mechanism.hpp#L410).*

<a id="pneumaticmechanism-safecommand"></a>

### `PneumaticMechanism::safeCommand`

```cpp
[[nodiscard]] bool safeCommand() const noexcept
```

The declared safe command (construction-time fact, for tests/logs). For the PER-LINE form, line 0's safe level — read safeLevel(i) for the others.

*function, declared at [`include/shulib/hal/mechanism.hpp:414`](../../include/shulib/hal/mechanism.hpp#L414).*

<a id="pneumaticmechanism-perline"></a>

### `PneumaticMechanism::perLine`

```cpp
[[nodiscard]] bool perLine() const noexcept
```

Whether this mechanism was built in the PER-LINE form (one declared safe level per line).

*function, declared at [`include/shulib/hal/mechanism.hpp:417`](../../include/shulib/hal/mechanism.hpp#L417).*

<a id="pneumaticmechanism-safelevel"></a>

### `PneumaticMechanism::safeLevel`

```cpp
[[nodiscard]] bool safeLevel(std::size_t i) const
```

Line `i`'s declared safe level: its own in the per-line form, the one safe command otherwise. PRECONDITION: `i` is below the line count.

*function, declared at [`include/shulib/hal/mechanism.hpp:421`](../../include/shulib/hal/mechanism.hpp#L421).*

<a id="pneumaticmechanism-lines"></a>

### `PneumaticMechanism::lines`

```cpp
[[nodiscard]] std::span<IDigitalOut* const> lines() const noexcept
```

The lines themselves, in the construction order — for anything this fan-out grammar does not cover (driving one cylinder of a pair alone on a bench check). NON-OWNING, like the span it was built from: the caller still owns every line.

*function, declared at [`include/shulib/hal/mechanism.hpp:429`](../../include/shulib/hal/mechanism.hpp#L429).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 79 lines, click to expand</summary>

```text

 The mechanism device seam (chunk F1, WS7/M4): IMechanism + the two concrete
 compositions every VEX mechanism reduces to at the device level — a group of
 motors on one shaft (MotorMechanism) and a set of digital lines switching one
 pneumatic circuit (PneumaticMechanism).

 ── What the interface is, and why it is this small ─────────────────────────────────
 IMechanism's virtual surface is exactly two members: applySafeState() and
 name(). That is deliberate, and it is the answer to "does the interface earn
 its existence": the ONE operation every mechanism supports uniformly —
 regardless of whether it is motors or air — is "put yourself in your declared
 safe state, NOW, synchronously". F2's guaranteed end-of-run park guard must be
 able to walk a heterogeneous list of mechanisms it knows nothing about and
 force every one of them safe at the buzzer; that requires a common base and
 nothing else does. Command-and-read surfaces are NOT unified here because the
 physics is not unified: a voltage command on a solenoid would be a lie, and a
 "command a double" abstraction over both would be a worse one. Code that
 commands a mechanism holds the concrete type; code that only needs "make it
 safe" (F2's park guard, a panic stop) holds IMechanism*.

 Rejected alternative — a fat IMechanism with setCommand(double)/read():
 it adds a vtable and a document without adding a capability (the anti-
 abstraction test in the F1 brief), and it bakes the motor shape into the seam
 the pneumatic clamp then has to fit through.

 ── The declared safe state (T4) ────────────────────────────────────────────────────
 The drivetrain's cancel safe state (0 V + Brake, motion.hpp) is defined once
 for ALL drive motors because a drivetrain is one thing. Mechanisms are not:
   * a loaded lift at 0 V + Coast DROPS ITS STACK — its safe state is Hold;
   * a jammed intake commanded Hold sits at stall current until the thermal
     fault fires (~55 °C, motor.hpp) — its safe state is Coast or Brake.
 So there is NO library-wide default that is safe for both, and this header
 refuses to pick one: the safe state is DECLARED, per mechanism, at
 construction, and applySafeState() applies that declaration. Every stop path
 in the manipulation layer (operation exit, cancel, failure) and F2's park
 guard land here, so the declaration is applied by construction, not by
 convention. Whether BrakeMode::Hold actually holds a LOADED cascade lift is a
 hardware claim no host test can verify: PROVISIONAL (A4: HA-92). (Until
 DEFECTS2, 2026-09-26, the declaration did not even reach a V5: the PROS
 adapter sent the zero as 0 V, which ignores the mode. It now sends it as
 brake() — hal/pros/motor.hpp, THE STOP, A4 register HA-142.)

 ── The operation claim (T3) ────────────────────────────────────────────────────────
 Two mechanisms running at once is required (intake while the lift settles).
 Two OPERATIONS driving ONE mechanism is a collision — the same argument that
 made one-active-motion structural at C2. The claim token below makes it
 structural here: an operation's start() takes the claim or trips a loud
 precondition; every operation exit releases it. It is a plain flag, not a
 mutex — single-task by contract like everything in this library. Pre-empt-
 then-replace (C2's policy) is deliberately NOT built in at this level: a
 sequencing layer that wants pre-emption cancels the old operation first
 (cancel releases the claim), which keeps the policy where the policy-owner
 lives (F2) and keeps a silent double-drive impossible everywhere.

 ── The claimant hook (chunk F2 — the gap its measurements exposed) ─────────────────
 F1 promised the end-of-run guard a span<IMechanism*> it could force safe.
 Building that guard found the promise short by one capability: the claim
 said THAT a mechanism was driven but not BY WHAT, so a stalled operation
 was unreachable from the guard — and applySafeState() alone lasts exactly
 until the live operation's next tick re-commands its voltage (measured:
 the re-command restores voltage but not brake mode, leaving the half-safe
 `brake=Hold, V=9.0` that passes any mode-only assertion). Worse, the
 unreleased claim makes the END ACTION's own operation throw at start().
 So the claim now carries an optional ICancellable: an operation that
 registers itself is reachable — the guard cancels it (inert + safe +
 claim released) instead of merely repainting the device state it will
 overwrite. tryClaim() without a claimant stays legal (F1 tests, third-party
 ops) but is INVISIBLE to the guard's cancel-all, which can then only
 force-release the claim and warn; register a claimant if an end-of-run
 guard must be able to stop your operation.

 ── Portability ─────────────────────────────────────────────────────────────────────
 The concrete compositions are written over the L0 seams (IMotor*/IDigitalOut*),
 so they run unchanged on hal/fake (host tests), hal/pros (R1 implements the
 devices, not the mechanisms) and hal/sim (H2). What a mechanism MEANS — which
 motors, which safe state, what confirms an action — stays with the team that
 built the robot; this file owns only the device grammar. No game semantics
 here, per the hal/vision.hpp house rule (classId is an opaque int for the
 same reason).
```

</details>
