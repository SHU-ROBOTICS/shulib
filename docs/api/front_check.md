<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/calibration/front_check.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `front_check.hpp`

front_check — did a new calibration put the robot's FRONT at the other end?

This header declares **2** types (11 members) and **2** free functions.

Extracted from [`include/shulib/calibration/front_check.hpp`](../../include/shulib/calibration/front_check.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class FrontVerdict`](#enum-class-frontverdict)
  - [`Same`](#frontverdict-same)
  - [`Flipped`](#frontverdict-flipped)
  - [`Undetermined`](#frontverdict-undetermined)
- [`struct FrontCheck`](#struct-frontcheck)
  - [`verdict`](#frontcheck-verdict)
  - [`shared`](#frontcheck-shared)
  - [`agree`](#frontcheck-agree)
  - [`flipped`](#frontcheck-flipped)
  - [`signOnly`](#frontcheck-signonly)
  - [`sideOnly`](#frontcheck-sideonly)
  - [`onlyEarlier`](#frontcheck-onlyearlier)
  - [`onlyNow`](#frontcheck-onlynow)
- [`frontVerdictWord`](#frontverdictword) — *free function*
- [`compareFronts`](#comparefronts) — *free function*

<a id="enum-class-frontverdict"></a>

## `enum class FrontVerdict`

```cpp
enum class FrontVerdict : std::uint8_t
```

What compareFronts() concluded.

*enum class, declared at [`include/shulib/calibration/front_check.hpp:54`](../../include/shulib/calibration/front_check.hpp#L54).*

<a id="frontverdict-same"></a>

### `FrontVerdict::Same`

```cpp
Same
```

a strict majority of the shared ports agree by side and sign

*enumerator, declared at [`include/shulib/calibration/front_check.hpp:55`](../../include/shulib/calibration/front_check.hpp#L55).*

<a id="frontverdict-flipped"></a>

### `FrontVerdict::Flipped`

```cpp
Flipped
```

a strict majority sit on the other side with the other sign

*enumerator, declared at [`include/shulib/calibration/front_check.hpp:56`](../../include/shulib/calibration/front_check.hpp#L56).*

<a id="frontverdict-undetermined"></a>

### `FrontVerdict::Undetermined`

```cpp
Undetermined
```

no shared drive port, or no majority (a tie included)

*enumerator, declared at [`include/shulib/calibration/front_check.hpp:57`](../../include/shulib/calibration/front_check.hpp#L57).*

<a id="struct-frontcheck"></a>

## `struct FrontCheck`

```cpp
struct FrontCheck
```

The comparison, port by port (header). The four kinds of shared port add up to `shared`.

*struct, declared at [`include/shulib/calibration/front_check.hpp:61`](../../include/shulib/calibration/front_check.hpp#L61).*

<a id="frontcheck-verdict"></a>

### `FrontCheck::verdict`

```cpp
FrontVerdict verdict = FrontVerdict::Undetermined
```

the decision

*field, declared at [`include/shulib/calibration/front_check.hpp:62`](../../include/shulib/calibration/front_check.hpp#L62).*

<a id="frontcheck-shared"></a>

### `FrontCheck::shared`

```cpp
std::uint8_t shared = 0
```

drive ports in both descriptions

*field, declared at [`include/shulib/calibration/front_check.hpp:63`](../../include/shulib/calibration/front_check.hpp#L63).*

<a id="frontcheck-agree"></a>

### `FrontCheck::agree`

```cpp
std::uint8_t agree = 0
```

same side, same sign

*field, declared at [`include/shulib/calibration/front_check.hpp:64`](../../include/shulib/calibration/front_check.hpp#L64).*

<a id="frontcheck-flipped"></a>

### `FrontCheck::flipped`

```cpp
std::uint8_t flipped = 0
```

other side, other sign

*field, declared at [`include/shulib/calibration/front_check.hpp:65`](../../include/shulib/calibration/front_check.hpp#L65).*

<a id="frontcheck-signonly"></a>

### `FrontCheck::signOnly`

```cpp
std::uint8_t signOnly = 0
```

same side, other sign

*field, declared at [`include/shulib/calibration/front_check.hpp:66`](../../include/shulib/calibration/front_check.hpp#L66).*

<a id="frontcheck-sideonly"></a>

### `FrontCheck::sideOnly`

```cpp
std::uint8_t sideOnly = 0
```

other side, same sign

*field, declared at [`include/shulib/calibration/front_check.hpp:67`](../../include/shulib/calibration/front_check.hpp#L67).*

<a id="frontcheck-onlyearlier"></a>

### `FrontCheck::onlyEarlier`

```cpp
std::uint8_t onlyEarlier = 0
```

in the earlier description only

*field, declared at [`include/shulib/calibration/front_check.hpp:68`](../../include/shulib/calibration/front_check.hpp#L68).*

<a id="frontcheck-onlynow"></a>

### `FrontCheck::onlyNow`

```cpp
std::uint8_t onlyNow = 0
```

in the new description only

*field, declared at [`include/shulib/calibration/front_check.hpp:69`](../../include/shulib/calibration/front_check.hpp#L69).*

<a id="frontverdictword"></a>

## `frontVerdictWord`

```cpp
[[nodiscard]] constexpr const char* frontVerdictWord(FrontVerdict v) noexcept
```

The words for a verdict, as the serial log and the screen say them.

*free function, declared at [`include/shulib/calibration/front_check.hpp:73`](../../include/shulib/calibration/front_check.hpp#L73).*

<a id="comparefronts"></a>

## `compareFronts`

```cpp
[[nodiscard]] constexpr FrontCheck compareFronts(const hal::DrivetrainDescription& earlier, const hal::DrivetrainDescription& now) noexcept
```

Compare the drive ports of `earlier` (the card's record, or — with no usable record — the typed drive ports) and `now` (this run's measurement) — header. Ports 1..kMaxSmartPorts are compared; an entry outside that range is not a drive port to this check.

*free function, declared at [`include/shulib/calibration/front_check.hpp:95`](../../include/shulib/calibration/front_check.hpp#L95).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 43 lines</summary>

```text

 front_check — did a new calibration put the robot's FRONT at the other end? (chunk CAL1,
 Part A1, 2026-10-08.)

 THE RULE (the team lead, 2026-10-06): the end that LEADS Calibrate's push is the front, and the
 front is never typed. A calibration record carries no "front" field — it carries each drive
 port's SIDE and SIGN, both relative to the push. So a recalibration pushed with the OTHER end
 leading gives EVERY port the other sign AND the other side (measured on the host test plant
 through today's "shulib Calibrate": LEFT -11 +12 -13 +14 -15 | RIGHT +16 -17 +18 -19 +20 became
 LEFT -16 +17 -18 +19 -20 | RIGHT +11 -12 +13 -14 +15). Every auton written for the old front
 would then drive backwards, and nothing downstream could tell. compareFronts() is the check.

 THE VERDICT, port by port, over the drive ports BOTH descriptions list (matched by |port|):
   agree    — same side, same sign: the same front;
   flipped  — other side AND other sign: the front moved to the other end;
   signOnly — same side, other sign: that motor was re-mounted or re-wired (not a front change);
   sideOnly — other side, same sign: a cable moved across (not a front change).
 FLIPPED when the flipped ports are a STRICT MAJORITY of the shared ones; SAME when the agreeing
 ones are; otherwise UNDETERMINED (no shared port, or no majority — a tie included) — never a
 guess. A port in only one of the two (a motor LEFT OUT, a cable moved to a new port) decides
 nothing and is counted. Signs alone would call a re-mounted motor a flip; sides alone would
 call a moved cable one; so both, per port.

 WHAT THE RECORDS CANNOT TELL: the LEFT and RIGHT drive cables swapped between the sides at the
 brain, with the right end leading, give the SAME record as the other end leading (on a
 mirror-mounted tank every swapped pair is mirror-signed — robot two's record, worked port by
 port; the host test holds the two records equal). So FLIPPED means "the earlier description
 calls the end that led this push its BACK", never "you pushed the wrong end" — only the
 operator knows which. A program that checks the front therefore saves nothing until the
 operator says (save as pushed, or run again), and it never turns a measurement to the old front:
 what is saved is always what was pushed.

 WHAT `earlier` IS: the card's calibration record (as the boot would choose it) — or, when the
 card holds no usable record, a description holding only the drive ports the team TYPED for its
 drive program (decided by the team lead, 2026-10-08). compareFronts() reads nothing but the two
 sides' signed ports, so either works unchanged: a typed table's entries carry each port's side
 and sign in the same convention as a record (the boot copies them unchanged into a
 description's sides), and "SAME as the typed ports" means the robot, driven from those ports,
 turns every motor the way this push measured. Neither description need be valid() — a typed
 one has no geometry — but a side whose count runs past hal::kMaxSmartPorts is not a side this
 check can read, and gives UNDETERMINED with every count 0 (it never reads past the array).

 PROS-free, allocation-free, constexpr, host-tested. NOTHING HERE HAS RUN ON A ROBOT.
```

</details>
