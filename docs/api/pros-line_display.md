<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/line_display.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `line_display.hpp`

ProsLineDisplay — ILineDisplay over the V5 controller's LCD (chunk R1a): where the D-4 status rows physically go.

This header declares **2** types (4 members).

Extracted from [`include/shulib/hal/pros/line_display.hpp`](../../include/shulib/hal/pros/line_display.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class DisplayController`](#enum-class-displaycontroller)
  - [`Master`](#displaycontroller-master)
  - [`Partner`](#displaycontroller-partner)
- [`class ProsLineDisplay`](#class-proslinedisplay)
  - [`ProsLineDisplay`](#proslinedisplay-proslinedisplay)
  - [`setLine`](#proslinedisplay-setline)

<a id="enum-class-displaycontroller"></a>

## `enum class DisplayController`

```cpp
enum class DisplayController
```

Which physical controller's LCD this display writes (mirrors the IController adapter's id — see controller.hpp).

*enum class, declared at [`include/shulib/hal/pros/line_display.hpp:55`](../../include/shulib/hal/pros/line_display.hpp#L55).*

<a id="displaycontroller-master"></a>

### `DisplayController::Master`

```cpp
Master
```

The driver's own controller (::pros::E_CONTROLLER_MASTER).

*enumerator, declared at [`include/shulib/hal/pros/line_display.hpp:56`](../../include/shulib/hal/pros/line_display.hpp#L56).*

<a id="displaycontroller-partner"></a>

### `DisplayController::Partner`

```cpp
Partner
```

The second controller tethered to it (::pros::E_CONTROLLER_PARTNER).

*enumerator, declared at [`include/shulib/hal/pros/line_display.hpp:57`](../../include/shulib/hal/pros/line_display.hpp#L57).*

<a id="class-proslinedisplay"></a>

## `class ProsLineDisplay`

```cpp
class ProsLineDisplay final : public ILineDisplay
```

Where the status rows actually land: ILineDisplay driven by pros::Controller::set_text. One setLine() is exactly one device write — the adapter does no pacing, no batching and no change-detection, because the firmware's write rate limit is a CONTENT-layer problem (ControllerFaultDisplay repaints only rows that changed) and a device adapter that silently withheld writes would be unfalsifiable. What it does add is the two things the seam promises and set_text does not give for free: truncation at ILineDisplay::kCols (never a wrap, which would corrupt the row below) and space-padding out to kCols, so a short line is a true overwrite instead of leaving the previous row's tail visible. A failed write — controller unplugged — is dropped, never thrown; the telemetry log already carries the same text. kCols itself is still unverified against real firmware (see the header: the vendored SDK doc and community practice disagree, 15 vs 19).

*class, declared at [`include/shulib/hal/pros/line_display.hpp:71`](../../include/shulib/hal/pros/line_display.hpp#L71).*

<a id="proslinedisplay-proslinedisplay"></a>

### `ProsLineDisplay::ProsLineDisplay`

```cpp
explicit ProsLineDisplay(DisplayController which = DisplayController::Master)
```

Opens this adapter's OWN pros::Controller handle on `which` (master by default). That handle is a thin wrapper keyed by controller id, so holding one here while the IController adapter holds another on the same id is two views of ONE device, not a second device — no coordination is required and none is performed. Constructing touches no hardware and cannot fail: an absent controller shows up as dropped writes in setLine(), not as an error here.

*function, declared at [`include/shulib/hal/pros/line_display.hpp:79`](../../include/shulib/hal/pros/line_display.hpp#L79).*

<a id="proslinedisplay-setline"></a>

### `ProsLineDisplay::setLine`

```cpp
void setLine(int row, std::string_view text) override
```

Overwrite `row` with `text`: truncated at kCols (NEVER wrapped), padded with spaces to kCols (a true overwrite — header). MUST NOT throw.

*function, declared at [`include/shulib/hal/pros/line_display.hpp:85`](../../include/shulib/hal/pros/line_display.hpp#L85).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 36 lines</summary>

```text

 ProsLineDisplay — ILineDisplay over the V5 controller's LCD (chunk R1a):
 where the D-4 status rows physically go.

 BINDS: pros::Controller::set_text(row, 0, text). The adapter owns its own
 pros::Controller handle — that class is a thin wrapper over the C API keyed
 by controller id, so a second handle on the same id (the IController
 adapter's) is two views of one device, not two devices.

 GEOMETRY — TRUNCATE AT kCols, NEVER WRAP (line_display.hpp:20-21): a
 wrapped status row would corrupt the row below it. kCols = 19 is HA-57,
 PROVISIONAL — and R1a found a CONFLICT while reading the vendored source:
 misc.hpp:322 documents set_text's col parameter as [0-14], which implies a
 15-column grid, not 19. Community practice says 19 visible characters;
 the vendored doc says 15; neither is a measurement. Registered as HA-107;
 the bench runbook writes a ruler string ("0123456789ABCDEFGHIJ") and
 counts what the physical LCD shows. Until then this adapter truncates at
 ILineDisplay::kCols exactly as the seam contract states.

 PADDING TO kCols, deliberately: set_text writes from column 0 and leaves
 whatever was beyond the new text's end — a shorter line would show the
 tail of the previous one ("ARM OK" over "ARMED FAULT" reads "ARM OKFAULT").
 Padding with spaces makes setLine() a true OVERWRITE, which is the verb
 the seam promises ("Overwrite row i").

 RATE: the firmware rate-limits controller writes (misc.hpp:312-313 "text
 setting is currently in beta … continuous fast updates will not work
 well"). Pacing lives ABOVE the seam (ControllerFaultDisplay already
 repaints only changed rows — line_display.hpp header); this adapter stays
 a dumb device write, one call = one set_text.

 MUST NOT THROW (contract): set_text returns an error code; a failed write
 (controller unplugged) is DROPPED — a status row has no fallback channel,
 and the telemetry log already carries the same information.

 HA register: HA-57, HA-107.
```

</details>
