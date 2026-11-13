<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/ai_vision.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `ai_vision.hpp`

ProsAiVision — hal::IVisionFrames (and hal::IReadFaults, MECH-A1's channel) over the vendored PROS 4.2.2 AI Vision C API, include/pros/ai_vision.h.

This header declares **1** type (19 members).

Extracted from [`include/shulib/hal/pros/ai_vision.hpp`](../../include/shulib/hal/pros/ai_vision.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ProsAiVision`](#class-prosaivision)
  - [`kModeMask`](#prosaivision-kmodemask)
  - [`kRetrySeconds`](#prosaivision-kretryseconds)
  - [`kVerifySeconds`](#prosaivision-kverifyseconds)
  - [`ProsAiVision`](#prosaivision-prosaivision)
  - [`ProsAiVision (overload 2)`](#prosaivision-prosaivision-2)
  - [`ProsAiVision (overload 3)`](#prosaivision-prosaivision-3)
  - [`operator=`](#prosaivision-operator-eq)
  - [`operator= (overload 2)`](#prosaivision-operator-eq-2)
  - [`~ProsAiVision`](#prosaivision-destructor-prosaivision)
  - [`read`](#prosaivision-read)
  - [`status`](#prosaivision-status)
  - [`faultedReads`](#prosaivision-faultedreads)
  - [`model`](#prosaivision-model)
  - [`modelProblem`](#prosaivision-modelproblem)
  - [`configured`](#prosaivision-configured)
  - [`setupProblem`](#prosaivision-setupproblem)
  - [`setups`](#prosaivision-setups)
  - [`settingsLost`](#prosaivision-settingslost)
  - [`settings`](#prosaivision-settings)

<a id="class-prosaivision"></a>

## `class ProsAiVision`

```cpp
class ProsAiVision final : public IVisionFrames, public IReadFaults
```

`hal::IVisionFrames` (+ `hal::IReadFaults`) over one AI Vision sensor: the settings written and read back, the AI model checked by name, a frame per read() that says how the read went (header). Constructed by the device provider only (`hal::pros::ProsDeviceProvider::aiVision()`), which refuses a bad settings block or a port that does not carry an AI Vision sensor first.

*class, declared at [`include/shulib/hal/pros/ai_vision.hpp:108`](../../include/shulib/hal/pros/ai_vision.hpp#L108).*

<a id="prosaivision-kmodemask"></a>

### `ProsAiVision::kModeMask`

```cpp
static constexpr std::uint8_t kModeMask = static_cast<std::uint8_t>( ::pros::E_AIVISION_MODE_TAGS | ::pros::E_AIVISION_MODE_COLORS | ::pros::E_AIVISION_MODE_OBJECTS | ::pros::E_AIVISION_MODE_COLOR_MERGE)
```

Every mode write uses this mask: the write sets EXACTLY the wanted modes, TAGS included (off).

*field, declared at [`include/shulib/hal/pros/ai_vision.hpp:111`](../../include/shulib/hal/pros/ai_vision.hpp#L111).*

<a id="prosaivision-kretryseconds"></a>

### `ProsAiVision::kRetrySeconds`

```cpp
static constexpr double kRetrySeconds = 0.5
```

A failed set-up is retried at most this often, in seconds (the camera may still be booting).

*field, declared at [`include/shulib/hal/pros/ai_vision.hpp:115`](../../include/shulib/hal/pros/ai_vision.hpp#L115).*

<a id="prosaivision-kverifyseconds"></a>

### `ProsAiVision::kVerifySeconds`

```cpp
static constexpr double kVerifySeconds = 1.0
```

While configured, the modes and every descriptor are read back, and the model re-asked, this often, in seconds (a lost setting, or another model, is found within it).

*field, declared at [`include/shulib/hal/pros/ai_vision.hpp:118`](../../include/shulib/hal/pros/ai_vision.hpp#L118).*

<a id="prosaivision-prosaivision"></a>

### `ProsAiVision::ProsAiVision`

```cpp
ProsAiVision(std::uint8_t port, const AiVisionSettings& settings, const IClock& clock)
```

Write the set-up NOW, on `port` (1..21), with `settings`, timing everything with `clock` (borrowed; it must outlive this adapter). PRECONDITION: `aiVisionSettingsProblem(settings) == nullptr` (the provider refuses a bad block before constructing). The settings are COPIED; the class-name table they point at is borrowed (AiVisionSettings says so).

*function, declared at [`include/shulib/hal/pros/ai_vision.hpp:124`](../../include/shulib/hal/pros/ai_vision.hpp#L124).*

<a id="prosaivision-prosaivision-2"></a>

### `ProsAiVision::ProsAiVision (overload 2)`

```cpp
ProsAiVision(const ProsAiVision&) = delete
```

Non-copyable: one adapter per camera (two would fight over its settings).

*function, declared at [`include/shulib/hal/pros/ai_vision.hpp:130`](../../include/shulib/hal/pros/ai_vision.hpp#L130).*

<a id="prosaivision-prosaivision-3"></a>

### `ProsAiVision::ProsAiVision (overload 3)`

```cpp
ProsAiVision(ProsAiVision&&) = delete
```

Non-copyable: one adapter per camera (two would fight over its settings). Non-movable: the provider holds it in place, and the status strings point into it.

*function, declared at [`include/shulib/hal/pros/ai_vision.hpp:132`](../../include/shulib/hal/pros/ai_vision.hpp#L132).*

<a id="prosaivision-operator-eq"></a>

### `ProsAiVision::operator=`

```cpp
ProsAiVision& operator=(const ProsAiVision&) = delete
```

Non-copyable: one adapter per camera (two would fight over its settings). Non-movable: the provider holds it in place, and the status strings point into it. Non-copy-assignable, same reasoning.

*function, declared at [`include/shulib/hal/pros/ai_vision.hpp:134`](../../include/shulib/hal/pros/ai_vision.hpp#L134).*

<a id="prosaivision-operator-eq-2"></a>

### `ProsAiVision::operator= (overload 2)`

```cpp
ProsAiVision& operator=(ProsAiVision&&) = delete
```

Non-copyable: one adapter per camera (two would fight over its settings). Non-movable: the provider holds it in place, and the status strings point into it. Non-copy-assignable, same reasoning. Non-move-assignable, same reasoning.

*function, declared at [`include/shulib/hal/pros/ai_vision.hpp:136`](../../include/shulib/hal/pros/ai_vision.hpp#L136).*

<a id="prosaivision-destructor-prosaivision"></a>

### `ProsAiVision::~ProsAiVision`

```cpp
~ProsAiVision() override = default
```

Non-copyable: one adapter per camera (two would fight over its settings). Non-movable: the provider holds it in place, and the status strings point into it. Non-copy-assignable, same reasoning. Non-move-assignable, same reasoning. Nothing to release: the camera keeps whatever was last written.

*function, declared at [`include/shulib/hal/pros/ai_vision.hpp:138`](../../include/shulib/hal/pros/ai_vision.hpp#L138).*

<a id="prosaivision-read"></a>

### `ProsAiVision::read`

```cpp
[[nodiscard]] VisionFrame read() noexcept override
```

One READ (header): the 1 Hz check of the settings and the model first, then any due re-set-up, then the count and each object. NotConfigured until a set-up succeeds; a failed count — or a read-back the 1 Hz check could not make — is a ReadFault with no detections (and the set-up marked lost); a skipped object makes the frame a PartialFault.

*function, declared at [`include/shulib/hal/pros/ai_vision.hpp:144`](../../include/shulib/hal/pros/ai_vision.hpp#L144).*

<a id="prosaivision-status"></a>

### `ProsAiVision::status`

```cpp
[[nodiscard]] CameraStatus status() const noexcept override
```

The set-up and the model check (IVisionFrames), for the boot line: `{configured(), setupProblem(), model(), modelProblem()}`. Touches nothing.

*function, declared at [`include/shulib/hal/pros/ai_vision.hpp:244`](../../include/shulib/hal/pros/ai_vision.hpp#L244).*

<a id="prosaivision-faultedreads"></a>

### `ProsAiVision::faultedReads`

```cpp
[[nodiscard]] int faultedReads() const noexcept override
```

Reads that were not evidence because a device call failed — a failed count, a 1 Hz check the camera did not answer, or a read with a skipped object (IReadFaults). Monotonic.

*function, declared at [`include/shulib/hal/pros/ai_vision.hpp:250`](../../include/shulib/hal/pros/ai_vision.hpp#L250).*

<a id="prosaivision-model"></a>

### `ProsAiVision::model`

```cpp
[[nodiscard]] ModelCheck model() const noexcept
```

The AI model check's state.

*function, declared at [`include/shulib/hal/pros/ai_vision.hpp:253`](../../include/shulib/hal/pros/ai_vision.hpp#L253).*

<a id="prosaivision-modelproblem"></a>

### `ProsAiVision::modelProblem`

```cpp
[[nodiscard]] const char* modelProblem() const noexcept
```

Why the model check is not Matches, in words — e.g. "class 3 is 'blueRing', the robot file expects 'yellowPin'"; "" while it Matches or was never asked.

*function, declared at [`include/shulib/hal/pros/ai_vision.hpp:256`](../../include/shulib/hal/pros/ai_vision.hpp#L256).*

<a id="prosaivision-configured"></a>

### `ProsAiVision::configured`

```cpp
[[nodiscard]] bool configured() const noexcept
```

True once the set-up was written AND read back; false again when it was found lost.

*function, declared at [`include/shulib/hal/pros/ai_vision.hpp:258`](../../include/shulib/hal/pros/ai_vision.hpp#L258).*

<a id="prosaivision-setupproblem"></a>

### `ProsAiVision::setupProblem`

```cpp
[[nodiscard]] const char* setupProblem() const noexcept
```

Why the last set-up attempt failed or was re-done ("" while it holds).

*function, declared at [`include/shulib/hal/pros/ai_vision.hpp:260`](../../include/shulib/hal/pros/ai_vision.hpp#L260).*

<a id="prosaivision-setups"></a>

### `ProsAiVision::setups`

```cpp
[[nodiscard]] int setups() const noexcept
```

How many set-up attempts were made (the boot's counts as 1; a failed one counts too).

*function, declared at [`include/shulib/hal/pros/ai_vision.hpp:262`](../../include/shulib/hal/pros/ai_vision.hpp#L262).*

<a id="prosaivision-settingslost"></a>

### `ProsAiVision::settingsLost`

```cpp
[[nodiscard]] int settingsLost() const noexcept
```

How often the 1 Hz check found a setting read back DIFFERENT (a check the camera did not answer is a failed read — faultedReads() — never counted here).

*function, declared at [`include/shulib/hal/pros/ai_vision.hpp:265`](../../include/shulib/hal/pros/ai_vision.hpp#L265).*

<a id="prosaivision-settings"></a>

### `ProsAiVision::settings`

```cpp
[[nodiscard]] const AiVisionSettings& settings() const noexcept
```

The settings this adapter writes, as it was constructed with them (the provider compares a second request's against them).

*function, declared at [`include/shulib/hal/pros/ai_vision.hpp:268`](../../include/shulib/hal/pros/ai_vision.hpp#L268).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 78 lines, click to expand</summary>

```text

 ProsAiVision — hal::IVisionFrames (and hal::IReadFaults, MECH-A1's channel) over the vendored PROS
 4.2.2 AI Vision C API, include/pros/ai_vision.h (chunk VIS1-A1, 2026-10-18).

 BINDS (the C API, as Bench Tests station 12 does; the C++ pros::AIVision::get_all_objects() returns a
 std::vector, which allocates, and its header includes <expected>, a C++23 header):
  * aivision_set_color (:352), then aivision_get_color (:369) read back — each descriptor;
  * aivision_set_enabled_detection_types (:252) with a mask covering TAGS|COLORS|OBJECTS|COLOR_MERGE,
    so the write sets EXACTLY the wanted modes, TAGS off; aivision_get_enabled_detection_types (:232)
    read back (the mask's meaning rests on the doc's one example, :242-244 — A4 register HA-196);
  * aivision_get_object_count (:463) and aivision_get_object (:479) — a READ (A4 register HA-195,
    HA-203);
  * aivision_get_class_name (:389) — with `objects` on, the name of each EXPECTED class id is read and
    compared BY NAME with the robot file's table (A4 register HA-197). Until every one Matches, no
    Object detection is passed on, and an Object whose id is past the table is NEVER passed on (its
    name was never checked). Re-asked after every set-up and at every 1 Hz check, matching or not (the
    doc: names may read wrong "for the first several hundred milliseconds of being plugged in",
    :373-374; and a camera swapped while nothing read it may carry another model) — so a match is
    never more than a second old.

 THERE IS NO CLASS COUNT. The header says id -1 "will return the number of class names" (:375); the
 vendored firmware/libpros.a (kernel 4.2.2; object vdml_ai_vision.c.o, aivision_get_class_name,
 disassembled with arm-none-eabi-objdump) is `bl vexDeviceAiVisionClassNameGet` → `port_mutex_give` →
 `movs r0, #1`: the SDK's value is DISCARDED and EVERY id, -1 included, returns PROS_SUCCESS (1);
 PROS_ERR only when the binding or the port mutex fails. So the check never asks for -1, zeroes the
 buffer before each read (a name the camera does not write reads as ''), and counts PROS_ERR as
 Unreadable and an empty or wrong name as Different. Where the PROS header and the binary disagree, the
 binary is the fact (VIS1's brief, re-check T1; re-read at VIS1-A1).

 NEVER CALLED (pinned as text by test/pros_adapter_fence_test.cpp): aivision_reset (:209 — "the
 initial state" is undocumented: it might drop the model the AI Vision Utility chose), any tag-family
 call (no tags in VIS1), aivision_start_awb (:418 — white balance is the camera's own; a balance taken
 mid-match would move every colour the team measured), the USB bounding-box overlay (a bench aid), and
 aivision_set_code (VIS1 sets no colour codes).

 THE SETTINGS ARE RE-WRITTEN, NEVER TRUSTED (A4 register HA-196): at construction; after ANY failed
 read (a camera re-plugged mid-run may come back without them); and whenever the 1 Hz check finds one
 changed. Each colour is read back after it is written (id and RGB compared — the two floats are NOT,
 because whether the camera stores them exactly is HA-196's question and refusing on a rounding would
 refuse a working camera). Every PROS setter returns 1 whatever the SDK did (the binary: each ends
 `movs r0, #1`), so the read-back is the only check there is. A set-up that fails is retried at most
 every kRetrySeconds and reads NotConfigured until it succeeds — never a frame from a camera configured
 by someone else.

 THE 1 Hz CHECK, before the first read due after each second: the modes and EVERY descriptor read
 back, then the model re-asked. A read-back that came back DIFFERENT is a lost setting (settingsLost(),
 and everything re-written); one the camera did not ANSWER (the mode read's PROS_ERR, or get_color's
 failure record — EXACTLY id, red, green, blue 127 and both floats +Inf, read in the binary; any other
 answer, an empty slot included, is compared) is a failed READ: a ReadFault frame, counted in
 faultedReads(), exactly as a failed count is — so an unplug counts the same whichever call meets it
 first. Its cost — 1 + the colours + the expected names, at most 28 device calls, once a second — is
 HA-203's, as is a set-up's (two calls per colour, two for the modes, one per expected name: at most 36,
 inside the read that finds it due). What it cannot see: a camera swapped AND answering again between
 two reads less than a second apart, with no read meeting it unplugged — its objects pass under the old
 check until the next second.

 A READ (A4 register HA-195 — the count and each object are separate calls; the camera may update between
 them — and A4 register HA-192: no frame counter and no timestamp, so the frame is stamped when the read
 COMPLETES): the count first, compared with PROS_ERR BEFORE anything else (PROS_ERR is INT32_MAX — a
 "count >= 24" check alone would read a failure as a full camera); a failed count is a ReadFault frame
 with NO detections, one faulted read, and the set-up marked lost. A count of 24 OR MORE is "possibly
 full" (`VisionFrame::truncated`; the vendored header calls 24 a parameter "given by VEX", :33, so a
 camera holding more may still REPORT 24 — A4 register HA-198) and reads 0..23. Each object is copied out
 BY VALUE — never a pointer or reference into the packed record (undefined on the ARM) — its box as the
 left edge, top edge, width and height (HA-193), an Object's score whole (the raw uint16; its scale is
 HA-198's). An object with no valid type (the binary's failure record is {id 127, type 127}) or a TAG is
 skipped, and a read with any skip is a PartialFault, counted: a torn read is not evidence. A colour blob
 whose descriptor THIS program did not write (a slot another program left set) and any colour CODE (VIS1
 writes none) are WITHHELD and counted (`VisionFrame::coloursWithheld`) — so every Colour detection's id
 is one the settings name.

 faultedReads() (IReadFaults) counts every ReadFault and PartialFault frame — monotonic, MECH-A1's
 contract — so a consumer written for that channel works unchanged. Unlike the scalar sensor adapters,
 nothing is HELD: the frame itself says the read failed (hal/vision_frames.hpp).

 NO HARDWARE CLAIM: host-tested through test/pros_shim/pros/ai_vision.h only, which tests this class
 against our BELIEF about PROS (HA-192..HA-198) and cannot test the belief. No AI Vision sensor has
 been read by this code; Bench Tests station 12's LIBRARY page (VIS1-B) is its first run on a robot.
```

</details>
