<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/absent_vision.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `absent_vision.hpp`

AbsentVision — the explicit statement that THIS ROBOT HAS NO OBJECT/COLOR DETECTION SOURCE, as an IVision.

This header declares **1** type (1 member) and **2** constants.

Extracted from [`include/shulib/hal/absent_vision.hpp`](../../include/shulib/hal/absent_vision.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class AbsentVision`](#class-absentvision)
  - [`objects`](#absentvision-objects)
- [`kInstallVisionPoller`](#kinstallvisionpoller) — *constant*
- [`kInstallVisionPoller<AbsentVision>`](#kinstallvisionpollerabsentvision) — *constant*

<a id="class-absentvision"></a>

## `class AbsentVision`

```cpp
class AbsentVision final : public IVision
```

"This robot has no object/color detection source", as an IVision. objects() is `{}` forever — a default-constructed std::vector allocates nothing, so the class is allocation-free, stateless and never throws. DO NOT POLL IT: consult kInstallVisionPoller at the wiring site (absent_tag_source.hpp's banner holds the reasoning). As with every Absent device, the composition root that wires this announces the absence once at boot (E1's principle 5).

*class, declared at [`include/shulib/hal/absent_vision.hpp:32`](../../include/shulib/hal/absent_vision.hpp#L32).*

<a id="absentvision-objects"></a>

### `AbsentVision::objects`

```cpp
[[nodiscard]] std::vector<ObjectObservation> objects() const override
```

No detections, ever — an empty vector, which allocates nothing. NOT "nothing in view this tick": there is no detector to view with, which is why no consumer should be polling it (class note).

*function, declared at [`include/shulib/hal/absent_vision.hpp:37`](../../include/shulib/hal/absent_vision.hpp#L37).*

<a id="kinstallvisionpoller"></a>

## `kInstallVisionPoller`

```cpp
template <class VisionT> inline constexpr bool kInstallVisionPoller = true
```

The vision-side sibling of kInstallTagCorrector (absent_tag_source.hpp — the reasoning lives there): SHOULD the composition root install (and drive) a vision-rate consumer over the source it declared as `VisionT`? True for every present source, including FakeVision (a fake is a live source a test drives). Specialized false for AbsentVision below. Declared before any consumer exists so M4's poller is built against the rule rather than against the trap. Same limitation: it answers from the DECLARED type, so ask it where the device member is declared, never through an `IVision*`.

*constant, declared at [`include/shulib/hal/absent_vision.hpp:49`](../../include/shulib/hal/absent_vision.hpp#L49).*

<a id="kinstallvisionpollerabsentvision"></a>

## `kInstallVisionPoller<AbsentVision>`

```cpp
template <> inline constexpr bool kInstallVisionPoller<AbsentVision> = false
```

The ruling applied to the absent detector: no vision-rate consumer is installed or driven over it, so no "vision alive, nothing seen" record can ever be manufactured for a robot with no detector.

*constant, declared at [`include/shulib/hal/absent_vision.hpp:55`](../../include/shulib/hal/absent_vision.hpp#L55).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 16 lines</summary>

```text

 AbsentVision — the explicit statement that THIS ROBOT HAS NO OBJECT/COLOR DETECTION SOURCE,
 as an IVision (chunk R3b §6).

 Not a test double: hal/fake/FakeVision exists to be DRIVEN (setObjects()/clear()); this
 class exists to say there is nothing to drive. See absent_gps.hpp's banner for the family
 ruling — preconditions stay, absence is spelled out at the call site
 (`.vision = &absentVision`), naming (`Null…` rejected), no Freeze Register row (F13) — and
 absent_tag_source.hpp's banner for the liveness trap the wiring rule below exists to block.

 IVision has no consumer in the tree yet (M4 owns manipulation targeting), so unlike the tag
 side there is no corrector to mis-record TODAY. The rule is still declared NOW, because the
 trap is identical in shape the moment an M4 consumer polls objects() and keeps a "vision
 alive, nothing seen" record: an AbsentVision returning `{}` is byte-indistinguishable from
 a live detector with nothing in view. Absence is a wiring decision, resolved once at
 construction — M4 inherits the rule instead of rediscovering the defect.
```

</details>
