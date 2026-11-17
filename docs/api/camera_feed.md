<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/vision/camera_feed.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `camera_feed.hpp`

CameraFeed — ONE read of the camera per period, handed to every consumer.

This header declares **3** types (15 members) and **1** free function.

Extracted from [`include/shulib/vision/camera_feed.hpp`](../../include/shulib/vision/camera_feed.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct CameraFeedConfig`](#struct-camerafeedconfig)
  - [`minReadPeriodSeconds`](#camerafeedconfig-minreadperiodseconds)
- [`cameraFeedConfigProblem`](#camerafeedconfigproblem) — *free function*
- [`struct CameraFeedStats`](#struct-camerafeedstats)
  - [`reads`](#camerafeedstats-reads)
  - [`usable`](#camerafeedstats-usable)
  - [`readFaults`](#camerafeedstats-readfaults)
  - [`partialFaults`](#camerafeedstats-partialfaults)
  - [`notConfigured`](#camerafeedstats-notconfigured)
  - [`truncated`](#camerafeedstats-truncated)
  - [`objectsWithheld`](#camerafeedstats-objectswithheld)
  - [`coloursWithheld`](#camerafeedstats-colourswithheld)
  - [`lastReadSeconds`](#camerafeedstats-lastreadseconds)
  - [`slowestReadSeconds`](#camerafeedstats-slowestreadseconds)
- [`class CameraFeed`](#class-camerafeed)
  - [`CameraFeed`](#camerafeed-camerafeed)
  - [`tick`](#camerafeed-tick)
  - [`latest`](#camerafeed-latest)
  - [`stats`](#camerafeed-stats)

<a id="struct-camerafeedconfig"></a>

## `struct CameraFeedConfig`

```cpp
struct CameraFeedConfig
```

The feed's one number — the TEAM's. 0 = UNSET, and refused (no library default).

*struct, declared at [`include/shulib/vision/camera_feed.hpp:39`](../../include/shulib/vision/camera_feed.hpp#L39).*

<a id="camerafeedconfig-minreadperiodseconds"></a>

### `CameraFeedConfig::minReadPeriodSeconds`

```cpp
double minReadPeriodSeconds = 0.0
```

Read at most this often, seconds — e.g. 0.033 (VEX's ~30 frames a second, A4 register HA-192).

*field, declared at [`include/shulib/vision/camera_feed.hpp:41`](../../include/shulib/vision/camera_feed.hpp#L41).*

<a id="camerafeedconfigproblem"></a>

## `cameraFeedConfigProblem`

```cpp
[[nodiscard]] constexpr const char* cameraFeedConfigProblem(const CameraFeedConfig& c) noexcept
```

The first problem with a feed config, or nullptr: a period that is UNSET, zero, negative, NaN or infinite (an infinite one would read once and never again).

*free function, declared at [`include/shulib/vision/camera_feed.hpp:46`](../../include/shulib/vision/camera_feed.hpp#L46).*

<a id="struct-camerafeedstats"></a>

## `struct CameraFeedStats`

```cpp
struct CameraFeedStats
```

What the feed has seen — for a bench page, and for A4 register HA-203's settle (the read's cost).

*struct, declared at [`include/shulib/vision/camera_feed.hpp:54`](../../include/shulib/vision/camera_feed.hpp#L54).*

<a id="camerafeedstats-reads"></a>

### `CameraFeedStats::reads`

```cpp
std::uint32_t reads = 0
```

reads made

*field, declared at [`include/shulib/vision/camera_feed.hpp:55`](../../include/shulib/vision/camera_feed.hpp#L55).*

<a id="camerafeedstats-usable"></a>

### `CameraFeedStats::usable`

```cpp
std::uint32_t usable = 0
```

of them, status Ok

*field, declared at [`include/shulib/vision/camera_feed.hpp:56`](../../include/shulib/vision/camera_feed.hpp#L56).*

<a id="camerafeedstats-readfaults"></a>

### `CameraFeedStats::readFaults`

```cpp
std::uint32_t readFaults = 0
```

of them, status ReadFault

*field, declared at [`include/shulib/vision/camera_feed.hpp:57`](../../include/shulib/vision/camera_feed.hpp#L57).*

<a id="camerafeedstats-partialfaults"></a>

### `CameraFeedStats::partialFaults`

```cpp
std::uint32_t partialFaults = 0
```

of them, status PartialFault (a torn read)

*field, declared at [`include/shulib/vision/camera_feed.hpp:58`](../../include/shulib/vision/camera_feed.hpp#L58).*

<a id="camerafeedstats-notconfigured"></a>

### `CameraFeedStats::notConfigured`

```cpp
std::uint32_t notConfigured = 0
```

of them, status NotConfigured

*field, declared at [`include/shulib/vision/camera_feed.hpp:59`](../../include/shulib/vision/camera_feed.hpp#L59).*

<a id="camerafeedstats-truncated"></a>

### `CameraFeedStats::truncated`

```cpp
std::uint32_t truncated = 0
```

usable reads that were POSSIBLY FULL (24 or more reported)

*field, declared at [`include/shulib/vision/camera_feed.hpp:60`](../../include/shulib/vision/camera_feed.hpp#L60).*

<a id="camerafeedstats-objectswithheld"></a>

### `CameraFeedStats::objectsWithheld`

```cpp
std::uint32_t objectsWithheld = 0
```

Object detections the adapter withheld, summed over the reads

*field, declared at [`include/shulib/vision/camera_feed.hpp:61`](../../include/shulib/vision/camera_feed.hpp#L61).*

<a id="camerafeedstats-colourswithheld"></a>

### `CameraFeedStats::coloursWithheld`

```cpp
std::uint32_t coloursWithheld = 0
```

Colour and Code detections the adapter withheld, summed

*field, declared at [`include/shulib/vision/camera_feed.hpp:62`](../../include/shulib/vision/camera_feed.hpp#L62).*

<a id="camerafeedstats-lastreadseconds"></a>

### `CameraFeedStats::lastReadSeconds`

```cpp
double lastReadSeconds = 0.0
```

how long the last read took: the clock after it less before it

*field, declared at [`include/shulib/vision/camera_feed.hpp:63`](../../include/shulib/vision/camera_feed.hpp#L63).*

<a id="camerafeedstats-slowestreadseconds"></a>

### `CameraFeedStats::slowestReadSeconds`

```cpp
double slowestReadSeconds = 0.0
```

the longest read so far, seconds

*field, declared at [`include/shulib/vision/camera_feed.hpp:64`](../../include/shulib/vision/camera_feed.hpp#L64).*

<a id="class-camerafeed"></a>

## `class CameraFeed`

```cpp
class CameraFeed
```

One camera read per period for every consumer (header). Holds references: the camera and the clock must outlive the feed. PRECONDITION: cameraFeedConfigProblem(cfg) == nullptr (a consumer that builds one asks first and refuses to construct, naming it).

*class, declared at [`include/shulib/vision/camera_feed.hpp:70`](../../include/shulib/vision/camera_feed.hpp#L70).*

<a id="camerafeed-camerafeed"></a>

### `CameraFeed::CameraFeed`

```cpp
CameraFeed(hal::IVisionFrames& camera, const hal::IClock& clock, const CameraFeedConfig& cfg) noexcept
```

Reads nothing until the first tick().

*function, declared at [`include/shulib/vision/camera_feed.hpp:73`](../../include/shulib/vision/camera_feed.hpp#L73).*

<a id="camerafeed-tick"></a>

### `CameraFeed::tick`

```cpp
bool tick()
```

Read the camera if at least the period has passed since the last read BEGAN (the first call always reads); never more than one read per call. True when it read — latest() is then a new frame.

*function, declared at [`include/shulib/vision/camera_feed.hpp:78`](../../include/shulib/vision/camera_feed.hpp#L78).*

<a id="camerafeed-latest"></a>

### `CameraFeed::latest`

```cpp
[[nodiscard]] const hal::VisionFrame& latest() const noexcept
```

The latest READ — a fault frame included, never the last good one held; a default frame (status NotRead) before the first.

*function, declared at [`include/shulib/vision/camera_feed.hpp:111`](../../include/shulib/vision/camera_feed.hpp#L111).*

<a id="camerafeed-stats"></a>

### `CameraFeed::stats`

```cpp
[[nodiscard]] const CameraFeedStats& stats() const noexcept
```

What the feed has seen.

*function, declared at [`include/shulib/vision/camera_feed.hpp:113`](../../include/shulib/vision/camera_feed.hpp#L113).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 27 lines</summary>

```text

 CameraFeed — ONE read of the camera per period, handed to every consumer (chunk VIS1-A2, 2026-11-03).

 ── Who reads the camera, and how often ─────────────────────────────────────────────
 A read is one device call for the count and one per detection (up to 25), plus the adapter's own
 checks (hal::IVisionFrames::read()), and the library is single-task by contract: every device here is
 read from the caller's own loop, never from a task of the library's. So the program's loop — the
 driver loop's per-tick hook, an auton step's wait — calls tick() once per ITS tick, and the feed reads
 the camera only when at least `minReadPeriodSeconds` has passed since its last read BEGAN (never twice
 in one call). Every consumer — the Toggle tracker, the piece finder, and later the automatic flip and
 the drive to a piece — is handed the SAME frame through latest(): two consumers never cost two reads,
 and none reads faster than the camera changes (VEX states about 30 frames a second — A4 register
 HA-192 — so about 0.033 s is the natural floor; the period is the TEAM's number, and an UNSET one is
 refused: no library default).

 ── latest() is the latest READ — never the last good one ───────────────────────────
 A fault frame included: a frame held over a failed read would read as a fresh "nothing changed"
 (hal/vision_frames.hpp). Only `FrameStatus::Ok` is evidence, and each consumer checks it.

 ── Every read is timed ─────────────────────────────────────────────────────────────
 With the clock before and after it (on the robot that clock is pros::micros()-backed —
 hal/pros/clock.hpp), so stats() carries the last and the slowest read's cost: the measurement
 A4 register HA-203 (one read is cheap enough for the driver loop's tick) needs, from the loop the
 read actually runs in.

 No task, no allocation, PROS-free, sim-free; single-task, like every device here. NO HARDWARE CLAIM:
 it has run against the host's fake camera and nothing else.
```

</details>
