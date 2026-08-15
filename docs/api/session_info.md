<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/session_info.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `session_info.hpp`

SessionInfo + the §18.5 session header — provenance as the FIRST lines of every run.

This header declares **1** type (5 members), **1** free function, and **3** constants.

Extracted from [`include/shulib/diag/session_info.hpp`](../../include/shulib/diag/session_info.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct SessionInfo`](#struct-sessioninfo)
  - [`buildHash`](#sessioninfo-buildhash)
  - [`routineId`](#sessioninfo-routineid)
  - [`alliance`](#sessioninfo-alliance)
  - [`side`](#sessioninfo-side)
  - [`portMap`](#sessioninfo-portmap)
- [`kMaxHashBytes`](#kmaxhashbytes) — *constant*
- [`kMaxFieldBytes`](#kmaxfieldbytes) — *constant*
- [`kMaxPortMapBytes`](#kmaxportmapbytes) — *constant*
- [`emitSessionHeader`](#emitsessionheader) — *free function*

<a id="struct-sessioninfo"></a>

## `struct SessionInfo`

```cpp
struct SessionInfo
```

Provenance for one run. string_views: the caller keeps the storage alive for the emitSessionHeader() call only (they are read synchronously, never retained — RunSummary re-copies what it needs into bounded arrays).

*struct, declared at [`include/shulib/diag/session_info.hpp:41`](../../include/shulib/diag/session_info.hpp#L41).*

<a id="sessioninfo-buildhash"></a>

### `SessionInfo::buildHash`

```cpp
std::string_view buildHash{}
```

From diag::compiledBuildHash() in the app's own TU. EMPTY = missing = LOUD.

*field, declared at [`include/shulib/diag/session_info.hpp:43`](../../include/shulib/diag/session_info.hpp#L43).*

<a id="sessioninfo-routineid"></a>

### `SessionInfo::routineId`

```cpp
std::string_view routineId{}
```

e.g. "redLeftTall"

*field, declared at [`include/shulib/diag/session_info.hpp:44`](../../include/shulib/diag/session_info.hpp#L44).*

<a id="sessioninfo-alliance"></a>

### `SessionInfo::alliance`

```cpp
std::string_view alliance{}
```

"red" / "blue" / "skills"

*field, declared at [`include/shulib/diag/session_info.hpp:45`](../../include/shulib/diag/session_info.hpp#L45).*

<a id="sessioninfo-side"></a>

### `SessionInfo::side`

```cpp
std::string_view side{}
```

"left" / "right"

*field, declared at [`include/shulib/diag/session_info.hpp:46`](../../include/shulib/diag/session_info.hpp#L46).*

<a id="sessioninfo-portmap"></a>

### `SessionInfo::portMap`

```cpp
std::string_view portMap{}
```

caller-authored at C5; G1 generates (header note)

*field, declared at [`include/shulib/diag/session_info.hpp:47`](../../include/shulib/diag/session_info.hpp#L47).*

<a id="kmaxhashbytes"></a>

## `kMaxHashBytes`

```cpp
inline constexpr std::size_t kMaxHashBytes = 47
```

Bounds for the header's caller-controlled fields (sanitized appends; the sink itself re-sanitizes — defense in depth, and these keep one field from eating the line).

*constant, declared at [`include/shulib/diag/session_info.hpp:53`](../../include/shulib/diag/session_info.hpp#L53).*

<a id="kmaxfieldbytes"></a>

## `kMaxFieldBytes`

```cpp
inline constexpr std::size_t kMaxFieldBytes = 24
```

routine/alliance/side

*constant, declared at [`include/shulib/diag/session_info.hpp:54`](../../include/shulib/diag/session_info.hpp#L54).*

<a id="kmaxportmapbytes"></a>

## `kMaxPortMapBytes`

```cpp
inline constexpr std::size_t kMaxPortMapBytes = 96
```

the port map is the long one

*constant, declared at [`include/shulib/diag/session_info.hpp:55`](../../include/shulib/diag/session_info.hpp#L55).*

<a id="emitsessionheader"></a>

## `emitSessionHeader`

```cpp
inline void emitSessionHeader(hal::ITelemetrySink& sink, const SessionInfo& info, units::Voltage batteryStart)
```

Emit the §18.5 session header: three [SES] Info lines (plus the [ERROR][SES] line FIRST when the hash is missing). `batteryStart` must be a live reading (header note). Byte shapes pinned by test/session_header_test.cpp.  [t=   0.00] [SES] run start · build 0b4948a-dirty · routine "redLeftTall" [t=   0.00] [SES] alliance red · side left · batt 12.40V [t=   0.00] [SES] ports L1,2,3 R4,5,6 IMU10

*free function, declared at [`include/shulib/diag/session_info.hpp:64`](../../include/shulib/diag/session_info.hpp#L64).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 27 lines</summary>

```text

 SessionInfo + the §18.5 session header — provenance as the FIRST lines of every
 run (WS13, chunk C5).

 §18.5: "First record of every run: git build hash + routine id + alliance/side +
 port map + battery start — lets us compare/reproduce runs and confirm exactly
 which binary produced a given log." The header is what turns a pile of logs into
 an archive: without it, two traces that disagree cannot even be attributed to
 two binaries.

 The header rides the log() channel as [SES]-tagged Info lines (the FaultLatch
 precedent: the OWNER formats structured text, the sink sanitizes and frames it),
 which is what gives it the exact "[t=…] [SES] …" §18.3 shape through TermSink —
 and lets a message-only sink capture provenance too.

 THE MISSING-HASH PATH IS LOUD (build_info.hpp carries the full rationale): an
 empty buildHash emits an [ERROR][SES] line FIRST — before anything else — and
 the header line renders the literal token MISSING. Never a plausible value.

 Field notes:
   * portMap is CALLER-AUTHORED text at C5 ("L1,2,3 R4,5,6 IMU10"): the core HAL
     deliberately has no port numbers (fakes don't have ports), so the honest v1
     is a pass-through string; G1's RobotBuilder generates it from the profile.
   * alliance/side are free text ("red"/"left"/"skills"); empty fields render as
     "-" so a blank never silently vanishes from a column.
   * battery start is READ, not caller-typed — a typed 12.6 that was actually
     11.9 is the lying-number class this chunk bans.
```

</details>
