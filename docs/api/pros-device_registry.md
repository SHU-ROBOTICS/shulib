<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/device_registry.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `device_registry.hpp`

ProsDeviceRegistry — IDeviceRegistry over PROS's smart-port registry.

This header declares **1** type (3 members).

Extracted from [`include/shulib/hal/pros/device_registry.hpp`](../../include/shulib/hal/pros/device_registry.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ProsDeviceRegistry`](#class-prosdeviceregistry)
  - [`ProsDeviceRegistry`](#prosdeviceregistry-prosdeviceregistry)
  - [`pluggedKind`](#prosdeviceregistry-pluggedkind)
  - [`kindOf`](#prosdeviceregistry-kindof)

<a id="class-prosdeviceregistry"></a>

## `class ProsDeviceRegistry`

```cpp
class ProsDeviceRegistry final : public IDeviceRegistry
```

IDeviceRegistry over `pros::c::registry_get_plugged_type`. Stateless, cacheless and constructible anywhere; the ONE line that turns a physical port into PROS's zero-indexed registry index lives in pluggedKind() and is pinned from both ends by the adapter test (header: the 2026-08-18 off-by-one that invented a device at index 21).

*class, declared at [`include/shulib/hal/pros/device_registry.hpp:89`](../../include/shulib/hal/pros/device_registry.hpp#L89).*

<a id="prosdeviceregistry-prosdeviceregistry"></a>

### `ProsDeviceRegistry::ProsDeviceRegistry`

```cpp
ProsDeviceRegistry() = default
```

Nothing to open, nothing to own — the registry is the brain's own table.

*function, declared at [`include/shulib/hal/pros/device_registry.hpp:92`](../../include/shulib/hal/pros/device_registry.hpp#L92).*

<a id="prosdeviceregistry-pluggedkind"></a>

### `ProsDeviceRegistry::pluggedKind`

```cpp
[[nodiscard]] DeviceKind pluggedKind(int port) const noexcept override
```

What PHYSICAL port `port` (1..kMaxSmartPorts) carries, right now. A port outside that range answers None WITHOUT touching PROS (header: uint8_t truncation would otherwise read a neighbouring or nonexistent index).

*function, declared at [`include/shulib/hal/pros/device_registry.hpp:97`](../../include/shulib/hal/pros/device_registry.hpp#L97).*

<a id="prosdeviceregistry-kindof"></a>

### `ProsDeviceRegistry::kindOf`

```cpp
[[nodiscard]] static DeviceKind kindOf(::pros::c::v5_device_e_t t) noexcept
```

PROS's device type as this library's DeviceKind. Every value of `v5_device_e_t` is named here on purpose — a device kind PROS adds later lands in `Other`, never in `Motor`, and the adapter test walks the whole enum so that stays true.

*function, declared at [`include/shulib/hal/pros/device_registry.hpp:116`](../../include/shulib/hal/pros/device_registry.hpp#L116).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 45 lines</summary>

```text

 ProsDeviceRegistry — IDeviceRegistry over PROS's smart-port registry (chunk R3d Part 2).
 The one place in the LIBRARY that converts a PHYSICAL port number into PROS's zero-indexed
 registry index.

 BINDS:
  * ::pros::c::registry_get_plugged_type(uint8_t index) — apix.h:640-660, returning
    v5_device_e_t. "Returns the type of the device plugged into the ZERO-INDEXED port …
    The V5 port number from 0-20." (HA-138)

 ═══ THE R1a-SHAPED TRAP, AND WHY THE CONVERSION IS ONE LINE ════════════════════════
 The registry is ZERO-INDEXED and NOTHING ELSE IN PROS IS — every motor, sensor and
 adapter in this tree takes a physical 1..21 port. src/programs/bench_r3a.cpp's registry comment (the
 one just above `kMaxRegistryIndex`) records the day that cost: the tester scanned 1..21 as
 if the index were physical, which shifted every reading by one port and invented a device
 at index 21 (physical 22, which does not exist). It produced a confident, completely wrong
 port map — and HA-120 had already predicted exactly that, because the 2026-08-13 "ADI
 expander" sighting came from index 21.
 The same two-conventions-in-one-API shape as HA-122's /usd/ prefix.

 So in the LIBRARY the conversion appears EXACTLY ONCE, on the line marked below, and
 test/pros_device_registry_adapter_test.cpp pins it FROM BOTH ENDS: physical port 1 must
 read registry index 0, and physical port 21 must read index 20. A test that only checked
 one end would pass with an off-by-one in the other direction. (src/programs/bench_r3a.cpp's census
 keeps its own copy, deliberately — see the note at the conversion line.)

 Out-of-range ports (0, 22, negatives) answer None WITHOUT calling PROS at all. That is not
 defensive decoration: `registry_get_plugged_type` takes a uint8_t, so a caller's 22 would
 be passed straight through and read whatever index 22 holds, and a negative would convert
 to a large index. The seam's contract (hal/device_registry.hpp) makes "not a port" mean
 "empty", and the screen is what makes that true.

 ═══ WHAT THIS ADAPTER DOES NOT DO ═════════════════════════════════════════════════
 It does not cache. A registry read is a table lookup in the PROS daemon's own state, and
 a cached census is a census that can be wrong about a cable pulled thirty seconds ago —
 which is precisely the situation the boot check exists to catch. Callers that need a
 consistent snapshot take one themselves, in one pass, and say so.

 It does not construct anything. That is the whole point: discovering what is plugged in
 must be cheaper than opening it (hal/device_registry.hpp's header).

 HA register: HA-138 (the zero-index mapping), and HA-120 for the sighting that predicted
 it. Host-tested through test/pros_shim/pros/apix.h, which transcribes the vendored
 v5_device_e_t values and the zero-indexed contract — and, like every shim, tests the
 adapter against our BELIEF about PROS rather than the belief itself.
```

</details>
