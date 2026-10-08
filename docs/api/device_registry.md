<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/device_registry.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `device_registry.hpp`

IDeviceRegistry — WHAT IS ACTUALLY PLUGGED INTO EACH SMART PORT.

This header declares **2** types (18 members) and **2** free functions.

Extracted from [`include/shulib/hal/device_registry.hpp`](../../include/shulib/hal/device_registry.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class DeviceKind`](#enum-class-devicekind)
  - [`None`](#devicekind-none)
  - [`Motor`](#devicekind-motor)
  - [`Imu`](#devicekind-imu)
  - [`Rotation`](#devicekind-rotation)
  - [`Distance`](#devicekind-distance)
  - [`Optical`](#devicekind-optical)
  - [`Vision`](#devicekind-vision)
  - [`Gps`](#devicekind-gps)
  - [`Radio`](#devicekind-radio)
  - [`Adi`](#devicekind-adi)
  - [`Other`](#devicekind-other)
- [`deviceKindWord`](#devicekindword) — *free function*
- [`class IDeviceRegistry`](#class-ideviceregistry)
  - [`~IDeviceRegistry`](#ideviceregistry-destructor-ideviceregistry)
  - [`IDeviceRegistry`](#ideviceregistry-ideviceregistry)
  - [`IDeviceRegistry (overload 2)`](#ideviceregistry-ideviceregistry-2)
  - [`IDeviceRegistry (overload 3)`](#ideviceregistry-ideviceregistry-3)
  - [`operator=`](#ideviceregistry-operator-eq)
  - [`operator= (overload 2)`](#ideviceregistry-operator-eq-2)
  - [`pluggedKind`](#ideviceregistry-pluggedkind)
- [`isSmartPort`](#issmartport) — *free function*

<a id="enum-class-devicekind"></a>

## `enum class DeviceKind`

```cpp
enum class DeviceKind : std::uint8_t
```

What kind of device a smart port carries. Ours, not PROS's (header): the kinds this tree has adapters for, plus Other for a device it knows nothing about. `None` is an EMPTY port — no device, or a port outside 1..kMaxSmartPorts, which answers None without asking the platform.

*enum class, declared at [`include/shulib/hal/device_registry.hpp:50`](../../include/shulib/hal/device_registry.hpp#L50).*

<a id="devicekind-none"></a>

### `DeviceKind::None`

```cpp
None
```

nothing plugged in (or the port number is out of range)

*enumerator, declared at [`include/shulib/hal/device_registry.hpp:51`](../../include/shulib/hal/device_registry.hpp#L51).*

<a id="devicekind-motor"></a>

### `DeviceKind::Motor`

```cpp
Motor
```

a V5 smart motor — hal/pros/motor.hpp

*enumerator, declared at [`include/shulib/hal/device_registry.hpp:52`](../../include/shulib/hal/device_registry.hpp#L52).*

<a id="devicekind-imu"></a>

### `DeviceKind::Imu`

```cpp
Imu
```

an inertial sensor — hal/pros/imu.hpp

*enumerator, declared at [`include/shulib/hal/device_registry.hpp:53`](../../include/shulib/hal/device_registry.hpp#L53).*

<a id="devicekind-rotation"></a>

### `DeviceKind::Rotation`

```cpp
Rotation
```

a rotation sensor — hal/pros/rotation.hpp

*enumerator, declared at [`include/shulib/hal/device_registry.hpp:54`](../../include/shulib/hal/device_registry.hpp#L54).*

<a id="devicekind-distance"></a>

### `DeviceKind::Distance`

```cpp
Distance
```

a distance sensor — hal/pros/distance.hpp

*enumerator, declared at [`include/shulib/hal/device_registry.hpp:55`](../../include/shulib/hal/device_registry.hpp#L55).*

<a id="devicekind-optical"></a>

### `DeviceKind::Optical`

```cpp
Optical
```

an optical sensor — hal/pros/optical.hpp

*enumerator, declared at [`include/shulib/hal/device_registry.hpp:56`](../../include/shulib/hal/device_registry.hpp#L56).*

<a id="devicekind-vision"></a>

### `DeviceKind::Vision`

```cpp
Vision
```

a vision sensor (including AI Vision) — no adapter yet; R2 owns the camera

*enumerator, declared at [`include/shulib/hal/device_registry.hpp:57`](../../include/shulib/hal/device_registry.hpp#L57).*

<a id="devicekind-gps"></a>

### `DeviceKind::Gps`

```cpp
Gps
```

a GPS sensor — hal/pros/gps.hpp

*enumerator, declared at [`include/shulib/hal/device_registry.hpp:58`](../../include/shulib/hal/device_registry.hpp#L58).*

<a id="devicekind-radio"></a>

### `DeviceKind::Radio`

```cpp
Radio
```

the VEXnet radio

*enumerator, declared at [`include/shulib/hal/device_registry.hpp:59`](../../include/shulib/hal/device_registry.hpp#L59).*

<a id="devicekind-adi"></a>

### `DeviceKind::Adi`

```cpp
Adi
```

an ADI (three-wire) expander — hal/pros/digital_in.hpp's pair form

*enumerator, declared at [`include/shulib/hal/device_registry.hpp:60`](../../include/shulib/hal/device_registry.hpp#L60).*

<a id="devicekind-other"></a>

### `DeviceKind::Other`

```cpp
Other
```

a device this library has no name for (serial, undefined, anything new)

*enumerator, declared at [`include/shulib/hal/device_registry.hpp:61`](../../include/shulib/hal/device_registry.hpp#L61).*

<a id="devicekindword"></a>

## `deviceKindWord`

```cpp
[[nodiscard]] inline const char* deviceKindWord(DeviceKind k) noexcept
```

The kind as one short word, for a census line or a refusal message. Never invents: an unknown byte reads "?".

*free function, declared at [`include/shulib/hal/device_registry.hpp:66`](../../include/shulib/hal/device_registry.hpp#L66).*

<a id="class-ideviceregistry"></a>

## `class IDeviceRegistry`

```cpp
class IDeviceRegistry
```

The seam that answers "what is plugged into this port?" without opening anything. Stated in PHYSICAL port numbers, 1..kMaxSmartPorts — the zero-indexed PROS call it is implemented over is the adapter's business and nobody else's (header). Non-owning and const: a registry is asked, never told.

*class, declared at [`include/shulib/hal/device_registry.hpp:87`](../../include/shulib/hal/device_registry.hpp#L87).*

<a id="ideviceregistry-destructor-ideviceregistry"></a>

### `IDeviceRegistry::~IDeviceRegistry`

```cpp
virtual ~IDeviceRegistry() = default
```

The polymorphic-base special members: a virtual destructor so a registry may be owned and destroyed through this interface, and defaulted copy/move because the interface itself holds no state.

*function, declared at [`include/shulib/hal/device_registry.hpp:92`](../../include/shulib/hal/device_registry.hpp#L92).*

<a id="ideviceregistry-ideviceregistry"></a>

### `IDeviceRegistry::IDeviceRegistry`

```cpp
IDeviceRegistry() = default
```

*Covered by the comment on [`~IDeviceRegistry`](#ideviceregistry-destructor-ideviceregistry) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/device_registry.hpp:93`](../../include/shulib/hal/device_registry.hpp#L93).*

<a id="ideviceregistry-ideviceregistry-2"></a>

### `IDeviceRegistry::IDeviceRegistry (overload 2)`

```cpp
IDeviceRegistry(const IDeviceRegistry&) = default
```

*Covered by the comment on [`~IDeviceRegistry`](#ideviceregistry-destructor-ideviceregistry) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/device_registry.hpp:94`](../../include/shulib/hal/device_registry.hpp#L94).*

<a id="ideviceregistry-ideviceregistry-3"></a>

### `IDeviceRegistry::IDeviceRegistry (overload 3)`

```cpp
IDeviceRegistry(IDeviceRegistry&&) = default
```

*Covered by the comment on [`~IDeviceRegistry`](#ideviceregistry-destructor-ideviceregistry) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/device_registry.hpp:95`](../../include/shulib/hal/device_registry.hpp#L95).*

<a id="ideviceregistry-operator-eq"></a>

### `IDeviceRegistry::operator=`

```cpp
IDeviceRegistry& operator=(const IDeviceRegistry&) = default
```

*Covered by the comment on [`~IDeviceRegistry`](#ideviceregistry-destructor-ideviceregistry) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/device_registry.hpp:96`](../../include/shulib/hal/device_registry.hpp#L96).*

<a id="ideviceregistry-operator-eq-2"></a>

### `IDeviceRegistry::operator= (overload 2)`

```cpp
IDeviceRegistry& operator=(IDeviceRegistry&&) = default
```

*Covered by the comment on [`~IDeviceRegistry`](#ideviceregistry-destructor-ideviceregistry) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/device_registry.hpp:97`](../../include/shulib/hal/device_registry.hpp#L97).*

<a id="ideviceregistry-pluggedkind"></a>

### `IDeviceRegistry::pluggedKind`

```cpp
[[nodiscard]] virtual DeviceKind pluggedKind(int port) const noexcept = 0
```

What PHYSICAL port `port` (1..kMaxSmartPorts) currently carries. Out of range answers None without asking the platform — a loop that walks 0..21, or 1..22, gets an honest "empty" rather than a shifted reading or a phantom device (header, and HA-120's 2026-08-18 finding). MUST NOT throw.

*function, declared at [`include/shulib/hal/device_registry.hpp:103`](../../include/shulib/hal/device_registry.hpp#L103).*

<a id="issmartport"></a>

## `isSmartPort`

```cpp
[[nodiscard]] inline constexpr bool isSmartPort(int port) noexcept
```

True when `port` is a physical smart port number a registry may be asked about. The ONE place the 1..kMaxSmartPorts range is spelled for this seam, so the fake, the adapter and every caller agree about the boundary.

*free function, declared at [`include/shulib/hal/device_registry.hpp:109`](../../include/shulib/hal/device_registry.hpp#L109).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 37 lines</summary>

```text

 IDeviceRegistry — WHAT IS ACTUALLY PLUGGED INTO EACH SMART PORT (chunk R3d Part 2).

 ── Why the library needs this at all ───────────────────────────────────────────────
 Every other HAL seam asks a device a question. This one asks the BRAIN a question the
 devices cannot answer: which ports have something in them, and what kind of something.
 Two things in R3d need exactly that and nothing more:
   * "shulib Calibrate" builds its candidate list from the ports that carry a MOTOR,
     because a calibration that starts from a typed list can only ever confirm the typing;
   * a program that boots from the record ("shulib Drive", the competition program, the Auton
     Tester) checks a calibration record against the ports BEFORE it constructs a
     single adapter, so a card carried over from another robot, or a robot re-plugged
     since the calibration, is refused instead of driven (calibration/boot_check.hpp).
 Neither wants to open a device to find out. Constructing an adapter to discover whether
 a device exists is how HA-111's invented port map turned into ten throwing constructors
 at boot: the probe must be cheaper than the thing it is probing for.

 ── PHYSICAL PORTS, 1..21 — and the trap this seam exists to contain ────────────────
 PROS's `registry_get_plugged_type()` is ZERO-INDEXED and nothing else in PROS is
 (apix.h: "the zero-indexed port … The V5 port number from 0-20"). Scanning 1..21 as if it
 were physical shifts every reading by one port and invents a device at index 21 — which
 is exactly what happened on 2026-08-18 and produced a confident, completely wrong port
 map, and what HA-120 had already predicted. THIS INTERFACE IS STATED IN PHYSICAL PORTS,
 1..kMaxSmartPorts, like every other port number a human or any other PROS API uses; the
 conversion lives in ONE line, in hal/pros/device_registry.hpp, and is tested from both
 ends. A port outside 1..21 answers None WITHOUT asking the platform — so a loop that
 walks 0..21 by mistake cannot reach past the end.

 ── DeviceKind is OURS, not PROS's ──────────────────────────────────────────────────
 A small enum of the kinds this tree has adapters for, plus Other for everything else.
 Deliberately not a re-export of `v5_device_e_t`: that would put a PROS type in the
 library's vocabulary (the PROS-free rule) and would make every consumer switch over
 values it has no opinion about. The mapping is one function in the adapter, and the
 adapter's test walks EVERY v5_device_e_t value so a new PROS device kind cannot silently
 become a motor.

 PROS-free, allocation-free, const — a registry is asked, never told.
```

</details>
