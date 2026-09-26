<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/device_provider.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `device_provider.hpp`

ProsDeviceProvider — hal::IDeviceProvider over the real V5 adapters.

This header declares **1** type (19 members).

Extracted from [`include/shulib/hal/pros/device_provider.hpp`](../../include/shulib/hal/pros/device_provider.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ProsDeviceProvider`](#class-prosdeviceprovider)
  - [`kSlots`](#prosdeviceprovider-kslots)
  - [`kRefusalBytes`](#prosdeviceprovider-krefusalbytes)
  - [`ProsDeviceProvider`](#prosdeviceprovider-prosdeviceprovider)
  - [`ProsDeviceProvider (overload 2)`](#prosdeviceprovider-prosdeviceprovider-2)
  - [`ProsDeviceProvider (overload 3)`](#prosdeviceprovider-prosdeviceprovider-3)
  - [`ProsDeviceProvider (overload 4)`](#prosdeviceprovider-prosdeviceprovider-4)
  - [`operator=`](#prosdeviceprovider-operator-eq)
  - [`operator= (overload 2)`](#prosdeviceprovider-operator-eq-2)
  - [`~ProsDeviceProvider`](#prosdeviceprovider-destructor-prosdeviceprovider)
  - [`motor`](#prosdeviceprovider-motor)
  - [`imu`](#prosdeviceprovider-imu)
  - [`gps`](#prosdeviceprovider-gps)
  - [`rotation`](#prosdeviceprovider-rotation)
  - [`digitalOut`](#prosdeviceprovider-digitalout)
  - [`clock`](#prosdeviceprovider-clock)
  - [`battery`](#prosdeviceprovider-battery)
  - [`telemetry`](#prosdeviceprovider-telemetry)
  - [`pacer`](#prosdeviceprovider-pacer)
  - [`lastRefusal`](#prosdeviceprovider-lastrefusal)

<a id="class-prosdeviceprovider"></a>

## `class ProsDeviceProvider`

```cpp
class ProsDeviceProvider final : public IDeviceProvider
```

The on-robot `hal::IDeviceProvider`: real adapters made on demand, one per port, each only after the brain's registry says the port carries the expected kind of device, and owned here for the life of the program (header). Refusal is `nullptr` plus a sentence naming the port; a conflicting second request for a made device is a precondition violation. Neither copyable nor movable — the graph holds pointers into this object's storage.

*class, declared at [`include/shulib/hal/pros/device_provider.hpp:136`](../../include/shulib/hal/pros/device_provider.hpp#L136).*

<a id="prosdeviceprovider-kslots"></a>

### `ProsDeviceProvider::kSlots`

```cpp
static constexpr std::size_t kSlots = kMaxSmartPorts + 1
```

One more than the highest physical port, so a port number indexes the storage directly and slot 0 is never used.

*field, declared at [`include/shulib/hal/pros/device_provider.hpp:140`](../../include/shulib/hal/pros/device_provider.hpp#L140).*

<a id="prosdeviceprovider-krefusalbytes"></a>

### `ProsDeviceProvider::kRefusalBytes`

```cpp
static constexpr std::size_t kRefusalBytes = 160
```

The longest refusal sentence this provider writes, NUL included. Sized so the builder's verdict (`robot::BuildVerdict::why`, 192 bytes, which prefixes a role such as "a RIGHT drive motor cannot be made: ") still carries the port and the start of the adapter's reason; longer text truncates, never overflows.

*field, declared at [`include/shulib/hal/pros/device_provider.hpp:146`](../../include/shulib/hal/pros/device_provider.hpp#L146).*

<a id="prosdeviceprovider-prosdeviceprovider"></a>

### `ProsDeviceProvider::ProsDeviceProvider`

```cpp
ProsDeviceProvider()
```

The provider as it runs on a brain: it owns a `ProsDeviceRegistry` and asks it, port by port, before constructing anything. Constructing the provider opens NO device — every adapter is made on its first request.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:151`](../../include/shulib/hal/pros/device_provider.hpp#L151).*

<a id="prosdeviceprovider-prosdeviceprovider-2"></a>

### `ProsDeviceProvider::ProsDeviceProvider (overload 2)`

```cpp
explicit ProsDeviceProvider(const IDeviceRegistry& registry)
```

The same provider over a caller's registry — a `hal::fake::FakeDeviceRegistry` in a host test that wants to program the census without the shim, or a program that already holds one. The registry is BORROWED and must outlive the provider.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:156`](../../include/shulib/hal/pros/device_provider.hpp#L156).*

<a id="prosdeviceprovider-prosdeviceprovider-3"></a>

### `ProsDeviceProvider::ProsDeviceProvider (overload 3)`

```cpp
ProsDeviceProvider(const ProsDeviceProvider&) = delete
```

Non-copyable: a copy would duplicate the adapters while the graph kept pointing at the originals.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:161`](../../include/shulib/hal/pros/device_provider.hpp#L161).*

<a id="prosdeviceprovider-prosdeviceprovider-4"></a>

### `ProsDeviceProvider::ProsDeviceProvider (overload 4)`

```cpp
ProsDeviceProvider(ProsDeviceProvider&&) = delete
```

Non-copyable: a copy would duplicate the adapters while the graph kept pointing at the originals. Non-movable, same reasoning.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:163`](../../include/shulib/hal/pros/device_provider.hpp#L163).*

<a id="prosdeviceprovider-operator-eq"></a>

### `ProsDeviceProvider::operator=`

```cpp
ProsDeviceProvider& operator=(const ProsDeviceProvider&) = delete
```

Non-copyable: a copy would duplicate the adapters while the graph kept pointing at the originals. Non-movable, same reasoning. Non-copy-assignable, same reasoning.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:165`](../../include/shulib/hal/pros/device_provider.hpp#L165).*

<a id="prosdeviceprovider-operator-eq-2"></a>

### `ProsDeviceProvider::operator= (overload 2)`

```cpp
ProsDeviceProvider& operator=(ProsDeviceProvider&&) = delete
```

Non-copyable: a copy would duplicate the adapters while the graph kept pointing at the originals. Non-movable, same reasoning. Non-copy-assignable, same reasoning. Non-move-assignable, same reasoning.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:167`](../../include/shulib/hal/pros/device_provider.hpp#L167).*

<a id="prosdeviceprovider-destructor-prosdeviceprovider"></a>

### `ProsDeviceProvider::~ProsDeviceProvider`

```cpp
~ProsDeviceProvider() override = default
```

Non-copyable: a copy would duplicate the adapters while the graph kept pointing at the originals. Non-movable, same reasoning. Non-copy-assignable, same reasoning. Non-move-assignable, same reasoning. Destroys every adapter this provider made — so it must outlive every graph built from it.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:169`](../../include/shulib/hal/pros/device_provider.hpp#L169).*

<a id="prosdeviceprovider-motor"></a>

### `ProsDeviceProvider::motor`

```cpp
[[nodiscard]] IMotor* motor(std::int8_t signedPort, MotorCartridge cartridge) override
```

The motor on `signedPort` (|p| in 1..21, NEGATIVE = reversed by PROS, once, inside the adapter), configured with `cartridge`'s gearset. Made on the first request the registry (kind MOTOR) and the adapter's read-back both allow; the same object ever after. nullptr + a refusal for a port outside 1..21, a cartridge byte that is not a V5 gearset, a port the registry says is empty or carries something else, or a motor that rejects its configuration. PRECONDITION: a later request for a made port states the SAME signed port and cartridge (header: one port, one configuration).

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:180`](../../include/shulib/hal/pros/device_provider.hpp#L180).*

<a id="prosdeviceprovider-imu"></a>

### `ProsDeviceProvider::imu`

```cpp
[[nodiscard]] IImu* imu(std::uint8_t port) override
```

The inertial sensor on `port`. Made on the first request the registry (kind IMU) allows, and its calibration STARTED right then, exactly once (header); the same object ever after. nullptr + a refusal for a port outside 1..21 or a port the registry says is empty or carries something else. PRECONDITION: this provider serves ONE IMU, so a request for a different port after one was made is a programming error.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:229`](../../include/shulib/hal/pros/device_provider.hpp#L229).*

<a id="prosdeviceprovider-gps"></a>

### `ProsDeviceProvider::gps`

```cpp
[[nodiscard]] IGps* gps(std::uint8_t port, units::Length leverArmForward, units::Length leverArmLeft) override
```

The GPS on `port`, with its lever arm from the robot centre (+X forward, +Y left, inches), which the adapter removes once, at the edge. Made on the first request the registry (kind GPS) and the adapter's own boot check both allow; the same object ever after. nullptr + a refusal for a port outside 1..21, a port the registry says is empty or carries something else, or a GPS the adapter rejects (a configured firmware offset, HA-06). PRECONDITION: ONE GPS per provider, and a later request states the same port and the same lever arm.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:268`](../../include/shulib/hal/pros/device_provider.hpp#L268).*

<a id="prosdeviceprovider-rotation"></a>

### `ProsDeviceProvider::rotation`

```cpp
[[nodiscard]] IRotation* rotation(std::uint8_t port) override
```

The rotation sensor on `port` — a tracking wheel's encoder, never reversed by port (the seam's port is unsigned; a pod's direction is its offset's sign). Made on the first request the registry (kind ROTATION) allows; the same object ever after. nullptr + a refusal for a port outside 1..21 or a port the registry says is empty or carries something else.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:311`](../../include/shulib/hal/pros/device_provider.hpp#L311).*

<a id="prosdeviceprovider-digitalout"></a>

### `ProsDeviceProvider::digitalOut`

```cpp
[[nodiscard]] IDigitalOut* digitalOut(std::uint8_t line, bool initialState) override
```

The digital out on the brain's 3-wire port `line` (1..8 = A..H) — a solenoid's line. Made on the first request, and MAKING IT DRIVES IT to `initialState` (ProsDigitalOut's constructor; HA-119); the same object ever after, never driven again by a request. nullptr + a refusal ONLY for a line outside 1..8 — the registry is not asked because it cannot see a solenoid (header; HA-158), so an in-range line is always made. PRECONDITION: a later request for a made line states the same `initialState` (one line, one configuration).

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:341`](../../include/shulib/hal/pros/device_provider.hpp#L341).*

<a id="prosdeviceprovider-clock"></a>

### `ProsDeviceProvider::clock`

```cpp
[[nodiscard]] IClock& clock() override
```

The V5's real clock (`ProsClock`, microseconds → seconds once). Never refused.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:368`](../../include/shulib/hal/pros/device_provider.hpp#L368).*

<a id="prosdeviceprovider-battery"></a>

### `ProsDeviceProvider::battery`

```cpp
[[nodiscard]] IBattery& battery() override
```

The V5 battery (`ProsBattery`). Never refused.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:370`](../../include/shulib/hal/pros/device_provider.hpp#L370).*

<a id="prosdeviceprovider-telemetry"></a>

### `ProsDeviceProvider::telemetry`

```cpp
[[nodiscard]] ITelemetrySink& telemetry() override
```

The diagnostics sink: a `diag::TermSink` over the USB serial (`ProsCharSink`, stdout) — exactly the sink src/main.cpp's hand-wired graphs built (the invented X-drive's still does). Never refused.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:374`](../../include/shulib/hal/pros/device_provider.hpp#L374).*

<a id="prosdeviceprovider-pacer"></a>

### `ProsDeviceProvider::pacer`

```cpp
[[nodiscard]] motion::ITickPacer& pacer() override
```

The 10 ms `ProsTickPacer` (`pros::Task::delay_until`, anchored on its first pace). Never refused.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:377`](../../include/shulib/hal/pros/device_provider.hpp#L377).*

<a id="prosdeviceprovider-lastrefusal"></a>

### `ProsDeviceProvider::lastRefusal`

```cpp
[[nodiscard]] const char* lastRefusal() const noexcept override
```

Why the most recent refused request returned nullptr, naming the port and — when the registry answered — what was found and what was expected. Plain ASCII. The empty string until something is refused; not cleared by a later success (the builder reads it immediately after the nullptr it explains).

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:383`](../../include/shulib/hal/pros/device_provider.hpp#L383).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 102 lines, click to expand</summary>

```text

 ProsDeviceProvider — hal::IDeviceProvider over the real V5 adapters (chunk G1 Part 2a,
 2026-09-24).

 ── What it is ──────────────────────────────────────────────────────────────────────
 The one object on the brain that knows what a port is. robot::Robot asks it for "the motor
 on port -11 with a blue cartridge", "the IMU on port 2", and it answers with a hal/pros
 adapter it constructed and OWNS (ruling B2: the provider owns, the graph borrows) — or with
 nullptr and a sentence in lastRefusal() naming the port. On the host the same seam is
 served by hal::fake::FakeDeviceProvider; the builder cannot tell the two apart, which is the
 point of the seam (hal/device_provider.hpp).

 ── THE REGISTRY IS ASKED BEFORE ANY ADAPTER IS CONSTRUCTED ──────────────────────────
 The G1 Part 2 brief's first version assumed a provider could refuse a dead port by catching
 the adapter's throw. The 2026-09-24 audit measured that false: only ProsMotor fails loudly on
 a port with nothing in it (its HA-98 read-back). ProsImu's and ProsRotation's constructors
 cannot throw at all, and ProsGps DEFERS its offset check on a device it cannot read. A
 provider that refused only on a throw would hand back a LIVE adapter for an EMPTY IMU, GPS or
 rotation port — and the robot would boot, calibrate nothing, and read a heading of zero
 forever.

 So before a port device is constructed, this provider asks an hal::IDeviceRegistry what is
 physically on that port (hal/device_registry.hpp — the brain's own table, read without
 opening anything). Any answer but the expected kind, EMPTY included, is a refusal naming the
 port, what was found and what was expected:

     port 13: nothing plugged in (expected a motor)
     port 2: a rotation sensor (expected an IMU)

 The registry's KIND answer being right for an IMU, a rotation sensor and a GPS is a hardware
 belief this file newly rests on — A4 register HA-141 (HA-138 states only the port indexing,
 which ProsDeviceRegistry owns). Reasoned, not measured: for those three kinds the registry is
 now the ONLY thing that can refuse an empty port, so its settle step is on the register.

 ── …AND THE ADAPTER'S OWN CHECK STILL RUNS ──────────────────────────────────────────
 The registry says WHAT is plugged in; it cannot say whether a motor ACCEPTS its
 configuration. ProsMotor's constructor sets degrees and the stated gearset and reads both
 back (HA-98). A motor whose kind is right but whose read-back fails is refused too, carrying
 the adapter's own message:

     port 13: the motor rejected its configuration: ProsMotor: encoder units read-back ...

 Two checks, both kept (the brief's ruling M2): the registry catches an empty or swapped port
 before anything is opened; the adapter catches a present device that will not be configured.
 Each construction sits inside its OWN try, so one bad port is one refusal and never an
 exception out of the builder. The adapter's message is copied through an ASCII filter,
 because a refusal ends up on the brain's screen and its fonts are ASCII-only (ProsMotor's
 messages carry an em dash, which the filter turns into '-').

 ── ONE PORT, ONE DEVICE, ONE CONFIGURATION ─────────────────────────────────────────
 A device is constructed on the FIRST request that the registry and the adapter both allow,
 and every later request for that port returns the SAME object — the graph holds these
 pointers for the life of the robot, and a second adapter on one port would leave half of it
 reading an object nobody commands (IDeviceProvider's idempotence contract). The registry is
 consulted when a device is MADE, not on every later request: once it exists it belongs to the
 graph, and losing it at runtime is the adapter's sentinel screening and the health layer's
 business, not a reason to hand the builder a different answer the second time it asks.

 Asking for a port that already has a device with a DIFFERENT configuration — the motor with
 the other sign or another cartridge, the GPS with another lever arm, a second IMU or GPS on
 another port — is a PRECONDITION VIOLATION, not a refusal. It cannot be a wiring fault (the
 wiring did not change between the two calls); it is two parts of one program disagreeing about
 one port, and quietly returning the first configuration would apply the wrong sign to a drive
 side without a word.

 ── THE 3-WIRE LINES: NO REGISTRY, AND MAKING ONE DRIVES IT (T3's addendum B2) ──────
 digitalOut(line, initialState) makes a ProsDigitalOut on the brain's own 3-wire port `line`
 (1..8 = A..H). The registry is NOT asked, because it cannot answer: the eight 3-wire ports are
 not smart ports, a solenoid on one has no digital identity, and nothing on the brain can tell a
 connected valve from an empty port (A4 register HA-158, reasoned). So the only refusal is a line
 outside 1..8; an in-range line is always made — and MADE MEANS DRIVEN: ProsDigitalOut's
 constructor sets the line to `initialState` (HA-119), which is why the level is an argument and
 why a later request for the same line returns the same object WITHOUT driving it again. A later
 request with a different initial state is a precondition, like a motor's sign. There is no `try`
 around the construction: the vendored DigitalOut constructor reports a bad port through errno
 (ENXIO, vendored adi.hpp:539-542), never by throwing, and the range is checked before it runs.

 ── IMU CALIBRATION STARTS HERE, EXACTLY ONCE ───────────────────────────────────────
 imu() starts the sensor's calibration (ProsImu::calibrate(), non-blocking — HA-108) when it
 constructs the adapter, and never again: a second reset() would re-zero the sensor under a
 live boot heading (HA-05), which ProsImu refuses by precondition. A program built on this
 provider therefore never calls calibrate() itself. The boot heading is 0 — the robot boots
 believing it faces +X — which is what src/main.cpp's hand-wired graphs did (HA-05's one
 owner is the start pose; a provider option for it arrives when a program needs one).

 ── WHAT IT DOES NOT DO ─────────────────────────────────────────────────────────────
 It applies NO dead-port policy. A port the boot check reported MISSING is never requested
 (robot::BuildOptions::absentDrivePorts); anything else it will not make refuses the build,
 through the builder, naming the port. It never hands out a fake — a test double in a
 competition binary erases the difference between "no device" and "a test injects here" (R3b
 §6). It allocates nothing: every device lives in fixed per-port std::optional storage. The
 controller and the controller's screen are not on the seam; a program keeps its own.

 ── NO HARDWARE CLAIM ───────────────────────────────────────────────────────────────
 Host-tested through test/pros_shim/ only, which tests this class against our BELIEF about
 PROS and cannot test the belief (shim_control.hpp says so in full). Since G1 Part 2b one
 program constructs one — "shulib Teleop" (src/teleop_program.cpp) — and that version of the
 program has not run on hardware, so no provider has yet made a device on a brain.

 No pros header is included here directly — only the hal/pros adapters, whose own diagnostic
 fences cover the vendored code (test/pros_adapter_fence_test.cpp requires a file with no
 direct pros include to carry no fence at all).
```

</details>
