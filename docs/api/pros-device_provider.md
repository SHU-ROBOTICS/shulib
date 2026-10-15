<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/device_provider.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `device_provider.hpp`

ProsDeviceProvider — hal::IDeviceProvider over the real V5 adapters.

This header declares **1** type (29 members).

Extracted from [`include/shulib/hal/pros/device_provider.hpp`](../../include/shulib/hal/pros/device_provider.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ProsDeviceProvider`](#class-prosdeviceprovider)
  - [`kSlots`](#prosdeviceprovider-kslots)
  - [`kRefusalBytes`](#prosdeviceprovider-krefusalbytes)
  - [`ProsDeviceProvider`](#prosdeviceprovider-prosdeviceprovider)
  - [`ProsDeviceProvider (overload 2)`](#prosdeviceprovider-prosdeviceprovider-2)
  - [`ProsDeviceProvider (overload 3)`](#prosdeviceprovider-prosdeviceprovider-3)
  - [`ProsDeviceProvider (overload 4)`](#prosdeviceprovider-prosdeviceprovider-4)
  - [`ProsDeviceProvider (overload 5)`](#prosdeviceprovider-prosdeviceprovider-5)
  - [`operator=`](#prosdeviceprovider-operator-eq)
  - [`operator= (overload 2)`](#prosdeviceprovider-operator-eq-2)
  - [`~ProsDeviceProvider`](#prosdeviceprovider-destructor-prosdeviceprovider)
  - [`motor`](#prosdeviceprovider-motor)
  - [`imu`](#prosdeviceprovider-imu)
  - [`gps`](#prosdeviceprovider-gps)
  - [`rotation`](#prosdeviceprovider-rotation)
  - [`digitalOut`](#prosdeviceprovider-digitalout)
  - [`distance`](#prosdeviceprovider-distance)
  - [`optical`](#prosdeviceprovider-optical)
  - [`digitalIn`](#prosdeviceprovider-digitalin)
  - [`readFaults`](#prosdeviceprovider-readfaults)
  - [`lineReadFaults`](#prosdeviceprovider-linereadfaults)
  - [`clock`](#prosdeviceprovider-clock)
  - [`battery`](#prosdeviceprovider-battery)
  - [`telemetry`](#prosdeviceprovider-telemetry)
  - [`serialCopyOpen`](#prosdeviceprovider-serialcopyopen)
  - [`serialCopyPath`](#prosdeviceprovider-serialcopypath)
  - [`serialCopyDroppedLines`](#prosdeviceprovider-serialcopydroppedlines)
  - [`serialCopyAppendOpen`](#prosdeviceprovider-serialcopyappendopen)
  - [`pacer`](#prosdeviceprovider-pacer)
  - [`lastRefusal`](#prosdeviceprovider-lastrefusal)

<a id="class-prosdeviceprovider"></a>

## `class ProsDeviceProvider`

```cpp
class ProsDeviceProvider final : public IDeviceProvider
```

The on-robot `hal::IDeviceProvider`: real adapters made on demand, one per port, each only after the brain's registry says the port carries the expected kind of device, and owned here for the life of the program (header). Refusal is `nullptr` plus a sentence naming the port; a conflicting second request for a made device is a precondition violation. Neither copyable nor movable — the graph holds pointers into this object's storage.

*class, declared at [`include/shulib/hal/pros/device_provider.hpp:183`](../../include/shulib/hal/pros/device_provider.hpp#L183).*

<a id="prosdeviceprovider-kslots"></a>

### `ProsDeviceProvider::kSlots`

```cpp
static constexpr std::size_t kSlots = kMaxSmartPorts + 1
```

One more than the highest physical port, so a port number indexes the storage directly and slot 0 is never used.

*field, declared at [`include/shulib/hal/pros/device_provider.hpp:187`](../../include/shulib/hal/pros/device_provider.hpp#L187).*

<a id="prosdeviceprovider-krefusalbytes"></a>

### `ProsDeviceProvider::kRefusalBytes`

```cpp
static constexpr std::size_t kRefusalBytes = 160
```

The longest refusal sentence this provider writes, NUL included. Sized so the builder's verdict (`robot::BuildVerdict::why`, 192 bytes, which prefixes a role such as "a RIGHT drive motor cannot be made: ") still carries the port and the start of the adapter's reason; longer text truncates, never overflows.

*field, declared at [`include/shulib/hal/pros/device_provider.hpp:193`](../../include/shulib/hal/pros/device_provider.hpp#L193).*

<a id="prosdeviceprovider-prosdeviceprovider"></a>

### `ProsDeviceProvider::ProsDeviceProvider`

```cpp
ProsDeviceProvider()
```

The provider as it runs on a brain: it owns a `ProsDeviceRegistry` and asks it, port by port, before constructing anything. Constructing the provider opens NO device — every adapter is made on its first request.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:198`](../../include/shulib/hal/pros/device_provider.hpp#L198).*

<a id="prosdeviceprovider-prosdeviceprovider-2"></a>

### `ProsDeviceProvider::ProsDeviceProvider (overload 2)`

```cpp
explicit ProsDeviceProvider(const IDeviceRegistry& registry)
```

The same provider over a caller's registry — a `hal::fake::FakeDeviceRegistry` in a host test that wants to program the census without the shim, or a program that already holds one. The registry is BORROWED and must outlive the provider.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:203`](../../include/shulib/hal/pros/device_provider.hpp#L203).*

<a id="prosdeviceprovider-prosdeviceprovider-3"></a>

### `ProsDeviceProvider::ProsDeviceProvider (overload 3)`

```cpp
explicit ProsDeviceProvider(const char* serialCopyFile, ProsFileMode copyMode = ProsFileMode::Truncate, const char* copyMountRoot = "/usd/")
```

The provider as it runs on a brain, keeping a COPY of its log on the SD card (header: AN OPTIONAL COPY OF THE LOG): `serialCopyFile` is a BARE file name ("drive_log.txt"; the adapter owns the /usd/ prefix), opened in `copyMode` (Truncate: one file per boot; Append: the file kept across boots). A null `serialCopyFile` is exactly the default constructor — no copy, telemetry() the serial TermSink. `copyMountRoot` is the card's mount point (it must end in '/'), injectable so a host test writes the same path to a scratch directory. A missing card is NOT a refusal: the copy refuses its lines and counts them, and serial is unchanged. PRECONDITION: a non-null `serialCopyFile` is non-empty and does not start with '/' (ProsBlockSink's own rule).

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:215`](../../include/shulib/hal/pros/device_provider.hpp#L215).*

<a id="prosdeviceprovider-prosdeviceprovider-4"></a>

### `ProsDeviceProvider::ProsDeviceProvider (overload 4)`

```cpp
ProsDeviceProvider(const ProsDeviceProvider&) = delete
```

Non-copyable: a copy would duplicate the adapters while the graph kept pointing at the originals.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:229`](../../include/shulib/hal/pros/device_provider.hpp#L229).*

<a id="prosdeviceprovider-prosdeviceprovider-5"></a>

### `ProsDeviceProvider::ProsDeviceProvider (overload 5)`

```cpp
ProsDeviceProvider(ProsDeviceProvider&&) = delete
```

Non-copyable: a copy would duplicate the adapters while the graph kept pointing at the originals. Non-movable, same reasoning.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:231`](../../include/shulib/hal/pros/device_provider.hpp#L231).*

<a id="prosdeviceprovider-operator-eq"></a>

### `ProsDeviceProvider::operator=`

```cpp
ProsDeviceProvider& operator=(const ProsDeviceProvider&) = delete
```

Non-copyable: a copy would duplicate the adapters while the graph kept pointing at the originals. Non-movable, same reasoning. Non-copy-assignable, same reasoning.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:233`](../../include/shulib/hal/pros/device_provider.hpp#L233).*

<a id="prosdeviceprovider-operator-eq-2"></a>

### `ProsDeviceProvider::operator= (overload 2)`

```cpp
ProsDeviceProvider& operator=(ProsDeviceProvider&&) = delete
```

Non-copyable: a copy would duplicate the adapters while the graph kept pointing at the originals. Non-movable, same reasoning. Non-copy-assignable, same reasoning. Non-move-assignable, same reasoning.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:235`](../../include/shulib/hal/pros/device_provider.hpp#L235).*

<a id="prosdeviceprovider-destructor-prosdeviceprovider"></a>

### `ProsDeviceProvider::~ProsDeviceProvider`

```cpp
~ProsDeviceProvider() override = default
```

Non-copyable: a copy would duplicate the adapters while the graph kept pointing at the originals. Non-movable, same reasoning. Non-copy-assignable, same reasoning. Non-move-assignable, same reasoning. Destroys every adapter this provider made — so it must outlive every graph built from it.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:237`](../../include/shulib/hal/pros/device_provider.hpp#L237).*

<a id="prosdeviceprovider-motor"></a>

### `ProsDeviceProvider::motor`

```cpp
[[nodiscard]] IMotor* motor(std::int8_t signedPort, MotorCartridge cartridge) override
```

The motor on `signedPort` (|p| in 1..21, NEGATIVE = reversed by PROS, once, inside the adapter), configured with `cartridge`'s gearset. Made on the first request the registry (kind MOTOR) and the adapter's read-back both allow; the same object ever after. nullptr + a refusal for a port outside 1..21, a cartridge byte that is not a V5 gearset, a port the registry says is empty or carries something else, or a motor that rejects its configuration. PRECONDITION: a later request for a made port states the SAME signed port and cartridge (header: one port, one configuration).

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:248`](../../include/shulib/hal/pros/device_provider.hpp#L248).*

<a id="prosdeviceprovider-imu"></a>

### `ProsDeviceProvider::imu`

```cpp
[[nodiscard]] IImu* imu(std::uint8_t port) override
```

The inertial sensor on `port`. Made on the first request the registry (kind IMU) allows, and its calibration STARTED right then, exactly once (header); the same object ever after. nullptr + a refusal for a port outside 1..21 or a port the registry says is empty or carries something else. PRECONDITION: this provider serves ONE IMU, so a request for a different port after one was made is a programming error.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:297`](../../include/shulib/hal/pros/device_provider.hpp#L297).*

<a id="prosdeviceprovider-gps"></a>

### `ProsDeviceProvider::gps`

```cpp
[[nodiscard]] IGps* gps(std::uint8_t port, units::Length leverArmForward, units::Length leverArmLeft) override
```

The GPS on `port`, with its lever arm from the robot centre (+X forward, +Y left, inches), which the adapter removes once, at the edge. Made on the first request the registry (kind GPS) and the adapter's own boot check both allow; the same object ever after. nullptr + a refusal for a port outside 1..21, a port the registry says is empty or carries something else, or a GPS the adapter rejects (a configured firmware offset, HA-06). PRECONDITION: ONE GPS per provider, and a later request states the same port and the same lever arm.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:336`](../../include/shulib/hal/pros/device_provider.hpp#L336).*

<a id="prosdeviceprovider-rotation"></a>

### `ProsDeviceProvider::rotation`

```cpp
[[nodiscard]] IRotation* rotation(std::uint8_t port) override
```

The rotation sensor on `port` — a tracking wheel's encoder, never reversed by port (the seam's port is unsigned; a pod's direction is its offset's sign). Made on the first request the registry (kind ROTATION) allows; the same object ever after. nullptr + a refusal for a port outside 1..21 or a port the registry says is empty or carries something else.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:379`](../../include/shulib/hal/pros/device_provider.hpp#L379).*

<a id="prosdeviceprovider-digitalout"></a>

### `ProsDeviceProvider::digitalOut`

```cpp
[[nodiscard]] IDigitalOut* digitalOut(std::uint8_t line, bool initialState) override
```

The digital out on the brain's 3-wire port `line` (1..8 = A..H) — a solenoid's line. Made on the first request, and MAKING IT DRIVES IT to `initialState` (ProsDigitalOut's constructor; HA-119); the same object ever after, never driven again by a request. nullptr + a refusal ONLY for a line outside 1..8 — the registry is not asked because it cannot see a solenoid (header; HA-158), so an in-range line is always made. PRECONDITION: a later request for a made line states the same `initialState` (one line, one configuration), and the line was not made as a digitalIn() (one line, one direction — MECH-A1).

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:410`](../../include/shulib/hal/pros/device_provider.hpp#L410).*

<a id="prosdeviceprovider-distance"></a>

### `ProsDeviceProvider::distance`

```cpp
[[nodiscard]] IDistance* distance(std::uint8_t port) override
```

MECH-A1: the distance sensor on `port` — a mechanism's piece sensor. Made on the first request the registry (kind DISTANCE) allows; the same object ever after. nullptr + a refusal for a port outside 1..21 or a port the registry says is empty or carries something else (header).

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:441`](../../include/shulib/hal/pros/device_provider.hpp#L441).*

<a id="prosdeviceprovider-optical"></a>

### `ProsDeviceProvider::optical`

```cpp
[[nodiscard]] IOptical* optical(std::uint8_t port) override
```

MECH-A1: the optical sensor on `port` — a mechanism's piece sensor; as distance(), with the registry's kind OPTICAL.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:467`](../../include/shulib/hal/pros/device_provider.hpp#L467).*

<a id="prosdeviceprovider-digitalin"></a>

### `ProsDeviceProvider::digitalIn`

```cpp
[[nodiscard]] IDigitalIn* digitalIn(std::uint8_t line) override
```

MECH-A1: the digital input on the brain's 3-wire port `line` (1..8 = A..H) — a limit switch. Made on the first request; the registry is NOT asked (it cannot see a switch, HA-158), so an in-range line is always made, and making it drives nothing. nullptr + a refusal ONLY for a line outside 1..8. PRECONDITION: the line was not made as a digitalOut() (one line, one direction).

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:496`](../../include/shulib/hal/pros/device_provider.hpp#L496).*

<a id="prosdeviceprovider-readfaults"></a>

### `ProsDeviceProvider::readFaults`

```cpp
[[nodiscard]] const IReadFaults* readFaults(std::uint8_t port) const noexcept override
```

MECH-A1: the read-fault count of the distance or optical sensor THIS provider made on `port` — that adapter itself, as an hal::IReadFaults; nullptr when neither was made there. Looks up, never makes (header).

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:516`](../../include/shulib/hal/pros/device_provider.hpp#L516).*

<a id="prosdeviceprovider-linereadfaults"></a>

### `ProsDeviceProvider::lineReadFaults`

```cpp
[[nodiscard]] const IReadFaults* lineReadFaults(std::uint8_t line) const noexcept override
```

MECH-A1: the read-fault count of the digital input THIS provider made on 3-wire `line`; nullptr when none was made there. Looks up, never makes.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:533`](../../include/shulib/hal/pros/device_provider.hpp#L533).*

<a id="prosdeviceprovider-clock"></a>

### `ProsDeviceProvider::clock`

```cpp
[[nodiscard]] IClock& clock() override
```

The V5's real clock (`ProsClock`, microseconds → seconds once). Never refused.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:545`](../../include/shulib/hal/pros/device_provider.hpp#L545).*

<a id="prosdeviceprovider-battery"></a>

### `ProsDeviceProvider::battery`

```cpp
[[nodiscard]] IBattery& battery() override
```

The V5 battery (`ProsBattery`). Never refused.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:547`](../../include/shulib/hal/pros/device_provider.hpp#L547).*

<a id="prosdeviceprovider-telemetry"></a>

### `ProsDeviceProvider::telemetry`

```cpp
[[nodiscard]] ITelemetrySink& telemetry() override
```

The diagnostics sink: a `diag::TermSink` over the USB serial (`ProsCharSink`, stdout) — exactly the sink src/main.cpp's hand-wired graphs built (the invented X-drive's until G1 Part 2c, which builds it over this provider). With a log copy (the log-copy constructor), a `diag::LogCopySink` over that TermSink and the card's: every line and summary to both, every per-tick record to serial only. Never refused.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:553`](../../include/shulib/hal/pros/device_provider.hpp#L553).*

<a id="prosdeviceprovider-serialcopyopen"></a>

### `ProsDeviceProvider::serialCopyOpen`

```cpp
[[nodiscard]] bool serialCopyOpen() const noexcept
```

True when this provider keeps a log copy AND its file on the card is open. False with no copy asked for, and with no card or a failed open (the copy then refuses every line).

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:562`](../../include/shulib/hal/pros/device_provider.hpp#L562).*

<a id="prosdeviceprovider-serialcopypath"></a>

### `ProsDeviceProvider::serialCopyPath`

```cpp
[[nodiscard]] const char* serialCopyPath() const noexcept
```

The copy's full path on the card ("/usd/drive_log.txt"), or "" with no copy asked for — for the program's one boot line.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:568`](../../include/shulib/hal/pros/device_provider.hpp#L568).*

<a id="prosdeviceprovider-serialcopydroppedlines"></a>

### `ProsDeviceProvider::serialCopyDroppedLines`

```cpp
[[nodiscard]] std::uint32_t serialCopyDroppedLines() const noexcept
```

Lines the copy LOST since construction: those its card refused (the write or the flush — every line, with no card), plus those left off because another task was mid-copy (diag::LogCopySink's one-writer guard; every line after a task PROS deleted mid-copy, too). 0 with no copy asked for. Read it where no other task is logging (the boot line does): the card's half is a plain count its writer keeps.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:577`](../../include/shulib/hal/pros/device_provider.hpp#L577).*

<a id="prosdeviceprovider-serialcopyappendopen"></a>

### `ProsDeviceProvider::serialCopyAppendOpen`

```cpp
[[nodiscard]] AppendOpen serialCopyAppendOpen() const noexcept
```

How the copy's Append open went (Appended / Created / Refused); Refused in Truncate mode, which never asks, and with no copy asked for.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:585`](../../include/shulib/hal/pros/device_provider.hpp#L585).*

<a id="prosdeviceprovider-pacer"></a>

### `ProsDeviceProvider::pacer`

```cpp
[[nodiscard]] motion::ITickPacer& pacer() override
```

The 10 ms `ProsTickPacer` (`pros::Task::delay_until`, anchored on its first pace). Never refused.

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:590`](../../include/shulib/hal/pros/device_provider.hpp#L590).*

<a id="prosdeviceprovider-lastrefusal"></a>

### `ProsDeviceProvider::lastRefusal`

```cpp
[[nodiscard]] const char* lastRefusal() const noexcept override
```

Why the most recent refused request returned nullptr, naming the port and — when the registry answered — what was found and what was expected. Plain ASCII. The empty string until something is refused; not cleared by a later success (the builder reads it immediately after the nullptr it explains).

*function, declared at [`include/shulib/hal/pros/device_provider.hpp:596`](../../include/shulib/hal/pros/device_provider.hpp#L596).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 143 lines, click to expand</summary>

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

 ── THE MECHANISM SENSORS (chunk MECH-A1, 2026-10-08) ───────────────────────────────
 distance(port) and optical(port) are a mechanism's PIECE sensors: made like rotation() — on the
 first request the registry allows (kind DISTANCE / OPTICAL), the same object ever after, a
 refusal naming the port and what was found ("port 9: a rotation sensor (expected a distance
 sensor)"). digitalIn(line) is a limit switch on a 3-wire port: like digitalOut(), the registry is
 NOT asked (it cannot see a switch, HA-158), only a line outside 1..8 is refused, and making one
 drives nothing. ONE LINE, ONE DIRECTION: a line made as an output and then asked for as an input,
 or the reverse, is a precondition — never a second adapter on one wire. readFaults(port) and
 lineReadFaults(line) answer the adapter THIS provider made there (ProsDistance, ProsOptical,
 ProsDigitalIn are each an hal::IReadFaults), or nullptr where it made none: they look up, never
 make, so asking for a count can never construct a device the build did not ask for.

 ── IMU CALIBRATION STARTS HERE, EXACTLY ONCE ───────────────────────────────────────
 imu() starts the sensor's calibration (ProsImu::calibrate(), non-blocking — HA-108) when it
 constructs the adapter, and never again: a second reset() would re-zero the sensor under a
 live boot heading (HA-05), which ProsImu refuses by precondition. A program built on this
 provider therefore never calls calibrate() itself. The boot heading is 0 — the robot boots
 believing it faces +X — which is what src/main.cpp's hand-wired graphs did (HA-05's one
 owner is the start pose; a provider option for it arrives when a program needs one). The
 competition program does not: since COMP-a the chosen auton's start pose is seated when the
 auton starts, through Localizer::reseatHeading (HA-05, amended), over this boot heading of 0.

 ── WHAT IT DOES NOT DO ─────────────────────────────────────────────────────────────
 It applies NO dead-port policy. A port the boot check reported MISSING is never requested
 (robot::BuildOptions::absentDrivePorts); anything else it will not make refuses the build,
 through the builder, naming the port. It never hands out a fake — a test double in a
 competition binary erases the difference between "no device" and "a test injects here" (R3b
 §6). It allocates nothing: every device lives in fixed per-port std::optional storage. The
 controller and the controller's screen are not on the seam; a program keeps its own.

 ── AN OPTIONAL COPY OF THE LOG ON THE SD CARD (chunk T4 Part 3, 2026-10-01) ──────────
 The team lead's N5 (a): every library program may keep a copy of its log on the card, so a
 practice run with no laptop still leaves the dead ports, every cut and the peak currents
 behind (the old "shulib Drive" kept one in /usd/drive_log.txt). His A (a): EVENTS AND
 SUMMARIES ONLY — the log() lines and summarize()'s block, never the per-tick records serial
 carries (diag/log_copy_sink.hpp says why: a record line every 10 ms would be a card write and a
 flush every 10 ms). Given a BARE file name, this provider owns the whole copy: a ProsBlockSink
 on the card in the mode asked (Truncate, one file per boot — "shulib Drive"'s; or Append, the
 file kept across boots — his B (a), for the competition program), a hal::BlockCharSink over it
 (one block write and one flush per line; a refusal dropped and COUNTED, never thrown), a second
 diag::TermSink over the same clock, and a diag::LogCopySink over the serial TermSink and that
 one — which is what telemetry() then returns. Without a file name it is today's provider, byte
 for byte: telemetry() is the serial TermSink. A missing or full card leaves serial unchanged;
 serialCopyOpen() and serialCopyDroppedLines() say so, for the program's one boot line (the T5
 visibility rule ProsBlockSink states). The copy begins when the provider is constructed: a
 banner a program prints before that is on serial only. The copy has ONE writer at a time
 (diag::LogCopySink's guard, the T4-3 review): a line that finds another task mid-copy stays on
 serial, is left off the card, and is counted in serialCopyDroppedLines(), and the card's next
 line says how many it missed there — no task ever waits on another's card write. A task PROS
 deletes while it is mid-copy (the register's HA-161 (a), reasoned) never releases the guard:
 the card's copy then ENDS for the rest of the boot, every later line counted, serial keeping all
 of them (diag/log_copy_sink.hpp, "A writer that never comes back": the chosen failure).

 ── NO HARDWARE CLAIM ───────────────────────────────────────────────────────────────
 Host-tested through test/pros_shim/ only, which tests this class against our BELIEF about
 PROS and cannot test the belief (shim_control.hpp says so in full). The programs that construct
 one — "shulib Drive" (src/programs/drive_program.cpp, since T4 Part 3; "shulib Teleop" did from
 G1 Part 2b until T4 Part 3 retired it), "shulib Auton Tester"
 (src/programs/auton_tester_program.cpp), "shulib Competition"
 (src/programs/competition_program.cpp, since COMP-b) and the invented X-drive
 (src/programs/xdrive_program.cpp, which refuses at boot on every robot) — have not run this
 version on hardware; no provider has yet made a device on a brain.

 No pros header is included here directly — only the hal/pros adapters, whose own diagnostic
 fences cover the vendored code (test/pros_adapter_fence_test.cpp requires a file with no
 direct pros include to carry no fence at all).
```

</details>
