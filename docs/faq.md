# FAQ — how it actually behaves

> **What this is.** The nuance the reference cannot carry: not "what is the signature" (the
> [API reference](api/README.md)) and not "what changed" (the [changelog](changelog.md)), but
> *what does this actually do* in the situations you meet at 11pm before a competition. Each
> answer links the deeper document; where behaviour rests on an unverified belief about
> hardware, the answer names its entry in the
> [Hardware Assumptions Register](hardware-assumptions.md).

## What happens when a sensor returns a PROS error mid-run?

Short version: **the run continues, the value freezes, and the failure is visible — never a
crash, never a NaN, never a fake zero.**

PROS reports a failed read with a sentinel (`PROS_ERR` / `PROS_ERR_F`, the latter literally
infinity). The `hal/pros` adapters screen every read **before** any conversion:

- **Sensors with a validity signal** (GPS `hasFix()`, IMU `isReady()`) report invalid, and the
  estimator dead-reckons past the gap. That path is exercised millions of times in the host
  suite.
- **Sensors without one** (motors, rotation sensors, the battery) hold their **last good
  value** and count the screened reads (each adapter's `faultedReads()`). Deliberately NOT
  zero: a zeroed encoder reads as "the robot stopped", which is exactly the lie that makes a
  dead-encoder runaway invisible — a *frozen* value is what the health layer's
  wheels-spin-but-no-motion cross-check is designed to catch, raising `ODO_STUCK` and (by
  default) aborting the active motion into a safe brake.
- **The conversions themselves throw on a sentinel** — a fail-loud backstop, not the normal
  path; the adapters exist to make it unreachable (register entry HA-08).

## Why does `hasFix()` go false in Driving Skills?

Because Driving Skills **has no GPS strip on the field walls**, and the VEX GPS is a camera
that watches the strip. No strip, no fix — all match long. This is a normal operating state,
not a fault (the health layer deliberately raises nothing for it): the estimator dead-reckons
on odometry + IMU, exactly as designed, and the localizer's quality state says so honestly.
`hasFix()` also goes false while the sensor boots, when it is unplugged, and when its view is
blocked — the rule is simply *false = ignore the GPS this tick*, and every consumer already
obeys it. What `hasFix()` does **not** judge is fix *quality* — "is this fix worth folding"
belongs to the fusion layer's gates (HA-61…67), not to the HAL.

## Why does the library bind `get_rotation()` and not `get_heading()`?

Both report the V5 IMU's yaw, but `get_heading()` wraps to [0, 360) while `get_rotation()`
counts **cumulatively** — 450° stays 450°. Two reasons the cumulative form is the contract
(HA-03):

1. **The yaw *rate* is differentiated from it.** Across the 360° wrap seam, differentiating
   the wrapped form produces a phantom near-full-circle step in one tick — a spike of hundreds
   of rad/s that would slam the fused heading. The cumulative form differentiates clean. (This
   is not hypothetical: it is a mutation the test suite was proven to catch.)
2. **Its sign convention is documented** ("clockwise rotations are positive"), so the one
   CW→CCW negation in the conversion is provably right, where the raw gyro's sign is
   undocumented (HA-04).

For the *heading itself* the two are equivalent after wrapping — the reason the ban is written
as a contract (and pinned by a guard test) rather than left to observation.

## My motor's position reads 1/360 of what it should. What happened?

Almost certainly: the motor was **not configured by shulib's adapter** — a raw `pros::Motor`
constructed with default arguments *leaves the device as it was*, and V5 encoder units live in
the motor, surviving from whatever program ran last. A motor left in `rotations` reports 1/360
of degrees. shulib's `ProsMotor` exists to make this impossible: its constructor demands the
cartridge color, sets degrees + gearset explicitly, **reads both back**, and refuses to
construct (loudly, at boot) if the device disagrees (HA-95/HA-98). If you see the 1/360
signature, some code path is constructing a raw `pros::Motor` beside the adapter.

## Why does my distance sensor read 393 inches?

Because there is **nothing in front of it** — and PROS reports that as a plain number, not an
error. `get_distance()` returns **9999** (millimeters) when it cannot detect an object; the
vendored header says so in one sentence, and 9999 mm converts to 393.66 inches — a value that
looks exactly like a far wall (HA-114). It is not `PROS_ERR`, it does not set a special state,
and nothing about it looks wrong downstream.

shulib's `ProsDistance` handles it for you: when the raw reading is 9999, `confidence()`
reports **0.0** — the seam's documented "no usable return" channel — while `distance()` stays
finite. So the rule for every consumer is: **threshold `confidence()` before trusting
`distance()`**, which is what the seam's header has said since the interface was written. A
dock-confirm that checks distance alone is reading the 393-inch phantom; a dock-confirm that
checks confidence first sees "no object", which is the truth.

One more nuance in the same sensor: PROS documents confidence as *"only available when
distance is > 200mm"*. At or below 200 mm the adapter reports confidence **1.0** — an object
close enough to return a real distance IS detected; the raw confidence channel simply does not
exist down there, and shulib refuses to report a number PROS says is not available (HA-115).

## Why did my pneumatic fire the moment the robot booted?

Because **constructing the PROS digital-out object actuates the port**. `pros::adi::DigitalOut`
takes an initial state in its constructor, drives the line immediately, and **defaults the
state to LOW** — so a bare construction physically moves any cylinder whose "safe" position is
the high state, at boot, before any of your code runs (HA-119).

shulib's `ProsDigitalOut` makes the default impossible: the initial state is a **required**
constructor argument. State it, and make it the same value as the owning
`PneumaticMechanism`'s declared safe state — then boot drives the line straight to safe, once,
with no glitch through the wrong state. If your robot moves at power-on, look for a digital
line whose constructor argument disagrees with its mechanism's safe state (or for a raw
`pros::adi::DigitalOut` constructed beside the adapter, riding the LOW default).

## The code said the SD-card writes succeeded — where did my log go?

Two different traps share this symptom.

**The path prefix.** PROS's SD API has two conventions in one surface: `usd_list_files()`
documents *"DO NOT PREPEND YOUR PATHS WITH /usd/"* — while `fopen` reaches the card **only**
through paths that start with `/usd/` (HA-122). Get it backwards and you either list nothing
or write nowhere. shulib's `ProsBlockSink` owns the prefix in exactly one place: you hand it a
bare file name (a leading `/` is rejected loudly at construction), and it builds the real
path itself.

**The missing card.** A sink with no card behind it must not stop the robot — so
`ProsBlockSink` constructs successfully either way and simply **refuses**: `write()` returns
false from the very first call, and `isOpen()` says why. The composition root reports that
once at boot. If your log "succeeded" with no file to show for it, check whether anything read
those return values — the seam makes them `[[nodiscard]]` for exactly this reason. And note
one honest limit even with a card: small writes sit in a buffer and can report accepted while
the card is already refusing — the failure then lands on `flush()`, which is why its result is
a `bool` too.

## Why does `ProsGps` refuse to construct on my robot?

Because your GPS has a **firmware offset configured** (`get_offset() != (0,0)`), probably by a
previous program calling `set_offset()`. shulib removes the sensor→center lever arm itself, in
one place, from your robot config — if the firmware *also* compensates, the arm gets
subtracted twice and every fix carries inches of heading-dependent bias, silently (HA-06).
The adapter checks at boot and fails loudly instead. Fix: clear the device offset (or
re-flash its settings), never work around the check.

## Where is `get_digital_new_press()`? I want "just pressed".

Deliberately not bound, ever. PROS's new-press read **consumes** the press: it is one shared
edge detector per controller object, so when two pieces of code watch the same button, one of
them silently misses every press (HA-104). shulib's seam reports button **levels**, and each
consumer owns its own tiny `ButtonEdge` — every consumer sees every press. If you are porting
driver code that used `get_digital_new_press()`, replace each call site with a `ButtonEdge`
member and feed it `pressed(...)` once per loop.

## Why doesn't shulib use PROS's `registry_get_plugged_type()` to find devices?

Because it has a different port convention from the rest of the SDK, and mixing them fails
*silently*.

`registry_get_plugged_type()` is **zero-indexed** — its own documentation says "the device plugged
into the zero-indexed port", range 0–20. Every device API — `motor_get_position()`,
`imu_get_rotation()`, `gps_get_status()` — is **one-indexed**, range 1–21. Both conventions live in
the same SDK, stated only in each function's own comment.

Get it wrong and nothing errors. You get a device list that is off by one, and then reads through
the device APIs land on the wrong hardware. A bench probe written this way once reported "a dead
motor and a dead IMU" on a robot where both were perfectly healthy: it was asking the motor API
about the port where the IMU actually lived, and `ENODEV` looks exactly like a dead motor.

No shulib adapter calls the registry. Each adapter takes the port you give it and passes it
straight to the one-indexed device API, so there is only ever one convention in play. If you write
your own port-scanning diagnostic, that is where the trap is waiting — add one, not zero.

## I'm porting shulib to non-PROS hardware. What does "the shim tests the adapter, not the belief" mean for me?

It is the honest limit of host-testing a hardware binding, and it transfers to your port
directly. shulib's adapters are tested against a hand-written, programmable stand-in for the
PROS SDK. That proves the adapter faithfully implements *our beliefs* about the SDK — units,
sign conventions, error sentinels — but if a belief is wrong, the stand-in and the adapter are
wrong *together*, and every test still passes. The beliefs therefore live in the
[Hardware Assumptions Register](hardware-assumptions.md) (HA-94 onward for the PROS set), each
labelled by confidence with the bench measurement that settles it.

For your port: the conversions (`*_conversion.hpp`) are pure and SDK-free — reuse them. Write
your adapters as thin glue over your SDK behind the same HAL interfaces, write down every
belief about *your* SDK's units and conventions as falsifiable claims, and plan the bench
session that checks each one before your robot trusts them. The guide's extending chapter
walks the pattern.
