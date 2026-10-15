#pragma once
//
// IDeviceProvider — devices BY PORT, and the one target-specific seam a builder needs
// (chunk G1 Part 1, 2026-09-18).
//
// ── Why this seam exists ────────────────────────────────────────────────────────────
// robot/robot.hpp turns a config::RobotConfig into the whole PROS-free object graph. To do
// that it needs actual devices, and a device is the ONE thing a config cannot contain: a
// port number is data, an `hal::IMotor` is a live object that only the target knows how to
// make. Ruling B2: the provider makes them and OWNS them; the builder BORROWS.
//
// That ownership split is not tidiness. Every F4 seam in this tree is a REFERENCE — the
// RobotContext, the Localizer, the MotorGroup all borrow — so ownership has to sit
// somewhere that outlives the graph. The provider is that somewhere on every target, and it
// is the only file that knows what a port is:
//
//     host tests   hal::fake::FakeDeviceProvider   — fakes from fixed storage, plant-bindable
//     the robot    hal::pros::ProsDeviceProvider   — one std::optional per port (G1 Part 2a;
//                                                    host-tested through the PROS shim, not
//                                                    yet used by any program in src/)
//     the sim      whatever hal/sim ends up being  — H2's
//
// Because the seam is a reference and the PROVIDER holds the storage, the same
// `robot::Robot` code serves all three with no `#ifdef` and no `pros/` include anywhere
// above hal/pros — which is the F4 principle applied to CONSTRUCTION rather than to reads.
//
// ── A device that cannot be made returns NULLPTR, and says which port ───────────────
// "Port 13 is empty" is the most common thing that will ever go wrong with a robot, and it
// deserves a sentence, not a stack unwind. So a provider that cannot make a device returns
// nullptr and records WHY in lastRefusal(), naming the port; the builder turns that into a
// refusal before any object exists — never a fake in a competition binary, never an unplanned
// null in the graph (landmine 5: absent is EXPLICIT, and it is spelled `hal::AbsentGps`, not
// `nullptr`).
//
// On the V5, HOW a dead port is found is not uniform, and this paragraph used to get it wrong
// (it said every adapter's constructor throws on a dead port — corrected at G1 Part 2a, after the
// 2026-09-24 audit measured otherwise). Only ProsMotor fails loudly, through its HA-98 read-back;
// ProsImu's and ProsRotation's constructors cannot throw at all, and ProsGps DEFERS its check on a
// device it cannot read. So hal::pros::ProsDeviceProvider asks the brain's port REGISTRY what is
// plugged in before it constructs anything (A4 register HA-141), refuses an empty or wrong-kind
// port on that answer, and ALSO catches an adapter's throw per device (a motor that answers the
// registry but rejects its configuration). Both checks name the port.
//
// A 3-WIRE (ADI) LINE IS THE EXCEPTION, and says so (T3's addendum B2, 2026-09-26). The brain's
// eight 3-wire ports, A..H, are not smart ports: a pneumatic solenoid on one has no digital
// identity, the registry cannot see it, and an unplugged line reads exactly like a working one
// (A4 register HA-158; hal/digital_out.hpp's no-validity note). So digitalOut() can refuse only a
// line that is not one of the eight — on the brain nothing can tell it a solenoid is missing. And
// MAKING a line DRIVES it (HA-119), which is why the line's level is an argument of the request.
//
// ── THE MECHANISM SENSORS, and their READ-FAULT counts (chunk MECH-A1, 2026-10-08) ──────
// The mechanism core builds every ROLE a mechanism declares (robot/mechanism_parts.hpp): its
// motors or lines as before, and now its sensors — a rotation sensor as its angle sensor (the
// existing rotation()), a distance or an optical sensor as its PIECE sensor (distance(), optical()),
// and a limit switch on a 3-wire line (digitalIn()). A smart sensor is refused like any port device
// (on the brain: the registry's kind answer, HA-141); a 3-wire INPUT, like an output, is refused only
// outside 1..8 (HA-158) — and one line is ONE direction: asking for a line as an input after it was
// made an output, or the reverse, is a precondition (two parts of a program disagreeing about one
// wire). Beside each piece sensor and switch the provider answers its READ-FAULT count
// (readFaults(port), lineReadFaults(line) — hal/read_faults.hpp says why: a held value is otherwise
// indistinguishable from a measured one). Those two are PURE like the rest: a provider that forgot
// them would silently re-open the false "piece gone" the count exists to stop.
//
// ── IDEMPOTENCE IS PART OF THE CONTRACT ─────────────────────────────────────────────
// Asking twice for the same port must return the SAME object. The graph borrows these
// pointers and holds them for the life of the robot; a provider that made a second adapter
// on the second call would leave half the graph pointing at an object nobody ticks. Every
// implementation therefore looks up first and constructs only on a miss.
//
// ── Why ITickPacer is only FORWARD-DECLARED here ────────────────────────────────────
// The pacer belongs on this seam: "advance the world to the next control tick" is exactly as
// target-specific as "give me the motor on port 3" (host: step the A2 plant; robot:
// delay_until the 10 ms boundary), and the provider is the one object that knows the target.
// But motion/motion_scheduler.hpp, where ITickPacer lives, sits four layers ABOVE hal/, and
// making an L0 header include it would invert the layering for one reference. A reference to
// an incomplete type is all a declaration needs, so the include stays where it belongs — in
// the consumer, which already has the whole motion stack.
//
// PROS-free, sim-free, allocation-free.

#include <cstdint>

#include "shulib/hal/battery.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/digital_in.hpp"
#include "shulib/hal/digital_out.hpp"
#include "shulib/hal/distance.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/gps.hpp"
#include "shulib/hal/imu.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/optical.hpp"
#include "shulib/hal/read_faults.hpp"
#include "shulib/hal/rotation.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::motion {
/// Forward declaration only — see the header note on why this seam names the pacer without
/// including the motion layer that defines it.
class ITickPacer;
}  // namespace shulib::motion

namespace shulib::hal {

/// The brain's own 3-wire (ADI) ports, lettered A..H on the brain and numbered 1..8 on this seam
/// (A = 1). An expander's lines are not on the seam.
inline constexpr std::uint8_t kBrainAdiLines = 8;

/// Devices by port, made or looked up on demand and owned by the provider (ruling B2).
///
/// The per-port accessors are NOT const: obtaining a device may construct one, and the provider
/// is the object that then owns it. They are also not `noexcept` by declaration — an
/// implementation is free to be, and both shipped ones are — because refusing to construct is
/// spelled as a nullptr return rather than as an exception, and pinning `noexcept` on the
/// interface would forbid an implementation that legitimately allocates its storage lazily.
class IDeviceProvider {
public:
    /// Polymorphic-base boilerplate: the destructor is virtual so a provider held as
    /// `IDeviceProvider&`/`*` destroys correctly — and destroying it destroys every device it
    /// made, which is why it must outlive every graph built from it. Declaring the destructor
    /// suppresses the implicit copy/move, which are re-defaulted below; a CONCRETE provider is
    /// normally non-copyable for the same reason a graph is (its devices are pointed at).
    virtual ~IDeviceProvider() = default;
    /// Default-constructible: the base has nothing to initialize.
    IDeviceProvider() = default;
    /// Copyable at the base, because the base holds no state; concrete providers decide.
    IDeviceProvider(const IDeviceProvider&) = default;
    /// Movable, same reasoning as the copy constructor.
    IDeviceProvider(IDeviceProvider&&) = default;
    /// Copy-assignable, same reasoning.
    IDeviceProvider& operator=(const IDeviceProvider&) = default;
    /// Move-assignable, same reasoning.
    IDeviceProvider& operator=(IDeviceProvider&&) = default;

    /// The motor on `signedPort` — |p| in 1..kMaxSmartPorts, SIGN = the PROS reversal, handed
    /// over AS STORED so the sign is applied exactly once, in the adapter (the one-place-for-a-
    /// sign rule that hal/motor_group.hpp and hal/drivetrain_description.hpp both state).
    /// `cartridge` is the human-stated gearset. Returns nullptr and sets lastRefusal() when the
    /// port is out of range, empty, or carries something that is not a motor. Idempotent: the
    /// same port yields the same object.
    [[nodiscard]] virtual IMotor* motor(std::int8_t signedPort, MotorCartridge cartridge) = 0;

    /// The inertial sensor on physical `port`, 1..kMaxSmartPorts. Returns nullptr + a refusal on
    /// a port that cannot serve one. Idempotent.
    [[nodiscard]] virtual IImu* imu(std::uint8_t port) = 0;

    /// The GPS on physical `port`, with its lever arm from the robot centre (`leverArmForward` is
    /// the +X body coordinate, `leverArmLeft` the +Y). The lever arm is a CONSTRUCTION argument
    /// rather than a later setter because the adapter removes it at the HAL edge, once, and a
    /// GPS that has already been read with the wrong arm has already lied. Returns nullptr + a
    /// refusal on a port that cannot serve one. Idempotent.
    [[nodiscard]] virtual IGps* gps(std::uint8_t port, units::Length leverArmForward,
                                    units::Length leverArmLeft) = 0;

    /// The rotation sensor on physical `port`, 1..kMaxSmartPorts — a tracking wheel's encoder.
    /// Returns nullptr + a refusal on a port that cannot serve one. Idempotent.
    [[nodiscard]] virtual IRotation* rotation(std::uint8_t port) = 0;

    /// The digital output on the brain's 3-wire port `line`, 1..kBrainAdiLines (= A..H; a letter
    /// byte such as 'A' is NOT translated — state the number) — a pneumatic solenoid's line.
    /// MAKING ONE DRIVES IT: the line is set to `initialState` the moment it is made (on a brain,
    /// inside `pros::adi::DigitalOut`'s constructor — A4 register HA-119), so the level is STATED
    /// here, never defaulted: state what the mechanism must be at program start. Returns nullptr
    /// and sets lastRefusal() for a line outside 1..8. Idempotent: the same line yields the same
    /// object, driven only when it was MADE — a later request drives nothing. PRECONDITION: a later
    /// request for a made line states the SAME `initialState` (one line, one configuration — two
    /// parts of a program disagreeing about a line's boot level is not a wiring fault). Nothing
    /// can refuse an EMPTY line: the brain's registry cannot see a solenoid (header; HA-158).
    /// PRECONDITION (since MECH-A1): the line was not made as a digitalIn() — one line, one
    /// direction. Added in T3's addendum B2 (2026-09-26); every implementer must answer it.
    [[nodiscard]] virtual IDigitalOut* digitalOut(std::uint8_t line, bool initialState) = 0;

    /// The DISTANCE sensor on physical `port`, 1..kMaxSmartPorts — a mechanism's piece sensor
    /// (MECH-A1). Returns nullptr and sets lastRefusal() when the port is out of range, empty, or
    /// carries something else. Idempotent: the same port yields the same object. Every implementer
    /// must answer it.
    [[nodiscard]] virtual IDistance* distance(std::uint8_t port) = 0;

    /// The OPTICAL sensor on physical `port`, 1..kMaxSmartPorts — a mechanism's piece sensor
    /// (MECH-A1). nullptr + a refusal on a port that cannot serve one. Idempotent. Every
    /// implementer must answer it.
    [[nodiscard]] virtual IOptical* optical(std::uint8_t port) = 0;

    /// The digital INPUT on the brain's 3-wire port `line`, 1..kBrainAdiLines (A = 1) — a limit
    /// switch (MECH-A1). Reading one drives nothing, and making one drives nothing. nullptr + a
    /// refusal ONLY for a line outside 1..8 on the brain: its registry cannot see a 3-wire device
    /// (header; HA-158), so an in-range line is always made. PRECONDITION: the line was not made as
    /// a digitalOut() (one line, one direction — and digitalOut() states the converse). Idempotent.
    [[nodiscard]] virtual IDigitalIn* digitalIn(std::uint8_t line) = 0;

    /// The READ-FAULT count of the piece sensor this provider made on smart `port` with distance()
    /// or optical() — how a consumer tells a HELD value from a measured one (hal/read_faults.hpp).
    /// nullptr when no such sensor was made there, or when this provider's sensors never screen a
    /// read. Never makes anything. Every implementer must answer it (header).
    [[nodiscard]] virtual const IReadFaults* readFaults(std::uint8_t port) const noexcept = 0;

    /// The READ-FAULT count of the digital input this provider made on 3-wire `line` with
    /// digitalIn(); nullptr when none was made there (or it never screens a read). Never makes
    /// anything. Every implementer must answer it.
    [[nodiscard]] virtual const IReadFaults* lineReadFaults(std::uint8_t line) const noexcept = 0;

    /// The run's clock. PORT-LESS and never refused: every target has one, and a robot without a
    /// clock cannot report that it has no clock.
    [[nodiscard]] virtual IClock& clock() = 0;

    /// The battery. Port-less and never refused, same reasoning as the clock.
    [[nodiscard]] virtual IBattery& battery() = 0;

    /// The one diagnostics sink for this robot. Port-less and never refused: a NullSink is a
    /// valid answer and is what "telemetry is off" means, so there is nothing to refuse.
    [[nodiscard]] virtual ITelemetrySink& telemetry() = 0;

    /// The tick pacer — how the world advances during a blocking verb. Port-less and never
    /// refused. See the header note on the forward declaration.
    [[nodiscard]] virtual motion::ITickPacer& pacer() = 0;

    /// Why the most recent device request returned nullptr, naming the PORT. The empty string
    /// when nothing has been refused. The returned pointer is owned by the provider and stays
    /// valid until the next refusal, which is enough for a builder that reads it immediately and
    /// copies it into a verdict.
    [[nodiscard]] virtual const char* lastRefusal() const noexcept = 0;
};

}  // namespace shulib::hal
