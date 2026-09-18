#pragma once
//
// FakeDeviceProvider — the host's IDeviceProvider (chunk G1 Part 1, 2026-09-18).
//
// ── What it is for ──────────────────────────────────────────────────────────────────
// robot::Robot asks a provider for devices by port. On the V5 that provider constructs
// hal/pros adapters (G1 Part 2). On the host it is this: fixed storage of F4 fakes, indexed
// by physical port, handed out on demand — so the SAME builder, the same graph and the same
// construction order are exercised off-robot, at laptop speed, with no PROS anywhere.
//
// ── The plant-attachment hooks, and why BINDING is one of them ──────────────────────
// A host test does not want a graph over inert fakes; it wants a graph over the A2 plant, so
// that what the builder produced can be DRIVEN and compared against a hand-wired rig bit for
// bit. Two hooks do that, and they exist for two different reasons:
//
//   * fakeMotor(port) / fakeRotation(port) / … hand back the provider's OWN storage, so a
//     test can attach those motors to a DrivePlant wheel as coupled members
//     (DrivePlant::attachCoupledMembers) — the five-per-side gear train robot two has.
//
//   * bindMotor(port, m) / bindImu / bindGps / bindRotation make a port serve a device the
//     CALLER owns. This one is load-bearing rather than convenient, and the measurement
//     that forced it is worth writing down: DrivePlant::wheelCommand() reads the PRIMARY
//     FakeMotor's commanded voltage when every member of the wheel agrees, and the members'
//     MEAN when they do not. The plant's primary motor is owned by the SimHarness and
//     handed to the plant at construction. A group built only from provider-owned fakes
//     therefore leaves that primary at 0 V and the plant drives on a mean that includes a
//     motor nobody commanded — not a different robot, a WRONG one. Binding port 1 to
//     harness.motor(0) is how the builder's group and the plant's wheel become the same
//     wheel. The IMU, the GPS and the tracking-wheel encoders are harness-owned and
//     plant-synthesized for exactly the same reason, and bind the same way.
//
// A bound device is BORROWED: the caller owns it and it must outlive the provider. Ruling
// B2 ("the provider owns them") is about the BUILDER never owning hardware; on the robot
// ProsDeviceProvider will own every device it makes, and here the SimHarness — which
// outlives the Robot in every test — owns the bound ones.
//
// ── refusePort(), so refusal is testable ────────────────────────────────────────────
// The most common thing that will ever go wrong with a robot is an empty or dead port, and
// the builder's answer to it (a verdict naming the port, before a single object exists) is
// the part of G1 that most needs a test. refusePort(13) makes this provider answer nullptr
// for anything on port 13, with a refusal string that names it — the host stand-in for a
// ProsMotor constructor throwing on a port with nothing in it.
//
// ── SIM-FREE, on purpose ────────────────────────────────────────────────────────────
// Nothing in this file includes shulib/sim/, and the CI layering guard enforces that. The
// PLANT attachment is done BY THE TEST, which may include sim/ and does; this header only
// has to expose the storage and the binding to make that possible. A fake provider that
// knew what a DrivePlant was would be a sim dependency inside hal/, and the guard that
// keeps estimators from reading ground truth is the same guard.
//
// Host-test infrastructure. PROS-free, sim-free, allocation-free.

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "shulib/core/check.hpp"
#include "shulib/hal/device_provider.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/fake/fake_gps.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_rotation.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal::fake {

/// The host IDeviceProvider: fakes by port, bindable to a caller's devices, with a per-port
/// refusal switch. Non-copyable and non-movable — the graph holds pointers into its storage.
class FakeDeviceProvider final : public IDeviceProvider {
public:
    /// One more than the highest physical port, so a port number indexes the arrays directly and
    /// index 0 is simply never used.
    static constexpr std::size_t kSlots = kMaxSmartPorts + 1;

    /// The four PORT-LESS handles, which this provider borrows rather than invents: on the host
    /// they are the harness's clock, the harness's battery, whatever sink the test wants to read
    /// back, and the test's own pacer (the plant-stepping one). All four must outlive the
    /// provider. They are constructor arguments rather than owned defaults on purpose — a
    /// default no-op pacer would let a blocking verb spin forever against a clock that never
    /// advances, which is a hang rather than a red test.
    FakeDeviceProvider(IClock& clock, IBattery& battery, ITelemetrySink& telemetry,
                       motion::ITickPacer& pacer) noexcept
        : clock_{&clock}, battery_{&battery}, telemetry_{&telemetry}, pacer_{&pacer} {}

    /// Non-copyable: a copy would duplicate the fake storage while the graph kept pointing at
    /// the original, so half the robot would read a device nobody drives.
    FakeDeviceProvider(const FakeDeviceProvider&) = delete;
    /// Non-movable, same reasoning.
    FakeDeviceProvider(FakeDeviceProvider&&) = delete;
    /// Non-copy-assignable, same reasoning.
    FakeDeviceProvider& operator=(const FakeDeviceProvider&) = delete;
    /// Non-move-assignable, same reasoning.
    FakeDeviceProvider& operator=(FakeDeviceProvider&&) = delete;
    /// Destroying the provider destroys every fake it owns, so it must outlive every graph built
    /// from it.
    ~FakeDeviceProvider() override = default;

    // ── the binding hooks (header: "the plant-attachment hooks") ───────────────────────

    /// Serve `port` with a motor the CALLER owns (typically a plant primary). The bound device
    /// must outlive this provider. Rebinding the same port replaces the binding.
    void bindMotor(std::uint8_t port, IMotor& motor) {
        boundMotor_[checkedIndex(port)] = &motor;
    }
    /// Serve `port` with an IMU the caller owns — the plant-synthesized one, in every rig here.
    void bindImu(std::uint8_t port, IImu& imu) { boundImu_[checkedIndex(port)] = &imu; }
    /// Serve `port` with a GPS the caller owns — the plant-synthesized one.
    void bindGps(std::uint8_t port, IGps& gps) { boundGps_[checkedIndex(port)] = &gps; }
    /// Serve `port` with a rotation sensor the caller owns — a plant tracking-wheel encoder.
    void bindRotation(std::uint8_t port, IRotation& rotation) {
        boundRotation_[checkedIndex(port)] = &rotation;
    }

    // ── the provider's OWN fakes, for plant attachment and for direct driving ──────────

    /// This provider's own FakeMotor for `port` — the object `motor()` hands out when the port is
    /// not bound. A test attaches these to a DrivePlant wheel as coupled members.
    [[nodiscard]] FakeMotor& fakeMotor(std::uint8_t port) { return motors_[checkedIndex(port)]; }
    /// This provider's own FakeImu for `port`.
    [[nodiscard]] FakeImu& fakeImu(std::uint8_t port) { return imus_[checkedIndex(port)]; }
    /// This provider's own FakeGps for `port`.
    [[nodiscard]] FakeGps& fakeGps(std::uint8_t port) { return gpss_[checkedIndex(port)]; }
    /// This provider's own FakeRotation for `port`.
    [[nodiscard]] FakeRotation& fakeRotation(std::uint8_t port) {
        return rotations_[checkedIndex(port)];
    }

    // ── the refusal switch ─────────────────────────────────────────────────────────────

    /// Make every device request for `port` answer nullptr, with a refusal naming the port — the
    /// host stand-in for an adapter constructor throwing on an empty or dead port.
    void refusePort(std::uint8_t port) { refused_[checkedIndex(port)] = true; }
    /// Undo refusePort(); a port is allowed by default, so this is only ever needed to re-arm one.
    void allowPort(std::uint8_t port) { refused_[checkedIndex(port)] = false; }
    /// Whether `port` is currently refused.
    [[nodiscard]] bool isRefused(std::uint8_t port) const { return refused_[checkedIndex(port)]; }

    // ── what was ASKED FOR, so a test can prove the builder passed the config through ──

    /// Whether `motor()` has been called for `port` — the evidence that the builder asked for
    /// exactly the ports the config names, and no others.
    [[nodiscard]] bool motorRequested(std::uint8_t port) const {
        return motorRequested_[checkedIndex(port)];
    }
    /// The cartridge the last `motor()` call for `port` passed. The byte 0 if never asked. This
    /// is how a test proves the human-stated, unverifiable cartridge reached the adapter rather
    /// than being dropped somewhere in the builder.
    [[nodiscard]] MotorCartridge motorCartridge(std::uint8_t port) const {
        return cartridges_[checkedIndex(port)];
    }
    /// The SIGNED port the last `motor()` call for `port` passed — `-11`, not `11`. 0 if never
    /// asked. This exists because a host fake CANNOT show a sign error by behaving differently:
    /// PROS applies the reversal inside the real adapter, and a fake indexed by |port| is
    /// identical either way. The sign is the single most dangerous number in a drivetrain
    /// description (a drive side wired backwards drives the robot backwards, silently), so it is
    /// checked structurally here rather than hoped for.
    [[nodiscard]] std::int8_t motorSignedPort(std::uint8_t port) const {
        return signedPorts_[checkedIndex(port)];
    }
    /// The +FORWARD lever arm the last `gps()` call for `port` passed (0 if never asked).
    [[nodiscard]] units::Length gpsLeverForward(std::uint8_t port) const {
        return leverForward_[checkedIndex(port)];
    }
    /// The +LEFT lever arm the last `gps()` call for `port` passed (0 if never asked).
    [[nodiscard]] units::Length gpsLeverLeft(std::uint8_t port) const {
        return leverLeft_[checkedIndex(port)];
    }

    // ── IDeviceProvider ────────────────────────────────────────────────────────────────

    /// The motor on `signedPort` — the bound one if the port is bound, else this provider's own
    /// fake. nullptr + a refusal on an out-of-range or refused port. Idempotent by construction:
    /// the storage is fixed, so the same port always yields the same address.
    [[nodiscard]] IMotor* motor(std::int8_t signedPort, MotorCartridge cartridge) override {
        const int port = DrivetrainDescription::portMagnitude(signedPort);
        if (!servable(port, "motor")) {
            return nullptr;
        }
        const auto i = static_cast<std::size_t>(port);
        motorRequested_[i] = true;
        cartridges_[i] = cartridge;
        signedPorts_[i] = signedPort;
        return boundMotor_[i] != nullptr ? boundMotor_[i] : static_cast<IMotor*>(&motors_[i]);
    }

    /// The IMU on `port` — bound or own fake; nullptr + a refusal otherwise.
    [[nodiscard]] IImu* imu(std::uint8_t port) override {
        const int p = static_cast<int>(port);
        if (!servable(p, "IMU")) {
            return nullptr;
        }
        const auto i = static_cast<std::size_t>(p);
        return boundImu_[i] != nullptr ? boundImu_[i] : static_cast<IImu*>(&imus_[i]);
    }

    /// The GPS on `port`, recording the lever arm it was asked for — bound or own fake;
    /// nullptr + a refusal otherwise.
    [[nodiscard]] IGps* gps(std::uint8_t port, units::Length leverArmForward,
                            units::Length leverArmLeft) override {
        const int p = static_cast<int>(port);
        if (!servable(p, "GPS")) {
            return nullptr;
        }
        const auto i = static_cast<std::size_t>(p);
        leverForward_[i] = leverArmForward;
        leverLeft_[i] = leverArmLeft;
        return boundGps_[i] != nullptr ? boundGps_[i] : static_cast<IGps*>(&gpss_[i]);
    }

    /// The rotation sensor on `port` — bound or own fake; nullptr + a refusal otherwise.
    [[nodiscard]] IRotation* rotation(std::uint8_t port) override {
        const int p = static_cast<int>(port);
        if (!servable(p, "rotation sensor")) {
            return nullptr;
        }
        const auto i = static_cast<std::size_t>(p);
        return boundRotation_[i] != nullptr ? boundRotation_[i]
                                            : static_cast<IRotation*>(&rotations_[i]);
    }

    /// The borrowed clock.
    [[nodiscard]] IClock& clock() override { return *clock_; }
    /// The borrowed battery.
    [[nodiscard]] IBattery& battery() override { return *battery_; }
    /// The borrowed telemetry sink.
    [[nodiscard]] ITelemetrySink& telemetry() override { return *telemetry_; }
    /// The borrowed pacer.
    [[nodiscard]] motion::ITickPacer& pacer() override { return *pacer_; }

    /// Why the most recent request returned nullptr, naming the port; the empty string when
    /// nothing has been refused.
    [[nodiscard]] const char* lastRefusal() const noexcept override { return refusal_; }

private:
    /// Bounds-check a port and return it as an array index. A port outside 1..kMaxSmartPorts is a
    /// programming error in the TEST (the library never calls these hooks), so it is a
    /// precondition rather than a silent clamp.
    [[nodiscard]] std::size_t checkedIndex(std::uint8_t port) const {
        SHULIB_PRECONDITION(port >= 1 && port <= static_cast<std::uint8_t>(kMaxSmartPorts),
                            "FakeDeviceProvider: port must be in 1..21");
        return static_cast<std::size_t>(port);
    }

    /// Whether `port` can be served, writing the refusal when it cannot. `what` names the device
    /// kind so the message reads "port 13 has no motor" rather than "port 13 refused".
    [[nodiscard]] bool servable(int port, const char* what) {
        if (port < 1 || port > static_cast<int>(kMaxSmartPorts)) {
            std::snprintf(refusal_, sizeof refusal_, "port %d is not a port in 1..21", port);
            return false;
        }
        if (refused_[static_cast<std::size_t>(port)]) {
            std::snprintf(refusal_, sizeof refusal_, "port %d has no %s", port, what);
            return false;
        }
        return true;
    }

    IClock* clock_;
    IBattery* battery_;
    ITelemetrySink* telemetry_;
    motion::ITickPacer* pacer_;

    FakeMotor motors_[kSlots]{};
    FakeImu imus_[kSlots]{};
    FakeGps gpss_[kSlots]{};
    FakeRotation rotations_[kSlots]{};

    IMotor* boundMotor_[kSlots]{};
    IImu* boundImu_[kSlots]{};
    IGps* boundGps_[kSlots]{};
    IRotation* boundRotation_[kSlots]{};

    bool refused_[kSlots]{};
    bool motorRequested_[kSlots]{};
    MotorCartridge cartridges_[kSlots]{};
    std::int8_t signedPorts_[kSlots]{};
    units::Length leverForward_[kSlots]{};
    units::Length leverLeft_[kSlots]{};

    char refusal_[96]{};
};

}  // namespace shulib::hal::fake
