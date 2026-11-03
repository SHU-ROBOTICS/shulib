#pragma once
//
// FakeDeviceProvider — the host's IDeviceProvider (chunk G1 Part 1, 2026-09-18).
//
// ── What it is for ──────────────────────────────────────────────────────────────────
// robot::Robot asks a provider for devices by port. On the V5 that provider constructs
// hal/pros adapters (hal::pros::ProsDeviceProvider, G1 Part 2a). On the host it is this: fixed
// storage of F4 fakes, indexed by physical port, handed out on demand — so the SAME builder, the
// same graph and the same construction order are exercised off-robot, at laptop speed, with no
// PROS anywhere.
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
// ProsDeviceProvider owns every device it makes, and here the SimHarness — which outlives the
// Robot in every test — owns the bound ones.
//
// ── refusePort(), so refusal is testable ────────────────────────────────────────────
// The most common thing that will ever go wrong with a robot is an empty or dead port, and
// the builder's answer to it (a verdict naming the port, before a single object exists) is
// the part of G1 that most needs a test. refusePort(13) makes this provider answer nullptr
// for anything on port 13, with a refusal string that names it — the host stand-in for the real
// provider refusing a port (ProsDeviceProvider refuses an EMPTY or wrong-kind port on the
// registry's answer, and a motor whose read-back fails on the adapter's throw).
//
// ── motorRequestCount(), so "never asked for" is testable (G1 Part 2a) ─────────────
// A port the boot check reported dead is handed to the builder as ABSENT and must never be
// requested. motorRequested() cannot prove that — it is set only when a request is SERVED — so
// every motor() call is also COUNTED per port, refused or not.
//
// ── The 3-wire lines, MADE the way the brain makes them (T3's addendum B2) ──────────
// digitalOut(line, initialState) serves a FakeDigitalOut per brain 3-wire line (1..8), or a line
// the caller bound (bindDigitalOut). Making a line DRIVES it: the first request that serves a line
// calls set(initialState) on it, exactly once, because on the brain pros::adi::DigitalOut's
// constructor does (HA-119) — a fake that skipped it would let a host test pass a builder that
// never states a boot level. Later requests return the same object and drive nothing; a later
// request with a different initial state is a precondition, as on the robot. refuseLine() is the
// host stand-in for a provider that will not make a line — which the on-robot provider can only do
// for a line outside 1..8, because nothing on a brain can see an empty 3-wire port (HA-158).
//
// ── The mechanism sensors and their read-fault counts (chunk MECH-A1, 2026-10-08) ──
// distance(port) / optical(port) serve this provider's own FakeDistance / FakeOptical, refused on a
// refusePort()'d port like every port device; digitalIn(line) serves its own FakeDigitalIn, refused
// on a refuseLine()'d line or outside 1..8 — and one line is one direction, as on the brain: an
// input asked for on a line made as an output, or the reverse, is a precondition. readFaults(port)
// and lineReadFaults(line) answer this provider's own hal::fake::FakeReadFaults for that port or
// line — but ONLY where distance()/optical() or digitalIn() SERVED a sensor, nullptr elsewhere,
// exactly the on-robot provider's contract: a builder that read a count for a sensor it never got
// would pass here and fail on the brain, so the fake does not let it pass. fakeReadFaults(port) /
// fakeLineReadFaults(line) hand a test those counters, to set one `failing`.
//
// ── The AI Vision camera (chunk VIS1-A1, 2026-10-18) ────────────────────────────────
// aiVision(port, settings) serves this provider's own hal::fake::FakeVisionFrames for that port — refused
// on a refusePort()'d port like every port device, AND for a settings block with a problem, AND for a
// second request whose settings are not `sameAiVisionSettings` as the first's (the on-robot provider's
// three refusals; a fake that let any of them pass would let a builder pass here and fail on the brain).
// fakeCamera(port) hands a test that camera, to set its frames, its faults and its status;
// cameraSettings(port) the settings the request that made it passed. readFaults(port) answers the
// FakeVisionFrames ITSELF — not a separate FakeReadFaults, as the distance and optical sensors' counts
// are — and only once aiVision() has SERVED it: the camera's fake is an IReadFaults, as the adapter is,
// so its count moves with the fault frames it returns (VIS1's re-check T9; fake_vision_frames.hpp).
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
#include "shulib/hal/fake/fake_digital_in.hpp"
#include "shulib/hal/fake/fake_digital_out.hpp"
#include "shulib/hal/fake/fake_distance.hpp"
#include "shulib/hal/fake/fake_gps.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_optical.hpp"
#include "shulib/hal/fake/fake_read_faults.hpp"
#include "shulib/hal/fake/fake_vision_frames.hpp"
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
    /// One more than the highest brain 3-wire line (8), for the same direct indexing.
    static constexpr std::size_t kLineSlots = kBrainAdiLines + 1;

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
    /// Serve 3-wire `line` (1..8) with a digital out the caller owns — a recording line, when a
    /// test wants every level it was ever driven to. Making it still DRIVES it (the header).
    void bindDigitalOut(std::uint8_t line, IDigitalOut& out) {
        boundLine_[checkedLine(line)] = &out;
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
    /// This provider's own FakeDigitalOut for 3-wire `line` (1..8) — what `digitalOut()` hands out
    /// when the line is not bound. Its `setCount()` includes the drive that made it.
    [[nodiscard]] FakeDigitalOut& fakeDigitalOut(std::uint8_t line) {
        return lines_[checkedLine(line)];
    }
    /// This provider's own FakeDistance for `port` — what `distance()` hands out (MECH-A1).
    [[nodiscard]] FakeDistance& fakeDistance(std::uint8_t port) {
        return distances_[checkedIndex(port)];
    }
    /// This provider's own FakeOptical for `port` — what `optical()` hands out (MECH-A1).
    [[nodiscard]] FakeOptical& fakeOptical(std::uint8_t port) { return opticals_[checkedIndex(port)]; }
    /// This provider's own FakeDigitalIn for 3-wire `line` (1..8) — what `digitalIn()` hands out.
    [[nodiscard]] FakeDigitalIn& fakeDigitalIn(std::uint8_t line) { return inputs_[checkedLine(line)]; }
    /// This provider's own read-fault counter for smart `port` — what `readFaults(port)` answers
    /// once a distance or optical sensor was served there. A test sets it `failing` (MECH-A1).
    [[nodiscard]] FakeReadFaults& fakeReadFaults(std::uint8_t port) {
        return readFaults_[checkedIndex(port)];
    }
    /// This provider's own read-fault counter for 3-wire `line` — what `lineReadFaults(line)`
    /// answers once a digital input was served there.
    [[nodiscard]] FakeReadFaults& fakeLineReadFaults(std::uint8_t line) {
        return lineReadFaults_[checkedLine(line)];
    }
    /// This provider's own camera for `port` — what `aiVision()` hands out (VIS1-A1). A test sets its
    /// frames, its failing and its status; it is also what `readFaults(port)` answers once served.
    [[nodiscard]] FakeVisionFrames& fakeCamera(std::uint8_t port) { return cameras_[checkedIndex(port)]; }
    /// Whether `aiVision()` SERVED a camera on `port` (VIS1-A1).
    [[nodiscard]] bool cameraMade(std::uint8_t port) const { return cameraMade_[checkedIndex(port)]; }
    /// The settings the request that made the camera on `port` passed — how a test proves the builder
    /// handed the config's block through. Read it only when `cameraMade(port)`.
    [[nodiscard]] const AiVisionSettings& cameraSettings(std::uint8_t port) const {
        return cameraSettings_[checkedIndex(port)];
    }

    // ── the refusal switch ─────────────────────────────────────────────────────────────

    /// Make every device request for `port` answer nullptr, with a refusal naming the port — the
    /// host stand-in for an adapter constructor throwing on an empty or dead port.
    void refusePort(std::uint8_t port) { refused_[checkedIndex(port)] = true; }
    /// Undo refusePort(); a port is allowed by default, so this is only ever needed to re-arm one.
    void allowPort(std::uint8_t port) { refused_[checkedIndex(port)] = false; }
    /// Whether `port` is currently refused.
    [[nodiscard]] bool isRefused(std::uint8_t port) const { return refused_[checkedIndex(port)]; }
    /// Make every `digitalOut()` request for 3-wire `line` (1..8) answer nullptr, with a refusal
    /// naming the line — the host stand-in for a provider that will not make a line.
    void refuseLine(std::uint8_t line) { lineRefused_[checkedLine(line)] = true; }
    /// Undo refuseLine().
    void allowLine(std::uint8_t line) { lineRefused_[checkedLine(line)] = false; }

    // ── what was ASKED FOR, so a test can prove the builder passed the config through ──

    /// Whether `motor()` has been called for `port` AND served it — the evidence that the builder
    /// asked for exactly the ports the config names, and no others. A REFUSED request does not
    /// set it; `motorRequestCount()` counts those too.
    [[nodiscard]] bool motorRequested(std::uint8_t port) const {
        return motorRequested_[checkedIndex(port)];
    }
    /// How many times `motor()` has been called for physical `port`, SERVED OR REFUSED (G1 Part
    /// 2a). This is the only way to prove a port was never ASKED for: `motorRequested()` stays
    /// false for a refused port, so a builder that asked for an absent port, got a refusal and
    /// ignored it would look innocent to it. Counted by |signedPort| before any refusal check.
    [[nodiscard]] int motorRequestCount(std::uint8_t port) const {
        return motorRequests_[checkedIndex(port)];
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
    /// How many times `digitalOut()` has been called for 3-wire `line` (1..8), SERVED OR REFUSED —
    /// counted before any refusal, so "never asked for" is provable (validate() must not ask: asking
    /// drives the line).
    [[nodiscard]] int digitalOutRequestCount(std::uint8_t line) const {
        return lineRequests_[checkedLine(line)];
    }
    /// Whether a digital INPUT was served on 3-wire `line` (MECH-A1). Asking for one never drives it.
    [[nodiscard]] bool digitalInMade(std::uint8_t line) const { return inputMade_[checkedLine(line)]; }
    /// Whether 3-wire `line` has been MADE (served once, and so driven once).
    [[nodiscard]] bool digitalOutMade(std::uint8_t line) const { return lineMade_[checkedLine(line)]; }
    /// The level 3-wire `line` was MADE at — the `initialState` of the request that made it. Read
    /// it only when `digitalOutMade(line)`; false otherwise, which means nothing.
    [[nodiscard]] bool digitalOutInitialState(std::uint8_t line) const {
        return lineInitial_[checkedLine(line)];
    }

    // ── IDeviceProvider ────────────────────────────────────────────────────────────────

    /// The motor on `signedPort` — the bound one if the port is bound, else this provider's own
    /// fake. nullptr + a refusal on an out-of-range or refused port. Idempotent by construction:
    /// the storage is fixed, so the same port always yields the same address.
    [[nodiscard]] IMotor* motor(std::int8_t signedPort, MotorCartridge cartridge) override {
        const int port = DrivetrainDescription::portMagnitude(signedPort);
        if (port >= 1 && port <= static_cast<int>(kMaxSmartPorts)) {
            ++motorRequests_[static_cast<std::size_t>(port)];  // counted BEFORE any refusal
        }
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

    /// The digital out on 3-wire `line` — bound or own fake; DRIVEN to `initialState` by the
    /// request that makes it, never by a later one (header). nullptr + a refusal for a line
    /// outside 1..8 or a refused line. PRECONDITION: a later request states the same
    /// `initialState` the line was made at.
    [[nodiscard]] IDigitalOut* digitalOut(std::uint8_t line, bool initialState) override {
        const int l = static_cast<int>(line);
        if (l >= 1 && l <= static_cast<int>(kBrainAdiLines)) {
            ++lineRequests_[static_cast<std::size_t>(l)];  // counted BEFORE any refusal
        } else {
            std::snprintf(refusal_, sizeof refusal_,
                          "3-wire port %d is not a brain 3-wire port (1..8 are A..H)", l);
            return nullptr;
        }
        const auto i = static_cast<std::size_t>(l);
        if (lineRefused_[i]) {
            std::snprintf(refusal_, sizeof refusal_, "3-wire port %c (line %d) has no digital out",
                          static_cast<char>('A' + l - 1), l);
            return nullptr;
        }
        SHULIB_PRECONDITION(!inputMade_[i],
                            "FakeDeviceProvider: a 3-wire line made as an INPUT was asked for as an "
                            "output -- one line, one direction");
        IDigitalOut* out =
            boundLine_[i] != nullptr ? boundLine_[i] : static_cast<IDigitalOut*>(&lines_[i]);
        if (lineMade_[i]) {
            SHULIB_PRECONDITION(lineInitial_[i] == initialState,
                                "FakeDeviceProvider: a 3-wire line was asked for again with a "
                                "different initial state -- one line, one configuration");
            return out;
        }
        out->set(initialState);  // MAKING a line drives it, as on the brain (HA-119)
        lineMade_[i] = true;
        lineInitial_[i] = initialState;
        return out;
    }

    /// MECH-A1: the distance sensor on `port` — this provider's own fake; nullptr + a refusal on an
    /// out-of-range or refused port. Idempotent (fixed storage).
    [[nodiscard]] IDistance* distance(std::uint8_t port) override {
        const int p = static_cast<int>(port);
        if (!servable(p, "distance sensor")) {
            return nullptr;
        }
        const auto i = static_cast<std::size_t>(p);
        distanceMade_[i] = true;
        return &distances_[i];
    }

    /// MECH-A1: the optical sensor on `port` — this provider's own fake; nullptr + a refusal on an
    /// out-of-range or refused port. Idempotent.
    [[nodiscard]] IOptical* optical(std::uint8_t port) override {
        const int p = static_cast<int>(port);
        if (!servable(p, "optical sensor")) {
            return nullptr;
        }
        const auto i = static_cast<std::size_t>(p);
        opticalMade_[i] = true;
        return &opticals_[i];
    }

    /// MECH-A1: the digital input on 3-wire `line` — this provider's own fake; nullptr + a refusal
    /// for a line outside 1..8 or a refused line. Drives nothing. PRECONDITION: the line was not
    /// made as an output (one line, one direction). Idempotent.
    [[nodiscard]] IDigitalIn* digitalIn(std::uint8_t line) override {
        const int l = static_cast<int>(line);
        if (l < 1 || l > static_cast<int>(kBrainAdiLines)) {
            std::snprintf(refusal_, sizeof refusal_,
                          "3-wire port %d is not a brain 3-wire port (1..8 are A..H)", l);
            return nullptr;
        }
        const auto i = static_cast<std::size_t>(l);
        if (lineRefused_[i]) {
            std::snprintf(refusal_, sizeof refusal_, "3-wire port %c (line %d) has no digital in",
                          static_cast<char>('A' + l - 1), l);
            return nullptr;
        }
        SHULIB_PRECONDITION(!lineMade_[i],
                            "FakeDeviceProvider: a 3-wire line made as an OUTPUT was asked for as an "
                            "input -- one line, one direction");
        inputMade_[i] = true;
        return &inputs_[i];
    }

    /// VIS1-A1: the camera on `port` — this provider's own FakeVisionFrames; nullptr + a refusal on an
    /// out-of-range or refused port, for a settings problem, or for settings other than the made
    /// camera's (header). Idempotent for equal settings (fixed storage).
    [[nodiscard]] IVisionFrames* aiVision(std::uint8_t port, const AiVisionSettings& settings) override {
        const int p = static_cast<int>(port);
        if (!servable(p, "AI Vision camera")) {
            return nullptr;
        }
        if (const char* problem = aiVisionSettingsProblem(settings)) {
            std::snprintf(refusal_, sizeof refusal_, "port %d: the camera's settings: %s", p, problem);
            return nullptr;
        }
        const auto i = static_cast<std::size_t>(p);
        if (cameraMade_[i]) {
            if (!sameAiVisionSettings(cameraSettings_[i], settings)) {
                std::snprintf(refusal_, sizeof refusal_,
                              "port %d: the camera was already set up with other settings -- one owner",
                              p);
                return nullptr;
            }
            return &cameras_[i];
        }
        cameraMade_[i] = true;
        cameraSettings_[i] = settings;
        return &cameras_[i];
    }

    /// MECH-A1: this provider's own read-fault counter for smart `port` — only where a distance or
    /// optical sensor was SERVED (header: the on-robot contract) — or (VIS1-A1) the camera served
    /// there, as its own count; nullptr otherwise. Never makes.
    [[nodiscard]] const IReadFaults* readFaults(std::uint8_t port) const noexcept override {
        const int p = static_cast<int>(port);
        if (p < 1 || p > static_cast<int>(kMaxSmartPorts)) {
            return nullptr;
        }
        const auto i = static_cast<std::size_t>(p);
        if (distanceMade_[i] || opticalMade_[i]) {
            return &readFaults_[i];
        }
        return cameraMade_[i] ? &cameras_[i] : nullptr;
    }

    /// MECH-A1: this provider's own read-fault counter for 3-wire `line` — only where a digital
    /// input was SERVED; nullptr otherwise. Never makes.
    [[nodiscard]] const IReadFaults* lineReadFaults(std::uint8_t line) const noexcept override {
        const int l = static_cast<int>(line);
        if (l < 1 || l > static_cast<int>(kBrainAdiLines)) {
            return nullptr;
        }
        const auto i = static_cast<std::size_t>(l);
        return inputMade_[i] ? &lineReadFaults_[i] : nullptr;
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

    /// Bounds-check a 3-wire line (1..8) for the TEST hooks and return it as an index — a
    /// precondition, like checkedIndex(): the library never calls these hooks.
    [[nodiscard]] std::size_t checkedLine(std::uint8_t line) const {
        SHULIB_PRECONDITION(line >= 1 && line <= kBrainAdiLines,
                            "FakeDeviceProvider: a 3-wire line must be in 1..8");
        return static_cast<std::size_t>(line);
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
    FakeDistance distances_[kSlots]{};  // MECH-A1: piece sensors and their counts
    FakeOptical opticals_[kSlots]{};
    FakeReadFaults readFaults_[kSlots]{};
    bool distanceMade_[kSlots]{};
    bool opticalMade_[kSlots]{};
    FakeVisionFrames cameras_[kSlots]{};  // VIS1-A1: the camera, its own read-fault count
    AiVisionSettings cameraSettings_[kSlots]{};
    bool cameraMade_[kSlots]{};

    IMotor* boundMotor_[kSlots]{};
    IImu* boundImu_[kSlots]{};
    IGps* boundGps_[kSlots]{};
    IRotation* boundRotation_[kSlots]{};

    bool refused_[kSlots]{};
    bool motorRequested_[kSlots]{};
    int motorRequests_[kSlots]{};
    MotorCartridge cartridges_[kSlots]{};
    std::int8_t signedPorts_[kSlots]{};
    units::Length leverForward_[kSlots]{};
    units::Length leverLeft_[kSlots]{};

    FakeDigitalOut lines_[kLineSlots]{};
    IDigitalOut* boundLine_[kLineSlots]{};
    bool lineRefused_[kLineSlots]{};
    int lineRequests_[kLineSlots]{};
    bool lineMade_[kLineSlots]{};
    bool lineInitial_[kLineSlots]{};
    FakeDigitalIn inputs_[kLineSlots]{};  // MECH-A1: limit switches and their counts
    FakeReadFaults lineReadFaults_[kLineSlots]{};
    bool inputMade_[kLineSlots]{};

    char refusal_[160]{};  // 96 until VIS1-A1: a camera settings refusal carries a whole sentence
};

}  // namespace shulib::hal::fake
