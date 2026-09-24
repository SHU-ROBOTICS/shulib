#pragma once
//
// ProsDeviceProvider — hal::IDeviceProvider over the real V5 adapters (chunk G1 Part 2a,
// 2026-09-24).
//
// ── What it is ──────────────────────────────────────────────────────────────────────
// The one object on the brain that knows what a port is. robot::Robot asks it for "the motor
// on port -11 with a blue cartridge", "the IMU on port 2", and it answers with a hal/pros
// adapter it constructed and OWNS (ruling B2: the provider owns, the graph borrows) — or with
// nullptr and a sentence in lastRefusal() naming the port. On the host the same seam is
// served by hal::fake::FakeDeviceProvider; the builder cannot tell the two apart, which is the
// point of the seam (hal/device_provider.hpp).
//
// ── THE REGISTRY IS ASKED BEFORE ANY ADAPTER IS CONSTRUCTED ──────────────────────────
// The G1 Part 2 brief's first version assumed a provider could refuse a dead port by catching
// the adapter's throw. The 2026-09-24 audit measured that false: only ProsMotor fails loudly on
// a port with nothing in it (its HA-98 read-back). ProsImu's and ProsRotation's constructors
// cannot throw at all, and ProsGps DEFERS its offset check on a device it cannot read. A
// provider that refused only on a throw would hand back a LIVE adapter for an EMPTY IMU, GPS or
// rotation port — and the robot would boot, calibrate nothing, and read a heading of zero
// forever.
//
// So before a port device is constructed, this provider asks an hal::IDeviceRegistry what is
// physically on that port (hal/device_registry.hpp — the brain's own table, read without
// opening anything). Any answer but the expected kind, EMPTY included, is a refusal naming the
// port, what was found and what was expected:
//
//     port 13: nothing plugged in (expected a motor)
//     port 2: a rotation sensor (expected an IMU)
//
// The registry's KIND answer being right for an IMU, a rotation sensor and a GPS is a hardware
// belief this file newly rests on — A4 register HA-141 (HA-138 states only the port indexing,
// which ProsDeviceRegistry owns). Reasoned, not measured: for those three kinds the registry is
// now the ONLY thing that can refuse an empty port, so its settle step is on the register.
//
// ── …AND THE ADAPTER'S OWN CHECK STILL RUNS ──────────────────────────────────────────
// The registry says WHAT is plugged in; it cannot say whether a motor ACCEPTS its
// configuration. ProsMotor's constructor sets degrees and the stated gearset and reads both
// back (HA-98). A motor whose kind is right but whose read-back fails is refused too, carrying
// the adapter's own message:
//
//     port 13: the motor rejected its configuration: ProsMotor: encoder units read-back ...
//
// Two checks, both kept (the brief's ruling M2): the registry catches an empty or swapped port
// before anything is opened; the adapter catches a present device that will not be configured.
// Each construction sits inside its OWN try, so one bad port is one refusal and never an
// exception out of the builder. The adapter's message is copied through an ASCII filter,
// because a refusal ends up on the brain's screen and its fonts are ASCII-only (ProsMotor's
// messages carry an em dash, which the filter turns into '-').
//
// ── ONE PORT, ONE DEVICE, ONE CONFIGURATION ─────────────────────────────────────────
// A device is constructed on the FIRST request that the registry and the adapter both allow,
// and every later request for that port returns the SAME object — the graph holds these
// pointers for the life of the robot, and a second adapter on one port would leave half of it
// reading an object nobody commands (IDeviceProvider's idempotence contract). The registry is
// consulted when a device is MADE, not on every later request: once it exists it belongs to the
// graph, and losing it at runtime is the adapter's sentinel screening and the health layer's
// business, not a reason to hand the builder a different answer the second time it asks.
//
// Asking for a port that already has a device with a DIFFERENT configuration — the motor with
// the other sign or another cartridge, the GPS with another lever arm, a second IMU or GPS on
// another port — is a PRECONDITION VIOLATION, not a refusal. It cannot be a wiring fault (the
// wiring did not change between the two calls); it is two parts of one program disagreeing about
// one port, and quietly returning the first configuration would apply the wrong sign to a drive
// side without a word.
//
// ── IMU CALIBRATION STARTS HERE, EXACTLY ONCE ───────────────────────────────────────
// imu() starts the sensor's calibration (ProsImu::calibrate(), non-blocking — HA-108) when it
// constructs the adapter, and never again: a second reset() would re-zero the sensor under a
// live boot heading (HA-05), which ProsImu refuses by precondition. A program built on this
// provider therefore never calls calibrate() itself. The boot heading is 0 — the robot boots
// believing it faces +X — which is what both of src/main.cpp's graphs do today (HA-05's one
// owner is the start pose; a provider option for it arrives when a program needs one).
//
// ── WHAT IT DOES NOT DO ─────────────────────────────────────────────────────────────
// It applies NO dead-port policy. A port the boot check reported MISSING is never requested
// (robot::BuildOptions::absentDrivePorts); anything else it will not make refuses the build,
// through the builder, naming the port. It never hands out a fake — a test double in a
// competition binary erases the difference between "no device" and "a test injects here" (R3b
// §6). It allocates nothing: every device lives in fixed per-port std::optional storage. The
// controller and the controller's screen are not on the seam; a program keeps its own.
//
// ── NO HARDWARE CLAIM ───────────────────────────────────────────────────────────────
// Host-tested through test/pros_shim/ only, which tests this class against our BELIEF about
// PROS and cannot test the belief (shim_control.hpp says so in full). No program constructs one
// yet: src/ is unchanged, and "shulib Teleop" is re-pointed at it in G1 Part 2b.
//
// No pros header is included here directly — only the hal/pros adapters, whose own diagnostic
// fences cover the vendored code (test/pros_adapter_fence_test.cpp requires a file with no
// direct pros include to carry no fence at all).

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <optional>

#include "shulib/core/check.hpp"
#include "shulib/diag/term_sink.hpp"
#include "shulib/hal/device_provider.hpp"
#include "shulib/hal/device_registry.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/pros/battery.hpp"
#include "shulib/hal/pros/char_sink.hpp"
#include "shulib/hal/pros/clock.hpp"
#include "shulib/hal/pros/device_registry.hpp"
#include "shulib/hal/pros/gps.hpp"
#include "shulib/hal/pros/imu.hpp"
#include "shulib/hal/pros/motor.hpp"
#include "shulib/hal/pros/rotation.hpp"
#include "shulib/hal/pros/tick_pacer.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal::pros {

/// The on-robot `hal::IDeviceProvider`: real adapters made on demand, one per port, each only
/// after the brain's registry says the port carries the expected kind of device, and owned here
/// for the life of the program (header). Refusal is `nullptr` plus a sentence naming the port;
/// a conflicting second request for a made device is a precondition violation. Neither copyable
/// nor movable — the graph holds pointers into this object's storage.
class ProsDeviceProvider final : public IDeviceProvider {
public:
    /// One more than the highest physical port, so a port number indexes the storage directly
    /// and slot 0 is never used.
    static constexpr std::size_t kSlots = kMaxSmartPorts + 1;

    /// The longest refusal sentence this provider writes, NUL included. Sized so the builder's
    /// verdict (`robot::BuildVerdict::why`, 192 bytes, which prefixes a role such as "a RIGHT
    /// drive motor cannot be made: ") still carries the port and the start of the adapter's
    /// reason; longer text truncates, never overflows.
    static constexpr std::size_t kRefusalBytes = 160;

    /// The provider as it runs on a brain: it owns a `ProsDeviceRegistry` and asks it, port by
    /// port, before constructing anything. Constructing the provider opens NO device — every
    /// adapter is made on its first request.
    ProsDeviceProvider() : ownRegistry_{}, registry_{&ownRegistry_} {}

    /// The same provider over a caller's registry — a `hal::fake::FakeDeviceRegistry` in a host
    /// test that wants to program the census without the shim, or a program that already holds
    /// one. The registry is BORROWED and must outlive the provider.
    explicit ProsDeviceProvider(const IDeviceRegistry& registry)
        : ownRegistry_{}, registry_{&registry} {}

    /// Non-copyable: a copy would duplicate the adapters while the graph kept pointing at the
    /// originals.
    ProsDeviceProvider(const ProsDeviceProvider&) = delete;
    /// Non-movable, same reasoning.
    ProsDeviceProvider(ProsDeviceProvider&&) = delete;
    /// Non-copy-assignable, same reasoning.
    ProsDeviceProvider& operator=(const ProsDeviceProvider&) = delete;
    /// Non-move-assignable, same reasoning.
    ProsDeviceProvider& operator=(ProsDeviceProvider&&) = delete;
    /// Destroys every adapter this provider made — so it must outlive every graph built from it.
    ~ProsDeviceProvider() override = default;

    // ── IDeviceProvider: the port devices ──────────────────────────────────────────────

    /// The motor on `signedPort` (|p| in 1..21, NEGATIVE = reversed by PROS, once, inside the
    /// adapter), configured with `cartridge`'s gearset. Made on the first request the registry
    /// (kind MOTOR) and the adapter's read-back both allow; the same object ever after.
    /// nullptr + a refusal for a port outside 1..21, a cartridge byte that is not a V5 gearset,
    /// a port the registry says is empty or carries something else, or a motor that rejects its
    /// configuration. PRECONDITION: a later request for a made port states the SAME signed port
    /// and cartridge (header: one port, one configuration).
    [[nodiscard]] IMotor* motor(std::int8_t signedPort, MotorCartridge cartridge) override {
        const int port = DrivetrainDescription::portMagnitude(signedPort);
        if (!isSmartPort(port)) {
            std::snprintf(refusal_, sizeof refusal_,
                          "port %d is not a smart port (ports are 1..21)",
                          static_cast<int>(signedPort));
            return nullptr;
        }
        const auto i = static_cast<std::size_t>(port);
        if (motors_[i].has_value()) {
            if (motorSigned_[i] != signedPort || motorCartridge_[i] != cartridge) {
                std::snprintf(conflict_, sizeof conflict_,
                              "ProsDeviceProvider: port %d already has a motor made as %d with "
                              "cartridge %d; asked again as %d with cartridge %d -- one port, "
                              "one configuration",
                              port, static_cast<int>(motorSigned_[i]),
                              static_cast<int>(motorCartridge_[i]), static_cast<int>(signedPort),
                              static_cast<int>(cartridge));
                SHULIB_PRECONDITION(false, conflict_);
            }
            return &*motors_[i];
        }
        MotorGearset gearset = MotorGearset::Green;
        if (!gearsetOf(cartridge, gearset)) {
            std::snprintf(refusal_, sizeof refusal_,
                          "port %d: cartridge byte %d is not a V5 gearset (red, green or blue)",
                          port, static_cast<int>(cartridge));
            return nullptr;
        }
        if (!expectKind(port, DeviceKind::Motor, "a motor")) {
            return nullptr;
        }
        try {
            motors_[i].emplace(signedPort, gearset);  // AS STORED: the one place a sign lives
        } catch (const std::exception& e) {
            motors_[i].reset();
            refuseRejected(port, "motor", e.what());
            return nullptr;
        }
        motorSigned_[i] = signedPort;
        motorCartridge_[i] = cartridge;
        return &*motors_[i];
    }

    /// The inertial sensor on `port`. Made on the first request the registry (kind IMU) allows,
    /// and its calibration STARTED right then, exactly once (header); the same object ever
    /// after. nullptr + a refusal for a port outside 1..21 or a port the registry says is empty
    /// or carries something else. PRECONDITION: this provider serves ONE IMU, so a request for a
    /// different port after one was made is a programming error.
    [[nodiscard]] IImu* imu(std::uint8_t port) override {
        const int p = static_cast<int>(port);
        if (!isSmartPort(p)) {
            std::snprintf(refusal_, sizeof refusal_,
                          "port %d is not a smart port (ports are 1..21)", p);
            return nullptr;
        }
        if (imu_.has_value()) {
            if (imuPort_ != port) {
                std::snprintf(conflict_, sizeof conflict_,
                              "ProsDeviceProvider: an IMU was already made on port %d; asked for "
                              "one on port %d -- this provider serves one IMU",
                              static_cast<int>(imuPort_), p);
                SHULIB_PRECONDITION(false, conflict_);
            }
            return &*imu_;
        }
        if (!expectKind(p, DeviceKind::Imu, "an IMU")) {
            return nullptr;
        }
        try {
            imu_.emplace(port, math::Angle{}, clock_);  // boot heading 0: faces +X (header)
            imu_->calibrate();                          // started ONCE, here (HA-108, HA-05)
        } catch (const std::exception& e) {
            imu_.reset();
            refuseRejected(p, "IMU", e.what());
            return nullptr;
        }
        imuPort_ = port;
        return &*imu_;
    }

    /// The GPS on `port`, with its lever arm from the robot centre (+X forward, +Y left,
    /// inches), which the adapter removes once, at the edge. Made on the first request the
    /// registry (kind GPS) and the adapter's own boot check both allow; the same object ever
    /// after. nullptr + a refusal for a port outside 1..21, a port the registry says is empty or
    /// carries something else, or a GPS the adapter rejects (a configured firmware offset,
    /// HA-06). PRECONDITION: ONE GPS per provider, and a later request states the same port and
    /// the same lever arm.
    [[nodiscard]] IGps* gps(std::uint8_t port, units::Length leverArmForward,
                            units::Length leverArmLeft) override {
        const int p = static_cast<int>(port);
        if (!isSmartPort(p)) {
            std::snprintf(refusal_, sizeof refusal_,
                          "port %d is not a smart port (ports are 1..21)", p);
            return nullptr;
        }
        if (gps_.has_value()) {
            if (gpsPort_ != port || gpsLeverForward_.value() != leverArmForward.value()
                || gpsLeverLeft_.value() != leverArmLeft.value()) {
                std::snprintf(conflict_, sizeof conflict_,
                              "ProsDeviceProvider: a GPS was already made on port %d with lever "
                              "arm (%g, %g) in; asked for port %d with (%g, %g) in -- one GPS, "
                              "one configuration",
                              static_cast<int>(gpsPort_), gpsLeverForward_.value(),
                              gpsLeverLeft_.value(), p, leverArmForward.value(),
                              leverArmLeft.value());
                SHULIB_PRECONDITION(false, conflict_);
            }
            return &*gps_;
        }
        if (!expectKind(p, DeviceKind::Gps, "a GPS")) {
            return nullptr;
        }
        try {
            gps_.emplace(port, leverArmForward, leverArmLeft);
        } catch (const std::exception& e) {
            gps_.reset();
            refuseRejected(p, "GPS", e.what());
            return nullptr;
        }
        gpsPort_ = port;
        gpsLeverForward_ = leverArmForward;
        gpsLeverLeft_ = leverArmLeft;
        return &*gps_;
    }

    /// The rotation sensor on `port` — a tracking wheel's encoder, never reversed by port (the
    /// seam's port is unsigned; a pod's direction is its offset's sign). Made on the first
    /// request the registry (kind ROTATION) allows; the same object ever after. nullptr + a
    /// refusal for a port outside 1..21 or a port the registry says is empty or carries
    /// something else.
    [[nodiscard]] IRotation* rotation(std::uint8_t port) override {
        const int p = static_cast<int>(port);
        if (!isSmartPort(p)) {
            std::snprintf(refusal_, sizeof refusal_,
                          "port %d is not a smart port (ports are 1..21)", p);
            return nullptr;
        }
        const auto i = static_cast<std::size_t>(p);
        if (rotations_[i].has_value()) {
            return &*rotations_[i];
        }
        if (!expectKind(p, DeviceKind::Rotation, "a rotation sensor")) {
            return nullptr;
        }
        try {
            rotations_[i].emplace(static_cast<std::int8_t>(p));
        } catch (const std::exception& e) {
            rotations_[i].reset();
            refuseRejected(p, "rotation sensor", e.what());
            return nullptr;
        }
        return &*rotations_[i];
    }

    // ── IDeviceProvider: the port-less handles ─────────────────────────────────────────

    /// The V5's real clock (`ProsClock`, microseconds → seconds once). Never refused.
    [[nodiscard]] IClock& clock() override { return clock_; }
    /// The V5 battery (`ProsBattery`). Never refused.
    [[nodiscard]] IBattery& battery() override { return battery_; }
    /// The diagnostics sink: a `diag::TermSink` over the USB serial (`ProsCharSink`, stdout) —
    /// exactly the sink both of src/main.cpp's graphs build by hand. Never refused.
    [[nodiscard]] ITelemetrySink& telemetry() override { return telemetry_; }
    /// The 10 ms `ProsTickPacer` (`pros::Task::delay_until`, anchored on its first pace). Never
    /// refused.
    [[nodiscard]] motion::ITickPacer& pacer() override { return pacer_; }

    /// Why the most recent refused request returned nullptr, naming the port and — when the
    /// registry answered — what was found and what was expected. Plain ASCII. The empty string
    /// until something is refused; not cleared by a later success (the builder reads it
    /// immediately after the nullptr it explains).
    [[nodiscard]] const char* lastRefusal() const noexcept override { return refusal_; }

private:
    /// The human-stated cartridge as the adapter's gearset. False for a byte outside the enum —
    /// a config that got past its own validation with one is refused here rather than guessed.
    [[nodiscard]] static bool gearsetOf(MotorCartridge c, MotorGearset& out) noexcept {
        switch (c) {
            case MotorCartridge::Red: out = MotorGearset::Red; return true;
            case MotorCartridge::Green: out = MotorGearset::Green; return true;
            case MotorCartridge::Blue: out = MotorGearset::Blue; return true;
        }
        return false;
    }

    /// What the registry found, as the words of a refusal: "nothing plugged in", "a rotation
    /// sensor", … Never invents: a byte outside the enum reads "an unrecognised device".
    [[nodiscard]] static const char* foundWords(DeviceKind k) noexcept {
        switch (k) {
            case DeviceKind::None: return "nothing plugged in";
            case DeviceKind::Motor: return "a motor";
            case DeviceKind::Imu: return "an IMU";
            case DeviceKind::Rotation: return "a rotation sensor";
            case DeviceKind::Distance: return "a distance sensor";
            case DeviceKind::Optical: return "an optical sensor";
            case DeviceKind::Vision: return "a vision sensor";
            case DeviceKind::Gps: return "a GPS";
            case DeviceKind::Radio: return "the radio";
            case DeviceKind::Adi: return "an ADI expander";
            case DeviceKind::Other: return "an unrecognised device";
        }
        return "an unrecognised device";
    }

    /// THE REGISTRY CHECK (header; A4 register HA-141). True when `port` carries `expected`;
    /// otherwise writes the refusal — port, what was found, what was expected — and returns
    /// false. Called before every construction and never after one.
    [[nodiscard]] bool expectKind(int port, DeviceKind expected, const char* expectedWords) {
        const DeviceKind found = registry_->pluggedKind(port);
        if (found == expected) {
            return true;
        }
        std::snprintf(refusal_, sizeof refusal_, "port %d: %s (expected %s)", port,
                      foundWords(found), expectedWords);
        return false;
    }

    /// Write "port N: the <what> rejected its configuration: <adapter's message>", the message
    /// copied BY HAND through an ASCII filter and bounded by the buffer: a UTF-8 lead byte
    /// becomes '-', its continuation bytes are dropped, and a control byte becomes a space, so
    /// nothing a font cannot draw reaches a screen and nothing overruns.
    void refuseRejected(int port, const char* what, const char* adapterMessage) noexcept {
        const int head = std::snprintf(refusal_, sizeof refusal_,
                                       "port %d: the %s rejected its configuration: ", port,
                                       what);
        std::size_t n = head < 0 ? 0 : static_cast<std::size_t>(head);
        if (n >= sizeof refusal_) {
            return;  // snprintf already truncated and terminated
        }
        const char* text = adapterMessage != nullptr ? adapterMessage : "(no message)";
        for (std::size_t k = 0; text[k] != '\0' && n + 1 < sizeof refusal_; ++k) {
            const auto byte = static_cast<unsigned char>(text[k]);
            if (byte >= 0xC0U) {
                refusal_[n++] = '-';  // a UTF-8 lead byte: one ASCII stand-in per character
            } else if (byte >= 0x80U) {
                continue;  // a UTF-8 continuation byte: already represented
            } else if (byte < 0x20U || byte == 0x7FU) {
                refusal_[n++] = ' ';
            } else {
                refusal_[n++] = static_cast<char>(byte);
            }
        }
        refusal_[n] = '\0';
    }

    // ── the registry the check asks (owned, or borrowed from the caller) ───────────────
    ProsDeviceRegistry ownRegistry_;
    const IDeviceRegistry* registry_;

    // ── the port-less handles, in construction order (the sink needs the clock) ────────
    ProsClock clock_{};
    ProsBattery battery_{};
    ProsCharSink usb_{};
    diag::TermSink telemetry_{clock_, usb_};
    ProsTickPacer pacer_{};

    // ── the port devices: fixed storage, made on demand, owned for the program's life ──
    std::optional<ProsMotor> motors_[kSlots]{};
    std::int8_t motorSigned_[kSlots]{};
    MotorCartridge motorCartridge_[kSlots]{};
    std::optional<ProsImu> imu_{};
    std::uint8_t imuPort_ = 0;
    std::optional<ProsGps> gps_{};
    std::uint8_t gpsPort_ = 0;
    units::Length gpsLeverForward_{0.0};
    units::Length gpsLeverLeft_{0.0};
    std::optional<ProsRotation> rotations_[kSlots]{};

    char refusal_[kRefusalBytes]{};
    char conflict_[192]{};
};

}  // namespace shulib::hal::pros
