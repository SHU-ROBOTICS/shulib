#pragma once
//
// robot::Robot — a config::RobotConfig, wired (chunk G1 Part 1, 2026-09-18).
//
// ── What this class is ──────────────────────────────────────────────────────────────
// The composition root, written once, for every robot this library will ever run. It owns
// the whole PROS-free object graph — kinematics, motor groups, geometry, odometry, fusion,
// the GPS corrector when there is a GPS, the Localizer, the fault latch, the health
// monitor, the RobotContext, the MotionDeps and the Chassis — and it BORROWS the devices
// from an hal::IDeviceProvider, which is the one object that knows what a port is on this
// target. Two lines at the composition root:
//
//     hal::pros::ProsDeviceProvider devices;      // the adapters, made on demand, by port
//     robot::Robot robot{kRobotConfig, devices};  // the whole graph, wired
//
// and `robot.chassis()` is the same facade a hand-wired graph produces. The provider on the first
// line exists since G1 Part 2a (hal/pros/device_provider.hpp, host-tested through the PROS shim);
// since G1 Part 2b the programs that drive robot two are built from these two pieces ("shulib
// Teleop" first, which T4 Part 3 retired; "shulib Drive", src/programs/drive_program.cpp, since
// then). On a brain the second line is spelled the way those programs spell it: `robot::validate()`
// first (it names the port or the field before anything is constructed), then
// `std::optional<Robot>::emplace` into storage that lives for the whole program, once, from
// `initialize()` (below: a Robot is constructed where it will live).
//
// ── IT ADDS A WAY TO WIRE; IT NEVER CHANGES THE PHYSICS (ruling B4) ─────────────────
// The graph below is the SAME objects, in the SAME order, with the SAME arguments as the
// hand-written graphs src/main.cpp carried (robot two's until G1 Part 2b; the invented X-drive's
// until 2c) and as the host rigs. That is not an aspiration:
// it is the acceptance test. test/robot_builder_test.cpp drives a builder-made Chassis and
// a hand-wired one across the A2 plant and requires the per-tick ground-truth samples to be
// memcmp-EQUAL, the commanded volts equal, and the exit reason and tick count equal —
// through a clean and a hostile scenario. If the builder and the longhand disagree by one
// ulp, the builder has changed something, and a rounding note is not an acceptable answer.
//
// THE BUILDER CONSTRUCTS AND NEVER TICKS. No localizer update, no motion tick, no motor
// write happens anywhere in this file. That is what makes the ordering above a discipline
// rather than a hazard — nothing here has an observable time-ordering, because no time
// passes and no device is written during construction. ONE exception, and it is not a
// command the builder invents (T3's addendum B2): a 3-wire line cannot be MADE without being
// DRIVEN (A4 register HA-119), so a pneumatic mechanism's lines are asked for at the config's
// stated START level, and on a brain that is the moment they take it — last, after the whole
// drive graph (the MECHANISMS section below).
//
// ── validate() is PURE and TOTAL; construction PRECONDITIONS on it (ruling B3) ──────
// A robot that cannot be built says WHICH PORT OR FIELD before a single object exists. That
// is the R3b refusal-at-boot rule, made library: `validate(cfg, devices)` checks the config
// against its own rules, makes the F5 wheel-count cross-check, and asks the provider for
// every device the config names — returning a verdict with a sentence. The constructor runs
// exactly that check FIRST and throws PreconditionError carrying the same sentence.
//
// Rejected: a factory returning std::optional<Robot>. Chassis is non-copyable and
// non-movable (its scheduler is pinned by a self-referential command-id stamp), so Robot is
// too, and returning one by value would depend on elision games the V5 toolchain should not
// be asked to play. A Robot is constructed WHERE IT WILL LIVE: on a brain, an EMPTY
// `std::optional<Robot>` that lives for the program, emplaced ONCE from `initialize()` —
// src/programs/drive_program.cpp's pattern. Never a `static Robot r{…}`: at namespace scope it is built
// before `initialize()` has installed the on-robot precondition handler, in an order across
// translation units nobody controls; inside a function, one whose constructor THROWS is
// constructed again on the next call, and PROS calls opcontrol() again after every disable. In a
// test, `std::optional<Robot>::emplace` too.
//
// ── ABSENT IS NOT NULL ──────────────────────────────────────────────────────────────
// A robot with no GPS gets hal::AbsentGps in its context and NO corrector in its Localizer.
// It never gets a nullptr (RobotContext's preconditions throw on one, and they are right to)
// and it never gets a fake — a test double inside a competition binary erases the difference
// between "no device is installed" and "a test injects here". Tags and vision are ABSENT
// unconditionally in Part 1, because no camera adapter exists yet (R2 owns it) and a field
// for hardware the library cannot open would be a promise rather than a description.
//
// ── ABSENT DRIVE PORTS: the boot check decides, the builder obeys (G1 Part 2a, M1) ──
// A tank drive port the boot check found DEAD (calibration/boot_check.hpp — the registry is read
// BEFORE any adapter exists, and the dead-port policy, teleop::evaluateDegradation, is applied
// there, once) is handed to the builder in `BuildOptions::absentDrivePorts`. Such a port is
// NEVER requested from the provider and becomes a `nullptr` slot in its side's MotorGroup — the
// group's ABSENT member (hal/motor_group.hpp), which it skips when commanding and excludes from
// every read. The builder applies NO policy of its own: re-running the policy over provider
// refusals was the rejected alternative, because it would report a dead port twice, silently
// tolerate a motor that ANSWERS the registry but rejects its configuration, and refuse every
// robot with fewer than three motors a side. So the builder refuses only what is structurally
// wrong with the list — a port that is not one of the description's drive ports, a list on a
// drivetrain that is not a tank, a side left with no present member — and everything the
// provider will not make still refuses the build, as before. With no absent ports (the
// default) every byte of the graph is what it was: Part 1's bit-identity cases are the pin.
// A port that dies AFTER boot is not the builder's either: the driver loop's protections
// (teleop/drive_protection.hpp, T4 Part 2) mark it ABSENT in its group mid-run
// (MotorGroup::markAbsent — out of every read, still commanded), and describe() names it
// from then on; nothing is ever rebuilt. Each group is labelled with its slots' port numbers,
// so those lines name the port.
//
// ── MECHANISMS: built from the config, ABSENT rather than refused (chunk T3) ────────
// Every `config::MechanismConfig` with motors becomes ONE hal::MotorMechanism, built in the
// constructor's BODY — after the whole drive graph, so the drive's construction order, its
// provider requests and its behaviour are exactly what they were (the bit-identity cases are
// still the pin, and one with mechanisms configured has joined them). Each motor is asked for
// with the MECHANISM's own signed port and cartridge; the mechanism gets its DECLARED safe brake
// mode and its config name (borrowed from this Robot's own copy of the config). validate() does
// NOT ask for mechanism motors: a motor the provider will not make leaves THAT mechanism ABSENT —
// mechanismAt() is nullptr, mechanismAbsence() carries the provider's own sentence naming the
// port — and the rest of the robot builds and drives, because an unplugged claw must not stop a
// robot from driving (the dead-port rule's spirit). A port a mechanism shares with the drive
// never gets this far: RobotConfig::valid() refuses the collision before anything is asked for.
// Since T3's addendum B2, a mechanism with 3-wire lines and no motors becomes ONE
// hal::PneumaticMechanism: each line asked for with the config's START level (making a line
// drives it — the one device write above), the mechanism's DECLARED safe level, its config name;
// pneumaticMechanism(name) finds it, and mechanisms() carries it beside the motor ones. A line the
// provider will not make leaves that mechanism ABSENT, like a motor. A mechanism with both motors
// and lines never gets here (valid() refuses it), and one with neither is reported as not built.
// The lookups by name and index are TYPED (a binding needs the concrete type), so a null from
// one of them does not mean "absent": builtMechanismAt(i) is the kind-neutral "is it built?", and
// mechanismAbsence(i) is empty exactly when it is non-null. A boot or panel line that reports
// each declared mechanism asks those two — never mechanismAt(), which is null for a built claw.
// On a field, a Robot built in initialize() is built while the brain is DISABLED, so whether a
// line's start level reaches the valve then, or only at the enable, is A4 register HA-157's too.
// The builder still never ticks and never commands a motor: a mechanism is first made safe by
// whatever drives it (a teleop::MechanismBinding at the driver-control loop's entry, an
// operation's exit), and a pneumatic one's first level after the start level is the driver's
// (a teleop::ToggleBinding) or an operation's.
//
// ── NO HARDWARE CLAIM ───────────────────────────────────────────────────────────────
// A Robot built from a config has driven the A2 host plant, through fakes, and nothing else.
// No config has ever been used on a physical robot. Since T4 Part 3 "shulib Drive" builds its
// graph with this class over the on-robot provider (src/programs/drive_program.cpp), as "shulib
// Teleop" did from G1 Part 2b until T4 Part 3 retired it; THIS version of the program has not run
// on hardware. Since G1 Part 2c the invented X-drive is built the same way
// (src/robots/xdrive_config.hpp, src/programs/xdrive_program.cpp) and no program wires a graph by hand; its ports
// are invented, so it refuses at boot on every robot that exists.
//
// PROS-free, sim-free, allocation-free in the control path (the graph is one in-place
// aggregate, sized by hal::kMaxSmartPorts — no unique_ptr anywhere, as with every other
// graph in this tree).

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <span>

#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/robot_context.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/health_monitor.hpp"
#include "shulib/hal/absent_gps.hpp"
#include "shulib/hal/absent_tag_source.hpp"
#include "shulib/hal/absent_vision.hpp"
#include "shulib/hal/device_provider.hpp"
#include "shulib/hal/drive_geometry.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/motor_group.hpp"
#include "shulib/kinematics/h_drive.hpp"
#include "shulib/kinematics/matrix_kinematics.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/drive_encoder_odometry.hpp"
#include "shulib/localization/gps_corrector.hpp"
#include "shulib/localization/i_corrector.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/odometry.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/localization/tracking_wheel.hpp"
#include "shulib/motion/motion.hpp"
#include "shulib/motion/motion_scheduler.hpp"

namespace shulib::robot {

/// The answer to "can this robot be built, on this target, right now?" — one bool and one
/// sentence. The sentence is what a person reads off the brain screen when the robot refuses to
/// boot, so it names the PORT or the FIELD and nothing else; 192 bytes is enough for the longest
/// message any check below can produce and small enough to sit on a boot stack.
struct BuildVerdict {
    /// True when every check passed and `Robot` may be constructed from the same pair.
    bool ok = false;
    /// The FIRST thing that was wrong, naming the port or the config field. Empty when `ok`.
    char why[192]{};
};

/// What the builder is told about THIS boot that the config cannot say: which tank drive ports the
/// boot check found dead, and (since COMP-a) a pacer to put between the chassis and the provider's.
/// Default-constructed, it changes nothing — every Part 1 caller's graph is byte-for-byte what it
/// was. The two fields have DIFFERENT lifetimes, and that is the thing to read before passing one:
/// `absentDrivePorts` is read during construction only; `pacer` is kept for the Robot's whole life.
struct BuildOptions {
    /// Tank drive ports the boot check reported MISSING, as port MAGNITUDES 1..21 — exactly the
    /// numbers `calibration::BootVerdict::isMissing()` answers for. Each becomes an ABSENT
    /// (`nullptr`) member of its side's MotorGroup and is never requested from the provider. A
    /// repeated entry is harmless (membership, not a count). BORROWED: read during `validate()`
    /// and during `Robot`'s construction only, never kept — the Robot recovers what it needs
    /// from its own groups.
    std::span<const int> absentDrivePorts = {};

    /// The pacer the Chassis is built with, or `nullptr` (the default) for `devices.pacer()` —
    /// exactly the graph every earlier caller got. A competition program passes its
    /// `sequence::RunGuard` here (constructed around `devices.pacer()` BEFORE the Robot), because
    /// the guard's deadline reaches a running motion only through the pacer seam
    /// (run_guard.hpp's banner) and a Chassis takes its pacer once, at construction.
    ///
    /// BORROWED FOR THE ROBOT'S WHOLE LIFE — unlike `absentDrivePorts`, this is NOT read during
    /// construction only: the Chassis keeps the reference (chassis.hpp's constructor), so the
    /// pacer must outlive the Robot. Declared after `absentDrivePorts` so the designated spelling
    /// reads in declaration order: `robot::BuildOptions{.absentDrivePorts = ..., .pacer = &guard}`.
    /// `validate()` does not read it (a pacer is not a device and cannot be refused).
    motion::ITickPacer* pacer = nullptr;

    /// True when physical port `port` is in `absentDrivePorts`.
    [[nodiscard]] constexpr bool isAbsent(int port) const noexcept {
        for (const int p : absentDrivePorts) {
            if (p == port) {
                return true;
            }
        }
        return false;
    }
};

/// Can `cfg` be built with `devices`? PURE, TOTAL, and the precondition `Robot`'s constructor
/// runs (ruling B3). Four passes, in this order:
///
///  1. **The config's own rules** — `config::RobotConfig::valid()`, whose message (including a
///     tank description's own refusal, verbatim) surfaces unchanged.
///  2. **The F5 cross-check** — the number of KINEMATIC WHEELS the drivetrain has must equal the
///     number of `hal::IMotor` handles the builder will put in the context. This is the same
///     check `motion::MotionDeps::validate()` makes at the bottom of the stack, and it is the
///     ONLY thing standing between a config typo and an unchecked `std::span::operator[]` on the
///     hot path — so it is made HERE too, before the deps exist, where it can name the config
///     field instead of saying "3 != 4". (It lives in exactly these two places and nowhere else;
///     `RobotConfig::valid()` deliberately does not duplicate it — see its documentation.)
///  3. **The absent ports** (`options.absentDrivePorts`, G1 Part 2a) — refused, naming the port,
///     when the list is given for a drivetrain that is not a tank (only a tank side is a group
///     that can carry a hole), when a port in it is not one of the description's drive ports,
///     or when it would leave a side with NO present member. Nothing else: the dead-port
///     POLICY was applied by the boot check that produced the list (header, ruling M1).
///  4. **Every device obtainable** — each drive port that is not absent, the IMU, the GPS if one
///     is configured, and each tracking-wheel encoder is requested from the provider. The first
///     `nullptr` becomes the verdict, carrying the provider's own `lastRefusal()` so the port
///     that is empty or dead is named by the layer that actually looked. An absent port is
///     never asked for, so a dead port is named once — by the boot check — and never twice.
///     MECHANISM motors are deliberately not asked for here (chunk T3): one the provider will
///     not make leaves that mechanism ABSENT when the Robot is built, and never refuses the robot.
///     Nor are a pneumatic mechanism's 3-wire lines (addendum B2): asking for one DRIVES it, and
///     the only refusal the on-robot provider can make — a line outside 1..8 — is pass 1's.
///     Their CONFIG is checked in pass 1 (cartridge, safe brake mode, safe and start levels,
///     ports and lines that collide with the drive or each other).
///
/// It is not `const` on the provider because obtaining a device may construct one; it is
/// idempotent, so calling it and then constructing a `Robot` asks for the same objects twice and
/// gets the same addresses both times.
[[nodiscard]] inline BuildVerdict validate(const config::RobotConfig& cfg,
                                           hal::IDeviceProvider& devices,
                                           const BuildOptions& options = {}) {
    BuildVerdict verdict;
    if (!cfg.valid(verdict.why, sizeof verdict.why)) {
        return verdict;
    }
    const int wheels = cfg.drivetrain.wheelCount();
    const int motors = cfg.drivetrain.driveMotorCount();
    if (wheels != motors) {
        std::snprintf(verdict.why, sizeof verdict.why,
                      "%s drivetrain has %d kinematic wheels but %d motor ports -- a wheel whose "
                      "port is 0 has nothing to command (check drivetrain.ports)",
                      config::drivetrainKindWord(cfg.drivetrain.kind), wheels, motors);
        return verdict;
    }
    if (!options.absentDrivePorts.empty()) {
        if (cfg.drivetrain.kind != config::DrivetrainKind::Tank) {
            std::snprintf(verdict.why, sizeof verdict.why,
                          "absent drive port %d was given for a %s drivetrain -- only a tank "
                          "side can run with an absent member",
                          options.absentDrivePorts.front(),
                          config::drivetrainKindWord(cfg.drivetrain.kind));
            return verdict;
        }
        const hal::DrivetrainDescription& d = cfg.drivetrain.tank.description;
        for (const int p : options.absentDrivePorts) {
            if (d.sideOf(p) == 0) {  // sideOf matches |stored port| == p, so -11 or 0 is no match
                std::snprintf(verdict.why, sizeof verdict.why,
                              "absent port %d is not one of this drivetrain's drive ports "
                              "(absent ports are port numbers 1..21 from the boot check)",
                              p);
                return verdict;
            }
        }
        const hal::DriveSideDescription* sides[2] = {&d.left, &d.right};
        const char* names[2] = {"LEFT", "RIGHT"};
        for (std::size_t s = 0; s < 2; ++s) {
            const hal::DriveSideDescription& side = *sides[s];
            std::size_t present = 0;
            char ports[72] = "";
            std::size_t used = 0;
            for (std::size_t i = 0; i < side.count; ++i) {
                const int port = hal::DrivetrainDescription::portMagnitude(side.signedPorts[i]);
                if (!options.isAbsent(port)) {
                    ++present;
                }
                char one[8] = "";
                std::snprintf(one, sizeof one, "%s%d", i == 0 ? "" : " ", port);
                for (std::size_t k = 0; one[k] != '\0' && used + 1 < sizeof ports; ++k) {
                    ports[used++] = one[k];
                }
                ports[used] = '\0';
            }
            if (present == 0) {
                std::snprintf(verdict.why, sizeof verdict.why,
                              "every %s drive port is absent (ports %s) -- a side needs at least "
                              "one motor that answers",
                              names[s], ports);
                return verdict;
            }
        }
    }
    const hal::MotorCartridge cartridge = cfg.drivetrain.cartridge();
    const auto askMotor = [&](std::int8_t signedPort, const char* role) noexcept -> bool {
        if (devices.motor(signedPort, cartridge) != nullptr) {
            return true;
        }
        std::snprintf(verdict.why, sizeof verdict.why, "%s cannot be made: %s", role,
                      devices.lastRefusal());
        return false;
    };
    switch (cfg.drivetrain.kind) {
        case config::DrivetrainKind::Tank: {
            const hal::DrivetrainDescription& d = cfg.drivetrain.tank.description;
            // An ABSENT port is skipped, never asked for: the boot check has already named it,
            // and asking would name it a second time (ruling M2).
            for (std::size_t i = 0; i < d.left.count; ++i) {
                if (options.isAbsent(hal::DrivetrainDescription::portMagnitude(
                        d.left.signedPorts[i]))) {
                    continue;
                }
                if (!askMotor(d.left.signedPorts[i], "a LEFT drive motor")) {
                    return verdict;
                }
            }
            for (std::size_t i = 0; i < d.right.count; ++i) {
                if (options.isAbsent(hal::DrivetrainDescription::portMagnitude(
                        d.right.signedPorts[i]))) {
                    continue;
                }
                if (!askMotor(d.right.signedPorts[i], "a RIGHT drive motor")) {
                    return verdict;
                }
            }
            break;
        }
        case config::DrivetrainKind::XDrive:
            for (const std::int8_t p : cfg.drivetrain.xdrive.ports) {
                if (!askMotor(p, "an X-drive wheel motor")) {
                    return verdict;
                }
            }
            break;
        case config::DrivetrainKind::HDrive:
            for (const std::int8_t p : cfg.drivetrain.hdrive.ports) {
                if (!askMotor(p, "an H-drive wheel motor")) {
                    return verdict;
                }
            }
            break;
    }
    if (devices.imu(cfg.drivetrain.imuPort()) == nullptr) {
        std::snprintf(verdict.why, sizeof verdict.why, "the IMU cannot be made: %s",
                      devices.lastRefusal());
        return verdict;
    }
    if (cfg.sensors.gps.present
        && devices.gps(cfg.sensors.gps.port, cfg.sensors.gps.leverArmForward,
                       cfg.sensors.gps.leverArmLeft)
               == nullptr) {
        std::snprintf(verdict.why, sizeof verdict.why, "the GPS cannot be made: %s",
                      devices.lastRefusal());
        return verdict;
    }
    if (cfg.odometry.kind == config::OdometryKind::TrackingWheels) {
        if (devices.rotation(cfg.odometry.wheels.forward.port) == nullptr) {
            std::snprintf(verdict.why, sizeof verdict.why,
                          "the forward tracking wheel cannot be made: %s", devices.lastRefusal());
            return verdict;
        }
        if (devices.rotation(cfg.odometry.wheels.lateral.port) == nullptr) {
            std::snprintf(verdict.why, sizeof verdict.why,
                          "the lateral tracking wheel cannot be made: %s", devices.lastRefusal());
            return verdict;
        }
    }
    verdict.ok = true;
    return verdict;
}

/// The §18.5 session header's PORT MAP for `cfg` — the text `diag::SessionInfo::portMap` carries
/// — generated from the config, so it cannot drift from the wiring (the "G1 generates" promise in
/// diag/session_info.hpp). `Robot::portMap()` is this over the Robot's own config; it is a free
/// function as well so the formatting can be tested on a description no valid Robot could carry
/// (a 21-port side).
///
/// The formats are the ones src/main.cpp printed by hand, so moving a program onto the builder
/// changed no session header (G1 Part 2b moved "shulib Teleop" — retired at T4 Part 3 — and G1
/// Part 2c the invented X-drive):
///
///  * **tank** — the retired `tankPortMapString()`'s: the description's source word, both sides' SIGNED
///    ports as stored (`%+d`, space-separated, each side capped at 39 characters), the IMU, then
///    — only when configured — `GPS<n>` and the two pods `ROTF<n> ROTL<n>`, then both sides'
///    inches per radian and the track width. For a drive-encoder tank without a GPS (robot two)
///    the text is byte-for-byte what `tankPortMapString()` printed (pinned against a transcription
///    of it in test/robot_builder_test.cpp, and against robot two's 2026-09-25 capture in
///    test/boot_source_test.cpp).
///  * **X-drive** — `portMapString()`'s: `FL BL BR FR` (the F5 wheel order), the two pods, the
///    GPS when present, the IMU.
///  * **H-drive** — the same shape: `L R S` (left line, right line, strafe wheel), pods, GPS, IMU.
///
/// Deliberately NOT marked for absent ports: this is what the robot is WIRED to be; a dead port
/// is `describe()`'s and the panel's to announce. Bounded BY HAND — assembled in a generous local
/// buffer and copied into `out` with an explicit bound, never a moving `snprintf` (the on-robot
/// build gate treats `-Wformat-truncation` as an error) — so it truncates rather than
/// overflowing and writes a NUL whenever `cap > 0`. Does not validate: it formats what is stored.
inline void writePortMap(const config::RobotConfig& cfg, char* out, std::size_t cap) noexcept {
    if (out == nullptr || cap == 0) {
        return;
    }
    // One side's signed ports, "%+d" space-separated, capped by hand — the retired
    // tankPortMapString()'s loop.
    const auto sidePorts = [](char* dst, std::size_t dstCap, const std::int8_t* ports,
                              std::size_t count) noexcept {
        std::size_t used = 0;
        for (std::size_t i = 0; i < count; ++i) {
            char one[8] = "";
            std::snprintf(one, sizeof one, "%s%+d", i != 0 ? " " : "", static_cast<int>(ports[i]));
            for (std::size_t k = 0; one[k] != '\0' && used + 1 < dstCap; ++k) {
                dst[used++] = one[k];
            }
            dst[used] = '\0';
        }
    };
    char gps[12] = "";
    if (cfg.sensors.gps.present) {
        std::snprintf(gps, sizeof gps, " GPS%u", static_cast<unsigned>(cfg.sensors.gps.port));
    }
    char pods[24] = "";
    if (cfg.odometry.kind == config::OdometryKind::TrackingWheels) {
        std::snprintf(pods, sizeof pods, " ROTF%d ROTL%d",
                      static_cast<int>(cfg.odometry.wheels.forward.port),
                      static_cast<int>(cfg.odometry.wheels.lateral.port));
    }
    char full[224] = "UNSET drivetrain";  // overwritten by every known kind below
    switch (cfg.drivetrain.kind) {
        case config::DrivetrainKind::Tank: {
            const hal::DrivetrainDescription& d = cfg.drivetrain.tank.description;
            constexpr std::size_t kSideCap = 40;  // tankPortMapString()'s, so 2b's switch was exact
            char l[kSideCap] = "";
            char r[kSideCap] = "";
            sidePorts(l, kSideCap, d.left.signedPorts, d.left.count);
            sidePorts(r, kSideCap, d.right.signedPorts, d.right.count);
            std::snprintf(full, sizeof full, "%s L %s R %s IMU%u%s%s ipr %.4f/%.4f TW%.2f",
                          d.source == hal::DescriptionSource::RobotMeasured ? "MEASURED" : "TYPED",
                          l, r, static_cast<unsigned>(d.imuPort), gps, pods,
                          d.left.inchesPerRadian.value(), d.right.inchesPerRadian.value(),
                          d.trackWidth.value());
            break;
        }
        case config::DrivetrainKind::XDrive: {
            const config::XDriveDrivetrain& x = cfg.drivetrain.xdrive;
            std::snprintf(full, sizeof full, "FL%d BL%d BR%d FR%d%s%s IMU%u",
                          static_cast<int>(x.ports[0]), static_cast<int>(x.ports[1]),
                          static_cast<int>(x.ports[2]), static_cast<int>(x.ports[3]), pods, gps,
                          static_cast<unsigned>(x.imuPort));
            break;
        }
        case config::DrivetrainKind::HDrive: {
            const config::HDriveDrivetrain& hcfg = cfg.drivetrain.hdrive;
            std::snprintf(full, sizeof full, "L%d R%d S%d%s%s IMU%u",
                          static_cast<int>(hcfg.ports[0]), static_cast<int>(hcfg.ports[1]),
                          static_cast<int>(hcfg.ports[2]), pods, gps,
                          static_cast<unsigned>(hcfg.imuPort));
            break;
        }
    }
    // The ONE bound: copy by hand, stop one short of `cap`, always terminate.
    std::size_t n = 0;
    for (; full[n] != '\0' && n + 1 < cap; ++n) {
        out[n] = full[n];
    }
    out[n] = '\0';
}

/// A whole robot: the PROS-free object graph a `config::RobotConfig` describes, owned, with its
/// devices borrowed from an `hal::IDeviceProvider`.
///
/// **The provider must outlive the Robot**, and so must everything the provider borrows (on the
/// host: the harness whose clock, battery, sink and pacer it was given). The Robot owns the
/// graph; it owns no hardware, by ruling B2.
///
/// Neither copyable nor movable, for the same structural reason `chassis::Chassis` is not: the
/// graph is a web of references into this object's own storage, so a copy or a move would leave
/// half of it pointing at the wrong place. Construct it where it will live.
class Robot {
public:
    /// Build the whole graph. PRECONDITION: `validate(cfg, devices, options)` passes — and this
    /// runs it, with the SAME options, so a config that cannot be built throws
    /// `PreconditionError` carrying the verdict's own sentence, before any member exists. `cfg`
    /// is COPIED (a boot sequence that decodes a record into a local and hands out a reference to
    /// it is the dangling-reference bug this copy makes impossible); `devices` is borrowed and
    /// must outlive the Robot; `options.absentDrivePorts` is read during construction only (its
    /// absent ports become `nullptr` group members and are never requested — G1 Part 2a, ruling
    /// M1), so that span may die as soon as the constructor returns — but `options.pacer`, when
    /// set, is KEPT by the Chassis and must outlive the Robot (COMP-a; `BuildOptions::pacer`).
    /// Default options build exactly the graph the two-argument form always built. LAST, after the
    /// whole drive graph, each configured mechanism with motors, or with 3-wire lines, is built —
    /// or recorded ABSENT with the provider's sentence (chunk T3 and its addendum B2; the file
    /// header's MECHANISMS section).
    Robot(const config::RobotConfig& cfg, hal::IDeviceProvider& devices,
          const BuildOptions& options = {})
        : cfg_{prechecked(cfg, devices, options)},
          tankKin_{makeTankKinematics(cfg_)},
          matrixKin_{makeMatrixKinematics(cfg_)},
          kin_{tankKin_.has_value() ? static_cast<const kinematics::IKinematics*>(&*tankKin_)
                                    : static_cast<const kinematics::IKinematics*>(&*matrixKin_)},
          wiring_{wireMotors(cfg_, devices, options)},
          left_{makeSideGroup(cfg_, wiring_.leftMembers, wiring_.leftLabels, wiring_.leftCount)},
          right_{makeSideGroup(cfg_, wiring_.rightMembers, wiring_.rightLabels,
                               wiring_.rightCount)},
          driveMotors_{makeDriveMotors(wiring_, left_, right_)},
          driveMotorCount_{static_cast<std::size_t>(cfg_.drivetrain.driveMotorCount())},
          groups_{makeGroupArray(left_, right_)},
          groupCount_{left_.has_value() ? std::size_t{2} : std::size_t{0}},
          imu_{devices.imu(cfg_.drivetrain.imuPort())},
          absentGps_{},
          gps_{cfg_.sensors.gps.present ? devices.gps(cfg_.sensors.gps.port,
                                                      cfg_.sensors.gps.leverArmForward,
                                                      cfg_.sensors.gps.leverArmLeft)
                                        : static_cast<hal::IGps*>(&absentGps_)},
          tags_{},
          vision_{},
          faults_{devices.telemetry(), devices.clock()},
          health_{faults_},
          leftGeometry_{sideGeometry(cfg_, /*right=*/false)},
          rightGeometry_{sideGeometry(cfg_, /*right=*/true)},
          driveOdom_{makeDriveEncoderOdometry(cfg_, left_, right_, *imu_, leftGeometry_,
                                              rightGeometry_)},
          pilonsOdom_{makePilonsOdometry(cfg_, devices, *imu_)},
          odom_{driveOdom_.has_value()
                    ? static_cast<localization::IOdometry*>(&*driveOdom_)
                    : static_cast<localization::IOdometry*>(&*pilonsOdom_)},
          fusion_{},
          gpsCorrector_{makeGpsCorrector(cfg_, devices, *gps_, *imu_)},
          correctors_{gpsCorrector_.has_value()
                          ? static_cast<localization::ICorrector*>(&*gpsCorrector_)
                          : nullptr},
          correctorCount_{gpsCorrector_.has_value() ? std::size_t{1} : std::size_t{0}},
          localizer_{devices.clock(), *imu_, *odom_, fusion_,
                     std::span<localization::ICorrector* const>{correctors_.data(),
                                                                correctorCount_}},
          ctx_{chassis::RobotContextConfig{
              .clock = &devices.clock(),
              .driveMotors = std::span<hal::IMotor* const>{driveMotors_.data(), driveMotorCount_},
              .imu = imu_,
              .gps = gps_,
              .battery = &devices.battery(),
              .telemetry = &devices.telemetry(),
              .tags = &tags_,
              .vision = &vision_}},
          deps_{.ctx = &ctx_,
                .localizer = &localizer_,
                .kinematics = kin_,
                .faults = &faults_,
                .health = &health_,
                .motorGroups =
                    std::span<hal::MotorGroup* const>{groups_.data(), groupCount_}},
          chassis_{deps_, options.pacer != nullptr ? *options.pacer : devices.pacer(),
                   effectiveTuning(cfg_)} {
        buildMechanisms(devices);
    }

    /// Non-copyable: the graph is a web of references into this object's own storage.
    Robot(const Robot&) = delete;
    /// Non-movable, same reasoning — and `chassis::Chassis` is non-movable in any case.
    Robot(Robot&&) = delete;
    /// Non-copy-assignable, same reasoning.
    Robot& operator=(const Robot&) = delete;
    /// Non-move-assignable, same reasoning.
    Robot& operator=(Robot&&) = delete;
    /// Destroys the graph. The provider and everything it borrows must still be alive.
    ~Robot() = default;

    /// The public facade every auton is written against — the whole point of the class.
    [[nodiscard]] chassis::Chassis& chassis() noexcept { return chassis_; }
    /// The facade, read-only.
    [[nodiscard]] const chassis::Chassis& chassis() const noexcept { return chassis_; }

    /// The fused estimator, for a caller that wants the quality class or a corrector's record
    /// rather than just the pose.
    [[nodiscard]] localization::Localizer& localizer() noexcept { return localizer_; }
    /// The estimator, read-only.
    [[nodiscard]] const localization::Localizer& localizer() const noexcept { return localizer_; }

    /// The run-scoped fault latch — what the run summary and the boot banner read.
    [[nodiscard]] diag::FaultLatch& faults() noexcept { return faults_; }
    /// The fault latch, read-only.
    [[nodiscard]] const diag::FaultLatch& faults() const noexcept { return faults_; }

    /// The health monitor (the A3 pathology → fault policy).
    [[nodiscard]] diag::HealthMonitor& health() noexcept { return health_; }
    /// The health monitor, read-only.
    [[nodiscard]] const diag::HealthMonitor& health() const noexcept { return health_; }

    /// The one object every layer reads hardware through — the Tier-3 seam for code that wants a
    /// device this facade does not expose.
    [[nodiscard]] chassis::RobotContext& context() noexcept { return ctx_; }
    /// The context, read-only.
    [[nodiscard]] const chassis::RobotContext& context() const noexcept { return ctx_; }

    /// The config this robot was built from, as stored.
    [[nodiscard]] const config::RobotConfig& config() const noexcept { return cfg_; }

    /// Whether this robot has a GPS corrector installed — the observable half of "absent is
    /// explicit": false means `hal::AbsentGps` is in the context and the Localizer's corrector
    /// span is EMPTY, not that a corrector is present and quiet.
    [[nodiscard]] bool hasGpsCorrector() const noexcept { return gpsCorrector_.has_value(); }

    /// How many `hal::MotorGroup`s this robot has — 2 on a tank (one per side), 0 on a holonomic
    /// drivetrain, whose wheels are single motors.
    [[nodiscard]] std::size_t motorGroupCount() const noexcept { return groupCount_; }

    /// The drive-encoder odometry this robot runs, for a panel that wants its cross-check (the
    /// per-side distances beside the fused estimate) — or `nullptr` when this robot runs
    /// tracking wheels instead. Read-only: the Localizer owns the updating.
    [[nodiscard]] const localization::DriveEncoderOdometry* driveOdometry() const noexcept {
        return driveOdom_.has_value() ? &*driveOdom_ : nullptr;
    }

    // ── the mechanisms (chunk T3; the file header's MECHANISMS section) ───────────────

    /// How many mechanisms the config declares, built or not, of both kinds —
    /// `config().mechanismCount`. `builtMechanismAt()`, `mechanismAt()`, `pneumaticMechanismAt()`
    /// and `mechanismAbsence()` take an index below this.
    [[nodiscard]] std::size_t mechanismCount() const noexcept { return cfg_.mechanismCount; }

    /// The `i`-th declared mechanism if it was BUILT, of EITHER kind — nullptr when it is ABSENT,
    /// has nothing to build, or `i` is not below `mechanismCount()`. This is THE "is this one
    /// built?" question: a boot line or a panel row that reports each declared mechanism asks
    /// this (and prints its `name()`), never `mechanismAt()`, which is nullptr for a built
    /// PNEUMATIC mechanism too and would report a working claw as absent.
    /// `mechanismAbsence(i)` is empty exactly when this is non-null.
    [[nodiscard]] hal::IMechanism* builtMechanismAt(std::size_t i) noexcept {
        MechanismSlot* slot = declaredSlot(*this, i);
        if (slot == nullptr) {
            return nullptr;
        }
        if (slot->built.has_value()) {
            return &*slot->built;
        }
        return slot->pneumatic.has_value() ? &*slot->pneumatic : nullptr;
    }
    /// The `i`-th declared mechanism if it was built, read-only (same rules).
    [[nodiscard]] const hal::IMechanism* builtMechanismAt(std::size_t i) const noexcept {
        const MechanismSlot* slot = declaredSlot(*this, i);
        if (slot == nullptr) {
            return nullptr;
        }
        if (slot->built.has_value()) {
            return &*slot->built;
        }
        return slot->pneumatic.has_value() ? &*slot->pneumatic : nullptr;
    }

    /// The `i`-th declared mechanism as a BUILT MOTOR mechanism — or nullptr when it is ABSENT
    /// (the provider refused one of its motors), is a pneumatic mechanism (see
    /// `pneumaticMechanismAt()`), has nothing to build, or `i` is not below `mechanismCount()`.
    /// A TYPED lookup, for code that needs the motor type (a `teleop::MechanismBinding`): null
    /// here does NOT mean "absent" — `builtMechanismAt()` answers that.
    [[nodiscard]] hal::MotorMechanism* mechanismAt(std::size_t i) noexcept {
        MechanismSlot* slot = declaredSlot(*this, i);
        return slot != nullptr && slot->built.has_value() ? &*slot->built : nullptr;
    }
    /// The `i`-th declared mechanism, read-only (same rules).
    [[nodiscard]] const hal::MotorMechanism* mechanismAt(std::size_t i) const noexcept {
        const MechanismSlot* slot = declaredSlot(*this, i);
        return slot != nullptr && slot->built.has_value() ? &*slot->built : nullptr;
    }

    /// The `i`-th declared mechanism as a BUILT PNEUMATIC mechanism (addendum B2) — or nullptr
    /// when it is ABSENT (the provider refused one of its lines), is a motor mechanism (see
    /// `mechanismAt()`), has nothing to build, or `i` is not below `mechanismCount()`. Typed,
    /// like `mechanismAt()`: null here does not mean "absent" either.
    [[nodiscard]] hal::PneumaticMechanism* pneumaticMechanismAt(std::size_t i) noexcept {
        MechanismSlot* slot = declaredSlot(*this, i);
        return slot != nullptr && slot->pneumatic.has_value() ? &*slot->pneumatic : nullptr;
    }
    /// The `i`-th declared pneumatic mechanism, read-only (same rules).
    [[nodiscard]] const hal::PneumaticMechanism* pneumaticMechanismAt(std::size_t i) const noexcept {
        const MechanismSlot* slot = declaredSlot(*this, i);
        return slot != nullptr && slot->pneumatic.has_value() ? &*slot->pneumatic : nullptr;
    }

    /// Why the `i`-th declared mechanism was NOT built: the provider's own sentence (naming the
    /// port or the 3-wire line) for an ABSENT one, a fixed sentence for one with nothing to
    /// build, and "no mechanism is declared at this index" for an `i` not below
    /// `mechanismCount()`. The empty string EXACTLY when `builtMechanismAt(i)` is non-null — a
    /// built mechanism of either kind, and nothing else. Stable for the Robot's lifetime.
    [[nodiscard]] const char* mechanismAbsence(std::size_t i) const noexcept {
        const MechanismSlot* slot = declaredSlot(*this, i);
        return slot != nullptr ? slot->absence : "no mechanism is declared at this index";
    }

    /// The BUILT MOTOR mechanism whose config name is `name` — nullptr when no declared mechanism
    /// has that name (or `name` is null), when that one was not built, or when it is a pneumatic
    /// mechanism (`pneumaticMechanism()` finds those: the lookup is typed because a binding needs
    /// the concrete type, and the library does not downcast). Names are unique: the config's
    /// `valid()` refuses two mechanisms with one name.
    [[nodiscard]] hal::MotorMechanism* mechanism(const char* name) noexcept {
        return mechanismAt(mechanismIndex(name));
    }
    /// The named mechanism, read-only (same rules).
    [[nodiscard]] const hal::MotorMechanism* mechanism(const char* name) const noexcept {
        return mechanismAt(mechanismIndex(name));
    }

    /// The BUILT PNEUMATIC mechanism whose config name is `name` (addendum B2) — what a
    /// `teleop::ToggleBinding` is bound to. nullptr when no declared mechanism has that name (or
    /// `name` is null), when that one was not built, or when it is a motor mechanism.
    [[nodiscard]] hal::PneumaticMechanism* pneumaticMechanism(const char* name) noexcept {
        return pneumaticMechanismAt(mechanismIndex(name));
    }
    /// The named pneumatic mechanism, read-only (same rules).
    [[nodiscard]] const hal::PneumaticMechanism* pneumaticMechanism(const char* name) const noexcept {
        return pneumaticMechanismAt(mechanismIndex(name));
    }

    /// Every BUILT mechanism of BOTH kinds, in config order, as the `hal::IMechanism*` span an
    /// end-of-run guard walks (`sequence::RunGuardConfig::mechanisms`) — an ABSENT one is not in
    /// it, because there is nothing there to make safe. Empty on a robot with none. The storage is
    /// this Robot's, so the span is valid for the Robot's lifetime.
    [[nodiscard]] std::span<hal::IMechanism* const> mechanisms() noexcept {
        return std::span<hal::IMechanism* const>{builtMechanisms_.data(), builtMechanismCount_};
    }

    /// The §18.5 session header's port map for this robot, generated from the config it was
    /// built with — `writePortMap(config(), out, cap)`, whose documentation gives the formats.
    /// Truncates rather than overflowing (the header field is `diag::kMaxPortMapBytes`, 96
    /// bytes); writes a NUL whenever cap > 0. Use this, never `describe()`, for the header:
    /// the banner is longer than the field.
    void portMap(char* out, std::size_t cap) const noexcept { writePortMap(cfg_, out, cap); }

    /// The BANNER LINE: identity, the drivetrain kind and where its numbers came from, both
    /// sides' ports AS STORED (signs included — the one place a sign lives), any drive port
    /// ABSENT (`DEGRADED: absent 3,13` — only when there is one: built absent at boot, or marked
    /// absent mid-run by the driver loop's protections, T4 Part 2), the geometry that
    /// governs, the estimator, and each optional device as present or ABSENT. Truncates rather
    /// than overflowing; writes at least a NUL whenever cap > 0.
    ///
    /// It is built from the CONFIG this robot was constructed with, not from a second copy of the
    /// numbers, so it cannot drift from the wiring — the same property src/main.cpp's retired
    /// `portMapString()` had and for the same reason. The absent ports are read back from the
    /// two groups' own slots, which were built from the same description in the same order.
    void describe(char* buf, std::size_t cap) const {
        if (buf == nullptr || cap == 0) {
            return;
        }
        constexpr std::size_t kSideCap = 72;
        char left[kSideCap] = "";
        char right[kSideCap] = "";
        const char* name = cfg_.identity.name[0] != '\0' ? cfg_.identity.name : "(unnamed)";
        switch (cfg_.drivetrain.kind) {
            case config::DrivetrainKind::Tank: {
                const hal::DrivetrainDescription& d = cfg_.drivetrain.tank.description;
                appendPorts(left, kSideCap, d.left.signedPorts, d.left.count);
                appendPorts(right, kSideCap, d.right.signedPorts, d.right.count);
                char absent[kSideCap] = "";
                appendAbsent(absent, kSideCap);
                std::snprintf(buf, cap,
                              "%s [%s] tank %s | L %s R %s%s%s | ipr L%.5f R%.5f W %.3f in | "
                              "IMU%u | %s | GPS %s tags ABSENT vision ABSENT",
                              name, cfg_.identity.team,
                              d.source == hal::DescriptionSource::RobotMeasured ? "MEASURED"
                                                                                : "TYPED",
                              left, right, absent[0] != '\0' ? " | DEGRADED: absent " : "",
                              absent, leftGeometry_.inchesPerRadian().value(),
                              rightGeometry_.inchesPerRadian().value(), d.trackWidth.value(),
                              static_cast<unsigned>(d.imuPort),
                              config::odometryKindWord(cfg_.odometry.kind),
                              gpsWord());
                return;
            }
            case config::DrivetrainKind::XDrive: {
                const config::XDriveDrivetrain& x = cfg_.drivetrain.xdrive;
                appendPorts(left, kSideCap, x.ports, 4);
                std::snprintf(buf, cap,
                              "%s [%s] x-drive TYPED | wheels %s | R %.3f in | IMU%u | %s | "
                              "GPS %s tags ABSENT vision ABSENT",
                              name, cfg_.identity.team, left, x.driveRadius.value(),
                              static_cast<unsigned>(x.imuPort),
                              config::odometryKindWord(cfg_.odometry.kind), gpsWord());
                return;
            }
            case config::DrivetrainKind::HDrive: {
                const config::HDriveDrivetrain& hcfg = cfg_.drivetrain.hdrive;
                appendPorts(left, kSideCap, hcfg.ports, 3);
                std::snprintf(buf, cap,
                              "%s [%s] h-drive TYPED | wheels %s | W %.3f in strafe@%.3f in | "
                              "IMU%u | %s | GPS %s tags ABSENT vision ABSENT",
                              name, cfg_.identity.team, left, hcfg.trackWidth.value(),
                              hcfg.strafeWheelOffset.value(), static_cast<unsigned>(hcfg.imuPort),
                              config::odometryKindWord(cfg_.odometry.kind), gpsWord());
                return;
            }
        }
        std::snprintf(buf, cap, "%s [%s] UNSET drivetrain", name, cfg_.identity.team);
    }

private:
    /// The borrowed `hal::IMotor` handles, gathered from the provider BY PORT before any group
    /// exists. One array per tank side (a side is N physical motors behind one group); one flat
    /// array for a holonomic drivetrain, whose wheels are single motors in the F5 wheel order.
    struct MotorWiring {
        /// The LEFT side's members, in the description's stored order. Tank only.
        hal::IMotor* leftMembers[hal::kMaxSmartPorts]{};
        /// The RIGHT side's members. Tank only.
        hal::IMotor* rightMembers[hal::kMaxSmartPorts]{};
        /// How many entries of `leftMembers` are real.
        std::size_t leftCount = 0;
        /// How many entries of `rightMembers` are real.
        std::size_t rightCount = 0;
        /// Each LEFT slot's port NUMBER (the signed port's magnitude), in the same order — the
        /// labels the side's MotorGroup prints a member by (T4 Part 2). Tank only.
        int leftLabels[hal::kMaxSmartPorts]{};
        /// Each RIGHT slot's port number, the same way. Tank only.
        int rightLabels[hal::kMaxSmartPorts]{};
        /// The holonomic wheels' motors, in the F5 wheel order. Holonomic only.
        hal::IMotor* wheels[static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels)]{};
        /// How many entries of `wheels` are real.
        std::size_t wheelCount = 0;
    };

    /// Run the verdict — WITH the constructor's options, so a port the boot check reported
    /// absent is not asked for here either — and throw its sentence if it fails, then hand back
    /// `cfg` for the member initializer. Being the FIRST member's initializer is what makes the
    /// refusal happen before any object exists (ruling B3). (Dropping the options here was the
    /// audit's defect 1: a robot a tolerant validate() had passed threw inside its own
    /// constructor. Test 2 constructs one to keep that from coming back.)
    [[nodiscard]] static const config::RobotConfig& prechecked(const config::RobotConfig& cfg,
                                                               hal::IDeviceProvider& devices,
                                                               const BuildOptions& options) {
        const BuildVerdict verdict = validate(cfg, devices, options);
        SHULIB_PRECONDITION(verdict.ok, verdict.why);
        return cfg;
    }

    /// The tank kinematics, or nothing on a holonomic drivetrain. The track width comes from the
    /// description as ONE number and is never retyped — MEASURED from a powered spin (effectively
    /// larger than a tape's, HA-135) or typed, and either way it arrives here already decided.
    [[nodiscard]] static std::optional<kinematics::TankKinematics> makeTankKinematics(
        const config::RobotConfig& cfg) {
        if (cfg.drivetrain.kind != config::DrivetrainKind::Tank) {
            return std::nullopt;
        }
        return std::optional<kinematics::TankKinematics>{
            std::in_place, cfg.drivetrain.tank.description.trackWidth};
    }

    /// The holonomic kinematics (x_drive.hpp's or h_drive.hpp's coefficient table), or nothing on
    /// a tank. Value-constructed from config DATA, exactly as chassis.hpp's standalone recipe
    /// does it by hand.
    [[nodiscard]] static std::optional<kinematics::MatrixKinematics> makeMatrixKinematics(
        const config::RobotConfig& cfg) {
        switch (cfg.drivetrain.kind) {
            case config::DrivetrainKind::XDrive:
                return std::optional<kinematics::MatrixKinematics>{
                    kinematics::xDrive(cfg.drivetrain.xdrive.driveRadius)};
            case config::DrivetrainKind::HDrive:
                return std::optional<kinematics::MatrixKinematics>{kinematics::hDrive(
                    {.trackWidth = cfg.drivetrain.hdrive.trackWidth,
                     .strafeWheelOffset = cfg.drivetrain.hdrive.strafeWheelOffset})};
            case config::DrivetrainKind::Tank:
                break;
        }
        return std::nullopt;
    }

    /// Ask the provider for every drive motor the config names, handing each signed port over AS
    /// STORED so the sign is applied exactly once, inside the adapter. A null here would be a
    /// provider that refused after `validate()` said it would not — a contract breach, and a loud
    /// one, because the alternative is an UNPLANNED nullptr inside a MotorGroup.
    ///
    /// The one PLANNED nullptr: a tank port in `options.absentDrivePorts` is not asked for at all
    /// and its slot is left empty — the group's ABSENT member, in the description's own order so
    /// every index a panel shows still means the same port (src/main.cpp's hand-wired `TankMotors`
    /// kept its slots for the same reason, until G1 Part 2b retired it). `validate()` has already refused a list that is not a tank's,
    /// names a port that is not a drive port, or empties a side.
    [[nodiscard]] static MotorWiring wireMotors(const config::RobotConfig& cfg,
                                                hal::IDeviceProvider& devices,
                                                const BuildOptions& options) {
        MotorWiring w;
        const hal::MotorCartridge cartridge = cfg.drivetrain.cartridge();
        const auto take = [&](std::int8_t signedPort) -> hal::IMotor* {
            hal::IMotor* m = devices.motor(signedPort, cartridge);
            SHULIB_PRECONDITION(m != nullptr,
                                "robot::Robot: the provider refused a drive motor that "
                                "validate() had already obtained");
            return m;
        };
        const auto takeUnlessAbsent = [&](std::int8_t signedPort) -> hal::IMotor* {
            return options.isAbsent(hal::DrivetrainDescription::portMagnitude(signedPort))
                       ? nullptr
                       : take(signedPort);
        };
        switch (cfg.drivetrain.kind) {
            case config::DrivetrainKind::Tank: {
                const hal::DrivetrainDescription& d = cfg.drivetrain.tank.description;
                for (std::size_t i = 0; i < d.left.count; ++i) {
                    w.leftLabels[w.leftCount] =
                        hal::DrivetrainDescription::portMagnitude(d.left.signedPorts[i]);
                    w.leftMembers[w.leftCount++] = takeUnlessAbsent(d.left.signedPorts[i]);
                }
                for (std::size_t i = 0; i < d.right.count; ++i) {
                    w.rightLabels[w.rightCount] =
                        hal::DrivetrainDescription::portMagnitude(d.right.signedPorts[i]);
                    w.rightMembers[w.rightCount++] = takeUnlessAbsent(d.right.signedPorts[i]);
                }
                break;
            }
            case config::DrivetrainKind::XDrive:
                for (const std::int8_t p : cfg.drivetrain.xdrive.ports) {
                    w.wheels[w.wheelCount++] = take(p);
                }
                break;
            case config::DrivetrainKind::HDrive:
                for (const std::int8_t p : cfg.drivetrain.hdrive.ports) {
                    w.wheels[w.wheelCount++] = take(p);
                }
                break;
        }
        return w;
    }

    /// ONE `hal::MotorGroup` over a tank side's members — the object that IS the motor for that
    /// kinematic wheel (R3b Part 1: N physical motors on one side go behind one group, or the
    /// surplus is silently uncommanded) — labelled with each slot's PORT NUMBER, so the driver
    /// loop's protections name a port a pit crew can find (T4 Part 2; the labels live in
    /// `wiring_`, which outlives the group). Nothing on a holonomic drivetrain, whose wheels are
    /// single motors.
    [[nodiscard]] static std::optional<hal::MotorGroup> makeSideGroup(
        const config::RobotConfig& cfg, hal::IMotor* const* members, const int* labels,
        std::size_t count) {
        if (cfg.drivetrain.kind != config::DrivetrainKind::Tank) {
            return std::nullopt;
        }
        return std::optional<hal::MotorGroup>{
            std::in_place, std::span<hal::IMotor* const>{members, count},
            teleop::SideMonitorConfig{}, std::span<const int>{labels, count}};
    }

    /// The context's drive-motor array, in KINEMATIC WHEEL ORDER: the two groups on a tank
    /// (0 = left, 1 = right, as `kinematics::TankKinematics` numbers them), or the holonomic
    /// wheels straight through in the F5 order.
    [[nodiscard]] static std::array<hal::IMotor*,
                                    static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels)>
    makeDriveMotors(const MotorWiring& wiring, std::optional<hal::MotorGroup>& left,
                    std::optional<hal::MotorGroup>& right) {
        std::array<hal::IMotor*, static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels)>
            out{};
        if (left.has_value()) {
            out[0] = &*left;
            out[1] = &*right;
            return out;
        }
        for (std::size_t i = 0; i < wiring.wheelCount && i < out.size(); ++i) {
            out[i] = wiring.wheels[i];
        }
        return out;
    }

    /// The groups the health tick evaluates each loop (`motion::MotionDeps::motorGroups`), or an
    /// array of nulls with count 0 on a holonomic drivetrain.
    [[nodiscard]] static std::array<hal::MotorGroup*, 2> makeGroupArray(
        std::optional<hal::MotorGroup>& left, std::optional<hal::MotorGroup>& right) {
        if (!left.has_value()) {
            return {nullptr, nullptr};
        }
        return {&*left, &*right};
    }

    /// One side's drive geometry, straight from the description — a separate object per side,
    /// because a calibration measures each side separately and an asymmetric drivetrain is
    /// representable. A holonomic drivetrain has no per-side wheel scale in its config (its
    /// odometry is the tracking wheels), so it gets the library default, which nothing reads.
    [[nodiscard]] static hal::DriveGeometry sideGeometry(const config::RobotConfig& cfg,
                                                          bool right) {
        if (cfg.drivetrain.kind != config::DrivetrainKind::Tank) {
            return hal::DriveGeometry{};
        }
        const hal::DrivetrainDescription& d = cfg.drivetrain.tank.description;
        return right ? d.rightGeometry() : d.leftGeometry();
    }

    /// Drive-encoder odometry over the two side GROUPS (R3b Part 2) — or nothing when this robot
    /// runs tracking wheels. The groups are the IMotors it reads, so a frozen member is already
    /// excluded by the group's median before the estimator ever sees a number.
    [[nodiscard]] static std::optional<localization::DriveEncoderOdometry>
    makeDriveEncoderOdometry(const config::RobotConfig& cfg, std::optional<hal::MotorGroup>& left,
                             std::optional<hal::MotorGroup>& right, hal::IImu& imu,
                             hal::DriveGeometry leftGeometry, hal::DriveGeometry rightGeometry) {
        if (cfg.odometry.kind != config::OdometryKind::DriveEncoders) {
            return std::nullopt;
        }
        return std::optional<localization::DriveEncoderOdometry>{
            std::in_place,  imu,          *left,
            *right,         leftGeometry, rightGeometry,
            cfg.drivetrain.tank.description.trackWidth};
    }

    /// Two-pod (Pilons) odometry — or nothing when this robot runs drive encoders. The ROLE of
    /// each wheel is fixed by which factory builds it, so a pair that would have been handed over
    /// in the wrong order cannot be expressed here at all.
    [[nodiscard]] static std::optional<localization::PilonsOdometry> makePilonsOdometry(
        const config::RobotConfig& cfg, hal::IDeviceProvider& devices, hal::IImu& imu) {
        if (cfg.odometry.kind != config::OdometryKind::TrackingWheels) {
            return std::nullopt;
        }
        const config::TrackingWheelConfig& f = cfg.odometry.wheels.forward;
        const config::TrackingWheelConfig& l = cfg.odometry.wheels.lateral;
        hal::IRotation* forwardSensor = devices.rotation(f.port);
        hal::IRotation* lateralSensor = devices.rotation(l.port);
        SHULIB_PRECONDITION(forwardSensor != nullptr && lateralSensor != nullptr,
                            "robot::Robot: the provider refused a tracking-wheel encoder that "
                            "validate() had already obtained");
        return std::optional<localization::PilonsOdometry>{
            std::in_place, imu,
            localization::TrackingWheel::forward(*forwardSensor, f.wheelDiameter, f.offset),
            localization::TrackingWheel::lateral(*lateralSensor, l.wheelDiameter, l.offset)};
    }

    /// The GPS corrector — installed if and only if the config says a GPS is present. When it is
    /// absent there is NO corrector and the Localizer's corrector span is empty, which is a
    /// different thing from a corrector that never proposes: a quiet corrector still shows in the
    /// records as a live source, and a robot that has no GPS must not look like one that has a
    /// silent one (landmine 5).
    [[nodiscard]] static std::optional<localization::GpsCorrector> makeGpsCorrector(
        const config::RobotConfig& cfg, hal::IDeviceProvider& devices, hal::IGps& gps,
        hal::IImu& imu) {
        if (!cfg.sensors.gps.present) {
            return std::nullopt;
        }
        return std::optional<localization::GpsCorrector>{std::in_place, devices.clock(), gps, imu};
    }

    /// The ChassisConfig the facade is built with: the caller's tuning, plus the one thing the
    /// builder knows and a person should not have to retype — the stall check's per-wheel drive
    /// geometry, and the fact that a drivetrain whose odometry IS its drive encoders has no
    /// independent motion source to cross-check against (odo_stall_check.hpp's §2 ruling).
    /// Exactly what `src/main.cpp`'s `tankChassisConfig()` did by hand until G1 Part 2b, and
    /// applied only where that function applied it: a holonomic drivetrain's config carries no
    /// per-wheel scale, so its tuning is passed through untouched — which is also what main.cpp's
    /// hand-wired X-drive graph did until G1 Part 2c built it here.
    [[nodiscard]] static config::TuningConfig effectiveTuning(const config::RobotConfig& cfg) {
        config::TuningConfig tuning = cfg.tuning;
        if (cfg.drivetrain.kind == config::DrivetrainKind::Tank) {
            tuning.motion.stall.setAllWheels(cfg.drivetrain.tank.description.leftGeometry());
            tuning.motion.stall.independentMotionSource =
                cfg.odometry.kind != config::OdometryKind::DriveEncoders;
        }
        return tuning;
    }

    /// "GPS present" or "ABSENT", for the banner.
    [[nodiscard]] const char* gpsWord() const noexcept {
        return cfg_.sensors.gps.present ? "present" : "ABSENT";
    }

    /// Write `count` signed ports into `out` as "1,-2,3", bounded by `cap` and always
    /// NUL-terminated. Appended BY HAND rather than with a moving snprintf for the same reason
    /// src/main.cpp's `tankPortMapString()` did it: the compiler cannot bound a
    /// `cap - strlen(dst)` argument, so -Wformat-truncation fires on a call that is in fact safe,
    /// and the on-robot build gate treats that warning as an error.
    static void appendPorts(char* out, std::size_t cap, const std::int8_t* ports,
                            std::size_t count) {
        std::size_t n = 0;
        for (std::size_t i = 0; i < count; ++i) {
            char one[8] = "";
            std::snprintf(one, sizeof one, "%s%d", i == 0 ? "" : ",",
                          static_cast<int>(ports[i]));
            for (std::size_t k = 0; one[k] != '\0' && n + 1 < cap; ++k) {
                out[n++] = one[k];
            }
        }
        if (cap > 0) {
            out[n < cap ? n : cap - 1] = '\0';
        }
    }

    /// Write the port numbers (magnitudes, "3,13") of every drive slot that is not PRESENT — a
    /// boot hole, or a member marked ABSENT mid-run (T4 Part 2) — LEFT side then RIGHT, read back
    /// from the groups' own slots, which were built from the description in its stored order, so
    /// slot i IS the description's port i. Empty on a complete robot and on a holonomic one.
    /// Bounded by hand like appendPorts(), always NUL-terminated.
    void appendAbsent(char* out, std::size_t cap) const {
        if (cap == 0) {
            return;
        }
        out[0] = '\0';
        if (!left_.has_value()) {
            return;  // holonomic: no groups, nothing can be absent
        }
        const hal::DrivetrainDescription& d = cfg_.drivetrain.tank.description;
        const hal::DriveSideDescription* sides[2] = {&d.left, &d.right};
        const hal::MotorGroup* groups[2] = {&*left_, &*right_};
        std::size_t n = 0;
        for (std::size_t s = 0; s < 2; ++s) {
            for (std::size_t i = 0; i < sides[s]->count; ++i) {
                if (groups[s]->isPresent(i)) {
                    continue;
                }
                char one[8] = "";
                std::snprintf(one, sizeof one, "%s%d", n == 0 ? "" : ",",
                              hal::DrivetrainDescription::portMagnitude(sides[s]->signedPorts[i]));
                for (std::size_t k = 0; one[k] != '\0' && n + 1 < cap; ++k) {
                    out[n++] = one[k];
                }
                out[n] = '\0';
            }
        }
    }

    // ── the graph, in CONSTRUCTION ORDER (declaration order IS initialization order) ────
    //    kinematics → motors → groups → sensors → diagnostics → geometry → odometry →
    //    fusion → correctors → localizer → context → deps → chassis. The same list, in the
    //    same order, as src/main.cpp's hand-wired graphs (its TankRobot until G1 Part 2b; its
    //    X-drive Robot until 2c).

    /// The config this robot was built from, COPIED (see the constructor).
    ///
    /// The PROVIDER is deliberately NOT stored beside it. Every device this graph needs was taken
    /// during construction and is held where it is used; a second handle to the provider would be
    /// an unused member that reads like an ownership claim, and ruling B2 is that the builder owns
    /// no hardware. The provider must still outlive the Robot — the class banner and the
    /// constructor both say so — but that is a lifetime contract, not a field.
    config::RobotConfig cfg_;

    std::optional<kinematics::TankKinematics> tankKin_;
    std::optional<kinematics::MatrixKinematics> matrixKin_;
    const kinematics::IKinematics* kin_;

    MotorWiring wiring_;
    std::optional<hal::MotorGroup> left_;
    std::optional<hal::MotorGroup> right_;
    std::array<hal::IMotor*, static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels)>
        driveMotors_;
    std::size_t driveMotorCount_;
    std::array<hal::MotorGroup*, 2> groups_;
    std::size_t groupCount_;

    hal::IImu* imu_;
    hal::AbsentGps absentGps_;
    hal::IGps* gps_;
    hal::AbsentTagSource tags_;
    hal::AbsentVision vision_;

    diag::FaultLatch faults_;
    diag::HealthMonitor health_;

    hal::DriveGeometry leftGeometry_;
    hal::DriveGeometry rightGeometry_;

    std::optional<localization::DriveEncoderOdometry> driveOdom_;
    std::optional<localization::PilonsOdometry> pilonsOdom_;
    localization::IOdometry* odom_;
    localization::ComplementaryFusion fusion_;

    std::optional<localization::GpsCorrector> gpsCorrector_;
    std::array<localization::ICorrector*, 1> correctors_;
    std::size_t correctorCount_;

    localization::Localizer localizer_;
    chassis::RobotContext ctx_;
    motion::MotionDeps deps_;
    chassis::Chassis chassis_;

    // ── the mechanisms, built in the constructor BODY, after everything above ──────────

    /// Sized to the on-robot provider's refusal buffer (hal::pros::ProsDeviceProvider's
    /// `kRefusalBytes`, 160) so the whole sentence it wrote survives the copy.
    static constexpr std::size_t kMechanismAbsenceBytes = 160;

    /// One declared mechanism's storage, in config order: the motors or lines it borrowed from the
    /// provider (the span the mechanism holds points HERE, so it lives as long as the Robot), the
    /// mechanism itself when it was built — a motor one OR a pneumatic one, never both — and the
    /// reason when it was not.
    struct MechanismSlot {
        /// The motors, in the config's order. Entries past its `motorCount` stay null.
        hal::IMotor* motors[config::kMaxMotorsPerMechanism]{};
        /// The built motor mechanism; empty when ABSENT, pneumatic, or empty.
        std::optional<hal::MotorMechanism> built;
        /// The 3-wire lines, in the config's order (addendum B2). Entries past its
        /// `pneumaticCount` stay null.
        hal::IDigitalOut* lines[config::kMaxPneumaticsPerMechanism]{};
        /// The built pneumatic mechanism; empty when ABSENT, a motor mechanism, or empty.
        std::optional<hal::PneumaticMechanism> pneumatic;
        /// Why it was not built (the provider's sentence, or a fixed one); empty when built.
        char absence[kMechanismAbsenceBytes]{};
    };

    /// Build every configured mechanism, in config order (the header's MECHANISMS section). A
    /// mechanism with motors: asks the provider for each motor with the mechanism's OWN signed
    /// port and cartridge. A mechanism with 3-wire lines (and so, by `valid()`, no motors): asks
    /// for each line at the mechanism's START level — which DRIVES it on a brain — and declares
    /// its SAFE level. Either way it stops at the first refusal, which leaves that mechanism ABSENT
    /// with the provider's sentence and goes on to the next. Commands nothing else. (A provider
    /// precondition — the same port asked for again with a different sign or cartridge, or the
    /// same line at a different level, which the config's collision checks make impossible from
    /// this Robot alone — still throws out of the constructor, loudly, as it should.)
    void buildMechanisms(hal::IDeviceProvider& devices) {
        for (std::size_t m = 0; m < cfg_.mechanismCount; ++m) {
            const config::MechanismConfig& mc = cfg_.mechanisms[m];
            MechanismSlot& slot = mechanismSlots_[m];
            if (mc.motorCount == 0) {
                buildPneumatic(devices, mc, slot);
                continue;
            }
            bool allMade = true;
            for (std::size_t i = 0; i < mc.motorCount; ++i) {
                hal::IMotor* motor = devices.motor(mc.motorPorts[i], mc.cartridge);
                if (motor == nullptr) {
                    copyAbsence(slot, devices.lastRefusal());
                    allMade = false;
                    break;
                }
                slot.motors[i] = motor;
            }
            if (!allMade) {
                continue;
            }
            slot.built.emplace(std::span<hal::IMotor* const>{slot.motors, mc.motorCount},
                               config::toBrakeMode(mc.safeMode), mc.name);
            builtMechanisms_[builtMechanismCount_++] = &*slot.built;
        }
    }

    /// Build one mechanism with no motors (addendum B2): a `hal::PneumaticMechanism` over its
    /// 3-wire lines, each asked for at `pneumaticStart` (making a line drives it — HA-119 — so this
    /// is the program-start level reaching the robot; built while the brain is DISABLED, as in
    /// `initialize()` on a field, whether it reaches the valve then is A4 register HA-157's
    /// second half), declared safe at `pneumaticSafe`, named from this Robot's config copy. A
    /// refused line leaves it ABSENT with the provider's sentence; a mechanism with no lines
    /// either is recorded as having nothing to build. A line made BEFORE
    /// the refused one stays made — driven to the start level, and part of no mechanism. On a
    /// brain that cannot happen: the on-robot provider refuses only a line outside 1..8, and
    /// `RobotConfig::valid()` refuses those first; only a provider that refuses more can do it.
    void buildPneumatic(hal::IDeviceProvider& devices, const config::MechanismConfig& mc,
                        MechanismSlot& slot) {
        if (mc.pneumaticCount == 0) {
            copyAbsence(slot, "no motors and no 3-wire lines -- nothing to build");
            return;
        }
        const bool start = config::toLineValue(mc.pneumaticStart);
        for (std::size_t i = 0; i < mc.pneumaticCount; ++i) {
            hal::IDigitalOut* line = devices.digitalOut(mc.pneumaticPorts[i], start);
            if (line == nullptr) {
                copyAbsence(slot, devices.lastRefusal(),
                            "the device provider refused a 3-wire line and gave no reason");
                return;
            }
            slot.lines[i] = line;
        }
        slot.pneumatic.emplace(std::span<hal::IDigitalOut* const>{slot.lines, mc.pneumaticCount},
                               config::toLineValue(mc.pneumaticSafe), mc.name);
        builtMechanisms_[builtMechanismCount_++] = &*slot.pneumatic;
    }

    /// Copy `why` into `slot.absence`, bounded by hand and always terminated; an empty or null
    /// sentence becomes `fallback`, so an ABSENT mechanism never reports an empty reason.
    static void copyAbsence(
        MechanismSlot& slot, const char* why,
        const char* fallback = "the device provider refused a motor and gave no reason") noexcept {
        const char* text = (why != nullptr && why[0] != '\0') ? why : fallback;
        std::size_t n = 0;
        for (; text[n] != '\0' && n + 1 < kMechanismAbsenceBytes; ++n) {
            slot.absence[n] = text[n];
        }
        slot.absence[n] = '\0';
    }

    /// The slot of the `i`-th DECLARED mechanism, or nullptr for an `i` not below
    /// `mechanismCount()`. The ONE bound every indexed accessor goes through (both constnesses,
    /// by the template), so a single check guards the slot array, and its absence is observable
    /// WITHOUT reading past it: an index in [count, capacity) would find an empty slot, whose
    /// absence text is "" — not the fixed sentence `mechanismAbsence()` gives that index.
    template <typename Self>
    [[nodiscard]] static auto declaredSlot(Self& self, std::size_t i) noexcept
        -> decltype(&self.mechanismSlots_[0]) {
        return i < self.cfg_.mechanismCount ? &self.mechanismSlots_[i] : nullptr;
    }

    /// The index of the declared mechanism named `name`, or `mechanismCount()` when none is (or
    /// `name` is null). Compared over the name field's 24 bytes — enough, because `valid()`
    /// guarantees every declared name has its NUL inside them.
    [[nodiscard]] std::size_t mechanismIndex(const char* name) const noexcept {
        if (name == nullptr) {
            return cfg_.mechanismCount;
        }
        for (std::size_t i = 0; i < cfg_.mechanismCount; ++i) {
            if (std::strncmp(cfg_.mechanisms[i].name, name, sizeof cfg_.mechanisms[i].name) == 0) {
                return i;
            }
        }
        return cfg_.mechanismCount;
    }

    MechanismSlot mechanismSlots_[config::kMaxMechanisms]{};
    std::array<hal::IMechanism*, config::kMaxMechanisms> builtMechanisms_{};
    std::size_t builtMechanismCount_ = 0;
};

}  // namespace shulib::robot
