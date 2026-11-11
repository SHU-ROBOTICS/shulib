#pragma once
//
// robot::MechanismParts — every ROLE of every mechanism, built from the config over a device
// provider (the mechanism core, chunk MECH-A1, 2026-10-08).
//
// ── What it is, and why it is its own class ─────────────────────────────────────────
// Until MECH-A1, robot::Robot built its mechanisms itself (chunk T3 and its addendum B2): a
// hal::MotorMechanism over each mechanism's motors, a hal::PneumaticMechanism over its 3-wire lines,
// ABSENT with the provider's sentence when one would not be made. The mechanism core adds SENSOR
// roles — an angle sensor, a home switch, a piece sensor — and two consumers of the result that are
// not a Robot: the mechanism service (MECH-A2), which runs each mechanism over its `Devices`, and
// Bench Tests' station 13 (MECH-A6), which builds no Robot and no drivetrain but must build the same
// parts to check a mechanism's direction before it is homed. So the builder became this class: the
// storage robot.hpp held (the motors and lines borrowed from the provider, the mechanisms built over
// them, the reason when one was not) MOVED here, and the sensor roles added. The Robot OWNS one and
// forwards every accessor it had to it, unchanged (robot.hpp's MECHANISMS section).
//
// ── The ORDER, per mechanism ────────────────────────────────────────────────────────
//   1. its ACTUATORS — the motors, each with the mechanism's own signed port and cartridge, or the
//      3-wire lines, each made at its START level (on a brain making a line DRIVES it, HA-119; the
//      per-line levels when the config states them). A refusal: the mechanism is ABSENT.
//   2. its ANGLE sensor (a rotation sensor): a refusal makes the mechanism ABSENT — its angle is how
//      it knows where it is.
//   3. its HOME SWITCH (a 3-wire input): always made on a brain (the registry cannot see a 3-wire
//      device — HA-158), so its boot line says "not checkable"; a provider that refuses it anyway
//      (only outside 1..8 on a brain, which RobotConfig::valid() refuses first) makes it ABSENT.
//   4. its PIECE sensor (a distance or optical sensor, or a limit switch): a refusal leaves it
//      RUNNING WITHOUT one — the mechanism is built, and its sensor steps say so (MECH-A2, N8).
// Once a mechanism is ABSENT the rest of its roles are NOT ASKED, and the mechanism is not in
// built(): there is nothing there for a guard to make safe, and nothing a step may command. A
// motor made before the refusal stays made — the provider owns it; nothing commands it.
//
// Beside each motor it records the provider's CURRENT LIMIT for it (IDeviceProvider::currentLimit —
// hal/motor_current_limit.hpp; MECH-S-A), into that mechanism's `Devices`: how the service writes a
// stated cap and reads every mechanism motor's limit back. It writes no limit itself.
//
// Beside each piece sensor and home switch it records the provider's READ-FAULT count for it
// (IDeviceProvider::readFaults / lineReadFaults — hal/read_faults.hpp), into that mechanism's
// `Devices`: the service's rule that a sample taken while it moved is HELD, never evidence.
//
// One `RoleFact` per role — what was asked, on which port or line, and what the provider said — is
// what a program's boot lines word (mechanism::roleLine, mechanism/wiring.hpp): "arm: motor 1 ->
// port 1 ok", "wrist: angle sensor -> port 10 ABSENT: port 10: a distance sensor (expected a
// rotation sensor)", "lift: home switch -> 3-wire B not checkable (...)".
//
// ── What it does NOT do ─────────────────────────────────────────────────────────────
// It never ticks and never commands a motor (the Robot's rule, unchanged): a mechanism is first
// made safe by whatever drives it. It does not check the config — RobotConfig::valid() is the
// owner of every port collision, and a caller builds only a config valid() accepted. It allocates
// nothing: every array is fixed, sized by the config's capacities.
//
// PROS-free, sim-free, allocation-free.

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>

#include "shulib/config/robot_config.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/device_provider.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/mechanism/devices.hpp"

namespace shulib::robot {

static_assert(mechanism::kDevicesMaxMotors == config::kMaxMotorsPerMechanism,
              "mechanism::Devices::currentLimits must hold every motor a mechanism config can have");

/// Every role of every mechanism a config declares, built over a device provider (header). Built
/// ONCE, by build(); non-copyable and non-movable — the built mechanisms hold spans into its storage.
class MechanismParts {
public:
    /// The longest reason a mechanism was not built (or a piece sensor was refused), NUL included —
    /// sized to the on-robot provider's refusal buffer (hal::pros::ProsDeviceProvider::kRefusalBytes)
    /// so the whole sentence it wrote survives the copy.
    static constexpr std::size_t kReasonBytes = 160;
    /// The most roles one mechanism has: its motors (or lines), an angle sensor, a home switch and a
    /// piece sensor.
    static constexpr std::size_t kMaxRoles =
        config::kMaxMotorsPerMechanism + config::kMaxPneumaticsPerMechanism + 3;

    /// Empty: nothing built.
    MechanismParts() = default;
    /// Non-copyable: the built mechanisms hold spans into this object's storage.
    MechanismParts(const MechanismParts&) = delete;
    /// Non-movable, same reasoning.
    MechanismParts(MechanismParts&&) = delete;
    /// Non-copy-assignable, same reasoning.
    MechanismParts& operator=(const MechanismParts&) = delete;
    /// Non-move-assignable, same reasoning.
    MechanismParts& operator=(MechanismParts&&) = delete;
    /// Destroys the mechanisms it built (never the devices: the provider owns those).
    ~MechanismParts() = default;

    /// Build every mechanism of `configs`, in order (header: THE ORDER). The configs are BORROWED,
    /// not copied: each built mechanism's name() is the pointer into them, exactly as the Robot's
    /// mechanisms have always borrowed the Robot's own config copy (robot_builder_test.cpp pins that
    /// pointer) — so `configs` must OUTLIVE this object. The Robot hands in its own copy (made for
    /// exactly that reason: it never keeps a caller's reference); a caller with no Robot keeps its
    /// configs in static storage beside the parts. `devices` is asked for every role and must
    /// outlive this object too. PRECONDITIONS: called once; at most config::kMaxMechanisms configs;
    /// the configs passed RobotConfig::valid() (a provider precondition — one port asked twice with
    /// two configurations, one 3-wire line asked as an output and an input — would otherwise throw
    /// out of here, loudly).
    void build(std::span<const config::MechanismConfig> configs, hal::IDeviceProvider& devices) {
        SHULIB_PRECONDITION(!builtOnce_, "MechanismParts::build: called twice -- build once");
        SHULIB_PRECONDITION(configs.size() <= config::kMaxMechanisms,
                            "MechanismParts::build: more mechanisms than config::kMaxMechanisms");
        builtOnce_ = true;
        configs_ = configs;
        count_ = configs.size();
        for (std::size_t m = 0; m < count_; ++m) {
            buildOne(m, devices);
        }
    }

    /// How many mechanisms were declared (built or not).
    [[nodiscard]] std::size_t size() const noexcept { return count_; }

    /// Every declared mechanism's devices, in config order — what the mechanism service binds over
    /// (MECH-A2). An ABSENT one has every pointer null and its `absence` set.
    [[nodiscard]] std::span<const mechanism::Devices> devices() const noexcept {
        return std::span<const mechanism::Devices>{devices_.data(), count_};
    }

    /// Every BUILT mechanism of both kinds, in config order — the `hal::IMechanism*` span an
    /// end-of-run guard walks. An ABSENT one is not in it.
    [[nodiscard]] std::span<hal::IMechanism* const> built() const noexcept {
        return std::span<hal::IMechanism* const>{built_.data(), builtCount_};
    }

    /// The `i`-th declared mechanism if it was built, of either kind; nullptr when ABSENT, when it has
    /// nothing to build, or when `i` is not below size().
    [[nodiscard]] hal::IMechanism* builtAt(std::size_t i) const noexcept {
        if (i >= count_) {
            return nullptr;
        }
        if (devices_[i].motors != nullptr) {
            return devices_[i].motors;
        }
        return devices_[i].pneumatic;
    }

    /// The `i`-th declared mechanism as a built MOTOR mechanism; nullptr when ABSENT, pneumatic, or
    /// `i` is not below size().
    [[nodiscard]] hal::MotorMechanism* motorAt(std::size_t i) const noexcept {
        return i < count_ ? devices_[i].motors : nullptr;
    }

    /// The `i`-th declared mechanism as a built PNEUMATIC mechanism; nullptr when ABSENT, a motor
    /// mechanism, or `i` is not below size().
    [[nodiscard]] hal::PneumaticMechanism* pneumaticAt(std::size_t i) const noexcept {
        return i < count_ ? devices_[i].pneumatic : nullptr;
    }

    /// Why the `i`-th declared mechanism was NOT built: the provider's sentence naming the port, or
    /// a fixed one for a mechanism with nothing to build. "" EXACTLY when builtAt(i) is non-null.
    /// "no mechanism is declared at this index" for an `i` not below size().
    [[nodiscard]] const char* absence(std::size_t i) const noexcept {
        return i < count_ ? slots_[i].absence : "no mechanism is declared at this index";
    }

    /// The `i`-th mechanism's ROLES, in boot-line order (motors or lines, angle sensor, home switch,
    /// piece sensor) — what each one's boot line words. Empty for an `i` not below size().
    [[nodiscard]] std::span<const mechanism::RoleFact> roles(std::size_t i) const noexcept {
        if (i >= count_) {
            return {};
        }
        return std::span<const mechanism::RoleFact>{slots_[i].roles.data(), slots_[i].roleCount};
    }

    /// The `i`-th mechanism's config (the borrowed one). PRECONDITION: `i` below size().
    [[nodiscard]] const config::MechanismConfig& config(std::size_t i) const {
        SHULIB_PRECONDITION(i < count_, "MechanismParts::config: no mechanism is declared at this index");
        return configs_[i];
    }

    /// The index of the mechanism whose config name is `name`, or size() when none is (or `name` is
    /// null). Compared over the name field's 24 bytes.
    [[nodiscard]] std::size_t indexOf(const char* name) const noexcept {
        if (name == nullptr) {
            return count_;
        }
        for (std::size_t i = 0; i < count_; ++i) {
            if (std::strncmp(configs_[i].name, name, sizeof configs_[i].name) == 0) {
                return i;
            }
        }
        return count_;
    }

private:
    /// One declared mechanism's storage: the devices it borrowed from the provider (the spans the
    /// built mechanisms hold point HERE), the mechanism built over them, the reasons, and its roles.
    struct Slot {
        hal::IMotor* motors[config::kMaxMotorsPerMechanism]{};
        std::optional<hal::MotorMechanism> motorMechanism;
        hal::IDigitalOut* lines[config::kMaxPneumaticsPerMechanism]{};
        bool safeLevels[config::kMaxPneumaticsPerMechanism]{};
        std::optional<hal::PneumaticMechanism> pneumaticMechanism;
        char absence[kReasonBytes]{};
        char pieceRefusal[kReasonBytes]{};
        std::array<mechanism::RoleFact, kMaxRoles> roles{};
        std::size_t roleCount = 0;
    };

    static constexpr const char* kMotorRole[] = {"motor 1", "motor 2", "motor 3", "motor 4"};
    static constexpr const char* kLineRole[] = {"line 1", "line 2"};

    /// Append one role's fact to `s`.
    static void addRole(Slot& s, const char* role, std::uint8_t port, std::uint8_t line,
                        mechanism::RoleState st, bool degradeOnly, const char* refusal) noexcept {
        if (s.roleCount < s.roles.size()) {
            s.roles[s.roleCount++] = mechanism::RoleFact{.role = role, .port = port, .line = line, .state = st,
                                                         .degradeOnly = degradeOnly,
                                                         .refusal = refusal != nullptr ? refusal : ""};
        }
    }

    /// Copy `why` into `dst`, bounded and always terminated; an empty or null sentence becomes
    /// `fallback`, so a refusal never reports an empty reason.
    static void copyReason(char (&dst)[kReasonBytes], const char* why, const char* fallback) noexcept {
        const char* text = (why != nullptr && why[0] != '\0') ? why : fallback;
        std::size_t n = 0;
        for (; text[n] != '\0' && n + 1 < kReasonBytes; ++n) {
            dst[n] = text[n];
        }
        dst[n] = '\0';
    }

    /// The physical port of a signed motor port.
    [[nodiscard]] static std::uint8_t physical(std::int8_t signedPort) noexcept {
        const int p = signedPort < 0 ? -static_cast<int>(signedPort) : static_cast<int>(signedPort);
        return static_cast<std::uint8_t>(p);
    }

    /// Build mechanism `m` in THE ORDER (header), recording each role.
    void buildOne(std::size_t m, hal::IDeviceProvider& devices) {
        const config::MechanismConfig& mc = configs_[m];
        Slot& s = slots_[m];
        mechanism::Devices& d = devices_[m];
        bool absent = false;

        // 1. the actuators — the motors (Robot::buildMechanisms until MECH-A1, moved) or the lines.
        if (mc.motorCount > 0) {
            for (std::size_t i = 0; i < mc.motorCount && i < config::kMaxMotorsPerMechanism; ++i) {
                const std::uint8_t port = physical(mc.motorPorts[i]);
                if (absent) {
                    addRole(s, kMotorRole[i], port, 0, mechanism::RoleState::NotAsked, false, "");
                    continue;
                }
                hal::IMotor* motor = devices.motor(mc.motorPorts[i], mc.cartridge);
                if (motor == nullptr) {
                    copyReason(s.absence, devices.lastRefusal(),
                               "the device provider refused a motor and gave no reason");
                    absent = true;
                    addRole(s, kMotorRole[i], port, 0, mechanism::RoleState::Refused, false, s.absence);
                    continue;
                }
                s.motors[i] = motor;
                addRole(s, kMotorRole[i], port, 0, mechanism::RoleState::Present, false, "");
            }
        } else if (mc.pneumaticCount > 0) {
            const bool perLine = mc.pneumaticSafeByLine[0] != static_cast<config::PneumaticLevel>(0);
            for (std::size_t i = 0; i < mc.pneumaticCount && i < config::kMaxPneumaticsPerMechanism; ++i) {
                const std::uint8_t line = mc.pneumaticPorts[i];
                if (absent) {
                    addRole(s, kLineRole[i], 0, line, mechanism::RoleState::NotAsked, false, "");
                    continue;
                }
                const bool start = config::toLineValue(perLine ? mc.pneumaticStartByLine[i] : mc.pneumaticStart);
                hal::IDigitalOut* out = devices.digitalOut(line, start);
                if (out == nullptr) {
                    copyReason(s.absence, devices.lastRefusal(),
                               "the device provider refused a 3-wire line and gave no reason");
                    absent = true;
                    addRole(s, kLineRole[i], 0, line, mechanism::RoleState::Refused, false, s.absence);
                    continue;
                }
                s.lines[i] = out;
                s.safeLevels[i] = config::toLineValue(perLine ? mc.pneumaticSafeByLine[i] : mc.pneumaticSafe);
                addRole(s, kLineRole[i], 0, line, mechanism::RoleState::NotCheckable, false, "");
            }
        } else {
            copyReason(s.absence, "no motors and no 3-wire lines -- nothing to build", "");
            absent = true;
        }

        // 2. the ANGLE sensor: a refusal makes the mechanism ABSENT.
        hal::IRotation* angle = nullptr;
        if (mc.rotationPort != 0) {
            if (absent) {
                addRole(s, "angle sensor", mc.rotationPort, 0, mechanism::RoleState::NotAsked, false, "");
            } else {
                angle = devices.rotation(mc.rotationPort);
                if (angle == nullptr) {
                    copyReason(s.absence, devices.lastRefusal(),
                               "the device provider refused a rotation sensor and gave no reason");
                    absent = true;
                    addRole(s, "angle sensor", mc.rotationPort, 0, mechanism::RoleState::Refused, false, s.absence);
                } else {
                    addRole(s, "angle sensor", mc.rotationPort, 0, mechanism::RoleState::Present, false, "");
                }
            }
        }

        // 3. the HOME switch: a 3-wire input — made, "not checkable"; a refusal makes it ABSENT.
        hal::IDigitalIn* home = nullptr;
        if (mc.switchLines[0] != 0) {
            const std::uint8_t line = mc.switchLines[0];
            if (absent) {
                addRole(s, "home switch", 0, line, mechanism::RoleState::NotAsked, false, "");
            } else {
                home = devices.digitalIn(line);
                if (home == nullptr) {
                    copyReason(s.absence, devices.lastRefusal(),
                               "the device provider refused a 3-wire input and gave no reason");
                    absent = true;
                    addRole(s, "home switch", 0, line, mechanism::RoleState::Refused, false, s.absence);
                } else {
                    addRole(s, "home switch", 0, line, mechanism::RoleState::NotCheckable, false, "");
                }
            }
        }

        // 4. the PIECE sensor: a refusal leaves the mechanism running WITHOUT one.
        hal::IDistance* pieceDistance = nullptr;
        hal::IOptical* pieceOptical = nullptr;
        hal::IDigitalIn* pieceSwitch = nullptr;
        const hal::IReadFaults* pieceFaults = nullptr;
        if (mc.distancePort != 0 || mc.opticalPort != 0) {
            const std::uint8_t port = mc.distancePort != 0 ? mc.distancePort : mc.opticalPort;
            if (absent) {
                addRole(s, "piece sensor", port, 0, mechanism::RoleState::NotAsked, false, "");
            } else {
                bool made = false;
                if (mc.distancePort != 0) {
                    pieceDistance = devices.distance(port);
                    made = pieceDistance != nullptr;
                } else {
                    pieceOptical = devices.optical(port);
                    made = pieceOptical != nullptr;
                }
                if (made) {
                    pieceFaults = devices.readFaults(port);
                    addRole(s, "piece sensor", port, 0, mechanism::RoleState::Present, true, "");
                } else {
                    copyReason(s.pieceRefusal, devices.lastRefusal(),
                               "the device provider refused a piece sensor and gave no reason");
                    addRole(s, "piece sensor", port, 0, mechanism::RoleState::Refused, true, s.pieceRefusal);
                }
            }
        } else if (mc.switchLines[1] != 0) {
            const std::uint8_t line = mc.switchLines[1];
            if (absent) {
                addRole(s, "piece sensor", 0, line, mechanism::RoleState::NotAsked, false, "");
            } else {
                pieceSwitch = devices.digitalIn(line);
                if (pieceSwitch != nullptr) {
                    pieceFaults = devices.lineReadFaults(line);
                    addRole(s, "piece sensor", 0, line, mechanism::RoleState::NotCheckable, true, "");
                } else {
                    copyReason(s.pieceRefusal, devices.lastRefusal(),
                               "the device provider refused a 3-wire input and gave no reason");
                    addRole(s, "piece sensor", 0, line, mechanism::RoleState::Refused, true, s.pieceRefusal);
                }
            }
        }

        if (absent) {
            d = mechanism::Devices{};
            d.absence = s.absence;
            return;
        }
        // Built: the actuator over the borrowed devices, then its sensors.
        if (mc.motorCount > 0) {
            s.motorMechanism.emplace(std::span<hal::IMotor* const>{s.motors, mc.motorCount},
                                     config::toBrakeMode(mc.safeMode), mc.name);
            d.motors = &*s.motorMechanism;
            built_[builtCount_++] = &*s.motorMechanism;
            // MECH-S-A: each built motor's own current limit, in the motors' order (looked up, never made)
            for (std::size_t i = 0; i < mc.motorCount && i < config::kMaxMotorsPerMechanism; ++i) {
                d.currentLimits[i] = devices.currentLimit(physical(mc.motorPorts[i]));
            }
        } else {
            const bool perLine = mc.pneumaticSafeByLine[0] != static_cast<config::PneumaticLevel>(0);
            const std::span<hal::IDigitalOut* const> lines{s.lines, mc.pneumaticCount};
            if (perLine) {
                s.pneumaticMechanism.emplace(lines, std::span<const bool>{s.safeLevels, mc.pneumaticCount}, mc.name);
            } else {
                s.pneumaticMechanism.emplace(lines, config::toLineValue(mc.pneumaticSafe), mc.name);
            }
            d.pneumatic = &*s.pneumaticMechanism;
            built_[builtCount_++] = &*s.pneumaticMechanism;
        }
        d.angleSensor = angle;
        d.homeSwitch = home;
        d.homeSwitchFaults = home != nullptr ? devices.lineReadFaults(mc.switchLines[0]) : nullptr;
        d.pieceDistance = pieceDistance;
        d.pieceOptical = pieceOptical;
        d.pieceSwitch = pieceSwitch;
        d.pieceFaults = pieceFaults;
        d.absence = "";
    }

    std::span<const config::MechanismConfig> configs_{};
    std::size_t count_ = 0;
    bool builtOnce_ = false;
    Slot slots_[config::kMaxMechanisms]{};
    std::array<mechanism::Devices, config::kMaxMechanisms> devices_{};
    std::array<hal::IMechanism*, config::kMaxMechanisms> built_{};
    std::size_t builtCount_ = 0;
};

}  // namespace shulib::robot
