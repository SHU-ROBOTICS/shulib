#pragma once
//
// RobotConfig — a whole robot, described as DATA (chunk G1 Part 1, 2026-09-18).
//
// ── Why this type exists ────────────────────────────────────────────────────────────
// Everything a composition root says about a robot — which ports drive, which way, how
// the chassis is measured, which sensors are on it, which mechanisms it carries, what the
// motion layer is tuned to — was spelled only as C++ object construction in one place
// (src/main.cpp's hand-wired graphs, until G1 Part 2b built "shulib Teleop" from a
// RobotConfig instead). That is a fine thing for a code-fluent team and a closed
// door for everyone else. This header is the OTHER door: the same facts as a plain value
// a person can type, a calibrator can fill in, or a generator can emit, and which
// robot/robot.hpp turns into the identical object graph.
//
//   ONE representation, THREE producers:
//     * a human typing a `constexpr RobotConfig` (today; the standalone path);
//     * calibration::DrivetrainCalibrator, whose DrivetrainDescription is EMBEDDED here
//       by value rather than mirrored (ruling B1 — a second struct with the same fields is
//       exactly how a calibration record and a config come to disagree about one robot);
//     * G3's codegen from a `.vexbot` file (master plan §16.2). That is why every field is
//       fixed-width and the whole type is constexpr-constructible: a generator emits
//       `inline constexpr RobotConfig kRobot{…};` and the compiler checks it.
//
// ── What this header is NOT ─────────────────────────────────────────────────────────
// It is not a replacement for longhand construction. chassis.hpp's standalone promise is
// unchanged and still pinned by its own file-free test: a builder is "an additional
// on-ramp, never the only path". G1 ADDS; it removes nothing.
//
// It also does not build anything. This file holds data and the rules that say when the
// data contradicts itself. Turning it into a robot is robot/robot.hpp's job, and the one
// cross-check that needs both a config and a drivetrain — every kinematic wheel has a
// motor — deliberately lives THERE and only there (see RobotConfig::valid()'s note).
//
// ── Laid out to be GENERATED and to be SERIALIZED ───────────────────────────────────
// Fixed-width fields, bounded arrays, no pointers, no std::string, no std::function, no
// allocation, no virtual anything in the data types. Capacities are named constants
// (ruling B6): a route or a robot that does not fit is two of them, not a reallocation.
//
// NO HARDWARE CLAIM IS MADE ANYWHERE IN THIS FILE. No RobotConfig has ever been used to
// build a graph that drove a physical robot; the only thing any config here has driven is
// the A2 host plant.
//
// PROS-free, sim-free, allocation-free.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "shulib/chassis/chassis.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::config {

/// The most mechanisms one robot may declare. A DESIGN constant (ruling B6), not a hardware
/// belief: nothing measured says eight, and a robot with more mechanisms than this is asking
/// for a bigger number here, not for a different data structure.
inline constexpr std::size_t kMaxMechanisms = 8;

/// The most motors one mechanism may drive. Design constant (B6), same reasoning as above.
inline constexpr std::size_t kMaxMotorsPerMechanism = 4;

/// The most pneumatic (ADI digital-out) lines one mechanism may actuate. Design constant (B6).
inline constexpr std::size_t kMaxPneumaticsPerMechanism = 2;

/// Which drivetrain a RobotConfig describes.
///
/// DELIBERATELY NOT `hal::DrivetrainKind`, and the distinction matters: that enum is the one
/// written into a persisted calibration record, it is append-only for that reason, and it knows
/// only `Tank` because the calibrator can only measure a tank today. THIS enum is the CONFIG
/// vocabulary and already carries the two holonomic kinds, because src/main.cpp's X-drive graph
/// existed since R1a and had to be expressible as data — since G1 Part 2c it is
/// (src/robots/xdrive_config.hpp, the tree's first `constexpr RobotConfig`). A tank config carries a
/// `hal::DrivetrainDescription`, whose own `kind` field is the persisted one; the two agree by
/// construction because valid() checks them against each other.
enum class DrivetrainKind : std::uint8_t {
    Tank = 1,    ///< two sides, no lateral authority — kinematics/tank.hpp
    XDrive = 2,  ///< four 45° omnis — kinematics/x_drive.hpp
    HDrive = 3,  ///< tank base + one transverse strafe wheel — kinematics/h_drive.hpp
};

/// The drivetrain kind as a word, for banners and refusal messages. "UNSET" for any byte outside
/// the enum, which is what a default-constructed config carries.
[[nodiscard]] inline const char* drivetrainKindWord(DrivetrainKind k) noexcept {
    switch (k) {
        case DrivetrainKind::Tank: return "tank";
        case DrivetrainKind::XDrive: return "x-drive";
        case DrivetrainKind::HDrive: return "h-drive";
    }
    return "UNSET";
}

/// Who this robot is, in words. Banner material only — nothing in the library branches on it.
/// Fixed-width and NUL-terminated so the whole config can be written to a card as it stands.
struct RobotIdentity {
    /// The robot's name, e.g. "Robot Two". Empty is legal (an unnamed robot still drives).
    char name[32]{};
    /// The team number, e.g. "1234A".
    char team[16]{};
    /// The season, e.g. "Push Back".
    char season[16]{};
};

/// A tank drivetrain: R3d's `hal::DrivetrainDescription`, EMBEDDED BY VALUE and unchanged
/// (ruling B1). Everything a tank needs is already in it — signed ports per side, inches per
/// radian per side, effective track width, cartridge, IMU port, source and provenance — and it
/// is the type the calibrator produces and the record persists, so a config built from a card is
/// a field copy and nothing else.
struct TankDrivetrain {
    /// The description, exactly as the calibrator or the typed table produced it. Its own
    /// `valid()` is called by RobotConfig::valid(), and its message surfaces verbatim.
    hal::DrivetrainDescription description{};
};

/// A symmetric 45° X-drive, typed by hand (no calibrator measures one yet).
struct XDriveDrivetrain {
    /// The four wheel ports in x_drive.hpp's canonical F5 wheel order — 0 front-left,
    /// 1 back-left, 2 back-right, 3 front-right — WITH THEIR SIGNS, exactly as
    /// `hal::DriveSideDescription::signedPorts` carries a tank's: `-11` is port 11 handed to the
    /// adapter negative, which PROS reverses exactly once. The sign lives here and nowhere else.
    /// A `0` entry is "no motor on that wheel", which is representable and refused by
    /// `robot::validate()`'s wheel-count cross-check, not here (see RobotConfig::valid()).
    std::int8_t ports[4]{};
    /// The cartridge in the drive motors. HUMAN-STATED and unverifiable from inside, exactly as
    /// `hal::MotorCartridge` documents; `valid()` refuses any byte outside the enum.
    hal::MotorCartridge cartridge = static_cast<hal::MotorCartridge>(0);
    /// The IMU's smart port, 1..hal::kMaxSmartPorts. 0 is UNSET and invalid — every drivetrain
    /// here is heading-owned by an IMU.
    std::uint8_t imuPort = 0;
    /// Centre-to-wheel distance in inches, the ONE geometry input `kinematics::xDrive()` takes.
    /// Must be finite and > 0.
    units::Length driveRadius{0.0};
};

/// An H-drive: a tank base plus one transverse strafe wheel, typed by hand.
struct HDriveDrivetrain {
    /// The three wheel ports in h_drive.hpp's wheel order — 0 LEFT drive line, 1 RIGHT drive
    /// line, 2 STRAFE — signed, with the same one-place-for-a-sign rule as the X-drive's.
    /// Spelling the strafe wheel as `ports[2]` rather than a separate field is deliberate: the
    /// kinematics numbers it as wheel 2, and a separate field would need a second rule saying
    /// where it goes in the motor array.
    std::int8_t ports[3]{};
    /// The cartridge in the drive motors. HUMAN-STATED; `valid()` refuses a byte outside the enum.
    hal::MotorCartridge cartridge = static_cast<hal::MotorCartridge>(0);
    /// The IMU's smart port, 1..hal::kMaxSmartPorts. 0 is UNSET and invalid.
    std::uint8_t imuPort = 0;
    /// Contact line to contact line, inches, finite and > 0 — `hDrive()`'s `trackWidth`.
    units::Length trackWidth{0.0};
    /// The strafe wheel's +FORWARD body coordinate, SIGNED and finite (negative = behind centre)
    /// — `hDrive()`'s `strafeWheelOffset`.
    units::Length strafeWheelOffset{0.0};
};

/// Which drivetrain this robot has, and its numbers.
///
/// A TAGGED STRUCT rather than a union: all three alternatives are stored and `kind` says which
/// one is read. The alternative — `std::variant` or a real union — was rejected because it costs
/// this type its aggregate-ness, and being an aggregate is precisely what lets a person (and G3's
/// generator) write `{.kind = …, .tank = {…}}` with designated initializers and have the compiler
/// check it. The price is about thirty bytes of static storage on a brain that has megabytes,
/// paid once.
struct DrivetrainConfig {
    /// Which of the three members below is the real one. Everything else is ignored — not
    /// validated, not read, not built.
    DrivetrainKind kind = DrivetrainKind::Tank;
    /// The tank alternative, read only when `kind == Tank`.
    TankDrivetrain tank{};
    /// The X-drive alternative, read only when `kind == XDrive`.
    XDriveDrivetrain xdrive{};
    /// The H-drive alternative, read only when `kind == HDrive`.
    HDriveDrivetrain hdrive{};

    /// Which alternative is live — the accessor the brief names, so call sites read
    /// `drivetrain.kind()`-shaped code even though the tag is a plain field.
    [[nodiscard]] constexpr DrivetrainKind kindOf() const noexcept { return kind; }

    /// The IMU port this drivetrain's heading comes from, whichever alternative is live.
    /// 0 when `kind` is not a known kind.
    [[nodiscard]] constexpr std::uint8_t imuPort() const noexcept {
        switch (kind) {
            case DrivetrainKind::Tank: return tank.description.imuPort;
            case DrivetrainKind::XDrive: return xdrive.imuPort;
            case DrivetrainKind::HDrive: return hdrive.imuPort;
        }
        return 0;
    }

    /// The cartridge in the drive motors, whichever alternative is live. The byte 0 (UNSET) when
    /// `kind` is not a known kind.
    [[nodiscard]] constexpr hal::MotorCartridge cartridge() const noexcept {
        switch (kind) {
            case DrivetrainKind::Tank: return tank.description.cartridge;
            case DrivetrainKind::XDrive: return xdrive.cartridge;
            case DrivetrainKind::HDrive: return hdrive.cartridge;
        }
        return static_cast<hal::MotorCartridge>(0);
    }

    /// How many KINEMATIC WHEELS the installed drivetrain has — 2 tank, 4 X-drive, 3 H-drive.
    /// This is the number `kinematics::IKinematics::wheelCount()` will report, derived from the
    /// config so `robot::validate()` can make the F5 cross-check before any object exists.
    /// 0 when `kind` is not a known kind.
    [[nodiscard]] constexpr int wheelCount() const noexcept {
        switch (kind) {
            case DrivetrainKind::Tank: return 2;
            case DrivetrainKind::XDrive: return 4;
            case DrivetrainKind::HDrive: return 3;
        }
        return 0;
    }

    /// How many `hal::IMotor` handles the builder will place in the RobotContext — ONE per
    /// kinematic wheel when the config is complete. A tank always contributes 2 (one MotorGroup
    /// per side, and a side with no ports is already refused by the description's own `valid()`);
    /// a holonomic drivetrain contributes one per NON-ZERO port slot, so a wheel whose port was
    /// left at 0 makes this number smaller than `wheelCount()` — which is exactly the
    /// disagreement `robot::validate()` exists to name.
    [[nodiscard]] constexpr int driveMotorCount() const noexcept {
        switch (kind) {
            case DrivetrainKind::Tank: return 2;
            case DrivetrainKind::XDrive: {
                int n = 0;
                for (const std::int8_t p : xdrive.ports) {
                    if (p != 0) {
                        ++n;
                    }
                }
                return n;
            }
            case DrivetrainKind::HDrive: {
                int n = 0;
                for (const std::int8_t p : hdrive.ports) {
                    if (p != 0) {
                        ++n;
                    }
                }
                return n;
            }
        }
        return 0;
    }
};

/// Which estimator feeds the Localizer.
enum class OdometryKind : std::uint8_t {
    /// The drive encoders themselves — `localization::DriveEncoderOdometry` over the two side
    /// groups. Needs no extra hardware and is the default for a tank. NOT available on a
    /// holonomic drivetrain: that estimator reads exactly two sides and an X-drive has none.
    DriveEncoders = 1,
    /// Two unpowered tracking wheels — `localization::PilonsOdometry`. The only choice for a
    /// holonomic drivetrain, and the better one for a tank that has the pods.
    TrackingWheels = 2,
};

/// The odometry kind as a word, for banners. "UNSET" for a byte outside the enum.
[[nodiscard]] inline const char* odometryKindWord(OdometryKind k) noexcept {
    switch (k) {
        case OdometryKind::DriveEncoders: return "drive-encoders";
        case OdometryKind::TrackingWheels: return "tracking-wheels";
    }
    return "UNSET";
}

/// One unpowered tracking wheel: where it is plugged in, how big it is, and where it sits.
/// The role (forward or lateral) is NOT a field — it is fixed by which slot of
/// `TrackingWheelsOdometry` the wheel occupies, exactly as `localization::TrackingWheel`'s
/// private constructor and two factories fix it, so a pair handed over in the wrong order cannot
/// be represented rather than merely being rejected.
struct TrackingWheelConfig {
    /// The rotation sensor's smart port, 1..hal::kMaxSmartPorts. 0 is UNSET and invalid.
    /// Unsigned: a rotation sensor's direction is a mounting fact expressed by the wheel's
    /// offset sign and the sensor's own reading, not by a negative port.
    std::uint8_t port = 0;
    /// Wheel diameter in inches, finite and > 0.
    units::Length wheelDiameter{0.0};
    /// The wheel's signed offset from the tracking centre, inches, finite — the +LEFT coordinate
    /// for the FORWARD wheel and the +FORWARD coordinate for the LATERAL one (tracking_wheel.hpp).
    units::Length offset{0.0};
};

/// The two-pod (Pilons) odometry's geometry.
struct TrackingWheelsOdometry {
    /// The FORWARD-rolling wheel (body +X); its `offset` is its +LEFT coordinate.
    TrackingWheelConfig forward{};
    /// The LATERAL-rolling wheel (body +Y); its `offset` is its +FORWARD coordinate.
    TrackingWheelConfig lateral{};
};

/// Which estimator this robot runs, and its geometry.
///
/// Same tagged-struct shape and the same reasoning as DrivetrainConfig. Note what this means for
/// a tank that HAS pods but is configured `DriveEncoders`: the `wheels` member is simply not
/// read — not validated, not port-claimed, not built. Declaring pods and then choosing drive
/// encoders is therefore a no-op, not an error; the brief calls that a warning rather than a
/// refusal, and the honest place for a warning is the boot banner, not a validity rule that would
/// make an over-complete config invalid.
struct OdometryConfig {
    /// Which estimator to build.
    OdometryKind kind = OdometryKind::DriveEncoders;
    /// The tracking-wheel geometry, read only when `kind == TrackingWheels`.
    TrackingWheelsOdometry wheels{};
};

/// The GPS, if this robot has one.
///
/// ABSENCE IS A FIELD, NOT A ZERO PORT. `present = false` means the robot has no GPS, and
/// robot/robot.hpp answers that with the explicit `hal::AbsentGps` in the context and NO
/// corrector installed — never a null and never a fake (landmine 5). A GPS-less robot is the
/// common case here: robot two has none, and Driving Skills has no field strip at all.
struct GpsConfig {
    /// Whether a GPS is installed. When false every other field is ignored.
    bool present = false;
    /// The GPS's smart port, 1..hal::kMaxSmartPorts. 0 is UNSET and invalid when `present`.
    std::uint8_t port = 0;
    /// The sensor's +FORWARD body coordinate from the robot centre, inches, finite.
    units::Length leverArmForward{0.0};
    /// The sensor's +LEFT body coordinate from the robot centre, inches, finite.
    units::Length leverArmLeft{0.0};
};

/// Every sensor this robot has that is not part of the drivetrain or the odometry.
///
/// TAGS AND VISION ARE DELIBERATELY ABSENT IN PART 1, and that is a statement about the tree
/// rather than about the schema: no camera adapter exists (R2 owns it), so a field here would
/// describe hardware the library cannot open. `robot::Robot` installs `hal::AbsentTagSource` and
/// `hal::AbsentVision` unconditionally, exactly as src/main.cpp's hand-wired graphs did.
struct SensorsConfig {
    /// The GPS, present or absent.
    GpsConfig gps{};
};

/// A motor mechanism's DECLARED safe brake mode, as config data — what `robot::Robot` hands to
/// `hal::MotorMechanism`'s constructor, and so what every stop of that mechanism lands in
/// (`applySafeState()`: this mode, then 0 V).
///
/// DELIBERATELY NOT `hal::BrakeMode`, and the reason is landmine-sized: `hal::BrakeMode`'s first
/// enumerator is `Coast`, so a value-initialised field of that type would silently declare COAST
/// — and a loaded arm or claw at 0 V and Coast drops its load. There is no safe default for a
/// mechanism (hal/mechanism.hpp's banner: Hold for a loaded lift, Coast or Brake for a jammed
/// intake), so the config refuses to have one: like `hal::MotorCartridge`, this enum has NO
/// `Unset` member, the byte 0 is what a config nobody filled in carries, and
/// `RobotConfig::valid()` refuses it for any mechanism that has motors. Fixed-width, so a
/// generator can emit it and a card can store it.
enum class SafeBrakeMode : std::uint8_t {
    Coast = 1,  ///< undriven — for an intake that must never sit at stall current
    Brake = 2,  ///< brakes and resists, but does not return to where it stopped
    Hold = 3,   ///< drives back to the angle it stopped at — the one that keeps a load up
};

/// The safe brake mode as a word, for banners and refusal messages. "UNSET" for any byte outside
/// the enum, which is what a default-constructed `MechanismConfig` carries.
[[nodiscard]] inline const char* safeBrakeModeWord(SafeBrakeMode m) noexcept {
    switch (m) {
        case SafeBrakeMode::Coast: return "coast";
        case SafeBrakeMode::Brake: return "brake";
        case SafeBrakeMode::Hold: return "hold";
    }
    return "UNSET";
}

/// True when `m` is one of the three modes — false for the UNSET byte 0 and any other byte.
[[nodiscard]] constexpr bool isSafeBrakeMode(SafeBrakeMode m) noexcept {
    return m == SafeBrakeMode::Coast || m == SafeBrakeMode::Brake || m == SafeBrakeMode::Hold;
}

/// The HAL brake mode a `SafeBrakeMode` stands for. Total: a byte outside the enum converts to
/// `Hold` — the mode that drops nothing — and never to Coast. That fallback is a floor, not a
/// default anyone relies on: `robot::Robot` only converts a mode `RobotConfig::valid()` accepted.
[[nodiscard]] constexpr hal::BrakeMode toBrakeMode(SafeBrakeMode m) noexcept {
    switch (m) {
        case SafeBrakeMode::Coast: return hal::BrakeMode::Coast;
        case SafeBrakeMode::Brake: return hal::BrakeMode::Brake;
        case SafeBrakeMode::Hold: return hal::BrakeMode::Hold;
    }
    return hal::BrakeMode::Hold;
}

/// A pneumatic mechanism's line LEVEL, as config data — its declared SAFE level and its START
/// level (`MechanismConfig::pneumaticSafe`, `pneumaticStart`). `Low` is the line off (`false`:
/// the solenoid unpowered), `High` the line on (`true`: powered).
///
/// DELIBERATELY NOT `bool`, for the reason `SafeBrakeMode` is not a `hal::BrakeMode`: a
/// value-initialised bool is `false`, so a field of that type would silently declare "off" for a
/// config nobody filled in — and whether off leaves a claw OPEN (dropping what it holds) or CLOSED
/// depends on how its valve is plumbed, which only the team knows. So, like `hal::MotorCartridge`,
/// this enum has NO `Unset` member, the byte 0 is what a config nobody filled in carries, and
/// `RobotConfig::valid()` refuses it for any mechanism that has 3-wire lines (T3's addendum B2).
enum class PneumaticLevel : std::uint8_t {
    Low = 1,   ///< the line off (`false`) — the solenoid unpowered
    High = 2,  ///< the line on (`true`) — the solenoid powered
};

/// The level as a word, for banners and refusal messages: "low", "high", or "UNSET" for any byte
/// outside the enum, which is what a default-constructed `MechanismConfig` carries.
[[nodiscard]] inline const char* pneumaticLevelWord(PneumaticLevel l) noexcept {
    switch (l) {
        case PneumaticLevel::Low: return "low";
        case PneumaticLevel::High: return "high";
    }
    return "UNSET";
}

/// True when `l` is `Low` or `High` — false for the UNSET byte 0 and any other byte.
[[nodiscard]] constexpr bool isPneumaticLevel(PneumaticLevel l) noexcept {
    return l == PneumaticLevel::Low || l == PneumaticLevel::High;
}

/// The line value a level stands for: `High` is `true`, `Low` is `false`. Only for a level
/// `RobotConfig::valid()` accepted — `robot::Robot` converts nothing else — because there is no
/// safe floor to fall back to here (unlike `toBrakeMode()`'s Hold): either value moves a cylinder.
[[nodiscard]] constexpr bool toLineValue(PneumaticLevel l) noexcept {
    return l == PneumaticLevel::High;
}

/// One mechanism — an intake, a lift, a clamp, a claw pivot, a pneumatic claw — as data.
///
/// BUILT since chunk T3 (2026-09-26): `robot::Robot` builds ONE `hal::MotorMechanism` per
/// mechanism that has motors, asking the provider for each motor with THIS mechanism's cartridge
/// (not the drivetrain's), names it with `name`, and exposes it by name (`Robot::mechanism()`) and
/// as the `hal::IMechanism*` span an end-of-run guard walks (`Robot::mechanisms()`). Since T3's
/// addendum B2 (the same day) it also builds ONE `hal::PneumaticMechanism` per mechanism that has
/// 3-wire lines and no motors — each line made at `pneumaticStart` (on a brain, making a line
/// DRIVES it), the mechanism's safe level `pneumaticSafe` — found by `Robot::pneumaticMechanism()`
/// and in the same span. A device the provider will not make leaves that mechanism ABSENT,
/// reported with the provider's sentence — never the whole robot refused. A mechanism with BOTH
/// motors and lines is refused by `RobotConfig::valid()`: one name is one object, and a voltage
/// and a level do not fan through one. What a mechanism MEANS — which buttons drive it, what
/// confirms it — stays with the program (teleop/mechanism_binding.hpp, manipulation/).
///
/// Two mechanisms that must move independently are TWO entries, never one entry with two motors:
/// a `MotorMechanism` fans ONE voltage to all its motors, so two claws in one entry would move
/// together on every button. The same holds for lines: a `PneumaticMechanism` fans ONE level to
/// every line, so its two lines are two valves fired together — NOT the two coils of a
/// double-coil solenoid, which need opposite levels and are not built by this library. How one
/// WOULD be, once a robot is known to have one (said here, deliberately not built — T3's addendum
/// B2): its two lines stay ONE mechanism (one name, one toggle button), and each line gains a
/// stated polarity; for an inverted line the builder asks the provider at the OPPOSITE start level
/// and hands the mechanism a one-line `hal::IDigitalOut` adapter that writes (and reports) the
/// opposite of what it is told — so `PneumaticMechanism` still fans one level, each coil gets its
/// own, and nothing else changes: not the toggle, not the guard's span.
struct MechanismConfig {
    /// The mechanism's name — the key `robot::Robot::mechanism()` looks it up by, and the key a
    /// command-id registry will bind against. Non-empty, NUL-terminated inside these 24 bytes, and
    /// unique among the robot's mechanisms (all three refused by `RobotConfig::valid()`). The Robot
    /// BORROWS this text for the mechanism's `name()` from its own copy of the config.
    char name[24]{};
    /// The mechanism's motor ports WITH THEIR SIGNS, same one-place-for-a-sign rule as the
    /// drivetrain's: a pivot that moves the wrong way on its forward button is fixed by flipping
    /// the sign HERE, nowhere else. Entries beyond `motorCount` are 0 and are not read.
    std::int8_t motorPorts[kMaxMotorsPerMechanism]{};
    /// How many entries of `motorPorts` are real. 0 is legal — a purely pneumatic mechanism (a
    /// mechanism with motors AND lines is refused).
    std::uint8_t motorCount = 0;
    /// The cartridge in THIS mechanism's motors — its own, never the drivetrain's (a RED claw on a
    /// BLUE drive is the normal case). HUMAN-STATED and unverifiable from inside, exactly as
    /// `hal::MotorCartridge` documents. The byte 0 is UNSET, and `RobotConfig::valid()` refuses it
    /// (or any byte outside the enum) whenever `motorCount > 0`; not read when `motorCount == 0`.
    hal::MotorCartridge cartridge = static_cast<hal::MotorCartridge>(0);
    /// The DECLARED safe brake mode every stop of this mechanism lands in. The byte 0 is UNSET, and
    /// `RobotConfig::valid()` refuses it whenever `motorCount > 0` — see `SafeBrakeMode` for why
    /// there is no default. Not read when `motorCount == 0`.
    SafeBrakeMode safeMode = static_cast<SafeBrakeMode>(0);
    /// The mechanism's pneumatic (ADI digital-out) lines: the brain's three-wire ports as numbers,
    /// 1..8 for A..H. Entries beyond `pneumaticCount` are 0 and are not read. No line may appear
    /// twice — in one mechanism or across two (refused by `RobotConfig::valid()`). BUILT since T3's
    /// addendum B2, as one `hal::PneumaticMechanism` (see above).
    std::uint8_t pneumaticPorts[kMaxPneumaticsPerMechanism]{};
    /// How many entries of `pneumaticPorts` are real. 0 is legal — a purely motorized mechanism.
    std::uint8_t pneumaticCount = 0;
    /// The level every line is driven to whenever anything STOPS this mechanism — its
    /// `applySafeState()`, which an end-of-run guard calls. (A driver's toggle button never
    /// commands it: a claw keeps its level through a disable.) The byte 0 is UNSET, and
    /// `RobotConfig::valid()` refuses it whenever `pneumaticCount > 0` — see `PneumaticLevel` for
    /// why there is no default. Not read when `pneumaticCount == 0`.
    PneumaticLevel pneumaticSafe = static_cast<PneumaticLevel>(0);
    /// The level every line is driven to when the Robot is BUILT — the program-start state (a claw
    /// that starts closed). On a brain, making a 3-wire line drives it (A4 register HA-119), so
    /// this is the one level the builder itself puts on the robot. The byte 0 is UNSET, refused by
    /// `RobotConfig::valid()` whenever `pneumaticCount > 0`. Not read when `pneumaticCount == 0`.
    PneumaticLevel pneumaticStart = static_cast<PneumaticLevel>(0);
};

/// The motion layer's gains, budgets and fault policy for this robot — `chassis::ChassisConfig`
/// passed through WHOLE, not mirrored, for exactly the reason ruling B1 gives for the drivetrain
/// description: a second struct with the same fields drifts. Every value in it is PROVISIONAL
/// until R5 tunes on hardware (HA-45/50/51/52).
using TuningConfig = chassis::ChassisConfig;

/// What claims a smart port, for the one-owner-per-port check. Ordered by nothing; the values are
/// interior to `RobotConfig::valid()` and are not persisted anywhere.
enum class PortUse : std::uint8_t {
    Free = 0,      ///< nothing claims this port
    DriveLeft,     ///< a tank LEFT-side drive motor
    DriveRight,    ///< a tank RIGHT-side drive motor
    DriveWheel,    ///< a holonomic drive motor (X-drive or H-drive wheel)
    Imu,           ///< the drivetrain's inertial sensor
    PodForward,    ///< the forward tracking wheel's rotation sensor
    PodLateral,    ///< the lateral tracking wheel's rotation sensor
    Gps,           ///< the GPS
    Mechanism,     ///< a mechanism motor
};

/// A port claim as words, for the collision message — "port 7 is claimed by the LEFT drive and by
/// a mechanism motor" reads as an instruction to go and fix one of them.
[[nodiscard]] inline const char* portUseWord(PortUse u) noexcept {
    switch (u) {
        case PortUse::Free: return "nothing";
        case PortUse::DriveLeft: return "the LEFT drive";
        case PortUse::DriveRight: return "the RIGHT drive";
        case PortUse::DriveWheel: return "a drive wheel";
        case PortUse::Imu: return "the IMU";
        case PortUse::PodForward: return "the forward tracking wheel";
        case PortUse::PodLateral: return "the lateral tracking wheel";
        case PortUse::Gps: return "the GPS";
        case PortUse::Mechanism: return "a mechanism motor";
    }
    return "something unnamed";
}

/// EVERYTHING about one robot, as one value.
///
/// Plain data with designated initializers at the call site, like every other config type in this
/// tree. Nothing validates itself on construction: a caller builds one and then asks `valid()`,
/// which is what lets a host test build a deliberately contradictory config by hand and watch it
/// be refused by name.
struct RobotConfig {
    /// Who this robot is — banner material.
    RobotIdentity identity{};
    /// Which drivetrain it has, and its numbers.
    DrivetrainConfig drivetrain{};
    /// Which estimator it runs.
    OdometryConfig odometry{};
    /// Which other sensors it carries.
    SensorsConfig sensors{};
    /// Its mechanisms. Entries beyond `mechanismCount` are ignored entirely.
    MechanismConfig mechanisms[kMaxMechanisms]{};
    /// How many entries of `mechanisms` are real. Must be <= kMaxMechanisms.
    std::uint8_t mechanismCount = 0;
    /// The motion layer's gains, budgets and fault policy. Defaults are the library's
    /// PROVISIONAL ones.
    TuningConfig tuning{};

    /// True when nothing in the config contradicts itself. `why` (optional; pass nullptr and 0 to
    /// skip) receives the FIRST contradiction found, naming the field or the port.
    ///
    /// WHAT IT CHECKS — exactly the mistakes a typo, a mis-wire or a mis-generated file produce:
    /// an unknown drivetrain or odometry kind byte; a tank whose `hal::DrivetrainDescription` is
    /// invalid (ITS OWN message surfaces verbatim — one representation, one refusal); a tank
    /// whose description disagrees with the config tag; an unknown cartridge byte; an IMU port
    /// outside 1..21; a holonomic port outside -21..21; the same holonomic wheel port twice; a
    /// non-finite or non-positive drive radius / track width; a non-finite strafe offset;
    /// `DriveEncoders` on a holonomic drivetrain (that estimator reads two sides and a holonomic
    /// drivetrain has none); a tracking wheel with an unset port, a non-positive diameter or a
    /// non-finite offset; a GPS declared present with an unset port or a non-finite lever arm;
    /// more mechanisms than capacity, a mechanism with an empty name or a name with no NUL inside
    /// its 24 bytes, too many motors or pneumatics, a motor port out of range or a pneumatic line
    /// outside 1..8, a 3-wire line listed twice (by two mechanisms or by one — T3's addendum B2);
    /// and — across ALL of the above at once — any smart port claimed by two different things,
    /// naming both (so a mechanism port that is also a drive port is refused here, before any
    /// device is asked for with either cartridge). THEN, in a second pass once every port is
    /// claimed (chunk T3 — so each collision above keeps its own message): a mechanism with BOTH
    /// motors and 3-wire lines (addendum B2); for every mechanism with motors, an UNSET or unknown
    /// cartridge byte, an UNSET or unknown safe brake mode byte; for every mechanism with lines
    /// (addendum B2), an UNSET or unknown safe level byte, an UNSET or unknown start level byte;
    /// and, for all mechanisms, two with the same name.
    ///
    /// WHAT IT DELIBERATELY DOES NOT CHECK, and why the omission is a design decision rather than
    /// a hole: that every kinematic wheel actually HAS a motor. A holonomic port left at 0 is
    /// representable ("no port" is the tree's meaning for 0 everywhere else), and the refusal it
    /// deserves is the F5 cross-check `robot::validate()` already has to make — the same
    /// wheel-count-equals-motor-count check `motion::MotionDeps::validate()` makes at the bottom
    /// of the stack. Making it here as WELL would be two owners for one rule, which is how two
    /// checks come to disagree; one owner, named in both places, is the choice.
    ///
    /// noexcept and total: it allocates nothing, throws nothing, and returns for every input.
    [[nodiscard]] bool valid(char* why = nullptr, std::size_t cap = 0) const noexcept {
        if (why != nullptr && cap > 0) {
            why[0] = '\0';
        }
        // One owner per smart port, accumulated across every section below. Index 0 is unused so
        // a physical port number indexes directly.
        PortUse owner[hal::kMaxSmartPorts + 1]{};
        const auto claim = [&](int port, PortUse use) noexcept -> bool {
            if (port < 1 || port > static_cast<int>(hal::kMaxSmartPorts)) {
                return true;  // range is each section's own check, with its own message
            }
            const auto i = static_cast<std::size_t>(port);
            if (owner[i] != PortUse::Free) {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap, "port %d is claimed by %s and by %s", port,
                                  portUseWord(owner[i]), portUseWord(use));
                }
                return false;
            }
            owner[i] = use;
            return true;
        };

        // ── the drivetrain ─────────────────────────────────────────────────────────────
        switch (drivetrain.kind) {
            case DrivetrainKind::Tank: {
                const hal::DrivetrainDescription& d = drivetrain.tank.description;
                // The description's OWN refusal, verbatim: one representation, one message, so a
                // person who has seen it from the calibrator sees the identical sentence here.
                if (!d.valid(why, cap)) {
                    return false;
                }
                if (!claim(static_cast<int>(d.imuPort), PortUse::Imu)) {
                    return false;
                }
                for (std::size_t i = 0; i < d.left.count; ++i) {
                    if (!claim(hal::DrivetrainDescription::portMagnitude(d.left.signedPorts[i]),
                               PortUse::DriveLeft)) {
                        return false;
                    }
                }
                for (std::size_t i = 0; i < d.right.count; ++i) {
                    if (!claim(hal::DrivetrainDescription::portMagnitude(d.right.signedPorts[i]),
                               PortUse::DriveRight)) {
                        return false;
                    }
                }
                break;
            }
            case DrivetrainKind::XDrive: {
                const XDriveDrivetrain& x = drivetrain.xdrive;
                if (!validCartridge(x.cartridge, why, cap)) {
                    return false;
                }
                if (!validImuPort(x.imuPort, why, cap)) {
                    return false;
                }
                if (!claim(static_cast<int>(x.imuPort), PortUse::Imu)) {
                    return false;
                }
                for (std::size_t i = 0; i < 4; ++i) {
                    if (!validWheelPort(x.ports[i], "x-drive", i, why, cap)) {
                        return false;
                    }
                    if (!claim(hal::DrivetrainDescription::portMagnitude(x.ports[i]),
                               PortUse::DriveWheel)) {
                        return false;
                    }
                }
                const double r = x.driveRadius.value();
                if (!std::isfinite(r) || r <= 0.0) {
                    if (why != nullptr && cap > 0) {
                        std::snprintf(why, cap,
                                      "x-drive driveRadius is %g -- must be finite and > 0 "
                                      "(0 = UNSET)",
                                      r);
                    }
                    return false;
                }
                break;
            }
            case DrivetrainKind::HDrive: {
                const HDriveDrivetrain& hcfg = drivetrain.hdrive;
                if (!validCartridge(hcfg.cartridge, why, cap)) {
                    return false;
                }
                if (!validImuPort(hcfg.imuPort, why, cap)) {
                    return false;
                }
                if (!claim(static_cast<int>(hcfg.imuPort), PortUse::Imu)) {
                    return false;
                }
                for (std::size_t i = 0; i < 3; ++i) {
                    if (!validWheelPort(hcfg.ports[i], "h-drive", i, why, cap)) {
                        return false;
                    }
                    if (!claim(hal::DrivetrainDescription::portMagnitude(hcfg.ports[i]),
                               PortUse::DriveWheel)) {
                        return false;
                    }
                }
                const double w = hcfg.trackWidth.value();
                if (!std::isfinite(w) || w <= 0.0) {
                    if (why != nullptr && cap > 0) {
                        std::snprintf(why, cap,
                                      "h-drive trackWidth is %g -- must be finite and > 0 "
                                      "(0 = UNSET)",
                                      w);
                    }
                    return false;
                }
                if (!std::isfinite(hcfg.strafeWheelOffset.value())) {
                    if (why != nullptr && cap > 0) {
                        std::snprintf(why, cap, "h-drive strafeWheelOffset is %g -- must be finite",
                                      hcfg.strafeWheelOffset.value());
                    }
                    return false;
                }
                break;
            }
            default:
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap, "drivetrain kind byte %d is not tank, x-drive or h-drive",
                                  static_cast<int>(static_cast<std::uint8_t>(drivetrain.kind)));
                }
                return false;
        }

        // ── the odometry ───────────────────────────────────────────────────────────────
        switch (odometry.kind) {
            case OdometryKind::DriveEncoders:
                // The one drivetrain/odometry cross-check: DriveEncoderOdometry reads exactly a
                // LEFT and a RIGHT side and converts each with its own geometry. A holonomic
                // drivetrain has neither, so this pairing is not merely unsupported, it is
                // unrepresentable — and saying so here is cheaper than a precondition three
                // objects deep.
                if (drivetrain.kind != DrivetrainKind::Tank) {
                    if (why != nullptr && cap > 0) {
                        std::snprintf(why, cap,
                                      "odometry DriveEncoders needs two drive sides; a %s "
                                      "drivetrain has none -- use TrackingWheels",
                                      drivetrainKindWord(drivetrain.kind));
                    }
                    return false;
                }
                break;
            case OdometryKind::TrackingWheels: {
                if (!validPod(odometry.wheels.forward, "forward", why, cap)) {
                    return false;
                }
                if (!claim(static_cast<int>(odometry.wheels.forward.port), PortUse::PodForward)) {
                    return false;
                }
                if (!validPod(odometry.wheels.lateral, "lateral", why, cap)) {
                    return false;
                }
                if (!claim(static_cast<int>(odometry.wheels.lateral.port), PortUse::PodLateral)) {
                    return false;
                }
                break;
            }
            default:
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap,
                                  "odometry kind byte %d is not DriveEncoders or TrackingWheels",
                                  static_cast<int>(static_cast<std::uint8_t>(odometry.kind)));
                }
                return false;
        }

        // ── the GPS ────────────────────────────────────────────────────────────────────
        if (sensors.gps.present) {
            const GpsConfig& g = sensors.gps;
            if (g.port < 1 || g.port > static_cast<std::uint8_t>(hal::kMaxSmartPorts)) {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap, "GPS port %d is not a port in 1..21 (0 = UNSET)",
                                  static_cast<int>(g.port));
                }
                return false;
            }
            if (!std::isfinite(g.leverArmForward.value())
                || !std::isfinite(g.leverArmLeft.value())) {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap,
                                  "GPS lever arm (%g forward, %g left) must be finite",
                                  g.leverArmForward.value(), g.leverArmLeft.value());
                }
                return false;
            }
            if (!claim(static_cast<int>(g.port), PortUse::Gps)) {
                return false;
            }
        }

        // ── the mechanisms ─────────────────────────────────────────────────────────────
        if (mechanismCount > static_cast<std::uint8_t>(kMaxMechanisms)) {
            if (why != nullptr && cap > 0) {
                std::snprintf(why, cap, "mechanismCount %d exceeds the capacity of %d",
                              static_cast<int>(mechanismCount), static_cast<int>(kMaxMechanisms));
            }
            return false;
        }
        // One owner per brain 3-wire line (addendum B2): the mechanism index + 1, 0 = free. A line
        // is not a smart port, so it has its own table; index 0 is unused so a line indexes directly.
        std::uint8_t lineOwner[9]{};
        for (std::size_t m = 0; m < mechanismCount; ++m) {
            const MechanismConfig& mech = mechanisms[m];
            if (mech.name[0] == '\0') {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap, "mechanism %d has an empty name", static_cast<int>(m));
                }
                return false;
            }
            // Checked BEFORE any message below prints the name with "%s" — an unterminated name
            // would be read past its 24 bytes — and because robot::Robot borrows it as a C string.
            if (!terminatedName(mech.name)) {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap,
                                  "mechanism %d's name has no NUL inside its %d bytes (23 "
                                  "characters at most)",
                                  static_cast<int>(m), static_cast<int>(sizeof mech.name));
                }
                return false;
            }
            if (mech.motorCount > static_cast<std::uint8_t>(kMaxMotorsPerMechanism)) {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap, "mechanism '%s' claims %d motors; the capacity is %d",
                                  mech.name, static_cast<int>(mech.motorCount),
                                  static_cast<int>(kMaxMotorsPerMechanism));
                }
                return false;
            }
            if (mech.pneumaticCount > static_cast<std::uint8_t>(kMaxPneumaticsPerMechanism)) {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap,
                                  "mechanism '%s' claims %d pneumatics; the capacity is %d",
                                  mech.name, static_cast<int>(mech.pneumaticCount),
                                  static_cast<int>(kMaxPneumaticsPerMechanism));
                }
                return false;
            }
            for (std::size_t i = 0; i < mech.motorCount; ++i) {
                const int a = hal::DrivetrainDescription::portMagnitude(mech.motorPorts[i]);
                if (a < 1 || a > static_cast<int>(hal::kMaxSmartPorts)) {
                    if (why != nullptr && cap > 0) {
                        std::snprintf(why, cap,
                                      "mechanism '%s' motor entry %d is not a port in 1..21",
                                      mech.name, static_cast<int>(mech.motorPorts[i]));
                    }
                    return false;
                }
                if (!claim(a, PortUse::Mechanism)) {
                    return false;
                }
            }
            for (std::size_t i = 0; i < mech.pneumaticCount; ++i) {
                const std::uint8_t line = mech.pneumaticPorts[i];
                if (line < 1 || line > 8) {
                    if (why != nullptr && cap > 0) {
                        std::snprintf(why, cap,
                                      "mechanism '%s' pneumatic entry %d is not an ADI line in 1..8",
                                      mech.name, static_cast<int>(line));
                    }
                    return false;
                }
                // Every earlier owner's name was checked (non-empty, terminated) on its own pass
                // of this loop, so printing it here is safe.
                const std::uint8_t earlier = lineOwner[line];
                if (earlier != 0) {
                    if (why != nullptr && cap > 0) {
                        const char letter = static_cast<char>('A' + line - 1);
                        if (earlier - 1U == m) {
                            std::snprintf(why, cap,
                                          "mechanism '%s' lists 3-wire port %c (line %d) twice",
                                          mech.name, letter, static_cast<int>(line));
                        } else {
                            std::snprintf(why, cap,
                                          "3-wire port %c (line %d) is claimed by mechanism '%s' "
                                          "and by mechanism '%s'",
                                          letter, static_cast<int>(line),
                                          mechanisms[earlier - 1U].name, mech.name);
                        }
                    }
                    return false;
                }
                lineOwner[line] = static_cast<std::uint8_t>(m + 1);
            }
        }
        // ── the mechanisms, second pass (chunk T3): what a BUILT mechanism needs ─────────
        // AFTER every port above is claimed, on purpose: a config with a collision AND an unset
        // cartridge is refused for the collision, with the message it always had — the port
        // checks are the older rules and their sentences are pinned. Everything here is about a
        // mechanism robot::Robot will construct, so an entry with no motors is not asked for a
        // cartridge or a safe mode it would never use, and one with no lines is not asked for a
        // level (addendum B2).
        for (std::size_t m = 0; m < mechanismCount; ++m) {
            const MechanismConfig& mech = mechanisms[m];
            if (mech.motorCount > 0 && mech.pneumaticCount > 0) {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap,
                                  "mechanism '%s' has %d motors AND %d 3-wire lines -- make them "
                                  "two mechanisms: one name is one motor mechanism or one "
                                  "pneumatic mechanism",
                                  mech.name, static_cast<int>(mech.motorCount),
                                  static_cast<int>(mech.pneumaticCount));
                }
                return false;
            }
            if (mech.motorCount > 0) {
                if (!validCartridge(mech.cartridge, nullptr, 0)) {
                    if (why != nullptr && cap > 0) {
                        std::snprintf(why, cap,
                                      "mechanism '%s' cartridge byte %d is not Red, Green or Blue "
                                      "(0 = UNSET) -- state the gearset in its motors",
                                      mech.name,
                                      static_cast<int>(static_cast<std::uint8_t>(mech.cartridge)));
                    }
                    return false;
                }
                if (!isSafeBrakeMode(mech.safeMode)) {
                    if (why != nullptr && cap > 0) {
                        std::snprintf(why, cap,
                                      "mechanism '%s' safe brake mode byte %d is not Coast, Brake "
                                      "or Hold (0 = UNSET) -- there is no default: at Coast a "
                                      "loaded mechanism drops",
                                      mech.name,
                                      static_cast<int>(static_cast<std::uint8_t>(mech.safeMode)));
                    }
                    return false;
                }
            }
            if (mech.pneumaticCount > 0) {
                if (!isPneumaticLevel(mech.pneumaticSafe)) {
                    if (why != nullptr && cap > 0) {
                        std::snprintf(why, cap,
                                      "mechanism '%s' pneumatic safe level byte %d is not Low or "
                                      "High (0 = UNSET) -- there is no default: which level drops "
                                      "what a claw holds depends on the valve",
                                      mech.name,
                                      static_cast<int>(
                                          static_cast<std::uint8_t>(mech.pneumaticSafe)));
                    }
                    return false;
                }
                if (!isPneumaticLevel(mech.pneumaticStart)) {
                    if (why != nullptr && cap > 0) {
                        std::snprintf(why, cap,
                                      "mechanism '%s' pneumatic start level byte %d is not Low or "
                                      "High (0 = UNSET) -- state it: the robot DRIVES the line to "
                                      "it when it is built",
                                      mech.name,
                                      static_cast<int>(
                                          static_cast<std::uint8_t>(mech.pneumaticStart)));
                    }
                    return false;
                }
            }
            for (std::size_t earlier = 0; earlier < m; ++earlier) {
                if (sameName(mechanisms[earlier].name, mech.name)) {
                    if (why != nullptr && cap > 0) {
                        std::snprintf(why, cap,
                                      "mechanisms %d and %d are both named '%s' -- a name is the "
                                      "key a mechanism is looked up by",
                                      static_cast<int>(earlier), static_cast<int>(m), mech.name);
                    }
                    return false;
                }
            }
        }
        return true;
    }

    /// True when `name` has a NUL inside its 24 bytes — the precondition for reading it as a C
    /// string at all.
    [[nodiscard]] static constexpr bool terminatedName(const char (&name)[24]) noexcept {
        for (const char ch : name) {
            if (ch == '\0') {
                return true;
            }
        }
        return false;
    }

    /// Two TERMINATED mechanism names, compared as C strings (bytes after the NUL are ignored, so
    /// a name typed into a reused buffer still compares by what it says).
    [[nodiscard]] static constexpr bool sameName(const char (&a)[24], const char (&b)[24]) noexcept {
        for (std::size_t i = 0; i < 24; ++i) {
            if (a[i] != b[i]) {
                return false;
            }
            if (a[i] == '\0') {
                return true;
            }
        }
        return true;
    }

    /// The one-line banner: identity, drivetrain kind and source, the geometry that governs, the
    /// estimator, and which optional devices are present. Truncates rather than overflowing;
    /// writes at least a NUL whenever cap > 0. Only meaningful on a `valid()` config.
    void describe(char* buf, std::size_t cap) const {
        if (buf == nullptr || cap == 0) {
            return;
        }
        const char* name = identity.name[0] != '\0' ? identity.name : "(unnamed)";
        switch (drivetrain.kind) {
            case DrivetrainKind::Tank: {
                const hal::DrivetrainDescription& d = drivetrain.tank.description;
                std::snprintf(buf, cap,
                              "%s [%s] tank %s: L%d/R%d ports, W %.3f in, %s odom, GPS %s",
                              name, identity.team,
                              d.source == hal::DescriptionSource::RobotMeasured ? "MEASURED"
                                                                                : "TYPED",
                              static_cast<int>(d.left.count), static_cast<int>(d.right.count),
                              d.trackWidth.value(), odometryKindWord(odometry.kind),
                              sensors.gps.present ? "present" : "ABSENT");
                return;
            }
            case DrivetrainKind::XDrive:
                std::snprintf(buf, cap,
                              "%s [%s] x-drive TYPED: ports %d/%d/%d/%d, R %.3f in, %s odom, "
                              "GPS %s",
                              name, identity.team, static_cast<int>(drivetrain.xdrive.ports[0]),
                              static_cast<int>(drivetrain.xdrive.ports[1]),
                              static_cast<int>(drivetrain.xdrive.ports[2]),
                              static_cast<int>(drivetrain.xdrive.ports[3]),
                              drivetrain.xdrive.driveRadius.value(),
                              odometryKindWord(odometry.kind),
                              sensors.gps.present ? "present" : "ABSENT");
                return;
            case DrivetrainKind::HDrive:
                std::snprintf(buf, cap,
                              "%s [%s] h-drive TYPED: ports %d/%d/%d, W %.3f in, %s odom, GPS %s",
                              name, identity.team, static_cast<int>(drivetrain.hdrive.ports[0]),
                              static_cast<int>(drivetrain.hdrive.ports[1]),
                              static_cast<int>(drivetrain.hdrive.ports[2]),
                              drivetrain.hdrive.trackWidth.value(),
                              odometryKindWord(odometry.kind),
                              sensors.gps.present ? "present" : "ABSENT");
                return;
        }
        std::snprintf(buf, cap, "%s [%s] UNSET drivetrain kind %d", name, identity.team,
                      static_cast<int>(static_cast<std::uint8_t>(drivetrain.kind)));
    }

    /// The cartridge check, shared by the two holonomic branches (the tank's lives inside
    /// `hal::DrivetrainDescription::valid()`, where it already was).
    [[nodiscard]] static bool validCartridge(hal::MotorCartridge c, char* why,
                                             std::size_t cap) noexcept {
        if (c == hal::MotorCartridge::Red || c == hal::MotorCartridge::Green
            || c == hal::MotorCartridge::Blue) {
            return true;
        }
        if (why != nullptr && cap > 0) {
            std::snprintf(why, cap, "cartridge byte %d is not Red, Green or Blue (UNSET is 0)",
                          static_cast<int>(static_cast<std::uint8_t>(c)));
        }
        return false;
    }

    /// The IMU-port range check, shared by the two holonomic branches.
    [[nodiscard]] static bool validImuPort(std::uint8_t port, char* why, std::size_t cap) noexcept {
        if (port >= 1 && port <= static_cast<std::uint8_t>(hal::kMaxSmartPorts)) {
            return true;
        }
        if (why != nullptr && cap > 0) {
            std::snprintf(why, cap, "IMU port %d is not a port in 1..21 (0 = UNSET)",
                          static_cast<int>(port));
        }
        return false;
    }

    /// One holonomic wheel port's range check. `0` (no port) passes HERE on purpose — see the
    /// "what it deliberately does not check" note on valid(); only a number that is not a port at
    /// all is refused. `kindWord` and `wheel` are what make the message point at a field.
    [[nodiscard]] static bool validWheelPort(std::int8_t port, const char* kindWord,
                                             std::size_t wheel, char* why,
                                             std::size_t cap) noexcept {
        if (port == 0) {
            return true;
        }
        const int a = hal::DrivetrainDescription::portMagnitude(port);
        if (a >= 1 && a <= static_cast<int>(hal::kMaxSmartPorts)) {
            return true;
        }
        if (why != nullptr && cap > 0) {
            std::snprintf(why, cap, "%s wheel %d entry %d is not a port in 1..21", kindWord,
                          static_cast<int>(wheel), static_cast<int>(port));
        }
        return false;
    }

    /// One tracking wheel's own checks (port set and in range, diameter finite and > 0, offset
    /// finite). `role` names the slot in the message.
    [[nodiscard]] static bool validPod(const TrackingWheelConfig& pod, const char* role, char* why,
                                       std::size_t cap) noexcept {
        if (pod.port < 1 || pod.port > static_cast<std::uint8_t>(hal::kMaxSmartPorts)) {
            if (why != nullptr && cap > 0) {
                std::snprintf(why, cap, "%s tracking wheel port %d is not a port in 1..21 "
                                        "(0 = UNSET)",
                              role, static_cast<int>(pod.port));
            }
            return false;
        }
        const double d = pod.wheelDiameter.value();
        if (!std::isfinite(d) || d <= 0.0) {
            if (why != nullptr && cap > 0) {
                std::snprintf(why, cap,
                              "%s tracking wheel diameter is %g -- must be finite and > 0 "
                              "(0 = UNSET)",
                              role, d);
            }
            return false;
        }
        if (!std::isfinite(pod.offset.value())) {
            if (why != nullptr && cap > 0) {
                std::snprintf(why, cap, "%s tracking wheel offset is %g -- must be finite", role,
                              pod.offset.value());
            }
            return false;
        }
        return true;
    }
};

/// The config seam (master plan decision #10, name kept): where a robot's description comes from.
/// ONE virtual call, made once at boot — a generated header, a card read at boot, or a constant
/// typed by hand all look the same to `robot::Robot` through this.
class IRobotConfig {
public:
    /// Polymorphic-base boilerplate: the destructor is virtual so a source held as
    /// `IRobotConfig&`/`*` destroys correctly, and DECLARING it suppresses the implicit copy/move,
    /// which are re-defaulted below. The base carries no state.
    virtual ~IRobotConfig() = default;
    /// Default-constructible: the base has nothing to initialize.
    IRobotConfig() = default;
    /// Copyable: the base holds no state, so a derived type decides what copying means.
    IRobotConfig(const IRobotConfig&) = default;
    /// Movable, same reasoning as the copy constructor.
    IRobotConfig(IRobotConfig&&) = default;
    /// Copy-assignable, same reasoning.
    IRobotConfig& operator=(const IRobotConfig&) = default;
    /// Move-assignable, same reasoning.
    IRobotConfig& operator=(IRobotConfig&&) = default;

    /// This robot's description. The reference must stay valid for as long as the caller uses it —
    /// every implementation in the tree returns a reference to storage it owns, never a temporary.
    [[nodiscard]] virtual const RobotConfig& config() const noexcept = 0;
};

/// An `IRobotConfig` over one config held BY VALUE — the implementation a generated header, a
/// hand-typed constant or a card read at boot all end up using. Holding by value rather than by
/// reference is deliberate: a boot sequence that decodes a record into a local and then hands out
/// a reference to it is the dangling-reference bug this type exists to make impossible.
class StaticRobotConfig final : public IRobotConfig {
public:
    /// Copy `cfg` into this source. The copy is the point; nothing is borrowed.
    explicit StaticRobotConfig(const RobotConfig& cfg) noexcept : cfg_{cfg} {}

    /// The held config.
    [[nodiscard]] const RobotConfig& config() const noexcept override { return cfg_; }

private:
    RobotConfig cfg_;
};

}  // namespace shulib::config
