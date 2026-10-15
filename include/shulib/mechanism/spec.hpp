#pragma once
//
// mechanism/spec.hpp — ONE MECHANISM AS DATA, and THE BUILD CHECK (the mechanism core, chunk
// MECH-A1, 2026-10-08).
//
// ── What this file is ───────────────────────────────────────────────────────────────
// The device-independent form every mechanism declaration becomes (mechanism/kinds.hpp's
// `spec()`): what a mechanism IS — its kind, its ROLES (motors, an angle sensor, a home switch, a
// piece sensor, 3-wire lines) and its numbers — and never which PORT. The ports are the wiring
// (mechanism/wiring.hpp), a separate block of the robot file: re-cabling changes only that block
// (the team lead, 2026-09-28: "mechanisms should be port agnostic ... bound to ports after they are
// created"). The mechanism service, the Robot's projection, the direction check, the route ids
// (mechanism/route_ids.hpp) and the robot panel's agreement test (G3a) all read THIS form.
//
// Every type here is a constexpr aggregate with fixed-width fields and bounded arrays — no
// allocation, no pointer the build cannot see through except the recorded file names — so the
// robot file's generator (G3a) can emit it and the COMPILER can check it.
//
// ── EVERY FIELD HAS AN UNSET VALUE, and nothing the team owns is invented (P6) ──────
// A field nobody stated is the byte 0 (or 0.0) and is REFUSED by specRefusal(), naming it. The
// three kinds of number that are NOT the team's — the controllers' starting gains, the smooth
// move's starting speed and acceleration, and the "reached" tolerance — are the library's STARTING
// values (kinds.hpp's positionStart(); PROVISIONAL, the hardware assumption the brief holds as
// HA-182 for MECH-A3, which registers it with the behaviours that run them).
//
// ── THE BUILD CHECK, three ways, one list ───────────────────────────────────────────
// specRefusal() is THE check: the FIRST thing wrong with one spec, as a `Refusal`, checked in a
// fixed order (one property per reason, so a test can pin the order). Each reason has:
//   * a SENTENCE (refusalText) — for a run-time caller, a host test, and the robot panel's
//     validator, which mirrors this list by sentence (G3a; its agreement test proves it);
//   * a NAMED FUNCTION in namespace `refused` that is deliberately NOT constexpr — refuse() calls
//     it from a consteval context, so the build stops and GCC prints the reason as the function's
//     name: "call to non-'constexpr' function 'void shulib::mechanism::refused::
//     position_a_preset_is_outside_the_travel_limits()'". The robot file's line is GCC's first
//     "in 'constexpr' expansion of" note (the guide's "reading a refusal").
// Appended only, like every enum the robot file and the panel share: a new reason goes at the END,
// so G3a's mirror never renumbers.
//
// ── Units ───────────────────────────────────────────────────────────────────────────
// A position mechanism is in DEGREES of its output (an arm) or INCHES (a lift: a spool, a linkage, a
// measured table). EVERY GAIN is per OUTPUT-SHAFT DEGREE whatever the units — the loop runs on the
// output angle, where gravity's cosine lives; the limits, presets, tolerance and profile stay in
// the mechanism's units (angleDegToPosition / positionToAngleDeg convert). A speed mechanism is in
// RPM of its output.
//
// PROS-free, sim-free, allocation-free.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "shulib/config/robot_config.hpp"
#include "shulib/hal/drivetrain_description.hpp"

namespace shulib::mechanism {

/// The most mechanisms one robot may declare — the robot config's capacity (8).
inline constexpr std::size_t kMaxMechanisms = config::kMaxMechanisms;
/// The most MOTOR roles one mechanism may have — the config's per-mechanism motor capacity (4).
inline constexpr std::size_t kMaxMotorRoles = config::kMaxMotorsPerMechanism;
/// The most 3-wire LINE roles one pneumatic mechanism may have — the config's capacity (2).
inline constexpr std::size_t kMaxLineRoles = config::kMaxPneumaticsPerMechanism;
/// The most named presets one position or speed mechanism may have.
inline constexpr std::size_t kMaxPresets = 6;
/// The most named positions one pneumatic mechanism may have (two, or three on two lines).
inline constexpr std::size_t kMaxPneumaticPositions = 3;
/// The most points a lift's measured height table may have.
inline constexpr std::size_t kMaxTablePoints = 8;
/// A mechanism name's storage, NUL included — config::MechanismConfig::name's width (23 characters).
inline constexpr std::size_t kNameBytes = 24;

/// The four KINDS the core runs — a declaration type in kinds.hpp per behaviour (an Arm and a Lift
/// are both Position; a Flywheel is Speed). The byte 0 is UNSET.
enum class Kind : std::uint8_t {
    Power = 1,      ///< run at a voltage (intakes, rollers, conveyors, robot two's claw pivots)
    Position = 2,   ///< go to a position and hold it (an Arm in degrees, a Lift in inches)
    Speed = 3,      ///< spin at an output RPM and say when it is "at speed" (a Flywheel)
    Pneumatic = 4,  ///< two or three named positions on 3-wire lines
};

/// A kind as a word for boot lines and refusals: "power", "position", "speed", "pneumatic", or
/// "UNSET" for any other byte.
[[nodiscard]] constexpr const char* kindWord(Kind k) noexcept {
    switch (k) {
        case Kind::Power:
            return "power";
        case Kind::Position:
            return "position";
        case Kind::Speed:
            return "speed";
        case Kind::Pneumatic:
            return "pneumatic";
    }
    return "UNSET";
}

/// What a SENSOR role is. Its port is the wiring's.
enum class SensorKind : std::uint8_t {
    None = 0,      ///< no such role on this mechanism
    Rotation = 1,  ///< a V5 rotation sensor (smart port) — an angle sensor
    Switch = 2,    ///< a limit switch / bumper (3-wire line)
    Distance = 3,  ///< a distance sensor as a piece sensor (smart port)
    Optical = 4,   ///< an optical sensor as a piece sensor (smart port)
};

/// One MOTOR role: how the motor is MOUNTED (its direction), never where it is plugged in.
struct MotorRole {
    /// The motor is mounted so that + turns the mechanism the other way (a mirrored lift motor):
    /// the projection makes its port NEGATIVE, and its adapter reverses it, once.
    bool reversed = false;
    /// WHERE this role's direction is stated, for the direction check's "change it HERE" (Q1 (a)):
    /// a GENERATED robot file's JSON path (G3a). nullptr in a hand-written file — the check then
    /// names `file`:`line`.
    const char* source = nullptr;
    /// The file the declaration was written in (std::source_location, captured by kinds.hpp at the
    /// team's own `.name` line) — the path the compiler opened, so host and brain differ (pin a
    /// suffix, never the whole path).
    const char* file = nullptr;
    /// The line of that `.name = ...` (exact: GCC 13 captures an initializer list at the
    /// declaration's closing line, so every role points at the `.name` line instead — measured).
    std::uint32_t line = 0;
};

/// One SENSOR role. Only the fields of its kind are read.
struct SensorRole {
    /// What it is; None = the mechanism has no such role.
    SensorKind kind = SensorKind::None;
    /// Rotation: mounted so that + reads the other way.
    bool reversed = false;
    /// Switch: the level that means PRESSED. `true` (1) everywhere today — that a VEX limit switch
    /// reads 1 when pressed is HA-184's claim.
    bool pressedLevel = true;
    /// Distance: an object closer than this many inches "sees a piece" (> 0).
    double seesWithinIn = 0.0;
    /// Optical: a proximity at or above this "sees a piece", in (0, 1].
    double seesProximity = 0.0;
    /// Where the role is stated in a GENERATED robot file (MotorRole::source); nullptr otherwise.
    const char* source = nullptr;
    /// The file the role's word (rotationSensor(), distanceSensor(), ...) was written in.
    const char* file = nullptr;
    /// The line it was written on.
    std::uint32_t line = 0;
};

/// A position mechanism's UNITS. The byte 0 is UNSET.
enum class Units : std::uint8_t {
    Degrees = 1,  ///< degrees of the output shaft (an Arm)
    Inches = 2,   ///< inches of travel (a Lift)
};

/// How a position mechanism's unit relates to its output shaft's angle. The byte 0 is UNSET.
enum class Geometry : std::uint8_t {
    Rotary = 1,  ///< degrees: the output angle itself (an arm)
    Spool = 2,   ///< inches: a spool of `spoolDiameterIn` on the output shaft
    Arm = 3,     ///< inches: height = pivotHeightIn + armStages * armLengthIn * sin(angle from level)
    Table = 4,   ///< inches: a measured table of (height in, output angle deg), strictly increasing
};

/// What holds a position mechanism up. The byte 0 is UNSET.
enum class Gravity : std::uint8_t {
    None = 1,      ///< nothing to hold: a turntable, a counterbalanced arm
    Cosine = 2,    ///< an arm: kG * cos(angle from level)
    Constant = 3,  ///< a lift: kG everywhere
};

/// How a position mechanism finds its zero. The byte 0 is UNSET.
enum class HomeMethod : std::uint8_t {
    None = 1,         ///< never homed: position moves are refused (NOT_HOMED); a held button jogs
    RestsAtStop = 2,  ///< it rests on its hard stop at program start: zero at bind, no motion
    Switch = 3,       ///< push at homeVolts until its home switch is pressed
    Stall = 4,        ///< push at homeVolts until its homing stall signature
    Absolute = 5,     ///< an absolute rotation sensor with a stated reading: known at bind (HA-186)
};

/// One named preset of a position or speed mechanism.
struct Preset {
    /// Its name (15 characters at most, NUL-terminated): "score".
    char name[16]{};
    /// Its value, in the mechanism's units (degrees, inches, or output rpm).
    double value = 0.0;
};

/// A STALL SIGNATURE as data — manipulation::StallConfig's three numbers. All 0 = none written. A
/// WRITTEN one (any of the three) must be whole and able to trip (specRefusal: amps above 0 and below
/// the motor's 2.5 A limit, an rpm above 0, a lasting above 0).
struct StallSpec {
    /// At least this current (A) ...
    double amps = 0.0;
    /// ... while the MOTOR shaft turns slower than this (rad/s) ...
    double speedRadS = 0.0;
    /// ... continuously for this long (s).
    double time = 0.0;
    /// Whether a signature was stated: its amps (a whole one — specRefusal refuses a written one
    /// without amps).
    [[nodiscard]] constexpr bool stated() const noexcept { return amps > 0.0; }
    /// Whether ANY of the three was written — what specRefusal checks a signature on, so one with a
    /// number left out is refused rather than read as "no signature".
    [[nodiscard]] constexpr bool written() const noexcept {
        return amps != 0.0 || speedRadS != 0.0 || time != 0.0;
    }
};

/// A POWER mechanism's numbers.
struct PowerSpec {
    /// The most any command may ask, (0, 12] V.
    double maxVolts = 0.0;
    /// Its jam signature — OPTIONAL on a power mechanism (every step is bounded by its time and the
    /// driver's hold-to-run by the finger; N10).
    StallSpec stall{};
};

/// A POSITION mechanism's numbers (an Arm or a Lift).
struct PositionSpec {
    /// Degrees or inches. UNSET refused.
    Units units = static_cast<Units>(0);
    /// How the units relate to the output angle. UNSET refused.
    Geometry geometry = static_cast<Geometry>(0);
    /// MOTOR turns per OUTPUT turn (> 0): gears(12, 60) is 5 (kinds.hpp — tooth counts, never a bare
    /// ratio, whose direction the drive geometry states the other way round).
    double gearRatio = 0.0;
    /// Spool: its diameter, inches.
    double spoolDiameterIn = 0.0;
    /// Arm (a linkage): one stage's length, pivot to pivot, inches.
    double armLengthIn = 0.0;
    /// Arm: the first pivot's height, inches.
    double pivotHeightIn = 0.0;
    /// Arm: 1 = a single arm or a four-bar; 2 = a double reverse four-bar; 3 the most (0 = UNSET).
    std::uint8_t armStages = 0;
    /// Table: {height in, output angle deg} rows, strictly increasing in both.
    double table[kMaxTablePoints][2]{};
    /// Table: how many rows of `table` are real.
    std::uint8_t tablePoints = 0;
    /// The travel limits, in the mechanism's units (REQUIRED, min < max).
    double minPosition = 0.0;
    /// The upper travel limit.
    double maxPosition = 0.0;
    /// The smooth move's top speed, units/s (> 0) — a STARTING value unless stated (header).
    double maxVelocity = 0.0;
    /// The smooth move's acceleration, units/s^2 (> 0) — a starting value unless stated.
    double maxAcceleration = 0.0;
    /// "Reached": within this of the target, in the mechanism's units (> 0) ...
    double tolerance = 0.0;
    /// ... continuously for this long, s (>= 0).
    double settleTime = 0.0;
    /// Proportional gain, V per output degree — a STARTING value unless stated (header).
    double kP = 0.0;
    /// Integral gain (the HOLD loop only), V per output degree-second.
    double kI = 0.0;
    /// Derivative gain, V per output degree/s.
    double kD = 0.0;
    /// Static friction feedforward, V (0 unless stated: the hold loop's integral covers it).
    double kS = 0.0;
    /// Velocity feedforward, V per output degree/s (12 V over the free speed unless stated).
    double kV = 0.0;
    /// Acceleration feedforward, V per output degree/s^2 (0 unless stated).
    double kA = 0.0;
    /// The gravity hold, V: at level (Cosine) or everywhere (Constant). The TEAM's number, within
    /// +-12 V (negative for a mechanism its elastics pull up harder than gravity pulls down).
    double kG = 0.0;
    /// What gravity does to it. UNSET refused.
    Gravity gravity = static_cast<Gravity>(0);
    /// Cosine: the output angle (deg) at which the arm is level.
    double horizontalAt = 0.0;
    /// How it finds zero. UNSET refused.
    HomeMethod home = static_cast<HomeMethod>(0);
    /// Where zero puts it, in its units (within the limits).
    double homePosition = 0.0;
    /// Switch / Stall: the gentle push toward the stop, V, signed (it must point at the home end),
    /// within +-12 V.
    double homeVolts = 0.0;
    /// Stall homing: the signature that means "at the stop".
    StallSpec homeStall{};
    /// A blocked move's or hold's signature — REQUIRED (N10).
    StallSpec stall{};
    /// A hold-to-run before homing runs at most this, V, 0..12 (0 = nothing moves it before homing).
    double unhomedVolts = 0.0;
    /// Absolute: the angle sensor's reading (deg, [0, 360)) when the mechanism is at homePosition.
    double absoluteSensorAtZeroDeg = 0.0;
    /// Its named presets, in its units.
    Preset presets[kMaxPresets]{};
    /// How many of `presets` are real.
    std::uint8_t presetCount = 0;
};

/// A SPEED mechanism's numbers (a Flywheel).
struct SpeedSpec {
    /// MOTOR turns per OUTPUT turn (> 0): gears(72, 12) is 1/6 (geared up).
    double gearRatio = 0.0;
    /// The fastest target accepted, output rpm (> 0) — the cartridge's free speed through the
    /// gearing, derived by kinds.hpp, never typed.
    double maxRpm = 0.0;
    /// Static feedforward, V.
    double kS = 0.0;
    /// Velocity feedforward, V per output rad/s (12 V over the free speed unless stated).
    double kV = 0.0;
    /// Acceleration feedforward, V per output rad/s^2.
    double kA = 0.0;
    /// Proportional gain on the speed error, V per rad/s — a STARTING value unless stated.
    double kP = 0.0;
    /// Integral gain on the speed error (acting near the target only — MECH-A3), V per rad.
    double kI = 0.0;
    /// "At speed": within this many output rpm of the target ...
    double atSpeedBandRpm = 0.0;
    /// ... continuously for this long, s (>= 0).
    double atSpeedTime = 0.0;
    /// Its named speeds, output rpm ("shoot" = 3000).
    Preset presets[kMaxPresets]{};
    /// How many of `presets` are real.
    std::uint8_t presetCount = 0;
    /// A jammed wheel's signature — REQUIRED (N10).
    StallSpec stall{};
};

/// What proves a pneumatic position was reached (Q2): nothing (SUCCEEDED after the move time), or
/// the piece sensor. A position that asks for the sensor on a mechanism WITHOUT one is counted done
/// after the move time and reported UNCONFIRMED (the team lead's Q2 (c)).
enum class Confirm : std::uint8_t {
    None = 0,         ///< nothing confirms it
    Sees = 1,         ///< a GRAB: the piece sensor sees a piece
    StopsSeeing = 2,  ///< a RELEASE: the piece sensor stops seeing one
};

/// One named position of a pneumatic mechanism.
struct PneumaticPositionSpec {
    /// Its name (15 characters at most): "closed".
    char name[16]{};
    /// The level of each line in this position, line 1 first (Q13 (a): the team states them).
    bool levels[kMaxLineRoles]{};
    /// What confirms reaching it.
    Confirm confirm = Confirm::None;
    /// How many levels the team WROTE — the wiring refuses a count that is not its line count (U2).
    std::uint8_t levelCount = 0;
};

/// A PNEUMATIC mechanism's numbers.
struct PneumaticSpec {
    /// Its positions, in the order written.
    PneumaticPositionSpec positions[kMaxPneumaticPositions]{};
    /// How many are real: 2 or 3.
    std::uint8_t positionCount = 0;
    /// The position at program start (an index).
    std::uint8_t start = 0;
    /// The position a forced safe state commands (an index).
    std::uint8_t safe = 0;
    /// How long the cylinder takes, s (> 0; the team's — no default, P6).
    double actuationTime = 0.0;
    /// With a piece sensor: how long after the move time a confirmation may take, s (>= 0).
    double confirmWindow = 0.0;
    /// WHERE the positions' levels are stated in a GENERATED robot file (its JSON path), for the
    /// direction check's "the levels are swapped: change them HERE"; nullptr in a hand-written file.
    const char* levelsSource = nullptr;
    /// The file the declaration was written in (captured at its `.name` line).
    const char* levelsFile = nullptr;
    /// That line.
    std::uint32_t levelsLine = 0;
    /// Whether the declaration's `start` named one of its positions.
    bool startNamed = true;
    /// Whether its `safe` named one of its positions.
    bool safeNamed = true;
};

/// ONE MECHANISM, as data (header).
struct MechanismSpec {
    /// Its name — boot lines, logs, the route ids' stem, and the run-time lookup's key.
    char name[kNameBytes]{};
    /// Which behaviour runs it. UNSET refused.
    Kind kind = static_cast<Kind>(0);
    /// Its motor roles, in order (a motor kind).
    MotorRole motors[kMaxMotorRoles]{};
    /// How many motor roles are real (1..4 for a motor kind; 0 for a pneumatic).
    std::uint8_t motorCount = 0;
    /// How many 3-wire line roles (1..2 for a pneumatic; 0 otherwise) — the WIRING's count.
    std::uint8_t lineCount = 0;
    /// Position / Speed: a Rotation role on its output shaft.
    SensorRole angleSensor{};
    /// Position: a Switch role (implied by switch homing).
    SensorRole homeSwitch{};
    /// Any kind: a Distance, Optical or Switch piece sensor.
    SensorRole pieceSensor{};
    /// A motor kind's cartridge — its own, never the drive's. UNSET refused.
    hal::MotorCartridge cartridge = static_cast<hal::MotorCartridge>(0);
    /// A motor kind's declared safe brake mode — what every stop lands in. UNSET refused.
    config::SafeBrakeMode safeMode = static_cast<config::SafeBrakeMode>(0);
    /// Power numbers (read when kind is Power).
    PowerSpec power{};
    /// Position numbers (read when kind is Position).
    PositionSpec position{};
    /// Speed numbers (read when kind is Speed).
    SpeedSpec speed{};
    /// Pneumatic numbers (read when kind is Pneumatic).
    PneumaticSpec pneumatic{};
    /// Where the declaration was written (its `.name = ...` line), for boot lines and the
    /// direction check — the path the compiler opened (pin a suffix, never the whole path).
    const char* declaredFile = nullptr;
    /// That line.
    std::uint32_t declaredLine = 0;
    /// Set by kinds.hpp when a preset or position name runs past 15 characters (its copy was cut).
    bool presetNameTooLong = false;
};

/// A spec's name as a view (up to its NUL, at most kNameBytes).
[[nodiscard]] constexpr std::string_view nameOf(const MechanismSpec& s) noexcept {
    std::size_t n = 0;
    while (n < kNameBytes && s.name[n] != '\0') {
        ++n;
    }
    return std::string_view{s.name, n};
}

/// Output degrees -> the mechanism's units, for a Position spec. Rotary: the angle; Spool: arc
/// length; Arm: height from the sine of the angle above level; Table: linear between rows, clamped
/// at both ends. Pure; positionToAngleDeg is the inverse.
[[nodiscard]] inline double angleDegToPosition(const PositionSpec& p, double outDeg) noexcept {
    constexpr double kPi = 3.14159265358979323846;
    switch (p.geometry) {
        case Geometry::Rotary:
            return outDeg;
        case Geometry::Spool:
            return outDeg / 360.0 * kPi * p.spoolDiameterIn;
        case Geometry::Arm:
            return p.pivotHeightIn + static_cast<double>(p.armStages) * p.armLengthIn *
                                         std::sin((outDeg - p.horizontalAt) * kPi / 180.0);
        case Geometry::Table: {
            const auto n = static_cast<std::size_t>(p.tablePoints);
            if (n < 2) {
                return 0.0;
            }
            if (outDeg <= p.table[0][1]) {
                return p.table[0][0];
            }
            for (std::size_t i = 1; i < n; ++i) {
                if (outDeg <= p.table[i][1]) {
                    const double t = (outDeg - p.table[i - 1][1]) / (p.table[i][1] - p.table[i - 1][1]);
                    return p.table[i - 1][0] + t * (p.table[i][0] - p.table[i - 1][0]);
                }
            }
            return p.table[n - 1][0];
        }
    }
    return 0.0;
}

/// The mechanism's units -> output degrees (the inverse of angleDegToPosition). Arm: asin, valid
/// within +-90 deg of level — the asin argument is clamped, and specRefusal() refuses limits outside
/// the formula's range, where the height is not monotonic. Table: linear, clamped at both ends.
[[nodiscard]] inline double positionToAngleDeg(const PositionSpec& p, double pos) noexcept {
    constexpr double kPi = 3.14159265358979323846;
    switch (p.geometry) {
        case Geometry::Rotary:
            return pos;
        case Geometry::Spool:
            return pos / (kPi * p.spoolDiameterIn) * 360.0;
        case Geometry::Arm: {
            double s = (pos - p.pivotHeightIn) / (static_cast<double>(p.armStages) * p.armLengthIn);
            s = s < -1.0 ? -1.0 : (s > 1.0 ? 1.0 : s);
            return p.horizontalAt + std::asin(s) * 180.0 / kPi;
        }
        case Geometry::Table: {
            const auto n = static_cast<std::size_t>(p.tablePoints);
            if (n < 2) {
                return 0.0;
            }
            if (pos <= p.table[0][0]) {
                return p.table[0][1];
            }
            for (std::size_t i = 1; i < n; ++i) {
                if (pos <= p.table[i][0]) {
                    const double t = (pos - p.table[i - 1][0]) / (p.table[i][0] - p.table[i - 1][0]);
                    return p.table[i - 1][1] + t * (p.table[i][1] - p.table[i - 1][1]);
                }
            }
            return p.table[n - 1][1];
        }
    }
    return 0.0;
}

/// EVERY reason the build refuses ONE mechanism's declaration — one property each, in the order
/// specRefusal() checks them (header). APPENDED ONLY: G3a's validator mirrors this list by sentence.
enum class Refusal : std::uint8_t {
    None = 0,  ///< nothing is wrong
    AMechanismHasNoName,  ///< its name is empty
    AMechanismNameDoesNotFit23Characters,  ///< its name has no NUL inside 24 bytes
    AMechanismSKindIsUNSET,  ///< its kind byte is none of the four
    AMotorMechanismNeeds14MotorRoles,  ///< a motor kind with 0 or more than 4 motor roles
    AMotorMechanismHas3WireLineRolesMakeTwoMechanisms,  ///< a motor kind with 3-wire line roles
    AMotorMechanismSCartridgeIsUNSET,  ///< a motor kind with no cartridge
    AMotorMechanismSSafeBrakeModeIsUNSET,  ///< a motor kind with no safe brake mode
    APneumaticMechanismHasMotorRoles,  ///< a pneumatic with motor roles
    APneumaticMechanismNeeds12LineRoles,  ///< a pneumatic with 0 or more than 2 lines
    TheAngleSensorRoleIsNotARotationSensor,  ///< the angle-sensor slot holds another kind
    AnAngleSensorRoleOnAMechanismThatIsNotPositionOrSpeed,  ///< an angle sensor on a power or pneumatic
    TheHomeSwitchRoleIsNotALimitSwitch,  ///< the home-switch slot holds another kind
    AHomeSwitchRoleOnAMechanismThatIsNotPosition,  ///< a home switch on a mechanism that is not position
    APieceSensorCannotBeARotationSensor,  ///< a rotation sensor as a piece sensor
    ADistancePieceSensorNeedsItsSeesWithinDistance0,  ///< a distance piece sensor with no threshold
    AnOpticalPieceSensorNeedsItsProximityThresholdIn01,  ///< an optical piece sensor's threshold outside (0, 1]
    PowerMaxVoltsIsNotIn012,  ///< a power kind's maxVolts outside (0, 12]
    PositionUnitsUNSET,  ///< a position kind's units byte is unset
    PositionGearRatioMustBe0,  ///< a position kind with no gearing
    PositionARotaryMechanismIsInDegrees,  ///< a rotary geometry in inches
    PositionALiftIsInInches,  ///< a lift geometry in degrees
    PositionSpoolDiameterMustBe0,  ///< a spool with no diameter
    PositionArmLengthMustBe0,  ///< a linkage with no arm length
    PositionAnArmNeeds13Stages,  ///< a linkage with 0 or more than 3 stages
    PositionAnArmSTravelLimitsReachPastItsLinkageTheHeightFormul,  ///< a linkage whose limits reach past its formula
    PositionALookupTableNeeds28Points,  ///< a measured table with fewer than 2 or more than 8 points
    PositionTheLookupTableIsNotStrictlyIncreasing,  ///< a measured table not increasing in both columns
    PositionTravelLimitsMissingOrMinMax,  ///< no travel limits, or min >= max
    PositionTheProfileNeedsMaxVelocityAndMaxAcceleration0,  ///< a profile speed or acceleration that is not > 0
    PositionToleranceMustBe0,  ///< a 'reached' tolerance that is not > 0
    PositionGravityUNSET,  ///< a gravity kind byte that is unset
    PositionHomingMethodUNSET,  ///< a homing method byte that is unset
    PositionHomingBySwitchNeedsAHomeSwitchRole,  ///< switch homing with no home switch
    PositionAbsoluteHomingNeedsAnAngleSensorRotationRole,  ///< absolute homing with no angle sensor
    PositionHomingNeedsHomeVolts,  ///< a homing move with no push
    PositionStallHomingNeedsItsStallSignature,  ///< stall homing with no signature
    PositionTheHomePositionIsOutsideTheTravelLimits,  ///< a home position past the limits
    PositionMorePresetsThanFit,  ///< more presets than kMaxPresets
    PositionAPresetHasNoName,  ///< a position preset with no name
    PositionAPresetIsOutsideTheTravelLimits,  ///< a position preset past the limits
    SpeedGearRatioAndMaxRpmMustBe0,  ///< a speed kind with no gearing or free speed
    SpeedTheAtSpeedBandMustBe0,  ///< a speed kind with no at-speed band
    SpeedMorePresetsThanFit,  ///< more speed presets than kMaxPresets
    SpeedAPresetHasNoName,  ///< a speed preset with no name
    SpeedAPresetIsPastMaxRpm,  ///< a speed preset past the free speed
    Pneumatic2Or3Positions,  ///< a pneumatic with fewer than 2 or more than 3 positions
    PneumaticStartSafeIsNotOneOfItsPositions,  ///< a start or safe index past the positions
    PneumaticActuationTimeMustBe0,  ///< no move time
    PneumaticAPositionHasNoName,  ///< a pneumatic position with no name
    PneumaticTwoPositionsHaveTheSameLineLevels,  ///< two positions alike on every wired line
    PneumaticItsStartNamesNoneOfItsPositions,  ///< a `start` naming no position
    PneumaticItsSafeNamesNoneOfItsPositions,  ///< a `safe` naming no position
    APresetOrPositionNameDoesNotFit15Characters,  ///< a preset or position name past 15 characters
    TwoPresetsOrPositionsShareAName,  ///< two presets (or positions) with one name
    AbsoluteHomingTheSensorSReadingAtZeroMustBeIn0360,  ///< an absolute sensor reading outside [0, 360)
    PositionTheStallSignatureIsUNSET,  ///< an arm or a lift with no stall signature (N10)
    PositionHomingPushesAwayFromItsHomeEnd,  ///< a homing push pointing away from its home end (U4)
    SpeedTheStallSignatureIsUNSET,  ///< a flywheel with no stall signature (N10)
    AStallSignatureSAmpsMustBeBelowTheMotorS25ACurrentLimit,  ///< a stated signature at or above 2.5 A (N10: it could never trip)
    AStallSignatureNeedsBelowMotorRpmAbove0,  ///< a stated signature at 0 rpm (N10: it could never trip)
    AStallSignatureNeedsLastingAbove0,  ///< a stated signature with no `.lasting` (MECH-A1: it would trip on every start)
    AStallSignatureNeedsAmpsAbove0,  ///< a WRITTEN signature (rpm or lasting) with no amps (MECH-A1)
    PositionGeometryUNSET,  ///< a lift with no .shape (MECH-A1)
    PositionTheSettleTimeIsNegative,  ///< a negative settle time (MECH-A1 review: it was named as the tolerance)
    SpeedTheAtSpeedLastingIsNegative,  ///< a negative at-speed lasting (MECH-A1 review: it was named as the band)
    PneumaticTheConfirmWindowIsNegative,  ///< a negative confirmWithin (MECH-A1 review: it was named as the move time)
    PositionTheHomingPushIsPast12Volts,  ///< a homing push past 12 V either way (MECH-A1 review)
    PositionTheGravityHoldIsPast12Volts,  ///< a gravity hold past 12 V either way (MECH-A1 review)
    PositionUnhomedVoltsIsNotIn0To12,  ///< an unhomedVolts outside [0, 12] (MECH-A1 review)
};

/// The SENTENCE for a refusal — what a run-time caller logs, a host test pins, and the robot
/// panel's validator mirrors (G3a). nullptr for None.
[[nodiscard]] constexpr const char* refusalText(Refusal r) noexcept {
    switch (r) {
        case Refusal::None:
            return nullptr;
        case Refusal::AMechanismHasNoName:
            return "a mechanism has no name";
        case Refusal::AMechanismNameDoesNotFit23Characters:
            return "a mechanism name does not fit 23 characters";
        case Refusal::AMechanismSKindIsUNSET:
            return "a mechanism's kind is UNSET";
        case Refusal::AMotorMechanismNeeds14MotorRoles:
            return "a motor mechanism needs 1..4 motor roles";
        case Refusal::AMotorMechanismHas3WireLineRolesMakeTwoMechanisms:
            return "a motor mechanism has 3-wire line roles (make two mechanisms)";
        case Refusal::AMotorMechanismSCartridgeIsUNSET:
            return "a motor mechanism's cartridge is UNSET";
        case Refusal::AMotorMechanismSSafeBrakeModeIsUNSET:
            return "a motor mechanism's safe brake mode is UNSET";
        case Refusal::APneumaticMechanismHasMotorRoles:
            return "a pneumatic mechanism has motor roles";
        case Refusal::APneumaticMechanismNeeds12LineRoles:
            return "a pneumatic mechanism needs 1..2 line roles";
        case Refusal::TheAngleSensorRoleIsNotARotationSensor:
            return "the angle sensor role is not a rotation sensor";
        case Refusal::AnAngleSensorRoleOnAMechanismThatIsNotPositionOrSpeed:
            return "an angle sensor role on a mechanism that is not position or speed";
        case Refusal::TheHomeSwitchRoleIsNotALimitSwitch:
            return "the home switch role is not a limit switch";
        case Refusal::AHomeSwitchRoleOnAMechanismThatIsNotPosition:
            return "a home switch role on a mechanism that is not position";
        case Refusal::APieceSensorCannotBeARotationSensor:
            return "a piece sensor cannot be a rotation sensor";
        case Refusal::ADistancePieceSensorNeedsItsSeesWithinDistance0:
            return "a distance piece sensor needs its 'sees within' distance (> 0)";
        case Refusal::AnOpticalPieceSensorNeedsItsProximityThresholdIn01:
            return "an optical piece sensor needs its proximity threshold in (0, 1]";
        case Refusal::PowerMaxVoltsIsNotIn012:
            return "power: maxVolts is not in (0, 12]";
        case Refusal::PositionUnitsUNSET:
            return "position: units UNSET";
        case Refusal::PositionGearRatioMustBe0:
            return "position: gearRatio must be > 0";
        case Refusal::PositionARotaryMechanismIsInDegrees:
            return "position: a rotary mechanism is in degrees";
        case Refusal::PositionALiftIsInInches:
            return "position: a lift is in inches";
        case Refusal::PositionSpoolDiameterMustBe0:
            return "position: spool diameter must be > 0";
        case Refusal::PositionArmLengthMustBe0:
            return "position: arm length must be > 0";
        case Refusal::PositionAnArmNeeds13Stages:
            return "position: an arm needs 1..3 stages";
        case Refusal::PositionAnArmSTravelLimitsReachPastItsLinkageTheHeightFormul:
            return "position: an arm's travel limits reach past its linkage (the height formula has no inverse there)";
        case Refusal::PositionALookupTableNeeds28Points:
            return "position: a lookup table needs 2..8 points";
        case Refusal::PositionTheLookupTableIsNotStrictlyIncreasing:
            return "position: the lookup table is not strictly increasing";
        case Refusal::PositionTravelLimitsMissingOrMinMax:
            return "position: travel limits missing or min >= max";
        case Refusal::PositionTheProfileNeedsMaxVelocityAndMaxAcceleration0:
            return "position: the profile needs maxVelocity and maxAcceleration > 0";
        case Refusal::PositionToleranceMustBe0:
            return "position: tolerance must be > 0";
        case Refusal::PositionGravityUNSET:
            return "position: gravity UNSET";
        case Refusal::PositionHomingMethodUNSET:
            return "position: homing method UNSET";
        case Refusal::PositionHomingBySwitchNeedsAHomeSwitchRole:
            return "position: homing by switch needs a homeSwitch role";
        case Refusal::PositionAbsoluteHomingNeedsAnAngleSensorRotationRole:
            return "position: absolute homing needs an angleSensor (rotation) role";
        case Refusal::PositionHomingNeedsHomeVolts:
            return "position: homing needs homeVolts";
        case Refusal::PositionStallHomingNeedsItsStallSignature:
            return "position: stall homing needs its stall signature";
        case Refusal::PositionTheHomePositionIsOutsideTheTravelLimits:
            return "position: the home position is outside the travel limits";
        case Refusal::PositionMorePresetsThanFit:
            return "position: more presets than fit";
        case Refusal::PositionAPresetHasNoName:
            return "position: a preset has no name";
        case Refusal::PositionAPresetIsOutsideTheTravelLimits:
            return "position: a preset is outside the travel limits";
        case Refusal::SpeedGearRatioAndMaxRpmMustBe0:
            return "speed: gearRatio and maxRpm must be > 0";
        case Refusal::SpeedTheAtSpeedBandMustBe0:
            return "speed: the at-speed band must be > 0";
        case Refusal::SpeedMorePresetsThanFit:
            return "speed: more presets than fit";
        case Refusal::SpeedAPresetHasNoName:
            return "speed: a preset has no name";
        case Refusal::SpeedAPresetIsPastMaxRpm:
            return "speed: a preset is past maxRpm";
        case Refusal::Pneumatic2Or3Positions:
            return "pneumatic: 2 or 3 positions";
        case Refusal::PneumaticStartSafeIsNotOneOfItsPositions:
            return "pneumatic: start/safe is not one of its positions";
        case Refusal::PneumaticActuationTimeMustBe0:
            return "pneumatic: actuationTime must be > 0";
        case Refusal::PneumaticAPositionHasNoName:
            return "pneumatic: a position has no name";
        case Refusal::PneumaticTwoPositionsHaveTheSameLineLevels:
            return "pneumatic: two positions have the same line levels";
        case Refusal::PneumaticItsStartNamesNoneOfItsPositions:
            return "pneumatic: its start names none of its positions";
        case Refusal::PneumaticItsSafeNamesNoneOfItsPositions:
            return "pneumatic: its safe names none of its positions";
        case Refusal::APresetOrPositionNameDoesNotFit15Characters:
            return "a preset or position name does not fit 15 characters";
        case Refusal::TwoPresetsOrPositionsShareAName:
            return "two presets or positions share a name";
        case Refusal::AbsoluteHomingTheSensorSReadingAtZeroMustBeIn0360:
            return "absolute homing: the sensor's reading at zero must be in [0, 360)";
        case Refusal::PositionTheStallSignatureIsUNSET:
            return "position: the stall signature (.stall) is UNSET -- a held mechanism must never sit at stall current";
        case Refusal::PositionHomingPushesAwayFromItsHomeEnd:
            return "position: the homing push points away from its home end (home at the bottom limit pushes with -volts, at the top with +volts)";
        case Refusal::SpeedTheStallSignatureIsUNSET:
            return "speed: the stall signature (.stall) is UNSET -- a jammed wheel must never sit at stall current";
        case Refusal::AStallSignatureSAmpsMustBeBelowTheMotorS25ACurrentLimit:
            return "a stall signature's amps must be below the motor's 2.5 A current limit (a motor limited at 2.5 A never reads more: the signature would never trip)";
        case Refusal::AStallSignatureNeedsBelowMotorRpmAbove0:
            return "a stall signature needs belowMotorRpm above 0 (a stalled motor still reads a little creep: at 0 it would never trip)";
        case Refusal::AStallSignatureNeedsLastingAbove0:
            return "a stall signature needs .lasting above 0 (a motor starting from rest draws stall-grade current with its shaft still: at 0 every start would read as a stall)";
        case Refusal::AStallSignatureNeedsAmpsAbove0:
            return "a stall signature needs its amps above 0 (its rpm or its lasting was written, its amps was not: the signature would be ignored)";
        case Refusal::PositionGeometryUNSET:
            return "position: the geometry is UNSET -- a lift needs its .shape: spool(...), linkage(...) or measured(...)";
        case Refusal::PositionTheSettleTimeIsNegative:
            return "position: the settle time must not be negative";
        case Refusal::SpeedTheAtSpeedLastingIsNegative:
            return "speed: the at-speed lasting must not be negative";
        case Refusal::PneumaticTheConfirmWindowIsNegative:
            return "pneumatic: confirmWithin must not be negative";
        case Refusal::PositionTheHomingPushIsPast12Volts:
            return "position: the homing push (homeVolts) must be within 12 V either way";
        case Refusal::PositionTheGravityHoldIsPast12Volts:
            return "position: the gravity hold (kG) must be within 12 V either way";
        case Refusal::PositionUnhomedVoltsIsNotIn0To12:
            return "position: unhomedVolts must be from 0 to 12";
    }
    return "an unknown refusal";
}

/// The refusals as NAMED FUNCTIONS (header): deliberately not constexpr, so refuse() calling one in a
/// consteval context stops the build and GCC prints the function's name — the reason, in words.
namespace refused {
/// Refusal::AMechanismHasNoName's name at the robot file's line.
inline void a_mechanism_has_no_name() {}
/// Refusal::AMechanismNameDoesNotFit23Characters's name at the robot file's line.
inline void a_mechanism_name_does_not_fit_twenty_three_characters() {}
/// Refusal::AMechanismSKindIsUNSET's name at the robot file's line.
inline void a_mechanisms_kind_is_unset() {}
/// Refusal::AMotorMechanismNeeds14MotorRoles's name at the robot file's line.
inline void a_motor_mechanism_needs_one_to_four_motor_roles() {}
/// Refusal::AMotorMechanismHas3WireLineRolesMakeTwoMechanisms's name at the robot file's line.
inline void a_motor_mechanism_has_three_wire_line_roles_make_two_mechanisms() {}
/// Refusal::AMotorMechanismSCartridgeIsUNSET's name at the robot file's line.
inline void a_motor_mechanisms_cartridge_is_unset() {}
/// Refusal::AMotorMechanismSSafeBrakeModeIsUNSET's name at the robot file's line.
inline void a_motor_mechanisms_safe_brake_mode_is_unset() {}
/// Refusal::APneumaticMechanismHasMotorRoles's name at the robot file's line.
inline void a_pneumatic_mechanism_has_motor_roles() {}
/// Refusal::APneumaticMechanismNeeds12LineRoles's name at the robot file's line.
inline void a_pneumatic_mechanism_needs_one_or_two_line_roles() {}
/// Refusal::TheAngleSensorRoleIsNotARotationSensor's name at the robot file's line.
inline void the_angle_sensor_role_is_not_a_rotation_sensor() {}
/// Refusal::AnAngleSensorRoleOnAMechanismThatIsNotPositionOrSpeed's name at the robot file's line.
inline void an_angle_sensor_role_on_a_mechanism_that_is_not_position_or_speed() {}
/// Refusal::TheHomeSwitchRoleIsNotALimitSwitch's name at the robot file's line.
inline void the_home_switch_role_is_not_a_limit_switch() {}
/// Refusal::AHomeSwitchRoleOnAMechanismThatIsNotPosition's name at the robot file's line.
inline void a_home_switch_role_on_a_mechanism_that_is_not_position() {}
/// Refusal::APieceSensorCannotBeARotationSensor's name at the robot file's line.
inline void a_piece_sensor_cannot_be_a_rotation_sensor() {}
/// Refusal::ADistancePieceSensorNeedsItsSeesWithinDistance0's name at the robot file's line.
inline void a_distance_piece_sensor_needs_its_sees_within_distance_more_than_zero() {}
/// Refusal::AnOpticalPieceSensorNeedsItsProximityThresholdIn01's name at the robot file's line.
inline void an_optical_piece_sensor_needs_its_proximity_threshold_in_above_0_and_at_most_1() {}
/// Refusal::PowerMaxVoltsIsNotIn012's name at the robot file's line.
inline void power_maxvolts_is_not_in_above_0_and_at_most_12() {}
/// Refusal::PositionUnitsUNSET's name at the robot file's line.
inline void position_units_unset() {}
/// Refusal::PositionGearRatioMustBe0's name at the robot file's line.
inline void position_gearratio_must_be_more_than_zero() {}
/// Refusal::PositionARotaryMechanismIsInDegrees's name at the robot file's line.
inline void position_a_rotary_mechanism_is_in_degrees() {}
/// Refusal::PositionALiftIsInInches's name at the robot file's line.
inline void position_a_lift_is_in_inches() {}
/// Refusal::PositionSpoolDiameterMustBe0's name at the robot file's line.
inline void position_spool_diameter_must_be_more_than_zero() {}
/// Refusal::PositionArmLengthMustBe0's name at the robot file's line.
inline void position_arm_length_must_be_more_than_zero() {}
/// Refusal::PositionAnArmNeeds13Stages's name at the robot file's line.
inline void position_an_arm_needs_one_to_three_stages() {}
/// Refusal::PositionAnArmSTravelLimitsReachPastItsLinkageTheHeightFormul's name at the robot file's line.
inline void position_an_arms_travel_limits_reach_past_its_linkage_the_height_formula_has_no_inverse_there() {}
/// Refusal::PositionALookupTableNeeds28Points's name at the robot file's line.
inline void position_a_lookup_table_needs_two_to_eight_points() {}
/// Refusal::PositionTheLookupTableIsNotStrictlyIncreasing's name at the robot file's line.
inline void position_the_lookup_table_is_not_strictly_increasing() {}
/// Refusal::PositionTravelLimitsMissingOrMinMax's name at the robot file's line.
inline void position_travel_limits_missing_or_min_at_least_max() {}
/// Refusal::PositionTheProfileNeedsMaxVelocityAndMaxAcceleration0's name at the robot file's line.
inline void position_the_profile_needs_maxvelocity_and_maxacceleration_more_than_zero() {}
/// Refusal::PositionToleranceMustBe0's name at the robot file's line.
inline void position_tolerance_must_be_more_than_zero() {}
/// Refusal::PositionGravityUNSET's name at the robot file's line.
inline void position_gravity_unset() {}
/// Refusal::PositionHomingMethodUNSET's name at the robot file's line.
inline void position_homing_method_unset() {}
/// Refusal::PositionHomingBySwitchNeedsAHomeSwitchRole's name at the robot file's line.
inline void position_homing_by_switch_needs_a_homeswitch_role() {}
/// Refusal::PositionAbsoluteHomingNeedsAnAngleSensorRotationRole's name at the robot file's line.
inline void position_absolute_homing_needs_an_anglesensor_rotation_role() {}
/// Refusal::PositionHomingNeedsHomeVolts's name at the robot file's line.
inline void position_homing_needs_homevolts() {}
/// Refusal::PositionStallHomingNeedsItsStallSignature's name at the robot file's line.
inline void position_stall_homing_needs_its_stall_signature() {}
/// Refusal::PositionTheHomePositionIsOutsideTheTravelLimits's name at the robot file's line.
inline void position_the_home_position_is_outside_the_travel_limits() {}
/// Refusal::PositionMorePresetsThanFit's name at the robot file's line.
inline void position_more_presets_than_fit() {}
/// Refusal::PositionAPresetHasNoName's name at the robot file's line.
inline void position_a_preset_has_no_name() {}
/// Refusal::PositionAPresetIsOutsideTheTravelLimits's name at the robot file's line.
inline void position_a_preset_is_outside_the_travel_limits() {}
/// Refusal::SpeedGearRatioAndMaxRpmMustBe0's name at the robot file's line.
inline void speed_gearratio_and_its_free_speed_must_be_more_than_zero() {}
/// Refusal::SpeedTheAtSpeedBandMustBe0's name at the robot file's line.
inline void speed_the_at_speed_band_must_be_more_than_zero() {}
/// Refusal::SpeedMorePresetsThanFit's name at the robot file's line.
inline void speed_more_presets_than_fit() {}
/// Refusal::SpeedAPresetHasNoName's name at the robot file's line.
inline void speed_a_preset_has_no_name() {}
/// Refusal::SpeedAPresetIsPastMaxRpm's name at the robot file's line.
inline void speed_a_preset_is_past_its_free_speed() {}
/// Refusal::Pneumatic2Or3Positions's name at the robot file's line.
inline void pneumatic_two_or_three_positions() {}
/// Refusal::PneumaticStartSafeIsNotOneOfItsPositions's name at the robot file's line.
inline void pneumatic_start_or_safe_is_not_one_of_its_positions() {}
/// Refusal::PneumaticActuationTimeMustBe0's name at the robot file's line.
inline void pneumatic_actuationtime_must_be_more_than_zero() {}
/// Refusal::PneumaticAPositionHasNoName's name at the robot file's line.
inline void pneumatic_a_position_has_no_name() {}
/// Refusal::PneumaticTwoPositionsHaveTheSameLineLevels's name at the robot file's line.
inline void pneumatic_two_positions_have_the_same_line_levels() {}
/// Refusal::PneumaticItsStartNamesNoneOfItsPositions's name at the robot file's line.
inline void pneumatic_its_start_names_none_of_its_positions() {}
/// Refusal::PneumaticItsSafeNamesNoneOfItsPositions's name at the robot file's line.
inline void pneumatic_its_safe_names_none_of_its_positions() {}
/// Refusal::APresetOrPositionNameDoesNotFit15Characters's name at the robot file's line.
inline void a_preset_or_position_name_does_not_fit_fifteen_characters() {}
/// Refusal::TwoPresetsOrPositionsShareAName's name at the robot file's line.
inline void two_presets_or_positions_share_a_name() {}
/// Refusal::AbsoluteHomingTheSensorSReadingAtZeroMustBeIn0360's name at the robot file's line.
inline void absolute_homing_the_sensors_reading_at_zero_must_be_in_from_0_up_to_360() {}
/// Refusal::PositionTheStallSignatureIsUNSET's name at the robot file's line.
inline void position_the_stall_signature_is_unset_a_held_mechanism_must_never_sit_at_stall_current() {}
/// Refusal::PositionHomingPushesAwayFromItsHomeEnd's name at the robot file's line.
inline void position_the_homing_push_points_away_from_its_home_end() {}
/// Refusal::SpeedTheStallSignatureIsUNSET's name at the robot file's line.
inline void speed_the_stall_signature_is_unset_a_jammed_wheel_must_never_sit_at_stall_current() {}
/// Refusal::AStallSignatureSAmpsMustBeBelowTheMotorS25ACurrentLimit's name at the robot file's line.
inline void a_stall_signatures_amps_must_be_below_the_motors_2_5_A_current_limit() {}
/// Refusal::AStallSignatureNeedsBelowMotorRpmAbove0's name at the robot file's line.
inline void a_stall_signature_needs_belowMotorRpm_above_0() {}
/// Refusal::AStallSignatureNeedsLastingAbove0's name at the robot file's line.
inline void a_stall_signature_needs_lasting_above_0() {}
/// Refusal::AStallSignatureNeedsAmpsAbove0's name at the robot file's line.
inline void a_stall_signature_needs_its_amps_above_0() {}
/// Refusal::PositionGeometryUNSET's name at the robot file's line.
inline void position_a_lifts_shape_is_unset_give_it_spool_linkage_or_measured() {}
/// Refusal::PositionTheSettleTimeIsNegative's name at the robot file's line.
inline void position_the_settle_time_must_not_be_negative() {}
/// Refusal::SpeedTheAtSpeedLastingIsNegative's name at the robot file's line.
inline void speed_the_at_speed_lasting_must_not_be_negative() {}
/// Refusal::PneumaticTheConfirmWindowIsNegative's name at the robot file's line.
inline void pneumatic_confirmwithin_must_not_be_negative() {}
/// Refusal::PositionTheHomingPushIsPast12Volts's name at the robot file's line.
inline void position_the_homing_push_must_be_within_12_volts_either_way() {}
/// Refusal::PositionTheGravityHoldIsPast12Volts's name at the robot file's line.
inline void position_the_gravity_hold_must_be_within_12_volts_either_way() {}
/// Refusal::PositionUnhomedVoltsIsNotIn0To12's name at the robot file's line.
inline void position_unhomedvolts_must_be_from_0_to_12() {}
}  // namespace refused

/// Stop the build with `r`'s named function (consteval contexts only); None does nothing.
consteval void refuse(Refusal r) {
    switch (r) {
        case Refusal::None:
            return;
        case Refusal::AMechanismHasNoName:
            refused::a_mechanism_has_no_name();
            return;
        case Refusal::AMechanismNameDoesNotFit23Characters:
            refused::a_mechanism_name_does_not_fit_twenty_three_characters();
            return;
        case Refusal::AMechanismSKindIsUNSET:
            refused::a_mechanisms_kind_is_unset();
            return;
        case Refusal::AMotorMechanismNeeds14MotorRoles:
            refused::a_motor_mechanism_needs_one_to_four_motor_roles();
            return;
        case Refusal::AMotorMechanismHas3WireLineRolesMakeTwoMechanisms:
            refused::a_motor_mechanism_has_three_wire_line_roles_make_two_mechanisms();
            return;
        case Refusal::AMotorMechanismSCartridgeIsUNSET:
            refused::a_motor_mechanisms_cartridge_is_unset();
            return;
        case Refusal::AMotorMechanismSSafeBrakeModeIsUNSET:
            refused::a_motor_mechanisms_safe_brake_mode_is_unset();
            return;
        case Refusal::APneumaticMechanismHasMotorRoles:
            refused::a_pneumatic_mechanism_has_motor_roles();
            return;
        case Refusal::APneumaticMechanismNeeds12LineRoles:
            refused::a_pneumatic_mechanism_needs_one_or_two_line_roles();
            return;
        case Refusal::TheAngleSensorRoleIsNotARotationSensor:
            refused::the_angle_sensor_role_is_not_a_rotation_sensor();
            return;
        case Refusal::AnAngleSensorRoleOnAMechanismThatIsNotPositionOrSpeed:
            refused::an_angle_sensor_role_on_a_mechanism_that_is_not_position_or_speed();
            return;
        case Refusal::TheHomeSwitchRoleIsNotALimitSwitch:
            refused::the_home_switch_role_is_not_a_limit_switch();
            return;
        case Refusal::AHomeSwitchRoleOnAMechanismThatIsNotPosition:
            refused::a_home_switch_role_on_a_mechanism_that_is_not_position();
            return;
        case Refusal::APieceSensorCannotBeARotationSensor:
            refused::a_piece_sensor_cannot_be_a_rotation_sensor();
            return;
        case Refusal::ADistancePieceSensorNeedsItsSeesWithinDistance0:
            refused::a_distance_piece_sensor_needs_its_sees_within_distance_more_than_zero();
            return;
        case Refusal::AnOpticalPieceSensorNeedsItsProximityThresholdIn01:
            refused::an_optical_piece_sensor_needs_its_proximity_threshold_in_above_0_and_at_most_1();
            return;
        case Refusal::PowerMaxVoltsIsNotIn012:
            refused::power_maxvolts_is_not_in_above_0_and_at_most_12();
            return;
        case Refusal::PositionUnitsUNSET:
            refused::position_units_unset();
            return;
        case Refusal::PositionGearRatioMustBe0:
            refused::position_gearratio_must_be_more_than_zero();
            return;
        case Refusal::PositionARotaryMechanismIsInDegrees:
            refused::position_a_rotary_mechanism_is_in_degrees();
            return;
        case Refusal::PositionALiftIsInInches:
            refused::position_a_lift_is_in_inches();
            return;
        case Refusal::PositionSpoolDiameterMustBe0:
            refused::position_spool_diameter_must_be_more_than_zero();
            return;
        case Refusal::PositionArmLengthMustBe0:
            refused::position_arm_length_must_be_more_than_zero();
            return;
        case Refusal::PositionAnArmNeeds13Stages:
            refused::position_an_arm_needs_one_to_three_stages();
            return;
        case Refusal::PositionAnArmSTravelLimitsReachPastItsLinkageTheHeightFormul:
            refused::position_an_arms_travel_limits_reach_past_its_linkage_the_height_formula_has_no_inverse_there();
            return;
        case Refusal::PositionALookupTableNeeds28Points:
            refused::position_a_lookup_table_needs_two_to_eight_points();
            return;
        case Refusal::PositionTheLookupTableIsNotStrictlyIncreasing:
            refused::position_the_lookup_table_is_not_strictly_increasing();
            return;
        case Refusal::PositionTravelLimitsMissingOrMinMax:
            refused::position_travel_limits_missing_or_min_at_least_max();
            return;
        case Refusal::PositionTheProfileNeedsMaxVelocityAndMaxAcceleration0:
            refused::position_the_profile_needs_maxvelocity_and_maxacceleration_more_than_zero();
            return;
        case Refusal::PositionToleranceMustBe0:
            refused::position_tolerance_must_be_more_than_zero();
            return;
        case Refusal::PositionGravityUNSET:
            refused::position_gravity_unset();
            return;
        case Refusal::PositionHomingMethodUNSET:
            refused::position_homing_method_unset();
            return;
        case Refusal::PositionHomingBySwitchNeedsAHomeSwitchRole:
            refused::position_homing_by_switch_needs_a_homeswitch_role();
            return;
        case Refusal::PositionAbsoluteHomingNeedsAnAngleSensorRotationRole:
            refused::position_absolute_homing_needs_an_anglesensor_rotation_role();
            return;
        case Refusal::PositionHomingNeedsHomeVolts:
            refused::position_homing_needs_homevolts();
            return;
        case Refusal::PositionStallHomingNeedsItsStallSignature:
            refused::position_stall_homing_needs_its_stall_signature();
            return;
        case Refusal::PositionTheHomePositionIsOutsideTheTravelLimits:
            refused::position_the_home_position_is_outside_the_travel_limits();
            return;
        case Refusal::PositionMorePresetsThanFit:
            refused::position_more_presets_than_fit();
            return;
        case Refusal::PositionAPresetHasNoName:
            refused::position_a_preset_has_no_name();
            return;
        case Refusal::PositionAPresetIsOutsideTheTravelLimits:
            refused::position_a_preset_is_outside_the_travel_limits();
            return;
        case Refusal::SpeedGearRatioAndMaxRpmMustBe0:
            refused::speed_gearratio_and_its_free_speed_must_be_more_than_zero();
            return;
        case Refusal::SpeedTheAtSpeedBandMustBe0:
            refused::speed_the_at_speed_band_must_be_more_than_zero();
            return;
        case Refusal::SpeedMorePresetsThanFit:
            refused::speed_more_presets_than_fit();
            return;
        case Refusal::SpeedAPresetHasNoName:
            refused::speed_a_preset_has_no_name();
            return;
        case Refusal::SpeedAPresetIsPastMaxRpm:
            refused::speed_a_preset_is_past_its_free_speed();
            return;
        case Refusal::Pneumatic2Or3Positions:
            refused::pneumatic_two_or_three_positions();
            return;
        case Refusal::PneumaticStartSafeIsNotOneOfItsPositions:
            refused::pneumatic_start_or_safe_is_not_one_of_its_positions();
            return;
        case Refusal::PneumaticActuationTimeMustBe0:
            refused::pneumatic_actuationtime_must_be_more_than_zero();
            return;
        case Refusal::PneumaticAPositionHasNoName:
            refused::pneumatic_a_position_has_no_name();
            return;
        case Refusal::PneumaticTwoPositionsHaveTheSameLineLevels:
            refused::pneumatic_two_positions_have_the_same_line_levels();
            return;
        case Refusal::PneumaticItsStartNamesNoneOfItsPositions:
            refused::pneumatic_its_start_names_none_of_its_positions();
            return;
        case Refusal::PneumaticItsSafeNamesNoneOfItsPositions:
            refused::pneumatic_its_safe_names_none_of_its_positions();
            return;
        case Refusal::APresetOrPositionNameDoesNotFit15Characters:
            refused::a_preset_or_position_name_does_not_fit_fifteen_characters();
            return;
        case Refusal::TwoPresetsOrPositionsShareAName:
            refused::two_presets_or_positions_share_a_name();
            return;
        case Refusal::AbsoluteHomingTheSensorSReadingAtZeroMustBeIn0360:
            refused::absolute_homing_the_sensors_reading_at_zero_must_be_in_from_0_up_to_360();
            return;
        case Refusal::PositionTheStallSignatureIsUNSET:
            refused::position_the_stall_signature_is_unset_a_held_mechanism_must_never_sit_at_stall_current();
            return;
        case Refusal::PositionHomingPushesAwayFromItsHomeEnd:
            refused::position_the_homing_push_points_away_from_its_home_end();
            return;
        case Refusal::SpeedTheStallSignatureIsUNSET:
            refused::speed_the_stall_signature_is_unset_a_jammed_wheel_must_never_sit_at_stall_current();
            return;
        case Refusal::AStallSignatureSAmpsMustBeBelowTheMotorS25ACurrentLimit:
            refused::a_stall_signatures_amps_must_be_below_the_motors_2_5_A_current_limit();
            return;
        case Refusal::AStallSignatureNeedsBelowMotorRpmAbove0:
            refused::a_stall_signature_needs_belowMotorRpm_above_0();
            return;
        case Refusal::AStallSignatureNeedsLastingAbove0:
            refused::a_stall_signature_needs_lasting_above_0();
            return;
        case Refusal::AStallSignatureNeedsAmpsAbove0:
            refused::a_stall_signature_needs_its_amps_above_0();
            return;
        case Refusal::PositionGeometryUNSET:
            refused::position_a_lifts_shape_is_unset_give_it_spool_linkage_or_measured();
            return;
        case Refusal::PositionTheSettleTimeIsNegative:
            refused::position_the_settle_time_must_not_be_negative();
            return;
        case Refusal::SpeedTheAtSpeedLastingIsNegative:
            refused::speed_the_at_speed_lasting_must_not_be_negative();
            return;
        case Refusal::PneumaticTheConfirmWindowIsNegative:
            refused::pneumatic_confirmwithin_must_not_be_negative();
            return;
        case Refusal::PositionTheHomingPushIsPast12Volts:
            refused::position_the_homing_push_must_be_within_12_volts_either_way();
            return;
        case Refusal::PositionTheGravityHoldIsPast12Volts:
            refused::position_the_gravity_hold_must_be_within_12_volts_either_way();
            return;
        case Refusal::PositionUnhomedVoltsIsNotIn0To12:
            refused::position_unhomedvolts_must_be_from_0_to_12();
            return;
    }
}

/// THE BUILD CHECK (header): the FIRST thing wrong with `s`, or Refusal::None. constexpr, so a robot
/// file's list is checked at build (wire() calls refuse(specRefusal(spec))) and a host test runs the
/// same function at run time. The ORDER is pinned by a test, one property per reason.
[[nodiscard]] constexpr Refusal specRefusal(const MechanismSpec& s) noexcept {
    if (s.name[0] == '\0') {
        return Refusal::AMechanismHasNoName;
    }
    bool nul = false;
    for (std::size_t i = 0; i < kNameBytes; ++i) {
        nul = nul || s.name[i] == '\0';
    }
    if (!nul) {
        return Refusal::AMechanismNameDoesNotFit23Characters;
    }
    const bool motorKind = s.kind == Kind::Power || s.kind == Kind::Position || s.kind == Kind::Speed;
    if (!motorKind && s.kind != Kind::Pneumatic) {
        return Refusal::AMechanismSKindIsUNSET;
    }
    if (motorKind) {
        if (s.motorCount < 1 || s.motorCount > kMaxMotorRoles) {
            return Refusal::AMotorMechanismNeeds14MotorRoles;
        }
        if (s.lineCount != 0) {
            return Refusal::AMotorMechanismHas3WireLineRolesMakeTwoMechanisms;
        }
        if (!(s.cartridge == hal::MotorCartridge::Red || s.cartridge == hal::MotorCartridge::Green ||
              s.cartridge == hal::MotorCartridge::Blue)) {
            return Refusal::AMotorMechanismSCartridgeIsUNSET;
        }
        if (!config::isSafeBrakeMode(s.safeMode)) {
            return Refusal::AMotorMechanismSSafeBrakeModeIsUNSET;
        }
    } else {
        if (s.motorCount != 0) {
            return Refusal::APneumaticMechanismHasMotorRoles;
        }
        if (s.lineCount < 1 || s.lineCount > kMaxLineRoles) {
            return Refusal::APneumaticMechanismNeeds12LineRoles;
        }
    }
    // The sensor roles: each of the kind its slot names, with its own number stated (P6).
    if (s.angleSensor.kind != SensorKind::None && s.angleSensor.kind != SensorKind::Rotation) {
        return Refusal::TheAngleSensorRoleIsNotARotationSensor;
    }
    if (s.angleSensor.kind != SensorKind::None && s.kind != Kind::Position && s.kind != Kind::Speed) {
        return Refusal::AnAngleSensorRoleOnAMechanismThatIsNotPositionOrSpeed;
    }
    if (s.homeSwitch.kind != SensorKind::None && s.homeSwitch.kind != SensorKind::Switch) {
        return Refusal::TheHomeSwitchRoleIsNotALimitSwitch;
    }
    if (s.homeSwitch.kind != SensorKind::None && s.kind != Kind::Position) {
        return Refusal::AHomeSwitchRoleOnAMechanismThatIsNotPosition;
    }
    if (s.pieceSensor.kind == SensorKind::Rotation) {
        return Refusal::APieceSensorCannotBeARotationSensor;
    }
    if (s.pieceSensor.kind == SensorKind::Distance && !(s.pieceSensor.seesWithinIn > 0.0)) {
        return Refusal::ADistancePieceSensorNeedsItsSeesWithinDistance0;
    }
    if (s.pieceSensor.kind == SensorKind::Optical &&
        !(s.pieceSensor.seesProximity > 0.0 && s.pieceSensor.seesProximity <= 1.0)) {
        return Refusal::AnOpticalPieceSensorNeedsItsProximityThresholdIn01;
    }
    // EVERY WRITTEN stall signature — a power's, a position's, its homing's, a flywheel's — must be
    // whole (MECH-A1: a signature with its amps left out was read as "no signature"), able to trip,
    // and must not trip on a start (N10; the first and the last MECH-A1's). The V5 motor's
    // current limit is 2500 mA by default (vendored pros/motors.h :1255-1257): a limited motor never
    // reads more, so a signature at or above it never fires. StallDetector's "not turning" is
    // |speed| <= the threshold, so 0 needs an exact zero a stalled motor's encoder does not report
    // (HA-93's creep). And its window is `lasting`: at 0 it trips on the first qualifying sample
    // (stall_detector.hpp's own note: "wrong for most") — a motor starting from rest draws
    // stall-grade current with its shaft still, so a stall-HOME would zero on its own spin-up.
    for (const StallSpec* st : {&s.power.stall, &s.position.stall, &s.position.homeStall, &s.speed.stall}) {
        if (!st->written()) {
            continue;
        }
        if (!(st->amps > 0.0)) {
            return Refusal::AStallSignatureNeedsAmpsAbove0;
        }
        if (!(st->amps < 2.5)) {
            return Refusal::AStallSignatureSAmpsMustBeBelowTheMotorS25ACurrentLimit;
        }
        if (!(st->speedRadS > 0.0)) {
            return Refusal::AStallSignatureNeedsBelowMotorRpmAbove0;
        }
        if (!(st->time > 0.0)) {
            return Refusal::AStallSignatureNeedsLastingAbove0;
        }
    }
    if (s.kind == Kind::Power) {
        if (!(s.power.maxVolts > 0.0 && s.power.maxVolts <= 12.0)) {
            return Refusal::PowerMaxVoltsIsNotIn012;
        }
    }
    if (s.kind == Kind::Position) {
        const PositionSpec& p = s.position;
        if (p.units != Units::Degrees && p.units != Units::Inches) {
            return Refusal::PositionUnitsUNSET;
        }
        // MECH-A1: a lift with no .shape carries geometry 0, which every geometry check below would
        // pass — refused by its starting profile's 0 speed for the wrong reason, or accepted outright
        // with a stated .tuning.maxSpeed (and then every angle reads 0 inches).
        if (p.geometry != Geometry::Rotary && p.geometry != Geometry::Spool && p.geometry != Geometry::Arm &&
            p.geometry != Geometry::Table) {
            return Refusal::PositionGeometryUNSET;
        }
        if (!(p.gearRatio > 0.0)) {
            return Refusal::PositionGearRatioMustBe0;
        }
        if (p.geometry == Geometry::Rotary && p.units != Units::Degrees) {
            return Refusal::PositionARotaryMechanismIsInDegrees;
        }
        if (p.geometry != Geometry::Rotary && p.units != Units::Inches) {
            return Refusal::PositionALiftIsInInches;
        }
        if (p.geometry == Geometry::Spool && !(p.spoolDiameterIn > 0.0)) {
            return Refusal::PositionSpoolDiameterMustBe0;
        }
        if (p.geometry == Geometry::Arm && !(p.armLengthIn > 0.0)) {
            return Refusal::PositionArmLengthMustBe0;
        }
        if (p.geometry == Geometry::Arm && (p.armStages < 1 || p.armStages > 3)) {
            return Refusal::PositionAnArmNeeds13Stages;
        }
        // Written as "not inside", so a limit or a pivot height that is not a number is refused too
        // (every range check below is written this way — MECH-A1's review).
        if (p.geometry == Geometry::Arm &&
            !(p.minPosition >= p.pivotHeightIn - p.armStages * p.armLengthIn &&
              p.maxPosition <= p.pivotHeightIn + p.armStages * p.armLengthIn)) {
            return Refusal::PositionAnArmSTravelLimitsReachPastItsLinkageTheHeightFormul;
        }
        if (p.geometry == Geometry::Table) {
            if (p.tablePoints < 2 || p.tablePoints > kMaxTablePoints) {
                return Refusal::PositionALookupTableNeeds28Points;
            }
            for (std::size_t i = 1; i < p.tablePoints; ++i) {
                if (!(p.table[i][0] > p.table[i - 1][0]) || !(p.table[i][1] > p.table[i - 1][1])) {
                    return Refusal::PositionTheLookupTableIsNotStrictlyIncreasing;
                }
            }
        }
        if (!(p.minPosition < p.maxPosition)) {
            return Refusal::PositionTravelLimitsMissingOrMinMax;
        }
        if (!(p.maxVelocity > 0.0) || !(p.maxAcceleration > 0.0)) {
            return Refusal::PositionTheProfileNeedsMaxVelocityAndMaxAcceleration0;
        }
        if (!(p.tolerance > 0.0)) {
            return Refusal::PositionToleranceMustBe0;
        }
        // Each number is named on its own (MECH-A1's review: a negative settle time was refused as
        // "tolerance must be > 0" about a correct tolerance).
        if (!(p.settleTime >= 0.0)) {
            return Refusal::PositionTheSettleTimeIsNegative;
        }
        if (p.gravity != Gravity::None && p.gravity != Gravity::Cosine && p.gravity != Gravity::Constant) {
            return Refusal::PositionGravityUNSET;
        }
        // A motor is commanded within +-12 V; a hold past it is a slip (20 V for 2.0 V), never a
        // setting. Signed: a lift its elastics pull up holds with a NEGATIVE kG (MECH-A1's review).
        if (!(p.kG >= -12.0 && p.kG <= 12.0)) {
            return Refusal::PositionTheGravityHoldIsPast12Volts;
        }
        if (p.home != HomeMethod::None && p.home != HomeMethod::RestsAtStop && p.home != HomeMethod::Switch &&
            p.home != HomeMethod::Stall && p.home != HomeMethod::Absolute) {
            return Refusal::PositionHomingMethodUNSET;
        }
        if (p.home == HomeMethod::Switch && s.homeSwitch.kind != SensorKind::Switch) {
            return Refusal::PositionHomingBySwitchNeedsAHomeSwitchRole;
        }
        if (p.home == HomeMethod::Absolute && s.angleSensor.kind != SensorKind::Rotation) {
            return Refusal::PositionAbsoluteHomingNeedsAnAngleSensorRotationRole;
        }
        if ((p.home == HomeMethod::Switch || p.home == HomeMethod::Stall) && p.homeVolts == 0.0) {
            return Refusal::PositionHomingNeedsHomeVolts;
        }
        // The push is signed (toward the home end) and at most the motor's 12 V either way; a push
        // that is not a number is refused here too (MECH-A1's review).
        if (!(p.homeVolts >= -12.0 && p.homeVolts <= 12.0)) {
            return Refusal::PositionTheHomingPushIsPast12Volts;
        }
        if (p.home == HomeMethod::Stall && !p.homeStall.stated()) {
            return Refusal::PositionStallHomingNeedsItsStallSignature;
        }
        // A homing MOVE must push TOWARD its home end: home at the bottom limit pushes down (-), at
        // the top pushes up (+); a home strictly inside the range may push either way (a switch
        // mid-travel). Otherwise the push drives AWAY, unlimited (no limits before homing), and a
        // stall-home zeroes on the OPPOSITE stop (U4).
        if ((p.home == HomeMethod::Switch || p.home == HomeMethod::Stall) &&
            ((p.homePosition == p.minPosition && p.homeVolts > 0.0) ||
             (p.homePosition == p.maxPosition && p.homeVolts < 0.0))) {
            return Refusal::PositionHomingPushesAwayFromItsHomeEnd;
        }
        if (p.home != HomeMethod::None && !(p.homePosition >= p.minPosition && p.homePosition <= p.maxPosition)) {
            return Refusal::PositionTheHomePositionIsOutsideTheTravelLimits;
        }
        // The jog before homing is a magnitude cap: 0 (nothing moves it) up to the motor's 12 V
        // (MECH-A1's review: 40 V compiled, and the motor would have clamped it to full power).
        if (!(p.unhomedVolts >= 0.0 && p.unhomedVolts <= 12.0)) {
            return Refusal::PositionUnhomedVoltsIsNotIn0To12;
        }
        if (!p.stall.stated() || !(p.stall.time > 0.0)) {
            return Refusal::PositionTheStallSignatureIsUNSET;
        }
        if (p.presetCount > kMaxPresets) {
            return Refusal::PositionMorePresetsThanFit;
        }
        for (std::size_t i = 0; i < p.presetCount; ++i) {
            if (p.presets[i].name[0] == '\0') {
                return Refusal::PositionAPresetHasNoName;
            }
            if (!(p.presets[i].value >= p.minPosition && p.presets[i].value <= p.maxPosition)) {
                return Refusal::PositionAPresetIsOutsideTheTravelLimits;
            }
        }
    }
    if (s.kind == Kind::Speed) {
        if (!(s.speed.gearRatio > 0.0) || !(s.speed.maxRpm > 0.0)) {
            return Refusal::SpeedGearRatioAndMaxRpmMustBe0;
        }
        if (!(s.speed.atSpeedBandRpm > 0.0)) {
            return Refusal::SpeedTheAtSpeedBandMustBe0;
        }
        if (!(s.speed.atSpeedTime >= 0.0)) {
            return Refusal::SpeedTheAtSpeedLastingIsNegative;
        }
        if (!s.speed.stall.stated() || !(s.speed.stall.time > 0.0)) {
            return Refusal::SpeedTheStallSignatureIsUNSET;
        }
        if (s.speed.presetCount > kMaxPresets) {
            return Refusal::SpeedMorePresetsThanFit;
        }
        for (std::size_t i = 0; i < s.speed.presetCount; ++i) {
            if (s.speed.presets[i].name[0] == '\0') {
                return Refusal::SpeedAPresetHasNoName;
            }
            if (!(s.speed.presets[i].value >= -s.speed.maxRpm && s.speed.presets[i].value <= s.speed.maxRpm)) {
                return Refusal::SpeedAPresetIsPastMaxRpm;
            }
        }
    }
    if (s.kind == Kind::Pneumatic) {
        const PneumaticSpec& q = s.pneumatic;
        if (q.positionCount < 2 || q.positionCount > kMaxPneumaticPositions) {
            return Refusal::Pneumatic2Or3Positions;
        }
        if (q.start >= q.positionCount || q.safe >= q.positionCount) {
            return Refusal::PneumaticStartSafeIsNotOneOfItsPositions;
        }
        if (!(q.actuationTime > 0.0)) {
            return Refusal::PneumaticActuationTimeMustBe0;
        }
        if (!(q.confirmWindow >= 0.0)) {
            return Refusal::PneumaticTheConfirmWindowIsNegative;
        }
        for (std::size_t i = 0; i < q.positionCount; ++i) {
            if (q.positions[i].name[0] == '\0') {
                return Refusal::PneumaticAPositionHasNoName;
            }
            for (std::size_t j = i + 1; j < q.positionCount; ++j) {
                bool same = true;
                for (std::size_t l = 0; l < s.lineCount && l < kMaxLineRoles; ++l) {
                    same = same && q.positions[i].levels[l] == q.positions[j].levels[l];
                }
                if (same) {
                    return Refusal::PneumaticTwoPositionsHaveTheSameLineLevels;
                }
            }
        }
    }
    if (s.presetNameTooLong) {
        return Refusal::APresetOrPositionNameDoesNotFit15Characters;
    }
    if (s.kind == Kind::Position) {
        for (std::size_t i = 0; i < s.position.presetCount && i < kMaxPresets; ++i) {
            for (std::size_t j = i + 1; j < s.position.presetCount && j < kMaxPresets; ++j) {
                if (std::string_view{s.position.presets[i].name} == std::string_view{s.position.presets[j].name}) {
                    return Refusal::TwoPresetsOrPositionsShareAName;
                }
            }
        }
        if (s.position.home == HomeMethod::Absolute &&
            !(s.position.absoluteSensorAtZeroDeg >= 0.0 && s.position.absoluteSensorAtZeroDeg < 360.0)) {
            return Refusal::AbsoluteHomingTheSensorSReadingAtZeroMustBeIn0360;
        }
    }
    if (s.kind == Kind::Speed) {
        for (std::size_t i = 0; i < s.speed.presetCount && i < kMaxPresets; ++i) {
            for (std::size_t j = i + 1; j < s.speed.presetCount && j < kMaxPresets; ++j) {
                if (std::string_view{s.speed.presets[i].name} == std::string_view{s.speed.presets[j].name}) {
                    return Refusal::TwoPresetsOrPositionsShareAName;
                }
            }
        }
    }
    if (s.kind == Kind::Pneumatic) {
        if (!s.pneumatic.startNamed) {
            return Refusal::PneumaticItsStartNamesNoneOfItsPositions;
        }
        if (!s.pneumatic.safeNamed) {
            return Refusal::PneumaticItsSafeNamesNoneOfItsPositions;
        }
        for (std::size_t i = 0; i < s.pneumatic.positionCount && i < kMaxPneumaticPositions; ++i) {
            for (std::size_t j = i + 1; j < s.pneumatic.positionCount && j < kMaxPneumaticPositions; ++j) {
                if (std::string_view{s.pneumatic.positions[i].name} ==
                    std::string_view{s.pneumatic.positions[j].name}) {
                    return Refusal::TwoPresetsOrPositionsShareAName;
                }
            }
        }
    }
    return Refusal::None;
}

/// The same check as a SENTENCE (nullptr = valid) — run-time callers, host tests, G3a's agreement
/// test.
[[nodiscard]] constexpr const char* specProblem(const MechanismSpec& s) noexcept {
    return refusalText(specRefusal(s));
}

}  // namespace shulib::mechanism
