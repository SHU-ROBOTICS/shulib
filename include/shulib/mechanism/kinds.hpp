#pragma once
//
// mechanism/kinds.hpp — WHAT A MECHANISM IS, the way a team writes it (the mechanism core, chunk
// MECH-A1, 2026-10-08).
//
// ONE declaration per mechanism, in the team's own words — a name, its motors and which way each is
// mounted, its cartridge, its gearing as tooth counts, what it does when stopped and what a jam
// looks like, its limits and presets — with NO port anywhere: the ports are the wiring
// (mechanism/wiring.hpp), a separate block of the robot file (P2; the team lead's Q1 (a): the
// binding lives only there).
//
//     inline constexpr Arm arm{
//         .name = "arm",
//         .motors = {Forward},
//         .cartridge = Green,
//         .gearing = gears(12, 60),         // a 12-tooth gear on the motor drives a 60 on the arm
//         .safe = Hold,
//         .stall = {.amps = 2.0, .belowMotorRpm = 10, .lasting = 0.5_s},
//         .range = {0, 160},                // degrees: down on its stop .. just past vertical
//         .gravity = cosine(1.6_volt, 45),  // 1.6 V holds it out level; it is level at 45 degrees
//         .home = restsAtStop(0),           // it rests on its stop at 0 when the program starts
//         .presets = {{"stow", 0}, {"score", 95}, {"high", 140}},
//     };
//
// ── THE MEMBERS ARE WRITTEN IN ONE ORDER PER KIND — C++ requires it ──────────────────
// A designated initializer must follow the declaration order, so each kind's members are ordered
// the way a student reads them: what it IS (name, motors, cartridge, gearing, shape, angle or piece
// sensor), what it does STOPPED (safe, stall, and — since MECH-S-A — currentCap), where it GOES
// (range, gravity, home, presets — or at-speed and presets), the extras (unhomedVolts, tuning). Power's
// maxVolts comes after its currentCap. Out of order is a plain C++ error
// ("designator order ... does not match declaration order"); the guide prints each kind's order.
//
// ── THE DECLARATION IS THE HANDLE (Q18 (a): "checked at build") ──────────────────────
// An auton names `arm`, a preset `arm["score"]`, a constant target `arm.to(120)`. A misspelt or
// removed mechanism is "'arm' was not declared in this scope"; a misspelt preset, a constant past
// the travel limits, a speed past the free speed, a pneumatic position the mechanism does not have
// — each is a consteval refusal at the auton's own line, NAMED (detail::no_such_preset_on_this_
// mechanism, ...); and the type-shaped slips are named too: `arm.to(90_deg)`, `lift.to(18)`,
// `intake["fast"]`, `claw.to(...)`, `.gearing = 5`. Nothing is generated in the core: G3a's robot
// panel writes these same declarations.
//
// ── EVERY FIELD HAS AN UNSET VALUE, and a number the team owns is never invented (P6) ─
// A field left out is UNSET, and the robot's list (mechanisms(...), wiring.hpp) refuses it AT BUILD,
// naming it (spec.hpp's refusals). Exactly three kinds of value are NOT the team's and come from the
// library, each a STARTING value, PROVISIONAL: the controllers' gains, the smooth move's speed and
// acceleration (a fraction of the cartridge's free speed through the stated gearing), and the
// "reached" tolerance — the requirements' "gains ship with starting values and are MEASURED on the
// robot later" (N7). They are HA-182 (registered by MECH-A3, 2026-10-15, with the behaviours that run
// them — mechanism/service.hpp), INVENTED until each mechanism is tuned on the robot;
// startingValuesLine() (wiring.hpp) words them per mechanism for a boot line — the programs print it
// since the core's fifth part (MECH-A5) — so nobody mistakes them for measurements.
// Each can be stated instead, one by one (`.tuning`). (The service's own integral clamps and window
// are HA-182 too, but LIBRARY constants: no `.tuning` field, not on that line — service.hpp.) No time
// limit is ever a default: every step states its own.
//
// ── UNITS ────────────────────────────────────────────────────────────────────────────
// An Arm is in DEGREES of the arm — plain numbers: `_deg` is a WRAPPING heading (270 reads -90), so
// `arm.to(90_deg)` does not compile. A Lift is in INCHES (`_in`). A Flywheel is in RPM of its output.
// Gains are per OUTPUT-SHAFT DEGREE for every position mechanism (a lift's per degree of its spool).
// The GEARING is tooth counts, driving then driven: gears(12, 60) is a 5:1 reduction (the output
// turns once per 5 motor turns). Never a bare number — the drive geometry states its ratio the other
// way round (hal/drive_geometry.hpp `motorToWheelRatio`, wheel turns per motor turn), and a bare
// 5 or 1.0/5 would compile either way and be wrong by 25x (re-check U1).
//
// ── WHERE TO CHANGE IT ───────────────────────────────────────────────────────────────
// Every declaration records the file and line of its `.name = ...` (std::source_location, a
// defaulted argument nobody writes), so a boot line or the direction check can name the robot
// file's line with no string typed by hand; a GENERATED robot file states each role's JSON path in
// `changeAt` instead (G3a). The path is the one the compiler opened — it differs between the host
// and the brain, so a test pins its suffix.
//
// PROS-free, allocation-free; everything here is constexpr or consteval.

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <source_location>
#include <string_view>
#include <type_traits>

#include "shulib/config/robot_config.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/mechanism/spec.hpp"
#include "shulib/units/literals.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::mechanism {

// ═══ THE NAME — and where the declaration was written ═══════════════════════════════════

/// A mechanism's name ("claw pivot 6": boot lines, logs, the route ids' stem, the run-time lookup).
/// Written as a plain string (`.name = "arm"`); it also records the FILE and LINE it was written on,
/// which is how the direction check and the boot lines point at the declaration (header).
struct Name {
    /// The name's text, borrowed from the team's literal (null = UNSET).
    const char* text = nullptr;
    /// The file the `.name = ...` was written in (the path the compiler opened).
    const char* file = nullptr;
    /// The line it was written on.
    std::uint32_t line = 0;
    /// UNSET.
    constexpr Name() noexcept = default;
    /// `.name = "arm"`: the text, and where it was written (the defaulted `at` is the team's line).
    consteval Name(const char* s, std::source_location at = std::source_location::current()) noexcept
        : text{s}, file{at.file_name()}, line{at.line()} {}
};

// ═══ ROLES ════════════════════════════════════════════════════════════════════════════

/// One MOTOR role: how the motor is MOUNTED (which way + turns the mechanism), never its port.
struct Motor {
    /// Set by Forward / Reversed: a role the mechanism has.
    bool present = false;
    /// Mounted so that + turns the mechanism the other way.
    bool reversed = false;
    /// Generated robot files only: this role's JSON path (the direction check's "change it HERE").
    const char* changeAt = nullptr;
};
/// A motor mounted so that + moves the mechanism its "+" way (up, in, out — the team's word).
inline constexpr Motor Forward{.present = true};
/// A motor mounted the other way round (the mirror-image motor of a two-motor lift).
inline constexpr Motor Reversed{.present = true, .reversed = true};

/// A mechanism's MOTOR roles, in order: `.motors = {Forward, Reversed}` (one to four; a fifth is
/// recorded as "too many" and refused by the list).
struct Motors {
    /// The roles, in the order written.
    Motor m[kMaxMotorRoles]{};
    /// How many were written (at most kMaxMotorRoles).
    std::uint8_t count = 0;
    /// More than kMaxMotorRoles were written.
    bool tooMany = false;
    /// Where the list was written (GCC 13 captures an initializer list at the declaration's CLOSING
    /// line — measured — so the spec points every role at the `.name` line instead).
    const char* file = nullptr;
    /// That line.
    std::uint32_t line = 0;
    /// UNSET: no motors.
    constexpr Motors() noexcept = default;
    /// `.motors = {Forward, Reversed}`.
    consteval Motors(std::initializer_list<Motor> list,
                     std::source_location at = std::source_location::current()) noexcept
        : file{at.file_name()}, line{at.line()} {
        for (const Motor& x : list) {
            if (count == kMaxMotorRoles) {
                tooMany = true;
                break;
            }
            m[count++] = x;
        }
    }
};

/// One SENSOR role (an angle sensor, a piece sensor). Made by the words below, never by hand.
struct Sensor {
    /// What it is; None = no such role.
    SensorKind kind = SensorKind::None;
    /// Rotation: mounted so that + reads the other way.
    bool reversed = false;
    /// Switch: the level that means PRESSED — `true` (HA-184's claim).
    bool pressedLevel = true;
    /// Distance: closer than this many inches = "sees a piece".
    double seesWithinIn = 0.0;
    /// Optical: proximity at or above this = "sees a piece", (0, 1].
    double seesProximity = 0.0;
    /// Generated robot files only: this role's JSON path.
    const char* changeAt = nullptr;
    /// Where it was written (captured).
    const char* file = nullptr;
    /// That line.
    std::uint32_t line = 0;
};
/// A V5 ROTATION sensor on the mechanism's output shaft — its angle sensor. `mounted` Reversed when
/// + reads the other way.
[[nodiscard]] constexpr Sensor rotationSensor(Motor mounted = Forward,
                                              std::source_location at = std::source_location::current()) noexcept {
    return Sensor{.kind = SensorKind::Rotation, .reversed = mounted.reversed, .file = at.file_name(), .line = at.line()};
}
/// A DISTANCE sensor as a piece sensor: "sees a piece" when something is closer than `within`.
[[nodiscard]] constexpr Sensor distanceSensor(units::Length within,
                                              std::source_location at = std::source_location::current()) noexcept {
    return Sensor{.kind = SensorKind::Distance, .seesWithinIn = within.value(), .file = at.file_name(), .line = at.line()};
}
/// An OPTICAL sensor as a piece sensor: "sees a piece" at proximity >= `proximity`, in (0, 1].
[[nodiscard]] constexpr Sensor opticalSensor(double proximity,
                                             std::source_location at = std::source_location::current()) noexcept {
    return Sensor{.kind = SensorKind::Optical, .seesProximity = proximity, .file = at.file_name(), .line = at.line()};
}
/// A LIMIT SWITCH (or bumper) on a 3-wire port as a piece sensor: pressed = sees a piece.
[[nodiscard]] constexpr Sensor limitSwitch(std::source_location at = std::source_location::current()) noexcept {
    return Sensor{.kind = SensorKind::Switch, .file = at.file_name(), .line = at.line()};
}

/// A STALL SIGNATURE: the motors draw at least `amps` while turning slower than `belowMotorRpm` (at
/// the MOTOR shaft — every other rpm a team writes is the output's) for `lasting`
/// (manipulation::StallDetector's three numbers). The team's — never a default. REQUIRED on an Arm,
/// a Lift and a Flywheel (N10: their behaviour goes on after the step — a hold, a spin — and a
/// blocked one must never sit at stall current, P4); optional on a Power mechanism. A stated one
/// must be able to trip: amps below the motor's 2.5 A limit, belowMotorRpm above 0, lasting above 0.
struct Stall {
    /// At least this current, A (below 2.5).
    double amps = 0.0;
    /// While the MOTOR shaft turns slower than this, rpm (above 0).
    double belowMotorRpm = 0.0;
    /// Continuously for this long (above 0).
    units::Time lasting{0.0};
};

/// A CURRENT CAP (chunk MECH-S-A, 2026-11-05): the most current EACH motor of the mechanism may draw,
/// amperes — `.currentCap = {.amps = 1.2}`. Optional on a Power, an Arm, a Lift and a Flywheel; the
/// team's number, never a default (none written = the motor's own 2.5 A). Set in every motor at boot
/// and enforced by the MOTOR itself from then on — every command, the Hold brake, a homing push, a
/// blocking loop that starves the service (A4 register HA-219: how fast, unmeasured). PER MOTOR: two
/// motors on one spool pull with twice it. The build refuses one below 0 or above 2.5 A, one under
/// 0.001 A (a V5 limit is whole milliamps), and any stated stall signature at or above it (a capped
/// motor never draws more: the signature could never trip) — a cap written as 0 is no cap, the same as
/// leaving it out; the boot warns when a signature is not MORE than 0.2 A below it, as the motor reads it in whole milliamps (HA-223). Choosing it — the ORDER four
/// currents must keep: the heaviest HOLD (with a game piece) < the stall signature < the cap − 0.2 A,
/// the heaviest MOVE's peak < the cap, and the cap ≤ what the string can take (about 10 lbf per amp
/// per motor for a red cartridge on a 1.5 in spool — HA-222, NOT measured). It does not limit the
/// carriage's momentum when it hits a stop (HA-222): push gently.
struct CurrentCap {
    /// At most this current per motor, A: from 0.001 (one milliamp) to 2.5. 0 = no cap stated.
    double amps = 0.0;
};

namespace detail {
/// A consteval refusal (spec.hpp's idiom): a gear needs at least one tooth.
inline void a_gear_has_at_least_one_tooth() {}
}  // namespace detail

/// THE GEARING, as tooth counts (header): `gears(12, 60)` = a 12-tooth gear on the motor drives a 60;
/// `gears(60, 12)` = geared UP; stages multiply, `gears(12, 36) * gears(12, 60)`; `direct` = none.
/// Sprockets the same way (teeth). A bare number does not compile, and names the fix.
struct Gearing {
    /// Motor turns per output turn (> 0); 0 = UNSET.
    double motorTurnsPerTurn = 0.0;
    /// UNSET.
    constexpr Gearing() noexcept = default;
    /// `.gearing = 5`: refused. The template parameter's NAME is the reason GCC prints ("use of
    /// deleted function ... [with Write_the_gearing_as_tooth_counts_like_gears_12_60 = int]") —
    /// a consteval refusal inside a converting constructor printed no reason (measured, n41).
    template <class Write_the_gearing_as_tooth_counts_like_gears_12_60>
        requires std::is_arithmetic_v<Write_the_gearing_as_tooth_counts_like_gears_12_60>
    Gearing(Write_the_gearing_as_tooth_counts_like_gears_12_60) = delete;
    /// Two stages in series: their reductions multiply.
    [[nodiscard]] constexpr Gearing operator*(Gearing o) const noexcept {
        Gearing g;
        g.motorTurnsPerTurn = motorTurnsPerTurn * o.motorTurnsPerTurn;
        return g;
    }
};
/// A gear pair, DRIVING (on the motor's side) then DRIVEN, in teeth. Refuses a gear with no tooth.
[[nodiscard]] consteval Gearing gears(int driving, int driven) {
    if (driving < 1 || driven < 1) {
        detail::a_gear_has_at_least_one_tooth();
    }
    Gearing g;
    g.motorTurnsPerTurn = static_cast<double>(driven) / static_cast<double>(driving);
    return g;
}
/// No gearing: the output turns with the motor.
inline constexpr Gearing direct = [] {
    Gearing g;
    g.motorTurnsPerTurn = 1.0;
    return g;
}();

// ═══ POSITION WORDS ═══════════════════════════════════════════════════════════════════

/// The holding term for gravity — `volts` is what holds the mechanism still where gravity pulls
/// hardest (an arm held LEVEL; a lift anywhere). Made by cosine(), constantGravity(), noGravity.
struct GravityTerm {
    /// Which shape. UNSET = 0, refused.
    Gravity kind = static_cast<Gravity>(0);
    /// The hold, V.
    double volts = 0.0;
    /// Cosine: the output angle (deg) at which the arm is level.
    double levelAtDeg = 0.0;
};
/// An ARM: `holdLevel` holds it out level, less as it points up (cosine); level at `levelAt` degrees.
[[nodiscard]] constexpr GravityTerm cosine(units::Voltage holdLevel, double levelAt) noexcept {
    return GravityTerm{Gravity::Cosine, holdLevel.value(), levelAt};
}
/// A LIFT: the same `volts` everywhere along its travel.
[[nodiscard]] constexpr GravityTerm constantGravity(units::Voltage volts) noexcept {
    return GravityTerm{Gravity::Constant, volts.value(), 0.0};
}
/// Nothing to hold (a turntable, a counterbalanced arm).
inline constexpr GravityTerm noGravity{Gravity::None, 0.0, 0.0};

/// How a position mechanism finds its zero — every moment the requirements list: at program start
/// with no motion (restsAtStop, absoluteSensor, a home switch already pressed), or by a homing MOVE
/// (homeBySwitch, homeByStall) at auton start, at driver start or on request, which the auton or the
/// driver file says. Made by the words below.
struct Homing {
    /// How. UNSET = 0, refused.
    HomeMethod method = static_cast<HomeMethod>(0);
    /// Where zero puts it (degrees / inches).
    double at = 0.0;
    /// absoluteSensor only: the sensor's reading (deg) when the mechanism is at `at`.
    double sensorAtZeroDeg = 0.0;
    /// A homing move's gentle push toward its stop, V, signed (it must point at the home end).
    double volts = 0.0;
    /// homeByStall only: the signature that means "at the stop".
    Stall stall{};
};
/// It RESTS on its stop when the program starts (an arm down on its hard stop): that is `at`.
[[nodiscard]] constexpr Homing restsAtStop(double at) noexcept { return Homing{.method = HomeMethod::RestsAtStop, .at = at}; }
/// The same for a lift: `at` in inches.
[[nodiscard]] constexpr Homing restsAtStop(units::Length at) noexcept { return restsAtStop(at.value()); }
/// Push at `toward` until its HOME SWITCH is pressed; that is `at`. Wire it with homeSwitch('B').
[[nodiscard]] constexpr Homing homeBySwitch(double at, units::Voltage toward) noexcept {
    return Homing{.method = HomeMethod::Switch, .at = at, .volts = toward.value()};
}
/// The same for a lift: `at` in inches.
[[nodiscard]] constexpr Homing homeBySwitch(units::Length at, units::Voltage toward) noexcept {
    return homeBySwitch(at.value(), toward);
}
/// Push at `toward` until it STALLS against its stop (the team's `signature`); that is `at`.
[[nodiscard]] constexpr Homing homeByStall(double at, units::Voltage toward, Stall signature) noexcept {
    return Homing{.method = HomeMethod::Stall, .at = at, .volts = toward.value(), .stall = signature};
}
/// The same for a lift: `at` in inches.
[[nodiscard]] constexpr Homing homeByStall(units::Length at, units::Voltage toward, Stall signature) noexcept {
    return homeByStall(at.value(), toward, signature);
}
/// An angle sensor that knows where it is at power-up: the mechanism is at `at` when the sensor
/// reads `sensorReadsDeg` — a FIXED offset, never "zero wherever it stands" (HA-186: that the V5
/// rotation sensor's cumulative reading starts at its absolute angle). Needs `.angleSensor`.
[[nodiscard]] constexpr Homing absoluteSensor(double at, double sensorReadsDeg) noexcept {
    return Homing{.method = HomeMethod::Absolute, .at = at, .sensorAtZeroDeg = sensorReadsDeg};
}
/// Never homed: position moves are refused (NOT_HOMED); a held button jogs it at `.unhomedVolts`.
inline constexpr Homing neverHomed{.method = HomeMethod::None};

/// An Arm's travel limits, degrees. REQUIRED (min < max).
struct DegreeRange {
    /// The lower limit.
    double min = 0.0;
    /// The upper limit.
    double max = 0.0;
};
/// A Lift's travel limits, inches. REQUIRED (min < max).
struct InchRange {
    /// The lower limit.
    units::Length min{0.0};
    /// The upper limit.
    units::Length max{0.0};
};
/// An Arm's named preset: `{"score", 95}`.
struct DegreePreset {
    /// Its name (15 characters at most).
    const char* name = nullptr;
    /// Its angle, degrees.
    double value = 0.0;
};
/// A Lift's named preset: `{"high", 22_in}`.
struct InchPreset {
    /// Its name (15 characters at most).
    const char* name = nullptr;
    /// Its height.
    units::Length value{0.0};
};
/// A Flywheel's named speed: `{"shoot", 3000}`.
struct RpmPreset {
    /// Its name (15 characters at most).
    const char* name = nullptr;
    /// Its output rpm.
    double rpm = 0.0;
};

/// A lift's SHAPE: how its inches relate to its output shaft's degrees. Made by spool(), linkage(),
/// measured().
struct LiftShape {
    /// Which shape. UNSET = 0, refused.
    Geometry geometry = static_cast<Geometry>(0);
    /// spool(): its diameter, inches.
    double spoolDiameterIn = 0.0;
    /// linkage(): one stage's arm length, inches.
    double armLengthIn = 0.0;
    /// linkage(): the first pivot's height, inches.
    double pivotHeightIn = 0.0;
    /// linkage(): 1 = a four-bar, 2 = a double reverse four-bar.
    std::uint8_t stages = 0;
    /// measured(): {height in, output angle deg} rows.
    double table[kMaxTablePoints][2]{};
    /// measured(): how many rows were written.
    std::uint8_t points = 0;
};
/// A string or chain on a spool of this diameter on the output shaft.
[[nodiscard]] constexpr LiftShape spool(units::Length diameter) noexcept {
    return LiftShape{.geometry = Geometry::Spool, .spoolDiameterIn = diameter.value()};
}
/// A four-bar (1 stage) or a double reverse four-bar (2 stages): each stage `armLength` long, the
/// first pivot `pivotHeight` up; its height follows the sine of the arm's angle from level.
[[nodiscard]] constexpr LiftShape linkage(units::Length armLength, units::Length pivotHeight, int stages) noexcept {
    return LiftShape{.geometry = Geometry::Arm, .armLengthIn = armLength.value(), .pivotHeightIn = pivotHeight.value(),
                     .stages = static_cast<std::uint8_t>(stages < 0 ? 0 : (stages > 255 ? 255 : stages))};
}
/// One measured pair of a lift's table: its height at an output angle.
struct HeightAt {
    /// The lift's height there.
    units::Length height{0.0};
    /// The output shaft's angle there, degrees.
    double degrees = 0.0;
};
/// Any other shape: measured pairs {height, output angle}, at least two, strictly increasing —
/// interpolated between them. More than kMaxTablePoints pairs is recorded and refused by the list.
template <std::size_t N>
[[nodiscard]] constexpr LiftShape measured(const HeightAt (&pairs)[N]) noexcept {
    LiftShape s{.geometry = Geometry::Table};
    for (std::size_t i = 0; i < N && i < kMaxTablePoints; ++i) {
        s.table[i][0] = pairs[i].height.value();
        s.table[i][1] = pairs[i].degrees;
    }
    s.points = static_cast<std::uint8_t>(N < 255 ? N : 255);
    return s;
}

/// A position mechanism's overrides of the library's STARTING values (header; N7), one by one —
/// 0 keeps the starting value. Gains per OUTPUT-SHAFT degree; speed and acceleration in the
/// mechanism's units per second.
struct PositionTuning {
    /// Proportional gain, V/deg (starting value 0.25).
    double kP = 0.0;
    /// The hold loop's integral gain, V/(deg*s) (starting value 0.3).
    double kI = 0.0;
    /// Derivative gain, V/(deg/s) (starting value 0.004).
    double kD = 0.0;
    /// Static feedforward, V (starting value 0).
    double kS = 0.0;
    /// Velocity feedforward, V/(deg/s) (starting value 12 V over the output's free speed).
    double kV = 0.0;
    /// Acceleration feedforward, V/(deg/s^2) (starting value 0).
    double kA = 0.0;
    /// The smooth move's top speed, units/s (starting value 75 % of the free speed).
    double maxSpeed = 0.0;
    /// Its acceleration, units/s^2 (starting value: the top speed in 0.25 s).
    double maxAcceleration = 0.0;
    /// "Reached" when within this of the target (starting value 1.5 deg, or 0.25 in) ...
    double within = 0.0;
    /// ... for this long (starting value 0.1 s).
    units::Time settle{0.0};
};
/// A flywheel's overrides of its starting values — 0 keeps the starting value.
struct SpeedTuning {
    /// Static feedforward, V (starting value 0).
    double kS = 0.0;
    /// Velocity feedforward, V per output rad/s (starting value 12 V over the free speed).
    double kV = 0.0;
    /// Proportional gain, V per rad/s (starting value 0.05).
    double kP = 0.0;
    /// Integral gain, V per rad (starting value 0.2).
    double kI = 0.0;
};

// ═══ GOALS — what a step or a driver preset asks of ONE mechanism ═════════════════════
// Made only by a declaration (arm["score"], arm.to(120), flywheel["shoot"], claw["closed"]) or by the
// route binder from a spec (MECH-A4), so a goal always names its mechanism. The service finds a
// goal's mechanism by NAME and KIND (a declaration written without `inline` has one address per
// translation unit — the brief's P12), never by address alone.

/// A position goal: an Arm's or a Lift's preset or constant target.
struct PositionGoal {
    /// The declaration's address (what a driver control's build check compares).
    const void* mechanism;
    /// The declaration's `.name` (what the run-time lookup compares).
    const char* mechanismName;
    /// The preset's name, or nullptr for a number.
    const char* presetName;
    /// The target, degrees or inches.
    double value;
    /// A run-time number (`clampedTo`): clamped at the limits by the soft stop, logged.
    bool computed;
    /// Degrees (an Arm's goal) or inches (a Lift's): the service finds only a mechanism of these units
    /// (UNSET, the byte 0, finds none).
    Units units;
};
/// A speed goal: a Flywheel's named speed or constant target.
struct SpeedGoal {
    /// The declaration's address.
    const void* mechanism;
    /// The declaration's `.name`.
    const char* mechanismName;
    /// The preset's name, or nullptr for a number.
    const char* presetName;
    /// The target, output rpm.
    double rpm;
    /// A run-time number (`clampedTo`).
    bool computed;
};
/// A pneumatic goal: one of its named positions.
struct PneumaticGoal {
    /// The declaration's address.
    const void* mechanism;
    /// The declaration's `.name`.
    const char* mechanismName;
    /// The position's index in the declaration.
    std::uint8_t position;
    /// The position's name (the lookup checks it at its index).
    const char* positionName;
};

namespace detail {
/// Two C strings equal (null never equals anything).
[[nodiscard]] constexpr bool same(const char* a, const char* b) noexcept {
    if (a == nullptr || b == nullptr) {
        return false;
    }
    return std::string_view{a} == std::string_view{b};
}
// The consteval refusals (spec.hpp's idiom): calling one (it is not constexpr) stops the build, and
// GCC names it — the message a student reads is the function's name and the line of the auton.
/// A preset the mechanism does not have.
inline void no_such_preset_on_this_mechanism() {}
/// A pneumatic position the mechanism does not have.
inline void no_such_position_on_this_mechanism() {}
/// A constant target past the travel limits.
inline void target_is_outside_the_travel_limits() {}
/// A constant speed past the free speed.
inline void speed_is_past_the_free_speed() {}
/// `arm.to(90_deg)` — an arm angle is a plain number.
inline void write_an_arm_angle_as_a_plain_number_like_to_90_not_90_deg() {}
/// `lift.to(18)` — a lift height is inches.
inline void a_lift_height_needs_inches_like_to_18_in() {}
/// `intake["fast"]` — a power mechanism has no presets.
inline void a_power_mechanism_has_no_presets_run_it_with_r_run() {}
/// `claw.to(...)` — a pneumatic position is named.
inline void a_pneumatic_position_is_named_like_claw_open_in_quotes() {}
/// Copy `src` into `dst`, truncated to fit with its NUL.
template <std::size_t N>
constexpr void copyText(char (&dst)[N], const char* src) noexcept {
    if (src == nullptr) {
        return;
    }
    for (std::size_t i = 0; i + 1 < N && src[i] != '\0'; ++i) {
        dst[i] = src[i];
    }
}
/// `s` fits in `n` bytes with its NUL.
[[nodiscard]] constexpr bool fits(const char* s, std::size_t n) noexcept {
    if (s == nullptr) {
        return false;
    }
    std::size_t i = 0;
    while (s[i] != '\0') {
        if (++i >= n) {
            return false;
        }
    }
    return true;
}
/// `s` is set and does not fit in N bytes with its NUL.
template <std::size_t N>
[[nodiscard]] constexpr bool tooLong(const char* s) noexcept {
    return s != nullptr && !fits(s, N);
}
/// The cartridge's nominal free speed at 12 V, motor rpm (100 / 200 / 600) — the basis of the
/// starting kV and the smooth move's starting speed; 0 for an UNSET cartridge (refused anyway).
[[nodiscard]] constexpr double cartridgeRpm(hal::MotorCartridge c) noexcept {
    switch (c) {
        case hal::MotorCartridge::Red:
            return 100.0;
        case hal::MotorCartridge::Green:
            return 200.0;
        case hal::MotorCartridge::Blue:
            return 600.0;
    }
    return 0.0;
}
/// The spec's name (a name past 23 characters is filled with '?' and no NUL — refused by the list,
/// "does not fit 23") and where the declaration was written.
constexpr void nameAndWhere(MechanismSpec& s, const Name& name) noexcept {
    if (name.text != nullptr && fits(name.text, kNameBytes)) {
        copyText(s.name, name.text);
    } else if (name.text != nullptr) {
        for (char& c : s.name) {
            c = '?';
        }
    }
    s.declaredFile = name.file;
    s.declaredLine = name.line;
}
/// What every motor kind has: its name and place, its motor roles (each pointing at the `.name`
/// line), its cartridge and its safe state.
constexpr void motorsAndCommon(MechanismSpec& s, const Name& name, const Motors& motors,
                               hal::MotorCartridge cartridge, config::SafeBrakeMode safe) noexcept {
    nameAndWhere(s, name);
    s.motorCount = motors.tooMany ? static_cast<std::uint8_t>(kMaxMotorRoles + 1) : motors.count;
    for (std::size_t i = 0; i < kMaxMotorRoles; ++i) {
        s.motors[i] = MotorRole{.reversed = motors.m[i].reversed, .source = motors.m[i].changeAt,
                                .file = name.file, .line = name.line};
    }
    s.cartridge = cartridge;
    s.safeMode = safe;
}
/// A sensor word as a spec role.
constexpr SensorRole role(const Sensor& x) noexcept {
    return SensorRole{.kind = x.kind, .reversed = x.reversed, .pressedLevel = x.pressedLevel,
                      .seesWithinIn = x.seesWithinIn, .seesProximity = x.seesProximity, .source = x.changeAt,
                      .file = x.file, .line = x.line};
}
/// A stall signature as spec data (motor rpm to rad/s).
constexpr StallSpec stallSpec(const Stall& s) noexcept {
    return StallSpec{.amps = s.amps, .speedRadS = s.belowMotorRpm * 2.0 * 3.14159265358979323846 / 60.0,
                     .time = s.lasting.value()};
}
/// THE STARTING VALUES (header; N7; HA-182 — invented) for a position mechanism, from its gearing and
/// cartridge: kP 0.25, kI 0.3 (the hold loop), kD 0.004, kS 0, kV 12 V over the output's free speed,
/// kA 0, the smooth move at 75 % of the free speed reached in 0.25 s, "reached" within `withinStart`
/// (1.5 deg / 0.25 in) for 0.1 s — each replaced by a stated `.tuning` value.
constexpr void positionStart(PositionSpec& p, const PositionTuning& t, hal::MotorCartridge c, double unitsPerDeg,
                             double withinStart) noexcept {
    const double freeDegS = p.gearRatio > 0.0 ? cartridgeRpm(c) * 6.0 / p.gearRatio : 0.0;  // output deg/s
    p.kP = t.kP > 0.0 ? t.kP : 0.25;
    p.kI = t.kI > 0.0 ? t.kI : 0.3;
    p.kD = t.kD > 0.0 ? t.kD : 0.004;
    p.kS = t.kS;  // 0 unless stated: the hold loop's integral covers friction
    p.kV = t.kV > 0.0 ? t.kV : (freeDegS > 0.0 ? 12.0 / freeDegS : 0.0);
    p.kA = t.kA;
    p.maxVelocity = t.maxSpeed > 0.0 ? t.maxSpeed : 0.75 * freeDegS * unitsPerDeg;
    p.maxAcceleration = t.maxAcceleration > 0.0 ? t.maxAcceleration : p.maxVelocity / 0.25;
    p.tolerance = t.within > 0.0 ? t.within : withinStart;
    p.settleTime = t.settle.value() > 0.0 ? t.settle.value() : 0.1;
}
/// A homing word into the spec.
constexpr void homing(PositionSpec& p, const Homing& h) noexcept {
    p.home = h.method;
    p.homePosition = h.at;
    p.homeVolts = h.volts;
    p.homeStall = stallSpec(h.stall);
    p.absoluteSensorAtZeroDeg = h.sensorAtZeroDeg;
}
/// The mechanism's STEEPEST units per OUTPUT degree — the one scale between its units and the loops'
/// degrees for a speed or an acceleration (the starting profile here; the service's move plan and soft
/// stop, mechanism/service.hpp): 1 for an arm; for a lift exact for a spool, a linkage's level slope
/// (its steepest) and a measured table's steepest segment. The steepest, so a speed in units turned
/// into degrees by it is never faster in units anywhere on the travel — conservative (0 for a shape
/// with nothing to scale by, refused anyway).
[[nodiscard]] constexpr double unitsPerOutputDeg(const PositionSpec& p) noexcept {
    switch (p.geometry) {
        case Geometry::Rotary:
            return 1.0;
        case Geometry::Spool:
            return 3.14159265358979323846 * p.spoolDiameterIn / 360.0;
        case Geometry::Arm:
            return p.armStages * p.armLengthIn * 3.14159265358979323846 / 180.0;
        case Geometry::Table: {
            // every segment, not the first: a later one can be the steepest (a table read off a lift
            // that speeds up toward its top)
            double steepest = 0.0;
            for (std::size_t i = 1; i < p.tablePoints && i < kMaxTablePoints; ++i) {
                const double dDeg = p.table[i][1] - p.table[i - 1][1];
                const double slope = dDeg > 0.0 ? (p.table[i][0] - p.table[i - 1][0]) / dDeg : 0.0;
                steepest = slope > steepest ? slope : steepest;
            }
            return steepest;
        }
    }
    return 0.0;
}
/// THE STARTING VALUES (header; HA-182 — invented) for a flywheel, from its free speed (output
/// rad/s): kS 0, kV 12 V over the free speed, kP 0.05 V per rad/s, kI 0.2 V per rad (the speed loop's
/// integral, which acts only near the target — mechanism/service.hpp) — each replaced by a stated
/// `.tuning` value.
constexpr void speedStart(SpeedSpec& q, const SpeedTuning& t, double freeRadS) noexcept {
    q.kS = t.kS;
    q.kV = t.kV > 0.0 ? t.kV : (freeRadS > 0.0 ? 12.0 / freeRadS : 0.0);
    q.kP = t.kP > 0.0 ? t.kP : 0.05;
    q.kI = t.kI > 0.0 ? t.kI : 0.2;
}
}  // namespace detail

/// The numbers a POSITION spec would run with NO `.tuning` — the library's starting values
/// (HA-182) for its cartridge, gearing and shape. What startingValuesLine() (wiring.hpp) compares a
/// spec against to say which of its numbers are the library's and which the team's.
[[nodiscard]] constexpr PositionSpec positionStartingValues(const MechanismSpec& s) noexcept {
    PositionSpec p = s.position;
    detail::positionStart(p, PositionTuning{}, s.cartridge, detail::unitsPerOutputDeg(p),
                          p.units == Units::Inches ? 0.25 : 1.5);
    return p;
}
/// The numbers a SPEED spec would run with NO `.tuning` (HA-182), from its cartridge through its
/// gearing.
[[nodiscard]] constexpr SpeedSpec speedStartingValues(const MechanismSpec& s) noexcept {
    SpeedSpec q = s.speed;
    const double freeRpm = q.gearRatio > 0.0 ? detail::cartridgeRpm(s.cartridge) / q.gearRatio : 0.0;
    detail::speedStart(q, SpeedTuning{}, freeRpm * 2.0 * 3.14159265358979323846 / 60.0);
    return q;
}

// ═══ THE FIVE KINDS ═══════════════════════════════════════════════════════════════════

/// POWER — run at a voltage (intakes, rollers, conveyors, robot two's claw pivots). Members in the
/// order C++ requires them written (header).
struct Power {
    /// Its name (23 characters at most).
    Name name{};
    /// Its motor roles.
    Motors motors{};
    /// Its motors' cartridge. REQUIRED.
    hal::MotorCartridge cartridge = static_cast<hal::MotorCartridge>(0);
    /// Optional: distanceSensor(2_in), opticalSensor(0.5), limitSwitch().
    Sensor pieceSensor{};
    /// What every stop lands in: Coast, Brake or Hold. REQUIRED.
    config::SafeBrakeMode safe = static_cast<config::SafeBrakeMode>(0);
    /// Optional: the jam signature (`stalls`, the STALLED verdict); with none, the boot says a held
    /// button is cut only by its release (HA-154).
    Stall stall{};
    /// Optional: the most current each motor may draw, set in the motor at boot (CurrentCap; MECH-S-A).
    CurrentCap currentCap{};
    /// The most any command may ask (the motor's own ceiling unless lowered), (0, 12] V.
    units::Voltage maxVolts{12.0};

    /// `intake["fast"]`: refused BY NAME — a power mechanism has no presets (run it: `r.run(intake,
    /// 12_volt, ...)`).
    template <class T>
    [[nodiscard]] consteval int operator[](T) const {
        detail::a_power_mechanism_has_no_presets_run_it_with_r_run();
        return 0;
    }

    /// This declaration as data.
    [[nodiscard]] constexpr MechanismSpec spec() const noexcept {
        MechanismSpec s{};
        detail::motorsAndCommon(s, name, motors, cartridge, safe);
        s.kind = Kind::Power;
        s.pieceSensor = detail::role(pieceSensor);
        s.power.maxVolts = maxVolts.value();
        s.power.stall = detail::stallSpec(stall);
        s.currentCapAmps = currentCap.amps;
        return s;
    }
};

/// ARM — go to an angle in DEGREES and hold it (arms, tilters, a claw pivot with presets).
struct Arm {
    /// Its name (23 characters at most).
    Name name{};
    /// Its motor roles.
    Motors motors{};
    /// Its motors' cartridge. REQUIRED.
    hal::MotorCartridge cartridge = static_cast<hal::MotorCartridge>(0);
    /// REQUIRED: gears(12, 60) — tooth counts, the motor's gear first.
    Gearing gearing{};
    /// Optional: rotationSensor() — then it, not the motor encoders, is the angle.
    Sensor angleSensor{};
    /// What every stop lands in. REQUIRED.
    config::SafeBrakeMode safe = static_cast<config::SafeBrakeMode>(0);
    /// REQUIRED: a blocked move's or hold's signature (P4, N10).
    Stall stall{};
    /// Optional: the most current each motor may draw, set in the motor at boot (CurrentCap; MECH-S-A).
    CurrentCap currentCap{};
    /// The travel limits, degrees. REQUIRED.
    DegreeRange range{};
    /// cosine(...) or noGravity. REQUIRED.
    GravityTerm gravity{};
    /// restsAtStop / homeBySwitch / homeByStall / absoluteSensor / neverHomed. REQUIRED.
    Homing home{};
    /// Optional: {"score", 95}, ... (six at most).
    DegreePreset presets[kMaxPresets]{};
    /// Optional: a held button's most before homing (0 = nothing moves it before homing).
    units::Voltage unhomedVolts{0.0};
    /// Optional: the most volts anything drives it DOWN with (0 = the motor's 12 V; API 2.10).
    units::Voltage maxDownVolts{0.0};
    /// Optional: overrides of the starting values (header).
    PositionTuning tuning{};

    /// A named preset: `arm["score"]`. A name the arm does not have FAILS THE BUILD.
    [[nodiscard]] consteval PositionGoal operator[](const char* preset) const {
        for (const DegreePreset& p : presets) {
            if (detail::same(p.name, preset)) {
                return PositionGoal{this, name.text, p.name, p.value, false, Units::Degrees};
            }
        }
        detail::no_such_preset_on_this_mechanism();
        return PositionGoal{};
    }
    /// A constant target in degrees: `arm.to(120)`. Past the travel limits FAILS THE BUILD.
    [[nodiscard]] consteval PositionGoal to(double degrees) const {
        if (!(degrees >= range.min && degrees <= range.max)) {
            detail::target_is_outside_the_travel_limits();
        }
        return PositionGoal{this, name.text, nullptr, degrees, false, Units::Degrees};
    }
    /// `arm.to(90_deg)`: refused BY NAME (`_deg` is a wrapping heading — 270 reads -90).
    template <class T>
        requires(!std::is_arithmetic_v<T>)
    [[nodiscard]] consteval PositionGoal to(const T&) const {
        detail::write_an_arm_angle_as_a_plain_number_like_to_90_not_90_deg();
        return PositionGoal{};
    }
    /// A target computed while the auton runs: clamped to the limits by the soft stop, and logged.
    [[nodiscard]] constexpr PositionGoal clampedTo(double degrees) const noexcept {
        return PositionGoal{this, name.text, nullptr, degrees, true, Units::Degrees};
    }

    /// This declaration as data.
    [[nodiscard]] constexpr MechanismSpec spec() const noexcept {
        MechanismSpec s{};
        detail::motorsAndCommon(s, name, motors, cartridge, safe);
        s.kind = Kind::Position;
        s.angleSensor = detail::role(angleSensor);
        if (home.method == HomeMethod::Switch) {
            s.homeSwitch = SensorRole{.kind = SensorKind::Switch, .file = name.file, .line = name.line};
        }
        PositionSpec& p = s.position;
        p.units = Units::Degrees;
        p.geometry = Geometry::Rotary;
        p.gearRatio = gearing.motorTurnsPerTurn;
        p.minPosition = range.min;
        p.maxPosition = range.max;
        p.gravity = gravity.kind;
        p.kG = gravity.volts;
        p.horizontalAt = gravity.levelAtDeg;
        detail::homing(p, home);
        p.stall = detail::stallSpec(stall);
        s.currentCapAmps = currentCap.amps;
        p.unhomedVolts = unhomedVolts.value();
        p.maxDownVolts = maxDownVolts.value();
        detail::positionStart(p, tuning, cartridge, 1.0, 1.5);
        for (std::size_t i = 0; i < kMaxPresets; ++i) {
            if (presets[i].name == nullptr) {
                break;
            }
            s.presetNameTooLong = s.presetNameTooLong || detail::tooLong<16>(presets[i].name);
            detail::copyText(p.presets[i].name, presets[i].name);
            p.presets[i].value = presets[i].value;
            p.presetCount = static_cast<std::uint8_t>(i + 1);
        }
        return s;
    }
};

/// LIFT — go to a height in INCHES and hold it (a spool lift, a four-bar, a DR4B, any measured
/// shape). Members in the Arm's order, its shape after the gearing.
struct Lift {
    /// Its name (23 characters at most).
    Name name{};
    /// Its motor roles.
    Motors motors{};
    /// Its motors' cartridge. REQUIRED.
    hal::MotorCartridge cartridge = static_cast<hal::MotorCartridge>(0);
    /// REQUIRED: gears(12, 36), or direct.
    Gearing gearing{};
    /// REQUIRED: spool(...), linkage(...) or measured({...}).
    LiftShape shape{};
    /// Optional: rotationSensor() on the output shaft.
    Sensor angleSensor{};
    /// What every stop lands in. REQUIRED.
    config::SafeBrakeMode safe = static_cast<config::SafeBrakeMode>(0);
    /// REQUIRED (P4, N10).
    Stall stall{};
    /// Optional: the most current each motor may draw, set in the motor at boot (CurrentCap; MECH-S-A) —
    /// how hard a string lift's string is pulled.
    CurrentCap currentCap{};
    /// The travel limits, inches. REQUIRED.
    InchRange range{};
    /// constantGravity(...) (or cosine for a linkage's arm, or noGravity). REQUIRED.
    GravityTerm gravity{};
    /// How it finds zero. REQUIRED.
    Homing home{};
    /// Optional: {"high", 22_in}, ... (six at most).
    InchPreset presets[kMaxPresets]{};
    /// Optional: a held button's most before homing.
    units::Voltage unhomedVolts{0.0};
    /// Optional: the most volts anything drives it DOWN with (0 = the motor's 12 V; API 2.10).
    units::Voltage maxDownVolts{0.0};
    /// Optional: overrides of the starting values.
    PositionTuning tuning{};

    /// A named preset: `lift["high"]`. A name the lift does not have FAILS THE BUILD.
    [[nodiscard]] consteval PositionGoal operator[](const char* preset) const {
        for (const InchPreset& p : presets) {
            if (detail::same(p.name, preset)) {
                return PositionGoal{this, name.text, p.name, p.value.value(), false, Units::Inches};
            }
        }
        detail::no_such_preset_on_this_mechanism();
        return PositionGoal{};
    }
    /// A constant target height: `lift.to(18_in)`. Past the travel limits FAILS THE BUILD.
    [[nodiscard]] consteval PositionGoal to(units::Length height) const {
        if (!(height.value() >= range.min.value() && height.value() <= range.max.value())) {
            detail::target_is_outside_the_travel_limits();
        }
        return PositionGoal{this, name.text, nullptr, height.value(), false, Units::Inches};
    }
    /// `lift.to(18)`: refused BY NAME (a height is inches: `lift.to(18_in)`).
    template <class T>
        requires std::is_arithmetic_v<T>
    [[nodiscard]] consteval PositionGoal to(T) const {
        detail::a_lift_height_needs_inches_like_to_18_in();
        return PositionGoal{};
    }
    /// A height computed while the auton runs: clamped to the limits by the soft stop, and logged.
    [[nodiscard]] constexpr PositionGoal clampedTo(units::Length height) const noexcept {
        return PositionGoal{this, name.text, nullptr, height.value(), true, Units::Inches};
    }

    /// This declaration as data.
    [[nodiscard]] constexpr MechanismSpec spec() const noexcept {
        MechanismSpec s{};
        detail::motorsAndCommon(s, name, motors, cartridge, safe);
        s.kind = Kind::Position;
        s.angleSensor = detail::role(angleSensor);
        if (home.method == HomeMethod::Switch) {
            s.homeSwitch = SensorRole{.kind = SensorKind::Switch, .file = name.file, .line = name.line};
        }
        PositionSpec& p = s.position;
        p.units = Units::Inches;
        p.geometry = shape.geometry;
        p.spoolDiameterIn = shape.spoolDiameterIn;
        p.armLengthIn = shape.armLengthIn;
        p.pivotHeightIn = shape.pivotHeightIn;
        p.armStages = shape.stages;
        for (std::size_t i = 0; i < kMaxTablePoints; ++i) {
            p.table[i][0] = shape.table[i][0];
            p.table[i][1] = shape.table[i][1];
        }
        p.tablePoints = shape.points;
        p.gearRatio = gearing.motorTurnsPerTurn;
        p.minPosition = range.min.value();
        p.maxPosition = range.max.value();
        p.gravity = gravity.kind;
        p.kG = gravity.volts;
        p.horizontalAt = gravity.levelAtDeg;
        detail::homing(p, home);
        p.stall = detail::stallSpec(stall);
        s.currentCapAmps = currentCap.amps;
        p.unhomedVolts = unhomedVolts.value();
        p.maxDownVolts = maxDownVolts.value();
        // The starting profile's scale, inches per output degree: exact for a spool; for a linkage
        // its steepest (level) slope and for a table its steepest segment's — conservative estimates.
        detail::positionStart(p, tuning, cartridge, detail::unitsPerOutputDeg(p), 0.25);
        for (std::size_t i = 0; i < kMaxPresets; ++i) {
            if (presets[i].name == nullptr) {
                break;
            }
            s.presetNameTooLong = s.presetNameTooLong || detail::tooLong<16>(presets[i].name);
            detail::copyText(p.presets[i].name, presets[i].name);
            p.presets[i].value = presets[i].value.value();
            p.presetCount = static_cast<std::uint8_t>(i + 1);
        }
        return s;
    }
};

/// "At speed": within `rpm` of the target for `lasting` (both the team's, P6). Made by within().
struct AtSpeed {
    /// The band, output rpm.
    double rpm = 0.0;
    /// How long it must stay inside it.
    units::Time lasting{0.0};
};
/// `.atSpeed = within(60, 0.1_s)`: within 60 rpm of the target for 0.1 s.
[[nodiscard]] constexpr AtSpeed within(double rpm, units::Time lasting) noexcept { return AtSpeed{rpm, lasting}; }

/// FLYWHEEL — spin at an RPM of its output and say when it is up to speed (the SPEED kind; re-check
/// N11: the declaration's word is what a team builds).
struct Flywheel {
    /// Its name (23 characters at most).
    Name name{};
    /// Its motor roles.
    Motors motors{};
    /// Its motors' cartridge. REQUIRED.
    hal::MotorCartridge cartridge = static_cast<hal::MotorCartridge>(0);
    /// REQUIRED: gears(72, 12) = geared UP 6:1.
    Gearing gearing{};
    /// Optional: rotationSensor() on the wheel — then it, not the motor encoders, is the speed.
    Sensor angleSensor{};
    /// What every stop lands in. REQUIRED.
    config::SafeBrakeMode safe = static_cast<config::SafeBrakeMode>(0);
    /// REQUIRED: a jammed wheel's signature (P4, N10).
    Stall stall{};
    /// Optional: the most current each motor may draw, set in the motor at boot (CurrentCap; MECH-S-A).
    CurrentCap currentCap{};
    /// REQUIRED: within(60, 0.1_s).
    AtSpeed atSpeed{};
    /// Its named speeds: {"shoot", 3000}, ... (six at most).
    RpmPreset presets[kMaxPresets]{};
    /// Optional: overrides of the starting values.
    SpeedTuning tuning{};

    /// The fastest it can spin: the cartridge's free speed through the gearing, output rpm.
    [[nodiscard]] constexpr double freeRpm() const noexcept {
        return gearing.motorTurnsPerTurn > 0.0 ? detail::cartridgeRpm(cartridge) / gearing.motorTurnsPerTurn : 0.0;
    }
    /// A named speed: `flywheel["shoot"]`. A name it does not have FAILS THE BUILD.
    [[nodiscard]] consteval SpeedGoal operator[](const char* preset) const {
        for (const RpmPreset& p : presets) {
            if (detail::same(p.name, preset)) {
                return SpeedGoal{this, name.text, p.name, p.rpm, false};
            }
        }
        detail::no_such_preset_on_this_mechanism();
        return SpeedGoal{};
    }
    /// A constant speed: `flywheel.at(3000)`. Past its free speed FAILS THE BUILD.
    [[nodiscard]] consteval SpeedGoal at(double rpm) const {
        if (!(rpm >= -freeRpm() && rpm <= freeRpm())) {
            detail::speed_is_past_the_free_speed();
        }
        return SpeedGoal{this, name.text, nullptr, rpm, false};
    }
    /// A speed computed while the auton runs: clamped to the free speed, and logged.
    [[nodiscard]] constexpr SpeedGoal clampedTo(double rpm) const noexcept {
        return SpeedGoal{this, name.text, nullptr, rpm, true};
    }

    /// This declaration as data (its free speed derived, its starting values filled).
    [[nodiscard]] constexpr MechanismSpec spec() const noexcept {
        MechanismSpec s{};
        detail::motorsAndCommon(s, name, motors, cartridge, safe);
        s.kind = Kind::Speed;
        s.angleSensor = detail::role(angleSensor);
        SpeedSpec& q = s.speed;
        q.gearRatio = gearing.motorTurnsPerTurn;
        q.maxRpm = freeRpm();
        q.stall = detail::stallSpec(stall);
        s.currentCapAmps = currentCap.amps;
        detail::speedStart(q, tuning, freeRpm() * 2.0 * 3.14159265358979323846 / 60.0);
        q.atSpeedBandRpm = atSpeed.rpm;
        q.atSpeedTime = atSpeed.lasting.value();
        for (std::size_t i = 0; i < kMaxPresets; ++i) {
            if (presets[i].name == nullptr) {
                break;
            }
            s.presetNameTooLong = s.presetNameTooLong || detail::tooLong<16>(presets[i].name);
            detail::copyText(q.presets[i].name, presets[i].name);
            q.presets[i].value = presets[i].rpm;
            q.presetCount = static_cast<std::uint8_t>(i + 1);
        }
        return s;
    }
};

/// One position's line levels, as written: `{Low}` (one line) or `{High, Low}` (two). Records HOW
/// MANY were written, so the wiring can refuse a position stating a different number of levels than
/// it has lines (U2: a bool array cannot tell an unstated level from Low).
struct Levels {
    /// The levels, line 1 first.
    bool l[kMaxLineRoles]{};
    /// How many were written.
    std::uint8_t count = 0;
    /// More than kMaxLineRoles were written.
    bool tooMany = false;
    /// UNSET: no levels.
    constexpr Levels() noexcept = default;
    /// `{High, Low}`.
    consteval Levels(std::initializer_list<bool> list) noexcept {
        for (const bool x : list) {
            if (count == kMaxLineRoles) {
                tooMany = true;
                break;
            }
            l[count++] = x;
        }
    }
};
/// A pneumatic POSITION: its name, each line's level in it, and whether reaching it is a GRAB (or a
/// RELEASE) a piece sensor confirms (Q2: with no sensor it is counted done after `moveTime`, reported
/// UNCONFIRMED).
struct NamedPosition {
    /// Its name (15 characters at most).
    const char* name = nullptr;
    /// Its line levels.
    Levels levels{};
    /// `grabs`, `releases`, or nothing.
    Confirm confirm = Confirm::None;
};
/// A pneumatic mechanism's positions, in order (two or three): `.positions = {{"closed", {Low}, grabs},
/// {"open", {High}}}`.
struct Positions {
    /// The positions, in the order written.
    NamedPosition p[kMaxPneumaticPositions]{};
    /// How many were written.
    std::uint8_t count = 0;
    /// More than kMaxPneumaticPositions were written.
    bool tooMany = false;
    /// Where the list was written (an initializer list — see Motors::file; the spec uses `.name`'s).
    const char* file = nullptr;
    /// That line.
    std::uint32_t line = 0;
    /// UNSET: no positions.
    constexpr Positions() noexcept = default;
    /// `{{"closed", {Low}, grabs}, {"open", {High}}}`.
    consteval Positions(std::initializer_list<NamedPosition> list,
                        std::source_location at = std::source_location::current()) noexcept
        : file{at.file_name()}, line{at.line()} {
        for (const NamedPosition& x : list) {
            if (count == kMaxPneumaticPositions) {
                tooMany = true;
                break;
            }
            p[count++] = x;
        }
    }
};
/// A line driven HIGH (on, `true` — the solenoid powered).
inline constexpr bool High = true;
/// A line driven LOW (off, `false` — the solenoid unpowered).
inline constexpr bool Low = false;
/// Reaching this position GRABS a piece: a piece sensor confirms it; with none, UNCONFIRMED and
/// counted done (Q2 (c)).
inline constexpr Confirm grabs = Confirm::Sees;
/// Reaching this position lets a piece GO: a piece sensor confirms it is gone.
inline constexpr Confirm releases = Confirm::StopsSeeing;

/// PNEUMATIC — two or three named positions, each the team's line levels (Q13 (a)).
struct Pneumatic {
    /// Its name (23 characters at most).
    Name name{};
    /// REQUIRED: {{"closed", {Low}, grabs}, {"open", {High}}} — one level per wired line.
    Positions positions{};
    /// REQUIRED: the position at program start (a position's name).
    const char* start = nullptr;
    /// REQUIRED: the position the end-of-run guard puts it in.
    const char* safe = nullptr;
    /// REQUIRED: how long the cylinder takes (no default, P6).
    units::Time moveTime{0.0};
    /// Optional: confirms a `grabs` / `releases` position.
    Sensor pieceSensor{};
    /// With a piece sensor: how long after moveTime the confirmation may take.
    units::Time confirmWithin{0.0};
    /// Generated robot files only: the positions' JSON path (G3a).
    const char* changeAt = nullptr;

    /// `claw.to(...)`: refused BY NAME (a position is named: claw["open"]).
    template <class T>
    [[nodiscard]] consteval PneumaticGoal to(const T&) const {
        detail::a_pneumatic_position_is_named_like_claw_open_in_quotes();
        return PneumaticGoal{};
    }
    /// A named position: `claw["closed"]`. A name it does not have FAILS THE BUILD.
    [[nodiscard]] consteval PneumaticGoal operator[](const char* position) const {
        for (std::size_t i = 0; i < positions.count; ++i) {
            if (detail::same(positions.p[i].name, position)) {
                return PneumaticGoal{this, name.text, static_cast<std::uint8_t>(i), positions.p[i].name};
            }
        }
        detail::no_such_position_on_this_mechanism();
        return PneumaticGoal{};
    }

    /// This declaration as data, with `lines` 3-wire lines — the WIRING says how many (lines('A') or
    /// lines('B', 'C')).
    [[nodiscard]] constexpr MechanismSpec spec(std::uint8_t lines = 1) const noexcept {
        MechanismSpec s{};
        detail::nameAndWhere(s, name);
        s.kind = Kind::Pneumatic;
        s.lineCount = lines;
        s.pieceSensor = detail::role(pieceSensor);
        PneumaticSpec& q = s.pneumatic;
        q.positionCount = positions.tooMany ? static_cast<std::uint8_t>(kMaxPneumaticPositions + 1) : positions.count;
        q.levelsFile = name.file;  // the `.name` line: exact (Motors::file's note)
        q.levelsLine = name.line;
        q.levelsSource = changeAt;
        q.startNamed = false;
        q.safeNamed = false;
        for (std::size_t i = 0; i < positions.count; ++i) {
            const NamedPosition& x = positions.p[i];
            s.presetNameTooLong = s.presetNameTooLong || detail::tooLong<16>(x.name);
            detail::copyText(q.positions[i].name, x.name);
            for (std::size_t l = 0; l < kMaxLineRoles; ++l) {
                q.positions[i].levels[l] = x.levels.l[l];
            }
            q.positions[i].levelCount = x.levels.tooMany ? static_cast<std::uint8_t>(kMaxLineRoles + 1) : x.levels.count;
            q.positions[i].confirm = x.confirm;
            if (detail::same(x.name, start)) {
                q.start = static_cast<std::uint8_t>(i);
                q.startNamed = true;
            }
            if (detail::same(x.name, safe)) {
                q.safe = static_cast<std::uint8_t>(i);
                q.safeNamed = true;
            }
        }
        q.actuationTime = moveTime.value();
        q.confirmWindow = confirmWithin.value();
        return s;
    }
};

/// THE WORDS a robot file (and later a driver file and an auton) uses unqualified: the robot's
/// namespace says `using namespace mechanism::words;` once — a using-directive, so an auton that
/// says `using namespace robot_two;` has them too. wiring.hpp adds its words to the same namespace.
namespace words {
using namespace units::literals;  // 12_volt, 2_in, 0.5_s — written in every robot file
/// The ARM declaration (degrees, held).
using mechanism::Arm;
/// The LIFT declaration (inches, held).
using mechanism::Lift;
/// The PNEUMATIC declaration (named positions).
using mechanism::Pneumatic;
/// The POWER declaration (volts).
using mechanism::Power;
/// The FLYWHEEL declaration (output rpm, "at speed").
using mechanism::Flywheel;
/// A motor mounted the + way.
using mechanism::Forward;
/// A motor mounted the other way.
using mechanism::Reversed;
/// An angle sensor: rotationSensor() / rotationSensor(Reversed).
using mechanism::rotationSensor;
/// A distance piece sensor: distanceSensor(2_in).
using mechanism::distanceSensor;
/// An optical piece sensor: opticalSensor(0.5).
using mechanism::opticalSensor;
/// A limit-switch piece sensor: limitSwitch().
using mechanism::limitSwitch;
/// A stall signature: {.amps, .belowMotorRpm, .lasting}.
using mechanism::Stall;
/// An arm's gravity: cosine(1.6_volt, 45).
using mechanism::cosine;
/// A lift's gravity: constantGravity(1.0_volt).
using mechanism::constantGravity;
/// No gravity to hold.
using mechanism::noGravity;
/// Homing: it rests on its stop at program start.
using mechanism::restsAtStop;
/// Homing: push until its switch is pressed.
using mechanism::homeBySwitch;
/// Homing: push until it stalls.
using mechanism::homeByStall;
/// Homing: an absolute angle sensor (HA-186).
using mechanism::absoluteSensor;
/// Never homed.
using mechanism::neverHomed;
/// A lift's shape: a spool.
using mechanism::spool;
/// A lift's shape: a four-bar or a DR4B.
using mechanism::linkage;
/// A lift's shape: measured pairs.
using mechanism::measured;
/// One measured pair.
using mechanism::HeightAt;
/// A flywheel's "at speed" band.
using mechanism::within;
/// A line driven high.
using mechanism::High;
/// A line driven low.
using mechanism::Low;
/// A position that grabs.
using mechanism::grabs;
/// A position that releases.
using mechanism::releases;
/// A gear pair, driving then driven.
using mechanism::gears;
/// No gearing.
using mechanism::direct;
/// The RED cartridge (100 rpm).
inline constexpr hal::MotorCartridge Red = hal::MotorCartridge::Red;
/// The GREEN cartridge (200 rpm).
inline constexpr hal::MotorCartridge Green = hal::MotorCartridge::Green;
/// The BLUE cartridge (600 rpm).
inline constexpr hal::MotorCartridge Blue = hal::MotorCartridge::Blue;
/// ONE word per brake mode for both places a team writes one: a mechanism's `.safe` (a
/// config::SafeBrakeMode) and the driver loop's `.releaseBrakeMode` (a hal::BrakeMode) — so a driver
/// file needs no `hal::BrakeMode::Coast` beside the robot file's `Coast` (N11).
struct BrakeWord {
    /// As a mechanism's declared safe state.
    config::SafeBrakeMode safe;
    /// As the drive's brake mode.
    hal::BrakeMode drive;
    /// Read as a mechanism's `.safe`.
    constexpr operator config::SafeBrakeMode() const noexcept { return safe; }  // NOLINT: a word, both ways
    /// Read as a drive brake mode.
    constexpr operator hal::BrakeMode() const noexcept { return drive; }  // NOLINT
};
/// HOLD: drives back to where it stopped — the one that keeps a load up.
inline constexpr BrakeWord Hold{config::SafeBrakeMode::Hold, hal::BrakeMode::Hold};
/// BRAKE: resists, but does not return to where it stopped.
inline constexpr BrakeWord Brake{config::SafeBrakeMode::Brake, hal::BrakeMode::Brake};
/// COAST: undriven — for a mechanism that must never sit at stall current.
inline constexpr BrakeWord Coast{config::SafeBrakeMode::Coast, hal::BrakeMode::Coast};
}  // namespace words

}  // namespace shulib::mechanism
