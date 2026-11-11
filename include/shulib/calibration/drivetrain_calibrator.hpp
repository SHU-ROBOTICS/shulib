#pragma once
//
// DrivetrainCalibrator — the robot describes its own drivetrain (chunk R3d Part 1, 2026-09-17;
// REBUILT at R3d Part 2F-1, 2026-09-24, after its first night on robot two).
//
// ── What it is for ──────────────────────────────────────────────────────────────────
// Almost every number a team is made to type about a drivetrain is discoverable on the robot.
// This state machine discovers them. It is handed every motor the brain found, an IMU and a
// clock, and it produces a hal::DrivetrainDescription: which ports drive, which way each one is
// wired, which side each belongs to, each side's inches per radian, and the effective track
// width. The three facts it CANNOT discover — the cartridge (physically unobservable: the rotor
// turns the same for every insert and the firmware already divides by whatever it was told), the
// drivetrain KIND, and which end is the FRONT — are stated by a person, and the front is stated
// PHYSICALLY: the end that LEADS the push is the FRONT (the team lead's ruling, 2026-10-06; until
// CAL1 Part B, 2026-11-05, the operator was asked afterwards which end went first — that question,
// Answer::FrontFirst / BackFirst and CalibrationReport::backFirst are gone). A push led with the
// other end yields every sign and both sides the other way round, which nothing in this class can
// see; a program compares the result with the card's record (calibration/front_check.hpp).
//
// ── PROVEN ON THE PLANT; IT HAS NEVER FINISHED ON A ROBOT ───────────────────────────
// Every property below is validated against the A2 host plant (sim/drive_plant.hpp) and nothing
// else. The Part 1 version of this class ran five times on robot two on 2026-09-24 and never
// finished — every run stopped at its side-assignment pulse or before it — and THIS version has
// never run on a robot at all. No physical drivetrain has ever been described by it. Where this
// header says the calibrator "measures" something it means "is designed to measure, and does
// measure on the plant". The numbers in CalibratorConfig are INVENTED — every one of them (A4
// register HA-136) — and the serial record this class exposes is what will say which must move.
//
// ── What Part 2F-1 changed, and why (each a finding of that first night) ────────────
//  * ONE PUSH, not two. The operator pushes the robot forward CalibratorConfig::pushDistance
//    (48 in) along a tape, once. It yields the drive ports, their signs AND each port's travel;
//    each side's scale is computed from that travel once the pulses have named the sides. The
//    distance question is gone: only one distance exists, so a wrong one cannot be picked.
//  * The side pulse RAMPS (3 → 6 → 9 V) and stops at the first level that turns the robot past
//    the yaw floor. On the host plant at the tree's usual static friction, one member of five at
//    the old single 3 V pulse turns the robot by EXACTLY nothing — the refusal robot two gave —
//    because the old test rig ran with no static friction at all. When no level turns the robot,
//    the pulse is CLASSIFIED instead of refused blind: a motor that turns but moves nothing
//    (a slipping gear) is LEFT OUT with a warning and the run goes on; a robot that rolled instead
//    of turning, or that nothing moved, is refused naming the port and saying which.
//  * REST includes the IMU's yaw rate, as the design always said and the code did not: hand
//    motion before a pulse used to leak straight into the pulse's yaw.
//  * The spin gained two SAFETY guards: it refuses at once if it turns the WRONG way (a side set
//    swapped consistently causes no fight, so nothing used to stop it for 20 s), and it steps up
//    once and then refuses if it cannot START turning (instead of stalling for 20 s).
//  * Numbers for a panel, not prose: CalibratorStatus carries the live counts and angles, a
//    Refusal code names every refusal, pushReady() is the DONE gate built from the finish's own
//    predicates, and the report carries one record per push port, per pulse and per spin
//    direction for the serial log (calibrate_panel.hpp is the pure screen model built on them).
//  * Part 2F-2 (2026-09-24) removed the three names 2F-1 had kept only so the Part 2 program
//    still compiled: Phase::ScalePush (the separate tape push), Answer::Distance24/48/72 (the
//    distance question) and PortObservation::tapeDeltaRad. The scale's input is the one push's
//    pushTravelRad; there is no distance to answer.
//
// ── The phases ──────────────────────────────────────────────────────────────────────
//   WaitImu      unpowered   wait for imu.isReady(), bounded.
//   PushCapture  UNPOWERED   the operator pushes the whole robot FORWARD pushDistance along a
//                            tape, straight. Every candidate whose shaft moved by more than
//                            movedThreshold is a DRIVE port and the DIRECTION it moved is its
//                            SIGN; its travel is kept for the scale. The push must be straight
//                            (straightBound) and long enough (minPushRad). The end that LED the
//                            push is the FRONT: the signs stand as captured. Ports that
//                            did not move are not drive motors and are never touched again.
//                            A candidate whose push reading is NOT FINITE (NaN, ±Inf) did not
//                            "move" either — it gave no reading — so it is not a drive port;
//                            unlike a still one it is NAMED, in the push record (NO READING)
//                            and in warning() (DIAG1 Part B; one predicate, movedOnPush()).
//                            Proven on the host test plant only: on a V5 no push reading is
//                            expected to be non-finite, because ProsMotor screens a failed read
//                            to the last good one first (A4 register HA-08).
//   SideAssign   powered     one member at a time, a brief pulse at a RAMP of levels. On a tank,
//                            driving ONE side forward turns the robot toward the other: LEFT
//                            forward turns it CLOCKWISE, which is NEGATIVE yaw in shulib's
//                            CCW-positive frame (hal/imu.hpp), and RIGHT forward is positive —
//                            reasoned from rigid-body kinematics, unmeasured on a robot (A4
//                            register HA-134). The pulse also CONFIRMS the captured sign — the
//                            pulsed member's own encoder must advance POSITIVE under sign x V —
//                            and a contradiction refuses the run naming the port. A port no level
//                            can turn the robot with is classified (PulseVerdict). Once every
//                            port is decided, each side's scale is pushDistance divided by the
//                            MEDIAN of its members' push travel.
//   SpinTest     powered     spin in place both ways; W = (dR - dL) / dtheta_imu per direction,
//                            averaged. On a floor this measures the EFFECTIVE width, which is
//                            LARGER than a taped one because a turning tank scrubs (A4 register
//                            HA-135). The coupled-side monitor watches both sides throughout; the
//                            wrong-way guard and the start check watch the yaw.
//   Done                     result() is the description; report() is every number behind it.
//
// ── The rules that make it correct, each with the mistake it prevents ───────────────
// 1. ONE PLACE FOR A SIGN, and it moves exactly once. During calibration NO sign is known, so
//    the candidates' adapters are constructed with POSITIVE ports and THIS class applies the
//    captured sign to a member's command and to its reading. The description it produces carries
//    the sign INSIDE the port number, and every later program constructs its adapter from that
//    signed port and never negates again. A calibrator that negated a member whose adapter was
//    ALSO built from a negative port would capture every sign backwards, consistently — invisible
//    on the host, a five-against-four fight on the robot. Hence the precondition: every candidate
//    port handed in must be POSITIVE.
// 2. NO WRAPPED ANGLE TOUCHES SHAFT ARITHMETIC. IMotor::position() is CUMULATIVE (hal/motor.hpp);
//    a math::Angle anywhere in a delta would wrap a two-turn spin to nothing. Shaft travel is
//    units::AngleDim and plain doubles throughout.
// 3. THE IMU'S dtheta IS ACCUMULATED PER TICK through Angle::errorTo, never taken as
//    end-minus-start: a single difference of two wrapped headings cannot see two full turns.
// 4. THE MEDIAN, NOT THE MEAN, for each side's scale — and it is hal::medianInPlace, the SAME
//    function hal::MotorGroup reports position() with. A member that travels short (robot two's
//    port 18, ~20 %, A4 register HA-130) therefore cancels EXACTLY between calibration and run
//    time rather than poisoning the scale or being accepted as the truth of the side. Since
//    DIAG1 Part B that median leaves out a value that is not finite, as the group does; the
//    push's own medians only ever see finite values (a non-finite push reading is not a drive
//    port, rule of the PushCapture phase above), so the one site that can meet one is the
//    spin's side travel, which reads the shafts live.
// 5. THE PUSH IS POWERLESS. Polarity is found with nothing driven, because a pulse on a coupled
//    train whose signs are unknown IS the fight the coupled-side monitor exists to prevent. Only
//    members whose sign is already captured are ever pulsed, one at a time, and a member left
//    out as slipping is never commanded again.
// 6. COAST, NEVER BRAKE, on every refusal, every abort and every pulse's idle members. A
//    calibration that stops with a person standing next to the robot must leave the robot
//    pushable.
// 7. A MISSED TICK IS A STOP. Every powered phase re-commands the same volts every tick, so a
//    caller that stops ticking would leave the motors running. If the clock has advanced by more
//    than commandTimeout since the last tick, the calibrator commands 0 V and refuses. (The C2
//    stalled-pacer lesson, applied here.) The ramp's levels and the spin's step-up are
//    re-commanded every tick like everything else.
//
// Tick-driven and NON-BLOCKING: the caller owns the loop and paces it (10 ms on the robot); the
// operator's answers arrive through answer(), never through a blocking read. Allocation-free
// (every array is sized by hal::kMaxSmartPorts). Raises NO faults — what to do about a refusal is
// the program's policy, exactly as the HAL adapters leave raising to the loop layer. PROS-free
// and sim-free.
//
// ── RUN AGAIN: construct a new one (Part 2F-1) ──────────────────────────────────────
// There is deliberately no reset(). A program offering RUN AGAIN rebuilds the calibrator in
// place — std::optional<DrivetrainCalibrator>::emplace(candidates, ports, imu, clock, config),
// then start() — which carries NOTHING over by construction: no sign, no side, no report row.
// A reset() would have to remember every field added later, and the one it forgot would leak
// between runs. The IMU is already calibrated by then, so WaitImu passes on its first tick.
//
// ── EVERY RUNTIME STRING IN THIS FILE IS ASCII, AND NONE CONTAINS A PERCENT SIGN ────
// Prompts, progress lines, refusals and records are painted on the V5's screen and read over
// USB serial. PROS 4.2.2 draws text through VEXos's own display routines (not LVGL, as this note
// said until 2F-1 — a read of the vendored libpros.a corrected it), whose fonts carry ASCII only,
// so a UTF-8 em dash renders as a missing glyph and as mojibake in a raw `cat` of the user port.
// And PROS hands the already-formatted text to VEXos AS A FORMAT STRING, so a '%' that reaches
// the screen is re-read as a conversion with no argument behind it: the spin-disagreement
// refusal printed two of them until 2F-1 ("pct" now). src/ has used "--" since R3a. The COMMENTS
// in this file keep their typography: they are read in an editor.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>

#include "shulib/core/check.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/imu.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/motor_group.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/teleop/coupled_side_monitor.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::calibration {

/// Radians per degree — the thresholds below are stated in the degrees a person can picture on a
/// panel and converted once, here, rather than written as decimal radians nobody can check.
inline constexpr double kDegreesToRadians = math::Angle::kPi / 180.0;

/// The reproducibility the calibrator CLAIMS for a side's scale: two complete runs of the same
/// procedure on the same drivetrain must agree within this fraction. PROVISIONAL (A4: HA-136) —
/// pinned against the A2 plant under encoder and IMU hostility; a bench session decides whether a
/// real drivetrain holds it.
inline constexpr double kReproScaleTolerance = 0.02;

/// The reproducibility the calibrator CLAIMS for the effective track width, as a fraction. Looser
/// than the scale's because the spin measurement runs through the floor's scrub, which is the one
/// thing the host plant does not model. PROVISIONAL (A4: HA-136).
inline constexpr double kReproWidthTolerance = 0.03;

/// The most pulse levels a ramp may have. Four: the default ramp uses three, and a fourth leaves
/// room to try one more without an array growing.
inline constexpr std::size_t kMaxPulseLevels = 4;

/// The most spin levels the start check may step through.
inline constexpr std::size_t kMaxSpinLevels = 4;

/// The most pulse records one run can produce: every port, at every level.
inline constexpr std::size_t kMaxPulseRecords = hal::kMaxSmartPorts * kMaxPulseLevels;

/// Where a run has got to. A run ends in exactly one of Done or Aborted.
enum class Phase : std::uint8_t {
    Idle,         ///< constructed, start() not yet called
    WaitImu,      ///< waiting for the IMU to finish its boot calibration
    PushCapture,  ///< UNPOWERED: watching the one push for drive ports, their signs and travel
    SideAssign,   ///< powered: a ramped pulse per drive port, naming its side from the yaw sign
    SpinTest,     ///< powered: a spin each way for the effective track width
    Done,         ///< result() is valid
    Aborted,      ///< refused or aborted; reason() says why, every motor is at 0 V + Coast
};

/// What the operator's touch means. Everything a person contributes to a calibration arrives
/// through exactly this enum — there is no blocking prompt anywhere in the class. Never persisted
/// (the record stores no answer), so CAL1 Part B's removal of FrontFirst and BackFirst renumbered
/// Abort safely: nothing on a card changed.
enum class Answer : std::uint8_t {
    Done,   ///< "I have finished pushing" — the end that led the push is the FRONT (CAL1)
    Abort,  ///< stop now
};

/// WHY a run ended in Aborted — a CODE, so a panel can say it in plain words keyed on a value
/// rather than on prose (prose is not an API), and a test can assert it. reason() carries the
/// sentence with the numbers; refusalPort() carries the port where the refusal can name one.
enum class Refusal : std::uint8_t {
    None,                ///< not refused (still running, or Done)
    Aborted,             ///< abort() or Answer::Abort — the operator, or the program on the operator's behalf
    ImuNotReady,         ///< the IMU never finished calibrating within imuReadyTimeout
    PushTooFew,          ///< DONE with fewer than two ports moved by the push
    PushCurved,          ///< the push turned the robot by more than straightBound
    PushTooShort,        ///< the push (or one side of it) turned the shafts less than minPushRad
    RestTimeout,         ///< nothing came to rest within restTimeout (a motor still turning, or the robot still rotating)
    NonDriveMoved,       ///< a motor that did not move on the push (or gave no reading at DONE) moved while one drive port was pulsed
    PulseContradiction,  ///< a pulsed member's own encoder went against its captured sign
    PulseRolled,         ///< its train moved at the top level but the robot did not turn (it rolled, or is off the floor)
    PulseStill,          ///< nothing moved at the top level
    OneSided,            ///< after the pulses, a side has no member
    SpinWrongWay,        ///< the spin turned the wrong way past spinWrongWay — the sides are swapped
    SpinNoStart,         ///< the spin could not start turning even at the top spin level
    SpinTimeout,         ///< the spin started but did not finish its turns within spinTimeout
    SpinFight,           ///< the coupled-side monitor saw a member fight its side during the spin
    SpinWidthInvalid,    ///< a spin direction measured a width that is not a positive number
    SpinDisagree,        ///< the two spin directions disagree by more than spinAgreement
    MissedTick,          ///< tick() was not called within commandTimeout while powered
    OutOfOrder,          ///< an answer arrived that no step was waiting for
    InvalidDescription,  ///< the measured description failed its own validity rules
};

/// What one pulse concluded about the port it drove — one per PulseRecord, and the deciding one on
/// the port's PortObservation.
enum class PulseVerdict : std::uint8_t {
    None,           ///< not pulsed
    Weak,           ///< the yaw stayed under the floor at this level; the next level is tried
    Left,           ///< negative yaw at or past the floor: a LEFT member (HA-134)
    Right,          ///< positive yaw at or past the floor: a RIGHT member
    Contradiction,  ///< its own encoder went against its captured sign — refused
    Decoupled,      ///< top level: its shaft turned, nothing else moved — a slipping gear, LEFT OUT
    Rolled,         ///< top level: other drive ports moved, the robot did not turn — refused
    Still,          ///< top level: nothing moved — refused
};

/// The three facts no measurement can supply, stated by a person before the run.
struct HumanFacts {
    /// Which drivetrain this is. Only Tank is supported: a holonomic drivetrain needs a strafe
    /// scale this procedure does not measure, and the calibrator refuses rather than guess.
    hal::DrivetrainKind kind = hal::DrivetrainKind::Tank;
    /// The cartridge in the drive motors — read off a motor, not inferred (see the header).
    hal::MotorCartridge cartridge = hal::MotorCartridge::Blue;
    /// The IMU's smart port, 1..hal::kMaxSmartPorts. Carried into the description so a later boot
    /// can check the device is still there.
    std::uint8_t imuPort = 0;
    /// The build stamp or session note to record as the description's provenance. Copied into the
    /// description's fixed buffer (truncated to fit) — the calibrator keeps no pointer to it past
    /// start().
    const char* provenance = "";
};

/// Every number the procedure uses. NOT ONE OF THEM IS A MEASUREMENT: all are INVENTED for a
/// first run and carried as fields so a bench session can move them without touching code.
/// PROVISIONAL, all of them (A4: HA-136) — `pushDistance` is a CHOICE rather than a belief.
struct CalibratorConfig {
    /// Shaft travel over the push above which a candidate counts as a DRIVE port — a FINITE
    /// travel: a push reading that is not finite is no reading, and never a drive port (DIAG1
    /// Part B). 5 degrees — the bench tester's MOTOR WATCH number, invented there and carried
    /// here unchanged so the two tools call the same push the same thing. PROVISIONAL
    /// (A4: HA-136).
    units::AngleDim movedThreshold{5.0 * kDegreesToRadians};
    /// |velocity| below which a motor counts as stopped, for the rest windows. PROVISIONAL
    /// (A4: HA-136).
    units::AngularVelocity restFloor{0.05};
    /// |IMU yaw rate| below which the ROBOT counts as still, for the same rest windows (added at
    /// 2F-1: the design always defined rest as "yaw rate and every velocity under a floor", and
    /// the code checked the velocities only, so a robot being turned by hand with its motors
    /// stationary counted as at rest). 2 deg/s. Bounded BELOW by the IMU's rest noise (unmeasured
    /// on a V5; the host's hostile IMU uses 0.5 deg/s, HA-22, so this is 4 sigma there), and
    /// ABOVE by the precondition restYawRateFloor x pulseTime < pulseYawFloor — a drift slow
    /// enough to pass the rest check then adds less than the floor to a pulse, and a side can only
    /// come out WRONG through a leak at least as large as the floor, so a slow drift can never flip
    /// a side on its own. PROVISIONAL (A4: HA-136).
    units::AngularVelocity restYawRateFloor{2.0 * kDegreesToRadians};
    /// How long everything must stay at rest before a pulse or a spin may start — so one pulse's
    /// coast-down cannot be read as the next pulse's motion. PROVISIONAL (A4: HA-136).
    units::Time restSettleTime{0.5};
    /// How long to wait for rest before refusing. Without a bound this wait is the one unbounded
    /// loop in the procedure. PROVISIONAL (A4: HA-136).
    units::Time restTimeout{3.0};
    /// The pulse ramp: one member is pulsed at pulseLevels[0], then — only if the robot's yaw
    /// stayed under pulseYawFloor — at the next level, and so on (added at 2F-1, replacing the
    /// single 3 V `pulseVolts`: one member of five delivers a fifth of its pulse to a coupled
    /// train under the plant's model, and robot two's port 11 turned the robot 0.14-0.17 deg at
    /// 3 V). 3 / 6 / 9 V: the first is the old pulse; 9 V for 0.4 s on ONE motor is a visible
    /// jerk, not a drive. Strictly increasing, each in (0, 12] V. PROVISIONAL (A4: HA-136).
    std::array<units::Voltage, kMaxPulseLevels> pulseLevels{units::Voltage{3.0}, units::Voltage{6.0},
                                                            units::Voltage{9.0}, units::Voltage{0.0}};
    /// How many entries of pulseLevels are used, 1..kMaxPulseLevels. PROVISIONAL (A4: HA-136).
    std::size_t pulseLevelCount = 3;
    /// How long one member is pulsed, at each level. PROVISIONAL (A4: HA-136).
    units::Time pulseTime{0.4};
    /// The |dtheta| a pulse must produce before its sign may be read as a side. PROVISIONAL
    /// (A4: HA-136).
    units::AngleDim pulseYawFloor{2.0 * kDegreesToRadians};
    /// At the top level, a port whose OWN shaft turned at least this far while no other drive
    /// port moved movedThreshold is DECOUPLED — its motor turns but does not drive its gear train
    /// (a slipping or stripped gear) — and is LEFT OUT rather than refused. 20 degrees of shaft:
    /// well past gear backlash, which a train that does not break free still shows. PROVISIONAL
    /// (A4: HA-136).
    units::AngleDim decoupledOwnMin{20.0 * kDegreesToRadians};
    /// The one push's length along the tape: 48 in, two field tiles. A CHOICE, not a belief —
    /// stated here and on the screen, so a wrong distance cannot be answered (there is no
    /// distance question any more). Each side's scale is this divided by the side's median
    /// travel.
    units::Length pushDistance{48.0};
    /// The |dtheta| the push may drift by before it is not a straight push. Also the bound that
    /// keeps a twist from inverting one side's captured signs. PROVISIONAL (A4: HA-136).
    units::AngleDim straightBound{3.0 * kDegreesToRadians};
    /// The least median shaft travel the push must produce — over the moved ports at DONE, and
    /// over each side once the sides are known — one full wheel turn. A shorter push divides the
    /// distance by a number too small to trust. PROVISIONAL (A4: HA-136).
    units::AngleDim minPushRad{2.0 * math::Angle::kPi};
    /// The spin's levels: it starts at spinLevels[0] and steps to the next only when the robot has
    /// not turned spinStartYaw within spinStartTime at the current one (added at 2F-1, replacing
    /// the single 3 V `spinVolts`). 3 / 6 V. Strictly increasing, each in (0, 12] V.
    /// PROVISIONAL (A4: HA-136).
    std::array<units::Voltage, kMaxSpinLevels> spinLevels{units::Voltage{3.0}, units::Voltage{6.0},
                                                          units::Voltage{0.0}, units::Voltage{0.0}};
    /// How many entries of spinLevels are used, 1..kMaxSpinLevels. PROVISIONAL (A4: HA-136).
    std::size_t spinLevelCount = 2;
    /// How long the spin may run at one level without turning the robot spinStartYaw before it
    /// steps up — or, at the top level, refuses: a spin that cannot turn the robot used to stall
    /// for the whole spinTimeout. PROVISIONAL (A4: HA-136).
    units::Time spinStartTime{2.0};
    /// The |yaw| a spin level must reach within spinStartTime to count as turning. PROVISIONAL
    /// (A4: HA-136).
    units::AngleDim spinStartYaw{5.0 * kDegreesToRadians};
    /// How far the spin may turn the WRONG way before it refuses — at once, mid-spin: sides that
    /// are swapped consistently cause no fight inside either side, so the coupled-side monitor is
    /// silent and nothing else would stop the robot spinning the wrong way under power.
    /// PROVISIONAL (A4: HA-136).
    units::AngleDim spinWrongWay{15.0 * kDegreesToRadians};
    /// How many full turns each direction of the spin must cover. More turns average more of the
    /// floor and dilute a single scuff; two is the compromise with how long a person will stand
    /// and watch. PROVISIONAL (A4: HA-136).
    double spinTurns = 2.0;
    /// How long one direction of the spin may take before the run refuses. PROVISIONAL
    /// (A4: HA-136).
    units::Time spinTimeout{20.0};
    /// The largest relative disagreement tolerated between the two directions' track widths.
    /// PROVISIONAL (A4: HA-136).
    double spinAgreement = 0.05;
    /// How long to wait for the IMU's boot calibration. PROVISIONAL (A4: HA-136).
    units::Time imuReadyTimeout{5.0};
    /// How far the clock may advance between two tick() calls before a powered phase treats the
    /// gap as a stalled caller, commands 0 V and refuses. PROVISIONAL (A4: HA-136).
    units::Time commandTimeout{0.1};
    /// The coupled-side fight detector's thresholds for the spin. Defaults are the drive
    /// program's and hal::MotorGroup's — one evaluator, one set of numbers (A4: HA-132/133).
    teleop::SideMonitorConfig monitor{};
};

/// What one run concluded about ONE candidate — the panel's table and the log's record.
struct PortObservation {
    /// The candidate's port as handed in: always POSITIVE (the sign is captured, not given).
    std::int8_t port = 0;
    /// The sign captured from the push (the end that led it is the FRONT): +1, −1, or 0
    /// for a candidate that did not move and is therefore not a drive motor — or whose push
    /// reading was not finite, which is no reading and no sign (the push record says NO READING).
    std::int8_t sign = 0;
    /// −1 LEFT, +1 RIGHT, 0 = not a drive port, not yet decided, or LEFT OUT.
    std::int8_t side = 0;
    /// Shaft travel over the push, radians, as READ (before the sign is applied) — live while the
    /// push is watched, then the number the sign was read off.
    double pushDeltaRad = 0.0;
    /// The same travel sign-corrected (sign × pushDeltaRad): the port's travel in the robot's
    /// forward direction — positive, since the end that led the push is the front.
    /// Exactly +0.0 for a port that is not a drive motor — a still one, and (since DIAG1 Part B)
    /// one whose push reading was not finite (NO READING), where sign × pushDeltaRad would be
    /// 0 × ±Inf or 0 × NaN, a NaN.
    double pushTravelRad = 0.0;
    /// The IMU's accumulated dtheta over this port's DECIDING pulse (its last), radians.
    double pulseYawRad = 0.0;
    /// This port's own sign-corrected shaft travel over that pulse, radians.
    double pulseShaftRad = 0.0;
    /// The level that pulse ran at, volts (0 if the port was not pulsed).
    double pulseLevelVolts = 0.0;
    /// The largest |travel| any OTHER drive port showed over that pulse, radians — its side-mates
    /// through the gear train, and the other side if the robot rolled.
    double pulseOthersRad = 0.0;
    /// What that pulse concluded (None if the port was not pulsed).
    PulseVerdict verdict = PulseVerdict::None;
    /// True when the port was LEFT OUT as decoupled: it moved on the push, it is on the
    /// drivetrain, and it is in no side of the description.
    bool excluded = false;
};

/// One pulse, as the serial record prints it: which port, at which level, and what it did.
struct PulseRecord {
    /// The pulsed port (positive, as handed in).
    std::int8_t port = 0;
    /// Which drive port of the run this was, 1-based.
    std::uint8_t ordinal = 0;
    /// The level, volts.
    double levelVolts = 0.0;
    /// The IMU's accumulated dtheta over the pulse, radians.
    double yawRad = 0.0;
    /// The pulsed member's own sign-corrected shaft travel, radians.
    double ownRad = 0.0;
    /// The largest |travel| any other drive port showed, radians.
    double othersMaxRad = 0.0;
    /// What this pulse concluded.
    PulseVerdict verdict = PulseVerdict::None;
};

/// One direction of the spin, as the serial record prints it.
struct SpinRecord {
    /// +1 counter-clockwise, −1 clockwise.
    double direction = 0.0;
    /// The level the spin finished at, volts (the start check may have stepped it up).
    double levelVolts = 0.0;
    /// The IMU's accumulated dtheta over the direction, radians.
    double yawRad = 0.0;
    /// The LEFT side's travel, inches (its median shaft travel times its scale).
    double leftTravelIn = 0.0;
    /// The RIGHT side's travel, inches.
    double rightTravelIn = 0.0;
    /// This direction's effective track width, inches.
    double widthIn = 0.0;
    /// How long the direction took, seconds.
    double seconds = 0.0;
};

/// Every number behind a result — for the panel, for the log, and for a human deciding whether to
/// believe the run. Populated as far as the run got, so it is worth reading after a REFUSAL too.
struct CalibrationReport {
    /// One row per candidate, in the order they were handed in.
    PortObservation ports[hal::kMaxSmartPorts]{};
    /// How many rows of `ports` are real.
    std::uint8_t portCount = 0;
    /// How many of them moved on the push and are therefore drive motors.
    std::uint8_t drivePortCount = 0;
    /// The push's length, inches — CalibratorConfig::pushDistance, recorded once the push is taken.
    double pushDistanceIn = 0.0;
    /// The IMU's dtheta over the push, radians, signed — how straight the push actually was.
    double pushStraightnessRad = 0.0;
    /// The LEFT side's median push travel, radians (the scale's denominator).
    double leftMedianRad = 0.0;
    /// The RIGHT side's median push travel, radians.
    double rightMedianRad = 0.0;
    /// The LEFT side's measured scale, inches per radian — available once the sides are known, so
    /// a panel (and a host test) can read it without waiting for the spin.
    double leftInchesPerRadian = 0.0;
    /// The RIGHT side's measured scale, inches per radian.
    double rightInchesPerRadian = 0.0;
    /// max − min of the LEFT side's members' push travel, radians: how far apart a side's motors
    /// read. Robot two's port 18 is expected to show here (A4 register HA-130).
    double leftSpreadRad = 0.0;
    /// max − min of the RIGHT side's members' push travel, radians.
    double rightSpreadRad = 0.0;
    /// The effective track width from the CCW spin, inches.
    double widthCcwIn = 0.0;
    /// The effective track width from the CW spin, inches.
    double widthCwIn = 0.0;
    /// |Wccw − Wcw| / their mean — the agreement the run was judged on.
    double widthDisagreement = 0.0;
    /// Every pulse the run drove, in order, at every level — one serial line each.
    PulseRecord pulses[kMaxPulseRecords]{};
    /// How many entries of `pulses` are real. A program printing the record prints the ones added
    /// since it last looked.
    std::uint8_t pulseCount = 0;
    /// The spin, one record per finished direction (0 = CCW, 1 = CW).
    SpinRecord spins[2]{};
    /// How many entries of `spins` are real.
    std::uint8_t spinCount = 0;
    /// The ports LEFT OUT as decoupled, in the order they were found.
    std::int8_t excludedPorts[hal::kMaxSmartPorts]{};
    /// How many entries of `excludedPorts` are real.
    std::uint8_t excludedCount = 0;
};

/// What a panel needs to paint this tick: where the run is, what the operator must DO, a live line
/// of prose for a log, and — since 2F-1 — the NUMBERS behind that prose, so a screen can show them
/// large without parsing text.
struct CalibratorStatus {
    /// The phase as of the last tick(), answer() or abort().
    Phase phase = Phase::Idle;
    /// The instruction for the person — or the refusal's one-line summary, once aborted. Owned by
    /// the calibrator and valid until its next tick(), answer() or abort(); at most 44 characters
    /// (the width the Part 2 program printed it at — the Part 2F-2 program paints
    /// calibrate_panel.hpp's words instead and does not show this; a caller that does has 44 to
    /// allow for).
    const char* prompt = "not started";
    /// A live line of this phase's numbers, NUL-terminated and bounded — for a serial log.
    char progress[96]{};
    /// Seconds since the current phase began, while a run is under way (the IMU wait's live
    /// number); once the run has ended, since the last running phase began.
    double phaseSeconds = 0.0;
    /// How many candidates were handed in.
    std::uint8_t candidates = 0;
    /// Candidates whose push travel is past movedThreshold (a finite travel — the same predicate
    /// the finish and pushReady() use) — live while the push is watched, the captured count after
    /// it.
    std::uint8_t movedNow = 0;
    /// The median |push travel| of the moved candidates, radians (0 with none moved).
    double pushTravelRad = 0.0;
    /// pushTravelRad ≥ minPushRad — the push's "far enough", the SAME predicate DONE is judged on.
    bool pushFarEnough = false;
    /// Every motor under restFloor AND the IMU's yaw rate under restYawRateFloor, this tick.
    bool atRest = false;
    /// The yaw accumulated over the push, radians — live while it is watched, then as captured.
    double driftRad = 0.0;
    /// Which drive port of pulseTotal is being tested, 1-based (0 before the pulses start).
    std::uint8_t pulseOrdinal = 0;
    /// How many drive ports the pulses will test.
    std::uint8_t pulseTotal = 0;
    /// The port under test (0 when none).
    std::int8_t pulsePort = 0;
    /// The ramp level being tried, volts (0 when none).
    double pulseLevelVolts = 0.0;
    /// True while a pulse is being DRIVEN; false in the rest windows around it.
    bool pulsing = false;
    /// The yaw since the current pulse, spin direction or rest window began, radians.
    double liveYawRad = 0.0;
    /// +1 while spinning counter-clockwise, −1 clockwise, 0 when not spinning.
    double spinDirection = 0.0;
    /// Turns covered in the current spin direction, signed by that direction (negative = the
    /// wrong way).
    double spinTurns = 0.0;
    /// The spin level in force, volts (0 when not spinning).
    double spinLevelVolts = 0.0;
    /// Drive ports decided LEFT so far.
    std::uint8_t leftCount = 0;
    /// Drive ports decided RIGHT so far.
    std::uint8_t rightCount = 0;
    /// Drive ports LEFT OUT as decoupled so far.
    std::uint8_t excludedCount = 0;
};

/// The tick-driven state machine described in the header. Non-owning: the motors, the IMU, the
/// clock and the arrays the spans view must all outlive it. Owns no loop, allocates nothing,
/// blocks nowhere, raises no faults. Not copyable or movable (it holds references); RUN AGAIN is a
/// new object (the header).
class DrivetrainCalibrator {
public:
    /// `candidates`: every motor the brain found, as adapters built with POSITIVE ports (rule 1 in
    /// the header). `ports`: their physical port numbers, parallel to `candidates`, each in
    /// 1..hal::kMaxSmartPorts and POSITIVE. Preconditions, all of them the caller's contract: the
    /// two spans are the same length; there are at least two candidates and at most
    /// hal::kMaxSmartPorts; no candidate is null; every port is positive, in range, and listed
    /// once; every config threshold is positive, the level ramps are strictly increasing within
    /// (0, 12] V, and restYawRateFloor x pulseTime < pulseYawFloor (CalibratorConfig).
    DrivetrainCalibrator(std::span<hal::IMotor* const> candidates,
                         std::span<const std::int8_t> ports, hal::IImu& imu, hal::IClock& clock,
                         const CalibratorConfig& config = {})
        : motors_{candidates}, ports_{ports}, imu_{imu}, clock_{clock}, cfg_{config} {
        SHULIB_PRECONDITION(candidates.size() == ports.size(),
                            "DrivetrainCalibrator: one port per candidate motor");
        SHULIB_PRECONDITION(candidates.size() >= 2,
                            "DrivetrainCalibrator: at least two candidate motors are needed");
        SHULIB_PRECONDITION(candidates.size() <= hal::kMaxSmartPorts,
                            "DrivetrainCalibrator: more candidates than the brain has ports");
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            SHULIB_PRECONDITION(candidates[i] != nullptr,
                                "DrivetrainCalibrator: a candidate motor is null");
            SHULIB_PRECONDITION(ports[i] > 0
                                    && ports[i] <= static_cast<std::int8_t>(hal::kMaxSmartPorts),
                                "DrivetrainCalibrator: every candidate port must be POSITIVE and "
                                "in 1..21 -- the calibrator applies the sign, so an adapter built "
                                "with a negative port would apply it twice");
            for (std::size_t j = i + 1; j < candidates.size(); ++j) {
                SHULIB_PRECONDITION(ports[j] != ports[i],
                                    "DrivetrainCalibrator: a port is listed twice");
            }
        }
        SHULIB_PRECONDITION(cfg_.movedThreshold.value() > 0.0
                                && cfg_.restFloor.value() > 0.0
                                && cfg_.restYawRateFloor.value() > 0.0
                                && cfg_.restSettleTime.value() >= 0.0
                                && cfg_.restTimeout.value() > cfg_.restSettleTime.value()
                                && cfg_.pulseTime.value() > 0.0
                                && cfg_.pulseYawFloor.value() > 0.0
                                && cfg_.decoupledOwnMin.value() > 0.0
                                && cfg_.pushDistance.value() > 0.0
                                && cfg_.straightBound.value() > 0.0
                                && cfg_.minPushRad.value() > 0.0
                                && cfg_.spinStartTime.value() > 0.0
                                && cfg_.spinStartTime.value() < cfg_.spinTimeout.value()
                                && cfg_.spinStartYaw.value() > 0.0
                                && cfg_.spinWrongWay.value() > 0.0
                                && cfg_.spinTurns > 0.0 && cfg_.spinTimeout.value() > 0.0
                                && cfg_.spinAgreement > 0.0 && cfg_.imuReadyTimeout.value() > 0.0
                                && cfg_.commandTimeout.value() > 0.0,
                            "DrivetrainCalibrator: every config threshold must be positive (and "
                            "restTimeout must exceed restSettleTime, spinStartTime be under "
                            "spinTimeout)");
        SHULIB_PRECONDITION(levelsValid(cfg_.pulseLevels.data(), cfg_.pulseLevelCount,
                                        kMaxPulseLevels),
                            "DrivetrainCalibrator: pulseLevels must be 1..4 strictly increasing "
                            "levels in (0, 12] V");
        SHULIB_PRECONDITION(levelsValid(cfg_.spinLevels.data(), cfg_.spinLevelCount,
                                        kMaxSpinLevels),
                            "DrivetrainCalibrator: spinLevels must be 1..4 strictly increasing "
                            "levels in (0, 12] V");
        SHULIB_PRECONDITION(cfg_.restYawRateFloor.value() * cfg_.pulseTime.value()
                                < cfg_.pulseYawFloor.value(),
                            "DrivetrainCalibrator: restYawRateFloor x pulseTime must be under "
                            "pulseYawFloor, or a drift slow enough to pass the rest check could "
                            "cross the yaw floor on its own");
        n_ = motors_.size();
        report_.portCount = static_cast<std::uint8_t>(n_);
        for (std::size_t i = 0; i < n_; ++i) {
            report_.ports[i].port = ports_[i];
        }
        status_.candidates = static_cast<std::uint8_t>(n_);
    }

    /// Not copyable: it holds references to hardware it does not own, and a copy would be a second
    /// state machine commanding the same motors.
    DrivetrainCalibrator(const DrivetrainCalibrator&) = delete;
    /// Not copy-assignable, for the same reason.
    DrivetrainCalibrator& operator=(const DrivetrainCalibrator&) = delete;

    /// Begin a run from the three stated facts. Preconditions (caller contract, not run-time
    /// conditions): the calibrator is Idle, the kind is Tank, the cartridge is a real cartridge,
    /// and the IMU port is in range and is not one of the candidate ports. Leaves every candidate
    /// at 0 V + Coast so the robot can be pushed, and enters WaitImu.
    void start(const HumanFacts& facts) {
        SHULIB_PRECONDITION(phase_ == Phase::Idle,
                            "DrivetrainCalibrator::start: a run is already under way");
        SHULIB_PRECONDITION(facts.kind == hal::DrivetrainKind::Tank,
                            "DrivetrainCalibrator::start: only a tank drivetrain is supported "
                            "(a holonomic one needs a strafe scale this procedure never takes)");
        SHULIB_PRECONDITION(facts.cartridge == hal::MotorCartridge::Red
                                || facts.cartridge == hal::MotorCartridge::Green
                                || facts.cartridge == hal::MotorCartridge::Blue,
                            "DrivetrainCalibrator::start: the cartridge must be stated");
        SHULIB_PRECONDITION(facts.imuPort >= 1
                                && facts.imuPort <= static_cast<std::uint8_t>(hal::kMaxSmartPorts),
                            "DrivetrainCalibrator::start: the IMU port must be in 1..21");
        for (std::size_t i = 0; i < n_; ++i) {
            SHULIB_PRECONDITION(ports_[i] != static_cast<std::int8_t>(facts.imuPort),
                                "DrivetrainCalibrator::start: the IMU port is also a candidate "
                                "motor port");
        }
        facts_ = facts;
        safeState();
        enter(Phase::WaitImu, Step::None);
        updateStatus();
    }

    /// One control tick. Cheap and non-blocking; the caller paces it (10 ms on the robot). A no-op
    /// once the run has ended.
    void tick() {
        if (phase_ == Phase::Idle || phase_ == Phase::Done || phase_ == Phase::Aborted) {
            return;
        }
        const units::Time now = clock_.now();
        if (ticked_ && poweredPhase()
            && (now.value() - lastTick_.value()) > cfg_.commandTimeout.value()) {
            // Rule 7: a powered phase holds its command between ticks, so a caller that stops
            // ticking leaves the motors running. Stop them and say so.
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "a tick was missed: %.3f s since the last one, ceiling %.3f s -- "
                          "motors stopped",
                          now.value() - lastTick_.value(), cfg_.commandTimeout.value());
            refuse(Refusal::MissedTick, 0, msg);
            return;
        }
        lastTick_ = now;
        ticked_ = true;
        accumulateYaw();  // rule 3: per tick, through errorTo — never end minus start

        switch (phase_) {
            case Phase::WaitImu: tickWaitImu(now); break;
            case Phase::PushCapture: tickPushCapture(); break;
            case Phase::SideAssign: tickSideAssign(now); break;
            case Phase::SpinTest: tickSpinTest(now); break;
            default: break;
        }
        updateStatus();
    }

    /// The operator's touch. Abort is accepted in any phase; every other answer is accepted only
    /// by the step that is waiting for it, and an answer that arrives out of order REFUSES the run
    /// naming both — a touch that silently did nothing would leave the operator believing a step
    /// had been taken. Precondition: a run is under way (anything but Idle/Done/Aborted), except
    /// for Abort, which is always allowed. Unchanged at 2F-1: DONE before the push is ready is
    /// still ACCEPTED here and judged by the finish (which refuses too few ports, a curved push or
    /// a short one); a panel keeps DONE off the screen until pushReady() — the gate is a readiness
    /// query, not a new refusal.
    void answer(Answer a) {
        if (a == Answer::Abort) {
            abort("aborted by the operator");
            return;
        }
        SHULIB_PRECONDITION(phase_ != Phase::Idle && phase_ != Phase::Done
                                && phase_ != Phase::Aborted,
                            "DrivetrainCalibrator::answer: no run is under way");
        switch (step_) {
            case Step::PushWatching:
                if (a == Answer::Done) {
                    finishPushCapture();  // CAL1: the leading end IS the front (no question)
                    updateStatus();
                    return;
                }
                break;
            default: break;
        }
        char msg[kReasonCap];
        std::snprintf(msg, sizeof msg,
                      "%s arrived while the run was %s -- nothing was recorded",
                      answerWord(a), waitingWord());
        refuse(Refusal::OutOfOrder, 0, msg);
    }

    /// Stop now: every candidate to 0 V + Coast (never Brake — rule 6), phase Aborted with
    /// Refusal::Aborted, `why` recorded as the reason. Allowed from ANY phase, including Done,
    /// where it discards a result the caller has not copied — call it only when abandoning the run
    /// is what is meant.
    void abort(const char* why = "aborted by the operator") {
        safeState();
        std::snprintf(reason_, sizeof reason_, "%s", why != nullptr ? why : "aborted");
        refusal_ = Refusal::Aborted;
        refusalPort_ = 0;
        phase_ = Phase::Aborted;
        step_ = Step::None;
        updateStatus();
    }

    /// Where the run has got to.
    [[nodiscard]] Phase phase() const noexcept { return phase_; }

    /// What a panel paints this tick (see CalibratorStatus).
    [[nodiscard]] const CalibratorStatus& status() const noexcept { return status_; }

    /// Why the run was refused or aborted — empty while it is still going or if it finished. Every
    /// refusal names its cause, the ones that can name a PORT do, and none contains a '%' (the
    /// header). The code is refusal().
    [[nodiscard]] const char* reason() const noexcept { return reason_; }

    /// Which refusal ended the run — Refusal::None while it runs or once it is Done.
    [[nodiscard]] Refusal refusal() const noexcept { return refusal_; }

    /// The port the refusal names, positive — or 0 when it names none (a push too short, a spin
    /// that disagreed, an abort).
    [[nodiscard]] int refusalPort() const noexcept { return refusalPort_; }

    /// The warnings a run that is NOT refused still owes the operator, joined by " | " — one
    /// sentence per port LEFT OUT as decoupled ("port 18 turns but does not move the robot -- its
    /// gear is slipping; it is left out"), and (since DIAG1 Part B) one per candidate whose push
    /// reading was not finite at DONE ("port 13 gave no reading at DONE -- not counted as a drive
    /// port; if it drives the robot, check it and run again"). Empty when there is none. Bounded:
    /// a sentence that does not fit is cut, and the report still names every port. ASCII, no '%'.
    [[nodiscard]] const char* warning() const noexcept { return warning_; }

    /// The measured description. Precondition: phase() == Done — a refused run has no result, and
    /// asking for one is a caller bug, not a value to be handed back.
    [[nodiscard]] const hal::DrivetrainDescription& result() const {
        SHULIB_PRECONDITION(phase_ == Phase::Done,
                            "DrivetrainCalibrator::result: the run did not finish (phase is not "
                            "Done) -- read reason() instead");
        return result_;
    }

    /// Every number behind the run, filled as far as the run got — worth reading after a refusal.
    [[nodiscard]] const CalibrationReport& report() const noexcept { return report_; }

    /// The config in force.
    [[nodiscard]] const CalibratorConfig& config() const noexcept { return cfg_; }

    /// True when answer(`a`) would be ACCEPTED right now — i.e. when a panel may OFFER it.
    /// Abort is always accepted; everything else only by the step that is waiting for it.
    ///
    /// ── WHY THIS EXISTS (added at R3d Part 2, for the calibrate program's button row) ──
    /// answer() REFUSES an out-of-order answer and ends the run, which is right (a touch that
    /// silently did nothing would leave the operator believing a step had been taken) — but it
    /// makes a wrong BUTTON destructive. A panel therefore has to know which answers are live,
    /// and Phase alone cannot tell it: PushCapture is two steps (watching, then awaiting
    /// FRONT/BACK). Before this query the only public signal was the PROSE of status().prompt,
    /// and prose is not an API.
    ///
    /// It is a PURE QUERY over the same step answer() switches on. It duplicates that switch, so
    /// the anti-drift device is a test rather than a comment: drivetrain_calibrator_test.cpp case
    /// 21 drives the run to each waiting step and asserts, for EVERY Answer, that accepts()
    /// agrees with what answer() actually does on a fresh run. accepts() says whether an answer
    /// is LEGAL; pushReady() says whether DONE is WISE.
    [[nodiscard]] bool accepts(Answer a) const noexcept {
        if (a == Answer::Abort) {
            return true;  // abort is accepted in every phase, including Idle and Done
        }
        switch (step_) {
            case Step::PushWatching: return a == Answer::Done;
            default: return false;
        }
    }

    /// True when the push is PLAUSIBLY COMPLETE, so a panel may OFFER DONE (added at 2F-1 — two of
    /// robot two's five starts ended because DONE was touched before any push, on a screen that
    /// offered it with nothing moved). All three of: at least two candidates past movedThreshold
    /// (a FINITE travel — the finish's own movedOnPush() predicate, so a candidate with no reading
    /// is counted by neither); their median |travel| at least minPushRad (the push's "far
    /// enough"); and everything at rest — every motor under restFloor AND the IMU's yaw rate
    /// under restYawRateFloor. Read LIVE from the motors and the IMU, through the SAME helpers
    /// the finish uses (pushSummary(), atRest()), so the gate cannot drift from the finish: DONE
    /// with the first two false is refused by the finish (PushTooFew, PushTooShort) under the
    /// same predicate. At-rest is the one clause the finish does NOT refuse on (the brief's "a
    /// readiness query, not a new refusal"): the gate is STRICTER than the finish, never looser.
    /// False outside the push.
    [[nodiscard]] bool pushReady() const {
        if (step_ != Step::PushWatching) {
            return false;
        }
        const PushSummary s = pushSummary();
        return s.moved >= 2 && s.medianRad >= cfg_.minPushRad.value() && atRest();
    }

private:
    /// The sub-step inside a phase. Private: a caller steers by Phase and Answer alone.
    enum class Step : std::uint8_t {
        None,
        PushWatching,
        SideRest,
        SidePulse,
        SpinRestBefore,
        SpinCcw,
        SpinRestMid,
        SpinCw,
        SpinRestAfter,
    };

    /// What the push has done so far, read live: how many candidates moved past the threshold and
    /// the median |travel| of those that did.
    struct PushSummary {
        std::uint8_t moved = 0;
        double medianRad = 0.0;
    };

    // ── phase bodies ───────────────────────────────────────────────────────────────

    void tickWaitImu(units::Time now) {
        if (imu_.isReady()) {
            baselineAll();
            enter(Phase::PushCapture, Step::PushWatching);
            return;
        }
        if (now.value() - phaseStart_.value() > cfg_.imuReadyTimeout.value()) {
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "the IMU never became ready within %.1f s -- is it plugged into port "
                          "%d?",
                          cfg_.imuReadyTimeout.value(), static_cast<int>(facts_.imuPort));
            refuse(Refusal::ImuNotReady, 0, msg);
        }
    }

    void tickPushCapture() {
        if (step_ != Step::PushWatching) {
            return;  // awaiting FRONT/BACK: the push was taken at DONE and must not move now
        }
        for (std::size_t i = 0; i < n_; ++i) {
            report_.ports[i].pushDeltaRad = motors_[i]->position().value() - base_[i];
        }
    }

    void tickSideAssign(units::Time now) {
        if (step_ == Step::SideRest) {
            if (tickRest(now, static_cast<int>(ports_[pulseIndex_]))) {
                beginPulse(now);
            }
            return;
        }
        // Step::SidePulse — powered, one member, re-commanded every tick (rule 7).
        commandOnly(pulseIndex_, static_cast<double>(sign_[pulseIndex_]) * currentPulseVolts());
        for (std::size_t j = 0; j < n_; ++j) {
            // Only the NON-DRIVE candidates must be still: the pulsed member's side-mates are
            // BACK-DRIVEN through the gear train and are expected to turn, the other side may
            // roll along, and a port LEFT OUT as decoupled is still on its train and is dragged
            // by it. A motor that is not on the drivetrain at all turning while one drive port is
            // pulsed means the push was wrong about it, or something else is being driven.
            if (j == pulseIndex_ || drive_[j]) {
                continue;
            }
            if (std::abs(motors_[j]->velocity().value()) > cfg_.restFloor.value()) {
                // Two different stories behind the same refusal: a port that read a number and
                // did not move on the push, and (DIAG1 Part B) a port that gave NO READING at
                // DONE and so was never counted as a drive port — possibly a drive motor whose
                // reading came back, back-driven now by its train (the net the no-reading rule
                // relies on). The second must not be told it "did not move on the push": nobody
                // knows that it did not. Both sentences fit the Aborted screen's three 50-column
                // lines (calibrate_panel.hpp), whose own words say "check the port shown".
                const bool noReading = !hal::isUsableReading(report_.ports[j].pushDeltaRad);
                char msg[kReasonCap];
                std::snprintf(msg, sizeof msg,
                              noReading ? "port %d moved (%.3f rad/s) while only port %d was "
                                          "pulsed -- it gave no reading at DONE, so it was not "
                                          "counted as a drive port"
                                        : "port %d moved (%.3f rad/s) while only port %d was "
                                          "pulsed -- it did not move on the push, so it is not "
                                          "a drive motor and must be still",
                              static_cast<int>(ports_[j]), motors_[j]->velocity().value(),
                              static_cast<int>(ports_[pulseIndex_]));
                refuse(Refusal::NonDriveMoved, static_cast<int>(ports_[j]), msg);
                return;
            }
        }
        if (now.value() - stepStart_.value() >= cfg_.pulseTime.value()) {
            finishPulse();
        }
    }

    void tickSpinTest(units::Time now) {
        switch (step_) {
            case Step::SpinRestBefore:
            case Step::SpinRestMid:
            case Step::SpinRestAfter: {
                if (tickRest(now, 0)) {
                    if (step_ == Step::SpinRestBefore) {
                        beginSpin(now, +1.0, Step::SpinCcw);
                    } else if (step_ == Step::SpinRestMid) {
                        beginSpin(now, -1.0, Step::SpinCw);
                    } else {
                        finishDone();
                    }
                }
                return;
            }
            default: break;
        }
        // Spinning: re-command every tick (rule 7), watch both sides for a fight, watch the yaw
        // (the wrong way, the start, the finish), watch the clock.
        commandSpin();
        if (fightDuringSpin()) {
            return;
        }
        const double directional = spinDirection_ * yaw_;
        if (directional < -cfg_.spinWrongWay.value()) {
            // SAFETY (2F-1): a consistently swapped side set makes no fight inside either side,
            // so the monitor above is silent; without this the robot would spin the WRONG way
            // under power until spinTimeout.
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "the robot spun the WRONG way: %.1f deg against the %s spin it was "
                          "driven for (bound %.1f deg) -- the sides are swapped",
                          -directional / kDegreesToRadians, spinDirection_ > 0.0 ? "CCW" : "CW",
                          cfg_.spinWrongWay.value() / kDegreesToRadians);
            refuse(Refusal::SpinWrongWay, 0, msg);
            return;
        }
        const double target = cfg_.spinTurns * 2.0 * math::Angle::kPi;
        if (directional >= target) {
            finishSpin(now);
            return;
        }
        if (!spinStarted_) {
            // START CHECK (2F-1): a spin that cannot turn the robot used to stall at its one
            // level for the whole spinTimeout. Each level gets spinStartTime to turn the robot
            // spinStartYaw (either way — the direction is the wrong-way guard's business, and
            // stepping UP a spin that is already going the wrong way would only make it worse);
            // then the next level; then a refusal.
            if (std::abs(yaw_ - levelStartYaw_) >= cfg_.spinStartYaw.value()) {
                spinStarted_ = true;
            } else if (now.value() - levelStart_.value() >= cfg_.spinStartTime.value()) {
                if (spinLevelIndex_ + 1 < cfg_.spinLevelCount) {
                    ++spinLevelIndex_;
                    levelStart_ = now;
                    levelStartYaw_ = yaw_;
                } else {
                    char msg[kReasonCap];
                    std::snprintf(msg, sizeof msg,
                                  "the drive cannot turn the robot: the %s spin turned it "
                                  "%.1f deg in %.1f s at %.1f V, the top spin level (need %.1f "
                                  "deg) -- is it pinned, or on blocks?",
                                  spinDirection_ > 0.0 ? "CCW" : "CW",
                                  std::abs(yaw_ - levelStartYaw_) / kDegreesToRadians,
                                  now.value() - levelStart_.value(), currentSpinVolts(),
                                  cfg_.spinStartYaw.value() / kDegreesToRadians);
                    refuse(Refusal::SpinNoStart, 0, msg);
                    return;
                }
            }
        }
        if (now.value() - stepStart_.value() > cfg_.spinTimeout.value()) {
            char msg[kReasonCap];
            // DIRECTIONAL turns (2F-1; it printed |yaw| before, which would report a spin going
            // the wrong way as progress — unreachable now behind the wrong-way guard, and still
            // the honest number).
            std::snprintf(msg, sizeof msg,
                          "the %s spin reached only %.2f of %.2f turns in %.1f s",
                          spinDirection_ > 0.0 ? "CCW" : "CW",
                          spinDirection_ * yaw_ / (2.0 * math::Angle::kPi) + 0.0, cfg_.spinTurns,
                          cfg_.spinTimeout.value());
            refuse(Refusal::SpinTimeout, 0, msg);
        }
    }

    /// One tick of a rest window, shared by the pulses and the spin. Returns true once everything
    /// has been at rest (motors AND the robot's yaw rate) for restSettleTime; refuses on
    /// restTimeout, naming the motor that is still turning or saying the robot is still rotating.
    /// `nextPort` is the port about to be pulsed, or 0 around the spin.
    [[nodiscard]] bool tickRest(units::Time now, int nextPort) {
        if (atRest()) {
            if (!resting_) {
                resting_ = true;
                restSince_ = now;
            }
            return now.value() - restSince_.value() >= cfg_.restSettleTime.value();
        }
        resting_ = false;
        if (now.value() - stepStart_.value() > cfg_.restTimeout.value()) {
            char where[40];
            if (nextPort > 0) {
                std::snprintf(where, sizeof where, "before pulsing port %d", nextPort);
            } else {
                std::snprintf(where, sizeof where, "around the spin");
            }
            const std::size_t m = firstMoving();
            char msg[kReasonCap];
            if (m < n_) {
                std::snprintf(msg, sizeof msg,
                              "port %d is still turning at %.3f rad/s after %.1f s of waiting "
                              "for rest %s -- a motor that reports motion while nothing is "
                              "driven is a frozen reading or a moving robot",
                              static_cast<int>(ports_[m]), motors_[m]->velocity().value(),
                              cfg_.restTimeout.value(), where);
                refuse(Refusal::RestTimeout, static_cast<int>(ports_[m]), msg);
            } else {
                std::snprintf(msg, sizeof msg,
                              "the robot is still rotating at %.1f deg/s after %.1f s of waiting "
                              "for rest %s, with every motor still -- hands off the robot, and "
                              "keep it on the floor",
                              imu_.yawRate().value() / kDegreesToRadians,
                              cfg_.restTimeout.value(), where);
                refuse(Refusal::RestTimeout, 0, msg);
            }
        }
        return false;
    }

    // ── phase transitions ──────────────────────────────────────────────────────────

    /// DONE: every candidate that moved by more than the threshold is a drive port and the
    /// DIRECTION it moved is its sign (MOTOR WATCH's rule, in the library); then the push must
    /// have been straight and long enough. "Moved" is movedOnPush() — the SAME predicate
    /// pushSummary() counts with — so a candidate whose push reading is not finite is not a drive
    /// port, and it is named in warning() rather than dropped in silence (DIAG1 Part B).
    void finishPushCapture() {
        std::uint8_t moved = 0;
        for (std::size_t i = 0; i < n_; ++i) {
            const double d = motors_[i]->position().value() - base_[i];
            report_.ports[i].pushDeltaRad = d;
            drive_[i] = movedOnPush(d);
            sign_[i] = drive_[i] ? static_cast<std::int8_t>(d > 0.0 ? 1 : -1) : std::int8_t{0};
            report_.ports[i].sign = sign_[i];
            if (drive_[i]) {
                ++moved;
            }
            if (!hal::isUsableReading(d)) {
                appendWarning(static_cast<int>(ports_[i]), Warning::NoReading);
            }
        }
        const PushSummary s = pushSummary();  // the SAME helper pushReady() reads
        report_.drivePortCount = moved;
        report_.pushDistanceIn = cfg_.pushDistance.value();
        report_.pushStraightnessRad = yaw_;
        if (s.moved < 2) {
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "the push moved %d port(s); a drivetrain needs at least 2 -- push the "
                          "WHOLE robot %.0f in along its length, then DONE",
                          static_cast<int>(s.moved), cfg_.pushDistance.value());
            refuse(Refusal::PushTooFew, 0, msg);
            return;
        }
        if (std::abs(yaw_) > cfg_.straightBound.value()) {
            // Also the push-yaw bound the sign capture used to lack: a twist big enough could
            // roll one side backward and invert every sign on it.
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "the push curved by %.2f deg (bound %.2f deg) -- push it again, "
                          "straight",
                          yaw_ / kDegreesToRadians, cfg_.straightBound.value() / kDegreesToRadians);
            refuse(Refusal::PushCurved, 0, msg);
            return;
        }
        if (s.medianRad < cfg_.minPushRad.value()) {
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "the drive shafts turned only %.2f rad over the push; at least %.2f "
                          "(one wheel turn) is needed -- push it the full %.0f in",
                          s.medianRad, cfg_.minPushRad.value(), cfg_.pushDistance.value());
            refuse(Refusal::PushTooShort, 0, msg);
            return;
        }
        applyFront();  // CAL1 (the team lead, 2026-10-06): the end that LED the push is the front
    }

    /// The front is the end that LED the push (the team lead's ruling, 2026-10-06): the captured
    /// signs stand as captured. A push that led with the other end gives every sign and both sides
    /// the other way round — no later step could see it (every member agrees with every other) —
    /// which is why a program compares the result with the card's record (front_check.hpp).
    void applyFront() {
        for (std::size_t i = 0; i < n_; ++i) {
            report_.ports[i].sign = sign_[i];
            // A port with no sign has no travel: +0.0, not 0 × its delta — for a candidate with
            // no reading that product is 0 × ±Inf or 0 × NaN, a NaN (DIAG1 Part B review round).
            report_.ports[i].pushTravelRad =
                sign_[i] != 0 ? static_cast<double>(sign_[i]) * report_.ports[i].pushDeltaRad
                              : 0.0;
        }
        pulseIndex_ = 0;
        levelIndex_ = 0;
        pulseOrdinal_ = 1;
        if (!advanceToNextDrivePort()) {
            // Unreachable: finishPushCapture demands >= 2 drive ports before this step exists.
            refuse(Refusal::PushTooFew, 0, "no drive port to pulse");
            return;
        }
        enter(Phase::SideAssign, Step::SideRest);
    }

    /// Baseline every shaft and the heading, then start driving this member at this level.
    void beginPulse(units::Time now) {
        baselineAll();
        stepStart_ = now;
        step_ = Step::SidePulse;
    }

    /// The pulse is over: record it, then read the side off the yaw's sign (after confirming the
    /// member's own encoder agreed with the captured sign) — or step up the ramp — or, at the top
    /// of the ramp, classify the port.
    void finishPulse() {
        const std::size_t i = pulseIndex_;
        const double level = currentPulseVolts();
        commandOnly(i, 0.0);  // 0 V + Coast on every candidate
        const double own = static_cast<double>(sign_[i]) * (motors_[i]->position().value() - base_[i]);
        double others = 0.0;
        for (std::size_t j = 0; j < n_; ++j) {
            if (j != i && drive_[j]) {
                others = std::max(others, std::abs(motors_[j]->position().value() - base_[j]));
            }
        }
        const bool lastLevel = levelIndex_ + 1 >= cfg_.pulseLevelCount;
        PulseVerdict verdict = PulseVerdict::Weak;
        if (std::abs(yaw_) >= cfg_.pulseYawFloor.value()) {
            verdict = own > 0.0 ? (yaw_ < 0.0 ? PulseVerdict::Left : PulseVerdict::Right)
                                : PulseVerdict::Contradiction;
        } else if (own <= -cfg_.movedThreshold.value()) {
            verdict = PulseVerdict::Contradiction;  // it clearly went backward, turn or no turn
        } else if (lastLevel) {
            verdict = others >= cfg_.movedThreshold.value()    ? PulseVerdict::Rolled
                      : own >= cfg_.decoupledOwnMin.value() ? PulseVerdict::Decoupled
                                                            : PulseVerdict::Still;
        }
        recordPulse(i, level, own, others, verdict);

        switch (verdict) {
            case PulseVerdict::Left:
            case PulseVerdict::Right:
                // Rule: LEFT alone drives the robot CLOCKWISE, which is NEGATIVE yaw in the
                // CCW-positive canonical frame; RIGHT alone is positive (A4 register HA-134 —
                // reasoned, and settled by the first pulses on a real floor, not by anything the
                // host plant can show).
                side_[i] = verdict == PulseVerdict::Left ? std::int8_t{-1} : std::int8_t{+1};
                report_.ports[i].side = side_[i];
                nextPort();
                return;
            case PulseVerdict::Weak:
                ++levelIndex_;
                enterStep(Step::SideRest);  // the same port, one level up, after a rest
                return;
            case PulseVerdict::Decoupled: {
                // F-R2: a motor that turns but moves nothing cannot drive the robot, and guessing
                // its side would be inventing. Leave it out, say so, go on. Never commanded again
                // (it has no side; every command path zeroes a port with no side).
                excluded_[i] = true;
                report_.ports[i].excluded = true;
                report_.excludedPorts[report_.excludedCount++] = ports_[i];
                appendWarning(static_cast<int>(ports_[i]), Warning::Slipping);
                nextPort();
                return;
            }
            case PulseVerdict::Contradiction: {
                // The push said this member turns one way under a forward push; the pulse drove it
                // that way and it went the other. One of the two readings is wrong, and averaging
                // them away would bake a reversed motor into the description.
                char msg[kReasonCap];
                std::snprintf(msg, sizeof msg,
                              "port %d: driven forward at its captured sign %+d at %.1f V, its "
                              "OWN encoder went %.3f rad -- the push and the pulse contradict "
                              "each other",
                              static_cast<int>(ports_[i]), static_cast<int>(sign_[i]), level, own);
                refuse(Refusal::PulseContradiction, static_cast<int>(ports_[i]), msg);
                return;
            }
            case PulseVerdict::Rolled: {
                char msg[kReasonCap];
                std::snprintf(msg, sizeof msg,
                              "port %d moved its gear train (others %.1f deg) but the robot "
                              "turned only %.2f deg even at %.1f V (floor %.2f deg) -- it rolled "
                              "instead of turning, or it is not on the FLOOR",
                              static_cast<int>(ports_[i]), others / kDegreesToRadians,
                              yaw_ / kDegreesToRadians, level,
                              cfg_.pulseYawFloor.value() / kDegreesToRadians);
                refuse(Refusal::PulseRolled, static_cast<int>(ports_[i]), msg);
                return;
            }
            case PulseVerdict::Still:
            default: {
                char msg[kReasonCap];
                std::snprintf(msg, sizeof msg,
                              "port %d could not move the robot at %.1f V: it turned %.2f deg "
                              "(floor %.2f deg), its own shaft %.1f deg, no other drive port "
                              "moved -- is it on the FLOOR? is a wheel stuck?",
                              static_cast<int>(ports_[i]), level, yaw_ / kDegreesToRadians,
                              cfg_.pulseYawFloor.value() / kDegreesToRadians,
                              own / kDegreesToRadians);
                refuse(Refusal::PulseStill, static_cast<int>(ports_[i]), msg);
                return;
            }
        }
    }

    /// The current port is decided (a side, or left out): on to the next drive port, or finish.
    void nextPort() {
        ++pulseIndex_;
        levelIndex_ = 0;
        if (advanceToNextDrivePort()) {
            ++pulseOrdinal_;
            enterStep(Step::SideRest);
            return;
        }
        finishSideAssign();
    }

    /// Every drive port has been pulsed: there must be a member on each side; then each side's
    /// scale is pushDistance divided by the MEDIAN of its members' push travel (rule 4).
    void finishSideAssign() {
        leftN_ = 0;
        rightN_ = 0;
        for (std::size_t i = 0; i < n_; ++i) {
            if (!drive_[i] || excluded_[i]) {
                continue;
            }
            if (side_[i] < 0) {
                leftIdx_[leftN_++] = i;
            } else {
                rightIdx_[rightN_++] = i;
            }
        }
        if (leftN_ == 0 || rightN_ == 0) {
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "the pulses found %u LEFT port(s) and %u RIGHT (%u left out) -- a "
                          "two-sided drivetrain needs at least one on each side",
                          static_cast<unsigned>(leftN_), static_cast<unsigned>(rightN_),
                          static_cast<unsigned>(report_.excludedCount));
            refuse(Refusal::OneSided, 0, msg);
            return;
        }
        // The push-direction travel of a drive port is |its push delta| BY CONSTRUCTION (its raw
        // sign is the direction it moved, and the end that led is the front — CAL1), so it is
        // positive. The median is taken over the travel CAPTURED AT DONE: the pulses have moved
        // every shaft since.
        double median[2] = {0.0, 0.0};
        double spread[2] = {0.0, 0.0};
        const char* names[2] = {"LEFT", "RIGHT"};
        for (int s = 0; s < 2; ++s) {
            const std::size_t* idx = (s == 0) ? leftIdx_ : rightIdx_;
            const std::size_t count = (s == 0) ? leftN_ : rightN_;
            // No guard for a non-finite value is needed below, and none is written: a drive port
            // is one movedOnPush() accepted, whose push reading is FINITE by construction, so its
            // push travel (sign x that reading) is finite too — the spread, the median and the
            // minPushRad check only ever see numbers (DIAG1 Part B).
            double v[hal::kMaxSmartPorts];
            double lo = 0.0;
            double hi = 0.0;
            for (std::size_t k = 0; k < count; ++k) {
                v[k] = report_.ports[idx[k]].pushTravelRad;
                lo = (k == 0 || v[k] < lo) ? v[k] : lo;
                hi = (k == 0 || v[k] > hi) ? v[k] : hi;
            }
            spread[s] = hi - lo;
            median[s] = hal::medianInPlace(v, count);  // THE median — hal::MotorGroup's own
            if (median[s] < cfg_.minPushRad.value()) {
                char msg[kReasonCap];
                std::snprintf(msg, sizeof msg,
                              "%s side turned only %.2f rad over the push; at least %.2f (one "
                              "wheel turn) is needed to measure a scale -- push it the full %.0f "
                              "in",
                              names[s], median[s], cfg_.minPushRad.value(),
                              cfg_.pushDistance.value());
                refuse(Refusal::PushTooShort, 0, msg);
                return;
            }
        }
        report_.leftMedianRad = median[0];
        report_.rightMedianRad = median[1];
        report_.leftSpreadRad = spread[0];
        report_.rightSpreadRad = spread[1];
        leftIpr_ = cfg_.pushDistance.value() / median[0];
        rightIpr_ = cfg_.pushDistance.value() / median[1];
        report_.leftInchesPerRadian = leftIpr_;
        report_.rightInchesPerRadian = rightIpr_;
        enter(Phase::SpinTest, Step::SpinRestBefore);
    }

    /// Baseline the sides and the heading, then start spinning `direction` (+1 = CCW) at the first
    /// spin level.
    void beginSpin(units::Time now, double direction, Step which) {
        baselineAll();
        leftMonitor_.reset();
        rightMonitor_.reset();
        spinDirection_ = direction;
        spinLevelIndex_ = 0;
        spinStarted_ = false;
        levelStart_ = now;
        levelStartYaw_ = 0.0;
        stepStart_ = now;
        step_ = which;
    }

    /// One direction of the spin is over: W = (dR − dL) / dtheta, with each side's travel taken
    /// through its own median and its own scale. On a real floor the number this produces is the
    /// EFFECTIVE width, LARGER than a tape's, because a turning tank scrubs (A4 register HA-135);
    /// the A2 plant scrubs nothing, so the host recovers the true width exactly and says nothing
    /// about the size of that gap.
    void finishSpin(units::Time now) {
        safeState();  // 0 V + Coast: the robot coasts to a stop, pushable throughout
        const double dL = sideTravel(leftIdx_, leftN_, leftIpr_);
        const double dR = sideTravel(rightIdx_, rightN_, rightIpr_);
        const double w = (dR - dL) / yaw_;
        SpinRecord& rec = report_.spins[report_.spinCount < 2 ? report_.spinCount : 1];
        rec = SpinRecord{spinDirection_, currentSpinVolts(), yaw_, dL, dR, w,
                         now.value() - stepStart_.value()};
        if (report_.spinCount < 2) {
            ++report_.spinCount;
        }
        if (spinDirection_ > 0.0) {
            report_.widthCcwIn = w;
            enterStep(Step::SpinRestMid);
            return;
        }
        report_.widthCwIn = w;
        const double a = report_.widthCcwIn;
        const double b = report_.widthCwIn;
        if (!std::isfinite(a) || !std::isfinite(b) || a <= 0.0 || b <= 0.0) {
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "the spin measured a track width of %.3f in one way and %.3f the "
                          "other -- a width must be positive",
                          a, b);
            refuse(Refusal::SpinWidthInvalid, 0, msg);
            return;
        }
        const double mean = 0.5 * (a + b);
        report_.widthDisagreement = std::abs(a - b) / mean;
        if (report_.widthDisagreement > cfg_.spinAgreement) {
            // "pct", never the percent sign: PROS re-reads screen text as a format (header).
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "the two spin directions disagree about the track width by %.1f pct "
                          "(%.3f in vs %.3f in; bound %.1f pct)",
                          100.0 * report_.widthDisagreement, a, b, 100.0 * cfg_.spinAgreement);
            refuse(Refusal::SpinDisagree, 0, msg);
            return;
        }
        width_ = mean;
        enterStep(Step::SpinRestAfter);
    }

    /// Assemble the description, check it against its OWN validity rules, and finish.
    void finishDone() {
        safeState();
        result_ = hal::DrivetrainDescription{};
        result_.kind = facts_.kind;
        result_.cartridge = facts_.cartridge;
        result_.imuPort = facts_.imuPort;
        result_.source = hal::DescriptionSource::RobotMeasured;
        fillSide(result_.left, leftIdx_, leftN_, leftIpr_);
        fillSide(result_.right, rightIdx_, rightN_, rightIpr_);
        result_.trackWidth = units::Length{width_};
        std::snprintf(result_.provenance, sizeof result_.provenance,
                      "%.40s | ROBOT-MEASURED: %u ports, push %.0f in, W %.3f in",
                      facts_.provenance != nullptr ? facts_.provenance : "",
                      static_cast<unsigned>(leftN_ + rightN_), report_.pushDistanceIn, width_);
        char why[128];
        if (!result_.valid(why, sizeof why)) {
            // A last line of defence: whatever the procedure concluded, an invalid description
            // must never be handed out as a result.
            char msg[kReasonCap];
            std::snprintf(msg, sizeof msg,
                          "the measured description does not hold together: %s",
                          why);
            refuse(Refusal::InvalidDescription, 0, msg);
            return;
        }
        phase_ = Phase::Done;
        step_ = Step::None;
    }

    // ── helpers ────────────────────────────────────────────────────────────────────

    /// True when `levels[0..count)` is a usable ramp: 1..max entries, each in (0, 12] V, strictly
    /// increasing (a ramp that repeats or falls would pulse the same port twice for nothing).
    [[nodiscard]] static bool levelsValid(const units::Voltage* levels, std::size_t count,
                                          std::size_t max) noexcept {
        if (count < 1 || count > max) {
            return false;
        }
        for (std::size_t k = 0; k < count; ++k) {
            const double v = levels[k].value();
            if (!(v > 0.0 && v <= hal::kMaxMotorVoltage.value())) {
                return false;
            }
            if (k > 0 && !(v > levels[k - 1].value())) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] double currentPulseVolts() const noexcept {
        return cfg_.pulseLevels[levelIndex_].value();
    }

    [[nodiscard]] double currentSpinVolts() const noexcept {
        return cfg_.spinLevels[spinLevelIndex_].value();
    }

    /// True in the two phases that command a voltage (rule 7's scope). The rest windows inside
    /// them are included on purpose: a caller that has stopped ticking is a stalled caller whether
    /// or not this particular tick would have driven something, and everything is already at 0 V,
    /// so refusing there costs nothing.
    [[nodiscard]] bool poweredPhase() const noexcept {
        return phase_ == Phase::SideAssign || phase_ == Phase::SpinTest;
    }

    /// The first candidate turning faster than restFloor, or n_ when every motor is still — so a
    /// rest that never arrives can NAME the motor that is preventing it instead of leaving a
    /// person to guess which of twelve it is.
    [[nodiscard]] std::size_t firstMoving() const {
        for (std::size_t i = 0; i < n_; ++i) {
            if (std::abs(motors_[i]->velocity().value()) > cfg_.restFloor.value()) {
                return i;
            }
        }
        return n_;
    }

    /// Every motor under restFloor AND the robot's yaw rate under restYawRateFloor — the design's
    /// definition of rest ("yaw rate and every velocity under a floor"), which until 2F-1 the code
    /// implemented without its first half.
    [[nodiscard]] bool atRest() const {
        return firstMoving() == n_
               && std::abs(imu_.yawRate().value()) <= cfg_.restYawRateFloor.value();
    }

    /// The push so far, read LIVE from every candidate: the count that moved on the push
    /// (movedOnPush() — a FINITE travel past movedThreshold) and the median |travel| of those.
    /// pushReady() and finishPushCapture() both read it, so the DONE gate and the DONE finish are
    /// one predicate — and the finish's own classification uses movedOnPush() too, so
    /// drivePortCount, this count, pushReady(), status().movedNow and the captured signs all
    /// describe the same candidates. (The finish reads every shaft again for its classification:
    /// one PREDICATE, two reads; on a FakeMotor they agree, on a robot they can differ only by
    /// what moved between two reads in the same tick.)
    [[nodiscard]] PushSummary pushSummary() const {
        PushSummary s;
        double v[hal::kMaxSmartPorts];
        std::size_t k = 0;
        for (std::size_t i = 0; i < n_; ++i) {
            const double d = motors_[i]->position().value() - base_[i];
            if (movedOnPush(d)) {
                v[k++] = std::abs(d);
            }
        }
        s.moved = static_cast<std::uint8_t>(k);
        s.medianRad = k > 0 ? hal::medianInPlace(v, k) : 0.0;
        return s;
    }

    /// "Moved on the push": a push delta (radians, as read, before any sign) that is a FINITE
    /// number whose size reaches movedThreshold. The ONE test the push's classification
    /// (finishPushCapture) and its live count (pushSummary, and through it pushReady() and the
    /// panel's movedNow) share — neither keeps its own comparison (DIAG1 Part B, D5). A NaN delta
    /// already failed the old `std::abs(d) >= threshold`; a ±Inf one passed it and became a drive
    /// port with a sign read off PROS's failure sentinel — now it is no reading, like the NaN, and
    /// finishPushCapture names it. Finite by hal::isUsableReading(), the median's own test.
    [[nodiscard]] bool movedOnPush(double d) const noexcept {
        return hal::isUsableReading(d) && std::abs(d) >= cfg_.movedThreshold.value();
    }

    void safeState() {
        for (std::size_t i = 0; i < n_; ++i) {
            motors_[i]->setVoltage(units::Voltage{0.0});
            motors_[i]->setBrakeMode(hal::BrakeMode::Coast);  // rule 6: never Brake
        }
    }

    /// Drive candidate `i` at `volts` and every other candidate at 0 V; Coast everywhere.
    void commandOnly(std::size_t i, double volts) {
        for (std::size_t j = 0; j < n_; ++j) {
            motors_[j]->setBrakeMode(hal::BrakeMode::Coast);
            motors_[j]->setVoltage(units::Voltage{j == i ? volts : 0.0});
        }
    }

    /// The spin command at the current level: the LEFT side backward and the RIGHT side forward
    /// turns the robot CCW (positive yaw), and the mirror image turns it CW. Each member gets its
    /// own captured sign; every candidate on no side — not a drive motor, or LEFT OUT — gets 0 V.
    void commandSpin() {
        const double level = currentSpinVolts();
        for (std::size_t j = 0; j < n_; ++j) {
            motors_[j]->setBrakeMode(hal::BrakeMode::Coast);
            motors_[j]->setVoltage(units::Voltage{0.0});
        }
        for (std::size_t k = 0; k < leftN_; ++k) {
            const std::size_t i = leftIdx_[k];
            motors_[i]->setVoltage(
                units::Voltage{static_cast<double>(sign_[i]) * -spinDirection_ * level});
        }
        for (std::size_t k = 0; k < rightN_; ++k) {
            const std::size_t i = rightIdx_[k];
            motors_[i]->setVoltage(
                units::Voltage{static_cast<double>(sign_[i]) * spinDirection_ * level});
        }
    }

    /// Run the SHARED coupled-side monitor over both sides for this tick; a PERSISTED disagreement
    /// is a fight (a wrong sign, a stripped gear, a frozen encoder) and refuses the run naming the
    /// port. Returns true when it refused.
    [[nodiscard]] bool fightDuringSpin() {
        const double leftVolts = -spinDirection_ * currentSpinVolts();
        const double rightVolts = spinDirection_ * currentSpinVolts();
        for (int s = 0; s < 2; ++s) {
            const std::size_t* idx = (s == 0) ? leftIdx_ : rightIdx_;
            const std::size_t count = (s == 0) ? leftN_ : rightN_;
            teleop::MemberSample samples[hal::kMaxSmartPorts];
            for (std::size_t k = 0; k < count; ++k) {
                const std::size_t i = idx[k];
                // The sign is applied HERE, once (rule 1): the adapters carry positive ports.
                samples[k] = teleop::MemberSample{
                    static_cast<double>(sign_[i]) * motors_[i]->velocity().value(), true};
            }
            teleop::CoupledSideMonitor& monitor = (s == 0) ? leftMonitor_ : rightMonitor_;
            const teleop::SideVerdict v = monitor.update(
                s == 0 ? leftVolts : rightVolts,
                std::span<const teleop::MemberSample>{samples, count});
            if (v.persistedCount > 0) {
                for (std::size_t k = 0; k < count; ++k) {
                    if ((v.persistedMask & (std::uint32_t{1} << k)) != 0u) {
                        char msg[kReasonCap];
                        std::snprintf(msg, sizeof msg,
                                      "port %d fought its %s side-mates for the whole "
                                      "persistence window during the spin -- stop and check "
                                      "that motor",
                                      static_cast<int>(ports_[idx[k]]),
                                      s == 0 ? "LEFT" : "RIGHT");
                        refuse(Refusal::SpinFight, static_cast<int>(ports_[idx[k]]), msg);
                        return true;
                    }
                }
            }
        }
        return false;
    }

    /// One side's travel since the last baseline, inches: the MEDIAN of its members' sign-corrected
    /// shaft deltas times that side's scale.
    [[nodiscard]] double sideTravel(const std::size_t* idx, std::size_t count, double ipr) const {
        double v[hal::kMaxSmartPorts];
        for (std::size_t k = 0; k < count; ++k) {
            const std::size_t i = idx[k];
            v[k] = static_cast<double>(sign_[i]) * (motors_[i]->position().value() - base_[i]);
        }
        return hal::medianInPlace(v, count) * ipr;
    }

    void fillSide(hal::DriveSideDescription& side, const std::size_t* idx, std::size_t count,
                  double ipr) const {
        side.count = static_cast<std::uint8_t>(count);
        for (std::size_t k = 0; k < count; ++k) {
            const std::size_t i = idx[k];
            // The SIGN goes INSIDE the port number — the one place it lives from here on.
            side.signedPorts[k] = static_cast<std::int8_t>(sign_[i] * ports_[i]);
        }
        side.inchesPerRadian = units::Length{ipr};
    }

    [[nodiscard]] bool advanceToNextDrivePort() {
        while (pulseIndex_ < n_ && !drive_[pulseIndex_]) {
            ++pulseIndex_;
        }
        return pulseIndex_ < n_;
    }

    /// Append one pulse to the report's record and make it the port's deciding observation.
    void recordPulse(std::size_t i, double level, double own, double others, PulseVerdict v) {
        if (report_.pulseCount < kMaxPulseRecords) {
            report_.pulses[report_.pulseCount++] =
                PulseRecord{ports_[i], pulseOrdinal_, level, yaw_, own, others, v};
        }
        PortObservation& o = report_.ports[i];
        o.pulseYawRad = yaw_;
        o.pulseShaftRad = own;
        o.pulseLevelVolts = level;
        o.pulseOthersRad = others;
        o.verdict = v;
    }

    /// Which sentence appendWarning() adds.
    enum class Warning : std::uint8_t {
        Slipping,   ///< LEFT OUT as decoupled at its top pulse level
        NoReading,  ///< its push reading at DONE was not finite: not counted as a drive port
    };

    /// One warning sentence for one port, appended (bounded; a warning that does not fit is cut —
    /// the report still names every port: excludedPorts for a slipping one, the push record's
    /// NO READING for one with no reading).
    void appendWarning(int port, Warning which) {
        const std::size_t used = std::strlen(warning_);
        if (used + 1 >= sizeof warning_) {
            return;
        }
        if (which == Warning::NoReading) {
            std::snprintf(warning_ + used, sizeof warning_ - used,
                          "%sport %d gave no reading at DONE -- not counted as a drive port; if "
                          "it drives the robot, check it and run again",
                          used > 0 ? " | " : "", port);
            return;
        }
        std::snprintf(warning_ + used, sizeof warning_ - used,
                      "%sport %d turns but does not move the robot -- its gear is slipping; it "
                      "is left out",
                      used > 0 ? " | " : "", port);
    }

    void baselineAll() {
        for (std::size_t i = 0; i < n_; ++i) {
            base_[i] = motors_[i]->position().value();
        }
        resetYaw();
    }

    void resetYaw() {
        yaw_ = 0.0;
        prevHeading_ = imu_.heading();
    }

    /// Rule 3: accumulate the shortest signed step every tick, so two full turns are two full
    /// turns and not a wrapped nothing.
    void accumulateYaw() {
        const math::Angle h = imu_.heading();
        yaw_ += prevHeading_.errorTo(h);
        prevHeading_ = h;
    }

    void enter(Phase p, Step s) {
        phase_ = p;
        phaseStart_ = clock_.now();
        enterStep(s);
        if (p != Phase::SideAssign && p != Phase::SpinTest) {
            safeState();  // an unpowered phase leaves the robot pushable
        }
    }

    void enterStep(Step s) {
        step_ = s;
        stepStart_ = clock_.now();
        resting_ = false;
        if (s == Step::SideRest || s == Step::SpinRestBefore || s == Step::SpinRestMid
            || s == Step::SpinRestAfter) {
            safeState();
            resetYaw();  // the live yaw a panel shows now starts at this rest window (2F-1)
        }
    }

    /// Refuse the run: record the cause and its code, stop every motor, end in Aborted. Every
    /// refusal in this class goes through here, which is what makes "0 V + Coast on every refusal"
    /// a property of one line rather than of a dozen call sites.
    /// `text` is already formatted. Every call site builds its message with a LITERAL format
    /// string into a local buffer and hands the result here, rather than passing a format through
    /// — the on-robot build gate compiles src/ under -Wformat-nonliteral, and a funnel taking
    /// `const char* format` would switch OFF the compiler's argument checking for every refusal
    /// message in the class, which is exactly where a wrong specifier costs a person the one
    /// sentence they get when the robot stops.
    void refuse(Refusal code, int port, const char* text) {
        std::snprintf(reason_, sizeof reason_, "%s", text);
        refusal_ = code;
        refusalPort_ = port;
        safeState();
        phase_ = Phase::Aborted;
        step_ = Step::None;
        updateStatus();
    }

    [[nodiscard]] static const char* answerWord(Answer a) noexcept {
        switch (a) {
            case Answer::Done: return "DONE";
            case Answer::Abort: return "ABORT";
        }
        return "?";
    }

    [[nodiscard]] const char* waitingWord() const noexcept {
        switch (step_) {
            case Step::PushWatching: return "watching the push (waiting for DONE)";
            default: return "driving the motors (no answer is expected)";
        }
    }

    void updateStatus() {
        status_.phase = phase_;
        status_.phaseSeconds = phase_ == Phase::Idle ? 0.0
                                                     : clock_.now().value() - phaseStart_.value();
        status_.atRest = phase_ != Phase::Idle && atRest();
        status_.liveYawRad = yaw_;
        status_.pulsing = phase_ == Phase::SideAssign && step_ == Step::SidePulse;
        const bool spinning = step_ == Step::SpinCcw || step_ == Step::SpinCw;
        status_.spinDirection = spinning ? spinDirection_ : 0.0;
        status_.spinTurns = spinning ? spinDirection_ * yaw_ / (2.0 * math::Angle::kPi) : 0.0;
        status_.spinLevelVolts = spinning ? currentSpinVolts() : 0.0;
        status_.excludedCount = report_.excludedCount;
        std::uint8_t left = 0;
        std::uint8_t right = 0;
        for (std::size_t i = 0; i < n_; ++i) {
            left = static_cast<std::uint8_t>(left + (report_.ports[i].side < 0 ? 1 : 0));
            right = static_cast<std::uint8_t>(right + (report_.ports[i].side > 0 ? 1 : 0));
        }
        status_.leftCount = left;
        status_.rightCount = right;
        if (step_ == Step::PushWatching) {
            // The yaw RATE is read on every tick of the push, though the push itself never uses it
            // (CAL1 Part B, 2026-11-05): the PROS IMU adapter's default rate is the derivative of
            // the rotation between FRESH reads (hal/pros/imu.hpp, DifferentiateRotation), and
            // atRest() skips its rate read while a motor turns — as every one does all push long —
            // so DONE's first rest tick would read a rate averaged over the WHOLE push, and a robot
            // turned by hand at 3 deg/s then read as still for that tick (IMU1's T11). Until Part B
            // the WHICH END tick between DONE and the rest window took that stale read.
            (void)imu_.yawRate();
            const PushSummary s = pushSummary();
            status_.movedNow = s.moved;
            status_.pushTravelRad = s.medianRad;
            status_.pushFarEnough = s.medianRad >= cfg_.minPushRad.value();
            status_.driftRad = yaw_;
        } else if (phase_ != Phase::Idle && phase_ != Phase::WaitImu) {
            status_.driftRad = report_.pushStraightnessRad;
        }
        const bool pulsePhase = phase_ == Phase::SideAssign;
        status_.pulseTotal = report_.drivePortCount;
        status_.pulseOrdinal = pulsePhase ? pulseOrdinal_ : status_.pulseOrdinal;
        status_.pulsePort = pulsePhase && pulseIndex_ < n_ ? ports_[pulseIndex_] : std::int8_t{0};
        status_.pulseLevelVolts = pulsePhase ? currentPulseVolts() : 0.0;

        switch (phase_) {
            case Phase::Idle:
                status_.prompt = "not started";
                status_.progress[0] = '\0';
                return;
            case Phase::WaitImu:
                status_.prompt = "WAIT -- the IMU is calibrating. Hands off.";
                std::snprintf(status_.progress, sizeof status_.progress, "%.1f s of %.1f s",
                              clock_.now().value() - phaseStart_.value(),
                              cfg_.imuReadyTimeout.value());
                return;
            case Phase::PushCapture: {
                std::snprintf(prompt_, sizeof prompt_, "PUSH IT %.0f IN, FRONT FIRST, then touch DONE.",
                              cfg_.pushDistance.value());
                status_.prompt = prompt_;
                std::snprintf(status_.progress, sizeof status_.progress,
                              "%u of %u moved, drift %+.1f deg, travel %.1f rad%s",
                              static_cast<unsigned>(status_.movedNow),
                              static_cast<unsigned>(n_), yaw_ / kDegreesToRadians,
                              status_.pushTravelRad, pushReady() ? " -- READY" : "");
                return;
            }
            case Phase::SideAssign:
                status_.prompt = "STAND CLEAR -- pulsing one motor at a time.";
                std::snprintf(status_.progress, sizeof status_.progress,
                              "port %d (%u of %u) %s at %.1f V, yaw %+.1f deg",
                              static_cast<int>(status_.pulsePort),
                              static_cast<unsigned>(status_.pulseOrdinal),
                              static_cast<unsigned>(status_.pulseTotal),
                              status_.pulsing ? "pulsing" : "settling", status_.pulseLevelVolts,
                              yaw_ / kDegreesToRadians);
                return;
            case Phase::SpinTest:
                status_.prompt = "STAND CLEAR -- spinning in place, both ways.";
                std::snprintf(status_.progress, sizeof status_.progress,
                              "%s %.2f of %.2f turns at %.1f V",
                              spinDirection_ > 0.0 ? "CCW" : "CW",
                              spinning ? status_.spinTurns : 0.0, cfg_.spinTurns,
                              spinning ? currentSpinVolts() : 0.0);
                return;
            case Phase::Done:
                status_.prompt = "DONE -- the drivetrain described itself.";
                result_.describe(status_.progress, sizeof status_.progress);
                return;
            case Phase::Aborted:
                status_.prompt = "STOPPED -- every motor is at 0 V, coasting.";
                std::snprintf(status_.progress, sizeof status_.progress, "%.95s", reason_);
                return;
        }
    }

    // ── state ──────────────────────────────────────────────────────────────────────
    std::span<hal::IMotor* const> motors_;
    std::span<const std::int8_t> ports_;
    hal::IImu& imu_;
    hal::IClock& clock_;
    CalibratorConfig cfg_;
    HumanFacts facts_{};

    std::size_t n_ = 0;
    Phase phase_ = Phase::Idle;
    Step step_ = Step::None;
    CalibratorStatus status_{};
    char prompt_[48] = "";
    /// How long a refusal may be. Wide enough that no reason is TRUNCATED — checked by the
    /// compiler, which under -Wformat-truncation knows every literal format in this class and
    /// every buffer they are written into. A refusal that lost its last clause would be the one
    /// sentence a person gets, cut off.
    static constexpr std::size_t kReasonCap = 256;
    char reason_[kReasonCap] = "";
    Refusal refusal_ = Refusal::None;
    int refusalPort_ = 0;
    char warning_[kReasonCap] = "";
    hal::DrivetrainDescription result_{};
    CalibrationReport report_{};

    double base_[hal::kMaxSmartPorts]{};
    std::int8_t sign_[hal::kMaxSmartPorts]{};
    std::int8_t side_[hal::kMaxSmartPorts]{};
    bool drive_[hal::kMaxSmartPorts]{};
    bool excluded_[hal::kMaxSmartPorts]{};
    std::size_t leftIdx_[hal::kMaxSmartPorts]{};
    std::size_t rightIdx_[hal::kMaxSmartPorts]{};
    std::size_t leftN_ = 0;
    std::size_t rightN_ = 0;

    units::Time phaseStart_{};
    units::Time stepStart_{};
    units::Time restSince_{};
    units::Time lastTick_{};
    units::Time levelStart_{};
    bool resting_ = false;
    bool ticked_ = false;

    math::Angle prevHeading_{};
    double yaw_ = 0.0;

    std::size_t pulseIndex_ = 0;
    std::size_t levelIndex_ = 0;
    std::uint8_t pulseOrdinal_ = 0;
    double leftIpr_ = 0.0;
    double rightIpr_ = 0.0;
    double width_ = 0.0;
    double spinDirection_ = 1.0;
    std::size_t spinLevelIndex_ = 0;
    double levelStartYaw_ = 0.0;
    bool spinStarted_ = false;
    teleop::CoupledSideMonitor leftMonitor_{cfg_.monitor};
    teleop::CoupledSideMonitor rightMonitor_{cfg_.monitor};
};

/// A short, stable, ASCII name for a refusal code — for the serial record ("[PULSE_ROLLED port
/// 11]") and for a test's failure message. Never shown as the operator's explanation: the panel's
/// words are calibrate_panel.hpp's.
[[nodiscard]] constexpr const char* refusalName(Refusal r) noexcept {
    switch (r) {
        case Refusal::None: return "NONE";
        case Refusal::Aborted: return "ABORTED";
        case Refusal::ImuNotReady: return "IMU_NOT_READY";
        case Refusal::PushTooFew: return "PUSH_TOO_FEW";
        case Refusal::PushCurved: return "PUSH_CURVED";
        case Refusal::PushTooShort: return "PUSH_TOO_SHORT";
        case Refusal::RestTimeout: return "REST_TIMEOUT";
        case Refusal::NonDriveMoved: return "NON_DRIVE_MOVED";
        case Refusal::PulseContradiction: return "PULSE_CONTRADICTION";
        case Refusal::PulseRolled: return "PULSE_ROLLED";
        case Refusal::PulseStill: return "PULSE_STILL";
        case Refusal::OneSided: return "ONE_SIDED";
        case Refusal::SpinWrongWay: return "SPIN_WRONG_WAY";
        case Refusal::SpinNoStart: return "SPIN_NO_START";
        case Refusal::SpinTimeout: return "SPIN_TIMEOUT";
        case Refusal::SpinFight: return "SPIN_FIGHT";
        case Refusal::SpinWidthInvalid: return "SPIN_WIDTH_INVALID";
        case Refusal::SpinDisagree: return "SPIN_DISAGREE";
        case Refusal::MissedTick: return "MISSED_TICK";
        case Refusal::OutOfOrder: return "OUT_OF_ORDER";
        case Refusal::InvalidDescription: return "INVALID_DESCRIPTION";
    }
    return "?";
}

/// A short, stable, ASCII word for a pulse verdict — the last column of a pulse's serial line.
[[nodiscard]] constexpr const char* pulseVerdictWord(PulseVerdict v) noexcept {
    switch (v) {
        case PulseVerdict::None: return "not pulsed";
        case PulseVerdict::Weak: return "under the floor -- next level";
        case PulseVerdict::Left: return "LEFT";
        case PulseVerdict::Right: return "RIGHT";
        case PulseVerdict::Contradiction: return "CONTRADICTION";
        case PulseVerdict::Decoupled: return "LEFT OUT (decoupled)";
        case PulseVerdict::Rolled: return "ROLLED";
        case PulseVerdict::Still: return "STILL";
    }
    return "?";
}

/// The three record formatters below add 0.0 to every signed number they print: a travel of
/// exactly nothing times a captured sign of −1 is IEEE negative zero, and "own -0.0 deg" in a log
/// reads as a motor that moved backward. (x + 0.0 is +0.0 for x = −0.0 and x for everything else.)
///
/// One line of the PUSH TABLE for the serial record (F1.5): the candidate's port, its travel as
/// read, whether it moved and the sign captured —
/// `push port 11 travel -373.2 deg MOVED sign -1`. A candidate whose push reading was not finite
/// keeps its port and its reading, and its word is `NO READING` instead of `still` (DIAG1 Part B:
/// it is not a drive port, and unlike a still one it is a motor the run could not read — the
/// number is spelled the C library's way, e.g. `+inf`). Written into `out` (NUL-terminated, cut to
/// `cap`); returns what std::snprintf returns. ASCII, no '%'.
inline int formatPushRecord(const PortObservation& o, char* out, std::size_t cap) {
    const char* word = o.sign != 0                           ? "MOVED"
                       : hal::isUsableReading(o.pushDeltaRad) ? "still"
                                                              : "NO READING";
    return std::snprintf(out, cap, "push port %d travel %+.1f deg %s sign %+d",
                         static_cast<int>(o.port), o.pushDeltaRad / kDegreesToRadians + 0.0,
                         word, static_cast<int>(o.sign));
}

/// One line per PULSE for the serial record (F1.5) — the numbers that would have explained robot
/// two's start B: `pulse port 12 (3 of 9) 6.0 V yaw -2.31 deg own +41.2 deg others 38.9 deg:
/// LEFT`. `total` is the run's drive-port count (the report's drivePortCount). ASCII, no '%'.
inline int formatPulseRecord(const PulseRecord& r, unsigned total, char* out, std::size_t cap) {
    return std::snprintf(out, cap,
                         "pulse port %d (%u of %u) %.1f V yaw %+.2f deg own %+.1f deg others "
                         "%.1f deg: %s",
                         static_cast<int>(r.port), static_cast<unsigned>(r.ordinal), total,
                         r.levelVolts, r.yawRad / kDegreesToRadians + 0.0,
                         r.ownRad / kDegreesToRadians + 0.0, r.othersMaxRad / kDegreesToRadians,
                         pulseVerdictWord(r.verdict));
}

/// One line per SPIN DIRECTION for the serial record (F1.5): `spin CCW 3.0 V yaw +720.0 deg
/// L -78.54 in R +78.54 in W 12.500 in 7.41 s`. ASCII, no '%'.
inline int formatSpinRecord(const SpinRecord& r, char* out, std::size_t cap) {
    return std::snprintf(out, cap,
                         "spin %s %.1f V yaw %+.1f deg L %+.2f in R %+.2f in W %.3f in %.2f s",
                         r.direction > 0.0 ? "CCW" : "CW", r.levelVolts,
                         r.yawRad / kDegreesToRadians + 0.0, r.leftTravelIn + 0.0,
                         r.rightTravelIn + 0.0, r.widthIn, r.seconds);
}

}  // namespace shulib::calibration
