// L9 PART A1 — backing into a point in a recipe: Routine::reverseTo(x, y, options) and
// Routine::faceAway(x, y, options), the two steps appended to the FROZEN recipe layer (register
// row F10) at API 2.4. Every case names the bug it would catch.
//
// What the steps are: driveTo and face turned around. Each computes ONE argument — the bearing
// from the live pose estimate to a field point, plus π — and delegates to exactly one Chassis
// verb (moveTo / turnTo). So the keystone here is the same as D1's sugar case: an ORACLE written
// fresh by hand (turnTo(atan2(...) + π), then moveTo at the re-read bearing + π) must drive the
// host test plant bit-identically. The route twin (a DriveTo waypoint with `reverse`) is pinned in
// path_runner_test.cpp beside G2's runner 10, where its recorder and bit-identity helpers live.
//
// ── The geometry rule (the brief's; every motion case below) ──────────────────────────────
// Never from the origin and never at bearing 0: two mutants hide there. From the origin, a
// bearing taken from the ORIGIN instead of the live pose IS the live bearing; and at bearing 0
// (and at about 59 % of a uniform sweep) `b + π` and `b − π` are bit-identical after Angle's
// wrap. So every leg starts at (−24, −12) facing 90° and backs into (0, 24): the acceptance
// route's first leg, bearing 0.98279372324732905 rad, where `b + π` = −2.1587989303424644 and
// `b − π` = −2.158798930342464 (they differ).
//
// ── TANK1 (API 2.3) and a tank's reversed leg ─────────────────────────────────────────────
// A tank moveTo steers back onto its line only for a leg that starts within 15° of its target
// heading, decided once at the leg's first live tick; outside that window it runs unsteered and
// logs one Warn. A reverseTo leg starts as far off its target heading as the robot's BACK is
// turned from the point. faceAway() puts it inside the window (measured: 0.75° off), so the
// reversed leg IS steered — which on a clean plant is invisible (the bend is ~0), so A1b proves
// it with a weak side and TANK1's off switch. Without faceAway(): 180° off from a robot FACING
// the point (A4), about 0 after driving straight out from it (A4b), and in between the Warn names
// the offset (A4c). A holonomic drive rotates in flight instead (A5, and A2b with faceAway()).
//
// Every number a check bounds was MEASURED on the built code first (L9-PROGRESS.md, Part A1,
// Step 2, and the review round's probe) and is quoted beside the check; a bound is the
// measurement plus a margin. Graded on GROUND TRUTH (the plant's per-tick TruthSample), never on
// the estimate the steps read. NOTHING here has run on a robot: HA-167 registers the belief that
// a tank backs up as far as its calibration says — its scale comes from one unpowered push —
// which no host test can see.

#include "doctest.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/sim/scenario.hpp"

using namespace motion_rig;
using shulib::PreconditionError;
using shulib::chassis::MotionOptions;
using shulib::chassis::Routine;
using shulib::chassis::RoutineResult;
using shulib::chassis::RoutineStopCause;
using shulib::control::ExitReason;
using shulib::hal::BrakeMode;
using shulib::hal::LogLevel;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::kinematics::TankKinematics;
using shulib::kinematics::xDrive;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::sim::SimHarness;
using shulib::sim::TruthSample;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;

namespace {

namespace sim = shulib::sim;
namespace units = shulib::units;

// ── the geometry rule ──────────────────────────────────────────────────────────────────────
constexpr double kStartX = -24.0;
constexpr double kStartY = -12.0;
constexpr double kPointX = 0.0;
constexpr double kPointY = 24.0;

Pose2d startPose() { return Pose2d{Length{kStartX}, Length{kStartY}, Angle::degrees(90.0)}; }

/// The bearing from the start to the point, and the heading a reversed arrival ends on — computed
/// here the way an author would, independently of the library's own expression.
double startBearing() { return std::atan2(kPointY - kStartY, kPointX - kStartX); }
Angle awayHeading() { return Angle::radians(startBearing() + Angle::kPi); }

Pose2d point() { return Pose2d{Length{kPointX}, Length{kPointY}, Angle{}}; }

/// path_runner_test.cpp's RecordingPacer, LOCAL to this file (the brief's choice: moving it into
/// motion_test_rig.hpp would put its 36 includers in every mutation's stale-binary set). It
/// samples AFTER the tick and BEFORE the plant advances — a sample is exactly the state the tick
/// was rendered on — and is hard-capped, so a wait that never ends turns a case RED, not a hang.
struct ReverseRecorder final : shulib::motion::ITickPacer {
    explicit ReverseRecorder(SimHarness& harness, int cap = 200000) : h{&harness}, capPaces{cap} {}

    void pace() override {
        if (static_cast<int>(truth.size()) >= capPaces) {
            throw std::runtime_error("ReverseRecorder: pace cap exceeded — a wait failed to end");
        }
        truth.push_back(h->sample());
        for (int i = 0; i < h->motorCount(); ++i) {
            volts.push_back(h->motor(i).commandedVoltage().value());
        }
        h->plant().step(Time{0.01});
    }

    SimHarness* h;
    int capPaces;
    std::vector<TruthSample> truth;  ///< one sample per tick, memcmp-comparable (packed doubles)
    std::vector<double> volts;       ///< every plant motor's commanded volts, per tick
};

/// ONE side's volts scaled — the drift robot two showed (its sides differ by ~9 %). `wheel` 0 is
/// the tank's LEFT side, 1 its RIGHT (the kinematic wheel order).
struct WeakSide final : sim::DegradationModel {
    WeakSide(int w, double s) : wheel{w}, scale{s} {}
    units::Voltage effectiveVoltage(int w, units::Voltage commanded, Time /*now*/,
                                    sim::Rng& /*rng*/) override {
        return w == wheel ? units::Voltage{commanded.value() * scale} : commanded;
    }
    int wheel;
    double scale;
};

/// MotionRig + the recorder + a Chassis, with the harness's log captured (the MOT Warn from the
/// motion layer and the RTN stop/skip lines from the chain both land in `log`).
struct ReverseRig {
    FakeTelemetrySink log;
    MotionRig rig;
    ReverseRecorder pacer;
    shulib::chassis::Chassis chassis;

    explicit ReverseRig(const shulib::kinematics::IKinematics& kin, const Pose2d& start = startPose(),
                        sim::DegradationModel* degradation = nullptr,
                        const shulib::chassis::ChassisConfig& cc = chassisConfig())
        : rig{kin, plantAt(start), &log, degradation}, pacer{rig.h}, chassis{rig.deps, pacer, cc} {}

    static sim::SimHarnessConfig plantAt(const Pose2d& pose) {
        sim::SimHarnessConfig cfg = plantConfig();
        cfg.plant.initialPose = pose;  // truth starts where startAt() will say it does
        return cfg;
    }

    [[nodiscard]] double now() { return rig.h.clock().now().value(); }

    /// Log entries at `level` whose message contains `needle`.
    [[nodiscard]] int count(LogLevel level, std::string_view needle) const {
        int n = 0;
        for (int i = 0; i < log.size(); ++i) {
            if (log.at(i).level == level && log.at(i).message.find(needle) != std::string::npos) {
                ++n;
            }
        }
        return n;
    }
};

/// TANK1's un-faced Warn, as the motion layer words it (move_to_pose.hpp, warnNotSteered).
constexpr const char* kNotSteered = "tank moveTo not steered";

/// The chassis config with TANK1 switched off — its documented off switch.
shulib::chassis::ChassisConfig tankSteerOff() {
    shulib::chassis::ChassisConfig cc = chassisConfig();
    cc.motion.tankSteer.maxSteer = 0.0;
    return cc;
}

/// Every plant motor at exactly 0 V under Brake — the chain's defined safe state after a stop.
void checkSafeState(ReverseRig& r) {
    for (int w = 0; w < r.rig.h.motorCount(); ++w) {
        CHECK(r.rig.h.motor(w).commandedVoltage().value() == 0.0);
        CHECK(r.rig.h.motor(w).brakeMode() == BrakeMode::Brake);
    }
}

bool sameTrace(const ReverseRecorder& a, const ReverseRecorder& b) {
    return a.truth.size() == b.truth.size() && !a.truth.empty()
           && std::memcmp(a.truth.data(), b.truth.data(), a.truth.size() * sizeof(TruthSample)) == 0
           && a.volts == b.volts;  // exact doubles, element by element
}

/// Identical, non-default options for BOTH arms of the oracle (a speed cap on each step), so a
/// step that dropped any field of its options would drive differently from its oracle (A12's
/// twin, inside the keystone).
constexpr MotionOptions kTurnOptions{.timeout = Time{4.0}, .maxAngularSpeed = AngularVelocity{4.0}};
constexpr MotionOptions kReverseOptions{.timeout = Time{8.0}, .maxLinearSpeed = Velocity{40.0}};

}  // namespace

// ═══ A8 — compile-time misuse pins (the negative tests ARE the assertions) ══════════════════════

// Bug caught: a new step accepting dimensionless doubles — `reverseTo(24, 36)` from someone
// thinking in inches OR in tiles, the silent-unit-misuse door §17 shut at every other step. A
// `reverseTo(double, double)` overload would compile every one of those calls (mutation 11).
template <typename... Args>
concept RoutineReverseToCallable = requires(Routine& r, Args... a) { r.reverseTo(a...); };
template <typename... Args>
concept RoutineFaceAwayCallable = requires(Routine& r, Args... a) { r.faceAway(a...); };

static_assert(!RoutineReverseToCallable<double, double>);
static_assert(!RoutineReverseToCallable<int, int>);
static_assert(RoutineReverseToCallable<Length, Length>);
static_assert(RoutineReverseToCallable<Length, Length, MotionOptions>);
static_assert(!RoutineFaceAwayCallable<double, double>);
static_assert(!RoutineFaceAwayCallable<int, int>);
static_assert(RoutineFaceAwayCallable<Length, Length>);
static_assert(RoutineFaceAwayCallable<Length, Length, MotionOptions>);
// An Angle is not a point: faceAway(90_deg) must not quietly mean "turn to 90° + π".
static_assert(!RoutineFaceAwayCallable<Angle>);

// ═══ A1 — the oracle, on a tank ══════════════════════════════════════════════════════════════

// Bug caught: faceAway()/reverseTo() drifting from the hand idiom they abbreviate — the + π dropped
// or taken the other way, the bearing read from the origin instead of the live estimate, a swapped
// atan2 argument, hidden motion logic, faceAway done as an in-place moveTo, an option field dropped.
// The hand arm is written FRESH here (the guide's tank idiom turned around) — an independent
// oracle — and both arms must agree to the bit AND land on the point, backwards, on truth.
TEST_CASE("L9 A1: faceAway + reverseTo == hand-written turnTo(atan2 + pi) + moveTo, bit for bit "
          "(tank)") {
    const TankKinematics kin{Length{12.0}};
    const Length tx{kPointX};
    const Length ty{kPointY};

    ReverseRig recipe{kin};
    Routine r{recipe.chassis, "backs-in"};
    r.startAt(startPose()).faceAway(tx, ty, kTurnOptions);
    REQUIRE(r.ok());
    // TANK1: the reversed leg must START inside the 15° engage window, or it is not steered.
    // Measured 0.0131 rad (0.75°) — faceAway's own settle tolerance, far inside 0.26 rad.
    const double startOff = std::abs(recipe.chassis.pose().heading().errorTo(awayHeading()));
    CHECK(startOff < 0.05);
    r.reverseTo(tx, ty, kReverseOptions);
    REQUIRE(r.ok());
    CHECK(r.result().steps == 3);
    CHECK(r.result().completed == 3);

    ReverseRig hand{kin};
    // The idiom by hand: seed, bearing from the ESTIMATE plus π, turn; re-read, move with the
    // re-read bearing plus π as the target heading. What makes this arm independent is that it
    // reads the pose and does the trigonometry itself — NOT how it spells the + π: its
    // Angle::radians(atan2 + kPi) is the very same floating-point work as the library's
    // Angle + Angle::radians(kPi) (each wraps the same sum once), bit for bit at every bearing.
    hand.chassis.setPose(startPose());
    const Pose2d here = hand.chassis.pose();
    const Angle turn =
        Angle::radians(std::atan2((ty - here.y()).value(), (tx - here.x()).value()) + Angle::kPi);
    REQUIRE(hand.chassis.turnTo(turn, kTurnOptions) == ExitReason::Settled);
    const Pose2d mid = hand.chassis.pose();
    const Angle arrive =
        Angle::radians(std::atan2((ty - mid.y()).value(), (tx - mid.x()).value()) + Angle::kPi);
    REQUIRE(hand.chassis.moveTo(Pose2d{tx, ty, arrive}, kReverseOptions) == ExitReason::Settled);

    CHECK(sameTrace(recipe.pacer, hand.pacer));  // per-tick truth by memcmp + every volt, exact
    CHECK(recipe.now() == hand.now());
    CHECK(recipe.pacer.truth.size() > 200);  // it really drove (measured 354 ticks)

    // The ground-truth anchors (catch BOTH arms sharing one sign error). Measured: 0.231 in from
    // the point, heading 0.00002 rad from b + π, truth (−0.128, 23.808).
    const Pose2d truth = recipe.rig.h.truePose();
    CHECK(posErr(truth, point()) < 1.0);
    CHECK(std::abs(truth.heading().errorTo(awayHeading())) < 0.10);
    CHECK(truth.x().value() > -3.0);  // it went +x from −24…
    CHECK(truth.y().value() > 20.0);  // …and +y from −12, the way the field says
    // TANK1: steered, so never the un-faced Warn.
    CHECK(recipe.count(LogLevel::Warn, kNotSteered) == 0);
    CHECK_FALSE(recipe.rig.latch.hasFault());
}

// ═══ A1b — the reversed leg IS steered (TANK1) ═══════════════════════════════════════════════

// Bug caught: a reversed tank leg that TANK1 does not steer — a faceAway that leaves the leg
// outside the 15° window (a wrong + π, a turn to the bearing instead of away from it), or a
// reverseTo that reaches the tank by some path the steering does not take. On a clean plant a
// steered and an unsteered leg look alike (the bend is ~0), so the drift robot two showed is put
// back: one side 9 % weak. Steered, the leg settles; with TANK1's off switch it times out
// beside its line — the difference is the whole claim.
TEST_CASE("L9 A1b: after faceAway, a tank's reverseTo leg is steered — a 9 % weak side still "
          "arrives, and does not with TANK1 off") {
    const TankKinematics kin{Length{12.0}};
    for (const int wheel : {0, 1}) {
        CAPTURE(wheel);  // 0 = left side weak, 1 = right side weak
        WeakSide weak{wheel, 0.91};

        ReverseRig on{kin, startPose(), &weak};
        Routine r{on.chassis, "weak-steered"};
        r.startAt(startPose())
            .faceAway(Length{kPointX}, Length{kPointY}, {.timeout = Time{4.0}})
            .reverseTo(Length{kPointX}, Length{kPointY}, {.timeout = Time{8.0}});
        // Measured: L 3.62 s, 0.36 in off the point; R 3.59 s, 0.37 in.
        CHECK(r.ok());
        CHECK(posErr(on.rig.h.truePose(), point()) < 1.0);
        CHECK(std::abs(on.rig.h.truePose().heading().errorTo(awayHeading())) < 0.10);
        CHECK(on.count(LogLevel::Warn, kNotSteered) == 0);

        WeakSide weakOff{wheel, 0.91};
        ReverseRig off{kin, startPose(), &weakOff, tankSteerOff()};
        Routine o{off.chassis, "weak-unsteered"};
        o.startAt(startPose())
            .faceAway(Length{kPointX}, Length{kPointY}, {.timeout = Time{4.0}})
            .reverseTo(Length{kPointX}, Length{kPointY}, {.timeout = Time{8.0}});
        // Measured: both sides TimedOut at the reverseTo, 3.32 in (L) and 2.76 in (R) beside it.
        CHECK_FALSE(o.ok());
        CHECK(std::string{o.result().stoppedName} == "reverseTo");
        CHECK(o.result().exit == ExitReason::TimedOut);
        CHECK(posErr(off.rig.h.truePose(), point()) > 1.5);
        CHECK(off.count(LogLevel::Warn, kNotSteered) == 0);  // off is silent: no window to miss
    }
}

// ═══ A2 — it really drives BACKWARDS ══════════════════════════════════════════════════════════

// Bug caught: a "reverse" that turns around and drives forwards — which A1's end state cannot tell
// apart from backing up, because both end on the point with the same heading. Only the path can:
// the true heading stays within ±10° of b + π on EVERY tick of the leg (it never turned round),
// the robot's displacement is backwards along its own heading, and it never moves forwards.
TEST_CASE("L9 A2: a tank's reverseTo leg drives BACKWARDS the whole way — never turns round, never "
          "moves forwards") {
    const TankKinematics kin{Length{12.0}};
    ReverseRig rr{kin};
    Routine r{rr.chassis, "backwards"};
    r.startAt(startPose()).faceAway(Length{kPointX}, Length{kPointY}, {.timeout = Time{4.0}});
    REQUIRE(r.ok());
    const std::size_t legStart = rr.pacer.truth.size();
    r.reverseTo(Length{kPointX}, Length{kPointY}, {.timeout = Time{8.0}});
    REQUIRE(r.ok());
    const std::vector<TruthSample>& t = rr.pacer.truth;
    REQUIRE(t.size() > legStart + 100);  // it really drove (about 200 ticks in the leg)

    const TruthSample& s0 = t[legStart];
    const double legLength = std::hypot(kPointX - s0.x, kPointY - s0.y);
    double worstBand = 0.0;
    double forward = 0.0;
    for (std::size_t i = legStart; i < t.size(); ++i) {
        worstBand = std::max(
            worstBand, std::abs(Angle::radians(t[i].theta).errorTo(awayHeading())));
        if (i + 1 < t.size()) {
            const double along = (t[i + 1].x - t[i].x) * std::cos(t[i].theta)
                                 + (t[i + 1].y - t[i].y) * std::sin(t[i].theta);
            if (along > 0.0) {
                forward += along;  // any motion the way the robot's nose points
            }
        }
    }
    const TruthSample& end = t.back();
    const double backward =
        -((end.x - s0.x) * std::cos(s0.theta) + (end.y - s0.y) * std::sin(s0.theta));

    // Measured: band 0.0129 rad (0.74°), backward 43.029 in of a 43.267 in leg, forward 0 in.
    CHECK(worstBand < 10.0 * Angle::kPi / 180.0);  // the discriminating assertion
    CHECK(backward > legLength - 1.0);
    CHECK(forward < 0.1);
    MESSAGE("reverse leg: band ", worstBand, " rad, backward ", backward, " of ", legLength,
            " in, forward ", forward, " in");
}

// Bug caught: "faceAway() first if the back must lead the whole way" being true only on a tank.
// On a holonomic drive reverseTo ALONE rotates in flight, so its nose leads early in the leg (A5);
// the docs' remedy is faceAway() first, and this is A2's per-tick check of that remedy on the
// X-drive: the heading stays in the band on every tick and the robot never moves nose-first.
TEST_CASE("L9 A2b: on the X-drive too, faceAway then reverseTo drives BACKWARDS the whole way") {
    const auto kin = xDrive(Length{7.0});
    ReverseRig rr{kin};
    Routine r{rr.chassis, "x-backwards"};
    r.startAt(startPose()).faceAway(Length{kPointX}, Length{kPointY}, {.timeout = Time{4.0}});
    REQUIRE(r.ok());
    const std::size_t legStart = rr.pacer.truth.size();
    r.reverseTo(Length{kPointX}, Length{kPointY}, {.timeout = Time{8.0}});
    REQUIRE(r.ok());
    const std::vector<TruthSample>& t = rr.pacer.truth;
    REQUIRE(t.size() > legStart + 100);  // it really drove (186 ticks in the leg)

    const TruthSample& s0 = t[legStart];
    const double legLength = std::hypot(kPointX - s0.x, kPointY - s0.y);
    double worstBand = 0.0;
    double forward = 0.0;
    for (std::size_t i = legStart; i < t.size(); ++i) {
        worstBand = std::max(
            worstBand, std::abs(Angle::radians(t[i].theta).errorTo(awayHeading())));
        if (i + 1 < t.size()) {
            const double along = (t[i + 1].x - t[i].x) * std::cos(t[i].theta)
                                 + (t[i + 1].y - t[i].y) * std::sin(t[i].theta);
            if (along > 0.0) {
                forward += along;
            }
        }
    }
    const TruthSample& end = t.back();
    const double backward =
        -((end.x - s0.x) * std::cos(s0.theta) + (end.y - s0.y) * std::sin(s0.theta));

    // Measured: band 0.0129 rad (0.74°), backward 43.030 in of a 43.267 in leg, forward 0 in.
    CHECK(worstBand < 10.0 * Angle::kPi / 180.0);
    CHECK(backward > legLength - 1.0);
    CHECK(forward < 0.1);
    CHECK(posErr(rr.rig.h.truePose(), point()) < 1.0);
}

// ═══ A4 — a tank WITHOUT faceAway ══════════════════════════════════════════════════════════════

// Bug caught: the documented un-faced behaviour drifting unnoticed — a reverseTo that some change
// starts steering from 180° off (the library making the author's turn, which C1's D12 and TANK1's
// window forbid), that loses its Warn, or that starts to "arrive" by a path nobody reviewed.
// What guide 9 and the VexBuilder verb table state, measured on the host test plant: from a robot
// FACING the point, reverseTo asks for a 180° heading change while translating; the leg starts
// outside TANK1's window, so it is NOT steered and logs ONE Warn naming 180.0 deg; the robot swings
// round as it drives and ends BESIDE the point, turned around, and the step times out — almost
// 8 in beside it at the end of this 43 in leg, and still about an inch at the end of a 6 in one.
// Landmine 7 (Angle::errorTo resolves an exact antipode to +π): the OUTCOME does not depend on it,
// only which SIDE the robot ends on — pinned by starting a hair either side of facing. The
// "exactly facing" start is NOT an exact antipode in doubles: the raw heading error is
// −3.1415926535897936, just past −π, and the wrap folds it to π − 4.4e-16 — counter-clockwise by
// ROUNDING, with errorTo's +π rule never reached (a mutant that disables the rule leaves this case
// green). A4d is the start where the rule itself decides.
TEST_CASE("L9 A4: reverseTo with no faceAway, from a robot FACING the point (tank) — not steered, "
          "one Warn, ends beside the point and times out") {
    const TankKinematics kin{Length{12.0}};
    const double lx = kPointX - kStartX;
    const double ly = kPointY - kStartY;
    const double lineLength = std::hypot(lx, ly);

    // offset = 0: exactly facing (CCW, by the wrap's rounding — see above); ±0.001 rad: the short
    // way. Measured: +7.807 / +7.808 / −7.808 in off the line; unwrapped rotation +3.1416 /
    // +3.1406 / −3.1406 rad; TimedOut at the 8 s budget (the robot stops moving at 3.39 s).
    for (const double offset : {0.0, 1e-3, -1e-3}) {
        CAPTURE(offset);
        const Pose2d facing{Length{kStartX}, Length{kStartY},
                            Angle::radians(startBearing() + offset)};
        ReverseRig rr{kin, facing};
        Routine r{rr.chassis, "no-face-away"};
        r.startAt(facing).reverseTo(Length{kPointX}, Length{kPointY}, {.timeout = Time{8.0}});

        CHECK_FALSE(r.ok());
        CHECK(r.result().cause == RoutineStopCause::MotionFailed);
        CHECK(r.result().exit == ExitReason::TimedOut);
        CHECK(std::string{r.result().stoppedName} == "reverseTo");
        // ONE Warn from TANK1, naming the start (180.0 deg exactly facing, 179.9 a hair off) and
        // the window — the leg was not steered.
        CHECK(rr.count(LogLevel::Warn, kNotSteered) == 1);
        CHECK(rr.count(LogLevel::Warn, offset == 0.0 ? "starts 180.0 deg off its target heading"
                                                     : "starts 179.9 deg off its target heading")
              == 1);
        CHECK(rr.count(LogLevel::Warn, "(steers within 15.0 deg) -- turn first") == 1);

        const Pose2d truth = rr.rig.h.truePose();
        const double cross = ((truth.x().value() - kStartX) * ly
                              - (truth.y().value() - kStartY) * lx) / lineLength;
        const double rotated = rr.pacer.truth.back().theta - rr.pacer.truth.front().theta;
        CHECK(std::abs(truth.heading().errorTo(awayHeading())) < 0.10);  // it DID turn round…
        CHECK(std::abs(rotated) > 3.0);                                  // …by swinging ~180°
        CHECK(posErr(truth, point()) > 7.0);  // …but ends beside the point, not on it
        CHECK(posErr(truth, point()) < 8.5);
        // The side follows the way it turned: a CCW (+) swing ends to the RIGHT of the line of
        // travel (+ in this cross product — positive is right of start → point), CW to the left.
        CHECK((rotated > 0.0) == (cross > 0.0));
        if (offset == 0.0) {
            CHECK(rotated > 0.0);  // π − 4.4e-16 after the wrap: CCW by rounding (not the rule — A4d)
        }
        checkSafeState(rr);
    }

    // A short leg does not rescue it: 6 in from the point, still facing it, the same unsteered
    // swing ends about an inch beside the point — outside the 0.5 in rule, so it too times out.
    // Measured: 1.067 in, TimedOut, one Warn naming 180.0 deg.
    const double b = startBearing();
    const Pose2d shortStart{Length{kPointX - 6.0 * std::cos(b)}, Length{kPointY - 6.0 * std::sin(b)},
                            Angle::radians(b)};
    ReverseRig sr{kin, shortStart};
    Routine s{sr.chassis, "short-no-face-away"};
    s.startAt(shortStart).reverseTo(Length{kPointX}, Length{kPointY}, {.timeout = Time{8.0}});
    CHECK_FALSE(s.ok());
    CHECK(s.result().exit == ExitReason::TimedOut);
    CHECK(std::string{s.result().stoppedName} == "reverseTo");
    CHECK(sr.count(LogLevel::Warn, "starts 180.0 deg off its target heading") == 1);
    CHECK(posErr(sr.rig.h.truePose(), point()) > 0.8);
    CHECK(posErr(sr.rig.h.truePose(), point()) < 1.3);
}

// Bug caught: the docs' "a robot that just drove straight out from the point can back straight
// back without faceAway()" going false — a reverseTo whose leg, from a robot whose back is ALREADY
// to the point, starts outside TANK1's window (a wrong + π, a bearing read from the wrong pose),
// so the most common way a student backs up (score, then back out the way it came) stops steering.
// The realistic flow: drive out to the point, then reverseTo where it started, no turn between.
TEST_CASE("L9 A4b: backing straight back the way it came needs no faceAway — the leg starts on "
          "its heading and is steered (tank)") {
    const TankKinematics kin{Length{12.0}};
    const Pose2d out = Pose2d{Length{kStartX}, Length{kStartY}, Angle::radians(startBearing())};
    const Pose2d home{Length{kStartX}, Length{kStartY}, Angle{}};
    for (const int wheel : {-1, 0, 1}) {
        CAPTURE(wheel);  // −1 = a clean drive; 0 / 1 = the left / right side 9 % weak
        WeakSide weak{wheel < 0 ? 0 : wheel, wheel < 0 ? 1.0 : 0.91};
        ReverseRig rr{kin, out, &weak};
        Routine r{rr.chassis, "back-out"};
        r.startAt(out).driveTo(Length{kPointX}, Length{kPointY}, {.timeout = Time{8.0}});
        REQUIRE(r.ok());
        const std::size_t legStart = rr.pacer.truth.size();
        r.reverseTo(Length{kStartX}, Length{kStartY}, {.timeout = Time{8.0}});
        // Measured: clean 1.85 s, 0.230 in from where it started; a side ×0.91: 1.94 s, 0.364 in.
        CHECK(r.ok());
        CHECK(posErr(rr.rig.h.truePose(), home) < 1.0);
        // It backed in facing the way it drove out — the arrival heading is the outbound bearing.
        CHECK(std::abs(rr.rig.h.truePose().heading().errorTo(Angle::radians(startBearing()))) < 0.10);
        // Inside the window: no "not steered" Warn — and with no Warn on a tank leg, TANK1 engaged.
        CHECK(rr.count(LogLevel::Warn, kNotSteered) == 0);
        // …and it really backed: never nose-first on the way back (measured 0 in).
        double forward = 0.0;
        const std::vector<TruthSample>& t = rr.pacer.truth;
        for (std::size_t i = legStart; i + 1 < t.size(); ++i) {
            const double along = (t[i + 1].x - t[i].x) * std::cos(t[i].theta)
                                 + (t[i + 1].y - t[i].y) * std::sin(t[i].theta);
            if (along > 0.0) {
                forward += along;
            }
        }
        CHECK(forward < 0.1);
    }
}

// Bug caught: "a reverseTo leg starts as far off its heading as the robot's back is turned from the
// point" — what the docs say instead of "always 180°" — being wrong in the middle, or the window
// being measured from the NOSE (a leg decided on the bearing, not the bearing + π). The back is
// turned 10° (inside the 15° window: steered, it arrives, no Warn), 20° (just outside: the Warn
// names 20.0) and 90° either way (the Warn names 90.0). A4 is 180°, A4b about 0.
TEST_CASE("L9 A4c: a reverseTo leg starts as far off its heading as the robot's back is turned "
          "from the point — steered inside 15°, a Warn naming the offset outside it (tank)") {
    const TankKinematics kin{Length{12.0}};
    struct Case {
        double backOffDeg;     // how far the robot's BACK is turned from the point
        const char* warnText;  // nullptr = inside the window: no Warn, and it arrives
    };
    // Measured: 10° → Settled 0.258 in, no Warn; 20° / 90° / −90° → one Warn each, naming the
    // offset (then TimedOut, 4.5 / 11.6 / 11.6 in off — not a claim, not pinned).
    for (const Case c : {Case{10.0, nullptr},
                         Case{20.0, "starts 20.0 deg off its target heading"},
                         Case{90.0, "starts 90.0 deg off its target heading"},
                         Case{-90.0, "starts 90.0 deg off its target heading"}}) {
        CAPTURE(c.backOffDeg);
        const Pose2d start{Length{kStartX}, Length{kStartY},
                           Angle::radians(startBearing() + Angle::kPi
                                          + c.backOffDeg * Angle::kPi / 180.0)};
        ReverseRig rr{kin, start};
        Routine r{rr.chassis, "back-turned"};
        r.startAt(start).reverseTo(Length{kPointX}, Length{kPointY}, {.timeout = Time{8.0}});
        if (c.warnText == nullptr) {
            CHECK(r.ok());
            CHECK(posErr(rr.rig.h.truePose(), point()) < 1.0);
            CHECK(rr.count(LogLevel::Warn, kNotSteered) == 0);
        } else {
            CHECK(rr.count(LogLevel::Warn, kNotSteered) == 1);
            CHECK(rr.count(LogLevel::Warn, c.warnText) == 1);
        }
    }
}

// Bug caught: landmine 7 unguarded — Angle::errorTo's exact-antipode rule (+π, never −π; F3)
// broken or bypassed, which A4 cannot see (its "exactly facing" start rounds past the antipode).
// Here the antipode IS exact in doubles: the robot faces straight down −x (heading +π, exactly
// representable) at a point on its own y, so the bearing is atan2(+0, −36) = +π, reverseTo's target
// is +π + π = 0 after the wrap, and the raw heading error is exactly −π — which the rule, and only
// the rule, turns into +π. So the swing is the rule's: counter-clockwise, ending RIGHT of the line
// of travel (+y here). (Measured: +3.1416 rad, 7.100 in beside the point at y = −4.900, TimedOut;
// with the rule disabled — `w <= −kPi` → `w < −kPi` in angle.hpp — it swings CW, −3.1416 rad, to
// y = −19.100.) At bearing π, `b + π` and `b − π` are the same bits, so this case is deliberately
// NOT one of the geometry rule's: it exists for the rule, not the steps.
TEST_CASE("L9 A4d: landmine 7 — at an EXACT antipode, errorTo's +π decides the swing: "
          "counter-clockwise (tank)") {
    const TankKinematics kin{Length{12.0}};
    const Pose2d start{Length{kStartX}, Length{kStartY}, Angle::radians(Angle::kPi)};
    const double pointX = kStartX - 36.0;  // straight ahead of the robot, on its own y
    REQUIRE(start.heading().radians() == Angle::kPi);  // +π exactly, never −π (F3)
    // The premise, checked rather than assumed: reverseTo's target, spelled as the library spells
    // it, is 0, and the raw heading error is exactly −π, before errorTo's rule.
    const Angle target = Angle::radians(std::atan2(0.0, pointX - kStartX))
                         + Angle::radians(Angle::kPi);
    REQUIRE(target.radians() == 0.0);
    REQUIRE(target.radians() - start.heading().radians() == -Angle::kPi);
    CHECK(start.heading().errorTo(target) == Angle::kPi);

    ReverseRig rr{kin, start};
    Routine r{rr.chassis, "exact-antipode"};
    r.startAt(start).reverseTo(Length{pointX}, Length{kStartY}, {.timeout = Time{8.0}});

    CHECK_FALSE(r.ok());
    CHECK(r.result().exit == ExitReason::TimedOut);
    CHECK(rr.count(LogLevel::Warn, "starts 180.0 deg off its target heading") == 1);
    const double rotated = rr.pacer.truth.back().theta - rr.pacer.truth.front().theta;
    const Pose2d truth = rr.rig.h.truePose();
    CHECK(rotated > 3.0);  // counter-clockwise, through the half turn
    // A CCW swing ends to the RIGHT of the line of travel (A4's sign relation). Travel here is
    // down −x, and facing −x a robot's right is +y: it ends above the start's y (measured +7.100).
    CHECK(truth.y().value() > kStartY + 5.0);
    CHECK(std::abs(truth.heading().errorTo(target)) < 0.10);
}

// ═══ A5 — the X-drive ═════════════════════════════════════════════════════════════════════════

// Bug caught: reverseTo only "working" on a tank because the author turned first. A holonomic drive
// rotates in flight (C1's thesis), so reverseTo ALONE, from a start heading nowhere near the
// arrival heading, must still land on the point turned around — with no Warn: the tank's window
// is not the X-drive's business. And the docs' other half: rotating in flight, it does NOT back in
// the whole way — its nose leads early in the leg — which is why they say faceAway() first when
// the back must lead (A2b). A reverseTo that planned its own turn first (the library making the
// author's turn) would drive no inch nose-first and turn this case red.
TEST_CASE("L9 A5: on the X-drive, reverseTo alone arrives turned around, rotating in flight") {
    const auto kin = xDrive(Length{7.0});
    ReverseRig rr{kin};
    Routine r{rr.chassis, "x-backs-in"};
    r.startAt(startPose()).reverseTo(Length{kPointX}, Length{kPointY}, {.timeout = Time{8.0}});
    REQUIRE(r.ok());
    // Measured: 2.07 s, 0.231 in from the point, heading 0.0016 rad from b + π.
    const Pose2d truth = rr.rig.h.truePose();
    CHECK(posErr(truth, point()) < 1.0);
    CHECK(std::abs(truth.heading().errorTo(awayHeading())) < 0.10);
    // …and it really rotated in flight to get there: the start heading, 90°, is 2.55 rad from
    // b + π (0.59 rad from the bearing itself), so this is not a leg that began turned around.
    CHECK(std::abs(Angle::degrees(90.0).errorTo(awayHeading())) > 2.0);
    for (int i = 0; i < rr.log.size(); ++i) {
        CHECK(rr.log.at(i).level != LogLevel::Warn);
    }
    // The nose leads early: travel along the robot's own nose, summed over the leg. Measured
    // 4.746 in (its heading reaches within 10° of b + π only 35.8 in into the 43.3 in leg).
    const std::vector<TruthSample>& t = rr.pacer.truth;
    double forward = 0.0;
    for (std::size_t i = 0; i + 1 < t.size(); ++i) {
        const double along = (t[i + 1].x - t[i].x) * std::cos(t[i].theta)
                             + (t[i + 1].y - t[i].y) * std::sin(t[i].theta);
        if (along > 0.0) {
            forward += along;
        }
    }
    CHECK(forward > 1.0);
}

// ═══ A6 — the error policy, through both new steps ═════════════════════════════════════════════

// Bug caught: a new step that fails without stopping the chain, stops under the WRONG name (a
// transcript saying "(driveTo)" for a reverseTo — mutation 4 — or "(face)" for a faceAway, whose
// motion-stop name is a second literal only a STARVED faceAway reaches — mutation 13), or drives
// when the chain has stopped (mutation 5). Each step carries TWO name literals — its skip name
// and its stop name — and every one is read here.
TEST_CASE("L9 A6: a starved reverseTo stops the chain under its own name; later steps skip, "
          "named") {
    const TankKinematics kin{Length{12.0}};
    ReverseRig rr{kin};
    Routine r{rr.chassis, "starved-reverse"};
    r.startAt(startPose())
        .faceAway(Length{kPointX}, Length{kPointY}, {.timeout = Time{4.0}})
        .reverseTo(Length{kPointX}, Length{kPointY},
                   {.timeout = Time{0.3}});  // starved: 43 in in 0.3 s
    REQUIRE_FALSE(r.ok());
    const int motionsAtStop = rr.chassis.scheduler().motionsStarted();
    const Pose2d parked = rr.rig.h.truePose();
    const double parkedAt = rr.now();

    // Every later step must be skipped, and named — including both new steps.
    r.faceAway(Length{24.0}, Length{-24.0}, {.timeout = Time{4.0}})
        .reverseTo(Length{24.0}, Length{-24.0}, {.timeout = Time{8.0}})
        .driveTo(Length{-30.0}, Length{30.0}, {.timeout = Time{8.0}});

    // …including a step aimed at the PARKED ESTIMATE itself. On a clean chain that point has no
    // bearing and throws (A7); a stopped chain must SKIP it before computing any bearing, or a step
    // the error policy says is skipped ends the program with an uncaught throw — e.g. startAt(P),
    // a first step that fails without moving (a never-live boot), then reverseTo(P). face and
    // driveTo share bearingTo and the same skip-first order; theirs had no pin before this case
    // (D1), so all four field steps are pinned here together.
    const Pose2d parkedEstimate = rr.chassis.pose();
    CHECK_NOTHROW(r.reverseTo(parkedEstimate.x(), parkedEstimate.y()));
    CHECK_NOTHROW(r.faceAway(parkedEstimate.x(), parkedEstimate.y()));
    CHECK_NOTHROW(r.face(parkedEstimate.x(), parkedEstimate.y()));
    CHECK_NOTHROW(r.driveTo(parkedEstimate.x(), parkedEstimate.y()));

    const RoutineResult res = r.result();
    CHECK(res.stoppedAt == 3);
    CHECK(std::string{res.stoppedName} == "reverseTo");
    CHECK(res.cause == RoutineStopCause::MotionFailed);
    CHECK(res.exit == ExitReason::TimedOut);
    CHECK(res.completed == 2);
    CHECK(res.skipped == 7);
    CHECK(res.steps == 10);

    CHECK(rr.count(LogLevel::Warn, "'starved-reverse' STOPPED at step 3 (reverseTo): motion TIMEOUT")
          == 1);
    CHECK(rr.count(LogLevel::Info, "step 4 (faceAway) skipped — stopped at step 3") == 1);
    CHECK(rr.count(LogLevel::Info, "step 5 (reverseTo) skipped — stopped at step 3") == 1);
    CHECK(rr.count(LogLevel::Info, "step 6 (driveTo) skipped — stopped at step 3") == 1);
    CHECK(rr.count(LogLevel::Info, "step 7 (reverseTo) skipped — stopped at step 3") == 1);
    CHECK(rr.count(LogLevel::Info, "step 8 (faceAway) skipped — stopped at step 3") == 1);
    CHECK(rr.count(LogLevel::Info, "step 9 (face) skipped — stopped at step 3") == 1);
    CHECK(rr.count(LogLevel::Info, "step 10 (driveTo) skipped — stopped at step 3") == 1);

    // The skipped steps did not drive: no motion started, the robot did not move, no time passed.
    CHECK(rr.chassis.scheduler().motionsStarted() == motionsAtStop);
    CHECK(posErr(rr.rig.h.truePose(), parked) == 0.0);
    CHECK(rr.now() == parkedAt);
    checkSafeState(rr);
}

// Bug caught: faceAway's STOP name — the second literal, reached only when its own motion fails —
// drifting to "face" (mutation 13) while its skip name stays right, so every skip line reads true
// and the one line that explains a stop names the wrong step.
TEST_CASE("L9 A6: a starved faceAway stops the chain naming faceAway") {
    const TankKinematics kin{Length{12.0}};
    ReverseRig rr{kin};
    Routine r{rr.chassis, "starved-turn"};
    r.startAt(startPose())
        .faceAway(Length{kPointX}, Length{kPointY},
                  {.timeout = Time{0.1}})  // starved: 147° of turn in 0.1 s
        .reverseTo(Length{kPointX}, Length{kPointY}, {.timeout = Time{8.0}});

    const RoutineResult res = r.result();
    CHECK_FALSE(res.ok);
    CHECK(res.stoppedAt == 2);
    CHECK(std::string{res.stoppedName} == "faceAway");
    CHECK(res.cause == RoutineStopCause::MotionFailed);
    CHECK(res.exit == ExitReason::TimedOut);
    CHECK(res.skipped == 1);
    CHECK(rr.count(LogLevel::Warn, "'starved-turn' STOPPED at step 2 (faceAway): motion TIMEOUT")
          == 1);
    CHECK(rr.count(LogLevel::Info, "step 3 (reverseTo) skipped — stopped at step 2") == 1);
    CHECK(rr.chassis.scheduler().motionsStarted() == 1);  // the turn only; the reverse never ran
    checkSafeState(rr);
}

// ═══ A7 — misuse ═══════════════════════════════════════════════════════════════════════════════

// Bug caught: nonsense accepted quietly, or a precondition throw corrupting the chain. A point the
// robot is ALREADY on has no bearing (so no "away" either), a NaN point has none either, and a NaN
// timeout is the facade's to refuse — each throws straight out of the step, the counters stay
// exactly where they were, nothing moves, and the chain stays usable.
TEST_CASE("L9 A7: misuse throws out of faceAway / reverseTo; the chain is untouched and usable") {
    const TankKinematics kin{Length{12.0}};
    ReverseRig rr{kin};
    const double nan = std::numeric_limits<double>::quiet_NaN();

    Routine r{rr.chassis, "misuse"};
    r.startAt(startPose()).faceAway(Length{kPointX}, Length{kPointY}, {.timeout = Time{4.0}});
    REQUIRE(r.ok());
    const RoutineResult before = r.result();
    const int motionsBefore = rr.chassis.scheduler().motionsStarted();
    const double t0 = rr.now();

    const Pose2d here = rr.chassis.pose();
    CHECK_THROWS_AS(r.faceAway(here.x(), here.y()), PreconditionError);
    CHECK_THROWS_AS(r.reverseTo(here.x(), here.y()), PreconditionError);
    CHECK_THROWS_AS(r.faceAway(Length{nan}, Length{0.0}), PreconditionError);
    CHECK_THROWS_AS(r.reverseTo(Length{0.0}, Length{nan}), PreconditionError);
    CHECK_THROWS_AS(r.faceAway(Length{kPointX}, Length{kPointY}, {.timeout = Time{nan}}),
                    PreconditionError);
    CHECK_THROWS_AS(r.reverseTo(Length{kPointX}, Length{kPointY}, {.timeout = Time{nan}}),
                    PreconditionError);
    CHECK_THROWS_AS(r.reverseTo(Length{kPointX}, Length{kPointY}, {.timeout = Time{-1.0}}),
                    PreconditionError);

    // The message names the degenerate case AND the steps that can reach it: L9 widened it from
    // "face/driveTo" to all four field steps, and the changelog says so — so it is pinned, from
    // either new step (and a step that stopped throwing here would fail the `threw` check, not
    // pass silently).
    for (const bool viaReverseTo : {true, false}) {
        CAPTURE(viaReverseTo);
        bool threw = false;
        try {
            if (viaReverseTo) {
                r.reverseTo(here.x(), here.y());
            } else {
                r.faceAway(here.x(), here.y());
            }
        } catch (const PreconditionError& e) {
            threw = true;
            const std::string what = e.what();
            CHECK(what.find("no defined bearing") != std::string::npos);
            CHECK(what.find("reverseTo") != std::string::npos);
            CHECK(what.find("faceAway") != std::string::npos);
        }
        CHECK(threw);
    }

    // None of that counted, stopped the chain, started a motion or took any time.
    const RoutineResult after = r.result();
    CHECK(after.ok);
    CHECK(after.steps == before.steps);
    CHECK(after.completed == before.completed);
    CHECK(rr.chassis.scheduler().motionsStarted() == motionsBefore);
    CHECK(rr.now() == t0);

    // Still fully usable: the reverse leg runs and arrives.
    r.reverseTo(Length{kPointX}, Length{kPointY}, {.timeout = Time{8.0}});
    CHECK(r.ok());
    CHECK(r.result().completed == before.completed + 1);
    CHECK(posErr(rr.rig.h.truePose(), point()) < 1.0);
}

// ═══ A12 — the caller's options, forwarded WHOLE ═══════════════════════════════════════════════

// Bug caught: a new step forwarding only the timeout (a fresh MotionOptions with the timeout copied
// — mutation 14) or default options. A capped leg that loses its cap still SETTLES, just sooner, so
// only time can see it: the capped leg must take genuinely longer than its uncapped twin. One case
// per step, each on the field its verb reads: the turn's yaw budget, the leg's speed budget.
TEST_CASE("L9 A12: faceAway forwards the yaw budget and reverseTo the speed budget") {
    const TankKinematics kin{Length{12.0}};

    auto turnTime = [&](const MotionOptions& options) {
        ReverseRig rr{kin};
        Routine r{rr.chassis, "turn-budget"};
        r.startAt(startPose()).faceAway(Length{kPointX}, Length{kPointY}, options);
        REQUIRE(r.ok());
        return rr.now();
    };
    auto reverseLegTime = [&](const MotionOptions& options) {
        ReverseRig rr{kin};
        Routine r{rr.chassis, "speed-budget"};
        r.startAt(startPose()).faceAway(Length{kPointX}, Length{kPointY}, {.timeout = Time{4.0}});
        REQUIRE(r.ok());
        const double t0 = rr.now();
        r.reverseTo(Length{kPointX}, Length{kPointY}, options);
        REQUIRE(r.ok());
        return rr.now() - t0;
    };

    // Measured: faceAway 1.35 s uncapped, 3.05 s at ≤1 rad/s (×2.26).
    const double turnFree = turnTime({.timeout = Time{8.0}});
    const double turnCapped =
        turnTime({.timeout = Time{8.0}, .maxAngularSpeed = AngularVelocity{1.0}});
    CHECK(turnCapped > turnFree * 1.5);

    // Measured: the reverseTo leg 1.86 s uncapped, 3.57 s at ≤15 in/s (×1.92).
    const double legFree = reverseLegTime({.timeout = Time{8.0}});
    const double legCapped =
        reverseLegTime({.timeout = Time{8.0}, .maxLinearSpeed = Velocity{15.0}});
    CHECK(legCapped > legFree * 1.5);
    MESSAGE("faceAway ", turnFree, " s -> ", turnCapped, " s capped; reverseTo leg ", legFree,
            " s -> ", legCapped, " s capped");
}

// Bug caught: reverseTo forwarding the timeout and the speed budget but dropping the YAW budget
// (the review's surviving mutant N1). On a tank the leg after faceAway barely turns, so a yaw cap
// never binds there (measured: bit-identical at 0.5 rad/s) — only a leg that turns in flight can
// see it: the X-drive, backing in from 90°, rotates 2.55 rad on the way. MotionOptions has three
// fields; reverseTo reads all three (timeout: A6; maxLinearSpeed: A12, A1; maxAngularSpeed: here).
// faceAway reads two — a turn in place commands no translation, so a maxLinearSpeed cap cannot
// change it (measured bit-identical at 0.01 in/s on both drives); its timeout and yaw budget are
// A6's and A12's.
TEST_CASE("L9 A12b: reverseTo forwards the yaw budget too — on the X-drive, where the leg turns "
          "in flight") {
    const auto kin = xDrive(Length{7.0});
    auto legTime = [&](const MotionOptions& options) {
        ReverseRig rr{kin};
        Routine r{rr.chassis, "x-yaw-budget"};
        r.startAt(startPose()).reverseTo(Length{kPointX}, Length{kPointY}, options);
        REQUIRE(r.ok());
        return rr.now();
    };
    // Measured: 2.07 s uncapped, 5.45 s at ≤0.5 rad/s (×2.63).
    const double uncapped = legTime({.timeout = Time{8.0}});
    const double capped =
        legTime({.timeout = Time{8.0}, .maxAngularSpeed = AngularVelocity{0.5}});
    CHECK(capped > uncapped * 1.5);
    MESSAGE("X-drive reverseTo ", uncapped, " s -> ", capped, " s at <= 0.5 rad/s");
}
