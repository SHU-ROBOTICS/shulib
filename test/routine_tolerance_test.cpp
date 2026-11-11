// STEP1-A — the ARRIVAL TOLERANCE (chassis::MotionOptions::positionTolerance / headingTolerance,
// motion::loosenedSettle; the STEP1 brief's D2 (a)), on the host test plant: a 12 in tank on the A2
// plant, both the memoryless plant (kA 0) and the motor-lag plant TANK1's probes used (kA 0.02) —
// the brief's probe P3, made a test. Every number is a host-test-plant number, never a robot's; how
// far robot two coasts after a leg that ends at the loosened speed bound is A4: HA-216, unmeasured.
// Say "host test plant", never "simulator".
//
// Each case attacks ONE rule (the brief's §2.2 numbers): ST2 a zero tolerance changes nothing, ST3 a
// looser tolerance ends a leg EARLIER and still within it — the distance ALONE would change nothing
// on a drive (P3: 1.80 s at 0.5, 1, 2 and 4 in alike), ST23 a tighter tolerance never lowers the
// rate bound, the brake's settle is untouched, and a robot whose own settle distance is 0 never
// divides by it.

#include "doctest.h"

#include <cmath>
#include <limits>
#include <vector>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/control/settled_util.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/motion/motion_config.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/units/literals.hpp"

using namespace shulib::units::literals;
using shulib::chassis::Chassis;
using shulib::chassis::MotionOptions;
using shulib::control::ExitReason;
using shulib::control::SettleConfig;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::motion::loosenedSettle;
using shulib::units::Length;
using shulib::units::Time;

namespace {

constexpr double kPi = 3.14159265358979323846;

/// The world below the chassis: the A2 plant, one 10 ms step per pace, and a per-pace trace of the
/// TRUE pose (the bit-identity witness of ST2).
struct World final : shulib::motion::ITickPacer {
    motion_rig::MotionRig* rig = nullptr;
    std::vector<double> trace;
    void pace() override {
        rig->h.plant().step(Time{0.01});
        const Pose2d p = rig->h.truePose();
        trace.push_back(p.x().value());
        trace.push_back(p.y().value());
        trace.push_back(p.heading().radians());
    }
};

/// The brief's probe tank (P3): 12 in track, the plant's kA chosen per case, the motion config whose
/// feedforward matches the plant's (motion_test_rig.hpp's gain agreement).
struct Tank {
    shulib::hal::fake::FakeTelemetrySink sink;
    shulib::kinematics::TankKinematics kin{Length{12.0}};
    motion_rig::MotionRig rig;
    World world;
    Chassis ch;
    explicit Tank(double plantKa)
        : rig{kin, cfgFor(plantKa), &sink},
          ch{rig.deps, world, shulib::chassis::ChassisConfig{.motion = motion_rig::motionConfig()}} {
        world.rig = &rig;
    }
    static shulib::sim::SimHarnessConfig cfgFor(double kA) {
        auto c = motion_rig::plantConfig();
        c.plant.wheelFf.kA = kA;
        return c;
    }
    [[nodiscard]] double now() { return rig.h.clock().now().value(); }
};

const Pose2d kForty{Length{40.0}, Length{0.0}, Angle{}};

/// One 40 in drive along +x (robot two's "40 in auton" leg); returns how long it took.
struct Leg {
    ExitReason exit = ExitReason::Running;
    double seconds = 0.0;
    double trueShort = 0.0;  ///< the TRUE distance to the target at the end, inches
    Pose2d truth{};
};
Leg drive40(double kA, const MotionOptions& o) {
    Tank t{kA};
    const double t0 = t.now();
    Leg l;
    l.exit = t.ch.moveTo(kForty, o);
    l.seconds = t.now() - t0;
    l.truth = t.rig.h.truePose();
    l.trueShort = std::hypot(l.truth.x().value() - 40.0, l.truth.y().value());
    return l;
}
/// One 90° turn in place.
Leg turn90(double kA, const MotionOptions& o) {
    Tank t{kA};
    const double t0 = t.now();
    Leg l;
    l.exit = t.ch.turnTo(Angle::degrees(90.0), o);
    l.seconds = t.now() - t0;
    l.truth = t.rig.h.truePose();
    l.trueShort = std::abs(l.truth.heading().errorTo(Angle::degrees(90.0)));
    return l;
}

}  // namespace

TEST_CASE("STEP1-A ST2: a ZERO tolerance changes nothing -- the 40 in leg and the 90 deg turn, tick for tick") {
    // Bug caught: a tolerance of 0 read as "a tolerance of 0" (maxError = 0: the leg could never
    // settle), or any change to the default path at all — the brief's "0 = the robot's own settle
    // numbers, bit for bit". The witness is every pace's TRUE pose, the exit and its tick, compared
    // EXACTLY (==, not a tolerance).
    for (const double kA : {0.0, 0.02}) {
        CAPTURE(kA);
        Tank a{kA};
        Tank b{kA};
        const ExitReason ea = a.ch.moveTo(kForty);
        const ExitReason eb = b.ch.moveTo(kForty, {.positionTolerance = 0_in, .headingTolerance = 0_deg});
        CHECK(ea == ExitReason::Settled);
        CHECK(ea == eb);
        CHECK(a.ch.lastCompleted().endTime.value() == b.ch.lastCompleted().endTime.value());
        CHECK(a.world.trace == b.world.trace);
        const ExitReason ta = a.ch.turnTo(Angle::degrees(90.0));
        const ExitReason tb = b.ch.turnTo(Angle::degrees(90.0), {.positionTolerance = 0_in, .headingTolerance = 0_deg});
        CHECK(ta == ExitReason::Settled);
        CHECK(ta == tb);
        CHECK(a.ch.lastCompleted().endTime.value() == b.ch.lastCompleted().endTime.value());
        CHECK(a.world.trace == b.world.trace);
        CHECK(a.world.trace.size() > 100);  // the witness is a real trace, not two empty vectors
    }
}

TEST_CASE("STEP1-A ST3: a looser tolerance ends a leg EARLIER, and within it -- both plants (P3)") {
    // Bug caught: the tolerance that raises the distance ALONE (G3a R.13's text — the brief's SM16): on
    // a drive leg the SPEED bound decides when the leg ends, so that does nothing (P3: 1.80 s at every
    // distance); the position tolerance applied to the heading (SM4); a tolerance that ends the leg
    // OUTSIDE itself. Times are the probe's, to the tick (each leg is a whole number of 10 ms paces).
    // The memoryless plant: 1.80 s default; 1.35 s at 2 in (0.93 in short); 1.12 s at 4 in.
    const Leg d0 = drive40(0.0, {});
    const Leg d2 = drive40(0.0, {.positionTolerance = 2_in});
    const Leg d4 = drive40(0.0, {.positionTolerance = 4_in});
    CHECK(d0.exit == ExitReason::Settled);
    CHECK(d2.exit == ExitReason::Settled);
    CHECK(d4.exit == ExitReason::Settled);
    CHECK(d0.seconds == doctest::Approx(1.80).epsilon(0.001));
    CHECK(d2.seconds == doctest::Approx(1.35).epsilon(0.001));
    CHECK(d4.seconds == doctest::Approx(1.12).epsilon(0.001));
    CHECK(d2.trueShort < 2.0);
    CHECK(d2.trueShort > 0.5);  // it really did stop where the default would not have
    CHECK(d4.trueShort < 4.0);
    // The motor-lag plant: 1.40 s default; 1.25 s at 2 in; 1.14 s at 4 in.
    const Leg l0 = drive40(0.02, {});
    const Leg l2 = drive40(0.02, {.positionTolerance = 2_in});
    const Leg l4 = drive40(0.02, {.positionTolerance = 4_in});
    CHECK(l0.seconds == doctest::Approx(1.40).epsilon(0.001));
    CHECK(l2.seconds == doctest::Approx(1.25).epsilon(0.001));
    CHECK(l4.seconds == doctest::Approx(1.14).epsilon(0.001));
    CHECK(l2.trueShort < 2.0);
    CHECK(l4.trueShort < 4.0);
    // The turn: 1.18 s default, 0.82 s at 5° (memoryless); 1.21 s → 0.63 s (lag) — within 5°.
    const Leg t0 = turn90(0.0, {});
    const Leg t5 = turn90(0.0, {.headingTolerance = 5_deg});
    CHECK(t0.exit == ExitReason::Settled);
    CHECK(t5.exit == ExitReason::Settled);
    CHECK(t0.seconds == doctest::Approx(1.18).epsilon(0.001));
    CHECK(t5.seconds == doctest::Approx(0.82).epsilon(0.001));
    CHECK(t5.trueShort < 5.0 * kPi / 180.0);
    const Leg lt0 = turn90(0.02, {});
    const Leg lt5 = turn90(0.02, {.headingTolerance = 5_deg});
    CHECK(lt0.seconds == doctest::Approx(1.21).epsilon(0.001));
    CHECK(lt5.seconds == doctest::Approx(0.63).epsilon(0.001));
    CHECK(lt5.trueShort < 5.0 * kPi / 180.0);
    // A heading tolerance does not reach the drive leg's DISTANCE, and a position tolerance does not
    // reach a turn: each axis reads its own.
    CHECK(drive40(0.0, {.headingTolerance = 5_deg}).seconds == doctest::Approx(1.80).epsilon(0.001));
    CHECK(turn90(0.0, {.positionTolerance = 4_in}).seconds == doctest::Approx(1.18).epsilon(0.001));
}

TEST_CASE("STEP1-A ST3: the tolerance reaches the Routine's steps -- driveTo carries it through moveTo") {
    // Bug caught: a Routine step that rebuilds its options and drops the new fields.
    Tank a{0.0};
    Tank b{0.0};
    shulib::chassis::Routine ra{a.ch, "default"};
    shulib::chassis::Routine rb{b.ch, "loose"};
    const double a0 = a.now();
    const double b0 = b.now();
    ra.driveTo(40_in, 0_in);
    rb.driveTo(40_in, 0_in, {.positionTolerance = 2_in});
    CHECK(ra.ok());
    CHECK(rb.ok());
    CHECK(a.now() - a0 == doctest::Approx(1.80).epsilon(0.001));
    CHECK(b.now() - b0 == doctest::Approx(1.35).epsilon(0.001));
}

TEST_CASE("STEP1-A ST23: loosenedSettle -- looser scales the rate, tighter never lowers it, settleTime kept") {
    // Bug caught: a tighter tolerance that lowers the rate bound in proportion (SM17: a 0.5° turn
    // whose rate bound sits inside the IMU's noise would never settle — motion_config.hpp's header);
    // a formula that touches settleTime.
    const shulib::motion::MotionConfig mc{};
    const SettleConfig tr = mc.translationSettle;  // {0.5 in, 1.0 in/s, 0.1 s}
    const SettleConfig hd = mc.headingSettle;      // {0.02 rad, 0.30 rad/s, 0.1 s}
    const SettleConfig t025 = loosenedSettle(tr, 0.25);
    CHECK(t025.maxError == 0.25);
    CHECK(t025.maxErrorRate == tr.maxErrorRate);  // tighter: the rate is NOT lowered
    CHECK(t025.settleTime == tr.settleTime);
    const double halfDeg = 0.5 * kPi / 180.0;
    const SettleConfig h05 = loosenedSettle(hd, halfDeg);
    CHECK(h05.maxError == halfDeg);
    CHECK(h05.maxErrorRate == hd.maxErrorRate);
    const SettleConfig t2 = loosenedSettle(tr, 2.0);
    CHECK(t2.maxError == 2.0);
    CHECK(t2.maxErrorRate == doctest::Approx(4.0));  // 1.0 × 2 / 0.5
    CHECK(t2.settleTime == tr.settleTime);
    const double fiveDeg = 5.0 * kPi / 180.0;
    const SettleConfig h5 = loosenedSettle(hd, fiveDeg);
    CHECK(h5.maxError == fiveDeg);
    CHECK(h5.maxErrorRate == doctest::Approx(0.30 * fiveDeg / 0.02));  // ≈ 1.31 rad/s, ~75°/s
    CHECK(h5.maxErrorRate == doctest::Approx(1.309).epsilon(0.001));
    // The EQUALITY edge: a tolerance equal to the robot's own changes nothing (t > e is strict).
    const SettleConfig same = loosenedSettle(tr, 0.5);
    CHECK(same.maxError == 0.5);
    CHECK(same.maxErrorRate == tr.maxErrorRate);
    // 0 and below: the robot's own config, whole.
    const SettleConfig zero = loosenedSettle(tr, 0.0);
    CHECK(zero.maxError == tr.maxError);
    CHECK(zero.maxErrorRate == tr.maxErrorRate);
    CHECK(zero.settleTime == tr.settleTime);
}

TEST_CASE("STEP1-A ST23: a robot whose own settle distance is 0 -- the rate stays its own, never inf or NaN") {
    // Bug caught: r·t/e unguarded (SM36): e = 0, r = 1 gives inf (the speed bound silently gone), and
    // e = 0, r = 0 gives NaN, which SettledUtil refuses — on EVERY leg with a tolerance.
    const SettleConfig e0r1{.maxError = 0.0, .maxErrorRate = 1.0, .settleTime = 0.1};
    const SettleConfig e0r0{.maxError = 0.0, .maxErrorRate = 0.0, .settleTime = 0.1};
    const SettleConfig a = loosenedSettle(e0r1, 2.0);
    const SettleConfig b = loosenedSettle(e0r0, 2.0);
    CHECK(a.maxError == 2.0);
    CHECK(a.maxErrorRate == 1.0);
    CHECK(std::isfinite(a.maxErrorRate));
    CHECK(b.maxError == 2.0);
    CHECK(b.maxErrorRate == 0.0);
    CHECK_FALSE(std::isnan(b.maxErrorRate));
    // ... and a motion built from such a robot's config, with a tolerance, does not throw.
    Tank t{0.0};
    shulib::motion::MotionConfig mc = motion_rig::motionConfig();
    mc.translationSettle = a;
    CHECK_NOTHROW(shulib::motion::MoveToPose(t.ch.deps(), kForty, mc, 5.0));
    mc.translationSettle = b;
    CHECK_NOTHROW(shulib::motion::MoveToPose(t.ch.deps(), kForty, mc, 5.0));
    // And through the facade: a chassis whose robot settle distance is 0 takes a 2 in tolerance.
    Tank u{0.0};
    shulib::chassis::ChassisConfig cc{.motion = motion_rig::motionConfig()};
    cc.motion.translationSettle = e0r0;
    Chassis ch{u.rig.deps, u.world, cc};
    CHECK_NOTHROW((void)ch.moveTo(Pose2d{Length{6.0}, Length{0.0}, Angle{}},
                                  {.timeout = 3_s, .positionTolerance = 2_in}));
}

TEST_CASE("STEP1-A ST23: through the facade -- a TIGHT 0.5 deg turn still settles, and brake() ignores both "
          "tolerances") {
    // Bug caught: SM17 seen through the facade (a 0.5° turn with a proportionally lowered rate bound
    // could time out), and a tolerance that leaks into brakeSettle (brake() must be the robot's own).
    Tank t{0.0};
    CHECK(t.ch.turnTo(Angle::degrees(90.0), {.timeout = 4_s, .headingTolerance = 0.5_deg}) == ExitReason::Settled);
    CHECK(std::abs(t.rig.h.truePose().heading().errorTo(Angle::degrees(90.0))) < 0.5 * kPi / 180.0);
    // brake(): two identical robots, one leg each, then brake() with and without the tolerances —
    // the same exit at the same tick and the same trace.
    Tank a{0.0};
    Tank b{0.0};
    (void)a.ch.moveTo(Pose2d{Length{20.0}, Length{0.0}, Angle{}}, {.timeout = 0.6_s});  // still moving
    (void)b.ch.moveTo(Pose2d{Length{20.0}, Length{0.0}, Angle{}}, {.timeout = 0.6_s});
    const ExitReason ba = a.ch.brake();
    const ExitReason bb = b.ch.brake({.positionTolerance = 4_in, .headingTolerance = 10_deg});
    CHECK(ba == bb);
    CHECK(a.ch.lastCompleted().endTime.value() == b.ch.lastCompleted().endTime.value());
    CHECK(a.world.trace == b.world.trace);
}

TEST_CASE("STEP1-A: MotionOptions refuses a negative, non-finite or WRAPPED-negative tolerance at the call") {
    // Bug caught: a tolerance validated nowhere (a NaN would reach SettledUtil as a NaN bound), or a
    // 359_deg — which an Angle stores as −1° — read as a size.
    Tank t{0.0};
    const double start = t.now();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    CHECK_THROWS_AS((void)t.ch.moveTo(kForty, {.positionTolerance = Length{-1.0}}), shulib::PreconditionError);
    CHECK_THROWS_AS((void)t.ch.moveTo(kForty, {.positionTolerance = Length{nan}}), shulib::PreconditionError);
    CHECK_THROWS_AS((void)t.ch.moveTo(kForty, {.positionTolerance = Length{inf}}), shulib::PreconditionError);
    CHECK_THROWS_AS((void)t.ch.turnTo(Angle::degrees(90.0), {.headingTolerance = -5_deg}), shulib::PreconditionError);
    CHECK_THROWS_AS((void)t.ch.turnTo(Angle::degrees(90.0), {.headingTolerance = 359_deg}), shulib::PreconditionError);
    CHECK(t.now() == start);  // nothing ran: every refusal came before the motion
    // 180° stores +π: a legal (if useless) size.
    CHECK_NOTHROW(MotionOptions{.headingTolerance = 180_deg}.validate());
}
