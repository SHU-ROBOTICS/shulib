// hal::MotorGroup through the A2 plant's COUPLED members (chunk R3b Part 1). The plant
// writes one wheel's synthesized shaft into every member of that wheel, so a group over
// them is exercised exactly as robot two's five-per-side gear train will exercise it.
//   2  an N = 1 group is BIT-IDENTICAL to the bare motor through a full plant routine
//                                                     (mutation: any arithmetic in the read path)
//   3  one FROZEN member through a routine: the group's numbers are bit-identical to the
//      healthy run; the member is counted; MOTOR_GROUP_DISAGREE is raised   (median → mean)
//   4  a FIGHTING member (sign-flipped via the plant): counted; HealthMonitor raises
//      MotorGroupDisagree; the scheduler CONTINUES degraded — the motion settles
//                                                     (detector disabled; fault not routed)
//   7  5-member groups per side on tank, ideal coupling: bit-identical to the 1-motor
//      baseline across the clean AND the hostile sweeps               (— the equivalence pin)
// Bit-identity here means the per-tick TRUTH samples (memcmp), the commanded volts, the
// exit reason and the tick count all match — never Approx.

#include "doctest.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <span>
#include <vector>

#include "motion_test_rig.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/motor_group.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/sim/hostile/composed.hpp"
#include "shulib/sim/rng.hpp"
#include "shulib/sim/scenario.hpp"

using namespace motion_rig;
using shulib::control::ExitReason;
using shulib::diag::FaultCode;
using shulib::hal::IMotor;
using shulib::hal::MotorGroup;
using shulib::hal::fake::FakeMotor;
using shulib::kinematics::TankKinematics;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::motion::MoveToPose;
using shulib::sim::MemberFault;
using shulib::sim::Rng;
using shulib::sim::SimHarness;
using shulib::sim::TruthSample;
using shulib::units::Time;

namespace {

/// The extra fakes of a two-wheel drive and the two groups over them (primary + extras
/// per wheel). Declared before the group so the pointer arrays exist when the groups
/// take their spans; the plant is told about the extras at the same time.
struct CoupledMembers {
    static constexpr std::size_t kMaxExtra = 20;
    std::array<FakeMotor, 2 * kMaxExtra> extra{};
    std::array<IMotor*, kMaxExtra + 1> leftPtrs{};
    std::array<IMotor*, kMaxExtra + 1> rightPtrs{};
    std::size_t perWheel;
    MotorGroup left;
    MotorGroup right;

    CoupledMembers(SimHarness& h, std::size_t membersPerWheel)
        : perWheel{membersPerWheel}, left{wire(h, 0, leftPtrs)}, right{wire(h, 1, rightPtrs)} {}

private:
    std::span<IMotor* const> wire(SimHarness& h, int wheel,
                                  std::array<IMotor*, kMaxExtra + 1>& ptrs) {
        ptrs[0] = &h.motor(wheel);
        std::array<FakeMotor*, kMaxExtra> ex{};
        for (std::size_t i = 1; i < perWheel; ++i) {
            FakeMotor& f = extra[static_cast<std::size_t>(wheel) * kMaxExtra + (i - 1)];
            ptrs[i] = &f;
            ex[i - 1] = &f;
        }
        h.plant().attachCoupledMembers(
            wheel, std::span<FakeMotor* const>{ex.data(), perWheel - 1});
        return std::span<IMotor* const>{ptrs.data(), perWheel};
    }
};

/// MotionRig's twin with the drive motors REPLACED by two groups (the same harness, the
/// same odometry, the same fusion) — the composition the tank robot uses.
struct GroupRig {
    SimHarness h;
    CoupledMembers m;
    std::array<IMotor*, 2> drive;
    shulib::chassis::RobotContext ctx;
    shulib::localization::PilonsOdometry odom;
    shulib::localization::ComplementaryFusion fusion;
    shulib::localization::Localizer loc;
    shulib::hal::fake::FakeTelemetrySink faultSink;
    shulib::diag::FaultLatch latch;
    shulib::diag::HealthMonitor health;
    std::array<MotorGroup*, 2> groups;
    shulib::motion::MotionDeps deps;

    GroupRig(const shulib::kinematics::IKinematics& kin, std::size_t membersPerWheel,
             const shulib::sim::SimHarnessConfig& cfg = plantConfig(),
             shulib::sim::DegradationModel* degradation = nullptr)
        : h{kin, cfg, nullptr, degradation},
          m{h, membersPerWheel},
          drive{&m.left, &m.right},
          ctx{{.clock = &h.clock(),
               .driveMotors = drive,
               .imu = &h.imu(),
               .gps = &h.gps(),
               .battery = &h.battery(),
               .telemetry = &h.context().telemetry(),
               .tags = &h.context().tags(),
               .vision = &h.context().vision()}},
          odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()},
          fusion{},
          loc{h.clock(), h.imu(), odom, fusion},
          latch{faultSink, h.clock()},
          health{latch},
          groups{&m.left, &m.right},
          deps{.ctx = &ctx,
               .localizer = &loc,
               .kinematics = &kin,
               .faults = &latch,
               .health = &health,
               .motorGroups = groups} {
        loc.setPose(cfg.plant.initialPose);
    }
};

/// Everything a trial produces that must match bit for bit — plus the disagreement
/// observable's PEAK over the run (it is a live state: once the motion settles and the
/// group is no longer commanded, the monitor correctly re-arms and reads 0 again, so the
/// end-of-run value proves nothing; the peak and the latch do).
struct Trace {
    std::vector<TruthSample> truth;
    std::vector<double> volts;  // both primaries, per tick
    std::vector<double> groupPos;   // wheel 0's reported position per tick (group or bare)
    ExitReason exit = ExitReason::Running;
    int ticks = 0;
    int peakDisagreeing[2] = {0, 0};        // per group, max disagreeingMembers() seen
    std::uint32_t peakMask[2] = {0, 0};     // per group, OR of every persisted mask seen
};

/// Sample the two groups' observables after a tick (a no-op for a rig without groups).
template <typename RigT>
void sampleGroups(RigT& rig, Trace& t) {
    for (std::size_t g = 0; g < rig.deps.motorGroups.size(); ++g) {
        const MotorGroup& grp = *rig.deps.motorGroups[g];
        t.peakDisagreeing[g] = std::max(t.peakDisagreeing[g], grp.disagreeingMembers());
        t.peakMask[g] |= grp.disagreeingMask();
    }
}

template <typename RigT>
Trace runTrace(RigT& rig, const Pose2d& target, int maxTicks = 1200) {
    Trace t;
    MoveToPose m{rig.deps, target, motionConfig(), 8.0};
    m.start();
    auto reason = ExitReason::Running;
    for (int i = 0; i < maxTicks && reason == ExitReason::Running; ++i) {
        rig.loc.update();
        reason = m.tick();
        ++t.ticks;
        t.volts.push_back(rig.h.motor(0).commandedVoltage().value());
        t.volts.push_back(rig.h.motor(1).commandedVoltage().value());
        t.groupPos.push_back(rig.deps.ctx->driveMotors()[0]->position().value());
        sampleGroups(rig, t);
        if (reason == ExitReason::Running) {
            rig.h.plant().step(Time{0.01});
        }
        t.truth.push_back(rig.h.sample());
    }
    t.exit = reason;
    return t;
}

bool sameTruth(const std::vector<TruthSample>& a, const std::vector<TruthSample>& b) {
    if (a.size() != b.size()) {
        return false;
    }
    return a.empty()
           || std::memcmp(a.data(), b.data(), a.size() * sizeof(TruthSample)) == 0;
}

void requireIdentical(const Trace& base, const Trace& group) {
    REQUIRE(group.exit == base.exit);
    REQUIRE(group.ticks == base.ticks);
    REQUIRE(sameTruth(base.truth, group.truth));
    REQUIRE(base.volts.size() == group.volts.size());
    for (std::size_t i = 0; i < base.volts.size(); ++i) {
        REQUIRE(base.volts[i] == group.volts[i]);  // bits, not Approx
    }
    REQUIRE(base.groupPos.size() == group.groupPos.size());
    for (std::size_t i = 0; i < base.groupPos.size(); ++i) {
        REQUIRE(base.groupPos[i] == group.groupPos[i]);
    }
}

/// The tank sweep's along-axis start/target draw, exactly as motion_sweep_test.cpp draws it.
struct AlongAxis {
    Pose2d start;
    Pose2d target;
};
AlongAxis alongAxisTrial(std::uint64_t seed) {
    Rng rng{seed * 104729};
    const Pose2d start{Length{rng.uniform(-40.0, 40.0)}, Length{rng.uniform(-40.0, 40.0)},
                       Angle::radians(rng.uniform(-Angle::kPi, Angle::kPi))};
    const double d = rng.uniform(-40.0, 40.0);
    const double th = start.heading().radians();
    return AlongAxis{start, Pose2d{Length{start.x().value() + d * std::cos(th)},
                                   Length{start.y().value() + d * std::sin(th)},
                                   start.heading()}};
}

}  // namespace

// ═══ 2. N = 1 is the bare motor ═════════════════════════════════════════════════════
// Bug caught: the group changing NUMBERS — any arithmetic in the read path (a scale, an
// offset, a mean where a pass-through belongs) shows here as a bit difference in truth.
TEST_CASE("MotorGroup 2: an N = 1 group is bit-identical to the bare motor through a routine") {
    const TankKinematics kin{Length{12.0}};
    for (std::uint64_t seed = 1; seed <= 4; ++seed) {
        CAPTURE(seed);
        const AlongAxis trial = alongAxisTrial(seed);
        auto pcfg = plantConfig();
        pcfg.plant.initialPose = trial.start;
        MotionRig base{kin, pcfg};
        GroupRig grouped{kin, 1, pcfg};
        const Trace b = runTrace(base, trial.target);
        const Trace g = runTrace(grouped, trial.target);
        REQUIRE(b.exit == ExitReason::Settled);
        requireIdentical(b, g);
        CHECK(grouped.latch.raiseCount(FaultCode::MotorGroupDisagree) == 0);
    }
}

// ═══ 3. One frozen member, through a routine ════════════════════════════════════════
// Bug caught: the 1/N drift reaching odometry-grade numbers — the group's position and
// velocity must be BIT-IDENTICAL with and without the dead member, while the member is
// COUNTED and the fault raised (a dead port is reported, never hidden).
TEST_CASE("MotorGroup 3 (plant): a member frozen at rest leaves the group's numbers bit-identical, "
          "is counted, and raises MOTOR_GROUP_DISAGREE once") {
    const TankKinematics kin{Length{12.0}};
    auto pcfg = plantConfig();
    pcfg.plant.initialPose = Pose2d{};
    const Pose2d target{Length{30.0}, Length{0.0}, Angle{}};

    GroupRig healthy{kin, 5, pcfg};
    GroupRig frozen{kin, 5, pcfg};
    frozen.h.plant().setMemberFault(0, 2, MemberFault::Frozen);  // left, member 2, at rest

    const Trace a = runTrace(healthy, target);
    const Trace b = runTrace(frozen, target);
    REQUIRE(a.exit == ExitReason::Settled);
    requireIdentical(a, b);  // the group's position() per tick is in the trace

    // The frozen member really is frozen (0 rad, 0 rad/s the whole way) …
    CHECK(frozen.m.left.member(2).position().value() == 0.0);
    CHECK(frozen.m.left.member(2).velocity().value() == 0.0);
    CHECK(frozen.m.left.member(1).position().value() > 1.0);
    // … the group ignored it, COUNTED it while the side was driven (the peak over the run —
    // at rest after settling the monitor legitimately re-arms to 0), and the health tick
    // raised the fault ONCE for the whole episode.
    CHECK(b.peakDisagreeing[0] == 1);
    CHECK(b.peakMask[0] == (std::uint32_t{1} << 2));
    CHECK(b.peakDisagreeing[1] == 0);
    CHECK(b.peakMask[1] == 0);
    CHECK(a.peakDisagreeing[0] == 0);
    CHECK(a.peakDisagreeing[1] == 0);
    CHECK(frozen.latch.raiseCount(FaultCode::MotorGroupDisagree) == 1);
    CHECK(frozen.latch.firstFault() == FaultCode::MotorGroupDisagree);
    CHECK(healthy.latch.raiseCount(FaultCode::MotorGroupDisagree) == 0);
    // After the settle the side is uncommanded: the episode has cleared, by design.
    CHECK(frozen.m.left.disagreeingMembers() == 0);
    CHECK_FALSE(frozen.health.motorGroupDisagreeing());
}

// ═══ 4. A fighting member: counted, raised, and the run CONTINUES ═══════════════════
// Bug caught: an undetected coupled fight (the detector not evaluated in the health tick,
// or the observable not routed to a fault), and the wrong POLICY — a fight aborting the
// motion, when the drive still works with one bad member and the driver must be told.
TEST_CASE("MotorGroup 4: a sign-flipped member raises MOTOR_GROUP_DISAGREE; the scheduler "
          "continues degraded and the motion settles") {
    const TankKinematics kin{Length{12.0}};
    auto pcfg = plantConfig();
    GroupRig rig{kin, 5, pcfg};
    PlantPacer pacer{rig.h};
    shulib::motion::MotionScheduler sched{rig.deps, pacer};
    rig.h.plant().setMemberFault(1, 3, MemberFault::SignFlipped);  // right, member 3

    MoveToPose m{sched.deps(), Pose2d{Length{30.0}, Length{0.0}, Angle{}}, motionConfig(), 8.0};
    // Ticked by hand (not waitUntilSettled) so the observable can be sampled DURING the
    // motion — it is a live state that re-arms to 0 once the side is no longer commanded.
    sched.async(m);
    Trace peak;
    int guard = 0;
    while (sched.hasActiveMotion() && guard++ < 2000) {
        (void)sched.tick();
        sampleGroups(rig, peak);
        pacer.pace();
    }
    const ExitReason reason = sched.lastExitReason();
    CHECK(reason == ExitReason::Settled);                        // continued, degraded
    CHECK(sched.lastCompleted().abortFault == FaultCode::None);  // no abort
    CHECK(rig.latch.raiseCount(FaultCode::MotorGroupDisagree) == 1);  // ONE episode
    CHECK(rig.latch.firstFault() == FaultCode::MotorGroupDisagree);
    CHECK(peak.peakDisagreeing[1] == 1);                            // the RIGHT group …
    CHECK(peak.peakMask[1] == (std::uint32_t{1} << 3));             // … member 3, by bit
    CHECK(peak.peakDisagreeing[0] == 0);
    CHECK(peak.peakMask[0] == 0);
    // The fault line names the subsystem and the count.
    bool sawLine = false;
    for (int i = 0; i < rig.faultSink.size(); ++i) {
        const auto& e = rig.faultSink.at(i);
        if (e.subsystem == "MOT" && e.message.find("MOTOR_GROUP_DISAGREE") != std::string::npos
            && e.message.find("1 member") != std::string::npos) {
            sawLine = true;
        }
    }
    CHECK(sawLine);
    // The median ignored the fighter's negated reading: the group's position is its mates'.
    CHECK(rig.m.right.position().value() == rig.m.right.member(0).position().value());
    CHECK(rig.m.right.member(3).position().value() == -rig.m.right.member(0).position().value());

    // The policy is CONFIGURABLE: with MotorGroupDisagree in the abort mask, the same
    // fight aborts the motion into the safe state and names the cause.
    GroupRig rig2{kin, 5, pcfg};
    PlantPacer pacer2{rig2.h};
    shulib::motion::MotionSchedulerConfig abortOnFight;
    abortOnFight.abortFaultMask |= shulib::motion::faultBit(FaultCode::MotorGroupDisagree);
    shulib::motion::MotionScheduler sched2{rig2.deps, pacer2, abortOnFight};
    rig2.h.plant().setMemberFault(1, 3, MemberFault::SignFlipped);
    MoveToPose m2{sched2.deps(), Pose2d{Length{30.0}, Length{0.0}, Angle{}}, motionConfig(), 8.0};
    sched2.async(m2);
    CHECK(sched2.waitUntilSettled() == ExitReason::Cancelled);
    CHECK(sched2.lastCompleted().abortFault == FaultCode::MotorGroupDisagree);
}

// ═══ 7. The equivalence pin: five per side is one per side, bit for bit ═════════════
// Bug caught: the group PERTURBING motion in any way — through the clean tank sweep and
// through the full composed hostile world (sag, slip, IMU bias/noise, encoder
// quantization, latency), where a group that behaved differently under noise would show.
TEST_CASE("MotorGroup 7: 5-member groups per side are bit-identical to the 1-motor baseline — "
          "clean tank sweep") {
    const TankKinematics kin{Length{12.0}};
    for (std::uint64_t seed = 1; seed <= 12; ++seed) {
        CAPTURE(seed);
        const AlongAxis trial = alongAxisTrial(seed);
        auto pcfg = plantConfig();
        pcfg.plant.initialPose = trial.start;
        MotionRig base{kin, pcfg};
        GroupRig grouped{kin, 5, pcfg};
        const Trace b = runTrace(base, trial.target);
        const Trace g = runTrace(grouped, trial.target);
        REQUIRE(b.exit == ExitReason::Settled);
        requireIdentical(b, g);
        CHECK(grouped.latch.raiseCount(FaultCode::MotorGroupDisagree) == 0);
        CHECK(grouped.m.left.disagreeingMembers() == 0);
        CHECK(grouped.m.right.disagreeingMembers() == 0);
    }
}

TEST_CASE("MotorGroup 7: 5-member groups per side are bit-identical to the 1-motor baseline — "
          "hostile tank sweep (FullHostility, same seed)") {
    const TankKinematics kin{Length{12.0}};
    for (std::uint64_t seed = 1; seed <= 6; ++seed) {
        CAPTURE(seed);
        const AlongAxis trial = alongAxisTrial(seed);
        auto pcfg = plantConfig();
        pcfg.plant.initialPose = trial.start;
        pcfg.plant.seed = seed;
        shulib::sim::FullHostility worldA{};
        shulib::sim::FullHostility worldB{};
        MotionRig base{kin, pcfg, nullptr, &worldA.model()};
        GroupRig grouped{kin, 5, pcfg, &worldB.model()};
        const Trace b = runTrace(base, trial.target, 2000);
        const Trace g = runTrace(grouped, trial.target, 2000);
        REQUIRE(b.ticks > 50);  // a real run, not an instant exit
        requireIdentical(b, g);
        // Ideal coupling under hostility is still agreement: no fight was ever seen.
        CHECK(grouped.latch.raiseCount(FaultCode::MotorGroupDisagree) == 0);
    }
}
