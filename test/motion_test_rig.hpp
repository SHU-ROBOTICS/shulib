#pragma once
//
// motion_test_rig.hpp — the shared closed-loop rig for the C1 motion tests.
//
// One struct wires the full stack a motion needs: A2 harness (plant behind the
// F4 fakes) + PilonsOdometry + ComplementaryFusion + Localizer + A1 fault latch
// + A3 HealthMonitor + MotionDeps. Tests grade against h.truePose() (ground
// truth) — never against the estimate the motion itself read.
//
// The run() loop is the documented controller-first shape (scenario.hpp):
// Localizer updates FIRST (sees the world at t), the motion ticks, the plant
// then advances to t+dt. The loop stops ON the exit tick (no trailing step), so
// truth/estimate are exactly what the verdict was rendered on.
//
// PLANT/MOTION GAIN AGREEMENT: motionConfig().wheelFf matches plantConfig()'s
// wheel model on purpose — feedforward is only meaningful when it matches the
// plant, exactly as R5's sysid gains must match the real robot.

#include <cmath>

#include "shulib/control/exit_group.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/health_monitor.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/motion/motion.hpp"
#include "shulib/motion/motion_config.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/quantity.hpp"

namespace motion_rig {

using shulib::units::Length;
using shulib::units::Time;

/// The plant every motion test drives: kA = 0 (memoryless — hand-derivable),
/// kS/kV in the plant's placeholder ballpark.
inline shulib::sim::SimHarnessConfig plantConfig() {
    shulib::sim::SimHarnessConfig cfg;
    cfg.plant.wheelFf = {.kS = 1.2, .kV = 0.17, .kA = 0.0};
    return cfg;
}

/// Motion config whose feedforward MATCHES plantConfig() (header note).
inline shulib::motion::MotionConfig motionConfig() {
    shulib::motion::MotionConfig m;
    m.wheelFf = {.kS = 1.2, .kV = 0.17, .kA = 0.0};
    return m;
}

/// Euclidean position error between two poses (inches).
inline double posErr(const shulib::math::Pose2d& a, const shulib::math::Pose2d& b) {
    return std::hypot((a.x() - b.x()).value(), (a.y() - b.y()).value());
}

/// |shortest heading error| between two poses (radians).
inline double headErr(const shulib::math::Pose2d& a, const shulib::math::Pose2d& b) {
    return std::abs(a.heading().errorTo(b.heading()));
}

/// The full closed-loop rig (see file header). `kinematics`, `harnessSink` and
/// `degradation` must outlive the rig.
struct MotionRig {
    shulib::sim::SimHarness h;
    shulib::localization::PilonsOdometry odom;
    shulib::localization::ComplementaryFusion fusion;
    shulib::localization::Localizer loc;
    shulib::hal::fake::FakeTelemetrySink faultSink;  // FaultLatch's log channel
    shulib::diag::FaultLatch latch;
    shulib::diag::HealthMonitor health;
    shulib::motion::MotionDeps deps;

    explicit MotionRig(const shulib::kinematics::IKinematics& kinematics,
                       const shulib::sim::SimHarnessConfig& cfg = plantConfig(),
                       shulib::hal::ITelemetrySink* harnessSink = nullptr,
                       shulib::sim::DegradationModel* degradation = nullptr)
        : h{kinematics, cfg, harnessSink, degradation},
          odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()},
          fusion{},
          loc{h.clock(), h.imu(), odom, fusion},
          latch{faultSink, h.clock()},
          health{latch},
          deps{.ctx = &h.context(),
               .localizer = &loc,
               .kinematics = &kinematics,
               .faults = &latch,
               .health = &health} {
        // Seed the ESTIMATE's position to the plant's initial truth (heading is
        // IMU-owned and already synthesized from truth). Without this a
        // non-origin start would begin with a fabricated position error.
        loc.setPose(cfg.plant.initialPose);
    }

    /// Run `m` to completion: start() + the controller-first loop, stopping ON
    /// the exit tick. Returns the exit reason (Running ⇒ maxTicks exhausted —
    /// itself a termination failure the caller asserts against).
    shulib::control::ExitReason run(shulib::motion::IMotion& m, int maxTicks = 1500,
                                    Time dt = Time{0.01}) {
        m.start();
        return resume(m, maxTicks, dt);
    }

    /// The loop body without start() — for tests that drive start() manually
    /// (restart semantics, mid-run retarget-by-new-motion).
    shulib::control::ExitReason resume(shulib::motion::IMotion& m, int maxTicks = 1500,
                                       Time dt = Time{0.01}) {
        auto reason = shulib::control::ExitReason::Running;
        for (int i = 0; i < maxTicks && reason == shulib::control::ExitReason::Running; ++i) {
            loc.update();
            reason = m.tick();
            if (reason == shulib::control::ExitReason::Running) {
                h.plant().step(dt);
            }
        }
        return reason;
    }
};

}  // namespace motion_rig
