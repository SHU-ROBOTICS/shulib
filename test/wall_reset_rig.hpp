#pragma once
//
// wall_reset_rig.hpp — the wall reset's closed-loop rig (chunk L6-A1): a TANK ChassisRig (the C4
// stack: harness, PilonsOdometry, ComplementaryFusion, Localizer, scheduler, Chassis) on the host
// field plant, with one side sensor per side stepped by the PACER after the drive plant — so the
// sensors read the TRUE pose of the instant the scheduler's next tick sees (the C2 loop shape).
//
// The sensors' TRUE mounts live in the plants (sim::SideSensorPlant); the mounts the reset BELIEVES
// are the WallSensor handles' (`leftHandle()` / `rightHandle()`), built from what a robot file would
// type — every adversarial case sets the two apart on purpose. Grade against `h().truePose()`.
//
// Every pace is capped (motion_rig::PlantPacer's rule): a wait that never ends throws instead of
// hanging the suite, and the memory rule caps the binary anyway.

#include <cmath>
#include <stdexcept>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/wall_reset_call.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/localization/override_walls.hpp"
#include "shulib/localization/wall_reset.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/sim/field_plant.hpp"
#include "shulib/units/quantity.hpp"

namespace wall_rig {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kD2R = kPi / 180.0;
/// The plant's inner wall face (its own number — the oracle's).
inline constexpr double kW = shulib::sim::FieldModel::kInnerFace;

/// The pacer: the drive plant one tick, then both side sensors at the new TRUE pose.
struct WallPacer final : shulib::motion::ITickPacer {
    WallPacer(shulib::sim::SimHarness& harness, shulib::sim::SideSensorPlant& l,
              shulib::sim::SideSensorPlant& r)
        : h{&harness}, left{&l}, right{&r} {}

    void pace() override {
        if (paces >= cap) {
            throw std::runtime_error("WallPacer: pace cap exceeded — a wait failed to end");
        }
        ++paces;
        if (paces == faultLeftAt) {
            left->fault(true);  // a read fault from this pace on (the S6 scenario)
        }
        h->plant().step(shulib::units::Time{0.01});
        const shulib::math::Pose2d t = h->truePose();
        left->step(t);
        right->step(t);
    }

    shulib::sim::SimHarness* h;
    shulib::sim::SideSensorPlant* left;
    shulib::sim::SideSensorPlant* right;
    int paces = 0;
    int cap = 20000;
    int faultLeftAt = -1;  ///< the pace at which the LEFT sensor starts faulting (-1 = never)
};

/// The plant's default TRUE side mounts: a 15-in robot's sensors 7.5 in out, at the fore-aft
/// centre, 10.5 in up (D10's advice), facing straight out.
inline shulib::sim::SideSensorPlantConfig trueLeft() {
    shulib::sim::SideSensorPlantConfig c{};
    c.forward = 0.0;
    c.left = 7.5;
    c.height = 10.5;
    c.facing = kPi / 2.0;
    c.noiseSigma = 0.05;
    c.seed = 11;
    return c;
}
inline shulib::sim::SideSensorPlantConfig trueRight() {
    shulib::sim::SideSensorPlantConfig c = trueLeft();
    c.left = -7.5;
    c.facing = -kPi / 2.0;
    c.seed = 12;
    return c;
}

/// The mount a robot file would type for the plant's TRUE mount `c` (measured, the default
/// tolerances, nothing else measured).
inline shulib::localization::SideMount believedFrom(const shulib::sim::SideSensorPlantConfig& c) {
    shulib::localization::SideMount m{};
    m.measured = true;
    m.forward = shulib::units::Length{c.forward};
    m.left = shulib::units::Length{c.left};
    m.height = shulib::units::Length{c.height};
    m.facing = shulib::units::AngleDim{c.facing};
    m.provenance = "host test plant";
    return m;
}

/// The robot centre's x that puts a LEFT sensor `d` in from the LEFT wall, nose along +y.
inline double xForLeftReading(double d, double left = 7.5) { return -kW + left + d; }

/// The rig.
struct WallRig {
    shulib::kinematics::TankKinematics kin{shulib::units::Length{11.0}};
    shulib::hal::fake::FakeTelemetrySink sink;
    shulib::sim::FieldModel field;
    shulib::sim::SideSensorPlant leftPlant;
    shulib::sim::SideSensorPlant rightPlant;
    motion_rig::MotionRig rig;
    WallPacer pacer;
    shulib::chassis::Chassis chassis;
    shulib::localization::SideMount leftMount;
    shulib::localization::SideMount rightMount;
    shulib::localization::WallResetLimits limits{};

    /// A tank at TRUE pose `start` (the estimate seeded to it — a FIELD start), on `fieldModel`;
    /// `degradation` (may be null; must outlive the rig) makes the harness's sensors hostile — e.g.
    /// sim::ImuHostileModel for an IMU that calibrates, drifts or jitters.
    explicit WallRig(const shulib::math::Pose2d& start,
                     const shulib::sim::FieldModel& fieldModel = shulib::sim::FieldModel::override2026(),
                     const shulib::sim::SideSensorPlantConfig& l = trueLeft(),
                     const shulib::sim::SideSensorPlantConfig& r = trueRight(),
                     shulib::sim::DegradationModel* degradation = nullptr)
        : field{fieldModel},
          leftPlant{field, l},
          rightPlant{field, r},
          rig{kin, configAt(start), &sink, degradation},
          pacer{rig.h, leftPlant, rightPlant},
          chassis{rig.deps, pacer, motion_rig::chassisConfig()},
          leftMount{believedFrom(l)},
          rightMount{believedFrom(r)} {
        leftPlant.step(rig.h.truePose());
        rightPlant.step(rig.h.truePose());
    }

    static shulib::sim::SimHarnessConfig configAt(const shulib::math::Pose2d& start) {
        shulib::sim::SimHarnessConfig cfg = motion_rig::plantConfig();
        cfg.plant.initialPose = start;
        return cfg;
    }

    shulib::sim::SimHarness& h() noexcept { return rig.h; }
    shulib::localization::Localizer& loc() noexcept { return rig.loc; }

    /// Drive a leg to the ESTIMATE's (x, y) facing `headingDeg`, brake, and let it settle.
    void leg(double x, double y, double headingDeg) {
        (void)chassis.moveTo(shulib::math::Pose2d{shulib::units::Length{x}, shulib::units::Length{y},
                                                  shulib::math::Angle::degrees(headingDeg)},
                             {.timeout = shulib::units::Time{8.0}});
        (void)chassis.brake();
        idle(0.3);
    }

    /// Let `seconds` pass with nothing commanded (the estimate goes live; the sensors fill).
    void idle(double seconds) {
        (void)chassis.waitUntil([] { return false; }, shulib::units::Time{seconds});
    }

    /// The LEFT side's handle, with the BELIEVED mount and the plant's read-fault count.
    shulib::localization::WallSensor leftHandle() {
        return shulib::localization::WallSensor{&leftPlant.sensor(), &leftPlant.faults(), leftMount,
                                                "not declared"};
    }
    /// The RIGHT side's handle.
    shulib::localization::WallSensor rightHandle() {
        return shulib::localization::WallSensor{&rightPlant.sensor(), &rightPlant.faults(), rightMount,
                                                "not declared"};
    }

    /// The explicit call against `wall` with the named side and `maxCorrection`.
    shulib::localization::WallResetReport reset(const shulib::localization::FieldWall& wall,
                                                shulib::localization::WallSide side, double maxCorrection,
                                                bool required = false) {
        shulib::localization::WallResetRequest req{};
        req.wall = wall;
        req.sensor = side;
        req.maxCorrection = shulib::units::Length{maxCorrection};
        req.required = required;
        const shulib::localization::WallSensor s =
            side == shulib::localization::WallSide::Left ? leftHandle() : rightHandle();
        return shulib::chassis::resetFromWall(chassis, s, req, limits);
    }

    /// How many log lines a subsystem has written.
    int linesFrom(const char* subsystem) const {
        int n = 0;
        for (int i = 0; i < sink.size(); ++i) {
            if (sink.at(i).subsystem == subsystem) {
                ++n;
            }
        }
        return n;
    }
};

}  // namespace wall_rig
