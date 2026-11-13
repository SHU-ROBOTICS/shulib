#pragma once
//
// toggle_visit_rig.hpp — the host rig for the Toggle step's tests (chunk AF1S-A1, 2026-11-11). Say "host
// test plant", never "simulator": every plant number here is the TEST's (invented), and nothing has run
// on a robot.
//
// One robot, the program's own pacer chain —
//
//     Chassis -> RunGuard -> MechanismService -> World (the plants step, the clock advances)
//
// — on the A2 tank plant (motion_rig::MotionRig, a 12-in tank), parked SQUARE in front of the RIGHT
// wall's Toggle (x = +70.2, a RED Quadrant's — HA-243), heading 0. Its front camera is VIS1-A2's rig's
// (camrig: the believed mount 7 in forward; the TRUE camera moved inside the stated tolerances), read
// through a hal::IVisionFrames adapter over the host camera model at the drive plant's TRUE pose, with
// an optional image AGE (the adapter returns the picture rendered `lagSeconds` ago, stamped now —
// HA-192's "the image is older than its read"). Three flippers are bound, one of each form the step
// drives (the brief's D7): an ARM ("flip arm", Green 5:1, stow 0 / flip 90 degrees) on sim::ArmPlant, a
// LIFT ("flip lift", a 1.5-in spool, stow 0 / flip 5 in) on its own ArmPlant, and a PNEUMATIC ("flip
// piston", stow Low / flip High, 0.3 s) whose cylinder the world extends over 0.25 s. The host Toggle
// (sim::TogglePlant) reads the flipper the test picks (`form`) — its TRUE angle — and the robot's front
// (the lens, 7 in forward of the tracking centre) from the drive plant's truth, and is redrawn into the
// camera model every tick with any extra quads a test adds (a second Toggle, a lookalike).
//
// Test-only (no TEST_CASE here). Its numbers are the probes' and the test's — the team's come from Step 0.

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "camera_scene_rig.hpp"
#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/toggle_visit.hpp"
#include "shulib/hal/fake/fake_digital_out.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/vision_frames.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/mechanism/devices.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/mechanism/wiring.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/sim/camera_plant.hpp"
#include "shulib/sim/mechanism_plant.hpp"
#include "shulib/sim/toggle_plant.hpp"
#include "shulib/vision/camera_feed.hpp"

namespace tv_fixture {
using namespace shulib::mechanism::words;
// The three flippers (the brief's D7) — a robot file's words, each form once.
inline constexpr Arm flipArm{
    .name = "flip arm",
    .motors = {Forward},
    .cartridge = Green,
    .gearing = gears(12, 60),
    .safe = Hold,
    .stall = {.amps = 1.0, .belowMotorRpm = 5, .lasting = 0.15_s},
    .range = {0, 120},
    .gravity = noGravity,
    .home = restsAtStop(0),
    .presets = {{"stow", 0}, {"flip", 90}, {"half", 45}},
};
inline constexpr Lift flipLift{
    .name = "flip lift",
    .motors = {Forward},
    .cartridge = Green,
    .gearing = direct,
    .shape = spool(1.5_in),
    .safe = Hold,
    .stall = {.amps = 1.0, .belowMotorRpm = 5, .lasting = 0.15_s},
    .range = {0_in, 6_in},
    .gravity = noGravity,
    .home = restsAtStop(0_in),
    .presets = {{"stow", 0_in}, {"flip", 5_in}},
};
inline constexpr Pneumatic flipPiston{
    .name = "flip piston",
    .positions = {{"stow", {Low}}, {"flip", {High}}},
    .start = "stow",
    .safe = "stow",
    .moveTime = 0.3_s,
};
inline constexpr MechanismTable kFlippers =
    mechanisms(wire(flipArm, motors(1)), wire(flipLift, motors(2)), wire(flipPiston, lines('A')));
}  // namespace tv_fixture

namespace tv_rig {

namespace sc = shulib::competition;
namespace sim = shulib::sim;
namespace vision = shulib::vision;
namespace fake = shulib::hal::fake;
namespace mech = shulib::mechanism;
namespace ov = shulib::vision::override2026;
using shulib::units::Length;
using shulib::units::Time;

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kWallX = 70.2;
inline constexpr double kLensFwd = camrig::kLensFwd;  // 7 in: the lens is the robot's front
inline constexpr std::uint8_t kRed = 1;
inline constexpr std::uint8_t kBlue = 2;
inline constexpr std::uint8_t kYellow = 3;

/// Which flipper the Toggle reads.
enum class Form : std::uint8_t { Arm, Lift, Pneumatic, Passive };

/// The ARM flipper's plant: Green 5:1 (40 rpm at the joint), friction 0.3 V, no gravity, stops at -2 and 125.
inline sim::ArmPlantConfig flipArmPlant() {
    sim::ArmPlantConfig c;
    c.gearRatio = 5.0;
    c.kS = 0.3;
    c.kV = 12.0 / (200.0 * 2.0 * kPi / 60.0 / 5.0);
    c.kA = 0.03 * c.kV;
    c.kG = 0.0;
    c.minStopDeg = -2.0;
    c.maxStopDeg = 125.0;
    return c;
}
/// The LIFT flipper's plant: Green direct on a 1.5-in spool, stops at 0 and 6.2 in (spool degrees).
inline sim::ArmPlantConfig flipLiftPlant() {
    sim::ArmPlantConfig c;
    c.gearRatio = 1.0;
    c.kS = 0.3;
    c.kV = 12.0 / (200.0 * 2.0 * kPi / 60.0);
    c.kA = 0.03 * c.kV;
    c.kG = 0.0;
    c.constantGravity = true;
    c.minStopDeg = -1.0;
    c.maxStopDeg = 6.2 / (kPi * 1.5) * 360.0;
    return c;
}

/// The camera, as the host camera model sees it from the drive plant's TRUE pose, with an image AGE.
struct PlantCamera final : shulib::hal::IVisionFrames {
    sim::CameraPlant* cam = nullptr;
    motion_rig::MotionRig* rig = nullptr;
    double lagSeconds = 0.0;  ///< the picture is this old at its read
    struct Kept {
        double t = 0.0;
        shulib::hal::VisionFrame f{};
    };
    static constexpr int kKeep = 96;
    Kept kept[kKeep]{};
    int count = 0;
    int head = 0;
    shulib::hal::VisionFrame read() noexcept override {
        const shulib::math::Pose2d p = rig->h.plant().truePose();
        const double t = rig->h.clock().now().value();
        shulib::hal::VisionFrame f = cam->read(p.x().value(), p.y().value(), p.heading().radians(), Time{t});
        if (!(lagSeconds > 0.0)) {
            return f;
        }
        kept[head] = Kept{t, f};
        head = (head + 1) % kKeep;
        count = count < kKeep ? count + 1 : kKeep;
        // the newest picture at least lagSeconds old (the oldest kept, before that many exist)
        shulib::hal::VisionFrame out = kept[(head - count + kKeep) % kKeep].f;
        for (int i = 0; i < count; ++i) {
            const Kept& k = kept[(head - 1 - i + 2 * kKeep) % kKeep];
            if (k.t <= t - lagSeconds + 1e-9) {
                out = k.f;
                break;
            }
        }
        out.readAt = Time{t};
        return out;
    }
    shulib::hal::CameraStatus status() const noexcept override {
        shulib::hal::CameraStatus s{};
        s.configured = true;
        return s;
    }
};

/// The robot's front's (the lens's, 7 in forward) TRUE distance from a wall's inner face through (wx, wy)
/// with the field on the side (inX, inY), along that normal — infinite when the front is beside the
/// Toggle's length, not in front of it.
inline double frontToWall(const shulib::math::Pose2d& p, double wx, double wy, double inX, double inY,
                          double halfLength = 12.9) {
    const double th = p.heading().radians();
    const double fx = p.x().value() + std::cos(th) * kLensFwd;
    const double fy = p.y().value() + std::sin(th) * kLensFwd;
    const double along = -(fx - wx) * inY + (fy - wy) * inX;
    if (std::fabs(along) > halfLength) {
        return 1e9;
    }
    return (fx - wx) * inX + (fy - wy) * inY;
}

/// The world below the service (header).
struct World final : shulib::motion::ITickPacer {
    struct Rig* r = nullptr;
    long paces = 0;
    std::function<void()> onPace;  ///< runs FIRST each pace (a test's script)
    void pace() override;
};

/// Options for a rig.
struct Options {
    ov::ToggleId wall = ov::toggles::kRightWall;  ///< which wall's Toggle the robot is parked in front of
    double lookFrom = 16.0;      ///< where the robot is PLACED: its lens this far from the face (TRUE)
    double headingDeg = 0.0;     ///< its true heading OFF square to that wall (0 = square)
    double lateral = 0.0;        ///< its true offset along the wall (left of the Toggle's centre, as seen)
    const mech::MechanismTable* table = nullptr;  ///< the flippers' table (nullptr: tv_fixture::kFlippers)
    double slowArm = 0.0;        ///< > 0: the arm's plant this many times slower (a slow flipper)
    bool drawToggle = true;      ///< false: the camera fires on none of the Toggle's faces (HA-199's miss)
    double lensHeight = 10.0;    ///< the camera's (believed) height
    double lensPitch = 0.0;      ///< the camera's (believed) pitch
    std::uint32_t seed = 7;      ///< the camera's jitter seed
    double jitterPx = 1.0;       ///< the camera's box jitter
    Form form = Form::Arm;       ///< which flipper the Toggle reads
    sim::TogglePlantConfig toggle = {};  ///< the host Toggle (its wall is set from `wall`)
    std::vector<sim::TogglePlantConfig> others;  ///< more host Toggles (each on its own wall)
    /// The chassis's motion config (nullptr: motion_rig::motionConfig()) — a test that widens a settle.
    const shulib::motion::MotionConfig* motion = nullptr;
};

/// Where the rig parks the robot: lens `lookFrom` out from `wall`'s Toggle along its normal, `lateral`
/// along it, heading toward the wall plus `headingDeg`.
inline shulib::math::Pose2d parkedAt(ov::ToggleId wall, double lookFrom, double headingDeg = 0.0,
                                     double lateral = 0.0) {
    const ov::TogglePlace& w = ov::kTogglePlaces[ov::toggleIndex(wall)];
    const double toward = std::atan2(-w.inY, -w.inX);
    const double th = toward + headingDeg * kPi / 180.0;
    const double lx = w.x + w.inX * lookFrom - w.inY * lateral;
    const double ly = w.y + w.inY * lookFrom + w.inX * lateral;
    return shulib::math::Pose2d{Length{lx - std::cos(th) * kLensFwd}, Length{ly - std::sin(th) * kLensFwd},
                                shulib::math::Angle::radians(th)};
}

/// `t` placed on `wall`'s Toggle.
inline sim::TogglePlantConfig onWall(sim::TogglePlantConfig t, ov::ToggleId wall) {
    const ov::TogglePlace& w = ov::kTogglePlaces[ov::toggleIndex(wall)];
    t.wallX = w.x;
    t.wallY = w.y;
    t.inX = w.inX;
    t.inY = w.inY;
    return t;
}

/// The host Toggle's default: the right wall, faces Y (field) / B (bottom) / R (outer) — the research's
/// red-side order, so two pushes from yellow (HA-231).
inline sim::TogglePlantConfig redSideToggle() {
    sim::TogglePlantConfig t{};
    t.fieldId = kYellow;
    t.bottomId = kBlue;
    t.outerId = kRed;
    t.yellowId = kYellow;
    t.setBack = 1.2;
    return t;
}

/// The robot (header). Members in LIFETIME order: devices and plants before the service, the service
/// before the guard and the chassis.
struct Rig {
    Options opt;
    fake::FakeTelemetrySink sink;
    shulib::kinematics::TankKinematics kin{Length{12.0}};
    motion_rig::MotionRig rig;
    // the flippers
    fake::FakeMotor armM, liftM;
    shulib::hal::IMotor* am[1]{&armM};
    shulib::hal::IMotor* lm[1]{&liftM};
    shulib::hal::MotorMechanism mArm{am, shulib::hal::BrakeMode::Hold, "flip arm"};
    shulib::hal::MotorMechanism mLift{lm, shulib::hal::BrakeMode::Hold, "flip lift"};
    fake::FakeDigitalOut lineA;
    shulib::hal::IDigitalOut* pl[1]{&lineA};
    shulib::hal::PneumaticMechanism mPiston{pl, false, "flip piston"};
    fake::FakeMotor* armF[1]{&armM};
    fake::FakeMotor* liftF[1]{&liftM};
    sim::ArmPlant armPlant;
    sim::ArmPlant liftPlant{liftF, flipLiftPlant()};
    double pistonOut = 0.0;  ///< the cylinder's extension, 0..1
    double pistonMin = 0.0;  ///< a cylinder caught on the way back stops here (0..1)
    double pistonMax = 1.0;  ///< a cylinder blocked on the way out stops here (0..1)
    // the Toggle and the camera
    sim::TogglePlant toggle;
    std::vector<std::unique_ptr<sim::TogglePlant>> others;
    std::vector<sim::ColourQuad> extras;  ///< other things in the camera's scene
    sim::CameraPlant camPlant;
    PlantCamera camera;
    vision::CameraFeed feed;
    vision::CameraModel model;
    sc::ExplicitToggleCamera cam;
    // the stack
    World world;
    mech::MechanismService svc{world};
    shulib::sequence::RunGuard guard{svc};
    shulib::chassis::Chassis chassis;
    mech::Devices devs[3]{{.motors = &mArm}, {.motors = &mLift}, {.pneumatic = &mPiston}};

    static sim::ArmPlantConfig armConfig(const Options& o) {
        sim::ArmPlantConfig c = flipArmPlant();
        if (o.slowArm > 0.0) {
            c.kV *= o.slowArm;  // the joint's free speed divided by slowArm
            c.kA = 0.03 * c.kV;
        }
        return c;
    }
    static shulib::motion::MotionConfig motionOf(const Options& o) {
        return o.motion != nullptr ? *o.motion : motion_rig::motionConfig();
    }
    static shulib::sim::SimHarnessConfig harness(const Options& o) {
        shulib::sim::SimHarnessConfig c = motion_rig::plantConfig();
        c.plant.initialPose = parkedAt(o.wall, o.lookFrom, o.headingDeg, o.lateral);
        return c;
    }

    explicit Rig(const Options& o = {})
        : opt{o},
          rig{kin, harness(o), &sink},
          armPlant{armF, armConfig(o)},
          toggle{onWall(o.toggle, o.wall)},
          camPlant{camrig::truth(o.lensHeight, o.lensPitch), o.seed},
          feed{camera, rig.h.clock(), vision::CameraFeedConfig{0.033}},
          model{camrig::model(o.lensHeight, o.lensPitch)},
          chassis{rig.deps, guard, shulib::chassis::ChassisConfig{.motion = motionOf(o)}} {
        camPlant.setJitterPx(o.jitterPx);
        camera.cam = &camPlant;
        camera.rig = &rig;
        cam.feed = &feed;
        cam.model = &model;
        cam.tracker = camrig::tcfg();
        world.r = this;
        toggle.attachArm(armPlant);
        toggle.setOurs(sc::oursFor(o.wall) == vision::ToggleColour::Red ? kRed : kBlue);
        for (const sim::TogglePlantConfig& t : o.others) {
            others.push_back(std::make_unique<sim::TogglePlant>(t));
        }
        svc.bind(o.table != nullptr ? *o.table : tv_fixture::kFlippers, devs, rig.h.clock(), rig.latch, sink);
        redraw();
    }

    [[nodiscard]] double now() { return rig.h.clock().now().value(); }
    /// Hold the drive's motors at 0 V for THIS pace (call from World::onPace, which runs after the motion
    /// commanded them and before the plant steps) — something blocking the robot.
    void stopDrive() {
        for (int i = 0; i < kin.wheelCount(); ++i) {
            rig.h.motor(i).setVoltage(shulib::units::Voltage{0.0});
        }
    }
    /// The primary Toggle's config, on its wall.
    [[nodiscard]] sim::TogglePlantConfig toggleConfig() const { return onWall(opt.toggle, opt.wall); }
    [[nodiscard]] shulib::math::Pose2d truth() { return rig.h.plant().truePose(); }
    /// The chosen flipper's TRUE angle, degrees (the lift's height and the piston's stroke mapped onto
    /// 0..90).
    [[nodiscard]] double flipperDeg() const {
        switch (opt.form) {
            case Form::Arm: return armPlant.trueDeg();
            case Form::Lift: return liftPlant.trueDeg() / 360.0 * kPi * 1.5 / 5.0 * 90.0;
            case Form::Pneumatic: return pistonOut * 90.0;
            case Form::Passive: return 0.0;
        }
        return 0.0;
    }
    void redraw() {
        camPlant.clear();
        for (const sim::ColourQuad& q : extras) {
            camPlant.addQuad(q);
        }
        if (opt.drawToggle) {
            toggle.draw(camPlant);
        }
        for (auto& t : others) {
            t->draw(camPlant);
        }
    }
    /// The front's TRUE distance from Toggle `t`'s wall (infinite beside it).
    [[nodiscard]] double frontTo(const sim::TogglePlantConfig& t) {
        return frontToWall(truth(), t.wallX, t.wallY, t.inX, t.inY);
    }
    /// Lines of `subsystem`, in order.
    [[nodiscard]] std::vector<std::string> lines(const char* subsystem) const {
        std::vector<std::string> out;
        for (int i = 0; i < sink.size(); ++i) {
            if (sink.at(i).subsystem == subsystem) {
                out.push_back(sink.at(i).message);
            }
        }
        return out;
    }
    /// The level of the last line of `subsystem`.
    [[nodiscard]] shulib::hal::LogLevel lastLevel(const char* subsystem) const {
        for (int i = sink.size() - 1; i >= 0; --i) {
            if (sink.at(i).subsystem == subsystem) {
                return sink.at(i).level;
            }
        }
        return shulib::hal::LogLevel::Trace;
    }

    /// An entry of `kind` with a `start` (RELATIVE by default).
    static shulib::competition::AutonEntry entry(sc::AutonKind kind = sc::AutonKind::Skills,
                                                 sc::StartPose start = sc::StartPose::relative()) {
        sc::AutonEntry e{};
        e.name = "toggle";
        e.kind = kind;
        e.start = start;
        e.endActionAt = Time{55.0};
        e.hardStopAt = Time{60.0};
        return e;
    }

    /// One guarded run whose scoring is `scoring`, deadline `endActionAt` from now.
    template <class Scoring>
    shulib::sequence::RunGuardReport runGuarded(Scoring&& scoring, double endActionAt = 55.0) {
        shulib::sequence::RunGuardConfig g{};
        g.endActionAt = Time{endActionAt};
        g.hardStopAt = Time{endActionAt + 2.0};
        g.mechanisms = svc.mechanisms();
        return guard.run(chassis, g, scoring, [] { return true; });
    }

    /// One visit with `service` (nullptr: NO service — a passive flipper's program), under a guard whose
    /// scoring deadline is `endActionAt` from the run's start.
    sc::ToggleVisitReport visitWith(mech::MechanismService* service, const sc::ToggleVisitRequest& rq,
                                    const sc::AutonEntry& e, double endActionAt = 55.0, bool lookOnly = false) {
        sc::ToggleVisitReport out{};
        runGuarded(
            [&] {
                out = lookOnly ? sc::lookAtToggle(cam, chassis, guard, service, e, rq, sink)
                               : sc::checkAndFlipToggle(cam, chassis, guard, service, e, rq, sink);
            },
            endActionAt);
        return out;
    }
    /// One visit with the rig's own service.
    sc::ToggleVisitReport visit(const sc::ToggleVisitRequest& rq, const sc::AutonEntry& e, double endActionAt = 55.0,
                                bool lookOnly = false) {
        return visitWith(&svc, rq, e, endActionAt, lookOnly);
    }
};

inline void World::pace() {
    if (paces > 300000) {
        throw std::runtime_error("tv_rig::World: pace cap exceeded -- a wait failed to end");
    }
    ++paces;
    if (onPace) {
        onPace();
    }
    const double dt = 0.01;
    r->armPlant.step(dt);
    r->liftPlant.step(dt);
    const bool high = r->lineA.commanded();
    r->pistonOut = std::fmin(r->pistonMax, std::fmax(r->pistonMin, r->pistonOut + (high ? dt : -dt) / 0.25));
    r->rig.h.plant().step(Time{dt});
    r->toggle.step(dt, r->flipperDeg(), r->frontTo(r->toggleConfig()));
    for (std::size_t i = 0; i < r->others.size(); ++i) {
        r->others[i]->step(dt, r->flipperDeg(), r->frontTo(r->opt.others[i]));
    }
    r->redraw();
}

/// The request the scenes use — every number a PLACEHOLDER (the probes' rig; the team's come from Step 0).
inline sc::ToggleVisitRequest armRequest(ov::ToggleId id = ov::toggles::kRightWall) {
    sc::ToggleVisitRequest r{};
    r.toggle = id;
    r.flipper = tv_fixture::flipArm["flip"];
    r.back = tv_fixture::flipArm["stow"];
    r.lookFrom = Length{16.0};
    r.offBy = Length{6.0};
    r.pushFrom = Length{4.0};
    r.lookWithin = Time{0.6};
    r.flipBound = Time{1.2};
    r.backBound = Time{1.2};
    r.settle = Time{0.3};
    r.driveBound = Time{1.5};
    r.margin = Time{0.3};
    r.maxImageAge = Time{0.1};
    r.maxPushes = 2;
    return r;
}
/// The lift form's request.
inline sc::ToggleVisitRequest liftRequest() {
    sc::ToggleVisitRequest r = armRequest();
    r.flipper = tv_fixture::flipLift["flip"];
    r.back = tv_fixture::flipLift["stow"];
    return r;
}
/// The pneumatic form's request.
inline sc::ToggleVisitRequest pistonRequest() {
    sc::ToggleVisitRequest r = armRequest();
    r.flipper = tv_fixture::flipPiston["flip"];
    r.back = tv_fixture::flipPiston["stow"];
    return r;
}
/// The PASSIVE form's request (no mechanism: the drive is the stroke).
inline sc::ToggleVisitRequest passiveRequest() {
    sc::ToggleVisitRequest r = armRequest();
    r.flipper = sc::FlipperGoal{};
    r.back = sc::FlipperGoal{};
    r.flipBound = Time{0.0};
    r.backBound = Time{0.0};
    return r;
}

/// Options for a scene with flipper `f` and the default red-side Toggle (Y field, B bottom, R outer).
inline Options scene(Form f = Form::Arm, ov::ToggleId wall = ov::toggles::kRightWall) {
    Options o{};
    o.wall = wall;
    o.form = f;
    o.toggle = redSideToggle();
    o.toggle.passive = f == Form::Passive;
    return o;
}

}  // namespace tv_rig
