// The wall reset's guide listing, COMPILED (chunk L6-A1): docs/guide/10-the-api.md's section "The
// wall reset — one axis against a field wall" quotes the lines between the markers below verbatim, and
// `api_doc_tool.py check-examples` fails the build if the two ever drift (this file's name matches
// the tool's `*example*_test.cpp`).
//
//   guide-10w -> docs/guide/10-the-api.md (the explicit call over a hand-wired side sensor)
//   guide-10x -> docs/guide/10-the-api.md (the robot file's side sensors, built — chunk L6-A2)
//   L6 guide-03 bands -> docs/guide/03-knowing-where-you-are.md's table of clean stretches (L6-A2): the
//                        numbers the chapter prints, recomputed from the library's own pre-checks
//
// guide-10w runs on the host test plant (wall_reset_rig.hpp): a tank parked beside the left wall, its
// left sensor 7 in from it, the estimate drifted 1.5 in — and the reset must SEAT x back to the truth.

#include "doctest.h"

#include <cmath>
#include <string>

#include "shulib/chassis/wall_reset_call.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/config/wall_sensor_config.hpp"
#include "shulib/hal/distance.hpp"
#include "shulib/localization/override_walls.hpp"
#include "shulib/localization/wall_reset.hpp"
#include "shulib/units/literals.hpp"
#include "wall_reset_rig.hpp"
#include "wall_robot_rig.hpp"

using namespace shulib::units::literals;

namespace {
namespace chassis = shulib::chassis;
namespace config = shulib::config;
namespace hal = shulib::hal;
namespace localization = shulib::localization;
namespace robot = shulib::robot;
namespace units = shulib::units;
using shulib::math::Angle;
using shulib::math::Pose2d;
}  // namespace

// ═══ guide-10w: the explicit call over a hand-wired side sensor ══════════════════════════════
// Bug caught: a guide listing that does not compile, or that teaches a call that does not seat — a
// mount typed as the guide types it, a request built as the guide builds it, on a robot whose
// estimate has drifted, must come back SEATED with x within 0.1 in of the truth.
TEST_CASE("guide-10w: re-seat x against the left wall with the explicit call") {
    wall_rig::WallRig rig{Pose2d{units::Length{-55.705}, units::Length{-38.0}, Angle::degrees(90.0)}};
    rig.idle(0.3);
    rig.loc().seatAxis(localization::Axis::X, rig.loc().pose().x() + units::Length{1.5});
    rig.idle(0.05);
    auto& sensor = rig.leftPlant.sensor();
    auto& faults = rig.leftPlant.faults();
    auto& c = rig;

    // A hand-wired robot's left side sensor, as the wall reset reads it: the
    // sensor, its read-fault count, and the mount a student MEASURED (an
    // example robot's numbers: type your own robot's, measured).
    localization::SideMount mount{};
    mount.measured = true;
    mount.left = 7.5_in;     // out from the robot's centre
    mount.height = 10.5_in;  // the lens above the floor
    mount.facing = units::AngleDim{90.0 * localization::kWallDegToRad};  // straight out the left
    mount.provenance = "EXAMPLE: who measured, with what, on what date";
    const localization::WallSensor left{&sensor, &faults, mount, "not declared"};

    // Re-seat x against the left wall: the beam square to it, on a clean
    // stretch of it, and x may move by at most 3 inches.
    const localization::WallResetReport report = chassis::resetFromWall(
        c.chassis, left,
        {.wall = localization::override2026::kLeftWall,
         .sensor = localization::WallSide::Left,
         .maxCorrection = 3_in},
        localization::WallResetLimits{});
    CHECK(report.seated());

    CHECK(std::abs(rig.loc().pose().x().value() - rig.h().truePose().x().value()) < 0.1);
    CHECK(report.continues());
}

// ═══ guide-10x: the robot file's side sensors, built (L6-A2) ═════════════════════════════════
// Bug caught: a guide listing of the robot file's `walls` block that does not compile as a constexpr
// block (robot two's sits in a constexpr table), or that teaches a block the Robot does not build — the
// sensors must come out of the provider on ports 4 and 5, each with the mount typed here.
namespace {
// The robot file's side distance sensors: one on each side, its port,
// and the mount a student MEASURED with a tape (UNSET until then: the
// robot still boots, and every wall reset with that sensor refuses).
// EVERY NUMBER HERE IS AN EXAMPLE ROBOT'S: type your own, measured, and
// how well, and who measured it, how and when -- never these.
constexpr config::WallSensorsConfig kWalls{
    .left = {.port = 4,
             .mount = {.measured = true,
                       .left = 7.5_in,
                       .height = 10.5_in,
                       .facing = units::AngleDim{90.0 * localization::kWallDegToRad},
                       .positionTolerance = 0.25_in,
                       .facingTolerance = units::AngleDim{2.0 * localization::kWallDegToRad},
                       .provenance = "EXAMPLE: who measured, with what, on what date"}},
    .right = {.port = 5,
              .mount = {.measured = true,
                        .left = -7.5_in,
                        .height = 10.5_in,
                        .facing = units::AngleDim{-90.0 * localization::kWallDegToRad},
                        .positionTolerance = 0.25_in,
                        .facingTolerance = units::AngleDim{2.0 * localization::kWallDegToRad},
                        .provenance = "EXAMPLE: who measured, with what, on what date"}},
    .note = "an example robot: both at the fore-aft centre, 10.5 in up"};
}  // namespace

TEST_CASE("guide-10x: the robot file's side sensors -- the walls block, built by the Robot") {
    static_assert(config::wallMountProblem(kWalls.left.mount) == nullptr);
    static_assert(config::wallMountProblem(kWalls.right.mount) == nullptr);
    wall_robot::WallWorld world{Pose2d{units::Length{-55.705}, units::Length{-38.0}, Angle::degrees(90.0)}};
    config::RobotConfig cfg = wall_robot::robotWith({});
    auto& devices = world.devices;

    cfg.sensors.walls = kWalls;
    robot::Robot robot{cfg, devices};
    // Each side as built: the sensor (nullptr when the provider refused
    // it) and why it is absent ("" exactly when it is there).
    hal::IDistance* left = robot.wallSensor(localization::WallSide::Left);
    const char* why = robot.wallSensorAbsence(localization::WallSide::Right);

    CHECK(left == &devices.fakeDistance(4));
    CHECK(std::string{why}.empty());
    CHECK(robot.wallSensors().right.mount.left.value() == -7.5);
    // The tolerances typed in the listing, carried as typed (D6 (a): each number with how well).
    CHECK(robot.wallSensors().left.mount.positionTolerance.value() == 0.25);
    CHECK(robot.wallSensors().right.mount.facingTolerance.value() == 2.0 * localization::kWallDegToRad);
    CHECK(std::string{robot.wallSensors().left.mount.provenance}.rfind("EXAMPLE", 0) == 0);
}

// ═══ guide-03's table: the clean stretches, from the library's own pre-checks (L6-A2) ═════════
// Bug caught: guide 03's table of where the beam may land drifting from the keep-outs it describes — a
// season-data change, a widening change, or a hand-copied number nobody re-derived. Each row is
// re-computed here through `localization::preCheck` (rules 2-4, reading nothing): a centred LEFT sensor
// 10.5 in up, square to the wall, a 7-in reading, the beam centre walked along the wall in 0.05-in steps;
// the row's stretch is where preCheck says "go on and sample". All four walls give the same table (the
// Loaders sit inside the Load Zones' corner spans), and the test proves that too.
TEST_CASE("L6 guide-03 bands: the clean stretches the chapter prints are the library's -- every wall, the cone "
          "unmeasured and measured, after 0 / 12 / 24 / 48 / 96 in of travel") {
    struct Row {
        double coneDeg;  // 0 = unmeasured (the limits' 18 degrees, the start pieces kept out)
        double travel;   // inches since the along axis was last set
        double from;     // the stretch on the positive side, |along|; from > to = none
        double to;
    };
    const Row rows[] = {
        {0.0, 0.0, 33.6, 41.8},  {0.0, 12.0, 34.8, 40.6}, {0.0, 24.0, 36.0, 39.4}, {0.0, 48.0, 1.0, 0.0},
        {0.0, 96.0, 1.0, 0.0},   {18.0, 0.0, 18.3, 41.8}, {18.0, 12.0, 19.5, 40.6}, {18.0, 24.0, 20.7, 39.4},
        {18.0, 48.0, 23.1, 37.0}, {18.0, 96.0, 27.9, 32.2},
    };
    for (const localization::FieldWall& w : localization::override2026::kWalls) {
        for (const Row& row : rows) {
            CAPTURE(w.name);
            CAPTURE(row.coneDeg);
            CAPTURE(row.travel);
            localization::SideMount m{};
            m.measured = true;
            m.left = units::Length{7.5};
            m.height = units::Length{10.5};
            m.facing = units::AngleDim{90.0 * localization::kWallDegToRad};
            m.coneHalfAngle = units::AngleDim{row.coneDeg * localization::kWallDegToRad};
            m.provenance = "test";
            localization::WallResetRequest req{};
            req.wall = w;
            req.sensor = localization::WallSide::Left;
            req.maxCorrection = units::Length{3.0};
            const localization::WallResetLimits lim{};
            const units::Length unc{lim.alongDriftRate * row.travel};
            const auto clean = [&](double a) {
                const double d = w.coord.value() + w.inward * (7.5 + 7.0);
                const bool onX = w.axis == localization::Axis::X;
                const double th = onX ? (w.inward > 0 ? 90.0 : -90.0) : (w.inward > 0 ? 180.0 : 0.0);
                const Pose2d p{units::Length{onX ? d : a}, units::Length{onX ? a : d}, Angle::degrees(th)};
                return localization::preCheck(p, m, req, lim, unc) == localization::WallResetVerdict::Seated;
            };
            // Walk the positive half; both halves are mirror images (checked at the stretch's middle).
            double from = 1.0;
            double to = 0.0;
            bool in = false;
            for (int k = 0; k <= 1400; ++k) {
                const double a = 0.05 * k;
                const bool ok = clean(a);
                if (ok && !in) {
                    from = a;
                    in = true;
                }
                if (ok) {
                    to = a;
                }
                if (!ok && in) {
                    break;
                }
            }
            if (row.from > row.to) {
                CHECK_FALSE(in);
                CHECK(from > to);  // no stretch at all
            } else {
                CHECK(std::abs(from - row.from) <= 0.051);
                CHECK(std::abs(to - row.to) <= 0.051);
                CHECK(clean(-0.5 * (row.from + row.to)));
            }
        }
    }
}
