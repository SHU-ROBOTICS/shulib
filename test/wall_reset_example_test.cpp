// The wall reset's guide listing, COMPILED (chunk L6-A1): docs/guide/10-the-api.md's section "The
// wall reset — one axis against a field wall" quotes the lines between the markers below verbatim, and
// `api_doc_tool.py check-examples` fails the build if the two ever drift (this file's name matches
// the tool's `*example*_test.cpp`).
//
//   guide-10w -> docs/guide/10-the-api.md (the explicit call over a hand-wired side sensor)
//
// It runs on the host test plant (wall_reset_rig.hpp): a tank parked beside the left wall, its left
// sensor 7 in from it, the estimate drifted 1.5 in — and the reset must SEAT x back to the truth.

#include "doctest.h"

#include <cmath>

#include "shulib/chassis/wall_reset_call.hpp"
#include "shulib/localization/override_walls.hpp"
#include "shulib/localization/wall_reset.hpp"
#include "shulib/units/literals.hpp"
#include "wall_reset_rig.hpp"

using namespace shulib::units::literals;

namespace {
namespace chassis = shulib::chassis;
namespace localization = shulib::localization;
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
    // sensor, its read-fault count, and the mount a student MEASURED.
    localization::SideMount mount{};
    mount.measured = true;
    mount.left = 7.5_in;     // out from the robot's centre
    mount.height = 10.5_in;  // the lens above the floor
    mount.facing = units::AngleDim{90.0 * localization::kWallDegToRad};  // straight out the left
    mount.provenance = "tape, 2026-10-29";
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
