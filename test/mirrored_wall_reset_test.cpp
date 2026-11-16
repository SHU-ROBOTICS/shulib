// The wall reset in a MIRRORED auton — MIRROR1-A's T6 (the MIRROR1 brief's §2.2, D4, D5 (1); the brief's
// probe P2 made a test), on the robot built from a robot file with side sensors (test/wall_robot_rig.hpp),
// through competition::runAuton, FIELD starts. Everything here is the host test plant; nothing here has
// run on a robot.
//
// Robot one is parked along the RED wall, nose +x, its RIGHT sensor 7 in from the wall, its estimate 1 in
// off; the route asks for `red wall, RIGHT sensor`. Robot two is robot one reflected across y = -x by hand
// — along the RIGHT wall, nose -y — and runs the SAME routine with the SAME request, the entry mirrored:
// the library must swap the wall (its image: the right wall, with ITS keep-outs) and the sensor (LEFT).
//
// Each case attacks ONE rule and names the bug it would catch.

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "shulib/chassis/routine.hpp"
#include "shulib/chassis/wall_reset_call.hpp"
#include "shulib/competition/auton_wall_reset.hpp"
#include "shulib/competition/field_mirror.hpp"
#include "shulib/competition/override_mirror.hpp"
#include "shulib/core/check.hpp"
#include "shulib/localization/override_walls.hpp"
#include "shulib/localization/wall_reset.hpp"
#include "shulib/units/literals.hpp"
#include "wall_robot_rig.hpp"

using namespace shulib::units::literals;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::Length;
namespace loc = shulib::localization;
namespace walls = shulib::localization::override2026;
namespace comp = shulib::competition;
using comp::override2026::kSkillsMirror;

namespace {

loc::WallResetRequest gReq{};
loc::WallResetReport gRep{};
bool gExplicit = false;  ///< the routine calls the EXPLICIT form instead (D5 (1))

shulib::chassis::RoutineResult resetOnly(comp::AutonContext& ctx) {
    shulib::chassis::Routine r{ctx, "wall"};
    r.then(
        [&ctx] {
            if (gExplicit) {
                loc::WallSensors* w = ctx.chassis.deps().ctx->wallSensors();
                gRep = shulib::chassis::resetFromWall(ctx.chassis, w->side(gReq.sensor), gReq, w->limits);
            } else {
                gRep = comp::resetFromWall(ctx, gReq);
            }
            return gRep.continues();
        },
        "wall reset");
    return r.result();
}

Pose2d rf(const Pose2d& p) {
    return Pose2d{Length{-p.y().value()}, Length{-p.x().value()}, Angle::degrees(-90.0 - p.heading().degrees())};
}

struct Outcome {
    loc::WallResetReport rep{};
    std::vector<std::string> wallLines;
    Pose2d before{};
    Pose2d after{};
};

/// One auton on the wall robot: truth `truth`, the entry's FIELD start `start` (for a MIRRORED entry,
/// ROBOT ONE's start — the runner seats its reflection), the request `rq`, mirrored or not; `withSensors`
/// false builds the same robot from a file that declares NO wall sensors (no slot: NO SENSOR).
Outcome runOne(const Pose2d& truth, const Pose2d& start, const loc::WallResetRequest& rq, bool mirror,
               bool withSensors = true) {
    Outcome o;
    wall_robot::WallWorld world{truth};
    shulib::robot::Robot& rb = world.build(
        withSensors ? wall_robot::robotWith(wall_robot::wallsFor(wall_rig::trueLeft(), wall_rig::trueRight()))
                    : wall_robot::robotWith({}));
    gReq = rq;
    gRep = loc::WallResetReport{};
    comp::AutonEntry e = wall_robot::entryFor("wall", &resetOnly, comp::StartPose::field(start));
    if (mirror) {
        e = comp::mirrored(e, kSkillsMirror);
    }
    const comp::AutonRunResult res = world.run(e);
    REQUIRE(res.started());
    o.rep = gRep;
    o.after = rb.localizer().pose();
    for (int i = 0; i < world.sink.size(); ++i) {
        if (world.sink.at(i).subsystem == "WALL") {
            o.wallLines.push_back(world.sink.at(i).message);
        }
    }
    return o;
}

/// Robot one's truth and belief (the brief's P2): the right sensor 7 in from the red wall, 1 in off in y.
Pose2d truthOne() { return Pose2d{Length{-36.0}, Length{-walls::kWallInnerFace.value() + 7.5 + 7.0}, 0_deg}; }
Pose2d believedOne() { return Pose2d{Length{-36.0}, Length{-walls::kWallInnerFace.value() + 7.5 + 8.0}, 0_deg}; }

/// The union of the kept-out spans along a wall, as sorted disjoint intervals.
std::vector<std::pair<double, double>> keptOut(const loc::FieldWall& w) {
    std::vector<std::pair<double, double>> v;
    for (const loc::WallKeepOut& k : w.keepOuts()) {
        v.emplace_back(k.from.value(), k.to.value());
    }
    std::sort(v.begin(), v.end());
    std::vector<std::pair<double, double>> u;
    for (const auto& s : v) {
        if (!u.empty() && s.first <= u.back().second + 1e-9) {
            u.back().second = std::max(u.back().second, s.second);
        } else {
            u.push_back(s);
        }
    }
    return u;
}

}  // namespace

// Bug caught: MM5 (the wall mapped, the side NOT — robot two's beam would point away from the wall: "not
// square", the reset lost); the wall not mapped; the sensor not swapped. Robot one's request on robot two
// SEATS the right wall with the LEFT sensor, its correction robot one's reflected: (0, -1) in -> (+1, 0).
// The line names the REAL wall and side, the mirrored note inside the sensor's parenthesis.
TEST_CASE("MIRROR1 T6 robot one's wall reset on robot two seats the RIGHT wall with the LEFT sensor") {
    const loc::WallResetRequest rq{.wall = walls::kRedWall, .sensor = loc::WallSide::Right, .maxCorrection = 3_in};
    const Outcome one = runOne(truthOne(), believedOne(), rq, false);
    const Outcome two = runOne(rf(truthOne()), believedOne(), rq, true);
    REQUIRE(one.rep.seated());
    REQUIRE(two.rep.seated());
    CHECK(one.rep.axis == loc::Axis::Y);
    CHECK(two.rep.axis == loc::Axis::X);
    CHECK(two.rep.side == loc::WallSide::Left);
    CHECK(one.rep.applied.value() == doctest::Approx(-1.0).epsilon(0.1));
    CHECK(two.rep.applied.value() == doctest::Approx(1.0).epsilon(0.1));
    CHECK(std::fabs(two.rep.applied.value() + one.rep.applied.value()) < 0.1);  // the reflection, to the noise
    // robot two's estimate seated onto its truth on x — the right wall's axis
    CHECK(std::fabs(two.after.x().value() - rf(truthOne()).x().value()) < 0.1);
    REQUIRE(two.wallLines.size() == 1U);
    CHECK(two.wallLines[0].rfind("right wall x SEATED (LEFT sensor, mirrored from red wall RIGHT): 54.70 -> ", 0) == 0);
    REQUIRE(one.wallLines.size() == 1U);
    CHECK(one.wallLines[0].rfind("red wall y SEATED (RIGHT sensor): -54.70 -> ", 0) == 0);  // byte-identical form
}

// Bug caught (a review finding, 2026-11-13 — the mutant survived): the NO SENSOR refusal in a mirrored
// auton naming robot one's wall and side, its note dropped (the refusal's call handed the request as
// written) — robot two's operators would read "red wall ... (RIGHT sensor)" for a reset meant for the
// right wall's LEFT sensor. On a robot whose file declares no wall sensors, the mirrored entry's ONE
// WALL line names the REAL wall and side with the mirrored note; unmirrored, L6's line, byte for byte.
TEST_CASE("MIRROR1 T6 a mirrored reset on a robot with NO wall sensors: refused naming the real wall, side and note") {
    const loc::WallResetRequest rq{.wall = walls::kRedWall, .sensor = loc::WallSide::Right, .maxCorrection = 3_in};
    const Outcome two = runOne(rf(truthOne()), believedOne(), rq, true, /*withSensors=*/false);
    CHECK(two.rep.verdict == loc::WallResetVerdict::NoSensor);
    CHECK(two.rep.side == loc::WallSide::Left);
    REQUIRE(two.wallLines.size() == 1U);
    CHECK(two.wallLines[0]
          == "right wall x REFUSED no sensor (LEFT sensor, mirrored from red wall RIGHT): this robot declares no wall "
             "sensors");
    const Outcome one = runOne(truthOne(), believedOne(), rq, false, /*withSensors=*/false);
    CHECK(one.rep.verdict == loc::WallResetVerdict::NoSensor);
    REQUIRE(one.wallLines.size() == 1U);
    CHECK(one.wallLines[0] == "red wall y REFUSED no sensor (RIGHT sensor): this robot declares no wall sensors");
}

// Bug caught: the wrong half of the swap. Robot two at its spot with each WRONG request run UNMIRRORED —
// the unmirrored request, the mirror wall with the side not swapped, the side swapped with the wall not
// — is refused "not square" (the brief's P2), nothing seated: only the full swap seats.
TEST_CASE("MIRROR1 T6 the three half-swaps on robot two are each refused not square, nothing seated") {
    const std::pair<loc::FieldWall, loc::WallSide> wrong[3] = {{walls::kRedWall, loc::WallSide::Right},
                                                              {walls::kRightWall, loc::WallSide::Right},
                                                              {walls::kRedWall, loc::WallSide::Left}};
    for (const auto& w : wrong) {
        const loc::WallResetRequest rq{.wall = w.first, .sensor = w.second, .maxCorrection = 3_in};
        const Outcome o = runOne(rf(truthOne()), rf(believedOne()), rq, false);
        CHECK(o.rep.verdict == loc::WallResetVerdict::NotSquare);
        CHECK(o.after.x().value() == doctest::Approx(rf(believedOne()).x().value()).epsilon(1e-12));
    }
    // and the full swap, written by hand on an UNMIRRORED entry, seats — the mirror IS that swap
    const loc::WallResetRequest byHand{.wall = walls::kRightWall, .sensor = loc::WallSide::Left, .maxCorrection = 3_in};
    CHECK(runOne(rf(truthOne()), rf(believedOne()), byHand, false).rep.seated());
}

// Bug caught: an author's OWN wall line (frame Own) left unreflected or matched to a season wall's name.
// Its image is reflected GEOMETRICALLY — the line, the side the field is on, its spans — its name kept.
TEST_CASE("MIRROR1 T6 an author's own wall line is reflected geometrically, its name kept") {
    const loc::FieldWall own =
        loc::FieldWall::line("my line", loc::Axis::Y, Length{-walls::kWallInnerFace.value()}, +1);
    const loc::WallResetRequest rq{.wall = own, .sensor = loc::WallSide::Right, .maxCorrection = 3_in};
    const Outcome two = runOne(rf(truthOne()), believedOne(), rq, true);
    CHECK(two.rep.seated());
    REQUIRE(two.wallLines.size() == 1U);
    CHECK(two.wallLines[0].rfind("my line x SEATED (LEFT sensor, mirrored from my line RIGHT): ", 0) == 0);
    const loc::FieldWall img = comp::mirroredWall(kSkillsMirror, own);
    CHECK(img.frame == loc::WallFrame::Own);
    CHECK(img.axis == loc::Axis::X);
    CHECK(img.coord.value() == walls::kWallInnerFace.value());
    CHECK(img.inward == -1);
    CHECK(std::string{img.name} == "my line");
}

// Bug caught: a season wall's image built from the source wall's spans instead of the REAL image wall
// (the right wall has no Loaders; the red wall's Loaders reflected onto it would be invented keep-outs) —
// and a reflection that loses a kept-out stretch. mirroredWall returns the real season wall; the
// geometric reflection of each season wall keeps out EXACTLY the stretches its image does (the Loaders
// lie inside the Load Zones, so the unions agree).
TEST_CASE("MIRROR1 T6 the season walls' images: the real image wall, and the same kept-out stretches") {
    const std::pair<const loc::FieldWall*, const loc::FieldWall*> pairs[4] = {
        {&walls::kRedWall, &walls::kRightWall},
        {&walls::kRightWall, &walls::kRedWall},
        {&walls::kLeftWall, &walls::kBlueWall},
        {&walls::kBlueWall, &walls::kLeftWall}};
    for (const auto& p : pairs) {
        CAPTURE(p.first->name);
        const loc::FieldWall img = comp::mirroredWall(kSkillsMirror, *p.first);
        CHECK(std::string{img.name} == std::string{p.second->name});
        CHECK(img.keepOutCount == p.second->keepOutCount);
        const loc::FieldWall geo = comp::reflectedWall(kSkillsMirror.reflection, *p.first);
        CHECK(geo.axis == p.second->axis);
        CHECK(geo.coord.value() == p.second->coord.value());
        CHECK(geo.inward == p.second->inward);
        const auto a = keptOut(geo);
        const auto b = keptOut(*p.second);
        REQUIRE(a.size() == b.size());
        for (std::size_t i = 0; i < a.size(); ++i) {
            CHECK(a[i].first == doctest::Approx(b[i].first).epsilon(1e-12));
            CHECK(a[i].second == doctest::Approx(b[i].second).epsilon(1e-12));
        }
    }
    CHECK(comp::mirroredSide(loc::WallSide::Left) == loc::WallSide::Right);
    CHECK(comp::mirroredSide(loc::WallSide::Right) == loc::WallSide::Left);
}

// Bug caught (D5 (1)): the EXPLICIT form — handed ONE sensor device — run inside a mirrored auton, reading
// robot one's side on robot two. A precondition naming the everyday form.
TEST_CASE("MIRROR1 T6 the explicit wall reset inside a mirrored run is a precondition naming the everyday form") {
    gExplicit = true;
    const loc::WallResetRequest rq{.wall = walls::kRedWall, .sensor = loc::WallSide::Right, .maxCorrection = 3_in};
    CHECK_THROWS_WITH_AS((void)runOne(rf(truthOne()), believedOne(), rq, true),
                         "chassis::resetFromWall: a mirrored run is open -- this form is handed one sensor and a "
                         "mirrored auton needs the other side's: call competition::resetFromWall(ctx, ...), which "
                         "swaps the wall and the sensor side",
                         shulib::PreconditionError);
    // unmirrored, the explicit form runs as ever
    const Outcome one = runOne(truthOne(), believedOne(), rq, false);
    gExplicit = false;
    CHECK(one.rep.seated());
}
