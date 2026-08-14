// Tests for localization/tag_map.hpp — where the tags are, where those numbers came from, and
// the rigid-body inversion that turns "a tag is HERE relative to me" into "I am THERE" (E3, T2).
//
// TWO THINGS ARE UNDER TEST AND THEY FAIL DIFFERENTLY.
//
//  * THE INVERSION is arithmetic, and its failure mode is a frame error — a swapped axis, a
//    dropped cross term, a rotation by the tag's heading instead of the robot's. Every such
//    error is INVISIBLE at heading 0 and at the origin, so NO CASE HERE USES EITHER. Every case
//    also uses a non-zero relative heading and non-zero rx AND ry, because a cross-term error
//    has somewhere to hide the moment one of those is zero. The anchor cases are worked entirely
//    by hand with the arithmetic written out, so they share nothing with the implementation.
//
//  * THE PROVENANCE CONTRACT is a policy, and its failure mode is an invented tag pose that
//    nobody can tell from a specified one. It is enforced by precondition, so the tests here
//    drive it through the precondition handler.

#include "doctest.h"

#include <cmath>
#include <limits>

#include "shulib/core/check.hpp"
#include "shulib/localization/tag_map.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::localization::TagMap;
using shulib::localization::TagPlacement;
using shulib::localization::TagProvenance;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::Length;

namespace {


/// The FORWARD composition, written here independently: given where the robot is and where a
/// tag sits relative to it, where is the tag on the field? Two lines of rigid-body algebra, and
/// deliberately NOT a call into anything in tag_map.hpp.
///     Tx = Rx + rx·cos(Rθ) − ry·sin(Rθ)
///     Ty = Ry + rx·sin(Rθ) + ry·cos(Rθ)
///     Tθ = Rθ + rθ
[[nodiscard]] Pose2d tagFieldPoseFrom(const Pose2d& robot, const Pose2d& tagInRobot) {
    const double c = std::cos(robot.heading().radians());
    const double s = std::sin(robot.heading().radians());
    const double rx = tagInRobot.x().value();
    const double ry = tagInRobot.y().value();
    return Pose2d{Length{robot.x().value() + rx * c - ry * s},
                  Length{robot.y().value() + rx * s + ry * c},
                  Angle::radians(robot.heading().radians() + tagInRobot.heading().radians())};
}

}  // namespace

// ── THE INVERSION ────────────────────────────────────────────────────────────────────────────

// Would catch: the position term being rotated by the TAG's heading instead of the ROBOT's —
// the one ordering mistake the formula invites, since Tθ is right there and already known.
//
// HAND-WORKED. Robot at (10, -6) facing 30 degrees. A tag sits at relative pose (20, 8, 45).
// Its field pose is therefore
//     Tθ = 30 + 45 = 75 degrees
//     Tx = 10 + 20·cos30 − 8·sin30 = 10 + 17.320508075688775 − 4  = 23.320508075688775
//     Ty = -6 + 20·sin30 + 8·cos30 = -6 + 10 + 6.928203230275509  = 10.928203230275509
// and the inversion must give the robot back:
//     Rθ = 75 − 45 = 30
//     Rx = 23.320508075688775 − (20·cos30 − 8·sin30) = 23.320508075688775 − 13.320508075688775 = 10
//     Ry = 10.928203230275509 − (20·sin30 + 8·cos30) = 10.928203230275509 − 16.928203230275509 = -6
TEST_CASE("TagMap inversion: hand-worked case, off-origin, off-axis heading, both terms non-zero") {
    const Pose2d tagField{Length{23.320508075688775}, Length{10.928203230275509},
                          Angle::degrees(75.0)};
    const Pose2d tagInRobot{Length{20.0}, Length{8.0}, Angle::degrees(45.0)};

    const Pose2d robot = TagMap::robotPoseFromTag(tagField, tagInRobot);
    CHECK(robot.x().value() == doctest::Approx(10.0).epsilon(1e-12));
    CHECK(robot.y().value() == doctest::Approx(-6.0).epsilon(1e-12));
    CHECK(robot.heading().degrees() == doctest::Approx(30.0).epsilon(1e-12));
}

// Would catch: the inversion using the wrong sign on the translation — visible only when the
// robot is NOT at the origin, which is why this case puts it at (30, 18).
//
// HAND-WORKED. Robot at (30, 18) facing 90 degrees (down +Y). A tag 48 inches straight ahead of
// it, on the far wall, facing back: field (30, 66) with facing -90 degrees. Relative to the
// robot the tag is at (48, 0) and turned 180 degrees.
//     Rθ = -90 − 180 = -270  ->  wraps to +90
//     Rx = 30 − (48·cos90 − 0·sin90) = 30 − 0  = 30
//     Ry = 66 − (48·sin90 + 0·cos90) = 66 − 48 = 18
TEST_CASE("TagMap inversion: hand-worked case with a wrapping heading difference") {
    const Pose2d tagField{Length{30.0}, Length{66.0}, Angle::degrees(-90.0)};
    const Pose2d tagInRobot{Length{48.0}, Length{0.0}, Angle::degrees(180.0)};

    const Pose2d robot = TagMap::robotPoseFromTag(tagField, tagInRobot);
    CHECK(robot.x().value() == doctest::Approx(30.0).epsilon(1e-12));
    CHECK(robot.y().value() == doctest::Approx(18.0).epsilon(1e-12));
    CHECK(robot.heading().degrees() == doctest::Approx(90.0).epsilon(1e-12));  // NOT -270
}

// Would catch: a frame error that survives one lucky heading. Seven robot headings x five
// relative tag poses, round-tripped through a forward composition written independently in this
// file. Not one case sits at the origin or at heading 0, and every relative pose has both
// components AND its heading non-zero.
TEST_CASE("TagMap inversion: round-trips against an independent forward composition") {
    const double robotHeadings[] = {-171.0, -104.0, -33.0, 27.0, 61.0, 128.0, 179.0};
    const double robotXs[] = {-41.5, 12.25, 63.0};
    const Pose2d relatives[] = {
        Pose2d{Length{18.0}, Length{7.5}, Angle::degrees(143.0)},
        Pose2d{Length{31.25}, Length{-12.0}, Angle::degrees(-88.0)},
        Pose2d{Length{9.0}, Length{22.0}, Angle::degrees(37.0)},
        Pose2d{Length{-14.0}, Length{5.5}, Angle::degrees(-175.0)},
        Pose2d{Length{46.0}, Length{-30.0}, Angle::degrees(91.0)},
    };
    int checked = 0;
    for (const double hd : robotHeadings) {
        for (const double rx : robotXs) {
            for (const Pose2d& rel : relatives) {
                const Pose2d robot{Length{rx}, Length{rx * 0.5 - 17.0}, Angle::degrees(hd)};
                const Pose2d tagField = tagFieldPoseFrom(robot, rel);
                const Pose2d back = TagMap::robotPoseFromTag(tagField, rel);
                CHECK(back.x().value() == doctest::Approx(robot.x().value()).epsilon(1e-10));
                CHECK(back.y().value() == doctest::Approx(robot.y().value()).epsilon(1e-10));
                CHECK(std::abs(back.heading().errorTo(robot.heading())) < 1e-12);
                ++checked;
            }
        }
    }
    CHECK(checked == 105);  // the sweep really ran
}

// Would catch: the inversion collapsing the two position axes — e.g. writing
// `Ry = Ty − (rx·cos − ry·sin)`, which is right whenever the robot faces 0 or 180 and wrong
// everywhere else. A dedicated axis-asymmetry probe: swapping rx and ry must move the answer.
TEST_CASE("TagMap inversion: the two position axes are not interchangeable") {
    const Pose2d tagField{Length{55.0}, Length{-12.0}, Angle::degrees(115.0)};
    const Pose2d a{Length{20.0}, Length{8.0}, Angle::degrees(40.0)};
    const Pose2d b{Length{8.0}, Length{20.0}, Angle::degrees(40.0)};
    const Pose2d ra = TagMap::robotPoseFromTag(tagField, a);
    const Pose2d rb = TagMap::robotPoseFromTag(tagField, b);
    CHECK(std::abs(ra.x().value() - rb.x().value()) > 1.0);
    CHECK(std::abs(ra.y().value() - rb.y().value()) > 1.0);
}

// ── THE MAP ITSELF ───────────────────────────────────────────────────────────────────────────

// Would catch: shulib quietly shipping a default field layout. A built-in map that nobody cited
// would be invented geometry wearing the clothes of a specification, and every team that forgot
// to override it would localize against fiction without a single diagnostic.
TEST_CASE("TagMap: a fresh map is EMPTY — shulib ships no field layout it cannot cite") {
    const TagMap map;
    CHECK(map.empty());
    CHECK(map.size() == 0);
    CHECK(map.find(0) == nullptr);
    CHECK(map.find(7) == nullptr);
    CHECK_FALSE(map.anyInvented());
}

// Would catch: lookup returning the wrong entry (or the first entry regardless of id), which
// would anchor the estimate to a tag it is not looking at — a fix that is confidently wrong by
// however far apart the two tags are, with a plausible residual.
TEST_CASE("TagMap: find() returns the entry for the id asked for, and only that one") {
    TagMap map;
    map.add(TagPlacement{3, Pose2d{Length{70.0}, Length{12.0}, Angle::degrees(180.0)},
                         TagProvenance::Invented, "test fixture"});
    map.add(TagPlacement{11, Pose2d{Length{-70.0}, Length{-24.0}, Angle::degrees(0.0)},
                         TagProvenance::Invented, "test fixture"});
    REQUIRE(map.size() == 2);
    REQUIRE(map.find(3) != nullptr);
    REQUIRE(map.find(11) != nullptr);
    CHECK(map.find(3)->fieldPose.x().value() == doctest::Approx(70.0));
    CHECK(map.find(11)->fieldPose.x().value() == doctest::Approx(-70.0));
    CHECK(map.find(4) == nullptr);   // not a near miss on an adjacent id
    CHECK(map.find(-1) == nullptr);
}

// Would catch: provenance degrading into a comment. If an entry could be added without saying
// where its numbers came from, a specified pose and a made-up one would be indistinguishable in
// the map, in telemetry, and in every downstream fix — and a map that is two inches off produces
// a corrector that is CONFIDENTLY two inches wrong, which no gate and no filter can reveal.
TEST_CASE("TagMap: an entry with no provenance is REFUSED, not defaulted") {
    TagMap map;
    const Pose2d anywhere{Length{40.0}, Length{9.0}, Angle::degrees(180.0)};

    CHECK_THROWS_AS(map.add(TagPlacement{1, anywhere, TagProvenance::Unspecified, "x"}),
                    shulib::PreconditionError);
    CHECK_THROWS_AS(map.add(TagPlacement{1, anywhere, TagProvenance::Invented, nullptr}),
                    shulib::PreconditionError);
    CHECK_THROWS_AS(map.add(TagPlacement{1, anywhere, TagProvenance::Invented, ""}), shulib::PreconditionError);
    CHECK_THROWS_AS(map.add(TagPlacement{-1, anywhere, TagProvenance::Invented, "x"}), shulib::PreconditionError);
    CHECK(map.empty());  // nothing partial got in

    // ...and a properly-labeled entry, invented or not, is accepted.
    map.add(TagPlacement{1, anywhere, TagProvenance::Invented, "made up for a host test"});
    CHECK(map.size() == 1);
}

// Would catch: a duplicate id being silently accepted. The second entry would simply never win a
// lookup, so the map still "works" and the wrong tag pose is used forever — the failure most
// likely to survive a code review, because nothing about it looks broken.
TEST_CASE("TagMap: a duplicate tag id is refused at setup, never resolved silently") {
    TagMap map;
    map.add(TagPlacement{5, Pose2d{Length{10.0}, Length{2.0}, Angle::degrees(90.0)},
                         TagProvenance::Specified, "host test"});
    CHECK_THROWS_AS(map.add(TagPlacement{5, Pose2d{Length{99.0}, Length{2.0}, Angle::degrees(90.0)},
                                         TagProvenance::Specified, "host test"}),
                    shulib::PreconditionError);
    REQUIRE(map.find(5) != nullptr);
    CHECK(map.find(5)->fieldPose.x().value() == doctest::Approx(10.0));  // the FIRST one stands
}

// Would catch: a non-finite tag pose entering the map, which would produce a NaN robot pose the
// moment that tag was seen — and NaN propagates silently through a complementary filter.
TEST_CASE("TagMap: a non-finite tag pose is refused") {
    TagMap map;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS(map.add(TagPlacement{2, Pose2d{Length{nan}, Length{0.0}, Angle::degrees(90.0)},
                                         TagProvenance::Invented, "host test"}),
                    shulib::PreconditionError);
    CHECK(map.empty());
}

// Would catch: `anyInvented()` reporting on the wrong axis (e.g. "any entry at all") — a run
// anchored to made-up field geometry must not read the same as one anchored to a measured field,
// and this is the flag that lets telemetry say so.
TEST_CASE("TagMap: anyInvented() distinguishes a cited map from a guessed one") {
    TagMap cited;
    cited.add(TagPlacement{1, Pose2d{Length{70.0}, Length{0.0}, Angle::degrees(180.0)},
                           TagProvenance::Specified, "field spec, section N"});
    cited.add(TagPlacement{2, Pose2d{Length{-70.0}, Length{0.0}, Angle::degrees(0.0)},
                           TagProvenance::Measured, "tape measure, 2026-08-12"});
    CHECK_FALSE(cited.anyInvented());

    TagMap guessed = cited;
    guessed.add(TagPlacement{3, Pose2d{Length{0.0}, Length{70.0}, Angle::degrees(-90.0)},
                             TagProvenance::Invented, "nobody has measured this"});
    CHECK(guessed.anyInvented());
    CHECK_FALSE(cited.anyInvented());  // the copy did not reach back
}

// Would catch: the fixed-capacity map overflowing into whatever follows it in memory. Refusing
// at setup is the only safe answer; growing would put an allocation on a path that must not have
// one, and silently dropping would make a tag mysteriously invisible mid-match.
TEST_CASE("TagMap: overflowing the fixed capacity is refused, not truncated and not grown") {
    TagMap map;
    for (int k = 0; k < static_cast<int>(TagMap::kMaxTags); ++k) {
        map.add(TagPlacement{k, Pose2d{Length{static_cast<double>(k)}, Length{1.0}, Angle{}},
                             TagProvenance::Invented, "host test"});
    }
    CHECK(map.size() == TagMap::kMaxTags);
    CHECK_THROWS_AS(map.add(TagPlacement{999, Pose2d{Length{1.0}, Length{1.0}, Angle{}},
                                         TagProvenance::Invented, "host test"}),
                    shulib::PreconditionError);
    CHECK(map.size() == TagMap::kMaxTags);
}
