// Contract tests for the vision seams (ITagSource / IVision). No internal logic to
// mutation-check yet — the PnP (corners→pose) and box→bearing reductions are M3/M4
// pure functions, red-teamed then. Here we pin the canonical struct shapes, the
// empty/no-detection default, exact round-trip, and interface polymorphism.

#include "doctest.h"

#include <vector>

#include "shulib/hal/fake/fake_tag_source.hpp"
#include "shulib/hal/fake/fake_vision.hpp"
#include "shulib/hal/vision.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::hal::ITagSource;
using shulib::hal::IVision;
using shulib::hal::ObjectObservation;
using shulib::hal::TagObservation;
using shulib::hal::fake::FakeTagSource;
using shulib::hal::fake::FakeVision;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::Length;

TEST_CASE("FakeTagSource: defaults to no tags; injected tags round-trip") {
    FakeTagSource src;
    CHECK(src.tags().empty());  // no tags visible by default

    const Pose2d rel{Length{12.0}, Length{-3.0}, Angle::degrees(170.0)};
    src.setTags({TagObservation{4, rel, 0.92}});

    const std::vector<TagObservation> got = src.tags();
    REQUIRE(got.size() == 1);
    CHECK(got[0].id == 4);
    CHECK(got[0].poseInRobot.approxEqual(rel));  // relative pose preserved, wrap-aware
    CHECK(got[0].confidence == doctest::Approx(0.92));

    const ITagSource& view = src;  // polymorphic seam
    CHECK(view.tags().size() == 1);
    src.clear();
    CHECK(src.tags().empty());
}

TEST_CASE("FakeTagSource: multiple tags are returned in order") {
    FakeTagSource src;
    src.setTags({
        TagObservation{0, Pose2d{Length{10.0}, Length{0.0}, Angle::degrees(180.0)}, 0.8},
        TagObservation{2, Pose2d{Length{20.0}, Length{5.0}, Angle::degrees(90.0)}, 0.6},
    });
    const std::vector<TagObservation> got = src.tags();
    REQUIRE(got.size() == 2);
    CHECK(got[0].id == 0);
    CHECK(got[1].id == 2);
    CHECK(got[1].poseInRobot.x().value() == doctest::Approx(20.0));
}

TEST_CASE("FakeVision: defaults to nothing; injected objects round-trip (bearing wraps)") {
    FakeVision vis;
    CHECK(vis.objects().empty());

    vis.setObjects({ObjectObservation{7, Angle::degrees(-150.0), 0.75}});
    const std::vector<ObjectObservation> got = vis.objects();
    REQUIRE(got.size() == 1);
    CHECK(got[0].classId == 7);
    CHECK(got[0].bearing.approxEqual(Angle::degrees(-150.0)));  // bearing is a wrapping Angle
    CHECK(got[0].confidence == doctest::Approx(0.75));

    const IVision& view = vis;
    CHECK(view.objects().size() == 1);
}
