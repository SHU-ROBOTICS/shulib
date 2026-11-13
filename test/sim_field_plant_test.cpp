// sim/field_plant.hpp — the host test plant's field and side sensors (chunk L6-A1; the L6 brief's
// test T14). The plant is the ORACLE every wall reset test grades against, so these cases pin it
// against closed forms derived by hand — never against its own output — and against the library's
// season header only where the two state the same drawing (agreement to 0.01 in: the plant keeps its
// own copy of the numbers, the A2 rule, so a typo in either side turns this red — M29).

#include "doctest.h"

#include <cmath>
#include <string>

#include "shulib/localization/override_walls.hpp"
#include "shulib/localization/wall_reset.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/sim/field_plant.hpp"
#include "shulib/units/quantity.hpp"

namespace {

namespace ow = shulib::localization::override2026;
namespace loc = shulib::localization;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::sim::FieldBox;
using shulib::sim::FieldModel;
using shulib::sim::SideSensorPlant;
using shulib::sim::SideSensorPlantConfig;
using shulib::units::Length;

constexpr double kPi = 3.14159265358979323846;
constexpr double kD2R = kPi / 180.0;

/// A field with ONE opaque wall: the plane x = 0 (a box behind it), as tall as `height`.
FieldModel oneWall(double height = 11.5) {
    FieldModel f;
    f.add(FieldBox{-2.0, 0.0, -200.0, 200.0, 0.0, height, false, true, "wall"});
    return f;
}

/// A left-facing sensor at the robot centre (left 0), `h` up, `rays` per axis.
SideSensorPlantConfig centred(double h = 10.5, int rays = 1) {
    SideSensorPlantConfig c{};
    c.forward = 0.0;
    c.left = 0.0;
    c.height = h;
    c.facing = kPi / 2.0;
    c.raysPerAxis = rays;
    return c;
}

/// The robot `d` in from the wall at x = 0, nose along +y, turned `skewDeg` (the beam then `skewDeg`
/// off the wall's normal).
Pose2d facing0(double d, double skewDeg = 0.0) {
    return Pose2d{Length{d}, Length{0.0}, Angle::degrees(90.0 + skewDeg)};
}

}  // namespace

// Bug caught: a ray cast that is not the closed form — the square wall at 8 in reads 8; a beam 10 deg
// off the normal reads 8 / cos 10 deg on the centre ray.
TEST_CASE("L6 T14 the ray cast: a square wall at 8 in reads 8; 10 deg off reads 8 / cos 10 on the centre") {
    const FieldModel f = oneWall();
    SideSensorPlant p{f, centred()};
    CHECK(p.rangeAt(facing0(8.0)) == doctest::Approx(8.0).epsilon(1e-12));
    CHECK(p.rangeAt(facing0(8.0, 10.0)) == doctest::Approx(8.0 / std::cos(10.0 * kD2R)).epsilon(1e-12));
    CHECK(p.rangeAt(facing0(8.0, -10.0)) == doctest::Approx(8.0 / std::cos(10.0 * kD2R)).epsilon(1e-12));
    // The raw cast, too: from (5, 0, 3) along -x hits x = 0 at 5; straight down from 3 in hits the
    // floor at 3.
    const auto h = f.cast(5.0, 0.0, 3.0, -1.0, 0.0, 0.0, 100.0);
    CHECK(h.hit);
    CHECK(h.range == doctest::Approx(5.0).epsilon(1e-12));
    CHECK(std::string{h.box->name} == "wall");
    const auto fl = f.cast(5.0, 0.0, 3.0, 0.0, 0.0, -1.0, 100.0);
    CHECK(fl.hit);
    CHECK(fl.box == nullptr);
    CHECK(fl.range == doctest::Approx(3.0).epsilon(1e-12));
}

// Bug caught: a cone that does not take the NEAREST surface (HA-205's belief) — a Pin standing 3 in
// in front of the wall, low, enters the cone's lower rays only: the nearest-hit sensor reads near it
// while the centre ray alone reads the wall; in BLEND mode the reading is the hits' mean (between).
TEST_CASE("L6 T14 the cone picks the nearest hit; blend mode the mean of the hits") {
    FieldModel f = oneWall();
    f.add(FieldBox{4.2, 5.8, -0.8, 0.8, 0.0, 6.5, false, false, "Pin"});  // 4.2..5.8 in from the wall
    SideSensorPlant centre{f, centred(8.0, 1)};
    CHECK(centre.rangeAt(facing0(10.0)) == doctest::Approx(10.0).epsilon(1e-12));  // over the Pin
    SideSensorPlant cone{f, centred(8.0, 7)};
    const double nearest = cone.rangeAt(facing0(10.0));
    CHECK(nearest < 6.5);  // a lower ray dips onto the Pin's top (6.5 in) before the wall
    CHECK(nearest > 4.2);
    SideSensorPlantConfig bc = centred(8.0, 7);
    bc.blend = true;
    SideSensorPlant blend{f, bc};
    const double mean = blend.rangeAt(facing0(10.0));
    CHECK(mean > nearest);
    CHECK(mean < 10.0 / std::cos(18.0 * kD2R) + 1e-9);
    // With the Pin gone, both modes read a square wall's nearest ray: exactly the perpendicular.
    const FieldModel bare = oneWall();
    SideSensorPlant cone2{bare, centred(8.0, 7)};
    CHECK(cone2.rangeAt(facing0(10.0)) == doctest::Approx(10.0).epsilon(1e-12));
}

// Bug caught: see-through boxes that block (a clear wall panel must pass the beam to whatever is
// behind it — HA-206's contrary evidence), and nothing within range publishing anything but 9999.
TEST_CASE("L6 T14 see-through boxes pass the beam; nothing in range publishes 9999 with confidence 0") {
    FieldModel f = oneWall();
    f.seeThroughWall("wall");
    SideSensorPlant p{f, centred(10.5, 1)};  // the centre ray: horizontal, so the floor is not in it
    p.step(facing0(8.0));
    CHECK(p.sensor().confidence() == 0.0);
    CHECK(p.sensor().distance().value() == doctest::Approx(9999.0 / 25.4).epsilon(1e-12));
    f.add(FieldBox{-4.0, -3.0, -10.0, 10.0, 0.0, 30.0, false, false, "hand behind the panel"});
    SideSensorPlant q{f, centred(10.5, 1)};
    CHECK(q.rangeAt(facing0(8.0)) == doctest::Approx(11.0).epsilon(1e-12));
    // Past 78.74 in (2000 mm) nothing is seen.
    const FieldModel w = oneWall();
    SideSensorPlant far{w, centred(10.5, 1)};
    far.step(facing0(80.0));
    CHECK(far.sensor().confidence() == 0.0);
    far.step(facing0(78.0));
    CHECK(far.sensor().confidence() == 1.0);
    CHECK(far.sensor().distance().value() == doctest::Approx(78.0).epsilon(1e-12));
}

// Bug caught: an update interval or a latency that is not what its config says (HA-209's two
// unknowns): updateEvery 3 holds each measurement three steps; latency 2 publishes two steps late.
TEST_CASE("L6 T14 the update hold and the latency delay") {
    const FieldModel f = oneWall();
    SideSensorPlantConfig c = centred();
    c.updateEvery = 3;
    SideSensorPlant hold{f, c};
    const double d[6] = {8.0, 8.5, 9.0, 9.5, 10.0, 10.5};
    double seen[6] = {};
    for (int i = 0; i < 6; ++i) {
        hold.step(facing0(d[i]));
        seen[i] = hold.sensor().distance().value();
    }
    CHECK(seen[0] == doctest::Approx(8.0));
    CHECK(seen[1] == doctest::Approx(8.0));
    CHECK(seen[2] == doctest::Approx(8.0));
    CHECK(seen[3] == doctest::Approx(9.5));
    CHECK(seen[5] == doctest::Approx(9.5));
    SideSensorPlantConfig l = centred();
    l.latencySteps = 2;
    SideSensorPlant lag{f, l};
    for (int i = 0; i < 6; ++i) {
        lag.step(facing0(d[i]));
        seen[i] = lag.sensor().distance().value();
    }
    CHECK(seen[0] == doctest::Approx(8.0));  // the line fills with the first measurement
    CHECK(seen[2] == doctest::Approx(8.0));
    CHECK(seen[3] == doctest::Approx(8.5));
    CHECK(seen[5] == doctest::Approx(9.5));
    // FROZEN: the published value stops changing.
    SideSensorPlant fr{f, centred()};
    fr.step(facing0(8.0));
    fr.freeze(true);
    fr.step(facing0(12.0));
    CHECK(fr.sensor().distance().value() == doctest::Approx(8.0));
    // A read fault: ProsDistance's PROS_ERR behaviour — the last good distance, confidence 0, the
    // count moving at every read.
    SideSensorPlant rf{f, centred()};
    rf.step(facing0(8.0));
    rf.fault(true);
    rf.step(facing0(12.0));
    CHECK(rf.sensor().distance().value() == doctest::Approx(8.0));
    CHECK(rf.sensor().confidence() == 0.0);
    const int c0 = rf.faults().faultedReads();
    CHECK(rf.faults().faultedReads() == c0 + 1);
}

// Bug caught: a plant whose noise is not reproducible from its seed (every wall reset test would
// then be a different test each run).
TEST_CASE("L6 T14 seeded determinism: the same seed, the same readings; another seed, others") {
    const FieldModel f = oneWall();
    SideSensorPlantConfig c = centred(10.5, 7);
    c.noiseSigma = 0.1;
    c.seed = 7;
    SideSensorPlant a{f, c};
    SideSensorPlant b{f, c};
    c.seed = 8;
    SideSensorPlant other{f, c};
    bool differs = false;
    for (int i = 0; i < 50; ++i) {
        a.step(facing0(8.0));
        b.step(facing0(8.0));
        other.step(facing0(8.0));
        CHECK(a.sensor().distance().value() == b.sensor().distance().value());
        differs = differs || a.sensor().distance().value() != other.sensor().distance().value();
    }
    CHECK(differs);
}

// Bug caught (RC35): a code-strip model that biases rays outside its band (9.06-11.44 in). A lens at
// 10 in with a horizontal centre ray hits the band; one at 8 in does not.
TEST_CASE("L6 T14 the code strip biases only the rays that hit its band") {
    const FieldModel f = oneWall();
    SideSensorPlantConfig in = centred(10.0, 1);
    in.strip.whiteBias = 0.4;
    in.strip.blackBias = 0.4;
    in.strip.whiteConfidence = 0.5;
    in.strip.blackConfidence = 0.5;
    SideSensorPlant onStrip{f, in};
    onStrip.step(facing0(8.0));
    CHECK(onStrip.sensor().distance().value() == doctest::Approx(8.4).epsilon(1e-12));
    CHECK(onStrip.sensor().confidence() == doctest::Approx(0.5).epsilon(1e-12));
    SideSensorPlantConfig below = in;
    below.height = 8.0;
    SideSensorPlant offStrip{f, below};
    offStrip.step(facing0(8.0));
    CHECK(offStrip.sensor().distance().value() == doctest::Approx(8.0).epsilon(1e-12));
    CHECK(offStrip.sensor().confidence() == 1.0);
    // Black and white differ when told to: the square under the beam decides.
    SideSensorPlantConfig bw = centred(10.0, 1);
    bw.strip.blackBias = 0.2;
    bw.strip.squareSize = 2.0;
    SideSensorPlant s{f, bw};
    // along 0.5 -> square 0 (white); along 2.5 -> square 1 (black)
    CHECK(s.rangeAt(Pose2d{Length{8.0}, Length{0.5}, Angle::degrees(90.0)}) == doctest::Approx(8.0));
    CHECK(s.rangeAt(Pose2d{Length{8.0}, Length{2.5}, Angle::degrees(90.0)}) == doctest::Approx(8.2));
}

// Bug caught (M29): the plant's field and the library's season walls disagreeing about the drawing —
// a wall at 70.0 instead of 70.205, a Loader or a start cluster moved. They agree to 0.01 in.
TEST_CASE("L6 T14 the plant's Override field agrees with override_walls.hpp to 0.01 in") {
    CHECK(std::abs(FieldModel::kInnerFace - ow::kWallInnerFace.value()) < 0.01);
    // The plant's walls, measured by a centre ray from the field's middle.
    const FieldModel f = FieldModel::override2026();
    for (const loc::FieldWall& w : ow::kWalls) {
        const double dx = w.axis == loc::Axis::X ? -w.inward : 0.0;
        const double dy = w.axis == loc::Axis::Y ? -w.inward : 0.0;
        // From (0, 40) or (40, 0): off the Toggles, the clusters and the Loaders.
        const double ox = w.axis == loc::Axis::X ? 0.0 : 40.0;
        const double oy = w.axis == loc::Axis::X ? 40.0 : 0.0;
        const auto hit = f.cast(ox, oy, 10.0, dx, dy, 0.0, 200.0);
        REQUIRE(hit.hit);
        INFO(w.name);
        CHECK(std::abs(hit.range - std::abs(w.coord.value())) < 0.01);
    }
    // The Loaders, the Toggle and the start pieces: the library's spans contain the plant's boxes.
    double loaderLo = 0.0;
    double loaderHi = 0.0;
    double pieceOuter = 0.0;
    double pieceInner = 1e9;
    for (const FieldBox& b : f.boxes()) {
        const std::string n = b.name;
        if (n == "Loader" && b.yMin < -60.0 && b.xMin > 0.0) {  // the red wall's +x Loader
            loaderLo = b.xMin;
            loaderHi = b.xMax;
        }
        if ((n == "start Cup" || n == "start Pin") && b.xMin < -60.0 && b.yMin > 0.0) {  // left wall, +y
            pieceOuter = std::max(pieceOuter, b.yMax);
            pieceInner = std::min(pieceInner, b.yMin);
        }
    }
    bool loaderFound = false;
    bool piecesFound = false;
    for (const loc::WallKeepOut& k : ow::kRedWall.keepOuts()) {
        if (std::string{k.why} == "Loader" && k.from.value() > 0.0) {
            loaderFound = true;
            CHECK(std::abs(k.from.value() - loaderLo) < 0.01);
            CHECK(std::abs(k.to.value() - loaderHi) < 0.01);
        }
    }
    for (const loc::WallKeepOut& k : ow::kLeftWall.keepOuts()) {
        if (std::string{k.why} == "start pieces" && k.from.value() > 0.0) {
            piecesFound = true;
            CHECK(std::abs(k.from.value() - pieceInner) < 0.01);
            CHECK(std::abs(k.to.value() - pieceOuter) < 0.01);
            CHECK(k.kind == loc::KeepOutKind::PieceZone);
        }
        if (std::string{k.why} == "Toggle bracket" && k.from.value() > 0.0) {
            CHECK(std::abs(k.from.value() - FieldModel::kToggleBodyHalf) < 0.01);
            CHECK(std::abs(k.to.value() - FieldModel::kToggleLength / 2.0) < 0.01);
        }
    }
    CHECK(loaderFound);
    CHECK(piecesFound);
    // The left and right walls carry no Loader; the red and blue walls two each.
    const auto count = [](const loc::FieldWall& w, const char* why) {
        int n = 0;
        for (const loc::WallKeepOut& k : w.keepOuts()) {
            n += std::string{k.why} == why ? 1 : 0;
        }
        return n;
    };
    CHECK(count(ow::kLeftWall, "Loader") == 0);
    CHECK(count(ow::kRightWall, "Loader") == 0);
    CHECK(count(ow::kRedWall, "Loader") == 2);
    CHECK(count(ow::kBlueWall, "Loader") == 2);
    CHECK(count(ow::kLeftWall, "Load Zone") == 2);
    CHECK(count(ow::kLeftWall, "corner") == 2);
}
