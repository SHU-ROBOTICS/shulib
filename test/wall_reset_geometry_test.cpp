// The wall reset's GEOMETRY (chunk L6-A1; the L6 brief's tests T1, T2 and T11): the inversion that
// turns one side sensor's reading into the robot centre's coordinate, the worst-case error bound
// around it, the heading tolerance that grows with turning, and an author's own wall line.
//
// THE ORACLE is the host field plant (sim/field_plant.hpp) — its own ray cast against its own copy
// of the field, from the TRUE pose and the TRUE mount — never the library's own geometry (the A2
// rule: a sign error shared by the code and its oracle would cancel and be invisible).

#include "doctest.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "shulib/localization/axis.hpp"
#include "shulib/localization/override_walls.hpp"
#include "shulib/localization/wall_reset.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/sim/field_plant.hpp"
#include "shulib/sim/rng.hpp"
#include "shulib/units/quantity.hpp"
#include "shulib/vision/override_elements.hpp"

namespace {

namespace loc = shulib::localization;
namespace ow = shulib::localization::override2026;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::sim::FieldModel;
using shulib::sim::SideSensorPlant;
using shulib::sim::SideSensorPlantConfig;
using shulib::units::AngleDim;
using shulib::units::Length;

constexpr double kPi = 3.14159265358979323846;
constexpr double kD2R = kPi / 180.0;

/// The beam's field direction that points AT a wall (its outward normal).
double beamAt(const loc::FieldWall& w) {
    if (w.axis == loc::Axis::X) {
        return w.inward > 0 ? kPi : 0.0;
    }
    return w.inward > 0 ? -kPi / 2.0 : kPi / 2.0;
}

/// A robot whose lens sits `d` in from `w` at `along` along it, facing so the beam is `skew` off the
/// normal — built from the TRUE mount, in plain trigonometry (independent of the library).
Pose2d placeFor(const loc::FieldWall& w, double forward, double left, double facing, double skew, double d,
                double along) {
    const double theta = beamAt(w) - facing + skew;
    const double lensAxis = w.coord.value() + w.inward * d;
    const double lx = w.axis == loc::Axis::X ? lensAxis : along;
    const double ly = w.axis == loc::Axis::X ? along : lensAxis;
    const double rx = lx - (forward * std::cos(theta) - left * std::sin(theta));
    const double ry = ly - (forward * std::sin(theta) + left * std::cos(theta));
    return Pose2d{Length{rx}, Length{ry}, Angle::radians(theta)};
}

double on(const Pose2d& p, loc::Axis a) { return a == loc::Axis::X ? p.x().value() : p.y().value(); }

loc::SideMount believed(double forward, double left, double facing, double bias) {
    loc::SideMount m{};
    m.measured = true;
    m.forward = Length{forward};
    m.left = Length{left};
    m.height = Length{10.5};
    m.facing = AngleDim{facing};
    m.rangeBias = Length{bias};
    return m;
}

}  // namespace

// Bug caught (T1): any sign or term wrong in the inversion — a wall's inward sign ignored (M1: the
// right and blue walls' beams then read as facing away and their expected range flips), the range
// used without its cosine (M2: a skewed beam's slant range taken as the perpendicular), the lever arm
// rotated by -heading (M3), the bias added instead of subtracted (M4). Every wall x every side x
// forward offset {0, 5} x skew {0, +/-4.9 deg} x bias {0, 0.3}: a NOISELESS ray-centre sensor on the
// field plant, and the library's answer equals the TRUE coordinate to 1e-9 in.
TEST_CASE("L6 T1 the inversion is exact: every wall, side, offset, skew and bias") {
    FieldModel field = FieldModel::override2026();
    field.removeNamed("start");
    int cases = 0;
    for (const loc::FieldWall& w : ow::kWalls) {
        for (const loc::WallSide side : {loc::WallSide::Left, loc::WallSide::Right}) {
            const double left = side == loc::WallSide::Left ? 7.5 : -7.5;
            const double facing = side == loc::WallSide::Left ? kPi / 2.0 : -kPi / 2.0;
            for (const double fwd : {0.0, 5.0}) {
                for (const double skewDeg : {0.0, 4.9, -4.9}) {
                    for (const double bias : {0.0, 0.3}) {
                        SideSensorPlantConfig cfg{};
                        cfg.forward = fwd;
                        cfg.left = left;
                        cfg.height = 10.5;
                        cfg.facing = facing;
                        cfg.raysPerAxis = 1;  // the ray-centre sensor
                        cfg.bias = bias;
                        SideSensorPlant plant{field, cfg};
                        const Pose2d truth = placeFor(w, fwd, left, facing, skewDeg * kD2R, 8.0, -38.0);
                        const double reading = plant.rangeAt(truth);
                        REQUIRE(reading < 20.0);  // the wall, not 9999
                        const loc::SideMount m = believed(fwd, left, facing, bias);
                        INFO(w.name << " " << loc::wallSideWord(side) << " f " << fwd << " skew " << skewDeg
                                    << " bias " << bias);
                        const double got = loc::axisFromReading(Length{reading}, truth, m, w).value();
                        CHECK(std::abs(got - on(truth, w.axis)) < 1e-9);
                        // The aim by the estimate (here = truth): the expected range IS the
                        // plant's bias-free reading, the beam is |skew| off square, and it lands
                        // at the plant's along coordinate.
                        const loc::WallAim aim =
                            loc::aimAt(truth, m, w, loc::WallResetLimits{}, Length{0.0});
                        CHECK(std::abs(aim.expectedRange.value() - (reading - bias)) < 1e-9);
                        CHECK(std::abs(aim.offSquare.value() - std::abs(skewDeg) * kD2R) < 1e-12);
                        ++cases;
                    }
                }
            }
        }
    }
    CHECK(cases == 4 * 2 * 2 * 3 * 2);
}

// Bug caught (T1, the aim's along coordinate — the keep-outs are tested there): a beam centre placed
// at the LENS's along coordinate instead of where the skewed beam lands. Against the plant: the hit
// point's along coordinate of the centre ray.
TEST_CASE("L6 T1 the aim's along coordinate is where the centre ray hits the wall") {
    for (const loc::FieldWall& w : ow::kWalls) {
        for (const double skewDeg : {0.0, 4.9, -4.9}) {
            const double facing = kPi / 2.0;
            const Pose2d truth = placeFor(w, 0.0, 7.5, facing, skewDeg * kD2R, 8.0, -38.0);
            const loc::SideMount m = believed(0.0, 7.5, facing, 0.0);
            const loc::WallAim aim = loc::aimAt(truth, m, w, loc::WallResetLimits{}, Length{0.0});
            // The lens sits at along -38, 8 in from the wall; the beam drifts 8 tan(skew) along.
            const double beam = beamAt(w) + skewDeg * kD2R;
            const double alongDir = w.axis == loc::Axis::X ? std::sin(beam) : std::cos(beam);
            const double perpDir = std::abs(w.axis == loc::Axis::X ? std::cos(beam) : std::sin(beam));
            const double expected = -38.0 + 8.0 * alongDir / perpDir;
            INFO(w.name << " skew " << skewDeg);
            CHECK(std::abs(aim.along.value() - expected) < 1e-9);
        }
    }
}

// Bug caught (T2): an error bound that does not bound — the mount-position term dropped (M5), the
// accuracy a 200-mm STEP instead of max(15 mm, 5 %) (M6: a cliff in the gate), a measured accuracy
// ignored (M6b), the heading tolerance stuck at its floor (M6c). 20 000 seeded draws inside the
// stated tolerances; the TRUE error never exceeds the bound, and the bound is not loose.
TEST_CASE("L6 T2 the error bound bounds: 20 000 draws inside the tolerances") {
    FieldModel field = FieldModel::override2026();
    field.removeNamed("start");
    const loc::FieldWall& w = ow::kLeftWall;
    const loc::WallResetLimits limits{};
    shulib::sim::Rng rng{20261029};
    for (const double fwd : {0.0, 5.0}) {
        for (const double measuredAcc : {0.0, 0.3}) {
            for (const double turnedDeg : {0.0, 180.0}) {
                double worstRatio = 0.0;
                double worstErr = 0.0;
                int over = 0;
                for (int k = 0; k < 2500; ++k) {
                    const double d = rng.uniform(2.5, 14.0);
                    const double estHeadingSkew = rng.uniform(-4.0, 4.0) * kD2R;
                    loc::SideMount m = believed(fwd, 7.5, kPi / 2.0, 0.0);
                    m.rangeAccuracy = Length{measuredAcc};
                    const double pt = m.positionTolerance.value();
                    const double ft = m.facingTolerance.value();
                    const AngleDim htol = loc::headingTolerance(AngleDim{turnedDeg * kD2R}, limits);
                    const double ht = htol.value();
                    // The TRUE mount and heading, inside the tolerances.
                    const double tf = fwd + rng.uniform(-pt, pt);
                    const double tl = 7.5 + rng.uniform(-pt, pt);
                    const double tphi = kPi / 2.0 + rng.uniform(-ft, ft);
                    const double dh = rng.uniform(-ht, ht);
                    // The estimate's heading; the truth is off it by dh.
                    const Pose2d truth = placeFor(w, tf, tl, tphi, estHeadingSkew + dh, d, -38.0);
                    SideSensorPlantConfig cfg{};
                    cfg.forward = tf;
                    cfg.left = tl;
                    cfg.facing = tphi;
                    cfg.raysPerAxis = 1;
                    SideSensorPlant plant{field, cfg};
                    const double trueRange = plant.rangeAt(truth);
                    // The sensor's error, inside the accuracy AT THE READING.
                    double reading = trueRange;
                    for (int tries = 0; tries < 20; ++tries) {
                        const double acc0 = loc::readingAccuracy(Length{trueRange}, m).value();
                        const double e = rng.uniform(-acc0, acc0);
                        if (std::abs(e) <= loc::readingAccuracy(Length{trueRange + e}, m).value()) {
                            reading = trueRange + e;
                            break;
                        }
                    }
                    const Pose2d est{truth.x(), truth.y(), Angle::radians(truth.heading().radians() - dh)};
                    const double answer = loc::axisFromReading(Length{reading}, est, m, w).value();
                    const double err = std::abs(answer - truth.x().value());
                    const double bound = loc::axisErrorBound(Length{reading}, est, m, w, htol).value();
                    if (err > bound + 1e-12) {
                        ++over;
                    }
                    worstRatio = std::max(worstRatio, err / bound);
                    worstErr = std::max(worstErr, err);
                }
                INFO("f " << fwd << " measured acc " << measuredAcc << " turned " << turnedDeg);
                CHECK(over == 0);
                CHECK(worstRatio >= 0.5);  // the bound is not loose
                CHECK(worstErr > 0.0);
            }
        }
    }
}

// Bug caught (the L6-A1 review's S7, S8, S11 — terms too small for T2's random draws to see): a
// tolerance dropped from the bound (the facing's; the FORWARD position's), or the accuracy taken at
// the RAW reading instead of the bias-corrected range. The bound is a worst case over CORNERS run
// through the exact geometry, so it is ATTAINED: with ONE tolerance at a time (the others zero, the
// accuracy a measured 1e-12), the plant's TRUE mount, heading or reading error placed at each corner,
// the true error never exceeds the bound at that reading and equals it, to 1e-9, at the worst corner.
// The heading is 4 deg off square, so the forward offset's term is not zero.
TEST_CASE("L6 T2 the error bound is attained at a corner, one tolerance at a time") {
    FieldModel field = FieldModel::override2026();
    field.removeNamed("start");
    const loc::FieldWall& w = ow::kLeftWall;
    const double fwd = 2.0;
    const double left = 7.5;
    const double facing = kPi / 2.0;
    const double skew = 4.0 * kD2R;
    const double d = 7.0;
    const auto base = [&] {
        loc::SideMount m = believed(fwd, left, facing, 0.0);
        m.positionTolerance = Length{0.0};
        m.facingTolerance = AngleDim{0.0};
        m.rangeAccuracy = Length{1e-12};
        return m;
    };
    // One corner: the TRUE mount / heading (and bias) on the plant; the reading the plant gives
    // (plus a sensor error `e`); returns {true error, bound at that reading}.
    struct Got {
        double err;
        double bound;
    };
    const auto corner = [&](const loc::SideMount& m, double tf, double tl, double tphi, double dh, double bias,
                            double e, AngleDim htol) {
        SideSensorPlantConfig cfg{};
        cfg.forward = tf;
        cfg.left = tl;
        cfg.facing = tphi;
        cfg.raysPerAxis = 1;
        cfg.bias = bias;
        SideSensorPlant plant{field, cfg};
        const Pose2d truth = placeFor(w, tf, tl, tphi, skew + dh, d, -38.0);
        const double reading = plant.rangeAt(truth) + e;
        const Pose2d est{truth.x(), truth.y(), Angle::radians(truth.heading().radians() - dh)};
        const double answer = loc::axisFromReading(Length{reading}, est, m, w).value();
        return Got{std::abs(answer - truth.x().value()),
                   loc::axisErrorBound(Length{reading}, est, m, w, htol).value()};
    };
    const auto attained = [](std::initializer_list<Got> corners) {
        double worstGap = 1e9;
        bool within = true;
        for (const Got& g : corners) {
            within = within && g.err <= g.bound + 1e-9;
            worstGap = std::min(worstGap, std::abs(g.bound - g.err));
        }
        CHECK(within);
        CHECK(worstGap < 1e-9);
    };
    const AngleDim none{0.0};
    SUBCASE("facing (S7)") {
        loc::SideMount m = base();
        m.facingTolerance = AngleDim{2.0 * kD2R};
        const double ft = m.facingTolerance.value();
        attained({corner(m, fwd, left, facing + ft, 0.0, 0.0, 0.0, none),
                  corner(m, fwd, left, facing - ft, 0.0, 0.0, 0.0, none)});
    }
    SUBCASE("position, forward and left (S8, M5)") {
        loc::SideMount m = base();
        m.positionTolerance = Length{0.25};
        const double pt = 0.25;
        attained({corner(m, fwd + pt, left + pt, facing, 0.0, 0.0, 0.0, none),
                  corner(m, fwd + pt, left - pt, facing, 0.0, 0.0, 0.0, none),
                  corner(m, fwd - pt, left + pt, facing, 0.0, 0.0, 0.0, none),
                  corner(m, fwd - pt, left - pt, facing, 0.0, 0.0, 0.0, none)});
        // The forward term is really in it: the bound exceeds the left term alone.
        const Got g = corner(m, fwd, left, facing, 0.0, 0.0, 0.0, none);
        CHECK(g.bound > pt * std::abs(std::sin(kPi / 2.0 + skew)) + 0.01);
    }
    SUBCASE("heading (M6c's term)") {
        const loc::SideMount m = base();
        const AngleDim ht{3.0 * kD2R};
        attained({corner(m, fwd, left, facing, +ht.value(), 0.0, 0.0, ht),
                  corner(m, fwd, left, facing, -ht.value(), 0.0, 0.0, ht)});
    }
    SUBCASE("the published accuracy at the BIAS-CORRECTED range (S11)") {
        loc::SideMount m = base();
        m.rangeAccuracy = Length{0.0};  // VEX's published max(15 mm, 5 %)
        m.rangeBias = Length{1.0};
        SideSensorPlantConfig cfg{};
        cfg.forward = fwd;
        cfg.left = left;
        cfg.raysPerAxis = 1;
        SideSensorPlant plant{field, cfg};
        const double trueRange = plant.rangeAt(placeFor(w, fwd, left, facing, skew, 12.5, -38.0));
        REQUIRE(0.05 * trueRange > loc::kPublishedCloseAccuracyInches);  // the 5 % regime
        // e = 5 % of (trueRange + e) — the reading's own corrected range — one way, then the other.
        const double up = 0.05 * trueRange / 0.95;
        const double down = -0.05 * trueRange / 1.05;
        for (const double e : {up, down}) {
            SideSensorPlantConfig c2 = cfg;
            c2.bias = 1.0;
            SideSensorPlant biased{field, c2};
            const Pose2d truth = placeFor(w, fwd, left, facing, skew, 12.5, -38.0);
            const double reading = biased.rangeAt(truth) + e;
            const double answer = loc::axisFromReading(Length{reading}, truth, m, w).value();
            const double err = std::abs(answer - truth.x().value());
            const double bound = loc::axisErrorBound(Length{reading}, truth, m, w, none).value();
            INFO("e " << e);
            CHECK(std::abs(bound - err) < 1e-9);
        }
    }
}

// Bug caught (T2, M6): the accuracy model as the 200-mm step — the bound would drop by 0.13-0.20 in
// across 7.87 in (a cliff in the precision gate). The assumed accuracy is never below max(15 mm, 5 %)
// and the bound is continuous in the reading; a measured accuracy replaces it (M6b).
TEST_CASE("L6 T2 the accuracy is max(15 mm, 5 %) everywhere, never a step; a measured one replaces it") {
    const loc::FieldWall& w = ow::kLeftWall;
    loc::SideMount m = believed(0.0, 7.5, kPi / 2.0, 0.0);
    const Pose2d est{Length{-55.0}, Length{-38.0}, Angle::degrees(90.0)};
    const AngleDim htol = loc::headingTolerance(AngleDim{0.0}, loc::WallResetLimits{});
    double prev = loc::axisErrorBound(Length{2.0}, est, m, w, htol).value();
    for (int i = 1; i <= 1400; ++i) {
        const double r = 2.0 + 0.01 * i;
        const double acc = loc::readingAccuracy(Length{r}, m).value();
        CHECK(acc >= std::max(15.0 / 25.4, 0.05 * r) - 1e-15);
        CHECK(acc <= std::max(15.0 / 25.4, 0.05 * r) + 1e-15);
        const double b = loc::axisErrorBound(Length{r}, est, m, w, htol).value();
        CHECK(std::abs(b - prev) < 0.01);  // continuous: no cliff anywhere in 2..16 in
        prev = b;
    }
    // At exactly 200 mm the two published figures meet.
    CHECK(loc::readingAccuracy(Length{200.0 / 25.4}, m).value()
          == doctest::Approx(15.0 / 25.4).epsilon(1e-12));
    // A measured accuracy (Step 0.3) replaces the published one at every range (M6b).
    loc::SideMount measured = m;
    measured.rangeAccuracy = Length{0.3};
    CHECK(loc::readingAccuracy(Length{10.0}, measured).value() == 0.3);
    CHECK(loc::readingAccuracy(Length{3.0}, measured).value() == 0.3);
    CHECK(loc::axisErrorBound(Length{8.0}, est, measured, w, htol).value()
          < loc::axisErrorBound(Length{8.0}, est, m, w, htol).value() - 0.25);
}

// Bug caught (T2, M6c): a heading tolerance that ignores the turning (a robot that turned 450 deg
// seats as if it had not). 2 deg + 2 % of the rotation: 2 deg at 0, 5.6 deg at 180, 11 deg at 450.
TEST_CASE("L6 T2 the heading tolerance grows with the rotation since the heading seed") {
    const loc::WallResetLimits limits{};
    CHECK(loc::headingTolerance(AngleDim{0.0}, limits).value() == doctest::Approx(2.0 * kD2R).epsilon(1e-12));
    CHECK(loc::headingTolerance(AngleDim{180.0 * kD2R}, limits).value()
          == doctest::Approx(5.6 * kD2R).epsilon(1e-12));
    CHECK(loc::headingTolerance(AngleDim{450.0 * kD2R}, limits).value()
          == doctest::Approx(11.0 * kD2R).epsilon(1e-12));
    CHECK(loc::headingTolerance(AngleDim{-450.0 * kD2R}, limits).value()
          == doctest::Approx(11.0 * kD2R).epsilon(1e-12));
    // And it reaches the bound: after 450 deg the same reading's bound is larger.
    const loc::SideMount m = believed(0.0, 7.5, kPi / 2.0, 0.0);
    const Pose2d est{Length{-55.0}, Length{-38.0}, Angle::degrees(90.0)};
    CHECK(loc::axisErrorBound(Length{8.0}, est, m, ow::kLeftWall,
                              loc::headingTolerance(AngleDim{450.0 * kD2R}, limits))
              .value()
          > loc::axisErrorBound(Length{8.0}, est, m, ow::kLeftWall,
                                loc::headingTolerance(AngleDim{0.0}, limits))
                    .value());
}

// Bug caught (T11, RC48): an author's OWN line (frame Own) that does not behave like the season wall
// it stands for — the inward sign ignored (M1 again: the rotated line's field is on its -y side), or a
// line's keep-outs not applied as given, or applied when none were given. A frame rotated -90 deg
// from F1: (x, y) -> (y, -x); the left wall x = -70.205 becomes the line y' = +70.205, inward -1.
TEST_CASE("L6 T11 an own wall line in a rotated frame gives the same seat as the season wall") {
    FieldModel field = FieldModel::override2026();
    field.removeNamed("start");
    SideSensorPlantConfig cfg{};
    cfg.raysPerAxis = 1;
    SideSensorPlant plant{field, cfg};
    const loc::SideMount m = believed(0.0, 7.5, kPi / 2.0, 0.0);
    for (const double skewDeg : {0.0, 3.0, -3.0}) {
        const Pose2d truth = placeFor(ow::kLeftWall, 0.0, 7.5, kPi / 2.0, skewDeg * kD2R, 7.0, -38.0);
        const double reading = plant.rangeAt(truth);
        const double f1 = loc::axisFromReading(Length{reading}, truth, m, ow::kLeftWall).value();
        // The same robot in the rotated frame.
        const Pose2d rot{truth.y(), Length{-truth.x().value()},
                         Angle::radians(truth.heading().radians() - kPi / 2.0)};
        const loc::FieldWall own =
            loc::FieldWall::line("own line", loc::Axis::Y, Length{ow::kWallInnerFace.value()}, -1);
        CHECK(own.frame == loc::WallFrame::Own);
        CHECK(own.keepOutCount == 0);
        const double y2 = loc::axisFromReading(Length{reading}, rot, m, own).value();
        CHECK(std::abs(-y2 - f1) < 1e-9);  // y' = -x
        // The pre-checks agree: square and, with no keep-outs, nothing kept out.
        loc::WallResetRequest req{};
        req.wall = own;
        req.maxCorrection = Length{3.0};
        const loc::WallResetLimits limits{};
        CHECK(loc::preCheck(rot, m, req, limits, Length{0.0}) == loc::WallResetVerdict::Seated);
        // Its own keep-out, where the beam lands (along x' = y = -38), applies as given.
        const std::array<loc::WallKeepOut, 1> spans{
            loc::WallKeepOut{Length{-40.0}, Length{-36.0}, loc::KeepOutKind::Fixed, "my bracket"}};
        req.wall = loc::FieldWall::line("own line", loc::Axis::Y, Length{ow::kWallInnerFace.value()}, -1,
                                        spans);
        CHECK(loc::preCheck(rot, m, req, limits, Length{0.0}) == loc::WallResetVerdict::KeepOut);
        const loc::WallAim aim = loc::aimAt(rot, m, req.wall, limits, Length{0.0});
        CHECK(std::string{aim.keepOutWhy} == "my bracket");
    }
}

// ═══ The season's facts in two headers (the L6 brief's §6.4) ══════════════════════════════════
// Bug caught: the camera's season data (vision/override_elements.hpp — VIS1) and the wall reset's
// (localization/override_walls.hpp — L6) drifting apart where they state the same fact: where each
// Toggle sits, and how long it is. Neither header includes the other (L6 must not depend on VIS1), so
// this test — written by the first L6 run built on a tree that has both (L6-A2) — includes both. Each
// Toggle's centre must agree within 0.01 in (VIS1 types 70.2, L6 computes 70.205 from A10's mm), each
// wall's side of the field must agree, and L6's Toggle keep-out (body and brackets) must cover VIS1's
// whole Toggle length (25.8 in without its caps; L6 keeps out A8's 25.99 with them).
TEST_CASE("L6 §6.4 the two season headers agree: every Toggle's place within 0.01 in, the field's side, and "
          "the keep-out covers the camera's Toggle") {
    namespace vov = shulib::vision::override2026;
    const double vHalf = vov::kToggleFace.lengthIn / 2.0;
    for (const vov::TogglePlace& t : vov::kTogglePlaces) {
        CAPTURE(t.x);
        CAPTURE(t.y);
        int matched = 0;
        for (const loc::FieldWall& w : loc::override2026::kWalls) {
            const bool onX = w.axis == loc::Axis::X;
            const double face = onX ? t.x : t.y;
            const double along = onX ? t.y : t.x;
            const double in = onX ? t.inX : t.inY;
            if (std::abs(in) < 0.5 || std::abs(face - w.coord.value()) > 1.0) {
                continue;  // not this wall
            }
            ++matched;
            CHECK(std::abs(face - w.coord.value()) <= 0.01);
            CHECK(along == 0.0);  // centred on its wall
            CHECK(static_cast<double>(w.inward) == in);
            // L6's Toggle spans (the body and both brackets) cover the camera's Toggle WITH NO GAP: swept
            // in order from its near end, each span must start at or before where the last one reached
            // (the brackets adjoin the body exactly — an equality the sweep accepts), and the sweep
            // must reach its far end. A min/max over the spans would pass a body shrunk inside its
            // brackets, leaving a stretch of Toggle face no keep-out names.
            std::vector<std::pair<double, double>> spans;
            for (const loc::WallKeepOut& k : w.keepOuts()) {
                const std::string why{k.why};
                if (why == "Toggle" || why == "Toggle bracket") {
                    spans.emplace_back(k.from.value(), k.to.value());
                }
            }
            REQUIRE(spans.size() == 3);
            std::sort(spans.begin(), spans.end());
            double reach = -vHalf;
            bool gap = false;
            for (const auto& [from, to] : spans) {
                if (to <= reach) {
                    continue;
                }
                if (from > reach) {
                    gap = true;
                    CAPTURE(reach);
                    CAPTURE(from);
                    break;
                }
                reach = to;
            }
            CHECK_FALSE(gap);
            CHECK(reach >= vHalf);
        }
        CHECK(matched == 1);
    }
}
