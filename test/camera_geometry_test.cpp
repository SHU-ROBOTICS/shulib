// vision::CameraModel and its helpers — pixels to bearings, elevations, ranges and field positions, each
// an INTERVAL from the stated tolerances; the mount's checks; boundAtImage (chunk VIS1-A1, the VIS1
// brief's §1.7, §1.9 and §2.2 "the geometry").
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════
//   * HAND ANCHORS that come from neither implementation: a point straight ahead at the lens's height
//     projects to the image's centre; a point ABOVE the axis projects to a SMALLER row (the anchor that
//     caught the host camera model's first sign error); a point to the LEFT has a POSITIVE bearing; a
//     camera yawed 20 degrees left sees its axis at +20 degrees (mutations MG1 the yaw's sign, MG2 the
//     image's down read as up).
//   * TWO FOCAL LENGTHS: 212.3 and 195.8 px, never a square pixel (MG3) — and each its own tolerance:
//     the box has 32 corners, (fx down, fy up) among them.
//   * THE PROPERTY: 480 Pins ACROSS THE WHOLE PICTURE seen through the INDEPENDENT host camera model with
//     a TRUE camera perturbed up to 98 % of every stated tolerance — the two focal lengths apart, the
//     lens 3 in off the centreline either way, the robot turned, every box edge jittered — every ground
//     range, bearing, elevation, size range and field fix holds the truth (MG4: nominal only — the
//     intervals collapse to the jitter and miss).
//   * THE SIZE RANGE across the whole width and through pitched cameras holds a thin rod's true range —
//     containment (the review's finding: it took the depth along the optical axis); every jitter term
//     LANDED ON: a pixel moved exactly kBoxEdgePx from the truth puts the truth on the interval's end.
//   * fieldFix: the lens's LEFT offset's sign (hand anchors); the mount's tolerance added exactly; a
//     non-finite or negative tolerance an INFINITE radius; a heading tolerance reaching the far side
//     counts the diametrically opposite point AND the far range's arc pointing straight away (the
//     verification round) — a sweep of bearing, range and heading over 450 fixes pins the radius EXACT:
//     never short, never wider than the sweep's step.
//   * EXACT HULLS (the verification round): a dense sweep of the WHOLE tolerance box at 100 % — the pitch
//     inside its tolerance, the principal point's column, each focal length's middle — never lies outside
//     a bearing, elevation or size range and reaches both its ends; the bearing's interior end landed on
//     by hand; a thin line STRADDLING the lens's level, projected by this file's own pinhole, inside its
//     size range for every true camera of the box.
//   * THE HORIZON: a ground range near it is REFUSED, never infinite — including a row INSIDE the margin,
//     where every corner's ray descends but the shallowest by less than it (MG8b); so is a plane met
//     within the minimum angle or behind the camera.
//   * boundAtImage: no motion changes nothing; motion adds EXACTLY |v| L and |w| L; a negative or
//     non-finite input is never a tight bound (MG5, MG6) — and the finding it fixes, pinned on the host
//     camera model: an image 66 ms or more old, the robot turning 180 deg/s, puts the true Pin OUTSIDE
//     the fix computed with the pose at read completion; the bound AT THE IMAGE always holds it.
//   * THE MOUNT'S CHECKS (config::cameraPlacementProblem) on both sides of each range, and
//     cameraModelProblem: no camera, an UNSET mount, a bad one, and the geometry's own limit — the pitch
//     with its tolerance past kMaxPitchSpanDeg, its edge (44 + 11 = 55) landed on.

#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "shulib/config/camera_config.hpp"
#include "shulib/hal/vision_frames.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/sim/camera_plant.hpp"
#include "shulib/units/quantity.hpp"
#include "shulib/vision/camera_geometry.hpp"

namespace config = shulib::config;
namespace hal = shulib::hal;
namespace sim = shulib::sim;
namespace vision = shulib::vision;
using shulib::units::Length;
using shulib::units::Time;

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

config::CameraPlacement mount(double h, double pitch, double yaw = 0.0, double forward = 7.0,
                              double left = 0.0) {
    config::CameraPlacement p{};
    p.measured = true;
    p.forward = Length{forward};
    p.left = Length{left};
    p.height = Length{h};
    p.pitchDeg = pitch;
    p.yawDeg = yaw;
    p.positionTolerance = Length{0.25};
    p.heightTolerance = Length{0.25};
    p.yawToleranceDeg = 1.0;
    p.pitchToleranceDeg = 1.0;
    p.provenance = "host test";
    return p;
}

/// Optics with NO tolerance, and a mount whose every tolerance is zero: the intervals of a model built from
/// them are the box-edge jitter ALONE — so a pixel moved by exactly kBoxEdgePx from the truth puts the
/// truth exactly on an end of the interval (the jitter's edge, landed on).
constexpr vision::CameraOptics kExactOptics{74.0, 63.0, 0.0, 320, 240};

config::CameraPlacement exactMount(double h, double pitch) {
    config::CameraPlacement p = mount(h, pitch);
    p.positionTolerance = Length{0.0};
    p.heightTolerance = Length{0.0};
    p.yawToleranceDeg = 0.0;
    p.pitchToleranceDeg = 0.0;
    return p;
}

/// The host camera model's camera with the optics' two focal lengths scaled by `sx` and `sy` apart.
void scaleOptics(sim::TrueCamera& t, double sx, double sy) {
    t.hfovDeg = 2.0 * std::atan(std::tan(37.0 * vision::kDegToRad) / sx) / vision::kDegToRad;
    t.vfovDeg = 2.0 * std::atan(std::tan(31.5 * vision::kDegToRad) / sy) / vision::kDegToRad;
}

/// The size range's truth for a Pin's box, by plain trigonometry from the scene (no projection): the box's
/// bottom row is the base's NEAR rim at the floor and its top row the top face's FAR rim (the lens is
/// above the Pin's top), so the box spans what a vertical line `H` tall would at this distance from the
/// lens. `dist` is the lens-to-centre distance, `lensH` the true lens height.
double pinLineEquivalent(double dist, double lensH, double H = 6.5, double r = 0.8) {
    return H / ((H - lensH) / (dist + r) + lensH / (dist - r));
}

/// The size range's answer for ONE camera `c`, written from its documentation: a vertical line `H` tall whose
/// box spans rows [top, bottom] around column `col` and centre row `vc` is H / (cos(b) x (rise(top) -
/// rise(bottom))) away, b the centre ray's bearing off the axis (the public ray()) and rise(v) = tan(pitch -
/// atan((v - cy) / fy)) — what the hull tests sweep over the tolerance box.
double sizeAt(const vision::CameraCorner& c, double H, double col, double vc, double top, double bottom) {
    double b = 0.0;
    double e = 0.0;
    vision::CameraModel::ray(c, col, vc, b, e);
    auto rise = [&](double row) { return std::tan(c.pitch - std::atan((row - c.cy) / c.fy)); };
    return H / (std::cos(b - c.yaw) * (rise(top) - rise(bottom)));
}

std::string problem(const config::CameraPlacement& p) {
    const char* w = config::cameraPlacementProblem(p);
    return w == nullptr ? std::string{"(none)"} : std::string{w};
}

vision::PoseBound poseAt(double x, double y, double headingRad, double posTol = 1.0, double headTol = 1.0) {
    vision::PoseBound b{};
    b.pose = shulib::math::Pose2d{Length{x}, Length{y}, shulib::math::Angle::radians(headingRad)};
    b.positionTolerance = Length{posTol};
    b.headingToleranceDeg = headTol;
    return b;
}

/// The TRUE camera of the latency scenes: the believed mount moved inside its stated tolerances
/// (+0.2 in forward and up, -0.8 degree pitch, +0.7 degree yaw, the optics 2 % off).
sim::TrueCamera latencyTruth() {
    sim::TrueCamera t{};
    t.forward = 7.2;
    t.height = 10.2;
    t.pitchDeg = -0.8;
    t.yawDeg = 0.7;
    t.hfovDeg = 75.5;
    t.vfovDeg = 62.0;
    return t;
}

/// One LAGGED read of a Pin at (36, pinY): the camera sees the robot where it WAS `lag` seconds before
/// the read completed (driving `speed` in/s along +X, turning `turnDegS`), and the fix is computed from
/// the pose at completion — widened by boundAtImage when `widen`. The truth is what the fix measures:
/// the Pin's NEAR RIM (its box's bottom row is the base's nearest point — the belief HA-200 names), one
/// radius (0.8 in) short of its centre along the line from the TRUE lens at the image's time. Returns
/// false when no Pin was seen.
bool laggedFix(double lag, double speed, double turnDegS, bool widen, double pinY, double& err,
               double& allowed) {
    sim::CameraPlant plant{latencyTruth(), 21};
    plant.setJitterPx(1.0);
    sim::Element pin{};
    pin.classId = 3;
    pin.x = 36.0;
    pin.y = pinY;
    plant.addElement(pin);
    const vision::CameraModel m{vision::kAiVisionOptics, mount(10.0, 0.0)};
    const double headingThen = -turnDegS * vision::kDegToRad * lag;
    const double xThen = -speed * lag;
    const hal::VisionFrame f = plant.read(xThen, 0.0, headingThen, Time{1.0});
    vision::PoseBound b = poseAt(0.0, 0.0, 0.0);
    if (widen) {
        b = vision::boundAtImage(b, speed, turnDegS, lag);
    }
    for (std::uint8_t i = 0; i < f.count; ++i) {
        const hal::Detection& d = f.detections[i];
        if (d.kind != hal::DetectionKind::Object || d.clipped()) {
            continue;
        }
        const double u = d.left + d.width / 2.0;
        const vision::Interval rg = m.groundRange(u, d.top + d.height);
        const vision::Interval br = m.bearing(u, d.top + d.height / 2.0);
        const vision::FieldFix fix = m.fieldFix(b, br, rg);
        if (!fix.valid) {
            return false;
        }
        const sim::TrueCamera t = latencyTruth();
        const double lensX = xThen + std::cos(headingThen) * t.forward;
        const double lensY = std::sin(headingThen) * t.forward;
        const double toPin = std::hypot(pin.x - lensX, pin.y - lensY);
        const double rimX = pin.x - 0.8 * (pin.x - lensX) / toPin;
        const double rimY = pin.y - 0.8 * (pin.y - lensY) / toPin;
        err = std::hypot(fix.x - rimX, fix.y - rimY);
        allowed = fix.radius;
        return true;
    }
    return false;
}

}  // namespace

TEST_CASE("VIS1-A1 geometry: TWO focal lengths, never a square pixel") {
    const vision::CameraModel m{vision::kAiVisionOptics, mount(8.0, 0.0)};
    CHECK(m.fx() == doctest::Approx(160.0 / std::tan(37.0 * vision::kDegToRad)));
    CHECK(m.fy() == doctest::Approx(120.0 / std::tan(31.5 * vision::kDegToRad)));
    CHECK(m.fx() == doctest::Approx(212.33).epsilon(0.001));
    CHECK(m.fy() == doctest::Approx(195.84).epsilon(0.001));
    CHECK(m.fx() - m.fy() > 15.0);
    CHECK(m.opticsTolerance() == doctest::Approx(0.05));
    CHECK(vision::kBoxEdgePx == doctest::Approx(2.0));
    CHECK(vision::kAiVisionOptics.widthPx == 320);
    CHECK(vision::kAiVisionOptics.heightPx == 240);
}

TEST_CASE("VIS1-A1 geometry: hand anchors -- straight ahead is the centre, ABOVE is a smaller row, LEFT is a "
          "positive bearing") {
    const vision::CameraModel m{vision::kAiVisionOptics, mount(8.0, 0.0)};
    const vision::CameraCorner n = m.nominal();
    double u = 0.0;
    double v = 0.0;
    REQUIRE(m.project(n, 7.0 + 24.0, 0.0, 8.0, u, v));
    CHECK(u == doctest::Approx(160.0));
    CHECK(v == doctest::Approx(120.0));
    REQUIRE(m.project(n, 7.0 + 24.0, 0.0, 13.0, u, v));
    CHECK(v < 120.0);  // up is a SMALLER row
    CHECK(v == doctest::Approx(120.0 - n.fy * 5.0 / 24.0));
    REQUIRE(m.project(n, 7.0 + 24.0, 6.0, 8.0, u, v));
    CHECK(u < 160.0);  // left is a SMALLER column
    CHECK(u == doctest::Approx(160.0 - n.fx * 6.0 / 24.0));
    CHECK_FALSE(m.project(n, 7.0 - 1.0, 0.0, 8.0, u, v));  // behind the lens
    // A lens 3 in LEFT of the tracking centre: a point straight ahead of the LENS is on the centre
    // column; one straight ahead of the tracking centre is 3 in to the lens's RIGHT (review X13: the
    // offset's sign in project()).
    const vision::CameraModel offset{vision::kAiVisionOptics, mount(8.0, 0.0, 0.0, 7.0, 3.0)};
    const vision::CameraCorner on = offset.nominal();
    REQUIRE(offset.project(on, 7.0 + 24.0, 3.0, 8.0, u, v));
    CHECK(u == doctest::Approx(160.0));
    REQUIRE(offset.project(on, 7.0 + 24.0, 0.0, 8.0, u, v));
    CHECK(u == doctest::Approx(160.0 + on.fx * 3.0 / 24.0));

    // The ray, the other way: a pixel above the centre looks UP, one left of it looks LEFT.
    double b = 0.0;
    double e = 0.0;
    vision::CameraModel::ray(n, 160.0, 120.0, b, e);
    CHECK(b == doctest::Approx(0.0));
    CHECK(e == doctest::Approx(0.0));
    vision::CameraModel::ray(n, 160.0, 100.0, b, e);
    CHECK(e > 0.0);
    CHECK(e == doctest::Approx(std::atan(20.0 / n.fy)));
    vision::CameraModel::ray(n, 120.0, 120.0, b, e);
    CHECK(b > 0.0);
    CHECK(b == doctest::Approx(std::atan(40.0 / n.fx)));
    CHECK(m.elevation(160.0, 100.0).lo > 0.0);
    CHECK(m.bearing(120.0, 120.0).lo > 0.0);
    CHECK(m.bearing(200.0, 120.0).hi < 0.0);
}

TEST_CASE("VIS1-A1 geometry P2.1: the host camera model's row for a floor point 24 in ahead is the "
          "pinhole's, and the library's ground range from it holds 24") {
    // By hand: a level lens 6 in up; the floor point 24 in ahead of the LENS is at v = 120 + fy * 6 / 24.
    sim::TrueCamera t{};
    t.forward = 7.0;
    t.height = 6.0;
    sim::CameraPlant plant{t};
    sim::ColourQuad q;
    q.c = {sim::P3{31.0, -0.5, 0.0}, sim::P3{31.0, 0.5, 0.0}, sim::P3{31.0, 0.5, 1.0},
           sim::P3{31.0, -0.5, 1.0}};
    q.colourId = 1;
    plant.addQuad(q);
    const hal::VisionFrame f = plant.read(0.0, 0.0, 0.0, Time{0.0});
    REQUIRE(f.count == 1);
    const double fy = 120.0 / std::tan(31.5 * vision::kDegToRad);
    const double row = 120.0 + fy * 6.0 / 24.0;
    CHECK(f.detections[0].top + f.detections[0].height == doctest::Approx(row).epsilon(0.01));
    const vision::CameraModel m{vision::kAiVisionOptics, mount(6.0, 0.0)};
    const vision::Interval r = m.groundRange(160.0, row);
    REQUIRE(r.valid);
    CHECK(r.contains(24.0));
    // About +-1 degree of pitch, +-0.25 in of height and +-5 % optics at a 14-degree depression: roughly
    // 21..30 in — a bound, not a point.
    CHECK(r.lo > 18.0);
    CHECK(r.hi < 32.0);
}

TEST_CASE("VIS1-A1 geometry P2.2: 480 Pins ACROSS THE WHOLE PICTURE through a TRUE camera perturbed "
          "inside the tolerances -- the two focal lengths APART, the lens off the centreline, the robot "
          "turned, every box edge jittered -- every range, bearing, elevation, size range and fix holds the "
          "truth") {
    // Review fixes: the first version perturbed both focal lengths by ONE shared factor (so the corner
    // (fx down, fy up) was never needed — groundRange missed 10 of 3824 with them apart), placed Pins only
    // within +-10 in of the axis (so the size range's missing sideways factor never showed), mounted the
    // lens on the centreline with the robot at heading 0 (so a sign error in the lens's LEFT offset —
    // fieldFix's and the host camera model's — cancelled), jittered nothing (so the edge-jitter terms
    // were never needed) and never checked elevation() or sizeRange().
    int checked = 0;
    int contained = 0;
    int offAxis = 0;
    double widest = 0.0;
    for (int i = 0; i < 480; ++i) {
        CAPTURE(i);
        const double sgn[5] = {(i & 1) ? 1.0 : -1.0, (i & 2) ? 1.0 : -1.0, (i & 4) ? 1.0 : -1.0,
                               (i & 8) ? 1.0 : -1.0, (i & 16) ? 1.0 : -1.0};
        const double frac = 0.98 * ((i % 7) / 6.0);  // 0 .. 98 % of each tolerance
        const double lensLeft = (i % 3 == 0) ? 0.0 : ((i % 3 == 1) ? 3.0 : -3.0);
        sim::TrueCamera t{};
        t.forward = 7.0;
        t.left = lensLeft;
        t.height = 8.0 + sgn[0] * 0.25 * frac;
        t.pitchDeg = 5.0 + sgn[1] * 1.0 * frac;
        t.yawDeg = sgn[2] * 1.0 * frac;
        scaleOptics(t, 1.0 + sgn[3] * 0.05 * frac, 1.0 + sgn[4] * 0.05 * frac);  // APART
        sim::CameraPlant plant{t, static_cast<std::uint32_t>(i + 1)};
        plant.setJitterPx(0.98);  // with the model's rounding to whole pixels, every edge within 2 px
        // The robot somewhere on the field, turned; the Pin at a bearing across the whole picture.
        const double rx = -30.0 + (i % 13) * 5.0;
        const double ry = 20.0 - (i % 11) * 4.0;
        const double rh = -2.5 + (i % 17) * (5.0 / 16.0);
        const double bDeg = -34.0 + 68.0 * ((i * 37) % 101) / 100.0;
        const double dist = 22.0 + 40.0 * ((i * 53) % 97) / 96.0;
        const double b = bDeg * vision::kDegToRad;
        const double lensX = rx + std::cos(rh) * 7.0 - std::sin(rh) * lensLeft;
        const double lensY = ry + std::sin(rh) * 7.0 + std::cos(rh) * lensLeft;
        sim::Element pin{};
        pin.classId = 3;
        pin.x = lensX + dist * std::cos(rh + b);
        pin.y = lensY + dist * std::sin(rh + b);
        plant.addElement(pin);
        const hal::VisionFrame f = plant.read(rx, ry, rh, Time{0.0});
        if (f.count != 1 || f.detections[0].clipped()) {
            continue;
        }
        const hal::Detection& d = f.detections[0];
        const vision::CameraModel m{vision::kAiVisionOptics, mount(8.0, 5.0, 0.0, 7.0, lensLeft)};
        const double uC = d.left + d.width / 2.0;
        const double vC = d.top + d.height / 2.0;
        const double vB = d.top + d.height;
        const vision::Interval rg = m.groundRange(uC, vB);
        if (!rg.valid) {
            continue;
        }
        ++checked;
        if (std::fabs(bDeg) > 20.0) {
            ++offAxis;
        }
        const vision::Interval br = m.bearing(uC, vC);
        const vision::Interval el = m.elevation(uC, vB);
        const vision::Interval sz = m.sizeRange(6.5, uC, vC, d.height);
        // The truths: the box's bottom is the near rim at the floor (the ground range and the bottom
        // row's elevation, from the TRUE lens height); the bearing is to the centre; the size range's is
        // the box's line-equivalent (pinLineEquivalent).
        const double trueNear = dist - 0.8;
        const double trueElevation = std::atan2(-t.height, trueNear);
        const double trueSize = pinLineEquivalent(dist, t.height);
        const vision::FieldFix fix = m.fieldFix(poseAt(rx, ry, rh, 0.0, 0.0), br, rg);
        const double fixErr = std::hypot(fix.x - pin.x, fix.y - pin.y);
        const bool ok = rg.contains(trueNear) && br.contains(b) && el.contains(trueElevation)
                        && sz.contains(trueSize) && fix.valid && fixErr <= fix.radius + 0.8;
        if (ok) {
            ++contained;
        } else {
            MESSAGE("P2.2 miss at " << i << ": range " << rg.contains(trueNear) << " bearing "
                                    << br.contains(b) << " elevation " << el.contains(trueElevation)
                                    << " size " << sz.contains(trueSize) << " [" << sz.lo << ", " << sz.hi
                                    << "] vs " << trueSize << " fix " << fixErr << " / " << fix.radius);
        }
        widest = std::fmax(widest, fix.radius);
    }
    MESSAGE("P2.2: " << checked << " placements checked (" << offAxis << " more than 20 degrees off the axis), "
                     << contained << " held the truth; widest fix radius " << widest << " in");
    CHECK(checked > 300);
    CHECK(offAxis > 100);
    CHECK(contained == checked);
}

TEST_CASE("VIS1-A1 geometry: the SIZE RANGE of a thin upright line holds its TRUE range ACROSS THE WHOLE "
          "PICTURE, through pitched cameras, the focal lengths perturbed APART") {
    // BUG CAUGHT (review): the size range took the depth ALONG THE OPTICAL AXIS for the horizontal range
    // — a Pin 30 degrees off the axis read about 13 % near (24 in out: [15.80, 21.66]), and a pitched
    // camera missed even on the axis; the test placed Pins straight ahead only. For a vertical line the
    // answer is now exact per corner, so the truth must be INSIDE the interval — containment, not an
    // overlap. The "line" is a rod 0.3 in across (thinner, and the host camera model's box is under a
    // pixel wide and not seen), so its truth is its line-equivalent range (pinLineEquivalent), a few
    // tenths of an inch short of its centre.
    int seen = 0;
    int held = 0;
    for (const double pitch : {-12.0, 0.0, 10.0}) {
        for (int k = 0; k < 32; ++k) {
            const double sg[5] = {(k & 1) ? 1.0 : -1.0, (k & 2) ? 1.0 : -1.0, (k & 4) ? 1.0 : -1.0,
                                  (k & 8) ? 1.0 : -1.0, (k & 16) ? 1.0 : -1.0};
            const vision::CameraModel m{vision::kAiVisionOptics, mount(10.0, pitch, 0.0, 7.0, 2.0)};
            sim::TrueCamera t{};
            t.forward = 7.0;
            t.left = 2.0;
            t.height = 10.0 + 0.98 * 0.25 * sg[0];
            t.pitchDeg = pitch + 0.98 * sg[1];
            t.yawDeg = 0.98 * sg[2];
            scaleOptics(t, 1.0 + 0.98 * 0.05 * sg[3], 1.0 + 0.98 * 0.05 * sg[4]);
            for (int bd = -36; bd <= 36; bd += 6) {
                for (double dist = 14.0; dist <= 70.0; dist += 7.0) {
                    sim::CameraPlant plant{t};
                    sim::Element line{};
                    line.diameter = 0.3;
                    line.x = 7.0 + dist * std::cos(bd * vision::kDegToRad);
                    line.y = 2.0 + dist * std::sin(bd * vision::kDegToRad);
                    plant.addElement(line);
                    const hal::VisionFrame f = plant.read(0.0, 0.0, 0.0, Time{0.0});
                    if (f.count != 1 || f.detections[0].clipped()) {
                        continue;
                    }
                    const hal::Detection& d = f.detections[0];
                    const vision::Interval s =
                        m.sizeRange(6.5, d.left + d.width / 2.0, d.top + d.height / 2.0, d.height);
                    ++seen;
                    if (s.contains(pinLineEquivalent(dist, t.height, 6.5, 0.15))) {
                        ++held;
                    } else {
                        MESSAGE("size range miss: pitch " << pitch << " k " << k << " bearing " << bd << " dist "
                                                          << dist << ": [" << s.lo << ", " << s.hi << "]");
                    }
                }
            }
        }
    }
    MESSAGE("size range: " << seen << " thin lines measured, " << held << " held their true range");
    CHECK(seen > 4000);
    CHECK(held == seen);

    // The hand anchor the review measured: a line 24 in out at 30 degrees, a level camera — the old
    // answer [15.80, 21.66] was about the AXIS depth (24 cos 30 = 20.8).
    const vision::CameraModel level{vision::kAiVisionOptics, mount(10.0, 0.0)};
    sim::TrueCamera t{};
    t.forward = 7.0;
    t.height = 10.0;
    sim::CameraPlant plant{t};
    sim::Element line{};
    line.diameter = 0.3;
    line.x = 7.0 + 24.0 * std::cos(30.0 * vision::kDegToRad);
    line.y = 24.0 * std::sin(30.0 * vision::kDegToRad);
    plant.addElement(line);
    const hal::VisionFrame f = plant.read(0.0, 0.0, 0.0, Time{0.0});
    REQUIRE(f.count == 1);
    const hal::Detection& d = f.detections[0];
    const vision::Interval s = level.sizeRange(6.5, d.left + d.width / 2.0, d.top + d.height / 2.0, d.height);
    CAPTURE(s.lo);
    CAPTURE(s.hi);
    CHECK(s.contains(pinLineEquivalent(24.0, 10.0, 6.5, 0.15)));
    CHECK(s.mid() > 24.0 * std::cos(30.0 * vision::kDegToRad) + 2.0);  // centred on the range, not the axis depth
}

TEST_CASE("VIS1-A1 geometry: every jitter term LANDED ON -- a pixel moved by exactly kBoxEdgePx from the "
          "truth puts the truth exactly on an end of the interval, never outside") {
    // With no other tolerance (kExactOptics, exactMount), an interval is the box-edge jitter ALONE. Move
    // the observed pixel(s) by exactly kBoxEdgePx AWAY from what the truth projects to, and the jitter's
    // own corner restores the truth: it must lie on the interval's end (to rounding), and a term that is
    // missing or half as wide (the bearing's centre jitter was +-kBoxEdgePx/2 until the review: the centre
    // of two edges that each move 2 px moves 2 px) leaves it outside.
    constexpr double kE = vision::kBoxEdgePx;
    constexpr double kTol = 1e-9;
    const vision::CameraModel m{kExactOptics, exactMount(10.0, 0.0)};
    const vision::CameraCorner n = m.nominal();
    // A vertical line 6.5 in tall, 24 in out at a bearing of +30 degrees (LEFT of the axis), from the
    // lens 7 in forward. A level camera puts its top and bottom in ONE column.
    const double b = 30.0 * vision::kDegToRad;
    const double fwd = 7.0 + 24.0 * std::cos(b);
    const double lft = 24.0 * std::sin(b);
    double uTop = 0.0;
    double vTop = 0.0;
    double uBottom = 0.0;
    double vBottom = 0.0;
    REQUIRE(m.project(n, fwd, lft, 6.5, uTop, vTop));
    REQUIRE(m.project(n, fwd, lft, 0.0, uBottom, vBottom));
    REQUIRE(uTop == doctest::Approx(uBottom));
    const double u = uTop;
    REQUIRE(u < 160.0);  // left of the axis: AWAY from it is a smaller column

    SUBCASE("bearing: the centre moved a whole kBoxEdgePx") {
        const vision::Interval br = m.bearing(u - kE, 120.0);  // away from the axis: reads MORE left
        CAPTURE(br.lo);
        CAPTURE(br.hi);
        CHECK(br.lo <= b + kTol);
        CHECK(br.lo >= b - kTol);  // ON the end
        const vision::Interval br2 = m.bearing(u + kE, 120.0);
        CHECK(br2.hi >= b - kTol);
        CHECK(br2.hi <= b + kTol);
    }
    SUBCASE("elevation and ground range: the bottom row AND the column moved a whole kBoxEdgePx") {
        // A higher row looks less down, and a column farther from the axis too (the ray turned aside is
        // nearer level): only the corner that undoes BOTH puts the truth on the interval's end. (Until the
        // verification round the column's own jitter was not carried: a column moved 2 px off the truth
        // left a pitched camera's elevation near the picture's corners up to 2.8e-3 rad outside.)
        const double e = std::atan2(-10.0, 24.0);
        const vision::Interval up = m.elevation(u - kE, vBottom - kE);
        CHECK(up.lo <= e + kTol);
        CHECK(up.lo >= e - kTol);
        const vision::Interval rg = m.groundRange(u - kE, vBottom - kE);  // reads farther
        CHECK(rg.lo <= 24.0 + kTol);
        CHECK(rg.lo >= 24.0 - kTol);
        // The row alone moved, the column true: the truth is INSIDE, no longer on the end (the column's
        // jitter carried both ways).
        const vision::Interval up2 = m.elevation(u, vBottom - kE);
        CHECK(up2.lo < e - 1e-6);
        CHECK(up2.contains(e));
    }
    SUBCASE("size range: the column, the top and the bottom each moved a whole kBoxEdgePx") {
        // Shrunk by an edge at each end (reads FARTHER) and moved away from the axis (reads farther
        // still): only the corner that undoes all three puts the truth on the interval's NEAR end.
        const double top = vTop + kE;
        const double bottom = vBottom - kE;
        const vision::Interval sz = m.sizeRange(6.5, u - kE, 0.5 * (top + bottom), bottom - top);
        CAPTURE(sz.lo);
        CAPTURE(sz.hi);
        CHECK(sz.lo <= 24.0 + kTol);
        CHECK(sz.lo >= 24.0 - kTol);
        // Each edge alone, moved the other way, puts the truth on the FAR end.
        const double top2 = vTop - kE;
        const double bottom2 = vBottom + kE;
        const vision::Interval sz2 = m.sizeRange(6.5, u + kE, 0.5 * (top2 + bottom2), bottom2 - top2);
        CHECK(sz2.hi >= 24.0 - kTol);
        CHECK(sz2.hi <= 24.0 + kTol);
    }
}

TEST_CASE("VIS1-A1 geometry P2.3: an interval WIDENS with each tolerance -- a mount typed loosely refuses "
          "more") {
    const vision::CameraModel tight{vision::kAiVisionOptics, mount(8.0, 5.0)};
    const vision::Interval a = tight.groundRange(160.0, 200.0);
    REQUIRE(a.valid);
    config::CameraPlacement loose = mount(8.0, 5.0);
    loose.pitchToleranceDeg = 3.0;
    const vision::Interval b = vision::CameraModel{vision::kAiVisionOptics, loose}.groundRange(160.0, 200.0);
    REQUIRE(b.valid);
    CHECK(b.width() > a.width());
    CHECK(b.lo < a.lo);
    CHECK(b.hi > a.hi);
    loose = mount(8.0, 5.0);
    loose.heightTolerance = Length{1.0};
    const vision::Interval c = vision::CameraModel{vision::kAiVisionOptics, loose}.groundRange(160.0, 200.0);
    CHECK(c.width() > a.width());
    loose = mount(8.0, 5.0);
    loose.yawToleranceDeg = 4.0;
    CHECK(vision::CameraModel{vision::kAiVisionOptics, loose}.bearing(160.0, 120.0).width()
          > tight.bearing(160.0, 120.0).width());
    // Every corner: 32 with the height tolerance, 16 without — the two focal lengths each at its own
    // extreme (review: they were tied, so (fx down, fy up) was never evaluated).
    int with = 0;
    int without = 0;
    int fxDownFyUp = 0;
    int fxUpFyDown = 0;
    tight.eachCorner(true, [&](const vision::CameraCorner& k) {
        ++with;
        if (k.fx < tight.fx() && k.fy > tight.fy()) {
            ++fxDownFyUp;
        }
        if (k.fx > tight.fx() && k.fy < tight.fy()) {
            ++fxUpFyDown;
        }
    });
    tight.eachCorner(false, [&](const vision::CameraCorner&) { ++without; });
    CHECK(with == 32);
    CHECK(without == 16);
    CHECK(fxDownFyUp == 8);
    CHECK(fxUpFyDown == 8);
}

TEST_CASE("VIS1-A1 geometry P2.4: near the horizon the ground range REFUSES -- never an infinite range") {
    const vision::CameraModel m{vision::kAiVisionOptics, mount(8.0, 0.0)};
    const vision::Interval nearHorizon = m.groundRange(160.0, 122.0);  // ~0.6 degree down
    CHECK_FALSE(nearHorizon.valid);
    CHECK_FALSE(nearHorizon.contains(100.0));
    CHECK_FALSE(m.groundRange(160.0, 100.0).valid);  // above the horizon
    const vision::Interval far = m.groundRange(160.0, 200.0);
    REQUIRE(far.valid);
    CHECK(std::isfinite(far.hi));
    // The MARGIN itself, landed on: a row where EVERY corner's ray descends, the shallowest by more than
    // 1 degree and less than the default 2 — refused at the default, accepted with a 1-degree margin.
    // BUG CAUGHT (mutation MG8b, the margin dropped to "anywhere below level"): the rows above refuse
    // because some corner looks UP, which that mutant still refuses; only a row inside the margin shows
    // it — there the shallowest ray meets the floor ~300 in out, a "range" that is a guess.
    double shallowestDeg = 90.0;
    m.eachCorner(true, [&](const vision::CameraCorner& c) {
        for (const double dv : {-vision::kBoxEdgePx, vision::kBoxEdgePx}) {
            double bearing = 0.0;
            double elevation = 0.0;
            vision::CameraModel::ray(c, 160.0, 131.0 + dv, bearing, elevation);
            shallowestDeg = std::fmin(shallowestDeg, -elevation / vision::kDegToRad);
        }
    });
    CAPTURE(shallowestDeg);
    REQUIRE(shallowestDeg > 1.0);
    REQUIRE(shallowestDeg < 2.0);
    CHECK_FALSE(m.groundRange(160.0, 131.0).valid);
    const vision::Interval looser = m.groundRange(160.0, 131.0, 1.0);
    REQUIRE(looser.valid);
    CHECK(std::isfinite(looser.hi));
    CHECK(looser.hi > 200.0);  // why the default refuses it: the far end is nowhere near a measurement
    // rangeToPlane: a plane above the lens met by a ray going up; refused within the angle, and refused
    // for a ray going the wrong way.
    const vision::CameraCorner n = m.nominal();
    double out = 0.0;
    CHECK(vision::CameraModel::rangeToPlane(n, 160.0, 60.0, 14.0, 0.5, out));
    CHECK(out > 0.0);
    CHECK(out == doctest::Approx(6.0 / std::tan(std::atan(60.0 / n.fy))));
    CHECK_FALSE(vision::CameraModel::rangeToPlane(n, 160.0, 120.5, 14.0, 0.5, out));  // ~0.15 degree
    CHECK_FALSE(vision::CameraModel::rangeToPlane(n, 160.0, 200.0, 14.0, 0.5, out));  // looks down
}

TEST_CASE("VIS1-A1 geometry P2.6: a camera YAWED 20 degrees left sees its axis at +20 (the yaw's sign)") {
    const vision::CameraModel m{vision::kAiVisionOptics, mount(10.0, 0.0, 20.0)};
    const vision::Interval b = m.bearing(160.0, 120.0);
    CHECK(b.contains(20.0 * vision::kDegToRad));
    CHECK(b.lo > 0.0);
    // And through the host camera model: a Pin 30 in out along the yawed axis lands at the centre column.
    sim::TrueCamera t{};
    t.forward = 7.0;
    t.height = 10.0;
    t.yawDeg = 20.0;
    sim::CameraPlant plant{t};
    sim::Element pin{};
    pin.x = 7.0 + 30.0 * std::cos(20.0 * vision::kDegToRad);
    pin.y = 30.0 * std::sin(20.0 * vision::kDegToRad);
    plant.addElement(pin);
    const hal::VisionFrame f = plant.read(0.0, 0.0, 0.0, Time{0.0});
    REQUIRE(f.count == 1);
    CHECK(f.detections[0].left + f.detections[0].width / 2.0 == doctest::Approx(160.0).epsilon(0.01));
}

TEST_CASE("VIS1-A1 geometry: the size range holds a Pin's true range; a box too small to measure is "
          "refused") {
    const vision::CameraModel m{vision::kAiVisionOptics, mount(10.0, 0.0)};
    for (const double dist : {18.0, 30.0, 48.0}) {
        CAPTURE(dist);
        sim::TrueCamera t{};
        t.forward = 7.0;
        t.height = 10.0;
        sim::CameraPlant plant{t};
        sim::Element pin{};
        pin.x = 7.0 + dist;
        plant.addElement(pin);
        const hal::VisionFrame f = plant.read(0.0, 0.0, 0.0, Time{0.0});
        REQUIRE(f.count == 1);
        const hal::Detection& d = f.detections[0];
        const vision::Interval s =
            m.sizeRange(6.5, d.left + d.width / 2.0, d.top + d.height / 2.0, d.height);
        REQUIRE(s.valid);
        CHECK(s.lo <= dist + 0.8);  // the range to the axis lies between the near rim and the centre
        CHECK(s.hi >= dist - 0.8);
        CHECK(s.contains(pinLineEquivalent(dist, 10.0)));  // and holds the box's own truth exactly
    }
    CHECK_FALSE(m.sizeRange(6.5, 160.0, 120.0, 5.0).valid);   // 2 edges of jitter and a pixel
    CHECK(m.sizeRange(6.5, 160.0, 120.0, 5.01).valid);
    // A height that is not a positive number says nothing.
    for (const double bad : {0.0, -6.5, kNaN, kInf}) {
        CAPTURE(bad);
        CHECK_FALSE(m.sizeRange(bad, 160.0, 120.0, 40.0).valid);
    }
}

TEST_CASE("VIS1-A1 geometry: the field fix carries the pose's tolerances -- its radius grows with each") {
    const vision::CameraModel m{vision::kAiVisionOptics, mount(10.0, 0.0)};
    const vision::Interval b{true, -0.02, 0.02};
    const vision::Interval r{true, 29.0, 31.0};
    const vision::FieldFix tight = m.fieldFix(poseAt(10.0, 5.0, 0.0, 0.0, 0.0), b, r);
    REQUIRE(tight.valid);
    CHECK(tight.x == doctest::Approx(10.0 + 7.0 + 30.0));
    CHECK(tight.y == doctest::Approx(5.0));
    const vision::FieldFix pos = m.fieldFix(poseAt(10.0, 5.0, 0.0, 2.0, 0.0), b, r);
    CHECK(pos.radius == doctest::Approx(tight.radius + 2.0));
    const vision::FieldFix head = m.fieldFix(poseAt(10.0, 5.0, 0.0, 0.0, 5.0), b, r);
    CHECK(head.radius > tight.radius + 2.0);  // 5 degrees at ~37 in
    // Turned 90 degrees left: the camera's forward offset and the range both point along +Y.
    const vision::FieldFix turned = m.fieldFix(poseAt(0.0, 0.0, 0.5 * shulib::math::Angle::kPi, 0.0, 0.0), b, r);
    CHECK(turned.x == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(turned.y == doctest::Approx(37.0));
    CHECK_FALSE(m.fieldFix(poseAt(0.0, 0.0, 0.0), vision::Interval{}, r).valid);
    CHECK_FALSE(m.fieldFix(poseAt(0.0, 0.0, 0.0), b, vision::Interval{}).valid);

    SUBCASE("the MOUNT's position tolerance adds to the radius exactly (review X16)") {
        config::CameraPlacement loose = mount(10.0, 0.0);
        loose.positionTolerance = Length{1.75};
        const vision::FieldFix wider =
            vision::CameraModel{vision::kAiVisionOptics, loose}.fieldFix(poseAt(10.0, 5.0, 0.0, 0.0, 0.0), b, r);
        CHECK(wider.radius == doctest::Approx(tight.radius + 1.5));  // 1.75 - the mount()'s 0.25
    }
    SUBCASE("a lens LEFT of the tracking centre, the robot turned 90 degrees: the lens at (-left, forward) "
            "(review X12)") {
        const vision::CameraModel off{vision::kAiVisionOptics, mount(10.0, 0.0, 0.0, 7.0, 3.0)};
        const vision::Interval ahead{true, 0.0, 0.0};
        const vision::Interval thirty{true, 30.0, 30.0};
        const vision::FieldFix fx =
            off.fieldFix(poseAt(0.0, 0.0, 0.5 * shulib::math::Angle::kPi, 0.0, 0.0), ahead, thirty);
        CHECK(fx.x == doctest::Approx(-3.0));
        CHECK(fx.y == doctest::Approx(7.0 + 30.0));
        // ... and at heading 0, 3 in to the LEFT (+y).
        const vision::FieldFix f0 = off.fieldFix(poseAt(0.0, 0.0, 0.0, 0.0, 0.0), ahead, thirty);
        CHECK(f0.x == doctest::Approx(37.0));
        CHECK(f0.y == doctest::Approx(3.0));
    }
    SUBCASE("a NEGATIVE or NON-FINITE tolerance is not a bound: the radius is INFINITE, never tight") {
        // BUG CAUGHT (review): std::fmax drops a NaN, so a heading tolerance of +Inf or NaN collapsed the
        // radius to the two position tolerances — 1.25 in for a fix 30 in out.
        for (const double bad : {kInf, kNaN, -1.0, -kInf}) {
            CAPTURE(bad);
            const vision::FieldFix h = m.fieldFix(poseAt(10.0, 5.0, 0.0, 1.0, bad), b, r);
            CHECK(h.valid);
            CHECK(std::isinf(h.radius));
            CHECK(h.x == doctest::Approx(tight.x));  // the nominal point still computed
            const vision::FieldFix p = m.fieldFix(poseAt(10.0, 5.0, 0.0, bad, 1.0), b, r);
            CHECK(p.valid);
            CHECK(std::isinf(p.radius));
        }
        // boundAtImage's own "not a bound" output reaches the same verdict.
        const vision::PoseBound unusable = vision::boundAtImage(poseAt(10.0, 5.0, 0.0), 10.0, 10.0, -1.0);
        CHECK(std::isinf(m.fieldFix(unusable, b, r).radius));
    }
    SUBCASE("a pose or an interval that is not a finite number: no fix at all (a heading cannot be one: "
            "math::Angle refuses it)") {
        CHECK_THROWS(poseAt(10.0, 5.0, kNaN));
        CHECK_FALSE(m.fieldFix(poseAt(kNaN, 5.0, 0.0), b, r).valid);
        CHECK_FALSE(m.fieldFix(poseAt(10.0, kInf, 0.0), b, r).valid);
        CHECK_FALSE(m.fieldFix(poseAt(0.0, 0.0, 0.0), vision::Interval{true, kNaN, 0.1}, r).valid);
        CHECK_FALSE(m.fieldFix(poseAt(0.0, 0.0, 0.0), b, vision::Interval{true, 29.0, kInf}).valid);
        CHECK_FALSE(m.fieldFix(poseAt(0.0, 0.0, 0.0), b, vision::Interval{true, 31.0, 29.0}).valid);
    }
    SUBCASE("a heading tolerance that reaches the FAR SIDE: the farthest point is diametrically opposite, "
            "counted -- never only the interval's two ends") {
        // Robot at the origin, heading 0; the nominal point (37, 0), 37 in from the robot's centre. The
        // corner at bearing 0.3, range 31 is |v| = 37.7 in from the centre, 0.245 rad off the nominal
        // point's direction: turned until it faces the nominal point's OPPOSITE it is |v| + 37 away. A
        // 180-degree interval's two ENDS turn it 180 degrees, 0.245 rad short of that (the review's
        // "under-bounds slightly"); past 180 the ends come back round and the old radius shrank.
        // BUG CAUGHT (verification round): that corner is not the farthest point either. Turned 180
        // degrees, the lens is 7 in BEHIND the centre and the ray at bearing 0 — INSIDE the bearing
        // interval — points straight away from the nominal point: 7 + 31 + 37 = 75 in, where the corner
        // reaches only 74.74 (the radius was 0.26 in short, and this test's old expected value with it).
        const vision::Interval wideB{true, -0.3, 0.3};
        const double vx = 7.0 + 31.0 * std::cos(0.3);
        const double vy = 31.0 * std::sin(0.3);
        const double cornerFar = std::hypot(vx, vy) + 37.0;
        for (const double tolDeg : {180.0, 181.0, 270.0, 1000.0}) {
            CAPTURE(tolDeg);
            const vision::FieldFix wide = m.fieldFix(poseAt(0.0, 0.0, 0.0, 0.0, tolDeg), wideB, r);
            CHECK(wide.radius == doctest::Approx(7.0 + 31.0 + 37.0 + 0.25));  // + the mount's 0.25
            CHECK(wide.radius > cornerFar + 0.25 + 0.25);
        }
        // At 179 degrees the turn that puts the lens straight behind is not reached; the farthest point is
        // the far arc's, straight away from the nominal point, at the interval's END (the lens 1 degree
        // short of behind) — still farther than the corner's opposite.
        const double c179 = std::cos(179.0 * vision::kDegToRad);
        const double s179 = std::sin(179.0 * vision::kDegToRad);
        const double at179 = std::hypot(7.0 * c179 - 37.0, 7.0 * s179) + 31.0;
        const vision::FieldFix wide179 = m.fieldFix(poseAt(0.0, 0.0, 0.0, 0.0, 179.0), wideB, r);
        CHECK(wide179.radius == doctest::Approx(at179 + 0.25).epsilon(1e-12));
        CHECK(at179 > cornerFar + 0.25);
        // Well short of the far side (90 degrees), the interval's ends are the farthest: nearer than far.
        const vision::FieldFix short90 = m.fieldFix(poseAt(0.0, 0.0, 0.0, 0.0, 90.0), wideB, r);
        CHECK(short90.radius < cornerFar + 0.25 - 10.0);
        // Monotone in the tolerance: never smaller for a looser heading.
        double last = 0.0;
        for (double tolDeg = 0.0; tolDeg <= 360.0; tolDeg += 7.5) {
            CAPTURE(tolDeg);
            const double rad = m.fieldFix(poseAt(0.0, 0.0, 0.0, 0.0, tolDeg), wideB, r).radius;
            CHECK(rad >= last - 1e-9);
            last = rad;
        }
    }
}

TEST_CASE("VIS1-A1 geometry: the field fix's radius is EXACT -- a dense sweep of bearing, range and heading "
          "never lies outside it and reaches it, for lenses ahead, aside and behind, at every heading "
          "tolerance") {
    // BUG CAUGHT (verification round): the radius took the farthest point of each (bearing, range) CORNER
    // only. Past about 140 degrees of heading tolerance the farthest point can be INSIDE the bearing
    // interval — the far range's arc pointing straight away from the nominal point — and the radius fell
    // short by up to 0.36 in (the verifier's sweep: 81 of 400 fixes at 359 degrees). Here every fix is
    // swept: the radius, less the mount's tolerance (the pose's is 0), must hold the farthest sampled
    // point and be no farther beyond it than the sweep's own step can hide.
    struct Lens {
        double fwd;
        double lft;
    };
    const Lens lenses[] = {{7.0, 0.0}, {7.0, 3.0}, {-6.0, -3.0}, {0.0, 0.0}, {2.0, 8.0}};
    const vision::Interval bearings[] = {{true, -0.3, 0.3}, {true, 0.1, 0.5}, {true, -0.65, -0.2}};
    const vision::Interval ranges[] = {{true, 29.0, 31.0}, {true, 6.0, 14.0}, {true, 40.0, 75.0}};
    const double tolerances[] = {0.0, 1.0, 30.0, 90.0, 140.0, 170.0, 179.0, 180.0, 270.0, 359.0};
    constexpr int kB = 24;
    constexpr int kH = 360;
    const double heading = 0.7;  // not a special angle
    int fixes = 0;
    double worstShort = 0.0;
    double worstOver = 0.0;
    for (const Lens& lens : lenses) {
        const vision::CameraModel m{vision::kAiVisionOptics, mount(10.0, 0.0, 0.0, lens.fwd, lens.lft)};
        for (const vision::Interval& b : bearings) {
            for (const vision::Interval& r : ranges) {
                for (const double tol : tolerances) {
                    const vision::FieldFix f = m.fieldFix(poseAt(3.0, -2.0, heading, 0.0, tol), b, r);
                    REQUIRE(f.valid);
                    const double radius = f.radius - 0.25;  // less the mount's 0.25
                    const double dTh = tol * vision::kDegToRad;
                    double farthest = 0.0;
                    for (int i = 0; i <= kB; ++i) {
                        const double bb = b.lo + (b.hi - b.lo) * i / kB;
                        for (const double rr : {r.lo, r.mid(), r.hi}) {
                            for (int j = 0; j <= kH; ++j) {
                                const double h = heading - dTh + 2.0 * dTh * j / kH;
                                const double x = 3.0 + std::cos(h) * lens.fwd - std::sin(h) * lens.lft
                                                 + rr * std::cos(h + bb);
                                const double y = -2.0 + std::sin(h) * lens.fwd + std::cos(h) * lens.lft
                                                 + rr * std::sin(h + bb);
                                farthest = std::fmax(farthest, std::hypot(x - f.x, y - f.y));
                            }
                        }
                    }
                    ++fixes;
                    worstShort = std::fmax(worstShort, farthest - radius);
                    worstOver = std::fmax(worstOver, radius - farthest);
                    if (!(farthest <= radius + 1e-9) || !(radius <= farthest + 0.05)) {
                        MESSAGE("lens (" << lens.fwd << ", " << lens.lft << ") bearings [" << b.lo << ", " << b.hi
                                         << "] ranges [" << r.lo << ", " << r.hi << "] tolerance " << tol
                                         << ": radius " << radius << ", farthest swept " << farthest);
                    }
                    CHECK(farthest <= radius + 1e-9);   // never short
                    CHECK(radius <= farthest + 0.05);   // never wider than the sweep's step hides
                }
            }
        }
    }
    MESSAGE(fixes << " fixes swept: worst short " << worstShort << " in, worst over " << worstOver << " in");
    CHECK(fixes == 5 * 3 * 3 * 10);
}

TEST_CASE("VIS1-A1 geometry: a point on the lens's LEVEL -- the bearing's end nearest the axis is its "
          "INTERIOR pitch's, landed on exactly; the far end is a corner's") {
    // BUG CAUGHT (verification round): the bearing was evaluated at the pitch tolerance's two ends only;
    // the ray's forward part, cos p + yd sin p, is LARGEST where row v's ray is level — inside the
    // tolerance for a point near the lens's height — so the bearing nearest the axis was missed (up to
    // 6.4e-4 rad at a 3-degree tolerance, the verifier's sweep). By hand: a point LEFT of the axis at
    // column u has bearing yaw + atan(|xr| / f), xr = (u - cx) / fx; the smallest comes with the yaw at
    // its low end, the column jittered TOWARD the axis, fx at its high end and f at its largest —
    // sqrt(1 + yd^2), at pitch atan(yd), yd the row's offset over fy at its LOW end, the row jittered
    // AWAY from the centre (the larger |yd|: the row's own jitter is carried too).
    config::CameraPlacement pl = mount(10.0, 0.0);
    pl.pitchToleranceDeg = 3.0;
    const vision::CameraModel m{vision::kAiVisionOptics, pl};
    const double u = 60.0;
    const double v = 122.0;
    const double ydLow = (v + vision::kBoxEdgePx - 120.0) / (0.95 * m.fy());
    REQUIRE(std::atan(ydLow) < 3.0 * vision::kDegToRad);  // the level pitch is INSIDE the tolerance
    REQUIRE(std::atan(ydLow) > 0.0);
    const double fLevel = std::sqrt(1.0 + ydLow * ydLow);
    const double nearAxis =
        -1.0 * vision::kDegToRad + std::atan((160.0 - (u + vision::kBoxEdgePx)) / (1.05 * m.fx()) / fLevel);
    // The far end: the yaw high, the column jittered AWAY, fx low, f smallest — at a pitch END: the
    // lower pitch with the larger yd (cos p + yd sin p falls as p falls below atan(yd)).
    const double pLow = -3.0 * vision::kDegToRad;
    const double fSmall = std::cos(pLow) + ydLow * std::sin(pLow);
    const double farAxis = 1.0 * vision::kDegToRad
                           + std::atan((160.0 - (u - vision::kBoxEdgePx)) / (0.95 * m.fx()) / fSmall);
    const vision::Interval bi = m.bearing(u, v);
    CAPTURE(bi.lo);
    CAPTURE(bi.hi);
    CHECK(bi.lo == doctest::Approx(nearAxis).epsilon(1e-12));
    CHECK(bi.hi == doctest::Approx(farAxis).epsilon(1e-12));
    // The pitch ends alone put the near end about 1.75e-4 rad farther from the axis (the forward part at
    // the upper end, cos 3 + yd sin 3 deg, short of sqrt(1 + yd^2)): a true camera at the level pitch,
    // inside its stated tolerance, read outside the interval.
    const double fEnds = std::cos(3.0 * vision::kDegToRad) + ydLow * std::sin(3.0 * vision::kDegToRad);
    const double endsOnly = -1.0 * vision::kDegToRad
                            + std::atan((160.0 - (u + vision::kBoxEdgePx)) / (1.05 * m.fx()) / fEnds);
    CHECK(endsOnly - nearAxis > 1e-4);
}

TEST_CASE("VIS1-A1 geometry: every interval is the EXACT range over the WHOLE tolerance box -- a dense sweep "
          "(the pitch INSIDE its tolerance, the column through the principal point, each focal length's "
          "middle) never lies outside it and reaches both its ends") {
    // BUG CAUGHT (verification round): the header said every output was monotone over the box "while the
    // box does not straddle the horizon; one that does is refused" — nothing refused, and two outputs were
    // not monotone: the bearing in the pitch (the forward part largest where the row's ray is level) and
    // the size range in the pitch (a box straddling the lens's level: the rise between its rows smallest
    // with the box symmetric about level — 0.84 % short at a 5-degree tolerance) and in the column (the
    // line of sight square to the image on the principal point's column). The fix round's probes ran at
    // 98 % of every tolerance; this sweep runs at 100 % and in between. The per-corner answers come from
    // the public ray() and the size range's documented formula; what is tested is the hull over the box.
    constexpr double kE = vision::kBoxEdgePx;
    int points = 0;
    int boxes = 0;
    int grounds = 0;
    for (const double pitch0 : {0.0, 10.0, -12.0}) {
        for (const double dp : {1.0, 3.0}) {
            CAPTURE(pitch0);
            CAPTURE(dp);
            config::CameraPlacement pl = mount(10.0, pitch0);
            pl.pitchToleranceDeg = dp;
            const vision::CameraModel m{vision::kAiVisionOptics, pl};
            const vision::CameraCorner n = m.nominal();
            const double levelRow = n.cy + n.fy * std::tan(pitch0 * vision::kDegToRad);
            auto sweep = [&](bool withYaw, auto&& fn) {
                for (const double sx : {0.95, 1.0, 1.05}) {
                    for (const double sy : {0.95, 1.0, 1.05}) {
                        for (int ip = 0; ip <= 40; ++ip) {
                            for (const double yaw : {-1.0, 1.0}) {
                                fn(vision::CameraCorner{n.fx * sx, n.fy * sy, n.cx, n.cy, n.h,
                                                        n.pitch + dp * vision::kDegToRad * (ip - 20) / 20.0,
                                                        n.yaw + yaw * vision::kDegToRad});
                                if (!withYaw) {
                                    break;
                                }
                            }
                        }
                    }
                }
            };
            // Columns 159 and 160.5: the jittered column reaches the principal point's (160), ON the
            // sweep's 0.5-pixel grid.
            for (const double u : {40.0, 159.0, 160.5, 290.0}) {
                for (const double v : {levelRow, 200.0}) {
                    CAPTURE(u);
                    CAPTURE(v);
                    double bLo = 1e9;
                    double bHi = -1e9;
                    double eLo = 1e9;
                    double eHi = -1e9;
                    sweep(true, [&](const vision::CameraCorner& c) {
                        for (int k = 0; k <= 8; ++k) {      // the column, 0.5 px apart
                            for (int j = 0; j <= 4; ++j) {  // the row, 1 px apart
                                double b = 0.0;
                                double e = 0.0;
                                vision::CameraModel::ray(c, u - kE + 0.5 * k, v - kE + 1.0 * j, b, e);
                                bLo = std::fmin(bLo, b);
                                bHi = std::fmax(bHi, b);
                                eLo = std::fmin(eLo, e);
                                eHi = std::fmax(eHi, e);
                            }
                        }
                    });
                    ++points;
                    const vision::Interval bi = m.bearing(u, v);
                    CHECK(bi.lo <= bLo + 1e-12);
                    CHECK(bi.hi >= bHi - 1e-12);
                    CHECK(bi.lo >= bLo - 1e-6);
                    CHECK(bi.hi <= bHi + 1e-6);
                    const vision::Interval ei = m.elevation(u, v);
                    CHECK(ei.lo <= eLo + 1e-12);
                    CHECK(ei.hi >= eHi - 1e-12);
                    CHECK(ei.lo >= eLo - 1e-9);
                    CHECK(ei.hi <= eHi + 1e-9);
                    // The ground range: the same rays at both lens heights (a point below the level only).
                    double gLo = 1e9;
                    double gHi = -1e9;
                    bool steep = true;
                    sweep(false, [&](const vision::CameraCorner& c0) {
                        for (const double dh : {-0.25, 0.25}) {
                            vision::CameraCorner c = c0;
                            c.h += dh;
                            for (int k = 0; k <= 8; ++k) {
                                for (int j = 0; j <= 4; ++j) {
                                    double b = 0.0;
                                    double e = 0.0;
                                    vision::CameraModel::ray(c, u - kE + 0.5 * k, v - kE + 1.0 * j, b, e);
                                    steep = steep && -e > 2.0 * vision::kDegToRad;
                                    gLo = std::fmin(gLo, c.h / std::tan(-e));
                                    gHi = std::fmax(gHi, c.h / std::tan(-e));
                                }
                            }
                        }
                    });
                    const vision::Interval gi = m.groundRange(u, v);
                    CHECK(gi.valid == steep);
                    if (steep) {
                        ++grounds;
                        CHECK(gi.lo <= gLo * (1.0 + 1e-12));
                        CHECK(gi.hi >= gHi * (1.0 - 1e-12));
                        CHECK(gi.lo >= gLo * (1.0 - 1e-12));
                        CHECK(gi.hi <= gHi * (1.0 + 1e-12));
                    }
                    for (const double h : {20.0, 60.0}) {
                        CAPTURE(h);
                        double sLo = 1e9;
                        double sHi = -1e9;
                        sweep(false, [&](const vision::CameraCorner& c) {  // the yaw changes no size range
                            for (int k = 0; k <= 8; ++k) {
                                for (const double dt : {-kE, 0.0, kE}) {
                                    for (const double db : {-kE, 0.0, kE}) {
                                        const double s = sizeAt(c, 6.5, u - kE + 0.5 * k, v, v - h / 2.0 + dt,
                                                                v + h / 2.0 + db);
                                        sLo = std::fmin(sLo, s);
                                        sHi = std::fmax(sHi, s);
                                    }
                                }
                            }
                        });
                        ++boxes;
                        const vision::Interval si = m.sizeRange(6.5, u, v, h);
                        CAPTURE(si.lo);
                        CAPTURE(si.hi);
                        CAPTURE(sLo);
                        CAPTURE(sHi);
                        CHECK(si.lo <= sLo * (1.0 + 1e-12));
                        CHECK(si.hi >= sHi * (1.0 - 1e-12));
                        CHECK(si.lo >= sLo * (1.0 - 5e-6));
                        CHECK(si.hi <= sHi * (1.0 + 5e-6));
                    }
                }
            }
        }
    }
    CHECK(points == 3 * 2 * 4 * 2);
    CHECK(boxes == 2 * points);
    MESSAGE(grounds << " ground ranges swept (the rest near or above the level: refused, as swept)");
    CHECK(grounds >= 12);
}

TEST_CASE("VIS1-A1 geometry: a box STRADDLING the lens's level near the picture's EDGE -- the size range's far "
          "end IS the largest range over the pitch, landed on (this test's own golden-section search)") {
    // The far end comes from the pitch INSIDE the tolerance where the product — the sideways factor times
    // the rise between the rows — is smallest. Off the axis that is NOT where the rise alone is smallest
    // (the sideways factor turns with the pitch too), so a search on the rise alone falls short. The far
    // end's corner, by hand: fx low and the column jittered AWAY from the axis (the smallest sideways
    // factor), the box shrunk by an edge at each end (the smallest rise), each fy end — at the pitch that
    // maximises the range, found here by golden section to 1e-13 rad.
    constexpr double kE = vision::kBoxEdgePx;
    config::CameraPlacement pl = mount(10.0, 10.0);
    pl.pitchToleranceDeg = 3.0;
    const vision::CameraModel m{vision::kAiVisionOptics, pl};
    const vision::CameraCorner n = m.nominal();
    const double levelRow = n.cy + n.fy * std::tan(10.0 * vision::kDegToRad);
    const double u = 300.0;
    const double h = 60.0;
    const double top = levelRow - h / 2.0 + kE;
    const double bottom = levelRow + h / 2.0 - kE;
    const double pLo = n.pitch - 3.0 * vision::kDegToRad;
    const double pHi = n.pitch + 3.0 * vision::kDegToRad;
    double best = 0.0;
    double riseOnlyShort = 0.0;
    for (const double sy : {0.95, 1.05}) {
        vision::CameraCorner c{n.fx * 0.95, n.fy * sy, n.cx, n.cy, n.h, n.pitch, n.yaw};
        auto range = [&](double p) {
            c.pitch = p;
            return sizeAt(c, 6.5, u + kE, levelRow, top, bottom);
        };
        const double g = 0.5 * (std::sqrt(5.0) - 1.0);
        double a = pLo;
        double b = pHi;
        while (b - a > 1e-13) {
            const double x1 = b - g * (b - a);
            const double x2 = a + g * (b - a);
            if (range(x1) < range(x2)) {
                a = x1;
            } else {
                b = x2;
            }
        }
        const double peak = range(0.5 * (a + b));
        REQUIRE(peak > range(pLo));  // the far end is INSIDE the pitch tolerance, not at a corner
        REQUIRE(peak > range(pHi));
        best = std::fmax(best, peak);
        // Where the rise between the rows alone is smallest: the box symmetric about level.
        const double riseOnly = 0.5 * (std::atan((top - c.cy) / c.fy) + std::atan((bottom - c.cy) / c.fy));
        riseOnlyShort = std::fmax(riseOnlyShort, (peak - range(riseOnly)) / peak);
    }
    const vision::Interval si = m.sizeRange(6.5, u, levelRow, h);
    CAPTURE(si.hi);
    CAPTURE(best);
    CHECK(si.hi >= best * (1.0 - 1e-12));  // never short
    CHECK(si.hi <= best * (1.0 + 1e-12));  // and exact
    MESSAGE("the rise alone's pitch falls " << riseOnlyShort << " (relative) short of the far end");
    CHECK(riseOnlyShort > 1e-7);
}

TEST_CASE("VIS1-A1 geometry: a thin line STRADDLING the lens's level -- its size range holds the truth for "
          "EVERY true camera of the box, the pitch INSIDE its tolerance included, every other tolerance at "
          "100 %, both edges and the column jittered by EXACTLY kBoxEdgePx") {
    // BUG CAUGHT (verification round): the verifier's scene — an upright line centred at the lens's height,
    // the true pitch inside the tolerance, fy at +5 %, both edges jittered outward by exactly kBoxEdgePx:
    // every one missed its far end (0.029 % at a 1-degree tolerance, 0.84 % at 5). The truth is projected
    // here by this file's OWN pinhole (neither the library's project() nor the host camera model's), at
    // sub-pixel precision, so no rounding hides a shortfall.
    auto handProject = [](double fx, double fy, double pitch, double X, double Y, double Z, double& u,
                          double& v) {
        const double a = X - 7.0;   // the lens: 7 in forward, on the centreline, 10 in up, yaw 0
        const double s = Y;
        const double dz = Z - 10.0;
        const double depth = a * std::cos(pitch) + dz * std::sin(pitch);
        u = 160.0 - fx * s / depth;
        v = 120.0 + fy * (a * std::sin(pitch) - dz * std::cos(pitch)) / depth;
        return depth > 1e-6;
    };
    constexpr double kE = vision::kBoxEdgePx;
    int seen = 0;
    int held = 0;
    for (const double pitch0 : {0.0, 10.0, -12.0}) {
        for (const double dp : {1.0, 3.0, 5.0}) {
            config::CameraPlacement pl = mount(10.0, pitch0);
            pl.pitchToleranceDeg = dp;
            const vision::CameraModel m{vision::kAiVisionOptics, pl};
            for (const double sx : {0.95, 1.05}) {
                for (const double sy : {0.95, 1.05}) {
                    for (int s = -4; s <= 4; ++s) {
                        const double pitch = (pitch0 + dp * s / 4.0) * vision::kDegToRad;
                        for (const double bearingDeg : {-25.0, 0.0, 18.0}) {
                            for (const double dist : {18.0, 40.0}) {
                                const double X = 7.0 + dist * std::cos(bearingDeg * vision::kDegToRad);
                                const double Y = dist * std::sin(bearingDeg * vision::kDegToRad);
                                double ut = 0.0;
                                double vt = 0.0;
                                double ub = 0.0;
                                double vb = 0.0;
                                REQUIRE(handProject(m.fx() * sx, m.fy() * sy, pitch, X, Y, 13.0, ut, vt));
                                REQUIRE(handProject(m.fx() * sx, m.fy() * sy, pitch, X, Y, 7.0, ub, vb));
                                const double top = vt - kE;  // grown by exactly an edge at each end
                                const double bottom = vb + kE;
                                const double uc = 0.5 * (ut + ub);
                                const double col = uc + (uc < 160.0 ? kE : -kE);  // moved TOWARD the axis
                                const vision::Interval r =
                                    m.sizeRange(6.0, col, 0.5 * (top + bottom), bottom - top);
                                ++seen;
                                if (r.valid && dist >= r.lo * (1.0 - 1e-12) && dist <= r.hi * (1.0 + 1e-12)) {
                                    ++held;
                                } else {
                                    MESSAGE("straddle miss: pitch " << pitch0 << " +- " << dp << " (true "
                                            << pitch / vision::kDegToRad << ") sx " << sx << " sy " << sy
                                            << " bearing " << bearingDeg << " dist " << dist << ": [" << r.lo
                                            << ", " << r.hi << "]");
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    MESSAGE("straddling lines: " << seen << " measured, " << held << " held their true range");
    CHECK(seen == 3 * 3 * 2 * 2 * 9 * 3 * 2);
    CHECK(held == seen);
}

TEST_CASE("VIS1-A1 geometry: in view FOR CERTAIN means every corner of the tolerance box") {
    const vision::CameraModel m{vision::kAiVisionOptics, mount(10.0, 0.0)};
    CHECK(m.inViewForCertain(7.0 + 40.0, 0.0, 0.0, 4.0));
    CHECK_FALSE(m.inViewForCertain(7.0 - 5.0, 0.0, 0.0, 4.0));   // behind the lens
    CHECK_FALSE(m.inViewForCertain(7.0 + 40.0, 40.0, 0.0, 4.0));  // 45 degrees left: outside 37
    CHECK_FALSE(m.inViewForCertain(7.0 + 5.0, 0.0, 0.0, 4.0));    // the floor 5 in out, under the view
    // Just inside the nominal edge is not "for certain" once the tolerances are counted.
    const double edgeLeft = 40.0 * std::tan(36.5 * vision::kDegToRad);
    CHECK_FALSE(m.inViewForCertain(7.0 + 40.0, edgeLeft, 10.0, 0.0));
    // The margin LANDED ON (review X19): a point near the left edge, its worst corner's column computed
    // through the same projection; a margin of exactly that column is in view, one a hair wider is not.
    const double fwd = 7.0 + 40.0;
    const double lft = 22.0;
    double minU = 1e9;
    double minV = 1e9;
    double maxV = -1e9;
    double maxU = -1e9;
    m.eachCorner(true, [&](const vision::CameraCorner& c) {
        double u = 0.0;
        double v = 0.0;
        REQUIRE(m.project(c, fwd, lft, 0.0, u, v));
        minU = std::fmin(minU, u);
        maxU = std::fmax(maxU, u);
        minV = std::fmin(minV, v);
        maxV = std::fmax(maxV, v);
    });
    CAPTURE(minU);
    REQUIRE(minU > 1.0);
    REQUIRE(minV > minU);  // the column is the binding edge
    REQUIRE(maxV < 240.0 - minU);
    REQUIRE(maxU < 320.0 - minU);
    CHECK(m.inViewForCertain(fwd, lft, 0.0, minU));
    CHECK_FALSE(m.inViewForCertain(fwd, lft, 0.0, std::nextafter(minU, 1e9)));
}

TEST_CASE("VIS1-A1 geometry R6.3: boundAtImage -- no motion changes nothing; motion adds EXACTLY |v| L and "
          "|w| L; a bad input is never a tight bound") {
    const vision::PoseBound b = poseAt(1.0, 2.0, 0.3, 1.0, 1.0);
    const vision::PoseBound still = vision::boundAtImage(b, 0.0, 0.0, 0.1);
    CHECK(still.positionTolerance.value() == doctest::Approx(1.0));
    CHECK(still.headingToleranceDeg == doctest::Approx(1.0));
    const vision::PoseBound noAge = vision::boundAtImage(b, 60.0, 180.0, 0.0);
    CHECK(noAge.positionTolerance.value() == doctest::Approx(1.0));
    CHECK(noAge.headingToleranceDeg == doctest::Approx(1.0));
    const vision::PoseBound moving = vision::boundAtImage(b, -30.0, -90.0, 0.1);  // the signs do not matter
    CHECK(moving.positionTolerance.value() == doctest::Approx(4.0));
    CHECK(moving.headingToleranceDeg == doctest::Approx(10.0));
    CHECK(moving.pose.x().value() == doctest::Approx(1.0));
    CHECK(moving.pose.y().value() == doctest::Approx(2.0));
    CHECK(moving.pose.heading().radians() == doctest::Approx(0.3));
    const vision::PoseBound fast = vision::boundAtImage(b, 60.0, 180.0, 0.15);
    CHECK(fast.positionTolerance.value() == doctest::Approx(10.0));
    CHECK(fast.headingToleranceDeg == doctest::Approx(28.0));

    SUBCASE("a negative or non-finite age, speed or turn rate: both tolerances infinite") {
        for (const double bad : {-0.01, kNaN, kInf}) {
            CAPTURE(bad);
            const vision::PoseBound w = vision::boundAtImage(b, 10.0, 10.0, bad);
            CHECK(std::isinf(w.positionTolerance.value()));
            CHECK(std::isinf(w.headingToleranceDeg));
        }
        for (const double bad : {kNaN, kInf, -kInf}) {
            CAPTURE(bad);
            CHECK(std::isinf(vision::boundAtImage(b, bad, 10.0, 0.1).positionTolerance.value()));
            CHECK(std::isinf(vision::boundAtImage(b, 10.0, bad, 0.1).headingToleranceDeg));
        }
    }
    SUBCASE("a pose stated EXACT (both tolerances 0) is a bound: widened to exactly |v| L and |w| L, never "
            "infinite (review X14, X15 -- the edge landed on)") {
        const vision::PoseBound exact = vision::boundAtImage(poseAt(0.0, 0.0, 0.0, 0.0, 0.0), 10.0, 10.0, 0.1);
        CHECK(exact.positionTolerance.value() == doctest::Approx(1.0));
        CHECK(exact.headingToleranceDeg == doctest::Approx(1.0));
        const vision::PoseBound parked = vision::boundAtImage(poseAt(0.0, 0.0, 0.0, 0.0, 0.0), 0.0, 0.0, 0.1);
        CHECK(parked.positionTolerance.value() == 0.0);
        CHECK(parked.headingToleranceDeg == 0.0);
    }
    SUBCASE("a negative or non-finite tolerance in the bound it widens: infinite, never smaller") {
        CHECK(std::isinf(vision::boundAtImage(poseAt(0, 0, 0, -1.0, 1.0), 10.0, 10.0, 0.1)
                             .positionTolerance.value()));
        CHECK(std::isinf(vision::boundAtImage(poseAt(0, 0, 0, 1.0, -1.0), 10.0, 10.0, 0.1)
                             .headingToleranceDeg));
        CHECK(std::isinf(vision::boundAtImage(poseAt(0, 0, 0, kNaN, 1.0), 10.0, 10.0, 0.1)
                             .positionTolerance.value()));
    }
}

TEST_CASE("VIS1-A1 geometry R6.1: turning 180 deg/s, a Pin 36 in ahead -- the fix from the pose at READ "
          "completion misses the true Pin once the image is ~66 ms old; the bound AT THE IMAGE always holds "
          "it") {
    int missed = 0;
    for (const double lagMs : {0.0, 33.0, 66.0, 100.0, 150.0}) {
        CAPTURE(lagMs);
        double e0 = 0.0;
        double a0 = 0.0;
        double e1 = 0.0;
        double a1 = 0.0;
        REQUIRE(laggedFix(lagMs / 1000.0, 0.0, 180.0, false, 0.0, e0, a0));
        REQUIRE(laggedFix(lagMs / 1000.0, 0.0, 180.0, true, 0.0, e1, a1));
        MESSAGE("R6.1 lag " << lagMs << " ms: at read err " << e0 << " allowed " << a0 << "; at image err "
                            << e1 << " allowed " << a1);
        if (e0 > a0) {
            ++missed;
        }
        CHECK(e1 <= a1);
    }
    CHECK(missed >= 2);  // the finding: un-widened, the "bound" is not one
}

TEST_CASE("VIS1-A1 geometry R6.2: driving 60 in/s past a Pin 14 in aside -- un-widened it misses at the "
          "longer lags; at the image it always holds") {
    int missed = 0;
    for (const double lagMs : {33.0, 66.0, 100.0, 150.0}) {
        CAPTURE(lagMs);
        double e0 = 0.0;
        double a0 = 0.0;
        double e1 = 0.0;
        double a1 = 0.0;
        REQUIRE(laggedFix(lagMs / 1000.0, 60.0, 0.0, false, 14.0, e0, a0));
        REQUIRE(laggedFix(lagMs / 1000.0, 60.0, 0.0, true, 14.0, e1, a1));
        MESSAGE("R6.2 lag " << lagMs << " ms: at read err " << e0 << " allowed " << a0 << "; at image err "
                            << e1 << " allowed " << a1);
        if (e0 > a0) {
            ++missed;
        }
        CHECK(e1 <= a1);
    }
    CHECK(missed >= 1);
}

TEST_CASE("VIS1-A1 config: the mount's checks, on both sides of every range") {
    CHECK(problem(mount(10.0, 0.0)) == "(none)");
    CHECK(problem(config::CameraPlacement{}) == "the camera's mount is UNSET -- measure it from the robot's tracking centre and type it");

    config::CameraPlacement p = mount(10.0, 0.0);
    for (const double h : {0.0, -1.0, 36.0, 40.0, kNaN}) {
        CAPTURE(h);
        p.height = Length{h};
        CHECK(problem(p) == "camera height outside (0, 36) in");
    }
    for (const double h : {0.01, 35.99}) {
        p.height = Length{h};
        CHECK(problem(p) == "(none)");
    }
    p = mount(10.0, 0.0);
    for (const double pitch : {45.0, -45.0, 60.0, kNaN}) {
        CAPTURE(pitch);
        p.pitchDeg = pitch;
        CHECK(problem(p) == "pitch outside +-45 degrees");
    }
    p.pitchDeg = 44.9;
    CHECK(problem(p) == "(none)");
    p.pitchDeg = -44.9;
    CHECK(problem(p) == "(none)");
    p = mount(10.0, 0.0);
    for (const double yaw : {-180.0, 180.01, kNaN}) {
        CAPTURE(yaw);
        p.yawDeg = yaw;
        CHECK(problem(p) == "yaw outside (-180, 180] degrees");
    }
    p.yawDeg = 180.0;
    CHECK(problem(p) == "(none)");
    p.yawDeg = -179.99;
    CHECK(problem(p) == "(none)");
    p = mount(10.0, 0.0);
    p.forward = Length{kNaN};
    CHECK(problem(p) == "the camera's forward or left offset is not a number");
    p = mount(10.0, 0.0);
    p.left = Length{kInf};
    CHECK(problem(p) == "the camera's forward or left offset is not a number");
    p = mount(10.0, 0.0);
    p.forward = Length{-6.0};  // behind the tracking centre is a place, not a mistake
    p.left = Length{-3.0};
    CHECK(problem(p) == "(none)");

    const char* kTol = "a mount tolerance is zero, UNSET or not a number -- state how well it was measured";
    for (int which = 0; which < 4; ++which) {
        for (const double bad : {0.0, -0.1, kNaN, kInf}) {
            CAPTURE(which);
            CAPTURE(bad);
            p = mount(10.0, 0.0);
            if (which == 0) {
                p.positionTolerance = Length{bad};
            } else if (which == 1) {
                p.heightTolerance = Length{bad};
            } else if (which == 2) {
                p.yawToleranceDeg = bad;
            } else {
                p.pitchToleranceDeg = bad;
            }
            CHECK(problem(p) == kTol);
        }
    }
    p = mount(10.0, 0.0);
    p.provenance = "";
    CHECK(problem(p) == "no provenance -- who measured the mount, how, when?");
    p.provenance = nullptr;
    CHECK(problem(p) == "no provenance -- who measured the mount, how, when?");
}

TEST_CASE("VIS1-A1 geometry: cameraModelProblem -- no camera, an UNSET mount, a bad one; a measured one "
          "builds") {
    config::CameraConfig c{};
    CHECK(std::string{vision::cameraModelProblem(c)} == "no camera is declared");
    c.port = 3;
    CHECK(std::string{vision::cameraModelProblem(c)}
          == "the camera's mount is UNSET -- measure it from the robot's tracking centre and type it");
    c.placement = mount(10.0, 0.0);
    CHECK(vision::cameraModelProblem(c) == nullptr);
    const vision::CameraModel m{vision::kAiVisionOptics, c.placement};
    CHECK(m.placement().height.value() == doctest::Approx(10.0));
    c.placement.pitchDeg = 50.0;
    CHECK(std::string{vision::cameraModelProblem(c)} == "pitch outside +-45 degrees");
    // The geometry's OWN limit (verification round): the pitch WITH its tolerance at most
    // kMaxPitchSpanDeg — on the edge exactly (44 + 11 = 55, binary-exact) it builds; a hair past it, either
    // way, it refuses — while the robot file's own check still accepts the mount (the camera is built and
    // read; only the geometry refuses).
    const std::string kSpan = "the pitch with its tolerance reaches past 55 degrees -- the geometry holds only "
                              "inside it (re-measure the tolerance: a level on the camera's back face reads "
                              "about 1 degree)";
    CHECK(vision::kMaxPitchSpanDeg == 55.0);
    for (const double pitch : {44.0, -44.0}) {
        CAPTURE(pitch);
        c.placement = mount(10.0, pitch);
        c.placement.pitchToleranceDeg = 11.0;
        CHECK(vision::cameraModelProblem(c) == nullptr);
        // The smallest tolerance whose sum with 44 is past 55 (nextafter(11, 12) still sums to 55.0).
        c.placement.pitchToleranceDeg = std::nextafter(55.0, 56.0) - 44.0;
        REQUIRE(44.0 + c.placement.pitchToleranceDeg > 55.0);
        const char* past = vision::cameraModelProblem(c);
        REQUIRE(past != nullptr);
        CHECK(std::string{past} == kSpan);
        CHECK(config::cameraPlacementProblem(c.placement) == nullptr);
    }
    c.placement = mount(10.0, 0.0);
    c.placement.pitchToleranceDeg = 56.0;
    CHECK(std::string{vision::cameraModelProblem(c)} == kSpan);
    // A mount with no camera is still "no camera".
    c.port = 0;
    CHECK(std::string{vision::cameraModelProblem(c)} == "no camera is declared");
}

TEST_CASE("VIS1-A1 geometry: the Interval's own arithmetic") {
    const vision::Interval i{true, 2.0, 6.0};
    CHECK(i.mid() == doctest::Approx(4.0));
    CHECK(i.width() == doctest::Approx(4.0));
    CHECK(i.contains(2.0));
    CHECK(i.contains(6.0));
    CHECK_FALSE(i.contains(6.0000001));
    CHECK_FALSE(i.contains(1.9999999));
    const vision::Interval invalid{false, 2.0, 6.0};
    CHECK_FALSE(invalid.contains(4.0));
}
