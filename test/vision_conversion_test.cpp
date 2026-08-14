// Tests for hal/vision_conversion.hpp — the corners->robot-relative-pose PnP (chunk E3, T3).
//
// ── WHY THIS FILE IS SHAPED THE WAY IT IS ────────────────────────────────────────────────────
// The single most dangerous test one can write here is a round-trip through a projector that
// SHARES A CAMERA MODEL with the PnP under test. An intrinsics error, a flipped image axis or a
// swapped yaw sign then cancels exactly, and the test passes while the code is wrong. That is
// the failure that bit C1, C3, C4 and E2's GPS frame tests, and this is the file most exposed
// to it.
//
// So the evidence here comes in THREE independent layers:
//
//   LAYER 1 — HAND-WORKED PIXELS.  Literal pixel coordinates, computed on paper from the
//   geometry (the arithmetic is written out in each case's comment), handed straight to the PnP.
//   These share nothing at all with any projector: if the PnP's camera->body mapping is wrong,
//   nothing cancels, because there is no second implementation involved.
//
//   LAYER 2 — THE PROJECTOR IS ITSELF PINNED.  `project()` below is written OUTWARD (field ->
//   body -> camera -> pixels) from the geometry, where the PnP works INWARD (pixels ->
//   homography -> camera -> body). Before it is used as an oracle it is checked against the
//   Layer-1 hand-computed pixels, so it is a verified instrument rather than a mirror.
//
//   LAYER 3 — THE SWEEP.  Only then is the projector used to sweep poses, headings, mounts and
//   ranges, which is what catches the errors too fiddly to do on paper.
//
// Deliberately NOT used anywhere in this file: shulib::math::Pose2d composition helpers, the
// TagMap inversion, or any part of vision_conversion.hpp itself. The scene geometry is rebuilt
// from words each time.

#include "doctest.h"

#include <cmath>
#include <cstddef>
#include <limits>

#include "shulib/hal/vision_conversion.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/units/quantity.hpp"

using shulib::hal::CameraIntrinsics;
using shulib::hal::CameraMount;
using shulib::hal::TagCorners;
using shulib::hal::tagCornersToRobotPose;
using shulib::hal::TagPnpResult;
using shulib::math::Angle;
using shulib::units::Length;

namespace {

constexpr double kPi = Angle::kPi;
constexpr double kDeg = kPi / 180.0;

/// The reference camera used throughout: 600 px focal length, 640x480 principal point.
/// Chosen so the hand arithmetic stays exact-ish (600/24 = 25 px per inch at 24 inches).
const CameraIntrinsics kCam{600.0, 600.0, 320.0, 240.0};

/// A scene, described the way a person would describe it out loud.
struct Scene {
    double robotX = 0.0;       ///< field, inches
    double robotY = 0.0;
    double robotHeading = 0.0; ///< field, radians, CCW from +X
    double mountX = 0.0;       ///< camera position in the ROBOT body frame, inches
    double mountY = 0.0;
    double mountYaw = 0.0;     ///< optical-axis direction in the body frame, radians CCW from +X
    double tagX = 0.0;         ///< field, inches
    double tagY = 0.0;
    double tagFacing = 0.0;    ///< field, radians: where the tag's OUTWARD normal points
    double tagHeight = 0.0;    ///< tag centre height ABOVE the camera, inches
    double tagSize = 6.0;      ///< tag edge length, inches
};

/// THE INDEPENDENT PROJECTOR (Layer 2). Built from the geometry outward, in four named steps,
/// with no reference to vision_conversion.hpp.
///
/// Step A — the tag's four corners in FIELD 3D. The tag face is vertical. A viewer standing in
/// front of the tag looks along -normal; their RIGHT is that look direction rotated 90 degrees
/// clockwise, and rotating (a, b) clockwise gives (b, -a), so
///        look  = (-cos phi, -sin phi)   =>   right = (-sin phi, cos phi).
/// Sanity, done in words: a tag on the far wall facing back at the robot has phi = 180 degrees,
/// giving right = (0, -1) — which is indeed the robot's right when the robot faces +X. "Up" is
/// world +Z. Corner k is centre + ax*right + az*up, with (ax, az) walking TL, TR, BR, BL.
///
/// Step B — field -> robot body. Translate, then rotate by -robotHeading.
/// Step C — body -> camera. Translate by the mount, then resolve onto the optical axis
/// (+Z_cam) and camera-right (+X_cam). Camera-right is 90 degrees clockwise of the optical
/// axis, and +Y is LEFT in the body frame, so right = (sin psi, -cos psi). +Y_cam is DOWN, so
/// it is the NEGATIVE of the height above the camera.
/// Step D — pinhole.
[[nodiscard]] TagCorners project(const Scene& s, const CameraIntrinsics& cam) {
    const double half = 0.5 * s.tagSize;
    const double rightX = -std::sin(s.tagFacing);
    const double rightY = std::cos(s.tagFacing);
    const double ax[4] = {-half, half, half, -half};   // TL, TR, BR, BL along the tag's right
    const double az[4] = {half, half, -half, -half};   // TL, TR, BR, BL along world up

    TagCorners out;
    for (std::size_t k = 0; k < 4; ++k) {
        // Step A
        const double fX = s.tagX + ax[k] * rightX;
        const double fY = s.tagY + ax[k] * rightY;
        const double fZ = s.tagHeight + az[k];
        // Step B
        const double dx = fX - s.robotX;
        const double dy = fY - s.robotY;
        const double bX = dx * std::cos(s.robotHeading) + dy * std::sin(s.robotHeading);
        const double bY = -dx * std::sin(s.robotHeading) + dy * std::cos(s.robotHeading);
        // Step C
        const double ex = bX - s.mountX;
        const double ey = bY - s.mountY;
        const double zc = ex * std::cos(s.mountYaw) + ey * std::sin(s.mountYaw);
        const double xc = ex * std::sin(s.mountYaw) - ey * std::cos(s.mountYaw);
        const double yc = -fZ;
        // Step D
        out.u[k] = cam.fx * xc / zc + cam.cx;
        out.v[k] = cam.fy * yc / zc + cam.cy;
    }
    return out;
}

/// The tag's pose RELATIVE to the robot, stated directly from the scene — the expectation the
/// PnP must reproduce. Same two-line rigid transform as Step B, and nothing else.
struct RelPose {
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
};
[[nodiscard]] RelPose expectedRelative(const Scene& s) {
    const double dx = s.tagX - s.robotX;
    const double dy = s.tagY - s.robotY;
    RelPose r;
    r.x = dx * std::cos(s.robotHeading) + dy * std::sin(s.robotHeading);
    r.y = -dx * std::sin(s.robotHeading) + dy * std::cos(s.robotHeading);
    r.heading = std::remainder(s.tagFacing - s.robotHeading, 2.0 * kPi);
    return r;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────────────────────
// LAYER 1 — hand-worked pixels. No projector involved.
// ─────────────────────────────────────────────────────────────────────────────────────────────

// Would catch: any error in the camera->body reduction — a swapped axis, a flipped image
// direction, a lost tag depth — on the simplest possible geometry. THE anchor case: everything
// else in this file is a perturbation of it.
//
// THE ARITHMETIC, done on paper. A 6-inch tag 24 inches straight ahead, facing the robot,
// camera at the robot's origin looking forward, tag centre at camera height. Its corners are at
// body (24, +-3, +-3), i.e. camera (X, Y, Z) = (-+3, -+3, 24). With fx = fy = 600:
//     u = 600 * (+-3) / 24 + 320 = 320 -+ 75   ->   245 and 395
//     v = 600 * (+-3) / 24 + 240 = 240 -+ 75   ->   165 and 315
// so the image is a 150 x 150 px square centred in the frame, TL first, going clockwise.
TEST_CASE("tagPnp: hand-computed head-on square -> 24 inches ahead, facing us") {
    TagCorners c;
    c.u[0] = 245.0; c.v[0] = 165.0;  // TL
    c.u[1] = 395.0; c.v[1] = 165.0;  // TR
    c.u[2] = 395.0; c.v[2] = 315.0;  // BR
    c.u[3] = 245.0; c.v[3] = 315.0;  // BL

    const TagPnpResult r = tagCornersToRobotPose(c, kCam, Length{6.0}, CameraMount{});
    REQUIRE(r.valid);
    CHECK(r.poseInRobot.x().value() == doctest::Approx(24.0).epsilon(1e-9));
    CHECK(r.poseInRobot.y().value() == doctest::Approx(0.0).epsilon(1e-9).scale(1.0));
    // The tag faces BACK at us: its outward normal points along the robot's -X.
    CHECK(r.poseInRobot.heading().degrees() == doctest::Approx(180.0).epsilon(1e-7));
    CHECK(r.range.value() == doctest::Approx(24.0).epsilon(1e-9));
    CHECK(r.reprojectionError == doctest::Approx(0.0).scale(1.0));
}

// Would catch: a sign error on the camera's mounting YAW — the error that makes the whole
// localizer mirror-image wrong the moment a camera is not pointed dead ahead, and that a
// yaw = 0 test can never see.
//
// SAME PIXELS as the anchor case, but the camera is bolted on turned 30 degrees to the LEFT.
// The pixels say "the tag is 24 inches along the optical axis, dead centre". The optical axis
// points 30 degrees left of the robot's nose, so the tag must be at body
//     (24*cos30, 24*sin30) = (20.784609690826528, 12)
// and it faces back along that axis: 30 + 180 = 210 degrees, which wraps to -150.
TEST_CASE("tagPnp: hand-computed — a camera yawed 30 degrees left moves the tag to the left") {
    TagCorners c;
    c.u[0] = 245.0; c.v[0] = 165.0;
    c.u[1] = 395.0; c.v[1] = 165.0;
    c.u[2] = 395.0; c.v[2] = 315.0;
    c.u[3] = 245.0; c.v[3] = 315.0;

    CameraMount mount;
    mount.yaw = Angle::degrees(30.0);
    const TagPnpResult r = tagCornersToRobotPose(c, kCam, Length{6.0}, mount);
    REQUIRE(r.valid);
    CHECK(r.poseInRobot.x().value() == doctest::Approx(20.784609690826528).epsilon(1e-9));
    CHECK(r.poseInRobot.y().value() == doctest::Approx(12.0).epsilon(1e-9));
    CHECK(r.poseInRobot.heading().degrees() == doctest::Approx(-150.0).epsilon(1e-7));
    CHECK(r.range.value() == doctest::Approx(24.0).epsilon(1e-9));
}

// Would catch: the camera's mounting OFFSET being dropped, double-applied or applied in the
// wrong frame — the AprilTag analogue of the GPS lever arm that gps_conversion.hpp calls out as
// "inches of silent bias" (E2's T4). A tag 24 inches ahead of a camera mounted 5 inches forward
// and 2 inches right of centre is 29 inches ahead of, and 2 inches right of, the robot's centre.
TEST_CASE("tagPnp: hand-computed — the camera mount offset lands in the body frame") {
    TagCorners c;
    c.u[0] = 245.0; c.v[0] = 165.0;
    c.u[1] = 395.0; c.v[1] = 165.0;
    c.u[2] = 395.0; c.v[2] = 315.0;
    c.u[3] = 245.0; c.v[3] = 315.0;

    CameraMount mount;
    mount.x = Length{5.0};
    mount.y = Length{-2.0};  // +Y is LEFT, so this is 2 inches to the RIGHT
    const TagPnpResult r = tagCornersToRobotPose(c, kCam, Length{6.0}, mount);
    REQUIRE(r.valid);
    CHECK(r.poseInRobot.x().value() == doctest::Approx(29.0).epsilon(1e-9));
    CHECK(r.poseInRobot.y().value() == doctest::Approx(-2.0).epsilon(1e-9));
    CHECK(r.poseInRobot.heading().degrees() == doctest::Approx(180.0).epsilon(1e-7));
    // The RANGE is measured from the CAMERA, not the robot centre — 24, not 29.
    CHECK(r.range.value() == doctest::Approx(24.0).epsilon(1e-9));
}

// Would catch: the tag's out-of-plane ROTATION being lost, mirrored, or read with the wrong
// sign — i.e. the entire yaw story. A head-on-only test cannot see any of it, because a head-on
// tag has R == I and every rotation bug evaluates to the identity.
//
// THE ARITHMETIC. A 6-inch tag whose centre is 24 inches straight ahead, but turned so its
// outward normal points at 150 degrees (30 degrees off head-on). Its "right" direction in the
// body frame is (-sin150, cos150) = (-0.5, -0.8660254037844387), so with half = 3:
//     TL = centre - 3*right + 3*up = (25.5,  2.598076211353316,  3)
//     TR = centre + 3*right + 3*up = (22.5, -2.598076211353316,  3)
//     BR = (22.5, -2.598076211353316, -3)
//     BL = (25.5,  2.598076211353316, -3)
// Camera at the origin looking forward: Z = bodyX, X = -bodyY, Y = -height. So
//     u_TL = 320 - 600*2.598076211353316/25.5 = 320 - 61.13120497301919 = 258.8687950269808
//     v_TL = 240 - 600*3/25.5              = 240 - 70.58823529411765  = 169.41176470588235
//     u_TR = 320 + 600*2.598076211353316/22.5 = 320 + 69.28203230275509 = 389.2820323027551
//     v_TR = 240 - 600*3/22.5              = 240 - 80                 = 160
//     u_BR = 389.2820323027551,  v_BR = 240 + 80                = 320
//     u_BL = 258.8687950269808,  v_BL = 240 + 70.58823529411765 = 310.58823529411765
TEST_CASE("tagPnp: hand-computed — a tag turned 30 degrees off head-on recovers its heading") {
    TagCorners c;
    c.u[0] = 258.8687950269808;  c.v[0] = 169.41176470588235;
    c.u[1] = 389.2820323027551;  c.v[1] = 160.0;
    c.u[2] = 389.2820323027551;  c.v[2] = 320.0;
    c.u[3] = 258.8687950269808;  c.v[3] = 310.58823529411765;

    const TagPnpResult r = tagCornersToRobotPose(c, kCam, Length{6.0}, CameraMount{});
    REQUIRE(r.valid);
    CHECK(r.poseInRobot.x().value() == doctest::Approx(24.0).epsilon(1e-7));
    CHECK(r.poseInRobot.y().value() == doctest::Approx(0.0).epsilon(1e-7).scale(1.0));
    CHECK(r.poseInRobot.heading().degrees() == doctest::Approx(150.0).epsilon(1e-6));
    CHECK(r.reprojectionError == doctest::Approx(0.0).scale(1.0));
}

// Would catch: the tag's HEIGHT above the camera leaking into the planar position — the whole
// point of calling this a "planar reduction". A tag mounted 15 inches above the camera is still
// 24 inches away HORIZONTALLY; if the height leaked in, x would read ~28.3 (the 3D distance).
//
// Same tag, 24 inches ahead, raised 15 inches. Corners at camera (X, Y, Z) = (-+3, -(15 +- 3), 24):
//     u = 320 -+ 75 (unchanged);  v = 600 * -(15 +- 3)/24 + 240 = 240 - 25*(15 +- 3)
//                                   -> v_top = 240 - 450 = -210,  v_bottom = 240 - 300 = -60
TEST_CASE("tagPnp: hand-computed — a tag above the camera keeps its horizontal position") {
    TagCorners c;
    c.u[0] = 245.0; c.v[0] = -210.0;
    c.u[1] = 395.0; c.v[1] = -210.0;
    c.u[2] = 395.0; c.v[2] = -60.0;
    c.u[3] = 245.0; c.v[3] = -60.0;

    const TagPnpResult r = tagCornersToRobotPose(c, kCam, Length{6.0}, CameraMount{});
    REQUIRE(r.valid);
    CHECK(r.poseInRobot.x().value() == doctest::Approx(24.0).epsilon(1e-9));
    CHECK(r.poseInRobot.y().value() == doctest::Approx(0.0).epsilon(1e-9).scale(1.0));
    CHECK(r.poseInRobot.heading().degrees() == doctest::Approx(180.0).epsilon(1e-7));
    CHECK(r.range.value() == doctest::Approx(24.0).epsilon(1e-9));  // HORIZONTAL, not 28.3
}

// ─────────────────────────────────────────────────────────────────────────────────────────────
// LAYER 2 — the projector is pinned against the same hand arithmetic before it is trusted.
// ─────────────────────────────────────────────────────────────────────────────────────────────

// Would catch: a bug in the TEST's own projector, which would otherwise make every sweep case
// below a mirror of the code under test instead of a check on it. This is the case that turns
// project() from a second opinion into a verified instrument.
TEST_CASE("projector: reproduces the hand-computed anchor pixels exactly") {
    Scene s;
    s.tagX = 24.0;
    s.tagFacing = kPi;  // facing back at a robot at the origin looking down +X
    const TagCorners c = project(s, kCam);
    CHECK(c.u[0] == doctest::Approx(245.0));
    CHECK(c.v[0] == doctest::Approx(165.0));
    CHECK(c.u[1] == doctest::Approx(395.0));
    CHECK(c.v[1] == doctest::Approx(165.0));
    CHECK(c.u[2] == doctest::Approx(395.0));
    CHECK(c.v[2] == doctest::Approx(315.0));
    CHECK(c.u[3] == doctest::Approx(245.0));
    CHECK(c.v[3] == doctest::Approx(315.0));
}

// Would catch: a sign error in the projector's own camera-yaw step (Step C), which would flip
// every yawed sweep case below and could cancel against a matching error in the PnP.
// Reasoned in words: the camera is turned 30 degrees LEFT, so a tag straight ahead of the ROBOT
// falls to the camera's RIGHT — u must be greater than cx.  Arithmetic: the tag centre is at
// camera (X, Z) = (24*sin30, 24*cos30) = (12, 20.784609690826528), so
//     u_centre = 600*12/20.784609690826528 + 320 = 346.41016151377545 + 320 = 666.4101615137755.
TEST_CASE("projector: a left-yawed camera pushes a straight-ahead tag to the image right") {
    Scene s;
    s.tagX = 24.0;
    s.tagFacing = kPi;
    s.mountYaw = 30.0 * kDeg;
    s.tagSize = 0.001;  // effectively a point, so the corner mean IS the centre
    const TagCorners c = project(s, kCam);
    const double uMean = 0.25 * (c.u[0] + c.u[1] + c.u[2] + c.u[3]);
    CHECK(uMean > kCam.cx);
    CHECK(uMean == doctest::Approx(666.4101615137755).epsilon(1e-6));
}

// ─────────────────────────────────────────────────────────────────────────────────────────────
// LAYER 3 — the sweep, now that the instrument is trusted.
// ─────────────────────────────────────────────────────────────────────────────────────────────

// Would catch: a frame error that hides at heading 0 and at the origin — the exact blind spot
// the brief names. NEITHER the robot heading NOR the robot position is zero in any case here,
// and the camera is mounted off-centre and yawed. A cross-term sign error has nowhere to hide.
TEST_CASE("tagPnp: recovers the relative pose across headings, positions, mounts and ranges") {
    const double headings[] = {-155.0, -70.0, -23.0, 17.0, 44.0, 118.0, 173.0};
    const double facings[] = {-120.0, -35.0, 40.0, 95.0, 160.0};
    int checked = 0;
    for (const double hd : headings) {
        for (const double fc : facings) {
            Scene s;
            s.robotX = -19.5;
            s.robotY = 31.25;
            s.robotHeading = hd * kDeg;
            s.mountX = 4.5;
            s.mountY = -2.75;
            s.mountYaw = 12.0 * kDeg;
            s.tagFacing = fc * kDeg;
            s.tagHeight = 9.0;
            s.tagSize = 4.0;
            // Put the tag 30 inches along the CAMERA's optical axis, then a bit off to one side,
            // so every case is genuinely off-boresight. (Field position computed from the robot
            // pose + mount; the PnP never sees any of this.)
            const double axis = s.robotHeading + s.mountYaw;
            const double camFieldX =
                s.robotX + s.mountX * std::cos(s.robotHeading) - s.mountY * std::sin(s.robotHeading);
            const double camFieldY =
                s.robotY + s.mountX * std::sin(s.robotHeading) + s.mountY * std::cos(s.robotHeading);
            s.tagX = camFieldX + 30.0 * std::cos(axis) - 5.0 * std::sin(axis);
            s.tagY = camFieldY + 30.0 * std::sin(axis) + 5.0 * std::cos(axis);

            const TagCorners c = project(s, kCam);
            CameraMount mount;
            mount.x = Length{s.mountX};
            mount.y = Length{s.mountY};
            mount.yaw = Angle::radians(s.mountYaw);
            const TagPnpResult r = tagCornersToRobotPose(c, kCam, Length{s.tagSize}, mount);
            REQUIRE(r.valid);

            const RelPose want = expectedRelative(s);
            CHECK(r.poseInRobot.x().value() == doctest::Approx(want.x).epsilon(1e-6));
            CHECK(r.poseInRobot.y().value() == doctest::Approx(want.y).epsilon(1e-6));
            CHECK(std::abs(r.poseInRobot.heading().errorTo(Angle::radians(want.heading))) < 1e-6);
            CHECK(r.reprojectionError < 1e-6);
            ++checked;
        }
    }
    CHECK(checked == 35);  // the sweep really ran (a broken loop bound would read 0)
}

// Would catch: a range-dependent scale error — the classic "tag size and focal length divided
// instead of multiplied" bug, which is exactly right at one distance and wrong at every other.
TEST_CASE("tagPnp: range scales correctly from 12 to 120 inches") {
    for (const double dist : {12.0, 24.0, 36.0, 60.0, 90.0, 120.0}) {
        Scene s;
        s.robotX = 7.0;
        s.robotY = -13.0;
        s.robotHeading = 37.0 * kDeg;
        s.tagFacing = 37.0 * kDeg + kPi;  // facing straight back at the robot
        s.tagX = s.robotX + dist * std::cos(s.robotHeading);
        s.tagY = s.robotY + dist * std::sin(s.robotHeading);
        s.tagSize = 6.0;
        const TagCorners c = project(s, kCam);
        const TagPnpResult r = tagCornersToRobotPose(c, kCam, Length{6.0}, CameraMount{});
        REQUIRE(r.valid);
        CHECK(r.range.value() == doctest::Approx(dist).epsilon(1e-6));
        CHECK(r.poseInRobot.x().value() == doctest::Approx(dist).epsilon(1e-6));
        CHECK(r.poseInRobot.y().value() == doctest::Approx(0.0).epsilon(1e-6).scale(1.0));
    }
}

// Would catch: a tag size that is ignored (the pose would come out at a fixed distance
// regardless) — a physical-constant bug that a single-size test cannot see. Doubling the tag's
// physical size while the IMAGE stays identical must double the recovered distance.
TEST_CASE("tagPnp: doubling the physical tag size doubles the recovered range") {
    TagCorners c;
    c.u[0] = 245.0; c.v[0] = 165.0;
    c.u[1] = 395.0; c.v[1] = 165.0;
    c.u[2] = 395.0; c.v[2] = 315.0;
    c.u[3] = 245.0; c.v[3] = 315.0;
    const TagPnpResult small = tagCornersToRobotPose(c, kCam, Length{6.0}, CameraMount{});
    const TagPnpResult big = tagCornersToRobotPose(c, kCam, Length{12.0}, CameraMount{});
    REQUIRE(small.valid);
    REQUIRE(big.valid);
    CHECK(small.range.value() == doctest::Approx(24.0).epsilon(1e-9));
    CHECK(big.range.value() == doctest::Approx(48.0).epsilon(1e-9));
}

// Would catch: R2 being told the wrong thing about the corner-order contract. This test was
// written expecting a rotation of the order to corrupt the pose. It does not — and the finding
// is worth more than the assumption was. All three behaviours are pinned here, because R2 will
// read this file to decide how carefully to wire the detector's corners.
//
//  * ROTATION is harmless: it rotates the recovered tag frame about its own face normal, and the
//    planar reduction discards exactly that degree of freedom.
//  * REVERSAL is catastrophic AND SILENT: the tag plane is mirrored, so the recovered face
//    normal points 180 degrees the wrong way — while the reprojection error stays at machine
//    zero, because a mirrored pose reprojects onto the very same four pixels. No self-check can
//    catch this; only a physical tag can.
//  * AN ADJACENT SWAP self-intersects and is rejected, so that one at least fails loudly.
TEST_CASE("tagPnp: corner order — rotation is harmless, REVERSAL is silently catastrophic") {
    TagCorners c;
    c.u[0] = 258.8687950269808;  c.v[0] = 169.41176470588235;
    c.u[1] = 389.2820323027551;  c.v[1] = 160.0;
    c.u[2] = 389.2820323027551;  c.v[2] = 320.0;
    c.u[3] = 258.8687950269808;  c.v[3] = 310.58823529411765;
    const TagPnpResult correct = tagCornersToRobotPose(c, kCam, Length{6.0}, CameraMount{});
    REQUIRE(correct.valid);
    REQUIRE(correct.poseInRobot.heading().degrees() == doctest::Approx(150.0).epsilon(1e-6));

    SUBCASE("every cyclic rotation gives the identical planar pose") {
        for (std::size_t shift = 1; shift < 4; ++shift) {
            TagCorners rotated;
            for (std::size_t k = 0; k < 4; ++k) {
                rotated.u[k] = c.u[(k + shift) % 4];
                rotated.v[k] = c.v[(k + shift) % 4];
            }
            const TagPnpResult r = tagCornersToRobotPose(rotated, kCam, Length{6.0}, CameraMount{});
            REQUIRE(r.valid);
            CHECK(r.poseInRobot.x().value() == doctest::Approx(24.0).epsilon(1e-7));
            CHECK(std::abs(r.poseInRobot.heading().errorTo(correct.poseInRobot.heading())) < 1e-6);
        }
    }
    SUBCASE("a reversed winding flips the heading 180 degrees and hides it perfectly") {
        const std::size_t reversed[4] = {0, 3, 2, 1};
        TagCorners r;
        for (std::size_t k = 0; k < 4; ++k) {
            r.u[k] = c.u[reversed[k]];
            r.v[k] = c.v[reversed[k]];
        }
        const TagPnpResult got = tagCornersToRobotPose(r, kCam, Length{6.0}, CameraMount{});
        REQUIRE(got.valid);
        CHECK(got.poseInRobot.x().value() == doctest::Approx(24.0).epsilon(1e-7));  // position fine
        CHECK(got.poseInRobot.heading().degrees() == doctest::Approx(-30.0).epsilon(1e-6));
        // exactly 180 degrees out from the truth...
        CHECK(std::abs(std::abs(got.poseInRobot.heading().errorTo(correct.poseInRobot.heading())) -
                       kPi) < 1e-6);
        // ...and invisible to the only self-check PnP has. THIS is the line R2 must read.
        CHECK(got.reprojectionError < 1e-9);
    }
    SUBCASE("swapping two adjacent corners self-intersects and is rejected") {
        const std::size_t swapped[4] = {0, 2, 1, 3};
        TagCorners r;
        for (std::size_t k = 0; k < 4; ++k) {
            r.u[k] = c.u[swapped[k]];
            r.v[k] = c.v[swapped[k]];
        }
        CHECK_FALSE(tagCornersToRobotPose(r, kCam, Length{6.0}, CameraMount{}).valid);
    }
}

// Would catch: a degenerate input silently producing a plausible-looking pose. A PnP that
// returns a pose for four collinear points is worse than one that returns nothing, because the
// corrector downstream has no way to tell.
TEST_CASE("tagPnp: degenerate and impossible inputs return {valid=false}, never a guess") {
    TagCorners good;
    good.u[0] = 245.0; good.v[0] = 165.0;
    good.u[1] = 395.0; good.v[1] = 165.0;
    good.u[2] = 395.0; good.v[2] = 315.0;
    good.u[3] = 245.0; good.v[3] = 315.0;

    SUBCASE("four collinear corners") {
        TagCorners c;
        for (std::size_t k = 0; k < 4; ++k) {
            c.u[k] = 100.0 + 20.0 * static_cast<double>(k);
            c.v[k] = 200.0;
        }
        CHECK_FALSE(tagCornersToRobotPose(c, kCam, Length{6.0}, CameraMount{}).valid);
    }
    SUBCASE("all four corners coincident") {
        TagCorners c;
        for (std::size_t k = 0; k < 4; ++k) {
            c.u[k] = 320.0;
            c.v[k] = 240.0;
        }
        CHECK_FALSE(tagCornersToRobotPose(c, kCam, Length{6.0}, CameraMount{}).valid);
    }
    SUBCASE("a non-finite pixel (the sensor contract's backstop)") {
        TagCorners c = good;
        c.u[2] = std::numeric_limits<double>::quiet_NaN();
        CHECK_FALSE(tagCornersToRobotPose(c, kCam, Length{6.0}, CameraMount{}).valid);
    }
    SUBCASE("zero or negative tag size") {
        CHECK_FALSE(tagCornersToRobotPose(good, kCam, Length{0.0}, CameraMount{}).valid);
        CHECK_FALSE(tagCornersToRobotPose(good, kCam, Length{-6.0}, CameraMount{}).valid);
    }
    SUBCASE("zero focal length") {
        const CameraIntrinsics broken{0.0, 0.0, 320.0, 240.0};
        CHECK_FALSE(tagCornersToRobotPose(good, broken, Length{6.0}, CameraMount{}).valid);
    }
    // Would catch: the SINGULARITY GUARD inside the linear solve being removed. Found by the E3
    // mutation campaign, and it took real searching to reach — for exactly collinear corners the
    // solve divides by zero, produces infinities and is caught by the finiteness screens further
    // down, so the guard looks redundant. It is not. Four corners that are collinear to within
    // 1e-13 of a pixel leave the pivot small but FINITE, the solve blows up to enormous finite
    // numbers, and every downstream screen then passes:
    //
    //     with the guard:     valid = false
    //     without the guard:  valid = TRUE, "a tag 5.99 inches away at x = 5.73, facing us"
    //
    // A fabricated but entirely plausible pose is far worse than no pose, because nothing
    // downstream — not the corrector's range band, not its gate, not the reprojection error —
    // has any way to tell it is fiction.
    SUBCASE("corners collinear to within 1e-13 px — plausible fiction, refused") {
        TagCorners c;
        for (std::size_t k = 0; k < 4; ++k) {
            c.u[k] = 100.0 + 20.0 * static_cast<double>(k);
            c.v[k] = 200.0;
        }
        c.v[1] += 1e-13;
        c.v[3] -= 1e-13;
        CHECK_FALSE(tagCornersToRobotPose(c, kCam, Length{6.0}, CameraMount{}).valid);
    }
}

// Would catch: pixel noise producing a wildly wrong pose with a silently ZERO reprojection
// error — i.e. a reprojection number that is decorative rather than computed. With noisy
// corners the recovered pose must stay sane AND the reported error must become non-zero.
TEST_CASE("tagPnp: reprojection error is real — it rises when the corners are perturbed") {
    Scene s;
    s.robotX = 5.0;
    s.robotY = 8.0;
    s.robotHeading = 25.0 * kDeg;
    s.tagX = 45.0;
    s.tagY = 20.0;
    s.tagFacing = -160.0 * kDeg;
    s.tagSize = 6.0;
    TagCorners c = project(s, kCam);
    const TagPnpResult clean = tagCornersToRobotPose(c, kCam, Length{6.0}, CameraMount{});
    REQUIRE(clean.valid);
    CHECK(clean.reprojectionError < 1e-9);

    c.u[0] += 1.5;
    c.v[2] -= 1.5;
    const TagPnpResult noisy = tagCornersToRobotPose(c, kCam, Length{6.0}, CameraMount{});
    REQUIRE(noisy.valid);
    CHECK(noisy.reprojectionError > 1e-3);  // it is computed, not decorative
    // ... and the pose degrades gracefully rather than exploding.
    CHECK(std::abs(noisy.poseInRobot.x().value() - clean.poseInRobot.x().value()) < 3.0);
    CHECK(std::abs(noisy.poseInRobot.y().value() - clean.poseInRobot.y().value()) < 3.0);
}

// Would catch: the recovered scale being taken from ONE of the tag's two in-plane axes instead
// of BOTH — a mutation that is completely invisible on noise-free input, because a perfect
// square projects to columns of exactly equal norm and `2/(n1 + n2)` and `1/n1` then agree to
// the bit. Found by the E3 mutation campaign; this is the test that closes it.
//
// THE PROBE, and it is exact. Take the head-on anchor image (a 150 x 150 px square, 6-inch tag,
// 24 inches away) and STRETCH IT HORIZONTALLY about the frame centre by a factor f — which is
// what an anisotropic mis-detection, or a mis-declared fx, looks like. The tag's X axis now
// implies a depth of 24/f while its Y axis still implies 24. Averaging the two column norms
// returns their HARMONIC mean; taking |r1| alone returns 24/f.
//
//     f = 1.2 :  axes imply 20 and 24  ->  harmonic mean 2/(1/20 + 1/24) = 21.81818...  (vs 20)
//     f = 2.0 :  axes imply 12 and 24  ->  harmonic mean 2/(1/12 + 1/24) = 16          (vs 12)
TEST_CASE("tagPnp: the scale averages BOTH in-plane axes (exact harmonic mean under a stretch)") {
    const double us[4] = {245.0, 395.0, 395.0, 245.0};
    const double vs[4] = {165.0, 165.0, 315.0, 315.0};
    const auto stretched = [&](double f) {
        TagCorners c;
        for (std::size_t k = 0; k < 4; ++k) {
            c.u[k] = 320.0 + (us[k] - 320.0) * f;
            c.v[k] = vs[k];
        }
        return tagCornersToRobotPose(c, kCam, Length{6.0}, CameraMount{});
    };

    const TagPnpResult unstretched = stretched(1.0);
    REQUIRE(unstretched.valid);
    CHECK(unstretched.range.value() == doctest::Approx(24.0).epsilon(1e-12));  // the anchor

    const TagPnpResult modest = stretched(1.2);
    REQUIRE(modest.valid);
    CHECK(modest.range.value() == doctest::Approx(21.818181818181817).epsilon(1e-9));

    const TagPnpResult severe = stretched(2.0);
    REQUIRE(severe.valid);
    CHECK(severe.range.value() == doctest::Approx(16.0).epsilon(1e-9));
    // Stated as the property rather than the number: the answer lies strictly BETWEEN what the
    // two axes each imply, which is only possible if both were used.
    CHECK(severe.range.value() > 12.0);
    CHECK(severe.range.value() < 24.0);
}

// Would catch: the reprojection error not rising with a distortion the pose cannot explain. The
// stretched quad above is not the image of any rigid square, so a correct solver must report
// residual — it is the only self-check R2 will have on a real camera.
TEST_CASE("tagPnp: a stretch the pose cannot explain shows up as reprojection error") {
    TagCorners c;
    const double us[4] = {245.0, 395.0, 395.0, 245.0};
    const double vs[4] = {165.0, 165.0, 315.0, 315.0};
    for (std::size_t k = 0; k < 4; ++k) {
        c.u[k] = 320.0 + (us[k] - 320.0) * 2.0;
        c.v[k] = vs[k];
    }
    const TagPnpResult r = tagCornersToRobotPose(c, kCam, Length{6.0}, CameraMount{});
    REQUIRE(r.valid);
    CHECK(r.reprojectionError > 5.0);  // several pixels: visibly not a rigid square
}
