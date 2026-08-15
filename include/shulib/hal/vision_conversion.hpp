#pragma once
//
// vision_conversion.hpp — the ONE place raw AprilTag image corners become shulib's canonical
// robot-relative tag pose (§7: convert exactly once, at the edge). Companion to
// gps_conversion.hpp and imu_conversion.hpp, and named to match them on purpose: everything in
// this file is a PURE FUNCTION over plain numbers, so it is host-testable with no sensor, no
// clock and no PROS.
//
// hal/vision.hpp scopes this precisely (lines 10-14): ITagSource yields each tag as a RELATIVE
// POSE in the robot BODY frame, "a planar reduction of the tag's 6-DOF pose suitable for
// ground-plane localization", and "the corners->pose PnP (needs camera intrinsics + tag size) is
// a pure, host-testable function built with the M3 AprilTagCorrector".
//
// ── WHY THIS IS A FREE FUNCTION AND NOT A METHOD ON THE CORRECTOR (chunk tension T3) ─────────
// The CALLER of this function is the R2 adapter, not the corrector. The seam already hands
// AprilTagCorrector a reduced `poseInRobot`; the corrector never sees a pixel. If the PnP lived
// inside the corrector, R2 would have to either duplicate it or reach into a localization class
// from a HAL adapter, and both are worse than the seam that already exists. So it lives here,
// beside the other two edge conversions, and the corrector does not include this header at all.
//
// ── FRAMES, STATED ONCE, EXPLICITLY ─────────────────────────────────────────────────────────
// CAMERA frame (the standard pinhole convention):
//     +X_cam = image right, +Y_cam = image DOWN, +Z_cam = out along the optical axis.
//     Projection: u = fx * X/Z + cx,  v = fy * Y/Z + cy.
// TAG frame (planar, origin at the tag's centre):
//     +X_tag = the tag's own right AS SEEN BY A VIEWER FACING IT, +Y_tag = DOWN the tag face,
//     +Z_tag = X_tag x Y_tag, which therefore points AWAY from that viewer (into the wall).
//     The tag's OUTWARD normal — the direction it "faces" — is -Z_tag.
//     A tag seen head-on has R == I, so a head-on tag's outward normal is -Z_cam: it looks
//     back down the optical axis. That identity is the sign convention's anchor.
// ROBOT BODY frame (F1, locked): +X forward, +Y LEFT, heading CCW-positive.
//
// CORNER ORDER — the input contract, and the one thing R2 must get right:
//     corners[0] = tag-frame (-s/2, -s/2)   (the viewer's TOP-LEFT)
//     corners[1] = tag-frame (+s/2, -s/2)   (TOP-RIGHT)
//     corners[2] = tag-frame (+s/2, +s/2)   (BOTTOM-RIGHT)
//     corners[3] = tag-frame (-s/2, +s/2)   (BOTTOM-LEFT)
// i.e. clockwise in the image when the tag is upright and facing the camera. Which order the V5
// AI Vision sensor / a Pi detector actually reports is UNVERIFIED (A4 register HA-69) — mapping
// the detector's order onto this one is R2's job and R2's test.
//
// HOW MUCH THE ORDER ACTUALLY MATTERS — measured, not assumed (E3; pinned by
// test/vision_conversion_test.cpp):
//   * A CYCLIC ROTATION of the order (0123 -> 1230, ...) changes NOTHING. It rotates the
//     recovered tag frame about its own face normal, and the planar reduction discards exactly
//     that degree of freedom. R2 does not have to find the detector's starting corner.
//   * A REVERSAL of the order (0123 -> 0321) is CATASTROPHIC AND SILENT. It mirrors the tag
//     plane, so the recovered face normal points 180 degrees the wrong way — while the
//     reprojection error stays at ~1e-14, because a mirrored pose reprojects onto the same four
//     pixels. Nothing in this function, and nothing downstream of it, can detect a reversed
//     winding. R2 must pin the detector's WINDING against a physical tag; a self-check cannot.
//   * A SWAP of two ADJACENT corners produces a self-intersecting quad and returns
//     {valid = false}, so that miswiring at least fails loudly.
//
// ── THE PLANAR REDUCTION, AND WHAT IT THROWS AWAY ───────────────────────────────────────────
// The 6-DOF solution is computed in full and then reduced to (x, y, heading) in the body frame:
//   * the tag's HEIGHT above the camera (t_y) is discarded — correct for a ground-plane
//     localizer, and the reason a tag mounted above the camera still yields the right x/y;
//   * the tag's PITCH and ROLL are discarded — its outward normal is projected onto the
//     horizontal plane and the heading is the direction of that projection.
// This assumes the camera is mounted LEVEL (no pitch/roll). A pitched camera turns a height
// difference into a range error, and nothing downstream can see that it happened
// (A4 register HA-70).
//
// ── THE ALGORITHM, AND ITS ONE STRUCTURAL ASSUMPTION ────────────────────────────────────────
// Four coplanar correspondences determine a homography exactly. With tag-plane points
// X = (x, y, 0):
//     [u v 1]^T ~ K * (x*r1 + y*r2 + t) = K * [r1 r2 t] * [x y 1]^T   =>   H = lambda*K*[r1 r2 t]
// so the pipeline is: DLT for H (8x8 solve, partial pivoting) -> G = K^-1 * H -> recover the
// scale from |r1| = |r2| = 1 -> orthonormalize (r1, r2) -> r3 = r1 x r2 -> reduce.
//
// H is normalized with h33 == 1, which fixes lambda = 1/t_z. That is legitimate here and not in
// general: h33 is the projective scale of the tag's CENTRE, which is zero only for a tag at
// infinity or exactly in the camera's plane — neither is a visible tag. The benefit is that the
// depth sign comes out right by construction (t_z = s > 0), so there is no "which of the two
// solutions is in front of the camera" branch to get wrong.
//
// PLANAR PnP HAS A KNOWN NEAR-AMBIGUITY: as a tag shrinks toward the image centre or turns
// nearly face-on, two poses with mirrored out-of-plane rotation project almost identically, so
// the recovered HEADING degrades much faster with range than the recovered POSITION does. This
// function does not pretend otherwise — it reports the reprojection error it achieved, and the
// corrector's trusted-range band (A4 register HA-73) is what keeps the caller out of the region
// where the ambiguity bites. A second (mirror) solution and a proper R_heading belong to E4.
//
// Nothing here allocates, throws, or reads a clock.

#include <cmath>
#include <cstddef>

#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal {

/// Pinhole intrinsics, in pixels. Focal lengths must be non-zero; no distortion model — the
/// adapter is expected to hand over UNDISTORTED corners (R2 owns that, and owns proving it).
struct CameraIntrinsics {
    /// Horizontal focal length in PIXELS (the projection is u = fx * X/Z + cx). The 0.0
    /// default is deliberately unusable: tagCornersToRobotPose rejects |fx| < 1e-9, so an
    /// intrinsics block nobody filled in fails closed rather than returning a plausible pose.
    double fx = 0.0;
    /// Vertical focal length in PIXELS (v = fy * Y/Z + cy). Rejected at 0 exactly like fx.
    /// Equal to fx only for square pixels, which is why the two are carried separately.
    double fy = 0.0;
    double cx = 0.0;  ///< principal point, PIXELS right from the image origin
    double cy = 0.0;  ///< principal point, PIXELS DOWN from the image origin (+v is down)
};

/// Where the camera sits on the robot, in the canonical body frame (F1: +X forward, +Y left).
/// `yaw` is the direction the OPTICAL AXIS points, CCW-positive from +X. The camera is assumed
/// level (A4 register HA-70). One owner for this offset, exactly as gps_conversion.hpp insists for the GPS
/// lever arm: applying it twice is inches of silent bias.
struct CameraMount {
    units::Length x{};  ///< camera position FORWARD of the robot origin (body +X)
    units::Length y{};  ///< camera position LEFT of the robot origin (body +Y is LEFT, F1)
    /// Direction the OPTICAL AXIS points, CCW-positive from body +X; 0 means the camera looks
    /// straight forward. This one rotation is ALL that is modelled — the camera is assumed
    /// level, so there is no mount pitch or roll, and a pitched camera silently becomes a
    /// range error that nothing downstream can detect.
    math::Angle yaw{};
};

/// Four image corners in pixels, in the order documented at the top of this file.
struct TagCorners {
    /// Pixel column (image RIGHT) of each corner. u[k] and v[k] are the SAME corner — these
    /// are parallel arrays, not two independent lists. The index order is the one documented
    /// at the top of this file, and the pixels must already be UNDISTORTED: a cyclic rotation
    /// of the order changes nothing, but a REVERSED winding is silently catastrophic.
    double u[4] = {0.0, 0.0, 0.0, 0.0};
    /// Pixel row of each corner, measured DOWN from the image origin (+v is down, the camera
    /// convention at the top of this file). Paired with u by index.
    double v[4] = {0.0, 0.0, 0.0, 0.0};
};

/// The planar reduction, plus the numbers a caller needs to decide whether to believe it.
struct TagPnpResult {
    bool valid = false;             ///< false => the geometry was degenerate; poseInRobot is unset
    math::Pose2d poseInRobot{};     ///< tag pose relative to the robot (canonical body frame)
    units::Length range{};          ///< HORIZONTAL distance from the CAMERA to the tag centre
    double reprojectionError = 0.0; ///< RMS pixel error of the recovered pose (0 for exact input)
};

namespace detail {

/// Solve A x = b for n <= 8 by Gaussian elimination with partial pivoting. Returns false if the
/// system is singular to working precision. Fixed storage: no allocation.
[[nodiscard]] inline bool solveLinear(double a[8][9], std::size_t n, double out[8]) noexcept {
    for (std::size_t col = 0; col < n; ++col) {
        std::size_t pivot = col;
        double best = std::abs(a[col][col]);
        for (std::size_t row = col + 1; row < n; ++row) {
            const double mag = std::abs(a[row][col]);
            if (mag > best) {
                best = mag;
                pivot = row;
            }
        }
        if (!(best > 1e-12)) {
            return false;  // singular: four collinear corners, or a degenerate view
        }
        if (pivot != col) {
            for (std::size_t k = col; k <= n; ++k) {
                const double tmp = a[col][k];
                a[col][k] = a[pivot][k];
                a[pivot][k] = tmp;
            }
        }
        for (std::size_t row = col + 1; row < n; ++row) {
            const double factor = a[row][col] / a[col][col];
            if (factor == 0.0) {
                continue;
            }
            for (std::size_t k = col; k <= n; ++k) {
                a[row][k] -= factor * a[col][k];
            }
        }
    }
    for (std::size_t i = n; i-- > 0;) {
        double sum = a[i][n];
        for (std::size_t k = i + 1; k < n; ++k) {
            sum -= a[i][k] * out[k];
        }
        out[i] = sum / a[i][i];
    }
    return true;
}

}  // namespace detail

/// Corners -> the tag's pose relative to the robot. THE function hal/vision.hpp:12 reserves.
///
/// Returns `{valid = false}` rather than throwing on: a non-finite input, a non-positive tag
/// size or focal length, a degenerate corner set, a tag behind the camera, or a recovered normal
/// with no horizontal component (a tag lying flat, which a ground-plane reduction cannot use).
/// Never throwing matters because R2's adapter runs this on sensor data, and sensor data is
/// exactly where the impossible input comes from.
[[nodiscard]] inline TagPnpResult tagCornersToRobotPose(const TagCorners& corners,
                                                        const CameraIntrinsics& intrinsics,
                                                        units::Length tagSize,
                                                        const CameraMount& mount) noexcept {
    TagPnpResult result;

    const double size = tagSize.value();
    const double fx = intrinsics.fx;
    const double fy = intrinsics.fy;
    const double ccx = intrinsics.cx;
    const double ccy = intrinsics.cy;
    if (!(size > 0.0) || !std::isfinite(size) || !std::isfinite(fx) || !std::isfinite(fy) ||
        !std::isfinite(ccx) || !std::isfinite(ccy) || std::abs(fx) < 1e-9 || std::abs(fy) < 1e-9) {
        return result;
    }
    for (std::size_t k = 0; k < 4; ++k) {
        if (!std::isfinite(corners.u[k]) || !std::isfinite(corners.v[k])) {
            return result;
        }
    }

    // Tag-plane corner coordinates, in the documented order.
    const double half = 0.5 * size;
    const double px[4] = {-half, half, half, -half};
    const double py[4] = {-half, -half, half, half};

    // ── DLT: 8 equations for h11..h32, with h33 fixed to 1 (header note). ────────────────────
    double sys[8][9] = {};
    for (std::size_t k = 0; k < 4; ++k) {
        const double x = px[k];
        const double y = py[k];
        const double u = corners.u[k];
        const double v = corners.v[k];
        double* rowU = sys[2 * k];
        rowU[0] = x;
        rowU[1] = y;
        rowU[2] = 1.0;
        rowU[6] = -u * x;
        rowU[7] = -u * y;
        rowU[8] = u;
        double* rowV = sys[2 * k + 1];
        rowV[3] = x;
        rowV[4] = y;
        rowV[5] = 1.0;
        rowV[6] = -v * x;
        rowV[7] = -v * y;
        rowV[8] = v;
    }
    double h[8] = {};
    if (!detail::solveLinear(sys, 8, h)) {
        return result;
    }
    for (std::size_t k = 0; k < 8; ++k) {
        if (!std::isfinite(h[k])) {
            return result;
        }
    }

    // ── G = K^-1 * H. K^-1 = [[1/fx, 0, -cx/fx], [0, 1/fy, -cy/fy], [0, 0, 1]]. ─────────────
    // Columns g1, g2, g3 of G; H's third row is (h[6], h[7], 1).
    const double hRow3[3] = {h[6], h[7], 1.0};
    double g[3][3];
    for (std::size_t col = 0; col < 3; ++col) {
        const double h1c = h[col];
        const double h2c = h[3 + col];
        const double h3c = hRow3[col];
        g[0][col] = (h1c - ccx * h3c) / fx;
        g[1][col] = (h2c - ccy * h3c) / fy;
        g[2][col] = h3c;
    }

    // ── Scale: |r1| = |r2| = 1. g3_z == 1 by the h33 normalization, so t_z = s > 0 and the
    // "tag in front of the camera" branch cannot be taken wrongly (header note). ─────────────
    const double n1 = std::sqrt(g[0][0] * g[0][0] + g[1][0] * g[1][0] + g[2][0] * g[2][0]);
    const double n2 = std::sqrt(g[0][1] * g[0][1] + g[1][1] * g[1][1] + g[2][1] * g[2][1]);
    if (!(n1 > 1e-12) || !(n2 > 1e-12)) {
        return result;
    }
    const double scale = 2.0 / (n1 + n2);

    double r1[3] = {scale * g[0][0], scale * g[1][0], scale * g[2][0]};
    double r2[3] = {scale * g[0][1], scale * g[1][1], scale * g[2][1]};
    const double t[3] = {scale * g[0][2], scale * g[1][2], scale * g[2][2]};
    if (!(t[2] > 0.0) || !std::isfinite(t[0]) || !std::isfinite(t[1]) || !std::isfinite(t[2])) {
        return result;  // tag behind the camera, or the solve blew up
    }

    // ── Orthonormalize (r1, r2) to the nearest orthonormal pair spanning the same plane, then
    // r3 = r1 x r2. Noise makes the raw columns neither unit nor perpendicular. ──────────────
    double sum[3] = {r1[0] + r2[0], r1[1] + r2[1], r1[2] + r2[2]};
    double dif[3] = {r1[0] - r2[0], r1[1] - r2[1], r1[2] - r2[2]};
    const double ns = std::sqrt(sum[0] * sum[0] + sum[1] * sum[1] + sum[2] * sum[2]);
    const double nd = std::sqrt(dif[0] * dif[0] + dif[1] * dif[1] + dif[2] * dif[2]);
    if (!(ns > 1e-12) || !(nd > 1e-12)) {
        return result;
    }
    const double kInvSqrt2 = 0.70710678118654752440;
    for (std::size_t k = 0; k < 3; ++k) {
        const double a = sum[k] / ns;
        const double b = dif[k] / nd;
        r1[k] = kInvSqrt2 * (a + b);
        r2[k] = kInvSqrt2 * (a - b);
    }
    const double r3[3] = {r1[1] * r2[2] - r1[2] * r2[1], r1[2] * r2[0] - r1[0] * r2[2],
                          r1[0] * r2[1] - r1[1] * r2[0]};

    // ── Reprojection error of the recovered pose, in pixels. Not used for gating here; handed
    // to the caller because it is the only self-check PnP can offer, and R2 will want it. ────
    double sq = 0.0;
    for (std::size_t k = 0; k < 4; ++k) {
        const double cx3 = r1[0] * px[k] + r2[0] * py[k] + t[0];
        const double cy3 = r1[1] * px[k] + r2[1] * py[k] + t[1];
        const double cz3 = r1[2] * px[k] + r2[2] * py[k] + t[2];
        if (!(cz3 > 1e-9)) {
            return result;
        }
        const double du = (fx * cx3 / cz3 + ccx) - corners.u[k];
        const double dv = (fy * cy3 / cz3 + ccy) - corners.v[k];
        sq += du * du + dv * dv;
    }
    const double rms = std::sqrt(sq / 4.0);

    // ── Camera frame -> robot body frame (the planar reduction). ─────────────────────────────
    // Optical axis  +Z_cam  ->  (cos psi,  sin psi) in the body frame.
    // Image right   +X_cam  ->  (sin psi, -cos psi)  (right is 90 deg CW of forward; +Y is LEFT).
    // +Y_cam (down) leaves the plane and is dropped — that is the reduction.
    const double psi = mount.yaw.radians();
    const double cpsi = std::cos(psi);
    const double spsi = std::sin(psi);
    const double bodyX = mount.x.value() + t[2] * cpsi + t[0] * spsi;
    const double bodyY = mount.y.value() + t[2] * spsi - t[0] * cpsi;

    // The tag's OUTWARD normal is -Z_tag = -r3 (header note). Project it onto the ground plane
    // and read its direction: that is the heading the tag "faces", in the body frame.
    const double nCamX = -r3[0];
    const double nCamZ = -r3[2];
    const double normalX = nCamZ * cpsi + nCamX * spsi;
    const double normalY = nCamZ * spsi - nCamX * cpsi;
    if (std::hypot(normalX, normalY) < 1e-9) {
        return result;  // the tag faces straight up or down: unusable for a ground-plane fix
    }
    const double heading = std::atan2(normalY, normalX);
    const double horizontalRange = std::hypot(t[0], t[2]);
    if (!std::isfinite(bodyX) || !std::isfinite(bodyY) || !std::isfinite(heading) ||
        !std::isfinite(horizontalRange) || !std::isfinite(rms)) {
        return result;
    }

    result.valid = true;
    result.poseInRobot =
        math::Pose2d{units::Length{bodyX}, units::Length{bodyY}, math::Angle::radians(heading)};
    result.range = units::Length{horizontalRange};
    result.reprojectionError = rms;
    return result;
}

}  // namespace shulib::hal
