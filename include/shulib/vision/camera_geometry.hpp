#pragma once
//
// Camera geometry — a detection's box turned into which way it is (BEARING), how high it looks
// (ELEVATION), how far it is (a GROUND range from where its base meets the floor, a SIZE range from its
// known real height) and where it is on the FIELD — every one an INTERVAL that carries the stated error
// of everything it was computed from (chunk VIS1-A1, 2026-10-22). Pure functions over plain numbers;
// PROS-free, sim-free, allocation-free.
//
// ── Frames ──────────────────────────────────────────────────────────────────────────
// Image: +u right, +v DOWN, (0,0) the top-left corner, 320 x 240 (A4 register HA-193). Body (F1): +X
// forward, +Y LEFT, +Z up from the FLOOR. Bearing: counter-clockwise from the body's +X (left
// positive), radians. Elevation: above the horizontal, radians. Field: F1's (origin at the field's
// centre).
//
// ── The error is an INTERVAL, not a sigma ───────────────────────────────────────────
// Each input carries a stated tolerance: the mount's height, pitch, yaw and position (measured by the
// team — config::CameraPlacement), the optics' two focal lengths, EACH on its own (A4 register HA-194),
// and a box edge's jitter (A4 register HA-202). Every interval is the EXACT range of its output over that
// whole tolerance box: the output is evaluated at every CORNER of the box, where it is monotone in an
// input, and also at the INTERIOR point where it is not — a pitch that makes a row's ray LEVEL (the
// bearing of a point on the lens's level; the size range of a box straddling it) and the principal
// point's column (the elevation, ground range and size range of a point whose jittered column may be on
// it). A box edge's jitter moves a point in BOTH its coordinates, and each function carries both (the
// size range through its two edges and its column). Each function's doc names its interior points, and a
// test sweeps each against the whole box at 100 % of every tolerance. That holds while the pitch with its
// tolerance stays inside +-kMaxPitchSpanDeg — every ray of every corner then points forward, and each
// input taken as monotone stays so — which cameraModelProblem() requires. A consumer that needs one value
// takes the interval and REFUSES when it is too wide. An interval is a bound on what the stated
// tolerances allow, never a probability: a mount that moved beyond its tolerance makes it wrong, which is
// why the tolerance is typed with its provenance.
//
// The stated tolerances include the CALLER's PoseBound, and that bound must hold for the pose WHEN THE
// IMAGE WAS TAKEN, not when the read completed: the image's age at read completion is UNMEASURED (A4
// register HA-192), and a robot turning at 180 deg/s carries a Pin 36 in ahead about 7 in off in 66 ms.
// boundAtImage() widens a bound by the robot's speed and turn rate times that age; a consumer that has
// no MEASURED age uses field positions only with the robot stopped.
//
// ── One camera explains it ──────────────────────────────────────────────────────────
// Where two quantities come from ONE camera (a band's two edges; a Pin's base and its height) the pitch,
// height and focal-length errors are COMMON to both. A consumer tests their agreement CORNER BY CORNER
// through eachCorner() — "is there one camera, inside the stated tolerances, that sees this box as X?" —
// never by overlapping two intervals each widened over every corner, which double-counts the common
// error (VIS1's probes: 129 of 540 lookalike bands passed with overlapped intervals, 86 per corner).
//
// ── The camera's axis, and what is not modelled ─────────────────────────────────────
// The optical axis is taken square to the housing's back face, where the team measures pitch and yaw
// (A4 register HA-194); a lens mounted off that axis is a yaw or pitch to type, not a tolerance to widen.
// The optics are TWO angles, not one: 74 x 63 degrees over 320 x 240 is not square pixels (fx = 160/tan
// 37 deg = 212.3 px, fy = 120/tan 31.5 deg = 195.8 px), so the two focal lengths are carried apart. The
// principal point is the image's centre; lens distortion is not modelled (HA-194's settle measures how
// far off-centre it bites); ROLL is not modelled (the mount is yaw and pitch only — re-mount a camera
// rolled more than about a degree).
//
// hal::CameraMount (hal/vision_conversion.hpp) encodes HA-70's LEVEL camera for the AprilTag PnP and is
// untouched: tags are not this header's.
//
// NO HARDWARE CLAIM: every function here has run against the host camera model (sim/camera_plant.hpp,
// written with its own projection) and nothing else.

#include <cmath>
#include <cstdint>
#include <limits>

#include "shulib/config/camera_config.hpp"
#include "shulib/hal/vision_frames.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::vision {

/// Degrees to radians (`math::Angle::kPi / 180`).
inline constexpr double kDegToRad = math::Angle::kPi / 180.0;

/// A closed interval [lo, hi], or invalid. Every geometric answer here is one; an invalid one means
/// nothing could be said (the producer's documentation says when).
struct Interval {
    bool valid = false;  ///< false: nothing could be said
    double lo = 0.0;     ///< the lower bound
    double hi = 0.0;     ///< the upper bound
    /// The middle of the interval.
    [[nodiscard]] constexpr double mid() const noexcept { return 0.5 * (lo + hi); }
    /// hi - lo.
    [[nodiscard]] constexpr double width() const noexcept { return hi - lo; }
    /// True when the interval is valid and holds `x` (both ends included).
    [[nodiscard]] constexpr bool contains(double x) const noexcept { return valid && x >= lo && x <= hi; }
};

/// The camera's optics — a DEVICE fact, the same for every AI Vision sensor: a library constant
/// (`kAiVisionOptics`, A4 register HA-194), never typed by a team.
struct CameraOptics {
    double hfovDeg = 0.0;       ///< the horizontal field of view across the width, degrees
    double vfovDeg = 0.0;       ///< the vertical field of view across the height, degrees
    double fovTolerance = 0.0;  ///< relative: 0.05 = each focal length known to +-5 %, each on its own
    int widthPx = hal::kVisionImageWidthPx;    ///< the report's width, pixels
    int heightPx = hal::kVisionImageHeightPx;  ///< the report's height, pixels
};

/// The AI Vision sensor's optics as VEX publishes them — 74 x 63 degrees (VEX's knowledge-base article,
/// read only through search snippets: the page answers 403) — with +-5 % on each focal length until the
/// bench measures them (A4 register HA-194).
inline constexpr CameraOptics kAiVisionOptics{74.0, 63.0, 0.05, hal::kVisionImageWidthPx,
                                              hal::kVisionImageHeightPx};

/// A box edge's jitter, pixels, for every edge-derived quantity — a model object's box AND a colour
/// band's top and bottom rows (A4 register HA-202: invented until the bench records boxes of a still
/// object). A band no taller than two of these cannot be measured.
inline constexpr double kBoxEdgePx = 2.0;

/// A robot pose with how far it may be off — the CALLER's statement (the localizer's quality, or a bound
/// a route author types; nothing here knows a robot's drift). It must bound the pose AT THE IMAGE's time,
/// not at the read's completion (A4 register HA-192): `boundAtImage()` widens a moving robot's bound.
struct PoseBound {
    math::Pose2d pose{};                ///< the field-frame pose (F1)
    units::Length positionTolerance{};  ///< +- radius on x and y, inches
    double headingToleranceDeg = 0.0;   ///< +- on the heading, degrees
};

/// A bound on the pose at the IMAGE's time from a bound at the read's completion: the position tolerance
/// grows by |speed| x age and the heading tolerance by |turn rate| x age. `maxImageAgeSeconds` is the
/// camera's image age at read completion, MEASURED on the bench (A4 register HA-192) — a consumer with no
/// measured age has no bound for a moving robot. A negative age, a non-finite speed, turn rate or age, or
/// a negative or non-finite tolerance in `atRead` is NOT a bound: both tolerances of the result are
/// infinite (every fix it touches becomes unusably wide — never a tight bound from a bad number).
[[nodiscard]] inline PoseBound boundAtImage(const PoseBound& atRead, double speedInPerS,
                                            double turnRateDegPerS, double maxImageAgeSeconds) noexcept {
    PoseBound b = atRead;
    const double position = atRead.positionTolerance.value();
    const double heading = atRead.headingToleranceDeg;
    const bool usable = maxImageAgeSeconds >= 0.0 && std::isfinite(maxImageAgeSeconds)
                        && std::isfinite(speedInPerS) && std::isfinite(turnRateDegPerS)
                        && position >= 0.0 && std::isfinite(position) && heading >= 0.0
                        && std::isfinite(heading);
    if (!usable) {
        b.positionTolerance = units::Length{std::numeric_limits<double>::infinity()};
        b.headingToleranceDeg = std::numeric_limits<double>::infinity();
        return b;
    }
    b.positionTolerance = units::Length{position + std::fabs(speedInPerS) * maxImageAgeSeconds};
    b.headingToleranceDeg = heading + std::fabs(turnRateDegPerS) * maxImageAgeSeconds;
    return b;
}

/// A point on the field with a conservative error RADIUS (a bound from the intervals, not a sigma).
struct FieldFix {
    bool valid = false;   ///< false: no position could be computed
    double x = 0.0;       ///< field x, inches (the nominal point)
    double y = 0.0;       ///< field y, inches (the nominal point)
    double radius = 0.0;  ///< every position the stated tolerances allow lies within this of (x, y)
};

/// One corner of the tolerance box: a concrete camera the model evaluates.
struct CameraCorner {
    double fx = 0.0;     ///< the horizontal focal length, pixels
    double fy = 0.0;     ///< the vertical focal length, pixels
    double cx = 0.0;     ///< the principal point's column, pixels
    double cy = 0.0;     ///< the principal point's row, pixels
    double h = 0.0;      ///< the lens's height above the floor, inches
    double pitch = 0.0;  ///< the optical axis above the horizontal, radians
    double yaw = 0.0;    ///< the optical axis counter-clockwise from the body's +X, radians
};

/// The camera model: optics and a measured placement, giving pure answers with intervals. PRECONDITION:
/// the placement has no problem (`cameraModelProblem()` returns nullptr for its config) — every consumer
/// asks that first and refuses to construct, naming the reason.
class CameraModel {
public:
    /// The model of a camera with optics `o` mounted at `p` (both copied).
    CameraModel(const CameraOptics& o, const config::CameraPlacement& p) noexcept : o_{o}, p_{p} {
        fx0_ = (o.widthPx / 2.0) / std::tan(o.hfovDeg * kDegToRad / 2.0);
        fy0_ = (o.heightPx / 2.0) / std::tan(o.vfovDeg * kDegToRad / 2.0);
    }

    /// The placement this model was built with.
    [[nodiscard]] const config::CameraPlacement& placement() const noexcept { return p_; }
    /// The nominal horizontal focal length, pixels (212.3 for kAiVisionOptics).
    [[nodiscard]] double fx() const noexcept { return fx0_; }
    /// The nominal vertical focal length, pixels (195.8 for kAiVisionOptics).
    [[nodiscard]] double fy() const noexcept { return fy0_; }
    /// The relative tolerance on each focal length.
    [[nodiscard]] double opticsTolerance() const noexcept { return o_.fovTolerance; }

    /// The NOMINAL camera (every tolerance at zero).
    [[nodiscard]] CameraCorner nominal() const noexcept {
        return {fx0_, fy0_, o_.widthPx / 2.0, o_.heightPx / 2.0, p_.height.value(),
                p_.pitchDeg * kDegToRad, p_.yawDeg * kDegToRad};
    }

    /// The bearing (body frame) and elevation, radians, of the ray through pixel (u, v) for camera
    /// corner `c`.
    static void ray(const CameraCorner& c, double u, double v, double& bearing,
                    double& elevation) noexcept {
        const double xr = (u - c.cx) / c.fx;  // right, per unit depth
        const double yd = (v - c.cy) / c.fy;  // down, per unit depth
        const double sp = std::sin(c.pitch);
        const double cp = std::cos(c.pitch);
        // The ray in a yaw-free frame (forward, left, up): the axis is (cp, 0, sp), image-right is
        // (0, -1, 0) and image-down is (sp, 0, -cp).
        const double f = cp + yd * sp;
        const double l = -xr;
        const double up = sp - yd * cp;
        bearing = std::atan2(l, f) + c.yaw;
        elevation = std::atan2(up, std::hypot(f, l));
    }

    /// The BEARING of the point (u, v) — a box's centre — over the yaw, pitch and focal-length tolerances
    /// and +-kBoxEdgePx of jitter on the centre's column AND its row, radians (the centre of two edges that
    /// each move kBoxEdgePx moves as far, when both move the same way). Its INTERIOR point (header): the
    /// ray's forward part, cos p + yd sin p, is LARGEST — the bearing nearest the axis — at the pitch that
    /// makes the row's ray level, atan(yd); when that pitch is inside the tolerance it is evaluated too, for
    /// each corner and each jittered row.
    [[nodiscard]] Interval bearing(double u, double v) const noexcept {
        Interval out{};
        double pLo = 0.0;
        double pHi = 0.0;
        pitchSpan(pLo, pHi);
        forEachCorner(/*useHeight=*/false, [&](const CameraCorner& c) {
            for (const double dv : {-kBoxEdgePx, kBoxEdgePx}) {
                CameraCorner level = c;
                level.pitch = std::atan((v + dv - c.cy) / c.fy);  // the pitch that makes this row's ray level
                const bool interior = level.pitch > pLo && level.pitch < pHi;
                for (const double du : {-kBoxEdgePx, kBoxEdgePx}) {
                    double b = 0.0;
                    double e = 0.0;
                    ray(c, u + du, v + dv, b, e);
                    widen(out, b);
                    if (interior) {
                        ray(level, u + du, v + dv, b, e);
                        widen(out, b);
                    }
                }
            }
        });
        return out;
    }

    /// The ELEVATION of the point (u, v), likewise, with +-kBoxEdgePx of jitter on the row AND the column,
    /// radians. Its INTERIOR point (header): the farther the column from the principal point's, the nearer
    /// level the ray, so when the jittered column reaches the principal point's that column is evaluated
    /// too — the elevation farthest from level.
    [[nodiscard]] Interval elevation(double u, double v) const noexcept {
        Interval out{};
        double cols[3] = {};
        const int colCount = jitteredColumns(u, cols);
        forEachCorner(/*useHeight=*/false, [&](const CameraCorner& c) {
            for (int k = 0; k < colCount; ++k) {
                for (const double dv : {-kBoxEdgePx, kBoxEdgePx}) {
                    double b = 0.0;
                    double e = 0.0;
                    ray(c, cols[k], v + dv, b, e);
                    widen(out, e);
                }
            }
        });
        return out;
    }

    /// The GROUND RANGE: the horizontal distance, inches, from the LENS to where the ray through (u,
    /// vBottom) meets the floor — the range of a thing whose base is ON the floor at that row (that a
    /// model box's bottom row IS the piece's base is A4 register HA-200), over the mount's height, pitch
    /// and focal-length tolerances and +-kBoxEdgePx of jitter on the row AND the column — with the
    /// principal point's column when the jittered column reaches it (elevation()'s interior point: the ray
    /// steepest there). Invalid when any evaluated ray descends less than `minDepressionDeg`: near the
    /// horizon the range is unbounded, so it refuses rather than report an infinity.
    [[nodiscard]] Interval groundRange(double u, double vBottom, double minDepressionDeg = 2.0) const noexcept {
        Interval out{};
        bool ok = true;
        double cols[3] = {};
        const int colCount = jitteredColumns(u, cols);
        forEachCorner(/*useHeight=*/true, [&](const CameraCorner& c) {
            for (int k = 0; k < colCount; ++k) {
                for (const double dv : {-kBoxEdgePx, kBoxEdgePx}) {
                    double b = 0.0;
                    double e = 0.0;
                    ray(c, cols[k], vBottom + dv, b, e);
                    if (!(-e > minDepressionDeg * kDegToRad)) {
                        ok = false;
                        return;
                    }
                    widen(out, c.h / std::tan(-e));
                }
            }
        });
        if (!ok) {
            out.valid = false;
        }
        return out;
    }

    /// The SIZE RANGE: the horizontal distance, inches, from the LENS to an UPRIGHT thing `realHeightIn`
    /// tall whose box spans `pxHeight` rows centred on row `vCentre`, at column `u` (that a model box spans
    /// the WHOLE piece is A4 register HA-200). For each corner of the tolerance box the answer is EXACT for
    /// a vertical line: the ray through image row v, turned to the line's bearing b off the optical axis,
    /// rises tan(pitch - atan((v - cy) / fy)) x cos(b) per inch of horizontal distance, so the line's top
    /// and bottom rows put it at realHeightIn / (cos(b) x (that rise at the top - at the bottom)) — the
    /// pitch AND the line of sight's sideways angle carried exactly (a Pin 30 degrees off the axis is about
    /// 15 % farther than its axis depth). Widened over the corners, +-kBoxEdgePx on each of the two edges
    /// and on the column. The sideways factor is taken at the box's OBSERVED centre row: that row moves
    /// only when both edges move the same way, and such a box never sets an end — moving an edge changes
    /// the rise between the rows at least 5.8 times as much as the sideways factor, over the whole pitch
    /// span (measured on a dense grid, VIS1-A1's verification round). Its INTERIOR points (header): the
    /// column through the principal point, when the jittered column may be it (the line of sight square to
    /// the image: the NEAR end); and — for the FAR end — the pitch at which the product of the sideways
    /// factor and the rise between the two rows is smallest, which for a box straddling the lens's level
    /// lies INSIDE the pitch tolerance (the rise between two rows is smallest with the box symmetric about
    /// level). That product is log-convex in the pitch, and for every pitch smallest with the box shrunk by
    /// kBoxEdgePx at each edge and the column farther from the axis, so its minimum is searched (bisection
    /// on its log's slope) for that box at each focal-length corner, and taken as a bound from below — the
    /// far end never short. A real
    /// piece's box also takes in the depth of its top face, so the size range of a Pin seen from above
    /// reads somewhat NEARER than its near rim (a consumer comparing it with the ground range allows for
    /// that). Invalid when the box is too small to measure (no taller than 2 edges of jitter and a pixel)
    /// or the height is not a positive number; a CLIPPED box is the caller's to refuse
    /// (`hal::Detection::clipped()`).
    [[nodiscard]] Interval sizeRange(double realHeightIn, double u, double vCentre,
                                     double pxHeight) const noexcept {
        Interval out{};
        if (!(pxHeight > 2.0 * kBoxEdgePx + 1.0) || !(realHeightIn > 0.0) || !std::isfinite(realHeightIn)) {
            return out;
        }
        const double vTop = vCentre - pxHeight / 2.0;
        const double vBottom = vCentre + pxHeight / 2.0;
        const double axisCol = o_.widthPx / 2.0;  // the principal point's column (header)
        double cols[3] = {};
        const int colCount = jitteredColumns(u, cols);
        forEachCorner(/*useHeight=*/false, [&](const CameraCorner& c) {
            for (int k = 0; k < colCount; ++k) {
                for (const double dt : {-kBoxEdgePx, kBoxEdgePx}) {
                    for (const double db : {-kBoxEdgePx, kBoxEdgePx}) {
                        const double perInch =
                            sizeTerms(c, c.pitch, cols[k], vCentre, vTop + dt, vBottom + db).product;
                        if (perInch > 0.0) {
                            widen(out, realHeightIn / perInch);
                        }
                    }
                }
            }
        });
        double pLo = 0.0;
        double pHi = 0.0;
        pitchSpan(pLo, pHi);
        const double colFar = std::fabs(cols[0] - axisCol) > std::fabs(cols[1] - axisCol) ? cols[0] : cols[1];
        const double top = vTop + kBoxEdgePx;  // the box shrunk at both edges: the smallest rise
        const double bottom = vBottom - kBoxEdgePx;
        const CameraCorner n = nominal();
        const double t = o_.fovTolerance;
        for (const double sx : {1.0 - t, 1.0 + t}) {
            for (const double sy : {1.0 - t, 1.0 + t}) {
                const CameraCorner c{n.fx * sx, n.fy * sy, n.cx, n.cy, n.h, n.pitch, n.yaw};
                if (!(sizeTerms(c, pLo, colFar, vCentre, top, bottom).slope < 0.0)
                    || !(sizeTerms(c, pHi, colFar, vCentre, top, bottom).slope > 0.0)) {
                    continue;  // no interior minimum: it is at an end, a corner above
                }
                double a = pLo;  // the log's slope < 0 at a, >= 0 at b
                double b = pHi;
                for (int i = 0; i < 40; ++i) {
                    const double mid = 0.5 * (a + b);
                    if (sizeTerms(c, mid, colFar, vCentre, top, bottom).slope < 0.0) {
                        a = mid;
                    } else {
                        b = mid;
                    }
                }
                // Log-convex: ln P(p) >= ln P(a) + slope(a) (p - a), and slope(a) < 0, so over [a, b] the
                // product is at least P(a) exp(slope(a) (b - a)) — a bound from below, the range's from above.
                const SizeTerms at = sizeTerms(c, a, colFar, vCentre, top, bottom);
                const double perInch = at.product * std::exp(at.slope * (b - a));
                if (perInch > 0.0) {
                    widen(out, realHeightIn / perInch);
                }
            }
        }
        return out;
    }

    /// The FIELD POSITION of a thing at `bearingI` (body frame, radians) and horizontal `rangeI` (inches)
    /// from the LENS, seen from the robot pose `robot`. The radius is how far the farthest point of the
    /// two intervals, at any heading of the pose's interval, lies from the nominal point — EXACT over the
    /// three together — plus the pose's position tolerance and the mount's. The farthest point is one of: a
    /// corner (each interval's end) at an end of the heading interval; a corner turned diametrically
    /// opposite the nominal point about the robot's centre, when the heading interval reaches that turn (a
    /// tolerance near or past 180 degrees); or the point of the FAR range's arc pointing straight away from
    /// the nominal point as seen from the lens, when its bearing is inside the bearing interval — at an end
    /// of the heading interval, or turned until the lens lies on the line through the robot's centre and
    /// the nominal point (with the lens behind the centre and the ray pointing straight away, that is the
    /// lens offset + range.hi + the nominal point's distance from the centre). Each is counted when the
    /// intervals reach it. Invalid when either interval is invalid or has a non-finite end, or the pose's x
    /// or y is not a finite number.
    /// A NEGATIVE or NON-FINITE position or heading tolerance is not a bound: the radius is then INFINITE
    /// (the nominal point still computed) — never a tight radius from a bad number.
    [[nodiscard]] FieldFix fieldFix(const PoseBound& robot, const Interval& bearingI,
                                    const Interval& rangeI) const noexcept {
        FieldFix f{};
        if (!finiteInterval(bearingI) || !finiteInterval(rangeI)) {
            return f;
        }
        const double cx = robot.pose.x().value();
        const double cy = robot.pose.y().value();
        const double th = robot.pose.heading().radians();  // finite: math::Angle refuses anything else
        if (!std::isfinite(cx) || !std::isfinite(cy)) {
            return f;
        }
        // The point at heading `heading`, relative to the robot's centre (cx, cy).
        auto offset = [&](double heading, double b, double r, double& x, double& y) {
            const double ch = std::cos(heading);
            const double sh = std::sin(heading);
            x = ch * p_.forward.value() - sh * p_.left.value() + r * std::cos(heading + b);
            y = sh * p_.forward.value() + ch * p_.left.value() + r * std::sin(heading + b);
        };
        double nx = 0.0;
        double ny = 0.0;
        offset(th, bearingI.mid(), rangeI.mid(), nx, ny);
        f.x = cx + nx;
        f.y = cy + ny;
        f.valid = true;
        const double posTol = robot.positionTolerance.value();
        const double headTolDeg = robot.headingToleranceDeg;
        if (!(posTol >= 0.0) || !std::isfinite(posTol) || !(headTolDeg >= 0.0) || !std::isfinite(headTolDeg)) {
            f.radius = std::numeric_limits<double>::infinity();
            return f;
        }
        const double dTh = headTolDeg * kDegToRad;
        const double rhoN = std::hypot(nx, ny);
        double worst = 0.0;
        for (const double b : {bearingI.lo, bearingI.hi}) {
            for (const double r : {rangeI.lo, rangeI.hi}) {
                double vx = 0.0;
                double vy = 0.0;
                offset(th, b, r, vx, vy);
                for (const double h : {th - dTh, th + dTh}) {
                    double x = 0.0;
                    double y = 0.0;
                    offset(h, b, r, x, y);
                    worst = std::fmax(worst, std::hypot(x - nx, y - ny));
                }
                // The turn that puts this corner diametrically opposite the nominal point, about the
                // robot's centre: inside the heading interval, the farthest is |v| + |n|.
                const double opposite = std::remainder(
                    math::Angle::kPi - (std::atan2(vy, vx) - std::atan2(ny, nx)), 2.0 * math::Angle::kPi);
                if (std::fabs(opposite) <= dTh) {
                    worst = std::fmax(worst, std::hypot(vx, vy) + rhoN);
                }
            }
        }
        // The far range's arc at turn h: its point straight away from the nominal point, seen from the
        // lens, is |nominal - lens(h)| + range.hi away — counted when its bearing is inside the interval.
        const double fwd = p_.forward.value();
        const double lft = p_.left.value();
        const double twoPi = 2.0 * math::Angle::kPi;
        auto farArc = [&](double h) {
            const double lx = std::cos(h) * fwd - std::sin(h) * lft;
            const double ly = std::sin(h) * fwd + std::cos(h) * lft;
            const double away = std::remainder(std::atan2(ly - ny, lx - nx) - h - bearingI.mid(), twoPi);
            if (std::fabs(away) <= 0.5 * bearingI.width()) {
                worst = std::fmax(worst, std::hypot(lx - nx, ly - ny) + rangeI.hi);
            }
        };
        farArc(th - dTh);
        farArc(th + dTh);
        // Inside the heading interval that distance is largest where the lens is on the line through the
        // robot's centre and the nominal point (either side): each such turn, when the interval reaches it.
        if (std::hypot(fwd, lft) > 0.0 && rhoN > 0.0) {
            const double toNominal = std::atan2(ny, nx);
            const double lensAt = std::atan2(lft, fwd);
            for (const double h : {toNominal - lensAt, toNominal + math::Angle::kPi - lensAt}) {
                if (std::fabs(std::remainder(h - th, twoPi)) <= dTh) {
                    farArc(h);
                }
            }
        }
        f.radius = worst + posTol + p_.positionTolerance.value();
        return f;
    }

    /// Whether the floor point at body-frame (`fwd`, `lft`) inches — or the point `up` inches above it —
    /// is inside the image, at least `marginPx` from every edge, for EVERY corner of the tolerance box.
    [[nodiscard]] bool inViewForCertain(double fwd, double lft, double up, double marginPx) const noexcept {
        bool all = true;
        forEachCorner(/*useHeight=*/true, [&](const CameraCorner& c) {
            double u = 0.0;
            double v = 0.0;
            if (!project(c, fwd, lft, up, u, v) || u < marginPx || u > o_.widthPx - marginPx
                || v < marginPx || v > o_.heightPx - marginPx) {
                all = false;
            }
        });
        return all;
    }

    /// The forward projection of the body-frame point (`fwd`, `lft`, `up`) inches through corner `c`
    /// into pixel (u, v); false when the point is not in front of the lens. Used by the in-view test (the
    /// host camera model has its own, written separately — the two must never share code).
    [[nodiscard]] bool project(const CameraCorner& c, double fwd, double lft, double up, double& u,
                               double& v) const noexcept {
        const double x = fwd - p_.forward.value();
        const double y = lft - p_.left.value();
        const double z = up - c.h;
        const double cyw = std::cos(c.yaw);
        const double syw = std::sin(c.yaw);
        const double xf = cyw * x + syw * y;  // into the yaw-free frame
        const double yl = -syw * x + cyw * y;
        const double sp = std::sin(c.pitch);
        const double cp = std::cos(c.pitch);
        const double depth = xf * cp + z * sp;
        if (depth <= 1e-6) {
            return false;
        }
        const double right = -yl;
        const double down = xf * sp - z * cp;
        u = c.cx + c.fx * right / depth;
        v = c.cy + c.fy * down / depth;
        return true;
    }

    /// Call `fn(corner)` for every corner of the tolerance box — 32 with the height tolerance, 16 without
    /// (each focal length, the pitch, the yaw and the height at each of its two extremes) — for a
    /// consumer that tests a CONSISTENCY between two quantities computed from ONE camera (the header:
    /// one camera explains it). Never overlap two intervals each widened over every corner.
    template <class F>
    void eachCorner(bool useHeight, F&& fn) const noexcept {
        forEachCorner(useHeight, fn);
    }

    /// For ONE corner: the horizontal lens distance, inches, at which the ray through pixel (u, v) meets
    /// the horizontal plane `planeIn` inches above the floor, in `out`. False when the ray meets it
    /// behind the camera or within `minAngleDeg` of level (unbounded there — refused, never an infinity).
    static bool rangeToPlane(const CameraCorner& c, double u, double v, double planeIn,
                             double minAngleDeg, double& out) noexcept {
        double b = 0.0;
        double e = 0.0;
        ray(c, u, v, b, e);
        const double rise = planeIn - c.h;
        if (std::fabs(e) < minAngleDeg * kDegToRad || e * rise <= 0.0) {
            return false;
        }
        out = rise / std::tan(e);
        return std::isfinite(out) && out > 0.0;
    }

private:
    /// The size range's terms for ONE camera `c` at pitch `p` (its own pitch ignored): `product` — the
    /// line of sight's sideways factor at the box's centre (column `col`, row `vc`) times the rise between
    /// rows `top` and `bottom` per inch of horizontal distance (the range is the height over it) — and
    /// `slope`, the derivative of the product's log in the pitch.
    struct SizeTerms {
        double product = 0.0;  ///< sideways x rise, per inch
        double slope = 0.0;    ///< d ln(product) / d pitch
    };

    [[nodiscard]] static SizeTerms sizeTerms(const CameraCorner& c, double p, double col, double vc,
                                             double top, double bottom) noexcept {
        const double sp = std::sin(p);
        const double cp = std::cos(p);
        const double xr = (col - c.cx) / c.fx;  // right, per unit depth
        const double yc = (vc - c.cy) / c.fy;   // down, per unit depth, at the centre row
        const double f = cp + yc * sp;          // the centre ray's forward part (> 0 inside the span)
        const double df = yc * cp - sp;         // its derivative in the pitch
        // Row v's ray rises tan(p - atan(yd)) per inch along the camera's own vertical plane.
        const double riseTop = std::tan(p - std::atan((top - c.cy) / c.fy));
        const double riseBottom = std::tan(p - std::atan((bottom - c.cy) / c.fy));
        const double sideways = f / std::hypot(f, xr);  // cos of the line of sight against the axis's plane
        const double share = xr * xr / (f * f + xr * xr);
        // d ln(rise difference)/dp = riseTop + riseBottom (sec^2 - sec^2 over tan - tan); d ln(sideways)/dp =
        // share x f'/f.
        return {sideways * (riseTop - riseBottom), riseTop + riseBottom + share * df / f};
    }

    /// The columns a jittered column `u` is evaluated at, into `cols`: u - kBoxEdgePx and u + kBoxEdgePx,
    /// and — when the jitter reaches it — the principal point's column (the interior point, header).
    /// Returns how many (2 or 3).
    [[nodiscard]] int jitteredColumns(double u, double (&cols)[3]) const noexcept {
        const double axisCol = o_.widthPx / 2.0;
        cols[0] = u - kBoxEdgePx;
        cols[1] = u + kBoxEdgePx;
        cols[2] = axisCol;
        return std::fabs(u - axisCol) < kBoxEdgePx ? 3 : 2;
    }

    /// The tolerance box's pitch interval [lo, hi], radians (forEachCorner's two pitches).
    void pitchSpan(double& lo, double& hi) const noexcept {
        const double p = p_.pitchDeg * kDegToRad;
        const double dp = p_.pitchToleranceDeg * kDegToRad;
        lo = p - dp;
        hi = p + dp;
    }

    /// A valid interval with finite, ordered ends (one a consumer built by hand may not be).
    [[nodiscard]] static bool finiteInterval(const Interval& i) noexcept {
        return i.valid && std::isfinite(i.lo) && std::isfinite(i.hi) && i.lo <= i.hi;
    }

    static void widen(Interval& i, double x) noexcept {
        if (!std::isfinite(x)) {
            return;
        }
        if (!i.valid) {
            i = {true, x, x};
            return;
        }
        i.lo = std::fmin(i.lo, x);
        i.hi = std::fmax(i.hi, x);
    }

    template <class F>
    void forEachCorner(bool useHeight, F&& fn) const noexcept {
        const CameraCorner n = nominal();
        const double t = o_.fovTolerance;
        const double dp = p_.pitchToleranceDeg * kDegToRad;
        const double dy = p_.yawToleranceDeg * kDegToRad;
        const double dh = useHeight ? p_.heightTolerance.value() : 0.0;
        // The two focal lengths are known APART (HA-194: two published angles, each measured on its
        // own), so each takes its own extreme — (fx down, fy up) is a corner like any other.
        for (const double sx : {1.0 - t, 1.0 + t}) {
            for (const double sy : {1.0 - t, 1.0 + t}) {
                for (const double pitch : {n.pitch - dp, n.pitch + dp}) {
                    for (const double yaw : {n.yaw - dy, n.yaw + dy}) {
                        for (const double h : {n.h - dh, n.h + dh}) {
                            fn(CameraCorner{n.fx * sx, n.fy * sy, n.cx, n.cy, h, pitch, yaw});
                            if (!useHeight) {
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    CameraOptics o_;
    config::CameraPlacement p_;
    double fx0_ = 0.0;
    double fy0_ = 0.0;
};

/// How far the mount's pitch WITH its tolerance may reach from level, degrees: inside it every ray of every
/// corner of the tolerance box points forward and the inputs the header calls monotone stay monotone, so
/// every interval here is exact (VIS1-A1's verification round: a ray of the image's edge row turns
/// backward past about 56.7 degrees). cameraModelProblem() refuses a mount that reaches past it.
inline constexpr double kMaxPitchSpanDeg = 55.0;

/// Why no CameraModel can be built for this robot's camera, or nullptr (then `CameraModel{
/// kAiVisionOptics, cfg.placement}` is valid): "no camera is declared" for port 0; the placement's own
/// problem (`config::cameraPlacementProblem` — an UNSET mount first); or a pitch whose tolerance takes it
/// past kMaxPitchSpanDeg (|pitch| + tolerance above 55 degrees — the geometry's own limit, not the robot
/// file's: the camera is still built and read). Every geometry consumer asks this first and refuses to
/// construct, naming the reason; the robot still drives (VIS1's D5).
[[nodiscard]] constexpr const char* cameraModelProblem(const config::CameraConfig& cfg) noexcept {
    if (cfg.port == 0) {
        return "no camera is declared";
    }
    if (const char* problem = config::cameraPlacementProblem(cfg.placement)) {
        return problem;
    }
    const double pitch = cfg.placement.pitchDeg < 0.0 ? -cfg.placement.pitchDeg : cfg.placement.pitchDeg;
    if (pitch + cfg.placement.pitchToleranceDeg > kMaxPitchSpanDeg) {
        return "the pitch with its tolerance reaches past 55 degrees -- the geometry holds only inside it "
               "(re-measure the tolerance: a level on the camera's back face reads about 1 degree)";
    }
    return nullptr;
}

}  // namespace shulib::vision
