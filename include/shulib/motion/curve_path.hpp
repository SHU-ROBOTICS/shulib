#pragma once
//
// curve_path.hpp — the PURE path of the curve verbs (chunk CURVE1, 2026-11): the smooth path a
// curveTo / reverseCurveTo drives, built from the robot's live pose and the author's points, its
// speed plan, and the refusals and warnings a path earns before the robot moves. No chassis, no
// scheduler, no PROS, no heap, no state beyond the path itself: a host tool (the route preview)
// builds and reads the same object the robot follows.
//
// ── The one rule to remember ─────────────────────────────────────────────────────────
// A point's heading is ALWAYS the way the robot's NOSE points — also on reverseCurveTo, where the
// robot travels opposite to it (its back leads). Writing the direction of TRAVEL on a reverse
// curve makes the end heading point back along the path, and the path refuses it (below).
// Do not list the robot's own spot as the first point: the path starts where the robot IS.
//
// ── The shape ────────────────────────────────────────────────────────────────────────
// Knots: the robot's live pose (filled by build(), at the curve's first live tick — never at
// construction: the boot window has no heading), then the author's points (at most
// kMaxCurvePoints). Each knot has a unit TANGENT in the direction of travel: the robot's heading
// at the start (+ pi reversing), a point's heading (+ pi reversing), or — for a point without
// one — the direction from the knot before it to the knot after it (Catmull-Rom); a last point
// without a heading takes the direction from the knot before it. Consecutive knots are joined by
// a cubic HERMITE segment whose tangent magnitude is the segment's chord, so the path passes
// through every point exactly, along its tangent. Nothing is stored densely: per segment, 17
// samples (u = 0, 1/16, ..., 1) of the arc length and of the speed plan, as float; every
// position, direction and curvature is evaluated from the two knots and their tangents.
//
// ── What is refused (checkCurve, before the robot moves) ───────────────────────────────
//   Invalid      the path doubles back or is degenerate, naming the point:
//                  * a headed first point within kCurveMinSpacing of the robot ("where the robot
//                    already is"); a headingless one there is DROPPED instead (one Info line);
//                  * a point within kCurveMinSpacing of the knot before it;
//                  * a point without a heading whose two neighbours are within kCurveMinSpacing
//                    of each other (an out-and-back through it has no direction — this is
//                    checked BEFORE the tangent is normalised, which would divide by ~0);
//                  * any point whose direction (its heading, or the one picked for it) is more
//                    than maxStartOff (90 deg) from the chord arriving at it or the chord leaving
//                    it — the start rule, at every point;
//                  * no point left (a lone headingless point at the robot, dropped).
//   StartBehind  the first point more than maxStartOff off the robot's nose (its back,
//                reversing): "turn first: face()" (reversing: "... off the robot's back ...
//                turn first: faceAway()").
//   TightBend    driven, but warned: the tightest bend's radius is under lookAhead / 2.
// Why there is no runtime "the path reverses" check: with every direction within 90 deg of both
// chords at its knot and tangent magnitude = chord, a segment's derivative along its chord c is
//   P'(u).c / |c|^2 = 6u(1-u) + (1-u)(1-3u) cos a + u(3u-2) cos b,
// linear in cos a and cos b in [0, 1], whose four corners 6u(1-u), (1-u)(1+3u), u(4-3u) and 1
// are all >= 0 on [0, 1] — so it never turns against its chord. A test property
// (test/curve_path_test.cpp CV1), not a branch.
//
// ── The speed plan (built once, in build()) ─────────────────────────────────────────────
// At each sample: the least of sqrt(bendAccel / K) — K the largest |curvature| over the sample
// intervals either side of it (each evaluated on its own segment, so a knot's ARRIVING side counts
// too: the interpolated plan is bounded between samples, not only at them) and within the next
// 2 in of path, so a bend is felt before its apex — maxAngular / K (the robot's turn-rate limit
// never has to carry the arc), and maxLinear; 0 at the end; then a BACKWARD pass,
// v[i] = min(v[i], sqrt(v[i+1]^2 + 2 decel ds)), so the robot is already at a bend's speed when
// it reaches the bend; then a FORWARD pass from kPlanStartSpeed (12 in/s) at the start,
// v[i] = min(v[i], sqrt(v[i-1]^2 + 2 accel ds)) — the plan rises by the ground actually covered,
// which under motor lag holds the robot back until it has moved (the host-test-plant probes ran
// both passes). speedLimitAt(s) interpolates it linearly in arc length.
//
// Every tunable this reads is CurveConfig's (motion_config.hpp), PROVISIONAL (A4: HA-246). Its
// own constants: kMaxCurvePoints and kCurveMinSpacing (design constants), kScanStep and kBendAhead
// (the plan's grid), and kPlanStartSpeed (12 in/s, the plan's value at the start — a fixed constant
// here, PROVISIONAL, HA-246). Proven on the host test plant only.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>

#include "shulib/core/check.hpp"
#include "shulib/diag/line_format.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/motion/motion_config.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::motion {

/// The most points one curve takes (a longer route is two curves). A design constant — the
/// path's fixed capacity, which keeps a curve on the auton task's stack — not a hardware belief.
inline constexpr std::size_t kMaxCurvePoints = 16;

/// The closest two knots may be (in): twice the translation settle's 0.5 in (HA-51). A
/// headingless first point closer than this to the robot is dropped, a headed one refused; any
/// other two consecutive knots closer than this are refused. A design constant.
inline constexpr double kCurveMinSpacing = 1.0;

/// One point the curve passes through, in FIELD inches: `{24_in, 12_in}` (the library picks the
/// smooth direction) or `{48_in, 24_in, 0_deg}` (pass here with the robot's NOSE pointing this
/// way — also on reverseCurveTo). The two constructors are the whole surface (F6-frozen as the
/// curve verbs' parameter type); there is deliberately no default constructor — it would mean "a
/// point at the field's origin", which nobody writes on purpose.
struct CurvePoint {
    units::Length x;      ///< field x (in)
    units::Length y;      ///< field y (in)
    math::Angle heading;  ///< the robot's NOSE heading here; meaningful only if hasHeading
    bool hasHeading;      ///< false: the library picks the smooth direction

    /// A point without a heading.
    CurvePoint(units::Length px, units::Length py) noexcept
        : x{px}, y{py}, heading{}, hasHeading{false} {}
    /// A point with the robot's nose heading `h` there.
    CurvePoint(units::Length px, units::Length py, math::Angle h) noexcept
        : x{px}, y{py}, heading{h}, hasHeading{true} {}
};

/// checkCurve()'s verdict, in the order it is decided.
enum class CurveVerdict : std::uint8_t {
    Ok = 0,           ///< drive it
    Invalid = 1,      ///< refused: degenerate, or doubles back (CurveInvalid says how)
    StartBehind = 2,  ///< refused: the first point more than maxStartOff off the nose (back, reversing)
    TightBend = 3,    ///< driven, warned: tightest bend radius < lookAhead / 2
};

/// Why a path is Invalid.
enum class CurveInvalid : std::uint8_t {
    None = 0,         ///< not invalid
    NoPoints = 1,     ///< nothing left to drive (a lone headingless point at the robot, dropped)
    AtRobot = 2,      ///< a HEADED first point within kCurveMinSpacing of the robot
    TooClose = 3,     ///< a point within kCurveMinSpacing of the knot before it
    NoDirection = 4,  ///< a headingless point whose two neighbours nearly coincide
    DoublesBack = 5,  ///< a point's direction more than maxStartOff from an adjacent chord
};

/// What checkCurve() found, with the numbers its words quote. `point` is the AUTHOR's 1-based
/// point number as written (a dropped first point does not renumber the rest); 0 = none.
struct CurveCheck {
    CurveVerdict verdict = CurveVerdict::Ok;  ///< the verdict
    CurveInvalid why = CurveInvalid::None;    ///< Invalid's reason
    int point = 0;                            ///< the point named (author's number), 0 = none
    double distance = 0.0;      ///< AtRobot / TooClose / NoDirection: the distance (in)
    double angle = 0.0;         ///< StartBehind / DoublesBack: the angle (rad)
    double tightestRadius = 0.0;  ///< TightBend: the tightest bend's radius (in)
    bool reverse = false;         ///< StartBehind: measured from the robot's BACK (reverseCurveTo)
};

/// A point on the path: where, which way, how sharply it bends.
struct CurveSample {
    double x = 0.0;          ///< field x (in)
    double y = 0.0;          ///< field y (in)
    double direction = 0.0;  ///< direction of TRAVEL (rad, (-pi, pi])
    double heading = 0.0;    ///< the robot's NOSE there: direction, + pi on a reverse curve
    double curvature = 0.0;  ///< signed curvature of travel (1/in), + bending left
};

/// The smooth path of one curve (file header). Built in two steps: the author's points at
/// construction, the robot's live pose by build(). FIXED capacity, no heap, trivially copyable.
/// Read by CurveFollow every tick and by host tools (the route preview): CurvePath, build(),
/// length(), at(), speedLimitAt() and checkCurve() are what a tool may rely on; nothing here is
/// frozen.
class CurvePath {
public:
    /// The author's points (1..kMaxCurvePoints, every x and y finite — preconditions) and the
    /// direction: `reverse` = the robot's BACK leads (reverseCurveTo). Holds the points; builds
    /// nothing until build().
    CurvePath(std::span<const CurvePoint> points, bool reverse) : reverse_{reverse} {
        SHULIB_PRECONDITION(!points.empty() && points.size() <= kMaxCurvePoints,
                            "CurvePath: a curve takes 1 to 16 points (a longer route is two curves)");
        for (const CurvePoint& p : points) {
            SHULIB_PRECONDITION(std::isfinite(p.x.value()) && std::isfinite(p.y.value()),
                                "CurvePath: every point's x and y must be finite");
        }
        authorCount_ = static_cast<int>(points.size());
        for (std::size_t i = 0; i < points.size(); ++i) {
            Knot& k = knots_[i + 1];
            k.x = points[i].x.value();
            k.y = points[i].y.value();
            k.heading = points[i].hasHeading ? points[i].heading.radians() : 0.0;
            k.hasHeading = points[i].hasHeading;
        }
    }

    /// Build the path from the robot's live pose `start` (its nose heading): the near-start drop,
    /// the tangents, the refusal facts checkCurve() reads, the arc-length table and the speed plan
    /// (`maxLinear` in/s and `maxAngular` rad/s: the motion's budgets). Re-callable (rebuilds from
    /// the held points). Total: a refused path is still built finite, so a tool can draw it.
    void build(const math::Pose2d& start, const CurveConfig& cfg, double maxLinear,
               double maxAngular) noexcept {
        if (base_ == 1) {
            // A rebuild starts from the points as written (a dropped point had no heading).
            knots_[1] = Knot{.x = droppedX_, .y = droppedY_};
        }
        base_ = 0;
        droppedDistance_ = -1.0;
        invalid_ = CurveCheck{};
        const Knot startKnot{.x = start.x().value(),
                             .y = start.y().value(),
                             .heading = start.heading().radians(),
                             .hasHeading = true};
        // The near-start rule (file header): a headingless first point at the robot is dropped —
        // the start takes its slot, so the points stay held once, in place.
        const double d1 = dist(startKnot, knots_[1]);
        if (d1 < kCurveMinSpacing && !knots_[1].hasHeading) {
            droppedDistance_ = d1;
            droppedX_ = knots_[1].x;
            droppedY_ = knots_[1].y;
            base_ = 1;
        }
        knots_[static_cast<std::size_t>(base_)] = startKnot;
        n_ = authorCount_ - base_;  // author points left = segments
        degenerate_ = (n_ == 0);
        if (degenerate_) {
            invalid_ = CurveCheck{.verdict = CurveVerdict::Invalid, .why = CurveInvalid::NoPoints,
                                  .point = 1, .distance = droppedDistance_};
            // A zero-length path at the robot: finite, drawable, never driven. (authorCount_ is 1
            // here, so slot 2 is free.)
            knots_[2] = startKnot;
            n_ = 1;
        }
        computeTangents();
        computeDirectionFacts();
        firstInvalid(cfg);
        computeArcLengths();
        computeSpeedPlan(cfg, maxLinear, maxAngular);
        built_ = true;
    }

    /// True once build() has run.
    [[nodiscard]] bool built() const noexcept { return built_; }
    /// True for reverseCurveTo (the robot's back leads).
    [[nodiscard]] bool reverse() const noexcept { return reverse_; }
    /// The author's point count as written (before any drop).
    [[nodiscard]] int authorPoints() const noexcept { return authorCount_; }
    /// Segments in the built path (= points driven through: a dropped first point is not one).
    [[nodiscard]] int segments() const noexcept { return n_; }
    /// The distance (in) of a DROPPED headingless first point from the robot; < 0 = none dropped.
    [[nodiscard]] double droppedFirstDistance() const noexcept { return droppedDistance_; }
    /// The author's number of the first point driven through (2 when point 1 was dropped).
    [[nodiscard]] int firstDrivenPoint() const noexcept { return base_ + 1; }

    /// The path's length (in), along the path.
    [[nodiscard]] double length() const noexcept { return cum_[static_cast<std::size_t>(n_)]; }
    /// The arc length (in) at which the path passes knot `i` (0 = the start, i = segments() the end).
    [[nodiscard]] double knotS(int i) const noexcept {
        return cum_[static_cast<std::size_t>(std::clamp(i, 0, n_))];
    }

    /// The path at arc length `s` (clamped to [0, length()]): evaluated analytically from the two
    /// knots and their tangents, `s` mapped to the segment's parameter through the arc-length table.
    [[nodiscard]] CurveSample at(double s) const noexcept {
        const Locus l = locate(s);
        return evalSample(l.seg, l.u);
    }

    /// The robot's NOSE pose at the path's end: the last point's heading if it has one, else the
    /// path's own end direction (+ pi reversing — the robot finishes with its back along the path).
    [[nodiscard]] math::Pose2d endPose() const {
        const Knot& e = K(n_);
        const double h = (e.hasHeading && !degenerate_) ? e.heading : at(length()).heading;
        return math::Pose2d{units::Length{e.x}, units::Length{e.y}, math::Angle::radians(h)};
    }

    /// The author's LAST point (x, y, and its nose heading — 0 when it has none), usable before
    /// build(): the curve publishes it as its target until the path exists.
    [[nodiscard]] math::Pose2d lastPoint() const {
        // (A lone point dropped by a build sits in droppedX_/Y_, its slot holding the start.)
        const Knot e = (base_ == 1 && authorCount_ == 1)
                           ? Knot{.x = droppedX_, .y = droppedY_}
                           : knots_[static_cast<std::size_t>(authorCount_)];
        return math::Pose2d{units::Length{e.x}, units::Length{e.y},
                            math::Angle::radians(e.hasHeading ? e.heading : 0.0)};
    }

    /// The closest point to `pose`, searched FORWARD ONLY from `fromS` over `windowIn` of path:
    /// never backwards, never the whole path — a path that comes back near itself must not make
    /// the robot jump to another leg of it. Returns an arc length in [fromS, fromS + windowIn],
    /// clamped to the path.
    [[nodiscard]] double project(const math::Pose2d& pose, double fromS, double windowIn) const noexcept {
        const double lo = std::clamp(fromS, 0.0, length());
        const double hi = std::min(length(), lo + std::max(windowIn, 0.0));
        const double px = pose.x().value();
        const double py = pose.y().value();
        auto d2 = [&](double s) {
            const CurveSample c = at(s);
            return (c.x - px) * (c.x - px) + (c.y - py) * (c.y - py);
        };
        double best = lo;
        double bestD = d2(lo);
        const int steps = static_cast<int>(std::ceil((hi - lo) / kProjectStep));
        for (int i = 1; i <= steps; ++i) {
            const double s = std::min(hi, lo + kProjectStep * i);
            const double d = d2(s);
            if (d < bestD) {
                bestD = d;
                best = s;
            }
        }
        // Refine inside one coarse step either side (golden section; bounded, deterministic).
        double a = std::max(lo, best - kProjectStep);
        double b = std::min(hi, best + kProjectStep);
        constexpr double kG = 0.6180339887498949;
        double c = b - kG * (b - a);
        double e = a + kG * (b - a);
        double dc = d2(c);
        double de = d2(e);
        for (int i = 0; i < 24; ++i) {
            if (dc < de) {
                b = e;
                e = c;
                de = dc;
                c = b - kG * (b - a);
                dc = d2(c);
            } else {
                a = c;
                c = e;
                dc = de;
                e = a + kG * (b - a);
                de = d2(e);
            }
        }
        const double r = 0.5 * (a + b);
        return d2(r) < bestD ? r : best;
    }

    /// The point `L` in of path past arc length `s` — past the end, on the end's line extended
    /// (along the end's direction of travel): the follower aims only at the drawn path or that line.
    [[nodiscard]] CurveSample lookAheadPoint(double s, double L) const noexcept {
        const double target = s + L;
        if (target < length()) {
            return at(target);
        }
        CurveSample e = at(length());
        const double over = target - length();
        e.x += over * std::cos(e.direction);
        e.y += over * std::sin(e.direction);
        e.curvature = 0.0;
        return e;
    }

    /// The tightest bend's radius (in) — 1 / the largest |curvature| on the path (scanned on each
    /// segment, at most kScanStep in apart, both ends of every sample interval — so each knot's
    /// arriving side too); +infinity for a path with no bend.
    [[nodiscard]] double tightestRadius() const noexcept {
        return maxCurvature_ > 0.0 ? 1.0 / maxCurvature_ : std::numeric_limits<double>::infinity();
    }
    /// Where (arc length, in) the tightest bend is.
    [[nodiscard]] double tightestAt() const noexcept { return maxCurvatureAt_; }
    /// The angle (rad, [0, pi]) between the robot's nose (its back, reversing) at the start and the
    /// bearing to the first point driven through.
    [[nodiscard]] double startAngleOff() const noexcept { return startOff_; }

    /// The speed plan (in/s) at arc length `s` (file header): interpolated from the per-sample table
    /// build() filled; 0 at the end.
    [[nodiscard]] double speedLimitAt(double s) const noexcept {
        const Locus l = locate(s);
        const std::size_t k = static_cast<std::size_t>(l.seg);
        const std::size_t j = static_cast<std::size_t>(l.sample);
        if (j >= kSamples - 1) {
            return static_cast<double>(speed_[k][kSamples - 1]);
        }
        const auto a = static_cast<double>(arc_[k][j]);
        const auto b = static_cast<double>(arc_[k][j + 1]);
        const double f = b > a ? std::clamp((l.local - a) / (b - a), 0.0, 1.0) : 0.0;
        const auto v0 = static_cast<double>(speed_[k][j]);
        const auto v1 = static_cast<double>(speed_[k][j + 1]);
        return v0 + f * (v1 - v0);
    }

    /// The speed plan's samples, for a tool or a test: kSamples per segment, segment-major.
    static constexpr std::size_t kSamples = 17;
    /// Sample count = segments() x kSamples (a knot is sampled twice: end of one segment, start of
    /// the next, at the same arc length and with the same speed).
    [[nodiscard]] int sampleCount() const noexcept { return n_ * static_cast<int>(kSamples); }
    /// Sample `i`'s arc length (in).
    [[nodiscard]] double sampleS(int i) const noexcept {
        const auto k = static_cast<std::size_t>(i) / kSamples;
        const auto j = static_cast<std::size_t>(i) % kSamples;
        return cum_[k] + static_cast<double>(arc_[k][j]);
    }
    /// Sample `i`'s planned speed (in/s).
    [[nodiscard]] double sampleSpeed(int i) const noexcept {
        return static_cast<double>(
            speed_[static_cast<std::size_t>(i) / kSamples][static_cast<std::size_t>(i) % kSamples]);
    }

    /// The refusal facts build() found (checkCurve() decides with them). Invalid or Ok.
    [[nodiscard]] const CurveCheck& invalidFacts() const noexcept { return invalid_; }

    /// The arc-length grid step (in) of the closest-point search, the curvature scan and the
    /// speed plan's 2 in look-ahead.
    static constexpr double kScanStep = 0.25;
    /// How far ahead (in) a bend is felt by the speed plan.
    static constexpr double kBendAhead = 2.0;
    /// The speed plan's value at the start (in/s), from which its forward pass rises at
    /// CurveConfig::accel per inch travelled. The host-test-plant probes' value; PROVISIONAL
    /// (A4: HA-246).
    static constexpr double kPlanStartSpeed = 12.0;

private:
    struct Knot {
        double x = 0.0;
        double y = 0.0;
        double heading = 0.0;  ///< the NOSE heading (rad), when hasHeading
        bool hasHeading = false;
        double tx = 1.0;  ///< unit tangent, direction of TRAVEL
        double ty = 0.0;
    };
    struct Locus {
        int seg = 0;
        double u = 0.0;
        int sample = 0;      ///< j with arc_[seg][j] <= local < arc_[seg][j+1]
        double local = 0.0;  ///< arc length within the segment
    };

    static constexpr double kProjectStep = 0.25;

    [[nodiscard]] static double dist(const Knot& a, const Knot& b) noexcept {
        return std::hypot(b.x - a.x, b.y - a.y);
    }
    [[nodiscard]] static double wrapPi(double a) noexcept {
        double w = std::remainder(a, 2.0 * math::Angle::kPi);
        if (w <= -math::Angle::kPi) {
            w += 2.0 * math::Angle::kPi;
        }
        return w;
    }
    /// The angle (rad, [0, pi]) between unit vector (tx, ty) and vector (cx, cy) (|c| > 0).
    [[nodiscard]] static double angleBetween(double tx, double ty, double cx, double cy) noexcept {
        const double l = std::hypot(cx, cy);
        if (!(l > 0.0)) {
            return 0.0;
        }
        const double c = std::clamp((tx * cx + ty * cy) / l, -1.0, 1.0);
        return std::acos(c);
    }

    /// Path knot `i` (0 = the start, segments() = the end).
    [[nodiscard]] const Knot& K(int i) const noexcept {
        return knots_[static_cast<std::size_t>(base_ + i)];
    }
    [[nodiscard]] Knot& K(int i) noexcept { return knots_[static_cast<std::size_t>(base_ + i)]; }

    void computeTangents() noexcept {
        const double flip = reverse_ ? math::Angle::kPi : 0.0;
        for (int i = 0; i <= n_; ++i) {
            Knot& k = K(i);
            if (k.hasHeading) {
                const double h = k.heading + flip;
                k.tx = std::cos(h);
                k.ty = std::sin(h);
                continue;
            }
            // Headingless: Catmull-Rom (neighbour before -> neighbour after), or for the last
            // point, from the knot before. The neighbour distance is checked BEFORE dividing.
            const Knot& prev = K(i - 1);
            const Knot& next = (i < n_) ? K(i + 1) : k;
            const double dx = next.x - prev.x;
            const double dy = next.y - prev.y;
            const double l = std::hypot(dx, dy);
            if (i < n_ && l < kCurveMinSpacing) {
                // No direction (checkCurve refuses it, naming the point): a finite fallback so a refused path still draws: the chord arriving here, else
                // the knot before's tangent.
                const double ax = k.x - prev.x;
                const double ay = k.y - prev.y;
                const double al = std::hypot(ax, ay);
                k.tx = al > 0.0 ? ax / al : prev.tx;
                k.ty = al > 0.0 ? ay / al : prev.ty;
                continue;
            }
            if (l > 0.0) {
                k.tx = dx / l;
                k.ty = dy / l;
            } else {
                k.tx = prev.tx;
                k.ty = prev.ty;
            }
        }
    }

    /// The direction-vs-chord angle (rad) at path knot `i`: the larger of the angles between its
    /// tangent and the chord arriving at it / the chord leaving it (the 90 deg rule).
    [[nodiscard]] double chordAngle(int i) const noexcept {
        const Knot& k = K(i);
        double worst = 0.0;
        if (i > 0) {
            const Knot& p = K(i - 1);
            worst = std::max(worst, angleBetween(k.tx, k.ty, k.x - p.x, k.y - p.y));
        }
        if (i < n_) {
            const Knot& q = K(i + 1);
            worst = std::max(worst, angleBetween(k.tx, k.ty, q.x - k.x, q.y - k.y));
        }
        return worst;
    }

    /// The start's angle off: the robot's direction of travel against the chord to the first knot.
    void computeDirectionFacts() noexcept {
        const Knot& s = K(0);
        const Knot& f = K(1);
        startOff_ = degenerate_ ? 0.0 : angleBetween(s.tx, s.ty, f.x - s.x, f.y - s.y);
    }

    /// The first Invalid fact, in point order (spacing, then direction, then doubling back).
    void firstInvalid(const CurveConfig& cfg) noexcept {
        if (invalid_.verdict == CurveVerdict::Invalid) {
            return;  // NoPoints
        }
        for (int i = 1; i <= n_; ++i) {
            const int author = i + base_;
            const double d = dist(K(i - 1), K(i));
            if (d < kCurveMinSpacing) {
                const bool atRobot = (i == 1 && K(i).hasHeading);
                invalid_ = CurveCheck{.verdict = CurveVerdict::Invalid,
                                      .why = atRobot ? CurveInvalid::AtRobot : CurveInvalid::TooClose,
                                      .point = author,
                                      .distance = d};
                return;
            }
            if (!K(i).hasHeading && i < n_) {
                const double nd = dist(K(i - 1), K(i + 1));
                if (nd < kCurveMinSpacing) {
                    invalid_ = CurveCheck{.verdict = CurveVerdict::Invalid,
                                          .why = CurveInvalid::NoDirection,
                                          .point = author,
                                          .distance = nd};
                    return;
                }
            }
            const double ca = chordAngle(i);
            if (ca > cfg.maxStartOff) {
                invalid_ = CurveCheck{.verdict = CurveVerdict::Invalid,
                                      .why = CurveInvalid::DoublesBack,
                                      .point = author,
                                      .angle = ca};
                return;
            }
        }
    }

    // ── the Hermite segment (tangent magnitude = chord) ──
    struct Eval {
        double x, y, dx, dy, ddx, ddy;
    };
    [[nodiscard]] Eval evalSeg(int seg, double u) const noexcept {
        const Knot& a = K(seg);
        const Knot& b = K(seg + 1);
        const double m = dist(a, b);
        const double u2 = u * u;
        const double u3 = u2 * u;
        const double h00 = 2 * u3 - 3 * u2 + 1;
        const double h10 = u3 - 2 * u2 + u;
        const double h01 = -2 * u3 + 3 * u2;
        const double h11 = u3 - u2;
        const double d00 = 6 * u2 - 6 * u;
        const double d10 = 3 * u2 - 4 * u + 1;
        const double d01 = -6 * u2 + 6 * u;
        const double d11 = 3 * u2 - 2 * u;
        const double e00 = 12 * u - 6;
        const double e10 = 6 * u - 4;
        const double e01 = -12 * u + 6;
        const double e11 = 6 * u - 2;
        return Eval{
            .x = h00 * a.x + h10 * m * a.tx + h01 * b.x + h11 * m * b.tx,
            .y = h00 * a.y + h10 * m * a.ty + h01 * b.y + h11 * m * b.ty,
            .dx = d00 * a.x + d10 * m * a.tx + d01 * b.x + d11 * m * b.tx,
            .dy = d00 * a.y + d10 * m * a.ty + d01 * b.y + d11 * m * b.ty,
            .ddx = e00 * a.x + e10 * m * a.tx + e01 * b.x + e11 * m * b.tx,
            .ddy = e00 * a.y + e10 * m * a.ty + e01 * b.y + e11 * m * b.ty,
        };
    }
    [[nodiscard]] CurveSample evalSample(int seg, double u) const noexcept {
        const Eval e = evalSeg(seg, u);
        const double sp = std::hypot(e.dx, e.dy);
        CurveSample c;
        c.x = e.x;
        c.y = e.y;
        if (sp > 1e-12) {
            c.direction = std::atan2(e.dy, e.dx);
            c.curvature = (e.dx * e.ddy - e.dy * e.ddx) / (sp * sp * sp);
        } else {
            const Knot& a = K(seg);
            c.direction = std::atan2(a.ty, a.tx);
            c.curvature = 0.0;
        }
        c.heading = reverse_ ? wrapPi(c.direction + math::Angle::kPi) : c.direction;
        return c;
    }

    /// |P'(u)| integrated over [u0, u1] (5-point Gauss-Legendre).
    [[nodiscard]] double speedIntegral(int seg, double u0, double u1) const noexcept {
        static constexpr std::array<double, 5> kX{0.0, -0.5384693101056831, 0.5384693101056831,
                                                  -0.9061798459386640, 0.9061798459386640};
        static constexpr std::array<double, 5> kW{0.5688888888888889, 0.4786286704993665,
                                                  0.4786286704993665, 0.2369268850561891,
                                                  0.2369268850561891};
        const double half = 0.5 * (u1 - u0);
        const double mid = 0.5 * (u1 + u0);
        double sum = 0.0;
        for (std::size_t i = 0; i < kX.size(); ++i) {
            const Eval e = evalSeg(seg, mid + half * kX[i]);
            sum += kW[i] * std::hypot(e.dx, e.dy);
        }
        return sum * half;
    }

    void computeArcLengths() noexcept {
        cum_[0] = 0.0;
        for (int k = 0; k < n_; ++k) {
            const auto uk = static_cast<std::size_t>(k);
            double s = 0.0;
            arc_[uk][0] = 0.0F;
            for (std::size_t j = 1; j < kSamples; ++j) {
                const double u0 = static_cast<double>(j - 1) / static_cast<double>(kSamples - 1);
                const double u1 = static_cast<double>(j) / static_cast<double>(kSamples - 1);
                s += speedIntegral(k, u0, u1);
                arc_[uk][j] = static_cast<float>(s);
            }
            cum_[uk + 1] = cum_[uk] + static_cast<double>(arc_[uk][kSamples - 1]);
        }
    }

    /// Arc length -> (segment, parameter u), through the arc-length table.
    [[nodiscard]] Locus locate(double s) const noexcept {
        Locus l;
        const double sc = std::clamp(s, 0.0, length());
        int k = 0;
        while (k + 1 < n_ && cum_[static_cast<std::size_t>(k) + 1] <= sc) {
            ++k;
        }
        const auto uk = static_cast<std::size_t>(k);
        const double local = sc - cum_[uk];
        l.seg = k;
        l.local = local;
        std::size_t j = 0;
        while (j + 1 < kSamples && static_cast<double>(arc_[uk][j + 1]) <= local) {
            ++j;
        }
        l.sample = static_cast<int>(j);
        if (j >= kSamples - 1) {
            l.u = 1.0;
            return l;
        }
        const auto a = static_cast<double>(arc_[uk][j]);
        const auto b = static_cast<double>(arc_[uk][j + 1]);
        const double f = b > a ? std::clamp((local - a) / (b - a), 0.0, 1.0) : 0.0;
        l.u = (static_cast<double>(j) + f) / static_cast<double>(kSamples - 1);
        return l;
    }

    /// |curvature| of segment `seg` at parameter `u`, evaluated ON that segment (so u = 1 is the
    /// segment's own end — the left-hand side of a knot, which at(knotS) never returns: a knot's
    /// arc length locates to the NEXT segment's start).
    [[nodiscard]] double absCurvature(int seg, double u) const noexcept {
        const Eval e = evalSeg(seg, u);
        const double sp = std::hypot(e.dx, e.dy);
        return sp > 1e-12 ? std::abs(e.dx * e.ddy - e.dy * e.ddx) / (sp * sp * sp) : 0.0;
    }

    /// The largest |curvature| on segment `seg` between its samples `j` and `j + 1`: both ends and
    /// at most kScanStep in apart, all on that segment. Folds what it sees into the tightest-bend
    /// scan (maxCurvature_ / maxCurvatureAt_).
    [[nodiscard]] double intervalCurvature(int seg, std::size_t j) noexcept {
        const auto us = static_cast<std::size_t>(seg);
        const auto a = static_cast<double>(arc_[us][j]);
        const auto b = static_cast<double>(arc_[us][j + 1]);
        const int m = std::max(1, static_cast<int>(std::ceil((b - a) / kScanStep)));
        double worst = 0.0;
        for (int i = 0; i <= m; ++i) {
            const double f = static_cast<double>(i) / static_cast<double>(m);
            const double u = (static_cast<double>(j) + f) / static_cast<double>(kSamples - 1);
            const double c = absCurvature(seg, u);
            worst = std::max(worst, c);
            if (c > maxCurvature_) {
                maxCurvature_ = c;
                maxCurvatureAt_ = cum_[us] + a + f * (b - a);  // locate() maps it back to this u
            }
        }
        return worst;
    }

    void computeSpeedPlan(const CurveConfig& cfg, double maxLinear, double maxAngular) noexcept {
        // The tightest bend is found by the interval scans below: every sample interval of every
        // segment, on its own segment, both ends (so each knot's arriving side too) and at most
        // kScanStep in apart.
        maxCurvature_ = 0.0;
        maxCurvatureAt_ = 0.0;
        const double L = length();
        // Each sample: the bend limit, the turn-rate cap, the budget. The curvature it is held to
        // is the larger of (a) the GUARANTEE — the two sample intervals either side of it, each on
        // its own segment (speedLimitAt interpolates between two samples, so a bound at both ends
        // of an interval bounds the whole interval; at a knot both sides of it are seen) — and
        // (b) the ANTICIPATION — the path over the next kBendAhead in, every kScanStep in, so a
        // bend is felt before its apex.
        const double vmax = std::max(maxLinear, 0.0);
        constexpr std::size_t kLast = kSamples - 1;
        for (int k = 0; k < n_; ++k) {
            const auto uk = static_cast<std::size_t>(k);
            for (std::size_t j = 0; j < kSamples; ++j) {
                const double s0 = cum_[uk] + static_cast<double>(arc_[uk][j]);
                double kk = 0.0;
                if (j > 0) {
                    kk = std::max(kk, intervalCurvature(k, j - 1));
                } else if (k > 0) {
                    kk = std::max(kk, intervalCurvature(k - 1, kLast - 1));
                }
                if (j < kLast) {
                    kk = std::max(kk, intervalCurvature(k, j));
                } else if (k + 1 < n_) {
                    kk = std::max(kk, intervalCurvature(k + 1, 0));
                }
                for (double s = s0; s <= s0 + kBendAhead + 1e-9; s += kScanStep) {
                    kk = std::max(kk, std::abs(at(std::min(s, L)).curvature));
                }
                double v = vmax;
                if (kk > 0.0) {
                    v = std::min(v, std::sqrt(cfg.bendAccel / kk));
                    v = std::min(v, maxAngular / kk);
                }
                speed_[uk][j] = static_cast<float>(v);
            }
        }
        // 0 at the end, then the BACKWARD pass: slow at no more than decel into every bend.
        const int last = n_ * static_cast<int>(kSamples) - 1;
        setSpeed(last, 0.0);
        for (int i = last - 1; i >= 0; --i) {
            const double ds = std::max(sampleS(i + 1) - sampleS(i), 0.0);
            const double vn = sampleSpeed(i + 1);
            setSpeed(i, std::min(sampleSpeed(i), std::sqrt(vn * vn + 2.0 * cfg.decel * ds)));
        }
        // Then a FORWARD pass from kPlanStartSpeed at the start, rising at no more than accel per
        // inch actually travelled (v[i] <= sqrt(v[i-1]^2 + 2 accel ds)) — what the host-test-plant
        // probes ran: under motor lag the robot covers less ground than its commanded ramp implies,
        // and this holds the plan back until it has.
        setSpeed(0, std::min(sampleSpeed(0), kPlanStartSpeed));
        for (int i = 1; i <= last; ++i) {
            const double ds = std::max(sampleS(i) - sampleS(i - 1), 0.0);
            const double vp = sampleSpeed(i - 1);
            setSpeed(i, std::min(sampleSpeed(i), std::sqrt(vp * vp + 2.0 * cfg.accel * ds)));
        }
        // A knot's two samples (same arc length) carry the same speed: the lower.
        for (int k = 1; k < n_; ++k) {
            const int a = k * static_cast<int>(kSamples) - 1;
            const double v = std::min(sampleSpeed(a), sampleSpeed(a + 1));
            setSpeed(a, v);
            setSpeed(a + 1, v);
        }
    }
    void setSpeed(int i, double v) noexcept {
        speed_[static_cast<std::size_t>(i) / kSamples][static_cast<std::size_t>(i) % kSamples] =
            static_cast<float>(v);
    }

    std::array<Knot, kMaxCurvePoints + 1> knots_{};
    double droppedX_ = 0.0;  ///< the author's point 1 as written, when dropped (restored on a
    double droppedY_ = 0.0;  ///< rebuild; a dropped point never has a heading)
    std::array<double, kMaxCurvePoints + 1> cum_{};
    std::array<std::array<float, kSamples>, kMaxCurvePoints> arc_{};
    std::array<std::array<float, kSamples>, kMaxCurvePoints> speed_{};
    CurveCheck invalid_{};
    double startOff_ = 0.0;
    double maxCurvature_ = 0.0;
    double maxCurvatureAt_ = 0.0;
    double droppedDistance_ = -1.0;
    int authorCount_ = 0;
    int n_ = 0;
    int base_ = 0;  ///< 1 when the author's point 1 was dropped (the start sits in its slot)
    bool reverse_ = false;
    bool built_ = false;
    bool degenerate_ = false;
};

/// The D3 verdicts for a built path (curve_path.hpp header), in order: Invalid (the first
/// degenerate or doubling-back point), then StartBehind (startAngleOff() > maxStartOff — exactly
/// maxStartOff drives), then TightBend (tightestRadius() < lookAhead / 2), else Ok. One function
/// the follower, the documentation's examples and host tools share. Precondition: built().
[[nodiscard]] inline CurveCheck checkCurve(const CurvePath& path, const CurveConfig& cfg) noexcept {
    if (path.invalidFacts().verdict == CurveVerdict::Invalid) {
        return path.invalidFacts();
    }
    if (path.startAngleOff() > cfg.maxStartOff) {
        return CurveCheck{.verdict = CurveVerdict::StartBehind,
                          .point = path.firstDrivenPoint(),
                          .angle = path.startAngleOff(),
                          .reverse = path.reverse()};
    }
    if (path.tightestRadius() < cfg.lookAhead / 2.0) {
        return CurveCheck{.verdict = CurveVerdict::TightBend,
                          .tightestRadius = path.tightestRadius()};
    }
    return CurveCheck{.verdict = CurveVerdict::Ok, .tightestRadius = path.tightestRadius()};
}

/// The words a CurveCheck earns, appended to `line` (ASCII, the one §18.3 formatter): the
/// refusals' "curve not driven: ..." and TightBend's "curve: tightest bend ..." — exactly what the
/// follower logs, so a host tool can show the same sentence. Ok appends nothing.
inline void appendCurveCheckWords(diag::lineformat::Line& line, const CurveCheck& c,
                                  const CurveConfig& cfg) {
    namespace lf = diag::lineformat;
    constexpr double kRadToDeg = 180.0 / math::Angle::kPi;
    auto pointWord = [&](int p) {
        line.appendLiteral("point ");
        lf::appendUnsigned(line, static_cast<unsigned long>(p > 0 ? p : 0));
    };
    switch (c.verdict) {
        case CurveVerdict::Ok:
            return;
        case CurveVerdict::TightBend:
            line.appendLiteral("curve: tightest bend ");
            lf::appendNum(line, c.tightestRadius, 1, 1);
            line.appendLiteral(" in -- tighter than the look-ahead can follow; expect it to cut the corner");
            return;
        case CurveVerdict::StartBehind:
            line.appendLiteral("curve not driven: its first point is ");
            lf::appendNum(line, c.angle * kRadToDeg, 1, 0);
            // Reversing, the angle is the robot's BACK's, and the turn that fixes it is faceAway().
            line.appendLiteral(c.reverse ? " deg off the robot's back (at most "
                                         : " deg off the robot's heading (at most ");
            lf::appendNum(line, cfg.maxStartOff * kRadToDeg, 1, 0);
            line.appendLiteral(c.reverse ? ") -- turn first: faceAway()" : ") -- turn first: face()");
            return;
        case CurveVerdict::Invalid:
            break;
    }
    line.appendLiteral("curve not driven: ");
    switch (c.why) {
        case CurveInvalid::None:
        case CurveInvalid::NoPoints:
            line.appendLiteral("no point left to drive to -- do not list the start");
            return;
        case CurveInvalid::AtRobot:
            pointWord(c.point);
            line.appendLiteral(" is where the robot already is (");
            lf::appendNum(line, c.distance, 1, 1);
            line.appendLiteral(" in away) -- do not list the start");
            return;
        case CurveInvalid::TooClose:
            pointWord(c.point);
            line.appendLiteral(" is ");
            lf::appendNum(line, c.distance, 1, 2);
            line.appendLiteral(" in from the point before it (at least ");
            lf::appendNum(line, kCurveMinSpacing, 1, 1);
            line.appendLiteral(")");
            return;
        case CurveInvalid::NoDirection:
            pointWord(c.point);
            line.appendLiteral(" has no direction: the points either side of it are ");
            lf::appendNum(line, c.distance, 1, 2);
            line.appendLiteral(" in apart -- give it a heading");
            return;
        case CurveInvalid::DoublesBack:
            pointWord(c.point);
            line.appendLiteral(" doubles back: its direction is ");
            lf::appendNum(line, c.angle * kRadToDeg, 1, 0);
            line.appendLiteral(" deg from the path into or out of it (at most ");
            lf::appendNum(line, cfg.maxStartOff * kRadToDeg, 1, 0);
            line.appendLiteral(") -- a heading is the robot's NOSE, also backwards");
            return;
    }
}

}  // namespace shulib::motion
