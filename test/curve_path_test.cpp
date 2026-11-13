// CURVE1-A — the curve verbs' PURE path (motion/curve_path.hpp): the shape through the author's
// points, the closest-point search, the refusals, the speed plan. No robot, no plant, no clock —
// every check here is arithmetic on a built CurvePath, so a host tool (the route preview) can rely
// on exactly what is pinned. The closed-loop proof is motion_curve_test.cpp.
//
// Each case tries to break ONE rule (the brief's §2.2 row in brackets), and names the mutation
// that must turn it red:
//   CV1   the path passes through its own points, along their headings; a free point's direction is
//         the Catmull-Rom chord; the length matches an INDEPENDENT Hermite polyline (4,000 samples a
//         segment, re-derived here from the documented rule — never read back from the table); on
//         every ACCEPTED path (the brief's routes + 200 seeded random point lists) the derivative
//         never turns against its chord               [M1 tangent x0, M2 free tangent from the
//                                                       previous point only, M16b the 90 deg rule
//                                                       on headed points only]
//   CV3   the start rule's boundary: EXACTLY 90 deg drives, 90.01 deg is refused   [M5 > -> >=];
//         reversing, the words name the BACK and faceAway()        [P3 the forward words reversing]
//   CV9   the closest point never JUMPS: forward-only and windowed            [M13 whole-path search]
//   CV12  the input edges: spacing 0.99 / 1.00 / 1.01 in, the near-start drop 0.99 / 1.00 / 1.01 in
//         (EXACTLY 1.0 lands on each "closer than" rule: R7, R8, R23 `<` -> `<=`), a headed first
//         point at the robot, a lone point at the robot, out-and-back, loop-in-the-middle, a heading
//         pointing back, the reverse example written with the TRAVEL direction; a refused path is
//         still built FINITE (a tool draws it)        [M16c the free-point check after the
//                                                       normalisation: NaN]
//   CV19  the speed plan: the bend felt 2 in ahead, the backward pass, the turn-rate cap
//                                                      [M22, M22b, M22c]; held BETWEEN samples and
//         into every knot on multi-point paths, every 0.05 in   [P1 the interval bound dropped, P2
//                                                      the knot's arriving side not seen]
//   CC    CurveConfig::valid() and MotionConfig::validate() refuse every bad curve knob
//                                                      [R26 valid() always true]
//   CV21  CurvePath is trivially copyable (a cheap, heap-free copy)
//   W     the words checkCurve's verdicts earn (guide 11 quotes them)

#include "doctest.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include "shulib/core/check.hpp"
#include "shulib/diag/line_format.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/motion/curve_path.hpp"
#include "shulib/motion/motion_config.hpp"
#include "shulib/units/quantity.hpp"

namespace {

using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::motion::checkCurve;
using shulib::motion::CurveCheck;
using shulib::motion::CurveConfig;
using shulib::motion::CurveInvalid;
using shulib::motion::CurvePath;
using shulib::motion::CurvePoint;
using shulib::motion::CurveVerdict;
using shulib::units::Length;

constexpr double kPi = Angle::kPi;
constexpr double kDeg = kPi / 180.0;

CurvePoint at(double x, double y, double deg) {
    return CurvePoint{Length{x}, Length{y}, Angle::degrees(deg)};
}
CurvePoint thr(double x, double y) { return CurvePoint{Length{x}, Length{y}}; }
Pose2d P(double x, double y, double deg) { return Pose2d{Length{x}, Length{y}, Angle::degrees(deg)}; }

/// A route: a start pose, points, a direction.
struct Route {
    const char* name;
    Pose2d start;
    std::vector<CurvePoint> pts;
    bool reverse = false;
};

/// The brief's eighteen routes (§0.4), the ones the host-test-plant probes drove.
std::vector<Route> briefRoutes() {
    return {
        {"straight48", P(0, 0, 0), {at(48, 0, 0)}},
        {"scurve", P(0, 0, 0), {thr(24, 12), at(48, 24, 0)}},
        {"lanechange", P(0, 0, 0), {at(48, 24, 0)}},
        {"bend90r36", P(0, 0, 0), {at(36, 36, 90)}},
        {"bend90r18", P(0, 0, 0), {at(18, 18, 90)}},
        {"bend90r12", P(0, 0, 0), {at(12, 12, 90)}},
        {"hairpin18", P(0, 0, 0), {at(36, 18, 90), at(0, 36, 180)}},
        {"hairpin9", P(0, 0, 0), {at(24, 9, 90), at(0, 18, 180)}},
        {"hairpin6", P(0, 0, 0), {at(18, 6, 90), at(0, 12, 180)}},
        {"off30", P(0, 0, 30), {at(48, 0, 0)}},
        {"off60", P(0, 0, 60), {at(48, 0, 0)}},
        {"off90", P(0, 0, 90), {at(48, 0, 0)}},
        {"off150", P(0, 0, 150), {at(48, 0, 0)}},
        {"snake", P(0, 0, 0), {thr(36, 18), thr(12, 48), thr(36, 72), at(60, 72, 0)}},
        {"fieldS", P(-48, -48, 90), {thr(-48, -12), thr(-12, 0), thr(24, 12), at(48, 48, 90)}},
        {"revS", P(0, 0, 180), {thr(24, 12), at(48, 24, 180)}, true},
        {"endfree", P(0, 0, 0), {thr(24, 12), thr(48, 24)}},
        {"crossing", P(0, 0, 0), {thr(30, 0), thr(36, 18), thr(24, 30), thr(12, 18), at(12, -24, -90)}},
    };
}

CurvePath built(const Route& r, const CurveConfig& cfg = {}, double vmax = 60.0, double wmax = 6.0) {
    CurvePath p{std::span<const CurvePoint>{r.pts}, r.reverse};
    p.build(r.start, cfg, vmax, wmax);
    return p;
}

bool accepted(const CurveCheck& c) {
    return c.verdict == CurveVerdict::Ok || c.verdict == CurveVerdict::TightBend;
}

// ── the independent oracle: the documented Hermite, re-derived ──────────────────────────
struct OKnot {
    double x, y, tx, ty;
};
/// The knots and tangents by the header's rule (start = the robot's heading (+pi reversing); a
/// headed point = its heading (+pi reversing); a free middle point = (next - previous); a free
/// last point = (last - previous)) — written here from the header text, not from the code.
std::vector<OKnot> oracleKnots(const Route& r) {
    std::vector<OKnot> k;
    const double flip = r.reverse ? kPi : 0.0;
    const double h0 = r.start.heading().radians() + flip;
    k.push_back({r.start.x().value(), r.start.y().value(), std::cos(h0), std::sin(h0)});
    for (const CurvePoint& p : r.pts) {
        k.push_back({p.x.value(), p.y.value(), 0.0, 0.0});
    }
    for (std::size_t i = 1; i < k.size(); ++i) {
        const CurvePoint& p = r.pts[i - 1];
        if (p.hasHeading) {
            const double h = p.heading.radians() + flip;
            k[i].tx = std::cos(h);
            k[i].ty = std::sin(h);
        } else {
            const OKnot& a = k[i - 1];
            const OKnot& b = (i + 1 < k.size()) ? k[i + 1] : k[i];
            const double l = std::hypot(b.x - a.x, b.y - a.y);
            k[i].tx = (b.x - a.x) / l;
            k[i].ty = (b.y - a.y) / l;
        }
    }
    return k;
}
void oracleEval(const OKnot& a, const OKnot& b, double u, double& x, double& y) {
    const double m = std::hypot(b.x - a.x, b.y - a.y);
    const double h00 = 2 * u * u * u - 3 * u * u + 1;
    const double h10 = u * u * u - 2 * u * u + u;
    const double h01 = -2 * u * u * u + 3 * u * u;
    const double h11 = u * u * u - u * u;
    x = h00 * a.x + h10 * m * a.tx + h01 * b.x + h11 * m * b.tx;
    y = h00 * a.y + h10 * m * a.ty + h01 * b.y + h11 * m * b.ty;
}
double oracleLength(const Route& r) {
    const std::vector<OKnot> k = oracleKnots(r);
    double len = 0.0;
    for (std::size_t i = 0; i + 1 < k.size(); ++i) {
        double px = 0.0;
        double py = 0.0;
        oracleEval(k[i], k[i + 1], 0.0, px, py);
        for (int j = 1; j <= 4000; ++j) {
            double x = 0.0;
            double y = 0.0;
            oracleEval(k[i], k[i + 1], j / 4000.0, x, y);
            len += std::hypot(x - px, y - py);
            px = x;
            py = y;
        }
    }
    return len;
}

/// The worst (most negative) cos between the path's direction and its segment's chord, over 64
/// samples a segment — >= 0 means the path never turns against its chord (the header's property).
double worstChordCos(const CurvePath& p) {
    double worst = 1.0;
    for (int k = 0; k < p.segments(); ++k) {
        const double s0 = p.knotS(k);
        const double s1 = p.knotS(k + 1);
        const auto a = p.at(s0);
        const auto b = p.at(s1);
        const double cx = b.x - a.x;
        const double cy = b.y - a.y;
        const double cl = std::hypot(cx, cy);
        for (int j = 0; j <= 64; ++j) {
            const double s = s0 + (s1 - s0) * j / 64.0;
            // Just inside the segment at its two ends, so a knot's sample is this segment's.
            const double sc = std::clamp(s, s0 + 1e-9, s1 - 1e-9);
            const auto c = p.at(sc);
            worst = std::min(worst, (std::cos(c.direction) * cx + std::sin(c.direction) * cy) / cl);
        }
    }
    return worst;
}

/// A small deterministic generator (no std:: distribution: their outputs differ between libraries).
struct Lcg {
    std::uint64_t s;
    double next() {  // [0, 1)
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>(s >> 11) / 9007199254740992.0;
    }
    double range(double lo, double hi) { return lo + (hi - lo) * next(); }
};

std::string words(const CurveCheck& c, const CurveConfig& cfg = {}) {
    shulib::diag::lineformat::Line line;
    shulib::motion::appendCurveCheckWords(line, c, cfg);
    return std::string{line.view()};
}

}  // namespace

// ═══ CV1 — the path through its own points ══════════════════════════════════════════════
// Bug caught: a tangent of the wrong size (a corner — M1), a free point whose direction ignores the
// point after it (M2), an arc-length table that drifts from the curve it indexes.
TEST_CASE("CV1 the path passes every point, along every heading, and its length is the Hermite's") {
    for (const Route& r : briefRoutes()) {
        CAPTURE(r.name);
        const CurvePath p = built(r);
        REQUIRE(p.built());
        REQUIRE(p.segments() == static_cast<int>(r.pts.size()));
        // at(0) is the start, along the robot's heading (its back reversing — heading is the nose).
        const auto s0 = p.at(0.0);
        CHECK(std::abs(s0.x - r.start.x().value()) <= 1e-9);
        CHECK(std::abs(s0.y - r.start.y().value()) <= 1e-9);
        CHECK(std::abs(Angle::radians(s0.heading).errorTo(r.start.heading())) <= 1e-9);
        const std::vector<OKnot> ok = oracleKnots(r);
        for (int i = 1; i <= p.segments(); ++i) {
            CAPTURE(i);
            const CurvePoint& q = r.pts[static_cast<std::size_t>(i) - 1];
            const auto c = p.at(p.knotS(i));
            CHECK(std::abs(c.x - q.x.value()) <= 1e-6);
            CHECK(std::abs(c.y - q.y.value()) <= 1e-6);
            // The direction at the knot, and a hair past it: a corner (M1, tangent x0) keeps the
            // knot's own direction AT the knot but turns away from it immediately after.
            const double want = std::atan2(ok[static_cast<std::size_t>(i)].ty,
                                           ok[static_cast<std::size_t>(i)].tx);
            CHECK(std::abs(Angle::radians(c.direction).errorTo(Angle::radians(want))) <= 1e-6);
            if (i < p.segments()) {
                const auto after = p.at(p.knotS(i) + 1e-3);
                CHECK(std::abs(Angle::radians(after.direction).errorTo(Angle::radians(want)))
                      <= 1e-3);
            }
            if (q.hasHeading) {  // a headed point is passed with the NOSE at its heading
                CHECK(std::abs(Angle::radians(c.heading).errorTo(q.heading)) <= 1e-6);
            } else if (i < p.segments()) {  // a free middle point: parallel to (next - previous)
                const auto& a = ok[static_cast<std::size_t>(i) - 1];
                const auto& b = ok[static_cast<std::size_t>(i) + 1];
                const double cr = std::atan2(b.y - a.y, b.x - a.x);
                CHECK(std::abs(Angle::radians(c.direction).errorTo(Angle::radians(cr))) <= 1e-6);
            }
        }
        // The length: within 0.1 % of the independent 4,000-sample polyline.
        const double ol = oracleLength(r);
        CAPTURE(ol);
        CAPTURE(p.length());
        CHECK(std::abs(p.length() - ol) <= 1e-3 * ol);
    }
}

// Bug caught: a free point's direction taken from the point before it only (M2) — the S's middle
// point then leaves along its arriving chord instead of the smooth (next - previous).
TEST_CASE("CV1 a free middle point's direction is (next - previous), not the arriving chord") {
    const Route r{"s", P(0, 0, 0), {thr(24, 12), at(48, 0, 0)}};
    const CurvePath p = built(r);
    const auto c = p.at(p.knotS(1));
    CHECK(std::abs(c.direction - std::atan2(0.0, 48.0)) <= 1e-9);  // (48,0) - (0,0): due +x
    CHECK(std::abs(c.direction - std::atan2(12.0, 24.0)) > 0.4);  // not the arriving chord's 26.6 deg
}

// Bug caught: the doubling-back rule checked on HEADED points only (M16b) — a free point's
// Catmull-Rom direction can point back against the chord leaving it (an out-and-back with its
// neighbours apart), and the path then reverses along that segment. The property, over every
// ACCEPTED path: the direction never turns against its segment's chord.
TEST_CASE("CV1 no accepted path ever turns against its chord (routes + 200 seeded random lists)") {
    const CurveConfig cfg{};
    int acceptedCount = 0;
    for (const Route& r : briefRoutes()) {
        const CurvePath p = built(r, cfg);
        if (accepted(checkCurve(p, cfg))) {
            ++acceptedCount;
            CAPTURE(r.name);
            CHECK(worstChordCos(p) >= -1e-9);
        }
    }
    CHECK(acceptedCount == 17);  // every route but off150 (StartBehind)
    // The out-and-back the brief names (`outbackNear`-style: a free point whose neighbours are far
    // apart, so it HAS a direction — pointing back the way the path leaves).
    {
        const Route r{"outbackNear", P(0, 0, 0), {thr(24, 12), at(4, 4, 180)}};
        const CurvePath p = built(r, cfg);
        const CurveCheck c = checkCurve(p, cfg);
        CHECK(c.verdict == CurveVerdict::Invalid);
        CHECK(c.why == CurveInvalid::DoublesBack);
        CHECK(c.point == 1);
        CHECK(worstChordCos(p) < -0.5);  // what the rule refuses: it WOULD reverse
    }
    Lcg rng{0xC0FFEEULL};
    int randomAccepted = 0;
    int randomRefused = 0;
    for (int n = 0; n < 200; ++n) {
        Route r{"random", P(rng.range(-24, 24), rng.range(-24, 24), rng.range(-180, 180)), {}};
        const int count = 1 + static_cast<int>(rng.next() * 5.0);
        for (int i = 0; i < count; ++i) {
            const double x = rng.range(-60, 60);
            const double y = rng.range(-60, 60);
            if (rng.next() < 0.5) {
                r.pts.push_back(thr(x, y));
            } else {
                r.pts.push_back(at(x, y, rng.range(-180, 180)));
            }
        }
        const CurvePath p = built(r, cfg);
        const CurveCheck c = checkCurve(p, cfg);
        if (accepted(c)) {
            ++randomAccepted;
            CAPTURE(n);
            CHECK(worstChordCos(p) >= -1e-9);
        } else {
            ++randomRefused;
        }
    }
    // The set must exercise both sides (measured: the counts below), or the property proves little.
    CAPTURE(randomAccepted);
    CHECK(randomAccepted >= 20);
    CHECK(randomRefused >= 20);
}

// ═══ CV3 — the start rule's boundary, landed on EXACTLY ═════════════════════════════════
// Bug caught: `>` written `>=` (M5) — a first point exactly 90 deg off the nose (the robot facing
// 90 deg, the point dead ahead along +x) would be refused, though the rule says 90 drives.
TEST_CASE("CV3 a first point exactly 90 deg off the nose drives; 90.01 deg is refused") {
    const CurveConfig cfg{};
    const Route r90{"off90", P(0, 0, 90), {at(48, 0, 0)}};
    const CurvePath p90 = built(r90, cfg);
    REQUIRE(p90.startAngleOff() == cfg.maxStartOff);  // the EQUALITY edge, exactly
    CHECK(checkCurve(p90, cfg).verdict != CurveVerdict::StartBehind);
    const Route r9001{"off90.01", P(0, 0, 90.01), {at(48, 0, 0)}};
    const CurvePath p9001 = built(r9001, cfg);
    REQUIRE(p9001.startAngleOff() > cfg.maxStartOff);
    const CurveCheck c = checkCurve(p9001, cfg);
    CHECK(c.verdict == CurveVerdict::StartBehind);
    CHECK(c.point == 1);
    // Mirrored for reverse: the robot's BACK is the reference — and the words say so, with the turn
    // that fixes it (face() would point the NOSE at the point: refused again).
    const Route rr{"rev-in-front", P(0, 0, 0), {thr(48, 0)}, true};
    const CurveCheck rc = checkCurve(built(rr, cfg), cfg);
    CHECK(rc.verdict == CurveVerdict::StartBehind);
    CHECK(rc.reverse);
    CHECK(words(rc) == "curve not driven: its first point is 180 deg off the robot's back (at most 90) "
                       "-- turn first: faceAway()");
    CHECK_FALSE(c.reverse);  // the forward refusal above keeps the forward words
    const Route rb{"rev-behind", P(0, 0, 0), {at(-48, 0, 0)}, true};
    CHECK(checkCurve(built(rb, cfg), cfg).verdict == CurveVerdict::Ok);
}

namespace {

/// The closest arc length to (x, y) on [lo, hi], by a 0.001 in scan (the truth CV9 grades against),
/// and its distance.
struct Near {
    double s;
    double d;
};
Near nearestOn(const CurvePath& p, double x, double y, double lo, double hi) {
    Near n{lo, 1e18};
    for (double s = lo; s <= hi; s += 0.001) {
        const auto c = p.at(s);
        const double d = std::hypot(c.x - x, c.y - y);
        if (d < n.d) {
            n = Near{s, d};
        }
    }
    return n;
}

/// Displace the LATER-leg point at `sY` by `t` in, square to the later leg, toward the EARLIER leg
/// [eLo, eHi]; require that pose to be nearer the earlier leg than the later one [lLo, lHi]; then
/// project() from 4 in back on the later leg must stay on it.
void requireNoJump(const CurvePath& p, double sY, double t, double eLo, double eHi, double lLo,
                   double lHi, double window) {
    const auto Y = p.at(sY);
    const Near E = nearestOn(p, Y.x, Y.y, eLo, eHi);
    const auto Ep = p.at(E.s);
    double nx = -std::sin(Y.direction);
    double ny = std::cos(Y.direction);
    if (nx * (Ep.x - Y.x) + ny * (Ep.y - Y.y) < 0.0) {
        nx = -nx;
        ny = -ny;
    }
    const double px = Y.x + t * nx;
    const double py = Y.y + t * ny;
    const Near late = nearestOn(p, px, py, lLo, lHi);
    const Near early = nearestOn(p, px, py, eLo, eHi);
    CAPTURE(late.d);
    CAPTURE(early.d);
    REQUIRE(early.d < late.d - 0.05);  // the pose IS nearer the earlier leg
    const double from = late.s - 4.0;
    REQUIRE(from >= lLo);
    const double got = p.project(Pose2d{Length{px}, Length{py}, Angle{}}, from, window);
    CHECK(got >= from);
    CHECK(std::abs(got - late.s) <= 0.05);
}

}  // namespace

// ═══ CV9 — the closest point never jumps ════════════════════════════════════════════════
// Bug caught: the closest point searched over the WHOLE path (M13) — a robot on a later leg that
// passes near an earlier one would be snapped back to it (or a robot on an earlier leg forward to a
// later one). The closed loop did NOT catch this on the probes (P7): this pure test must.
TEST_CASE("CV9 project() is forward-only and windowed: a crossing and a 3 in-wide hairpin") {
    const CurveConfig cfg{};
    SUBCASE("the crossing path: on the LATER leg, nearer the earlier one") {
        const Route r = briefRoutes()[17];
        REQUIRE(std::string{r.name} == "crossing");
        const CurvePath p = built(r, cfg);
        // The last segment, (12, 18) -> (12, -24), crosses the first leg: find where, by scan.
        double sX = p.knotS(4);
        double bestD = 1e18;
        for (double s = p.knotS(4); s <= p.length(); s += 0.05) {
            const auto c = p.at(s);
            const Near e = nearestOn(p, c.x, c.y, 0.0, p.knotS(1));
            if (e.d < bestD) {
                bestD = e.d;
                sX = s;
            }
        }
        REQUIRE(bestD < 0.05);  // they cross
        // AT the crossing, 0.1 in square off the later leg: ~0 in from the earlier leg.
        requireNoJump(p, sX, 0.1, 0.0, p.knotS(1), p.knotS(4), p.length(), cfg.searchWindow);
    }
    SUBCASE("a 3 in-wide hairpin: on the return leg, nearer the outbound one") {
        // Two straight legs 3 in apart (y = 0 out, y = 3 back), joined by a tight turn.
        const Route r{"hairpin3", P(0, 0, 0), {at(24, 0, 0), at(24, 3, 180), at(0, 3, 180)}};
        const CurvePath p = built(r, cfg);
        // Halfway down the return leg, moved square toward the outbound leg until it is 0.1 in
        // nearer it: (gap + 0.1) / 2.
        const double sY = 0.5 * (p.knotS(2) + p.length());
        const auto Y = p.at(sY);
        const double gap = nearestOn(p, Y.x, Y.y, 0.0, p.knotS(1)).d;
        CAPTURE(gap);
        REQUIRE(gap <= 3.5);  // "3 in-wide"
        requireNoJump(p, sY, (gap + 0.1) / 2.0, 0.0, p.knotS(1), p.knotS(2), p.length(),
                      cfg.searchWindow);
    }
    SUBCASE("a pose BEHIND the current s: s never decreases") {
        const CurvePath p = built(briefRoutes()[0], cfg);  // straight48
        const double got = p.project(P(10, 0, 0), 20.0, cfg.searchWindow);
        CHECK(got == 20.0);
        // ...and the window bounds how far it may move forward in one call.
        CHECK(p.project(P(40, 0, 0), 20.0, cfg.searchWindow) <= 20.0 + cfg.searchWindow + 1e-12);
    }
}

// ═══ CV12 — the input edges ═════════════════════════════════════════════════════════════
// Bug caught: a spacing rule that is off by its own boundary; the near-start drop applied to a HEADED
// point (or not at all); a refusal that names the wrong point; a refused path built with NaN
// (M16c: the free-point check moved after the normalisation — the probe's divide-by-zero).
TEST_CASE("CV12 spacing: consecutive points 0.99 in apart are refused by name, 1.01 in accepted") {
    const CurveConfig cfg{};
    const CurveCheck bad = checkCurve(built({"", P(0, 0, 0), {thr(24, 0), at(24.99, 0, 0)}}, cfg), cfg);
    CHECK(bad.verdict == CurveVerdict::Invalid);
    CHECK(bad.why == CurveInvalid::TooClose);
    CHECK(bad.point == 2);
    CHECK(bad.distance == doctest::Approx(0.99));
    CHECK(words(bad) == "curve not driven: point 2 is 0.99 in from the point before it (at least 1.0)");
    const CurveCheck ok = checkCurve(built({"", P(0, 0, 0), {thr(24, 0), at(25.01, 0, 0)}}, cfg), cfg);
    CHECK(ok.verdict != CurveVerdict::Invalid);
    // EXACTLY kCurveMinSpacing apart ("closer than 1 in" is refused, so 1.0 drives) — R7 (< -> <=).
    const CurvePath exact = built({"", P(0, 0, 0), {thr(24, 0), at(25, 0, 0)}}, cfg);
    REQUIRE(exact.knotS(2) - exact.knotS(1) == doctest::Approx(1.0));
    CHECK(exact.invalidFacts().verdict == CurveVerdict::Ok);
    CHECK(accepted(checkCurve(exact, cfg)));
}

// Bug caught: the no-direction rule off by its own boundary (R23: `<` -> `<=`). A headingless point
// whose two neighbours are EXACTLY 1 in apart has a direction (theirs); 0.99 in apart it has none.
TEST_CASE("CV12 a headingless point's neighbours: 0.99 in apart is NoDirection, exactly 1.0 is not") {
    const CurveConfig cfg{};
    // The robot at (0, 0) facing +y; a free point up at (0.5, 10); the last point back down beside
    // the start, facing -y. The free point's neighbours are the start and the last point.
    const CurvePath exact = built({"", P(0, 0, 90), {thr(0.5, 10), at(1, 0, -90)}}, cfg);
    CHECK(exact.invalidFacts().verdict == CurveVerdict::Ok);  // exactly 1.0 apart: (1, 0) - (0, 0)
    const CurveCheck c = checkCurve(built({"", P(0, 0, 90), {thr(0.5, 10), at(0.99, 0, -90)}}, cfg), cfg);
    CHECK(c.verdict == CurveVerdict::Invalid);
    CHECK(c.why == CurveInvalid::NoDirection);
    CHECK(c.point == 1);
    CHECK(c.distance == doctest::Approx(0.99));
}

TEST_CASE("CV12 the near-start drop: a headingless first point 0.99 in away is dropped, 1.01 kept") {
    const CurveConfig cfg{};
    {
        const CurvePath p = built({"", P(0, 0, 0), {thr(0.99, 0), at(48, 0, 0)}}, cfg);
        CHECK(p.droppedFirstDistance() == doctest::Approx(0.99));
        CHECK(p.segments() == 1);
        CHECK(p.firstDrivenPoint() == 2);
        const CurveCheck c = checkCurve(p, cfg);
        CHECK(c.verdict == CurveVerdict::Ok);  // no false TightBend from a 0.99 in "segment"
        CHECK(std::isinf(p.tightestRadius()));
        // A rebuild from elsewhere starts from the points as written (the dropped one restored).
        CurvePath q = p;
        q.build(P(-24, 0, 0), cfg, 60.0, 6.0);
        CHECK(q.droppedFirstDistance() < 0.0);
        CHECK(q.segments() == 2);
        CHECK(q.at(q.knotS(1)).x == doctest::Approx(0.99));
    }
    {
        const CurvePath p = built({"", P(0, 0, 0), {thr(1.01, 0), at(48, 0, 0)}}, cfg);
        CHECK(p.droppedFirstDistance() < 0.0);
        CHECK(p.segments() == 2);
    }
    {  // EXACTLY 1.0 in away: kept ("closer than 1 in" is dropped) — R8 (< -> <=)
        const CurvePath p = built({"", P(0, 0, 0), {thr(1, 0), at(48, 0, 0)}}, cfg);
        CHECK(p.droppedFirstDistance() < 0.0);
        CHECK(p.segments() == 2);
        CHECK(p.invalidFacts().verdict == CurveVerdict::Ok);  // nor refused as too close
    }
    {  // a HEADED first point at the robot is refused, never dropped
        const CurvePath p = built({"", P(0, 0, 0), {at(0.99, 0, 0), at(48, 0, 0)}}, cfg);
        const CurveCheck c = checkCurve(p, cfg);
        CHECK(c.verdict == CurveVerdict::Invalid);
        CHECK(c.why == CurveInvalid::AtRobot);
        CHECK(c.point == 1);
        CHECK(words(c) == "curve not driven: point 1 is where the robot already is (1.0 in away) -- "
                          "do not list the start");
    }
    {  // a lone headingless point at the robot: dropped, nothing left
        const CurvePath p = built({"", P(5, 5, 0), {thr(5.5, 5)}}, cfg);
        const CurveCheck c = checkCurve(p, cfg);
        CHECK(c.verdict == CurveVerdict::Invalid);
        CHECK(c.why == CurveInvalid::NoPoints);
        CHECK(words(c) == "curve not driven: no point left to drive to -- do not list the start");
        CHECK(std::isfinite(p.length()));
        CHECK(p.lastPoint().x().value() == 5.5);  // the point as written, for the waiting record
    }
}

TEST_CASE("CV12 paths that double back are refused, naming the point — and are still built finite") {
    const CurveConfig cfg{};
    struct Case {
        Route r;
        CurveInvalid why;
        int point;
    };
    const std::vector<Case> cases = {
        // out-and-back through a free point: its neighbours coincide -> no direction (the NaN)
        {{"outback", P(0, 0, 0), {thr(24, 12), at(0, 0, 180)}}, CurveInvalid::NoDirection, 1},
        {{"outbackNearNeighbours", P(0, 0, 0), {thr(24, 12), at(0.5, 0.5, 180)}},
         CurveInvalid::NoDirection, 1},
        // a loop in the middle: a heading pointing back along the arriving chord
        {{"loopmid", P(0, 0, 0), {at(24, 0, 180), at(0, 24, 180)}}, CurveInvalid::DoublesBack, 1},
        // a heading pointing back (forward curve, the end facing the start)
        {{"revHead", P(0, 0, 0), {at(36, 0, 180)}}, CurveInvalid::DoublesBack, 1},
        // the reverse example written with the TRAVEL direction (the first draft's mistake)
        {{"revEx180", P(0, 24, 0), {thr(-12, 30), at(-36, 36, 180)}, true}, CurveInvalid::DoublesBack, 2},
    };
    for (const Case& k : cases) {
        CAPTURE(k.r.name);
        const CurvePath p = built(k.r, cfg);
        const CurveCheck c = checkCurve(p, cfg);
        CHECK(c.verdict == CurveVerdict::Invalid);
        CHECK(c.why == k.why);
        CHECK(c.point == k.point);
        // Built finite: a tool draws a refused path (M16c: NaN here).
        REQUIRE(std::isfinite(p.length()));
        for (int i = 0; i <= 64; ++i) {
            const auto s = p.at(p.length() * i / 64.0);
            REQUIRE(std::isfinite(s.x));
            REQUIRE(std::isfinite(s.y));
            REQUIRE(std::isfinite(s.direction));
            REQUIRE(std::isfinite(s.curvature));
            REQUIRE(std::isfinite(p.speedLimitAt(p.length() * i / 64.0)));
        }
    }
    // The NoDirection words name the fix.
    const CurveCheck nd = checkCurve(built(cases[0].r, cfg), cfg);
    CHECK(words(nd) == "curve not driven: point 1 has no direction: the points either side of it are "
                       "0.00 in apart -- give it a heading");
    const CurveCheck db = checkCurve(built(cases[4].r, cfg), cfg);
    CHECK(words(db).find("curve not driven: point 2 doubles back: its direction is ") == 0);
    CHECK(words(db).find("a heading is the robot's NOSE, also backwards") != std::string::npos);
    // The reverse example as it must be written (the NOSE: 0 deg) is accepted.
    const Route revEx0{"revEx0", P(0, 24, 0), {thr(-12, 30), at(-36, 36, 0)}, true};
    CHECK(accepted(checkCurve(built(revEx0, cfg), cfg)));
}

TEST_CASE("CV12 counts: 1 and 16 points are accepted, 0 and 17 are preconditions") {
    const CurveConfig cfg{};
    std::vector<CurvePoint> sixteen;
    for (int i = 1; i <= 16; ++i) {
        sixteen.push_back(thr(6.0 * i, (i % 2 == 0) ? 1.0 : -1.0));
    }
    const CurvePath p16 = built({"16", P(0, 0, 0), sixteen}, cfg);
    CHECK(p16.segments() == 16);
    CHECK(accepted(checkCurve(p16, cfg)));
    CHECK(accepted(checkCurve(built({"1", P(0, 0, 0), {thr(48, 0)}}, cfg), cfg)));
    std::vector<CurvePoint> seventeen = sixteen;
    seventeen.push_back(thr(200, 0));
    CHECK_THROWS(CurvePath(std::span<const CurvePoint>{seventeen}, false));
    CHECK_THROWS(CurvePath(std::span<const CurvePoint>{}, false));
    std::vector<CurvePoint> nan = {thr(24, 0), thr(36, std::numeric_limits<double>::quiet_NaN())};
    CHECK_THROWS(CurvePath(std::span<const CurvePoint>{nan}, false));
}

// ═══ CV19 — the speed plan (pure) ═══════════════════════════════════════════════════════
// Bug caught: the bend felt only AT a sample, not over the 2 in after it (M22 — the robot enters
// the bend above its speed); the backward pass removed (M22b — it brakes IN the bend, not before);
// the turn-rate cap removed (M22c — a tight bend's planned v * k exceeds maxAngularSpeed, which the
// omega clamp then silently under-turns).
TEST_CASE("CV19 the speed plan: the bend 2 in ahead, the backward pass, the turn-rate cap") {
    const CurveConfig cfg{};
    // A 24 in straight (samples every 1.5 in) into a 2.5 in-radius quarter turn: the last straight
    // sample before the bend is 1.5 in before it — inside the 2 in the plan must look ahead.
    const Route r{"straightIntoBend", P(0, 0, 0), {at(24, 0, 0), at(26.5, 2.5, 90)}};
    const CurvePath p = built(r, cfg);
    const double bendStart = p.knotS(1);
    // The bend's own limit, from the curvature the plan sees (dense scan here, independently).
    for (int i = 0; i < p.sampleCount(); ++i) {
        const double s = p.sampleS(i);
        double kk = 0.0;
        for (double t = s; t <= s + 2.0 + 1e-9; t += 0.01) {
            kk = std::max(kk, std::abs(p.at(std::min(t, p.length())).curvature));
        }
        const double bendLimit = kk > 0.0 ? std::sqrt(cfg.bendAccel / kk) : 1e9;
        CAPTURE(i);
        CAPTURE(s);
        // 0.5 % slack: the plan scans every 0.25 in, this every 0.01 in.
        CHECK(p.sampleSpeed(i) <= bendLimit * 1.005 + 1e-9);
    }
    // The sample 1.5 in before the bend is already at the bend's speed (M22: it would not be).
    double kBend = 0.0;
    for (double t = bendStart; t <= bendStart + 2.0; t += 0.01) {
        kBend = std::max(kBend, std::abs(p.at(t).curvature));
    }
    const double vBend = std::sqrt(cfg.bendAccel / kBend);
    CHECK(p.speedLimitAt(bendStart - 1.5) <= vBend * 1.01);
    // The backward pass: approaching the bend, the plan never asks to slow faster than decel.
    for (int i = 0; i + 1 < p.sampleCount(); ++i) {
        const double ds = p.sampleS(i + 1) - p.sampleS(i);
        const double v0 = p.sampleSpeed(i);
        const double v1 = p.sampleSpeed(i + 1);
        CAPTURE(i);
        CHECK(v0 * v0 <= v1 * v1 + 2.0 * cfg.decel * ds + 1e-3);  // M22b: 60 -> bend in one step
    }
    // And it is 0 at the end, rising from kPlanStartSpeed at the start.
    CHECK(p.speedLimitAt(p.length()) == 0.0);
    CHECK(p.speedLimitAt(0.0) == doctest::Approx(CurvePath::kPlanStartSpeed));
    // The turn-rate cap: with a hard grip (120 in/s^2) a 2.5 in bend would be taken at
    // sqrt(120 * 2.5) = 17.3 in/s = 6.9 rad/s > 6 — the plan must cap it to maxAngular / k.
    CurveConfig hard = cfg;
    hard.bendAccel = 120.0;
    const CurvePath ph = built(r, hard, 60.0, 6.0);
    double worstOmega = 0.0;
    for (int i = 0; i < ph.sampleCount(); ++i) {
        worstOmega = std::max(worstOmega,
                              ph.sampleSpeed(i) * std::abs(ph.at(ph.sampleS(i)).curvature));
    }
    CAPTURE(worstOmega);
    CHECK(worstOmega <= 6.0 + 1e-6);
    CHECK(worstOmega >= 5.5);  // the cap is what binds here (not the grip)
}

// Bug caught: the plan bounded only AT its samples. Samples are 1/16 of a segment apart (5 in on an
// 85 in segment) while the bend scan looked only 2 in ahead of each (P1), and at a knot at(knotS) is
// the NEXT segment, so the arriving segment's end was never seen (P2): on the host test plant the
// robot then took the last inches into a knot 13-43 % faster than the bend grip allows (the review
// of 2026-11-12). Here the interpolated plan is checked every 0.05 in, and just short of every knot,
// on every multi-point route and a long 2-point one: v <= sqrt(bendAccel / |k|) and v |k| <=
// maxAngular, 0.5 % slack (measured worst 1.0002 at 0.01 in; 1.23-1.46 before the fix).
TEST_CASE("CV19 the plan holds between its samples and into every knot (every 0.05 in, 5 paths)") {
    std::vector<Route> routes;
    for (const Route& r : briefRoutes()) {
        if (r.pts.size() >= 3) {
            routes.push_back(r);  // snake, fieldS, crossing (and any later multi-point route)
        }
    }
    routes.push_back(briefRoutes()[16]);  // endfree: two free points
    routes.push_back({"long2seg", P(0, 0, 0), {at(80, 10, 60), at(140, 60, 0)}});
    REQUIRE(routes.size() == 5);
    for (const double grip : {60.0, 120.0}) {
        CurveConfig cfg{};
        cfg.bendAccel = grip;  // 120: the turn-rate cap binds on the tighter bends
        for (const Route& r : routes) {
            CAPTURE(std::string{r.name});
            CAPTURE(grip);
            const CurvePath p = built(r, cfg, 60.0, 6.0);
            std::vector<double> ss;
            for (double s = 0.0; s <= p.length(); s += 0.05) {
                ss.push_back(s);
            }
            for (int k = 1; k <= p.segments(); ++k) {
                ss.push_back(p.knotS(k) - 1e-6);  // the knot's ARRIVING side
            }
            double worstGrip = 0.0;
            double worstTurn = 0.0;
            for (const double s : ss) {
                const double kk = std::abs(p.at(s).curvature);
                const double v = p.speedLimitAt(s);
                if (kk > 0.0) {
                    worstGrip = std::max(worstGrip, v / std::sqrt(cfg.bendAccel / kk));
                    worstTurn = std::max(worstTurn, v * kk / 6.0);
                }
            }
            CAPTURE(worstGrip);
            CAPTURE(worstTurn);
            CHECK(worstGrip <= 1.005);
            CHECK(worstTurn <= 1.005);
        }
    }
}

// ═══ CC — the curve knobs' refusals ══════════════════════════════════════════════════════
// Bug caught: CurveConfig::valid() not checking (R26: always true) — a NaN look-ahead poisoning the
// pursuit arc, a 0 handoffLast silently disabling the last-resort hand-off, an angle of pi or more
// making the start rule or the guard meaningless. Under A4 this is the only check on the knobs the
// team tunes on the robot.
TEST_CASE("CC CurveConfig::valid() and MotionConfig::validate() refuse every bad curve knob") {
    using shulib::PreconditionError;
    using shulib::motion::MotionConfig;
    CHECK(CurveConfig{}.valid());
    CHECK_NOTHROW(MotionConfig{}.validate());
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    double CurveConfig::* const positive[] = {
        &CurveConfig::lookAhead,    &CurveConfig::bendAccel,       &CurveConfig::accel,
        &CurveConfig::decel,        &CurveConfig::searchWindow,    &CurveConfig::handoffDistance,
        &CurveConfig::handoffCross, &CurveConfig::handoffLast,     &CurveConfig::endSpeedGain,
        &CurveConfig::lostTime,     &CurveConfig::stuckTime};
    for (double CurveConfig::* f : positive) {
        for (const double bad : {nan, inf, -inf, 0.0, -1e-9, -1.0}) {
            MotionConfig mc{};
            mc.curve.*f = bad;
            CHECK_FALSE(mc.curve.valid());
            CHECK_THROWS_AS(mc.validate(), PreconditionError);
        }
        MotionConfig mc{};
        mc.curve.*f = 1e-9;  // the smallest kind of positive is legal
        CHECK(mc.curve.valid());
    }
    double CurveConfig::* const angles[] = {&CurveConfig::handoffHeading, &CurveConfig::lostHeading,
                                            &CurveConfig::maxStartOff};
    for (double CurveConfig::* f : angles) {
        for (const double bad : {nan, inf, 0.0, -0.1, kPi, 4.0}) {
            MotionConfig mc{};
            mc.curve.*f = bad;
            CHECK_FALSE(mc.curve.valid());
            CHECK_THROWS_AS(mc.validate(), PreconditionError);
        }
        MotionConfig mc{};
        mc.curve.*f = std::nextafter(kPi, 0.0);  // just under pi: legal
        CHECK(mc.curve.valid());
    }
    // handoffLast in (0, 1]: EXACTLY 1 is legal (the equality edge), the next double above is not.
    MotionConfig mc{};
    mc.curve.handoffLast = 1.0;
    CHECK(mc.curve.valid());
    CHECK_NOTHROW(mc.validate());
    mc.curve.handoffLast = std::nextafter(1.0, 2.0);
    CHECK_FALSE(mc.curve.valid());
    CHECK_THROWS_AS(mc.validate(), PreconditionError);
}

// ═══ CV21 — a cheap copy ════════════════════════════════════════════════════════════════
// Proves the copy is a memcpy (no heap, no hand-written copy); it does NOT prove "no pointer
// member" — that is read in the review (§7).
TEST_CASE("CV21 CurvePath is trivially copyable") {
    static_assert(std::is_trivially_copyable_v<CurvePath>);
    CHECK(std::is_trivially_copyable_v<CurvePath>);
    static_assert(std::is_trivially_copyable_v<CurvePoint>);
}

// ═══ W — the TightBend and StartBehind words, the D3 text the guide quotes ══════════════════
TEST_CASE("W the hairpin warns TightBend with its radius; a start behind names the angle") {
    const CurveConfig cfg{};
    const CurveCheck tb = checkCurve(built(briefRoutes()[8], cfg), cfg);  // hairpin6
    REQUIRE(tb.verdict == CurveVerdict::TightBend);
    CHECK(tb.tightestRadius < cfg.lookAhead / 2.0);
    CHECK(tb.tightestRadius > 3.9);
    CHECK(words(tb) == "curve: tightest bend 4.0 in -- tighter than the look-ahead can follow; "
                       "expect it to cut the corner");
    // The 9 in hairpin (radius ~5.7) does not warn.
    CHECK(checkCurve(built(briefRoutes()[7], cfg), cfg).verdict == CurveVerdict::Ok);
    const CurveCheck sb = checkCurve(built(briefRoutes()[12], cfg), cfg);  // off150
    REQUIRE(sb.verdict == CurveVerdict::StartBehind);
    CHECK(words(sb) == "curve not driven: its first point is 150 deg off the robot's heading (at most "
                       "90) -- turn first: face()");
}
