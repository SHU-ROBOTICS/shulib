// math::Reflection — MIRROR1-A's T1 (the MIRROR1 brief's §2.2): the reflection against an INDEPENDENT
// formula, the shared vectors file, the line rule, the -0.0 rule and both velocity frames. Pure math: no
// plant, no robot. Each case attacks ONE rule and names the bug it would catch.
//
// THE REFERENCE IS NEVER THE LIBRARY'S SHORTCUT. The library reflects by swapping and negating
// coordinates (exact, because the line lies at a multiple of 45 deg). Every expectation here is
// computed the long way — rotate the plane by -a, flip y, rotate back by +a, with cos and sin — so a
// shortcut with a wrong sign or a swapped pair cannot agree with it by construction. That reference
// carries its own rounding (a few ulps of the coordinate), so positions are compared to
// 1e-12 x (1 + |x| + |y|) against it, and EXACTNESS is pinned separately: a reflected reflection gives
// the position back EQUAL (==), and each reflected coordinate is, bit for bit up to its sign, one of the
// input's.

#include "doctest.h"

#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "shulib/core/check.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/reflection.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::Frame;
using shulib::math::Pose2d;
using shulib::math::Reflection;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Velocity;
using Line = shulib::math::Reflection::Line;

namespace {

constexpr double kPi = 3.14159265358979323846;

/// The line's heading, radians — the reference's OWN table (not the library's).
double lineHeading(Line l) {
    switch (l) {
        case Line::XAxis: return 0.0;
        case Line::Diagonal: return kPi / 4.0;
        case Line::YAxis: return kPi / 2.0;
        case Line::AntiDiagonal: return 3.0 * kPi / 4.0;
        case Line::None: break;
    }
    return 0.0;
}

/// The INDEPENDENT reflection of the vector (x, y) across the line at `a`: rotate by -a, flip y, rotate
/// back by +a (file banner).
void reflectRef(double a, double x, double y, double& ox, double& oy) {
    const double c = std::cos(a);
    const double s = std::sin(a);
    const double u = c * x + s * y;    // rotate by -a
    const double v = -s * x + c * y;
    const double fv = -v;              // flip
    ox = c * u - s * fv;               // rotate by +a
    oy = s * u + c * fv;
}

/// The INDEPENDENT reflected heading: the unit vector along `h` reflected, read back with atan2.
double reflectHeadingRef(double a, double h) {
    double ox = 0.0;
    double oy = 0.0;
    reflectRef(a, std::cos(h), std::sin(h), ox, oy);
    return std::atan2(oy, ox);
}

/// The shortest angle between two headings, radians.
double headingGap(double a, double b) { return std::fabs(std::remainder(a - b, 2.0 * kPi)); }

bool sameBits(double a, double b) { return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b); }

/// |a| is |b| or |c| bit for bit (the reflected coordinate is, up to its sign, one of the input's).
bool isOneOf(double a, double b, double c) {
    return sameBits(std::fabs(a), std::fabs(b)) || sameBits(std::fabs(a), std::fabs(c));
}

constexpr Line kLines[4] = {Line::XAxis, Line::Diagonal, Line::YAxis, Line::AntiDiagonal};

// ── the shared vectors file (the brief's D10/R10) ───────────────────────────────────────────────────

struct Vector {
    bool field = true;  ///< FIELD (the skills mirror) or RELATIVE (the robot-frame mirror)
    double in[3]{};     ///< x, y in inches, heading in degrees
    double out[3]{};
    int line = 0;       ///< its line number in the file (1-based)
};

/// Read one vector per line: `FIELD x y h -> x' y' h'` or `RELATIVE ...`; '#' comments and blank lines
/// skipped. A malformed line refuses the WHOLE file, naming its line number in `error`.
bool readVectors(std::istream& in, std::vector<Vector>& out, std::string& error) {
    std::string text;
    int n = 0;
    while (std::getline(in, text)) {
        ++n;
        const std::size_t first = text.find_first_not_of(" \t\r");
        if (first == std::string::npos || text[first] == '#') {
            continue;
        }
        char mode[16] = "";
        Vector v;
        v.line = n;
        int used = 0;
        const int got = std::sscanf(text.c_str(), "%15s %lf %lf %lf -> %lf %lf %lf %n", mode, &v.in[0], &v.in[1],
                                    &v.in[2], &v.out[0], &v.out[1], &v.out[2], &used);
        if (got != 7 || used != static_cast<int>(text.size())) {
            error = "line " + std::to_string(n) + ": not 'MODE x y h -> x y h': " + text;
            return false;
        }
        const std::string m{mode};
        if (m != "FIELD" && m != "RELATIVE") {
            error = "line " + std::to_string(n) + ": the mode must be FIELD or RELATIVE, not '" + m + "'";
            return false;
        }
        v.field = m == "FIELD";
        out.push_back(v);
    }
    return true;
}

std::vector<Vector> sharedVectors() {
    std::ifstream f{std::string{SHULIB_SOURCE_DIR} + "/test/goldens/mirror/skills_mirror_vectors.txt"};
    REQUIRE(f.good());
    std::vector<Vector> v;
    std::string error;
    const bool ok = readVectors(f, v, error);
    INFO(error);
    REQUIRE(ok);
    return v;
}

}  // namespace

// Bug caught: the library's mirror and the route maker's drawing disagreeing (the table is THEIRS too);
// MM1 (the field heading as -h, the RELATIVE rule) and any sign or swap slip on a field number. Every
// line of the shared table through the library: positions EQUAL (a negation is exact, and "-55.7"
// parses to the negation of "55.7"), headings within 1e-9 deg, wrapped; and the table is not empty.
TEST_CASE("MIRROR1 T1 every line of skills_mirror_vectors.txt, through math::Reflection") {
    const std::vector<Vector> vs = sharedVectors();
    int field = 0;
    int relative = 0;
    for (const Vector& v : vs) {
        CAPTURE(v.line);
        const Reflection r = Reflection::across(v.field ? Line::AntiDiagonal : Line::XAxis);
        const Pose2d got = r.apply(Pose2d{Length{v.in[0]}, Length{v.in[1]}, Angle::degrees(v.in[2])});
        CHECK(got.x().value() == v.out[0]);
        CHECK(got.y().value() == v.out[1]);
        CHECK(headingGap(got.heading().radians(), v.out[2] * kPi / 180.0) <= 1e-9 * kPi / 180.0);
        CHECK(got.heading().degrees() > -180.0);  // F3: (-180, 180]
        for (const double c : {got.x().value(), got.y().value(), got.heading().radians()}) {
            CHECK_FALSE((c == 0.0 && std::signbit(c)));  // never -0.0 (MM17)
        }
        (v.field ? field : relative) += 1;
        // and back: the table read right to left is the same mirror
        const Pose2d back = r.apply(got);
        CHECK(back.x().value() == v.in[0]);
        CHECK(back.y().value() == v.in[1]);
    }
    CHECK(field >= 17);
    CHECK(relative >= 5);
}

// Bug caught: a reader that silently skips a malformed line (a vector lost, the table passing on what is
// left) — each malformed form refuses the whole file BY LINE NUMBER.
TEST_CASE("MIRROR1 T1 the vectors reader refuses a malformed line by its number") {
    const char* const bad[] = {
        "# ok\nFIELD 1 2 3 -> 4 5 6\nFIELD 1 2 -> 3 4 5\n",            // a number missing: line 3
        "FIELD 1 2 3 -> 4 5 6\n\nMIRROR 1 2 3 -> 4 5 6\n",             // an unknown mode: line 3
        "RELATIVE 1 2 3 4 5 6\n",                                       // no arrow: line 1
        "FIELD 1 2 3 -> 4 5 6 7\n",                                     // a trailing number: line 1
    };
    const int lineOf[] = {3, 3, 1, 1};
    for (int i = 0; i < 4; ++i) {
        CAPTURE(i);
        std::istringstream in{bad[i]};
        std::vector<Vector> v;
        std::string error;
        CHECK_FALSE(readVectors(in, v, error));
        CHECK(error.rfind("line " + std::to_string(lineOf[i]) + ":", 0) == 0);
    }
    std::istringstream good{"# c\n\nFIELD -36 -55.7 0 -> 55.7 36 -90\nRELATIVE 1 2 3 -> 1 -2 -3\n"};
    std::vector<Vector> v;
    std::string error;
    CHECK(readVectors(good, v, error));
    CHECK(v.size() == 2U);
}

// Bug caught: a sign or a swap wrong on ANY of the four lines, anywhere on the field or far off it; a
// heading reflected by the wrong rule (-h, 180 - h); a reflected reflection that does not come back; a
// "-0.0" reaching a screen (MM17: the + 0.0 dropped). A seeded sweep of 10,000 poses plus the special
// ones (0, -0.0, the axis itself, +-1e4, every multiple of 45 deg, +-180), against the independent
// rotate-flip-rotate.
TEST_CASE("MIRROR1 T1 10,000 poses on every line: the independent formula, the round trip, never -0.0") {
    std::mt19937_64 rng{20261113};
    std::uniform_real_distribution<double> pos{-1e4, 1e4};
    std::uniform_real_distribution<double> head{-kPi, kPi};
    std::vector<Pose2d> poses;
    const double specials[] = {0.0, -0.0, 1e4, -1e4, 23.5, -47.0, 70.205, 1e-9};
    for (const double x : specials) {
        for (const double y : specials) {
            for (int k = -4; k <= 4; ++k) {
                poses.emplace_back(Length{x}, Length{y}, Angle::degrees(45.0 * k));
            }
            poses.emplace_back(Length{x}, Length{y}, Angle::radians(-0.0));
        }
        poses.emplace_back(Length{x}, Length{-x}, Angle::degrees(-45.0));  // on y = -x, along it
        poses.emplace_back(Length{x}, Length{x}, Angle::degrees(45.0));    // on y = x, along it
    }
    while (poses.size() < 10000U) {
        poses.emplace_back(Length{pos(rng)}, Length{pos(rng)}, Angle::radians(head(rng)));
    }
    long checked = 0;
    for (const Line line : kLines) {
        const Reflection r = Reflection::across(line);
        const double a = lineHeading(line);
        for (const Pose2d& p : poses) {
            const double x = p.x().value();
            const double y = p.y().value();
            const double h = p.heading().radians();
            const Pose2d q = r.apply(p);
            double ex = 0.0;
            double ey = 0.0;
            reflectRef(a, x, y, ex, ey);
            const double tol = 1e-12 * (1.0 + std::fabs(x) + std::fabs(y));
            const bool posOk = std::fabs(q.x().value() - ex) <= tol && std::fabs(q.y().value() - ey) <= tol;
            const bool headOk = headingGap(q.heading().radians(), reflectHeadingRef(a, h)) <= 1e-12;
            // exactness: each coordinate is one of the input's, up to sign; the round trip is EQUAL
            const bool exact = isOneOf(q.x().value(), x, y) && isOneOf(q.y().value(), x, y);
            const Pose2d back = r.apply(q);
            const bool roundTrip = back.x().value() == x && back.y().value() == y
                                   && headingGap(back.heading().radians(), h) <= 1e-12;
            const bool noNegZero = !(q.x().value() == 0.0 && std::signbit(q.x().value()))
                                   && !(q.y().value() == 0.0 && std::signbit(q.y().value()))
                                   && !(q.heading().radians() == 0.0 && std::signbit(q.heading().radians()));
            if (!(posOk && headOk && exact && roundTrip && noNegZero)) {
                INFO("line " << static_cast<int>(line) << " pose (" << x << ", " << y << ", " << h << ") -> ("
                             << q.x().value() << ", " << q.y().value() << ", " << q.heading().radians() << ")");
                CHECK(posOk);
                CHECK(headOk);
                CHECK(exact);
                CHECK(roundTrip);
                CHECK(noNegZero);
            }
            ++checked;
        }
    }
    CHECK(checked == 40000);
}

// Bug caught: the identity "applied" (a heading re-wrapped through Angle::radians can move a bit — the
// brief's landmine 6): apply() on the identity returns its input BIT FOR BIT, -0.0 included.
TEST_CASE("MIRROR1 T1 the identity returns its input bit for bit") {
    const Reflection id{};
    CHECK(id.isIdentity());
    CHECK(id == Reflection::identity());
    CHECK(id.line() == Line::None);
    const Pose2d p{Length{-0.0}, Length{12.5}, Angle::radians(-0.0)};
    const Pose2d q = id.apply(p);
    CHECK(sameBits(q.x().value(), -0.0));
    CHECK(sameBits(q.heading().radians(), -0.0));
    const ChassisSpeeds s{Velocity{-0.0}, Velocity{3.0}, AngularVelocity{-0.0}};
    CHECK(sameBits(id.apply(s, Frame::Field).vx().value(), -0.0));
    CHECK(sameBits(id.apply(s, Frame::Body).omega().value(), -0.0));
    const Angle a = Angle::degrees(37.0);
    CHECK(sameBits(id.apply(a).radians(), a.radians()));
}

// Bug caught: a line accepted that is not a multiple of 45 deg (irrational coefficients — the position is
// no longer exact), or a multiple mapped to the wrong line (135 and -45 are the SAME line).
TEST_CASE("MIRROR1 T1 acrossLineAt: every multiple of 45 deg maps to its line; any other line is refused") {
    const Line want[9] = {Line::XAxis,    Line::Diagonal, Line::YAxis,        Line::AntiDiagonal, Line::XAxis,
                          Line::Diagonal, Line::YAxis,    Line::AntiDiagonal, Line::XAxis};
    for (int k = -4; k <= 4; ++k) {
        CAPTURE(k);
        CHECK(Reflection::acrossLineAt(Angle::degrees(45.0 * k)).line() == want[k + 4]);
    }
    CHECK(Reflection::acrossLineAt(Angle::degrees(-45.0)) == Reflection::across(Line::AntiDiagonal));
    CHECK_THROWS_AS((void)Reflection::acrossLineAt(Angle::degrees(30.0)), shulib::PreconditionError);
    CHECK_THROWS_AS((void)Reflection::acrossLineAt(Angle::degrees(44.9999)), shulib::PreconditionError);
    CHECK_THROWS_AS((void)Reflection::acrossLineAt(Angle::degrees(1.0)), shulib::PreconditionError);
    // The stated tolerance, 1e-9 deg (a review finding, 2026-11-13 — 1e-5 survived): 1e-8 deg off either
    // side is refused, 1e-10 deg off is the line (the degrees -> radians -> degrees round trip moves a
    // value near 45 by ~1e-14 deg, far inside both margins).
    CHECK_THROWS_AS((void)Reflection::acrossLineAt(Angle::degrees(45.0 + 1e-8)), shulib::PreconditionError);
    CHECK_THROWS_AS((void)Reflection::acrossLineAt(Angle::degrees(45.0 - 1e-8)), shulib::PreconditionError);
    CHECK(Reflection::acrossLineAt(Angle::degrees(45.0 + 1e-10)).line() == Line::Diagonal);
    CHECK(Reflection::acrossLineAt(Angle::degrees(45.0 - 1e-10)).line() == Line::Diagonal);
}

// Bug caught: drive() reflected wrong — a FIELD command's vector not reflected or its turn not reversed;
// a BODY command keeping its turn rate (MM4) or its sideways sign. The reference is PHYSICAL: robot one
// at heading h drives body speed b, i.e. field velocity f = robotToField(b, h); robot two, its mirror
// image at heading r(h), drives the field velocity reflected (independently) with the turn reversed —
// which, in robot two's own frame, must be what the library's BODY rule gives.
TEST_CASE("MIRROR1 T1 ChassisSpeeds in both frames, against the field velocity of the mirror image") {
    std::mt19937_64 rng{7};
    std::uniform_real_distribution<double> sp{-60.0, 60.0};
    std::uniform_real_distribution<double> head{-kPi, kPi};
    for (const Line line : kLines) {
        const Reflection r = Reflection::across(line);
        const double a = lineHeading(line);
        for (int i = 0; i < 500; ++i) {
            const ChassisSpeeds b{Velocity{sp(rng)}, Velocity{sp(rng)}, AngularVelocity{sp(rng) / 20.0}};
            const Angle h = Angle::radians(head(rng));
            // FIELD: the vector reflected, the turn reversed
            const ChassisSpeeds f = shulib::math::robotToField(b, h);
            double fx = 0.0;
            double fy = 0.0;
            reflectRef(a, f.vx().value(), f.vy().value(), fx, fy);
            const ChassisSpeeds rf = r.apply(f, Frame::Field);
            CHECK(std::fabs(rf.vx().value() - fx) <= 1e-12 * (1.0 + std::fabs(fx)));
            CHECK(std::fabs(rf.vy().value() - fy) <= 1e-12 * (1.0 + std::fabs(fy)));
            CHECK(rf.omega().value() == -f.omega().value());
            // BODY: robot two's frame at the reflected heading reads the reflected field velocity
            const ChassisSpeeds want =
                shulib::math::fieldToRobot(ChassisSpeeds{Velocity{fx}, Velocity{fy}, AngularVelocity{-b.omega().value()}},
                                           Angle::radians(reflectHeadingRef(a, h.radians())));
            const ChassisSpeeds rb = r.apply(b, Frame::Body);
            CHECK(std::fabs(rb.vx().value() - want.vx().value()) <= 1e-9);
            CHECK(std::fabs(rb.vy().value() - want.vy().value()) <= 1e-9);
            CHECK(rb.omega().value() == want.omega().value());
        }
    }
    // never -0.0, in both frames
    const ChassisSpeeds zero{Velocity{0.0}, Velocity{0.0}, AngularVelocity{0.0}};
    for (const Line line : kLines) {
        for (const Frame frame : {Frame::Field, Frame::Body}) {
            const ChassisSpeeds z = Reflection::across(line).apply(zero, frame);
            CHECK_FALSE(std::signbit(z.vx().value()));
            CHECK_FALSE(std::signbit(z.vy().value()));
            CHECK_FALSE(std::signbit(z.omega().value()));
        }
    }
}
