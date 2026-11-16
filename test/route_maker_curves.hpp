#pragma once
//
// route_maker_curves.hpp — CV1, THE CURVE GOLDEN (chunk RMP1-A1, 2026-11-14; the RMP1 brief [R6], §2.2 CV1).
//
// The route builder draws and checks a curve with a JavaScript port of the library's CurvePath
// (tools/route_maker/src/model/curve.mjs). This makes the table that holds the two together: 200 SEEDED
// curves (2 to 16 points, headed and not, forwards and backwards, every refusal and the warning — each
// kind of curve forced in turn), built from a start pose by the REAL motion::CurvePath, judged by the REAL
// motion::checkCurve, worded by the REAL appendCurveCheckWords, and sampled by CurvePath::at at 64 arc
// lengths. It is written by `shulib_route_maker curves` into test/goldens/route_maker/curves.txt; the
// suite (route_maker_golden_test.cpp) re-makes it in memory and fails when the committed file differs,
// and the page's node test (curve.test.mjs) reads the committed file: the verdict, why, point and words
// EXACTLY, the positions within 0.001 in (the arc-length table is float; the maths libraries' last bits
// differ — the brief's A.1).
//
// The text format, one curve per four lines (numbers are std::to_chars' shortest round-trip text, so the
// page parses the C++'s doubles exactly):
//   C <i> <reverse 0|1> <start x> <start y> <start heading deg> <n> then n times: <x> <y> <heading deg | ->
//   V <verdict> <why> <point> <the library's words, possibly empty>
//   L <length in>
//   S <x0> <y0> ... <x63> <y63>    -- at(length * k / 63), k = 0..63, to 6 decimals

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string>
#include <vector>

#include "route_maker_describe.hpp"
#include "shulib/diag/line_format.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/motion/curve_path.hpp"
#include "shulib/motion/motion_config.hpp"

namespace route_maker {

/// splitmix64 — a seeded, portable generator (the same numbers on every compiler).
struct Seeded {
    std::uint64_t s;
    std::uint64_t next() {
        std::uint64_t z = (s += 0x9e3779b97f4a7c15ULL);
        z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
        z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
        return z ^ (z >> 31);
    }
    /// Uniform in [a, b).
    double in(double a, double b) { return a + (b - a) * static_cast<double>(next() >> 11) * 0x1.0p-53; }
    /// Uniform integer in [a, b].
    int pick(int a, int b) { return a + static_cast<int>(next() % static_cast<std::uint64_t>(b - a + 1)); }
};

/// A number as the page stores it: rounded to 0.001.
[[nodiscard]] inline double stored(double v) {
    const double r = std::round(v * 1000.0) / 1000.0;
    return r == 0.0 ? 0.0 : r;
}

[[nodiscard]] inline const char* verdictWord(shulib::motion::CurveVerdict v) {
    using shulib::motion::CurveVerdict;
    switch (v) {
        case CurveVerdict::Ok: return "Ok";
        case CurveVerdict::Invalid: return "Invalid";
        case CurveVerdict::StartBehind: return "StartBehind";
        case CurveVerdict::TightBend: return "TightBend";
    }
    return "?";
}
[[nodiscard]] inline const char* whyWord(shulib::motion::CurveInvalid w) {
    using shulib::motion::CurveInvalid;
    switch (w) {
        case CurveInvalid::None: return "None";
        case CurveInvalid::NoPoints: return "NoPoints";
        case CurveInvalid::AtRobot: return "AtRobot";
        case CurveInvalid::TooClose: return "TooClose";
        case CurveInvalid::NoDirection: return "NoDirection";
        case CurveInvalid::DoublesBack: return "DoublesBack";
    }
    return "?";
}

/// One author's point: x, y and (when `headed`) the nose heading in degrees.
struct GenPoint {
    double x, y, h;
    bool headed;
};

/// The 200 curves (file banner). Each curve's KIND is forced by its number, so every verdict appears.
[[nodiscard]] inline std::string curvesTable(int count = 200) {
    namespace mo = shulib::motion;
    constexpr double kPi = 3.14159265358979323846;
    const mo::CurveConfig cfg{};
    std::string out =
        "# CV1 -- the curve golden (test/route_maker_curves.hpp; `shulib_route_maker curves`). Made by the C++;\n"
        "# read by tools/route_maker/test/curve.test.mjs. Do not edit by hand: re-make it and read the diff.\n";
    for (int i = 0; i < count; ++i) {
        const std::uint64_t seed = std::uint64_t{0x5eed0000U} + static_cast<unsigned>(i);
        Seeded g{seed};
        const int kind = i % 8;
        const bool reverse = kind == 3 || (kind >= 4 && g.pick(0, 3) == 0);
        const double sx = stored(g.in(-40, 40)), sy = stored(g.in(-40, 40)), sh = stored(g.in(-179, 180));
        const int n = (i % 25 == 0) ? 16 : g.pick(2, 9);
        std::vector<GenPoint> pts;
        double dir = sh + (reverse ? 180.0 : 0.0);  // direction of travel, degrees
        double x = sx, y = sy;
        for (int k = 0; k < n; ++k) {
            double step = g.in(8, 26);
            double turn = g.in(-35, 35);
            if (kind == 4 && k == 0) {
                turn = (g.pick(0, 1) ? 1 : -1) * g.in(100, 170);  // the first point behind the nose (or back)
            }
            if (kind == 5 && k == n / 2) {
                step = g.in(0.2, 0.9);  // too close to the point before
            }
            if (kind == 7 && k == 0 && g.pick(0, 1) == 0) {
                step = g.in(0.1, 0.9);  // at the robot (headed: refused; headingless: dropped)
            }
            dir += turn;
            x += step * std::cos(dir * kPi / 180.0);
            y += step * std::sin(dir * kPi / 180.0);
            GenPoint p{stored(x), stored(y), 0.0, g.in(0, 1) < 0.3};
            if (p.headed) {
                p.h = stored(std::remainder(dir + (reverse ? 180.0 : 0.0) + g.in(-25, 25), 360.0));
            }
            if (kind == 7 && k == 0 && step < 1.0 && g.pick(0, 1) == 0) {
                p.headed = true;
                p.h = stored(sh);
            }
            if (kind == 7 && k == n - 1 && g.pick(0, 1) == 0) {
                p.headed = true;  // a heading pointing back along the path: doubles back
                p.h = stored(std::remainder(dir + (reverse ? 0.0 : 180.0) + g.in(-30, 30), 360.0));
            }
            pts.push_back(p);
            if (kind == 6 && k == 1 && n >= 3) {
                // an out-and-back: the NEXT point lands near the one before this headingless point
                pts.back().headed = false;
                const GenPoint& before = pts[pts.size() - 2];
                x = before.x + g.in(-0.4, 0.4);
                y = before.y + g.in(-0.4, 0.4);
                pts.push_back(GenPoint{stored(x), stored(y), 0.0, true});
                pts.back().h = stored(sh);
                ++k;
            }
        }
        if (pts.size() > mo::kMaxCurvePoints) {
            pts.resize(mo::kMaxCurvePoints);
        }
        std::vector<mo::CurvePoint> cp;
        for (const GenPoint& p : pts) {
            if (p.headed) {
                cp.emplace_back(shulib::units::Length{p.x}, shulib::units::Length{p.y}, shulib::math::Angle::degrees(p.h));
            } else {
                cp.emplace_back(shulib::units::Length{p.x}, shulib::units::Length{p.y});
            }
        }
        mo::CurvePath path{std::span<const mo::CurvePoint>{cp.data(), cp.size()}, reverse};
        const shulib::math::Pose2d start{shulib::units::Length{sx}, shulib::units::Length{sy}, shulib::math::Angle::degrees(sh)};
        path.build(start, cfg, 60.0, 6.0);
        const mo::CurveCheck c = mo::checkCurve(path, cfg);
        shulib::diag::lineformat::Line words;
        mo::appendCurveCheckWords(words, c, cfg);

        out += "C " + std::to_string(i) + " " + (reverse ? "1" : "0") + " " + num(sx) + " " + num(sy) + " " + num(sh) + " "
               + std::to_string(pts.size());
        for (const GenPoint& p : pts) {
            out += " " + num(p.x) + " " + num(p.y) + " " + (p.headed ? num(p.h) : std::string{"-"});
        }
        out += "\nV " + std::string{verdictWord(c.verdict)} + " " + whyWord(c.why) + " " + std::to_string(c.point) + " "
               + std::string{words.view()} + "\n";
        out += "L " + num(path.length()) + "\nS";
        for (int k = 0; k < 64; ++k) {
            const mo::CurveSample s = path.at(path.length() * k / 63.0);
            char xy[64];
            std::snprintf(xy, sizeof xy, " %.6f %.6f", s.x, s.y);  // 1e-6 in: far inside CV1's 0.001 in
            out += xy;
        }
        out += "\n";
    }
    return out;
}

}  // namespace route_maker
