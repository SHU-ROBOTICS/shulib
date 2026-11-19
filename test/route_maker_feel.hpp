#pragma once
//
// route_maker_feel.hpp — FE1, the feel golden (chunk RMP1-B; the RMP1 brief §2.2, probe P2): the library's own
// shapeAxis at 9 630 points, printed as exact hex doubles. The suite re-makes it and compares it with
// test/goldens/route_maker/feel_curve.txt; the page's node test reads the same file and compares its JavaScript
// copy (tools/route_maker/src/model/feel.mjs) with Object.is — the sign of zero counts. Host only.

#include <cmath>
#include <cstdio>
#include <string>

#include "shulib/teleop/input_shaping.hpp"

namespace route_maker {

/// Each line "x deadband expo shapeAxis(x)" in %a: 6 dead zones x 5 curves x (257 grid + 64 off-grid) points.
[[nodiscard]] inline std::string feelCurveTable() {
    const double deadbands[] = {0.0, 0.05, 0.08, 0.15, 0.3, 0.999};
    const double curves[] = {0.0, 0.25, 0.3, 0.6, 1.0};
    unsigned long long seed = 12345;
    std::string out;
    char line[160];
    const auto put = [&](double x, double db, double ex) {
        std::snprintf(line, sizeof line, "%a %a %a %a\n", x, db, ex, shulib::teleop::shapeAxis(x, {db, ex}));
        out += line;
    };
    for (const double db : deadbands) {
        for (const double ex : curves) {
            for (int i = -128; i <= 128; ++i) {
                put(i / 128.0, db, ex);
            }
            for (int k = 0; k < 64; ++k) {  // off-grid points from a 64-bit LCG (node re-reads the x, never re-makes it)
                seed = seed * 6364136223846793005ULL + 1442695040888963407ULL;
                put(static_cast<double>(seed >> 11) / 9007199254740992.0 * 2.0 - 1.0, db, ex);
            }
        }
    }
    return out;
}

/// FE1b, the EDGES the grid never lands on (the second review): each dead zone's own edge and the doubles either
/// side of it, -0 and +0, the rails; dead zones and curves on and past their limits (valid() refuses those, and
/// the library then passes the stick through). Same line form as feelCurveTable().
[[nodiscard]] inline std::string feelEdgeTable() {
    const double deadbands[] = {0.0, 0.05, 0.08, 0.3, 0.999, 1.0, -0.1};
    const double curves[] = {0.0, 0.3, 1.0, -0.2, 1.5};
    std::string out;
    char line[160];
    for (const double db : deadbands) {
        for (const double ex : curves) {
            const double edge = db > 0.0 ? db : 0.05;
            const double xs[] = {0.0, -0.0, edge, std::nextafter(edge, 0.0), std::nextafter(edge, 1.0), 1.0, 0.5};
            for (const double x : xs) {
                for (const double s : {1.0, -1.0}) {
                    const double v = s * x;
                    std::snprintf(line, sizeof line, "%a %a %a %a\n", v, db, ex, shulib::teleop::shapeAxis(v, {db, ex}));
                    out += line;
                }
            }
        }
    }
    return out;
}

}  // namespace route_maker
