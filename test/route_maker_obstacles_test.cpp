// route_maker_obstacles_test.cpp — THE FIELD'S SOLID STRUCTURES the route builder checks against (chunk RMP1-H): the
// description carries them with their source, and every footprint agrees with the manual's drawings and with the two
// other copies of the Loaders' numbers in the tree (the library's wall keep-outs, the host test plant's field). The
// footprint-on-picture check is the page's (tools/route_maker/test/page/obstacles.page.test.mjs: the browser reads
// field.webp; this suite has no image decoder).

#include <cmath>
#include <set>
#include <string>

#include "doctest.h"
#include "override_field_obstacles.hpp"
#include "route_maker_robots.hpp"
#include "shulib/competition/override_mirror.hpp"
#include "shulib/localization/override_walls.hpp"
#include "shulib/sim/field_plant.hpp"

namespace {

namespace ob = route_maker::obstacles;

std::size_t countOf(const std::string& text, const std::string& what) {
    std::size_t n = 0;
    for (std::size_t at = text.find(what); at != std::string::npos; at = text.find(what, at + 1)) {
        ++n;
    }
    return n;
}

}  // namespace

TEST_CASE("RMP1-H: the description carries the thirteen structures, each with its source and its footprint") {
    const auto src = route_maker::robotSource("x");
    REQUIRE(src.has_value());
    const std::string j = route_maker::describe(*src);
    CHECK(countOf(j, "\"kind\": \"goal\"") == 9);
    CHECK(countOf(j, "\"kind\": \"Loader\"") == 4);
    CHECK(countOf(j, "\"source\": \"Game Manual Appendix A (v2.0 = v2.1): Field Element Locations (A10), Goal Specifications (A7)\"") == 9);
    CHECK(countOf(j, "\"source\": \"Game Manual Appendix A (v2.0 = v2.1): Field Element Locations (A10), Loader Specifications (A9)\"") == 4);
    CHECK(countOf(j, "\"footprint\": [[") == 13);
    std::set<std::string> ids;
    for (const auto& o : ob::kObstacles) {
        ids.insert(o.id);
        CHECK(countOf(j, std::string{"{\"id\": \""} + o.id + "\", \"name\": \"" + o.name + "\"") == 1);
    }
    CHECK(ids.size() == 13);
    // a red goal's centre and overhang, written as the page reads them; only the four red and blue goals overhang
    CHECK(countOf(j, "\"x\": 47.091, \"y\": -23.547, \"overhangIn\": 1.057, \"source\"") == 1);
    CHECK(countOf(j, "\"overhangIn\": 1.057,") == 4);
    CHECK(countOf(j, "\"overhangIn\": 0,") == 9);
}

TEST_CASE("RMP1-H: the overhang is A7's — a red or blue goal narrows from its 142.5 mm base to 88.8 mm; nothing else") {
    for (const auto& o : ob::kObstacles) {
        INFO(o.name);
        const std::string name{o.name};
        const bool colour = name.rfind("red goal", 0) == 0 || name.rfind("blue goal", 0) == 0;
        CHECK(o.overhang == (colour ? doctest::Approx((142.5 - 88.8) / 2.0 / 25.4) : doctest::Approx(0.0)));
    }
}

TEST_CASE("RMP1-H: the goals sit on A10's grid — mirror images across the skills diagonal, colour for colour (HA-258)") {
    const auto& m = shulib::competition::override2026::kSkillsMirror.reflection;
    for (const auto& o : ob::kObstacles) {
        if (o.kind != ob::Kind::Goal) {
            continue;
        }
        const auto q = m.apply(shulib::units::Length{o.x}, shulib::units::Length{o.y});
        const std::string colour = std::string{o.name}.substr(0, std::string{o.name}.find(' '));
        int images = 0;
        for (const auto& p : ob::kObstacles) {
            if (p.kind == ob::Kind::Goal && std::abs(p.x - q.x.value()) < 1e-9 && std::abs(p.y - q.y.value()) < 1e-9) {
                ++images;
                CHECK(std::string{p.name}.rfind(colour, 0) == 0);
            }
        }
        INFO(o.name);
        CHECK(images == 1);
        // on A10's lines: every centre at 0, +-23.547 or +-47.091 in (2381.3 / 2979.3 mm less 1783.2)
        for (const double v : {o.x, o.y}) {
            const double a = std::abs(v);
            CHECK((a < 1e-12 || std::abs(a - 598.1 / 25.4) < 1e-9 || std::abs(a - 1196.1 / 25.4) < 1e-9));
        }
    }
    // the Loaders are NOT mirror images (on the red and blue walls only): robot two's path is checked on its own
    const auto& l = ob::kObstacles[9];
    const auto q = m.apply(shulib::units::Length{l.x}, shulib::units::Length{l.y});
    for (const auto& p : ob::kObstacles) {
        CHECK_FALSE((std::abs(p.x - q.x.value()) < 1e-6 && std::abs(p.y - q.y.value()) < 1e-6));
    }
}

TEST_CASE("RMP1-H: a goal's footprint is A7's base — 5.61 in flat to flat, R3.22 corners, convex, counterclockwise") {
    for (const auto& o : ob::kObstacles) {
        const auto f = ob::footprint(o);
        INFO(o.name);
        REQUIRE(f.size() == (o.kind == ob::Kind::Goal ? 20U : 4U));
        double area = 0.0;
        for (std::size_t i = 0; i < f.size(); ++i) {
            const auto& a = f[i];
            const auto& b = f[(i + 1) % f.size()];
            const auto& c = f[(i + 2) % f.size()];
            area += a[0] * b[1] - b[0] * a[1];
            CHECK((b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]) >= -1e-9);  // convex, turning left
            CHECK(std::abs(a[0]) <= ob::kFace + 0.0005);  // inside the walls (on the 0.001 in grid)
            CHECK(std::abs(a[1]) <= ob::kFace + 0.0005);
        }
        CHECK(area > 0.0);
        if (o.kind == ob::Kind::Goal) {
            double far = 0.0;
            double flat = 0.0;
            for (const auto& p : f) {
                far = std::fmax(far, std::hypot(p[0] - o.x, p[1] - o.y));
                flat = std::fmax(flat, std::abs(p[0] - o.x));
            }
            CHECK(far == doctest::Approx(81.9 / 25.4).epsilon(0.0005));
            CHECK(flat == doctest::Approx(142.5 / 2.0 / 25.4).epsilon(0.0005));
            // the square-and-circle base's area, 4 (h s + r^2 (pi/4 - atan(s/h))) less the chords' sliver
            const double h = ob::kGoalHalfWidth;
            const double r = ob::kGoalCornerRadius;
            const double s = std::sqrt(r * r - h * h);
            CHECK(area / 2.0 == doctest::Approx(4.0 * (h * s + r * r * (std::acos(-1.0) / 4.0 - std::atan(s / h)))).epsilon(0.005));
        }
    }
}

TEST_CASE("RMP1-H: the Loaders agree with the library's wall keep-outs and the host test plant's field (HA-212)") {
    namespace walls = shulib::localization::override2026;
    const shulib::sim::FieldModel plant = shulib::sim::FieldModel::override2026();
    int loaders = 0;
    for (const auto& o : ob::kObstacles) {
        if (o.kind != ob::Kind::Loader) {
            continue;
        }
        ++loaders;
        const auto f = ob::footprint(o);
        const double x0 = std::fmin(f[0][0], f[2][0]);
        const double x1 = std::fmax(f[0][0], f[2][0]);
        INFO(o.name);
        CHECK(std::abs(o.x) == doctest::Approx(walls::kLoaderCentre.value()));
        CHECK(x1 - x0 == doctest::Approx(2.0 * walls::kLoaderHalfWidth.value()).epsilon(0.001));
        CHECK(std::abs(o.y) == doctest::Approx(walls::kWallInnerFace.value()));
        int matched = 0;
        for (const auto& b : plant.boxes()) {
            const bool sameWall =((b.yMin + b.yMax) / 2.0 > 0.0) == (o.y > 0.0);
            if (std::string{b.name} == "Loader" && sameWall && std::abs((b.xMin + b.xMax) / 2.0 - o.x) < 0.01
                && std::abs(b.xMax - b.xMin - (x1 - x0)) < 0.01) {
                const double plantInner = o.inY > 0.0 ? b.yMax : b.yMin;
                const double ourInner = o.inY > 0.0 ? std::fmax(f[0][1], f[2][1]) : std::fmin(f[0][1], f[2][1]);
                CHECK(plantInner == doctest::Approx(ourInner).epsilon(0.001));
                ++matched;
            }
        }
        CHECK(matched == 1);
    }
    CHECK(loaders == 4);
}
