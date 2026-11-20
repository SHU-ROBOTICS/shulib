// route_maker_runner_test.cpp — the preview engine's RUNNER (test/route_maker_runner.hpp; chunk RMP1-A2e), on the
// GCC side: the goals it builds at run time from a preset's NAME are the goals the compile-time `m["name"]` builds,
// field for field — the same declaration, its name, the preset's name, the value, NOT computed, the units. The names
// are compared as TEXT, as the library compares them: GCC's compile-time goal points at its own copies of the same
// literals (measured: equal text, other addresses). The rest of the runner is tested on the module and this native
// program by node (tools/route_maker/test/wasm/).
//
// (`computed` changes nothing a route can show: it matters only for a target past the travel limits, which no preset
// can be — the build refuses one. So only this direct comparison sees it: mutation RM28.)

#include <string>
#include <string_view>
#include <utility>

#include "doctest.h"
#include "mechanism_fixtures.hpp"
#include "route_maker_runner.hpp"

namespace {

route_maker::StepWords wordsFor(const shulib::mechanism::MechanismTable& t, const char* mech, const char* key,
                                const char* value) {
    route_maker::StepWords w;
    w.kv = {{"mech", mech}, {key, value}};
    w.mech = route_maker::mechanismNamed(t, mech);
    return w;
}

void sameGoal(const shulib::mechanism::PositionGoal& a, const shulib::mechanism::PositionGoal& b) {
    CHECK(a.mechanism == b.mechanism);
    CHECK(std::string_view{a.mechanismName} == b.mechanismName);
    CHECK((a.presetName == nullptr) == (b.presetName == nullptr));  // m.to(...) names no preset (RMP1-C)
    if (a.presetName != nullptr && b.presetName != nullptr) {
        CHECK(std::string_view{a.presetName} == b.presetName);
    }
    CHECK(a.value == b.value);
    CHECK(a.computed == b.computed);
    CHECK(a.units == b.units);
}

}  // namespace

TEST_CASE("RMP1-A2e runner: a preset by NAME is the goal the compile-time m[\"name\"] builds, field for field") {
    using namespace mech_fixture::robot_x;
    const auto& t = kRobotXMechanisms;
    sameGoal(route_maker::presetGoal(t, wordsFor(t, "arm", "preset", "score")), arm["score"]);
    sameGoal(route_maker::presetGoal(t, wordsFor(t, "arm", "preset", "stow")), arm["stow"]);
    sameGoal(route_maker::presetGoal(t, wordsFor(t, "lift", "preset", "low")), lift["low"]);
    sameGoal(route_maker::presetGoal(t, wordsFor(t, "lift", "preset", "high")), lift["high"]);
    const auto pg = route_maker::positionGoal(t, wordsFor(t, "hood", "pos", "middle"));
    const shulib::mechanism::PneumaticGoal want = hood["middle"];
    CHECK(pg.mechanism == want.mechanism);
    CHECK(std::string_view{pg.mechanismName} == want.mechanismName);
    CHECK(pg.position == want.position);
    CHECK(std::string_view{pg.positionName} == want.positionName);
    CHECK_THROWS_WITH_AS((void)route_maker::presetGoal(t, wordsFor(t, "lift", "preset", "nope")),
                         doctest::Contains("no_such_preset_on_this_mechanism"), route_maker::LinesError);
}

TEST_CASE("RMP1-A2e runner: a number is read as its C++ unit literal is -- a negative one is the literal negated") {
    using namespace shulib::units::literals;
    CHECK(route_maker::quantity<shulib::units::Length>("-55.7").value() == (-55.7_in).value());
    CHECK(route_maker::quantity<shulib::units::Length>("12").value() == (12_in).value());
    CHECK(route_maker::degreesLiteral("-90").radians() == (-90_deg).radians());
    CHECK(route_maker::degreesLiteral("180").radians() == (180_deg).radians());
    CHECK(route_maker::isNumberText("-0.125"));
    CHECK_FALSE(route_maker::isNumberText("1e3"));
    CHECK_FALSE(route_maker::isNumberText("+1"));
    CHECK_FALSE(route_maker::isNumberText("1."));
    CHECK_FALSE(route_maker::isNumberText("-"));
}

TEST_CASE("RMP1-C runner: a move TO A NUMBER is the goal the compile-time m.to(...) builds -- an arm's plain degrees, a lift's inches -- refused past the travel") {
    // BUG CAUGHT: PREVIEW running a "go up 6 in" to another height than the code (the lines carry the height the page
    // tracked: the export's own `lift.to(6_in)`), a degrees target read as inches, a target past the travel driven.
    using namespace mech_fixture::robot_x;
    using namespace shulib::units::literals;
    const auto& t = kRobotXMechanisms;
    sameGoal(route_maker::toGoal(t, wordsFor(t, "lift", "to", "16.5")), lift.to(16.5_in));
    sameGoal(route_maker::toGoal(t, wordsFor(t, "lift", "to", "0")), lift.to(0_in));
    sameGoal(route_maker::toGoal(t, wordsFor(t, "lift", "to", "24")), lift.to(24_in));
    sameGoal(route_maker::toGoal(t, wordsFor(t, "arm", "to", "95.5")), arm.to(95.5));
    sameGoal(route_maker::toGoal(t, wordsFor(t, "arm", "to", "0")), arm.to(0));  // the bottom of its travel: taken
    sameGoal(route_maker::toGoal(t, wordsFor(t, "arm", "to", "160")), arm.to(160));
    for (const auto& [m, v] : {std::pair{"lift", "24.001"}, std::pair{"lift", "-0.5"}, std::pair{"arm", "160.5"}, std::pair{"arm", "-1"}}) {
        CAPTURE(m);
        CAPTURE(v);
        CHECK_THROWS_WITH_AS((void)route_maker::toGoal(t, wordsFor(t, m, "to", v)), doctest::Contains("target_is_outside_the_travel_limits"),
                             route_maker::LinesError);
    }
    CHECK_THROWS_WITH_AS((void)route_maker::toGoal(t, wordsFor(t, "hood", "to", "1")), doctest::Contains("not an arm or a lift"),
                         route_maker::LinesError);
}
