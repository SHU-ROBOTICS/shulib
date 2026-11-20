// route_maker_runner_test.cpp — the preview engine's RUNNER (test/route_maker_runner.hpp; chunk RMP1-A2e), on the
// GCC side: the goals it builds at run time from a preset's NAME are the goals the compile-time `m["name"]` builds,
// field for field — the same declaration, its name, the preset's name, the value, NOT computed, the units. The names
// are compared as TEXT, as the library compares them: GCC's compile-time goal points at its own copies of the same
// literals (measured: equal text, other addresses). Since RMP1-E it also runs whole routes on robot one for the
// mechanisms' recorded state (the tap, every pace). The rest of the runner is tested on the module and this native
// program by node (tools/route_maker/test/wasm/).
//
// (`computed` changes nothing a route can show: it matters only for a target past the travel limits, which no preset
// can be — the build refuses one. So only this direct comparison sees it: mutation RM28.)

#include <algorithm>
#include <cmath>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "doctest.h"
#include "mechanism_fixtures.hpp"
#include "mechanism_service_rig.hpp"
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

// ── RMP1-E: the mechanisms' state, every pace, in the run's record ───────────────────────────────────────────────

namespace {

/// Run `steps` (route lines' step lines) on robot one; `onBuilt` (a test's) gets the fakes' world.
route_preview::Model runOnOne(const std::string& steps, route_preview::PreviewOptions opt = {}) {
    route_maker::RunState& g = route_maker::state();
    g = route_maker::RunState{};
    g.route = route_maker::parseLines("shulib-route 1\nroute robot=one name=skills park=57 stop=59\nstart x=-36 y=-60 h=90\n" + steps);
    return route_maker::runParsed(route_maker::previewRobot("one", &route_maker::registerRoute), opt);
}

/// Below the tap: the cascade's own reading after each step of the world, starting where the tap's first reading is.
struct CascadeAfter final : shulib::motion::ITickPacer {
    shulib::motion::ITickPacer* inner = nullptr;
    const shulib::mechanism::PositionSlot* cascade = nullptr;
    std::vector<double> at;
    void pace() override {
        inner->pace();
        at.push_back(cascade->position());
    }
};

/// Mechanism `slot`'s state at the record's sample `k` (the tap's last samples.size(), as mechanismsOf takes them).
double stateAt(const route_preview::Model& m, std::size_t slot, std::size_t k) {
    const std::vector<double>& v = route_maker::state().tap.at.at(slot);
    REQUIRE(v.size() >= m.samples.size());
    return v.at(v.size() - m.samples.size() + k);
}

}  // namespace

TEST_CASE("RMP1-E runner: home, up 6 in, down 2 in, back to the bottom, open the claw -- the cascade rises, falls and returns; the claw opens at its step") {
    // BUG CAUGHT: a strip that shows the page's guess, not the library's run (a height that never moves, a piston that
    // never changes), a state read from the wrong slot, or a height a pace behind the robot or rounded.
    CascadeAfter after;
    route_preview::PreviewOptions opt;
    opt.onBuilt = [&after](tester_rig::TesterStack& s, shulib::motion::ITickPacer*& world) {
        after.inner = world;
        world = &after;
        after.cascade = &static_cast<const shulib::mechanism::PositionSlot&>(*s.mechanisms.slotPtr(0));
        after.at.push_back(after.cascade->position());
    };
    const route_preview::Model m = runOnOne("step do=home mech=cascade bound=2\nstep do=moveTo mech=cascade to=6 bound=3\n"
                                            "step do=moveTo mech=cascade to=4 bound=3\nstep do=moveTo mech=cascade to=0 bound=3\n"
                                            "step do=set mech=claw pos=open\nstep do=run mech=right%20toggle volts=-6 for=0.5\n", opt);
    const auto& g = route_maker::state();
    REQUIRE(g.page.size() == 6);
    const auto& t = *g.table;
    const double tol = t.specs[0].position.tolerance;
    const double bottom = shulib::bench::robot_one_facts::cascadeBottom().value();
    CHECK(tol > 0.0);
    const auto at = [&](std::size_t step, bool last) {  // the cascade (slot 0) where page step `step` begins or ends
        const double clock = last ? g.page[step].t1 : g.page[step].t0;
        return stateAt(m, 0, route_maker::sampleAt(m, clock));
    };
    for (const auto& p : g.page) {
        CHECK(p.verdict == "ok");
    }
    CHECK(std::abs(at(0, true) - bottom) <= tol);  // homed at the bottom row
    CHECK(std::abs(at(1, true) - 6.0) <= tol);     // up 6 in
    CHECK(std::abs(at(2, true) - 4.0) <= tol);     // down 2 in
    CHECK(std::abs(at(3, true) - bottom) <= tol);  // back to the bottom
    double top = 0.0;
    for (std::size_t k = 0; k < m.samples.size(); ++k) {
        top = std::max(top, stateAt(m, 0, k));
    }
    CHECK(top <= 6.0 + tol);
    // EVERY pace: the tap's cascade is the library's reading after that pace's step of the world, exactly
    const std::vector<double>& tapped = g.tap.at.at(0);
    REQUIRE(tapped.size() == after.at.size());
    std::size_t off = 0;
    std::size_t moved = 0;
    for (std::size_t k = 0; k < tapped.size(); ++k) {
        off += tapped[k] != after.at[k] ? 1U : 0U;
        moved += k > 0 && after.at[k] != after.at[k - 1] ? 1U : 0U;
    }
    CHECK(off == 0U);
    CHECK(moved > 100U);  // it moves through the run: the comparison is not of a height standing still
    // the claw (slot 3): closed until its step, open from the pace after the step began (the lines move at once)
    const std::size_t c0 = route_maker::sampleAt(m, g.page[4].t0);
    CHECK(stateAt(m, 3, c0) == 0.0);
    CHECK(stateAt(m, 3, c0 + 1) == 1.0);
    const std::size_t end = route_maker::sampleAt(m, g.page[5].t1);  // the route's end (the end action follows)
    CHECK(stateAt(m, 3, end) == 1.0);
    for (const std::size_t slot : {4U, 5U}) {
        CHECK(stateAt(m, slot, end) == 0.0);  // the flipper and the ramp never moved
    }
    // the right toggle (slot 1) runs BACKWARD during its step only; the left (slot 2) never
    const std::size_t r0 = route_maker::sampleAt(m, g.page[5].t0);
    CHECK(stateAt(m, 1, r0) == 0.0);
    CHECK(stateAt(m, 1, r0 + 1) == -1.0);
    CHECK(stateAt(m, 2, r0 + 1) == 0.0);
}

TEST_CASE("RMP1-E runner: a lift that STALLS on something in its way records the height where it stopped, and the step failed") {
    // BUG CAUGHT: the strip drawing the target (6 in) for a lift that stopped at 3 in, or a stalled step shown done.
    namespace mech = shulib::mechanism;
    const mech::MechanismSpec& cascade = shulib::bench::robot_one::kRobotOneMechanisms.specs[0];
    const double blockedAt = 3.0;
    route_preview::PreviewOptions opt;
    opt.onBuilt = [&](tester_rig::TesterStack&, shulib::motion::ITickPacer*& world) {
        auto* fakes = dynamic_cast<tester_rig::FakesByKind*>(world);
        REQUIRE(fakes != nullptr);
        fakes->arms().at(0)->setObstacleDeg(mech::positionToAngleDeg(cascade.position, blockedAt));
    };
    const route_preview::Model m = runOnOne("step do=home mech=cascade bound=2\nstep do=moveTo mech=cascade to=6 bound=3\n"
                                            "step do=wait s=0.5\n", opt);
    const auto& g = route_maker::state();
    CHECK(g.page[0].verdict == "ok");
    CHECK(g.page[1].verdict == "failed");
    CHECK(g.page[2].verdict == "skipped");
    bool stalled = false;
    for (const route_preview::Step& s : m.steps) {
        stalled = stalled || (s.what == "cascade" && s.verdict == "STALLED");
    }
    CHECK(stalled);
    const double end = stateAt(m, 0, m.samples.size() - 1);
    CHECK(std::abs(end - blockedAt) <= cascade.position.tolerance);
    CHECK(std::abs(stateAt(m, 0, route_maker::sampleAt(m, g.page[1].t1)) - blockedAt) <= cascade.position.tolerance);
}

TEST_CASE("RMP1-E runner: the tap reads a flywheel's direction, and a piston at none of its positions as -1") {
    // BUG CAUGHT: a spinning flywheel recorded as stopped; a piston at no position recorded as 255 (the slot's own mark).
    mech_rig::Rig r{mech_fixture::robot_x::kRobotXMechanisms, [](shulib::hal::fake::FakeDeviceProvider& d) { d.refuseLine(3); }};
    const auto state = [&r](std::size_t i) { return route_maker::MechTap::stateOf(*r.svc.slotPtr(i)); };
    CHECK(state(2) == 0.0);
    r.svc.speed(2).spinUp(3000.0, shulib::units::Time{1.0});
    CHECK(state(2) == 1.0);
    r.svc.speed(2).stop();
    CHECK(state(2) == 0.0);
    CHECK(state(4) == -1.0);  // the hood: its line C refused at boot (ABSENT), so at none of its positions
}
