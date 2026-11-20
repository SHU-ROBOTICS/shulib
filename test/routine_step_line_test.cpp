// RMP1-K D4 (API 2.11) — Routine logs ONE Info [RTN] line as each step SUCCEEDS, so the route builder can tick a
// run's steps live: `routine 'x': step N (<verb or mechanism>) done in S s[, at <end>][ (<OUTCOME>)]`, "started"
// for a start form still running. A failed, stopped or skipped step keeps its own line and gets none of these.

#include "doctest.h"

#include <filesystem>
#include <fstream>
#include <regex>
#include <string>
#include <vector>

#include "mechanism_auton_rig.hpp"
#include "mechanism_fixtures.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/match_runner.hpp"

namespace {
using namespace mech_fixture::robot_x;

RoutineResult every(AutonContext& ctx) {
    Routine r{ctx, "lines"};
    r.pause(0.5_s);
    r.move(lift.to(18_in), 4_s);
    r.set(hood["up"]);
    r.startMove(arm["score"], 1.5_s);
    r.waitUntilDone(arm, 1.5_s);
    r.run(intake, 12_volt, 0.3_s);
    r.then([] { return true; }, "act");
    return r.result();
}

RoutineResult failing(AutonContext& ctx) {
    Routine r{ctx, "fails"};
    r.onFail(carryOn).then([] { return false; }, "bad");
    r.then([] { return true; }, "ok");
    r.then([] { return false; }, "stop");
    r.then([] { return true; }, "after");
    return r.result();
}

RoutineResult skips(AutonContext& ctx) {
    Routine r{ctx, "skips"};
    r.onFail(skipTo("L")).waitFor([] { return false; }, 0.4_s, "never");  // FAILS after 0.4 s, then skips to L
    r.pause(0.2_s);
    r.label("L").pause(0.3_s);
    return r.result();
}

RoutineResult stops(AutonContext& ctx) {
    Routine r{ctx, "stops"};
    r.run(intake, 12_volt, 0.2_s);
    r.stop(intake);
    return r.result();
}

/// The two members the Routine's context constructor reads (robot two's claw needs no runAuton).
struct Ctx {
    shulib::chassis::Chassis& chassis;
    shulib::mechanism::MechanismService* mechanismService;
};

/// The "done in" / "started in" lines of `rtn` (a rig's RTN lines), in order.
std::vector<std::string> doneLines(const std::vector<std::string>& rtn) {
    std::vector<std::string> out;
    for (const std::string& l : rtn) {
        if (l.find(") done in ") != std::string::npos || l.find(") started in ") != std::string::npos) {
            out.push_back(l);
        }
    }
    return out;
}

double seconds(const std::string& l) {
    const std::size_t at = l.find(" in ") + 4;
    return std::stod(l.substr(at));
}
}  // namespace

TEST_CASE("RMP1-K D4: one [RTN] line per SUCCEEDED step -- its number, verb, seconds from its door, a mechanism's end") {
    // Bug caught: a step that ends well says nothing (the page cannot tick it live), a wrong number, cumulative seconds,
    // a mechanism's end missing or wrong, or a start form called done.
    auton_rig::RobotX x;
    const auto res = shulib::competition::runAuton(x.chassis, x.guard, auton_rig::entryFor("lines", &every), nullptr, x.sink,
                                                   x.svc, auton_rig::kPeriod);
    REQUIRE(res.routine.ok);
    REQUIRE(res.routine.completed == 7);
    const std::vector<std::string> d = doneLines(x.lines("RTN"));
    REQUIRE(d.size() == 7);
    const std::string n = R"(\d+\.\d\d)";
    CHECK(std::regex_match(d[0], std::regex("routine 'lines': step 1 \\(pause\\) done in " + n + " s")));
    CHECK(seconds(d[0]) == doctest::Approx(0.5).epsilon(0.05));
    std::smatch m;
    REQUIRE(std::regex_match(d[1], m, std::regex("routine 'lines': step 2 \\(lift\\) done in " + n + " s, at (\\d+\\.\\d) in")));
    CHECK(std::stod(m[1]) == doctest::Approx(18.0).epsilon(0.03));
    CHECK(std::regex_match(d[2], std::regex("routine 'lines': step 3 \\(hood\\) done in " + n + " s, at 'up'")));
    CHECK(seconds(d[2]) < 1.0);  // from ITS door, not the run's start (2 s and more by now)
    CHECK(seconds(d[1]) > 0.1);
    CHECK(d[3] == "routine 'lines': step 4 (arm) started in 0.00 s");
    REQUIRE(std::regex_match(d[4], m, std::regex("routine 'lines': step 5 \\(arm\\) done in " + n + " s, at (\\d+\\.\\d) deg")));
    CHECK(std::stod(m[1]) == doctest::Approx(95.0).epsilon(0.03));
    CHECK(std::regex_match(d[5], std::regex("routine 'lines': step 6 \\(intake\\) done in " + n + " s")));
    CHECK(seconds(d[5]) == doctest::Approx(0.3).epsilon(0.15));
    CHECK(d[6] == "routine 'lines': step 7 (act) done in 0.00 s");
    for (int i = 0; i < x.sink.size(); ++i) {
        if (x.sink.at(i).message.find(") done in ") != std::string::npos) {
            CHECK(x.sink.at(i).level == shulib::hal::LogLevel::Info);
        }
    }
}

TEST_CASE("RMP1-K D4: a failed, stopped or skipped step logs no done line -- its own line stays as it was") {
    // Bug caught: a done line for a step that failed (the page would tick it green), or the old lines changed.
    auton_rig::RobotX x;
    const auto res = shulib::competition::runAuton(x.chassis, x.guard, auton_rig::entryFor("fails", &failing), nullptr, x.sink,
                                                   x.svc, auton_rig::kPeriod);
    CHECK(res.routine.stoppedAt == 3);
    const std::vector<std::string> d = doneLines(x.lines("RTN"));
    REQUIRE(d.size() == 1);
    CHECK(d[0] == "routine 'fails': step 2 (ok) done in 0.00 s");
    const std::vector<std::string> all = x.lines("RTN");
    const auto has = [&all](const std::string& s) {
        for (const std::string& l : all) {
            if (l == s) {
                return true;
            }
        }
        return false;
    };
    CHECK(has("routine 'fails': step 1 (bad) FAILED: action FAILED — carrying on; drive safed"));
    CHECK(has("routine 'fails' STOPPED at step 3 (stop): action FAILED — skipping the rest; drive safed"));
    CHECK(has("routine 'fails': step 4 (after) skipped — stopped at step 3"));
}

TEST_CASE("RMP1-K D4: a step counted done UNCONFIRMED says so; the route builder reads these very lines") {
    // Bug caught: the outcome bracket dropped (a sensorless grab reads as confirmed), or the page's reader and the library
    // drifting apart -- the lines below are the shared fixture tools/route_maker/test/fixtures/robot_log/d4_lines.log, which
    // run_log.test.mjs reads too.
    auton_rig::RobotTwo two;
    Ctx ctx{two.chassis, &two.svc};
    {
        using namespace mech_fixture::robot_two;
        Routine r{ctx, "grab"};
        r.set(claw["open"]);
        r.set(claw["closed"]);  // no piece sensor: UNCONFIRMED, counted done
        REQUIRE(r.ok());
    }
    const std::vector<std::string> d = doneLines(two.lines("RTN"));
    std::vector<std::string> shared;
    std::ifstream in{std::filesystem::path(SHULIB_SOURCE_DIR) / "tools/route_maker/test/fixtures/robot_log/d4_lines.log"};
    for (std::string l; std::getline(in, l);) {
        const std::size_t at = l.find("[RTN] ");
        if (at != std::string::npos) {
            shared.push_back(l.substr(at + 6));
        }
    }
    REQUIRE(d.size() == 2);
    INFO(d[0]);
    INFO(d[1]);
    REQUIRE(shared.size() == 2);
    CHECK(d == shared);
    const std::string unconfirmed = " s, at 'closed' (UNCONFIRMED)";
    CHECK(d[1].rfind("routine 'grab': step 2 (claw) done in ", 0) == 0);
    CHECK(d[1].size() > unconfirmed.size());
    CHECK(d[1].compare(d[1].size() - unconfirmed.size(), unconfirmed.size(), unconfirmed) == 0);
}

TEST_CASE("RMP1-K D4: a skip's landing step counts its seconds from its own door; a stop step names its mechanism") {
    // Bug caught: the landing step's "done in" counting from the FAILED step's door (the page would show 0.7 s for a
    // 0.3 s pause), or a stop() step logged under another name than its mechanism's.
    {
        auton_rig::RobotX x;
        const auto res = shulib::competition::runAuton(x.chassis, x.guard, auton_rig::entryFor("skips", &skips), nullptr,
                                                       x.sink, x.svc, auton_rig::kPeriod);
        CHECK(res.routine.failedOn == 1);
        const std::vector<std::string> d = doneLines(x.lines("RTN"));
        REQUIRE(d.size() == 1);
        CHECK(d[0].rfind("routine 'skips': step 3 (pause) done in ", 0) == 0);
        CHECK(seconds(d[0]) == doctest::Approx(0.3).epsilon(0.1));
    }
    {
        auton_rig::RobotX x;
        const auto res = shulib::competition::runAuton(x.chassis, x.guard, auton_rig::entryFor("stops", &stops), nullptr,
                                                       x.sink, x.svc, auton_rig::kPeriod);
        REQUIRE(res.routine.ok);
        const std::vector<std::string> d = doneLines(x.lines("RTN"));
        REQUIRE(d.size() == 2);
        CHECK(d[1] == "routine 'stops': step 2 (intake) done in 0.00 s");
    }
}
