// dropped_autons_test.cpp — the boot's words for an auton the registry DROPPED
// (src/common/dropped_autons.hpp; chunk T4 Part 3, the team lead's ruling of 2026-10-02 on COMP-b's
// C5: "drop just that auton"). Both programs that register the team's autons — the competition
// program and the Auton Tester — say these lines at boot; their source pins
// (test/competition_source_test.cpp, test/auton_tester_source_test.cpp) prove each calls this, and
// these cases prove what it says. Every case names the bug it would catch.

#include "doctest.h"

#include <string>
#include <vector>

#include "common/dropped_autons.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/ui/touch_panel.hpp"
#include "shulib/units/quantity.hpp"

using shulib::chassis::RoutineResult;
using shulib::competition::AutonContext;
using shulib::competition::AutonEntry;
using shulib::competition::AutonKind;
using shulib::competition::AutonPeriods;
using shulib::competition::AutonRegistry;
using shulib::competition::kMaxAutonRefusals;
using shulib::competition::OnRefusal;
using shulib::competition::StartPose;
using shulib::units::Time;

namespace {

RoutineResult noop(AutonContext& /*ctx*/) { return RoutineResult{}; }

AutonEntry entry(const char* name, double hardStop) {
    AutonEntry e{};
    e.name = name;
    e.kind = AutonKind::Match;
    e.routine = &noop;
    e.start = StartPose::relative();
    e.endActionAt = Time{25.0};
    e.hardStopAt = Time{hardStop};
    return e;
}

AutonRegistry dropping() {
    return AutonRegistry{AutonPeriods{.match = Time{30.0}, .skills = Time{60.0}},
                         OnRefusal::DropEntry};
}

std::vector<std::string> said(const AutonRegistry& reg, bool driverControlRuns = true) {
    std::vector<std::string> out;
    shulib::bench::sayDroppedAutons(reg, driverControlRuns,
                                    [&out](const char* line) { out.emplace_back(line); });
    return out;
}

}  // namespace

// (DA1) Bug caught: a boot that says nothing when an auton was dropped (the student sees theirs
// missing from the menu and the log is silent), says it without the auton's NAME, says the drop
// cost something it did not (driver control, the other autons), or speaks when nothing was
// dropped. Each dropped entry is ONE line, the registry's own sentence after `DROPPED: `. The
// summary says the DROPPED ones are off the menu — never "none is on the menu", which a pit crew
// read as an empty menu (the T4-3 review; re-pinned 2026-10-08). Since ROB1-BT every robot's line is ONE
// format, its autons file's path an argument from its words (robot one's shorter wording, which fits 200
// bytes with the longer path): "N auton(s) ... DROPPED -- NOT on the menu".
TEST_CASE("dropped autons (DA1): one summary line, then each dropped entry by its own sentence") {
    AutonRegistry none = dropping();
    none.add(entry("left rush", 29.0));
    CHECK(said(none).empty());

    AutonRegistry reg = dropping();
    reg.add(entry("too late", 31.0));
    reg.add(entry("left rush", 29.0));
    reg.add(entry("also late", 45.0));
    const std::vector<std::string> lines = said(reg);
    REQUIRE(lines.size() == 3);
    CHECK(lines[0]
          == "2 auton(s) REFUSED at registration and DROPPED -- NOT on the menu (each named below); every "
             "other auton and driver control still run -- fix src/team/autons.cpp");
    CHECK(lines[1] == "DROPPED: " + std::string{reg.refusal(0)});
    CHECK(lines[1].rfind("DROPPED: auton 'too late': hardStopAt 31.00 s", 0) == 0);
    CHECK(lines[2] == "DROPPED: " + std::string{reg.refusal(1)});
    CHECK(lines[2].rfind("DROPPED: auton 'also late': ", 0) == 0);
}

// (DA2) Bug caught: a line TermSink cuts (200 bytes) or the screen-safety rules break (a '%' PROS
// would re-read as a format), at the worst case — the longest sentence the registry can keep — and
// the refusals past the kept sentences silently uncounted.
TEST_CASE("dropped autons (DA2): every line within 200 bytes and ASCII; the rest past the cap counted") {
    AutonRegistry reg = dropping();
    for (std::size_t i = 0; i < kMaxAutonRefusals + 3; ++i) {
        reg.add(entry("aaaaaaaaaaaaaaaaaaaaaaaaaaaaa", 99.0));  // 29 characters: the longest name
    }
    const std::vector<std::string> lines = said(reg);
    REQUIRE(lines.size() == 1 + kMaxAutonRefusals + 1);
    CHECK(lines[0].rfind("11 auton(s) REFUSED", 0) == 0);
    // Past the cap the first line never claims EACH is named (the T4-3 verification): eight are,
    // the rest only counted.
    CHECK(lines[0]
          == "11 auton(s) REFUSED at registration and DROPPED -- NOT on the menu (the first 8 named below); "
             "every other auton and driver control still run -- fix src/team/autons.cpp");
    CHECK(lines.back() == "DROPPED: 3 more, whose sentences were not kept (the registry keeps 8)");
    for (const std::string& l : lines) {
        CAPTURE(l);
        CHECK(l.size() < shulib::bench::kDroppedAutonLineBytes);
        CHECK(shulib::ui::screenText(l, shulib::bench::kDroppedAutonLineBytes));
    }
    // A registry that throws (the default) never drops: nothing to say.
    AutonRegistry strict{AutonPeriods{.match = Time{30.0}, .skills = Time{60.0}}};
    CHECK(said(strict).empty());
}

// (DA3) Bug caught: the count past the kept sentences off by one at its boundary — exactly as many
// dropped as the registry keeps (no "more" line: every one was named), and ONE more (its line says
// 1, never silent). DA2's three past the cap could not tell `>` from `> kept + 1` (the T4 late
// step's LDA2 survived it). The first line's "(each named below)" switches at the same boundary
// (the T4-3 verification).
TEST_CASE("dropped autons (DA3): the 'more' line at the cap's boundary -- none at the cap, one past it") {
    AutonRegistry atCap = dropping();
    for (std::size_t i = 0; i < kMaxAutonRefusals; ++i) {
        atCap.add(entry("late", 31.0));
    }
    std::vector<std::string> lines = said(atCap);
    REQUIRE(lines.size() == 1 + kMaxAutonRefusals);
    CHECK(lines.back().rfind("DROPPED: auton 'late': ", 0) == 0);
    CHECK(lines.front().find("(each named below)") != std::string::npos);  // every one IS named
    AutonRegistry onePast = dropping();
    for (std::size_t i = 0; i < kMaxAutonRefusals + 1; ++i) {
        onePast.add(entry("late", 31.0));
    }
    lines = said(onePast);
    REQUIRE(lines.size() == 1 + kMaxAutonRefusals + 1);
    CHECK(lines.front().rfind("9 auton(s) REFUSED", 0) == 0);
    CHECK(lines.front().find("(the first 8 named below)") != std::string::npos);  // one is not
    CHECK(lines.front().find("each named") == std::string::npos);
    CHECK(lines.back() == "DROPPED: 1 more, whose sentences were not kept (the registry keeps 8)");
}

// (DA4) Bug caught: the Auton Tester's boot claiming that driver control still runs — its
// opcontrol() does nothing by design (the T4-3 review) — or the competition program's losing the
// one clause the team lead's ruling is about. The flag changes that clause and nothing else.
TEST_CASE("dropped autons (DA4): driver control is named only where it runs") {
    AutonRegistry reg = dropping();
    reg.add(entry("too late", 31.0));
    const std::vector<std::string> tester = said(reg, false);
    const std::vector<std::string> competition = said(reg, true);
    REQUIRE(tester.size() == 2);
    REQUIRE(competition.size() == 2);
    CHECK(tester[0] == "1 auton(s) REFUSED at registration and DROPPED -- NOT on the menu (each named below); "
                       "every other auton still runs -- fix src/team/autons.cpp");
    CHECK(tester[0].find("driver control") == std::string::npos);
    CHECK(competition[0].find("every other auton and driver control still run") != std::string::npos);
    CHECK(tester[1] == competition[1]);  // the entry's own line is the same in both programs
}
