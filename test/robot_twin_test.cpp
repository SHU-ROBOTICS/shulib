// THE TWIN TEST (chunk ROB1-BT, 2026-11-16; the ROB1 brief's D2 (a), §1.3 G1-T, T7, HA-256). Say "host test
// plant", never "simulator". Nothing here has run on a robot.
//
// The team lead (2026-11-13): "IT IS LITERALLY THE SAME ROBOT BUILD JUST A MIRRORED ROUTINE". So robot two is
// robot one's twin, and this file fails the host suite the day the two robots differ anywhere but where they
// are allowed to — each robot's own FACT VALUES (ports, measured numbers: its facts file), its name and paths,
// and the one auton line that mirrors the skills route. What it tries to break:
//   * the mechanism regions — a level renamed, a motor added, `Hold` turned into `Coast` on ONE robot (GM17);
//   * the facts files — a fact missing, renamed, re-staged or out of order on one; a placeholder that is not
//     robot one's own number for that fact;
//   * the driver files' plumbing (each robot's buttons, feel and header map are its own: the team lead,
//     2026-11-20);
//   * the chassis tables, the words, and the autons lists (robot two's = robot one's, its skills route mirrored).
//   * the cascade's named LEVELS — a fact value, but held equal on both robots while the host suite runs the
//     shared route on one robot's names (the last case says why).

#include "doctest.h"

#include <cstddef>
#include <regex>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "common/robot_words.hpp"
#include "robots/chassis_table.hpp"
#include "robots/mechanism_table.hpp"
#include "source_pins.hpp"
#include "shulib/competition/override_mirror.hpp"
#include "team/autons.hpp"
#include "team/skills_route.hpp"
#include "team/robot_one_autons.hpp"
#include "team/robot_one_driver.hpp"
#include "team/robot_two_driver.hpp"

namespace sb = shulib::bench;
namespace one = sb::robot_one_facts;
namespace two = sb::robot_two_facts;

namespace {

/// The text between two whole marker lines, each present exactly once.
std::string between(const std::string& text, const std::string& begin, const std::string& end) {
    const std::size_t a = text.find(begin + "\n");
    const std::size_t b = text.find(end + "\n");
    REQUIRE_MESSAGE(a != std::string::npos, "missing ", begin);
    REQUIRE_MESSAGE(b != std::string::npos, "missing ", end);
    REQUIRE(text.find(begin + "\n", a + 1) == std::string::npos);
    REQUIRE(text.find(end + "\n", b + 1) == std::string::npos);
    REQUIRE(a < b);
    return text.substr(a + begin.size() + 1, b - a - begin.size() - 1);
}

/// `s` with every spelling of robot one's NAME made robot two's.
std::string asRobotTwo(std::string s) {
    const std::vector<std::pair<std::string, std::string>> names = {
        {"robot_one", "robot_two"}, {"RobotOne", "RobotTwo"}, {"robot one", "robot two"}};
    for (const auto& [from, to] : names) {
        for (std::size_t at = s.find(from); at != std::string::npos; at = s.find(from, at + to.size())) {
            s.replace(at, from.size(), to);
        }
    }
    return s;
}

/// The code of `text`: comments stripped, every run of whitespace one space.
std::string code(const std::string& text) { return source_pins::squash(source_pins::stripComments(text)); }

/// The `#define <name>(...)` macro of `text`, through its last continued line, squashed.
std::string macroText(const std::string& text, const std::string& name) {
    const std::size_t at = text.find("#define " + name + "(");
    REQUIRE_MESSAGE(at != std::string::npos, "missing macro ", name);
    std::size_t end = at;
    while (true) {
        const std::size_t eol = text.find('\n', end);
        REQUIRE(eol != std::string::npos);
        const bool continued = eol > 0 && text[eol - 1] == '\\';
        end = eol + 1;
        if (!continued) {
            break;
        }
    }
    std::string m = text.substr(at, end - at);
    for (std::size_t i = m.find("\\\n"); i != std::string::npos; i = m.find("\\\n", i)) {
        m.replace(i, 2, " ");
    }
    return source_pins::squash(m);
}

/// Robot `n`'s autons registered on a fresh registry (sb::kAutonPeriods).
template <class Register>
shulib::competition::AutonRegistry registered(Register reg) {
    shulib::competition::AutonRegistry r{sb::kAutonPeriods};
    reg(r);
    return r;
}

}  // namespace

TEST_CASE("twin (GM17): robot two's mechanism region is robot one's, token for token, but for the robot's name") {
    // BUG CAUGHT: one robot's cascade given a third motor, a renamed mechanism, `Hold` turned into `Coast`, a
    // literal number typed into one region — the robots stop being one build, and a route written for one does
    // something else on the other.
    const std::string table = source_pins::readFile("src/robots/mechanism_table.hpp");
    const std::string r1 = code(between(table, "// [robot one mechanisms]", "// [/robot one mechanisms]"));
    const std::string r2 = code(between(table, "// [robot two mechanisms]", "// [/robot two mechanisms]"));
    REQUIRE(r1.size() > 500);  // the region is really there (a cascade, two toggles, three pistons, the list)
    CHECK(asRobotTwo(r1) == r2);
    CHECK(r1.find("robot_two") == std::string::npos);  // robot one names nothing of robot two's (T7)
    CHECK(r2.find("robot_one") == std::string::npos);
    // ...and each region reads ITS OWN facts file through `facts::`.
    CHECK(source_pins::count(code(table), "namespace robot_one { using namespace mechanism::words; namespace facts = robot_one_facts;") == 1);
    CHECK(source_pins::count(code(table), "namespace robot_two { using namespace mechanism::words; namespace facts = robot_two_facts;") == 1);
}

TEST_CASE("twin: robot two's facts are robot one's facts -- the same names, stages and order; each placeholder robot one's own number") {
    // BUG CAUGHT: a fact added to one robot only, renamed, moved to another stage (a robot two that builds its
    // Drive on Stage A alone), or a placeholder pointing at ANOTHER robot-one fact (robot two's host runs on a
    // number robot one does not have there).
    const std::size_t n1 = sizeof one::kRobotOneFacts / sizeof one::kRobotOneFacts[0];
    const std::size_t n2 = sizeof two::kRobotTwoFacts / sizeof two::kRobotTwoFacts[0];
    REQUIRE(n1 == n2);
    for (std::size_t i = 0; i < n1; ++i) {
        CAPTURE(i);
        CHECK(std::string_view{one::kRobotOneFacts[i].name} == two::kRobotTwoFacts[i].name);
        CHECK(one::kRobotOneFacts[i].needed == two::kRobotTwoFacts[i].needed);
    }
    // In the text: every robot-two fact `kX = owed(one::kX.value(), ...` (robot one's value: the host runs robot two
    // as robot one) or, once typed, `typed(one::kX.placeholder, ...` (robot one's own, unchanging placeholder — so
    // typing robot one later never makes a typed robot-two value "still the placeholder"): robot one's SAME fact,
    // and the facts declared in robot one's order.
    const std::string t1 = code(source_pins::readFile("src/robots/robot_one_facts.hpp"));
    const std::string t2 = code(source_pins::readFile("src/robots/robot_two_facts.hpp"));
    const std::regex decl{R"(inline constexpr auto (k\w+) = (owed|typed)\()"};
    std::vector<std::string> ids1, ids2;
    for (auto it = std::sregex_iterator(t1.begin(), t1.end(), decl); it != std::sregex_iterator(); ++it) {
        ids1.push_back((*it)[1]);
    }
    for (auto it = std::sregex_iterator(t2.begin(), t2.end(), decl); it != std::sregex_iterator(); ++it) {
        ids2.push_back((*it)[1]);
        const std::string after = t2.substr(static_cast<std::size_t>(it->position() + it->length()));
        CAPTURE((*it)[1].str());
        const std::string kept = (*it)[2] == "owed" ? ".value()," : ".placeholder,";
        CHECK(after.rfind("one::" + (*it)[1].str() + kept, 0) == 0);
    }
    CHECK(ids1 == ids2);
    CHECK(ids2.size() == n2);
}

TEST_CASE("twin: every robot-two fact is OWED until the builder reads it off robot two (but its six chosen settings: Calibrate's two thresholds and the autons' four drive settings)") {
    // BUG CAUGHT: a robot-two fact typed by copying robot one's — a robot two that links on robot one's ports
    // without anyone having looked at robot two. The ONLY typed ones are Calibrate's two thresholds: chosen, not
    // measured, for the same 10-motor drive (the ROB1-BT merge, 2026-11-17). (The day the builder's robot-two
    // answers are TYPED — the coordinator's G1-F edit — this case is re-pinned in the same change.)
    CHECK(two::kRobotTwoFactCount == one::kRobotOneFactCount);
    CHECK_MESSAGE(two::kRobotTwoOwedCount == two::kRobotTwoFactCount - 6,
                  "a robot-two fact is TYPED: re-pin this case with it (its value and its source, read off robot two)");
    for (const auto& f : two::kRobotTwoFacts) {
        CAPTURE(f.name);
        const std::string_view n{f.name};
        const bool threshold = n == "Calibrate's pulse yaw floor, degrees" ||
                               n == "Calibrate's rest yaw rate, degrees per second" ||
                               n == "the autons' top drive speed, in/s" || n == "the autons' top turn rate, rad/s" ||
                               n == "the autons' heading correction gain" || n == "the autons' heading arrival, degrees";
        CHECK(f.typed == threshold);
    }
    CHECK(two::kCalibratePulseYawFloorDeg.value() == one::kCalibratePulseYawFloorDeg.value());
    CHECK(two::kCalibrateRestYawRateDegPerS.value() == one::kCalibrateRestYawRateDegPerS.value());
    // ...so robot two's Bench Tests and Calibrate wait on every Stage A fact, its other programs on every fact.
    CHECK(two::owedFor(sb::Program::BenchTests) == 17);
    CHECK(two::owedFor(sb::Program::Calibrate) == 17);
    CHECK(two::owedFor(sb::Program::Drive) == 31);
    CHECK(two::owedFor(sb::Program::Tune) == 31);
}

TEST_CASE("twin: the driver files' plumbing is robot one's, but for the robot's name -- each robot's buttons, feel and header map are its OWN") {
    // BUG CAUGHT: robot two's file growing its own plumbing (another include, a second loop, a class) where only
    // the team's three regions may differ, or a region's markers lost (the route builder writes between them).
    // The team lead, 2026-11-20 ("can you relax that rule"): the buttons, the feel and the map are the team's to
    // set on EACH robot; test/robot_two_driver_test.cpp checks robot two's against robot two's own declarations.
    const auto outside = [](std::string t, const std::string& id) {
        for (const std::string r : {"driver file", "feel", "control words"}) {
            const std::string b = "// [" + r + " " + id + "]\n";
            const std::string e = "// [/" + r + " " + id + "]\n";
            CAPTURE(b);
            REQUIRE(source_pins::count(t, b) == 1);
            REQUIRE(source_pins::count(t, e) == 1);
            const std::size_t i = t.find(b);
            const std::size_t j = t.find(e);
            REQUIRE(i < j);
            t.erase(i, j + e.size() - i);
        }
        return t;
    };
    const std::string f1 = outside(source_pins::readFile("src/team/robot_one_driver.hpp"), "one");
    const std::string f2 = outside(source_pins::readFile("src/team/robot_two_driver.hpp"), "two");
    CHECK(asRobotTwo(code(f1)) == code(f2));
    CHECK(source_pins::count(code(f2), "using RobotTwoDriver = DriverFile<robot_two::kControls, robot_two::kFeel, robot_two::kLoop>;") == 1);
}

TEST_CASE("twin: the chassis tables are robot one's design -- robot two's drive UNSET until its own Calibrate, never robot one's measured numbers") {
    // BUG CAUGHT: robot two's old measured drive (ports 11-20, signed, IMU 2) left in its table — a twin that
    // boots from ANOTHER robot's drive facts; or a drive port typed on one robot that Calibrate discovers.
    // Robot one's drive was MEASURED by its own Calibrate (2026-11-17, transcribed — no SD card); robot two's
    // waits for ITS run: every drive fact UNSET, never robot one's numbers (each robot is measured on its own).
    const sb::ChassisTable& a = sb::kRobotOneChassis;
    const sb::ChassisTable& b = sb::kRobotTwoChassis;
    CHECK(b.leftCount == 0);
    CHECK(b.rightCount == 0);
    CHECK(b.imuPort == 0);
    CHECK_FALSE(b.measured);
    CHECK_FALSE(b.signsMeasured);
    CHECK(b.wheelDiameterIn == 0.0);
    CHECK(b.trackWidthIn == 0.0);
    CHECK(b.externalRatio == 0.0);
    CHECK((a.leftCount == 0 || a.measured));  // robot one's: unset, or a measurement
    CHECK((b.cartridge == sb::Cartridge::Unset || b.cartridge == a.cartridge));
    CHECK(std::string_view{b.robot} == "2026 CASCADE ROBOT (robot two)");
    // Calibrate's thresholds: the twin's same 10-motor drive needs robot one's (a default 2.0 deg floor refuses it).
    CHECK(b.pulseYawFloorDeg == a.pulseYawFloorDeg);
    CHECK(b.restYawRateFloorDegPerS == a.restYawRateFloorDegPerS);
    CHECK(b.pulseYawFloorDeg > 0.0);
}

TEST_CASE("twin: the words -- robot two says robot one's words, but for its own paths, name and record name") {
    // BUG CAUGHT: robot two printing "claw" words for a cascade, or robot one's file paths ("fix
    // src/team/robot_one_autons.cpp" on robot two), or robot one's record name (robot two's boot accepting
    // robot one's card).
    const sb::RobotWords& w1 = sb::kRobotOneWords;
    const sb::RobotWords& w2 = sb::kRobotTwoWords;
    CHECK(w2.claw == w1.claw);
    CHECK(std::string_view{w2.rowWord} == w1.rowWord);
    CHECK(std::string_view{w2.lcdAbsent} == w1.lcdAbsent);
    CHECK(std::string_view{w2.declaredMotorLabel} == w1.declaredMotorLabel);
    CHECK(std::string_view{w2.driverFileWords} == "robot two's driver file");
    CHECK(std::string_view{w1.driverFileWords} == "robot one's driver file");
    CHECK(std::string_view{w2.driverFilePath} == "src/team/robot_two_driver.hpp");
    CHECK(std::string_view{w2.autonsFilePath} == "src/team/autons.cpp");
    CHECK(std::string_view{w2.recordName} == "robot-two");
    CHECK(std::string_view{w1.recordName} == "robot-one");
    CHECK(std::string_view{w2.routeFilePath} == w1.routeFilePath);  // the ONE route both share
    CHECK(std::string_view{w1.routeFilePath} == "src/team/skills_route.cpp");
}

TEST_CASE("twin: the autons -- robot two registers robot one's list, in robot one's order, each route marked mirrored MIRRORED (the one difference)") {
    // BUG CAUGHT: robot two's list drifting from robot one's (an auton on one robot only, another schedule);
    // robot two running a route marked mirrored UNmirrored (both robots in the same Quadrant), one NOT marked
    // mirrored mirrored, or robot one mirrored (RMP1-C D2: every route of src/team/skills_route.cpp's list).
    const auto r1 = registered(&sb::registerRobotOneAutons);
    const auto r2 = registered(&sb::registerAutons);
    // A new auton goes in BOTH robots' files (guide chapter 16, "Writing the next auton").
    REQUIRE_MESSAGE(r2.size() == r1.size(), "an auton in one robot's autons file only: add it to BOTH");
    REQUIRE(r1.size() >= 3);
    for (std::size_t i = 0; i < r1.size(); ++i) {
        CAPTURE(i);
        const auto& a = r1.at(i);
        const auto& b = r2.at(i);
        CHECK(std::string_view{a.name} == b.name);
        CHECK(a.kind == b.kind);
        CHECK(a.kind == shulib::competition::AutonKind::Skills);
        CHECK(a.endActionAt.value() == b.endActionAt.value());
        CHECK(a.hardStopAt.value() == b.hardStopAt.value());
        CHECK(a.mirror == nullptr);  // robot one runs everything as written
        bool team = false;
        bool marked = false;
        for (const sb::TeamRoute& route : sb::teamRoutes()) {
            const bool same = std::string_view{route.entry().name} == a.name;
            team = team || same;
            marked = marked || (same && route.mirror == sb::Mirror::ForRobotTwo);
        }
        CHECK((b.mirror != nullptr) == marked);  // robot two mirrors ONLY the routes marked so
        if (team) {
            CHECK(a.routine == b.routine);  // ONE route function, both robots
        }
        if (marked) {
            CHECK(b.mirror == &shulib::competition::override2026::kSkillsMirror);
        }
    }
    int skillsRoutes = 0;
    for (std::size_t i = 0; i < r1.size(); ++i) {
        skillsRoutes += std::string_view{r1.at(i).name} == "skills" ? 1 : 0;
    }
    CHECK(skillsRoutes == 1);
}

TEST_CASE("twin: robot two's check is robot one's check, for robot two -- and every robot-two fact has EXACTLY ONE right below it, quoting its name") {
    // BUG CAUGHT: robot two's OWED refusal keyed on ROBOT ONE's id (robot two's builds would link on robot one's
    // placeholders, robot one's would refuse on robot two's facts); one of its four checks dropped or weakened; a
    // robot-two fact whose check was deleted (its build would link while it is owed — only the src gate's
    // facts proof would see it); a check quoting another fact's name.
    const std::string t1 = source_pins::readFile("src/robots/robot_one_facts.hpp");
    const std::string t2 = source_pins::readFile("src/robots/robot_two_facts.hpp");
    std::string expected = asRobotTwo(macroText(t1, "ROBOT_ONE_CHECK"));
    for (const char* shared : {"same", "needs", "contains"}) {  // the machinery robot two names from robot one's
        const std::string from = std::string{"robot_two_facts::"} + shared + "(";
        const std::string to = std::string{"robot_one_facts::"} + shared + "(";
        for (std::size_t at = expected.find(from); at != std::string::npos; at = expected.find(from, at + to.size())) {
            expected.replace(at, from.size(), to);
        }
    }
    {
        const std::string from = "ROBOT_ONE_CHECK";
        for (std::size_t at = expected.find(from); at != std::string::npos; at = expected.find(from, at + 1)) {
            expected.replace(at, from.size(), "ROBOT_TWO_CHECK");
        }
    }
    CHECK(macroText(t2, "ROBOT_TWO_CHECK") == expected);
    // ...and below every fact, its own check.
    const std::string c2 = code(t2);
    const std::regex decl{R"(inline constexpr auto (k\w+) = (owed|typed)\()"};
    const std::regex nameOf{R"re(Stage::\w+, ("(?:[^"\\]|\\.)*")\s*[,)])re"};
    int facts = 0;
    for (auto it = std::sregex_iterator(c2.begin(), c2.end(), decl); it != std::sregex_iterator(); ++it) {
        ++facts;
        const std::string var = (*it)[1];
        CAPTURE(var);
        const std::size_t from = static_cast<std::size_t>(it->position());
        const std::size_t semi = c2.find(';', from);
        std::smatch name;
        const std::string body = c2.substr(from, semi - from + 1);
        REQUIRE(std::regex_search(body, name, nameOf));
        // The very next statement is ROBOT_TWO_CHECK(var, "<the same name>");
        const std::string next = c2.substr(semi + 1, var.size() + name[1].str().size() + 24);
        CHECK(next.rfind(" ROBOT_TWO_CHECK(" + var + ", " + name[1].str() + ");", 0) == 0);
    }
    CHECK(facts == two::kRobotTwoFactCount);
    CHECK(source_pins::count(c2, "ROBOT_TWO_CHECK(k") == static_cast<std::size_t>(facts));
}

TEST_CASE("twin: robot two's facts HELPERS are robot one's, line for line (the stall signatures, the stops, the levels, the left toggle's port, the counts)") {
    // BUG CAUGHT: a helper fixed on one robot only (robot one's homing signature given its own rpm, say) — both
    // regions still read `facts::cascadeHomingStall()`, so the region test passes, but the twins home differently.
    const std::string t1 = code(source_pins::readFile("src/robots/robot_one_facts.hpp"));
    std::string t2 = code(source_pins::readFile("src/robots/robot_two_facts.hpp"));
    for (std::size_t at = t2.find("one::kCascadeRowCount"); at != std::string::npos; at = t2.find("one::kCascadeRowCount", at)) {
        t2.replace(at, 5, "");  // robot two reads robot one's row count (the rows' array size is the machinery's)
    }
    for (const char* sig : {"constexpr int owedCount()", "constexpr int owedFor(Program p)",
                            "constexpr units::Length cascadeBottom()", "constexpr units::Length cascadeTop()",
                            "constexpr mechanism::Stall cascadeStall()", "constexpr mechanism::Stall cascadeHomingStall()",
                            "constexpr const Level& lv(std::size_t i)", "constexpr int leftTogglePort()"}) {
        CAPTURE(sig);
        CHECK(asRobotTwo(source_pins::bodyOf(t1, sig)) == source_pins::bodyOf(t2, sig));
    }
}

TEST_CASE("twin: the cascade's NAMED levels are the same on both robots -- the host suite runs the shared route on robot two's declarations") {
    // BUG CAUGHT (ROB1-BT's review round): robot two's levels typed to other heights than robot one's. The host
    // suite and the route preview are one bench build, so the shared skills route's `cascade` is robot TWO's
    // there; a named step (`cascade[facts::lv(i).name]`) finds robot one's slot only when robot one has that
    // preset AT THAT VALUE (MechanismService::find). Different levels would make robot one's host runs say
    // "the step did nothing" while its robot build is right. If the builder reads different levels off robot
    // two, this test is the place to decide: the host then compiles the shared route once per robot.
    const shulib::mechanism::MechanismSpec a = sb::robot_one::cascade.spec();
    const shulib::mechanism::MechanismSpec b = sb::robot_two::cascade.spec();
    REQUIRE(a.position.presetCount == b.position.presetCount);
    for (std::size_t i = 0; i < a.position.presetCount; ++i) {
        CAPTURE(i);
        CHECK(std::string_view{a.position.presets[i].name} == std::string_view{b.position.presets[i].name});
        CHECK(a.position.presets[i].value == b.position.presets[i].value);
    }
}
