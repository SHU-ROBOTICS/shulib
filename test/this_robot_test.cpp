// THE BUILD'S TEAM FILES AND WORDS (chunk ROB1-A, 2026-11-11; the ROB1 brief's §1.1 this_robot.hpp, tests T1,
// T7, T10's words half). Say "host test plant", never "simulator". Nothing here has run on a robot.
//
// WHAT THIS FILE TRIES TO BREAK
//   * EVERY ROBOT IS EVERY SELECTION'S CASE (T1, GM1, GM3): the driver selection and the autons dispatcher
//     give each robot ITS OWN — robot one's driver file and registerRobotOneAutons() on robot one, robot
//     two's on robot two and (today's bytes) on the bench bot and the X-drive — by identity, never a count;
//     robot one's beacons are not prefixes of robot two's.
//   * THE WORDS (T10's words, R7, GM21): robot two's words are its old bytes verbatim; robot one's say
//     "mechanism" / "MECH" / its own paths, never "claw" or robot two's files; the controller's absent word
//     and the dropped-autons line are each robot's, within their budgets.
//   * THE PROGRAMS CHOOSE NO ROBOT (§7's greps, T7): no program or common file names RobotTwoDriver,
//     RobotOneDriver, registerAutons( or registerRobotOneAutons( except src/common/this_robot.hpp; robot one's
//     team files name nothing of robot two's and robot two's nothing of robot one's.

#include "doctest.h"

#include <cstddef>
#include <regex>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include "build_selection.hpp"
#include "common/driver_panel.hpp"
#include "common/dropped_autons.hpp"
#include "common/robot_words.hpp"
#include "common/this_robot.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "source_pins.hpp"

namespace sb = shulib::bench;
using sb::RobotId;
using sb::WordsOf;

namespace {

constexpr RobotId kEveryRobot[] = {RobotId::BenchBot, RobotId::RobotTwo, RobotId::InventedXDrive, RobotId::RobotOne};

shulib::chassis::RoutineResult noop(shulib::competition::AutonContext& /*ctx*/) {
    return shulib::chassis::RoutineResult{};
}

}  // namespace

// ═══ T1 — the selections ══════════════════════════════════════════════════════════════

static_assert(std::is_same_v<sb::DriverOf<RobotId::RobotOne>::type, sb::RobotOneDriver>);
static_assert(std::is_same_v<sb::DriverOf<RobotId::RobotTwo>::type, sb::RobotTwoDriver>);
static_assert(std::is_same_v<sb::DriverOf<RobotId::BenchBot>::type, sb::RobotTwoDriver>);
static_assert(std::is_same_v<sb::DriverOf<RobotId::InventedXDrive>::type, sb::RobotTwoDriver>);
static_assert(!std::is_same_v<sb::RobotOneDriver, sb::RobotTwoDriver>);
static_assert(std::is_same_v<sb::ThisRobotDriver, sb::RobotTwoDriver>, "the host is the bench build");

TEST_CASE("this robot (T1): the autons dispatcher gives every robot ITS OWN registration, by identity") {
    // BUG CAUGHT (GM3): the dispatcher's RobotOne case calling robot two's registerAutons() — robot one's
    // Auton Tester would offer robot two's autons; or robot two handed robot one's.
    CHECK(sb::autonsFor(RobotId::RobotOne) == &sb::registerRobotOneAutons);
    CHECK(sb::autonsFor(RobotId::RobotTwo) == &sb::registerAutons);
    CHECK(sb::autonsFor(RobotId::BenchBot) == &sb::registerAutons);
    CHECK(sb::autonsFor(RobotId::InventedXDrive) == &sb::registerAutons);
    CHECK(sb::kThisRobotsAutons == &sb::registerAutons);  // the host is the bench build
    // ...and each really registers its robot's list: since ROB1-BT both are SKILLS lists, and only robot two's
    // runs the shared skills route MIRRORED (the twins' one difference — test/robot_twin_test.cpp).
    shulib::competition::AutonRegistry one{sb::kAutonPeriods};
    sb::autonsFor(RobotId::RobotOne)(one);
    REQUIRE(one.size() == 3);
    CHECK(one.at(0).kind == shulib::competition::AutonKind::Skills);
    CHECK(one.at(2).mirror == nullptr);
    shulib::competition::AutonRegistry two{sb::kAutonPeriods};
    sb::registerThisRobotsAutons(two);
    REQUIRE(two.size() == 3);
    CHECK(two.at(0).kind == shulib::competition::AutonKind::Skills);
    CHECK(two.at(2).mirror != nullptr);
}

TEST_CASE("this robot (T1): robot one's beacons are its own -- never a prefix of robot two's, nor robot two's of robot one's") {
    // BUG CAUGHT: a beacon the gate's NUL-terminated search could find inside another build's.
    const std::string_view one = sb::robotBeacon(RobotId::RobotOne);
    const std::string_view two = sb::robotBeacon(RobotId::RobotTwo);
    CHECK(one == "shulib-robot=robot-one");
    CHECK(one != two);
    CHECK(two.find(one) == std::string_view::npos);
    CHECK(one.find(two) == std::string_view::npos);
    // The VARIANT beacons, read out of src/build_selection.hpp's own chain (the review of ROB1-A: this case
    // used to compare strings typed in the test, so a changed beacon in the header left it green — the host
    // build only ever sees the bench branch's kVariantBeacon). Every branch's literal, robot one's six by
    // their `one` prefix: none is a prefix of another build's, robot two's included, either way.
    const std::string sel = source_pins::stripComments(source_pins::readFile("src/build_selection.hpp"));
    const std::string decl = "inline constexpr char kVariantBeacon[] = \"";
    std::vector<std::string> beacons;
    for (std::size_t at = sel.find(decl); at != std::string::npos; at = sel.find(decl, at + 1)) {
        const std::size_t open = at + decl.size();
        beacons.push_back(sel.substr(open, sel.find('"', open) - open));
    }
    REQUIRE(beacons.size() == 14);  // the src build gate's fourteen builds, one branch each (Tune's two since
                                    // ROB1-A's integration onto TUNE1-B1)
    int robotOne = 0;
    for (const std::string& mine : beacons) {
        CHECK(mine.rfind("shulib-robot-variant=", 0) == 0);
        const bool isOne = mine.rfind("shulib-robot-variant=one", 0) == 0;
        robotOne += isOne ? 1 : 0;
        for (const std::string& theirs : beacons) {
            if (&theirs == &mine) {
                continue;
            }
            CAPTURE(mine);
            CAPTURE(theirs);
            CHECK(mine != theirs);
            // Robot one's: never a prefix of another build's; and no other build's a prefix of robot one's,
            // except robot one's own Bench Tests `…=one` inside its other five (the gate matches each
            // beacon NUL-terminated, so a prefix is harmless there — and robot one's six are robot one's).
            if (isOne) {
                const bool theirsIsOne = theirs.rfind("shulib-robot-variant=one", 0) == 0;
                CHECK((theirs.rfind(mine, 0) != 0 || (mine == "shulib-robot-variant=one" && theirsIsOne)));
                CHECK((mine.rfind(theirs, 0) != 0 || theirs == "shulib-robot-variant=one"));
            }
        }
    }
    CHECK(robotOne == 6);  // Bench Tests, Drive, Calibrate, the Auton Tester, Competition, Tune
}

// ═══ the words ════════════════════════════════════════════════════════════════════════

TEST_CASE("this robot (R7): the CLAW words are robot two's OLD BYTES (the bench's, the host's default); both cascade robots say 'mechanism' and their own files") {
    // BUG CAUGHT (GM14, GM15): the claw words re-worded (the host's claw tests and the bench builds carry them
    // byte for byte); a cascade robot handed the claw words (GM21) — "claw" on a robot with a cascade, or
    // another robot's driver path printed by its Drive (ROB1-BT: robot two is a cascade now).
    const sb::RobotWords& claws = sb::kClawWords;
    CHECK(claws.claw);
    CHECK(std::string_view{claws.rowWord} == "CLAW");
    CHECK(std::string_view{claws.lcdAbsent} == sb::kLcdClawAbsent);
    CHECK(std::string_view{claws.declaredMotorLabel} == sb::kDeclaredMotorLabel);
    CHECK(claws.recordName == nullptr);
    const sb::RobotWords& two = sb::wordsFor(RobotId::RobotTwo);
    CHECK_FALSE(two.claw);
    CHECK(std::string_view{two.rowWord} == "MECH");
    CHECK(std::string_view{two.lcdAbsent} == "MECH ABSENT");
    CHECK(std::string_view{two.driverFilePath} == "src/team/robot_two_driver.hpp");
    CHECK(std::string_view{two.autonsFilePath} == "src/team/autons.cpp");
    const sb::RobotWords& one = sb::wordsFor(RobotId::RobotOne);
    CHECK_FALSE(one.claw);
    CHECK(std::string_view{one.rowWord} == "MECH");
    CHECK(std::string_view{one.lcdAbsent} == "MECH ABSENT");
    CHECK(std::string_view{one.driverFilePath} == "src/team/robot_one_driver.hpp");
    CHECK(std::string_view{one.autonsFilePath} == "src/team/robot_one_autons.cpp");
    for (const char* w : {one.driverFileWords, one.driverFilePath, one.autonsFilePath, one.rowWord, one.lcdAbsent,
                          one.declaredMotorLabel}) {
        CAPTURE(w);
        const std::string s{w};
        CHECK(s.find("claw") == std::string::npos);
        CHECK(s.find("CLAW") == std::string::npos);
        CHECK(s.find("two") == std::string::npos);
    }
    // The bench bot and the X-drive keep the claw words, robot two's old bytes (landmine 10; they print none).
    CHECK(&sb::wordsFor(RobotId::BenchBot) == &sb::kClawWords);
    CHECK(&sb::wordsFor(RobotId::InventedXDrive) == &sb::kClawWords);
    CHECK(&sb::ThisRobotWords::value == &sb::kClawWords);
    // Every robot's words fit their budgets.
    for (const RobotId r : kEveryRobot) {
        CAPTURE(static_cast<int>(r));
        CHECK(sb::wordLength(sb::wordsFor(r).rowWord) == 4);
        CHECK(sb::wordLength(sb::wordsFor(r).lcdAbsent) <= sb::kControllerCols);
    }
}

TEST_CASE("this robot (R7): the controller's absent word is the robot's -- 'CLAW ABSENT' by default (the claw words), 'MECH ABSENT' on both cascade robots") {
    // BUG CAUGHT (GM21b's LCD half): robot one's controller saying CLAW ABSENT for an unplugged cascade motor.
    sb::ControllerFacts f{};
    f.connected = true;
    f.clawAbsent = true;
    CHECK(std::string_view{sb::controllerWord(f)} == "CLAW ABSENT");
    CHECK(std::string_view{sb::controllerWord(f, WordsOf<RobotId::RobotTwo>{})} == "MECH ABSENT");
    CHECK(std::string_view{sb::controllerWord(f, WordsOf<RobotId::RobotOne>{})} == "MECH ABSENT");
    CHECK(std::string_view{sb::controllerScreenLines(f, WordsOf<RobotId::RobotOne>{}).line[2]} == "MECH ABSENT");
    CHECK(std::string_view{sb::controllerScreenLines(f).line[2]} == "CLAW ABSENT");
    f.clawAbsent = false;
    CHECK(std::string_view{sb::controllerWord(f, WordsOf<RobotId::RobotOne>{})} == "ok");
}

TEST_CASE("this robot (R7): the dropped-autons line names the ROBOT'S autons file, within 200 bytes, for both robots") {
    // BUG CAUGHT: robot one's boot telling a student to "fix src/team/autons.cpp" — robot two's file; a robot
    // one line longer than TermSink's 200 bytes (cut on the brain).
    shulib::competition::AutonRegistry reg{
        shulib::competition::AutonPeriods{.match = shulib::units::Time{30.0}, .skills = shulib::units::Time{60.0}},
        shulib::competition::OnRefusal::DropEntry};
    shulib::competition::AutonEntry e{};
    e.name = "far too late";
    e.kind = shulib::competition::AutonKind::Match;
    e.routine = &noop;
    e.start = shulib::competition::StartPose::relative();
    e.endActionAt = shulib::units::Time{25.0};
    e.hardStopAt = shulib::units::Time{99.0};
    reg.add(e);
    REQUIRE(reg.refusedCount() == 1);
    for (const bool runs : {true, false}) {
        std::vector<std::string> one;
        sb::sayDroppedAutons(reg, runs, [&one](const char* l) { one.emplace_back(l); }, WordsOf<RobotId::RobotOne>{});
        std::vector<std::string> two;
        sb::sayDroppedAutons(reg, runs, [&two](const char* l) { two.emplace_back(l); });
        REQUIRE(!one.empty());
        REQUIRE(!two.empty());
        CAPTURE(one[0]);
        CHECK(one[0].find("fix src/team/robot_one_autons.cpp") != std::string::npos);
        CHECK(one[0].find("src/team/autons.cpp") == std::string::npos);
        CHECK(two[0].find("fix src/team/autons.cpp") != std::string::npos);
        // ROB1-BT: robot two says the MECHANISM words now — and still names ITS OWN file (the path is its words',
        // never chosen on "claw or not": that choice would send robot two's team to robot one's file).
        std::vector<std::string> twin;
        sb::sayDroppedAutons(reg, runs, [&twin](const char* l) { twin.emplace_back(l); }, WordsOf<RobotId::RobotTwo>{});
        REQUIRE(!twin.empty());
        CHECK(twin[0].find("fix src/team/autons.cpp") != std::string::npos);
        CHECK(twin[0].find("robot_one") == std::string::npos);
        CHECK(one[0].size() < sb::kDroppedAutonLineBytes);
        // ROB1-BT review round: the route builder's routes live in their OWN file — both robots say so on the
        // line after the summary (RMP1-C: every route, not only the skills route); the bench words do not.
        const std::string route = "(the route builder's routes are fixed in src/team/skills_route.cpp)";
        REQUIRE(one.size() == two.size() + 1);
        REQUIRE(twin.size() == two.size() + 1);
        CHECK(one[1] == route);
        CHECK(twin[1] == route);
        for (const std::string& l : two) {
            CHECK(l.find("skills_route") == std::string::npos);
        }
    }
}

// ═══ the programs choose no robot (§7's greps; T7) ══════════════════════════════════════════

namespace {

/// One `cond ? a : b` in a program's code: each operand's kind and text (a string-literal run joined, or an
/// identifier, or "other" — a call).
struct WordChoice {
    enum Kind { Literal, Name, Other };
    Kind firstKind = Other;
    std::string first;
    Kind secondKind = Other;
    std::string second;
};

/// The operand starting at `i` (whitespace skipped): a run of adjacent string literals, else everything up
/// to a top-level `stop` character (`::` never stops it). `i` ends just past it.
WordChoice::Kind operandAt(const std::string& c, std::size_t& i, const std::string& stops, std::string& text) {
    const auto skip = [&] {
        while (i < c.size() && (c[i] == ' ' || c[i] == '\n' || c[i] == '\t')) {
            ++i;
        }
    };
    skip();
    text.clear();
    if (i < c.size() && c[i] == '"') {
        while (i < c.size() && c[i] == '"') {
            for (++i; i < c.size() && c[i] != '"'; ++i) {
                if (c[i] == '\\') {
                    text += c[i++];
                }
                text += c[i];
            }
            ++i;
            skip();
        }
        return WordChoice::Literal;
    }
    int depth = 0;
    for (; i < c.size(); ++i) {
        if (c.compare(i, 2, "::") == 0) {
            text += "::";
            ++i;
            continue;
        }
        if (depth == 0 && stops.find(c[i]) != std::string::npos) {
            break;
        }
        depth += (c[i] == '(' || c[i] == '{' || c[i] == '[') ? 1 : (c[i] == ')' || c[i] == '}' || c[i] == ']') ? -1 : 0;
        text += c[i];
    }
    text.erase(text.find_last_not_of(" \n\t") + 1);
    const std::regex ident{R"([A-Za-z_]\w*)"};
    return std::regex_match(text, ident) ? WordChoice::Name : WordChoice::Other;
}

/// Every `cond ?` choice in the comment-stripped code of `file`.
std::vector<WordChoice> choicesOn(const std::string& file, const std::string& cond) {
    const std::string c = source_pins::stripComments(source_pins::readFile(file));
    std::vector<WordChoice> out;
    for (std::size_t at = c.find(cond); at != std::string::npos; at = c.find(cond, at + 1)) {
        std::size_t i = at + cond.size();
        while (i < c.size() && (c[i] == ' ' || c[i] == '\n' || c[i] == '\t')) {
            ++i;
        }
        if (i >= c.size() || c[i] != '?') {
            continue;  // `if constexpr (cond)`, a declaration — not a choice
        }
        // What comes BEFORE the condition must be a context that cannot negate it: `= `, `(` or `,` (an
        // initialiser, a call's argument). A `!`, `not`, `false &&`, `x ==` or anything else fails here —
        // the walker reads the operands' ORDER, and a negated condition flips which robot gets which words
        // with the order unchanged (the verifier of ROB1-A: `!kRobotOneBuild ? kRunStatusOne : kRunStatusTwo`
        // and `!ThisRobotWords::value.claw ? …` each left the suite green).
        std::size_t b = at;
        while (b > 0 && (c[b - 1] == ' ' || c[b - 1] == '\n' || c[b - 1] == '\t')) {
            --b;
        }
        const char before = b > 0 ? c[b - 1] : '\0';
        const char twoBefore = b > 1 ? c[b - 2] : '\0';
        const bool plainContext = before == '(' || before == ',' ||
                                  (before == '=' && std::string{"=!<>"}.find(twoBefore) == std::string::npos);
        CAPTURE(file);
        CAPTURE(c.substr(b > 40 ? b - 40 : 0, (b > 40 ? 40 : b) + cond.size() + 4));
        REQUIRE_MESSAGE(plainContext, "a per-robot choice whose condition is not plainly `= cond ?`, `(cond ?` or "
                                      "`, cond ?` -- a negation would flip which robot gets which words");
        ++i;
        WordChoice w;
        w.firstKind = operandAt(c, i, ":", w.first);
        REQUIRE(i < c.size());
        REQUIRE(c[i] == ':');
        ++i;
        w.secondKind = operandAt(c, i, ",;)", w.second);
        out.push_back(w);
    }
    return out;
}

bool has(const std::string& text, const char* word) { return text.find(word) != std::string::npos; }

/// Text that is robot two's only — its driver file, its autons file, its claws.
bool saysRobotTwo(const std::string& t) {
    return has(t, "robot two") || has(t, "robot_two") || has(t, "claw") || has(t, "Its claws") ||
           has(t, "src/team/autons.cpp");
}

/// The text of the literal run a `constexpr … name = "…" "…";` defines in `file`.
std::string definedLiteral(const std::string& file, const std::string& name) {
    const std::string c = source_pins::stripComments(source_pins::readFile(file));
    const std::size_t at = c.find(" " + name + " =");
    REQUIRE_MESSAGE(at != std::string::npos, file, " defines no ", name);
    std::size_t i = c.find('=', at) + 1;
    std::string text;
    REQUIRE(operandAt(c, i, ";", text) == WordChoice::Literal);
    return text;
}

}  // namespace

TEST_CASE("this robot (R7): EVERY per-robot choice in the programs puts robot one's words on robot one's side -- in order, and robot one's say nothing of robot two") {
    // BUG CAUGHT (the review of ROB1-A, RV10): Drive's `kRobotOneBuild ? …One : …Two` choices compile only in
    // robot one's ARM builds, and no pin named them — `false ? kRunStatusOne : kRunStatusTwo` linked robot one's
    // Drive with robot two's run status ("robot two's driver file", "Its claws") and the suite stayed green.
    // And Drive's field-DISABLED line said "each claw pivot" on robot one (the review's finding — now chosen
    // like Competition's end-of-driver line). Every choice is walked from the source, both operands read.
    const char* kDriveFile = "src/programs/drive_program.cpp";
    const std::vector<WordChoice> drive = choicesOn(kDriveFile, "kRobotOneBuild");
    CHECK(drive.size() == 6);  // run status, absent devices, opcontrol live, driver control, two boot lines
    int names = 0;
    for (const WordChoice& w : drive) {
        CAPTURE(w.first);
        CAPTURE(w.second);
        REQUIRE(w.firstKind == w.secondKind);
        if (w.firstKind == WordChoice::Name) {
            ++names;
            // `kXOne : kXTwo` — the same name, robot one's FIRST; and what each name defines says so.
            REQUIRE(w.first.size() > 3);
            CHECK(w.first.substr(w.first.size() - 3) == "One");
            CHECK(w.second == w.first.substr(0, w.first.size() - 3) + "Two");
            const std::string one = definedLiteral(kDriveFile, w.first);
            const std::string two = definedLiteral(kDriveFile, w.second);
            CHECK((has(one, "robot one") || has(one, "ROBOT ONE")));
            CHECK_FALSE(saysRobotTwo(one));
            CHECK(has(two, "robot two"));
        } else {
            REQUIRE(w.firstKind == WordChoice::Literal);
            CHECK((has(w.first, "robot one") || has(w.first, "robot_one")));
            CHECK_FALSE(saysRobotTwo(w.first));
            CHECK((has(w.second, "robot two") || has(w.second, "robot_two")));
        }
    }
    CHECK(names == 3);
    // The claw-word choices with LITERAL operands: the claw words first, the mechanism words (no claw, no robot
    // two file) second — Drive's field-disabled line and Competition's end-of-driver line. (The tester's banner
    // chose its autons FILE that way until ROB1-BT — "robot two or not" — which would have sent robot two's team
    // to robot one's file once robot two said the mechanism words: it prints its words' path now, below.)
    for (const char* file : {"src/programs/drive_program.cpp", "src/programs/competition_program.cpp"}) {
        CAPTURE(file);
        int literals = 0;
        for (const WordChoice& w : choicesOn(file, "ThisRobotWords::value.claw")) {
            if (w.firstKind != WordChoice::Literal) {
                continue;  // controllerScreenLines(f) : controllerScreenLines(f, ThisRobotWords{}) and the like
            }
            ++literals;
            CAPTURE(w.first);
            CAPTURE(w.second);
            REQUIRE(w.secondKind == WordChoice::Literal);
            CHECK(saysRobotTwo(w.first));
            CHECK_FALSE(saysRobotTwo(w.second));
        }
        CHECK(literals == 1);
    }
    const std::string tester = source_pins::squash(source_pins::stripComments(source_pins::readFile("src/programs/auton_tester_program.cpp")));
    CHECK(choicesOn("src/programs/auton_tester_program.cpp", "ThisRobotWords::value.claw").empty());
    CHECK(source_pins::count(tester, "ThisRobotWords::value.autonsFilePath") == 1);
}

TEST_CASE("this robot (T7): no program or common file names a robot's driver or autons except this_robot.hpp; each robot's team files name nothing of the other's") {
    // BUG CAUGHT: a program choosing a robot outside the one chain (`RobotTwoDriver driver;` left in one
    // program: robot one's build would run robot two's buttons over a cascade), a direct registerAutons()
    // beside the dispatcher, robot one's files reaching into robot two's namespace or the reverse.
    const std::vector<std::string> files = [] {
        std::vector<std::string> v = source_pins::sourcesUnder("src/programs");
        for (const std::string& f : source_pins::sourcesUnder("src/common")) v.push_back(f);
        return v;
    }();
    REQUIRE(files.size() > 10);
    for (const std::string& f : files) {
        if (f.find("this_robot.hpp") != std::string::npos) continue;
        CAPTURE(f);
        const std::string code = source_pins::stripComments(source_pins::readFile(f));
        CHECK(source_pins::count(code, "RobotTwoDriver") == 0);
        CHECK(source_pins::count(code, "RobotOneDriver") == 0);
        CHECK(source_pins::count(code, "registerAutons(") == 0);
        CHECK(source_pins::count(code, "registerRobotOneAutons(") == 0);
    }
    for (const char* f : {"src/team/robot_one_driver.hpp", "src/team/robot_one_autons.hpp", "src/team/robot_one_autons.cpp"}) {
        CAPTURE(f);
        const std::string code = source_pins::stripComments(source_pins::readFile(f));
        CHECK(code.find("robot_two") == std::string::npos);
        CHECK(code.find("RobotTwo") == std::string::npos);
    }
    for (const char* f : {"src/team/robot_two_driver.hpp", "src/team/autons.hpp", "src/team/autons.cpp"}) {
        CAPTURE(f);
        const std::string code = source_pins::stripComments(source_pins::readFile(f));
        CHECK(code.find("robot_one") == std::string::npos);
        CHECK(code.find("RobotOne") == std::string::npos);
    }
}

TEST_CASE("this robot (§7): robot one's names reach no program or common file but this_robot.hpp -- the brief's grep, line for line") {
    // BUG CAUGHT: a program reading robot one's ledger or table by name (`robot_one_facts::kRobotOneFactCount`
    // in Drive's boot log, `robot_one::kRobotOneMechanisms` in the panel's row budget) — robot one's names
    // spreading outside its own files and the one selector, where a second robot's would follow. Mirrors
    // the brief's §7 grep exactly: every line naming `robot_one` must name robot one's own file
    // (`src/team/robot_one_…` or `src/robots/…`), this_robot.hpp exempt.
    std::vector<std::string> files = source_pins::sourcesUnder("src/programs");
    for (const std::string& f : source_pins::sourcesUnder("src/common")) files.push_back(f);
    REQUIRE(files.size() > 10);
    int offenders = 0;
    for (const std::string& f : files) {
        if (f.find("src/common/this_robot.hpp") != std::string::npos) continue;
        const std::string text = source_pins::readFile(f);
        std::size_t at = 0;
        while (at < text.size()) {
            std::size_t end = text.find('\n', at);
            if (end == std::string::npos) end = text.size();
            const std::string_view line(text.data() + at, end - at);
            if (line.find("robot_one") != std::string_view::npos && line.find("src/robots/") == std::string_view::npos
                && line.find("src/team/robot_one_") == std::string_view::npos) {
                CAPTURE(f);
                CAPTURE(std::string(line));
                ++offenders;
                CHECK(false);
            }
            at = end + 1;
        }
    }
    CHECK(offenders == 0);
}

TEST_CASE("this robot (D3): Drive's facts line counts the ledger itself -- every fact, the typed ones, the owed ones") {
    // BUG CAUGHT: the boot line reporting robot one's facts wrong — the owed count in the typed slot (a
    // robot with 17 owed facts announced as 17 typed), a stale constant, a cut line. The counts are taken
    // HERE from the ledger's own summaries, one by one, not from the constants the line prints.
    namespace facts = sb::robot_one_facts;
    int all = 0;
    int typed = 0;
    for (const facts::FactSummary& f : facts::kRobotOneFacts) {
        ++all;
        typed += f.typed ? 1 : 0;
    }
    REQUIRE(all > 0);
    REQUIRE(typed * 2 != all);  // so the typed and owed slots can never print the same number
    const std::string want = "robot one: " + std::to_string(all) + " facts, " + std::to_string(typed)
                             + " typed (src/robots/robot_one_facts.hpp); " + std::to_string(all - typed)
                             + " still owed, none needed by this program";
    char line[240];  // Drive's buffer
    const int n = sb::robotOneFactsLine(line, sizeof line);
    CHECK(std::string(line) == want);
    CHECK(n == static_cast<int>(want.size()));
    CHECK(want.size() <= 200U);  // one TermSink line (diag/term_sink.hpp's kMaxMessageBytes)
    // A short buffer is cut and NUL-terminated, never overrun; the return still counts the whole line.
    char shortLine[12];
    shortLine[11] = 'X';
    CHECK(sb::robotOneFactsLine(shortLine, sizeof shortLine) == static_cast<int>(want.size()));
    CHECK(std::string(shortLine) == want.substr(0, 11));
    // A ledger with some facts typed (robot one's has none yet, so its own line cannot tell the facts
    // count from the owed count): 17 facts, 5 owed -> 12 typed, 5 owed — each number in its own slot.
    CHECK(sb::robotOneFactsLineFor(line, sizeof line, 17, 5) >= 0);
    CHECK(std::string(line)
          == "robot one: 17 facts, 12 typed (src/robots/robot_one_facts.hpp); 5 still owed, none needed by "
             "this program");
    CHECK(sb::robotOneFactsLineFor(line, sizeof line, 17, 0) >= 0);
    CHECK(std::string(line).find("17 facts, 17 typed") != std::string::npos);
    CHECK(std::string(line).find("; 0 still owed") != std::string::npos);
}

TEST_CASE("this robot (D3, ROB1-BT): robot two's facts line counts ROBOT TWO's ledger, and its Drive prints it") {
    // BUG CAUGHT: robot two's boot line printing robot ONE's counts or path (the builder would read robot
    // one's typed count on robot two's screen while every robot-two fact is owed); thisRobotFactsLine()
    // choosing robot one's line in robot two's build; robot two's Drive printing no facts line at all.
    int all = 0;
    int typed = 0;
    for (const sb::robot_one_facts::FactSummary& f : sb::robot_two_facts::kRobotTwoFacts) {
        ++all;
        typed += f.typed ? 1 : 0;
    }
    REQUIRE(all > 0);
    const std::string want = "robot two: " + std::to_string(all) + " facts, " + std::to_string(typed)
                             + " typed (src/robots/robot_two_facts.hpp); " + std::to_string(all - typed)
                             + " still owed, none needed by this program";
    char line[240];  // Drive's buffer
    CHECK(sb::robotTwoFactsLine(line, sizeof line) == static_cast<int>(want.size()));
    CHECK(std::string(line) == want);
    // ...and it is not robot one's line (the two ledgers' counts differ today: if they ever agree, this
    // check still holds through the robot's name and path).
    char other[240];
    sb::robotOneFactsLine(other, sizeof other);
    CHECK(std::string(line) != std::string(other));
    // The host is neither robot, so the build's choice is pinned by its text: robot two's build prints
    // robot two's line, robot one's robot one's, and both robots have a ledger.
    const std::string c =
        source_pins::squash(source_pins::stripComments(source_pins::readFile("src/common/this_robot.hpp")));
    CHECK(source_pins::count(c, "if constexpr (kRobotId == RobotId::RobotTwo) { return robotTwoFactsLine(out, size); } "
                                "else { return robotOneFactsLine(out, size); }") == 1);
    CHECK(source_pins::count(c, "inline constexpr bool kThisRobotHasLedger = kRobotId == RobotId::RobotOne || "
                                "kRobotId == RobotId::RobotTwo;") == 1);
    const std::string drive = source_pins::squash(
        source_pins::stripComments(source_pins::readFile("src/programs/drive_program.cpp")));
    CHECK(source_pins::count(drive, "if constexpr (kThisRobotHasLedger) { thisRobotFactsLine(line, sizeof line); "
                                    "log.log(hal::LogLevel::Info, kTag, line); }") == 1);
}
