// ROBOT ONE'S FACTS LEDGER (chunk ROB1-A, 2026-11-11; the ROB1 brief's D3 (a), tests T2 and T3; ROB1-C2,
// 2026-11-15: the builder's first answers typed). Say "host test plant", never "simulator". Nothing has run on
// robot one: a typed fact is the builder's word or a starting value chosen for the bench, never a reading.
//
// WHAT THIS FILE TRIES TO BREAK
//   * THE LEDGER IS COMPLETE AND HONEST (T2): every fact in src/robots/robot_one_facts.hpp has EXACTLY ONE
//     check below it quoting its name verbatim (GM4: a check deleted; GM7 — the misquote — is refused at
//     build, test/compile_refusals/s78); kRobotOneFacts lists every fact once; the counts are the facts';
//     the four static_asserts live in the check macro only; a typed fact has a real source (s74, s75) and a
//     value of its own (s76, s77); and EVERY number in robot one's regions comes from the ledger — a digit in
//     robot one's mechanism region outside a comment, a ledger name or a preset slot's index fails (GM5: a
//     literal `30_in` in the cascade's range), and every bare WORD there is the declaration's structure
//     (RV4/RV5: `.cartridge = Red` typed into the region); robot one's chassis table holds only UNSET zeros
//     but the IMU port and cartridge, which are typed there with a provenance. same() is tested both ways
//     for EVERY value type (RV6/RV7).
//   * THE STAGES ARE RIGHT (T3): Stage A needed by all six programs; Stage B by Drive, the Auton Tester,
//     Competition and Tune — never Bench Tests or Calibrate, which must build on Stage A alone to MEASURE Stage B
//     (landmine 5; GM8, GM9); Recorded by none. And EVERY value of `Program` is classified (the review of
//     ROB1-A, RV-T): needsStageB() has one case per enumerator src/build_selection.hpp declares — walked
//     from that file's text, so a program added later (TUNE1-B1's Tune was the first) is seen here — and no `default`;
//     every one but Bench Tests and Calibrate needs Stage B (fail closed).
//   * ROB1-C2's TYPING (2026-11-15): every fact the builder gave that night, and every starting value chosen for
//     the bench, is TYPED — none slipped back to owed — and every typed fact's source is one of the two the
//     ledger allows ("the team's builder, 2026-11-15 …" or "chosen, not measured -- …"), so a guess typed with
//     another source turns this red; what is still owed is only what nobody has said (Bench Tests and Calibrate
//     owe exactly ONE fact: which port is the right side toggle). The cascade's levels may be EMPTY (manual for
//     now) — a preset bound to an unnamed level is refused at build (test/compile_refusals/s83).

#include "doctest.h"

#include <cctype>
#include <cstddef>
#include <regex>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "build_selection.hpp"
#include "robots/chassis_table.hpp"
#include "robots/robot_one_facts.hpp"
#include "source_pins.hpp"

namespace facts = shulib::bench::robot_one_facts;
using facts::FactSummary;
using facts::needs;
using facts::Stage;
using shulib::bench::Program;
using shulib::units::Length;
using shulib::units::Time;

namespace {

/// Robot one's programs — one per robot-one build (src/build_selection.hpp's chain): the five of ROB1-A and,
/// since its integration onto TUNE1-B1, shulib Tune.
constexpr Program kRobotOnePrograms[] = {Program::BenchTests, Program::Calibrate, Program::Drive,
                                         Program::AutonTester, Program::Competition, Program::Tune};

/// One fact as the source writes it.
struct SourceFact {
    std::string var;     ///< kCascadePorts
    bool typed = false;  ///< `typed(` rather than `owed(`
    std::string name;    ///< its name literal, unquoted
    std::string source;  ///< typed: its source literal, unquoted
    std::size_t end = 0; ///< where its call ends in the comment-stripped text
};

/// The top-level arguments of the call whose `(` is at `open` (string-aware); `end` gets the index past `)`.
std::vector<std::string> argsAt(const std::string& s, std::size_t open, std::size_t& end) {
    std::vector<std::string> args;
    std::string cur;
    int depth = 0;
    bool quoted = false;
    for (std::size_t i = open; i < s.size(); ++i) {
        const char c = s[i];
        if (quoted) {
            cur += c;
            if (c == '\\' && i + 1 < s.size()) {
                cur += s[++i];
            } else if (c == '"') {
                quoted = false;
            }
            continue;
        }
        if (c == '"') {
            quoted = true;
            cur += c;
            continue;
        }
        if (c == '(' || c == '[' || c == '{') {
            if (depth++ == 0) continue;
        } else if (c == ')' || c == ']' || c == '}') {
            if (--depth == 0) {
                args.push_back(cur);
                end = i + 1;
                return args;
            }
        } else if (c == ',' && depth == 1) {
            args.push_back(cur);
            cur.clear();
            continue;
        }
        cur += c;
    }
    end = s.size();
    return args;
}

/// The text of a string-literal argument, unquoted and trimmed ("" when it is not one literal).
std::string literal(const std::string& arg) {
    const std::size_t a = arg.find('"');
    const std::size_t b = arg.rfind('"');
    return (a == std::string::npos || b == a) ? std::string{} : arg.substr(a + 1, b - a - 1);
}

/// Every `inline constexpr auto kX = owed(...)` / `typed(...)` in the ledger, in order.
std::vector<SourceFact> sourceFacts(const std::string& code) {
    std::vector<SourceFact> out;
    const std::regex decl{R"(inline\s+constexpr\s+auto\s+(k\w+)\s*=\s*(owed|typed)\s*\()"};
    for (auto it = std::sregex_iterator(code.begin(), code.end(), decl); it != std::sregex_iterator(); ++it) {
        SourceFact f;
        f.var = (*it)[1];
        f.typed = (*it)[2] == "typed";
        const std::size_t open = static_cast<std::size_t>(it->position(0) + it->length(0) - 1);
        const std::vector<std::string> args = argsAt(code, open, f.end);
        if (f.typed) {
            REQUIRE_MESSAGE(args.size() == 5, f.var, ": typed( takes placeholder, value, stage, name, source");
            f.name = literal(args[3]);
            f.source = literal(args[4]);
        } else {
            REQUIRE_MESSAGE(args.size() == 3, f.var, ": owed( takes placeholder, stage, name");
            f.name = literal(args[2]);
        }
        out.push_back(f);
    }
    return out;
}

std::string ledgerCode() {
    return source_pins::stripComments(source_pins::readFile("src/robots/robot_one_facts.hpp"));
}

/// Text between two marker comments of `file` (comments kept), the markers excluded.
std::string region(const std::string& file, const std::string& open, const std::string& close) {
    const std::string text = source_pins::readFile(file);
    const std::size_t a = text.find(open);
    const std::size_t b = text.find(close);
    REQUIRE_MESSAGE(a != std::string::npos, file, " has no ", open);
    REQUIRE_MESSAGE(b != std::string::npos, file, " has no ", close);
    REQUIRE(a < b);
    return text.substr(a + open.size(), b - a - open.size());
}

/// The digits a region's CODE writes as numbers: comments stripped, string literals removed, the preset
/// slots `facts::lv(0..5)` removed, then every identifier removed — what is left that holds a digit.
std::vector<std::string> strayNumbers(const std::string& regionText) {
    std::string code = source_pins::stripComments(regionText);
    code = std::regex_replace(code, std::regex{R"("(\\.|[^"\\])*")"}, " ");
    code = std::regex_replace(code, std::regex{R"(facts::lv\([0-5]\))"}, " ");
    code = std::regex_replace(code, std::regex{R"([A-Za-z_]\w*)"}, " ");
    std::vector<std::string> out;
    const std::regex number{R"([0-9][0-9.'_a-zA-Z]*)"};
    for (auto it = std::sregex_iterator(code.begin(), code.end(), number); it != std::sregex_iterator(); ++it) {
        out.push_back(it->str());
    }
    return out;
}

/// The BARE words a region's code writes — identifiers not reached through `.`, `::` or `->` (comments and
/// string literals stripped). A ledger WORD fact typed straight into the declaration (`.cartridge = Red`,
/// `.motors = {Forward, Reversed}`) is one; `facts::kCascadeCartridge.value()` is not (only `facts` is bare).
std::set<std::string> bareWords(const std::string& regionText) {
    std::string code = source_pins::stripComments(regionText);
    code = std::regex_replace(code, std::regex{R"("(\\.|[^"\\])*")"}, " ");
    std::set<std::string> out;
    const std::regex word{R"((\.|::|->)?\s*([A-Za-z_]\w*))"};
    for (auto it = std::sregex_iterator(code.begin(), code.end(), word); it != std::sregex_iterator(); ++it) {
        if (!(*it)[1].matched) {
            out.insert((*it)[2]);
        }
    }
    return out;
}

/// The structure of a declaration — never a fact. Anything bare outside this list is a value typed into
/// robot one's region instead of its facts file.
const std::set<std::string> kStructuralWords = {
    "inline", "constexpr", "Lift", "cascade", "MechanismTable", "kRobotOneMechanisms", "mechanisms", "wire",
    "motors", "gears", "measured", "Hold", "constantGravity", "units", "homeByStall", "facts",
    // ROB1-C2's five: two power mechanisms (braked when let go — the builder's "it will stop") and three pistons
    // (the claw grabs a piece), wired by smart port and by 3-wire line.
    "Power", "rightToggle", "leftToggle", "Brake", "Pneumatic", "claw", "flipper", "ramp", "grabs", "lines",
};

}  // namespace

// ═══ T3 — the stages ═════════════════════════════════════════════════════════════════

static_assert(needs(Program::BenchTests, Stage::A) && needs(Program::Calibrate, Stage::A) &&
                  needs(Program::Drive, Stage::A) && needs(Program::AutonTester, Stage::A) &&
                  needs(Program::Competition, Stage::A) && needs(Program::Tune, Stage::A),
              "Stage A is needed by all six programs");
static_assert(needs(Program::Tune, Stage::B),
              "shulib Tune needs Stage B: it homes the cascade with the homing push, jogs it at the unhomed volts "
              "and moves it to its first and last level (ROB1-A's integration onto TUNE1-B1)");
static_assert(!needs(Program::BenchTests, Stage::B) && !needs(Program::Calibrate, Stage::B),
              "Bench Tests and Calibrate must build on Stage A alone: they MEASURE Stage B (landmine 5)");

namespace {

/// The enumerators of src/build_selection.hpp's `enum class Program`, in declaration order (none has an
/// explicit value, so static_cast<Program>(index) is the enumerator).
std::vector<std::string> programEnumerators() {
    const std::string sel = source_pins::stripComments(source_pins::readFile("src/build_selection.hpp"));
    const std::size_t at = sel.find("enum class Program {");
    REQUIRE(at != std::string::npos);
    const std::size_t end = sel.find("};", at);
    const std::string body = sel.substr(at + 20, end - at - 20);
    REQUIRE(body.find('=') == std::string::npos);  // no explicit values: the index IS the enumerator
    std::vector<std::string> out;
    const std::regex name{R"(([A-Za-z_]\w*)\s*,?)"};
    for (auto it = std::sregex_iterator(body.begin(), body.end(), name); it != std::sregex_iterator(); ++it) {
        out.push_back((*it)[1]);
    }
    return out;
}

}  // namespace

TEST_CASE("robot one facts (T3): EVERY Program is classified -- one case each in needsStageB(), no default, and all but Bench Tests and Calibrate need Stage B") {
    // BUG CAUGHT (the review of ROB1-A): needs() decided Stage B with `p == Drive || p == AutonTester || p ==
    // Competition`, so a program added later — TUNE1-B1's "shulib Tune", which homes the cascade, jogs it at its
    // unhomed volts and moves it to its first and last level — would build robot one on Stage A alone and drive
    // the placeholders. Walked from the enum's TEXT, so a new enumerator is seen here the day it is added.
    const std::vector<std::string> programs = programEnumerators();
    REQUIRE(programs.size() >= 6);
    const std::string code = ledgerCode();
    const std::size_t fn = code.find("constexpr bool needsStageB(Program p) {");
    REQUIRE(fn != std::string::npos);
    const std::size_t sw = code.find("switch (p) {", fn);
    const std::size_t close = code.find("\n    }\n", sw);
    REQUIRE(sw != std::string::npos);
    REQUIRE(close != std::string::npos);
    const std::string body = code.substr(sw, close - sw);
    CHECK(source_pins::count(body, "default") == 0);
    CHECK(source_pins::count(body, "case Program::") == programs.size());
    for (std::size_t i = 0; i < programs.size(); ++i) {
        const std::string& name = programs[i];
        CAPTURE(name);
        CHECK(source_pins::count(body, "case Program::" + name + ":") == 1);
        const auto p = static_cast<Program>(i);
        const bool measures = name == "BenchTests" || name == "Calibrate";
        CHECK(facts::needsStageB(p) == !measures);
        CHECK(needs(p, Stage::B) == !measures);
        CHECK(needs(p, Stage::A));
        CHECK_FALSE(needs(p, Stage::Recorded));
    }
    // A value past the last enumerator (what an unhandled program looks like here) NEEDS Stage B.
    CHECK(facts::needsStageB(static_cast<Program>(programs.size())));
    // needs()' Stage B is needsStageB() — never a second, hand-written list.
    CHECK(source_pins::count(code, "return needsStageB(p);") == 1);
    CHECK(source_pins::count(code, "p == Program::") == 0);
}

TEST_CASE("robot one facts (T3): Stage A for all six programs, Stage B for Drive, the Auton Tester, Competition and Tune (never Bench Tests or Calibrate), Recorded for none") {
    // BUG CAUGHT (GM8): Stage B needed by Bench Tests — the team locked out of the tool that measures Stage
    // B; (GM9) Stage A not needed by Calibrate — a placeholder motor port reaching the program that POWERS
    // candidates; a camera fact gating a build that never reads it ([R9]).
    for (const Program p : kRobotOnePrograms) {
        CAPTURE(static_cast<int>(p));
        CHECK(needs(p, Stage::A));
        CHECK_FALSE(needs(p, Stage::Recorded));
    }
    CHECK_FALSE(needs(Program::BenchTests, Stage::B));
    CHECK_FALSE(needs(Program::Calibrate, Stage::B));
    CHECK(needs(Program::Drive, Stage::B));
    CHECK(needs(Program::AutonTester, Stage::B));
    CHECK(needs(Program::Competition, Stage::B));
    CHECK(needs(Program::Tune, Stage::B));  // it moves the cascade (ROB1-A's integration onto TUNE1-B1)
    // Each program's owed count is exactly the owed facts of the stages it needs — counted here
    // independently of owedFor().
    for (const Program p : kRobotOnePrograms) {
        int expect = 0;
        for (const FactSummary& f : facts::kRobotOneFacts) {
            if (!f.typed && (f.needed == Stage::A ||
                             (f.needed == Stage::B && (p == Program::Drive || p == Program::AutonTester ||
                                                       p == Program::Competition || p == Program::Tune)))) {
                ++expect;
            }
        }
        CAPTURE(static_cast<int>(p));
        CHECK(facts::owedFor(p) == expect);
    }
    // Bench Tests and Calibrate owe exactly the OWED Stage A facts, the three driving programs and Tune the owed
    // Stage A and B — and none owes a Recorded fact.
    int a = 0;
    int b = 0;
    int recorded = 0;
    int owedA = 0;
    int owedB = 0;
    for (const FactSummary& f : facts::kRobotOneFacts) {
        a += f.needed == Stage::A ? 1 : 0;
        b += f.needed == Stage::B ? 1 : 0;
        recorded += f.needed == Stage::Recorded ? 1 : 0;
        owedA += (f.needed == Stage::A && !f.typed) ? 1 : 0;
        owedB += (f.needed == Stage::B && !f.typed) ? 1 : 0;
    }
    CHECK(a == 19);  // ROB1-A's ten, ROB1-C2's side toggles (four) and pistons' letters (three), and Calibrate's two thresholds (2026-11-16)
    CHECK(b == 19);  // ROB1-A's five, and ROB1-C2's side toggle directions (two), pistons' levels and strokes (six),
                     // the homing's own stall signature (the review of ROB1-C2), the autons' four drive settings
                     // (2026-11-17), and the cascade's most volts DOWN (2026-11-20, API 2.10)
    CHECK(owedB == 0);  // 2026-11-17: station 13's rows, the chosen volts, the directions and levels -- all typed
    CHECK(recorded == 2);
    CHECK(facts::owedFor(Program::BenchTests) == owedA);
    CHECK(facts::owedFor(Program::Calibrate) == owedA);
    CHECK(facts::owedFor(Program::Drive) == owedA + owedB);
    CHECK(facts::owedFor(Program::Tune) == owedA + owedB);
}

// ═══ T2 — the ledger is complete and honest ══════════════════════════════════════════════

TEST_CASE("robot one facts (T2): every fact has EXACTLY ONE check right below it, quoting its name verbatim") {
    // BUG CAUGHT (GM4): a fact whose check was deleted — its OWED refusal gone, a placeholder free to reach
    // a motor; a check that quotes another fact's name (the build refuses a misquote — s78 — and this pins
    // the pairing in the source); a second check for one fact.
    const std::string code = ledgerCode();
    const std::vector<SourceFact> all = sourceFacts(code);
    REQUIRE(all.size() == 40);  // 33, Calibrate's two thresholds (2026-11-16), the autons' four drive settings (2026-11-17),
                                // the cascade's most volts DOWN (2026-11-20)
    const std::regex check{R"re(ROBOT_ONE_CHECK\s*\(\s*(k\w+)\s*,\s*"((\\.|[^"\\])*)"\s*\))re"};
    std::vector<std::pair<std::string, std::string>> checks;
    for (auto it = std::sregex_iterator(code.begin(), code.end(), check); it != std::sregex_iterator(); ++it) {
        checks.emplace_back((*it)[1], (*it)[2]);
    }
    CHECK(checks.size() == all.size());
    for (const SourceFact& f : all) {
        CAPTURE(f.var);
        int n = 0;
        for (const auto& [var, words] : checks) {
            if (var == f.var) {
                ++n;
                CHECK(words == f.name);
            }
        }
        CHECK(n == 1);
        // RIGHT BELOW it: the next statement after the fact's `;` is its own check.
        const std::size_t next = code.find("ROBOT_ONE_CHECK", f.end);
        REQUIRE(next != std::string::npos);
        const std::size_t open = code.find('(', next);
        const std::size_t comma = code.find(',', open);
        std::string var = code.substr(open + 1, comma - open - 1);
        var.erase(0, var.find_first_not_of(" \t\n"));
        var.erase(var.find_last_not_of(" \t\n") + 1);
        CHECK(var == f.var);
    }
    // The macro holds the only static_asserts in the ledger (the banner says four).
    CHECK(source_pins::count(code, "static_assert(") == 4);
    // Never consteval (P1: it fails EVERY build and names nothing — landmine 2).
    CHECK(source_pins::count(code, "consteval") == 0);
}

TEST_CASE("robot one facts (T2): kRobotOneFacts lists EVERY fact once, the counts are the facts', and every name is unique") {
    // BUG CAUGHT: a fact typed into the ledger but left out of the summary — Drive's boot line and RPV1-B's
    // hatch would then under-count what is owed; two facts sharing a name (their OWED lines would merge in
    // the gate's set compare).
    const std::string code = ledgerCode();
    const std::vector<SourceFact> all = sourceFacts(code);
    const std::size_t at = code.find("kRobotOneFacts[] = {");
    REQUIRE(at != std::string::npos);
    const std::string list = code.substr(at, code.find("};", at) - at);
    int owed = 0;
    std::set<std::string> names;
    for (const SourceFact& f : all) {
        CAPTURE(f.var);
        CHECK(source_pins::count(list, "summary(" + f.var + ")") == 1);
        owed += f.typed ? 0 : 1;
        CHECK(names.insert(f.name).second);
        CHECK_FALSE(f.name.empty());
    }
    CHECK(facts::kRobotOneFactCount == static_cast<int>(all.size()));
    CHECK(source_pins::count(list, "summary(") == all.size());
    CHECK(facts::kRobotOneOwedCount == owed);
    for (const FactSummary& f : facts::kRobotOneFacts) {
        CAPTURE(f.name);
        CHECK(names.count(f.name) == 1);  // the summary carries the SAME words the source writes
    }
}

TEST_CASE("robot one facts (T2): a typed fact carries a real source of its own, and owed facts carry none") {
    // BUG CAUGHT (GM6, R4): a fact typed with an empty source, or with the gate copy's marker (a ledger copied
    // back from the gate's copy) — in the source, and in the machinery the build's checks use.
    for (const SourceFact& f : sourceFacts(ledgerCode())) {
        CAPTURE(f.var);
        if (f.typed) {
            CHECK_FALSE(f.source.empty());
            CHECK(f.source.find("gate copy") == std::string::npos);
        }
    }
    static_assert(facts::realSource("the builder, tape, 2026-11-14"));
    static_assert(!facts::realSource(""));
    static_assert(!facts::realSource(nullptr));
    static_assert(!facts::realSource("gate copy -- same as the placeholder, checked"));
    static_assert(!facts::realSource("typed in the gate copy"));
    static_assert(facts::kRefuseGateCopySource, "the repository's ledger refuses the gate copy's source");
    // ...and the switch is NOT in the ledger's text (the review of ROB1-A: a text switch the gate's copy
    // flipped came back WITH a copied-back ledger, which then linked). It is the copy's marker FILE, read
    // once by `__has_include`; the ledger states each value once, inside that #if/#else and nowhere else.
    const std::string raw = source_pins::readFile("src/robots/robot_one_facts.hpp");
    CHECK(source_pins::count(raw, "#if __has_include(\"../../build/ROBOT-ONE-GATE-COPY-ONLY\")\n"
                                  "inline constexpr bool kRefuseGateCopySource = false;\n#else\n"
                                  "inline constexpr bool kRefuseGateCopySource = true;\n#endif\n") == 1);
    CHECK(source_pins::count(raw, "kRefuseGateCopySource =") == 2);
    // [R10] value() reads the TYPED value once typed — never the placeholder it keeps.
    constexpr auto t = facts::typed(1.0, 2.5, Stage::A, "x", "the builder");
    static_assert(t.value() == 2.5 && t.placeholder == 1.0);
    constexpr auto o = facts::owed(1.0, Stage::A, "x");
    static_assert(o.value() == 1.0 && !o.typed && o.source == nullptr);
    // same() is field-wise for every value type a fact holds — BOTH ways for each (the review of ROB1-A, RV6/RV7:
    // a same() that answered "different" for Rows or PortList let a value typed EQUAL to its placeholder
    // through with no "same as the placeholder, checked"; only the double and Teeth were tested). Each type:
    // a value is the same as itself, and differs when any ONE field differs.
    static_assert(facts::same(1.0, 1.0) && !facts::same(1.0, 1.5));
    static_assert(facts::same(3, 3) && !facts::same(3, 4));
    static_assert(facts::same('A', 'A') && !facts::same('A', 'B'));  // a 3-wire letter (ROB1-C2)
    static_assert(facts::same(true, true) && !facts::same(true, false) && !facts::same(false, true));  // a line level
    static_assert(facts::same(shulib::mechanism::words::Forward, shulib::mechanism::words::Forward) &&
                  !facts::same(shulib::mechanism::words::Forward, shulib::mechanism::words::Reversed));  // one motor
    static_assert(facts::same(facts::FlipperKind::PassiveBar, facts::FlipperKind::PassiveBar) &&
                  !facts::same(facts::FlipperKind::PassiveBar, facts::FlipperKind::MotorArm));
    static_assert(facts::same(Length{2.0}, Length{2.0}) && !facts::same(Length{2.0}, Length{2.5}));
    static_assert(facts::same(Time{0.3}, Time{0.3}) && !facts::same(Time{0.3}, Time{0.4}));
    static_assert(!facts::same("L1", "L2") && facts::same("L1", "L1") && !facts::same("L1", nullptr));
    static_assert(facts::same(facts::Teeth{12, 60}, facts::Teeth{12, 60}));
    static_assert(!facts::same(facts::Teeth{12, 60}, facts::Teeth{12, 36}));
    static_assert(!facts::same(facts::Teeth{12, 60}, facts::Teeth{36, 60}));
    static_assert(facts::same(facts::PortList{{8, 9}, 2}, facts::PortList{{8, 9}, 2}));
    static_assert(!facts::same(facts::PortList{{8, 9}, 2}, facts::PortList{{8, 9, 10}, 3}));
    static_assert(!facts::same(facts::PortList{{8, 9}, 2}, facts::PortList{{8, 10}, 2}));
    static_assert(!facts::same(facts::PortList{{8, 9}, 2}, facts::PortList{{8, 9}, 1}));
    using Pi = facts::Pair<int>;
    static_assert(facts::same(Pi{8, 9}, Pi{8, 9}) && !facts::same(Pi{8, 9}, Pi{7, 9}) && !facts::same(Pi{8, 9}, Pi{8, 7}));
    using Pm = facts::Pair<shulib::mechanism::Motor>;
    constexpr shulib::mechanism::Motor fwd = shulib::mechanism::words::Forward;
    constexpr shulib::mechanism::Motor rev = shulib::mechanism::words::Reversed;
    static_assert(facts::same(Pm{fwd, rev}, Pm{fwd, rev}) && !facts::same(Pm{fwd, rev}, Pm{rev, rev}) &&
                  !facts::same(Pm{fwd, rev}, Pm{fwd, fwd}));
    // Rows: the ledger's placeholder against itself, and against itself with ONE field of ONE row changed —
    // each row, both fields.
    constexpr facts::Rows rows = facts::kCascadeRows.placeholder;
    static_assert(facts::same(rows, rows));
    for (std::size_t i = 0; i < facts::kCascadeRowCount; ++i) {
        CAPTURE(i);
        facts::Rows h = rows;
        h.row[i].height = Length{h.row[i].height.value() + 0.5};
        CHECK_FALSE(facts::same(rows, h));
        facts::Rows d = rows;
        d.row[i].degrees += 1.0;
        CHECK_FALSE(facts::same(rows, d));
    }
    // Levels: each slot's name and height.
    constexpr facts::Levels levels = facts::kCascadeLevels.placeholder;
    static_assert(facts::same(levels, levels));
    for (std::size_t i = 0; i < shulib::mechanism::kMaxPresets; ++i) {
        CAPTURE(i);
        facts::Levels n = levels;
        n.level[i].name = "renamed";
        CHECK_FALSE(facts::same(levels, n));
        facts::Levels a = levels;
        a.level[i].at = Length{a.level[i].at.value() + 0.5};
        CHECK_FALSE(facts::same(levels, a));
    }
    // CameraMount: each of its four fields.
    constexpr facts::CameraMount cam{Length{6.0}, Length{0.0}, Length{10.0}, 0.0};
    static_assert(facts::same(cam, cam));
    static_assert(!facts::same(cam, facts::CameraMount{Length{7.0}, Length{0.0}, Length{10.0}, 0.0}));
    static_assert(!facts::same(cam, facts::CameraMount{Length{6.0}, Length{1.0}, Length{10.0}, 0.0}));
    static_assert(!facts::same(cam, facts::CameraMount{Length{6.0}, Length{0.0}, Length{11.0}, 0.0}));
    static_assert(!facts::same(cam, facts::CameraMount{Length{6.0}, Length{0.0}, Length{10.0}, 5.0}));
}

TEST_CASE("robot one facts (T2): every number in robot one's mechanism region comes from the ledger; its chassis table holds only UNSET zeros") {
    // BUG CAUGHT (GM5): a number typed straight into robot one's declaration (`.range = {0_in, 30_in}`) —
    // never owed, never refused, never typed with a source; a non-zero guess in robot one's chassis table.
    const std::string mech =
        region("src/robots/mechanism_table.hpp", "// [robot one mechanisms]", "// [/robot one mechanisms]");
    CHECK(mech.find("cascade") != std::string::npos);  // the scan reads the real region
    const std::vector<std::string> stray = strayNumbers(mech);
    for (const std::string& n : stray) {
        FAIL_CHECK("a literal number in robot one's mechanism region: '" << n
                   << "' -- type it in src/robots/robot_one_facts.hpp and read it from there");
    }
    CHECK(stray.empty());
    // The scan itself sees a literal (it is not blind).
    CHECK(strayNumbers(".range = {facts::cascadeBottom(), 30_in},").size() == 1);
    CHECK(strayNumbers("{facts::lv(5).name, facts::lv(5).at}").empty());
    CHECK(strayNumbers("{facts::lv(6).name}").size() == 1);
    // ...and every WORD (the review of ROB1-A, RV4/RV5: the digit scan deletes identifiers, so `.cartridge =
    // Red` or `.motors = {Forward, Reversed}` typed straight into the region slipped past it — both are
    // ledger facts). Every bare word in the region is the declaration's structure, never a value.
    for (const std::string& w : bareWords(mech)) {
        CAPTURE(w);
        CHECK_MESSAGE(kStructuralWords.count(w) == 1,
                      "a bare word in robot one's mechanism region: '", w,
                      "' -- a value belongs in src/robots/robot_one_facts.hpp, read as facts::...");
    }
    CHECK(bareWords(".cartridge = Red,") == std::set<std::string>{"Red"});
    CHECK(bareWords(".motors = {Forward, Reversed},") == std::set<std::string>{"Forward", "Reversed"});
    CHECK(bareWords(".cartridge = mechanism::words::Red,") == std::set<std::string>{"mechanism"});
    CHECK(bareWords(".cartridge = facts::kCascadeCartridge.value(), // Red\n.name = \"Red\"").count("Red") == 0);
    // Robot one's chassis table: every number an UNSET zero (the table's own rule — D3) — except the IMU's
    // port and the drive cartridge, the facts form's A4 and A2, which Calibrate cannot discover: those two
    // are TYPED here by the coordinator (guide 21's order, step 1), and only with the table's provenance
    // no longer saying "nothing typed" (the review of ROB1-A: the guide sent the builder to Calibrate with
    // both UNSET, and Calibrate refuses an unset cartridge).
    const std::string table = region("src/robots/chassis_table.hpp", "inline constexpr ChassisTable kRobotOneChassis = {",
                                     "};\n\n/// Robot `id`'s chassis table");
    // Since 2026-11-17 its DRIVE is shulib Calibrate's measurement, transcribed (no SD card): the ports, the IMU's
    // port and the three geometry numbers are the run's, allowed ONLY while the table says MEASURED by Calibrate
    // (robot_one_tables_test pins each to the run's log). Every OTHER number stays an UNSET zero.
    const bool calibrated = shulib::bench::kRobotOneChassis.measured &&
                            std::string_view{shulib::bench::kRobotOneChassis.provenance}
                                    .rfind("MEASURED by shulib Calibrate", 0) == 0;
    std::string rest = std::regex_replace(table, std::regex{R"(\.imuPort\s*=\s*[0-9]+\s*,)"}, " ");
    CHECK(rest.size() < table.size());  // the IMU line was there to take out
    if (calibrated) {
        rest = std::regex_replace(rest, std::regex{R"(\.(left|right)\s*=\s*\{[^}]*\},)"}, " ");
        rest = std::regex_replace(rest, std::regex{R"(\.(leftCount|rightCount)\s*=\s*[0-9]+,)"}, " ");
        rest = std::regex_replace(rest, std::regex{R"(\.(wheelDiameterIn|trackWidthIn|externalRatio)\s*=\s*[0-9.]+,)"}, " ");
    }
    for (const std::string& n : strayNumbers(rest)) {
        CAPTURE(n);
        CHECK((n == "0" || n == "0.0"));
    }
    const auto& one = shulib::bench::kRobotOneChassis;
    const bool nothingTyped = std::string_view{one.provenance}.rfind("nothing typed", 0) == 0;
    if (one.imuPort != 0 || one.cartridge != shulib::bench::Cartridge::Unset) {
        CHECK_MESSAGE(!nothingTyped, "robot one's chassis table has the IMU port or the cartridge typed, and "
                                     "its provenance still says 'nothing typed' -- say who read it and when");
    }
}

// ═══ ROB1-C2 — the builder's first answers, typed ═══════════════════════════════════════════

TEST_CASE("robot one facts (ROB1-C2): every fact the builder gave on 2026-11-15, and every starting value chosen for the bench, is TYPED -- each with the builder's words or 'chosen, not measured'; every other typed fact says who, how and WHEN") {
    // BUG CAUGHT: a fact the builder gave left (or put back) OWED — robot one's builds would refuse for nothing;
    // one of that night's facts whose source is neither the builder's words nor "chosen, not measured" — a
    // guess typed as a fact (landmine 17); Bench Tests or Calibrate waiting on more than the one fact nobody
    // has said. NOT a bug (the review of ROB1-C2): a fact typed LATER — station 13's rows, a Tune COPY — in its
    // own words and on its own date ("shulib Tune COPY, 2026-11-17"): the ledger's banner asks only <who, how,
    // when>, so for those this test asks for a date and no "guess", never tonight's two wordings.
    const std::set<std::string> given = {
        "every motor port the Devices screen lists that is not a drive motor",
        "the cascade's motor ports",
        "the cascade motors' directions",
        "the cascade motors' cartridge",
        "the cascade's gear teeth, motor then spool",
        "the cascade's starting stall signature, amps",
        "the cascade's starting stall signature, below this motor rpm",
        "the cascade's starting stall signature, lasting this long",
        "the cascade's starting current cap per motor, amps",
        "the flipper's kind",
        "the side toggles' motor ports, either order",
        "the side toggles' cartridge",
        "the side toggles' starting current cap per motor, amps",
        "the claw's 3-wire port",
        "the front toggle's 3-wire port (the flipper)",
        "the ramp's 3-wire port",
        "the cascade's levels, by name and stack height",
        "the claw's stroke time",
        "the front toggle's stroke time",
        "the ramp's stroke time",
        "the cascade's homing stall signature, amps",
    };
    std::set<std::string> typedNames;
    for (const SourceFact& f : sourceFacts(ledgerCode())) {
        CAPTURE(f.var);
        if (!f.typed) {
            continue;
        }
        typedNames.insert(f.name);
        if (given.count(f.name) == 1) {
            const bool builder = f.source.rfind("the team's builder, 2026-11-15", 0) == 0;
            const bool chosen = f.source.rfind("chosen, not measured -- ", 0) == 0;
            CHECK_MESSAGE((builder || chosen),
                          "a fact typed on 2026-11-15 has the builder's words or 'chosen, not measured' as its source: ",
                          f.source);
        } else {
            CHECK_MESSAGE(std::regex_search(f.source, std::regex{R"(20\d\d-\d\d-\d\d)"}),
                          "a typed fact's source says WHEN (a date): ", f.source);
            CHECK_MESSAGE(f.source.find("guess") == std::string::npos, "a typed fact is never a guess: ", f.source);
        }
    }
    for (const std::string& name : given) {
        CAPTURE(name);
        CHECK(typedNames.count(name) == 1);
    }
    // The facts nobody has said are the ONLY Stage A owed: which port is the right side toggle.
    std::set<std::string> owedA;
    for (const FactSummary& f : facts::kRobotOneFacts) {
        if (!f.typed && f.needed == Stage::A) {
            owedA.insert(f.name);
        }
    }
    CHECK(owedA.size() <= 1);
    if (owedA.size() == 1) {
        CHECK(*owedA.begin() == "the right side toggle's port");
    }
    CHECK(facts::owedFor(Program::BenchTests) == static_cast<int>(owedA.size()));
    CHECK(facts::owedFor(Program::Calibrate) == static_cast<int>(owedA.size()));
}

TEST_CASE("robot one facts (ROB1-C2): the typed values are the builder's -- ports 10 and 1 facing each other, blue, direct; the side toggles 9 and 2, green; the pistons on A, B and C; the front toggle the flipper") {
    // BUG CAUGHT: a typo in a typed fact (port 11 for 1, the toggles' cartridge typed red, the ramp on B), which
    // the build alone would accept; a starting pair that breaks chapter 18's order (the signature not more than
    // 0.2 A under the cap).
    static_assert(facts::kCascadePorts.value().first == 10 && facts::kCascadePorts.value().second == 1);
    static_assert(!facts::kCascadeDirections.value().first.reversed && facts::kCascadeDirections.value().second.reversed);
    static_assert(facts::kCascadeCartridge.value() == shulib::hal::MotorCartridge::Blue);
    static_assert(facts::kCascadeTeeth.value().motor == 1 && facts::kCascadeTeeth.value().spool == 1);
    static_assert(facts::kCascadeCapAmps.value() == 2.0 && facts::kCascadeStallAmps.value() == 1.5);
    static_assert(facts::kCascadeStallRpm.value() == 5.0 && facts::kCascadeStallTime.value().value() == 0.3);
    static_assert(facts::kCascadeStallAmps.value() < facts::kCascadeCapAmps.value() - 0.2);
    // The homing's own signature: chapter 18's 0.45 A for a gentle push — below the jam signature (a gentle
    // push draws less than a jam) and the cap. Not the jam's: a 1.5 A homing signature never ends a 3 V home.
    static_assert(facts::kCascadeHomingStallAmps.value() == 0.45);
    static_assert(facts::kCascadeHomingStallAmps.value() < facts::kCascadeStallAmps.value());
    static_assert(facts::cascadeHomingStall().belowMotorRpm == facts::cascadeStall().belowMotorRpm);
    static_assert(facts::cascadeHomingStall().lasting.value() == facts::cascadeStall().lasting.value());
    static_assert(facts::kSideTogglePorts.value().first == 9 && facts::kSideTogglePorts.value().second == 2);
    static_assert(facts::kSideToggleCartridge.value() == shulib::hal::MotorCartridge::Green);
    static_assert(facts::kSideToggleCapAmps.value() > 0.0 && facts::kSideToggleCapAmps.value() <= 2.5);
    static_assert(facts::kClawLine.value() == 'A' && facts::kFlipperLine.value() == 'B' && facts::kRampLine.value() == 'C');
    static_assert(facts::kFlipperKind.value() == facts::FlipperKind::Pneumatic);
    const facts::PortList& listed = facts::kNonDriveMotorPorts.value();
    CHECK(listed.count == 4);
    const std::set<int> ports(listed.port, listed.port + listed.count);
    CHECK(ports == std::set<int>{10, 1, 9, 2});
    // The levels: typed as NONE (manual for now) — every slot empty.
    for (std::size_t i = 0; i < shulib::mechanism::kMaxPresets; ++i) {
        CAPTURE(i);
        CHECK(facts::lv(i).name == nullptr);
    }
    // ...and the rows: the library's least, two — the bottom stop and the top stop station 13 RECORDs.
    static_assert(facts::kCascadeRowCount == 2);
    // The left side toggle is the OTHER of the two ports, whichever the right one is typed as.
    CHECK(facts::leftTogglePort() != facts::kRightTogglePort.value());
    CHECK(((facts::leftTogglePort() == 9 || facts::leftTogglePort() == 2)));
}
