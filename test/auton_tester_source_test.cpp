// auton_tester_source_test.cpp — source pins for what the host cannot compile: "shulib Auton
// Tester"'s program TU (src/programs/auton_tester_program.cpp includes pros/screen.h) and
// src/main.cpp's cases for it (chunk COMP-t, 2026-09-28; the brief's §5 source pins, the re-check's
// findings 4, 6, 9 and 12). Read through SHULIB_SOURCE_DIR (the api_reference_fidelity_test
// pattern); comments are stripped before any code pin, so a sentence that mentions a name is never
// mistaken for a call.
//
// A pin catches only the spellings it names — that is why the boot also PROVES the pacer chain at
// run time (pacerChainProven, host-tested in test/auton_abort_watch_test.cpp (13b)). What only a
// pin can guard is said where it is: the disabled read (mutation 23 — the host cannot call PROS,
// and no bench station can disable the robot) and the exact Robot construction (mutation 24).

#include "doctest.h"

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

std::string readFile(const std::string& rel) {
    const fs::path p = fs::path(SHULIB_SOURCE_DIR) / rel;
    std::ifstream in(p);
    REQUIRE_MESSAGE(in.good(), "cannot open ", p.string(),
                    " — a pin that cannot read its subject is decoration");
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

/// `text` with // and /* */ comments removed (string and character literals kept, line breaks
/// kept).
std::string stripComments(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    std::size_t i = 0;
    const std::size_t n = text.size();
    while (i < n) {
        const char c = text[i];
        if (c == '"' || c == '\'') {
            const char q = c;
            out += c;
            ++i;
            while (i < n && text[i] != q) {
                if (text[i] == '\\' && i + 1 < n) {
                    out += text[i];
                    ++i;
                }
                out += text[i];
                ++i;
            }
            if (i < n) {
                out += text[i];
                ++i;
            }
        } else if (c == '/' && i + 1 < n && text[i + 1] == '/') {
            while (i < n && text[i] != '\n') {
                ++i;
            }
        } else if (c == '/' && i + 1 < n && text[i + 1] == '*') {
            i += 2;
            while (i + 1 < n && !(text[i] == '*' && text[i + 1] == '/')) {
                if (text[i] == '\n') {
                    out += '\n';
                }
                ++i;
            }
            i += 2;
        } else {
            out += c;
            ++i;
        }
    }
    return out;
}

/// Every string literal's contents in `code` (comments already stripped), adjacent ones joined.
std::vector<std::string> stringLiterals(const std::string& code) {
    std::vector<std::string> out;
    std::size_t i = 0;
    bool joining = false;
    while (i < code.size()) {
        if (code[i] == '\'') {  // a character literal: skip it whole
            ++i;
            while (i < code.size() && code[i] != '\'') {
                i += code[i] == '\\' ? std::size_t{2} : std::size_t{1};
            }
            ++i;
            continue;
        }
        if (code[i] != '"') {
            if (code[i] != ' ' && code[i] != '\n' && code[i] != '\t') {
                joining = false;
            }
            ++i;
            continue;
        }
        std::string lit;
        ++i;
        while (i < code.size() && code[i] != '"') {
            if (code[i] == '\\' && i + 1 < code.size()) {
                lit += code[i];
                ++i;
            }
            lit += code[i];
            ++i;
        }
        ++i;
        if (joining && !out.empty()) {
            out.back() += lit;
        } else {
            out.push_back(lit);
        }
        joining = true;
    }
    return out;
}

std::size_t count(const std::string& hay, const std::string& needle) {
    std::size_t n = 0;
    for (std::size_t at = hay.find(needle); at != std::string::npos;
         at = hay.find(needle, at + needle.size())) {
        ++n;
    }
    return n;
}

/// The body of the first function whose header contains `signature`: from its `{` to the
/// matching `}` (braces inside literals are not in this file's code).
std::string bodyOf(const std::string& code, const std::string& signature) {
    const std::size_t at = code.find(signature);
    REQUIRE_MESSAGE(at != std::string::npos, "no function '", signature, "'");
    const std::size_t open = code.find('{', at);
    int depth = 0;
    for (std::size_t i = open; i < code.size(); ++i) {
        if (code[i] == '{') {
            ++depth;
        } else if (code[i] == '}' && --depth == 0) {
            return code.substr(open + 1, i - open - 1);
        }
    }
    FAIL("unbalanced braces after '", signature, "'");
    return {};
}

/// `text` with every run of whitespace collapsed to one space, trimmed.
std::string squash(const std::string& text) {
    std::string out;
    bool space = false;
    for (const char c : text) {
        if (c == ' ' || c == '\n' || c == '\t' || c == '\r') {
            space = !out.empty();
            continue;
        }
        if (space) {
            out += ' ';
            space = false;
        }
        out += c;
    }
    return out;
}

const std::string kProgramFile = "src/programs/auton_tester_program.cpp";

}  // namespace

// Bug caught (mutation 17): the program calling runAuton itself — the tester's sequence (the
// countdown, the quiet tick, the one try) would be bypassed, and the host tests would test a
// sequence the robot never runs.
TEST_CASE("pins: the program runs autons ONLY through runTesterOnce") {
    const std::string code = stripComments(readFile(kProgramFile));
    CHECK(count(code, "runTesterOnce(") == 1);
    CHECK(count(code, "runAuton(") == 0);
    CHECK(count(code, "registerAutons(") == 1);
}

// Bug caught (the re-check's finding 4; mutations 24 and 25): the Robot built WITHOUT the options
// that carry the guard (`buildOptions()` passed straight through — the guard never paces, and no
// abort can stop a motion), or built on the watch (the abandon requested and never acted on), and
// the chain proof moved after the task starts or out of the try (a failed proof would not refuse).
TEST_CASE("pins: the Robot is built WITH opts, the guard its pacer, the chain proven before the task") {
    const std::string code = stripComments(readFile(kProgramFile));
    CHECK(count(code, "st.robot.emplace(st.boot.built.config, *st.devices, opts);") == 1);
    CHECK(count(code, "st.robot.emplace(") == 1);
    CHECK(count(code, "robot::validate(st.boot.built.config, *st.devices, opts)") == 1);
    CHECK(count(code, "opts.pacer = &*st.guard;") == 1);
    CHECK(count(code, ".pacer =") == 1);
    CHECK(count(code, "st.guard.emplace(*st.watch)") == 1);
    CHECK(count(code, "st.watch.emplace(*st.painter") == 1);
    CHECK(count(code, "st.painter.emplace(st.devices->pacer())") == 1);
    CHECK(count(code, "attachGuard(") == 1);

    CHECK(count(code, "pacerChainProven(") == 1);
    const std::size_t tryAt = code.find("    try {\n        st.devices.emplace();");
    const std::size_t catchAt = code.find("} catch (const PreconditionError& e) {\n        unbuild();");
    const std::size_t robotAt = code.find("st.robot.emplace(");
    const std::size_t proofAt = code.find("pacerChainProven(");
    const std::size_t registerAt = code.find("registerAutons(");
    const std::size_t taskAt = code.find("st.task.emplace(");
    REQUIRE(tryAt != std::string::npos);
    REQUIRE(catchAt != std::string::npos);
    CHECK(tryAt < robotAt);
    CHECK(robotAt < proofAt);
    CHECK(proofAt < catchAt);  // inside the try: a throw or a false proof refuses the boot
    CHECK(tryAt < registerAt);
    CHECK(registerAt < catchAt);
    CHECK(catchAt < taskAt);
    CHECK(count(code, "st.task.emplace(&testerTask, nullptr, TASK_PRIORITY_DEFAULT, "
                      "TASK_STACK_DEPTH_DEFAULT,") == 1);
    CHECK(count(code, "st.task.emplace(") == 1);
}

// Bug caught (the re-check's finding 4; mutations 21-23): the PROS reads widened past one line
// each — the press/release counts or the error status read in the program, where no host test can
// see them, or the disabled flag dropped (mutation 23: this pin is its ONLY guard).
TEST_CASE("pins: PROS's two reads are ONE line each, through the host-tested reductions") {
    const std::string code = stripComments(readFile(kProgramFile));
    CHECK(squash(bodyOf(code, "TesterTouch readTouch()"))
          == "return testerTouchFrom(pros::c::screen_touch_status(), "
             "static_cast<int>(pros::E_TOUCH_ERROR));");
    CHECK(squash(bodyOf(code, "AbortInputs read() override"))
          == "return abortInputsFrom(readTouch(), *master_, "
             "pros::c::competition_is_disabled() != 0);");
    CHECK(count(code, "screen_touch_status") == 1);
    CHECK(count(code, "press_count") == 0);
    CHECK(count(code, "release_count") == 0);
    CHECK(count(code, "competition_is_disabled") == 1);
}

// Bug caught (the re-check's finding 6): a throw after boot killing the task silently (a C++
// exception out of a PROS task function is std::terminate, the drive in whatever state it was
// last given) — the task's whole body must be one try whose handlers stop the tester, then idle.
TEST_CASE("pins: the tester's task is one try, both handlers stop the tester, then it idles") {
    const std::string code = stripComments(readFile(kProgramFile));
    const std::string body = squash(bodyOf(code, "void testerTask(void*"));
    CHECK(body.rfind("TesterState& st = state(); try {", 0) == 0);
    CHECK(count(body, "} catch (const PreconditionError& e) { stopTheTester(e.what()); }") == 1);
    // (COMP-t's review) any other std::exception stops the tester in its OWN words.
    CHECK(count(body, "} catch (const std::exception& e) { stopTheTester(e.what());") == 1);
    CHECK(body.find("catch (const std::exception& e)") < body.find("catch (...)"));
    CHECK(count(body, "catch (...) { stopTheTester(\"an unknown exception\"); }") == 1);
    const std::size_t lastCatch = body.find("catch (...)");
    const std::size_t idle = body.find("for (;;) { pros::delay(100); }");
    CHECK(idle != std::string::npos);
    CHECK(lastCatch < idle);
    CHECK(body.size() - (idle + std::string{"for (;;) { pros::delay(100); }"}.size()) == 0);
    // stopTheTester's screen footer is the literal the file checks at compile time.
    const std::string stop = bodyOf(code, "void stopTheTester(const char* what) noexcept");
    CHECK(count(stop, "\"The drive is stopped (0 V + Brake). Restart the program.\"") == 1);
    CHECK(count(code, "kStoppedFooter =\n    \"The drive is stopped (0 V + Brake). Restart the "
                      "program.\";") == 1);
}

// Bug caught (the re-check's finding 12): the SD log bypassing the adapter that owns /usd/ — a raw
// fopen (SAFE-SAVE's platform scan allows it in three files only), a /usd/ literal (a second owner
// of the prefix), or the log opened truncating (every reboot erases the session's runs).
TEST_CASE("pins: the SD run log goes through the adapter's Append mode, and names no /usd/") {
    const std::string code = stripComments(readFile(kProgramFile));
    CHECK(count(code, "fopen") == 0);
    CHECK(count(code, "\"/usd/") == 0);
    for (const std::string& lit : stringLiterals(code)) {
        CHECK_MESSAGE(lit.find("/usd/") == std::string::npos, "a /usd/ literal: ", lit);
    }
    CHECK(count(code, "ProsFileMode::Append") == 1);
    CHECK(count(code, "constexpr const char* kSdRunLogFile = \"shulib_auton_runs.txt\";") == 1);
}

// Bug caught: the program commanding the drive itself — every motion the tester makes must be the
// auton's, through runAuton, under the guard.
TEST_CASE("pins: the program names no motion verb") {
    const std::string code = stripComments(readFile(kProgramFile));
    for (const char* verb : {".drive(", ".moveTo(", ".turnTo(", ".brake(", "setVoltage(",
                             "move_voltage", "cancel(", ".hold(", ".face("}) {
        CAPTURE(verb);
        CHECK(count(code, verb) == 0);
    }
}

// Bug caught (T4's N4): a slot number in a string the robot prints — the slot is the documents'
// to name (it moves from 7 to 2 when T4 Part 3 retires Teleop), and a printed one would go stale.
TEST_CASE("pins: no string in the tester's files names a slot number") {
    const std::regex slot{"slot[ -]*[0-9]", std::regex::icase};
    for (const char* file : {"src/programs/auton_tester_program.cpp",
                             "src/programs/auton_tester_program.hpp",
                             "src/programs/auton_tester_panel.hpp",
                             "src/programs/auton_tester_run.hpp",
                             "src/programs/auton_abort_watch.hpp", "src/team/autons.cpp",
                             "src/team/autons.hpp"}) {
        CAPTURE(file);
        for (const std::string& lit : stringLiterals(stripComments(readFile(file)))) {
            CHECK_MESSAGE(!std::regex_search(lit, slot), "a slot number in: ", lit);
        }
    }
}

// Bug caught (the re-check's finding 9): a PROS header in a file the host suite compiles as
// PROS-free. The CI grep covers include/shulib only, and the host shim provides pros/rtos.hpp and
// pros/misc.hpp — so such an include would still compile on the host and hide a robot-only call.
TEST_CASE("pins: the pure tester headers and the students' file include no pros/ header") {
    const std::regex prosInclude{R"(#\s*include\s*[<"]pros/)"};
    for (const char* file : {"src/programs/auton_abort_watch.hpp",
                             "src/programs/auton_tester_run.hpp",
                             "src/programs/auton_tester_panel.hpp", "src/team/autons.hpp",
                             "src/team/autons.cpp"}) {
        CAPTURE(file);
        CHECK_FALSE(std::regex_search(stripComments(readFile(file)), prosInclude));
    }
}

// Bug caught (mutation 18's family): src/main.cpp doing something in the tester's build from a
// PROS entry point other than initialize() — a second painter over the tester's screen, an auton in
// autonomous(), or driver control in opcontrol() (the tester's task owns the robot; PROS deletes
// the competition tasks on a mode change — HA-161).
TEST_CASE("pins: src/main.cpp's AutonTester cases outside initialize() only return") {
    const std::string code = stripComments(readFile("src/main.cpp"));
    for (const char* fn : {"void disabled()", "void competition_initialize()",
                           "void autonomous()", "void opcontrol()"}) {
        CAPTURE(fn);
        const std::string body = bodyOf(code, fn);
        const std::size_t at = body.find("case Program::AutonTester:");
        REQUIRE(at != std::string::npos);
        std::size_t end = body.find("case Program::", at + 1);
        if (end == std::string::npos) {
            end = body.rfind('}');  // the switch's closing brace
        }
        CHECK(squash(body.substr(at, end - at)) == "case Program::AutonTester: return;");
    }
    const std::string init = squash(bodyOf(code, "void initialize()"));
    CHECK(count(init, "case Program::AutonTester: shulib::bench::autonTesterInitialize("
                      "kVariantBeacon, bootStatusBits(bits, sizeof bits)); return;") == 1);
}

// ── COMP-t's review (2026-09-29): the program code that FEEDS and ACTS ON the safety rules ──────

// Bug caught (the review's P1-P6, each of which SURVIVED the whole suite and the gate): the RUN
// gate's inputs forged in readFacts — `f.bootFromRecord = true` (D9 bypassed: a typed-table boot
// could drive the 40 in toward the operator), `f.competitionStatus = 0` (RUN under a field or a
// switch), `f.controllerConnected = true` (RUN without the abort's hands); the chain proof's refusal
// made unreachable (`&& st.runs < 0`: R13's "refused, not warned" as strong as an unchecked if);
// RESULT's finger-down wait dropped (`showResult(rec, …, false)`); the painter's full repaint on a
// button change dropped (a touch tested against a frame not on the screen). runAllowed() and
// pacerChainProven() are host-tested; the program lines that feed them are not host-compiled
// (pros/screen.h), so they are pinned WHOLE here, the way the two PROS reads are.
TEST_CASE("pins: readFacts, the painter's repaint rule, the chain proof's refusal, RESULT's finger") {
    const std::string code = stripComments(readFile(kProgramFile));
    CHECK(squash(bodyOf(code, "TesterFacts readFacts() {"))
          == "TesterState& st = state(); const robot::Robot& r = *st.robot; TesterFacts f; f.now = "
             "r.context().clock().now().value(); f.estimate = r.localizer().pose(); f.estimateLive = "
             "r.localizer().qualityClass() != localization::Localizer::Quality::Uninitialized; "
             "f.controllerConnected = st.master->isConnected(); f.competitionStatus = "
             "pros::c::competition_get_status(); f.abortCause = st.watch->cause(); f.sdLogging = "
             "st.sdLog.has_value(); f.bootFromRecord = st.boot.source == "
             "calibration::BootSource::UseRecord; f.faultCount = r.faults().faultCount(); "
             "f.lastFault = r.faults().lastFault(); f.buildStamp = kBuildTimeText.data(); return f;");
    CHECK(count(code, ".bootFromRecord") == 1);  // set in readFacts, and nowhere else
    CHECK(count(code, ".competitionStatus") == 1);
    CHECK(count(code, ".controllerConnected") == 1);

    CHECK(squash(bodyOf(code, "void ScreenPacer::pace() {"))
          == "if (enabled_) { const TesterFacts f = readFacts(); const TesterView next = "
             "state().screen->view(f); if (!paintedOnce_ || !sameButtons(next, painted_)) { "
             "paintFrame(next); painted_ = next; paintedAt_ = f.now; paintedOnce_ = true; } else if "
             "(f.now - paintedAt_ >= kLiveRepaintSeconds) { paintText(painted_, next); painted_ = "
             "next; paintedAt_ = f.now; } } inner_->pace();");

    const std::string init = squash(bodyOf(code, "void autonTesterInitialize("));
    CHECK(count(init, "if (!pacerChainProven(st.robot->chassis(), *st.guard, *st.watch)) { "
                      "unbuild(); refuse(\"the tester's pacer chain is WRONG\", ") == 1);
    const std::size_t proof = init.find("if (!pacerChainProven(");
    REQUIRE(proof != std::string::npos);
    const std::size_t proofEnd = init.find("} } catch (const PreconditionError& e) {", proof);
    REQUIRE(proofEnd != std::string::npos);
    const std::string refusal = init.substr(proof, proofEnd - proof);
    CHECK(refusal.size() - refusal.rfind("); return;") == std::string{"); return;"}.size() + 1);

    const std::string runOne = squash(bodyOf(code, "void runOne(int index, std::int32_t "));
    CHECK(count(runOne, "const TesterTouch t = readTouch(); "
                        "st.screen->showResult(rec, t.sample.releaseCount, t.fingerDown);") == 1);
    CHECK(count(code, "showResult(") == 1);
}

// Bug caught (the review's M2 and M4, both SURVIVING the suite and a clean package build): the
// Robot's PRESENT mechanisms never reaching the guard — the collection losing its `++` (every
// present mechanism written to slot 0 and the count left 0: the boot line would say "0 of 3" and
// nothing would be safed), or runOne handing runTesterOnce an empty span. The collection is pinned,
// and the span over it appears exactly twice: runOne's run and stopTheTester's last-resort stop.
TEST_CASE("pins: the present mechanisms are collected and handed to the run and the stop") {
    const std::string code = stripComments(readFile(kProgramFile));
    CHECK(count(code, "st.present[st.presentCount++] = m;") == 1);
    CHECK(count(code, "presentCount") == 5);  // the member, the collection, two spans, the boot
    CHECK(count(code, "std::span<hal::IMechanism* const>{st.present.data(), st.presentCount}")
          == 2);
    const std::string runOne = squash(bodyOf(code, "void runOne(int index, std::int32_t "));
    CHECK(count(runOne, "std::span<hal::IMechanism* const>{st.present.data(), st.presentCount}, "
                        "observer, runPressCount);") == 1);
    const std::string stop = squash(bodyOf(code, "void stopTheTester(const char* what) noexcept"));
    CHECK(count(stop, "stopAfterProgramError(st.robot->chassis(), std::span<hal::IMechanism* "
                      "const>{st.present.data(), st.presentCount}, *st.watch);") == 1);
}

// Bug caught (the review, 2026-09-29): a finger that lands between the read whose release touched
// RUN and the watch's arm never aborting — runTesterOnce arms with RUN's own read's press count
// (host-tested, run (15b)), and the program must hand it the count of THAT read, not a later one.
TEST_CASE("pins: RUN's own touch read is the abort's touch baseline") {
    const std::string code = stripComments(readFile(kProgramFile));
    const std::string task = squash(bodyOf(code, "void testerTask(void*"));
    CHECK(count(task, "const TesterTouch t = readTouch(); const TesterAction a = "
                      "st.screen->touch(t.sample, st.painter->drawn(), readFacts()); if (a.kind == "
                      "TesterActionKind::Run) { runOne(a.entry, t.pressCount); }") == 1);
    CHECK(count(code, "runOne(") == 2);  // its definition and that one call
}

// Bug caught (the review, 2026-09-29): the boot line cut past TermSink's 199 bytes at a real build
// hash (the SD path lost). The program prints the three lines testerBootLines() makes (host-tested
// at every field's worst case, run (23b)), and no one-line boot format of its own.
TEST_CASE("pins: the boot lines are testerBootLines', all three logged") {
    const std::string code = stripComments(readFile(kProgramFile));
    CHECK(count(code, "testerBootLines(") == 1);
    CHECK(count(squash(code), "for (const char* bootLine : boot.line) { "
                              "logBoth(hal::LogLevel::Info, bootLine); }") == 1);
    for (const std::string& lit : stringLiterals(code)) {
        CHECK_MESSAGE(lit.find("boot: shulib") == std::string::npos, "a boot line here: ", lit);
        CHECK_MESSAGE(lit.find("boot source") == std::string::npos, "a boot line here: ", lit);
        CHECK_MESSAGE(lit.find("SD run log") == std::string::npos, "an SD boot line here: ", lit);
    }
}
