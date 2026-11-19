// drive_program_source_test.cpp — source pins for the new "shulib Drive" (chunk T4 Part 3,
// 2026-10-01; the T4 brief's P3-T3): what the host cannot compile — the program TU
// (src/programs/drive_program.cpp includes pros/rtos.hpp and reads the competition state) and
// src/main.cpp's cases for it — read as text through SHULIB_SOURCE_DIR, with comments stripped
// before any code pin, so a sentence that mentions a name is never mistaken for a call.
//
// Also the pins that kept the RETIRED "shulib Teleop" retired (its file, its names, its build
// switch, its words), and Part 1b's fold: the buttons left src/robots/mechanism_table.hpp, and the
// driver file is the only place they live.
//
// A pin catches only the spellings it names. What it guards best is what no host test can reach:
// the order of a boot, which object a program's loop is, what is painted and when, and a name that
// must never come back. The helpers are test/source_pins.hpp's (one copy for every program's pins:
// T4 Part 3's late step moved this file's own extras there).

#include "doctest.h"

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

#include "source_pins.hpp"

namespace {

namespace fs = std::filesystem;

using source_pins::count;
using source_pins::readFile;
using source_pins::sourcesUnder;
using source_pins::squeeze;
using source_pins::stringLiterals;
using source_pins::stripComments;

/// The file's code: comments stripped, whitespace squeezed outside literals.
std::string code(const std::string& rel) { return squeeze(stripComments(readFile(rel))); }

/// The body of `signature` in `c`, its own braces left out (source_pins::bodyOf).
std::string bodyOf(const std::string& c, const std::string& signature) {
    return source_pins::bodyOf(c, signature);
}

constexpr const char* kDrive = "src/programs/drive_program.cpp";

}  // namespace

// ═══ The boot: the driver file bound inside the graph's ONE try ═════════════════════════════
TEST_CASE("P3-T3: shulib Drive binds its mechanism service and the driver file ONCE, inside the boot's "
          "try, right after the Robot's emplace — and releases the service FIRST in its catch") {
    // BUG CAUGHT: the bind moved outside the try (a control's precondition would escape
    // initialize()); the service bound over anything but the Robot's own built devices (landmine 20)
    // or not at all (every button would command a slot that does nothing); the driver file bound
    // before the service; a catch that resets the robot while the service still points into it; a
    // second bind somewhere a later driver period would re-run; a robot the driver file's controls
    // are not for said as "the library graph REFUSED to build" (MECH-A5's review: it is the driver
    // file's own refusal, T4's headline — Station F's table — its reason naming the mechanism).
    // (Re-pinned at the T4-3 review: every refusal after the provider goes through refuseBuilt(),
    // which releases the service FIRST since MECH-A5 — pinned below.)
    const std::string c = code(kDrive);
    const std::string boot = bodyOf(c, "void driveInitialize(");
    CHECK(count(c, "st.driver.bind(") == 1U);
    CHECK(count(c, "->bind(") == 1U);
    CHECK(count(boot,
                "try { st.devices.emplace(kLogCopyFile); const robot::BuildVerdict verdict = "
                "robot::validate(st.boot.built.config, *st.devices, st.boot.built.buildOptions()); "
                "if (!verdict.ok) { refuseBuilt(\"the library graph REFUSED to build\", "
                "verdict.why); return; }")
          == 1U);
    CHECK(count(boot,
                "st.robot.emplace(st.boot.built.config, *st.devices, st.boot.built.buildOptions()); "
                "st.mechanisms.emplace(st.devices->pacer()); st.mechanisms->bind(kMechanisms, "
                "st.robot->mechanismParts().devices(), st.robot->context().clock(), st.robot->faults(), "
                "st.robot->context().telemetry()); char why[192] = \"\"; if (!st.driver.bind(*st.mechanisms, "
                "why, sizeof why)) { refuseBuilt(\"the driver file REFUSED this robot\", why); return; } } "
                "catch (const PreconditionError& e) { refuseBuilt(\"the library graph REFUSED to build\", "
                "e.what()); return; }")
          == 1U);
    CHECK(count(boot, "try {") == 1U);
    CHECK(count(boot, "catch (") == 1U);
    // The fault latch, the session header and the boot lines come after the try, in that order.
    const std::size_t latch = boot.find("attachFaultLatch(&r.faults());");
    const std::size_t header = boot.find("diag::emitSessionHeader(log, session, ");
    const std::size_t lines = boot.find("logBootLines(r, log, st.boot, *st.devices, *st.mechanisms);");
    REQUIRE(latch != std::string::npos);
    REQUIRE(header != std::string::npos);
    REQUIRE(lines != std::string::npos);
    CHECK(boot.find("} catch (") < latch);
    CHECK(latch < header);
    CHECK(header < lines);
    CHECK(count(c, "logBootLines(r, log, st.boot, *st.devices, *st.mechanisms);") == 1U);
}

// ═══ The loop: the driver file's run(), and nothing else ════════════════════════════════════
TEST_CASE("P3-T3: shulib Drive drives ONLY through the driver file's run(), once, in runDrive(); "
          "the refused path idles; autonomous commands nothing") {
    // BUG CAUGHT: the program calling the library's loop directly (its own config, its own
    // bindings — the driver file bypassed); a refused boot that falls through into run() over an
    // empty optional; an autonomous that reaches the chassis.
    const std::string c = code(kDrive);
    CHECK(count(c, "st.driver.run(") == 1U);
    CHECK(count(c, "runDriverControl(") == 0U);
    const std::string run = bodyOf(c, "void runDrive()");
    CHECK(count(run,
                "st.driver.run(r, *st.mechanisms, st.devices->pacer(), [] { return "
                "pros::c::competition_is_disabled() != 0; }, screen);")
          == 1U);
    // The refused path: paints the refusal again (its serial line says why once more) and idles,
    // before anything reads the robot.
    CHECK(count(run, "if (!st.robot.has_value()) { paintRefusal(); while (true) { "
                     "pros::delay(250); } }")
          == 1U);
    const std::size_t refused = run.find("if (!st.robot.has_value()) {");
    const std::size_t idle = run.find("while (true) { pros::delay(250); } }");
    const std::size_t deref = run.find("robot::Robot& r = *st.robot;");
    REQUIRE(refused != std::string::npos);
    REQUIRE(idle != std::string::npos);
    REQUIRE(deref != std::string::npos);
    CHECK(refused < idle);
    CHECK(idle < deref);
    // competition_is_disabled appears ONLY in the disabled lambda.
    CHECK(count(c, "competition_is_disabled") == 1U);
    // Autonomous commands nothing.
    const std::string auton = bodyOf(c, "void driveAutonomous()");
    CHECK(count(auton, "chassis()") == 0U);
    CHECK(count(auton, ".run(") == 0U);
    CHECK(count(auton, "kDriveAutonomousReason") == 1U);
}

// ═══ None of the old program, and none of Teleop's, survives by accident ═══════════════════════
TEST_CASE("P3-T3: none of the OLD shulib Drive survives — no stick mapping, no bare adapter, no SD "
          "sink of its own: the SD copy is the provider's, reached by the bare name drive_log.txt") {
    // BUG CAUGHT: the replaced program's open-loop path or its own log file kept alongside the new
    // one (two writers of drive_log.txt; a stick that bypasses the library).
    const std::string c = code(kDrive);
    for (const char* gone : {"mapSticks(", "tankSideVolts(", "ProsMotor", "ProsBlockSink",
                             "clawBannerLine(", "#include \"robots/chassis_table.hpp\"",
                             "kChassis"}) {
        CAPTURE(gone);
        CHECK(count(c, gone) == 0U);
    }
    CHECK(count(c, "constexpr const char* kLogCopyFile = \"drive_log.txt\";") == 1U);
    CHECK(count(c, "st.devices.emplace(kLogCopyFile);") == 1U);
    CHECK(count(c, "drive_log") == 1U);  // the name is said once, bare (the provider adds /usd/)
    CHECK(count(c, "/usd/") == 0U);
}

TEST_CASE("P3-T3: shulib Drive prints each mechanism's boot lines from the build's list, the bound "
          "service AND the driver file's controls, and its exit line names what the disable did to the "
          "claws") {
    // BUG CAUGHT (test 1l's, moved from "shulib Teleop"; the mechanism core's words since MECH-A5): a
    // boot block handed another list, no controls (no control line), or another sink than the
    // provider's (the SD copy would miss it); each line's level dropped (an ABSENT one would read as
    // fine).
    const std::string c = code(kDrive);
    CHECK(count(c,
                "mechanismBootLines(kMechanisms, r, mechanisms, &ThisRobotDriver::kControls, [&log](const "
                "MechanismLine& m) { log.log(m.level, \"MECH\", m.text); });")
          == 1U);
    CHECK(count(c, "mechanismBootLines(") == 1U);
    CHECK(count(c, "ClawBootLine(") == 0U);
    // Robot two's sentence VERBATIM, chosen on its claw words; robot one's (no claws) beside it (the review of
    // ROB1-A — test/this_robot_test.cpp walks every such choice's order); the pin starts at the call, so
    // a `!` before the condition breaks it (the verifier of ROB1-A, F10b).
    CHECK(count(c,
                "std::snprintf(line, sizeof line, ThisRobotWords::value.claw ? "
                "\"opcontrol(): field DISABLED -- the driver file's loop exited cleanly after \" "
                "\"cancel() (0 V + Brake); each claw pivot made safe, the pneumatic claw left as it \" "
                "\"was; %lu screen prints so far\" : "
                "\"opcontrol(): field DISABLED -- the driver file's loop exited cleanly after \" "
                "\"cancel() (0 V + Brake); each motor mechanism made safe, each pneumatic left as it \" "
                "\"was; %lu screen prints so far\", static_cast<unsigned long>(g_screenPrints));")
          == 1U);
    // The boot source hands the build's mechanism table to the decision (test 1l's last pin).
    CHECK(count(code("src/common/boot_source.cpp"),
                "decideBootSourceFrom( record, registry, fromTable, missing, kMechanisms, "
                "kChassis.robot,")
          == 1U);
}

// ═══ src/main.cpp ═══════════════════════════════════════════════════════════════════════════
TEST_CASE("P3-T3: src/main.cpp's Drive cases call the new program's four entry points, and nothing "
          "of the retired Teleop is left in it") {
    // BUG CAUGHT: a leftover #include of the deleted program (the build breaks — or, with a stale
    // header left behind, links the old program); the Drive case's autonomous painting the tester's
    // sentence; a case calling the wrong program.
    const std::string c = code("src/main.cpp");
    CHECK(count(c, "programs/teleop_program.hpp") == 0U);
    CHECK(count(c, "#include \"programs/drive_program.hpp\"") == 1U);
    CHECK(count(c, "case Program::Drive: shulib::bench::driveInitialize(kVariantBeacon, "
                   "bootStatusBits(bits, sizeof bits)); return;")
          == 1U);
    CHECK(count(c, "case Program::Drive: shulib::bench::benchStateScreen(\"AUTONOMOUS\", "
                   "shulib::bench::kDriveAutonomousReason); shulib::bench::driveAutonomous(); "
                   "return;")
          == 1U);
    CHECK(count(c, "case Program::Drive: shulib::bench::runDrive(); return;") == 1U);
    CHECK(count(c, "Program::Teleop") == 0U);
}

// ═══ The retired program stays retired ══════════════════════════════════════════════════════
TEST_CASE("P3-T3: \"shulib Teleop\" is retired everywhere — its files are gone, its names appear in "
          "no code under src/ or include/shulib/, and no string literal there says \"shulib "
          "Teleop\" or \"(slot 2)\"") {
    // BUG CAUGHT: a stale copy of the deleted program, a call to its entry points, its build switch
    // read anywhere, or a runtime sentence still sending a driver to slot 2 (Calibrate's save line,
    // the X-drive's refusal, a library header's words).
    const fs::path root{SHULIB_SOURCE_DIR};
    CHECK_FALSE(fs::exists(root / "src/programs/teleop_program.cpp"));
    CHECK_FALSE(fs::exists(root / "src/programs/teleop_program.hpp"));
    // The needles are built from pieces, so this file never matches itself.
    const std::vector<std::string> names = {
        std::string{"teleop"} + "_program", std::string{"Teleop"} + "Claws",
        std::string{"run"} + "Teleop", std::string{"teleop"} + "Initialize",
        std::string{"teleop"} + "Autonomous", std::string{"kTeleop"} + "AutonomousReason",
        std::string{"SHULIB_PROGRAM_"} + "LIBRARY"};
    const std::string teleopWords = std::string{"shulib "} + "Teleop";
    const std::string slotTwo = std::string{"(slot "} + "2)";
    std::size_t files = 0;
    for (const char* dir : {"src", "include/shulib"}) {
        for (const std::string& rel : sourcesUnder(dir)) {
            ++files;
            const std::string c = stripComments(readFile(rel));
            for (const std::string& name : names) {
                CAPTURE(rel);
                CAPTURE(name);
                CHECK(count(c, name) == 0U);
            }
            for (const std::string& lit : stringLiterals(c)) {
                CAPTURE(rel);
                CAPTURE(lit);
                CHECK(lit.find(teleopWords) == std::string::npos);
                CHECK(lit.find(slotTwo) == std::string::npos);
            }
        }
    }
    CHECK(files > 50U);  // the walk really read the tree
}

// ═══ Part 1b, folded: the buttons live in the driver file and nowhere else ═══════════════════
TEST_CASE("P3-T3: the buttons left the mechanism table — kTeleopRezeroButton, controlsMatchTable and "
          "\"until T4 Part 1b\" appear in no code under src/ or test/, and no src/robots/ table sets "
          ".forward, .reverse, .volts or .toggle") {
    // BUG CAUGHT: the table's copy of the buttons (or the re-zero) left behind, so a button changed
    // in the driver file and not the table would leave two answers to "which button does what" —
    // the tie that kept them equal retired with Teleop, so nothing would catch the drift.
    const std::vector<std::string> gone = {std::string{"kTeleop"} + "RezeroButton",
                                           std::string{"controls"} + "MatchTable",
                                           std::string{"until T4 "} + "Part 1b"};
    for (const char* dir : {"src", "test"}) {
        for (const std::string& rel : sourcesUnder(dir)) {
            if (rel == "test/drive_program_source_test.cpp") continue;  // this pin names them
            const std::string c = stripComments(readFile(rel));
            for (const std::string& name : gone) {
                CAPTURE(rel);
                CAPTURE(name);
                CHECK(count(c, name) == 0U);
            }
        }
    }
    std::size_t tables = 0;
    for (const std::string& rel : sourcesUnder("src/robots")) {
        const std::string c = squeeze(stripComments(readFile(rel)));
        if (count(c, "MechanismTable") == 0) continue;  // a mechanism table (not the X-drive's wheels)
        ++tables;
        for (const char* field : {".forward =", ".reverse =", ".volts =", ".toggle ="}) {
            CAPTURE(rel);
            CAPTURE(field);
            CHECK(count(c, field) == 0U);
        }
    }
    CHECK(tables >= 1U);
    // ...and the table's types have no such member: the driver file states them (its controls).
    const std::string table = code("src/robots/mechanism_table.hpp");
    for (const char* member : {"forward;", "reverse;", "volts;", "toggle;",
                               "std::optional<hal::ControllerButton>"}) {
        CAPTURE(member);
        CHECK(count(table, member) == 0U);
    }
    const std::string driver = code("src/team/robot_two_driver.hpp");
    CHECK(count(driver, "inline constexpr DriverControls kControls = controls(kRobotTwoMechanisms,") == 1U);
}

TEST_CASE("P3-T3: the library's loop is called only by the shared driver file and the invented "
          "X-drive; NO src/ file builds T3's superseded bindings any more (MECH-A5)") {
    // BUG CAUGHT: a program that drives robot two through its own call to the loop, or builds its own
    // MechanismBinding / ToggleBinding — robot two's claws driven by a second path beside the mechanism
    // service (the loop refuses the two together, but only a program that hands it both). Until
    // MECH-A5 the one builder was src/common/driver_controls.hpp's binder, and the one caller robot
    // two's driver file itself; the binder retired with that file, and the call is DriverFile's, which
    // every robot's driver file runs (the MECH brief's N6).
    std::vector<std::string> callers;
    std::vector<std::string> builders;
    for (const std::string& rel : sourcesUnder("src")) {
        const std::string c = stripComments(readFile(rel));
        if (count(c, "runDriverControl(") > 0) callers.push_back(rel);
        if (count(c, "MechanismBinding") + count(c, "ToggleBinding") > 0) builders.push_back(rel);
    }
    CHECK(callers == std::vector<std::string>{"src/common/driver_file.hpp", "src/programs/xdrive_program.cpp"});
    CHECK(builders.empty());
}

// ═══ The screen, the page flip and the controller: wired to the SHARED pieces (the late step) ═══
TEST_CASE("P3-T3: shulib Drive paints through the shared painters only — the refusal screen, the "
          "visible page by driveRepaint(), one touch read per tick into the pager") {
    // BUG CAUGHT: a refusal said on serial only (the driver stares at a blank screen), a refusal
    // painted with words of the program's own or the wrong bar word; a page painted whole every
    // tick (PROS leaks every print) or a row repainted though it did not change; a print that
    // bypasses printText() (a '%' re-read as a format); the touch read widened past one line, or
    // the pager fed something but the release-edge sample and the press count; a page flip that
    // waits for the next 200 ms refresh.
    const std::string c = code(kDrive);
    CHECK(count(bodyOf(c, "void paintRefusal(bool serialLine = true)"),
                "paintRefusalScreen(RefusalScreen{.tag = kTag, .barWord = kDriveRefusedWord.data(), "
                ".buildTime = kBuildTimeText.data(), .headline = st.refusalHeadline, .why = "
                "st.refusalWhy, .footer = kDriveRefusalFooter, .logToSerial = serialLine});")
          == 1U);
    CHECK(count(c, "paintRefusalScreen(") == 1U);
    CHECK(count(c, "paintRefusal();") == 2U);  // the boot's refuse() and the refused opcontrol()
    CHECK(count(bodyOf(c, "void refuse(const char* headline, const char* why)"),
                "paintRefusal();")
          == 1U);
    CHECK(source_pins::squash(bodyOf(c, "ScreenTouch readTouch()"))
          == "return screenTouchFrom(pros::c::screen_touch_status(), "
             "static_cast<int>(pros::E_TOUCH_ERROR));");
    CHECK(count(c, "screen_touch_status") == 1U);
    CHECK(count(c, "readTouch()") == 3U);  // its definition, each tick's read, the pager's start
    CHECK(count(c, "DriveScreen screen{r, st.boot, st.driver, *st.mechanisms, *st.lcd, readTouch()};") == 1U);
    CHECK(count(c, "printText(") == 0U);
    CHECK(count(c, "screen_erase") == 0U);
    CHECK(count(c, "screen_print") == 0U);
    // The tick: the touch every tick (a flip repaints at once), the rows and the paint on the first
    // tick, a flip and every kRefreshTicks, the controller every kLcdEveryTicks.
    const std::string tick = bodyOf(c, "void operator()(const math::ChassisSpeeds& command,");
    CHECK(source_pins::squash(tick)
          == "++tick_; const double now = r_.context().clock().now().value(); const ScreenTouch "
             "touch = readTouch(); const bool flipped = pager_.touch(touch.sample, "
             "touch.pressCount, now); if (tick_ == 1 || flipped || tick_ % kRefreshTicks == 0) { "
             "refreshRows(command, connected, shaper, protection); "
             "paint(driveView(pager_.page(), rows_, pager_.armed(now))); } if (tick_ % "
             "kLcdEveryTicks == 1) { refreshLcd(connected, protection, now); }");
    CHECK(count(c, "static constexpr int kRefreshTicks = 20;") == 1U);
    CHECK(count(c, "static constexpr int kLcdEveryTicks = 5;") == 1U);
    // The paint follows driveRepaint()'s plan: whole only when it says so, else only its marks.
    const std::string paint = bodyOf(c, "void paint(const DriveView& next)");
    CHECK(count(paint, "const DriveRepaint plan = driveRepaint(painted_, next, havePainted_);") == 1U);
    CHECK(count(paint, "if (plan.whole) { clearPanel();") == 1U);
    CHECK(count(paint, "clearPanel()") == 1U);
    CHECK(count(paint, "if ((plan.lines & (1u << i)) == 0) { continue; }") == 1U);
    CHECK(count(paint, "if ((plan.buttons & (1u << i)) != 0) {") == 1U);
    CHECK(count(paint, "painted_ = next; havePainted_ = true;") == 1U);
    CHECK(count(c, "clearPanel()") == 1U);
    CHECK(count(c, "driveRepaint(") == 1U);
}

TEST_CASE("P3-T3: shulib Drive's MAIN rows and controller lines are the shared panel's words, with "
          "the protections' state; the cut length is the driver file's config") {
    // BUG CAUGHT: the PROTECTION row or the controller row worded by the program (the competition
    // program would say something else); the controller's lines chosen outside the ONE chooser or
    // written in a burst (PROS drops all but the first); the protections' REFUSED / CUT / dead
    // words never reaching the controller; a cut length retyped.
    const std::string c = code(kDrive);
    // (The PROTECTION and controller rows are made by drive_panel.hpp's driveLiveRows() from the
    // shared panel's words — pinned there and in the refreshRows() pin above; here, the cut length
    // handed to it is the config's.)
    CHECK(count(c, "f.cutSeconds = kCutSeconds;") == 1U);
    CHECK(count(c, "constexpr double kCutSeconds = ThisRobotDriver::kLoop.protection.cutTicks * "
                   "ThisRobotDriver::kLoop.tickPeriod.value();")
          == 1U);
    const std::string lcd = bodyOf(c, "void refreshLcd(bool connected,");
    CHECK(source_pins::squash(lcd)
          == "ControllerFacts f{}; f.batteryVolts = r_.context().battery().voltage().value(); "
             "f.headingDegrees = r_.localizer().pose().heading().radians() * 180.0 / "
             "math::Angle::kPi; f.hasFault = r_.faults().hasFault(); f.lastFault = "
             "r_.faults().lastFault(); f.connected = connected; f.degraded = "
             "boot_.verdict.decision == calibration::BootDecision::Degraded; f.clawAbsent = "
             "anyClawAbsent(r_); addProtectionFacts(f, protection); const ControllerScreen want = "
             "ThisRobotWords::value.claw ? controllerScreenLines(f) : controllerScreenLines(f, ThisRobotWords{}); "
             "const int row = lcdWriter_.next(want, now); if (row >= 0) { "
             "lcd_.setLine(row, want.line[row]); }");
    CHECK(count(c, "setLine(") == 1U);
    // ROB1-A: robot two's claw-word form and the robot's-words form, chosen on the build's robot (T6).
    CHECK(count(c, "controllerScreenLines(") == 2U);
    CHECK(count(c, "controllerScreenLines(f, ThisRobotWords{})") == 1U);
    CHECK(count(c, "ControllerLineWriter lcdWriter_{};") == 1U);
    CHECK(count(c, "st.lcd.emplace();") == 1U);
    // Every word on these rows is the shared header's: no literal here says what they say.
    for (const std::string& lit : stringLiterals(c)) {
        CAPTURE(lit);
        for (const char* shared : {"drive OK", "REFUSED see brain", "CUT see brain", "CONNECTED",
                                   "NO CONTROLLER", "DEGRADED see brain", "CLAW ABSENT", "MECH ABSENT"}) {
            CHECK(lit.find(shared) == std::string::npos);
        }
    }
}

TEST_CASE("P3-T3: shulib Drive says where its log copy is ONCE, in the words both programs share") {
    // BUG CAUGHT: a copy that never opened believed recorded, or the sentence typed twice (the
    // competition program's would drift from this one's).
    const std::string c = code(kDrive);
    CHECK(count(c, "#include \"common/log_copy_line.hpp\"") == 1U);
    // (Re-pinned at the T4-3 review: the card is asked whether it is in, so a copy that failed to
    // open says NO SD CARD only when there is none; `false, false`: a Truncate copy.)
    CHECK(count(c, "const LogCopyLine copy = logCopyBootLine( logCopyOpenFrom(devices.serialCopyOpen(), "
                   "pros::c::usd_is_installed() != 0, false, false), devices.serialCopyPath(), "
                   "devices.serialCopyDroppedLines()); log.log(copy.warn ? hal::LogLevel::Warn : "
                   "hal::LogLevel::Info, kTag, copy.text);")
          == 1U);
    CHECK(count(c, "usd_is_installed") == 1U);
    CHECK(count(c, "logCopyBootLine(") == 1U);
    for (const std::string& lit : stringLiterals(c)) {
        CAPTURE(lit);
        CHECK(lit.find("log copy") == std::string::npos);
    }
}

// ═══ The T4-3 review's findings: a refusal reaches the card; the glue is the pure model's ═══════
TEST_CASE("P3-T3 (T4-3): a refusal decided after the provider is said THROUGH it first — the SD "
          "copy, truncated by this boot, holds why — then the graph released, the service first") {
    // BUG CAUGHT (the review's finding: a refused boot left an EMPTY drive_log.txt and the reason on
    // serial only): the refusal logged after the provider is gone (or never); the provider released
    // before the Robot, or the Robot before the mechanism service that points into it (MECH-A5: a
    // dangling borrow); the serial line said twice.
    const std::string c = code(kDrive);
    CHECK(source_pins::squash(bodyOf(c, "void refuseBuilt(const char* headline, const char* why)"))
          == "DriveState& st = state(); std::snprintf(st.refusalHeadline, sizeof "
             "st.refusalHeadline, \"%.63s\", headline); std::snprintf(st.refusalWhy, sizeof "
             "st.refusalWhy, \"%.191s\", why); const bool logged = st.devices.has_value(); if "
             "(logged) { logBootRefusal(st.devices->telemetry(), kTag, st.refusalHeadline, "
             "st.refusalWhy); } st.mechanisms.reset(); st.robot.reset(); st.devices.reset(); "
             "paintRefusal(!logged);");
    CHECK(count(c, "refuseBuilt(") == 4U);  // its definition and the three refusals after the provider
                                            // (the driver file's its own again: MECH-A5's review)
    CHECK(count(c, "logBootRefusal(") == 1U);
    // The serial line is left to the sink ONLY when the sink said it: `logged` is read BEFORE the
    // provider is released (BUG CAUGHT, the T4-3 verification: a refusal with no provider painted
    // with its serial line off — said nowhere on serial; and read after the release it would be
    // false always, saying every refusal on serial twice).
    CHECK(count(c, "paintRefusal(!logged);") == 1U);
    CHECK(count(c, "paintRefusal(false);") == 0U);
    CHECK(count(bodyOf(c, "void paintRefusal(bool serialLine = true)"), ".logToSerial = serialLine")
          == 1U);
    // The boot source's refusal (no provider yet) is the only one that does not use it.
    CHECK(count(bodyOf(c, "void driveInitialize("), "refuse(st.boot.headline, st.boot.why);") == 1U);
}

TEST_CASE("P3-T3 (T4-3): the rows, the repaint and the boot's generated words come from "
          "drive_panel.hpp's pure functions, handed the right facts") {
    // BUG CAUGHT (the review's RS1-RS10 survived as edits to the program's own glue): a fact read
    // into the wrong field (the RIGHT group for the LEFT row, a fault count while nothing is
    // latched), the source rows handed the wrong boot facts, a partial repaint that does not use
    // driveLineToPaint(), a level or a generated line typed in the program instead of the model's.
    const std::string c = code(kDrive);
    CHECK(source_pins::squash(bodyOf(c, "void refreshRows(const math::ChassisSpeeds& command,"))
          == "const math::Pose2d pose = r_.localizer().pose(); const "
             "localization::DriveEncoderOdometry* odom = r_.driveOdometry(); const "
             "diag::FaultLatch& faults = r_.faults(); const bool freed = "
             "driver_.releasedLeftoverAtEntry(mechanisms_); const ClawRow claw = "
             "ThisRobotWords::value.claw ? clawRowText(kMechanisms, r_, mechanisms_, freed) : "
             "clawRowText(kMechanisms, r_, mechanisms_, freed, ThisRobotWords{}); DriveLiveFacts f{}; "
             "f.cutSeconds = kCutSeconds; "
             "f.connected = connected; f.cmdVx = command.vx().value(); f.cmdOmega = "
             "command.omega().value(); f.estX = pose.x().value(); f.estY = pose.y().value(); "
             "f.estHeadingDeg = pose.heading().radians() * 180.0 / math::Angle::kPi; f.quality = "
             "r_.localizer().qualityClass(); f.mode = ThisRobotDriver::kFeel.mode; f.fieldCentric = "
             "shaper.fieldCentric; f.hasOdometry = odom != nullptr; f.headingDisagreement = odom "
             "!= nullptr ? odom->lastHeadingDisagreement() : 0.0; f.batteryVolts = "
             "r_.context().battery().voltage().value(); f.travelLeft = odom != nullptr ? "
             "odom->lastSideTravel().left.value() : 0.0; f.travelRight = odom != nullptr ? "
             "odom->lastSideTravel().right.value() : 0.0; f.faultCount = faults.hasFault() ? "
             "faults.faultCount() : 0; f.firstFault = faults.firstFault(); f.lastFault = "
             "faults.lastFault(); f.clawText = claw.text; f.clawAbsent = anyClawAbsent(r_); "
             "f.clawFreed = freed; driveLiveRows(rows_, f, protection, *left_, *right_);");
    CHECK(count(c, "left_ = groups[0]; right_ = groups[1];") == 1U);
    CHECK(count(c,
                "driveSourceRows(rows_, boot_.source == calibration::BootSource::UseTable, "
                "boot_.verdict.decision == calibration::BootDecision::Degraded, "
                "boot_.verdict.reason);")
          == 1U);
    const std::string paint = bodyOf(c, "void paint(const DriveView& next)");
    CHECK(count(paint, "paintLine(driveLineToPaint(painted_, next, i), kPanelBg);") == 1U);
    CHECK(count(paint, "text[0]") == 0U);
    const std::string boot = bodyOf(c, "void logBootLines(");
    // ROB1-C2: robot one's driver line says its map is the BUILDER's, the team's (ROB1-A's said "a DEFAULT -- the
    // team's to change" until the builder gave the map on 2026-11-15). The pin starts at the choice, so a negated
    // condition or the old words break it (the review of ROB1-C2: RV7, the string reverted, survived the suite).
    CHECK(count(boot,
                "std::snprintf(line, sizeof line, kRobotOneBuild ? "
                "\"driver control: robot one's driver file (src/team/robot_one_driver.hpp, the builder's \" "
                "\"map -- the team's) -> runDriverControl, its feel (the next line); re-zero button \" "
                "\"%.5s; mode %.12s\" : ")
          == 1U);
    CHECK(count(c, "a DEFAULT") == 0U);
    // RMP1-B: the feel's line is MADE from the driver file's kFeel, right after the driver line -- never typed,
    // never pinned to the library's default (a team's feel would stop every build).
    CHECK(count(boot, "logWrapped(log, hal::LogLevel::Info, line); feelLine(line, sizeof line, ThisRobotDriver::kFeel); "
                      "logWrapped(log, hal::LogLevel::Info, line);") == 1U);
    CHECK(count(c, "deadband 0.05, no expo") == 0U);
    CHECK(count(boot,
                "log.log(driveBootLevel(boot.source == calibration::BootSource::UseRecord), kTag, "
                "line);")
          == 1U);
    CHECK(count(boot,
                "logWrapped(log, driveBootLevel(boot.verdict.decision == "
                "calibration::BootDecision::Ok), check);")
          == 1U);
    CHECK(count(boot,
                "if (driveDegradedBootLine(line, sizeof line, boot.built.absent().size(), "
                "boot.verdict.reason)) { logWrapped(log, hal::LogLevel::Warn, line); }")
          == 1U);
    CHECK(count(boot,
                "(void)driveProtectionsLine(protections, sizeof protections, "
                "ThisRobotDriver::kLoop.protection, ThisRobotDriver::kLoop.tickPeriod.value(), "
                "teleop::SideMonitorConfig{}.persistTicks); logWrapped(log, hal::LogLevel::Info, "
                "protections);")
          == 1U);
    // None of the moved decisions is still typed in the program.
    CHECK(count(c, "driveRow(") == 1U);  // the run status row, set once in the constructor
    for (const char* moved : {"faultRowText(", "protectionRowText(", "controllerRowText(",
                              "driveGroupRow(", "sourceRows(", "\"DEGRADED: %u recorded",
                              "\"drive protections (the library's"}) {
        CAPTURE(moved);
        CHECK(count(c, moved) == 0U);
    }
}
