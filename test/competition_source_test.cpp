// competition_source_test.cpp — source pins for what the host cannot compile: "shulib
// Competition"'s program TU (src/programs/competition_program.cpp includes pros/screen.h) and
// src/main.cpp's cases for it (chunk COMP-b, 2026-09-30; the COMP brief's B2, B8, B9, B11, B19,
// B20, B30 and its re-check's RC1, RC3, RC7, ND1, ND2, ND5). Read through SHULIB_SOURCE_DIR with
// test/source_pins.hpp's helpers; comments are stripped before any code pin.
//
// A pin catches only the spellings it names. The program's DECISIONS are host-tested where they
// live (competition_run.hpp, competition_watch.hpp, competition_screen.hpp; the boot's chain proof,
// src/common/guarded_run.hpp). What only a pin can guard is the program code that FEEDS those
// decisions and ACTS on them — the PROS reads, the order of the boot, the objects' wiring, the
// auton task's loop, which task paints, what each log line reads and where it goes, the driver
// page's and the controller's facts. So every function of the program file that does one of those
// is pinned here WHOLE (P4, P6, P7, P14) or by its order (P2, P8); the boot's body and the
// entry points' simple forwarding by the statements that matter (P2, P12). COMP-b's review
// (2026-10-01) found seven mutations of the program file that the earlier, partial pins let
// through (ND1's and ND2's menu, the SD card's one writer, the AUTON row, an idle-half throw); P14
// is the fix, and P15 pins the one ORDER in competition_run.hpp a single-threaded host test cannot
// observe (the END estimate read before FINISHED). P16 pins the other thing no single-threaded test
// can observe: the store-then-load handshake between the opcontrol task and the auton task (the
// verification round, 2026-10-01: three mutations of it survived the suite).

#include "doctest.h"

#include <cstddef>
#include <regex>
#include <string>

#include "source_pins.hpp"

namespace {

using source_pins::bodyOf;
using source_pins::count;
using source_pins::readFile;
using source_pins::squash;
using source_pins::stringLiterals;
using source_pins::stripComments;

const std::string kProgramFile = "src/programs/competition_program.cpp";

std::string program() { return stripComments(readFile(kProgramFile)); }

}  // namespace

// Bug caught: the program running an auton around the tested request — runAuton or the serve
// called directly (the report and its after-FINISHED order, ND5, would be bypassed, and the host
// tests would test a request the robot never runs), or the team's autons registered twice.
TEST_CASE("pins (P1): the program runs autons ONLY through runAutonRequest") {
    const std::string code = program();
    CHECK(count(code, "runAutonRequest(") == 1);
    CHECK(count(code, "serveAutonRequest(") == 0);
    CHECK(count(code, "reportAutonRun(") == 0);
    CHECK(count(code, "runAuton(") == 0);
    // ROB1-A: the build's robot's autons, through src/common/this_robot.hpp — once — and never a
    // robot's own registration directly.
    CHECK(count(code, "registerThisRobotsAutons(") == 1);
    CHECK(count(code, "registerAutons(") == 0);
    CHECK(count(code, "registerRobotOneAutons(") == 0);
}

// Bug caught (B7, B8, B11, B30; the mechanism service and the route registry since MECH-A5): the
// Robot built WITHOUT the options that carry the guard (no disable could end an auton), the chain in
// the wrong order (Chassis -> guard -> MECHANISM SERVICE -> watch -> the provider's pacer: a service
// left out ticks no mechanism in an auton — the MECH brief's P1 control case; one constructed after
// the Robot cannot be in its chain — landmine 1), the service bound outside the try, over anything
// but the Robot's own devices (landmine 20), or after the proof (an unbound service ticks nothing);
// the route ids registered before the bind or never (M47: every route marker an unknown id); the
// watch never attached; the chain proof outside the try or after the task (a failed proof would not
// refuse); the driver file bound outside the try or before the service; the unbuild in an order that
// leaves something pointing at what it released; or the task started before the fault latch is
// attached.
TEST_CASE("pins (P2): the boot — B8's order, the chain, the service, the route ids, the proof and the "
          "binding in the ONE try") {
    const std::string code = program();
    CHECK(count(code, "st.watch.emplace(st.devices->pacer(), *st.status);") == 1);
    CHECK(count(code, "st.mechanisms.emplace(*st.watch);") == 1);
    CHECK(count(code, "st.guard.emplace(*st.mechanisms);") == 1);
    CHECK(count(code, "st.guard.emplace(") == 1);
    CHECK(count(code, "st.mechanisms.emplace(") == 1);
    CHECK(count(squash(code), "st.mechanisms->bind(kMechanisms, st.robot->mechanismParts().devices(), "
                              "st.robot->context().clock(), st.robot->faults(), st.robot->context().telemetry());")
          == 1);
    CHECK(count(code, "->bind(") == 1);
    CHECK(count(code, "st.driver.bind(") == 1);
    CHECK(count(code, "st.commands.emplace();") == 1);
    CHECK(count(code, "st.routes.emplace(st.robot->chassis(), *st.mechanisms, kMechanisms, "
                      "st.devices->telemetry());") == 1);
    CHECK(count(code, "st.routes->registerInto(*st.commands);") == 1);
    CHECK(count(code, "registerInto(") == 1);
    CHECK(count(code, "st.watch->attachGuard(*st.guard);") == 1);
    CHECK(count(code, "attachGuard(") == 1);
    CHECK(count(code, "robot::BuildOptions opts = st.boot.built.buildOptions();") == 1);
    CHECK(count(code, "opts.pacer = &*st.guard;") == 1);
    CHECK(count(code, ".pacer =") == 1);
    CHECK(count(code, "robot::validate(st.boot.built.config, *st.devices, opts)") == 1);
    CHECK(count(code, "st.robot.emplace(st.boot.built.config, *st.devices, opts);") == 1);
    CHECK(count(code, "st.robot.emplace(") == 1);
    CHECK(count(code, "pacerChainMissing(") == 1);
    CHECK(count(code, "pacerChainProven(") == 0);

    const std::string init = squash(bodyOf(code, "void competitionInitialize("));
    // T4 Part 3 (Decision B): the provider is built WITH the log copy, appended; re-pointed
    // one-for-one from "try { st.devices.emplace();".
    const std::size_t tryAt =
        init.find("try { st.devices.emplace(kLogCopyFile, hal::pros::ProsFileMode::Append);");
    const std::size_t serviceAt = init.find("st.mechanisms.emplace(*st.watch);");
    const std::size_t robotAt = init.find("st.robot.emplace(");
    const std::size_t serviceBindAt = init.find("st.mechanisms->bind(");
    const std::size_t bindAt = init.find("char why[192] = \"\"; if (!st.driver.bind(*st.mechanisms, why, sizeof why)) "
                                         "{ refuseBuilt(\"the driver's controls REFUSED\", why); return; }");
    const std::size_t routesAt = init.find("st.routes->registerInto(*st.commands);");
    // T4 Part 3 (the team lead's C5 ruling, 2026-10-02): the registry DROPS a refused entry --
    // built so right before the team's registration, inside the try.
    const std::size_t registerAt = init.find(
        "st.registry.emplace(kAutonPeriods, competition::OnRefusal::DropEntry); "
        "registerThisRobotsAutons(*st.registry);");
    const std::size_t proofAt = init.find("if (const ChainGap gap = pacerChainMissing(st.robot->chassis(), "
                                          "*st.guard, *st.mechanisms, *st.watch); gap != ChainGap::None) {");
    const std::size_t catchAt = init.find("} catch (const std::exception& e) { refuseBuilt(");
    const std::size_t latchAt = init.find("attachFaultLatch(&st.robot->faults());");
    const std::size_t taskAt = init.find("st.task.emplace(");
    for (const std::size_t at : {tryAt, serviceAt, robotAt, serviceBindAt, bindAt, routesAt, registerAt, proofAt,
                                 catchAt, latchAt, taskAt}) {
        REQUIRE(at != std::string::npos);
    }
    CHECK(tryAt < serviceAt);
    CHECK(serviceAt < robotAt);       // the service in the chain BEFORE the Robot (landmine 1)
    CHECK(robotAt < serviceBindAt);   // bound right after the Robot, inside the try:
    const std::size_t robotEnd = init.find(';', robotAt) + 1;  // nothing between
    CHECK(serviceBindAt == robotEnd + 1);
    CHECK(serviceBindAt < bindAt);    // the driver file bound to the BOUND service (B11)
    CHECK(bindAt < routesAt);         // the route ids after the bind
    CHECK(routesAt < registerAt);
    CHECK(bindAt < registerAt);
    CHECK(registerAt < proofAt);
    CHECK(proofAt < catchAt);  // inside the try: a throw or a false proof refuses the boot
    CHECK(catchAt < latchAt);
    CHECK(latchAt < taskAt);   // the latch attached BEFORE the auton task exists (B8, B19)
    CHECK(count(init, "st.task.emplace(&autonTask, nullptr, TASK_PRIORITY_DEFAULT, "
                      "TASK_STACK_DEPTH_DEFAULT, \"auton task\");") == 1);
    CHECK(count(code, "st.task.emplace(") == 1);
    // Each refusal inside the try un-builds, then refuses, then returns -- a refusal that went on
    // would start a task over a destroyed Robot. (Re-pinned one-for-one at the T4-3 review: each goes
    // through refuseBuilt(), which says the refusal through the provider's sink FIRST -- so the log
    // copy, comp_log.txt, holds why the boot refused -- then un-builds, then paints with no serial
    // line of its own.)
    CHECK(count(init, "refuseBuilt(\"the library graph REFUSED to build\", verdict.why); "
                      "return;") == 1);
    CHECK(count(init, "refuseBuilt(\"the competition program's pacer chain is WRONG\", chain); return;") == 1);
    // The driver file's OWN refusal (T4's headline, kept at MECH-A5's review): a robot its controls
    // are not for is said as the driver's, its reason naming the mechanism — never "the library graph".
    CHECK(count(init, "refuseBuilt(\"the driver's controls REFUSED\", why); return;") == 1);
    CHECK(count(init, "std::snprintf(chain, sizeof chain, \"%.150s -- a build error, not a robot fault\", "
                      "chainGapWords(gap));") == 1);
    CHECK(count(init, "} catch (const std::exception& e) { refuseBuilt(\"the competition "
                      "program REFUSED to start\", e.what()); return; }") == 1);
    // The serial line is left to the sink ONLY when the sink said it (`logged`, read before
    // unbuild() releases the provider -- the T4-3 verification: with no provider the refusal was
    // painted with its serial line off and said nowhere on serial).
    CHECK(count(init, "const auto refuseBuilt = [&st, &unbuild](const char* headline, const char* "
                      "why) { const bool logged = st.devices.has_value(); if (logged) { "
                      "logBootRefusal(st.devices->telemetry(), kTag, headline, why); } unbuild(); "
                      "refuse(headline, why, !logged); };") == 1);
    CHECK(count(code, "refuse(headline, why, false)") == 0);
    CHECK(count(init, "refuseBuilt(") == 4);  // its four refusals (the driver file's its own again: MECH-A5's review)
    CHECK(count(init, "unbuild();") == 1);    // only inside refuseBuilt
    CHECK(count(code, "logBootRefusal(") == 1);
    CHECK(count(squash(bodyOf(code, "void refuse(const char* headline, const char* why, bool "
                                    "serialLine = true)")),
                ".footer = kNothingPowered, .logToSerial = serialLine});") == 1);
    // The unbuild, each object before what its DESTRUCTOR reads: the route ids (they point at the
    // Chassis and the service) first; the SERVICE before the Robot -- ~MechanismService reads every
    // slot's mechanism (holdsClaim()), and the mechanisms, the fault latch and the clock it was bound
    // with are the Robot's (MECH-A5's review: the first order reset the Robot first, so a refused boot
    // ran the service's destructor over destroyed mechanisms); then the Robot, the guard, the watch.
    CHECK(count(squash(code), "const auto unbuild = [&st] { st.menu.reset(); st.registry.reset(); "
                              "st.routes.reset(); st.commands.reset(); st.mechanisms.reset(); st.robot.reset(); "
                              "st.guard.reset(); st.watch.reset(); st.status.reset(); st.devices.reset(); };")
          == 1);
    // ND2: the one place the typed-table fact is set, from the boot's own decision -- and BEFORE
    // the auton task exists (a plain bool written after the task that reads it is running would be
    // a data race, and the first menu tick could read the default, false: every auton refused).
    CHECK(count(code, "st.flags.bootFromRecord = st.boot.source == "
                      "calibration::BootSource::UseRecord;") == 1);
    CHECK(count(code, ".bootFromRecord =") == 1);
    const std::size_t fromRecordAt = init.find("st.flags.bootFromRecord = ");
    REQUIRE(fromRecordAt != std::string::npos);
    CHECK(catchAt < fromRecordAt);  // set only for a boot that was built
    CHECK(fromRecordAt < taskAt);
}

// Bug caught (the pure-PROS-read pattern, B30): a PROS read widened past one line — the status bits
// decoded in the program (where no host test can see them), the touch's counts read here — or the
// watch fed from anything but competition_get_status(); or the program reading
// competition_is_disabled() on its own path while the watch and the driver loop read the status.
TEST_CASE("pins (P3): PROS's reads are ONE line each, through the host-tested reductions") {
    const std::string code = program();
    CHECK(squash(bodyOf(code, "std::uint8_t readStatus()"))
          == "return pros::c::competition_get_status();");
    CHECK(squash(bodyOf(code, "ScreenTouch readTouch()"))
          == "return screenTouchFrom(pros::c::screen_touch_status(), "
             "static_cast<int>(pros::E_TOUCH_ERROR));");
    CHECK(squash(bodyOf(code, "std::uint8_t read() override")) == "return readStatus();");
    CHECK(count(code, "competition_get_status") == 1);
    CHECK(count(code, "screen_touch_status") == 1);
    CHECK(count(code, "competition_is_disabled") == 0);
    CHECK(count(code, "competition_is_autonomous") == 0);
    CHECK(count(code, "press_count") == 0);
    CHECK(count(code, "release_count") == 0);
}

// Bug caught (RC1, RC3, B19, ND5): the auton task's loop — a throw escaping it (a C++ exception out
// of a PROS task is std::terminate), an idle that ticks the Chassis (two tasks on the robot during
// driver control), an idle that waits for ever on notify_take (no status-change lines, no menu), the
// report written before the terminal word, or a program error's words lost for the fault page. The
// body is pinned WHOLE: it is the program's heart and no host test can run it.
TEST_CASE("pins (P4): the auton task is ONE loop, a one-tick idle, one try per tick, never dies") {
    const std::string code = program();
    // Since COMP-b's review: the heartbeat watched every idle tick, and a request served only after
    // the driver loop has LEFT (waitForDriverLoop, host-tested in (R21)); then the NEWEST request
    // (a second autonomous() may have collapsed in during the wait); runAutonRequest records any
    // program error's reason itself (R22).
    CHECK(squash(bodyOf(code, "void autonTask(void*"))
          == "CompetitionState& st = state(); CompetitionParts parts{st.robot->chassis(), *st.guard, "
             "*st.watch, *st.menu, *st.registry, st.devices->telemetry(), *st.mechanisms, &*st.commands, "
             "st.handshake, st.flags}; StatusObserver "
             "observer; MenuPainter menu; DriverLoopWatcher loop; const auto now = [&st] { return "
             "st.devices->clock().now().value(); }; for (;;) { try { "
             "(void)pros::c::task_notify_take(true, kTickMs); loop.observe(st.presence, now()); "
             "std::uint32_t request = 0; double requestedAt = 0.0; if (st.handshake.pending(st.served, "
             "request, requestedAt)) { const DriverLoopWait waited = waitForDriverLoop(st.presence, "
             "loop, now, [] { pros::delay(kTickMs); }); (void)st.handshake.pending(st.served, request, "
             "requestedAt); st.served = request; runAutonRequest(parts, st.last, request, requestedAt, "
             "++st.serves, observer, st.sdLog.has_value() ? &*st.sdLog : nullptr, st.errorWhat, "
             "waited); paintResult(); } logStatusChange(); paintMenuPhase(menu); } catch (const "
             "std::exception& e) { idleProgramError(e.what()); } catch (...) { idleProgramError(\"an "
             "unknown exception\"); } }");
    CHECK(count(code, "task_notify_take(") == 1);
    CHECK(count(code, "TIMEOUT_MAX") == 0);  // the idle is ONE tick (RC3), never for ever
    CHECK(count(code, "task_join") == 0);    // B19: waits are a timeout or an atomic, never a join
    CHECK(count(code, "constexpr std::uint32_t kTickMs = 10;") == 1);
    // Nothing on the idle half ticks the Chassis: the menu phase and the status log never name it.
    for (const char* fn : {"void paintMenuPhase(MenuPainter& p)", "void logStatusChange()",
                           "void idleProgramError(const char* what) noexcept",
                           "void paintResult()"}) {
        CAPTURE(fn);
        const std::string body = bodyOf(code, fn);
        CHECK(count(body, "chassis") == 0);
        CHECK(count(body, ".wait(") == 0);
        CHECK(count(body, "update(") == 0);
    }
}

// Bug caught (B9, B30; through the mechanism service since MECH-A5): the Robot's PRESENT mechanisms
// never reaching the guard — the parts built without the service (the request's runAuton call then
// the span overload with nothing to safe), or without the route registry (every ROUTE auton refused);
// the boot line's "N of M mechanisms present" counting anything but the service's built mechanisms.
// The request's own use of them is test/competition_run_test.cpp's (R12, R18) and (P15) below.
TEST_CASE("pins (P5): the mechanism service and the route registry are handed to the request") {
    const std::string code = program();
    CHECK(count(code, "presentCount") == 0);
    CHECK(count(code, "st.present") == 0);
    CHECK(count(squash(bodyOf(code, "void autonTask(void*")), "st.devices->telemetry(), *st.mechanisms, "
                                                              "&*st.commands, st.handshake, st.flags};") == 1);
    CHECK(count(code, "static_cast<int>(st.mechanisms->mechanisms().size())") == 1);
    const std::string run = stripComments(readFile("src/programs/competition_run.hpp"));
    CHECK(count(run, "rec.run = competition::runAuton(p.chassis, p.guard, entry, p.commands, p.telemetry,\n"
                     "                                            p.mechanisms, p.registry.periodFor(entry));") == 1);
    CHECK(count(run, "competition::runAuton(") == 1);
}

// Bug caught (RC3, RC7, ND4): a PROS phase task that PAINTS (PROS deletes it mid-statement on a
// mode change, and two painters fight), or builds on benchStateScreen (its erase wipes the menu,
// its words are false here), or forgets to hand the screen over; autonomous() commanding anything
// itself, waiting with task_join or a bound, or not requesting at all.
TEST_CASE("pins (P6): the phase functions log and hand the screen over, and paint NOTHING") {
    const std::string code = program();
    CHECK(squash(bodyOf(code, "void competitionDisabled()"))
          == "CompetitionState& st = state(); logPhase(\"disabled()\", \"entry\", \"\"); "
             "st.owner.handTo(ScreenOwner::Menu); logPhase(\"disabled()\", \"normal return\", "
             "\"the screen to the menu\");");
    CHECK(squash(bodyOf(code, "void competitionConnected()"))
          == "CompetitionState& st = state(); logPhase(\"competition_initialize()\", \"entry\", "
             "\"\"); st.owner.handTo(ScreenOwner::Menu); logPhase(\"competition_initialize()\", "
             "\"normal return\", \"the screen to the menu\");");
    CHECK(squash(bodyOf(code, "void competitionAutonomous()"))
          == "CompetitionState& st = state(); logPhase(\"autonomous()\", \"entry\", \"\"); if "
             "(!st.task.has_value()) { logPhase(\"autonomous()\", \"normal return\", \"the boot was "
             "REFUSED: no auton runs\"); return; } st.owner.handTo(ScreenOwner::AutonStatus); const "
             "std::uint32_t request = st.handshake.request(st.devices->clock().now().value()); "
             "st.task->notify(); while (!st.handshake.isFinished(request)) { pros::delay(kTickMs); } "
             "logPhase(\"autonomous()\", \"normal return\", \"its request FINISHED\");");
    CHECK(count(code, "benchStateScreen") == 0);
    // No painter is named by a phase function (their bodies are pinned whole above).
}

// Bug caught (B2, B11, RC3, B5(1)): opcontrol() touching the robot before the auton task's terminal
// word (two tasks on the Chassis), the wait given an upper bound, driver control on the guard's
// chain (a driver stick run through the auton's pacer), a driver loop that does not end on the
// autonomous bit or a waiting request, that does not CLAIM the robot (DriverLoopPresence::enter --
// refused while a request waits) or beat each tick, or never LEAVES (the auton task would wait out
// the heartbeat at every request -- COMP-b's review, deviation 7 made to hold), the screen not handed
// back to the menu, or the B5(1) entry and normal-return lines dropped (the review's RV28/RV29:
// Station K reads them). The body is pinned WHOLE.
TEST_CASE("pins (P7): opcontrol() waits, claims, runs the driver file, leaves, hands back -- whole") {
    const std::string code = program();
    CHECK(squash(bodyOf(code, "void runCompetition()"))
          == "CompetitionState& st = state(); logPhase(\"opcontrol()\", \"entry\", \"\"); if "
             "(!st.robot.has_value()) { logPhase(\"opcontrol()\", \"normal return\", \"the boot was "
             "REFUSED: nothing to drive\"); return; } st.lcdWriter.forget(); double waited = 0.0; do { "
             "waited += waitForAutonTask( st.handshake, [&st] { return "
             "st.devices->clock().now().value(); }, [] { pros::delay(kTickMs); }, [](double w) { "
             "sayAutonEnding(w); }); } while (!st.presence.enter(st.handshake)); "
             "st.owner.handTo(ScreenOwner::Driver); char line[kCompetitionLineBytes]; "
             "std::snprintf(line, sizeof line, \"opcontrol(): driver control starts -- the driver "
             "file, after %.3f s waiting for \" \"the auton task\", waited); "
             "logLine(hal::LogLevel::Info, line); robot::Robot& r = *st.robot; DriverPanel panel{r, "
             "st}; st.driver.run(r, *st.mechanisms, st.devices->pacer(), [&st] { st.presence.beat(); return "
             "driverControlMustEnd(readStatus(), st.handshake.busy()); }, panel); st.presence.leave(); "
             "st.owner.handOver(ScreenOwner::Driver, ScreenOwner::Menu); std::snprintf(line, sizeof "
             "line, ThisRobotWords::value.claw ? \"opcontrol(): driver control ended -- 0 V + Brake, each "
             "claw pivot safe, the \" \"pneumatic claw kept; %lu screen prints so far\" : \"opcontrol(): "
             "driver control ended -- 0 V + Brake, each motor mechanism safe, \" \"each pneumatic kept; %lu "
             "screen prints so far\", static_cast<unsigned "
             "long>(g_screenPrints)); logLine(hal::LogLevel::Info, line); logPhase(\"opcontrol()\", "
             "\"normal return\", \"the screen back to the menu\");");
    CHECK(count(code, "st.driver.run(") == 1);
    CHECK(count(code, "waitForAutonTask(") == 1);
    CHECK(count(code, "presence.enter(") == 1);
    CHECK(count(code, "presence.leave(") == 1);
    CHECK(count(code, "presence.beat(") == 1);
}

// Bug caught (B20, RC3): a painter that paints without holding the screen — the status page or the
// result page after the opcontrol task took it (the result over the driver page), the menu phase or
// the driver page deciding from a stale owner — or AUTON ENDING painted without TAKING the screen.
TEST_CASE("pins (P8): every paint is decided by the screen owner") {
    const std::string code = program();
    CHECK(count(squash(bodyOf(code, "void starting(const CompetitionRunRecord& rec) override")),
                "if (st.owner.holds(ScreenOwner::AutonStatus)) { paintViewWhole(") == 1);
    CHECK(count(squash(bodyOf(code, "void paintResult()")),
                "if (st.owner.holds(ScreenOwner::AutonStatus)) { paintViewWhole(") == 1);
    CHECK(count(code, "p.tracker.next(st.owner.snapshot(), ScreenOwner::Menu, now)") == 1);
    CHECK(count(code, "menuPhaseStep(action, refused, p.menuUp)") == 1);
    CHECK(count(code, "tracker_.next(st_.owner.snapshot(), ScreenOwner::Driver, now)") == 1);
    const std::string ending = squash(bodyOf(code, "void sayAutonEnding(double waitedSeconds)"));
    CHECK(ending.find("st.owner.handTo(ScreenOwner::Driver);") < ending.find("paintViewWhole("));
    // Every whole-page paint is one of these five call sites, and no other paint exists.
    CHECK(count(code, "paintViewWhole(") == 6);  // its definition, the changes' fallback, four uses
    CHECK(count(code, "paintMenuFrame(") == 2);
    CHECK(count(code, "paintAutonsRefused()") == 2);  // its definition and the menu phase
    CHECK(count(code, "printText(") == 0);          // every string through the shared painters
    CHECK(count(code, "screen_erase") == 0);
}

// Bug caught (ND5; HA-177): the SD run log bypassing the adapter that owns /usd/, opened truncating
// (every reboot would erase the tester's runs too), or naming a second file.
TEST_CASE("pins (P9): the SD run log is the shared file, through the adapter's Append mode") {
    const std::string code = program();
    CHECK(count(code, "fopen") == 0);
    for (const std::string& lit : stringLiterals(code)) {
        CHECK_MESSAGE(lit.find("/usd/") == std::string::npos, "a /usd/ literal: ", lit);
        CHECK_MESSAGE(lit.find("shulib_auton_runs") == std::string::npos, "a second name: ", lit);
    }
    CHECK(count(code, "st.sdBlock.emplace(kAutonRunLogFile, hal::pros::ProsFileMode::Append);")
          == 1);
    // Two Append opens since T4 Part 3: the run log (above) and the provider's log copy (P17).
    CHECK(count(code, "ProsFileMode::") == 2);
    CHECK(count(code, "ProsFileMode::Append") == 2);
    // The boot lines are competitionBootLines' three (host-tested at the worst case), each to the
    // card too; no one-line boot format of the program's own.
    CHECK(count(code, "competitionBootLines(") == 1);
    CHECK(count(squash(code), "for (const char* bootLine : boot.line) { "
                              "logLine(hal::LogLevel::Info, bootLine, true); }") == 1);
    for (const std::string& lit : stringLiterals(code)) {
        CHECK_MESSAGE(lit.find("SD run log") == std::string::npos, "an SD boot line here: ", lit);
        CHECK_MESSAGE(lit.find("boot source") == std::string::npos, "a boot line here: ", lit);
    }
}

// Bug caught: the program commanding the robot itself — every motion is the auton's (through
// runAuton, under the guard) or the driver's (through the driver file); the program-error stop is
// the shared one inside the request.
TEST_CASE("pins (P10): the program names no motion verb") {
    const std::string code = program();
    for (const char* verb : {".drive(", ".moveTo(", ".turnTo(", ".brake(", "setVoltage(",
                             "move_voltage", "cancel(", ".hold(", ".face(", "applySafeState(",
                             "stopAfterProgramError("}) {
        CAPTURE(verb);
        CHECK(count(code, verb) == 0);
    }
}

// Bug caught (RC5; MECH-A5): the mechanism boot lines printed from words of the program's own (the
// worksheet's J.3.1, J.9.1 and K quote the shared words, true for every program), from another list
// than the build's, without the driver file's controls, or not through the provider's sink (the log
// copy would miss them); the ROUTES line missing (a route editor's ids unconfirmed at boot). Until
// MECH-A5 two claw lines per robot (motorClawBootLine / pneumaticClawBootLine) said most of it.
TEST_CASE("pins (P11): the mechanism boot lines are the shared panel's, from the build's list and the "
          "driver file's controls, and the ROUTES line follows them") {
    const std::string code = program();
    CHECK(count(squash(code), "mechanismBootLines(kMechanisms, *st.robot, *st.mechanisms, &ThisRobotDriver::kControls, "
                              "[&st](const MechanismLine& m) { st.devices->telemetry().log(m.level, \"MECH\", "
                              "m.text); });") == 1);
    CHECK(count(code, "mechanismBootLines(") == 1);
    CHECK(count(code, "ClawBootLine(") == 0);
    CHECK(count(squash(code), "const RoutesLine routes = routesBootLine(st.routes->ids()); "
                              "logLine(hal::LogLevel::Info, routes.text, true);") == 1);
    CHECK(count(code, "#include \"common/driver_panel.hpp\"") == 1);
    CHECK(count(code, "teleop_claws") == 0);
    CHECK(count(code, "kRobotTwoMechanisms") == 0);  // the build's table by the selected name only
}

// Bug caught (B29, RC7): src/main.cpp's Competition cases doing anything but forwarding to the
// program's function for that entry point — a state screen painted over the menu, an auton run
// from main.cpp, driver control not the program's.
TEST_CASE("pins (P12): src/main.cpp's Competition cases each forward to the program's function") {
    const std::string code = stripComments(readFile("src/main.cpp"));
    const struct {
        const char* fn;
        const char* want;
    } cases[] = {
        {"void initialize()", "case Program::Competition: shulib::bench::competitionInitialize("
                              "kVariantBeacon, bootStatusBits(bits, sizeof bits)); return;"},
        {"void disabled()", "case Program::Competition: shulib::bench::competitionDisabled(); return;"},
        {"void competition_initialize()",
         "case Program::Competition: shulib::bench::competitionConnected(); return;"},
        {"void autonomous()",
         "case Program::Competition: shulib::bench::competitionAutonomous(); return;"},
        {"void opcontrol()", "case Program::Competition: shulib::bench::runCompetition(); return;"},
    };
    for (const auto& c : cases) {
        CAPTURE(c.fn);
        const std::string body = bodyOf(code, c.fn);
        const std::size_t at = body.find("case Program::Competition:");
        REQUIRE(at != std::string::npos);
        std::size_t end = body.find("case Program::", at + 1);
        if (end == std::string::npos) {
            end = body.rfind('}');  // the switch's closing brace
        }
        CHECK(squash(body.substr(at, end - at)) == c.want);
    }
}

// Bug caught (T4's N4, RC4): a slot number in a string the robot prints (the documents name the
// slot; a printed one goes stale), or a PROS header in a file the host suite compiles as PROS-free
// (the host shim provides some pros/ headers, so such an include would still compile on the host
// and hide a robot-only call).
TEST_CASE("pins (P13): no slot number in any string; the pure headers include no pros/ header") {
    const std::regex slot{"slot[ -]*[0-9]", std::regex::icase};
    for (const char* file :
         {"src/programs/competition_program.cpp", "src/programs/competition_program.hpp",
          "src/programs/competition_run.hpp", "src/programs/competition_screen.hpp",
          "src/programs/competition_watch.hpp", "src/common/driver_panel.hpp",
          "src/common/screen_paint.cpp", "src/common/screen_paint.hpp"}) {
        CAPTURE(file);
        for (const std::string& lit : stringLiterals(stripComments(readFile(file)))) {
            CHECK_MESSAGE(!std::regex_search(lit, slot), "a slot number in: ", lit);
        }
    }
    const std::regex prosInclude{R"(#\s*include\s*[<"]pros/)"};
    for (const char* file :
         {"src/programs/competition_run.hpp", "src/programs/competition_screen.hpp",
          "src/programs/competition_watch.hpp", "src/programs/competition_program.hpp",
          "src/common/guarded_run.hpp", "src/common/screen_touch.hpp",
          "src/common/auton_run_log.hpp", "src/common/driver_panel.hpp",
          "src/common/screen_paint.hpp", "src/common/state_screens.hpp"}) {
        CAPTURE(file);
        CHECK_FALSE(std::regex_search(stripComments(readFile(file)), prosInclude));
    }
}

// Bug caught (COMP-b's review, 2026-10-01: RV17-RV27, RV33 -- each left the full suite green): the
// program file's code that feeds ND1, ND2, ND5 and deviation 6 and writes what Station K reads. Each
// body is pinned WHOLE, with the bug its pin was written for:
//   logLine            -- every line to the SD card too (two writers on a sink that is not
//                         thread-safe; opcontrol()'s line on the card before driver control: ND5);
//                         and (SER1, 2026-10-08) the pre-provider line back to printf + fflush
//                         (stdout's shared, unguarded buffer -- HA-189) instead of ONE write()
//                         per piece through hal::pros::serialWrapped();
//   logPhase           -- the phase lines' status bits or millis lost (B5(1); HA-161, HA-162);
//   paintAutonsRefused -- the refused page's headline or reason (ND1, ND2; the review's PD);
//   paintMenuPhase     -- a typed-table boot painting the SELECTOR (ND2: it invites a choice that is
//                         refused), a blind read not consumed (a tap made during driver control
//                         acted on later), the selector believed up for ever (the refusal repainted
//                         every tick -- PROS leaks every print);
//   logStatusChange    -- the last status never kept (a "change" line every 10 ms);
//   idleProgramError   -- an idle-half throw not latched (later autons run on an unproven state:
//                         deviation 6, ND1), its reason not recorded, or its line said every tick;
//   DriverPanel        -- the controller refreshed late (fix 3) or not at all, the AUTON row never
//                         refused (ND1/ND2 during driver control), a fact taken from the wrong source
//                         (DEGRADED never said), a controller line written outside the writer;
//   sayAutonEnding     -- AUTON ENDING never reaching the controller, or its write not remembered
//                         (the next write inside PROS's rate, and lost).
TEST_CASE("pins (P14): the program's log, screen and controller functions, each WHOLE") {
    const std::string code = program();
    const struct {
        const char* signature;
        const char* body;
    } pins[] = {
        {"void logLine(hal::LogLevel level, const char* text, bool alsoSd = false)",
         "CompetitionState& st = state(); if (st.devices.has_value()) { "
         "st.devices->telemetry().log(level, kTag, text); } else { "
         "hal::pros::serialWrapped(kTag, text); } if (alsoSd && st.sdLog.has_value()) { "
         "st.sdLog->log(level, kTag, text); }"},
        {"void logPhase(const char* what, const char* when, const char* note)",
         "char bits[64]; competitionBits(readStatus(), bits, sizeof bits); char "
         "line[kCompetitionLineBytes]; (void)competitionPhaseLine(line, sizeof line, what, when, "
         "pros::millis(), bits, note); logLine(hal::LogLevel::Info, line);"},
        {"void paintAutonsRefused()",
         "CompetitionState& st = state(); const bool error = st.flags.programError.load(); "
         "paintRefusalScreen(RefusalScreen{ .tag = kTag, .barWord = kCompetitionName.data(), "
         ".buildTime = kBuildTimeText.data(), .headline = error ? \"AUTONS REFUSED: a program "
         "error on the auton task\" : \"AUTONS REFUSED: this boot used the TYPED TABLE\", .why = "
         "error ? st.errorWhat : \"The typed chassis table's front may be the OTHER end from the "
         "calibration \" \"record's, so an auton could drive the wrong way. Run shulib Calibrate, "
         "then \" \"restart.\", .footer = error ? kDriverStillRuns : kTableFooter, .logToSerial = "
         "false});"},
        {"void paintMenuPhase(MenuPainter& p)",
         "CompetitionState& st = state(); const double now = st.devices->clock().now().value(); "
         "const ScreenTouch touch = readTouch(); const PaintAction action = "
         "p.tracker.next(st.owner.snapshot(), ScreenOwner::Menu, now); const bool refused = "
         "autonsRefused(st.flags); const MenuPhaseStep step = menuPhaseStep(action, refused, "
         "p.menuUp); p.menuUp = step.menuUpAfter; if (step.consumeBlind) { "
         "(void)st.menu->touch(touch.sample, competition::MenuView{}); } if (step.paintRefused) { "
         "paintAutonsRefused(); } if (step.paintMenuWhole) { p.drawn = st.menu->view(now); "
         "paintMenuFrame(p.drawn); p.paintedAt = now; } if (!step.actOnDrawn) { return; } const "
         "int before = st.menu->choice(); (void)st.menu->touch(touch.sample, p.drawn); if "
         "(st.menu->choice() != before) { char line[kCompetitionLineBytes]; const "
         "competition::AutonEntry* e = st.menu->chosenEntry(); std::snprintf(line, sizeof line, "
         "\"menu: the auton is now '%.29s'\", e != nullptr ? e->name : \"NONE\"); "
         "logLine(hal::LogLevel::Info, line); } const competition::MenuView next = "
         "st.menu->view(now); if (!sameMenuButtons(next, p.drawn)) { paintMenuFrame(next); p.drawn "
         "= next; p.paintedAt = now; } else if (now - p.paintedAt >= kLiveRepaintSeconds) { "
         "paintMenuText(p.drawn, next); p.drawn = next; p.paintedAt = now; }"},
        {"void logStatusChange()",
         "CompetitionState& st = state(); const std::uint8_t status = readStatus(); if (status == "
         "st.lastStatus) { return; } char bits[64]; char line[kCompetitionLineBytes]; "
         "std::snprintf(line, sizeof line, \"competition status %.44s at %lu ms\", "
         "competitionBits(status, bits, sizeof bits), static_cast<unsigned long>(pros::millis())); "
         "logLine(hal::LogLevel::Info, line); st.lastStatus = status;"},
        {"void idleProgramError(const char* what) noexcept",
         "CompetitionState& st = state(); st.flags.programError.store(true); "
         "std::snprintf(st.errorWhat, sizeof st.errorWhat, \"the auton task threw outside a "
         "request: %.80s -- the robot was not touched\", what != nullptr ? what : \"\"); if "
         "(st.idleErrorSaid) { return; } st.idleErrorSaid = true; try { char "
         "line[kCompetitionLineBytes]; std::snprintf(line, sizeof line, \"auton task: PROGRAM "
         "ERROR outside a request: %.80s -- the robot was not \" \"touched; later autons REFUSED "
         "until a restart\", what != nullptr ? what : \"\"); logLine(hal::LogLevel::Error, line); "
         "} catch (...) { }"},
        // T4 Part 3 (Part 2b's item 6): the hook takes the FOURTH argument, the drive protections'
        // state, and hands it to the page (the PROTECTION row) and the controller (REFUSED, CUT,
        // the dead ports) -- re-pointed one-for-one; every other statement unchanged.
        {"void operator()(const math::ChassisSpeeds&",
         "++tick_; const double now = robot_.context().clock().now().value(); const PaintAction "
         "action = tracker_.next(st_.owner.snapshot(), ScreenOwner::Driver, now); if (action == "
         "PaintAction::Whole) { painted_ = view(connected, protection); paintViewWhole(painted_); "
         "paintedAt_ = now; } else if (action == PaintAction::Changes && now - paintedAt_ >= "
         "kLiveRepaintSeconds) { const CompetitionView next = view(connected, protection); "
         "paintViewChanges(painted_, next); painted_ = next; paintedAt_ = now; } if (tick_ % "
         "kLcdEveryTicks == 1) { refreshLcd(connected, protection, now); }"},
        {"CompetitionView view(bool connected,",
         "const diag::FaultLatch& f = robot_.faults(); faultRow_ = faultRowText(f.hasFault() ? "
         "f.faultCount() : 0, f.firstFault(), f.lastFault()); const bool freed = "
         "st_.driver.releasedLeftoverAtEntry(*st_.mechanisms); if constexpr (ThisRobotWords::value.claw) { "
         "clawRow_ = clawRowText(kMechanisms, robot_, *st_.mechanisms, freed); } else { clawRow_ = "
         "clawRowText(kMechanisms, robot_, *st_.mechanisms, freed, ThisRobotWords{}); } CompetitionDriverFacts "
         "facts{}; facts.protection = protectionRowText(protection, kCutSeconds); facts.auton = "
         "competitionAutonRow(st_.last, *st_.registry, "
         "st_.flags.programError.load(), st_.flags.bootFromRecord); facts.controller = "
         "controllerRowText(connected); facts.faultRow = faultRow_.text; facts.faultRed = "
         "f.hasFault(); facts.clawRow = clawRow_.text; facts.clawAmber = anyClawAbsent(robot_) || "
         "freed; facts.buildStamp = kBuildTimeText; "
         "return competitionDriverView(facts);"},
        {"void refreshLcd(bool connected, const teleop::DriveProtectionState& protection, double now)",
         "ControllerFacts f{}; f.batteryVolts = robot_.context().battery().voltage().value(); "
         "f.headingDegrees = robot_.localizer().pose().heading().radians() * 180.0 / "
         "math::Angle::kPi; f.hasFault = robot_.faults().hasFault(); f.lastFault = "
         "robot_.faults().lastFault(); f.connected = connected; f.degraded = "
         "st_.boot.verdict.decision == calibration::BootDecision::Degraded; f.clawAbsent = "
         "anyClawAbsent(robot_); addProtectionFacts(f, protection); const ControllerScreen want = "
         "ThisRobotWords::value.claw ? controllerScreenLines(f) : controllerScreenLines(f, ThisRobotWords{}); "
         "const int row = state().lcdWriter.next(want, now); if (row >= "
         "0) { state().lcd->setLine(row, want.line[row]); }"},
        {"void sayAutonEnding(double waitedSeconds)",
         "CompetitionState& st = state(); st.owner.handTo(ScreenOwner::Driver); const "
         "competition::AutonEntry* running = st.menu->chosenEntry(); const char* name = running != "
         "nullptr ? running->name : \"NONE\"; paintViewWhole(competitionEndingView(name, "
         "waitedSeconds, kBuildTimeText)); ControllerFacts f{}; f.autonEnding = true; const "
         "ControllerScreen ending = ThisRobotWords::value.claw ? controllerScreenLines(f) : "
         "controllerScreenLines(f, ThisRobotWords{}); st.lcd->setLine(2, ending.line[2]); "
         "st.lcdWriter.wrote(2, ending.line[2], st.devices->clock().now().value()); char "
         "line[kCompetitionLineBytes]; std::snprintf(line, sizeof line, \"opcontrol(): AUTON "
         "ENDING -- '%.29s' has not handed the robot back %.2f s after \" \"driver control started "
         "(a wait the guard cannot cut?); driver control waits\", name, waitedSeconds); "
         "logLine(hal::LogLevel::Warn, line);"},
    };
    for (const auto& pin : pins) {
        CAPTURE(pin.signature);
        CHECK(count(code, pin.signature) == 1);
        CHECK(squash(bodyOf(code, pin.signature)) == pin.body);
    }
    // The controller is written in exactly two places: AUTON ENDING's line, and the writer's line.
    CHECK(count(code, "->setLine(") == 2);
    // logLine is the program's ONE line writer, and only the boot and the auton task pass `alsoSd`.
    // The boot's lines, an ABSENT mechanism's, (T4 Part 3, C5) each DROPPED auton's, and (MECH-A5) the
    // ROUTES line.
    CHECK(count(code, ", true);") == 4);
}

// Bug caught (RC1, ND5 -- the review's RV2 and RV3, both green on the full suite): the END estimate
// and this request's faults read AFTER the terminal word (they race the driver loop's localizer
// update in chassis.drive(): a torn END estimate), or the program-error stop moved after it (the
// drive to 0 V + Brake and the claws forced safe while the driver drives). A single-threaded host
// test sees the same numbers either way (test/competition_run_test.cpp's (R18) watches the stop's
// order through a probe mechanism), so serveAutonRequest's tail is pinned by ORDER: the stop, the
// latch, the watch disarmed, the reads, the menu unlocked, and FINISHED -- its last act.
TEST_CASE("pins (P15): serveAutonRequest's tail -- the stop and the reads BEFORE FINISHED, its last act") {
    const std::string code = stripComments(readFile("src/programs/competition_run.hpp"));
    // Its parameter list ends in a `= {}` default, so the body is the brace after its ") {".
    const std::size_t defAt = code.find("inline CompetitionRunRecord& serveAutonRequest(");
    REQUIRE(defAt != std::string::npos);
    const std::size_t openAt = code.find(") {", defAt);
    REQUIRE(openAt != std::string::npos);
    const std::string body = squash(bodyOf(code.substr(openAt), ") {"));
    CHECK(body.rfind("rec = CompetitionRunRecord{};", 0) == 0);  // the right body, from its start
    const char* order[] = {"stopAfterProgramError(p.chassis, p.mechanisms.mechanisms());",
                           "p.flags.programError.store(true);",
                           "p.watch.disarm();",
                           "rec.endEstimate = localizer.pose();",
                           "rec.faultsAfter = faults.faultCount();",
                           "p.menu.unlock();",
                           "p.handshake.finish(request);"};
    std::size_t prev = 0;
    for (const char* step : order) {
        CAPTURE(step);
        CHECK(count(body, step) == 1);
        const std::size_t at = body.find(step);
        REQUIRE(at != std::string::npos);
        CHECK(at >= prev);
        prev = at;
    }
    const std::string tail = "p.handshake.finish(request); return rec;";
    REQUIRE(body.size() >= tail.size());
    CHECK(body.substr(body.size() - tail.size()) == tail);  // FINISHED, then nothing but the return
    CHECK(count(code, "handshake.finish(") == 1);
    CHECK(count(code, ".finish(") == 1);
}

// Bug caught (the verification round, 2026-10-01 -- V02, V03 and V05, all green on the full suite):
// the store-then-load handshake that keeps the opcontrol task and the auton task from both deciding
// the robot is theirs (competition_run.hpp: AutonHandshake's class doc, DriverLoopPresence::enter).
// enter() must STORE its claim before it READS the request, and the request's increment and every
// read of the pair must be sequentially consistent (no memory_order argument). With the read first
// (V02), opcontrol() can read "no request", the auton task then read "no loop" and serve, and the
// loop run a tick inside the auton (its exit's cancel() ends the motion); a relaxed increment (V03)
// breaks the seq_cst half; an enter() that does not beat (V05) leaves a loop that has only just
// claimed the robot looking stale (test/competition_run_test.cpp's (R21) tests that one too). A
// single-threaded host test cannot observe the first two, so every member of the pair is pinned
// WHOLE.
TEST_CASE("pins (P16): the handshake's store-then-load pair -- the claim before the read, all seq_cst") {
    const std::string code = stripComments(readFile("src/programs/competition_run.hpp"));
    const struct {
        const char* signature;
        const char* body;
    } pins[] = {
        {"bool enter(const AutonHandshake& handshake) noexcept",
         "beats_.fetch_add(1); active_.store(true); if (handshake.busy()) { active_.store(false); "
         "return false; } return true;"},
        {"void beat() noexcept", "beats_.fetch_add(1);"},
        {"void leave() noexcept", "active_.store(false);"},
        {"bool active() const noexcept", "return active_.load();"},
        {"std::uint32_t beats() const noexcept", "return beats_.load();"},
        {"std::uint32_t request(double atSeconds) noexcept",
         "const double ms = atSeconds > 0.0 ? atSeconds * 1000.0 + 0.5 : 0.0; "
         "atMs_.store(static_cast<std::uint32_t>(ms), std::memory_order_relaxed); return "
         "requested_.fetch_add(1) + 1;"},
        {"bool pending(std::uint32_t served",
         "const std::uint32_t r = requested_.load(); if (r == served) { return false; } request = "
         "r; atSeconds = static_cast<double>(atMs_.load(std::memory_order_relaxed)) / 1000.0; "
         "return true;"},
        {"bool busy() const noexcept", "return finished_.load() != requested_.load();"},
        {"void finish(std::uint32_t n) noexcept", "finished_.store(n);"},
    };
    for (const auto& pin : pins) {
        CAPTURE(pin.signature);
        CHECK(count(code, pin.signature) == 1);
        CHECK(squash(bodyOf(code, pin.signature)) == pin.body);
    }
    // The only relaxed operations are the request time's store and load (ordered by the seq_cst
    // increment and read around them); the presence names no memory order at all.
    CHECK(count(code, "memory_order_relaxed") == 2);
    CHECK(count(bodyOf(code, "class DriverLoopPresence"), "memory_order") == 0);
    CHECK(count(code, "requested_.fetch_add(") == 1);
    CHECK(count(code, "active_.store(") == 3);  // enter's claim and its undo, and leave()
}

// Bug caught (T4 Part 3; the team lead's N5 (a), A (a) and B (a)): the competition program's SD
// copy of its log opened TRUNCATING (a reboot mid-match would erase that match's log -- B (a)'s
// reason), named with a /usd/ path of its own (the provider's ProsBlockSink owns the prefix), taken
// from a second ProsBlockSink of the program's (a second writer on the card, and every record line
// with it -- A (a)), or never said at boot (a practice match believed recorded with no card in the
// brain), or said in words of its own (the two programs' lines would drift).
TEST_CASE("pins (P17): the log copy -- the provider's, APPENDED, said once in the shared words") {
    const std::string code = program();
    CHECK(count(code, "constexpr const char* kLogCopyFile = \"comp_log.txt\";") == 1);
    CHECK(count(code, "kLogCopyFile") == 2);  // its definition and the provider's construction
    CHECK(count(code, "st.devices.emplace(") == 1);
    CHECK(count(code, "st.devices.emplace(kLogCopyFile, hal::pros::ProsFileMode::Append);") == 1);
    CHECK(count(code, "ProsBlockSink") == 1);  // the run log's (P9) -- never a second one for the copy
    CHECK(count(code, "#include \"common/log_copy_line.hpp\"") == 1);
    CHECK(count(code, "logCopyBootLine(") == 1);
    // (Re-pinned at the T4-3 review: the shared mapping, with the card asked whether it is in, so a
    // copy that failed to open says NO SD CARD only when there is none; `true`: an Append copy.)
    CHECK(count(squash(code),
                "const LogCopyOpen how = logCopyOpenFrom( st.devices->serialCopyOpen(), "
                "pros::c::usd_is_installed() != 0, true, st.devices->serialCopyAppendOpen() == "
                "hal::pros::AppendOpen::Created); const LogCopyLine copy = "
                "logCopyBootLine(how, st.devices->serialCopyPath(), "
                "st.devices->serialCopyDroppedLines()); logLine(copy.warn ? hal::LogLevel::Warn : "
                "hal::LogLevel::Info, copy.text);")
          == 1);
    CHECK(count(code, "usd_is_installed") == 1);
    for (const std::string& lit : stringLiterals(code)) {
        CHECK_MESSAGE(lit.find("log copy") == std::string::npos, "a log-copy line of its own: ", lit);
    }
}

// Bug caught (T4 Part 3; the team lead's ruling of 2026-10-02 on COMP-b's C5, "drop just that
// auton"): one auton entry the registry refuses costing the whole boot -- driver control included
// -- because the registry was built to throw (the default) inside the boot's try; or a dropped
// entry never named at boot (the student sees it missing from the menu and the log is silent), or
// named in words of the program's own; or the students' file wrapped in a try of the program's (a
// throw half-way through it would lose every later entry).
TEST_CASE("pins (P18): the team's autons register through a registry that DROPS a refused entry") {
    const std::string code = program();
    CHECK(count(code, "st.registry.emplace(") == 1);
    CHECK(count(code, "st.registry.emplace(kAutonPeriods, competition::OnRefusal::DropEntry);") == 1);
    CHECK(count(code, "OnRefusal::Throw") == 0);
    CHECK(count(code, "#include \"common/dropped_autons.hpp\"") == 1);
    // `true`: the competition program's driver control still runs past a dropped auton (C5).
    CHECK(count(squash(code), "sayDroppedAutons(*st.registry, true, [](const char* dropped) { "
                              "logLine(hal::LogLevel::Warn, dropped, true); });")
          == 1);
    CHECK(count(code, "refusal(") == 0);       // the sentences are said by the shared words only
    CHECK(count(code, "refusedCount(") == 0);
    for (const std::string& lit : stringLiterals(code)) {
        CHECK_MESSAGE(lit.find("DROPPED") == std::string::npos, "a drop line of its own: ", lit);
    }
    // The dropped lines come AFTER the registry is built and the boot is past its try (the run
    // log they go to opens after the try), and before the auton task exists.
    const std::string init = squash(bodyOf(code, "void competitionInitialize("));
    const std::size_t sdAt = init.find("st.sdBlock.emplace(");
    const std::size_t sayAt = init.find("sayDroppedAutons(");
    const std::size_t taskAt = init.find("st.task.emplace(");
    REQUIRE(sdAt != std::string::npos);
    REQUIRE(sayAt != std::string::npos);
    REQUIRE(taskAt != std::string::npos);
    CHECK(sdAt < sayAt);
    CHECK(sayAt < taskAt);
}

// Bug caught (T4 Part 3, Part 2b's item 6): the driver page and the controller never told of a cut
// or a refusal -- the hook taking three arguments (the library's loop then hands it nothing), the
// PROTECTION row built from a number of the program's own (a retyped cut length drifts from the
// config the loop runs with), or the controller's facts left without the protections' words.
TEST_CASE("pins (P19): the driver page's hook takes the protections' state; the cut length is the config's") {
    const std::string code = program();
    CHECK(count(squash(code), "void operator()(const math::ChassisSpeeds& , bool connected, const "
                              "teleop::DriveShaperState& , const teleop::DriveProtectionState& "
                              "protection) {")
          == 1);
    CHECK(count(squash(code), "constexpr double kCutSeconds = "
                              "ThisRobotDriver::kLoop.protection.cutTicks * "
                              "ThisRobotDriver::kLoop.tickPeriod.value();")
          == 1);
    CHECK(count(code, "protectionRowText(") == 1);
    CHECK(count(code, "addProtectionFacts(") == 1);
    CHECK(count(code, "#include \"shulib/teleop/drive_protection.hpp\"") == 1);
}
