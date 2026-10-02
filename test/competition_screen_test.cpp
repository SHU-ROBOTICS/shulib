// competition_screen_test.cpp — "shulib Competition"'s SCREEN (src/programs/competition_screen.hpp;
// chunk COMP-b, 2026-09-30; the COMP brief's B20 and its re-check's RC3 and ND4): the atomic
// SCREEN OWNER and each painter's tracker, the menu phase's rule for a touch, and every page the
// program paints. Every case names the bug it would catch.
//
// RC3 asks for exactly this: "the owner handover is a small PROS-free state machine, host-tested:
// a paint from a non-owner is refused; the handovers happen in the order above". Two tasks paint on
// the robot and PROS deletes a third mid-statement, so every rule is walked here with the three
// painters' trackers side by side — the auton task's menu, its status/result, and the opcontrol
// task's driver page — through a match, a Timed Run, a frozen auton and a stale handover.
// Host only; nothing here has run on a robot (worksheet Station K).

#include "doctest.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "competition_rig.hpp"
#include "programs/competition_run.hpp"
#include "programs/competition_screen.hpp"
#include "team/autons.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/ui/touch_panel.hpp"
#include "shulib/units/quantity.hpp"

using shulib::bench::CompetitionRunRecord;
using shulib::bench::CompetitionSkip;
using shulib::bench::CompetitionView;
using shulib::bench::MenuPhaseStep;
using shulib::bench::menuPhaseStep;
using shulib::bench::OwnerTracker;
using shulib::bench::PaintAction;
using shulib::bench::ScreenOwner;
using shulib::bench::ScreenOwnerCell;
using shulib::competition::AutonRegistry;
using shulib::competition::AutonRunOutcome;
using shulib::units::Time;

namespace {

const shulib::kinematics::TankKinematics& tank() {
    static const shulib::kinematics::TankKinematics k{shulib::units::Length{12.0}};
    return k;
}

/// The three painters, each with its own tracker, reading ONE owner (the program's shape).
struct Painters {
    ScreenOwnerCell owner;
    OwnerTracker menu;    // the auton task, the menu phase
    OwnerTracker driver;  // the opcontrol task, the driver page
    /// What each painter does at `now`; the status/result painter is a plain holds() check.
    struct Tick {
        PaintAction menu;
        PaintAction driver;
        bool status;
    };
    Tick at(double now) {
        const ScreenOwnerCell::Snapshot s = owner.snapshot();
        return Tick{menu.next(s, ScreenOwner::Menu, now), driver.next(s, ScreenOwner::Driver, now),
                    owner.holds(ScreenOwner::AutonStatus)};
    }
};

/// True when at most one painter paints on this tick (a non-owner's paint is refused).
bool onePainter(const Painters::Tick& t) {
    const int painting = (t.menu != PaintAction::None ? 1 : 0)
                         + (t.driver != PaintAction::None ? 1 : 0) + (t.status ? 1 : 0);
    return painting <= 1;
}

/// Every line of `v` is screen-safe at its budget and above the rows VEXos keeps.
void checkView(const CompetitionView& v) {
    std::string page = std::string{"["} + v.header.text + "] [" + v.big.text + "]";
    for (std::size_t i = 0; i < v.lineCount; ++i) {
        page += std::string{" | "} + v.lines[i].text;
    }
    CAPTURE(page);  // every line of the page, when a check below fails
    CHECK(v.unsafeLines == 0);
    CHECK(v.droppedLines == 0);
    CHECK(shulib::ui::screenText(v.header.text, shulib::ui::kMediumCols));
    CHECK(shulib::ui::screenText(v.barRight.text, shulib::ui::kSmallCols));
    if (v.hasBig) {
        CHECK(shulib::ui::screenText(v.big.text, shulib::ui::kLargeCols));
    }
    for (std::size_t i = 0; i < v.lineCount; ++i) {
        CAPTURE(v.lines[i].text);
        CHECK(shulib::ui::screenText(v.lines[i].text, shulib::ui::kSmallCols));
        CHECK(v.lines[i].y + shulib::ui::kSmallHeight <= shulib::ui::kPanelLastRow);
    }
}

std::string lineText(const CompetitionView& v, std::size_t i) {
    REQUIRE(i < v.lineCount);
    return v.lines[i].text;
}

}  // namespace

// ═══ the owner ═══════════════════════════════════════════════════════════════════════════════

// (S1) Bug caught: a handover that does not move the generation (the new owner would never know
// it took the screen, and would paint only what CHANGED over another page's pixels), handOver()
// taking the screen from an owner it did not name (a stale hand-back after a newer take), or
// handTo() refusing (a phase task PROS is about to delete must never block).
TEST_CASE("screen (S1): handTo always lands and moves the generation; handOver only from `from`") {
    ScreenOwnerCell c;
    CHECK(c.snapshot().owner == ScreenOwner::Menu);
    CHECK(c.snapshot().generation == 0);
    CHECK(c.handTo(ScreenOwner::AutonStatus) == ScreenOwner::Menu);
    CHECK(c.snapshot().owner == ScreenOwner::AutonStatus);
    CHECK(c.snapshot().generation == 1);
    CHECK(c.handTo(ScreenOwner::AutonStatus) == ScreenOwner::AutonStatus);  // to itself: still moves
    CHECK(c.snapshot().generation == 2);
    CHECK_FALSE(c.handOver(ScreenOwner::Driver, ScreenOwner::Menu));  // Driver does not hold it
    CHECK(c.snapshot().owner == ScreenOwner::AutonStatus);
    CHECK(c.snapshot().generation == 2);  // a refused handOver changes nothing
    CHECK(c.handOver(ScreenOwner::AutonStatus, ScreenOwner::Driver));
    CHECK(c.snapshot().owner == ScreenOwner::Driver);
    CHECK(c.snapshot().generation == 3);
    CHECK(c.holds(ScreenOwner::Driver));
    CHECK_FALSE(c.holds(ScreenOwner::Menu));
    CHECK_FALSE(c.holds(ScreenOwner::AutonStatus));
}

// (S2) Bug caught (RC3: "a paint from a non-owner is refused"): a tracker that paints while
// another holds the screen, that does not paint WHOLE on a take (a page painted as changes over
// someone else's pixels), that never paints the second whole frame (an old owner's in-flight paint
// left on the screen), or that keeps repainting whole (PROS leaks every print).
TEST_CASE("screen (S2): a tracker — None unless it holds; Whole on a take and once more; then Changes") {
    ScreenOwnerCell c;
    OwnerTracker t;
    c.handTo(ScreenOwner::Driver);
    CHECK(t.next(c.snapshot(), ScreenOwner::Menu, 0.0) == PaintAction::None);  // not its screen
    c.handTo(ScreenOwner::Menu);
    CHECK(t.next(c.snapshot(), ScreenOwner::Menu, 1.0) == PaintAction::Whole);  // the take
    CHECK(t.next(c.snapshot(), ScreenOwner::Menu, 1.1) == PaintAction::Changes);
    CHECK(t.next(c.snapshot(), ScreenOwner::Menu, 1.199) == PaintAction::Changes);
    CHECK(t.next(c.snapshot(), ScreenOwner::Menu, 1.2) == PaintAction::Whole);  // the second paint
    for (double now = 1.21; now < 5.0; now += 0.01) {
        CHECK(t.next(c.snapshot(), ScreenOwner::Menu, now) == PaintAction::Changes);
    }
    c.handTo(ScreenOwner::Menu);  // a phase function re-hands the screen to the same owner
    CHECK(t.next(c.snapshot(), ScreenOwner::Menu, 5.0) == PaintAction::Whole);
    c.handTo(ScreenOwner::Driver);
    CHECK(t.next(c.snapshot(), ScreenOwner::Menu, 5.1) == PaintAction::None);
    c.handTo(ScreenOwner::Menu);  // taken back: whole again, even with no time passed
    CHECK(t.next(c.snapshot(), ScreenOwner::Menu, 5.1) == PaintAction::Whole);
}

// (S3) Bug caught (RC3's order, ND4): a field match walked from the boot to the end — the menu
// after the boot and in competition_initialize() and disabled(), the auton status in autonomous(),
// the driver page from opcontrol()'s take, the menu when the field disables the robot at the end —
// with two painters painting on the same tick anywhere in it, or the right painter NOT repainting
// whole when the screen comes to it.
TEST_CASE("screen (S3): a field match — who paints, in RC3's order, never two at once") {
    Painters p;
    double t = 0.0;
    const auto run = [&](double seconds, PaintAction menu, PaintAction driver, bool status) {
        // The first tick after a handover, then the rest of the phase: one painter only.
        const Painters::Tick first = p.at(t);
        CHECK(onePainter(first));
        CHECK(first.menu == menu);
        CHECK(first.driver == driver);
        CHECK(first.status == status);
        for (double end = t + seconds; t < end; t += 0.01) {
            CHECK(onePainter(p.at(t)));
        }
    };
    p.owner.handTo(ScreenOwner::Menu);  // initialize(): the boot hands the screen to the menu
    run(1.0, PaintAction::Whole, PaintAction::None, false);
    p.owner.handTo(ScreenOwner::Menu);  // competition_initialize(): the field connected
    run(1.0, PaintAction::Whole, PaintAction::None, false);
    p.owner.handTo(ScreenOwner::Menu);  // disabled(): before the match
    run(1.0, PaintAction::Whole, PaintAction::None, false);
    p.owner.handTo(ScreenOwner::AutonStatus);  // autonomous(): the status, then the result
    run(15.0, PaintAction::None, PaintAction::None, true);
    p.owner.handTo(ScreenOwner::Menu);  // disabled(): between the periods
    run(1.0, PaintAction::Whole, PaintAction::None, false);
    p.owner.handTo(ScreenOwner::Driver);  // opcontrol(): after the terminal word
    run(60.0, PaintAction::None, PaintAction::Whole, false);
    p.owner.handTo(ScreenOwner::Menu);  // disabled(): the match is over
    const bool handedBack = p.owner.handOver(ScreenOwner::Driver, ScreenOwner::Menu);
    CHECK_FALSE(handedBack);  // disabled() came first: the loop's hand-back is stale, and refused
    run(1.0, PaintAction::Whole, PaintAction::None, false);
}

// (S4) Bug caught (B2's AUTON ENDING, RC3): a frozen auton past B2's constant — the opcontrol task
// takes the screen (AUTON ENDING) while the auton task still runs; the auton task's result paint
// must then be SKIPPED (the result is only logged), and the driver page must repaint whole when the
// terminal word finally comes (a second take of the same owner).
TEST_CASE("screen (S4): AUTON ENDING — the result paint skipped; the driver page whole at the word") {
    Painters p;
    p.owner.handTo(ScreenOwner::AutonStatus);  // autonomous()
    CHECK(p.at(0.0).status);
    p.owner.handTo(ScreenOwner::Driver);  // opcontrol(): B2's constant passed, AUTON ENDING
    const Painters::Tick ending = p.at(0.3);
    CHECK_FALSE(ending.status);  // the auton task's result paint is refused from here on
    CHECK(ending.driver == PaintAction::Whole);
    CHECK(p.at(0.4).driver == PaintAction::Changes);
    p.owner.handTo(ScreenOwner::Driver);  // the terminal word: the driver page takes it again
    CHECK(p.at(2.0).driver == PaintAction::Whole);
    CHECK_FALSE(p.at(2.0).status);
}

// (S5) Bug caught: a Timed Run or a field that goes from autonomous STRAIGHT to driver control (no
// disabled() between — HA-162 is unmeasured), and a stale hand-back: autonomous() takes the screen
// while the driver loop is ending, and the loop's handOver(Driver -> Menu) must not take it from the
// auton status.
TEST_CASE("screen (S5): autonomous straight to driver control; a stale hand-back never wins") {
    Painters p;
    p.owner.handTo(ScreenOwner::AutonStatus);
    CHECK(p.at(0.0).status);
    p.owner.handTo(ScreenOwner::Driver);  // opcontrol(), no disabled() between
    CHECK(p.at(0.1).driver == PaintAction::Whole);
    CHECK(p.at(0.1).menu == PaintAction::None);
    p.owner.handTo(ScreenOwner::AutonStatus);  // a second autonomous() before the loop ended
    CHECK_FALSE(p.owner.handOver(ScreenOwner::Driver, ScreenOwner::Menu));  // the loop's hand-back
    CHECK(p.owner.holds(ScreenOwner::AutonStatus));
    const Painters::Tick after = p.at(0.2);
    CHECK(after.status);
    CHECK(after.menu == PaintAction::None);
    CHECK(after.driver == PaintAction::None);
}

// (S6) Bug caught: a lost update in the owner word under contention — a handTo() or handOver()
// that loads and stores instead of compare-exchanging (two tasks' takes collapse into one
// generation, so a painter misses its take and paints changes over another page). Four threads
// hammer the cell; the generation must equal the number of handovers that reported success.
TEST_CASE("screen (S6): the owner under contention — every successful handover moves the generation") {
    ScreenOwnerCell c;
    std::atomic<std::uint32_t> landed{0};
    std::vector<std::thread> threads;
    for (int k = 0; k < 4; ++k) {
        threads.emplace_back([&c, &landed, k] {
            for (int i = 0; i < 20000; ++i) {
                const auto to = static_cast<ScreenOwner>((i + k) % 3);
                if ((i + k) % 2 == 0) {
                    c.handTo(to);
                    landed.fetch_add(1);
                } else if (c.handOver(static_cast<ScreenOwner>((i + k + 1) % 3), to)) {
                    landed.fetch_add(1);
                }
            }
        });
    }
    for (std::thread& th : threads) {
        th.join();
    }
    CHECK(c.snapshot().generation == landed.load());
}

// ═══ the menu phase's rule for a touch ═══════════════════════════════════════════════════════

// (S7) Bug caught: a touch ACTED ON while the selector is not the frame on the screen — during a
// take (the frame is being painted now), while another task holds the screen (a tap during driver
// control becoming a choice), or while autons are refused (a choice the next autonomous() would
// refuse anyway) — or a take that does not paint whole, a refusal page never painted, or a refusal
// that arrives while the selector is up leaving the selector there (the fix of 2026-09-30).
// Every input.
TEST_CASE("screen (S7): menuPhaseStep — every input; a touch acts ONLY on the drawn, owned selector") {
    for (const PaintAction a : {PaintAction::None, PaintAction::Whole, PaintAction::Changes}) {
        for (const bool refused : {false, true}) {
            for (const bool menuUp : {false, true}) {
                CAPTURE(static_cast<int>(a));
                CAPTURE(refused);
                CAPTURE(menuUp);
                const MenuPhaseStep s = menuPhaseStep(a, refused, menuUp);
                // Every read is consumed exactly once: blind, or against the drawn frame.
                CHECK(s.consumeBlind != s.actOnDrawn);
                // Acted on only against the selector already up, owned, not a take, not refused.
                CHECK(s.actOnDrawn
                      == (a == PaintAction::Changes && !refused && menuUp));
                // Nothing painted by a painter that does not hold the screen.
                if (a == PaintAction::None) {
                    CHECK_FALSE(s.paintRefused);
                    CHECK_FALSE(s.paintMenuWhole);
                    CHECK_FALSE(s.menuUpAfter);
                }
                // Refused: the selector is never painted, and never left up.
                if (refused) {
                    CHECK_FALSE(s.paintMenuWhole);
                    CHECK_FALSE(s.menuUpAfter);
                    CHECK(s.paintRefused == (a == PaintAction::Whole || (a != PaintAction::None && menuUp)));
                }
                // Not refused and owned: a take (or a selector not yet up) paints it whole.
                if (!refused && a != PaintAction::None) {
                    CHECK(s.paintMenuWhole == (a == PaintAction::Whole || !menuUp));
                    CHECK(s.menuUpAfter);
                    CHECK_FALSE(s.paintRefused);
                }
            }
        }
    }
}

// ═══ the pages ═══════════════════════════════════════════════════════════════════════════════

// (S8) Bug caught: a page that says the wrong thing about a real request — the status page naming
// another auton or promising a run that is refused, the result page's word disagreeing with the
// log's (one source: competitionResultWord), a FINISHED painted in red or an ABANDON in green, the
// END estimate missing, or a line over its budget. Real records from the plant: the 40 in, a
// disable mid-auton, NONE.
TEST_CASE("screen (S8): the status and result pages for real requests — the log's word, in its tone") {
    {
        comp_rig::CompetitionStack s{tank()};
        s.choose(0);  // the 40 in
        const CompetitionRunRecord& r = s.serve();
        REQUIRE(std::string_view{shulib::bench::competitionResultWord(r)} == "FINISHED");
        const CompetitionView st = shulib::bench::competitionStatusView(r, s.registry, "BUILD 12:34:56");
        checkView(st);
        CHECK(std::string{st.header.text} == "AUTON: 40in forward");
        CHECK(std::string{st.big.text} == "AUTON RUNNING");
        CHECK(st.bar == shulib::ui::Tone::Amber);
        CHECK(lineText(st, 0) == "MATCH park T+27.0s stop T+29.0s RELATIVE");
        CHECK(std::string{st.barRight.text} == "BUILD 12:34:56");
        const CompetitionView res = shulib::bench::competitionResultView(r, s.registry, "BUILD 12:34:56");
        checkView(res);
        CHECK(std::string{res.big.text} == "FINISHED");
        CHECK(res.bar == shulib::ui::Tone::Green);
        CHECK(lineText(res, 0) == "ROUTINE_RAN, in time");
        char want[64];
        std::snprintf(want, sizeof want, "end action succeeded; scoring ended T+%.2fs",
                      r.run.guard.scoringEnded.value());
        CHECK(lineText(res, 1) == want);
        CHECK(lineText(res, 2) == "START (0.0, 0.0) 0.0 deg");
        CHECK(r.endEstimate.x().value() > 39.0);  // the END estimate, ~40 in ahead ...
        std::snprintf(want, sizeof want, "END   (%.1f, %.1f) %.1f deg", r.endEstimate.x().value(),
                      r.endEstimate.y().value(), r.endEstimate.heading().degrees());
        CHECK(lineText(res, 3) == want);  // ... is the line, to the tenth
        CHECK(lineText(res, 4) == "faults this run: none");
        CHECK(lineText(res, 5) == "driver control starts when the field says");
    }
    {
        comp_rig::CompetitionStack s{tank()};
        s.choose(1);  // the acceptance run, disabled at 1 s
        const double start = s.now();
        s.status.onRead = [&](double now) {
            s.status.status = now - start >= 1.0 ? comp_rig::kDisabledOnField : comp_rig::kAutonEnabled;
        };
        const CompetitionRunRecord& r = s.serve();
        REQUIRE(r.run.guard.abandoned);
        const CompetitionView res = shulib::bench::competitionResultView(r, s.registry, "BUILD x");
        checkView(res);
        CHECK(std::string{res.big.text} == "ABANDONED");
        CHECK(res.bar == shulib::ui::Tone::Amber);
        CHECK(lineText(res, 0) == "ABANDONED: disabled (status 0x05)");
        CHECK(lineText(res, 1).rfind("end action skipped; scoring ended T+", 0) == 0);
    }
    {
        comp_rig::CompetitionStack s{tank()};  // nothing chosen: NONE
        const CompetitionRunRecord& r = s.serve();
        for (const CompetitionView& v :
             {shulib::bench::competitionStatusView(r, s.registry, "BUILD x"),
              shulib::bench::competitionResultView(r, s.registry, "BUILD x")}) {
            checkView(v);
            CHECK(std::string{v.header.text} == "AUTON: NONE");
            CHECK(std::string{v.big.text} == "NONE");
            CHECK(v.bar == shulib::ui::Tone::Grey);
            CHECK(lineText(v, 0) == "no auton was chosen: nothing moves");
        }
    }
}

// (S9) Bug caught (ND1, ND2): a refused request's pages not saying WHY (a typed-table boot, a program
// error earlier on this boot) or not saying that driver control still runs; a program error's page
// losing the throw's own words or overflowing its budget (a 159-character throw), or promising
// later autons; runAuton's own refusal not shown.
TEST_CASE("screen (S9): refusals and a program error — why, that driver control runs, in budget") {
    AutonRegistry reg{shulib::bench::kAutonPeriods};
    shulib::bench::registerAutons(reg);
    CompetitionRunRecord r{};
    r.request = 3;
    r.entry = 0;
    r.skip = CompetitionSkip::BootFromTable;
    for (const CompetitionView& v : {shulib::bench::competitionStatusView(r, reg, "BUILD x"),
                                     shulib::bench::competitionResultView(r, reg, "BUILD x")}) {
        checkView(v);
        CHECK(std::string{v.big.text} == "REFUSED");
        CHECK(v.bar == shulib::ui::Tone::Red);
        CHECK(lineText(v, 0) == "this boot used the TYPED TABLE: its front");
        CHECK(lineText(v, 1) == "may be the OTHER end -- run shulib Calibrate");
        CHECK(lineText(v, 2) == "driver control still runs");
    }
    r.skip = CompetitionSkip::AfterProgramError;
    for (const CompetitionView& v : {shulib::bench::competitionStatusView(r, reg, "BUILD x"),
                                     shulib::bench::competitionResultView(r, reg, "BUILD x")}) {
        checkView(v);
        CHECK(lineText(v, 0) == "a program error earlier on this boot");
        CHECK(lineText(v, 1) == "restart the program to run autons again");
        CHECK(lineText(v, 2) == "driver control still runs");
    }
    r.skip = CompetitionSkip::None;
    r.threw = true;
    std::memset(r.threwWhat, 'w', sizeof r.threwWhat - 1);  // the longest throw, one "word"
    const CompetitionView thrown = shulib::bench::competitionResultView(r, reg, "BUILD x");
    checkView(thrown);
    CHECK(std::string{thrown.big.text} == "PROGRAM ERROR");
    CHECK(thrown.bar == shulib::ui::Tone::Red);
    CHECK(lineText(thrown, 0) == "a program error -- the robot is stopped");
    CHECK(thrown.lineCount == 6);  // the throw over three lines, then the two sentences
    CHECK(lineText(thrown, 4) == "driver control still runs");
    CHECK(lineText(thrown, 5) == "autons are REFUSED until a restart");
    std::snprintf(r.threwWhat, sizeof r.threwWhat, "%s",
                  "test: a student's std::out_of_range in the fourth step of the auton");
    const CompetitionView words = shulib::bench::competitionResultView(r, reg, "BUILD x");
    checkView(words);
    CHECK(lineText(words, 1) == "test: a student's std::out_of_range in the fourth");
    CHECK(lineText(words, 2) == "step of the auton");
    r.threw = false;
    r.run.outcome = AutonRunOutcome::Refused;
    std::snprintf(r.run.why, sizeof r.run.why, "%s",
                  "auton '40in forward' refused: a route auton needs a command registry to fire "
                  "its markers, and this program passes none");
    const CompetitionView refused = shulib::bench::competitionResultView(r, reg, "BUILD x");
    checkView(refused);
    CHECK(std::string{refused.big.text} == "REFUSED");
    CHECK(lineText(refused, refused.lineCount - 1) == "nothing moved");
}

// (S10) Bug caught: the pages at the WORST case of every field — a 29-character auton name, huge
// numbers, an estimate far off the field, an ignored abandon — cutting silently (unsafeLines counts
// a line over budget BEFORE the safety copy cuts it), dropping a line past the eighth, or painting
// into the rows VEXos keeps.
TEST_CASE("screen (S10): every page at the worst case of every field — nothing cut, nothing dropped") {
    AutonRegistry reg{shulib::bench::kAutonPeriods};
    shulib::competition::AutonEntry e{};
    e.name = "abcdefghijklmnopqrstuvwxyz012";  // 29: the name budget
    e.kind = shulib::competition::AutonKind::Skills;
    e.routine = [](shulib::competition::AutonContext& ctx) {
        shulib::chassis::Routine r{ctx.chassis, "x"};
        return r.result();
    };
    e.start = shulib::competition::StartPose::relative();
    e.endActionAt = Time{57.0};
    e.hardStopAt = Time{59.0};
    reg.add(e);
    CompetitionRunRecord r{};
    r.request = 4294967295u;
    r.entry = 0;
    r.run.outcome = AutonRunOutcome::RoutineRan;
    r.cause = shulib::bench::CompetitionCause::LeftAutonomous;
    r.statusAtLatch = 0xFF;
    r.run.guard.endActionRan = true;
    r.run.guard.scoringEnded = Time{9999.99};
    r.run.guard.pacesSeen = 2147483647;
    r.run.seated = shulib::math::Pose2d{shulib::units::Length{-9999.9}, shulib::units::Length{-9999.9},
                                        shulib::math::Angle{}};
    r.endEstimate = r.run.seated;
    r.faultsBefore = 0;  // a fault COUNT: never negative (a negative one would overflow the
    r.faultsAfter = 2147483647;  // difference -- undefined behaviour, not a worst case)
    r.lastFault = shulib::diag::FaultCode::MotionTimeout;
    for (const bool abandoned : {false, true}) {
        CAPTURE(abandoned);
        r.run.guard.abandoned = abandoned;  // false: an IGNORED abandon (latched, never acted on)
        const CompetitionView res = shulib::bench::competitionResultView(r, reg, "BUILD 23:59:59");
        checkView(res);
        CHECK(std::string{res.header.text} == "AUTON: abcdefghijklmnopqrstuvwxyz012");
        if (!abandoned) {
            CHECK(std::string{res.big.text} == "ABANDON IGNORED");
            CHECK(res.bar == shulib::ui::Tone::Red);
            CHECK(lineText(res, 0) == "abandon IGNORED: left autonomous");
            CHECK(res.lines[0].tone == shulib::ui::Tone::Red);
        }
    }
    checkView(shulib::bench::competitionStatusView(r, reg, "BUILD 23:59:59"));
    const CompetitionView ending =
        shulib::bench::competitionEndingView("abcdefghijklmnopqrstuvwxyz012", 99999.99, "BUILD x");
    checkView(ending);
    CHECK(std::string{ending.big.text} == "AUTON ENDING");
    CHECK(lineText(ending, 1) == "still running: 'abcdefghijklmnopqrstuvwxyz012'");
    CHECK(lineText(ending, 2) == "not handed back within 99999.99 s");
}

// (S11) Bug caught (ND1, ND2 on the driver page; B11): the driver page's AUTON row not saying that
// autons are refused (a program error, a typed-table boot) — the driver would expect the next
// autonomous() to run — or naming the wrong request's word; the controller row wrong; the rows out
// of Teleop's place or order.
TEST_CASE("screen (S11): the driver page — the AUTON row's four states; the rows in order") {
    AutonRegistry reg{shulib::bench::kAutonPeriods};
    shulib::bench::registerAutons(reg);
    CompetitionRunRecord last{};
    CHECK(std::string{shulib::bench::competitionAutonRow(last, reg, false, true).text}
          == "auton: none requested on this boot");
    CHECK(std::string{shulib::bench::competitionAutonRow(last, reg, true, true).text}
          == "AUTONS REFUSED: a program error -- restart");
    CHECK(shulib::bench::competitionAutonRow(last, reg, true, true).tone == shulib::ui::Tone::Red);
    CHECK(std::string{shulib::bench::competitionAutonRow(last, reg, false, false).text}
          == "AUTONS REFUSED: typed-table boot -- Calibrate");
    last.request = 1;
    last.entry = 0;
    last.run.outcome = AutonRunOutcome::RoutineRan;
    last.run.routine.ok = true;
    last.run.guard.endActionSucceeded = true;
    const shulib::bench::PanelRow ok = shulib::bench::competitionAutonRow(last, reg, false, true);
    CHECK(std::string{ok.text} == "auton FINISHED: 40in forward");
    CHECK(ok.tone == shulib::ui::Tone::Green);
    shulib::bench::CompetitionDriverFacts f{};
    f.auton = ok;
    f.controller = shulib::bench::controllerRowText(false);
    f.faultRow = "faults: none";
    f.clawRow = "claws: 3 built";
    f.buildStamp = "BUILD 01:02:03";
    const CompetitionView v = shulib::bench::competitionDriverView(f);
    checkView(v);
    REQUIRE(v.lineCount == 4);
    CHECK(std::string{v.header.text} == "shulib COMPETITION");
    CHECK(lineText(v, 0) == "auton FINISHED: 40in forward");
    CHECK(lineText(v, 1) == "NO CONTROLLER -- the drive gets 0 V");
    CHECK(v.lines[1].tone == shulib::ui::Tone::Red);
    CHECK(lineText(v, 2) == "faults: none");
    CHECK(v.lines[2].tone == shulib::ui::Tone::Green);
    CHECK(lineText(v, 3) == "claws: 3 built");
    CHECK(v.lines[0].y == shulib::bench::kCompetitionDriverRowY0);
    CHECK(v.lines[3].y == shulib::bench::kCompetitionDriverRowY0 + 3 * shulib::ui::kSmallHeight);
}

// (S12) Bug caught: a line over its budget CUT SILENTLY — the safety copy cuts it either way, but a
// page must COUNT it (unsafeLines), or a sentence the program built too long would lose its end on
// the screen and every test of the page would still pass. The driver page's rows come from the
// program's own facts (the claw and fault rows are formatted elsewhere), so a row handed in over
// the SMALL budget, or carrying a '%', must be counted — and still painted cut and safe.
TEST_CASE("screen (S12): a row over budget, or not screen-safe, is COUNTED, never cut silently") {
    shulib::bench::CompetitionDriverFacts f{};
    f.auton.tone = shulib::ui::Tone::White;
    const std::string longRow(shulib::ui::kSmallCols + 9, 'x');  // 59 characters
    f.faultRow = longRow.c_str();
    f.clawRow = "claws: 100% ok";  // a '%' (PROS re-reads printed text as a format)
    const CompetitionView v = shulib::bench::competitionDriverView(f);
    CHECK(v.unsafeLines == 2);
    REQUIRE(v.lineCount == 4);
    CHECK(std::strlen(v.lines[2].text) == shulib::ui::kSmallCols);  // cut at the budget
    CHECK(shulib::ui::screenText(v.lines[2].text, shulib::ui::kSmallCols));
    CHECK(shulib::ui::screenText(v.lines[3].text, shulib::ui::kSmallCols));  // made safe
    f.faultRow = "faults: none";
    f.clawRow = "claws: ok";
    CHECK(shulib::bench::competitionDriverView(f).unsafeLines == 0);
}

// (S13) Bug caught (the review's RV14, RV16): a result word painted in the wrong colour -- CUT BY
// DEADLINE (the clock ended it, not a fault) in red, a PROGRAM ERROR in amber, TOO LATE in red --
// and the driver page's AUTON row hiding a program error behind a typed-table boot (both refuse
// autons; the program error is the newer, louder fact, and "restart" is what fixes it first).
TEST_CASE("screen (S13): every result word's tone; the AUTON row's precedence when both refuse") {
    using shulib::bench::competitionResultTone;
    using shulib::ui::Tone;
    CHECK(competitionResultTone("FINISHED") == Tone::Green);
    CHECK(competitionResultTone("NONE") == Tone::Grey);
    for (const char* amber : {"ABANDONED", "CUT BY DEADLINE", "TOO LATE"}) {
        CAPTURE(amber);
        CHECK(competitionResultTone(amber) == Tone::Amber);
    }
    for (const char* red : {"PROGRAM ERROR", "REFUSED", "ABANDON IGNORED", "NOT STARTED",
                            "HARD STOP", "ROUTINE STOPPED"}) {
        CAPTURE(red);
        CHECK(competitionResultTone(red) == Tone::Red);
    }
    AutonRegistry reg{shulib::bench::kAutonPeriods};
    shulib::bench::registerAutons(reg);
    const CompetitionRunRecord last{};
    const shulib::bench::PanelRow both = shulib::bench::competitionAutonRow(last, reg, true, false);
    CHECK(std::string{both.text} == "AUTONS REFUSED: a program error -- restart");
    CHECK(both.tone == Tone::Red);
}

// (S14) Bug caught (the review's PB1, on the screen): a request served after its period painted as a
// run ("AUTON RUNNING") or as a refusal, or without the status that ended the period -- TOO LATE,
// amber, what the status said, that nothing moved and driver control still runs, on the status page
// and the result page alike; and the driver page's AUTON row naming it.
TEST_CASE("screen (S14): TOO LATE -- the status and result pages, and the driver page's AUTON row") {
    comp_rig::CompetitionStack s{tank()};
    s.choose(0);
    s.status.status = comp_rig::kDisabledOnField;
    const CompetitionRunRecord& r = s.serve();
    REQUIRE(r.skip == CompetitionSkip::TooLate);
    for (const CompetitionView& v : {shulib::bench::competitionStatusView(r, s.registry, "BUILD x"),
                                     shulib::bench::competitionResultView(r, s.registry, "BUILD x")}) {
        checkView(v);
        CHECK(std::string{v.header.text} == "AUTON: 40in forward");
        CHECK(std::string{v.big.text} == "TOO LATE");
        CHECK(v.bar == shulib::ui::Tone::Amber);
        CHECK(lineText(v, 0) == "autonomous was over when the auton task served it");
        CHECK(lineText(v, 1) == "the status said: disabled (status 0x05)");
        CHECK(lineText(v, 2) == "nothing moved");
        CHECK(lineText(v, 3) == "driver control still runs");
    }
    const shulib::bench::PanelRow row = shulib::bench::competitionAutonRow(r, s.registry, false, true);
    CHECK(std::string{row.text} == "auton TOO LATE: 40in forward");
    CHECK(row.tone == shulib::ui::Tone::Amber);
}
