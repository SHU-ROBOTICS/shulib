// AF1S-A1 — the Toggle step's EDGES on the host test plant (competition/toggle_visit.hpp), added by the
// AF1S-A1 review round (2026-11-11): the paths and limits the first tests left unpinned — the camera seen
// frozen on the drive in (with the position gate on as well, and a camera that hiccups and recovers; the
// verification round, 2026-11-12: with a maxImageAge longer than the drive in — the HOLD),
// a drive that does not arrive (in or out), the motion cancelled on a cut, the
// PASSIVE bar and the PNEUMATIC beyond their happy paths, the position gate's wiring, the retract-now
// threshold from both sides, landed exactly, and its skipped settle, `settle` honoured, a one-push visit's time check
// and rule 4's `>=` landed EXACTLY (remaining() equal to need, to the last bit),
// the run-time limits landed close (the wall window, the field distance, the arm's "at back"), and rule
// 6's lens-travel clause. Same rig as the scenes (test/toggle_visit_rig.hpp); every scene judges the
// step by the PLANT as well as its report. ADVERSARIAL: each case tries to break ONE rule. Say "host
// test plant": nothing here has run on a robot, and every number is a placeholder.

#include "doctest.h"

#include <cmath>
#include <string>
#include <vector>

#include "toggle_visit_rig.hpp"

using namespace tv_rig;
using shulib::control::ExitReason;
using shulib::manipulation::MechanismOutcome;
using sc::ToggleVisitVerdict;
using vision::ToggleColour;
using vision::ToggleWhy;

namespace {

mech::PositionSlot& armSlot(Rig& r) { return *r.svc.find(tv_fixture::flipArm); }
mech::PneumaticSlot& pistonSlot(Rig& r) { return *r.svc.find(tv_fixture::flipPiston); }

Options faces(Options o, std::uint8_t start, std::uint8_t next, std::uint8_t third) {
    o.toggle.fieldId = start;
    o.toggle.bottomId = next;
    o.toggle.outerId = third;
    return o;
}

// ONE [TGL] line, ASCII, <= 160 bytes, Info only for the two successes (the scenes' check).
void oneLine(const Rig& r, const sc::ToggleVisitReport& rep) {
    const std::vector<std::string> l = r.lines("TGL");
    REQUIRE(l.size() == 1U);
    CHECK(l[0].size() <= 160U);
    for (const char c : l[0]) {
        CHECK(static_cast<unsigned char>(c) >= 0x20);
        CHECK(static_cast<unsigned char>(c) < 0x7F);
    }
    CHECK((r.lastLevel("TGL") == shulib::hal::LogLevel::Info) == rep.ok());
}

// The tracker's POSITION gate on, and a pose bound stated (so the step hands it over).
void gateOn(Rig& r, double tolIn, double poseIn) {
    r.cam.tracker.positionToleranceIn = tolIn;
    r.cam.tracker.maxPoseToleranceIn = 6.0;
    r.cam.tracker.maxFixRadiusIn = 8.0;
    r.cam.poseTolerance = Length{poseIn};
    r.cam.headingToleranceDeg = 1.0;
}

// The look before's own duration on this rig (a lookOnly run), seconds.
double lookSeconds(const sc::ToggleVisitRequest& q) {
    Rig r{scene(Form::Arm)};
    return r.visit(q, Rig::entry(), 55.0, /*lookOnly=*/true).seconds;
}

}  // namespace

// ═══ The camera seen FROZEN before the push (the review's correctness finding) ═══════════════════════

TEST_CASE("AF1S-A1 T3 (review): the camera FROZEN while parked BEFORE the look, the Toggle turned OURS under it (our "
          "other robot) — the stale picture says YELLOW; the drive in moves the lens and shows it frozen -> FROZEN "
          "BEFORE PUSH: nothing pushed, NO contact while ours, the Toggle still scores ours") {
    Options o = faces(scene(Form::Arm), kYellow, kRed, kBlue);  // one roll: Y -> R (ours)
    Rig r{o};
    (void)r.camera.read();  // the camera's last picture: YELLOW
    r.camPlant.setState(sim::CameraState::Frozen);
    r.toggle.rollNow();
    for (int i = 0; i < 50; ++i) {
        r.toggle.step(0.01, 0.0, 30.0);
    }
    r.redraw();
    REQUIRE(r.toggle.fieldFace() == kRed);
    REQUIRE(r.toggle.seated());
    const std::uint32_t seq0 = armSlot(r).commandSeq();
    const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
    CHECK(rep.before == ToggleColour::Yellow);  // the stale look (the tracker's documented limit, parked)
    CHECK(rep.verdict == ToggleVisitVerdict::FrozenBeforePush);
    CHECK(rep.outcome() == MechanismOutcome::Unconfirmed);
    CHECK(rep.pushes == 0);
    CHECK(armSlot(r).commandSeq() == seq0);
    CHECK(r.toggle.flipperContactsWhileOurs() == 0);
    CHECK(r.toggle.strokes() == 1);  // the other robot's only
    CHECK(r.toggle.scoredColour() == kRed);
    oneLine(r, rep);
}

TEST_CASE("AF1S-A1 T3 (review): a camera frozen before push 2's drive in (after push 1's look counted) -> FROZEN "
          "BEFORE PUSH after 1 push; push 2 never made") {
    Rig r{scene(Form::Arm)};  // Y -> B -> R: push 1 gives the third colour
    const int m0 = r.chassis.scheduler().motionsStarted();
    bool frozen = false;
    r.world.onPace = [&] {
        if (!frozen && r.chassis.scheduler().motionsStarted() - m0 == 3) {  // push 2's drive in has begun
            frozen = true;
            r.camPlant.setState(sim::CameraState::Frozen);
        }
    };
    const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
    CHECK(frozen);
    CHECK(rep.verdict == ToggleVisitVerdict::FrozenBeforePush);
    CHECK(rep.pushes == 1);
    CHECK(r.toggle.strokes() == 1);
    CHECK(r.toggle.fieldFace() == kBlue);
    oneLine(r, rep);
    CHECK(r.lines("TGL")[0].find("push 2 not made") != std::string::npos);
}

TEST_CASE("AF1S-A1 T3 (review, the fix's re-check): WITH THE POSITION GATE ON (a pose bound stated) — the camera "
          "frozen while parked before the look, the Toggle turned OURS under it -> still FROZEN BEFORE PUSH, no "
          "contact while ours (the gate drops a frozen picture's drifting band as 'no band' before the tracker "
          "can say 'camera frozen?': the frozen check runs on an ungated twin)") {
    for (const double tol : {0.25, 1.0, 3.0}) {
        for (const double age : {0.1, 1.0}) {
            CAPTURE(tol);
            CAPTURE(age);
            Rig r{faces(scene(Form::Arm), kYellow, kRed, kBlue)};
            (void)r.camera.read();
            r.camPlant.setState(sim::CameraState::Frozen);
            r.toggle.rollNow();
            for (int i = 0; i < 50; ++i) {
                r.toggle.step(0.01, 0.0, 30.0);
            }
            r.redraw();
            REQUIRE(r.toggle.fieldFace() == kRed);
            gateOn(r, tol, 1.0);
            sc::ToggleVisitRequest q = armRequest();
            q.maxImageAge = Time{age};
            const std::uint32_t seq0 = armSlot(r).commandSeq();
            const sc::ToggleVisitReport rep = r.visit(q, Rig::entry());
            CHECK(rep.positionChecked);  // the look before WAS position-checked
            CHECK(rep.before == ToggleColour::Yellow);
            CHECK(rep.verdict == ToggleVisitVerdict::FrozenBeforePush);
            CHECK(rep.pushes == 0);
            CHECK(armSlot(r).commandSeq() == seq0);
            CHECK(r.toggle.flipperContactsWhileOurs() == 0);
            CHECK(r.toggle.scoredColour() == kRed);
            oneLine(r, rep);
        }
    }
}

// ═══ The verification round (2026-11-12): a maxImageAge LONGER than the drive in ═════════════════════
// The frozen check needs "camera frozen?" to hold on a picture maxImageAge newer than the read that raised
// it. With a generous maxImageAge (HA-192 is unmeasured: a team may type a large one "to be safe") the
// drive in ARRIVES before that — and the step pushed (the arm) or had already rolled the Toggle (the
// passive bar, whose drive in IS the stroke), a Toggle that was ours. Now the drive in HOLDS (rule 5).

TEST_CASE("AF1S-A1 T3 (verification): the camera FROZEN while parked, the Toggle turned OURS under it, with a "
          "maxImageAge LONGER than the drive in (1.5 and 2.0 s; the drive in takes ~1.3 s) — ARM, PNEUMATIC and "
          "PASSIVE, gate off and on -> FROZEN BEFORE PUSH, no contact while ours, the Toggle still scores ours") {
    for (const Form f : {Form::Arm, Form::Pneumatic, Form::Passive}) {
        for (const bool gate : {false, true}) {
            for (const double age : {0.1, 1.0, 1.5, 2.0}) {
                CAPTURE(static_cast<int>(f));
                CAPTURE(gate);
                CAPTURE(age);
                Rig r{faces(scene(f), kYellow, kRed, kBlue)};
                (void)r.camera.read();
                r.camPlant.setState(sim::CameraState::Frozen);
                r.toggle.rollNow();
                for (int i = 0; i < 50; ++i) {
                    r.toggle.step(0.01, 0.0, 30.0);
                }
                r.redraw();
                REQUIRE(r.toggle.fieldFace() == kRed);
                if (gate) {
                    gateOn(r, 1.0, 1.0);
                }
                sc::ToggleVisitRequest q = f == Form::Arm         ? armRequest()
                                           : f == Form::Pneumatic ? pistonRequest()
                                                                  : passiveRequest();
                q.maxImageAge = Time{age};
                const int m0 = r.chassis.scheduler().motionsStarted();
                const sc::ToggleVisitReport rep =
                    r.visitWith(f == Form::Passive ? nullptr : &r.svc, q, Rig::entry());
                CHECK(rep.before == ToggleColour::Yellow);
                CHECK(rep.verdict == ToggleVisitVerdict::FrozenBeforePush);
                CHECK(rep.pushes == 0);
                CHECK(r.toggle.flipperContactsWhileOurs() == 0);
                CHECK(r.toggle.strokes() == 1);  // the other robot's only
                CHECK(r.toggle.scoredColour() == kRed);
                CHECK(r.chassis.scheduler().motionsStarted() - m0 == 2);  // in (stopped or held), out
                if (f != Form::Passive && age >= 1.5) {
                    // The drive in ARRIVED (settled) before the evidence was stale: the HOLD at the push
                    // spot caught it, not the stop on the way in.
                    CHECK(rep.driveIn == ExitReason::Settled);
                }
                if (f == Form::Passive) {
                    CHECK(rep.driveIn == ExitReason::Cancelled);  // stopped short of the Toggle
                }
                oneLine(r, rep);
            }
        }
    }
}

TEST_CASE("AF1S-A1 T3 (verification): the ARM's hold at the push spot, the camera LIVE again during it (frozen from "
          "the drive in's start until 0.1 s after it arrives, maxImageAge 1.5 s; and until 1.75 s after it arrives, "
          "maxImageAge 3 s — past HALF the hold's budget) -> the push waits for the camera to be believed, then "
          "FLIPPED in 2 pushes") {
    struct Case {
        double age;
        double liveAfter;
    };
    for (const Case c : {Case{1.5, 0.1}, Case{3.0, 1.75}}) {
        CAPTURE(c.age);
        CAPTURE(c.liveAfter);
        Rig r{scene(Form::Arm)};  // Y -> B -> R
        sc::ToggleVisitRequest q = armRequest();
        q.maxImageAge = Time{c.age};
        q.lookWithin = Time{1.6};
        const int m0 = r.chassis.scheduler().motionsStarted();
        const std::uint32_t seq0 = armSlot(r).commandSeq();
        double arrivedAt = -1.0;
        double liveAt = -1.0;
        double pushAt = -1.0;
        bool frozen = false;
        r.world.onPace = [&] {
            const int m = r.chassis.scheduler().motionsStarted() - m0;
            if (m == 1 && !frozen) {
                frozen = true;
                r.camPlant.setState(sim::CameraState::Frozen);
            }
            if (m == 1 && arrivedAt < 0.0 && !r.chassis.scheduler().hasActiveMotion()) {
                arrivedAt = r.now();
            }
            if (arrivedAt >= 0.0 && liveAt < 0.0 && r.now() - arrivedAt >= c.liveAfter) {
                liveAt = r.now();
                r.camPlant.setState(sim::CameraState::Normal);
            }
            if (pushAt < 0.0 && armSlot(r).commandSeq() != seq0) {
                pushAt = r.now();
            }
        };
        const sc::ToggleVisitReport rep = r.visit(q, Rig::entry(), 55.0);
        REQUIRE(arrivedAt >= 0.0);
        REQUIRE(liveAt >= 0.0);
        REQUIRE(pushAt >= 0.0);
        CHECK(pushAt >= liveAt);       // nothing pushed while the question was open
        CHECK(pushAt - liveAt < 0.2);  // ... and the hold ends as the camera is believed, not at its bound
        CHECK(rep.verdict == ToggleVisitVerdict::Flipped);
        CHECK(rep.pushes == 2);
        CHECK(r.toggle.scoredColour() == kRed);
        oneLine(r, rep);
    }
}

TEST_CASE("AF1S-A1 T3 (verification): the PASSIVE bar's RESUMED drive in gets only what is LEFT of driveBound — "
          "paused 0.2 s in (frozenMoveIn 5 in), live again, driveBound 1.2 s: the two legs' MOVING time stays within "
          "driveBound, the resumed leg times out short of the push spot (no full stroke, never FLIPPED)") {
    Rig r{scene(Form::Passive)};
    r.cam.tracker.frozenMoveIn = 5.0;  // the excursion, 12 in, is still more than twice it
    sc::ToggleVisitRequest q = passiveRequest();
    q.maxImageAge = Time{0.6};
    q.driveBound = Time{1.2};
    const int m0 = r.chassis.scheduler().motionsStarted();
    double startAt = -1.0;
    double pausedAt = -1.0;
    double resumedAt = -1.0;
    double resumedEnd = -1.0;
    int phase = 0;
    r.world.onPace = [&] {
        const int m = r.chassis.scheduler().motionsStarted() - m0;
        const bool active = r.chassis.scheduler().hasActiveMotion();
        if (phase == 0 && m == 1) {
            phase = 1;
            startAt = r.now();
            r.camPlant.setState(sim::CameraState::Frozen);
        }
        if (phase == 1 && m == 1 && !active) {
            phase = 2;
            pausedAt = r.now();
        }
        if (phase == 2 && r.now() - pausedAt >= 0.1) {
            phase = 3;
            r.camPlant.setState(sim::CameraState::Normal);
        }
        if (phase == 3 && m == 2) {
            phase = 4;
            resumedAt = r.now();
        }
        if (phase == 4 && m == 2 && !active) {
            phase = 5;
            resumedEnd = r.now();
        }
    };
    const sc::ToggleVisitReport rep = r.visitWith(nullptr, q, Rig::entry(), 55.0);
    REQUIRE(phase == 5);
    const double moving = (pausedAt - startAt) + (resumedEnd - resumedAt);
    CHECK(moving <= 1.2 + 0.011);  // driveBound, to the tick: the resumed leg is not given a fresh one
    CHECK(rep.driveIn == ExitReason::TimedOut);
    CHECK_FALSE(rep.fullStroke[0]);
    CHECK(rep.verdict != ToggleVisitVerdict::Flipped);
    oneLine(r, rep);
}

TEST_CASE("AF1S-A1 T3 (verification): the PASSIVE bar's hold budget SPENT ACROSS two holds — frozen 0.2 s at the drive "
          "in's start (held, believed again, driven on), then frozen for good 0.05 s into the resumed drive -> the "
          "second hold runs out with 'camera frozen?' still open: FROZEN BEFORE PUSH, the Toggle never touched") {
    for (const double age : {0.6, 1.5}) {
        CAPTURE(age);
        Rig r{scene(Form::Passive)};
        sc::ToggleVisitRequest q = passiveRequest();
        q.maxImageAge = Time{age};
        const int m0 = r.chassis.scheduler().motionsStarted();
        double frozeAt = -1.0;
        double resumedAt = -1.0;
        int phase = 0;
        r.world.onPace = [&] {
            const int m = r.chassis.scheduler().motionsStarted() - m0;
            if (phase == 0 && m == 1) {
                phase = 1;
                frozeAt = r.now();
                r.camPlant.setState(sim::CameraState::Frozen);
            }
            if (phase == 1 && r.now() - frozeAt >= 0.2) {
                phase = 2;
                r.camPlant.setState(sim::CameraState::Normal);
            }
            if (phase == 2 && m == 2) {
                phase = 3;
                resumedAt = r.now();
            }
            if (phase == 3 && r.now() - resumedAt >= 0.05) {
                phase = 4;
                r.camPlant.setState(sim::CameraState::Frozen);
            }
        };
        const sc::ToggleVisitReport rep = r.visitWith(nullptr, q, Rig::entry(), 55.0);
        CHECK(phase == 4);
        CHECK(rep.verdict == ToggleVisitVerdict::FrozenBeforePush);
        CHECK(rep.pushes == 0);
        CHECK(r.toggle.strokes() == 0);
        CHECK(r.toggle.fieldFace() == kYellow);
        CHECK(r.chassis.scheduler().motionsStarted() - m0 == 3);  // in (stopped), in (resumed, stopped), out
        oneLine(r, rep);
    }
}

TEST_CASE("AF1S-A1 T3 (verification): the PASSIVE bar's drive in STOPS on 'camera frozen?' raised, HOLDS, and — the "
          "camera live again — drives ON within what is left of driveBound: a full stroke, FLIPPED in 2 pushes "
          "(one extra motion: the resumed drive in)") {
    for (const double age : {0.6, 1.5}) {
        for (const double hiccup : {0.2, 0.5}) {
            CAPTURE(age);
            CAPTURE(hiccup);
            Rig r{scene(Form::Passive)};
            sc::ToggleVisitRequest q = passiveRequest();
            q.maxImageAge = Time{age};
            q.lookWithin = Time{1.6};
            const int m0 = r.chassis.scheduler().motionsStarted();
            double frozeAt = -1.0;
            bool done = false;
            int contactsFrozen = 0;
            r.world.onPace = [&] {
                if (!done && r.chassis.scheduler().motionsStarted() - m0 == 1) {
                    done = true;
                    frozeAt = r.now();
                    r.camPlant.setState(sim::CameraState::Frozen);
                }
                if (frozeAt >= 0.0 && r.toggle.strokes() > 0) {
                    ++contactsFrozen;  // the bar reached the Toggle while the camera was still frozen
                }
                if (frozeAt >= 0.0 && r.now() - frozeAt >= hiccup) {
                    r.camPlant.setState(sim::CameraState::Normal);
                    frozeAt = -1.0;
                }
            };
            const sc::ToggleVisitReport rep = r.visitWith(nullptr, q, Rig::entry(), 55.0);
            CHECK(rep.verdict == ToggleVisitVerdict::Flipped);
            CHECK(rep.pushes == 2);
            CHECK(rep.fullStroke[0]);
            CHECK(rep.fullStroke[1]);
            CHECK(contactsFrozen == 0);
            CHECK(r.toggle.scoredColour() == kRed);
            CHECK(r.chassis.scheduler().motionsStarted() - m0 == 5);  // in (stopped), in (resumed), out, in, out
            oneLine(r, rep);
        }
    }
}

TEST_CASE("AF1S-A1 T3 (verification) PINNED LIMIT: a PASSIVE bar that reaches the Toggle before the lens has moved "
          "frozenMoveIn (contact from 14-15 in, a 16-in look spot, frozenMoveIn 2 in) strokes a Toggle that was ours on "
          "the stale look — and the verdict still says FROZEN BEFORE PUSH; from 12 in it is stopped in time "
          "(guide 14 states this; a fix turns this test red)") {
    for (const double rollIn : {12.0, 14.0, 15.0}) {
        for (const double age : {0.1, 1.5}) {
            CAPTURE(rollIn);
            CAPTURE(age);
            Options o = faces(scene(Form::Passive), kYellow, kRed, kBlue);
            o.toggle.passiveRollIn = rollIn;
            Rig r{o};
            (void)r.camera.read();
            r.camPlant.setState(sim::CameraState::Frozen);
            r.toggle.rollNow();
            for (int i = 0; i < 50; ++i) {
                r.toggle.step(0.01, 0.0, 30.0);
            }
            r.redraw();
            REQUIRE(r.toggle.fieldFace() == kRed);
            sc::ToggleVisitRequest q = passiveRequest();
            q.maxImageAge = Time{age};
            const sc::ToggleVisitReport rep = r.visitWith(nullptr, q, Rig::entry());
            CHECK(rep.verdict == ToggleVisitVerdict::FrozenBeforePush);
            CHECK(rep.pushes == 0);
            CHECK(r.toggle.flipperContactsWhileOurs() == (rollIn > 13.0 ? 1 : 0));
            CHECK(r.toggle.scoredColour() == (rollIn > 13.0 ? kBlue : kRed));
        }
    }
}

TEST_CASE("AF1S-A1 T3 (review, the fix's re-check): WITH THE POSITION GATE ON — a camera frozen as the drive in "
          "begins -> FROZEN BEFORE PUSH; frozen on the way OUT (at 12.5 in, a band in view) to a look spot 20 in out "
          "-> UNKNOWN AFTER with the reason 'camera frozen?' (the gated tracker alone drops the drifting band as "
          "'no band')") {
    for (const int where : {1, 2}) {
        CAPTURE(where);
        Options o = scene(Form::Arm);
        sc::ToggleVisitRequest q = armRequest();
        if (where == 2) {  // 7.5 in of travel after the freeze: far enough for the gate to drop the stale band
            o.lookFrom = 20.0;
            q.lookFrom = Length{20.0};
            q.offBy = Length{9.0};
            q.driveBound = Time{3.0};
        }
        Rig r{o};
        gateOn(r, 0.25, 1.0);
        const int m0 = r.chassis.scheduler().motionsStarted();
        bool frozen = false;
        r.world.onPace = [&] {
            const int m = r.chassis.scheduler().motionsStarted() - m0;
            const bool go = where == 1 ? m == 1 && r.chassis.scheduler().hasActiveMotion()
                                       : m == 2 && r.frontTo(r.toggleConfig()) >= 12.5;  // a band in view
            if (!frozen && go) {
                frozen = true;
                r.camPlant.setState(sim::CameraState::Frozen);
            }
        };
        const sc::ToggleVisitReport rep = r.visit(q, Rig::entry());
        CHECK(frozen);
        if (where == 1) {
            CHECK(rep.verdict == ToggleVisitVerdict::FrozenBeforePush);
            CHECK(rep.pushes == 0);
            CHECK(r.toggle.strokes() == 0);
        } else {
            CHECK(rep.verdict == ToggleVisitVerdict::UnknownAfter);
            CHECK(rep.pushes == 1);
            CHECK(rep.whyAfter == ToggleWhy::CameraFrozen);
        }
        oneLine(r, rep);
    }
}

TEST_CASE("AF1S-A1 T3 (review): a camera that HICCUPS — frozen 0.2-0.5 s at the start of EACH drive in, shorter than "
          "maxImageAge, then live again — is believed again: FLIPPED after 2 pushes, never FROZEN BEFORE PUSH on "
          "push 2 from push 1's cleared episode") {
    for (const double hiccup : {0.2, 0.5}) {
        CAPTURE(hiccup);
        Rig r{scene(Form::Arm)};
        sc::ToggleVisitRequest q = armRequest();
        q.maxImageAge = Time{0.6};
        q.lookWithin = Time{1.6};
        const int m0 = r.chassis.scheduler().motionsStarted();
        int lastM = 0;
        int episodes = 0;
        double frozeAt = -1.0;
        r.world.onPace = [&] {
            const int m = r.chassis.scheduler().motionsStarted() - m0;
            if (m != lastM) {
                lastM = m;
                if (m % 2 == 1) {  // a drive in begins
                    r.camPlant.setState(sim::CameraState::Frozen);
                    frozeAt = r.now();
                    ++episodes;
                }
            }
            if (frozeAt >= 0.0 && r.now() - frozeAt >= hiccup) {
                r.camPlant.setState(sim::CameraState::Normal);
                frozeAt = -1.0;
            }
        };
        const sc::ToggleVisitReport rep = r.visit(q, Rig::entry());
        CHECK(episodes == 2);
        CHECK(rep.verdict == ToggleVisitVerdict::Flipped);
        CHECK(rep.pushes == 2);
        CHECK(r.toggle.scoredColour() == kRed);
        oneLine(r, rep);
    }
}

// ═══ DRIVE FAILED — in and out ══════════════════════════════════════════════════════════════════════

TEST_CASE("AF1S-A1 T7 (review): the drive IN blocked (the robot cannot move: something in its way) -> DRIVE FAILED "
          "'in': nothing pushed, the flipper never commanded, the Toggle untouched, and the drive OUT still made") {
    Rig r{scene(Form::Arm)};
    const int m0 = r.chassis.scheduler().motionsStarted();
    r.world.onPace = [&] {
        if (r.chassis.scheduler().hasActiveMotion() && r.chassis.scheduler().motionsStarted() - m0 == 1) {
            r.stopDrive();
        }
    };
    const std::uint32_t seq0 = armSlot(r).commandSeq();
    const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
    CHECK(rep.verdict == ToggleVisitVerdict::DriveFailed);
    CHECK(std::string{rep.detail} == "in");
    CHECK(rep.driveIn == ExitReason::TimedOut);
    CHECK(rep.driveOut == ExitReason::Settled);
    CHECK(rep.pushes == 0);
    CHECK(rep.outcome() == MechanismOutcome::Unconfirmed);
    CHECK(armSlot(r).commandSeq() == seq0);
    CHECK(r.toggle.strokes() == 0);
    CHECK(r.chassis.scheduler().motionsStarted() - m0 == 2);  // in, then out
    oneLine(r, rep);
    CHECK(r.lines("TGL")[0] == "right wall Toggle: the drive in did not arrive (timed out) after 0 pushes");
}

TEST_CASE("AF1S-A1 T7 (review): the drive OUT blocked after push 1 -> DRIVE FAILED 'out': no look after (it would not "
          "be from the look spot), no push 2") {
    Rig r{scene(Form::Arm)};
    const int m0 = r.chassis.scheduler().motionsStarted();
    r.world.onPace = [&] {
        if (r.chassis.scheduler().hasActiveMotion() && r.chassis.scheduler().motionsStarted() - m0 == 2) {
            r.stopDrive();
        }
    };
    const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
    CHECK(rep.verdict == ToggleVisitVerdict::DriveFailed);
    CHECK(std::string{rep.detail} == "out");
    CHECK(rep.driveIn == ExitReason::Settled);
    CHECK(rep.driveOut == ExitReason::TimedOut);
    CHECK(rep.pushes == 1);
    CHECK(rep.after == ToggleColour::Unknown);
    CHECK(rep.whyAfter == ToggleWhy::NoRead);  // no look after was made
    CHECK(r.toggle.strokes() == 1);
    CHECK(r.chassis.scheduler().motionsStarted() - m0 == 2);  // no push 2's drive in
    CHECK(r.frontTo(r.toggleConfig()) < 6.0);               // where the block left it
    oneLine(r, rep);
    CHECK(r.lines("TGL")[0] == "right wall Toggle: the drive out did not arrive (timed out) after 1 push");
}

// ═══ R3: the motion cancelled on every exit ═════════════════════════════════════════════════════════

TEST_CASE("AF1S-A1 T6 (review, R3): the run's deadline lands MID-DRIVE-IN (a loop stall leaves 0.05 s) -> CUT, and "
          "the step's drive is NOT left running in the scheduler when the step returns") {
    Rig r{scene(Form::Arm)};
    bool jumped = false;
    r.world.onPace = [&] {
        if (!jumped && r.chassis.scheduler().hasActiveMotion()) {
            jumped = true;
            r.rig.h.clock().advance(Time{r.guard.remaining().value() - 0.05});
        }
    };
    sc::ToggleVisitReport rep{};
    bool active = true;
    r.runGuarded([&] {
        rep = sc::checkAndFlipToggle(r.cam, r.chassis, r.guard, &r.svc, Rig::entry(), armRequest(), r.sink);
        active = r.chassis.scheduler().hasActiveMotion();
    });
    CHECK(jumped);
    CHECK(rep.verdict == ToggleVisitVerdict::Cut);
    CHECK(rep.outcome() == MechanismOutcome::Cancelled);
    CHECK(rep.pushes == 0);
    CHECK_FALSE(active);
    CHECK(r.toggle.strokes() == 0);
    oneLine(r, rep);
}

// ═══ The PASSIVE bar beyond its happy path ══════════════════════════════════════════════════════════

TEST_CASE("AF1S-A1 T6 (review, R7) PASSIVE: the loop stalls while the bar is at the Toggle, leaving 2.0 s -> no "
          "mechanism touched (there is none: no service at all), NO settle, the drive out at once, RETRACTED FOR "
          "TIME, the robot off the Toggle") {
    Options o = scene(Form::Passive);
    o.toggle.touchIn = 15.0;
    Rig r{o};
    sc::ToggleVisitRequest q = passiveRequest();
    q.settle = Time{0.6};
    q.margin = Time{0.1};
    const int m0 = r.chassis.scheduler().motionsStarted();
    bool jumped = false;
    double inEnd = -1.0;  // the last pace the drive in was still live
    double outStart = -1.0;
    r.world.onPace = [&] {
        shulib::motion::MotionScheduler& s = r.chassis.scheduler();
        if (!jumped && r.guard.running() && r.toggle.strokes() >= 1) {
            jumped = true;
            r.rig.h.clock().advance(Time{r.guard.remaining().value() - 2.0});
        }
        if (s.motionsStarted() - m0 == 1 && s.hasActiveMotion()) {
            inEnd = r.now();
        }
        if (outStart < 0.0 && s.motionsStarted() - m0 == 2) {
            outStart = r.now();
        }
    };
    sc::ToggleVisitReport rep{};
    bool touched = true;
    double left = -1.0;
    r.runGuarded([&] {
        rep = sc::checkAndFlipToggle(r.cam, r.chassis, r.guard, nullptr, Rig::entry(), q, r.sink);
        touched = r.toggle.touched();
        left = r.guard.remaining().value();
    });
    CHECK(jumped);
    CHECK(rep.verdict == ToggleVisitVerdict::RetractedForTime);
    CHECK(rep.outcome() == MechanismOutcome::TimedOut);
    CHECK_FALSE(touched);
    CHECK(left > 0.0);
    REQUIRE(inEnd >= 0.0);
    REQUIRE(outStart >= 0.0);
    CHECK(outStart - inEnd < 0.05);  // no 0.6-s settle
    oneLine(r, rep);
}

TEST_CASE("AF1S-A1 T14 (review, R1) PASSIVE: the drive in STOPPED SHORT of the push spot (blocked at 7 in) after the "
          "bar already rolled the Toggle to ours -> OURS AFTER A SHORT STROKE, never FLIPPED") {
    Options o = faces(scene(Form::Passive), kYellow, kRed, kBlue);
    o.toggle.passiveRollIn = 8.0;  // this bar reaches the Toggle from 8 in
    Rig r{o};
    const int m0 = r.chassis.scheduler().motionsStarted();
    r.world.onPace = [&] {
        if (r.chassis.scheduler().hasActiveMotion() && r.chassis.scheduler().motionsStarted() - m0 == 1
            && r.frontTo(r.toggleConfig()) <= 7.0) {
            r.stopDrive();
        }
    };
    const sc::ToggleVisitReport rep = r.visitWith(nullptr, passiveRequest(), Rig::entry());
    CHECK(rep.verdict == ToggleVisitVerdict::OursAfterShortStroke);
    CHECK(rep.outcome() == MechanismOutcome::Unconfirmed);
    CHECK_FALSE(rep.fullStroke[0]);
    CHECK(rep.driveIn == ExitReason::TimedOut);
    CHECK(rep.after == ToggleColour::Red);
    CHECK(r.toggle.strokes() == 1);
    oneLine(r, rep);
}

TEST_CASE("AF1S-A1 T8 (review): the PNEUMATIC and the PASSIVE forms through the rule's rows — none, sticky, over-roll "
          "(red-side Y -> B -> R, ours RED): the verdict and pushes as the rule says, never a push on ours") {
    struct Row {
        sim::ToggleMode mode;
        ToggleVisitVerdict verdict;
        int pushes;
    };
    for (const Form f : {Form::Pneumatic, Form::Passive}) {
        for (const Row row : {Row{sim::ToggleMode::Normal, ToggleVisitVerdict::Flipped, 2},
                              Row{sim::ToggleMode::Sticky, ToggleVisitVerdict::NoChange, 1},
                              Row{sim::ToggleMode::OverRoll, ToggleVisitVerdict::Flipped, 1}}) {
            CAPTURE(static_cast<int>(f));
            CAPTURE(static_cast<int>(row.mode));
            Options o = scene(f);
            o.toggle.mode = row.mode;
            Rig r{o};
            const sc::ToggleVisitReport rep = f == Form::Passive ? r.visitWith(nullptr, passiveRequest(), Rig::entry())
                                                                 : r.visit(pistonRequest(), Rig::entry());
            CHECK(rep.verdict == row.verdict);
            CHECK(rep.pushes == row.pushes);
            CHECK(r.toggle.strokes() <= 2);
            CHECK(r.toggle.flipperContactsWhileOurs() == 0);
            if (rep.ok()) {
                CHECK(r.toggle.scoredColour() == kRed);
            }
            oneLine(r, rep);
        }
    }
}

// ═══ The PNEUMATIC: readiness, its bounds, and what it cannot sense ═════════════════════════════════

TEST_CASE("AF1S-A1 T9 (review): a PNEUMATIC left at 'flip', or still running a step -> FLIPPER NOT READY with its own "
          "reason; nothing commanded, nothing read") {
    for (const bool running : {false, true}) {
        CAPTURE(running);
        Rig r{scene(Form::Pneumatic)};
        pistonSlot(r).set(tv_fixture::flipPiston["flip"].position);
        if (!running) {
            for (int i = 0; i < 100 && pistonSlot(r).running(); ++i) {
                r.svc.pace();
            }
            REQUIRE_FALSE(pistonSlot(r).running());
        } else {
            REQUIRE(pistonSlot(r).running());
        }
        const std::uint32_t seq0 = pistonSlot(r).commandSeq();
        const sc::ToggleVisitReport rep = r.visit(pistonRequest(), Rig::entry());
        CHECK(rep.verdict == ToggleVisitVerdict::FlipperNotReady);
        CHECK(std::string{rep.detail}.find(running ? "still running" : "not at its back position") != std::string::npos);
        CHECK(pistonSlot(r).commandSeq() == seq0);
        CHECK(r.feed.stats().reads == 0U);
        oneLine(r, rep);
    }
}

TEST_CASE("AF1S-A1 T9 (review): an ARM still running a step -> FLIPPER NOT READY 'still running' (not the at-back "
          "reason)") {
    Rig r{scene(Form::Arm)};
    armSlot(r).moveTo(0.0, Time{2.0}, false);  // a step toward where it already is: running for its settle
    REQUIRE(armSlot(r).running());
    const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
    CHECK(rep.verdict == ToggleVisitVerdict::FlipperNotReady);
    CHECK(std::string{rep.detail}.find("still running") != std::string::npos);
}

TEST_CASE("AF1S-A1 T6 (review): a PNEUMATIC's flipBound / backBound against its slot's own wait (move time + confirm "
          "window): equal -> it runs; a tick under, either one -> FLIPPER NOT READY before anything is read (rule 4 "
          "must never count less than the step waits)") {
    double slotBound = 0.0;
    {
        Rig r{scene(Form::Pneumatic)};
        slotBound = pistonSlot(r).lastStepBound();
    }
    REQUIRE(slotBound > 0.0);
    struct Case {
        double flip;
        double back;
        bool ready;
    };
    for (const Case c : {Case{slotBound, slotBound, true}, Case{slotBound - 0.01, slotBound, false},
                         Case{slotBound, slotBound - 0.01, false}}) {
        CAPTURE(c.flip);
        CAPTURE(c.back);
        Rig r{scene(Form::Pneumatic)};
        sc::ToggleVisitRequest q = pistonRequest();
        q.flipBound = Time{c.flip};
        q.backBound = Time{c.back};
        const sc::ToggleVisitReport rep = r.visit(q, Rig::entry());
        if (c.ready) {
            CHECK(rep.verdict == ToggleVisitVerdict::Flipped);
        } else {
            CHECK(rep.verdict == ToggleVisitVerdict::FlipperNotReady);
            CHECK(std::string{rep.detail}.find("longer than flipBound or backBound") != std::string::npos);
            CHECK(r.feed.stats().reads == 0U);
        }
        oneLine(r, rep);
    }
}

TEST_CASE("AF1S-A1 T7 (review) PINNED: a PNEUMATIC's stroke is not SENSED — a cylinder blocked half-way out reports "
          "its push SUCCEEDED (a 'full stroke'); one caught on the way back reports its retract SUCCEEDED and the "
          "robot drives out (never FLIPPER STUCK). The stated limit (banner, guide 14, HA-235)") {
    {
        Rig r{scene(Form::Pneumatic)};
        r.pistonMax = 0.5;  // blocked at 45 degrees: short of the roll
        const sc::ToggleVisitReport rep = r.visit(pistonRequest(), Rig::entry());
        CHECK(rep.push[0].outcome == MechanismOutcome::Succeeded);
        CHECK(rep.fullStroke[0]);
        CHECK(rep.verdict == ToggleVisitVerdict::NoChange);
        CHECK(r.toggle.strokes() == 0);
        oneLine(r, rep);
    }
    {
        Rig r{scene(Form::Pneumatic)};
        r.pistonMin = 0.6;  // caught at 54 degrees on the way back
        const int m0 = r.chassis.scheduler().motionsStarted();
        const sc::ToggleVisitReport rep = r.visit(pistonRequest(), Rig::entry());
        CHECK(rep.back[0].outcome == MechanismOutcome::Succeeded);
        CHECK(rep.verdict != ToggleVisitVerdict::FlipperStuck);
        CHECK(r.chassis.scheduler().motionsStarted() - m0 >= 2);  // it drove out with the cylinder still out
        CHECK(r.pistonOut >= 0.6);
        oneLine(r, rep);
    }
}

// ═══ The position gate's wiring ═════════════════════════════════════════════════════════════════════

TEST_CASE("AF1S-A1 T1 (review, R5): the step hands the stated pose bound to the tracker's POSITION gate — with the true "
          "Toggle the view says position-checked only when a bound is stated; a lookalike plate 6 in in front of the "
          "wall is refused BY THE GATE (no band) with a bound, and reaches containment (NOT AT THE SPOT) without") {
    auto gateOn = [](Rig& r, double tolIn, double poseIn) {
        r.cam.tracker.positionToleranceIn = tolIn;
        r.cam.tracker.maxPoseToleranceIn = 6.0;
        r.cam.tracker.maxFixRadiusIn = 8.0;
        r.cam.poseTolerance = Length{poseIn};
        r.cam.headingToleranceDeg = poseIn > 0.0 ? 1.0 : 0.0;
    };
    for (const double pose : {1.0, 0.0}) {  // the TRUE Toggle; the gate configured on, a bound stated or not
        CAPTURE(pose);
        Rig r{scene(Form::Arm)};
        gateOn(r, 3.0, pose);
        const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
        CHECK(rep.verdict == ToggleVisitVerdict::Flipped);
        CHECK(rep.positionChecked == (pose > 0.0));
        oneLine(r, rep);
    }
    {  // the gate off entirely: never checked
        Rig r{scene(Form::Arm)};
        const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
        CHECK_FALSE(rep.positionChecked);
    }
    for (const double pose : {1.0, 0.0}) {  // a BLUE lookalike at the face's height, 6 in in front; the faces unseen
        CAPTURE(pose);
        Options o = faces(scene(Form::Arm), kRed, kYellow, kBlue);
        o.drawToggle = false;
        Rig r{o};
        sim::ColourQuad q;
        q.c = {sim::P3{64.2, -13.0, 12.4}, sim::P3{64.2, 13.0, 12.4}, sim::P3{64.2, 13.0, 13.4},
               sim::P3{64.2, -13.0, 13.4}};
        q.colourId = kBlue;
        r.extras.push_back(q);
        r.redraw();
        gateOn(r, 0.25, pose);
        const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
        if (pose > 0.0) {
            CHECK(rep.verdict == ToggleVisitVerdict::UnknownBefore);
            CHECK(rep.whyBefore == ToggleWhy::NoBand);
        } else {
            CHECK(rep.verdict == ToggleVisitVerdict::NotAtSpot);
            CHECK(rep.before == ToggleColour::Blue);
        }
        CHECK(r.toggle.strokes() == 0);
        oneLine(r, rep);
    }
}

// ═══ Retract-now from both sides of its threshold, and `settle` ═════════════════════════════════════

TEST_CASE("AF1S-A1 T6 (review, R7): retract-now's threshold backBound + settle + driveBound + margin (3.4 s) from BOTH "
          "sides — a loop stall while the push is out leaving 0.05 s MORE -> no retract-now (the stall also runs out "
          "the push's own wait, so `back` replaces it the normal way, then the settle, and the visit goes on); 0.05 s "
          "LESS -> RETRACTED FOR TIME, back at once, and the drive out begins with NO settle") {
    for (const double side : {+0.05, -0.05}) {
        CAPTURE(side);
        Options o = scene(Form::Arm);
        o.toggle.touchIn = 15.0;
        Rig r{o};
        sc::ToggleVisitRequest q = armRequest();
        q.settle = Time{0.6};
        q.margin = Time{0.1};
        const double threshold = q.backBound.value() + q.settle.value() + q.driveBound.value() + q.margin.value();
        const int m0 = r.chassis.scheduler().motionsStarted();
        const std::uint32_t steps0 = armSlot(r).stepsBegun();
        bool jumped = false;
        double backEnd = -1.0;
        double outStart = -1.0;
        r.world.onPace = [&] {
            if (!jumped && r.guard.running() && r.armPlant.trueDeg() >= 35.0) {
                jumped = true;
                r.rig.h.clock().advance(Time{r.guard.remaining().value() - (threshold + side)});
            }
            if (backEnd < 0.0 && armSlot(r).stepsBegun() >= steps0 + 2U && !armSlot(r).running()) {
                backEnd = r.now();
            }
            if (outStart < 0.0 && r.chassis.scheduler().motionsStarted() - m0 == 2) {
                outStart = r.now();
            }
        };
        const sc::ToggleVisitReport rep = r.visit(q, Rig::entry());
        REQUIRE(jumped);
        REQUIRE(backEnd >= 0.0);
        REQUIRE(outStart >= 0.0);
        if (side > 0.0) {
            CHECK(rep.verdict == ToggleVisitVerdict::NoChange);         // the push stopped short: no roll
            CHECK(outStart - backEnd >= q.settle.value() - 0.011);       // the settle kept
        } else {
            CHECK(rep.verdict == ToggleVisitVerdict::RetractedForTime);
            CHECK(outStart - backEnd < 0.05);                            // no settle
        }
        CHECK(rep.push[0].outcome == MechanismOutcome::Cancelled);  // replaced by `back`, never left RUNNING
        CHECK(rep.back[0].outcome == MechanismOutcome::Succeeded);
        oneLine(r, rep);
    }
}

TEST_CASE("AF1S-A1 T6 (review): retract-now landed EXACTLY on its threshold — remaining() EQUAL to backBound + settle "
          "+ driveBound + margin (bounds chosen exact in binary: 3.375 s) while the push is out -> RETRACTED FOR "
          "TIME (the check retracts at the threshold: `>` to carry on, not `>=`)") {
    Options o = scene(Form::Arm);
    o.toggle.touchIn = 15.0;
    Rig r{o};
    sc::ToggleVisitRequest q = armRequest();
    q.flipBound = Time{1.25};
    q.backBound = Time{1.25};
    q.settle = Time{0.5};
    q.driveBound = Time{1.5};
    q.margin = Time{0.125};
    const double threshold = q.backBound.value() + q.settle.value() + q.driveBound.value() + q.margin.value();
    REQUIRE(threshold == 3.375);
    int phase = 0;
    double seen = -1.0;
    r.world.onPace = [&] {
        if (phase == 1) {  // the clock as the step's next check reads it (nothing moves it between paces)
            seen = r.guard.remaining().value();
            phase = 2;
        }
        if (phase == 0 && r.guard.running() && r.armPlant.trueDeg() >= 35.0) {
            phase = 1;
            // Jump so that, after this pace's own 10-ms step, deadline - now IS the threshold to the bit.
            const double n = r.now();
            const double deadline = n + r.guard.remaining().value();
            double a = deadline - threshold - 0.01 - n;
            for (int i = 0; i < 200 && deadline - ((n + a) + 0.01) != threshold; ++i) {
                a = deadline - ((n + a) + 0.01) > threshold ? std::nextafter(a, 1e9) : std::nextafter(a, 0.0);
            }
            r.rig.h.clock().advance(Time{a});
        }
    };
    const sc::ToggleVisitReport rep = r.visit(q, Rig::entry());
    REQUIRE(phase == 2);
    CHECK(seen == threshold);  // the edge itself, not near it
    CHECK(rep.verdict == ToggleVisitVerdict::RetractedForTime);
    CHECK_FALSE(r.toggle.touched());
    oneLine(r, rep);
}

TEST_CASE("AF1S-A1 T6 (review, HA-232): `settle` is honoured — after every retract the drive out waits at least "
          "`settle`") {
    Rig r{scene(Form::Arm)};
    sc::ToggleVisitRequest q = armRequest();
    q.settle = Time{0.5};
    const int m0 = r.chassis.scheduler().motionsStarted();
    std::vector<double> backEnds;
    std::vector<double> outStarts;
    const std::uint32_t steps0 = armSlot(r).stepsBegun();
    std::uint32_t seenSteps = steps0;
    bool backRunning = false;
    int lastStarted = 0;
    r.world.onPace = [&] {
        const std::uint32_t steps = armSlot(r).stepsBegun();
        if (steps != seenSteps) {
            seenSteps = steps;
            backRunning = ((steps - steps0) % 2U) == 0U;  // push, back, push, back
        }
        if (backRunning && !armSlot(r).running()) {
            backRunning = false;
            backEnds.push_back(r.now());
        }
        const int started = r.chassis.scheduler().motionsStarted() - m0;
        if (started != lastStarted) {
            lastStarted = started;
            if (started % 2 == 0) {
                outStarts.push_back(r.now());
            }
        }
    };
    const sc::ToggleVisitReport rep = r.visit(q, Rig::entry());
    CHECK(rep.verdict == ToggleVisitVerdict::Flipped);
    REQUIRE(backEnds.size() == 2U);
    REQUIRE(outStarts.size() == 2U);
    for (std::size_t i = 0; i < 2U; ++i) {
        CAPTURE(i);
        CHECK(outStarts[i] - backEnds[i] >= q.settle.value() - 0.011);
    }
}

// ═══ Rule 4 with maxPushes = 1 ══════════════════════════════════════════════════════════════════════

TEST_CASE("AF1S-A1 T6 (review): maxPushes = 1 — the time left just over need(1) and well under need(2) -> it proceeds "
          "(a one-push visit is checked against need(1)), and FLIPPED in one push") {
    sc::ToggleVisitRequest q = armRequest();
    q.maxPushes = 1;
    const double look = lookSeconds(q);
    const double need1 = sc::toggleVisitNeedSeconds(q, 1);
    REQUIRE(sc::worstCaseSeconds(q) == doctest::Approx(need1));
    REQUIRE(need1 + 0.001 < sc::toggleVisitNeedSeconds(q, 2));
    Rig r{faces(scene(Form::Arm), kYellow, kRed, kBlue)};  // one roll to ours
    const sc::ToggleVisitReport rep = r.visit(q, Rig::entry(), look + need1 + 0.001);
    CHECK(rep.verdict == ToggleVisitVerdict::Flipped);
    CHECK(rep.pushes == 1);
    CHECK(rep.secondsNeeded == doctest::Approx(need1));
    CHECK(r.toggle.scoredColour() == kRed);
}

TEST_CASE("AF1S-A1 T6 (review): rule 4 landed EXACTLY — remaining() at the check EQUAL to need(1), to the last bit, "
          "proceeds (>=, not >) and FLIPS; the next deadline down (fewer seconds left than need) is NOT ENOUGH TIME") {
    sc::ToggleVisitRequest q = armRequest();
    q.maxPushes = 1;  // one check only: the report keeps the check's own reading (push 2 would overwrite it)
    const double look = lookSeconds(q);
    const double need1 = sc::toggleVisitNeedSeconds(q, 1);
    // The deadline whose remaining() at the check IS need1: the check records what it read, so walk the
    // deadline one representable step at a time until it lands (on this rig look + need1 lands at once).
    double end = look + need1;
    sc::ToggleVisitReport rep{};
    int tries = 0;
    for (; tries < 64; ++tries) {
        Rig r{faces(scene(Form::Arm), kYellow, kRed, kBlue)};
        rep = r.visit(q, Rig::entry(), end);
        if (rep.secondsLeftAtCheck == rep.secondsNeeded) {
            CHECK(rep.verdict == ToggleVisitVerdict::Flipped);
            CHECK(rep.pushes == 1);
            CHECK(r.toggle.scoredColour() == kRed);
            break;
        }
        end = rep.secondsLeftAtCheck < rep.secondsNeeded ? std::nextafter(end, 1e9) : std::nextafter(end, 0.0);
    }
    REQUIRE(tries < 64);
    CHECK(rep.secondsLeftAtCheck == rep.secondsNeeded);  // the edge itself, not near it
    CHECK(rep.secondsNeeded == doctest::Approx(need1));
    // Just under: step the deadline down until the reading drops below need1 (a few representable steps).
    sc::ToggleVisitReport under{};
    for (int k = 0; k < 64; ++k) {
        end = std::nextafter(end, 0.0);
        Rig r{faces(scene(Form::Arm), kYellow, kRed, kBlue)};
        const std::uint32_t seq0 = armSlot(r).commandSeq();
        under = r.visit(q, Rig::entry(), end);
        if (under.secondsLeftAtCheck < under.secondsNeeded) {
            CHECK(armSlot(r).commandSeq() == seq0);
            break;
        }
    }
    CHECK(under.secondsLeftAtCheck < under.secondsNeeded);
    CHECK(under.secondsLeftAtCheck > under.secondsNeeded - 1e-9);
    CHECK(under.verdict == ToggleVisitVerdict::NotEnoughTime);
}

// ═══ The run-time limits, landed close ══════════════════════════════════════════════════════════════

TEST_CASE("AF1S-A1 T5 (review, R6): the wall window +-45 degrees — facing 44.9 degrees off the named wall passes the "
          "wall check; 45.1 -> WRONG WALL before anything is read") {
    for (const double h : {44.9, -44.9, 45.1, -45.1}) {
        CAPTURE(h);
        Options o = scene(Form::Arm);
        o.headingDeg = h;
        Rig r{o};
        const sc::ToggleVisitReport rep =
            r.visit(armRequest(), Rig::entry(sc::AutonKind::Skills, sc::StartPose::field(r.truth())));
        if (std::fabs(h) < 45.0) {
            CHECK(rep.verdict != ToggleVisitVerdict::WrongWall);
            CHECK(r.feed.stats().reads > 0U);
        } else {
            CHECK(rep.verdict == ToggleVisitVerdict::WrongWall);
            CHECK(r.feed.stats().reads == 0U);
        }
        CHECK(r.toggle.strokes() == 0);
    }
}

TEST_CASE("AF1S-A1 T5 (review, R2): the FIELD start's distance check — the estimate (offBy - 0.05) in back from the "
          "truth still passes (and flips: the camera agrees); (offBy + 0.05) in back -> NOT AT THE SPOT 'estimate'") {
    for (const double extra : {-0.05, +0.05}) {
        CAPTURE(extra);
        Rig r{scene(Form::Arm)};
        const sc::ToggleVisitRequest q = armRequest();
        const shulib::math::Pose2d p = r.truth();
        r.chassis.setPose(shulib::math::Pose2d{p.x() - Length{q.offBy.value() + extra}, p.y(), p.heading()});
        const sc::ToggleVisitReport rep = r.visit(q, Rig::entry(sc::AutonKind::Skills, sc::StartPose::field(p)));
        CHECK(rep.lensFromWallIn == doctest::Approx(q.lookFrom.value() + q.offBy.value() + extra).epsilon(0.002));
        if (extra < 0.0) {
            CHECK(rep.verdict == ToggleVisitVerdict::Flipped);
        } else {
            CHECK(rep.verdict == ToggleVisitVerdict::NotAtSpot);
            CHECK(r.toggle.strokes() == 0);
            CHECK(r.lines("TGL")[0].find("estimate") != std::string::npos);
        }
        oneLine(r, rep);
    }
}

TEST_CASE("AF1S-A1 T9 (review): the arm 'at back' within ITS tolerance only — left 2.5 x its tolerance from `back` "
          "-> FLIPPER NOT READY, never moved") {
    Rig r{scene(Form::Arm)};
    const double tol = armSlot(r).spec().position.tolerance;
    armSlot(r).moveTo(2.5 * tol, Time{2.0}, false);
    for (int i = 0; i < 300 && armSlot(r).running(); ++i) {
        r.svc.pace();
    }
    REQUIRE(armSlot(r).position() > tol);
    REQUIRE(armSlot(r).position() < 10.0 * tol);
    const std::uint32_t seq0 = armSlot(r).commandSeq();
    const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
    CHECK(rep.verdict == ToggleVisitVerdict::FlipperNotReady);
    CHECK(std::string{rep.detail}.find("not at its back position") != std::string::npos);
    CHECK(armSlot(r).commandSeq() == seq0);
}

// ═══ Rule 6's lens-travel clause — reached only with a settle tolerance wider than the excursion ═════

TEST_CASE("AF1S-A1 T3 (review, rule 6): a chassis whose translation settle accepts 11 in (wider than the 12-in "
          "excursion less frozenMoveIn) 'arrives' at once both ways, so the lens has NOT moved past frozenMoveIn "
          "since the retract -> UNKNOWN AFTER 'no picture after the retract', never the unchanged YELLOW counted") {
    shulib::motion::MotionConfig m = motion_rig::motionConfig();
    m.translationSettle.maxError = 11.0;
    m.translationSettle.maxErrorRate = 100.0;
    Options o = scene(Form::Arm);
    o.motion = &m;
    Rig r{o};
    const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
    CHECK(rep.verdict == ToggleVisitVerdict::UnknownAfter);
    CHECK(rep.afterTooEarly);
    CHECK(rep.after == ToggleColour::Unknown);
    oneLine(r, rep);
    CHECK(r.lines("TGL")[0].find("no picture after the retract") != std::string::npos);
}
