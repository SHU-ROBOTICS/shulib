// AF1S-A1 — the Toggle step END TO END on the host test plant (competition/toggle_visit.hpp; the AF1S brief's
// §2.2 T1–T11 and T14). A 12-in tank under a REAL Chassis whose pacer chain is the program's (Chassis ->
// RunGuard -> MechanismService -> the world), its front camera read through the host camera model at the drive
// plant's TRUE pose, three flippers bound (an arm, a lift, a pneumatic) and the host Toggle (sim/toggle_plant.hpp)
// coupled to the flipper's TRUE angle and the robot's TRUE front — test/toggle_visit_rig.hpp. Every scene judges
// the step by the PLANT (its true face, scoredColour(), its strokes and contacts), never by the step's own
// report alone. ADVERSARIAL: each case tries to break ONE rule. Say "host test plant": nothing here has run on a
// robot, and every number is a placeholder the team replaces from Step 0.

#include "doctest.h"

#include <cmath>
#include <string>
#include <vector>

#include "shulib/chassis/routine.hpp"
#include "toggle_visit_rig.hpp"

using namespace tv_rig;
using shulib::manipulation::MechanismOutcome;
using sc::ToggleVisitVerdict;
using vision::ToggleColour;
using vision::ToggleWhy;

namespace {

// The arm's slot, and its command number (nothing commanded = unchanged).
mech::PositionSlot& armSlot(Rig& r) { return *r.svc.find(tv_fixture::flipArm); }

// Every visit writes ONE [TGL] line, ASCII, at most 160 bytes, Info only for the two successes (T11) — checked
// after every scene that calls it.
void oneLine(const Rig& r, const sc::ToggleVisitReport& rep) {
    const std::vector<std::string> l = r.lines("TGL");
    REQUIRE(l.size() == 1U);
    CHECK(l[0].size() <= 160U);
    for (const char c : l[0]) {
        CHECK(static_cast<unsigned char>(c) >= 0x20);
        CHECK(static_cast<unsigned char>(c) < 0x7F);
    }
    CHECK((r.lastLevel("TGL") == shulib::hal::LogLevel::Info) == rep.ok());
    const bool says = l[0].find("seating not checked") != std::string::npos;
    if (rep.ok()) {  // R1: every success line ENDS "seating not checked"
        CHECK(l[0].size() >= 19U);
        CHECK(l[0].substr(l[0].size() - 19) == "seating not checked");
    } else {
        CHECK_FALSE(says);
    }
}

// Faces for a Toggle starting `start`, the next face up `next` (the roll order), the third `third`.
Options faces(Options o, std::uint8_t start, std::uint8_t next, std::uint8_t third) {
    o.toggle.fieldId = start;
    o.toggle.bottomId = next;
    o.toggle.outerId = third;
    return o;
}

}  // namespace

// ═══ T1 — never a push on OUR colour ════════════════════════════════════════════════════════════════

TEST_CASE("AF1S-A1 T1: a RED Quadrant's Toggle showing RED, and a BLUE Quadrant's showing BLUE -> ALREADY OURS; the "
          "flipper never commanded, the drive never started, the Toggle never touched") {
    struct Case {
        ov::ToggleId wall;
        std::uint8_t ours;
    };
    for (const Case c : {Case{ov::toggles::kRightWall, kRed}, Case{ov::toggles::kLeftWall, kBlue}}) {
        CAPTURE(static_cast<int>(c.wall));
        Options o = faces(scene(Form::Arm, c.wall), c.ours, kYellow, c.ours == kRed ? kBlue : kRed);
        Rig r{o};
        const std::uint32_t seq0 = armSlot(r).commandSeq();
        const int m0 = r.chassis.scheduler().motionsStarted();
        const sc::ToggleVisitReport rep = r.visit(armRequest(c.wall), Rig::entry());
        CHECK(rep.verdict == ToggleVisitVerdict::AlreadyOurs);
        CHECK(rep.outcome() == MechanismOutcome::Succeeded);
        CHECK(rep.pushes == 0);
        CHECK(armSlot(r).commandSeq() == seq0);
        CHECK(r.chassis.scheduler().motionsStarted() == m0);
        CHECK(r.toggle.strokes() == 0);
        CHECK(r.toggle.flipperContactsWhileOurs() == 0);
        CHECK(r.toggle.scoredColour() == c.ours);
        oneLine(r, rep);
    }
}

TEST_CASE("AF1S-A1 T1: push 1 turns it OURS (an over-rolling Toggle) -> FLIPPED after ONE push, never a second") {
    Options o = scene(Form::Arm);
    o.toggle.mode = sim::ToggleMode::OverRoll;  // Y -> (B) -> R in one stroke
    Rig r{o};
    const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
    CHECK(rep.verdict == ToggleVisitVerdict::Flipped);
    CHECK(rep.pushes == 1);
    CHECK(r.toggle.strokes() == 1);
    CHECK(r.toggle.scoredColour() == kRed);
    CHECK(r.toggle.flipperContactsWhileOurs() == 0);
    oneLine(r, rep);
}

TEST_CASE("AF1S-A1 T1: seeded scenes (camera noise, both roll orders, sticky, over-roll, our OTHER robot having flipped "
          "it before the visit) — the plant counts NO contact while its TRUE seated face is ours, and never a third "
          "push") {
    int visits = 0;
    for (const std::uint32_t seed : {3U, 11U, 29U}) {
        for (const int order : {0, 1}) {
            for (const sim::ToggleMode mode : {sim::ToggleMode::Normal, sim::ToggleMode::Sticky,
                                               sim::ToggleMode::OverRoll}) {
                for (const bool otherFirst : {false, true}) {  // our other robot flipped it before the look
                    CAPTURE(seed);
                    CAPTURE(order);
                    CAPTURE(static_cast<int>(mode));
                    CAPTURE(otherFirst);
                    Options o = order == 0 ? faces(scene(Form::Arm), kYellow, kBlue, kRed)
                                           : faces(scene(Form::Arm), kYellow, kRed, kBlue);
                    o.seed = seed;
                    o.jitterPx = 1.5;
                    o.toggle.mode = mode;
                    Rig r{o};
                    const bool rolled = otherFirst;
                    if (otherFirst) {
                        r.toggle.rollNow();
                        for (int i = 0; i < 50; ++i) {
                            r.toggle.step(0.01, 0.0, 30.0);
                        }
                        r.redraw();
                    }
                    const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
                    ++visits;
                    CHECK(r.toggle.flipperContactsWhileOurs() == 0);
                    CHECK(rep.pushes <= 2);
                    CHECK(r.toggle.strokes() - (rolled ? 1 : 0) <= 2);
                    if (rep.verdict == ToggleVisitVerdict::Flipped) {
                        CHECK(r.toggle.scoredColour() == kRed);
                    }
                    oneLine(r, rep);
                }
            }
        }
    }
    CHECK(visits == 36);
}

TEST_CASE("AF1S-A1 T1 PINNED (a stated limit): our OTHER robot flips the Toggle to OURS while this robot drives IN "
          "(after its look, before its push) — the step cannot see it (the camera names no colour close in) and "
          "pushes our Toggle; the route keeps the other robot away from a Toggle being visited") {
    Options o = faces(scene(Form::Arm), kYellow, kRed, kBlue);  // one roll: Y -> R (ours)
    Rig r{o};
    bool rolled = false;
    r.world.onPace = [&] {
        if (!rolled && r.chassis.scheduler().hasActiveMotion()) {
            rolled = true;
            r.toggle.rollNow();
        }
    };
    const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
    CHECK(rolled);
    CHECK(r.toggle.flipperContactsWhileOurs() == 1);
    CHECK(rep.pushes >= 1);
    oneLine(r, rep);
}

TEST_CASE("AF1S-A1 T1 (R5): a LOOKALIKE while the true face is OURS — the Toggle's own faces unseen (HA-199's miss), "
          "a BLUE plate where the camera can take it for the face. PINNED as it is: at the face's height, 4 in in "
          "front of the wall, the step PUSHES OUR Toggle away, with the position gate OFF and ON (its 3-in "
          "tolerance plus the fix radius admits 4 in); 1.4 in lower, CONTAINMENT refuses the band, nothing pushed") {
    auto plate = [](double x, double z0) {
        sim::ColourQuad q;
        q.c = {sim::P3{x, -13.0, z0}, sim::P3{x, 13.0, z0}, sim::P3{x, 13.0, z0 + 1.0}, sim::P3{x, -13.0, z0 + 1.0}};
        q.colourId = kBlue;
        return q;
    };
    struct Case {
        double z0;
        bool gate;
        ToggleVisitVerdict expect;
        int contacts;
    };
    for (const Case c : {Case{12.4, false, ToggleVisitVerdict::NoChange, 1}, Case{12.4, true, ToggleVisitVerdict::NoChange, 1},
                         Case{11.0, false, ToggleVisitVerdict::NotAtSpot, 0}}) {
        CAPTURE(c.z0);
        CAPTURE(c.gate);
        Options o = faces(scene(Form::Arm), kRed, kYellow, kBlue);
        o.drawToggle = false;
        Rig r{o};
        r.extras.push_back(plate(66.2, c.z0));
        if (c.gate) {
            r.cam.tracker.positionToleranceIn = 3.0;
            r.cam.tracker.maxPoseToleranceIn = 6.0;
            r.cam.tracker.maxFixRadiusIn = 8.0;
            r.cam.poseTolerance = Length{1.0};
            r.cam.headingToleranceDeg = 1.0;
        }
        r.redraw();
        const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
        CHECK(rep.verdict == c.expect);
        CHECK(rep.before == ToggleColour::Blue);
        CHECK(r.toggle.flipperContactsWhileOurs() == c.contacts);
        CHECK(r.toggle.scoredColour() == (c.contacts > 0 ? kYellow : kRed));
        oneLine(r, rep);
    }
}

// ═══ T2 — never a push on UNKNOWN ═══════════════════════════════════════════════════════════════════

TEST_CASE("AF1S-A1 T2: every reason the tracker gives for UNKNOWN -> UNKNOWN BEFORE with THAT reason; nothing "
          "commanded, nothing driven, the Toggle untouched") {
    struct Case {
        const char* name;
        ToggleWhy why;
    };
    const Case cases[] = {{"no read (the camera unplugged from the start)", ToggleWhy::NoRead},
                          {"camera silent (unplugged mid-look)", ToggleWhy::CameraSilent},
                          {"read not usable (a torn read last)", ToggleWhy::ReadNotUsable},
                          {"no band (no Toggle in front)", ToggleWhy::NoBand},
                          {"cut by the image (4 in from the face)", ToggleWhy::CutByImage},
                          {"maybe the bottom face (10 in from the face)", ToggleWhy::MaybeBottomFace},
                          {"two colours", ToggleWhy::TwoColours},
                          {"not held yet (lookWithin shorter than the hold)", ToggleWhy::NotHeldYet},
                          {"another colour directly above (turned 8 degrees)", ToggleWhy::ColourAbove},
                          {"camera full (24 detections)", ToggleWhy::CameraFull},
                          {"another colour beside it (a blue band cut by the image)", ToggleWhy::ConflictingPartialBand}};
    for (const Case& c : cases) {
        CAPTURE(std::string{c.name});
        Options o = scene(Form::Arm);
        sc::ToggleVisitRequest q = armRequest();
        if (c.why == ToggleWhy::CutByImage) {
            o.lookFrom = 4.5;
            q.lookFrom = Length{16.0};
        } else if (c.why == ToggleWhy::MaybeBottomFace) {
            o.lookFrom = 10.0;
        } else if (c.why == ToggleWhy::ColourAbove) {
            o.headingDeg = 8.0;
        } else if (c.why == ToggleWhy::NoBand) {
            o.drawToggle = false;
        } else if (c.why == ToggleWhy::NotHeldYet) {
            q.lookWithin = Time{0.1};  // the hold is 0.15 s
        }
        Rig r{o};
        if (c.why == ToggleWhy::TwoColours) {
            r.extras.push_back(camrig::face(kBlue, -3.0));
        } else if (c.why == ToggleWhy::ConflictingPartialBand) {
            sim::ColourQuad strip;  // a wide blue band high behind the face, cut by the image's top edge
            strip.c = {sim::P3{kWallX + 1.0, -4.0, 19.0}, sim::P3{kWallX + 1.0, 4.0, 19.0},
                       sim::P3{kWallX + 1.0, 4.0, 30.0}, sim::P3{kWallX + 1.0, -4.0, 30.0}};
            strip.colourId = kBlue;
            r.extras.push_back(strip);
        } else if (c.why == ToggleWhy::CameraFull) {
            for (int i = 0; i < 24; ++i) {  // 24 small unrelated patches on the wall below it: "possibly full"
                sim::ColourQuad q2;
                const double y = -6.0 + (i % 6) * 2.0;
                const double z = 3.0 + (i / 6) * 2.0;
                q2.c = {sim::P3{kWallX, y, z}, sim::P3{kWallX, y + 1.0, z}, sim::P3{kWallX, y + 1.0, z + 1.0},
                        sim::P3{kWallX, y, z + 1.0}};
                q2.colourId = 6;
                r.extras.push_back(q2);
            }
        }
        r.redraw();
        if (c.why == ToggleWhy::NoRead) {
            r.camPlant.setState(sim::CameraState::Unplugged);
        }
        const double t0 = r.now();
        bool once = false;
        r.world.onPace = [&] {
            if (c.why == ToggleWhy::CameraSilent && !once && r.now() >= t0 + 0.05) {
                once = true;
                r.camPlant.setState(sim::CameraState::Unplugged);
            }
            if (c.why == ToggleWhy::ReadNotUsable && !once && r.now() >= t0 + 0.55) {
                once = true;  // the last read before the look ends is torn
                r.camPlant.setState(sim::CameraState::Torn);
            }
        };
        if (c.why == ToggleWhy::ReadNotUsable) {
            q.lookWithin = Time{0.58};
            r.camPlant.setState(sim::CameraState::Normal);
            // The colour must not be sure before the torn read: no hold until then — a turned robot.
            r.extras.push_back(camrig::face(kBlue, -3.0));  // two colours until the tear: never sure
            r.redraw();
        }
        const std::uint32_t seq0 = armSlot(r).commandSeq();
        const int m0 = r.chassis.scheduler().motionsStarted();
        const sc::ToggleVisitReport rep = r.visit(q, Rig::entry());
        CHECK(rep.verdict == ToggleVisitVerdict::UnknownBefore);
        CHECK(rep.whyBefore == c.why);
        CHECK(rep.pushes == 0);
        CHECK(armSlot(r).commandSeq() == seq0);
        CHECK(r.chassis.scheduler().motionsStarted() == m0);
        CHECK(r.toggle.strokes() == 0);
        oneLine(r, rep);
        CHECK(r.lines("TGL")[0].find(vision::toggleWhyWords(c.why)) != std::string::npos);
    }
}

// ═══ T3 — a stale or frozen frame ═══════════════════════════════════════════════════════════════════

TEST_CASE("AF1S-A1 T3 (R3): the camera is READ the whole way through the drive in and the drive out (never a "
          "blocking drive that leaves it unread): a read at least every 33-ms period while the step's motion is "
          "live") {
    Rig r{scene(Form::Arm)};
    std::uint32_t readsDriving = 0;
    long pacesDriving = 0;
    std::uint32_t last = 0;
    r.world.onPace = [&] {
        const std::uint32_t now = r.feed.stats().reads;
        if (r.chassis.scheduler().hasActiveMotion()) {
            readsDriving += now - last;
            ++pacesDriving;
        }
        last = now;
    };
    const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
    CHECK(rep.verdict == ToggleVisitVerdict::Flipped);
    REQUIRE(pacesDriving > 400);  // four drives of ~1.3 s on 10-ms ticks
    // one read per 33-ms period at most, at least one per 4 ticks: the feed's period on a 10-ms tick
    CHECK(static_cast<double>(readsDriving) >= 0.24 * static_cast<double>(pacesDriving));
    oneLine(r, rep);
}

TEST_CASE("AF1S-A1 T3 (R3, P5(a)): a camera FROZEN at the look spot as the drive in begins -> seen frozen on the "
          "drive in (the lens moves, the box does not): FROZEN BEFORE PUSH, nothing pushed, the robot back out — "
          "never a push resting on a picture the drive showed stale (the review round's fix)") {
    Rig r{scene(Form::Arm)};
    bool frozen = false;
    double frontAtStop = -1.0;
    r.world.onPace = [&] {
        if (!frozen && r.chassis.scheduler().hasActiveMotion()) {
            frozen = true;
            r.camPlant.setState(sim::CameraState::Frozen);
        }
    };
    const std::uint32_t seq0 = armSlot(r).commandSeq();
    const int m0 = r.chassis.scheduler().motionsStarted();
    sc::ToggleVisitReport rep{};
    r.runGuarded([&] {
        rep = sc::checkAndFlipToggle(r.cam, r.chassis, r.guard, &r.svc, Rig::entry(), armRequest(), r.sink);
        frontAtStop = r.frontTo(r.toggleConfig());
    });
    CHECK(rep.verdict == ToggleVisitVerdict::FrozenBeforePush);
    CHECK(rep.pushes == 0);
    CHECK(rep.driveIn == shulib::control::ExitReason::Cancelled);
    CHECK(rep.driveOut == shulib::control::ExitReason::Settled);
    CHECK(armSlot(r).commandSeq() == seq0);
    CHECK(r.chassis.scheduler().motionsStarted() - m0 == 2);
    CHECK(r.toggle.strokes() == 0);
    CHECK(frontAtStop == doctest::Approx(16.0).epsilon(0.05));  // back at the look spot
    oneLine(r, rep);
    CHECK(r.lines("TGL")[0].find("camera frozen? seen on the drive in -- push 1 not made") != std::string::npos);
}

TEST_CASE("AF1S-A1 T3 (R4, R8, P8): a freeze half-way in -> seen on the rest of the drive in, nothing pushed (FROZEN "
          "BEFORE PUSH); at the push spot before the roll, at the push spot after it, or half-way out -> never a "
          "colour accepted after the push (UNKNOWN AFTER every time)") {
    for (int where = 1; where <= 4; ++where) {
        CAPTURE(where);
        Rig r{scene(Form::Arm)};
        bool frozen = false;
        bool wasOut = false;
        r.world.onPace = [&] {
            if (frozen) {
                return;
            }
            const double front = r.frontTo(r.toggleConfig());
            const double deg = r.armPlant.trueDeg();
            wasOut = wasOut || deg > 85.0;
            const bool go = where == 1   ? front <= 10.0
                            : where == 2 ? deg > 20.0
                            : where == 3 ? wasOut && deg < 40.0
                                         : wasOut && deg < 5.0 && front >= 10.0;
            if (go) {
                frozen = true;
                r.camPlant.setState(sim::CameraState::Frozen);
            }
        };
        const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
        CHECK(frozen);
        if (where == 1) {
            CHECK(rep.verdict == ToggleVisitVerdict::FrozenBeforePush);
            CHECK(rep.pushes == 0);
            CHECK(r.toggle.strokes() == 0);
        } else {
            CHECK(rep.verdict == ToggleVisitVerdict::UnknownAfter);
            CHECK(rep.after == ToggleColour::Unknown);
            CHECK(rep.pushes == 1);
        }
        oneLine(r, rep);
    }
}

TEST_CASE("AF1S-A1 T3 (R4, P5(b)): a STICKY Toggle and a LIVE camera that names the colour AT the push spot (12 in "
          "up, tilted 10 degrees, pushing from 10 in) -> NO CHANGE, not UNKNOWN AFTER") {
    Options o = scene(Form::Arm);
    o.lensHeight = 12.0;
    o.lensPitch = 10.0;
    o.toggle.mode = sim::ToggleMode::Sticky;
    o.toggle.reachIn = 11.0;
    Rig r{o};
    sc::ToggleVisitRequest q = armRequest();
    q.pushFrom = Length{10.0};
    q.offBy = Length{8.0};  // this mount's band at the spot is ~11–23.5 in wide (it hardly ranges)
    const sc::ToggleVisitReport rep = r.visit(q, Rig::entry());
    CHECK(rep.verdict == ToggleVisitVerdict::NoChange);
    CHECK(rep.after == ToggleColour::Yellow);
    CHECK(rep.push[0].outcome == MechanismOutcome::Stalled);
    CHECK(rep.outcome() == MechanismOutcome::Stalled);
    oneLine(r, rep);
}

TEST_CASE("AF1S-A1 T3 (R8, MA6, MA25): a camera whose PICTURE is 3 s old at its read (exaggerated so it shows) — "
          "after the drive out it still shows the Toggle from before the roll; only a picture from after the "
          "retract counts, so the flip is seen (FLIPPED), never NO CHANGE from the old picture") {
    Options o = scene(Form::Arm);
    o.lensHeight = 12.0;
    o.lensPitch = 10.0;
    o.toggle.reachIn = 11.0;
    Rig r{o};
    r.camera.lagSeconds = 3.0;
    sc::ToggleVisitRequest q = armRequest();
    q.pushFrom = Length{10.0};
    q.offBy = Length{8.0};
    q.maxImageAge = Time{3.0};
    q.lookWithin = Time{4.0};
    const sc::ToggleVisitReport rep = r.visit(q, Rig::entry());
    CHECK(rep.verdict == ToggleVisitVerdict::Flipped);
    CHECK(rep.pushes == 2);
    CHECK(r.toggle.scoredColour() == kRed);
    oneLine(r, rep);
}

// ═══ T4 — two Toggles, two colours ══════════════════════════════════════════════════════════════════

TEST_CASE("AF1S-A1 T4: two Toggle-coloured bands of different colours in the window -> UNKNOWN BEFORE 'two colours' "
          "(covered in T2); a second Toggle on ANOTHER wall, in the scene the whole time -> ignored, the named one "
          "judged and flipped") {
    Options o = scene(Form::Arm);
    sim::TogglePlantConfig other = redSideToggle();
    other.fieldId = kBlue;
    other.bottomId = kRed;
    other.outerId = kYellow;
    o.others.push_back(onWall(other, ov::toggles::kBlueWall));
    Rig r{o};
    const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
    CHECK(rep.verdict == ToggleVisitVerdict::Flipped);
    CHECK(r.toggle.scoredColour() == kRed);
    CHECK(r.others[0]->strokes() == 0);
    oneLine(r, rep);
}

// ═══ T5 — the robot off the spot ════════════════════════════════════════════════════════════════════

TEST_CASE("AF1S-A1 T5 (R2, P7): a robot parked 8 in TOO FAR (true 24 in, look spot 16 +- 6: its band OVERLAPS the "
          "window but is not inside it) -> NOT AT THE SPOT; nothing driven, nothing pushed") {
    Options o = scene(Form::Arm);
    o.lookFrom = 24.0;
    Rig r{o};
    const int m0 = r.chassis.scheduler().motionsStarted();
    const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
    CHECK(rep.verdict == ToggleVisitVerdict::NotAtSpot);
    CHECK(rep.range.lo < 22.0);  // it overlaps the window ...
    CHECK(rep.range.hi > 22.0);  // ... and is not inside it
    CHECK(r.chassis.scheduler().motionsStarted() == m0);
    CHECK(r.toggle.strokes() == 0);
    oneLine(r, rep);
    CHECK(r.lines("TGL")[0].find("not at the spot") != std::string::npos);
}

TEST_CASE("AF1S-A1 T5: the true look distance swept 14..22 in — accepted (and flipped) only where the band lies "
          "wholly inside the window") {
    for (const double d : {14.0, 16.0, 17.0, 18.0, 20.0, 22.0}) {
        CAPTURE(d);
        Options o = scene(Form::Arm);
        o.lookFrom = d;
        Rig r{o};
        const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
        if (d <= 17.0) {
            CHECK(rep.verdict == ToggleVisitVerdict::Flipped);
        } else {
            CHECK(rep.verdict == ToggleVisitVerdict::NotAtSpot);
            CHECK(r.toggle.strokes() == 0);
        }
        oneLine(r, rep);
    }
}

TEST_CASE("AF1S-A1 T5: FIELD start — the estimate 12 in (2 x offBy) back from where the robot truly is -> NOT AT THE "
          "SPOT (the camera agreed; the field check did not); facing 60 degrees off the named wall -> WRONG WALL "
          "before anything is read; facing it -> it flips, and the line does not say 'wall not checked'") {
    {
        Rig r{scene(Form::Arm)};
        const shulib::math::Pose2d p = r.truth();
        r.chassis.setPose(shulib::math::Pose2d{p.x() - Length{12.0}, p.y(), p.heading()});
        const sc::ToggleVisitReport rep =
            r.visit(armRequest(), Rig::entry(sc::AutonKind::Skills, sc::StartPose::field(p)));
        CHECK(rep.verdict == ToggleVisitVerdict::NotAtSpot);
        CHECK(rep.lensFromWallIn == doctest::Approx(28.0).epsilon(0.02));
        CHECK(r.toggle.strokes() == 0);
        oneLine(r, rep);
        CHECK(r.lines("TGL")[0].find("estimate") != std::string::npos);
    }
    {
        Options o = scene(Form::Arm);
        o.headingDeg = 60.0;
        Rig r{o};
        const sc::ToggleVisitReport rep =
            r.visit(armRequest(), Rig::entry(sc::AutonKind::Skills, sc::StartPose::field(r.truth())));
        CHECK(rep.verdict == ToggleVisitVerdict::WrongWall);
        CHECK(r.feed.stats().reads == 0U);
        CHECK(rep.seconds == 0.0);
        oneLine(r, rep);
    }
    {
        Rig r{scene(Form::Arm)};
        const sc::ToggleVisitReport rep =
            r.visit(armRequest(), Rig::entry(sc::AutonKind::Skills, sc::StartPose::field(r.truth())));
        CHECK(rep.verdict == ToggleVisitVerdict::Flipped);
        CHECK(rep.wallChecked);
        oneLine(r, rep);
        CHECK(r.lines("TGL")[0].find("wall not checked") == std::string::npos);
    }
    {
        // The left wall (heading 180) and the blue wall (heading +90), each faced square: accepted.
        for (const ov::ToggleId w : {ov::toggles::kLeftWall, ov::toggles::kBlueWall}) {
            CAPTURE(static_cast<int>(w));
            Options o = faces(scene(Form::Arm, w), kRed, kYellow, kBlue);
            Rig r{o};
            const sc::ToggleVisitReport rep =
                r.visit(armRequest(w), Rig::entry(sc::AutonKind::Skills, sc::StartPose::field(r.truth())));
            CHECK(rep.verdict != ToggleVisitVerdict::WrongWall);
            CHECK(rep.wallChecked);
        }
    }
}

TEST_CASE("AF1S-A1 T5: a RELATIVE start proceeds, and its line says 'wall not checked (relative start)'") {
    Rig r{scene(Form::Arm)};
    const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
    CHECK(rep.verdict == ToggleVisitVerdict::Flipped);
    CHECK_FALSE(rep.wallChecked);
    oneLine(r, rep);
    CHECK(r.lines("TGL")[0].find("wall not checked (relative start)") != std::string::npos);
}

// ═══ T6 — the period ending ═════════════════════════════════════════════════════════════════════════

TEST_CASE("AF1S-A1 T6: remaining() 5 ms UNDER need(2) after the look -> NOT ENOUGH TIME, nothing commanded; 1 ms "
          "OVER, with every stage within a tick of its bound -> both pushes, the flipper back and the robot off the "
          "Toggle before the deadline") {
    sc::ToggleVisitRequest q = armRequest();
    q.driveBound = Time{1.30};  // the host tank's 12-in drive: 1.29 s
    q.flipBound = Time{0.88};   // the host arm's 90 degrees: 0.87 s
    q.backBound = Time{0.88};
    q.lookWithin = Time{0.4};
    double look = 0.0;
    {
        Rig r{scene(Form::Arm)};
        look = r.visit(q, Rig::entry(), 55.0, /*lookOnly=*/true).seconds;
        CHECK(look > 0.0);
    }
    const double need = sc::worstCaseSeconds(q);
    {
        Rig r{scene(Form::Arm)};
        const std::uint32_t seq0 = armSlot(r).commandSeq();
        const int m0 = r.chassis.scheduler().motionsStarted();
        const sc::ToggleVisitReport rep = r.visit(q, Rig::entry(), look + need - 0.005);
        CHECK(rep.verdict == ToggleVisitVerdict::NotEnoughTime);
        CHECK(rep.secondsLeftAtCheck < rep.secondsNeeded);
        CHECK(rep.secondsNeeded == doctest::Approx(need));
        CHECK(rep.outcome() == MechanismOutcome::TimedOut);
        CHECK(armSlot(r).commandSeq() == seq0);
        CHECK(r.chassis.scheduler().motionsStarted() == m0);
        oneLine(r, rep);
    }
    {
        Rig r{scene(Form::Arm)};
        sc::ToggleVisitReport rep{};
        double left = -1.0;
        bool touched = true;
        double deg = 99.0;
        double front = 0.0;
        r.runGuarded(
            [&] {
                rep = sc::checkAndFlipToggle(r.cam, r.chassis, r.guard, &r.svc, Rig::entry(), q, r.sink);
                left = r.guard.remaining().value();
                touched = r.toggle.touched();
                deg = r.armPlant.trueDeg();
                front = r.frontTo(r.toggleConfig());
            },
            look + need + 0.001);
        CHECK(rep.verdict == ToggleVisitVerdict::Flipped);
        CHECK(rep.pushes == 2);
        CHECK(left > 0.0);
        CHECK_FALSE(touched);
        CHECK(deg < 5.0);
        CHECK(front > 15.0);
        CHECK(r.toggle.scoredColour() == kRed);
        oneLine(r, rep);
    }
}

TEST_CASE("AF1S-A1 T6 (R7): the brain's loop STALLS while the flipper is out, leaving 2.0 s (under backBound + settle + "
          "driveBound + margin = 3.4) -> back AT ONCE, no settle, the drive out, RETRACTED FOR TIME — and the robot "
          "OFF the Toggle (the host Toggle counts it touching until its front is 15 in out) when the step returns, "
          "before the deadline") {
    Options o = scene(Form::Arm);
    o.toggle.touchIn = 15.0;
    Rig r{o};
    sc::ToggleVisitRequest q = armRequest();
    q.settle = Time{0.6};
    q.margin = Time{0.1};
    bool jumped = false;
    r.world.onPace = [&] {
        if (!jumped && r.guard.running() && r.armPlant.trueDeg() >= 35.0) {
            jumped = true;
            r.rig.h.clock().advance(Time{r.guard.remaining().value() - 2.0});
        }
    };
    sc::ToggleVisitReport rep{};
    bool touched = true;
    double deg = 99.0;
    double left = -1.0;
    r.runGuarded([&] {
        rep = sc::checkAndFlipToggle(r.cam, r.chassis, r.guard, &r.svc, Rig::entry(), q, r.sink);
        touched = r.toggle.touched();
        deg = r.armPlant.trueDeg();
        left = r.guard.remaining().value();
    });
    CHECK(jumped);
    CHECK(rep.verdict == ToggleVisitVerdict::RetractedForTime);
    CHECK(rep.outcome() == MechanismOutcome::TimedOut);
    CHECK(rep.push[0].outcome == MechanismOutcome::Cancelled);  // the push was replaced
    CHECK(rep.back[0].outcome == MechanismOutcome::Succeeded);
    CHECK_FALSE(touched);
    CHECK(deg < 30.0);
    CHECK(left > 0.0);
    oneLine(r, rep);
}

TEST_CASE("AF1S-A1 T6: the guard's deadline lands MID-LOOK -> CUT, outcome CANCELLED (a route stops whatever its "
          "choice), the step returning within one tick of the deadline") {
    Rig r{scene(Form::Arm)};
    sc::ToggleVisitReport rep{};
    double tStart = 0.0;
    double tEnd = 0.0;
    r.runGuarded(
        [&] {
            tStart = r.now();
            rep = sc::checkAndFlipToggle(r.cam, r.chassis, r.guard, &r.svc, Rig::entry(), armRequest(), r.sink);
            tEnd = r.now();
        },
        0.08);
    CHECK(rep.verdict == ToggleVisitVerdict::Cut);
    CHECK(rep.outcome() == MechanismOutcome::Cancelled);
    CHECK(tEnd - tStart <= 0.08 + 0.0101);
    CHECK(r.toggle.strokes() == 0);
    oneLine(r, rep);
}

// ═══ T7 — a flipper that stalls ═════════════════════════════════════════════════════════════════════

TEST_CASE("AF1S-A1 T7: the push STALLS on a sticky Toggle -> `back` commanded on the very next tick, the look after, "
          "NO CHANGE, outcome STALLED") {
    Options o = scene(Form::Arm);
    o.toggle.mode = sim::ToggleMode::Sticky;
    Rig r{o};
    mech::PositionSlot& s = armSlot(r);
    double stalledAt = -1.0;
    double backAt = -1.0;
    std::uint32_t steps = 0;
    r.world.onPace = [&] {
        if (stalledAt < 0.0 && s.lastStep().outcome == MechanismOutcome::Stalled) {
            stalledAt = r.now();
            steps = s.stepsBegun();
        }
        if (stalledAt >= 0.0 && backAt < 0.0 && s.stepsBegun() > steps) {
            backAt = r.now();
        }
    };
    const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
    CHECK(rep.verdict == ToggleVisitVerdict::NoChange);
    CHECK(rep.push[0].outcome == MechanismOutcome::Stalled);
    CHECK(rep.back[0].outcome == MechanismOutcome::Succeeded);
    CHECK(rep.outcome() == MechanismOutcome::Stalled);
    REQUIRE(stalledAt >= 0.0);
    REQUIRE(backAt >= 0.0);
    CHECK(backAt - stalledAt <= 0.0101);
    CHECK(r.armPlant.trueDeg() < 5.0);
    oneLine(r, rep);
}

TEST_CASE("AF1S-A1 T7: the RETRACT stalls (something wedged under the flipper) -> FLIPPER STUCK, outcome STALLED, and "
          "the robot does NOT drive out (the drive in was the only motion; it stays at the push spot)") {
    Rig r{scene(Form::Arm)};
    bool wedged = false;
    r.world.onPace = [&] {
        if (!wedged && r.armPlant.trueDeg() > 85.0) {
            wedged = true;
            r.armPlant.setWedgeDeg(50.0);
        }
    };
    const int m0 = r.chassis.scheduler().motionsStarted();
    const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
    CHECK(rep.verdict == ToggleVisitVerdict::FlipperStuck);
    CHECK(rep.outcome() == MechanismOutcome::Stalled);
    CHECK(rep.back[0].outcome == MechanismOutcome::Stalled);
    CHECK(r.chassis.scheduler().motionsStarted() - m0 == 1);
    CHECK(r.frontTo(r.toggleConfig()) < 5.0);
    oneLine(r, rep);
    CHECK(r.lines("TGL")[0].find("'flip arm'") != std::string::npos);
}

TEST_CASE("AF1S-A1 T7: TIMED_OUT likewise — a push cut short of the roll by its bound -> back, NO CHANGE (UNCONFIRMED, "
          "not STALLED); a retract cut by its bound -> FLIPPER STUCK, no drive out") {
    {
        Rig r{scene(Form::Arm)};
        sc::ToggleVisitRequest q = armRequest();
        q.flipBound = Time{0.4};  // the arm reaches ~50 degrees: short of the roll
        const sc::ToggleVisitReport rep = r.visit(q, Rig::entry());
        CHECK(rep.verdict == ToggleVisitVerdict::NoChange);
        CHECK(rep.push[0].outcome == MechanismOutcome::TimedOut);
        CHECK(rep.back[0].outcome == MechanismOutcome::Succeeded);
        CHECK(rep.outcome() == MechanismOutcome::Unconfirmed);
        oneLine(r, rep);
    }
    {
        Rig r{scene(Form::Arm)};
        sc::ToggleVisitRequest q = armRequest();
        q.backBound = Time{0.3};
        const int m0 = r.chassis.scheduler().motionsStarted();
        const sc::ToggleVisitReport rep = r.visit(q, Rig::entry());
        CHECK(rep.verdict == ToggleVisitVerdict::FlipperStuck);
        CHECK(rep.back[0].outcome == MechanismOutcome::TimedOut);
        CHECK(r.chassis.scheduler().motionsStarted() - m0 == 1);
        oneLine(r, rep);
    }
}

// ═══ T8 — at most two pushes, the second only on the third colour (P3's 84 cases, on the plants) ════════════════

namespace {
enum class Trouble { None, Sticky, OverRoll, JamOnPush2, UnknownAfter1, FrozenAfter, NoTimeForPush2 };
struct Expect {
    ToggleVisitVerdict verdict;
    int pushes;
};
// THE RULE, written apart from the step (the oracle): faces [start, next, third], "ours" `ours`.
Expect oracle(std::uint8_t start, std::uint8_t next, std::uint8_t third, std::uint8_t ours, Trouble t) {
    if (start == ours) {
        return {ToggleVisitVerdict::AlreadyOurs, 0};
    }
    switch (t) {
        case Trouble::Sticky: return {ToggleVisitVerdict::NoChange, 1};
        case Trouble::UnknownAfter1:
        case Trouble::FrozenAfter: return {ToggleVisitVerdict::UnknownAfter, 1};
        case Trouble::OverRoll:
            if (third == ours) {
                return {ToggleVisitVerdict::Flipped, 1};
            }
            return {ToggleVisitVerdict::Flipped, 2};  // third, then two more faces: next
        case Trouble::JamOnPush2:
            if (next == ours) {
                return {ToggleVisitVerdict::Flipped, 1};
            }
            return {ToggleVisitVerdict::StillNotOurs, 2};
        case Trouble::NoTimeForPush2:
            if (next == ours) {
                return {ToggleVisitVerdict::Flipped, 1};
            }
            return {ToggleVisitVerdict::NoTimeForSecondPush, 1};
        case Trouble::None: break;
    }
    if (next == ours) {
        return {ToggleVisitVerdict::Flipped, 1};
    }
    return {ToggleVisitVerdict::Flipped, 2};
}
}  // namespace

TEST_CASE("AF1S-A1 T8: P3's 84 skills cases ON THE PLANTS — both roll orders x both Quadrant colours x starts Y/R/B "
          "x {none, sticky, over-roll, a jam on push 2, UNKNOWN after push 1, a frozen camera, no time for push 2}: "
          "the verdict and the pushes as the rule says, never a third push, never a push on ours") {
    const std::uint8_t cycleA[3] = {kYellow, kBlue, kRed};  // Y -> B -> R (the research's red side)
    const std::uint8_t cycleB[3] = {kYellow, kRed, kBlue};  // Y -> R -> B
    int cases = 0;
    for (const int order : {0, 1}) {
        const std::uint8_t* cyc = order == 0 ? cycleA : cycleB;
        for (const ov::ToggleId wall : {ov::toggles::kRightWall, ov::toggles::kLeftWall}) {
            const std::uint8_t ours = wall == ov::toggles::kRightWall ? kRed : kBlue;
            for (int s = 0; s < 3; ++s) {
                const std::uint8_t start = cyc[s];
                const std::uint8_t next = cyc[(s + 1) % 3];
                const std::uint8_t third = cyc[(s + 2) % 3];
                for (const Trouble t : {Trouble::None, Trouble::Sticky, Trouble::OverRoll, Trouble::JamOnPush2,
                                        Trouble::UnknownAfter1, Trouble::FrozenAfter, Trouble::NoTimeForPush2}) {
                    CAPTURE(order);
                    CAPTURE(static_cast<int>(wall));
                    CAPTURE(static_cast<int>(start));
                    CAPTURE(static_cast<int>(t));
                    Options o = faces(scene(Form::Arm, wall), start, next, third);
                    o.toggle.mode = t == Trouble::Sticky     ? sim::ToggleMode::Sticky
                                    : t == Trouble::OverRoll ? sim::ToggleMode::OverRoll
                                                             : sim::ToggleMode::Normal;
                    Rig r{o};
                    bool done = false;
                    bool wasOut = false;
                    r.world.onPace = [&] {
                        if (done) {
                            return;
                        }
                        const double deg = r.armPlant.trueDeg();
                        wasOut = wasOut || deg > 85.0;
                        if (t == Trouble::JamOnPush2 && r.toggle.facesRolled() >= 1 && r.toggle.seated()) {
                            done = true;
                            r.toggle.setMode(sim::ToggleMode::Sticky);
                        } else if (t == Trouble::UnknownAfter1 && wasOut && deg < 5.0) {
                            done = true;
                            r.camPlant.setState(sim::CameraState::Unplugged);
                        } else if (t == Trouble::FrozenAfter && deg > 20.0) {
                            done = true;  // frozen as push 1 begins (frozen on the drive in is T3's)
                            r.camPlant.setState(sim::CameraState::Frozen);
                        } else if (t == Trouble::NoTimeForPush2 && wasOut && deg < 5.0 && !armSlot(r).running()) {
                            done = true;  // a loop stall during the settle: 5.0 s left, under need(1)
                            r.rig.h.clock().advance(Time{r.guard.remaining().value() - 5.0});
                        }
                    };
                    const sc::ToggleVisitReport rep = r.visit(armRequest(wall), Rig::entry());
                    const Expect e = oracle(start, next, third, ours, t);
                    CHECK(rep.verdict == e.verdict);
                    CHECK(rep.pushes == e.pushes);
                    CHECK(r.toggle.strokes() <= 2);
                    CHECK(r.toggle.flipperContactsWhileOurs() == 0);
                    if (rep.ok()) {
                        CHECK(r.toggle.scoredColour() == ours);
                    }
                    oneLine(r, rep);
                    ++cases;
                }
            }
        }
    }
    CHECK(cases == 84);
}

// ═══ T9 — not allowed ═══════════════════════════════════════════════════════════════════════════════

TEST_CASE("AF1S-A1 T9: a MATCH auton -> NOT SKILLS; no camera -> NO CAMERA; each with the clock unmoved, nothing "
          "read, nothing commanded") {
    {
        Rig r{scene(Form::Arm)};
        const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry(sc::AutonKind::Match));
        CHECK(rep.verdict == ToggleVisitVerdict::NotSkills);
        CHECK(rep.seconds == 0.0);
        CHECK(r.feed.stats().reads == 0U);
        oneLine(r, rep);
    }
    {
        Rig r{scene(Form::Arm)};
        r.cam.feed = nullptr;
        const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
        CHECK(rep.verdict == ToggleVisitVerdict::NoCamera);
        CHECK(rep.seconds == 0.0);
        oneLine(r, rep);
    }
    {
        Rig r{scene(Form::Arm)};
        r.cam.tracker.holdSeconds = 0.0;  // a config the tracker refuses
        const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
        CHECK(rep.verdict == ToggleVisitVerdict::NoCamera);
        CHECK(r.feed.stats().reads == 0U);
        oneLine(r, rep);
    }
}

namespace tv_unhomed {
using namespace shulib::mechanism::words;
inline constexpr Arm flipArm{
    .name = "flip arm",
    .motors = {Forward},
    .cartridge = Green,
    .gearing = gears(12, 60),
    .safe = Hold,
    .stall = {.amps = 1.0, .belowMotorRpm = 5, .lasting = 0.15_s},
    .range = {0, 120},
    .gravity = noGravity,
    .home = homeByStall(0, -3_volt, {.amps = 0.4, .belowMotorRpm = 2, .lasting = 0.2_s}),
    .presets = {{"stow", 0}, {"flip", 90}, {"half", 45}},
};
inline constexpr MechanismTable kTable =
    mechanisms(wire(flipArm, motors(1)), wire(tv_fixture::flipLift, motors(2)), wire(tv_fixture::flipPiston, lines('A')));
}  // namespace tv_unhomed

TEST_CASE("AF1S-A1 T9: the flipper ABSENT, NOT HOMED, or NOT AT `back` -> FLIPPER NOT READY, and the step NEVER moves "
          "it there itself; no mechanism service for a motor flipper likewise") {
    {
        Options o = scene(Form::Arm);
        o.table = &tv_unhomed::kTable;
        Rig r{o};
        const std::uint32_t seq0 = armSlot(r).commandSeq();
        const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
        CHECK(rep.verdict == ToggleVisitVerdict::FlipperNotReady);
        CHECK(std::string{rep.detail}.find("not homed") != std::string::npos);
        CHECK(armSlot(r).commandSeq() == seq0);
        CHECK(r.feed.stats().reads == 0U);
        oneLine(r, rep);
    }
    {
        Rig r{scene(Form::Arm)};
        // Put the arm at "half" (45 degrees) first, as a route might have left it.
        armSlot(r).moveTo(45.0, Time{2.0}, false);
        for (int i = 0; i < 300 && armSlot(r).running(); ++i) {
            r.svc.pace();
        }
        REQUIRE(armSlot(r).position() == doctest::Approx(45.0).epsilon(0.05));
        const std::uint32_t seq0 = armSlot(r).commandSeq();
        const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
        CHECK(rep.verdict == ToggleVisitVerdict::FlipperNotReady);
        CHECK(std::string{rep.detail}.find("not at its back position") != std::string::npos);
        CHECK(armSlot(r).commandSeq() == seq0);  // never moved to `back`
        CHECK(r.armPlant.trueDeg() > 40.0);
        CHECK(rep.seconds == 0.0);
        oneLine(r, rep);
    }
    {
        Options o = scene(Form::Arm);
        Rig r{o};
        mech::MechanismService none{r.world};  // bound to nothing: the flipper is not on "this robot"
        const sc::ToggleVisitReport rep = r.visitWith(&none, armRequest(), Rig::entry());
        CHECK(rep.verdict == ToggleVisitVerdict::FlipperNotReady);
        oneLine(r, rep);
    }
}

namespace tv_other {
using namespace shulib::mechanism::words;
// ANOTHER robot's arm of the same name, whose "stow" is somewhere else (autons are per robot).
inline constexpr Arm flipArm{
    .name = "flip arm",
    .motors = {Forward},
    .cartridge = Green,
    .gearing = gears(12, 60),
    .safe = Hold,
    .stall = {.amps = 1.0, .belowMotorRpm = 5, .lasting = 0.15_s},
    .range = {0, 120},
    .gravity = noGravity,
    .home = restsAtStop(0),
    .presets = {{"stow", 5}, {"flip", 90}},
};
}  // namespace tv_other

TEST_CASE("AF1S-A1 T9: a `back` this robot's flipper does not have (another robot's same-named arm's preset) -> "
          "FLIPPER NOT READY, nothing commanded — the run-time pair check, past the request's own name check") {
    Rig r{scene(Form::Arm)};
    sc::ToggleVisitRequest q = armRequest();
    q.back = tv_other::flipArm["stow"];  // same name, same units: the request accepts it; this robot has no stow at 5
    REQUIRE(sc::toggleVisitRequestProblem(q) == nullptr);
    const std::uint32_t seq0 = armSlot(r).commandSeq();
    const sc::ToggleVisitReport rep = r.visit(q, Rig::entry());
    CHECK(rep.verdict == ToggleVisitVerdict::FlipperNotReady);
    CHECK(std::string{rep.detail}.find("not on this robot") != std::string::npos);
    CHECK(armSlot(r).commandSeq() == seq0);
    oneLine(r, rep);
}

TEST_CASE("AF1S-A1 T9: the flipper's motor ABSENT at boot -> FLIPPER NOT READY with the boot's absence sentence") {
    Options o = scene(Form::Arm);
    Rig r{o};
    // A second service over the same table with the arm's actuator missing.
    mech::Devices devs[3]{{.motors = nullptr, .absence = "port 1: no motor"}, {.motors = &r.mLift}, {.pneumatic = &r.mPiston}};
    mech::MechanismService svc{r.world};
    svc.bind(tv_fixture::kFlippers, devs, r.rig.h.clock(), r.rig.latch, r.sink);
    const sc::ToggleVisitReport rep = r.visitWith(&svc, armRequest(), Rig::entry());
    CHECK(rep.verdict == ToggleVisitVerdict::FlipperNotReady);
    CHECK(std::string{rep.detail}.find("port 1") != std::string::npos);
}

TEST_CASE("AF1S-A1 programming errors are LOUD: a request with a problem, an excursion of 2 x frozenMoveIn or less, "
          "a call with no run live") {
    Rig r{scene(Form::Arm)};
    sc::ToggleVisitRequest bad = armRequest();
    bad.maxPushes = 3;
    CHECK_THROWS(r.visit(bad, Rig::entry()));
    sc::ToggleVisitRequest near = armRequest();
    near.pushFrom = Length{12.0};  // 16 - 12 = 4 = 2 x frozenMoveIn (2): refused
    Rig r2{scene(Form::Arm)};
    CHECK_THROWS(r2.visit(near, Rig::entry()));
    near.pushFrom = Length{11.99};
    Rig r3{scene(Form::Arm)};
    CHECK_NOTHROW(r3.visit(near, Rig::entry()));
    Rig r4{scene(Form::Arm)};
    const sc::AutonEntry e = Rig::entry();
    CHECK_THROWS((void)sc::checkAndFlipToggle(r4.cam, r4.chassis, r4.guard, &r4.svc, e, armRequest(), r4.sink));
}

// ═══ T10 — the three forms ══════════════════════════════════════════════════════════════════════════

TEST_CASE("AF1S-A1 T10: an ARM, a LIFT and a PNEUMATIC flipper each flip the host Toggle, logging their [MECH] lines; "
          "a PASSIVE bar flips it with the drive alone and commands no mechanism (no service at all)") {
    struct Case {
        Form form;
        const char* name;
    };
    for (const Case c : {Case{Form::Arm, "flip arm"}, Case{Form::Lift, "flip lift"}, Case{Form::Pneumatic, "flip piston"},
                         Case{Form::Passive, nullptr}}) {
        CAPTURE(static_cast<int>(c.form));
        Rig r{scene(c.form)};
        const sc::ToggleVisitRequest q = c.form == Form::Arm    ? armRequest()
                                         : c.form == Form::Lift ? liftRequest()
                                         : c.form == Form::Pneumatic ? pistonRequest()
                                                                     : passiveRequest();
        const sc::ToggleVisitReport rep = r.visitWith(c.form == Form::Passive ? nullptr : &r.svc, q, Rig::entry());
        CHECK(rep.verdict == ToggleVisitVerdict::Flipped);
        CHECK(rep.pushes == 2);
        CHECK(r.toggle.scoredColour() == kRed);
        int mech = 0;
        for (const std::string& l : r.lines("MECH")) {
            mech += l.find("SUCCEEDED") != std::string::npos ? 1 : 0;
            if (c.name != nullptr && l.find("SUCCEEDED") != std::string::npos) {
                CHECK(l.find(c.name) != std::string::npos);
            }
        }
        CHECK(mech == (c.name != nullptr ? 4 : 0));  // two pushes, two retracts — or none at all
        oneLine(r, rep);
    }
}

// ═══ T11 — the report, the line, and the route's then() ═════════════════════════════════════════════

TEST_CASE("AF1S-A1 T11: through a Routine's then() — FLIPPED continues the chain; NO CHANGE stops it as a "
          "MECHANISM FAILED step naming the outcome; a CUT stops it as CANCELLED (the default stop: STEP1-A's "
          "onFail forms are not in this base)") {
    struct Case {
        sim::ToggleMode mode;
        double deadline;
        bool ok;
    };
    for (const Case c : {Case{sim::ToggleMode::Normal, 55.0, true}, Case{sim::ToggleMode::Sticky, 55.0, false},
                         Case{sim::ToggleMode::Normal, 0.08, false}}) {
        CAPTURE(static_cast<int>(c.mode));
        Options o = scene(Form::Arm);
        o.toggle.mode = c.mode;
        Rig r{o};
        shulib::chassis::RoutineResult res{};
        const sc::AutonEntry e = Rig::entry();
        const sc::ToggleVisitRequest q = armRequest();
        r.runGuarded(
            [&] {
                shulib::chassis::Routine route{r.chassis, "skills"};
                route.then([&] { return sc::checkAndFlipToggle(r.cam, r.chassis, r.guard, &r.svc, e, q, r.sink).outcome(); },
                           "right wall Toggle");
                route.then([] { return true; }, "next");
                res = route.result();
            },
            c.deadline);
        CHECK(res.ok == c.ok);
        if (c.ok) {
            CHECK(res.completed == 2);
        } else {
            CHECK(res.stoppedAt == 1);
            CHECK(std::string{res.stoppedName} == "right wall Toggle");
            CHECK(res.cause == shulib::chassis::RoutineStopCause::MechanismFailed);
            CHECK(res.skipped == 1);
            // The route's stop line names the outcome: STALLED for the sticky Toggle, CANCELLED for the cut.
            const char* word = c.deadline < 1.0 ? "CANCELLED" : "STALLED";
            bool named = false;
            for (const std::string& l : r.lines("RTN")) {
                named = named || (l.find("right wall Toggle") != std::string::npos && l.find(word) != std::string::npos);
            }
            CHECK(named);
        }
    }
}

// ═══ T14 — a Toggle stuck part-way (R1) ═════════════════════════════════════════════════════════════

TEST_CASE("AF1S-A1 T14 (R1, D11) PINNED: after a FULL stroke the Toggle JAMS part-way toward ours — at 50 and 100 "
          "degrees the camera reads ours and the step says FLIPPED (its line ending 'seating not checked') while the "
          "game counts it YELLOW; at 70 it reads nothing (UNKNOWN AFTER). The honest limit, documented") {
    struct Case {
        double jam;
        ToggleVisitVerdict verdict;
    };
    for (const Case c : {Case{50.0, ToggleVisitVerdict::Flipped}, Case{70.0, ToggleVisitVerdict::UnknownAfter},
                         Case{100.0, ToggleVisitVerdict::Flipped}}) {
        CAPTURE(c.jam);
        Options o = faces(scene(Form::Arm), kYellow, kRed, kBlue);  // rolling TOWARD red (ours)
        o.toggle.mode = sim::ToggleMode::Jammed;
        o.toggle.jamDeg = c.jam;
        Rig r{o};
        const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
        CHECK(rep.verdict == c.verdict);
        CHECK(r.toggle.jammed());
        CHECK(r.toggle.scoredColour() == kYellow);
        oneLine(r, rep);
    }
}

TEST_CASE("AF1S-A1 T14 (R1): a push that STALLED, or TIMED OUT, past the roll, then reads OURS -> OURS AFTER A SHORT "
          "STROKE, outcome UNCONFIRMED (never FLIPPED)") {
    for (const bool stalled : {true, false}) {
        CAPTURE(stalled);
        Options o = faces(scene(Form::Arm), kYellow, kRed, kBlue);
        Rig r{o};
        sc::ToggleVisitRequest q = armRequest();
        if (stalled) {
            r.armPlant.setObstacleDeg(70.0);  // past the roll (60), short of the flip (90)
        } else {
            q.flipBound = Time{0.5};
        }
        const sc::ToggleVisitReport rep = r.visit(q, Rig::entry());
        CHECK(rep.verdict == ToggleVisitVerdict::OursAfterShortStroke);
        CHECK(rep.push[0].outcome == (stalled ? MechanismOutcome::Stalled : MechanismOutcome::TimedOut));
        CHECK(rep.outcome() == MechanismOutcome::Unconfirmed);
        CHECK(rep.after == ToggleColour::Red);
        CHECK(r.toggle.scoredColour() == kRed);  // here it did seat — the step cannot know that
        oneLine(r, rep);
    }
}

TEST_CASE("AF1S-A1 T14 (R1): a Toggle JAMMED 100 degrees toward ours BEFORE the visit (our other robot) reads ours -> "
          "ALREADY OURS, 'seating not checked', nothing pushed — while the game counts it YELLOW") {
    Options o = faces(scene(Form::Arm), kYellow, kRed, kBlue);
    o.toggle.mode = sim::ToggleMode::Jammed;
    o.toggle.jamDeg = 100.0;
    Rig r{o};
    r.toggle.rollNow();
    for (int i = 0; i < 100; ++i) {
        r.toggle.step(0.01, 0.0, 30.0);
    }
    r.redraw();
    REQUIRE(r.toggle.jammed());
    const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry());
    CHECK(rep.verdict == ToggleVisitVerdict::AlreadyOurs);
    CHECK(rep.pushes == 0);
    CHECK(r.toggle.scoredColour() == kYellow);
    oneLine(r, rep);
}

// ═══ lookAtToggle — rules 1–3, never moves anything ═════════════════════════════════════════════════

TEST_CASE("AF1S-A1 lookAtToggle: SEEN NOT OURS / ALREADY OURS / the refusal — never a motion, never a command") {
    for (const std::uint8_t start : {kYellow, kRed}) {
        CAPTURE(static_cast<int>(start));
        Rig r{faces(scene(Form::Arm), start, kBlue, start == kRed ? kYellow : kRed)};
        const std::uint32_t seq0 = armSlot(r).commandSeq();
        const int m0 = r.chassis.scheduler().motionsStarted();
        const sc::ToggleVisitReport rep = r.visit(armRequest(), Rig::entry(), 55.0, /*lookOnly=*/true);
        CHECK(rep.verdict == (start == kRed ? ToggleVisitVerdict::AlreadyOurs : ToggleVisitVerdict::SeenNotOurs));
        CHECK(rep.before == (start == kRed ? ToggleColour::Red : ToggleColour::Yellow));
        CHECK(armSlot(r).commandSeq() == seq0);
        CHECK(r.chassis.scheduler().motionsStarted() == m0);
        CHECK(r.toggle.strokes() == 0);
        oneLine(r, rep);
    }
}
