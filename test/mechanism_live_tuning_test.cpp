// mechanism/live_tuning.hpp + MechanismService::rederive() — a live change TAKES EFFECT, exactly where it
// should and nowhere else (chunk TUNE1-A, 2026-11-10; the MECH-S/TUNE1 brief's §2.2 TA1, TA2, TA4, TA6,
// TA7, TA8). ADVERSARIAL by TWINS: two rigs built alike run tick for tick; one is changed, and the test
// pins the difference in the very next command to the volt the changed number accounts for — P6
// inverted (before this chunk a changed spec moved the hold by +0.0000 V: the PIDs had copied their
// gains). A refused change must leave the twins equal, bit for bit. Over the host test plant (its
// constants invented — HA-183); nothing here has run on a robot.

#include "doctest.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "mechanism_live_tuning_rig.hpp"
#include "shulib/mechanism/live_tuning.hpp"
#include "shulib/units/quantity.hpp"

namespace {
namespace mech = shulib::mechanism;
namespace units = shulib::units;
using mech::LiveRefusal;
using mech::MechanismOutcome;
using mech::Tunable;
using tune_rig::TuneRig;

/// Move both twins to `in` and let them hold there `holdTicks` ticks, in step.
void holdBoth(TuneRig& a, TuneRig& b, double in, int holdTicks) {
    a.pos().moveTo(in, units::Time{8.0}, false);
    b.pos().moveTo(in, units::Time{8.0}, false);
    for (int i = 0; i < 3000 && (a.pos().running() || b.pos().running()); ++i) {
        a.tick();
        b.tick();
    }
    REQUIRE(a.pos().lastStep().outcome == MechanismOutcome::Succeeded);
    REQUIRE(b.pos().lastStep().outcome == MechanismOutcome::Succeeded);
    for (int i = 0; i < holdTicks; ++i) {
        a.tick();
        b.tick();
    }
    REQUIRE(a.volts() == b.volts());  // twins: identical so far
}
/// The hold's error NOW, output degrees (the hold loop's own: its target as degrees minus where it is).
double holdErrorDeg(TuneRig& r) {
    return mech::positionToAngleDeg(r.tuning.spec(0).position, r.pos().target()) - r.pos().jointDeg();
}
}  // namespace

// ── TA1 ──────────────────────────────────────────────────────────────────────────────
TEST_CASE("TA1: kP x4 in a hold changes the very next command by exactly ΔkP x the error (P6 inverted: never +0.0000 V)") {
    TuneRig a{tune_fixture::kCascade};
    TuneRig b{tune_fixture::kCascade};
    holdBoth(a, b, 12.0, 40);
    const double e = holdErrorDeg(a);
    const double dKp = 1.0 - 0.25;
    REQUIRE(std::abs(dKp * e) > 0.05);  // an error the change can be seen in
    const mech::Applied r = a.tuning.apply(0, Tunable::KP, 1.0);
    REQUIRE(r.ok);
    CHECK(std::string{r.sentence} == "'cascade': kP 0.25 -> 1");
    CHECK(a.mechCount("'cascade': retuned live -- kP 0.25 -> 1") == 1);
    a.tick();
    b.tick();
    CHECK(std::abs((a.volts() - b.volts()) - dKp * e) < 1e-9);
}

TEST_CASE("TA1: kI changed in a hold is BUMPLESS -- the command moves only by the new gain's share of this tick's error, not by ΔkI x the wound integral") {
    TuneRig a{tune_fixture::kCascade};
    TuneRig b{tune_fixture::kCascade};
    holdBoth(a, b, 12.0, 300);  // the plant's gravity is 1.6 V, the spec's 1.0 V: the hold's integral carries ~0.6 V
    const double e = holdErrorDeg(a);
    const double dKi = 0.375 - 0.3;
    REQUIRE(a.tuning.apply(0, Tunable::KI, 0.375).ok);
    a.tick();
    b.tick();
    const double diff = a.volts() - b.volts();
    CHECK(std::abs(diff - dKi * e * 0.01) < 1e-9);  // the I-term's own increment at the new gain, nothing more
    CHECK(std::abs(diff) < 0.01);                    // a kept accumulator would jump ΔkI x ~2 deg·s ≈ 0.15 V
}

TEST_CASE("TA1: a new top speed takes effect at the NEXT move") {
    TuneRig a{tune_fixture::kCascade};
    TuneRig b{tune_fixture::kCascade};
    REQUIRE(a.tuning.apply(0, Tunable::MaxSpeed, 2.0).ok);
    a.pos().moveTo(12.0, units::Time{10.0}, false);
    b.pos().moveTo(12.0, units::Time{10.0}, false);
    double peakA = 0.0;
    double peakB = 0.0;
    double lastA = a.pos().position();
    double lastB = b.pos().position();
    int ticksA = 0;
    for (int i = 0; i < 1100; ++i) {
        a.tick();
        b.tick();
        peakA = std::max(peakA, (a.pos().position() - lastA) / 0.01);
        peakB = std::max(peakB, (b.pos().position() - lastB) / 0.01);
        lastA = a.pos().position();
        lastB = b.pos().position();
        ticksA += a.pos().running() ? 1 : 0;
    }
    CHECK(a.pos().lastStep().outcome == MechanismOutcome::Succeeded);
    CHECK(peakA <= 2.0 * 1.15);
    CHECK(peakB > 4.0);           // the twin cruises at its starting 5.89 in/s
    CHECK(ticksA >= 590);         // 12 in at 2 in/s: six seconds at least
}

TEST_CASE("TA1: a narrowed max -- a 6 V push stops at the NEW limit (12 in + its tolerance), where its twin goes on") {
    TuneRig a{tune_fixture::kCascade};
    TuneRig b{tune_fixture::kCascade};
    REQUIRE(a.tuning.apply(0, Tunable::RangeMax, 12.0).ok);
    a.pos().jog(units::Voltage{6.0});
    b.pos().jog(units::Voltage{6.0});
    double highestA = 0.0;
    double highestB = 0.0;
    for (int i = 0; i < 600; ++i) {
        a.tick();
        b.tick();
        highestA = std::max(highestA, a.pos().position());
        highestB = std::max(highestB, b.pos().position());
    }
    CHECK(highestA <= 12.0 + 0.25);
    CHECK(highestA > 11.0);   // it got there
    CHECK(highestB > 15.0);   // the twin's limit is still 24 in
    CHECK(a.pos().holding());
}

TEST_CASE("TA1: a new stall signature trips AS THE NEW NUMBERS SAY (a fresh window from the change)") {
    TuneRig a{tune_fixture::kCascade};
    TuneRig b{tune_fixture::kCascade};
    holdBoth(a, b, 12.0, 100);
    REQUIRE(a.motors[0][0].current().value() > 0.25);  // the loaded hold draws ~0.33 A on this plant
    REQUIRE(a.tuning.apply(0, Tunable::StallAmps, 0.2).ok);
    a.tick(10);
    b.tick(10);
    REQUIRE(a.tuning.apply(0, Tunable::StallLasting, 0.3).ok);
    int ticks = 0;
    while (a.stallFaults() == 0 && ticks < 200) {
        a.tick();
        b.tick();
        ++ticks;
    }
    CHECK(a.stallFaults() == 1);
    CHECK(a.pos().lastStep().outcome == MechanismOutcome::Stalled);
    CHECK(ticks >= 30);
    CHECK(ticks <= 32);
    CHECK(b.stallFaults() == 0);  // 2.0 A, the robot file's: never at this hold
}

TEST_CASE("TA1: a new cap reaches the motors (written, read back, said); UNDO back to none writes the motor's 2.5 A default") {
    TuneRig a{tune_fixture::kCascade};
    TuneRig b{tune_fixture::kCascade};
    CHECK(a.limitAmps() == 2.5);
    REQUIRE(a.tuning.apply(0, Tunable::CurrentCap, 2.3).ok);  // above its 2.0 A stall signature (the build's rule)
    CHECK(a.limitAmps() == 2.3);
    CHECK(b.limitAmps() == 2.5);
    CHECK(a.mechCount("'cascade': current cap 2.30 A per motor (1 motor: 2.30 A in all) -- set and read back") == 1);
    REQUIRE(a.tuning.step(0, Tunable::CurrentCap, -1).ok);
    CHECK(a.limitAmps() == 2.25);
    REQUIRE(a.tuning.undo().ok);
    CHECK(a.limitAmps() == 2.3);
    REQUIRE(a.tuning.undo().ok);
    CHECK(a.limitAmps() == 2.5);
    CHECK(a.mechCount("'cascade': current cap 2.30 A removed live -- each motor's limit written back to the motor's "
                      "2.50 A default") == 1);
    // and the once-a-second re-check no longer writes a cap that is gone
    const int writes = a.motors[0][0].currentLimitWrites();
    a.tick(300);
    CHECK(a.motors[0][0].currentLimitWrites() == writes);
}

// ── TA2 ──────────────────────────────────────────────────────────────────────────────
TEST_CASE("TA2: a change while a STEP runs is refused -- the spec byte for byte, and the twins' commands equal to the end") {
    TuneRig a{tune_fixture::kCascade};
    TuneRig b{tune_fixture::kCascade};
    a.pos().moveTo(12.0, units::Time{8.0}, false);
    b.pos().moveTo(12.0, units::Time{8.0}, false);
    a.tick(20);
    b.tick(20);
    REQUIRE(a.pos().running());
    mech::MechanismSpec before;
    std::memcpy(static_cast<void*>(&before), &a.tuning.spec(0), sizeof before);
    CHECK_FALSE(a.svc.canRederive(0));
    const mech::Applied r = a.tuning.apply(0, Tunable::KP, 1.0);
    CHECK_FALSE(r.ok);
    CHECK(r.why == LiveRefusal::StepRunning);
    CHECK(std::string{r.sentence} ==
          "'cascade': kP 0.25 -> 1 refused -- a step is running on it: a change waits until the step ends");
    CHECK_FALSE(a.svc.rederive(0, "x"));  // the service's own refusal: nothing said
    CHECK(a.mechCount("retuned live") == 0);
    CHECK(std::memcmp(&before, &a.tuning.spec(0), sizeof before) == 0);
    CHECK(a.tuning.undoDepth() == 0);
    for (int i = 0; i < 900; ++i) {
        a.tick();
        b.tick();
        REQUIRE(a.volts() == b.volts());
    }
    CHECK(a.pos().lastStep().outcome == MechanismOutcome::Succeeded);
    CHECK(a.tuning.apply(0, Tunable::KP, 1.0).ok);  // between steps: applied
}

TEST_CASE("TA2: a change from INSIDE a tick (a sink called by the step's verdict) is refused; the next one, between ticks, is applied") {
    TuneRig a{tune_fixture::kCascade};
    mech::Applied inside{};
    bool insideRederive = true;
    bool called = false;
    mech::MechanismSpec before;
    std::memcpy(static_cast<void*>(&before), &a.tuning.spec(0), sizeof before);
    a.sink.hook = [&](std::string_view tag, std::string_view message) {
        if (!called && tag == "MECH" && message.find("SUCCEEDED") != std::string_view::npos) {
            called = true;
            inside = a.tuning.apply(0, Tunable::KP, 1.0);
            insideRederive = a.svc.rederive(0, "from inside");
        }
    };
    a.pos().moveTo(6.0, units::Time{8.0}, false);
    a.runStep();
    REQUIRE(called);
    CHECK_FALSE(inside.ok);
    CHECK(inside.why == LiveRefusal::InsideATick);
    CHECK_FALSE(insideRederive);
    CHECK(std::memcmp(&before, &a.tuning.spec(0), sizeof before) == 0);
    CHECK(a.mechCount("retuned live") == 0);
    a.sink.hook = nullptr;
    CHECK(a.tuning.apply(0, Tunable::KP, 1.0).ok);
}

// ── TA4 ──────────────────────────────────────────────────────────────────────────────
TEST_CASE("TA4: UNDO restores each previous spec BIT FOR BIT, 16 deep; the 17th is refused; RESET is the robot file's, bit for bit") {
    TuneRig r{tune_fixture::kCascade};
    // a fresh LiveTuning holds the robot file's table, byte for byte
    CHECK(std::memcmp(&r.tuning.table(), &tune_fixture::kCascade, sizeof(mech::MechanismTable)) == 0);
    struct Change {
        Tunable t;
        int dir;
    };
    const Change changes[17] = {{Tunable::KP, +1},        {Tunable::CurrentCap, -1}, {Tunable::RangeMax, -1},
                                {Tunable::StallAmps, -1}, {Tunable::KI, +1},         {Tunable::MaxSpeed, -1},
                                {Tunable::KD, -1},        {Tunable::GravityHold, +1}, {Tunable::Within, +1},
                                {Tunable::Settle, +1},    {Tunable::KA, +1},         {Tunable::StallRpm, +1},
                                {Tunable::StallLasting, -1}, {Tunable::KV, -1},     {Tunable::KS, +1},
                                {Tunable::MaxAcceleration, -1}, {Tunable::CurrentCap, -1}};
    std::vector<mech::MechanismSpec> snap(18);
    std::memcpy(static_cast<void*>(&snap[0]), &r.tuning.spec(0), sizeof(mech::MechanismSpec));
    for (std::size_t k = 0; k < 17; ++k) {
        const mech::Applied a = r.tuning.step(0, changes[k].t, changes[k].dir);
        INFO(a.sentence);
        REQUIRE(a.ok);
        std::memcpy(static_cast<void*>(&snap[k + 1]), &r.tuning.spec(0), sizeof(mech::MechanismSpec));
    }
    CHECK(r.tuning.undoDepth() == 16);
    CHECK(r.limitAmps() == 2.40);
    for (std::size_t j = 1; j <= 16; ++j) {
        const mech::Applied a = r.tuning.undo();
        INFO(a.sentence);
        REQUIRE(a.ok);
        CHECK(std::memcmp(&snap[17 - j], &r.tuning.spec(0), sizeof(mech::MechanismSpec)) == 0);
    }
    CHECK(r.limitAmps() == 2.5);  // the slot re-derived what each undo restored: the cap, added second, is gone
    const mech::Applied seventeenth = r.tuning.undo();
    CHECK_FALSE(seventeenth.ok);
    CHECK(seventeenth.why == LiveRefusal::NothingToUndo);
    CHECK(std::string{seventeenth.sentence} == "nothing to undo");
    CHECK(std::memcmp(&snap[1], &r.tuning.spec(0), sizeof(mech::MechanismSpec)) == 0);  // the first change fell off
    CHECK(r.tuning.changed(0));
    // RESET: the robot file's, bit for bit — and itself undoable
    const mech::Applied reset = r.tuning.reset(0);
    REQUIRE(reset.ok);
    CHECK(std::string{reset.sentence} == "'cascade': RESET: the robot file's numbers");
    CHECK(std::memcmp(&tune_fixture::kCascade.specs[0], &r.tuning.spec(0), sizeof(mech::MechanismSpec)) == 0);
    CHECK_FALSE(r.tuning.changed(0));
    CHECK(r.limitAmps() == 2.5);
    CHECK(r.tuning.reset(0).ok);  // already the file's: nothing recorded
    CHECK(r.tuning.undoDepth() == 1);
    REQUIRE(r.tuning.undo().ok);
    CHECK(std::memcmp(&snap[1], &r.tuning.spec(0), sizeof(mech::MechanismSpec)) == 0);
    CHECK(r.tuning.undoDepth() == 0);
}

TEST_CASE("TA4: an undo waits while that mechanism's step runs (the record kept); across mechanisms it undoes the LAST change, naming it") {
    TuneRig r{tune_fixture::kFlyAndPivot};
    REQUIRE(r.tuning.step(0, Tunable::KP, +1).ok);
    REQUIRE(r.tuning.step(1, Tunable::MaxVolts, -1).ok);
    r.svc.speed(0).spinUp(3000.0, units::Time{3.0}, false);
    r.tick(5);
    REQUIRE(r.svc.speed(0).running());
    const mech::Applied u = r.tuning.undo();  // the pivot's change: its step is not running
    CHECK(u.ok);
    CHECK(std::string{u.sentence} == "'pivot': undo: max volts 11.95 -> 12");
    CHECK(r.tuning.undoDepth() == 1);
    const mech::Applied w = r.tuning.undo();  // the flywheel's: its step runs
    CHECK_FALSE(w.ok);
    CHECK(w.why == LiveRefusal::StepRunning);
    CHECK(std::string{w.sentence} ==
          "'flywheel': undo refused -- a step is running on it: a change waits until the step ends");
    CHECK(r.tuning.undoDepth() == 1);
    r.runStep(0);
    CHECK(r.tuning.undo().ok);
    CHECK_FALSE(r.tuning.changed(0));
}

// ── TA6 ──────────────────────────────────────────────────────────────────────────────
TEST_CASE("TA6: presets are never live -- after every live change find(lift[\"high\"]) still finds the slot, at the file's 22 in") {
    TuneRig r{tune_fixture::kTallLift};
    CHECK(r.svc.find(tune_fixture::tallLift["high"]) == &r.pos());
    for (const Tunable t : {Tunable::KP, Tunable::KI, Tunable::MaxSpeed, Tunable::Within, Tunable::GravityHold,
                            Tunable::StallAmps, Tunable::CurrentCap, Tunable::RangeMax}) {
        INFO(mech::parameterOf(t).name);
        REQUIRE(r.tuning.step(0, t, -1).ok);
        CHECK(r.svc.find(tune_fixture::tallLift["high"]) == &r.pos());
        CHECK(r.tuning.spec(0).position.presets[2].value == 22.0);
    }
    // a range narrowed past the preset is the BUILD's refusal — the preset stays where the file put it
    const mech::Applied past = r.tuning.apply(0, Tunable::RangeMax, 21.0);
    CHECK_FALSE(past.ok);
    CHECK(past.build == mech::Refusal::PositionAPresetIsOutsideTheTravelLimits);
    CHECK(r.svc.find(tune_fixture::tallLift["high"]) == &r.pos());
}

// ── TA7 ──────────────────────────────────────────────────────────────────────────────
TEST_CASE("TA7: a narrowed max BELOW the target it holds is REFUSED -- hold, spec and motors untouched; after a move inside it, accepted") {
    TuneRig a{tune_fixture::kCascade};
    TuneRig b{tune_fixture::kCascade};
    holdBoth(a, b, 20.0, 50);
    REQUIRE(a.pos().holding());
    mech::MechanismSpec before;
    std::memcpy(static_cast<void*>(&before), &a.tuning.spec(0), sizeof before);
    mech::Applied r{};
    CHECK_NOTHROW(r = a.tuning.apply(0, Tunable::RangeMax, 12.0));
    CHECK_FALSE(r.ok);
    CHECK(r.why == LiveRefusal::LeavesOutHeldTarget);
    CHECK(std::string{r.sentence} ==
          "'cascade': upper limit 24 -> 12 refused -- it holds at 20.00 in, outside the new limits: move it inside first");
    CHECK(std::memcmp(&before, &a.tuning.spec(0), sizeof before) == 0);
    for (int i = 0; i < 200; ++i) {
        a.tick();
        b.tick();
        REQUIRE(a.volts() == b.volts());
    }
    CHECK(a.pos().position() == doctest::Approx(20.0).epsilon(0.01));
    // a TEST move inside the new limits, then the same narrowing: accepted, and the hold stays put
    a.pos().moveTo(10.0, units::Time{8.0}, false);
    a.runStep();
    REQUIRE(a.pos().lastStep().outcome == MechanismOutcome::Succeeded);
    REQUIRE(a.tuning.apply(0, Tunable::RangeMax, 12.0).ok);
    a.tick(200);
    CHECK(a.pos().holding());
    CHECK(std::abs(a.pos().position() - 10.0) < 0.25);
}

TEST_CASE("TA7: the EQUALITY edge -- a hold at exactly 12 in takes a max of exactly 12 in; a hair below it is refused") {
    TuneRig a{tune_fixture::kCascade};
    a.pos().moveTo(12.0, units::Time{8.0}, false);
    a.runStep();
    REQUIRE(a.pos().holding());
    CHECK_FALSE(a.tuning.apply(0, Tunable::RangeMax, std::nextafter(12.0, 0.0)).ok);
    CHECK(a.tuning.apply(0, Tunable::RangeMax, 12.0).ok);
}

TEST_CASE("TA7: a held button moving it -- a travel limit change is refused until it lets go; other numbers are not") {
    TuneRig a{tune_fixture::kCascade};
    a.pos().jog(units::Voltage{4.0});
    a.tick(30);
    REQUIRE(a.pos().heldButtonMoving());
    const mech::Applied r = a.tuning.apply(0, Tunable::RangeMax, 20.0);
    CHECK_FALSE(r.ok);
    CHECK(r.why == LiveRefusal::HeldButtonMoving);
    CHECK(a.tuning.apply(0, Tunable::KP, 0.3).ok);  // a gain: fine mid-push
    a.pos().holdHere();
    a.tick();
    CHECK(a.tuning.apply(0, Tunable::RangeMax, 20.0).ok);
}

// ── TA8 ──────────────────────────────────────────────────────────────────────────────
namespace {
/// Two flywheel rigs spun up to 3000 rpm and at speed, in step.
void spinBoth(TuneRig& a, TuneRig& b) {
    a.svc.speed(0).spinUp(3000.0, units::Time{3.0}, false);
    b.svc.speed(0).spinUp(3000.0, units::Time{3.0}, false);
    for (int i = 0; i < 400 && a.svc.speed(0).running(); ++i) {
        a.tick();
        b.tick();
    }
    REQUIRE(a.svc.speed(0).lastStep().outcome == MechanismOutcome::Succeeded);
    a.tick(30);
    b.tick(30);
    REQUIRE(a.volts() == b.volts());
}
constexpr double kTargetRadS = 3000.0 * 2.0 * 3.14159265358979323846 / 60.0;
}  // namespace

TEST_CASE("TA8: a FLYWHEEL's live set takes effect at the next tick -- kP, kV, kS by their exact share; kI bumpless") {
    {
        TuneRig a{tune_fixture::kFlyAndPivot};
        TuneRig b{tune_fixture::kFlyAndPivot};
        spinBoth(a, b);
        const double e = kTargetRadS - a.svc.speed(0).rpm() * 2.0 * 3.14159265358979323846 / 60.0;
        REQUIRE(a.tuning.apply(0, Tunable::KP, 0.2).ok);
        a.tick();
        b.tick();
        CHECK(std::abs((a.volts() - b.volts()) - 0.15 * e) < 1e-9);
    }
    {
        TuneRig a{tune_fixture::kFlyAndPivot};
        TuneRig b{tune_fixture::kFlyAndPivot};
        spinBoth(a, b);
        const double kV = a.tuning.value(0, Tunable::KV);
        REQUIRE(a.tuning.step(0, Tunable::KV, +1, /*fine=*/true).ok);  // x1.05: 10 V of feedforward to 10.5 (x1.25 would clamp at 12)
        const double dKv = a.tuning.value(0, Tunable::KV) - kV;
        a.tick();
        b.tick();
        CHECK(std::abs((a.volts() - b.volts()) - dKv * kTargetRadS) < 1e-9);
    }
    {
        TuneRig a{tune_fixture::kFlyAndPivot};
        TuneRig b{tune_fixture::kFlyAndPivot};
        spinBoth(a, b);
        REQUIRE(a.tuning.step(0, Tunable::KS, +1).ok);
        a.tick();
        b.tick();
        CHECK(std::abs((a.volts() - b.volts()) - 0.05) < 1e-9);
    }
    {
        TuneRig a{tune_fixture::kFlyAndPivot};
        TuneRig b{tune_fixture::kFlyAndPivot};
        spinBoth(a, b);
        const double e = kTargetRadS - a.svc.speed(0).rpm() * 2.0 * 3.14159265358979323846 / 60.0;
        REQUIRE(a.tuning.apply(0, Tunable::KI, 0.25).ok);
        a.tick();
        b.tick();
        CHECK(std::abs((a.volts() - b.volts()) - 0.05 * e * 0.01) < 1e-9);  // ΔkI x this tick's increment only
    }
}

TEST_CASE("TA8: a FLYWHEEL's band, stall signature and cap take effect; it offers no kA") {
    TuneRig a{tune_fixture::kFlyAndPivot};
    TuneRig b{tune_fixture::kFlyAndPivot};
    spinBoth(a, b);
    REQUIRE(a.svc.speed(0).atSpeed());
    REQUIRE(a.tuning.apply(0, Tunable::AtSpeedBand, 0.001).ok);
    a.tick();
    b.tick();
    CHECK_FALSE(a.svc.speed(0).atSpeed());
    CHECK(b.svc.speed(0).atSpeed());
    // a stall signature it now meets while spinning (motor 500 rpm < 600; its current over 0.05 A): STALLED in 0.5 s
    REQUIRE(a.tuning.apply(0, Tunable::StallRpm, 600.0).ok);
    REQUIRE(a.tuning.apply(0, Tunable::StallAmps, 0.05).ok);
    int ticks = 0;
    while (a.stallFaults() == 0 && ticks < 100) {
        a.tick();
        b.tick();
        ++ticks;
    }
    CHECK(a.stallFaults() == 1);
    CHECK(ticks >= 50);
    CHECK(ticks <= 52);
    CHECK(b.stallFaults() == 0);
    REQUIRE(a.tuning.apply(0, Tunable::CurrentCap, 1.0).ok);
    CHECK(a.limitAmps(0) == 1.0);
    const mech::ParameterList l = mech::parametersOf(a.tuning.spec(0));
    for (std::size_t i = 0; i < l.count; ++i) {
        CHECK(l.items[i].id != Tunable::KA);
    }
    CHECK(a.tuning.apply(0, Tunable::KA, 0.01).why == LiveRefusal::NotLive);
}

TEST_CASE("TA8: a POWER mechanism's live set -- a lowered maxVolts clamps a held button at once; its stall signature; its cap") {
    TuneRig a{tune_fixture::kFlyAndPivot};
    TuneRig b{tune_fixture::kFlyAndPivot};
    a.svc.power(1).drive(units::Voltage{12.0});
    b.svc.power(1).drive(units::Voltage{12.0});
    a.tick();
    b.tick();
    REQUIRE(a.volts(1) == 12.0);
    REQUIRE(a.tuning.apply(1, Tunable::MaxVolts, 6.0).ok);
    a.tick();
    b.tick();
    CHECK(a.volts(1) == 6.0);
    CHECK(b.volts(1) == 12.0);
    // the pivot pushed against something: 1.0 A, still — under its 1.5 A signature; at 0.8 A it trips in 0.3 s
    a.motors[1][0].setCurrent(units::Current{1.0});
    b.motors[1][0].setCurrent(units::Current{1.0});
    a.tick(40);
    b.tick(40);
    REQUIRE(a.stallFaults() == 0);
    REQUIRE(a.tuning.apply(1, Tunable::StallAmps, 0.8).ok);
    int ticks = 0;
    while (a.stallFaults() == 0 && ticks < 100) {
        a.tick();
        b.tick();
        ++ticks;
    }
    CHECK(a.stallFaults() == 1);
    CHECK(ticks >= 30);
    CHECK(ticks <= 32);
    CHECK(b.stallFaults() == 0);
    REQUIRE(a.tuning.apply(1, Tunable::CurrentCap, 1.2).ok);
    CHECK(a.limitAmps(1) == 1.2);
}

// ── the service's own half ──────────────────────────────────────────────────────────
TEST_CASE("MechanismService::rederive: one Info line in the brief's form; a null `what` says the numbers were re-read; preconditions") {
    TuneRig r{tune_fixture::kCascade};
    CHECK(r.svc.canRederive(0));
    CHECK(r.svc.rederive(0, nullptr));
    CHECK(r.mechCount("'cascade': retuned live -- its numbers re-read") == 1);
    CHECK_FALSE(r.svc.canRederive(1));
    CHECK_THROWS_AS((void)r.svc.rederive(1), shulib::PreconditionError);
    shulib::hal::fake::FakeClock clock;
    pos_rig::PlantWorld w;
    w.clock = &clock;
    mech::MechanismService unbound{w};
    CHECK_FALSE(unbound.canRederive(0));
    CHECK_THROWS_AS((void)unbound.rederive(0), shulib::PreconditionError);
}

TEST_CASE("LiveTuning::attach refuses a service bound over anything but its own table (landmine 10: a change must reach the slots)") {
    TuneRig r{tune_fixture::kCascade};
    mech::LiveTuning other{tune_fixture::kCascade};
    CHECK_THROWS_AS(other.attach(r.svc), shulib::PreconditionError);  // r.svc runs r.tuning's copy, not this one
    CHECK_THROWS_AS(r.tuning.attach(r.svc), shulib::PreconditionError);  // twice
    CHECK_THROWS_AS((void)other.apply(0, Tunable::KP, 0.3), shulib::PreconditionError);  // never attached
}

// ── the chapters quote these lines EXACTLY ───────────────────────────────────────────
namespace {
/// A chapter's text with every run of whitespace one space (a quoted line may wrap in the source).
std::string chapter(const std::string& file) {
    std::ifstream in{std::string{SHULIB_SOURCE_DIR} + "/docs/guide/" + file};
    REQUIRE(in.good());
    std::stringstream buf;
    buf << in.rdbuf();
    std::string out;
    bool space = false;
    for (const char c : buf.str()) {
        const bool ws = c == ' ' || c == '\n' || c == '\t';
        if (ws && !space) {
            out += ' ';
        } else if (!ws) {
            out += c;
        }
        space = ws;
    }
    return out;
}
}  // namespace

TEST_CASE("guide 11 quotes live tuning's MECH and TUNE lines exactly as the code makes them; guide 18 its COPY block") {
    TuneRig r{tune_fixture::kTallLift};  // 'lift', its file stating .tuning = {.kD = 0.01}
    REQUIRE(r.tuning.step(0, Tunable::KP, +1).ok);
    REQUIRE(r.tuning.undo().ok);
    REQUIRE(r.tuning.step(0, Tunable::KP, +1).ok);
    REQUIRE(r.tuning.reset(0).ok);
    REQUIRE(r.tuning.apply(0, Tunable::CurrentCap, 2.3).ok);
    REQUIRE(r.tuning.undo().ok);  // the cap removed live, every motor back at its default
    REQUIRE(r.tuning.apply(0, Tunable::CurrentCap, 2.3).ok);
    r.motors[0][0].refuseCurrentLimit(true);
    REQUIRE(r.tuning.undo().ok);  // ... and a motor that would not go back
    CHECK(r.limitAmps() == 2.3);
    const std::string g11 = chapter("11-reading-the-diagnostics.md");
    const char* mech[] = {
        "'lift': retuned live -- kP 0.25 -> 0.3125",
        "'lift': retuned live -- undo: kP 0.3125 -> 0.25",
        "'lift': retuned live -- RESET: the robot file's numbers",
        "'lift': current cap 2.30 A removed live -- each motor's limit written back to the motor's 2.50 A default",
        "'lift': motor 1 (port 1) did not go back to the motor's 2.50 A default after its live cap was removed: it reads 2.30 A",
    };
    for (const char* line : mech) {
        INFO(line);
        CHECK(r.mechCount(line) >= 1);
        CHECK(g11.find(std::string{"`"} + line + "`") != std::string::npos);
    }
    // the TUNE block: guide 11's two member examples and its end line, guide 18's whole block
    TuneRig b{tune_fixture::kTallLift};
    REQUIRE(b.tuning.step(0, Tunable::KP, +1).ok);
    REQUIRE(b.tuning.apply(0, Tunable::MaxSpeed, 2.0).ok);
    shulib::hal::fake::FakeTelemetrySink s;
    REQUIRE(b.tuning.copyBlock(0, s) == 4);
    std::vector<std::string> block;
    for (int i = 0; i < s.size(); ++i) {
        block.push_back(s.at(i).message);
    }
    REQUIRE(block.size() == 6U);
    const std::string begin = "'lift': copy begins -- write these into its declaration (tuned live): ";
    CHECK(block[0].rfind(begin, 0) == 0);
    CHECK(g11.find("`" + begin + "src/robots/mechanism_table.hpp:41`") != std::string::npos);
    CHECK(g11.find("`.tuning.kP = 0.3125`") != std::string::npos);
    CHECK(g11.find("`'lift': copy ends -- 2 lines`") != std::string::npos);
    TuneRig st{tune_fixture::kTallLift};
    REQUIRE(st.tuning.step(0, Tunable::StallAmps, -1).ok);
    shulib::hal::fake::FakeTelemetrySink s2;
    REQUIRE(st.tuning.copyBlock(0, s2) == 1);
    CHECK(g11.find("`" + std::string{s2.at(1).message} + "`") != std::string::npos);  // the .stall example
    const std::string g18 = chapter("18-mechanisms.md");
    std::string listing = begin + "src/robots/mechanism_table.hpp:41";
    for (std::size_t i = 1; i < block.size(); ++i) {
        listing += " " + block[i];
    }
    CHECK(g18.find("```text " + listing + " ```") != std::string::npos);
}

// ── TUNE1-A's review (2026-11-10): what the first tests left unguarded ─────────────────────────────
namespace {
/// Three twins in step: `b` keeps the robot file's numbers; `a` and `c` take number `t` at `va` and `vc`
/// (between steps), then `start` runs on all three and they tick together until `a`'s command first
/// differs from `b`'s. Until that tick the three are identical, so each difference is exactly the changed
/// gain's share of ONE input both shared — and their RATIO is the ratio of the two changes.
struct Split {
    double da = 0.0;  ///< a's command minus b's, at the first tick they differ
    double dc = 0.0;  ///< c's minus b's, the same tick
    int tick = -1;    ///< which tick after `start` (-1: never within `cap`)
    bool running = false;  ///< a's step was still running then
};
template <class Start>
Split splitAfter(TuneRig& a, TuneRig& b, TuneRig& c, Tunable t, double va, double vc, Start&& start, int cap = 2000) {
    REQUIRE(a.tuning.apply(0, t, va).ok);
    REQUIRE(c.tuning.apply(0, t, vc).ok);
    start(a);
    start(b);
    start(c);
    Split s{};
    for (int i = 1; i <= cap; ++i) {
        a.tick();
        b.tick();
        c.tick();
        if (a.volts() != b.volts()) {
            s = Split{a.volts() - b.volts(), c.volts() - b.volts(), i, a.pos().running()};
            break;
        }
    }
    return s;
}
}  // namespace

TEST_CASE("TA1 (review): a live kP reaches the MOVE loop -- the first move tick the twins differ, inside the profile, by ΔkP x the same error") {
    TuneRig a{tune_fixture::kCascade};
    TuneRig b{tune_fixture::kCascade};
    TuneRig c{tune_fixture::kCascade};
    const Split s = splitAfter(a, b, c, Tunable::KP, 1.0, 0.5,
                               [](TuneRig& r) { r.pos().moveTo(12.0, units::Time{8.0}, false); });
    REQUIRE(s.tick > 0);
    CHECK(s.tick <= 3);    // in the move's first ticks — a move loop that kept its gains differs only at the hold, ~2 s on
    CHECK(s.running);
    REQUIRE(std::abs(s.dc) > 1e-6);
    CHECK(std::abs(s.da / s.dc - (1.0 - 0.25) / (0.5 - 0.25)) < 1e-9);  // ΔkP_a·e / ΔkP_c·e: one error, two gains
}

TEST_CASE("TA1 (review): a live kD reaches the MOVE loop and the HOLD loop -- each by ΔkD x the same measured rate") {
    {
        TuneRig a{tune_fixture::kCascade};
        TuneRig b{tune_fixture::kCascade};
        TuneRig c{tune_fixture::kCascade};
        const Split s = splitAfter(a, b, c, Tunable::KD, 0.012, 0.008,
                                   [](TuneRig& r) { r.pos().moveTo(12.0, units::Time{8.0}, false); });
        REQUIRE(s.tick > 0);
        CHECK(s.tick <= 3);
        CHECK(s.running);
        REQUIRE(std::abs(s.dc) > 1e-9);
        CHECK(std::abs(s.da / s.dc - (0.012 - 0.004) / (0.008 - 0.004)) < 1e-6);
    }
    {
        TuneRig a{tune_fixture::kCascade};
        TuneRig b{tune_fixture::kCascade};
        TuneRig c{tune_fixture::kCascade};
        for (TuneRig* r : {&a, &b, &c}) {
            r->pos().moveTo(12.0, units::Time{8.0}, false);
        }
        for (int i = 0; i < 3000 && a.pos().running(); ++i) {
            a.tick();
            b.tick();
            c.tick();
        }
        REQUIRE(a.pos().holding());
        a.tick(5);
        b.tick(5);
        c.tick(5);
        const Split s = splitAfter(a, b, c, Tunable::KD, 0.012, 0.008, [](TuneRig&) {}, 1);
        REQUIRE(s.tick == 1);  // the very next tick of the HOLD
        CHECK_FALSE(s.running);
        REQUIRE(std::abs(s.dc) > 1e-12);
        CHECK(std::abs(s.da / s.dc - (0.012 - 0.004) / (0.008 - 0.004)) < 1e-6);
    }
}

TEST_CASE("TA1 (review): a lowered ACCELERATION takes effect in the soft stop -- a push toward a limit is taken over EARLIER than its twin's") {
    TuneRig a{tune_fixture::kCascade};
    TuneRig b{tune_fixture::kCascade};
    REQUIRE(a.tuning.apply(0, Tunable::MaxAcceleration, 4.0).ok);  // from the starting 23.56 in/s^2
    a.pos().jog(units::Voltage{6.0});
    b.pos().jog(units::Voltage{6.0});
    int tickA = -1;
    int tickB = -1;
    for (int i = 1; i <= 1500 && (tickA < 0 || tickB < 0); ++i) {
        a.tick();
        b.tick();
        tickA = tickA < 0 && a.pos().softStops() > 0 ? i : tickA;
        tickB = tickB < 0 && b.pos().softStops() > 0 ? i : tickB;
    }
    REQUIRE(tickA > 0);
    REQUIRE(tickB > 0);
    CHECK(tickA < tickB);  // a stop at 4 in/s^2 needs ~6x the distance of one at 23.56
}

TEST_CASE("TA1 (review): a live HOMING signature takes effect -- a longer lasting trips the homing stall exactly that much later") {
    TuneRig a{tune_fixture::kMidHome};
    TuneRig b{tune_fixture::kMidHome};
    // this plant's −2 V push draws at most 0.42 A against its stop: the file's 0.6 A never trips (TIMED_OUT)
    REQUIRE(a.tuning.apply(0, Tunable::HomeStallAmps, 0.3).ok);
    REQUIRE(b.tuning.apply(0, Tunable::HomeStallAmps, 0.3).ok);
    REQUIRE(a.tuning.apply(0, Tunable::HomeStallLasting, 0.5).ok);  // the file's 0.2 s
    a.pos().home(units::Time{6.0});
    b.pos().home(units::Time{6.0});
    const int ticksA = a.runStep();
    const int ticksB = b.runStep();
    REQUIRE(b.pos().lastStep().outcome == MechanismOutcome::Succeeded);
    REQUIRE(a.pos().lastStep().outcome == MechanismOutcome::Succeeded);
    CHECK(ticksA - ticksB >= 29);
    CHECK(ticksA - ticksB <= 31);  // 0.3 s more at 10 ms a tick
}

TEST_CASE("TA1 (review): an UNCHANGED stall signature keeps its window -- a kP change half-way through it trips at the same tick as its twin") {
    TuneRig a{tune_fixture::kCascade};
    TuneRig b{tune_fixture::kCascade};
    holdBoth(a, b, 12.0, 100);
    REQUIRE(a.motors[0][0].current().value() > 0.25);
    REQUIRE(a.tuning.apply(0, Tunable::StallAmps, 0.2).ok);  // both: a fresh 0.5 s window from here
    REQUIRE(b.tuning.apply(0, Tunable::StallAmps, 0.2).ok);
    a.tick(25);
    b.tick(25);
    REQUIRE(a.stallFaults() == 0);
    REQUIRE(a.tuning.step(0, Tunable::KP, +1, /*fine=*/true).ok);  // half-way: a gain, the signature unchanged
    int ticksA = -1;
    int ticksB = -1;
    for (int i = 1; i <= 100 && (ticksA < 0 || ticksB < 0); ++i) {
        a.tick();
        b.tick();
        ticksA = ticksA < 0 && a.stallFaults() > 0 ? i : ticksA;
        ticksB = ticksB < 0 && b.stallFaults() > 0 ? i : ticksB;
    }
    REQUIRE(ticksB > 0);
    CHECK(ticksA == ticksB);  // a rebuilt detector would trip 25 ticks later (a delayed STALLED)
}

TEST_CASE("TA1 (review): a stall signature raised to within 0.2 A of a stated cap is warned again, as at boot (HA-223)") {
    TuneRig a{tune_fixture::kMidHome};  // cap 1.5 A, stall 1.0 A
    const char* warn = "'mid home lift': its stall signature (1.35 A) is within 0.2 A of its current cap (1.50 A): a "
                       "capped motor may read a little under its cap, so it may never trip (HA-223)";
    CHECK(a.mechCount(warn) == 0);
    REQUIRE(a.tuning.apply(0, Tunable::StallAmps, 1.35).ok);
    CHECK(a.mechCount(warn) == 1);
    CHECK(a.limitAmps() == 1.5);  // the cap itself unchanged, written again
}

TEST_CASE("TA8 (review): a flywheel's kI changed TWICE between ticks is still bumpless -- the term was kept under each") {
    TuneRig a{tune_fixture::kFlyAndPivot};
    TuneRig b{tune_fixture::kFlyAndPivot};
    spinBoth(a, b);
    const double e = kTargetRadS - a.svc.speed(0).rpm() * 2.0 * 3.14159265358979323846 / 60.0;
    REQUIRE(a.tuning.apply(0, Tunable::KI, 0.25).ok);
    REQUIRE(a.tuning.apply(0, Tunable::KI, 0.3).ok);
    a.tick();
    b.tick();
    CHECK(std::abs((a.volts() - b.volts()) - 0.1 * e * 0.01) < 1e-9);  // (0.3 − 0.2) x this tick's increment only
}

// ── rule 3 at both limits, for undo and reset (the review) ────────────────────────────────────
namespace {
/// The switch-homed lift over a plant with no gravity, its switch pressed at 12 in, HOMED there.
struct SwitchRig : TuneRig {
    SwitchRig()
        : TuneRig{tune_fixture::kSwitchLift, tune_rig::liftPlant(0.0),
                  pos_rig::flywheelPlant(600, 1.0 / 6.0, 1.0, 0.4, 0.2), 0.01, tune_rig::switchAt(12.0)} {
        pos().home(units::Time{6.0});
        runStep();
        REQUIRE(pos().lastStep().outcome == MechanismOutcome::Succeeded);
        REQUIRE(pos().homed());
    }
    void moveAndHold(double in) {
        pos().moveTo(in, units::Time{8.0}, false);
        runStep();
        REQUIRE(pos().lastStep().outcome == MechanismOutcome::Succeeded);
        REQUIRE(pos().holding());
    }
};
}  // namespace

TEST_CASE("TA7 (review): the LOWER limit -- raised past the held target, refused; at EXACTLY it, accepted; one ulp past, refused") {
    SwitchRig r;
    r.moveAndHold(8.0);
    mech::MechanismSpec before;
    std::memcpy(static_cast<void*>(&before), &r.tuning.spec(0), sizeof before);
    mech::Applied a{};
    CHECK_NOTHROW(a = r.tuning.apply(0, Tunable::RangeMin, 9.0));
    CHECK_FALSE(a.ok);
    CHECK(a.why == LiveRefusal::LeavesOutHeldTarget);
    CHECK(std::string{a.sentence} ==
          "'switch lift': lower limit 0 -> 9 refused -- it holds at 8.00 in, outside the new limits: move it inside first");
    CHECK(std::memcmp(&before, &r.tuning.spec(0), sizeof before) == 0);
    // the equality edge lands exactly: the hold's target IS positionToAngleDeg(8 in), the candidate's limit too
    const double hair = std::nextafter(8.0, 9.0);
    REQUIRE(mech::positionToAngleDeg(r.tuning.spec(0).position, hair) >
            mech::positionToAngleDeg(r.tuning.spec(0).position, 8.0));
    CHECK(r.tuning.apply(0, Tunable::RangeMin, hair).why == LiveRefusal::LeavesOutHeldTarget);
    CHECK(r.tuning.apply(0, Tunable::RangeMin, 8.0).ok);
}

TEST_CASE("TA7 (review): a held button moving it -- a LOWER limit change is refused too") {
    SwitchRig r;
    r.pos().jog(units::Voltage{-4.0});
    r.tick(20);
    REQUIRE(r.pos().heldButtonMoving());
    CHECK(r.tuning.apply(0, Tunable::RangeMin, 1.0).why == LiveRefusal::HeldButtonMoving);
    r.pos().holdHere();
    r.tick();
    CHECK(r.tuning.apply(0, Tunable::RangeMin, 1.0).ok);  // let go: inside, applied
}

TEST_CASE("TA7 (review): holding NOTHING -- a limit narrowed past WHERE IT IS is refused (its next hold would jump); at exactly it, accepted") {
    TuneRig a{tune_fixture::kCascade};
    a.pos().moveTo(20.0, units::Time{8.0}, false);
    a.runStep();
    REQUIRE(a.pos().lastStep().outcome == MechanismOutcome::Succeeded);
    a.tick(50);
    a.pos().release();  // any controller button's STOP: the slot is no longer active
    a.tick();
    REQUIRE_FALSE(a.pos().holding());
    REQUIRE(a.pos().homed());
    const double at = a.pos().position();
    REQUIRE(at > 19.0);
    mech::MechanismSpec before;
    std::memcpy(static_cast<void*>(&before), &a.tuning.spec(0), sizeof before);
    mech::Applied r{};
    CHECK_NOTHROW(r = a.tuning.apply(0, Tunable::RangeMax, 12.0));
    CHECK_FALSE(r.ok);
    CHECK(r.why == LiveRefusal::LeavesOutWhereItIs);
    char want[200];
    std::snprintf(want, sizeof want,
                  "'cascade': upper limit 24 -> 12 refused -- it is at %.2f in, outside the new limits: move it inside first",
                  at);
    CHECK(std::string{r.sentence} == want);
    CHECK(std::memcmp(&before, &a.tuning.spec(0), sizeof before) == 0);
    // the HOME of a lift that rests at its stop holds WHERE IT IS: with the limit refused, no jump
    a.pos().home(units::Time{2.0});
    a.tick();
    CHECK(std::abs(a.pos().target() - at) < 0.05);
    CHECK(std::abs(a.volts()) < 3.0);  // the reviewer's probe: −11 V when the narrowing was accepted
    a.pos().release();
    a.tick();
    // the equality edge, exactly: a max whose degrees ARE where it is is accepted; one ulp under it is not
    const double jd = a.pos().jointDeg();
    const mech::PositionSpec& p = a.tuning.spec(0).position;
    double edge = a.pos().position();
    for (int k = 0; k < 64 && mech::positionToAngleDeg(p, edge) != jd; ++k) {
        edge = mech::positionToAngleDeg(p, edge) < jd ? std::nextafter(edge, 1e9) : std::nextafter(edge, -1e9);
    }
    REQUIRE(mech::positionToAngleDeg(p, edge) == jd);
    const double under = std::nextafter(edge, 0.0);
    REQUIRE(mech::positionToAngleDeg(p, under) < jd);
    CHECK(a.tuning.apply(0, Tunable::RangeMax, under).why == LiveRefusal::LeavesOutWhereItIs);
    CHECK(a.tuning.apply(0, Tunable::RangeMax, edge).ok);
    a.pos().holdHere();  // the let-go's hold: where it is, no jump
    a.tick();
    CHECK(std::abs(a.volts()) < 3.0);
}

TEST_CASE("TA7 (review): holding nothing -- a limit that leaves it no further outside than before is accepted; widening always is") {
    SwitchRig r;
    r.moveAndHold(8.0);
    r.pos().release();
    r.tick();
    REQUIRE_FALSE(r.pos().holding());
    CHECK(r.tuning.apply(0, Tunable::RangeMin, 9.0).why == LiveRefusal::LeavesOutWhereItIs);  // the LOWER one too
    CHECK(r.tuning.apply(0, Tunable::RangeMin, 7.0).ok);
    CHECK(r.tuning.apply(0, Tunable::RangeMax, 23.0).ok);   // the far end: it stays inside
    CHECK(r.tuning.apply(0, Tunable::RangeMin, 0.0).ok);    // widened back toward the file's
}

TEST_CASE("TA4 (review): an UNDO that would narrow a range past the hold is refused, the record kept -- never the rederive precondition's throw") {
    TuneRig a{tune_fixture::kCascade};
    REQUIRE(a.tuning.apply(0, Tunable::RangeMax, 12.0).ok);
    REQUIRE(a.tuning.apply(0, Tunable::RangeMax, 20.0).ok);
    a.pos().moveTo(18.0, units::Time{8.0}, false);
    a.runStep();
    REQUIRE(a.pos().holding());
    mech::Applied u{};
    CHECK_NOTHROW(u = a.tuning.undo());
    CHECK_FALSE(u.ok);
    CHECK(u.why == LiveRefusal::LeavesOutHeldTarget);
    CHECK(std::string{u.sentence} == "'cascade': undo refused -- the new travel limits leave out the target it is "
                                     "holding: move it inside first");
    CHECK(a.tuning.undoDepth() == 2);
    CHECK(a.tuning.value(0, Tunable::RangeMax) == 20.0);
    a.pos().moveTo(10.0, units::Time{8.0}, false);
    a.runStep();
    CHECK(a.tuning.undo().ok);
    CHECK(a.tuning.value(0, Tunable::RangeMax) == 12.0);
}

TEST_CASE("TA4 (review): a RESET while a step runs is refused with its own sentence, nothing changed") {
    TuneRig a{tune_fixture::kCascade};
    REQUIRE(a.tuning.apply(0, Tunable::KP, 0.3).ok);
    a.pos().moveTo(12.0, units::Time{8.0}, false);
    a.tick(5);
    REQUIRE(a.pos().running());
    const mech::Applied r = a.tuning.reset(0);
    CHECK_FALSE(r.ok);
    CHECK(r.why == LiveRefusal::StepRunning);
    CHECK(std::string{r.sentence} ==
          "'cascade': RESET refused -- a step is running on it: a change waits until the step ends");
    CHECK(a.tuning.value(0, Tunable::KP) == 0.3);
    CHECK(a.tuning.undoDepth() == 1);
}

TEST_CASE("rederive's precondition: a spec the build refuses, written under a bound slot, is a precondition -- never re-derived") {
    pos_rig::ArmRig r{tune_fixture::cascade.spec(), tune_rig::liftPlant()};
    r.spec.position.tolerance = 0.0;  // the slot runs THIS spec: the build would refuse it
    CHECK_THROWS_AS((void)r.svc.rederive(0), shulib::PreconditionError);
}

TEST_CASE("TA7 (review): already OUTSIDE its limits (carried onto the hard stop past one, let go) -- a change that leaves it no further outside is accepted; further, refused") {
    SwitchRig r;
    r.pos().release();           // it coasts
    r.arm->addLoadVolts(-12.0);  // a load carries it UP onto the plant's hard stop at 24.5 in, past the 24 in limit
    r.tick(400);
    r.arm->addLoadVolts(0.0);
    r.tick();
    REQUIRE_FALSE(r.pos().holding());
    REQUIRE(r.pos().position() > 24.25);
    CHECK(r.tuning.apply(0, Tunable::RangeMin, 1.0).ok);  // the far end: it is no further outside than it was
    CHECK(r.tuning.apply(0, Tunable::RangeMax, 23.75).why == LiveRefusal::LeavesOutWhereItIs);  // further outside
    r.pos().holdHere();  // its next hold: clamped to the limit in force (24 in), as before any change
    r.tick();
    CHECK(r.pos().target() == 24.0);
}

// ── TUNE1-A's verification round (2026-11-11): where it IS is a LIVE read ─────────────────────────
namespace {
/// A motor whose reading MOVES between two reads once armed: each position() read returns the reading,
/// then advances it `stepRad` — PROS updates a motor's data in the background, so live tuning's check
/// and the service's own can see two readings (a coasting or sagging lift, jitter at a limit).
struct DriftingMotor final : shulib::hal::IMotor {
    shulib::hal::IMotor* in;
    mutable double off = 0.0;
    bool armed = false;
    double stepRad = 0.0;
    explicit DriftingMotor(shulib::hal::IMotor* m) : in(m) {}
    void setVoltage(units::Voltage v) override { in->setVoltage(v); }
    units::Voltage commandedVoltage() const override { return in->commandedVoltage(); }
    void setBrakeMode(shulib::hal::BrakeMode b) override { in->setBrakeMode(b); }
    shulib::hal::BrakeMode brakeMode() const override { return in->brakeMode(); }
    units::AngleDim position() const override {
        const double v = in->position().value() + off;
        if (armed) {
            off += stepRad;
        }
        return units::AngleDim{v};
    }
    units::AngularVelocity velocity() const override { return in->velocity(); }
    units::Current current() const override { return in->current(); }
    double temperature() const override { return in->temperature(); }
};
/// The cascade moved to 20 in, held, then let go (holding nothing): where it is, in inches.
double letGoAt20(TuneRig& a) {
    a.pos().moveTo(20.0, units::Time{8.0}, false);
    a.runStep();
    REQUIRE(a.pos().lastStep().outcome == MechanismOutcome::Succeeded);
    a.tick(50);
    a.pos().release();
    a.tick();
    REQUIRE_FALSE(a.pos().holding());
    REQUIRE_FALSE(a.pos().heldButtonMoving());
    return a.pos().position();
}
}  // namespace

TEST_CASE("TA7 (verification round): where it is MOVES between live tuning's check and the service's -- the change is REFUSED, never thrown: the spec byte for byte, no record, no line, the slot's limits as they were") {
    TuneRig a{tune_fixture::kCascade};
    const double at = letGoAt20(a);
    DriftingMotor d{a.imotors[0][0]};
    a.imotors[0][0] = &d;  // the MotorMechanism reads through this array
    mech::MechanismSpec before;
    std::memcpy(static_cast<void*>(&before), &a.tuning.spec(0), sizeof before);
    const int retunedBefore = a.mechCount("retuned live");
    d.stepRad = 0.01;  // about 0.0075 in a read on the 1.5 in spool: the first read inside, the next past
    d.armed = true;
    mech::Applied r{};
    CHECK_NOTHROW(r = a.tuning.apply(0, Tunable::RangeMax, at + 0.003));
    d.armed = false;
    CHECK_FALSE(r.ok);
    CHECK(r.why == LiveRefusal::LeavesOutWhereItIs);
    const std::string s{r.sentence};
    CHECK(s.rfind("'cascade': upper limit 24 -> ", 0) == 0);
    CHECK(s.find(" refused -- it is at ") != std::string::npos);
    const std::string tail = " in, outside the new limits: move it inside first";
    REQUIRE(s.size() > tail.size());
    CHECK(s.substr(s.size() - tail.size()) == tail);
    CHECK(std::memcmp(&a.tuning.spec(0), &before, sizeof before) == 0);  // byte for byte
    CHECK_FALSE(a.tuning.changed(0));
    CHECK(a.tuning.undoDepth() == 0);
    CHECK(a.mechCount("retuned live") == retunedBefore);
    // NOT re-derived: its next hold begins where it is (past at + 0.003, inside the 24 in still in force) --
    // a slot given the new limit would clamp it to at + 0.003
    const double now = a.pos().position();
    REQUIRE(now > at + 0.003);
    a.pos().holdHere();
    a.tick();
    CHECK(a.pos().target() > at + 0.003);
    a.imotors[0][0] = d.in;
}

TEST_CASE("TA7 (verification round): an UNDO the moving reading makes the service refuse is refused with undo's own sentence -- the record kept, undone later") {
    TuneRig a{tune_fixture::kCascade};
    const double at = letGoAt20(a);
    REQUIRE(a.tuning.apply(0, Tunable::RangeMax, at + 0.003).ok);  // inside: no drift yet
    REQUIRE(a.tuning.apply(0, Tunable::RangeMax, 23.0).ok);        // widened back, inside the file's 24
    DriftingMotor d{a.imotors[0][0]};
    a.imotors[0][0] = &d;
    const int retunedBefore = a.mechCount("retuned live");
    d.stepRad = 0.01;
    d.armed = true;
    mech::Applied u{};
    CHECK_NOTHROW(u = a.tuning.undo());
    d.armed = false;
    CHECK_FALSE(u.ok);
    CHECK(u.why == LiveRefusal::LeavesOutWhereItIs);
    CHECK(std::string{u.sentence} ==
          std::string{"'cascade': undo refused -- "} + mech::liveRefusalText(LiveRefusal::LeavesOutWhereItIs));
    CHECK(a.tuning.undoDepth() == 2);
    CHECK(a.tuning.value(0, Tunable::RangeMax) == 23.0);
    CHECK(a.mechCount("retuned live") == retunedBefore);
    a.imotors[0][0] = d.in;  // back where it was: the same undo is taken
    const mech::Applied again = a.tuning.undo();
    CHECK(again.ok);
    CHECK(a.tuning.value(0, Tunable::RangeMax) == at + 0.003);
    CHECK(a.tuning.undoDepth() == 1);
}

TEST_CASE("MechanismService::rederive (verification round): a spec written past the hold by its own caller is REFUSED -- false, nothing said, nothing re-derived (its twin's next command, to the bit), never a throw") {
    pos_rig::ArmRig a{tune_fixture::cascade.spec(), tune_rig::liftPlant()};
    pos_rig::ArmRig b{tune_fixture::cascade.spec(), tune_rig::liftPlant()};
    a.slot().moveTo(20.0, units::Time{8.0}, false);
    b.slot().moveTo(20.0, units::Time{8.0}, false);
    a.runStep();
    b.runStep();
    a.tick(40);
    b.tick(40);
    REQUIRE(a.slot().holding());
    REQUIRE(a.volts() == b.volts());
    a.spec.position.maxPosition = 12.0;  // past the 20 in it holds
    a.spec.position.kP *= 4.0;           // and a gain the slot copies
    bool ok = true;
    CHECK_NOTHROW(ok = a.svc.rederive(0, "x"));
    CHECK_FALSE(ok);
    CHECK(a.mechCount("retuned live") == 0);
    a.tick();
    b.tick();
    CHECK(a.volts() == b.volts());  // the hold's kP not re-derived; the hold not moved
    CHECK(a.slot().target() == 20.0);
    a.spec = tune_fixture::cascade.spec();  // the caller puts the old spec back (the doc says so)
    CHECK(a.svc.rederive(0, "y"));
}
