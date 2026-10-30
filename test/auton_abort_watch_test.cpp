// auton_abort_watch_test.cpp — the Auton Tester's ABORT (src/programs/auton_abort_watch.hpp; chunk
// COMP-t, 2026-09-28; the brief's §5 tests (6)–(13b)). Every case names the bug it would catch.
//
// The watch is the one thing between an operator's hand and a moving robot on the bench: any
// controller button, a touch of the screen, the controller dropping or the robot being disabled
// must end the countdown or the run. So the decision (abortCauseOf), the PROS reductions
// (testerTouchFrom / abortInputsFrom — the ONLY place PROS's reads are interpreted; the program's
// two reads are one line each, pinned by test/auton_tester_source_test.cpp), the latch, the
// baseline and the every-tick request are each tried here against the input that would break them.

#include "doctest.h"

#include <cstdint>
#include <vector>

#include "auton_tester_rig.hpp"
#include "motion_test_rig.hpp"
#include "programs/auton_abort_watch.hpp"
#include "programs/auton_tester_run.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/fake/fake_controller.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/units/quantity.hpp"

using shulib::bench::abortCauseOf;
using shulib::bench::abortCauseShort;
using shulib::bench::abortCauseWord;
using shulib::bench::abortInputsFrom;
using shulib::bench::AbortCause;
using shulib::bench::AbortInputs;
using shulib::bench::AbortWatch;
using shulib::bench::buttonsDownOf;
using shulib::bench::pacerChainProven;
using shulib::bench::TesterTouch;
using shulib::bench::testerTouchFrom;
using shulib::hal::ControllerButton;
using shulib::hal::fake::FakeController;
using shulib::kinematics::TankKinematics;
using shulib::sequence::RunGuard;
using shulib::sequence::RunGuardConfig;
using shulib::units::Length;
using shulib::units::Time;
using tester_rig::FakeProsTouch;
using tester_rig::kTouchError;

namespace {

/// An input script that counts its reads and stamps them into a shared sequence.
struct CountingInputs final : shulib::bench::IAbortInputs {
    AbortInputs next{};
    int reads = 0;
    std::vector<char>* order = nullptr;
    AbortInputs read() override {
        ++reads;
        if (order != nullptr) {
            order->push_back('r');
        }
        return next;
    }
};

/// An inner pacer that counts and stamps.
struct CountingPacer final : shulib::motion::ITickPacer {
    int paces = 0;
    std::vector<char>* order = nullptr;
    void pace() override {
        ++paces;
        if (order != nullptr) {
            order->push_back('p');
        }
    }
};

AbortInputs quiet() {
    AbortInputs in;
    in.controllerConnected = true;
    in.pressCount = 7;
    return in;
}

}  // namespace

// (6) Bug caught: a cause dropped from the decision (the controller dropping would no longer
// abort — mutation 2), a precedence flipped (a disabled robot reported as a button), an error
// read counted as a touch (R2a's class), or a touch compared with anything but "not equal to the
// baseline" (mutation 3a: `>` baseline + 1 misses a single press).
TEST_CASE("abort (6): abortCauseOf — every cause alone, every precedence pair, the touch rule") {
    const std::int32_t base = 7;
    CHECK(abortCauseOf(quiet(), base) == AbortCause::None);

    AbortInputs d = quiet();
    d.disabled = true;
    CHECK(abortCauseOf(d, base) == AbortCause::Disabled);
    AbortInputs c = quiet();
    c.controllerConnected = false;
    CHECK(abortCauseOf(c, base) == AbortCause::ControllerLost);
    AbortInputs b = quiet();
    b.buttonsDown = 1u << static_cast<int>(ControllerButton::B);
    CHECK(abortCauseOf(b, base) == AbortCause::Button);
    AbortInputs t = quiet();
    t.pressCount = base + 1;
    CHECK(abortCauseOf(t, base) == AbortCause::Touch);

    // Precedence: Disabled > ControllerLost > Button > Touch, every pair.
    AbortInputs all = quiet();
    all.disabled = true;
    all.controllerConnected = false;
    all.buttonsDown = 0x0FFF;
    all.pressCount = base + 3;
    CHECK(abortCauseOf(all, base) == AbortCause::Disabled);
    all.disabled = false;
    CHECK(abortCauseOf(all, base) == AbortCause::ControllerLost);
    all.controllerConnected = true;
    CHECK(abortCauseOf(all, base) == AbortCause::Button);
    all.buttonsDown = 0;
    CHECK(abortCauseOf(all, base) == AbortCause::Touch);
    AbortInputs dt = quiet();  // Disabled beats Touch, Button; ControllerLost beats Touch
    dt.disabled = true;
    dt.pressCount = base + 1;
    CHECK(abortCauseOf(dt, base) == AbortCause::Disabled);
    AbortInputs db = quiet();
    db.disabled = true;
    db.buttonsDown = 1;
    CHECK(abortCauseOf(db, base) == AbortCause::Disabled);
    AbortInputs cb = quiet();
    cb.controllerConnected = false;
    cb.buttonsDown = 1;
    CHECK(abortCauseOf(cb, base) == AbortCause::ControllerLost);
    AbortInputs ct = quiet();
    ct.controllerConnected = false;
    ct.pressCount = base + 1;
    CHECK(abortCauseOf(ct, base) == AbortCause::ControllerLost);
    AbortInputs bt = quiet();
    bt.buttonsDown = 1u << 11;
    bt.pressCount = base + 1;
    CHECK(abortCauseOf(bt, base) == AbortCause::Button);

    // The touch rule: an error read never counts, whatever its count; equal = None; ANY other
    // count (one more, many more, fewer, the error value -1) = Touch.
    AbortInputs e = quiet();
    e.touchError = true;
    e.pressCount = -1;
    CHECK(abortCauseOf(e, base) == AbortCause::None);
    e.pressCount = base + 5;
    CHECK(abortCauseOf(e, base) == AbortCause::None);
    for (const std::int32_t n : {base + 1, base + 2, base + 100, base - 1, -1, 0}) {
        CAPTURE(n);
        AbortInputs k = quiet();
        k.pressCount = n;
        CHECK(abortCauseOf(k, base) == AbortCause::Touch);
    }

    // The words fit their budgets (the log's and the screen's), and every cause has its own.
    for (const AbortCause cause : {AbortCause::None, AbortCause::Button, AbortCause::Touch,
                                   AbortCause::ControllerLost, AbortCause::Disabled}) {
        CHECK(std::string_view{abortCauseWord(cause)}.size() <= 27);
        CHECK(std::string_view{abortCauseShort(cause)}.size() <= 15);
    }
}

// (6b) Bug caught (the re-check's finding 4): the PROS reductions — the only place the program's
// reads are interpreted — reading the RELEASE count as the press count (mutation 3b: a touch would
// abort on the lift, or never), dropping the error mapping (mutation 22: an error read's -1 counts
// as a touch), or hard-coding the controller as connected (mutation 21: the controller dropping
// would never abort). Built on a struct with PROS's own field names, and press_count differs from
// release_count in EVERY case so a swap is visible.
TEST_CASE("abort (6b): testerTouchFrom and abortInputsFrom read the right PROS fields") {
    FakeProsTouch t;
    t.touch_status = tester_rig::kTouchHeld;
    t.x = 123;
    t.y = 45;
    t.press_count = 5;
    t.release_count = 4;
    const TesterTouch r = testerTouchFrom(t, static_cast<int>(kTouchError));
    CHECK(r.pressCount == 5);
    CHECK(r.sample.releaseCount == 4);
    CHECK(r.sample.x == 123);
    CHECK(r.sample.y == 45);
    CHECK_FALSE(r.sample.error);
    CHECK(r.fingerDown);  // a good read, press above release: a finger on the glass

    t.touch_status = tester_rig::kTouchReleased;
    t.press_count = 9;
    t.release_count = 9;
    const TesterTouch lifted = testerTouchFrom(t, static_cast<int>(kTouchError));
    CHECK_FALSE(lifted.fingerDown);
    CHECK(lifted.pressCount == 9);

    t.press_count = 3;  // (never physical, but a swap must still be visible)
    t.release_count = 8;
    const TesterTouch odd = testerTouchFrom(t, static_cast<int>(kTouchError));
    CHECK(odd.pressCount == 3);
    CHECK(odd.sample.releaseCount == 8);
    CHECK_FALSE(odd.fingerDown);

    // An error read: PROS puts -1 in every field. It is an error, never a finger.
    FakeProsTouch e;
    e.touch_status = kTouchError;
    e.x = -1;
    e.y = -1;
    e.press_count = -1;
    e.release_count = -2;  // not equal, so "press > release" would say a finger — it must not
    const TesterTouch er = testerTouchFrom(e, static_cast<int>(kTouchError));
    CHECK(er.sample.error);
    CHECK_FALSE(er.fingerDown);
    // Every other status is a good read.
    for (const auto st : {tester_rig::kTouchReleased, tester_rig::kTouchPressed,
                          tester_rig::kTouchHeld}) {
        FakeProsTouch g;
        g.touch_status = st;
        g.press_count = 2;
        g.release_count = 1;
        CHECK_FALSE(testerTouchFrom(g, static_cast<int>(kTouchError)).sample.error);
    }

    // abortInputsFrom: the connection and the buttons from the IController, the touch's error and
    // press count from the touch, disabled as given.
    FakeController pad;
    pad.setConnected(false);
    AbortInputs in = abortInputsFrom(r, pad, true);
    CHECK_FALSE(in.controllerConnected);
    CHECK(in.disabled);
    CHECK(in.pressCount == 5);
    CHECK_FALSE(in.touchError);
    CHECK(in.buttonsDown == 0);
    pad.setConnected(true);
    pad.setPressed(ControllerButton::B, true);
    in = abortInputsFrom(er, pad, false);
    CHECK(in.controllerConnected);
    CHECK_FALSE(in.disabled);
    CHECK(in.touchError);
    CHECK(in.buttonsDown == (1u << 9));  // B is bit 9
    CHECK(abortCauseOf(abortInputsFrom(er, pad, false), -1) == AbortCause::Button);
    pad.setPressed(ControllerButton::B, false);
    CHECK(abortCauseOf(abortInputsFrom(er, pad, false), 5) == AbortCause::None);  // error: no touch
}

// (7) Bug caught: a button whose bit is wrong or missing (it would not abort, or the log would name
// the wrong button).
TEST_CASE("abort (7): buttonsDownOf — each of the twelve buttons sets its own bit") {
    FakeController pad;
    pad.setConnected(true);
    CHECK(buttonsDownOf(pad) == 0);
    for (int b = 0; b <= static_cast<int>(ControllerButton::A); ++b) {
        CAPTURE(b);
        FakeController one;
        one.setPressed(static_cast<ControllerButton>(b), true);
        CHECK(buttonsDownOf(one) == static_cast<std::uint16_t>(1u << b));
        pad.setPressed(static_cast<ControllerButton>(b), true);
    }
    CHECK(buttonsDownOf(pad) == 0x0FFF);
    char names[40] = "";
    shulib::bench::buttonNamesOf(0x0FFF, names, sizeof names);
    CHECK(std::string_view{names} == "L1+L2+R1+R2+UP+DOWN+LEFT+RIGHT+X+B+Y+A");
    shulib::bench::buttonNamesOf(1u << 9, names, sizeof names);
    CHECK(std::string_view{names} == "B");
    shulib::bench::buttonNamesOf((1u << 0) | (1u << 3), names, sizeof names);
    CHECK(std::string_view{names} == "L1+R2");
    char tiny[6] = "";
    shulib::bench::buttonNamesOf(0x0FFF, tiny, sizeof tiny);  // never a half name
    CHECK(std::string_view{tiny} == "L1+L2");
}

// (8) Bug caught: a DISARMED watch that still reads (or aborts, or asks the guard anything) — it
// would abort a run that is not its own, or a READY page's touches would latch; and a pace counter
// that counts only while armed (the boot's chain proof would read no pace — R13).
TEST_CASE("abort (8): disarmed, the watch only paces — and paces() counts every pace") {
    CountingPacer inner;
    CountingInputs inputs;
    inputs.next = quiet();
    inputs.next.buttonsDown = 0x0FFF;  // everything down: a disarmed watch must still do nothing
    inputs.next.disabled = true;
    AbortWatch watch{inner, inputs};
    for (int i = 0; i < 5; ++i) {
        watch.pace();
    }
    CHECK(inputs.reads == 0);
    CHECK(inner.paces == 5);
    CHECK(watch.cause() == AbortCause::None);
    CHECK_FALSE(watch.armed());
    CHECK(watch.paces() == 5);
    inputs.next = quiet();
    watch.arm();
    CHECK(watch.armed());
    watch.pace();
    watch.pace();
    CHECK(watch.paces() == 7);  // armed paces count too
    CHECK(inner.paces == 7);
    watch.disarm();
    watch.pace();
    CHECK(watch.paces() == 8);
}

// (9) Bug caught: arm() keeping a stale baseline (mutation 4: a touch made on READY — the RUN tap
// itself — would abort the countdown at once) or keeping an old latch (mutation 6: the second run
// after an abort would abort at T+0).
TEST_CASE("abort (9): arm() takes the press count NOW as the baseline, and clears the latch") {
    CountingPacer inner;
    CountingInputs inputs;
    inputs.next = quiet();
    inputs.next.pressCount = 40;  // forty taps made before RUN — the RUN tap among them
    AbortWatch watch{inner, inputs};
    watch.arm();
    for (int i = 0; i < 3; ++i) {
        watch.pace();
    }
    CHECK(watch.cause() == AbortCause::None);
    inputs.next.pressCount = 41;
    watch.pace();
    CHECK(watch.cause() == AbortCause::Touch);
    watch.disarm();
    CHECK(watch.cause() == AbortCause::Touch);  // readable after disarm, until the next arm
    // Re-arm with the touch still counted 41: a fresh baseline, a cleared latch.
    watch.arm();
    CHECK(watch.cause() == AbortCause::None);
    CHECK(watch.buttonsAtLatch() == 0);
    watch.pace();
    CHECK(watch.cause() == AbortCause::None);
}

// (9b) Bug caught (COMP-t's review, 2026-09-29): a touch that lands AFTER the read whose release
// touched RUN but BEFORE the watch arms — the RUN line's SD write sits in that gap — counted into a
// baseline read at arm(), so it never aborts, even held through the whole countdown. arm(baseline)
// takes RUN's own read: the count at arm() already one higher is an abort at the first pace; a
// baseline equal to the count is not; and it clears an old latch like arm().
TEST_CASE("abort (9b): arm(baseline) — a finger that landed after RUN's read aborts at once") {
    CountingPacer inner;
    CountingInputs inputs;
    inputs.next = quiet();
    inputs.next.pressCount = 41;  // RUN's release was read at 40; a finger landed since
    AbortWatch watch{inner, inputs};
    watch.arm(40);
    CHECK(inputs.reads == 0);  // no read of its own: the baseline is the caller's
    watch.pace();
    CHECK(watch.cause() == AbortCause::Touch);
    // The same count as RUN's read: nothing landed since — no abort; a later press is one.
    watch.arm(41);
    CHECK(watch.cause() == AbortCause::None);  // the old latch is cleared
    CHECK(watch.buttonsAtLatch() == 0);
    watch.pace();
    watch.pace();
    CHECK(watch.cause() == AbortCause::None);
    inputs.next.pressCount = 42;
    watch.pace();
    CHECK(watch.cause() == AbortCause::Touch);
    // An error read after arm(baseline) says nothing; the baseline is kept for the next good one.
    watch.arm(42);
    inputs.next.touchError = true;
    inputs.next.pressCount = -1;
    watch.pace();
    CHECK(watch.cause() == AbortCause::None);
    inputs.next.touchError = false;
    inputs.next.pressCount = 42;
    watch.pace();
    CHECK(watch.cause() == AbortCause::None);
}

// (10) Bug caught (found while probing, §0.5.2): a baseline taken from an ERROR read (-1) at arm():
// the next GOOD read's real count differs from -1 and reads as a touch — R2a's "error read taken as
// a tap" class (mutation 5). The first good read supplies the baseline and aborts nothing; a real
// press after it does.
TEST_CASE("abort (10): an error read at arm, then a good read, aborts nothing; a press after does") {
    CountingPacer inner;
    CountingInputs inputs;
    inputs.next = quiet();
    inputs.next.touchError = true;
    inputs.next.pressCount = -1;
    AbortWatch watch{inner, inputs};
    watch.arm();
    watch.pace();  // still erroring: nothing
    CHECK(watch.cause() == AbortCause::None);
    inputs.next.touchError = false;
    inputs.next.pressCount = 12;  // the first good read: THIS is the baseline
    watch.pace();
    CHECK(watch.cause() == AbortCause::None);
    inputs.next.touchError = true;  // another error read in between: still nothing
    inputs.next.pressCount = -1;
    watch.pace();
    CHECK(watch.cause() == AbortCause::None);
    inputs.next.touchError = false;
    inputs.next.pressCount = 12;
    watch.pace();
    CHECK(watch.cause() == AbortCause::None);
    inputs.next.pressCount = 13;  // a real press
    watch.pace();
    CHECK(watch.cause() == AbortCause::Touch);
}

// (11) Bug caught: a latch that forgets (the input clears and the run resumes) or that is
// OVERWRITTEN by a second cause (the log would blame the wrong thing); buttonsAtLatch taken at the
// wrong read.
TEST_CASE("abort (11): the latch holds the FIRST cause and its buttons") {
    CountingPacer inner;
    CountingInputs inputs;
    inputs.next = quiet();
    AbortWatch watch{inner, inputs};
    watch.arm();
    inputs.next.buttonsDown = (1u << 9) | (1u << 0);  // B + L1
    watch.pace();
    CHECK(watch.cause() == AbortCause::Button);
    CHECK(watch.buttonsAtLatch() == ((1u << 9) | (1u << 0)));
    const int readsAtLatch = inputs.reads;
    inputs.next.buttonsDown = 0;  // released
    watch.pace();
    CHECK(watch.cause() == AbortCause::Button);
    inputs.next.disabled = true;  // a second, "more serious" cause
    inputs.next.controllerConnected = false;
    inputs.next.buttonsDown = 1u << 11;
    watch.pace();
    CHECK(watch.cause() == AbortCause::Button);
    CHECK(watch.buttonsAtLatch() == ((1u << 9) | (1u << 0)));
    CHECK(inputs.reads == readsAtLatch);  // latched: it stops reading
}

// (12) Bug caught: the request made only on the latch EDGE (mutation 1). The guard DROPS a request
// made while no run is live (run_guard.hpp's ABANDON banner) — so an abort latched in the
// countdown, before the guard armed, must still be requested once the run is live. Shown both
// ways: a direct request made while idle IS dropped at the arm (the reason for the rule), and the
// watch's latch from before the arm still abandons the run at its first pace.
TEST_CASE("abort (12): latched while idle -> no request; a run live -> requested, run abandoned") {
    const TankKinematics kin{Length{12.0}};
    tester_rig::TesterStack s{kin};
    // The guard's own rule: an idle request is dropped when the next run arms.
    s.guard.requestAbandon();
    const auto clean = s.guard.run(
        s.chassis, RunGuardConfig{.endActionAt = Time{0.2}, .hardStopAt = Time{0.4}},
        [&] { s.chassis.wait(Time{0.05}); }, [] { return true; });
    CHECK_FALSE(clean.abandoned);

    // Latch while idle.
    s.watch.arm();
    s.inputs.pad.setPressed(ControllerButton::B, true);
    s.chassis.wait(Time{0.01});
    CHECK(s.watch.cause() == AbortCause::Button);
    CHECK_FALSE(s.guard.abandonRequested());  // idle: nothing to abandon
    s.inputs.pad.setPressed(ControllerButton::B, false);

    bool requestedAtFirstPace = false;
    int pacesBeforeRequest = -1;
    const auto rep = s.guard.run(
        s.chassis, RunGuardConfig{.endActionAt = Time{2.0}, .hardStopAt = Time{3.0}},
        [&] {
            for (int i = 0; i < 20; ++i) {
                s.chassis.wait(Time{0.01});
                if (s.guard.abandonRequested() && pacesBeforeRequest < 0) {
                    pacesBeforeRequest = i;
                }
            }
            requestedAtFirstPace = pacesBeforeRequest == 0;
        },
        [] { return true; });
    CHECK(requestedAtFirstPace);
    CHECK(rep.abandoned);
    CHECK_FALSE(rep.endActionRan);  // an abandoned run skips its end action
    s.watch.disarm();
}

// (13) Bug caught: the watch advancing the world BEFORE it reads (an abort read one tick late), or
// pacing the inner pacer twice / not at all (the robot clock drifts, or the world stops).
TEST_CASE("abort (13): each pace reads the inputs, THEN paces the inner pacer, exactly once") {
    std::vector<char> order;
    CountingPacer inner;
    inner.order = &order;
    CountingInputs inputs;
    inputs.order = &order;
    inputs.next = quiet();
    AbortWatch watch{inner, inputs};
    watch.arm();
    order.clear();  // arm()'s own read
    for (int i = 0; i < 4; ++i) {
        watch.pace();
    }
    CHECK(order == std::vector<char>{'r', 'p', 'r', 'p', 'r', 'p', 'r', 'p'});
    CHECK(inner.paces == 4);
}

// (13b) Bug caught (the re-check's finding 4; R13): a Chassis whose pacer is not the guard-over-
// the-watch chain. Built WITHOUT `opts` it paces neither (the plant alone — mutation 24's shape);
// built with `.pacer = &watch` it paces the watch only (the abandon would be requested and never
// acted on — mutation 25). And (COMP-t's review, 2026-09-29) the chain paced right but the watch
// NEVER ATTACHED to the guard, or attached to another one: it paces and latches, its abandon goes
// nowhere, and the run drives on through the button (measured: 14.22 in after B, a green FINISHED).
// Since MECH-A5 the chain has the robot's mechanism service between the guard and the watch (the
// tester rig's is the program's); test/guarded_run_test.cpp (G1) proves a chain WITHOUT it fails.
// The boot's chain proof must say true for the right chain and false for every wrong one — and
// none of the proofs commands the drive.
TEST_CASE("abort (13b): pacerChainProven — the right chain only, and it commands nothing") {
    const TankKinematics kin{Length{12.0}};
    {
        tester_rig::TesterStack s{kin, 1, 1.0, motion_rig::plantConfig(), nullptr, 0};
        const double before = s.plant.distance;
        CHECK(pacerChainProven(s.chassis, s.guard, s.mechanisms, s.watch));
        CHECK(s.plant.distance - before == 0.0);
        CHECK(s.probe.energizedWhileIdle == 0);
    }
    motion_rig::MotionRig rig{kin};
    motion_rig::PlantPacer plant{rig.h};
    CountingInputs inputs;
    inputs.next = quiet();
    AbortWatch watch{plant, inputs};
    shulib::mechanism::MechanismService svc{watch};
    svc.bind(shulib::bench::kNoMechanisms, {}, rig.h.clock(), rig.latch, rig.faultSink);
    RunGuard guard{svc};
    watch.attachGuard(guard);
    shulib::chassis::Chassis onWatch{rig.deps, watch, motion_rig::chassisConfig()};
    CHECK_FALSE(pacerChainProven(onWatch, guard, svc, watch));
    shulib::chassis::Chassis onPlant{rig.deps, plant, motion_rig::chassisConfig()};
    CHECK_FALSE(pacerChainProven(onPlant, guard, svc, watch));
    CHECK(plant.distance == 0.0);
    for (const shulib::hal::IMotor* m : rig.h.context().driveMotors()) {
        CHECK(m->commandedVoltage().value() == 0.0);
    }
    // The guard's own count is every pace, a run live or not (it is what the proof reads).
    const std::uint64_t g0 = guard.pacesTotal();
    guard.pace();
    guard.pace();
    CHECK(guard.pacesTotal() == g0 + 2);

    // The right pacing, the back link missing: never attached, then attached to another guard.
    motion_rig::MotionRig rig2{kin};
    motion_rig::PlantPacer plant2{rig2.h};
    CountingInputs inputs2;
    inputs2.next = quiet();
    AbortWatch unattached{plant2, inputs2};
    shulib::mechanism::MechanismService svc2{unattached};
    svc2.bind(shulib::bench::kNoMechanisms, {}, rig2.h.clock(), rig2.latch, rig2.faultSink);
    RunGuard guard2{svc2};
    shulib::chassis::Chassis onChain{rig2.deps, guard2, motion_rig::chassisConfig()};
    CHECK_FALSE(unattached.attachedTo(guard2));
    CHECK_FALSE(pacerChainProven(onChain, guard2, svc2, unattached));
    unattached.attachGuard(guard);  // the OTHER guard
    CHECK_FALSE(pacerChainProven(onChain, guard2, svc2, unattached));
    unattached.attachGuard(guard2);
    CHECK(unattached.attachedTo(guard2));
    CHECK(pacerChainProven(onChain, guard2, svc2, unattached));  // the same chain, now linked back
    CHECK(plant2.distance == 0.0);
}
