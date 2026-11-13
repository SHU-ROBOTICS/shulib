// teleop::MechanismBinding and the runDriverControl bindings overload (chunk T3, brief §2 items
// 1-2, tests 2-12 at the library level).
//
// ═══ WHAT THIS FILE PINS ════════════════════════════════════════════════════════════
// Every rule mechanism_binding.hpp's banner states, asserted at the BOTTOM of the stack — the
// ordered command events a RecordingMotor saw — never against the binding's own idea of what it
// did. Two halves:
//
//   * the BINDING alone (T3 B-cases): hold-to-run, safe ONCE on release (then silence), both held
//     is a hold (R2), the outside-stop latch and what re-arms it, a claimed mechanism untouched,
//     the construction refusals, and validBindings()'s set rules;
//   * the binding inside THE LOOP (T3 L-cases), on the host test plant: a press read at tick k is
//     commanded in tick k, two pivots in one tick, a lying disconnected controller, the disable
//     path (cancel first, then every pivot safe, inside the disable tick), entry safe before tick
//     1, a claim already on a mechanism at entry left alone (and the gap that leaves), the loop's
//     own refusal before tick 1, and the drive's volts BIT-IDENTICAL with and without bindings.
//
// The claws here are named after robot two's (ports 6 and 7, R1/R2 and L1/L2, 12 V, Hold) only
// because that is the case that motivated the chunk; nothing below reads a robot table (that is
// the program's, in src/, and its wiring test belongs to the program half of the chunk).
//
// NO HARDWARE CLAIM. RecordingMotors and the host test plant; no pivot has moved.

#include "doctest.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include "mechanism_test_rig.hpp"
#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/teleop/driver_control.hpp"
#include "shulib/teleop/mechanism_binding.hpp"
#include "shulib/teleop/teleop_loop.hpp"
#include "shulib/units/quantity.hpp"

using mech_rig::RecordingMotor;
using shulib::PreconditionError;
using shulib::hal::BrakeMode;
using shulib::hal::ControllerAxis;
using shulib::hal::ControllerButton;
using shulib::hal::IMotor;
using shulib::hal::MotorMechanism;
using shulib::math::ChassisSpeeds;
using shulib::teleop::DriverLoopConfig;
using shulib::teleop::DriverProfile;
using shulib::teleop::DriveShaper;
using shulib::teleop::DriveShaperState;
using shulib::teleop::HeadingSample;
using shulib::teleop::MechanismBinding;
using shulib::teleop::runDriverControl;
using shulib::teleop::validBindings;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Voltage;
using Event = RecordingMotor::Event;
using State = MechanismBinding::State;

namespace {

constexpr Voltage kFull{12.0};

/// One claw pivot: a RecordingMotor behind a single-motor MotorMechanism whose declared safe
/// state is Hold. Non-movable (the mechanism is), so it is constructed where it lives.
struct Pivot {
    explicit Pivot(const char* name, BrakeMode safe = BrakeMode::Hold)
        : mech{std::span<IMotor* const>{motors}, safe, name} {}
    RecordingMotor motor;
    std::array<IMotor*, 1> motors{&motor};
    MotorMechanism mech;
};

/// True when events [from, from+2) are exactly the declared safe state: Brake(mode) THEN 0 V.
bool isSafeAt(const std::vector<Event>& ev, std::size_t from, BrakeMode mode = BrakeMode::Hold) {
    return ev.size() >= from + 2 && ev[from].kind == Event::Kind::Brake && ev[from].mode == mode
           && ev[from + 1].kind == Event::Kind::Voltage && ev[from + 1].volts == 0.0;
}

/// True when event `i` is a voltage command of exactly `volts`.
bool isVoltage(const std::vector<Event>& ev, std::size_t i, double volts) {
    return i < ev.size() && ev[i].kind == Event::Kind::Voltage && ev[i].volts == volts;
}

/// How many Brake events the list holds.
int brakeEvents(const std::vector<Event>& ev) {
    int n = 0;
    for (const Event& e : ev) {
        n += e.kind == Event::Kind::Brake ? 1 : 0;
    }
    return n;
}

[[nodiscard]] bool sameBits(double a, double b) {
    return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

}  // namespace

// ═══════════════════════════════ the BINDING alone ═══════════════════════════════════

// ═══ B1. Hold-to-run: re-commanded EVERY tick, at exactly ±V ════════════════════════
TEST_CASE("T3 B1: forward alone commands +V EVERY tick, reverse alone -V every tick — the "
          "stated voltage, exactly") {
    // BUG CAUGHT: commanding only on the press edge (a motor that lost a command never gets it
    // back while the button stays down); a wrong sign on reverse; a voltage other than the stated.
    Pivot p{"claw pivot 6"};
    MechanismBinding b{p.mech, ControllerButton::R1, ControllerButton::R2, kFull};
    for (int k = 0; k < 7; ++k) {
        b.tick(true, true, false);
    }
    REQUIRE(p.motor.eventCount() == 7);
    for (std::size_t i = 0; i < 7; ++i) {
        CAPTURE(i);
        CHECK(isVoltage(p.motor.events(), i, 12.0));
    }
    CHECK(b.state() == State::Forward);
    // Straight from forward to reverse in one tick (the driver rolls one finger to the other):
    // level semantics — no stop in between, the other direction at once.
    b.tick(true, false, true);
    b.tick(true, false, true);
    REQUIRE(p.motor.eventCount() == 9);
    CHECK(isVoltage(p.motor.events(), 7, -12.0));
    CHECK(isVoltage(p.motor.events(), 8, -12.0));
    CHECK(b.state() == State::Reverse);
    // A binding at a lower stated voltage commands THAT voltage.
    Pivot q{"wrist"};
    MechanismBinding half{q.mech, ControllerButton::Up, ControllerButton::Down, Voltage{6.5}};
    half.tick(true, false, true);
    CHECK(isVoltage(q.motor.events(), 0, -6.5));
}

// ═══ B2 (brief test 2). Release: safe exactly ONCE, then silence ═════════════════════
TEST_CASE("T3 B2: release ⇒ exactly [Brake Hold, 0 V] once, then NO event over 50 released "
          "ticks — from forward and from reverse") {
    // BUG CAUGHT: re-safing every tick (starves a later operation and may re-capture the hold
    // point — the banner's reason); a stop that sets 0 V before the mode (a momentary coast); a
    // stop that forgets the mode entirely; no stop at all (the motor keeps its last +12 V).
    for (const bool forward : {true, false}) {
        CAPTURE(forward);
        Pivot p{"claw pivot 7"};
        MechanismBinding b{p.mech, ControllerButton::L1, ControllerButton::L2, kFull};
        for (int k = 0; k < 4; ++k) {
            b.tick(true, forward, !forward);
        }
        REQUIRE(p.motor.eventCount() == 4);
        b.tick(true, false, false);  // the release
        REQUIRE(p.motor.eventCount() == 6);
        CHECK(isSafeAt(p.motor.events(), 4));
        CHECK(p.motor.brakeMode() == BrakeMode::Hold);
        CHECK(p.motor.commandedVoltage().value() == 0.0);
        CHECK(b.state() == State::Stopped);
        for (int k = 0; k < 50; ++k) {
            b.tick(true, false, false);
        }
        CHECK(p.motor.eventCount() == 6);  // silence
        // The declared mode is the MECHANISM's, not a constant: a Brake-safe mechanism gets Brake.
        Pivot c{"intake", BrakeMode::Brake};
        MechanismBinding bc{c.mech, ControllerButton::X, ControllerButton::B, kFull};
        bc.tick(true, true, false);
        bc.tick(true, false, false);
        CHECK(isSafeAt(c.motor.events(), 1, BrakeMode::Brake));
    }
}

// ═══ B3 (brief test 3, ruling R2). Both held is a HOLD ══════════════════════════════
TEST_CASE("T3 B3: both held ⇒ safe once and nothing energised; releasing ONE lets the other "
          "drive") {
    // BUG CAUGHT: "forward wins" (or reverse wins) on a tie — the ruling is that neither does;
    // a both-held state that re-safes every tick; a both-held state that sticks after one release.
    SUBCASE("both pressed from rest") {
        Pivot p{"claw pivot 6"};
        MechanismBinding b{p.mech, ControllerButton::R1, ControllerButton::R2, kFull};
        for (int k = 0; k < 20; ++k) {
            b.tick(true, true, true);
        }
        REQUIRE(p.motor.eventCount() == 2);  // the Idle binding's one safe state, no more
        CHECK(isSafeAt(p.motor.events(), 0));
        CHECK_FALSE(p.motor.everEnergized());
        b.tick(true, false, true);  // forward let go: reverse drives, in this tick
        CHECK(isVoltage(p.motor.events(), 2, -12.0));
    }
    SUBCASE("the second button joins while the first drives") {
        Pivot p{"claw pivot 6"};
        MechanismBinding b{p.mech, ControllerButton::R1, ControllerButton::R2, kFull};
        b.tick(true, true, false);
        b.tick(true, true, false);
        b.tick(true, true, true);  // R2 joins
        REQUIRE(p.motor.eventCount() == 4);
        CHECK(isSafeAt(p.motor.events(), 2));
        for (int k = 0; k < 20; ++k) {
            b.tick(true, true, true);
        }
        CHECK(p.motor.eventCount() == 4);
        CHECK(b.state() == State::Stopped);
        b.tick(true, true, false);  // R2 let go: forward drives again at once
        CHECK(isVoltage(p.motor.events(), 4, 12.0));
    }
}

// ═══ B4. Idle: a fresh binding makes the mechanism safe the first time it is not driving ═══
TEST_CASE("T3 B4: a fresh (or reset) binding applies the safe state on its first non-driving "
          "tick, once — and a button held on its first tick drives at once") {
    // BUG CAUGHT: starting in 'Stopped' — a binding ticked without makeSafe() would then leave
    // the mechanism in whatever state it was found (a coasting arm) until the first press.
    Pivot p{"claw pivot 6"};
    MechanismBinding b{p.mech, ControllerButton::R1, ControllerButton::R2, kFull};
    CHECK(b.state() == State::Idle);
    b.tick(true, false, false);
    b.tick(true, false, false);
    REQUIRE(p.motor.eventCount() == 2);
    CHECK(isSafeAt(p.motor.events(), 0));

    Pivot q{"claw pivot 7"};
    MechanismBinding c{q.mech, ControllerButton::L1, ControllerButton::L2, kFull};
    c.reset();
    c.makeSafe();
    c.tick(true, true, false);  // held on the very first tick
    REQUIRE(q.motor.eventCount() == 3);
    CHECK(isSafeAt(q.motor.events(), 0));
    CHECK(isVoltage(q.motor.events(), 2, 12.0));
    // reset() ALONE (no makeSafe after it) returns the binding to Idle, so its next non-driving
    // tick still makes the mechanism safe — reset() never leaves a driven mechanism un-stopped.
    c.reset();
    CHECK(c.state() == State::Idle);
    c.tick(true, false, false);
    REQUIRE(q.motor.eventCount() == 5);
    CHECK(isSafeAt(q.motor.events(), 3));
}

// ═══ B6 (brief test 6, binding half). A disconnect is a release — and a lie is ignored ═══
TEST_CASE("T3 B6: disconnected ⇒ safe once and nothing driven even when the caller hands over "
          "TRUE levels; a still-held button drives again on the reconnect tick") {
    // BUG CAUGHT: trusting the levels while disconnected (a caller that read a lying controller
    // would drive the claw with nobody at the controls); treating a disconnect as a latch that
    // needs a release (the brief: disconnect = release).
    Pivot p{"claw pivot 6"};
    MechanismBinding b{p.mech, ControllerButton::R1, ControllerButton::R2, kFull};
    b.tick(true, true, false);
    // Disconnected, and the caller claims ONE button is held — first the forward one, then the
    // reverse one. (A lie of BOTH would prove nothing: a binding that trusted it would read
    // "both held" and hold anyway. Measured: mutation MB3 survived that version of this test.)
    for (int k = 0; k < 10; ++k) {
        b.tick(false, true, false);
    }
    for (int k = 0; k < 10; ++k) {
        b.tick(false, false, true);
    }
    REQUIRE(p.motor.eventCount() == 3);
    CHECK(isSafeAt(p.motor.events(), 1));
    CHECK_FALSE(std::any_of(p.motor.events().begin() + 1, p.motor.events().end(),
                            [](const Event& e) { return e.kind == Event::Kind::Voltage
                                                        && e.volts != 0.0; }));
    CHECK(b.state() == State::Stopped);
    b.tick(true, true, false);  // reconnect, R1 still down
    CHECK(isVoltage(p.motor.events(), 3, 12.0));
}

// ═══ B9 (brief test 9). An outside stop is not undone by a finger still on the button ═══
TEST_CASE("T3 B9: an outside makeSafe() while held ⇒ safe NOW and inert until an observed "
          "release; re-arms on release-then-press") {
    // BUG CAUGHT: the next tick undoing the outside stop (the claw drives straight back after a
    // park guard or a panic stop, because the driver never let go); a latch that never re-arms.
    Pivot p{"claw pivot 6"};
    MechanismBinding b{p.mech, ControllerButton::R1, ControllerButton::R2, kFull};
    b.tick(true, true, false);
    b.makeSafe();
    REQUIRE(p.motor.eventCount() == 3);
    CHECK(isSafeAt(p.motor.events(), 1));
    CHECK(b.state() == State::Inert);
    for (int k = 0; k < 30; ++k) {
        b.tick(true, true, false);  // finger still on R1
    }
    CHECK(p.motor.eventCount() == 3);
    b.tick(true, true, true);  // pressing the OTHER button too is not a release
    b.tick(true, false, true);  // nor is letting go of only one
    CHECK(p.motor.eventCount() == 3);
    CHECK(b.state() == State::Inert);
    b.tick(true, false, false);  // the release: re-armed, and nothing commanded (already safe)
    CHECK(p.motor.eventCount() == 3);
    CHECK(b.state() == State::Stopped);
    b.tick(true, true, false);  // the new press drives
    CHECK(isVoltage(p.motor.events(), 3, 12.0));
}

TEST_CASE("T3 B9b: a DISCONNECT is not an observed release — it neither clears an outside stop "
          "nor lets one that lands while disconnected be undone on the reconnect") {
    // BUG CAUGHT: a controller blip silently re-arming a stopped claw with the driver's finger
    // still on the button. (The deliberate refinement of the brief's "disconnect = release": a
    // release for the LEVEL rules, never for the outside-stop latch.)
    SUBCASE("inert, then a blip, then the reconnect with the button still down") {
        Pivot p{"claw pivot 6"};
        MechanismBinding b{p.mech, ControllerButton::R1, ControllerButton::R2, kFull};
        b.tick(true, true, false);
        b.makeSafe();
        for (int k = 0; k < 5; ++k) {
            b.tick(false, false, false);
        }
        b.tick(true, true, false);
        CHECK(p.motor.eventCount() == 3);
        CHECK(b.state() == State::Inert);
    }
    SUBCASE("the outside stop lands WHILE disconnected") {
        Pivot p{"claw pivot 6"};
        MechanismBinding b{p.mech, ControllerButton::R1, ControllerButton::R2, kFull};
        b.tick(true, true, false);
        b.tick(false, false, false);  // a blip: safe once (the level rule)
        REQUIRE(p.motor.eventCount() == 3);
        b.makeSafe();  // nobody can see the button now
        REQUIRE(p.motor.eventCount() == 5);
        CHECK(b.state() == State::Inert);
        b.tick(true, true, false);  // reconnect, R1 down the whole time
        CHECK(p.motor.eventCount() == 5);
        b.tick(true, false, false);
        b.tick(true, true, false);
        CHECK(isVoltage(p.motor.events(), 5, 12.0));
    }
    SUBCASE("an outside stop after a CONNECTED release does not latch") {
        Pivot p{"claw pivot 6"};
        MechanismBinding b{p.mech, ControllerButton::R1, ControllerButton::R2, kFull};
        b.tick(true, true, false);
        b.tick(true, false, false);
        b.makeSafe();
        CHECK(b.state() == State::Stopped);
        b.tick(true, true, false);
        CHECK(isVoltage(p.motor.events(), p.motor.events().size() - 1, 12.0));
    }
}

// ═══ B10 (brief test 10). A claimed mechanism is never commanded ════════════════════
TEST_CASE("T3 B10: a mechanism someone else has claimed() is NEVER commanded — makeSafe() "
          "included — and a press seen during the claim does not carry over after it") {
    // BUG CAUGHT: a binding that fights an operation for the device (it would re-command its
    // voltage under a running op, or repaint the safe state the op's claimant owns); a binding
    // that takes a claim of its own (checked: the claim below is the ONLY one, and it releases).
    Pivot p{"claw pivot 6"};
    MechanismBinding b{p.mech, ControllerButton::R1, ControllerButton::R2, kFull};
    REQUIRE(p.mech.tryClaim());
    for (int k = 0; k < 10; ++k) {
        b.tick(true, true, false);
    }
    b.makeSafe();
    b.tick(true, false, true);
    b.tick(true, true, true);
    CHECK(p.motor.eventCount() == 0);
    CHECK(p.mech.claimed());
    p.mech.releaseClaim();
    b.tick(true, true, true);  // the claim is gone, the fingers are not: still nothing
    b.tick(true, true, false);
    CHECK(p.motor.eventCount() == 0);
    CHECK(b.state() == State::Inert);
    b.tick(true, false, false);  // an observed release re-arms
    b.tick(true, true, false);
    CHECK(isVoltage(p.motor.events(), 0, 12.0));
    CHECK_FALSE(p.mech.claimed());  // and the binding never took one

    // The same, with NO outside stop in the middle — the claim branch alone must latch it.
    // (Measured: with the makeSafe() above, mutation MB9 — "a press during a claim leaves the
    // binding armed" — survived, because the makeSafe latched it by itself.)
    Pivot r{"claw pivot 6b"};
    MechanismBinding d{r.mech, ControllerButton::X, ControllerButton::B, kFull};
    REQUIRE(r.mech.tryClaim());
    d.tick(true, true, false);  // pressed while an operation owns the mechanism
    r.mech.releaseClaim();
    for (int k = 0; k < 5; ++k) {
        d.tick(true, true, false);  // the operation is gone; the finger is not
    }
    CHECK(r.motor.eventCount() == 0);
    CHECK(d.state() == State::Inert);

    // A claim seen with NOTHING held leaves it armed AND Stopped — "the claimant owns the stop" —
    // so the idle ticks after the claim ends command nothing: the operation's own exit already
    // made the mechanism safe, and a second safe state here would repaint over it (and, on a V5
    // since DEFECTS2, send a second brake() that may re-capture the hold point). Then the next
    // press after the claim drives. (Review round 1: with only the press below, a claim branch
    // that set Idle instead of Stopped — mutant MB25 — survived.)
    Pivot q{"claw pivot 7"};
    MechanismBinding c{q.mech, ControllerButton::L1, ControllerButton::L2, kFull};
    REQUIRE(q.mech.tryClaim());
    c.tick(true, false, false);
    CHECK(c.state() == State::Stopped);
    q.mech.releaseClaim();
    for (int k = 0; k < 5; ++k) {
        c.tick(true, false, false);  // the operation is gone and nothing is held: silence
    }
    CHECK(q.motor.eventCount() == 0);
    CHECK(c.state() == State::Stopped);
    c.tick(true, true, false);
    CHECK(isVoltage(q.motor.events(), 0, 12.0));
}

// ═══ B10b (the claim ruling, binding half). releaseLeftoverClaim() ═════════════════════
TEST_CASE("T3 B10b: releaseLeftoverClaim() releases a claim that is there (and says so), never "
          "calls its claimant, commands nothing, leaves the state alone — and answers false, "
          "forgetting the last answer, when there is none") {
    // BUG CAUGHT: a release that calls the claimant's cancel() (the dead operation's object lived
    // on a deleted task's stack); a release that also makes the mechanism safe (the loop's entry
    // safe state is its own step, and a second one repaints); a flag that never clears (the panel
    // would say FREED forever); a flag that is set without a claim.
    struct Spy final : shulib::hal::ICancellable {
        void cancel() override { ++cancels; }
        int cancels = 0;
    };
    Pivot p{"claw pivot 6"};
    MechanismBinding b{p.mech, ControllerButton::R1, ControllerButton::R2, kFull};
    CHECK_FALSE(b.releasedLeftoverClaim());  // a fresh binding has released nothing
    Spy spy;
    REQUIRE(p.mech.tryClaim(spy));
    b.tick(true, true, false);  // pressed under the claim: Inert, nothing commanded
    REQUIRE(b.state() == State::Inert);
    CHECK(b.releaseLeftoverClaim());
    CHECK(b.releasedLeftoverClaim());
    CHECK_FALSE(p.mech.claimed());
    CHECK(p.mech.claimant() == nullptr);
    CHECK(spy.cancels == 0);
    CHECK(p.motor.eventCount() == 0);       // commands nothing
    CHECK(b.state() == State::Inert);       // the state is the loop's reset()'s to change
    CHECK_FALSE(b.releaseLeftoverClaim());  // nothing there now
    CHECK_FALSE(b.releasedLeftoverClaim());  // and the last answer is forgotten
    CHECK(p.motor.eventCount() == 0);
    // It never TAKES a claim: a mechanism nobody claimed stays unclaimed.
    CHECK_FALSE(p.mech.claimed());
}

// ═══ B11 (brief test 11, binding half). Refused at CONSTRUCTION ═════════════════════
TEST_CASE("T3 B11: construction refuses forward == reverse, a non-button, V <= 0, V > 12, NaN "
          "and infinity — and accepts the edges") {
    Pivot p{"claw pivot 6"};
    const auto make = [&](ControllerButton f, ControllerButton r, double v) {
        MechanismBinding b{p.mech, f, r, Voltage{v}};
        static_cast<void>(b);
    };
    CHECK_THROWS_AS(make(ControllerButton::R1, ControllerButton::R1, 12.0), PreconditionError);
    CHECK_THROWS_AS(make(static_cast<ControllerButton>(12), ControllerButton::R1, 12.0),
                    PreconditionError);
    CHECK_THROWS_AS(make(ControllerButton::R1, static_cast<ControllerButton>(-1), 12.0),
                    PreconditionError);
    CHECK_THROWS_AS(make(ControllerButton::R1, ControllerButton::R2, 0.0), PreconditionError);
    CHECK_THROWS_AS(make(ControllerButton::R1, ControllerButton::R2, -12.0), PreconditionError);
    CHECK_THROWS_AS(make(ControllerButton::R1, ControllerButton::R2, 12.000001),
                    PreconditionError);
    CHECK_THROWS_AS(make(ControllerButton::R1, ControllerButton::R2, std::nan("")),
                    PreconditionError);
    CHECK_THROWS_AS(
        make(ControllerButton::R1, ControllerButton::R2, std::numeric_limits<double>::infinity()),
        PreconditionError);
    CHECK_NOTHROW(make(ControllerButton::R1, ControllerButton::R2, 12.0));  // the rail itself
    CHECK_NOTHROW(make(ControllerButton::A, ControllerButton::L1, 1e-3));   // any positive V
    CHECK(p.motor.eventCount() == 0);  // constructing a binding commands nothing
}

TEST_CASE("T3 B11b: validBindings() refuses a null, a re-zero clash, a button used twice and a "
          "mechanism bound twice — naming the button and the mechanisms") {
    Pivot six{"claw pivot 6"};
    Pivot seven{"claw pivot 7"};
    MechanismBinding a{six.mech, ControllerButton::R1, ControllerButton::R2, kFull};
    MechanismBinding b{seven.mech, ControllerButton::L1, ControllerButton::L2, kFull};
    char why[192] = "stale";
    const auto verdict = [&](std::span<MechanismBinding* const> set, ControllerButton rezero) {
        const bool ok = validBindings(set, rezero, why, sizeof why);
        return ok ? std::string{} : std::string{why};
    };

    SUBCASE("robot two's pair, with the re-zero on A, is valid and leaves no stale sentence") {
        std::array<MechanismBinding*, 2> set{&a, &b};
        CHECK(verdict(set, ControllerButton::A).empty());
        CHECK(std::strlen(why) == 0);
        CHECK(validBindings(set, ControllerButton::A));  // the no-buffer form
        CHECK(validBindings(std::span<MechanismBinding* const>{}, ControllerButton::L1));
    }
    SUBCASE("the loop's DEFAULT re-zero button is L1 — the port-7 forward button") {
        std::array<MechanismBinding*, 2> set{&a, &b};
        const std::string w = verdict(set, DriverLoopConfig{}.rezeroButton);
        CHECK(w.find("L1") != std::string::npos);
        CHECK(w.find("claw pivot 7") != std::string::npos);
        CHECK(w.find("re-zero") != std::string::npos);
        // A reverse button is checked too, not only a forward one.
        CHECK(verdict(set, ControllerButton::R2).find("R2") != std::string::npos);
    }
    SUBCASE("one button on two mechanisms") {
        MechanismBinding c{seven.mech, ControllerButton::Y, ControllerButton::R2, kFull};
        std::array<MechanismBinding*, 2> set{&a, &c};
        const std::string w = verdict(set, ControllerButton::A);
        CHECK(w.find("button R2") != std::string::npos);
        CHECK(w.find("claw pivot 6") != std::string::npos);
        CHECK(w.find("claw pivot 7") != std::string::npos);
    }
    SUBCASE("one mechanism bound twice, on four different buttons") {
        MechanismBinding c{six.mech, ControllerButton::X, ControllerButton::B, kFull};
        std::array<MechanismBinding*, 2> set{&a, &c};
        const std::string w = verdict(set, ControllerButton::A);
        CHECK(w.find("claw pivot 6") != std::string::npos);
        CHECK(w.find("bound twice") != std::string::npos);
    }
    SUBCASE("a null entry") {
        std::array<MechanismBinding*, 2> set{&a, nullptr};
        CHECK(verdict(set, ControllerButton::A).find("binding 1 is null") != std::string::npos);
    }
}

// ═══════════════════════════════ the binding in THE LOOP ═════════════════════════════

namespace {

/// A controller that COUNTS every read and LIES while disconnected (it keeps reporting whatever
/// the test set), so a loop that polled it anyway is visible in the counters and in the motors.
class LyingController final : public shulib::hal::IController {
public:
    [[nodiscard]] double axis(ControllerAxis a) const override {
        if (!connected) {
            ++readsWhileDisconnected;
        }
        return axes[static_cast<std::size_t>(a)];
    }
    [[nodiscard]] bool pressed(ControllerButton b) const override {
        ++buttonReads;
        if (!connected) {
            ++readsWhileDisconnected;
        }
        return buttons[static_cast<std::size_t>(b)];
    }
    [[nodiscard]] bool isConnected() const override { return connected; }

    void hold(ControllerButton b, bool down) { buttons[static_cast<std::size_t>(b)] = down; }
    void releaseAll() { buttons = {}; }

    std::array<double, 4> axes{};
    std::array<bool, 12> buttons{};
    bool connected = true;
    mutable int buttonReads = 0;
    mutable int readsWhileDisconnected = 0;
};

constexpr DriverLoopConfig kLoopCfg{.tickPeriod = Time{0.01}, .rezeroButton = ControllerButton::A};

HeadingSample trusted() { return HeadingSample{.radians = 0.0, .trusted = true}; }

const shulib::kinematics::TankKinematics kTank{Length{11.0}};

/// What one tick left behind, read in perTick (after drive() and the bindings, before pace()).
struct LoopTick {
    double leftVolts = 0.0;
    double rightVolts = 0.0;
    ChassisSpeeds command{};
    std::size_t sixEvents = 0;
    std::size_t sevenEvents = 0;
    double sixVolts = 0.0;
    double sevenVolts = 0.0;
};

/// Robot two's pair: pivot 6 on R1/R2, pivot 7 on L1/L2, 12 V, Hold — and the loop around them.
struct ClawLoop {
    /// The chassis's own sink — where the loop says what it did that nobody pressed for (a
    /// released leftover claim, tag TEL).
    shulib::hal::fake::FakeTelemetrySink sink;
    motion_rig::ChassisRig c{kTank, motion_rig::plantConfig(), &sink};
    LyingController pad;
    DriveShaper shaper{DriverProfile{}};
    Pivot six{"claw pivot 6"};
    Pivot seven{"claw pivot 7"};
    MechanismBinding bind6{six.mech, ControllerButton::R1, ControllerButton::R2, kFull};
    MechanismBinding bind7{seven.mech, ControllerButton::L1, ControllerButton::L2, kFull};
    std::array<MechanismBinding*, 2> bindings{&bind6, &bind7};

    /// Run `ticks` ticks; `script(k)` runs at the TOP of tick k (inside the disabled predicate,
    /// which the loop polls first). `withBindings` false runs the 2a signature.
    std::vector<LoopTick> run(int ticks, const std::function<void(int)>& script,
                              bool withBindings = true,
                              const DriverLoopConfig& cfg = kLoopCfg) {
        std::vector<LoopTick> log;
        int k = 0;
        const auto disabled = [&] {
            if (k >= ticks) {
                return true;
            }
            script(k);
            return false;
        };
        const auto perTick = [&](const ChassisSpeeds& command, bool, const DriveShaperState&) {
            log.push_back(LoopTick{.leftVolts = c.rig.h.motor(0).commandedVoltage().value(),
                                   .rightVolts = c.rig.h.motor(1).commandedVoltage().value(),
                                   .command = command,
                                   .sixEvents = six.motor.events().size(),
                                   .sevenEvents = seven.motor.events().size(),
                                   .sixVolts = six.motor.commandedVoltage().value(),
                                   .sevenVolts = seven.motor.commandedVoltage().value()});
            ++k;
        };
        if (withBindings) {
            runDriverControl(c.chassis, pad, c.pacer, shaper, cfg,
                             std::span<MechanismBinding* const>{bindings}, disabled,
                             [] { return trusted(); }, perTick);
        } else {
            runDriverControl(c.chassis, pad, c.pacer, shaper, cfg, disabled,
                             [] { return trusted(); }, perTick);
        }
        return log;
    }
};

}  // namespace

// ═══ L4 (brief test 4) + L5 (brief test 5). Two pivots, one tick; a press lands in its tick ═══
TEST_CASE("T3 L4/L5: R1 + L2 together ⇒ +12 V on pivot 6 and -12 V on pivot 7 IN THE SAME TICK "
          "they are read; each release lands in its own tick") {
    // BUG CAUGHT: swapped buttons (L1 driving 6), swapped pivots, a wrong sign, a wrong voltage,
    // bindings ticked BEFORE drive() or after pace() (a press would slip a tick), one binding
    // shadowing the other (only the first in the span ticked).
    ClawLoop w;
    constexpr int kPress = 3;
    constexpr int kRelease6 = 7;
    constexpr int kRelease7 = 9;
    const auto log = w.run(12, [&](int k) {
        w.pad.hold(ControllerButton::R1, k >= kPress && k < kRelease6);
        w.pad.hold(ControllerButton::L2, k >= kPress && k < kRelease7);
    });
    REQUIRE(log.size() == 12U);
    // Before the press: only the entry safe state on each (two events), nothing energised.
    CHECK(log[kPress - 1].sixEvents == 2U);
    CHECK(log[kPress - 1].sevenEvents == 2U);
    // THE press tick: both at full power, opposite ways, in the tick the buttons were read.
    CHECK(log[kPress].sixVolts == 12.0);
    CHECK(log[kPress].sevenVolts == -12.0);
    CHECK(isVoltage(w.six.motor.events(), 2, 12.0));
    CHECK(isVoltage(w.seven.motor.events(), 2, -12.0));
    // Each release is safe in ITS tick, and not a tick later.
    CHECK(log[kRelease6 - 1].sixVolts == 12.0);
    CHECK(log[kRelease6].sixVolts == 0.0);
    CHECK(isSafeAt(w.six.motor.events(), log[kRelease6].sixEvents - 2));
    CHECK(log[kRelease6].sevenVolts == -12.0);  // 7 is still held
    CHECK(log[kRelease7].sevenVolts == 0.0);
    CHECK(isSafeAt(w.seven.motor.events(), log[kRelease7].sevenEvents - 2));
    // After each release, silence to the last tick: entry safe + 4 drives + one safe = 8 events
    // on pivot 6 — and the disable at the end adds its own safe state (the outside stop), +2.
    CHECK(log.back().sixEvents == log[kRelease6].sixEvents);
    CHECK(log[kRelease6].sixEvents == 2U + 4U + 2U);
    CHECK(log.back().sevenEvents == log[kRelease7].sevenEvents);
    CHECK(w.six.motor.events().size() == 2U + 4U + 2U + 2U);
    CHECK(brakeEvents(w.six.motor.events()) == 3);  // entry, release, disable — once each
    CHECK(brakeEvents(w.seven.motor.events()) == 3);
}

// ═══ L6 (brief test 6). A LYING disconnected controller ═════════════════════════════
TEST_CASE("T3 L6: disconnected with a controller that still claims every button ⇒ each pivot safe "
          "once, NOTHING read or commanded while disconnected, resume on the reconnect tick") {
    // BUG CAUGHT: reading the buttons while disconnected (a real V5 reads false then, HA-103,
    // which would hide it; this controller does not), or trusting what was read.
    ClawLoop w;
    const auto log = w.run(14, [&](int k) {
        w.pad.connected = k < 4 || k >= 10;  // gone for ticks 4..9
        w.pad.hold(ControllerButton::R1, true);
        w.pad.hold(ControllerButton::L1, true);
        w.pad.hold(ControllerButton::L2, k >= 4);  // "pressed" during the drop: a lie, ignored
    });
    REQUIRE(log.size() == 14U);
    CHECK(w.pad.readsWhileDisconnected == 0);
    CHECK(log[3].sixVolts == 12.0);
    CHECK(log[3].sevenVolts == 12.0);
    for (std::size_t k = 4; k < 10; ++k) {
        CAPTURE(k);
        CHECK(log[k].sixVolts == 0.0);
        CHECK(log[k].sevenVolts == 0.0);
        CHECK(log[k].sixEvents == log[4].sixEvents);  // one safe state at tick 4, then silence
    }
    CHECK(isSafeAt(w.six.motor.events(), log[4].sixEvents - 2));
    // The reconnect tick: R1 is still down ⇒ pivot 6 drives at once; pivot 7 has BOTH L1 and L2
    // down ⇒ it holds (R2 ruling), and it is already safe, so it stays silent.
    CHECK(log[10].sixVolts == 12.0);
    CHECK(log[10].sevenVolts == 0.0);
    CHECK(log[10].sevenEvents == log[9].sevenEvents);
    // Connected ticks read the re-zero button once and each binding's two buttons once: 5 reads.
    CHECK(w.pad.buttonReads == 8 * 5);
    // The loop hands the binding the CONNECTED flag, not a constant: a field disable that lands
    // while the controller is gone must latch the pivot (nobody can see the button), which only
    // happens if the binding was told this tick was disconnected.
    w.run(2, [&](int) {
        w.pad.connected = false;
        w.pad.releaseAll();
    });
    CHECK(w.bind6.state() == State::Inert);
}

// ═══ L7 (brief test 7). A field disable while both pivots are held ══════════════════
TEST_CASE("T3 L7: disabled while held ⇒ chassis.cancel() FIRST, then both pivots safe, inside "
          "the disable tick (no pace), then the loop returns") {
    // BUG CAUGHT: leaving a claw at +12 V into a field disable (PROS deletes the task; nothing
    // after this return runs); safing the claws but not the drive, or in the wrong order; a
    // disable path that paces once more.
    ClawLoop w;
    // A snoop on pivot 6: when its safe state lands, what did the DRIVE look like at that
    // instant? cancel() puts the drive at 0 V + Brake, so "cancel first" is observable here.
    struct Snoop final : IMotor {
        RecordingMotor* inner = nullptr;
        shulib::sim::SimHarness* h = nullptr;
        std::vector<BrakeMode> driveModeAtBrake;
        std::vector<double> driveVoltsAtCommand;  ///< the drive's volts when a pivot volt landed
        void setVoltage(Voltage v) override {
            if (v.value() != 0.0) {
                driveVoltsAtCommand.push_back(h->motor(0).commandedVoltage().value());
            }
            inner->setVoltage(v);
        }
        [[nodiscard]] Voltage commandedVoltage() const override {
            return inner->commandedVoltage();
        }
        void setBrakeMode(BrakeMode m) override {
            driveModeAtBrake.push_back(h->motor(0).brakeMode());
            inner->setBrakeMode(m);
        }
        [[nodiscard]] BrakeMode brakeMode() const override { return inner->brakeMode(); }
        [[nodiscard]] shulib::units::AngleDim position() const override {
            return inner->position();
        }
        [[nodiscard]] shulib::units::AngularVelocity velocity() const override {
            return inner->velocity();
        }
        [[nodiscard]] shulib::units::Current current() const override { return inner->current(); }
        [[nodiscard]] double temperature() const override { return inner->temperature(); }
    };
    Snoop snoop;
    snoop.inner = &w.six.motor;
    snoop.h = &w.c.rig.h;
    w.six.motors[0] = &snoop;  // the mechanism's span points at this array
    const auto log = w.run(6, [&](int) {
        w.pad.hold(ControllerButton::R1, true);
        w.pad.hold(ControllerButton::L2, true);
        w.pad.axes[1] = 1.0;  // and the drive is moving
    });
    REQUIRE(log.size() == 6U);
    CHECK(log.back().sixVolts == 12.0);
    CHECK(log.back().sevenVolts == -12.0);
    CHECK(log.back().leftVolts > 1.0);
    // The disable tick added EXACTLY the safe state to each, and paced nothing.
    CHECK(w.six.motor.events().size() == log.back().sixEvents + 2);
    CHECK(w.seven.motor.events().size() == log.back().sevenEvents + 2);
    CHECK(isSafeAt(w.six.motor.events(), log.back().sixEvents));
    CHECK(isSafeAt(w.seven.motor.events(), log.back().sevenEvents));
    CHECK(w.c.pacer.paces == 6);
    CHECK(w.c.rig.h.motor(0).commandedVoltage().value() == 0.0);
    CHECK(w.c.rig.h.motor(0).brakeMode() == BrakeMode::Brake);
    // The snoop saw the drive ALREADY braked when pivot 6's disable-time safe state landed.
    REQUIRE_FALSE(snoop.driveModeAtBrake.empty());
    CHECK(snoop.driveModeAtBrake.back() == BrakeMode::Brake);
    // And on the FIRST tick, pivot 6's +12 V landed after that tick's drive() had already put
    // the drive's volts on the motors (the loop's order: drive, THEN the bindings). Before tick 0
    // the drive motor was at 0 V, so a loop that ticked the bindings first would record 0 here.
    REQUIRE_FALSE(snoop.driveVoltsAtCommand.empty());
    CHECK(snoop.driveVoltsAtCommand.front() > 1.0);
    // And the binding is inert: whatever ticks next (nothing will, on a brain) cannot drive it.
    CHECK(w.bind6.state() == State::Inert);
}

// ═══ L8 (brief test 8). Safe before tick 1 ═════════════════════════════════════════
TEST_CASE("T3 L8: every pivot is made safe at loop ENTRY, before the first disabled() poll — and "
          "re-entry after a disable forgets the latch, so a held button drives at once") {
    // BUG CAUGHT: a loop that first touches a mechanism when a button is pressed — the claw would
    // sit in whatever state autonomous (or a previous run) left it; a re-entry that stays inert.
    ClawLoop w;
    std::size_t atFirstPoll6 = 99;
    std::size_t atFirstPoll7 = 99;
    const auto log = w.run(3, [&](int k) {
        if (k == 0) {
            atFirstPoll6 = w.six.motor.events().size();
            atFirstPoll7 = w.seven.motor.events().size();
        }
        w.pad.hold(ControllerButton::R1, true);
    });
    CHECK(atFirstPoll6 == 2U);
    CHECK(atFirstPoll7 == 2U);
    CHECK(isSafeAt(w.six.motor.events(), 0));
    CHECK(isSafeAt(w.seven.motor.events(), 0));
    CHECK(log[0].sixVolts == 12.0);  // held from the first tick: drives in the first tick
    REQUIRE(w.bind6.state() == State::Inert);  // the disable latched it…
    // …and PROS calling opcontrol() again (a second call) starts from nothing.
    const auto again = w.run(2, [&](int) { w.pad.hold(ControllerButton::R1, true); });
    REQUIRE(again.size() == 2U);
    CHECK(again[0].sixVolts == 12.0);

    SUBCASE("disabled on the very first poll: safe at entry, safe at the disable, nothing else") {
        ClawLoop d;
        d.pad.hold(ControllerButton::R1, true);
        int perTicks = 0;
        runDriverControl(
            d.c.chassis, d.pad, d.c.pacer, d.shaper, kLoopCfg,
            std::span<MechanismBinding* const>{d.bindings}, [] { return true; },
            [] { return trusted(); },
            [&](const ChassisSpeeds&, bool, const DriveShaperState&) { ++perTicks; });
        CHECK(perTicks == 0);
        REQUIRE(d.six.motor.events().size() == 4U);
        CHECK(isSafeAt(d.six.motor.events(), 0));
        CHECK(isSafeAt(d.six.motor.events(), 2));
        CHECK_FALSE(d.six.motor.everEnergized());
        CHECK(d.pad.buttonReads == 0);
    }
}

// ═══ L10 (brief test 10, loop half; the claim ruling, T3 Part 2). A claim already there ══
namespace {

/// A claimant that must NEVER be called: the stranded operation it stands for lived on the stack of
/// a task PROS deleted, so calling it would be a use-after-free on a brain.
struct DeadClaimant final : shulib::hal::ICancellable {
    void cancel() override { ++cancels; }
    int cancels = 0;
};

/// The TEL lines the loop wrote, in order.
std::vector<std::string> telLines(const shulib::hal::fake::FakeTelemetrySink& sink) {
    std::vector<std::string> out;
    for (int i = 0; i < sink.size(); ++i) {
        if (sink.at(i).subsystem == "TEL") {
            CHECK(sink.at(i).level == shulib::hal::LogLevel::Warn);
            out.push_back(sink.at(i).message);
        }
    }
    return out;
}

}  // namespace

TEST_CASE("T3 L10: a claim left on a bound mechanism when runDriverControl starts is RELEASED at "
          "entry and SAID, once — then the pivot is made safe and its button drives from the first "
          "tick (the team lead's ruling, 2026-09-26)") {
    // BUG CAUGHT: a stranded claim (an autonomous operation the field's disable deleted
    // mid-operation) locking the driver out of the claw for the whole driver period — what this
    // case pinned as the GAP before the ruling; a release nobody is told about; a release AFTER the
    // entry safe state (the pivot would skip its safe state and first move at +12 V); a release
    // that calls the dead claimant's cancel() (its object lived on the deleted task's stack); a
    // release that also frees a mechanism nobody claimed, or logs one that was not claimed.
    ClawLoop w;
    DeadClaimant dead;
    REQUIRE(w.six.mech.tryClaim(dead));  // what an autonomous task deleted mid-operation leaves
    std::size_t sinkAtFirstPoll = 99;
    std::size_t sixEventsAtFirstPoll = 99;
    const auto log = w.run(8, [&](int k) {
        if (k == 0) {
            sinkAtFirstPoll = telLines(w.sink).size();
            sixEventsAtFirstPoll = w.six.motor.events().size();
        }
        w.pad.hold(ControllerButton::R1, true);
        w.pad.hold(ControllerButton::L1, true);
        w.pad.axes[1] = 1.0;  // and the driver is driving
    });
    REQUIRE(log.size() == 8U);
    // Released, and never through the claimant.
    CHECK_FALSE(w.six.mech.claimed());
    CHECK(w.six.mech.claimant() == nullptr);
    CHECK(dead.cancels == 0);
    // Said ONCE, before the first poll, naming the mechanism — and nothing about pivot 7.
    CHECK(sinkAtFirstPoll == 1U);
    const std::vector<std::string> said = telLines(w.sink);
    REQUIRE(said.size() == 1U);
    CHECK(said[0]
          == "driver control: RELEASED a claim left on mechanism 'claw pivot 6' at entry (an "
             "operation that outlived its task) -- its buttons work from the first tick");
    CHECK(w.bind6.releasedLeftoverClaim());  // what a panel reads
    CHECK_FALSE(w.bind7.releasedLeftoverClaim());
    // Then the ordinary entry: the declared safe state BEFORE the first poll (the release came
    // first, or makeSafe() would have skipped the claimed pivot), and R1 drives from tick 0.
    CHECK(sixEventsAtFirstPoll == 2U);
    CHECK(isSafeAt(w.six.motor.events(), 0));
    for (const LoopTick& t : log) {
        CHECK(t.sixVolts == 12.0);
        CHECK(t.sevenVolts == 12.0);
    }
    CHECK(log.back().leftVolts > 1.0);  // the drive is unaffected
    // The binding took no claim of its own while driving, and the disable safes it as usual.
    CHECK_FALSE(w.six.mech.claimed());
    CHECK(isSafeAt(w.six.motor.events(), w.six.motor.events().size() - 2));
    // The NEXT entry finds no claim: nothing is said, and the panel's word goes away.
    const auto again = w.run(2, [&](int) { w.pad.hold(ControllerButton::R1, true); });
    REQUIRE(again.size() == 2U);
    CHECK(telLines(w.sink).size() == 1U);
    CHECK_FALSE(w.bind6.releasedLeftoverClaim());
    CHECK(again[0].sixVolts == 12.0);

    SUBCASE("both pivots claimed (one anonymously): two lines, in binding order") {
        ClawLoop two;
        DeadClaimant d6;
        REQUIRE(two.six.mech.tryClaim(d6));
        REQUIRE(two.seven.mech.tryClaim());  // an anonymous claim is released the same way
        two.run(1, [](int) {});
        const std::vector<std::string> lines = telLines(two.sink);
        REQUIRE(lines.size() == 2U);
        CHECK(lines[0].find("'claw pivot 6'") != std::string::npos);
        CHECK(lines[1].find("'claw pivot 7'") != std::string::npos);
        CHECK_FALSE(two.six.mech.claimed());
        CHECK_FALSE(two.seven.mech.claimed());
        CHECK(isSafeAt(two.seven.motor.events(), 0));
        CHECK(d6.cancels == 0);
    }
}

// ═══ L13 (the claim ruling's other half). A claim taken DURING the loop is still obeyed ══
TEST_CASE("T3 L13: a claim taken DURING driver control is never commanded by the loop — not by "
          "the held button, not by the disable — and is not released; only the NEXT entry "
          "releases it (the ruling is at entry, and only there)") {
    // BUG CAUGHT: a loop that releases claims every tick (an operation a program runs from perTick
    // would lose its mechanism mid-run to a finger on the button); a binding that commands a
    // mechanism an operation owns; a disable path that safes a claimed mechanism under its owner.
    ClawLoop w;
    constexpr int kClaimAt = 3;
    const auto log = w.run(10, [&](int k) {
        if (k == kClaimAt) {
            REQUIRE(w.six.mech.tryClaim());  // an operation starts on pivot 6, mid-driving
        }
        w.pad.hold(ControllerButton::R1, true);
    });
    REQUIRE(log.size() == 10U);
    CHECK(log[kClaimAt - 1].sixVolts == 12.0);
    // From the claim on: not one more event on pivot 6 — not +12 V, not the disable's safe state.
    CHECK(w.six.motor.events().size() == log[kClaimAt - 1].sixEvents);
    CHECK(w.six.mech.claimed());  // still the operation's when the loop returns
    CHECK(w.bind6.state() == State::Inert);
    CHECK(telLines(w.sink).empty());  // nothing was released, so nothing was said
    CHECK_FALSE(w.bind6.releasedLeftoverClaim());
    // PROS calls opcontrol() again: at THAT entry the claim is a leftover, and it is released.
    const auto again = w.run(2, [&](int) { w.pad.hold(ControllerButton::R1, true); });
    REQUIRE(again.size() == 2U);
    CHECK_FALSE(w.six.mech.claimed());
    CHECK(telLines(w.sink).size() == 1U);
    CHECK(again[0].sixVolts == 12.0);
}

// ═══ L11 (brief test 11, loop half). The loop refuses a bad set BEFORE anything moves ═══
TEST_CASE("T3 L11: runDriverControl refuses a binding on the re-zero button (the default L1) or a "
          "button used twice — before the first poll, before touching any mechanism") {
    // BUG CAUGHT: a loop that starts anyway, so L1 both re-zeroes field-centric AND drives pivot 7;
    // a loop that releases a leftover claim BEFORE it has checked the set (a refused program must
    // leave every mechanism exactly as it found it — the claim ruling acts only on a loop that runs).
    ClawLoop w;
    REQUIRE(w.six.mech.tryClaim());
    int polled = 0;
    const auto never = [&](const ChassisSpeeds&, bool, const DriveShaperState&) {};
    const auto poll = [&] {
        ++polled;
        return false;
    };
    const DriverLoopConfig defaultRezero{.tickPeriod = Time{0.01}};  // rezeroButton = L1
    CHECK_THROWS_AS(runDriverControl(w.c.chassis, w.pad, w.c.pacer, w.shaper, defaultRezero,
                                     std::span<MechanismBinding* const>{w.bindings}, poll,
                                     [] { return trusted(); }, never),
                    PreconditionError);
    MechanismBinding clash{w.seven.mech, ControllerButton::R2, ControllerButton::Y, kFull};
    std::array<MechanismBinding*, 2> dup{&w.bind6, &clash};
    CHECK_THROWS_AS(runDriverControl(w.c.chassis, w.pad, w.c.pacer, w.shaper, kLoopCfg,
                                     std::span<MechanismBinding* const>{dup}, poll,
                                     [] { return trusted(); }, never),
                    PreconditionError);
    CHECK(polled == 0);
    CHECK(w.c.pacer.paces == 0);
    CHECK(w.six.motor.eventCount() == 0);
    CHECK(w.seven.motor.eventCount() == 0);
    CHECK(w.six.mech.claimed());       // the leftover claim is still there…
    CHECK(telLines(w.sink).empty());   // …and nothing claimed to have released it
    CHECK_FALSE(w.bind6.releasedLeftoverClaim());
}

// ═══ L12 (brief test 12). The drive does not know the bindings exist ════════════════
TEST_CASE("T3 L12: the same stick-and-button script with and without bindings commands "
          "BIT-IDENTICAL drive volts and chassis commands on every tick") {
    // BUG CAUGHT: a binding that reads the sticks, a loop that re-orders drive() around the
    // bindings, a binding press that leaks into the drive (the re-zero edge, the shaper).
    const auto script = [](ClawLoop& w) {
        return [&w](int k) {
            w.pad.axes[1] = std::sin(0.13 * k);        // LeftY
            w.pad.axes[2] = 0.5 * std::cos(0.07 * k);  // RightX
            w.pad.hold(ControllerButton::R1, (k / 5) % 2 == 0);
            w.pad.hold(ControllerButton::L2, (k / 3) % 3 == 1);
            w.pad.hold(ControllerButton::R2, k % 11 == 0);
            w.pad.connected = k % 17 != 16;
        };
    };
    ClawLoop with;
    ClawLoop without;
    const auto a = with.run(120, script(with), /*withBindings=*/true);
    const auto b = without.run(120, script(without), /*withBindings=*/false);
    REQUIRE(a.size() == 120U);
    REQUIRE(b.size() == 120U);
    int moving = 0;
    for (std::size_t k = 0; k < a.size(); ++k) {
        CAPTURE(k);
        CHECK(sameBits(a[k].leftVolts, b[k].leftVolts));
        CHECK(sameBits(a[k].rightVolts, b[k].rightVolts));
        CHECK(sameBits(a[k].command.vx().value(), b[k].command.vx().value()));
        CHECK(sameBits(a[k].command.omega().value(), b[k].command.omega().value()));
        moving += std::abs(a[k].leftVolts) > 1.0 ? 1 : 0;
    }
    CHECK(moving > 40);  // the drive really moved — identity of two idle runs would prove nothing
    CHECK(with.six.motor.everEnergized());  // and the bindings really drove, in the same run
    CHECK_FALSE(without.six.motor.everEnergized());
}
