// teleop::ToggleBinding and runDriverControl's two-span overload (T3's addendum B2, brief §2
// items 1-2 and §2 item 5's binding and loop tests) — the pneumatic claw on A.
//
// ═══ WHAT THIS FILE PINS ════════════════════════════════════════════════════════════
// Every rule mechanism_binding.hpp's TOGGLE section states, asserted at the BOTTOM of the stack —
// the ordered levels a recording line was driven to — never against the binding's own idea of
// what it did. Two halves:
//
//   * the TOGGLE alone (T-cases): one press is one flip and holding does nothing more; the flip is
//     the opposite of what the line was last COMMANDED, whoever commanded it; the initial level is
//     stated and sent once in the binding's life, never at construction and never on a later
//     entry; a level the binding could not see counts as held, so a press made while the
//     controller was gone is not replayed — and the one belief that rests on (the first CONNECTED
//     tick's level is live), pinned at its boundary; a claimed mechanism is never commanded and a
//     press under a claim is dropped; and the set rules validBindings() applies across both kinds;
//   * the toggle inside THE LOOP (TL-cases), on the host test plant beside robot two's two claw
//     pivots: the initial level before the first poll, a press read at tick k flipped in tick k
//     (after drive() and after the hold bindings), a lying disconnected controller, a field
//     disable that makes the pivots safe and leaves the claw exactly where it is — through a
//     re-entry too — the kinds never touching each other's mechanisms, the drive bit-identical,
//     and the loop's refusal before tick 1.
//
// The claw here is named after robot two's (3-wire port A, button A, starting CLOSED) only because
// that is the case that motivated the addendum. WHICH LEVEL CLOSES robot two's claw is not known
// (the program states it, marked unverified); in this file "closed" is simply the line off.
//
// NO HARDWARE CLAIM. Recording lines and the host test plant; no solenoid has fired.

#include "doctest.h"

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <span>
#include <string>
#include <type_traits>
#include <vector>

#include "mechanism_test_rig.hpp"
#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/digital_out.hpp"
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
using shulib::hal::IDigitalOut;
using shulib::hal::IMotor;
using shulib::hal::MotorMechanism;
using shulib::hal::PneumaticMechanism;
using shulib::math::ChassisSpeeds;
using shulib::teleop::DriverLoopConfig;
using shulib::teleop::DriverProfile;
using shulib::teleop::DriveShaper;
using shulib::teleop::DriveShaperState;
using shulib::teleop::HeadingSample;
using shulib::teleop::MechanismBinding;
using shulib::teleop::runDriverControl;
using shulib::teleop::ToggleBinding;
using shulib::teleop::validBindings;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Voltage;
using Event = RecordingMotor::Event;
using Levels = std::vector<bool>;

namespace {

constexpr Voltage kFull{12.0};
constexpr bool kClosed = false;  ///< in this file only: CLOSED is the line off
constexpr bool kOpen = true;

/// A 3-wire line that records EVERY level it is driven to, in order, and can run a hook at the
/// instant of each write (a snoop on what the rest of the robot looked like then).
class RecordingLine final : public IDigitalOut {
public:
    void set(bool value) override {
        if (onSet) {
            onSet(value);
        }
        writes.push_back(value);
        level = value;
    }
    [[nodiscard]] bool commanded() const override { return level; }

    Levels writes;
    bool level = false;
    std::function<void(bool)> onSet;
};

/// One pneumatic claw: a recording line behind a one-line PneumaticMechanism whose declared safe
/// level is `safe` (closed unless a case says otherwise). Non-movable, so built where it lives.
struct Claw {
    explicit Claw(const char* name, bool safe = kClosed)
        : mech{std::span<IDigitalOut* const>{lines}, safe, name} {}
    RecordingLine line;
    std::array<IDigitalOut*, 1> lines{&line};
    PneumaticMechanism mech;
};

/// One claw pivot: a RecordingMotor behind a single-motor MotorMechanism, safe in Hold.
struct Pivot {
    explicit Pivot(const char* name) : mech{std::span<IMotor* const>{motors}, BrakeMode::Hold, name} {}
    RecordingMotor motor;
    std::array<IMotor*, 1> motors{&motor};
    MotorMechanism mech;
};

/// True when events [from, from+2) are exactly Brake(Hold) THEN 0 V.
bool isSafeAt(const std::vector<Event>& ev, std::size_t from) {
    return ev.size() >= from + 2 && ev[from].kind == Event::Kind::Brake
           && ev[from].mode == BrakeMode::Hold && ev[from + 1].kind == Event::Kind::Voltage
           && ev[from + 1].volts == 0.0;
}

[[nodiscard]] bool sameBits(double a, double b) {
    return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

}  // namespace

// ═══════════════════════════════ the TOGGLE alone ════════════════════════════════════

// ═══ T1. One press, one flip ════════════════════════════════════════════════════════
TEST_CASE("B2 T1: one press ⇒ exactly one flip; the button held for 50 ticks ⇒ still one; press, "
          "release, press ⇒ two flips") {
    // BUG CAUGHT: flipping on the LEVEL instead of the edge (a held button would chatter the valve
    // open-closed-open every 10 ms, emptying the tank); a release counted as a press; a flip that
    // is not the opposite of the level before it.
    Claw c{"claw"};
    ToggleBinding t{c.mech, ControllerButton::A, kClosed};
    t.enter();
    REQUIRE(c.line.writes == Levels{kClosed});
    t.tick(true, false);  // seen up
    CHECK(c.line.writes.size() == 1U);
    t.tick(true, true);  // THE press: one flip, in this tick
    CHECK(c.line.writes == (Levels{kClosed, kOpen}));
    for (int k = 0; k < 50; ++k) {
        t.tick(true, true);  // held: nothing more
    }
    CHECK(c.line.writes.size() == 2U);
    t.tick(true, false);  // the release is not a press
    CHECK(c.line.writes.size() == 2U);
    t.tick(true, true);  // the second press
    CHECK(c.line.writes == (Levels{kClosed, kOpen, kClosed}));
    // Ten quick taps (one tick down, one tick up) are ten flips, alternating.
    for (int k = 0; k < 10; ++k) {
        t.tick(true, false);
        t.tick(true, true);
    }
    REQUIRE(c.line.writes.size() == 13U);
    for (std::size_t i = 1; i < c.line.writes.size(); ++i) {
        CAPTURE(i);
        CHECK(c.line.writes[i] != c.line.writes[i - 1]);
    }
}

// ═══ T2. A flip is the opposite of what the line was last COMMANDED ═════════════════
TEST_CASE("B2 T2: a press flips what the line was LAST COMMANDED — even when something else "
          "commanded it since the last press") {
    // BUG CAUGHT: a binding that keeps its own copy of "open" — after an end-of-run guard (or an
    // operation) closed the claw, the next press would command closed AGAIN, and the driver would
    // see a press that did nothing, then have to press twice.
    Claw c{"claw", /*safe=*/kClosed};
    ToggleBinding t{c.mech, ControllerButton::A, kClosed};
    t.enter();
    t.tick(true, false);
    t.tick(true, true);  // open
    t.tick(true, false);
    REQUIRE(c.mech.commanded() == kOpen);
    c.mech.applySafeState();  // something else closes it: the declared safe level
    REQUIRE(c.line.writes == (Levels{kClosed, kOpen, kClosed}));
    t.tick(true, true);  // the next press must OPEN it
    CHECK(c.line.writes.back() == kOpen);
    CHECK(c.line.writes.size() == 4U);
    // And the other way: something opens it while the binding believes it closed.
    t.tick(true, false);
    t.tick(true, true);  // closed
    c.mech.set(kOpen);
    t.tick(true, false);
    t.tick(true, true);
    CHECK(c.line.writes.back() == kClosed);
}

// ═══ T3. The initial level: stated, sent once in the binding's life ═════════════════
static_assert(!std::is_constructible_v<ToggleBinding, PneumaticMechanism&, ControllerButton>,
              "the initial level has NO default: a ToggleBinding without it must not compile");
static_assert(std::is_constructible_v<ToggleBinding, PneumaticMechanism&, ControllerButton, bool>);
static_assert(!std::is_copy_constructible_v<ToggleBinding>);
static_assert(!std::is_move_constructible_v<ToggleBinding>);

TEST_CASE("B2 T3: the initial level is the STATED one, sent at the FIRST enter() and never at "
          "construction, on a tick, or on a later enter()") {
    // BUG CAUGHT: a default level (the static_asserts above — the property is a signature, so it
    // is pinned at compile time); construction that commands (a claw that moves when the program
    // builds its bindings); a level re-sent every tick or on every entry (a claw that closes on
    // the re-enable after a field pause — the thing "a disable commands nothing" exists to stop).
    for (const bool stated : {kOpen, kClosed}) {
        CAPTURE(stated);
        Claw c{"claw"};
        ToggleBinding t{c.mech, ControllerButton::A, stated};
        CHECK(c.line.writes.empty());  // built, not commanded
        CHECK_FALSE(t.entered());
        CHECK(t.initialState() == stated);
        CHECK(t.button() == ControllerButton::A);
        CHECK(&t.mechanism() == &c.mech);
        t.enter();
        CHECK(c.line.writes == Levels{stated});  // the STATED level, not a default
        CHECK(t.entered());
        for (int k = 0; k < 30; ++k) {
            t.tick(true, false);
        }
        t.enter();  // PROS calls opcontrol() again after a field disable
        t.enter();
        for (int k = 0; k < 5; ++k) {
            t.tick(true, false);
        }
        CHECK(c.line.writes.size() == 1U);  // once in its life
    }
    // A button byte that is not one of the twelve is refused, and refusing commands nothing.
    Claw c{"claw"};
    CHECK_THROWS_AS(ToggleBinding(c.mech, static_cast<ControllerButton>(12), kClosed),
                    PreconditionError);
    CHECK_THROWS_AS(ToggleBinding(c.mech, static_cast<ControllerButton>(-1), kClosed),
                    PreconditionError);
    CHECK_NOTHROW(ToggleBinding(c.mech, ControllerButton::X, kClosed));
    CHECK(c.line.writes.empty());
}

// ═══ T4. A press must be SEEN ══════════════════════════════════════════════════════
TEST_CASE("B2 T4: nothing flips while disconnected, a press made while the controller was gone is "
          "never replayed, and a button already down at entry or at a reconnect is not a press") {
    // BUG CAUGHT: trusting a level handed over while disconnected (a lying controller flips the
    // claw with nobody at the controls); an edge detector that remembers "up" across a drop, so the
    // reconnect with the button down replays the press it could not see (the claw opens on what it
    // holds when a cable blips); a button held into driver control flipping at the first tick.
    SUBCASE("the lie: A 'pressed' while disconnected, and still down on the reconnect") {
        Claw c{"claw"};
        ToggleBinding t{c.mech, ControllerButton::A, kClosed};
        t.enter();
        t.tick(true, false);  // seen UP before the drop — the case that would replay
        for (int k = 0; k < 10; ++k) {
            t.tick(false, true);  // the caller hands over TRUE while disconnected
        }
        CHECK(c.line.writes.size() == 1U);
        t.tick(true, true);  // reconnected, A still down: not a press
        t.tick(true, true);
        CHECK(c.line.writes.size() == 1U);
        t.tick(true, false);  // seen up…
        t.tick(true, true);   // …then down: THAT is a press
        CHECK(c.line.writes == (Levels{kClosed, kOpen}));
    }
    SUBCASE("WHAT THE GUARANTEE RESTS ON: the first connected tick's level is taken as live") {
        // B2 review round 1, finding 1 — pinned so that any change to it is deliberate, NOT
        // because it is safe. The binding has only `connected` to tell an unknown level from a
        // real one. If the controller reported itself connected one tick BEFORE its buttons read
        // live — a stale "up" while the driver still holds A — that "up" is believed, and the next
        // live "down" is a press nobody made: the claw flips. Whether a V5 controller ever does
        // that (the connected flag and the button data changing together) is unmeasured and not
        // yet in the A4 register; the header and the docs say "provided".
        Claw c{"claw"};
        ToggleBinding t{c.mech, ControllerButton::A, kClosed};
        t.enter();
        t.tick(true, false);
        t.tick(false, false);  // the cable goes; the driver's finger goes down on A meanwhile
        t.tick(true, false);   // connected again, but A reads a STALE up (the case in question)
        CHECK(c.line.writes == Levels{kClosed});
        t.tick(true, true);  // the live level: A was held the whole time — and it flips
        CHECK(c.line.writes == (Levels{kClosed, kOpen}));
    }
    SUBCASE("pressed AND released while gone: nothing to replay, and the next press counts") {
        Claw c{"claw"};
        ToggleBinding t{c.mech, ControllerButton::A, kClosed};
        t.enter();
        t.tick(true, false);
        t.tick(false, true);
        t.tick(false, false);
        t.tick(true, false);  // back, button up
        CHECK(c.line.writes.size() == 1U);
        t.tick(true, true);  // the first press after the reconnect tick flips at once
        CHECK(c.line.writes.size() == 2U);
    }
    SUBCASE("down at loop entry: held, not pressed") {
        Claw c{"claw"};
        ToggleBinding t{c.mech, ControllerButton::A, kClosed};
        t.enter();
        for (int k = 0; k < 5; ++k) {
            t.tick(true, true);
        }
        CHECK(c.line.writes.size() == 1U);
        t.tick(true, false);
        t.tick(true, true);
        CHECK(c.line.writes.size() == 2U);
    }
    SUBCASE("ticked before any enter(): a button down on the very first tick is still held") {
        // The constructor seeds the edge "held" too, so a binding ticked by code that skipped
        // enter() (not the loop, which always enters first) cannot flip on a finger it never saw
        // arrive — and, never entered, it has sent no initial level either.
        Claw c{"claw"};
        ToggleBinding t{c.mech, ControllerButton::A, kClosed};
        t.tick(true, true);
        t.tick(true, true);
        CHECK(c.line.writes.empty());
        t.tick(true, false);
        t.tick(true, true);  // seen up, then down: a press
        CHECK(c.line.writes == Levels{kOpen});  // the opposite of the (never-driven) line's level
    }
    SUBCASE("pressed while the loop was not running, and down at the re-entry") {
        Claw c{"claw"};
        ToggleBinding t{c.mech, ControllerButton::A, kClosed};
        t.enter();
        t.tick(true, false);
        t.tick(true, true);  // open
        t.tick(true, false);  // seen UP last — the case a stale edge would replay
        t.enter();            // the loop left (a field pause) and came back; A went down meanwhile
        t.tick(true, true);
        CHECK(c.line.writes == (Levels{kClosed, kOpen}));
        t.tick(true, false);
        t.tick(true, true);  // a seen press after the re-entry still flips
        CHECK(c.line.writes == (Levels{kClosed, kOpen, kClosed}));
    }
}

// ═══ T5. A claimed mechanism is never commanded ═════════════════════════════════════
TEST_CASE("B2 T5: a claimed mechanism is never commanded — not the initial level at a claimed first "
          "entry (nor later), not a press — and a press under a claim is DROPPED, never carried "
          "over") {
    // BUG CAUGHT: a toggle that fights an operation for the valve; one that sends its start level
    // late, the moment a claim ends (a movement nobody pressed for); one that stores a press seen
    // under a claim and fires it after; one that takes a claim of its own.
    Claw c{"claw"};
    ToggleBinding t{c.mech, ControllerButton::A, kOpen};
    REQUIRE(c.mech.tryClaim());
    t.enter();
    CHECK(c.line.writes.empty());
    CHECK(t.entered());  // the one chance to send it has passed
    c.mech.releaseClaim();
    t.tick(true, false);  // A seen UP with nobody claiming the valve…
    REQUIRE(c.mech.tryClaim());  // …then an operation takes it…
    t.tick(true, true);          // …and A goes down under the claim
    CHECK(c.line.writes.empty());
    c.mech.releaseClaim();
    for (int k = 0; k < 5; ++k) {
        t.tick(true, true);  // the claim is gone; the finger is not: nothing carried over
    }
    t.enter();  // and a later entry never sends the start level late
    CHECK(c.line.writes.empty());
    // (Why the up-tick is taken with nobody claiming: a toggle that skipped the edge during a
    // claim — mutant MT10 — would never see an up-tick taken UNDER the claim, so the entry's
    // "held" seed would still be in place and would hide the carry-over this case exists for.)
    t.tick(true, false);
    t.tick(true, true);  // a new, seen press: flips what the line was last told (off)
    CHECK(c.line.writes == Levels{true});
    CHECK_FALSE(c.mech.claimed());  // the binding never took one
}

// ═══ T6. validBindings() — hold bindings and toggles as ONE set ═════════════════════
TEST_CASE("B2 T6: validBindings() over both kinds refuses a toggle on the re-zero button, a toggle "
          "sharing a button with a pivot or another toggle, a claw toggled twice and a null — by "
          "name") {
    Pivot six{"claw pivot 6"};
    Pivot seven{"claw pivot 7"};
    Claw claw{"claw"};
    MechanismBinding a{six.mech, ControllerButton::R1, ControllerButton::R2, kFull};
    MechanismBinding b{seven.mech, ControllerButton::L1, ControllerButton::L2, kFull};
    ToggleBinding onA{claw.mech, ControllerButton::A, kClosed};
    std::array<MechanismBinding*, 2> holds{&a, &b};
    char why[192] = "stale";
    const auto verdict = [&](std::span<ToggleBinding* const> toggles, ControllerButton rezero) {
        const bool ok = validBindings(holds, toggles, rezero, why, sizeof why);
        return ok ? std::string{} : std::string{why};
    };

    SUBCASE("robot two's set — pivots on R1/R2 and L1/L2, the claw on A, the re-zero on Y — is "
            "valid") {
        std::array<ToggleBinding*, 1> toggles{&onA};
        CHECK(verdict(toggles, ControllerButton::Y).empty());
        CHECK(std::strlen(why) == 0);
        CHECK(validBindings(holds, toggles, ControllerButton::Y));  // the no-buffer form
        CHECK(validBindings(std::span<MechanismBinding* const>{},
                            std::span<ToggleBinding* const>{}, ControllerButton::L1));
    }
    SUBCASE("the claw on the re-zero button — A, G1 Part 2b's re-zero before the move to Y") {
        std::array<ToggleBinding*, 1> toggles{&onA};
        const std::string w = verdict(toggles, ControllerButton::A);
        CHECK(w.find("button A toggles mechanism 'claw'") != std::string::npos);
        CHECK(w.find("re-zero") != std::string::npos);
    }
    SUBCASE("the claw on a pivot's button") {
        ToggleBinding onR2{claw.mech, ControllerButton::R2, kClosed};
        std::array<ToggleBinding*, 1> toggles{&onR2};
        const std::string w = verdict(toggles, ControllerButton::Y);
        CHECK(w.find("button R2 drives both mechanism 'claw pivot 6' and mechanism 'claw'")
              != std::string::npos);
    }
    SUBCASE("two claws on one button") {
        Claw other{"second claw"};
        ToggleBinding otherOnA{other.mech, ControllerButton::A, kClosed};
        std::array<ToggleBinding*, 2> toggles{&onA, &otherOnA};
        const std::string w = verdict(toggles, ControllerButton::Y);
        CHECK(w.find("button A drives both mechanism 'claw' and mechanism 'second claw'")
              != std::string::npos);
    }
    SUBCASE("one claw toggled twice, on two different buttons") {
        ToggleBinding onB{claw.mech, ControllerButton::B, kClosed};
        std::array<ToggleBinding*, 2> toggles{&onA, &onB};
        const std::string w = verdict(toggles, ControllerButton::Y);
        CHECK(w.find("mechanism 'claw' is toggled twice (toggle bindings 0 and 1)")
              != std::string::npos);
    }
    SUBCASE("a null toggle") {
        std::array<ToggleBinding*, 2> toggles{&onA, nullptr};
        CHECK(verdict(toggles, ControllerButton::Y).find("toggle binding 1 is null")
              != std::string::npos);
    }
    SUBCASE("the order: a hold binding's problem is named before a toggle's") {
        // L1 is pivot 7's forward button, the claw's button AND the re-zero: both kinds clash,
        // and the hold binding is the one reported — the order the header documents.
        ToggleBinding onL1{claw.mech, ControllerButton::L1, kClosed};
        std::array<ToggleBinding*, 1> toggles{&onL1};
        CHECK(verdict(toggles, ControllerButton::L1).find("drives mechanism 'claw pivot 7'")
              != std::string::npos);
        // With the hold binding off L1, the toggle's clash is the one reported.
        std::array<MechanismBinding*, 1> onlySix{&a};
        CHECK_FALSE(validBindings(onlySix, toggles, ControllerButton::L1, why, sizeof why));
        CHECK(std::string{why}.find("button L1 toggles mechanism 'claw'") != std::string::npos);
    }
    SUBCASE("the hold-only form is the same rules with no toggles") {
        CHECK(validBindings(holds, ControllerButton::A));
        CHECK_FALSE(validBindings(holds, ControllerButton::L2, why, sizeof why));
        CHECK(std::string{why}.find("button L2 drives mechanism 'claw pivot 7'")
              != std::string::npos);
    }
}

// ═══════════════════════════════ the toggle in THE LOOP ══════════════════════════════

namespace {

/// A controller that COUNTS every read and LIES while disconnected (it keeps reporting whatever
/// the test set), so a loop that polled it anyway is visible in the counters and on the line.
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

    std::array<double, 4> axes{};
    std::array<bool, 12> buttons{};
    bool connected = true;
    mutable int buttonReads = 0;
    mutable int readsWhileDisconnected = 0;
};

/// The re-zero on Y — the team lead's ruling of 2026-09-26, which freed A for the claw.
constexpr DriverLoopConfig kLoopCfg{.tickPeriod = Time{0.01}, .rezeroButton = ControllerButton::Y};

HeadingSample trusted() { return HeadingSample{.radians = 0.0, .trusted = true}; }

const shulib::kinematics::TankKinematics kTank{Length{11.0}};

/// What one tick left behind, read in perTick (after drive() and the bindings, before pace()).
struct LoopTick {
    double leftVolts = 0.0;
    double rightVolts = 0.0;
    ChassisSpeeds command{};
    std::size_t clawWrites = 0;
    bool clawLevel = false;
    std::size_t sixEvents = 0;
    std::size_t sevenEvents = 0;
    double sixVolts = 0.0;
};

/// Which runDriverControl signature a run uses.
enum class Loop { Both, HoldsOnly, NoBindings };

/// Robot two's driver controls: pivot 6 on R1/R2, pivot 7 on L1/L2 (12 V, Hold), and the claw on A
/// starting closed — around the loop, on the host test plant.
struct ClawControls {
    motion_rig::ChassisRig c{kTank};
    LyingController pad;
    DriveShaper shaper{DriverProfile{}};
    Pivot six{"claw pivot 6"};
    Pivot seven{"claw pivot 7"};
    Claw claw{"claw"};
    MechanismBinding bind6{six.mech, ControllerButton::R1, ControllerButton::R2, kFull};
    MechanismBinding bind7{seven.mech, ControllerButton::L1, ControllerButton::L2, kFull};
    ToggleBinding toggle{claw.mech, ControllerButton::A, kClosed};
    std::array<MechanismBinding*, 2> holds{&bind6, &bind7};
    std::array<ToggleBinding*, 1> toggles{&toggle};

    /// Run `ticks` ticks; `script(k)` runs at the TOP of tick k (inside the disabled predicate,
    /// which the loop polls first).
    std::vector<LoopTick> run(int ticks, const std::function<void(int)>& script,
                              Loop which = Loop::Both, const DriverLoopConfig& cfg = kLoopCfg) {
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
                                   .clawWrites = claw.line.writes.size(),
                                   .clawLevel = claw.line.level,
                                   .sixEvents = six.motor.events().size(),
                                   .sevenEvents = seven.motor.events().size(),
                                   .sixVolts = six.motor.commandedVoltage().value()});
            ++k;
        };
        switch (which) {
            case Loop::Both:
                runDriverControl(c.chassis, pad, c.pacer, shaper, cfg,
                                 std::span<MechanismBinding* const>{holds},
                                 std::span<ToggleBinding* const>{toggles}, disabled,
                                 [] { return trusted(); }, perTick);
                break;
            case Loop::HoldsOnly:
                runDriverControl(c.chassis, pad, c.pacer, shaper, cfg,
                                 std::span<MechanismBinding* const>{holds}, disabled,
                                 [] { return trusted(); }, perTick);
                break;
            case Loop::NoBindings:
                runDriverControl(c.chassis, pad, c.pacer, shaper, cfg, disabled,
                                 [] { return trusted(); }, perTick);
                break;
        }
        return log;
    }
};

}  // namespace

// ═══ TL1. The claw on A, in the loop ═══════════════════════════════════════════════
TEST_CASE("B2 TL1: the claw's initial level lands before the first poll; a press read at tick k is "
          "flipped IN tick k — after drive() and after the pivots; holding does nothing more; "
          "nothing re-sends the level") {
    // BUG CAUGHT: toggles never entered (the claw sits wherever it was); toggles ticked before
    // drive() or after pace() (a press slips a tick); a level re-sent each tick; the loop reading
    // A for the pivots or a pivot's button for the claw.
    ClawControls w;
    std::size_t writesAtFirstPoll = 99;
    std::vector<double> driveVoltsAtFlip;
    std::vector<std::size_t> sixEventsAtFlip;
    w.claw.line.onSet = [&](bool) {
        driveVoltsAtFlip.push_back(w.c.rig.h.motor(0).commandedVoltage().value());
        sixEventsAtFlip.push_back(w.six.motor.events().size());
    };
    const auto log = w.run(30, [&](int k) {
        if (k == 0) {
            writesAtFirstPoll = w.claw.line.writes.size();
        }
        w.pad.axes[1] = k >= 3 ? 1.0 : 0.0;  // the drive starts moving on the press tick
        w.pad.hold(ControllerButton::R1, k >= 3);  // …and so does pivot 6
        w.pad.hold(ControllerButton::A, (k >= 3 && k < 20) || (k >= 25 && k < 27));
    });
    REQUIRE(log.size() == 30U);
    CHECK(writesAtFirstPoll == 1U);
    CHECK(w.claw.line.writes.front() == kClosed);
    CHECK(log[2].clawWrites == 1U);
    CHECK(log[2].clawLevel == kClosed);
    CHECK(log[3].clawWrites == 2U);  // THE press tick
    CHECK(log[3].clawLevel == kOpen);
    for (std::size_t k = 3; k < 25; ++k) {
        CAPTURE(k);
        CHECK(log[k].clawWrites == 2U);  // held to 19, released 20..24: nothing more
    }
    CHECK(log[25].clawWrites == 3U);
    CHECK(log[25].clawLevel == kClosed);
    CHECK(log.back().clawWrites == 3U);
    CHECK(w.claw.line.writes.size() == 3U);  // and the disable added nothing
    // The press tick's flip landed after that tick's drive() had put volts on the drive, and after
    // pivot 6's first +12 V (entry safe = 2 events, then the drive = 3): drive, pivots, toggles.
    REQUIRE(driveVoltsAtFlip.size() == 3U);
    CHECK(driveVoltsAtFlip[1] > 1.0);
    CHECK(sixEventsAtFlip[1] == 3U);
    CHECK(log[3].sixVolts == 12.0);
    // Each connected tick read the re-zero once, each pivot's two buttons once, and A once.
    CHECK(w.pad.buttonReads == 30 * (1 + 4 + 1));
}

// ═══ TL2. A LYING disconnected controller ═══════════════════════════════════════════
TEST_CASE("B2 TL2: A 'pressed' on a disconnected controller is never read, never flips, and is "
          "not replayed on the reconnect — the next SEEN press is") {
    // BUG CAUGHT: the loop reading A while disconnected (a real V5 reads false then, HA-103, which
    // would hide it; this controller does not); a toggle replaying the press it could not see.
    ClawControls w;
    const auto log = w.run(20, [&](int k) {
        w.pad.connected = k < 4 || k >= 10;  // gone for ticks 4..9
        w.pad.hold(ControllerButton::A, (k >= 5 && k < 12) || k >= 14);
    });
    REQUIRE(log.size() == 20U);
    CHECK(w.pad.readsWhileDisconnected == 0);
    for (std::size_t k = 0; k < 14; ++k) {
        CAPTURE(k);
        CHECK(log[k].clawWrites == 1U);  // the entry level only — through the drop and the reconnect
    }
    CHECK(log[14].clawWrites == 2U);  // seen up at 12-13, down at 14: a press
    CHECK(log[14].clawLevel == kOpen);
}

// ═══ TL3. A field disable while the claw is OPEN ═══════════════════════════════════
TEST_CASE("B2 TL3: a disable while the claw is open commands NOTHING to it — it stays open — while "
          "both pivots are made safe in the same tick; the next entry commands nothing either") {
    // BUG CAUGHT: the disable path making toggles safe too (the claw closes — or, with the other
    // plumbing, opens — on every field disable and drops what it holds); a re-entry that re-sends
    // the initial level (the claw snaps closed on the re-enable); a finger held across the pause
    // read as a new press. (This pins the LIBRARY's half only: what the brain does with the line
    // while disabled is A4 register HA-157, which no host test can settle.)
    ClawControls w;
    const auto log = w.run(6, [&](int k) {
        w.pad.hold(ControllerButton::A, k >= 2 && k < 4);  // pressed at tick 2, UP from tick 4
        w.pad.hold(ControllerButton::R1, true);
        w.pad.hold(ControllerButton::L2, true);
    });
    REQUIRE(log.size() == 6U);
    REQUIRE(log.back().clawLevel == kOpen);
    REQUIRE(log.back().clawWrites == 2U);
    // The disable tick: each pivot got exactly its safe state; the claw got nothing.
    CHECK(w.six.motor.events().size() == log.back().sixEvents + 2);
    CHECK(isSafeAt(w.six.motor.events(), log.back().sixEvents));
    CHECK(w.seven.motor.events().size() == log.back().sevenEvents + 2);
    CHECK(w.claw.line.writes == (Levels{kClosed, kOpen}));
    CHECK(w.claw.mech.commanded() == kOpen);
    // PROS calls opcontrol() again. A went DOWN during the pause (it was last seen up): a press
    // the loop could not see, which must not be replayed at the re-entry.
    const auto again = w.run(4, [&](int k) { w.pad.hold(ControllerButton::A, k < 2); });
    REQUIRE(again.size() == 4U);
    CHECK(w.claw.line.writes == (Levels{kClosed, kOpen}));  // no entry level, no phantom flip
    const auto third = w.run(2, [&](int k) { w.pad.hold(ControllerButton::A, k == 1); });
    REQUIRE(third.size() == 2U);
    CHECK(w.claw.line.writes == (Levels{kClosed, kOpen, kClosed}));  // a seen press still works
}

// ═══ TL4. The kinds never touch each other's mechanisms; the drive never notices ══════
TEST_CASE("B2 TL4: A touches only the claw, the pivot buttons only their pivots — and the drive's "
          "volts are BIT-IDENTICAL with both kinds, with hold bindings only, and with none") {
    // BUG CAUGHT: a toggle ticked with a pivot's button (or a pivot with A); a toggle that pokes a
    // hold mechanism; a loop that re-orders drive() around either kind.
    SUBCASE("A alone: the pivots see only their entry safe state") {
        ClawControls w;
        w.run(40, [&](int k) { w.pad.hold(ControllerButton::A, (k / 3) % 2 == 1); });
        CHECK(w.claw.line.writes.size() > 5U);
        CHECK(w.six.motor.events().size() == 4U);  // entry safe + the disable's
        CHECK(w.seven.motor.events().size() == 4U);
        CHECK_FALSE(w.six.motor.everEnergized());
        CHECK_FALSE(w.seven.motor.everEnergized());
    }
    SUBCASE("the pivot buttons alone: the claw sees only its entry level") {
        ClawControls w;
        w.run(40, [&](int k) {
            w.pad.hold(ControllerButton::R1, (k / 4) % 2 == 1);
            w.pad.hold(ControllerButton::L2, (k / 5) % 2 == 1);
            w.pad.hold(ControllerButton::R2, k % 7 == 0);
        });
        CHECK(w.six.motor.everEnergized());
        CHECK(w.seven.motor.everEnergized());
        CHECK(w.claw.line.writes == Levels{kClosed});
    }
    SUBCASE("the drive: one script, three signatures, identical bits") {
        const auto script = [](ClawControls& w) {
            return [&w](int k) {
                w.pad.axes[1] = std::sin(0.11 * k);
                w.pad.axes[2] = 0.4 * std::cos(0.05 * k);
                w.pad.hold(ControllerButton::A, (k / 4) % 2 == 1);
                w.pad.hold(ControllerButton::R1, (k / 6) % 2 == 0);
                w.pad.hold(ControllerButton::L2, k % 9 < 3);
                w.pad.connected = k % 19 != 18;
            };
        };
        ClawControls both;
        ClawControls holdsOnly;
        ClawControls none;
        const auto a = both.run(120, script(both), Loop::Both);
        const auto b = holdsOnly.run(120, script(holdsOnly), Loop::HoldsOnly);
        const auto n = none.run(120, script(none), Loop::NoBindings);
        REQUIRE(a.size() == 120U);
        REQUIRE(b.size() == 120U);
        REQUIRE(n.size() == 120U);
        int moving = 0;
        for (std::size_t k = 0; k < a.size(); ++k) {
            CAPTURE(k);
            CHECK(sameBits(a[k].leftVolts, n[k].leftVolts));
            CHECK(sameBits(a[k].rightVolts, n[k].rightVolts));
            CHECK(sameBits(b[k].leftVolts, n[k].leftVolts));
            CHECK(sameBits(a[k].command.vx().value(), n[k].command.vx().value()));
            CHECK(sameBits(a[k].command.omega().value(), n[k].command.omega().value()));
            // And the pivots behave identically whether or not a toggle rides beside them.
            CHECK(a[k].sixEvents == b[k].sixEvents);
            CHECK(sameBits(a[k].sixVolts, b[k].sixVolts));
            moving += std::abs(a[k].leftVolts) > 1.0 ? 1 : 0;
        }
        CHECK(moving > 40);                                // the drive really moved
        CHECK(both.claw.line.writes.size() > 5U);          // the claw really toggled
        CHECK(holdsOnly.claw.line.writes.empty());         // …and only where it was bound
        CHECK(both.six.motor.everEnergized());
    }
}

// ═══ TL5. The loop refuses a bad set BEFORE anything moves ══════════════════════════
TEST_CASE("B2 TL5: runDriverControl refuses a toggle on the re-zero button, or on a pivot's "
          "button — before the first poll, before the claw's initial level, before any pivot") {
    // BUG CAUGHT: a loop that starts anyway, so A both re-zeroes field-centric AND flips the claw;
    // a loop that sends the claw's initial level before it has checked the set.
    ClawControls w;
    int polled = 0;
    const auto never = [&](const ChassisSpeeds&, bool, const DriveShaperState&) {};
    const auto poll = [&] {
        ++polled;
        return polled > 3;  // a loop that failed to refuse still ends — red, not a hang
    };
    const DriverLoopConfig rezeroOnA{.tickPeriod = Time{0.01}, .rezeroButton = ControllerButton::A};
    CHECK_THROWS_AS(runDriverControl(w.c.chassis, w.pad, w.c.pacer, w.shaper, rezeroOnA,
                                     std::span<MechanismBinding* const>{w.holds},
                                     std::span<ToggleBinding* const>{w.toggles}, poll,
                                     [] { return trusted(); }, never),
                    PreconditionError);
    ToggleBinding onL2{w.claw.mech, ControllerButton::L2, kClosed};
    std::array<ToggleBinding*, 1> clash{&onL2};
    CHECK_THROWS_AS(runDriverControl(w.c.chassis, w.pad, w.c.pacer, w.shaper, kLoopCfg,
                                     std::span<MechanismBinding* const>{w.holds},
                                     std::span<ToggleBinding* const>{clash}, poll,
                                     [] { return trusted(); }, never),
                    PreconditionError);
    CHECK(polled == 0);
    CHECK(w.c.pacer.paces == 0);
    CHECK(w.claw.line.writes.empty());
    CHECK_FALSE(w.toggle.entered());
    CHECK(w.six.motor.eventCount() == 0);
    CHECK(w.seven.motor.eventCount() == 0);
}
