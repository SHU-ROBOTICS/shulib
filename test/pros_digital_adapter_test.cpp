// Adapter tests for ProsDigitalOut / ProsDigitalIn THROUGH THE HOST SHIM
// (chunk R1b), plus the FakeDigitalIn contract. The shim tests the adapters
// against our belief about PROS (HA-119/120/121); hardware tests the belief.
// What only these can prove: construction actuates with the STATED state, the
// bool→1/0 wiring is right, commanded() reports the command and not the
// world, the level binding survives two consumers, and the PROS_ERR screen
// never reads a dead port as pressed.

#include "doctest.h"

#include <array>

#include "pros/shim_control.hpp"

#include "shulib/hal/controller.hpp"  // ButtonEdge — reused above the seam
#include "shulib/hal/fake/fake_digital_in.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/pros/digital_in.hpp"
#include "shulib/hal/pros/digital_out.hpp"

using shulib::hal::ButtonEdge;
using shulib::hal::IDigitalOut;
using shulib::hal::PneumaticMechanism;
using shulib::hal::fake::FakeDigitalIn;
using shulib::hal::pros::ProsDigitalIn;
using shulib::hal::pros::ProsDigitalOut;

TEST_CASE("ProsDigitalOut: CONSTRUCTION ACTUATES — the stated initial state is driven, once (T3)") {
    // BUG CAUGHT: the required initialState argument ignored (PROS's LOW
    // default sneaking back in) — on a clamp whose safe state is HIGH, boot
    // would physically fling the cylinder low then high. The write history
    // must be exactly {1}: the stated state, driven once, no glitch through
    // the unsafe value.
    pros::shim::resetAll();
    ProsDigitalOut out{'a', true};
    const auto& line = pros::shim::adiLine(0, 'a');
    REQUIRE(line.writes.size() == 1);
    CHECK(line.writes[0] == 1);
    CHECK(line.value == 1);
    CHECK(out.commanded() == true);
}

TEST_CASE("ProsDigitalOut: set() drives the line 1/0 (wired, not inverted)") {
    // BUG CAUGHT (mutation 7): set() inverted — every clamp command
    // backwards: "close on the goal" opens, at the exact moment the routine
    // advances because commanded() still reports what it was told.
    pros::shim::resetAll();
    ProsDigitalOut out{2, false};
    auto& line = pros::shim::adiLine(0, 2);
    CHECK(line.value == 0);

    out.set(true);
    CHECK(line.value == 1);
    CHECK(out.commanded() == true);

    out.set(false);
    CHECK(line.value == 0);
    CHECK(out.commanded() == false);
}

TEST_CASE("ProsDigitalOut: commanded() reports the COMMAND, never the world (refused write visible)") {
    // BUG CAUGHT (mutation 8): commanded() re-reading the device — the seam
    // documents there IS no feedback channel (digital_out.hpp:44), so a
    // "readback of the world" is a lie that happens to agree until the port
    // refuses. Here the port refuses (PROS_ERR): the line stays put, but
    // commanded() must report the intent — that DIVERGENCE is what makes the
    // refusal visible, counted in faultedWrites().
    pros::shim::resetAll();
    ProsDigitalOut out{4, false};
    auto& line = pros::shim::adiLine(0, 4);
    line.broken = true;

    out.set(true);
    CHECK(line.value == 0);            // the wire refused
    CHECK(out.commanded() == true);    // the command, reported anyway
    CHECK(out.faultedWrites() == 1);
}

TEST_CASE("ProsDigitalOut: expander form addresses {smartPort, adiPort}, letters normalize (T6/HA-120)") {
    // BUG CAUGHT: the expander ctor wired to the brain's own ADI (dropping
    // the smart port) — the pneumatic fires on the wrong 3-wire bank; or
    // letter ports mis-normalized ('c' is port 3, vendored adi.hpp:59).
    pros::shim::resetAll();
    ProsDigitalOut out{static_cast<std::uint8_t>(3), static_cast<std::uint8_t>('c'), true};
    CHECK(pros::shim::adiLine(3, 'c').value == 1);
    CHECK(pros::shim::adiLine(3, 3).value == 1);   // 'c' ≡ 3 — same line
    CHECK(pros::shim::adiLine(0, 'c').value == 0); // NOT the brain's own bank
}

TEST_CASE("ProsDigitalOut + PneumaticMechanism: the ctor state AGREES with the declared safe state (T3)") {
    // BUG CAUGHT: the boot-window mismatch — an adapter constructed LOW
    // under a mechanism whose declared safe state is HIGH means the cylinder
    // physically moves at boot and again at the first applySafeState(). The
    // pattern pinned here: ONE constant states the safe level, the adapter
    // ctor and the mechanism both take it, and the line's write history
    // never contains the unsafe value — construction to park, glitch-free.
    pros::shim::resetAll();
    constexpr bool kClampSafe = true;  // this robot's clamp: safe = closed
    ProsDigitalOut out{'h', kClampSafe};
    std::array<IDigitalOut*, 1> lines{&out};
    PneumaticMechanism clamp{lines, kClampSafe, "clamp"};

    clamp.applySafeState();
    const auto& line = pros::shim::adiLine(0, 'h');
    CHECK(line.value == 1);
    for (const auto driven : line.writes) {
        CHECK(driven == 1);  // never the unsafe value, ctor included
    }
    CHECK(clamp.commanded() == kClampSafe);
}

TEST_CASE("ProsDigitalIn: binds the LEVEL — two consumers both see it (never the consuming new_press)") {
    // BUG CAUGHT (mutation 9 — needs two consumers to be visible at all):
    // the adapter bound to get_new_press(), which CONSUMES the press
    // (HA-121). One consumer polling looks identical under both bindings;
    // with TWO (a homing routine and a telemetry page watching one switch),
    // the second silently misses every press. Both must read the level, as
    // often as they like.
    pros::shim::resetAll();
    ProsDigitalIn homing{1};
    ProsDigitalIn telemetry{1};
    auto& line = pros::shim::adiLine(0, 1);

    line.value = 1;
    CHECK(homing.state() == true);
    CHECK(telemetry.state() == true);  // the starved consumer under new_press
    CHECK(homing.state() == true);     // a LEVEL: re-reading does not consume
    CHECK(line.newPressCalls == 0);    // and the forbidden call never ran

    line.value = 0;
    CHECK(homing.state() == false);
    CHECK(telemetry.state() == false);
}

TEST_CASE("ProsDigitalIn: PROS_ERR never reads as pressed — held to last good (T7)") {
    // BUG CAUGHT (mutation 6's family): the screen removed — PROS_ERR
    // (INT32_MAX) != 0 would read TRUE, so a dead ADI port is a homing
    // switch permanently 'pressed': the lift believes it is homed while it
    // climbs into the hard stop.
    pros::shim::resetAll();
    ProsDigitalIn in{5};
    auto& line = pros::shim::adiLine(0, 5);

    // Dead from boot: must read released (the safe initial hold), not true.
    line.broken = true;
    CHECK(in.state() == false);
    CHECK(in.faultedReads() == 1);

    // Alive, pressed, then dies mid-run: hold the last good level.
    line.broken = false;
    line.value = 1;
    CHECK(in.state() == true);
    line.broken = true;
    CHECK(in.state() == true);  // held — not INT32_MAX!=0 logic, but last-good
    CHECK(in.faultedReads() == 2);
}

TEST_CASE("ProsDigitalIn: expander form and letter ports address the same lines (T6/HA-120)") {
    // BUG CAUGHT: the {smartPort, adiPort} ctor collapsing onto the brain's
    // bank, or 'A' failing to normalize — the homing switch reads a
    // different physical wire than the one the robot config names.
    pros::shim::resetAll();
    ProsDigitalIn onExpander{static_cast<std::uint8_t>(2), static_cast<std::uint8_t>('A')};
    pros::shim::adiLine(2, 1).value = 1;   // 'A' ≡ 1, expander on smart 2
    pros::shim::adiLine(0, 1).value = 0;   // brain's own port 1 stays low
    CHECK(onExpander.state() == true);
}

TEST_CASE("IDigitalIn + ButtonEdge: per-consumer edges over the level — N consumers, one press each (T2)") {
    // BUG CAUGHT: edge state migrating INTO the seam (a stateful "just
    // pressed" on the adapter) — exactly the consuming-read shape the seam
    // bans. Reusing ButtonEdge above the seam, two consumers each own their
    // edge and BOTH see the same physical press once.
    pros::shim::resetAll();
    ProsDigitalIn in{6};
    auto& line = pros::shim::adiLine(0, 6);
    ButtonEdge homingEdge;
    ButtonEdge loggerEdge;

    line.value = 1;
    CHECK(homingEdge.update(in.state()) == true);
    CHECK(loggerEdge.update(in.state()) == true);   // both consumers see it
    CHECK(homingEdge.update(in.state()) == false);  // held ≠ a second press
    line.value = 0;
    (void)homingEdge.update(in.state());
    (void)loggerEdge.update(in.state());
    line.value = 1;
    CHECK(homingEdge.update(in.state()) == true);   // a real second press
    CHECK(loggerEdge.update(in.state()) == true);
}

TEST_CASE("FakeDigitalIn: level round-trips and reads are counted") {
    // BUG CAUGHT: a fake that latches (state() consuming the level) or a
    // readCount that misses const reads — a consumer that forgot to poll and
    // one that polls every tick would look identical to a test.
    FakeDigitalIn in;
    CHECK(in.state() == false);  // default: released
    in.setState(true);
    CHECK(in.state() == true);
    CHECK(in.state() == true);   // a level, not an edge
    CHECK(in.readCount() == 3);
}
