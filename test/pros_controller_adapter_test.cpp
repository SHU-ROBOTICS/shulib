// Adapter tests for ProsController + ButtonEdge THROUGH THE HOST SHIM (chunk
// R1a). The shim tests the adapter against our belief about PROS (HA-103/104);
// hardware tests the belief. The shim's get_digital_new_press really CONSUMES,
// so the two-consumer test fails on behaviour, not on a counter alone.

#include "doctest.h"

#include "pros/shim_control.hpp"

#include "shulib/hal/controller.hpp"
#include "shulib/hal/fake/fake_controller.hpp"
#include "shulib/hal/pros/controller.hpp"

using shulib::hal::ButtonEdge;
using shulib::hal::ControllerAxis;
using shulib::hal::ControllerButton;
using shulib::hal::fake::FakeController;
using shulib::hal::pros::ControllerId;
using shulib::hal::pros::ProsController;

namespace {
pros::shim::ControllerState& master() {
    return pros::shim::controllerState(pros::E_CONTROLLER_MASTER);
}
}  // namespace

TEST_CASE("ProsController: axis normalizes −127..127 → [−1,1] through the conversion (wired)") {
    // BUG CAUGHT (mutation M14 shape): raw counts returned as-is — the teleop
    // layer multiplies by the speed budget, so raw 127 would command 127× the
    // budget. Hand-computed: 64/127 = 0.5039370078740157.
    pros::shim::resetAll();
    ProsController c{ControllerId::Master};
    master().analog[static_cast<std::size_t>(pros::E_CONTROLLER_ANALOG_LEFT_Y)] = 127;
    master().analog[static_cast<std::size_t>(pros::E_CONTROLLER_ANALOG_RIGHT_X)] = 64;
    master().analog[static_cast<std::size_t>(pros::E_CONTROLLER_ANALOG_LEFT_X)] = -127;
    CHECK(c.axis(ControllerAxis::LeftY) == doctest::Approx(1.0));
    CHECK(c.axis(ControllerAxis::RightX) == doctest::Approx(0.5039370078740157));
    CHECK(c.axis(ControllerAxis::LeftX) == doctest::Approx(-1.0));
    CHECK(c.axis(ControllerAxis::RightY) == doctest::Approx(0.0));
}

TEST_CASE("ProsController: buttons are LEVELS via get_digital — new_press is NEVER called (trap C)") {
    // BUG CAUGHT (mutation M10): binding get_digital_new_press() — it consumes
    // the event, so with two consumers one silently loses (proven below).
    // Beyond the behavioural test, the call count pins the binding directly.
    pros::shim::resetAll();
    ProsController c{ControllerId::Master};
    master().digital[static_cast<std::size_t>(pros::E_CONTROLLER_DIGITAL_A)] = true;
    CHECK(c.pressed(ControllerButton::A));
    CHECK(c.pressed(ControllerButton::A));  // a level: still true on the second read
    CHECK_FALSE(c.pressed(ControllerButton::B));
    CHECK(master().newPressCalls == 0);
}

TEST_CASE("ButtonEdge: TWO consumers of the same press both see it (the reason edges live above the seam)") {
    // BUG CAUGHT (mutation M10, the two-consumer visibility): with PROS's
    // consuming read, the second consumer misses every press — an auton-select
    // UI and a mechanism toggle sharing button A would fight invisibly. With
    // level reads + per-consumer ButtonEdge, both see the rising edge.
    pros::shim::resetAll();
    ProsController c{ControllerId::Master};
    ButtonEdge uiEdge;
    ButtonEdge mechEdge;

    // Button up: no edges.
    CHECK_FALSE(uiEdge.update(c.pressed(ControllerButton::A)));
    CHECK_FALSE(mechEdge.update(c.pressed(ControllerButton::A)));

    // Press: BOTH consumers see the rising edge on their next update.
    master().digital[static_cast<std::size_t>(pros::E_CONTROLLER_DIGITAL_A)] = true;
    CHECK(uiEdge.update(c.pressed(ControllerButton::A)));
    CHECK(mechEdge.update(c.pressed(ControllerButton::A)));

    // Held: an edge fires once, not per tick.
    CHECK_FALSE(uiEdge.update(c.pressed(ControllerButton::A)));
    CHECK_FALSE(mechEdge.update(c.pressed(ControllerButton::A)));

    // Release and re-press: fires again.
    master().digital[static_cast<std::size_t>(pros::E_CONTROLLER_DIGITAL_A)] = false;
    CHECK_FALSE(uiEdge.update(c.pressed(ControllerButton::A)));
    master().digital[static_cast<std::size_t>(pros::E_CONTROLLER_DIGITAL_A)] = true;
    CHECK(uiEdge.update(c.pressed(ControllerButton::A)));

    // Control experiment — the shim's CONSUMING read really does starve a
    // second consumer (documents WHY the adapter must not bind it):
    pros::shim::resetAll();
    pros::v5::Controller raw{pros::E_CONTROLLER_MASTER};
    master().digital[static_cast<std::size_t>(pros::E_CONTROLLER_DIGITAL_B)] = true;
    CHECK(raw.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B) == 1);  // consumer 1 wins
    CHECK(raw.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B) == 0);  // consumer 2 starves
}

TEST_CASE("ProsController: isConnected is the positive validity signal (HA-103)") {
    // BUG CAUGHT: dropping the signal — a disconnected controller reads 0 on
    // every channel, indistinguishable from centred sticks, and the teleop
    // loop would keep driving on a dead controller instead of braking.
    pros::shim::resetAll();
    ProsController c{ControllerId::Master};
    CHECK(c.isConnected());
    master().analog[static_cast<std::size_t>(pros::E_CONTROLLER_ANALOG_LEFT_Y)] = 100;
    master().connected = false;
    CHECK_FALSE(c.isConnected());
    CHECK(c.axis(ControllerAxis::LeftY) == doctest::Approx(0.0));  // the ambiguous zero
}

TEST_CASE("ProsController: master and partner are two independent devices (VEX U's two drivers)") {
    // BUG CAUGHT: both adapters hard-wired to MASTER — the partner's inputs
    // silently drive nothing, discovered mid-match when the second driver's
    // controls do nothing.
    pros::shim::resetAll();
    ProsController m{ControllerId::Master};
    ProsController p{ControllerId::Partner};
    auto& partner = pros::shim::controllerState(pros::E_CONTROLLER_PARTNER);
    partner.analog[static_cast<std::size_t>(pros::E_CONTROLLER_ANALOG_LEFT_Y)] = 127;
    CHECK(p.axis(ControllerAxis::LeftY) == doctest::Approx(1.0));
    CHECK(m.axis(ControllerAxis::LeftY) == doctest::Approx(0.0));
}

TEST_CASE("FakeController: mirrors the disconnected-reads-zero behaviour (HA-103)") {
    // BUG CAUGHT: the fake diverging from the adapter's semantics — a teleop
    // test green against the fake and wrong against the adapter (the C7
    // fake/adapter divergence class). Same scenario, same numbers, both seams.
    FakeController f;
    f.setAxis(ControllerAxis::LeftY, 0.75);
    f.setPressed(ControllerButton::A, true);
    CHECK(f.axis(ControllerAxis::LeftY) == doctest::Approx(0.75));
    CHECK(f.pressed(ControllerButton::A));
    f.setConnected(false);
    CHECK_FALSE(f.isConnected());
    CHECK(f.axis(ControllerAxis::LeftY) == doctest::Approx(0.0));
    CHECK_FALSE(f.pressed(ControllerButton::A));
}
