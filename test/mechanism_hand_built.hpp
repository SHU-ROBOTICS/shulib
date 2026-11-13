#pragma once
//
// mechanism_hand_built.hpp — ENTRIES BUILT BY HAND, each breaking exactly ONE per-entry wiring rule
// (mechanism::entryProblem), for THE MIRROR'S SECOND DIRECTION (chunk MECH-A1's review round,
// 2026-10-15; the MECH brief's §3.1: "wiringProblem and mechanisms() refuse the same inputs (the
// mirror, both directions)").
//
// A `Wired` entry need not come from wire(...): a tool or a test can build one by hand (D17), and it
// then never passed wire()'s checks. Each entry below starts from a valid wire(...) of a fixture
// robot and breaks ONE field. The SAME constant is read twice:
//   * the host suite (test/mechanism_wiring_test.cpp) proves entryProblem() names it and
//     wiringProblem() refuses it with its sentence — at run time;
//   * the compile-refusal harness (test/compile_refusals/hb_*.cpp) proves mechanisms() REFUSES it at
//     build, by the name wire() uses for the same mistake.
// So no hand-built entry can walk around the rules a declaration is held to, and the two halves can
// never be fed different inputs. kArmByHand and kFlywheelByHand are the POSITIVE pair: two different
// entries with no declaration behind them (`declared` nullptr) are two mechanisms, not "the same
// mechanism wired twice".

#include "mechanism_fixtures.hpp"

namespace mech_fixture::hand_built {
using namespace shulib::mechanism::words;
namespace m = shulib::mechanism;

/// `w` with `edit` applied — at build.
template <class Edit>
consteval m::Wired edited(m::Wired w, Edit edit) {
    edit(w);
    return w;
}

// ── each breaks ONE rule, in entryProblem()'s order ─────────────────────────────────────────────
/// NameDiffers: the wiring carries another mechanism's name.
inline constexpr m::Wired kNameDiffers = edited(wire(robot_x::arm, motors(1)), [](m::Wired& w) { w.wiring.mechanism[0] = 'b'; });
/// MotorRoleUnbound: the lift's second motor role on port 0.
inline constexpr m::Wired kMotorUnbound =
    edited(wire(robot_x::lift, motors(2, 3), homeSwitch('B')), [](m::Wired& w) { w.wiring.motors[1] = 0; });
/// SmartPortPast21, through a motor role.
inline constexpr m::Wired kMotorPort22 = edited(wire(robot_x::arm, motors(1)), [](m::Wired& w) { w.wiring.motors[0] = 22; });
/// SmartPortPast21, through the angle sensor.
inline constexpr m::Wired kAnglePort22 =
    edited(wire(robot_y::wrist, motors(1), angleSensor(10)), [](m::Wired& w) { w.wiring.angleSensor = 22; });
/// MotorPortExtra: a second motor port on the one-motor arm.
inline constexpr m::Wired kMotorExtra = edited(wire(robot_x::arm, motors(1)), [](m::Wired& w) { w.wiring.motors[1] = 7; });
/// LineRoleUnbound: the hood's second line unbound.
inline constexpr m::Wired kLineUnbound =
    edited(wire(robot_x::hood, lines('C', 'D')), [](m::Wired& w) { w.wiring.lines[1] = m::Line::None; });
/// LinePastH, through a line role.
inline constexpr m::Wired kLine9 =
    edited(wire(robot_x::hood, lines('C', 'D')), [](m::Wired& w) { w.wiring.lines[0] = static_cast<m::Line>(9); });
/// LinePastH, through the home switch.
inline constexpr m::Wired kHomeLine9 = edited(wire(robot_x::lift, motors(2, 3), homeSwitch('B')),
                                              [](m::Wired& w) { w.wiring.homeSwitch = static_cast<m::Line>(9); });
/// LineExtra: a second line on the one-line claw.
inline constexpr m::Wired kLineExtra =
    edited(wire(robot_two::claw, lines('A')), [](m::Wired& w) { w.wiring.lines[1] = m::Line::B; });
/// LevelsVsLines: one of the hood's positions states one level on its two lines (U2).
inline constexpr m::Wired kLevelsVsLines =
    edited(wire(robot_x::hood, lines('C', 'D')), [](m::Wired& w) { w.spec.pneumatic.positions[1].levelCount = 1; });
/// AngleSensorUnplugged: the wrist's angle sensor unbound.
inline constexpr m::Wired kAngleUnplugged =
    edited(wire(robot_y::wrist, motors(1), angleSensor(10)), [](m::Wired& w) { w.wiring.angleSensor = 0; });
/// AngleSensorExtra: an angle sensor port on the arm, which has none.
inline constexpr m::Wired kAngleExtra = edited(wire(robot_x::arm, motors(1)), [](m::Wired& w) { w.wiring.angleSensor = 10; });
/// HomeSwitchUnplugged: the lift's home switch unbound.
inline constexpr m::Wired kHomeUnplugged = edited(wire(robot_x::lift, motors(2, 3), homeSwitch('B')),
                                                  [](m::Wired& w) { w.wiring.homeSwitch = m::Line::None; });
/// HomeSwitchExtra: a home switch line on the arm, which has none.
inline constexpr m::Wired kHomeExtra = edited(wire(robot_x::arm, motors(1)), [](m::Wired& w) { w.wiring.homeSwitch = m::Line::E; });
/// PieceSensorUnplugged: the intake's distance sensor unbound.
inline constexpr m::Wired kPieceUnplugged =
    edited(wire(robot_x::intake, motors(5), pieceSensor(9)), [](m::Wired& w) { w.wiring.pieceSensorPort = 0; });
/// PieceSensorExtra: a piece sensor port on the arm, which has none.
inline constexpr m::Wired kPieceExtra = edited(wire(robot_x::arm, motors(1)), [](m::Wired& w) { w.wiring.pieceSensorPort = 8; });
/// PieceSwitchUnplugged: the catapult's limit-switch piece sensor unbound.
inline constexpr m::Wired kPieceSwitchUnplugged = edited(wire(robot_y::catapult, motors(7), pieceSensor('E')),
                                                         [](m::Wired& w) { w.wiring.pieceSwitch = m::Line::None; });
/// PieceSwitchExtra: a piece switch line on the arm, which has none.
inline constexpr m::Wired kPieceSwitchExtra =
    edited(wire(robot_x::arm, motors(1)), [](m::Wired& w) { w.wiring.pieceSwitch = m::Line::F; });

// ── the POSITIVE pair: valid, and with no declaration behind them ───────────────────────────────
/// robot x's arm, its declaration forgotten.
inline constexpr m::Wired kArmByHand = edited(wire(robot_x::arm, motors(1)), [](m::Wired& w) { w.declared = nullptr; });
/// robot x's flywheel, its declaration forgotten.
inline constexpr m::Wired kFlywheelByHand =
    edited(wire(robot_x::flywheel, motors(4)), [](m::Wired& w) { w.declared = nullptr; });
}  // namespace mech_fixture::hand_built
