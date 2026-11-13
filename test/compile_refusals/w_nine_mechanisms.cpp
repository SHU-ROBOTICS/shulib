// w_nine_mechanisms: nine mechanisms (a robot holds eight).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Power p1{.name = "p1", .motors = {Forward}, .cartridge = Red, .safe = Coast};
inline constexpr Power p2{.name = "p2", .motors = {Forward}, .cartridge = Red, .safe = Coast};
inline constexpr Power p3{.name = "p3", .motors = {Forward}, .cartridge = Red, .safe = Coast};
inline constexpr Power p4{.name = "p4", .motors = {Forward}, .cartridge = Red, .safe = Coast};
inline constexpr Power p5{.name = "p5", .motors = {Forward}, .cartridge = Red, .safe = Coast};
inline constexpr Power p6{.name = "p6", .motors = {Forward}, .cartridge = Red, .safe = Coast};
inline constexpr Power p7{.name = "p7", .motors = {Forward}, .cartridge = Red, .safe = Coast};
inline constexpr Power p8{.name = "p8", .motors = {Forward}, .cartridge = Red, .safe = Coast};
inline constexpr Power p9{.name = "p9", .motors = {Forward}, .cartridge = Red, .safe = Coast};
inline constexpr MechanismTable m = mechanisms(wire(p1, motors(1)), wire(p2, motors(2)), wire(p3, motors(3)), wire(p4, motors(4)), wire(p5, motors(5)), wire(p6, motors(6)), wire(p7, motors(7)), wire(p8, motors(8)), wire(p9, motors(9)));
// expect: more_mechanisms_than_a_robot_can_have
