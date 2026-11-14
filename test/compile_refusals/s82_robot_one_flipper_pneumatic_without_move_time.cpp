// s82_robot_one_flipper_pneumatic_without_move_time: robot one's PNEUMATIC flipper form (guide chapter 21,
// test/robot_one_flipper_forms_example_test.cpp) pasted without its measured stroke time — a pneumatic with no move time
// would be counted done the instant its line flips (the ROB1 brief's T13, GM24). run_refusal.py judges it.
#include "refusal_common.hpp"
#include "robots/mechanism_table.hpp"
inline constexpr Pneumatic flipperAir{.name = "flipper", .positions = {{"stow", {Low}}, {"flip", {High}}},
                                      .start = "stow", .safe = "stow"};
inline constexpr MechanismTable withAir = mechanisms(
    wire(shulib::bench::robot_one::cascade, motors(shulib::bench::robot_one_facts::kCascadePorts.value().first,
                                                   shulib::bench::robot_one_facts::kCascadePorts.value().second)),
    wire(flipperAir, lines('A')));
// expect: pneumatic_actuationtime_must_be_more_than_zero
