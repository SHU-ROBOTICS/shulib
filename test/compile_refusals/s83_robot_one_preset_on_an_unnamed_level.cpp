// s83_robot_one_preset_on_an_unnamed_level: a driver-file preset bound to robot one's FIRST level while the facts
// file names none (ROB1-C2: the builder's cascade is manual for now — kCascadeLevels typed as NONE). The rule
// that remains wherever a preset is bound: its level is typed first. run_refusal.py judges it.
#include "robots/mechanism_table.hpp"
#include "shulib/mechanism/driver_inputs.hpp"
namespace sb = shulib::bench;
namespace facts = sb::robot_one_facts;
inline constexpr shulib::mechanism::PositionGoal g = sb::robot_one::cascade[facts::lv(0).name];
// expect: no_such_preset_on_this_mechanism
