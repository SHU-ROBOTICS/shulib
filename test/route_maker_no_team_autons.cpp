// route_maker_no_team_autons.cpp — the route builder's engine (`shulib_route_maker` and the WebAssembly module;
// chunk RMP1-A2e) runs the BUILDER's route on robot one's or robot two's list, never the team's autons: the runner
// gives each of RPV1's robot rows its own register function. These two only let those rows link without src/team/
// (the team's files change after the freeze; the engine does not compile them), and refuse if ever called.

#include <stdexcept>

#include "team/autons.hpp"
#include "team/robot_one_autons.hpp"

namespace shulib::bench {

void registerAutons(competition::AutonRegistry&) {
    throw std::logic_error{"the route builder's engine never runs the team's autons"};
}
void registerRobotOneAutons(competition::AutonRegistry&) {
    throw std::logic_error{"the route builder's engine never runs the team's autons"};
}

}  // namespace shulib::bench
