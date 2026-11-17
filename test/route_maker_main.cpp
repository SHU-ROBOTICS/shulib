// route_maker_main.cpp — `shulib_route_maker`, the route builder's native program (chunk RMP1-A1,
// 2026-11-14; the RMP1 brief A.3.3 [W9], §1.1). Host only: nothing a robot program compiles.
//
//   shulib_route_maker describe --robot one|two|x|y|z   one robot's description as JSON (route_maker_describe.hpp)
//   shulib_route_maker curves                           CV1's curve golden (route_maker_curves.hpp)
//   shulib_route_maker feel                             FE1's feel golden (route_maker_feel.hpp, RMP1-B)
//   shulib_route_maker feel-edges                       FE1b's: the dead zone's edges, -0, the limits
//
// CI's route-builder job runs `describe` for robots one and two and bakes the result into the page
// (tools/route_maker/build.py --describe). `preview` — the runner, the record as JSON — is RMP1-A2e's, added
// to THIS file then (one main for the native program and the WebAssembly module, W9).
//
// Exit codes: 0 done; 2 a usage error (the usage is printed).

#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>

#include "route_maker_curves.hpp"
#include "route_maker_feel.hpp"
#include "route_maker_robots.hpp"

namespace {

int usage() {
    std::fputs("usage: shulib_route_maker describe --robot one|two|x|y|z\n"
               "       shulib_route_maker curves\n"
               "       shulib_route_maker feel | feel-edges\n", stderr);
    return 2;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc >= 2 && std::strcmp(argv[1], "curves") == 0 && argc == 2) {
        const std::string t = route_maker::curvesTable();
        std::fwrite(t.data(), 1, t.size(), stdout);
        return 0;
    }
    if (argc == 2 && (std::strcmp(argv[1], "feel") == 0 || std::strcmp(argv[1], "feel-edges") == 0)) {
        const std::string t = std::strcmp(argv[1], "feel") == 0 ? route_maker::feelCurveTable() : route_maker::feelEdgeTable();
        std::fwrite(t.data(), 1, t.size(), stdout);
        return 0;
    }
    if (argc == 4 && std::strcmp(argv[1], "describe") == 0 && std::strcmp(argv[2], "--robot") == 0) {
        const auto src = route_maker::robotSource(argv[3]);
        if (!src) {
            std::fprintf(stderr, "shulib_route_maker: no robot \"%s\" (one, two, x, y or z)\n", argv[3]);
            return usage();
        }
        const std::string j = route_maker::describe(*src);
        std::fwrite(j.data(), 1, j.size(), stdout);
        return 0;
    }
    return usage();
}
