// route_maker_main.cpp — `shulib_route_maker`, the route builder's ONE main (chunks RMP1-A1, RMP1-A2e; the RMP1
// brief A.3.3 [W9], §1.1): the native program AND the WebAssembly module (tools/route_maker/wasm/). Host only:
// nothing a robot program compiles.
//
//   shulib_route_maker describe --robot one|two|x|y|z   one robot's description as JSON (route_maker_describe.hpp)
//   shulib_route_maker curves                           CV1's curve golden (route_maker_curves.hpp)
//   shulib_route_maker feel                             FE1's feel golden (route_maker_feel.hpp, RMP1-B)
//   shulib_route_maker feel-edges                       FE1b's: the dead zone's edges, -0, the limits
//   shulib_route_maker preview <lines>|-                the route's run as a record (route_maker_runner.hpp);
//                                                       `-` reads the lines from stdin. The page calls the module's
//                                                       callMain(["preview", lines]).
//   (the test module only) golden <name> | goldens      a golden's EXPORTED C++, compiled in, run the same way (WG1)
//
// Exit codes: 0 done (a failed or skipped step too: the record says so); 1 the route threw, went THROUGH, was
// refused or looped; 2 a usage error or lines it cannot run; 3 the program itself failed.

#include <cstdio>
#include <cstring>
#include <exception>
#include <string>
#include <string_view>

#include "route_maker_curves.hpp"
#include "route_maker_feel.hpp"
#include "route_maker_robots.hpp"
#include "route_maker_runner.hpp"

#ifdef SHULIB_ROUTE_MAKER_TEST_STEPS
int runGolden(const char* name);  // route_maker_goldens.cpp
void listGoldens();
#endif

namespace {

int usage() {
    std::fputs("usage: shulib_route_maker describe --robot one|two|x|y|z\n"
               "       shulib_route_maker curves\n"
               "       shulib_route_maker feel | feel-edges\n"
               "       shulib_route_maker preview <lines>|-\n", stderr);
    return 2;
}

int run(int argc, char** argv) {
    if (argc == 2 && std::strcmp(argv[1], "curves") == 0) {
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
#ifdef __EMSCRIPTEN__
        // The C++ names come from the robot files' source lines, which the module cannot read: the page's
        // descriptions are the native program's, baked at its build (A.3.2 (b)).
        std::fputs("shulib_route_maker: describe runs in the native program only\n", stderr);
        return 2;
#else
        const auto src = route_maker::robotSource(argv[3]);
        if (!src) {
            std::fprintf(stderr, "shulib_route_maker: no robot \"%s\" (one, two, x, y or z)\n", argv[3]);
            return usage();
        }
        const std::string j = route_maker::describe(*src);
        std::fwrite(j.data(), 1, j.size(), stdout);
        return 0;
#endif
    }
    if (argc == 3 && std::strcmp(argv[1], "preview") == 0) {
#ifndef __EMSCRIPTEN__  // the module takes its lines as the argument (no stdin: no file system in its glue)
        if (std::strcmp(argv[2], "-") == 0) {
            std::string lines;
            char buf[4096];
            for (std::size_t n; (n = std::fread(buf, 1, sizeof buf, stdin)) > 0;) {
                lines.append(buf, n);
            }
            return route_maker::preview(lines);
        }
#endif
        return route_maker::preview(argv[2]);
    }
#ifdef SHULIB_ROUTE_MAKER_TEST_STEPS
    if (argc == 3 && std::strcmp(argv[1], "golden") == 0) {
        return runGolden(argv[2]);
    }
    if (argc == 2 && std::strcmp(argv[1], "goldens") == 0) {
        listGoldens();
        return 0;
    }
#endif
    return usage();
}

}  // namespace

int main(int argc, char** argv) {
    try {
        return run(argc, argv);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "shulib_route_maker: the program itself failed: %s\n", e.what());
        return 3;
    }
}
