// route_preview_main.cpp — the ROUTE PREVIEW program's main (chunk RPV1-A, 2026-11-12): it only forwards
// argv to route_preview::runProgram (test/route_preview_run.hpp), where ALL the logic lives — the exit
// code of a preview that itself failed included — so the suite can call it. Not a `*_test.cpp`, so the
// suite's glob never compiles it; test/CMakeLists.txt builds it as its own program, shulib_route_preview,
// and tools/route_preview.py runs it under the suite's bounds.
// Say "host test plant" (the project's one name for it): nothing here runs on a robot.

#include <iostream>
#include <string_view>
#include <vector>

#include "route_preview_run.hpp"

int main(int argc, char** argv) {
    std::vector<std::string_view> args;
    for (int i = 1; i < argc; ++i) {
        args.emplace_back(argv[i]);
    }
    return route_preview::runProgram(args, std::cout, std::cerr);
}
