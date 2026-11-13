// "shulib Tune" — the build's mechanism table handed to the library's Tune program (chunk TUNE1-B1,
// 2026-11-10; src/programs/tune_program.hpp says what it is). Every line of the program is the
// library's: include/shulib/mechanism/tune_program.hpp, through include/shulib/hal/pros/tune.hpp.
//
// ROBOT-GENERIC (the MECH-S/TUNE1 brief's re-check #10): nothing here names a robot. A robot gets Tune
// by having a mechanism table — the Makefile's MECHANISM_ROBOTS list and src/build_selection.hpp's
// chain accept PROGRAM=tune for it — and this file reads the BUILD's table, kMechanisms. The
// static_assert below is the last line of defence behind the Makefile's $(error): a Tune build of a
// robot that declares no mechanism stops here, naming why. Every .cpp under src/ compiles into every
// build, so it judges only a build whose program IS Tune.

#include "programs/tune_program.hpp"

#include "build_selection.hpp"
#include "robots/mechanism_table.hpp"
#include "shulib/hal/pros/tune.hpp"

static_assert(shulib::bench::kProgram != shulib::bench::Program::Tune || shulib::bench::kMechanisms.count > 0,
              "PROGRAM=tune needs a robot whose mechanism table is not empty: this ROBOT declares no mechanism "
              "(src/robots/mechanism_table.hpp), and the Tune program builds ONLY mechanisms");

void shulib::bench::tuneInitialize(const char* variantBeacon, const char* competitionStatusBits) {
    shulib::hal::pros::tuneInitialize(kMechanisms, variantBeacon, competitionStatusBits);
}

void shulib::bench::runTune() { shulib::hal::pros::runTune(kMechanisms); }
