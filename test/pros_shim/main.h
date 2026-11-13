#pragma once
//
// HOST SHIM for a PROS project's <main.h> (chunk CAL1, Part A1, 2026-10-08): the five entry points
// a PROS program defines, declared with C linkage exactly as the project's include/main.h :42-48
// declares them — and nothing else (the real file also pulls in all of api.h, which the host
// cannot compile). It exists so a team's whole program file, which starts `#include "main.h"`,
// compiles in the host suite exactly as written (CAL1 Part A2's example test of the standard
// Calibrate's team file). test/CMakeLists.txt puts test/pros_shim/ BEFORE include/ on the host's
// include path, so this file, never the real one, answers that include in the host build.

#ifndef SHULIB_HOST_PROS_SHIM
#error "test/pros_shim/ is the HOST TEST shim for PROS — it must NEVER reach a robot build. \
Only test/CMakeLists.txt defines SHULIB_HOST_PROS_SHIM."
#endif

extern "C" {
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
}
