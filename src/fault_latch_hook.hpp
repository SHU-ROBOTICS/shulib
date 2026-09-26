#pragma once
//
// fault_latch_hook.hpp — how a program built in ANOTHER translation unit reaches src/main.cpp's
// precondition handler (G1 Part 2b, 2026-09-26; the brief's A12 and landmine 11).
//
// src/main.cpp installs the on-robot precondition policy FIRST in initialize(), before any adapter
// exists (core/check.hpp §18.4): a failed SHULIB_PRECONDITION raises FaultCode::Precondition on the
// run's fault latch — when there is one — and then throws. Until G1 Part 2b the latch pointer was
// set by main.cpp itself (`g_faults = &r.faults`), because both object graphs lived in main.cpp.
// "shulib Teleop"'s graph is now a robot::Robot built in src/teleop_program.cpp, which cannot see
// main.cpp's anonymous namespace; this one function is the hand-off. The program calls it once,
// right after its Robot exists, so a precondition AFTER boot still latches exactly as it did.
//
// DEFINED in src/main.cpp for EVERY build (not under a program's #if): every src/*.cpp compiles
// in all six builds (the teleop program included), so a symbol one of them calls must exist in
// all six or the link fails. In the builds whose program never calls it, --gc-sections drops it.

namespace shulib::diag {
class FaultLatch;
}  // namespace shulib::diag

namespace shulib::bench {

/// Point src/main.cpp's precondition handler at `latch` (nullptr detaches it: the handler then
/// only throws). Call it once the latch exists and before anything can raise on it; the latch must
/// outlive every later precondition — in practice, the program's lifetime.
void attachFaultLatch(diag::FaultLatch* latch) noexcept;

}  // namespace shulib::bench
