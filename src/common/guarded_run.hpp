#pragma once
//
// guarded_run.hpp — what every program that runs an auton under a sequence::RunGuard shares: the
// boot's CHAIN PROOF and the PROGRAM-ERROR STOP (moved here at chunk COMP-b's first step,
// 2026-09-30 — the COMP brief's decision ND3 (b), the coordinator's ruling of 2026-09-28).
//
// ── Why here, and why now ───────────────────────────────────────────────────────────
// "shulib Auton Tester" (chunk COMP-t, 2026-09-28) wrote both, typed on its own abort watch, in
// src/programs/auton_tester_run.hpp. The competition program (COMP-b) needs the same two with its
// OWN watch — the tester's AbortWatch aborts on ANY button, and in a match a driver's bumped button
// must never end the auton, so COMP-b may not reuse that class (the COMP-t brief's §4). The COMP
// brief's rule for a piece two programs need is that it MOVES to one shared place and is never
// copied (its B11), so they were made generic here, at the second consumer, and the tester's call
// sites point here. Their behaviour did not change: the bodies are COMP-t's, line for line.
//
// ── The chain proof ─────────────────────────────────────────────────────────────────
// A program that can end an auton early puts a WATCH between the guard and the real pacer:
//     Chassis --pace()--> RunGuard --inner--> <the program's watch> --inner--> ... the tick pacer
// The watch reads what ends a run (the tester: any button, a touch, a controller drop, a disable;
// the competition program: a disable, or the match leaving autonomous) and asks the guard to
// abandon it. Two wiring mistakes would each leave a run that NOTHING can stop, and both compile:
// a Chassis built without the guard as its pacer (neither layer ever paces), and a watch never
// attached to the guard (it paces and latches, and asks nobody — measured by COMP-t's review: the
// run drove 14.22 in past the button with a green FINISHED). So each program PROVES its chain at
// boot, and refuses to boot when the proof fails. pacerChainProven() is that proof, for ANY watch
// with the two members it reads (the WatchLike concept below).
//
// ── The program-error stop ──────────────────────────────────────────────────────────
// When a throw reaches a program's own auton task — a precondition (a programming error) or any
// other exception — the task must leave the robot SAFE and must not die: a C++ exception out of a
// PROS task function is std::terminate, with the drive in whatever state it was last given.
// stopAfterProgramError() is that stop: the drive to 0 V + Brake (Chassis::cancel, the facade's
// panic stop), then every PRESENT mechanism to its declared safe state — each step tried on its
// own, so a step that throws cannot skip the rest, and nothing leaves it. A program calls it ONLY
// from the task that owns the robot at that moment (the competition program: inside a request,
// before its terminal word — after that the driver task may be driving). The tester keeps a
// thin wrapper (auton_tester_run.hpp) that also disarms its own watch.
//
// PROS-free, header-only. Host-tested in test/guarded_run_test.cpp, and — through the tester's
// call sites — by test/auton_abort_watch_test.cpp (13b) and test/auton_tester_run_test.cpp (21c),
// which did not change. NOTHING HERE HAS RUN ON A ROBOT.

#include <concepts>
#include <cstdint>
#include <span>

#include "shulib/chassis/chassis.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench {

/// One motion tick, in seconds: the idle tick the chain proof waits. Each program that proves its
/// chain static_asserts it equals hal::pros::ProsTickPacer::kTickMs (this header is PROS-free).
inline constexpr double kGuardedTickSeconds = 0.01;

/// What the chain proof reads from a watch: its own count of every pace() since construction,
/// armed or not, and whether it was attached to a given guard.
template <class Watch>
concept WatchLike = requires(const Watch& w, const sequence::RunGuard& g) {
    { w.paces() } -> std::convertible_to<std::uint64_t>;
    { w.attachedTo(g) } -> std::convertible_to<bool>;
};

/// THE BOOT'S CHAIN PROOF (file banner): one idle tick (Chassis::wait commands nothing) must pace
/// BOTH the guard and the watch, or no abort could reach the motion; and the watch must be
/// ATTACHED to this guard — the chain's one link back, which no pace can show. A program refuses
/// its boot when this is false. Call it before anything paints through the chain, from the task
/// that owns the Chassis.
template <WatchLike Watch>
[[nodiscard]] bool pacerChainProven(chassis::Chassis& chassis, const sequence::RunGuard& guard,
                                    const Watch& watch) {
    const std::uint64_t g0 = guard.pacesTotal();
    const std::uint64_t w0 = watch.paces();
    chassis.wait(units::Time{kGuardedTickSeconds});
    return guard.pacesTotal() > g0 && watch.paces() > w0 && watch.attachedTo(guard);
}

/// THE PROGRAM-ERROR STOP (file banner): the drive at 0 V + Brake (the panic stop), then every
/// PRESENT mechanism to its safe state. Never throws: each step is tried on its own, so one that
/// fails cannot skip the rest. Only from the task that owns the robot at that moment.
inline void stopAfterProgramError(chassis::Chassis& chassis,
                                  std::span<hal::IMechanism* const> present) noexcept {
    try {
        chassis.cancel();
    } catch (...) {  // NOLINT(bugprone-empty-catch): the stop must reach every mechanism
    }
    for (hal::IMechanism* m : present) {
        try {
            if (m != nullptr) {
                m->applySafeState();
            }
        } catch (...) {  // NOLINT(bugprone-empty-catch): one broken mechanism skips no other
        }
    }
}

}  // namespace shulib::bench
