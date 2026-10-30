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
// sites point here. Their behaviour did not change then: the bodies were COMP-t's, line for line. The
// mechanism core's MECH-A5 (2026-10-29) changed both: the proof's third layer (the service) and the
// stop's claimant cancel (below).
//
// ── The chain proof ─────────────────────────────────────────────────────────────────
// A program that can end an auton early puts a WATCH between the guard and the real pacer, and —
// since the mechanism core's MECH-A5 (2026-10-29) — the robot's MECHANISM SERVICE between the two:
//     Chassis --pace()--> RunGuard --> MechanismService --> <the program's watch> --> ... the tick pacer
// The guard checks its deadlines first, then the service ticks every mechanism, then the world
// advances (the MECH requirements' order). The watch reads what ends a run (the tester: any button, a
// touch, a controller drop, a disable; the competition program: a disable, or the match leaving
// autonomous) and asks the guard to abandon it. Three wiring mistakes would each leave a run that
// NOTHING can stop or a mechanism nothing runs, and each compiles: a Chassis built without the guard
// as its pacer (no layer ever paces), a watch never attached to the guard (it paces and latches, and
// asks nobody — measured by COMP-t's review: the run drove 14.22 in past the button with a green
// FINISHED), and a service left out of the chain or never bound (every mechanism step of an auton then
// waits on a mechanism nothing ticks — the MECH brief's P1 control case: the intake ran to its bound
// while the robot drove past the piece). So each program PROVES its chain at boot, and refuses to boot
// when the proof fails, naming the layer that did not pace. pacerChainMissing() is that proof, for ANY
// watch with the two members it reads (the WatchLike concept below).
//
// ── The program-error stop ──────────────────────────────────────────────────────────
// When a throw reaches a program's own auton task — a precondition (a programming error) or any
// other exception — the task must leave the robot SAFE and must not die: a C++ exception out of a
// PROS task function is std::terminate, with the drive in whatever state it was last given.
// stopAfterProgramError() is that stop: the drive to 0 V + Brake (Chassis::cancel, the facade's
// panic stop), then every PRESENT mechanism to its declared safe state AND its registered claimant
// cancelled — the mechanism service's slot (MECH-A5), so a slot that held a behaviour when the throw
// came is inert and cannot command it again at a later tick (the "half-safe" failure hal/mechanism.hpp
// describes: a safe state alone lasts until the next tick of whatever still believes it owns the
// mechanism) — each step tried on its own, so a step that throws cannot skip the rest, and nothing
// leaves it. A program calls it ONLY
// from the task that owns the robot at that moment (the competition program: inside a request,
// before its terminal word — after that the driver task may be driving). The tester keeps a
// thin wrapper (auton_tester_run.hpp) that also disarms its own watch.
//
// PROS-free, header-only. Host-tested in test/guarded_run_test.cpp, and — through the tester's
// call sites — by test/auton_abort_watch_test.cpp (13b) and test/auton_tester_run_test.cpp (21c).
// NOTHING HERE HAS RUN ON A ROBOT.

#include <concepts>
#include <cstdint>
#include <span>

#include "shulib/chassis/chassis.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/mechanism/service.hpp"
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

/// The layer of the auton pacer chain that one idle tick did NOT reach (pacerChainMissing()), or None.
enum class ChainGap {
    None,         ///< the guard, the service and the watch were each paced, and the watch is attached
    Guard,        ///< the Chassis is not paced through the run guard
    Service,      ///< the mechanism service did not tick: it is not under the guard, or it is not bound
    Watch,        ///< the watch was not paced: it is not under the service
    NotAttached,  ///< the watch paces, but asks nobody: it is not attached to this guard
};

/// The refusal's words for `g` — what the boot prints after "the pacer chain is WRONG -- ".
[[nodiscard]] constexpr const char* chainGapWords(ChainGap g) noexcept {
    switch (g) {
        case ChainGap::None:
            return "every layer paced";
        case ChainGap::Guard:
            return "one idle tick did not pace the run guard: the Chassis is not built on it";
        case ChainGap::Service:
            return "one idle tick did not tick the mechanism service: it is not under the run guard, or "
                   "it was never bound";
        case ChainGap::Watch:
            return "one idle tick did not pace the watch: it is not under the mechanism service";
        case ChainGap::NotAttached:
            return "the watch is not attached to the run guard: an abort would ask nobody";
    }
    return "?";
}

/// THE BOOT'S CHAIN PROOF (file banner): one idle tick (Chassis::wait commands nothing) must pace
/// the guard, TICK the mechanism service (service.ticks() advanced — an unbound service ticks
/// nothing) and pace the watch, and the watch must be ATTACHED to this guard — the chain's one link
/// back, which no pace can show. Returns the FIRST gap in that order, or ChainGap::None. A program
/// refuses its boot unless it is None (naming the gap: chainGapWords()). Call it after the service is
/// bound and before anything paints through the chain, from the task that owns the Chassis.
template <WatchLike Watch>
[[nodiscard]] ChainGap pacerChainMissing(chassis::Chassis& chassis, const sequence::RunGuard& guard,
                                         const mechanism::MechanismService& service, const Watch& watch) {
    const std::uint64_t g0 = guard.pacesTotal();
    const long s0 = service.ticks();
    const std::uint64_t w0 = watch.paces();
    chassis.wait(units::Time{kGuardedTickSeconds});
    if (guard.pacesTotal() <= g0) {
        return ChainGap::Guard;
    }
    if (service.ticks() <= s0) {
        return ChainGap::Service;
    }
    if (watch.paces() <= w0) {
        return ChainGap::Watch;
    }
    return watch.attachedTo(guard) ? ChainGap::None : ChainGap::NotAttached;
}

/// pacerChainMissing() == ChainGap::None — the proof as a yes / no.
template <WatchLike Watch>
[[nodiscard]] bool pacerChainProven(chassis::Chassis& chassis, const sequence::RunGuard& guard,
                                    const mechanism::MechanismService& service, const Watch& watch) {
    return pacerChainMissing(chassis, guard, service, watch) == ChainGap::None;
}

/// THE PROGRAM-ERROR STOP (file banner): the drive at 0 V + Brake (the panic stop), then every
/// PRESENT mechanism to its safe state, then its registered claimant (the mechanism service's slot)
/// cancelled — inert, safe, released, a running step CANCELLED — so nothing re-commands it at a later
/// tick. Never throws: each step is tried on its own, so one that fails cannot skip the rest. Only
/// from the task that owns the robot at that moment.
inline void stopAfterProgramError(chassis::Chassis& chassis,
                                  std::span<hal::IMechanism* const> present) noexcept {
    try {
        chassis.cancel();
    } catch (...) {  // NOLINT(bugprone-empty-catch): the stop must reach every mechanism
    }
    for (hal::IMechanism* m : present) {
        if (m == nullptr) {
            continue;
        }
        try {
            m->applySafeState();
        } catch (...) {  // NOLINT(bugprone-empty-catch): one broken mechanism skips no other
        }
        try {
            if (hal::ICancellable* claimant = m->claimant(); claimant != nullptr) {
                claimant->cancel();
            }
        } catch (...) {  // NOLINT(bugprone-empty-catch): one broken claimant skips no other
        }
    }
}

}  // namespace shulib::bench
