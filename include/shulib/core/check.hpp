#pragma once
//
// Precondition checking for shulib core.
//
// SHULIB_PRECONDITION(cond, msg) guards a CALLER contract. On a violation it calls
// precondition_failed(), which routes through an installable POLICY HANDLER — the
// §18.4 policy seam (resolved at chunk A1; this replaced the former TODO here).
//
// The two policies, and why the seam exists:
//   * HOST/TEST (the DEFAULT handler, installed at startup automatically): THROW a
//     catchable PreconditionError, so a contract breach turns a test RED instead of
//     silently corrupting state (e.g. a NaN flowing into the pose estimate).
//   * ON-ROBOT (installed once at init by R1's hal/pros bootstrap): raise
//     diag::FaultCode::Precondition on the fault latch, then THROW the same
//     PreconditionError — which the motion scheduler catches at the task boundary and
//     converts to a FAULT_ABORT exit + a safe drivetrain state. Recovery happens at
//     the MOTION boundary, not the call site: the call sites guard invariants
//     (bounds, non-null, finite) past which continuing would be undefined behavior,
//     so "log and continue right here" is not a safe fallback — unwinding to the
//     nearest boundary that CAN recover is. One bad reading degrades one motion to a
//     fault code; it never aborts the auton.
//
// The call sites never change — SHULIB_PRECONDITION(cond, msg) everywhere, on every
// target — only the installed policy differs. That is the whole design.
//
// HANDLER CONTRACT (load-bearing): a handler must NOT return — it throws (both shipped
// policies do) or otherwise diverts control. precondition_failed() is [[noreturn]];
// if a broken handler does return, std::terminate() fires rather than letting
// execution continue past a violated invariant into undefined behavior. (This
// terminate is reachable ONLY via a handler that violates the seam's contract — the
// shipped policies can never hit it.)
//
// Concurrency contract: the handler is installed ONCE, at startup, before any other
// task exists (host: never re-installed outside tests; robot: in R1's init). The slot
// is a plain pointer read on the hot path — no atomics, because installation is not
// concurrent with use by contract.

#include <exception>
#include <stdexcept>

namespace shulib {

struct PreconditionError : std::logic_error {
    using std::logic_error::logic_error;
};

/// The policy hook type. MUST NOT RETURN (see header contract).
using PreconditionHandler = void (*)(const char* message);

/// The host/test default policy: throw, so breaches turn tests red.
[[noreturn]] inline void throwingPreconditionHandler(const char* message) {
    throw PreconditionError(message);
}

namespace detail {
inline PreconditionHandler& preconditionHandlerSlot() noexcept {
    static PreconditionHandler handler = &throwingPreconditionHandler;
    return handler;
}
}  // namespace detail

/// Install a policy handler; returns the PREVIOUS handler so a caller (e.g. a test)
/// can restore it. Passing nullptr restores the default throwing policy — the seam is
/// never left empty.
inline PreconditionHandler setPreconditionHandler(PreconditionHandler handler) noexcept {
    PreconditionHandler& slot = detail::preconditionHandlerSlot();
    const PreconditionHandler previous = slot;
    slot = (handler != nullptr) ? handler : &throwingPreconditionHandler;
    return previous;
}

/// The currently installed policy (introspection; used by tests to restore).
[[nodiscard]] inline PreconditionHandler preconditionHandler() noexcept {
    return detail::preconditionHandlerSlot();
}

[[noreturn]] inline void precondition_failed(const char* message) {
    detail::preconditionHandlerSlot()(message);
    // Only reachable through a handler that violated its must-not-return contract:
    // never continue past a violated invariant (header note).
    std::terminate();
}

}  // namespace shulib

#define SHULIB_PRECONDITION(cond, msg) \
    ((cond) ? (void)0 : ::shulib::precondition_failed(msg))
