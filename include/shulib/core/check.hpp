#pragma once
//
// Precondition checking for shulib core.
//
// SHULIB_PRECONDITION(cond, msg) guards a caller contract. On a violation it
// calls precondition_failed(), which in host/test builds THROWS a catchable
// PreconditionError — so a contract breach turns a test red instead of
// silently corrupting state (e.g. a NaN flowing into the pose estimate).
//
// TODO(M0 / master-plan §18.4): on the robot we will route a violated
// precondition to the fault-log + a safe fallback rather than throwing, so a
// single bad reading degrades gracefully instead of aborting the auton. The
// call sites do not change — only this policy does.

#include <stdexcept>

namespace shulib {

struct PreconditionError : std::logic_error {
    using std::logic_error::logic_error;
};

[[noreturn]] inline void precondition_failed(const char* message) {
    throw PreconditionError(message);
}

}  // namespace shulib

#define SHULIB_PRECONDITION(cond, msg) \
    ((cond) ? (void)0 : ::shulib::precondition_failed(msg))
