// Tests for core/check.hpp's §18.4 policy seam (chunk A1 resolved the old TODO here).
// What each targets:
//  * DEFAULT POLICY: with nothing installed, a violation still throws PreconditionError
//    — every existing CHECK_THROWS_AS(PreconditionError) in the suite depends on this,
//    so the seam must be invisible until a policy is installed.
//  * ROUTING: an installed handler receives the violation (with its message) INSTEAD of
//    the default throw — this is the seam R1's on-robot fault-log policy plugs into.
//  * RESTORABILITY: setPreconditionHandler returns the previous handler and nullptr
//    restores the default — without this, one test (or one init path) could silently
//    change precondition behavior for everything after it.
// Every test restores the default via RAII, even on assertion failure — the rest of
// the suite depends on the throwing policy.

#include "doctest.h"

#include <string>

#include "shulib/core/check.hpp"

using shulib::PreconditionError;
using shulib::PreconditionHandler;
using shulib::preconditionHandler;
using shulib::setPreconditionHandler;
using shulib::throwingPreconditionHandler;

namespace {
struct AltPolicyError {};  // distinct type: catching it proves the ALTERNATE path ran

int g_altCalls = 0;
std::string g_altMessage;

[[noreturn]] void altHandler(const char* message) {
    ++g_altCalls;
    g_altMessage = message;
    throw AltPolicyError{};
}

/// Restores the previously installed handler on scope exit (assertion-failure safe).
struct HandlerGuard {
    explicit HandlerGuard(PreconditionHandler h) : prev_{setPreconditionHandler(h)} {}
    ~HandlerGuard() { setPreconditionHandler(prev_); }
    HandlerGuard(const HandlerGuard&) = delete;
    HandlerGuard& operator=(const HandlerGuard&) = delete;
    PreconditionHandler prev_;
};
}  // namespace

TEST_CASE("check policy: the default (host) policy throws PreconditionError — breaches "
          "turn tests red with no setup") {
    CHECK(preconditionHandler() == &throwingPreconditionHandler);
    CHECK_THROWS_AS(SHULIB_PRECONDITION(false, "default policy"), PreconditionError);
    // …and with the exact message (what the on-robot fault line will carry too).
    CHECK_THROWS_WITH(SHULIB_PRECONDITION(false, "exact message"), "exact message");
}

TEST_CASE("check policy: an installed handler receives the violation INSTEAD of the "
          "default throw — call sites unchanged") {
    HandlerGuard guard{&altHandler};
    g_altCalls = 0;
    g_altMessage.clear();

    CHECK_THROWS_AS(SHULIB_PRECONDITION(1 == 2, "routed message"), AltPolicyError);
    CHECK(g_altCalls == 1);
    CHECK(g_altMessage == "routed message");  // the message reaches the policy intact
}

TEST_CASE("check policy: a PASSING check never consults the policy — the seam costs "
          "nothing on the happy path") {
    HandlerGuard guard{&altHandler};
    g_altCalls = 0;
    SHULIB_PRECONDITION(true, "never seen");
    CHECK(g_altCalls == 0);
}

TEST_CASE("check policy: setPreconditionHandler returns the previous handler, and "
          "restoring it restores the previous behavior") {
    const PreconditionHandler prev = setPreconditionHandler(&altHandler);
    CHECK(prev == &throwingPreconditionHandler);  // what was installed before
    CHECK(preconditionHandler() == &altHandler);

    const PreconditionHandler wasAlt = setPreconditionHandler(prev);
    CHECK(wasAlt == &altHandler);
    CHECK(preconditionHandler() == &throwingPreconditionHandler);
    CHECK_THROWS_AS(SHULIB_PRECONDITION(false, "back to default"), PreconditionError);
}

TEST_CASE("check policy: nullptr restores the default — the seam can never be left empty") {
    {
        HandlerGuard guard{&altHandler};
        (void)setPreconditionHandler(nullptr);
        CHECK(preconditionHandler() == &throwingPreconditionHandler);
        CHECK_THROWS_AS(SHULIB_PRECONDITION(false, "null resets"), PreconditionError);
    }
    // Guard restored the pre-test state; the suite continues on the default policy.
    CHECK(preconditionHandler() == &throwingPreconditionHandler);
}
