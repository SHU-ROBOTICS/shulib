#pragma once
//
// build_info — the git build hash plumbing for the §18.5 session header (WS13, C5).
//
// THE POINT: six months after a run, the log must say WHICH BINARY produced it.
// The hash therefore comes from the BUILD SYSTEM, never from code — code cannot
// know its own commit. The consuming build defines
//
//     -DSHULIB_BUILD_HASH="\"<git describe --always --dirty>\""
//
// (test/CMakeLists.txt does exactly this for the host suite; the PROS Makefile
// gains the same line at R1). The --dirty suffix is NOT optional politeness: a
// clean-looking hash from a modified tree IS a wrong hash, and a wrong hash is
// worse than an absent one — it sends the 2am investigation to the wrong commit
// with full confidence.
//
// THE LOUDNESS CONTRACT (§18.5, the brief's landmine): when the macro is absent,
// compiledBuildHash() returns EMPTY, and every renderer treats empty as MISSING —
// an [ERROR] line in the session header, the literal token "MISSING" in the run
// summary. Nothing anywhere invents a plausible-looking placeholder. There is no
// "unknown"/"0000000" fallback BY DESIGN; grep for one before adding it, then
// don't.
//
// Header-only nuance, stated honestly: the macro is evaluated PER TRANSLATION
// UNIT. The value that reaches a log is the one seen by the TU that populated the
// SessionInfo (in practice: the one place a program builds its session header).
// A program that never defines the macro gets the loud MISSING path — which is
// correct, not a bug.

#include <string_view>

namespace shulib::diag {

/// The build hash the build system injected, or EMPTY if it injected none.
/// Empty means MISSING and must be rendered loudly (header note).
[[nodiscard]] constexpr std::string_view compiledBuildHash() noexcept {
#ifdef SHULIB_BUILD_HASH
    return SHULIB_BUILD_HASH;
#else
    return {};
#endif
}

}  // namespace shulib::diag
