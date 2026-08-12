#pragma once
//
// The shulib API version — the mechanism behind the Freeze Register's promise.
//
// The register (docs/roadmap.md, "Freeze Register") says a LOCKED contract
// changes "only with a schemaVersion/API-version bump and a migration path".
// Until D2 (2026-08-12) no such version existed anywhere in the tree; this
// header is that mechanism, made inspectable in code instead of prose.
//
// ═══ The policy: what a version bump concretely IS ═══════════════════════════════
// shulib is a header-only C++ library, so its API contract is SOURCE
// compatibility — what user programs compile to and mean:
//
//   * BREAKING (bump kApiMajor): any change that can make a previously-
//     compiling user program fail to compile or silently change meaning —
//     removing or renaming a frozen member or type; changing a frozen
//     signature's parameter, return, const, ref or noexcept shape; changing
//     documented semantics a frozen surface carries (pre-empt, cancel's safe
//     state, wait-for-live, the fault policy...); re-meaning or renumbering
//     an existing enumerator. A major bump REQUIRES a migration note next to
//     the Freeze Register row: the old spelling, the new spelling, the
//     mechanical rewrite, and why the break was worth it.
//
//   * ADDITIVE (bump kApiMinor): every previously-valid program compiles
//     unchanged with unchanged behaviour — new members, new overloads, new
//     options-struct fields whose default preserves the old behaviour,
//     appended enumerators. This is the intended growth path of every frozen
//     surface; freezing well means never needing a major bump.
//
// Frozen C++ surfaces are additionally enforced STRUCTURALLY: F6 carries a
// compile-time signature pin (test/f6_signature_pin_test.cpp) that fails the
// build, naming F6, if a frozen signature changes shape. If that pin sent
// you here and the change is intended: you are making a breaking change —
// update the pin, bump kApiMajor, write the migration note, and update the
// register row. If the change can be re-expressed additively instead, do
// that; it is almost always cheaper for everyone downstream.
//
// The data-schema freezes (F7/F8: .vexbot sub-schemas; F9: the SHUL/2 wire
// protocol) freeze DATA, not C++ source. They carry their own schemaVersion
// fields inside the artifacts (designed at Phase G/H, not here) and are
// governed by this same breaking-vs-additive policy.

namespace shulib {

/// Bumped ONLY for breaking changes to a frozen public surface, always with
/// a migration note (policy above). 2 = the shulib v2 rebuild; the surface
/// frozen at D2 (2026-08-12, register row F6) is API 2.0.
inline constexpr int kApiMajor = 2;

/// Bumped for additive extensions of a frozen surface (new verbs, new
/// options fields, appended enumerators). Reset to 0 on a major bump.
inline constexpr int kApiMinor = 0;

/// "major.minor", for session headers / logs that want one printable token.
inline constexpr const char* kApiVersionString = "2.0";

}  // namespace shulib
