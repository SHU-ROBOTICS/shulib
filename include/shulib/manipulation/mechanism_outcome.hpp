#pragma once
//
// MechanismOutcome — the verdict vocabulary of a bounded mechanism operation
// (chunk F1, WS7/M4). A SEPARATE vocabulary from control::ExitReason, on
// purpose, and the reasoning deserves its length because D1 §2.7 warned that
// every new result vocabulary costs something:
//
// A mechanism has an outcome no motion can have: THE OPERATION COMPLETED AND
// THE THING DID NOT HAPPEN. The jaws closed on schedule, on a healthy
// mechanism, on nothing — no watchdog fired (so it is not TimedOut) and the
// task certainly did not succeed (so it is emphatically not Settled). That is
// `Unconfirmed`, and ExitReason has no honest place for it:
//   * Appending Unconfirmed to ExitReason would let a Chassis::moveTo
//     SYNTACTICALLY return it, which it never can — a vocabulary that admits
//     impossible values makes every exhaustive switch over it a lie.
//   * Reusing ExitReason plus a separate confirmed() flag is the cheapest shape
//     and the most dangerous: a caller that reads the verdict and not the flag
//     sees "Settled" on a failed grab — precisely the silent success the whole
//     error policy exists to prevent.
// So mechanisms get their own verdict, ExitReason stays exactly the four values
// a motion can produce, and the one place the two vocabularies meet —
// Routine::then() — maps them under test, inside the library, with only
// Succeeded mapping to success. This is a scoped enum with NO conversion to
// bool: "if (outcome)" does not compile, so Unconfirmed cannot be truthy by
// accident.
//
// `Stalled` is a verdict AND a pathology: the operation reports Stalled (so a
// sequencing layer can choose a jam-specific response — back off and retry is
// sensible for a jam and useless for a timeout) and the operation raises
// FaultCode::MechanismStalled (so triage sees the robot is unwell). `TimedOut`
// deliberately raises NO fault — see mechanism_op.hpp for that ruling (T6).
//
// Explicit values, append-only, like every verdict vocabulary in this project.
// Not currently on the F9 wire (DebugRecord has no mechanism slot — mechanisms
// do not emit per-tick records at F1), but log lines carry the spellings below,
// so the values are treated as stable anyway: re-meaning a value someone
// grepped a transcript for is the same bug class at human scale.

#include <cstdint>

namespace shulib::manipulation {

enum class MechanismOutcome : std::uint8_t {
    Running = 0,      ///< still working; tick again next loop iteration
    Succeeded = 1,    ///< completed AND confirmed (where the operation defines a
                      ///< confirmation; completed, where it does not)
    Unconfirmed = 2,  ///< the operation ran to completion and the confirmation
                      ///< said the world did not change — healthy mechanism,
                      ///< failed task. Strategy, not pathology: NO fault.
    TimedOut = 3,     ///< the watchdog fired before the operation completed
    Cancelled = 4,    ///< stopped from outside via cancel()
    Stalled = 5,      ///< the stall detector tripped (jam / mechanical bind) —
                      ///< FaultCode::MechanismStalled raised
};

/// Stable spelling for log lines. Never returns null; an out-of-range cast
/// renders as "UNKNOWN" (never a crash) — faultCodeName's rule.
[[nodiscard]] constexpr const char* mechanismOutcomeName(MechanismOutcome o) noexcept {
    switch (o) {
        case MechanismOutcome::Running: return "RUNNING";
        case MechanismOutcome::Succeeded: return "SUCCEEDED";
        case MechanismOutcome::Unconfirmed: return "UNCONFIRMED";
        case MechanismOutcome::TimedOut: return "TIMED_OUT";
        case MechanismOutcome::Cancelled: return "CANCELLED";
        case MechanismOutcome::Stalled: return "STALLED";
    }
    return "UNKNOWN";
}

}  // namespace shulib::manipulation
