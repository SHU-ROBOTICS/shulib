#pragma once
//
// AbsentGps — the explicit statement that THIS ROBOT HAS NO GPS, as an IGps (chunk R3b §6).
//
// Not a test double. hal/fake/FakeGps carries setPose()/setHasFix() mutators whose whole
// purpose is to be driven by a test; this class carries none, deliberately, because there is
// nothing to drive — the device does not exist. The two used to be conflated at the
// composition root (a fake wired into the competition binary as a stand-in for "none"), which
// shipped test scaffolding on the robot and erased the difference between "this robot has no
// such device" and "a test will inject readings here later". That conflation is the defect
// this class exists to fix, which is also why it lives HERE, beside null_sink.hpp, and not in
// hal/fake/.
//
// THE RULING IT IMPLEMENTS (R3b §6.2): RobotContext's non-null preconditions DO NOT relax.
// An accidental omission is still `nullptr` and still fails loudly at construction, naming the
// missing handle; a deliberate absence is spelled out at the call site — `.gps = &absentGps` —
// as a statement about the robot's configuration. "I forgot" and "this robot has none" stay
// distinguishable in the source, which is the whole point.
//
// NAMING — a recorded decision. `NullGps` (consistent with NullSink) was REJECTED: NullSink is
// a WORKING sink that discards, whereas this says NO SUCH DEVICE IS INSTALLED. The semantics
// genuinely differ, and the composition root is exactly where that difference must read
// clearly. Consistency loses to precision.
//
// NO Freeze Register row: this is an IMPLEMENTATION of the F4-locked IGps contract, not a new
// contract — the register's own F13 precedent ("adapters are implementations, not contracts").
//
// UNLIKE the absent vision-side classes, this header carries NO composition wiring rule (no
// kInstall… constant — compare absent_tag_source.hpp). GpsCorrector PULLS state inside
// propose() and keeps no per-poll liveness record, and a permanently false hasFix() is a mode
// the seam itself blesses — so installing a GpsCorrector over an AbsentGps is bit-identical to
// installing no corrector at all (proven in test/absent_device_test.cpp), and no wiring
// decision hangs on the device being absent. Absence must gate wiring only where a consumer
// POLLS and records liveness; that is the tag/vision side's problem, not this one's.

#include "shulib/hal/gps.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal {

/// "This robot has no GPS", as an IGps. Contract-legal with ZERO changes to any consumer —
/// hal/gps.hpp blesses it in as many words: "A permanently false hasFix() is a SUPPORTED
/// mode, not a fault: Driving Skills runs on a field with no GPS strip and the estimator
/// dead-reckons." Every member returns a compile-time constant: hasFix() is false forever,
/// pose() is the finite origin forever, rmsError() is one large finite "no information"
/// figure forever. Stateless, allocation-free, never throws. One honesty obligation rides
/// with it (E1's principle 5, T5's no-SD-card precedent): an absent GPS reads exactly like a
/// GPS that is merely off the strip, so the COMPOSITION ROOT that wires this must announce
/// the absence once at boot — silence here is a wiring decision, and wiring decisions get
/// said out loud.
class AbsentGps final : public IGps {
public:
    /// The "no information" RMS figure, in inches: ~16 miles, astronomically wider than any
    /// field (a VEX field's diagonal is ~204 in), so no plausible quality gate can trust a
    /// fix carrying it. WHY THIS CONSTANT AND NOT A SENTINEL: infinity() violates the seam's
    /// stated finite/non-negative contract and trips this tree's finite guards; and
    /// numeric_limits<double>::max() is finite in name only — the first downstream multiply
    /// (GpsCorrector's σ_meas = rmsTrustFactor · rms, then σ² in its confidence formula)
    /// overflows it to inf, manufacturing the exact non-finite the contract bans one
    /// arithmetic step later. (2 · 1e6)² ≈ 4e12 stays comfortably inside double range
    /// through every formula that consumes it.
    static constexpr double kNoInformationRmsInches = 1.0e6;

    /// The origin — a fixed, FINITE pose, forever, and it never throws: exactly what the seam
    /// requires of pose() while hasFix() is false ("UNSPECIFIED but MUST be finite"). Since
    /// hasFix() is permanently false here, a contract-following caller never reads this as
    /// truth; the origin is simply the cheapest finite value that honors the letter of the
    /// contract for callers that read it anyway.
    [[nodiscard]] math::Pose2d pose() const override { return math::Pose2d{}; }

    /// kNoInformationRmsInches, forever — finite and non-negative as the seam demands, and
    /// large enough that a corrector treating it as a real confidence weights the "fix" to
    /// (almost exactly) nothing.
    [[nodiscard]] units::Length rmsError() const override {
        return units::Length{kNoInformationRmsInches};
    }

    /// False, forever — the seam-blessed "SUPPORTED mode, not a fault". The estimator
    /// dead-reckons for the life of the run, exactly as it does for a whole Driving Skills
    /// run on a stripless field.
    [[nodiscard]] bool hasFix() const override { return false; }
};

}  // namespace shulib::hal
