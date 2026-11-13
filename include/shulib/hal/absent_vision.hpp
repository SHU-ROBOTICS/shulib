#pragma once
//
// AbsentVision — the explicit statement that THIS ROBOT HAS NO OBJECT/COLOR DETECTION SOURCE,
// as an IVision (chunk R3b §6).
//
// Not a test double: hal/fake/FakeVision exists to be DRIVEN (setObjects()/clear()); this
// class exists to say there is nothing to drive. See absent_gps.hpp's banner for the family
// ruling — preconditions stay, absence is spelled out at the call site
// (`.vision = &absentVision`), naming (`Null…` rejected), no Freeze Register row (F13) — and
// absent_tag_source.hpp's banner for the liveness trap the wiring rule below exists to block.
//
// IVision has no consumer in the tree yet (M4 owns manipulation targeting), so unlike the tag
// side there is no corrector to mis-record TODAY. The rule is still declared NOW, because the
// trap is identical in shape the moment an M4 consumer polls objects() and keeps a "vision
// alive, nothing seen" record: an AbsentVision returning `{}` is byte-indistinguishable from
// a live detector with nothing in view. Absence is a wiring decision, resolved once at
// construction — M4 inherits the rule instead of rediscovering the defect.

#include <concepts>
#include <vector>

#include "shulib/hal/vision.hpp"

namespace shulib::hal {

/// "This robot has no object/color detection source", as an IVision. objects() is `{}`
/// forever — a default-constructed std::vector allocates nothing, so the class is
/// allocation-free, stateless and never throws. DO NOT POLL IT: consult kInstallVisionPoller
/// at the wiring site (absent_tag_source.hpp's banner holds the reasoning). As with every
/// Absent device, the composition root that wires this announces the absence once at boot
/// (E1's principle 5).
class AbsentVision final : public IVision {
public:
    /// No detections, ever — an empty vector, which allocates nothing. NOT "nothing in view
    /// this tick": there is no detector to view with, which is why no consumer should be
    /// polling it (class note).
    [[nodiscard]] std::vector<ObjectObservation> objects() const override { return {}; }
};

/// The vision-side sibling of kInstallTagCorrector (absent_tag_source.hpp — the reasoning
/// lives there): SHOULD the composition root install (and drive) a vision-rate consumer over
/// the source it declared as `VisionT`? True for every present source, including FakeVision
/// (a fake is a live source a test drives). Specialized false for AbsentVision below.
/// Declared before any consumer exists so M4's poller is built against the rule rather than
/// against the trap. Same limitation: it answers from the DECLARED type, so ask it where the
/// device member is declared, never through an `IVision*`.
template <class VisionT>
    requires std::derived_from<VisionT, IVision>
inline constexpr bool kInstallVisionPoller = true;

/// The ruling applied to the absent detector: no vision-rate consumer is installed or driven
/// over it, so no "vision alive, nothing seen" record can ever be manufactured for a robot
/// with no detector.
template <>
inline constexpr bool kInstallVisionPoller<AbsentVision> = false;

}  // namespace shulib::hal
