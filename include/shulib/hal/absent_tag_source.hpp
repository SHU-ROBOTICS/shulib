#pragma once
//
// AbsentTagSource — the explicit statement that THIS ROBOT HAS NO APRILTAG SOURCE (no AI
// Vision sensor, no coprocessor), as an ITagSource (chunk R3b §6).
//
// Not a test double: hal/fake/FakeTagSource exists to be DRIVEN (setTags()/clear()); this
// class exists to say there is nothing to drive. See absent_gps.hpp's banner for the full
// ruling this family implements — preconditions stay, absence is spelled out at the call
// site (`.tags = &absentTags`), naming (`Null…` rejected: NullSink is a working sink that
// discards, this says no device is installed), and why no Freeze Register row (F13:
// implementations, not contracts).
//
// ── THE LIVENESS TRAP, and the wiring rule below (R3b §6.5) ────────────────────────────────
// This class returning `{}` is byte-indistinguishable from a LIVE camera seeing no tags —
// and AprilTagCorrector's poll() DEPENDS on that distinction meaning something:
// localization/apriltag_corrector.hpp records that "a poll that sees NOTHING is still
// information — 'we looked, the camera is alive, there was no tag'", which is how an
// off-camera path stays distinguishable from a dead vision task. Poll a corrector over THIS
// class and it records "camera alive, no tags" about a robot that has no camera at all — a
// false diagnostic manufactured by the very class added to make absence honest, and
// noTagTicks()/pollCount() stop meaning what their headers say.
//
// RULING: an absent source must not be polled. Absence is a WIRING decision, resolved once at
// construction — not a per-tick value that happens to look like nothing. The composition root
// must not install (or drive) a corrector for a source it knows is absent, and
// kInstallTagCorrector below is that rule as a compile-time answer it can cite.

#include <concepts>
#include <vector>

#include "shulib/hal/vision.hpp"

namespace shulib::hal {

/// "This robot has no AprilTag source", as an ITagSource. tags() is `{}` forever — a
/// default-constructed std::vector allocates nothing, so the class is allocation-free,
/// stateless and never throws. DO NOT POLL IT: an empty frame from this class is a false
/// "camera alive, saw nothing" record (banner, §6.5) — consult kInstallTagCorrector at the
/// wiring site instead of installing a corrector and hoping empty returns are harmless. And
/// as with every Absent device, the composition root that wires this announces the absence
/// once at boot (E1's principle 5): a robot with no camera should SAY so, not merely never
/// see anything.
class AbsentTagSource final : public ITagSource {
public:
    /// No tags, ever — an empty vector, which allocates nothing. NOT "no tags visible this
    /// tick": there is no camera to see with, which is exactly why a corrector must never be
    /// polled over it (class note).
    [[nodiscard]] std::vector<TagObservation> tags() const override { return {}; }
};

/// The §6.5 wiring rule as a compile-time answer: SHOULD the composition root install (and
/// drive) an AprilTagCorrector over the tag source it declared as `TagSourceT`? True for
/// every present source — a hal/pros adapter obviously, and FakeTagSource too, since a fake
/// is a live source a test drives (present-but-empty is REAL information: "we looked, no
/// tag"). Specialized false for AbsentTagSource below, and for any future Absent tag source
/// alongside its own class. Zero-cost and resolved where absence itself is resolved — once,
/// at construction, from the CONCRETE type the composition root already knows — so it needs
/// no isPresent() on the F4-frozen seam, no RTTI, and no per-tick check. LIMITATION, stated:
/// it answers from the DECLARED type. Ask it where the device member is declared; through an
/// `ITagSource*` it can only answer for the seam (true), not for the device behind it.
template <class TagSourceT>
    requires std::derived_from<TagSourceT, ITagSource>
inline constexpr bool kInstallTagCorrector = true;

/// The ruling itself: no corrector is installed or driven over an absent tag source, so the
/// corrector's liveness counters keep meaning what their headers say — pollCount() == 0 is
/// "nobody polls", never "there is no camera". Flipping this to true is precisely the §6.5
/// defect: a manufactured "camera alive, no tags" record on a robot with no camera
/// (test/absent_device_test.cpp holds the demonstration).
template <>
inline constexpr bool kInstallTagCorrector<AbsentTagSource> = false;

}  // namespace shulib::hal
