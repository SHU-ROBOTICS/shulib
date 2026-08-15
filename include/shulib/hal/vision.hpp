#pragma once
//
// IVision / ITagSource — the AI Vision seams. Decision #7 (locked): the V5 AI Vision
// sensor AND a coprocessor (Pi) both sit behind ITagSource. The V5 sensor reports BOTH
// AprilTags (4 image corners + id) and classified objects/colors (bounding box + score),
// so one hal/pros adapter implements BOTH interfaces from it; a Pi adapter can too.
//
// CANONICAL FORMS — the raw V5 pixel corners / boxes are reduced to canonical
// robot-relative quantities in the adapter ("convert once at the edge", §7):
//  * ITagSource yields each visible AprilTag as a RELATIVE POSE in the robot BODY frame
//    (+X forward, +Y left — F1), a planar reduction of the tag's 6-DOF pose suitable for
//    ground-plane localization. The corners→pose PnP (needs camera intrinsics + tag size)
//    is a pure, host-testable function built with the M3 AprilTagCorrector; at M1 this is
//    the seam the corrector reads.
//  * IVision yields each object as a class id + BEARING (horizontal angle to the object,
//    relative to robot +X) + confidence, for manipulation targeting (M4). The
//    box-center→bearing reduction (needs camera FOV) is likewise an M4 pure function.
//
// Detections are returned BY VALUE: vision runs OFF the 10 ms control hot path (the
// adapter polls it at a lower rate). The consumer timestamps via IClock and owns
// staleness/latency handling (the corrector, M3) — the detection data itself is
// timestamp-free, like the GPS pose.

#include <vector>

#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"

namespace shulib::hal {

/// One visible AprilTag, reduced to a robot-relative planar pose.
struct TagObservation {
    /// AprilTag id within the configured family. DEFAULTED, like every other value struct in
    /// the tree: these two were the only sensor observations with no default member
    /// initializers, so `TagObservation t; t.poseInRobot = …;` left `id` and `confidence`
    /// INDETERMINATE — and the corrector's screen cannot save it, because the screen IS the
    /// read (`!std::isfinite(obs.confidence)` on an indeterminate double is already UB), while
    /// an indeterminate id that happens to hit a real map entry yields a confident fix against
    /// the wrong tag. Aggregate initialisation is unaffected. A corrector looks this up in its map of known
    /// field placements; an id with no map entry is discarded, never guessed at.
    int id = 0;
    /// Tag pose RELATIVE to the robot, canonical body frame (F1: +X forward, +Y left, heading
    /// CCW-positive), inches and radians. Already the PLANAR reduction: the tag's height above
    /// the camera, its pitch and its roll were discarded at the edge and are not recoverable.
    math::Pose2d poseInRobot{};
    /// Detector confidence, [0, 1]. Not a probability that the pose is right — a corrector
    /// DIVIDES its measurement sigma by it, so larger means a tighter fix, and 0 means unusable.
    double confidence = 0.0;
};

/// One visible classified object / color, reduced to a robot-relative bearing.
struct ObjectObservation {
    /// Detected class / color descriptor id, as configured on the detector. Opaque to shulib:
    /// nothing here maps an id to a meaning — the manipulation code that asked for it owns that.
    int classId = 0;
    /// Horizontal angle to the object measured from robot +X (forward), CCW-positive, wrapped to
    /// (-π, π]. A BEARING only: a bounding box carries no range, so this says which way to
    /// turn and never how far to drive.
    math::Angle bearing{};
    /// Detector confidence, [0, 1]. Carried for M4 targeting to rank candidates with; no
    /// consumer in the tree reads it yet, so nothing currently gates on a low value.
    double confidence = 0.0;
};

/// AprilTag source (decision #7: V5 AI Vision OR a coprocessor, behind this one seam).
class ITagSource {
public:
    /// The polymorphic-base boilerplate, and why it is spelled out: the destructor is virtual so
    /// deleting through `ITagSource*` is well-defined, and declaring it suppresses the implicit
    /// copy/move, which are therefore re-defaulted. The seam holds no state, so all five are
    /// trivial — an implementation is REFERENCED and never owned (RobotContext keeps a
    /// non-owning pointer, and the adapter must outlive the context).
    virtual ~ITagSource() = default;
    ITagSource() = default;
    ITagSource(const ITagSource&) = default;
    ITagSource(ITagSource&&) = default;
    ITagSource& operator=(const ITagSource&) = default;
    ITagSource& operator=(ITagSource&&) = default;

    /// AprilTags currently visible, each as a relative pose in the robot frame.
    [[nodiscard]] virtual std::vector<TagObservation> tags() const = 0;
};

/// Object / color detection source (manipulation targeting, M4).
class IVision {
public:
    /// Same polymorphic-base boilerplate as ITagSource, and for the same reason: a virtual
    /// destructor for delete-through-base, with copy/move re-defaulted after declaring it. It
    /// matters here that this base is stateless — decision #7 expects ONE adapter to inherit
    /// both this and ITagSource off a single V5 AI Vision sensor, and two empty bases cost that
    /// adapter nothing.
    virtual ~IVision() = default;
    IVision() = default;
    IVision(const IVision&) = default;
    IVision(IVision&&) = default;
    IVision& operator=(const IVision&) = default;
    IVision& operator=(IVision&&) = default;

    /// Classified objects / colors currently visible.
    [[nodiscard]] virtual std::vector<ObjectObservation> objects() const = 0;
};

}  // namespace shulib::hal
