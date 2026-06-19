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
    int id;                    // AprilTag id within the configured family
    math::Pose2d poseInRobot;  // tag pose relative to the robot (canonical body frame)
    double confidence;         // [0, 1]
};

/// One visible classified object / color, reduced to a robot-relative bearing.
struct ObjectObservation {
    int classId;          // detected class / color descriptor id
    math::Angle bearing;  // horizontal angle to the object, relative to robot +X (forward)
    double confidence;    // [0, 1]
};

/// AprilTag source (decision #7: V5 AI Vision OR a coprocessor, behind this one seam).
class ITagSource {
public:
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
