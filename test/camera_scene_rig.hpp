#pragma once
//
// The rig the camera's MEANING tests share (chunk VIS1-A2): the mount the library BELIEVES and the TRUE
// camera the host camera model sees through (moved inside the stated tolerances: 0.2 in forward and up,
// 0.8 degree down, 0.7 degree yawed, the field of view 75.5 x 62 against the believed 74 x 63), the team
// numbers the brief's probes used, a robot parked square to the +x wall's Toggle, and the scenes'
// helpers. Test-only (not a *_test.cpp: no TEST_CASE here). Its numbers are the PROBES' — the team's own
// come from the bench; nothing here is a library default.

#include <cmath>
#include <cstdint>

#include "shulib/config/camera_config.hpp"
#include "shulib/hal/vision_frames.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/sim/camera_plant.hpp"
#include "shulib/units/quantity.hpp"
#include "shulib/vision/camera_geometry.hpp"
#include "shulib/vision/override_elements.hpp"
#include "shulib/vision/piece_finder.hpp"
#include "shulib/vision/toggle_tracker.hpp"

namespace camrig {

namespace config = shulib::config;
namespace hal = shulib::hal;
namespace sim = shulib::sim;
namespace vision = shulib::vision;
using shulib::units::Length;
using shulib::units::Time;

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kWallX = 70.2;   // the +x wall's inner face (pA10: 140.41 / 2)
inline constexpr double kLensFwd = 7.0;  // the lens 7 in forward of the robot's tracking centre
inline const vision::override2026::ToggleFace& kFace = vision::override2026::kToggleFace;

/// The mount the library believes: `h` up, pitched `pitch`, +-0.25 in and +-1 degree.
inline config::CameraPlacement believed(double h, double pitch = 0.0) {
    config::CameraPlacement p{};
    p.measured = true;
    p.forward = Length{kLensFwd};
    p.height = Length{h};
    p.pitchDeg = pitch;
    p.positionTolerance = Length{0.25};
    p.heightTolerance = Length{0.25};
    p.yawToleranceDeg = 1.0;
    p.pitchToleranceDeg = 1.0;
    p.provenance = "host test";
    return p;
}

/// The library's model of that mount.
inline vision::CameraModel model(double h, double pitch = 0.0) {
    return vision::CameraModel{vision::kAiVisionOptics, believed(h, pitch)};
}

/// The TRUE camera: the believed one moved INSIDE the stated tolerances.
inline sim::TrueCamera truth(double h, double pitch = 0.0) {
    sim::TrueCamera t{};
    t.forward = kLensFwd + 0.2;
    t.height = h + 0.2;
    t.pitchDeg = pitch - 0.8;
    t.yawDeg = 0.7;
    t.hfovDeg = 75.5;
    t.vfovDeg = 62.0;
    return t;
}

/// The tracker numbers the probes used (Red 1, Blue 2, Yellow 3; hold 0.15 s; alive 0.10 s; ...).
inline vision::ToggleTrackerConfig tcfg(double windowDeg = 25.0) {
    vision::ToggleTrackerConfig c{};
    c.ids = {1, 2, 3};
    c.holdSeconds = 0.15;
    c.aliveSeconds = 0.10;
    c.minAspect = 3.0;
    c.minLengthFraction = 0.6;
    c.maxLengthFraction = 1.3;
    c.minRangeIn = 2.0;
    c.maxRangeIn = 96.0;
    c.bearingWindowDeg = windowDeg;
    c.frozenMoveIn = 2.0;
    c.frozenTurnDeg = 2.0;
    return c;
}

/// A pose bound: +-1 in and +-1 degree unless said.
inline vision::PoseBound poseAt(double x, double y, double headingRad, double posTol = 1.0, double headTol = 1.0) {
    vision::PoseBound b{};
    b.pose = shulib::math::Pose2d{Length{x}, Length{y}, shulib::math::Angle::radians(headingRad)};
    b.positionTolerance = Length{posTol};
    b.headingToleranceDeg = headTol;
    return b;
}

/// The robot's x that puts its LENS `lensToWall` in from the +x wall, square to it.
inline double robotXFor(double lensToWall) { return kWallX - lensToWall - kLensFwd; }

/// The +x wall's Toggle field face alone (no bottom face), `colour`, centred `y` along the wall.
inline sim::ColourQuad face(std::uint8_t colour, double y = 0.0, double bottom = kFace.bottomIn,
                            double setBack = 1.2) {
    return sim::CameraPlant::toggleFace(kWallX, y, -1.0, 0.0, kFace.lengthIn, bottom, kFace.topIn, setBack, colour);
}

/// A seated Toggle on the +x wall: field face `fieldColour`, bottom face `bottomColour` (0 = none).
inline void addToggle(sim::CameraPlant& plant, std::uint8_t fieldColour, std::uint8_t bottomColour,
                      double bottom = kFace.bottomIn, double setBack = 1.2) {
    plant.addToggle(kWallX, 0.0, -1.0, 0.0, kFace.lengthIn, bottom, kFace.topIn, setBack, fieldColour, bottomColour);
}

/// Park with the lens `lensToWall` from the wall, heading `heading`, for `secs` of reads every 33 ms,
/// advancing `t`; the view at the end.
inline vision::ToggleView park(sim::CameraPlant& plant, vision::ToggleTracker& tr, double lensToWall, double& t,
                               double secs, double heading = 0.0) {
    const double rx = robotXFor(lensToWall);
    for (double e = 0.0; e < secs; e += 0.033) {
        t += 0.033;
        tr.update(plant.read(rx, 0.0, heading, Time{t}), poseAt(rx, 0.0, heading));
    }
    return tr.view(t);
}

/// `reads` reads from the robot at (rx, ry, heading) with the bound `pb`, 33 ms apart; the view at the end.
inline vision::ToggleView readsAt(sim::CameraPlant& plant, vision::ToggleTracker& tr, double rx, double ry,
                                  double heading, const vision::PoseBound& pb, int reads, double& t) {
    for (int i = 0; i < reads; ++i) {
        t += 0.033;
        tr.update(plant.read(rx, ry, heading, Time{t}), pb);
    }
    return tr.view(t);
}

/// A hand-made usable frame of `n` detections stamped `at`.
inline hal::VisionFrame frameOf(const hal::Detection* ds, int n, double at) {
    hal::VisionFrame f{};
    f.status = hal::FrameStatus::Ok;
    f.model = hal::ModelCheck::Matches;
    f.readAt = Time{at};
    for (int i = 0; i < n; ++i) {
        f.detections[f.count++] = ds[i];
    }
    f.reportedCount = n;
    return f;
}

/// A Colour detection.
inline hal::Detection colourBox(std::uint8_t id, int left, int top, int width, int height) {
    hal::Detection d{};
    d.kind = hal::DetectionKind::Colour;
    d.id = id;
    d.left = static_cast<std::int16_t>(left);
    d.top = static_cast<std::int16_t>(top);
    d.width = static_cast<std::int16_t>(width);
    d.height = static_cast<std::int16_t>(height);
    return d;
}

/// An Object detection.
inline hal::Detection objectBox(std::uint8_t cls, int left, int top, int width, int height, int score = 90) {
    hal::Detection d{};
    d.kind = hal::DetectionKind::Object;
    d.id = cls;
    d.left = static_cast<std::int16_t>(left);
    d.top = static_cast<std::int16_t>(top);
    d.width = static_cast<std::int16_t>(width);
    d.height = static_cast<std::int16_t>(height);
    d.score = score;
    return d;
}

// ── the piece finder's rig (a lens 10 in up, pitched 10 degrees down) ───────────────
inline constexpr std::uint8_t kYellowPin = 3;
inline constexpr std::uint8_t kCup = 4;

/// The finder numbers the probes used.
inline vision::PieceFinderConfig fcfg() {
    vision::PieceFinderConfig c{};
    c.holdSeconds = 0.15;
    c.aliveSeconds = 0.1;
    c.minScore = 50.0;
    c.standingAspect = 0.8;
    c.lyingAspect = 1.5;
    c.viewMarginPx = 4.0;
    c.raisedMinIn = 5.0;  // the scenes' Goal top is 5.8 in (the probes' F3)
    return c;
}

/// The TRUE camera of the finder's scenes: believed 10 in up, pitched -10; the truth -0.2 in forward,
/// +0.2 in up, -0.8 degree, -0.7 degree yaw, 72.8 x 64 degrees.
inline sim::TrueCamera pieceTruth() {
    sim::TrueCamera t{};
    t.forward = kLensFwd - 0.2;
    t.height = 10.2;
    t.pitchDeg = -10.8;
    t.yawDeg = -0.7;
    t.hfovDeg = 72.8;
    t.vfovDeg = 64.0;
    return t;
}

/// A yellow Pin standing at (x, y), its base `baseZ` up (lying when `lying`).
inline sim::Element pin(double x, double y, double baseZ = 0.0, bool lying = false) {
    sim::Element e{};
    e.classId = kYellowPin;
    e.x = x;
    e.y = y;
    e.baseZ = baseZ;
    e.lying = lying;
    return e;
}

/// A route's target: the yellow Pin expected at (x, y) within `tol`, standing on the floor.
inline vision::PieceTarget yellowPinAt(double x, double y = 0.0, double tol = 5.0,
                                       vision::Posture want = vision::Posture::OnFloor) {
    vision::PieceTarget tg{};
    tg.classMask = 1U << kYellowPin;
    tg.x = x;
    tg.y = y;
    tg.toleranceIn = tol;
    tg.want = want;
    return tg;
}

/// `secs` of reads every 33 ms with the robot at (rx, 0) facing +x, its bound `tol` in / 1 degree; the
/// answer at the end.
inline vision::PieceFinder::Result look(sim::CameraPlant& plant, vision::PieceFinder& f, double rx, double tol,
                                        double& t, double secs) {
    const vision::PoseBound pb = poseAt(rx, 0.0, 0.0, tol, 1.0);
    for (double e = 0.0; e < secs; e += 0.033) {
        t += 0.033;
        f.update(plant.read(rx, 0.0, 0.0, Time{t}), pb);
    }
    return f.answer(t);
}

}  // namespace camrig
