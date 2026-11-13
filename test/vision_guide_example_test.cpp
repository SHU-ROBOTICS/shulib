// Guide chapter 20 ("The camera"), compiled and run (cases guide-20a to guide-20e; chunk VIS1-A2). Every
// ```cpp line the chapter shows is a line of this file — tools/api_doc_tool.py's check-examples fails the
// build if one drifts — and each case below checks what the chapter says that listing does. The camera is
// the host camera model (sim/camera_plant.hpp) behind the frame seam, so the listings run against a scene.

#include "doctest.h"

#include <cmath>
#include <cstdint>

#include "shulib/config/camera_config.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/vision_frames.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/sim/camera_plant.hpp"
#include "shulib/units/quantity.hpp"
#include "shulib/vision/camera_feed.hpp"
#include "shulib/vision/camera_geometry.hpp"
#include "shulib/vision/override_elements.hpp"
#include "shulib/vision/piece_finder.hpp"
#include "shulib/vision/toggle_tracker.hpp"

namespace guide_camera {
namespace hal = shulib::hal;
namespace vision = shulib::vision;
namespace override2026 = shulib::vision::override2026;

// ═══ THE TEAM'S NUMBERS — every one from the bench; none has a library default ═════════════════
inline constexpr vision::ToggleTrackerConfig kToggleNumbers{
    .ids = {.red = 1, .blue = 2, .yellow = 3},  // the descriptors taught in VEXcode's AI Vision Utility
    .holdSeconds = 0.15,       // the same colour this long before it counts
    .aliveSeconds = 0.10,      // and a usable read no older than this
    .minAspect = 3.0,          // a face's band: at least 3 times wider than tall
    .minLengthFraction = 0.6,  // and 0.6 .. 1.3 of the Toggle's 25.8 in long
    .maxLengthFraction = 1.3,
    .minRangeIn = 2.0,         // "in front of my arm": 2 .. 96 in from the lens,
    .maxRangeIn = 96.0,
    .bearingWindowDeg = 25.0,  // within 25 degrees of straight ahead
    .frozenMoveIn = 2.0,       // the lens moved 2 in or the robot turned 2 degrees
    .frozenTurnDeg = 2.0,      // and the band did not move a pixel: "camera frozen?"
};
inline constexpr vision::PieceFinderConfig kPieceNumbers{
    .holdSeconds = 0.15,     // FOUND must hold this long
    .aliveSeconds = 0.10,
    .minScore = 50.0,        // the model's score below this is ignored
    .lyingAspect = 1.5,      // a box 1.5 times wider than tall lies on its side
    .standingAspect = 0.8,   // one at most 0.8 stands
    .viewMarginPx = 4.0,     // the place must be 4 px inside the picture
    .raisedMinIn = 5.0,      // the lowest a raised piece sits on this field (a Goal's top), inches
};

// ═══ ONE LOOP READS THE CAMERA FOR EVERYONE ═════════════════════════════════════════════════════
struct CameraUser {
    vision::CameraFeed feed;
    vision::ToggleTracker toggle;
    vision::PieceFinder pins;

    CameraUser(hal::IVisionFrames& camera, const hal::IClock& clock, const vision::CameraModel& model)
        : feed{camera, clock, {.minReadPeriodSeconds = 0.033}},
          toggle{kToggleNumbers, model},
          pins{kPieceNumbers, model} {}

    void tick(const vision::PoseBound& pose) {  // once per pass of the loop the program already runs
        if (feed.tick()) {
            toggle.update(feed.latest(), pose);
            pins.update(feed.latest(), pose);
        }
    }
};

// ═══ (test only) the host camera model behind the frame seam, seen from one pose ════════════════
class SceneCamera final : public hal::IVisionFrames {
public:
    SceneCamera(shulib::sim::CameraPlant& plant, const hal::IClock& clock, double x, double y, double h)
        : plant_{plant}, clock_{clock}, x_{x}, y_{y}, h_{h} {}
    [[nodiscard]] hal::VisionFrame read() noexcept override {
        ++reads;
        return plant_.read(x_, y_, h_, clock_.now());
    }
    [[nodiscard]] hal::CameraStatus status() const noexcept override { return {}; }
    int reads = 0;

private:
    shulib::sim::CameraPlant& plant_;
    const hal::IClock& clock_;
    double x_;
    double y_;
    double h_;
};

// The camera 10 in up, tipped 10 degrees down, 7 in ahead of the tracking centre, measured to +-0.25 in and
// 1 degree.
inline shulib::config::CameraPlacement measuredMount() {
    shulib::config::CameraPlacement p{};
    p.measured = true;
    p.forward = shulib::units::Length{7.0};
    p.height = shulib::units::Length{10.0};
    p.pitchDeg = -10.0;
    p.positionTolerance = shulib::units::Length{0.25};
    p.heightTolerance = shulib::units::Length{0.25};
    p.yawToleranceDeg = 1.0;
    p.pitchToleranceDeg = 1.0;
    p.provenance = "guide example";
    return p;
}

// The scene: the +x wall's Toggle showing RED (its bottom face YELLOW), and a yellow Pin standing 26 in
// in front of the lens, 6 in to the right; the robot at (33.2, 0) facing the wall, the lens 30 in from it.
constexpr double kRobotX = 33.2;
inline void buildScene(shulib::sim::CameraPlant& plant) {
    const override2026::ToggleFace& face = override2026::kToggleFace;
    plant.addToggle(70.2, 0.0, -1.0, 0.0, face.lengthIn, face.bottomIn, face.topIn, 1.2, 1, 3);
    shulib::sim::Element pin{};
    pin.classId = static_cast<std::uint8_t>(override2026::ModelClass::YellowPin);
    pin.x = kRobotX + 7.0 + 26.0;
    pin.y = -6.0;
    plant.addElement(pin);
}

inline vision::PoseBound poseNow() {
    vision::PoseBound pose{};
    pose.pose = shulib::math::Pose2d{shulib::units::Length{kRobotX}, shulib::units::Length{0.0},
                                     shulib::math::Angle::radians(0.0)};
    pose.positionTolerance = shulib::units::Length{1.0};
    pose.headingToleranceDeg = 1.0;
    return pose;
}

}  // namespace guide_camera

using namespace guide_camera;

TEST_CASE("guide-20a: the team's numbers pass every check; the UNSET defaults do not") {
    CHECK(vision::toggleTrackerConfigProblem(kToggleNumbers) == nullptr);
    CHECK(vision::pieceFinderConfigProblem(kPieceNumbers) == nullptr);
    CHECK(vision::cameraFeedConfigProblem({.minReadPeriodSeconds = 0.033}) == nullptr);
    CHECK(vision::toggleTrackerConfigProblem(vision::ToggleTrackerConfig{}) != nullptr);
    CHECK(vision::pieceFinderConfigProblem(vision::PieceFinderConfig{}) != nullptr);
    CHECK(vision::cameraFeedConfigProblem(vision::CameraFeedConfig{}) != nullptr);
}

TEST_CASE("guide-20b: one loop ticked every 10 ms reads the camera 25 times a second -- the 0.033-s period rounded "
          "up to the loop's tick, every fourth tick (40 ms); never once per tick -- and both consumers see the same "
          "frame") {
    shulib::sim::CameraPlant plant{shulib::sim::TrueCamera{.forward = 7.0, .height = 10.0, .pitchDeg = -10.0}};
    buildScene(plant);
    shulib::hal::fake::FakeClock clock;
    SceneCamera camera{plant, clock, kRobotX, 0.0, 0.0};
    const vision::CameraModel model{vision::kAiVisionOptics, measuredMount()};
    CameraUser user{camera, clock, model};
    for (int tick = 0; tick < 100; ++tick) {  // one second of a 10 ms loop
        user.tick(poseNow());
        clock.advance(shulib::units::Time{0.010});
    }
    CHECK(camera.reads == 25);
    CHECK(user.feed.stats().reads == static_cast<std::uint32_t>(camera.reads));
}

TEST_CASE("guide-20c: the tracker reports the RED face once it has held -- a colour SEEN, never 'ours'") {
    shulib::sim::CameraPlant plant{shulib::sim::TrueCamera{.forward = 7.0, .height = 10.0, .pitchDeg = -10.0}};
    buildScene(plant);
    shulib::hal::fake::FakeClock clock;
    SceneCamera camera{plant, clock, kRobotX, 0.0, 0.0};
    CameraUser user{camera, clock, vision::CameraModel{vision::kAiVisionOptics, measuredMount()}};
    bool sawRed = false;
    for (int tick = 0; tick < 50; ++tick) {
        user.tick(poseNow());
        const vision::ToggleView seen = user.toggle.view(clock.now().value());
        if (seen.colour == vision::ToggleColour::Red) {
            // a RED face is SEEN where a seated face could be — the far-range cue, never a set Toggle
            sawRed = true;
        }
        if (tick == 0) {
            CHECK(seen.why == vision::ToggleWhy::NotHeldYet);  // one read is never enough
        }
        clock.advance(shulib::units::Time{0.010});
    }
    CHECK(sawRed);
}

TEST_CASE("guide-20d: the finder takes the named Pin -- FOUND, its fix holding the Pin -- and any other verdict "
          "skips the step") {
    shulib::sim::CameraPlant plant{shulib::sim::TrueCamera{.forward = 7.0, .height = 10.0, .pitchDeg = -10.0}};
    buildScene(plant);
    shulib::hal::fake::FakeClock clock;
    SceneCamera camera{plant, clock, kRobotX, 0.0, 0.0};
    CameraUser user{camera, clock, vision::CameraModel{vision::kAiVisionOptics, measuredMount()}};
    user.pins.setTarget({.classMask = 1U << static_cast<unsigned>(override2026::ModelClass::YellowPin),
                         .x = 66.2, .y = -6.0, .toleranceIn = 6.0, .want = vision::Posture::OnFloor});
    bool took = false;
    bool skipped = false;
    for (int tick = 0; tick < 50; ++tick) {
        user.tick(poseNow());
        const vision::PieceFinder::Result answer = user.pins.answer(clock.now().value());
        switch (answer.verdict) {
            case vision::PieceVerdict::Found:  // drive to answer.sighting.fix (L4's job)
                took = true;
                CHECK(answer.sighting.fix.radius < 6.0);
                CHECK(std::hypot(answer.sighting.fix.x - 66.2, answer.sighting.fix.y + 6.0) <= answer.sighting.fix.radius);
                break;
            default:  // NONE, AMBIGUOUS, NOT IN VIEW, UNKNOWN: skip the step — never another piece
                skipped = true;
                break;
        }
        clock.advance(shulib::units::Time{0.010});
    }
    CHECK(took);
    CHECK(skipped);  // before the hold, UNKNOWN
}

TEST_CASE("guide-20e: a pose paired with a frame is the pose at the IMAGE's time -- widen it by the measured age") {
    const double speedInPerS = 30.0;
    const double turnDegPerS = 90.0;
    const double kImageAgeSeconds = 0.05;  // the bench's measurement (Step 0.7) — until then, stop to look
    const vision::PoseBound atRead = poseNow();
    const vision::PoseBound atImage = vision::boundAtImage(atRead, speedInPerS, turnDegPerS, kImageAgeSeconds);
    CHECK(atImage.positionTolerance.value() == doctest::Approx(1.0 + 1.5));
    CHECK(atImage.headingToleranceDeg == doctest::Approx(1.0 + 4.5));
}
