#pragma once
//
// ToggleCamera — the robot's camera answer for the Toggle, as an AUTON reaches it (chunk AF1S-A2,
// 2026-11-15; the AF1S brief's §1.3 — L6-A2's optional-slot pattern, the context's SECOND optional slot).
//
// ── Two pieces, one header ──────────────────────────────────────────────────────────────────────
//   * ToggleCameraConfig — the robot file's TOGGLE BLOCK (config::SensorsConfig::toggleCamera): the TEAM's
//     tracker numbers (vision::ToggleTrackerConfig) and the camera feed's read period
//     (vision::CameraFeedConfig), every one UNSET by default, so every robot file written before it
//     builds the same robot. It sits BESIDE the camera's block, not inside it: config/camera_config.hpp
//     is included by the tracker's and the feed's own headers, so config::CameraConfig cannot hold their
//     types (a circular include — the brief's §1.3 named CameraConfig; its re-check R9 met the same wall
//     for the Quadrant colours).
//   * ToggleCamera — the SLOT: ONE vision::CameraFeed over the robot's camera, the vision::CameraModel of
//     its MEASURED mount, and the tracker numbers. robot::Robot fills it; the context's optional
//     `toggleCamera` slot (chassis/robot_context.hpp) points at it; the everyday
//     competition::checkAndFlipToggle(ctx, ...) reads it through `ctx.chassis.deps().ctx`, exactly as the
//     everyday wall reset reads the wall sensors. It holds the tracker's CONFIG, not a tracker: the step
//     builds a FRESH tracker for every visit (AF1S-A1), so the hold that counts always began inside it.
//
// ── When the slot is READY ──────────────────────────────────────────────────────────────────────
// The block DECLARED (any number in it set), toggleCameraProblem() nullptr — a camera declared; its mount
// MEASURED and inside the geometry's limits (vision::cameraModelProblem); the tracker's numbers and the
// feed's accepted (toggleTrackerConfigProblem, cameraFeedConfigProblem); the tracker's three colour ids
// among the colours the camera's settings write (an id the camera is never taught would read UNKNOWN
// for ever) — AND the camera BUILT by the provider. Otherwise NOT READY, with the FIRST reason in that
// order (absence()). A refusal never refuses the robot: an unusable Toggle block costs the skills step its
// camera (NO CAMERA, the reason in its [TGL] line), never the robot its drive — the camera's own rule
// (VIS1's D5) and the brief's mutation M-A2-2. config::RobotConfig::valid() does not check the block.
//
// ── Why the slot is "declared, then ready" — not "built, or null" ───────────────────────────────
// The brief has the context's slot null on any refusal. It cannot be: motion::MotionScheduler's shadow
// context copies every slot pointer when the CHASSIS is built, inside robot::Robot's member initializers,
// and the camera is built in the Robot's BODY, last (VIS1-A1's order: every provider request before it
// stays where it was). So — L6-A2's wall-sensor slot exactly — the context's pointer is decided by the
// CONFIG (the block declared), the object it points at is filled in the body, and ready() says whether it
// was built (a wall side's sensor is ABSENT inside a present slot the same way). robot::Robot's own
// toggleCamera() is the brief's "built, or null", with toggleCameraAbsence() its sentence.
//
// PROS-free, sim-free, allocation-free; single-task. The slot never reads the camera by itself: only the
// step's own waits tick its feed. NOTHING HERE HAS RUN ON A ROBOT.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>

#include "shulib/config/camera_config.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/vision_frames.hpp"
#include "shulib/vision/camera_feed.hpp"
#include "shulib/vision/camera_geometry.hpp"
#include "shulib/vision/toggle_tracker.hpp"

namespace shulib::vision {

/// The robot file's Toggle block (file banner): the team's tracker numbers and the camera feed's read
/// period, UNSET by default. DECLARED as soon as any number in it is set — so a block typed half-way is
/// declared and refused naming the missing number, never silently ignored.
struct ToggleCameraConfig {
    /// The tracker's numbers (toggle_tracker.hpp — every one the TEAM's, from Step 0.3 at the robot).
    ToggleTrackerConfig tracker{};
    /// The feed's read period (camera_feed.hpp — the TEAM's, about 0.033 s for 30 a second; A4 register
    /// HA-192).
    CameraFeedConfig feed{};

    /// True when any number in the block is set (a NaN counts as set: it is refused by name, not ignored).
    /// EVERY field of both configs is listed here: a field added to either must be added here too, or a
    /// block typed with only that field set is silently ignored (test/robot_toggle_camera_test.cpp sets each
    /// one alone).
    [[nodiscard]] constexpr bool declared() const noexcept {
        const ToggleTrackerConfig& t = tracker;
        if (t.ids.red != 0 || t.ids.blue != 0 || t.ids.yellow != 0) {
            return true;
        }
        const double numbers[] = {t.holdSeconds,         t.aliveSeconds,       t.minAspect,      t.minLengthFraction,
                                  t.maxLengthFraction,   t.minRangeIn,         t.maxRangeIn,     t.bearingWindowDeg,
                                  t.frozenMoveIn,        t.frozenTurnDeg,      t.positionToleranceIn,
                                  t.maxPoseToleranceIn,  t.maxFixRadiusIn,     feed.minReadPeriodSeconds};
        for (const double n : numbers) {
            if (n != 0.0) {  // NaN != 0.0 is true
                return true;
            }
        }
        return false;
    }
};

/// What is wrong with a Toggle block for a camera, if anything: `part` names the piece ("" when the
/// sentence names it already) and `why` is the sentence, nullptr when nothing is wrong.
struct ToggleCameraCheck {
    /// The piece the sentence is about, as a prefix ("the Toggle block's tracker: "), or "".
    const char* part = "";
    /// The reason, or nullptr: the block may be built.
    const char* why = nullptr;
};

/// Whether every one of the tracker's three colour ids is a colour the camera's settings WRITE (the
/// camera fires only on a colour it was taught): nullptr, or the first id that is not.
[[nodiscard]] constexpr const char* toggleIdsProblem(const hal::AiVisionSettings& settings,
                                                     const ToggleColourIds& ids) noexcept {
    if (settings.colourCount == 0) {
        return "the camera's settings write no colours -- teach it the Toggle's three faces (the AI Vision "
               "Utility) and type them in the camera's block";
    }
    const std::uint8_t count =
        settings.colourCount < hal::kMaxColourDescriptors ? settings.colourCount : hal::kMaxColourDescriptors;
    const auto written = [&settings, count](std::uint8_t id) {
        for (std::uint8_t i = 0; i < count; ++i) {
            if (settings.colours[i].id == id) {
                return true;
            }
        }
        return false;
    };
    if (!written(ids.red)) {
        return "its RED id is not one of the colours the camera's settings write";
    }
    if (!written(ids.blue)) {
        return "its BLUE id is not one of the colours the camera's settings write";
    }
    if (!written(ids.yellow)) {
        return "its YELLOW id is not one of the colours the camera's settings write";
    }
    return nullptr;
}

/// THE ONE ORDER of the block's checks (file banner, "When the slot is READY", less the camera being BUILT,
/// which only the Robot knows): the block declared; a camera declared and its mount measured and inside
/// the geometry's limits (cameraModelProblem); the tracker's numbers; the feed's; the three ids among the
/// camera's colours. Pure, total, constexpr — a robot file may `static_assert` it.
[[nodiscard]] constexpr ToggleCameraCheck toggleCameraCheck(const config::CameraConfig& camera,
                                                            const ToggleCameraConfig& block) noexcept {
    if (!block.declared()) {
        return {"", "no Toggle block is declared (the robot file's sensors.toggleCamera)"};
    }
    if (const char* p = cameraModelProblem(camera)) {
        return {"", p};
    }
    if (const char* p = toggleTrackerConfigProblem(block.tracker)) {
        return {"the Toggle block's tracker: ", p};
    }
    if (const char* p = cameraFeedConfigProblem(block.feed)) {
        return {"the Toggle block's feed: ", p};
    }
    if (const char* p = toggleIdsProblem(camera.settings, block.tracker.ids)) {
        return {"the Toggle block: ", p};
    }
    return {};
}

/// toggleCameraCheck()'s sentence alone (nullptr when the block may be built) — for a `static_assert`.
[[nodiscard]] constexpr const char* toggleCameraProblem(const config::CameraConfig& camera,
                                                        const ToggleCameraConfig& block) noexcept {
    return toggleCameraCheck(camera, block).why;
}

/// The longest absence sentence the slot keeps, NUL included (a part, then a sentence of the library's).
inline constexpr std::size_t kToggleCameraReasonBytes = 224;

/// The SLOT (file banner): built EMPTY — not ready, "not built yet" — and filled once, by build().
/// Neither copyable nor movable: its feed holds references, and the context points at it.
class ToggleCamera {
public:
    /// Not ready: "not built yet".
    ToggleCamera() noexcept = default;
    /// Non-copyable (the context points at this object).
    ToggleCamera(const ToggleCamera&) = delete;
    /// Non-movable, same reason.
    ToggleCamera(ToggleCamera&&) = delete;
    /// Non-copy-assignable.
    ToggleCamera& operator=(const ToggleCamera&) = delete;
    /// Non-move-assignable.
    ToggleCamera& operator=(ToggleCamera&&) = delete;
    /// Nothing to release: the camera and the clock are borrowed.
    ~ToggleCamera() = default;

    /// Fill the slot ONCE: READY when toggleCameraCheck(cameraConfig, block) passes AND `camera` (the
    /// robot's BUILT camera, nullptr when it was not built) is non-null — the feed over `camera` and
    /// `clock` with the block's read period, the model of the config's mount, the block's tracker numbers.
    /// Otherwise NOT READY with the first reason (the checks' order, then "the camera was not built").
    /// Reads nothing from the camera. `camera` and `clock` must outlive this object. A second call is a
    /// programming error (a precondition: a slot is filled once, at boot).
    void build(hal::IVisionFrames* camera, const hal::IClock& clock, const config::CameraConfig& cameraConfig,
               const ToggleCameraConfig& block) {
        SHULIB_PRECONDITION(!filled_, "vision::ToggleCamera::build: the slot is filled once, at boot");
        filled_ = true;
        const ToggleCameraCheck c = toggleCameraCheck(cameraConfig, block);
        if (c.why != nullptr) {
            std::snprintf(absence_, sizeof absence_, "%s%s", c.part, c.why);
            return;
        }
        if (camera == nullptr) {
            std::snprintf(absence_, sizeof absence_, "%s",
                          "the camera was not built (the boot line's camera words say why)");
            return;
        }
        feed_.emplace(*camera, clock, block.feed);
        model_.emplace(kAiVisionOptics, cameraConfig.placement);
        tracker_ = block.tracker;
        absence_[0] = '\0';
    }

    /// True when build() built it: the feed, the model and the tracker numbers are there.
    [[nodiscard]] bool ready() const noexcept { return feed_.has_value() && model_.has_value(); }
    /// The feed, ticked only inside the step's own waits — nullptr when not ready.
    [[nodiscard]] CameraFeed* feed() noexcept { return ready() ? &*feed_ : nullptr; }
    /// The model of the measured mount — nullptr when not ready.
    [[nodiscard]] const CameraModel* model() const noexcept { return ready() ? &*model_ : nullptr; }
    /// The team's tracker numbers (all UNSET when not ready).
    [[nodiscard]] const ToggleTrackerConfig& tracker() const noexcept { return tracker_; }
    /// Why it is not ready — "" EXACTLY when ready(); "not built yet" before build(). Stable for the
    /// object's lifetime once built.
    [[nodiscard]] const char* absence() const noexcept { return absence_; }

private:
    std::optional<CameraFeed> feed_{};
    std::optional<CameraModel> model_{};
    ToggleTrackerConfig tracker_{};
    bool filled_ = false;
    char absence_[kToggleCameraReasonBytes] = "not built yet";
};

}  // namespace shulib::vision
