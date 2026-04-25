// include/ai_vision.hpp
//
// Wrapper around pros::AIVision for the V5 AI Vision Sensor (276-8659).
// Provides typed, convenient access to AI-classified detections with
// derived fields (center coordinates, normalized offsets) that the
// underlying API returns as raw top-left bounding box coordinates.
//
// Image dimensions are fixed by hardware: 320 x 240 pixels.
// Center of frame is (160, 120).
//
// This file does not depend on the chassis or IMU. It is safe to use
// on its own to verify the sensor is detecting your target objects
// before integrating with the rest of the autonomous routine.

#pragma once

#include "pros/ai_vision.hpp"

#include <optional>
#include <vector>

namespace ai_vision {

// Sensor image dimensions, fixed by hardware.
constexpr int IMAGE_WIDTH    = 320;
constexpr int IMAGE_HEIGHT   = 240;
constexpr int IMAGE_CENTER_X = IMAGE_WIDTH / 2;
constexpr int IMAGE_CENTER_Y = IMAGE_HEIGHT / 2;

// A single AI-classified object detection.
// Coordinates are in pixels, with origin (0, 0) at the top-left of the image.
struct Detection {
    int classId;   // AI Classification class ID, as configured in the AI Vision Utility
    int xCenter;   // Center X of bounding box in pixels (0 .. IMAGE_WIDTH)
    int yCenter;   // Center Y of bounding box in pixels (0 .. IMAGE_HEIGHT)
    int width;     // Bounding box width in pixels
    int height;    // Bounding box height in pixels
    int score;     // Detection confidence, 0 .. 100 (higher is more confident)

    // X offset from frame center, normalized to [-1.0, +1.0].
    // Negative means the object is left of center, positive means right.
    // Useful as the error signal for a "center on object" P-controller.
    double xOffsetNormalized() const;

    // Bounding box area in pixels. Larger area is a rough proxy
    // for "closer to the camera" — useful as a distance heuristic.
    int area() const;
};

// Wrapper around the V5 AI Vision Sensor. Construct once with the port
// the sensor is plugged into, call initialize() during robot startup,
// then query detection methods from anywhere.
//
// Thread safety: the underlying pros::AIVision is generally safe to
// query from multiple tasks, but if you call query methods from
// autonomous and a separate logging task at the same time you may
// see slightly different snapshots between calls. For the skills
// routine, queries happen sequentially from one task, so this is fine.
class Camera {
public:
    // Bind the camera to a Smart Port (1 .. 21). Construction does not
    // touch the hardware; call initialize() once the brain is up.
    explicit Camera(int port);

    // Reset the sensor and enable AI Object detection mode.
    // Call this exactly once from your initialize() function.
    // Returns true on apparent success. The PROS API does not surface
    // a meaningful failure code here, so a "true" mostly means the
    // calls were made — actual success is verified by checking that
    // detections come through during testing.
    bool initialize();

    // Return the largest (by bounding box area) currently-visible
    // detection matching the given class ID, or std::nullopt if none.
    // "Largest" is a reasonable proxy for "closest / most relevant".
    std::optional<Detection> getLargestOfClass(int classId);

    // Return all currently-visible detections matching the given
    // class, sorted by bounding box area descending (largest first).
    std::vector<Detection> getAllOfClass(int classId);

    // Return all currently-visible AI-classified detections,
    // regardless of class, sorted by area descending.
    std::vector<Detection> getAll();

private:
    pros::AIVision sensor;
};

}  // namespace ai_vision
