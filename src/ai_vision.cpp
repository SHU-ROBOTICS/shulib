// src/ai_vision.cpp
//
// Implementation of the AI Vision Sensor wrapper.
// See include/ai_vision.hpp for the public interface and design rationale.

#include "ai_vision.hpp"

#include <algorithm>

namespace ai_vision {

// ---------------------------------------------------------------------------
// Detection helpers
// ---------------------------------------------------------------------------

double Detection::xOffsetNormalized() const {
    // Centered (xCenter == IMAGE_CENTER_X) -> 0.0
    // Left edge (xCenter == 0)             -> -1.0
    // Right edge (xCenter == IMAGE_WIDTH)  -> +1.0
    //
    // Note we divide by IMAGE_CENTER_X (half width) so the output is a
    // fraction of half-frame, which is what callers want for a P-controller
    // error signal: "how far from center as a fraction of the available range".
    return static_cast<double>(xCenter - IMAGE_CENTER_X) / IMAGE_CENTER_X;
}

int Detection::area() const {
    return width * height;
}

// ---------------------------------------------------------------------------
// Internal helper: convert raw API object to our cleaner struct
// ---------------------------------------------------------------------------

// Translate a pros::AIVision::Object that has already been verified to be
// an AI-classified object (not an AprilTag, not a color blob) into our
// Detection struct. The raw API exposes a tagged union; we read from the
// `element` arm here and the caller is responsible for type-checking.
static Detection fromRawObject(const pros::AIVision::Object& raw) {
    Detection d;
    d.classId = raw.id;
    d.width   = raw.object.element.width;
    d.height  = raw.object.element.height;
    d.score   = raw.object.element.score;

    // Raw API reports the top-left corner of the bounding box.
    // Convert to center coordinates because every downstream calculation
    // (centering, distance estimation) is more natural with center coords.
    d.xCenter = raw.object.element.xoffset + d.width  / 2;
    d.yCenter = raw.object.element.yoffset + d.height / 2;

    return d;
}

// ---------------------------------------------------------------------------
// Camera class
// ---------------------------------------------------------------------------

Camera::Camera(int port) : sensor(port) {}

bool Camera::initialize() {
    // reset() clears any prior configuration on the sensor and re-initializes
    // it. After reset, no detection types are enabled by default — we have
    // to explicitly turn on the modes we care about.
    sensor.reset();

    // For the skills routine we only need AI-classified objects. We are
    // intentionally NOT enabling AprilTag or color blob detection here;
    // adding those would clutter the get_all_objects() return with
    // detections we'd just have to filter out anyway.
    sensor.enable_detection_types(pros::AivisionModeType::objects);

    return true;
}

std::vector<Detection> Camera::getAll() {
    std::vector<Detection> result;
    auto raw_objects = sensor.get_all_objects();
    result.reserve(raw_objects.size());

    for (const auto& raw : raw_objects) {
        // The same API surface returns AprilTags and color blobs when those
        // modes are enabled. We only enabled objects in initialize(), but
        // type-checking here keeps the wrapper robust if someone later
        // enables additional modes elsewhere.
        if (pros::AIVision::is_type(raw, pros::AivisionDetectType::object)) {
            result.push_back(fromRawObject(raw));
        }
    }

    // Sort largest first. Most callers want "the most prominent thing in
    // view", and bounding box area is the cheapest proxy for that.
    std::sort(result.begin(), result.end(),
              [](const Detection& a, const Detection& b) {
                  return a.area() > b.area();
              });

    return result;
}

std::vector<Detection> Camera::getAllOfClass(int classId) {
    auto all = getAll();
    std::vector<Detection> filtered;
    filtered.reserve(all.size());

    for (const auto& d : all) {
        if (d.classId == classId) {
            filtered.push_back(d);
        }
    }

    // Already sorted by area descending because getAll() sorts before
    // returning; preserving that order here.
    return filtered;
}

std::optional<Detection> Camera::getLargestOfClass(int classId) {
    auto matches = getAllOfClass(classId);
    if (matches.empty()) {
        return std::nullopt;
    }
    return matches.front();
}

}  // namespace ai_vision
