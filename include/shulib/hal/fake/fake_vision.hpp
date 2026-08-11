#pragma once
//
// FakeVision — a deterministic IVision for host tests. Inject a list of visible objects
// (class id + robot-relative bearing + confidence) so manipulation targeting can be
// driven with exact geometry, including the no-objects (empty) case.

#include <vector>

#include "shulib/hal/vision.hpp"

namespace shulib::hal::fake {

class FakeVision final : public IVision {
public:
    [[nodiscard]] std::vector<ObjectObservation> objects() const override { return objects_; }

    void setObjects(std::vector<ObjectObservation> objects) { objects_ = std::move(objects); }
    void clear() { objects_.clear(); }

private:
    std::vector<ObjectObservation> objects_;  // default: nothing detected
};

}  // namespace shulib::hal::fake
