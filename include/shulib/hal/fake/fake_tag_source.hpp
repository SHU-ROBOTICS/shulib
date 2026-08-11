#pragma once
//
// FakeTagSource — a deterministic ITagSource for host tests. Inject a list of visible
// AprilTags (id + robot-relative pose + confidence) so the localization corrector can
// be driven with exact geometry, including the no-tags (empty) case.

#include <vector>

#include "shulib/hal/vision.hpp"

namespace shulib::hal::fake {

class FakeTagSource final : public ITagSource {
public:
    [[nodiscard]] std::vector<TagObservation> tags() const override { return tags_; }

    void setTags(std::vector<TagObservation> tags) { tags_ = std::move(tags); }
    void clear() { tags_.clear(); }

private:
    std::vector<TagObservation> tags_;  // default: no tags visible
};

}  // namespace shulib::hal::fake
