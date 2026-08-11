#pragma once
//
// FakeCorrector — a deterministic ICorrector for host tests. A test injects the proposal it should
// return next (including a deliberately bad one — wrong heading, NaN pose, zero std-dev — to prove
// the fusion/Localizer screening), and the fake records what it was asked, so tests can assert the
// Localizer hands it the PREDICTED pose and the tick dt. No HAL, no time — purely injectable.

#include "shulib/localization/correction.hpp"
#include "shulib/localization/i_corrector.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::localization::fake {

class FakeCorrector final : public ICorrector {
public:
    explicit FakeCorrector(const char* name = "fake") : name_{name} {}

    /// What propose() returns from now on.
    void setProposal(const CorrectionProposal& p) { next_ = p; }

    [[nodiscard]] CorrectionProposal propose(const math::Pose2d& predicted,
                                             units::Time dt) override {
        ++calls_;
        lastPredicted_ = predicted;
        lastDt_ = dt;
        return next_;
    }
    [[nodiscard]] const char* name() const noexcept override { return name_; }

    // --- test inspection ---
    [[nodiscard]] int calls() const noexcept { return calls_; }
    [[nodiscard]] math::Pose2d lastPredicted() const noexcept { return lastPredicted_; }
    [[nodiscard]] units::Time lastDt() const noexcept { return lastDt_; }

private:
    const char* name_;
    CorrectionProposal next_{};
    int calls_ = 0;
    math::Pose2d lastPredicted_{};
    units::Time lastDt_{};
};

}  // namespace shulib::localization::fake
