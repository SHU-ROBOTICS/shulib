#pragma once
//
// FakeLineDisplay — records every row write through the ILineDisplay seam so host
// tests can assert the CONTROLLER SCREEN's exact content AND its write discipline
// (D-4's "rewrite only what changed" — the write counter is what makes a per-tick
// repaint a testable regression instead of a field-day surprise). The row-device
// equivalent of FakeCharSink. Host-test only (uses std::string).
//
// Truncation at kCols happens HERE, as the seam contract says implementations do
// — so a test reading rows sees exactly what a real 19-column LCD would hold.

#include <string>
#include <string_view>

#include "shulib/core/check.hpp"
#include "shulib/hal/line_display.hpp"

namespace shulib::hal::fake {

class FakeLineDisplay final : public ILineDisplay {
public:
    void setLine(int row, std::string_view text) override {
        SHULIB_PRECONDITION(row >= 0 && row < kRows, "FakeLineDisplay::setLine: row out of range");
        const std::size_t take =
            text.size() < static_cast<std::size_t>(kCols) ? text.size()
                                                          : static_cast<std::size_t>(kCols);
        rows_[row] = std::string{text.substr(0, take)};
        ++writes_[row];
        ++totalWrites_;
    }

    /// Current content of `row` (as the device holds it: truncated at kCols).
    [[nodiscard]] const std::string& row(int i) const {
        SHULIB_PRECONDITION(i >= 0 && i < kRows, "FakeLineDisplay::row: row out of range");
        return rows_[i];
    }

    /// How many times `row` has been written — the repaint-discipline probe.
    [[nodiscard]] int writeCount(int i) const {
        SHULIB_PRECONDITION(i >= 0 && i < kRows, "FakeLineDisplay::writeCount: row out of range");
        return writes_[i];
    }

    [[nodiscard]] int totalWrites() const noexcept { return totalWrites_; }

    void clear() noexcept {
        for (int i = 0; i < kRows; ++i) {
            rows_[i].clear();
            writes_[i] = 0;
        }
        totalWrites_ = 0;
    }

private:
    std::string rows_[static_cast<std::size_t>(kRows)];
    int writes_[static_cast<std::size_t>(kRows)] = {0, 0, 0};
    int totalWrites_ = 0;
};

}  // namespace shulib::hal::fake
