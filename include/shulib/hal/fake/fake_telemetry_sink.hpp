#pragma once
//
// FakeTelemetrySink — records emitted lines so host tests can assert WHAT was logged
// (a fault was reported, a retry warned, a gating decision logged). Unlike NullSink it
// keeps the history. Host-test only (uses std::vector / std::string).

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include "shulib/core/check.hpp"
#include "shulib/hal/telemetry_sink.hpp"

namespace shulib::hal::fake {

class FakeTelemetrySink final : public ITelemetrySink {
public:
    struct Entry {
        LogLevel level;
        std::string subsystem;
        std::string message;
    };

    void log(LogLevel level, std::string_view subsystem, std::string_view message) override {
        entries_.push_back(Entry{level, std::string{subsystem}, std::string{message}});
    }

    [[nodiscard]] int size() const noexcept { return static_cast<int>(entries_.size()); }
    [[nodiscard]] bool empty() const noexcept { return entries_.empty(); }

    [[nodiscard]] const Entry& at(int i) const {
        SHULIB_PRECONDITION(i >= 0 && i < size(), "FakeTelemetrySink::at: index out of range");
        return entries_[static_cast<std::size_t>(i)];
    }

    [[nodiscard]] const Entry& last() const {
        SHULIB_PRECONDITION(!entries_.empty(), "FakeTelemetrySink::last: no entries");
        return entries_.back();
    }

    void clear() noexcept { entries_.clear(); }

private:
    std::vector<Entry> entries_;
};

}  // namespace shulib::hal::fake
