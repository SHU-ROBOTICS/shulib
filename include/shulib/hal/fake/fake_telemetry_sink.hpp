#pragma once
//
// FakeTelemetrySink — records emitted lines so host tests can assert WHAT was logged
// (a fault was reported, a retry warned, a gating decision logged). Unlike NullSink it
// keeps the history. Host-test only (uses std::vector / std::string).
//
// Since chunk A1 it also records the per-tick DebugRecord channel (wantsRecord() true +
// emit() overridden AS A PAIR, per the seam contract), so tests can assert what a tick
// loop emitted — the same role for §18.2 records that entries play for §18.3 messages.
// Access to both histories is bounds-checked for the same reason: a test asserting
// "record N said X" must not be able to read past the end and pass by accident.

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include "shulib/core/check.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/run_summary.hpp"
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

    [[nodiscard]] bool wantsRecord() const noexcept override { return true; }

    void emit(const diag::DebugRecord& record) override { records_.push_back(record); }

    /// Records the C5 summary channel too (RunSummary is a value type with bounded
    /// in-struct strings, so retaining copies is safe by construction).
    void summarize(const diag::RunSummary& summary) override { summaries_.push_back(summary); }

    // --- leveled-message history ---
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

    // --- per-tick record history ---
    [[nodiscard]] int recordCount() const noexcept { return static_cast<int>(records_.size()); }

    [[nodiscard]] const diag::DebugRecord& recordAt(int i) const {
        SHULIB_PRECONDITION(i >= 0 && i < recordCount(),
                            "FakeTelemetrySink::recordAt: index out of range");
        return records_[static_cast<std::size_t>(i)];
    }

    [[nodiscard]] const diag::DebugRecord& lastRecord() const {
        SHULIB_PRECONDITION(!records_.empty(), "FakeTelemetrySink::lastRecord: no records");
        return records_.back();
    }

    // --- run-summary history (C5) ---
    [[nodiscard]] int summaryCount() const noexcept { return static_cast<int>(summaries_.size()); }

    [[nodiscard]] const diag::RunSummary& summaryAt(int i) const {
        SHULIB_PRECONDITION(i >= 0 && i < summaryCount(),
                            "FakeTelemetrySink::summaryAt: index out of range");
        return summaries_[static_cast<std::size_t>(i)];
    }

    [[nodiscard]] const diag::RunSummary& lastSummary() const {
        SHULIB_PRECONDITION(!summaries_.empty(), "FakeTelemetrySink::lastSummary: no summaries");
        return summaries_.back();
    }

    /// Clears ALL histories.
    void clear() noexcept {
        entries_.clear();
        records_.clear();
        summaries_.clear();
    }

private:
    std::vector<Entry> entries_;
    std::vector<diag::DebugRecord> records_;
    std::vector<diag::RunSummary> summaries_;
};

}  // namespace shulib::hal::fake
