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
//
// THE LINE HOOK (chunk RPV1-A, 2026-11-12): an OPT-IN observer called after each stored line
// (setOnLine), with the count of lines kept so far. Nothing sets it by default, so every existing
// test behaves exactly as before. The route preview (test/route_preview_run.hpp) sets it to stop an
// auton that keeps logging without letting time pass: a routine that loops after a stop logs one
// "skipped" line per step and never paces, and this sink keeps every line, so without the hook such a
// run grows until it exhausts memory. The same hook also notes, as each line is stored, the run
// guard's end-of-run line and every mechanism verdict logged while an exception unwinds (which steps
// are the park's, and the step a run threw inside). The hook may end the process (std::_Exit) or
// throw; it runs inside log(), which the library calls from code that may be noexcept — a hook that
// throws must only be set where every caller can carry the exception. No file under src/ includes this header,
// so no robot program sees the hook.

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
        if (onLine_ != nullptr) {
            onLine_(onLineCtx_, entries_.size());
        }
    }

    /// The line hook (header: THE LINE HOOK): `fn(ctx, lines)` is called after each line is stored,
    /// `lines` being how many this sink now keeps. nullptr removes it (the default: no hook).
    void setOnLine(void (*fn)(void* ctx, std::size_t lines), void* ctx) noexcept {
        onLine_ = fn;
        onLineCtx_ = ctx;
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
    void (*onLine_)(void*, std::size_t) = nullptr;
    void* onLineCtx_ = nullptr;
};

}  // namespace shulib::hal::fake
