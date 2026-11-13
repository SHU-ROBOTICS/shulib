// diag::LogCopySink — a copy of a sink's leveled messages and run summaries, never its per-tick
// records (chunk T4 Part 3; the team lead's decisions N5 (a) and A (a)). The copy the provider
// keeps on the SD card is built from it (hal/pros/device_provider.hpp; its shim tests are in
// test/pros_device_provider_test.cpp, 1n-1q).
//
// Each case names the bug it catches; the mutation that turns it red is in T4-PROGRESS "Part 3".

#include "doctest.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <future>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/log_copy_sink.hpp"
#include "shulib/diag/run_summary.hpp"
#include "shulib/diag/term_sink.hpp"
#include "shulib/hal/fake/fake_char_sink.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/math/twist2d.hpp"

// The counters live in `apriltag_corrector_cost_test.cpp`, at namespace scope with external
// linkage, behind the global operator new/delete replacements defined there. Declared (not
// defined) here so this file instruments the same program-wide allocator.
namespace shulib_alloc_probe {
extern std::size_t allocations;
extern bool counting;
}  // namespace shulib_alloc_probe

using shulib::diag::DebugRecord;
using shulib::diag::LogCopySink;
using shulib::diag::RunSummary;
using shulib::diag::TermSink;
using shulib::hal::ITelemetrySink;
using shulib::hal::LogLevel;
using shulib::hal::fake::FakeCharSink;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeTelemetrySink;

namespace {

/// Two TermSinks over one clock — the shape the provider builds: serial (`primary`) and the
/// card's (`copy`), each over its own character device.
struct TwoTerms {
    FakeClock clock;
    FakeCharSink serial;
    FakeCharSink card;
    TermSink primary{clock, serial};
    TermSink copy{clock, card};
    LogCopySink sink{primary, copy};
};

/// The number of '\n'-terminated lines in `text`.
int lineCount(const std::string& text) {
    int n = 0;
    for (const char c : text) {
        if (c == '\n') ++n;
    }
    return n;
}

/// A drive()-shaped record: what Chassis::drive() hands the sink on every tick.
DebugRecord tickRecord(int k) {
    DebugRecord r;
    r.t = shulib::units::Time{0.01 * k};
    return r;
}

/// A telemetry sink that only COUNTS, into fixed fields — so a scope can assert that nothing
/// allocated while lines, records and summaries passed through the copy.
struct CountingSink final : ITelemetrySink {
    int logs = 0;
    int records = 0;
    int summaries = 0;
    bool wants = false;
    void log(LogLevel, std::string_view, std::string_view) override { ++logs; }
    [[nodiscard]] bool wantsRecord() const noexcept override { return wants; }
    void emit(const DebugRecord&) override { ++records; }
    void summarize(const RunSummary&) override { ++summaries; }
};

/// One journal both sinks write to, in call order: "P" for the primary, "C" for the copy.
struct Journal {
    std::array<char, 16> calls{};
    std::size_t n = 0;
    void add(char who) {
        if (n < calls.size()) calls[n++] = who;
    }
    [[nodiscard]] std::string str() const { return std::string{calls.data(), n}; }
};

struct JournalSink final : ITelemetrySink {
    JournalSink(Journal& j, char who) : journal{&j}, me{who} {}
    Journal* journal;
    char me;
    void log(LogLevel, std::string_view, std::string_view) override { journal->add(me); }
    void summarize(const RunSummary&) override { journal->add(me); }
};

}  // namespace

TEST_CASE("T4 P3 LC-1: N drive()-shaped records put N lines on the primary and ZERO on the copy; a "
          "log() line reaches both, whole, byte for byte") {
    // BUG CAUGHT (the brief's mutation "LogCopySink forwards emit() to the copy"; decision A): a
    // copy at the character level — every per-tick record on the SD card, one block write and one
    // flush every 10 ms (HA-60 says a card write is fine at a boundary, not in a tick).
    TwoTerms t;
    for (int k = 0; k < 100; ++k) t.sink.emit(tickRecord(k));
    CHECK(lineCount(t.serial.text()) == 100);
    CHECK(t.card.empty());
    t.serial.clear();
    t.clock.advance(shulib::units::Time{1.5});
    t.sink.log(LogLevel::Warn, "TEL", "WARNING: port 18 (LEFT) went ABSENT at runtime");
    CHECK(lineCount(t.serial.text()) == 1);
    CHECK(t.card.text() == t.serial.text());  // the same line, stamped from the same clock
    CHECK(t.card.text().find("[WARN][TEL] WARNING: port 18 (LEFT) went ABSENT at runtime")
          != std::string::npos);
}

TEST_CASE("T4 P3 LC-2: summarize()'s block reaches both, whole — the copy's lines are the primary's") {
    // BUG CAUGHT: the decorator's summarize() left at the base's no-op (the seam's decorator rule):
    // the end-of-run block silently missing from the card.
    TwoTerms t;
    RunSummary s{};
    s.motionsStarted = 3;
    s.motionsSettled = 2;
    s.motionsTimedOut = 1;
    t.sink.summarize(s);
    CHECK(lineCount(t.serial.text()) >= 6);
    CHECK(t.card.text() == t.serial.text());
}

TEST_CASE("T4 P3 LC-3: wantsRecord() is the PRIMARY's — records are built for the primary, never "
          "for the copy, and a primary that wants none is not overruled by a copy that does") {
    // BUG CAUGHT: wantsRecord() answered from the copy (a primary that wants no records then pays
    // to build one every tick for nobody), or always true / always false (the pair rule broken).
    CountingSink primaryWants;
    primaryWants.wants = true;
    CountingSink copyWantsNone;
    CHECK(LogCopySink{primaryWants, copyWantsNone}.wantsRecord());
    CountingSink primaryWantsNone;
    FakeTelemetrySink copyWants;  // a sink that wants every record
    REQUIRE(copyWants.wantsRecord());
    CHECK_FALSE(LogCopySink{primaryWantsNone, copyWants}.wantsRecord());
}

TEST_CASE("T4 P3 LC-4: the order — the primary first, then the copy, for a line and for a summary; "
          "a record to the primary alone") {
    // BUG CAUGHT: the copy written first (a copy that blocks on a slow card would hold the serial
    // line back), or a channel sent to one sink only.
    Journal j;
    JournalSink primary{j, 'P'};
    JournalSink copy{j, 'C'};
    LogCopySink sink{primary, copy};
    sink.log(LogLevel::Info, "DRV", "one");
    sink.summarize(RunSummary{});
    sink.emit(DebugRecord{});
    sink.log(LogLevel::Error, "TEL", "two");
    CHECK(j.str() == "PCPCPC");
    CHECK(&sink.primary() == &primary);
    CHECK(&sink.copy() == &copy);
}

TEST_CASE("T4 P3 LC-5: nothing allocates — a line, a record and a summary through the copy") {
    // BUG CAUGHT: a copy that buffers (a std::string per line, a queue) — the copy runs inside the
    // driver loop's tick, where nothing allocates.
    CountingSink primary;
    primary.wants = true;
    CountingSink copy;
    LogCopySink sink{primary, copy};
    const DebugRecord r{};
    const RunSummary s{};
    std::size_t allocations = 0;
    {
        shulib_alloc_probe::allocations = 0;
        shulib_alloc_probe::counting = true;
        for (int k = 0; k < 50; ++k) {
            sink.log(LogLevel::Info, "TEL", "a line");
            sink.emit(r);
        }
        sink.summarize(s);
        shulib_alloc_probe::counting = false;
        allocations = shulib_alloc_probe::allocations;
    }
    CHECK(allocations == 0U);
    CHECK(primary.logs == 50);
    CHECK(primary.records == 50);
    CHECK(primary.summaries == 1);
    CHECK(copy.logs == 50);
    CHECK(copy.records == 0);
    CHECK(copy.summaries == 1);
}

TEST_CASE("T4 P3 LC-6: through the REAL drive path — 200 Chassis::drive() ticks put 200 record lines "
          "on the primary and NONE on the copy; the drive's own log line reaches both") {
    // BUG CAUGHT (decision A, the re-check of 2026-09-28's finding 2): the copy as a character-level
    // tee under the provider's TermSink — Chassis::drive() emits a record on EVERY call, so the
    // card would get a line and a flush every 10 ms of driver control.
    FakeClock clock;
    FakeCharSink serial;
    FakeCharSink card;
    TermSink primary{clock, serial};
    TermSink copy{clock, card};
    LogCopySink sink{primary, copy};
    const shulib::kinematics::TankKinematics kin{shulib::units::Length{12.0}};
    motion_rig::ChassisRig c{kin, motion_rig::plantConfig(), &sink};
    REQUIRE(c.rig.h.context().telemetry().wantsRecord());  // the records ARE built, for serial
    const std::string serialBefore = serial.text();
    const std::string cardBefore = card.text();
    const shulib::math::ChassisSpeeds cmd{shulib::units::Velocity{10.0}, shulib::units::Velocity{0.0},
                                          shulib::units::AngularVelocity{0.5}};
    for (int k = 0; k < 200; ++k) {
        c.chassis.drive(cmd, shulib::math::Frame::Body);
        c.pacer.pace();
    }
    const std::string serialRun = serial.text().substr(serialBefore.size());
    const std::string cardRun = card.text().substr(cardBefore.size());
    CHECK(lineCount(serialRun) >= 200);  // a record line per tick, at least
    CHECK(cardRun.empty());              // and not one of them on the card
    // A log line said through the same context still reaches both.
    c.rig.h.context().telemetry().log(LogLevel::Info, "DRV", "after the drive");
    const std::string serialTail = serial.text().substr(serialBefore.size() + serialRun.size());
    CHECK(card.text().substr(cardBefore.size()) == serialTail);
    CHECK(lineCount(serialTail) == 1);
}

// ═══ ONE WRITER ON THE COPY AT A TIME (the T4-3 review, 2026-10-08) ═══════════════════════════
namespace {

/// A copy that, while it writes, has ANOTHER writer arrive at the same LogCopySink — what a second
/// task logging mid-copy looks like, made deterministic: the re-entry happens INSIDE the first
/// line's copy, exactly where a preempting task would find the guard taken.
class ReenteringCopy final : public ITelemetrySink {
public:
    void log(LogLevel level, std::string_view subsystem, std::string_view message) override {
        lines.emplace_back(message);
        tags.emplace_back(subsystem);
        levels.push_back(level);
        events.emplace_back(message);
        if (reenter != nullptr && reentries > 0) {
            --reentries;
            reenter->log(level, subsystem, "the other task's line");
            RunSummary s{};
            reenter->summarize(s);
        }
    }
    void summarize(const RunSummary& /*summary*/) override {
        ++summaries;
        events.emplace_back(kSummaryMark);
    }
    /// What `events` holds for a summary: lines and summaries in ONE list, in the order the copy
    /// got them (the verification's finding: two lists could not show a gap line written AFTER the
    /// summary it belongs before).
    static constexpr const char* kSummaryMark = "<summary>";
    LogCopySink* reenter = nullptr;
    int reentries = 0;
    std::vector<std::string> lines;
    std::vector<std::string> tags;
    std::vector<LogLevel> levels;
    std::vector<std::string> events;  ///< every line's message and every summary's mark, in order
    int summaries = 0;
};

}  // namespace

TEST_CASE("T4 P3 LC-7: a line (or summary) arriving while another writer is mid-copy reaches the "
          "PRIMARY, is left OFF the copy and is COUNTED; the guard is released after every copy") {
    // BUG CAUGHT (the T4-3 review): two tasks writing the card copy's one FILE* at once (no guard:
    // the re-entered line lands on the copy in the middle of another); a guard that never releases
    // (every later line silently lost from the card); a kept-off line not counted (the boot line's
    // "lost" under-says); the primary losing a line because the copy was busy.
    FakeTelemetrySink primary;
    ReenteringCopy copy;
    LogCopySink sink{primary, copy};
    copy.reenter = &sink;
    copy.reentries = 1;
    sink.log(LogLevel::Info, "CMP", "the first task's line");
    // The primary got both lines (the other task's arrived mid-copy) — the copy only the first.
    REQUIRE(primary.size() == 2);
    CHECK(primary.at(0).message == "the first task's line");
    CHECK(primary.at(1).message == "the other task's line");
    REQUIRE(copy.lines.size() == 1U);
    CHECK(copy.lines[0] == "the first task's line");
    CHECK(copy.summaries == 0);              // the re-entered summary was kept off too
    CHECK(sink.contendedLines() == 2U);      // the line and the summary
    // The guard was released: the next line and summary, from one writer, reach the copy — the
    // line preceded by ONE gap line saying, on the copy, how many it missed there (BUG CAUGHT: a
    // gap that opens mid-run counted where nobody reads it after boot; a gap line on every later
    // line; a gap line on the primary, which missed nothing).
    sink.log(LogLevel::Warn, "CMP", "a later line");
    sink.summarize(RunSummary{});
    REQUIRE(copy.lines.size() == 3U);
    CHECK(copy.lines[1]
          == "copy: 2 line(s) left off here while another task was writing it -- serial has them");
    CHECK(copy.tags[1] == LogCopySink::kGapTag);
    CHECK(copy.levels[1] == LogLevel::Warn);
    CHECK(copy.lines[2] == "a later line");
    CHECK(copy.summaries == 1);
    CHECK(sink.contendedLines() == 2U);
    CHECK(primary.size() == 3);  // no gap line on the primary
    for (int i = 0; i < primary.size(); ++i) {
        CHECK(primary.at(i).message.find("left off") == std::string::npos);
    }
    // Said once: the next line has no gap line before it.
    sink.log(LogLevel::Info, "CMP", "and another");
    REQUIRE(copy.lines.size() == 4U);
    CHECK(copy.lines[3] == "and another");
}

TEST_CASE("T4 P3 LC-8: a gap is said before a SUMMARY too, and a second gap says only its own count") {
    // BUG CAUGHT: a gap said only before a log() line (a summary next on the card hides it), or a
    // second gap that re-counts the first (the reported mark never advanced).
    FakeTelemetrySink primary;
    ReenteringCopy copy;
    LogCopySink sink{primary, copy};
    copy.reenter = &sink;
    copy.reentries = 1;
    sink.log(LogLevel::Info, "CMP", "one");  // 2 kept off inside it
    sink.summarize(RunSummary{});             // the gap line, then the summary
    REQUIRE(copy.lines.size() == 2U);
    CHECK(copy.lines[1]
          == "copy: 2 line(s) left off here while another task was writing it -- serial has them");
    CHECK(copy.summaries == 1);
    // In ONE ordered list: the gap line IMMEDIATELY before the summary it precedes on the card
    // (BUG CAUGHT, the T4-3 verification's V5: the gap line written after the summary — "here"
    // then points past the block, not at the place the lines were missed).
    CHECK(copy.events
          == std::vector<std::string>{
              "one",
              "copy: 2 line(s) left off here while another task was writing it -- serial has them",
              ReenteringCopy::kSummaryMark});
    copy.reentries = 1;
    sink.log(LogLevel::Info, "CMP", "two");  // 2 more kept off
    sink.log(LogLevel::Info, "CMP", "three");
    REQUIRE(copy.lines.size() == 5U);
    CHECK(copy.lines[2] == "two");
    CHECK(copy.lines[3]
          == "copy: 2 line(s) left off here while another task was writing it -- serial has them");
    CHECK(copy.lines[4] == "three");
    CHECK(sink.contendedLines() == 4U);
}

// ═══ A WRITER THAT NEVER COMES BACK (the T4-3 verification, 2026-10-08) ═══════════════════════
namespace {

/// A copy whose FIRST line does not return until the test lets it go: the shape of a task PROS
/// DELETED mid-line (the A4 register's HA-161 (a), reasoned) — it runs nothing more, so it never
/// leaves the copy and never releases the guard. Every later line and summary is recorded, under a
/// lock: the writer that never returns and the next period's writer are real threads here.
class StuckCopy final : public ITelemetrySink {
public:
    void log(LogLevel /*level*/, std::string_view /*subsystem*/,
             std::string_view message) override {
        if (!stuck_.exchange(true)) {
            held_.set_value();
            gone_.wait();  // "deleted": it returns only when the test cleans up
            return;
        }
        const std::lock_guard<std::mutex> lock{m_};
        events_.emplace_back(message);
    }
    void summarize(const RunSummary& /*summary*/) override {
        const std::lock_guard<std::mutex> lock{m_};
        events_.emplace_back("<summary>");
    }
    /// Returns once the first writer is inside its copy, holding the guard.
    void waitUntilHeld() { heldSeen_.wait(); }
    /// Clean-up only: a deleted task never comes back.
    void letGo() { release_.set_value(); }
    [[nodiscard]] std::vector<std::string> events() {
        const std::lock_guard<std::mutex> lock{m_};
        return events_;
    }

private:
    std::atomic<bool> stuck_{false};
    std::promise<void> held_;
    std::future<void> heldSeen_ = held_.get_future();
    std::promise<void> release_;
    std::shared_future<void> gone_ = release_.get_future().share();
    std::mutex m_;
    std::vector<std::string> events_;
};

}  // namespace

TEST_CASE("T4 P3 LC-9: a writer that NEVER COMES BACK from its copy (a task PROS deleted mid-line) "
          "ends the copy for the boot — every later line reaches the primary and is counted, none "
          "(nor a gap line) reaches the copy, and no writer waits") {
    // BUG CAUGHT (the T4-3 verification's held-guard probe, made a test; the chosen behaviour is
    // log_copy_sink.hpp's "A writer that never comes back"): a guard taken back from a holder that
    // still holds it, after some count of lines (two writers on the card's FILE at once — behind a
    // deleted writer, the FILE half-updated and maybe a lock below it held); a gap line written on
    // the copy while another writer holds it; a BLOCKING lock (the next driver period's loop would
    // wait forever on a task that no longer exists); a lost line left uncounted; the primary
    // missing a line.
    FakeTelemetrySink primary;
    StuckCopy copy;
    LogCopySink sink{primary, copy};
    // The opcontrol task's line after its loop saw the disable; PROS deletes the task mid-copy.
    std::thread deleted{[&sink] {
        sink.log(LogLevel::Info, "DRV", "opcontrol(): field DISABLED -- the loop exited");
    }};
    copy.waitUntilHeld();
    // The next driver period's task (PROS calls opcontrol() again): its lines and a summary — far
    // more than any count a "the holder must be dead by now" rule might wait for.
    constexpr int kLater = 500;
    std::future<void> later = std::async(std::launch::async, [&sink] {
        for (int i = 0; i < kLater; ++i) {
            sink.log(LogLevel::Warn, "TEL", "a CUT, a dead port, a peak");
        }
        sink.summarize(RunSummary{});
    });
    const bool neverWaited =
        later.wait_for(std::chrono::seconds{10}) == std::future_status::ready;
    CHECK(neverWaited);
    if (neverWaited) {
        CHECK(copy.events().empty());  // the copy ended at the deleted writer's line
        CHECK(sink.contendedLines() == static_cast<std::uint32_t>(kLater + 1));  // + the summary
        CHECK(primary.size() == 1 + kLater);  // serial kept every line ...
        CHECK(primary.summaryCount() == 1);   // ... and the summary
    }
    // Clean-up — and the contrast: a writer that DOES come back (one merely slow) releases the
    // guard, and the next line says the gap first, exactly as LC-7 pins.
    copy.letGo();
    deleted.join();
    later.wait();
    sink.log(LogLevel::Info, "DRV", "after");
    CHECK(copy.events()
          == std::vector<std::string>{"copy: 501 line(s) left off here while another task was "
                                      "writing it -- serial has them",
                                      "after"});
}
