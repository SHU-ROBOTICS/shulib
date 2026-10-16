#pragma once
//
// LogCopySink — a COPY of a sink's leveled messages and run summaries in a second sink, and
// nothing else (chunk T4 Part 3, the team lead's decisions N5 (a) and A (a), 2026-09-27/28).
//
// ── Why this exists ─────────────────────────────────────────────────────────────────
// The old "shulib Drive" wrote every line it printed to a text file on the SD card as well, so a
// practice run with no laptop still left the dead ports, every cut and the peak currents behind.
// The library's programs logged to USB serial only. The team lead's N5 (a): every library program
// gets an OPTIONAL copy of its log on the card. His A (a): the copy holds EVENTS AND SUMMARIES
// ONLY — log() lines and summarize()'s block — and never a per-tick record. The reason is the
// cost: the provider's sink is a TermSink, and Chassis::drive() hands it a DebugRecord on EVERY
// tick, which TermSink prints as a line — 100 lines a second in driver control. A copy taken at
// the CHARACTER level (a tee under the TermSink) would put every one of them on the card, one
// block write and one flush each (hal::BlockCharSink), 100 flushes a second; the register's
// HA-60 says a card write is fine at a boundary, not in a tick. So the copy is taken here, at
// the TELEMETRY level, where the channels are still apart.
//
// ── What goes where ─────────────────────────────────────────────────────────────────
//   log()          the PRIMARY, then the COPY — the same call, whole, in that order;
//   summarize()    the PRIMARY, then the COPY (the seam's decorator rule: a decorator that
//                  forgot it would silently eat the summary);
//   wantsRecord()  the PRIMARY's answer, and
//   emit()         the PRIMARY only — the copy never sees a per-tick record, so it is never
//                  asked to format one. They are overridden as a PAIR, the seam's rule.
// The copy is typically a second TermSink, over the same clock, writing through a
// hal::BlockCharSink to an hal::pros::ProsBlockSink on the card — so its lines are the serial
// log's own lines, byte for byte, with the records left out. A copy that refuses (no card, a full
// card) is the copy's business: BlockCharSink drops and counts, and the primary is untouched.
//
// ── One writer on the copy at a time (the T4-3 review, 2026-10-08) ──────────────────────
// A program logs through this sink from more than one task: the competition program's auton
// task, PROS's phase tasks and the opcontrol task's driver loop each say lines, so around a mode
// change two can arrive at once. The copy writes ONE FILE* on the card, and PROS's C library
// does NOT serialize two tasks' writes to one FILE*: newlib's per-FILE locks are no-op stubs in
// the vendored toolchain (A4 register HA-189, reasoned from the binaries). So the COPY half takes
// a try-lock: the line that finds another task mid-copy is NOT copied
// — never waited for, so a card write in one task can never stall another's loop — and is counted
// (contendedLines()); the provider adds the count to the lines its copy lost
// (hal/pros/device_provider.hpp, serialCopyDroppedLines()). A count said only at boot would hide
// a gap that opens mid-run, so the COPY says it too, where it happened: the next line that does
// reach the copy is preceded by ONE Warn line, `[DIA] copy: <n> line(s) left off here while another
// task was writing it -- serial has them` (kGapTag, noteGap()) — on the copy only; the primary
// already holds every line. The flag is a std::atomic_flag: a lock-free C++ guarantee, nothing of
// PROS's (on the brain's one ARMv7 core, an exclusive load/store pair; never run there). The
// PRIMARY half takes no guard — it is the primary's business, and the serial's needs none since
// chunk SER1 (2026-10-08): hal::pros::ProsCharSink sends each line as ONE direct write(), never
// through stdout's FILE, and PROS's own serial write mutex keeps one call whole (HA-189 — reasoned
// from PROS's library, not yet seen). Its one exposure is PROS's, not ours: a task deleted INSIDE
// that write() is believed to leave PROS's mutex held (HA-190, reasoned) — and then every later
// write() of the primary blocks, so no writer ever reaches the copy below it: the copy ENDS at the
// same line as serial (this guard's give-up covers contention on the copy, not a blocked primary).
//
// ── A writer that never comes back (the T4-3 verification, 2026-10-08) ─────────────────
// The guard is released when the writer holding it RETURNS from its copy — normally, or past a
// copy that threw. The A4 register's HA-161 (a), REASONED and never observed, says PROS DELETES the
// task running a phase function at a mode change, mid-statement, and a deleted task returns from
// nothing. Both driver programs log a line right after their loop sees the field's disable — the
// moment PROS is about to delete that task. A deletion that lands while that task is inside the
// copy half leaves the guard held FOR THE REST OF THE BOOT: every later line and summary, from any
// task, still reaches the primary (serial keeps them all), is left off the copy and counted
// (contendedLines()) — and the gap line, said only under the guard, never appears. The card's copy
// simply ends there. How often a deletion lands inside a copy is UNMEASURED. This is the chosen
// failure, for two reasons. (1) To release the guard, something must know its holder is dead. The
// check the verification proposed, PROS's task_get_state() on the holder's handle, rests on a
// hardware belief no register entry holds: that it reads DELETED for a task PROS deleted, and never
// the state of the next task PROS builds in the same memory (left for the coordinator). (2) The
// deleted writer may have stopped inside newlib's fwrite or the card's driver, leaving the FILE
// half-updated and any lock below it held (unmeasured). A released guard would hand that state to
// the next writer — the driver loop included — where it could BLOCK. A held guard turns it into
// counted lost card lines, and no task waits. The way past it is a different design: one
// card-writer task that alone touches the FILE, fed by the others (the coordinator's call). Pinned
// by LC-9 (test/log_copy_sink_test.cpp).
//
// PROS-free, header-only, allocation-free; it holds two borrowed references, the flag and two
// counts. Apart from the copy's guard, single-task like the rest of diag/. Host-tested in
// test/log_copy_sink_test.cpp. NOTHING HERE HAS RUN ON A ROBOT.

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <string_view>

#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/run_summary.hpp"
#include "shulib/hal/telemetry_sink.hpp"

namespace shulib::diag {

/// A decorator that hands every leveled message and every run summary to TWO sinks — the
/// primary, then the copy — and every per-tick record to the primary ONLY (file banner: the
/// team lead's decision A (a), "events and summaries only"). Both sinks are BORROWED and must
/// outlive this object; nothing here allocates or throws. The copy has ONE writer at a time: a
/// line or summary that finds another task mid-copy is left off the copy and counted, and the
/// copy's next line says how many it missed there (file banner, One writer on the copy at a time)
/// — once the writer that held it returns. A writer that never returns (a task PROS deleted
/// mid-line) ends the copy for the rest of the boot: every later line is counted and none is copied
/// (file banner, A writer that never comes back).
class LogCopySink final : public hal::ITelemetrySink {
public:
    /// `primary` gets everything; `copy` gets log() and summarize() only. Both must outlive this.
    LogCopySink(hal::ITelemetrySink& primary, hal::ITelemetrySink& copy) noexcept
        : primary_{&primary}, copy_{&copy} {}

    /// It holds the copy's guard: neither copied nor moved.
    LogCopySink(const LogCopySink&) = delete;
    LogCopySink& operator=(const LogCopySink&) = delete;
    LogCopySink(LogCopySink&&) = delete;
    LogCopySink& operator=(LogCopySink&&) = delete;

    /// The line to the primary, then the same line to the copy — whole, in that order (the copy
    /// only when no other task is mid-copy; else counted, contendedLines()).
    void log(hal::LogLevel level, std::string_view subsystem,
             std::string_view message) override {
        primary_->log(level, subsystem, message);
        if (!enterCopy()) {
            return;
        }
        const CopyHeld held{*this};
        noteGap();
        copy_->log(level, subsystem, message);
    }

    /// The PRIMARY's answer: records are built only when the primary consumes them, never for
    /// the copy, which never receives one. Overridden as a PAIR with emit() (the seam's rule).
    [[nodiscard]] bool wantsRecord() const noexcept override { return primary_->wantsRecord(); }

    /// To the PRIMARY only: a per-tick record never reaches the copy (file banner).
    void emit(const DebugRecord& record) override { primary_->emit(record); }

    /// The summary to the primary, then to the copy (the seam's decorator rule) — the copy under
    /// the same one-writer guard as log().
    void summarize(const RunSummary& summary) override {
        primary_->summarize(summary);
        if (!enterCopy()) {
            return;
        }
        const CopyHeld held{*this};
        noteGap();
        copy_->summarize(summary);
    }

    /// The sink that gets everything.
    [[nodiscard]] hal::ITelemetrySink& primary() const noexcept { return *primary_; }

    /// The sink that gets the leveled messages and the summaries only. For READING back (which sink
    /// it is): a line written through it directly bypasses the one-writer guard, and nothing in
    /// the library does that.
    [[nodiscard]] hal::ITelemetrySink& copy() const noexcept { return *copy_; }

    /// Lines and summaries left off the copy because another task was mid-copy (file banner),
    /// since construction — every one after a writer that never returned included. They reached
    /// the primary.
    [[nodiscard]] std::uint32_t contendedLines() const noexcept {
        return contended_.load(std::memory_order_relaxed);
    }

    /// The subsystem tag of the copy's own GAP line (file banner): the diagnostics layer's.
    static constexpr std::string_view kGapTag = "DIA";

private:
    /// Under the guard, before the line or summary it is about to copy: when lines were kept off
    /// the copy since the last gap line, ONE Warn line on the copy saying how many (file banner).
    /// `reported_` is read and written under the guard only.
    void noteGap() {
        const std::uint32_t now = contended_.load(std::memory_order_relaxed);
        if (now == reported_) {
            return;
        }
        char line[112];
        std::snprintf(line, sizeof line,
                      "copy: %lu line(s) left off here while another task was writing it -- serial "
                      "has them",
                      static_cast<unsigned long>(now - reported_));
        reported_ = now;
        copy_->log(hal::LogLevel::Warn, kGapTag, line);
    }

    /// Take the copy's guard; false (and counted) when another writer holds it.
    bool enterCopy() noexcept {
        if (copyBusy_.test_and_set(std::memory_order_acquire)) {
            contended_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        return true;
    }

    /// Releases the copy's guard when its writer leaves the copy — normally, or past a copy that
    /// broke the sink contract by throwing, so one bad line never leaves the guard held. A writer
    /// that never leaves (a task PROS deleted mid-line runs no destructor) keeps it held for the
    /// rest of the boot, by design (file banner, A writer that never comes back).
    struct CopyHeld {
        explicit CopyHeld(LogCopySink& held) noexcept : sink{held} {}
        CopyHeld(const CopyHeld&) = delete;
        CopyHeld& operator=(const CopyHeld&) = delete;
        ~CopyHeld() { sink.copyBusy_.clear(std::memory_order_release); }
        LogCopySink& sink;
    };

    hal::ITelemetrySink* primary_;
    hal::ITelemetrySink* copy_;
    std::atomic_flag copyBusy_{};
    std::atomic<std::uint32_t> contended_{0};
    std::uint32_t reported_ = 0;  ///< contended_ as of the last gap line (noteGap())
};

}  // namespace shulib::diag
