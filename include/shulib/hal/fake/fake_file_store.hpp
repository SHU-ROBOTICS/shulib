#pragma once
//
// FakeFileStore — hal::IFileStore as named files in memory, with the card's failures on demand
// (chunk SAFE-SAVE Part A, 2026-09-29). Host-test only (std::vector, std::string).
//
// It exists so the calibration record's safe save (calibration/record_store.hpp) can be broken at
// EVERY operation it performs, in every way the card can break it, and the claim "every failure
// leaves the old record or the new one" can be checked rather than argued.
//
// ── The model, and what it assumes ──────────────────────────────────────────────────
// Every read() and replace() is one OPERATION, numbered from 0 in call order (cardPresent() is a
// query, not an operation). A fault is planned against an operation's number. A fault touches only
// the file its operation names: every OTHER file is unchanged — that is HA-166 (4), assumed here
// and said so — and a file flushed, closed and read back whole stays as read — HA-166 (5).
//
// REPORTED faults (the card says so):
//   * OpenRefused — a read reports not opened; a replace reports `Open` and leaves the file as it
//     was (a refused open changes nothing).
//   * ShortWrite(j) — a replace keeps the first j new bytes and reports `Write`.
//   * FlushFalse — a replace keeps EVERY new byte and reports `Flush` (as FakeBlockSink does).
//   * ShortRead(j) — a read opens and hands back at most j bytes; the file is unchanged.
// SILENT faults (the card says nothing — these are what the save's read-backs exist for):
//   * SilentCut(j) — a replace reports `None` and stores only the first j new bytes: the close
//     that could not report.
//   * SilentFlip(i) — a replace reports `None` and stores the new bytes with byte i inverted.
//   * SilentNoop — a replace reports `None` and leaves an EXISTING file exactly as it was: a write
//     whose data never reached the medium. An ABSENT file is created EMPTY with `amount` 0 (the
//     open's directory entry landed, the data did not), and stays absent with `amount` 1 (nothing
//     landed at all — the card then contradicts "a file opened for writing exists", and the save
//     can only call that UNCONFIRMED).
//   * SilentReadFlip(i) — a read reports opened and hands back the file with byte i inverted; the
//     file itself is unchanged.
// A CRASH at operation k (a power cut, a brownout, a pulled card):
//   * a crashed replace leaves its file as one of Absent / Empty / Prefix(j) — its first j new
//     bytes / AllNew / Torn(j) — the OLD length, new bytes up to j and old bytes after (or just the
//     first j new bytes when the old file was shorter than j) — and reports `Write`: it failed past
//     its open. A crashed read reports not opened and changes no file.
//   * From then on EVERY operation is refused. With `AfterCrash::CardGone` the card reads as
//     absent (cardPresent() false, reads not opened, replaces `NoCard`) — also a card PULLED while
//     the program keeps running. With `AfterCrash::CardSilent` the card still reads as present but
//     answers nothing (reads not opened, replaces `Open`): HA-122 (1) was confirmed at boot, never
//     for a card pulled mid-run.
//   * reboot() then puts the card back: it clears the refusal, the planned faults, the operation
//     count and the log, and keeps every file as the crash left it.
//
// An OPERATION LOG (file, operation, bytes, result) records every call, for the tests' order
// assertions. There is no rename and no remove on the seam, because the brain has none (HA-166);
// setFile()/eraseFile() are the TEST's hands on the card, not operations.
//
// Bare names are enforced exactly as the PROS adapters enforce them (a leading '/' or an empty
// name is a PreconditionError), so a library bug that pre-prefixes a name fails on the host.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "shulib/core/check.hpp"
#include "shulib/hal/file_store.hpp"

namespace shulib::hal::fake {

/// Which kind of operation a log entry records.
enum class FakeFileOp : std::uint8_t {
    Read,     ///< a read()
    Replace,  ///< a replace()
};

/// What a planned fault does to the operation it is planned against (header: the model).
enum class FakeFileFault : std::uint8_t {
    None,            ///< the operation runs normally
    OpenRefused,     ///< read: not opened; replace: `Open`, the file untouched
    ShortWrite,      ///< replace: keeps the first `amount` new bytes, reports `Write`
    FlushFalse,      ///< replace: keeps every new byte, reports `Flush`
    ShortRead,       ///< read: opened, hands back at most `amount` bytes
    SilentCut,       ///< replace: reports `None`, stores the first `amount` new bytes
    SilentFlip,      ///< replace: reports `None`, stores the new bytes with byte `amount` inverted
    SilentNoop,      ///< replace: reports `None`, leaves an existing file as it was (see header)
    SilentReadFlip,  ///< read: opened, hands back the file with byte `amount` inverted
    Crash,           ///< the operation is interrupted: `leave` says what its file holds
};

/// What a crashed replace leaves in its file (header: the model).
enum class CrashLeave : std::uint8_t {
    Absent,  ///< no file at all
    Empty,   ///< an empty file
    Prefix,  ///< the first `amount` new bytes
    AllNew,  ///< every new byte
    Torn,    ///< the old length: new bytes up to `amount`, old bytes after
};

/// How the card answers after a crash, until reboot().
enum class AfterCrash : std::uint8_t {
    CardGone,    ///< cardPresent() false, reads not opened, replaces `NoCard`
    CardSilent,  ///< cardPresent() TRUE, reads not opened, replaces `Open`
};

/// One planned fault: which operation, what happens, and (for a crash) what it leaves.
struct FakeFileFaultPlan {
    /// The operation's number, from 0 in call order since construction or the last reboot().
    std::size_t atOperation = 0;
    /// What happens to that operation.
    FakeFileFault fault = FakeFileFault::None;
    /// The fault's number: a byte count (ShortWrite, ShortRead, SilentCut, Prefix, Torn) or a byte
    /// index (SilentFlip, SilentReadFlip — taken modulo the length; an empty file flips nothing).
    std::size_t amount = 0;
    /// What a crashed replace leaves in its file.
    CrashLeave leave = CrashLeave::AllNew;
    /// How the card answers after a crash.
    AfterCrash after = AfterCrash::CardGone;
};

/// One operation as it happened.
struct FakeFileLogEntry {
    /// Read or replace.
    FakeFileOp op = FakeFileOp::Read;
    /// The bare name the caller passed.
    std::string file;
    /// Replace: the bytes the caller asked for. Read: the bytes handed back.
    std::vector<std::byte> bytes;
    /// Read: whether it opened. Replace: whether it got past its open (`Write`, `Flush`, `None`).
    bool opened = false;
    /// Read: the count handed back. Replace: the bytes the file holds afterwards.
    std::size_t count = 0;
    /// Replace: what it reported. Read: `None`.
    ReplaceStep step = ReplaceStep::None;
    /// Whether a planned fault (or the refusal after a crash) touched this operation.
    bool faulted = false;
};

/// hal::IFileStore as named files in memory, with every failure the card can have, on demand —
/// reported, silent, and a crash at any operation (header: the model). Host-test only.
class FakeFileStore final : public IFileStore {
public:
    /// A card holding no files, whose results name each file `<displayRoot><name>` — tests pass
    /// "/usd/" so the messages read as they do on the brain.
    explicit FakeFileStore(std::string displayRoot = "/usd/") : root_{std::move(displayRoot)} {}

    // ── IFileStore ──────────────────────────────────────────────────────────────────────

    /// False with no card, and after a crash planned with AfterCrash::CardGone (until reboot()).
    [[nodiscard]] bool cardPresent() const override {
        return present_ && !(crashed_ && after_ == AfterCrash::CardGone);
    }

    /// Read `bareName` from its start (header: the model — faults, crash, log).
    [[nodiscard]] StoreRead read(const char* bareName, std::span<std::byte> into) override {
        requireBare(bareName);
        StoreRead out{};
        detail::copyStorePath(out.path, (root_ + bareName).c_str());
        FakeFileLogEntry entry{};
        entry.op = FakeFileOp::Read;
        entry.file = bareName;
        const FakeFileFaultPlan plan = takePlan();
        if (!present_ || crashed_) {
            entry.faulted = crashed_;
            log_.push_back(std::move(entry));
            return out;
        }
        if (plan.fault == FakeFileFault::Crash) {
            crashed_ = true;
            after_ = plan.after;
            entry.faulted = true;
            log_.push_back(std::move(entry));
            return out;  // a crashed read opens nothing and changes no file
        }
        const auto held = find(bareName);
        if (!held.has_value() || plan.fault == FakeFileFault::OpenRefused) {
            entry.faulted = plan.fault == FakeFileFault::OpenRefused;
            log_.push_back(std::move(entry));
            return out;
        }
        std::vector<std::byte> served = files_[*held].second;
        if (plan.fault == FakeFileFault::ShortRead && plan.amount < served.size()) {
            served.resize(plan.amount);
        }
        if (plan.fault == FakeFileFault::SilentReadFlip && !served.empty()) {
            served[plan.amount % served.size()] ^= std::byte{0xFF};
        }
        const std::size_t take = into.size() < served.size() ? into.size() : served.size();
        for (std::size_t i = 0; i < take; ++i) {
            into[i] = served[i];
        }
        out.opened = true;
        out.count = take;
        entry.opened = true;
        entry.count = take;
        entry.bytes.assign(served.begin(), served.begin() + static_cast<std::ptrdiff_t>(take));
        entry.faulted = plan.fault == FakeFileFault::ShortRead
                        || plan.fault == FakeFileFault::SilentReadFlip;
        log_.push_back(std::move(entry));
        return out;
    }

    /// Replace `bareName` with `bytes` (header: the model — faults, crash, log).
    [[nodiscard]] StoreReplace replace(const char* bareName,
                                       std::span<const std::byte> bytes) override {
        requireBare(bareName);
        StoreReplace out{};
        detail::copyStorePath(out.path, (root_ + bareName).c_str());
        FakeFileLogEntry entry{};
        entry.op = FakeFileOp::Replace;
        entry.file = bareName;
        entry.bytes.assign(bytes.begin(), bytes.end());
        const FakeFileFaultPlan plan = takePlan();
        const auto held = find(bareName);
        entry.count = held.has_value() ? files_[*held].second.size() : 0U;
        if (!present_ || crashed_) {
            out.failed = present_ && after_ == AfterCrash::CardSilent ? ReplaceStep::Open
                                                                        : ReplaceStep::NoCard;
            entry.step = out.failed;
            entry.faulted = crashed_;
            log_.push_back(std::move(entry));
            return out;
        }
        if (plan.fault == FakeFileFault::OpenRefused) {
            out.failed = ReplaceStep::Open;
            entry.step = out.failed;
            entry.faulted = true;
            log_.push_back(std::move(entry));
            return out;
        }
        const std::vector<std::byte> incoming(bytes.begin(), bytes.end());
        const std::vector<std::byte> previous =
            held.has_value() ? files_[*held].second : std::vector<std::byte>{};
        std::optional<std::vector<std::byte>> stored = incoming;
        switch (plan.fault) {
            case FakeFileFault::ShortWrite:
                stored = prefix(incoming, plan.amount);
                out.failed = ReplaceStep::Write;
                break;
            case FakeFileFault::FlushFalse:
                out.failed = ReplaceStep::Flush;
                break;
            case FakeFileFault::SilentCut:
                stored = prefix(incoming, plan.amount);
                break;
            case FakeFileFault::SilentFlip:
                if (!stored->empty()) {
                    (*stored)[plan.amount % stored->size()] ^= std::byte{0xFF};
                }
                break;
            case FakeFileFault::SilentNoop:
                if (held.has_value()) {
                    stored = previous;
                } else if (plan.amount == 0) {
                    stored = std::vector<std::byte>{};
                } else {
                    stored = std::nullopt;
                }
                break;
            case FakeFileFault::Crash:
                stored = crashLeaves(plan, incoming, previous);
                out.failed = ReplaceStep::Write;
                crashed_ = true;
                after_ = plan.after;
                break;
            default:
                break;
        }
        if (stored.has_value()) {
            put(bareName, std::move(*stored));
        } else {
            eraseFile(bareName);
        }
        entry.step = out.failed;
        entry.opened = true;
        const auto after = find(bareName);
        entry.count = after.has_value() ? files_[*after].second.size() : 0U;
        entry.faulted = plan.fault != FakeFileFault::None;
        log_.push_back(std::move(entry));
        return out;
    }

    // ── The test's hands on the card (not operations; nothing is logged) ────────────────

    /// Put a file on the card as a laptop would.
    void setFile(const std::string& name, std::vector<std::byte> bytes) {
        put(name, std::move(bytes));
    }

    /// Take a file off the card as a laptop would (the SEAM has no delete — HA-166 (2)).
    void eraseFile(const std::string& name) {
        const auto at = find(name);
        if (at.has_value()) {
            files_.erase(files_.begin() + static_cast<std::ptrdiff_t>(*at));
        }
    }

    /// What the card holds under `name`, or nothing when there is no such file.
    [[nodiscard]] std::optional<std::vector<std::byte>> file(const std::string& name) const {
        const auto at = find(name);
        if (!at.has_value()) {
            return std::nullopt;
        }
        return files_[*at].second;
    }

    /// Insert or remove the card itself (default: present). Without a card every read is not
    /// opened and every replace reports `NoCard`, and nothing is created.
    void setCardPresent(bool present) noexcept { present_ = present; }

    /// Plan a fault against an operation number (header: the model). Several may be planned; a
    /// second plan for the same number replaces the first.
    void plan(const FakeFileFaultPlan& p) {
        for (FakeFileFaultPlan& existing : plans_) {
            if (existing.atOperation == p.atOperation) {
                existing = p;
                return;
            }
        }
        plans_.push_back(p);
    }

    /// Put the card back and restart the program: clears the crash's refusal, every planned
    /// fault, the operation count and the log. The files stay exactly as they were left.
    void reboot() {
        crashed_ = false;
        plans_.clear();
        operations_ = 0;
        log_.clear();
    }

    /// Every operation since construction or the last reboot(), in order.
    [[nodiscard]] const std::vector<FakeFileLogEntry>& log() const noexcept { return log_; }

    /// How many operations have run since construction or the last reboot().
    [[nodiscard]] std::size_t operations() const noexcept { return operations_; }

    /// Whether a planned crash has happened (and reboot() has not yet been called).
    [[nodiscard]] bool crashed() const noexcept { return crashed_; }

private:
    static void requireBare(const char* name) {
        SHULIB_PRECONDITION(name != nullptr && name[0] != '\0',
                            "FakeFileStore: name is null/empty");
        SHULIB_PRECONDITION(name[0] != '/',
                            "FakeFileStore: name must be BARE -- the store owns the mount prefix "
                            "(HA-122)");
    }

    [[nodiscard]] std::optional<std::size_t> find(const std::string& name) const {
        for (std::size_t i = 0; i < files_.size(); ++i) {
            if (files_[i].first == name) {
                return i;
            }
        }
        return std::nullopt;
    }

    void put(const std::string& name, std::vector<std::byte> bytes) {
        const auto at = find(name);
        if (at.has_value()) {
            files_[*at].second = std::move(bytes);
            return;
        }
        files_.emplace_back(name, std::move(bytes));
    }

    [[nodiscard]] FakeFileFaultPlan takePlan() {
        const std::size_t index = operations_++;
        for (const FakeFileFaultPlan& p : plans_) {
            if (p.atOperation == index) {
                return p;
            }
        }
        return FakeFileFaultPlan{};
    }

    [[nodiscard]] static std::vector<std::byte> prefix(const std::vector<std::byte>& v,
                                                       std::size_t n) {
        return {v.begin(), v.begin() + static_cast<std::ptrdiff_t>(n < v.size() ? n : v.size())};
    }

    [[nodiscard]] static std::optional<std::vector<std::byte>> crashLeaves(
        const FakeFileFaultPlan& p, const std::vector<std::byte>& incoming,
        const std::vector<std::byte>& previous) {
        switch (p.leave) {
            case CrashLeave::Absent: return std::nullopt;
            case CrashLeave::Empty: return std::vector<std::byte>{};
            case CrashLeave::Prefix: return prefix(incoming, p.amount);
            case CrashLeave::AllNew: return incoming;
            case CrashLeave::Torn: {
                std::vector<std::byte> torn = prefix(incoming, p.amount);
                for (std::size_t i = torn.size(); i < previous.size(); ++i) {
                    torn.push_back(previous[i]);
                }
                return torn;
            }
        }
        return incoming;
    }

    std::string root_;
    std::vector<std::pair<std::string, std::vector<std::byte>>> files_;
    std::vector<FakeFileFaultPlan> plans_;
    std::vector<FakeFileLogEntry> log_;
    std::size_t operations_ = 0;
    bool present_ = true;
    bool crashed_ = false;
    AfterCrash after_ = AfterCrash::CardGone;
};

}  // namespace shulib::hal::fake
