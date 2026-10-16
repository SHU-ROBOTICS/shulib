#pragma once
//
// ProsBlockSink — IBlockSink over a PROS FILE* on the V5's SD card (chunk
// R1b): where the blackbox's binary blocks physically go. The device seam E1
// shipped the interface for (block_sink.hpp names R1 as this file's owner);
// diag/'s SdSink writes through it.
//
// BINDS:
//  * ::pros::usd::is_installed() — the no-card probe, at construction
//    (vendored misc.hpp:555-568; HA-122)
//  * std::fopen/std::fwrite/std::fflush on "/usd/<name>" — the PROS kernel
//    mounts the SD card at /usd/ for newlib file IO (HA-122)
//
// ── THE /usd/ PATH QUIRK (HA-122 — register + FAQ) ─────────────────────────────────
// Two conventions in ONE API, the same shape as R1a's 0-vs-1-indexed registry
// finding: usd_list_files() documents "DO NOT PREPEND YOUR PATHS WITH /usd/"
// (vendored misc.h:824-825) while fopen REQUIRES the /usd/ prefix. This
// adapter owns the prefix in exactly one place: the constructor takes a BARE
// file name (a leading '/' is a loud precondition, so the double-prefix
// mistake cannot compile out of sight), and joins it to `mountRoot` —
// "/usd/" on the robot, injectable so a host test can point it at a real
// temp directory and exercise THIS code path, not a copy of it.
//
// ── NO CARD AT BOOT (T5's ruling) ──────────────────────────────────────────────────
// Construction SUCCEEDS, write() returns false from the first call, and the
// fact is visible through isOpen() — checked once by the composition root /
// diagnostics layer, which owns saying it out loud (hal/ is below diag/). A
// missing SD card must not stop a robot from driving, and E1's drop-and-
// count design already handles a sink that refuses; a throw here would turn
// a missing card into a dead robot. Rejected: silently succeeding — the
// exact "nothing looks wrong" failure the blackbox exists to avoid.
//
// CONTRACT (block_sink.hpp): bytes VERBATIM, synchronous, MUST NOT throw —
// fwrite/fflush cannot throw, and both paths return bool. A short fwrite
// (card full, yanked, dying) returns false and leaves a prefix on the
// device, which is exactly what the format is designed to decode up to.
// flush() is fflush() — pushing newlib's buffer to the FatFS driver; there
// is no fsync in PROS's exposed surface, so "on the medium" is as strong as
// the platform allows (HA-122's weak half — bench: yank the card after a
// flushed write and count what survived). PROS's own usd_write_r does call
// vexFileSync after every write it passes on, ignoring what it returns; what
// that sync commits is VEXos's (HA-166 (5)).
//
// OWNERSHIP: this adapter OWNS its FILE* (fclose at destruction), so it is
// non-copyable/non-movable — unlike ProsCharSink, which borrows a descriptor (the serial's, fd 1).
//
// ── THE APPEND MODE (chunk COMP-t, 2026-09-28 — additive) ─────────────────────────
// ProsFileMode::Append keeps every earlier byte of the file and writes after them: the
// Auton Tester's run log, ONE file on the card across reboots. The decision is made in
// ONE function, openForAppend() below, host-tested with a scripted opener: ask with "rb"
// whether the file exists, open "ab", and create with "wb" ONLY when the file was absent,
// so an existing log is never truncated whichever way VEXos answers an append on a
// missing file (PROS hands an O_APPEND open to vexFileOpenWrite; whether that creates is
// VEXos's — A4 register HA-177). Every mode string lives in this file, none with a '+'
// (the brain refuses read-write opens), and the two-argument constructor still means
// Truncate, byte for byte, for every existing caller.
//
// HA register: HA-122, HA-177 (docs/hardware-assumptions.md).

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "pros/misc.hpp"
#pragma GCC diagnostic pop

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string>
#include <string_view>

#include "shulib/core/check.hpp"
#include "shulib/hal/block_sink.hpp"

namespace shulib::hal::pros {

/// How a ProsBlockSink opens its file (header: THE APPEND MODE). The default, and every caller
/// before COMP-t, is Truncate.
enum class ProsFileMode : std::uint8_t {
    Truncate,  ///< "wb": one file per boot (the blackbox's; every existing caller)
    Append,    ///< keep every earlier byte and write after them (openForAppend below)
};

/// How an Append open went — for the caller's one boot line.
enum class AppendOpen : std::uint8_t {
    Appended,  ///< the file existed; new bytes go after its last byte
    Created,   ///< the file did not exist; it was created
    Refused,   ///< no card, or the file exists but would not open for appending (never truncated)
};

/// THE APPEND OPEN, decided here and nowhere else (header: THE APPEND MODE): ask with "rb"
/// whether `path` exists (closing that probe with `close`), open "ab", and if that fails create
/// with "wb" ONLY when the file was absent — an existing log is never truncated. `open(path,
/// mode)` returns a FILE* or nullptr; the adapter passes std::fopen, a host test a script.
/// `how` says which branch was taken. No mode here has a '+'. Returns the open FILE*, or
/// nullptr (then `how` is Refused).
template <class Open, class Close>
[[nodiscard]] std::FILE* openForAppend(const char* path, Open&& open, Close&& close,
                                       AppendOpen& how) {
    bool existed = false;
    if (std::FILE* probe = open(path, "rb"); probe != nullptr) {
        existed = true;
        close(probe);
    }
    if (std::FILE* f = open(path, "ab"); f != nullptr) {
        how = existed ? AppendOpen::Appended : AppendOpen::Created;
        return f;
    }
    if (!existed) {
        if (std::FILE* f = open(path, "wb"); f != nullptr) {
            how = AppendOpen::Created;
            return f;
        }
    }
    how = AppendOpen::Refused;
    return nullptr;
}

/// IBlockSink over a PROS FILE* on the V5's SD card — where the blackbox's binary blocks
/// physically land. It OWNS the file: opened at construction — truncating by default (one file
/// per boot), or appending (ProsFileMode::Append, since COMP-t) — and closed at destruction, so
/// it must outlive every SdSink or char sink writing through it. A MISSING CARD IS NOT AN
/// ERROR — construction still succeeds and the sink simply refuses (isOpen() false, every
/// write()/flush() false), because a robot must still drive without a card, and E1's
/// drop-and-count design already handles a sink that says no. It also owns the /usd/ prefix:
/// pass a BARE file name.
class ProsBlockSink final : public IBlockSink {
public:
    /// Open `<mountRoot><fileName>` for binary writing (truncating — one
    /// blackbox file per boot). `fileName` must be BARE: no leading '/' (the
    /// adapter owns the /usd/ prefix — header note). `mountRoot` must end in
    /// '/'; it defaults to the robot truth and exists so host tests exercise
    /// the same join+open path. No card / failed open → a refusing sink, NOT
    /// a throw (T5): isOpen() false, every write()/flush() false.
    explicit ProsBlockSink(const char* fileName, const char* mountRoot = "/usd/")
        : ProsBlockSink(fileName, ProsFileMode::Truncate, mountRoot) {}

    /// The same, in `mode`: Truncate is exactly the constructor above; Append keeps the file's
    /// earlier bytes and writes after them (openForAppend — `appendOpen()` says how it went).
    /// The prefix rule, the no-card refusal and the preconditions are the same.
    ProsBlockSink(const char* fileName, ProsFileMode mode, const char* mountRoot = "/usd/") {
        SHULIB_PRECONDITION(fileName != nullptr && fileName[0] != '\0',
                            "ProsBlockSink: fileName is null/empty");
        SHULIB_PRECONDITION(fileName[0] != '/',
                            "ProsBlockSink: fileName must be BARE — the adapter owns the "
                            "/usd/ prefix (do not pre-prefix; HA-122)");
        SHULIB_PRECONDITION(mountRoot != nullptr && mountRoot[0] != '\0',
                            "ProsBlockSink: mountRoot is null/empty");
        SHULIB_PRECONDITION(std::string_view{mountRoot}.back() == '/',
                            "ProsBlockSink: mountRoot must end in '/'");
        path_ = std::string{mountRoot} + fileName;
        if (::pros::usd::is_installed() != 0) {  // the no-card probe (HA-122)
            if (mode == ProsFileMode::Append) {
                file_ = openForAppend(
                    path_.c_str(), [](const char* p, const char* m) { return std::fopen(p, m); },
                    [](std::FILE* f) { std::fclose(f); }, append_);
            } else {
                file_ = std::fopen(path_.c_str(), "wb");
            }
        }
    }

    /// fclose the file, which also pushes newlib's remaining buffer out. Nothing else
    /// holds this FILE*, so anything still writing through this sink — an SdSink, most
    /// likely — must be destroyed first. No-op on a refusing sink.
    ///
    /// THE ONE UNREPORTABLE FAILURE, stated rather than left implicit. fclose FLUSHES before
    /// it closes and returns EOF if that write fails — the card-full, card-yanked, dying-card
    /// case — and a destructor has no channel to say so. Every other path in this class is
    /// bool-valued for exactly that reason (write() is even [[nodiscard]], and block_sink.hpp
    /// justifies it: "a caller that ignores the result cannot notice a truncated file"). The
    /// last buffered block is both the most likely to be lost and the only one whose loss
    /// nothing here can report. A caller that needs the final bytes CONFIRMED must call
    /// flush(), which is bool for this reason, before letting the sink die.
    ~ProsBlockSink() override {
        if (file_ != nullptr) {
            std::fclose(file_);
        }
    }

    /// Non-copyable and non-movable: this adapter OWNS the FILE*, and a second handle to
    /// it would fclose the same file twice. (ProsCharSink merely borrows a descriptor — the
    /// serial's, fd 1 — which is why it carries no such restriction: ownership, not policy.)
    ProsBlockSink(const ProsBlockSink&) = delete;
    ProsBlockSink& operator=(const ProsBlockSink&) = delete;
    ProsBlockSink(ProsBlockSink&&) = delete;
    ProsBlockSink& operator=(ProsBlockSink&&) = delete;

    /// Verbatim bytes; false unless EVERY byte was accepted (a short write
    /// leaves a prefix — the format decodes up to the cut). False always
    /// while refusing (no card / failed open).
    [[nodiscard]] bool write(std::span<const std::byte> bytes) noexcept override {
        if (file_ == nullptr) {
            return false;
        }
        if (bytes.empty()) {
            return true;  // nothing to accept; also keeps fwrite off a null data()
        }
        return std::fwrite(bytes.data(), 1, bytes.size(), file_) == bytes.size();
    }

    /// fflush to the FatFS driver (the platform's strongest "on the medium" —
    /// header note). False on device failure or while refusing.
    bool flush() noexcept override {
        if (file_ == nullptr) {
            return false;
        }
        return std::fflush(file_) == 0;
    }

    /// False = the sink is refusing (no card at boot, or the open failed).
    /// The composition root checks this ONCE and reports through the
    /// diagnostics layer — the T5 visibility rule.
    [[nodiscard]] bool isOpen() const noexcept { return file_ != nullptr; }

    /// The full path this sink writes (for the one-time diagnostics line).
    [[nodiscard]] const char* path() const noexcept { return path_.c_str(); }

    /// How an Append open went: Appended, Created, or Refused (no card, or an existing file
    /// that would not open for appending). Always Refused in Truncate mode, which never asks.
    [[nodiscard]] AppendOpen appendOpen() const noexcept { return append_; }

private:
    std::string path_;
    std::FILE* file_ = nullptr;
    AppendOpen append_ = AppendOpen::Refused;
};

}  // namespace shulib::hal::pros
