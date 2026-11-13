#pragma once
//
// IFileStore — the two things the brain's SD card offers that a SAFE save can be built on (chunk
// SAFE-SAVE Part A, 2026-09-29): read a whole named file, or replace one by truncating it. The
// brain has no rename, no delete, no stat and no read-write open (below). PROS does pass an "ab"
// (append) open to a different VEXos call; what VEXos does with it is not measured, and an append
// is not a replace, so this seam leaves it out.
//
// ── Why a store, and why it is a SIBLING of IBlockSink / IBlockSource ───────────────
// IBlockSink and IBlockSource are single-file HANDLES: an adapter opens its one file at
// construction and closes it at destruction, which is exactly right for a blackbox (one file per
// boot) and for the boot's one read of the calibration record. The calibration record's SAFE save
// needs TWO named files — the record and its backup copy — each opened, written, closed and read
// back several times in one save, in an order that is the whole point
// (calibration/record_store.hpp).
// Widening either handle into "open any file, any number of times" would change what every
// existing user of it holds. So this is a new, additive seam over the same two adapters: the PROS
// implementation (hal/pros/file_store.hpp) builds a ProsBlockSource or a ProsBlockSink per call,
// each in its own scope, and the /usd/ prefix stays owned where it already is (HA-122).
//
// ── What is DELIBERATELY missing: rename, remove, stat, a read-write open (HA-166) ──
// What the vendored libpros.a and libc.a show (HA-166, read 2026-09-26), and what they leave to
// VEXos:
//   (1) fopen "wb" reaches VEXos's create-a-file call, which is TAKEN to empty an existing file at
//       the OPEN, before a byte is written (reasoned from those libraries, not measured). An "ab"
//       open reaches a different VEXos call, vexFileOpenWrite, instead — what that call does to
//       an existing file is VEXos's, and not measured;
//   (2) newlib's rename() is _link_r + _unlink_r, and remove()/unlink() are _unlink_r; PROS 4.2.2
//       defines _link, _unlink, _stat and mkdir as stubs that fail with ENOSYS. So there is no
//       rename, no delete, no directory, and no stat() to ask whether a file exists;
//   (3) PROS's usd_open_r refuses a read-write open (EINVAL): no file can be updated in place.
// A seam offering any of those would pass every HOST test (Linux has them) and fail on the robot.
// So the seam has none of them, and a file EXISTS when read() opens it.
//
// ── Two names, one file (HA-166 (6)) ────────────────────────────────────────────────
// FAT matches a long name without regard to letter case and drops trailing dots and spaces, so
// under FAT "gains.bin", "GAINS.BIN" and "gains.bin." are ONE file, while Linux, and so every
// host test, holds three. The brain's card is TAKEN to follow FAT's rule (HA-166 (6): FAT's rule
// is documented; that VEXos applies it is not measured). Were it so, a save whose backup aliased
// its primary would skip the backup (it reads back identical) and then truncate the only copy.
// sameFileOnCard() below is FAT's rule, and the safe save's preconditions use it; if the card
// tells such names apart after all, the check only refuses a few name pairs it need not have.
//
// ── What replace() promises, and what it cannot ────────────────────────────────────
//   * The old contents are to be treated as GONE from the open onward (HA-166 (1)). A replace that
//     fails at its write or its flush has already destroyed what was there.
//   * `ReplaceStep::None` does NOT mean the bytes are on the card. The writer's close cannot report
//     a failure (hal/pros/block_sink.hpp's destructor, and PROS's close returns 0 whatever VEXos
//     answers — HA-166 (5)). Only a READ-BACK proves anything, and the caller does it: see
//     calibration/record_store.hpp's put(), which never decides from what replace() reported.
//
// ── Exceptions and allocation ──────────────────────────────────────────────────────
// The methods are NOT noexcept. An implementation may throw PreconditionError for a name that is
// not bare, plus whatever its adapters' construction throws (core/check.hpp: both shipped policies
// throw; the PROS adapters allocate their path string at construction). A refusal by the CARD —
// no card, an open refused, a short write, a failed flush, a short read — is always a RETURNED
// value, never a throw. A noexcept wrapper would turn an adapter's precondition into
// std::terminate with no message. This header itself allocates nothing.
//
// Additive; not one of the frozen F4 seams. PROS-free. HA register: HA-166, HA-122.

#include <cstddef>
#include <cstdint>
#include <span>

namespace shulib::hal {

/// Capacity of the `path` a store result carries, terminator included. A longer path is CUT in
/// the result and nothing else: the path is for messages only, and the operation used the full
/// path the adapter holds.
inline constexpr std::size_t kStorePathBytes = 96;

/// What one read() saw: whether the file opened, how many bytes came back, and the path it used.
struct StoreRead {
    /// False: no card, no such file, or the open was refused. On this brain a file EXISTS exactly
    /// when a read opens it (there is no stat — HA-166 (2)).
    bool opened = false;
    /// Bytes copied into the caller's buffer, from the START of the file. 0 on an open but EMPTY
    /// file; a count below the buffer's size means the file ran out (a SHORT file, which the
    /// caller's format refuses by name). 0 when `opened` is false — both shipped stores do so,
    /// and calibration/record_store.hpp does not rely on it: it counts a read that did not open
    /// as 0 bytes, and clamps any count to the buffer, whatever a store reports.
    std::size_t count = 0;
    /// The path the read used — on the brain the adapter's `/usd/<name>` — for messages only.
    char path[kStorePathBytes] = "";
};

/// Where a replace() stopped. `None` means it reported no failure, which does NOT mean the bytes
/// are on the card (the close cannot report — header note); only a read-back proves that.
enum class ReplaceStep : std::uint8_t {
    None,    ///< opened, wrote every byte, flushed, closed — as far as the card REPORTED
    NoCard,  ///< no card: nothing was opened, nothing was created, the file is untouched
    Open,    ///< a card, but the truncating open was refused; the file is untouched
    Write,   ///< the open succeeded (the old contents are GONE) and the write fell short
    Flush,   ///< every byte written, and the flush reported failure (the bytes may still be there)
};

/// What one replace() did: the step it stopped at, and the path it used.
struct StoreReplace {
    /// The step that failed, or `None` (which proves nothing about the medium — header note).
    ReplaceStep failed = ReplaceStep::None;
    /// The path the replace used — on the brain the adapter's `/usd/<name>` — for messages only.
    char path[kStorePathBytes] = "";
};

/// The brain's SD card as a program can actually use it: READ a whole named file from its start,
/// or REPLACE a named file by truncating it. Deliberately NO rename, NO remove, NO exists-by-stat
/// and NO read-write open — the brain has none of them (HA-166, header). Names are BARE (no
/// leading '/'): the implementation owns the mount prefix. Not noexcept: a name that is not bare
/// is a thrown PreconditionError, while every refusal by the card is a returned value.
class IFileStore {
public:
    /// The polymorphic-base special members: a virtual destructor so a store may be owned and
    /// destroyed through this interface, and defaulted copy/move because the interface itself
    /// holds no state. Stores are held BY REFERENCE by the save and the load.
    virtual ~IFileStore() = default;
    IFileStore() = default;
    IFileStore(const IFileStore&) = default;
    IFileStore(IFileStore&&) = default;
    IFileStore& operator=(const IFileStore&) = default;
    IFileStore& operator=(IFileStore&&) = default;

    /// True when a card is in the brain, as the platform reports it RIGHT NOW (on the brain,
    /// `usd_is_installed()`; HA-122 (1) — confirmed at boot, never for a card pulled mid-run).
    [[nodiscard]] virtual bool cardPresent() const = 0;

    /// Read the file `bareName` from its START into `into`: up to `into.size()` bytes. The result
    /// says whether it opened, how many bytes came back, and the path used. Changes no file.
    [[nodiscard]] virtual StoreRead read(const char* bareName, std::span<std::byte> into) = 0;

    /// Replace the file `bareName` with `bytes`: open it TRUNCATING (treat the old contents as gone
    /// from here — HA-166 (1)), write, flush, close. Creates the file if it does not exist. The
    /// result names the step that failed; `None` proves nothing about the medium (header note) —
    /// read the file back to know.
    [[nodiscard]] virtual StoreReplace replace(const char* bareName,
                                               std::span<const std::byte> bytes) = 0;
};

/// True when FAT's name rule makes `a` and `b` the SAME file (header: "Two names, one file"):
/// equal once ASCII letter case is ignored and trailing dots and spaces are dropped. The brain's
/// card is taken to follow that rule (HA-166 (6); its VEXos half is not measured). So "gains.bin"
/// matches "GAINS.BIN" and "gains.bin.", and not "gains.bak".
/// constexpr, so two name constants can be checked at compile time. Null matches only null. (A FAT
/// short-name alias such as "SHULIB~1.BIN" is not checked: no caller names a file that way.)
[[nodiscard]] constexpr bool sameFileOnCard(const char* a, const char* b) noexcept {
    if (a == nullptr || b == nullptr) {
        return a == b;
    }
    std::size_t lengthA = 0;
    while (a[lengthA] != '\0') {
        ++lengthA;
    }
    std::size_t lengthB = 0;
    while (b[lengthB] != '\0') {
        ++lengthB;
    }
    while (lengthA > 0 && (a[lengthA - 1] == '.' || a[lengthA - 1] == ' ')) {
        --lengthA;
    }
    while (lengthB > 0 && (b[lengthB - 1] == '.' || b[lengthB - 1] == ' ')) {
        --lengthB;
    }
    if (lengthA != lengthB) {
        return false;
    }
    const auto lower = [](char c) {
        return c >= 'A' && c <= 'Z' ? static_cast<char>(c - 'A' + 'a') : c;
    };
    for (std::size_t i = 0; i < lengthA; ++i) {
        if (lower(a[i]) != lower(b[i])) {
            return false;
        }
    }
    return true;
}

namespace detail {
/// Copy `src` into a store result's path, cut to fit with a terminator. A null `src` gives "".
/// Bounded by hand (not snprintf) so a long path is a decision, not a -Wformat-truncation warning.
inline void copyStorePath(char (&dst)[kStorePathBytes], const char* src) noexcept {
    std::size_t i = 0;
    if (src != nullptr) {
        for (; i + 1 < kStorePathBytes && src[i] != '\0'; ++i) {
            dst[i] = src[i];
        }
    }
    dst[i] = '\0';
}
}  // namespace detail

}  // namespace shulib::hal
