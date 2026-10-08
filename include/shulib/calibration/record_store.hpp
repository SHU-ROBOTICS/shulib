#pragma once
//
// The calibration record's SAFE SAVE and its LOAD (chunk SAFE-SAVE Part A, 2026-09-29): a save
// designed so a failure part-way does not leave the card without a good record — given HA-166 (4)
// and (5), short of a stated two-fault limit — a load that falls back to a verified backup copy,
// and a report that says what the card MEASURABLY holds afterwards.
//
// ── The defect this exists for (found 2026-09-26) ───────────────────────────────────
// "shulib Calibrate" saved by opening the record file with fopen "wb" and writing it. On this brain
// "wb" reaches VEXos's create-a-file call, which is taken to empty an existing file AT THE OPEN
// (HA-166 (1); reasoned, not measured), so every failure after the open — a refused write, a
// refused flush, a close that cannot report, a pulled card, a power cut — left the card with NO
// usable record, and the program then printed that the earlier record "is still on the card". The
// obvious fix, "write a temporary file and rename it over the old one", cannot be built: the brain
// has no rename and no delete (HA-166 (2)), and no read-write open (HA-166 (3)). A rename-based
// save would pass every host test (Linux has rename) and fail on the robot.
//
// ── The design: BACKUP FIRST, then the record, then refresh the backup ──────────────
// P is the primary file (the record), K its backup. put(X, b) means: replace(X, b), keep the step
// it reported for the WORDS only, then ALWAYS read X back; put succeeds if and only if the read
// opened and returned exactly b (same length, same bytes) and accept() takes it. The decision
// never uses the step replace() reported — the close cannot report (hal/file_store.hpp), and a
// false flush that kept the bytes is a SAVE — Calibrate's ruling P5 applied as written: "SAVED is
// a measurement or it is not said", and so is its absence.
//   0. The bytes must be accepted and a card present; otherwise NOT SAVED with nothing opened.
//   1. READ THE STARTING CARD: P and K, both, always — and P a SECOND time, before K, when its
//      first read is not accepted. A refused, short or altered read is not a measurement (below:
//      "last seen"), and without the re-read one such misread of a GOOD record file would make the
//      save treat it as damaged, back nothing up, and truncate the only copy at step 2. The record
//      IN EFFECT BEFORE is the file chooseEffective() names. If it is P and K is not byte-identical
//      to it, put(K, old) — and if that fails, stop with P never opened for writing. If P is NOT
//      accepted, K is not written: it may hold the only good record.
//   2. put(P, bytes). If that fails and the old record came from P: if P already reads back as the
//      old bytes, nothing changed and there is no restore; otherwise put(P, old) — the restore.
//   3. Only if step 2 succeeded: put(K, bytes), refreshing the backup. Its failure changes nothing
//      about P.
//   4. MEASURE, never assume: read P and K again. The result is UNCONFIRMED, and nothing more is
//      claimed, when (u1) the card is gone, (u2) no operation in the whole save got a file open,
//      or (u3) a file does not read back as the save last saw it. Otherwise chooseEffective() names
//      the file the next boot will use; SAVED if and only if that file is P and it holds the new
//      bytes.
// THE GUARANTEE, and its conditions. Given HA-166 (4) (writing one file never damages another) and
// (5) (a file flushed, closed and read back whole is on the medium), ONE failure at any step — a
// refused open or read, a short write or read, a false flush, a write that silently did not land,
// a power cut or a pulled card — leaves the card holding the record in effect before the save, or
// the new one; so does a silent write fault followed by a crash, and a misread the checks can see
// (refused, short, or failing the CRC) at any one read followed by a crash or by a write fault.
// The tests enumerate each of these. THE LIMIT — what it does NOT defend: a misread at step 1 that
// the checks cannot catch, followed by a second failure (a failed write or a power cut) in the
// same save. Two kinds: (i) the record file misread as damaged at BOTH of its first two reads;
// (ii) a read that returns a VALID record the file does not hold — a stale read (HA-139 is the
// belief that a read returns what the card holds): the record file read as some other record
// than it holds (it is damaged, missing or different), or the backup (damaged, older or missing)
// read as identical to the record. The second failure can then lose the record, or leave the next
// boot a record other than the one in effect (an older one). The save's words rest on what step
// 1 READ: after (i) they say only that the record file "did not read as usable"; after (ii) they
// call the record the stale read returned "the earlier record", and can say it is still on the
// card. The tests pin both kinds (THE STATED LIMIT, measured).
//
// ── "Last seen", and why a refused read is not a measurement of absence ─────────────
// For each file the save keeps what it last read — (opened, length, CRC-32), never a copy. A read
// that opens sets it. A read that does NOT open, after a replace of that file got past its open,
// sets it to UNKNOWN: the save has just written that file and there is no delete, so the refusal
// is a failed read, not an absent file. Any other refused read leaves it as it was. Step 4 compares
// against it — which is what keeps a card that fails at the save's very first read from being
// "measured" as empty, and reported as holding nothing, with a good record on it.
//
// ── The load rule, in one place ─────────────────────────────────────────────────────
// chooseEffective(): P if accepted, else K if accepted, else none. The save's step 1 and step 4 use
// it, and so does loadWithBackup(), which reads K only when P is not accepted. For the record,
// "accepted" is decodeRecord() succeeding — magic, version, length (exactly 180), CRC and valid() —
// so a TORN file is a refusal, never a misread.
//
// ── Contract ────────────────────────────────────────────────────────────────────────
//   * Allocates nothing. Exception-NEUTRAL, never noexcept: it catches nothing and throws only what
//     the store throws plus SHULIB_PRECONDITION on its own arguments (null names; two names FAT's
//     rule makes ONE file — equal, or differing only in letter case or in trailing dots and
//     spaces, hal::sameFileOnCard(), HA-166 (6); bytes longer than MaxBytes). A noexcept here
//     would turn an adapter's bare-name precondition into std::terminate with no message.
//   * accept is a template parameter (a callable), never std::function:
//       bool accept(std::span<const std::byte> bytes, char* why, std::size_t whyCap)
//     It sees every byte read, up to MaxBytes + 32 — so a LONGER file reaches it and must be
//     refused by length. `why` may be null with whyCap 0. An EMPTY file is never passed to it:
//     an empty file is never accepted, and neither are empty bytes to save.
//   * A store's read COUNT is never trusted past the buffer it was handed: a count above it (a
//     broken store — IFileStore promises "up to into.size()") is clamped to the buffer, so it can
//     never become a CRC, a copy or an accept() over bytes past the end of a stack array. And a
//     read that did NOT open counts 0 bytes, whatever count it carries — never accepted, never
//     copied, never a CRC (both the load and the save check `opened` themselves as well).
//   * STACK (sizes measured with the V5's arm-none-eabi-g++, 2026-09-29; re-measured 2026-10-01).
//     safeReplace<MaxBytes> holds exactly two arrays of MaxBytes + 32 bytes (the old record, for
//     the restore; one read scratch every read reuses) — 424 bytes for the record, 496 with the
//     save's other state — and returns a SaveReport (452 bytes) into the caller's frame.
//     loadWithBackup<MaxBytes> returns a BackupLoad (MaxBytes + 640: 820 bytes for the record)
//     into the caller's frame; loadRecord() holds one of those on its own frame and returns a
//     LoadedRecord (792 bytes). MaxBytes is capped at 1024 (kMaxStoredFileBytes) so no caller puts
//     a large file on a task stack by accident.
//
// NO HARDWARE CLAIM: nothing here has run on a brain. The six beliefs it rests on are HA-166.
// PROS-free, sim-free, allocation-free.

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>

#include "shulib/calibration/calibration_record.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/file_store.hpp"

namespace shulib::calibration {

// ═══ THE LOAD RULE ══════════════════════════════════════════════════════════════════

/// Which of the two files a load uses, or a save measured as the one the next boot will use.
enum class Which : std::uint8_t {
    Primary,  ///< the primary file (for the record: shulib_drivetrain.bin)
    Backup,   ///< the backup copy (for the record: shulib_drivetrain.bak)
    None,     ///< neither is usable
};

/// THE load rule, in one place: the primary file if it is accepted, else the backup if it is
/// accepted, else neither. The save's first and last reads and loadWithBackup() all use it.
[[nodiscard]] constexpr Which chooseEffective(bool primaryAccepted, bool backupAccepted) noexcept {
    return primaryAccepted ? Which::Primary : (backupAccepted ? Which::Backup : Which::None);
}

/// Bytes a store's read buffer holds beyond the largest file of its kind, so a LONGER file shows
/// up as a length refusal instead of "the first MaxBytes bytes happened to fit".
inline constexpr std::size_t kStoreReadSlackBytes = 32;

/// The largest file safeReplace() and loadWithBackup() will put on a stack (header: STACK).
inline constexpr std::size_t kMaxStoredFileBytes = 1024;

/// Capacity of a refusal reason, terminator included — the size the boot's messages already use.
inline constexpr std::size_t kFileReasonBytes = 192;

/// One file, as a load saw it — generic over the file's kind.
struct FileRead {
    /// The load read this file (the backup only when the primary was not accepted).
    bool tried = false;
    /// False: no card, or no such file (or the open was refused).
    bool opened = false;
    /// Bytes read; 0 on an open but EMPTY file, and 0 when the read did not open.
    std::size_t count = 0;
    /// accept() took it (never for an empty file, never for a read that did not open).
    bool accepted = false;
    /// accept()'s refusal, when the file opened, held bytes and was not accepted; "" otherwise.
    char why[kFileReasonBytes] = "";
    /// The path the store's read used (the adapter's path() on the brain), for messages only.
    char path[hal::kStorePathBytes] = "";
};

/// What loadWithBackup() found: which file it chose, what it saw of each, and the chosen bytes.
/// A return value, so it lives in the CALLER's frame — MaxBytes + 640 bytes on a V5 (header:
/// STACK).
template <std::size_t MaxBytes>
struct BackupLoad {
    /// Primary, Backup, or None — decided by chooseEffective().
    Which from = Which::None;
    /// The primary file as the load saw it (always tried).
    FileRead primary{};
    /// The backup as the load saw it (tried only when the primary was not accepted).
    FileRead backup{};
    /// The chosen file's length; 0 when `from` is None.
    std::size_t count = 0;
    /// The chosen file's bytes, first `count` of them; meaningful unless `from` is None.
    std::array<std::byte, MaxBytes + kStoreReadSlackBytes> bytes{};
};

namespace detail {
/// The bytes a read put in a buffer of `capacity`: 0 when it did not open, and never more than the
/// buffer holds, whatever the store reported (header: Contract — a broken store's count).
[[nodiscard]] constexpr std::size_t countRead(const hal::StoreRead& r,
                                              std::size_t capacity) noexcept {
    return !r.opened ? 0U : (r.count < capacity ? r.count : capacity);
}

/// Read `name` into `into` and judge it with `accept`, filling `seen`. Returns the count read.
template <typename Accept>
std::size_t loadOne(hal::IFileStore& store, const char* name, std::span<std::byte> into,
                    FileRead& seen, Accept& accept) {
    const hal::StoreRead r = store.read(name, into);
    seen.tried = true;
    seen.opened = r.opened;
    seen.count = countRead(r, into.size());
    std::memcpy(seen.path, r.path, sizeof seen.path);
    // `opened` here as well as inside countRead(), as the save's readFile() does: a read that did
    // not open is never accepted, whatever count the store reported (header: Contract).
    seen.accepted = seen.opened && seen.count > 0
                    && accept(std::span<const std::byte>{into.data(), seen.count}, seen.why,
                              sizeof seen.why);
    if (seen.accepted) {
        seen.why[0] = '\0';
    }
    return seen.count;
}
}  // namespace detail

/// THE BYTE-LEVEL LOAD. Reads `primary`; reads `backup` ONLY when the primary is not accepted;
/// chooses by chooseEffective(). Generic over the file's kind (the record's is loadRecord(); a
/// sibling file of up to kMaxStoredFileBytes reuses this). Not noexcept: preconditions (a null
/// name, or two names FAT's rule makes one file — hal::sameFileOnCard()) and the store's own
/// throws propagate; every refusal by the card is in the result.
template <std::size_t MaxBytes, typename Accept>
[[nodiscard]] BackupLoad<MaxBytes> loadWithBackup(hal::IFileStore& store, const char* primary,
                                                  const char* backup, Accept&& accept) {
    static_assert(MaxBytes >= 1 && MaxBytes <= kMaxStoredFileBytes,
                  "loadWithBackup: MaxBytes must be 1..1024 -- the buffer lives on the caller's "
                  "stack");
    SHULIB_PRECONDITION(primary != nullptr && backup != nullptr,
                        "loadWithBackup: a file name is null");
    SHULIB_PRECONDITION(!hal::sameFileOnCard(primary, backup),
                        "loadWithBackup: the primary and its backup must be two different files on "
                        "the card (FAT ignores letter case and trailing dots and spaces)");
    BackupLoad<MaxBytes> out{};
    const std::size_t primaryCount =
        detail::loadOne(store, primary, std::span<std::byte>{out.bytes}, out.primary, accept);
    std::size_t backupCount = 0;
    if (!out.primary.accepted) {
        backupCount =
            detail::loadOne(store, backup, std::span<std::byte>{out.bytes}, out.backup, accept);
    }
    out.from = chooseEffective(out.primary.accepted, out.backup.accepted);
    out.count = out.from == Which::Primary  ? primaryCount
                : out.from == Which::Backup ? backupCount
                                            : 0U;
    return out;
}

// ═══ THE SAVE ═══════════════════════════════════════════════════════════════════════

/// A save's outcome word. (Not calibrate_panel.hpp's SaveOutcome, which is the PANEL's three
/// states; a caller maps Saved to that one's Saved, and both others to its NotSaved.)
enum class SaveResult : std::uint8_t {
    Saved,        ///< measured: the next boot uses the primary file, and it holds the new bytes
    NotSaved,     ///< measured: the next boot does NOT get the new bytes (the report says what)
    Unconfirmed,  ///< the card could not be measured (u1-u3): nothing is claimed
};

/// Why a save stopped before touching the card (nothing was opened), or None.
enum class SaveRefusal : std::uint8_t {
    None,                ///< the save touched the card
    NoCard,              ///< cardPresent() was false at the start
    BytesRefused,        ///< accept() refused the bytes to save (the reason is in `why`)
    DescriptionInvalid,  ///< saveRecordSafely(): the description's valid() refused it (`why`)
    EncodeFailed,        ///< saveRecordSafely(): the description would not encode (`why`)
};

/// Why a save is Unconfirmed (header: step 4), or None.
enum class UnconfirmedWhy : std::uint8_t {
    None,             ///< the save was measured
    CardGone,         ///< (u1) cardPresent() is false at the end
    NothingOpened,    ///< (u2) no operation in the whole save got a file open
    ReadBackDiffers,  ///< (u3) a file did not read back as the save last saw it
};

/// What a file held at the save's final read (step 4), relative to the save's two records.
enum class FileHolds : std::uint8_t {
    NotMeasured,  ///< step 4 did not run (the save stopped before touching the card)
    Absent,       ///< the read did not open
    NotAccepted,  ///< it opened and was empty, short, long or refused by accept()
    New,          ///< accepted, byte-identical to this save's bytes
    Earlier,      ///< accepted, byte-identical to the record in effect before the save
    Other,        ///< accepted, and neither
};

/// One put(): a replace, then ALWAYS a read-back. `reported` is for the words only — `verified`
/// alone says whether the file holds the bytes (header).
struct PutRecord {
    /// The save issued this put.
    bool attempted = false;
    /// What the replace reported (hal::ReplaceStep). Never used to decide anything.
    hal::ReplaceStep reported = hal::ReplaceStep::None;
    /// The read-back opened.
    bool readOpened = false;
    /// Bytes the read-back returned.
    std::size_t readCount = 0;
    /// The read-back returned exactly the bytes written, and accept() took them.
    bool verified = false;
};

/// What a safe save did and what it MEASURED. Every field is filled from what happened and from
/// the final read of both files — none from what a step meant to do.
struct SaveReport {
    /// Saved, NotSaved or Unconfirmed (header: step 4).
    SaveResult outcome = SaveResult::NotSaved;
    /// Why the save stopped before touching the card, or None.
    SaveRefusal refused = SaveRefusal::None;
    /// The refusal's reason for BytesRefused / DescriptionInvalid / EncodeFailed; "" otherwise.
    char why[kFileReasonBytes] = "";
    /// Length of the bytes this save wrote (or meant to).
    std::size_t length = 0;
    /// The record in effect BEFORE the save, by chooseEffective() over step 1's reads.
    Which before = Which::None;
    /// Step 1: the backup made from the primary's old bytes (attempted only when K differed).
    PutRecord backupFirst{};
    /// Step 2: the new bytes into the primary.
    PutRecord primaryWrite{};
    /// Step 2's restore of the old bytes into the primary (attempted only when needed).
    PutRecord restore{};
    /// Step 3: the new bytes into the backup (attempted only when step 2 verified).
    PutRecord backupRefresh{};
    /// Why the outcome is Unconfirmed, or None.
    UnconfirmedWhy unconfirmed = UnconfirmedWhy::None;
    /// On (u3): the primary file did not read back as last seen.
    bool primaryDisagreed = false;
    /// On (u3): the backup did not read back as last seen.
    bool backupDisagreed = false;
    /// The file the next boot will use, MEASURED at step 4 — None when Unconfirmed (no claim).
    Which effective = Which::None;
    /// What step 4's read of the primary found (a record of the read, not a claim, when
    /// Unconfirmed).
    FileHolds primaryAfter = FileHolds::NotMeasured;
    /// What step 4's read of the backup found (likewise).
    FileHolds backupAfter = FileHolds::NotMeasured;
    /// The primary file's path as the store named it ("" if the card was never touched).
    char primaryPath[hal::kStorePathBytes] = "";
    /// The backup's path as the store named it ("" if the card was never touched).
    char backupPath[hal::kStorePathBytes] = "";

    /// Whether the save ever asked for the primary file's truncating open (step 2 or its restore)
    /// — the destructive moment (HA-166 (1)).
    [[nodiscard]] constexpr bool primaryOpenedForWriting() const noexcept {
        return primaryWrite.attempted || restore.attempted;
    }
};

namespace detail {

/// What the save last read of one file (header: "last seen").
struct LastSeen {
    /// NotOpened (initially, and after a refused read of a file not yet written), Opened, or
    /// Unknown (a refused read after a replace of it got past its open).
    enum class State : std::uint8_t { NotOpened, Opened, Unknown };
    State state = State::NotOpened;
    std::size_t length = 0;
    std::uint32_t crc = 0;
};

/// One read as the save judged it.
struct SaveRead {
    bool opened = false;
    std::size_t count = 0;
    bool accepted = false;
};

/// A replace got past its open: the file's old contents are gone and it now exists.
[[nodiscard]] constexpr bool pastOpen(hal::ReplaceStep s) noexcept {
    return s == hal::ReplaceStep::None || s == hal::ReplaceStep::Write
           || s == hal::ReplaceStep::Flush;
}

/// The whole save, as one object so its two buffers and its per-file state live in one frame.
template <std::size_t MaxBytes, typename Accept>
class SafeSave {
public:
    SafeSave(hal::IFileStore& store, const char* primary, const char* backup,
             std::span<const std::byte> bytes, Accept& accept, SaveReport& report)
        : store_{store}, bytes_{bytes}, accept_{accept}, report_{report} {
        names_[kP] = primary;
        names_[kK] = backup;
    }

    void run() {
        report_.length = bytes_.size();
        // 0. Refuse before the card is touched.
        if (bytes_.empty() || !accept_(bytes_, report_.why, sizeof report_.why)) {
            report_.refused = SaveRefusal::BytesRefused;
            report_.outcome = SaveResult::NotSaved;
            return;
        }
        report_.why[0] = '\0';
        if (!store_.cardPresent()) {
            report_.refused = SaveRefusal::NoCard;
            report_.outcome = SaveResult::NotSaved;
            return;
        }
        // 1. The starting card: both files, always — and P again when its first read is not
        //    accepted, because one refused, short or altered read is not a measurement (header).
        SaveRead p = readFile(kP);
        if (!p.accepted) {
            p = readFile(kP);
        }
        if (p.accepted) {
            keepAsOld(p.count);
        }
        const SaveRead k = readFile(kK);
        report_.before = chooseEffective(p.accepted, k.accepted);
        bool mayWritePrimary = true;
        if (report_.before == Which::Backup) {
            keepAsOld(k.count);  // the earlier record, for the words; P is NOT backed up from K
        } else if (report_.before == Which::Primary) {
            const bool backupIsOld = k.opened && scratchIs(oldView());
            if (!backupIsOld) {
                mayWritePrimary = put(kK, oldView(), report_.backupFirst);
            }
        }
        // 2. The record, then (if it failed) the restore.
        bool primaryHoldsNew = false;
        if (mayWritePrimary) {
            primaryHoldsNew = put(kP, bytes_, report_.primaryWrite);
            if (!primaryHoldsNew && report_.before == Which::Primary) {
                const bool unchanged = report_.primaryWrite.readOpened && scratchIs(oldView());
                if (!unchanged) {
                    (void)put(kP, oldView(), report_.restore);
                }
            }
        }
        // 3. Refresh the backup — only once P verifiably holds the new bytes.
        if (primaryHoldsNew) {
            (void)put(kK, bytes_, report_.backupRefresh);
        }
        // 4. Measure.
        measure();
    }

private:
    static constexpr int kP = 0;
    static constexpr int kK = 1;
    static constexpr std::size_t kBuffer = MaxBytes + kStoreReadSlackBytes;

    [[nodiscard]] std::span<const std::byte> oldView() const noexcept {
        return std::span<const std::byte>{old_.data(), oldLength_};
    }
    [[nodiscard]] std::span<const std::byte> scratchView(std::size_t n) const noexcept {
        return std::span<const std::byte>{scratch_.data(), n};
    }
    /// The last read opened and returned exactly `b`.
    [[nodiscard]] bool scratchIs(std::span<const std::byte> b) const noexcept {
        return lastRead_.opened && lastRead_.count == b.size()
               && (b.empty() || std::memcmp(scratch_.data(), b.data(), b.size()) == 0);
    }
    void keepAsOld(std::size_t n) noexcept {
        std::memcpy(old_.data(), scratch_.data(), n);
        oldLength_ = n;
    }
    void notePath(int f, const char (&path)[hal::kStorePathBytes]) noexcept {
        std::memcpy(f == kP ? report_.primaryPath : report_.backupPath, path, sizeof path);
    }

    /// Read file `f` into the scratch, update its last-seen state, and judge it.
    SaveRead readFile(int f) {
        const hal::StoreRead r = store_.read(names_[f], std::span<std::byte>{scratch_});
        notePath(f, r.path);
        SaveRead out{};
        out.opened = r.opened;
        out.count = countRead(r, scratch_.size());
        if (r.opened) {
            anyOpened_ = true;
            seen_[f] = LastSeen{LastSeen::State::Opened, out.count,
                                crc32Ieee(scratchView(out.count))};
        } else if (writtenPastOpen_[f]) {
            seen_[f] = LastSeen{LastSeen::State::Unknown, 0U, 0U};
        }
        out.accepted = out.opened && out.count > 0 && accept_(scratchView(out.count), nullptr, 0U);
        lastRead_ = out;
        return out;
    }

    /// put(X, b): replace, then ALWAYS read back; success only on the read-back (header).
    bool put(int f, std::span<const std::byte> b, PutRecord& rec) {
        rec.attempted = true;
        const hal::StoreReplace w = store_.replace(names_[f], b);
        notePath(f, w.path);
        rec.reported = w.failed;
        if (pastOpen(w.failed)) {
            writtenPastOpen_[f] = true;
            anyOpened_ = true;
        }
        const SaveRead back = readFile(f);
        rec.readOpened = back.opened;
        rec.readCount = back.count;
        rec.verified = back.accepted && scratchIs(b);
        return rec.verified;
    }

    /// Step 4's read of file `f`: compare with last-seen BEFORE it is updated, then classify.
    FileHolds measureOne(int f, bool& disagreed, bool& accepted) {
        const LastSeen before = seen_[f];
        const SaveRead r = readFile(f);
        switch (before.state) {
            case LastSeen::State::Unknown: disagreed = !r.opened; break;
            case LastSeen::State::NotOpened: disagreed = r.opened; break;
            case LastSeen::State::Opened:
                disagreed = !r.opened || r.count != before.length
                            || crc32Ieee(scratchView(r.count)) != before.crc;
                break;
        }
        accepted = r.accepted;
        if (!r.opened) {
            return FileHolds::Absent;
        }
        if (!r.accepted) {
            return FileHolds::NotAccepted;
        }
        if (scratchIs(bytes_)) {
            return FileHolds::New;
        }
        if (report_.before != Which::None && scratchIs(oldView())) {
            return FileHolds::Earlier;
        }
        return FileHolds::Other;
    }

    void measure() {
        bool primaryAccepted = false;
        bool backupAccepted = false;
        report_.primaryAfter = measureOne(kP, report_.primaryDisagreed, primaryAccepted);
        report_.backupAfter = measureOne(kK, report_.backupDisagreed, backupAccepted);
        if (!store_.cardPresent()) {
            report_.unconfirmed = UnconfirmedWhy::CardGone;
        } else if (!anyOpened_) {
            report_.unconfirmed = UnconfirmedWhy::NothingOpened;
        } else if (report_.primaryDisagreed || report_.backupDisagreed) {
            report_.unconfirmed = UnconfirmedWhy::ReadBackDiffers;
        }
        if (report_.unconfirmed != UnconfirmedWhy::None) {
            report_.outcome = SaveResult::Unconfirmed;
            report_.effective = Which::None;
            return;
        }
        report_.effective = chooseEffective(primaryAccepted, backupAccepted);
        report_.outcome = report_.effective == Which::Primary
                                  && report_.primaryAfter == FileHolds::New
                              ? SaveResult::Saved
                              : SaveResult::NotSaved;
    }

    hal::IFileStore& store_;
    const char* names_[2] = {nullptr, nullptr};
    std::span<const std::byte> bytes_;
    Accept& accept_;
    SaveReport& report_;
    std::array<std::byte, kBuffer> old_{};      // the record in effect before, for the restore
    std::size_t oldLength_ = 0;
    std::array<std::byte, kBuffer> scratch_{};  // every read lands here
    SaveRead lastRead_{};
    LastSeen seen_[2]{};
    bool writtenPastOpen_[2] = {false, false};
    bool anyOpened_ = false;
};

}  // namespace detail

/// THE SAFE SAVE, byte-level and generic (header: the design). Replaces `primary` with `bytes`,
/// designed so that a failure part-way leaves the card holding the file in effect before or the
/// new one — given HA-166 (4) and (5), and short of the two-fault limit (header: THE GUARANTEE) —
/// and returns what it MEASURED. `bytes` must be accepted and at most MaxBytes long (a longer one
/// is a precondition), and the two names must be two files on the card (hal::sameFileOnCard()).
/// Not noexcept: preconditions and the store's own throws propagate.
template <std::size_t MaxBytes, typename Accept>
[[nodiscard]] SaveReport safeReplace(hal::IFileStore& store, const char* primary,
                                     const char* backup, std::span<const std::byte> bytes,
                                     Accept&& accept) {
    static_assert(MaxBytes >= 1 && MaxBytes <= kMaxStoredFileBytes,
                  "safeReplace: MaxBytes must be 1..1024 -- two buffers of it live on the stack");
    SHULIB_PRECONDITION(primary != nullptr && backup != nullptr,
                        "safeReplace: a file name is null");
    SHULIB_PRECONDITION(!hal::sameFileOnCard(primary, backup),
                        "safeReplace: the primary and its backup must be two different files on "
                        "the card (FAT ignores letter case and trailing dots and spaces)");
    SHULIB_PRECONDITION(bytes.size() <= MaxBytes,
                        "safeReplace: the bytes are longer than MaxBytes");
    SaveReport report{};
    detail::SafeSave<MaxBytes, Accept> save{store, primary, backup, bytes, accept, report};
    save.run();
    return report;
}

// ═══ THE RECORD'S OWN ═══════════════════════════════════════════════════════════════

/// The calibration record's accept(): the bytes DECODE — magic, version, length (exactly
/// kRecordBytes), CRC-32 and valid() — with decodeRecord()'s refusal written into `why`.
[[nodiscard]] inline bool acceptRecord(std::span<const std::byte> bytes, char* why,
                                       std::size_t whyCap) noexcept {
    hal::DrivetrainDescription decoded{};
    return decodeRecord(bytes, decoded, why, whyCap);
}

/// Save a drivetrain description as the calibration record, SAFELY (safeReplace() over
/// kRecordFileName and kRecordBackupFileName). Refuses an invalid description, or one that will
/// not encode, BEFORE touching the card (the report's `refused` and `why` say which).
[[nodiscard]] inline SaveReport saveRecordSafely(hal::IFileStore& store,
                                                 const hal::DrivetrainDescription& description) {
    SaveReport report{};
    if (!description.valid(report.why, sizeof report.why)) {
        report.refused = SaveRefusal::DescriptionInvalid;
        report.outcome = SaveResult::NotSaved;
        return report;
    }
    std::byte encoded[kRecordBytes]{};
    const std::size_t n = encodeRecord(description, encoded);
    if (n != kRecordBytes) {
        std::snprintf(report.why, sizeof report.why, "%d of %d bytes encoded", static_cast<int>(n),
                      static_cast<int>(kRecordBytes));
        report.refused = SaveRefusal::EncodeFailed;
        report.outcome = SaveResult::NotSaved;
        return report;
    }
    return safeReplace<kRecordBytes>(store, kRecordFileName, kRecordBackupFileName,
                                     std::span<const std::byte>{encoded, n}, acceptRecord);
}

/// One record file, as the load saw it: FileRead with `accepted` read as "decoded".
struct RecordFileRead {
    /// The load read this file (the backup only when the record file did not decode).
    bool tried = false;
    /// False: no card, or no such file.
    bool opened = false;
    /// Bytes read; 0 on an open but EMPTY file, and 0 when the read did not open.
    std::size_t count = 0;
    /// decodeRecord() accepted it.
    bool decoded = false;
    /// decodeRecord()'s refusal, when the file opened, held bytes and did not decode; "" otherwise.
    char why[kFileReasonBytes] = "";
    /// The path the store's read used (the adapter's path() on the brain), for messages only.
    char path[hal::kStorePathBytes] = "";
};

/// The calibration record as the load found it: which file, the description, and each file's read
/// — enough for a boot to say, per file, "could not be opened", "is EMPTY" or the decoder's reason.
struct LoadedRecord {
    /// Primary, Backup or None.
    Which from = Which::None;
    /// The decoded description; meaningful unless `from` is None.
    hal::DrivetrainDescription description{};
    /// The record file (kRecordFileName), always read.
    RecordFileRead primary{};
    /// The backup (kRecordBackupFileName), read only when the record file did not decode.
    RecordFileRead backup{};
};

namespace detail {
inline void copyRecordFileRead(RecordFileRead& to, const FileRead& from) noexcept {
    to.tried = from.tried;
    to.opened = from.opened;
    to.count = from.count;
    to.decoded = from.accepted;
    std::memcpy(to.why, from.why, sizeof to.why);
    std::memcpy(to.path, from.path, sizeof to.path);
}
}  // namespace detail

/// Load the calibration record: the record file if it decodes, else its backup if THAT decodes,
/// else none (loadWithBackup() over acceptRecord). The backup is not read when the record file
/// decodes. Not noexcept: the store's own throws propagate.
[[nodiscard]] inline LoadedRecord loadRecord(hal::IFileStore& store) {
    const BackupLoad<kRecordBytes> load =
        loadWithBackup<kRecordBytes>(store, kRecordFileName, kRecordBackupFileName, acceptRecord);
    LoadedRecord out{};
    detail::copyRecordFileRead(out.primary, load.primary);
    detail::copyRecordFileRead(out.backup, load.backup);
    out.from = load.from;
    if (out.from != Which::None
        && !decodeRecord(std::span<const std::byte>{load.bytes.data(), load.count},
                         out.description)) {
        out.from = Which::None;  // unreachable: accept() IS this decode; kept so it cannot lie
    }
    return out;
}

// ═══ THE WORDS ══════════════════════════════════════════════════════════════════════

/// Whether the boot of the driver programs this Calibrate ships with ("shulib Drive" and the
/// competition program since T4 Part 3; "shulib Teleop" before it) reads the BACKUP when the
/// record file does not decode. The save's words depend on it; no default, so
/// every caller states it.
enum class BootReadsBackup : std::uint8_t {
    No,   ///< the boot reads the record file only: a damaged one REFUSES
    Yes,  ///< the boot falls back to a decodable backup (in amber)
};

/// Longest line formatSaveReport() emits, terminator included — exactly Calibrate's emitf buffer,
/// so a line passed through emitf("%s") is never cut. Every format below is budgeted against it
/// with each bounded string at its full precision (what -Wformat-truncation assumes).
inline constexpr std::size_t kSaveReportLineBytes = 240;

namespace detail {

/// The file's path from the report, or its bare name when the card was never touched.
[[nodiscard]] inline const char* pathOr(const char* path, const char* bare) noexcept {
    return path[0] != '\0' ? path : bare;
}

/// The step a replace reported failing, as a noun for "(The card reported the <noun> of ...)".
[[nodiscard]] inline const char* stepNoun(hal::ReplaceStep s) noexcept {
    switch (s) {
        case hal::ReplaceStep::Open: return "open";
        case hal::ReplaceStep::Write: return "write";
        case hal::ReplaceStep::Flush: return "flush";
        case hal::ReplaceStep::NoCard:
        case hal::ReplaceStep::None: break;
    }
    return "replace";
}

/// What a file holds, in words (at most 55 characters).
[[nodiscard]] inline const char* holdsWords(FileHolds h) noexcept {
    switch (h) {
        case FileHolds::New: return "the new record from this run";
        case FileHolds::Earlier: return "the record in effect before this run (an older record)";
        case FileHolds::Other: return "an older record";
        case FileHolds::NotAccepted: return "a damaged file that does not decode";
        case FileHolds::Absent: return "nothing (the file is missing)";
        case FileHolds::NotMeasured: break;
    }
    return "something this run did not measure";
}

/// Why a put did not verify, as a clause (at most 45 characters).
inline void putFailureClause(char (&out)[kSaveReportLineBytes], const PutRecord& put,
                             std::size_t length) noexcept {
    if (put.reported == hal::ReplaceStep::NoCard) {
        std::snprintf(out, sizeof out, "the card was gone");
    } else if (put.reported == hal::ReplaceStep::Open) {
        std::snprintf(out, sizeof out, "the card refused to open it for writing");
    } else if (!put.readOpened) {
        std::snprintf(out, sizeof out, "it would not re-open for the read-back");
    } else if (put.readCount != length) {
        std::snprintf(out, sizeof out, "it read back %d of %d bytes",
                      static_cast<int>(put.readCount), static_cast<int>(length));
    } else {
        std::snprintf(out, sizeof out, "it read back different bytes");
    }
}

}  // namespace detail

/// The serial lines for a save of the DRIVETRAIN record, one `emit(const char*)` per line: ASCII,
/// no '%', each under kSaveReportLineBytes. EVERY sentence is derived from the measured report —
/// "is still on the card" only when the final read found the earlier record in the record file
/// and the next boot uses it (untouched, or written back and re-read). `bootReadsBackup` says
/// whether the boot this build ships with falls back to the backup. The formatter is TOTAL: a
/// measured combination with no named branch prints the measurement itself (which file the next
/// boot uses, and what it holds). The words name the drivetrain record, "shulib Calibrate" and "the
/// next boot" — never a drive program by name: more than one program boots from the record
/// ("shulib Drive", the competition program and the Auton Tester since T4 Part 3, which retired the
/// "shulib Teleop" these words were first written beside). A sibling file words its own lines from
/// the same SaveReport.
template <typename Emit>
void formatSaveReport(const SaveReport& r, BootReadsBackup bootReadsBackup, Emit&& emit) {
    char line[kSaveReportLineBytes];
    const char* p = detail::pathOr(r.primaryPath, kRecordFileName);
    const char* k = detail::pathOr(r.backupPath, kRecordBackupFileName);

    // ── Refused before the card was touched ────────────────────────────────────────
    if (r.refused != SaveRefusal::None) {
        switch (r.refused) {
            case SaveRefusal::NoCard: emit("NOT SAVED -- no SD card in the brain."); break;
            case SaveRefusal::DescriptionInvalid:
                std::snprintf(line, sizeof line,
                              "NOT SAVED -- the measured description is not valid: %.150s", r.why);
                emit(line);
                break;
            case SaveRefusal::EncodeFailed:
                std::snprintf(line, sizeof line, "NOT SAVED -- the record would not encode: %.150s",
                              r.why);
                emit(line);
                break;
            case SaveRefusal::BytesRefused:
            case SaveRefusal::None:
                std::snprintf(line, sizeof line,
                              "NOT SAVED -- the bytes to save do not decode: %.150s", r.why);
                emit(line);
                break;
        }
        emit("Nothing on the card was opened or written by this run: whatever it held, it still "
             "holds.");
        return;
    }

    if (r.outcome == SaveResult::Unconfirmed) {
        // ── Unconfirmed: say why, and claim nothing more ────────────────────────────
        if (r.unconfirmed == UnconfirmedWhy::CardGone) {
            emit("SAVE UNCONFIRMED -- the SD card is no longer in the brain.");
        } else if (r.unconfirmed == UnconfirmedWhy::NothingOpened) {
            emit("SAVE UNCONFIRMED -- the card is in the brain, but nothing on it could be opened "
                 "or created (write-protected, full, or failing).");
        } else {
            const char* which = r.primaryDisagreed && r.backupDisagreed ? "both files"
                                : r.primaryDisagreed                    ? p
                                                                        : k;
            std::snprintf(line, sizeof line,
                          "SAVE UNCONFIRMED -- the card did not read back the same way twice "
                          "(%.95s): pulled, or failing.",
                          which);
            emit(line);
        }
        emit("This run cannot say which record the next boot will use. Put the card back if it "
             "was pulled, copy both files");
        emit("off it on a laptop, then start the drive program and read its 'calibration record:' "
             "line.");
    } else if (r.outcome == SaveResult::Saved) {
        // ── Saved: the record file holds the new bytes, measured ────────────────────
        std::snprintf(line, sizeof line,
                      "SAVED -- %.95s: written, RE-READ and compared byte for byte (%d bytes), "
                      "and re-read at the end.",
                      p, static_cast<int>(r.length));
        emit(line);
        if (r.backupAfter == FileHolds::New) {
            std::snprintf(line, sizeof line,
                          "The backup %.95s re-reads identical. The next boot reads the record "
                          "file.",
                          k);
            emit(line);
        } else {
            emit("The next boot reads the record file.");
            std::snprintf(line, sizeof line,
                          "The backup %.95s does NOT hold this record -- it holds %s. The next "
                          "save refreshes it.",
                          k, detail::holdsWords(r.backupAfter));
            emit(line);
        }
    } else {
        // ── Not saved: why, then what the card measurably holds ─────────────────────
        char clause[kSaveReportLineBytes];
        if (r.backupFirst.attempted && !r.backupFirst.verified) {
            detail::putFailureClause(clause, r.backupFirst, r.length);
            std::snprintf(line, sizeof line,
                          "NOT SAVED -- the backup copy of the earlier record did not stick in "
                          "%.95s (%.60s),",
                          k, clause);
            emit(line);
            emit("so the record file was never opened for writing.");
        } else if (r.primaryWrite.attempted && !r.primaryWrite.verified) {
            detail::putFailureClause(clause, r.primaryWrite, r.length);
            std::snprintf(line, sizeof line,
                          "NOT SAVED -- the new record did not stick in %.95s (%.60s).", p, clause);
            emit(line);
        } else {
            emit("NOT SAVED -- the card does not hold the new record where the next boot reads "
                 "it.");
        }
        const bool earlierInPrimary =
            r.effective == Which::Primary && r.primaryAfter == FileHolds::Earlier;
        const bool earlierInBackup =
            r.effective == Which::Backup && r.backupAfter == FileHolds::Earlier;
        if (earlierInPrimary && r.restore.attempted && r.restore.verified) {
            emit("The earlier record was written back to the record file and re-read: it is still "
                 "on the card, and the next boot uses it as before.");
        } else if (earlierInPrimary && !r.restore.attempted) {
            emit("The record file was not changed by this run: the earlier record is still on the "
                 "card, and the next boot uses it as before.");
        } else if (earlierInBackup && r.before == Which::Primary) {
            std::snprintf(line, sizeof line,
                          "The record file %.95s is damaged; the earlier record is safe in the "
                          "backup.",
                          p);
            emit(line);
        } else if (earlierInBackup && r.before == Which::Backup) {
            // What step 1 READ, not what the card held: a record file misread twice at the start
            // reads exactly like a damaged one (header: THE LIMIT).
            std::snprintf(line, sizeof line,
                          "The record file did not read as usable when this run began; the backup "
                          "copy %.95s holds an earlier record.",
                          k);
            emit(line);
        } else if (r.effective == Which::None) {
            emit("Nothing usable is on the card (neither file decodes): the next boot will REFUSE "
                 "until a run saves.");
        } else {
            // TOTAL: a combination with no named branch prints the measurement itself.
            std::snprintf(line, sizeof line,
                          "Measured at the end of this run: the next boot's rule picks %s, which "
                          "holds %s.",
                          r.effective == Which::Primary ? "the record file" : "the backup",
                          detail::holdsWords(r.effective == Which::Primary ? r.primaryAfter
                                                                           : r.backupAfter));
            emit(line);
        }
        if (r.effective == Which::Backup) {
            if (bootReadsBackup == BootReadsBackup::Yes) {
                emit("The next boot uses the BACKUP copy (amber on the brain's screen). Rerun "
                     "shulib Calibrate to save a new record.");
            } else {
                std::snprintf(line, sizeof line,
                              "The drive program in this build does not read the backup: the "
                              "next boot will REFUSE. Rerun shulib Calibrate, or on a laptop copy "
                              "%s over %s.",
                              kRecordBackupFileName, kRecordFileName);
                emit(line);
            }
        }
    }

    // ── What the card REPORTED along the way, judged against the final read ────────────
    const PutRecord* puts[4] = {&r.backupFirst, &r.primaryWrite, &r.restore, &r.backupRefresh};
    const FileHolds wrote[4] = {FileHolds::Earlier, FileHolds::New, FileHolds::Earlier,
                                FileHolds::New};
    const bool toPrimary[4] = {false, true, true, false};
    for (int i = 0; i < 4; ++i) {
        const PutRecord& put = *puts[i];
        if (!put.attempted || put.reported == hal::ReplaceStep::None) {
            continue;
        }
        const char* file = toPrimary[i] ? p : k;
        if (put.reported == hal::ReplaceStep::NoCard) {
            std::snprintf(line, sizeof line, "(The card was gone when %.95s was to be written.)",
                          file);
            emit(line);
            continue;
        }
        const FileHolds after = toPrimary[i] ? r.primaryAfter : r.backupAfter;
        const bool whole = r.outcome != SaveResult::Unconfirmed && after == wrote[i];
        std::snprintf(line, sizeof line, "(The card reported the %s of %.95s failing%s)",
                      detail::stepNoun(put.reported), file,
                      whole ? ", yet it reads back whole." : ".");
        emit(line);
    }
}

}  // namespace shulib::calibration
