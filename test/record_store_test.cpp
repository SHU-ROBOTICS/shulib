// record_store_test.cpp — SAFE-SAVE Part A: the calibration record's safe save does not leave the
// card without a good record — on a card that behaves as HA-166 (4) and (5) assume (the FakeFileStore
// does), and short of the two-fault limit the header states — the load falls back to a verified
// backup, and the report says only what it MEASURED (include/shulib/calibration/record_store.hpp).
//
// HOW THESE TESTS TRY TO BREAK IT
//   * THIRTEEN STARTING CARDS, each with its record in effect before the save written here as a
//     LITERAL (which file, and which fixture's bytes) — never computed by the code under test.
//   * THE CRASH-POINT TEST (the load-bearing one): the operation log of a clean save is recorded,
//     then the save is crashed at EVERY index of it, leaving the file being replaced in EVERY state
//     (absent, empty, each prefix, all new, each torn split), from every starting card, with the
//     card then reported GONE and, separately, still reported PRESENT but silent. After a "reboot"
//     the load must return, byte for byte, the record in effect before or the new one; the report
//     made while the card was pulled must be Unconfirmed — for the reason the log shows — or name
//     exactly what the reboot loads.
//   * ONE FAULT PER RUN: every reported and silent fault the fake models, at every index, from every
//     card, with the Saved / NotSaved / Unconfirmed oracles.
//   * SILENT WRITE, THEN A CRASH; A MISREAD, THEN A CRASH; A MISREAD, THEN A WRITE FAULT: the pairs.
//     A read fault the checks can see, followed by a crash or a write fault, is covered since the
//     record file is re-read when its first read is not accepted (review of 2026-10-01). What stays
//     EXCLUDED is the stated limit (header: THE LIMIT; HA-139): a misread at step 1 the checks cannot
//     catch — the record file misread at BOTH of its first reads, or a read that returns a valid
//     record the file does not hold (a stale read) — then a second failure. A test that it is
//     defended could not pass; the two STATED LIMIT tests pin what it costs instead.
//   * THE ORDER, read from the operation log: the save begins by reading P (twice when the first
//     read did not decode) and then K; every replace is read back before anything else is replaced;
//     the record file is never opened for writing before the backup verifiably holds the old
//     record; the backup is never written from a record that did not decode; no restore when the
//     record file already reads back as the old one.
//   * Directed cases for every named branch, the edges, the preconditions, and the words: a golden
//     for every branch of formatSaveReport() under both BootReadsBackup values — from a real save
//     where one reaches it, and from a synthetic report for the three no real save reaches (the
//     TOTAL line, "does not hold the new record where the next boot reads it", "the card was
//     gone") — and an enumeration of measured combinations in which no line may claim what its
//     report does not say.
//
// FIXTURE: robot two's description is COPIED from test/boot_source_test.cpp :112-146 (not moved:
// that file is edited by chunks in flight — SAFE-SAVE landmine 12). It was rebuilt there from the
// 2026-09-25 Teleop capture's printed values; its provenance carries the Calibrate BUILD stamp
// "Sep 24 2026 17:42:16", not the time the record was written (which run wrote it is not provable
// from the tree). These are NOT the card's own bytes.
//
// NO HARDWARE CLAIM: every card here is a FakeFileStore. What the brain's SD card does is HA-166.

#include "doctest.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "shulib/calibration/calibration_record.hpp"
#include "shulib/calibration/record_store.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/fake/fake_file_store.hpp"
#include "shulib/hal/file_store.hpp"
#include "shulib/units/quantity.hpp"

namespace {

using shulib::PreconditionError;
using shulib::calibration::acceptRecord;
using shulib::calibration::BackupLoad;
using shulib::calibration::BootReadsBackup;
using shulib::calibration::chooseEffective;
using shulib::calibration::decodeRecord;
using shulib::calibration::encodeRecord;
using shulib::calibration::FileHolds;
using shulib::calibration::formatSaveReport;
using shulib::calibration::kRecordBackupFileName;
using shulib::calibration::kRecordBytes;
using shulib::calibration::kRecordFileName;
using shulib::calibration::kSaveReportLineBytes;
using shulib::calibration::loadRecord;
using shulib::calibration::loadWithBackup;
using shulib::calibration::PutRecord;
using shulib::calibration::safeReplace;
using shulib::calibration::saveRecordSafely;
using shulib::calibration::SaveRefusal;
using shulib::calibration::SaveReport;
using shulib::calibration::SaveResult;
using shulib::calibration::UnconfirmedWhy;
using shulib::calibration::Which;
using shulib::hal::DescriptionSource;
using shulib::hal::DrivetrainDescription;
using shulib::hal::DrivetrainKind;
using shulib::hal::MotorCartridge;
using shulib::hal::ReplaceStep;
using shulib::hal::fake::AfterCrash;
using shulib::hal::fake::CrashLeave;
using shulib::hal::fake::FakeFileFault;
using shulib::hal::fake::FakeFileFaultPlan;
using shulib::hal::fake::FakeFileLogEntry;
using shulib::hal::fake::FakeFileOp;
using shulib::hal::fake::FakeFileStore;
using shulib::units::Length;

using Bytes = std::vector<std::byte>;

// ── robot two's calibration record — COPIED from test/boot_source_test.cpp :112-146 ─────────
constexpr std::int8_t kRecordLeft[4] = {-16, 17, -18, -20};
constexpr std::int8_t kRecordRight[4] = {11, -12, 13, 15};
constexpr double kLeftIpr = 1.25442;
constexpr double kRightIpr = 1.36703;
constexpr double kTrackWidth = 12.006;
constexpr const char* kProvenance =
    "shulib Calibrate Sep 24 2026 17:42:16 | ROBOT-MEASURED: 8 ports, push 48 in, W 12.006 in";

DrivetrainDescription robotTwoRecord() {
    DrivetrainDescription d{};
    d.kind = DrivetrainKind::Tank;
    d.cartridge = MotorCartridge::Blue;
    d.imuPort = 2;
    for (std::size_t i = 0; i < 4; ++i) {
        d.left.signedPorts[i] = kRecordLeft[i];
        d.right.signedPorts[i] = kRecordRight[i];
    }
    d.left.count = 4;
    d.right.count = 4;
    d.left.inchesPerRadian = Length{kLeftIpr};
    d.right.inchesPerRadian = Length{kRightIpr};
    d.trackWidth = Length{kTrackWidth};
    d.source = DescriptionSource::RobotMeasured;
    std::snprintf(d.provenance, sizeof d.provenance, "%s", kProvenance);
    return d;
}
// ── end of the copy ─────────────────────────────────────────────────────────────────────────

/// The "older good" record: robot two's with a different track width (and so a different CRC).
DrivetrainDescription olderRecord() {
    DrivetrainDescription d = robotTwoRecord();
    d.trackWidth = Length{11.5};
    return d;
}

/// The record this run saves: what a careful rerun might measure (the scale ~8 % larger).
DrivetrainDescription newDescription() {
    DrivetrainDescription d = robotTwoRecord();
    d.left.inchesPerRadian = Length{1.3551};
    d.right.inchesPerRadian = Length{1.4760};
    d.trackWidth = Length{12.412};
    std::snprintf(d.provenance, sizeof d.provenance, "%s",
                  "shulib Calibrate rerun | ROBOT-MEASURED: 8 ports, push 48 in, W 12.412 in");
    return d;
}

Bytes encoded(const DrivetrainDescription& d) {
    Bytes b(kRecordBytes);
    REQUIRE(encodeRecord(d, b) == kRecordBytes);
    return b;
}

const Bytes& kGood() {
    static const Bytes b = encoded(robotTwoRecord());
    return b;
}
const Bytes& kOlder() {
    static const Bytes b = encoded(olderRecord());
    return b;
}
const Bytes& kNew() {
    static const Bytes b = encoded(newDescription());
    return b;
}

Bytes prefixOf(const Bytes& b, std::size_t n) { return {b.begin(), b.begin() + static_cast<std::ptrdiff_t>(n)}; }

Bytes withByteFlipped(Bytes b, std::size_t at) {
    b[at] ^= std::byte{0x01};
    return b;
}

/// A TORN record: `front`'s bytes up to `at`, `back`'s after — 180 bytes whose CRC cannot match.
Bytes torn(const Bytes& front, const Bytes& back, std::size_t at) {
    Bytes b = prefixOf(front, at);
    b.insert(b.end(), back.begin() + static_cast<std::ptrdiff_t>(at), back.end());
    return b;
}

/// The test's OWN notion of "decodes" — decodeRecord directly, never the store's accept.
bool decodes(const Bytes& b) {
    DrivetrainDescription d{};
    return !b.empty() && decodeRecord(b, d);
}

std::string whyNot(const Bytes& b) {
    DrivetrainDescription d{};
    char why[192] = "";
    (void)decodeRecord(b, d, why, sizeof why);
    return why;
}

// ── THE THIRTEEN STARTING CARDS, each with its LITERAL record in effect before ───────────────
struct StartCard {
    std::string label;              // a string: doctest prints a char* as an address
    std::optional<Bytes> primary;   // the record file (P); nullopt = absent
    std::optional<Bytes> backup;    // the backup (K); nullopt = absent
    Which before;                   // LITERAL: which file is in effect before the save
    const Bytes* beforeBytes;       // LITERAL: the fixture it holds (nullptr = none)
};

const std::vector<StartCard>& cards() {
    static const std::vector<StartCard> all = [] {
        const Bytes shortRecord = prefixOf(kGood(), 100);
        const Bytes badCrc = withByteFlipped(kGood(), 100);           // a provenance byte changed
        const Bytes badCrcOlder = withByteFlipped(kOlder(), 177);     // a CRC byte changed
        const Bytes tornRecord = torn(kNew(), kGood(), 90);           // new bytes to 90, old after
        Bytes goodPlus8 = kGood();
        goodPlus8.insert(goodPlus8.end(), 8, std::byte{0x00});
        return std::vector<StartCard>{
            {"1: P absent, K absent", std::nullopt, std::nullopt, Which::None, nullptr},
            {"2: P absent, K good", std::nullopt, kGood(), Which::Backup, &kGood()},
            {"3: P good, K absent", kGood(), std::nullopt, Which::Primary, &kGood()},
            {"4: P good, K equal to P", kGood(), kGood(), Which::Primary, &kGood()},
            {"5: P good, K older good", kGood(), kOlder(), Which::Primary, &kGood()},
            {"6: P good, K short (100 bytes)", kGood(), shortRecord, Which::Primary, &kGood()},
            {"7: P good, K 180 bytes bad CRC", kGood(), badCrc, Which::Primary, &kGood()},
            {"8: P short (100 bytes), K good", shortRecord, kGood(), Which::Backup, &kGood()},
            {"9: P torn (180 bytes, bad CRC), K good", tornRecord, kGood(), Which::Backup, &kGood()},
            {"10: P empty, K good", Bytes{}, kGood(), Which::Backup, &kGood()},
            {"11: P good + 8 bytes, K good", goodPlus8, kGood(), Which::Backup, &kGood()},
            {"12: P short, K short", shortRecord, prefixOf(kOlder(), 100), Which::None, nullptr},
            {"13: P bad CRC, K bad CRC", badCrc, badCrcOlder, Which::None, nullptr},
        };
    }();
    return all;
}

FakeFileStore cardStore(const StartCard& c) {
    FakeFileStore s{"/usd/"};
    if (c.primary.has_value()) s.setFile(kRecordFileName, *c.primary);
    if (c.backup.has_value()) s.setFile(kRecordBackupFileName, *c.backup);
    return s;
}

// ── what a rebooted robot loads ─────────────────────────────────────────────────────────────
struct Boot {
    Which from = Which::None;
    Bytes bytes;
    bool readBackup = false;
};

Boot rebootAndLoad(FakeFileStore& s) {
    s.reboot();
    const BackupLoad<kRecordBytes> l =
        loadWithBackup<kRecordBytes>(s, kRecordFileName, kRecordBackupFileName, acceptRecord);
    Boot b{};
    b.from = l.from;
    b.bytes.assign(l.bytes.begin(), l.bytes.begin() + static_cast<std::ptrdiff_t>(l.count));
    for (const FakeFileLogEntry& e : s.log()) {
        if (e.file == kRecordBackupFileName) b.readBackup = true;
    }
    return b;
}

/// The test's OWN load rule over the card's files, to cross-check loadWithBackup.
Which independentRule(const FakeFileStore& s) {
    const auto p = s.file(kRecordFileName);
    if (p.has_value() && decodes(*p)) return Which::Primary;
    const auto k = s.file(kRecordBackupFileName);
    if (k.has_value() && decodes(*k)) return Which::Backup;
    return Which::None;
}

const char* whichName(Which w) {
    return w == Which::Primary ? "P" : w == Which::Backup ? "K" : "none";
}

// ── THE ORDER, from the operation log (the save's own view: what its reads returned) ────────
/// Index of step 1's read of K: 1, or 2 when P's first read did not decode (the re-read).
std::size_t stepOneKIndex(const std::vector<FakeFileLogEntry>& log) {
    return !log.empty() && log[0].opened && decodes(log[0].bytes) ? 1U : 2U;
}

std::string orderViolation(const std::vector<FakeFileLogEntry>& log, const Bytes& fresh) {
    if (log.empty()) return "";
    const auto readOf = [&](std::size_t i, const char* file) {
        return i < log.size() && log[i].op == FakeFileOp::Read && log[i].file == file;
    };
    if (!readOf(0, kRecordFileName)) return "the save did not begin by reading P";
    const std::size_t kAt = stepOneKIndex(log);
    if (kAt == 2 && !readOf(1, kRecordFileName)) {
        return "P's first read did not decode and the save did not read P again";
    }
    if (!readOf(kAt, kRecordBackupFileName)) {
        return "the save did not read K right after P (P read again only when it did not decode)";
    }
    const FakeFileLogEntry& pRead = log[kAt - 1];
    const bool pAccepted = pRead.opened && decodes(pRead.bytes);
    const Bytes old = pAccepted ? pRead.bytes : Bytes{};
    const bool kEqualAtStart = pAccepted && log[kAt].opened && log[kAt].bytes == old;
    bool kHoldsOld = kEqualAtStart;
    bool pVerifiedNew = false;
    int primaryReplaces = 0;
    std::optional<FakeFileLogEntry> lastPReadBack;
    for (std::size_t i = 0; i < log.size(); ++i) {
        const FakeFileLogEntry& e = log[i];
        const bool isP = e.file == kRecordFileName;
        if (e.op == FakeFileOp::Replace) {
            if (i + 1 >= log.size() || log[i + 1].op != FakeFileOp::Read || log[i + 1].file != e.file) {
                return "op " + std::to_string(i) + ": a replace of " + e.file
                       + " was not read back before anything else";
            }
            if (isP) {
                if (pAccepted && !kHoldsOld) {
                    return "op " + std::to_string(i)
                           + ": P opened for writing before K verifiably held the old record";
                }
                if (primaryReplaces == 1) {  // the restore
                    if (!pAccepted || e.bytes != old) {
                        return "op " + std::to_string(i) + ": a second replace of P that is not a restore of old";
                    }
                    if (lastPReadBack.has_value() && lastPReadBack->opened && lastPReadBack->bytes == old) {
                        return "op " + std::to_string(i) + ": a restore although P already read back as old";
                    }
                } else if (primaryReplaces > 1) {
                    return "op " + std::to_string(i) + ": P replaced three times";
                }
                ++primaryReplaces;
            } else {
                if (!pVerifiedNew) {
                    if (!pAccepted) {
                        return "op " + std::to_string(i)
                               + ": K written before P held the new bytes, in a save whose P did not decode";
                    }
                    if (e.bytes != old) {
                        return "op " + std::to_string(i) + ": K written with something other than old before P held the new bytes";
                    }
                    if (kEqualAtStart) {
                        return "op " + std::to_string(i) + ": K written in step 1 although it already equalled P";
                    }
                } else if (e.bytes != fresh) {
                    return "op " + std::to_string(i) + ": step 3 wrote K with something other than the new bytes";
                }
                kHoldsOld = false;
            }
        } else if (i > kAt) {
            if (!isP) {
                kHoldsOld = pAccepted && e.opened && e.bytes == old;
            } else if (log[i - 1].op == FakeFileOp::Replace && log[i - 1].file == kRecordFileName) {
                lastPReadBack = e;
                if (log[i - 1].bytes == fresh && e.opened && e.bytes == fresh) pVerifiedNew = true;
            }
        }
    }
    return "";
}

// ── THE INVARIANT and the outcome oracles ───────────────────────────────────────────────────
enum class RunKind { Clean, Crash, OneFault, Pair };

struct RunFacts {
    RunKind kind = RunKind::Clean;
    bool cardGoneCrash = false;  // a crash happened and the card was then reported gone
    bool faultOnRead = false;    // the one injected fault hit a read
    bool stepOneTouched = false; // a fault hit a read step 1 DECIDED from (set by runOnce)
};

/// Whether a fault touched a read step 1 decides from — its LAST read of P (the re-read, when
/// there was one) or its read of K — read from the run's own log, so it follows the sequence the
/// save actually ran. A fault on a first read of P that the re-read then saw past does not count:
/// the save must still name the literal record in effect before.
bool stepOneFaulted(const std::vector<FakeFileLogEntry>& log) {
    std::size_t k = 0;
    while (k < log.size() && log[k].op == FakeFileOp::Read && log[k].file == kRecordFileName) ++k;
    if (k == 0 || k >= log.size()) return true;  // no whole step 1 (the order check reports it)
    return log[k - 1].faulted || log[k].faulted;
}

std::string violation(const StartCard& c, const SaveReport& rep, FakeFileStore& s,
                      const RunFacts& f) {
    const Bytes& fresh = kNew();
    const std::string order = orderViolation(s.log(), fresh);
    if (!order.empty()) return "ORDER: " + order;
    // What the card answered, from the log, BEFORE the reboot clears it: (u2) is "no read opened
    // and no replace got past its open" — the oracle for the Unconfirmed reason below.
    const bool anyOpened = std::any_of(s.log().begin(), s.log().end(),
                                       [](const FakeFileLogEntry& e) { return e.opened; });
    const bool presentAtEnd = s.cardPresent();
    const bool savedTouchesCard = rep.refused == SaveRefusal::None;
    if (!savedTouchesCard) return "the save refused before the card";
    if (!f.stepOneTouched && rep.before != c.before) {
        return std::string{"report.before is "} + whichName(rep.before) + ", the literal is " + whichName(c.before);
    }
    const Boot boot = rebootAndLoad(s);
    if (independentRule(s) != boot.from) return "loadWithBackup disagrees with the test's own rule";
    if (boot.from == Which::Primary && boot.readBackup) return "the boot read K although P decoded";
    // THE INVARIANT: the record in effect before, or the new one — never nothing, never a mixture.
    if (boot.from == Which::None) {
        if (c.before != Which::None) return "a record was in effect before and the reboot loads NOTHING";
    } else if (boot.bytes != fresh && (c.beforeBytes == nullptr || boot.bytes != *c.beforeBytes)) {
        return "the reboot loads a record that is neither the earlier one nor the new one";
    }
    if (f.cardGoneCrash
        && (rep.outcome != SaveResult::Unconfirmed || rep.unconfirmed != UnconfirmedWhy::CardGone)) {
        return "the card was reported gone and the report is not Unconfirmed (u1)";
    }
    if (rep.outcome == SaveResult::Unconfirmed) {
        if (rep.effective != Which::None) return "Unconfirmed yet it names an effective file";
        if (f.kind == RunKind::OneFault && !f.faultOnRead
            && rep.unconfirmed != UnconfirmedWhy::NothingOpened) {
            return "Unconfirmed from a WRITE fault other than (u2)";
        }
        if (f.kind == RunKind::Clean) return "a clean save is Unconfirmed";
        // THE REASON, from the log: with the card present at the end, (u2) exactly when nothing
        // opened, else (u3) — never (u2)'s "nothing could be opened or created" about a card that
        // opened something, and never (u3) about one that opened nothing.
        if (!presentAtEnd) {
            if (rep.unconfirmed != UnconfirmedWhy::CardGone) return "card gone, reason not (u1)";
        } else if (anyOpened) {
            if (rep.unconfirmed != UnconfirmedWhy::ReadBackDiffers) {
                return "Unconfirmed with the card present and something opened, reason not (u3)";
            }
            if (!rep.primaryDisagreed && !rep.backupDisagreed) return "(u3) names no file";
        } else if (rep.unconfirmed != UnconfirmedWhy::NothingOpened) {
            return "Unconfirmed with the card present and nothing opened, reason not (u2)";
        }
        return "";
    }
    // Not Unconfirmed: the report names EXACTLY what the reboot loads.
    if (rep.effective != boot.from) {
        return std::string{"report names "} + whichName(rep.effective) + ", the reboot loads " + whichName(boot.from);
    }
    if (boot.from != Which::None) {
        const FileHolds holds = boot.from == Which::Primary ? rep.primaryAfter : rep.backupAfter;
        if (holds == FileHolds::New && boot.bytes != fresh) return "report says New, the bytes are not";
        if (holds == FileHolds::Earlier
            && (c.beforeBytes == nullptr || boot.bytes != *c.beforeBytes)) {
            return "report says Earlier, the bytes are not the literal earlier record";
        }
        if (holds != FileHolds::New && holds != FileHolds::Earlier) {
            return "the effective file is reported as neither New nor Earlier";
        }
    }
    const auto p = s.file(kRecordFileName);
    const auto k = s.file(kRecordBackupFileName);
    if (rep.outcome == SaveResult::Saved) {
        if (boot.from != Which::Primary || boot.bytes != fresh) return "Saved, and the reboot does not load P = new";
        const bool kSame = k.has_value() && p.has_value() && *k == *p;
        if ((rep.backupAfter == FileHolds::New) != kSame) return "'the backup holds the same bytes' is not what K re-reads";
    } else {
        if (c.before == Which::None) {
            if (boot.from != Which::None) return "NotSaved from an empty card, and the reboot loads something";
        } else if (boot.bytes != *c.beforeBytes) {
            return "NotSaved, and the reboot does not load the literal earlier record";
        }
    }
    if (f.kind == RunKind::Clean) {
        if (rep.outcome != SaveResult::Saved) return "a clean save is not Saved";
        if (!p.has_value() || *p != fresh || !k.has_value() || *k != fresh || rep.backupAfter != FileHolds::New) {
            return "a clean save did not leave BOTH files holding the new record";
        }
    }
    return "";
}

/// Run one save from a card with the given plans; return the violation text ("" = holds).
struct RunOutcome {
    std::string failure;
    SaveReport report;
    std::size_t operations = 0;
};

RunOutcome runOnce(const StartCard& c, const std::vector<FakeFileFaultPlan>& plans, RunFacts facts) {
    FakeFileStore s = cardStore(c);
    for (const FakeFileFaultPlan& p : plans) s.plan(p);
    RunOutcome out{};
    out.report = saveRecordSafely(s, newDescription());
    out.operations = s.operations();
    if (facts.kind == RunKind::Crash || facts.kind == RunKind::Pair) {
        facts.cardGoneCrash = s.crashed() && !s.cardPresent();
    }
    facts.stepOneTouched = stepOneFaulted(s.log());
    out.failure = violation(c, out.report, s, facts);
    return out;
}

std::vector<FakeFileLogEntry> cleanLog(const StartCard& c) {
    FakeFileStore s = cardStore(c);
    (void)saveRecordSafely(s, newDescription());
    return s.log();
}

/// Every leave-as state for a crashed replace of an n-byte file: absent, empty, each prefix
/// (1..n-1), all new, each torn split (1..n-1). `sample` narrows the two sweeps to `js`.
std::vector<FakeFileFaultPlan> crashStates(std::size_t at, std::size_t n, AfterCrash after,
                                           const std::vector<std::size_t>* js) {
    std::vector<FakeFileFaultPlan> v;
    v.push_back({at, FakeFileFault::Crash, 0, CrashLeave::Absent, after});
    v.push_back({at, FakeFileFault::Crash, 0, CrashLeave::Empty, after});
    v.push_back({at, FakeFileFault::Crash, 0, CrashLeave::AllNew, after});
    std::vector<std::size_t> all;
    if (js == nullptr) {
        for (std::size_t j = 1; j < n; ++j) all.push_back(j);
        js = &all;
    }
    for (const std::size_t j : *js) {
        v.push_back({at, FakeFileFault::Crash, j, CrashLeave::Prefix, after});
        v.push_back({at, FakeFileFault::Crash, j, CrashLeave::Torn, after});
    }
    return v;
}

std::string joined(const std::vector<std::string>& lines) {
    std::string s;
    for (const std::string& l : lines) s += l + "\n";
    return s;
}

std::vector<std::string> words(const SaveReport& r, BootReadsBackup b) {
    std::vector<std::string> lines;
    formatSaveReport(r, b, [&](const char* l) { lines.emplace_back(l); });
    return lines;
}

std::size_t firstReplaceOf(const std::vector<FakeFileLogEntry>& log, const char* file,
                           std::size_t nth = 0) {
    for (std::size_t i = 0; i < log.size(); ++i) {
        if (log[i].op == FakeFileOp::Replace && log[i].file == file) {
            if (nth == 0) return i;
            --nth;
        }
    }
    FAIL("no such replace in the clean log");
    return 0;
}

}  // namespace

// ═══ FIXTURES ════════════════════════════════════════════════════════════════════════════════

TEST_CASE("SAFE-SAVE A: the fixtures are what the table says -- good ones decode, damaged ones are "
          "refused for the reason named") {
    // BUG CAUGHT: a damaged fixture that happens to decode (or a good one that does not) would turn
    // every test below into a test of the wrong card.
    CHECK(decodes(kGood()));
    CHECK(decodes(kOlder()));
    CHECK(decodes(kNew()));
    CHECK(kGood() != kOlder());
    CHECK(kGood() != kNew());
    CHECK(kOlder() != kNew());
    for (const StartCard& c : cards()) {
        CAPTURE(c.label);
        const bool pGood = c.primary.has_value() && decodes(*c.primary);
        const bool kGoodCard = c.backup.has_value() && decodes(*c.backup);
        CHECK(chooseEffective(pGood, kGoodCard) == c.before);
    }
    CHECK(whyNot(prefixOf(kGood(), 100)).find("100 bytes") != std::string::npos);
    CHECK(whyNot(withByteFlipped(kGood(), 100)).find("CRC-32 mismatch") != std::string::npos);
    CHECK(whyNot(torn(kNew(), kGood(), 90)).find("CRC-32 mismatch") != std::string::npos);
    CHECK(whyNot(withByteFlipped(kOlder(), 177)).find("CRC-32 mismatch") != std::string::npos);
    Bytes plus8 = kGood();
    plus8.insert(plus8.end(), 8, std::byte{0});
    CHECK(whyNot(plus8).find("188 bytes") != std::string::npos);
    // The new record differs from the good one from byte 33 on (the left scale): a torn split at or
    // below 33 is the good record itself, which is why the torn sweep covers every j.
    std::size_t firstDiff = 0;
    while (kNew()[firstDiff] == kGood()[firstDiff]) ++firstDiff;
    CHECK(firstDiff == 33);
}

TEST_CASE("SAFE-SAVE A: chooseEffective is P, else K, else none -- THE load rule") {
    // BUG CAUGHT: mutation 6 (prefer K when both are accepted) — a boot from an older backup while
    // a good record sits in the record file.
    static_assert(chooseEffective(true, true) == Which::Primary);
    static_assert(chooseEffective(true, false) == Which::Primary);
    static_assert(chooseEffective(false, true) == Which::Backup);
    static_assert(chooseEffective(false, false) == Which::None);
    CHECK(chooseEffective(true, true) == Which::Primary);
}

TEST_CASE("SAFE-SAVE A: the backup's name sits beside the record's, bare, and a different file ON "
          "THE CARD") {
    // BUG CAUGHT: a second spelling of either name (a program reading a file another never wrote),
    // a pre-prefixed name, or a backup that IS the record file — on a FAT card that includes a name
    // differing only in letter case or trailing dots and spaces (review of 2026-10-01): the save
    // would read the "backup" back as identical, skip it, and truncate the only copy.
    using shulib::hal::sameFileOnCard;
    CHECK(std::string{kRecordFileName} == "shulib_drivetrain.bin");
    CHECK(std::string{kRecordBackupFileName} == "shulib_drivetrain.bak");
    CHECK(std::strchr(kRecordBackupFileName, '/') == nullptr);
    static_assert(shulib::calibration::detail::isBareFileName(kRecordBackupFileName));
    static_assert(!shulib::calibration::detail::isBareFileName("/usd/x.bak"));
    static_assert(!shulib::calibration::detail::isBareFileName("usd/x.bak"));
    static_assert(!shulib::calibration::detail::isBareFileName(""));
    static_assert(!sameFileOnCard(kRecordFileName, kRecordBackupFileName));
    static_assert(sameFileOnCard("a.bin", "a.bin"));
    static_assert(sameFileOnCard("gains.bin", "GAINS.BIN"));
    static_assert(sameFileOnCard("Gains.Bin", "gAINS.bIN"));
    static_assert(sameFileOnCard("gains.bin", "gains.bin."));
    static_assert(sameFileOnCard("gains.bin", "gains.bin "));
    static_assert(sameFileOnCard("gains.bin. .", "GAINS.BIN"));
    static_assert(!sameFileOnCard("a.bin", "a.bak"));
    static_assert(!sameFileOnCard("a.bin", "a.bin2"));
    static_assert(!sameFileOnCard("a.bin2", "a.bin"));
    static_assert(!sameFileOnCard("a.bin", ".a.bin"));      // a LEADING dot is part of the name
    // Only the letters fold: '@'/'`' and '['/'{' differ by 32 too, and a "c | 0x20" fold would
    // merge them.
    static_assert(!sameFileOnCard("@.bin", "`.bin"));
    static_assert(!sameFileOnCard("[.bin", "{.bin"));
    static_assert(!sameFileOnCard("a.bin", nullptr));
    static_assert(sameFileOnCard(nullptr, nullptr));
}

// ═══ THE CLEAN SAVE, from every card ═════════════════════════════════════════════════════════

TEST_CASE("SAFE-SAVE A: a clean save from every starting card is SAVED, leaves both files holding the "
          "new record, and names the literal record in effect before") {
    // BUG CAUGHT: mutation 5 (step 3 dropped: the backup keeps the old record), mutation 6 (card 5's
    // literal: P was in effect), mutation 17 (card 2's K must be seen as the earlier record), and
    // any order violation on the happy path.
    for (const StartCard& c : cards()) {
        CAPTURE(c.label);
        const RunOutcome r = runOnce(c, {}, RunFacts{RunKind::Clean});
        CHECK_MESSAGE(r.failure.empty(), r.failure);
        CHECK(r.report.outcome == SaveResult::Saved);
        CHECK(r.report.before == c.before);
        CHECK(std::string{r.report.primaryPath} == "/usd/shulib_drivetrain.bin");
        CHECK(std::string{r.report.backupPath} == "/usd/shulib_drivetrain.bak");
    }
}

TEST_CASE("SAFE-SAVE A: the clean save's operation sequence, card by card, is exactly the design's") {
    // BUG CAUGHT: an extra or missing step that the invariants alone might tolerate — a backup
    // written when it already equals P (card 4), the backup-first step skipped (card 3/5/6/7), K
    // written in step 1 from a card whose P does not decode (cards 8-13), and P's re-read missing
    // where P does not decode (cards 1, 2, 8-13) or present where it does (cards 3-7).
    const auto sequence = [](const StartCard& c) {
        std::string s;
        for (const FakeFileLogEntry& e : cleanLog(c)) {
            s += e.op == FakeFileOp::Read ? "r" : "w";
            s += e.file == kRecordFileName ? "P " : "K ";
        }
        return s;
    };
    const std::string backupFirst = "rP rK wK rK wP rP wK rK rP rK ";
    const std::string noBackupFirst = "rP rK wP rP wK rK rP rK ";
    const std::string pReRead = "rP rP rK wP rP wK rK rP rK ";
    for (const StartCard& c : cards()) {
        CAPTURE(c.label);
        const bool pDecodes = c.primary.has_value() && decodes(*c.primary);
        const bool kEqualsP = pDecodes && c.backup.has_value() && *c.backup == *c.primary;
        CHECK(sequence(c)
              == (!pDecodes ? pReRead : !kEqualsP ? backupFirst : noBackupFirst));
    }
}

// ═══ THE CRASH-POINT TEST ════════════════════════════════════════════════════════════════════

TEST_CASE("SAFE-SAVE A: THE CRASH-POINT TEST -- a crash at every operation, in every leave-as state, "
          "from every card, card gone or silent: the reboot loads the earlier record or the new one, "
          "and the report is Unconfirmed or names exactly that") {
    // BUG CAUGHT: every ordering mistake the design exists to prevent — the record opened before
    // the backup verifiably holds it (mutation 1), a backup written from a record that did not
    // decode (3, 9), a refused read taken as "absent" (20), a report that outruns its measurement
    // (10, 15, 19, 21). Enumerated from the RECORDED clean log, so an operation added later is
    // covered because it exists.
    std::size_t runs = 0;
    std::size_t replaceIndices = 0;
    int failures = 0;
    for (const StartCard& c : cards()) {
        const std::vector<FakeFileLogEntry> log = cleanLog(c);
        for (std::size_t at = 0; at < log.size(); ++at) {
            const bool isReplace = log[at].op == FakeFileOp::Replace;
            replaceIndices += isReplace ? 1U : 0U;
            for (const AfterCrash after : {AfterCrash::CardGone, AfterCrash::CardSilent}) {
                const std::vector<FakeFileFaultPlan> states =
                    isReplace ? crashStates(at, log[at].bytes.size(), after, nullptr)
                              : std::vector<FakeFileFaultPlan>{
                                    {at, FakeFileFault::Crash, 0, CrashLeave::AllNew, after}};
                for (const FakeFileFaultPlan& plan : states) {
                    RunFacts facts{RunKind::Crash};
                    const RunOutcome r = runOnce(c, {plan}, facts);
                    ++runs;
                    if (!r.failure.empty() && failures < 8) {
                        ++failures;
                        FAIL_CHECK(c.label << " | crash at op " << at << " leave "
                                           << static_cast<int>(plan.leave) << " j " << plan.amount
                                           << std::string{after == AfterCrash::CardGone ? " GONE" : " SILENT"}
                                           << ": " << r.failure);
                    }
                }
            }
        }
    }
    MESSAGE("crash-point runs: " << runs << " (13 cards, " << replaceIndices
                                  << " replace indices x 361 states + every read index, x 2 variants)");
    CHECK(failures == 0);
    // Derived by hand (SAFE-SAVE-PROGRESS §3, and the review findings' section for the re-read):
    // 30 replace indices (4 backup-first cards x 3 + 9 others x 2) x 361 x 2 = 21660, plus the read
    // indices x 2: 4 x 7 + card 4's 6 + the eight re-reading cards' 7 each = 90 reads, 180 runs.
    CHECK(runs == 21840);
}

// ═══ ONE FAULT PER RUN ═══════════════════════════════════════════════════════════════════════

TEST_CASE("SAFE-SAVE A: ONE FAULT PER RUN -- every reported and silent fault at every operation from "
          "every card: Saved means P = new, NotSaved means the literal earlier record, Unconfirmed "
          "only from a read fault or (u2)") {
    // BUG CAUGHT: put() deciding from the step replace() reported (18); a write fault turned into
    // an Unconfirmed; a silent write that the read-back does not catch (11, via SilentNoop and the
    // order check); a restore that runs when P already reads back as old (16).
    std::size_t runs = 0;
    int failures = 0;
    for (const StartCard& c : cards()) {
        const std::vector<FakeFileLogEntry> log = cleanLog(c);
        for (std::size_t at = 0; at < log.size(); ++at) {
            std::vector<FakeFileFaultPlan> plans;
            const std::size_t n = log[at].bytes.size();
            if (log[at].op == FakeFileOp::Replace) {
                plans.push_back({at, FakeFileFault::OpenRefused});
                plans.push_back({at, FakeFileFault::FlushFalse});
                plans.push_back({at, FakeFileFault::SilentNoop});
                for (std::size_t j = 0; j < n; ++j) {
                    plans.push_back({at, FakeFileFault::ShortWrite, j});
                    plans.push_back({at, FakeFileFault::SilentCut, j});
                    plans.push_back({at, FakeFileFault::SilentFlip, j});
                }
            } else {
                plans.push_back({at, FakeFileFault::OpenRefused});
                const std::size_t held = std::max<std::size_t>(n, kRecordBytes);
                for (std::size_t j = 0; j < held; ++j) {
                    plans.push_back({at, FakeFileFault::ShortRead, j});
                    plans.push_back({at, FakeFileFault::SilentReadFlip, j});
                }
            }
            for (const FakeFileFaultPlan& plan : plans) {
                RunFacts facts{RunKind::OneFault};
                facts.faultOnRead = log[at].op == FakeFileOp::Read;
                const RunOutcome r = runOnce(c, {plan}, facts);
                ++runs;
                if (!r.failure.empty() && failures < 8) {
                    ++failures;
                    FAIL_CHECK(c.label << " | fault " << static_cast<int>(plan.fault) << " amount "
                                       << plan.amount << " at op " << at << ": " << r.failure);
                }
            }
        }
    }
    MESSAGE("one-fault runs: " << runs);
    CHECK(failures == 0);
    // Derived by hand: 45908 before the re-read (SAFE-SAVE-PROGRESS §3), plus the re-read on the
    // eight cards whose P does not decode — refused, and a short read / an altered byte at every j
    // below max(P's length, 180): 7 x 361 + card 11's 188-byte P, 377 = 2904. 45908 + 2904 = 48812.
    CHECK(runs == 48812);
}

// ═══ SILENT WRITE, THEN A CRASH ══════════════════════════════════════════════════════════════

TEST_CASE("SAFE-SAVE A: SILENT WRITE THEN A CRASH -- a silent write fault at each replace, then a crash "
          "at every later operation in every leave-as class: the invariant holds") {
    // BUG CAUGHT: mutation 2 — a backup whose read-back was skipped. A silent fault then leaves K
    // damaged while the save believes it holds the old record, P is opened, and a crash loses the
    // only good copy. (A READ fault followed by a crash is the next test's.)
    //
    // The crash sweeps EVERY leave-as state (every prefix and torn split). The SILENT fault's own
    // number is SAMPLED at the boundaries that change what the read-back sees — empty (0), one byte
    // (1), the first byte where the new record differs from the good one (33), mid-file (90), the
    // CRC (176) and the last byte (179), plus the no-op; every value of it is swept in the one-fault
    // test above, and the full product here would be ~2 million runs.
    std::size_t runs = 0;
    int failures = 0;
    for (const StartCard& c : cards()) {
        const std::vector<FakeFileLogEntry> log = cleanLog(c);
        for (std::size_t i = 0; i < log.size(); ++i) {
            if (log[i].op != FakeFileOp::Replace) continue;
            std::vector<FakeFileFaultPlan> silents;
            silents.push_back({i, FakeFileFault::SilentNoop});
            for (const std::size_t j : std::vector<std::size_t>{0, 1, 90, 176, 179}) {
                silents.push_back({i, FakeFileFault::SilentCut, j});
            }
            for (const std::size_t j : std::vector<std::size_t>{0, 33, 90, 176, 179}) {
                silents.push_back({i, FakeFileFault::SilentFlip, j});
            }
            for (const FakeFileFaultPlan& silent : silents) {
                // The faulted run's own log decides which later operations exist.
                FakeFileStore probe = cardStore(c);
                probe.plan(silent);
                (void)saveRecordSafely(probe, newDescription());
                const std::vector<FakeFileLogEntry> faulted = probe.log();
                for (std::size_t at = i + 1; at < faulted.size(); ++at) {
                    const bool isReplace = faulted[at].op == FakeFileOp::Replace;
                    for (const AfterCrash after : {AfterCrash::CardGone, AfterCrash::CardSilent}) {
                        const std::vector<FakeFileFaultPlan> states =
                            isReplace ? crashStates(at, faulted[at].bytes.size(), after, nullptr)
                                      : std::vector<FakeFileFaultPlan>{
                                            {at, FakeFileFault::Crash, 0, CrashLeave::AllNew, after}};
                        for (const FakeFileFaultPlan& crash : states) {
                            RunFacts facts{RunKind::Pair};
                            const RunOutcome r = runOnce(c, {silent, crash}, facts);
                            ++runs;
                            if (!r.failure.empty() && failures < 8) {
                                ++failures;
                                FAIL_CHECK(c.label << " | silent " << static_cast<int>(silent.fault)
                                                   << "/" << silent.amount << " at op " << i
                                                   << " then crash at op " << at << " leave "
                                                   << static_cast<int>(crash.leave) << " j "
                                                   << crash.amount << ": " << r.failure);
                            }
                        }
                    }
                }
            }
        }
    }
    MESSAGE("silent-write-then-crash runs: " << runs);
    CHECK(failures == 0);
    // Derived by hand (SAFE-SAVE-PROGRESS §3). The only later REPLACE is step 2's restore of P,
    // after a silent fault on step 2's write, on the five cards whose P was in effect (3, 4, 5, 6,
    // 7) and for the ten silent faults that change P (a SilentNoop leaves the old record: no
    // restore): 50 sites x 2 variants x 361 leave-as states = 36100. Every other later operation is
    // a READ, crashed once per variant: 2080 runs. 36100 + 2080 = 38180. (P's re-read on the cards
    // whose P does not decode comes before every replace, so it adds no later operation here.)
    CHECK(runs == 38180);
}

// ═══ A MISREAD, THEN A CRASH ═════════════════════════════════════════════════════════════════

namespace {

/// The five READ faults both misread enumerations put at read index `i` — each one the save's checks
/// can SEE: refused, empty, short (100), byte 0 altered (the magic), byte 90 altered (the body). (No
/// starting card's damage sits at byte 0 or 90, so an altered byte never turns a damaged file into a
/// valid record: that would be THE LIMIT's second kind, which the two STATED LIMIT tests measure.)
std::vector<FakeFileFaultPlan> visibleMisreadsAt(std::size_t i) {
    return {{i, FakeFileFault::OpenRefused},    {i, FakeFileFault::ShortRead, 0},
            {i, FakeFileFault::ShortRead, 100}, {i, FakeFileFault::SilentReadFlip, 0},
            {i, FakeFileFault::SilentReadFlip, 90}};
}

}  // namespace

TEST_CASE("SAFE-SAVE A: A MISREAD THEN A CRASH -- a read fault at each read, then a crash at every "
          "later operation: the invariant holds, the first read of a GOOD record file included") {
    // BUG CAUGHT: the review of 2026-10-01's two-fault case — robot two's card today (card 3: a good
    // record file, no backup yet), its record file's first read refused or short, so the save took
    // it for damaged, backed nothing up, truncated it, and a power cut lost the only record. P's
    // re-read is the fix; without it this test fails at read index 0 on cards 3-7. Also any read
    // fault that steers the save into writing before it has a verified copy.
    //
    // The READ fault is sampled by kind — refused, empty, short (100), byte 0 altered (the magic),
    // byte 90 altered (the body) — and the crash's leave-as state at the boundaries that change
    // what the reboot sees: absent, empty, all new, a prefix of 1, 33, 34, 90, 176 or 179 bytes, a
    // torn split at the same six (at 33 and below a torn record file still IS the old record; from
    // 34 it is a mixture). Every later operation is crashed, in both variants.
    // NOT covered, on purpose: the stated limit (header: THE LIMIT) — a misread the checks cannot
    // catch. The two STATED LIMIT tests after the directed ones measure it.
    const std::vector<std::size_t> js{1, 33, 34, 90, 176, 179};
    std::size_t runs = 0;
    int failures = 0;
    for (const StartCard& c : cards()) {
        const std::vector<FakeFileLogEntry> log = cleanLog(c);
        for (std::size_t i = 0; i < log.size(); ++i) {
            if (log[i].op != FakeFileOp::Read) continue;
            for (const FakeFileFaultPlan& misread : visibleMisreadsAt(i)) {
                FakeFileStore probe = cardStore(c);
                probe.plan(misread);
                (void)saveRecordSafely(probe, newDescription());
                const std::vector<FakeFileLogEntry> faulted = probe.log();
                for (std::size_t at = i + 1; at < faulted.size(); ++at) {
                    const bool isReplace = faulted[at].op == FakeFileOp::Replace;
                    for (const AfterCrash after : {AfterCrash::CardGone, AfterCrash::CardSilent}) {
                        const std::vector<FakeFileFaultPlan> states =
                            isReplace ? crashStates(at, faulted[at].bytes.size(), after, &js)
                                      : std::vector<FakeFileFaultPlan>{
                                            {at, FakeFileFault::Crash, 0, CrashLeave::AllNew, after}};
                        for (const FakeFileFaultPlan& crash : states) {
                            const RunOutcome r = runOnce(c, {misread, crash}, RunFacts{RunKind::Pair});
                            ++runs;
                            if (!r.failure.empty() && failures < 8) {
                                ++failures;
                                FAIL_CHECK(c.label << " | misread " << static_cast<int>(misread.fault)
                                                   << "/" << misread.amount << " at op " << i
                                                   << " then crash at op " << at << " leave "
                                                   << static_cast<int>(crash.leave) << " j "
                                                   << crash.amount << ": " << r.failure);
                            }
                        }
                    }
                }
            }
        }
    }
    MESSAGE("misread-then-crash runs: " << runs);
    CHECK(failures == 0);
    // Derived by hand (SAFE-SAVE-PROGRESS, "review findings": the three clean-log shapes, then per
    // read index which operations the faulted run still has): a later READ is crashed 2 ways, a
    // later REPLACE 2 x 15. Backup-first cards (3, 5, 6, 7): 1250 each; card 4: 1070; the eight
    // cards whose P does not decode: 1100 each. 4 x 1250 + 1070 + 8 x 1100 = 14870.
    CHECK(runs == 14870);
}

// ═══ A MISREAD, THEN A WRITE FAULT ═══════════════════════════════════════════════════════════

TEST_CASE("SAFE-SAVE A: A MISREAD THEN A WRITE FAULT -- a read fault at each read, then a reported or "
          "silent write fault at every later replace: the invariant holds") {
    // BUG CAUGHT: the review of 2026-10-01's two-fault case in its commonest form — no power cut,
    // only a card that refuses or cuts a write (a full card): the record file's first read refused
    // or short, then step 2's write falls short. Without P's re-read, card 3 loses robot two's only
    // record and card 5 boots the older backup. HA-166 says a single misread the checks can see,
    // followed by a crash OR a write fault, is defended: the previous test enumerates the crash,
    // this one the write fault (verification round, 2026-10-04 — the claim had been measured only by
    // a scratch probe).
    //
    // The READ fault is the previous test's five; the WRITE fault is every kind the fake models — the
    // open refused, a false flush, a write that silently never landed, and a short write, a silent
    // cut and a silent flip at 0, 1, 33, 34, 90, 176 and 179 (the crash sweep's boundaries, plus
    // empty) — at every later REPLACE of the faulted run. A later READ is not faulted here: two read
    // faults are not this pair. The stated limit (header: THE LIMIT) stays excluded.
    const std::vector<std::size_t> js{0, 1, 33, 34, 90, 176, 179};
    std::size_t runs = 0;
    int failures = 0;
    for (const StartCard& c : cards()) {
        const std::vector<FakeFileLogEntry> log = cleanLog(c);
        for (std::size_t i = 0; i < log.size(); ++i) {
            if (log[i].op != FakeFileOp::Read) continue;
            for (const FakeFileFaultPlan& misread : visibleMisreadsAt(i)) {
                FakeFileStore probe = cardStore(c);
                probe.plan(misread);
                (void)saveRecordSafely(probe, newDescription());
                const std::vector<FakeFileLogEntry> faulted = probe.log();
                for (std::size_t at = i + 1; at < faulted.size(); ++at) {
                    if (faulted[at].op != FakeFileOp::Replace) continue;
                    std::vector<FakeFileFaultPlan> writes = {{at, FakeFileFault::OpenRefused},
                                                             {at, FakeFileFault::FlushFalse},
                                                             {at, FakeFileFault::SilentNoop}};
                    for (const std::size_t j : js) {
                        writes.push_back({at, FakeFileFault::ShortWrite, j});
                        writes.push_back({at, FakeFileFault::SilentCut, j});
                        writes.push_back({at, FakeFileFault::SilentFlip, j});
                    }
                    for (const FakeFileFaultPlan& write : writes) {
                        const RunOutcome r = runOnce(c, {misread, write}, RunFacts{RunKind::Pair});
                        ++runs;
                        if (!r.failure.empty() && failures < 8) {
                            ++failures;
                            FAIL_CHECK(c.label << " | misread " << static_cast<int>(misread.fault)
                                               << "/" << misread.amount << " at op " << i
                                               << " then write fault " << static_cast<int>(write.fault)
                                               << "/" << write.amount << " at op " << at << ": "
                                               << r.failure);
                        }
                    }
                }
            }
        }
    }
    MESSAGE("misread-then-write-fault runs: " << runs);
    CHECK(failures == 0);
    // Derived by hand from the same per-read-index table as the previous test (SAFE-SAVE-PROGRESS,
    // "review findings"), counting only the later REPLACES of each faulted run, x 5 misreads x 24
    // write faults. Backup-first cards (3, 5, 6, 7): index 0 and 1 leave 3 replaces each, index 5 (step
    // 2's read-back: the restore) 1, the rest 0 — 7 x 5 x 24 = 840 each. Card 4: index 0 leaves 2,
    // index 1 (K now differs: the backup copy runs) 3, index 3 (the restore) 1 — 6 x 120 = 720. The
    // eight cards whose P does not decode: index 0, 1 and 2 leave 2 each — 6 x 120 = 720 each.
    // 4 x 840 + 720 + 8 x 720 = 9840.
    CHECK(runs == 9840);
}

// ═══ DIRECTED ════════════════════════════════════════════════════════════════════════════════

TEST_CASE("SAFE-SAVE A directed: good P, step 2's write cut short -> the old record is WRITTEN BACK "
          "to P and the reboot loads P, not the backup") {
    // BUG CAUGHT: mutation 4 (the restore dropped): the reboot would then load K, and the report
    // could not say "written back".
    for (const std::size_t card : {std::size_t{2}, std::size_t{4}}) {  // cards 3 and 5
        const StartCard& c = cards()[card];
        CAPTURE(c.label);
        const std::size_t step2 = firstReplaceOf(cleanLog(c), kRecordFileName);
        FakeFileStore s = cardStore(c);
        s.plan({step2, FakeFileFault::ShortWrite, 90});
        const SaveReport rep = saveRecordSafely(s, newDescription());
        CHECK(rep.outcome == SaveResult::NotSaved);
        CHECK(rep.restore.attempted);
        CHECK(rep.restore.verified);
        CHECK(rep.effective == Which::Primary);
        CHECK(rep.primaryAfter == FileHolds::Earlier);
        const std::string text = joined(words(rep, BootReadsBackup::No));
        CHECK(text.find("written back to the record file and re-read") != std::string::npos);
        const Boot boot = rebootAndLoad(s);
        CHECK(boot.from == Which::Primary);
        CHECK_FALSE(boot.readBackup);
        CHECK(boot.bytes == kGood());
    }
}

TEST_CASE("SAFE-SAVE A directed: good P, step 2's open refused -> NO second replace of P, and the "
          "report says the record file was not changed") {
    // BUG CAUGHT: mutation 16 — a restore that rewrites P although it already reads back as the old
    // record (a second destructive open for nothing).
    const StartCard& c = cards()[2];  // card 3
    const std::size_t step2 = firstReplaceOf(cleanLog(c), kRecordFileName);
    FakeFileStore s = cardStore(c);
    s.plan({step2, FakeFileFault::OpenRefused});
    const SaveReport rep = saveRecordSafely(s, newDescription());
    int primaryReplaces = 0;
    for (const FakeFileLogEntry& e : s.log()) {
        if (e.op == FakeFileOp::Replace && e.file == kRecordFileName) ++primaryReplaces;
    }
    CHECK(primaryReplaces == 1);
    CHECK_FALSE(rep.restore.attempted);
    CHECK(rep.outcome == SaveResult::NotSaved);
    CHECK(joined(words(rep, BootReadsBackup::No)).find("The record file was not changed by this run")
          != std::string::npos);
    // The truncating open WAS asked for (and refused): primaryOpenedForWriting() says "asked".
    CHECK(rep.primaryOpenedForWriting());
}

TEST_CASE("SAFE-SAVE A directed: primaryOpenedForWriting() -- false exactly when the save never asked "
          "for the record file's truncating open") {
    // BUG CAUGHT: review mutant N22 (the member returning a constant): the one public answer to
    // "was the destructive open ever requested", which B1's words may lean on.
    const StartCard& five = cards()[4];
    FakeFileStore failedCopy = cardStore(five);
    failedCopy.plan({firstReplaceOf(cleanLog(five), kRecordBackupFileName), FakeFileFault::ShortWrite,
                     50});
    const SaveReport stopped = saveRecordSafely(failedCopy, newDescription());
    CHECK(stopped.backupFirst.attempted);
    CHECK_FALSE(stopped.backupFirst.verified);
    CHECK_FALSE(stopped.primaryOpenedForWriting());  // the backup copy failed: P never opened
    for (const FakeFileLogEntry& e : failedCopy.log()) {
        CHECK_FALSE((e.op == FakeFileOp::Replace && e.file == kRecordFileName));
    }
    FakeFileStore clean = cardStore(five);
    CHECK(saveRecordSafely(clean, newDescription()).primaryOpenedForWriting());
    FakeFileStore none{"/usd/"};
    none.setCardPresent(false);
    CHECK_FALSE(saveRecordSafely(none, newDescription()).primaryOpenedForWriting());
}

TEST_CASE("SAFE-SAVE A directed: a FALSE FLUSH on step 2 that kept the bytes is SAVED, and the "
          "reported failure is printed on its own line (D8 (a))") {
    // BUG CAUGHT: mutation 18 — put() taking its success from the step replace() reported. SAVED is
    // a measurement; so is its absence.
    const StartCard& c = cards()[2];  // card 3
    const std::size_t step2 = firstReplaceOf(cleanLog(c), kRecordFileName);
    FakeFileStore s = cardStore(c);
    s.plan({step2, FakeFileFault::FlushFalse});
    const SaveReport rep = saveRecordSafely(s, newDescription());
    CHECK(rep.outcome == SaveResult::Saved);
    CHECK(rep.primaryWrite.reported == ReplaceStep::Flush);
    CHECK(rep.primaryWrite.verified);
    const std::vector<std::string> lines = words(rep, BootReadsBackup::No);
    CHECK(std::find(lines.begin(), lines.end(),
                    "(The card reported the flush of /usd/shulib_drivetrain.bin failing, yet it reads "
                    "back whole.)")
          != lines.end());
}

TEST_CASE("SAFE-SAVE A directed: card 2, the card PULLED at step 2 while the program runs -> "
          "Unconfirmed with (u1)'s words, never 'nothing usable'") {
    // BUG CAUGHT: mutation 21 (u1 removed: u3's words print instead), and the report that a pulled
    // card used to earn — "nothing is usable, Teleop will refuse" — which is false the moment the
    // card goes back in.
    const StartCard& c = cards()[1];
    const std::size_t step2 = firstReplaceOf(cleanLog(c), kRecordFileName);
    FakeFileStore s = cardStore(c);
    s.plan({step2, FakeFileFault::Crash, 90, CrashLeave::Prefix, AfterCrash::CardGone});
    const SaveReport rep = saveRecordSafely(s, newDescription());
    CHECK(rep.outcome == SaveResult::Unconfirmed);
    CHECK(rep.unconfirmed == UnconfirmedWhy::CardGone);
    const std::vector<std::string> lines = words(rep, BootReadsBackup::No);
    REQUIRE_FALSE(lines.empty());
    CHECK(lines[0] == "SAVE UNCONFIRMED -- the SD card is no longer in the brain.");
    CHECK(joined(lines).find("Nothing usable") == std::string::npos);
    CHECK(rebootAndLoad(s).bytes == kGood());
}

TEST_CASE("SAFE-SAVE A directed: card 5, the card stops answering at step 3 while still reported "
          "present -> Unconfirmed with (u3)'s words") {
    // BUG CAUGHT: mutation 15 — (u3) removed: the report would call the save measured on a card
    // that no longer reads back.
    const StartCard& c = cards()[4];
    const std::size_t step3 = firstReplaceOf(cleanLog(c), kRecordBackupFileName, 1);
    FakeFileStore s = cardStore(c);
    s.plan({step3, FakeFileFault::Crash, 0, CrashLeave::AllNew, AfterCrash::CardSilent});
    const SaveReport rep = saveRecordSafely(s, newDescription());
    CHECK(rep.outcome == SaveResult::Unconfirmed);
    CHECK(rep.unconfirmed == UnconfirmedWhy::ReadBackDiffers);
    const std::vector<std::string> lines = words(rep, BootReadsBackup::No);
    REQUIRE_FALSE(lines.empty());
    CHECK(lines[0] == "SAVE UNCONFIRMED -- the card did not read back the same way twice (both "
                      "files): pulled, or failing.");
}

TEST_CASE("SAFE-SAVE A directed: card 5, the card stops answering at the save's FIRST read while "
          "still reported present -> Unconfirmed with (u2)'s words, never 'nothing usable'") {
    // BUG CAUGHT: mutation 19 — (u2) removed: a card that answered nothing would be "measured" as
    // empty, and the report would say nothing usable is on it, with a good record sitting there.
    const StartCard& c = cards()[4];
    FakeFileStore s = cardStore(c);
    s.plan({0, FakeFileFault::Crash, 0, CrashLeave::AllNew, AfterCrash::CardSilent});
    const SaveReport rep = saveRecordSafely(s, newDescription());
    CHECK(rep.outcome == SaveResult::Unconfirmed);
    CHECK(rep.unconfirmed == UnconfirmedWhy::NothingOpened);
    const std::vector<std::string> lines = words(rep, BootReadsBackup::No);
    REQUIRE_FALSE(lines.empty());
    CHECK(lines[0] == "SAVE UNCONFIRMED -- the card is in the brain, but nothing on it could be "
                      "opened or created (write-protected, full, or failing).");
    CHECK(joined(lines).find("Nothing usable") == std::string::npos);
    CHECK(rebootAndLoad(s).bytes == kGood());
}

TEST_CASE("SAFE-SAVE A directed: card 1, the create of the record file refused -> Unconfirmed, (u2)") {
    // BUG CAUGHT: a card that is present but refuses every open, read as "nothing usable, Teleop
    // will refuse" — a claim about a card that was never measured.
    const StartCard& c = cards()[0];
    FakeFileStore s = cardStore(c);
    s.plan({firstReplaceOf(cleanLog(c), kRecordFileName), FakeFileFault::OpenRefused});
    const SaveReport rep = saveRecordSafely(s, newDescription());
    CHECK(rep.primaryWrite.reported == ReplaceStep::Open);
    CHECK(rep.outcome == SaveResult::Unconfirmed);
    CHECK(rep.unconfirmed == UnconfirmedWhy::NothingOpened);
}

TEST_CASE("SAFE-SAVE A directed: card 2, a short write at step 2 -> NOT SAVED, 'the record file did "
          "not read as usable; the backup holds an earlier record', naming the backup's bytes") {
    // BUG CAUGHT: mutation 17 — step 1 reading K only when P decodes: the save could not know K
    // held the earlier record, and the words (and the report's `before`) would be wrong.
    const StartCard& c = cards()[1];
    const std::size_t step2 = firstReplaceOf(cleanLog(c), kRecordFileName);
    FakeFileStore s = cardStore(c);
    s.plan({step2, FakeFileFault::ShortWrite, 90});
    const SaveReport rep = saveRecordSafely(s, newDescription());
    CHECK(rep.outcome == SaveResult::NotSaved);
    CHECK(rep.before == Which::Backup);
    CHECK(rep.effective == Which::Backup);
    CHECK(rep.backupAfter == FileHolds::Earlier);
    CHECK(joined(words(rep, BootReadsBackup::No))
              .find("The record file did not read as usable when this run began; the backup copy "
                    "/usd/shulib_drivetrain.bak holds an earlier record.")
          != std::string::npos);
    const Boot boot = rebootAndLoad(s);
    CHECK(boot.from == Which::Backup);
    CHECK(boot.bytes == kGood());
}

TEST_CASE("SAFE-SAVE A directed: a SILENT cut on step 3's write -> SAVED, and the backup is measured "
          "damaged") {
    // BUG CAUGHT: mutation 10 — a report built from the steps' own results instead of the final
    // read (it would say the backup holds the new record).
    const StartCard& c = cards()[2];
    const std::size_t step3 = firstReplaceOf(cleanLog(c), kRecordBackupFileName, 1);
    FakeFileStore s = cardStore(c);
    s.plan({step3, FakeFileFault::SilentCut, 90});
    const SaveReport rep = saveRecordSafely(s, newDescription());
    CHECK(rep.outcome == SaveResult::Saved);
    CHECK(rep.backupAfter == FileHolds::NotAccepted);
    CHECK(joined(words(rep, BootReadsBackup::No))
              .find("it holds a damaged file that does not decode") != std::string::npos);
}

TEST_CASE("SAFE-SAVE A directed: card 5, a SHORT READ on step 4's read of P, the card otherwise fine "
          "-> Unconfirmed (u3): the report never outruns its measurement") {
    // BUG CAUGHT: mutation 10 again — the general oracle alone would accept a Saved here (the reboot
    // DOES load the new record), so this pins that a measurement that disagrees is never papered
    // over.
    const StartCard& c = cards()[4];
    const std::vector<FakeFileLogEntry> log = cleanLog(c);
    const std::size_t step4p = log.size() - 2;
    REQUIRE(log[step4p].op == FakeFileOp::Read);
    REQUIRE(log[step4p].file == kRecordFileName);
    FakeFileStore s = cardStore(c);
    s.plan({step4p, FakeFileFault::ShortRead, 100});
    const SaveReport rep = saveRecordSafely(s, newDescription());
    CHECK(rep.outcome == SaveResult::Unconfirmed);
    CHECK(rep.unconfirmed == UnconfirmedWhy::ReadBackDiffers);
    CHECK(rep.primaryDisagreed);
    CHECK_FALSE(rep.backupDisagreed);
    const std::vector<std::string> lines = words(rep, BootReadsBackup::No);
    REQUIRE_FALSE(lines.empty());
    CHECK(lines[0] == "SAVE UNCONFIRMED -- the card did not read back the same way twice "
                      "(/usd/shulib_drivetrain.bin): pulled, or failing.");
}

TEST_CASE("SAFE-SAVE A directed: a write that SILENTLY never landed on step 2 -> the old record is "
          "still there, NOT SAVED, and the backup is never written with the new bytes") {
    // BUG CAUGHT: mutation 11 — step 2's put without its byte comparison. The old record DECODES,
    // so a put that only asks accept() would call it verified, and step 3 would then write the new
    // bytes into K while P holds the old ones (the order check's rule).
    const StartCard& c = cards()[2];
    const std::size_t step2 = firstReplaceOf(cleanLog(c), kRecordFileName);
    FakeFileStore s = cardStore(c);
    s.plan({step2, FakeFileFault::SilentNoop});
    const SaveReport rep = saveRecordSafely(s, newDescription());
    CHECK(orderViolation(s.log(), kNew()).empty());
    CHECK_FALSE(rep.primaryWrite.verified);
    CHECK_FALSE(rep.backupRefresh.attempted);
    CHECK(rep.outcome == SaveResult::NotSaved);
    CHECK(s.file(kRecordBackupFileName) == std::optional<Bytes>{kGood()});
}

TEST_CASE("SAFE-SAVE A directed: a create that reports success and leaves NO file (card 2, step 2) -> "
          "Unconfirmed, and the reboot still loads the backup") {
    // The one silent WRITE fault the model allows to end Unconfirmed, pinned so it stays a stated
    // exception rather than a surprise: the card says the open succeeded and then has no such file,
    // which contradicts "a file opened for writing exists" (nothing can delete it). The save cannot
    // tell a lost directory entry from a failing read, so it claims nothing — and the invariant
    // (the reboot loads the earlier record) holds regardless.
    const StartCard& c = cards()[1];
    const std::size_t step2 = firstReplaceOf(cleanLog(c), kRecordFileName);
    FakeFileStore s = cardStore(c);
    s.plan({step2, FakeFileFault::SilentNoop, 1});
    const SaveReport rep = saveRecordSafely(s, newDescription());
    CHECK(rep.outcome == SaveResult::Unconfirmed);
    CHECK(rep.unconfirmed == UnconfirmedWhy::ReadBackDiffers);
    CHECK(rep.primaryDisagreed);
    const Boot boot = rebootAndLoad(s);
    CHECK(boot.from == Which::Backup);
    CHECK(boot.bytes == kGood());
}

namespace {

/// The index of the n-th replace of `file` in the log of a save from `card` with `plans` — for a
/// directed second fault placed AFTER a first one that changed the sequence.
std::size_t replaceIndexAfter(const StartCard& c, const std::vector<FakeFileFaultPlan>& plans,
                              const char* file, std::size_t nth = 0) {
    FakeFileStore probe = cardStore(c);
    for (const FakeFileFaultPlan& p : plans) probe.plan(p);
    (void)saveRecordSafely(probe, newDescription());
    return firstReplaceOf(probe.log(), file, nth);
}

}  // namespace

TEST_CASE("SAFE-SAVE A directed: the record file's FIRST read refused or short, then step 2's write "
          "cut short (cards 3 and 5) -> P is read again, backed up, restored; the record survives") {
    // BUG CAUGHT: the two-fault case the review of 2026-10-01 measured on robot two's own card
    // shape (card 3: a good record file, no backup). Without the re-read, P counts as damaged, K
    // is never written, step 2 truncates P and the cut write leaves NOTHING usable (card 3) or the
    // OLDER backup (card 5). With it, the save backs P up and restores it.
    for (const std::size_t card : {std::size_t{2}, std::size_t{4}}) {
        for (const FakeFileFaultPlan& misread :
             {FakeFileFaultPlan{0, FakeFileFault::OpenRefused},
              FakeFileFaultPlan{0, FakeFileFault::ShortRead, 100}}) {
            const StartCard& c = cards()[card];
            CAPTURE(c.label);
            CAPTURE(static_cast<int>(misread.fault));
            const std::size_t step2 = replaceIndexAfter(c, {misread}, kRecordFileName);
            FakeFileStore s = cardStore(c);
            s.plan(misread);
            s.plan({step2, FakeFileFault::ShortWrite, 90});
            const SaveReport rep = saveRecordSafely(s, newDescription());
            REQUIRE(s.log().size() > 2);
            CHECK(s.log()[1].op == FakeFileOp::Read);
            CHECK(s.log()[1].file == kRecordFileName);  // the re-read
            CHECK(rep.before == Which::Primary);
            CHECK(rep.backupFirst.verified);
            CHECK(rep.restore.verified);
            CHECK(rep.outcome == SaveResult::NotSaved);
            CHECK(joined(words(rep, BootReadsBackup::No)).find("written back to the record file")
                  != std::string::npos);
            const Boot boot = rebootAndLoad(s);
            CHECK(boot.from == Which::Primary);
            CHECK(boot.bytes == kGood());
        }
    }
}

TEST_CASE("SAFE-SAVE A directed: card 1, the card stops answering right after the record file's "
          "CREATE got past its open (all bytes landed) -> Unconfirmed (u3), never (u2)'s words") {
    // BUG CAUGHT: review mutants N01/N02 — a replace past its open no longer counted as "opened"
    // (or Write dropped from "past its open"): card 1 is the one card where no READ ever opens, so
    // the reason flips to (u2), "nothing on it could be opened or created", about a card that did
    // create the file and holds the new record. The general oracle checks the same rule on every
    // crash and pair run.
    const StartCard& c = cards()[0];
    const std::size_t create = firstReplaceOf(cleanLog(c), kRecordFileName);
    FakeFileStore s = cardStore(c);
    s.plan({create, FakeFileFault::Crash, 0, CrashLeave::AllNew, AfterCrash::CardSilent});
    const SaveReport rep = saveRecordSafely(s, newDescription());
    CHECK(rep.outcome == SaveResult::Unconfirmed);
    CHECK(rep.unconfirmed == UnconfirmedWhy::ReadBackDiffers);
    CHECK(rep.primaryDisagreed);
    CHECK_FALSE(rep.backupDisagreed);
    const std::vector<std::string> lines = words(rep, BootReadsBackup::No);
    REQUIRE_FALSE(lines.empty());
    CHECK(lines[0] == "SAVE UNCONFIRMED -- the card did not read back the same way twice "
                      "(/usd/shulib_drivetrain.bin): pulled, or failing.");
    CHECK(rebootAndLoad(s).bytes == kNew());
}

TEST_CASE("SAFE-SAVE A directed: card 8, the backup's step-1 read refused, then step 2's open "
          "refused -> Unconfirmed (u3) naming the BACKUP only") {
    // BUG CAUGHT: review mutant N03 — (u3)'s "not opened, then opened" case set to never disagree:
    // the save would call the card measured, though the backup it never saw opened at the end. A
    // two-fault case, and the only one that reaches that branch.
    const StartCard& c = cards()[7];
    FakeFileStore s = cardStore(c);
    const std::size_t kRead = stepOneKIndex(cleanLog(c));
    const std::size_t step2 =
        replaceIndexAfter(c, {{kRead, FakeFileFault::OpenRefused}}, kRecordFileName);
    s.plan({kRead, FakeFileFault::OpenRefused});
    s.plan({step2, FakeFileFault::OpenRefused});
    const SaveReport rep = saveRecordSafely(s, newDescription());
    CHECK(rep.outcome == SaveResult::Unconfirmed);
    CHECK(rep.unconfirmed == UnconfirmedWhy::ReadBackDiffers);
    CHECK_FALSE(rep.primaryDisagreed);
    CHECK(rep.backupDisagreed);
    const std::vector<std::string> lines = words(rep, BootReadsBackup::No);
    REQUIRE_FALSE(lines.empty());
    CHECK(lines[0] == "SAVE UNCONFIRMED -- the card did not read back the same way twice "
                      "(/usd/shulib_drivetrain.bak): pulled, or failing.");
    CHECK(rebootAndLoad(s).bytes == kGood());
}

TEST_CASE("SAFE-SAVE A: THE STATED LIMIT (i), measured -- the record file misread at BOTH of its "
          "first reads, then step 2's write cut short: the record is lost (card 3) or the boot falls to "
          "the OLDER backup (card 5), and the words claim nothing the save did not read") {
    // Not a defence: the first kind of the limit the header and HA-166 state, pinned so it stays
    // stated. If either reboot ever loads robot two's record here, the limit has MOVED — update THE
    // LIMIT in record_store.hpp and HA-166's blast radius before changing this test. What it DOES
    // pin: even here, no line says the earlier record survived, and none says the card held no
    // usable record — only that the record file "did not read as usable when this run began".
    for (const std::size_t card : {std::size_t{2}, std::size_t{4}}) {
        const StartCard& c = cards()[card];
        CAPTURE(c.label);
        const std::vector<FakeFileFaultPlan> twice = {{0, FakeFileFault::OpenRefused},
                                                      {1, FakeFileFault::OpenRefused}};
        const std::size_t step2 = replaceIndexAfter(c, twice, kRecordFileName);
        FakeFileStore s = cardStore(c);
        for (const FakeFileFaultPlan& p : twice) s.plan(p);
        s.plan({step2, FakeFileFault::ShortWrite, 90});
        const SaveReport rep = saveRecordSafely(s, newDescription());
        CHECK(rep.outcome == SaveResult::NotSaved);
        CHECK_FALSE(rep.backupFirst.attempted);
        const std::string text = joined(words(rep, BootReadsBackup::Yes));
        CHECK(text.find("still on the card") == std::string::npos);
        CHECK(text.find("no usable record file") == std::string::npos);
        const Boot boot = rebootAndLoad(s);
        if (card == 2) {
            CHECK(boot.from == Which::None);
            CHECK(text.find("Nothing usable is on the card") != std::string::npos);
        } else {
            CHECK(boot.from == Which::Backup);
            CHECK(boot.bytes == kOlder());
            CHECK(text.find("The record file did not read as usable when this run began; the "
                            "backup copy /usd/shulib_drivetrain.bak holds an earlier record.")
                  != std::string::npos);
        }
    }
}

namespace {

/// A store whose read at ONE operation index hands back `bytes` as an open, whole read — a STALE read
/// (HA-139 is the belief that a read returns what the card holds). Every other call is the fake's,
/// and the fake counts and logs this read as its own (its log holds what the file really held).
class StaleReadStore final : public shulib::hal::IFileStore {
public:
    StaleReadStore(FakeFileStore& inner, std::size_t at, Bytes bytes)
        : inner_{inner}, at_{at}, bytes_{std::move(bytes)} {}
    [[nodiscard]] bool cardPresent() const override { return inner_.cardPresent(); }
    [[nodiscard]] shulib::hal::StoreRead read(const char* name, std::span<std::byte> into) override {
        const std::size_t op = inner_.operations();
        shulib::hal::StoreRead r = inner_.read(name, into);
        if (op == at_) {
            REQUIRE(bytes_.size() <= into.size());
            std::copy(bytes_.begin(), bytes_.end(), into.begin());
            r.opened = true;
            r.count = bytes_.size();
        }
        return r;
    }
    [[nodiscard]] shulib::hal::StoreReplace replace(const char* name,
                                                    std::span<const std::byte> bytes) override {
        return inner_.replace(name, bytes);
    }

private:
    FakeFileStore& inner_;
    std::size_t at_;
    Bytes bytes_;
};

}  // namespace

TEST_CASE("SAFE-SAVE A: THE STATED LIMIT (ii), measured -- a step-1 read that returns a VALID record "
          "the file does not hold (a stale read), then step 2's write fails: the next boot gets an "
          "OLDER record, or none, and the words can call the stale record the earlier one") {
    // Not a defence: the second kind of THE LIMIT (header; HA-166's blast radius), pinned so it stays
    // stated. No check the save makes can tell a stale copy from the file: the bytes ARE a valid
    // record. If any reboot here loads robot two's record, the limit has MOVED — update THE LIMIT in
    // record_store.hpp and HA-166 before changing this test. (Verification round, 2026-10-04: the
    // limit had named only a DAMAGED backup read as identical; an OLDER one — case 1, the verifier's
    // probe — costs the same.)
    struct Case {
        std::string label;
        std::size_t card;        // index into cards()
        std::size_t staleAt;     // the step-1 read that returns `stale`: 0 = P's first, 1 = K's
        const Bytes* stale;      // what that read returns
        bool crash;              // step 2's write: a crash (torn at 90, card gone), else short at 90
        Which bootFrom;          // what the reboot loads
        const Bytes* bootBytes;  // and its bytes (nullptr = nothing)
    };
    const std::vector<Case> cases = {
        {"1: card 5, K (OLDER, good) read as P's bytes; crash", 4, 1, &kGood(), true, Which::Backup,
         &kOlder()},
        {"2: card 7, K (bad CRC) read as P's bytes; crash", 6, 1, &kGood(), true, Which::None, nullptr},
        {"3: card 3, K (missing) read as P's bytes; crash", 2, 1, &kGood(), true, Which::None, nullptr},
        {"4: card 3, P (good) read as an OLDER record; short write", 2, 0, &kOlder(), false,
         Which::Primary, &kOlder()},
        {"5: card 8, P (short) read as an OLDER record, K good IN EFFECT; short write", 7, 0,
         &kOlder(), false, Which::Primary, &kOlder()},
    };
    for (const Case& t : cases) {
        CAPTURE(t.label);
        const StartCard& c = cards()[t.card];
        // Step 2's replace of P in the stale run (the stale read changes the sequence).
        std::size_t step2 = 0;
        {
            FakeFileStore probe = cardStore(c);
            StaleReadStore stale{probe, t.staleAt, *t.stale};
            (void)saveRecordSafely(stale, newDescription());
            step2 = firstReplaceOf(probe.log(), kRecordFileName);
        }
        FakeFileStore s = cardStore(c);
        if (t.crash) {
            s.plan({step2, FakeFileFault::Crash, 90, CrashLeave::Torn, AfterCrash::CardGone});
        } else {
            s.plan({step2, FakeFileFault::ShortWrite, 90});
        }
        StaleReadStore stale{s, t.staleAt, *t.stale};
        const SaveReport rep = saveRecordSafely(stale, newDescription());
        CHECK(rep.before == Which::Primary);  // what step 1 READ — never what the card held
        const std::string text = joined(words(rep, BootReadsBackup::Yes));
        const Boot boot = rebootAndLoad(s);
        CHECK(boot.from == t.bootFrom);
        if (t.bootBytes != nullptr) CHECK(boot.bytes == *t.bootBytes);
        CHECK(boot.bytes != kGood());  // robot two's record is gone: that is the limit
        if (t.crash) {
            CHECK(rep.outcome == SaveResult::Unconfirmed);  // the card was gone: nothing claimed
            CHECK(text.find("still on the card") == std::string::npos);
        } else {
            // The words rest on step 1's read: they call the stale OLDER record "the earlier record"
            // and say it survived — true of what step 1 read, false of the record in effect.
            CHECK(rep.outcome == SaveResult::NotSaved);
            CHECK(rep.restore.verified);
            CHECK(text.find("The earlier record was written back to the record file and re-read: it "
                            "is still on the card")
                  != std::string::npos);
        }
    }
}

// ═══ EDGES ═══════════════════════════════════════════════════════════════════════════════════

TEST_CASE("SAFE-SAVE A edge: no card -> nothing opened, NOT SAVED, and the words say so") {
    FakeFileStore s{"/usd/"};
    s.setFile(kRecordFileName, kGood());
    s.setCardPresent(false);
    const SaveReport rep = saveRecordSafely(s, newDescription());
    CHECK(s.log().empty());
    CHECK(rep.outcome == SaveResult::NotSaved);
    CHECK(rep.refused == SaveRefusal::NoCard);
    CHECK(rep.before == Which::None);
    CHECK(rep.primaryAfter == FileHolds::NotMeasured);
    CHECK(words(rep, BootReadsBackup::No)
          == std::vector<std::string>{
              "NOT SAVED -- no SD card in the brain.",
              "Nothing on the card was opened or written by this run: whatever it held, it still "
              "holds."});
}

TEST_CASE("SAFE-SAVE A edge: an invalid description, or bytes that do not decode -> nothing is "
          "opened") {
    // BUG CAUGHT: a save that opens (and so truncates) the record file for bytes nobody can boot
    // from. (EncodeFailed is unreachable from a valid description: encodeRecord fails only on a
    // short buffer, and the save's is exactly kRecordBytes. The generic path's twin — bytes accept()
    // refuses — is what is exercised.)
    FakeFileStore s{"/usd/"};
    s.setFile(kRecordFileName, kGood());
    DrivetrainDescription bad = newDescription();
    bad.cartridge = static_cast<MotorCartridge>(0);  // UNSET: valid() refuses it
    const SaveReport rep = saveRecordSafely(s, bad);
    CHECK(s.log().empty());
    CHECK(rep.refused == SaveRefusal::DescriptionInvalid);
    CHECK(std::string{rep.why}.find("cartridge") != std::string::npos);

    const Bytes damaged = withByteFlipped(kNew(), 100);
    const SaveReport rep2 = safeReplace<kRecordBytes>(s, kRecordFileName, kRecordBackupFileName,
                                                      damaged, acceptRecord);
    CHECK(s.log().empty());
    CHECK(rep2.refused == SaveRefusal::BytesRefused);
    CHECK(std::string{rep2.why}.find("CRC-32 mismatch") != std::string::npos);
    CHECK(s.file(kRecordFileName) == std::optional<Bytes>{kGood()});
}

TEST_CASE("SAFE-SAVE A edge: K LONGER than P but sharing its first 180 bytes -> K is rewritten") {
    // BUG CAUGHT: a "K equals P" check that compares a prefix, not the whole file.
    FakeFileStore s{"/usd/"};
    s.setFile(kRecordFileName, kGood());
    Bytes longer = kGood();
    longer.insert(longer.end(), 8, std::byte{0});
    s.setFile(kRecordBackupFileName, longer);
    (void)saveRecordSafely(s, newDescription());
    REQUIRE(s.log().size() > 2);
    CHECK(s.log()[2].op == FakeFileOp::Replace);
    CHECK(s.log()[2].file == kRecordBackupFileName);
    CHECK(s.log()[2].bytes == kGood());
}

TEST_CASE("SAFE-SAVE A edge: a K that differs from P ONLY in its CRC bytes -> K is rewritten") {
    // BUG CAUGHT: mutation 8 — the step-1 skip comparing length only.
    FakeFileStore s{"/usd/"};
    s.setFile(kRecordFileName, kGood());
    s.setFile(kRecordBackupFileName, withByteFlipped(kGood(), 178));
    (void)saveRecordSafely(s, newDescription());
    REQUIRE(s.log().size() > 2);
    CHECK(s.log()[2].op == FakeFileOp::Replace);
    CHECK(s.log()[2].file == kRecordBackupFileName);
}

TEST_CASE("SAFE-SAVE A edge: saving the record the card already holds -> SAVED, both files equal") {
    FakeFileStore s{"/usd/"};
    s.setFile(kRecordFileName, kNew());
    s.setFile(kRecordBackupFileName, kNew());
    const SaveReport rep = saveRecordSafely(s, newDescription());
    CHECK(rep.outcome == SaveResult::Saved);
    CHECK(rep.primaryAfter == FileHolds::New);
    CHECK(rep.backupAfter == FileHolds::New);
    CHECK_FALSE(rep.backupFirst.attempted);
}

TEST_CASE("SAFE-SAVE A edge: at boot, a record file that decodes -> the backup is never read") {
    // BUG CAUGHT: mutation 7 — loadWithBackup reading K when P is accepted.
    FakeFileStore s{"/usd/"};
    s.setFile(kRecordFileName, kGood());
    s.setFile(kRecordBackupFileName, kOlder());
    const auto rec = loadRecord(s);
    CHECK(rec.from == Which::Primary);
    CHECK_FALSE(rec.backup.tried);
    REQUIRE(s.log().size() == 1);
    CHECK(s.log()[0].file == kRecordFileName);
    CHECK(rec.description.trackWidth.value() == kTrackWidth);
}

TEST_CASE("SAFE-SAVE A edge: loadRecord falls back to a decodable backup and says why the record file "
          "did not decode, in the decoder's own words") {
    FakeFileStore s{"/usd/"};
    const Bytes tornRecord = torn(kNew(), kGood(), 90);
    s.setFile(kRecordFileName, tornRecord);
    s.setFile(kRecordBackupFileName, kOlder());
    const auto rec = loadRecord(s);
    CHECK(rec.from == Which::Backup);
    CHECK(rec.primary.tried);
    CHECK(rec.primary.opened);
    CHECK(rec.primary.count == kRecordBytes);
    CHECK_FALSE(rec.primary.decoded);
    CHECK(std::string{rec.primary.why} == whyNot(tornRecord));
    CHECK(rec.backup.decoded);
    CHECK(std::string{rec.backup.why}.empty());
    CHECK(std::string{rec.primary.path} == "/usd/shulib_drivetrain.bin");
    CHECK(std::string{rec.backup.path} == "/usd/shulib_drivetrain.bak");
    CHECK(rec.description.trackWidth.value() == 11.5);
}

TEST_CASE("SAFE-SAVE A edge: both damaged -> both reasons named, each the decoder's") {
    FakeFileStore s{"/usd/"};
    const Bytes shortP = prefixOf(kGood(), 100);
    const Bytes badK = withByteFlipped(kOlder(), 177);
    s.setFile(kRecordFileName, shortP);
    s.setFile(kRecordBackupFileName, badK);
    const auto rec = loadRecord(s);
    CHECK(rec.from == Which::None);
    CHECK(std::string{rec.primary.why} == whyNot(shortP));
    CHECK(std::string{rec.backup.why} == whyNot(badK));
    CHECK_FALSE(std::string{rec.primary.why}.empty());
    CHECK_FALSE(std::string{rec.backup.why}.empty());
}

TEST_CASE("SAFE-SAVE A edge: the boot's three primary-file facts -- not open, EMPTY, refused -- are all "
          "there for the boot's sentences, and an absent backup is 'not opened' with no reason") {
    // What B2 rebuilds its unchanged sentences from: opened / count / why / path.
    FakeFileStore none{"/usd/"};
    const auto a = loadRecord(none);
    CHECK_FALSE(a.primary.opened);
    CHECK(std::string{a.primary.why}.empty());
    CHECK(a.backup.tried);
    CHECK_FALSE(a.backup.opened);
    CHECK(std::string{a.backup.why}.empty());
    CHECK(std::string{a.primary.path} == "/usd/shulib_drivetrain.bin");

    FakeFileStore empty{"/usd/"};
    empty.setFile(kRecordFileName, Bytes{});
    const auto b = loadRecord(empty);
    CHECK(b.primary.opened);
    CHECK(b.primary.count == 0);
    CHECK_FALSE(b.primary.decoded);
    CHECK(std::string{b.primary.why}.empty());  // an EMPTY file is its own sentence, not accept()'s
}

TEST_CASE("SAFE-SAVE A edge: the preconditions -- null names, two names for ONE file on the card, "
          "bytes longer than MaxBytes -- throw before the card is touched, and the store's own throws "
          "propagate") {
    // BUG CAUGHT: a noexcept anywhere in the chain (a precondition would become std::terminate), a
    // save that opens the card before checking its own arguments, and (review of 2026-10-01) a
    // backup name that differs from the primary's only in letter case or a trailing dot or space:
    // one file on a FAT card, two on the host, so the host would pass a save that destroys it.
    FakeFileStore s{"/usd/"};
    const Bytes b = kNew();
    CHECK_THROWS_AS((void)safeReplace<kRecordBytes>(s, nullptr, "k", b, acceptRecord), PreconditionError);
    CHECK_THROWS_AS((void)safeReplace<kRecordBytes>(s, "p", "p", b, acceptRecord), PreconditionError);
    CHECK_THROWS_AS((void)safeReplace<100>(s, "p", "k", b, acceptRecord), PreconditionError);
    CHECK_THROWS_AS((void)loadWithBackup<kRecordBytes>(s, "p", "p", acceptRecord), PreconditionError);
    CHECK_THROWS_AS((void)loadWithBackup<kRecordBytes>(s, "p", nullptr, acceptRecord), PreconditionError);
    for (const char* alias : {"GAINS.BIN", "gains.bin.", "gains.bin ", "Gains.Bin.."}) {
        CAPTURE(std::string{alias});
        CHECK_THROWS_AS((void)safeReplace<kRecordBytes>(s, "gains.bin", alias, b, acceptRecord),
                        PreconditionError);
        CHECK_THROWS_AS((void)loadWithBackup<kRecordBytes>(s, alias, "gains.bin", acceptRecord),
                        PreconditionError);
    }
    CHECK(s.log().empty());
    CHECK_THROWS_AS((void)safeReplace<kRecordBytes>(s, "/usd/p", "k", b, acceptRecord), PreconditionError);
    CHECK_THROWS_AS((void)loadWithBackup<kRecordBytes>(s, "/usd/p", "k", acceptRecord), PreconditionError);
    s.setCardPresent(false);  // two different files: accepted, and the save stops at "no card"
    CHECK(safeReplace<kRecordBytes>(s, "gains.bin", "gains.bak", b, acceptRecord).refused
          == SaveRefusal::NoCard);
}

TEST_CASE("SAFE-SAVE A: the generic save and load serve a sibling file of another size") {
    // What R5a's gains file will do: its own names, its own accept, its own MaxBytes.
    constexpr std::size_t kSize = 40;
    const auto accept40 = [](std::span<const std::byte> bytes, char* why, std::size_t cap) {
        const bool ok = bytes.size() == kSize && bytes[0] == std::byte{0x47};
        if (!ok && why != nullptr && cap > 0) std::snprintf(why, cap, "not a 40-byte G file");
        return ok;
    };
    Bytes first(kSize, std::byte{0x01});
    first[0] = std::byte{0x47};
    Bytes second(kSize, std::byte{0x02});
    second[0] = std::byte{0x47};
    FakeFileStore s{"/usd/"};
    s.setFile("gains.bin", first);
    const SaveReport rep = safeReplace<kSize>(s, "gains.bin", "gains.bak", second, accept40);
    CHECK(rep.outcome == SaveResult::Saved);
    CHECK(rep.before == Which::Primary);
    CHECK(s.file("gains.bak") == std::optional<Bytes>{second});
    s.setFile("gains.bin", prefixOf(second, 10));
    const auto l = loadWithBackup<kSize>(s, "gains.bin", "gains.bak", accept40);
    CHECK(l.from == Which::Backup);
    CHECK(std::string{l.primary.why} == "not a 40-byte G file");
    CHECK(Bytes(l.bytes.begin(), l.bytes.begin() + static_cast<std::ptrdiff_t>(l.count)) == second);
}

namespace {

/// A BROKEN store: every read that opens reports 100 bytes more than it copied — more than the
/// buffer it was handed (IFileStore promises "up to into.size()"). Everything else is the fake's.
class OvercountingStore final : public shulib::hal::IFileStore {
public:
    explicit OvercountingStore(FakeFileStore& inner) : inner_{inner} {}
    [[nodiscard]] bool cardPresent() const override { return inner_.cardPresent(); }
    [[nodiscard]] shulib::hal::StoreRead read(const char* name, std::span<std::byte> into) override {
        shulib::hal::StoreRead r = inner_.read(name, into);
        if (r.opened) r.count += 100;
        return r;
    }
    [[nodiscard]] shulib::hal::StoreReplace replace(const char* name,
                                                    std::span<const std::byte> bytes) override {
        return inner_.replace(name, bytes);
    }

private:
    FakeFileStore& inner_;
};

}  // namespace

TEST_CASE("SAFE-SAVE A edge: a store that reports MORE bytes than the buffer holds -> the save and "
          "the load never hand accept() (or a CRC, or a copy) a byte past the buffer") {
    // BUG CAUGHT: review finding (2026-10-01) — the count trusted as-is: accept(), the CRC and the
    // old-record copy would run over the stack past the read buffer (AddressSanitizer saw it in
    // crc32Ieee). One bug in the next store (a simulator, a bridge) must be a refusal, never stack
    // corruption on the robot. The clamp is in one helper used by both entry points; each is
    // checked here separately.
    constexpr std::size_t kBuffer = kRecordBytes + shulib::calibration::kStoreReadSlackBytes;
    std::size_t widest = 0;
    const auto recordingAccept = [&](std::span<const std::byte> bytes, char* why, std::size_t cap) {
        widest = std::max(widest, bytes.size());
        return acceptRecord(bytes, why, cap);
    };
    FakeFileStore inner = cardStore(cards()[4]);  // card 5: both files hold 180 good bytes
    OvercountingStore broken{inner};
    const auto load = loadWithBackup<kRecordBytes>(broken, kRecordFileName, kRecordBackupFileName,
                                                   recordingAccept);
    CHECK(widest == kBuffer);  // clamped to the buffer, and accept() did see it (length refusal)
    CHECK(load.primary.count == kBuffer);
    CHECK(load.from == Which::None);
    widest = 0;
    const SaveReport rep = safeReplace<kRecordBytes>(broken, kRecordFileName, kRecordBackupFileName,
                                                     kNew(), recordingAccept);
    CHECK(widest == kBuffer);
    CHECK(rep.outcome != SaveResult::Saved);  // no read-back can match: the count is a lie
}

namespace {

/// A BROKEN store: every read says it did NOT open, yet keeps the count (and the bytes) the fake's
/// read produced. IFileStore says the count is 0 then; this store does not, and the load and the
/// save must not depend on it.
class CountsWhenRefusedStore final : public shulib::hal::IFileStore {
public:
    explicit CountsWhenRefusedStore(FakeFileStore& inner) : inner_{inner} {}
    [[nodiscard]] bool cardPresent() const override { return inner_.cardPresent(); }
    [[nodiscard]] shulib::hal::StoreRead read(const char* name, std::span<std::byte> into) override {
        shulib::hal::StoreRead r = inner_.read(name, into);
        r.opened = false;
        return r;
    }
    [[nodiscard]] shulib::hal::StoreReplace replace(const char* name,
                                                    std::span<const std::byte> bytes) override {
        return inner_.replace(name, bytes);
    }

private:
    FakeFileStore& inner_;
};

}  // namespace

TEST_CASE("SAFE-SAVE A edge: a store whose read says NOT OPENED but carries a count -> no byte of it is "
          "counted or accepted, in the load or in the save") {
    // BUG CAUGHT: verification-round mutant V14 (2026-10-02) — countRead() keeping the count of a read
    // that did not open. No test reached it (both shipped stores return 0 there), and the load had
    // just lost its own `opened` check: loadRecord() then returned from=Primary for a file whose read
    // "did not open". Both guards are back (countRead's and loadOne's), and this pins the result.
    int accepts = 0;
    const auto countingAccept = [&](std::span<const std::byte> bytes, char* why, std::size_t cap) {
        ++accepts;
        return acceptRecord(bytes, why, cap);
    };
    FakeFileStore inner = cardStore(cards()[4]);  // card 5: both files hold good records
    CountsWhenRefusedStore broken{inner};
    const auto load = loadWithBackup<kRecordBytes>(broken, kRecordFileName, kRecordBackupFileName,
                                                   countingAccept);
    CHECK(accepts == 0);
    CHECK_FALSE(load.primary.opened);
    CHECK(load.primary.count == 0);
    CHECK_FALSE(load.primary.accepted);
    CHECK(load.backup.tried);
    CHECK(load.backup.count == 0);
    CHECK_FALSE(load.backup.accepted);
    CHECK(load.from == Which::None);
    CHECK(load.count == 0);
    const auto record = loadRecord(broken);
    CHECK(record.from == Which::None);
    CHECK(record.primary.count == 0);
    CHECK_FALSE(record.primary.decoded);
    CHECK(record.backup.count == 0);

    accepts = 0;
    const SaveReport rep = safeReplace<kRecordBytes>(broken, kRecordFileName, kRecordBackupFileName,
                                                     kNew(), countingAccept);
    CHECK(accepts == 1);               // the bytes to save, at step 0 — never a read that did not open
    CHECK(rep.before == Which::None);  // neither step-1 read counts (THE LIMIT (i) on this card)
    CHECK(rep.primaryWrite.attempted);
    CHECK_FALSE(rep.primaryWrite.readOpened);
    CHECK(rep.primaryWrite.readCount == 0);
    CHECK_FALSE(rep.primaryWrite.verified);
    CHECK(rep.outcome == SaveResult::Unconfirmed);  // the record file, written, never re-read
    CHECK(rep.unconfirmed == UnconfirmedWhy::ReadBackDiffers);
}

TEST_CASE("SAFE-SAVE A edge: EMPTY bytes are refused before the card, even by an accept() that takes "
          "anything") {
    // BUG CAUGHT: review mutant N12 — the save's own empty check dropped, leaving it to accept():
    // the header promises an empty file is never saved (it is never accepted on the way back).
    FakeFileStore s{"/usd/"};
    s.setFile("x.bin", Bytes(8, std::byte{1}));
    const auto anything = [](std::span<const std::byte>, char*, std::size_t) { return true; };
    const SaveReport rep =
        safeReplace<16>(s, "x.bin", "x.bak", std::span<const std::byte>{}, anything);
    CHECK(rep.refused == SaveRefusal::BytesRefused);
    CHECK(rep.outcome == SaveResult::NotSaved);
    CHECK(s.log().empty());
    CHECK(s.file("x.bin") == std::optional<Bytes>{Bytes(8, std::byte{1})});
}

TEST_CASE("SAFE-SAVE A edge: a store result's path is CUT to 95 characters plus its terminator -- "
          "never one byte past the array") {
    // BUG CAUGHT: review mutant N07 (the copy's bound off by one: a write one past the 96-byte
    // array). The array sits in a struct before a guard, so an overrun lands in the guard.
    using shulib::hal::kStorePathBytes;
    struct Guarded {
        char path[kStorePathBytes];
        char guard[8];
    };
    Guarded g{};
    std::memset(g.guard, 0x5A, sizeof g.guard);
    const std::string longName(150, 'x');
    shulib::hal::detail::copyStorePath(g.path, longName.c_str());
    const std::ptrdiff_t kept = std::find(g.path, g.path + kStorePathBytes, '\0') - g.path;
    CHECK(kept == static_cast<std::ptrdiff_t>(kStorePathBytes - 1));
    CHECK(std::string(g.path, static_cast<std::size_t>(kept)) == longName.substr(0, kStorePathBytes - 1));
    for (const char guard : g.guard) CHECK(guard == 0x5A);
    const std::string exact(kStorePathBytes - 1, 'y');
    shulib::hal::detail::copyStorePath(g.path, exact.c_str());
    CHECK(std::string{g.path} == exact);
    shulib::hal::detail::copyStorePath(g.path, nullptr);
    CHECK(std::string{g.path}.empty());
}

// ═══ THE WORDS ═══════════════════════════════════════════════════════════════════════════════

namespace {

/// A save from `card` with `plans`, and its words under `b`.
std::vector<std::string> branchWords(std::size_t card, const std::vector<FakeFileFaultPlan>& plans,
                                     BootReadsBackup b, SaveReport* out = nullptr) {
    FakeFileStore s = cardStore(cards()[card]);
    for (const FakeFileFaultPlan& p : plans) s.plan(p);
    const SaveReport rep = saveRecordSafely(s, newDescription());
    if (out != nullptr) *out = rep;
    return words(rep, b);
}

/// The index of the n-th replace of `file` in card `card`'s CLEAN log — the goldens name an
/// operation by the design's step, so they survive a change in how many reads step 1 makes.
std::size_t replaceOp(std::size_t card, const char* file, std::size_t nth = 0) {
    return firstReplaceOf(cleanLog(cards()[card]), file, nth);
}

/// The index of the clean save's LAST operation (step 4's read of K) from card `card`.
std::size_t lastOp(std::size_t card) { return cleanLog(cards()[card]).size() - 1; }

}  // namespace

TEST_CASE("SAFE-SAVE A words: a GOLDEN for every branch, under both BootReadsBackup values") {
    // BUG CAUGHT: mutation 14 ("still on the card" for a BACKUP branch), a branch that prints the
    // wrong file, a NOT SAVED text that says the earlier record survived when it did not (the
    // defect this chunk exists for), and any drift in the words B1 will print. Since the review of
    // 2026-10-01 also: the (u3) line naming the backup alone (review mutant N04), the two put
    // clauses no golden reached ("would not re-open", "different bytes" — N06, N06b), and a
    // reported failure on step 1 whose file step 3 later overwrote (N18).
    using V = std::vector<std::string>;
    const std::string savedLine =
        "SAVED -- /usd/shulib_drivetrain.bin: written, RE-READ and compared byte for byte (180 bytes), "
        "and re-read at the end.";
    const std::string identical =
        "The backup /usd/shulib_drivetrain.bak re-reads identical. The next boot reads the record "
        "file.";
    const std::string writtenBack =
        "The earlier record was written back to the record file and re-read: it is still on the "
        "card, and the next boot uses it as before.";
    const std::size_t c1 = 0;  // card 1: no files
    const std::size_t c3 = 2;  // card 3: a good record file, no backup (robot two's card today)
    const std::size_t c5 = 4;  // card 5: a good record file, an older backup
    const char* P = kRecordFileName;
    const char* K = kRecordBackupFileName;
    for (const BootReadsBackup b : {BootReadsBackup::No, BootReadsBackup::Yes}) {
        CAPTURE(static_cast<int>(b));
        // SAVED, the backup holding the same bytes (card 3, clean).
        CHECK(branchWords(c3, {}, b) == V{savedLine, identical});
        // SAVED, the backup NOT holding them — damaged (a silent cut on step 3).
        CHECK(branchWords(c3, {{replaceOp(c3, K, 1), FakeFileFault::SilentCut, 90}}, b)
              == V{savedLine, "The next boot reads the record file.",
                   "The backup /usd/shulib_drivetrain.bak does NOT hold this record -- it holds a "
                   "damaged file that does not decode. The next save refreshes it."});
        // SAVED, the backup still holding the earlier record (step 3's open refused).
        CHECK(branchWords(c3, {{replaceOp(c3, K, 1), FakeFileFault::OpenRefused}}, b)
              == V{savedLine, "The next boot reads the record file.",
                   "The backup /usd/shulib_drivetrain.bak does NOT hold this record -- it holds the "
                   "record in effect before this run (an older record). The next save refreshes it.",
                   "(The card reported the open of /usd/shulib_drivetrain.bak failing.)"});
        // SAVED, no backup at all (card 1, step 3's open refused).
        CHECK(branchWords(c1, {{replaceOp(c1, K), FakeFileFault::OpenRefused}}, b)
              == V{savedLine, "The next boot reads the record file.",
                   "The backup /usd/shulib_drivetrain.bak does NOT hold this record -- it holds "
                   "nothing (the file is missing). The next save refreshes it.",
                   "(The card reported the open of /usd/shulib_drivetrain.bak failing.)"});
        // SAVED with a reported failure (a false flush on step 2) — D8 (a).
        CHECK(branchWords(c3, {{replaceOp(c3, P), FakeFileFault::FlushFalse}}, b)
              == V{savedLine, identical,
                   "(The card reported the flush of /usd/shulib_drivetrain.bin failing, yet it reads "
                   "back whole.)"});
        // SAVED with a reported failure on STEP 1 (a false flush of the backup copy, which step 3
        // then overwrote): the backup no longer holds what step 1 wrote, so no "reads back whole".
        CHECK(branchWords(c5, {{replaceOp(c5, K), FakeFileFault::FlushFalse}}, b)
              == V{savedLine, identical,
                   "(The card reported the flush of /usd/shulib_drivetrain.bak failing.)"});
        // NOT SAVED, the record file not changed (step 2's open refused).
        CHECK(branchWords(c3, {{replaceOp(c3, P), FakeFileFault::OpenRefused}}, b)
              == V{"NOT SAVED -- the new record did not stick in /usd/shulib_drivetrain.bin (the card "
                   "refused to open it for writing).",
                   "The record file was not changed by this run: the earlier record is still on the "
                   "card, and the next boot uses it as before.",
                   "(The card reported the open of /usd/shulib_drivetrain.bin failing.)"});
        // NOT SAVED, the backup-first copy failed, so P was never opened.
        CHECK(branchWords(c5, {{replaceOp(c5, K), FakeFileFault::ShortWrite, 50}}, b)
              == V{"NOT SAVED -- the backup copy of the earlier record did not stick in "
                   "/usd/shulib_drivetrain.bak (it read back 50 of 180 bytes),",
                   "so the record file was never opened for writing.",
                   "The record file was not changed by this run: the earlier record is still on the "
                   "card, and the next boot uses it as before.",
                   "(The card reported the write of /usd/shulib_drivetrain.bak failing.)"});
        // NOT SAVED, the earlier record written back (step 2's write cut short).
        CHECK(branchWords(c3, {{replaceOp(c3, P), FakeFileFault::ShortWrite, 90}}, b)
              == V{"NOT SAVED -- the new record did not stick in /usd/shulib_drivetrain.bin (it read "
                   "back 90 of 180 bytes).",
                   writtenBack,
                   "(The card reported the write of /usd/shulib_drivetrain.bin failing.)"});
        // NOT SAVED, written back — step 2's write landed, its READ-BACK would not open.
        CHECK(branchWords(c3, {{replaceOp(c3, P) + 1, FakeFileFault::OpenRefused}}, b)
              == V{"NOT SAVED -- the new record did not stick in /usd/shulib_drivetrain.bin (it would "
                   "not re-open for the read-back).",
                   writtenBack});
        // NOT SAVED, written back — step 2's write SILENTLY altered one byte.
        CHECK(branchWords(c3, {{replaceOp(c3, P), FakeFileFault::SilentFlip, 90}}, b)
              == V{"NOT SAVED -- the new record did not stick in /usd/shulib_drivetrain.bin (it read "
                   "back different bytes).",
                   writtenBack});
        // NOT SAVED, nothing usable (card 1, step 2's write cut short).
        CHECK(branchWords(c1, {{replaceOp(c1, P), FakeFileFault::ShortWrite, 90}}, b)
              == V{"NOT SAVED -- the new record did not stick in /usd/shulib_drivetrain.bin (it read "
                   "back 90 of 180 bytes).",
                   "Nothing usable is on the card (neither file decodes): the next boot will REFUSE "
                   "until a run saves.",
                   "(The card reported the write of /usd/shulib_drivetrain.bin failing.)"});
        // UNCONFIRMED (u1), (u2), (u3) naming the record file, (u3) naming the backup.
        const V tail = {"This run cannot say which record the next boot will use. Put the card back "
                        "if it was pulled, copy both files",
                        "off it on a laptop, then start the drive program and read its 'calibration "
                        "record:' line."};
        V u1 = {"SAVE UNCONFIRMED -- the SD card is no longer in the brain."};
        u1.insert(u1.end(), tail.begin(), tail.end());
        u1.push_back("(The card reported the write of /usd/shulib_drivetrain.bin failing.)");
        u1.push_back("(The card was gone when /usd/shulib_drivetrain.bin was to be written.)");
        CHECK(branchWords(c3, {{replaceOp(c3, P), FakeFileFault::Crash, 90, CrashLeave::Torn,
                                AfterCrash::CardGone}},
                          b)
              == u1);
        V u2 = {"SAVE UNCONFIRMED -- the card is in the brain, but nothing on it could be opened or "
                "created (write-protected, full, or failing)."};
        u2.insert(u2.end(), tail.begin(), tail.end());
        u2.push_back("(The card reported the open of /usd/shulib_drivetrain.bin failing.)");
        CHECK(branchWords(c1, {{replaceOp(c1, P), FakeFileFault::OpenRefused}}, b) == u2);
        V u3 = {"SAVE UNCONFIRMED -- the card did not read back the same way twice "
                "(/usd/shulib_drivetrain.bin): pulled, or failing."};
        u3.insert(u3.end(), tail.begin(), tail.end());
        CHECK(branchWords(c5, {{lastOp(c5) - 1, FakeFileFault::ShortRead, 100}}, b) == u3);
        V u3k = {"SAVE UNCONFIRMED -- the card did not read back the same way twice "
                 "(/usd/shulib_drivetrain.bak): pulled, or failing."};
        u3k.insert(u3k.end(), tail.begin(), tail.end());
        CHECK(branchWords(c5, {{lastOp(c5), FakeFileFault::ShortRead, 100}}, b) == u3k);
    }
}

TEST_CASE("SAFE-SAVE A words: the three branches no real save reaches, from a synthetic report -- "
          "the TOTAL line, 'does not hold the new record where the next boot reads it', 'the card "
          "was gone'") {
    // BUG CAUGHT: review mutant N05 (the TOTAL line naming the other file), and drift in the two
    // lines the formatter keeps for combinations the save's own steps never produce. Each report
    // below is built by hand; the save is not run.
    SaveReport r{};
    r.outcome = SaveResult::NotSaved;
    r.length = kRecordBytes;
    r.before = Which::Primary;
    r.primaryWrite.attempted = true;
    r.primaryWrite.readOpened = true;
    r.primaryWrite.readCount = kRecordBytes;
    r.effective = Which::Primary;
    r.primaryAfter = FileHolds::Other;
    r.backupAfter = FileHolds::Other;
    std::snprintf(r.primaryPath, sizeof r.primaryPath, "%s", "/usd/shulib_drivetrain.bin");
    std::snprintf(r.backupPath, sizeof r.backupPath, "%s", "/usd/shulib_drivetrain.bak");
    const std::string head = "NOT SAVED -- the new record did not stick in "
                             "/usd/shulib_drivetrain.bin (it read back different bytes).";
    CHECK(words(r, BootReadsBackup::No)
          == std::vector<std::string>{
              head, "Measured at the end of this run: the next boot's rule picks the record file, "
                    "which holds an older record."});
    r.effective = Which::Backup;
    CHECK(words(r, BootReadsBackup::Yes)
          == std::vector<std::string>{
              head,
              "Measured at the end of this run: the next boot's rule picks the backup, which holds "
              "an older record.",
              "The next boot uses the BACKUP copy (amber on the brain's screen). Rerun shulib "
              "Calibrate to save a new record."});
    // "does not hold the new record where the next boot reads it": no step failed by its own
    // read-back, yet the final read did not find the new record where the boot reads.
    SaveReport d{};
    d.outcome = SaveResult::NotSaved;
    d.length = kRecordBytes;
    d.primaryWrite.attempted = true;
    d.primaryWrite.verified = true;
    d.effective = Which::None;
    d.primaryAfter = FileHolds::NotAccepted;
    d.backupAfter = FileHolds::Absent;
    CHECK(words(d, BootReadsBackup::No)
          == std::vector<std::string>{
              "NOT SAVED -- the card does not hold the new record where the next boot reads it.",
              "Nothing usable is on the card (neither file decodes): the next boot will REFUSE "
              "until a run saves."});
    // "the card was gone": a put whose replace found no card.
    SaveReport g{};
    g.outcome = SaveResult::NotSaved;
    g.length = kRecordBytes;
    g.before = Which::Primary;
    g.primaryWrite.attempted = true;
    g.primaryWrite.reported = ReplaceStep::NoCard;
    g.effective = Which::Primary;
    g.primaryAfter = FileHolds::Earlier;
    CHECK(words(g, BootReadsBackup::No)
          == std::vector<std::string>{
              "NOT SAVED -- the new record did not stick in shulib_drivetrain.bin (the card was "
              "gone).",
              "The record file was not changed by this run: the earlier record is still on the "
              "card, and the next boot uses it as before.",
              "(The card was gone when shulib_drivetrain.bin was to be written.)"});
}

TEST_CASE("SAFE-SAVE A words: the BACKUP branches differ by BootReadsBackup -- 'uses the BACKUP' "
          "only when this build's boot reads it, 'will REFUSE' otherwise") {
    // BUG CAUGHT: B1 shipping words that promise a fallback its Teleop does not have (the reason
    // BootReadsBackup has no default), and mutation 14.
    // Card 3, step 2 cut short AND the restore cut short: P is damaged, the earlier record is safe
    // in the backup (the backup-first step put it there).
    const std::size_t step2 = replaceOp(2, kRecordFileName);
    const std::size_t restore =
        replaceIndexAfter(cards()[2], {{step2, FakeFileFault::ShortWrite, 90}}, kRecordFileName, 1);
    const std::vector<FakeFileFaultPlan> damagedP = {{step2, FakeFileFault::ShortWrite, 90},
                                                     {restore, FakeFileFault::ShortWrite, 20}};
    const std::string head1 =
        "NOT SAVED -- the new record did not stick in /usd/shulib_drivetrain.bin (it read back 90 of "
        "180 bytes).";
    const std::string head2 =
        "The record file /usd/shulib_drivetrain.bin is damaged; the earlier record is safe in the "
        "backup.";
    const std::string failed = "(The card reported the write of /usd/shulib_drivetrain.bin failing.)";
    SaveReport rep{};
    CHECK(branchWords(2, damagedP, BootReadsBackup::No, &rep)
          == std::vector<std::string>{
              head1, head2,
              "The drive program in this build does not read the backup: the next boot will "
              "REFUSE. Rerun shulib Calibrate, or on a laptop copy shulib_drivetrain.bak over "
              "shulib_drivetrain.bin.",
              failed, failed});
    CHECK(rep.restore.attempted);
    CHECK_FALSE(rep.restore.verified);
    CHECK(branchWords(2, damagedP, BootReadsBackup::Yes)
          == std::vector<std::string>{
              head1, head2,
              "The next boot uses the BACKUP copy (amber on the brain's screen). Rerun shulib "
              "Calibrate to save a new record.",
              failed, failed});
    // Card 2, step 2 cut short: the record file did not read as usable, the backup holds an
    // earlier record — said as what step 1 READ (header: THE LIMIT), never "there was none".
    const std::string notUsable =
        "The record file did not read as usable when this run began; the backup copy "
        "/usd/shulib_drivetrain.bak holds an earlier record.";
    const std::vector<FakeFileFaultPlan> cut = {
        {replaceOp(1, kRecordFileName), FakeFileFault::ShortWrite, 90}};
    const std::vector<std::string> no = branchWords(1, cut, BootReadsBackup::No);
    const std::vector<std::string> yes = branchWords(1, cut, BootReadsBackup::Yes);
    REQUIRE(no.size() == 4);
    REQUIRE(yes.size() == 4);
    CHECK(no[1] == notUsable);
    CHECK(yes[1] == notUsable);
    CHECK(no[2].find("will REFUSE") != std::string::npos);
    CHECK(yes[2].find("next boot uses the BACKUP") != std::string::npos);
    for (const auto& l : no) CHECK(l.find("still on the card") == std::string::npos);
    for (const auto& l : yes) CHECK(l.find("still on the card") == std::string::npos);
}

TEST_CASE("SAFE-SAVE A words: the refusals before the card, each with its reason") {
    SaveReport r{};
    r.refused = SaveRefusal::DescriptionInvalid;
    std::snprintf(r.why, sizeof r.why, "%s", "cartridge byte 0 is not Red, Green or Blue");
    CHECK(words(r, BootReadsBackup::No)
          == std::vector<std::string>{
              "NOT SAVED -- the measured description is not valid: cartridge byte 0 is not Red, "
              "Green or Blue",
              "Nothing on the card was opened or written by this run: whatever it held, it still "
              "holds."});
    r.refused = SaveRefusal::EncodeFailed;
    std::snprintf(r.why, sizeof r.why, "%s", "0 of 180 bytes encoded");
    CHECK(words(r, BootReadsBackup::Yes)[0] == "NOT SAVED -- the record would not encode: 0 of 180 bytes encoded");
    r.refused = SaveRefusal::BytesRefused;
    CHECK(words(r, BootReadsBackup::Yes)[0].rfind("NOT SAVED -- the bytes to save do not decode", 0) == 0);
}

TEST_CASE("SAFE-SAVE A words: EVERY measured combination prints something, ASCII, no '%', under the "
          "line budget -- and no line claims what its report does not say") {
    // BUG CAUGHT: a formatter that is not TOTAL (a combination that prints nothing), a line that
    // runs past Calibrate's emitf buffer, a '%' that would reach a printf, and — the defect this
    // chunk exists for — a sentence chosen from what the code meant to do rather than from the
    // measurement: "still on the card", "boot from the BACKUP", "will REFUSE", "re-reads identical",
    // "reads back whole", SAVED / NOT SAVED / UNCONFIRMED, each checked against the fields that
    // alone may justify it — and the (u3) line naming exactly the file(s) that disagreed. Paths at
    // their full 95 characters, so the budget is tested at its worst, and DIFFERENT for the two
    // files, so a line naming the wrong one is seen.
    std::string longP = "/usd/";
    while (longP.size() < shulib::hal::kStorePathBytes - 1) longP += "p";
    std::string longK = "/usd/";
    while (longK.size() < shulib::hal::kStorePathBytes - 1) longK += "k";
    const FileHolds holds[] = {FileHolds::NotMeasured, FileHolds::Absent, FileHolds::NotAccepted,
                               FileHolds::New, FileHolds::Earlier, FileHolds::Other};
    const Which whiches[] = {Which::Primary, Which::Backup, Which::None};
    const auto put = [](int kind, ReplaceStep step) {
        PutRecord p{};
        p.attempted = kind > 0;
        p.verified = kind == 1;
        p.reported = kind > 0 ? step : ReplaceStep::None;
        p.readOpened = kind != 3;
        p.readCount = kind == 2 ? 90U : kRecordBytes;
        return p;
    };
    std::size_t combos = 0;
    int failures = 0;
    const auto fail = [&](const std::string& why, const std::string& text) {
        if (failures < 6) FAIL_CHECK(why << "\n" << text);
        ++failures;
    };
    for (const SaveResult outcome : {SaveResult::Saved, SaveResult::NotSaved, SaveResult::Unconfirmed}) {
    for (const UnconfirmedWhy u : {UnconfirmedWhy::None, UnconfirmedWhy::CardGone,
                                   UnconfirmedWhy::NothingOpened, UnconfirmedWhy::ReadBackDiffers}) {
    if ((outcome == SaveResult::Unconfirmed) != (u != UnconfirmedWhy::None)) continue;
    // (u3): the record file alone, the backup alone, or both disagreed; one case otherwise.
    for (int disagreed = 0; disagreed < 3; ++disagreed) {
    if (u != UnconfirmedWhy::ReadBackDiffers && disagreed > 0) continue;
    for (const Which before : whiches) {
    for (const Which effective : whiches) {
    for (const FileHolds pAfter : holds) {
    for (const FileHolds kAfter : holds) {
    for (int restoreKind = 0; restoreKind < 4; ++restoreKind) {
    for (int firstKind = 0; firstKind < 4; ++firstKind) {
    for (const ReplaceStep step : {ReplaceStep::Write, ReplaceStep::NoCard}) {
    for (const BootReadsBackup b : {BootReadsBackup::No, BootReadsBackup::Yes}) {
        SaveReport r{};
        r.outcome = outcome;
        r.unconfirmed = u;
        r.primaryDisagreed = u == UnconfirmedWhy::ReadBackDiffers && disagreed != 1;
        r.backupDisagreed = u == UnconfirmedWhy::ReadBackDiffers && disagreed != 0;
        r.before = before;
        r.effective = effective;
        r.primaryAfter = pAfter;
        r.backupAfter = kAfter;
        r.length = kRecordBytes;
        r.backupFirst = put(firstKind, step);
        r.primaryWrite = put(firstKind == 2 ? 0 : 2, ReplaceStep::Flush);
        r.restore = put(restoreKind, step);
        r.backupRefresh = put(restoreKind == 1 ? 1 : 0, ReplaceStep::Flush);
        std::snprintf(r.primaryPath, sizeof r.primaryPath, "%s", longP.c_str());
        std::snprintf(r.backupPath, sizeof r.backupPath, "%s", longK.c_str());
        ++combos;
        const std::vector<std::string> lines = words(r, b);
        const std::string text = joined(lines);
        if (lines.empty()) { fail("printed nothing", text); continue; }
        for (const std::string& l : lines) {
            if (l.size() >= kSaveReportLineBytes) fail("a line over the budget", l);
            for (const char ch : l) {
                if (ch == '%' || ch < 0x20 || ch > 0x7E) { fail("non-ASCII or %", l); break; }
            }
        }
        const auto has = [&](const char* s) { return text.find(s) != std::string::npos; };
        const bool savedLine = std::any_of(lines.begin(), lines.end(), [](const std::string& l) {
            return l.rfind("SAVED --", 0) == 0;
        });
        if (savedLine != (outcome == SaveResult::Saved)) fail("SAVED without Saved", text);
        if (has("NOT SAVED") != (outcome == SaveResult::NotSaved)) fail("NOT SAVED without NotSaved", text);
        if (has("SAVE UNCONFIRMED") != (outcome == SaveResult::Unconfirmed)) fail("UNCONFIRMED mismatch", text);
        if (has("still on the card")
            && !(outcome == SaveResult::NotSaved && effective == Which::Primary
                 && pAfter == FileHolds::Earlier
                 && (!r.restore.attempted || r.restore.verified))) {
            fail("'still on the card' without the measurement", text);
        }
        if (has("uses the BACKUP")
            && !(outcome == SaveResult::NotSaved && effective == Which::Backup && b == BootReadsBackup::Yes)) {
            fail("'uses the BACKUP' without a measured backup and a boot that reads it", text);
        }
        if (has("does not read the backup")
            && !(outcome == SaveResult::NotSaved && effective == Which::Backup && b == BootReadsBackup::No)) {
            fail("'does not read the backup' without a measured backup and a boot that ignores it", text);
        }
        if (has("will REFUSE")
            && !(outcome == SaveResult::NotSaved
                 && (effective == Which::None || (effective == Which::Backup && b == BootReadsBackup::No)))) {
            fail("'will REFUSE' without a measured refusal", text);
        }
        if (has("will REFUSE until a run saves")
            && !(outcome == SaveResult::NotSaved && effective == Which::None)) {
            fail("'nothing usable' without a measured nothing", text);
        }
        if (has("re-reads identical") && !(outcome == SaveResult::Saved && kAfter == FileHolds::New)) {
            fail("'re-reads identical' without the backup measured New", text);
        }
        if (has("yet it reads back whole") && outcome == SaveResult::Unconfirmed) {
            fail("'reads back whole' on an Unconfirmed save", text);
        }
        if (has("is damaged; the earlier record is safe")
            && !(effective == Which::Backup && kAfter == FileHolds::Earlier && before == Which::Primary)) {
            fail("'safe in the backup' without the measurement", text);
        }
        if (has("did not read as usable when this run began")
            && !(effective == Which::Backup && kAfter == FileHolds::Earlier && before == Which::Backup)) {
            fail("'did not read as usable' without the measurement", text);
        }
        if (has("no usable record file")) fail("a claim about the card step 1 could not measure", text);
        if (u == UnconfirmedWhy::ReadBackDiffers) {
            const std::string named = r.primaryDisagreed && r.backupDisagreed ? "(both files)"
                                      : r.primaryDisagreed                    ? "(" + longP + ")"
                                                                              : "(" + longK + ")";
            if (lines[0] != "SAVE UNCONFIRMED -- the card did not read back the same way twice "
                                + named + ": pulled, or failing.") {
                fail("the (u3) line does not name exactly the file(s) that disagreed", text);
            }
        }
        if (outcome == SaveResult::Unconfirmed
            && (has("next boot uses") || has("next boot reads") || has("will REFUSE")
                || has("Nothing usable"))) {
            fail("an Unconfirmed save that claims which record boots", text);
        }
    }}}}}}}}}}}
    MESSAGE("formatter combinations: " << combos);
    CHECK(failures == 0);
    // 2 measured outcomes + (u1) + (u2) + 3 x (u3) = 7 report kinds, x 3 before x 3 effective x 6
    // x 6 file states x 4 x 4 put kinds x 2 steps x 2 BootReadsBackup = 7 x 20736 = 145152.
    CHECK(combos == 145152);
}
