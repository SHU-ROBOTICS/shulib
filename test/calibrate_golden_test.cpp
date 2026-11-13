// calibrate_golden_test.cpp — the standard Calibrate THROUGH ITS REAL PROS SHELL, 34 scenarios, each
// equal to its committed GOLDEN: the serial log, the screen's draw log and the card's two files
// (chunk CAL1, Part A2b, 2026-10-30 — the CAL1 brief's §2.A test 10; test/calibrate_golden_rig.hpp is
// the machine, and says what runs through what).
//
// THE SCENARIOS are the brief's 34, with the probe harness's own knobs (brief-CAL1
// d6a/v3/harness4.cpp): robot two's fifteen (one, clean, slip18, curved, nocard, noimu, disable,
// flip-again, flip-new, flip-disable, swap, watch, watch-typo-again, watch-typo-save, undeclared), D6
// (a)'s eleven (typed-first, typed-flip-new, typed-flip-again, typed-flip-disable, typed-card-wins and
// its untyped control, typed-undecodable, typed-undecodable-flip, typed-tie, typed-nocommon,
// typed-watch-typo) and the check of D6 (a)'s eight (typed-partial, empty-watch-untyped,
// undecodable-untyped, bakonly-garbage-untyped, bakonly-garbage-typed, emptyfile-untyped,
// nocard-watch-again, typed-nocard). Robot two's facts are DERIVED from its tables exactly as its glue
// derives them (CAL1 Part B: withTypedDriveFrom(calibrateFactsFor(kRobotTwoChassis,
// kRobotTwoMechanisms), kRobotTwoChassis) — the rig's factsFor()), with NO typed drive ports (its
// table is not a description the boot can drive from); every golden stayed unchanged when they
// stopped being written literally (A2b wrote them so); the typed scenarios add their own.
//
// THE GOLDENS (test/goldens/calibrate/<scenario>.serial.log, .screen.log, .card.txt) were made by this
// test from the library as it stood at CAL1 Part A2b, and BEFORE they were committed every file was
// compared with the brief's probe goldens (brief-CAL1 d6a/v3/run_new/) and every difference given a
// named reason (CAL1-PROGRESS.md, Part A2b) — they are not "whatever the code printed". The host is
// deterministic, so every timing line is pinned as it prints. Two things are normalized, and only
// two: the card's directory (this machine's) and the build's git hash. A golden that no longer
// matches is a CHANGE in what the program says, draws or saves: read the first differing line the
// failure names, decide whether the change is meant, and only then regenerate — with
// SHULIB_GOLDEN_OUT=<dir> set, every scenario's actual files are ALSO written there (the case still
// compares, and still fails), to be read and copied in by hand. Nothing regenerates itself.
//
// BESIDE THE GOLDENS, each scenario asserts what the files cannot show: the declared ports were never
// configured or commanded (the shim's gearing, units, voltage, brake-mode and brake() counters at 0 —
// the watch's C reads write nothing, HA-187 (1) as the shim models it); the WATCH's C reads, counted
// by the shim, are exactly 2 per push it judged and 1 per push it did not (the CAL1 brief's test 8b
// "2 per push", through the real shell — A2a's PC1 counts them per call, its T5j per push on the
// fakes); what a FORWARD command on the saved record does to the plant's wheels (the probe's score:
// 10 of 10 after a front-led push or a cable swap saved with USE NEW, 0 of 10 after USE NEW on a
// wrong push); the undeclared motors' voltages (8 never powered, 9 pulsed to 9 V); and two equalities
// the brief names — typed-card-wins IS its untyped control (with a record on the card the typed ports
// change nothing), typed-nocard IS nocard (no card, no check, typed or not). A seeded scenario never
// runs without its seed: `one`'s saved record, REQUIRED to exist and decode first.
//
// A SOURCE PIN holds the composition: hal::pros::runCalibrate()'s body is EXACTLY the same five the
// rig builds (ProsDeviceRegistry, ProsDeviceProvider over it, ProsFileStore with its DEFAULT root,
// ProsBrainScreen, ProsCalibrateIo) and ONE run of the program over them, line for line — so the
// rig's one difference, the card's root, is the only one. The card's root is a short mkdtemp
// directory (workDir()), so the save's lines carry its path whole on any host. Nothing here has run
// on a robot.

#include <unistd.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "calibrate_golden_rig.hpp"
#include "doctest.h"
#include "shulib/calibration/calibration_record.hpp"
#include "shulib/calibration/record_store.hpp"

#ifndef SHULIB_SOURCE_DIR
#error "SHULIB_SOURCE_DIR must be defined by the build (test/CMakeLists.txt)"
#endif

namespace {

using namespace cal_golden;
namespace fs = std::filesystem;

/// The expectations a golden file cannot hold, per scenario, written in the test (never computed
/// by the code under test): the probe's FORWARD score on the saved record ("10 of 10", or "none"
/// when the card holds no record file), and the WATCH's position reads per declared port.
struct Expect {
    const char* forward = "none";
    int reads6 = 0;
    int reads7 = 0;   ///< claw pivot 7 — or 0 when the facts declare 12 instead (typo12)
    int reads12 = 0;  ///< the typo: drive port 12 declared a mechanism
};

struct Named {
    Scenario sc;
    Expect expect;
};

/// The 34 (header), each the probe's knobs. The reads: a push begun is read once (the watch's
/// baseline), a push the calibrator ACCEPTED once more at DONE — so 2 per accepted push, 1 per push
/// refused at DONE (CAL1 A2a.1 #3: a refused push is never judged), 0 with no push.
Named scenarioNamed(const std::string& name) {
    Named n;
    Scenario& s = n.sc;
    Expect& e = n.expect;
    s.name = name;
    e = {"10 of 10", 2, 2, 0};
    if (name == "one") {
    } else if (name == "clean") {
        s.runs = 2;
        e = {"10 of 10", 4, 4, 0};
    } else if (name == "slip18") {
        s.decouple18 = true;
        e = {"9 of 9", 2, 2, 0};
    } else if (name == "curved") {
        s.runs = 2;
        s.curve[0] = 1.6;  // run 1 curves: STOPPED PUSH CURVED at DONE; run 2 straight
        e = {"10 of 10", 3, 3, 0};
    } else if (name == "nocard") {
        s.card = false;
        e = {"none", 2, 2, 0};
    } else if (name == "noimu") {
        s.runs = 2;
        s.imu = false;
        e = {"none", 0, 0, 0};
    } else if (name == "disable") {
        s.disableAfterPulses = 8.0;  // mid-pulses
        e = {"none", 2, 2, 0};
    } else if (name == "flip-again") {  // held -> RUN AGAIN -> a front-led run 3: SAME
        s.runs = 3;
        s.pushDir[1] = -1.0;
        s.heldChoice[1] = kRunAgain;
        e = {"10 of 10", 6, 6, 0};
    } else if (name == "flip-new") {  // the wrong push, then USE NEW: the warned-of case
        s.runs = 2;
        s.pushDir[1] = -1.0;
        s.heldChoice[1] = kLeftBox;
        e = {"0 of 10", 4, 4, 0};
    } else if (name == "flip-disable") {  // held, then the field disables: the card keeps run 1's
        s.runs = 2;
        s.pushDir[1] = -1.0;
        s.heldChoice[1] = kDisable;
        e = {"10 of 10", 4, 4, 0};
    } else if (name == "swap") {  // seeded; the bundles swapped; USE NEW is RIGHT here
        s.swapBundles = true;
        s.heldChoice[0] = kLeftBox;
        s.seed = Seed::FrontLedRecord;
    } else if (name == "watch") {  // a claw the push turns: SAVE keeps it declared
        s.clawTurnsPort = 6;
        s.heldChoice[0] = kLeftBox;
    } else if (name == "watch-typo-again") {  // drive port 12 declared: held, RUN AGAIN, held, disable
        s.runs = 2;
        s.typo12 = true;
        s.heldChoice[0] = kRunAgain;
        s.heldChoice[1] = kDisable;
        s.seed = Seed::FrontLedRecord;
        e = {"10 of 10", 4, 0, 4};
    } else if (name == "watch-typo-save") {  // the informed SAVE: the record loses port 12
        s.typo12 = true;
        s.heldChoice[0] = kLeftBox;
        s.seed = Seed::FrontLedRecord;
        e = {"9 of 9", 2, 0, 2};
    } else if (name == "undeclared") {
        s.undeclared = true;
    } else if (name == "typed-first") {  // an empty card, the typed ports' end leading: SAME, exactly
        s.typedCase = 8;
    } else if (name == "typed-flip-new") {  // the other end leads: held, USE NEW
        s.typedCase = 8;
        s.pushDir[0] = -1.0;
        s.heldChoice[0] = kLeftBox;
        e = {"0 of 10", 2, 2, 0};
    } else if (name == "typed-flip-again") {
        s.runs = 2;
        s.typedCase = 8;
        s.pushDir[0] = -1.0;
        s.heldChoice[0] = kRunAgain;
        e = {"10 of 10", 4, 4, 0};
    } else if (name == "typed-flip-disable") {
        s.typedCase = 8;
        s.pushDir[0] = -1.0;
        s.heldChoice[0] = kDisable;
        e = {"none", 2, 2, 0};
    } else if (name == "typed-card-wins") {  // seeded; the typed ports FLIPPED against the record
        s.typedCase = 2;
        s.seed = Seed::FrontLedRecord;
    } else if (name == "typed-card-wins-untyped") {  // its control: no typed ports
        s.seed = Seed::FrontLedRecord;
    } else if (name == "typed-undecodable") {
        s.typedCase = 8;
        s.seed = Seed::Garbage;
    } else if (name == "typed-undecodable-flip") {
        s.typedCase = 8;
        s.pushDir[0] = -1.0;
        s.heldChoice[0] = kLeftBox;
        s.seed = Seed::Garbage;
        e = {"0 of 10", 2, 2, 0};
    } else if (name == "typed-tie") {
        s.typedCase = 3;
    } else if (name == "typed-nocommon") {
        s.typedCase = 4;
    } else if (name == "typed-watch-typo") {  // empty card; drive port 12 declared; typed ports drive it
        s.runs = 2;
        s.typo12 = true;
        s.typedCase = 8;
        s.heldChoice[0] = kRunAgain;
        s.heldChoice[1] = kDisable;
        e = {"none", 4, 0, 4};
    } else if (name == "typed-partial") {
        s.typedCase = 9;
    } else if (name == "empty-watch-untyped") {  // the discard words follow the CARD, not the reference
        s.runs = 2;
        s.typo12 = true;
        s.heldChoice[0] = kRunAgain;
        s.heldChoice[1] = kDisable;
        e = {"none", 4, 0, 4};
    } else if (name == "undecodable-untyped") {
        s.seed = Seed::Garbage;
    } else if (name == "bakonly-garbage-untyped") {
        s.seed = Seed::BackupGarbage;
    } else if (name == "bakonly-garbage-typed") {
        s.typedCase = 8;
        s.seed = Seed::BackupGarbage;
    } else if (name == "emptyfile-untyped") {
        s.seed = Seed::EmptyRecordFile;
    } else if (name == "nocard-watch-again") {
        s.runs = 2;
        s.card = false;
        s.typo12 = true;
        s.heldChoice[0] = kRunAgain;
        s.heldChoice[1] = kDisable;
        e = {"none", 4, 0, 4};
    } else if (name == "typed-nocard") {
        s.card = false;
        s.typedCase = 8;
        e = {"none", 2, 2, 0};
    } else {
        FAIL("no such golden scenario: " << name);
    }
    return n;
}

/// The card's two files as a golden holds them: each file's size and its bytes in hex, 30 a line,
/// or "absent".
std::string cardText(const Result& r) {
    std::ostringstream o;
    const auto one = [&o](const char* name, const std::optional<std::vector<char>>& f) {
        if (!f) {
            o << name << ": absent\n";
            return;
        }
        o << name << ": " << f->size() << " bytes\n";
        for (std::size_t i = 0; i < f->size(); i += 30) {
            o << " ";
            for (std::size_t k = i; k < i + 30 && k < f->size(); ++k) {
                char h[4];
                std::snprintf(h, sizeof h, " %02x", static_cast<unsigned>(static_cast<unsigned char>((*f)[k])));
                o << h;
            }
            o << "\n";
        }
    };
    one(shulib::calibration::kRecordFileName, r.bin);
    one(shulib::calibration::kRecordBackupFileName, r.bak);
    return o.str();
}

std::string joinLines(const std::vector<std::string>& v) {
    std::string s;
    for (const auto& l : v) {
        s += l;
        s += '\n';
    }
    return s;
}

std::string goldenPath(const std::string& scenario, const char* kind) {
    return std::string{SHULIB_SOURCE_DIR} + "/test/goldens/calibrate/" + scenario + "." + kind;
}

/// The golden's text, and whether it could be read (a missing golden FAILS: it is never "no
/// difference").
bool readGolden(const std::string& path, std::string& out) {
    std::ifstream f{path, std::ios::binary};
    if (!f) {
        return false;
    }
    out.assign(std::istreambuf_iterator<char>{f}, std::istreambuf_iterator<char>{});
    return true;
}

/// The first line where `a` and `b` differ, 1-based, with both lines — for a failure that says
/// WHERE, not only that.
std::string firstDifference(const std::string& actual, const std::string& golden) {
    std::istringstream a{actual};
    std::istringstream g{golden};
    std::string la;
    std::string lg;
    for (int n = 1;; ++n) {
        const bool ha = static_cast<bool>(std::getline(a, la));
        const bool hg = static_cast<bool>(std::getline(g, lg));
        if (!ha && !hg) {
            return "(no line differs: the files differ only in their final newline)";
        }
        if (!ha || !hg || la != lg) {
            return "line " + std::to_string(n) + "\n  golden: " + (hg ? lg : "<end of file>")
                   + "\n  actual: " + (ha ? la : "<end of file>");
        }
    }
}

/// The actual files, written beside each other under $SHULIB_GOLDEN_OUT when it is set (header).
void writeActual(const std::string& scenario, const std::string& serial, const std::string& screen,
                 const std::string& card) {
    const char* outDir = std::getenv("SHULIB_GOLDEN_OUT");
    if (outDir == nullptr || outDir[0] == '\0') {
        return;
    }
    fs::create_directories(outDir);
    const auto put = [&](const char* kind, const std::string& text) {
        std::ofstream f{std::string{outDir} + "/" + scenario + "." + kind, std::ios::binary};
        f << text;
    };
    put("serial.log", serial);
    put("screen.log", screen);
    put("card.txt", card);
}

/// The longest card root whose record file's path the save's lines print WHOLE: they carry at most
/// 95 characters of a path (formatSaveReport's "%.95s"), and a cut root could not be normalized.
constexpr std::size_t kMaxCardRoot = 95 - std::string_view{shulib::calibration::kRecordFileName}.size();
static_assert(kMaxCardRoot == 74);  // "shulib_drivetrain.bin" (and its backup's name) is 21

/// The work directory, made ONCE per process by mkdtemp (unique, so the suite may run beside
/// another), the card inside it — or empty when none could be made (runScenario() then fails,
/// naming why). Its card root must print whole (kMaxCardRoot), so the root is kept SHORT whatever
/// the environment says (CAL1-A2b's review, 2026-10-30: macOS's TMPDIR, /var/folders/xx/<28>/T/,
/// is 49 characters — with the old "shulib_calgold_<pid>" name every scenario failed there):
/// "scg-XXXXXX" under this machine's temporary directory when the root fits there, else under
/// /tmp, whose length no environment variable sets.
fs::path workDir() {
    static const fs::path dir = [] {
        std::vector<fs::path> bases;
        std::error_code ec;
        const fs::path tmp = fs::temp_directory_path(ec);
        if (!ec && !tmp.empty()) {
            bases.push_back(tmp);
        }
        bases.emplace_back("/tmp");
        for (const fs::path& base : bases) {
            const std::string templ = (base / "scg-XXXXXX").string();
            if ((fs::path{templ} / "card").string().size() + 1 > kMaxCardRoot) {
                continue;  // this base would cut the card's path in the save's lines
            }
            std::vector<char> name(templ.begin(), templ.end());
            name.push_back('\0');
            if (::mkdtemp(name.data()) != nullptr) {
                return fs::path{name.data()};
            }
        }
        return fs::path{};
    }();
    return dir;
}

/// Every run, once per process: the equality cases and the seeded scenarios reuse them.
std::map<std::string, Result>& results() {
    static std::map<std::string, Result> r;
    return r;
}

const Result& runScenario(const std::string& name);

/// `one`'s saved record — the seed every "card holds a record" scenario starts from, REQUIRED to
/// exist and decode before any seeded scenario runs (the CAL1 check of D6 (a), C2).
const std::vector<char>& frontLedSeed() {
    const std::string oneName = "one";  // named: a temporary here draws -Wdangling-reference
    const Result& one = runScenario(oneName);
    REQUIRE(one.bin.has_value());
    REQUIRE(one.bin->size() == shulib::calibration::kRecordBytes);
    shulib::hal::DrivetrainDescription d{};
    const auto* b = reinterpret_cast<const std::byte*>(one.bin->data());
    REQUIRE(shulib::calibration::decodeRecord(std::span<const std::byte>{b, one.bin->size()}, d));
    return *one.bin;
}

const Result& runScenario(const std::string& name) {
    auto it = results().find(name);
    if (it != results().end()) {
        return it->second;
    }
    const Named n = scenarioNamed(name);
    const std::vector<char> garbage(180, 'x');
    const std::vector<char> empty;
    const std::vector<char>* bin = nullptr;
    const std::vector<char>* bak = nullptr;
    switch (n.sc.seed) {
        case Seed::None: break;
        case Seed::FrontLedRecord: bin = bak = &frontLedSeed(); break;
        case Seed::Garbage: bin = bak = &garbage; break;
        case Seed::BackupGarbage: bak = &garbage; break;
        case Seed::EmptyRecordFile: bin = &empty; break;
    }
    REQUIRE_MESSAGE(!workDir().empty(),
                    "no work directory could be made (mkdtemp under this machine's temporary "
                    "directory and under /tmp)");
    Rig rig{n.sc, workDir()};
    const std::string root = rig.cardRoot();
    REQUIRE_MESSAGE(root.size() <= kMaxCardRoot,
                    "the card root is too long for the save's lines to carry the card's path whole: "
                    << root);
    Result r = rig.run(factsFor(n.sc), bin, bak);
    return results().emplace(name, std::move(r)).first->second;
}

/// One scenario against its goldens, and everything a golden cannot show (header).
void checkScenario(const std::string& name) {
    CAPTURE(name);
    const Named n = scenarioNamed(name);
    const Result& r = runScenario(name);
    REQUIRE(r.captured);
    REQUIRE_FALSE(r.timedOut);

    // every serial line is ONE tagged line ("[R3D] " ... "\n")
    REQUIRE_FALSE(r.serial.empty());
    CHECK(r.serial.back() == '\n');
    {
        std::istringstream in{r.serial};
        std::string line;
        int untagged = 0;
        while (std::getline(in, line)) {
            untagged += line.rfind("[R3D] ", 0) == 0 ? 0 : 1;
        }
        CHECK(untagged == 0);
    }

    // the goldens
    const std::string screen = joinLines(r.screen);
    const std::string card = cardText(r);
    writeActual(name, r.serial, screen, card);
    const std::pair<const char*, const std::string*> files[] = {
        {"serial.log", &r.serial}, {"screen.log", &screen}, {"card.txt", &card}};
    for (const auto& f : files) {
        const char* kind = f.first;
        const std::string* actual = f.second;
        CAPTURE(kind);
        std::string golden;
        const std::string path = goldenPath(name, kind);
        if (!readGolden(path, golden)) {
            FAIL_CHECK("no golden at " << path);
            continue;
        }
        const bool same = *actual == golden;
        if (!same) {
            INFO("first difference: " << firstDifference(*actual, golden));
            CHECK(same);
        }
    }

    // the declared ports: never configured, never commanded — only read, through the C reads
    const int declared[] = {6, n.sc.typo12 ? 12 : 7};
    for (const int p : declared) {
        CAPTURE(p);
        const auto& m = r.motors[static_cast<std::size_t>(p)];
        CHECK(m.setGearingCalls == 0);
        CHECK(m.setEncoderUnitsCalls == 0);
        CHECK(m.moveVoltageCalls == 0);
        CHECK(m.setBrakeModeCalls == 0);
        CHECK(m.brakeCalls == 0);
        CHECK(r.maxAbsMv[static_cast<std::size_t>(p)] == 0.0);
        // the shim's default units are rotations: each read is one position and one units read,
        // never a gearing read (that is the COUNTS path only)
        CHECK(r.cReads[static_cast<std::size_t>(p)].units == r.cReads[static_cast<std::size_t>(p)].position);
        CHECK(r.cReads[static_cast<std::size_t>(p)].gearing == 0);
    }
    CHECK(r.cReads[6].position == n.expect.reads6);
    CHECK(r.cReads[7].position == n.expect.reads7);
    CHECK(r.cReads[12].position == n.expect.reads12);
    if (n.sc.typo12) {
        // claw pivot 7 is NOT declared under the typo: a candidate the push does not turn — geared
        // once at construction, never powered, never read by the watch
        CHECK(r.motors[7].setGearingCalls == 1);
        CHECK(r.maxAbsMv[7] == 0.0);
    }
    if (n.sc.undeclared) {
        CHECK(r.motors[8].setGearingCalls == 1);
        CHECK(r.maxAbsMv[8] == 0.0);        // 8 did not turn: never powered
        CHECK(r.maxAbsMv[9] == 9000.0);     // 9 the push turned: pulsed up to 9 V
        CHECK(r.poweredSteps[9] > 0);
    }

    // what a FORWARD command on the saved record does to the plant's wheels
    CHECK(r.forward == n.expect.forward);
}

/// Two runs that must be the same run: serial, screen and card.
void checkSameRun(const std::string& a, const std::string& b) {
    CAPTURE(a);
    CAPTURE(b);
    const Result& ra = runScenario(a);
    const Result& rb = runScenario(b);
    REQUIRE(ra.captured);
    REQUIRE(rb.captured);
    if (ra.serial != rb.serial) {
        INFO("first serial difference: " << firstDifference(ra.serial, rb.serial));
        CHECK(ra.serial == rb.serial);
    }
    CHECK(ra.screen == rb.screen);
    CHECK(cardText(ra) == cardText(rb));
}

std::string readSource(const char* relative) {
    std::ifstream f{std::string{SHULIB_SOURCE_DIR} + "/" + relative};
    REQUIRE_MESSAGE(f.good(), "cannot open " << relative);
    return std::string{std::istreambuf_iterator<char>{f}, std::istreambuf_iterator<char>{}};
}

}  // namespace

TEST_CASE("golden one: a clean run on an empty card -- the FIRST record") { checkScenario("one"); }
TEST_CASE("golden clean: two runs, RUN AGAIN between") { checkScenario("clean"); }
TEST_CASE("golden slip18: port 18 slips -- LEFT OUT") { checkScenario("slip18"); }
TEST_CASE("golden curved: a curved push refused at DONE, then a straight one") { checkScenario("curved"); }
TEST_CASE("golden nocard: no SD card -- no check, NOT SAVED") { checkScenario("nocard"); }
TEST_CASE("golden noimu: no IMU -- refused before anything is made") { checkScenario("noimu"); }
TEST_CASE("golden disable: the field disables the robot mid-pulses") { checkScenario("disable"); }
TEST_CASE("golden flip-again: FRONT FLIPPED, RUN AGAIN, then SAME") { checkScenario("flip-again"); }
TEST_CASE("golden flip-new: FRONT FLIPPED, USE NEW after a wrong push") { checkScenario("flip-new"); }
TEST_CASE("golden flip-disable: the field disables a held FRONT FLIPPED") { checkScenario("flip-disable"); }
TEST_CASE("golden swap: the cable bundles swapped, USE NEW") { checkScenario("swap"); }
TEST_CASE("golden watch: a claw the push turns, SAVE") { checkScenario("watch"); }
TEST_CASE("golden watch-typo-again: a drive port declared -- RUN AGAIN, then a disable") {
    checkScenario("watch-typo-again");
}
TEST_CASE("golden watch-typo-save: a drive port declared -- the informed SAVE") {
    checkScenario("watch-typo-save");
}
TEST_CASE("golden undeclared: two undeclared mechanism motors") { checkScenario("undeclared"); }
TEST_CASE("golden typed-first: an empty card, SAME as the typed drive ports") { checkScenario("typed-first"); }
TEST_CASE("golden typed-flip-new: FRONT FLIPPED against the typed ports, USE NEW") {
    checkScenario("typed-flip-new");
}
TEST_CASE("golden typed-flip-again: FRONT FLIPPED against the typed ports, RUN AGAIN") {
    checkScenario("typed-flip-again");
}
TEST_CASE("golden typed-flip-disable: the field disables a typed FRONT FLIPPED") {
    checkScenario("typed-flip-disable");
}
TEST_CASE("golden typed-card-wins: a record on the card decides; the typed ports change nothing") {
    checkScenario("typed-card-wins");
}
TEST_CASE("golden typed-card-wins-untyped: its control") { checkScenario("typed-card-wins-untyped"); }
TEST_CASE("golden typed-undecodable: files that do not decode -- the typed ports decide") {
    checkScenario("typed-undecodable");
}
TEST_CASE("golden typed-undecodable-flip: ... and FRONT FLIPPED against them") {
    checkScenario("typed-undecodable-flip");
}
TEST_CASE("golden typed-tie: a half-flipped tie -- CANNOT TELL") { checkScenario("typed-tie"); }
TEST_CASE("golden typed-nocommon: no port in common -- CANNOT TELL") { checkScenario("typed-nocommon"); }
TEST_CASE("golden typed-watch-typo: a declared drive port the typed ports drive with") {
    checkScenario("typed-watch-typo");
}
TEST_CASE("golden typed-partial: SAME, but some typed ports differ") { checkScenario("typed-partial"); }
TEST_CASE("golden empty-watch-untyped: the discard words follow the card") {
    checkScenario("empty-watch-untyped");
}
TEST_CASE("golden undecodable-untyped: both files undecodable, no typed ports -- NOT DONE") {
    checkScenario("undecodable-untyped");
}
TEST_CASE("golden bakonly-garbage-untyped: only the backup, undecodable") {
    checkScenario("bakonly-garbage-untyped");
}
TEST_CASE("golden bakonly-garbage-typed: only the backup, undecodable, typed ports") {
    checkScenario("bakonly-garbage-typed");
}
TEST_CASE("golden emptyfile-untyped: an EMPTY record file") { checkScenario("emptyfile-untyped"); }
TEST_CASE("golden nocard-watch-again: no card, a declared drive port") { checkScenario("nocard-watch-again"); }
TEST_CASE("golden typed-nocard: no card, typed ports -- no check") { checkScenario("typed-nocard"); }

TEST_CASE("golden: typed-card-wins IS its untyped control -- serial, screen and card") {
    checkSameRun("typed-card-wins", "typed-card-wins-untyped");
}

TEST_CASE("golden: typed-nocard IS nocard -- no card, no check, typed or not") {
    checkSameRun("typed-nocard", "nocard");
}

TEST_CASE("golden: every scenario file in the goldens directory is one of the 34") {
    // A golden left behind by a renamed scenario would never be compared again: the directory
    // holds exactly the 34 scenarios' three files each.
    const char* names[] = {"one", "clean", "slip18", "curved", "nocard", "noimu", "disable",
                           "flip-again", "flip-new", "flip-disable", "swap", "watch",
                           "watch-typo-again", "watch-typo-save", "undeclared", "typed-first",
                           "typed-flip-new", "typed-flip-again", "typed-flip-disable",
                           "typed-card-wins", "typed-card-wins-untyped", "typed-undecodable",
                           "typed-undecodable-flip", "typed-tie", "typed-nocommon",
                           "typed-watch-typo", "typed-partial", "empty-watch-untyped",
                           "undecodable-untyped", "bakonly-garbage-untyped",
                           "bakonly-garbage-typed", "emptyfile-untyped", "nocard-watch-again",
                           "typed-nocard"};
    static_assert(sizeof names / sizeof names[0] == 34);
    std::vector<std::string> want;
    for (const char* n : names) {
        for (const char* kind : {".card.txt", ".screen.log", ".serial.log"}) {
            want.push_back(std::string{n} + kind);
        }
    }
    std::sort(want.begin(), want.end());
    std::vector<std::string> have;
    for (const auto& e : fs::directory_iterator{std::string{SHULIB_SOURCE_DIR} + "/test/goldens/calibrate"}) {
        have.push_back(e.path().filename().string());
    }
    std::sort(have.begin(), have.end());
    CHECK(have == want);
}

TEST_CASE("golden: runCalibrate() composes the same five the rig does, the card at its default") {
    const std::string src = readSource("include/shulib/hal/pros/calibrate.hpp");
    const auto at = src.find("inline void runCalibrate(");
    REQUIRE(at != std::string::npos);
    const auto end = src.find("\n}\n", at);
    REQUIRE(end != std::string::npos);
    const std::string body = src.substr(at, end - at);
    // EXACTLY these lines, and nothing else (CAL1-A2b's review, 2026-10-30: a pin that only found
    // each line passed a body that ran the program twice): the five, the card at its DEFAULT root
    // (the brain's), the program over them, one run.
    CHECK(body == "inline void runCalibrate(const calibration::CalibrateFacts& facts,\n"
                  "                         const char* buildStamp = __DATE__ \" \" __TIME__) {\n"
                  "    ProsDeviceRegistry registry;\n"
                  "    ProsDeviceProvider devices{registry};\n"
                  "    ProsFileStore card;\n"
                  "    ProsBrainScreen screen;\n"
                  "    ProsCalibrateIo io;\n"
                  "    calibration::CalibrateProgram program{facts, {devices, registry, card, screen, io}, "
                  "buildStamp};\n"
                  "    program.run();");
    // and the rig builds the same five, the card's root its one difference
    const std::string rig = readSource("test/calibrate_golden_rig.hpp");
    CHECK(rig.find("shulib::hal::pros::ProsDeviceProvider devices{registry};") != std::string::npos);
    CHECK(rig.find("shulib::hal::pros::ProsFileStore card{root.c_str()};") != std::string::npos);
    CHECK(rig.find("facts, {devices, registry, card, screen, io}, kStamp};") != std::string::npos);
}
