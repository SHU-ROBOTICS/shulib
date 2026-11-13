// fake_file_store_test.cpp — hal::fake::FakeFileStore does exactly what its header says (chunk
// SAFE-SAVE Part A). Every oracle in record_store_test.cpp is only as good as this fake: a crash
// that quietly leaves the new bytes, a "short write" that keeps everything, or a refusal after a
// crash that still lets a read through would make the crash-point test pass against a save that
// loses records. So each fault is pinned here, byte for byte, against a literal expectation.

#include "doctest.h"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_file_store.hpp"
#include "shulib/hal/file_store.hpp"

namespace {

using shulib::PreconditionError;
using shulib::hal::ReplaceStep;
using shulib::hal::StoreRead;
using shulib::hal::StoreReplace;
using shulib::hal::fake::AfterCrash;
using shulib::hal::fake::CrashLeave;
using shulib::hal::fake::FakeFileFault;
using shulib::hal::fake::FakeFileOp;
using shulib::hal::fake::FakeFileStore;

using Bytes = std::vector<std::byte>;

Bytes bytes(std::initializer_list<int> v) {
    Bytes b;
    for (const int x : v) b.push_back(static_cast<std::byte>(x));
    return b;
}

const Bytes kOld = bytes({1, 2, 3, 4, 5, 6});
const Bytes kNewer = bytes({9, 8, 7, 6});

/// read `name` whole into a vector (nullopt when not opened).
std::optional<Bytes> readBack(FakeFileStore& s, const char* name) {
    std::byte buf[64]{};
    const StoreRead r = s.read(name, buf);
    if (!r.opened) return std::nullopt;
    return Bytes(buf, buf + r.count);
}

}  // namespace

TEST_CASE("FakeFileStore: a clean replace and read round-trip, name the path under the display root, "
          "and log both operations") {
    FakeFileStore s{"/usd/"};
    const StoreReplace w = s.replace("a.bin", kOld);
    CHECK(w.failed == ReplaceStep::None);
    CHECK(std::string{w.path} == "/usd/a.bin");
    std::byte buf[64]{};
    const StoreRead r = s.read("a.bin", buf);
    CHECK(r.opened);
    CHECK(r.count == kOld.size());
    CHECK(std::string{r.path} == "/usd/a.bin");
    CHECK(Bytes(buf, buf + r.count) == kOld);
    REQUIRE(s.log().size() == 2);
    CHECK(s.log()[0].op == FakeFileOp::Replace);
    CHECK(s.log()[0].bytes == kOld);
    CHECK(s.log()[1].op == FakeFileOp::Read);
    CHECK(s.log()[1].bytes == kOld);
    CHECK(s.operations() == 2);
    // A replace TRUNCATES: a shorter file replaces a longer one exactly.
    (void)s.replace("a.bin", kNewer);
    CHECK(s.file("a.bin") == std::optional<Bytes>{kNewer});
    // A read into a smaller buffer hands back that many bytes, from the start.
    std::byte two[2]{};
    CHECK(s.read("a.bin", two).count == 2);
    CHECK(two[0] == kNewer[0]);
}

TEST_CASE("FakeFileStore: a missing file is not opened; no card opens nothing and creates nothing") {
    FakeFileStore s{"/usd/"};
    CHECK_FALSE(readBack(s, "none.bin").has_value());
    s.setCardPresent(false);
    CHECK_FALSE(s.cardPresent());
    CHECK(s.replace("new.bin", kOld).failed == ReplaceStep::NoCard);
    s.setCardPresent(true);
    CHECK_FALSE(s.file("new.bin").has_value());
}

TEST_CASE("FakeFileStore: each REPORTED fault does what the header says, on the planned operation "
          "only") {
    FakeFileStore s{"/usd/"};
    s.setFile("f", kOld);
    s.plan({0, FakeFileFault::OpenRefused});
    CHECK(s.replace("f", kNewer).failed == ReplaceStep::Open);
    CHECK(s.file("f") == std::optional<Bytes>{kOld});  // untouched

    s.plan({1, FakeFileFault::ShortWrite, 2});
    CHECK(s.replace("f", kNewer).failed == ReplaceStep::Write);
    CHECK(s.file("f") == std::optional<Bytes>{bytes({9, 8})});  // the prefix is KEPT

    s.plan({2, FakeFileFault::FlushFalse});
    CHECK(s.replace("f", kNewer).failed == ReplaceStep::Flush);
    CHECK(s.file("f") == std::optional<Bytes>{kNewer});  // every byte kept

    s.plan({3, FakeFileFault::ShortRead, 3});
    CHECK(readBack(s, "f") == std::optional<Bytes>{bytes({9, 8, 7})});
    CHECK(s.file("f") == std::optional<Bytes>{kNewer});  // a read changes no file

    s.plan({4, FakeFileFault::OpenRefused});
    CHECK_FALSE(readBack(s, "f").has_value());
    CHECK(readBack(s, "f") == std::optional<Bytes>{kNewer});  // operation 5: no fault
}

TEST_CASE("FakeFileStore: each SILENT fault reports nothing and changes what the card holds (or hands "
          "back)") {
    FakeFileStore s{"/usd/"};
    s.setFile("f", kOld);
    s.plan({0, FakeFileFault::SilentCut, 1});
    CHECK(s.replace("f", kNewer).failed == ReplaceStep::None);
    CHECK(s.file("f") == std::optional<Bytes>{bytes({9})});

    s.plan({1, FakeFileFault::SilentFlip, 2});
    CHECK(s.replace("f", kNewer).failed == ReplaceStep::None);
    CHECK(s.file("f") == std::optional<Bytes>{bytes({9, 8, 7 ^ 0xFF, 6})});

    s.plan({2, FakeFileFault::SilentReadFlip, 0});
    CHECK(readBack(s, "f") == std::optional<Bytes>{bytes({9 ^ 0xFF, 8, 7 ^ 0xFF, 6})});
    CHECK(s.file("f") == std::optional<Bytes>{bytes({9, 8, 7 ^ 0xFF, 6})});

    s.plan({3, FakeFileFault::SilentNoop});
    CHECK(s.replace("f", kOld).failed == ReplaceStep::None);
    CHECK(s.file("f") == std::optional<Bytes>{bytes({9, 8, 7 ^ 0xFF, 6})});  // unchanged
    s.plan({4, FakeFileFault::SilentNoop, 0});
    CHECK(s.replace("g", kOld).failed == ReplaceStep::None);
    CHECK(s.file("g") == std::optional<Bytes>{Bytes{}});  // absent → created EMPTY
    s.plan({5, FakeFileFault::SilentNoop, 1});
    CHECK(s.replace("h", kOld).failed == ReplaceStep::None);
    CHECK_FALSE(s.file("h").has_value());  // absent → stays absent
}

TEST_CASE("FakeFileStore: a CRASHED replace leaves each state literally, reports Write, and every "
          "later operation is refused until reboot()") {
    struct Case {
        CrashLeave leave;
        std::size_t j;
        std::optional<Bytes> left;
    };
    // old = 1 2 3 4 5 6, new = 9 8 7 6
    const Case cases[] = {
        {CrashLeave::Absent, 0, std::nullopt},
        {CrashLeave::Empty, 0, Bytes{}},
        {CrashLeave::Prefix, 3, bytes({9, 8, 7})},
        {CrashLeave::AllNew, 0, kNewer},
        {CrashLeave::Torn, 2, bytes({9, 8, 3, 4, 5, 6})},  // the OLD length: new to 2, old after
        {CrashLeave::Torn, 4, bytes({9, 8, 7, 6, 5, 6})},
    };
    for (const Case& c : cases) {
        for (const AfterCrash after : {AfterCrash::CardGone, AfterCrash::CardSilent}) {
            CAPTURE(static_cast<int>(c.leave));
            CAPTURE(c.j);
            FakeFileStore s{"/usd/"};
            s.setFile("f", kOld);
            s.setFile("other", kOld);
            s.plan({0, FakeFileFault::Crash, c.j, c.leave, after});
            CHECK(s.replace("f", kNewer).failed == ReplaceStep::Write);
            CHECK(s.crashed());
            CHECK(s.file("f") == c.left);
            CHECK(s.file("other") == std::optional<Bytes>{kOld});  // HA-166 (4): the model
            CHECK(s.cardPresent() == (after == AfterCrash::CardSilent));
            CHECK_FALSE(readBack(s, "other").has_value());
            CHECK(s.replace("other", kNewer).failed
                  == (after == AfterCrash::CardGone ? ReplaceStep::NoCard : ReplaceStep::Open));
            CHECK(s.file("other") == std::optional<Bytes>{kOld});  // a refused replace changes nothing
            s.reboot();
            CHECK_FALSE(s.crashed());
            CHECK(s.cardPresent());
            CHECK(s.log().empty());
            CHECK(s.operations() == 0);
            CHECK(s.file("f") == c.left);  // reboot keeps the files as the crash left them
            CHECK(readBack(s, "other") == std::optional<Bytes>{kOld});
        }
    }
    // A torn write over a SHORTER old file is just the new prefix.
    FakeFileStore s{"/usd/"};
    s.setFile("f", bytes({1}));
    s.plan({0, FakeFileFault::Crash, 3, CrashLeave::Torn, AfterCrash::CardGone});
    (void)s.replace("f", kNewer);
    CHECK(s.file("f") == std::optional<Bytes>{bytes({9, 8, 7})});
}

TEST_CASE("FakeFileStore: a CRASHED read opens nothing and changes no file") {
    FakeFileStore s{"/usd/"};
    s.setFile("f", kOld);
    s.plan({0, FakeFileFault::Crash, 0, CrashLeave::Absent, AfterCrash::CardGone});
    CHECK_FALSE(readBack(s, "f").has_value());
    CHECK(s.crashed());
    CHECK(s.file("f") == std::optional<Bytes>{kOld});
}

TEST_CASE("FakeFileStore: names must be BARE, exactly as the PROS adapters demand") {
    // BUG CAUGHT: a fake more forgiving than the adapters, so a library bug that pre-prefixes
    // "/usd/" passes on the host and fails on the brain.
    FakeFileStore s{"/usd/"};
    std::byte buf[4]{};
    CHECK_THROWS_AS((void)s.read("/usd/f", buf), PreconditionError);
    CHECK_THROWS_AS((void)s.replace("/f", kOld), PreconditionError);
    CHECK_THROWS_AS((void)s.read("", buf), PreconditionError);
    CHECK_THROWS_AS((void)s.replace(nullptr, kOld), PreconditionError);
    CHECK(s.operations() == 0);
}
