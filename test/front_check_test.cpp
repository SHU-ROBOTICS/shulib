// calibration::compareFronts — did a new calibration put the robot's FRONT at the other end?
// (chunk CAL1 Part A1, the brief's §2.A test 3.) NOTHING HERE HAS RUN ON A ROBOT.
//
// Every pair below is written OUT, port by port, with the counts and the verdict it must give typed
// in by hand — never computed by the code under test, and never by "flipping" one record into
// another (there is no such function: what is saved is always what was pushed). The pairs are the
// brief's §1.4 table (probed on the host as CAL1's P7): robot two's real 2026-09-25 record
// (test/boot_source_test.cpp's fixture: L -16 +17 -18 -20 | R +11 -12 +13 +15) against itself, the
// other end, its own typed table, and each way a record can differ without the front moving; the
// cable swap the records CANNOT tell from a flip; and (the team lead's D6 (a), 2026-10-08) robot
// two's TYPED table as `earlier` — the reference a first record is checked against when the card
// holds none — against the host test plant's front-led and other-end records. (F2 alone also
// DERIVES two records, from a model of robot two's motors and cabling — the test's own arithmetic,
// to check the typed other-end and cable-swap rows, never the library's.)
//
// Each case names the bug it catches; the mutations that must turn it red are in the brief
// (§2.A mutations 1-3) and in CAL1-PROGRESS.md (Part A1), each observed red on a fresh binary:
//   F1  the fourteen record pairs, each count and each verdict           (>= for >; signOnly as
//       flipped; signs only, sides ignored)
//   F2  the cable swap gives the SAME record as the other end leading — FLIPPED, and the two
//       records equal port for port: what the records cannot tell. DERIVED, not typed: a model of
//       robot two (its motors, their mounting, the port cabled to each — read off kRecord) gives
//       both records, and a robot NOT mirror-mounted gives two different ones
//   F3  D6 (a): robot two's typed table as `earlier` — SAME 10 of 10 against the plant's front-led
//       record, FLIPPED 10 of 10 against the other-end record
//   F4  the strict majority at its edges: 2 of 3 flipped is FLIPPED; 2 of 4 is not; 2 agree and 2
//       flipped of 5 (one re-mounted) is NEITHER — a plurality is not a majority
//   F5  invariants over every pair: the four kinds add up to `shared`; swapping the two
//       descriptions keeps the verdict, swaps onlyEarlier/onlyNow and keeps the rest; the order
//       of ports within a side changes nothing; a port is matched by |port|
//   F6  nothing to compare: two empty descriptions, an entry outside 1..21 — CANNOT TELL, counts 0
//   F7  a side whose count runs past the array is not read: CANNOT TELL with every count 0, proven
//       at COMPILE time (a constant evaluation that read past the array would not compile)
//   F8  the verdict words, exactly

#include "doctest.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

#include "shulib/calibration/front_check.hpp"
#include "shulib/hal/drivetrain_description.hpp"

namespace {

using shulib::calibration::compareFronts;
using shulib::calibration::FrontCheck;
using shulib::calibration::FrontVerdict;
using shulib::calibration::frontVerdictWord;
using shulib::hal::DrivetrainDescription;

/// A description holding only two sides of signed ports — what compareFronts() reads. Not valid()
/// (no IMU, no geometry), on purpose: the check must not need it (a typed one has none).
constexpr DrivetrainDescription sides(std::initializer_list<int> left,
                                      std::initializer_list<int> right) {
    DrivetrainDescription d{};
    std::size_t i = 0;
    for (const int p : left) {
        d.left.signedPorts[i++] = static_cast<std::int8_t>(p);
    }
    d.left.count = static_cast<std::uint8_t>(i);
    i = 0;
    for (const int p : right) {
        d.right.signedPorts[i++] = static_cast<std::int8_t>(p);
    }
    d.right.count = static_cast<std::uint8_t>(i);
    return d;
}

/// The counts a pair must give, typed in by hand.
struct Want {
    int shared, agree, flipped, signOnly, sideOnly, onlyEarlier, onlyNow;
    FrontVerdict verdict;
};

void checkPair(const char* what, const DrivetrainDescription& earlier,
               const DrivetrainDescription& now, const Want& w) {
    const FrontCheck c = compareFronts(earlier, now);
    INFO(what);
    CHECK(static_cast<int>(c.shared) == w.shared);
    CHECK(static_cast<int>(c.agree) == w.agree);
    CHECK(static_cast<int>(c.flipped) == w.flipped);
    CHECK(static_cast<int>(c.signOnly) == w.signOnly);
    CHECK(static_cast<int>(c.sideOnly) == w.sideOnly);
    CHECK(static_cast<int>(c.onlyEarlier) == w.onlyEarlier);
    CHECK(static_cast<int>(c.onlyNow) == w.onlyNow);
    CHECK(c.verdict == w.verdict);
}

// Robot two's REAL record, as the card held it on 2026-09-25 (the camera end led that push).
constexpr DrivetrainDescription kRecord = sides({-16, 17, -18, -20}, {11, -12, 13, 15});
// The same robot, the same wiring, pushed with the OTHER end leading: every port on the other
// side with the other sign — written out, port by port.
constexpr DrivetrainDescription kOtherEnd = sides({-11, 12, -13, -15}, {16, -17, 18, 20});
// Robot two's TYPED chassis table (src/robots/chassis_table.hpp kRobotTwoChassis: .left / .right),
// whose front is the 15/16 end.
constexpr DrivetrainDescription kTypedTable =
    sides({-11, 12, -13, 14, -15}, {20, -19, 18, -17, 16});
// The host test plant in robot two's shape, pushed with the end the typed table calls front (the
// brief's §0.5 golden, run 1), and with the other end leading (its run 2).
constexpr DrivetrainDescription kPlantFrontLed =
    sides({-11, 12, -13, 14, -15}, {16, -17, 18, -19, 20});
constexpr DrivetrainDescription kPlantOtherEnd =
    sides({-16, 17, -18, 19, -20}, {11, -12, 13, -14, 15});

/// Every pair of F1 (and F2's, F3's), for the invariants of F5.
struct Pair {
    const char* what;
    DrivetrainDescription earlier;
    DrivetrainDescription now;
    Want want;
};

std::vector<Pair> allPairs() {
    using V = FrontVerdict;
    return {
        {"the same record", kRecord, kRecord, {8, 8, 0, 0, 0, 0, 0, V::Same}},
        {"the other end led", kRecord, kOtherEnd, {8, 0, 8, 0, 0, 0, 0, V::Flipped}},
        {"robot two's 09-25 record vs its typed table", kRecord, kTypedTable,
         {8, 0, 8, 0, 0, 0, 2, V::Flipped}},
        {"a port re-cabled (13 -> 9, same side, same sign)", kRecord,
         sides({-16, 17, -18, -20}, {11, -12, 9, 15}), {7, 7, 0, 0, 0, 1, 1, V::Same}},
        {"a side short a motor (15 LEFT OUT)", kRecord, sides({-16, 17, -18, -20}, {11, -12, 13}),
         {7, 7, 0, 0, 0, 1, 0, V::Same}},
        {"flipped AND a side short (15 LEFT OUT)", kRecord,
         sides({-11, 12, -13}, {16, -17, 18, 20}), {7, 0, 7, 0, 0, 1, 0, V::Flipped}},
        {"one motor re-mounted (12's sign)", kRecord, sides({-16, 17, -18, -20}, {11, 12, 13, 15}),
         {8, 7, 0, 1, 0, 0, 0, V::Same}},
        {"one cable across sides (13 to the left, same sign)", kRecord,
         sides({-16, 17, -18, -20, 13}, {11, -12, 15}), {8, 7, 0, 0, 1, 0, 0, V::Same}},
        {"another robot's card (no drive port in common)", kRecord, sides({-1, 2}, {3, -4}),
         {0, 0, 0, 0, 0, 8, 4, V::Undetermined}},
        {"a tie (4 agree, 4 flipped)", kRecord,
         sides({-16, 17, -18, -20, -11, 12, -13, -15}, {9}), {8, 4, 4, 0, 0, 0, 1, V::Undetermined}},
        {"flipped, one moved across (12), one re-cabled (20 -> 9)", kRecord,
         sides({-11, -12, -13, -15}, {16, -17, 18, 9}), {7, 0, 6, 0, 1, 1, 1, V::Flipped}},
        {"3 of 8 flipped, 5 agree", kRecord, sides({-16, 17, -18, -11, 12}, {20, 13, 15}),
         {8, 5, 3, 0, 0, 0, 0, V::Same}},
        {"one motor a side, flipped", sides({-1}, {2}), sides({-2}, {1}),
         {2, 0, 2, 0, 0, 0, 0, V::Flipped}},
        // F2: robot two's LEFT and RIGHT drive bundles exchanged at the brain (ports 16 17 18 20
        // now drive the right side's motors and 11 12 13 15 the left's), the CAMERA end leading
        // as before. A mirror-mounted tank's swapped pair is mirror-signed, so each port now reads
        // the other side with the other sign — worked port by port: 16 drove left motor 1 (-),
        // now right motor 1 (+16); ... — exactly the other-end record.
        {"the LEFT and RIGHT drive cables swapped, the camera end leading", kRecord,
         sides({-11, 12, -13, -15}, {16, -17, 18, 20}), {8, 0, 8, 0, 0, 0, 0, V::Flipped}},
        // F3 (D6 (a)): the typed ports as the reference.
        {"typed table vs the plant's front-led record", kTypedTable, kPlantFrontLed,
         {10, 10, 0, 0, 0, 0, 0, V::Same}},
        {"typed table vs the plant's other-end record", kTypedTable, kPlantOtherEnd,
         {10, 0, 10, 0, 0, 0, 0, V::Flipped}},
    };
}

/// True when two descriptions list the same signed ports on the same sides (order aside).
bool sameSignedSides(const DrivetrainDescription& a, const DrivetrainDescription& b) {
    for (int p = 1; p <= static_cast<int>(shulib::hal::kMaxSmartPorts); ++p) {
        if (a.sideOf(p) != b.sideOf(p) || a.signedPortOf(p) != b.signedPortOf(p)) {
            return false;
        }
    }
    return true;
}

}  // namespace

TEST_CASE("FrontCheck F1: the fourteen record pairs give exactly the counts and verdicts typed "
          "in by hand") {
    // BUG CAUGHT: FLIPPED on >= (the tie pair turns FLIPPED); a re-mounted motor counted as
    // flipped (12's sign); a check on signs only (the re-mounted pair and the moved cable both
    // become "flipped"); a port in one record only counted as shared; a moved cable (13 to the
    // left) called a flip.
    const auto pairs = allPairs();
    REQUIRE(pairs.size() == 16);
    for (std::size_t i = 0; i < 14; ++i) {
        checkPair(pairs[i].what, pairs[i].earlier, pairs[i].now, pairs[i].want);
    }
}

namespace {

/// F2's model of a tank: its drive MOTORS, in left/right pairs (pair k = left motor k and right
/// motor k, the order the record lists them), each with the side it sits on, the sign its port
/// reads when the robot is pushed with the camera end leading (its mounting), and the port cabled
/// to it. What a calibration records is derived from it — never typed.
struct ModelMotor {
    int side;  ///< -1 left, +1 right
    int sign;  ///< -1 or +1, camera end leading
    int port;  ///< 1..21
};
struct TankModel {
    std::vector<ModelMotor> left;
    std::vector<ModelMotor> right;
};

/// The robot behind a record, read off it: each entry is one motor on its side, its sign, its port.
TankModel modelOf(const DrivetrainDescription& d) {
    TankModel m;
    for (std::size_t i = 0; i < d.left.count; ++i) {
        const int p = d.left.signedPorts[i];
        m.left.push_back({-1, p < 0 ? -1 : 1, p < 0 ? -p : p});
    }
    for (std::size_t i = 0; i < d.right.count; ++i) {
        const int p = d.right.signedPorts[i];
        m.right.push_back({+1, p < 0 ? -1 : 1, p < 0 ? -p : p});
    }
    return m;
}

/// What a push records on `m`. With the OTHER end leading the robot moves the other way through
/// the push, so every motor turns the other way (its sign flips) and the side the calibration
/// assigns it from the push swaps — the host test plant's measurement through today's
/// "shulib Calibrate" (front_check.hpp's header).
DrivetrainDescription recordOf(const TankModel& m, bool otherEndLeads) {
    std::vector<int> left;
    std::vector<int> right;
    for (const auto* motors : {&m.left, &m.right}) {
        for (const ModelMotor& mm : *motors) {
            const int side = otherEndLeads ? -mm.side : mm.side;
            const int sign = otherEndLeads ? -mm.sign : mm.sign;
            (side < 0 ? left : right).push_back(sign * mm.port);
        }
    }
    DrivetrainDescription d{};
    for (std::size_t i = 0; i < left.size(); ++i) {
        d.left.signedPorts[i] = static_cast<std::int8_t>(left[i]);
    }
    d.left.count = static_cast<std::uint8_t>(left.size());
    for (std::size_t i = 0; i < right.size(); ++i) {
        d.right.signedPorts[i] = static_cast<std::int8_t>(right[i]);
    }
    d.right.count = static_cast<std::uint8_t>(right.size());
    return d;
}

/// The LEFT and RIGHT drive bundles exchanged at the brain: the port that drove left motor k now
/// drives right motor k, and the other way round. The motors and their mounting do not change.
TankModel cablesSwapped(TankModel m) {
    REQUIRE(m.left.size() == m.right.size());
    for (std::size_t k = 0; k < m.left.size(); ++k) {
        std::swap(m.left[k].port, m.right[k].port);
    }
    return m;
}

}  // namespace

TEST_CASE("FrontCheck F2: a LEFT/RIGHT cable swap gives the same record as the other end leading "
          "-- the records cannot tell the two apart (derived from a model of the robot)") {
    // BUG CAUGHT: any later "fix" that claims FLIPPED means "you pushed the wrong end". The two
    // physical causes produce one record; a program that saved the old front on FLIPPED would
    // reverse every auton on a correctly pushed, re-cabled robot. This pins the fact the hold's
    // words are built on — derived from robot two's motors, not typed twice: the model is read
    // off the real record, both records come out of it, and the equality holds only because
    // robot two is MIRROR-mounted (each pair's two motors read opposite signs), which a robot
    // that is not gives a different answer for.
    const TankModel robotTwo = modelOf(kRecord);
    REQUIRE(robotTwo.left.size() == 4);
    REQUIRE(robotTwo.right.size() == 4);
    for (std::size_t k = 0; k < 4; ++k) {
        INFO("pair ", k);
        CHECK(robotTwo.left[k].sign == -robotTwo.right[k].sign);  // mirror-mounted, pair by pair
    }
    // The model is robot two: it gives back the real record, and its other-end push gives the
    // other-end record F1 types by hand.
    CHECK(sameSignedSides(recordOf(robotTwo, false), kRecord));
    CHECK(sameSignedSides(recordOf(robotTwo, true), kOtherEnd));
    // The cable swap, the camera end leading as before: the SAME record as the other end leading.
    const DrivetrainDescription swapped = recordOf(cablesSwapped(robotTwo), false);
    CHECK(sameSignedSides(swapped, recordOf(robotTwo, true)));
    CHECK(sameSignedSides(swapped, kOtherEnd));
    // ... and F1's typed swap row is that derived record.
    const auto pairs = allPairs();
    const Pair& swapRow = pairs[13];
    REQUIRE(std::string{swapRow.what}.find("swapped") != std::string::npos);
    CHECK(sameSignedSides(swapRow.now, swapped));
    const FrontCheck a = compareFronts(kRecord, swapped);
    const FrontCheck b = compareFronts(kRecord, kOtherEnd);
    CHECK(a.verdict == FrontVerdict::Flipped);
    CHECK(b.verdict == FrontVerdict::Flipped);
    CHECK(a.flipped == 8);
    CHECK(b.flipped == 8);

    // The equality is the robot's, not the test's: re-mount ONE left motor (pair 0 now reads the
    // same sign on both sides) and the cable swap's record differs from the other end's in that
    // pair's two ports — the records COULD then tell those two ports' causes apart.
    TankModel notMirrored = robotTwo;
    notMirrored.left[0].sign = robotTwo.right[0].sign;
    const DrivetrainDescription swappedNM = recordOf(cablesSwapped(notMirrored), false);
    const DrivetrainDescription otherEndNM = recordOf(notMirrored, true);
    CHECK_FALSE(sameSignedSides(swappedNM, otherEndNM));
    int differing = 0;
    for (int p = 1; p <= static_cast<int>(shulib::hal::kMaxSmartPorts); ++p) {
        differing += (swappedNM.sideOf(p) != otherEndNM.sideOf(p)
                      || swappedNM.signedPortOf(p) != otherEndNM.signedPortOf(p))
                         ? 1
                         : 0;
    }
    CHECK(differing == 2);  // pair 0's two ports, and only they
}

TEST_CASE("FrontCheck F3: (D6 (a)) robot two's typed drive ports as the reference -- SAME 10 of 10 "
          "against the front-led push, FLIPPED 10 of 10 against the other end") {
    // BUG CAUGHT: a check that needs a VALID description (a typed one has no IMU and no
    // geometry), or reads anything but the two sides, or treats the typed table's right side
    // (typed 20 -19 18 -17 16, measured 16 -17 18 -19 20) as ordered.
    REQUIRE_FALSE(kTypedTable.valid());
    const auto pairs = allPairs();
    checkPair(pairs[14].what, pairs[14].earlier, pairs[14].now, pairs[14].want);
    checkPair(pairs[15].what, pairs[15].earlier, pairs[15].now, pairs[15].want);
}

TEST_CASE("FrontCheck F4: the strict majority at its edges -- a plurality is not a majority") {
    // BUG CAUGHT: FLIPPED on >= or SAME on >= (2 of 4); "the larger of agree and flipped wins"
    // (2 agree, 2 flipped, 1 re-mounted: neither is MORE than half of 5, so neither decides);
    // a halved share with >= (`agree >= shared / 2`: an integer 5 / 2 is 2, so 2 agree would be a
    // "majority" of 5). (`agree > shared / 2` is NOT a bug — in integers it is the same test as
    // `2 * agree > shared` — and no case here can or should tell the two apart.)
    using V = FrontVerdict;
    // 2 of 3 flipped (port 3 agrees): FLIPPED.
    checkPair("2 of 3 flipped", sides({-1, 2}, {3}), sides({}, {1, -2, 3}),
              {3, 1, 2, 0, 0, 0, 0, V::Flipped});
    // 2 of 3 agree (port 3 flipped): SAME.
    checkPair("2 of 3 agree", sides({-1, 2}, {3}), sides({-1, 2, -3}, {}),
              {3, 2, 1, 0, 0, 0, 0, V::Same});
    // 1 agree, 1 flipped, 2 re-mounted of 4: CANNOT TELL.
    checkPair("1 agree, 1 flipped, 2 re-mounted", sides({-1, 2}, {3, 4}), sides({-1}, {-2, -3, -4}),
              {4, 1, 1, 2, 0, 0, 0, V::Undetermined});
    // 2 of 4 flipped, 2 agree: CANNOT TELL.
    checkPair("2 of 4 flipped, 2 agree", sides({-1, 2}, {3, -4}), sides({-1, 2, -3, 4}, {}),
              {4, 2, 2, 0, 0, 0, 0, V::Undetermined});
    // 2 agree, 2 flipped, 1 re-mounted of 5: neither is a strict majority.
    checkPair("2 agree, 2 flipped, 1 re-mounted", sides({-1, 2, 5}, {3, -4}),
              sides({-1, 2, -5, -3, 4}, {}), {5, 2, 2, 1, 0, 0, 0, V::Undetermined});
    // 3 agree, 2 flipped of 5: SAME (3 > 2.5).
    checkPair("3 agree, 2 flipped", sides({-1, 2, 5}, {3, -4}), sides({-1, 2, 5, -3, 4}, {}),
              {5, 3, 2, 0, 0, 0, 0, V::Same});
}

TEST_CASE("FrontCheck F5: invariants over every pair -- the kinds add up, the two sides of the "
          "comparison are symmetric, and the order of ports and the sign of a match do not matter") {
    // BUG CAUGHT: a port counted twice or not at all; onlyEarlier and onlyNow swapped; a verdict
    // that depends on which record is called earlier; a match on the SIGNED value (so -11 never
    // finds +11 and a flip reads as two one-only ports).
    for (const Pair& p : allPairs()) {
        INFO(p.what);
        const FrontCheck ab = compareFronts(p.earlier, p.now);
        const FrontCheck ba = compareFronts(p.now, p.earlier);
        CHECK(ab.agree + ab.flipped + ab.signOnly + ab.sideOnly == ab.shared);
        CHECK(ba.verdict == ab.verdict);
        CHECK(ba.shared == ab.shared);
        CHECK(ba.agree == ab.agree);
        CHECK(ba.flipped == ab.flipped);
        CHECK(ba.signOnly == ab.signOnly);
        CHECK(ba.sideOnly == ab.sideOnly);
        CHECK(ba.onlyEarlier == ab.onlyNow);
        CHECK(ba.onlyNow == ab.onlyEarlier);
        // Reverse the order of each side of `now`: nothing changes.
        DrivetrainDescription reversed = p.now;
        for (auto* side : {&reversed.left, &reversed.right}) {
            for (std::size_t i = 0, j = side->count; i + 1 < j; ++i, --j) {
                std::swap(side->signedPorts[i], side->signedPorts[j - 1]);
            }
        }
        const FrontCheck rv = compareFronts(p.earlier, reversed);
        CHECK(rv.verdict == ab.verdict);
        CHECK(rv.agree == ab.agree);
        CHECK(rv.flipped == ab.flipped);
        CHECK(rv.signOnly == ab.signOnly);
        CHECK(rv.sideOnly == ab.sideOnly);
    }
    // A flip is found only by |port|: the other end's -11 is the record's +11.
    const FrontCheck c = compareFronts(sides({}, {11}), sides({-11}, {}));
    CHECK(c.shared == 1);
    CHECK(c.flipped == 1);
    CHECK(c.onlyEarlier == 0);
    CHECK(c.onlyNow == 0);
}

TEST_CASE("FrontCheck F6: nothing to compare -- CANNOT TELL, never a guess") {
    // BUG CAUGHT: "no shared port" read as SAME (0 flipped is not more than 0, but neither is 0
    // agree); an entry outside 1..21 matched (a typed -22 or 0 would then decide a verdict).
    const FrontCheck empty = compareFronts(sides({}, {}), sides({}, {}));
    CHECK(empty.verdict == FrontVerdict::Undetermined);
    CHECK(empty.shared == 0);
    CHECK(empty.onlyEarlier == 0);
    CHECK(empty.onlyNow == 0);
    const FrontCheck oneSided = compareFronts(kRecord, sides({}, {}));
    CHECK(oneSided.verdict == FrontVerdict::Undetermined);
    CHECK(oneSided.onlyEarlier == 8);
    // Entries outside 1..21 are not drive ports to the check: 0, 22, -22 and -128 decide nothing.
    const FrontCheck outside = compareFronts(sides({0, -22}, {22, -128}), sides({0, 22}, {-22}));
    CHECK(outside.verdict == FrontVerdict::Undetermined);
    CHECK(outside.shared == 0);
    CHECK(outside.onlyEarlier == 0);
    CHECK(outside.onlyNow == 0);
    // ... and beside real ports they change nothing.
    const FrontCheck mixed = compareFronts(sides({-16, 17, -18, -20, 22}, {11, -12, 13, 15, 0}),
                                           kOtherEnd);
    CHECK(mixed.verdict == FrontVerdict::Flipped);
    CHECK(mixed.shared == 8);
    CHECK(mixed.flipped == 8);
    CHECK(mixed.onlyEarlier == 0);
}

namespace {
constexpr DrivetrainDescription unreadable() {
    DrivetrainDescription d = sides({-1, 2}, {3});
    d.left.count = static_cast<std::uint8_t>(shulib::hal::kMaxSmartPorts + 1);  // past the array
    return d;
}
// F7 at COMPILE time: a constant evaluation that read signedPorts[21] would not compile, so these
// lines prove the guard keeps every read inside the array.
static_assert(compareFronts(unreadable(), kRecord).verdict == FrontVerdict::Undetermined);
static_assert(compareFronts(kRecord, unreadable()).verdict == FrontVerdict::Undetermined);
static_assert(compareFronts(unreadable(), unreadable()).shared == 0);
// A side of exactly kMaxSmartPorts entries IS read (the guard is not off by one).
constexpr DrivetrainDescription fullLeft() {
    DrivetrainDescription d{};
    for (std::size_t i = 0; i < shulib::hal::kMaxSmartPorts; ++i) {
        d.left.signedPorts[i] = static_cast<std::int8_t>(i + 1);
    }
    d.left.count = static_cast<std::uint8_t>(shulib::hal::kMaxSmartPorts);
    return d;
}
static_assert(static_cast<std::size_t>(compareFronts(fullLeft(), fullLeft()).agree)
              == shulib::hal::kMaxSmartPorts);
static_assert(compareFronts(fullLeft(), fullLeft()).verdict == FrontVerdict::Same);
// And the table's own pairs hold at compile time too (a constexpr regression is a build failure).
static_assert(compareFronts(kRecord, kOtherEnd).verdict == FrontVerdict::Flipped);
static_assert(compareFronts(kTypedTable, kPlantFrontLed).agree == 10);
}  // namespace

TEST_CASE("FrontCheck F7: a side whose count runs past the array is not read -- CANNOT TELL, every "
          "count 0 (also proven at compile time above)") {
    // BUG CAUGHT: compareFronts trusting `count` (a corrupt or hand-built description with
    // count 22 would read past signedPorts — undefined behaviour on the robot).
    const FrontCheck c = compareFronts(unreadable(), kRecord);
    CHECK(c.verdict == FrontVerdict::Undetermined);
    CHECK(c.shared == 0);
    CHECK(c.agree == 0);
    CHECK(c.flipped == 0);
    CHECK(c.onlyEarlier == 0);
    CHECK(c.onlyNow == 0);
    const FrontCheck d = compareFronts(kRecord, unreadable());
    CHECK(d.verdict == FrontVerdict::Undetermined);
    CHECK(d.onlyEarlier == 0);
    DrivetrainDescription rightBad = kRecord;
    rightBad.right.count = 255;
    CHECK(compareFronts(rightBad, kRecord).verdict == FrontVerdict::Undetermined);
    CHECK(compareFronts(rightBad, kRecord).shared == 0);
    // A full side (21 entries) is read.
    CHECK(compareFronts(fullLeft(), fullLeft()).agree == 21);
}

TEST_CASE("FrontCheck F8: the verdict words, exactly") {
    // BUG CAUGHT: a reworded verdict (the serial log's `front check:` line and the held screen's
    // header read these words; a worksheet greps them).
    CHECK(std::string{frontVerdictWord(FrontVerdict::Same)} == "SAME FRONT");
    CHECK(std::string{frontVerdictWord(FrontVerdict::Flipped)} == "FRONT FLIPPED");
    CHECK(std::string{frontVerdictWord(FrontVerdict::Undetermined)} == "CANNOT TELL");
    CHECK(std::string{frontVerdictWord(static_cast<FrontVerdict>(7))} == "?");
    // A default FrontCheck decides nothing.
    const FrontCheck none{};
    CHECK(none.verdict == FrontVerdict::Undetermined);
}
