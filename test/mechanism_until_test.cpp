// mechanism/until.hpp — a step's done-conditions (chunk MECH-A2, the MECH brief's §1.6).
//
// The algebra a step reads: which bits, which time, whose sensor. Each case tries to break one rule:
// two times keep the EARLIER (an `|` that kept the left one would let a long time hide a short one);
// two different NAMED sensors in one condition are FLAGGED (a stored name can say only one — a
// silently dropped sensor is the failure); the step's OWN sensor beside a name is KEPT as a fact
// (`readsOwnSensorToo()`), because only the step knows whether the name is its own mechanism's (the
// MECH-A2 review: `seesPiece | stopsSeeingPiece(intake)` on the intake is ONE sensor); a declaration
// with no `.name` is never read as "this mechanism"; `orAfter` adds a time to a sensor condition
// without losing it; the words reach a robot file's namespace. The build-time half
// (`seesPiece(arm)`, `seesPiece(nameless)` refused by name) is in test/compile_refusals/.

#include "doctest.h"

#include <string_view>

#include "mechanism_fixtures.hpp"
#include "shulib/mechanism/until.hpp"
#include "shulib/units/literals.hpp"

namespace {
namespace mech = shulib::mechanism;
using mech::DoneBy;
using mech::Until;
using shulib::units::Time;
using namespace shulib::units::literals;

namespace lists {
using namespace shulib::mechanism::words;
/// A declaration written WITHOUT `.name` — on no robot's list (A1 refuses it at wire()), but a hand-made
/// `Until::sees(...)` can still name it.
inline constexpr Power nameless{.motors = {Forward}, .cartridge = Blue, .pieceSensor = opticalSensor(0.5), .safe = Coast};
}  // namespace lists
}  // namespace

TEST_CASE("MECH-A2 until: each single condition sets exactly its bit, its time and its sensor") {
    const Until t = Until::time(Time{1.5});
    CHECK(t.has(Until::kTime));
    CHECK_FALSE(t.has(Until::kStalled));
    CHECK_FALSE(t.has(Until::kSees));
    CHECK_FALSE(t.has(Until::kStops));
    CHECK(t.seconds() == doctest::Approx(1.5));
    CHECK(t.sensor() == Until::kSelf);
    CHECK_FALSE(t.conflicting());

    const Until st = Until::stalled();
    CHECK(st.has(Until::kStalled));
    CHECK_FALSE(st.has(Until::kTime));
    CHECK(st.seconds() == 0.0);

    CHECK(Until::sees().has(Until::kSees));
    CHECK_FALSE(Until::sees().has(Until::kStops));
    CHECK(Until::stopsSeeing().has(Until::kStops));
    CHECK_FALSE(Until::stopsSeeing().has(Until::kSees));

    const Until b = Until::bound();
    CHECK_FALSE(b.has(Until::kTime));
    CHECK_FALSE(b.has(Until::kStalled));
    CHECK_FALSE(b.has(Until::kSees));
    CHECK_FALSE(b.has(Until::kStops));

    // ANOTHER mechanism's sensor is stored by NAME (P12: one address per translation unit, at most).
    const Until other = Until::sees(mech_fixture::robot_y::grip);
    REQUIRE(other.sensor() != nullptr);
    CHECK(std::string_view{other.sensor()} == "grip");
    CHECK(std::string_view{Until::stopsSeeing(mech_fixture::robot_x::intake).sensor()} == "intake");
    // The bit values are the brief's verbatim (2 is left free for "reached", a position step's own).
    CHECK(Until::kTime == 1);
    CHECK(Until::kStalled == 4);
    CHECK(Until::kSees == 8);
    CHECK(Until::kStops == 16);
}

TEST_CASE("MECH-A2 until: | joins conditions; two times keep the EARLIER, whichever side it is on") {
    const Until a = Until::time(Time{2.0}) | Until::time(Time{0.5});
    CHECK(a.seconds() == doctest::Approx(0.5));
    const Until b = Until::time(Time{0.5}) | Until::time(Time{2.0});
    CHECK(b.seconds() == doctest::Approx(0.5));
    // A time on ONE side only is kept from that side.
    CHECK((Until::sees() | Until::time(Time{3.0})).seconds() == doctest::Approx(3.0));
    CHECK((Until::time(Time{3.0}) | Until::stalled()).seconds() == doctest::Approx(3.0));
    // Bits OR.
    const Until c = Until::sees() | Until::stalled();
    CHECK(c.has(Until::kSees));
    CHECK(c.has(Until::kStalled));
    CHECK_FALSE(c.has(Until::kTime));
}

TEST_CASE("MECH-A2 until: orAfter keeps the sensor condition and adds the time (seesPiece.orAfter(2.5_s))") {
    const Until u = mech::seesPiece.orAfter(2.5_s);
    CHECK(u.has(Until::kSees));
    CHECK(u.has(Until::kTime));
    CHECK(u.seconds() == doctest::Approx(2.5));
    CHECK(u.sensor() == Until::kSelf);
    CHECK_FALSE(u.conflicting());
    // On another mechanism's condition: the name survives.
    const Until v = Until::stopsSeeing(mech_fixture::robot_y::grip).orAfter(1_s);
    REQUIRE(v.sensor() != nullptr);
    CHECK(std::string_view{v.sensor()} == "grip");
    CHECK(v.has(Until::kStops));
    // orAfter on a condition that already has a later time: the earlier wins.
    CHECK((Until::time(Time{5.0}) | Until::sees()).orAfter(Time{1.0}).seconds() == doctest::Approx(1.0));
}

TEST_CASE("MECH-A2 until: sensor conditions NAMING two different mechanisms are flagged, never silently merged") {
    // two others
    CHECK((Until::sees(mech_fixture::robot_y::grip) | Until::sees(mech_fixture::robot_y::roller)).conflicting());
    CHECK((Until::stopsSeeing(mech_fixture::robot_y::roller) | Until::sees(mech_fixture::robot_y::grip)).conflicting());
    // the same other twice (by content), and self twice: one sensor -> fine
    CHECK_FALSE((Until::sees(mech_fixture::robot_y::grip) | Until::stopsSeeing(mech_fixture::robot_y::grip)).conflicting());
    CHECK_FALSE((Until::sees(mech_fixture::robot_y::grip) | Until::stopsSeeing(mech_fixture::robot_y::grip)).readsOwnSensorToo());
    CHECK_FALSE((Until::sees() | Until::stopsSeeing()).conflicting());
    CHECK_FALSE((Until::sees() | Until::stopsSeeing()).readsOwnSensorToo());
    CHECK((Until::sees() | Until::stopsSeeing()).sensor() == Until::kSelf);
    // a time or a stall beside one sensor is not a second sensor
    CHECK_FALSE((Until::sees(mech_fixture::robot_y::grip) | Until::time(Time{1.0}) | Until::stalled()).conflicting());
    CHECK_FALSE((Until::sees(mech_fixture::robot_y::grip) | Until::time(Time{1.0}) | Until::stalled()).readsOwnSensorToo());
    // a conflict survives further joins, on either side
    const Until bad = Until::sees(mech_fixture::robot_y::grip) | Until::sees(mech_fixture::robot_y::roller);
    CHECK((bad | Until::time(Time{1.0})).conflicting());
    CHECK((Until::time(Time{1.0}) | bad).conflicting());
}

TEST_CASE("MECH-A2 until: the step's OWN sensor beside a NAME is kept as a fact, not refused -- one sensor if the name is the step's own mechanism (the step decides)") {
    // seesPiece | stopsSeeingPiece(intake): on the intake itself, ONE sensor (the MECH-A2 review: it
    // was refused as "two different mechanisms' piece sensors"); on any other mechanism, two.
    for (const Until u : {Until::sees() | Until::stopsSeeing(mech_fixture::robot_y::grip),
                          Until::stopsSeeing(mech_fixture::robot_y::grip) | Until::sees(),
                          Until::sees(mech_fixture::robot_y::grip) | Until::stopsSeeing()}) {
        CHECK_FALSE(u.conflicting());
        CHECK(u.readsOwnSensorToo());
        REQUIRE(u.sensor() != Until::kSelf);
        CHECK(std::string_view{u.sensor()} == "grip");  // the NAME survives, whichever side it was on
        CHECK(u.has(Until::kSees));
        CHECK(u.has(Until::kStops));
    }
    // ... and survives further joins
    const Until both = Until::sees() | Until::sees(mech_fixture::robot_y::grip);
    CHECK((both | Until::time(Time{1.0})).readsOwnSensorToo());
    CHECK((Until::time(Time{1.0}) | both).readsOwnSensorToo());
    CHECK((both | Until::sees()).readsOwnSensorToo());
    CHECK(std::string_view{(Until::time(Time{1.0}) | both).sensor()} == "grip");
    CHECK(both.orAfter(Time{1.0}).readsOwnSensorToo());
    // a THIRD sensor, another name: two names now, conflicting whatever the step
    CHECK((both | Until::sees(mech_fixture::robot_y::roller)).conflicting());
    CHECK((Until::sees(mech_fixture::robot_y::roller) | both).conflicting());
    // own sensor alone is never "own too"
    CHECK_FALSE(Until::sees().readsOwnSensorToo());
    CHECK_FALSE(Until::sees(mech_fixture::robot_y::grip).readsOwnSensorToo());
}

TEST_CASE("MECH-A2 until: a declaration with NO .name is stored as an EMPTY name -- never as 'this mechanism', so the step cannot read its own sensor in its place") {
    const Until u = Until::sees(lists::nameless);
    REQUIRE(u.sensor() != Until::kSelf);  // a null name stored as-is WOULD be kSelf
    CHECK(std::string_view{u.sensor()}.empty());
    const Until v = Until::stopsSeeing(lists::nameless);
    REQUIRE(v.sensor() != Until::kSelf);
    CHECK(std::string_view{v.sensor()}.empty());
    // beside the step's own sensor it is a name like any other (the step refuses it: no list has it)
    CHECK((Until::sees() | v).readsOwnSensorToo());
    // beside ANOTHER name: two names
    CHECK((u | Until::sees(mech_fixture::robot_y::grip)).conflicting());
}

TEST_CASE("MECH-A2 until: the words -- seesPiece / stopsSeeingPiece / stalls, and another mechanism's sensor at build") {
    using namespace mech_fixture::robot_y;  // a robot file's namespace: its `using namespace mechanism::words;` brings them
    constexpr Until own = seesPiece;
    static_assert(own.has(Until::kSees) && own.sensor() == Until::kSelf);
    constexpr Until gone = stopsSeeingPiece;
    static_assert(gone.has(Until::kStops));
    static_assert(stalls.has(Until::kStalled));
    // Another mechanism's: compiled because the grip declares a piece sensor (the refusal is the
    // harness's: a mechanism with none fails the build, named).
    constexpr Until g = seesPiece(grip);
    static_assert(g.has(Until::kSees));
    CHECK(std::string_view{g.sensor()} == "grip");
    constexpr Until r = stopsSeeingPiece(roller);
    CHECK(std::string_view{r.sensor()} == "roller");
    CHECK(r.has(Until::kStops));
}

TEST_CASE("MECH-A2 until: doneByWord -- every word a step's log line prints, and an out-of-range value") {
    CHECK(std::string_view{mech::doneByWord(DoneBy::None)} == "-");
    CHECK(std::string_view{mech::doneByWord(DoneBy::Time)} == "time");
    CHECK(std::string_view{mech::doneByWord(DoneBy::Reached)} == "reached");
    CHECK(std::string_view{mech::doneByWord(DoneBy::Stalled)} == "stalled");
    CHECK(std::string_view{mech::doneByWord(DoneBy::Sees)} == "sees a piece");
    CHECK(std::string_view{mech::doneByWord(DoneBy::StopsSeeing)} == "stopped seeing a piece");
    CHECK(std::string_view{mech::doneByWord(static_cast<DoneBy>(42))} == "?");
    CHECK(static_cast<int>(DoneBy::StopsSeeing) == 5);
}
