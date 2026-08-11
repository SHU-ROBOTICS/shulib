// Tests for shulib::units — runtime arithmetic correctness AND compile-time
// dimensional safety. The static_asserts ARE the negative tests: code that must
// NOT be well-formed (adding inches to seconds) is proven ill-formed at compile
// time, in-build, with no separate failing-compile harness.

#include <concepts>
#include <type_traits>

#include "doctest.h"
#include "shulib/math/angle.hpp"
#include "shulib/units/literals.hpp"
#include "shulib/units/quantity.hpp"

using namespace shulib::units;
using namespace shulib::units::literals;

// ----- compile-time NEGATIVE tests: the type system must reject these -----
namespace {
template <class A, class B>
concept Addable = requires(A a, B b) { a + b; };

// same dimension adds; mixed dimensions do NOT compile
static_assert(Addable<Length, Length>);
static_assert(!Addable<Length, Time>);          // inches + seconds  -> rejected
static_assert(!Addable<Length, Velocity>);      // inches + in/s     -> rejected
static_assert(!Addable<Voltage, Length>);       // volts  + inches   -> rejected

// multiply / divide compute the right derived dimension
static_assert(std::same_as<decltype(Length{} / Time{}), Velocity>);
static_assert(std::same_as<decltype(Velocity{} / Time{}), Acceleration>);
static_assert(std::same_as<decltype(Velocity{} * Time{}), Length>);
static_assert(std::same_as<decltype(Length{} / Length{}), Number>);   // ratio is dimensionless

// the 2026-06-19 electric-current dimension (I) composes and is type-distinct
static_assert(std::same_as<decltype(Voltage{} * Current{}), Power>);  // V·A = W
static_assert(std::same_as<decltype(Power{} / Voltage{}), Current>);  // W/V = A
static_assert(!Addable<Current, Voltage>);                            // amps + volts  -> rejected
static_assert(!Addable<Current, Number>);                             // amps + scalar -> rejected

// a bare double cannot become a Length implicitly (ctor is explicit)
static_assert(!std::is_convertible_v<double, Length>);
}  // namespace

TEST_CASE("literals construct canonical values") {
    CHECK((24.0_in).value()  == doctest::Approx(24.0));
    CHECK((1.0_tile).value() == doctest::Approx(24.0));   // a field tile is 24"
    CHECK((2_tile).value()   == doctest::Approx(48.0));   // integer literal form
    CHECK((2.0_s).value()    == doctest::Approx(2.0));
    CHECK((12.0_volt).value()== doctest::Approx(12.0));
}

TEST_CASE("electric-current dimension carries amperes and composes to power") {
    const Current i{2.5};
    CHECK(i.value() == doctest::Approx(2.5));
    const Power p = Voltage{12.0} * i;  // 12 V · 2.5 A = 30 W
    CHECK(p.value() == doctest::Approx(30.0));
}

TEST_CASE("_ms converts to seconds at the boundary (the locked time unit)") {
    CHECK((500.0_ms).value() == doctest::Approx(0.5));    // 500 ms = 0.5 s, NOT 500
    CHECK((20_ms).value()    == doctest::Approx(0.02));
    CHECK((1000.0_ms == 1.0_s));                          // same dimension, equal value
}

TEST_CASE("same-dimension arithmetic") {
    CHECK((2.0_in + 3.0_in).value() == doctest::Approx(5.0));
    CHECK((5.0_in - 8.0_in).value() == doctest::Approx(-3.0));
    CHECK((3.0_in * 2.0).value()    == doctest::Approx(6.0));
    CHECK((10.0_in / 4.0).value()   == doctest::Approx(2.5));
    CHECK(2.0_in < 3.0_in);
    CHECK(3.0_in >= 3.0_in);
}

TEST_CASE("dimensioned division yields the right value and type") {
    const Velocity v = 10.0_in / 2.0_s;
    CHECK(v.value() == doctest::Approx(5.0));             // 5 in/s
    const Acceleration a = v / 0.5_s;
    CHECK(a.value() == doctest::Approx(10.0));            // 10 in/s^2
    const Length d = v * 4.0_s;
    CHECK(d.value() == doctest::Approx(20.0));            // 20 in
}

TEST_CASE("_deg / _rad build a wrapping Angle, not a bare Quantity") {
    static_assert(std::same_as<decltype(90.0_deg), ::shulib::math::Angle>);
    CHECK((90.0_deg).degrees()  == doctest::Approx(90.0));
    CHECK((540.0_deg).degrees() == doctest::Approx(180.0));   // wrapped, proving it's an Angle
}
