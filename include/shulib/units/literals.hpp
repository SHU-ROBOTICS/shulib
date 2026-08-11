#pragma once
//
// User-defined literals for shulib units. Each converts to CANONICAL units at
// the point of writing, so the rest of the code never sees a raw unit again:
//   distance = 24_in;  field = 1_tile;  dt = 20_ms;  v = 12_volt;
//
// `_ms` divides by 1000 -> seconds, enforcing the "internal seconds, always"
// decision at the literal boundary. `_deg`/`_rad` build an (auto-wrapped)
// shulib::math::Angle, NOT a bare Quantity (decision §13 #3).
//
// Each literal has two forms: a floating (long double) and an integer
// (unsigned long long), so both `24_in` and `24.0_in` compile. The narrowing
// to double is made explicit (the build is -Wconversion -Werror).

#include "shulib/math/angle.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::units::literals {

// length
constexpr Length operator""_in(long double v) noexcept { return Length{static_cast<double>(v)}; }
constexpr Length operator""_in(unsigned long long v) noexcept { return Length{static_cast<double>(v)}; }
constexpr Length operator""_tile(long double v) noexcept { return Length{static_cast<double>(v) * 24.0}; }
constexpr Length operator""_tile(unsigned long long v) noexcept { return Length{static_cast<double>(v) * 24.0}; }

// time (canonical = seconds)
constexpr Time operator""_s(long double v) noexcept { return Time{static_cast<double>(v)}; }
constexpr Time operator""_s(unsigned long long v) noexcept { return Time{static_cast<double>(v)}; }
constexpr Time operator""_ms(long double v) noexcept { return Time{static_cast<double>(v) / 1000.0}; }
constexpr Time operator""_ms(unsigned long long v) noexcept { return Time{static_cast<double>(v) / 1000.0}; }

// voltage
constexpr Voltage operator""_volt(long double v) noexcept { return Voltage{static_cast<double>(v)}; }
constexpr Voltage operator""_volt(unsigned long long v) noexcept { return Voltage{static_cast<double>(v)}; }

// angle -> a wrapping Angle (decision §13 #3), so 90_deg is a heading, not a scalar
inline ::shulib::math::Angle operator""_deg(long double v) {
    return ::shulib::math::Angle::degrees(static_cast<double>(v));
}
inline ::shulib::math::Angle operator""_deg(unsigned long long v) {
    return ::shulib::math::Angle::degrees(static_cast<double>(v));
}
inline ::shulib::math::Angle operator""_rad(long double v) {
    return ::shulib::math::Angle::radians(static_cast<double>(v));
}
inline ::shulib::math::Angle operator""_rad(unsigned long long v) {
    return ::shulib::math::Angle::radians(static_cast<double>(v));
}

}  // namespace shulib::units::literals
