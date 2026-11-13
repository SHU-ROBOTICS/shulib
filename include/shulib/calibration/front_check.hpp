#pragma once
//
// front_check — did a new calibration put the robot's FRONT at the other end? (chunk CAL1,
// Part A1, 2026-10-08.)
//
// THE RULE (the team lead, 2026-10-06): the end that LEADS Calibrate's push is the front, and the
// front is never typed. A calibration record carries no "front" field — it carries each drive
// port's SIDE and SIGN, both relative to the push. So a recalibration pushed with the OTHER end
// leading gives EVERY port the other sign AND the other side (measured on the host test plant
// through today's "shulib Calibrate": LEFT -11 +12 -13 +14 -15 | RIGHT +16 -17 +18 -19 +20 became
// LEFT -16 +17 -18 +19 -20 | RIGHT +11 -12 +13 -14 +15). Every auton written for the old front
// would then drive backwards, and nothing downstream could tell. compareFronts() is the check.
//
// THE VERDICT, port by port, over the drive ports BOTH descriptions list (matched by |port|):
//   agree    — same side, same sign: the same front;
//   flipped  — other side AND other sign: the front moved to the other end;
//   signOnly — same side, other sign: that motor was re-mounted or re-wired (not a front change);
//   sideOnly — other side, same sign: a cable moved across (not a front change).
// FLIPPED when the flipped ports are a STRICT MAJORITY of the shared ones; SAME when the agreeing
// ones are; otherwise UNDETERMINED (no shared port, or no majority — a tie included) — never a
// guess. A port in only one of the two (a motor LEFT OUT, a cable moved to a new port) decides
// nothing and is counted. Signs alone would call a re-mounted motor a flip; sides alone would
// call a moved cable one; so both, per port.
//
// WHAT THE RECORDS CANNOT TELL: the LEFT and RIGHT drive cables swapped between the sides at the
// brain, with the right end leading, give the SAME record as the other end leading (on a
// mirror-mounted tank every swapped pair is mirror-signed — robot two's record, worked port by
// port; the host test holds the two records equal). So FLIPPED means "the earlier description
// calls the end that led this push its BACK", never "you pushed the wrong end" — only the
// operator knows which. A program that checks the front therefore saves nothing until the
// operator says (save as pushed, or run again), and it never turns a measurement to the old front:
// what is saved is always what was pushed.
//
// WHAT `earlier` IS: the card's calibration record (as the boot would choose it) — or, when the
// card holds no usable record, a description holding only the drive ports the team TYPED for its
// drive program (decided by the team lead, 2026-10-08). compareFronts() reads nothing but the two
// sides' signed ports, so either works unchanged: a typed table's entries carry each port's side
// and sign in the same convention as a record (the boot copies them unchanged into a
// description's sides), and "SAME as the typed ports" means the robot, driven from those ports,
// turns every motor the way this push measured. Neither description need be valid() — a typed
// one has no geometry — but a side whose count runs past hal::kMaxSmartPorts is not a side this
// check can read, and gives UNDETERMINED with every count 0 (it never reads past the array).
//
// PROS-free, allocation-free, constexpr, host-tested. NOTHING HERE HAS RUN ON A ROBOT.

#include <cstddef>
#include <cstdint>

#include "shulib/hal/drivetrain_description.hpp"

namespace shulib::calibration {

/// What compareFronts() concluded.
enum class FrontVerdict : std::uint8_t {
    Same,          ///< a strict majority of the shared ports agree by side and sign
    Flipped,       ///< a strict majority sit on the other side with the other sign
    Undetermined,  ///< no shared drive port, or no majority (a tie included)
};

/// The comparison, port by port (header). The four kinds of shared port add up to `shared`.
struct FrontCheck {
    FrontVerdict verdict = FrontVerdict::Undetermined;  ///< the decision
    std::uint8_t shared = 0;       ///< drive ports in both descriptions
    std::uint8_t agree = 0;        ///< same side, same sign
    std::uint8_t flipped = 0;      ///< other side, other sign
    std::uint8_t signOnly = 0;     ///< same side, other sign
    std::uint8_t sideOnly = 0;     ///< other side, same sign
    std::uint8_t onlyEarlier = 0;  ///< in the earlier description only
    std::uint8_t onlyNow = 0;      ///< in the new description only
};

/// The words for a verdict, as the serial log and the screen say them.
[[nodiscard]] constexpr const char* frontVerdictWord(FrontVerdict v) noexcept {
    switch (v) {
        case FrontVerdict::Same: return "SAME FRONT";
        case FrontVerdict::Flipped: return "FRONT FLIPPED";
        case FrontVerdict::Undetermined: return "CANNOT TELL";
    }
    return "?";
}

namespace detail {

/// True when both of `d`'s sides have a count compareFronts() can read (header).
[[nodiscard]] constexpr bool frontSidesReadable(const hal::DrivetrainDescription& d) noexcept {
    return static_cast<std::size_t>(d.left.count) <= hal::kMaxSmartPorts
           && static_cast<std::size_t>(d.right.count) <= hal::kMaxSmartPorts;
}

}  // namespace detail

/// Compare the drive ports of `earlier` (the card's record, or — with no usable record — the
/// typed drive ports) and `now` (this run's measurement) — header. Ports 1..kMaxSmartPorts are
/// compared; an entry outside that range is not a drive port to this check.
[[nodiscard]] constexpr FrontCheck compareFronts(const hal::DrivetrainDescription& earlier,
                                                 const hal::DrivetrainDescription& now) noexcept {
    FrontCheck c{};
    if (!detail::frontSidesReadable(earlier) || !detail::frontSidesReadable(now)) {
        return c;  // Undetermined, every count 0: never a read past a side's array
    }
    for (int p = 1; p <= static_cast<int>(hal::kMaxSmartPorts); ++p) {
        const int se = earlier.sideOf(p);
        const int sn = now.sideOf(p);
        if (se == 0 && sn == 0) {
            continue;
        }
        if (se == 0) {
            ++c.onlyNow;
            continue;
        }
        if (sn == 0) {
            ++c.onlyEarlier;
            continue;
        }
        ++c.shared;
        const bool sameSign = (earlier.signedPortOf(p) < 0) == (now.signedPortOf(p) < 0);
        const bool sameSide = se == sn;
        if (sameSide && sameSign) {
            ++c.agree;
        } else if (!sameSide && !sameSign) {
            ++c.flipped;
        } else if (sameSide) {
            ++c.signOnly;
        } else {
            ++c.sideOnly;
        }
    }
    if (2 * c.flipped > c.shared) {
        c.verdict = FrontVerdict::Flipped;
    } else if (2 * c.agree > c.shared) {
        c.verdict = FrontVerdict::Same;
    } else {
        c.verdict = FrontVerdict::Undetermined;
    }
    return c;
}

}  // namespace shulib::calibration
