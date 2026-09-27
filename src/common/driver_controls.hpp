#pragma once
//
// driver_controls.hpp — the GENERIC half of a driver file (chunk T4 Part 1a, 2026-09-27): a
// driver's controls as plain data, the compile-time checks on a set of them, THE binder that turns
// them into the library's bindings on a robot::Robot, and THE rule for trusting a heading.
//
// ── Why this file exists ────────────────────────────────────────────────────────────
// A driver file (src/team/robot_two_driver.hpp is the first) is the team's driver code: what every
// button does, the feel, and the call into the library's loop. Everything a driver file would
// otherwise have to carry that is NOT "which button does what" lives here, written and tested ONCE,
// so a driver file stays short enough to read top to bottom and a second robot's driver file would
// copy only its own controls:
//   * HoldControl / ToggleControl — one control as one line of data;
//   * five constexpr predicates, ONE PROPERTY EACH, so a driver file's failing static_assert names
//     the real fault (a button used twice; a control on the re-zero button; a volts outside
//     (0, 12]; a control naming a mechanism the robot does not have, of the wrong kind, or twice;
//     a mechanism no control names);
//   * DriverBindings — resolves each control to the robot's mechanism BY NAME, refuses a robot the
//     controls are not for, and owns the bindings in static storage;
//   * driverHeading() — THE heading rule every driver-control program uses.
//
// ── The binder's rules (DriverBindings::bind) ───────────────────────────────────────
// Each control is resolved against the mechanisms the robot DECLARES —
// robot.config().mechanisms[0 .. mechanismCount()) — by NAME, never by index, so a table that lists
// its entries in another order still gets the right buttons on the right motors. Then:
//   * a name the robot does not declare is REFUSED — never read as "absent": a typo would
//     otherwise leave a pivot silently unbound;
//   * a name of the wrong KIND is REFUSED (a hold needs a motor mechanism, a toggle a pneumatic
//     one) — never skipped;
//   * a mechanism the robot declares that NO control names is REFUSED: the controls are for a
//     different robot (T3's test 1i refused the same thing by index);
//   * a mechanism the robot declares but did not BUILD (its motor unplugged, its 3-wire line
//     refused) is SKIPPED: no binding, its buttons do nothing, and the drive still runs;
//   * a toggle's initial level is the robot's CONFIG start level —
//     config::toLineValue(mechanisms[i].pneumaticStart), the level robot::Robot made the line at —
//     never what the line is commanded to now. So bind() may run before or after an autonomous
//     has moved the claw: the toggle still starts where the robot started, and the FIRST
//     driver-control entry in a run sends that level (the team lead's ruling, 2026-09-26: "starts
//     closed — whatever autonomous left it at");
//   * teleop::validBindings() runs LAST, over what was bound, against the loop's re-zero button —
//     the same check runDriverControl() makes as a PRECONDITION before its first tick, where a
//     refusal would be a throw inside driver control.
// A refusal returns false with its reason in `why` and leaves NOTHING bound. A binding's own
// constructor may THROW a PreconditionError (a volts outside (0, 12], a button byte that is not a
// button): a program calls bind() inside its boot's try, and its catch calls unbind() before it
// tears the robot down, so no binding outlives the mechanisms it points at.
//
// ── Storage ─────────────────────────────────────────────────────────────────────────
// A DriverBindings lives in a program's static storage, never on opcontrol()'s stack: PROS deletes
// the driver-control task when the field disables the robot and runs no destructor, and the loop
// holds these bindings by address. It is non-copyable and non-movable for the same reason. Its
// default constructor cannot throw and touches no hardware (empty optionals).
//
// ── The words ───────────────────────────────────────────────────────────────────────
// Every refusal is its OWN literal format with a precision on every %s: src/ compiles with
// -Wformat-nonliteral and -Wformat-truncation as errors, and a program shows the sentence on the
// brain's screen and in its log. Each sentence is pinned by a host test.
//
// PROS-FREE ON PURPOSE: the host suite runs all of it (test/driver_controls_test.cpp against the
// fake provider; test/robot_two_driver_test.cpp through the PROS shim with robot two's real
// table). It lives in src/, not include/shulib/, because it is program-side glue the team's driver
// files share — promoting it into the library is a later decision, not this file's.
//
// NO HARDWARE CLAIM: host-tested only. No driver has pressed a button through it, and nothing here
// has run on a robot.

#include <cstddef>
#include <cstdio>
#include <optional>
#include <span>
#include <string_view>

#include "robots/mechanism_table.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/teleop/mechanism_binding.hpp"
#include "shulib/teleop/teleop_loop.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench {

// ═══ THE CONTROLS, AS DATA ═══════════════════════════════════════════════════════════════

/// One motor mechanism driven hold-to-run: `forward` held drives it at +`volts`, `back` held at
/// -`volts`, neither (or both) puts it in its declared safe state once. Aggregate, so a driver file
/// writes one line per mechanism and a test can build a bad one by hand. `mechanism` is the name
/// the robot declares (the mechanism table's), matched exactly.
struct HoldControl {
    const char* mechanism;           ///< the declared mechanism's name, e.g. "claw pivot 6"
    hal::ControllerButton forward;   ///< held => +volts
    hal::ControllerButton back;      ///< held => -volts
    units::Voltage volts;            ///< the magnitude while a button is held, in (0, 12]
};

/// One pneumatic mechanism flipped by one button: each press opens it if it is closed and closes it
/// if it is open. Aggregate, as HoldControl.
struct ToggleControl {
    const char* mechanism;           ///< the declared mechanism's name, e.g. "claw"
    hal::ControllerButton button;    ///< each press flips it
};

// ═══ THE COMPILE-TIME CHECKS — one property each ════════════════════════════════════════
// Each is constexpr, so a driver file static_asserts it and `make` refuses a set that could not
// work, naming which rule broke. Each is also callable at run time (the host test breaks every one
// by hand). A robot with no hold (or no toggle) passes `{}` for that list: C++ has no empty `[]`
// array.

/// Two control names that are the same string. A null name is nobody's name.
constexpr bool controlNameIs(const char* a, const char* b) noexcept {
    return a != nullptr && b != nullptr && std::string_view{a} == std::string_view{b};
}

/// How many buttons a set of controls uses, counting each use: two per hold, one per toggle.
constexpr std::size_t controlButtonCount(std::span<const HoldControl> holds,
                                         std::span<const ToggleControl> toggles) noexcept {
    return 2U * holds.size() + toggles.size();
}

/// The `k`-th button use of the set: each hold's forward, then its back, in order, then each
/// toggle's button. `k` must be below controlButtonCount().
constexpr hal::ControllerButton controlButtonAt(std::span<const HoldControl> holds,
                                                std::span<const ToggleControl> toggles,
                                                std::size_t k) noexcept {
    if (k < 2U * holds.size()) {
        const HoldControl& h = holds[k / 2U];
        return k % 2U == 0U ? h.forward : h.back;
    }
    return toggles[k - 2U * holds.size()].button;
}

/// Check 1: no button does two jobs — no button appears twice across the holds and the toggles, a
/// hold whose forward is its own back included. Compared use by use, so it holds for any button
/// byte, not only the twelve named ones.
constexpr bool controlButtonsDistinct(std::span<const HoldControl> holds,
                                      std::span<const ToggleControl> toggles) noexcept {
    const std::size_t n = controlButtonCount(holds, toggles);
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            if (controlButtonAt(holds, toggles, i) == controlButtonAt(holds, toggles, j)) {
                return false;
            }
        }
    }
    return true;
}

/// Check 2: no control uses `reserved` — the loop's field-centric re-zero button (a re-zero on a
/// claw's button would re-zero the field frame every time the claw moved).
constexpr bool controlsAvoid(std::span<const HoldControl> holds,
                             std::span<const ToggleControl> toggles,
                             hal::ControllerButton reserved) noexcept {
    for (std::size_t k = 0; k < controlButtonCount(holds, toggles); ++k) {
        if (controlButtonAt(holds, toggles, k) == reserved) {
            return false;
        }
    }
    return true;
}

/// Check 3: every hold's volts is in (0, hal::kMaxMotorVoltage] — written as a negated conjunction,
/// so a NaN fails it (every comparison with NaN is false), and so does an infinity.
constexpr bool holdVoltsValid(std::span<const HoldControl> holds) noexcept {
    for (const HoldControl& h : holds) {
        const double v = h.volts.value();
        if (!(v > 0.0 && v <= hal::kMaxMotorVoltage.value())) {
            return false;
        }
    }
    return true;
}

/// True when `name` is the name of motor entry `k` of `t`, for some k.
constexpr bool tableHasMotorNamed(const MechanismTable& t, const char* name) noexcept {
    for (std::size_t k = 0; k < motorEntryCount(t); ++k) {
        if (controlNameIs(t.motors[k].name, name)) {
            return true;
        }
    }
    return false;
}

/// True when `name` is the name of pneumatic entry `k` of `t`, for some k.
constexpr bool tableHasPneumaticNamed(const MechanismTable& t, const char* name) noexcept {
    for (std::size_t k = 0; k < pneumaticEntryCount(t); ++k) {
        if (controlNameIs(t.pneumatics[k].name, name)) {
            return true;
        }
    }
    return false;
}

/// Check 4: every control names a mechanism `t` declares, of the right KIND — a hold a MOTOR entry,
/// a toggle a PNEUMATIC one — and no mechanism is named by two controls (of either kind).
constexpr bool controlsNameTable(std::span<const HoldControl> holds,
                                 std::span<const ToggleControl> toggles,
                                 const MechanismTable& t) noexcept {
    for (std::size_t i = 0; i < holds.size(); ++i) {
        if (!tableHasMotorNamed(t, holds[i].mechanism)) {
            return false;
        }
        for (std::size_t j = i + 1; j < holds.size(); ++j) {
            if (controlNameIs(holds[j].mechanism, holds[i].mechanism)) {
                return false;
            }
        }
        for (const ToggleControl& c : toggles) {
            if (controlNameIs(c.mechanism, holds[i].mechanism)) {
                return false;
            }
        }
    }
    for (std::size_t i = 0; i < toggles.size(); ++i) {
        if (!tableHasPneumaticNamed(t, toggles[i].mechanism)) {
            return false;
        }
        for (std::size_t j = i + 1; j < toggles.size(); ++j) {
            if (controlNameIs(toggles[j].mechanism, toggles[i].mechanism)) {
                return false;
            }
        }
    }
    return true;
}

/// Check 5: every mechanism `t` declares has a control of its kind — each motor entry a hold, each
/// pneumatic entry a toggle. (A mechanism nobody can work is a table and a driver file that
/// disagree about the robot.)
constexpr bool controlsCoverTable(std::span<const HoldControl> holds,
                                  std::span<const ToggleControl> toggles,
                                  const MechanismTable& t) noexcept {
    for (std::size_t k = 0; k < motorEntryCount(t); ++k) {
        bool named = false;
        for (const HoldControl& h : holds) {
            named = named || controlNameIs(h.mechanism, t.motors[k].name);
        }
        if (!named) {
            return false;
        }
    }
    for (std::size_t k = 0; k < pneumaticEntryCount(t); ++k) {
        bool named = false;
        for (const ToggleControl& c : toggles) {
            named = named || controlNameIs(c.mechanism, t.pneumatics[k].name);
        }
        if (!named) {
            return false;
        }
    }
    return true;
}

// ═══ THE HEADING RULE ════════════════════════════════════════════════════════════════════

/// THE heading field-centric turns the sticks by (driver_control.hpp's contract), for every
/// driver-control program — "shulib Teleop", the X-drive and every driver file call this one; there
/// is no second copy. The fused heading, TRUSTED only when all three hold:
///   * the estimate is live — the Localizer is past Uninitialized (motion.hpp's wait-for-live
///     contract; Degraded has four causes and says nothing specific about the heading, so it is not
///     the test);
///   * the IMU itself reports ready;
///   * no IMU_LOST fault has been raised this run.
/// Read before the tick's drive(), so it is the previous tick's estimate (10 ms old on a V5).
inline teleop::HeadingSample driverHeading(const robot::Robot& r) {
    const bool trusted =
        r.localizer().qualityClass() != localization::Localizer::Quality::Uninitialized
        && r.context().imu().isReady() && r.faults().raiseCount(diag::FaultCode::ImuLost) == 0;
    return teleop::HeadingSample{.radians = r.localizer().pose().heading().radians(),
                                 .trusted = trusted};
}

// ═══ THE BINDER ══════════════════════════════════════════════════════════════════════════

/// The bindings a set of controls makes on one robot::Robot (header, "The binder's rules").
///
/// Static storage only (header, "Storage"): non-copyable and non-movable, because the loop holds
/// the bindings inside it by address.
class DriverBindings {
public:
    /// The most controls of EACH kind: a robot declares at most config::kMaxMechanisms mechanisms,
    /// and each control names a different one.
    static constexpr std::size_t kCapacity = config::kMaxMechanisms;

    /// Nothing bound. Cannot throw and touches no hardware (empty optionals).
    DriverBindings() = default;
    /// Non-copyable: two copies would be two state machines on one mechanism.
    DriverBindings(const DriverBindings&) = delete;
    /// Non-movable: the loop holds the bindings by address.
    DriverBindings(DriverBindings&&) = delete;
    /// Non-copy-assignable, same reasoning.
    DriverBindings& operator=(const DriverBindings&) = delete;
    /// Non-move-assignable, same reasoning.
    DriverBindings& operator=(DriverBindings&&) = delete;
    ~DriverBindings() = default;

    /// Bind `holds` and `toggles` to `robot`'s mechanisms BY NAME (header, "The binder's rules"),
    /// then check what was bound against the loop's re-zero button `rezero`
    /// (`teleop::validBindings`). True when the set is bound and valid. False, with the reason in
    /// `why` (bounded to `cap`, NUL-terminated) and NOTHING bound, when the robot is not the one
    /// these controls are for. Replaces whatever an earlier call bound. A binding's constructor may
    /// THROW a PreconditionError — call it inside the boot's try, and unbind() in the catch.
    bool bind(robot::Robot& robot, std::span<const HoldControl> holds,
              std::span<const ToggleControl> toggles, hal::ControllerButton rezero, char* why,
              std::size_t cap) {
        unbind();
        if (why != nullptr && cap > 0) {
            why[0] = '\0';
        }
        if (holds.size() > kCapacity || toggles.size() > kCapacity) {
            if (why != nullptr && cap > 0) {
                std::snprintf(why, cap,
                              "%u hold and %u toggle controls: a robot declares at most %u "
                              "mechanisms, so there can be no more of either",
                              static_cast<unsigned>(holds.size()),
                              static_cast<unsigned>(toggles.size()),
                              static_cast<unsigned>(kCapacity));
            }
            return false;
        }
        const config::RobotConfig& cfg = robot.config();
        // ── 1. Resolve every control to a DECLARED mechanism of its kind. Nothing is built yet,
        //       so a refusal here constructs nothing.
        std::size_t holdAt[kCapacity]{};
        for (std::size_t h = 0; h < holds.size(); ++h) {
            const char* name = holds[h].mechanism;
            if (name == nullptr) {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap, "hold control %u names no mechanism (a null name)",
                                  static_cast<unsigned>(h));
                }
                return false;
            }
            const std::size_t i = declaredIndex(robot, name);
            if (i == robot.mechanismCount()) {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap,
                                  "a control names '%.23s', which the robot does not declare",
                                  name);
                }
                return false;
            }
            if (cfg.mechanisms[i].motorCount == 0) {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap,
                                  "'%.23s' is not a motor mechanism: a hold control needs one",
                                  name);
                }
                return false;
            }
            holdAt[h] = i;
        }
        std::size_t toggleAt[kCapacity]{};
        for (std::size_t t = 0; t < toggles.size(); ++t) {
            const char* name = toggles[t].mechanism;
            if (name == nullptr) {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap, "toggle control %u names no mechanism (a null name)",
                                  static_cast<unsigned>(t));
                }
                return false;
            }
            const std::size_t i = declaredIndex(robot, name);
            if (i == robot.mechanismCount()) {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap,
                                  "a control names '%.23s', which the robot does not declare",
                                  name);
                }
                return false;
            }
            if (cfg.mechanisms[i].pneumaticCount == 0) {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap,
                                  "'%.23s' is not a pneumatic mechanism: a toggle control needs "
                                  "one",
                                  name);
                }
                return false;
            }
            toggleAt[t] = i;
        }
        // ── 2. Every mechanism the robot DECLARES has a control of its kind.
        for (std::size_t i = 0; i < robot.mechanismCount(); ++i) {
            const config::MechanismConfig& m = cfg.mechanisms[i];
            const bool named = m.motorCount > 0 ? namesHold(holds, m) : namesToggle(toggles, m);
            if (!named) {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap, "the robot declares '%.23s', and no control names it",
                                  m.name);
                }
                return false;
            }
        }
        // ── 3. Build a binding for every mechanism the robot BUILT (an ABSENT one is skipped: its
        //       buttons do nothing). A binding's constructor may throw (header).
        for (std::size_t h = 0; h < holds.size(); ++h) {
            hal::MotorMechanism* m = robot.mechanismAt(holdAt[h]);
            if (m == nullptr) {
                continue;  // declared, not built: ABSENT
            }
            hold_[holdCount_].emplace(*m, holds[h].forward, holds[h].back, holds[h].volts);
            holdPtrs_[holdCount_] = &*hold_[holdCount_];
            ++holdCount_;
        }
        for (std::size_t t = 0; t < toggles.size(); ++t) {
            hal::PneumaticMechanism* p = robot.pneumaticMechanismAt(toggleAt[t]);
            if (p == nullptr) {
                continue;  // declared, not built: ABSENT
            }
            // The initial level is the robot's START level, from the CONFIG — the level the Robot
            // made the line at — never what the line is commanded to now (header).
            toggle_[toggleCount_].emplace(
                *p, toggles[t].button,
                config::toLineValue(cfg.mechanisms[toggleAt[t]].pneumaticStart));
            togglePtrs_[toggleCount_] = &*toggle_[toggleCount_];
            ++toggleCount_;
        }
        // ── 4. The loop's own check, last, over what was bound.
        if (!teleop::validBindings(this->holds(), this->toggles(), rezero, why, cap)) {
            unbind();
            return false;
        }
        return true;
    }

    /// Forget every binding (nothing is commanded). bind() starts with it; a program whose boot
    /// caught a throw from bind() calls it before it tears the robot down, so no binding is left
    /// pointing at a destroyed mechanism.
    void unbind() noexcept {
        for (auto& h : hold_) h.reset();
        for (auto& t : toggle_) t.reset();
        for (auto& p : holdPtrs_) p = nullptr;
        for (auto& p : togglePtrs_) p = nullptr;
        holdCount_ = 0;
        toggleCount_ = 0;
    }

    /// Every hold binding bound (built mechanisms only), in the controls' order — the loop's
    /// first span.
    [[nodiscard]] std::span<teleop::MechanismBinding* const> holds() const noexcept {
        return std::span<teleop::MechanismBinding* const>{holdPtrs_, holdCount_};
    }
    /// Every toggle bound (built mechanisms only), in the controls' order — the loop's second
    /// span.
    [[nodiscard]] std::span<teleop::ToggleBinding* const> toggles() const noexcept {
        return std::span<teleop::ToggleBinding* const>{togglePtrs_, toggleCount_};
    }

    /// The hold binding on the mechanism named `name`, or nullptr when no hold is bound to it (it
    /// was not built, no control names it, or it is not a motor mechanism). For a program's
    /// screen and boot lines.
    [[nodiscard]] const teleop::MechanismBinding* holdFor(const char* name) const noexcept {
        for (const teleop::MechanismBinding* b : holds()) {
            if (controlNameIs(b->mechanism().name(), name)) {
                return b;
            }
        }
        return nullptr;
    }
    /// The toggle bound to the mechanism named `name`, or nullptr (as holdFor()).
    [[nodiscard]] const teleop::ToggleBinding* toggleFor(const char* name) const noexcept {
        for (const teleop::ToggleBinding* t : toggles()) {
            if (controlNameIs(t->mechanism().name(), name)) {
                return t;
            }
        }
        return nullptr;
    }

    /// True when the LAST driver-control entry released a claim an operation had left on any bound
    /// mechanism (the team lead's claim ruling; the loop logged which, `[WARN][TEL]`) — what a
    /// program's claw row shows as ` FREED`.
    [[nodiscard]] bool anyReleasedLeftoverClaim() const noexcept {
        for (const teleop::MechanismBinding* b : holds()) {
            if (b->releasedLeftoverClaim()) return true;
        }
        for (const teleop::ToggleBinding* t : toggles()) {
            if (t->releasedLeftoverClaim()) return true;
        }
        return false;
    }

private:
    /// A config name field as a view: at most its array's length, NUL or not.
    static std::string_view configName(const config::MechanismConfig& m) noexcept {
        std::size_t n = 0;
        while (n < sizeof m.name && m.name[n] != '\0') ++n;
        return std::string_view{m.name, n};
    }
    /// The index of the declared mechanism named `name`, or `robot.mechanismCount()` when the
    /// robot declares none by that name. `name` is non-null.
    static std::size_t declaredIndex(const robot::Robot& robot, const char* name) noexcept {
        for (std::size_t i = 0; i < robot.mechanismCount(); ++i) {
            if (configName(robot.config().mechanisms[i]) == std::string_view{name}) {
                return i;
            }
        }
        return robot.mechanismCount();
    }
    /// True when some hold names the declared mechanism `m`.
    static bool namesHold(std::span<const HoldControl> holds,
                          const config::MechanismConfig& m) noexcept {
        for (const HoldControl& h : holds) {
            if (h.mechanism != nullptr && configName(m) == std::string_view{h.mechanism}) {
                return true;
            }
        }
        return false;
    }
    /// True when some toggle names the declared mechanism `m`.
    static bool namesToggle(std::span<const ToggleControl> toggles,
                            const config::MechanismConfig& m) noexcept {
        for (const ToggleControl& t : toggles) {
            if (t.mechanism != nullptr && configName(m) == std::string_view{t.mechanism}) {
                return true;
            }
        }
        return false;
    }

    std::optional<teleop::MechanismBinding> hold_[kCapacity];
    std::optional<teleop::ToggleBinding> toggle_[kCapacity];
    teleop::MechanismBinding* holdPtrs_[kCapacity]{};
    teleop::ToggleBinding* togglePtrs_[kCapacity]{};
    std::size_t holdCount_ = 0;
    std::size_t toggleCount_ = 0;
};

}  // namespace shulib::bench
