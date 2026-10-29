#pragma once
//
// mechanism/driver_inputs.hpp — THE DRIVER FILE'S MECHANISM CONTROLS: hold-to-run, toggle, preset and
// the homing controls, each naming its mechanism by its DECLARATION, checked at build against the
// robot's list, and run THROUGH the mechanism service (the mechanism core, chunk MECH-A4, 2026-10-22).
//
// ── What a driver file writes ────────────────────────────────────────────────────────
// ONE list, checked AT BUILD (consteval controls(...): every rule below is made there, so the file
// carries no static_assert of its own):
//
//     inline constexpr DriverControls kControls = controls(kRobotXMechanisms,
//         whileHeld(arm, Button::R1, Button::R2, 6_volt),  // up / down while held; let go: it holds there
//         preset(arm["score"], Button::Right, 1.5_s),      // the most recent button wins
//         whileHeld(lift, Button::L1, Button::L2, 12_volt),
//         homeAtDriverStart(lift, 1.5_s),                  // after a restart it may not be on its switch
//         toggle(flywheel["shoot"], Button::X),            // on at "shoot" / off
//         whileHeld(intake, Button::A, Button::B, 12_volt),
//         autonOnly(hood));                                // the hood is the autons' alone
//
// ── What each control does (DriverInputs::tick, once per driver-control tick) ──────────
//   * whileHeld on a POWER mechanism: held → drive(±V) every tick; released → stop (its safe state).
//     On an ARM or a LIFT: held → jog(±V) (soft-stopped at the limits once homed; at most
//     `.unhomedVolts` before); released → HOLD HERE (gravity included, clamped to the range). Both
//     buttons held = neither (T3's ruling R2): the release rule.
//   * toggle: a SEEN press flips a two-position pneumatic, or spins a flywheel up to the NAMED speed
//     (`toggle(flywheel["shoot"], X)`) / stops it. A flywheel toggle's step is bounded by
//     kToggleAtSpeedWindowSeconds — only its logged "at speed" verdict: the wheel keeps spinning.
//   * preset: a SEEN press sends the mechanism to a preset or a constant (`arm["score"]`,
//     `lift.to(18_in)`, `flywheel["shoot"]`, `claw["open"]`) within its time limit; then it holds or
//     keeps spinning. A pneumatic position states no time limit (its move time is the robot file's).
//   * homeOnPress: a SEEN press homes it by its declaration's method, bounded. homeAtDriverStart:
//     ONCE at driver-control entry, when nothing has homed it (a restart can leave a switch- or
//     stall-homed mechanism off its home: its buttons would be dead — the brief's U5). WHILE THAT
//     HOME RUNS, the mechanism's own holds, presets and homing press WAIT (the exception below).
//   * autonOnly: no driver control, on purpose (Q8) — the build's "every mechanism has a control"
//     rule then passes it.
//
// ── THE MOST RECENT INPUT WINS (the team lead's Q8 (a)) ─────────────────────────────────
// Arbitration is by the service slot's COMMAND NUMBER (Slot::commandSeq(), bumped by every command
// and by a stall that ends a behaviour after its step). A hold-to-run that sees a number it did not
// write has been SUPERSEDED — by a preset, a toggle, another control, an auton leftover, a stall —
// and commands NOTHING until its buttons are seen released while connected (T3's "an outside stop
// is never undone by a finger that never left the button", generalized).
//
// A PRESS MUST BE SEEN — for a toggle, a preset and a homeOnPress: at entry its button counts as
// held (one already down on the entry tick does nothing until it is released and pressed again),
// and so does its button while the controller is disconnected (a drop is never a press, and a
// reconnect with the button still down is not one either). A HOLD is a level, not a press (T3's
// rule for a hold binding): its button already down on the entry tick drives at once, and a
// disconnect releases it that tick. So a disconnect is a release for the holds, never a press for
// the rest. All of it rests on one unmeasured belief, T3's ToggleBinding's too: the controller's
// connected flag and its button levels come back TOGETHER (A4 register HA-147, invented) — the
// first connected tick's level is taken as live, so a stale "up" read on it while the driver still
// holds the button would make the next live "down" a press nobody made.
//
// THE ONE EXCEPTION — the driver-start home (U5). While a homeAtDriverStart home is running, that
// mechanism's OWN controls wait: a hold held during it is SUPERSEDED (nothing until its buttons are
// seen released after the home ends), and a preset's or a homeOnPress's press is consumed and
// dropped. Without it, a driver touching the lift in the home's first 1.5 s would CANCEL the home
// ("replaced by a newer command") and leave the lift unhomed — every preset NOT_HOMED and every
// hold at `.unhomedVolts` (0: nothing) for the rest of the match, the dead buttons U5 exists to
// prevent. Waiting costs at most the home's own time limit. Other mechanisms' controls never wait.
//
// ── Where it runs ───────────────────────────────────────────────────────────────────
// teleop::runDriverControl's service overload (teleop/teleop_loop.hpp) calls enter() at loop entry
// (after the service's enterDriverControl()) and tick() each tick after the T3 bindings' place and
// BEFORE the service's tickFromDriverLoop() — so a press read at tick k is commanded AND ticked in
// tick k — and before the loop's perTick.
//
// ── Static storage ──────────────────────────────────────────────────────────────────
// A program keeps its DriverInputs in static storage beside its service, like T3's bindings: PROS
// deletes the driver task on a mode change (HA-161) and a deleted task runs no destructor. bind()
// once, at boot, AFTER the service is bound.
//
// PROS-free, allocation-free. Nothing here has run on a robot.

#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "shulib/core/check.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/mechanism/spec.hpp"
#include "shulib/mechanism/wiring.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::mechanism {

/// A driver toggle's flywheel spin-up: its "at speed" verdict window, seconds (HA-182 — invented,
/// PROVISIONAL; a LIBRARY constant). Not a match number: the wheel keeps spinning whatever the
/// verdict; it only bounds the step the toggle logs.
inline constexpr double kToggleAtSpeedWindowSeconds = 5.0;

// ═══ THE CONTROLS, AS DATA (a driver file's lines) ═══════════════════════════════════════

/// What one driver control does (file header).
enum class ControlKind : std::uint8_t {
    Hold,               ///< whileHeld: two buttons, + / − volts while held
    Toggle,             ///< one button: flips a two-position pneumatic, or a flywheel's named speed on / off
    Preset,             ///< one button: a position, a speed or a pneumatic position
    HomeOnPress,        ///< one button: home it by its declaration's method
    HomeAtDriverStart,  ///< no button: home it once at driver-control entry, when nothing homed it
    AutonOnly,          ///< no control, on purpose
};

/// One driver-file line. Made only by the words below, each taking the mechanism's DECLARATION (or a
/// goal it made), so a control always names its mechanism.
struct Control {
    /// What it does.
    ControlKind kind = ControlKind::Hold;
    /// The kind of mechanism it commands (a declaration's kind).
    Kind mechanismKind = Kind::Power;
    /// The declaration's address — what controls() checks against the robot's list.
    const void* mechanism = nullptr;
    /// The declaration's `.name` — what DriverInputs::bind() finds its slot by.
    const char* mechanismName = nullptr;
    /// A hold's FORWARD (up) button; every other control's one button.
    hal::ControllerButton button = hal::ControllerButton::A;
    /// A hold's BACK (down) button.
    hal::ControllerButton back = hal::ControllerButton::A;
    /// A hold's volts, (0, 12].
    double volts = 0.0;
    /// A preset's or a homing's time limit, seconds (P4: every action bounded).
    double bound = 0.0;
    /// A preset's target: degrees, inches or output rpm (a flywheel toggle's speed too).
    double value = 0.0;
    /// A pneumatic preset's position (its index in the declaration).
    std::uint8_t position = 0;
    /// The preset's (or a pneumatic position's) name; nullptr for a constant target.
    const char* presetName = nullptr;
    /// An arm's (degrees) or a lift's (inches) control: its units; UNSET for the other kinds.
    Units units = static_cast<Units>(0);
};

namespace refused {
/// whileHeld(...) on a pneumatic or a flywheel (n24).
inline void whileHeld_is_for_a_power_mechanism_an_arm_or_a_lift_use_toggle_or_preset() {}
/// toggle(flywheel, ...) without its speed (n51).
inline void a_toggle_on_a_flywheel_names_its_speed_like_toggle_flywheel_shoot() {}
/// toggle(...) on a power mechanism, an arm or a lift.
inline void a_toggle_is_for_a_two_position_pneumatic_or_a_flywheel_speed() {}
/// homeOnPress(...) / homeAtDriverStart(...) on a power mechanism, a flywheel or a pneumatic.
inline void homeOnPress_and_homeAtDriverStart_are_for_an_arm_or_a_lift() {}
/// A control names a mechanism the robot's list does not have (n19).
inline void a_control_names_a_mechanism_this_robot_does_not_have() {}
/// A control names a button that is not one of the controller's twelve (a cast byte).
inline void a_control_names_a_button_that_is_not_one_of_the_twelve() {}
/// L1 does two jobs (a button used twice NAMES the button; the error points at the controls( line).
inline void button_L1_does_two_jobs() {}
/// L2 does two jobs.
inline void button_L2_does_two_jobs() {}
/// R1 does two jobs (n17).
inline void button_R1_does_two_jobs() {}
/// R2 does two jobs.
inline void button_R2_does_two_jobs() {}
/// Up does two jobs.
inline void button_Up_does_two_jobs() {}
/// Down does two jobs.
inline void button_Down_does_two_jobs() {}
/// Left does two jobs.
inline void button_Left_does_two_jobs() {}
/// Right does two jobs (m2).
inline void button_Right_does_two_jobs() {}
/// X does two jobs.
inline void button_X_does_two_jobs() {}
/// B does two jobs.
inline void button_B_does_two_jobs() {}
/// Y does two jobs.
inline void button_Y_does_two_jobs() {}
/// A does two jobs.
inline void button_A_does_two_jobs() {}
/// A hold's volts outside (0, 12] (n20).
inline void a_hold_needs_volts_above_0_and_at_most_12() {}
/// A preset or a homing control with no time limit (n34).
inline void a_preset_or_a_homing_needs_a_time_limit_above_zero() {}
/// Two controls of one kind on one mechanism — two holds, two toggles, two homings, two presets to
/// one target (n21). A hold beside presets is fine: the most recent input wins (Q8).
inline void two_controls_of_the_same_kind_on_one_mechanism() {}
/// A toggle on a three-position pneumatic, or on a flywheel's constant speed instead of a named one
/// (n22).
inline void a_toggle_needs_a_two_position_pneumatic_or_a_flywheel_named_speed() {}
/// A homing control on a mechanism declared neverHomed.
inline void a_homing_control_on_a_mechanism_that_is_never_homed() {}
/// A mechanism with no control and not marked autonOnly (n18).
inline void a_mechanism_has_no_driver_control_mark_it_autonOnly_if_that_is_meant() {}
/// A mechanism marked autonOnly that also has a control.
inline void a_mechanism_marked_autonOnly_also_has_a_control() {}
/// More than DriverControls::kCap controls.
inline void more_controls_than_fit() {}
/// A switch- or stall-homed mechanism held or preset with no way to home it in driver control (n48):
/// its buttons would be dead after any restart off its home — add homeAtDriverStart or homeOnPress.
/// The declaration's `.unhomedVolts` answers for its HOLDS only (they move it unhomed at those
/// volts); it is never a way to HOME it — a jog never zeroes a mechanism — so a PRESET on it would
/// end NOT_HOMED for the rest of the run, and a preset needs one of the two homing controls.
inline void a_mechanism_that_must_be_homed_has_no_way_to_home_in_driver_control() {}
/// A never-homed mechanism held with no `.unhomedVolts` (n49): its buttons would command 0 V.
inline void a_never_homed_mechanism_needs_unhomedVolts_for_its_buttons_to_move_it() {}
/// A preset on a never-homed mechanism (n50): always NOT_HOMED — a dead button.
inline void a_preset_on_a_mechanism_that_is_never_homed() {}
}  // namespace refused

// ── the words ──────────────────────────────────────────────────────────────────────────

/// WHILE HELD on a POWER mechanism: `forward` held = +volts, `back` held = −volts, let go = stop (its
/// declared safe state). (Re-check 2026-10-07: "hold" read as a position AND a brake mode.)
[[nodiscard]] constexpr Control whileHeld(const Power& m, hal::ControllerButton forward, hal::ControllerButton back,
                                          units::Voltage volts) noexcept {
    return Control{.kind = ControlKind::Hold, .mechanismKind = Kind::Power, .mechanism = &m,
                   .mechanismName = m.name.text, .button = forward, .back = back, .volts = volts.value()};
}
/// WHILE HELD on an ARM: `up` held = +volts (a jog), `down` held = −volts, let go = HOLD HERE.
[[nodiscard]] constexpr Control whileHeld(const Arm& m, hal::ControllerButton up, hal::ControllerButton down,
                                          units::Voltage volts) noexcept {
    return Control{.kind = ControlKind::Hold, .mechanismKind = Kind::Position, .mechanism = &m,
                   .mechanismName = m.name.text, .button = up, .back = down, .volts = volts.value(),
                   .units = Units::Degrees};
}
/// WHILE HELD on a LIFT: as on an arm.
[[nodiscard]] constexpr Control whileHeld(const Lift& m, hal::ControllerButton up, hal::ControllerButton down,
                                          units::Voltage volts) noexcept {
    return Control{.kind = ControlKind::Hold, .mechanismKind = Kind::Position, .mechanism = &m,
                   .mechanismName = m.name.text, .button = up, .back = down, .volts = volts.value(),
                   .units = Units::Inches};
}
/// `whileHeld(hood, ...)` / `whileHeld(flywheel, ...)`: refused BY NAME (use toggle or preset).
template <class D>
    requires(std::same_as<D, Pneumatic> || std::same_as<D, Flywheel>)
consteval Control whileHeld(const D&, hal::ControllerButton, hal::ControllerButton, units::Voltage) {
    refused::whileHeld_is_for_a_power_mechanism_an_arm_or_a_lift_use_toggle_or_preset();
    return Control{};
}
/// Each SEEN press flips a TWO-position pneumatic.
[[nodiscard]] constexpr Control toggle(const Pneumatic& m, hal::ControllerButton b) noexcept {
    return Control{.kind = ControlKind::Toggle, .mechanismKind = Kind::Pneumatic, .mechanism = &m,
                   .mechanismName = m.name.text, .button = b};
}
/// Each SEEN press spins a flywheel up to THE NAMED SPEED, or stops it: `toggle(flywheel["shoot"],
/// Button::X)` (re-check 2026-10-07: "its first preset" changed silently when presets were reordered).
[[nodiscard]] constexpr Control toggle(SpeedGoal g, hal::ControllerButton b) noexcept {
    return Control{.kind = ControlKind::Toggle, .mechanismKind = Kind::Speed, .mechanism = g.mechanism,
                   .mechanismName = g.mechanismName, .button = b, .value = g.rpm, .presetName = g.presetName};
}
/// `toggle(flywheel, X)`: refused BY NAME — name the speed (n51).
template <class D>
    requires std::same_as<D, Flywheel>
consteval Control toggle(const D&, hal::ControllerButton) {
    refused::a_toggle_on_a_flywheel_names_its_speed_like_toggle_flywheel_shoot();
    return Control{};
}
/// `toggle(intake, X)` / `toggle(arm, X)`: refused BY NAME.
template <class D>
    requires(std::same_as<D, Power> || std::same_as<D, Arm> || std::same_as<D, Lift>)
consteval Control toggle(const D&, hal::ControllerButton) {
    refused::a_toggle_is_for_a_two_position_pneumatic_or_a_flywheel_speed();
    return Control{};
}
/// A SEEN press sends an arm or a lift to `g` (`arm["score"]`, `arm.to(95)`, `lift.to(18_in)`) within
/// `bound`, then it holds there.
[[nodiscard]] constexpr Control preset(PositionGoal g, hal::ControllerButton b, units::Time bound) noexcept {
    return Control{.kind = ControlKind::Preset, .mechanismKind = Kind::Position, .mechanism = g.mechanism,
                   .mechanismName = g.mechanismName, .button = b, .bound = bound.value(), .value = g.value,
                   .presetName = g.presetName, .units = g.units};
}
/// A SEEN press spins a flywheel up to `g` (`flywheel["shoot"]`, `flywheel.at(3000)`) within `bound`
/// (its "at speed" verdict); it keeps spinning.
[[nodiscard]] constexpr Control preset(SpeedGoal g, hal::ControllerButton b, units::Time bound) noexcept {
    return Control{.kind = ControlKind::Preset, .mechanismKind = Kind::Speed, .mechanism = g.mechanism,
                   .mechanismName = g.mechanismName, .button = b, .bound = bound.value(), .value = g.rpm,
                   .presetName = g.presetName};
}
/// A SEEN press puts a pneumatic in position `g` (`claw["open"]`) — no time limit: its move time is the
/// robot file's.
[[nodiscard]] constexpr Control preset(PneumaticGoal g, hal::ControllerButton b) noexcept {
    return Control{.kind = ControlKind::Preset, .mechanismKind = Kind::Pneumatic, .mechanism = g.mechanism,
                   .mechanismName = g.mechanismName, .button = b, .position = g.position,
                   .presetName = g.positionName};
}
/// A SEEN press homes an ARM by its declaration's method, within `bound`.
[[nodiscard]] constexpr Control homeOnPress(const Arm& m, hal::ControllerButton b, units::Time bound) noexcept {
    return Control{.kind = ControlKind::HomeOnPress, .mechanismKind = Kind::Position, .mechanism = &m,
                   .mechanismName = m.name.text, .button = b, .bound = bound.value(), .units = Units::Degrees};
}
/// A SEEN press homes a LIFT by its declaration's method, within `bound`.
[[nodiscard]] constexpr Control homeOnPress(const Lift& m, hal::ControllerButton b, units::Time bound) noexcept {
    return Control{.kind = ControlKind::HomeOnPress, .mechanismKind = Kind::Position, .mechanism = &m,
                   .mechanismName = m.name.text, .button = b, .bound = bound.value(), .units = Units::Inches};
}
/// ONCE at driver-control entry, an ARM nothing has homed is homed, within `bound`.
[[nodiscard]] constexpr Control homeAtDriverStart(const Arm& m, units::Time bound) noexcept {
    return Control{.kind = ControlKind::HomeAtDriverStart, .mechanismKind = Kind::Position, .mechanism = &m,
                   .mechanismName = m.name.text, .bound = bound.value(), .units = Units::Degrees};
}
/// ONCE at driver-control entry, a LIFT nothing has homed is homed, within `bound`.
[[nodiscard]] constexpr Control homeAtDriverStart(const Lift& m, units::Time bound) noexcept {
    return Control{.kind = ControlKind::HomeAtDriverStart, .mechanismKind = Kind::Position, .mechanism = &m,
                   .mechanismName = m.name.text, .bound = bound.value(), .units = Units::Inches};
}
/// `homeOnPress(intake, ...)`: refused BY NAME — only an arm or a lift homes.
template <class D>
    requires(std::same_as<D, Power> || std::same_as<D, Flywheel> || std::same_as<D, Pneumatic>)
consteval Control homeOnPress(const D&, hal::ControllerButton, units::Time) {
    refused::homeOnPress_and_homeAtDriverStart_are_for_an_arm_or_a_lift();
    return Control{};
}
/// `homeAtDriverStart(intake, ...)`: refused BY NAME — only an arm or a lift homes.
template <class D>
    requires(std::same_as<D, Power> || std::same_as<D, Flywheel> || std::same_as<D, Pneumatic>)
consteval Control homeAtDriverStart(const D&, units::Time) {
    refused::homeOnPress_and_homeAtDriverStart_are_for_an_arm_or_a_lift();
    return Control{};
}
/// NO driver control, on purpose (Q8): `autonOnly(hood)`.
template <class Declared>
    requires requires(const Declared& d) { d.name.text; }
[[nodiscard]] constexpr Control autonOnly(const Declared& m) noexcept {
    return Control{.kind = ControlKind::AutonOnly, .mechanism = &m, .mechanismName = m.name.text};
}

namespace detail {
/// Refuse button `i` used twice, NAMING it (the order is hal::ControllerButton's).
constexpr void refuseButtonTwice(std::size_t i) {
    switch (i) {
        case 0: refused::button_L1_does_two_jobs(); return;
        case 1: refused::button_L2_does_two_jobs(); return;
        case 2: refused::button_R1_does_two_jobs(); return;
        case 3: refused::button_R2_does_two_jobs(); return;
        case 4: refused::button_Up_does_two_jobs(); return;
        case 5: refused::button_Down_does_two_jobs(); return;
        case 6: refused::button_Left_does_two_jobs(); return;
        case 7: refused::button_Right_does_two_jobs(); return;
        case 8: refused::button_X_does_two_jobs(); return;
        case 9: refused::button_B_does_two_jobs(); return;
        case 10: refused::button_Y_does_two_jobs(); return;
        case 11: refused::button_A_does_two_jobs(); return;
        default: refused::a_control_names_a_button_that_is_not_one_of_the_twelve(); return;
    }
}
/// The control's time limit is a real one: above 0, finite (a NaN or an infinity is no limit).
[[nodiscard]] constexpr bool realBound(double s) noexcept { return s > 0.0 && s < 1e9; }
/// True when `c` takes `b` (a hold's two buttons; a toggle's, a preset's, a homeOnPress's one).
[[nodiscard]] constexpr bool takes(const Control& c, hal::ControllerButton b) noexcept {
    switch (c.kind) {
        case ControlKind::Hold: return c.button == b || c.back == b;
        case ControlKind::Toggle:
        case ControlKind::Preset:
        case ControlKind::HomeOnPress: return c.button == b;
        case ControlKind::HomeAtDriverStart:
        case ControlKind::AutonOnly: return false;
    }
    return false;
}
}  // namespace detail

/// A driver file's controls, checked (controls(...)).
struct DriverControls {
    /// The most controls one driver file holds.
    static constexpr std::size_t kCap = 24;
    /// The controls, in the order written.
    Control c[kCap]{};
    /// How many are real.
    std::uint8_t count = 0;
    /// The controls as a span.
    [[nodiscard]] constexpr std::span<const Control> all() const noexcept {
        return std::span<const Control>{c, count < kCap ? count : kCap};
    }
    /// No control takes `b` — the driver loop's reserved button (its re-zero).
    [[nodiscard]] constexpr bool avoids(hal::ControllerButton b) const noexcept {
        for (const Control& x : all()) {
            if (detail::takes(x, b)) {
                return false;
            }
        }
        return true;
    }
};

/// THE DRIVER FILE'S ONE LIST (file header), checked AT BUILD against the robot's list `robot`, each
/// rule refused BY NAME (refused::): every control names a mechanism this robot has; no button does
/// two jobs (the button named); a hold's volts in (0, 12]; a preset's and a homing's time limit above
/// 0 (a pneumatic preset states none); no two controls of one kind on one mechanism (a hold beside
/// presets is fine — Q8; two presets are two kinds of one only when they aim at the same target); a
/// toggle only on a two-position pneumatic or a flywheel's NAMED speed; a homing control only on a
/// mechanism that homes; a switch- or stall-homed arm or lift preset only with a way to home it in
/// driver control (homeAtDriverStart or homeOnPress), and held only with one of those or its
/// `.unhomedVolts` (U5 — a jog moves it unhomed but never homes it); a never-homed one held only
/// with `.unhomedVolts`, and never preset; every mechanism of the list has a control or is marked
/// autonOnly (and then has none); at most DriverControls::kCap controls.
template <class... C>
[[nodiscard]] consteval DriverControls controls(const MechanismTable& robot, C... list) {
    DriverControls d{};
    for (const Control& x : std::array<Control, sizeof...(C)>{list...}) {
        if (d.count == DriverControls::kCap) {
            refused::more_controls_than_fit();
            break;
        }
        d.c[d.count++] = x;
    }
    bool used[12]{};
    const auto take = [&](hal::ControllerButton b) {
        const auto i = static_cast<std::size_t>(b);
        if (i >= 12 || used[i]) {
            detail::refuseButtonTwice(i);
        } else {
            used[i] = true;
        }
    };
    for (std::size_t i = 0; i < d.count; ++i) {
        const Control& x = d.c[i];
        const int m = robot.indexOf(x.mechanism);
        if (m < 0) {
            refused::a_control_names_a_mechanism_this_robot_does_not_have();
            continue;
        }
        const MechanismSpec& s = robot.specs[static_cast<std::size_t>(m)];
        switch (x.kind) {
            case ControlKind::Hold:
                take(x.button);
                take(x.back);
                if (!(x.volts > 0.0 && x.volts <= 12.0)) {
                    refused::a_hold_needs_volts_above_0_and_at_most_12();
                }
                break;
            case ControlKind::Toggle:
                take(x.button);
                if ((s.kind == Kind::Pneumatic && s.pneumatic.positionCount != 2) ||
                    (s.kind == Kind::Speed && x.presetName == nullptr)) {
                    refused::a_toggle_needs_a_two_position_pneumatic_or_a_flywheel_named_speed();
                }
                break;
            case ControlKind::Preset:
                take(x.button);
                if (x.mechanismKind != Kind::Pneumatic && !detail::realBound(x.bound)) {
                    refused::a_preset_or_a_homing_needs_a_time_limit_above_zero();
                }
                break;
            case ControlKind::HomeOnPress:
                take(x.button);
                [[fallthrough]];
            case ControlKind::HomeAtDriverStart:
                if (!detail::realBound(x.bound)) {
                    refused::a_preset_or_a_homing_needs_a_time_limit_above_zero();
                }
                if (s.position.home == HomeMethod::None) {
                    refused::a_homing_control_on_a_mechanism_that_is_never_homed();
                }
                break;
            case ControlKind::AutonOnly:
                break;
        }
        for (std::size_t j = i + 1; j < d.count; ++j) {
            const Control& y = d.c[j];
            if (y.mechanism != x.mechanism || y.kind != x.kind) {
                continue;
            }
            if (x.kind == ControlKind::Preset && (x.value != y.value || x.position != y.position)) {
                continue;  // two presets to two targets: the most recent wins
            }
            refused::two_controls_of_the_same_kind_on_one_mechanism();
        }
    }
    for (std::size_t m = 0; m < robot.count && m < kMaxMechanisms; ++m) {
        bool controlled = false;
        bool marked = false;
        bool homes = false;
        bool holds = false;
        bool presets = false;
        for (const Control& x : d.all()) {
            if (x.mechanism != robot.declared[m] || robot.declared[m] == nullptr) {
                continue;
            }
            if (x.kind == ControlKind::AutonOnly) {
                marked = true;
            } else {
                controlled = true;
            }
            homes = homes || x.kind == ControlKind::HomeAtDriverStart || x.kind == ControlKind::HomeOnPress;
            holds = holds || x.kind == ControlKind::Hold;
            presets = presets || x.kind == ControlKind::Preset;
        }
        if (!controlled && !marked) {
            refused::a_mechanism_has_no_driver_control_mark_it_autonOnly_if_that_is_meant();
        }
        if (controlled && marked) {
            refused::a_mechanism_marked_autonOnly_also_has_a_control();
        }
        // U5: a position mechanism that must be HOMED by a move (a switch, a stall) is NOT homed after
        // a restart that leaves it off its home; its holds would then move it at .unhomedVolts (0 = not
        // at all) and its presets end NOT_HOMED — dead buttons for the run. .unhomedVolts rescues the
        // HOLDS only: no jog ever zeroes it (the service zeroes at bind and in a homing step), so a
        // PRESET needs a homing control whatever its volts (the review's finding; A4-D27).
        const MechanismSpec& s = robot.specs[m];
        if (s.kind != Kind::Position || !(holds || presets)) {
            continue;
        }
        const bool movesToHome = s.position.home == HomeMethod::Switch || s.position.home == HomeMethod::Stall;
        if (movesToHome && !homes && (presets || !(s.position.unhomedVolts > 0.0))) {
            refused::a_mechanism_that_must_be_homed_has_no_way_to_home_in_driver_control();
        }
        if (s.position.home == HomeMethod::None && presets) {
            refused::a_preset_on_a_mechanism_that_is_never_homed();
        }
        if (s.position.home == HomeMethod::None && holds && !(s.position.unhomedVolts > 0.0)) {
            refused::a_never_homed_mechanism_needs_unhomedVolts_for_its_buttons_to_move_it();
        }
    }
    return d;
}

// ═══ THE RUN-TIME INPUTS ═════════════════════════════════════════════════════════════════

/// A driver file's controls at run time, bound to the robot's mechanism service (file header). Lives
/// in the program's static storage beside the service; bind() once, at boot, after the service.
/// Non-copyable and non-movable: the loop holds it by reference, and a copy would be a second set of
/// button states commanding the same mechanisms.
class DriverInputs {
public:
    /// The most controls (DriverControls::kCap).
    static constexpr std::size_t kCap = DriverControls::kCap;

    /// Unbound: it commands nothing until bind().
    DriverInputs() = default;
    /// Non-copyable (the class's reason).
    DriverInputs(const DriverInputs&) = delete;
    /// Non-copy-assignable (the class's reason).
    DriverInputs& operator=(const DriverInputs&) = delete;
    /// Non-movable (the class's reason).
    DriverInputs(DriverInputs&&) = delete;
    /// Non-move-assignable (the class's reason).
    DriverInputs& operator=(DriverInputs&&) = delete;
    /// Nothing to release: the service owns every claim.
    ~DriverInputs() = default;

    /// BIND, once, at boot, after `svc` is bound: each control resolved to its slot by the
    /// declaration's NAME. `controls` is BORROWED (static storage: a driver file's `inline constexpr`).
    /// A control on a mechanism that is ABSENT this boot still binds — it commands a slot that does
    /// nothing (the drive runs). PRECONDITIONS (the controls were checked against ONE robot's list at
    /// build; these check them against the robot the SERVICE is bound to, so a driver file built for
    /// another robot never commands a stranger): not bound yet; `svc` bound; every control but an
    /// autonOnly names a mechanism the service has, of the control's kind and units; a toggle's
    /// pneumatic has two positions; a named preset (or a flywheel toggle's named speed) is this
    /// mechanism's, at the same value; a constant target is inside its travel limits or free speed; a
    /// pneumatic preset's position is this mechanism's, by name; a homing control's mechanism homes.
    void bind(MechanismService& svc, const DriverControls& controls) {
        SHULIB_PRECONDITION(svc_ == nullptr, "DriverInputs::bind: called twice -- bind once, at boot");
        SHULIB_PRECONDITION(svc.bound(), "DriverInputs::bind: the mechanism service is not bound -- bind it first");
        SHULIB_PRECONDITION(controls.count <= kCap, "DriverInputs::bind: more controls than DriverControls holds");
        for (std::size_t i = 0; i < controls.count; ++i) {
            const Control& c = controls.c[i];
            if (c.kind == ControlKind::AutonOnly) {
                slot_[i] = kNoSlot;
                continue;
            }
            const Slot* s = svc.slotFor(c.mechanismName);
            SHULIB_PRECONDITION(s != nullptr,
                                "DriverInputs::bind: a control names a mechanism the service does not have (the "
                                "controls were checked against another robot's list?)");
            SHULIB_PRECONDITION(s->kind() == c.mechanismKind,
                                "DriverInputs::bind: a control's mechanism is another KIND on this robot (another "
                                "robot's declaration of the same name)");
            const MechanismSpec& spec = s->spec();
            SHULIB_PRECONDITION(c.mechanismKind != Kind::Position || spec.position.units == c.units,
                                "DriverInputs::bind: an arm's control names a lift on this robot, or a lift's an arm");
            SHULIB_PRECONDITION(matchesThisRobot(c, spec),
                                "DriverInputs::bind: a control's preset, position, speed or homing is not this "
                                "robot's mechanism's (another robot's declaration of the same name)");
            slot_[i] = s->index();
        }
        controls_ = &controls;
        svc_ = &svc;
    }
    /// No control takes `b` — the loop's re-zero button. True when unbound.
    [[nodiscard]] bool avoids(hal::ControllerButton b) const noexcept {
        return controls_ == nullptr || controls_->avoids(b);
    }
    /// True once bind() has run.
    [[nodiscard]] bool bound() const noexcept { return svc_ != nullptr; }
    /// True when bind() bound it to `svc` — the loop's check that its inputs command the service it
    /// ticks.
    [[nodiscard]] bool boundTo(const MechanismService& svc) const noexcept { return svc_ == &svc; }

    /// DRIVER-CONTROL ENTRY (after the service's enterDriverControl()): every hold released; every
    /// toggle's, preset's and homeOnPress's button counted as HELD (a press must be SEEN — one
    /// already down on the entry tick does nothing until released and pressed again; a hold's button
    /// is a level, and down at entry it drives at once — file header); then each homeAtDriverStart
    /// homes its mechanism when nothing has homed it (skipped when homed — by the auton, at boot, or
    /// an earlier entry), and while that home runs the mechanism's own controls wait (file header).
    /// PRECONDITION: bound.
    void enter() {
        SHULIB_PRECONDITION(bound(), "DriverInputs::enter: not bound -- bind() at boot");
        for (std::size_t i = 0; i < kCap; ++i) {
            holdState_[i] = HoldState::Released;
            mySeq_[i] = 0;
            startHomeSeq_[i] = 0;
            edge_[i] = hal::ButtonEdge{};
            (void)edge_[i].update(true);
        }
        for (std::size_t i = 0; i < controls_->count; ++i) {
            const Control& c = controls_->c[i];
            if (c.kind == ControlKind::HomeAtDriverStart && !svc_->position(slot_[i]).homed()) {
                PositionSlot& p = svc_->position(slot_[i]);
                p.home(units::Time{c.bound});
                startHomeSeq_[i] = p.commandSeq();  // THIS home: anything newer that replaced it ends the wait
            }
        }
    }

    /// ONE DRIVER TICK: read every control's buttons (only while `connected`) and command the service
    /// (file header) — the service ticks right after (teleop_loop.hpp). PRECONDITION: bound.
    void tick(const hal::IController& c, bool connected) {
        SHULIB_PRECONDITION(bound(), "DriverInputs::tick: not bound -- bind() at boot");
        for (std::size_t i = 0; i < controls_->count; ++i) {
            const Control& x = controls_->c[i];
            switch (x.kind) {
                case ControlKind::Hold:
                    tickHold(i, x, c, connected);
                    break;
                case ControlKind::Toggle:
                    if (pressedNow(i, x, c, connected)) {
                        if (x.mechanismKind == Kind::Pneumatic) {
                            PneumaticSlot& p = svc_->pneumatic(slot_[i]);
                            p.set(p.commandedPosition() == 0 ? std::uint8_t{1} : std::uint8_t{0});
                        } else {
                            SpeedSlot& f = svc_->speed(slot_[i]);
                            if (f.spinning()) {
                                f.stop();
                            } else {
                                f.spinUp(x.value, units::Time{kToggleAtSpeedWindowSeconds}, /*computed=*/false);
                            }
                        }
                    }
                    break;
                case ControlKind::Preset:
                    // the press is read (consumed) FIRST, then dropped while the driver-start home runs
                    if (pressedNow(i, x, c, connected) && !startHomeRuns(slot_[i])) {
                        if (x.mechanismKind == Kind::Position) {
                            svc_->position(slot_[i]).moveTo(x.value, units::Time{x.bound}, /*computed=*/false);
                        } else if (x.mechanismKind == Kind::Speed) {
                            svc_->speed(slot_[i]).spinUp(x.value, units::Time{x.bound}, /*computed=*/false);
                        } else {
                            svc_->pneumatic(slot_[i]).set(x.position);
                        }
                    }
                    break;
                case ControlKind::HomeOnPress:
                    if (pressedNow(i, x, c, connected) && !startHomeRuns(slot_[i])) {
                        svc_->position(slot_[i]).home(units::Time{x.bound});
                    }
                    break;
                case ControlKind::HomeAtDriverStart:
                case ControlKind::AutonOnly:
                    break;
            }
        }
    }

private:
    static constexpr std::uint8_t kNoSlot = 0xFF;
    /// A hold-to-run's state (file header: the most recent input wins).
    enum class HoldState : std::uint8_t {
        Released,    ///< not driving; a held button drives it
        Driving,     ///< it commanded the slot last
        Superseded,  ///< something newer spoke while it was held: nothing until seen released
    };

    /// The bind() check of one control against the service's spec of its mechanism.
    [[nodiscard]] static bool matchesThisRobot(const Control& c, const MechanismSpec& s) noexcept {
        switch (c.kind) {
            case ControlKind::Hold:
                return c.mechanismKind == Kind::Power || c.mechanismKind == Kind::Position;
            case ControlKind::Toggle:
                if (c.mechanismKind == Kind::Pneumatic) {
                    return s.pneumatic.positionCount == 2;
                }
                return c.mechanismKind == Kind::Speed && c.presetName != nullptr &&
                       hasPreset(s.speed.presets, s.speed.presetCount, c.presetName, c.value);
            case ControlKind::Preset:
                if (c.mechanismKind == Kind::Pneumatic) {
                    return c.position < s.pneumatic.positionCount && c.position < kMaxPneumaticPositions &&
                           c.presetName != nullptr &&
                           std::string_view{s.pneumatic.positions[c.position].name} == std::string_view{c.presetName};
                }
                if (c.mechanismKind == Kind::Speed) {
                    return c.presetName != nullptr ? hasPreset(s.speed.presets, s.speed.presetCount, c.presetName, c.value)
                                                   : std::abs(c.value) <= s.speed.maxRpm;
                }
                return c.presetName != nullptr
                           ? hasPreset(s.position.presets, s.position.presetCount, c.presetName, c.value)
                           : (c.value >= s.position.minPosition && c.value <= s.position.maxPosition);
            case ControlKind::HomeOnPress:
            case ControlKind::HomeAtDriverStart:
                return c.mechanismKind == Kind::Position && s.position.home != HomeMethod::None;
            case ControlKind::AutonOnly:
                return true;
        }
        return false;
    }
    /// One of `presets` is called `name` AND holds `value`.
    [[nodiscard]] static bool hasPreset(const Preset (&presets)[kMaxPresets], std::uint8_t count, const char* name,
                                        double value) noexcept {
        for (std::size_t i = 0; i < count && i < kMaxPresets; ++i) {
            if (std::string_view{presets[i].name} == std::string_view{name}) {
                return presets[i].value == value;
            }
        }
        return false;
    }
    /// A SEEN press of control `i`'s one button: a rising edge while connected. While disconnected
    /// the button's level is unknown: it is RECORDED as held (so a reconnect with it down is not a
    /// press) and the tick is never a press — feeding "held" into an edge whose last level was up is
    /// itself a rising edge, so the drop's result must be discarded, not returned (the review's
    /// blocker: robot two's claw flipped, and robot x's three presets fired, on a drop with every
    /// button up). T3's ToggleBinding::forgetLevel() is the same rule.
    [[nodiscard]] bool pressedNow(std::size_t i, const Control& x, const hal::IController& c, bool connected) {
        if (!connected) {
            (void)edge_[i].update(true);
            return false;
        }
        return edge_[i].update(c.pressed(x.button));
    }
    /// True while the driver-start home that enter() began on slot `slot` is still running — the
    /// step THIS input began (its command number unchanged since) and no verdict yet (file header:
    /// the one exception).
    [[nodiscard]] bool startHomeRuns(std::uint8_t slot) const {
        for (std::size_t j = 0; j < controls_->count; ++j) {
            if (controls_->c[j].kind == ControlKind::HomeAtDriverStart && slot_[j] == slot && startHomeSeq_[j] != 0) {
                const Slot& s = svc_->slot(slot);
                if (s.running() && s.commandSeq() == startHomeSeq_[j]) {
                    return true;
                }
            }
        }
        return false;
    }
    /// One hold-to-run's tick (file header).
    void tickHold(std::size_t i, const Control& h, const hal::IController& c, bool connected) {
        const bool f = connected && c.pressed(h.button);
        const bool b = connected && c.pressed(h.back);
        Slot& s = svc_->slot(slot_[i]);
        if (startHomeRuns(slot_[i])) {
            // The driver-start home runs: this hold WAITS. Held now = SUPERSEDED — nothing until its
            // buttons are seen released, which the state below sees once the home has ended.
            if (f || b) {
                holdState_[i] = HoldState::Superseded;
            }
            return;
        }
        if (holdState_[i] == HoldState::Superseded) {
            if (!f && !b && connected) {
                holdState_[i] = HoldState::Released;  // seen released: it may drive again
            }
            return;
        }
        if (holdState_[i] == HoldState::Driving && s.commandSeq() != mySeq_[i]) {
            // Something newer spoke since this hold's last command: it is superseded while held.
            holdState_[i] = (f || b) ? HoldState::Superseded : HoldState::Released;
            return;
        }
        if (f != b) {
            const units::Voltage v{f ? h.volts : -h.volts};
            if (h.mechanismKind == Kind::Power) {
                svc_->power(slot_[i]).drive(v);
            } else {
                svc_->position(slot_[i]).jog(v);
            }
            holdState_[i] = HoldState::Driving;
            mySeq_[i] = s.commandSeq();
            return;
        }
        if (holdState_[i] == HoldState::Driving) {  // released, both held, or disconnected: once
            if (h.mechanismKind == Kind::Power) {
                svc_->power(slot_[i]).stop();
            } else {
                svc_->position(slot_[i]).holdHere();
            }
            holdState_[i] = HoldState::Released;
        }
    }

    MechanismService* svc_ = nullptr;
    const DriverControls* controls_ = nullptr;
    std::uint8_t slot_[kCap]{};
    HoldState holdState_[kCap]{};
    std::uint32_t mySeq_[kCap]{};
    std::uint32_t startHomeSeq_[kCap]{};  ///< a homeAtDriverStart's home: the slot's number after it began (0: none)
    hal::ButtonEdge edge_[kCap]{};
};

/// The driver file's control words, beside the robot file's (kinds.hpp, wiring.hpp): a driver file
/// whose robot namespace says `using namespace mechanism::words;` has them (teleop/teleop_loop.hpp adds
/// Button, DriverProfile, DriverLoopConfig, ArcadeBody and ArcadeField).
namespace words {
/// A driver file's checked list of controls.
using mechanism::DriverControls;
/// The list itself: `controls(kRobotXMechanisms, ...)`.
using mechanism::controls;
/// Hold-to-run: two buttons, + / − volts while held.
using mechanism::whileHeld;
/// One button flips a two-position pneumatic, or a flywheel's named speed on / off.
using mechanism::toggle;
/// One button: a preset, a constant, a speed or a pneumatic position.
using mechanism::preset;
/// One button homes an arm or a lift.
using mechanism::homeOnPress;
/// An arm or a lift nothing homed is homed once at driver-control start.
using mechanism::homeAtDriverStart;
/// No driver control, on purpose.
using mechanism::autonOnly;
}  // namespace words

}  // namespace shulib::mechanism
