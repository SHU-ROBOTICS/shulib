// The A2 plant's two ADDITIVE coupled-train extensions (chunk R3d Part 1, brief T6). Each case
// names the bug it catches and the mutation that must turn it red:
//   18  the EQUAL-COMMAND shortcut is the pre-R3d path, bit for bit: a five-member wheel whose
//       members all carry the same volts is byte-identical to a one-member wheel, across a long
//       mixed-voltage run                     (make the shortcut an unconditional mean: red here
//                                              AND in motor_group_plant_test 7's equivalence pin)
//   19  MemberFault::Reversed — a motor wired backwards through a POSITIVE port: its commanded
//       volts turn the train the other way AND its reading is the negation of the wheel's shaft,
//       so under +V the ROBOT reverses while the member's own encoder counts up. On an EXTRA
//       under an equal command the reading is negated and the plant is untouched
//                                             (drop the command negation; drop the read negation)
//   20  the coupled-train command model: one member of a five-motor train at 3 V with its mates
//       at 0 turns the wheel at exactly the 0.6 V mean — bit-identical to a lone wheel commanded
//       0.6 V                                  (take the primary's command instead of the mean)
//
// Bit-identity here is memcmp over the per-tick TruthSample stream and `==` on doubles — never
// Approx. Test 18's wider half is the WHOLE pre-existing suite: its assertion count before and
// after these extensions is recorded in the R3d progress log, and it did not move.
//
// R3d PART 2F-1 added two more ADDITIVE extensions, each off by default, and the cases below that
// make the plant able to FAIL the way robot two did (the Part 2F brief, §0 and F1.4):
//   21  coast-follow never touches a wheel that is being DRIVEN: with both wheels commanded,
//       κ = 0.7 is bit-identical to κ = 0             (apply the follow to every wheel: red)
//   22  coast-follow's shape: one side driven at v, the other coasting, gives ω = (1 − κ)·v / W
//       and vx = (1 + κ)·v / 2 — κ = 0 the pinned pivot, κ = 1 exactly ZERO yaw (the robot ROLLS)
//                                                  (follow the body's vx instead: red at κ = 0.5)
//   23  coast-follow reads the BRAKE MODE: a side on Brake or Hold at 0 V stays pinned, and ONE
//       member on Hold is enough                   (ignore the brake mode: red)
//   24  MemberFault::Decoupled: its command does not reach the train, its encoder reports its own
//       free spin, it starts from where it stood, and decoupledDrag carries a fraction of the
//       train's travel                             (let its volts into the mean: red)
//   25  TONIGHT, on the plant (the regression that documents 2026-09-24): at the tree's usual
//       kS = 1.2, one member of five at 3 V for 0.4 s turns the robot EXACTLY 0.00°; 6 V is still
//       exactly 0 (the dead band is inclusive); 9 V turns it 6.47° pinned and 0.00° at κ = 1
//   26  the new preconditions: κ and the drag in [0, 1]; κ > 0 only on a two-wheel kinematics
// THE PLANT'S NUMBERS ARE INVENTED (κ, kS, the drag): these cases prove the plant's RULES, which
// the calibrator's cases then use to prove the calibrator's branches — not the robot's physics.

#include "doctest.h"

#include <array>
#include <cstring>
#include <span>
#include <vector>

#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/sim/drive_plant.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::hal::BrakeMode;
using shulib::hal::fake::FakeMotor;
using shulib::kinematics::TankKinematics;
using shulib::sim::MemberFault;
using shulib::sim::SimHarness;
using shulib::sim::SimHarnessConfig;
using shulib::sim::TruthSample;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Voltage;

namespace {

constexpr double kDt = 0.01;

/// kS = 0 so a fractional mean voltage still MOVES the wheel: with the tree's usual kS = 1.2 a
/// 0.6 V mean sits inside the static-friction dead band and every mean-vs-primary difference
/// would be hidden behind a zero. kA = 0 keeps the model memoryless, so a tick's velocity is a
/// hand-derivable function of that tick's volts alone.
[[nodiscard]] SimHarnessConfig meanVisibleConfig() {
    SimHarnessConfig cfg;
    cfg.plant.wheelFf = {.kS = 0.0, .kV = 0.17, .kA = 0.0};
    return cfg;
}

/// A two-wheel tank plant with `perWheel` COUPLED members on each wheel; member 0 is the
/// harness's own primary FakeMotor and 1.. are extras this struct owns. `kin` must outlive it.
struct Train {
    static constexpr std::size_t kMaxExtra = 20;

    SimHarness h;
    std::array<FakeMotor, 2 * kMaxExtra> extra{};
    std::size_t perWheel;

    Train(const TankKinematics& kin, std::size_t members, const SimHarnessConfig& cfg)
        : h{kin, cfg}, perWheel{members} {
        for (int wheel = 0; wheel < 2; ++wheel) {
            std::array<FakeMotor*, kMaxExtra> ex{};
            for (std::size_t i = 1; i < perWheel; ++i) {
                ex[i - 1] = &extra[static_cast<std::size_t>(wheel) * kMaxExtra + (i - 1)];
            }
            h.plant().attachCoupledMembers(
                wheel, std::span<FakeMotor* const>{ex.data(), perWheel - 1});
        }
    }

    [[nodiscard]] FakeMotor& member(int wheel, std::size_t i) {
        return i == 0 ? h.motor(wheel)
                      : extra[static_cast<std::size_t>(wheel) * kMaxExtra + (i - 1)];
    }

    /// What a hal::MotorGroup does: the SAME volts to every member of the side.
    void commandAll(int wheel, double volts) {
        for (std::size_t i = 0; i < perWheel; ++i) {
            member(wheel, i).setVoltage(Voltage{volts});
        }
    }
};

/// The two sides' volts on tick `i`. Most doubles satisfy ((((v+v)+v)+v)+v)/5 == v exactly — so a
/// table of "irregular-looking" numbers would make the equal-command shortcut UNFALSIFIABLE and
/// its mutation a silent no-op. Every value below except the two round ones was SEARCHED FOR
/// (2 M uniform draws in ±12 V) as one where that identity FAILS, so the shortcut's bit claim is
/// a claim about bits that really can differ.
[[nodiscard]] double leftVoltsAt(int i) {
    const double table[] = {-6.642264849431651, 3.939652931392187,  -7.966838906242931,
                            11.9,               3.222949576457017,  -3.2733618711702963};
    return table[static_cast<std::size_t>(i / 7) % 6];
}
[[nodiscard]] double rightVoltsAt(int i) {
    const double table[] = {-7.056990892336163, 3.5310925086640506, 11.9,
                            -3.260077451321221, 3.939652931392187,  -6.642264849431651};
    return table[static_cast<std::size_t>(i / 5) % 6];
}

[[nodiscard]] bool sameTruth(const std::vector<TruthSample>& a, const std::vector<TruthSample>& b) {
    if (a.size() != b.size()) {
        return false;
    }
    return a.empty() || std::memcmp(a.data(), b.data(), a.size() * sizeof(TruthSample)) == 0;
}

}  // namespace

// ═══ 18. The equal-command shortcut is the pre-R3d path, bit for bit ════════════════
// Bug caught: the coupled-train model silently changing what every EXISTING coupled scenario
// does. A hal::MotorGroup fans ONE voltage to all five members, so the mean and the primary's
// value are the same number mathematically — but not the same BITS: ((((v+v)+v)+v)+v)/5 rounds
// where v does not. Every R3b coupled result rides on this shortcut.
TEST_CASE("DrivePlant 18: a five-member wheel under one fanned command is bit-identical to a "
          "one-member wheel") {
    const TankKinematics kin{Length{12.5}};
    const SimHarnessConfig cfg = meanVisibleConfig();
    Train one{kin, 1, cfg};
    Train five{kin, 5, cfg};

    std::vector<TruthSample> a, b;
    for (int i = 0; i < 400; ++i) {
        one.commandAll(0, leftVoltsAt(i));
        one.commandAll(1, rightVoltsAt(i));
        five.commandAll(0, leftVoltsAt(i));
        five.commandAll(1, rightVoltsAt(i));
        one.h.plant().step(Time{kDt});
        five.h.plant().step(Time{kDt});
        a.push_back(one.h.sample());
        b.push_back(five.h.sample());
        // Every member of the five reads the SAME shaft as the lone wheel's motor, exactly.
        for (std::size_t m = 0; m < 5; ++m) {
            REQUIRE(five.member(0, m).position().value() == one.member(0, 0).position().value());
            REQUIRE(five.member(1, m).velocity().value() == one.member(1, 0).velocity().value());
        }
    }
    CHECK(sameTruth(a, b));
    CHECK(a.back().x != 0.0);       // the run actually went somewhere …
    CHECK(a.back().theta != 0.0);   // … and turned, so both wheels were exercised

    // The shortcut is a BIT claim, not an arithmetic one: for these voltages the mean of five
    // equal values is genuinely a different double from the value itself, which is what makes
    // the mutation "unconditional mean" observable rather than a no-op. (For most doubles it is
    // NOT — hence the searched table above.)
    int differing = 0;
    for (int i = 0; i < 42; i += 7) {
        const double v = leftVoltsAt(i);
        if (((((v + v) + v) + v) + v) / 5.0 != v) {
            ++differing;
        }
    }
    CHECK(differing >= 5);
}

// ═══ 19. MemberFault::Reversed — wired backwards through a POSITIVE port ════════════
// Bug caught: modelling a backwards motor as a read-only negation (that is SignFlipped, and it
// leaves the command going the RIGHT way). A real motor mounted the other way round on the gear
// train drives the wheel backwards under +V while its own encoder counts up — and that pair is
// exactly what the calibrator's sign capture and its own-encoder cross-check must survive.
TEST_CASE("DrivePlant 19: a Reversed PRIMARY drives the train backwards under +V while its own "
          "encoder counts up; a Reversed EXTRA under an equal command changes only the reading") {
    const TankKinematics kin{Length{12.5}};
    const SimHarnessConfig cfg = meanVisibleConfig();

    // ── the primary half ──────────────────────────────────────────────────────────────
    Train healthy{kin, 1, cfg};
    Train reversed{kin, 1, cfg};
    reversed.h.plant().setMemberFault(0, 0, MemberFault::Reversed);
    reversed.h.plant().setMemberFault(1, 0, MemberFault::Reversed);
    for (int i = 0; i < 100; ++i) {
        healthy.commandAll(0, 4.0);
        healthy.commandAll(1, 4.0);
        reversed.commandAll(0, 4.0);
        reversed.commandAll(1, 4.0);
        healthy.h.plant().step(Time{kDt});
        reversed.h.plant().step(Time{kDt});
    }
    CHECK(healthy.h.truePose().x().value() > 5.0);                     // +V drives it forward …
    CHECK(reversed.h.truePose().x().value() < -5.0);                   // … and backwards when
    CHECK(reversed.h.truePose().x().value()                            //     both motors are
          == -healthy.h.truePose().x().value());                       //     wired backwards
    // The READING is the negation of the wheel's shaft — so with the robot reversing, the
    // motor's own encoder counts UP, exactly as a backwards-mounted motor's does.
    CHECK(reversed.member(0, 0).position().value() > 0.0);
    CHECK(reversed.member(0, 0).position().value()
          == healthy.member(0, 0).position().value());
    CHECK(reversed.member(0, 0).velocity().value()
          == healthy.member(0, 0).velocity().value());

    // A HAND PUSH is the other direction of the same fact: nothing is commanded, the robot is
    // shoved forward, and the backwards motor's encoder counts DOWN — which is the −1 the
    // calibrator's push capture must record for it.
    Train pushed{kin, 1, cfg};
    pushed.h.plant().setMemberFault(0, 0, MemberFault::Reversed);
    for (int i = 0; i < 50; ++i) {
        pushed.commandAll(0, -4.0);  // a reversed motor at −4 V rolls its wheel FORWARD
        pushed.commandAll(1, 4.0);
        pushed.h.plant().step(Time{kDt});
    }
    CHECK(pushed.h.truePose().x().value() > 2.0);           // the robot really went forward
    CHECK(pushed.member(0, 0).position().value() < 0.0);    // its encoder counted DOWN
    CHECK(pushed.member(1, 0).position().value() > 0.0);    // the healthy side counted up

    // ── the extra half: reading negated, plant untouched ─────────────────────────────
    Train cleanFive{kin, 5, cfg};
    Train faultedFive{kin, 5, cfg};
    faultedFive.h.plant().setMemberFault(0, 3, MemberFault::Reversed);
    std::vector<TruthSample> a, b;
    for (int i = 0; i < 200; ++i) {
        cleanFive.commandAll(0, leftVoltsAt(i));
        cleanFive.commandAll(1, rightVoltsAt(i));
        faultedFive.commandAll(0, leftVoltsAt(i));
        faultedFive.commandAll(1, rightVoltsAt(i));
        cleanFive.h.plant().step(Time{kDt});
        faultedFive.h.plant().step(Time{kDt});
        a.push_back(cleanFive.h.sample());
        b.push_back(faultedFive.h.sample());
    }
    CHECK(sameTruth(a, b));  // every member carried the same volts: the wheel never noticed
    CHECK(faultedFive.member(0, 3).position().value()
          == -cleanFive.member(0, 3).position().value());
    CHECK(faultedFive.member(0, 3).velocity().value()
          == -cleanFive.member(0, 3).velocity().value());
    CHECK(faultedFive.member(0, 2).position().value()
          == cleanFive.member(0, 2).position().value());  // its mates are untouched
    CHECK(faultedFive.member(0, 3).position().value() != 0.0);

    // On the READ side alone, Reversed and SignFlipped are indistinguishable — the difference
    // lives entirely in the command path, which is the whole reason Reversed had to be a NEW
    // member rather than a change to SignFlipped's semantics.
    Train flippedFive{kin, 5, cfg};
    flippedFive.h.plant().setMemberFault(0, 3, MemberFault::SignFlipped);
    for (int i = 0; i < 200; ++i) {
        flippedFive.commandAll(0, leftVoltsAt(i));
        flippedFive.commandAll(1, rightVoltsAt(i));
        flippedFive.h.plant().step(Time{kDt});
    }
    CHECK(flippedFive.member(0, 3).position().value()
          == faultedFive.member(0, 3).position().value());
}

// ═══ 20. The coupled-train command model ════════════════════════════════════════════
// Bug caught: the plant reading a wheel's command from its PRIMARY alone (the pre-R3d line). A
// calibrator that pulses ONE member of a five-motor train would then move nothing at all, and a
// test written against it would pass for the wrong reason — brief landmine 4.
TEST_CASE("DrivePlant 20: one member of a five-motor train at 3 V turns the wheel at exactly the "
          "0.6 V mean") {
    const TankKinematics kin{Length{12.5}};
    const SimHarnessConfig cfg = meanVisibleConfig();

    Train train{kin, 5, cfg};
    Train reference{kin, 1, cfg};
    std::vector<TruthSample> a, b;
    for (int i = 0; i < 150; ++i) {
        // ONE member per wheel driven; its four mates free-wheel at 0 V.
        train.commandAll(0, 0.0);
        train.commandAll(1, 0.0);
        train.member(0, 4).setVoltage(Voltage{3.0});
        train.member(1, 2).setVoltage(Voltage{3.0});
        reference.commandAll(0, 0.6);  // (3 + 0 + 0 + 0 + 0) / 5
        reference.commandAll(1, 0.6);
        train.h.plant().step(Time{kDt});
        reference.h.plant().step(Time{kDt});
        a.push_back(train.h.sample());
        b.push_back(reference.h.sample());
    }
    CHECK(sameTruth(a, b));
    CHECK(a.back().x > 1.0);  // the train MOVED — the pre-R3d plant would have sat still
    // And the arithmetic is the stated one: 0.6 V at kS = 0, kV = 0.17.
    CHECK(train.h.plant().trueWheelSpin()[0].value() == doctest::Approx(0.6 / 0.17).epsilon(1e-12));

    // The mean is over EVERY member, primary included: two members at 3 V is twice the speed.
    Train two{kin, 5, cfg};
    two.commandAll(0, 0.0);
    two.commandAll(1, 0.0);
    two.member(0, 0).setVoltage(Voltage{3.0});
    two.member(0, 4).setVoltage(Voltage{3.0});
    two.h.plant().step(Time{kDt});
    CHECK(two.h.plant().trueWheelSpin()[0].value() == doctest::Approx(1.2 / 0.17).epsilon(1e-12));

    // A Reversed member's volts enter the mean NEGATED — the command half of test 19, in the
    // path the calibrator's single-member pulse actually takes.
    Train rev{kin, 5, cfg};
    rev.h.plant().setMemberFault(0, 4, MemberFault::Reversed);
    rev.commandAll(0, 0.0);
    rev.commandAll(1, 0.0);
    rev.member(0, 4).setVoltage(Voltage{-3.0});  // the calibrator's sign[i] x pulseVolts
    rev.h.plant().step(Time{kDt});
    CHECK(rev.h.plant().trueWheelSpin()[0].value() == doctest::Approx(0.6 / 0.17).epsilon(1e-12));
    CHECK(rev.member(0, 4).position().value() < 0.0);  // reading negated: sign x it is positive
}

// ═══ R3d Part 2F-1 — the plant made able to FAIL like robot two ═════════════════════
namespace {

constexpr double kW = 12.5;         // the plant's TRUE track width in these cases
constexpr double kKv = 0.17;        // volts per in/s of wheel surface speed
constexpr double kR = 3.25 / 2.0;   // DrivePlantConfig::driveWheelDiameter / 2

/// kA = 0 (memoryless) with a chosen static-friction dead band and coast-follow fraction.
[[nodiscard]] SimHarnessConfig coastConfig(double kS, double kappa, double drag = 0.0) {
    SimHarnessConfig cfg;
    cfg.plant.wheelFf = {.kS = kS, .kV = kKv, .kA = 0.0};
    cfg.plant.coastFollow = kappa;
    cfg.plant.decoupledDrag = drag;
    return cfg;
}

/// The calibrator's single-member pulse, as the plant sees it: member `m` of the LEFT wheel at
/// `volts`, every other member of both wheels at 0 V + Coast, for `ticks` steps.
void pulseLeftMember(Train& t, std::size_t m, double volts, int ticks) {
    for (int i = 0; i < ticks; ++i) {
        t.commandAll(0, 0.0);
        t.commandAll(1, 0.0);
        t.member(0, m).setVoltage(Voltage{volts});
        t.h.plant().step(Time{kDt});
    }
}

}  // namespace

// ═══ 21. Coast-follow never touches a DRIVEN wheel ══════════════════════════════════
// Bug caught: the follow rule leaking onto wheels that are being commanded — every closed-loop
// scenario that ever turned on coast-follow would then drive a different robot.
TEST_CASE("DrivePlant 21: with both wheels driven, coast-follow at 0.7 is bit-identical to none") {
    const TankKinematics kin{Length{kW}};
    Train off{kin, 5, coastConfig(0.0, 0.0)};
    Train on{kin, 5, coastConfig(0.0, 0.7)};
    std::vector<TruthSample> a, b;
    for (int i = 0; i < 300; ++i) {
        off.commandAll(0, leftVoltsAt(i));  // every value in both tables is non-zero, so no
        off.commandAll(1, rightVoltsAt(i)); // wheel is ever coasting
        on.commandAll(0, leftVoltsAt(i));
        on.commandAll(1, rightVoltsAt(i));
        off.h.plant().step(Time{kDt});
        on.h.plant().step(Time{kDt});
        a.push_back(off.h.sample());
        b.push_back(on.h.sample());
    }
    CHECK(sameTruth(a, b));
    CHECK(a.back().theta != 0.0);  // it turned, so both wheels were exercised
}

// ═══ 22. Coast-follow's shape ═══════════════════════════════════════════════════════
// Bug caught: a follow rule of the wrong shape — the calibrator's ROLLED branch is proven against
// it, so its κ = 1 end must be exactly "no turn at all" and its κ = 0 end exactly the old pivot.
TEST_CASE("DrivePlant 22: one side driven at v and the other coasting gives omega = (1 - k) v / W "
          "and vx = (1 + k) v / 2; k = 1 ROLLS the robot with exactly zero yaw") {
    const TankKinematics kin{Length{kW}};
    const double v = (3.0 / 5.0) / kKv;  // one member of five at 3 V, kS = 0: the 0.6 V mean
    for (const double kappa : {0.0, 0.25, 0.5, 0.9, 1.0}) {
        CAPTURE(kappa);
        Train t{kin, 5, coastConfig(0.0, kappa)};
        pulseLeftMember(t, 4, 3.0, 40);
        const auto spin = t.h.plant().trueWheelSpin();
        CHECK(spin[0].value() == doctest::Approx(v).epsilon(1e-12));
        CHECK(spin[1].value() == doctest::Approx(kappa * v).epsilon(1e-12));
        const auto tw = t.h.trueBodyTwist();
        CHECK(tw.omega().value() == doctest::Approx((kappa - 1.0) * v / kW).epsilon(1e-12));
        CHECK(tw.vx().value() == doctest::Approx((1.0 + kappa) * v / 2.0).epsilon(1e-12));
        if (kappa == 1.0) {
            // Exactly zero, not approximately: the calibrator's ROLLED case depends on it.
            CHECK(t.h.sample().theta == 0.0);
            // … and the robot really went somewhere: all of v, for 0.4 s, straight ahead.
            CHECK(t.h.sample().x == doctest::Approx(v * 0.4).epsilon(1e-12));
        }
        // The coasting side's ENCODERS turn with it — the "other drive ports moved" signal.
        CHECK(t.member(1, 3).position().value()
              == doctest::Approx(kappa * v * 0.4 / kR).epsilon(1e-9));
    }
}

// ═══ 23. Coast-follow reads the brake mode ══════════════════════════════════════════
// Bug caught: the plant treating "0 V" as "coasting" — a held side would then roll along, and a
// test could never tell a calibrator that commands Coast from one that commands Hold.
TEST_CASE("DrivePlant 23: a side on Brake or Hold at 0 V stays pinned whatever k is, and ONE held "
          "member is enough") {
    const TankKinematics kin{Length{kW}};
    Train pinned{kin, 5, coastConfig(0.0, 0.0)};
    pulseLeftMember(pinned, 4, 3.0, 40);
    for (const BrakeMode mode : {BrakeMode::Brake, BrakeMode::Hold}) {
        for (const std::size_t held : {std::size_t{0}, std::size_t{3}}) {
            CAPTURE(static_cast<int>(mode));
            CAPTURE(held);
            Train t{kin, 5, coastConfig(0.0, 1.0)};
            t.member(1, held).setBrakeMode(mode);  // one member of the idle side resists
            pulseLeftMember(t, 4, 3.0, 40);
            CHECK(t.h.plant().trueWheelSpin()[1].value() == 0.0);
            CHECK(t.h.sample().theta == pinned.h.sample().theta);  // the pinned pivot, bit for bit
            CHECK(t.h.sample().x == pinned.h.sample().x);
        }
    }
    // The control: the same κ = 1 plant with every idle member coasting rolls instead.
    Train rolling{kin, 5, coastConfig(0.0, 1.0)};
    pulseLeftMember(rolling, 4, 3.0, 40);
    CHECK(rolling.h.sample().theta == 0.0);
    CHECK(pinned.h.sample().theta < 0.0);  // LEFT alone turns it clockwise (HA-134)
}

// ═══ 24. MemberFault::Decoupled — a slipping gear ═══════════════════════════════════
// Bug caught: modelling port 18's fault as a slow encoder (the train would still move under its
// command, and the calibrator's DECOUPLED branch could never be reached), or letting its reading
// jump when the gear lets go.
TEST_CASE("DrivePlant 24: a Decoupled member moves nothing, reads its own free spin from where it "
          "stood, and decoupledDrag carries a fraction of the train's travel") {
    const TankKinematics kin{Length{kW}};
    const double kS = 1.2;
    Train t{kin, 5, coastConfig(kS, 0.0)};
    // Roll the robot forward first, so the member has a non-zero reading to be anchored at.
    for (int i = 0; i < 30; ++i) {
        t.commandAll(0, 4.0);
        t.commandAll(1, 4.0);
        t.h.plant().step(Time{kDt});
    }
    t.commandAll(0, 0.0);
    t.commandAll(1, 0.0);
    t.h.plant().step(Time{kDt});
    const double before = t.member(0, 2).position().value();
    const double mateBefore = t.member(0, 1).position().value();
    const double xBefore = t.h.sample().x;
    REQUIRE(before > 1.0);
    t.h.plant().setMemberFault(0, 2, MemberFault::Decoupled);
    t.h.plant().step(Time{kDt});
    CHECK(t.member(0, 2).position().value() == before);  // no jump when the gear lets go

    // Its command reaches nothing: 9 V on it, 0 on its mates — the robot does not move at all …
    pulseLeftMember(t, 2, 9.0, 40);
    CHECK(t.h.sample().x == xBefore);
    CHECK(t.h.sample().theta == 0.0);
    CHECK(t.member(0, 1).position().value() == mateBefore);  // … its mates read nothing …
    // … and it reads its OWN free spin: (9 − kS) / kV in/s at the shaft, for 0.4 s.
    const double ownRad = (9.0 - kS) / kKv * 0.4 / kR;
    CHECK(t.member(0, 2).position().value() - before == doctest::Approx(ownRad).epsilon(1e-9));
    CHECK(t.member(0, 2).velocity().value() == doctest::Approx((9.0 - kS) / kKv / kR).epsilon(1e-12));

    // Driving its FOUR coupled mates moves the train at their common command — the Decoupled
    // member is out of the mean, not averaged in at 0 V (which would have given 4/5 of it).
    Train four{kin, 5, coastConfig(kS, 0.0)};
    Train reference{kin, 1, coastConfig(kS, 0.0)};
    four.h.plant().setMemberFault(0, 0, MemberFault::Decoupled);  // even the PRIMARY
    for (int i = 0; i < 50; ++i) {
        four.commandAll(0, 0.0);
        for (std::size_t m = 1; m < 5; ++m) {
            four.member(0, m).setVoltage(Voltage{5.0});
        }
        four.commandAll(1, 5.0);
        reference.commandAll(0, 5.0);
        reference.commandAll(1, 5.0);
        four.h.plant().step(Time{kDt});
        reference.h.plant().step(Time{kDt});
    }
    CHECK(four.h.sample().x == reference.h.sample().x);
    CHECK(four.h.sample().theta == reference.h.sample().theta);
    CHECK(four.member(0, 0).position().value() == 0.0);  // drag 0: it saw none of that travel

    // decoupledDrag: the same run with 0.7 of the train's travel reaching the slipping member —
    // robot two's port 18, 22–33 % short on a push.
    Train dragged{kin, 5, coastConfig(kS, 0.0, 0.7)};
    dragged.h.plant().setMemberFault(0, 3, MemberFault::Decoupled);
    for (int i = 0; i < 50; ++i) {
        dragged.commandAll(0, 0.0);
        for (const std::size_t m : {std::size_t{0}, std::size_t{1}, std::size_t{2}, std::size_t{4}}) {
            dragged.member(0, m).setVoltage(Voltage{5.0});
        }
        dragged.commandAll(1, 5.0);
        dragged.h.plant().step(Time{kDt});
    }
    const double train = dragged.member(0, 1).position().value();
    REQUIRE(train > 1.0);
    CHECK(dragged.member(0, 3).position().value() == doctest::Approx(0.7 * train).epsilon(1e-12));
}

// ═══ 25. Tonight, on the plant ══════════════════════════════════════════════════════
// The regression that DOCUMENTS 2026-09-24 (R3d-PROGRESS Part 3): the calibrator's own host rig
// ran at kS = 0 on purpose, and at the tree's usual kS = 1.2 the default single-member pulse
// turns this plant by exactly nothing — which is what robot two did at port 11.
TEST_CASE("DrivePlant 25: at kS = 1.2 one member of five at 3 V for 0.4 s turns the robot exactly "
          "0.00 deg; 6 V is still 0; 9 V turns it 6.47 deg pinned and 0.00 deg rolling") {
    const TankKinematics kin{Length{kW}};
    for (const double volts : {3.0, 6.0}) {
        CAPTURE(volts);
        Train t{kin, 5, coastConfig(1.2, 0.0)};
        pulseLeftMember(t, 4, volts, 40);
        CHECK(t.h.sample().theta == 0.0);  // exactly — the mean (volts / 5) is inside the dead band
        CHECK(t.h.sample().x == 0.0);
        CHECK(t.member(0, 4).position().value() == 0.0);  // not even the pulsed member's own shaft
    }
    Train nine{kin, 5, coastConfig(1.2, 0.0)};
    pulseLeftMember(nine, 4, 9.0, 40);
    const double expected = -((9.0 / 5.0 - 1.2) / kKv) / kW * 0.4;  // LEFT alone: clockwise
    CHECK(nine.h.sample().theta == doctest::Approx(expected).epsilon(1e-12));
    CHECK(nine.h.sample().theta * 180.0 / 3.14159265358979323846
          == doctest::Approx(-6.47).epsilon(0.001));
    Train rolled{kin, 5, coastConfig(1.2, 1.0)};
    pulseLeftMember(rolled, 4, 9.0, 40);
    CHECK(rolled.h.sample().theta == 0.0);
    CHECK(rolled.h.sample().x > 1.0);
}

// ═══ 26. The new preconditions ══════════════════════════════════════════════════════
TEST_CASE("DrivePlant 26: coast-follow and the decoupled drag are fractions, and coast-follow is "
          "refused on a kinematics that is not a two-wheel tank") {
    const TankKinematics kin{Length{kW}};
    CHECK_THROWS_AS((SimHarness{kin, coastConfig(0.0, -0.01)}), PreconditionError);
    CHECK_THROWS_AS((SimHarness{kin, coastConfig(0.0, 1.01)}), PreconditionError);
    CHECK_THROWS_AS((SimHarness{kin, coastConfig(0.0, 0.0, -0.1)}), PreconditionError);
    CHECK_THROWS_AS((SimHarness{kin, coastConfig(0.0, 0.0, 1.1)}), PreconditionError);
    CHECK_NOTHROW((SimHarness{kin, coastConfig(0.0, 1.0, 1.0)}));
    const auto x = shulib::kinematics::xDrive(Length{8.0});
    CHECK_THROWS_AS((SimHarness{x, coastConfig(0.0, 0.5)}), PreconditionError);
    CHECK_NOTHROW((SimHarness{x, coastConfig(0.0, 0.0)}));
}
