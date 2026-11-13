// The A2 plant's PER-WHEEL gains (chunk R5a Part A, A.6 — DrivePlantConfig::wheelFfOverride).
// Until R5a every wheel of the plant shared one `wheelFf`, so no host test could tell a
// drivetrain's LEFT from its RIGHT: a characterizer that swapped the sides, or fitted one side
// twice, produced the same numbers either way. This file pins the override, and pins that the
// DEFAULT — every slot empty — is the plant every earlier suite ran on, bit for bit. Each case
// names the bug it catches and the mutation that must turn it red:
//   P1  every slot filled with `wheelFf`'s OWN gains is memcmp-identical to the default plant,
//       per tick, on a tank AND an X-drive at kA > 0 — the override path computes what the shared
//       path computes                    (build the override model from different numbers: red)
//   P2  ONE wheel overridden, in both orientations: the other wheel's spin is bit-identical to a
//       default plant's, the overridden wheel's to a plant whose `wheelFf` IS the override
//       (apply the override to the wrong wheel: red; ignore the override: red; fall back to the
//        override for an EMPTY slot: red)
//   P3  the default path IS MotorModel(wheelFf): a default plant's wheel spin equals a hand-driven
//       MotorModel chain bit for bit     (change the default path's numbers: red)
//   P4  a Decoupled member on an overridden wheel spins free under THAT wheel's gains
//                                        (advanceDecoupled on the shared model: red)
//   P5  the override makes LEFT and RIGHT visible: a faster LEFT turns a tank clockwise, a faster
//       RIGHT counter-clockwise, and the two runs are mirror images
//   P6  the preconditions: a slot past wheelCount(), and gains MotorModel refuses
//   P7  under coast-follow (κ = 0.6) an overridden DRIVEN wheel turns under its own gains and the
//       coasting wheel follows it, both orientations   (the coast-follow path on the shared model:
//                                                        red)
// The wider half of the default's bit-identity is recorded, not asserted here: the WHOLE existing
// suite's counts before and after, and a pristine-tree hash of eight default runs (every
// MemberFault, coast-follow, the composed hostile world with a jittered dt) matching after the
// change — in the R5a progress log. A golden hash is not a CI test, because a libm change would
// turn it red (drive_plant.hpp's cross-libm caveat).
// THE GAINS BELOW ARE INVENTED, like every number the plant runs on.

#include "doctest.h"

#include <array>
#include <cstring>
#include <span>
#include <vector>

#include "shulib/control/feedforward.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/sim/drive_plant.hpp"
#include "shulib/sim/motor_model.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::control::FeedforwardGains;
using shulib::hal::fake::FakeMotor;
using shulib::kinematics::TankKinematics;
using shulib::sim::MemberFault;
using shulib::sim::MotorModel;
using shulib::sim::SimHarness;
using shulib::sim::SimHarnessConfig;
using shulib::sim::TruthSample;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;
using shulib::units::Voltage;

namespace {

constexpr double kDt = 0.01;
constexpr FeedforwardGains kShared{.kS = 1.0, .kV = 0.16, .kA = 0.02};
constexpr FeedforwardGains kOther{.kS = 1.1, .kV = 0.168, .kA = 0.024};

/// Volts on tick `i` for wheel `w`: ramps, steps, reversals, zeros inside and outside the dead
/// band, and a clamp — every branch of MotorModel::advance.
[[nodiscard]] double voltsAt(int w, int i) {
    const double t = kDt * i;
    switch ((i / 50) % 6) {
        case 0: return (w % 2 == 0 ? 2.0 : 1.7) * t;
        case 1: return w % 2 == 0 ? 6.0 : -6.0;
        case 2: return 0.0;
        case 3: return -9.5 + 0.3 * w;
        case 4: return 13.0;  // FakeMotor clamps at 12
        default: return 0.7 * w - 0.9;  // inside the dead band for small w
    }
}

[[nodiscard]] bool sameTruth(const std::vector<TruthSample>& a, const std::vector<TruthSample>& b) {
    if (a.size() != b.size()) {
        return false;
    }
    return a.empty() || std::memcmp(a.data(), b.data(), a.size() * sizeof(TruthSample)) == 0;
}

[[nodiscard]] SimHarnessConfig withShared() {
    SimHarnessConfig cfg;
    cfg.plant.wheelFf = kShared;
    return cfg;
}

/// Run `h` for `ticks` of voltsAt() and collect the truth and every wheel's spin.
struct Run {
    std::vector<TruthSample> truth;
    std::vector<double> spin[4];
    std::vector<double> encoder[4];
};
Run drive(SimHarness& h, int ticks) {
    Run r;
    for (int i = 0; i < ticks; ++i) {
        for (int w = 0; w < h.motorCount(); ++w) {
            h.motor(w).setVoltage(Voltage{voltsAt(w, i)});
        }
        h.plant().step(Time{kDt});
        r.truth.push_back(h.sample());
        const auto spin = h.plant().trueWheelSpin();
        for (int w = 0; w < h.motorCount(); ++w) {
            r.spin[w].push_back(spin[w].value());
            r.encoder[w].push_back(h.motor(w).position().value());
        }
    }
    return r;
}

[[nodiscard]] bool sameBits(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != b.size()) {
        return false;
    }
    return a.empty() || std::memcmp(a.data(), b.data(), a.size() * sizeof(double)) == 0;
}

}  // namespace

// ═══ P1. Every slot filled with wheelFf's own gains is the default plant ═════════════
// Bug caught: an override path whose arithmetic differs from the shared path's — a copy of the
// gains through a float, a re-derived model, a different step. Then a test that overrides only to
// make the sides differ would also be testing a different MOTOR MODEL, and every "left vs right"
// conclusion would carry that difference in it.
TEST_CASE("DrivePlant P1: every slot overridden with wheelFf's own gains is bit-identical to the "
          "default plant") {
    SUBCASE("tank") {
        const TankKinematics kin{Length{12.5}};
        SimHarnessConfig filled = withShared();
        filled.plant.wheelFfOverride[0] = kShared;
        filled.plant.wheelFfOverride[1] = kShared;
        SimHarness a{kin, withShared()};
        SimHarness b{kin, filled};
        const Run ra = drive(a, 600);
        const Run rb = drive(b, 600);
        CHECK(sameTruth(ra.truth, rb.truth));
        for (int w = 0; w < 2; ++w) {
            CHECK(sameBits(ra.spin[w], rb.spin[w]));
            CHECK(sameBits(ra.encoder[w], rb.encoder[w]));
        }
        CHECK(ra.truth.back().x != 0.0);  // it went somewhere …
        CHECK(ra.truth.back().theta != 0.0);  // … and turned, so both wheels were exercised
    }
    SUBCASE("X-drive, all four wheels") {
        const auto kin = shulib::kinematics::xDrive(Length{7.0});
        SimHarnessConfig filled = withShared();
        for (int w = 0; w < 4; ++w) {
            filled.plant.wheelFfOverride[static_cast<std::size_t>(w)] = kShared;
        }
        SimHarness a{kin, withShared()};
        SimHarness b{kin, filled};
        const Run ra = drive(a, 600);
        const Run rb = drive(b, 600);
        CHECK(sameTruth(ra.truth, rb.truth));
        for (int w = 0; w < 4; ++w) {
            CHECK(sameBits(ra.spin[w], rb.spin[w]));
        }
    }
}

// ═══ P2. One wheel overridden: that wheel, and only that wheel ══════════════════════
// Bug caught: the override landing on the wrong wheel (an index reversed, a LEFT/RIGHT swap), the
// override being ignored, or an EMPTY slot falling back to a filled one instead of `wheelFf`. On a
// tank each wheel's spin depends on its own command alone, so the other wheel must match a
// default plant EXACTLY and the overridden one must match a plant whose shared gains ARE the
// override — both orientations, because a swap is invisible in a symmetric test.
TEST_CASE("DrivePlant P2: an override moves exactly its own wheel, in both orientations") {
    const TankKinematics kin{Length{12.5}};
    for (int overridden = 0; overridden < 2; ++overridden) {
        CAPTURE(overridden);
        const int other = 1 - overridden;
        SimHarnessConfig one = withShared();
        one.plant.wheelFfOverride[static_cast<std::size_t>(overridden)] = kOther;
        SimHarnessConfig allOther;
        allOther.plant.wheelFf = kOther;

        SimHarness plantOne{kin, one};
        SimHarness plantShared{kin, withShared()};
        SimHarness plantOther{kin, allOther};
        const Run r1 = drive(plantOne, 600);
        const Run rs = drive(plantShared, 600);
        const Run ro = drive(plantOther, 600);
        CHECK(sameBits(r1.spin[other], rs.spin[other]));
        CHECK(sameBits(r1.spin[overridden], ro.spin[overridden]));
        CHECK(sameBits(r1.encoder[overridden], ro.encoder[overridden]));
        // …and the override is not a no-op: the two gain sets really do differ on this schedule.
        CHECK_FALSE(sameBits(rs.spin[overridden], ro.spin[overridden]));
    }
}

// ═══ P3. The default path IS MotorModel(wheelFf) ════════════════════════════════════
// Bug caught: any numeric change to the default path — a per-wheel model built from different
// numbers, a changed step — which P1 alone could miss if both of its arms moved together. The
// reference here is the MotorModel class driven by hand, one advance per tick, from the volts the
// wheel actually received (FakeMotor's clamp included).
TEST_CASE("DrivePlant P3: a default plant's wheel spin is MotorModel(wheelFf) advanced by hand, "
          "bit for bit") {
    const TankKinematics kin{Length{12.5}};
    SimHarness h{kin, withShared()};
    const MotorModel reference{kShared};
    double v[2] = {0.0, 0.0};
    for (int i = 0; i < 600; ++i) {
        for (int w = 0; w < 2; ++w) {
            h.motor(w).setVoltage(Voltage{voltsAt(w, i)});
        }
        h.plant().step(Time{kDt});
        const auto spin = h.plant().trueWheelSpin();
        for (int w = 0; w < 2; ++w) {
            v[w] = reference.advance(Velocity{v[w]}, h.motor(w).commandedVoltage(), Time{kDt}).value();
            REQUIRE(spin[w].value() == v[w]);  // bits, not Approx
        }
    }
}

// ═══ P4. A Decoupled member spins free under ITS wheel's gains ══════════════════════
// Bug caught: advanceDecoupled() left on the shared model — a slipping member on an overridden
// side would then spin at the OTHER side's speed, which is exactly the left/right confusion A.6
// exists to make visible.
TEST_CASE("DrivePlant P4: a Decoupled member on an overridden wheel spins free under that "
          "wheel's gains") {
    const TankKinematics kin{Length{12.5}};
    SimHarnessConfig cfg = withShared();
    cfg.plant.wheelFfOverride[1] = kOther;
    SimHarness h{kin, cfg};
    std::array<FakeMotor, 1> extra{};
    std::array<FakeMotor*, 1> ptr{&extra[0]};
    h.plant().attachCoupledMembers(1, std::span<FakeMotor* const>{ptr.data(), ptr.size()});
    h.plant().setMemberFault(1, 1, MemberFault::Decoupled);
    const MotorModel own{kOther};
    const double r = 0.5 * 3.25;  // DrivePlantConfig::driveWheelDiameter / 2
    double spin = 0.0;
    double shaft = 0.0;
    for (int i = 0; i < 300; ++i) {
        h.motor(1).setVoltage(Voltage{0.0});   // the train is not driven …
        extra[0].setVoltage(Voltage{7.0});     // … only the slipping member is
        h.plant().step(Time{kDt});
        spin = own.advance(Velocity{spin}, Voltage{7.0}, Time{kDt}).value();
        shaft += spin * kDt / r;
        REQUIRE(extra[0].velocity().value() == spin / r);  // bits: drag 0 adds exactly nothing
    }
    CHECK(extra[0].position().value() == shaft);
    CHECK(h.plant().trueWheelSpin()[1].value() == 0.0);  // its command never reached the train
}

// ═══ P5. LEFT and RIGHT are now distinguishable ═════════════════════════════════════
// Bug caught: nothing in A.6 itself — this is the property the characterizer's tests rely on,
// stated where the plant is: under equal volts a tank with a faster LEFT turns clockwise
// (negative yaw, CCW-positive frame), a faster RIGHT counter-clockwise, by the same amount.
TEST_CASE("DrivePlant P5: equal volts on unequal sides curve a tank toward the slower side, "
          "mirror-image in the two orientations") {
    const TankKinematics kin{Length{12.5}};
    double theta[2] = {0.0, 0.0};
    for (int faster = 0; faster < 2; ++faster) {
        SimHarnessConfig cfg = withShared();
        cfg.plant.wheelFfOverride[static_cast<std::size_t>(faster)] =
            FeedforwardGains{.kS = 1.0, .kV = 0.15, .kA = 0.02};  // a smaller kV is a faster side
        SimHarness h{kin, cfg};
        for (int i = 0; i < 200; ++i) {
            h.motor(0).setVoltage(Voltage{6.0});
            h.motor(1).setVoltage(Voltage{6.0});
            h.plant().step(Time{kDt});
        }
        theta[faster] = h.sample().theta;
    }
    CHECK(theta[0] < 0.0);  // LEFT faster: clockwise
    CHECK(theta[1] > 0.0);  // RIGHT faster: counter-clockwise
    CHECK(theta[0] == -theta[1]);
}

// ═══ P7. The override on the coast-follow path ══════════════════════════════════════
// Bug caught (the R5a review of 2026-10-04): the coast-follow step (DrivePlantConfig::coastFollow >
// 0) is the THIRD place a wheel is advanced, and no case ran it with an override — it could turn
// the driven wheel on the shared gains with every earlier case green. Here one wheel is overridden
// and DRIVEN, the other coasts (0 V + Coast) and follows κ = 0.6 of it; the driven wheel must spin
// bit for bit as on a coast-follow plant whose `wheelFf` IS the override, and the follower with it.
// Both orientations.
TEST_CASE("DrivePlant P7: under coast-follow an overridden driven wheel turns under its own gains, "
          "and the coasting wheel follows it, in both orientations") {
    const TankKinematics kin{Length{12.5}};
    for (int driven = 0; driven < 2; ++driven) {
        CAPTURE(driven);
        const int coasting = 1 - driven;
        SimHarnessConfig one = withShared();
        one.plant.coastFollow = 0.6;
        one.plant.wheelFfOverride[static_cast<std::size_t>(driven)] = kOther;
        SimHarnessConfig allOther;
        allOther.plant.wheelFf = kOther;
        allOther.plant.coastFollow = 0.6;
        SimHarnessConfig shared = withShared();
        shared.plant.coastFollow = 0.6;
        auto runOne = [&](const SimHarnessConfig& cfg) {
            SimHarness h{kin, cfg};
            Run r;
            for (int i = 0; i < 600; ++i) {
                // Never 0 V on the driven wheel: with BOTH wheels coasting nothing is followed and
                // each decays on its own model, which would test the coasting wheel's gains instead.
                const double v = voltsAt(driven, i);
                h.motor(driven).setBrakeMode(shulib::hal::BrakeMode::Coast);
                h.motor(driven).setVoltage(Voltage{v == 0.0 ? 3.0 : v});
                h.motor(coasting).setBrakeMode(shulib::hal::BrakeMode::Coast);
                h.motor(coasting).setVoltage(Voltage{0.0});
                h.plant().step(Time{kDt});
                const auto spin = h.plant().trueWheelSpin();
                for (int w = 0; w < 2; ++w) {
                    r.spin[w].push_back(spin[w].value());
                }
            }
            return r;
        };
        const Run r1 = runOne(one);
        const Run ro = runOne(allOther);
        const Run rs = runOne(shared);
        CHECK(sameBits(r1.spin[driven], ro.spin[driven]));
        CHECK(sameBits(r1.spin[coasting], ro.spin[coasting]));
        CHECK_FALSE(sameBits(r1.spin[driven], rs.spin[driven]));  // the override is not a no-op
        // …and the follower really follows (the case exercises coast-follow, not a pinned wheel).
        bool followed = false;
        for (std::size_t i = 0; i < r1.spin[driven].size(); ++i) {
            if (r1.spin[driven][i] != 0.0 && r1.spin[coasting][i] == 0.6 * r1.spin[driven][i]) {
                followed = true;
            }
        }
        CHECK(followed);
    }
}

// ═══ P6. The preconditions ═══════════════════════════════════════════════════════════
TEST_CASE("DrivePlant P6: an override past wheelCount(), or with gains MotorModel refuses, is a "
          "precondition breach") {
    const TankKinematics kin{Length{12.5}};
    SimHarnessConfig past = withShared();
    past.plant.wheelFfOverride[2] = kOther;  // a tank has wheels 0 and 1
    CHECK_THROWS_AS(SimHarness(kin, past), PreconditionError);
    SimHarnessConfig zeroKv = withShared();
    zeroKv.plant.wheelFfOverride[0] = FeedforwardGains{.kS = 1.0, .kV = 0.0, .kA = 0.0};
    CHECK_THROWS_AS(SimHarness(kin, zeroKv), PreconditionError);
    SimHarnessConfig negativeKa = withShared();
    negativeKa.plant.wheelFfOverride[1] = FeedforwardGains{.kS = 1.0, .kV = 0.16, .kA = -0.01};
    CHECK_THROWS_AS(SimHarness(kin, negativeKa), PreconditionError);
}
