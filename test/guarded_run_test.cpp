// guarded_run_test.cpp — the pieces every auton-running program shares, as COMP-b's first step
// moved them into src/common/ (the COMP brief's ND3 (b), 2026-09-30): the boot's chain proof and
// the program-error stop (src/common/guarded_run.hpp), the touch reduction
// (src/common/screen_touch.hpp), the status bits (src/common/state_screens.hpp) and the painters'
// PROS-free half (src/common/screen_paint.hpp). Every case names the bug it would catch.
//
// "shulib Auton Tester"'s own tests still run the same code through the tester's call sites
// (test/auton_abort_watch_test.cpp (6b), (13b); test/auton_tester_run_test.cpp (21c)). THESE cases
// prove what those cannot: that the pieces are GENERIC — the chain proof over a watch that is not
// the tester's, the stop with no watch at all — which is what the competition program (COMP-b)
// relies on. Since the mechanism core's MECH-A5 (2026-10-29) the proof has a THIRD layer, the
// robot's mechanism service, and the stop cancels each mechanism's registered claimant: (G1) is the
// proof that a chain WITHOUT the service — the MECH brief's P1 control case — fails (mutation M28),
// and (G2b) that a service slot driving a pivot when the throw came cannot command it again. Host
// test plant only; nothing here has run on a robot.

#include "doctest.h"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <regex>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "auton_tester_rig.hpp"
#include "common/guarded_run.hpp"
#include "common/screen_paint.hpp"
#include "common/screen_touch.hpp"
#include "common/state_screens.hpp"
#include "motion_test_rig.hpp"
#include "programs/auton_abort_watch.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/hal/fake/fake_mechanism.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/units/quantity.hpp"

using shulib::bench::ChainGap;
using shulib::bench::chainGapWords;
using shulib::bench::kGuardedTickSeconds;
using shulib::bench::pacerChainMissing;
using shulib::bench::pacerChainProven;
using shulib::bench::screenTouchFrom;
using shulib::bench::ScreenTouch;
using shulib::bench::stopAfterProgramError;
using shulib::bench::WatchLike;
using shulib::kinematics::TankKinematics;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::mechanism::MechanismService;
using shulib::sequence::RunGuard;
using shulib::units::Length;
using shulib::units::Time;

namespace {

/// A watch that is NOT the tester's: it counts every pace and remembers the guard it was given —
/// the two things the chain proof reads, and nothing else (the competition program's watch has the
/// same two, over other causes).
struct MinimalWatch final : shulib::motion::ITickPacer {
    explicit MinimalWatch(shulib::motion::ITickPacer& inner) : inner_{&inner} {}
    void attachGuard(RunGuard& g) noexcept { guard_ = &g; }
    [[nodiscard]] bool attachedTo(const RunGuard& g) const noexcept { return guard_ == &g; }
    [[nodiscard]] std::uint64_t paces() const noexcept { return paces_; }
    void pace() override {
        ++paces_;
        inner_->pace();
    }

private:
    shulib::motion::ITickPacer* inner_;
    RunGuard* guard_ = nullptr;
    std::uint64_t paces_ = 0;
};

/// A type the proof could not read (no attachedTo): it must not satisfy the concept.
struct NotAWatch {
    [[nodiscard]] std::uint64_t paces() const noexcept { return 0; }
};

static_assert(WatchLike<MinimalWatch>);
static_assert(WatchLike<shulib::bench::AbortWatch>, "the tester's watch must still qualify");
static_assert(!WatchLike<NotAWatch>, "a watch the proof cannot ask about its guard is refused");

const TankKinematics& tank() {
    static const TankKinematics k{Length{12.0}};
    return k;
}

/// Every drive motor at 0 V under Brake: the program-error stop's safe state.
bool driveSafe(motion_rig::MotionRig& rig) {
    for (const shulib::hal::IMotor* m : rig.h.context().driveMotors()) {
        if (m->commandedVoltage().value() != 0.0
            || m->brakeMode() != shulib::hal::BrakeMode::Brake) {
            return false;
        }
    }
    return true;
}

struct ThrowingMechanism final : shulib::hal::IMechanism {
    int calls = 0;
    void applySafeState() override {
        ++calls;
        throw std::runtime_error("test: a mechanism's safe state threw");
    }
    [[nodiscard]] const char* name() const noexcept override { return "thrower"; }
};

/// PROS's touch struct, by its field names (include/pros/screen.h).
enum FakeTouchStatus { kReleased = 0, kPressed, kHeld, kError };
struct FakeProsTouch {
    FakeTouchStatus touch_status = kReleased;  // NOLINT: PROS's name
    std::int16_t x = 0;
    std::int16_t y = 0;
    std::int32_t press_count = 0;    // NOLINT: PROS's name
    std::int32_t release_count = 0;  // NOLINT: PROS's name
};

std::string readSource(const std::string& rel) {
    std::ifstream in(std::filesystem::path(SHULIB_SOURCE_DIR) / rel);
    REQUIRE_MESSAGE(in.good(), "cannot open ", rel);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

}  // namespace

// (G1) Bug caught: the chain proof written for the tester's watch alone (the competition program's
// could not use it — the brief's "never copied" would force a copy), or a proof that passes a
// wrong chain. Over a watch that is NOT the tester's, with the robot's MECHANISM SERVICE between
// the guard and the watch (MECH-A5): the right chain proves; a service NEVER BOUND (it ticks
// nothing), a service LEFT OUT of the chain (the MECH brief's P1 control case: the guard paces the
// watch directly — mutation M28, a proof that ignores the service, passes it), a watch outside the
// service, a Chassis on the service (the guard never paces), on the watch, on the plant, a watch
// never attached and a watch attached to ANOTHER guard each FAIL — each naming its gap, in the
// proof's order — and no proof commands the drive.
TEST_CASE("common (G1): pacerChainMissing — guard -> service -> watch, the right chain only, each gap named") {
    motion_rig::MotionRig rig{tank()};
    motion_rig::PlantPacer plant{rig.h};
    MinimalWatch watch{plant};
    tester_rig::RobotTwoFakes two;
    MechanismService svc{watch};
    RunGuard guard{svc};
    shulib::chassis::Chassis onChain{rig.deps, guard, motion_rig::chassisConfig()};
    watch.attachGuard(guard);
    // The service UNBOUND: the guard paces it, it forwards to the watch, and it ticks nothing.
    CHECK(pacerChainMissing(onChain, guard, svc, watch) == ChainGap::Service);
    CHECK_FALSE(pacerChainProven(onChain, guard, svc, watch));
    svc.bind(mech_fixture::robot_two::kRobotTwoMechanisms, two.devs, rig.h.clock(), rig.latch, rig.faultSink);
    CHECK(pacerChainMissing(onChain, guard, svc, watch) == ChainGap::None);
    CHECK(pacerChainProven(onChain, guard, svc, watch));
    // The back link: attached to ANOTHER guard, then never attached (a second chain of the same shape).
    RunGuard other{plant};
    watch.attachGuard(other);
    CHECK(pacerChainMissing(onChain, guard, svc, watch) == ChainGap::NotAttached);
    watch.attachGuard(guard);
    CHECK(pacerChainProven(onChain, guard, svc, watch));
    MinimalWatch unattached{plant};
    MechanismService svc2{unattached};
    svc2.bind(shulib::bench::kNoMechanisms, {}, rig.h.clock(), rig.latch, rig.faultSink);
    RunGuard guard2{svc2};
    shulib::chassis::Chassis onChain2{rig.deps, guard2, motion_rig::chassisConfig()};
    CHECK(pacerChainMissing(onChain2, guard2, svc2, unattached) == ChainGap::NotAttached);
    // THE P1 CONTROL CASE: the guard paces the WATCH, the service is bound but outside the chain.
    RunGuard skips{watch};
    shulib::chassis::Chassis onSkip{rig.deps, skips, motion_rig::chassisConfig()};
    watch.attachGuard(skips);
    CHECK(pacerChainMissing(onSkip, skips, svc, watch) == ChainGap::Service);
    watch.attachGuard(guard);
    // TWO gaps at once — the guard paces the PLANT, so neither the service nor the watch is in the
    // chain: the FIRST in the proof's order is named (the service), never a later one (MECH-A5's
    // review, mutation R11: the service and watch checks swapped passed every one-gap chain above).
    RunGuard overPlant{plant};
    shulib::chassis::Chassis onOverPlant{rig.deps, overPlant, motion_rig::chassisConfig()};
    watch.attachGuard(overPlant);
    CHECK(pacerChainMissing(onOverPlant, overPlant, svc, watch) == ChainGap::Service);
    watch.attachGuard(guard);
    // The watch outside the service: guard -> service -> the plant.
    MinimalWatch aside{plant};
    MechanismService straight{plant};
    straight.bind(shulib::bench::kNoMechanisms, {}, rig.h.clock(), rig.latch, rig.faultSink);
    RunGuard guard3{straight};
    aside.attachGuard(guard3);
    shulib::chassis::Chassis onStraight{rig.deps, guard3, motion_rig::chassisConfig()};
    CHECK(pacerChainMissing(onStraight, guard3, straight, aside) == ChainGap::Watch);
    // A Chassis on the service, on the watch, on the plant: the guard never paces.
    shulib::chassis::Chassis onService{rig.deps, svc, motion_rig::chassisConfig()};
    CHECK(pacerChainMissing(onService, guard, svc, watch) == ChainGap::Guard);
    shulib::chassis::Chassis onWatch{rig.deps, watch, motion_rig::chassisConfig()};
    CHECK(pacerChainMissing(onWatch, guard, svc, watch) == ChainGap::Guard);
    shulib::chassis::Chassis onPlant{rig.deps, plant, motion_rig::chassisConfig()};
    CHECK(pacerChainMissing(onPlant, guard, svc, watch) == ChainGap::Guard);
    CHECK(plant.distance == 0.0);
    for (const shulib::hal::IMotor* m : rig.h.context().driveMotors()) {
        CHECK(m->commandedVoltage().value() == 0.0);
    }
    CHECK(two.m6.commandedVoltage().value() == 0.0);  // no proof commands a mechanism either
    // One proof is ONE idle tick of the program's own tick (the plant's clock says so).
    const double t0 = rig.h.clock().now().value();
    CHECK(pacerChainProven(onChain, guard, svc, watch));
    CHECK(rig.h.clock().now().value() - t0 == doctest::Approx(kGuardedTickSeconds));
    // The refusal's words name each gap (what the programs print after "the pacer chain is WRONG").
    CHECK(std::string_view{chainGapWords(ChainGap::Service)}
          == "one idle tick did not tick the mechanism service: it is not under the run guard, or it was "
             "never bound");
    CHECK(std::string_view{chainGapWords(ChainGap::Guard)}
          == "one idle tick did not pace the run guard: the Chassis is not built on it");
    CHECK(std::string_view{chainGapWords(ChainGap::Watch)}
          == "one idle tick did not pace the watch: it is not under the mechanism service");
    CHECK(std::string_view{chainGapWords(ChainGap::NotAttached)}
          == "the watch is not attached to the run guard: an abort would ask nobody");
}

// (G2) Bug caught: the program-error stop needing a watch (the competition program has its own),
// leaving the drive energized, skipping the mechanisms after one that throws, tripping on a null,
// or throwing itself (out of a PROS task function that is std::terminate). With a live motion and
// with a direct drive command.
TEST_CASE("common (G2): stopAfterProgramError(chassis, present) — drive safe, every mechanism, no throw") {
    for (const bool withMotion : {false, true}) {
        CAPTURE(withMotion);
        motion_rig::MotionRig rig{tank()};
        motion_rig::PlantPacer plant{rig.h};
        shulib::chassis::Chassis chassis{rig.deps, plant, motion_rig::chassisConfig()};
        shulib::motion::MoveToPose leg{chassis.deps(), Pose2d{Length{30.0}, Length{0.0}, Angle{}},
                                       motion_rig::motionConfig(), 5.0};
        if (withMotion) {
            chassis.scheduler().async(leg);
            chassis.wait(Time{0.1});
            REQUIRE(chassis.scheduler().hasActiveMotion());
        } else {
            chassis.drive(shulib::math::ChassisSpeeds{shulib::units::Velocity{20.0},
                                                      shulib::units::Velocity{0.0},
                                                      shulib::units::AngularVelocity{0.0}},
                          shulib::math::Frame::Body);
        }
        REQUIRE_FALSE(driveSafe(rig));
        shulib::hal::fake::FakeMechanism first{"first"};
        ThrowingMechanism middle;
        shulib::hal::fake::FakeMechanism last{"last"};
        shulib::hal::IMechanism* present[] = {&first, nullptr, &middle, &last};
        CHECK_NOTHROW(
            stopAfterProgramError(chassis, std::span<shulib::hal::IMechanism* const>{present}));
        CHECK(driveSafe(rig));
        CHECK_FALSE(chassis.scheduler().hasActiveMotion());
        CHECK(first.safeStateApplications() == 1);
        CHECK(middle.calls == 1);
        CHECK(last.safeStateApplications() == 1);  // after the thrower and the null: still safed
        // An empty span is legal (a robot with no mechanism built): the drive still stops.
        chassis.drive(shulib::math::ChassisSpeeds{shulib::units::Velocity{20.0},
                                                  shulib::units::Velocity{0.0},
                                                  shulib::units::AngularVelocity{0.0}},
                      shulib::math::Frame::Body);
        REQUIRE_FALSE(driveSafe(rig));
        CHECK_NOTHROW(stopAfterProgramError(chassis, {}));
        CHECK(driveSafe(rig));
    }
    static_assert(noexcept(stopAfterProgramError(std::declval<shulib::chassis::Chassis&>(),
                                                 std::span<shulib::hal::IMechanism* const>{})),
                  "the program-error stop must never throw");
}

// (G2b) Bug caught (the MECH brief's §1.13; MECH-A5): the program-error stop SAFING a mechanism but
// leaving its owner live — "the half-safe failure" hal/mechanism.hpp describes. A mechanism-service
// slot driving claw pivot 6 at 12 V when the throw came: a safe state alone lasts one tick, then the
// slot (still active, still holding the claim) commands 12 V again. The stop must also CANCEL the
// registered claimant: the pivot at 0 V under Hold, the slot inert and its claim released, and the
// NEXT service tick leaves it there. And an F1-style claimant on another mechanism is cancelled too,
// once; one that throws skips no other; a mechanism with no claimant is only safed; and a mechanism
// whose SAFE STATE throws still has its claimant cancelled (MECH-A5's review, mutation R1: the safe
// state and the cancel in ONE try — the throw would skip the cancel, and the slot driving that
// mechanism would command it again at the next tick).
TEST_CASE("common (G2b): stopAfterProgramError cancels each present mechanism's claimant -- a slot "
          "that was driving cannot command again") {
    motion_rig::MotionRig rig{tank()};
    motion_rig::PlantPacer plant{rig.h};
    tester_rig::RobotTwoFakes two;
    MechanismService svc{plant};
    svc.bind(mech_fixture::robot_two::kRobotTwoMechanisms, two.devs, rig.h.clock(), rig.latch, rig.faultSink);
    shulib::chassis::Chassis chassis{rig.deps, svc, motion_rig::chassisConfig()};
    svc.power(0).drive(shulib::units::Voltage{12.0});  // a hold-to-run on pivot 6, as R1 held leaves it
    chassis.wait(Time{0.02});
    REQUIRE(two.m6.commandedVoltage().value() == 12.0);
    REQUIRE(two.pivot6.claimed());
    REQUIRE(two.pivot6.claimant() == &svc.slot(0));
    struct Counting final : shulib::hal::ICancellable {
        shulib::hal::IMechanism* mech = nullptr;
        int cancels = 0;
        bool throws = false;
        void cancel() override {
            ++cancels;
            if (throws) {
                throw std::runtime_error("test: a claimant's cancel threw");
            }
            mech->releaseClaim();
        }
    };
    shulib::hal::fake::FakeMechanism op{"op"};
    Counting opClaimant;
    opClaimant.mech = &op;
    REQUIRE(op.tryClaim(opClaimant));
    shulib::hal::fake::FakeMechanism thrower{"thrower"};
    Counting throwing;
    throwing.mech = &thrower;
    throwing.throws = true;
    REQUIRE(thrower.tryClaim(throwing));
    shulib::hal::fake::FakeMechanism plain{"plain"};
    ThrowingMechanism safeThrows;  // its applySafeState() throws — and it is claimed
    Counting safeThrowsClaimant;
    safeThrowsClaimant.mech = &safeThrows;
    REQUIRE(safeThrows.tryClaim(safeThrowsClaimant));
    shulib::hal::IMechanism* present[] = {&safeThrows, &thrower, &two.pivot6, &op, &plain};
    CHECK_NOTHROW(stopAfterProgramError(chassis, std::span<shulib::hal::IMechanism* const>{present}));
    CHECK(safeThrows.calls == 1);               // its safe state was tried (and threw) ...
    CHECK(safeThrowsClaimant.cancels == 1);     // ... and its claimant was STILL cancelled
    CHECK_FALSE(safeThrows.claimed());
    CHECK(two.m6.commandedVoltage().value() == 0.0);
    CHECK(two.m6.brakeMode() == shulib::hal::BrakeMode::Hold);
    CHECK_FALSE(two.pivot6.claimed());  // the slot's cancel released it
    CHECK(opClaimant.cancels == 1);
    CHECK_FALSE(op.claimed());
    CHECK(throwing.cancels == 1);       // it threw -- and skipped nothing after it
    CHECK(plain.safeStateApplications() == 1);
    // THE HALF-SAFE CHECK: the service ticks on, and the pivot stays where the stop put it.
    for (int i = 0; i < 5; ++i) {
        chassis.wait(Time{0.01});
        CHECK(two.m6.commandedVoltage().value() == 0.0);
    }
    CHECK_FALSE(two.pivot6.claimed());
}

// (G3) Bug caught: the neutral touch reduction reading the RELEASE count as the press count (a
// touch would abort on the lift, or a selector's tap would replay), a coordinate swapped, or an
// error read not marked (its -1 counts would read as a release). press_count differs from
// release_count in every case, so a swap is visible.
TEST_CASE("common (G3): screenTouchFrom reads PROS's five fields into the sample and press count") {
    FakeProsTouch t;
    t.touch_status = kHeld;
    t.x = 123;
    t.y = 45;
    t.press_count = 5;
    t.release_count = 4;
    const ScreenTouch r = screenTouchFrom(t, static_cast<int>(kError));
    CHECK(r.pressCount == 5);
    CHECK(r.sample.releaseCount == 4);
    CHECK(r.sample.x == 123);
    CHECK(r.sample.y == 45);
    CHECK_FALSE(r.sample.error);
    FakeProsTouch e;
    e.touch_status = kError;
    e.press_count = -1;
    e.release_count = -1;
    CHECK(screenTouchFrom(e, static_cast<int>(kError)).sample.error);
    for (const auto st : {kReleased, kPressed, kHeld}) {
        FakeProsTouch g;
        g.touch_status = st;
        CHECK_FALSE(screenTouchFrom(g, static_cast<int>(kError)).sample.error);
    }
    // The tester's reduction is this one plus its finger-down fact (its own test (6b) pins that).
    const shulib::bench::TesterTouch tt =
        shulib::bench::testerTouchFrom(t, static_cast<int>(kError));
    CHECK(tt.sample.releaseCount == r.sample.releaseCount);
    CHECK(tt.pressCount == r.pressCount);
    CHECK(tt.sample.x == r.sample.x);
    CHECK(tt.sample.y == r.sample.y);
    CHECK(tt.fingerDown);
}

// (G4) Bug caught: the status bits moved with a value changed (the tester's RUN gate and the
// competition program's watch would read the wrong bit — PROS's own enum is checked against them
// in state_screens.cpp, which the host cannot compile, so the values are pinned here too).
TEST_CASE("common (G4): the status bits are PROS's (DISABLED 1, AUTONOMOUS 2, CONNECTED 4)") {
    CHECK(shulib::bench::kStatusDisabled == 0x01);
    CHECK(shulib::bench::kStatusAutonomous == 0x02);
    CHECK(shulib::bench::kStatusConnected == 0x04);
}

// (G5) Bug caught: the palette changed on the move (the tester's pages would change colour), a
// refusal literal that does not fit passing the painter's own check, or a line-change test that
// misses a tone change (a colour change would never be repainted).
TEST_CASE("common (G5): the painter's palette, its refusal budgets and lineDiffers") {
    using shulib::ui::Tone;
    CHECK(shulib::bench::kPanelBg == 0x101418u);
    CHECK(shulib::bench::toneRgb(Tone::Blue) == 0x1E5AA8u);
    CHECK(shulib::bench::toneRgb(Tone::Amber) == 0xE0A000u);
    CHECK(shulib::bench::toneRgb(Tone::Green) == 0x2E8B3Au);
    CHECK(shulib::bench::toneRgb(Tone::Red) == 0xC0392Bu);
    CHECK(shulib::bench::toneRgb(Tone::Grey) == 0x707070u);
    CHECK(shulib::bench::toneRgb(Tone::White) == 0xF0F0F0u);
    CHECK(shulib::bench::refusalBarFits("shulib AUTON TESTER"));
    CHECK_FALSE(shulib::bench::refusalBarFits("a bar word far too long for the header's budget"));
    CHECK_FALSE(shulib::bench::refusalBarFits("100% refused"));  // a '%' never reaches the screen
    CHECK(shulib::bench::refusalFooterFits(
        "The drive is stopped (0 V + Brake). Restart the program."));
    CHECK_FALSE(shulib::bench::refusalFooterFits(
        "one two three four five six seven eight nine ten eleven twelve thirteen fourteen "
        "fifteen sixteen seventeen eighteen nineteen"));  // three SMALL rows: too long
    shulib::competition::MenuLine a{};
    shulib::competition::MenuLine b{};
    CHECK_FALSE(shulib::bench::lineDiffers(a, b));
    b.tone = Tone::Red;
    CHECK(shulib::bench::lineDiffers(a, b));
    b.tone = a.tone;
    b.text[0] = 'x';
    CHECK(shulib::bench::lineDiffers(a, b));
}

// (G6) Bug caught: a PROS header in a shared header the host suite compiles as PROS-free (the CI
// grep covers include/shulib only, and the host shim provides some PROS headers — such an include
// could compile here and hide a robot-only call), or the painters' .cpp losing its PROS half (it
// would not paint at all).
TEST_CASE("common (G6): the shared headers are PROS-free; the painters' .cpp is the PROS half") {
    const std::regex prosInclude{R"(#\s*include\s*[<"]pros/)"};
    for (const char* file : {"src/common/guarded_run.hpp", "src/common/screen_touch.hpp",
                             "src/common/screen_paint.hpp", "src/common/state_screens.hpp"}) {
        CAPTURE(file);
        CHECK_FALSE(std::regex_search(readSource(file), prosInclude));
    }
    CHECK(std::regex_search(readSource("src/common/screen_paint.cpp"),
                            std::regex{R"(#include "pros/screen.h")"}));
}
