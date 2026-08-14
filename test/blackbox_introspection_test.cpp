// The E1 INTROSPECTION PATH, end to end: a corrector's numbers → the fusion policy's
// verdict → the Localizer's audit → the record → the blackbox file → the decoder.
//
// This is the honest half of the E1 DoD (tension T3). At E1 there are no real
// correctors — E2/E3/E4 build them — so the gate decisions here come from a
// DELIBERATELY SYNTHETIC corrector and policy that emit known residuals, a known
// Mahalanobis value, and each GateReason in turn. What that proves is that the PATH
// carries them faithfully; it does not prove any real gate is correct, and this file
// must not be read as evidence that it is. When E2 lands, the only new thing is real
// numbers.
//
// What each case targets:
//  * THE FULL PATH with a synthetic gate: every gating field arrives in the DECODED
//    file with the value the corrector/policy produced. A break anywhere — the audit
//    not leaving the policy, the Localizer dropping it, the record producer not
//    stamping it, the encoder skipping it — turns this red.
//  * THE REAL POLICY: ComplementaryFusion (which exists today) fills the reason,
//    residual and trust weight for itself, so the path is not only exercised by a
//    double.
//  * THE FAULT PATH, unmocked: a real LoopMonitor overrun raises a real fault, the
//    scheduler stamps it onto the record, and the blackbox dumps the preceding ticks.
//    Before E1 the record's `fault` field had no producer at all, so this path could
//    never have fired on a real run.
//  * D-7's post-run triage on the terminal, from the same data as the file.

#include "doctest.h"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "motion_test_rig.hpp"

#include "shulib/chassis/robot_context.hpp"
#include "shulib/diag/blackbox_format.hpp"
#include "shulib/diag/blackbox_reader.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/health_monitor.hpp"
#include "shulib/diag/sd_sink.hpp"
#include "shulib/diag/term_sink.hpp"
#include "shulib/hal/fake/fake_block_sink.hpp"
#include "shulib/hal/fake/fake_char_sink.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/correction.hpp"
#include "shulib/localization/fake/fake_corrector.hpp"
#include "shulib/localization/i_corrector.hpp"
#include "shulib/localization/i_fusion_policy.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/motion/run_reporter.hpp"

using shulib::diag::DebugRecord;
using shulib::diag::FaultCode;
using shulib::diag::GateReason;
using shulib::diag::SdSink;
using shulib::diag::SdSinkConfig;
using shulib::diag::SdSinkStorage;
using shulib::hal::fake::FakeBlockSink;
using shulib::hal::fake::FakeCharSink;
using shulib::localization::CorrectionProposal;
using shulib::localization::FusionResult;
using shulib::localization::GateAudit;
using shulib::localization::ICorrector;
using shulib::localization::IFusionPolicy;
namespace bb = shulib::diag::blackbox;
namespace units = shulib::units;

namespace {

/// What the synthetic gate should say this tick. One script, read by BOTH doubles, so
/// the numbers the corrector produces are literally the numbers the audit reports.
struct GateScript {
    double offsetX = 0.0;   ///< the fix sits this far from the predicted pose in x
    double offsetY = 0.0;   ///< …and this far in y — so the residual is known exactly
    double mahalanobis = 0.0;
    double covarianceTrace = 0.0;
    double headingResidual = 0.0;
    GateReason reason = GateReason::None;
    bool hasFix = true;     ///< false ⇒ the corrector has nothing this tick
};

/// A corrector that proposes a fix at a KNOWN offset from the predicted pose. That is
/// the whole trick: the residual the file should carry is a number this test chose.
class ScriptedCorrector final : public ICorrector {
public:
    explicit ScriptedCorrector(const GateScript& script) : script_{&script} {}

    [[nodiscard]] CorrectionProposal propose(const shulib::math::Pose2d& predicted,
                                             units::Time /*dt*/) override {
        CorrectionProposal p;
        p.valid = script_->hasFix;
        p.fieldPose = shulib::math::Pose2d{units::Length{predicted.x().value() + script_->offsetX},
                                           units::Length{predicted.y().value() + script_->offsetY},
                                           predicted.heading()};
        p.confidence = 0.5;
        p.positionStdDev = units::Length{1.0};
        return p;
    }
    [[nodiscard]] const char* name() const noexcept override { return "scripted"; }

private:
    const GateScript* script_;
};

/// A fusion policy that renders the scripted verdict and computes the residual from
/// what the corrector actually proposed. It deliberately leaves the position ALONE:
/// this test is about the audit path, and a moving estimate would make the expected
/// residuals depend on the fusion math instead of on the script.
class ScriptedPolicy final : public IFusionPolicy {
public:
    explicit ScriptedPolicy(const GateScript& script) : script_{&script} {}

    [[nodiscard]] FusionResult fuse(const shulib::math::Pose2d& predicted,
                                    std::span<const CorrectionProposal> valid,
                                    units::Time /*dt*/) override {
        FusionResult fr;
        fr.x = predicted.x();
        fr.y = predicted.y();
        GateAudit audit;
        if (!valid.empty()) {
            audit.residualX = units::Length{valid[0].fieldPose.x().value() - predicted.x().value()};
            audit.residualY = units::Length{valid[0].fieldPose.y().value() - predicted.y().value()};
        }
        audit.residualHeading = units::AngleDim{script_->headingResidual};
        audit.mahalanobis = script_->mahalanobis;
        audit.covarianceTrace = script_->covarianceTrace;
        audit.reason = script_->reason;
        fr.audit = audit;
        return fr;
    }

private:
    const GateScript* script_;
};

/// The scheduler stack with a caller-chosen fusion policy and corrector list, and the
/// blackbox as the record sink. Built by hand rather than through motion_rig's
/// SchedulerRig because that rig fixes the policy — and the policy is what this file
/// is testing. The plant keeps its own (null) sink, so the ONLY records in the file
/// are the ones the scheduler produced.
struct IntrospectionRig {
    shulib::kinematics::MatrixKinematics kin = motion_rig::hBotKinematics();
    shulib::sim::SimHarness h{kin, motion_rig::plantConfig()};
    shulib::localization::PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(),
                                              h.makeLateralTrackingWheel()};
    std::array<ICorrector*, 1> correctors{};
    shulib::localization::Localizer loc;

    FakeBlockSink device;
    std::vector<DebugRecord> ring;
    std::vector<std::byte> buffer;
    SdSink blackbox;

    shulib::hal::fake::FakeTelemetrySink faultSink;
    shulib::diag::FaultLatch latch{faultSink, h.clock()};
    shulib::diag::HealthMonitor health{latch};
    shulib::chassis::RobotContext ctx;
    shulib::motion::MotionDeps deps;
    motion_rig::PlantPacer pacer{h};
    shulib::motion::MotionScheduler sched;

    IntrospectionRig(IFusionPolicy& policy, ICorrector* corrector, std::size_t ringTicks,
                     const SdSinkConfig& sinkCfg, std::size_t bufferBytes = 65536,
                     const shulib::motion::MotionSchedulerConfig& schedCfg = {})
        : correctors{corrector},
          loc{h.clock(), h.imu(), odom, policy,
              corrector != nullptr ? std::span<ICorrector* const>{correctors}
                                   : std::span<ICorrector* const>{}},
          ring(ringTicks),
          buffer(bufferBytes),
          blackbox{device, h.clock(), SdSinkStorage{ring, buffer}, sinkCfg},
          ctx{shulib::chassis::RobotContextConfig{.clock = &h.context().clock(),
                                                  .driveMotors = h.context().driveMotors(),
                                                  .imu = &h.context().imu(),
                                                  .gps = &h.context().gps(),
                                                  .battery = &h.context().battery(),
                                                  .telemetry = &blackbox,
                                                  .tags = &h.context().tags(),
                                                  .vision = &h.context().vision()}},
          deps{.ctx = &ctx,
               .localizer = &loc,
               .kinematics = &kin,
               .faults = &latch,
               .health = &health},
          sched{deps, pacer, schedCfg} {}

    /// One idle scheduler tick plus one world step — the shape every C-phase loop has.
    void tick(units::Time dt = units::Time{0.01}) {
        (void)sched.tick();
        h.plant().step(dt);
    }
};

/// Every tick record in the device's bytes, in file order.
std::vector<DebugRecord> decodeTicks(const FakeBlockSink& device) {
    std::vector<DebugRecord> out;
    bb::BlackboxReader reader{device.view()};
    REQUIRE(reader.status() == bb::ReadStatus::Ok);
    bb::BlackboxReader::Frame frame;
    while (reader.next(frame)) {
        if (frame.type != bb::FrameType::Tick) {
            continue;
        }
        DebugRecord r;
        bool corrupt = false;
        REQUIRE(bb::decodeTick(frame.payload, r, corrupt));
        CHECK_FALSE(corrupt);
        out.push_back(r);
    }
    return out;
}

}  // namespace

// Would catch: a break ANYWHERE on the introspection path — the audit not leaving the
// fusion policy, the Localizer dropping it, the record producer not stamping it, or
// the encoder skipping the gating fields. Every value asserted here was chosen by the
// script, so a zero anywhere is a hole, not a coincidence.
TEST_CASE("introspection: a synthetic corrector's numbers reach the decoded file") {
    GateScript script;
    ScriptedPolicy policy{script};
    ScriptedCorrector corrector{script};
    IntrospectionRig rig{policy, &corrector, 16, SdSinkConfig{.streamTicks = true}};
    rig.blackbox.open({});

    struct Step {
        double offsetX;
        double offsetY;
        double mahalanobis;
        double trace;
        double headingResidual;
        GateReason reason;
    };
    // Every GateReason in the vocabulary, in turn — including the two (NoFix,
    // HighYawRate) that only a real corrector will ever produce, so the wire path for
    // them is proven before E2 needs it.
    const Step steps[] = {
        {0.0, 0.0, 0.0, 0.0, 0.0, GateReason::None},
        {1.5, -2.25, 0.75, 0.125, 0.03125, GateReason::Accepted},
        {9.0, 4.0, 6.5, 0.25, -0.0625, GateReason::RejectedInnovation},
        {-3.25, 0.5, 12.75, 0.5, 0.125, GateReason::RejectedMahalanobis},
        {0.0, 0.0, 0.0, 0.75, 0.0, GateReason::RejectedNoFix},
        {2.0, 2.0, 1.5, 1.25, 0.25, GateReason::RejectedHighYawRate},
    };

    for (const Step& s : steps) {
        script.offsetX = s.offsetX;
        script.offsetY = s.offsetY;
        script.mahalanobis = s.mahalanobis;
        script.covarianceTrace = s.trace;
        script.headingResidual = s.headingResidual;
        script.reason = s.reason;
        rig.tick();
    }
    CHECK(rig.blackbox.flush());

    const std::vector<DebugRecord> ticks = decodeTicks(rig.device);
    REQUIRE(ticks.size() == 6);
    for (std::size_t i = 0; i < ticks.size(); ++i) {
        CAPTURE(i);
        CHECK(ticks[i].gateReason == steps[i].reason);
        CHECK(ticks[i].gateResidualX.value() == doctest::Approx(steps[i].offsetX).epsilon(1e-9));
        CHECK(ticks[i].gateResidualY.value() == doctest::Approx(steps[i].offsetY).epsilon(1e-9));
        CHECK(ticks[i].gateResidualHeading.value() == steps[i].headingResidual);
        CHECK(ticks[i].gateMahalanobis == steps[i].mahalanobis);
        CHECK(ticks[i].covarianceTrace == steps[i].trace);
    }
    // …and every reason really did appear (a stamp that always wrote the same value
    // would satisfy the loop above if the script were constant).
    for (const Step& s : steps) {
        bool found = false;
        for (const DebugRecord& r : ticks) {
            found = found || r.gateReason == s.reason;
        }
        CHECK(found);
    }
}

// Would catch: the REAL fusion policy leaving the gating slots empty — the case that
// matters when no test double is present. ComplementaryFusion is the policy that ships
// today, and it must fill what it genuinely knows: the verdict, the innovation it
// acted on, and its scalar trust weight.
TEST_CASE("introspection: the real ComplementaryFusion fills its own audit") {
    shulib::localization::ComplementaryFusion policy{
        {.maxNudgeRate = units::Velocity{12.0}, .innovationGate = units::Length{6.0}}};
    shulib::localization::fake::FakeCorrector corrector{"gps"};
    IntrospectionRig rig{policy, &corrector, 8, SdSinkConfig{.streamTicks = true}};
    rig.blackbox.open({});

    // Tick 1: no fix at all → pure dead reckoning, and the audit says so.
    corrector.setProposal(CorrectionProposal{});
    rig.tick();

    // Tick 2: a fix 2 inches away, inside the 6-inch gate → ACCEPTED, trust = 0.4.
    const shulib::math::Pose2d before = rig.loc.pose();
    CorrectionProposal accepted;
    accepted.valid = true;
    accepted.fieldPose = shulib::math::Pose2d{units::Length{before.x().value() + 2.0},
                                              units::Length{before.y().value()},
                                              before.heading()};
    accepted.confidence = 0.4;
    accepted.positionStdDev = units::Length{1.0};
    corrector.setProposal(accepted);
    rig.tick();
    CHECK(rig.loc.lastCorrection().audit.reason == GateReason::Accepted);
    CHECK(rig.loc.lastCorrection().audit.covarianceTrace == doctest::Approx(0.4));

    // Tick 3: a wild fix 40 inches away → REJECTED by the innovation bound, and the
    // rejected innovation is what gets recorded.
    const shulib::math::Pose2d now = rig.loc.pose();
    CorrectionProposal wild = accepted;
    wild.fieldPose = shulib::math::Pose2d{units::Length{now.x().value() + 40.0},
                                          units::Length{now.y().value()}, now.heading()};
    corrector.setProposal(wild);
    rig.tick();
    CHECK(rig.loc.lastCorrection().audit.reason == GateReason::RejectedInnovation);
    CHECK(rig.blackbox.flush());

    const std::vector<DebugRecord> ticks = decodeTicks(rig.device);
    REQUIRE(ticks.size() == 3);
    CHECK(ticks[0].gateReason == GateReason::None);
    CHECK(ticks[0].covarianceTrace == 0.0);

    CHECK(ticks[1].gateReason == GateReason::Accepted);
    CHECK(ticks[1].gateResidualX.value() == doctest::Approx(2.0).epsilon(1e-6));
    CHECK(ticks[1].covarianceTrace == doctest::Approx(0.4));
    CHECK(ticks[1].correctionDx.value() > 0.0);  // the nudge actually moved the estimate
    CHECK(ticks[1].gateMahalanobis == 0.0);      // honest: no covariance at this tier (E4)

    CHECK(ticks[2].gateReason == GateReason::RejectedInnovation);
    CHECK(ticks[2].gateResidualX.value() == doctest::Approx(40.0).epsilon(1e-6));
    CHECK(ticks[2].correctionDx.value() == doctest::Approx(0.0).epsilon(1e-9));
}

// Would catch: DebugRecord::fault staying unpopulated — the hole E1 found. Nothing in
// the tree wrote that field before this chunk, which meant TermSink's ` flt=NAME`
// could never appear on a real run AND the flight recorder's trigger could never fire.
// This case uses a REAL fault from a REAL LoopMonitor: no fault is injected by hand.
TEST_CASE("introspection: a real fault reaches the record, and the flight recorder dumps") {
    shulib::localization::ComplementaryFusion policy;
    IntrospectionRig rig{policy, nullptr, 4, SdSinkConfig{}};  // ring-only: the D-6 posture
    rig.blackbox.open({});

    for (int i = 0; i < 6; ++i) {
        rig.tick();
    }
    CHECK(rig.device.empty());        // healthy ticks: not one byte
    CHECK_FALSE(rig.blackbox.dumped());

    // A tick that blows the loop budget: the scheduler's LoopMonitor raises
    // LOOP_OVERRUN inside the tick, so the record emitted by that same tick carries it.
    rig.h.plant().step(units::Time{0.5});
    rig.tick();

    CHECK(rig.latch.firstFault() == FaultCode::LoopOverrun);
    CHECK(rig.blackbox.dumped());
    CHECK_FALSE(rig.device.empty());
    CHECK(rig.blackbox.triage().fault == FaultCode::LoopOverrun);

    bb::BlackboxReader reader{rig.device.view()};
    REQUIRE(reader.status() == bb::ReadStatus::Ok);
    bb::BlackboxReader::Frame frame;
    REQUIRE(reader.next(frame));
    REQUIRE(frame.type == bb::FrameType::Triage);  // triage FIRST
    bb::TriageInfo info;
    DebugRecord faultTick;
    bool corrupt = false;
    REQUIRE(bb::decodeTriage(frame.payload, info, faultTick, corrupt));
    CHECK(info.fault == FaultCode::LoopOverrun);
    CHECK(info.precedingTicks == 4);
    CHECK(faultTick.fault == FaultCode::LoopOverrun);
    CHECK(faultTick.dt.value() > 0.1);  // the overrun is visible in the record itself

    int history = 0;
    while (reader.next(frame)) {
        CHECK(frame.type == bb::FrameType::Tick);
        DebugRecord r;
        REQUIRE(bb::decodeTick(frame.payload, r, corrupt));
        CHECK(r.fault == FaultCode::None);  // the ticks BEFORE the fault were healthy
        ++history;
    }
    CHECK(history == 4);
}

// Would catch: D-7's post-run triage never reaching the terminal, or reaching it on a
// clean run (noise where there is no story). Same data as the file, one screen.
TEST_CASE("introspection: RunReporter prints the triage block, and only after a fault") {
    shulib::localization::ComplementaryFusion policy;

    SUBCASE("a clean run says nothing about triage") {
        IntrospectionRig rig{policy, nullptr, 4, SdSinkConfig{}};
        FakeCharSink out;
        shulib::diag::TermSink term{rig.h.clock(), out};
        shulib::motion::RunReporter report{term, rig.sched, nullptr, &rig.blackbox};
        report.sessionStart({.buildHash = "abc1234", .routineId = "clean"});
        for (int i = 0; i < 4; ++i) {
            rig.tick();
        }
        report.finishRun();
        CHECK(out.text().find("[TRI]") == std::string::npos);
    }

    SUBCASE("a faulted run ends with why it broke") {
        IntrospectionRig rig{policy, nullptr, 4, SdSinkConfig{}};
        FakeCharSink out;
        shulib::diag::TermSink term{rig.h.clock(), out};
        shulib::motion::RunReporter report{term, rig.sched, nullptr, &rig.blackbox};
        report.sessionStart({.buildHash = "abc1234", .routineId = "faulty"});
        for (int i = 0; i < 4; ++i) {
            rig.tick();
        }
        rig.h.plant().step(units::Time{0.5});
        rig.tick();
        REQUIRE(rig.blackbox.dumped());
        report.finishRun();

        const std::string& text = out.text();
        CHECK(text.find("[ERROR][TRI] fault LOOP_OVERRUN @") != std::string::npos);
        CHECK(text.find("preceding 4 brownout no") != std::string::npos);
        CHECK(text.find("[ERROR][TRI] state pos(") != std::string::npos);
        CHECK(text.find("cmd#0▸0") != std::string::npos);
        // The summary comes first, the root cause last — the last thing on the screen
        // is why it broke.
        CHECK(text.find("RUN SUMMARY") < text.find("[TRI] fault"));
    }
}

// Would catch: the blackbox's own drop count never reaching the run summary — a file
// full of holes and a terminal that says everything is fine.
TEST_CASE("introspection: blackbox drops show up in the run summary, and only when real") {
    shulib::localization::ComplementaryFusion policy;

    SUBCASE("drops are reported") {
        // A starved buffer: streaming ticks overflow it, so the sink must drop, count,
        // and say so — never block the loop and never grow. (Sized at the legal
        // minimum: the header plus one triage frame.)
        IntrospectionRig rig{policy, nullptr, 4, SdSinkConfig{.streamTicks = true},
                             bb::kHeaderBytes + bb::kFrameHeaderBytes + bb::kTriagePayloadBytes};
        FakeCharSink out;
        shulib::diag::TermSink term{rig.h.clock(), out};
        shulib::motion::RunReporter report{term, rig.sched, nullptr, &rig.blackbox};
        report.sessionStart({.buildHash = "abc1234", .routineId = "starved"});
        for (int i = 0; i < 12; ++i) {
            rig.tick();
        }
        report.finishRun();
        CHECK(rig.blackbox.droppedFrames() > 0);
        CHECK(out.text().find("· blackbox dropped ") != std::string::npos);
    }

    SUBCASE("a run with no blackbox claims nothing about one") {
        IntrospectionRig rig{policy, nullptr, 4, SdSinkConfig{}};
        FakeCharSink out;
        shulib::diag::TermSink term{rig.h.clock(), out};
        shulib::motion::RunReporter report{term, rig.sched};  // no blackbox handed over
        report.sessionStart({.buildHash = "abc1234", .routineId = "plain"});
        rig.tick();
        report.finishRun();
        CHECK(out.text().find("blackbox") == std::string::npos);
    }
}
