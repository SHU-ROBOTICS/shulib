// Tests for diag/debug_record.hpp + the emit seam's cost mechanism. What each targets:
//  * SCHEMA COMPLETENESS + TYPES: every §18.2 field exists with its typed unit — this
//    test populates ALL of them, so deleting or retyping a field breaks the build here
//    first, before it silently reshapes the (future-frozen, F9) wire.
//  * WIRE-STABLE VOCABULARIES: GateReason numbers and the qualityClass ↔
//    Localizer::Quality mapping are pinned — a reorder of EITHER enum turns this red.
//  * THE NULL-SINK COST PROOF: via emitRecord(), a record is never even POPULATED when
//    nothing consumes it — the builder must not run for a NullSink. This is the test
//    that catches "always build the record and let NullSink drop it".

#include "doctest.h"

#include <array>
#include <cstdint>
#include <type_traits>

#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/null_sink.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::diag::DebugRecord;
using shulib::diag::FaultCode;
using shulib::diag::GateReason;
using shulib::hal::ITelemetrySink;
using shulib::hal::NullSink;
using shulib::hal::emitRecord;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::localization::Localizer;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::Pose2d;
namespace units = shulib::units;

// ── Typed-unit pins (F3 reaches the schema): retyping a dimensioned field to a bare
// double must fail HERE, visibly, not slip through to the F9 freeze. ──
static_assert(std::is_same_v<decltype(DebugRecord::t), units::Time>);
static_assert(std::is_same_v<decltype(DebugRecord::dt), units::Time>);
static_assert(std::is_same_v<decltype(DebugRecord::targetPose), shulib::math::Pose2d>);
static_assert(std::is_same_v<decltype(DebugRecord::measuredPose), shulib::math::Pose2d>);
static_assert(std::is_same_v<decltype(DebugRecord::errorX), units::Length>);
static_assert(std::is_same_v<decltype(DebugRecord::errorHeading), units::AngleDim>);
static_assert(std::is_same_v<decltype(DebugRecord::commanded), shulib::math::ChassisSpeeds>);
static_assert(std::is_same_v<decltype(DebugRecord::imuYaw), shulib::math::Angle>);
static_assert(std::is_same_v<decltype(DebugRecord::imuYawRate), units::AngularVelocity>);
static_assert(std::is_same_v<decltype(DebugRecord::gateResidualX), units::Length>);
static_assert(std::is_same_v<decltype(DebugRecord::correctionDx), units::Length>);
static_assert(std::is_same_v<decltype(DebugRecord::correctionDTheta), units::AngleDim>);
static_assert(std::is_same_v<decltype(DebugRecord::batteryVoltage), units::Voltage>);
static_assert(std::is_same_v<decltype(DebugRecord::batteryCurrent), units::Current>);
static_assert(std::is_same_v<decltype(DebugRecord::fault), shulib::diag::FaultCode>);
// Wire widths of the schema's raw integer fields (F9 serializes these verbatim).
static_assert(std::is_same_v<decltype(DebugRecord::activeCommandId), std::uint32_t>);
static_assert(std::is_same_v<decltype(DebugRecord::activeCommandState), std::uint8_t>);
static_assert(std::is_same_v<decltype(DebugRecord::qualityClass), std::uint8_t>);
static_assert(std::is_same_v<std::underlying_type_t<GateReason>, std::uint8_t>);
// C5 schema additions (the diagnostics-plan's one time-sensitive act): the D-2 drop
// counters and the D-3 attribution slots must be IN the record before the H1 freeze,
// at these exact widths/types.
static_assert(std::is_same_v<decltype(DebugRecord::droppedRecords), std::uint32_t>);
static_assert(std::is_same_v<decltype(DebugRecord::droppedLines), std::uint32_t>);
static_assert(std::is_same_v<decltype(DebugRecord::tickPhase),
                             std::array<units::Time, 8>>);
static_assert(std::is_same_v<std::underlying_type_t<shulib::diag::TickPhase>, std::uint8_t>);

TEST_CASE("DebugRecord: a default record is a valid 'quiet' tick (nothing built yet)") {
    const DebugRecord r{};
    CHECK(r.t.value() == 0.0);
    CHECK(r.dt.value() == 0.0);
    CHECK(r.wheelCount == 0);
    CHECK(r.activeCommandId == 0);
    CHECK(r.activeCommandState == 0);
    CHECK_FALSE(r.deadReckoning);
    CHECK(r.qualityClass == 0);
    CHECK(r.quality == 0.0);
    CHECK(r.covarianceTrace == 0.0);
    CHECK(r.gateMahalanobis == 0.0);
    CHECK(r.gateReason == GateReason::None);
    CHECK_FALSE(r.clampedThisTick);
    CHECK_FALSE(r.strafeFallbackActive);
    CHECK(r.fault == FaultCode::None);
    CHECK(r.batteryVoltage.value() == 0.0);
    // C5 additions: quiet defaults — nothing dropped, nothing attributed.
    CHECK(r.droppedRecords == 0);
    CHECK(r.droppedLines == 0);
    for (const auto& phase : r.tickPhase) {
        CHECK(phase.value() == 0.0);
    }
}

TEST_CASE("DebugRecord: the complete §18.2 field set is populatable and reads back") {
    // Populating EVERY field is the completeness pin: removing one breaks this build.
    DebugRecord r;
    r.t = units::Time{12.34};
    r.dt = units::Time{0.01};
    r.targetPose = Pose2d{units::Length{24.0}, units::Length{36.0}, Angle::degrees(90.0)};
    r.measuredPose = Pose2d{units::Length{23.6}, units::Length{35.8}, Angle::degrees(89.7)};
    r.errorX = units::Length{0.4};
    r.errorY = units::Length{0.2};
    r.errorHeading = units::AngleDim{0.005};
    r.commanded = ChassisSpeeds{units::Velocity{18.0}, units::Velocity{4.0},
                                units::AngularVelocity{0.1}};
    r.wheelCount = 4;
    r.wheelVoltage[0] = units::Voltage{11.2};
    r.wheelCurrent[0] = units::Current{1.8};
    r.imuYaw = Angle::degrees(89.7);
    r.imuYawRate = units::AngularVelocity{0.09};
    r.activeCommandId = 7;
    r.activeCommandState = 1;
    r.deadReckoning = true;
    r.qualityClass = 1;
    r.quality = 0.91;
    r.covarianceTrace = 0.02;
    r.gateResidualX = units::Length{0.8};
    r.gateResidualY = units::Length{0.5};
    r.gateResidualHeading = units::AngleDim{0.003};
    r.gateMahalanobis = 1.9;
    r.gateReason = GateReason::Accepted;
    r.correctionDx = units::Length{0.05};
    r.correctionDy = units::Length{-0.03};
    r.correctionDTheta = units::AngleDim{0.0};
    r.clampedThisTick = true;
    r.strafeFallbackActive = true;
    r.fault = FaultCode::LoopOverrun;
    r.batteryVoltage = units::Voltage{12.4};
    r.batteryCurrent = units::Current{3.1};
    r.droppedRecords = 3;
    r.droppedLines = 47;
    r.tickPhase[static_cast<std::size_t>(shulib::diag::TickPhase::Localization)] =
        units::Time{0.0042};
    r.tickPhase[static_cast<std::size_t>(shulib::diag::TickPhase::Motion)] =
        units::Time{0.0021};

    CHECK(r.t.value() == 12.34);
    CHECK(r.targetPose.heading().degrees() == doctest::Approx(90.0));
    CHECK(r.commanded.vx().value() == 18.0);
    CHECK(r.wheelVoltage[0].value() == 11.2);
    CHECK(r.gateReason == GateReason::Accepted);
    CHECK(r.fault == FaultCode::LoopOverrun);
    CHECK(r.droppedRecords == 3);
    CHECK(r.droppedLines == 47);
    CHECK(r.tickPhase[0].value() == doctest::Approx(0.0042));
    CHECK(r.tickPhase[1].value() == doctest::Approx(0.0021));
}

TEST_CASE("DebugRecord: per-wheel capacity is tied to the kinematics contract") {
    // If these ever diverge, a drivetrain could produce more wheels than the record can
    // carry — they must move together (and F5 already promises kMaxWheels is generous).
    CHECK(DebugRecord::kMaxWheels == shulib::kinematics::WheelSpeeds::kMaxWheels);
    CHECK(static_cast<int>(DebugRecord{}.wheelVoltage.size()) == DebugRecord::kMaxWheels);
    CHECK(static_cast<int>(DebugRecord{}.wheelCurrent.size()) == DebugRecord::kMaxWheels);
}

TEST_CASE("TickPhase: numeric values are wire-stable (F9) and the array reserves spare "
          "slots — a reorder or a capacity shrink turns this red") {
    // Bug this catches: someone renumbers the attribution vocabulary (silently
    // re-labeling every historical trace) or "tidies" the array down to the defined
    // phases, spending the spare slots the C5 schema reservation exists to keep.
    using shulib::diag::TickPhase;
    CHECK(static_cast<std::uint8_t>(TickPhase::Localization) == 0);
    CHECK(static_cast<std::uint8_t>(TickPhase::Motion) == 1);
    CHECK(static_cast<std::uint8_t>(TickPhase::Health) == 2);
    CHECK(static_cast<std::uint8_t>(TickPhase::Telemetry) == 3);
    CHECK(static_cast<std::uint8_t>(TickPhase::Scheduler) == 4);
    CHECK(static_cast<std::uint8_t>(TickPhase::User) == 5);
    CHECK(shulib::diag::kTickPhaseSlots == 8);  // 6 defined + 2 spare, reserved pre-F9
    CHECK(static_cast<int>(DebugRecord{}.tickPhase.size()) == shulib::diag::kTickPhaseSlots);
}

TEST_CASE("GateReason: numeric values are wire-stable (F9) — a reorder turns this red") {
    CHECK(static_cast<std::uint8_t>(GateReason::None) == 0);
    CHECK(static_cast<std::uint8_t>(GateReason::Accepted) == 1);
    CHECK(static_cast<std::uint8_t>(GateReason::RejectedInnovation) == 2);
    CHECK(static_cast<std::uint8_t>(GateReason::RejectedMahalanobis) == 3);
    CHECK(static_cast<std::uint8_t>(GateReason::RejectedNoFix) == 4);
    CHECK(static_cast<std::uint8_t>(GateReason::RejectedHighYawRate) == 5);
    // Appended at E2 (the first real corrector). APPEND-ONLY: these three took the next
    // free values and none of the five above moved, so a blackbox written before E2 still
    // decodes to the same meanings.
    CHECK(static_cast<std::uint8_t>(GateReason::RejectedNormalizedInnovation) == 6);
    CHECK(static_cast<std::uint8_t>(GateReason::RejectedStaleFix) == 7);
    CHECK(static_cast<std::uint8_t>(GateReason::RejectedSensorQuality) == 8);
    // Appended at E3 (the AprilTag corrector), on the same rule: the next free values, and
    // nothing above them moved. `RejectedSensorQuality` was REUSED for a below-floor tag
    // detection rather than duplicated, because it is the same statement about a sensor.
    CHECK(static_cast<std::uint8_t>(GateReason::RejectedNoTagMapEntry) == 9);
    CHECK(static_cast<std::uint8_t>(GateReason::RejectedTagRange) == 10);
    CHECK(static_cast<std::uint8_t>(GateReason::RejectedObservationAge) == 11);
    // Appended at E4 (the EKF tier), same rule again. This one is not a REJECTION: it is the
    // estimator declaring that its own confidence was wrong and re-initialising its covariance
    // (never its state — E4's T2). It lives in this vocabulary because the record has one
    // gating slot and that is where a reader looks to find out why the estimator did what it
    // did on a tick.
    CHECK(static_cast<std::uint8_t>(GateReason::CovarianceReinit) == 12);
}

TEST_CASE("DebugRecord.qualityClass: the documented numeric mirror of Localizer::Quality "
          "— a reorder of EITHER enum turns this red") {
    // diag/ stays a leaf (no localization include in the header), so the mapping is a
    // documented contract; this is the test that keeps the documentation true.
    CHECK(static_cast<int>(Localizer::Quality::Uninitialized) == 0);
    CHECK(static_cast<int>(Localizer::Quality::DeadReckon) == 1);
    CHECK(static_cast<int>(Localizer::Quality::Corrected) == 2);
    CHECK(static_cast<int>(Localizer::Quality::Degraded) == 3);
}

TEST_CASE("emitRecord + NullSink: the record is never POPULATED — the builder must not "
          "run (the §18.2 null-sink cost proof)") {
    NullSink null;
    ITelemetrySink& sink = null;
    int builds = 0;
    emitRecord(sink, [&] {
        ++builds;  // population cost stand-in
        return DebugRecord{};
    });
    CHECK(builds == 0);  // red if anyone "simplifies" emitRecord to always build
}

TEST_CASE("emitRecord + a consuming sink: built exactly once and delivered intact") {
    FakeTelemetrySink fake;
    ITelemetrySink& sink = fake;
    int builds = 0;
    emitRecord(sink, [&] {
        ++builds;
        DebugRecord r;
        r.t = units::Time{1.25};
        r.activeCommandId = 42;
        return r;
    });
    CHECK(builds == 1);
    REQUIRE(fake.recordCount() == 1);
    CHECK(fake.lastRecord().t.value() == 1.25);
    CHECK(fake.lastRecord().activeCommandId == 42);
}

TEST_CASE("FakeTelemetrySink: record history is ordered, bounds-checked, and cleared "
          "with the message history") {
    FakeTelemetrySink fake;
    CHECK_THROWS_AS((void)fake.lastRecord(), PreconditionError);   // nothing emitted yet
    CHECK_THROWS_AS((void)fake.recordAt(0), PreconditionError);

    DebugRecord a;
    a.activeCommandId = 1;
    DebugRecord b;
    b.activeCommandId = 2;
    fake.emit(a);
    fake.emit(b);
    REQUIRE(fake.recordCount() == 2);
    CHECK(fake.recordAt(0).activeCommandId == 1);
    CHECK(fake.recordAt(1).activeCommandId == 2);
    CHECK_THROWS_AS((void)fake.recordAt(-1), PreconditionError);
    CHECK_THROWS_AS((void)fake.recordAt(2), PreconditionError);

    fake.clear();
    CHECK(fake.recordCount() == 0);
}
