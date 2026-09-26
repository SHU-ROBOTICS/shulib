#pragma once
//
// IOdometry — the dead-reckoning seam the Localizer predicts from (chunk R3b Part 2,
// 2026-09-14).
//
// ── Why a seam, and why these five members ──────────────────────────────────────────
// Until this chunk the chain motion → IPoseSource → Localizer → PilonsOdometry → 2 ×
// IRotation was rigid: Localizer took a CONCRETE PilonsOdometry&, and PilonsOdometry
// precondition-requires two tracking wheels. A robot without tracking wheels — robot two,
// and most VEX robots — therefore could not have a Localizer at all, which is the gap the
// roadmap's "What R3b must BUILD" item 2 measured ("LemLib does drive-encoder odometry and
// shulib cannot"). The Localizer never needed PilonsOdometry specifically: it called
// exactly four things on it — update(), pose(), setPose() and lastDeltaImplausible() —
// and this interface was those four and nothing else, so the seam is the Localizer's real
// dependency made explicit, not a new abstraction. PilonsOdometry implements it with NO
// behavioural change (the whole existing suite is the bit-identity pin), and
// DriveEncoderOdometry is the second implementation.
//
// THE FIFTH MEMBER, lastTickAudit() (chunk DIAG1, 2026-09-26). The verdict alone turned out
// not to be enough to act on: on 2026-09-25 robot two raised `implausible odometry tick`
// twice, and the line could not say which check tripped or on what values, so the one
// fault that ENDS a motion by default could not be explained. The account of WHY belongs to
// the gate that computed the verdict — each implementation's own sub-expressions — so it
// is part of this contract rather than something a caller could reconstruct: the flags are
// those sub-expressions copied as they stand, and every implementation sets its verdict
// FROM the audit (diag/odom_tick_audit.hpp), so the two cannot disagree. This was additive
// only — no existing member changed, and IOdometry is not in the Freeze Register — but it
// is a new PURE virtual, so any out-of-tree implementation must add it.
//
// The contract every implementation owes (PilonsOdometry's, generalized):
//   * update() integrates one tick and advances the pose; the CALLER owns the cadence.
//   * pose() is a pure read — the same value until the next update() or setPose().
//   * HEADING IS IMU-OWNED: pose().heading() is the IMU's reading, never integrated from
//     wheels (pilons_odometry.hpp decision #3; the < 1° heading spec depends on it).
//     setPose() therefore teleports POSITION only.
//   * lastDeltaImplausible() is the TRUST GATE: true when the last tick was not to be
//     trusted (an oversized heading change, an oversized wheel delta, or a non-finite
//     integration). An implausible-but-finite delta is REPORTED, NEVER WITHHELD
//     (plausibility_guard.hpp principle 4); only a non-finite tick freezes position.
//     HealthMonitor turns the flag into ODO_STUCK.
//   * lastTickAudit() is the gate's ACCOUNT of that same tick — which checks tripped and
//     the values they saw — refreshed by EVERY update() (not sticky), and its tripped() is
//     lastDeltaImplausible() bit for bit. HealthMonitor prints it on the fault line.
// Raises no faults (raising is the loop layer's policy). Single-task by contract.

#include "shulib/diag/odom_tick_audit.hpp"
#include "shulib/math/pose2d.hpp"

namespace shulib::localization {

/// The dead-reckoning seam: the five members Localizer predicts from, and nothing else.
/// Implementations integrate BODY travel from their own sensors into a field-frame Pose2d
/// whose heading is the IMU's (never wheel-derived), advance only when update() is called,
/// teleport position only on setPose(), and report — never withhold — an untrustworthy
/// tick through lastDeltaImplausible(), with the gate's account of why through
/// lastTickAudit(). PilonsOdometry (two tracking wheels) and DriveEncoderOdometry (the drive
/// motors' own encoders) are the two implementations.
class IOdometry {
public:
    /// Virtual so a polymorphic owner could destroy through the base; nothing in shulib
    /// does — the Localizer holds an IOdometry& and the caller owns the concrete object,
    /// which must outlive it. Copy/move re-defaulted so the base imposes no policy (the
    /// IMotor precedent, hal/motor.hpp).
    virtual ~IOdometry() = default;
    IOdometry() = default;
    IOdometry(const IOdometry&) = default;
    IOdometry(IOdometry&&) = default;
    IOdometry& operator=(const IOdometry&) = default;
    IOdometry& operator=(IOdometry&&) = default;

    /// One integration tick: read the sensors, integrate, accumulate. The caller calls it
    /// once per control tick (the Localizer does, inside its own update()).
    virtual void update() = 0;

    /// The accumulated field-frame estimate — canonical inches, heading the IMU's. A pure
    /// read: unchanged between ticks.
    [[nodiscard]] virtual math::Pose2d pose() const noexcept = 0;

    /// Teleport the POSITION (x, y); heading stays IMU-owned. Must re-baseline whatever the
    /// implementation differences so the teleport itself injects no phantom motion.
    virtual void setPose(const math::Pose2d& p) = 0;

    /// True iff the last update() was untrustworthy (the trust gate — header contract).
    [[nodiscard]] virtual bool lastDeltaImplausible() const noexcept = 0;

    /// The trust gate's account of the LAST update(): every tick, not sticky. Its flags are
    /// the gate's own sub-expressions and its tripped() equals lastDeltaImplausible() for the
    /// same tick; before the first update() it is the default (nothing tripped). setPose()
    /// leaves it alone, as it leaves the verdict. Added at DIAG1 (header).
    [[nodiscard]] virtual diag::OdomTickAudit lastTickAudit() const noexcept = 0;
};

}  // namespace shulib::localization
