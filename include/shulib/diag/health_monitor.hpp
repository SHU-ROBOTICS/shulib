#pragma once
//
// HealthMonitor — sensor/power pathology → FaultCode, edge-triggered (WS13, chunk A3).
//
// ── Why this exists ─────────────────────────────────────────────────────────────────
// A1 built the fault vocabulary (FaultCode) and the latch (FaultLatch); A3 is the
// chunk that must prove "every sensor pathology raises a fault code with a safe
// fallback" is REAL. The estimators deliberately do not raise faults themselves —
// PilonsOdometry exposes lastDeltaImplausible(), the Localizer exposes quality
// state, the HAL exposes isReady()/voltage()/temperature() — because raising is
// POLICY, and fault.hpp assigns that policy to the loop layer ("OdoStuck …
// raised by the C/E layers"). This class is that policy, factored so the A3 test
// loops and C1's real tick loop share ONE implementation instead of each
// hand-rolling edge detection.
//
// ── Why it takes raw OBSERVABLES, not component references ──────────────────────────
// tick(Observations) receives plain bools/values the caller already reads each tick
// (imu.isReady(), odom.lastDeltaImplausible(), localizer.lastCorrection().gated,
// battery.voltage(), max motor temperature). Rejected: holding IImu&/Localizer&
// references — diag/ is a dependency LEAF (debug_record.hpp: localization may
// include diag, NEVER the reverse), so the monitor cannot name estimator types; and
// raw values also keep it trivially testable and reusable for any future source of
// the same observables. LoopOverrun is deliberately NOT here — LoopMonitor (A1)
// already owns timing and its dt bookkeeping; two monitors, two concerns.
//
// ── Edge-triggered, per EPISODE (the anti-spam contract) ────────────────────────────
// A pathology that persists for 500 ticks is ONE episode, not 500 faults: each
// condition raises on its false→true transition and re-arms when the condition
// clears. FaultLatch already counts cascades; flooding it with one code per tick
// would bury the first-fault story that latch exists to tell (a 2am log with 500
// IMU_LOST lines is the legacy anti-pattern §18 bans). Brownout adds HYSTERESIS
// (recoverVolts above the trip point) so a pack sagging around the threshold under
// a pulsing load cannot chatter episodes.
//
// ── Semantics that are contracts, each pinned by test ───────────────────────────────
//  * IMU_LOST fires only on a loss AFTER the IMU has been seen ready — the boot
//    calibration window (isReady() false from t=0) is NORMAL, not a fault. Waiting
//    out calibration is the loop's job (C1); reporting it as a loss would make every
//    clean boot start with a spurious fault.
//  * BROWNOUT is additionally LATCHED here (brownedOut() stays true for the run) —
//    the E1 "latched brownout marker" semantics: a pack that collapsed and bounced
//    back is still a collapsed pack, and the end-of-run summary must say so.
//  * GPS no-fix is NOT a fault, by design: Driving Skills has no strip at all, so
//    "no fix" is a normal operating state the Localizer already reports as quality
//    decay. What IS raised is GPS_GATE_REJECT — a fix that arrived and was rejected
//    by the fusion gate (the fixGated observable), i.e. a sensor actively lying.
//  * MOTOR_OVER_TEMP threshold defaults to 55 °C — the V5's documented first
//    throttle step. PROVISIONAL (A4 register HA-44): the exact droop onset on our
//    motors is unmeasured until R4.
//  * brownoutVolts default 10.5 V — PROVISIONAL (A4 register HA-42): the true V5
//    cutoff behaviour under load is unmeasured until R3/R4.
//    (Register: docs/planning/hardware-assumptions.md.)
//
// Single-task by contract, like the rest of diag/ (see fault.hpp).

#include <cmath>
#include <cstdio>

#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::diag {

struct HealthMonitorConfig {
    /// Battery voltage at/below which a BROWNOUT episode trips. PROVISIONAL (A4: HA-42).
    units::Voltage brownoutVolts{10.5};
    /// Voltage the pack must RECOVER above before a new brownout episode can trip
    /// (hysteresis; must be >= brownoutVolts). PROVISIONAL (A4: HA-42).
    units::Voltage brownoutRecoverVolts{10.8};
    /// Motor temperature (°C) at/above which MOTOR_OVER_TEMP trips. PROVISIONAL (A4: HA-44).
    double maxMotorTempC = 55.0;
};

class HealthMonitor {
public:
    /// The per-tick observables. The caller reads these from the components it
    /// already owns; every default is the HEALTHY value, so a caller without some
    /// source (e.g. no motor temps wired yet) simply leaves the field alone.
    struct Observations {
        bool imuReady = true;          ///< IImu::isReady()
        bool odomImplausible = false;  ///< PilonsOdometry::lastDeltaImplausible()
        bool odomStalled = false;      ///< caller-computed wheels-spin-but-no-motion
                                       ///< cross-check (see note below)
        bool fixGated = false;         ///< Localizer::lastCorrection().gated
        units::Voltage batteryVolts{12.6};  ///< IBattery::voltage()
        double maxMotorTempC = 0.0;    ///< max IMotor::temperature() over the drive
    };
    // On odomStalled: a frozen/disconnected encoder is INVISIBLE to the M2 estimator
    // (zero travel is a perfectly plausible reading — found and recorded at A3), so
    // the detection must come from a cross-check the LOOP owns: commanded/spinning
    // wheels + no reported motion over a window. The monitor takes the verdict as an
    // observable; building the windowed cross-check into the estimator is E-phase
    // work (fault.hpp: OdoStuck is "raised by the C/E layers").

    HealthMonitor(FaultLatch& faults, const HealthMonitorConfig& config = {})
        : faults_{faults}, cfg_{config} {
        SHULIB_PRECONDITION(std::isfinite(cfg_.brownoutVolts.value())
                                && cfg_.brownoutVolts.value() > 0.0,
                            "HealthMonitor: brownoutVolts must be finite and > 0");
        SHULIB_PRECONDITION(cfg_.brownoutRecoverVolts.value() >= cfg_.brownoutVolts.value(),
                            "HealthMonitor: brownoutRecoverVolts must be >= brownoutVolts");
        SHULIB_PRECONDITION(std::isfinite(cfg_.maxMotorTempC) && cfg_.maxMotorTempC > 0.0,
                            "HealthMonitor: maxMotorTempC must be finite and > 0");
    }

    /// Evaluate one tick's observables; raise one fault per NEW episode (header).
    void tick(const Observations& o) {
        // IMU: ready-at-least-once → lost. (Boot window: never-ready is not a loss.)
        if (o.imuReady) {
            imuSeenReady_ = true;
            imuLostActive_ = false;  // recovered → re-arm
        } else if (imuSeenReady_ && !imuLostActive_) {
            imuLostActive_ = true;
            faults_.raise(FaultCode::ImuLost, "IMU", "isReady dropped mid-run");
        }

        // Odometry: implausible tick OR the caller's stall cross-check.
        const bool odoBad = o.odomImplausible || o.odomStalled;
        if (odoBad && !odoActive_) {
            odoActive_ = true;
            faults_.raise(FaultCode::OdoStuck, "LOC",
                          o.odomStalled ? "wheels spin but odometry reports no motion"
                                        : "implausible odometry tick");
        } else if (!odoBad) {
            odoActive_ = false;
        }

        // Fusion gate: a fix arrived and was rejected (a lying absolute source).
        if (o.fixGated && !gateActive_) {
            gateActive_ = true;
            faults_.raise(FaultCode::GpsGateReject, "LOC", "fix rejected by fusion gate");
        } else if (!o.fixGated) {
            gateActive_ = false;
        }

        // Battery: trip at/below the threshold; re-arm only above the recover level.
        const double v = o.batteryVolts.value();
        if (v <= cfg_.brownoutVolts.value()) {
            if (!brownoutActive_) {
                brownoutActive_ = true;
                brownedOut_ = true;  // the run-scoped latched marker (E1 semantics)
                char buf[48];
                std::snprintf(buf, sizeof buf, "battery=%.2fV", v);
                faults_.raise(FaultCode::Brownout, "PWR", buf);
            }
        } else if (v >= cfg_.brownoutRecoverVolts.value()) {
            brownoutActive_ = false;  // hysteresis: only a real recovery re-arms
        }

        // Motor thermal: any motor at/over the throttle threshold.
        if (o.maxMotorTempC >= cfg_.maxMotorTempC) {
            if (!overTempActive_) {
                overTempActive_ = true;
                char buf[48];
                std::snprintf(buf, sizeof buf, "temp=%.1fC", o.maxMotorTempC);
                faults_.raise(FaultCode::MotorOverTemp, "PWR", buf);
            }
        } else {
            overTempActive_ = false;
        }
    }

    /// True once ANY brownout episode has occurred this run (latched; header note).
    [[nodiscard]] bool brownedOut() const noexcept { return brownedOut_; }
    /// True while the IMU is in a lost episode (seen ready, currently not).
    [[nodiscard]] bool imuLost() const noexcept { return imuLostActive_; }

    /// New-run boundary (mirrors FaultLatch::clear()): forget episodes AND the
    /// brownout marker; the boot-window rule starts over (imuSeenReady resets).
    void reset() noexcept {
        imuSeenReady_ = false;
        imuLostActive_ = false;
        odoActive_ = false;
        gateActive_ = false;
        brownoutActive_ = false;
        brownedOut_ = false;
        overTempActive_ = false;
    }

private:
    FaultLatch& faults_;
    HealthMonitorConfig cfg_;
    bool imuSeenReady_ = false;
    bool imuLostActive_ = false;
    bool odoActive_ = false;
    bool gateActive_ = false;
    bool brownoutActive_ = false;
    bool brownedOut_ = false;
    bool overTempActive_ = false;
};

}  // namespace shulib::diag
