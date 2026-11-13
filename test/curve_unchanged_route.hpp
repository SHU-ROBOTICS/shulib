#pragma once
//
// curve_unchanged_route.hpp — CURVE1-A's CV16 fixture: "NOTHING ELSE CHANGES". One six-leg auton of
// TODAY's verbs (moveTo, driveTo, reverseTo, turnTo, face, driveTo), driven through the real Chassis
// and Routine on the host test plant, on a tank with one side 9 % weak (so TANK1's steering and its
// yaw observer are live) and on an X-drive; everything it produces — every DebugRecord through the
// blackbox encoder's bytes, every log line, the true pose after every pace — folded into one 64-bit
// FNV-1a hash. The hash was MEASURED with this exact header compiled against the run's MERGE BASE
// (1ed0e26, before CURVE1 existed — CURVE1-PROGRESS.md), and motion_curve_test.cpp pins it: the curve
// verbs may add, but may not move one byte of what the other verbs do. Self-contained on purpose
// (no CURVE1 header is included), so the same file compiles against the base.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string_view>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/diag/blackbox_format.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/units/quantity.hpp"

namespace curve_cv16 {

/// FNV-1a, 64-bit.
struct Fnv {
    std::uint64_t h = 1469598103934665603ULL;
    void bytes(const void* p, std::size_t n) {
        const auto* b = static_cast<const unsigned char*>(p);
        for (std::size_t i = 0; i < n; ++i) {
            h ^= b[i];
            h *= 1099511628211ULL;
        }
    }
    void text(std::string_view s) { bytes(s.data(), s.size()); }
    void num(double v) { bytes(&v, sizeof v); }
};

/// One side's volts scaled (the weak side TANK1's tests use: wheel 1 = the right side).
struct WeakSide final : shulib::sim::DegradationModel {
    int wheel = 1;
    double scale = 1.0;
    shulib::units::Voltage effectiveVoltage(int w, shulib::units::Voltage c, shulib::units::Time,
                                            shulib::sim::Rng&) override {
        return w == wheel ? shulib::units::Voltage{c.value() * scale} : c;
    }
};

/// A pacer that steps the plant and folds the TRUE pose into the hash after every pace.
struct HashPacer final : shulib::motion::ITickPacer {
    shulib::sim::SimHarness* h;
    Fnv* fnv;
    int paces = 0;
    HashPacer(shulib::sim::SimHarness& harness, Fnv& f) : h{&harness}, fnv{&f} {}
    void pace() override {
        if (++paces > 100000) {
            throw std::runtime_error("HashPacer: pace cap exceeded");
        }
        h->plant().step(shulib::units::Time{0.01});
        const shulib::math::Pose2d t = h->truePose();
        fnv->num(t.x().value());
        fnv->num(t.y().value());
        fnv->num(t.heading().radians());
    }
};

/// The six legs, on `kin`, from the origin; returns the hash of records + log + truth.
inline std::uint64_t routeHash(const shulib::kinematics::IKinematics& kin, double weakScale) {
    using shulib::units::Length;
    using shulib::math::Angle;
    using shulib::math::Pose2d;
    Fnv fnv;
    shulib::hal::fake::FakeTelemetrySink sink;
    WeakSide world;
    world.scale = weakScale;
    motion_rig::MotionRig rig{kin, motion_rig::plantConfig(), &sink, &world};
    HashPacer pacer{rig.h, fnv};
    shulib::chassis::Chassis chassis{rig.deps, pacer, motion_rig::chassisConfig()};
    shulib::chassis::Routine r{chassis, "cv16"};
    r.moveTo(Pose2d{Length{36.0}, Length{0.0}, Angle{}})
        .driveTo(Length{36.0}, Length{30.0})
        .reverseTo(Length{36.0}, Length{6.0})
        .turnTo(Angle::degrees(180.0))
        .face(Length{0.0}, Length{0.0})
        .driveTo(Length{0.0}, Length{0.0});
    fnv.num(static_cast<double>(r.result().completed));
    fnv.num(static_cast<double>(r.result().stoppedAt));
    namespace bb = shulib::diag::blackbox;
    std::byte buf[bb::kTickPayloadBytes];
    for (int i = 0; i < sink.recordCount(); ++i) {
        const std::size_t n = bb::encodeTick(std::span<std::byte>{buf}, sink.recordAt(i));
        fnv.bytes(buf, n);
    }
    for (int i = 0; i < sink.size(); ++i) {
        fnv.num(static_cast<double>(static_cast<int>(sink.at(i).level)));
        fnv.text(sink.at(i).subsystem);
        fnv.text(sink.at(i).message);
    }
    return fnv.h;
}

}  // namespace curve_cv16
