#pragma once
//
// route_preview_hash.hpp — FNV-1a fingerprints of one tester-rig run, for RPV1-B1's RP35 (chunk
// RPV1-A, 2026-11-12): "a TesterStack built with the new parameter left at its default runs exactly as
// before". The fingerprints were captured over the BASE tree (d33d25d, before the rig took its
// mechanism-list parameter) by a probe that includes this very header, and are pinned as constants in
// test/route_preview_test.cpp; the case asserts the same numbers on today's rig.
//
// Two streams, each independent of the route preview's own recorder:
//   * TRUTH — a tap pacer put directly above the plant pacer records SimHarness::sample() after every
//     pace (the bytes of all seven doubles);
//   * THE SINK — every log line (level, subsystem, message) and, from every DebugRecord, the fields a
//     motion writes (t, both poses, the errors, the commanded twist, the command id and state).

#include <cstdint>
#include <cstring>
#include <string_view>

#include "routine_done_line.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/sim/scenario.hpp"

namespace rp_hash {

/// FNV-1a, 64 bits.
struct Fnv {
    std::uint64_t h = 1469598103934665603ULL;
    void bytes(const void* p, std::size_t n) {
        const auto* b = static_cast<const unsigned char*>(p);
        for (std::size_t i = 0; i < n; ++i) {
            h ^= b[i];
            h *= 1099511628211ULL;
        }
    }
    void f64(double v) { bytes(&v, sizeof v); }
    void u32(std::uint32_t v) { bytes(&v, sizeof v); }
    void text(std::string_view s) {
        u32(static_cast<std::uint32_t>(s.size()));
        bytes(s.data(), s.size());
    }
};

/// The tap (header): forwards, THEN hashes the truth sample the pace produced.
struct TruthTap final : shulib::motion::ITickPacer {
    shulib::motion::ITickPacer* inner = nullptr;
    const shulib::sim::SimHarness* h = nullptr;
    Fnv fnv;
    long paces = 0;
    void pace() override {
        inner->pace();
        ++paces;
        const shulib::sim::TruthSample s = h->sample();
        fnv.bytes(&s, sizeof s);
    }
};

/// The sink's fingerprint (header).
[[nodiscard]] inline std::uint64_t sinkHash(const shulib::hal::fake::FakeTelemetrySink& sink) {
    Fnv f;
    for (int i = 0; i < sink.size(); ++i) {
        const auto& e = sink.at(i);
        if (routine_d4::isStepDoneLine(e.level, e.subsystem, e.message)) {
            continue;  // RMP1-K D4's line: after this golden's base (routine_done_line.hpp)
        }
        f.u32(static_cast<std::uint32_t>(e.level));
        f.text(e.subsystem);
        f.text(e.message);
    }
    for (int i = 0; i < sink.recordCount(); ++i) {
        const auto& r = sink.recordAt(i);
        f.f64(r.t.value());
        f.f64(r.targetPose.x().value());
        f.f64(r.targetPose.y().value());
        f.f64(r.targetPose.heading().radians());
        f.f64(r.measuredPose.x().value());
        f.f64(r.measuredPose.y().value());
        f.f64(r.measuredPose.heading().radians());
        f.f64(r.errorX.value());
        f.f64(r.errorY.value());
        f.f64(r.errorHeading.value());
        f.f64(r.commanded.vx().value());
        f.f64(r.commanded.vy().value());
        f.f64(r.commanded.omega().value());
        f.u32(r.activeCommandId);
        f.u32(r.activeCommandState);
    }
    return f.h;
}

}  // namespace rp_hash
