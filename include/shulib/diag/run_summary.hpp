#pragma once
//
// RunSummary — the end-of-run one-screen summary, as DATA (master plan §18.3; WS13,
// chunk C5). The §18.3 block is one of the two ideas worth salvaging from the legacy
// "logging extreme" code, re-expressed the §18 way: STRUCTURED FIELDS the sinks
// format, never an essay assembled in a motion loop.
//
// Why a struct on the sink seam (hal::ITelemetrySink::summarize) rather than text
// pushed through log(): the summary is a RECORD with one producer and many possible
// renderings — TermSink draws the boxed block, E1's SdSink will append it to the
// blackbox, H1's wire can carry it to VexBuilder. Formatting it before the seam
// would freeze the terminal rendering as the only consumer (§18.1 "one record, many
// sinks" applies to every record type, not just the per-tick one).
//
// VALUE TYPE, deliberately: the provenance strings live in bounded in-struct arrays
// (set via setBuildHash/setRoutineId), not string_views — a sink that RETAINS a
// summary (FakeTelemetrySink, the E1 blackbox) must not be handed dangling views
// into some caller's stack. Nothing here allocates.
//
// Content mapping, stated honestly (§18.3's sketch shows "scored 6 pin · 1 cup"):
// game-object scoring belongs to the strategy layer (the G-phase command registry —
// the library cannot know what a motion scored). The M2-honest equivalent is the
// MOTION LEDGER (started/settled/timeout/cancelled/aborted); when G2's registry
// exists, scoring lands as an ADDITIVE field, not a reshape.
//
// The "MISSING" contract (§18.5): an EMPTY buildHash means the build system did not
// provide one. Renderers must say MISSING, loudly — never invent a plausible value.
// A wrong hash is worse than an absent one.

#include <cstdint>
#include <cstring>
#include <string_view>

#include "shulib/diag/fault.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::diag {

struct RunSummary {
    // ── the motion ledger (the C5 stand-in for "scored/failed"; header note) ────────
    int motionsStarted = 0;
    int motionsSettled = 0;
    int motionsTimedOut = 0;
    int motionsCancelled = 0;  ///< user/pre-empt cancels (no causal fault)
    int motionsAborted = 0;    ///< fault-policy / task-boundary aborts

    // ── the heading story (§18.3 "heading: max … final …") ──────────────────────────
    /// False when no motion produced heading data (record stream off, or nothing
    /// ran) — renderers show "n/a", never a fabricated 0.0 (a 0.0° claim with no
    /// data behind it is exactly the lying-number failure C5's brief bans).
    bool hasHeadingData = false;
    units::AngleDim headingMax{};    ///< worst per-motion final |heading error| (radians)
    units::AngleDim headingFinal{};  ///< the LAST motion's final |heading error| (radians)

    // ── run health ──────────────────────────────────────────────────────────────────
    int gatingRejects = 0;       ///< GPS_GATE_REJECT episodes (FaultLatch tally)
    bool brownout = false;       ///< HealthMonitor::brownedOut() — latched, E1 semantics
    units::Time worstLoopDt{};   ///< LoopMonitor::worstDt()
    FaultCode firstFault = FaultCode::None;  ///< the ROOT CAUSE (FaultLatch first-fault)
    units::Time firstFaultTime{};            ///< when it latched (0 if none)

    // ── D-2: silent degradation is a bug — drops are REPORTED here too ──────────────
    std::uint32_t droppedRecords = 0;  ///< RateLimitedSink::droppedRecords()
    std::uint32_t droppedLines = 0;    ///< RateLimitedSink::droppedLines()

    // ── provenance (§18.5, mirrored from the session header) ────────────────────────
    units::Voltage batteryStart{};
    units::Voltage batteryEnd{};

    /// Empty ⇒ MISSING (rendered loudly; header note). 47 bytes admits a full
    /// 40-char git SHA plus a "-dirty" suffix.
    void setBuildHash(std::string_view hash) noexcept { copyBounded(buildHash_, sizeof buildHash_, hash); }
    void setRoutineId(std::string_view id) noexcept { copyBounded(routineId_, sizeof routineId_, id); }
    [[nodiscard]] std::string_view buildHash() const noexcept { return buildHash_; }
    [[nodiscard]] std::string_view routineId() const noexcept { return routineId_; }

private:
    static void copyBounded(char* dst, std::size_t cap, std::string_view src) noexcept {
        const std::size_t take = src.size() < cap - 1 ? src.size() : cap - 1;
        if (take > 0) {  // memcpy from a null/empty view is UB even for 0 bytes
            std::memcpy(dst, src.data(), take);
        }
        dst[take] = '\0';
    }

    char buildHash_[48] = "";
    char routineId_[32] = "";
};

}  // namespace shulib::diag
