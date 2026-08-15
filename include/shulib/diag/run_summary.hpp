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

/// The end-of-run summary as DATA, never as an assembled essay: structured fields that one
/// producer fills and any number of renderers format — the boxed terminal block, an
/// appended blackbox frame, a wire message. A VALUE TYPE that owns its provenance strings
/// in bounded in-struct arrays and allocates nothing, so a sink may RETAIN a copy without
/// holding a dangling view into some caller's stack. Assembled once per run and delivered
/// through hal::ITelemetrySink::summarize().
struct RunSummary {
    // ── the motion ledger (the C5 stand-in for "scored/failed"; header note) ────────
    /// Motions the scheduler handed a start(). It EXCEEDS the four outcome counts below
    /// whenever a motion was still running when the summary was taken — they partition
    /// the FINISHED motions only, so started minus their sum is what was still in flight.
    int motionsStarted = 0;
    int motionsSettled = 0;   ///< Exited inside its tolerances — the only outcome that means success
    int motionsTimedOut = 0;  ///< Exited on the watchdog; each one also raised MOTION_TIMEOUT
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
    /// Frames the E1 blackbox (diag::SdSink) dropped because its RAM byte budget was
    /// exhausted, or because a device write failed. A SEPARATE counter from the two
    /// above on purpose: those are rate-limiter drops on the terminal channel, and
    /// merging two different failures into one number is how a diagnostic starts
    /// lying. 0 also means "no blackbox was attached", which is why renderers show
    /// this one only when it is non-zero (TermSink's summarize note). — E1
    std::uint32_t blackboxDropped = 0;

    // ── provenance (§18.5, mirrored from the session header) ────────────────────────
    /// Pack volts READ at session start, never caller-typed: a typed 12.6 that was really
    /// 11.9 is exactly the lying number this record exists to avoid.
    units::Voltage batteryStart{};
    /// Pack volts read when the summary was assembled; with batteryStart, the run's sag.
    /// Both are 0 V on a summary nobody filled in — there is no "unset" sentinel here.
    units::Voltage batteryEnd{};

    /// Empty ⇒ MISSING (rendered loudly; header note). 47 bytes admits a full
    /// 40-char git SHA plus a "-dirty" suffix.
    void setBuildHash(std::string_view hash) noexcept { copyBounded(buildHash_, sizeof buildHash_, hash); }
    /// Copy the auton routine's name (e.g. "redLeftTall") in, TRUNCATED at 31 characters.
    /// Empty is ordinary here — only buildHash treats empty as the loud MISSING case.
    void setRoutineId(std::string_view id) noexcept { copyBounded(routineId_, sizeof routineId_, id); }

    /// The stored hash; EMPTY means the build system provided none, which renderers must
    /// print as MISSING rather than anything plausible-looking. LIFETIME: the view points
    /// into THIS object — it dies with the summary, the next setBuildHash() invalidates
    /// it, and a copied summary hands back views into the COPY. That is the whole reason
    /// this is a value type rather than a struct of string_views.
    [[nodiscard]] std::string_view buildHash() const noexcept { return buildHash_; }

    /// The stored routine name; empty if never set. Same lifetime rule as buildHash():
    /// the view is into this object, never into what the caller passed setRoutineId().
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
