#pragma once
//
// ILineDisplay — where short status LINES physically go (the V5 controller's LCD;
// a captured fake in tests). Diagnostics-plan D-4's SEAM (WS13, chunk C5).
//
// Why a seam and not a PROS call: the core is PROS-free and CI enforces it —
// exactly the ICharSink pattern (A1). C5 builds the CONTENT and this seam; the
// actual pros::Controller::set_text adapter is R1's glue, one file, zero core
// changes. Tests use hal::fake::FakeLineDisplay and assert exact rows.
//
// Why line-oriented (not ICharSink reuse): the controller LCD is a ROW device —
// three fixed rows, overwritten in place, no scrollback — so "append bytes" is
// the wrong verb; "set row i to text" is the device's real contract, and it is
// what lets content code express "rewrite only what changed" (V5 controller
// writes are slow and firmware-rate-limited; a per-tick full repaint is how a
// display starves — see ControllerFaultDisplay).
//
// Geometry: kRows = 3, kCols = 19 — the V5 controller's documented text grid.
// HARDWARE CLAIM, honest scope: unverified against real firmware until R1.
// PROVISIONAL (A4: HA-57). Implementations TRUNCATE text beyond kCols (never
// wrap — a wrapped status row would corrupt the row below it).
//
// NOT part of the frozen F4 ten (that freeze covers the 10 runtime robot-HAL
// interfaces); an ADDITIVE diagnostics-output seam, like ICharSink before it.
//
// Contract: setLine() is synchronous on the caller's task and MUST NOT THROW FOR ANY DEVICE
// CONDITION — an unplugged controller, a refused write, a firmware error code is DROPPED, never
// raised: a status row has no fallback channel and the telemetry log carries the same text.
//
// A row outside [0, kRows) is a DIFFERENT thing — a caller precondition breach, not a device
// condition — and the two shipped implementations answer it differently ON PURPOSE, which the
// old one-line wording made look like a contradiction. The host FAKE trips
// SHULIB_PRECONDITION, so a test that addresses a row that does not exist turns red where a
// person is watching. The PROS adapter returns silently, because on a robot mid-match the
// precondition handler's throw would unwind a motion over a cosmetic write. Same breach, two
// policies, each right for its target: the seam's rule is that a bad row is the caller's to
// keep, and neither implementation is obliged to make it survivable.

#include <string_view>

namespace shulib::hal {

/// Where short status LINES physically go — the V5 controller's LCD, or a capturing fake in
/// tests. A ROW device, not a byte stream: three fixed rows overwritten in place, no scrollback,
/// so the verb is "set row i to text" rather than "append bytes". That shape is what lets
/// content code rewrite only the rows that CHANGED, which matters because V5 controller writes
/// are slow and firmware-rate-limited — a per-tick full repaint is how the display starves.
/// Not one of the frozen F4 ten; an additive diagnostics-output seam, like ICharSink.
class ILineDisplay {
public:
    /// The V5 controller text grid. PROVISIONAL (A4: HA-57) — see header.
    static constexpr int kRows = 3;
    /// Columns per row. Text beyond it is TRUNCATED by implementations and never wrapped — a
    /// wrapped status row would overwrite the row below it. Also unverified against real
    /// firmware: the vendored PROS header implies 15 columns, community practice says 19, and
    /// neither is a measurement (A4: HA-57, HA-107).
    static constexpr int kCols = 19;

    /// Polymorphic-base boilerplate: the destructor is virtual so deleting through an
    /// `ILineDisplay*` is well-defined, and declaring it suppresses the implicit copy/move,
    /// which are re-defaulted here. Nothing in this tree deletes one that way, though — the
    /// seam is stateless (the rows live in the implementation) and a display is REFERENCED,
    /// never owned: ControllerFaultDisplay holds a non-owning `ILineDisplay&`, so the
    /// implementation must outlive every display bound to it.
    virtual ~ILineDisplay() = default;
    ILineDisplay() = default;
    ILineDisplay(const ILineDisplay&) = default;
    ILineDisplay(ILineDisplay&&) = default;
    ILineDisplay& operator=(const ILineDisplay&) = default;
    ILineDisplay& operator=(ILineDisplay&&) = default;

    /// Overwrite row `row` (0-based, caller keeps row < kRows) with `text`,
    /// truncated at kCols. MUST NOT throw.
    virtual void setLine(int row, std::string_view text) = 0;
};

}  // namespace shulib::hal
