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
// Contract: setLine() is synchronous on the caller's task, MUST NOT throw, and
// row is in [0, kRows) — a bad row is the CALLER's precondition to keep.

#include <string_view>

namespace shulib::hal {

class ILineDisplay {
public:
    /// The V5 controller text grid. PROVISIONAL (A4: HA-57) — see header.
    static constexpr int kRows = 3;
    static constexpr int kCols = 19;

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
