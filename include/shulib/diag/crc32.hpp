#pragma once
//
// CRC-32/IEEE — the one checksum in shulib, in the one place both of its callers can reach
// (moved here from calibration/calibration_record.hpp at chunk H1 Part A1, 2026-09-29).
//
// ── Who calls it ────────────────────────────────────────────────────────────────────
//   * the calibration record (calibration/calibration_record.hpp, R3d Part 2): 4 bytes over a
//     180-byte record, read once at boot — calibration::crc32Ieee forwards here, so that
//     spelling keeps compiling;
//   * the SHUL/2 wire (diag/shul2_format.hpp, H1): 4 bytes on EVERY frame, verified before a
//     single byte inside the frame is believed.
//
// ── Why it lives in diag/, and why it is a leaf ─────────────────────────────────────
// The wire is diag/'s, and diag/ is a leaf layer: nothing in it may include calibration/
// (calibration already includes diag/blackbox_format.hpp, so the reverse include would
// invert the layering). So the algorithm moved DOWN, to the lowest layer that needs it, and
// the calibration spelling became a forwarding function. Two copies were the rejected
// alternative: two implementations of one checksum agree until the day one of them is
// "fixed". A namespace-scope `using` was rejected too — the documentation gate would publish
// it as a "type alias", a false kind for a function.
//
// ── Why bitwise, with no lookup table ───────────────────────────────────────────────
// A 1 KiB table in a header a composition root includes is 1 KiB of flash; the bitwise loop
// is eight shift-and-xor steps per byte. For the calibration record that was 180 bytes once
// per boot (Hardware Assumptions HA-140). For the wire it is every frame — about 436 bytes of
// CRC input per tick frame — and the arithmetic at that rate is a Hardware Assumption of the
// wire's sink (Part A2), not settled here.
//
// ── Why the textbook one ────────────────────────────────────────────────────────────
// Reflected polynomial 0xEDB88320, initial value all-ones, final inversion: the CRC-32 of
// zlib, of `crc32`, of PNG and Ethernet. So any other tool computes the same number —
// `python3 -c "import zlib; print(hex(zlib.crc32(b'123456789')))"` prints 0xcbf43926 — which
// is how every golden test derives its expected value WITHOUT running this code.
//
// PROS-free, allocation-free, and a leaf: <cstddef>, <cstdint> and <span> only.

#include <cstddef>
#include <cstdint>
#include <span>

namespace shulib::diag {

/// CRC-32/IEEE 802.3 over `bytes`: reflected polynomial 0xEDB88320, initial value all-ones,
/// final inversion — the same number `zlib.crc32`, `crc32` and every V5 tool produce, which
/// is what lets an INDEPENDENT implementation derive a golden test's expected value.
/// Computed BITWISE with no lookup table (header: 1 KiB of flash is the wrong trade in a header
/// a composition root includes). The check value: "123456789" gives 0xCBF43926.
[[nodiscard]] inline std::uint32_t crc32Ieee(std::span<const std::byte> bytes) noexcept {
    std::uint32_t crc = 0xFFFFFFFFU;
    for (const std::byte b : bytes) {
        crc ^= static_cast<std::uint32_t>(static_cast<std::uint8_t>(b));
        for (int bit = 0; bit < 8; ++bit) {
            const std::uint32_t mask = (crc & 1U) != 0U ? 0xEDB88320U : 0U;
            crc = (crc >> 1) ^ mask;
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

}  // namespace shulib::diag
