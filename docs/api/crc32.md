<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/crc32.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `crc32.hpp`

CRC-32/IEEE — the one checksum in shulib, in the one place both of its callers can reach.

This header declares **1** free function.

Extracted from [`include/shulib/diag/crc32.hpp`](../../include/shulib/diag/crc32.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`crc32Ieee`](#crc32ieee) — *free function*

<a id="crc32ieee"></a>

## `crc32Ieee`

```cpp
[[nodiscard]] inline std::uint32_t crc32Ieee(std::span<const std::byte> bytes) noexcept
```

CRC-32/IEEE 802.3 over `bytes`: reflected polynomial 0xEDB88320, initial value all-ones, final inversion — the same number `zlib.crc32`, `crc32` and every V5 tool produce, which is what lets an INDEPENDENT implementation derive a golden test's expected value. Computed BITWISE with no lookup table (header: 1 KiB of flash is the wrong trade in a header a composition root includes). The check value: "123456789" gives 0xCBF43926.

*free function, declared at [`include/shulib/diag/crc32.hpp:48`](../../include/shulib/diag/crc32.hpp#L48).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 34 lines</summary>

```text

 CRC-32/IEEE — the one checksum in shulib, in the one place both of its callers can reach
 (moved here from calibration/calibration_record.hpp at chunk H1 Part A1, 2026-09-29).

 ── Who calls it ────────────────────────────────────────────────────────────────────
   * the calibration record (calibration/calibration_record.hpp, R3d Part 2): 4 bytes over a
     180-byte record, read once at boot — calibration::crc32Ieee forwards here, so that
     spelling keeps compiling;
   * the SHUL/2 wire (diag/shul2_format.hpp, H1): 4 bytes on EVERY frame, verified before a
     single byte inside the frame is believed.

 ── Why it lives in diag/, and why it is a leaf ─────────────────────────────────────
 The wire is diag/'s, and diag/ is a leaf layer: nothing in it may include calibration/
 (calibration already includes diag/blackbox_format.hpp, so the reverse include would
 invert the layering). So the algorithm moved DOWN, to the lowest layer that needs it, and
 the calibration spelling became a forwarding function. Two copies were the rejected
 alternative: two implementations of one checksum agree until the day one of them is
 "fixed". A namespace-scope `using` was rejected too — the documentation gate would publish
 it as a "type alias", a false kind for a function.

 ── Why bitwise, with no lookup table ───────────────────────────────────────────────
 A 1 KiB table in a header a composition root includes is 1 KiB of flash; the bitwise loop
 is eight shift-and-xor steps per byte. For the calibration record that was 180 bytes once
 per boot (Hardware Assumptions HA-140). For the wire it is every frame — about 436 bytes of
 CRC input per tick frame — and the arithmetic at that rate is a Hardware Assumption of the
 wire's sink (Part A2), not settled here.

 ── Why the textbook one ────────────────────────────────────────────────────────────
 Reflected polynomial 0xEDB88320, initial value all-ones, final inversion: the CRC-32 of
 zlib, of `crc32`, of PNG and Ethernet. So any other tool computes the same number —
 `python3 -c "import zlib; print(hex(zlib.crc32(b'123456789')))"` prints 0xcbf43926 — which
 is how every golden test derives its expected value WITHOUT running this code.

 PROS-free, allocation-free, and a leaf: <cstddef>, <cstdint> and <span> only.
```

</details>
