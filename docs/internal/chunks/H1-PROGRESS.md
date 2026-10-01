# H1 — progress log (live)

- **Chunk:** H1 — the SHUL/2 telemetry wire (brief: [`H1-shul2-wire.md`](H1-shul2-wire.md))
- **Executor model:** opus
- Appended as the work goes, with a heading per part. Every time in a section header is a `date`
  reading taken when the section was written. **Nothing in this chunk has run on a robot, and the wire
  has never crossed a real link.** Every byte here is a host byte.

---

## Part A1 — the read side (host-only): the CRC move, the format, the reader

- **Started:** Tue Sep 29 2026 (`date` at log creation, see the first entry).
- **Tree at start:** `9239538`, clean, in the git worktree `.claude/worktrees/h1-a1` (branch `h1-a1`) —
  NOT the main working tree.
- **Go:** the coordinator's workflow run on the team lead's go (the brief's header: "Accept all" on
  D0–D9, 2026-09-26 ~23:10). Do not commit; no `pros` command; no `/dev/ttyACM*`. Mutation backups and
  probes under `.claude/coordinator/scratch/h1-a1/` (never `/tmp`, never the repository root).
- **Scope:** Part A1 ONLY (§1 items 1–5, §2 Part A1's tests and mutations 1–26, §3 Part A1's
  documents). NOT A2 (the sink, HA-168/169) and NOT B (the F9 freeze).

### 03:39 — start
- Read `docs/internal/RESUMING.md` (the chunk loop, the standards, pace rules 1–8) and the brief in full
  (the team lead's answers, the 28 Re-check amendments, D0–D9, §0–§6, "Found while reading"). This log
  created before any code. Next: re-check every file:line the brief cites against the tree (the tree
  moved since 2026-09-26: SRC1, T4 Part 1a, TANK1 Part A, the 2026-09-28 plan docs).

### 03:43 — the tree re-check: every file:line the brief cites, and each delta with how A1 adapts
Re-opened at `9239538` in this worktree. Unchanged citations (within a line or two) are not repeated:
`blackbox_format.hpp` :30-38, :48-53, :173-351 (ByteWriter/ByteReader), :353-362 (header widths),
:364-391 (whole-or-nothing encoders); `blackbox_reader.hpp` :32-52, :104-232, :129-136, :169-172;
`calibration_record.hpp` :28-39, :63 (includes `diag/blackbox_format.hpp`), :192, :256;
`session_info.hpp` :42-44; `motion_scheduler.hpp` :1179 (`r.t = now`, the raw clock); `line_format.hpp`
:66-75 (the continuation back-off); `version.hpp` :38-41; `wheel_speeds.hpp` :37; `run_summary.hpp` :80;
`debug_record.hpp` :10-17, :46-47, :97-100, :118-121, :191-212; `fault.hpp` :14-18;
`test/calibration_record_test.cpp` :38, :188-201; `api_doc_tool.py` :744 ("type alias");
`test/blackbox_format_test.cpp` :150 (independent LE readers).

Deltas, each with its adaptation:

1. **TANK1 Part A — the TSTR record flag.** `DebugRecord::tankSteerActive` was appended
   (`debug_record.hpp` :214-221), and the blackbox tick's flags byte now has THREE meanings: bit 0
   `clampedThisTick`, bit 1 `strafeFallbackActive`, **bit 2 `tankSteerActive`**, bits 3-7 written 0
   (`blackbox_format.hpp` :437-442 note, :473-475 encode, :557-561 decode). The tick width is unchanged
   (428) and `kFormatVersion` stayed 1. The brief (§1 item 2 "The Tick", §2 "Forward compatibility")
   says bits 2-7 are written 0 and ignored. **Adapted:** SHUL/2 1.0's tick flags are bit 0, bit 1,
   bit 2 as the blackbox defines them, bits 3-7 written 0 and ignored; the forward-compatibility test
   sets bits 3-7 (not 2-7) and checks bits 0-2 unchanged; the Tick golden pins bit 2 as well. No format
   decision moves: D2 makes the Tick the blackbox's bytes, so the bit arrives on the wire with no code
   of H1's. `diagnostics-plan.md` :45-48 (TANK1's paragraph: "H1's wire decoder should expose that bit
   rather than ignore it") is satisfied by the same fact and says so after A1.
2. **API 2.3.** TANK1 bumped `kApiMinor` to 3 (`version.hpp` :101; `kApiMajor` is at :46, brief said
   ~:48/~:75). The Hello carries `apiMinor` = 3 in this build; tests read `shulib::kApiMinor` rather
   than a literal where the value is not a hand-derived golden, and the golden Hello is built with
   explicit bytes. **Adapted:** the changelog block is "### Added since 2.3 (…)" under "## API 2.3",
   not "since 2.2" (the brief was written under 2.2). Still no bump (D7).
3. **Line drift in `blackbox_format.hpp`** (TANK1's notes): encodeTick :444-501 (brief ~:440-493),
   wheelCount encode :466 (brief ~:459), safeAngle :503-514 (~:495-506), decodeTick :516-587 with the
   width test at :521 (~:513) and the wheelCount read at :549 (~:541), the summary table :589-595
   (~:581-586), encode/decodeSummary :597-663 with its width test at :634 (~:625). Same code; cited
   from the tree below.
4. **`term_sink.hpp`**: `log()` stamps from the clock at :93 (brief ~:92); the caps are :282-283
   (brief ~:277-278). Same values (tag 16, message 200); A2's concern, not A1's.
5. **`version.hpp`/`telemetry_sink.hpp` LogLevel** (:63-83, brief ~:64-84): still no explicit values —
   landmine 1 holds.
6. **SRC1** moved `src/` into `robots/ team/ programs/ common/`. H1 A1 touches no `src/` file, so
   nothing to adapt; the brief's "Do not touch" list (T3 Part 2, G1 Part 2c, COMP-b) is historical —
   both landed. The live in-flight list is the coordinator's (comp-t, tank1b, DIAG1 B, SAFE-SAVE A,
   L9 A1, R5a A, T4 2a).
7. **SAFE-SAVE Part A is IN FLIGHT and edits `calibration_record.hpp`** (its draft: ~:52-54 and a new
   constant). A1 edits only the CRC's body and doc (:106-120 → a forwarding function) and the
   "Why a CRC here" comment (:28-39). Kept minimal; FLAGGED for the coordinator's merge.
8. **TANK1 Part B (in flight) may add ` xt` to result lines.** Result lines ride `log()` as text, so
   in SHUL/2 v1 they arrive inside Log frames, verbatim; no format impact. (A longer line is an A2
   question: the sink's 200-byte message cap is a writer policy.)
9. **H4 (planned 2026-09-27, `build-order.md` :1693-1695)** — the robot console, "carried by H1's
   SHUL/2 frames (its peer→runtime range carries the touches)". A second future consumer of D4's
   ranges; the ranges already reserve 64-127 peer → runtime and 2-63 runtime → peer for new data
   types, so nothing moves. Noted in the format header's range comment.
10. **Docs**: HA-140's *Source* is at :1437-1451 (brief ~:1426); guide ch.11 ("Reading the
    diagnostics")'s API-link sentence is at :489 (brief ~:460); `vexbuilder-integration.md` §5.2
    is :413-434, §5.3 :436-446, §1's H1 row :69, §0 :35; master plan §18.1 :828-830 ✓;
    `diagnostics-plan.md` the F9 question :27-53, the H1 row :86; build-order H1 status :1666-1669.
    *(Review round: the H1 row was :91 at `9239538` — D9 (b) under "H1-A1 review findings".)*

### 03:56 — resumed (the run that wrote the entries above stopped after the re-check; nothing else had been written)
- `git status` at resume: only this log untracked; the scratch directory empty. Nothing to reconcile.
- **Correction to delta 2 above:** `version.hpp` is 97 lines; `kApiMajor` is at **:48** (as the brief
  said) and `kApiMinor = 3` at **:92** (with TANK1's note :75-91), `kApiVersionString "2.3"` at :95.
  The ":101" / ":46" figures above were wrong. The adaptation is unchanged.
- Re-read myself before designing (not taken from the entries above): `blackbox_format.hpp` in full
  (the tick table :423-442 with TANK1's bit-2 note, encodeTick :447-501 with the wheelCount write at
  :466, safeAngle :508-514, decodeTick :519-587 with the width test at :521 and the wheelCount read at
  :549, the summary :589-663), `blackbox_reader.hpp` in full (the boundary note :32-52, the stop-and-
  mark-truncated tail :169-172), `calibration_record.hpp` :1-140 (the CRC :106-121, "Why a CRC here"
  :28-39, the include of `diag/blackbox_format.hpp` at :64), `debug_record.hpp` in full,
  `telemetry_sink.hpp` in full (`LogLevel` :64-84, no explicit values), `line_format.hpp` :66-75 (the
  continuation back-off), `session_info.hpp` :42-52, `fault.hpp` :44-83 (`FaultCode` 0-12),
  `test/blackbox_format_test.cpp` in full (its independent LE readers :153-172), `test/CMakeLists.txt`
  (the `*_test.cpp` glob; the doc gates), `.github/workflows/ci.yml` :40-140 (the two include greps and
  the ARM gate's command), `HANDOFF-remaining-work.md` :208-216 (the stale-binary recipe).

### ~04:11 (no `date` read; the probe file written at the end of this step has mtime 04:11) — the code: the CRC move, the format, the reader, the wheelCount fix, the comment sweep
- **`include/shulib/diag/crc32.hpp` (NEW)** — `diag::crc32Ieee`, moved verbatim (same body, doc
  comment carried over and extended). Leaf: `<cstddef> <cstdint> <span>` only.
- **`calibration/calibration_record.hpp`** — `calibration::crc32Ieee` is now a documented one-line
  FORWARDING function (`return diag::crc32Ieee(bytes);`), the "Why a CRC here" comment gains a
  sentence saying where the algorithm lives and why. **Delta from the brief (logged for the
  coordinator's merge with SAFE-SAVE Part A):** besides :28-39 and :106-121 the brief allowed, ONE
  more line changed — `#include "shulib/diag/crc32.hpp"` added after the existing
  `diag/blackbox_format.hpp` include (now :67-68). Unavoidable: the forwarding function must see the
  declaration; relying on a transitive include through `blackbox_format.hpp` would be an accident, not
  a contract.
- **`diag/blackbox_format.hpp`** — the wheelCount fix at its source: `encodeTick` writes the count
  clamped to [0, `DebugRecord::kMaxWheels`] (was: negative → 0, above 255 wraps); `decodeTick` decodes a
  byte above 8 as 8 and raises `corrupt`. Both doc comments say so. "What H1 inherits" (:48-60)
  rewritten: the Tick and Summary payloads ARE these bytes, and what that costs once F9 freezes.
- **`diag/blackbox_reader.hpp`** :49-56 — appended: SHUL/2 added its own per-frame check and resync;
  the FILE still has none.
- **`diag/term_sink.hpp`** :5 — "never the JSON wire (that is Shul2Sink at H1)" → the wire is BINARY,
  pointing at `shul2_format.hpp` / `shul2_reader.hpp`, the sink being A2's (D1).
- **`include/shulib/diag/shul2_format.hpp` (NEW)** — namespace `shulib::diag::shul2`: envelope
  constants and offsets, `FrameType` 1-5, the data-type ranges, roles, `kSignature`, `WidthRow` +
  `kWidthTable` (row 1.0 {256, 428, 168, 32, 14}) with the brief's static_asserts (plus one tying
  this header's own layout constants to the row), the checked API bytes (`static_assert` 0..255, then
  the cast), `ReaderProfile` + `kThisReader`, `Verdict` + `negotiate`, `ScanResult` + `scanFrame`,
  `encodeFrame` / `encodeHello` / `encodeLog` / `encodeEnd`, `decodeHello` / `decodeLog` / `decodeEnd`,
  and `decodeTick` / `decodeSummary` (thin: `blackbox::decode*` on `payload.first(knownWidth)`),
  `WireLevel` + `wireLevel` / `logLevelFromWire` (switches both ways), `utf8Prefix`.
- **`include/shulib/diag/shul2_reader.hpp` (NEW)** — `Shul2Reader` (whole capture, borrowed span,
  `next()` / `drain()` / `finished()` / `status()` / `counters()` / `latestSession()`), `CaptureStatus`,
  `ReaderCounters`, `SessionReport`.
- Probe: the three new headers + `calibration_record.hpp` compile clean on the host with the suite's
  flags and on ARM with the gate's flags (`.claude/coordinator/scratch/h1-a1/probe_headers.cpp`).

Design choices the brief left open, each with its reason:
1. **The Hello is decoded with VIEWS** (the brief: "its Hello (a view)"): `Hello`'s text fields are
   `string_view`s into the payload, ending at the first NUL. The reader borrows the capture anyway.
2. **`encodeHello` always writes THIS build's Hello** (signature, version, 1.0 widths) from a
   `SessionInfo` — there is no encoder from an arbitrary `Hello` struct. The writer never writes
   anything else; tests that need a foreign Hello (bad signature, 1.7, 2.0) patch bytes and re-frame
   (which recomputes the CRC) — that IS "hand-built".
3. **`encodeFrame` takes the raw type byte** (so tests can frame types 0/60/64/200) and accepts a
   payload already sitting at `out[10..]` (the in-place case A2's single scratch buffer will use).
4. **`Shul2Reader` takes a `ReaderProfile` (default `kThisReader`)**, not only `negotiate`: then a
   1.0 capture can be READ by a hypothetical 1.1 reader (OlderMinor end to end), and the known widths
   used for `extensionBytesIgnored` come from the profile's row for `min(streamMinor, newest)`.
5. **Delivered payload = the WHOLE payload**; `shul2::decodeTick` / `decodeSummary` / `decodeEnd` read
   the known prefix (the brief's `payload.first(knownWidth)`), so mutation 10 has one place to live.
6. **Sequence accounting covers every frame of an accepted session, unknown types and re-Hellos
   included** (the writer offered them; skipping them unaccounted would show as false gaps). The
   type dispatch comes after the sequence check.
7. **A refused session "stays open" through its End** until the next differing Hello: in a major
   this build cannot read, type 5 may not mean End ("everything else may change at a major").
8. **`sessionsLinkLossUnknown` counts ACCEPTED sessions only** (a refused session has no link
   figure; it is in `refusedSessions`), so after `finished()`: sessions = refused + known + unknown.
9. **`endDisagreements` for `framesFailed > lostFrames` applies only to seq-0 sessions** — for a
   late join the frames lost before the join were never seen, so the comparison means nothing (the
   brief's late-join test asks for `linkLossKnown false`, `sessionsLinkLossUnknown 1`, "nothing
   wraps"; the test also pins `endDisagreements 0` for it). `framesOffered ≠ seq` counts for any End.
10. **`lengthOverruns` counts per displaced tail EPISODE** (one pending NeedMore run cleared by a
    verified frame), not per NeedMore position — false sync pairs inside a corrupted frame's bytes
    would otherwise inflate it. The brief asks `≥ 1`.
11. **`SessionReport::lateJoin` is "did not start at its seq-0 Hello"** (a late join OR a mismatched
    re-Hello), while the COUNTER `lateJoins` counts only Hellos at seq > 0 with no session open.
12. **A counter the brief does not list: `framesVerified`** — `status()` needs "no frame verified".

### 10:41 — the tests (47 new cases), the goldens' Python, the first green suite
- **`test/shul2_test_support.hpp` (NEW)** — the shared capture builder (`Capture`, payload helpers
  built on the FORMAT'S OWN encoders — the Tick/Summary on `blackbox::encode*`), independent LE
  readers/writer (never `ByteReader`), `record(k)` with every field distinct, `checkSameRecordBits`
  (f64s by bit pattern), and a test-side LIVE `Reassembler` (what H2's socket loop will be:
  `scanFrame`, wait on NeedMore, the tail rule only at end of input).
- **`test/shul2_format_test.cpp` (NEW, 23 cases)** and **`test/shul2_reader_test.cpp` (NEW, 24
  cases)** — the brief's §2 Part A1 list, each case headed by what it would catch.
- **The CRC literals, computed OUTSIDE the code** (exact commands; outputs in brackets):
  - `python3 -c "import zlib; print([hex(zlib.crc32(s)) for s in (b'', b'a', b'123456789')])"`
    → `['0x0', '0xe8b7be43', '0xcbf43926']` (the check values);
  - the envelope golden: `python3 -c "import zlib; f=bytes([0xFE,0x53,0x03,0x5A,0x01,0x02,0x03,0x04,0x03,0x00])+b'abc'; print(hex(zlib.crc32(f[2:])))"`
    → `0x2eb168d9`;
  - the Log frame: `python3 -c "import zlib,struct; p=bytes([1,2,3,0,2,0])+struct.pack('<d',12.5)+b'MOThi'; f=bytes([0xFE,0x53,0x03,0x00])+struct.pack('<IH',7,len(p))+p; print(len(p), p.hex(), hex(zlib.crc32(f[2:])))"`
    → `19 01020300020000000000000029404d4f546869 0xbaf80e96`;
  - *(Superseded in the review round — D9 (a) under "H1-A1 review findings": the golden's values
    changed and its CRC is now `0x75a478de`; the committed derivation is the script in the test's own
    comment.)* The whole Tick frame: `python3 .claude/coordinator/scratch/h1-a1/tick_golden.py` → `0x45daba39`
    (the script builds the 428-byte payload from the blackbox tick TABLE with `struct.pack_into`, every
    field non-zero and each an exact binary value, then the envelope with seq 258, then `zlib.crc32`
    over bytes [2, 438)). The C++ test builds its expected 442 bytes with an independent LE writer from
    the same table (hand-derived IEEE-754 patterns in comments) and compares every byte.
  - The Hello has NO CRC golden, deliberately: its byte 33 is `shulib::kApiMinor`, which L9 Part A1
    (in flight) moves to 4 — a CRC literal over it would break on that merge for no format reason. The
    Hello golden checks bytes 32-33 against `version.hpp`, every other field against hand literals.
- **A test bug found and fixed on the first run** (worth recording because the API invites it): one
  case built its reader from a TEMPORARY (`Shul2Reader reader{without(c, 5)}`), and the reader
  BORROWS its span, so it read freed memory (lostFrames 0, no End). Fixed to a named buffer; the empty-
  capture case likewise; the constructor's `///` now says "never hand it a temporary buffer".
  (`BlackboxReader` has the same borrowing shape and the same hazard; not changed.)
- **First full green:** `./build/test/shulib_tests` → `test cases: 1772 | 1772 passed | 0 failed | 3
  skipped`, `assertions: 2668247 | 2668247 passed`. 11.0 s for the whole suite; the 47 new cases alone
  take ~1.8 s (the flip loop's 7,956 damaged captures included).

### 10:48 — the 26 mutations (+ 25b), each on a FRESH binary, observed red, restored
Driver: `.claude/coordinator/scratch/h1-a1/mutate.py` (scratch, not committed). Per mutation: back up
the file under `scratch/h1-a1/backup/`, apply an exact replacement (occurrence count asserted), touch
the SHUL/2, blackbox and calibration test TUs, `cmake --build build/test --target shulib_tests/fast`,
**require "Linking CXX executable shulib_tests"** (every run below linked), run `-tc="SHUL/2*,blackbox*"`,
record the failing cases, restore with a fresh timestamp. After the last one, the three headers were
`cmp`-identical to their backups and a clean relink ran green (77/77 filtered). (Two pre-existing
`ekf_fusion_seam_test.cpp` cases print a MESSAGE under the same filter; they never failed and are not
listed.) Baseline under the filter: 77 cases, 0 failed.

| # | Mutation | Red (the cases that failed) |
|---|---|---|
| 1 | CRC over the payload only (scan AND encode) | goldens: envelope, Log, whole Tick; the flip loop |
| 2 | reader skips the CRC check | flip loop, transport, garbage prefix, scanFrame, live 4000, chunking |
| 3 | resync skips the claimed length | flip loop, scanFrame, live 4000, chunking |
| 4 | 4096 bound removed | scanFrame (60000 → must be Skip consumed 1) |
| 5 | gap without the 2^32 modulo | the wrap test (a loss ACROSS the wrap) |
| 6 | negotiate accepts another major | the version table; the 2.0 session |
| 7 | negotiate refuses a newer minor | the version table; the 1.7 stream; the 1.1-reader test |
| 8 | same-minor width mismatch accepted | the version table |
| 9 | older minor checked against the newest row | the version table (1.1 profile); the 1.1-reader test |
| 10 | newer-minor Tick decoded from the whole payload | the 1.7 stream |
| 11 | extension bytes not counted (all 3 sites) | the 1.7 stream; the 1.1-reader test |
| 12 | logFixedBytes read as the bytes before the text | the 1.7 stream; the 1.1-reader test; wrong-width |
| 13 | Warn ↔ Info codes swapped | Log golden; level-code golden |
| 14 | unknown level code cast to LogLevel | unknown-code test; the level-7 Log round trip |
| 15 | UTF-8 prefix without back-off | utf8Prefix |
| 16 | a repeated Hello starts a new session | the re-Hello test |
| 17 | signature NUL → 0x20 | Hello golden |
| 18 | refused sessions' frames delivered | signature alterations; 2.0 session (after the fix below); 1.1-reader |
| 19 | a stale frame delivered | the stale test |
| 20 | tail NeedMore STOPS the reader | the tail case; the flip loop; the garbage prefix |
| 21 | truncated set although a later frame verified | the tail case; the flip loop; the garbage prefix |
| 22 | nonzero envelope flags refused | forward compatibility |
| 23 | nonzero reserved Hello byte refused | forward compatibility |
| 24 | link loss computed for a late join, unsaturated | late-join test; the saturation test |
| 25 | wheelCount decode clamp removed | wheelCount test |
| 25b | wheelCount ENCODE clamp removed (added: the brief's item 4 fixes both sides) | wheelCount test |
| 26 | errorX/errorY swapped in BOTH encodeTick and decodeTick | **round trips stayed GREEN** (SHUL/2 Tick round trip, blackbox round trips); **goldens RED** (SHUL/2 whole-Tick golden, blackbox per-field golden) — the reason the goldens exist, measured again |

**A test found too weak by a mutation, and fixed:** on its FIRST run, mutation 18 left the 2.0 test
GREEN. The 2.0 Hello there is 40 bytes, so it declares no widths, and every one of its frames failed
the width check even with the refusal gone — defence in depth hid the mutation. Added the dangerous
case to that test: a 2.0 stream that kept 1.0's Hello layout and frame widths (this build's Hello with
the major patched to 2, then a 1.0-shaped Tick, Log and End). Re-run: RED (the row above), and green
restored.

### 10:57 — the documents (§3 Part A1), and the prose review (greps run, hits read)
Every hand-edited document, with what moved (each written as a self-contained block for the merge):
1. **`docs/vexbuilder-integration.md`** — §5.2 REWRITTEN as the contract (:413-658): status stated
   exactly (format + reader exist, host-only; no sink, no program writes it, no socket; NOT frozen;
   "never crossed a real link"); 5.2.1 the rules (binary not JSON and why; LE binary64 bit-exact;
   canonical units; the raw time base; unknown values arrive as numbers); 5.2.2 the envelope table,
   the CRC spec (zlib-compatible, its coverage, the `python3 -c` check) and the resync rule with the
   live buffer (≥ 4110 bytes, the ~nine-tick-frame stall, then recovery) and the whole-capture tail
   rule; 5.2.3 frame types, directions, D4's ranges, one writer per capture, what v1 does not carry;
   5.2.4 the Hello table and what the signature does and cannot do; 5.2.5 the 428-byte Tick and
   168-byte Summary offset tables (from `blackbox_format.hpp`'s tables, units added), the tick flag
   bits incl. TANK1's bit 2, the Log and End tables; 5.2.6 the value tables (FaultCode 0-12,
   GateReason 0-12, MotionState 0-5, qualityClass 0-3, TickPhase 0-5 + spare 6-7, level codes);
   5.2.7 sessions, sequence, link loss, what every major keeps, and the version rules (D4, D8's
   older-minor rule) with the 1.7 / 2.0 tests named. §5.3: a "Markers, in v1 of the wire" paragraph
   (Log text only; a data frame is an additive type later). §1: the H1 row (❌ → 🚧, Part A1 landed,
   NOT frozen, never on a real link) and — **a delta, logged** — the blackbox row's "the working
   precedent SHUL/2 should follow" → "the precedent SHUL/2 followed: its Tick and Summary ARE the
   blackbox's bytes", because it is H1's claim and was false after A1. §0 item 3 (:33-36) — **a delta
   from the brief, which gives §0 to Part B:** "F9 is not written" was false after A1, so it now reads
   "F9 is not frozen (its format and reader exist since 2026-09-29, host-only — §5.2)"; Part B still
   owns the freeze wording. §7's risk row and §8.2 item 5 are untouched (still true; Part B's).
2. **`docs/shulib-v2-master-plan.md`** §18.1 (:828-833) — "`TermSink` is not the JSON wire" → "is not
   the wire", plus two sentences: binary not JSON (D1, why), the Tick/Summary are the blackbox's bytes,
   the format is `vexbuilder-integration.md` §5.2. Decision row 13 (:568) untouched (Part B's).
3. **`docs/diagnostics-plan.md`** — after the TANK1 paragraph (:57-67) a new "Answered at H1"
   paragraph answers the F9 question (D5: five frame types; RunSummary YES as the blackbox's bytes;
   MotionOutcome/per-motion results NO as data — Log text; no Triage; the TSTR bit arrives with no wire
   code of its own). The "Already planned" H1 row (:102) reads Part A1 ✅ host-only.
4. **`docs/hardware-assumptions.md`** HA-140's *Source* (:1451-1456) — appended: the bitwise CRC
   lives in `diag/crc32.hpp` since H1 Part A1, the calibration record calls it through the forwarder,
   the judgement unchanged; the wire's rate is a separate trade (A2's HA-169). No HA added (HA-168/169
   are A2's).
5. **`docs/changelog.md`** — a new "### Added since 2.3 (2026-09-29) — the SHUL/2 wire format and its
   reader" block at the top of "## API 2.3" (:19-62): ADDED (the three headers), FIXED (wheelCount,
   both sides), CHANGED-same-behaviour (the forwarder), the borrowing warning, "What you must do:
   nothing". (Brief said "since 2.2"; TANK1 moved the API to 2.3 — delta 2 at the top.)
6. **Guide ch.11 ("Reading the diagnostics")** — a new paragraph after "The exact signatures" (:491-495)
   linking `shul2_format`, `shul2_reader` and `crc32`, saying nothing writes the wire yet. (Named by
   title: COMP-b may renumber chapters.)
7. **`docs/roadmap.md`** — **a delta from the brief (which gives the roadmap to Part B only):** the M6
   `SHUL/2` box (:1821) is `[~]` with its partial stated and its owners named (A2 the sink, B the
   freeze). RESUMING's standard is "roadmap checkboxes under-claim honestly (`[~]` for partial)"; leaving
   `[ ]` would under-state, `[x]` would over-claim. The F9 row, the ~:1999 "You are here" sentence and
   M6's other boxes are untouched (Part B's / the coordinator's) — see "for the coordinator" below.
8. **`docs/internal/build-order.md`** H1 (:1670-1676) — a "Status 2026-09-29: PART A1 BUILT, HOST-ONLY,
   NOT FROZEN" line appended under the BRIEFED status. The `**Next:**` pointer (:632) is the
   coordinator's (it names every chunk in flight) — untouched.
9. `docs/api/` regenerated (new pages `crc32.md`, `shul2_format.md`, `shul2_reader.md`; changed
   `README.md`, `all-entities.md`, `blackbox_format.md`, `blackbox_reader.md`, `calibration_record.md`,
   `term_sink.md`; `mkdocs.yml`'s generated nav gains the three pages). `PROJECT-BRIEFING.md`'s
   generated block regenerated (LAST, after the suite binary existed).

**Deliberately NOT touched (with why):** guide ch.14 ("What it cannot do yet") :414-417 — "the live
telemetry wire to VexBuilder" is still not built, true until A2/H2; its rewrite is A2's.
`telemetry_sink.hpp` :4-6/:86 and `debug_record.hpp` :6-8, `run_summary.hpp` :11 (they name
`Shul2Sink`) — A2's sweep. `version.hpp` — Part B's (D7: no bump). The F9 row and freeze wording
everywhere — Part B's.

**The prose review — greps RUN, every hit READ** (public docs = `docs/` minus `docs/api/` and
`docs/internal/`, plus `README.md`):
- `grep -rn "SHUL/2\|SHUL2\|Shul2\|shul2" docs README.md --include=*.md` minus `docs/api/` and
  `docs/internal/` → 54 hits after the edits (counted at 10:59; the pre-edit listing was read first). Read all. Stale ones
  fixed above (vexbuilder §0/§1/§5.2/§5.3, master plan §18.1, diagnostics-plan). Left, still true:
  master plan :113, :148, :228, :267, :431, :452, :464, :567, :568 (Part B), :672-673, :697, :709,
  :763-764, :825; roadmap :67, :143 (F9 row, Part B), :1820, :1826, :1969, :1972, :2011, :2040;
  vexbuilder :90, :399, :730, :740. **Stale and NOT mine (flagged):** roadmap :1999 — "The `.vexbot`
  codegen and the `SHUL/2` wire are still unbuilt (G3–G4, H1)" inside a dated G2-era "You are here"
  paragraph; now half-false (the format and reader exist). Part B's list names "the ~:1960 sentence",
  which appears to be this one after the file grew.
- `grep -rn "JSON"` → master plan :512/:727 (the .vexbot), :882 (the legacy escapeJSONString),
  vexbuilder :192/:333 (the manifest's JSON lines), roadmap :1374/:1390/:1804 (legacy logger, the
  manifest) — none is about the wire. The two wire-JSON sentences (master plan §18.1, `term_sink.hpp`
  :5) are fixed; `vexbuilder` :431 and changelog :32 are mine (they name the old spelling on purpose).
- `grep -rn "crc32\|CRC-32\|crc32Ieee"` → HA-140 (updated), roadmap :283 (the record is CRC-32'd —
  true), changelog (mine), vexbuilder (mine), guide ch.11 (mine).
- `grep -rn -i "telemetry wire\|the wire\|live telemetry\|F9"` → guide ch.11 :483 ("what's planned
  beyond the terminal (live telemetry, replay)" — still planned: true), ch.14 :414 (A2's), the
  diagnostics plan :17/:27/:30/:44/:51 (pre-freeze wording — true until Part B), :279 (the H1 row of
  the chunk table — Part B), HA register hits on "wire" are about motor wiring (unrelated).
- `grep -rn "wheelCount"` → vexbuilder (mine), changelog (mine), roadmap :1105 (the IKinematics
  contract — unrelated). `grep -rn "calibration_record"` → HA-140 (updated), changelog :665 and
  roadmap :284 (the record's own description — true). `grep -rn "term_sink"` → roadmap :1332/:1338,
  guide ch.11 :15 (test names — true). `grep -rn -i "checksum\|bit flip"` → HA-140 :1440 ("the
  blackbox has no checksum" — still true of the FILE), guide ch.11 :493 (mine). *(Review round: also
  roadmap :265 and :1206, unrelated — D9 (c) under "H1-A1 review findings".)*
- New header names `crc32.hpp`, `shul2_format.hpp`, `shul2_reader.hpp` → only the hits above.

### 11:03 — deltas from the brief (the full list), and what the coordinator must know
**Deltas from the brief, each with its reason:**
1. Tick flags: bits 0-2 are meanings (TANK1 gave bit 2 `tankSteerActive`), bits 3-7 reserved — the
   forward-compatibility test sets 3-7 and checks 0-2; the Tick golden pins bit 2. (Brief: 0-1 / 2-7.)
2. API 2.3, not 2.2: the changelog block is "Added since 2.3"; the Hello's apiMinor is read from
   `version.hpp` in every test (no literal), and the Hello has no CRC golden for that reason.
3. `calibration_record.hpp` gains one `#include "shulib/diag/crc32.hpp"` line (:68) beyond the two
   ranges the brief allowed — the forwarder needs the declaration.
4. vexbuilder-integration §0 item 3 and §1's blackbox row edited in A1 (the brief gave §0 to Part B
   and said "change only H1's row" in §1): both sentences were false the moment A1 landed.
5. roadmap M6 `SHUL/2` box `[ ]` → `[~]` (the brief gave the roadmap to Part B only): the honest-
   partial standard.
6. Mutation 25b added (the ENCODE clamp) — the brief's item 4 fixes both sides; mutation 25 covers
   only the decode.
7. `Shul2Reader` takes a `ReaderProfile` (brief: only `negotiate` does) — lets a 1.0 capture be read
   end to end by a hypothetical 1.1 reader (OlderMinor), and keys extension counting to the profile.
8. A counter the brief does not list, `framesVerified` (status() needs it).
9. The chunking property also compares `bytesSkipped` and `crcFailures` between the live reassembler
   and the whole-capture reader (the brief asks only for the frames).
10. The flip loop also asserts `bytesSkipped == 442` exactly (the damaged frame's bytes, no more).
11. The brief's "Do not touch" list is historical (T3 Part 2, G1 2c, COMP-b have landed); the live
    in-flight list came from the coordinator. No `src/` file, no `tools/`, no worksheet, no
    `session_info.hpp` edit — `git status` confirms.

**For the coordinator's merge:**
- **`include/shulib/calibration/calibration_record.hpp` is also edited by SAFE-SAVE Part A (in
  flight).** Mine: the "Why a CRC here" paragraph (:28-42, one sentence appended at :39-42), the new
  include (:68), and the CRC's body/doc (:110-117 now: a forwarding function). SAFE-SAVE's draft cites
  "CRC-32 :106-121" — after this lands that citation moves to `diag/crc32.hpp`.
- **`docs/changelog.md`:** my block is "### Added since 2.3 (2026-09-29)" at the top of "## API 2.3".
  L9 Part A1 (in flight) bumps the API to 2.4 — if it lands first, this block belongs under
  "## API 2.4" as "Added since 2.4" (its text names no version otherwise).
- **L9's API 2.4 bump is merge-safe for my tests:** the Hello's API bytes are checked against
  `shulib::kApiMajor/kApiMinor`, never a literal.
- **TANK1 Part B (in flight) may add ` xt` to result lines:** result lines ride Log frames as text —
  no format impact, nothing to merge.
- **`docs/api/`, `mkdocs.yml` and `PROJECT-BRIEFING.md`** are regenerated here; regenerate at merge.
- **Stale, not mine (found while reading):** roadmap :1999 ("the `SHUL/2` wire [is] still unbuilt",
  in a G2-era "You are here" paragraph — Part B's list calls it "the ~:1960 sentence");
  vexbuilder-integration §0 :19-20 ("shulib cannot open a `.vexbot` file today. At all. … Everything
  in the G phase is unbuilt" — false since G1/G2; the G-phase's); `BlackboxReader` borrows its span the
  same way `Shul2Reader` does, so it has the same temporary-buffer hazard my own test hit (not
  changed — E1's; a doc line would do). The brief's "Found while reading" items (§8.2 item 4 still
  names the dropped legacy importer; `version.hpp` :24-28's policy wording; `ByteReader::text`
  joining around an interior NUL) are unchanged and still open.

### 11:09 — every gate, final lines (on the final tree; nothing committed)
- Host suite (`cmake --build build/test -j8` relinked after the last test edit; doc gates ran in it):
  `test cases: 1772 | 1772 passed | 0 failed | 3 skipped` / `assertions: 2668260 | 2668260 passed | 0
  failed` — 1,725 before + the 47 new cases.
- `python3 tools/src_build_gate.py self-test` → `src build gate self-test: OK (189 detector cases, each
  against a real build)`; `check` → `src build gate: PASS (all 6 builds link; …)`, each build "exit 0,
  8/8 TUs compiled, … 0 disallowed". Nothing in `src/` changed; this is the proof.
- ARM header gate (the CI command, 195 headers as one TU, `-Wconversion -Wsign-conversion -Wshadow
  -Werror -Os -mcpu=cortex-a9 …`) → `ARM compile gate: CLEAN (all v2 headers)`.
- CI greps → `library is PROS-free (all of include/shulib except hal/pros/)`; `layering holds: core is
  sim-free`. The three new headers include only `<…>`, `diag/`, `hal/telemetry_sink.hpp` and
  `shulib/version.hpp` (landmine 6).
- `api_doc_tool`: self-test `OK`; check-coverage `3115 public entities across 159 headers, all
  documented`; check-fresh exit 0 (silent); check-examples `467 quoted lines, all verbatim (5 source
  files); 3 listings of whole-run files, each one unbroken run`; check-removability `no public doc
  references docs/internal/`.
- `doc_staleness_audit.py` self-test `OK (10 detector/filter cases)`; audit `clean (27 live docs …)`.
- `briefing_status.py check` → `generated block current` (regenerated after the suite binary existed).
- `git status`: only the files listed in this log; no `src/`, no `tools/`, no `test/` file but the three
  new ones. **Part A1 stops here: not committed** — the coordinator verifies and commits. A2 (the sink,
  HA-168/169) and B (the F9 freeze) are not started.

---

## H1-A1 review findings and their disposition

### 21:46 — start (the builder again; three reviewers: correctness, tests, docs)
- The coordinator relayed 23 findings from three lenses (correctness 5, tests 10, docs 8). Overlaps,
  deduplicated: correctness #1 = tests #2 (a re-Hello's sequence step; the reviewers' R5 = N1);
  correctness #4 = tests #6 + #7 + one bullet of #9 (the 4096 scan bound R2 = N3, the stale boundary
  R3 = N10, `truncatedAt` earliest R4 = N2). So 18 distinct items.
- Re-read before touching anything: RESUMING's standing rules (the chunk loop §3, the standards, pace
  rules 1–8), this log in full, the brief in full, `shul2_format.hpp`, `shul2_reader.hpp`, the three test
  files, and the reviewers' scratch (`review-correctness/rmutate.py`, `review-testgate/mut.py` — both
  mutated SCRATCH COPIES of the tree, never this worktree). The worktree's three headers are
  `cmp`-identical to my backups of 10:46-10:47 (`scratch/h1-a1/backup/`): nothing moved since the last
  entry.

### 03:37 — resumed after the usage-limit stop (`date` at resume: Wed Sep 30 02:57:22 EDT 2026)
The run that wrote the entry above was stopped by the account's usage limit (2026-09-29, between
~22:20 and ~01:05 EDT) with nothing logged after it. Its work was in the tree and in its scratch
(`scratch/h1-a1/review-fix/`), so this run first established what it had done, trusting none of it:
- **Its trail, from file times:** `tick_golden_v2.py` 21:47; `cost_probe.cpp` 21:50;
  `blackbox_reader.hpp` 22:08; `vexbuilder-integration.md`, `roadmap.md`, `hardware-assumptions.md`
  22:09; `docs/api/` regenerated 22:09:48; a full build 22:10-22:11 (`review-fix/build_full.log`); a new
  mutation driver (`review-fix/mutate_review.py`, 22:14) that ran 50 mutations 22:15-22:20
  (`review-fix/results_221504.jsonl`: all 50 red, every restore `cmp`-true). The two test files and the
  two new headers carry its edits too (their edit times are lost: the headers' to its driver's restore
  bumps at 22:20, the tests' to a touch of every `test/*.cpp` at 02:55:49, see below).
  `build-order.md` and `changelog.md` were not touched by it, nor this log after its start entry.
- **`git status`** lists only this chunk's files; `git diff -- src/ Makefile tools/ .github/
  test/CMakeLists.txt` is EMPTY — no gate self-test plant was left behind, and no `src/` file moved.
- **No mutation left applied.** The three headers its driver backs up are `cmp`-identical to its backups
  (`review-fix/backup/`); `blackbox_format.hpp` is also identical to the first round's backup.
  `shul2_format.hpp` and `shul2_reader.hpp` differ from the FIRST round's backups (10:46) only by the
  review round's intended edits — a `diff` shows comment changes plus exactly one code line, the
  `decodeHello` major check (C2 below).
- **The binary on disk was not trusted.** It had been relinked at 02:56:03, before this run began (every
  `test/*.cpp` touched at 02:55:49 and all 178 objects rebuilt — the coordinator's, not this run's).
  Rebuilt from source: the briefing regenerated, `cmake -S test -B build/test`, then
  `cmake --build build/test --clean-first -j8` (03:10:50; 178 objects compiled, "Linking CXX executable
  shulib_tests", every doc gate green), the briefing regenerated again once the binary existed, and
  `./build/test/shulib_tests` → `test cases: 1781 | 1781 passed | 0 failed | 3 skipped`, `assertions:
  2671224 | 2671224 passed | 0 failed`. 1,725 before A1 + 56 of A1's (47 of the first round + 9 of the
  review round).
- **Correction to the entry above:** the relayed list has **24** findings, not 23: correctness 5
  (C1-C5), tests 10 (T1-T10), docs **9** (D1-D9). Overlaps: C1 = T2 (R5 = N1; T2 adds N8); C4 = T6 + T7
  + T9's stale-boundary bullet (R2 = N3, R4 = N2, R3 = N10). So **21 distinct items**, not 18.

### 03:44 — the dispositions: each finding re-checked at its file and line on the final tree
Numbering: C = correctness, T = tests, D = docs, in the order relayed. Every mutation named below was
re-run by THIS run on the final tree, on a fresh binary, under the sound recipe (next entry); "red"
means observed red there, with the failing cases taken from doctest's XML report. The killed run's
edits were each re-read and verified, not taken as done; two of its statements were WRONG and are
corrected here (C5's "densest pattern", and a recipe gap it left in its own final step — next entry).

- **C1 + T2 (major) — a repeat Hello's sequence step untested (R5 = N1), and its extension bytes
  (N8). REAL: a test gap; the code was right** (`shul2_reader.hpp` `onHello` accounts a repeat's seq
  through `inSequence` before counting it). Tests: the re-Hello case (`shul2_reader_test.cpp:905`) now
  closes with an End and asserts `lostFrames`, `staleFrames`, `linkLostFrames`, `endDisagreements` and
  `sessionsLinkLossUnknown` all 0 with `linkLossKnown`; a new case at a writer's real cadence (`:950`:
  a re-Hello once 100 frames followed the last — Hellos at 0, 101, …, 1010 — over 1,000 Ticks: nothing
  lost, nothing link-lost); a stale identical re-Hello (seq 3 after 5) inside the stale case (`:766-780`:
  not delivered, not a repeat, `staleFrames` 1); and the 1.7 stream carries a re-Hello (`:1020`), so
  `extensionBytesIgnored` is 16 × 11. **N1 red** (5 cases), **N8 red** (the 1.7 stream). The
  reviewer's probe re-run on the final tree (`review-correctness/probes/r5_probe.cpp`): `repeats=9
  lost=0 linkLost=0 linkKnown=1 delivered=1011`.
- **C2 (minor) — `decodeHello` read another major's bytes 12 onward with 1.0's layout and set
  `complete`. REAL; fixed at its source.** `shul2_format.hpp:679`: `decodeHello` now returns after the
  version fields when `versionMajor != kVersionMajor`, as it already did for a short payload, so
  `complete` stays false and every later field keeps its default. Documented at `Hello` (:350-354,
  `complete` :360-361), `decodeHello` (:660-666), `ReaderProfile` (:310-312: a profile of another major
  can NAME a stream but never accept one, because `complete` is false for it) and
  `SessionReport::hello` (`shul2_reader.hpp:152-153`). Verdicts do not move: `negotiate` judges the
  major before `complete` (the version table passes unchanged). The finding's alternative — only
  amending the doc — was rejected: the contract says another major is refused whole, "read nothing
  from it" (`vexbuilder-integration.md` §5.2.7), and a report that prints a future layout's bytes as a
  routine id is exactly the confident nonsense the reader exists to refuse. Tests: a new case
  (`shul2_format_test.cpp:520`: majors 0, 2 and 65535 over 1.0's whole layout — `complete` false,
  role/flags/widths/epoch/API bytes 0, every text view empty, still `RefusedMajor`; the same bytes at
  major 1 decode whole) and the 2.0 look-alike's report (`shul2_reader_test.cpp:1117-1125`). **Mutation
  C2 (the fix reverted) red** in both. The reviewer's probe P1 re-run on the final tree
  (`review-correctness/probes/reader_probe.cpp`): `verdict=REFUSED_MAJOR complete=0 role=0 routine=''`,
  `declared tick=0 summary=0 epoch=0 apiMajor=0` — its own assertion that `complete` is true now FAILS,
  which is the defect gone.
- **C3 (minor) — the contract never told a live reader to apply the tail rule when its stream ends.
  REAL (a contract gap).** Stated in `vexbuilder-integration.md` §5.2.2 (:473-478: when a live stream
  ends, what is held IS a whole capture — apply the tail rule, or a corrupted length in the last 4 KB
  holds back every frame after it, the End included, for good; an idle timeout is the reader's own
  policy, with its cost), in `scanFrame`'s doc (`shul2_format.hpp:451-454`) and in the reader's header
  ("What a live reader must hold, and what it must do when its stream ends", `shul2_reader.hpp:68-77`).
  Test: `shul2_reader_test.cpp:1244` — the brief's named tail case (428 → 2476 on the third frame before
  the End) fed live one byte at a time: while the stream is open the reassembler has emitted 0 1 2 and
  holds everything from the bad length on; once it ends, the tail rule delivers 0 1 2 4 5 6, the same as
  the whole-capture reader. **M20 red** on it (the comparison with the whole-capture reader). The
  reviewer's probe re-run (`review-correctness/probes/live_eof.cpp`): waiting `0 1 2 (held 1372
  bytes)`, with the tail rule `0 1 2 4 5 6`.
- **C4 + T6 + T7 + T9's stale bullet (minor) — the 4096 bound (R2 = N3), the 2^31 stale boundary
  (R3 = N10) and `truncatedAt` = the EARLIEST candidate (R4 = N2) untested at their edges. REAL: test
  gaps.** Tests: a 4096-byte payload scanned as a Frame, consumed 4110 (`shul2_format_test.cpp:1141-1148`);
  a Log of exactly 4096 payload bytes (a 255-byte tag, the longest a u8 declares) delivered by the
  reader (`shul2_reader_test.cpp:498`); d = 2^31 exactly stale and d = 2^31 − 1 lost (`:798-805`); and a
  last frame (a Log) carrying a false header `FE 53 … A0 0F` (length 4000) cut at every offset:
  `truncatedAt` is always the Log's start, and the false candidate is asserted to run off the end too
  (`:549`). **N3 red** (2 cases), **N10 red**, **N2 red**.
- **C5 (minor) — the worst-case reading cost, unstated. REAL (informational); stated — and the killed
  run's statement CORRECTED.** It wrote "10 s per MB every 5 (the densest pattern whose every pair
  declares a length the rule accepts)". Period 5 is not the densest. A candidate at p forces 0x53 at
  p + 1 and a length byte of at most 0x10 at p + 9, so p + 1, p + 8 and p + 9 cannot start another;
  an exhaustive search (`resume/densest.py`, every subset of Z_P for P ≤ 22) and an exact one with no
  period bound (`resume/densest_karp.py`: Karp's maximum-mean cycle over the 512 states of the
  constraint's 9-byte window) both give **3 candidates per 10 bytes**, e.g. `FE 53 0F FE 53 0F FE 53 00
  10` repeated (lengths 4096, 3923, 3923): about 1,197 bytes of CRC per input byte, 2.9× the reviewer's
  one-per-10 pattern. Measured (`resume/cost_probe2.cpp`, `g++ -O2`, best of 3, 1,000,000 bytes each;
  `resume/crc_rate.cpp` for the CRC's raw rate), 2026-09-30:
  | pattern | 03:28, load ~17 (CRC 98.2 MB/s) | 03:20-03:25, load 26-34 (CRC 54.7 MB/s) |
  |---|---|---|
  | one candidate per 10 bytes (the reviewer's P6) | 5.97 s | 7.14 s |
  | one per 5 bytes | 9.65 s | 13.67 s |
  | four per 16 bytes (`FE 53` ×4, `00 10` ×4) | 12.75 s | 12.23 s |
  | **three per 10 bytes (the densest)** | **14.39 s** | **23.33 s** |
  | uniform random bytes | 0.3 ms | 0.6 ms |
  (The host was shared with other builders throughout; the reviewer's P6 on 2026-09-29 was 4.8 s.)
  Stated in `shul2_format.hpp`'s resync paragraph (:50-61), the reader's "What reading costs"
  (`shul2_reader.hpp:79-84`) and `vexbuilder-integration.md` §5.2.2 (:480-487), each with the
  table-driven-CRC note (the bitwise one is the brain's choice, for flash). No code change: the reader is
  host-only and has no time budget.
- **T1 (major) — a writer restart without an End (a seq-0 Hello inside an OPEN session) untested
  (N26). REAL: a test gap.** Test: `shul2_reader_test.cpp:864`, with an identical restart Hello and with
  one of another epoch: both sessions delivered whole (0-5, then 0-4), `sessions` 2, no stale, no loss,
  no repeat, no mismatch, no late join, `sessionsLinkLossUnknown` 1 (the first session never sent its
  End), the second read from its seq-0 Hello with its link loss known. **N26 red.** The contract says it
  (§5.2.7 :657-659).
- **T3 (major) — a late join followed by the writer's next periodic re-Hello untested (N28). REAL.**
  The random-offset case (`shul2_reader_test.cpp:813`) now carries a second re-Hello at seq 202 after
  the join: `sessions` 1, `helloRepeats` 1, `helloMismatches` 0, no loss, no stale. **N28 red.**
- **T4 (major) — the Hello text fields' width bound never exercised; a test comment claimed it was
  (N4, N4b). REAL.** The comment at the round-trip case is corrected (`shul2_format_test.cpp:457-460`:
  this build's encoder always ends a field at a NUL, so the decoder's own bound is the next case's). New
  case `:480`: a HAND-BUILT Hello with every text field filled to its width with no NUL (a letter per
  field, so a view that crosses into the next field shows) and the reserved tail 0xFF, re-framed so the
  CRC verifies, decoded directly AND through the reader (whose report views the capture itself, so an
  over-read meets real neighbouring bytes): every view is exactly its width. **N4 red, N4b red.**
- **T5 (minor) — the public decoders' short-payload guards untested (N13, N18). REAL.** New case
  `shul2_format_test.cpp:1225`: `decodeTick`, `decodeSummary`, `decodeEnd`, `decodeLog` and `decodeHello`
  each refuse 0, 1 and width − 1 bytes, and decode at exactly the width. Every short payload is a view
  into a larger buffer, so a missing guard reads bytes that exist and returns a deterministic wrong
  `true` rather than undefined behaviour off the end. **N13 red, N18 red.** The reviewers called their
  End mutant (N19) equivalent — it was, by construction: it also dropped `payload.first(32)`, leaving a
  bounds-checked read. **Added N19b** (only the guard removed, `first(32)` kept — a real over-read):
  **red** on the same case.
- **T8 (minor) — `status()` unpinned for an accepted session followed by a refused one (N32). REAL.**
  The 2.0 case (`shul2_reader_test.cpp:1137-1148`) adds a standard session followed by its writer's
  re-open under major 2: `sessions` 2, `refusedSessions` 1, the latest verdict `RefusedMajor`, status
  `Ok`. **N32 red.**
- **T9 (minor) — several counter and boundary rules untested. REAL, each:** a refused session's
  identical re-Hello (N6: the look-alike, `:1104-1115` — one session, one repeat, 4 frames in the refused
  session); a mismatch is not a late join (N7: `:938`); a late join's End checked for `framesOffered`
  (N12: `:644-648`); a stale End does not close its session (N20: `:782-796`); the loss sums saturate
  (N5, N21: `:691` — three maximal gaps of 0x7FFFFFFE, then a second session's lost frame); the 255-byte
  tag encodes (N11: `shul2_format_test.cpp:1197-1204`); `utf8Prefix` on a view whose next byte is 0x80
  (N14: `:415-422`); and the stale boundary (N10, under C4). **N5, N6, N7, N11, N12, N14, N20, N21 red.**
  Added **A18b** (a refused session's identical re-Hello falls through to the accepted path and is
  delivered): **red** (the 2.0 case).
- **T10 (minor) — the logged recipe gives false greens on a fresh build directory (header mutations,
  ODR). REAL for a fresh directory; the first round's own runs were valid** (the reviewer read their
  `compiler_depend.make` at 10:40:04, before the 10:43-10:47 mutations; this run could not re-read it — it
  has been regenerated since — so it re-ran all 27 first-round mutations under the sound recipe instead:
  all red, next entry). The first round's wording here ("After the last one, the three headers were
  `cmp`-identical to their backups") was accurate; the per-mutation `cmp` claim the reviewer quotes was
  in the builder's REPORT, not this log, and the first driver had no such check. The sound recipe, now
  with a per-mutation `cmp`, is the next entry.
- **D1 (minor) — roadmap :1999 still said the wire is unbuilt. REAL; fixed** by the killed run in the
  paragraph's own dated-correction style (`roadmap.md:2000-2001`: "*(Corrected 2026-09-29, H1 Part A1:
  the `SHUL/2` wire's format and its reader now exist, host-only; its sink and the F9 freeze do not, and
  it has never crossed a real link.)*"). Re-read: the sentence and the M6 box (:1821) now agree.
- **D2 (minor) — the Tick golden's "a swap of any two shows" was false (repeated values). REAL;
  fixed at the root** by the killed run: every field now holds a distinct value (51 f64s distinct by
  bit pattern — wheel voltages 101..108, currents −101..−108, phases 17..24, and `dt` 0.375, `imuYaw` 1.5,
  `imuYawRate` −2.5 among them; the six u8s and three u32s distinct too), the comment says what the
  golden still cannot see (a swap of two flag BITS, one byte 0x07) and that `blackbox_format_test`'s
  per-field golden does, and the CRC is **0x75a478de**. Verified by this run: the script as committed
  in the test's comment (`shul2_format_test.cpp:246-257`), extracted and run → `0x75a478de`
  (`resume/tick_golden_from_test_comment.py`); every hand-derived bit pattern in the test re-derived with
  `struct.pack` (all 27 scalar values and the three arrays match). New mutations: **26b** (`dt` ↔
  `targetPose.heading`, both 0.5 in the OLD golden, swapped on both sides) **red** — the whole-Tick
  golden, the blackbox per-field golden and the non-finite-Angle case; **26c** (flag bits 0 ↔ 1 on both
  sides) **red** in the blackbox per-field golden and the forward-compatibility case, with the whole-Tick
  golden green — exactly its stated blind spot. **M26 red** as before.
- **D3 (minor) — `blackbox_reader.hpp` still listed only a non-finite heading as raising `corrupt`;
  the blackbox header names were never grepped. REAL; fixed** (`blackbox_reader.hpp:21-24` and
  :41-43 name the wheelCount clamp). The killed run also corrected a pre-existing slip the reviewer
  did not name: the comment said "raises corrupt()", but `BlackboxReader` has no `corrupt()` — the flag is
  `decodeTick`'s out-parameter (`blackbox_format.hpp:533`); it now says so. The greps are in the prose
  review below.
- **D4 (minor) — HA-140's *Settle* and *Contained* not reconciled with the shared CRC. REAL;
  fixed** (`hardware-assumptions.md:1464-1466`: the table decision is now made for both formats at once,
  with the wire's sink; :1469-1472: one function shared by two formats, a change moves both, both pin it
  with CRCs computed outside the code — verified: the record's golden carries `0x45C46E98` from
  `zlib.crc32`, `calibration_record_test.cpp:124-125`). HA-169 is deliberately NOT named: it is not
  registered until Part A2 (the same reason as D8).
- **D5 (minor) — §5.2.1 said heading errors are "not wrapped". REAL; fixed** (`vexbuilder-integration.md`
  :439-443: heading errors, residuals and corrections are plain radians the decoder neither wraps nor
  validates; `errorHeading` is written as the shortest signed error in (−π, π]). Verified: its only two
  writers compute it with `Angle::errorTo` (`move_to_pose.hpp:214` and `:384`, `turn_to.hpp:123`), which
  returns (−π, π] (`angle.hpp:62-66`).
- **D6 (minor) — §5.2 omitted the reader's delivery rules; §7 had no F9 row. REAL; fixed**
  (§5.2.7 :660-666, "What a reader delivers": only inside an accepted session, exact declared widths,
  the Log's non-text bytes equal to `logFixedBytes`, frames outside a session or in a refused one skipped
  and counted; §7 :732, an F9 row: published, not frozen, until H1's last part).
- **D7 (minor) — the build-order H1 entry: a dated line edited without a log entry, and out of order.
  REAL; fixed by this run** (`build-order.md` H1): the 2026-09-26 line restored verbatim ("… until H2.
  Nothing built.") and this part's status placed ABOVE it, newest first, closing "The status line below
  is kept as written" (the convention at the TANK1 entry). The status itself was rewritten for the
  reviewed state (56 cases; the findings; 51 mutations re-run).
- **D8 (minor) — `shul2_format.hpp`'s public prose cited HA-168 and "the planned robot console". REAL;
  fixed** (:12-13: "a Hardware Assumption the wire's sink (H1 Part A2) will register"; :67: "and any
  later peer → runtime data"). `grep -rn "HA-16[89]\|robot console"` over `include/shulib/diag/`, the
  rendered pages and the SHUL/2 tests → nothing.
- **D9 (minor) — this log's accuracy. REAL, each:** (a) the whole-Tick golden's command at the 10:41
  entry points at a gitignored scratch script (`.claude/` is gitignored) and its value `0x45daba39` is
  obsolete since D2: the committed, reproducible derivation is the script in the test's own comment
  (`test/shul2_format_test.cpp:246-257`), which prints `0x75a478de`; (b) the 03:43 re-check's "the H1
  row :86" in `diagnostics-plan.md` was :91 at `9239538` (`git show 9239538:docs/diagnostics-plan.md`);
  (c) the 10:57 checksum grep listed 2 hits of 4 — also `roadmap.md:265` (calibration persistence "with a
  checksum") and `:1206` (C3's XOR checksums), both unrelated and still true. Corrected by these notes;
  the three lines above carry a pointer here.

### 03:46 — the mutations, re-run on the final tree under the sound recipe (T10)
Driver: `scratch/h1-a1/review-fix/resume/mutate_resume.py` (scratch, not committed). It IMPORTS the
definitions the killed run ran (`review-fix/mutate_review.py`: the reviewers' surviving mutants, C2,
26b, 26c; through it, the first round's 26 + 25b from `scratch/h1-a1/mutate.py`) and adds A18b and
N19b. **The recipe:**
0. One NORMAL `cmake --build build/test -j8` first (every doc gate runs, and `compiler_depend.make`
   then lists every TU's headers); the filtered baseline must be green (`102 cases, 0 failed`).
1. Per mutation: back up each touched file, apply exact replacements (each `old` must occur exactly
   `count` times, or the run is void).
2. `cmake --build build/test --target shulib_tests/fast -j8`; REQUIRE "Linking CXX executable
   shulib_tests" AND that the "Building CXX object" lines cover EVERY object `compiler_depend.make` lists
   as including a mutated header (2 for `shul2_format.hpp`/`shul2_reader.hpp`, 17 for
   `blackbox_format.hpp`) — otherwise an includer keeps a stale inline definition and the linker may
   keep it (the ODR trap T10 names). Every row below had `missingIncluders: []`.
3. Run `-tc="SHUL/2*,blackbox*,*alibration*"` with doctest's XML reporter and take the failing cases
   from it. (The killed run parsed the console's "TEST CASE:" lines, which also name two cases that only
   print a MESSAGE under this filter — `C1 routine: a 2% wheel miscalibration …` and `EkfFusion: a small
   persistent disagreement …` — so each of its failing lists carried two names that never failed; its
   summary counts were right.)
4. Restore from the backup, `cmp` it against the backup (every row: true), bump its timestamp.
5. **Relink with `/fast` BEFORE the final normal build** — a gap found here, at 03:39: the doc gates run
   before the link and the briefing check executes the binary ON DISK, which is still the last mutant,
   so the first run's closing normal build FAILED ("Suite … RED — 1 case(s) failing") and never
   relinked. (The killed run's driver had no closing build at all, which is why the binary it left was a
   mutant.) Then the normal build, the filtered cases green, every touched file `cmp`-equal to its backup.

Runs: 03:33:36-03:39:45 (51 mutations, `resume/results_033336.jsonl`; closing step relinked by hand as
in step 5, then `102 cases | 102 passed`) and 03:42:10-03:42:44 (N19b, with step 5 in the driver:
relinked, normal build rc 0, `102 cases, 0 failed`, `cmp` true). **52 of 52 red.**

| # | Mutation (the review round's, and this run's) | Red (the cases that failed) |
|---|---|---|
| N1 (= R5) | a repeat Hello skips the sequence rule | random offset; clean stream at N = 100; stale (the stale re-Hello); re-Hello; 1.7 stream |
| N2 (= R4) | `truncatedAt` records the LATEST tail candidate | truncatedAt with a false frame inside |
| N3 (= R2) | `scanFrame`'s bound `>=` (a 4096-byte payload refused) | scanFrame; a Log of exactly 4096 bytes |
| N4 | a Hello text field read to `width` inclusive | Hello fields that fill their widths |
| N4b | the field's width bound dropped | Hello fields that fill their widths |
| N5 | the loss sums wrap (plain add) | loss saturation |
| N6 | a refused session's identical re-Hello starts a new session | the 2.0 case (the look-alike) |
| N7 | a mismatched re-Hello also counts as a late join | re-Hello |
| N8 | a repeat Hello's extension bytes not counted | 1.7 stream |
| N10 (= R3) | stale boundary `d > 2^31` | stale (the boundary) |
| N11 | `encodeLog` refuses a 255-byte tag | encoders; a Log of exactly 4096 bytes |
| N12 | a late-joined session's End not checked for `framesOffered` | late join |
| N13 | `decodeTick`'s short-payload guard removed | short payloads refused |
| N14 | `utf8Prefix` `<` for `<=` | utf8Prefix |
| N18 | `decodeSummary`'s short-payload guard removed | short payloads refused |
| N19b | `decodeEnd`'s guard removed, `payload.first(32)` kept (added here) | short payloads refused |
| N20 | a stale End still closes its session | stale (the stale End) |
| N21 | the link-loss total summed without saturation | loss saturation |
| N26 | a seq-0 Hello inside an open session judged as a repeat | restart without an End |
| N28 | a late-joined session keeps the PREVIOUS Hello for repeats | random offset (the re-Hello at 202) |
| N32 | `status()` Refused when the LATEST session was refused | the 2.0 case (accepted, then refused) |
| C2 | C2's fix reverted (another major's bytes 12+ decoded) | another major's Hello; the 2.0 case |
| 26b | `dt` ↔ `targetPose.heading` in BOTH encodeTick and decodeTick | whole-Tick golden; blackbox per-field golden; non-finite Angle |
| 26c | tick flag bits 0 ↔ 1 in BOTH | blackbox per-field golden; forward compatibility — the whole-Tick golden GREEN, its stated blind spot |
| A18b | a refused session's identical re-Hello is delivered (added here) | the 2.0 case |

**The first round's 27, re-run under this recipe on the final tree: all red** (failing-case counts):
M1 4, M2 6, M3 4, M4 1, M5 2, M6 3, M7 3, M8 1, M9 2, M10 1, M11 2, M12 3, M13 2, M14 2, M15 1, M16 6,
M17 1, M18 2, M19 1, M20 4, M21 3, M22 1, M23 1, M24 3, M25 1, M25b 1, M26 2 — the cases of the first
table plus, where they apply, the review round's new ones (M6 and M16 also turn the new major and
cadence cases red; M20 the live end-of-stream case). M26 is red in the whole-Tick golden and the blackbox
per-field golden; the round trips stay green, as the first time.

### 03:48 — the review round's documents, and its prose review (greps RUN on the final tree, hits READ)
**Documents the review round moved** (each a self-contained edit for the merge):
`include/shulib/diag/shul2_format.hpp` and `shul2_reader.hpp` (the C2 code line and the prose of C2,
C3, C5, D8), `include/shulib/diag/blackbox_reader.hpp` (D3), `docs/vexbuilder-integration.md` §5.2.1
(D5), §5.2.2 (C3, C5), §5.2.7 (D6, T1's restart sentence), §7 (D6's F9 row), `docs/roadmap.md` :2000-2001
(D1), `docs/hardware-assumptions.md` HA-140 (D4), `docs/internal/build-order.md` H1 (D7), this log (D9),
and the regenerated `docs/api/` (`shul2_format.md`, `shul2_reader.md`, `blackbox_reader.md`, the index
pages). **Deliberately not touched:** the changelog block (its sentences are still true — "another major
is refused"; nothing in it named the Hello's `complete` or the reading cost); guide ch.11 ("Reading the
diagnostics") and the diagnostics plan (still true); guide ch.14 ("What it cannot do yet") :414-417 —
"the live telemetry wire to VexBuilder" is still unbuilt, and its rewrite is A2's, as the first round
logged.

Public docs = `docs/` minus `docs/api/` and `docs/internal/`, plus `README.md`. Each grep, its hit
count, and what the hits say (all read):
- `SHUL/2\|SHUL2\|Shul2\|shul2` → 58: the first round's 54 plus roadmap :2000 (D1's correction),
  hardware-assumptions :1464 and :1471 (D4) and vexbuilder §7 :732 (D6). All true.
- `shul2_format\|shul2_reader\|Shul2Reader\|crc32` → 16: HA-140 (4), changelog (6), guide ch.11 (3),
  vexbuilder (3). All true.
- `blackbox_format` → 6 and `blackbox_reader\|BlackboxReader` → 6 (D3 asked for these): roadmap :552-556
  and :1611-1615 (E1's description), changelog :965-967 (E1's entry), guide ch.11 :362 ("Reading a
  blackbox file… will not crash on a damaged file" — true, and silent on the flag, so nothing to add) and
  :489 (links), HA-140 :1440 ("`diag/blackbox_format.hpp` has no checksum" — still true of the FILE). None
  describes what raises `corrupt`, so none went stale with the wheelCount clamp.
- `corrupt()` → 0 (the spelling D3 removed from `blackbox_reader.hpp` appears in no public doc);
  `-i corrupt` → 29: vexbuilder :439 and :570 (the decode flag and the wheelCount clamp — true), :468,
  :470, :475 (a corrupted length), :541-542 (the signature); the rest are about pose, odometry or
  motors — unrelated.
- `HA-168\|HA-169` → 0 and `robot console` → 0 (D8). `HA-140` → 3 (the index row :255, the R3d Part 2
  note :1020, the entry :1437) — the entry is D4's; the index row's claim ("worth its 4 bytes…") is
  unchanged by sharing the function.
- `JSON` → 12: the first round's hits, none new. `F9` → 25: the first round's plus vexbuilder §7 :732
  and roadmap :2001, both about the freeze still to come — true.
- `-w wire` (the 3-wire-port and motor-wiring hits set aside, the rest read): the review round's own
  are HA-140 :1464, :1465 and :1471 (D4) and roadmap :2000 (D1) — true; every other telemetry-sense hit
  is a sentence the first round wrote or read (vexbuilder §1/§5, the master plan §18.1, the diagnostics
  plan, guide ch.11 :491-494, ch.14 :414-417 — A2's), still true.
- `wheelCount` → 5 (vexbuilder, changelog, roadmap :1105's IKinematics contract) — unchanged.
- `tail rule\|live reader\|live stream` → 2 and `per MB\|table-driven` → 2: all vexbuilder §5.2.2 (C3,
  C5) — the only public statements of either rule, and they match the headers.
- `heading error\|errorHeading` → 23: vexbuilder :439-443 (D5) and :564 (the Tick table); guide ch.11
  :89 and :112 (`drift`, `heading max … final` — absolute per-motion figures, consistent with D5's
  sentence), and the rest are about localization accuracy — unrelated.
- `re-Hello\|late join\|restart` → 4: vexbuilder :659 (T1's sentence) and :665 (D6's bullet) — true;
  the cookbook ch.02 :157 and HA :1107 — unrelated.
- `-i checksum\|bit flip` → 4: HA-140 :1440, guide ch.11 :493, roadmap :265 and :1206 (D9 (c)).
- C7 removability: `api_doc_tool check-removability` → no public doc references `docs/internal/`
  (below). Nothing here says "simulator": the only "simulated" in the chunk's text is "H2's simulated
  sensor frames" — VexBuilder's physics engine's sensors, the brief's own words (D4), not the host test
  plant.

### 04:20 — every gate on the final tree (nothing committed)
- **Host suite**, from source: the briefing regenerated, `cmake -S test -B build/test`,
  `cmake --build build/test --clean-first -j8` (03:52:25: 178 objects compiled, "Linking CXX executable
  shulib_tests", every doc gate green), the briefing regenerated again once the binary existed →
  `test cases: 1781 | 1781 passed | 0 failed | 3 skipped`, `assertions: 2671224 | 2671224 passed | 0
  failed`. A1's cases: 56 (`shul2_format_test.cpp` 26, `shul2_reader_test.cpp` 30), the first round's 47
  plus the review round's 9.
- `python3 tools/src_build_gate.py self-test` → `src build gate self-test: OK (189 detector cases, each
  against a real build)`; `check` → `src build gate: PASS (all 6 builds link; …)`, each build "exit 0,
  8/8 TUs compiled, warnings: 6 vendor-ignored, 0 disallowed", `[include-root]` and `[printed-path]`
  clean. Nothing in `src/` changed; this is the proof.
- ARM header gate (the CI command; its TU written to `review-fix/resume/all_headers.cpp`, never the
  repository root): `TU includes 195 headers` → `ARM compile gate: CLEAN (all v2 headers)`.
- CI greps → `library is PROS-free (all of include/shulib except hal/pros/)`; `layering holds: core is
  sim-free`. The three new headers include only `<…>`, `diag/`, `hal/telemetry_sink.hpp` and
  `shulib/version.hpp` (landmine 6).
- `api_doc_tool`: `self-test` OK; `check-coverage` `3115 public entities across 159 headers, all
  documented`; `check-fresh` exit 0 (`docs/api/` regenerated after the C5 prose edit — the headers' own
  top comments are rendered into their pages); `check-examples` `467 quoted lines, all verbatim (5 source
  files); 3 listings of whole-run files, each one unbroken run`; `check-removability` `no public doc
  references docs/internal/`.
- `doc_staleness_audit.py` `self-test` → `OK (10 detector/filter cases)`; audit → `clean (27 live
  docs, …)`. `briefing_status.py check` → `generated block current` (the block now reads 1,781 cases,
  195 headers, 14 honest partials, H1 among the open chunks).
- **State:** `git status` lists only this chunk's files (no `src/`, `Makefile`, `tools/`, CI or
  `test/CMakeLists.txt` change); the three headers any mutation touched are `cmp`-identical to the
  resumed round's backups. **Not committed** — the coordinator verifies and commits. A2 (the sink,
  HA-168/169) and B (the F9 freeze) are not started.

**For the coordinator (the review round):**
- **A recipe gap that is not H1's alone:** after ANY mutation run, the first normal build fails its own
  doc gate, because `briefing_status.py check` executes the binary on disk — still the last mutant —
  before the link step, so the build stops and never relinks. Relink with `--target shulib_tests/fast`
  first, then build normally (step 5 above). `HANDOFF-remaining-work.md`'s stale-binary recipe may want
  the sentence; it is a shared plan document, so it is left to you.
- **Shared documents this round touched** (each a self-contained edit): `vexbuilder-integration.md`
  (§5.2.1, §5.2.2, §5.2.7 — H1's own section — and one §7 row), `roadmap.md` :2000-2001 (a dated
  correction appended inside the G2-era "You are here" paragraph, in its own style; Part B's list still
  owns rewriting that paragraph), `hardware-assumptions.md` HA-140 (*Settle*, *Contained*),
  `build-order.md` H1 (this part's status line, above the restored 2026-09-26 line). `docs/api/`,
  `mkdocs.yml` and `PROJECT-BRIEFING.md` are generated: regenerate at the merge.
- The earlier "for the coordinator" notes (11:03) stand unchanged: `calibration_record.hpp` is shared with
  SAFE-SAVE Part A; the changelog block moves under "## API 2.4" if L9 lands first; the tests read the
  API bytes from `version.hpp`, so L9's bump is merge-safe.

### 09:56 — resumed a second time (`date` at resume: Thu Oct 01 09:48:04 EDT 2026)
The run that wrote the 03:37-04:20 entries did NOT finish: its transcript (the coordinator's session
store, read only) ends at 2026-09-30 04:22:35 EDT with its last tool call REJECTED ("Request interrupted
by user for tool use") — the call that was to re-run the doc gates — and no final report. So this run
re-established the state first, trusting none of it:
- **`git status`** lists only this chunk's files (20 modified, 10 untracked, as before);
  `git diff -- src/ Makefile tools/ .github/ test/CMakeLists.txt` is EMPTY: no gate self-test plant left
  behind, no `src/` change.
- **No mutation left applied.** `shul2_format.hpp`, `shul2_reader.hpp` and `blackbox_format.hpp` are
  `cmp`-identical to their newest backups (`scratch/h1-a1/review-fix/resume/backup/`, taken at each
  file's last mutation, 03:39-03:42 on 09-30); `blackbox_format.hpp` also to both older backups. The
  reviewers' drivers (`review-correctness/rmutate.py`, `review-testgate/mut.py`) target their own
  scratch copies (`tree-now/`, `tree/`), never this worktree — read, not assumed.
- **The mutation evidence matches the log.** `resume/results_033336.jsonl` (51 rows) and
  `resume/results_034210.jsonl` (N19b): 52 rows, every verdict RED, every `linked` true, every
  `missingIncluders` empty, every `restoredCmp` true — the 03:46 tables as written. The code and tests
  on disk now are the ones those 52 runs (and the 03:52 clean build) ran against: the workflow's
  transcripts list every Edit/Write any agent made in this worktree, and the interrupted run's last
  header edits were at 03:26-03:30 (C5's cost prose, comments only), before its mutation runs began at
  03:33:36; no agent has edited a test file since the killed run's 21:53-21:57 edits of 09-29 (the
  02:55:49 mtime is a touch); the headers' mtimes are their restore bumps (03:39-03:42). Nothing to
  re-run there.
- **Two edits the interrupted run made AFTER its gates and its last log write, never logged:**
  `vexbuilder-integration.md` §5.2.2's measurement note (:485-487), at 04:21:57 and 04:22:09 (the
  transcript's Edit calls; file mtime 04:22:09): "(measured 2026-09-30, idle and loaded)" became
  "(measured 2026-09-30; the spread is the machine's other load)", and "(shulib's own is bitwise, to
  save flash on the robot's brain)" became "(shulib's is bitwise, to save the brain's flash)". The new
  wording is the TRUE one — neither measurement was idle (load ~17 and 26-34, the 03:44 table) — so it
  is kept; the gates it never saw are re-run below.
- **A claim written before its evidence, corrected:** the `build-order.md` H1 status line (mtime
  2026-09-30 03:34:31) said "all 51 mutations were re-run on the final tree, each observed red" while
  that run was still going (03:33:36-03:39:45), and N19b (03:42) made it 52. Now: "all 52 mutations (the
  first round's 27 and the review round's 25)". (The line was never committed, so it is corrected in
  place rather than kept as written.) The 03:44 D7 entry's "(… 51 mutations re-run)" reads 52 by this note.
- **The test binary was not trusted** (mtime 2026-09-30 03:52:50, the interrupted run's clean build):
  rebuilt from source below.
- **Every finding stays dispositioned as logged at 03:44; none was re-opened.** Re-read against the
  tree, not the log: the code (`shul2_format.hpp`, `shul2_reader.hpp` in full), all 56 SHUL/2 cases
  (each assertion each finding asked for is present), and the documents each D finding names — every
  file:line the 03:44 entry cites holds what it says (the reader's "What reading costs" runs to :85, one
  line past the :79-84 cited). Two of its statements re-checked at their source: D5's "only writers" —
  `errorHeading` is assigned only at `move_to_pose.hpp:538` and `turn_to.hpp:227`, both fed by
  `Angle::errorTo` (`:214`, `:384`, `:584`; `:123`), which returns (−π, π] (`angle.hpp:62-66`); and D4's
  "both pin it" — the record's golden `0x45C46E98` is zlib's (`calibration_record_test.cpp:124-125`).
  C5's "three per ten bytes" model re-read (`resume/densest_karp.py`: no candidate at p + 1, p + 8 or
  p + 9 of another — the header's stated constraint, exactly), and its densest pattern's three lengths
  (4096, 3923, 3923) all pass the 4096 bound.

### 10:13 — every gate on the final tree (nothing committed)
- **Host suite**, from source: the briefing regenerated, `cmake -S test -B build/test`, then
  `cmake --build build/test --clean-first -j8` (09:57:44-09:58:11: 178 objects compiled, "Linking CXX
  executable shulib_tests", no warning; every doc gate green, the briefing's suite line left uncompared
  while no binary existed, as the tool says), the briefing regenerated again once the binary existed →
  `./build/test/shulib_tests`: `test cases: 1781 | 1781 passed | 0 failed | 3 skipped`, `assertions:
  2671224 | 2671224 passed | 0 failed` — the 09-30 counts. A1's 56 cases: format 26, reader 30.
- `python3 tools/src_build_gate.py self-test` → `src build gate self-test: OK (189 detector cases, each
  against a real build)` (09:59-10:06); `check` → `src build gate: PASS (all 6 builds link; …)`, each
  build "exit 0, 8/8 TUs compiled, warnings: 6 vendor-ignored, 0 disallowed", `[include-root]` and
  `[printed-path]` clean (10:08). No `src/` file changed; this is the proof.
- **A sequencing slip of this run, and what it cost:** the ARM gate, the CI greps and a first pass of the
  doc gates ran at 09:59-10:00, WHILE the src gate's self-test was running — and the self-test plants its
  faults in REAL files and restores them (`find -newermt` lists what the gate rewrote: `Makefile`,
  `test/CMakeLists.txt`, `test/vendor/doctest.h`, two host tests,
  `include/shulib/config/robot_config.hpp`, a vendored lvgl header and nine `src/` files). Those first
  passes may have read a planted file, so none of them is evidence. Every one was re-run after the src
  gate finished, on the quiet tree, and those runs are the ones below. (The restores are byte-exact:
  `git status` lists the same 30 entries before and after, and git compares contents.)
- **After the src gate finished** (10:10:45-10:12:23): `cmake --build build/test -j8` recompiled all
  178 objects (the self-test's restores bump the mtimes of widely included files) and linked, every doc
  gate green in it; `./build/test/shulib_tests` → `1781 | 1781 passed | 0 failed | 3 skipped`,
  `2671224 | 2671224 passed`. (No gate reads THIS log's text: the staleness audit's path sweep covers
  public docs and three planning documents — RESUMING, build-order, the briefing — "never the records",
  and the briefing only notes that this log exists. So the edits to this entry after 10:10 move no
  result; `build-order.md`'s edit, 09:56, came before every gate run.)
- ARM header gate, 10:13 (the CI command; its TU in `review-fix/resume2/all_headers.cpp`, never the
  repository root): `TU includes 195 headers` → `ARM compile gate: CLEAN (all v2 headers)`
  (arm-none-eabi-g++ 13.2.1).
- CI greps, 10:13 → `library is PROS-free (all of include/shulib except hal/pros/)`; `layering holds:
  core is sim-free`. The three new headers' includes are as logged (landmine 6).
- `api_doc_tool`, 10:11: `self-test` OK; `check-coverage` `3115 public entities across 159 headers, all
  documented`; `check-fresh` exit 0; `check-examples` `467 quoted lines, all verbatim (5 source files);
  3 listings of whole-run files, each one unbroken run`; `check-removability` `no public doc references
  docs/internal/`. And at 10:13 `generate` rewrote `docs/api/` and `mkdocs.yml` BYTE-IDENTICALLY (an md5
  of every file before and after), so the pages are the headers' own.
- `doc_staleness_audit.py`, 10:11: `self-test` → `OK (10 detector/filter cases)`; audit → `clean (27
  live docs, …)`. `briefing_status.py check` → `generated block current` (1,781 cases, 195 headers, 14
  honest partials, H1 among the open chunks).
- `git status` at 10:13: the same 30 entries (20 modified, 10 untracked), all this chunk's;
  `git diff -- src/ Makefile tools/ .github/ test/CMakeLists.txt` empty; the three mutated headers still
  `cmp`-identical to their last backups. `grep -rn "HA-16[89]\|robot console"` over every public file
  this chunk touches → nothing (HA-168/169 appear only in this log).

**For the coordinator (this resumption):**
- The interrupted run's final report never reached you; the 03:37-04:20 entries and this one are the
  record. Its dispositions stand as logged, with the two corrections at 09:56 (the 52 count; the unlogged
  §5.2.2 wording edits, kept). **Not committed.** A2 (the sink, HA-168/169) and B (the F9 freeze) are not
  started.
- **`shulib-v2` has moved since this worktree's base `9239538`:** TANK1 Part B (`a5bd83f`), COMP-t
  (`0f56301`), L9 Part A1 (`fa915b0`, API 2.4) and two plan commits. Read-only checks for the merge
  (`git diff 9239538 shulib-v2`): none of them touched `blackbox_format.hpp`, `debug_record.hpp`,
  `run_summary.hpp`, `session_info.hpp`, `telemetry_sink.hpp` or `calibration_record.hpp`, so the wire's
  layouts and the CRC forwarder merge as built (SAFE-SAVE Part A has not landed; the 11:03 note on
  `calibration_record.hpp` stands). `test/CMakeLists.txt` there still globs `*_test.cpp`, so both
  SHUL/2 test files are built. `version.hpp` there is 2.4: the tests read the Hello's API bytes from it
  (merge-safe), and the changelog block now belongs under "## API 2.4" as "### Added since 2.4
  (2026-09-29)", after L9's block. TANK1 Part B's ` xt` is result-line TEXT, so it rides Log frames —
  no format impact, as predicted. D5's sentence still holds there (the same two `errorHeading` writers,
  `move_to_pose.hpp:540` and `turn_to.hpp:227`). Documents edited on both sides: `changelog.md`;
  `hardware-assumptions.md` (mine: inside HA-140 only); `roadmap.md`; guide ch.11 (mine: one paragraph
  after "The exact signatures"); `diagnostics-plan.md` (theirs: the C5 row; mine: the "Answered at H1"
  paragraph and the H1 row); `vexbuilder-integration.md` (theirs: §4's `"drive"` row; mine: §0, §1,
  §5.2, §5.3, §7); `mkdocs.yml` (theirs: a guide chapter; mine: three API pages); `build-order.md` (the
  H1 entry is unchanged there, so this part's status line inserts cleanly). `docs/api/` and
  `PROJECT-BRIEFING.md` are generated: regenerate after the merge.

## The coordinator's merge — H1 Part A1 (2026-10-01, 10:15–10:40 EDT)

- **The run:** the build (2026-09-29), three reviews (24 findings, 21 distinct), and the fix round — cut by the
  account's usage limits twice and resumed with continuation notes; at the first resumption its test binary was
  the M26 mutant's link (two golden cases red), relinked from source by the coordinator (1,781 green) before the
  fix resumed. All 21 fixed; 52 mutations red. Transport commit `caf1bc4` in the worktree `h1-a1`.
- **The merge** onto `de2a4f8`: one hand-resolved conflict — the changelog (this part's section moved under
  `## API 2.4` and re-headed "Added since 2.4 (2026-10-01)": L9's 2.4 landed first); `docs/api` and the briefing
  regenerated; the rest auto-merged and read (vexbuilder §5.2, the roadmap's M6 box, guide 11, HA-140's Source,
  `version.hpp` in a different hunk from L9's). build-order (the H1 status line, the Next line) and
  REMAINING-WORK (row 30, the totals) brought to the merged state.
- **Gates on the merged tree:** suite **1,892 | 1,892**, 3 skipped; `src_build_gate.py` self-test OK (198) and
  check PASS on all seven builds; the ARM header gate over all 196 headers; both CI greps; `check-fresh`, the
  briefing (self-test and check), removability and the staleness audit — all clean; no plant
  (`.claude/coordinator/logs/h1_merge_verify.log`).
- **Six coordinator mutations, each on a fresh binary, all RED** (`.claude/coordinator/muts_h1a1.json`): a frame
  whose CRC does not match accepted (6 cases); a sequence exactly 2³¹ behind not stale (1); another major's Hello
  read past its version bytes (2); a `wheelCount` above 8 decoded as given (1); the CRC without its final
  inversion (7); a truncated tail never reported (3).
- **Not done:** Part A2 (the writer, HA-168/169) and Part B (the F9 freeze). Nothing of H1 has crossed a real link.
