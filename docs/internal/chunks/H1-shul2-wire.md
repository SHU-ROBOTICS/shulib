# H1 — the SHUL/2 telemetry wire: `DebugRecord` streamed, sequenced, checked and versioned (freezes F9)

> **THE TEAM LEAD'S ANSWERS (2026-09-26, ~23:10): every recommendation accepted** — "Accept all" on
> D0–D9 as written below. Builders start after tonight's claw and auton merges (his ruling of ~14:15:
> "briefs now, build after tonight"); each part still starts on its own go, at its turn in the order
> the brief gives. Committed by the coordinator with its plan entries.

> **Execution brief.** Written 2026-09-26 from a read-only pass over `shulib-v2` at `a01e3a2` (the diag
> headers, the blackbox format and reader, the calibration record's CRC, the telemetry and block seams,
> the fakes, and every public doc that names SHUL/2). **Re-checked the same evening** against the same
> tree and a checker's 21 findings; the dispositions follow this header. **STATUS: WAITS FOR THE TEAM
> LEAD'S GO**, and decisions D0–D9 below need his answer first. **Three parts if D9 is ruled as
> recommended, each with its own executor run, verification and commit:** Part A1 builds the read side
> (the CRC move, the format, the reader; host-only), Part A2 builds the write side (the sink; host-only),
> and Part B freezes F9. B runs only after A1 and A2 are verified and committed. **Executor model:
> `opus`. Do not commit.** Live log: `docs/internal/chunks/H1-PROGRESS.md`, created FIRST and appended
> as you go. Reserved Hardware Assumptions numbers: **HA-168 and HA-169 only**. If this runs beside the
> work listed below, build it in an isolated worktree off the then-current `shulib-v2` HEAD. **Honest
> size:** A1 about 1–1.5 days, A2 about 1 day, B about half a day. The handoff's "~1 day" counted
> neither the stream reader nor the sink's own tests.

## Re-check 2026-09-26 (evening) — amendments

Each checker finding (`findings-H1.json`, in its order) was re-opened at its cited file and line.

1. **Real.** `kVersionMinor >= 0` on a `u16` fails the host AND the ARM build (`-Werror=type-limits`,
   compiled with the project's flags), and repeats the vacuous pin F1 removed. Part B pins no minor (item 8).
2. **Real.** The CRC covers the signature, so a mangling transport fails the check first. Signature
   kept, its rationale corrected; the reader gets a named capture status; HA-168 and a transform test fixed.
3. **Real.** The whole-capture tail rule is specified (a `NeedMore` at the end resyncs one byte;
   `truncated` only if nothing verifies after it), with a named tail test and named flips (§2).
4. **Real.** Sink tests now decode the sink's own Hello, Log and End and check every field; the
   re-Hello cadence, `summarize()` and the open/close lifecycle are stated; one mutation per field.
5. **Real** (`session_info.hpp` ~:42-44). The sink encodes the Hello ONCE at `open()` and re-sends
   those bytes; a stack-buffer test and a landmine pin it.
6. **Real.** The Tick also carries `MotionState` (0–5), the `Localizer::Quality` mirror (0–3) and the
   tick flag bits. Added to D6, the Part B pin, the register row and the contract tables.
7. **Real.** Blackbox's decoders take EXACTLY their width, so "older minor accepted" has no decode path
   once the tick grows. Put to the team lead as **D8**, with a permanent test as the recommendation.
8. **Real.** D4 now states what every major keeps: the envelope, the Hello's type and its payload bytes
   0–11. Pinned in Part B.
9. **Real.** D7 now sets out both precedents (no bump since 2.2; G2's 2.1 → 2.2 bump that locked F8)
   and why F9 differs. Recommendation unchanged; Part B names F9 in `version.hpp`.
10. **Real.** Link loss is per session, saturating, "unknown" for a late join or a missing End; the
    sink never sends a re-Hello at a wrapped sequence 0.
11. **Real.** Nonzero flags, reserved bytes and Hello roles 1 and 7 are tested with the CRC
    recomputed, with mutations; the reader's role rule is stated (accept, report, deliver by type).
12. **Real.** The width table gains the Hello (256); a short Hello is `LayoutMismatch`;
    `logFixedBytes` is defined as every Log byte that is not text.
13. **Real.** The three `math::Angle` fields wrap and refuse non-finite values; the NaN/±Inf/−0.0
    cases cover the other f64 fields only, compared by bit pattern.
14. **Real.** Landmine 6 allows `shulib/version.hpp` and `core/check.hpp`; landmine 7 covers
    `int` → `u8` for the API bytes.
15. **Real.** The doc gate counts a namespace-scope `using` and labels it "type alias". The alias is
    a documented forwarding function; the new CRC test targets `diag::crc32Ieee`; two doc lines added.
16. **Real.** The WIRE-STABLE ranges are `debug_record.hpp`'s (re-attributed in §0); Part B's sweep
    gains the listed lines plus a grep; `telemetry_sink.hpp` is "verify; likely unchanged".
17. **Real.** The shared list now names guide ch.11/ch.14, `vexbuilder-integration.md`, the
    `docs/api` index files, `PROJECT-BRIEFING.md`, `calibration_record.hpp` (SAFE-SAVE) and L7's CRC use.
18. **Real.** HA numbers are reserved (coordinator notes), not renumbered; HA-169 gets HA-168's
    settle; the rate comparison is restated in bytes per second.
19. **Real.** 64 KiB holds 151 tick frames after the header, so the end-to-end run sizes `SdSink`'s
    buffer to the run and asserts no drops; `LossyBlockSink` models a refusal AND a partial prefix.
20. **Real.** D0 now cites the 2026-09-23 ruling on VexBuilder-only work; D6 cites the register's
    "nothing freezes on one consumer's evidence" (F12's row).
21. **Real.** The §5.2 rewrite carries the Tick and Summary offset tables and every value table;
    §5.3 ("markers") joins A1's doc list.

Found in this re-check (not in the checker's list):

22. **The Log's time base was wrong.** `DebugRecord::t` is the scheduler's RAW clock reading
    (`motion_scheduler.hpp` ~:1179) and TermSink stamps lines raw (`term_sink.hpp` ~:92); an
    epoch-relative Log `t` would sit `epochSeconds` away from the ticks. Every wire time is now raw.
23. **Hello and End must travel in both directions**, but D4 put them in the runtime → peer range.
    The ranges now govern data frames only, and one capture carries one writer's bytes.
24. **The truncation test's offset 0 is not a cut** (the capture ends on a frame boundary). The loop
    runs over offsets 1 … 14+n−1; offset 0 asserts `!truncated && !sawEnd`.
25. **A live reader stalls on a corrupted length** until 14 + 4096 bytes arrive (about nine tick
    frames, ~90 ms at 100 Hz), then recovers them. The contract says so and sizes H2's buffer.
26. **Unknown Log level codes** decode as their raw number and map to "unknown", never by cast.
27. **Pace rule 7.** Part A's doc list reached eight hand-edited documents before these amendments
    grew it further. Put to the team lead as **D9** (split Part A into A1 and A2).
28. **`ByteReader::text` skips an interior NUL** and joins the text around it. Not H1's (its Hello
    decoder ends a field at the first NUL); listed under "Found while reading".

## Decisions for the team lead (recommendation given for each; nothing below decides them silently)

- **D0 — Timing.** H1 is L10 ("lower priority for the season", `HANDOFF-remaining-work.md` ~:460),
  and nothing reads the wire until H2, which waits on VexBuilder's planned Rapier physics engine. That
  engine is not being built (the team lead, 2026-09-23).
  - The same day he ruled that "work that exists only to serve VexBuilder's files (G3, G4) waits until
    after L1–L9 unless he says otherwise" (`HANDOFF-remaining-work.md` ~:165-168). L7 and L9-rest are
    still open, and the coordinator's plan starts builders after tonight's claw and auton merges.
  - On 2026-09-26 he said "I genuinely want to complete the full library asap" (`build-order.md`
    ~:1543-1544, the G3a entry).
  - **Recommendation:** run it in this wave, reading the 2026-09-26 request as the "unless he says
    otherwise". But it is the one next-wave chunk no program consumes, so it goes LAST when builder
    slots are short. It touches no `src/` file and no header another chunk has in flight.
- **D1 — Binary, not JSON.** Two documents disagree. The master plan §18.1 (~:828-830) and
  `term_sink.hpp` ~:5 call SHUL/2 "the JSON wire". The contract VexBuilder was given,
  `vexbuilder-integration.md` §5.2 (~:432), says "SHUL/2 should follow this [blackbox] shape, not
  invent a second one". **Recommendation: binary**, little-endian, fixed-width fields, IEEE-754
  binary64 with no narrowing (the blackbox's rules, `blackbox_format.hpp` ~:30-38). JSON cannot
  carry NaN or Inf. A double only round-trips through decimal text with 17 significant digits and a
  correct parser on both sides. JSON would also bring heap and formatting cost onto the brain.
  Part A1 fixes both JSON mentions.
- **D2 — The Tick and Summary payloads ARE blackbox v1's bytes.** Recommendation: SHUL/2 calls
  `blackbox::encodeTick` / `encodeSummary` and their decoders (`blackbox_format.hpp` ~:440-493,
  ~:592-654) and writes no second encoder.
  - Why: one field order, by construction. A blackbox Tick frame from the SD card can then be
    re-wrapped as a SHUL/2 frame without decoding it, which is the premise of H2's DoD ("a real
    on-robot run replays in VexBuilder").
  - The cost: F9 then pins those bytes. A future change to the blackbox layout must fork its own
    encoder, and the Part A1 golden fails if it does not. And a minor that GROWS the tick must give
    the tick decoder an older-minor path (D8).
  - Rejected: a separate encoder in the same field order. Two encoders that must agree are E1's
    mutation-1b blindness (`build-order.md` ~:376-383).
- **D3 — Every frame carries a check.** Recommendation: two sync bytes, a u32 sequence number, a u16
  length and a CRC-32/IEEE on every frame, plus an 8-byte signature in the Hello (layouts in §1).
  - The overhead is 14 bytes per frame (about 3 % on a tick). The CPU cost on the brain is HA-169.
  - The check is verified before anything inside the frame is read, the Hello's signature included.
    §1 item 2 says what the signature then does, and what it cannot do.
  - Rejected, no check (the blackbox's choice): `blackbox_reader.hpp` ~:47-52 says a wire over a
    lossy link "must add its own frame check", and after the freeze a check could only arrive with a
    major version bump.
  - Rejected, COBS byte-stuffing: PROS already COBS-frames stdout (HA-129), a local socket does not
    need it, and sync + length + CRC resynchronises within a bounded number of bytes.
- **D4 — How versions evolve (the DoD's "negotiation").** Recommendation:
  - A different MAJOR version is refused whole.
  - **What every major keeps.** The envelope (sync bytes, field offsets and widths, the 4096 bound,
    the CRC and what it covers), the Hello's type value (1), and the Hello's payload bytes 0–11
    (signature, versionMajor, versionMinor) are the same in EVERY major, the way PNG keeps its
    signature. Without this a 1.x reader could not even name a 2.0 stream as refused; it would see
    only skipped bytes. Everything else may change at a major, including what the flag bits mean.
  - A NEWER minor is ACCEPTED. The reader decodes the prefix of each payload that it knows, and
    ignores and counts the appended bytes.
  - An OLDER minor is accepted. Its declared widths must equal THAT minor's row in the reader's
    table, and the fields it lacks take their documented defaults. How that stays decodable when the
    tick grows is D8.
  - Unknown frame types are skipped by their length and counted. Unknown flag bits and reserved
    bytes are ignored, so a later minor may give them only a meaning that is safe to ignore.
  - Frame types: Hello (1) and End (5) are SESSION frames that open and close a session in either
    direction (a peer's Hello carries role 1). The ranges govern DATA frames: 2–63 runtime → peer,
    64–127 peer → runtime (H2's simulated sensor frames), 128–255 reserved.
  - One capture carries ONE writer's bytes. A recorder that keeps both directions of H2's link keeps
    two captures; two writers interleaved into one byte stream break the sequence rules.
  - This deliberately DIFFERS from the blackbox, which refuses any other width
    (`blackbox_reader.hpp` ~:129-136). §6 of the integration contract promises "unknown newer fields
    are ignored, never fatal", and the DoD requires an unknown future version to be handled. The
    blackbox's width cross-check survives where it belongs: at the SAME minor, a width mismatch is
    refused, which catches a changed layout whose version was never bumped.
  - Rejected: the blackbox's refuse-anything-else rule.
- **D5 — What v1 carries.** Recommendation: five frame types — Hello, Tick, Log (the message channel,
  which the blackbox omits only because it is fixed-width), Summary (`RunSummary`) and End.
  - v1 does NOT carry per-motion results or `MotionOutcome` as data. No seam channel carries them;
    they ride `log()` as text (`term_sink.hpp` ~:22-25), so they reach the wire inside Log frames.
    The same holds for G2's markers (`vexbuilder-integration.md` §5.3 promises "markers"): in v1
    they arrive as the runner's Log lines. A data frame for either is an additive type later.
  - No inbound frames. Those are H2's, in the reserved peer range.
  - No Triage frame. A stream carries every tick, including the tick that faulted.
  - This answers the question `diagnostics-plan.md` ~:48-51 left to H1.
- **D6 — When F9 freezes, and what it freezes.** Recommendation: Part B right after A1 and A2, with
  ONE in-tree consumer (the reader). F8 froze the same way at G2.
  - The counter-evidence, stated: the register's own principle is that "nothing freezes on one
    consumer's evidence" (`roadmap.md` F12's row, ~:146), and this consumer is written by the same
    executor in the same chunk. A checker found nine major defects in this brief before a line was
    written; catching that kind of defect is exactly what a second consumer is for.
  - The alternative is to freeze at H2, once VexBuilder's reader has stressed the wire: the
    build → second consumer → freeze path of F6, F10 and F11. Recommended against because that
    consumer is not being built, so waiting could mean waiting indefinitely. D4's rules and D8's
    permanent test make the freeze cheap to live with: every frame type can grow by appending.
  - **F9 would freeze** the BYTES and the RULES: the envelope and D4's cross-major invariant; the
    frame types and ranges; the 1.0 Hello, Tick, Log, Summary and End layouts; the CRC and its
    coverage; the time base (every wire time is the clock's raw reading); the session, sequence and
    version rules (D4, D8); one writer per capture; the wire level codes; and, through the tick,
    `DebugRecord`'s field order and the values of `FaultCode`, `GateReason`, `TickPhase`,
    `motion::MotionState`, the `qualityClass` mirror of `Localizer::Quality`, and the tick flag bits.
  - **F9 would NOT freeze** the C++ classes (`Shul2Sink`, `Shul2Reader`, `scanFrame`'s C++ shape,
    the decode helpers, the reader's counter names), the writer policies (re-Hello interval, text
    caps, what `open()` does on an open session) or the transport (H2's). Part B says so in the
    register.
- **D7 — No API version bump in H1.** Two precedents point different ways.
  - Since 2.2, every addition outside a frozen surface landed with no bump: DIAG1, COMP-a, T3 Part 1
    and others (`changelog.md` ~:19-436, "No version bump — every change is ADDITIVE: nothing frozen
    changed shape").
  - The only data freeze so far came WITH a bump: `version.hpp` ~:59-69 records 2.2 as "chunk G2's
    additive growth (2026-09-18), the change that LOCKED register row F8", and
    `f8_signature_pin_test.cpp` ~:489 asserts `kApiMinor >= 2` for that reason. D6 cites F8 as its
    model.
  - **Recommendation: no bump.** F8 froze C++ SIGNATURES (`CommandRegistry`, `PathRunner::run`,
    `writeManifest`, each pinned by a `static_cast`), so the API version had to name them. F9 freezes
    BYTES that carry their own version in every Hello, which is what `version.hpp` ~:38-41 says the
    data freezes do, and D6 freezes no H1 class. H1's headers are additions outside any frozen
    surface, the practice since 2.2. So that the version mechanism still names the freeze, Part B
    records F9's lock in `version.hpp`'s data-freeze paragraph (~:38-41).
  - If you rule for a bump instead: 2.3 in Part B, with `version.hpp`'s `kApiMinor` note,
    `kApiVersionString` and a changelog heading.
- **D8 — How "an older minor is accepted" stays true when the tick grows.** `blackbox::decodeTick`
  and `decodeSummary` accept EXACTLY their width (`blackbox_format.hpp` ~:513, ~:625). The day a minor
  appends a `DebugRecord` field, a reader built on them cannot decode an older stream's 428-byte ticks,
  and D4's older-minor rule fails with nothing to say so.
  - **Recommendation:** keep D2, freeze D4's rule, and enforce it NOW with a permanent test in Part B.
    A hand-derived 1.0 capture, checked in as bytes, must decode under THIS build's reader with the
    verdict `Exact` or `OlderMinor`, every 1.0 field bit-identical and every field the 1.0 layout
    lacks at its documented default. It passes trivially at 1.0, and goes red, naming F9, the day a
    minor grows the tick without an older-minor decode path. D2's cost says so.
  - Rejected, a prefix-tolerant decoder built now: there is no longer layout to test it against, so
    its only test would be against a layout invented for the test.
  - Rejected, dropping the rule from the freeze: the DoD's "handles an unknown future version" would
    then hold in one direction only, and §6 of the integration contract promises both ("missing
    fields get safe defaults").
- **D9 — Split Part A in two (pace rule 7).** Part A's doc list ran to eight hand-edited documents
  (rule 7: past about five, split into parts), and this re-check added the reader's tail and status
  rules, the sink's lifecycle and about fifteen mutations.
  - **Recommendation: split.** Part A1 is the read side (§1 items 1–5: the CRC move, the format, the
    reader, the `wheelCount` fix). It is everything VexBuilder needs to read a stream, and its tests
    build captures with the format's own encoders. Part A2 is the write side (items 6–7: the sink,
    its tests, the end-to-end run, HA-168 and HA-169). Each has its own run, verification and commit,
    and B follows both.
  - If you rule for one Part A, the executor runs A1 and A2 as one part with the union of their lists.

## Do not touch — work in flight

- **T3 Part 2** (worktree `t3-part2`): `src/teleop_program.*`, `src/boot_source.*`,
  `include/shulib/teleop/*`, `calibration/boot_check.hpp`, and the comments in
  `src/mechanism_table.hpp`. Its doc edits include guide ch.07, ch.10, ch.11 and ch.14.
- **G1 Part 2c's final run** (worktree `g1-2c-final`): `src/main.cpp`, the new one-chain header
  `src/build_selection.hpp`, the robot selection in `src/chassis_table.hpp` /
  `src/mechanism_table.hpp`, `tools/src_build_gate.py`, the `Makefile`, and on its branch the comment
  in `diag/session_info.hpp` and `vexbuilder-integration.md` §1 ~:64.
- **COMP-b** (next): the competition program in `src/`, `main.cpp`'s chain, the worksheet's Station K,
  and a new guide chapter, which may renumber the chapters after it.
- **Next-wave siblings** (briefs only today): SAFE-SAVE edits `calibration_record.hpp` (~:52-54 and a
  new constant), and L7 adds a CRC-bound record that calls `crc32Ieee` (its draft ~:21-22), so the
  calibration spelling must keep working.

So: **no `src/` file, no `tools/src_build_gate.py`, no worksheet, and no edit to
`diag/session_info.hpp`** (H1 reads it only). Nothing attaches a `Shul2Sink` to a program.
`calibration_record.hpp` is edited only at the CRC's body (~:106-121) and the comment that says where
it lives (~:28-39).

These documents are shared with the chunks above: `changelog.md`, `hardware-assumptions.md`,
`roadmap.md`, `build-order.md`, `vexbuilder-integration.md`, guide ch.11 ("Reading the diagnostics")
and ch.14 ("What it cannot do yet"), the `docs/api` index files (`README.md`, `all-entities.md`) and
`PROJECT-BRIEFING.md`. Write H1's changes to them as self-contained blocks (a new `###` entry, a new
subsection, one appended status sentence, one table row) so the coordinator can merge them. Do not
rewrite anyone else's paragraph. The coordinator regenerates `docs/api` and the briefing at merge.
Name guide chapters by title as well as number in the PROGRESS log, because COMP-b may renumber them.

## 0. Why

Register row F9 (`roadmap.md` ~:143) and M6's first box (~:1782) have been open since the plan was
written, and the whole diag tree has been built around the assumption that they will close:

- `debug_record.hpp` ~:10-17: "F9 (frozen at chunk H1) is the wire serialization of EXACTLY this
  record".
- `fault.hpp` ~:14-18 makes the same promise for `FaultCode`, and `debug_record.hpp`'s `GateReason` /
  `TickPhase` "WIRE-STABLE" notes (~:46-47, ~:97-100) make it for those two enumerations.
  `motion.hpp` ~:144-146 makes it for `MotionState`, the vocabulary of `activeCommandState`.
- C5 reserved the drop counters and two spare tick-phase slots "BEFORE the F9 freeze"
  (`debug_record.hpp` ~:118-121, ~:191-212).
- E1 wrote down exactly what H1 should inherit (`blackbox_format.hpp` ~:48-53) and the one
  guarantee it must NOT inherit (`blackbox_reader.hpp` ~:32-52: there is no per-frame check).

So the schema has been treated as frozen for eight weeks, but no wire exists. `vexbuilder-integration.md`
§0/§1/§7 (~:35, ~:69, ~:479) tells VexBuilder that "F9 is not written". The build-order entry
(~:1602-1608) sets the DoD: **round-trip serialization verified; version negotiation handles an
unknown future version; F9 frozen.**

## 1. Scope — IN

### Part A1 — the read side (host-only)

1. **`include/shulib/diag/crc32.hpp`** (NEW, a leaf: `<cstddef>`, `<cstdint>`, `<span>` only).
   - Move `crc32Ieee` there VERBATIM from `calibration/calibration_record.hpp` ~:106-121, with its doc
     comment, as `diag::crc32Ieee`. It is bitwise and uses no table; HA-140's reasoning carries over,
     and HA-169 restates it at the stream's rate.
   - `calibration_record.hpp` keeps the spelling `calibration::crc32Ieee` as a one-line documented
     FORWARDING function (`return diag::crc32Ieee(bytes);`). So `test/calibration_record_test.cpp`
     ~:38, the record's encoder and decoder (~:192, ~:256) and L7's draft compile unchanged. Its `///`
     line says where the algorithm now lives.
   - Rejected, a namespace-scope `using diag::crc32Ieee;`: the doc gate counts it as a declaration
     (`tools/api_doc_tool.py` ~:942-960) and renders its kind as "type alias" (~:742-744), which
     would publish a false kind for a function.
   - Rejected, including `calibration_record.hpp` from `diag/`: that inverts the layering
     (calibration already includes `diag/blackbox_format.hpp`, ~:64).
   - Rejected, a second copy of the CRC.
2. **`include/shulib/diag/shul2_format.hpp`** (NEW), in namespace `shulib::diag::shul2`. Every
   field is little-endian and written byte by byte through `blackbox::ByteWriter` / `ByteReader`
   (~:174-349). Never a struct memcpy.
   - **The envelope**, 14 bytes of overhead per frame. It is the same in EVERY major version (D4):

     | Offset | Field | Notes |
     |---|---|---|
     | 0 | `u8` sync0 = 0xFE | 0xFE never occurs in valid UTF-8 text |
     | 1 | `u8` sync1 = 0x53 | |
     | 2 | `u8` type | |
     | 3 | `u8` flags | 0 in major 1; readers IGNORE unknown bits, so a later minor may give a bit only a meaning that is safe to ignore |
     | 4 | `u32` sequence | |
     | 8 | `u16` payloadBytes | at most `kMaxPayloadBytes` = 4096 |
     | 10 | payload | |
     | 10+n | `u32` CRC-32/IEEE | `diag::crc32Ieee` (zlib-compatible) over bytes [2, 10+n): type through the end of the payload |

   - **Frame types:** `Hello = 1`, `Tick = 2`, `Log = 3`, `Summary = 4`, `End = 5`. 0 is never
     assigned. Hello and End are session frames used in either direction; data frames follow D4's
     ranges. A verified frame of any type this build does not assign (0 included) is skipped by its
     length and counted.
   - **One capture, one writer.** A capture, or one direction of a live link, carries ONE writer's
     bytes (D4).
   - **The time base.** Every time on the wire (the Tick's `t`, the Log's `t`, the End's `endTime`)
     is the injected clock's RAW reading in seconds. That is the base the scheduler stamps
     `DebugRecord::t` in (`motion_scheduler.hpp` ~:1179) and TermSink stamps lines in
     (`term_sink.hpp` ~:92). The Hello's `epochSeconds` is the same clock's reading at `open()`; a
     reader subtracts it for session-relative time. The blackbox does the same (header
     `epochSeconds`, raw end time).
   - **The Hello** (v1.0, 256 bytes):

     | Offset | Field |
     |---|---|
     | 0 | `u8[8]` signature `89 53 48 55 4C 00 0D 0A` (0x89 "SHUL" NUL CR LF) |
     | 8 | `u16` versionMajor = 1 |
     | 10 | `u16` versionMinor = 0 |
     | 12 | `u8` role (0 = the runtime that emits records, 1 = a peer; other values reserved) |
     | 13 | `u8` reserved |
     | 14 | `u16` flags = 0 |
     | 16 | `u16` tickPayloadBytes = 428 |
     | 18 | `u16` summaryPayloadBytes = 168 |
     | 20 | `u16` endPayloadBytes = 32 |
     | 22 | `u16` logFixedBytes = 14 |
     | 24 | `f64` epochSeconds (the clock's raw reading at `open()`) |
     | 32 | `u8` apiMajor (`shulib::kApiMajor`) |
     | 33 | `u8` apiMinor (`shulib::kApiMinor`) |
     | 34 | `u16` reserved |
     | 36 | `u8[48]` buildHash |
     | 84 | `u8[32]` routineId |
     | 116 | `u8[16]` alliance |
     | 132 | `u8[16]` side |
     | 148 | `u8[96]` portMap |
     | 244 | `u8[12]` reserved (zero) |

     - The type (1) and payload bytes 0–11 are the same in every major (D4). Bytes 12 onward belong
       to their major.
     - The text fields are NUL-padded to their widths (`ByteWriter::text`), and a decoded field ends
       at its first NUL. The widths are the blackbox header's (~:352-360). An EMPTY build hash stays
       empty: MISSING must stay loud, per `session_info.hpp`.
     - `apiMajor` / `apiMinor` come from `version.hpp` (~:48, ~:75, both `int`) through a checked
       cast.
     - Reserved bytes are written 0 and never judged by a reader, by the same rule as flag bits.
     - **What the signature does, and what it cannot do.** Its high-bit byte, NUL and CR LF guarantee
       that a transport which strips bit 7, truncates at a NUL, or translates newlines (in either
       direction) corrupts EVERY Hello. That matters most for newline translation, the one transform
       that can leave a Tick intact (a Tick holding no 0x0A or 0x0D byte), so no session ever starts
       on such a link. But the per-frame CRC catches the corruption BEFORE the signature is compared,
       so the reader reports such a capture as `NoFrame` or `NoSession` (item 3), never as
       `BadSignature`. `BadSignature` means a Hello whose CRC verifies and whose signature is wrong: a
       writer bug, or a stream that is not SHUL/2 inside a SHUL-shaped envelope.
   - **The Tick** is `blackbox::encodeTick`'s 428 bytes (D2), laid out at `blackbox_format.hpp`
     ~:424-435. Units are canonical (`quantity.hpp` ~:14, ~:134-141): inches, radians, seconds,
     volts, amperes, in/s and rad/s. Headings are wrapped radians. The small vocabularies travel as
     their numbers: `fault` (u16 `FaultCode`), `gateReason` (u8 `GateReason`), `activeCommandState`
     (u8 `motion::MotionState`), `qualityClass` (u8, the `Localizer::Quality` mirror 0–3), and the
     flags byte (bit 0 `clampedThisTick`, bit 1 `strafeFallbackActive`, bits 2–7 written 0 and
     ignored). A value this build does not name decodes as its number and is never refused.
   - **The Summary** is `blackbox::encodeSummary`'s 168 bytes (`blackbox_format.hpp` ~:581-586). The
     encoder's `blackboxDropped` argument is the summary's OWN `blackboxDropped` field
     (`run_summary.hpp` ~:80). The wire never overwrites another sink's figure; the wire's own drops
     ride the End frame.
   - **The Log** (14 bytes before the text):

     | Offset | Field |
     |---|---|
     | 0 | `u8` level (wire code) |
     | 1 | `u8` flags (bit 0: tag truncated, bit 1: message truncated; bits 2–7 ignored) |
     | 2 | `u8` tagBytes |
     | 3 | `u8` reserved |
     | 4 | `u16` messageBytes |
     | 6 | `f64` t (the clock's raw reading when `log()` was called) |
     | 14 | tag, then message |

     - The text always starts at offset 14. Fields that later minors append go AFTER the text.
     - `logFixedBytes` means every Log byte that is not text: `payloadBytes − tagBytes −
       messageBytes`. It is 14 in 1.0; a minor that appends k bytes after the text declares 14 + k.
     - The text is carried VERBATIM, never sanitized: the length prefix is the framing, and
       sanitizing is a display concern.
     - The wire level codes are Error 0, Warn 1, Info 2, Debug 3, Trace 4, mapped by a `switch` in
       both directions and never by a cast, because `hal::LogLevel` (`telemetry_sink.hpp` ~:64-84)
       declares no explicit values. An unknown code decodes as its raw number, and the mapping to
       `hal::LogLevel` reports "unknown" (returns false).
   - **The End** (32 bytes):

     | Offset | Field |
     |---|---|
     | 0 | `u32` framesOffered (the End frame's own sequence number) |
     | 4 | `u32` framesFailed (the device refused them) |
     | 8 | `u32` recordsSeen |
     | 12 | `u32` linesSeen |
     | 16 | `u32` linesTruncated |
     | 20 | `u32` notOpenDropped |
     | 24 | `f64` endTime (the clock's raw reading at `close()`) |

   - **A per-minor width table**, one row per minor and append-only, whose row 1.0 is {hello 256,
     tick 428, summary 168, end 32, logFixed 14}. Add
     `static_assert(blackbox::kTickPayloadBytes == table[kVersionMinor].tick)` and the same for the
     summary, so that growing `encodeTick` without bumping the wire minor fails the build.
   - **`negotiate(const Hello& stream, const ReaderProfile& reader)`**: a pure function that returns
     exactly one verdict, checked in this order. (The decoded `Hello` carries its payload length, and
     `decodeHello` decodes bytes 0–11 from any payload of 12 bytes or more, so a short Hello can still
     be judged.)
     - `BadSignature`: a Hello payload shorter than 12 bytes, or a signature that differs.
     - `RefusedMajor`: a versionMajor other than the reader's.
     - `LayoutMismatch`: a Hello payload shorter than 256; OR, at the SAME minor, any declared width
       that differs from the reader's row; OR, at an OLDER minor, any declared width that differs
       from THAT minor's row in the reader's table; OR, at a NEWER minor, any declared width shorter
       than the reader's newest row.
     - `OlderMinor`, `NewerMinor` or `Exact`.

     It takes the reader's profile as a parameter, so tests can instantiate a hypothetical 1.1
     reader with its own 1.1 row. `kThisReader` is this build's profile.
   - **`scanFrame(std::span<const std::byte> buffered)`**: pure, and returns exactly one of:
     - `NeedMore` (consumed 0): the bytes are a proper prefix of a frame that could still be valid.
       That includes the empty span, a lone 0xFE, and a valid header whose frame is longer than the
       bytes held.
     - `Frame{consumed = 14 + n, type, flags, seq, payload view}`: the sync matched, n ≤ 4096, and
       the CRC verified.
     - `Skip{consumed, reason}`: no sync at offset 0 (consumed = up to the next 0xFE, or everything),
       a length above 4096 (consumed 1), or a CRC failure (consumed 1).

     **After a failed candidate, scanning always resumes at the NEXT byte.** It never skips the
     claimed length. The cost is bounded: at most 4104 bytes of CRC per candidate position.
   - **Encoders** for the envelope and for every payload this header owns (`encodeFrame`,
     `encodeHello`, `encodeLog`, `encodeEnd`), each whole-or-nothing like blackbox's (~:365-389). The
     sink (A2) composes them, and A1's tests build captures with them.
   - **Decoders** `decodeHello`, `decodeLog` (views into the caller's bytes; text is never assumed
     NUL-terminated) and `decodeEnd`. Each takes a payload of AT LEAST its known width and decodes
     that prefix. The one exception is `decodeHello`'s invariant part: it decodes bytes 0–11 from
     any payload of 12 bytes or more, and the rest only from 256 or more. Ticks and summaries decode
     through blackbox's decoders on `payload.first(knownWidth)`.
   - **A UTF-8-safe prefix helper for the Log's caps.** It uses the same continuation-byte back-off
     as `line_format.hpp` ~:67-75, without the sanitizing.
3. **`include/shulib/diag/shul2_reader.hpp`** (NEW): `Shul2Reader` over a WHOLE capture span. It
   borrows the span, allocates nothing, never throws, and is built on `scanFrame` + `negotiate`. It
   mirrors `BlackboxReader` (~:104-232) EXCEPT at the tail, where the blackbox's "stop and mark
   truncated" (~:169-172) would be wrong once there is a check.
   - **Delivery.** It delivers verified frames `{type, seq, payload}` of the five known types, only
     from ACCEPTED sessions (`Exact`, `OlderMinor`, `NewerMinor`), and only when the frame's width
     agrees with its session's Hello:
     - Tick, Summary and End: exactly the declared width.
     - Log: `payloadBytes ≥ 14 + tagBytes + messageBytes`, and its non-text bytes equal the declared
       `logFixedBytes`.
     - A frame that disagrees is counted in `wrongWidthSkipped`, never delivered.
     - Bytes beyond this build's known width are counted in `extensionBytesIgnored` (Hello: payload
       − 256; Log: non-text − 14; the others: payload − known width).
   - **Sessions.**
     - A Hello with seq 0 starts a new session.
     - A Hello with seq > 0 while no session is open is a late join; the session starts there
       (`lateJoins`).
     - A Hello with seq > 0 inside a session must match that session's Hello byte for byte (a
       repeat, `helloRepeats`). Otherwise it counts as a `helloMismatch` and starts a new session.
     - An End closes its session. A frame outside any open session (before the first Hello, or
       after an End and before the next Hello) is counted in `framesOutsideSession`, never
       delivered.
     - A refused session's frames are counted in `framesInRefusedSessions` until the next Hello,
       never delivered.
     - Any role value is accepted and reported. Delivery is by type, never by role.
   - **Sequence.** Let d = (seq − expected) mod 2^32, where expected = the last accepted seq + 1.
     - d = 0: in order.
     - 0 < d < 2^31: d frames were lost (`lostFrames`, per session and in total).
     - d ≥ 2^31: a stale or duplicate frame, skipped and counted in `staleFrames`.
     - A sequence that wraps 0xFFFFFFFF → 0 is in order and starts nothing: only a HELLO at seq 0
       starts a session.
   - **Link loss, per session.** `linkLostFrames = max(0, lostFrames − End.framesFailed)`. It is
     computed ONLY for a session read from its seq-0 Hello whose End was delivered; any other session
     counts in `sessionsLinkLossUnknown`. An End whose `framesOffered` differs from its own sequence
     number, or whose `framesFailed` exceeds the frames its session lost, counts in
     `endDisagreements`, and the figure saturates at 0 instead of wrapping.
   - **The tail of a capture.** A whole capture never receives more bytes, so a `NeedMore` at
     position p is treated as a failed candidate: scanning resumes at p + 1.
     - If a frame verifies later, p was a corrupted length; it counts in `lengthOverruns`.
     - `truncated` is set if and only if the capture ends inside a candidate that no later frame
       displaced: the earliest `NeedMore` after the last verified frame. `truncatedAt` gives its
       offset.
     - A capture that ends exactly on a frame boundary is not truncated.
   - **Capture status** (one named verdict for the whole capture):
     - `Empty`: no bytes.
     - `NoFrame`: bytes, but no frame verified. A bit-7-stripping or NUL-truncating transport, or
       not SHUL/2 at all.
     - `NoSession`: frames verified, but no Hello. A newline-translating transport, or a capture
       that ended before the first re-Hello.
     - `Refused`: every session was refused; the latest session's verdict says why.
     - `Ok`: at least one session was accepted.
   - **Counters**, capture-wide: `bytesSkipped` (every byte not inside a verified frame),
     `crcFailures`, `lengthOverruns`, `sessions`, `refusedSessions`, `lateJoins`, `framesDelivered`,
     `framesOutsideSession`, `framesInRefusedSessions`, `unknownTypeSkipped`, `wrongWidthSkipped`,
     `extensionBytesIgnored`, `staleFrames`, `helloRepeats`, `helloMismatches`, `lostFrames`,
     `linkLostFrames` (summed over the sessions where it is known), `sessionsLinkLossUnknown`,
     `endDisagreements`, `sessionsEnded`, `truncated` and `truncatedAt`. And for the latest session:
     its Hello (a view), verdict, `lateJoin`, `lostFrames`, `sawEnd`, its End, `linkLossKnown` and
     `linkLostFrames`.
   - H2's socket loop composes `scanFrame` itself and waits on `NeedMore`. There is no socket code
     here.
4. **A found-while-reading defect, fixed AT ITS SOURCE** (the E1 code).
   - `blackbox::decodeTick` accepts any `wheelCount` byte (~:541), so a decoded 255 would let any
     consumer that loops to `wheelCount` index past the 8-wide arrays. Nothing in the tree loops
     that way today; H2's peer will.
   - Decode now clamps to `DebugRecord::kMaxWheels` (8, `wheel_speeds.hpp` ~:37) and raises
     `corrupt`. Encode clamps to [0, 8] instead of wrapping above 255 (~:459).
5. **A1's comment sweep:**
   - `blackbox_format.hpp` ~:48-53 ("What H1 inherits"): SHUL/2's Tick and Summary payloads ARE these
     bytes.
   - `blackbox_reader.hpp` ~:32-52: SHUL/2 added its own check.
   - `calibration_record.hpp` ~:28-39: the bitwise CRC now lives in `diag/crc32.hpp`.
   - `term_sink.hpp` ~:5: "never the JSON wire" becomes the binary SHUL/2 wire, pointing at
     `shul2_format.hpp` (D1).

### Part A2 — the write side (host-only)

6. **`include/shulib/diag/shul2_sink.hpp`** (NEW): `Shul2Sink final : hal::ITelemetrySink` over
   `hal::IBlockSink` + `hal::IClock`. It implements the F4 seam and never changes it. Its header says
   to pass it the SAME clock the program's `RobotContext` uses, so its times share the records' base.
   - **Lifecycle.**
     - `open(const SessionInfo&)` starts a session: the per-session counters reset, the sequence
       returns to 0, the epoch is read, and the Hello is ENCODED ONCE into a member 256-byte buffer.
       `SessionInfo`'s views are read during `open()` only and never kept (`session_info.hpp`
       ~:42-44). Every re-Hello re-sends the cached bytes, so it matches the first byte for byte by
       construction.
     - `open()` while a session is open closes that session first (its End), so no session ends
       without its account.
     - `close()` sends the End, flushes the device, and leaves the sink closed. `close()` with no
       open session writes nothing.
     - `enabled = false` makes the sink inert: no call writes a byte.
   - **Channels.**
     - `emit()` sends one Tick frame, `log()` one Log frame, and `summarize()` one Summary frame. It
       overrides `summarize()`; the default no-op would drop the summary (`telemetry_sink.hpp`
       ~:122-129).
     - Records, lines and summaries handed over while no session is open are counted in
       `notOpenDropped` and never written. That count rides the NEXT session's End, and covers
       everything since the previous End (or since construction).
   - Each frame is ONE `IBlockSink::write` from a member scratch buffer sized to the largest frame
     (442 bytes). There is no staging and no background task: it is single-task, like
     `sd_sink.hpp` ~:35-44.
   - The sequence number advances for every frame OFFERED to the device, whether or not the write
     succeeds, so a device failure appears as a gap AND in the End's `framesFailed`.
   - **Re-Hello.** `Shul2SinkConfig{enabled = true, helloEveryFrames = 100}`.
     - With N > 0, a re-Hello goes out immediately before the next frame once N frames have followed
       the last Hello, so the Hellos carry sequence numbers 0, N+1, 2(N+1), …
     - N = 0 sends a Hello only at `open()`.
     - A re-Hello that falls due at a wrapped sequence 0 waits one frame: only a session's first
       Hello carries seq 0.
     - The decision is a pure helper, `helloDue(framesSinceHello, everyFrames, nextSeq)`, so the wrap
       case is testable without 2^32 frames. This is a writer POLICY, not frozen.
   - `wantsRecord()` returns `enabled`, and is overridden as a PAIR with `emit()`
     (`telemetry_sink.hpp` ~:38-41).
   - Log caps: tag 16 bytes, message 200 bytes (TermSink's, `term_sink.hpp` ~:277-278), cut on a
     UTF-8 boundary by item 2's helper. They are a writer policy. Truncation sets the flag bits, and a
     line with either flag set counts once in `linesTruncated`.
   - The Log's `t` and the End's `endTime` are the clock's raw readings (item 2's time base).
7. **A2's comment sweep:**
   - `telemetry_sink.hpp` ~:4-6 and ~:86: verify; likely unchanged, since both already name
     `Shul2Sink`.
   - `run_summary.hpp` ~:11 and `debug_record.hpp` ~:6-8: the sink now exists.
   - `term_sink.hpp` ~:5: A1's pointer gains the sink.

### Part B — the F9 freeze (after A1 and A2 are committed)

8. **`test/f9_wire_pin_test.cpp`** (NEW) follows `test/f8_signature_pin_test.cpp`'s header
   discipline: what is frozen, what stays additive, what is deliberately not pinned, and what to do
   when it fails. Every `static_assert` NAMES F9. Pin:
   - the envelope, the same in every major: the sync bytes, the field offsets (sequence at 4, length
     at 8, payload at 10), the overhead (14) and `kMaxPayloadBytes` (4096);
   - the Hello's invariant part: type 1, the signature bytes, versionMajor at payload offset 8 and
     versionMinor at 10; and its 1.0 width, 256;
   - `kVersionMajor == 1`;
   - the width table: the 1.0 row {256, 428, 168, 32, 14} unchanged, and exactly
     `kVersionMinor + 1` rows. **No pin on `kVersionMinor`'s value.** The header says so, in the F8
     pin's words (~:473-476): the vacuous `>= 0` is what F1's reviewer removed, and on a `u16` it
     also fails the build with `-Werror=type-limits`;
   - the frame-type values and ranges, and the wire level codes;
   - every `FaultCode` value 0–12, every `GateReason` value 0–12, `TickPhase` 0–5,
     `kTickPhaseSlots == 8`, `DebugRecord::kMaxWheels == 8`, `motion::MotionState` 0–5, and the
     `Localizer::Quality` values 0–3 that `qualityClass` mirrors. (The test may include `motion/` and
     `localization/`; no `diag/` header may.)

   Two `TEST_CASE`s whose messages name F9:
   - `diag::crc32Ieee` over the published check values ("" → 0x00000000, "a" → 0xE8B7BE43,
     "123456789" → 0xCBF43926);
   - **the permanent 1.0 capture (D8):** a hand-derived 1.0 capture, checked in as bytes, read by THIS
     build's reader gives `Exact` or `OlderMinor`, every 1.0 field bit-identical, and every field the
     1.0 layout lacks at its documented default.

   The byte goldens (which also pin the tick flag bits and the enum bytes) stay in A1's tests. They
   are the artifact's own pin, as the manifest golden is for F8.
9. **The comment updates** that say F9 is now frozen:
   - `version.hpp` ~:38-41: F9 LOCKED at H1 Part B (<date>); its version is the Hello's
     major and minor, not `kApiMinor` (D7);
   - `debug_record.hpp` ~:10-17, `fault.hpp` ~:14-18, `blackbox_format.hpp` ~:48-53 (A1's text, now
     past tense), and the header of `shul2_format.hpp`;
   - read `debug_record.hpp` ~:46, ~:98-101, ~:118-121, ~:191 and `motion.hpp` ~:121, ~:144-146, and
     change only what reads as future;
   - then grep `include/` and `test/` for `F9`, `H1 freeze` and `future-frozen` and read every hit
     (known: `debug_record_test.cpp` ~:4, ~:44, ~:66; `fault_test.cpp` ~:2, ~:30).

## 2. Tests (adversarial) and mutations

**Goldens are derived by hand from the layout tables.** Read them back with INDEPENDENT little-endian
readers (the `blackbox_format_test.cpp` ~:150 precedent). Compute each CRC literal outside the code
with `python3 -c "import zlib; …"`, and log the exact command in the PROGRESS log. Never capture a
golden from the encoder.

### Part A1

**Goldens:** the envelope; the Hello with every field at its offset; the Log; the End; and a whole
Tick frame including its CRC (which also pins the tick flag bits and the enum bytes).

**Round trip:**
- Every frame type survives field for field, and the tick uses a record with every field distinct and
  non-default.
- NaN, ±Inf, −0.0 and denormals survive BIT-EXACTLY in every f64 field EXCEPT the three `math::Angle`
  fields (`targetPose.heading`, `measuredPose.heading`, `imuYaw`). Compare by bit pattern (`memcpy` to
  `u64`), never with `==`, which fails on NaN and cannot see −0.0 (`blackbox_format_test.cpp`
  ~:100-148 compares with `==`). For the Angle fields, a non-finite value decodes to 0 with `corrupt`
  raised (`safeAngle`, ~:495-506): assert exactly that.
- Every `FaultCode`, plus the unknown value 0xFFFF, travels as its number. So do every `GateReason`,
  every `MotionState`, `qualityClass` 0–3 and an unknown 200.
- A Log survives an empty tag and message, and a message carrying every byte value 0x00–0xFF
  (framing unbroken, text verbatim). Level code 7 decodes as 7 and maps to "unknown".
- `diag::crc32Ieee`: "" → 0x00000000, "a" → 0xE8B7BE43, "123456789" → 0xCBF43926 (each computed with
  zlib and logged). The calibration record's golden and `CalibrationRecord 1b`
  (`calibration_record_test.cpp` ~:188-201) still pass unchanged through the forwarding function.

**Signature** (at the `negotiate` level, with the CRC recomputed): each alteration gives
`BadSignature`, never acceptance: 0x89 with bit 7 cleared; the NUL removed; CR LF → LF; LF → CR LF.

**Transport** (at the reader level, on real transforms of a whole capture, CRC NOT recomputed). Build a
capture of a Hello, 50 Ticks (at least one chosen so that its WHOLE frame, envelope and CRC included,
contains no 0x0A or 0x0D byte), some Logs and an End. Then apply: (1) every byte AND 0x7F; (2) each frame cut at its first 0x00 (a C-string transport,
one write per frame); (3) every 0x0A → 0x0D 0x0A; (4) every 0x0D 0x0A → 0x0A. For each: zero frames
delivered, `sessions == 0`, and the exact status: `NoFrame` for (1) and (2); `NoSession` with
`framesOutsideSession ≥ 1` for (3) and (4).

**Negotiation** (checked against the 1.0 profile AND a hypothetical 1.1 profile with its own row):
- Run the whole table: major ∈ {0, 1, 2, 65535} × minor ∈ {0, 1, 7, 65535} × declared widths {equal,
  shorter, longer}, plus Hello payloads of 11, 12 and 255 bytes.
- **The unknown future version:** hand-build a 1.7 stream whose Hello is 272 bytes and declares every
  width +16 (`logFixedBytes` 30, the 16 after the text), and whose every frame carries 16 appended
  bytes. The decoded prefix must be bit-identical to 1.0, `extensionBytesIgnored == 16 ×` frames
  (Hellos included), and nothing may be refused.
- **A 2.0 session is refused whole.** Build its Hello in the invariant envelope with the frozen bytes
  0–11 (everything after them may differ). Zero frames delivered, its Hello still reports 2.0, and
  `framesInRefusedSessions` counts its frames. A 1.0 session that FOLLOWS it in the same capture IS
  delivered (`sessions` 2, `refusedSessions` 1, status `Ok`).
- At the same minor, a width mismatch is refused. A newer minor with a SHORTER width is refused. Under
  the 1.1 profile, a 1.0 stream whose widths differ from the 1.0 row is refused, and one whose widths
  equal it is `OlderMinor`.

**Forward compatibility** (CRC recomputed):
- Envelope flags 0xFF: delivered.
- Hello flags 0xFFFF and every reserved byte 0xFF: accepted.
- Log reserved 0xFF and flag bits 2–7 set: decoded.
- Tick flag bits 2–7 set: decoded, with bits 0–1 unchanged.
- Hello roles 1 and 7: accepted, reported, and frames delivered by type.

**Framing and integrity:**
- Types 0, 60, 64 and 200 are skipped by their length, counted, and the stream continues.
- **The flip loop.** For ONE Tick frame, at every byte position, apply each of the 8 single-bit flips
  and an XOR with 0xFF (442 × 9 cases). Run the loop in two placements: mid-capture with more than
  4110 bytes after the frame, and as the third frame before the End. In every case, exactly that frame
  is lost, every later frame (the End included) is delivered, `lostFrames == 1` and `truncated` is
  false.
- **The tail case, named:** the third frame before the End with its length set to 2476 (one bit flip
  of 428 = 0x01AC). The two later frames and the End are delivered, `lengthOverruns ≥ 1`, and
  `truncated` is false.
- A garbage prefix containing false sync pairs is skipped up to the Hello, with `bytesSkipped` equal
  to the prefix length.
- A corrupted length of 60000 is not a frame; it resyncs by 1 byte.
- A corrupted length of 4000 on a LIVE stream (the test's reassembler) returns `NeedMore` until 4014
  bytes are held, then fails the CRC, and **the frames inside those 4000 bytes are recovered**.
- A capture cut at every offset 1 … 14+n−1 of its last frame gives `truncated`, `truncatedAt` at that
  frame's start, every earlier frame delivered, and `!sawEnd`. Cut at offset 0 (the capture ends on a
  frame boundary): `!truncated` and `!sawEnd`.

**Chunking property:** feed a scripted stream through `scanFrame` with a test-side reassembler, in
every chunk size 1–64 and in seeded random splits, and apply the whole-capture tail rule at the end of
input. The frames must be identical to `Shul2Reader` reading the whole buffer.

**Sequence** (hand-built captures):
- A frame cut out of a complete session: `lostFrames 1`, `linkLostFrames 1`.
- One gap and an End declaring `framesFailed 1`: `linkLostFrames 0`.
- A late join whose End's `framesFailed` exceeds the gaps seen: `linkLossKnown` false,
  `sessionsLinkLossUnknown 1`, and nothing wraps.
- A session with no End: `linkLossKnown` false.
- A seq-0 session whose End's `framesFailed` exceeds its gaps: `endDisagreements 1`,
  `linkLostFrames 0`.
- The wrap 0xFFFFFFFE → 0xFFFFFFFF → 0 → 1 (Ticks) has no gap and does not start a session.
- A duplicate frame is stale: skipped and counted.
- A capture starting at a random offset delivers from the first Hello; the frames before it count in
  `framesOutsideSession`.
- An identical re-Hello is not a new session. A different one at seq > 0 counts a mismatch and starts
  a new session.
- A frame after an End and before the next Hello counts in `framesOutsideSession`.

**`wheelCount`:** the decode of bytes 9 and 255 gives 8 with `corrupt` raised; the encode of 300 gives
the byte 8.

### Part A2

**Sink** (every test uses a `FakeClock` started at a NON-ZERO time, so a stamp on the wrong base
fails):
- Nothing is written before `open()`, and what was offered is counted and appears in the next
  session's End. `enabled = false` means `emitRecord`'s builder is NEVER invoked, and `open`, `log`,
  `emit`, `summarize` and `close` write zero bytes.
- **The Hello the sink writes**, decoded: every `SessionInfo` field, `epochSeconds` equal to the
  clock at `open()`, `apiMajor`/`apiMinor` equal to `version.hpp`'s, the widths equal to the 1.0 row,
  role 0.
- **`SessionInfo` is not retained:** build it from a stack `char` buffer, `open()`, overwrite the
  buffer, then drive 250 frames with `helloEveryFrames = 100`. Every re-Hello is byte-identical to the
  first.
- **The re-Hello cadence:** with N = 100, Hellos at seq 0, 101 and 202 exactly; with N = 1, at 0, 2,
  4, …; with N = 0, only at 0. `helloDue` with `nextSeq == 0` returns false however many frames have
  passed, and true on the next frame.
- **The End**, decoded after a scripted session (k records, m lines of which t were truncated, s
  summaries, x items offered before `open()`, one device refusal): `framesOffered` equals the End's
  own seq, `framesFailed 1`, `recordsSeen k`, `linesSeen m`, `linesTruncated t`, `notOpenDropped x`,
  and `endTime` equals the clock at `close()`.
- **The Log:** `t` equals the clock's raw reading at `log()` (NOT that minus the epoch). A 17-byte tag
  is cut to 16 with flag bit 0. A 201-byte message with a multi-byte character straddling the cap is
  cut at the boundary with flag bit 1. A line with both counts once in `linesTruncated`.
- Each `LogLevel` maps to its wire code (golden) and back.
- `summarize()` writes one Summary frame whose `blackboxDropped` is the summary's own field.
- Exactly one `write()` per frame. Use a test-local `LossyBlockSink`, because `FakeBlockSink`'s
  `setCapacity` (~:47-49) fails EVERY write after the cap. Two modes:
  - it refuses one write whole: the reader shows `lostFrames 1`, End `framesFailed 1`,
    `linkLostFrames 0`, and the later frames delivered;
  - it keeps a 100-byte prefix of one write and reports failure (`block_sink.hpp` ~:30-32 allows
    this): the same, plus `crcFailures ≥ 1` or `lengthOverruns ≥ 1`, and the next frame delivered.
- A second `open()` while open writes the first session's End, then a new Hello at seq 0 (the reader
  shows `sessions 2`, `sessionsEnded ≥ 1`). A second `close()` writes nothing, and so does `close()`
  with no session open.
- `RateLimitedSink{Shul2Sink}` stamps `droppedRecords` onto the wire (D-2 end to end).

**End to end on the host test plant:**
- Build the rig the `blackbox_introspection_test.cpp` ~:144-195 way, with the sink injected through
  `RobotContextConfig::telemetry` and given the rig's clock.
- Run a seeded `moveTo` once with `SdSink{streamTicks = true}` and once with `Shul2Sink`. Size the
  `SdSink` buffer to hold the whole run, because 64 KiB holds 151 tick frames after its header and a
  full buffer drops frames whole (`sd_sink.hpp` ~:380-396); assert `droppedFrames() == 0`. The records
  decoded from the two captures must be equal field for field (f64s by bit pattern).
- Re-wrap the blackbox capture's Tick payloads as SHUL/2 frames after a Hello. They must decode
  identically, which proves H2's replay premise.

### Mutations

Run each on a FRESH binary, observe it red, and log it.

**Part A1:**
1. The CRC covers the payload only.
2. The reader skips the CRC check.
3. Resync skips the claimed length.
4. The 4096 bound is removed (bound the test so a stall fails instead of hanging).
5. The gap is computed without the modulo-2^32 wrap.
6. `negotiate` accepts another major.
7. `negotiate` refuses a newer minor.
8. A width mismatch at the same minor is accepted.
9. An older minor's widths are not checked against that minor's row (the 1.1 profile).
10. A newer-minor tick is decoded from the whole payload.
11. Extension bytes are not counted.
12. `logFixedBytes` is read as the bytes BEFORE the text.
13. Two wire level codes are swapped.
14. An unknown level code is cast to `hal::LogLevel`.
15. The UTF-8 prefix helper splits a sequence.
16. A repeated Hello starts a new session.
17. The signature's NUL is changed to 0x20.
18. Frames of a refused session are delivered.
19. A stale frame is delivered.
20. At the capture's tail, `NeedMore` stops the reader instead of resyncing.
21. `truncated` is set although a later frame verified.
22. The reader refuses nonzero envelope flags.
23. The reader refuses a nonzero reserved Hello byte.
24. `linkLostFrames` is computed for a late-join session, without saturation.
25. The `wheelCount` clamp is removed.
26. **Two fields are swapped in BOTH `encodeTick` and `decodeTick`.** The round trip must stay GREEN
    and the golden must go RED. Log that result as the reason the goldens exist.

**Part A2:**
27. The sequence does not advance on a failed write.
28. `framesFailed` is not incremented.
29. `messageBytes` is written as the untruncated length.
30. The sink writes before `open()`.
31. `wantsRecord()` returns true while disabled.
32. The Summary's `blackboxDropped` is overwritten with the sink's own figure.
33. `summarize()` is not overridden.
34. Each End field in turn is written as 0: `recordsSeen`, `linesSeen`, `linesTruncated`,
    `notOpenDropped`, `endTime` (five runs).
35. The Hello's `epochSeconds` is written as 0; then `apiMajor`/`apiMinor` as 0; then the routine id
    is dropped (three runs).
36. Re-Hellos are re-encoded from the `SessionInfo` views kept since `open()`.
37. The Log's `t` is stamped as clock minus epoch.
38. The re-Hello cadence is off by one; a re-Hello is sent with N = 0; a re-Hello is sent at a wrapped
    seq 0 (three runs).
39. A second `open()` while open writes no End.

**Part B:** change each pinned constant in turn. The build must fail with a message that NAMES F9;
copy the message into the log. Then make the reader's tick decode require 436 bytes (a 1.1 without an
older-minor path, D8): the permanent 1.0-capture test must go red naming F9.

## 3. Documentation (same change)

**Part A1:**
- `docs/vexbuilder-integration.md`:
  - §5.2 is rewritten as the contract, with units:
    - every layout table: the envelope, the Hello, the Log, the End, AND the 428-byte Tick and
      168-byte Summary offset tables (`blackbox_format.hpp` ~:424-435, ~:581-586);
    - the value tables: `FaultCode`, `GateReason`, `TickPhase`, `MotionState`, `qualityClass`, the
      tick flag bits, the wire level codes, and the frame types with D4's ranges;
    - the CRC spec (zlib-compatible, and its coverage) and the time base;
    - the resync rule, and a live reader's buffer: at least 14 + 4096 bytes, and the stall a
      corrupted length causes (about nine tick frames) before its frames are recovered;
    - the session, sequence, re-Hello and version rules, including what every major keeps and D8;
    - one writer per capture, and what v1 does not carry.
  - §5.3: in v1, markers reach VexBuilder only as Log text (the runner's lines); a data frame for them
    is an additive type later.
  - §1's H1 row: the format and reader exist, host-only, not yet frozen. (G1 Part 2c edits the row at
    ~:64; change only H1's row.)
  - The wire has never crossed a real link, and the doc says so.
- `docs/shulib-v2-master-plan.md` §18.1 ~:828-830: "the JSON wire" becomes binary (D1).
- `docs/diagnostics-plan.md`: ~:30-51 answers the F9 question (D5), and the ~:86 row reads H1 Part A1.
- `docs/hardware-assumptions.md`: HA-140's *Source* (~:1426) names `diag/crc32.hpp` as where the
  bitwise CRC lives, called by the calibration record.
- `docs/changelog.md`: a new `### Added since 2.2 (<date>) — the SHUL/2 wire format and its reader`
  block covering ADDED, FIXED (`wheelCount`), "`calibration::crc32Ieee` now forwards to
  `diag::crc32Ieee`", and "What you must do: nothing".
- Guide ch.11 ("Reading the diagnostics") ~:460: links to the new API pages (`crc32`, `shul2_format`,
  `shul2_reader`).
- `docs/api` regenerated. `build-order.md` H1's status line. The briefing regenerated LAST.

**Part A2:**
- `docs/hardware-assumptions.md`: HA-168 and HA-169 (below), with index rows and one appended status
  sentence. The numbers are reserved in the coordinator's notes (HA-162–167 belong to other chunks in
  this wave); a gap in the register is expected if H1 lands first. No renumbering.
- `docs/changelog.md`: a new `### Added since 2.2 (<date>) — the SHUL/2 sink` block.
- Guide ch.14 ("What it cannot do yet") ~:377-379: the wire's format, reader and sink exist and are
  host-proven, and nothing streams it yet. It currently ends "a USB tether or the simulator"; rewrite
  it truthfully, with no "simulator".
- Guide ch.11 ~:460: a link to the sink's API page.
- `docs/vexbuilder-integration.md` §1's H1 row: the sink exists too, host-only, not yet frozen.
- `docs/api` regenerated. `build-order.md` H1's status line. The briefing regenerated LAST.

**Part B:**
- `docs/roadmap.md`:
  - The F9 row becomes ✅ LOCKED <date>, stating exactly what is frozen (D6's list).
  - It says out loud what is NOT frozen: the C++ classes, the writer policies (re-Hello interval, text
    caps, `open()` on an open session) and the transport (H2's).
  - It names its enforcement (the pin file, A1's goldens, the permanent 1.0 capture) and its caveat:
    one consumer, written in the same chunk; never on a robot or a real link.
  - The M6 SHUL/2 box is `[x]`, marked host-only.
  - The ~:1960 sentence and the ~:2001 row are updated.
- `docs/vexbuilder-integration.md` §0 ~:35 ("F9 is not written"), §1, the §7 risk row ~:479, §8.2
  item 5 (~:511).
- `docs/shulib-v2-master-plan.md` decision row 13 (~:568).
- `changelog.md`: one line, "F9 FROZEN".
- `H1-COMPLETED.md`. `build-order.md` status. The coordinator checks the pointers.

Public docs cite evidence by date, never by a `docs/internal` path (C7). The prose review is logged:
grep `docs/` and `README.md` for `SHUL`, `Shul2`, `F9`, `wire`, `JSON`, `crc32` and every new header
name, and read every hit.

**HA-168 — PROS's stdout carries SHUL/2's bytes unaltered, and an overfull write BLOCKS or DROPS.**
*Claim:* (a) On PROS 4.2.2, bytes written to stdout (the brain's USB user port) arrive at the laptop
unaltered inside PROS's COBS `sout` framing. That includes 0x00, 0x0A, 0x0D and bytes ≥ 0x80: no
newline translation and no 7-bit stripping. (b) When PROS's transmit buffer is full, a write either
BLOCKS the calling task or DROPS bytes. *Source:* `diag/shul2_format.hpp` (the per-frame CRC and the
signature make (a)'s failure total; the CRC and the resync exist for (b)'s drop case),
`diag/shul2_reader.hpp` (the capture status that names it) and `diag/shul2_sink.hpp` (one synchronous
write per frame on the caller's task). *Confidence:* (a) **reasoned**, because COBS framing was
measured (HA-129) and COBS exists to carry 0x00. (b) is **UNKNOWN (labelled invented)**. HA-60 records
that the legacy code needed ~900-byte chunks with delays on V5 serial. *Settle:* the first chunk that
attaches a `Shul2Sink` to an on-brain program; none is planned, and H2's link is a host socket. Stream
the Hello and 1000 ticks, read the raw user port with the HA-129 recipe, and note whether stdout's own
buffering holds frames back. *Blast radius:* if (a) is false, no session ever starts. A bit-7-stripping
or NUL-truncating link leaves the reader at `NoFrame`; a newline-translating one leaves it at
`NoSession`, because the signature's CR LF guarantees every Hello dies even when a Tick survives. Either
way `bytesSkipped` covers the capture: loud and contained. If writes BLOCK, the control tick stalls and
`LoopMonitor` reports overruns (visible). If they DROP, the reader reports CRC failures and
`linkLostFrames` (visible). **Contained:** no `src/` program uses the sink.

**HA-169 — the SHUL/2 stream fits the brain.** *Claim:* at the ~100 Hz record rate a tick stream is
~44 KB/s (442-byte frames). Encoding one frame plus its bitwise CRC-32 (436 bytes, ~3,500 bit-steps)
costs the control task well under 0.1 ms on the V5's Cortex-A9, and the USB user port sustains
44 KB/s. *Source:* `diag/shul2_sink.hpp`, and `diag/crc32.hpp`'s no-table choice. HA-140 judged that
choice for 180 bytes once per boot; here it is about 43.6 KB/s of CRC input. *Confidence:*
**invented**; the CPU half is reasoned from arithmetic. *Settle:* the same as HA-168's: the first chunk
that attaches a `Shul2Sink` to an on-brain program times `emit()` there and measures the port's
sustained throughput. None is planned. *Blast radius:* loop overruns (visible). The fix is
`RateLimitedSink`'s rate, which is a policy change, not a format change.

## 4. OUT

- H2: the `hal/sim` adapter, the agent socket, `server.json`, inbound sensor frames, record/replay,
  and the overlay.
- Any PROS stdout adapter (and its buffering), and any `src/` wiring.
- H3's tuner.
- A data frame for per-motion results or markers.
- A blackbox format change beyond the `wheelCount` clamp.
- Any change to `ITelemetrySink` (F4).
- An API version bump (D7).
- Any hardware claim: the wire has only ever carried host bytes.

## 5. Checks

For each part:
- The suite is green, with the counts logged.
- Every mutation is RED on a FRESH binary, using the stale-binary recipe (`HANDOFF-remaining-work.md`
  ~:208-216).
- `python3 tools/src_build_gate.py self-test && check`. Nothing in `src/` changed; this proves it.
- The ARM header gate (`-Wconversion -Wsign-conversion -Werror`).
- Both CI greps: no `pros/` or `sim/` include in the new headers.
- `api_doc_tool` self-test, coverage, check-fresh, check-examples and check-removability.
- `doc_staleness_audit.py`.
- The briefing regenerated LAST and checked.

## 6. Landmines

1. `hal::LogLevel` has no explicit values: map it by `switch`, never by cast, in both directions.
2. `blackbox::decodeTick` requires EXACTLY 428 bytes (~:513): pass `payload.first(428)`.
3. `math::Angle` refuses non-finite values: headings decode through `blackbox::safeAngle` (~:500), and
   the round trip's non-finite cases exclude the three Angle fields.
4. The stale-binary trap: touch every test that includes an edited header, and require "Linking CXX
   executable shulib_tests".
5. A round trip cannot see a field moved in both directions (mutation 26). Only hand-derived goldens
   can.
6. `diag/` stays a leaf. The new headers include only `diag/`, the `hal/` seams, `units/`, `math/`,
   `core/check.hpp`, `kinematics/wheel_speeds.hpp` (through `debug_record.hpp`) and
   `shulib/version.hpp` (which includes nothing). Never `motion/` or `localization/`; the Part B test
   may.
7. Checked casts, or the ARM gate fails on `-Wconversion`: `size_t` → `u16` for lengths, and `int` →
   `u8` for `apiMajor` / `apiMinor`.
8. Do not sanitize wire text, and never assume NUL termination on decode.
9. Never keep `SessionInfo`'s views past `open()`: cache the encoded Hello.
10. Every time on the wire is the clock's raw reading, never epoch-relative.
11. The calibration CRC spelling is a documented forwarding function, not a `using` (the doc gate
    would label it a "type alias").
12. `diag/session_info.hpp` and `calibration_record.hpp` are edited by other chunks. H1 does not edit
    the first, and edits only ~:28-39 and ~:106-121 of the second.
13. Say "host test plant", never "simulator".
14. Use HA-168 and HA-169 only.

## Found while reading (not H1's to fix)

- `vexbuilder-integration.md` §8.2 ~:510 still lists "G4 — legacy importer" (dropped 2026-09-26),
  and item 3 predates G3a. That is the coordinator's, because it may collide with G3a's doc edits.
- D7's version-policy wording: `version.hpp` ~:24-28 reads as if every additive change bumps
  `kApiMinor`, while the `kApiMinor` comment (~:50-51) says "additive extensions of a frozen surface",
  which is what the practice since 2.2 follows. One clarifying sentence after the parallel worktrees
  merge; not a restructure of the changelog inside H1.
- `blackbox::ByteReader::text` (`blackbox_format.hpp` ~:314-325) skips an interior NUL and joins the
  text on either side of it, so a field written as "ab\0cd" decodes as "abcd". E1's reader; H1's Hello
  decoder ends a field at its first NUL instead.
- SAFE-SAVE's draft cites `calibration_record.hpp` "CRC-32 :106-121". If H1 lands first, that
  citation moves to `diag/crc32.hpp`.
