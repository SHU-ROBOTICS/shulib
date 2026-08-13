# E1 Progress Log — SdSink + estimator introspection

- 21:10:03 START. Creating progress log; beginning required reading (brief first).
- 21:10:23 Read brief, debug_record.hpp (full), char_sink.hpp, diagnostics-plan.md (D-6/D-7/D-8, §18.4 discharge table), A1-COMPLETED.md, RESUMING.md. Next: C5-COMPLETED, diag/ sinks, localization seam.
- 21:12:59 Reading complete on the seam side. Key findings so far:
  * Localizer already exposes lastCorrection() (AppliedCorrection: dx,dy,gated,clamped,source) — the introspection carrier exists but carries NO residual/Mahalanobis/reason/covariance.
  * IFusionPolicy::fuse returns FusionResult (x,y,applied,gated,clamped,appliedConfidence) — no audit either.
  * Only move_to_pose.hpp stamps correctionDx/Dy/clampedThisTick onto the record; turn_to/strafe/brake/hold/idle do NOT. That is the gap the introspection path has to close.
  * MotionScheduler::tickBody() calls localizer->update() FIRST each tick and already owns CommandIdStampSink, the decorator that stamps id + tickPhase onto EVERY record — the natural single place to stamp estimator fields too (push, like setTickPhases).
  * TermSink::emit does NOT render any gate/correction field except CLMP, so central stamping cannot disturb A1/C5 goldens.
- BASELINE (before any edit): suite 707 cases / 916,216 assertions / 3 skipped — SUCCESS.

## Rulings on the three tensions (made BEFORE the format was written)

- T1 — off-task writes vs "no background task": **KEEP THE STANDING DECISION.** No task, no
  thread, no async anything. Records are encoded into a caller-owned RAM buffer synchronously on
  the caller's task; bytes reach the device only when the caller says so (motion boundary, fault,
  auton end, close). The byte budget + drop counter absorb the cost of a caller who flushes too
  rarely. REJECTED: a writer task (would be the tree's first two-task design, unbuildable
  PROS-free at M2, and would end host determinism — every closed-loop test in this project is
  reproducible from a seed BECAUSE there is exactly one task).
- T2 — text seam vs binary blackbox: **NEW ADDITIVE SIBLING SEAM `hal::IBlockSink`**
  (`write(std::span<const std::byte>) -> bool`, plus a non-pure `flush()`), NOT a redefinition of
  ICharSink. REJECTED: widening ICharSink's contract ("one write() carries one complete line" is
  load-bearing for TermSink's framing proof; a seam that means two things means nothing) and
  base64/hex-over-ICharSink (a text blackbox is explicitly rejected by the brief, and it would
  blow the byte budget by 33%+ for nothing).
- T3 — the gating DoD cannot close at E1: **HALF-CLOSE IT, AND SAY SO.** E1 ships the recording
  path + the introspection surface and proves it with a synthetic corrector/policy emitting every
  GateReason. E2 owns the other half (real gate decisions). The DoD line will be reported `[~]`,
  never green on the strength of a test double.
- 21:19:13 Design locked. Writing code now, in order: hal/block_sink.hpp (T2 seam) → fake_block_sink → diag/blackbox_format.hpp (encoder+layout) → diag/blackbox_reader.hpp (decoder) → diag/sd_sink.hpp → introspection wiring (correction.hpp GateAudit, ComplementaryFusion, Localizer, CommandIdStampSink) → tests.
  Format v1: 256-byte header (magic SHBB, version, headerBytes, tickRecordBytes, epoch, ring cap, byte budget, bounded provenance strings), then typed frames (4-byte frame header: type,reserved,payloadBytes). Tick payload 428 B, all f64 little-endian — NO f32 narrowing anywhere, so "field-by-field equality" after a round trip is literally true rather than "equal after documented narrowing".
- 21:30:00 Core headers written: hal/block_sink.hpp, hal/fake/fake_block_sink.hpp, diag/blackbox_format.hpp (encoder+layout, 428-byte tick), diag/blackbox_reader.hpp (decoder, refuse-do-not-misread), diag/sd_sink.hpp (ring + budget + dump + graceful end). Introspection wired: GateAudit on FusionResult/AppliedCorrection, populated by ComplementaryFusion, carried by Localizer, stamped by CommandIdStampSink.
- 21:30:00 FINDING (real, pre-existing): DebugRecord::fault had NO producer anywhere in the tree. TermSink has rendered " flt=NAME" since A1 and it could never appear on a real run, and the D-6 dump trigger depends on it. Fixed in the layer that owns record population (Rule 4): the scheduler stamp decorator now stamps the tick fault from the FaultLatch. Second finding, same layer: only MoveToPose stamped correctionDx/Dy/clamped — every other motion and the idle record left the fusion story blank.
- 21:30:00 Suite after the wiring changes, BEFORE any new test: 707/916,216 still green (no existing assertion depended on those fields being empty).
- 21:36:00 blackbox_format_test.cpp GREEN (20 cases here, 20,348 assertions in this file alone — the per-field golden sweeps all 428 bytes for each of 44 fields).
  FIX found by a test I expected to pass: encodeTick() into an undersized buffer wrote a PARTIAL record before latching overflow. ByteWriter is per-append safe, but the encoder as a whole was not "whole or nothing". All six encoders now check the size up front and write nothing on failure. That matters because a half-written frame sitting in a shared staging buffer is the kind of thing a later optimistic cursor update turns into corrupt data.
- 21:41:08 ALL THREE TEST FILES GREEN. Full suite 749 cases / 936,865 assertions (from 707/916,216): +42 cases, +20,649 assertions.
  Test-found bug in my own test (kept as a lesson): resizing a std::vector the SdSink already held a span over left the sink writing into freed memory. Fixed by sizing the buffer at construction — the caller-owned-storage contract has to be honoured by the CALLER, and the test was the first caller to get it wrong.
- 21:41:08 Starting the mandatory mutation campaign. Runner gates on build success (D1 tripped this twice).
- 21:44:33 MUTATIONS 1-16 run. 14 RED as required. **TWO GREEN HOLES FOUND** — both the same class: the suite tested the FORMATs ability to carry the sinks self-description, and never tested that the sink puts real values in it.
  * HOLE #1 (M14): the end frames tickFrames and bytesWritten could both be hard-coded to 0 and the entire suite stayed green. The close() case asserted tickFrames == 0 for a ring-only run (true either way) and never looked at bytesWritten. A file that lies about how much of the run it contains sends a reader hunting for data that was never missing.
  * HOLE #2 (M15): the header epoch, ringCapacity and byteBudget written BY THE SINK could all be zeroed with the suite green. The format test checked those fields via a direct encodeHeader() call — never through SdSink. Consequence: a blackbox with epoch 0 timestamps the whole run wrong, and ringCapacity 0 removes the readers only way to know how deep the flight recorders history goes.
  * Notable NON-hole (M1b): the symmetric offset change — moving a field in the encoder AND the decoder together — left the round trip GREEN (748/749 cases passed) and was caught ONLY by the byte-exact per-field golden. That is the exact class of bug a round trip structurally cannot see, demonstrated rather than argued.
- 21:47:21 MUTATION M18 came back GREEN: making stage() commit a half-encoded frame changes nothing observable. Investigated rather than assumed: that branch is UNREACHABLE by construction — every encoder now checks its size up front (the earlier fix), and stage() always hands it a span of exactly payloadBytes, so encode() cannot return a short count. So this is a green mutation on dead-defensive code, NOT a missing test, and I am not going to invent a test that pretends otherwise. Keeping the guard (a future encoder could fail for a non-size reason) and documenting it as deliberately unreachable.
- 21:47:21 Green holes #1 and #2 CLOSED and re-mutated: M14-REDO and M15-REDO are now RED, each caught by exactly its new case (sd_sink_test.cpp:422/429/431 and :452/453/454). While closing #1 I also found and fixed a REAL defect: the end frames byte field read 0 for the most common run shape (caller-paced flushing means nothing is confirmed until the final write). Redefined it as bytesBefore — the frames own offset in the file, which a reader can VERIFY against where it found the frame.
- 21:50:41 Docs: HA-58/59/60 registered (ring depth, RAM budget, assumed SD flush cost — all invented, all labelled in-header with PROVISIONAL (A4: HA-nn)). diagnostics-plan D-6/D-7 marked delivered with what-was-built notes; D-8 marked NOT delivered and re-homed to F2 with the reasoning (it shares nothing with the blackbox but the word diagnostics). Roadmap M3 WS13 checkboxes: two [x], one [~] (introspection, half-closed), plus the you-are-here entry.
- 21:55:42 E1-COMPLETED.md written (T1/T2/T3 in their own section, findings, 27-mutation table, DoD with the honest [~]). Guide chapter 11 gained a "The blackbox" section (no ```cpp blocks — nothing to compile against until R1 exists, so prose + a text layout diagram instead of an aspirational example) and the summary bullet notes the conditional blackbox-dropped field. guide-maintenance.md gained its row.
- 21:56:45 FINAL VERIFICATION, all re-run from scratch after the last edit:
    suite      752 cases / 936,895 assertions / 3 deliberate skips — SUCCESS (baseline was 707/916,216)
    GUARD1     PASS (PROS-free, all of include/shulib)
    GUARD2     PASS (no sim/ leakage)
    ARM gate   CLEAN, 110 headers (104 before this chunk + 6 new)
    doc gates  self-test / coverage / freshness / verbatim examples / removability — ALL PASS
    git        HEAD still 71349d1 (the brief). NOTHING COMMITTED. CI guard scope already covers include/shulib wholesale since C7 — no ci.yml change needed.
- 21:56:45 DONE. E1 complete, in the working tree, pending review.
