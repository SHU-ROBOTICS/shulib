# SER1 — COMPLETED (landed 2026-10-15, `2506067`; host- and build-gated, NOT run on a robot)

> The USB serial log keeps every line whole when two tasks write it (A4 register HA-189). The team lead's go of
> 2026-10-08 with his fix, re-ruled the same day to ONE-CALL WRITES when the checker found his first choice
> (stdout unbuffered at boot) would overflow the PROS serial daemon's 2 KB stack. Written FROM
> [`SER1-PROGRESS.md`](SER1-PROGRESS.md) (the build, stopped by two pauses and resumed; its review round; a fresh
> verifier's round) and the coordinator's merge verification at its end. Brief:
> [`SER1-serial-lines-whole.md`](SER1-serial-lines-whole.md) (its amendments A1–A7 override its body). Executor
> model: `opus`, in the isolated worktree `.claude/worktrees/ser1`. *(This record was written after the commit
> that landed SER1 — `2506067` closed the code, the documents and the merge record, but not this file or the
> roadmap's pointer; RESUMING's rule 5 says a chunk without them is not closed, so they follow in their own
> commit.)*

## The one-sentence honest status

Every line shulib sends to the brain's serial stream now leaves in ONE `write()` on the serial's descriptor —
which PROS's own serial lock keeps whole — instead of going through the C library's single, unlocked stdout
buffer, and the `src/` build gate refuses any program object that names stdio's FILEs or calls a stdout/stderr
printer; all of it is reasoned from the vendored `libc.a`/`libpros.a` and proven on the host and in all seven
builds, none of it yet on a robot (HA-189 settles only at the bench).

## Built (`2506067`)

- `include/shulib/hal/pros/char_sink.hpp`: `ProsCharSink::write()` sends one direct `write()` per call (pieces of
  at most `kSerialWriteMaxBytes`, 512, each whole — so no shulib call can reach PROS's ~2 KB limit on one write,
  HA-191); `writeWhole()` / `PosixWrite` (the remainder after a short write; a stop, never a spin, on 0 or −1);
  `serialLine(format, …)` (one printf-style line, at most 256 bytes with its newline, a longer one CUT visibly);
  `serialWrapped(tag, text)` (any length, as `[tag] ` lines broken at spaces, never cut); a default constructor
  for the robot's serial (fd 1). `ProsCharSink(std::FILE*)` now writes that FILE's descriptor (a BEHAVIOUR CHANGE,
  in the changelog).
- Every `src/` stdout writer converted to `serialLine` / `serialWrapped`; the bytes of every boot line unchanged
  except a line longer than 255 characters, which now arrives as `[TAG] ` pieces.
- `tools/src_build_gate.py` step 9: the object check over every `src/` object of every build (40 banned names,
  each defined by the vendored `libc.a` or `libstdc++.a`, the 25 stdout/stderr printers among them; the list itself
  checked against `libc.a` before any build).
- `tools/api_doc_tool.py`: four entries it had published under misread names corrected, and its coverage gate
  now fails an unreadable name.
- `test/serial_line_test.cpp` (31 cases); the register's HA-189 fixed in code, HA-190 and HA-191 registered.

## What the reviews changed

- **Review round:** three reviewers, 16 findings, 13 subjects after overlaps — all REAL. Twelve fixed (among them:
  step 9 had listed 7 of the 25 stdout printers `libc.a` defines; the doc tool's misread names); one — HA-189's
  dedicated two-task bench test — put to the team lead as a scope call.
- **Verification round:** a fresh verifier, 4 findings, all REAL, all fixed (an empty `[T] ... ` continuation after a
  newline — now RED in the suite; check-coverage's unreadable-name failure pinned; HA-190's Source wording; the
  512-byte pieces put to the team lead).
- **The coordinator's merge verification** (the record at the end of the progress log): every gate on the merged tree
  (2,353 tests; all seven builds; the ARM header gate, 215 headers; the compile-refusal harness 195/195); the whole
  `serial_line_test.cpp` under ASan + UBSan, 31/31; hand mutations SM1–SM7, **7 of 7 caught** (SM2 by the 4 GiB cap
  on the suite — the mutant that once froze the coordinator's laptop).

## Open — the team lead's calls

1. **The 512-byte pieces.** His ruling, by its letter, is one call per line whatever its length; a text longer than
   512 bytes through a team's own sink now goes in several whole calls. Every line shulib prints (at most 384
   bytes) is still one call. The alternative is the old single call, with HA-191's hang documented as the team's
   to avoid. Recorded in build-order's SER1 section and REMAINING-WORK 18c.
2. **HA-189's bench test:** (a) a station in Bench Tests (its 3 × 4 menu is full — a layout change or a station
   retired), or (b) K.7.3 accepted in its place, in writing.

## Hardware assumptions

HA-189 (two tasks printing at once — fixed in code; settled only at the bench), HA-190 (a task PROS deletes INSIDE
`ser_write_r` is believed to leave PROS's serial lock held — every later serial line of that boot would wait; reasoned,
not seen), HA-191 (one serial `write()` must fit PROS's 2047-byte serial stream). None is settled.

## Not done here (named, not silent)

- An ASan CI job would make the sanitizer run standing (`ci.yml` / `test/CMakeLists.txt`); today it is the
  coordinator's step.
- CAL1's hand-over: its shell's print goes through `hal::pros::serialWrapped("R3D", text)`, ONE call (CAL1 Part A2's).
