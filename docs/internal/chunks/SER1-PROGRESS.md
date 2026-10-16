# SER1 — progress log (live)

- **Chunk:** SER1 — the USB serial log keeps every line whole when two tasks write it (HA-189)
  (brief: [`SER1-serial-lines-whole.md`](SER1-serial-lines-whole.md), its amendments A1–A7 OVERRIDE its body)
- **Executor model:** opus
- Appended as the work goes. Every time in a section header is a `date` reading taken when the section
  was written, not an estimate written afterwards. **Nothing in this chunk has run on a robot.**

---

## Start

- **Started:** Thu Oct  8 06:14:12 PM EDT 2026 (`date`).
- **Tree at start:** `3b5a39c`, clean, in the git worktree `.claude/worktrees/ser1` (branch `ser1`) — NOT
  the main working tree.
- **Go:** the coordinator's workflow run, 2026-10-08, on the team lead's go for SER1 (re-ruled to ONE-CALL
  WRITES). Do not commit; no `pros` command; no `/dev/ttyACM*`. Mutation backups and probes under
  `.claude/coordinator/scratch/ser1/` (never `/tmp`, never the repository root).
- **In flight beside this chunk (not edited here):** MECH-A1 (`include/shulib/mechanism/`,
  `hal/device_provider.hpp`, `hal/pros/device_provider.hpp` — it constructs `ProsCharSink usb_{}`, which
  A3's default constructor must keep compiling unchanged — `hal/fake/`, `config/robot_config.hpp`, `robot/`,
  `test/compile_refusals/`, `test/CMakeLists.txt`, `ci.yml`), CAL1's brief amendment, MECH-A2.

### 18:14 — start
- Read `docs/internal/RESUMING.md` (the chunk loop, the standards, pace rules 1-8) and the brief in full
  (header, the re-check's A1–A7, §0–§6). This log created before any code.

### 18:15–18:20 — required reading, the census and the baseline *(`date` 18:20)*
- Read, in the tree at `3b5a39c`: `include/shulib/hal/pros/char_sink.hpp` (whole), `hal/char_sink.hpp`,
  `tools/src_build_gate.py` (whole: the header, `verdict()`, `find_nm()`, the self-test's plant helpers and
  cases 1-35), `test/pros_platform_adapter_test.cpp` (the old sink test), `test/CMakeLists.txt` (it GLOBS
  `test/*_test.cpp` — a new test file needs no CMake edit; MECH-A1 owns that file), `.github/workflows/ci.yml`
  (the guards; the CI PROS-free guard skips `hal/pros/`), `docs/api/pros-char_sink.md`, the register's HA-189
  (summary row :343, long form :2670-2699, the Held line :39), and the checker's probes in
  `.claude/coordinator/scratch/ser1-check/` (`serialline/proto.cpp` + `.su`, `hosttest/pending.cpp`,
  `hosttest/dgram.cpp`, `pros/ser_driver.dis`).
- **A2's census RE-COUNTED at `3b5a39c`** (`grep` for every stdio writer in `src/` and `include/shulib/`):
  exactly A2's 20 `printf` sites and 14 `std::fflush(stdout)` lines, at A2's line numbers; no `puts`,
  `putchar`, `fputs`, `std::cout`. `include/shulib/` has only the sink's own `fwrite`/`fflush` and
  `hal/pros/block_sink.hpp` :200/:209 (the SD card's FILE — allowed by A1). `src/programs/bench_r3a.cpp`
  :621-622 is the bench SD probe's `fwrite`/`fflush` on a card FILE (allowed by A1).
- **Baseline:** briefing regenerated (the new PROGRESS file changes its block), host build exit 0, suite
  **2253 test cases passed, 0 failed, 3 skipped** (2,783,363 assertions).

### 18:20–18:28 — the header, and a doc-tool defect it exposed *(`date` 18:28)*
- `include/shulib/hal/pros/char_sink.hpp` rewritten: `writeWhole()` (the A5 loop, injectable write for the
  tests), `PosixWrite` (the ONE `::write` call), `ProsCharSink()` (A3, STDOUT_FILENO, names no FILE),
  `ProsCharSink(std::FILE*)` (fileno + ONE fflush), `serialLine()` (A4: 256 bytes on the caller's stack,
  exactly one `\n`), `serialWrapped()` (the wrap A2 asks of a site whose line can pass the buffer).
- **DELTA — `serialWrapped()` is new (the brief names one function, `serialLine`).** A2 says a site whose
  line can exceed the buffer WRAPS, one call per piece, "like logWrapped"; the census has SIX such sites
  (the boot source's lines run to 511 chars, `kBootLineBytes` = 512; the refusal lines join a 63-char
  headline and a 191-char reason; the competition program's claw lines run to 255 + the tag; "shulib
  Drive"'s `kRunStatus` is one ~430-char *(408 — corrected at the review)* sentence; the X-drive's refusal joins a 191-char reason and a
  fixed sentence). One shared wrapper beside `serialLine()` (and available to teams) instead of six private
  copies. **And `serialLine()` itself CUTS (visibly, `...[cut]`), it cannot wrap:** `vsnprintf` never yields
  the bytes past its buffer, so a printf-style call can only wrap with a larger buffer (the stack A4 bounds)
  or the heap (the §6 landmine: the buffer is on the caller's stack, always). Every shulib site whose line
  can pass 256 bytes goes through `serialWrapped()`, which never cuts; `serialLine()`'s cut is the backstop
  for a team's over-long format, pinned by a test.
- **DELTA — the buffer is `kSerialLineBytes + 1`** (the `+ 1` is vsnprintf's NUL), so a line of exactly 256
  bytes WITH its `\n` is sent whole whether or not the format typed the `\n`; with a 256-byte buffer a
  255-char text from a format ending `\n` would have been cut while the same text without the `\n` was
  not. The pin `vsnprintf(buf, sizeof buf, …)` stands.
- **DELTA — `ProsCharSink(std::FILE*)` also requires a DESCRIPTOR** (`fileno(out) >= 0`): a FILE with no
  descriptor would make every write fail silently.
- **A doc-tool defect, fixed (`tools/api_doc_tool.py`):** the tool reads a declaration's NAME and KIND from
  the first `(` at depth zero, and `[[gnu::format(printf, 1, 2)]]`'s own `(` came first — `serialLine`
  rendered as `## ?` in `docs/api/pros-char_sink.md`, and check-coverage PASSED (it counted "?" as
  documented). Fixed by skipping a leading `[[...]]` list when reading the name and the kind (the rendered
  signature keeps it); the self-test's fixture gained a function and a constant behind a parenthesised
  attribute. **Mutation:** the skip removed → self-test FAILS (4 checks, the names read `?`) → restored → OK.
  No other header in the tree changed name (the regeneration touched only char_sink's page, the index and
  all-entities).

### 19:52 — RESUMED after the team lead's pause *(`date` Thu Oct  8 07:52:11 PM EDT 2026)*
- The previous run was STOPPED partway (the usage-limit pause, ~18:45). Read this log, `git status`, and
  `git diff -- src/ Makefile` (EMPTY — no gate self-test plant left in `src/` or the `Makefile`;
  `tools/src_build_gate.py` unmodified). In the tree: the header (above), the doc-tool fix and its regenerated
  pages, the briefing. Left in the scratch: the stack probe (`scratch/ser1/stack/`, shulib's frames
  measured — the banner still holds placeholders `SL_S`/`SL_F`/`SW`/`WR`) and libc's objects extracted for the
  disassembly; PROS's not yet. Nothing in `src/`, the gate, the tests or the register was started.
- Continuing from the stack measurement. Every result below is from a FRESH build (a killed run may have
  left a mutant binary linked).

### 19:55–20:10 — the fresh build, and the stack MEASURED *(`date` 20:10)*
- **Fresh host build** of the resumed tree (briefing regenerated first; configure + full build from
  source): exit 0; suite **2253 passed, 0 failed, 3 skipped** (2,783,369 assertions) — the old sink test
  (`pros_platform_adapter_test.cpp`) passes on the new header.
- **The stack, A4's numbers for kSerialLineBytes = 256, measured:** the probe
  (`scratch/ser1/stack/stack_probe.cpp`) compiled with the BUILD's own flags (common.mk's
  `-mfloat-abi=hard -Os -mthumb -ffunction-sections -funwind-tables`, plus the Makefile's `-Wformat=2`;
  the previous run had used the header gate's `softfp` — the frames came out identical) and
  `-fstack-usage`: `serialLine` 288 (+16 for the saved varargs registers: 304 from its prologue),
  `serialWrapped` 336 (+ its send lambda 16), `writeWhole` 32. The libraries' frames: a call-graph walk
  (`scratch/ser1/stack/graph.py`) over `objdump -dr` of 42 objects pulled from the vendored
  `libc.a`/`libm.a`/`libpros.a` — frame from each prologue (push ×4, vpush ×8, `sub sp`), edges from the
  call/branch relocations, a tail branch counted as a call (over-estimates only), plus what a listing
  cannot show: vfs's indirect `_write → ser_write_r`, and `ser_write_r`'s VARIABLE-LENGTH array
  (`sub sp, sp, r3` with r3 = (cobs_encode_measure + 8) & ~7: 264 bytes for a 256-byte line, 392 for a
  384-byte one). Results (below the caller): `%s` format 852, full-line write 868 → **≈ 0.87 KB**; `%f`
  **≈ 1.24 KB** (_dtoa_r → __pow5mult → __multiply → _Balloc → _calloc_r → _malloc_r → … the scheduler's
  resume — an upper bound); `serialWrapped` **≈ 0.92 KB**; the sink's write of a 384-byte Line **≈ 0.69 KB**.
  The checker's own estimates at 256 were 0.82/0.98/0.74 — mine are higher because the walk takes every
  branch; the banner states the walk's method and calls them upper bounds.
- **A banner claim corrected before it was written down:** the placeholder text said a task deleted
  mid-`%f` "can leave the heap lock held". PROS's `__malloc_lock` is `b.w rtos_suspend_all` (mlock.c.o) —
  the scheduler is suspended, no other task can run to delete it. The banner now says the three
  ALLOCATE (and how PROS locks the heap) — no hang claim. Also measured: FreeRTOS's context save
  (`FreeRTOS_SWI_Handler`, portASM.S.o: 8 + 56 + 4 + 260 FPU + 4 ≈ 332 bytes on the task's stack) — every
  task pays it at every instant; stated beside the numbers. TASK_STACK_DEPTH_DEFAULT is 0x2000 words
  (32 KB, `include/pros/rtos.h` :72), _MIN 0x200 (2 KB) — the banner says _MIN leaves too little for `%f`.

### 20:10–20:30 — every src/ stdout writer converted *(`date` 20:30)*
- **The census at the base (A2), re-counted before converting** (`grep` for printf/puts/putchar/fputs/
  cout/cerr and `fflush(stdout)` over `src/`): A2's 20 `printf` sites and 14 `std::fflush(stdout)` lines,
  at A2's line numbers, nothing else. A **baseline object scan** of the bench build before converting
  (`arm-none-eabi-nm -u` per object): `printf` in 10 objects, `puts` in `bench_r3a.cpp.o` (the :178
  `printf("%s\n")` — A2's named example, confirmed in the object), `_impure_ptr` in 9.
- **Converted — each site, how, and its LONGEST line (with its `\n`):**

  | Site (base line) | Now | Longest line |
  |---|---|---|
  | `main.cpp` :144 `[BLD] robot tables` | `serialLine` | 112 (`kRobotBeacon` ≤ 28) |
  | `boot_source.cpp` :52 (+ fflush :53) | `serialWrapped(tag, line)` | `[tag] ` + 511 (`kBootLineBytes` 512) → WRAPS |
  | `screen_paint.cpp` :108 (+ :110) | `serialWrapped(tag, {"REFUSED: ", headline, " -- ", why})` | 63 + 191 + 16 → WRAPS |
  | `state_screens.cpp` :30 (+ :31) | `serialWrapped("R3A", {…})` | 206 (Calibrate's AUTONOMOUS reason) — one piece; wraps if a caller's `why` grows |
  | `bench_r3a.cpp` :178 (+ :179) — `emit()` | `serialWrapped(nullptr, line)` (no tag: the line carries its own) | `emitf`'s 219; a direct `emit` of a longer literal wraps |
  | `bench_r3a.cpp` :3127, :3133 (+ :3136) | 6 × `serialLine` | 201 (63 chars of status bits) |
  | `competition_program.cpp` :267 (+ :268) — `logLine` before the provider | `serialWrapped(kTag, text)` | a refusal's text → WRAPS |
  | `competition_program.cpp` :632 (+ :638) | 5 × `serialLine` | 115 |
  | `competition_program.cpp` :781, :785 (+ :788) | `serialWrapped(kTag, ….text)` | `[CMP] ` + 255 (`kClawLineBytes` 256) → WRAPS |
  | `drive_program.cpp` :599 (+ :610) | 7 × `serialLine` + `serialWrapped(kTag, kRunStatus)` | `kRunStatus` ≈ 430 *(408, measured — corrected at the review, 2026-10-15)* → WRAPS |
  | `drive_program.cpp` :607 | `serialLine` | 199 |
  | `auton_tester_program.cpp` :439 (+ :445) | 5 × `serialLine` | 112 |
  | `xdrive_program.cpp` :225 (+ :228) | `serialWrapped(kTag, {"X-DRIVE REFUSED AT BOOT: ", why, " -- …"})` | 292 (a 191-char reason) → WRAPS |
  | `xdrive_program.cpp` :249, :252 (+ :255) | 3 × `serialLine` | 191 |
  | `calibrate_program.cpp` :193 (+ :194) — `emit()` | `serialWrapped("R3D", line)` | `emitf`'s 239 + 6; a longer line wraps |
  | `calibrate_program.cpp` :1044, :1055 (+ :1058) | 11 × `serialLine` | 194 |

  A multi-line `printf` became one call per line; a banner's leading blank line rides in the first
  line's call (`"\n[TAG] booted: …"` — one write, two whole lines; the bytes unchanged). The bytes on the
  serial are unchanged everywhere EXCEPT a line that used to pass 255 characters: it now arrives as
  `[TAG] ` pieces, the later ones `[TAG] ... ` — wrapped, never cut (the drive program's run status is the
  one such line printed on every boot).
- **Unused `<cstdio>` includes dropped** where the file now uses nothing from it (`main.cpp`,
  `boot_source.cpp`, `screen_paint.cpp`). Prose in the same files moved: `boot_source.hpp` (:42, :594 —
  "through printf"), `screen_paint.hpp` (the refusal's serial line now wraps), `state_screens.hpp`
  (`benchStateScreen`'s line), `competition_program.cpp`'s file banner (the "SERIAL has had the same
  exposure since COMP-b, unguarded" paragraph — §1.5 — now says what SER1 did and names HA-190).
- **All seven builds** (`make … -j8` from clean, a quick pass before the gate): exit 0, the same 8
  vendored `lv_obj_style.h` warnings as the baseline, nothing of ours; and **no object of any build**
  references `puts`, `printf`, `_impure_ptr` or any other A1 name (`nm -u` over every `bin/**/*.o`).
  (Lengths computed from the literals with every `%s` at its bound — status bits 63, the longest
  variant beacon of that program, a 191-character reason; `scratch/ser1/` holds no separate probe.)

### 20:30–20:45 — the src gate's step 9 (A1) *(`date` 20:45)*
- `tools/src_build_gate.py`: **step 9, ONE-CALL WRITES** — `STDIO_BANNED` (A1's 18 names, `_impure_ptr`
  included), `STDIO_EXEMPT_OBJECT` (`_pros_ld_timestamp.o`), `object_of()`, `built_objects()`,
  `stdio_problems()` (`arm-none-eabi-nm -A -u` over every `bin/**/*.o`, via the existing `find_nm()`;
  FAILS naming the object and the symbol, and FAILS when a src/ TU has no object — a check that read
  nothing would pass), called from `verdict()` after step 8; the header's step list (step 9's why, what it
  allows — fwrite/fflush/fopen/snprintf — and what it cannot see), the summary and PASS lines.
- **Self-test cases 36–39** (plants in `SRC_PLANT` as EXTERNAL functions, declared first): 36
  `printf("x\n")` → FAIL naming `bench_r3a.cpp.o` and `` `puts` ``; 37 `fflush(stdout)` → FAIL naming
  `_impure_ptr` and NOT `fflush`; 38 the negative control, `fwrite`+`fflush` to a `FILE*` PARAMETER →
  PASS; 39 (no extra build) 38's `bin/` with `bench_r3a.cpp.o` deleted → step 9 reports the missing object.
  **Trial-run 20:41 against real bench builds** (the module imported, cases 36–38): 36 FAIL naming
  `bin/programs/bench_r3a.cpp.o` + `puts`; 37 FAIL naming `_impure_ptr`; 38 PASS; the real bench build
  PASS. The full self-test runs below.

### 20:45–21:00 — the tests (A6) and the mutations, every one RUN and SEEN red *(`date` 21:00)*
- **`test/serial_line_test.cpp` (NEW, 24 cases)** — globbed by `test/CMakeLists.txt` (MECH-A1's file; NOT
  edited). A SOCK_DGRAM socketpair dup'd onto fd 1 (`StdoutCapture`, RAII; non-blocking by default so a
  test can never hang; nothing asserted while fd 1 is captured) counts write() calls as datagrams — the
  checker's `hosttest/dgram.cpp` probe made a fixture; the pending-bytes case is the checker's
  `hosttest/pending.cpp` (its second half; its first half, the no-newline `pread`, is the test A6
  REPLACED — the probe showed the fwrite+fflush mutant passes it). Cases: the default sink (one datagram
  per line, a 300-byte line whole, the text verbatim); default-constructible/nothrow/`sizeof` ≤ two
  pointers (no member buffer) + `ProsCharSink usb_{};` in `device_provider.hpp` exactly once (A3's pin);
  the pending "B" (→ `"C\n"`); the constructor's one flush (→ `"AC\n"`); null FILE / `fmemopen` FILE
  (no descriptor) → `PreconditionError`; `writeWhole()` through `WriteFake` (call-capped at 50): one call,
  no call for empty, the remainder on a 100-per-call device, stop on 0 and on −1 (also after a short
  write), a device claiming more than it was given; `serialLine()`: one call per line, one `\n` (never
  doubled; `"%s",""` and a null format → `"\n"`; a leading blank line rides in the call), the 255/256/257
  boundary (whole/whole/cut/cut), the 2,000-byte `%s` (exactly 256 bytes, marker last) — the case the
  coordinator runs under ASan; **two threads × 400 lines** through a BLOCKING capture with a 5 s send
  timeout and a reader thread (every line whole, each exactly once); `serialWrapped()`: short/parts/
  empty/null tag/16-char tag cap/empty tag; newlines; the 256/257 boundary; 1,500 chars of words (every
  piece ≤ 256, one `\n`, `[DRV] ... ` continuations, rejoins exactly, every break at a space); a
  PATTERNED 600-char word (moves whole to a fresh piece, then hard breaks); the strict carry room (a
  space at index 7 with tag `T`); no tag. Source pins: `char_sink.hpp` names no `pros/` header and no
  `pros::c::`; `serialLine`'s body has `char buf[kSerialLineBytes + 1];`, `std::vsnprintf(buf, sizeof buf,
  format, args)` and no `static`/`thread_local` (and `serialWrapped`'s `char piece[kSerialLineBytes];`);
  no `src/` source CALLS printf/puts/putchar/fputs/vprintf/fprintf/vfprintf/perror/putc/fputc or NAMES
  stdout/stderr/cout/cerr/clog (comments and literals stripped; ≥ 20 files walked); the bench `emit()`.
- **Two tests changed:** `test/competition_source_test.cpp`'s P14 pin of `logLine` (its body now
  `hal::pros::serialWrapped(kTag, text);` — the pin's bug-caught list says why); the old sink test in
  `pros_platform_adapter_test.cpp` renamed "unbuffered" (it passes unchanged); a comment in
  `pros_adapter_fence_test.cpp`. **Two expectations of mine were wrong and were corrected, not the code:**
  the boundary case counted 4 calls where the code (rightly) makes 5, and the long-word case expected
  the word hard-broken after "start" where the rule (the LAST space that leaves the rest room) moves the
  word to a fresh piece first — the behaviour kept, the test now pins it.
- **Mutations** (`scratch/ser1/mut.py` + `muts_ser1.json`; backups under `scratch/ser1/backup/`; per
  mutation: apply → touch every test TU whose depfile names the file → `cmake --build … --target
  shulib_tests/fast` (the doc gates would fail on the shifted lines; `/fast` skips them) and REQUIRE the
  link → run → restore byte-exact, hash-checked, mtime bumped → **rebuild the restored tree**).
  **A runner defect caught and fixed on the way:** the first full run did NOT rebuild after restoring, so
  S20–S22 (src/ text the host only reads: nothing to recompile) ran on S19's still-linked MUTANT binary
  and "went red" by crashing — that run is DISCARDED; and S1 went red only at compile (my `sizeof`
  static_assert, since the mutant added a member), so S1 was rewritten to the same size to reach the
  pending-bytes case A6 asks for; S14 SURVIVED because the long word was one repeated letter (a garbled
  copy of `w`s is still `w`s) — the word is now patterned. **The run of record (20:57–20:59), 22 of 22
  RED:**

  | # | Mutation | Red by |
  |---|---|---|
  | S1 | the sink back to fwrite+fflush on its FILE | the pending-bytes case |
  | S2 | the sink's write split: text, then "\n" | the default-sink datagram case |
  | S3 | the short-write loop re-sending from the start | the remainder case |
  | S4 | the loop continuing on 0 | the stop case (call cap reached) |
  | S5 | the loop continuing on −1 | the stop case |
  | S6 | the FILE constructor's fflush removed | the construction-flush case |
  | S7 | serialLine without the newline guarantee | 3 cases |
  | S8 | `kSerialLineBytes` = 4096 | the build: `static_assert(kSerialLineBytes <= 1024)` |
  | S9 | serialLine's buffer `static` | the two-thread case (**20 of 20 runs red; the clean tree 20 of 20 green**) and the pin |
  | S10 | the newline doubled for a format ending in one | the one-newline case |
  | S11 | a full 256-byte line with its newline cut anyway | the boundary case |
  | S12 | serialWrapped's carry room `<` → `<=` | the strict-room case |
  | S13 | the carry starting AT the space | the words and long-word cases |
  | S14 | the carried words copied back to front | the patterned long-word case |
  | S15 | breaking at the FIRST space | the words case (184 assertions) |
  | S16 | the "full" test one late | 4 cases |
  | S17 | an empty text sends nothing | the short-text case |
  | S18 | the tag not capped at 16 | the short-text case |
  | S19 | vsnprintf bounded past the buffer | the 2,000-byte case: `*** stack smashing detected ***`, abort (rc 134) — seen in a separate run |
  | S20 | bench `emit()` back to `printf("%s\n")` | the src/ pin and the emit pin |
  | S21 | main.cpp's [BLD] line back to printf | the src/ pin |
  | S22 | an `fflush(stdout)` back in state_screens.cpp | the src/ pin |

  The restored tree after the run: fresh build, the file green.

### 21:00–21:10 — the gate mutations on real ARM builds *(`date` 21:08)*
- `scratch/ser1/gate/cases.py` runs step 9's cases 36–39 (or one real bench verdict) from the CURRENT
  gate file in a fresh interpreter; `gate/gmut.py` applies each mutation (backup, exactly-one anchor,
  restore in a `finally`, hash-checked, mtime bumped). On the clean gate: cases 36–39 OK. **6 of 6 RED:**

  | # | Mutation | Red by |
  |---|---|---|
  | G1 | bench `emit()` back to `printf("%s\n")` | the real bench build: `bin/programs/bench_r3a.cpp.o references \`puts\`` (step 9) |
  | G2 | main.cpp's [BLD] line back to `std::printf` | the real bench build: `bin/main.cpp.o references \`printf\`` (it compiles: `main.h` brings stdio) |
  | G3 | `STDIO_BANNED` without `_impure_ptr` | case 37 (an `fflush(stdout)` PASSED) |
  | G4 | step 9's missing-object check removed | case 39 |
  | G5 | step 9 not called from `verdict()` | cases 36 and 37 |
  | G6 | the scan over DEFINED symbols (`-g --defined-only`) instead of `-u` | cases 36 and 37 |

### 21:08–21:21 — the documents (§1.5 as amended by A7) *(`date` 21:21)*
- **`include/shulib/diag/log_copy_sink.hpp`** (§1.5's :33-36, now :33-37 and :45-49): the copy's guard says
  PROS's C library does NOT serialize two tasks on one FILE* (HA-189); the PRIMARY half needs no guard since
  SER1 (one `write()` per line, PROS's mutex) and its one exposure is PROS's (HA-190).
  **`include/shulib/hal/pros/block_sink.hpp`** :45, :183-184: "ProsCharSink borrows stdout" → "borrows a
  descriptor (the serial's, fd 1)". `docs/api/` regenerated (char_sink, log_copy_sink, block_sink pages, the
  index, all-entities).
- **`docs/hardware-assumptions.md`:** the Held line loses HA-190..HA-191 (now rows); "7 of 174 settled"; the
  T4 Part 3 paragraph's unnumbered belief (§1.5's :129-130) now says it was numbered HA-189 and fixed by SER1;
  a SER1 paragraph after HA-189's ("With them, 174"); the index: HA-189's row names the fix and Station
  K.7.3, HA-190 and HA-191 rows; HA-189's long form gains *The fix* and *Settle (as fixed)* (its "nothing
  calls setvbuf" kept — A7: it stays true); HA-190's and HA-191's long forms (claim, source, confidence,
  settle, blast radius) after it. HA-191's 2047 re-read from the disassembly (`ser_driver_initialize` +0x38,
  `movw r0, #2047`, the `xStreamBufferGenericCreateStatic` size); HA-190's take/give offsets (+0x72/+0xa6)
  from `ser_write_r`'s.
- **`docs/changelog.md`:** "Fixed and added since 2.4 (2026-10-08, later)" at the top of `## API 2.4` —
  the fix, the additions, the `ProsCharSink(FILE*)` behaviour change, the programs' long lines now wrapped,
  what a team must do (and that step 9 binds `src/team/`).
- **Guide 17** (§1.5's :306-312): after the card copy's paragraph, serial's lines are whole since SER1 —
  and HA-190's case (the next driver control's first line would wait for ever, so the driver loop never
  starts; restart the program).
- **Guide 11** (A7: the rule for a team's own code in a GUIDE page): a new section "Printing your own lines:
  one line, one call" before the blackbox — a `cpp` listing quoted VERBATIM from a new compiled example,
  **`guide-11a`** in `test/guide_examples_test.cpp` (it sends its lines to /dev/null unless
  `SHULIB_GUIDE_PRINT` is set); the never-printf rule and its why; the three limits (2 KB, the stack, HA-190);
  the exact-signatures paragraph links `pros-char_sink.md`. `docs/guide/README.md`'s chapter-11 row says so.
- **`docs/cookbook/README.md`** :101 told a reader to "put a `printf`" between two steps — now "a printed
  line … (Print with `serialLine`, never `printf`)" (found by the prose-review grep, below).
- **The worksheet, Station K** (`chunks/R3a-BENCH-WORKSHEET.md`): **K.7.3** (every line whole around every
  Timed-Run mode change; HA-190's stop) and **K.7.4** (`pros terminal` connected to a RUNNING program once);
  "What to send back" item 6 names them. **DELTA from §1.5:** the brief's two tasks printing NUMBERED lines
  for 60 s needs a bench program that no build carries — K.7.3 reads the competition program's own lines
  instead (no sequence numbers), said in the step and in HA-189's long form; the numbered test is owed
  (for the coordinator: a bench program, i.e. a Makefile/gate change, outside SER1's scope).
  *[Corrected at the review, 2026-10-15: that reason is WRONG — Bench Tests is a station framework, and a
  station needs no Makefile or gate change; the real cost is its full 3 × 4 menu. And the delta was the team
  lead's to rule on, not this builder's. The disposition below.]*
- **`build-order.md`** (SER1's own section: BUILT, not yet verified or committed) and **REMAINING-WORK** row
  18c (the same). The roadmap's "You are here", HANDOFF and ORIENTATION carry no SER1 line — the pointer
  moves are the coordinator's at the close. The briefing regenerated.

### 00:24 (2026-10-15) — RESUMED after the team lead's night pause *(`date` Thu Oct 15 12:24:13 AM EDT 2026)*
- The previous run was STOPPED at 21:23 on 2026-10-08 (the team lead paused all work for the night;
  resumed 2026-10-15). Read this log, `git status`, and `git diff -- src/ Makefile`.
- **A gate self-test plant WAS LEFT in `src/`:** `src/build_selection.hpp` :135 read
  `inline constexpr Program kProgram = Program::BenchTests;` under the tank-drive beacon — the src gate's
  self-test case 21 (`tools/src_build_gate.py` :2143-2161), written at 21:22:23, the minute the run was
  killed (its `_replace` never reached its restore). SER1 does not edit `build_selection.hpp` at all, so it
  was RESTORED with `git checkout` (the planted copy kept as
  `scratch/ser1/backup/planted-case21__build_selection.hpp`) and its timestamp bumped. The `Makefile` diff is
  EMPTY (its 21:20 timestamp is an earlier self-test case's restore); every other self-test plant target
  (`robot_config.hpp`, `test/build_selection_test.cpp`, `test/vendor/doctest.h`, `test/CMakeLists.txt`,
  `src/robots/*`, `lv_obj_style.h`, `test/chassis_table_test.cpp`) is unmodified; the `src/` diffs left are
  SER1's own conversions (read in full: `main.cpp`, `boot_source.{hpp,cpp}`, `screen_paint.{hpp,cpp}`,
  `state_screens.{hpp,cpp}`, the six programs) — no plant residue (`grep` for plant markers: none).
- Every mutation backup in `scratch/ser1/backup/` compared to the tree: the `src/` and gate ones are
  byte-identical to the files (restored); `char_sink.hpp` differs from its pre-mutation backups ONLY in
  the banner's stdio-writers paragraph (:38-44), a comment edited at 21:21 during the documents pass (A3's
  §6 rewording: the daemon's banners go through stdout's buffer, in which no shulib text waits after SER1;
  both end in "\n\n\n").
- **The killed run had reached the final gate pass** (the self-test, at case 21). Still owed: a fresh
  build from source, the prose-review greps (logged), every gate's final line, the report. Continuing.

### 00:24–00:33 — the fresh build, and what the killed run left undone *(`date` 00:33)*
- **Fresh host build from source** (briefing regenerated; `build/test` DELETED, configured and built anew):
  the build FAILED at the doc gates — `docs/api/pros-char_sink.md` was stale by two lines: the banner edit
  of 21:21 (above) moved every declaration after it, and the killed run never regenerated. `api_doc_tool.py
  generate` → rebuilt: exit 0, 0 warnings. Briefing regenerated after the suite binary existed. Suite
  **2278 passed, 0 failed, 3 skipped** (2,784,362 assertions) = the base's 2253 + this file's 24 +
  `guide-11a`; `test/serial_line_test.cpp` alone **24 / 24** (816 assertions).
- **An over-claim in the register, caught by the regenerated briefing:** it read "174 registered, **8**
  settled" where the register says 7. `tools/briefing_status.py` :162 counts every summary ROW that
  contains the word "settled", and HA-189's new row ended "— settled only at the bench". Reworded to
  "— unconfirmed until the bench"; the count is 7 again (`grep -c "^| HA-.*settled"` → 7, the base's
  count). *(For the coordinator: the counter is a word match, so any row that says "not settled" counts —
  a tool change, outside SER1.)*
- **The prose review** (pace rule 4) — run before this entry was written, every hit read:
  - `grep -rn -i "printf\|fflush\|puts(\|std::cout\|setvbuf"` over `README.md` and `docs/` (live docs:
    not `docs/api/`, `chunks/`, `transcripts/`): the hits are SER1's own new text (changelog, guide 11,
    cookbook, register), the SD card's fflush (HA-122 and others — the card, correct), the master plan's
    "raw `std::cout`" (legacy code, history), and **HA-189's long form, which still said the sink "does
    fwrite + fflush" (present tense) and named the two REJECTED fixes as the settle's example** — now
    "did … until SER1", and "SER1 chose neither: one direct `write()` per line, below".
  - `grep -rn -i "ProsCharSink\|char_sink\|interleave\|atomic per call\|lines never\|unguarded"`: the
    `char_sink.hpp:11` citations in `build-order.md` :755 and `PROJECT-BRIEFING.md` :701 are the R1 split's
    HISTORY (where "R1 owns this" sat in 2026-08) — not rewritten; guide 11 :124 ("lines never carry it")
    is another subject; the rest are SER1's.
  - `grep -rn "printf\|fflush\|stdout"` over `src/`, `include/shulib/`, `test/*.hpp` (comments, the
    code's own prose): **`include/shulib/hal/char_sink.hpp` was stale twice** — "R1 adds the on-robot
    stdout adapter" and, in a `///` comment, "the concrete sinks DO (a FILE* on the robot, …)" — now "R1
    added the on-robot adapter (… the USB serial's descriptor, one write() per line since SER1)" and "a
    file descriptor on the robot"; `docs/api/` regenerated (`char_sink.md`). **NOT edited — MECH-A1's
    file:** `include/shulib/hal/pros/device_provider.hpp` :424 says the TermSink is "over the USB serial
    (`ProsCharSink`, stdout)" — still true of the descriptor (fd 1 is stdout's), so left for MECH-A1 /
    the coordinator to word "(`ProsCharSink`, fd 1)" if wanted. `calibrate_program.cpp` :659 and
    `bench_r3a.cpp` :626 are the card's fflush (allowed); `shul2_format.hpp` :11/:165 speak of PROS's
    COBS framing of stdout (true).
  - `grep -rn "THIS VERSION of\|run-status\|run status"` over the worksheet, guide and cookbook (the one
    boot line SER1 changes on every boot, "shulib Drive"'s 408-character run status, now TWO pieces):
    worksheet :562 (Station F's "on serial only" list) now says the run status is two lines, the second
    beginning `[DRV] ... `; :1358 ("Its run-status line still ends `Its claws …`") stays true of the last
    piece; :533 is the SCREEN's row (unchanged).
  - `grep -rn -i "step 8\|eight steps\|nine steps\|step 9"` and the gate's name over the docs: no live doc
    counts the src gate's steps (its header does, and lists step 9); `RESUMING.md` points to the header.
  - `grep -rn -i "garbled\|torn\|two tasks\|serial.*whole"`: only SER1's own text and unrelated "torn"
    (a torn card file, the changelog's T4 entry).
  - Also reflowed one over-long line in guide 17's new paragraph (prose unchanged).

### 00:33–00:52 — every gate, on the final tree *(`date` Thu Oct 15 12:52 AM EDT 2026)*
- Host: fresh configure + build (exit 0, 0 warnings), the doc gates inside it green; suite **2278 passed,
  0 failed, 3 skipped** (2,784,362 assertions).
- `api_doc_tool.py`: check-fresh exit 0; check-examples "525 quoted lines, all verbatim (6 source files);
  6 listings of whole-run files, each one unbroken run"; self-test "OK"; check-coverage "3673 public
  entities across 170 headers, all documented"; check-removability "no public doc references
  docs/internal/".
- `doc_staleness_audit.py`: "clean (29 live docs, 0 numeric claims checked)"; its self-test "OK (10
  detector/filter cases)". `briefing_status.py check`: "generated block current"; self-test "OK (24 cases)".
- CI greps: "library is PROS-free (all of include/shulib except hal/pros/)"; "layering holds: core is
  sim-free". ARM header gate: "TU includes 207 headers" → "ARM compile gate: CLEAN (all v2 headers)"
  (the generated TU in `scratch/ser1/all_headers.cpp`, not `/tmp`).
- `src_build_gate.py self-test` (00:33–00:48): "OK (218 detector cases, each against a real build)" —
  cases 36–39 included; the tree after it: `src/build_selection.hpp` and the `Makefile` unmodified (no
  plant left this time). `src_build_gate.py check` (00:49–00:51): all seven builds exit 0, 11/11 TUs,
  8 vendor-ignored, 0 disallowed, "program and stdout printers checked"; "src build gate: PASS (… no src/
  object writes through stdout's FILE)".
- **Mutations not re-run on resumption** (finished work, not redone): the run of record (20:57–20:59,
  22/22 red, each on a freshly linked binary, backups hash-restored) and the gate run (6/6 red) stand —
  the only change to `char_sink.hpp` since is the banner comment at :38-44, which no mutation's anchor
  touches; `hal/char_sink.hpp`'s edit is two comments.
- **Still the coordinator's:** `serialLine()`'s 2,000-byte `%s` case under `-fsanitize=address`
  (`serial_line_test.cpp`, "a 2,000-byte %s sends exactly kSerialLineBytes"); the verification; the
  COMPLETED record, the roadmap/HANDOFF pointer moves and the commit.

---

## SER1 review findings and their disposition

### 10:01 (2026-10-15) — three reviewers' findings, received *(`date` Thu Oct 15 10:01:17 AM EDT 2026)*
- The builder again, on the coordinator's workflow run: three reviewers (correctness, tests, docs) returned 16
  findings; deduplicated to 13 subjects (correctness #1 = tests #6, the stdout printers step 9 misses;
  correctness #2 = docs #11, HA-191's containment line; tests #9 = docs #15, the doc tool's `?` name).
  Each is re-checked HERE before it is fixed or answered — the reviewers' probes are read, not trusted.
- Tree before any change: as the 00:52 entry left it (no file touched since but an ignored `__pycache__`;
  no self-test plant in `src/` or the `Makefile`; `git diff -- Makefile src/build_selection.hpp` empty).

### 10:01–10:16 — the header, its tests, and every mutation re-run *(`date` 10:16)*
- **Re-checked first (`scratch/ser1/fix3/recheck_h1_h5.log`):** the reviewers' H1–H5 mutants run through
  `mut.py` on the tree as it stood — all five SURVIVED the 24 cases (fresh link, 8 TUs recompiled, restored
  hash-checked). The findings are real.
- **`include/shulib/hal/pros/char_sink.hpp`:**
  - **HA-191's root, not its wording (correctness #2 / docs #11):** `ProsCharSink::write()` offered a text
    of ANY length to one `write()` — the reviewer's 60,000-byte fuzz arrived as one datagram — and on the
    brain one frame over ~2,035 bytes waits for ever holding PROS's mutex, while `ser_write_r` copies the
    frame onto the writer's stack (≈ its own length again). NEW `kSerialWriteMaxBytes` = **512**:
    `writeWhole()` offers each call at most that, so no call this header makes can outgrow PROS's stream,
    and the sink's stack is bounded whatever the text. 512, not 1024: above every line shulib prints
    (`Line::kCapacity` 384 — still ONE call, pinned by a `static_assert` in the test), and the smaller
    ceiling keeps the sink's worst case at the size of a full serialLine. `static_assert`s:
    `kSerialWriteMaxBytes <= 1024`, `kSerialLineBytes <= kSerialWriteMaxBytes`. A device claiming more than
    a call was OFFERED now counts as what it was offered (the old "claims more → the whole text was sent"
    would, with pieces, have ended the loop with later pieces never offered). **DELTA from the ruling's
    letter, not its intent:** "one line, one call" holds for every line up to 512 bytes; a longer text
    through a team's own sink is no longer one call — it was a hang before.
  - **serialWrapped's fill made exactly greedy (found while pinning H4/H5):** a FULL piece followed by a
    space now breaks AT that space (dropped). Before, the space started the next piece (`[T] ...  next`,
    a doubled space) or the scan carried the last word — which fit — to the next piece; and that leading
    space was the only way a continuation could begin with a space, i.e. the only case H5 (`k >= start`)
    changes. With the fix H5 is EQUIVALENT (proof in the disposition below). Frame 336 → 344 bytes.
    *[Corrected 10:40: equivalent only with a TAG — with none, `k >= 0` never ends the scan and it reads
    below the buffer; RED under ASan. The disposition, item 4.]* *[Corrected again 15:50 (the verification
    round): NOT equivalent even with a tag — a DOUBLE space after a full piece leaves its second space at
    the continuation's first character, and H5 breaks there, sending an empty `[T] ... `. H5 is now RED in the
    suite. The verification round's item 1.]*
  - Docs in the same header: serialLine's doc says a format vsnprintf REFUSES sends an empty line (never
    the buffer's stale bytes); the CONTRACT paragraph and both LIMITS bullets say "reasoned … not seen"
    (docs #13); the team rule's gate sentence names what step 9 checks and points at what it cannot see
    (correctness #1); the stack heading says "read from the compiled code; upper bounds" (docs #13).
- **The stack, re-walked (`scratch/ser1/fix3/stack/`):** the probe recompiled from the CURRENT header with
  the build's flags and `-fstack-usage` (serialLine 288, serialWrapped 344, writeWhole 32 — the old two
  unchanged), the same 44 objects, `graph.py` with `ser_write_r`'s VLA at 264/392/520 for a 256/384/512-byte
  frame: the sink's write of a 384-byte Line 692 bytes (the banner's 0.69 KB, reproduced), of a full
  512-byte piece **820 → "at most ≈ 0.82 KB"**; serialWrapped 924 → still ≈ 0.92 KB.
- **`test/serial_line_test.cpp` — 24 → 30 cases:** the sink's pieces (a 384-byte line ONE call, 512 one, 513
  two, a patterned 1,500 bytes as 512/512/476 in order); writeWhole's ceiling (fake: 1,200 → 512/512/176;
  exactly 512 → one; a 100-per-call device → 12 calls, none over 512, every byte once); the liar across
  pieces (claims 600 on each of three pieces → 1,200, the third offered); **K3** (H2): `serialLine("[T] %ls",
  L"é")` → exactly `"\n"`; **K1** (H1): `run(300) + "\nab"` → 3 calls, the last `[T] ab`, and `"\nabcd\nlast"`
  → `[T] last` sent; **the greedy oracle** (H4): `greedyWrap()`, an independent word wrap sharing no code
  with serialWrapped, against 100 generated texts (a fixed LCG; word lengths 1–14, every fifth 1–60) under
  tags "", "T", "DRV" and a 16-char tag — every piece EXACTLY equal — plus "shulib Drive"'s 408-character
  run status: exactly 2 pieces, the first over 240 bytes; the old words case now compares exactly too;
  **a space right after a full piece** (the fix): `run(251) + " next"`, `run(245) + " abcde next"` and the
  review's `251 x, " ", 246 y, " z"` (no empty piece, no leading space, the fitting word kept). The host
  source pin grew the same names as step 9 (`iprintf`, `viprintf`, `fiprintf`, `putchar_unlocked`,
  `wprintf`, `vwprintf`, `putwchar`, `putwchar_unlocked`, `psignal`; `stdin`, `wcout`, `wcerr`, `wclog`,
  `__sf`). The 2,000-byte case's comment and the one-newline case now say the coordinator's ASan run is
  the WHOLE file (tests #8).
- **Every mutation RUN on a fresh link, restored hash-checked (`fix3/muts_review.log`, `muts_s1.log`) — 31
  of 33 RED in the suite; the two survivors (H3, H5) RED under ASan:** *[15:50: H5 is RED in the suite since
  the verification round's double-space case (its item 1); H3 is still ASan-only.]*

  | # | Mutation | Result |
  |---|---|---|
  | S1–S22 | the run of record's 22, re-run on the final header (S3's anchor now `ask`; S1's re-anchored past the new `write()` doc) | all RED (S8 at compile) |
  | H1 | the newline resets `len` but not `start` | RED — K1 |
  | H2 | `formatted < 0` guard removed | RED — K3 |
  | H3 | `n == 0 ||` removed (reads `buf[-1]`) | SURVIVED the suite — RED under ASan, below |
  | H4 | the scan's `break` removed (earliest fitting space) | RED — the greedy oracle (and the run status: 31 pieces) |
  | H5 | `k >= start` | SURVIVED the suite — RED under ASan (no tag: a read below the buffer; the disposition, item 4) |
  | F1 | the full-piece-then-space break removed (the pre-review code) | RED — 3 cases, 36 assertions |
  | C1 | the ceiling removed | RED — the sink's pieces, the ceiling case |
  | C2 | the ceiling one short (511) | RED |
  | C3 | an over-claim counted as the whole text | RED — the liar across pieces |
  | C4 | `kSerialWriteMaxBytes` = 4096 | RED at compile (`static_assert`) |
  | C5 | the sink's `write()` as one raw `::write` of any length | RED — the sink's pieces |

  Restored tree: fresh build, the file 30/30.
- **H3 under ASan (tests #8):** `fix3/asan/build.sh` builds `test_main.cpp` + the WHOLE
  `serial_line_test.cpp` with `-fsanitize=address -fno-omit-frame-pointer` and the suite's warning flags
  (`-Werror`). Clean header: **30/30, no ASan report** (this covers the 2,000-byte `%s` case the coordinator
  owes, and every other case). H3 applied (backup `scratch/ser1/backup/H3asan__char_sink.hpp`, restored
  hash-checked, mtime bumped): **`AddressSanitizer: stack-buffer-overflow … in serialLine … char_sink.hpp`**,
  rc 1. The coordinator's owed run is now the WHOLE file under ASan (the recipe above), not one case.

### 10:05–10:16 — the doc tool: the `?` class, and four names it had already published wrong *(`date` 10:16)*
- **Re-checked (tests #9 / docs #15), and the class is wider than `?`:** a dump of every name
  `build_pages()` reads (3,673 at the time) against "an identifier, `~identifier`, `name<args>` or an
  operator" found **four misread names already PUBLISHED in `docs/api/`**, none of them SER1's, all counted
  as documented: `MenuButton::1]` and `MenuLine::1]` (`auton_menu.hpp` :139, :157 — `char label[ui::kMediumCols
  + 1]`: a bound with a space), `Wrapped::1]` (`calibrate_panel.hpp` :503 — `char line[Lines][Cols + 1]`), and
  `Handler::manipulation::MechanismOutcome`, kinded "function" (`command_registry.hpp` :81 —
  `manipulation::MechanismOutcome (*fn)(…) = nullptr`, a pointer-to-function FIELD named after its return
  type). A check for `?` alone would have passed all four.
- **Fixed at the root (`tools/api_doc_tool.py`):** `coverage_problems()` (shared by `check-coverage` and the
  self-test) reports every entity whose name fails `READABLE_NAME`, and `check-coverage` FAILS on any
  ("UNREADABLE NAME", the declaration printed); the reader learned the two shapes — `_bare_identifier()`
  strips every `[...]` group before taking the last token, and `POINTER_DECLARATOR` (`R (*fn)(…)`, `(&fn)`,
  `(Owner::*fn)`) names the field and kinds it a field in `_member_name()` and `_looks_like_function()`.
  The fixture gained `struct Declarators` (a spaced bound, two bounds, a function-pointer field); the
  self-test plants a `?` member, a `1]` member and a `?` namespace-scope entity and requires each reported.
  `docs/api/` regenerated: the four now read `label`, `text`, `line`, `fn` (field) — new anchors; nothing
  outside the generated pages linked the old ones (grep). 3,674 entities (the new `kSerialWriteMaxBytes`).
  **For the coordinator:** `docs/api/calibrate_panel.md` changed (its `Wrapped::line` row) while CAL1-A1
  edits `calibration/` — a merge conflict there is resolved by regenerating.
- **Mutations (`fix3/doctool/dmut.py`, backups under `scratch/ser1/backup/D*__api_doc_tool.py`, each
  restored hash-checked) — 7 of 7 RED:** D1 the report never made → self-test ("planted … not all
  reported: []"); D2 the bracket strip removed → self-test + `check-coverage` FAILS naming the three `1]`;
  D3 the pointer name removed → self-test + `check-coverage` FAILS on `Handler::manipulation::MechanismOutcome`;
  D4 the pointer kind removed → self-test ("…is a field: function"); D5 `READABLE_NAME` accepting anything →
  self-test; D6 D2+D3 together → `check-coverage` names all FOUR shipped misreads; D7 SER1's first fix (the
  `[[...]]` skip) removed → `check-coverage` FAILS on `'?'` at `char_sink.hpp` :232 — the original defect
  can no longer pass silently.

### 10:16–10:44 — the src gate's step 9, widened and pinned; its mutations on real ARM builds *(`date` 10:44)*
- **Re-checked first (`scratch/ser1/fix3/gp/forms.py`, 33 one-function probes — 31 compile; the
  headers declare no `putwchar_unlocked` or `_global_impure_ptr` — compiled with the build's flags,
  `nm -u` each):** `printf` with arguments → `printf` alone; `viprintf`, `iprintf`, `vprintf`,
  `putchar_unlocked`, `wprintf`, `vwprintf`, `putwchar`, `psignal`, `perror` → their own names alone;
  `std::cout`/`cerr`/`clog`/`wcout`/`wcerr`/`wclog` → `_ZSt4cout` … `_ZSt5wclog`; `_printf_r(_REENT, …)` →
  `_printf_r` + `_impure_ptr`; `fputs(…, stdout)` → `fputc` + `_impure_ptr`; `&__sf[1]` → `__sf` and NO
  `_impure_ptr`; `fdopen(1, "w")` → `fdopen`; `dprintf(1, …)` → `dprintf`; `assert` → `__assert_func`;
  `getchar` → `getchar`. And from `firmware/libc.a`'s own symbol table, the family of printers that write
  stdout/stderr without naming it: **25** (`printf`, `iprintf`, `vprintf`, `viprintf`, `puts`, `putchar`,
  `putchar_unlocked`, `wprintf`, `vwprintf`, `putwchar`, `putwchar_unlocked`, `perror`, `psignal` and the
  12 `_r` forms libc.a defines); A1's list held 7 of them.
- **`tools/src_build_gate.py`:** `STDIO_BANNED` 18 → 40 (the 18 missing family members, the wide streams,
  `__sf`); `STDIO_IMPLICIT_WRITER`, `VENDORED_LIBC`, `_defined_symbols()`, `toolchain_libstdcxx()` (common.mk's
  own `MFLAGS` read, never typed twice; `-print-file-name=libstdc++.a`), `stdio_list_problems()` and
  `_check_with()`; `cmd_check` runs the list check once before any build (`[stdio-list]`); the problem text
  names `__sf`; the summary says "stdio printers"; the PASS line and step 9's header rewritten (four groups;
  what it does NOT see, longer and exact). Self-test **case 40** (no build — the list against libc.a and
  libstdc++.a, a gap and a typo REPORTED, and `check` running it, via `_check_with()` with `viprintf` dropped
  and `VARIANTS` empty) and **case 41** (one real bench build: nine forms, each FAILS by name, no warning).
  The plant compiled first on its own with the build's flags and `-Werror`: clean, the nine names in `nm -u`.
- **Cases 36–41 from the gate's OWN code** (`fix3/gate/cases2.py` execs the self-test's SER1 section from the
  current file — no copy of the cases): clean gate, real builds, **24 checks OK** (1m40s); `src/` left with
  no plant (`git diff` of `bench_r3a.cpp` = SER1's own lines).
- **Gate mutations (`fix3/gate/gmut2.py`; backups `scratch/ser1/backup/review-G*__*`; each restored in a
  `finally`, hash-checked, mtime bumped) — 14 of 14 RED:**

  | # | Mutation | Red by |
  |---|---|---|
  | GM1 | `STDIO_BANNED` cut to `{puts, _impure_ptr}` (the review's) | 40 (18 gaps) and 41 — the nine-form build reported NOTHING: the blindness itself |
  | GM2 | `putchar_unlocked` dropped | 40 (the gap, by name) and 41 |
  | GM3 | `_ZSt5wcout` misspelt `_ZSt5wcoutt` | 40 ("a misspelt name bans nothing") and 41 |
  | GM4 | `__sf` dropped | 41 |
  | GM5 | `STDIO_IMPLICIT_WRITER` narrowed (no `putchar_unlocked`) | 40 (the family read short) |
  | GM6 | `check`'s `[stdio-list]` line dropped | 40 (`check` with a gapped list did not name it) |
  | GM7 | `printf` dropped | 40 and 41 (`printf` with arguments no longer reported) |
  | GM8 | `_ZSt4cout` dropped | 41 |
  | G3–G6 | the run of record's four, re-run on the final gate | cases 37, 39, 36/37, 36/37 |
  | G1, G2 | bench `emit()` / main.cpp's `[BLD]` line back to printf — a real bench verdict | step 9 (`puts`, `printf`) |

  Restored gate: cases 36–41, 24 checks OK.

### 10:44–10:48 — the last edits, regenerated, and every gate but the src gate's *(`date` 10:48)*
- Applied after the mutation runs (they back up and read these files): `log_copy_sink.hpp` (HA-189 "reasoned
  … not yet seen", HA-190 "believed", and the copy ENDING with a blocked primary), `drive_program.cpp` (408),
  the gate's step-9 call comment. `docs/api/` regenerated; briefing regenerated before the build and after the
  suite existed. **Fresh host build** (`build/test` deleted, configured, built with its doc gates): exit 0,
  **0 warnings**; suite **2284 passed, 0 failed, 3 skipped** (2,785,212 assertions) = 2278 + the six new
  cases; `serial_line_test.cpp` **30/30** (1,664 assertions).
- `fix3/gates.sh` (`final-gates.log`): api_doc_tool self-test OK; check-coverage "3674 public entities across
  170 headers, all documented"; check-fresh 0; check-examples "525 quoted lines, all verbatim (6 source
  files); 6 listings of whole-run files, each one unbroken run"; check-removability "no public doc references
  docs/internal/"; briefing self-test OK (24) and "generated block current"; staleness self-test OK (10) and
  "clean (29 live docs …)"; CI greps "library is PROS-free …" and "layering holds: core is sim-free"; ARM
  header gate "TU includes 207 headers" → "ARM compile gate: CLEAN (all v2 headers)".
- **The prose review (pace rule 4), run before this entry — every hit read:** `grep` over `README.md`,
  `docs/`, `include/`, `src/`, `tools/` for "stdout's printers|stdout printers|writes through stdout's FILE|
  reaches for stdout" → only the gate's own step-9 call comment (fixed, above); "settled only here|SER1's
  record names|measured 2026-10-08|waits for ever (HA-191)|is never released" → guide 17 :323 only, inside
  its "also believed and not seen" sentence (true as hedged); the four OLD anchors and names
  (`wrapped-1`, `menubutton-1`, `menuline-1`, `handler-manipulation-mechanismoutcome`, `MenuButton::1]`,
  `Handler::manipulation`) → only this log; "2 KB|2,035|2047" in the guide, cookbook, changelog, README →
  SER1's own sentences, each now consistent with the 512 ceiling; "1024|384 bytes per call|one write of any"
  in the register, changelog, guide, build-order, REMAINING-WORK, briefing → the rewritten HA-191 and the
  changelog's before/after; "check-coverage|coverage gate" in the live docs → roadmap :1940 and
  PROJECT-BRIEFING :260 ("fails naming any public entity with no documentation" — still true, now also
  unreadable names; shared documents, not reworded here), `docs-publishing.md`'s what-to-do table GAINED the
  `UNREADABLE NAME` row; "last space|broken at spaces|LAST space" → the changelog's "broken at spaces"
  (true), three unrelated panels' wrappers, the test's long-word comment (true); "step 9|src_build_gate" →
  the changelog, build-order, the register — all SER1's, all current.

### 10:48–11:06 — the src build gate, on the final tree *(`date` 11:06)*
- `src_build_gate.py self-test` (10:48–11:03): **"OK (235 detector cases, each against a real build)"** — 218 +
  case 40's 6 + case 41's 11. `check` (11:03–11:06): `[stdio-list] 40 banned names, every one defined by the
  vendored libc.a or libstdc++.a, and all 25 stdout/stderr printers libc.a defines among them`; all seven
  builds exit 0, 11/11 TUs, 8 vendor-ignored, **0 disallowed**, "program and stdio printers checked"; **"src
  build gate: PASS (… no src/ object names stdio's FILEs or calls a stdout/stderr printer)"**. After it:
  `git diff -- Makefile src/build_selection.hpp` empty (no plant left).

### The disposition, finding by finding *(`date` 11:06)*
Each was re-checked here before anything changed (the evidence above and in `scratch/ser1/fix3/`). All 13
subjects are REAL; 12 are fixed in this tree; one (10, below) is a scope decision put to the team lead.

1. **Step 9 missed stdout printers (correctness #1, tests #6) — REAL, FIXED.** Re-checked with my own probe
   (`fix3/gp/forms.py`, the build's flags): `printf` with arguments → `printf` only; `viprintf`,
   `putchar_unlocked`, `wprintf`, `vwprintf`, `putwchar`, `psignal` → their own names only; `std::wcout` →
   `_ZSt5wcout`; `&__sf[1]` → `__sf` with no `_impure_ptr`. Read from `firmware/libc.a`'s own symbol table, the
   family of printers that write stdout/stderr without naming it has **25 members; A1's list held 7** (`nm`
   with the A1-only list: 18 reported gaps). Fixed: `STDIO_BANNED` = A1's 18 + those 18 + `_ZSt5wcout/wcerr/
   wclog` + `__sf` (40); `STDIO_IMPLICIT_WRITER` (the family as a pattern) and `stdio_list_problems()` check
   the LIST on every `check`, before any build ([stdio-list]): every family member libc.a defines must be in
   it, every name in it must be defined by libc.a or the toolchain's libstdc++.a (a misspelt name bans
   nothing). Self-test **case 40** (no build): the real list clean; the family found; a list without
   `viprintf` and a misspelt `_ZSt5wcoutt` each REPORTED by name; and `check` itself runs it (cmd_check with
   `viprintf` dropped and no builds FAILS on `[stdio-list] viprintf:`). **Case 41** (one real bench build):
   `printf` with arguments, `std::cout`, `viprintf`, `putchar_unlocked`, `std::wprintf`, `std::wcout`,
   `putwchar`, `psignal`, `&__sf[1]` — each FAILS by its own name, and the plants raise no warning (so the
   case proves step 9, not the warning policy). The texts that over-claimed now say what it checks: the PASS
   line ("no src/ object names stdio's FILEs or calls a stdout/stderr printer"), the header's step 9 (its
   four groups, and a longer "What it does NOT see": `fdopen(1, …)`, a stdout FILE* from outside src/,
   stdin's readers, `__assert_func`/terminate on the way to a crash, the vendored libraries' own stdio), the
   banner's team rule, guide 11, the changelog, HA-189. Not banned, on purpose: the wide FILE-taking forms
   (`fwprintf`, `fputws` …) — aimed at stdout they need `_impure_ptr` or `__sf`, which are banned.
2. **HA-191's containment wrong for the sink (correctness #2, docs #11) — REAL, FIXED AT THE ROOT** (the
   ceiling, 10:01–10:16 above) and the register's Source/Settle/Blast radius rewritten to name what bounds
   each path (`writeWhole()`'s 512 for every call; 256 for serialLine/serialWrapped; `Line::kCapacity` 384
   pinned ≤ 512).
3. **H1 (tests #3) — REAL, KILLED** (K1).
4. **H4/H5 (tests #4) — REAL; H4 KILLED** (the greedy oracle — exact pieces against an independent wrap —
   and the run status's exact 2 pieces). **H5 survives the suite and is KILLED UNDER ASan.** With a TAG it is
   equivalent, by construction: `k >= start` differs from `k > start` only when `piece[start]` is a space at
   a break. On a first piece (start = prefix) that space's carry, `kLast − prefix − 1` characters, never fits
   the next piece's room, `kLast − prefix − 4`, so the scan hard-breaks either way; on a continuation,
   `piece[start]` is the carry's first character (never a space: the carry begins after the LAST space) or,
   when nothing was carried, the character that forced the break — a space only when the piece was FULL and
   the next character a space, which the fix now breaks AT and drops. But with NO tag, prefix = start = 0 and
   `k >= 0` is always true for a `size_t`: a first piece with no space scans on BELOW `piece` — an
   out-of-bounds stack read whose outcome depends on what lies there (the suite's no-tag case stayed green by
   luck: the stray byte failed the room test). Run under ASan through an include overlay (the mutant header
   in `fix3/asan/overlay_h5/`, the worktree untouched): **`AddressSanitizer: stack-buffer-overflow … in
   serialWrapped`**, rc 1. So H5, like H3, is caught only by the coordinator's whole-file ASan run; the
   review's empty-piece shape is in the suite as a regression case.
   *[Corrected 15:50 (the verification round, its item 1): the proof above is WRONG for a continuation.
   "When nothing was carried, the character that forced the break" misses the space branch's own
   continuation: it starts EMPTY (`start = len = next`), so the next character — a SECOND space after the
   full piece — lands at `piece[start]`, and H5 breaks there (`serialWrapped("T", run(251) + "  " +
   run(260, 'y'))`: an empty `[T] ... ` and a fourth piece). Not equivalent; now RED in the suite.]*
5. **GM1 (tests #5) — REAL, KILLED** (cases 40 and 41; the gate mutations below).
6. **H2 (tests #7) — REAL, KILLED** (K3; and the doc comment now says what a refused format sends).
7. **H3 needs the whole file under ASan (tests #8) — REAL.** Run here (H3 RED under ASan, the clean file
   clean); the test comments, build-order's SER1 section and this log now say the coordinator's owed run is
   the WHOLE file (`fix3/asan/build.sh` is the recipe). An ASan CI job is NOT added: `ci.yml` and
   `test/CMakeLists.txt` are MECH-A1's in this window — for the coordinator.
8. **The doc tool's `?` (tests #9, docs #15) — REAL, and WIDER: four misread names were already published**
   (the 10:05–10:16 entry). Fixed as a class: `check-coverage` fails on any unreadable name; the reader
   learned both shapes; 7 of 7 mutations RED.
9. **HA-190's settle and blast radius (docs #10) — REAL, FIXED.** Re-checked in the code:
   `competition_program.cpp` :805/:819/:836 log `entry` first (`opcontrol()` before `st.driver.run` at :863),
   and `LogCopySink::log()` writes the primary before it takes the copy guard. The register's settle now
   describes a capture that stops at a mode change with the robot NOT answering the sticks after the next
   enable; its blast radius says the SD copy stops at the same line (not "unaffected"). The same correction
   in worksheet K.7.3, guide 17 (the next PHASE blocks; both logs end there) and `log_copy_sink.hpp`'s banner.
10. **The numbered two-task bench test dropped (docs #12) — REAL; NOT BUILT — PUT TO THE TEAM LEAD.** The
    executor's reason ("a Makefile/gate change") was WRONG — Bench Tests is a station framework — and the
    delta was his to rule on. The real cost, checked: its menu is FULL (`bench_r3a.hpp` :50, :142 — twelve
    stations, 3 × 4; `bench_r3a.cpp` :2990-3000 — no row left on the 240-px panel), so a station means a new
    menu layout or a station retired, plus a capture check — a design choice, not a fix. Done here: K.7.3's
    heading says "EVIDENCE toward HA-189, whose dedicated settle is owed"; the register's *Settle (as fixed)*
    keeps the dedicated test as THE settle and says it is owed and why, with no forward reference (it is a
    public document, so it names no internal file); the owed test is recorded in build-order's SER1 section
    and REMAINING-WORK 18c, each naming it as the team lead's call; the false reason is annotated in place
    above. **For the coordinator to put to him:** (a) a numbered-line station in Bench Tests (a layout
    change), or (b) K.7.3 accepted in its place, in writing.
11. **Reasoned PROS behaviour stated as fact (docs #13) — REAL, FIXED:** the CONTRACT paragraph ("reasoned
    from its library (HA-189), not yet seen"), both LIMITS bullets, `log_copy_sink.hpp` ("reasoned, not
    yet seen"), guide 11 (HA-191 "believed … reasoned from PROS's library, not yet seen", HA-190 the same,
    the stack "upper bounds read from the compiled code … not a run on a robot"), the changelog's FIXED
    paragraph ("is believed to leave"); the banner's stack heading no longer says "measured, not
    estimated". The reviewer's frame re-measure (serialLine 288, writeWhole 32) matches mine; serialWrapped
    is 344 since the fix.
12. **The CAL1 hand-over missing from this log (docs #14) — REAL, RECORDED HERE:** CAL1's PROS shell
    (`calibrateInitialize()`, which prints before any provider exists) and its library program's
    `CalibrateIo` serial line must each be ONE call. A7 named `hal::pros::serialLine("[R3D] %s", text)`;
    **this builder recommends `hal::pros::serialWrapped("R3D", text)` instead — a DELTA from A7:**
    `serialLine` CUTS past 256 bytes and Calibrate's lines are not bounded by it (SER1 moved
    `calibrate_program.cpp`'s own `emit()` to `serialWrapped("R3D", line)` for exactly that reason, :194-196),
    while `serialWrapped` never cuts and needs no provider either. Step 9 fails a CAL1 file that keeps the
    `printf` + `fflush(stdout)` (`printf` and `_impure_ptr`), and since the review every other stdio printer
    too. CAL1's builder works in its own worktree; whichever lands second adapts (the brief's §5).
13. **Small inaccuracies (docs #16) — each REAL, FIXED:** `drive_program.cpp`'s "~430" → 408 (and this log's
    two ≈ 430s annotated); worksheet Station F :499 says a refusal past 255 characters arrives in `[DRV] ...`
    pieces; the changelog says the FILE constructor "now ALSO refuses a FILE with no descriptor" (it refused
    a null FILE before — `git show 3b5a39c:…/char_sink.hpp`); guide 11's Covers line names "how your own code
    prints lines"; K.7 is "the SD logs and the serial".

### For the coordinator, after the review *(nothing committed)*
- **The team lead's call (disposition 10):** HA-189's dedicated numbered two-task bench test — (a) a station in
  Bench Tests (its 3 × 4 menu is full: a layout change or a station retired), or (b) K.7.3 accepted in its
  place, in writing. Recorded in build-order's SER1 section and REMAINING-WORK 18c.
- **Owed by the coordinator:** the WHOLE `test/serial_line_test.cpp` under `-fsanitize=address`
  (`scratch/ser1/fix3/asan/build.sh <name>`; clean here, 30/30) — it is the only check that sees H3 and H5.
  *[15:50: H3 only — H5 is RED in the suite since the verification round; the file is 31 cases, clean under
  ASan.]*
  An ASan CI job would make it standing (`ci.yml` / `test/CMakeLists.txt`, MECH-A1's files in this window).
- **Merge notes:** `docs/api/calibrate_panel.md` (CAL1-A1's area), `auton_menu.md` and `command_registry.md`
  changed only by the doc tool's name fix — on a conflict, regenerate. `hal/pros/device_provider.hpp` :424
  still says "(`ProsCharSink`, stdout)" — MECH-A1's file, true of the descriptor, left as before.
  PROJECT-BRIEFING :260 and roadmap :1940 describe the coverage gate as failing on an undocumented entity —
  still true; that it now also fails an unreadable name is said in the changelog and `docs-publishing.md`.
- **CAL1's hand-over (disposition 12):** its shell's print → `hal::pros::serialWrapped("R3D", text)` (a delta
  from A7's `serialLine`, reason given); `CalibrateIo`'s serial line ONE call; step 9 fails anything else.

### 14:51–15:17 (2026-10-15) — RESUMED after the team lead's pause; one more name the reader misread, found on MECH-A1's tip *(`date` Thu Oct 15 02:51:30 PM EDT 2026 → 03:17:04 PM)*
- **Why resumed:** the team lead paused all work at ~11:10 near a usage limit; this fix round's run had worked
  10:01–11:06 and its disposition (above) was already written, all 13 subjects. MECH-A1 has since LANDED on main
  (`fd36e07`); the coordinator merges SER1 onto that tip.
- **Nothing left applied by the stopped run — checked, not assumed.** `git status`: SER1's 43 tracked files + its two
  new ones, nothing else; `git diff -- Makefile src/build_selection.hpp` empty; no plant in `src/` (`grep` for the
  nine step-9 forms and "plant" — only the "host test plant" prose). Every file a mutation tool backed up compared
  with its LAST backup in `scratch/ser1/backup/`: `char_sink.hpp` byte-identical to `S1__` and `H3asan__` (the
  run of record's pre-mutation copies — so every S/H/F/C result above was taken on THIS header), `api_doc_tool.py`
  to `D7__`, `bench_r3a.cpp` to `review-G1__`, `main.cpp` to `review-G2__`, `state_screens.cpp` to `S22__`;
  `src_build_gate.py` differs from `review-G6__` ONLY in the step-9 call comment the 10:44 entry records as applied
  after the gate mutations. The `src/` diff read in full: SER1's conversions only. The files with an mtime after
  10:48 (`Makefile`, `build_selection.hpp`, `main.cpp`, the robot tables, `calibrate_program.cpp`, …) are the src
  gate's own plant-and-restore during its 10:48–11:06 run: none is in `git diff` beyond SER1's own lines.
- **Not redone (finished, observed on fresh links):** the S1–S22, H1–H5, F1, C1–C5 runs, the gate's GM1–GM8 and
  G1–G6 (real ARM builds), D1–D7, H3/H5 under ASan. The stale binary was NOT trusted: rebuilt from scratch below.
- **NEW — found by testing the merge, not by a reviewer:** SER1's tool files laid over `fd36e07` in
  `scratch/ser1/merge-probe/` (`git archive fd36e07`, then every SER1 file but the six shared plan docs and
  `docs/api/`): `check-coverage` **FAILED** — `UNREADABLE NAME: 'BrakeWord::config::SafeBrakeMode'` and
  `'BrakeWord::hal::BrakeMode'` (`mechanism/kinds.hpp` :1182/:1184, `constexpr operator config::SafeBrakeMode()
  const noexcept`). A CONVERSION operator: the reader took the head's last token, the TYPE, for the name — on main
  they are PUBLISHED today as `BrakeWord::config::SafeBrakeMode` (`git show fd36e07:docs/api/kinds.md` :3496). The
  new gate was right to fail; merged as it stood, SER1 would have broken main's build. And the unqualified shape
  is worse: `explicit operator bool()` reads as `bool`, which READABLE_NAME accepts — silent.
  - **Fixed at the root (`tools/api_doc_tool.py`):** `CONVERSION_OPERATOR_HEAD` — at the parameter list, a head
    ending `operator <type>` names `operator <type>` (whitespace collapsed; `_looks_like_function` already kinds it
    a function); `NOT_A_NAME` — `unreadable_name()` also fails a lone keyword (`bool`, `int`, `const`, `operator`,
    …), so the misread that LOOKS like an identifier fails too. Fixture: `Declarators` gained `explicit operator
    bool()`, `constexpr operator std::size_t()`, `operator const char*()` — named, kinded function, anchored apart
    (`_spell`); the planted list gained a `bool` member, which must be REPORTED.
  - **Mutations (`scratch/ser1/fix3/doctool/dmuts_conv.{py,json,log}`, `dmut.py`; backups
    `backup/DC*__api_doc_tool.py`, each restored hash-checked, mtime bumped) — 3 of 3 RED:** DC1 the conversion
    reading removed → self-test (`Declarators: ['bool', …, 'char*', …, 'std::size_t']`) AND `check-coverage` on
    the merged probe (the two `BrakeWord` names); DC2 the keyword check removed → self-test (the planted `bool` not
    reported); DC3 both → self-test and the merged probe.
  - **On the merged probe, clean tool:** self-test OK; `check-coverage` "4514 public entities across 177 headers,
    all documented"; `generate` then a diff against `fd36e07`'s `docs/api/`: exactly SER1's expected changes — the
    two `BrakeWord` operators (`#brakeword-operator-config-safebrakemode`, `#brakeword-operator-hal-brakemode`),
    the four names of the 10:05 entry, `pros-char_sink.md`'s new entities; nothing else. No file outside
    `docs/api/` links the old `BrakeWord` anchors (`git grep` on `fd36e07`). **Host build of the merged probe
    (fresh): exit 0, 0 warnings; suite 2352 passed, 0 failed, 3 skipped (2,789,545 assertions).**
  - Docs in step: the changelog's "FIXED (the API reference)" (a conversion operator now named `operator T`; the
    gate also fails a lone keyword); `docs-publishing.md`'s `UNREADABLE NAME` row (its sentence also ended
    mid-clause — "four were |" — now whole).
- **One more wording fix (docs #10's neighbour, found re-reading HA-190):** its *Source* said `log_copy_sink.hpp`
  "names the same exposure for the card copy, which it survives" — beside a blast radius saying the copy STOPS. Now:
  a writer deleted inside the CARD copy, which the copy survives (its guard gives up), and a blocked serial ends
  the copy too — what the banner says. *[Corrected 15:50 (the verification round, its item 3): this rewording
  was itself wrong — the banner says the copy ENDS there for the rest of the boot; what survives is the
  program (no task waits). The Source line now says so.]*
- **Every gate on the final tree** (`scratch/ser1/fix4/`): `api_doc_tool generate` (no page changed by today's
  edits — no conversion operator in this tree) and the briefing regenerated; `build/test` DELETED, configured,
  built: exit 0, **0 warnings** (`build.log`); briefing regenerated with the suite binary present; suite **2284
  passed, 0 failed, 3 skipped (2,785,212 assertions)**; `serial_line_test.cpp` **30/30** (1,664). The WHOLE
  `serial_line_test.cpp` under `-fsanitize=address` (`fix3/asan/build.sh sl_final4`): **30/30, no report**.
  `gates.sh` (`gates.log`): api_doc_tool self-test OK, check-coverage "3674 … all documented", check-fresh 0,
  check-examples "525 quoted lines, all verbatim …", check-removability clean; briefing self-test OK (24) and
  "generated block current"; staleness self-test OK (10) and "clean (29 live docs …)"; CI greps "library is
  PROS-free …", "layering holds …"; ARM header gate "TU includes 207 headers" → **CLEAN**. `src_build_gate.py
  self-test` (14:58–15:13) **"OK (235 detector cases, each against a real build)"**; `check` (→15:16:50)
  `[stdio-list]` 40 names, all 25 printers among them; seven builds exit 0, 11/11 TUs, 0 disallowed, "program and
  stdio printers checked"; **"src build gate: PASS (…)"**; after it `git diff -- Makefile src/build_selection.hpp`
  empty.
- **Prose review (pace rule 4), run before this entry, every hit read:** "UNREADABLE NAME|unreadable name|name it
  cannot read|names it misread|conversion operator" over `README.md`, `docs/`, `include/`, `tools/` → the
  changelog, `docs-publishing.md`, the tool's own comments and message (its "(`?`, `1]`, a type)" still true);
  "named after its|named after the type|lone keyword|only a keyword" → the same, plus two unrelated DOCS2/T3
  records; "which it survives|card copy" in the register and the guide → the register's :130 and :2680 (true, the
  copy's one-writer guard), and the rewritten HA-190 Source.

### For the coordinator, added on resumption *(nothing committed)*
- **Merge onto `fd36e07`:** regenerate `docs/api/` after merging (`kinds.md` and `all-entities.md` change for
  `BrakeWord`'s two conversion operators, which main publishes today under their TYPE names; the merged probe shows
  the regenerated tree passing every doc gate and the whole suite). When merging the changelog, the "FIXED (the API
  reference)" paragraph can name them: `BrakeWord::config::SafeBrakeMode` / `BrakeWord::hal::BrakeMode` (anchors
  `#brakeword-config-safebrakemode`, `#brakeword-hal-brakemode`) → `BrakeWord::operator config::SafeBrakeMode` /
  `BrakeWord::operator hal::BrakeMode` — left to the merge because `BrakeWord` does not exist at this worktree's base.
- Everything under "For the coordinator, after the review" above still stands (the team lead's call on the
  numbered bench test; the WHOLE serial test under ASan; CAL1's hand-over; the merge notes).

---

## Verification-round findings and their disposition

### 15:33 (2026-10-15) — a fresh verifier's four findings, received *(`date` Thu Oct 15 03:33:44 PM EDT 2026)*
- The builder again, on the coordinator's workflow run. Four findings, all minor. Each was re-checked HERE before
  anything changed (the verifier's probes compiled and run from `scratch/ser1/fix5/`, never trusted as reported).
- Tree before any change: as the 15:17 entry left it. The three files this round changes were backed up first
  (`scratch/ser1/backup/fix5-pre__{char_sink.hpp,serial_line_test.cpp,api_doc_tool.py}`).

### 15:33–15:46 — the wrap: a double space after a full piece, and an empty piece after a newline *(`date` 15:46)*
- **Re-checked (`fix5/probe_h5.cpp`, the verifier's probe, compiled against THIS worktree's header):** CLEAN header,
  `serialWrapped("T", run(251) + "  " + run(260, 'y'))` → 3 pieces [256] `[T] xxx…`, [256] `[T] ...  yyy…`, [23];
  `run(251) + " \nab"` → [256], **[9] `[T] ... `**, [7] `[T] ab` — an EMPTY continuation, sent by the clean code.
  Both REAL. The H5 claim's error, found by reading the proof against the code: "when nothing was carried, the
  character that forced the break" misses the space branch's OWN continuation, which starts empty (`start = len =
  next`); the next character, a second space, lands at `piece[start]`, where H5 may break.
- **`include/shulib/hal/pros/char_sink.hpp`:**
  - The newline branch sends its piece only when it is the line's FIRST (`start == prefix`: sent even empty, so a
    blank line stays blank) or holds text (`len > start`) — never a continuation a break left empty. The end of the
    text already suppressed it (`len > start || !sentAny`); the two paths now agree. Every other send was already
    non-empty: the scan cuts at `k > start`, a hard break at `len >= kLast`.
  - The double space is KEPT, by design, and now said: only the ONE space at a break is dropped; any other space is
    the text's own (a double space at a break leaves its other space at the end of one piece or the start of the
    next), so nothing else is lost — the property the rejoin check pins. Dropping the whole run would lose text.
  - The `///` comment says both; `docs/api/` regenerated (`pros-char_sink.md` only).
  - **The frames re-measured** (`fix5/stack/`: the probe with the build's ARM flags, `--std=gnu++20`,
    `-fstack-usage`): serialLine 288, serialWrapped **344**, writeWhole 32 — unchanged, so the banner's numbers stand.
- **`test/serial_line_test.cpp` — 30 → 31 cases:** NEW "after a FULL piece, a SECOND space is the text's — kept, never
  an empty piece; a newline sends no empty piece": the verifier's shape (exactly 3 calls, the second `[T] ...  ` + 246
  `y`), `run(251) + " \nab"` (exactly 2: no `[T] ... ` between), `run(251) + "  \nab"` (3: the kept space is sent as
  `[T] ...  `, as the end of the text would send it), the same shape with NO tag (3; the continuation starts at
  index 4, so H5 is caught there too without reading below the buffer), and — added at 16:10 to pin the `///`
  comment's other half — `"DRV"`, `run(247) + "  " + run(260, 'y')`: the double space INSIDE a piece, where the scan
  breaks at the LAST space and the other one ends the first piece (`[DRV] xxx… \n`, then 245 and 15 `y`); no call
  may end `... \n` (an empty continuation, any tag); both double-space lines' pieces must rejoin. **`rejoins()` tightened:** it let a piece that BEGAN with a space keep the break's space
  — the PRE-review behaviour (F1's doubled space) — and so would have REJECTED the clean double-space output. Now
  the break consumes exactly the one space after a piece's text whenever there is one (a hard break never lands
  before a space: a space there is a break AT it).
- **Every mutation RUN on a fresh link, restored hash-checked (`fix5/muts_fix5.{json,log}`, `../mut.py`) — 36 runs,
  35 RED in the suite (2 of them at compile), H3 RED under ASan:**

  | # | Mutation | Result |
  |---|---|---|
  | S1–S22, H2–H4, F1, C1–C5 | the run of record's and the review's, on THIS header | all RED (S8, C4 at compile) |
  | H1 | the newline resets `len` but not `start` (re-anchored past the new `if`) | RED — K1 and the new case |
  | **H5** | `k >= start` | **RED IN THE SUITE** — the new case (the verifier's shape) |
  | H3 | `n == 0 \|\|` removed | SURVIVED the suite — RED under ASan (`fix5/asan_h3.log`: `stack-buffer-overflow … in serialLine`, through an include overlay, the worktree untouched) |
  | N1 | the newline's condition removed (the pre-fix code) | RED — the new case |
  | N2 | `start == prefix \|\|` removed (an empty first piece skipped) | RED — the blank-line case |
  | N3 | `\|\| len > start` removed (a continuation with text before a newline skipped) | RED — K1 and the new case |

  Restored tree: fresh build, the file 31/31.
- **The WHOLE file under ASan** (`fix3/asan/build.sh sl_fix5`): **31/31, no report** (`fix5/asan_clean.log`).
- **The verifier's fuzz** (`fix5/fuzz/`, rebuilt under ASan against this header): seeds 1–5 × 15,000 texts — **0
  failures, 0 empty continuations**. Control, the same fuzz on the PRE-fix header (an include overlay): seed 2 → 14
  empty continuations, seed 3 → 12 — so the fuzz sees the defect and the fix removes it.

### 15:37–15:40 — check-coverage's UNREADABLE NAME failure, pinned *(`date` 15:40)*
- **Re-checked:** the self-test called `coverage_problems()` only (its two calls, in the SER1 block) and never
  `do_check_coverage()`; the verifier's VD3 (`if unreadable:` → `if False:`) passes the old self-test and the tree's
  `check-coverage` (the tree has no misread name). REAL.
- **Fixed (`tools/api_doc_tool.py`):** `do_check_coverage(only=None, pages=None)` — `pages` is the self-test's (a page
  set checked in place of `build_pages()`); its docstring now says it also FAILS an unreadable name. The self-test
  runs it on four one-entity pages, every entity documented: `?`, `1]` and `bool` must each return 1 and print
  `UNREADABLE NAME: '<name>'` with `fixture.hpp:7`; `readableName` must return 0, print nothing on stderr and say "all
  documented" — the control, so the failure is the NAME's, not a coverage miss.
- **Mutations (`fix5/doctool/dmuts_vd*.{json,log}`, `../fix3/doctool/dmut.py`; backups
  `backup/VD*__api_doc_tool.py`, each restored hash-checked, mtime bumped) — 6 of 6 RED by the self-test:** VD3 the
  branch never fails (its `check-coverage` on the tree still rc 0, as the verifier showed — the self-test is now what
  sees it); VD6 the branch prints but returns 0; VD7 it fails without naming the entity; VD8 without the header:line;
  VD9 the `pages` hook ignored; VD10 check-coverage failing every page set (the control's side). One run aborted on
  VD8's first anchor (it named the wrong neighbour line) BEFORE writing anything — the tool compared byte-equal to
  its backup — and was re-run with the right anchor.
- **On the merged probe of main** (`scratch/ser1/merge-probe/`, the new tool copied in): self-test OK;
  `check-coverage` "4514 public entities across 177 headers, all documented".

### 15:41 — HA-190's Source line, and the 512-byte delta put to the team lead
- **HA-190 (`docs/hardware-assumptions.md`):** re-checked against `log_copy_sink.hpp` :59-63 ("leaves the guard held FOR
  THE REST OF THE BOOT … The card's copy simply ends there") and guide 17 :310-311 ("ends there for the rest of that
  boot"): the 14:51 rewording ("which the copy survives") was WRONG. Now: "names the same exposure for a writer deleted
  inside the CARD copy: the copy ends there for the rest of the boot, but no task waits (its guard gives up rather than
  waits) — and it says a blocked serial ends the copy too." That entry's line is annotated above.
- **The 512-byte pieces (the verifier's item 4):** REAL as a process gap — disposition 10's own standard (a delta from
  the brief is the team lead's to rule on) applies to it, and it was only in this log and the changelog. The code
  is unchanged. Put to him in build-order's SER1 section ("**Also his to accept:** …") and REMAINING-WORK 18c
  ("**Also yours to accept:** …"; its next-step column says "your rulings on the owed bench test and the 512-byte
  pieces"), and in the coordinator's list below.

### 15:46–15:48 — the log's wrong claims, annotated in place
- The 10:16 entry's "With the fix H5 is EQUIVALENT" and its mutation summary ("the two survivors (H3, H5)"),
  disposition 4's proof, "the only check that sees H3 and H5", and the 14:51 entry's HA-190 rewording — each
  annotated *[Corrected 15:50 …]* where it stands, not rewritten.

### 15:48–16:14 — every gate, on the final tree *(`date` Thu Oct 15 04:14:07 PM EDT 2026)*
- **Fresh host build at 15:48** (`api_doc_tool generate` and the briefing first; `build/test` deleted, configured,
  built with its doc gates): exit 0, **0 warnings**; briefing regenerated with the suite binary present; suite
  **2285 passed, 0 failed, 3 skipped** (2,785,249 assertions) = 2284 + the new case.
- **`src_build_gate.py self-test` (15:51–16:06): "OK (235 detector cases, each against a real build)"; `check`
  (→16:09:56):** `[stdio-list]` 40 names, all 25 printers among them; seven builds exit 0, 11/11 TUs, 0 disallowed,
  "program and stdio printers checked"; **"src build gate: PASS (…)"**; after it `git diff -- Makefile
  src/build_selection.hpp` empty (`fix5/srcgate.log`). The host build waited for it (its plants are in `src/`, which
  host pins read).
- **The test file's 16:10 addition (the `DRV` line), then everything again:** `build/test` deleted, configured,
  built: exit 0, **0 warnings** (`fix5/final2-build.log`); briefing regenerated before and after; suite **2285
  passed, 0 failed, 3 skipped (2,785,260 assertions)**; `serial_line_test.cpp` **31/31 (1,712)**. The 14 wrap
  mutations re-RUN on that file (`fix5/muts_fix5_wrap.{json,log}`: S12–S18, H1, H4, H5, F1, N1–N3) — **14 of 14
  RED**, restored hash-checked (header `aa0ec416…` before and after), restored tree 31/31. The WHOLE file under ASan
  (`sl_fix5b`): **31/31, no report**. The header did not change after the 15:48 build, so the src gate's run stands.
- **`fix3/gates.sh` (`fix5/gates2.log`):** api_doc_tool self-test OK, check-coverage "3674 public entities across 170
  headers, all documented", check-fresh 0, check-examples "525 quoted lines, all verbatim (6 source files); 6
  listings of whole-run files, each one unbroken run", check-removability clean; briefing self-test OK (24) and
  "generated block current"; staleness self-test OK (10) and "clean (29 live docs …)"; CI greps "library is PROS-free
  …", "layering holds: core is sim-free"; ARM header gate "TU includes 207 headers" → **CLEAN**.
- **Prose review (pace rule 4), run before this entry, every hit read:** "equivalent" near H5/`k >= start` → only this
  log's 10:16 entry and disposition 4 (each now annotated) and the new test's comment ("the fix round had called H5
  equivalent", true as history); "empty piece|empty continuation|hold(s) nothing" → the two wrap tests (true: the
  first's three inputs send none, the second pins it), the header's new sentence, and unrelated "holds nothing" prose
  (R3a/T4 logs, `block_char_sink.hpp`, `i_fusion_policy.hpp`); "copy survives|which it survives|the copy survives"
  outside this log → none; "H3 and H5|H3/H5|H3, H5" outside this log → none; "ONE-CALL|one `write()` per line" → the
  header's banner and the changelog's heading (true of every line shulib prints; the changelog's BEHAVIOUR CHANGE
  paragraph names the 512-byte pieces), REMAINING-WORK 18c and build-order (this round's delta lines), the register's
  HA-190/191 preamble (true), `triage.hpp` (the sink frames each line — true, a line ≤ 384).

### The verification round's disposition, finding by finding *(`date` 16:14)*
All four were re-checked here before anything changed; all four are REAL; all four are fixed in this tree.

1. **H5 is NOT equivalent with a tag; the clean header sent an empty `[T] ... ` after a newline — REAL, FIXED.**
   Reproduced with the verifier's own probe on this header (above). The proof's gap: the space branch's
   continuation starts empty, so a SECOND space lands at `piece[start]`. Fixed at the root, not only pinned: the
   newline branch no longer sends a continuation a break left empty (the end of the text already did not); the
   double space is kept on purpose (lossless — only the break's one space is dropped) and the `///` comment now says
   both. The verifier's case is in the suite and **H5 is RED in the suite** (no longer an ASan-only mutant); N1–N3
   pin the new condition from both sides; the fuzz shows 0 empty continuations in 75,000 texts (12–14 per 15,000 on
   the old header). `rejoins()` was also wrong for this shape (it would have rejected the clean output) and is
   tightened. The two log claims (:424, disposition 4) are annotated in place. Frames unchanged (344).
2. **check-coverage's UNREADABLE NAME failure unpinned — REAL, FIXED.** `do_check_coverage()` takes the self-test's
   page set; the self-test runs it on three planted unreadable names and one readable control; VD3 (the verifier's)
   and five more mutations of the wiring are RED by the self-test.
3. **HA-190's Source said the card copy "survives" — REAL (my 14:51 rewording), FIXED** to what the sink's banner and
   guide 17 say: the copy ENDS there for the rest of the boot; no task waits.
4. **The 512-byte pieces not put to the team lead — REAL, RECORDED.** By disposition 10's own standard this delta from
   his ONE-CALL WRITES ruling is his to accept; it is now in build-order's SER1 section, REMAINING-WORK 18c and the
   list below. The code is unchanged (it removes HA-191's hang; every shulib line, ≤ 384 bytes, is still one call).

### For the coordinator, after the verification round *(nothing committed)*
- **For the team lead to accept (NEW):** the sink's 512-byte pieces — `writeWhole()` offers each `write()` at most
  `kSerialWriteMaxBytes` (512), so a text longer than that through a team's own sink goes in several calls; his
  ONE-CALL WRITES ruling (A3, §1.1), by its letter, is one call per line whatever its length. Every line shulib prints
  (`Line::kCapacity` 384, pinned; `serialLine`/`serialWrapped` 256) is still ONE call; before the cap, a text past
  PROS's ~2,035-byte frame would have hung its writer (HA-191). If he rejects it, the alternative is the old single
  call with HA-191's hang documented as the team's to avoid. Put to him beside the bench-test call (disposition 10).
- **Owed by the coordinator (unchanged, now 31 cases):** the WHOLE `test/serial_line_test.cpp` under
  `-fsanitize=address` (`scratch/ser1/fix3/asan/build.sh <name>`; clean here, 31/31) — now the only check that sees
  H3 (H5 is RED in the suite).
- **Merge note (NEW):** `tools/api_doc_tool.py`'s `do_check_coverage()` gained a `pages=None` parameter; its one
  command-line caller is unchanged, and on the merged probe of main the new tool's self-test is OK and
  `check-coverage` "4514 … all documented". `docs/api/pros-char_sink.md` changed again (`serialWrapped`'s comment).
- Everything under "For the coordinator, after the review" and "added on resumption" above still stands (the
  bench-test call; CAL1's hand-over; the merge notes; `BrakeWord`'s two conversion operators).

### The coordinator's merge verification — SER1 (2026-10-15 16:20–21:52, main `fd36e07` + the squash of `ser1`)
- **Audit before the merge:** no gate plant, mutant or probe marker in the worktree's added lines; the transport commit
  `c9e4140`, then `git merge --squash ser1`; no stray file in the root.
- **Four document conflicts:** the changelog (SER1's entry first, dated as merged), the register (HA-189 already on main;
  SER1's HA-190 and HA-191 beside the core's held rows; the count **176**, "7 of 176 settled"), REMAINING-WORK row 18c and
  build-order (SER1 LANDED 2026-10-15), `docs/api/` and the briefing (regenerated).
- **Gates on the merged tree** (`.claude/coordinator/logs/ser1_merge_verify.out`, 16:20): host build with the doc gates; suite
  **2,353 | 2,353 passed, 3 skipped**; `src_build_gate.py` self-test OK (235 detector cases) and check **PASS, all 7 builds**
  (the new object check: no `src/` object names stdio's FILEs or calls a stdout/stderr printer); ARM header gate over **215**
  headers clean; CI greps empty; api check-fresh / examples / coverage / self-test, removability, the briefing and the
  staleness audit clean; the compile-refusal harness **195 of 195**. The owed sanitizer run: the whole
  `test/serial_line_test.cpp` under ASan + UBSan, **31 of 31** (the 2,000-byte `%s` case included).
- **The coordinator's hand mutations** (`muts_ser1_main.json`, on `include/shulib/hal/pros/char_sink.hpp`, each on a fresh
  link, the whole suite under a 4 GiB / 300 s cap, restored and hash-checked): **7 of 7 caught** — SM1 (the write loop goes on after a 0 return), SM3 (stderr for stdout),
  SM4 (the FILE constructor's one `fflush` removed), SM5 (`serialLine`'s newline guarantee), SM6 (a line of exactly the buffer
  without its newline not cut) and SM7 (an empty continuation piece sent again) RED; **SM2** (a short write re-sent from the start)
  loops while the fake keeps growing — killed by the 4 GiB cap at ~3.3 GB (caught). The restored binary: 2,353 | 2,353 passed;
  `git diff -- include src test tools` empty afterwards (`.claude/coordinator/logs/ser1_finish.out`, 21:42–21:52).
- **A coordinator incident, recorded so it is never repeated:** the first mutation run (16:50) was killed mid-SM2 by the
  session ending; SM2 (a short write re-sent from the start) loops for ever while its test's fake keeps appending, and the
  test binary left linked with it grew to 54–58 GB whenever anything ran it (the briefing tool among them) — the kernel and
  systemd-oomd killed it, the desktop with it, twice. Fixed in the coordinator's tooling, not the library: every suite and
  mutation run is capped (`ulimit -v` 4 GiB + `timeout`), a mutation restores its file and relinks in a `finally`, and the
  binary is rebuilt before the briefing is regenerated. (`tools/briefing_status.py` running the binary uncapped is a
  follow-up for a tooling change.)
