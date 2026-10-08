# SER1 — the USB serial log keeps every line whole when two tasks write it (HA-189)

> **Status 2026-10-08: the team lead's GO; his fix, re-ruled the same afternoon: ONE-CALL WRITES.** He first chose
> "Unbuffered stdout"; the brief's checker then showed, from the linked V5 image, that unbuffered stdout makes every
> `printf` take a ~1.2 KB stack frame (`__sbprintf`) and that PROS's own serial daemon (a 2 KB stack) overflows
> printing its banner when `pros terminal` connects to a running program — a FATAL halt — so he re-ruled: **"One-call
> writes"** — stdout stays as newlib sets it (line-buffered); shulib's serial sink sends each line with ONE direct
> `write()` (PROS's own serial mutex keeps one call whole); every `src/` line that reaches stdout goes through the same
> one-call path. Brief: the coordinator's (short form, his speedup), re-checked by the same checker. Nothing is built.
> Size: **~¾–1 builder-day.** HA: HA-189 is the belief this fixes; **HA-190 and HA-191 NEW** (§4).


## Re-check 2026-10-08 (the same checker, second pass) — amendments; they OVERRIDE the body below

Verdict "needs-changes", no blocker: one direct `write()` on fd 1 ("sout") is ONE framed message under PROS's
`write_mtx` and touches no stdio state (only `_write_r`'s `errno` and a one-byte config read in `ser_write_r` —
benign). Evidence and probes: `.claude/coordinator/scratch/ser1-check/` (`serialline/`, `hosttest/pending.cpp`,
`hosttest/dgram.cpp`, `tree/`).

- **A1 (the gate's object check, replaces §1.3's list).** BAN, on `nm -u` of every `src/` object of every build
  (`bin/**/*.o` minus `_pros_ld_timestamp.o`, inside `verdict()`, using the existing `find_nm()`): `puts`, `_puts_r`,
  `putchar`, `putc`, `fputc`, `__swbuf_r`, `fputs`, `printf`, `iprintf`, `vprintf`, `fprintf`, `vfprintf`, `fiprintf`,
  `perror`, `_ZSt4cout`, `_ZSt4cerr`, `_ZSt4clog`, and **`_impure_ptr`** (what any use of stdin/stdout/stderr compiles
  to — it catches `fwrite`/`fflush` ON STDOUT). ALLOW `fwrite`/`fflush`/`fopen` (the SD card's files: `block_sink.hpp`
  :200/:209, the bench probe :621-622). Self-test plants in `SRC_PLANT` as extern functions: `printf("x\n")` FAILS
  naming `bench_r3a.cpp.o` and `puts`; `fflush(stdout)` FAILS naming `_impure_ptr`; `fwrite` to a `FILE*` PARAMETER
  PASSES (the negative control).
- **A2 (the census, replaces §1.2's).** 20 printf sites at bdcbc7c+: `main.cpp` :144; `boot_source.cpp` :52;
  `screen_paint.cpp` :108; `bench_r3a.cpp` :178, :3127, :3133; `competition_program.cpp` :267, :632, :781, :785;
  `drive_program.cpp` :599, :607; `auton_tester_program.cpp` :439; `xdrive_program.cpp` :225, :249, :252;
  `state_screens.cpp` :30; `calibrate_program.cpp` :193, :1044, :1055 — AND the 14 `std::fflush(stdout)` lines,
  all DELETED: `boot_source.cpp` :53, `screen_paint.cpp` :110, `xdrive_program.cpp` :228/:255, `drive_program.cpp`
  :610, `calibrate_program.cpp` :194/:1058, `auton_tester_program.cpp` :445, `competition_program.cpp` :268/:638/:788,
  `state_screens.cpp` :31, `bench_r3a.cpp` :179/:3136. Re-count at the tip. Record each site's LONGEST line; a site
  whose line can exceed the buffer WRAPS (one call per piece, like `logWrapped`), never cuts.
- **A3 (the sink's constructors, replaces §1.1's constructor fflush).** A NEW `ProsCharSink() noexcept` that writes
  `STDOUT_FILENO` and names no FILE (no `fileno`, no `fflush`, no `stdout` — so `_impure_ptr` leaves the objects);
  `explicit ProsCharSink(std::FILE*)` stays (it takes `fileno` and does ONE `fflush`, documented "construct before
  anything else writes that FILE"). Source-compatible: `usb_{}` (device_provider.hpp :553) uses the new default; pin it.
  §6's "(single-task)" reworded: PROS's serial daemon still prints its banner through stdout's buffer on every "pRb";
  after SER1 no shulib text is ever in that buffer, and both banners end in "\n\n\n" (flushed whole).
- **A4 (`serialLine`'s buffer and stack, replaces §1.1's 512 and "under ~1 KB").** `kSerialLineBytes` = **256**,
  sized from A2's census (wrap past it). Measured with the project's ARM flags and `-fstack-usage`: at 512 the peak is
  ≈1.08 KB (`%s`), ≈1.24 KB (`%f`, which also ALLOCATES through `_dtoa_r` → malloc), ≈1.25 KB (the write of a 511-byte
  line: `ser_write_r` copies the frame onto its own stack); at 256 ≈0.82 / 0.98 / 0.74 KB. State the measured numbers
  in the banner, say `%f`/`%e`/`%g` allocate, drop the `Line::kCapacity` static_assert (no `Line` passes through it),
  and pin `vsnprintf(buf, sizeof buf, …)`.
- **A5 (the write loop, replaces §1.1's).** `ser_write_r` returns the full length or 0 (errno set); PROS's `_write`
  returns −1 for a bad fd; it never returns a partial count, but the loop must still be correct: **if 0 < n < len,
  write the remainder; if n ≤ 0, STOP** (and count the loss where a counter exists). glibc marks `write`
  `warn_unused_result` and the host build is `-Werror` — the result must be USED, not `(void)`-cast.
- **A6 (tests, replaces §1.4's first two bullets).**
  - **Pending bytes:** after constructing the FILE sink on a `tmpfile()`, `fwrite` "B" into the FILE and leave it
    buffered, then `sink.write("C\n")` and `pread` from 0: correct → `"C\n"`; the `fwrite`+`fflush` mutant → `"BC\n"`.
  - **One call per line:** a `SOCK_DGRAM` socketpair dup'd onto fd 1 (flush `std::cout`/stdout first; restore fd 1 with
    RAII — every test shares one binary): a whole line arrives as ONE datagram, a split line as two. Use it for both the
    default sink and `serialLine`.
  - **Short writes:** the injectable write seam ONLY for these: 100 bytes per call → the remainder, never a resend; a
    fake returning 0 / −1 → the loop stops (the fake caps its call count, so a spinning mutant FAILS, never hangs).
  - **Mutations added:** the loop continuing on 0 or −1; the FILE constructor's `fflush` removed.
  - No sanitizer job exists (`ci.yml`; `test/CMakeLists.txt` :22-23): the coordinator runs `serialLine`'s 2,000-byte
    `%s` case once under `-fsanitize=address`.
- **A7 (documents and hand-overs).** Drop HA-189's "nothing calls setvbuf" from §1.5's edits (it stays true). The
  rule for a team's own code goes in a GUIDE page (11 or 13) as well as the banner (teams author their code — G4).
  The banner names the stdio writers left after SER1 — all PROS's: the serial daemon's banners; ext-ADI's
  `puts("Error: validate_type")` (reachable through shulib's expander ports, `digital_in.hpp` :78, `digital_out.hpp`
  :97); crash dumps; `task_fn_wrapper`'s `fprintf(stderr)` — they meet shulib's lines only at line boundaries.
  HA-190 also covers `serr` (stderr's waits are unbounded too). HA-191: the ceiling is ~2,035 bytes per line (4-byte
  stream id + data + framing + delimiter in 2047). CAL1's hand-over names its shell's `printf("[R3D] %s\n")` +
  `fflush(stdout)` (`scratch/brief-CAL1/proposed3/.../calibrate.hpp` :54-55) → `serialLine("[R3D] %s", text)`.

## 0. Why — what the vendored binaries say (HA-189; the checker's disassembly, `scratch/ser1-check/`)

- The competition program logs from more than one task (the auton task, PROS's phase tasks, the opcontrol task's
  driver loop — `src/programs/competition_program.cpp` :74-82). PROS's phase functions (`initialize`, `disabled`,
  `competition_initialize`, `autonomous`, `opcontrol`) each run in their OWN PROS task; shulib creates exactly two
  tasks of its own (`competition_program.cpp` ~:791, `auton_tester_program.cpp` ~:562).
- Every task's `stdout` is the SAME `FILE` (`__sf[1]`; `prvInitialiseNewTask` points each task's reent there). newlib
  marks it line-buffered at start-up (`global_stdio_init`, flags 0x0009); its per-FILE locks are no-op stubs
  (`libc_a-lock.o`, `bx lr`; `libpros.a` overrides only the malloc and env locks). So two tasks inside `fwrite`/`printf`
  on stdout share one buffer with no lock: bytes can mix or be lost.
- **One direct `write()` call is safe:** newlib's `write` → `_write_r` → PROS's `ser_write_r`, which builds the frame on
  the CALLER'S stack and sends it under its write mutex (`mutex_take` +0x72, `ser_output_write` +0x9c, `mutex_give`
  +0xa6) — one call, one whole frame, no shared buffer. Two limits: PROS's serial stream holds **2047 bytes** and
  `stream_buf_send` waits until the WHOLE frame fits, so one call must stay well under ~2 KB (a larger one waits
  forever holding the mutex — HA-191); shulib's lines are at most `Line::kCapacity` = 384 bytes
  (`diag/line_format.hpp` :49) and TermSink's messages 200 (`diag/term_sink.hpp` :285).
- **Why not unbuffered stdout** (the first choice): the FATAL above, plus ~2 KB of stack per `printf` in every task — a
  trap for any team task created with a small stack. **Why not a lock of ours:** a task PROS deletes while holding it
  would block every later line. (PROS's own write mutex has the same exposure, inside one `ser_write_r` call — HA-190;
  SER1 neither causes nor cures it.)

## 1. Scope — IN

1. **`include/shulib/hal/pros/char_sink.hpp`** (PROS-free: `<cstdio>`, `<unistd.h>` only — no `pros/` header):
   - `ProsCharSink::write()` sends the line with ONE `::write(fd, data, len)` on the FILE's descriptor (`fileno(out_)`,
     resolved once in the constructor), looping only on a SHORT write (the remainder, never a re-send), and never
     touching the FILE's buffer — no `fwrite`, no `fflush` on the hot path. The constructor `fflush`es the FILE ONCE
     (it runs before shulib starts any task: every provider is built before either of shulib's two tasks — cite the
     lines) so nothing printed earlier is left behind the first direct write.
   - A free function `hal::pros::serialLine(const char* fmt, ...)` (printf-style, `__attribute__((format(printf,1,2)))`):
     formats into a buffer ON THE CALLER'S STACK of `kSerialLineBytes` (= 512; static_assert ≥ `Line::kCapacity` + the
     prefix, and ≤ 1024 so it stays far under the 2047-byte stream), cuts an over-long line at the buffer with a
     visible `...` marker, guarantees ONE trailing `\n`, and sends it with ONE `::write(STDOUT_FILENO, …)`. Its stack
     cost (vsnprintf's frame + the buffer) is stated in the banner, measured from the object (`-fstack-usage` or the
     disassembly), and must be under ~1 KB.
   - The banner: the false "fwrite is atomic per call … lines never interleave" REPLACED by the true rule and its
     evidence (HA-189), the 2 KB ceiling (HA-191), the deletion exposure of PROS's own mutex (HA-190), and the rule for
     a team's own code (one call per line through `serialLine` or the sink; no `puts`, no `printf("…\n")`, no
     `std::cout` chains from more than one task; keep every write under 2 KB).
2. **`src/` — every stdout writer goes through one call** (the checker's census: ~20 call sites in `competition_program.cpp`,
   `xdrive_program.cpp`, `calibrate_program.cpp`, `bench_r3a.cpp`, `drive_program.cpp`, `auton_tester_program.cpp`,
   `main.cpp`, `common/state_screens.cpp`, `common/screen_paint.cpp`, `common/boot_source.cpp` — re-count at the tip):
   each `printf`/`puts`/`putchar` that ends a line becomes `hal::pros::serialLine(…)` (or a sink write where a sink is
   in hand), one line per call; a multi-line boot block becomes one call per line. **`bench_r3a.cpp` :178's
   `std::printf("%s\n", line)` compiles to `puts`, which newlib sends as TWO writes** — it is the named example.
   Calibrate's `printf`s convert too (CAL1 Part B later replaces that file; its library program's `CalibrateIo` serial
   line must use the same one-call path — the hand-over in §5).
3. **The `src` build gate** (`tools/src_build_gate.py`, step 8 or a new step): no `src/` OBJECT references `puts`,
   `putchar`, `putc`, `fputc`, `fputs`, `printf`, `iprintf`, `vprintf`, `fwrite` or `fflush` on stdout — checked on the
   objects (the compiler rewrites `printf("x\n")` → `puts` invisibly in the source), with a self-test case that plants
   a `printf("x\n")` and sees the gate fail by name. (`snprintf`/`vsnprintf` into a buffer stay allowed.)
4. **Tests (adversarial, host — glibc; they prove the CALLS and their one-write shape, not newlib — that is HA-189's
   bench settle):**
   - The sink on a `tmpfile()`: write a line WITHOUT a newline; read it back with `pread(fileno(f), …, 0)` with no
     `fflush` — present only if `write()` bypassed the buffer (a `fwrite` regression shows 0 bytes).
   - One call per line: the sink over a pipe or an `LD_PRELOAD`-free wrapper — the test-only seam the executor chooses
     (e.g. the sink's write function injectable, default `::write`) — counts EXACTLY ONE write call per `write()` of a
     300-byte line, the whole line in it; and a short-write fake (it accepts 100 bytes per call) yields the remainder,
     never a duplicate.
   - `serialLine`: one call per line; the cut at `kSerialLineBytes` with its marker; exactly one `\n` (a format that
     already ends with `\n` gets no second one); a `%s` of 2,000 bytes never writes past the buffer (ASan in the suite's
     sanitizer job if one exists, else a canary).
   - Source pins: `char_sink.hpp` includes no `pros/` header (the CI PROS-free guard SKIPS `hal/pros/` — so a pin is the
     only check); no `printf`/`puts`/`putchar` in `src/` source text outside comments (the gate checks the objects).
   - **Mutations, each run and seen red:** the sink back to `fwrite`+`fflush`; the sink's write split into two calls
     (text, then `"\n"`); the short-write loop re-sending from the start; `serialLine` without the newline guarantee;
     `serialLine`'s buffer to 4096 (the static_assert); one `src/` site back to `printf("…\n")` (the gate's object
     check); `bench_r3a.cpp` :178 back to `printf("%s\n")`.
5. **Documents (same change):** the changelog ("Fixed since 2.4": the serial log's lines stay whole across tasks; one
   new free function and a new sink behaviour — additive); HA-189 (Source: the fix; settled only at the bench), HA-190,
   HA-191 (§4); `diag/log_copy_sink.hpp` :33-36 and `competition_program.cpp` :79-81 (they say serial is unguarded);
   `docs/hardware-assumptions.md` :129-130 and HA-189's "nothing calls setvbuf" line; `docs/guide/17-the-competition-program.md`
   :306-312 ("serial keeps every line" — true after SER1 except HA-190's deletion case: say so); guide 11 if it speaks of
   interleaving; `docs/api/` regenerated; build-order, REMAINING-WORK, `SER1-PROGRESS.md` (created first), the briefing.
   Station K (the worksheet, K.7) gains HA-189's bench settle: two tasks printing numbered lines through the default
   `pros terminal` for 60 s, plus Timed-Run mode changes while they print — every line whole, every number once; and
   connect `pros terminal` to a RUNNING program once (the banner still prints; no FATAL).

## 2. OUT

- Unbuffered stdout (re-ruled out: the FATAL); a lock of ours; one printing task (~1–2 days).
- `ENABLE_BANNER(false)` — not needed: stdout stays line-buffered, so the daemon's banner costs what it costs today.
- The SD card copy (its own guard since T4 Part 3). PROS's own mutex-on-deletion hang (HA-190 records it; a fix would
  be PROS's, or a single writer task — the team lead's "one printing task" option, not chosen).

## 3. Checks (the coordinator re-runs every one)

The host suite; the ARM header gate; `python3 tools/src_build_gate.py check` (all seven builds, 0 disallowed warnings —
and the new object check) and `self-test`; the doc gates (`api_doc_tool.py` check-fresh/coverage/examples/
removability, the briefing, the staleness audit); the coordinator's own mutations on the merged tree.

## 4. Hardware Assumptions — HA-189 amended; HA-190 and HA-191 NEW (the register's next free is HA-190)

- **HA-190 — a task PROS deletes while it is inside `ser_write_r` leaves PROS's serial write mutex held, and every
  later serial write blocks forever.** Reasoned from `libpros.a`: `ser_write_r` holds its mutex across
  `stream_buf_send` with unbounded waits; `task_delete` releases no mutex. The competition program's `opcontrol()` logs
  two lines right after its loop sees a disable — the moment PROS deletes that task (`competition_program.cpp` :77-78).
  Settle: Station K's Timed-Run mode changes while two tasks print. Blast radius: every later serial line of that boot
  blocks its writer — a driver loop that logs would stall. Not caused or cured by SER1.
- **HA-191 — one serial write must fit PROS's 2047-byte stream:** a larger frame waits forever holding the mutex.
  Reasoned (`ser_driver_initialize` +0x38; `stream_buf_send`). shulib's lines are ≤ 384 bytes; `serialLine` ≤ 512.

## 5. Coordination

- **CAL1** (its brief is being amended): its PROS shell's `calibrateInitialize()` prints before any provider exists, and
  `CalibrateIo`'s serial line must be ONE call — through `hal::pros::serialLine` (or the sink). The coordinator adds this
  to CAL1's brief at its commit; whichever lands second adapts.
- **MECH-A1/A2** build `include/` only; no conflict. T4's files: `competition_program.cpp`, `drive_program.cpp`,
  `auton_tester_program.cpp` are touched only at their stdout calls.

## 6. Landmines

- Do not build the line in a MEMBER buffer of the sink or a static buffer in `serialLine` — two tasks would share it.
  The buffer is on the caller's stack, always.
- A direct `write()` after `printf` text still sitting in stdout's buffer would reorder output: the constructor's one
  `fflush` (single-task) and the gate's ban on `src/` stdio make that impossible in shulib's own code; say it in the
  banner for teams.
- The host tests prove shulib's calls; newlib's behaviour on the brain is HA-189's settle. Say "host test plant" /
  "host", never "simulator".
