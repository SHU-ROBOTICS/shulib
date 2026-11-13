#!/usr/bin/env python3
"""THE BOUNDED SUITE RUN — the one way the tools run build/test/shulib_tests.

WHY THIS EXISTS
---------------
On 2026-10-15 the coordinator's mutation run SM2 (a short serial write re-sent from the start)
made the suite loop for ever while a test's fake grew without bound: it reached 58 GB in the
mutation runner. The run was killed with that mutant still LINKED, and the next thing to run the
binary — `briefing_status.py generate`, unbounded — took it to 54 GB and the desktop down with it.
The build's doc gate ran the same unbounded code (briefing_status.py check, then
doc_staleness_audit.py, which held all of the output in memory for up to 30 minutes).

So every tool that runs the suite runs it through `run()` here (briefing_status.py,
doc_staleness_audit.py, release.py), under bounds far above a real run (the suite peaks near
344 MiB of address space and ~60 MB resident, and takes ~21–37 s):

  * 4 GiB of address space (RLIMIT_AS), 256 MiB of output (RLIMIT_FSIZE; the output goes to a
    temporary FILE, so the tool itself never holds it — on a tmpfs that file is RAM, bounded by
    the same 256 MiB), no core dumps, 600 s of CPU time as a kernel backstop, 300 s of wall time;
  * each bound is the SMALLER of this one and the caller's own (soft and hard), set as both soft
    and hard in the child, so a caller under a tighter `ulimit` keeps it — and the verdict
    quotes the bound that APPLIED;
  * the run stays in the CALLER'S process group, so a `timeout`, a Ctrl-C or a SIGTERM aimed at
    the caller's group reaches it; on Linux the child also asks the kernel to SIGKILL it if this
    tool dies first (PR_SET_PDEATHSIG), so it can never outlive the tool; a wall-time timeout
    kills it directly. The timeout kill and the death tie cover the DIRECT child only — a
    descendant is reached only by a signal to the caller's group, and survives a normal exit.
    The suite does not fork.
  * the bounds are per process (RLIMIT_AS is not a cgroup), and a platform that refuses one
    (macOS does not enforce RLIMIT_AS) runs without that one rather than failing every run:
    there the run has the output, CPU and time bounds only, and `self-test` says so.

THE VERDICT
-----------
A run is BROKEN — never "not built", never a count to commit — when it was killed by a signal
(a crash, or a bound that kills), did not finish in time, could not be started, printed no
doctest summary, exited non-zero with a summary that says nothing failed, used HALF or more of
its memory bound at its peak (a hit the code under test caught and swallowed — the library's own
`catch (...)` blocks turn an allocation failure into an ordinary failure by design, and
`CHECK_THROWS` passes on it — so neither doctest nor the summary would show it), or printed any of
doctest's reports of a CAUGHT std::bad_alloc anywhere in its output (a test case or an assertion
that THREW it, or one that "threw a DIFFERENT exception" that was it): doctest catches an
allocation failure and goes on to a normal summary, so a memory-bound hit would otherwise read
as an ordinary red count. A test that expects std::bad_alloc and gets it prints nothing (with
`-s`, "... threw as expected!"), so it is not mistaken for one. A run with failing tests that exits
non-zero is NOT broken: its summary is counted, red. Known limit: a red run that prints more
than 256 MiB of failure reports (about two million failing assertions) hits the output bound
and reads BROKEN.

USAGE
  python3 tools/suite_run.py green       # exit 0 only if the suite runs to a summary with 0 failures
  python3 tools/suite_run.py show        # print the verdict
  python3 tools/suite_run.py self-test   # prove each bound and each verdict rule can fire
"""

import os
import re
import signal
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BINARY = os.path.join(REPO, "build/test/shulib_tests")

MEMORY_BYTES = 4 * 1024 ** 3
OUTPUT_BYTES = 256 * 1024 ** 2
TIMEOUT_S = 300
CPU_SECONDS = 2 * TIMEOUT_S  # the kernel's own backstop, if the tool is gone before its timeout
_TAIL_BYTES = 65536  # doctest's summary is the last thing it prints
# Every form doctest prints for a CAUGHT std::bad_alloc (read from a real doctest probe's output):
#   ERROR: test case THREW exception: std::bad_alloc
#   ERROR: CHECK( f() ) THREW exception: "std::bad_alloc"              (also REQUIRE, CHECK_NOTHROW)
#   ERROR: CHECK_THROWS_AS( f(), X ) threw a DIFFERENT exception: "std::bad_alloc"
#   ERROR: CHECK_THROWS_WITH_AS( f(), "x", X ) threw a DIFFERENT exception! (contents: "std::bad_alloc")
_BAD_ALLOC = re.compile(rb'(?:THREW exception: "?|DIFFERENT exception: "|DIFFERENT exception! \(contents: ")'
                        rb'std::bad_alloc')
_BAD_ALLOC_MAX = 80  # longer than any match above: the overlap kept between scan chunks


def _cap(want, soft, hard, infinity):
    """The bound to set: the smallest of `want` and the caller's finite soft and hard limits."""
    cap = want
    for cur in (soft, hard):
        if cur != infinity:
            cap = min(cap, cur)
    return cap


def _applied(memory, output, cpu):
    """The bounds the child will really get (the caller's limits are inherited): (memory, output,
    cpu), each None where this platform has no such limit."""
    try:
        import resource
    except ImportError:
        return None, None, None
    got = []
    for which, want in ((resource.RLIMIT_AS, memory), (resource.RLIMIT_FSIZE, output),
                        (resource.RLIMIT_CPU, cpu)):
        try:
            soft, hard = resource.getrlimit(which)
            got.append(_cap(want, soft, hard, resource.RLIM_INFINITY))
        except (ValueError, OSError):
            got.append(None)
    return got[0], got[1], got[2]


def _size(n):
    """A byte count as the verdict prints it (KiB under 1 MiB, so a small bound never reads "0 MiB")."""
    return f"{n // 1024 ** 2} MiB" if n >= 1024 ** 2 else f"{n // 1024} KiB"


def _prctl():
    """Linux's prctl, resolved in the PARENT (no import between fork and exec); None elsewhere or if
    this Python has no ctypes — the death tie is then absent, every other bound still applies."""
    if not sys.platform.startswith("linux"):
        return None
    try:
        import ctypes
        return ctypes.CDLL(None, use_errno=True).prctl
    except (ImportError, OSError, AttributeError):
        return None


def _child_setup(memory, output, cpu, parent):
    """The child's limits and its tie to this process, applied between fork and exec (header)."""
    prctl = _prctl()

    def apply():
        import resource
        for which, want in ((resource.RLIMIT_AS, memory), (resource.RLIMIT_FSIZE, output),
                            (resource.RLIMIT_CORE, 0), (resource.RLIMIT_CPU, cpu)):
            try:
                soft, hard = resource.getrlimit(which)
                cap = _cap(want, soft, hard, resource.RLIM_INFINITY)
                resource.setrlimit(which, (cap, cap))  # lowering a hard limit is always allowed
            except (ValueError, OSError):
                pass  # a platform that refuses this bound runs without it (header)
        if prctl is not None:
            prctl(1, signal.SIGKILL)  # PR_SET_PDEATHSIG
            if os.getppid() != parent:  # the tool died before the prctl took effect
                os._exit(1)
    return apply


def run(cmd, memory=MEMORY_BYTES, output=OUTPUT_BYTES, timeout=TIMEOUT_S, cwd=REPO,
        scan_chunk=1 << 20, cpu=None):
    """Run `cmd` (a list) under the bounds. Returns (the end of its stdout, why it is BROKEN or '',
    its exit status — negative for a signal, None if it never ran or was cut at the timeout)."""
    cpu = CPU_SECONDS if cpu is None else cpu
    mem_applied, out_applied, cpu_applied = _applied(memory, output, cpu)
    with tempfile.TemporaryFile() as out:
        try:
            p = subprocess.Popen(cmd, cwd=cwd, stdin=subprocess.DEVNULL, stdout=out,
                                 stderr=subprocess.DEVNULL,
                                 preexec_fn=_child_setup(memory, output, cpu, os.getpid()))
        except (OSError, subprocess.SubprocessError) as e:
            return "", f"it could not be run ({e})", None
        try:
            rc, peak = _reap(p, timeout)
        except subprocess.TimeoutExpired:
            p.kill()
            try:
                p.wait(timeout=30)
            except subprocess.TimeoutExpired:
                pass
            return "", f"it did not finish within the {timeout} s bound", None
        except BaseException:
            p.kill()
            raise
        # The WHOLE output is scanned for a caught allocation failure — doctest reports it where
        # the test ran, far ahead of the summary — a chunk at a time, overlapping across chunks.
        out.seek(0)
        caught_bad_alloc = False
        carry = b""
        while True:
            chunk = out.read(scan_chunk)
            if not chunk:
                break
            window = carry + chunk
            if _BAD_ALLOC.search(window):
                caught_bad_alloc = True
                break
            carry = window[-_BAD_ALLOC_MAX:]
        size = out.seek(0, os.SEEK_END)
        out.seek(max(0, size - _TAIL_BYTES))
        text = out.read().decode("utf-8", errors="replace")
    mem = _size(mem_applied) if mem_applied is not None else "no memory bound"
    outb = _size(out_applied if out_applied is not None else output)
    cpus = f"{cpu_applied} s of CPU" if cpu_applied is not None else "no CPU bound"
    if rc < 0:
        return text, (f"it was killed by signal {-rc} — a crash, or a bound ({mem}, {outb} of output, "
                      f"{cpus})"), rc
    if caught_bad_alloc:
        return text, ("a test ran out of memory under the bound (doctest caught std::bad_alloc; "
                      f"{mem})"), rc
    if mem_applied is not None and peak is not None and peak * 2 >= mem_applied:
        return text, (f"it used {_size(peak)} at its peak — half or more of its {mem} bound: an "
                      "allocation failure was probably caught and swallowed"), rc
    return text, "", rc


def _reap(p, timeout):
    """Wait for `p` with os.wait4 — its exit status AND its peak resident memory, which Popen.wait
    cannot give. Returns (exit status, peak bytes or None); raises TimeoutExpired at `timeout`."""
    import time
    end = time.monotonic() + timeout
    while True:
        pid, status, usage = os.wait4(p.pid, os.WNOHANG)
        if pid == p.pid:
            p.returncode = os.waitstatus_to_exitcode(status)  # Popen must not reap it again
            scale = 1024 if sys.platform.startswith("linux") else 1  # Linux reports KiB, macOS bytes
            return p.returncode, (usage.ru_maxrss * scale if usage.ru_maxrss else None)
        if time.monotonic() >= end:
            raise subprocess.TimeoutExpired(p.args, timeout)
        time.sleep(0.02)


def parse(out, why=""):
    """doctest's two summary lines → the counts; {"broken": why} when the run is BROKEN (header)."""
    if why:
        return {"broken": why}
    cases = re.search(r"test cases:\s*(\d+)\s*\|\s*(\d+) passed \|\s*(\d+) failed"
                      r"\s*\|\s*(\d+) skipped", out)
    asserts = re.search(r"assertions:\s*(\d+)\s*\|\s*(\d+) passed \|\s*(\d+) failed", out)
    if not cases or not asserts:
        return {"broken": "it exited without doctest's summary lines"}
    return {"cases": int(cases.group(1)), "failed": int(cases.group(3)),
            "skipped": int(cases.group(4)), "assertions": int(asserts.group(1)),
            "afailed": int(asserts.group(3))}


def verdict(text, why, rc):
    """parse(), plus the exit status: a summary that says nothing failed with a non-zero exit is BROKEN."""
    got = parse(text, why)
    if "broken" not in got and rc not in (None, 0) and got["failed"] == 0 and got["afailed"] == 0:
        return {"broken": f"it exited {rc} although its summary says nothing failed"}
    return got


def counts(binary=BINARY, cmd=None, **bounds):
    """The built suite's verdict: None if `binary` is not built, else verdict() of a bounded run of
    `cmd` (default: the binary itself)."""
    binary = os.path.abspath(binary)
    if not os.path.exists(binary):
        return None
    return verdict(*run(cmd or [binary, "--no-intro=true"], **bounds))


def green_status(got):
    """`green`'s verdict on counts(): (exit status, message) — 0 only for a summary with nothing failing."""
    if got is None:
        return 1, "suite run: build/test/shulib_tests is not built"
    if "broken" in got:
        return 1, f"suite run: BROKEN — {got['broken']}"
    if got["failed"] or got["afailed"]:
        return 1, f"suite run: RED — {got['failed']} case(s), {got['afailed']} assertion(s) failing"
    return 0, f"suite run: green — {got['cases']} cases, {got['assertions']} assertions"


def recovery():
    """The order that works when the binary on disk is BROKEN (RELEASING.md §3)."""
    return ("rm -f build/test/shulib_tests && python3 tools/briefing_status.py generate && "
            "cmake --build build/test && python3 tools/briefing_status.py generate")


def memory_bound_enforced():
    """True when this platform enforces RLIMIT_AS on a child (Linux does; macOS does not)."""
    hog = [sys.executable, "-c",
           "import sys\ntry:\n    bytearray(768 * 1024 ** 2)\nexcept MemoryError:\n    sys.exit(3)\n"]
    try:
        p = subprocess.run(hog, preexec_fn=_child_setup(256 * 1024 ** 2, 1024 ** 2, 60, os.getpid()),
                           stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL, timeout=30)
    except (OSError, subprocess.SubprocessError):
        return False
    return p.returncode == 3


def _alive(pid):
    """True if `pid` runs and is not a zombie (a killed child its new parent has not reaped yet)."""
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    try:
        with open(f"/proc/{pid}/stat") as f:
            return f.read().rsplit(")", 1)[1].split()[0] != "Z"
    except OSError:
        return True


def _wait_gone(pid, seconds=5.0):
    import time
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if not _alive(pid):
            return True
        time.sleep(0.05)
    return not _alive(pid)


def self_test():
    """Every bound and every verdict rule, seen firing, on fake suites run through /bin/sh (so a
    noexec temporary directory cannot fail it). Each bound's case prints a GOOD summary after the
    misbehaviour, so it reads BROKEN only if that bound really applied. Exit 0 or 1."""
    import contextlib
    import inspect
    import io
    import shlex
    import time
    fails = []
    notes = []
    linux = sys.platform.startswith("linux")
    here = os.path.dirname(os.path.abspath(__file__))
    summary_red = ("[doctest] test cases:   12 |   10 passed |  2 failed |  1 skipped\n"
                   "[doctest] assertions:   40 |   37 passed |  3 failed |\n")
    summary_green = ("[doctest] test cases:   12 |   11 passed |  0 failed |  1 skipped\n"
                     "[doctest] assertions:   40 |   40 passed |  0 failed |\n")
    red = {"cases": 12, "failed": 2, "skipped": 1, "assertions": 40, "afailed": 3}
    green = {"cases": 12, "failed": 0, "skipped": 1, "assertions": 40, "afailed": 0}
    small = {"memory": 512 * 1024 ** 2, "output": 1024 ** 2, "timeout": 1.5, "cpu": 20}
    # The defaults never drift above the bounds the header promises.
    if not (MEMORY_BYTES <= 4 * 1024 ** 3 and OUTPUT_BYTES <= 256 * 1024 ** 2 and TIMEOUT_S <= 300
            and CPU_SECONDS <= 600):
        fails.append("a default bound is above the header's promise (4 GiB, 256 MiB, 300 s, 600 s CPU)")
    sig = inspect.signature(run).parameters
    if (sig["memory"].default, sig["output"].default, sig["timeout"].default, sig["cpu"].default) != (
            MEMORY_BYTES, OUTPUT_BYTES, TIMEOUT_S, None):
        fails.append("run()'s default bounds are not the module's constants")
    # counts() passes NO bound of its own to run() (a stub records what it is given).
    seen = []
    real_run = globals()["run"]
    try:
        globals()["run"] = lambda cmd, **kw: (seen.append(kw), (summary_green, "", 0))[1]
        counts(sys.executable)
    finally:
        globals()["run"] = real_run
    if seen != [{}]:
        fails.append(f"counts() overrides the bounds: run() was given {seen}")
    # green's exit status, through main() itself (counts() stubbed).
    real_counts, real_argv = globals()["counts"], sys.argv
    try:
        for got_, want_ in ((None, 1), ({"broken": "x"}, 1), (red, 1), (green, 0)):
            globals()["counts"] = lambda *a, _g=got_, **k: _g
            sys.argv = ["suite_run.py", "green"]
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                rc_ = main()
            if rc_ != want_:
                fails.append(f"main() green on {got_}: exit {rc_}, want {want_}")
    finally:
        globals()["counts"], sys.argv = real_counts, real_argv
    # The clamp, as arithmetic (the platform's own infinity is passed in).
    INF = -7
    for want, soft, hard, expect in ((100, INF, INF, 100), (100, 60, INF, 60), (100, INF, 40, 40),
                                     (100, 70, 50, 50), (100, 200, 300, 100)):
        if _cap(want, soft, hard, INF) != expect:
            fails.append(f"_cap({want}, {soft}, {hard}) = {_cap(want, soft, hard, INF)}, want {expect}")
    enforced = memory_bound_enforced()
    if linux and not enforced:
        fails.append("the memory bound is not enforced on Linux — RLIMIT_AS must be applied (a bound dropped?)")
    with tempfile.TemporaryDirectory() as d:
        def script(name, body):
            p = os.path.join(d, name)
            with open(p, "w") as f:
                f.write(body)
            return ["/bin/sh", p]

        def summary_then(code=1, summary=summary_red):
            return f"printf '%s' '{summary}'\nexit {code}\n"

        def doctest_line(line):
            # a real doctest report line, 300 KB ahead of the summary (a LATER scan chunk)
            return ("head -c 300000 /dev/zero | tr '\\0' x\n"
                    f"printf '%s\\n' '{line}'\nhead -c 300000 /dev/zero | tr '\\0' y\n")

        reports = [  # every form doctest prints for a caught std::bad_alloc (the header)
            'probe.cpp:10: ERROR: test case THREW exception: std::bad_alloc',
            'probe.cpp:11: ERROR: CHECK( grow_bool() ) THREW exception: "std::bad_alloc"',
            'probe.cpp:12: FATAL ERROR: REQUIRE( grow_bool() ) THREW exception: "std::bad_alloc"',
            'probe.cpp:13: ERROR: CHECK_NOTHROW( grow() ) THREW exception: "std::bad_alloc"',
            'probe.cpp:14: ERROR: CHECK_THROWS_AS( grow(), std::runtime_error ) threw a DIFFERENT '
            'exception: "std::bad_alloc"',
            'probe.cpp:15: ERROR: CHECK_THROWS_WITH_AS( grow(), "x", std::runtime_error ) threw a '
            'DIFFERENT exception! (contents: "std::bad_alloc")',
        ]
        m = _BAD_ALLOC.search(reports[1].encode())
        straddle_fill = 4096 - (m.start() + 10)  # the boundary falls 10 bytes into the matched text
        if not (straddle_fill > 0 and straddle_fill + m.start() < 4096 < straddle_fill + m.end()):
            fails.append("straddle: the case no longer puts the chunk boundary inside the match")
        swallow_py = os.path.join(d, "swallow.py")
        with open(swallow_py, "w") as f:
            f.write("x = b'x' * (300 * 1024 ** 2)\nprint(" + repr(summary_green) + ", end='')\n")
        cases = [
            ("red", script("red", summary_then(1)), red, "a red run (doctest exits 1) is counted"),
            ("tail", script("tail", "head -c 200000 /dev/zero | tr '\\0' x\n" + summary_then(1)), red,
             "a summary after 200 KB of output is found (the tail is read)"),
            ("expected_bad_alloc", script("expected", doctest_line(
                'probe.cpp:20: SUCCESS: CHECK_THROWS_AS( grow(), std::bad_alloc ) threw as expected!"std::bad_alloc"')
                + summary_then(1)), red, "a test that EXPECTS std::bad_alloc is not a caught one"),
            ("straddle", script("straddle", f"head -c {straddle_fill} /dev/zero | tr '\\0' x\n"
                                f"printf '%s\\n' '{reports[1]}'\n" + summary_then(1)),
             "broken", "a report whose MATCH straddles a scan-chunk boundary (the overlap carried)"),
            ("swallowed", script("swallowed", f"exec '{sys.executable}' '{swallow_py}'\n"), "broken",
             "300 MiB used under a 512 MiB bound with a green summary: a swallowed allocation failure"),
            ("green_rc", script("green_rc", summary_then(3, summary_green)), "broken",
             "a summary saying nothing failed, with a non-zero exit"),
            ("spew", script("spew", "trap '' XFSZ\nhead -c 4000000 /dev/zero\n" + summary_then(1)), "broken",
             "4 MB of output under the 1 MiB output bound"),
            ("crash", script("crash", f"printf '%s' '{summary_red}'\nkill -TERM $$\n"), "broken",
             "a signal death AFTER a good summary"),
            ("mute", script("mute", "echo a run that never printed its summary\n"), "broken",
             "no summary: broken, never 'not built'"),
        ] + [(f"bad_alloc_{i}", script(f"bad_alloc_{i}", doctest_line(line) + summary_then(1)), "broken",
              f"a caught bad_alloc, as doctest prints it: {line[line.index('ERROR'):][:60]}")
             for i, line in enumerate(reports)]
        if enforced:
            hog_py = os.path.join(d, "hog.py")
            with open(hog_py, "w") as f:
                f.write("import sys\ntry:\n    bytearray(1024 ** 3)\nexcept MemoryError:\n"
                        "    print('HOG-STOPPED', flush=True)\n    sys.exit(3)\n")
            cases.append(("hog", script("hog", f"'{sys.executable}' '{hog_py}'\n[ $? -eq 3 ] && exit 3\n"
                                        + summary_then(1)),
                          "broken", "a 1 GiB allocation under the 512 MiB memory bound"))
        else:
            notes.append(f"the memory bound is not enforced on {sys.platform}: its case is skipped")
        for name, cmd, want, why in cases:
            text, broke, rc = run(cmd, scan_chunk=4096, **small)
            got = verdict(text, broke, rc)
            ok = (isinstance(got, dict) and "broken" in got) if want == "broken" else (got == want)
            if name == "hog" and ok and "HOG-STOPPED" not in text:
                ok = False  # it must be the bound that stopped it, not a failed launch
            if not ok:
                fails.append(f"{name}: got {got} ({why})")
        # green's rule: only a counted summary with nothing failing passes.
        for got_, want_ in ((None, 1), ({"broken": "x"}, 1), (red, 1), (green, 0),
                            ({**green, "afailed": 1}, 1), ({**green, "failed": 1}, 1)):
            if green_status(got_)[0] != want_:
                fails.append(f"green_status({got_}) = {green_status(got_)[0]}, want {want_}")
        # counts() itself: absent → None; present → run with the given command.
        if counts(os.path.join(d, "absent")) is not None:
            fails.append("counts() of an absent binary is not None")
        gr = script("green_counts", summary_then(0, summary_green))
        got = counts(gr[1], cmd=gr, **small)
        if got != green:
            fails.append(f"counts() of a green fake: {got}")
        # The time bound: a 600 s sleep is cut in about the bound, and the sleeper is gone.
        pidf = os.path.join(d, "slow.pid")
        t0 = time.monotonic()
        _, broke, _ = run(script("slow", f"echo $$ > '{pidf}'\nexec sleep 600\n"), **small)
        took = time.monotonic() - t0
        sleeper = int(open(pidf).read().strip()) if os.path.exists(pidf) else None
        if not broke or took > 10 or sleeper is None or not _wait_gone(sleeper):
            fails.append(f"slow: a 600 s run was not cut at the 1.5 s bound (took {took:.1f} s, "
                         f"verdict {broke!r}, sleeper {sleeper})")
        if linux:
            # The child dies with the tool: SIGTERM a helper process that is inside run() — the
            # sleeper must not outlive it (PR_SET_PDEATHSIG; header).
            pidf2 = os.path.join(d, "orphan.pid")
            helper = subprocess.Popen(
                [sys.executable, "-c",
                 f"import sys; sys.path.insert(0, {here!r}); import suite_run as s; "
                 f"s.run(['/bin/sh', '-c', 'echo $$ > ' + {shlex.quote(pidf2)!r} + '; exec sleep 600'], timeout=600)"],
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            end = time.monotonic() + 10
            while not os.path.exists(pidf2) and time.monotonic() < end:
                time.sleep(0.05)
            time.sleep(0.2)
            helper.send_signal(signal.SIGTERM)
            helper.wait(timeout=10)
            orphan = int(open(pidf2).read().strip()) if os.path.exists(pidf2) else None
            if orphan is None or not _wait_gone(orphan):
                fails.append("orphan: the suite outlived a SIGTERM to the tool running it "
                             "(PR_SET_PDEATHSIG not applied?)")
                if orphan:
                    try:
                        os.kill(orphan, signal.SIGKILL)
                    except OSError:
                        pass
            # The clamp where the limits are SET: under a caller whose SOFT limit is 1 GiB (its hard
            # one left as it is — unlimited, or a wrapper's `ulimit`), the child's address space reads
            # 1 GiB soft AND hard — the caller's tighter bound kept, the hard limit lowered to it —
            # and its core and CPU limits as set.
            limits = subprocess.run(
                [sys.executable, "-c",
                 "import resource, sys; "
                 "h = resource.getrlimit(resource.RLIMIT_AS)[1]; "
                 "want = 1024 ** 3 if h == resource.RLIM_INFINITY else min(1024 ** 3, h); "
                 "resource.setrlimit(resource.RLIMIT_AS, (want, h)); print('WANT', want); "
                 f"sys.path.insert(0, {here!r}); import suite_run as s; "
                 "print(s.run(['/bin/sh', '-c', 'cat /proc/self/limits'], timeout=20)[0])"],
                capture_output=True, text=True, timeout=60).stdout
            w = re.search(r"WANT (\d+)", limits)
            want_as = w.group(1) if w else str(1024 ** 3)
            m = re.search(r"Max address space\s+(\S+)\s+(\S+)", limits)
            c = re.search(r"Max core file size\s+(\S+)\s+(\S+)", limits)
            u = re.search(r"Max cpu time\s+(\S+)\s+(\S+)", limits)
            if not (m and m.groups() == (want_as, want_as)):
                fails.append(f"clamp: under a caller whose soft limit is {want_as} the child's address "
                             f"space reads {m.groups() if m else 'nothing'} (want it soft and hard)")
            if not (c and c.groups() == ("0", "0")):
                fails.append(f"core: the child's core limit reads {c.groups() if c else 'nothing'} (want 0)")
            if not (u and u.groups() == (str(CPU_SECONDS), str(CPU_SECONDS))):
                fails.append(f"cpu: the child's CPU limit reads {u.groups() if u else 'nothing'} "
                             f"(want {CPU_SECONDS} s)")
            # The production bounds: run() with NO bounds given reads back 4 GiB / 256 MiB / 600 s (each
            # clamped to this process's own limits, exactly as the child gets them).
            import resource
            text, _, _ = run(["/bin/sh", "-c", "cat /proc/self/limits"], timeout=20)
            for name, which, want in (("Max address space", "RLIMIT_AS", MEMORY_BYTES),
                                      ("Max file size", "RLIMIT_FSIZE", OUTPUT_BYTES),
                                      ("Max cpu time", "RLIMIT_CPU", CPU_SECONDS)):
                soft, hard = resource.getrlimit(getattr(resource, which))
                exp = str(_cap(want, soft, hard, resource.RLIM_INFINITY))
                got = re.search(name + r"\s+(\S+)\s+(\S+)", text)
                if not (got and got.groups() == (exp, exp)):
                    fails.append(f"defaults: {name} reads {got.groups() if got else 'nothing'}, want {exp}")
        else:
            notes.append(f"the process-death tie and /proc readback are Linux's: skipped on {sys.platform}")
    for f in fails:
        print(f"suite run self-test: FAIL {f}", file=sys.stderr)
    if fails:
        return 1
    n = len(cases) + 5 + 1 + 6 + 2 + 1 + 1 + 1 + 4 + 1 + (3 if linux else 0)
    print(f"suite run self-test: OK ({n} cases: the defaults; the bounds' arithmetic; {len(cases)} fake "
          f"suites — red, tail, every caught-bad_alloc form and a look-alike, a chunk straddle, a "
          f"green summary with a bad exit, output, a crash, no summary{', memory' if enforced else ''}; "
          f"green's rule; run()'s defaults; counts() adds no bound; main()'s green exit; counts(); the time bound"
          f"{'; the death tie; the clamp, core and CPU limits read back; the production bounds read back' if linux else ''})"
          + (f" — NOTE: {'; '.join(notes)}" if notes else ""))
    return 0


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "show"
    if cmd not in ("self-test", "show", "green"):
        print(f"usage: {sys.argv[0]} green|show|self-test", file=sys.stderr)
        return 2
    if cmd == "self-test":
        return self_test()
    got = counts()
    if cmd == "show":
        print(got)
        return 0
    status, message = green_status(got)
    print(message, file=sys.stderr if status else sys.stdout)
    return status


if __name__ == "__main__":
    sys.exit(main())
