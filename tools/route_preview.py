#!/usr/bin/env python3
"""THE ROUTE PREVIEW's launcher — one command to SEE a whole auton before the robot runs it.

    python3 tools/route_preview.py "40in forward"                 # robot two's auton, by its exact name
    python3 tools/route_preview.py "40in forward" --place=-48,-48,90
    python3 tools/route_preview.py "40in forward" --robot one     # robot one's auton, on its own mechanisms
    python3 tools/route_preview.py --list                         # every auton's name

WHAT IT DOES (chunk RPV1-A, 2026-11-12; the RPV1 brief's §1.1). It builds the preview program
(`shulib_route_preview`, test/route_preview_main.cpp — the host test build, configured if it is not yet),
runs the named auton from the team's REAL autons file through the REAL library on the HOST TEST PLANT,
exactly as "shulib Auton Tester" runs it, prints the step table, and writes ONE offline page
(build/route_preview/<name>.html, robot one's under one/: the field to scale, the path, every step
numbered with its time, failures in their colours with the library's own sentences). Open the page in
any browser; it fetches nothing. Nothing here runs on a robot; it never writes or changes your code.

THE BOUNDS. The program runs under the test suite's own bounds — IMPORTED from tools/suite_run.py by
path, so the two can never drift: 4 GiB of address space, 256 MiB of output, 600 s of CPU, no core dump,
300 s of wall time, tied to this process so it never outlives it. An auton that loops forever while
logging is stopped by the program itself (the "no-pace stop": exit 1 with the page written, or 3 when the
page could not be written); one that loops without logging is stopped here, at 300 s, and this tool says
so.

THE STAMP is taken NOW, at run time — `git describe --always --dirty --abbrev=7` and the first 12 hex of
the SHA-256 of the robot's autons file — never from the build's configure-time hash, which can read clean
for an edited autons file. Both are printed on the page and in the table.

EXIT CODES. 0 drawn, nothing wrong (a TOUCH note is fine); 1 drawn, and the auton was refused, threw,
went THROUGH a wall or a Loader, or looped without letting time pass — or one of the bounds above cut the
run (the 300 s wall time, a signal: then NO page was written for this run, because the program removes
the auton's last page before it runs it); 2 usage — an unknown name (it lists the real names), an
unknown robot (it lists the robots) or an unknown flag, nothing drawn; 3 the tool itself failed — the
program did not build, the page could not be written, or the preview threw (its words are printed).

    python3 tools/route_preview.py self-test    # proves each bound, the passthrough, the stamp, the real
                                                # command line (through a stub program), and parses every
                                                # route_preview.py line of guide chapter 16
"""

import argparse
import hashlib
import importlib.util
import os
import shlex
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BUILD = os.path.join(REPO, "build", "test")
BINARY = os.path.join(BUILD, "shulib_route_preview")
GUIDE = os.path.join(REPO, "docs", "guide", "16-testing-an-auton-on-the-bench.md")
AUTONS_FILES = {"two": "src/team/autons.cpp", "one": "src/team/robot_one_autons.cpp"}
TIMEOUT_SENTENCE = ("the auton did not finish in {} s of laptop time — a loop of steps that never lets "
                    "time pass?")
NO_PAGE_SENTENCE = ("No page was written for this run (the program removes the auton's last page before it "
                    "runs it).")
EXIT_TOOL_FAILED = 3  # the program's kExitToolFailed (test/route_preview_run.hpp): the tool itself failed


def _load_suite_run():
    """tools/suite_run.py, loaded BY PATH (the bounds' one definition)."""
    spec = importlib.util.spec_from_file_location("shulib_suite_run", os.path.join(REPO, "tools", "suite_run.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


SUITE_RUN = _load_suite_run()


def parser():
    """The command line (the self-test parses guide chapter 16's lines with this same parser)."""
    p = argparse.ArgumentParser(prog="route_preview.py",
                                description="Preview one auton on the host test plant: a page and a table.")
    p.add_argument("name", nargs="?", help="the auton's name exactly as the brain's menu shows it")
    p.add_argument("--list", action="store_true", help="list every auton's name and draw nothing")
    p.add_argument("--place", metavar="X,Y,DEG",
                   help="where a RELATIVE auton starts: inches, inches, degrees (e.g. --place=-48,-48,90)")
    p.add_argument("--robot", default="two", choices=sorted(AUTONS_FILES), help="whose autons (default: two)")
    p.add_argument("--out", metavar="FILE",
                   help="where to write the page (default build/route_preview/<name>.html; robot one's under one/)")
    return p


def _join_place(argv):
    """`--place -48,-48,90` (a value starting with '-') as `--place=-48,-48,90`, which argparse reads."""
    out = []
    i = 0
    while i < len(argv):
        if argv[i] == "--place" and i + 1 < len(argv):
            out.append("--place=" + argv[i + 1])
            i += 2
            continue
        out.append(argv[i])
        i += 1
    return out


def stamp(cwd=REPO):
    """The commit, NOW (header: THE STAMP); "" when git cannot say."""
    try:
        return subprocess.run(["git", "describe", "--always", "--dirty", "--abbrev=7"], cwd=cwd,
                              capture_output=True, text=True, timeout=30, check=False).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ""


def autons_hash(path):
    """The first 12 hex of the SHA-256 of `path`; "" when it cannot be read."""
    try:
        with open(path, "rb") as f:
            return hashlib.sha256(f.read()).hexdigest()[:12]
    except OSError:
        return ""


def run_bounded(cmd, timeout=None, memory=None, output=None, cpu=None):
    """Run `cmd` under the suite's bounds (header), its stdout and stderr this tool's own. Returns
    (exit status — negative for a signal, None when cut at `timeout` — and whether it timed out)."""
    timeout = SUITE_RUN.TIMEOUT_S if timeout is None else timeout
    memory = SUITE_RUN.MEMORY_BYTES if memory is None else memory
    output = SUITE_RUN.OUTPUT_BYTES if output is None else output
    cpu = SUITE_RUN.CPU_SECONDS if cpu is None else cpu
    p = subprocess.Popen(cmd, cwd=REPO, stdin=subprocess.DEVNULL,
                         preexec_fn=SUITE_RUN._child_setup(memory, output, cpu, os.getpid()))
    try:
        return p.wait(timeout=timeout), False
    except subprocess.TimeoutExpired:
        p.kill()
        p.wait()
        return None, True
    except BaseException:
        p.kill()
        raise


def build():
    """Configure build/test if it is not yet, then build the preview program. False if either fails."""
    if not os.path.exists(os.path.join(BUILD, "CMakeCache.txt")):
        if subprocess.run(["cmake", "-S", "test", "-B", "build/test"], cwd=REPO, check=False).returncode != 0:
            print("route preview: could not configure the host build (cmake -S test -B build/test)", file=sys.stderr)
            return False
    jobs = str(max(1, min(8, os.cpu_count() or 1)))
    if subprocess.run(["cmake", "--build", "build/test", "--target", "shulib_route_preview", "-j", jobs],
                      cwd=REPO, check=False).returncode != 0:
        print("route preview: the preview program did not build (see the compiler's words above)", file=sys.stderr)
        return False
    return True


def command(args, binary=BINARY):
    """The program's command line for parsed `args`, the stamp taken now."""
    cmd = [binary]
    if args.name is not None:
        cmd.append(args.name)
    if args.list:
        cmd.append("--list")
    if args.place is not None:
        cmd += ["--place", args.place]
    if args.robot != "two":
        cmd += ["--robot", args.robot]
    if args.out is not None:
        cmd += ["--out", os.path.abspath(args.out)]
    s = stamp()
    if s:
        cmd += ["--stamp", s]
    h = autons_hash(os.path.join(REPO, AUTONS_FILES[args.robot]))
    if h:
        cmd += ["--autons-hash", h]
    return cmd


def run(argv, binary=BINARY, build_fn=build, timeout=None):
    """THE COMMAND: parse `argv`, build, run `binary` under the bounds (header), say what a cut run means.
    The self-test runs this very function with a stub for `binary` and `build_fn`."""
    args = parser().parse_args(_join_place(argv))
    if not build_fn():
        return EXIT_TOOL_FAILED
    timeout = SUITE_RUN.TIMEOUT_S if timeout is None else timeout
    rc, timed_out = run_bounded(command(args, binary), timeout=timeout)
    if timed_out:
        print("route preview: " + TIMEOUT_SENTENCE.format(timeout) + " " + NO_PAGE_SENTENCE, file=sys.stderr)
        return 1
    if rc is not None and rc < 0:
        print(f"route preview: the program was killed by signal {-rc} — a crash, or one of its bounds "
              f"({SUITE_RUN.MEMORY_BYTES // 1024 ** 3} GiB of memory, {SUITE_RUN.OUTPUT_BYTES // 1024 ** 2} MiB of "
              f"output, {SUITE_RUN.CPU_SECONDS} s of CPU). " + NO_PAGE_SENTENCE, file=sys.stderr)
        return 1
    return rc


def main(argv):
    if argv[:1] == ["self-test"]:
        return self_test()
    return run(argv)


# ── self-test ───────────────────────────────────────────────────────────────────────────────────


def guide_lines(path=GUIDE):
    """Every `route_preview.py` command line inside a ```bash block of guide chapter 16."""
    lines = []
    in_bash = False
    with open(path, encoding="utf-8") as f:
        for raw in f:
            line = raw.rstrip("\n")
            if line.strip().startswith("```"):
                in_bash = line.strip() == "```bash" if not in_bash else False
                continue
            if in_bash and "route_preview.py" in line:
                lines.append(line.split("#", 1)[0].strip() if " #" in line else line.strip())
    return lines


def guide_args(line):
    """The arguments after `tools/route_preview.py` on a guide line."""
    words = shlex.split(line)
    at = next(i for i, w in enumerate(words) if w.endswith("route_preview.py"))
    return words[at + 1:]


STUB = """#!{py}
import json, os, resource, signal, sys, time
lim = {{n: list(resource.getrlimit(getattr(resource, n))) for n in ("RLIMIT_AS", "RLIMIT_FSIZE", "RLIMIT_CPU",
                                                                   "RLIMIT_CORE")}}
with open(os.environ["RP_STUB_OUT"], "w") as f:
    json.dump({{"argv": sys.argv[1:], "limits": lim}}, f)
if os.environ.get("RP_STUB_SLEEP"):
    time.sleep(float(os.environ["RP_STUB_SLEEP"]))
if os.environ.get("RP_STUB_KILL"):
    os.kill(os.getpid(), signal.SIGKILL)
sys.exit(int(os.environ.get("RP_STUB_EXIT", "0")))
"""


def stub_test(check, stamp_now, head):
    """Self-test 10 (above): run() end to end with a stub program — the bounds reach it, every flag and
    the stamp reach it, its exit code comes back, and a cut or killed run is 1 with its sentence."""
    import contextlib
    import io
    import json
    with tempfile.TemporaryDirectory(dir=os.path.join(REPO, "build")) as tmp:
        stub = os.path.join(tmp, "stub_program")
        with open(stub, "w") as f:
            f.write(STUB.format(py=sys.executable))
        os.chmod(stub, 0o755)
        out = os.path.join(tmp, "seen.json")
        saved = {k: os.environ.get(k) for k in ("RP_STUB_OUT", "RP_STUB_SLEEP", "RP_STUB_KILL", "RP_STUB_EXIT")}

        def call(argv, exit_code=0, sleep=None, kill=False, timeout=None, built=True):
            os.environ["RP_STUB_OUT"] = out
            os.environ["RP_STUB_EXIT"] = str(exit_code)
            for k, v in (("RP_STUB_SLEEP", sleep), ("RP_STUB_KILL", "1" if kill else None)):
                if v is None:
                    os.environ.pop(k, None)
                else:
                    os.environ[k] = str(v)
            if os.path.exists(out):
                os.remove(out)
            err = io.StringIO()
            with contextlib.redirect_stderr(err):
                rc = run(argv, binary=stub, build_fn=lambda: built, timeout=timeout)
            seen = None
            if os.path.exists(out):
                with open(out, encoding="utf-8") as f:
                    seen = json.load(f)
            return rc, seen, err.getvalue()

        try:
            import resource

            def bound(which, want):  # the bound suite_run applies: the smaller of `want` and this process's own
                soft, hard = resource.getrlimit(which)
                cap = SUITE_RUN._cap(want, soft, hard, resource.RLIM_INFINITY)
                return [cap, cap]

            rc, seen, _ = call(["40in forward", "--place=-48,-48,90"])
            argv = seen["argv"] if seen else []
            lim = seen["limits"] if seen else {}
            check(rc == 0 and seen is not None, f"run() runs the program and returns its exit 0 (got {rc})")
            check(lim.get("RLIMIT_AS") == bound(resource.RLIMIT_AS, SUITE_RUN.MEMORY_BYTES),
                  f"the program runs under the address-space bound ({lim.get('RLIMIT_AS')})")
            check(lim.get("RLIMIT_CORE") == [0, 0]
                  and lim.get("RLIMIT_FSIZE") == bound(resource.RLIMIT_FSIZE, SUITE_RUN.OUTPUT_BYTES)
                  and lim.get("RLIMIT_CPU") == bound(resource.RLIMIT_CPU, SUITE_RUN.CPU_SECONDS),
                  f"... no core, the output and CPU bounds ({lim})")
            check(argv[:3] == ["40in forward", "--place", "-48,-48,90"],
                  f"the name and --place=X,Y,DEG reach the program as --place X,Y,DEG ({argv[:3]})")
            check("--stamp" in argv and head in argv[argv.index("--stamp") + 1] and
                  argv[argv.index("--stamp") + 1] == stamp_now, f"--stamp is the commit read now, naming HEAD {head}")
            two = autons_hash(os.path.join(REPO, AUTONS_FILES["two"]))
            check("--autons-hash" in argv and argv[argv.index("--autons-hash") + 1] == two,
                  f"--autons-hash is robot two's file's ({two})")
            check("--robot" not in argv, "robot two is the program's default: no --robot passed")
            rc, seen, _ = call(["skills", "--robot", "one"])
            argv = seen["argv"] if seen else []
            check(argv[1:3] == ["--robot", "one"], f"--robot one reaches the program ({argv[1:3]})")
            one = autons_hash(os.path.join(REPO, AUTONS_FILES["one"]))
            check(bool(one) and one != two and "--autons-hash" in argv
                  and argv[argv.index("--autons-hash") + 1] == one,
                  f"--autons-hash is robot ONE's file's ({one}), never robot two's")
            for code in (1, 2, 3):
                rc, _, _ = call(["seven"], exit_code=code)
                check(rc == code, f"the program's exit {code} comes back from run() (got {rc})")
            rc, _, err = call(["seven"], sleep=30, timeout=1)
            check(rc == 1 and TIMEOUT_SENTENCE.format(1) in err and NO_PAGE_SENTENCE in err,
                  f"a run cut at its wall-time bound is exit 1 and says so, and that no page was written (got {rc})")
            rc, _, err = call(["seven"], kill=True)
            check(rc == 1 and "killed by signal 9" in err and NO_PAGE_SENTENCE in err,
                  f"a killed program is exit 1 and says so (got {rc})")
            rc, seen, _ = call(["seven"], built=False)
            check(rc == EXIT_TOOL_FAILED and seen is None, f"a program that did not build is exit 3, nothing run "
                                                            f"(got {rc})")
        finally:
            for k, v in saved.items():
                if v is None:
                    os.environ.pop(k, None)
                else:
                    os.environ[k] = v


def self_test():
    py = sys.executable
    failures = []

    def check(ok, what):
        print(("ok    " if ok else "FAIL  ") + what)
        if not ok:
            failures.append(what)

    # 0. The bounds ARE the suite's (imported, not copied).
    check(SUITE_RUN.MEMORY_BYTES == 4 * 1024 ** 3 and SUITE_RUN.OUTPUT_BYTES == 256 * 1024 ** 2
          and SUITE_RUN.TIMEOUT_S == 300 and SUITE_RUN.CPU_SECONDS == 600,
          "the bounds are tools/suite_run.py's: 4 GiB, 256 MiB, 300 s, 600 s of CPU")
    # 1. Address space: a child bounded at 256 MiB cannot allocate 512 MiB (it could without the bound,
    #    even under a caller's 4 GiB ulimit).
    rc, _ = run_bounded([py, "-c", "b = bytearray(512 * 1024 ** 2)"], memory=256 * 1024 ** 2, timeout=60)
    check(rc not in (0, None), f"the address-space bound applies (a 512 MiB allocation under 256 MiB: exit {rc})")
    rc, _ = run_bounded([py, "-c", "b = bytearray(64 * 1024 ** 2)"], memory=256 * 1024 ** 2, timeout=60)
    check(rc == 0, f"... and a 64 MiB allocation under it runs (exit {rc})")
    # 2. Output: a child bounded at 1 MiB cannot write a 2 MiB file.
    with tempfile.TemporaryDirectory(dir=os.path.join(REPO, "build")) as tmp:
        target = os.path.join(tmp, "big")
        rc, _ = run_bounded([py, "-c", f"open({target!r}, 'wb').write(b'x' * (2 * 1024 ** 2))"],
                            output=1024 ** 2, timeout=60)
        check(rc not in (0, None), f"the output bound applies (2 MiB under 1 MiB: exit {rc})")
    # 3. CPU: a busy child bounded at 1 s of CPU is killed by the kernel.
    rc, _ = run_bounded([py, "-c", "while True: pass"], cpu=1, timeout=60)
    check(rc is not None and rc < 0, f"the CPU bound applies (a busy loop under 1 s: exit {rc})")
    # 4. Wall time: a sleeping child is cut at the timeout, and the tool says so.
    rc, timed_out = run_bounded([py, "-c", "import time; time.sleep(30)"], timeout=1)
    check(timed_out and rc is None, "the wall-time bound cuts a child that never finishes")
    check("did not finish in 300 s of laptop time" in TIMEOUT_SENTENCE.format(SUITE_RUN.TIMEOUT_S),
          "the timeout's sentence names the 300 s bound")
    # 5. No core dump.
    rc, _ = run_bounded([py, "-c", "import resource, sys; sys.exit(0 if resource.getrlimit(resource.RLIMIT_CORE) "
                         "== (0, 0) else 9)"], timeout=60)
    check(rc == 0, f"no core dump: RLIMIT_CORE is 0 in the child (exit {rc})")
    # 6. The exit code is passed through.
    for code in (0, 1, 2, 3):
        rc, _ = run_bounded([py, "-c", f"import sys; sys.exit({code})"], timeout=60)
        check(rc == code, f"exit {code} is passed through (got {rc})")
    # 7. The stamp is taken now: a changed copy of the autons file hashes differently, nothing reconfigured.
    s = stamp()
    head = subprocess.run(["git", "rev-parse", "--short=7", "HEAD"], cwd=REPO, capture_output=True, text=True,
                          check=False).stdout.strip()
    check(bool(s) and bool(head) and head in s, f"the commit stamp is read now ({s!r} names HEAD {head!r})")
    with tempfile.TemporaryDirectory(dir=os.path.join(REPO, "build")) as tmp:
        copy = os.path.join(tmp, "autons.cpp")
        with open(os.path.join(REPO, AUTONS_FILES["two"]), "rb") as f:
            original = f.read()
        with open(copy, "wb") as f:
            f.write(original)
        before = autons_hash(copy)
        with open(copy, "ab") as f:
            f.write(b"\n// an edit\n")
        after = autons_hash(copy)
        check(len(before) == 12 and before != after and before == autons_hash(os.path.join(REPO, AUTONS_FILES["two"])),
              f"the autons hash changes with the file ({before} -> {after})")
    # 8. Every route_preview.py line in guide chapter 16 parses with THIS parser (an unknown flag fails it).
    lines = guide_lines()
    check(len(lines) >= 3, f"guide chapter 16 carries route_preview.py command lines ({len(lines)} found)")
    for line in lines:
        rest = guide_args(line)
        if rest[:1] == ["self-test"]:
            check(True, f"guide line parses: {line}")
            continue
        try:
            parser().parse_args(_join_place(rest))
            check(True, f"guide line parses: {line}")
        except SystemExit:
            check(False, f"guide line does NOT parse: {line}")
    # 9. ... and the parser refuses an unknown flag (so 8 can fail).
    try:
        parser().parse_args(["x", "--colour"])
        check(False, "an unknown flag is refused")
    except SystemExit:
        check(True, "an unknown flag is refused")
    # 10. THE REAL COMMAND — run(), the function main() calls — with a STUB in the program's place: the
    #     stub writes what it was given (its argv and its limits) and exits or sleeps as told.
    stub_test(check, s, head)
    print(f"route_preview self-test: {'FAILED' if failures else 'ok'} ({len(failures)} failure(s))")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
