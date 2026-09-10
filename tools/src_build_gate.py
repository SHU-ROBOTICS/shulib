#!/usr/bin/env python3
"""src build gate — src/ compiles AND links for the V5, every robot variant (GATE1; third
variant at R3b Session 2).

WHY THIS EXISTS
---------------
CI cross-compiles every header under include/shulib/ for the V5 and holds that line
hard — but until GATE1 it never compiled src/ itself: not main.cpp, not bench_r3a.cpp.
The host build does not either (test/CMakeLists.txt globs test/*_test.cpp). The only
thing standing between a broken src/ and the robot was somebody running `make` by hand,
and the Makefile's own comment records what that cost: src/ compiled with NO warning
flags until 2026-08-18, and a data abort shipped to the robot as a result.

This gate drives the REAL `make` — never a re-implemented compile line, which would be
a second source of truth drifting from common.mk — and it is the real BUILD, link
included: measured 2026-08-19, the old soft-float firmware/liblvgl ABI blocker is gone
and `make` links end to end at arm-none-eabi-g++ 13.2.1 with CXX_STANDARD:=gnu++20.

THE VARIANTS (the Makefile's validated ROBOT switch; anything else is an $(error))
-----------------------------------------------------------------------------------
  ROBOT=bench   (default)  the measured tank bench bot — boots the bench tester
  ROBOT=xdrive             the invented X-drive wiring (HA-111)      -DSHULIB_ROBOT_XDRIVE_INVENTED
  ROBOT=tank               the 2026 tank chassis, robot two          -DSHULIB_ROBOT_TANK_2026
                           (R3b Session 2: boots the bench tester like bench; its chassis
                           table ships UNSET; no library graph is built for it yet)

WHAT `check` DOES, per variant (bench, then xdrive, then tank)
--------------------------------------------------------------
 1. Removes bin/ and .d/ (exactly what `make clean` removes; both gitignored, so the
    tree stays clean). make is INCREMENTAL: a gate that compiles nothing reports green,
    and that stale-artifact class nearly faked a mutation result in R3b. Freshness is
    therefore structural (delete) AND observable (the compiled-TU count is printed and
    checked against the TU list globbed from src/ itself — generated, never enumerated,
    the A4 precedent: an enumerated gate list rots).
 2. Runs `make ROBOT=<variant> V=1`. V=1 makes common.mk's test_output_2 print every
    full compiler command line, which is where the structural assertions read from.
    EXTRA_CXXFLAGS is NEVER passed on the command line — that is precisely how the
    build hash used to get dropped (GATE1 §4.1).
 3. Fails on any compiler/linker error, naming the file.
 4. Applies the warning policy below.
 5. Asserts the variant define and the build hash really landed (structural: the -D
    flags on every C++ compile line; end-to-end: the `git describe` string must appear
    in the bytes of bin/hot.package.elf). This encodes the §4.1 defect permanently:
    the documented variant flag used to be a silent no-op, and the one working
    invocation silently shipped a hash-less binary.
    THE DEFINE CHECK IS TOKEN-EXACT (R3b Session 2): the compile line is split on
    whitespace and the variant's define must be a WHOLE token, and every OTHER variant's
    define must be absent as a token. A substring check was defeated by GATE1's own
    mutation 5b (a look-alike `-DSHULIB_ROBOT_XDRIVE_INVENTED_BROKEN`), which the
    warning-count detector then caught for xdrive — but for tank that detector cannot
    fire (see the differential below), so the structural check has to be exact on its own.
 6. Asserts the VARIANT IDENTITY BEACON (R3b Session 2): src/main.cpp defines one
    string per variant — `shulib-robot-variant=bench|xdrive|tank` — under the SAME
    preprocessor facts that select the wiring, and prints it at boot so it is linked in.
    The expected beacon must be in the bytes of bin/hot.package.elf and the other two
    must NOT be. This is the end-to-end proof that the `#if` in the SOURCE took the
    branch the Makefile asked for — the one case no command-line check can see is a
    macro renamed in main.cpp, and a tank build that silently carried the bench bot's
    chassis table onto robot two is HA-111's defect class again.
 7. Asserts bin/hot.package.bin exists — the link is part of the gate.

Across the builds, the VARIANT DIFFERENTIAL (the brief's warning-count detector):
the bench build must emit at least one -Wunused-function from src/main.cpp (the invented
X-drive wiring is dead code there — the Makefile documents those warnings as the correct,
informative evidence of exactly that) and the xdrive build must emit none from
src/main.cpp (the wiring is live). This catches what the flag check cannot: the define
passed on the command line but the #ifdef in main.cpp renamed or broken.
  KNOWN COUPLING, stated so it is a decision and not a surprise: if a future chunk
  deletes the invented X-drive wiring, or makes it live in both variants, this
  differential is the assertion to update — it restates the Makefile's "the warnings are
  how you see the wiring is dead" rationale and should change only when that does.
  (The documented count is TWO since R3b Session 2 — robot() and portMapString();
  shaped() moved into include/shulib/teleop/stick_mapping.hpp. The assertion is >= 1.)
  TANK BEHAVES LIKE BENCH here (R3b Session 2): the X-drive wiring is dead code in the
  tank build too, so tank must also emit >= 1 -Wunused-function from src/main.cpp. That
  means the warning-count detector CANNOT tell a tank build from a bench build — which
  is exactly why the beacon (step 6) exists: it is the only detector that can see a tank
  build that silently became a bench build.

THE WARNING POLICY (GATE1 §4.3)
-------------------------------
  errors                                 -> FAIL, naming the file
  warnings from a vendored tree          -> IGNORED BY PATH (include/liblvgl/,
                                            include/pros/ — third-party code we do not
                                            edit; today: 1 x
                                            -Wdeprecated-enum-enum-conversion from
                                            include/liblvgl/core/lv_obj_style.h).
                                            Never by category: killing the category
                                            would silence it in OUR code too.
  our warnings, category in ALLOWED      -> allowed. Today ALLOWED is exactly
                                            {-Wunused-function}, the only category with
                                            a written justification (Makefile:15-32).
  any other warning, anywhere in ours    -> FAIL

  Allowed by CATEGORY, never by file+line: line numbers move with every edit, and a
  gate that needs a re-baseline per commit gets switched off.

  THE RESIDUAL HOLE, named so it is a known limit instead of a surprise: a genuinely
  NEW dead function in src/ passes this gate — category allowance cannot tell a new
  dead function from the documented ones. That is the accepted cost of not
  re-baselining line numbers. "A gate's exclusion list is where its holes live" (D3);
  this paragraph is the exclusion list.

MISSING TOOLCHAIN = FAIL, NEVER SKIP
------------------------------------
If arm-none-eabi-g++ is absent the gate FAILS LOUDLY before touching make. A gate that
skip-and-passes when its compiler is missing is worse than no gate: the green tick lies.
(Likewise: `git describe` yielding nothing fails the hash assertion rather than skipping
it — every environment this gate supports, local checkout and CI checkout alike, has
git; a build_info-style hash-less binary is legal at RUNTIME, but a gate that cannot
verify identity must say so in red, not in green.)

USAGE — one command locally, the identical command in CI (arm-compile-gate job)
-------------------------------------------------------------------------------
  python3 tools/src_build_gate.py check       # the gate (also the no-arg default)
  python3 tools/src_build_gate.py self-test   # prove the detectors fire: plant each
                                              # fault, run the REAL build, confirm the
                                              # verdict, restore byte-exact
"""

import os
import re
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARM_GXX = "arm-none-eabi-g++"

# The variants, in the order the gate builds them, with the define each one must carry as
# a whole compile-line token (None = must carry NONE of the others) and the identity beacon
# each one's linked package must contain (and the others must not). Both tables mirror
# the Makefile's ROBOT block and src/main.cpp's #if chain; a variant added to one place and
# not the others fails here, which is the point of having the table.
VARIANTS = ("bench", "xdrive", "tank")
VARIANT_DEFINES = {
    "bench": None,
    "xdrive": "-DSHULIB_ROBOT_XDRIVE_INVENTED",
    "tank": "-DSHULIB_ROBOT_TANK_2026",
}
VARIANT_BEACONS = {
    "bench": b"shulib-robot-variant=bench",
    "xdrive": b"shulib-robot-variant=xdrive",
    "tank": b"shulib-robot-variant=tank",
}
# Which variants leave the invented X-drive wiring DEAD (>= 1 -Wunused-function from
# src/main.cpp) versus LIVE (exactly 0). See the header's VARIANT DIFFERENTIAL.
XDRIVE_WIRING_DEAD_IN = ("bench", "tank")

# Vendored third-party trees: warnings here are ignored BY PATH (never by category).
VENDOR_PREFIXES = ("include/liblvgl/", "include/pros/")

# The only warning category with a written justification (Makefile:15-32): the
# bench/tank-variant -Wunused-function warnings from src/main.cpp are correct, informative
# output. Anything else arriving in our code should fail and be considered.
ALLOWED_CATEGORIES = {"-Wunused-function"}

# Source extensions, mirroring the Makefile's CEXTS / ASMEXTS / CXXEXTS.
CXX_EXTS = (".cpp", ".c++", ".cc")
ALL_EXTS = CXX_EXTS + (".c", ".s", ".S")

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
DIAG = re.compile(
    r"^(?P<path>[^:\s][^:]*?):(?P<line>\d+):(?:\d+:)?\s*"
    r"(?P<kind>fatal error|error|warning):\s*(?P<msg>.*)$")
CATEGORY = re.compile(r"\[(-W[A-Za-z0-9=+._-]+)\]\s*$")
COMPILED = re.compile(r"^Compiled (\S+) \[(OK|WARNINGS|ERRORS)\]")


def _norm(path):
    while path.startswith("./"):
        path = path[2:]
    return path


def git_describe():
    """The exact incantation the Makefile uses for SHULIB_GIT_HASH."""
    try:
        r = subprocess.run(["git", "describe", "--always", "--dirty", "--abbrev=7"],
                           cwd=REPO, capture_output=True, text=True, timeout=30)
        return r.stdout.strip() if r.returncode == 0 else ""
    except Exception:
        return ""


def expected_tus():
    """Every TU make will compile, globbed from src/ — generated, never enumerated."""
    out = []
    for root, _dirs, files in os.walk(os.path.join(REPO, "src")):
        for f in files:
            if f.endswith(ALL_EXTS):
                out.append(os.path.relpath(os.path.join(root, f), REPO))
    return sorted(out)


def find_compiler(path_env=None):
    """Resolve the cross-compiler. path_env overrides $PATH so the self-test can prove
    the missing-toolchain verdict without uninstalling a compiler."""
    return shutil.which(ARM_GXX, path=path_env)


class BuildResult:
    def __init__(self):
        self.exit = None
        self.text = ""            # ANSI-stripped combined output
        self.errors = []          # (path, line, msg)
        self.vendor = []          # (path, line, category)
        self.allowed = []         # (path, line, category)
        self.disallowed = []      # (path, line, category, msg)
        self.compiled = []        # TU paths make actually compiled this run
        self.cmdline = {}         # TU path -> full compiler command line (V=1)
        self.hash_expected = ""   # git describe at build time


def run_build(robot, path_env=None):
    """One structurally fresh `make ROBOT=<robot> V=1`, parsed."""
    for d in ("bin", ".d"):  # what `make clean` removes; both gitignored (landmine 3/8)
        shutil.rmtree(os.path.join(REPO, d), ignore_errors=True)

    b = BuildResult()
    b.hash_expected = git_describe()  # same moment as make's parse-time $(shell ...)

    env = dict(os.environ)
    if path_env is not None:
        env["PATH"] = path_env
    p = subprocess.run(["make", f"ROBOT={robot}", "V=1"],
                       cwd=REPO, capture_output=True, text=True, env=env, timeout=600)
    b.exit = p.returncode
    b.text = ANSI.sub("", p.stdout + p.stderr)

    tus = expected_tus()
    for line in b.text.splitlines():
        m = COMPILED.match(line)
        if m:
            b.compiled.append(_norm(m.group(1)))
            continue
        for tu in tus:
            if " -c " in line and line.rstrip().endswith(" " + tu):
                b.cmdline[tu] = line
        m = DIAG.match(line)
        if not m:
            continue
        path, lno, kind, msg = (_norm(m.group("path")), m.group("line"),
                                m.group("kind"), m.group("msg"))
        if kind in ("error", "fatal error"):
            b.errors.append((path, lno, msg))
            continue
        cm = CATEGORY.search(msg)
        cat = cm.group(1) if cm else "(uncategorized)"
        if path.startswith(VENDOR_PREFIXES):
            b.vendor.append((path, lno, cat))
        elif cat in ALLOWED_CATEGORIES:
            b.allowed.append((path, lno, cat))
        else:
            # fail-closed: an uncategorized warning in our code is disallowed too
            b.disallowed.append((path, lno, cat, msg))
    return b


def verdict(robot, b):
    """Apply the policy to one build. Returns (ok, [problem strings])."""
    problems = []

    if b.errors:
        for path, lno, msg in b.errors:
            problems.append(f"ERROR in {path}:{lno} — {msg}")
    if b.exit != 0 and not b.errors:
        tail = "\n      ".join(b.text.splitlines()[-15:])
        problems.append(f"make exited {b.exit} with no parsed compiler error — "
                        f"output tail:\n      {tail}")
    if b.exit != 0 or b.errors:
        return False, problems  # nothing downstream is meaningful on a failed build

    # Freshness, observable: every TU in src/ must have been compiled THIS run.
    exp = expected_tus()
    if sorted(b.compiled) != exp:
        problems.append(
            f"compiled {len(b.compiled)}/{len(exp)} TUs ({sorted(b.compiled)} vs "
            f"expected {exp}) — an incremental no-op or a missed source file")

    for path, lno, cat, msg in b.disallowed:
        problems.append(f"disallowed warning {cat} at {path}:{lno} — {msg}")

    hot_bin = os.path.join(REPO, "bin", "hot.package.bin")
    if not os.path.exists(hot_bin):
        problems.append("bin/hot.package.bin missing — the link is part of this gate")

    # Structural §4.1 assertions, read off the real V=1 compile command lines. TOKEN-EXACT:
    # a look-alike define (GATE1 mutation 5b) is not the define.
    want_define = VARIANT_DEFINES[robot]
    other_defines = [d for v, d in VARIANT_DEFINES.items() if v != robot and d is not None]
    for tu in exp:
        if not tu.endswith(CXX_EXTS):
            continue
        cmd = b.cmdline.get(tu)
        if cmd is None:
            problems.append(f"no compile command line captured for {tu} "
                            f"(V=1 output changed shape?)")
            continue
        tokens = cmd.split()
        if b.hash_expected and "-DSHULIB_BUILD_HASH=" not in cmd:
            problems.append(f"{tu} compiled WITHOUT -DSHULIB_BUILD_HASH — "
                            f"the §4.1 hash-drop defect is back")
        if want_define is not None and want_define not in tokens:
            problems.append(f"ROBOT={robot} but {tu} compiled without the whole token "
                            f"{want_define} — the variant switch is a silent no-op "
                            f"again (§4.1)")
        for other in other_defines:
            if other in tokens:
                problems.append(f"ROBOT={robot} but {tu} carries another variant's "
                                f"define {other}")

    # End-to-end, from the bytes of the linked ELF: the hash string must survive into it
    # (exactly the `strings` measurement that exposed §4.1's second half), and so must
    # THIS variant's identity beacon — and no other variant's (header, step 6).
    elf = os.path.join(REPO, "bin", "hot.package.elf")
    elf_bytes = b""
    if os.path.exists(elf):
        with open(elf, "rb") as f:
            elf_bytes = f.read()
    if b.hash_expected:
        if os.path.exists(elf) and b.hash_expected.encode() not in elf_bytes:
            problems.append(f"build hash '{b.hash_expected}' NOT in "
                            f"bin/hot.package.elf — the binary has no identity")
    else:
        problems.append("git describe yielded nothing — build-hash identity cannot be "
                        "asserted (a skip is not a pass; fix git, do not silence this)")
    if os.path.exists(elf):
        if VARIANT_BEACONS[robot] not in elf_bytes:
            problems.append(
                f"ROBOT={robot} but the beacon {VARIANT_BEACONS[robot].decode()!r} is NOT "
                f"in bin/hot.package.elf — src/main.cpp's #if chain did not take the "
                f"{robot} branch (a renamed macro? the define landed but the source "
                f"never saw it)")
        for v, beacon in VARIANT_BEACONS.items():
            if v != robot and beacon in elf_bytes:
                problems.append(
                    f"ROBOT={robot} but bin/hot.package.elf carries the {v} beacon "
                    f"{beacon.decode()!r} — this is a {v} build wearing a {robot} label")

    return not problems, problems


def _main_cpp_unused_function_count(b):
    return sum(1 for path, _l, cat in b.allowed
               if path == "src/main.cpp" and cat == "-Wunused-function")


def _summary(robot, b):
    return (f"  [{robot:6}] exit {b.exit}, {len(b.compiled)}/{len(expected_tus())} TUs "
            f"compiled, warnings: {len(b.allowed)} allowed "
            f"({', '.join(sorted(ALLOWED_CATEGORIES))}), {len(b.vendor)} vendor-ignored, "
            f"{len(b.disallowed)} disallowed, hash "
            f"{'asserted (' + b.hash_expected + ')' if b.hash_expected else 'UNAVAILABLE'}"
            f", beacon checked")


def cmd_check(path_env=None, quiet=False):
    def say(msg):
        if not quiet:
            print(msg)

    if find_compiler(path_env) is None:
        if not quiet:  # quiet is only ever the self-test proving this exact verdict
            print(f"src build gate: FAIL — {ARM_GXX} not found on PATH.\n"
                  f"  The gate does NOT skip when its compiler is missing: a green "
                  f"tick that lies is worse than no gate.\n"
                  f"  Install it (CI: apt-get install gcc-arm-none-eabi) and re-run.",
                  file=sys.stderr)
        return 1

    builds = {}
    all_problems = []
    for robot in VARIANTS:
        b = run_build(robot, path_env)
        builds[robot] = b
        ok, problems = verdict(robot, b)
        say(_summary(robot, b))
        if not ok:
            all_problems += [f"[{robot}] {p}" for p in problems]

    # The variant differential (see the header: the brief's warning-count detector).
    if not all_problems:
        for robot in VARIANTS:
            n = _main_cpp_unused_function_count(builds[robot])
            if robot in XDRIVE_WIRING_DEAD_IN and n < 1:
                all_problems.append(
                    f"[variant] {robot} build emitted no -Wunused-function from "
                    f"src/main.cpp — either the variant switch built the wrong variant, or "
                    f"the invented X-drive wiring is no longer dead code; if the code "
                    f"changed on purpose, update this differential AND the Makefile's "
                    f"WARNFLAGS rationale together")
            if robot not in XDRIVE_WIRING_DEAD_IN and n != 0:
                all_problems.append(
                    f"[variant] {robot} build emitted {n} -Wunused-function from "
                    f"src/main.cpp — the X-drive wiring should be LIVE in this variant; "
                    f"ROBOT={robot} looks like a silent no-op (§4.1)")

    if all_problems:
        print("\nsrc build gate: FAIL", file=sys.stderr)
        for p in all_problems:
            print(f"  {p}", file=sys.stderr)
        return 1

    say(f"src build gate: PASS (all {len(VARIANTS)} variants link; policy: errors fail, "
        f"vendor warnings path-ignored, {'/'.join(sorted(ALLOWED_CATEGORIES))} allowed by "
        f"category, all else fails; defines token-exact; beacons asserted)")
    return 0


# ── self-test ──────────────────────────────────────────────────────────────────────
# Prove each detector CAN fire: plant the fault, run the REAL build against the planted
# tree, confirm the verdict, restore byte-exact. A gate nobody has seen go red is a
# rumour (doc_staleness_audit.py precedent).

SRC_PLANT = os.path.join(REPO, "src", "bench_r3a.cpp")           # unguarded in both variants
VENDOR_PLANT = os.path.join(REPO, "include", "liblvgl", "core", "lv_obj_style.h")
MAKEFILE = os.path.join(REPO, "Makefile")


class _plant:
    """Append `addition` to `path` for the duration of the block; restore byte-exact."""

    def __init__(self, path, addition):
        self.path, self.addition = path, addition

    def __enter__(self):
        with open(self.path, "rb") as f:
            self.original = f.read()
        with open(self.path, "ab") as f:
            f.write(self.addition.encode())

    def __exit__(self, *exc):
        with open(self.path, "wb") as f:
            f.write(self.original)
        with open(self.path, "rb") as f:
            assert f.read() == self.original, f"restore failed for {self.path}"
        return False


class _replace:
    """Replace the ONE occurrence of `old` in `path` with `new` for the duration of the
    block; restore byte-exact. Asserts exactly one occurrence, so a plant that silently
    matched nothing (the §4.1 class, inside the gate's own self-test) cannot pass."""

    def __init__(self, path, old, new):
        self.path, self.old, self.new = path, old.encode(), new.encode()

    def __enter__(self):
        with open(self.path, "rb") as f:
            self.original = f.read()
        assert self.original.count(self.old) == 1, \
            f"plant target must occur exactly once in {self.path}: {self.old!r}"
        with open(self.path, "wb") as f:
            f.write(self.original.replace(self.old, self.new))

    def __exit__(self, *exc):
        with open(self.path, "wb") as f:
            f.write(self.original)
        with open(self.path, "rb") as f:
            assert f.read() == self.original, f"restore failed for {self.path}"
        return False


def do_self_test():
    if find_compiler() is None:
        print(f"src build gate self-test: FAIL — {ARM_GXX} not found; the self-test "
              f"runs real builds and does not skip.", file=sys.stderr)
        return 1

    failures = []
    checks = 0

    def expect(cond, msg):
        nonlocal checks
        checks += 1
        if not cond:
            failures.append(msg)

    # 1: a syntax error in a src/ file -> FAIL, naming the file (the data-abort class).
    with _plant(SRC_PLANT, "\nthis line is not C++\n"):
        b = run_build("bench")
        ok, problems = verdict("bench", b)
        expect(not ok, "1: a syntax error in src/ was not caught")
        expect(any("bench_r3a.cpp" in p for p in problems),
               "1: the failure did not name the broken file")

    # 2: a NEW disallowed warning in src/ -> FAIL (policy is not "ignore all warnings").
    with _plant(SRC_PLANT, "\nstatic int gate1_selftest_unused_variable = 42;\n"):
        b = run_build("bench")
        ok, problems = verdict("bench", b)
        expect(not ok, "2: a disallowed warning (-Wunused-variable) in src/ passed")
        expect(any("-Wunused-variable" in p and "bench_r3a.cpp" in p for p in problems),
               "2: the failure did not name the category and file")

    # 3: the allowlisted -Wunused-function -> PASS (policy is not "ban all warnings",
    #    which would force deleting the documented dead X-drive wiring).
    with _plant(SRC_PLANT, "\nnamespace { void gate1_selftest_dead_function() {} }\n"):
        b = run_build("bench")
        ok, problems = verdict("bench", b)
        expect(ok, f"3: an allowlisted -Wunused-function FAILED the gate: {problems}")
        expect(any(p == "src/bench_r3a.cpp" for p, _l, _c in b.allowed),
               "3: the planted -Wunused-function never fired (the case proved nothing)")

    # 4: a vendor-path warning of a DISALLOWED category -> PASS (proves the exclusion
    #    is by PATH — the category alone would fail it).
    with _plant(VENDOR_PLANT, "\nstatic int gate1_selftest_vendor_probe = 7;\n"):
        b = run_build("bench")
        ok, problems = verdict("bench", b)
        expect(ok, f"4: a vendor-path warning FAILED the gate: {problems}")
        expect(any("lv_obj_style.h" in p and c == "-Wunused-variable"
                   for p, _l, c in b.vendor),
               "4: the planted vendor warning never fired (the case proved nothing)")

    # 5: nothing planted (the real tree) -> PASS, via the full every-variant check.
    expect(cmd_check(quiet=True) == 0, "5: the gate FAILS on the real, unplanted tree")

    # 6: missing toolchain -> FAIL, loudly, before any make runs (a skip is not a pass).
    with tempfile.TemporaryDirectory() as empty:
        expect(cmd_check(path_env=empty, quiet=True) == 1,
               "6: a missing toolchain did not FAIL the gate (skip-and-pass is the "
               "worst failure mode this gate has)")

    # 7: the Makefile's tank append DROPPED (brief R3b-S2 test 15's mutation: ROBOT=tank
    #    silently builds bench) -> FAIL, structural detector naming the define. The
    #    warning-count detector CANNOT catch this one (tank behaves like bench), which is
    #    why the structural check must fire on its own here.
    tank_line = "override EXTRA_CXXFLAGS+=-DSHULIB_ROBOT_TANK_2026\n"
    with _replace(MAKEFILE, tank_line, "# (self-test plant: tank append dropped)\n"):
        b = run_build("tank")
        ok, problems = verdict("tank", b)
        expect(not ok, "7: ROBOT=tank with its Makefile append dropped PASSED — the "
                       "silent-no-op class is back for the third variant")
        expect(any("-DSHULIB_ROBOT_TANK_2026" in p and "silent no-op" in p
                   for p in problems),
               "7: the failure did not name the missing tank define")
        expect(any("beacon" in p for p in problems),
               "7: the beacon did not ALSO catch the bench build wearing the tank label")

    # 8: a LOOK-ALIKE tank define (GATE1 mutation 5b's shape) -> FAIL on the token-exact
    #    structural check AND on the beacon — a substring check would have passed this,
    #    and for tank nothing else could have caught it.
    with _replace(MAKEFILE, tank_line,
                  "override EXTRA_CXXFLAGS+=-DSHULIB_ROBOT_TANK_2026_BROKEN\n"):
        b = run_build("tank")
        ok, problems = verdict("tank", b)
        expect(not ok, "8: a look-alike tank define PASSED (the substring hole is open)")
        expect(any("whole token -DSHULIB_ROBOT_TANK_2026" in p for p in problems),
               "8: the token-exact structural check did not fire on the look-alike")
        expect(any("beacon" in p and "NOT in" in p for p in problems),
               "8: the beacon check did not fire on the look-alike")

    # 9: the same drop for xdrive (GATE1's hand-run mutation 5a, now permanent) -> FAIL.
    xdrive_line = "override EXTRA_CXXFLAGS+=-DSHULIB_ROBOT_XDRIVE_INVENTED\n"
    with _replace(MAKEFILE, xdrive_line, "# (self-test plant: xdrive append dropped)\n"):
        b = run_build("xdrive")
        ok, problems = verdict("xdrive", b)
        expect(not ok, "9: ROBOT=xdrive with its Makefile append dropped PASSED")
        expect(any("-DSHULIB_ROBOT_XDRIVE_INVENTED" in p for p in problems),
               "9: the failure did not name the missing xdrive define")

    for f in failures:
        print(f"  SELF-TEST FAILURE — {f}", file=sys.stderr)
    if failures:
        return 1
    print(f"src build gate self-test: OK ({checks} detector cases, each against a "
          f"real build)")
    return 0


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else "check"
    if arg == "check":
        return cmd_check()
    if arg == "self-test":
        return do_self_test()
    print(__doc__.split("USAGE")[0], file=sys.stderr)
    print(f"unknown subcommand '{arg}' — use: check | self-test", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
