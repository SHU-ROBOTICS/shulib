#!/usr/bin/env python3
"""upload_auton.py -- build a program, upload it to its slot, and say plainly whether it worked (RMP1-F D5).

    python3 tools/upload_auton.py white              White Bot's Auton Tester (make ROBOT=one PROGRAM=autontest), slot 6
    python3 tools/upload_auton.py white drive        White Bot's Drive (PROGRAM=drive), slot 1 -- the driver controls
    python3 tools/upload_auton.py white competition  White Bot's Competition (PROGRAM=competition), slot 2
    python3 tools/upload_auton.py white bench        Bench Tests (PROGRAM=tester), slot 3; `calibrate` slot 4, `tune` slot 7
    python3 tools/upload_auton.py white all          ALL SIX, one after another (Drive, Competition, the Auton Tester,
                                                     Bench Tests, Calibrate, Tune), and a line for each at the end
    python3 tools/upload_auton.py black ...          Black Bot (ROBOT=tank), the same
    (`one` is White Bot and `two` Black Bot, as before: robot one and robot two in the code and the docs.)
    python3 tools/upload_auton.py self-test

Then `pros upload --slot <its slot> --name "<its name>" --description "<git describe>[ | <the routes>]" . <the brain's
port>` (pros-cli takes the port as an argument after the project folder; the port from `pros lsusb`'s "V5 System Ports",
without a controller; the description is what `route_builder.py robot one` reads back, RMP1-K D5). Prints ✓ BUILT and
✓ UPLOADED, or ✗ and the first real error in plain words. Windows and Linux: Python 3 (`python` on Windows), `make` and
`pros` from PATH (the PROS terminal), else from the PROS toolchain's folder (PROS_TOOLCHAIN, else the one the PROS
extension installed, which it names only in its own PROS Terminal -- a VS Code task has neither: its bin/make, as
`pros make` finds it, and the pros-cli folder beside it). Everything make and pros print goes to build/upload_auton.log
as they print it.
"""
import os
import re
import shutil
import subprocess
import sys
import threading
from pathlib import Path

# Each program: its PROGRAM= word, its slot and its name on the brain (the team lead, 2026-11-20: Drive 1, Competition 2;
# the Auton Tester 6). The tester is the default, as the route builder's Send and the first tasks call it.
PROGRAMS = {"tester": ("autontest", 6, "shulib Auton Tester"), "drive": ("drive", 1, "shulib Drive"),
            "competition": ("competition", 2, "shulib Competition"), "bench": ("tester", 3, "Bench Tests"),
            "calibrate": ("calibrate", 4, "shulib Calibrate"), "tune": ("tune", 7, "shulib Tune")}
ALL = ("drive", "competition", "tester", "bench", "calibrate", "tune")  # `all`: in slot order of use
SLOT, NAME = PROGRAMS["tester"][1], PROGRAMS["tester"][2]
JOBS = os.cpu_count() or 1  # make's parallel jobs (the team lead, 2026-11-20: "i want to easily be able to iterate")
ROBOTS = {"one": "one", "two": "tank"}
NAMES = {"one": "White Bot", "two": "Black Bot"}  # what people see (the team lead, 2026-11-20): the Python tools' one spelling
ALIASES = {"white": "one", "black": "two"}
SKILLS = "src/team/skills_route.cpp"
PROS_HELP = "https://pros.cs.purdue.edu/v5/getting-started/"
TERMINAL = "the PROS terminal (VS Code: PROS, then Integrated Terminal)"
BEGIN = re.compile(r'^// ═══ ROUTE MAKER — route "[^"]*" — BEGIN ═+$')
END = re.compile(r'^// ═══ ROUTE MAKER — route "[^"]*" — END ═+$')
OWED = re.compile(r"static assertion failed: (robot (?:one|two)): OWED -- (.*) \((src/robots/robot_\w+_facts\.hpp)\)")
COMPILE = re.compile(r"^(?P<file>.+?):(?P<line>\d+):(?:\d+:)? (?:fatal )?error: (?P<msg>.+)$")
SKILLS_AT = re.compile(re.escape(SKILLS) + r":(\d+)")
PORT = re.compile(r"^\s*(/dev/\S+|COM\d+)\s+-\s+(.*)$")
ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
NO_BRAIN = "No brain found. Plug the brain in with USB (not the controller), turn it on, then run this again."
IN_REGION = ("That is the route builder's code (between the ROUTE MAKER lines): re-export from the route builder, never "
             "edit it by hand.")


def robot_id(word):
    """'one' or 'two' for one/two/white/black (any capitals), else None."""
    w = word.lower()
    return w if w in ROBOTS else ALIASES.get(w)


def marks():
    """✓, ✗ and → where the terminal can print them, else OK, X and ->."""
    try:
        "✓✗→".encode(sys.stdout.encoding or "ascii")
        return "✓", "✗", "→"
    except (UnicodeEncodeError, LookupError):
        return "OK", "X", "->"


def run(cmd, root, log, timeout=None, env=None):
    """Run `cmd` in `root` (with `env`, else this process's) (stdin closed: a prompt fails instead of waiting), each line into `log` as it is printed (so
    Ctrl+C keeps it): (exit code, or None when it gave no answer in `timeout` s; its output)."""
    out = []

    def copy(pipe):
        with pipe, open(log, "a", encoding="utf-8") as f:
            for raw in pipe:
                out.append(ANSI.sub("", raw.decode("utf-8", errors="replace")))
                f.write(out[-1])
                f.flush()

    with open(log, "a", encoding="utf-8") as f:
        f.write(f"$ {' '.join(cmd)}\n")
    p = subprocess.Popen(cmd, cwd=root, env=env, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    reader = threading.Thread(target=copy, args=(p.stdout,), daemon=True)
    reader.start()
    code, end = None, None
    try:
        code = p.wait(timeout)
    except subprocess.TimeoutExpired:
        end = f"(stopped: no answer in {round(timeout)} s)"
    except KeyboardInterrupt:
        end = "(stopped: Ctrl+C)"
        raise
    finally:
        if p.poll() is None:
            p.kill()
        reader.join(5)  # a child's child may hold the pipe open: never wait on it long
        with open(log, "a", encoding="utf-8") as f:
            f.write(f"\n{end or f'(exit {code})'}\n\n")
    return code, "".join(out)


def region(root):
    """The ROUTE MAKER region of skills_route.cpp: (first line, last line, route names), or None."""
    try:
        lines = (Path(root) / SKILLS).read_text(encoding="utf-8").splitlines()
    except OSError:
        return None
    b = [i for i, x in enumerate(lines, 1) if BEGIN.match(x)]
    e = [i for i, x in enumerate(lines, 1) if END.match(x)]
    if len(b) != 1 or len(e) != 1 or e[0] < b[0]:
        return None
    names = re.findall(r'^\s*e\.name = "([^"]*)";', "\n".join(lines[b[0]:e[0] - 1]), re.M)
    return b[0], e[0], names


def inside(root, line):
    """Is skills_route.cpp's `line` in its ROUTE MAKER region (the two marker lines included)?"""
    r = region(root)
    return bool(r) and r[0] <= line <= r[1]


def from_region(lines, i, root):
    """An error in another file (a library header) that the region caused: GCC's "required from" or "In file included
    from" lines around error line `i` (to the file's Compiled line, or the next error) name a region line."""
    near = []
    for step in (-1, 1):
        j = i + step
        while 0 <= j < len(lines) and not (COMPILE.match(lines[j].strip()) or lines[j].startswith(("Compiled", "make"))):
            near.append(lines[j].replace("\\", "/"))
            j += step
    return any(m and inside(root, int(m.group(1))) for m in map(SKILLS_AT.search, near))


def build_error(text, root):
    """The first real error in make's output, in plain words (a list of lines)."""
    owed = list(dict.fromkeys((m.group(1), m.group(2), m.group(3)) for m in OWED.finditer(text)))
    if owed:
        who, _, ledger = owed[0]
        who = NAMES.get(who.split()[1], who)  # the compiler's "robot one", in the name people use
        return [f"{who}'s build refuses while a fact it needs is OWED (not measured or typed yet):"] + \
               [f"  - {name}" for _, name, _ in owed] + [f"Type them in {ledger} (or ask), then run this again."]
    lines = text.splitlines()
    for i, raw in enumerate(lines):
        m = COMPILE.match(raw.strip())
        if not m:
            continue
        where = m.group("file").replace("\\", "/")
        line = int(m.group("line"))
        said = [f"{where}:{line}: {m.group('msg')}"]
        if where == SKILLS or where.endswith("/" + SKILLS):
            if inside(root, line):
                return said + [IN_REGION]
            return said + ["That is outside the ROUTE MAKER lines (the plumbing): undo the change there, or ask."]
        if from_region(lines, i, root):
            return said + [IN_REGION]
        return said + ["Fix that line (or ask), then run this again."]
    for raw in lines:
        if "undefined reference" in raw or re.search(r"\*\*\*.*Stop\.", raw):
            return [raw.strip(), "Ask the coordinator with these words."]
    last = [x.strip() for x in lines if x.strip()]
    return [last[-1] if last else "make printed nothing.", "Ask the coordinator with these words."]


def brain_port(text, kind="system"):
    """The brain's port from `pros lsusb`, sorted as pros sorts it: its "V5 System Ports" (`kind` "user": its "V5 User
    Ports", where the program's output comes -- RMP1-K D3; a controller is listed in both, by name). (port, None);
    (None, why) when that is not exactly one brain; (None, None) when the list is not in pros's shape (then pros picks)."""
    section, seen, system = None, False, []
    for raw in text.splitlines():
        m = PORT.match(raw)
        if m:
            if section == kind:
                system.append((m.group(1), m.group(2)))
        elif raw.strip():
            low = raw.lower()
            section = "system" if "system port" in low else "user" if "user port" in low else "other"
            seen = seen or section == kind
    if not seen:
        return None, None
    brains = sorted({p for p, d in system if "Controller" not in d})
    if len(brains) == 1:
        return brains[0], None
    if not brains:
        return None, ("Only the controller is plugged in. Plug the BRAIN in with USB (not the controller), then run this "
                      "again." if system else NO_BRAIN)
    if all("Brain" in d for p, d in system if p in brains):
        return None, f"Two brains are plugged in ({', '.join(brains)}): unplug one, then run this again."
    return None, (f"More than one V5 port is plugged in ({', '.join(brains)}): keep only the brain's USB cable in (unplug "
                  "the controller's), then run this again.")


def description(root, names):
    """`pros upload --description`: this checkout's build (`git describe`), and for a program that carries routes (the
    Auton Tester, Competition) the routes, so a check of the brain can say what each slot carries (RMP1-K D5). `names`
    None: the build alone. ASCII (pros reads the brain's slot file as ASCII), at most 60 characters."""
    try:
        got = subprocess.run(["git", "-C", str(root), "describe", "--always", "--dirty"], stdin=subprocess.DEVNULL,
                             capture_output=True, text=True, timeout=30)
        build = got.stdout.strip() if got.returncode == 0 and got.stdout.strip() else "no git"
    except (OSError, subprocess.SubprocessError):
        build = "no git"
    text = build if names is None else f"{build} | {', '.join(names) if names else 'no routes'}"
    return "".join(c for c in text if " " <= c <= "~")[:60]


def upload_error(text, slot=SLOT, name=NAME, desc=""):
    """Why pros upload failed, in plain words (`desc`: the --description this script passed, for the command it prints)."""
    low = text.lower()
    if "no v5" in low or "no device" in low or "no ports" in low or "no port provided" in low:
        return NO_BRAIN
    if "permission denied" in low:
        return "This computer may not use the brain's port. On Linux: sudo usermod -aG dialout $USER, then log out and in."
    if "aborted" in low:  # a question this script cannot answer (which brain; too many files on the brain; ...)
        return (f'pros stopped to ask a question. Run the upload yourself to answer it: pros upload --slot {slot} --name "{name}"'
                + (f' --description "{desc}"' if desc else ""))
    last = [x.strip() for x in text.splitlines() if x.strip()]
    return (last[-1] if last else "pros printed nothing.") + " (Ask the coordinator with these words.)"


def no_answer(seconds):
    return f"pros gave no answer in {round(seconds)} s: unplug the brain's USB cable, plug it back in, then run this again."


def pros_extension():
    """The toolchain folder the PROS extension's one-click install made (VS Code's globalStorage), or None. The extension
    puts it on PATH and in PROS_TOOLCHAIN for its own PROS Terminal only, so a VS Code task (route_builder.py's Send)
    sees neither."""
    base = (os.environ.get("APPDATA") if os.name == "nt" else Path.home() / "Library" / "Application Support"
            if sys.platform == "darwin" else os.environ.get("XDG_CONFIG_HOME") or Path.home() / ".config")
    install = Path(base or ".") / "Code" / "User" / "globalStorage" / "sigbots.pros" / "install"
    return next((str(t) for t in (install / "pros-toolchain-windows" / "usr", install / "pros-toolchain-linux",
                                  install / "pros-toolchain-macos") if base and (t / "bin").is_dir()), None)


def find_tools():
    """(make, pros, the build's environment): each from PATH, else from the PROS toolchain's folder -- make from
    PROS_TOOLCHAIN/bin exactly as `pros make` finds it (and, as `pros make` does, that bin first on the build's PATH,
    so the toolchain's compiler builds), pros from the pros-cli folder the PROS extension installs beside the
    toolchain. With no PROS_TOOLCHAIN, the extension's own folder, as its PROS Terminal has it (that bin first on the
    build's PATH, PROS_TOOLCHAIN set). Never `pros make` itself: when it cannot start make it says so and still exits
    0, and an upload after it would send the OLD program."""
    make, pros, env = shutil.which("make"), shutil.which("pros"), None
    tc = os.environ.get("PROS_TOOLCHAIN") or (None if make and pros else pros_extension())
    if tc:
        bin_ = Path(tc) / "bin"
        if tc != os.environ.get("PROS_TOOLCHAIN"):  # the extension's folder: the build sees it as its PROS Terminal does
            env = dict(os.environ, PROS_TOOLCHAIN=tc, PATH=str(bin_) + os.pathsep + os.environ.get("PATH", ""))
        if not make:
            make = shutil.which("make", path=str(bin_))
            if make:
                env = dict(env or os.environ, PATH=str(bin_) + os.pathsep + os.environ.get("PATH", ""))
        if not pros:
            near = [d for up in (Path(tc).parent, Path(tc).parent.parent) for d in sorted(up.glob("pros-cli-*"))]
            pros = shutil.which("pros", path=os.pathsep.join(map(str, near))) if near else None
    return make, pros, env


def main(argv):
    root, wait = Path(__file__).resolve().parent.parent, 1.0
    while len(argv) >= 2 and argv[0] in ("--root", "--wait"):  # the self-test's stand-in repository and short waits
        root, wait = (Path(argv[1]), wait) if argv[0] == "--root" else (root, float(argv[1]))
        argv = argv[2:]
    if not 1 <= len(argv) <= 2 or not robot_id(argv[0]) or (len(argv) == 2 and argv[1] not in (*PROGRAMS, "all")):
        print("Say which robot, and which program if not the Auton Tester: python3 tools/upload_auton.py white   "
              "(or black; one and two work too; then drive, competition, tester, bench, calibrate, tune or all)")
        return 2
    if len(argv) == 2 and argv[1] == "all":  # each in turn, as if asked one by one; then one line each
        ok, bad, _to = marks()
        done = []
        for name in ALL:
            print(f"\n== {PROGRAMS[name][2]} (slot {PROGRAMS[name][1]}) ==", flush=True)
            code = main([*(["--root", str(root)] if root != Path(__file__).resolve().parent.parent else []),
                         *(["--wait", str(wait)] if wait != 1.0 else []), argv[0], name])
            done.append((name, code))
            if code != 0 and name in ("drive", "competition"):
                pass  # keep going: the other programs may still go up (each says its own ✗)
        print("\n" + "\n".join(f"{ok if c == 0 else bad} slot {PROGRAMS[n][1]} {PROGRAMS[n][2]}" for n, c in done))
        return 0 if all(c == 0 for _n, c in done) else 1
    ok, bad, to = marks()
    which = robot_id(argv[0])
    program, slot, name = PROGRAMS[argv[1] if len(argv) == 2 else "tester"]
    words = {"autontest": "Auton Tester", "drive": "Drive", "competition": "Competition", "tester": "Bench Tests",
             "calibrate": "Calibrate", "tune": "Tune"}[program]
    make, pros, env = find_tools()
    if not make or not pros:
        missing = " and ".join(n for n, p in (("make", make), ("pros", pros)) if not p)
        print(f"{bad} {missing} not found. Run this in {TERMINAL}, or install PROS: {PROS_HELP}")
        return 1
    log = root / "build" / "upload_auton.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    log.write_text("", encoding="utf-8")
    cmd = [make, f"-j{JOBS}", f"ROBOT={ROBOTS[which]}", f"PROGRAM={program}"]  # every core: a full build ~3x faster
    print(f"Building {NAMES[which]}'s {words}: make ROBOT={ROBOTS[which]} PROGRAM={program} (a few minutes the "
          "first time; Ctrl+C stops it)...", flush=True)
    if env and Path(make).parent == Path(env.get("PROS_TOOLCHAIN", "")) / "bin":
        print(f"  (make from the PROS toolchain: {make})", flush=True)
    code, text = run(cmd, root, log, env=env)
    if code != 0:
        print(f"{bad} NOT BUILT")
        print("\n".join("  " + x for x in build_error(text, root)))
        print(f"  (everything make printed: {log})")
        return 1
    print(f"{ok} BUILT", flush=True)
    code, text = run([pros, "lsusb"], root, log, timeout=60 * wait)
    port, why = brain_port(text) if code == 0 else (None, None)  # lsusb failed: pros upload finds the port itself
    if code is None:
        why = no_answer(60 * wait)
    if why:
        print(f"{bad} NOT UPLOADED: {why}")
        return 1
    r = region(root)
    names = r[2] if r and r[2] else []
    desc = description(root, names if program in ("autontest", "competition") else None)
    cmd = [pros, "upload", "--slot", str(slot), "--name", name, "--description", desc] + ([".", port] if port else [])
    print(f"Uploading to slot {slot}{' on ' + port if port else ''}...", flush=True)
    code, text = run(cmd, root, log, timeout=300 * wait)
    if code != 0 or "finished uploading" not in text.lower():
        said = ("pros did not say it finished uploading." if code == 0 else
                no_answer(300 * wait) if code is None else upload_error(text, slot, name, desc))
        print(f"{bad} NOT UPLOADED: {said}")
        print(f"  (everything pros printed: {log})")
        return 1
    if program != "autontest":
        said = {"drive": " (the driver controls are in it)",
                "competition": f" {to} pick the auton on its screen (its driver control is the driver file)"}.get(program, "")
        print(f"{ok} UPLOADED to slot {slot}. On the brain: {name}{said}")
        return 0
    print(f"{ok} UPLOADED to slot {slot}. On the brain: {name} {to} {' or '.join(names or ['your route'])} {to} RUN")
    return 0


# ── self-test: the script against fake `make` and `pros` on PATH; the fake pros takes pros-cli 3.5.6's own grammar
# (pros/cli/upload.py: `upload [OPTIONS] [PATH] [PORT]`, an unknown option is exit 2) and prints its own lsusb shape ──

FAKE = r"""
import os, sys, time
sys.stdout.reconfigure(encoding="utf-8")  # as the real tools print, whatever the console
tool, args = sys.argv[1], sys.argv[2:]
case = os.environ["FAKE_CASE"]
with open(os.environ["FAKE_CALLS"], "a", encoding="utf-8") as f:
    f.write(tool + " " + " ".join(args) + "\n")
def stop(*lines, code=2):
    print("\n".join(lines))
    sys.exit(code)
if tool == "tcmake":  # the toolchain folder's make: its bin must be first on PATH, as `pros make` puts it
    if os.environ["PATH"].split(os.pathsep)[0] != os.environ["FAKE_TC"]:
        stop("arm-none-eabi-g++: command not found (the toolchain's bin is not first on PATH)")
    tool = "make"
if tool == "make":
    if case == "slow":
        print("Compiled src/team/autons.cpp [OK]", flush=True)
        time.sleep(30)
    if case == "owed":
        for _ in range(2):
            print("src/robots/robot_one_facts.hpp:651:1: error: static assertion failed: robot one: OWED -- the cascade's motor ports (src/robots/robot_one_facts.hpp)")
            print("src/robots/robot_one_facts.hpp:652:1: error: static assertion failed: robot one: OWED -- the AI Vision camera's port (src/robots/robot_one_facts.hpp)")
        sys.exit(2)
    if case == "owed2":  # robot two's ledger (robot_two_facts.hpp:82's words)
        print("src/robots/robot_two_facts.hpp:90:1: error: static assertion failed: robot two: OWED -- the right side toggle's run direction (src/robots/robot_two_facts.hpp)")
        sys.exit(2)
    where = {"region": "C:\\Users\\team\\shulib\\src\\team\\skills_route.cpp:6:5", "edge": "src/team/skills_route.cpp:11:1",
             "plumbing": "src/team/skills_route.cpp:12:1", "elsewhere": "src/team/autons.cpp:40:3", "curly": "src/team/autons.cpp:40:3"}
    if case in where:
        said = "\u2018r\u2019" if case == "curly" else "'r'"
        stop("Compiled src/team/skills_route.cpp \x1b[31;01m[ERRORS]\x1b[0m", "In file included from src/team/skills_route.cpp:1:",
             where[case] + ": error: " + said + " was not declared in this scope", "make: *** [common.mk:279: bin/team/skills_route.cpp.o] Error 1")
    if case in ("header", "headerplain"):
        stop("Compiled src/team/skills_route.cpp [ERRORS]", "In file included from ./include/shulib/chassis/route.hpp:3,",
             "                 from src/team/skills_route.cpp:1:",
             "./include/shulib/chassis/route.hpp: In instantiation of 'void shulib::f() [with T = int]':",
             ("src/team/skills_route.cpp:3:5" if case == "header" else "src/team/autons.cpp:40:3") + ":   required from here",
             "./include/shulib/chassis/route.hpp:120:9: error: static assertion failed: a step needs a unit",
             "make: *** [common.mk:279: bin/team/skills_route.cpp.o] Error 1")
    if case == "link":
        stop("Linking hot project with ./bin/cold.package.elf [ERRORS]", "ld: bin/team/autons.cpp.o: in function `shulib::f()':",
             "autons.cpp:(.text+0x1c): undefined reference to `shulib::g()'", "collect2: error: ld returned 1 exit status",
             "make: *** [common.mk:253: bin/hot.package.elf] Error 1")
    stop("Linking hot project with ./bin/cold.package.elf [OK]", code=0)
if args[:1] == ["lsusb"]:
    if case == "lsusbfail":  # a list cut short: not trusted, pros picks
        stop("There are no connected VEX EDR V5 System Ports", "Error: the USB list failed", code=1)
    if case == "hang":  # still running after the script's 3 s: it was not stopped
        time.sleep(5)
        with open(os.environ["FAKE_CALLS"], "a", encoding="utf-8") as f:
            f.write("pros lsusb ran past its timeout\n")
    if case == "oddformat":
        stop('{"v5/system": [{"device": "COM3"}]}', code=0)
    brain = "VEX Robotics V5 Brain - 1CA1A500 - VEX Robotics "
    joy = "/dev/ttyACM2 - VEX Robotics V5 Controller - 0F1 - VEX Robotics Communications Port"
    system, user = {
        "nobrain": ([], []),
        "controller": ([joy], []),
        "ports": ([joy, "/dev/ttyACM0 - " + brain + "Communications Port"], ["/dev/ttyACM1 - " + brain + "User Port"]),
        "twobrains": (["/dev/ttyACM0 - " + brain + "Communications Port", "/dev/ttyACM3 - VEX Robotics V5 Brain - 2B00 - VEX Robotics Communications Port"],
                      ["/dev/ttyACM1 - " + brain + "User Port", "/dev/ttyACM4 - VEX Robotics V5 Brain - 2B00 - VEX Robotics User Port"]),
        "win": (["COM3 - VEX Robotics Communications Port (COM3)"], ["COM4 - VEX Robotics User Port (COM4)"]),
        "winusb": (["COM3 - USB Serial Device (COM3)"], ["COM4 - USB Serial Device (COM4)"]),
        "winboth": (["COM3 - VEX Robotics Communications Port (COM3)", "COM5 - VEX Robotics Communications Port (COM5)"],
                    ["COM4 - VEX Robotics User Port (COM4)"]),
    }.get(case, (["/dev/ttyACM0 - " + brain + "Communications Port"], ["/dev/ttyACM1 - " + brain + "User Port"]))
    for header, ports in (("VEX EDR V5 System Ports", system), ("VEX EDR V5 User Ports", user), ("VEX EDR Cortex Microcontroller Ports", [])):
        print(header + ":\n" + "\n".join(ports) if ports else "There are no connected " + header)
    sys.exit(1 if case == "lsusbbad" else 0)  # lsusbbad: a whole list, then a failure (RMP1-K: never trusted)
if args[:1] == ["terminal"]:  # RMP1-K: `terminal PORT --no-banner` (cli/terminal.py); its POSIX console reads a TTY
    if os.name != "nt" and not os.isatty(0):
        stop("termios.error: (25, 'Inappropriate ioctl for device')", code=1)
    import signal
    signal.signal(signal.SIGINT, lambda *a: sys.exit(0))  # pros's own handler stops the terminal quietly
    port = args[1]
    print("Established terminal connection", flush=True)
    replay = os.environ.get("FAKE_TERMINAL", "")
    gone = lambda: os.path.exists(os.environ.get("FAKE_UNPLUG", os.devnull + "x"))
    for line in (open(replay, encoding="utf-8", errors="replace").read().split("\n") if os.path.exists(replay) else []):
        if gone():
            break
        print(line, flush=True)
        time.sleep(float(os.environ.get("FAKE_TERMINAL_EVERY", "0")))
    while not gone():  # an open terminal waits for more; Ctrl+C ends it
        time.sleep(0.05)
    stop("WARNING - pros.serial.terminal.terminal:reader - Connection to " + port + " broken - pros-cli version:3.5.6",
         "WARNING - pros.serial.terminal.terminal:stop - Stopping terminal - pros-cli version:3.5.6", code=0)
if args[:2] == ["v5", "read-file"]:  # pros-cli 3.5.6 cannot run it (cli/v5_utils.py: an `outfile` its function never takes)
    stop("Traceback (most recent call last):", "TypeError: read_file() got an unexpected keyword argument 'outfile'", code=1)
if args[:1] == ["user-script"]:  # RMP1-K D5 (cli/user_script.py): pros's own Python runs the script, with pros importable.
    import configparser, importlib.util, json, types  # FAKE_SLOTS, a JSON {slot: [name, description, date] | "unread"};
    slots = json.loads(os.environ.get("FAKE_SLOTS", "{}"))  # FAKE_CASE "slotsfail": the brain does not answer
    class V5Device:  # only what reading needs: anything else (a write, an erase) is an AttributeError, so the script fails
        def __init__(self, port):
            if case == "slotsfail":
                raise OSError("could not open port " + port.port + ": the brain did not answer")
            self.files = ["slot_%s.%s" % (n, k) for n in sorted(slots, key=int) for k in ("bin", "ini")]
        def get_dir_count(self, vid=1, options=0):
            return len(self.files)
        def get_file_metadata_by_idx(self, i, options=0):
            return {"idx": i, "size": 1024, "type": self.files[i][-3:], "filename": self.files[i]}
        def read_ini(self, remote_name):
            print("Downloading " + remote_name, flush=True)  # pros's progress bar label, on stdout when piped
            got = slots[remote_name[len("slot_"):-len(".ini")]]
            if got == "unread":
                return None  # V5Device.read_ini: a VEXCommError reads as None
            ini = configparser.ConfigParser()
            ini.read_string("[project]\nversion = 3.5.6\n[program]\nname = %s\ndescription = %s\ndate = %s\n" % tuple(got))
            return ini
    class DirectPort:
        def __init__(self, port):
            self.port = port
    for name in ("pros", "pros.serial", "pros.serial.devices", "pros.serial.devices.vex", "pros.serial.ports"):
        sys.modules[name] = types.ModuleType(name)
    sys.modules["pros.serial.devices.vex"].V5Device, sys.modules["pros.serial.ports"].DirectPort = V5Device, DirectPort
    path = os.path.abspath(args[1])
    print("Loading " + os.path.dirname(args[1]) + " from " + path, flush=True)
    spec = importlib.util.spec_from_file_location(os.path.dirname(args[1]), path)
    spec.loader.exec_module(importlib.util.module_from_spec(spec))
    sys.exit(0)
if args[:1] != ["upload"]:
    stop("Error: No such command " + " ".join(args[:1]))
pos, it = [], iter(args[1:])
for a in it:
    if a in ("--slot", "--name", "--after", "--target", "--project", "--icon", "--description"):
        next(it, None)
    elif a.startswith("-"):
        stop("Error: No such option: " + a)
    else:
        pos.append(a)
if len(pos) > 2 or (pos and not os.path.exists(pos[0])):
    stop("Error: [PATH] [PORT] not understood: " + " ".join(pos))
port = pos[1] if len(pos) == 2 else "/dev/ttyACM0"
if case == "nodevice":
    stop("No v5 ports were found! If you think you have a v5 plugged in, run this command again with the --debug flag",
         "Error: No port provided or located. Make sure to specify --target if needed.", code=1)
if case == "question":
    stop("Multiple v5 Brains were found. Please choose one to upload the program: [1CA1A500 | 2B00]: ", "Aborted!", code=1)
if case == "quiet":
    sys.exit(0)
if case == "uploadfail":
    stop("Error: [Errno 13] could not open port " + port + ": Permission denied", code=1)
print('Uploading program "' + args[args.index('--name') + 1] + '" (hot.package.bin) to V5 slot ' + args[args.index('--slot') + 1] + ' on ' + port + " (compressed)")
done = 'Finished uploading "' + args[args.index('--name') + 1] + '" (hot.package.bin) to V5'
if case == "late":  # pros has ended with its last line still on the way (its own child prints it): read to the end
    import subprocess
    sys.stdout.flush()
    subprocess.Popen([sys.executable, "-c", "import time; time.sleep(0.5); print(%r)" % done], stdout=sys.stdout)
    sys.exit(0)
print(done)
"""

SKILLS_STANDIN = """#include "team/skills_route.hpp"
namespace shulib::bench {
// ═══ ROUTE MAKER — route "skills" — BEGIN ═══════════════════════════════════
namespace {
chassis::RoutineResult skillsSteps(competition::AutonContext& ctx) {
    r.driveTo(24_in, 0_in);
    e.name = "skills";                                // what the brain's menu shows
    e.name = "left 1";                                // what the brain's menu shows
}
}  // namespace
// ═══ ROUTE MAKER — route "skills" — END ═════════════════════════════════════
std::span<const TeamRoute> teamRoutes() { return kTeamRoutes; }
}  // namespace shulib::bench
"""

POSIX_ONLY = ("os.fork", "os.uname", "os.getuid", "os.setsid", "os.killpg", "import fcntl", "import termios", "import pwd",
              "import grp", "shell=True", "/dev/null", "/bin/sh", "signal.SIGKILL")


def self_test():
    """Every case D5 names and the review's, against fakes; returns (the failures, how many cases)."""
    import signal
    import time
    here = Path(__file__).resolve()
    work = here.parent.parent / "build" / "upload_auton_selftest"
    shutil.rmtree(work, ignore_errors=True)
    (work / "bin").mkdir(parents=True)
    (work / "repo" / "src" / "team").mkdir(parents=True)
    (work / "fake.py").write_text(FAKE, encoding="utf-8")
    def stand_in(folder, tool, says=None):  # `says`: the name the fake records and acts as (default: `tool`)
        folder.mkdir(parents=True, exist_ok=True)
        if os.name == "nt":
            (folder / f"{tool}.cmd").write_text(f'@"{sys.executable}" "{work / "fake.py"}" {says or tool} %*\n', encoding="utf-8")
        else:
            p = folder / tool
            p.write_text(f'#!/bin/sh\nexec "{sys.executable}" "{work / "fake.py"}" {says or tool} "$@"\n', encoding="utf-8")
            p.chmod(0o755)

    for tool in ("make", "pros"):
        stand_in(work / "bin", tool)
    stand_in(work / "prosonly", "pros")  # PATH with pros but no make (a plain terminal with pros-cli installed)
    stand_in(work / "makeonly", "make")
    # the PROS extension's install folder: the toolchain (PROS_TOOLCHAIN) with bin/make, and pros-cli beside it
    stand_in(work / "ext" / "install" / "pros-toolchain-test" / "bin", "make", says="tcmake")
    stand_in(work / "ext" / "install" / "pros-cli-test", "pros")
    (work / "bare" / "pros-toolchain-test" / "bin").mkdir(parents=True)  # a toolchain folder with no make in it
    failures, count = [], [0]

    def env_for(name, path=True, enc="utf-8", tc=None, ext=None):
        """`path`: True the stand-ins' bin, False an empty folder, else that folder; `tc` the PROS_TOOLCHAIN folder
        (None: unset); `ext` (the folder VS Code's globalStorage is in, the extension's toolchain in it) -- None: an empty
        one, never this computer's own."""
        (work / "calls.txt").write_text("", encoding="utf-8")
        home = str(ext[0] if ext else work / "nohome")
        env = dict(os.environ, FAKE_CASE=name, FAKE_CALLS=str(work / "calls.txt"), PYTHONIOENCODING=enc,
                   PATH=str(work / "bin") if path is True else str(work / "empty") if path is False else str(path),
                   APPDATA=home, XDG_CONFIG_HOME=home, HOME=home, USERPROFILE=home, GIT_CEILING_DIRECTORIES=str(work))
        env.pop("PROS_TOOLCHAIN", None)
        env.pop("FAKE_TC", None)
        if tc:
            env.update(PROS_TOOLCHAIN=str(tc), FAKE_TC=str(Path(tc) / "bin"))
        if ext:
            env.update(FAKE_TC=str(Path(ext[1]) / "bin"))
        return env

    def case(name, robot, want_code, want, unwanted=(), calls=None, path=True, skills=SKILLS_STANDIN, enc="utf-8", extra=(),
             tc=None, ext=None):
        count[0] += 1
        (work / "repo" / SKILLS).write_text(skills, encoding="utf-8")
        p = subprocess.run([sys.executable, str(here), "--root", str(work / "repo"), *extra, *robot.split()], env=env_for(name, path, enc, tc, ext),
                           stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=120)
        out = p.stdout.decode("utf-8", errors="replace")
        made = (work / "calls.txt").read_text(encoding="utf-8").splitlines()
        if p.returncode != want_code:
            failures.append(f"{name}: exit {p.returncode}, wanted {want_code}\n{out}")
        failures.extend(f"{name}: missing {w!r}\n{out}" for w in want if w not in out)
        failures.extend(f"{name}: should not say {w!r}\n{out}" for w in (*unwanted, "Traceback") if w in out)
        if calls is not None and made != calls:
            failures.append(f"{name}: ran {made}, wanted {calls}")

    make1, lsusb = f"make -j{JOBS} ROBOT=one PROGRAM=autontest", "pros lsusb"
    upload = f"pros upload --slot 6 --name {NAME} --description no git | skills, left 1"  # no repository above the stand-in
    case("ok", "one", 0, ["Building White Bot's Auton Tester: make ROBOT=one PROGRAM=autontest", "✓ BUILT",
                          "✓ UPLOADED to slot 6. On the brain: shulib Auton Tester → skills or left 1 → RUN"],
         calls=[make1, lsusb, upload + " . /dev/ttyACM0"])
    case("ok", "two", 0, ["Building Black Bot's Auton Tester: make ROBOT=tank", "✓ UPLOADED to slot 6"], calls=[f"make -j{JOBS} ROBOT=tank PROGRAM=autontest", lsusb, upload + " . /dev/ttyACM0"])
    case("owed", "one", 1, ["✗ NOT BUILT", "White Bot's build refuses while a fact it needs is OWED (not measured or typed yet):\n"
                            "    - the cascade's motor ports\n    - the AI Vision camera's port\n  Type them in src/robots/robot_one_facts.hpp"],
         unwanted=["✓ BUILT", "UPLOADED", "robot one's build"], calls=[make1])
    case("owed2", "black", 1, ["Black Bot's build refuses while a fact it needs is OWED (not measured or typed yet):\n"
                               "    - the right side toggle's run direction\n  Type them in src/robots/robot_two_facts.hpp"],
         unwanted=["White Bot", "robot two's build"], calls=[f"make -j{JOBS} ROBOT=tank PROGRAM=autontest"])
    case("ok", "ONE", 0, ["Building White Bot's Auton Tester: make ROBOT=one PROGRAM=autontest"], calls=[make1, lsusb, upload + " . /dev/ttyACM0"])
    case("ok", "Two", 0, ["Building Black Bot's Auton Tester: make ROBOT=tank"])  # the old words, any capitals
    case("region", "one", 1, ["✗ NOT BUILT", "src/team/skills_route.cpp:6: 'r' was not declared in this scope",
                              "re-export from the route builder"], unwanted=["[ERRORS]", "Error 1"])
    case("edge", "one", 1, ["src/team/skills_route.cpp:11: 'r'", "re-export from the route builder"])  # the END line is the region's
    case("plumbing", "one", 1, ["src/team/skills_route.cpp:12: 'r' was not declared", "outside the ROUTE MAKER lines"],
         unwanted=["re-export"])
    case("elsewhere", "two", 1, ["src/team/autons.cpp:40: 'r' was not declared", "Fix that line"], unwanted=["re-export", "ROUTE MAKER"])
    case("header", "one", 1, ["./include/shulib/chassis/route.hpp:120: static assertion failed: a step needs a unit",
                              "re-export from the route builder"], unwanted=["Fix that line"])  # required from the BEGIN line
    case("headerplain", "one", 1, ["route.hpp:120: static assertion failed", "Fix that line"], unwanted=["re-export"])
    case("link", "one", 1, ["✗ NOT BUILT", "autons.cpp:(.text+0x1c): undefined reference to `shulib::g()'", "Ask the coordinator"])
    case("curly", "one", 1, ["X NOT BUILT", "src/team/autons.cpp:40: ?r? was not declared"], enc="ascii")  # cp1252-like console
    case("nobrain", "one", 1, ["✓ BUILT", "✗ NOT UPLOADED: No brain found. Plug the brain in with USB (not the controller)"],
         calls=[make1, lsusb])
    case("controller", "one", 1, ["✗ NOT UPLOADED: Only the controller is plugged in. Plug the BRAIN in with USB"], calls=[make1, lsusb])
    case("ports", "one", 0, ["✓ UPLOADED"], calls=[make1, lsusb, upload + " . /dev/ttyACM0"])
    case("twobrains", "one", 1, ["Two brains are plugged in (/dev/ttyACM0, /dev/ttyACM3)"], calls=[make1, lsusb])
    case("win", "one", 0, ["✓ UPLOADED to slot 6"], calls=[make1, lsusb, upload + " . COM3"])
    case("winusb", "one", 0, ["✓ UPLOADED to slot 6"], calls=[make1, lsusb, upload + " . COM3"])
    case("winboth", "one", 1, ["More than one V5 port is plugged in (COM3, COM5)"], calls=[make1, lsusb])
    case("oddformat", "one", 0, ["✓ UPLOADED"], calls=[make1, lsusb, upload])  # not pros's shape: pros picks the port
    case("lsusbfail", "one", 0, ["✓ UPLOADED"], calls=[make1, lsusb, upload])
    case("hang", "one", 1, ["✗ NOT UPLOADED: pros gave no answer in 3 s"], extra=["--wait", "0.05"],
         calls=[make1, lsusb] if os.name != "nt" else None)  # Windows: the .cmd stand-in's python outlives cmd.exe
    case("quiet", "one", 1, ["✗ NOT UPLOADED: pros did not say it finished uploading."])
    case("late", "one", 0, ["✓ UPLOADED to slot 6"])
    case("uploadfail", "one", 1, ["✗ NOT UPLOADED: This computer may not use the brain's port"])
    case("nodevice", "one", 1, ["✗ NOT UPLOADED: No brain found."])
    case("question", "one", 1, ['✗ NOT UPLOADED: pros stopped to ask a question. Run the upload yourself to answer it: '
                                'pros upload --slot 6 --name "shulib Auton Tester" --description "no git | skills, left 1"'])
    case("ok", "one", 0, ["✓ UPLOADED to slot 6. On the brain: shulib Auton Tester → your route → RUN"],
         skills=SKILLS_STANDIN.replace("// ═══ ROUTE MAKER", "// was ROUTE MAKER"))  # no markers: no names, no crash
    case("ok", "one", 0, ["→ your route → RUN"], skills=SKILLS_STANDIN + SKILLS_STANDIN)  # two pairs: no names
    case("ok", "one", 1, ["✗ make and pros not found. Run this in the PROS terminal (VS Code: PROS, then Integrated Terminal)",
                          PROS_HELP], calls=[], path=False)
    tc = work / "ext" / "install" / "pros-toolchain-test"
    case("ok", "one", 0, ["(make from the PROS toolchain: ", "✓ BUILT", "✓ UPLOADED to slot 6"],
         calls=["tc" + make1, lsusb, upload + " . /dev/ttyACM0"], path=work / "prosonly", tc=tc)  # make from PROS_TOOLCHAIN/bin
    case("ok", "two", 0, ["(make from the PROS toolchain: ", "✓ UPLOADED to slot 6"],
         calls=[f"tcmake -j{JOBS} ROBOT=tank PROGRAM=autontest", lsusb, upload + " . /dev/ttyACM0"], path=False, tc=tc)  # both from there
    case("ok", "one", 0, ["✓ UPLOADED to slot 6"], unwanted=["from the PROS toolchain"],
         calls=[make1, lsusb, upload + " . /dev/ttyACM0"], tc=tc)  # make on PATH wins: the toolchain's is not used
    case("ok", "one", 1, ["✗ make not found. Run this in the PROS terminal"], unwanted=["make and pros"], calls=[],
         path=work / "prosonly", tc=work / "bare" / "pros-toolchain-test")  # a toolchain with no make: never `pros make`
    case("ok", "one", 1, ["✗ pros not found."], calls=[], path=work / "makeonly", tc=work / "bare" / "pros-toolchain-test")

    case("ok", "three", 2, ["Say which robot"], calls=[])
    # RMP1-L: White Bot and Black Bot (the team lead, 2026-11-20) -- `white`/`black`, any capitals; one/two still work
    case("ok", "white drive", 0, ["Building White Bot's Drive: make ROBOT=one PROGRAM=drive", "✓ UPLOADED to slot 1"],
         calls=[f"make -j{JOBS} ROBOT=one PROGRAM=drive", lsusb, "pros upload --slot 1 --name shulib Drive --description no git . /dev/ttyACM0"])
    case("ok", "Black", 0, ["Building Black Bot's Auton Tester: make ROBOT=tank PROGRAM=autontest", "✓ UPLOADED to slot 6"],
         calls=[f"make -j{JOBS} ROBOT=tank PROGRAM=autontest", lsusb, upload + " . /dev/ttyACM0"])
    case("ok", "WHITE tune", 0, ["Building White Bot's Tune: make ROBOT=one PROGRAM=tune"],
         calls=[f"make -j{JOBS} ROBOT=one PROGRAM=tune", lsusb, "pros upload --slot 7 --name shulib Tune --description no git . /dev/ttyACM0"])
    case("ok", "whitebot", 2, ["Say which robot", "python3 tools/upload_auton.py white"], calls=[])
    # the team lead, 2026-11-20 ("how easy is it to export the driver controls"): Drive and Competition, each to its slot
    case("ok", "one drive", 0, ["Building White Bot's Drive: make ROBOT=one PROGRAM=drive", "✓ BUILT",
                                "✓ UPLOADED to slot 1. On the brain: shulib Drive (the driver controls are in it)"],
         unwanted=["Auton Tester", "RUN"],
         calls=[f"make -j{JOBS} ROBOT=one PROGRAM=drive", lsusb, "pros upload --slot 1 --name shulib Drive --description no git . /dev/ttyACM0"])
    case("ok", "two competition", 0, ["Building Black Bot's Competition: make ROBOT=tank PROGRAM=competition",
                                      "✓ UPLOADED to slot 2. On the brain: shulib Competition → pick the auton on its screen"],
         unwanted=["Auton Tester"],
         calls=[f"make -j{JOBS} ROBOT=tank PROGRAM=competition", lsusb,
                "pros upload --slot 2 --name shulib Competition --description no git | skills, left 1 . /dev/ttyACM0"])
    case("ok", "one tester", 0, ["✓ UPLOADED to slot 6. On the brain: shulib Auton Tester"],
         calls=[make1, lsusb, upload + " . /dev/ttyACM0"])  # the default, said
    case("question", "one drive", 1, ['pros upload --slot 1 --name "shulib Drive" --description "no git"'])  # its own slot in the hand command
    case("ok", "one autontest", 2, ["Say which robot, and which program"], calls=[])  # the words are drive/competition/tester
    case("ok", "one drive extra", 2, ["Say which robot"], calls=[])
    case("ok", "one bench", 0, ["Building White Bot's Bench Tests: make ROBOT=one PROGRAM=tester", "✓ UPLOADED to slot 3. On the brain: Bench Tests"],
         calls=[f"make -j{JOBS} ROBOT=one PROGRAM=tester", lsusb, "pros upload --slot 3 --name Bench Tests --description no git . /dev/ttyACM0"])
    case("ok", "two tune", 0, ["✓ UPLOADED to slot 7. On the brain: shulib Tune"],
         calls=[f"make -j{JOBS} ROBOT=tank PROGRAM=tune", lsusb, "pros upload --slot 7 --name shulib Tune --description no git . /dev/ttyACM0"])
    every = [(n, PROGRAMS[n]) for n in ALL]
    case("ok", "one all", 0, ["✓ slot 1 shulib Drive\n✓ slot 2 shulib Competition\n✓ slot 6 shulib Auton Tester\n"
                              "✓ slot 3 Bench Tests\n✓ slot 4 shulib Calibrate\n✓ slot 7 shulib Tune"],
         calls=[c for _n, (prog, slot, name) in every for c in (f"make -j{JOBS} ROBOT=one PROGRAM={prog}", lsusb,
                                                                 f"pros upload --slot {slot} --name {name} --description no git"
                                                                 + (" | skills, left 1" if prog in ("autontest", "competition") else "")
                                                                 + " . /dev/ttyACM0")])
    case("owed", "one all", 1, ["✗ slot 1 shulib Drive", "✗ slot 7 shulib Tune"])  # each fails on its own; the summary says so
    # RMP1-I review: a VS Code task has no PROS_TOOLCHAIN; the extension's own install folder (globalStorage) is found
    gs = work / "gs"
    tcx = gs / ("Library/Application Support" if sys.platform == "darwin" else "") / "Code/User/globalStorage/sigbots.pros/install"
    tcx = tcx / ("pros-toolchain-windows/usr" if os.name == "nt" else "pros-toolchain-linux")
    stand_in(tcx / "bin", "make", says="tcmake")
    stand_in(tcx.parent.parent / "pros-cli-windows" if os.name == "nt" else tcx.parent / "pros-cli-linux", "pros")
    ran = ["pros lsusb", upload + " . /dev/ttyACM0"]
    case("ok", "one", 0, ["(make from the PROS toolchain: ", "✓ BUILT", "✓ UPLOADED"], path=False, ext=(gs, tcx),
         calls=[f"tcmake -j{JOBS} ROBOT=one PROGRAM=autontest"] + ran)
    case("ok", "one", 0, ["✓ UPLOADED"], unwanted=["from the PROS toolchain"], ext=(gs, tcx),
         calls=[f"make -j{JOBS} ROBOT=one PROGRAM=autontest"] + ran)  # both on PATH: the extension's folder is not used
    stand_in(work / "sysmake", "make", says="tcmake")  # Linux: make on PATH, the compiler only in the extension's bin
    case("ok", "one", 0, ["✓ UPLOADED"], unwanted=["from the PROS toolchain"], path=work / "sysmake", ext=(gs, tcx),
         calls=[f"tcmake -j{JOBS} ROBOT=one PROGRAM=autontest"] + ran)
    if os.name != "nt":  # Ctrl+C while make runs: a plain ✗, never a traceback, and what make printed is in the log
        count[0] += 1
        log, until = work / "repo" / "build" / "upload_auton.log", time.time() + 10
        log.unlink(missing_ok=True)  # the last case's log is not this one's
        p = subprocess.Popen([sys.executable, str(here), "--root", str(work / "repo"), "one"], env=env_for("slow"),
                             stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        live = False  # make's line in the log while make still runs
        while not live and time.time() < until:
            time.sleep(0.05)
            live = log.exists() and "Compiled" in log.read_text(encoding="utf-8")
        p.send_signal(signal.SIGINT)
        out = p.communicate(timeout=60)[0].decode("utf-8", errors="replace")
        said = log.read_text(encoding="utf-8")
        if p.returncode != 130 or "✗ STOPPED (Ctrl+C)" not in out or "Traceback" in out or not live or \
                "Compiled src/team/autons.cpp [OK]" not in said or "(stopped: Ctrl+C)" not in said:
            failures.append(f"ctrl-c: exit {p.returncode}, the log {'live' if live else 'not written as make printed'}\n{out}\nthe log:\n{said}")
    # RMP1-K D3: the USER port (where the program's output comes), found as the system port is; D5: the description
    lsusb = ("VEX EDR V5 System Ports:\n/dev/ttyACM0 - VEX Robotics V5 Brain - 1CA1 - VEX Robotics Communications Port\n"
             "/dev/ttyACM2 - VEX Robotics V5 Controller - 0F1 - VEX Robotics Communications Port\n"
             "VEX EDR V5 User Ports:\n/dev/ttyACM1 - VEX Robotics V5 Brain - 1CA1 - VEX Robotics User Port\n"
             "/dev/ttyACM3 - VEX Robotics V5 Controller - 0F1 - VEX Robotics User Port\n")
    for name, text, kind, want in (
            ("the user port, a controller beside it", lsusb, "user", ("/dev/ttyACM1", None)),
            ("the system port unchanged", lsusb, "system", ("/dev/ttyACM0", None)),
            ("a controller only: said", lsusb.replace("/dev/ttyACM1 - VEX Robotics V5 Brain", "x"), "user", (None, "Only the controller")),
            ("Windows: the user COM port", "VEX EDR V5 System Ports:\nCOM3 - VEX Robotics Communications Port (COM3)\n"
             "VEX EDR V5 User Ports:\nCOM4 - VEX Robotics User Port (COM4)\n", "user", ("COM4", None)),
            ("no user port listed: no brain", "VEX EDR V5 System Ports:\n/dev/ttyACM0 - VEX Robotics V5 Brain - 1 - Port\n"
             "There are no connected VEX EDR V5 User Ports\n", "user", (None, "No brain found"))):
        count[0] += 1
        got = brain_port(text, kind)
        if got[0] != want[0] or (want[1] is None) != (got[1] is None) or (want[1] and want[1] not in got[1]):
            failures.append(f"brain_port: {name}: {got}")
    count[0] += 1
    git = work / "git"
    (git / "src").mkdir(parents=True)
    run = lambda *a: subprocess.run(["git", "-C", str(git), "-c", "user.name=t", "-c", "user.email=t@t", *a], capture_output=True, text=True)  # noqa: E731
    if run("init", "-q").returncode == 0 and not run("commit", "-q", "--allow-empty", "-m", "x").returncode:
        head = run("rev-parse", "--short", "HEAD").stdout.strip()
        got = description(git, ["skills", "left 1", "\u00e9t\u00e9", "x" * 80])
        if not got.startswith(f"{head} | skills, left 1, t, xxx") or len(got) != 60 or not got.isascii():
            failures.append(f"description: {got!r} (the commit {head})")
    source = here.read_text(encoding="utf-8").split("# ── self-test")[0]
    failures.extend(f"a POSIX-only call in the script: {w}" for w in POSIX_ONLY if w in source)
    shutil.rmtree(work, ignore_errors=True)
    return failures, count[0]


if __name__ == "__main__":
    sys.stdout.reconfigure(errors="replace")  # what this console cannot print becomes "?", never a crash
    if sys.argv[1:] == ["self-test"]:
        problems, n = self_test()
        print("\n".join(problems) if problems else f"upload_auton self-test: OK ({n} cases against fake make and pros)")
        sys.exit(1 if problems else 0)
    try:
        sys.exit(main(sys.argv[1:]))
    except KeyboardInterrupt:
        print(f"\n{marks()[1]} STOPPED (Ctrl+C). What make and pros printed so far: build/upload_auton.log")
        sys.exit(130)
