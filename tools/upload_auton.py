#!/usr/bin/env python3
"""upload_auton.py -- build the Auton Tester, upload it, and say plainly whether it worked (RMP1-F D5).

    python3 tools/upload_auton.py one    robot one: make ROBOT=one PROGRAM=autontest
    python3 tools/upload_auton.py two    robot two: make ROBOT=tank PROGRAM=autontest
    python3 tools/upload_auton.py self-test

Then `pros upload --slot 6 --name "shulib Auton Tester" . <the brain's port>` (pros-cli takes the port as an argument
after the project folder; the port from `pros lsusb`'s "V5 System Ports", without a controller). Prints ✓ BUILT and
✓ UPLOADED, or ✗ and the first real error in plain words. Windows and Linux: Python 3 (`python` on Windows), `make` and
`pros` from PATH (the PROS terminal), else from the PROS toolchain's folder (PROS_TOOLCHAIN, which the PROS extension
sets in every VS Code terminal: its bin/make, as `pros make` finds it, and the pros-cli folder beside it). Everything
make and pros print goes to build/upload_auton.log as they print it.
"""
import os
import re
import shutil
import subprocess
import sys
import threading
from pathlib import Path

SLOT = 6
NAME = "shulib Auton Tester"
ROBOTS = {"one": "one", "two": "tank"}
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


def brain_port(text):
    """The brain's port from `pros lsusb`, sorted as pros sorts it: its "V5 System Ports" (a controller is listed there
    too, by name). (port, None); (None, why) when that is not exactly one brain; (None, None) when the list is not in
    pros's shape (then pros picks)."""
    section, seen, system = None, False, []
    for raw in text.splitlines():
        m = PORT.match(raw)
        if m:
            if section == "system":
                system.append((m.group(1), m.group(2)))
        elif raw.strip():
            section = "system" if "system port" in raw.lower() else "other"
            seen = seen or section == "system"
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


def upload_error(text):
    """Why pros upload failed, in plain words."""
    low = text.lower()
    if "no v5" in low or "no device" in low or "no ports" in low or "no port provided" in low:
        return NO_BRAIN
    if "permission denied" in low:
        return "This computer may not use the brain's port. On Linux: sudo usermod -aG dialout $USER, then log out and in."
    if "aborted" in low:  # a question this script cannot answer (which brain; too many files on the brain; ...)
        return f'pros stopped to ask a question. Run the upload yourself to answer it: pros upload --slot {SLOT} --name "{NAME}"'
    last = [x.strip() for x in text.splitlines() if x.strip()]
    return (last[-1] if last else "pros printed nothing.") + " (Ask the coordinator with these words.)"


def no_answer(seconds):
    return f"pros gave no answer in {round(seconds)} s: unplug the brain's USB cable, plug it back in, then run this again."


def find_tools():
    """(make, pros, the build's environment): each from PATH, else from the PROS toolchain's folder -- make from
    PROS_TOOLCHAIN/bin exactly as `pros make` finds it (and, as `pros make` does, that bin first on the build's PATH,
    so the toolchain's compiler builds), pros from the pros-cli folder the PROS extension installs beside the
    toolchain. Never `pros make` itself: when it cannot start make it says so and still exits 0, and an upload after
    it would send the OLD program."""
    make, pros, env = shutil.which("make"), shutil.which("pros"), None
    tc = os.environ.get("PROS_TOOLCHAIN")
    if tc:
        bin_ = Path(tc) / "bin"
        if not make:
            make = shutil.which("make", path=str(bin_))
            if make:
                env = dict(os.environ, PATH=str(bin_) + os.pathsep + os.environ.get("PATH", ""))
        if not pros:
            near = [d for up in (Path(tc).parent, Path(tc).parent.parent) for d in sorted(up.glob("pros-cli-*"))]
            pros = shutil.which("pros", path=os.pathsep.join(map(str, near))) if near else None
    return make, pros, env


def main(argv):
    root, wait = Path(__file__).resolve().parent.parent, 1.0
    while len(argv) >= 2 and argv[0] in ("--root", "--wait"):  # the self-test's stand-in repository and short waits
        root, wait = (Path(argv[1]), wait) if argv[0] == "--root" else (root, float(argv[1]))
        argv = argv[2:]
    if len(argv) != 1 or argv[0] not in ROBOTS:
        print("Say which robot: python3 tools/upload_auton.py one   (or two)")
        return 2
    ok, bad, to = marks()
    which = argv[0]
    make, pros, env = find_tools()
    if not make or not pros:
        missing = " and ".join(n for n, p in (("make", make), ("pros", pros)) if not p)
        print(f"{bad} {missing} not found. Run this in {TERMINAL}, or install PROS: {PROS_HELP}")
        return 1
    log = root / "build" / "upload_auton.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    log.write_text("", encoding="utf-8")
    cmd = [make, f"ROBOT={ROBOTS[which]}", "PROGRAM=autontest"]
    print(f"Building robot {which}'s Auton Tester: make ROBOT={ROBOTS[which]} PROGRAM=autontest (a few minutes the "
          "first time; Ctrl+C stops it)...", flush=True)
    if env:
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
    cmd = [pros, "upload", "--slot", str(SLOT), "--name", NAME] + ([".", port] if port else [])  # [PATH] [PORT]
    print(f"Uploading to slot {SLOT}{' on ' + port if port else ''}...", flush=True)
    code, text = run(cmd, root, log, timeout=300 * wait)
    if code != 0 or "finished uploading" not in text.lower():
        said = ("pros did not say it finished uploading." if code == 0 else
                no_answer(300 * wait) if code is None else upload_error(text))
        print(f"{bad} NOT UPLOADED: {said}")
        print(f"  (everything pros printed: {log})")
        return 1
    r = region(root)
    names = r[2] if r and r[2] else ["your route"]
    print(f"{ok} UPLOADED to slot {SLOT}. On the brain: {NAME} {to} {' or '.join(names)} {to} RUN")
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
print('Uploading program "shulib Auton Tester" (hot.package.bin) to V5 slot ' + args[args.index('--slot') + 1] + ' on ' + port + " (compressed)")
done = 'Finished uploading "shulib Auton Tester" (hot.package.bin) to V5'
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

    def env_for(name, path=True, enc="utf-8", tc=None):
        """`path`: True the stand-ins' bin, False an empty folder, else that folder; `tc` the PROS_TOOLCHAIN folder
        (None: unset, whatever this computer's is)."""
        (work / "calls.txt").write_text("", encoding="utf-8")
        env = dict(os.environ, FAKE_CASE=name, FAKE_CALLS=str(work / "calls.txt"), PYTHONIOENCODING=enc,
                   PATH=str(work / "bin") if path is True else str(work / "empty") if path is False else str(path))
        env.pop("PROS_TOOLCHAIN", None)
        env.pop("FAKE_TC", None)
        if tc:
            env.update(PROS_TOOLCHAIN=str(tc), FAKE_TC=str(Path(tc) / "bin"))
        return env

    def case(name, robot, want_code, want, unwanted=(), calls=None, path=True, skills=SKILLS_STANDIN, enc="utf-8", extra=(),
             tc=None):
        count[0] += 1
        (work / "repo" / SKILLS).write_text(skills, encoding="utf-8")
        p = subprocess.run([sys.executable, str(here), "--root", str(work / "repo"), *extra, robot], env=env_for(name, path, enc, tc),
                           stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=120)
        out = p.stdout.decode("utf-8", errors="replace")
        made = (work / "calls.txt").read_text(encoding="utf-8").splitlines()
        if p.returncode != want_code:
            failures.append(f"{name}: exit {p.returncode}, wanted {want_code}\n{out}")
        failures.extend(f"{name}: missing {w!r}\n{out}" for w in want if w not in out)
        failures.extend(f"{name}: should not say {w!r}\n{out}" for w in (*unwanted, "Traceback") if w in out)
        if calls is not None and made != calls:
            failures.append(f"{name}: ran {made}, wanted {calls}")

    make1, lsusb = "make ROBOT=one PROGRAM=autontest", "pros lsusb"
    upload = f"pros upload --slot 6 --name {NAME}"
    case("ok", "one", 0, ["✓ BUILT", "✓ UPLOADED to slot 6. On the brain: shulib Auton Tester → skills or left 1 → RUN"],
         calls=[make1, lsusb, upload + " . /dev/ttyACM0"])
    case("ok", "two", 0, ["✓ UPLOADED to slot 6"], calls=["make ROBOT=tank PROGRAM=autontest", lsusb, upload + " . /dev/ttyACM0"])
    case("owed", "one", 1, ["✗ NOT BUILT", "robot one's build refuses while a fact it needs is OWED (not measured or typed yet):\n"
                            "    - the cascade's motor ports\n    - the AI Vision camera's port\n  Type them in src/robots/robot_one_facts.hpp"],
         unwanted=["✓ BUILT", "UPLOADED"], calls=[make1])
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
                                'pros upload --slot 6 --name "shulib Auton Tester"'])
    case("ok", "one", 0, ["✓ UPLOADED to slot 6. On the brain: shulib Auton Tester → your route → RUN"],
         skills=SKILLS_STANDIN.replace("// ═══ ROUTE MAKER", "// was ROUTE MAKER"))  # no markers: no names, no crash
    case("ok", "one", 0, ["→ your route → RUN"], skills=SKILLS_STANDIN + SKILLS_STANDIN)  # two pairs: no names
    case("ok", "one", 1, ["✗ make and pros not found. Run this in the PROS terminal (VS Code: PROS, then Integrated Terminal)",
                          PROS_HELP], calls=[], path=False)
    tc = work / "ext" / "install" / "pros-toolchain-test"
    case("ok", "one", 0, ["(make from the PROS toolchain: ", "✓ BUILT", "✓ UPLOADED to slot 6"],
         calls=["tc" + make1, lsusb, upload + " . /dev/ttyACM0"], path=work / "prosonly", tc=tc)  # make from PROS_TOOLCHAIN/bin
    case("ok", "two", 0, ["(make from the PROS toolchain: ", "✓ UPLOADED to slot 6"],
         calls=["tcmake ROBOT=tank PROGRAM=autontest", lsusb, upload + " . /dev/ttyACM0"], path=False, tc=tc)  # both from there
    case("ok", "one", 0, ["✓ UPLOADED to slot 6"], unwanted=["from the PROS toolchain"],
         calls=[make1, lsusb, upload + " . /dev/ttyACM0"], tc=tc)  # make on PATH wins: the toolchain's is not used
    case("ok", "one", 1, ["✗ make not found. Run this in the PROS terminal"], unwanted=["make and pros"], calls=[],
         path=work / "prosonly", tc=work / "bare" / "pros-toolchain-test")  # a toolchain with no make: never `pros make`
    case("ok", "one", 1, ["✗ pros not found."], calls=[], path=work / "makeonly", tc=work / "bare" / "pros-toolchain-test")

    case("ok", "three", 2, ["Say which robot"], calls=[])
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
