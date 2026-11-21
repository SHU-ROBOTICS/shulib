#!/usr/bin/env python3
"""route_builder.py -- the route builder on this laptop, with Send to robot (RMP1-I).

    python3 tools/route_builder.py             open it in the browser (python on Windows; Ctrl+C stops it)
    python3 tools/route_builder.py robot white   what is in each of White Bot's slots, against the wanted table (or black;
                                                 one and two work too: robot one and robot two in the code and the docs)
    python3 tools/route_builder.py self-test

Serves THIS checkout's route builder (tools/route_maker/src, put together by tools/route_maker/build.py's own code) on
127.0.0.1, at the same port every start (the browser keeps the page's draft per port). The page reads and writes
src/team/ through it (any browser, no folder dialog), and Send to robot runs tools/upload_auton.py, its lines streamed
into the page as they come. The robots' facts and PREVIEW's engine: this laptop's build/route_maker_site, else the
published site's, kept in build/route_builder_cache/ for a venue with no internet. 127.0.0.1 only; the token in every
path (and in a header for a write or a send); the Host header checked; nothing served but the page's files; writes only
to files under src/team/, all or nothing, and none while a send runs. Python's standard library only.
RMP1-K: Watch the robot reads the brain's USER port (`pros terminal`, the port from `pros lsusb` as upload_auton finds
the system port) and streams its lines to the page, saved in build/robot_runs/; one reader at a time, closed for a
send and reopened after, the helper's child. What is on White Bot reads the slots over the SYSTEM port (tools/
brain_slots.py, run by `pros user-script`: pros-cli's own V5Device) -- read-only, and through a running helper when there
is one (one pros on the brain at a time).
"""
import datetime
import hmac
import html
import http.client
import importlib.util
import json
import os
import queue
import re
import secrets
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import urllib.parse
import urllib.request
import webbrowser
import zlib
from http.server import BaseHTTPRequestHandler, SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location("route_maker_build", TOOLS / "route_maker" / "build.py")
site = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(site)
sys.path.insert(0, str(TOOLS))
import upload_auton as ua  # noqa: E402 -- the ONE build + upload (its tools, its slot)
PUBLISHED = "https://docs.shurobotics.com/route-builder/"
ENGINE = ("engine/worker.mjs", "engine/preview.wasm")
MIME = {".mjs": "text/javascript; charset=utf-8", ".wasm": "application/wasm"}
MAX_BODY = 4 << 20
MAX_DOWNLOAD = 64 << 20  # one published file, at most
NO_FACTS = ("The route builder has no robots' facts on this laptop yet: connect to the internet and start it again (it "
            "keeps them for the venue), or build it (tools/route_maker/README.md).")
GROUP = {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP} if os.name == "nt" else {"start_new_session": True}
REFUSED = "refused: open the route builder with the link it printed in its terminal"
# RMP1-K D5: what each slot should hold (the team lead, 2026-11-20). None: never judged -- slot 5 is whatever the PROS
# extension's own Upload button built last (project.pros), slot 8 the team's old code ("we arent going to delete dan's code").
WANTED = {1: "shulib Drive", 2: "shulib Competition", 3: "Bench Tests", 4: "shulib Calibrate", 5: None,
          6: "shulib Auton Tester", 7: "shulib Tune", 8: None}
LEFT_ALONE = {5: ("the PROS button's build (optional)", "optional, never judged"),
              8: ("the team's old code", "never touched by shulib")}
UNPLUGGED = "The brain's USB was unplugged (or the brain turned off): plug the BRAIN in, then Watch the robot again."
FACTS_FROM = ("src/robots", "include")  # what the robots' descriptions are built from (test/route_maker_main.cpp)


def preferred_port(root):
    """This checkout's own port, the same at every start: the page keeps its draft and theme per port."""
    return 47000 + zlib.crc32(str(root).encode("utf-8")) % 1000


def say(text):
    """A line in the helper's terminal; a terminal that is gone never stops a send."""
    try:
        print(text, flush=True)
    except (OSError, ValueError):
        pass


def download(url, cache):
    """The published robots.json and engine into `cache`, all or nothing: a cut, broken or refused download, or a file
    the swap cannot move, keeps the old. Never raises (a venue's WiFi is not the start's end)."""
    part, old = cache.with_name(cache.name + ".part"), cache.with_name(cache.name + ".old")
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}) if url.startswith("http://127.") else urllib.request.ProxyHandler())
    try:
        if old.is_dir() and not cache.exists():  # a swap cut short last time
            old.rename(cache)
        shutil.rmtree(part, ignore_errors=True)
        for rel in ("robots.json",) + ENGINE:
            with opener.open(urllib.parse.urljoin(url, rel), timeout=5) as r:
                data, want = r.read(MAX_DOWNLOAD + 1), r.headers.get("Content-Length")
            if len(data) > MAX_DOWNLOAD or (want is not None and len(data) != int(want)) or (rel.endswith(".wasm") and data[:4] != b"\0asm"):
                raise ValueError(f"{rel}: cut short, too big or not the engine")
            (part / rel).parent.mkdir(parents=True, exist_ok=True)
            (part / rel).write_bytes(data)
        got = json.loads((part / "robots.json").read_text(encoding="utf-8"))
        got["robots"]["one"]["api"], got["robots"]["two"]["api"], got["build"]["commit"], got["build"]["date"]  # what the page bakes
        shutil.rmtree(old, ignore_errors=True)
        if cache.exists():
            cache.rename(old)
        try:
            part.rename(cache)
        except OSError:
            if old.exists():
                old.rename(cache)
            raise
        shutil.rmtree(old, ignore_errors=True)
        return True
    except Exception:  # noqa: BLE001 -- OSError, http.client's IncompleteRead, a bad JSON: every one keeps the old
        shutil.rmtree(part, ignore_errors=True)
        return False


def facts_differ(root, commit):
    """None when this checkout's robots (FACTS_FROM, as they are on disk) are the ones `commit`'s facts were built from;
    else the words (git says they differ, or cannot tell)."""
    try:
        same = subprocess.run(["git", "-C", str(root), "diff", "--quiet", str(commit), "--", *FACTS_FROM], stdin=subprocess.DEVNULL,
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30).returncode == 0
    except (OSError, subprocess.SubprocessError):
        same = False
    return None if same else (f"Careful: the robots' facts are the published site's ({commit}), and this checkout's robots differ "
                              "from them (or git cannot tell): what Export writes may not fit this robot. Tell the coordinator.")


def facts(root, url):
    """(robots.json's data, the engine's folder or None, where they came from), or (None, None, None)."""
    for folder, said in ((root / "build" / "route_maker_site", "this laptop's build"), (root / "build" / "route_builder_cache", None)):
        if said is None:
            said = "the published site" if download(url, folder) else "the published site, kept from the last time online"
        try:
            data = json.loads((folder / "robots.json").read_text(encoding="utf-8"))
            data["robots"]["one"], data["robots"]["two"]
        except (OSError, ValueError, KeyError, TypeError):
            continue
        return data, folder if all((folder / f).is_file() for f in ENGINE) else None, said
    return None, None, None


def write_all_or_nothing(path, data):
    """`data` into `path`: a temp file beside it, then one replace (its old mode kept)."""
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.", suffix=".part")
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        os.chmod(tmp, path.stat().st_mode & 0o777 if path.exists() else 0o644)
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise


def stop(p, grace=10):
    """End a send: upload_auton, make and the compilers (its whole process group), as a terminal's Ctrl+C would. `p` may
    have ended already (poll() says None while another thread waits on it): never an error."""
    if p is None or p.poll() is not None:
        return
    if os.name == "nt":
        subprocess.run(["taskkill", "/T", "/F", "/PID", str(p.pid)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    else:
        for sig, wait in ((signal.SIGINT, grace), (signal.SIGKILL, 0)):
            try:
                os.killpg(p.pid, sig)
                p.wait(wait) if wait else None
            except (ProcessLookupError, PermissionError, subprocess.TimeoutExpired):
                pass
    try:
        p.wait(10)
    except subprocess.TimeoutExpired:
        pass


# ── RMP1-K: the brain's ports (D3), its run's lines (D2) and its slots (D5) ──────────────────────────────────────────

def pros_run(args, root, timeout=60, env=None):
    """(exit code or None when it gave no answer in `timeout` s, its output) of `pros <args>`; (None, words) without pros."""
    pros = ua.find_tools()[1]
    if not pros:
        return None, f"pros not found (install PROS: {ua.PROS_HELP})"
    try:
        p = subprocess.run([pros, *args], cwd=root, stdin=subprocess.DEVNULL, capture_output=True, timeout=timeout,
                           env=dict(os.environ, PYTHONIOENCODING="utf-8", **(env or {})))
    except subprocess.TimeoutExpired:
        return None, ua.no_answer(timeout)
    except OSError as e:
        return None, f"pros could not start: {e}"
    return p.returncode, ua.ANSI.sub("", (p.stdout + p.stderr).decode("utf-8", errors="replace"))


def brain_port(root, kind):
    """(the brain's `kind` port -- "user" or "system" -- or None, the words why not), from `pros lsusb` as upload_auton
    reads it: never a guess. A list not in pros's shape: (None, words) too (a read needs the port named)."""
    code, text = pros_run(["lsusb"], root)
    if code is None:
        return None, text
    port, why = ua.brain_port(text, kind) if code == 0 else (None, None)
    alone = ua.brain_port(text, "system")[1] if code == 0 and not port else None
    if alone and alone.startswith("Only the controller"):  # a controller-only USB lists no user port of its own
        why = alone
    return (port, None) if port else (None, why or f"pros lsusb did not list the brain's {kind} port: {ua.NO_BRAIN}")


def open_reader(root, port):
    """`pros terminal <port> --no-banner`, its output to a pipe; on POSIX its stdin a pseudo-terminal (pros's console
    needs a tty: termios), on Windows a console with no window. The helper's child, in its own group (stop() ends it)."""
    pros = ua.find_tools()[1]
    kw = {"cwd": root, "stdout": subprocess.PIPE, "stderr": subprocess.STDOUT,
          "env": dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUNBUFFERED="1")}
    if os.name == "nt":
        p = subprocess.Popen([pros, "terminal", port, "--no-banner"], stdin=subprocess.DEVNULL,
                             creationflags=subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW, **kw)
        p.tty = None
        return p
    import pty
    tty, child = pty.openpty()
    try:
        p = subprocess.Popen([pros, "terminal", port, "--no-banner"], stdin=child, start_new_session=True, **kw)
    except OSError:
        os.close(tty)
        raise
    finally:
        os.close(child)
    p.tty = tty
    return p


def close_reader(p):
    """End a reader (Ctrl+C as a terminal would, then kill) and free its pseudo-terminal; None: nothing open."""
    if p is None:
        return
    stop(p, grace=3)
    if getattr(p, "tty", None) is not None:
        os.close(p.tty)
        p.tty = None


def read_slots(root):
    """({slot: {"name", "description", "date"} or {"name": None, "unread": why}}, None) of the brain's programs over its
    SYSTEM port, or (None, words). ONE `pros user-script tools/brain_slots.py` (pros-cli's own V5Device: the files, then
    each slot_N.ini's [program] name, description and date -- v5_device.py generate_ini_file); its one SHULIB_SLOTS line
    read, whatever pros prints around it. Reads only."""
    port, why = brain_port(root, "system")
    if not port:
        return None, why
    code, text = pros_run(["user-script", str(TOOLS / "brain_slots.py")], root, timeout=120, env={"SHULIB_BRAIN_PORT": port})
    got = None
    for raw in text.splitlines():
        m = re.match(r"\s*SHULIB_SLOTS (\{.*\})\s*$", raw)
        if m:
            try:
                got = json.loads(m.group(1))
            except ValueError:
                got = None
    if code is None:
        return None, text
    if not isinstance(got, dict) or not isinstance(got.get("slots"), dict):
        last = got.get("error") if isinstance(got, dict) and got.get("error") else (text.strip().splitlines() or [f"exit {code}"])[-1]
        return None, f"pros could not read the brain's programs: {last}"
    out = {}
    for n, s in got["slots"].items():
        out[int(n)] = ({"name": None, "unread": s.get("unread") or "not read", "description": "", "date": ""} if "unread" in s
                       else {"name": s.get("name", ""), "description": s.get("description", ""), "date": s.get("date", "")})
    return out, None


def slot_table(have):
    """The wanted table beside what is there: [{slot, want, have, ok, fix, date, description, unread}] (D5). A slot whose
    file could not be read is said so -- never "empty", never "holds the wrong program"."""
    rows = []
    for n, want in WANTED.items():
        got = have.get(n)
        name = got["name"] if got else None
        unread = bool(got and got.get("unread"))
        row = {"slot": n, "want": want or LEFT_ALONE[n][0], "have": name, "unread": unread,
               "date": got and got["date"], "description": got and got["description"]}
        if want is None:
            rows.append(dict(row, ok=True, fix=LEFT_ALONE[n][1]))
            continue
        ok = name == want
        where = [k for k, g in have.items() if g["name"] == want and k != n]
        fix = ("" if ok else f"slot {n} could not be read ({got['unread']}): check again" if unread
               else f"slot {n} is empty: upload {want} there" if not name else f"slot {n} holds {name}: upload {want} there")
        if not ok and not unread and where:
            fix += f" (it is in slot {where[0]} now)"
        rows.append(dict(row, ok=ok, fix=fix))
    return rows


def robot_words(robot, rows, why):
    """The table in a terminal's words (`python3 tools/route_builder.py robot white`); `robot` 'one' or 'two'."""
    if rows is None:
        return f"{ua.NAMES[robot]}: {why}"
    ok, bad, _ = ua.marks()
    out = [f"{ua.NAMES[robot]}: what is in each slot (read from the brain; nothing changed)"]
    for r in rows:
        when = f"  uploaded {r['date'][:16].replace('T', ' ')}" if r["date"] else ""
        build = f"  [{r['description']}]" if r["description"] else ""
        have = "could not be read" if r.get("unread") else r["have"] or "empty"
        out.append(f"  {ok if r['ok'] else bad} slot {r['slot']}: {have}" + ("" if r["ok"] else f" -- {r['fix']}") + when + build)
    return "\n".join(out)


def robot_via_helper(state, root, robot):
    """(rows, why) from the route builder running on this checkout (its lock: never a read while it sends), or None when
    none runs -- so a terminal's check never puts a second pros on the brain beside the page's."""
    if not running(state, root):
        return None
    try:
        s = json.loads(state.read_text(encoding="utf-8"))
        c = http.client.HTTPConnection("127.0.0.1", int(s["port"]), timeout=180)
        c.request("POST", f"/{s['token']}/api/robot?robot={robot}", body=b"",
                  headers={"Host": f"127.0.0.1:{s['port']}", "X-Shulib-Token": s["token"], "Content-Length": "0"})
        r = c.getresponse()
        body = r.read().decode("utf-8", errors="replace")
        c.close()
    except (OSError, ValueError, KeyError, TypeError) as e:
        return None, f"the route builder running on this checkout did not answer ({e}): stop it, or try again"
    if r.status != 200:
        return None, f"the route builder running on this checkout says: {body}"
    got = json.loads(body)
    return got["rows"], got["why"]


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = os.name != "nt"  # Windows' SO_REUSEADDR would share a port another program listens on

    def __init__(self, root, url):
        root = Path(root).resolve()
        for port in (preferred_port(root), 0):  # this checkout's own port, else any free one (this start's draft is apart)
            try:
                super().__init__(("127.0.0.1", port), Handler)
                break
            except OSError:
                if not port:
                    raise
        self.root, self.url, self.token = root, url, secrets.token_urlsafe(24)
        self.port = self.server_address[1]
        self.hosts = {f"127.0.0.1:{self.port}", f"localhost:{self.port}"}
        self.lock, self.child = threading.Lock(), None
        self.reader, self.watching, self.checking = None, False, False  # RMP1-K: the user port's reader; one watch, one check
        site.REPO = self.root  # the team's files (driver files, the shared route) from this checkout
        self.modules = set(site.module_graph(site.ENTRY))
        self.load()

    def load(self):
        """The robots' facts and the engine (facts()), and whether this checkout's robots differ from the published ones."""
        self.data, self.engine, self.said = facts(self.root, self.url)
        self.differs = None if self.said in (None, "this laptop's build") else facts_differ(self.root, self.data["build"]["commit"])

    def busy(self):
        """A send is running (its child has not ended)."""
        return self.child is not None and self.child.poll() is None

    def handle_error(self, request, client_address):
        if not isinstance(sys.exc_info()[1], ConnectionError):  # a closed tab is not an error
            super().handle_error(request, client_address)

    def page(self):
        if self.data is None:  # retried at each load: a laptop online since gets the facts
            self.load()
        if self.data is None:
            return f"<!doctype html><meta charset=utf-8><title>Route builder</title><p>{html.escape(NO_FACTS)}</p>"
        baked = site.baked_data(self.data["robots"])
        baked["build"] = dict(self.data.get("build", baked["build"]), **({"differs": self.differs} if self.differs else {}))
        self.modules = set(site.module_graph(site.ENTRY))
        text = site.page(site.CSP_SITE, f'<script type="module" src="src/{site.ENTRY}"></script>', baked,
                         '<meta name="shulib-engine" content="engine/worker.mjs">' if self.engine else "")
        meta = f'<meta name="shulib-local" content="{self.token}" data-folder="{html.escape(self.root.name)}">'
        return text.replace('<meta name="shulib-app" content="route-builder">', f'<meta name="shulib-app" content="route-builder">\n{meta}', 1)


class Handler(BaseHTTPRequestHandler):
    server_version = "shulib-route-builder"

    def log_message(self, *args):
        pass

    def reply(self, code, body, ctype="text/plain; charset=utf-8", *extra):
        data = body.encode("utf-8") if isinstance(body, str) else body
        self.send_response(code)
        for k, v in (("Content-Type", ctype), ("Content-Length", str(len(data))), ("Cache-Control", "no-store"),
                     ("X-Content-Type-Options", "nosniff"), ("X-Frame-Options", "DENY"),
                     ("Content-Security-Policy", "frame-ancestors 'none'"), ("Referrer-Policy", "no-referrer"), *extra):
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        self.handle_one("GET")

    def do_POST(self):
        self.handle_one("POST")

    def handle_one(self, method):
        """/<token>/<path>: the token in every path (never a cookie, which every 127.0.0.1 port would get), and for a
        write or a send in the X-Shulib-Token header too (only the page's own script sets it)."""
        srv, url = self.server, urllib.parse.urlsplit(self.path)
        q = urllib.parse.parse_qs(url.query)
        if self.headers.get("Host") not in srv.hosts:
            return self.reply(403, "refused: this page is only for http://127.0.0.1")
        _, given, *rest = url.path.split("/", 2)
        token, origin = srv.token.encode(), self.headers.get("Origin")
        if (not hmac.compare_digest(given.encode(), token) or (origin and origin not in {f"http://{x}" for x in srv.hosts})
                or (method == "POST" and not hmac.compare_digest(self.headers.get("X-Shulib-Token", "").encode(), token))):
            return self.reply(403, REFUSED)
        if not rest:  # /<token>: the page's own files are relative to /<token>/
            return self.reply(303, "", "text/plain", ("Location", f"/{srv.token}/"))
        path = "/" + rest[0]
        if method == "GET" and path in ("/", "/app.html"):
            return self.reply(200, srv.page(), "text/html; charset=utf-8")
        if method == "GET" and path.startswith("/src/") and path[5:] in srv.modules:
            return self.reply(200, (site.SRC / path[5:]).read_bytes(), MIME[".mjs"])
        if method == "GET" and path[1:] in ENGINE and srv.engine:
            return self.reply(200, (srv.engine / path[1:]).read_bytes(), MIME[Path(path).suffix])
        if method == "GET" and path == "/api/ping":  # a second start finds this one
            return self.reply(200, json.dumps({"root": str(srv.root)}), "application/json")
        if method == "GET" and path == "/api/log":
            log = srv.root / "build" / "upload_auton.log"
            return self.reply(200, log.read_bytes()) if log.is_file() else self.reply(204, "")
        if path == "/api/file":
            return self.team_file(method, (q.get("path") or [""])[0])
        if method == "POST" and path == "/api/send":
            return self.send((q.get("robot") or [""])[0])
        if method == "POST" and path == "/api/watch":
            return self.watch()
        if method == "POST" and path == "/api/robot":
            return self.robot((q.get("robot") or [""])[0])
        return self.reply(404, "not here")

    def team_file(self, method, rel):
        team = (self.server.root / "src" / "team").resolve()
        try:
            p = (self.server.root / rel).resolve()  # `..` and symlinks resolved first
        except (OSError, ValueError, RuntimeError):
            p = team
        if not rel or team not in p.parents or p.is_dir():
            return self.reply(403, f"refused: the route builder reads and writes only files under src/team/, not {rel!r}")
        if method == "GET":
            return self.reply(200, p.read_bytes()) if p.is_file() else self.reply(204, "")  # none (no console error)
        try:
            n = int(self.headers.get("Content-Length", ""))
        except ValueError:
            return self.reply(411, "not written: the request said no length")
        if not 0 <= n <= MAX_BODY:
            return self.reply(413, "not written: too big for a team file")
        data = self.rfile.read(n)
        try:
            data.decode("utf-8")
            if len(data) != n:
                raise ValueError
        except ValueError:
            return self.reply(400, "not written: the text did not all arrive")
        with self.server.lock:  # a send starts only under this lock: none can start mid-write
            if self.server.busy():  # what a send builds stays what it was when it started
                return self.reply(409, "not written: a send to the robot is running. Wait for it to finish, then write again.")
            try:
                write_all_or_nothing(p, data)
            except OSError as e:
                return self.reply(500, f"not written: {e.strerror or e}")
        return self.reply(200, "written")

    def send(self, robot):
        srv = self.server
        if robot not in ("one", "two"):
            return self.reply(400, "say which robot: one or two")
        with srv.lock:
            if srv.busy():
                return self.reply(409, "A send is already running: wait for it to finish.")
            if srv.checking:
                return self.reply(409, "Reading the robot's slots: wait a moment, then send.")
            close_reader(srv.reader)  # D3: the brain's port is the upload's now; the watch reopens it after
            srv.reader = None
            try:
                srv.child = p = subprocess.Popen(
                    [sys.executable, str(TOOLS / "upload_auton.py"), "--root", str(srv.root), robot], cwd=srv.root,
                    env=dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUNBUFFERED="1"), stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, **GROUP)
            except OSError as e:
                return self.reply(500, f"✗ could not start tools/upload_auton.py: {e}")
        tab = True  # the tab may close: the send carries on to its end, its lines still read, still this helper's

        def out(message):
            nonlocal tab
            try:
                if tab:
                    self.wfile.write((json.dumps(message) + "\n").encode("utf-8"))
                    self.wfile.flush()
            except OSError:
                tab = False

        try:
            say(f"Sending to {ua.NAMES[robot]}...")
            try:
                self.send_response(200)
                for k, v in (("Content-Type", "application/x-ndjson; charset=utf-8"), ("Cache-Control", "no-store"),
                             ("X-Content-Type-Options", "nosniff")):
                    self.send_header(k, v)
                self.end_headers()
            except OSError:
                tab = False
            for raw in p.stdout:
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                say("  " + line)
                out({"line": line})
            out({"done": True, "code": p.wait()})
        finally:
            if p.poll() is None:  # this handler ends only with the send: never a send no one owns
                stop(p)
            with srv.lock:
                if srv.child is p:
                    srv.child = None
        self.close_connection = True

    def stream_start(self):
        """The 200 and the headers of a stream of JSON lines; False when the tab has gone already."""
        try:
            self.send_response(200)
            for k, v in (("Content-Type", "application/x-ndjson; charset=utf-8"), ("Cache-Control", "no-store"),
                         ("X-Content-Type-Options", "nosniff")):
                self.send_header(k, v)
            self.end_headers()
            return True
        except OSError:
            return False

    def emit(self, message):
        """One JSON line to the tab; False when the tab has gone (the watch then ends, and its reader with it)."""
        try:
            self.wfile.write((json.dumps(message) + "\n").encode("utf-8"))
            self.wfile.flush()
            return True
        except OSError:
            return False

    def watch(self):
        """RMP1-K D2/D3: the brain's USER port, line by line, to the tab -- and into build/robot_runs/<time>.log. One watch
        at a time; while a send runs the reader is closed (the send closed it) and the watch waits, then reopens it. A ping
        each second finds a closed tab. Ends when the tab goes, the port goes, or no brain is there (said)."""
        srv = self.server
        with srv.lock:
            if srv.watching:
                return self.reply(409, "The robot's log is already open in another tab: close that one first.")
            srv.watching = True
        runs = srv.root / "build" / "robot_runs"
        saved = runs / (datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S") + ".log")
        try:
            if not self.stream_start():
                return
            runs.mkdir(parents=True, exist_ok=True)
            with open(saved, "a", encoding="utf-8") as log:
                self.emit({"saved": saved.relative_to(srv.root).as_posix()})
                self.watch_loop(srv, log)
        finally:
            with srv.lock:
                close_reader(srv.reader)
                srv.reader, srv.watching = None, False
            self.close_connection = True

    def watch_loop(self, srv, log):
        while True:
            if srv.busy():  # a send has the brain: wait for it (pinging), then reopen
                if not self.emit({"paused": "Sending to the robot: the log reopens when it is done."}):
                    return
                while srv.busy():
                    time.sleep(0.2)
                    if not self.emit({}):
                        return
            port, why = brain_port(srv.root, "user")
            if not port:
                return self.emit({"error": why})
            with srv.lock:
                if srv.busy():
                    continue
                try:
                    srv.reader = p = open_reader(srv.root, port)
                except OSError as e:
                    return self.emit({"error": f"pros terminal could not start: {e}"})
            if not self.emit({"open": port}):
                return
            lines, broken = queue.Queue(), [False]

            def pump(out=p.stdout):
                for raw in out:
                    lines.put(raw.decode("utf-8", errors="replace").rstrip("\r\n"))
                lines.put(None)
            threading.Thread(target=pump, daemon=True).start()
            while True:
                try:
                    line = lines.get(timeout=1)
                except queue.Empty:
                    if not self.emit({}):
                        return
                    continue
                if line is None:
                    break
                log.write(line + "\n")
                log.flush()
                broken[0] = broken[0] or (" broken" in line and "Connection to" in line)
                if not self.emit({"line": line}):
                    return
            with srv.lock:
                if srv.reader is p:
                    close_reader(p)
                    srv.reader = None
            if srv.busy():
                continue  # the send closed it: reopen after
            return self.emit({"error": UNPLUGGED if broken[0] else "The robot's log stopped (pros terminal ended): Watch the robot again."})

    def robot(self, robot):
        """RMP1-K D5: the wanted table beside what is in each slot (JSON). Never while a send uses the brain."""
        srv = self.server
        if robot not in ("one", "two"):
            return self.reply(400, "say which robot: one or two")
        with srv.lock:
            if srv.busy() or srv.checking:
                return self.reply(409, "The brain is busy (a send, or another check): wait for it to finish.")
            srv.checking = True
        try:
            have, why = read_slots(srv.root)
        finally:
            with srv.lock:
                srv.checking = False
        body = {"robot": robot, "rows": slot_table(have) if have is not None else None, "why": why}
        return self.reply(200, json.dumps(body), "application/json")


def running(state, root):
    """The URL of a route builder already running on this checkout, or None."""
    try:
        s = json.loads(state.read_text(encoding="utf-8"))
        c = http.client.HTTPConnection("127.0.0.1", int(s["port"]), timeout=2)
        c.request("GET", f"/{s['token']}/api/ping")
        r = c.getresponse()
        same = r.status == 200 and json.loads(r.read())["root"] == str(root)
        c.close()
    except (OSError, ValueError, KeyError, TypeError):
        return None
    return f"http://127.0.0.1:{s['port']}/{s['token']}/" if same else None


def main(argv):
    root, url, browser = TOOLS.parent, PUBLISHED, True
    while argv and argv[0] in ("--root", "--site", "--no-browser"):  # the self-test's stand-in checkout and site
        if argv[0] == "--no-browser":
            browser, argv = False, argv[1:]
        else:
            root, url, argv = (Path(argv[1]), url, argv[2:]) if argv[0] == "--root" else (root, argv[1], argv[2:])
    if len(argv) == 2 and argv[0] == "robot" and ua.robot_id(argv[1]):  # RMP1-K D5, in the terminal (RMP1-L: white/black)
        root, robot = Path(root).resolve(), ua.robot_id(argv[1])
        via = robot_via_helper(root / "build" / "route_builder.json", root, robot)  # one pros on the brain at a time
        if via is None:
            have, why = read_slots(root)
            via = (slot_table(have) if have is not None else None, why)
        print(robot_words(robot, *via), flush=True)
        return 0 if via[0] is not None else 1
    if argv:
        print("Say nothing to open it: python3 tools/route_builder.py   (or robot white, robot black, self-test)")
        return 2
    root = Path(root).resolve()
    state = root / "build" / "route_builder.json"
    there = running(state, root)
    if there:
        print(f"The route builder is already running: {there}", flush=True)
        if browser:
            webbrowser.open(there)
        return 0
    srv = Server(root, url)
    link = f"http://127.0.0.1:{srv.port}/{srv.token}/"
    state.parent.mkdir(parents=True, exist_ok=True)
    state.write_text(json.dumps({"port": srv.port, "token": srv.token, "pid": os.getpid()}), encoding="utf-8")
    print(f"Route builder: {link}  (Ctrl+C stops it)", flush=True)
    print(f"The robots' facts: {srv.said} ({srv.data['build']['commit']}, {srv.data['build']['date']})"
          + ("" if srv.engine else "; no PREVIEW") if srv.data else NO_FACTS, flush=True)
    if srv.differs:
        print(srv.differs, flush=True)
    make, pros, _ = ua.find_tools()
    if not (make and pros):
        print(f"Send to robot will say ✗: {' and '.join(n for n, t in (('make', make), ('pros', pros)) if not t)} not found "
              f"(install PROS: {ua.PROS_HELP})", flush=True)
    for name in ("SIGTERM", "SIGHUP", "SIGBREAK"):  # a closed terminal or a stopped task: the same clean stop
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), signal.default_int_handler)
    if browser:
        webbrowser.open(link)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop(srv.child)
        close_reader(srv.reader)  # D3: the reader dies with the helper
        srv.server_close()
        try:
            if srv.token in state.read_text(encoding="utf-8"):  # still ours (never another start's)
                state.unlink()
        except OSError:
            pass
    say("Route builder stopped.")
    return 0


# ── self-test: the helper against a stand-in checkout, upload_auton's fake make and pros on PATH (its FAKE) ──

FAKE_CASE_FILE = """import os, time
with open(os.environ["FAKE_PIDS"], "a") as f:  # each fake's pid, and what it is ("make ROBOT=one", "pros upload")
    f.write(str(os.getpid()) + " " + " ".join(os.sys.argv[1:3]) + "\\n")
case = os.environ.get("FAKE_CASE_FILE")
if case and os.path.exists(case):  # the case for this send ("ok", "region", ...), how slow its make and its upload are
    os.environ["FAKE_CASE"], make_s, upload_s = (open(case).read().split() + ["0", "0"])[:3]
    step = os.sys.argv[1:3]
    time.sleep(float(make_s) if step[:1] == ["make"] else float(upload_s) if step == ["pros", "upload"] else 0)
if os.sys.argv[1:3] == ["pros", "lsusb"]:  # RMP1-K D3: a slow USB list (the window a send may start in)
    time.sleep(float(os.environ.get("FAKE_LSUSB_S", "0")))
"""


def stand_in(work, robots=None):
    """A stand-in checkout in `work` (repo/: src/team/, a fact outside it, the robots' facts and an engine) and
    upload_auton's fake make and pros (bin/); the environment that puts them first (and no VS Code folder: never this
    computer's PROS). Shared with the page tests."""
    shutil.rmtree(work, ignore_errors=True)
    repo = work / "repo"
    for d in ("src/team/sub", "src/robots", "src/programs", "build/route_maker_site/engine", "bin", "site/engine", "nohome"):
        (repo / d if not d.startswith(("bin", "site", "nohome")) else work / d).mkdir(parents=True)
    (repo / "src/team/skills_route.cpp").write_text(ua.SKILLS_STANDIN, encoding="utf-8")
    (repo / "src/robots/robot_one_facts.hpp").write_text("SECRET\n", encoding="utf-8")
    (repo / "src/programs/drive_program.cpp").write_text("driverLineMatchesFeel\n", encoding="utf-8")
    (work / "outside.txt").write_text("OUTSIDE\n", encoding="utf-8")
    fake = {"build": {"commit": "abc1234567", "date": "2026-11-20", "api": "2.9"},
            "robots": {"one": {"api": "2.9", "name": "robot one"}, "two": {"api": "2.9", "name": "robot two"}}}
    for folder in (repo / "build/route_maker_site", work / "site"):
        (folder / "robots.json").write_text(Path(robots).read_text(encoding="utf-8") if robots else json.dumps(fake), encoding="utf-8")
        (folder / "engine/worker.mjs").write_text("// the engine\n", encoding="utf-8")
        (folder / "engine/preview.wasm").write_bytes(b"\0asm")
    if robots:  # a real page: no stand-in engine (PREVIEW is the engine tests')
        shutil.rmtree(repo / "build/route_maker_site/engine")
    (work / "fake.py").write_text(FAKE_CASE_FILE + ua.FAKE, encoding="utf-8")
    for tool in ("make", "pros"):
        if os.name == "nt":
            (work / "bin" / f"{tool}.cmd").write_text(f'@"{sys.executable}" "{work / "fake.py"}" {tool} %*\n', encoding="utf-8")
        else:
            (work / "bin" / tool).write_text(f'#!/bin/sh\nexec "{sys.executable}" "{work / "fake.py"}" {tool} "$@"\n', encoding="utf-8")
            (work / "bin" / tool).chmod(0o755)
    home = str(work / "nohome")
    return {"root": str(repo), "python": sys.executable, "env": {"PATH": str(work / "bin"), "FAKE_CALLS": str(work / "calls.txt"),
            "FAKE_PIDS": str(work / "pids.txt"), "FAKE_CASE": "ok", "FAKE_CASE_FILE": str(work / "case.txt"),
            "APPDATA": home, "XDG_CONFIG_HOME": home, "HOME": home, "GIT_CEILING_DIRECTORIES": str(work)}}


def self_test():
    """Every case RMP1-I's brief and its review name; returns (the failures, how many cases)."""
    import contextlib
    import io
    import socket
    import time
    work = TOOLS.parent / "build" / "route_builder_selftest"
    made = stand_in(work)
    repo = work / "repo"
    symlink = True
    try:
        (repo / "src/team/escape.txt").symlink_to(work / "outside.txt")
    except OSError:  # Windows without the right to make one
        symlink = False
    saved_env = dict(os.environ)
    os.environ.update(made["env"])
    os.environ.pop("PROS_TOOLCHAIN", None)
    failures, count = [], [0]

    class WindowsLookup(subprocess.Popen):  # every OS as Windows: a bare program is looked up on OUR PATH, never env's
        def __init__(self, args, *a, **kw):
            if kw.get("env") is not None and not os.path.dirname(args[0]) and not shutil.which(args[0]):
                raise FileNotFoundError(2, "not on this process's PATH (Windows never looks on env's)", args[0])
            super().__init__(args, *a, **kw)
    real_popen, subprocess.Popen = subprocess.Popen, WindowsLookup

    def check(name, ok, said=""):
        count[0] += 1
        if not ok:
            failures.append(f"{name}: {said}")

    def ask(srv, method, path, body=None, headers=None, prefix=True, header=True, host=None):
        """A request to `srv`: the token in the path (`prefix`) and in the header (`header`) unless told otherwise."""
        c = http.client.HTTPConnection("127.0.0.1", srv.port, timeout=30)
        hd = {"Host": host or f"127.0.0.1:{srv.port}", **({"X-Shulib-Token": srv.token} if header else {}), **(headers or {})}
        c.request(method, (f"/{srv.token}" if prefix else "") + path, body=body, headers=hd)
        r = c.getresponse()
        out = (r.status, r.read(), dict(r.getheaders()))
        c.close()
        return out

    def raw(srv, head, body=b"", wait=10):
        """A request written by hand (a length the client would never send); the answer, b"" when none came in `wait` s."""
        s = socket.create_connection(("127.0.0.1", srv.port), timeout=wait)
        try:
            s.sendall(head.replace("{T}", srv.token).replace("{P}", str(srv.port)).encode() + b"\r\n\r\n" + body)
            return s.recv(65536)
        except TimeoutError:
            return b""
        finally:
            s.close()

    def alive(pid):
        if os.name == "nt":
            out = subprocess.run(["tasklist", "/FI", f"PID eq {pid}", "/NH", "/FO", "CSV"], capture_output=True, text=True).stdout
            return f'"{pid}"' in out
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        try:  # a zombie is gone: its parent just has not read its end yet
            return Path(f"/proc/{pid}/stat").read_text().split(") ")[1][0] != "Z"
        except OSError:
            return True

    def fakes(what=""):
        """The fakes' pids (those whose step contains `what`)."""
        p = work / "pids.txt"
        return [int(x.split()[0]) for x in p.read_text().splitlines() if what in x] if p.exists() else []

    def wait_for(cond, s=10):
        until = time.time() + s
        while not cond() and time.time() < until:
            time.sleep(0.05)
        return cond()

    def stream(srv, robot="one", until=None, each=lambda m: None):
        """POST a send; its lines as they arrive (stopping early at a line containing `until`; `each` sees every message),
        and the done code."""
        c = http.client.HTTPConnection("127.0.0.1", srv.port, timeout=10 if until else 60)
        c.request("POST", f"/{srv.token}/api/send?robot={robot}", headers={"Host": f"127.0.0.1:{srv.port}", "X-Shulib-Token": srv.token})
        r = c.getresponse()
        if r.status != 200:
            return r.status, [r.read().decode()], None, c
        lines, code = [], None
        while True:
            try:
                line = r.readline()
            except TimeoutError:  # nothing came while the send ran: not streamed
                return 200, lines, None, c
            if not line:
                break
            m = json.loads(line)
            each(m)
            if "line" in m:
                lines.append(m["line"])
                if until and until in m["line"]:
                    return 200, lines, None, c
            code = m.get("code", code)
        c.close()
        return 200, lines, code, None

    def serve(root=repo, url=None):
        srv = Server(root, url or "http://127.0.0.1:9/")
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        return srv

    def case_files(srv):
        st, body, hd = ask(srv, "GET", "/")
        page = body.decode()
        check("the page at /<token>/, with the meta", st == 200 and f'<meta name="shulib-local" content="{srv.token}" data-folder="repo">' in page, st)
        check("no cookie (every 127.0.0.1 port would get it)", "Set-Cookie" not in hd, hd)
        check("the page has the engine and the facts' build", 'content="engine/worker.mjs"' in page and "abc1234567" in page
              and "frame-ancestors 'none'" in hd.get("Content-Security-Policy", ""), hd)
        st, _, hd = ask(srv, "GET", "", header=False)
        check("/<token> goes to /<token>/", st == 303 and hd.get("Location") == f"/{srv.token}/", (st, hd))
        st, body, _ = ask(srv, "GET", "/src/ui/app.mjs", header=False)
        check("a module of the page", st == 200 and body == (site.SRC / "ui/app.mjs").read_bytes(), st)
        st, body, _ = ask(srv, "GET", "/engine/preview.wasm", header=False)
        check("the engine", st == 200 and body == b"\0asm", st)
        for path in ("/", "/app.html", "/src/ui/app.mjs", "/api/file?path=src/team/skills_route.cpp", f"/?t={srv.token}"):
            for name, pre in (("no token in the path", ""), ("a wrong token", "/" + "x" * 32), ("the token one short", f"/{srv.token[:-1]}")):
                check(f"{name}: {path}", ask(srv, "GET", pre + path, prefix=False)[0] == 403)
        for host in ("evil.example", f"evil.example:{srv.port}", f"127.0.0.1:{srv.port + 1}", "127.0.0.1"):
            check(f"Host {host}", ask(srv, "GET", "/app.html", host=host)[0] == 403)
        check("Host localhost", ask(srv, "GET", "/app.html", host=f"localhost:{srv.port}")[0] == 200)
        for path in ("/src/../../upload_auton.py", "/src/ui/", "/src/", "/build/route_builder.json", "/tools/route_builder.py",
                     "/src/team/skills_route.cpp", "/robots.json", "/x/app.html"):
            got = ask(srv, "GET", path)
            check(f"nothing else served: {path}", got[0] == 404 and b"SECRET" not in got[1], got[0])
        st, body, _ = ask(srv, "GET", "/api/file?path=src/team/skills_route.cpp")
        check("a read under src/team/", st == 200 and body.decode() == ua.SKILLS_STANDIN, st)
        check("a read of a missing file", ask(srv, "GET", "/api/file?path=src/team/none.cpp")[:2] == (204, b""))
        crlf = b"line 1\r\nline 2\r\n\xe2\x9c\x93\r\n"
        st, _, _ = ask(srv, "POST", "/api/file?path=src/team/autons.cpp", body=crlf)
        check("a write under src/team/, byte for byte", st == 200 and (repo / "src/team/autons.cpp").read_bytes() == crlf
              and ask(srv, "GET", "/api/file?path=src/team/autons.cpp")[1] == crlf, st)
        if os.name != "nt":
            check("a new file is rw-r--r--", (repo / "src/team/autons.cpp").stat().st_mode & 0o777 == 0o644)
            (repo / "src/team/autons.cpp").chmod(0o640)
            ask(srv, "POST", "/api/file?path=src/team/autons.cpp", body=b"x\n")
            check("a write keeps the file's mode", (repo / "src/team/autons.cpp").stat().st_mode & 0o777 == 0o640)
        bad = ["src/team/../robots/robot_one_facts.hpp", "../outside.txt", str(repo / "src/robots/robot_one_facts.hpp"),
               "src/robots/robot_one_facts.hpp", "src/team/sub", "src/team", "src/team/", "", "src/team/sub/../../robots/robot_one_facts.hpp"]
        bad += ["src/team/escape.txt"] if symlink else []
        for rel in bad:
            q = urllib.parse.quote(rel, safe="")
            st, body, _ = ask(srv, "GET", f"/api/file?path={q}")
            check(f"a read refused: {rel!r}", st == 403 and b"SECRET" not in body and b"OUTSIDE" not in body, (st, body))
            check(f"a write refused: {rel!r}", ask(srv, "POST", f"/api/file?path={q}", body=b"HACKED")[0] == 403)
        check("nothing written outside", (repo / "src/robots/robot_one_facts.hpp").read_text() == "SECRET\n"
              and (work / "outside.txt").read_text() == "OUTSIDE\n" and not list(work.rglob("*HACKED*")))
        before = (repo / "src/team/skills_route.cpp").read_bytes()
        target = "/api/file?path=src/team/skills_route.cpp"
        for name, kw in (("a write with the token only in the path (no header)", {"header": False}),
                         ("a write with the token only in the header", {"prefix": False}),
                         ("a write from another website", {"headers": {"Origin": "http://evil.example"}}),
                         ("a write from another local page", {"headers": {"Origin": f"http://127.0.0.1:{srv.port + 1}"}})):
            check(name, ask(srv, "POST", target, body=b"HACKED", **kw)[0] == 403)
        check("a write from the page itself", ask(srv, "POST", "/api/file?path=src/team/x.cpp", body=b"ok",
                                                  headers={"Origin": f"http://127.0.0.1:{srv.port}"})[0] == 200)
        post = "POST /{T}/api/file?path=src/team/skills_route.cpp HTTP/1.1\r\nHost: 127.0.0.1:{P}\r\nX-Shulib-Token: {T}"
        s = socket.create_connection(("127.0.0.1", srv.port), timeout=10)  # a write cut off mid-way: the old file stays
        s.sendall((post.replace("{T}", srv.token).replace("{P}", str(srv.port)) + "\r\nContent-Length: 4000\r\n\r\nHALF").encode())
        s.shutdown(socket.SHUT_WR)
        got = b"".join(iter(lambda: s.recv(4096), b""))
        s.close()
        check("a write cut off mid-way", b" 400 " in got and (repo / "src/team/skills_route.cpp").read_bytes() == before, got)
        check("a write with no length", b" 411 " in raw(srv, post, b"HACKED") and (repo / "src/team/skills_route.cpp").read_bytes() == before)
        check("a write with a negative length is refused at once (never read to the end)",
              b" 413 " in raw(srv, post + "\r\nContent-Length: -1", b"HACKED", wait=5) and (repo / "src/team/skills_route.cpp").read_bytes() == before)
        st, body, _ = ask(srv, "POST", target, body=b"\xff\xfe not text")
        check("a write that is not UTF-8 text", st == 400 and (repo / "src/team/skills_route.cpp").read_bytes() == before, (st, body))
        real = os.replace
        os.replace = lambda *a: (_ for _ in ()).throw(OSError(28, "No space left on device"))
        try:
            st, body, _ = ask(srv, "POST", target, body=b"NEW")
        finally:
            os.replace = real
        check("a write that fails at the end", st == 500 and b"No space" in body and (repo / "src/team/skills_route.cpp").read_bytes() == before
              and not list((repo / "src/team").glob(".*.part")), (st, body))
        check("a write too big", ask(srv, "POST", target, body=b"x", headers={"Content-Length": str(MAX_BODY + 1)})[0] in (413, 400))

    def case_send(srv):
        (repo / "src/team/skills_route.cpp").write_text(ua.SKILLS_STANDIN, encoding="utf-8")
        (work / "calls.txt").unlink(missing_ok=True)
        check("a GET never sends", ask(srv, "GET", "/api/send?robot=one")[0] == 404 and not (work / "calls.txt").exists())
        os.environ["FAKE_CASE"] = "ok"
        st, lines, code, _ = stream(srv)
        text = "\n".join(lines)
        want = ["Building White Bot's Auton Tester", "✓ BUILT", f"Uploading to slot {ua.SLOT} on /dev/ttyACM0",
                f"✓ UPLOADED to slot {ua.SLOT}. On the brain: shulib Auton Tester → skills or left 1 → RUN"]
        at = [text.find(w) for w in want]
        check("Send: the lines in order", st == 200 and code == 0 and -1 not in at and at == sorted(at), text)
        heard = sys.stdout.getvalue() if isinstance(sys.stdout, io.StringIO) else ""  # the helper's terminal (run() captures it)
        check("the helper's terminal names the robot (RMP1-L)", "Sending to White Bot...\n" in heard, heard[-300:])
        st, lines, code, _ = stream(srv, robot="two")
        heard = sys.stdout.getvalue() if isinstance(sys.stdout, io.StringIO) else ""
        check("... and Black Bot's send, Black Bot's", code == 0 and any(x.startswith("Building Black Bot's Auton Tester") for x in lines)
              and "Sending to Black Bot...\n" in heard, (lines, heard[-300:]))
        check("a robot that is not one or two", ask(srv, "POST", "/api/send?robot=three")[0] == 400)
        check("Send needs the header", ask(srv, "POST", "/api/send?robot=one", header=False)[0] == 403)

        class Gone(io.StringIO):  # the helper's terminal closed
            def write(self, _):
                raise OSError(5, "Input/output error")
        with contextlib.redirect_stdout(Gone()):
            st, lines, code, _ = stream(srv)
        check("a closed terminal never stops a send, nor keeps it", code == 0 and any("✓ UPLOADED" in x for x in lines)
              and wait_for(lambda: srv.child is None), (code, lines, srv.child))
        os.environ["FAKE_CASE"] = "region"
        st, lines, code, _ = stream(srv)
        text = "\n".join(lines)
        check("Send: a compile error in plain words", code == 1 and "✗ NOT BUILT" in text and "✓ BUILT" not in text
              and "src/team/skills_route.cpp:6: 'r' was not declared in this scope" in text and "re-export" in text, text)
        st, body, _ = ask(srv, "GET", "/api/log")
        check("Show everything: make's own lines", st == 200 and b"[ERRORS]" in body, body[:200])
        (work / "pids.txt").unlink(missing_ok=True)
        os.environ["FAKE_CASE"] = "slow"  # make prints a line, then takes 30 s
        st, lines, _, conn = stream(srv, until="Building")
        live = srv.child is not None and srv.child.poll() is None
        check("Send streams as the lines come (make still running)", st == 200 and live and any("Building" in x for x in lines), lines)
        wait_for(lambda: fakes("make"))
        st, body, _ = ask(srv, "POST", "/api/send?robot=two")
        check("a second Send while one runs", st == 409 and b"already running" in body and srv.busy(), (st, body))
        before = (repo / "src/team/skills_route.cpp").read_bytes()
        st, body, _ = ask(srv, "POST", "/api/file?path=src/team/skills_route.cpp", body=b"CHANGED MID-BUILD")
        check("no write while a send builds", st == 409 and b"send to the robot is running" in body
              and (repo / "src/team/skills_route.cpp").read_bytes() == before, (st, body))
        child = srv.child
        stop(child)
        if conn:
            conn.close()
        wait_for(lambda: not any(alive(x) for x in fakes()))
        check("stopping a send ends make too", child is not None and child.poll() is not None and fakes()
              and not any(alive(x) for x in fakes()), fakes())
        # the tab closes mid-send: the send carries on to its end, still this helper's (a second Send: 409) until it ends
        (work / "pids.txt").unlink(missing_ok=True)
        (work / "case.txt").write_text("ok 1 3", encoding="utf-8")  # make takes 1 s, the upload 3 s
        try:
            st, lines, _, conn = stream(srv, until="Building")
            child = srv.child
            conn.close()
            uploading = wait_for(lambda: fakes("pros upload"), 20)
            owned = srv.child is child and child.poll() is None and ask(srv, "POST", "/api/send?robot=one")[0] == 409
            wait_for(lambda: child.poll() is not None, 30)
            check("a tab closed mid-send: the send carries on and stays the helper's", uploading and owned and child.returncode == 0
                  and wait_for(lambda: srv.child is None), (uploading, owned, child.returncode, srv.child))
        finally:
            (work / "case.txt").unlink(missing_ok=True)
        # a send has ended but its handler has not let go (this test holds the lock it ends under), and the next send
        # starts: the ended one is not busy, and its handler's end never clears the next one's child
        (work / "case.txt").write_text("ok 1 0", encoding="utf-8")
        seen = {}

        def hold(m):
            if "a" not in seen and "Building" in m.get("line", ""):
                srv.lock.acquire()
                seen["a"] = srv.child
            elif "a" in seen and "b" not in seen and m.get("done"):
                seen["ended"] = srv.child is seen["a"] and seen["a"].poll() is not None and not srv.busy()
                srv.child = seen["b"] = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"], **GROUP)
                srv.lock.release()
        try:
            code = stream(srv, each=hold)[2]
            check("a send that has ended is not busy (its handler still finishing)", code == 0 and seen.get("ended"), seen)
            check("... and its end never clears the next send's child", seen.get("b") is not None and srv.child is seen["b"]
                  and srv.busy(), (srv.child, seen))
        finally:
            if "a" in seen and "b" not in seen:
                srv.lock.release()
            stop(seen.get("b"))
            with srv.lock:
                srv.child = None
            (work / "case.txt").unlink(missing_ok=True)

        done = subprocess.Popen([sys.executable, "-c", "pass"])
        done.wait()

        class Ended:  # a send whose end another thread is reading: poll() says None, the process is gone
            pid = done.pid
            poll = staticmethod(lambda: None)
            wait = staticmethod(lambda t=None: 0)
        try:
            stop(Ended())
            check("stopping a send that has just ended", True)
        except Exception as e:  # noqa: BLE001
            check("stopping a send that has just ended", False, repr(e))

    # ── RMP1-K: Watch the robot (D2/D3) and Check robot one (D5), against the fake pros's terminal and v5 commands ──
    log1 = TOOLS / "route_maker" / "test" / "fixtures" / "robot_log" / "robot_one_2026-11-19_run3.log"

    def more(r, until=None, s=30):
        """A watch's messages from response `r` up to the first `until(m)` holds for (None: to its end), at most `s` s (its
        pings would keep a wait alive forever); pings left out."""
        got, end = [], time.time() + s
        while time.time() < end:
            try:
                line = r.readline()
            except TimeoutError:
                return got
            if not line:
                return got
            m = json.loads(line)
            if m:
                got.append(m)
                if until and until(m):
                    return got
        return got

    def watch_stream(srv, until=None, timeout=20):
        """POST a watch: (status, its messages up to `until` (more()), the connection, the response)."""
        c = http.client.HTTPConnection("127.0.0.1", srv.port, timeout=timeout)
        c.request("POST", f"/{srv.token}/api/watch", headers={"Host": f"127.0.0.1:{srv.port}", "X-Shulib-Token": srv.token})
        r = c.getresponse()
        if r.status != 200:
            return r.status, [r.read().decode()], c, r
        return 200, more(r, until), c, r

    def readers():
        return [x for x in fakes("terminal") if alive(x)]

    def case_watch(srv):
        unplug = work / "unplug"
        unplug.unlink(missing_ok=True)
        os.environ.update(FAKE_CASE="ok", FAKE_TERMINAL=str(log1), FAKE_TERMINAL_EVERY="0", FAKE_UNPLUG=str(unplug))
        (work / "pids.txt").unlink(missing_ok=True)
        want = log1.read_text(encoding="utf-8", errors="replace").split("\n")
        last = want[-1] if want[-1] else want[-2]
        st, got, conn, resp = watch_stream(srv, until=lambda m: m.get("line") == last)
        lines = [m["line"] for m in got if "line" in m]
        opened = [m.get("open") for m in got if "open" in m]
        check("watch: the brain's USER port, from pros lsusb (never a guess)", opened == ["/dev/ttyACM1"], got[:3])
        check("watch: the robot's lines as they come, the real 11-19 run whole", lines[1:] == [x.rstrip("\r") for x in want[:len(lines) - 1]]
              and lines[0] == "Established terminal connection" and any("1 of 23 steps done" in x for x in lines), lines[:3])
        saved = srv.root / [m["saved"] for m in got if "saved" in m][0]
        check("watch: saved to build/robot_runs/", saved.parent == srv.root / "build" / "robot_runs"
              and "END estimate (52.51, -13.32)" in saved.read_text(encoding="utf-8"), saved)
        check("watch: the reader is the helper's child, given a terminal (pros's console needs one)", len(readers()) == 1
              and srv.reader is not None and srv.reader.poll() is None, readers())
        st2, body, _, _ = watch_stream(srv)
        check("a second reader refused", st2 == 409 and "already open" in body[0], (st2, body))
        # a Send closes the reader first (the brain's port is the upload's), then the watch reopens it
        reader = srv.reader
        st3, sent, code, _ = stream(srv)
        closed = reader.poll() is not None
        rest = more(resp, lambda m: "open" in m)
        check("a Send closes the reader before its upload, and the watch reopens it after", code == 0 and closed
              and any("paused" in m for m in rest) and rest[-1].get("open") == "/dev/ttyACM1"
              and not any("Traceback" in m.get("line", "") for m in rest) and wait_for(lambda: len(readers()) == 1), (code, closed, rest))
        unplug.write_text("", encoding="utf-8")  # the cable pulled mid-run
        end = more(resp)
        check("a port that vanishes mid-run: said plainly, the watch ends, no reader left", end and end[-1].get("error") == UNPLUGGED
              and wait_for(lambda: not readers() and srv.reader is None and not srv.watching), (end[-3:], readers()))
        resp.close()
        conn.close()
        unplug.unlink()
        os.environ["FAKE_CASE"] = "controller"
        st, got, _, _ = watch_stream(srv)
        check("a controller-only USB: said plainly, no reader", st == 200 and got[-1].get("error", "").startswith("Only the controller")
              and not readers(), got)
        os.environ["FAKE_CASE"] = "ok"
        st, got, conn, resp = watch_stream(srv, until=lambda m: "open" in m)
        resp.close()  # the tab closed: the next ping finds it, the reader goes
        conn.close()
        gone = wait_for(lambda: not readers() and not srv.watching, 10)
        st, got, conn, resp = watch_stream(srv, until=lambda m: "open" in m)
        check("a tab closed: its reader goes, and the next watch may open", gone and st == 200 and got[-1].get("open"), (readers(), got))
        resp.close()
        conn.close()
        wait_for(lambda: not srv.watching and not readers(), 10)
        # a send that starts while the watch is still listing the USB: the watch re-checks under the lock and opens no
        # reader beside the upload (the send closed "the" reader before this one existed), then opens it after
        os.environ.update(FAKE_CASE="ok", FAKE_LSUSB_S="1.5")
        (work / "case.txt").write_text("ok 0 3", encoding="utf-8")
        (work / "pids.txt").unlink(missing_ok=True)
        conn = resp = None
        try:
            st, got, conn, resp = watch_stream(srv, until=lambda m: "saved" in m)
            wait_for(lambda: fakes("lsusb"), 5)
            sent, beside = [], []
            th = threading.Thread(target=lambda: sent.append(stream(srv)))
            th.start()
            while th.is_alive():
                if srv.busy() and readers():
                    beside.append(readers())
                time.sleep(0.05)
            th.join()
            rest = more(resp, lambda m: "open" in m)
            check("a send that starts while the watch lists the USB: no reader beside the upload; the watch opens after",
                  sent and sent[0][2] == 0 and not beside and any("paused" in m for m in rest) and rest[-1].get("open") == "/dev/ttyACM1",
                  (beside, rest, sent and sent[0][:3]))
        finally:
            os.environ["FAKE_LSUSB_S"] = "0"
            (work / "case.txt").unlink(missing_ok=True)
            for x in (resp, conn):
                if x is not None:
                    x.close()
            wait_for(lambda: not srv.watching and not readers(), 10)

    def case_robot(srv):
        good = {str(n): [w or "VEXcode", f"v0.1.1-470-gabc1234 | skills" if n in (2, 6) else "Made with PROS", f"2026-11-20T0{n}:00:00"]
                for n, w in WANTED.items()}
        os.environ.update(FAKE_CASE="ok", FAKE_SLOTS=json.dumps(good))
        (work / "calls.txt").unlink(missing_ok=True)
        st, body, _ = ask(srv, "POST", "/api/robot?robot=one")
        got = json.loads(body)
        rows = got["rows"] or [{}] * 8
        calls = (work / "calls.txt").read_text(encoding="utf-8").splitlines()
        check("Check robot one: every slot right (pros's own 'Loading' and 'Downloading slot_N.ini' lines around the answer)",
              st == 200 and [r.get("ok") for r in rows] == [True] * 8 and rows[1]["have"] == "shulib Competition"
              and rows[5]["description"] == "v0.1.1-470-gabc1234 | skills" and rows[0]["date"] == "2026-11-20T01:00:00", got)
        check("... read-only: lsusb, then ONE pros user-script tools/brain_slots.py (the fake V5Device can only read)",
              calls == ["pros lsusb", f"pros user-script {TOOLS / 'brain_slots.py'}"], calls)
        check("the old code's slot (8): never touched by shulib", rows[7]["slot"] == 8 and rows[7]["ok"]
              and rows[7]["want"] == "the team's old code" and rows[7]["fix"] == "never touched by shulib", rows[7])
        check("the PROS button's slot (5): optional, never judged", rows[4]["slot"] == 5 and rows[4]["ok"]
              and rows[4]["want"] == "the PROS button's build (optional)" and rows[4]["fix"] == "optional, never judged", rows[4])
        out = subprocess.run([sys.executable, str(Path(__file__).resolve()), "--root", str(repo), "robot", "one"], capture_output=True,
                             text=True, encoding="utf-8", timeout=60, env=dict(os.environ, PYTHONIOENCODING="utf-8"))
        check("python3 tools/route_builder.py robot one: each slot's whole line (its upload time and its build's tag)", out.returncode == 0
              and out.stdout.startswith("White Bot: what is in each slot") and "  ✓ slot 6: shulib Auton Tester  uploaded 2026-11-20 06:00  [v0.1.1-470-gabc1234 | skills]\n" in out.stdout
              and "  ✓ slot 8: VEXcode  uploaded 2026-11-20 08:00  [Made with PROS]\n" in out.stdout, out.stdout + out.stderr)
        bad = dict(good, **{"2": ["shulib Auton Tester", "x", "2026-11-20T02:00:00"], "8": ["somebody's old code", "", ""]})
        del bad["6"]
        del bad["5"]
        os.environ["FAKE_SLOTS"] = json.dumps(bad)
        rows = json.loads(ask(srv, "POST", "/api/robot?robot=one")[1])["rows"]
        check("one slot wrong: the plain fix", not rows[1]["ok"] and rows[1]["fix"] == "slot 2 holds shulib Auton Tester: upload shulib Competition there", rows[1])
        check("one slot empty: the plain fix", not rows[5]["ok"] and rows[5]["fix"].startswith("slot 6 is empty: upload shulib Auton Tester there")
              and rows[5]["fix"].endswith("(it is in slot 2 now)"), rows[5])
        check("the old code's slot left alone, whatever is in it", rows[7]["ok"] and rows[7]["have"] == "somebody's old code", rows[7])
        check("the PROS button's slot empty: still fine (it is optional)", rows[4]["ok"] and rows[4]["have"] is None, rows[4])
        out = subprocess.run([sys.executable, str(Path(__file__).resolve()), "--root", str(repo), "robot", "White"], capture_output=True,
                             text=True, encoding="utf-8", timeout=60, env=dict(os.environ, PYTHONIOENCODING="utf-8"))
        check("python3 tools/route_builder.py robot White (RMP1-L): the table in the terminal, in White Bot's name", out.returncode == 0
              and out.stdout.startswith("White Bot: what is in each slot") and "✗ slot 2: shulib Auton Tester -- slot 2 holds shulib Auton Tester: upload shulib Competition there" in out.stdout
              and "✓ slot 1: shulib Drive  uploaded 2026-11-20 01:00" in out.stdout, out.stdout + out.stderr)
        out = subprocess.run([sys.executable, str(Path(__file__).resolve()), "--root", str(repo), "robot", "grey"], capture_output=True,
                             text=True, encoding="utf-8", timeout=60, env=dict(os.environ, PYTHONIOENCODING="utf-8"))
        check("a robot it does not know: how to say it, in the names", out.returncode == 2 and out.stdout
              == "Say nothing to open it: python3 tools/route_builder.py   (or robot white, robot black, self-test)\n", out.stdout + out.stderr)
        os.environ["FAKE_SLOTS"] = json.dumps(dict(good, **{"2": "unread"}))
        rows = json.loads(ask(srv, "POST", "/api/robot?robot=one")[1])["rows"]
        check("a slot whose file cannot be read: said so, never 'empty' or 'holds ?: upload'", not rows[1]["ok"] and rows[1]["unread"]
              and rows[1]["have"] is None and rows[1]["fix"] == "slot 2 could not be read (pros could not read slot_2.ini): check again"
              and [r["ok"] for r in rows] == [True, False, True, True, True, True, True, True], rows[1])
        os.environ["FAKE_SLOTS"] = json.dumps(dict(good, **{"8": ["VEXcode", "100% auton", "2026-11-20T08:00:00"]}))
        got = json.loads(ask(srv, "POST", "/api/robot?robot=one")[1])
        rows = got["rows"] or [{}] * 8
        check("another tool's '%' in slot 8 (pros's ConfigParser interpolates): the whole table, slot 8's row as written",
              [r.get("ok") for r in rows] == [True] * 8 and not rows[7]["unread"] and rows[7]["have"] == "VEXcode"
              and rows[7]["description"] == "100% auton" and rows[1]["have"] == "shulib Competition", got)
        os.environ.update(FAKE_SLOTS=json.dumps(good), FAKE_CASE="slotsfail")
        got = json.loads(ask(srv, "POST", "/api/robot?robot=one")[1])
        check("the brain does not answer pros: said in pros's words, no table", got["rows"] is None and got["why"]
              == "pros could not read the brain's programs: OSError: could not open port /dev/ttyACM0: the brain did not answer", got)
        os.environ["FAKE_CASE"] = "lsusbbad"
        got = json.loads(ask(srv, "POST", "/api/robot?robot=one")[1])
        check("a USB list pros says failed is not trusted (no read on a port from it)", got["rows"] is None
              and got["why"].startswith("pros lsusb did not list the brain's system port"), got)
        os.environ["FAKE_CASE"] = "nobrain"
        got = json.loads(ask(srv, "POST", "/api/robot?robot=two")[1])
        check("no brain: said plainly", got["rows"] is None and got["why"].startswith("No brain found"), got)
        os.environ["FAKE_CASE"] = "ok"
        # a terminal's check asks the route builder running on this checkout (its lock), never a second pros beside it
        state = repo / "build" / "route_builder.json"
        state.parent.mkdir(parents=True, exist_ok=True)
        state.write_text(json.dumps({"port": srv.port, "token": srv.token, "pid": os.getpid()}), encoding="utf-8")
        try:
            (work / "calls.txt").unlink(missing_ok=True)
            srv.checking = True  # the page is reading the slots right now
            out = subprocess.run([sys.executable, str(Path(__file__).resolve()), "--root", str(repo), "robot", "one"], capture_output=True,
                                 text=True, encoding="utf-8", timeout=60, env=dict(os.environ, PYTHONIOENCODING="utf-8"))
            srv.checking = False
            check("robot one beside a running route builder: asks it, and its 'busy' stands (no pros of its own)", out.returncode == 1
                  and out.stdout.startswith("White Bot: the route builder running on this checkout says: The brain is busy")
                  and not (work / "calls.txt").exists(), out.stdout + out.stderr)
            out = subprocess.run([sys.executable, str(Path(__file__).resolve()), "--root", str(repo), "robot", "black"], capture_output=True,
                                 text=True, encoding="utf-8", timeout=60, env=dict(os.environ, PYTHONIOENCODING="utf-8"))
            check("... and its table when it is free (robot black: Black Bot's)", out.returncode == 0 and "✓ slot 2: shulib Competition" in out.stdout
                  and out.stdout.startswith("Black Bot: what is in each slot"), out.stdout + out.stderr)
        finally:
            srv.checking = False
            state.unlink(missing_ok=True)
        os.environ["FAKE_CASE"] = "slow"
        st, lines, _, conn = stream(srv, until="Building")
        check("no check while a send uses the brain", ask(srv, "POST", "/api/robot?robot=one")[0] == 409)
        stop(srv.child)
        conn.close()
        wait_for(lambda: srv.child is None or srv.child.poll() is not None)
        os.environ["FAKE_CASE"] = "ok"
        srv.checking = True  # a slot read holds the brain's port: no send starts under it
        st, body, _, conn = stream(srv)
        srv.checking = False
        if conn is not None:
            conn.close()
        check("no send while the slots are read", st == 409 and "Reading the robot's slots" in body[0], (st, body[:2]))

    def case_facts():
        global MAX_DOWNLOAD
        local = repo / "build" / "route_maker_site"
        hold = repo / "build" / "site.hold"
        cache = repo / "build" / "route_builder_cache"
        old, part = cache.with_name(cache.name + ".old"), cache.with_name(cache.name + ".part")
        pub = ThreadingHTTPServer(("127.0.0.1", 0), lambda *a: _Static(*a, directory=str(work / "site")))
        threading.Thread(target=pub.serve_forever, daemon=True).start()
        up = f"http://127.0.0.1:{pub.server_address[1]}/"
        cut = ThreadingHTTPServer(("127.0.0.1", 0), _Cut)
        threading.Thread(target=cut.serve_forever, daemon=True).start()
        bad = f"http://127.0.0.1:{cut.server_address[1]}/"
        try:
            site_json = json.loads((work / "site/robots.json").read_text(encoding="utf-8"))
            site_json["build"]["commit"] = "def9876543"
            (work / "site/robots.json").write_text(json.dumps(site_json), encoding="utf-8")
            assert download(up, cache)
            data, engine, said = facts(repo, up)
            check("this laptop's build first, a kept download or not", said == "this laptop's build" and data["build"]["commit"] == "abc1234567", said)
            local.rename(hold)
            data, engine, said = facts(repo, up)
            check("no local build: the published site's, kept", data and engine and said == "the published site"
                  and (cache / "engine/preview.wasm").read_bytes() == b"\0asm", said)
            def kept():  # the cache as it was, and no half of a swap left behind
                w = cache / "engine/preview.wasm"
                return (not old.exists() and not part.exists() and w.is_file() and w.read_bytes() == b"\0asm"
                        and json.loads((cache / "robots.json").read_text())["build"]["commit"] == "def9876543")
            limit, MAX_DOWNLOAD = MAX_DOWNLOAD, 1 << 10  # "big": a whole file, one byte more than this
            try:
                for shape in ("big", "length", "chunked", "notwasm"):
                    _Cut.shape = shape
                    try:
                        got = download(bad, cache)
                    except Exception as e:  # noqa: BLE001
                        got = repr(e)
                    check(f"a download cut short or too big ({shape}) keeps the old", got is False and kept(), got)
            finally:
                MAX_DOWNLOAD = limit
            site_json["build"]["commit"] = "new0000000"
            (work / "site/robots.json").write_text(json.dumps(site_json), encoding="utf-8")
            real_rename = Path.rename
            Path.rename = lambda p, to: (_ for _ in ()).throw(OSError(13, "in use")) if p.name.endswith(".part") else real_rename(p, to)
            try:
                got = download(up, cache)
            finally:
                Path.rename = real_rename
            check("a download the swap cannot move in: the old put back", got is False and kept(), got)
            cache.rename(old)  # a swap cut between its two renames (the laptop off)
            check("... and a swap cut short is put back at the next start, offline", download("http://127.0.0.1:9/", cache) is False
                  and kept(), sorted(x.name for x in cache.parent.iterdir()))
            (work / "site/robots.json").write_text("{not json", encoding="utf-8")
            data, engine, said = facts(repo, up)
            check("a broken download keeps the old", data and data["build"]["commit"] == "def9876543"
                  and said == "the published site, kept from the last time online", said)
            srv = serve(repo, bad)
            st, body, _ = ask(srv, "GET", "/")
            check("the published site's facts: the page says when this checkout's robots may differ", srv.differs and st == 200
                  and b"Careful: the robots' facts are the published site's (def9876543)" in body, srv.differs)
            srv.shutdown()
            srv.server_close()
            pub.shutdown()
            pub.server_close()
            data, engine, said = facts(repo, up)
            check("offline: the kept facts", data and engine and "kept" in said, said)
            shutil.rmtree(cache, ignore_errors=True)
            srv = serve(repo, up)
            st, body, _ = ask(srv, "GET", "/")
            check("nothing anywhere: one plain line, still open", srv.data is None and st == 200 and html.escape(NO_FACTS).encode() in body, (st, body[:200]))
            hold.rename(local)
            st, body, _ = ask(srv, "GET", "/")
            check("... and the route builder at the next load once there are facts", st == 200 and b'name="shulib-local"' in body, body[:200])
            srv.shutdown()
            srv.server_close()
        finally:
            if hold.exists():
                hold.rename(local)
            cut.shutdown()
            cut.server_close()

    def case_differs():
        git = work / "git"
        (git / "src/robots").mkdir(parents=True)
        (git / "src/robots/robot_two_facts.hpp").write_text("kLift = 1;\n", encoding="utf-8")
        run = lambda *a: subprocess.run(["git", "-C", str(git), "-c", "user.name=t", "-c", "user.email=t@t", *a],  # noqa: E731
                                        capture_output=True, text=True, timeout=60)
        os.environ["PATH"] = saved_env.get("PATH", "")  # this computer's git (Windows finds a program on OUR PATH only)
        try:
            if run("init", "-q").returncode:
                return check("git is here (facts_differ)", False, "git init failed")
            run("add", "-A")
            run("commit", "-qm", "facts")
            head = run("rev-parse", "--short=10", "HEAD").stdout.strip()
            check("the facts' commit, this checkout's robots unchanged: nothing to say", facts_differ(git, head) is None, head)
            (git / "src/robots/robot_two_facts.hpp").write_text("kLift = 2;\n", encoding="utf-8")
            check("a robot's facts changed since: said", "differ" in (facts_differ(git, head) or ""))
            check("a commit this checkout does not have: said", facts_differ(git, "0123456789") is not None)
        finally:
            os.environ["PATH"] = made["env"]["PATH"]

    def bound(port):
        """A socket on `port` bound as the Server binds (a closed connection's TIME_WAIT does not hold it on POSIX)."""
        s = socket.socket()
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, int(Server.allow_reuse_address))
        s.bind(("127.0.0.1", port))
        return s

    def case_port():
        free = True
        try:
            bound(preferred_port(repo)).close()
        except OSError:
            free = False  # another program has it here: the fallback below still runs
        a = serve()
        a.shutdown()
        a.server_close()
        b = serve()
        b.shutdown()
        b.server_close()
        check("the same port at every start (the page's draft is kept per port)", not free or a.port == b.port == preferred_port(repo), (a.port, b.port))
        hold = bound(a.port)
        hold.listen()
        try:
            c = serve()
            ok = c.port != a.port and ask(c, "GET", "/")[0] == 200
            c.shutdown()
            c.server_close()
            check("its port taken by another program: another free one", ok, c.port)
        finally:
            hold.close()

    def case_process():
        if os.name == "nt":
            sigs = [("Ctrl+Break", signal.CTRL_BREAK_EVENT)]
        else:
            sigs = [("Ctrl+C", signal.SIGINT), ("a stopped VS Code task (SIGTERM)", signal.SIGTERM), ("a closed terminal (SIGHUP)", signal.SIGHUP)]
        env = dict(os.environ, FAKE_CASE="slow", PYTHONIOENCODING="utf-8")
        cmd = [sys.executable, str(Path(__file__).resolve()), "--root", str(repo), "--site", "http://127.0.0.1:9/", "--no-browser"]
        for k, (name, sig) in enumerate(sigs):
            a = subprocess.Popen(cmd, env=env, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 **({"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP} if os.name == "nt" else {}))
            try:
                first = a.stdout.readline().decode()
                state = repo / "build/route_builder.json"
                if k == 0:
                    b = subprocess.run(cmd, env=env, stdin=subprocess.DEVNULL, capture_output=True, timeout=30)
                    check("a second start opens the running one", first.startswith("Route builder: http://127.0.0.1:") and b.returncode == 0
                          and first.split()[2] in b.stdout.decode() and "already running" in b.stdout.decode() and a.poll() is None, (first, b.stdout))
                    check("... only on this checkout", running(state, repo) == first.split()[2] and running(state, work / "elsewhere") is None)
                s = json.loads(state.read_text())
                (work / "pids.txt").unlink(missing_ok=True)
                c = http.client.HTTPConnection("127.0.0.1", s["port"], timeout=60)
                c.request("POST", f"/{s['token']}/api/send?robot=one", headers={"Host": f"127.0.0.1:{s['port']}", "X-Shulib-Token": s["token"]})
                r = c.getresponse()
                wait_for(lambda: fakes("make"))
                a.send_signal(sig)
                out = first + a.communicate(timeout=60)[0].decode()
                c.close()
                wait_for(lambda: not any(alive(x) for x in fakes()))
                check(f"{name} ends the helper and make, cleanly", a.returncode == 0 and r.status == 200 and fakes()
                      and not any(alive(x) for x in fakes()) and "Traceback" not in out and "Route builder stopped." in out
                      and not state.exists(), (a.returncode, fakes(), out))
            finally:
                if a.poll() is None:
                    a.kill()
                    a.wait()
        # RMP1-K D3: the robot's log reader is the helper's child and ends with it
        name, sig = sigs[-1] if os.name == "nt" else ("a stopped VS Code task (SIGTERM)", signal.SIGTERM)
        env = dict(os.environ, FAKE_CASE="ok", FAKE_TERMINAL=str(log1), PYTHONIOENCODING="utf-8")
        a = subprocess.Popen(cmd, env=env, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             **({"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP} if os.name == "nt" else {}))
        try:
            a.stdout.readline()
            s = json.loads((repo / "build/route_builder.json").read_text())
            (work / "pids.txt").unlink(missing_ok=True)
            c = http.client.HTTPConnection("127.0.0.1", s["port"], timeout=30)
            c.request("POST", f"/{s['token']}/api/watch", headers={"Host": f"127.0.0.1:{s['port']}", "X-Shulib-Token": s["token"]})
            r = c.getresponse()
            opened = wait_for(lambda: [x for x in fakes("terminal") if alive(x)])
            a.send_signal(sig)
            a.communicate(timeout=60)
            r.close()
            c.close()
            check(f"{name} ends the robot's log reader too", r.status == 200 and opened and fakes("terminal")
                  and wait_for(lambda: not any(alive(x) for x in fakes("terminal"))), fakes("terminal"))
        finally:
            if a.poll() is None:
                a.kill()
                a.wait()

    tasks = json.loads("".join(x for x in (TOOLS.parent / ".vscode/tasks.json").read_text(encoding="utf-8").splitlines(True)
                               if not x.lstrip().startswith("//")))["tasks"][0]
    check("VS Code's first task opens it, by the name the docs and the page use", tasks["label"] == "Open the route builder"
          and tasks["args"] == ["tools/route_builder.py"] and tasks["isBackground"] and tasks["windows"]["command"] == "python", tasks)
    every = json.loads("".join(x for x in (TOOLS.parent / ".vscode/tasks.json").read_text(encoding="utf-8").splitlines(True)
                               if not x.lstrip().startswith("//")))["tasks"]
    for robot, label in (("one", "White Bot (robot one)"), ("two", "Black Bot (robot two)")):  # RMP1-L D3: the names, the bridge
        t = [x for x in every if x["label"] == f"What is on {label}"]
        check(f"VS Code's task 'What is on {label}' (RMP1-K D5)", len(t) == 1 and t[0]["args"] == ["tools/route_builder.py", "robot", robot]
              and t[0]["windows"]["command"] == "python", t)
        t = [x for x in every if x["label"] in (f"Upload routes to {label}", f"Upload everything to {label}")]
        check(f"VS Code's two upload tasks for {label}", [x["args"] for x in t] == [["tools/upload_auton.py", robot], ["tools/upload_auton.py", robot, "all"]], t)
    picker = json.loads("".join(x for x in (TOOLS.parent / ".vscode/tasks.json").read_text(encoding="utf-8").splitlines(True)
                                if not x.lstrip().startswith("//")))["inputs"][0]["options"]
    check("VS Code's robot picker: the names, the values one and two", picker == [{"label": "White Bot (robot one)", "value": "one"},
                                                                                  {"label": "Black Bot (robot two)", "value": "two"}], picker)
    check("no task still says 'robot one' or 'robot two' alone", not [x["label"] for x in every if re.search(r"(?<!\()robot (one|two)", x["label"])], every)

    def run(case, *args):  # a crash is a named failure, and the next case still runs
        try:
            case(*args)
        except Exception as e:  # noqa: BLE001 -- the self-test reports every way a case can end
            failures.append(f"{case.__name__}: crashed: {e!r}")

    run(case_port)
    srv = serve()
    try:
        with contextlib.redirect_stdout(io.StringIO()):  # the helper's own terminal lines
            run(case_files, srv)
            run(case_send, srv)
            run(case_watch, srv)
            run(case_robot, srv)
    finally:
        stop(srv.child)
        srv.shutdown()
        srv.server_close()
    run(case_facts)
    run(case_differs)
    run(case_process)
    subprocess.Popen = real_popen
    os.environ.clear()
    os.environ.update(saved_env)
    shutil.rmtree(work, ignore_errors=True)
    return failures, count[0]


class _Static(SimpleHTTPRequestHandler):  # the self-test's stand-in published site
    def log_message(self, *args):
        pass


class _Cut(BaseHTTPRequestHandler):  # the self-test's flaky venue WiFi: robots.json whole; the engine too big, cut short or not one
    shape = "length"

    def log_message(self, *args):
        pass

    def do_GET(self):
        body = json.dumps({"build": {"commit": "cut0000000", "date": "2026-11-21", "api": "2.9"},
                           "robots": {"one": {"api": "2.9"}, "two": {"api": "2.9"}}}).encode()
        if self.path.endswith("robots.json"):
            self.send_response(200)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            return self.wfile.write(body)
        self.protocol_version = "HTTP/1.1"
        self.send_response(200)
        if self.shape == "chunked":
            self.send_header("Transfer-Encoding", "chunked")
            self.end_headers()
            self.wfile.write(b"4\r\n\0asm\r\n")  # and no last chunk: the connection ends
        elif self.shape == "big":  # whole (no length: to the connection's end), one byte more than MAX_DOWNLOAD
            self.end_headers()
            self.wfile.write(b"\0asm" + b"x" * (MAX_DOWNLOAD - 3))
        else:
            self.send_header("Content-Length", "100" if self.shape == "length" else "6")
            self.end_headers()
            self.wfile.write(b"\0asm" if self.shape == "length" else b"<html>")
        self.close_connection = True


if __name__ == "__main__":
    sys.stdout.reconfigure(errors="replace")  # what this console cannot print becomes "?", never a crash
    if sys.argv[1:2] == ["stand-in"]:  # the page tests' checkout: stand-in <folder> [<robots.json>]
        print(json.dumps(stand_in(Path(sys.argv[2]).resolve(), *sys.argv[3:4])))
        sys.exit(0)
    if sys.argv[1:] == ["self-test"]:
        problems, n = self_test()
        print("\n".join(problems) if problems else f"route_builder self-test: OK ({n} cases against a stand-in checkout)")
        sys.exit(1 if problems else 0)
    try:
        sys.exit(main(sys.argv[1:]))
    except KeyboardInterrupt:  # Ctrl+C before it was serving (the download at its start)
        say("Route builder stopped.")
        sys.exit(0)
