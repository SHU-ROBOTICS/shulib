#!/usr/bin/env python3
"""route_builder.py -- the route builder on this laptop, with Send to robot (RMP1-I).

    python3 tools/route_builder.py             open it in the browser (python on Windows; Ctrl+C stops it)
    python3 tools/route_builder.py self-test

Serves THIS checkout's route builder (tools/route_maker/src, put together by tools/route_maker/build.py's own code) on
127.0.0.1, at the same port every start (the browser keeps the page's draft per port). The page reads and writes
src/team/ through it (any browser, no folder dialog), and Send to robot runs tools/upload_auton.py, its lines streamed
into the page as they come. The robots' facts and PREVIEW's engine: this laptop's build/route_maker_site, else the
published site's, kept in build/route_builder_cache/ for a venue with no internet. 127.0.0.1 only; the token in every
path (and in a header for a write or a send); the Host header checked; nothing served but the page's files; writes only
to files under src/team/, all or nothing, and none while a send runs. Python's standard library only.
"""
import hmac
import html
import http.client
import importlib.util
import json
import os
import secrets
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
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


def stop(p):
    """End a send: upload_auton, make and the compilers (its whole process group), as a terminal's Ctrl+C would. `p` may
    have ended already (poll() says None while another thread waits on it): never an error."""
    if p is None or p.poll() is not None:
        return
    if os.name == "nt":
        subprocess.run(["taskkill", "/T", "/F", "/PID", str(p.pid)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    else:
        for sig, wait in ((signal.SIGINT, 10), (signal.SIGKILL, 0)):
            try:
                os.killpg(p.pid, sig)
                p.wait(wait) if wait else None
            except (ProcessLookupError, PermissionError, subprocess.TimeoutExpired):
                pass
    try:
        p.wait(10)
    except subprocess.TimeoutExpired:
        pass


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
            say(f"Sending to robot {robot}...")
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
    if argv:
        print("Say nothing to open it: python3 tools/route_builder.py   (or self-test)")
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
            "APPDATA": home, "XDG_CONFIG_HOME": home, "HOME": home}}


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
        want = [f"Building robot one's Auton Tester", "✓ BUILT", f"Uploading to slot {ua.SLOT} on /dev/ttyACM0",
                f"✓ UPLOADED to slot {ua.SLOT}. On the brain: shulib Auton Tester → skills or left 1 → RUN"]
        at = [text.find(w) for w in want]
        check("Send: the lines in order", st == 200 and code == 0 and -1 not in at and at == sorted(at), text)
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

    tasks = json.loads("".join(x for x in (TOOLS.parent / ".vscode/tasks.json").read_text(encoding="utf-8").splitlines(True)
                               if not x.lstrip().startswith("//")))["tasks"][0]
    check("VS Code's first task opens it, by the name the docs and the page use", tasks["label"] == "Open the route builder"
          and tasks["args"] == ["tools/route_builder.py"] and tasks["isBackground"] and tasks["windows"]["command"] == "python", tasks)

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
