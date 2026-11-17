#!/usr/bin/env python3
"""Builds the route builder (chunk RMP1-A1, 2026-11-14; the RMP1 brief A.3.3, A.3.5, A.3.9, W1, W3, W10).

    python3 tools/route_maker/build.py --out build/route_maker_site --describe build/test/shulib_route_maker

writes, into --out:

  app.html                    THE SITE FORM: the page, its modules as files under src/ (ES modules), its stylesheet
                              and its data inline. NEVER index.html: the docs page docs/route-builder.md renders to
                              route-builder/index.html and `mkdocs build --strict` would silently overwrite it (W3).
  src/**                      the modules app.html loads (its Content-Security-Policy allows 'self' scripts).
  robots.json                 the robots' descriptions baked into both forms (also here as a file).
  shulib-route-builder.html   THE ONE FILE: everything inside (the modules bundled into one script, the field
                              picture as a data: URI) -- what "Download for offline use" hands a team laptop;
                              double-click it, no network.

The robots' descriptions come from the C++ (`shulib_route_maker describe --robot one|two`, built from the SAME
commit as the library: the published page knows the robots as of its build -- landmine 18) through --describe,
or from a file through --robots. Each form gets its OWN exact Content-Security-Policy (A.3.9, W1); both carry
`<link rel="icon" href="data:,">` and `<meta name="shulib-app" content="route-builder">`. The build REFUSES (exit 1)
an output that names a URL other than the XML namespaces, or that carries a laptop's or a runner's path
(RG1: /home/, /Users/, C:\\, /runner/, .claude -- checked in every file AND inside every base64 data: URI, which
a plain grep cannot see into).

Python's standard library only. Nothing it writes is committed (A.3.3): CI builds it on every push and pages.yml
on every publish.
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
SRC = HERE / 'src'
ENTRY = 'ui/app.mjs'

# A.3.9 as amended by W1: one exact policy per form.
CSP_ONE_FILE = ("default-src 'none'; script-src 'unsafe-inline' 'wasm-unsafe-eval'; style-src 'unsafe-inline'; "
                "img-src data: blob:; worker-src blob:; connect-src 'self'")
CSP_SITE = ("default-src 'none'; script-src 'self' 'unsafe-inline' 'wasm-unsafe-eval'; style-src 'unsafe-inline'; "
            "img-src 'self' data: blob:; worker-src 'self' blob:; connect-src 'self'")

SITE_PAGE = 'app.html'
ONE_FILE = 'shulib-route-builder.html'
ALLOWED_URLS = re.compile(r'https?://www\.w3\.org/(2000/svg|1999/xlink|1999/xhtml)')
ANY_URL = re.compile(r'https?://[^\s"\'<>)]*')
RG1 = [b'/home/', b'/Users/', b'C:\\', b'/runner/', b'.claude']
DATA_URI = re.compile(rb'data:[a-zA-Z0-9.+/-]*;base64,([A-Za-z0-9+/=]+)')


class BuildError(Exception):
    """A refusal, said in words."""


def fail(words: str) -> None:
    raise BuildError(words)


# ── the module graph and the one-file bundle ────────────────────────────────────────────────────────────
IMPORT = re.compile(r"^import\s*\{([^}]*)\}\s*from\s*'([^']+)';[ \t]*$", re.M)
EXPORT = re.compile(r'^export\s+(async\s+function|function|class|const|let)\s+([A-Za-z_$][\w$]*)', re.M)


def module_graph(entry: str) -> list[str]:
    """Every module from `entry`, dependencies first (paths relative to src/)."""
    order: list[str] = []
    state: dict[str, str] = {}

    def visit(rel: str) -> None:
        if state.get(rel) == 'done':
            return
        if state.get(rel) == 'busy':
            fail(f'build: a circular import reaches {rel}')
        state[rel] = 'busy'
        text = (SRC / rel).read_text(encoding='utf-8')
        if re.search(r'^\s*import\s*\*|^\s*export\s+default|^\s*export\s*\{|\bimport\s*\(', text, re.M):
            fail(f'build: {rel} uses an import or export form the bundler does not take (only `import {{...}} from` and `export <declaration>`)')
        for _, spec in IMPORT.findall(text):
            visit(os.path.normpath(os.path.join(os.path.dirname(rel), spec)).replace(os.sep, '/'))
        state[rel] = 'done'
        order.append(rel)

    visit(entry)
    return order


def bundle(entry: str) -> str:
    """The modules as ONE script: each in its own function scope, its exports an object, imports read from it."""
    out = ["'use strict';", 'const __shulib = Object.create(null);']
    for rel in module_graph(entry):
        text = (SRC / rel).read_text(encoding='utf-8')
        names = [m.group(2) for m in EXPORT.finditer(text)]

        def imp(m: re.Match) -> str:
            spec = os.path.normpath(os.path.join(os.path.dirname(rel), m.group(2))).replace(os.sep, '/')
            parts = []
            for item in m.group(1).split(','):
                item = item.strip()
                if not item:
                    continue
                if ' as ' in item:
                    a, b = [x.strip() for x in item.split(' as ')]
                    parts.append(f'{a}: {b}')
                else:
                    parts.append(item)
            return f"const {{ {', '.join(parts)} }} = __shulib[{json.dumps(spec)}];"

        body = IMPORT.sub(imp, text)
        body = re.sub(r'^export\s+', '', body, flags=re.M)
        out.append(f'__shulib[{json.dumps(rel)}] = (() => {{\n{body}\nreturn {{ {", ".join(names)} }};\n}})();')
    script = '\n'.join(out)
    return script.replace('</script', '<\\/script')


# ── the robots and the build stamp ──────────────────────────────────────────────────────────────────────
def git(*args: str) -> str:
    try:
        return subprocess.run(['git', *args], cwd=REPO, capture_output=True, text=True, check=True, timeout=30).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ''


def robots_from(describe: str | None, robots_file: str | None) -> dict:
    if robots_file:
        data = json.loads(Path(robots_file).read_text(encoding='utf-8'))
        return {'one': data['one'], 'two': data['two']}
    if not describe:
        fail('build: say where the robots come from: --describe <shulib_route_maker> (CI) or --robots <file>')
    out = {}
    for rid in ('one', 'two'):
        res = subprocess.run([describe, 'describe', '--robot', rid], capture_output=True, text=True, timeout=120)
        if res.returncode != 0:
            fail(f'build: `{describe} describe --robot {rid}` failed: {res.stderr.strip()}')
        out[rid] = json.loads(res.stdout)
    return out


def shared_route_present() -> bool:
    """§6.3: robot two's update puts the shared skills-route file in src/team/ with its one marked region."""
    f = REPO / 'src' / 'team' / 'skills_route.cpp'
    if not f.exists():
        return False
    text = f.read_text(encoding='utf-8')
    return bool(re.search(r'^// ═══ ROUTE MAKER — route "[^"]*" — BEGIN ═+\r?$', text, re.M))


def driver_files() -> dict:
    """RMP1-B: each robot's driver file as it is in this commit -- the Controls tab reads its map from it. Base64:
    it is the TEAM's text, so a URL or a path in one of their comments never trips HS8 or RG1 (they judge the
    builder's own output) and never stops the site's build."""
    out = {}
    for rid in ('one', 'two'):
        f = REPO / 'src' / 'team' / f'robot_{rid}_driver.hpp'
        if f.exists():
            text = f.read_text(encoding='utf-8').replace('\r\n', '\n')
            out[rid] = {'path': f'src/team/robot_{rid}_driver.hpp', 'base64': base64.b64encode(text.encode('utf-8')).decode('ascii')}
    return out


def baked_data(robots: dict) -> dict:
    api = robots['one'].get('api', '?')
    present = shared_route_present()
    return {
        'build': {'commit': git('rev-parse', '--short=10', 'HEAD') or 'unknown', 'date': git('log', '-1', '--format=%cs') or 'unknown',
                  'api': api},
        'robots': robots,
        'sharedRoute': {'file': 'src/team/skills_route.cpp', 'present': present, 'robots': ['one', 'two'] if present else ['one']},
        'driverFiles': driver_files(),
        # the drive program's boot line still names the DEFAULT feel (a static_assert): the page refuses any other
        'feelDefaultOnly': 'driverLineMatchesFeel' in (REPO / 'src' / 'programs' / 'drive_program.cpp').read_text(encoding='utf-8'),
    }


def json_in_html(data) -> str:
    return json.dumps(data, separators=(',', ':'), ensure_ascii=True).replace('</', '<\\/')


# ── the page ─────────────────────────────────────────────────────────────────────────────────────────────
def page(csp: str, app_script: str, baked: dict) -> str:
    field = json.loads((HERE / 'assets' / 'field.json').read_text(encoding='utf-8'))
    field['url'] = 'data:image/webp;base64,' + base64.b64encode((HERE / 'assets' / 'field.webp').read_bytes()).decode('ascii')
    tpl = (SRC / 'app.html').read_text(encoding='utf-8')
    fills = {
        '{{CSP}}': csp,
        '{{BUILD}}': f"{baked['build']['commit']} {baked['build']['date']} API {baked['build']['api']}",
        '{{STYLE}}': (SRC / 'style.css').read_text(encoding='utf-8'),
        '{{BROWSER_CHECK}}': (SRC / 'browser_check.js').read_text(encoding='utf-8'),
        '{{ROBOTS_JSON}}': json_in_html(baked),
        '{{FIELD_JSON}}': json_in_html(field),
        '{{APP_SCRIPT}}': app_script,
    }
    for k, v in fills.items():
        if k not in tpl:
            fail(f'build: the template has no {k}')
        tpl = tpl.replace(k, v)
    return tpl


def check_output(out: Path) -> None:
    """No URL but the XML namespaces; no laptop or runner path anywhere, decoded data: URIs included (RG1)."""
    for f in sorted(out.rglob('*')):
        if not f.is_file():
            continue
        raw = f.read_bytes()
        text = raw.decode('utf-8', errors='replace')
        for m in ANY_URL.finditer(text):
            if not ALLOWED_URLS.match(m.group(0)):
                fail(f'build: {f.relative_to(out)} names a URL, {m.group(0)!r}: the page requests nothing (HS8)')
        blobs = [raw] + [base64.b64decode(m.group(1)) for m in DATA_URI.finditer(raw)]
        for blob in blobs:
            for bad in RG1:
                if bad in blob:
                    fail(f'build: {f.relative_to(out)} carries {bad.decode()!r} -- a laptop\'s or a runner\'s path (RG1)')


def build(out: Path, robots: dict) -> None:
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    baked = baked_data(robots)
    # THE SITE FORM: app.html + the module files
    for rel in module_graph(ENTRY):
        dst = out / 'src' / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(SRC / rel, dst)
    (out / SITE_PAGE).write_text(page(CSP_SITE, f'<script type="module" src="src/{ENTRY}"></script>', baked), encoding='utf-8', newline='\n')
    (out / 'robots.json').write_text(json.dumps(baked, indent=2) + '\n', encoding='utf-8', newline='\n')
    # THE ONE FILE
    (out / ONE_FILE).write_text(page(CSP_ONE_FILE, f'<script type="module">\n{bundle(ENTRY)}\n</script>', baked),
                                encoding='utf-8', newline='\n')
    if (out / 'index.html').exists():
        fail('build: never index.html (W3)')
    check_output(out)


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--out', required=True, help='the folder to write (replaced)')
    ap.add_argument('--describe', help='the shulib_route_maker program (it writes robots one and two)')
    ap.add_argument('--robots', help='a JSON file {"one": ..., "two": ...} instead of --describe')
    a = ap.parse_args(argv)
    try:
        build(Path(a.out), robots_from(a.describe, a.robots))
    except BuildError as e:
        print(e, file=sys.stderr)
        return 1
    out = Path(a.out)
    print(f'route builder built into {out}: {SITE_PAGE} ({(out / SITE_PAGE).stat().st_size} bytes + src/), '
          f'{ONE_FILE} ({(out / ONE_FILE).stat().st_size} bytes)')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
