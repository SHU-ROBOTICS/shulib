#!/usr/bin/env python3
"""Reading checks for the public docs (chunk DOCS3).

  check-nav   every public page is in mkdocs.yml's menu exactly once, and every
              menu entry names a page that exists
  check-tone  no public page says "honest" (or honesty, honestly), and no page
              opens with a "this has not run on a robot yet" status line
  self-test   each check, shown failing on a planted case

Standard library only, like every doc tool here. The menu is read line by line;
a line this tool does not understand is refused by name, never guessed at.
"""

from __future__ import annotations

import glob
import os
import re
import sys
from collections import Counter

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOCS = os.path.join(REPO, "docs")
MKDOCS = os.path.join(REPO, "mkdocs.yml")

# Pages the site never publishes (prepare_site.py drops them).
UNPUBLISHED = ("internal",)

# Public files outside docs/ that check-tone also reads.
TONE_EXTRA = ["README.md", "test/README.md"]

# Exempt from check-tone, with the reason. docs/api/ is generated from the
# header comments under include/, which are frozen until after the competition;
# a header-comment pass removes the word there, and then this exemption goes.
TONE_EXEMPT = {"api": "generated from include/ comments (frozen); fixed at the source"}


class NavError(Exception):
    """A menu line this tool cannot read."""


# ── check-nav ─────────────────────────────────────────────────────────────────

ITEM = re.compile(r"^(?P<indent> +)- (?P<rest>\S.*)$")
TARGET = re.compile(r"^[A-Za-z0-9_][A-Za-z0-9_./-]*\.md$")
URL = re.compile(r"^https?://\S+$")


def _strip_comment(rest: str) -> str:
    if rest[:1] in "\"'":
        end = rest.find(rest[0], 1)
        if end < 0:
            return rest
        return rest[: end + 1] + re.sub(r"\s+#.*$", "", rest[end + 1:])
    return re.sub(r"\s+#.*$", "", rest)


def _split_entry(rest: str, where: str) -> tuple[str, str | None]:
    """'Title: path' -> (Title, path); 'Title:' -> (Title, None); 'path' -> ('', path)."""
    if rest[:1] in "\"'":
        end = rest.find(rest[0], 1)
        if end < 0 or rest[end + 1: end + 2] != ":":
            raise NavError(f"{where}: cannot read this quoted title: {rest!r}")
        title, value = rest[1:end], rest[end + 2:].strip()
    elif rest.endswith(":"):
        title, value = rest[:-1], ""
    elif ": " in rest:
        title, value = rest.split(": ", 1)
        value = value.strip()
    else:
        title, value = "", rest
    if title and (title[0] in "{[!&*|>%@`" or title.strip() != title):
        raise NavError(f"{where}: cannot read this title: {title!r}")
    if not value:
        if not title:
            raise NavError(f"{where}: empty menu line")
        return title, None
    if not (TARGET.match(value) or URL.match(value)):
        raise NavError(f"{where}: not a page path or a web link: {value!r}")
    return title, value


def parse_nav(text: str, name: str = "mkdocs.yml") -> list[tuple[int, str]]:
    """Every page path in the nav block, with its line number. Raises NavError."""
    lines = text.split("\n")
    starts = [i for i, line in enumerate(lines) if line.rstrip() == "nav:"]
    if len(starts) != 1:
        raise NavError(f"{name}: expected one 'nav:' line, found {len(starts)}")
    entries: list[tuple[int, str]] = []
    section: tuple[int, int] | None = None  # (line, indent) of a 'Title:' awaiting pages
    for i in range(starts[0] + 1, len(lines)):
        raw, n = lines[i], i + 1
        stripped = raw.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if not raw.startswith(" "):
            if re.match(r"^[A-Za-z_][A-Za-z0-9_]*:", raw):
                break  # the next top-level key: the menu is over
            raise NavError(f"{name}:{n}: a menu line must be indented: {raw!r}")
        m = ITEM.match(raw)
        if not m:
            raise NavError(f"{name}:{n}: not a menu line: {raw!r}")
        indent = len(m.group("indent"))
        if section and indent <= section[1]:
            raise NavError(f"{name}:{section[0]}: this section has no pages")
        section = None
        _title, target = _split_entry(_strip_comment(m.group("rest")), f"{name}:{n}")
        if target is None:
            section = (n, indent)
        elif not URL.match(target):
            entries.append((n, target))
    if section:
        raise NavError(f"{name}:{section[0]}: this section has no pages")
    return entries


def site_path(rel: str) -> str:
    """A docs/ path as the site names it: the root README becomes index.md."""
    return "index.md" if rel == "README.md" else rel


def public_pages(docs: str = DOCS) -> set[str]:
    out = set()
    for path in glob.glob(os.path.join(docs, "**", "*.md"), recursive=True):
        rel = os.path.relpath(path, docs).replace(os.sep, "/")
        if rel.split("/")[0] not in UNPUBLISHED:
            out.add(site_path(rel))
    return out


def nav_findings(entries: list[tuple[int, str]], pages: set[str]) -> list[str]:
    found = []
    counts = Counter(target for _n, target in entries)
    for n, target in entries:
        if target == "README.md":
            found.append(f"mkdocs.yml:{n}: README.md is index.md on the site — name it index.md")
        elif target.split("/")[0] in UNPUBLISHED or ".." in target.split("/"):
            found.append(f"mkdocs.yml:{n}: {target} is never published")
        elif target not in pages:
            found.append(f"mkdocs.yml:{n}: {target} names no page")
    for target, count in sorted(counts.items()):
        if count > 1:
            found.append(f"mkdocs.yml: {target} is in the menu {count} times")
    for page in sorted(pages - set(counts)):
        found.append(f"docs/{'README.md' if page == 'index.md' else page}: not in the menu")
    return found


def check_nav(mkdocs: str = MKDOCS, docs: str = DOCS) -> int:
    try:
        entries = parse_nav(open(mkdocs, encoding="utf-8").read())
    except NavError as e:
        print(f"check-nav: FAIL — {e}", file=sys.stderr)
        return 1
    pages = public_pages(docs)
    found = nav_findings(entries, pages)
    for f in found:
        print(f"check-nav: {f}", file=sys.stderr)
    if found:
        print(f"check-nav: FAIL — {len(found)} problem(s). Every public page goes in the menu "
              "exactly once (mkdocs.yml, nav:).", file=sys.stderr)
        return 1
    print(f"check-nav: OK — {len(pages)} public pages, each in the menu once")
    return 0


# ── check-tone ────────────────────────────────────────────────────────────────

# Letters around the word, not \b: '_' is a word character, so \b would miss "_honest_".
HONEST = re.compile(r"(?<![A-Za-z])honest(?:y|ly)?(?![A-Za-z])", re.I)
# The ways a page's opening says "this has not run". Each shape has self-test sentences, one per
# word it lists, that no other shape matches — so dropping any word fails the self-test.
STATUS_SHAPES = {
    "has not run": r"\bha(?:s|ve)(?: not| never|n['’]t) (?:yet )?(?:been )?(?:run|driven|tested|used|tried)\b",
    "never driven": r"\bnever driven\b",
    "nothing has run": r"\b(?:nothing|none|neither|nobody|no(?: [\w-]+){1,3}?)(?: (?:of|in|on) [^.!?]{1,40}?| here)?"
                       r" ha(?:s|ve) (?:yet |ever )?(?:been )?(?:run|driven|tested|used|tried)\b",
    "yet to run": r"\bha(?:s|ve) yet to (?:run|be|drive)\b",
    "not run on a robot": r"\bnot (?:yet )?(?:been )?(?:run|tried|tested|used|driven) on (?:a |the )?"
                          r"(?:robot|brain|hardware)\b",
    "untested": r"\buntested\b",
}
STATUS = re.compile("|".join(f"(?:{p})" for p in STATUS_SHAPES.values()), re.I)
FENCE = re.compile(r"^\s*(```|~~~)")


def opening(lines: list[str]) -> list[str]:
    """The lines before the page's first H2 (a '## ' inside a code block does not count)."""
    fenced = False
    for i, line in enumerate(lines):
        if FENCE.match(line):
            fenced = not fenced
        elif not fenced and line.startswith("## "):
            return lines[:i]
    return lines


def tone_findings(text: str, name: str) -> list[str]:
    lines = text.split("\n")
    found = []
    for n, line in enumerate(lines, 1):
        for m in HONEST.finditer(line):
            found.append(f"{name}:{n}: the word \"{m.group(0)}\" — say the fact plainly instead")
    # The opening, joined into one string so a sentence split over lines is still one sentence.
    joined, starts = "", []
    for n, line in enumerate(opening(lines), 1):
        starts.append((len(joined), n))
        joined += re.sub(r"^\s*>\s?", "", line).replace("*", "").replace("_", "") + " "
    for m in STATUS.finditer(joined):
        n = max(line_no for off, line_no in starts if off <= m.start())
        found.append(f"{name}:{n}: a status line in the page's opening (\"{m.group(0)}\") — a limit "
                     "goes where it applies, or in \"What it cannot do yet\"")
    return found


def tone_pages(docs: str = DOCS, repo: str = REPO, extra: list[str] = TONE_EXTRA) -> list[str]:
    out = []
    for path in sorted(glob.glob(os.path.join(docs, "**", "*.md"), recursive=True)):
        top = os.path.relpath(path, docs).replace(os.sep, "/").split("/")[0]
        if top not in UNPUBLISHED and top not in TONE_EXEMPT:
            out.append(path)
    return out + [os.path.join(repo, f) for f in extra if os.path.exists(os.path.join(repo, f))]


def check_tone(docs: str = DOCS, repo: str = REPO, extra: list[str] = TONE_EXTRA) -> int:
    found = []
    pages = tone_pages(docs, repo, extra)
    for path in pages:
        found += tone_findings(open(path, encoding="utf-8").read(), os.path.relpath(path, repo))
    for f in found:
        print(f"check-tone: {f}", file=sys.stderr)
    if found:
        print(f"check-tone: FAIL — {len(found)} finding(s)", file=sys.stderr)
        return 1
    print(f"check-tone: OK — {len(pages)} pages (docs/api/ exempt: {TONE_EXEMPT['api']})")
    return 0


# ── the lists other tools keep ────────────────────────────────────────────────

def glob_dirs(globs: list[str]) -> set[str]:
    return {os.path.dirname(g) for g in globs}


def list_findings(doc_globs: list[str], audit_globs: list[str], page_dirs: set[str]) -> list[str]:
    """check-examples' folders must all be in the staleness audit's, and every folder
    that holds a public page must be in check-examples' (the hole a new folder falls into)."""
    found = [f"{d}/ is checked by check-examples but not by the staleness audit's PATH_GLOBS"
             for d in sorted(glob_dirs(doc_globs) - glob_dirs(audit_globs))]
    found += [f"{d}/ holds public pages but is not in api_doc_tool.py's PUBLIC_DOC_GLOBS"
              for d in sorted(page_dirs - glob_dirs(doc_globs))]
    return found


def real_lists() -> tuple[list[str], list[str], set[str]]:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import api_doc_tool  # noqa: E402
    import doc_staleness_audit  # noqa: E402
    page_dirs = {os.path.dirname("docs/" + p) if p != "index.md" else "docs" for p in public_pages()}
    return api_doc_tool.PUBLIC_DOC_GLOBS, doc_staleness_audit.PATH_GLOBS, page_dirs


# ── self-test ─────────────────────────────────────────────────────────────────

GOOD_NAV = """\
site_name: x
nav:
  - Home: index.md
  # a comment
  - Robot:
      - guide/README.md  # a trailing comment
      - "Set up: the first week": guide/07-set-up.md   # a trailing comment
      - Web: https://example.com/
# BEGIN GENERATED API NAV
  - API reference:
      - api/README.md
# END GENERATED API NAV
"""
GOOD_PAGES = {"index.md", "guide/README.md", "guide/07-set-up.md", "api/README.md"}

# One sentence per word of each STATUS shape (the comment names the word it holds).
WITNESSES = {
    "has not run": [
        "It has not run.",                    # has, not, run
        "We have not run it.",                # have
        "It has never run.",                  # never
        "It hasn't run.",                     # n't
        "It hasn’t run.",                     # n’t
        "It has not yet run.",                # yet
        "It has not been run.",               # been
        "This code hasn't driven a robot.",   # driven
        "We have not tested it.",             # tested
        "The team has not used it.",          # used
        "The team has not tried it.",         # tried
    ],
    "never driven": ["A robot never driven by this code."],
    "nothing has run": [
        "Nothing has run.",                       # nothing, has, run
        "None of them has run.",                  # none, of
        "Neither has run.",                       # neither
        "Nobody has tried it.",                   # nobody, tried
        "No robot has run this code.",            # no + one word
        "No AI Vision sensor has run this code.",  # no + three words
        "Nothing in this recipe has run.",        # in
        "Nothing on this page has run.",          # on
        "Nothing here has run.",                  # here
        "No robots have run it.",                 # have
        "Nothing has yet run.",                   # yet
        "Nothing has ever run.",                  # ever
        "Nothing here has been tested.",          # been, tested
        "No driver has driven it.",               # driven
        "No driver has used either.",             # used
    ],
    "yet to run": [
        "This page has yet to run on a robot.",   # has, run
        "They have yet to drive it.",             # have, drive
        "It has yet to be tested.",               # be
    ],
    "not run on a robot": [
        "This page: not yet run on a robot.",     # yet, run, a, robot
        "It was not tested on the brain.",        # tested, the, brain
        "It was not used on a robot.",            # used
        "Not been tried on a robot.",             # been, tried
        "It was not driven on hardware.",         # driven, hardware
    ],
    "untested": ["It is untested on hardware."],
}

# A scratch repository for the commands themselves: which files they read, and their exit codes.
TREE = {
    "docs/README.md": "# Home\n",
    "docs/faq.md": "# FAQ\n",
    "docs/guide/02-field.md": "# Two\n",
    "docs/api/a.md": "# A\n",
    "docs/internal/notes.md": "# Notes\n",
    "README.md": "# Repo\n",
    "test/README.md": "# Tests\n",
    "mkdocs.yml": "nav:\n  - Home: index.md\n  - FAQ: faq.md\n  - Learn:\n      - guide/02-field.md\n"
                  "  - API: api/a.md\n",
}


def _command_cases(case) -> None:
    import contextlib
    import io
    import tempfile

    with tempfile.TemporaryDirectory(prefix="shulib-readability-") as root:
        def write(rel, text):
            os.makedirs(os.path.dirname(os.path.join(root, rel)), exist_ok=True)
            with open(os.path.join(root, rel), "w", encoding="utf-8") as f:
                f.write(text)

        def run(command, **kw):
            out = io.StringIO()
            with contextlib.redirect_stdout(out), contextlib.redirect_stderr(out):
                code = command(**kw)
            return code, out.getvalue()

        def tone():
            return run(check_tone, docs=os.path.join(root, "docs"), repo=root, extra=["README.md", "test/README.md"])

        def nav():
            return run(check_nav, mkdocs=os.path.join(root, "mkdocs.yml"), docs=os.path.join(root, "docs"))

        for rel, text in TREE.items():
            write(rel, text)
        code, out = tone()
        case("tone command: a clean tree passes, reading its 5 pages", code == 0 and "OK — 5 pages" in out)
        for rel, planted in ((r, r not in ("docs/api/a.md", "docs/internal/notes.md")) for r in TREE if r.endswith(".md")):
            write(rel, TREE[rel] + "\nTo be honest, a plant.\n")
            code, out = tone()
            if planted:
                case(f"tone command: a plant in {rel} fails and names it", code == 1 and f"{rel}:3:" in out)
            else:
                case(f"tone command: a plant in {rel} is not read", code == 0)
            write(rel, TREE[rel])
        code, out = nav()
        case("nav command: a clean tree passes", code == 0 and "OK — 4 public pages" in out)
        for rel in ("docs/new.md", "docs/guide/03-new.md", "docs/api/b.md"):
            write(rel, "# New\n")
            code, out = nav()
            case(f"nav command: {rel} in no menu fails and is named", code == 1 and f"{rel}: not in the menu" in out)
            os.remove(os.path.join(root, rel))
        write("docs/internal/more.md", "# More\n")
        case("nav command: an internal page needs no menu line", nav()[0] == 0)
        write("mkdocs.yml", TREE["mkdocs.yml"] + "  - {Bad: x.md}\n")
        case("nav command: a menu it cannot read fails", nav()[0] == 1)


def self_test(plant: bool = False) -> int:
    failures, ran = [], []

    def case(name, ok):
        ran.append(name)
        if not ok:
            failures.append(name)

    if plant:
        case("a planted failing case", False)

    def refused(nav):
        try:
            parse_nav(nav)
        except NavError:
            return True
        return False

    def parsed(nav):
        try:
            return parse_nav(nav)
        except NavError:
            return None

    def parsed_or_empty(nav):
        return parsed(nav) or []

    # check-nav
    entries = parsed(GOOD_NAV) or []
    case("nav: a good menu passes", entries != [] and nav_findings(entries, GOOD_PAGES) == [])
    case("nav: a trailing comment and a quoted title are read",
         [t for _n, t in entries] == ["index.md", "guide/README.md", "guide/07-set-up.md", "api/README.md"])
    case("nav: a page missing from the menu fails",
         any("guide/08.md: not in the menu" in f for f in nav_findings(entries, GOOD_PAGES | {"guide/08.md"})))
    twice = GOOD_NAV.replace("      - api/README.md", "      - api/README.md\n      - Again: api/README.md")
    case("nav: a page listed twice fails",
         any("2 times" in f for f in nav_findings(parsed_or_empty(twice), GOOD_PAGES)))
    gone = GOOD_NAV.replace("guide/07-set-up.md", "guide/07-gone.md")
    case("nav: an entry naming no page fails",
         any("guide/07-gone.md names no page" in f for f in nav_findings(parsed_or_empty(gone), GOOD_PAGES)))
    case("nav: the root README must be index.md",
         any("index.md on the site" in f
             for f in nav_findings(parsed_or_empty(GOOD_NAV.replace("Home: index.md", "Home: README.md")), GOOD_PAGES)))
    case("nav: an internal page is never published",
         any("never published" in f
             for f in nav_findings(parsed_or_empty(GOOD_NAV.replace("guide/07-set-up.md", "internal/x.md")), GOOD_PAGES)))
    case("nav: a flow-style line is refused", refused(GOOD_NAV.replace("- Home: index.md", "- {Home: index.md}")))
    case("nav: a tagged value is refused", refused(GOOD_NAV.replace("Home: index.md", "Home: !include index.md")))
    case("nav: a title with a YAML anchor or tag is refused",
         refused(GOOD_NAV.replace("- Home: index.md", "- &a Home: index.md")))
    case("nav: a non-page target is refused", refused(GOOD_NAV.replace("Home: index.md", "Home: app.html")))
    case("nav: an unquoted title holding ': ' is refused",
         refused(GOOD_NAV.replace("Home: index.md", "Home: the start: index.md")))
    case("nav: a section with no pages is refused",
         refused(GOOD_NAV.replace("  # a comment\n", "  - Empty:\n")))
    case("nav: an unindented item is refused", refused(GOOD_NAV.replace("  - Home: index.md", "- Home: index.md")))
    case("nav: a line that is not an item is refused", refused(GOOD_NAV.replace("  # a comment", "  Robot")))
    case("nav: two nav blocks are refused", refused(GOOD_NAV + "nav:\n  - Home: index.md\n"))
    case("nav: an empty section at the end is refused", refused(GOOD_NAV + "  - Empty:\n"))
    case("nav: the next top-level key ends the menu",
         parsed(GOOD_NAV + "theme:\n  name: material\n  features:\n    - navigation.tabs\n") == entries)
    case("nav: a title with a space before its ':' is refused",
         refused(GOOD_NAV.replace("Home: index.md", "Home : index.md")))
    case("nav: a path through '..' is never published",
         any("guide/../index.md is never published" in f
             for f in nav_findings(parsed_or_empty(GOOD_NAV.replace("Home: index.md", "Home: guide/../index.md")),
                                   GOOD_PAGES)))
    for c in "{[!&*|>%@`":  # characters YAML reserves at the start of a plain title
        case(f"nav: a title starting with {c!r} is refused",
             refused(GOOD_NAV.replace("- Home: index.md", f"- {c}Home: index.md")))
    case("nav: a quoted title not followed by ':' is refused",
         refused(GOOD_NAV.replace("- Home: index.md", '- "Home" index.md')))
    case("nav: a quoted title holding ' #' is read, not cut as a comment",
         parsed(GOOD_NAV.replace("- Home: index.md", '- "Step #1: home": index.md')) == entries)

    # check-tone
    for word in ("honest", "Honesty", "HONESTLY"):
        case(f"tone: '{word}' fails", len(tone_findings(f"# T\n\n## A\n\nIt is {word}.\n", "p")) == 1)
    case("tone: every use on a line is named", len(tone_findings("# T\n\nhonest, honestly.\n", "p")) == 2)
    case("tone: the word inside a code block fails",
         len(tone_findings("# T\n\n## A\n\n```cpp\n// honestly\n```\n", "p")) == 1)
    case("tone: 'honestly' in inline code fails", len(tone_findings("# T\n\n## A\n\n`honestly`\n", "p")) == 1)
    case("tone: a clean page passes", tone_findings("# T\n\nPlain words.\n\n## A\n\nMore.\n", "p") == [])
    preamble = "# T\n\n**Nothing in this chapter has run on a robot yet.**\n\n## A\n\nText.\n"
    case("tone: a status line in the opening fails", len(tone_findings(preamble, "p")) == 1)
    case("tone: a status line split over two lines fails",
         len(tone_findings("# T\n\n> The program has\n> not been run on a robot.\n\n## A\n", "p")) == 1)
    case("tone: 'never driven' alone fails",
         len(tone_findings("# T\n\nA robot never driven by this code.\n\n## A\n", "p")) == 1)
    case("tone: 'not yet run on a robot' alone fails",
         len(tone_findings("# T\n\nThis page: not yet run on a robot.\n\n## A\n", "p")) == 1)
    case("tone: emphasis inside the status line does not hide it",
         len(tone_findings("# T\n\nIt has **never** driven a robot.\n\n## A\n", "p")) == 1)
    case("tone: the same line after the first H2 is not an opening",
         tone_findings("# T\n\nIntro.\n\n## A\n\nIt has never driven a robot.\n", "p") == [])
    case("tone: a '## ' inside a code block does not end the opening",
         len(tone_findings("# T\n\n```sh\n## not a heading\n```\n\nIt has not run.\n\n## A\n", "p")) == 1)
    case("tone: the finding names its line",
         tone_findings("# T\n\nx\n\nhas never driven\n\n## A\n", "p")[0].startswith("p:5:"))
    # Every word of every shape: a sentence that its own shape matches and no other shape does.
    for shape, sentences in WITNESSES.items():
        for s in sentences:
            hits = {k for k, p in STATUS_SHAPES.items() if re.search(p, s, re.I)}
            case(f"tone: {s!r} is shape '{shape}' alone", hits == {shape})
            case(f"tone: {s!r} in the opening fails", len(tone_findings(f"# T\n\n{s}\n\n## A\n", "p")) == 1)
    case("tone: every shape has sentences", set(WITNESSES) == set(STATUS_SHAPES))
    # Real openings the check once missed, and the forms around them.
    for s in ("**Nothing in this program has run on a robot in this form.**",  # guide 19, 2026-11-15
              "on the host test plant. **No driver has used either on a robot yet.**",  # cookbook 06
              "none of them exists yet. **No AI Vision sensor has run this code.**",  # guide 20
              "(\"shulib Competition\", since chunk COMP-b); neither has run on a\nrobot yet.",  # src/team
              "It has _never_ driven.", "It is __untested__."):
        case(f"tone: {s!r} in the opening fails", len(tone_findings(f"# T\n\n{s}\n\n## A\n", "p")) == 1)
    for s in ("Built 2026-11-03, never run on a robot; no capture has seen it.",  # a dated record
              "Do not run Calibrate on a robot with its wheels on the ground.", "No motion can run forever.",
              "It has run on robot two since 2026-09-24.", "A dishonest-looking line."):
        case(f"tone: {s!r} is not a status line", tone_findings(f"# T\n\n{s}\n\n## A\n", "p") == [])
    case("tone: a status line split inside a quote, matching only when joined, fails",
         len(tone_findings("# T\n\n> Nothing here\n> has run.\n\n## A\n", "p")) == 1)
    case("tone: a '## ' inside a ~~~ block does not end the opening",
         len(tone_findings("# T\n\n~~~\n## not a heading\n~~~\n\nIt has not run.\n\n## A\n", "p")) == 1)
    case("tone: a '## ' inside an indented fence does not end the opening",
         len(tone_findings("# T\n\n  ```\n## not a heading\n  ```\n\nIt has not run.\n\n## A\n", "p")) == 1)
    case("tone: an H3 does not end the opening",
         len(tone_findings("# T\n\n### Before you start\n\nIt has not run.\n\n## A\n", "p")) == 1)
    case("tone: a page with no H2 is read whole", len(tone_findings("# T\n\nIntro.\n\nIt has not run.\n", "p")) == 1)
    for s in ("An _honest_ answer.", "_Honest caveat:_ x", "__honestly__", "*honesty*", "snake_honest_case"):
        case(f"tone: {s!r} fails", len(tone_findings(f"# T\n\n## A\n\n{s}\n", "p")) == 1)

    _command_cases(case)

    # the lists other tools keep
    case("lists: a folder check-examples reads but the audit does not fails",
         list_findings(["docs/*.md", "docs/start/*.md"], ["docs/*.md"], {"docs"}) != [])
    case("lists: a folder of public pages outside check-examples fails",
         list_findings(["docs/*.md"], ["docs/*.md"], {"docs", "docs/start"}) != [])
    case("lists: matching lists pass", list_findings(["docs/*.md"], ["docs/*.md"], {"docs"}) == [])
    real = list_findings(*real_lists())
    case("lists: the tree's own lists agree (" + "; ".join(real) + ")", real == [])

    for name in failures:
        print(f"self-test: FAIL — {name}", file=sys.stderr)
    if failures:
        return 1
    print(f"self-test: OK — {len(ran)} cases, each fails where it should")
    return 0


def planted_fails(self_test) -> bool:
    """A self-test run with one planted failing case must exit 1 — else its own exit is broken."""
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return self_test(plant=True) == 1


def main(argv: list[str]) -> int:
    commands = {"check-nav": check_nav, "check-tone": check_tone, "self-test": self_test}
    if len(argv) != 2 or argv[1] not in commands:
        print(__doc__, file=sys.stderr)
        return 2
    if argv[1] == "self-test" and not planted_fails(self_test):
        print("self-test: FAIL — a run with a planted failing case exits 0", file=sys.stderr)
        return 1
    return commands[argv[1]]()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
