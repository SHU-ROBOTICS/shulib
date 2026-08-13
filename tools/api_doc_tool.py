#!/usr/bin/env python3
"""shulib API documentation tool — one parser, three jobs (chunk D3).

    generate        write docs/api/ from the headers
    check-coverage  fail, NAMING the member, if a public member has no /// doc
    check-fresh     fail if docs/api/ differs from a fresh generation
    self-test       exercise the parser and the gate against fixtures

WHY THIS EXISTS, AND WHY IT IS NOT DOXYGEN
------------------------------------------
"Generate the reference so it cannot drift" is right, and it is only half of
what people think it buys. A generator faithfully extracts whatever comments
exist; a member added with NO comment is silently absent from the reference,
and the reference still looks complete. That is worse than a stale document,
because nothing looks wrong.

So the coverage gate and the generator must agree, exactly, on what "a public
member" and "documented" mean. One parser guarantees that. Two tools — a
generator plus a separate linter, or Doxygen plus anything — guarantee only
that they will disagree eventually, silently, at the worst moment.

Doxygen was the alternative and was rejected on evidence, not taste: it is a
new toolchain dependency (not installed on the development machine), it has no
native markdown emitter, and its WARN_IF_UNDOCUMENTED is a *different* notion
of documented from whatever an extractor would use. The full ruling is in the
D3 completion record.

THIS IS A TOOL, NOT LIBRARY CODE. The library is header-only C++ and never
depends on it. The host TEST build runs it as a gate, which is deliberate: a
gate you can forget to run is not a gate.

SCOPE, deliberately narrow: it parses the two headers listed in TARGETS, in
this repository's house comment style. It is not a C++ parser and must never
pretend to be one. If it is ever pointed at a header it cannot parse, the
right response is to fail loudly, not to guess.
"""

from __future__ import annotations

import argparse
import difflib
import os
import re
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# The headers whose public surface is documented and gated. Adding one here is
# the whole cost of covering a new frozen surface.
TARGETS = [
    {
        "header": "include/shulib/chassis/chassis.hpp",
        "out": "docs/api/chassis.md",
        "title": "`Chassis` — the frozen facade",
        "blurb": (
            "The public surface every autonomous routine is written against. "
            "Frozen as register row F6 on 2026-08-12: every member below "
            "changes only with a major API-version bump plus a migration note."
        ),
    },
    {
        "header": "include/shulib/chassis/routine.hpp",
        "out": "docs/api/routine.md",
        "title": "`Routine` — the recipe layer",
        "blurb": (
            "The Tier-2 chain: a complete autonomous routine as a sequence of "
            "named steps, each delegating to exactly one `Chassis` verb."
        ),
    },
]

INDEX_OUT = "docs/api/README.md"


# ── the parser ────────────────────────────────────────────────────────────────────

class Member:
    """One public member: how it is spelled, what documents it, where it lives."""

    def __init__(self, kind, name, signature, doc, line):
        self.kind = kind            # "function" | "field" | "enumerator"
        self.name = name
        self.signature = signature  # rendered, whitespace-normalized
        self.doc = doc              # list[str], may be empty  == undocumented
        self.line = line            # 1-based line in the header

    @property
    def documented(self):
        # CAMPAIGN FIND (D3 hole probe H3): `bool(self.doc)` accepted a BARE
        # `///` with no text — the member passed the gate and the reference
        # rendered an empty paragraph. "Nothing looks wrong" is precisely the
        # failure this gate exists to prevent, so it must not be reproducible
        # inside the gate. Documentation must have content.
        return any(line.strip() for line in self.doc)


class TypeDecl:
    def __init__(self, kind, name, doc, line):
        self.kind = kind            # "class" | "struct" | "enum class"
        self.name = name
        self.doc = doc
        self.line = line
        self.members = []


def _strip_doc(line):
    """`/// text` -> `text`; `///` alone -> ``. None if not a doc comment."""
    s = line.strip()
    if not s.startswith("///"):
        return None
    return s[3:].lstrip()


def _trailing_doc(line):
    """`int x = 0;  ///< text` -> `text`, else None."""
    at = line.find("///<")
    if at < 0:
        return None
    return line[at + 4:].strip()


def _normalize(text):
    """Collapse a possibly multi-line declaration into one rendered line."""
    # Drop trailing comments before collapsing, or a `//` would swallow the rest.
    lines = []
    for raw in text.splitlines():
        cut = raw.find("///<")
        if cut < 0:
            cut = raw.find("//")
        lines.append(raw[:cut] if cut >= 0 else raw)
    return re.sub(r"\s+", " ", " ".join(lines)).strip()


def _member_name(sig):
    """The identifier a human would call this member, from its rendered form."""
    depth = 0
    for i, ch in enumerate(sig):
        if ch == "(" and depth == 0:
            head = sig[:i].strip()
            # `operator=` / `operator()` etc. keep the keyword with the symbol.
            m = re.search(r"operator\s*\S+$", head)
            if m:
                return re.sub(r"\s+", "", m.group(0))
            return head.split()[-1].lstrip("*&") if head.split() else "?"
        if ch == "<":
            depth += 1
        elif ch == ">":
            depth = max(0, depth - 1)
    # No parameter list: a field. Name is the identifier before = or { or ;.
    head = re.split(r"[={;]", sig)[0].strip()
    return head.split()[-1].lstrip("*&") if head.split() else "?"


def _is_special_defaulted(sig):
    """`= delete` / `= default` — the run-of-special-members case."""
    return bool(re.search(r"=\s*(delete|default)\s*$", sig))


def parse_header(path):
    """Public types and their public members, in source order.

    Deliberately not a C++ parser: it understands this repository's house
    style (one declaration per statement, /// doc comments directly above or
    ///< trailing) and nothing else.
    """
    text = open(os.path.join(REPO, path), encoding="utf-8").read()
    lines = text.splitlines()

    banner = _leading_banner(lines)
    types = []

    i = 0
    pending = []
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()

        doc = _strip_doc(line)
        if doc is not None:
            pending.append(doc)
            i += 1
            continue
        if not stripped or stripped.startswith("//"):
            pending = []
            i += 1
            continue

        m = re.match(r"^(class|struct|enum class)\s+([A-Za-z_]\w*)\s*(final)?\s*\{",
                     stripped)
        if m and not line.startswith(" "):
            kind, name = m.group(1), m.group(2)
            decl = TypeDecl(kind, name, pending, i + 1)
            pending = []
            i = _parse_body(lines, i, decl)
            types.append(decl)
            continue

        pending = []
        i += 1

    return banner, types


def _leading_banner(lines):
    """The file's opening `//` commentary — the design prose worth carrying."""
    out = []
    started = False
    for line in lines:
        s = line.strip()
        if s.startswith("#pragma once"):
            started = True
            continue
        if not started:
            continue
        if s.startswith("//"):
            out.append(s[2:].rstrip())
            continue
        if s == "":
            if out:
                break
            continue
        break
    while out and not out[-1].strip():
        out.pop()
    return out


def _parse_body(lines, start, decl):
    """Walk a type body from its opening brace; append public members.

    The subtlety that bit the first draft: a one-line inline function
    (`int f() const { return x; }`) contains a `;` INSIDE its body, so
    "declaration ends at the first `;`" cuts in the wrong place and renders a
    signature with the body glued on. Termination is therefore found by
    scanning for the first `;`, `{`, or (constructor init list) `:` that is
    outside parentheses, angle brackets, string literals, and `::`.
    """
    if decl.kind == "enum class":
        return _parse_enum_body(lines, start, decl)

    access = "private" if decl.kind == "class" else "public"
    i = start + 1
    pending = []
    acc = []
    acc_line = 0

    while i < len(lines):
        line = lines[i]
        stripped = line.strip()

        if not acc:
            if stripped.startswith("}"):
                return i + 1  # the type's own closing brace
            doc = _strip_doc(line)
            if doc is not None:
                pending.append(doc)
                i += 1
                continue
            if not stripped or stripped.startswith("//"):
                pending = []
                i += 1
                continue
            m = re.match(r"^(public|private|protected)\s*:\s*$", stripped)
            if m:
                access = m.group(1)
                pending = []
                i += 1
                continue

        acc.append(line)
        if len(acc) == 1:
            acc_line = i + 1
        joined = "\n".join(_strip_line_comment(l) for l in acc)
        how, idx = _find_terminator(joined)
        if how == "brace" and not _looks_like_function(_normalize(joined[:idx])):
            # A `{` on a declaration with no parameter list is a brace
            # INITIALIZER (`units::Time timeout{0.0};`), not a function body.
            # Terminating there would silently drop the default value from the
            # rendered signature — a generator that drops information is worse
            # than no generator.
            how, idx = _find_terminator(joined, brace_ends=False)
        if how is None:
            i += 1
            continue

        sig = _normalize(joined[:idx])
        trailing = _trailing_doc(acc[-1])
        doc = list(pending) if pending else ([trailing] if trailing else [])

        if access == "public" and _is_declaration(sig):
            if re.match(r"^(class|struct|enum)\b", sig):
                raise SystemExit(
                    "api_doc_tool: a PUBLIC nested type was found and this tool "
                    "does not handle them:\n"
                    f"    {decl.name} :: {sig}\n"
                    "Extend the tool deliberately rather than letting its members "
                    "vanish from the reference (which is the exact failure the doc "
                    "coverage gate exists to prevent).")
            kind = "function" if _looks_like_function(sig) else "field"
            decl.members.append(Member(kind, _member_name(sig), sig, doc, acc_line))

        # A run of `= delete` / `= default` special members shares one comment
        # placed above the run; ANY other declaration consumes it, so the next
        # member needs its own. (Without that second half, adding a member
        # directly below a documented one would inherit its neighbour's doc —
        # the hole this gate exists to close.)
        if not _is_special_defaulted(sig):
            pending = []

        i = _skip_body(lines, i, joined, idx) if how in ("brace", "colon") else i + 1
        acc = []

    return i


def _skip_body(lines, i, joined, idx):
    """Advance past an inline function body, back to class-body level."""
    rest = joined[idx:]
    seen = "{" in rest
    bal = rest.count("{") - rest.count("}")
    j = i
    while (not seen) or bal > 0:
        j += 1
        if j >= len(lines):
            break
        code = _strip_line_comment(lines[j])
        if "{" in code:
            seen = True
        bal += code.count("{") - code.count("}")
    return j + 1


def _find_terminator(text, brace_ends=True):
    """('semi'|'brace'|'colon', index) of the end of a declaration, or (None, -1).

    Ignores anything inside (), <>, {} or a string literal, and steps over `::`.
    With brace_ends=False a `{` opens a brace-initializer instead of ending the
    declaration, and only `;` terminates.
    """
    paren = 0
    angle = 0
    brace = 0
    in_str = False
    i = 0
    n = len(text)
    while i < n:
        ch = text[i]
        if in_str:
            if ch == "\\":
                i += 2
                continue
            if ch == '"':
                in_str = False
            i += 1
            continue
        if ch == '"':
            in_str = True
        elif ch == "(":
            paren += 1
        elif ch == ")":
            paren -= 1
        elif ch == "<":
            angle += 1
        elif ch == ">":
            angle = max(0, angle - 1)
        elif ch == "}" and not brace_ends:
            brace = max(0, brace - 1)
        elif paren == 0 and angle == 0 and brace == 0:
            if ch == ";":
                return "semi", i
            if ch == "{":
                if not brace_ends:
                    brace += 1
                    i += 1
                    continue
                return "brace", i
            if ch == ":":
                if text[i:i + 2] == "::":
                    i += 2
                    continue
                if i > 0 and text[i - 1] == ":":
                    i += 1
                    continue
                return "colon", i
        i += 1
    return None, -1


def _strip_line_comment(line):
    cut = line.find("///<")
    if cut < 0:
        cut = line.find("//")
    return line[:cut] if cut >= 0 else line


def _is_declaration(sig):
    if not sig:
        return False
    if sig.startswith(("#", "using ", "friend ", "static_assert")):
        return False
    return True


def _looks_like_function(sig):
    depth = 0
    for ch in sig:
        if ch == "<":
            depth += 1
        elif ch == ">":
            depth = max(0, depth - 1)
        elif ch == "(" and depth == 0:
            return True
    return False


def _brace_delta(text):
    code = "\n".join(_strip_line_comment(l) for l in text.splitlines())
    return code.count("{") - code.count("}")


def _parse_enum_body(lines, start, decl):
    depth = lines[start].count("{") - lines[start].count("}")
    i = start + 1
    pending = []
    while i < len(lines) and depth > 0:
        line = lines[i]
        stripped = line.strip()
        depth += _brace_delta(line)
        if depth <= 0:
            break
        doc = _strip_doc(line)
        if doc is not None:
            pending.append(doc)
            i += 1
            continue
        if not stripped or stripped.startswith("//"):
            pending = []
            i += 1
            continue
        m = re.match(r"^([A-Za-z_]\w*)\s*(=[^,]*)?,?", stripped)
        if m:
            trailing = _trailing_doc(line)
            doc_lines = pending if pending else ([trailing] if trailing else [])
            decl.members.append(
                Member("enumerator", m.group(1), _normalize(stripped).rstrip(","),
                       doc_lines, i + 1))
            pending = []
        i += 1
    return i + 1


# ── rendering ─────────────────────────────────────────────────────────────────────

GENERATED_NOTE = (
    "<!-- GENERATED FILE — DO NOT EDIT BY HAND.\n"
    "     Source: {header}\n"
    "     Regenerate: python3 tools/api_doc_tool.py generate\n"
    "     The host test build fails if this file is out of date, so an edit here\n"
    "     is reverted by the next build rather than reviewed. Edit the header. -->\n"
)


def _anchor(text):
    keep = [c.lower() for c in text if c.isalnum() or c in " -_"]
    return "".join(keep).strip().replace(" ", "-")


def assign_anchors(types):
    """Give every type and member a UNIQUE, stable anchor and display label.

    Overloads share a name, so a naive name-derived anchor collides and every
    link to `scheduler` lands on the first one — a reference whose links go to
    the wrong member is worse than one with no links. Numbering follows SOURCE
    order, which is deterministic, so the regeneration check stays usable.
    """
    for t in types:
        t.anchor = _anchor(f"{t.kind} {t.name}")
        seen = {}
        for mem in t.members:
            spelled = (mem.name
                       .replace("~", "destructor-")
                       .replace("operator=", "operator-assign"))
            base = _anchor(f"{t.name}-{spelled}")
            n = seen.get(base, 0) + 1
            seen[base] = n
            mem.anchor = base if n == 1 else f"{base}-{n}"
            mem.label = mem.name if n == 1 else f"{mem.name} (overload {n})"


def render(target, banner, types):
    header = target["header"]
    out = [GENERATED_NOTE.format(header=header)]
    out.append(f"# {target['title']}\n")
    out.append(target["blurb"] + "\n")
    out.append(
        f"Extracted from [`{header}`](../../{header}) — this page **is** that header's "
        "documentation, reformatted, so it cannot disagree with the code. Prose about "
        "*how to think about* the API lives in the "
        "[user guide](../guide/README.md); worked recipes live in the "
        "[cookbook](../cookbook/README.md); this page is the complete, mechanical "
        "list of what exists.\n")

    out.append("## Contents\n")
    for t in types:
        out.append(f"- [`{t.kind} {t.name}`](#{t.anchor})")
        for mem in t.members:
            out.append(f"  - [`{mem.label}`](#{mem.anchor})")
    out.append("")

    for t in types:
        out.append(f'<a id="{t.anchor}"></a>\n')
        out.append(f"## `{t.kind} {t.name}`\n")
        if t.doc:
            out.append(" ".join(t.doc) + "\n")
        out.append(
            f"*Declared at [`{header}:{t.line}`](../../{header}#L{t.line}).*\n")
        if not t.members:
            out.append("_No public members._\n")
        for mem in t.members:
            out.append(f'<a id="{mem.anchor}"></a>\n')
            out.append(f"### `{t.name}::{mem.label}`\n")
            out.append("```cpp")
            out.append(mem.signature)
            out.append("```\n")
            out.append(" ".join(mem.doc) if mem.doc else "**UNDOCUMENTED.**")
            out.append("")
            out.append(
                f"*{mem.kind}, declared at "
                f"[`{header}:{mem.line}`](../../{header}#L{mem.line}).*\n")

    out.append("## Design commentary, from the header\n")
    out.append(
        "The header opens with the reasoning behind these shapes. It is reproduced "
        "here in full because a reference that only lists signatures teaches nobody "
        "*why*.\n")
    out.append("```text")
    out.extend(banner)
    out.append("```")
    return "\n".join(out).rstrip() + "\n"


def render_index(pages):
    out = [
        "<!-- GENERATED FILE — DO NOT EDIT BY HAND.\n"
        "     Regenerate: python3 tools/api_doc_tool.py generate -->\n",
        "# API reference\n",
        "Every public member of shulib's autonomous-routine API, extracted from the "
        "headers. This page is generated, so it cannot fall behind the code: a member "
        "added to a documented header appears here the next time the tool runs, and "
        "the host test build fails if it has not.\n",
        "**A member with no documentation comment fails the build**, naming itself. "
        "That gate is what makes \"generated\" mean \"complete\" rather than "
        "\"generated from whatever someone remembered to write\".\n",
        "## Pages\n",
    ]
    for target, _banner, types in pages:
        rel = os.path.basename(target["out"])
        out.append(f"- [{target['title']}]({rel}) — {target['blurb']}")
    out.append("")
    out.append("## Every public member, alphabetically\n")
    out.append("| Member | Type | Page |")
    out.append("|---|---|---|")
    rows = []
    for target, _banner, types in pages:
        rel = os.path.basename(target["out"])
        for t in types:
            for mem in t.members:
                rows.append((f"{t.name}::{mem.label}", t.name,
                             f"[{rel}]({rel}#{mem.anchor})"))
    for name, tname, link in sorted(rows, key=lambda r: (r[0].lower(), r[1])):
        rows_name = name.replace("|", "\\|")
        out.append(f"| `{rows_name}` | `{tname}` | {link} |")
    out.append("")
    out.append("## Where the other documents fit\n")
    out.append(
        "- The [user guide](../guide/README.md) teaches the ideas in order, and "
        "chapter 10 is the API *as prose* — when to reach for a verb, what it does "
        "when things go wrong, which gotchas bite. It deliberately does not restate "
        "signatures; this reference owns those.\n"
        "- The [cookbook](../cookbook/README.md) answers \"how do I write the routine "
        "I am writing right now\", with compiled recipes.\n"
        "- This reference answers \"what exactly exists, and what is its exact "
        "spelling\".")
    return "\n".join(out).rstrip() + "\n"


# ── the three jobs ────────────────────────────────────────────────────────────────

def build_pages():
    pages = []
    for target in TARGETS:
        banner, types = parse_header(target["header"])
        assign_anchors(types)
        pages.append((target, banner, types))
    return pages


def do_generate(outroot=None):
    root = outroot or REPO
    pages = build_pages()
    written = []
    for target, banner, types in pages:
        path = os.path.join(root, target["out"])
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(render(target, banner, types))
        written.append(target["out"])
    path = os.path.join(root, INDEX_OUT)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(render_index(pages))
    written.append(INDEX_OUT)
    return written


def do_check_coverage():
    missing = []
    for target, _banner, types in build_pages():
        for t in types:
            if not t.doc:
                missing.append((target["header"], t.line, f"{t.kind} {t.name}",
                                f"{t.kind} {t.name}"))
            for mem in t.members:
                if not mem.documented:
                    missing.append((target["header"], mem.line,
                                    f"{t.name}::{mem.name}", mem.signature))
    if not missing:
        return 0
    print("", file=sys.stderr)
    print("DOC COVERAGE FAILURE — a public member has no /// documentation.",
          file=sys.stderr)
    print("The generated API reference would omit it silently, and the reference",
          file=sys.stderr)
    print("would still look complete. That is why this fails the build.",
          file=sys.stderr)
    print("", file=sys.stderr)
    for header, line, name, sig in missing:
        print(f"  UNDOCUMENTED: {name}", file=sys.stderr)
        print(f"      at {header}:{line}", file=sys.stderr)
        print(f"      {sig}", file=sys.stderr)
    print("", file=sys.stderr)
    print("Fix: write a /// comment directly above the declaration (or a ///<",
          file=sys.stderr)
    print("comment on the same line) saying what it is FOR, then rebuild.",
          file=sys.stderr)
    print("A run of `= delete` / `= default` special members may share one",
          file=sys.stderr)
    print("comment placed above the run, with no blank line between them.",
          file=sys.stderr)
    return 1


def do_check_fresh():
    tmp = tempfile.mkdtemp(prefix="shulib-api-")
    try:
        written = do_generate(outroot=tmp)
        stale = []
        for rel in written:
            fresh = open(os.path.join(tmp, rel), encoding="utf-8").read()
            committed_path = os.path.join(REPO, rel)
            if not os.path.exists(committed_path):
                stale.append((rel, ["(file does not exist)"]))
                continue
            committed = open(committed_path, encoding="utf-8").read()
            if committed != fresh:
                diff = list(difflib.unified_diff(
                    committed.splitlines(), fresh.splitlines(),
                    fromfile=f"{rel} (committed)", tofile=f"{rel} (fresh)",
                    lineterm="", n=1))[:40]
                stale.append((rel, diff))
        # A hand-written file dropped into docs/api/ is caught here too: the
        # generated set is the whole directory's contents, by definition.
        apidir = os.path.join(REPO, "docs", "api")
        expected = {os.path.basename(r) for r in written}
        if os.path.isdir(apidir):
            for name in sorted(os.listdir(apidir)):
                if name not in expected:
                    stale.append((f"docs/api/{name}",
                                  ["(not produced by the generator — docs/api/ is "
                                   "generated in full; put hand-written prose in "
                                   "docs/guide/ or docs/cookbook/)"]))
        if not stale:
            return 0
        print("", file=sys.stderr)
        print("API REFERENCE IS STALE — docs/api/ does not match the headers.",
              file=sys.stderr)
        print("Run:  python3 tools/api_doc_tool.py generate", file=sys.stderr)
        print("", file=sys.stderr)
        # This gate runs before any C++ compiles, so it fires FIRST when a
        # frozen signature changes — ahead of the F6/F10 signature pins that
        # name the real problem. Regenerating is the right fix for a comment
        # or a new member; it is the WRONG fix for a frozen signature, and a
        # reader who only sees "run this command" would make the break look
        # official before the pin gets its say. (It still gets its say on the
        # next build — verified — but the first message a person acts on
        # should not point the wrong way.) Found while verifying D3.
        print("Before you regenerate: if the diff below changes a SIGNATURE on "
              "a frozen surface", file=sys.stderr)
        print("(Chassis = F6, Routine = F10), that is a BREAKING CHANGE, not a "
              "stale document.", file=sys.stderr)
        print("Regenerating would only make the break look intentional. See "
              "include/shulib/version.hpp.", file=sys.stderr)
        print("", file=sys.stderr)
        for rel, diff in stale:
            print(f"  {rel}", file=sys.stderr)
            for line in diff:
                print(f"    {line}", file=sys.stderr)
        return 1
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ── the documentation-wide gates (D3 campaign finds H1/H6 and H7) ────────────────

# Public markdown, in the order a reviewer would read it. docs/internal/ is
# excluded everywhere: it is the development log, dropped at the squash-merge
# to main.
#
# docs/api/ is included here but skipped by the VERBATIM scan only, because it
# is GENERATED — check-fresh compares that whole directory byte-for-byte
# against a fresh run, which is strictly stronger and also catches a
# hand-written file dropped there. It is NOT skipped by the removability
# check: generated output is still public output, and the reference reproduces
# each header's banner in full, so an internal path in a header comment lands
# in a published page. (Learned the hard way in the same session; see
# do_check_removability.)
PUBLIC_DOC_GLOBS = ["docs/*.md", "docs/guide/*.md", "docs/cookbook/*.md", "docs/api/*.md"]
PUBLIC_DOC_FILES = ["README.md", "test/README.md"]

# The compiled-example sources are a GLOB, not a list: a gate you must remember
# to update when adding a file is a gate that rots (the same reasoning as the
# ARM gate's generated header list).
EXAMPLE_SOURCE_GLOB = "test/*example*_test.cpp"

# The C7 removability property: docs/internal/ must be droppable without
# breaking a single public link.
REMOVABILITY_TERMS = re.compile(r"internal/|chunks/|RESUMING|build-order")


def _public_docs():
    import glob as _glob
    out = []
    for pattern in PUBLIC_DOC_GLOBS:
        out.extend(_glob.glob(os.path.join(REPO, pattern)))
    for rel in PUBLIC_DOC_FILES:
        path = os.path.join(REPO, rel)
        if os.path.exists(path):
            out.append(path)
    return sorted(os.path.relpath(p, REPO) for p in out)


def do_check_examples():
    """Every ```cpp line in public docs must appear VERBATIM in a compiled test.

    CAMPAIGN FIND (D3, mutations H1 and H6): this rule has existed since C8 and
    was enforced only by the internal verify harness — so a rotted example
    passed the build AND passed CI. A `300_ms` retyped as `300_s` in a chapter
    survived everything. The rule is only a mechanism when a machine runs it.
    """
    import glob as _glob
    sources = sorted(_glob.glob(os.path.join(REPO, EXAMPLE_SOURCE_GLOB)))
    if not sources:
        print("DOC EXAMPLE SCAN: no test/*example*_test.cpp found — the scan would "
              "pass vacuously, which is worse than failing.", file=sys.stderr)
        return 1
    known = set()
    for src in sources:
        for line in open(src, encoding="utf-8").read().splitlines():
            known.add(line.strip())

    bad = []
    checked = 0
    for rel in _public_docs():
        if rel.startswith("docs/api/"):
            continue  # generated in full; check-fresh is the stronger rule (above)
        text = open(os.path.join(REPO, rel), encoding="utf-8").read()
        for bi, block in enumerate(re.findall(r"```cpp\n(.*?)```", text, re.S), 1):
            for line in block.splitlines():
                s = line.strip()
                if not s:
                    continue
                checked += 1
                if s not in known:
                    bad.append((rel, bi, s))
    if not bad:
        print(f"doc example scan: {checked} quoted lines, all verbatim "
              f"({len(sources)} source files)")
        return 0
    print("", file=sys.stderr)
    print("NOT VERBATIM — a documented example has drifted from the code that "
          "compiles it.", file=sys.stderr)
    print("Fix the EXAMPLE TEST first, then re-quote the listing from it. Never "
          "hand-edit", file=sys.stderr)
    print("the markdown toward agreement: only one of the two sides is checked "
          "by a compiler.", file=sys.stderr)
    print("", file=sys.stderr)
    for rel, bi, line in bad[:40]:
        print(f"  {rel} (block {bi}): {line}", file=sys.stderr)
    if len(bad) > 40:
        print(f"  … and {len(bad) - 40} more", file=sys.stderr)
    return 1


def do_check_removability():
    """No public document may reference docs/internal/.

    CAMPAIGN FIND (D3, mutation H7): the C7 removability property was checked
    only by the internal verify harness, so a public doc could link into
    docs/internal/ and everything stayed green — until the squash-merge to
    main, where the link breaks and the property is discovered by a reader.

    SECOND FIND, same session: the first version of this check skipped
    docs/api/ along with the verbatim scan — and the generated reference
    reproduces each header's design banner IN FULL, so a header comment naming
    `docs/internal/chunks/...` walked straight into a public document. (The
    D3 verification harness caught it; this gate had not.) The exclusion that
    is right for the example scan is wrong here: freshness proves the file
    matches the header, not that the header is fit to publish. Generated
    output is public output.
    """
    hits = []
    for rel in _public_docs():
        for n, line in enumerate(
                open(os.path.join(REPO, rel), encoding="utf-8").read().splitlines(), 1):
            if REMOVABILITY_TERMS.search(line):
                hits.append((rel, n, line.strip()))
    if not hits:
        print("removability: no public doc references docs/internal/")
        return 0
    print("", file=sys.stderr)
    print("REMOVABILITY VIOLATION — a public document references the internal "
          "development log.", file=sys.stderr)
    print("docs/internal/ is dropped when this branch squash-merges to main, so "
          "the reference", file=sys.stderr)
    print("becomes a broken link in the published documentation.", file=sys.stderr)
    print("", file=sys.stderr)
    for rel, n, line in hits[:30]:
        print(f"  {rel}:{n}: {line[:120]}", file=sys.stderr)
    return 1


# ── self-test ─────────────────────────────────────────────────────────────────────

FIXTURE = '''#pragma once
//
// Fixture banner line one.
// Fixture banner line two.

#include <cstddef>

namespace fixture {

/// A documented enum.
enum class Colour {
    Red,    ///< the documented one
    Green,
};

/// A documented struct.
struct Thing {
    /// Documented field.
    int a = 0;
    int b = 0;  ///< documented by a trailing comment
    int c = 0;
    ///
    int d = 0;

    void undocumentedFn() const {}
};

/// A documented class.
class Widget {
public:
    /// The constructor.
    explicit Widget(int seed) noexcept : seed_{seed} {}

    /// Not copyable: two handles would fork the state.
    Widget(const Widget&) = delete;
    Widget& operator=(const Widget&) = delete;
    ~Widget() = default;

    int leaks() const { return seed_; }

    /// Documented, with a default argument and a const overload below.
    int query(int fallback = 3) const noexcept { return fallback; }

    /// The non-const overload.
    int query(int fallback, bool loud) { return loud ? fallback : seed_; }

    /// A template member, whose template head is part of how you call it.
    template <typename Pred>
    int count(Pred&& pred, int limit = 5) const { return pred(seed_) ? limit : 0; }

private:
    int hidden() const { return seed_; }
    int seed_;
};

}  // namespace fixture
'''


def _selftest_parse(tmpdir):
    path = os.path.join(tmpdir, "fixture.hpp")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(FIXTURE)
    global REPO
    saved = REPO
    REPO = tmpdir
    try:
        return parse_header("fixture.hpp")
    finally:
        REPO = saved


def do_self_test():
    failures = []

    def check(cond, what):
        if not cond:
            failures.append(what)

    tmp = tempfile.mkdtemp(prefix="shulib-api-selftest-")
    try:
        banner, types = _selftest_parse(tmp)

        # Bug caught: the parser silently finding no types (which would make the
        # coverage gate vacuously green — the worst possible failure here).
        names = [t.name for t in types]
        check(names == ["Colour", "Thing", "Widget"], f"types found: {names}")

        by = {t.name: t for t in types}

        # Bug caught: enumerators not treated as members, so an undocumented
        # enumerator would never be reported.
        colour = [m.name for m in by["Colour"].members]
        check(colour == ["Red", "Green"], f"enumerators: {colour}")
        check(by["Colour"].members[0].documented, "trailing ///< on an enumerator")
        check(not by["Colour"].members[1].documented, "undocumented enumerator")

        # Bug caught: a trailing ///< not counting as documentation (which would
        # flag most of RoutineResult) or counting for the WRONG member.
        thing = {m.name: m for m in by["Thing"].members}
        check(set(thing) == {"a", "b", "c", "d", "undocumentedFn"},
              f"Thing: {set(thing)}")
        check(thing["a"].documented, "/// above a field")
        check(thing["b"].documented, "///< beside a field")
        check(not thing["c"].documented, "field with no comment must be undocumented")

        # Bug caught: FOUND BY MUTATION (D3 hole probe H3) — a bare `///` with
        # no text passed the gate, and the reference rendered an empty
        # paragraph. A gate that accepts empty documentation reproduces, inside
        # itself, the exact "nothing looks wrong" failure it exists to prevent.
        check(not thing["d"].documented, "an EMPTY /// must not count as documented")

        # Bug caught: THE HOLE this gate exists to close — a member added right
        # after a documented one inheriting its neighbour's comment.
        check(not thing["undocumentedFn"].documented,
              "a member must not inherit the previous member's comment")

        widget = by["Widget"].members
        wnames = [m.name for m in widget]
        check(wnames == ["Widget", "Widget", "operator=", "~Widget", "leaks",
                         "query", "query", "count"], f"Widget members: {wnames}")

        # Bug caught: private members leaking into the public reference.
        check("hidden" not in wnames, "private members must not appear")

        wby = {}
        for m in widget:
            wby.setdefault(m.name, []).append(m)

        # Bug caught: the run-of-special-members rule not applying, or applying
        # too widely (leaking a comment past a normal member).
        check(all(m.documented for m in wby["Widget"]), "= delete run shares one doc")
        check(wby["operator="][0].documented, "= delete run covers operator=")
        check(wby["~Widget"][0].documented, "= default in the run is covered")
        check(not wby["leaks"][0].documented,
              "the special-member comment must NOT leak to the next normal member")

        # Bug caught: an overload set collapsed to one entry, or a default
        # argument / const / noexcept dropped from the rendered signature — a
        # generator that silently drops a qualifier is worse than none.
        check(len(wby["query"]) == 2, "both overloads survive")
        sigs = [m.signature for m in wby["query"]]
        check("int query(int fallback = 3) const noexcept" in sigs,
              f"const/noexcept/default-arg preserved: {sigs}")
        check("int query(int fallback, bool loud)" in sigs,
              f"second overload preserved: {sigs}")
        # Bug caught: the template head dropped from a template member, so the
        # reference shows a signature nobody can call. FOUND BY MUTATION G5 —
        # the first fixture had no template member, so this exact regression
        # walked past the self-test and had to be caught downstream.
        tmpl = wby["count"][0].signature
        check(tmpl == "template <typename Pred> int count(Pred&& pred, int limit = 5) const",
              f"template head preserved: {tmpl!r}")

        ctor = [m.signature for m in wby["Widget"]][0]
        check(ctor == "explicit Widget(int seed) noexcept",
              f"ctor rendered without its init list: {ctor!r}")

        # Bug caught: the header's design commentary silently dropped, leaving a
        # reference of signatures with no reasoning.
        check(banner[:2] == ["", " Fixture banner line one."], f"banner: {banner[:2]}")

        # Bug caught: NON-DETERMINISM — without this, the regeneration check in
        # the build would fail at random and get switched off.
        assign_anchors(types)
        first = render(TARGETS[0], banner, types)
        second = render(TARGETS[0], banner, types)
        check(first == second, "render is deterministic")
        one = do_generate(outroot=os.path.join(tmp, "a"))
        two = do_generate(outroot=os.path.join(tmp, "b"))
        check(one == two, "generate writes the same file set twice")
        for rel in one:
            a = open(os.path.join(tmp, "a", rel), encoding="utf-8").read()
            b = open(os.path.join(tmp, "b", rel), encoding="utf-8").read()
            check(a == b, f"{rel} is byte-identical across runs")

    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if failures:
        print("SELF-TEST FAILURES:", file=sys.stderr)
        for f in failures:
            print(f"  - {f}", file=sys.stderr)
        return 1
    print("api_doc_tool self-test: OK")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("command",
                    choices=["generate", "check-coverage", "check-fresh",
                             "check-examples", "check-removability", "self-test"])
    args = ap.parse_args()
    if args.command == "generate":
        for rel in do_generate():
            print(f"wrote {rel}")
        return 0
    if args.command == "check-coverage":
        return do_check_coverage()
    if args.command == "check-fresh":
        return do_check_fresh()
    if args.command == "check-examples":
        return do_check_examples()
    if args.command == "check-removability":
        return do_check_removability()
    return do_self_test()


if __name__ == "__main__":
    sys.exit(main())
