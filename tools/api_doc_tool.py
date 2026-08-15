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

SCOPE: every public header under include/shulib/, discovered by GLOB, minus a
short exclusion list that states its reasons (see TARGET_EXCLUSIONS). It parses
this repository's house style — one declaration per statement, /// above or
///< trailing — and is not a C++ parser and must never pretend to be one. If it
is ever pointed at a shape it cannot parse, the right response is to fail
loudly, not to guess.

WHY THE TARGET LIST IS DERIVED AND NOT WRITTEN DOWN (chunk DOCS2)
-----------------------------------------------------------------
Until DOCS2 this was a two-entry hand-written list, and the reference covered
two of roughly 160 public types. Expanding a hand-written list to ~115 entries
would move the failure rather than fix it: a header added later and forgotten
is invisible to the generator AND to the coverage gate, which is the exact
"silently absent, and the reference still looks complete" failure described
above. So the list is a glob, on the same reasoning already recorded below for
EXAMPLE_SOURCE_GLOB: a gate you must remember to update when adding a file is a
gate that rots.

WHAT DOCS2 FOUND WHEN IT POINTED THE PARSER AT THE REST OF THE TREE
-------------------------------------------------------------------
The old opener regex required `{` immediately after an optional `final`, so it
never saw a type with a base-class list (`class Localizer final : public
IPoseSource`) or an enum with an explicit underlying type (`enum class FaultCode
: std::uint16_t`) — 59 top-level definitions in all, close to every concrete
implementation class in the library. It also only ever walked type BODIES, so
nothing at namespace scope existed for it: 85 free functions, 32 constants and
11 `using` aliases, including all of spec/accuracy.hpp and all of version.hpp,
which both parsed to zero items. Three LOCKED contracts — the coordinate frame,
the accuracy targets and the units vocabulary — were invisible in their
entirety.

None of that failed anything, because the two headers it was pointed at happen
to have no base list. A parser that cannot see a declaration reports nothing at
all, which is why this file now fails loudly on any shape it does not
recognise instead of skipping it.
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

API_ROOT = "include/shulib"
INDEX_OUT = "docs/api/README.md"
# The alphabetical index is its own page rather than a section of the overview.
# Measured: as one table it is ~180 KB of the overview's 190 KB, which buries
# the scope paragraph — the one paragraph on this whole site that has already
# had to be corrected twice — under sixteen hundred table rows.
ALL_OUT = "docs/api/all-entities.md"
MKDOCS = "mkdocs.yml"

# Directories under include/shulib/ that are NOT part of the published API.
# Each states its reason, because "a gate's exclusion list is where its holes
# live" (D3) — an unexplained exclusion is indistinguishable from an oversight.
TARGET_EXCLUSIONS = (
    # Test-only by CI guard: nothing outside sim/ may include shulib/sim/, so
    # no shipped robot binary can reach it. Documenting it in the public
    # reference would advertise a surface the guard forbids depending on.
    ("include/shulib/sim/", "test-only — enforced by the sim-layering CI guard"),
    # Test doubles. Public headers by file placement, test fixtures by charter:
    # hal/fake/ and localization/fake/ exist so the suite can drive the real
    # seams. They are documented for test authors in test/README.md.
    ("/fake/", "test doubles — documented for test authors in test/README.md"),
)

# Shapes this tool deliberately does not extract, recorded rather than silent.
# Neither is "missing" from a page: the header's design banner is reproduced in
# full on every page, so both are visible in prose. What they are absent from is
# the mechanical member list and the alphabetical index.
#
#   * PREPROCESSOR MACROS (SHULIB_PRECONDITION, SHULIB_TRACE). A macro is not a
#     declaration and has no signature, access or type; extracting one would
#     mean inventing a rendering for it. Both are explained at length in their
#     own headers' banners.
#   * PROTECTED members (one section in the tree, motion/move_to_pose.hpp).
#     This is the reference for the PUBLIC surface; the subclass-extension
#     surface is guide chapter 13's subject.
#
# Both are stated on the generated index so a reader is told, not left to infer
# completeness.

# One page per header. The page is named after the header, with any path BELOW
# the subsystem folded in with '-'; the subsystem itself is carried by the nav,
# not by the filename.
#
#   chassis/chassis.hpp -> chassis.md      hal/motor.hpp      -> motor.md
#   chassis/routine.hpp -> routine.md      hal/pros/motor.hpp -> pros-motor.md
#
# Rejected: mirroring the include tree (docs/api/hal/pros/motor.md). It puts
# pages at three different depths, so every ../../include/... link the renderer
# emits becomes depth-dependent, and it renames docs/api/chassis.md and
# docs/api/routine.md — named by test/api_reference_fidelity_test.cpp,
# test/CMakeLists.txt, four public documents and a reviewer's verify harness.
# Flat keeps every page at depth 1 and every existing link correct.
#
# Uniqueness is ASSERTED below, not assumed.


def page_name(header):
    """docs/api/ file name for a header path, by the rule above."""
    rel = header[len(API_ROOT) + 1:]           # e.g. "hal/pros/motor.hpp"
    parts = rel.split("/")
    tail = parts[1:] if len(parts) > 1 else parts   # drop the subsystem dir
    return "-".join(tail)[:-len(".hpp")] + ".md"


def subsystem_of(header):
    """The header's directory under include/shulib/ — "hal", "hal/pros", "".

    The full relative directory rather than its first component, so the PROS
    adapters group on their own in the nav instead of interleaving with the
    seams they implement.
    """
    return os.path.dirname(header[len(API_ROOT) + 1:])


# Nav group order and labels. The order follows the architecture top-down (the
# facade a routine author starts at, down to the hardware seams); an unknown
# subsystem — a directory added by a later chunk — is appended alphabetically
# with a title-cased label, so a new directory is never silently dropped and
# never needs this table edited to appear.
SUBSYSTEM_LABELS = [
    ("chassis", "Chassis and routines"),
    ("motion", "Motion"),
    ("control", "Control"),
    ("kinematics", "Kinematics"),
    ("localization", "Localization"),
    ("manipulation", "Manipulation"),
    ("sequence", "Sequencing"),
    ("diag", "Diagnostics"),
    ("math", "Math and frames"),
    ("units", "Units"),
    ("hal", "HAL — the hardware seams"),
    ("hal/pros", "HAL — the PROS adapters"),
    ("core", "Core"),
    ("spec", "Spec"),
    ("", "Top level"),
]


def grouped_pages(pages):
    """[(label, [page, …]), …] — every page, grouped and ordered for the nav."""
    known = [k for k, _ in SUBSYSTEM_LABELS]
    labels = dict(SUBSYSTEM_LABELS)
    by_sub = {}
    for page in pages:
        by_sub.setdefault(page[0]["subsystem"], []).append(page)
    order = [k for k in known if k in by_sub]
    order += sorted(k for k in by_sub if k not in known)
    return [(labels.get(k, k.replace("/", " ").title() or "Top level"),
             sorted(by_sub[k], key=lambda p: p[0]["out"]))
            for k in order]


# Nav labels are what a reader actually sees, and a sidebar reading "I corrector
# / I fusion policy / Pid / Gps conversion" looks like nobody proof-read it. The
# table maps the tokens whose house spelling is not title-case; everything else
# falls through to plain capitalisation, so a new header needs no entry here to
# get a sane label.
LABEL_WORDS = {
    "pid": "PID", "gps": "GPS", "imu": "IMU", "ekf": "EKF", "sd": "SD",
    "hal": "HAL", "pros": "PROS", "api": "API", "odo": "Odometry",
    "apriltag": "AprilTag", "blackbox": "Blackbox", "pilons": "Pilons",
    "x": "X", "h": "H", "adi": "ADI",
}


def _title_words(stem):
    """`pros-motor` -> `Motor (PROS)`; `i_corrector` -> `ICorrector`."""
    suffix = ""
    if stem.startswith("pros-"):
        suffix, stem = " (PROS)", stem[len("pros-"):]
    parts = stem.replace("-", "_").split("_")
    # `i_corrector` / `i_fusion_policy` are interfaces; the house spelling is
    # ICorrector / IFusionPolicy, and splitting the I off reads as a typo.
    if len(parts) > 1 and parts[0] == "i":
        return "I" + "".join(p[:1].upper() + p[1:] for p in parts[1:]) + suffix
    words = [LABEL_WORDS.get(p, p) for p in parts]
    head = words[0]
    if head not in LABEL_WORDS.values():
        head = head[:1].upper() + head[1:]
    tail = [w if w in LABEL_WORDS.values() else w for w in words[1:]]
    return " ".join([head] + tail) + suffix


def discover_headers():
    """Every public header, sorted, with a stable locale-independent order."""
    import glob as _glob
    found = _glob.glob(os.path.join(REPO, API_ROOT, "**", "*.hpp"), recursive=True)
    rel = [os.path.relpath(p, REPO).replace(os.sep, "/") for p in found]
    keep = []
    for header in rel:
        if any(pat in "/" + header for pat, _why in TARGET_EXCLUSIONS):
            continue
        keep.append(header)
    return sorted(keep)


def build_targets():
    """The generation + coverage set, derived from the tree.

    Deliberately ONE list. Splitting generation from gating — pages for
    everything, the gate only on frozen contracts — was the alternative DOCS2
    weighed hardest and rejected: it recreates, by construction, the
    two-hand-maintained-lists shape whose gap DOCS1 was bitten by, and it lets
    an ungated page rot to a full column of "UNDOCUMENTED" while staying
    generated, fresh and green. The full ruling, including the Freeze Register
    amendment it required, is in the DOCS2 development record.
    """
    targets = []
    claimed = {}
    for header in discover_headers():
        name = page_name(header)
        if name in claimed:
            raise SystemExit(
                "api_doc_tool: two headers want the same page file:\n"
                f"    {claimed[name]}\n    {header}\n"
                f"  both -> docs/api/{name}\n"
                "Page names must be unique. Extend page_name() deliberately.")
        claimed[name] = header
        targets.append({
            "header": header,
            "out": f"docs/api/{name}",
            "subsystem": subsystem_of(header),
            "title": f"`{os.path.basename(header)}`",
            "nav_label": _title_words(name[:-len(".md")]),
        })
    return targets


# ── the parser ────────────────────────────────────────────────────────────────────

class Member:
    """One public member: how it is spelled, what documents it, where it lives."""

    # "function" | "field" | "enumerator" | "alias"          (inside a type)
    # "free function" | "constant" | "type alias"            (namespace scope)
    def __init__(self, kind, name, signature, doc, line):
        self.kind = kind
        self.name = name
        self.signature = signature  # rendered, whitespace-normalized
        self.doc = doc              # list[str], may be empty  == undocumented
        self.line = line            # 1-based line in the header
        self.owner = None           # TypeDecl, or None at namespace scope

    @property
    def qualified(self):
        return f"{self.owner.qualified}::{self.name}" if self.owner else self.name

    @property
    def documented(self):
        # CAMPAIGN FIND (D3 hole probe H3): `bool(self.doc)` accepted a BARE
        # `///` with no text — the member passed the gate and the reference
        # rendered an empty paragraph. "Nothing looks wrong" is precisely the
        # failure this gate exists to prevent, so it must not be reproducible
        # inside the gate. Documentation must have content.
        return any(line.strip() for line in self.doc)


class TypeDecl:
    """A public type, and — since DOCS2 — any public type nested inside it.

    WHY NESTED TYPES ARE THEIR OWN DECL AND NOT FLATTENED (DOCS2)
    -------------------------------------------------------------
    The tool used to refuse a public nested type outright, with an error saying
    to extend it deliberately rather than let its members vanish. The obvious
    shortcut is to flatten `Outer::Inner` into the parent's member list. That is
    rejected: `BlackboxReader::Frame` would render as though it were a peer of
    `BlackboxReader::open()`, and Frame's own four fields would disappear
    entirely — the precise failure the refusal existed to prevent. A nested type
    keeps its own section, its own anchors, its own indented place in the
    contents, and its own rows in the alphabetical index.
    """

    # "class" | "struct" | "enum class" | "enum" | "union"
    def __init__(self, kind, name, doc, line, signature="", parent=None):
        self.kind = kind
        self.name = name
        self.signature = signature  # the declaration head, as written
        self.doc = doc
        self.line = line
        self.parent = parent
        self.members = []
        self.nested = []

    @property
    def qualified(self):
        return f"{self.parent.qualified}::{self.name}" if self.parent else self.name

    @property
    def depth(self):
        return 0 if self.parent is None else self.parent.depth + 1

    @property
    def documented(self):
        return any(line.strip() for line in self.doc)

    def walk(self):
        """This type, then each nested type, depth-first in source order."""
        yield self
        for child in self.nested:
            yield from child.walk()


def _strip_doc(line):
    """`/// text` -> `text`; `///` alone -> ``. None if not a doc comment.

    A `///<` line is deliberately NOT one. It is a TRAILING comment, and the
    house style wraps long ones onto continuation lines:

        Unconfirmed = 2,  ///< the operation ran to completion and the
                          ///< confirmation said the world did not change

    Because `///<` also starts with `///`, this function used to return
    `< confirmation said…` for that second line, which then sat in `pending`
    and became the documentation for the NEXT enumerator. FOUND IN THE
    PUBLISHED TREE: docs/api/mechanism_outcome.md documented `Unconfirmed` as
    "< confirmation; completed, where it does not)" — Succeeded's second line —
    and `TimedOut` with Unconfirmed's. Three enumerators carried a confident
    sentence about a different value, the coverage gate scored all three as
    documented, and the page looked complete. That is the exact failure this
    tool exists to prevent, committed inside the tool. See _continuation().
    """
    s = line.strip()
    if not s.startswith("///") or s.startswith("///<"):
        return None
    return s[3:].lstrip()


def _continuation(line):
    """Text of a standalone `///<` continuation line, else None."""
    s = line.strip()
    return s[4:].strip() if s.startswith("///<") else None


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
    # A using-declaration names its alias, and the name is NOT the last token
    # before the first `(` — `using PreconditionHandler = void (*)(const char*)`
    # would otherwise be called "void".
    if sig.startswith("using "):
        rest = sig[len("using "):].strip()
        if "=" in rest:
            return rest.split("=", 1)[0].strip()
        return rest.rstrip(";").split("::")[-1].strip()
    # `operator=` / `operator()` / `operator[]` / `operator""_in` keep the
    # keyword with the symbol. Matched directly rather than read off the text
    # before the first `(`, which for `operator()` cuts the name in half.
    m = OPERATOR_NAME.search(sig)
    if m:
        return re.sub(r"\s+", "", m.group(0))
    angle = brace = 0
    for i, ch in enumerate(sig):
        if ch == "<":
            angle += 1
        elif ch == ">":
            angle = max(0, angle - 1)
        elif ch == "{":
            brace += 1
        elif ch == "}":
            brace = max(0, brace - 1)
        elif angle == 0 and brace == 0:
            if ch == "=":
                break            # an initializer, not a parameter list
            if ch == "(":
                return _bare_identifier(sig[:i])
    # No parameter list: a field or a constant. Name is the identifier before
    # = or { or ;.
    return _bare_identifier(re.split(r"[={;]", sig)[0])


def _bare_identifier(head):
    """Last token of a declarator head, without `*`/`&` or an array bound.

    `inline constexpr char kMagic[4]` names kMagic, not `kMagic[4]` — an
    anchor and an index row carrying the bound would not match what a reader
    searches for.
    """
    tokens = head.strip().split()
    if not tokens:
        return "?"
    return re.sub(r"\[.*$", "", tokens[-1].lstrip("*&")) or "?"


def _is_special_defaulted(sig):
    """`= delete` / `= default` — the run-of-special-members case."""
    return bool(re.search(r"=\s*(delete|default)\s*$", sig))


# A type opener, at any scope. The three shapes the pre-DOCS2 regex could not
# see are all here: an underlying type on an enum, a base-class list, and
# `final` followed by either. `enum class` must precede bare `enum` in the
# alternation or the former parses as the latter with a name of "class".
TYPE_OPENER = re.compile(
    r"^(class|struct|union|enum\s+class|enum)\s+([A-Za-z_]\w*)"
    r"\s*(final\b)?\s*(:[^{]*)?\{")

# A forward declaration (`struct Frame;`) or an opaque enum declaration. It
# introduces no members, so it is not an entry — but it must be RECOGNISED,
# or it falls through to the field parser and renders as a member named Frame.
FORWARD_DECL = re.compile(
    r"^(class|struct|union|enum\s+class|enum)\s+[A-Za-z_]\w*\s*(:[^;{]*)?;\s*$")

NAMESPACE_OPEN = re.compile(r"^namespace\s+([\w:]+)?\s*\{")

# Namespace-scope statements that declare nothing a reader can call.
NON_DECLARATIONS = ("#", "static_assert", "extern \"C\"", "}", "using namespace")


def parse_header(path):
    """Every public entity in a header, in source order.

    Returns (banner, entries) where an entry is either a TypeDecl (carrying its
    members and any nested TypeDecls) or a Member (a free function, constant or
    type alias at namespace scope).

    Deliberately not a C++ parser: it understands this repository's house style
    (one declaration per statement, /// doc comments directly above or ///<
    trailing) and nothing else. Anything it does not recognise is a loud
    failure, never a skip — see the module docstring for what skipping cost.
    """
    lines = open(os.path.join(REPO, path), encoding="utf-8").read().splitlines()
    banner = _leading_banner(lines)
    entries = []
    _parse_namespace_scope(lines, path, entries)
    return banner, entries


def _parse_namespace_scope(lines, path, entries):
    """Walk column-0 declarations, skipping over type and function bodies."""
    i = 0
    pending = []
    tmpl = []           # a pending `template <...>` head
    ns = []             # the open namespace stack, for the detail/ exclusion
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
            tmpl = []
            i += 1
            continue
        if line.startswith((" ", "\t")):
            # Continuation lines are consumed by the branches below; anything
            # still indented here belongs to a body already skipped past.
            i += 1
            continue

        m = NAMESPACE_OPEN.match(stripped)
        if m:
            ns.append(m.group(1) or "")
            pending = []
            i += 1
            continue
        if stripped.startswith("}"):
            if ns:
                ns.pop()
            pending = []
            i += 1
            continue
        if stripped.startswith(NON_DECLARATIONS):
            pending = []
            tmpl = []
            i += 1
            continue

        # A `template <...>` head belongs to whatever follows it, which may be a
        # type (units::Quantity, manipulation::RunUntilConfirmed) or a free
        # function (hal::emitRecord). It is part of how you name the thing, so
        # dropping it renders a declaration nobody can spell.
        if stripped.startswith("template"):
            tmpl.append(stripped)
            if _angle_balance("\n".join(tmpl)) <= 0:
                pass  # complete head; hold it for the next declaration
            i += 1
            continue

        # Entities inside an internal namespace are implementation detail. This
        # is the only name-based exclusion in the tool and it is narrow on
        # purpose: `detail` is the repository's stated convention, used twice
        # (core/check.hpp, hal/vision_conversion.hpp), and both hold helpers
        # that no caller outside their own header may use.
        public_here = "detail" not in ns

        head = " ".join(tmpl + [stripped]) if tmpl else stripped
        mt = TYPE_OPENER.match(stripped)
        if mt:
            kind = re.sub(r"\s+", " ", mt.group(1))
            decl = TypeDecl(kind, mt.group(2), pending, i + 1,
                            signature=_normalize(head.split("{", 1)[0]))
            pending = []
            tmpl = []
            i = _parse_type_body(lines, i, decl, path)
            if public_here:
                entries.append(decl)
            continue
        if FORWARD_DECL.match(stripped):
            pending = []
            tmpl = []
            i += 1
            continue

        # A free function, a constant, or a type alias.
        i, sig, doc, dline = _read_declaration(lines, i, pending, path, tmpl)
        pending = []
        tmpl = []
        if public_here and _is_declaration(sig):
            entries.append(Member(_member_kind(sig, namespace_scope=True),
                                  _member_name(sig), sig, doc, dline))


def _angle_balance(text):
    return text.count("<") - text.count(">")


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


def _read_declaration(lines, start, pending, path, template=()):
    """Accumulate one declaration from `start`.

    Returns (next_index, sig, doc_lines, declaration_line).

    The subtlety that bit the first draft: a one-line inline function
    (`int f() const { return x; }`) contains a `;` INSIDE its body, so
    "declaration ends at the first `;`" cuts in the wrong place and renders a
    signature with the body glued on. Termination is therefore found by
    scanning for the first `;`, `{`, or (constructor init list) `:` that is
    outside parentheses, angle brackets, braces, string and character literals,
    and `::`.
    """
    i = start
    acc = []
    how, idx, joined = None, -1, ""
    while i < len(lines):
        acc.append(lines[i])
        joined = "\n".join(_strip_line_comment(l) for l in acc)
        how, idx = _find_terminator(joined)
        if how == "brace" and not _looks_like_function(_normalize(joined[:idx])):
            # A `{` on a declaration with no parameter list is a brace
            # INITIALIZER (`units::Time timeout{0.0};`), not a function body.
            # Terminating there would silently drop the default value from the
            # rendered signature — a generator that drops information is worse
            # than no generator.
            how, idx = _find_terminator(joined, brace_ends=False)
        if how is not None:
            break
        i += 1
    if how is None:
        raise SystemExit(
            f"api_doc_tool: unterminated declaration in {path}, "
            f"line {start + 1}:\n    {lines[start].strip()}\n"
            "This tool understands one declaration per statement. Fail loudly "
            "rather than guess — see the module docstring.")

    head = joined[:idx]
    if template:
        head = " ".join(template) + " " + head.lstrip()
    sig = _normalize(head)
    trailing = _trailing_doc(acc[-1])
    doc = list(pending) if pending else ([trailing] if trailing else [])
    nxt = (_skip_body(lines, i, joined, idx, how)
           if how in ("brace", "colon") else i + 1)
    return nxt, sig, doc, start + 1


def _parse_type_body(lines, start, decl, path):
    """Walk a type body from its opening brace; append public members.

    Nested public types recurse into their own TypeDecl (see TypeDecl's
    docstring for why they are not flattened). A nested type in a private or
    protected section is walked too — so its body is skipped correctly — but
    recorded nowhere.
    """
    if decl.kind in ("enum class", "enum"):
        return _parse_enum_body(lines, start, decl, path)

    access = "private" if decl.kind == "class" else "public"
    i = start + 1
    pending = []
    tmpl = []

    while i < len(lines):
        line = lines[i]
        stripped = line.strip()

        if stripped.startswith("}"):
            return i + 1  # the type's own closing brace
        doc = _strip_doc(line)
        if doc is not None:
            pending.append(doc)
            i += 1
            continue
        cont = _continuation(line)
        if cont is not None:
            # A wrapped `///<` belongs to the member it trails, not the next one.
            if decl.members:
                decl.members[-1].doc.append(cont)
            i += 1
            continue
        if not stripped or stripped.startswith(("//", "#")):
            pending = []
            tmpl = []
            i += 1
            continue
        m = re.match(r"^(public|private|protected)\s*:\s*$", stripped)
        if m:
            access = m.group(1)
            pending = []
            tmpl = []
            i += 1
            continue
        if stripped.startswith("template"):
            tmpl.append(stripped)
            i += 1
            continue

        mt = TYPE_OPENER.match(stripped)
        if mt:
            kind = re.sub(r"\s+", " ", mt.group(1))
            head = " ".join(tmpl + [stripped])
            nested = TypeDecl(kind, mt.group(2), pending, i + 1,
                              signature=_normalize(head.split("{", 1)[0]),
                              parent=decl)
            pending = []
            tmpl = []
            i = _parse_type_body(lines, i, nested, path)
            if access == "public":
                decl.nested.append(nested)
            continue
        if FORWARD_DECL.match(stripped):
            pending = []
            tmpl = []
            i += 1
            continue

        i, sig, doc, dline = _read_declaration(lines, i, pending, path, tmpl)
        tmpl = []

        if access == "public" and _is_declaration(sig):
            decl.members.append(
                Member(_member_kind(sig, namespace_scope=False),
                       _member_name(sig), sig, doc, dline))

        # A run of `= delete` / `= default` special members shares one comment
        # placed above the run; ANY other declaration consumes it, so the next
        # member needs its own. (Without that second half, adding a member
        # directly below a documented one would inherit its neighbour's doc —
        # the hole this gate exists to close.)
        if not _is_special_defaulted(sig):
            pending = []

    return i


def _member_kind(sig, namespace_scope):
    if sig.startswith("using "):
        return "type alias" if namespace_scope else "alias"
    if _looks_like_function(sig):
        return "free function" if namespace_scope else "function"
    return "constant" if namespace_scope else "field"


def _skip_body(lines, i, joined, idx, how):
    """Advance past an inline function body; return the next line index.

    `idx` is where the declaration ended: the body's `{` when how == "brace", or
    the `:` opening a constructor initializer list when how == "colon".

    DOCS2 REWROTE THIS, and the old version's bug is worth keeping written down
    because it is the shape this whole tool is about. It counted `{` and `}`
    textually from `idx` and stopped as soon as the count balanced with at least
    one brace seen. A constructor initializer list containing a braced value —

        HoldPose(const MotionDeps& deps, double holdFor, const MotionConfig& cfg = {})
            : MoveToPose(deps, math::Pose2d{}, cfg, 0.0,
                         PoseMotionOptions{.capturePoseAtLive = true}) {

    — balances on `math::Pose2d{}` alone, so it declared the body finished on
    the FIRST line of the initializer list and resumed parsing in the middle of
    an expression. Two headers in the tree do that and both failed to parse at
    all once the tool was pointed at them.

    The initializer list is now walked with real depth tracking, and the body's
    opening brace is identified by the one property that separates it from a
    member initializer in this house style: a member initializer brace follows
    its member name directly (`att_{att}`), while the body brace follows
    whitespace or the `)` of the last initializer.
    """
    text = joined
    j = i
    paren = angle = brace = 0
    scan = idx
    in_body = how == "brace"
    if in_body:
        brace = 1
        scan = idx + 1
    while True:
        while scan < len(text):
            ch = text[scan]
            if ch == '"':
                scan = _skip_string(text, scan)
                continue
            if ch == "'":
                m = CHAR_LITERAL.match(text, scan)
                scan = m.end() if m else scan + 1
                continue
            if in_body:
                if ch == "{":
                    brace += 1
                elif ch == "}":
                    brace -= 1
                    if brace == 0:
                        return j + 1
            elif ch == "(":
                paren += 1
            elif ch == ")":
                paren -= 1
            elif ch == "<":
                angle += 1
            elif ch == ">":
                angle = max(0, angle - 1)
            elif ch == "{":
                if (paren == 0 and angle == 0 and brace == 0
                        and (scan == 0 or text[scan - 1] in " \t\n)")):
                    in_body = True
                    brace = 1
                else:
                    brace += 1
            elif ch == "}":
                brace = max(0, brace - 1)
            scan += 1
        j += 1
        if j >= len(lines):
            return j
        text += "\n" + _strip_line_comment(lines[j])


def _skip_string(text, i):
    """Index just past a string literal starting at `i`."""
    i += 1
    while i < len(text):
        if text[i] == "\\":
            i += 2
            continue
        if text[i] == '"':
            return i + 1
        i += 1
    return i


CHAR_LITERAL = re.compile(r"'(\\.|[^'\\])'")


def _find_terminator(text, brace_ends=True):
    """('semi'|'brace'|'colon', index) of the end of a declaration, or (None, -1).

    Ignores anything inside (), <>, {} or a string/character literal, and steps
    over `::`. With brace_ends=False a `{` opens a brace-initializer instead of
    ending the declaration, and only `;` terminates.

    DOCS2 fixed two latent holes here, both of which only bite outside the two
    headers this tool used to be pointed at:

      * brace DEPTH was tracked only at depth 0, so a nested brace inside a
        brace-initializer (`= {{1, 2}}`) closed the initializer early and cut
        the rendered signature in half.
      * character literals were not recognised at all, so a `';'` or a `'{'` in
        an initializer — `inline constexpr char kMagic[4] = {'S','H','B','B'};`
        is one line away from that — would terminate the declaration inside its
        own literal.
    """
    paren = angle = brace = 0
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
            i += 1
            continue
        if ch == "'":
            m = CHAR_LITERAL.match(text, i)
            i += m.end() - m.start() if m else 1
            continue
        if ch == "(":
            paren += 1
        elif ch == ")":
            paren -= 1
        elif ch == "<":
            angle += 1
        elif ch == ">":
            angle = max(0, angle - 1)
        elif ch == "{":
            if brace_ends and paren == 0 and angle == 0 and brace == 0:
                return "brace", i
            brace += 1
        elif ch == "}":
            brace = max(0, brace - 1)
        elif paren == 0 and angle == 0 and brace == 0:
            if ch == ";":
                return "semi", i
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


def _split_top_commas(text):
    """Split on commas outside (), <>, {} and literals — for one-line enums."""
    parts = []
    depth = 0
    start = 0
    i = 0
    while i < len(text):
        ch = text[i]
        if ch in "(<{":
            depth += 1
        elif ch in ")>}":
            depth = max(0, depth - 1)
        elif ch == "'":
            m = CHAR_LITERAL.match(text, i)
            if m:
                i = m.end()
                continue
        elif ch == "," and depth == 0:
            parts.append(text[start:i])
            start = i + 1
        i += 1
    parts.append(text[start:])
    return parts


def _strip_line_comment(line):
    cut = line.find("///<")
    if cut < 0:
        cut = line.find("//")
    return line[:cut] if cut >= 0 else line


def _is_declaration(sig):
    if not sig:
        return False
    # `using` was excluded until DOCS2, which cost the reference the whole
    # units vocabulary: `using Length = Quantity<1,0,0,0,0>` and its ten
    # siblings ARE the F3 contract's public spelling, and `using
    # std::logic_error::logic_error` is why PreconditionError{"msg"} compiles.
    # A using-DIRECTIVE (`using namespace x`) declares nothing and stays out.
    if sig.startswith(("#", "static_assert", "using namespace ")):
        return False
    # A friend DECLARATION grants access to something declared elsewhere and is
    # not this type's surface. A hidden-friend DEFINITION is the opposite: it is
    # found only by ADL on this type, so it is callable API that exists nowhere
    # else — units::Quantity's seven arithmetic operators are all of this shape,
    # and dropping them would leave the units page unable to say that Length
    # supports `+`.
    if re.match(r"^friend\s+(class|struct|union|enum)\b", sig):
        return False
    return True


# `operator` followed by whatever spells it: (), [], a run of punctuation, or a
# user-defined-literal suffix. Needed because the first `(` at depth zero — the
# rule everything else uses — lands INSIDE the name for `operator()`.
OPERATOR_NAME = re.compile(
    r"\boperator\s*(\(\)|\[\]|\"\"[A-Za-z_]\w*|[-+*/%^&|~!<>=,]+"
    r"|\bnew\b\s*(\[\])?|\bdelete\b\s*(\[\])?)")


def _looks_like_function(sig):
    """Does this declaration have a parameter list?

    Two bugs the oracle caught, both of which mislabelled a FIELD as a function
    and then named it after part of its own initializer:

        double integralLimit = std::numeric_limits<double>::infinity();
        units::AngularVelocity headingDriftRate{(1.0 / 60.0) * kPi / 180.0};

    The first has a `(` at depth zero, after an `=`. The second has one inside a
    brace initializer, which was not tracked at all. So a top-level `=` ends the
    search (everything after it is an initializer), and braces are a depth.
    """
    if OPERATOR_NAME.search(sig):
        return True                       # every operator declaration is one
    angle = brace = 0
    for ch in sig:
        if ch == "<":
            angle += 1
        elif ch == ">":
            angle = max(0, angle - 1)
        elif ch == "{":
            brace += 1
        elif ch == "}":
            brace = max(0, brace - 1)
        elif angle == 0 and brace == 0:
            if ch == "=":
                return False
            if ch == "(":
                return True
    return False


ENUMERATOR = re.compile(r"^([A-Za-z_]\w*)\s*(=.*)?$", re.S)


def _parse_enum_body(lines, start, decl, path):
    """Enumerators, in source order, from the opening brace to its match.

    Rewritten at DOCS2 because the line-based version could not see a ONE-LINE
    enum — `enum class Role { Forward, Lateral };` computed a brace depth of
    zero on its first line and returned with no members at all. Two of those
    exist in the tree (TrackingWheel::Role, Localizer::Quality), and a silently
    empty enum is a vacuously green coverage gate: the exact failure this file
    exists to prevent, reproduced inside it.

    Documentation placement, stated because it constrains how enums are written:
    a `///` run above a line documents the FIRST enumerator on it and a `///<`
    documents the LAST. On the house one-enumerator-per-line form those are the
    same enumerator; on a one-liner with several, the rest are undocumented and
    the gate says so — which is the pressure to expand it, and is intended.
    """
    i = start
    bal = 0
    started = False
    pending = []
    while i < len(lines):
        raw = lines[i]
        code = _strip_line_comment(raw)
        body = code.split("{", 1)[1] if (i == start and "{" in code) else code
        bal += code.count("{") - code.count("}")
        if "{" in code:
            started = True
        closing = started and bal <= 0
        if closing:
            body = body.rsplit("}", 1)[0]

        doc = _strip_doc(raw)
        if doc is not None:
            pending.append(doc)
            i += 1
            continue
        cont = _continuation(raw)
        if cont is not None and not body.strip():
            # A wrapped `///<` belongs to the enumerator it trails. Without this
            # it landed in `pending` and became the NEXT enumerator's doc — see
            # _strip_doc, and mechanism_outcome.md, which shipped that way.
            if decl.members:
                decl.members[-1].doc.append(cont)
            i += 1
            if closing:
                break
            continue

        if body.strip():
            trailing = _trailing_doc(raw)
            items = [p.strip() for p in _split_top_commas(body) if p.strip()]
            for k, item in enumerate(items):
                m = ENUMERATOR.match(item)
                if not m:
                    raise SystemExit(
                        f"api_doc_tool: unrecognised enumerator in {path}, "
                        f"line {i + 1} of `{decl.kind} {decl.qualified}`:\n"
                        f"    {item}\n"
                        "Fail loudly rather than drop it — an enumerator missing "
                        "from the reference is a value nobody knows exists.")
                edoc = []
                if k == 0 and pending:
                    edoc = list(pending)
                elif k == len(items) - 1 and trailing:
                    edoc = [trailing]
                decl.members.append(
                    Member("enumerator", m.group(1), _normalize(item), edoc, i + 1))
            pending = []
        elif not raw.strip() or raw.strip().startswith("//"):
            pending = []

        i += 1
        if closing:
            break
    return i


# ── rendering ─────────────────────────────────────────────────────────────────────

# Design commentary longer than this many lines starts folded. Chosen as "about
# a screen": short enough that a folded block is genuinely saving the reader,
# long enough that most headers' commentary still shows without a click.
BANNER_FOLD_LINES = 45

GENERATED_NOTE = (
    "<!-- GENERATED FILE — DO NOT EDIT BY HAND.\n"
    "     Source: {header}\n"
    "     Regenerate: python3 tools/api_doc_tool.py generate\n"
    "     The host test build fails if this file is out of date, so an edit here\n"
    "     is reverted by the next build rather than reviewed. Edit the header. -->\n"
)


def _anchor(text):
    keep = [c.lower() for c in text.replace("::", "-")
            if c.isalnum() or c in " -_"]
    return "".join(keep).strip().replace(" ", "-")


# Operator punctuation, spelled out. _anchor() drops every character that is not
# alphanumeric, so without this `operator+`, `operator+=` and `operator==` all
# reduce to "operator" — they collide, get numbered as if they were overloads of
# each other, and the reference then LABELS them "(overload 2)". That is not a
# broken link, which is why it survived review: it is a page that renders a
# confident, wrong sentence about the API. Found while pointing the tool at
# units/quantity.hpp, which declares seven distinct operators on one type.
OPERATOR_WORDS = {
    "+": "plus", "-": "minus", "*": "star", "/": "slash", "%": "mod",
    "^": "xor", "&": "and", "|": "or", "~": "compl", "!": "not",
    "=": "eq", "<": "lt", ">": "gt", "(": "call", "[": "index",
    ",": "comma", '"': "quote", ")": "", "]": "",
}


def _spell(name):
    """A member name as ASCII words, so distinct spellings get distinct anchors."""
    if name.startswith("~"):
        return "destructor-" + name[1:]
    if not name.startswith("operator"):
        return name
    out = ["operator"]
    buf = ""
    for ch in name[len("operator"):]:
        if ch.isalnum() or ch == "_":
            buf += ch
            continue
        if buf:
            out.append(buf)
            buf = ""
        word = OPERATOR_WORDS.get(ch)
        if word:
            out.append(word)
    if buf:
        out.append(buf)
    return "-".join(out)


def sections_of(entries):
    """Page sections in render order: each type, then its nested types.

    Members render INSIDE their owner's section, so a depth-first walk puts a
    nested type's section immediately after the last member of its parent —
    which is both deterministic (source order throughout) and the order a
    reader expects: the whole of `BlackboxReader`, then `BlackboxReader::Frame`.
    """
    out = []
    for entry in entries:
        if isinstance(entry, TypeDecl):
            out.extend(entry.walk())
        else:
            out.append(entry)
    return out


def assign_anchors(entries):
    """Give every entity a UNIQUE, stable anchor and display label.

    Overloads share a name, so a naive name-derived anchor collides and every
    link to `scheduler` lands on the first one — a reference whose links go to
    the wrong member is worse than one with no links. Numbering follows SOURCE
    order, which is deterministic, so the regeneration check stays usable.

    The `seen` map is PAGE-wide rather than per-type, because since DOCS2 a page
    also carries free functions (`operator""_in` has two overloads on one page)
    and nested types whose members would otherwise number independently.
    """
    seen = {}

    def claim(base):
        n = seen.get(base, 0) + 1
        seen[base] = n
        return (base if n == 1 else f"{base}-{n}"), n

    for section in sections_of(entries):
        if isinstance(section, TypeDecl):
            section.anchor, _ = claim(_anchor(f"{section.kind} {section.qualified}"))
            section.label = section.qualified
            for mem in section.members:
                mem.owner = section
                mem.anchor, n = claim(
                    _anchor(f"{section.qualified}-{_spell(mem.name)}"))
                mem.label = mem.name if n == 1 else f"{mem.name} (overload {n})"
        else:
            section.anchor, n = claim(_anchor(_spell(section.name)))
            section.label = (section.name if n == 1
                             else f"{section.name} (overload {n})")


# A trailing development-process citation — "(chunk C4, WS6/M2)", "(master plan
# §17)". True, useful in the header, and noise in a one-line summary aimed at
# someone who has never heard of chunk C4. Stripped from the BLURB only: the
# banner is reproduced in full further down every page, so nothing is lost, and
# the citation stays where it was written.
PROVENANCE_TAIL = re.compile(
    r"\s*\((?=[^)]*(?:chunk\b|master plan|WS\d))[^)]*\)\s*(?=[.;]?$)")


def blurb_of(banner):
    """The header's own first sentence — the page's and the index's summary.

    Derived, never written down: a hand-written blurb per page would be 115
    hand-maintained sentences about code that moves, which is the failure this
    whole tool exists to remove.
    """
    lead = []
    for line in banner:
        if not line.strip():
            if lead:
                break
            continue
        lead.append(line.strip())
    text = re.sub(r"\s+", " ", " ".join(lead)).strip()
    cut = text.find(". ", 40)
    if cut > 0:
        text = text[:cut + 1]
    text = PROVENANCE_TAIL.sub("", text).strip()
    if text and text[-1] not in ".!?…":
        text += "."
    if len(text) > 260:
        text = text[:257].rstrip() + "…"
    return text or "_(this header carries no design banner)_"


def excluded_shapes(targets):
    """(macro names, headers with a protected section) — DERIVED, not typed.

    The reference's "what is not here" paragraph names both. A hand-typed count
    there is a number that goes stale in a document whose whole purpose is that
    it cannot; this project has been bitten by exactly that often enough that
    deleting a stale count in favour of a derived one is a written standard.
    """
    macros = set()
    protected = []
    for target in targets:
        text = open(os.path.join(REPO, target["header"]), encoding="utf-8").read()
        macros.update(re.findall(r"^#define\s+([A-Z][A-Z0-9_]*)\s*\(", text, re.M))
        if re.search(r"^\s*protected:", text, re.M):
            protected.append(target["header"])
    return sorted(macros), sorted(protected)


def _entity_block(out, header, entity, heading, kindword, shared_with=None):
    out.append(f'<a id="{entity.anchor}"></a>\n')
    out.append(f"{heading}\n")
    if entity.signature:
        out.append("```cpp")
        out.append(entity.signature)
        out.append("```\n")
    if shared_with is not None:
        # A run of `= delete` / `= default` special members shares ONE comment,
        # by design (see _parse_type_body). Rendering that comment six times
        # under six consecutive members is what an interface page looked like
        # until DOCS2: the same paragraph, verbatim, filling the screen above the
        # members a reader came for. The rule is stated once and pointed at
        # instead — which also makes the rule visible, where repetition made it
        # look like an accident.
        anchor, label = shared_with
        out.append(f"*Covered by the comment on [`{label}`](#{anchor}) — one "
                   "comment documents this run of special members.*")
    else:
        out.append(" ".join(entity.doc) if entity.doc else "**UNDOCUMENTED.**")
    out.append("")
    out.append(f"*{kindword}, declared at "
               f"[`{header}:{entity.line}`](../../{header}#L{entity.line}).*\n")


def page_summary(entries):
    """`3 types (28 members), 2 free functions and 1 constant` — one line.

    A reader landing on a page should be able to tell in one glance whether it
    is a two-constant header or a sixty-member scheduler, without scrolling the
    contents list to find out.
    """
    types = members = 0
    free = {}
    for section in sections_of(entries):
        if isinstance(section, TypeDecl):
            types += 1
            members += len(section.members)
        else:
            free[section.kind] = free.get(section.kind, 0) + 1
    parts = []
    if types:
        parts.append(f"**{types}** type{'s' if types != 1 else ''}"
                     + (f" ({members} member{'s' if members != 1 else ''})"
                        if members else " (no public members)"))
    for kind in ("free function", "constant", "type alias"):
        n = free.get(kind, 0)
        if n:
            parts.append(f"**{n}** {kind}{'s' if n != 1 else ''}")
    if not parts:
        return ""
    if len(parts) > 1:
        parts[-1] = "and " + parts[-1]
    return ("This header declares "
            + (", ".join(parts) if len(parts) > 2 else " ".join(parts)) + ".")


def render(target, banner, entries):
    header = target["header"]
    out = [GENERATED_NOTE.format(header=header)]
    out.append(f"# {target['title']}\n")
    out.append(blurb_of(banner) + "\n")
    summary = page_summary(entries)
    if summary:
        out.append(summary + "\n")
    out.append(
        f"Extracted from [`{header}`](../../{header}) — this page **is** that header's "
        "documentation, reformatted, so it cannot disagree with the code. Prose about "
        "*how to think about* the API lives in the "
        "[user guide](../guide/README.md); worked recipes live in the "
        "[cookbook](../cookbook/README.md); this page is the complete, mechanical "
        "list of what exists.\n")

    sections = sections_of(entries)
    if not sections:
        out.append("This header declares no public types, functions or constants "
                   "of its own — it exists for the design commentary below, for "
                   "the includes it gathers, or for preprocessor macros (which "
                   "this tool does not extract; see the "
                   "[reference overview](README.md)).\n")
    else:
        out.append("## Contents\n")
        for section in sections:
            if isinstance(section, TypeDecl):
                pad = "  " * section.depth
                out.append(f"{pad}- [`{section.kind} {section.qualified}`]"
                           f"(#{section.anchor})")
                for mem in section.members:
                    out.append(f"{pad}  - [`{mem.label}`](#{mem.anchor})")
            else:
                out.append(f"- [`{section.label}`](#{section.anchor}) "
                           f"— *{section.kind}*")
        out.append("")

    for section in sections:
        if isinstance(section, TypeDecl):
            _entity_block(out, header, section,
                          f"## `{section.kind} {section.qualified}`",
                          section.kind)
            if not section.members and not section.nested:
                out.append("_No public members._\n")
            owner_of_doc = None       # (anchor, label) of the run's first member
            for prev, mem in zip([None] + section.members, section.members):
                shared = None
                if (prev is not None and mem.doc and prev.doc == mem.doc
                        and _is_special_defaulted(mem.signature)):
                    owner_of_doc = owner_of_doc or (prev.anchor, prev.label)
                    shared = owner_of_doc
                else:
                    owner_of_doc = None
                _entity_block(out, header, mem,
                              f"### `{section.qualified}::{mem.label}`", mem.kind,
                              shared_with=shared)
        else:
            _entity_block(out, header, section, f"## `{section.label}`",
                          section.kind)

    out.append("## Design commentary, from the header\n")
    out.append(
        "The header opens with the reasoning behind these shapes. It is reproduced "
        "here in full because a reference that only lists signatures teaches nobody "
        "*why*.\n")
    # Folded, because "in full" is sometimes two hundred lines and a wall of them
    # under every page is how a reference stops being read. Short commentary
    # starts open so the common case costs no click; the threshold is stated
    # rather than tuned, and either way nothing is hidden — the summary line says
    # how much there is.
    fold = len(banner) > BANNER_FOLD_LINES
    out.append(f'<details markdown="1"{"" if fold else " open"}>')
    out.append(f"<summary>The header’s own reasoning — {len(banner)} lines"
               f"{', click to expand' if fold else ''}</summary>\n")
    out.append("```text")
    out.extend(banner)
    out.append("```\n")
    out.append("</details>")
    return "\n".join(out).rstrip() + "\n"


NAV_BEGIN = ("# BEGIN GENERATED API NAV — regenerate with: "
             "python3 tools/api_doc_tool.py generate")
NAV_END = "# END GENERATED API NAV"


def render_nav(pages):
    """The `API reference` nav section, generated into mkdocs.yml.

    WHY THIS IS GENERATED (chunk DOCS2)
    -----------------------------------
    Measured before deciding: a page that exists under docs/ but is absent from
    mkdocs.yml's nav does NOT fail the strict build. It emits an INFO line and
    exits 0 — so the page ships, unreachable, and nothing anywhere says so. With
    two pages that was survivable by hand. With a page per header it is a
    certainty, and it is the failure this project fears most: documentation that
    exists, passes every gate, and no reader can reach.

    So the nav block is derived from the same target list as the pages, spliced
    between markers, and compared byte-for-byte by check-fresh. A page that
    cannot be reached now fails the build instead of shipping quietly.
    """
    out = [NAV_BEGIN,
           "  - API reference:",
           # A bare path: with `navigation.indexes` this page IS the section, so
           # clicking "API reference" lands on the page that explains it.
           "      - api/README.md",
           f"      - All entities (A–Z): api/{os.path.basename(ALL_OUT)}"]
    for label, group in grouped_pages(pages):
        out.append(f"      - {label}:")
        for target, _banner, _entries in group:
            rel = os.path.basename(target["out"])
            out.append(f"          - {target['nav_label']}: api/{rel}")
    out.append(NAV_END)
    return "\n".join(out)


def splice_nav(text, block):
    """Replace the marked nav region of mkdocs.yml with `block`."""
    start = text.find(NAV_BEGIN)
    end = text.find(NAV_END)
    if start < 0 or end < 0:
        raise SystemExit(
            f"api_doc_tool: {MKDOCS} has no generated-nav markers.\n"
            f"  Expected a region delimited by:\n    {NAV_BEGIN}\n    {NAV_END}\n"
            "Without them a generated page can ship unreachable — see "
            "render_nav().")
    return text[:start] + block + text[end + len(NAV_END):]


def render_index(pages):
    macros, protected = excluded_shapes([t for t, _b, _e in pages])
    macro_list = ", ".join(f"`{m}`" for m in macros) or "none in the tree today"
    if not protected:
        protected_list = "none in the tree today"
    elif len(protected) == 1:
        protected_list = ("one section in the tree, in "
                          f"`{protected[0][len(API_ROOT) + 1:]}`")
    else:
        protected_list = (f"{len(protected)} sections in the tree, in "
                          + ", ".join(f"`{p[len(API_ROOT) + 1:]}`"
                                      for p in protected))
    total_entities = 0
    total_undocumented = 0
    for _target, _banner, entries in pages:
        for section in sections_of(entries):
            total_entities += 1
            if not section.documented:
                total_undocumented += 1
            for mem in getattr(section, "members", []):
                total_entities += 1
                if not mem.documented:
                    total_undocumented += 1

    out = [
        "<!-- GENERATED FILE — DO NOT EDIT BY HAND.\n"
        "     Regenerate: python3 tools/api_doc_tool.py generate -->\n",
        "# API reference\n",
        # A reference this size is intimidating on arrival, and most readers do
        # not need most of it. So the first thing on the page is the short
        # answer, before the scale is mentioned at all.
        "> **Writing an autonomous routine? You need two of these pages.**\n"
        "> [`Chassis`](chassis.md) is the facade every routine is written "
        "against, and [`Routine`](routine.md) is the fluent recipe layer on top "
        "of it. Everything else on this page is the machinery underneath — real, "
        "documented, and safe to ignore until you want it.\n",
        f"**Every public entity in every shipped header** — {total_entities:,} "
        f"of them across {len(pages)} headers: types and their members, nested "
        "types, free functions, namespace-scope constants and type aliases. "
        "Extracted from the headers, so it cannot fall behind the code: anything "
        "added to a shipped header appears here the next time the tool runs, and "
        "the host test build fails if it has not.\n",
        "**A public entity with no documentation comment fails the build**, naming "
        "itself and its file and line. That gate is what makes \"generated\" mean "
        "\"complete\" rather than \"generated from whatever someone remembered to "
        "write\".\n",
        # ── The scope paragraph. Read the two comments below before editing it. ──
        #
        # DOCS1 wrote this paragraph twice and got it wrong the first time: it
        # said the page "covers the surfaces that are frozen", which the repo
        # flatly contradicted (SEVEN contracts are LOCKED; two were on the page).
        # The lesson recorded then was that a tidy rationale the repo contradicts
        # is worse than an ugly true one.
        #
        # DOCS2 changed the underlying fact — the page now covers everything — so
        # the paragraph is rewritten again, and the trap is the same one facing
        # the other direction: it is now very easy to write "the reference is
        # complete" and be wrong. It is not complete. Four things are absent,
        # every one of them listed below with its reason, and the last two are
        # absent from the MEMBER LISTS while still being visible in the design
        # commentary each page reproduces. Nothing here claims a property that
        # is not mechanically true.
        "**What is not here, and why.** Four exclusions, all deliberate:\n",
        "- **`include/shulib/sim/`** — the host simulator. Test-only, and not by "
        "convention: a CI guard fails the build if anything outside `sim/` "
        "includes it, so no robot binary can reach it.\n"
        "- **`hal/fake/` and `localization/fake/`** — the test doubles the suite "
        "drives the real seams with. Public by file placement, test fixtures by "
        "charter; `test/README.md` is their documentation.\n"
        f"- **Preprocessor macros** ({macro_list}). A macro has no signature, no "
        "access and no type, so there is nothing for an extractor to render "
        "without inventing it. Each is explained at length in its own header's "
        "design commentary, which every page below reproduces in full — so they "
        "are on the site, in prose, but not in the member lists or the index.\n"
        f"- **`protected` members** — {protected_list}. This reference documents "
        "the surface you *call*; the surface you *subclass* is "
        "[guide chapter 13](../guide/13-extending-the-library.md)'s subject.\n",
        "**Being on this page does not freeze anything.** Most of what follows is "
        "unfrozen and expected to move. The Freeze Register in the "
        "[roadmap](../roadmap.md) is the only place a contract is locked, and it "
        "is enforced by compile-time signature pins, not by this page: changing a "
        "frozen signature fails a C++ test that names the register row, while "
        "changing anything else here costs one `///` edit and a regeneration. "
        "Those are different mechanisms and only the first is a promise.\n",
        "**What the gate does not check.** It proves every entity *has* a "
        "documentation comment. It has no opinion about whether that comment "
        "says anything — `/// Sets the voltage.` on `setVoltage` passes. Only a "
        "reader catches that, which is why the headers are written to be read and "
        "why each page below ends with the header's own design commentary rather "
        "than a bare list of signatures.\n",
        "Prose about *how to think about* the API lives in the "
        "[user guide](../guide/README.md) — chapter 10 is the API as prose, and "
        "deliberately does not restate signatures. Worked recipes live in the "
        "[cookbook](../cookbook/README.md). This page answers \"what exactly "
        "exists, and what is its exact spelling\".\n",
        "## Pages\n",
    ]
    for label, group in grouped_pages(pages):
        out.append(f"### {label}\n")
        out.append("| Page | Header | What it is |")
        out.append("|---|---|---|")
        for target, banner, _entries in group:
            rel = os.path.basename(target["out"])
            header = target["header"]
            out.append(f"| [{target['nav_label']}]({rel}) "
                       f"| [`{header[len(API_ROOT) + 1:]}`](../../{header}) "
                       f"| {blurb_of(banner).replace('|', chr(92) + '|')} |")
        out.append("")

    out.append("## Every public entity, alphabetically\n")
    out.append(
        f"**[The alphabetical index]({os.path.basename(ALL_OUT)})** lists all "
        f"{total_entities:,} of them with a link to each. Nested types appear "
        "under their qualified name (`BlackboxReader::Frame::type`), so a "
        "member of a nested type is findable by the name you would actually "
        "write.\n")
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


def index_rows(pages):
    """(qualified name, kind, page-relative link) for every public entity."""
    rows = []
    for target, _banner, entries in pages:
        rel = os.path.basename(target["out"])
        for section in sections_of(entries):
            if isinstance(section, TypeDecl):
                rows.append((section.qualified, section.kind,
                             f"[{rel}]({rel}#{section.anchor})"))
                for mem in section.members:
                    rows.append((f"{section.qualified}::{mem.label}", mem.kind,
                                 f"[{rel}]({rel}#{mem.anchor})"))
            else:
                rows.append((section.label, section.kind,
                             f"[{rel}]({rel}#{section.anchor})"))
    return sorted(rows, key=lambda r: (r[0].lower(), r[1], r[2]))


def render_all(pages):
    rows = index_rows(pages)
    out = [
        "<!-- GENERATED FILE — DO NOT EDIT BY HAND.\n"
        "     Regenerate: python3 tools/api_doc_tool.py generate -->\n",
        "# Every public entity, alphabetically\n",
        f"All {len(rows):,} of them, across {len(pages)} shipped headers: types, "
        "their members, nested types and their members, free functions, "
        "namespace-scope constants and type aliases. Generated from the headers "
        "by the same parse that produces the pages, so a name missing here is a "
        "name missing everywhere — which is why the build fails if this file is "
        "not byte-identical to a fresh run.\n",
        "Nested types appear under their qualified name "
        "(`BlackboxReader::Frame::type`), so a member of a nested type is "
        "findable by the name you would actually write. Overloads are numbered "
        "in source order and each has its own link.\n",
        "The [reference overview](README.md) says what is deliberately *not* "
        "here, and why.\n",
    ]
    # Split by initial letter, with a jump bar. Sixteen hundred rows in one
    # table is a scroll, not an index; the bar is what makes it usable on a
    # phone and what makes Ctrl-F unnecessary.
    groups = {}
    for name, kind, link in rows:
        first = name.lstrip("~_").upper()[:1]
        groups.setdefault(first if first.isalpha() else "#", []).append(
            (name, kind, link))
    letters = sorted(groups, key=lambda c: (c == "#", c))
    out.append(" · ".join(f"[{c}](#{_anchor(c) or 'other'})" for c in letters) + "\n")
    for letter in letters:
        out.append(f"## {letter if letter != '#' else 'Other'}\n")
        out.append("| Name | Kind | Page |")
        out.append("|---|---|---|")
        for name, kind, link in groups[letter]:
            out.append(f"| `{name.replace('|', chr(92) + '|')}` | {kind} | {link} |")
        out.append("")
    return "\n".join(out).rstrip() + "\n"


# ── the jobs ──────────────────────────────────────────────────────────────────────

def build_pages():
    pages = []
    for target in build_targets():
        banner, entries = parse_header(target["header"])
        assign_anchors(entries)
        pages.append((target, banner, entries))
    return pages


def do_generate(outroot=None):
    root = outroot or REPO
    pages = build_pages()
    written = []
    for target, banner, entries in pages:
        path = os.path.join(root, target["out"])
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(render(target, banner, entries))
        written.append(target["out"])
    for rel, text in ((INDEX_OUT, render_index(pages)),
                      (ALL_OUT, render_all(pages))):
        path = os.path.join(root, rel)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(text)
        written.append(rel)

    # The nav is part of the generated output, not a thing to remember (see
    # render_nav). The source mkdocs.yml is always the input, so generating into
    # a scratch tree cannot drift from generating in place.
    source = open(os.path.join(REPO, MKDOCS), encoding="utf-8").read()
    path = os.path.join(root, MKDOCS)
    os.makedirs(os.path.dirname(path) or root, exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(splice_nav(source, render_nav(pages)))
    written.append(MKDOCS)
    return written


def do_check_coverage(only=None):
    """Report every public entity with no /// documentation.

    `only` narrows the report to one header — the whole tree is 115 headers, and
    someone documenting one of them needs the list for that one, not a wall.
    It narrows the REPORT, never the gate: the build always runs it unfiltered.
    """
    missing = []
    pages = build_pages()
    if only:
        only = only.replace(os.sep, "/")
        pages = [p for p in pages if p[0]["header"].endswith(only)]
        if not pages:
            raise SystemExit(f"api_doc_tool: no target header matches {only!r}")
    for target, _banner, entries in pages:
        for section in sections_of(entries):
            if not section.documented:
                what = (f"{section.kind} {section.qualified}"
                        if isinstance(section, TypeDecl) else section.name)
                missing.append((target["header"], section.line, what,
                                section.signature or what))
            for mem in getattr(section, "members", []):
                if not mem.documented:
                    missing.append((target["header"], mem.line,
                                    f"{section.qualified}::{mem.name}",
                                    mem.signature))
    if not missing:
        total = sum(1 for _t, _b, e in pages for s in sections_of(e)
                    for _ in [s] + list(getattr(s, "members", [])))
        print(f"doc coverage: {total} public entities across {len(pages)} "
              "headers, all documented")
        return 0
    print("", file=sys.stderr)
    print(f"DOC COVERAGE FAILURE — {len(missing)} public entities have no /// "
          "documentation.", file=sys.stderr)
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
        # generated set is the whole directory's contents, by definition. So is
        # a page LEFT BEHIND by a header that was renamed or deleted, which is
        # the failure that arrives with a page-per-header reference.
        apidir = os.path.join(REPO, "docs", "api")
        expected = {os.path.basename(r) for r in written if r.startswith("docs/api/")}
        if os.path.isdir(apidir):
            for name in sorted(os.listdir(apidir)):
                if name not in expected:
                    stale.append((f"docs/api/{name}",
                                  ["(not produced by the generator — docs/api/ is "
                                   "generated in full; delete it, or if its header "
                                   "still exists, work out why the tool no longer "
                                   "produces it)"]))
        if not stale:
            return 0
        print("", file=sys.stderr)
        print("API REFERENCE IS STALE — docs/api/ (or mkdocs.yml's generated "
              "nav) does not", file=sys.stderr)
        print("match the headers.", file=sys.stderr)
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
#
# DOCS1 FIND: this pattern and prepare_site.py's screen had DRIFTED APART. That
# script (the publish-time gate) also refuses `-COMPLETED.md` and `-PROGRESS.md`;
# this one — the BUILD-time gate — did not. So `docs/roadmap.md` carried a bare
# `E3-COMPLETED.md` through a green build and would have failed the release at
# the last step, which is the one place a failure is most expensive and least
# expected. Two gates screening for "the same" property with two different lists
# is D3's "a gate's exclusion list is where its holes live", in its other form:
# the hole is not in one list, it is in the GAP between two. The terms are
# unified here; the self-test below pins that this gate catches every term
# prepare_site.py does, so they cannot drift apart again silently.
#
# The self-test found a SECOND gap the moment it was written, which is the
# argument for writing it: prepare_site.py screens the bare string
# `docs/internal` with no trailing slash, and `internal/` does not match it. A
# public sentence reading "docs/internal is dropped at release" passed here and
# failed there. Both spellings are covered now.
#
# THIRD gap, found the same session by reading rather than by either gate: five
# public documents cited completion records WITHOUT the extension —
# "C2-COMPLETED §Mutations", "A3-COMPLETED §3.7". Requiring `\.md` missed every
# one, and so does prepare_site.py, so those would have published as citations
# to documents that do not exist on the site. The property being protected is
# "no public document sends a reader into the development log", and a bare
# `C2-COMPLETED` does exactly that. The extension is dropped from the pattern:
# this gate is deliberately STRICTER than the publish-time screen, which the
# self-test permits (it proves coverage of that screen, not equality with it).
REMOVABILITY_TERMS = re.compile(
    r"docs/internal|internal/|chunks/|RESUMING|build-order"
    r"|-COMPLETED|-PROGRESS")


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

/// A namespace-scope constant, whose initializer contains a `;` and a `{`
/// inside CHARACTER LITERALS — the shape that used to terminate a declaration
/// inside its own literal.
inline constexpr char kPunctuation[3] = {';', '{', '}'};

/// The UNBALANCED version of the same shape, and the one that actually proves
/// the point: the three literals above happen to balance, so a parser blind to
/// character literals still lands in the right place by luck. A lone `';'` does
/// not — it terminates the declaration inside its own value.
inline constexpr char kSemicolon = ';';

/// And a lone `'{'`, which opens a brace depth that never closes.
inline constexpr char kOpenBrace = '{';

/// A namespace-scope type alias — the units vocabulary's shape.
using Count = int;

/// A free function at namespace scope.
inline int freeHelper(int a, int b = 2) { return a + b; }

/// A templated free function, whose template head is part of how you call it.
template <typename T>
inline T freeTemplate(T v) { return v; }

/// Two overloads of one operator, plus a different operator, all of which
/// reduce to the same characters once punctuation is stripped.
constexpr Count operator""_ct(unsigned long long v) { return static_cast<Count>(v); }

namespace detail {
/// An internal helper: PRESENT in the header, ABSENT from the reference.
inline int internalOnly() { return 0; }
}  // namespace detail

/// A documented enum.
enum class Colour {
    Red,    ///< the documented one
    Green,
};

/// An enum with an explicit underlying type — invisible to the pre-DOCS2 opener.
enum class Sized : unsigned char {
    /// The documented one.
    First = 1,
    Second = 2,
};

/// An enum documented in the house TRAILING style, with a WRAPPED comment.
/// A `///<` continuation also starts with `///`, so it used to be read as a
/// leading comment for the NEXT enumerator — three enumerators in the shipped
/// tree carried a confident sentence about a different value, and the coverage
/// gate scored all three as documented.
enum class Wrapped {
    Alpha = 0,   ///< the first one, whose sentence runs long enough that it
                 ///< wraps onto a continuation line
    Beta = 1,    ///< the second one, which must NOT inherit Alpha's tail
};

struct Forward;

/// A documented struct.
struct Thing {
    /// Documented field.
    int a = 0;
    int b = 0;  ///< documented by a trailing comment whose sentence runs long
                ///< enough to wrap onto a continuation line
    int c = 0;
    ///
    int d = 0;

    void undocumentedFn() const {}
};

/// A derived type, whose base-class list hid it from the pre-DOCS2 opener.
struct Derived : Thing {
    /// A member of the derived type.
    int extra = 0;
};

/// FIELDS WHOSE INITIALIZERS CONTAIN A `(`. Both were mislabelled as functions
/// and then named after part of their own initializer, in the shipped tree,
/// until an independent parse found them. The rule that separates them from a
/// real declaration is that their `(` follows a top-level `=`, or sits inside a
/// brace initializer.
struct Initializers {
    /// A field initialized by CALLING something.
    double ceiling = defaultCeiling();
    /// A field whose BRACE initializer contains a parenthesised expression.
    double scaled{(3.0 + 4.0) * 2.0};
    /// A genuine function, for contrast: the `(` here is a parameter list.
    double scale(double by) const { return scaled * by; }
};

/// A documented class.
class Widget {
public:
    /// A PUBLIC nested type. Its own members must survive; flattening it into
    /// Widget's member list would lose them, which is the failure the tool used
    /// to refuse nested types outright to avoid.
    struct Slot {
        /// A field of the nested type.
        int index = 0;
        /// A ONE-LINE nested enum: no brace depth crosses a line boundary.
        enum class Side { Left, Right };
    };

    /// A public member alias.
    using Size = std::size_t;

    /// The constructor. Its initializer list contains a BRACED value, which the
    /// pre-DOCS2 body-skipper counted as the function body opening and closing.
    explicit Widget(int seed) noexcept : slot_{Slot{seed}}, seed_{seed} {}

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

    /// Three operators whose punctuation collapses to the same anchor unless it
    /// is spelled out.
    Widget& operator+=(int n) { seed_ += n; return *this; }
    bool operator==(const Widget& o) const noexcept { return seed_ == o.seed_; }
    int operator[](int n) const noexcept { return seed_ + n; }

protected:
    /// The subclass-extension surface, deliberately NOT in the reference.
    int protectedHook() const { return seed_; }

private:
    /// A PRIVATE nested type: it and its members must not appear anywhere.
    struct Secret {
        int leaked = 0;
    };

    int hidden() const { return seed_; }
    Slot slot_;
    int seed_;
};

}  // namespace fixture
'''

NAV_FIXTURE = """site_name: demo
nav:
  - Home: index.md
# BEGIN GENERATED API NAV — regenerate with: python3 tools/api_doc_tool.py generate
  - API reference:
      - Overview: api/README.md
# END GENERATED API NAV
  - FAQ: faq.md
"""


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
        banner, entries = _selftest_parse(tmp)
        types = [e for e in entries if isinstance(e, TypeDecl)]
        frees = [e for e in entries if not isinstance(e, TypeDecl)]

        # Bug caught: the parser silently finding no types (which would make the
        # coverage gate vacuously green — the worst possible failure here).
        names = [t.name for t in types]
        check(names == ["Colour", "Sized", "Wrapped", "Thing", "Derived",
                        "Initializers", "Widget"], f"types found: {names}")
        by = {t.name: t for t in types}

        # Bug caught: FOUND IN THE SHIPPED TREE by a reader, not by any gate. A
        # `///<` continuation line also starts with `///`, so it was read as a
        # LEADING comment and became the next member's documentation —
        # docs/api/mechanism_outcome.md published three enumerators each
        # carrying a sentence about a different value, and the coverage gate
        # scored all three as documented because the text was non-empty. The
        # tool reproducing its own headline failure mode is the worst outcome
        # available here, so both shapes (enumerator and field) are pinned.
        wrapped = {m.name: " ".join(m.doc) for m in by["Wrapped"].members}
        check(wrapped.get("Alpha", "").endswith("continuation line"),
              f"a wrapped ///< stays with its own enumerator: {wrapped}")
        check(wrapped.get("Beta") ==
              "the second one, which must NOT inherit Alpha's tail",
              f"the NEXT enumerator does not inherit the continuation: {wrapped}")

        # ── DOCS2 shapes: everything below was INVISIBLE before this chunk ──

        # Bug caught: a type with a base-class list, or an enum with an explicit
        # underlying type, never reaching the parser at all. Fifty-nine
        # definitions in the real tree were in this hole, including every
        # concrete implementation class and three LOCKED contracts.
        check("Derived" in names, "a type with a base-class list must be seen")
        check("Sized" in names, "an enum with an underlying type must be seen")
        by = {t.name: t for t in types}
        check(by["Sized"].signature == "enum class Sized : unsigned char",
              f"underlying type kept in the rendered head: {by['Sized'].signature!r}")
        check(by["Derived"].signature == "struct Derived : Thing",
              f"base list kept in the rendered head: {by['Derived'].signature!r}")
        check([m.name for m in by["Sized"].members] == ["First", "Second"],
              "enumerators of an underlying-typed enum")
        check(by["Sized"].members[0].documented and
              not by["Sized"].members[1].documented,
              "the /// run documents the first enumerator only")

        # Bug caught: a FORWARD declaration parsed as a field named `Forward`,
        # which would put a member in the reference that does not exist.
        check("Forward" not in names, "a forward declaration is not a type entry")
        check(all(m.name != "Forward" for m in frees),
              "a forward declaration is not a namespace-scope member")

        # Bug caught: namespace-scope declarations invisible — 85 free functions,
        # 32 constants and 11 aliases in the real tree, including all of
        # spec/accuracy.hpp and all of version.hpp.
        fnames = [m.name for m in frees]
        check(fnames == ["kPunctuation", "kSemicolon", "kOpenBrace", "Count",
                         "freeHelper", "freeTemplate", 'operator""_ct'],
              f"namespace-scope entities: {fnames}")
        fby = {m.name: m for m in frees}
        check(fby["kPunctuation"].kind == "constant", "constant kind")
        # Bug caught: FOUND BY MUTATION (DOCS2 campaign M3, which stayed GREEN on
        # the first fixture). `kPunctuation`'s three literals happen to BALANCE,
        # so a parser blind to character literals lands in the right place by
        # luck and the mutation survives. These two do not balance: without
        # literal handling the first terminates inside its own value and the
        # second opens a brace depth that never closes.
        check(fby["kSemicolon"].signature == "inline constexpr char kSemicolon = ';'",
              f"a `;` inside a char literal is not a terminator: "
              f"{fby['kSemicolon'].signature!r}")
        check(fby["kOpenBrace"].signature == "inline constexpr char kOpenBrace = '{'",
              f"a `{{` inside a char literal is not a brace: "
              f"{fby['kOpenBrace'].signature!r}")
        check(fby["Count"].kind == "type alias", "type-alias kind")
        check(fby["freeHelper"].kind == "free function", "free-function kind")
        # Bug caught: a `;` or `{` inside a CHARACTER literal terminating the
        # declaration inside its own initializer, cutting the rendered value in
        # half — silently, because the truncation is still valid markdown.
        check(fby["kPunctuation"].signature ==
              "inline constexpr char kPunctuation[3] = {';', '{', '}'}",
              f"char-literal punctuation survives: {fby['kPunctuation'].signature!r}")
        check(fby["freeTemplate"].signature ==
              "template <typename T> inline T freeTemplate(T v)",
              f"free template head kept: {fby['freeTemplate'].signature!r}")

        # Bug caught: `namespace detail` contents published as public API.
        check(all("internalOnly" not in m.name for m in frees),
              "namespace detail must not reach the reference")

        # Bug caught: FOUND BY MUTATION (DOCS2 campaign M10 and M11, both GREEN
        # on the first fixture) and, before that, by diffing this parser against
        # clang's AST over the real tree. A field whose INITIALIZER contains a
        # `(` was classified as a function and then named after part of that
        # initializer — `AxisGains::integralLimit` rendered as
        # `AxisGains::std::numeric_limits<double>::infinity`. The self-test could
        # not see it because no fixture field had a call or a parenthesised
        # brace-initializer. Both shapes are here now, with a real function
        # beside them so the rule cannot be satisfied by calling everything a
        # field.
        init = {m.name: m for m in by["Initializers"].members}
        check(set(init) == {"ceiling", "scaled", "scale"}, f"Initializers: {set(init)}")
        check(init["ceiling"].kind == "field",
              f"a field initialized by a CALL is a field: {init['ceiling'].kind}")
        check(init["ceiling"].signature == "double ceiling = defaultCeiling()",
              f"…and keeps its initializer: {init['ceiling'].signature!r}")
        check(init["scaled"].kind == "field",
              f"a field with a parenthesised brace initializer is a field: "
              f"{init['scaled'].kind}")
        check(init["scaled"].signature == "double scaled{(3.0 + 4.0) * 2.0}",
              f"…and keeps it whole: {init['scaled'].signature!r}")
        check(init["scale"].kind == "function",
              "a real parameter list is still a function")

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
        check(" ".join(thing["b"].doc).endswith("continuation line"),
              f"a wrapped ///< on a FIELD stays with it: {thing['b'].doc}")
        check(not thing["c"].documented,
              "…and does not become the next field's documentation")
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
        check(wnames == ["Size", "Widget", "Widget", "operator=", "~Widget",
                         "leaks", "query", "query", "count", "operator+=",
                         "operator==", "operator[]"],
              f"Widget members: {wnames}")

        # Bug caught: private members leaking into the public reference.
        check("hidden" not in wnames, "private members must not appear")
        # Bug caught: `protected` — the SUBCLASS surface — rendered as though it
        # were callable API. It is a stated exclusion, so it must be tested.
        check("protectedHook" not in wnames, "protected members must not appear")

        # ── nested public types (the blocker this chunk was called for) ──
        # Bug caught: the flattening shortcut. `Slot` as a member of Widget
        # would put a bare name in the member list and DROP Slot's own two
        # members entirely — the failure the old SystemExit existed to refuse.
        check([n.name for n in by["Widget"].nested] == ["Slot"],
              f"nested types of Widget: {[n.name for n in by['Widget'].nested]}")
        slot = by["Widget"].nested[0]
        check(slot.qualified == "Widget::Slot", f"qualified: {slot.qualified}")
        check([m.name for m in slot.members] == ["index"],
              f"Slot members: {[m.name for m in slot.members]}")
        check([n.name for n in slot.nested] == ["Side"], "a type nested TWO deep")
        side = slot.nested[0]
        check(side.qualified == "Widget::Slot::Side", f"qualified: {side.qualified}")
        # Bug caught: a ONE-LINE enum returning zero enumerators, which makes the
        # coverage gate vacuously green for it. Two exist in the real tree.
        check([m.name for m in side.members] == ["Left", "Right"],
              f"one-line nested enum: {[m.name for m in side.members]}")
        check(not any(m.documented for m in side.members),
              "undocumented one-line enumerators are reported as undocumented")
        # Bug caught: a PRIVATE nested type, or its members, reaching the page.
        check(all(n.name != "Secret" for n in by["Widget"].nested),
              "a private nested type must not appear")
        check(all(m.name != "leaked" for n in by["Widget"].nested
                  for m in n.members), "a private nested type's members must not appear")

        # Bug caught: a public member `using` alias dropped. This cost the real
        # reference the entire units vocabulary.
        check(widget[0].kind == "alias" and widget[0].documented,
              "a public member alias is a documented member")

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

        # Bug caught: the constructor's INITIALIZER LIST glued onto the rendered
        # signature — and, since DOCS2, the far worse version: an initializer
        # containing a braced value (`slot_{Slot{seed}}`) read as the function
        # body, so parsing resumed mid-expression and the rest of the type was
        # lost. Two real headers failed to parse at all on exactly that.
        ctor = [m.signature for m in wby["Widget"]][0]
        check(ctor == "explicit Widget(int seed) noexcept",
              f"ctor rendered without its init list: {ctor!r}")
        check(wby["leaks"][0].signature == "int leaks() const",
              "parsing resumed at the right place after a braced initializer list")

        # Bug caught: the header's design commentary silently dropped, leaving a
        # reference of signatures with no reasoning.
        check(banner[:2] == ["", " Fixture banner line one."], f"banner: {banner[:2]}")

        # ── anchors ──
        assign_anchors(entries)
        anchors = {}
        for section in sections_of(entries):
            anchors.setdefault(section.anchor, []).append(section.label)
            for mem in getattr(section, "members", []):
                anchors.setdefault(mem.anchor, []).append(mem.label)
        dupes = {a: labels for a, labels in anchors.items() if len(labels) > 1}
        # Bug caught: two entities sharing an anchor, so every link to one lands
        # on the other. A reference whose links go to the wrong member is worse
        # than one with no links at all.
        check(not dupes, f"anchors are unique: {dupes}")
        # Bug caught: operator punctuation stripped to nothing, so `operator+=`,
        # `operator==` and `operator[]` collide, get numbered against each other
        # and are then LABELLED "(overload 2)" — a confident, wrong sentence
        # about the API rather than a broken link. Found on units/quantity.hpp.
        wa = {m.name: (m.anchor, m.label) for m in by["Widget"].members}
        check(wa["operator+="][0] == "widget-operator-plus-eq", f"{wa['operator+=']}")
        check(wa["operator=="][0] == "widget-operator-eq-eq", f"{wa['operator==']}")
        check(wa["operator[]"][0] == "widget-operator-index", f"{wa['operator[]']}")
        check(wa["operator+="][1] == "operator+=",
              f"a distinct operator must not be labelled an overload: {wa['operator+=']}")
        # Nested-type members must reach the alphabetical index under their
        # qualified name — the DoD's explicit requirement.
        index_names = {f"{s.qualified}::{m.label}"
                       for s in sections_of(entries) if isinstance(s, TypeDecl)
                       for m in s.members}
        check("Widget::Slot::index" in index_names,
              "a nested type's member is indexed under its qualified name")
        check("Widget::Slot::Side::Left" in index_names,
              "a doubly-nested enumerator is indexed under its qualified name")

        # Bug caught: NON-DETERMINISM — without this, the regeneration check in
        # the build would fail at random and get switched off.
        fixture_target = {"header": "fixture.hpp", "out": "docs/api/fixture.md",
                          "subsystem": "", "title": "`fixture.hpp`",
                          "nav_label": "Fixture"}
        first = render(fixture_target, banner, entries)
        second = render(fixture_target, banner, entries)
        check(first == second, "render is deterministic")
        # Bug caught: the shared-comment collapse dropping the comment
        # ALTOGETHER (rendering the pointer on the first member of the run too,
        # so the text appears nowhere), or applying to members that merely
        # happen to be adjacent. The fixture's Widget has a three-member
        # `= delete` / `= default` run under one comment.
        check(first.count("Not copyable: two handles would fork the state.") == 1,
              "a shared special-member comment is rendered exactly once")
        check(first.count("one comment documents this run of special members") == 2,
              "…and the other two members of the run point at it")
        check("Documented, with a default argument" in first,
              "a normal member's own comment is untouched by the collapse")

        # Every entity must actually reach the page. A generator that parses a
        # member and then forgets to render it is the same silent omission one
        # layer later.
        for section in sections_of(entries):
            check(f'<a id="{section.anchor}"></a>' in first,
                  f"rendered: {section.label}")
            for mem in getattr(section, "members", []):
                check(f'<a id="{mem.anchor}"></a>' in first,
                      f"rendered: {section.label}::{mem.label}")

        one = do_generate(outroot=os.path.join(tmp, "a"))
        two = do_generate(outroot=os.path.join(tmp, "b"))
        check(one == two, "generate writes the same file set twice")
        for rel in one:
            a = open(os.path.join(tmp, "a", rel), encoding="utf-8").read()
            b = open(os.path.join(tmp, "b", rel), encoding="utf-8").read()
            check(a == b, f"{rel} is byte-identical across runs")

        # ── every generated page is reachable (DOCS2) ──
        # Bug caught: a page that exists, passes every gate and appears in no
        # nav — measured to emit INFO and exit 0 under `mkdocs build --strict`,
        # so it ships unreachable and silent. The nav is generated from the same
        # target list and compared by check-fresh, so this is the property that
        # makes that impossible rather than merely unlikely.
        pages = build_pages()
        nav = render_nav(pages)
        for target, _b, _e in pages:
            check(f"api/{os.path.basename(target['out'])}" in nav,
                  f"page in nav: {target['out']}")
        check(nav.count("api/") == len(pages) + 2,
              "the nav lists every page, the overview and the A–Z, nothing else")
        check("\n      - api/README.md" in nav,
              "the overview is the section's own landing page (navigation.indexes)")
        spliced = splice_nav(NAV_FIXTURE, nav)
        check(NAV_BEGIN in spliced and NAV_END in spliced and
              "  - FAQ: faq.md" in spliced and "  - Home: index.md" in spliced,
              "splicing the nav preserves the rest of mkdocs.yml")
        check(splice_nav(spliced, nav) == spliced, "splicing is idempotent")
        # Bug caught: the markers silently absent, so `generate` writes an
        # mkdocs.yml with no API nav at all and nothing says so.
        try:
            splice_nav("site_name: demo\n", nav)
            check(False, "a missing nav marker must fail loudly")
        except SystemExit:
            pass

        # Bug caught: two headers mapping to one page, so one silently overwrites
        # the other and the reference is short by a whole file.
        outs = [t["out"] for t, _b, _e in pages]
        check(len(outs) == len(set(outs)), "page paths are unique")

        # ── the two removability screens must not drift apart again (DOCS1) ──
        # This gate runs at BUILD time; prepare_site.py's runs at PUBLISH time.
        # They screen the same property, so a term one refuses and the other
        # allows is a document that builds green and fails the release — which
        # is exactly what `E3-COMPLETED.md` in docs/roadmap.md did. Rather than
        # trust two hand-maintained lists to stay equal, read the publish-time
        # list out of its source and prove this pattern matches every term in
        # it. A term added there and forgotten here now fails the build.
        site_src = open(os.path.join(REPO, "tools/prepare_site.py"),
                        encoding="utf-8").read()
        block = re.search(r'for pat in \((.*?)\):', site_src, re.S)
        check(block is not None,
              "prepare_site.py's screen list is still findable by this test")
        if block:
            publish_terms = re.findall(r'"([^"]+)"', block.group(1))
            check(len(publish_terms) >= 7,
                  f"found {len(publish_terms)} publish-time screen terms (expected >= 7)")
            for term in publish_terms:
                # "](internal/" and friends embed the bare term this gate uses;
                # matching the term anywhere in the string is the property.
                check(REMOVABILITY_TERMS.search(term) is not None,
                      f"build-time removability screens the publish-time term {term!r}")

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
    ap.add_argument("header", nargs="?",
                    help="check-coverage only: narrow the REPORT to one header")
    args = ap.parse_args()
    if args.command == "generate":
        for rel in do_generate():
            print(f"wrote {rel}")
        return 0
    if args.command == "check-coverage":
        return do_check_coverage(args.header)
    if args.command == "check-fresh":
        return do_check_fresh()
    if args.command == "check-examples":
        return do_check_examples()
    if args.command == "check-removability":
        return do_check_removability()
    return do_self_test()


if __name__ == "__main__":
    sys.exit(main())
