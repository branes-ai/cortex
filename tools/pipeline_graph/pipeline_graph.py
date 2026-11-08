#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""pipeline_graph — lint, cross-check and render a pipeline spec (#610).

A pipeline spec (docs/methodology/spec-format.mdx) describes an algorithm taken
from a paper as a transformation graph: the nodes (transformations), the edges
(the data structures they exchange in memory), the call schedule and the
invariants the whole graph must satisfy. A delta spec describes a paper that
changes an existing pipeline, against its `base`.

Commands:
  check  SPEC...   lint each spec, resolve every code / test reference against
                   the repository, and verify the generated page and graph
                   source are up to date. Exit 1 on an error; gaps are reported,
                   not errors.
  render SPEC...   write the generated page (<page_dir>/<name>.mdx), the graph
                   source (<figure_dir>/<name>.dot) and, when Graphviz `dot` is
                   on PATH, the figure (<name>.svg).
  gaps   SPEC...   print the gap report: nodes without code, bench or tests,
                   invariants without a check, deviations from the paper.

Run from anywhere; paths in a spec are relative to the repository root.
"""

from __future__ import annotations

import argparse
import copy
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover
    sys.exit("pipeline_graph: needs PyYAML (python3 -m pip install pyyaml)")

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "branes.pipeline/1"
PAGE_DIR = ROOT / "docs" / "methodology"
FIGURE_DIR = ROOT / "docs" / "assessments" / "figures" / "methodology"
STATUSES = {"implemented", "partial", "planned", "missing"}
INVARIANT_STATUSES = {"holds", "failing", "unknown"}
EDGE_KINDS = {"data", "input", "state", "output"}
DELTA_OPS = {"add", "replace", "remove"}

NODE_KEYS = {"id", "stage", "title", "does", "reads", "writes", "source", "pre", "post", "code", "bench",
             "tests", "status", "optional", "precision", "deviations", "op", "note"}
EDGE_KEYS = {"id", "kind", "contents", "code", "symbol", "invariants", "precision", "deviations", "op", "note"}
TOP_KEYS = {"schema", "name", "title", "summary", "sources", "edges", "nodes", "schedule", "system_invariants",
            "base", "changes", "unchanged_consumers"}


@dataclass
class Report:
    errors: list[str] = field(default_factory=list)
    gaps: list[str] = field(default_factory=list)

    def error(self, msg: str) -> None:
        self.errors.append(msg)

    def gap(self, msg: str) -> None:
        self.gaps.append(msg)


# ── loading and delta resolution ─────────────────────────────────────────────

class _UniqueKeyLoader(yaml.SafeLoader):
    """SafeLoader that rejects a duplicate mapping key. PyYAML keeps the last
    one silently, so a second `deviations:` would erase the first list's gaps."""


def _construct_unique_mapping(loader: yaml.SafeLoader, node: yaml.MappingNode, deep: bool = False) -> dict:
    seen = set()
    for key_node, _ in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in seen:
            raise yaml.constructor.ConstructorError(None, None, f"duplicate key {key!r}", key_node.start_mark)
        seen.add(key)
    return yaml.SafeLoader.construct_mapping(loader, node, deep=deep)


_UniqueKeyLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping)


def load_yaml(path: Path) -> dict:
    with path.open(encoding="utf-8") as f:
        data = yaml.load(f, Loader=_UniqueKeyLoader)  # noqa: S506 - a SafeLoader subclass
    if not isinstance(data, dict):
        raise ValueError(f"{path}: a spec is a mapping")
    return data


def apply_delta(base: dict, delta: dict, rep: Report) -> dict:
    """The delta's pipeline: the base with the delta's node and edge operations
    applied. An entry's `op` is add, replace or remove; a replace or remove
    must name an entry the base has, an add one it does not."""
    out = copy.deepcopy(base)
    for key in ("name", "title", "summary"):
        if key in delta:
            out[key] = delta[key]
    out["sources"] = list(base.get("sources", [])) + list(delta.get("sources", []))
    for section in ("edges", "nodes"):
        items = {e["id"]: e for e in out.get(section, [])}
        order = [e["id"] for e in out.get(section, [])]
        for entry in delta.get(section, []):
            op = entry.get("op")
            eid = entry.get("id")
            if op not in DELTA_OPS:
                rep.error(f"delta {section} '{eid}': op must be one of {sorted(DELTA_OPS)}")
                continue
            if op == "add":
                if eid in items:
                    rep.error(f"delta {section} '{eid}': add, but the base already has it (use replace)")
                    continue
                order.append(eid)
            elif eid not in items:
                rep.error(f"delta {section} '{eid}': {op}, but the base has no such entry")
                continue
            if op == "remove":
                order.remove(eid)
                items.pop(eid)
                continue
            items[eid] = {k: v for k, v in entry.items() if k != "op"}
        out[section] = [items[i] for i in order]
    if "schedule" in delta:
        out["schedule"] = delta["schedule"]
    if "system_invariants" in delta:
        out["system_invariants"] = delta["system_invariants"]
    out.pop("base", None)
    return out


def changed_edge_consumers(base: dict, delta: dict) -> dict[str, list[str]]:
    """For every edge the delta replaces or removes: the base nodes that read or
    write it and that the delta does not itself replace or remove. Each one is
    still the base's version of a node that consumes a changed data structure."""
    revisited = {n["id"] for n in delta.get("nodes", []) if n.get("op") in ("replace", "remove")}
    changed = {e["id"] for e in delta.get("edges", []) if e.get("op") in ("replace", "remove")}
    out: dict[str, list[str]] = {}
    for edge in sorted(changed):
        users = [n["id"] for n in base.get("nodes", [])
                 if edge in n.get("reads", []) + n.get("writes", []) and n["id"] not in revisited]
        if users:
            out[edge] = users
    return out


def resolve(path: Path, rep: Report) -> tuple[dict, dict | None]:
    """The spec at `path`, with a delta applied to its base; and the delta itself
    (None for a base spec)."""
    spec = load_yaml(path)
    if "base" not in spec:
        return spec, None
    base_path = (path.parent / spec["base"]).resolve()
    if not base_path.is_file():
        rep.error(f"base spec not found: {spec['base']}")
        return spec, spec
    base = load_yaml(base_path)
    if "base" in base:
        rep.error("a delta's base must be a full spec, not another delta")
    merged = apply_delta(base, spec, rep)
    merged["_base"] = base
    merged["_base_path"] = base_path
    return merged, spec


# ── lint and cross-check ─────────────────────────────────────────────────────

_TEST_CASE = re.compile(r'TEST_CASE\(\s*"((?:[^"\\]|\\.)*)"')
_COMMENT = re.compile(r'//[^\n]*|/\*.*?\*/|"(?:[^"\\\n]|\\.)*"', re.S)
_file_cache: dict[Path, str] = {}


def code_only(text: str) -> str:
    """C/C++ source with its comments removed (string literals kept), so a
    symbol named only in a comment doesn't satisfy a reference."""
    return _COMMENT.sub(lambda m: m.group(0) if m.group(0).startswith('"') else " ", text)


def file_text(rel: str) -> str | None:
    p = ROOT / rel
    if not p.is_file():
        return None
    if p not in _file_cache:
        _file_cache[p] = p.read_text(encoding="utf-8", errors="replace")
    return _file_cache[p]


def check_ref(where: str, ref: dict | None, rep: Report, what: str) -> bool:
    """A reference {file, symbol?, case?}: the file exists, the symbol occurs in
    it as a word, the case is a prefix of one of its TEST_CASE names."""
    if not ref:
        return False
    if not isinstance(ref, dict) or "file" not in ref:
        rep.error(f"{where}: {what} must be a mapping with a file")
        return False
    text = file_text(ref["file"])
    if text is None:
        rep.error(f"{where}: {what} file not found: {ref['file']}")
        return False
    if "symbol" in ref and not re.search(rf"\b{re.escape(ref['symbol'])}\b", code_only(text)):
        rep.error(f"{where}: {what} symbol '{ref['symbol']}' not in the code of {ref['file']} (comments don't count)")
        return False
    if "case" in ref:
        names = _TEST_CASE.findall(text)
        if not any(n.startswith(ref["case"]) for n in names):
            rep.error(f"{where}: {what} case '{ref['case']}' matches no TEST_CASE in {ref['file']}")
            return False
    return True


def strings(node: object, where: str = ""):
    """Every string in a parsed spec, with where it sits."""
    if isinstance(node, str):
        yield where, node
    elif isinstance(node, dict):
        for k, v in node.items():
            if not str(k).startswith("_"):
                yield from strings(v, f"{where}.{k}" if where else str(k))
    elif isinstance(node, list):
        for i, v in enumerate(node):
            yield from strings(v, f"{where}[{i}]")


def lint(spec: dict, delta: dict | None, rep: Report) -> None:
    raw = delta if delta is not None else spec
    # YAML ends a plain scalar at " #": "(#474, #574)" silently becomes "(#474,".
    # An unbalanced bracket is the symptom; quote the string.
    for where, text in strings(raw):
        if text.count("(") != text.count(")") or text.count("[") != text.count("]"):
            rep.error(f"{where}: unbalanced brackets in \"{text}\" (an unquoted ' #' starts a YAML comment)")
    for k in raw:
        if k not in TOP_KEYS:
            rep.error(f"unknown top-level key '{k}'")
    if raw.get("schema") != SCHEMA:
        rep.error(f"schema must be '{SCHEMA}'")
    # Checked on the document as written: a delta that omitted its name would
    # inherit the base's, and `render` would overwrite the base's page.
    for k in ("name", "title"):
        if not raw.get(k):
            rep.error(f"missing '{k}'")
    if raw.get("name") and not re.fullmatch(r"[a-z0-9][a-z0-9-]*", raw["name"]):
        rep.error("name must be kebab-case (it names the generated files)")
    if delta is not None and "_base" in spec and raw.get("name") == spec["_base"].get("name"):
        rep.error("a delta needs its own name: the base's would overwrite the base's generated page")

    sources = set()
    for s in spec.get("sources", []):
        if not s.get("id") or not s.get("cite"):
            rep.error(f"source {s.get('id')!r}: needs an id and a cite")
        sources.add(s.get("id"))
    edges = {}
    for e in spec.get("edges", []):
        eid = e.get("id")
        if not eid:
            rep.error("an edge has no id")
            continue
        if eid in edges:
            rep.error(f"duplicate edge '{eid}'")
        edges[eid] = e
        for k in e:
            if k not in EDGE_KEYS:
                rep.error(f"edge '{eid}': unknown key '{k}'")
        if e.get("kind", "data") not in EDGE_KINDS:
            rep.error(f"edge '{eid}': kind must be one of {sorted(EDGE_KINDS)}")
        if e.get("code"):
            check_ref(f"edge '{eid}'", {"file": e["code"], **({"symbol": e["symbol"]} if e.get("symbol") else {})},
                      rep, "code")
        else:
            rep.gap(f"edge '{eid}': no implementing type")
        for inv in e.get("invariants", []):
            if not inv.get("text"):
                rep.error(f"edge '{eid}': an invariant has no text")
            if inv.get("check"):
                check_ref(f"edge '{eid}' invariant '{inv.get('text')}'", inv["check"], rep, "check")
            else:
                rep.gap(f"edge '{eid}': no validator for \"{inv.get('text')}\"")
        for d in e.get("deviations", []):
            rep.gap(f"edge '{eid}': deviates from the paper: {d}")

    nodes = {}
    for n in spec.get("nodes", []):
        nid = n.get("id")
        if not nid:
            rep.error("a node has no id")
            continue
        if nid in nodes:
            rep.error(f"duplicate node '{nid}'")
        nodes[nid] = n
        for k in n:
            if k not in NODE_KEYS:
                rep.error(f"node '{nid}': unknown key '{k}'")
        status = n.get("status")
        if status not in STATUSES:
            rep.error(f"node '{nid}': status must be one of {sorted(STATUSES)}")
        for k in ("title", "does"):
            if not n.get(k):
                rep.error(f"node '{nid}': missing '{k}'")
        for k in ("reads", "writes"):
            for eid in n.get(k, []):
                if eid not in edges:
                    rep.error(f"node '{nid}' {k} unknown edge '{eid}'")
        if not n.get("writes"):
            rep.error(f"node '{nid}': writes nothing")
        src = n.get("source", {})
        if not src or src.get("ref") not in sources:
            rep.error(f"node '{nid}': source must name one of the spec's sources")
        if status == "missing":
            rep.gap(f"node '{nid}' ({n.get('stage', '')}): not implemented")
            continue
        if status == "planned":
            # Specified ahead of the code: in the schedule, not yet built.
            rep.gap(f"node '{nid}' ({n.get('stage', '')}): planned, not implemented yet")
            continue
        if not n.get("code"):
            rep.gap(f"node '{nid}': no implementing code")
        else:
            check_ref(f"node '{nid}'", {"file": n["code"]}, rep, "code")
        if n.get("bench"):
            check_ref(f"node '{nid}'", n["bench"], rep, "bench")
        else:
            rep.gap(f"node '{nid}': no stage bench")
        if n.get("tests"):
            for t in n["tests"]:
                check_ref(f"node '{nid}'", t, rep, "test")
        else:
            rep.gap(f"node '{nid}': no unit tests")
        if not n.get("pre") or not n.get("post"):
            rep.gap(f"node '{nid}': pre- or post-conditions not stated")
        if status == "partial":
            rep.gap(f"node '{nid}' ({n.get('stage', '')}): partially implemented")
        for d in n.get("deviations", []):
            rep.gap(f"node '{nid}': deviates from the paper: {d}")

    # Every edge is produced (an input, or some node writes it) and consumed.
    for eid, e in edges.items():
        writers = [n for n in nodes.values() if eid in n.get("writes", [])]
        readers = [n for n in nodes.values() if eid in n.get("reads", [])]
        if e.get("kind") != "input" and not writers:
            rep.error(f"edge '{eid}': no node writes it and it is not an input")
        if e.get("kind") == "input" and writers:
            rep.error(f"edge '{eid}': an input, but node(s) {[n['id'] for n in writers]} write it")
        if not readers and e.get("kind") != "output":
            rep.error(f"edge '{eid}': no node reads it and it is not an output")
        if readers and e.get("kind") == "output":
            rep.error(f"edge '{eid}': an output, but node(s) {[n['id'] for n in readers]} read it")

    sched = spec.get("schedule", {})
    if not sched.get("events"):
        rep.error("schedule: no events")
    if sched.get("driver"):
        check_ref("schedule", {"file": sched["driver"]}, rep, "driver")
    called = set()
    for ev in sched.get("events", []):
        if not ev.get("event"):
            rep.error("schedule: an entry has no 'event'")
        for st in ev.get("steps", []):
            c = st.get("call")
            if c not in nodes:
                rep.error(f"schedule '{ev.get('event')}': calls unknown node '{c}'")
            called.add(c)
    for nid, n in nodes.items():
        if nid not in called and n.get("status") != "missing":
            rep.error(f"node '{nid}' is never called by the schedule")
        if nid in called and n.get("status") == "missing":
            rep.error(f"node '{nid}' is missing but the schedule calls it")

    for inv in spec.get("system_invariants", []):
        iid = inv.get("id")
        if not iid or not inv.get("text"):
            rep.error(f"system invariant '{iid}': needs an id and a text")
        if inv.get("status", "unknown") not in INVARIANT_STATUSES:
            rep.error(f"system invariant '{iid}': status must be one of {sorted(INVARIANT_STATUSES)}")
        if inv.get("check"):
            check_ref(f"system invariant '{iid}'", inv["check"], rep, "check")
        else:
            rep.gap(f"system invariant '{iid}': no check")
        if inv.get("status") == "failing":
            rep.gap(f"system invariant '{iid}': failing — {inv.get('note', '')}".rstrip(" —"))

    if delta is not None and "_base" in spec:
        acknowledged = delta.get("unchanged_consumers", {}) or {}
        for edge, users in changed_edge_consumers(spec["_base"], delta).items():
            for u in users:
                why = (acknowledged.get(edge) or {}).get(u) if isinstance(acknowledged.get(edge), dict) else None
                if not why:
                    rep.error(f"delta changes edge '{edge}', but base node '{u}' consumes it and is neither "
                              f"replaced nor listed under unchanged_consumers.{edge}.{u} with a reason")


# ── rendering ────────────────────────────────────────────────────────────────

def dot_source(spec: dict) -> str:
    def q(s: str) -> str:
        return '"' + s.replace('"', '\\"') + '"'

    lines = [
        f"// Generated by tools/pipeline_graph/pipeline_graph.py from the {spec['name']} spec. Do not edit.",
        f"digraph {q(spec['name'])} {{",
        '  graph [rankdir=LR, fontname="Helvetica", fontsize=11, bgcolor="transparent", nodesep=0.25, ranksep=0.45];',
        '  node [fontname="Helvetica", fontsize=10];',
        '  edge [color="#5b6b82", arrowsize=0.6];',
    ]
    for e in spec.get("edges", []):
        kind = e.get("kind", "data")
        shape = {"input": "parallelogram", "state": "cylinder", "data": "note", "output": "parallelogram"}[kind]
        fill = {"input": "#e8eef8", "state": "#fdf1d8", "data": "#f4f6f9", "output": "#efe8f8"}[kind]
        lines.append(f'  {q("e:" + e["id"])} [label={q(e["id"])}, shape={shape}, style=filled, '
                     f'fillcolor="{fill}", color="#8a97ab"];')
    for n in spec.get("nodes", []):
        status = n.get("status")
        fill = {"implemented": "#d9f2e3", "partial": "#fff1c2", "planned": "#dde7f7", "missing": "#f9d6d5"}[status]
        style = "filled,dashed" if n.get("optional") else "filled"
        label = f"{n.get('stage', '')}  {n['id']}".strip()
        lines.append(f'  {q("n:" + n["id"])} [label={q(label)}, shape=box, style="{style},rounded", '
                     f'fillcolor="{fill}", color="#4a5568"];')
    # A node that reads and writes the same structure updates it in place: one
    # double-headed arrow, not a loop of two.
    for n in spec.get("nodes", []):
        reads, writes = n.get("reads", []), n.get("writes", [])
        for eid in reads:
            if eid in writes:
                lines.append(f'  {q("n:" + n["id"])} -> {q("e:" + eid)} [dir=both, color="#b7791f"];')
            else:
                lines.append(f'  {q("e:" + eid)} -> {q("n:" + n["id"])};')
        for eid in writes:
            if eid not in reads:
                lines.append(f'  {q("n:" + n["id"])} -> {q("e:" + eid)};')
    inputs = [e["id"] for e in spec.get("edges", []) if e.get("kind") == "input"]
    if inputs:
        lines.append("  { rank=source; " + " ".join(q("e:" + i) for i in inputs) + " }")
    lines.append("}")
    return "\n".join(lines) + "\n"


def md_escape(s: object) -> str:
    """Prose for an MDX table cell or list item: `|` would end a cell, and a bare
    `<` or `{` opens JSX."""
    out = str(s).replace("\n", " ").strip()
    for a, b in (("|", "\\|"), ("<", "&lt;"), (">", "&gt;"), ("{", "&#123;"), ("}", "&#125;")):
        out = out.replace(a, b)
    return out


def ref_md(ref: dict | None) -> str:
    if not ref:
        return "—"
    s = f"`{ref['file']}`"
    if ref.get("symbol"):
        s += f" `{ref['symbol']}`"
    if ref.get("case"):
        s += f" “{md_escape(ref['case'])}…”"
    return s


def page_source(spec: dict, delta: dict | None, rep: Report) -> str:
    name = spec["name"]
    title = spec["title"]
    src_ids = {s["id"]: s for s in spec.get("sources", [])}
    out = [
        "---",
        f"title: \"{title}: the transformation graph\"",
        f"description: \"The {title} pipeline as a transformation graph (#610): its transformations, the data "
        "structures they exchange, the call schedule, the invariants, and what cortex implements.\"",
        "sidebar:",
        f"  label: \"{title}\"",
        "---",
        "",
        f"{{/* Generated by tools/pipeline_graph/pipeline_graph.py from "
        f"docs/methodology/specs/{name}.yaml. Do not edit; edit the spec and run `render`. */}}",
        "",
    ]
    if spec.get("summary"):
        out += [md_escape(spec["summary"]), ""]
    if delta is not None:
        out += [f"A **delta** on [{spec['_base']['title']}]({spec['_base']['name']}.mdx): it changes the nodes and "
                "data structures listed under *Changes from the base*, and keeps the rest.", ""]
    out += [
        f"![The {title} transformation graph: boxes are transformations (green implemented, amber partial, "
        "blue planned, red missing; dashed optional), parallelograms are sensor inputs and outputs, the cylinder "
        "is the shared filter state, notes are the data structures passed between transformations, and a "
        f"double-headed arrow is a transformation that updates a structure in place.]"
        f"(../assessments/figures/methodology/{name}.svg)",
        "",
        "## Sources",
        "",
    ]
    for s in spec.get("sources", []):
        out.append(f"- **{s['id']}** — {md_escape(s['cite'])}")
    out += ["", "## Transformations", "",
            "Each has the shape *(state, inputs, params) → (state′, diagnostics)*.", "",
            "| Stage | Node | Does | Reads → writes | Paper | Status | Code | Bench |",
            "|---|---|---|---|---|---|---|---|"]
    for n in spec.get("nodes", []):
        src = n.get("source", {})
        paper = src.get("ref", "")
        if src.get("where"):
            paper += f", {src['where']}"
        rw = f"{', '.join(n.get('reads', []))} → {', '.join(n.get('writes', []))}"
        status = n.get("status", "") + (" (optional)" if n.get("optional") else "")
        out.append(f"| {n.get('stage', '')} | **{n['id']}** | {md_escape(n.get('does', ''))} | {md_escape(rw)} | "
                   f"{md_escape(paper)} | {status} | {('`' + n['code'] + '`') if n.get('code') else '—'} | "
                   f"{ref_md(n.get('bench'))} |")
    out += ["", "### Contracts", ""]
    for n in spec.get("nodes", []):
        if not (n.get("pre") or n.get("post") or n.get("deviations") or n.get("precision")):
            continue
        out.append(f"**{n['id']}** ({n.get('stage', '')}, {n.get('title', '')})")
        out.append("")
        if n.get("pre"):
            out.append(f"- *Pre:* {'; '.join(md_escape(p) for p in n['pre'])}")
        if n.get("post"):
            out.append(f"- *Post:* {'; '.join(md_escape(p) for p in n['post'])}")
        for t in n.get("tests", []):
            out.append(f"- *Test:* {ref_md(t)}")
        if n.get("precision"):
            p = n["precision"]
            out.append(f"- *Precision:* {md_escape(p.get('policy', ''))} — {md_escape(p.get('note', ''))}"
                       if isinstance(p, dict) else f"- *Precision:* {md_escape(p)}")
        for d in n.get("deviations", []):
            out.append(f"- *Deviates from the paper:* {md_escape(d)}")
        out.append("")
    out += ["## Data structures", "",
            "| Edge | Kind | Contents | Type | Invariants (✓ has a validator) |",
            "|---|---|---|---|---|"]
    for e in spec.get("edges", []):
        invs = "; ".join(("✓ " if i.get("check") else "✗ ") + md_escape(i["text"]) for i in e.get("invariants", []))
        typ = f"`{e['code']}`" + (f" `{e['symbol']}`" if e.get("symbol") else "") if e.get("code") else "—"
        out.append(f"| **{e['id']}** | {e.get('kind', 'data')} | {md_escape(e.get('contents', ''))} | {typ} | "
                   f"{invs or '—'} |")
    notes = [e for e in spec.get("edges", []) if e.get("precision") or e.get("deviations")]
    if notes:
        out.append("")
        for e in notes:
            if e.get("precision"):
                out.append(f"- **{e['id']}** precision: {md_escape(e['precision'])}")
            for d in e.get("deviations", []):
                out.append(f"- **{e['id']}** deviates from the paper: {md_escape(d)}")
    sched = spec.get("schedule", {})
    out += ["", "## Schedule", ""]
    if sched.get("driver"):
        out += [f"The driver is `{sched['driver']}`.", ""]
    out.append("```text")
    for ev in sched.get("events", []):
        rate = f" ({ev['rate_hz']} Hz)" if ev.get("rate_hz") else ""
        out.append(f"on {ev['event']}{rate}:")
        for st in ev.get("steps", []):
            s = f"  {st['call']}"
            if st.get("foreach"):
                s += f"  for each {st['foreach']}"
            if st.get("when"):
                s += f"  [when {st['when']}]"
            if st.get("note"):
                s += f"  — {st['note']}"
            out.append(s)
    out += ["```", "", "## System invariants", "",
            "| Invariant | Check | Status |", "|---|---|---|"]
    for inv in spec.get("system_invariants", []):
        st = inv.get("status", "unknown")
        if inv.get("note"):
            st += f" — {md_escape(inv['note'])}"
        out.append(f"| {md_escape(inv['text'])} | {ref_md(inv.get('check'))} | {st} |")
    if delta is not None:
        out += ["", "## Changes from the base", ""]
        for section in ("edges", "nodes"):
            for entry in delta.get(section, []):
                out.append(f"- {entry.get('op')} {section[:-1]} **{entry.get('id')}**"
                           + (f": {md_escape(entry['note'])}" if entry.get("note") else ""))
        for edge, users in (delta.get("unchanged_consumers") or {}).items():
            for u, why in (users or {}).items():
                out.append(f"- kept base node **{u}** on changed edge **{edge}**: {md_escape(why)}")
    out += ["", "## Gap report", "",
            f"What `pipeline_graph.py gaps` reports for this spec ({len(rep.gaps)} items):", ""]
    out += [f"- {md_escape(g)}" for g in rep.gaps] or ["- none"]
    return "\n".join(out) + "\n"


def outputs(spec: dict, delta: dict | None, rep: Report) -> dict[Path, str]:
    name = spec["name"]
    return {PAGE_DIR / f"{name}.mdx": page_source(spec, delta, rep),
            FIGURE_DIR / f"{name}.dot": dot_source(spec)}


# ── commands ─────────────────────────────────────────────────────────────────

def analyse(path: Path) -> tuple[dict, dict | None, Report]:
    rep = Report()
    try:
        spec, delta = resolve(path, rep)
    except (OSError, ValueError, yaml.YAMLError) as e:
        rep.error(str(e))
        return {}, None, rep
    lint(spec, delta, rep)
    return spec, delta, rep


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["check", "render", "gaps"])
    ap.add_argument("specs", nargs="+", type=Path)
    args = ap.parse_args(argv)

    status = 0
    for path in args.specs:
        spec, delta, rep = analyse(path.resolve())
        label = path.as_posix()
        for e in rep.errors:
            print(f"{label}: error: {e}", file=sys.stderr)
        if rep.errors:
            status = 1
            continue
        if args.command == "gaps":
            print(f"{label}: {len(rep.gaps)} gaps")
            for g in rep.gaps:
                print(f"  - {g}")
            continue
        files = outputs(spec, delta, rep)
        if args.command == "render":
            for p, text in files.items():
                p.parent.mkdir(parents=True, exist_ok=True)
                p.write_text(text, encoding="utf-8")
                print(f"wrote {p.relative_to(ROOT).as_posix()}")
            dot = shutil.which("dot")
            dot_file = FIGURE_DIR / f"{spec['name']}.dot"
            if dot:
                svg = dot_file.with_suffix(".svg")
                subprocess.run([dot, "-Tsvg", str(dot_file), "-o", str(svg)], check=True)
                print(f"wrote {svg.relative_to(ROOT).as_posix()}")
            else:
                print("Graphviz dot not on PATH: the .svg was not regenerated", file=sys.stderr)
        else:
            for p, text in files.items():
                have = p.read_text(encoding="utf-8") if p.is_file() else None
                if have != text:
                    print(f"{label}: error: {p.relative_to(ROOT).as_posix()} is out of date; run "
                          f"`python3 tools/pipeline_graph/pipeline_graph.py render {label}`", file=sys.stderr)
                    status = 1
            svg = FIGURE_DIR / f"{spec['name']}.svg"
            if not svg.is_file():
                print(f"{label}: error: {svg.relative_to(ROOT).as_posix()} is missing", file=sys.stderr)
                status = 1
            if status == 0:
                print(f"{label}: ok ({len(rep.gaps)} gaps)")
    return status


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
