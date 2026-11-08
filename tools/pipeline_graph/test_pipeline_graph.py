# SPDX-License-Identifier: MIT
"""Self-tests for pipeline_graph.py (#610). Run: python3 -m unittest discover tools/pipeline_graph"""

from __future__ import annotations

import sys
import tempfile
import textwrap
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pipeline_graph as pg  # noqa: E402

BASE = textwrap.dedent("""\
    schema: branes.pipeline/1
    name: toy
    title: Toy
    sources:
      - {id: paper, cite: "A paper"}
    edges:
      - {id: Sample, kind: input, contents: a sample}
      - {id: State, kind: state, contents: the state}
      - {id: Diag, kind: output, contents: what happened}
    nodes:
      - id: predict
        title: Predict
        does: propagate the state
        reads: [State, Sample]
        writes: [State]
        source: {ref: paper}
        status: implemented
      - id: correct
        title: Correct
        does: update the state
        reads: [State]
        writes: [State, Diag]
        source: {ref: paper}
        status: implemented
    schedule:
      events:
        - event: sample
          steps:
            - {call: predict}
            - {call: correct}
    """)


def edit(text: str, old: str, new: str) -> str:
    """`text` with `old` replaced once; fails if `old` is absent, so a fixture
    edit can never silently test the unedited spec."""
    assert text.count(old) == 1, f"fixture edit does not apply: {old!r}"
    return text.replace(old, new)


def write(d: Path, name: str, text: str) -> Path:
    p = d / name
    p.write_text(text, encoding="utf-8")
    return p


class PipelineGraphTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self._tmp.name)

    def tearDown(self) -> None:
        self._tmp.cleanup()

    def analyse(self, text: str, name: str = "spec.yaml"):
        return pg.analyse(write(self.dir, name, text))

    def test_a_well_formed_spec_lints_clean_and_reports_its_gaps(self) -> None:
        _, _, rep = self.analyse(BASE)
        self.assertEqual(rep.errors, [])
        self.assertIn("node 'predict': no implementing code", rep.gaps)
        self.assertIn("node 'correct': no stage bench", rep.gaps)

    def test_a_yaml_comment_that_truncates_a_string_is_an_error(self) -> None:
        text = edit(BASE, "does: update the state", "does: update the state (#12, #13)")
        _, _, rep = self.analyse(text)
        self.assertTrue(any("unbalanced brackets" in e for e in rep.errors), rep.errors)

    def test_graph_holes_are_errors(self) -> None:
        cases = {
            "reads unknown edge": edit(BASE, "reads: [State]\n", "reads: [Stat]\n"),
            "no node reads it": edit(BASE, "{id: Diag, kind: output", "{id: Diag, kind: data"),
            "is never called": edit(BASE, "        - {call: correct}\n", ""),
            "calls unknown node": edit(BASE, "{call: correct}", "{call: corect}"),
            "source must name": edit(BASE, "    source: {ref: paper}\n    status: implemented\nschedule",
                                     "    source: {ref: other}\n    status: implemented\nschedule"),
        }
        for want, text in cases.items():
            with self.subTest(want):
                _, _, rep = self.analyse(text)
                self.assertTrue(any(want in e for e in rep.errors), rep.errors)

    def test_a_missing_file_or_test_case_reference_is_an_error(self) -> None:
        text = edit(BASE, "status: implemented\n  - id: correct",
                    "status: implemented\n    code: no/such/file.hpp\n  - id: correct")
        _, _, rep = self.analyse(text)
        self.assertTrue(any("code file not found: no/such/file.hpp" in e for e in rep.errors), rep.errors)
        text = edit(
            BASE, "status: implemented\n  - id: correct",
            "status: implemented\n    bench: {file: tests/sdk/msckf_stages.cpp, case: no such case}\n"
            "  - id: correct")
        _, _, rep = self.analyse(text)
        self.assertTrue(any("matches no TEST_CASE" in e for e in rep.errors), rep.errors)

    def test_a_delta_that_changes_an_edge_must_revisit_every_consumer(self) -> None:
        write(self.dir, "toy.yaml", BASE)
        delta = textwrap.dedent("""\
            schema: branes.pipeline/1
            base: toy.yaml
            name: toy-invariant
            title: Toy, invariant error
            edges:
              - {id: State, op: replace, kind: state, contents: the state in another error convention}
            nodes:
              - id: predict
                op: replace
                title: Predict
                does: propagate in the new convention
                reads: [State, Sample]
                writes: [State]
                source: {ref: paper}
                status: implemented
            """)
        _, _, rep = self.analyse(delta, "delta.yaml")
        # `correct` still consumes State as the base defined it.
        self.assertTrue(any("base node 'correct' consumes it" in e for e in rep.errors), rep.errors)

        acknowledged = delta + textwrap.dedent("""\
            unchanged_consumers:
              State: {correct: "its Jacobian does not depend on the error convention"}
            """)
        spec, _, rep = self.analyse(acknowledged, "delta.yaml")
        self.assertEqual(rep.errors, [])
        self.assertEqual([n["id"] for n in spec["nodes"]], ["predict", "correct"])
        self.assertEqual(spec["nodes"][0]["does"], "propagate in the new convention")

    def test_delta_operations_are_checked_against_the_base(self) -> None:
        write(self.dir, "toy.yaml", BASE)
        head = "schema: branes.pipeline/1\nbase: toy.yaml\nname: d\ntitle: D\n"
        for body, want in [
            ("nodes:\n  - {id: predict, op: add}\n", "the base already has it"),
            ("nodes:\n  - {id: smooth, op: replace}\n", "the base has no such entry"),
            ("nodes:\n  - {id: predict, op: patch}\n", "op must be one of"),
        ]:
            with self.subTest(want):
                _, _, rep = self.analyse(head + body, "d.yaml")
                self.assertTrue(any(want in e for e in rep.errors), rep.errors)

    def test_the_figure_draws_an_in_place_update_as_one_arrow(self) -> None:
        spec, _, _ = self.analyse(BASE)
        dot = pg.dot_source(spec)
        self.assertIn('"n:predict" -> "e:State" [dir=both', dot)
        self.assertNotIn('"e:State" -> "n:predict"', dot)
        self.assertIn('"e:Sample" -> "n:predict"', dot)

    def test_the_page_escapes_what_mdx_would_parse(self) -> None:
        text = edit(BASE, "does: update the state", "does: \"update in posit<32,2> {fast} | slow\"")
        spec, delta, rep = self.analyse(text)
        page = pg.page_source(spec, delta, rep)
        self.assertIn("posit&lt;32,2&gt; &#123;fast&#125; \\| slow", page)


if __name__ == "__main__":
    unittest.main()
