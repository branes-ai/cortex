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

    def test_a_planned_node_may_be_scheduled_a_missing_one_may_not(self) -> None:
        planned = edit(BASE, "writes: [State, Diag]\n    source: {ref: paper}\n    status: implemented",
                       "writes: [State, Diag]\n    source: {ref: paper}\n    status: planned")
        _, _, rep = self.analyse(planned)
        self.assertEqual(rep.errors, [])
        self.assertIn("node 'correct' (): planned, not implemented yet", rep.gaps)
        _, _, rep = self.analyse(planned.replace("status: planned", "status: missing"))
        self.assertTrue(any("is missing but the schedule calls it" in e for e in rep.errors), rep.errors)

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

    def test_a_duplicate_key_is_rejected_not_silently_overwritten(self) -> None:
        text = edit(BASE, "    does: update the state\n", "    does: update the state\n    does: something else\n")
        _, _, rep = self.analyse(text)
        self.assertTrue(any("duplicate key 'does'" in e for e in rep.errors), rep.errors)

    def test_a_delta_must_name_itself(self) -> None:
        write(self.dir, "toy.yaml", BASE)
        for header, want in [("schema: branes.pipeline/1\nbase: toy.yaml\ntitle: D\n", "missing 'name'"),
                             ("schema: branes.pipeline/1\nbase: toy.yaml\nname: toy\ntitle: D\n",
                              "a delta needs its own name")]:
            with self.subTest(want):
                _, _, rep = self.analyse(header, "d.yaml")
                self.assertTrue(any(want in e for e in rep.errors), rep.errors)

    def test_what_the_page_needs_is_linted(self) -> None:
        cases = {
            "needs an id and a cite": edit(BASE, '{id: paper, cite: "A paper"}', "{id: paper}"),
            "needs an id and a text": BASE + "system_invariants:\n  - {id: nees}\n",
        }
        for want, text in cases.items():
            with self.subTest(want):
                _, _, rep = self.analyse(text)
                self.assertTrue(any(want in e for e in rep.errors), rep.errors)

    def with_file(self, text: str, check) -> None:
        (self.dir / "a.hpp").write_text(text, encoding="utf-8")
        saved = pg.ROOT
        pg.ROOT = self.dir
        pg._file_cache.clear()
        try:
            check()
        finally:
            pg.ROOT = saved
            pg._file_cache.clear()

    def resolves(self, ref: dict) -> bool:
        return pg.check_ref("here", {"file": "a.hpp", **ref}, pg.Report(), "code")

    def test_a_symbol_must_be_an_identifier_in_code(self) -> None:
        src = textwrap.dedent("""\
            // Result is documented here
            /* and apply
               here */
            struct Other {};
            const char* s = "Quoted";
            const char q = '"'; struct AfterChar {}; // "Hidden
            const int big = 1'000'000; struct AfterSeparator {};
            const char* r = R"x(RawBody "not code" )x"; struct AfterRaw {};
            """)

        def check() -> None:
            for sym, ok in [("Result", False), ("apply", False), ("Quoted", False), ("Hidden", False),
                            ("RawBody", False), ("Other", True), ("AfterChar", True), ("AfterSeparator", True),
                            ("AfterRaw", True), ("Othe", False)]:
                with self.subTest(sym):
                    self.assertEqual(self.resolves({"symbol": sym}), ok)

        self.with_file(src, check)

    def test_a_case_must_be_an_active_test_case(self) -> None:
        src = textwrap.dedent("""\
            TEST_CASE("live case", "[t]") {}
            // TEST_CASE("commented out", "[t]") {}
            /* TEST_CASE("block commented", "[t]") {} */
            TEMPLATE_TEST_CASE("templated case", "[t]", float, double) {}
            const char* s = "TEST_CASE(\\"in a string\\")";
            """)

        def check() -> None:
            for case, ok in [("live", True), ("templated", True), ("commented out", False),
                             ("block commented", False), ("in a string", False)]:
                with self.subTest(case):
                    self.assertEqual(self.resolves({"case": case}), ok)

        self.with_file(src, check)

    def test_an_unhashable_key_is_an_error_not_a_traceback(self) -> None:
        _, _, rep = self.analyse("? [schema]\n: x\n" + BASE)
        self.assertTrue(any("unhashable key" in e for e in rep.errors), rep.errors)

    def test_a_decision_without_a_reason_is_a_gap(self) -> None:
        text = edit(BASE, "    does: propagate the state\n",
                    "    does: propagate the state\n"
                    "    decisions:\n"
                    "      - {question: input over an interval, chosen: the end sample held,\n"
                    "         alternatives: [both samples (RK4)]}\n"
                    "      - {question: integrator, chosen: RK4, reason: the paper's choice}\n")
        spec, delta, rep = self.analyse(text)
        self.assertEqual(rep.errors, [])
        self.assertIn("node 'predict': decision 'input over an interval' has no recorded reason "
                      "(chosen: the end sample held)", rep.gaps)
        self.assertFalse(any("integrator" in g for g in rep.gaps))
        page = pg.page_source(spec, delta, rep)
        self.assertIn("the end sample held over both samples (RK4). **no reason recorded**", page)

        bad = edit(BASE, "    does: propagate the state\n",
                   "    does: propagate the state\n    decisions:\n      - {question: integrator}\n")
        _, _, rep = self.analyse(bad)
        self.assertTrue(any("needs a question and what was chosen" in e for e in rep.errors), rep.errors)

        for chosen in ("0", "false"):
            with self.subTest(chosen=chosen):
                zero = edit(BASE, "    does: propagate the state\n",
                            "    does: propagate the state\n    decisions:\n"
                            f"      - {{question: substeps, chosen: {chosen}, reason: one is enough}}\n")
                _, _, rep = self.analyse(zero)
                self.assertEqual(rep.errors, [])

    def test_a_bound_needs_its_derivation_and_a_consumer(self) -> None:
        good = BASE + ("system_invariants:\n"
                       "  - id: closure\n"
                       "    text: S2 closes on the truth\n"
                       "    bound: {value: 1.1e-4 rad, consumer: correct, derivation: a tenth of 0.5 px}\n")
        spec, delta, rep = self.analyse(good)
        self.assertEqual(rep.errors, [])
        self.assertIn("(bound 1.1e-4 rad, from what **correct** resolves: a tenth of 0.5 px)",
                      pg.page_source(spec, delta, rep))
        cases = {
            "needs a value and its derivation": good.replace(", derivation: a tenth of 0.5 px", ""),
            "consumer must name the node": good.replace("consumer: correct", "consumer: nobody"),
        }
        _, _, rep = self.analyse(good.replace("value: 1.1e-4 rad", "value: 0"))
        self.assertEqual(rep.errors, [])
        for want, text in cases.items():
            with self.subTest(want):
                _, _, rep = self.analyse(text)
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
