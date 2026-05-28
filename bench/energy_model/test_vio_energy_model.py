# SPDX-License-Identifier: MIT
"""Tests for vio_energy_model.

The pure helpers (profile parsing, modeled-vs-measured math, Markdown
formatting) always run. The graphs-backed estimation runs only when
`branes-ai/graphs` is importable, and is skipped otherwise — so this suite
passes in any environment.

    cd bench/energy_model && python -m unittest -v
"""
import importlib.util
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vio_energy_model as vem  # noqa: E402

_HAS_GRAPHS = importlib.util.find_spec("graphs") is not None

SAMPLE = {
    "sequence": "V1_01_easy/mav0",
    "energy": {"rapl_available": True, "energy_j": 220.0, "avg_power_w": 18.0},
    "operator_profile": [
        {"name": "pyramid", "flops": 1.7e10, "bytes": 9.6e8},
        {"name": "fast", "flops": 4.3e9, "bytes": 2.7e8},
        {"name": "klt", "flops": 6.0e10, "bytes": 7.5e9},
        {"name": "msckf_propagate", "flops": 2.1e10, "bytes": 1.3e8},
        {"name": "msckf_update", "flops": 8.0e9, "bytes": 5.0e7},
    ],
}


class PureHelpers(unittest.TestCase):
    def test_stage_specs(self):
        specs = vem.stage_specs(SAMPLE)
        self.assertEqual([s["name"] for s in specs],
                         ["pyramid", "fast", "klt", "msckf_propagate", "msckf_update"])
        self.assertAlmostEqual(specs[2]["flops"], 6.0e10)
        self.assertAlmostEqual(specs[0]["bytes"], 9.6e8)

    def test_stage_specs_empty(self):
        self.assertEqual(vem.stage_specs({}), [])

    def test_measured_energy(self):
        avail, j, w = vem.measured_energy(SAMPLE)
        self.assertTrue(avail)
        self.assertAlmostEqual(j, 220.0)
        self.assertAlmostEqual(w, 18.0)
        # Missing section degrades to unavailable.
        avail2, j2, _ = vem.measured_energy({})
        self.assertFalse(avail2)
        self.assertEqual(j2, 0.0)

    def test_modeled_over_measured(self):
        self.assertAlmostEqual(vem.modeled_over_measured(200.0, 50.0), 0.25)
        self.assertIsNone(vem.modeled_over_measured(0.0, 50.0))
        self.assertIsNone(vem.modeled_over_measured(-1.0, 50.0))

    def test_markdown_formatting(self):
        out = {
            "sequence": "seq",
            "skus": [
                {"sku": "cpu", "precision": "FP32", "modeled_energy_j": 0.027,
                 "modeled_avg_power_w": 88.0, "modeled_latency_s": 0.3, "per_stage": []},
                {"sku": "kpu_t64", "precision": "INT8", "modeled_energy_j": 0.00036,
                 "modeled_avg_power_w": 2.5, "modeled_latency_s": 0.14, "per_stage": []},
            ],
            "validation": {"cpu_modeled_energy_j": 0.027, "measured_energy_j": 0.22,
                           "modeled_over_measured": 0.124},
        }
        md = vem.to_markdown(out)
        self.assertIn("| cpu | FP32 |", md)
        self.assertIn("| kpu_t64 | INT8 |", md)
        self.assertIn("Modeled vs measured", md)
        self.assertIn("0.124", md)

    def test_unknown_sku_is_rejected(self):
        with self.assertRaises(SystemExit):
            vem.main(["/dev/null", "--skus", "definitely_not_a_sku"])


@unittest.skipUnless(_HAS_GRAPHS, "branes-ai/graphs not importable")
class GraphsBacked(unittest.TestCase):
    def test_build_subgraphs(self):
        specs = vem.stage_specs(SAMPLE)
        sgs = vem.build_subgraphs(specs)
        self.assertEqual(len(sgs), len(specs))
        self.assertEqual(sgs[0].total_flops, int(round(specs[0]["flops"])))
        self.assertEqual(sgs[0].total_macs, 0)

    def test_run_all_skus(self):
        out = vem.run(SAMPLE, ["cpu", "gpu", "kpu_t64"])
        self.assertEqual([r["sku"] for r in out["skus"]], ["cpu", "gpu", "kpu_t64"])
        for r in out["skus"]:
            self.assertGreater(r["modeled_energy_j"], 0.0)
            self.assertGreaterEqual(r["modeled_avg_power_w"], 0.0)
        # Measurement present ⇒ a CPU validation block is produced.
        self.assertIn("validation", out)
        self.assertIsNotNone(out["validation"]["modeled_over_measured"])


if __name__ == "__main__":
    unittest.main()
