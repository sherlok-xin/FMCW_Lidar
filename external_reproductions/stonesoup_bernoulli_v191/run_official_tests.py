"""Run only locked upstream Bernoulli component tests; do not alter upstream."""
import datetime
import json
import os
from pathlib import Path
import sys

import numpy as np
import pytest


HERE = Path(__file__).resolve().parent
OUT = HERE.parents[1] / "results/external_reproductions/stonesoup_bernoulli_v191"
SEED = 191


class RecordTests:
    def __init__(self):
        self.nodes = []
        self.reports = []

    def pytest_collection_finish(self, session):
        self.nodes = [item.nodeid for item in session.items]
        print("\nImported official package:", sys.modules["stonesoup"].__file__, flush=True)

    def pytest_runtest_logreport(self, report):
        self.reports.append({
            "nodeid": report.nodeid, "when": report.when,
            "outcome": report.outcome, "duration_seconds": report.duration,
        })


if __name__ == "__main__":
    os.chdir(HERE / "upstream")
    np.random.seed(SEED)
    args = [
        "-vv", "-ra", "--tb=long",
        "--junitxml=" + str(OUT / "official_tests.junit.xml"),
        "-k", "(bernoulli and not vis_informed) or BernoulliParticleState",
        "stonesoup/predictor/tests/test_particle.py",
        "stonesoup/updater/tests/test_particle.py",
        "stonesoup/types/tests/test_state.py",
        "stonesoup/types/tests/test_prediction.py",
    ]
    recorder = RecordTests()
    print("Official unmodified source tests; NumPy seed:", SEED, flush=True)
    print("pytest arguments:", json.dumps(args), flush=True)
    code = int(pytest.main(args, plugins=[recorder]))
    summary = {
        "utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "seed": SEED, "pytest_arguments": args, "exit_code": code,
        "node_ids": recorder.nodes, "reports": recorder.reports,
        "stonesoup_import_path": getattr(sys.modules.get("stonesoup"), "__file__", None),
        "excluded_scope": "VisibilityInformed Bernoulli extension and non-Bernoulli components; "
                          "the task uses the standard scalar-P_D Bernoulli model only.",
        "upstream_source_modified": False,
    }
    (OUT / "official_test_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    sys.exit(code)
