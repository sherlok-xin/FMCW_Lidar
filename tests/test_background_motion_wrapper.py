"""Synthetic reporting-contract tests; no data/labels are loaded."""
import sys
import unittest
import json
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import background_motion_dp_mvp as m


class WrapperTests(unittest.TestCase):
    def test_numpy_comparison_hit_count_is_json_serializable(self):
        nodes=[{"frame":0,"center":np.zeros(3)}]
        count=m.silver_hits((0,),nodes,{0:np.zeros(3)})
        self.assertIs(type(count),int)
        self.assertEqual(json.loads(json.dumps({"hits":count})),{"hits":1})

    def test_rank_is_lower_bound_unless_reference_retained(self):
        path=(0,1,2)
        parts={"cumulative_score":3.,"duration_s":2.}
        pool=[{"score":4.,"path_sha256":"other"},
              {"score":3.,"path_sha256":m.digest(path)}]
        r=m.rank_reference(path,parts,pool,{})
        self.assertEqual(r["strict_exceedance_count"],1)
        self.assertEqual(r["exact_rank_in_representatives"],2)
        self.assertEqual(r["global_combinatorial_rank"],"UNKNOWN")
        pool[1]["path_sha256"]="not_retained"
        r=m.rank_reference(path,parts,pool,{})
        self.assertEqual(r["score_rank_lower_bound"],2)
        self.assertEqual(r["exact_rank_in_representatives"],"UNKNOWN_NOT_RETAINED")

    def test_ineligible_reference_has_unknown_rank(self):
        r=m.rank_reference((0,1),{"cumulative_score":0.,"duration_s":1.},[],{})
        self.assertEqual(r["score_rank_lower_bound"],"UNKNOWN")
        r=m.rank_reference((0,1,2),{"cumulative_score":0.,"duration_s":1.9},[],{})
        self.assertEqual(r["score_rank_lower_bound"],"UNKNOWN")

    def test_real_time_path_score_decomposition(self):
        nodes=[{"time":t,"frame":i,"center":np.array([t,0.,0.]),
            "base_component":.5,"compactness_component":.2,
            "support_component":.1,"background_evidence_term":.7} for i,t in enumerate((0.,1.2,2.9))]
        edges=[{"u":i,"v":i+1,"dt":dt,"gap_penalty":0.,"velocity":np.array([1.,0.,0.])}
               for i,dt in enumerate((1.2,1.7))]
        old=m.path_parts((0,1,2),nodes,edges,"original_score")
        new=m.path_parts((0,1,2),nodes,edges,"background_motion")
        self.assertAlmostEqual(old["cumulative_score"],2.4)
        self.assertAlmostEqual(new["cumulative_score"],4.5)
        self.assertEqual(new["duration_s"],2.9)
        self.assertEqual(new["speed_median_mps"],1.)
        self.assertEqual(new["first_frame"],0)
        self.assertEqual(new["last_frame"],2)


if __name__=="__main__":
    unittest.main()
