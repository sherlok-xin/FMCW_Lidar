"""One frozen scalar semantic gate, not a synthetic multi-frame experiment.

Never correct the upstream posterior. Exit 2 means the observed-empty-scan
Bayesian existence check failed and the reproduction must stop.
"""
from datetime import datetime, timedelta, timezone
from decimal import Decimal, localcontext
import hashlib
import json
import math
from pathlib import Path
import sys


HERE = Path(__file__).resolve().parent
OUT = HERE.parents[1] / "results/external_reproductions/stonesoup_bernoulli_v191"


def main():
    protocol_bytes = (OUT / "protocol.json").read_bytes()
    config = json.loads(protocol_bytes)
    digest = hashlib.sha256(protocol_bytes).hexdigest()
    assert digest == json.loads((OUT / "protocol_receipt.json").read_text())["protocol_sha256"]
    tests = json.loads((OUT / "official_test_summary.json").read_text())
    assert tests["exit_code"] == 0 and len(tests["node_ids"]) == 7

    # Independent ordinary binary64 arithmetic, before importing Stone Soup.
    previous = float(config["r_prev"])
    birth = float(config["p_birth"])
    survival = float(config["p_survival"])
    detection = float(config["P_D"])
    predicted = birth * (1 - previous) + survival * previous
    expected_empty = predicted * (1 - detection) / (1 - predicted * detection)
    with localcontext() as context:
        context.prec = 60
        r, b, s, pd = (Decimal(str(x)) for x in (previous, birth, survival, detection))
        dp = b * (1 - r) + s * r
        de = dp * (1 - pd) / (1 - dp * pd)
    reference = {
        "calculated_at_utc": datetime.now(timezone.utc).isoformat(),
        "protocol_sha256": digest,
        "arithmetic": "Python ordinary float / IEEE binary64; Decimal60 crosscheck only",
        "r_prev": previous, "p_birth": birth, "p_survival": survival, "P_D": detection,
        "r_pred_binary64": predicted, "r_empty_binary64": expected_empty,
        "r_pred_float_hex": predicted.hex(), "r_empty_float_hex": expected_empty.hex(),
        "r_pred_decimal60": str(dp), "r_empty_decimal60": str(de),
        "binary64_vs_decimal_abs_error": abs(expected_empty - float(de)),
        "float_mantissa_bits": sys.float_info.mant_dig,
        "independent_of_stonesoup": True,
    }
    (OUT / "closed_form_scalar_reference.json").write_text(json.dumps(reference, indent=2) + "\n")

    import numpy as np
    from stonesoup.models.measurement.linear import LinearGaussian
    from stonesoup.models.transition.linear import ConstantVelocity
    from stonesoup.predictor.particle import BernoulliParticlePredictor
    from stonesoup.resampler.particle import SystematicResampler
    from stonesoup.sampler.detection import GaussianDetectionParticleSampler, SwitchingDetectionSampler
    from stonesoup.sampler.particle import ParticleSampler
    from stonesoup.types.array import StateVectors
    from stonesoup.types.detection import MissedDetection
    from stonesoup.types.hypothesis import SingleHypothesis
    from stonesoup.types.multihypothesis import MultipleHypothesis
    from stonesoup.types.state import BernoulliParticleState
    from stonesoup.updater.particle import BernoulliParticleUpdater

    np.random.seed(config["rng_seed"])
    n = config["prior"]["particle_count"]
    nb = config["birth_model"]["birth_particle_count"]
    t0 = datetime.fromisoformat(config["timestamp_prev"])
    t1 = t0 + timedelta(seconds=config["dt_seconds"])
    prior = BernoulliParticleState(
        StateVectors(np.array([config["prior"]["position_grid_m"], np.zeros(n)])),
        weight=np.full(n, 1 / n), existence_probability=previous, timestamp=t0,
    )
    fallback = ParticleSampler(
        distribution_func=np.random.uniform,
        params={"low": np.array(config["birth_model"]["uniform_low"]),
                "high": np.array(config["birth_model"]["uniform_high"]), "size": (nb, 2)},
        ndim_state=2,
    )
    sampler = SwitchingDetectionSampler(
        detection_sampler=GaussianDetectionParticleSampler(nb), backup_sampler=fallback,
    )
    predictor = BernoulliParticlePredictor(
        transition_model=ConstantVelocity(noise_diff_coeff=config["process_noise_diffusion"]),
        birth_sampler=sampler, birth_probability=birth, survival_probability=survival,
    )
    prediction = predictor.predict(prior, timestamp=t1)
    updater = BernoulliParticleUpdater(
        measurement_model=LinearGaussian(ndim_state=2, mapping=(0,),
                                        noise_covar=np.array([[config["measurement_noise_variance_m2"]]])),
        clutter_rate=config["clutter_rate_per_scan"],
        clutter_distribution=config["clutter_spatial_density_per_m"],
        detection_probability=detection,
        resampler=SystematicResampler(), nsurv_particles=config["updater"]["nsurv_particles"],
        regulariser=None, constraint_func=None,
    )
    hypotheses = MultipleHypothesis([
        SingleHypothesis(prediction, MissedDetection(timestamp=t1)),
    ])
    update = updater.update(hypotheses)

    def particle_summary(state):
        weights = np.asarray(state.weight, dtype=float)
        return {
            "count": len(state), "weights": weights.tolist(),
            "weights_finite": bool(np.isfinite(weights).all()),
            "weights_nonnegative": bool((weights >= 0).all()),
            "weights_sum": float(weights.sum()),
            "effective_sample_size": float(1 / np.square(weights).sum()),
            "conditional_mean": np.asarray(state.mean, dtype=float).ravel().tolist(),
        }

    actual_pred = float(prediction.existence_probability)
    actual_post = float(update.existence_probability)
    pred_matches = math.isclose(actual_pred, predicted, rel_tol=config["tolerance_relative"],
                                abs_tol=config["tolerance_absolute"])
    post_matches = math.isclose(actual_post, expected_empty, rel_tol=config["tolerance_relative"],
                                abs_tol=config["tolerance_absolute"])
    result = {
        **reference,
        "executed_at_utc": datetime.now(timezone.utc).isoformat(),
        "legal_hypothesis_container": type(hypotheses).__name__,
        "legal_hypothesis_count": len(hypotheses),
        "measurement_type": type(hypotheses[0].measurement).__name__,
        "single_hypothesis_bool": bool(hypotheses[0]), "any_hypotheses": any(hypotheses),
        "measurement_set_size": 0, "observation_executed": True,
        "official_predictor_r_pred": actual_pred, "official_updater_r_post": actual_post,
        "predictor_matches_closed_form": pred_matches,
        "updater_matches_empty_scan_closed_form": post_matches,
        "absolute_empty_scan_discrepancy": abs(actual_post - expected_empty),
        "existence_finite_and_bounded": all(math.isfinite(x) and 0 <= x <= 1
                                             for x in (actual_pred, actual_post)),
        "prediction_particles": particle_summary(prediction),
        "update_particles": particle_summary(update),
        "no_truth_state_or_presence_input": True,
        "upstream_algorithm_changed": False,
        "additional_model_terms_explain_discrepancy": False,
        "reason": "Constant positive P_D, observed empty set, normalized conditional density. "
                  "Poisson empty-clutter likelihood cancels. Source guard skips measurement correction.",
        "status": "PASS" if pred_matches and post_matches else "STOPPED_AT_EMPTY_SCAN_SEMANTIC_GATE",
    }
    (OUT / "closed_form_check.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2), flush=True)
    return 0 if pred_matches and post_matches else 2


if __name__ == "__main__":
    sys.exit(main())
