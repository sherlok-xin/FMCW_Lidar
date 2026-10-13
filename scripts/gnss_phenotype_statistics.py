"""Descriptive statistics for oracle-conditioned raw-cloud regions.

This module does not load labels, identify UAV points, train a classifier,
select thresholds, or invoke a detector/tracker. XYZ validity is the caller's
responsibility: nonfinite coordinates are rejected rather than silently masked.
Rich numeric fields retain explicit nonfinite-value accounting.

Canonical region-row metadata used by the paired/run utilities:
``sequence_id, frame_index, time_s, alignment_scenario, region_role,
region_id, gnss_segment_id, gnss_supported, raw_points``.
Roles are ``UAV_CONDITIONED`` and ``LOCAL_CLUTTER_PROXY``. A scenario (and,
where present, ``region_radius_m``) identifies a single conditioning recipe.
Missing-run utilities additionally honor ``closure_supported=False`` as an
observation gap, not a zero-return measurement. Neither support flag certifies
independent target presence or LiDAR FOV coverage.
"""

from collections import defaultdict

import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import minimum_spanning_tree
from scipy.spatial import cKDTree, distance


def _number(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def numeric_descriptors(values, prefix):
    """Finite-only population summaries; nonfinite counts are never discarded.

    Standard deviation uses ddof=0. Empty finite sets produce NaN summaries,
    not zero. ``zero_count`` counts finite exact zeros, not missing values.
    Quantiles use NumPy's default linear interpolation, fixed in this module.
    """
    values = np.asarray(values, dtype=np.float64).reshape(-1)
    finite = values[np.isfinite(values)]
    prefix = str(prefix).rstrip("_")
    result = {
        f"{prefix}_count": int(len(values)),
        f"{prefix}_finite_count": int(len(finite)),
        f"{prefix}_nonfinite_count": int(len(values) - len(finite)),
        f"{prefix}_nan_count": int(np.isnan(values).sum()),
        f"{prefix}_posinf_count": int(np.isposinf(values).sum()),
        f"{prefix}_neginf_count": int(np.isneginf(values).sum()),
        f"{prefix}_zero_count": int(np.sum(finite == 0)),
    }
    names = ("min", "p05", "q25", "median", "q75", "p95", "max")
    quantiles = np.quantile(finite, [0, .05, .25, .5, .75, .95, 1]) if len(finite) else np.full(7, np.nan)
    result.update({f"{prefix}_{name}": float(value) for name, value in zip(names, quantiles)})
    result[f"{prefix}_mean"] = float(np.mean(finite)) if len(finite) else float("nan")
    result[f"{prefix}_std"] = float(np.std(finite, ddof=0)) if len(finite) else float("nan")
    return result


def _covariance_description(points, los, prefix):
    """Population covariance of observed support, not uncertainty covariance."""
    names = (
        "eigenvalue_1_m2", "eigenvalue_2_m2", "eigenvalue_3_m2",
        "radial_variance_m2", "tangential_variance_m2",
    )
    result = {f"{prefix}_{name}": float("nan") for name in names}
    if not len(points):
        return result
    delta = points - np.mean(points, axis=0)
    covariance = delta.T @ delta / len(points)
    eigenvalues = np.maximum(np.linalg.eigvalsh(covariance)[::-1], 0)
    result.update({f"{prefix}_eigenvalue_{i + 1}_m2": float(v) for i, v in enumerate(eigenvalues)})
    if los is not None:
        radial = float(max(los @ covariance @ los, 0))
        result[f"{prefix}_radial_variance_m2"] = radial
        result[f"{prefix}_tangential_variance_m2"] = float(max(np.trace(covariance) - radial, 0))
    return result


def _voxel_mean_description(points, size, los, pairwise_limit):
    """Same-child fixed-origin cell means, not a filtering/candidate operation."""
    prefix = f"voxel_{size:g}m_mean"
    if len(points):
        cells, inverse, children = np.unique(
            np.floor(points / size).astype(np.int64), axis=0,
            return_inverse=True, return_counts=True)
        means = np.column_stack([
            np.bincount(inverse, weights=points[:, i], minlength=len(cells)) / children
            for i in range(3)])
    else:
        children = np.empty(0, dtype=np.int64)
        means = np.empty((0, 3), dtype=np.float64)
    n = len(means)
    result = {
        f"occupied_voxels_{size:g}m": n,
        f"{prefix}_count": n,
        f"{prefix}_raw_child_support": len(points),
        f"{prefix}_collapse_fraction": 1 - n / len(points) if len(points) else float("nan"),
        f"{prefix}_geometry_status": "EMPTY_UNKNOWN" if not n else ("DEGENERATE_NOT_BODY_SIZE" if n == 1 else "OBSERVED_CELL_MEANS_NOT_BODY_SIZE"),
        f"{prefix}_pairwise_budget_exceeded": bool(n > pairwise_limit),
    }
    names = ("span_x_m", "span_y_m", "span_z_m", "max_axis_span_m", "diameter_m", "range_thickness_m", "radial_span_m", "radial_rms_m", "tangential_rms_m")
    result.update({f"{prefix}_{name}": float("nan") for name in names})
    result.update(_covariance_description(means, los, f"{prefix}_cov"))
    result.update(numeric_descriptors(children, f"voxel_{size:g}m_children"))
    if not n:
        return result
    spans = np.ptp(means, axis=0)
    result.update({f"{prefix}_span_{axis}_m": float(v) for axis, v in zip("xyz", spans)})
    result[f"{prefix}_max_axis_span_m"] = float(spans.max())
    result[f"{prefix}_range_thickness_m"] = float(np.ptp(np.linalg.norm(means, axis=1)))
    if n == 1:
        result[f"{prefix}_diameter_m"] = 0.0
    elif n <= pairwise_limit:
        result[f"{prefix}_diameter_m"] = float(distance.pdist(means).max())
    if los is not None:
        delta = means - np.mean(means, axis=0)
        radial = delta @ los
        tangential = delta - radial[:, None] * los
        result[f"{prefix}_radial_span_m"] = float(np.ptp(radial))
        result[f"{prefix}_radial_rms_m"] = float(np.sqrt(np.mean(radial**2)))
        result[f"{prefix}_tangential_rms_m"] = float(np.sqrt(np.mean(np.sum(tangential**2, axis=1))))
    return result


def spatial_descriptors(xyz, center, voxel_sizes=(.04, 1.0), pairwise_limit=2000):
    """Return observed-support geometry without filtering or clustering.

    ``xyz`` and ``center`` are in one sensor-relative Cartesian frame (metres).
    ``center`` defines the LOS and oracle-offset diagnostic, not the covariance
    origin. Voxel occupancy uses fixed zero-origin floor(xyz/size), deliberately
    representation-dependent and not claimed to be rotation/translation invariant.
    Same raw children also produce arithmetic cell means; their unweighted
    observed spans/diameter/covariance describe representation collapse only.

    NN uses every raw/unique point via a full KD-tree. Exact diameter, pairwise
    distances, tangential diameter and unique-XYZ MST are computed only when
    unique count <= pairwise_limit; otherwise they remain NaN with a budget flag.
    No subsampling or approximate diameter is substituted. Exact duplicates
    affect raw NN/covariance but not unique-XYZ MST or diameter.

    Singleton spans/covariance/diameter are mathematically zero, flagged as
    degenerate observed support rather than zero UAV body size. NN/MST for
    fewer than two relevant points are NaN.
    """
    points = np.asarray(xyz, dtype=np.float64)
    if points.size == 0:
        points = np.empty((0, 3), dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("xyz must have shape (n, 3) with finite coordinates; caller must audit invalid XYZ")
    center = np.asarray(center, dtype=np.float64)
    if center.shape != (3,) or not np.isfinite(center).all():
        raise ValueError("center must be three finite sensor-relative coordinates")
    if not isinstance(pairwise_limit, (int, np.integer)) or pairwise_limit < 2:
        raise ValueError("pairwise_limit must be an integer >= 2")
    unique = np.unique(points, axis=0)
    n, nu = len(points), len(unique)
    radius = float(np.linalg.norm(center))
    los = center / radius if radius > 0 else None
    result = {
        "raw_points": n, "unique_xyz_points": nu, "coincident_return_count": n - nu,
        "geometry_status": "EMPTY_UNKNOWN" if not n else ("DEGENERATE_NOT_BODY_SIZE" if nu == 1 else "OBSERVED_SUPPORT_NOT_BODY_SIZE"),
        "los_status": "DEFINED" if los is not None else "UNKNOWN_ZERO_CENTER",
        "pairwise_limit_unique_points": int(pairwise_limit),
        "pairwise_budget_exceeded": bool(nu > pairwise_limit),
        "pairwise_status": "BUDGET_EXCEEDED" if nu > pairwise_limit else ("EXACT" if nu >= 2 else "UNDEFINED_LESS_THAN_TWO_UNIQUE"),
    }
    for size in voxel_sizes:
        size = float(size)
        if not np.isfinite(size) or size <= 0:
            raise ValueError("voxel sizes must be finite and positive")
        result.update(_voxel_mean_description(points, size, los, pairwise_limit))
    scalar_names = (
        "span_x_m", "span_y_m", "span_z_m", "max_axis_span_m", "diameter_m",
        "range_thickness_m", "mst_connectivity_edge_max_m", "mst_total_length_m",
        "radial_span_m", "tangential_diameter_m", "radial_rms_m", "tangential_rms_m",
        "mean_offset_x_m", "mean_offset_y_m", "mean_offset_z_m", "mean_oracle_offset_norm_m",
    )
    result.update({name: float("nan") for name in scalar_names})
    result["mst_edge_count"] = 0
    raw_nn = cKDTree(points).query(points, k=2)[0][:, 1] if n >= 2 else np.empty(0)
    unique_nn = cKDTree(unique).query(unique, k=2)[0][:, 1] if nu >= 2 else np.empty(0)
    result.update(numeric_descriptors(raw_nn, "nn_m"))
    result.update(numeric_descriptors(unique_nn, "unique_nn_m"))
    result.update(_covariance_description(points, los, "raw_cov"))
    result.update(_covariance_description(unique, los, "unique_cov"))
    result.update(numeric_descriptors([], "unique_pairwise_m"))
    if not n:
        return result
    spans = np.ptp(points, axis=0)
    result.update({f"span_{axis}_m": float(v) for axis, v in zip("xyz", spans)})
    result["max_axis_span_m"] = float(spans.max())
    result["range_thickness_m"] = float(np.ptp(np.linalg.norm(points, axis=1)))
    mean = np.mean(points, axis=0)
    offset = mean - center
    result.update({f"mean_offset_{axis}_m": float(v) for axis, v in zip("xyz", offset)})
    result["mean_oracle_offset_norm_m"] = float(np.linalg.norm(offset))
    tangent_unique = None
    if los is not None:
        delta = points - mean
        radial = delta @ los
        tangential = delta - radial[:, None] * los
        result["radial_span_m"] = float(np.ptp(radial))
        result["radial_rms_m"] = float(np.sqrt(np.mean(radial**2)))
        result["tangential_rms_m"] = float(np.sqrt(np.mean(np.sum(tangential**2, axis=1))))
        du = unique - np.mean(unique, axis=0)
        tangent_unique = du - (du @ los)[:, None] * los
    if nu == 1:
        result["diameter_m"] = 0.0
        if los is not None:
            result["tangential_diameter_m"] = 0.0
    elif nu <= pairwise_limit:
        pairs = distance.pdist(unique)
        result.update(numeric_descriptors(pairs, "unique_pairwise_m"))
        result["diameter_m"] = float(pairs.max())
        # Sparse conversion avoids scipy's dense csgraph near-zero truncation.
        edges = minimum_spanning_tree(csr_matrix(distance.squareform(pairs))).data
        if len(edges) != nu - 1:
            raise ValueError("unique-XYZ MST is not connected")
        result["mst_edge_count"] = int(len(edges))
        result["mst_connectivity_edge_max_m"] = float(edges.max())
        result["mst_total_length_m"] = float(edges.sum())
        if tangent_unique is not None:
            result["tangential_diameter_m"] = float(distance.pdist(tangent_unique).max())
    return result


def _scenario(row):
    return row.get("alignment_scenario", row.get("scenario_id", row.get("scenario", "UNKNOWN")))


def _role(row):
    role = str(row.get("region_role", row.get("role", ""))).upper()
    if role in {"UAV_CONDITIONED", "UAV", "TARGET", "TARGET_PROXY", "UAV_CONDITIONED_PROXY"}:
        return "TARGET"
    if role in {"LOCAL_CLUTTER_PROXY", "CLUTTER_PROXY", "CONTROL", "LOCAL_CONTROL"}:
        return "CONTROL"
    return "UNKNOWN"


def _frame_identity(row):
    return (
        row.get("sequence_id", "UNKNOWN"), row.get("frame_index", row.get("time_s")),
        _scenario(row), row.get("region_radius_m", row.get("target_radius_m", "UNKNOWN")),
    )


def matched_effects(region_rows, features, group_keys):
    """Same-frame, equal-frame-weight exploratory AUC/effect summaries.

    Group metadata must be copied from the target conditioning recipe onto its
    controls. Every (sequence/frame/scenario/radius) has at most one target row.
    Different recipes require different scenarios/radii, not duplicate targets.
    AUC_i is the mean indicator(target>control)+0.5*tie over finite controls in
    frame i; reported AUC is mean_i AUC_i, not a point-weighted or pooled AUC.
    Direction is always higher-UAV; it is never flipped when AUC < 0.5.

    Paired delta_i = target_i - median(valid controls_i). ``iqr_scale_effect``
    divides median(delta_i) by the average IQR of target_i and matched control
    medians. Zero scale => NaN, not infinite separation. These summaries are
    exploratory proxy-region descriptions, not classifier performance.
    No p-values, confidence intervals or score thresholds are calculated.
    """
    groups = defaultdict(list)
    group_keys = tuple(group_keys)
    for row in region_rows:
        if _role(row) in {"TARGET", "CONTROL"}:
            groups[tuple(row.get(k, "UNKNOWN") for k in group_keys)].append(row)
    output = []
    for group, rows in sorted(groups.items(), key=lambda item: repr(item[0])):
        frames = defaultdict(lambda: {"targets": [], "controls": []})
        for row in rows:
            frames[_frame_identity(row)]["targets" if _role(row) == "TARGET" else "controls"].append(row)
        for frame in frames.values():
            if len(frame["targets"]) > 1:
                raise ValueError("multiple target rows per frame/scenario/radius; recipes must have separate identifiers")
        targets = [f for f in frames.values() if f["targets"]]
        for feature in features:
            frame_aucs, deltas, target_values, control_medians = [], [], [], []
            controls_total = controls_nonfinite = valid_pairs = nonfinite_target_frames = 0
            no_control_frames = no_finite_control_frames = 0
            for frame in targets:
                target = _number(frame["targets"][0].get(feature))
                raw_controls = [_number(r.get(feature)) for r in frame["controls"]]
                controls_total += len(raw_controls)
                controls = np.asarray([v for v in raw_controls if np.isfinite(v)])
                controls_nonfinite += len(raw_controls) - len(controls)
                if not np.isfinite(target):
                    nonfinite_target_frames += 1
                if not raw_controls:
                    no_control_frames += 1
                elif not len(controls):
                    no_finite_control_frames += 1
                if not np.isfinite(target) or not len(controls):
                    continue
                frame_aucs.append(float(np.mean((target > controls).astype(float) + .5 * (target == controls))))
                control_median = float(np.median(controls))
                target_values.append(target)
                control_medians.append(control_median)
                deltas.append(target - control_median)
                valid_pairs += len(controls)
            auc = float(np.mean(frame_aucs)) if frame_aucs else float("nan")
            delta_stats = numeric_descriptors(deltas, "paired_delta")
            tv = numeric_descriptors(target_values, "matched_target")
            cv = numeric_descriptors(control_medians, "matched_control_median")
            scale = .5 * ((tv["matched_target_q75"] - tv["matched_target_q25"]) + (cv["matched_control_median_q75"] - cv["matched_control_median_q25"]))
            output.append({
                **dict(zip(group_keys, group)), "feature": feature,
                "orientation": "HIGHER_UAV_FIXED_NOT_SELECTED", "weighting": "EQUAL_FRAME_THEN_EQUAL_VALID_CONTROLS",
                "frames_total": len(frames), "target_frames_total": len(targets),
                "control_only_frames": len(frames) - len(targets),
                "valid_frames": len(frame_aucs), "excluded_frames": len(targets) - len(frame_aucs),
                "nonfinite_target_frames": nonfinite_target_frames,
                "frames_without_controls": no_control_frames,
                "frames_without_finite_controls": no_finite_control_frames,
                "controls_total": controls_total, "controls_nonfinite": controls_nonfinite,
                "valid_pairs": valid_pairs, "excluded_pairs": controls_total - valid_pairs,
                "matched_auc_higher_uav": auc, "cliffs_delta": 2 * auc - 1,
                "iqr_scale": float(scale),
                "iqr_scale_effect": float(delta_stats["paired_delta_median"] / scale) if np.isfinite(scale) and scale > 0 else float("nan"),
                **delta_stats, **tv, **cv,
                "interpretation": "EXPLORATORY_CONDITIONED_REGION_VS_LOCAL_CLUTTER_PROXY_NOT_TEST_PERFORMANCE",
            })
    return output


def _truth(value):
    if isinstance(value, str):
        return value.strip().lower() in {"true", "1", "yes", "supported", "valid"}
    if value is None:
        return False
    return bool(value)


def _supported(row):
    for key in ("gnss_supported", "closure_supported", "region_observation_available"):
        if key in row and not _truth(row[key]):
            return False
    count = _number(row.get("raw_points", row.get("raw_nonzero_return_count")))
    return np.isfinite(count) and count >= 0


def _segment(row):
    return row.get("gnss_segment_id", row.get("segment_id", "UNKNOWN"))


def zero_run_records(frame_region_rows):
    """Observed zero-region runs plus explicit observation gaps, in real time.

    Streams are sequence/scenario/region/radius-specific. A run cannot cross a
    GNSS segment, an unsupported/closure-absent row, a missing frame index, or a
    time outage >2*median bag dt. Median dt uses unique sequence frame times;
    an explicit positive ``bag_median_dt_s`` overrides that estimate if supplied.
    Cadence from a short/sparse stream may itself be uncertain: it is reported.

    Observed span is last-minus-first zero-sample time (a lower bound), never
    framecount times a fabricated dt. Adjacent supported nonzero samples bracket
    start/end only within the same contiguous segment. Boundary/gap runs are
    censored. Unsupported rows and inferred missing samples are OBSERVATION_GAP,
    never treated as zero returns or independent target absence.
    """
    rows = list(frame_region_rows)
    bag_times = defaultdict(set)
    supplied_cadences = defaultdict(set)
    streams = defaultdict(list)
    for row in rows:
        sequence = row.get("sequence_id", "UNKNOWN")
        t = _number(row.get("time_s", row.get("lidar_time_s")))
        if np.isfinite(t):
            bag_times[sequence].add(t)
        supplied_dt = _number(row.get("bag_median_dt_s"))
        if np.isfinite(supplied_dt) and supplied_dt > 0:
            supplied_cadences[sequence].add(supplied_dt)
        stream = (sequence, _scenario(row), row.get("region_id", _role(row)), row.get("region_radius_m", row.get("target_radius_m", "UNKNOWN")))
        streams[stream].append(row)
    cadence = {}
    for sequence, times in bag_times.items():
        supplied = supplied_cadences[sequence]
        if len(supplied) > 1:
            raise ValueError("conflicting supplied bag_median_dt_s in one sequence")
        dt = np.diff(sorted(times))
        positive = dt[dt > 0]
        cadence[sequence] = next(iter(supplied)) if supplied else (float(np.median(positive)) if len(positive) else float("nan"))
    output = []
    for stream, samples in sorted(streams.items(), key=lambda item: repr(item[0])):
        sequence, scenario, region_id, region_radius = stream
        samples.sort(key=lambda r: (_number(r.get("time_s", r.get("lidar_time_s"))), _number(r.get("frame_index"))))
        dt_median = cadence.get(sequence, float("nan"))
        context = {
            "sequence_id": sequence, "alignment_scenario": scenario, "region_id": region_id,
            "region_radius_m": region_radius, "region_role": samples[0].get("region_role", samples[0].get("role", "UNKNOWN")),
            "bag_median_dt_s": dt_median,
        }

        def time(row):
            return _number(row.get("time_s", row.get("lidar_time_s")))

        def boundary(previous, current):
            if previous is None:
                return "STREAM_BOUNDARY"
            if _segment(previous) != _segment(current):
                return "GNSS_SEGMENT_CHANGE"
            old_index, new_index = _number(previous.get("frame_index")), _number(current.get("frame_index"))
            if np.isfinite(old_index) and np.isfinite(new_index) and new_index != old_index + 1:
                return "MISSING_OR_NONCONTIGUOUS_FRAME_INDEX"
            interval = time(current) - time(previous)
            if not np.isfinite(interval) or interval <= 0:
                return "INVALID_OR_NONINCREASING_TIME"
            if np.isfinite(dt_median) and interval > 2 * dt_median:
                return "FRAME_TIME_OUTAGE"
            return None

        current_run, prior_row, run_left = [], None, None
        run_left_reason = "STREAM_BOUNDARY"

        def emit_run(right_row=None, right_reason="STREAM_BOUNDARY"):
            if not current_run:
                return
            first, last = current_run[0], current_run[-1]
            left_valid = run_left is not None and _supported(run_left) and boundary(run_left, first) is None
            right_valid = right_row is not None and _supported(right_row) and boundary(last, right_row) is None
            left_time = time(run_left) if left_valid else float("nan")
            right_time = time(right_row) if right_valid else float("nan")
            output.append({
                **context, "record_kind": "ZERO_REGION_RUN", "gnss_segment_id": _segment(first),
                "first_frame_index": first.get("frame_index", "UNKNOWN"), "last_frame_index": last.get("frame_index", "UNKNOWN"),
                "frame_count": len(current_run), "start_time_s": time(first), "end_time_s": time(last),
                "observed_span_s": time(last) - time(first),
                "left_censored": not left_valid, "right_censored": not right_valid,
                "left_bracket_time_s": left_time, "right_bracket_time_s": right_time,
                "duration_upper_bound_s": right_time - left_time if left_valid and right_valid else float("nan"),
                "start_reason": "SUPPORTED_NONZERO_BRACKET" if left_valid else run_left_reason,
                "end_reason": "SUPPORTED_NONZERO_BRACKET" if right_valid else right_reason,
                "interpretation": "ZERO_COUNT_IN_ORACLE_REGION_NOT_SENSOR_MISS_OR_VERIFIED_ABSENCE",
            })

        gap_rows = []

        def emit_gap():
            if not gap_rows:
                return
            first, last = gap_rows[0], gap_rows[-1]
            output.append({
                **context, "record_kind": "OBSERVATION_GAP", "gnss_segment_id": _segment(first),
                "first_frame_index": first.get("frame_index", "UNKNOWN"), "last_frame_index": last.get("frame_index", "UNKNOWN"),
                "frame_count": len(gap_rows), "start_time_s": time(first), "end_time_s": time(last),
                "observed_span_s": time(last) - time(first), "left_censored": True, "right_censored": True,
                "left_bracket_time_s": float("nan"), "right_bracket_time_s": float("nan"),
                "duration_upper_bound_s": float("nan"), "start_reason": "UNSUPPORTED_OR_CLOSURE_ABSENT",
                "end_reason": "UNSUPPORTED_OR_CLOSURE_ABSENT",
                "interpretation": "NO_USABLE_CONDITIONED_OBSERVATION_NOT_ZERO_RETURN_EVIDENCE",
            })

        for row in samples:
            reason = boundary(prior_row, row)
            if reason is not None and prior_row is not None:
                emit_run(right_reason=reason)
                current_run = []
                emit_gap()
                gap_rows = []
                if reason in {"MISSING_OR_NONCONTIGUOUS_FRAME_INDEX", "FRAME_TIME_OUTAGE", "INVALID_OR_NONINCREASING_TIME"}:
                    output.append({
                        **context, "record_kind": "OBSERVATION_GAP", "gnss_segment_id": "UNKNOWN_INTERVAL",
                        "first_frame_index": "UNKNOWN", "last_frame_index": "UNKNOWN", "frame_count": None,
                        "start_time_s": time(prior_row), "end_time_s": time(row),
                        "observed_span_s": time(row) - time(prior_row), "left_censored": True, "right_censored": True,
                        "left_bracket_time_s": float("nan"), "right_bracket_time_s": float("nan"),
                        "duration_upper_bound_s": float("nan"), "start_reason": reason, "end_reason": reason,
                        "interpretation": "UNOBSERVED_INTERVAL_NOT_ZERO_RETURN_EVIDENCE",
                    })
            if not _supported(row) or not np.isfinite(time(row)):
                emit_run(right_reason="UNSUPPORTED_OR_CLOSURE_ABSENT")
                current_run = []
                gap_rows.append(row)
            else:
                emit_gap()
                gap_rows = []
                count = _number(row.get("raw_points", row.get("raw_nonzero_return_count")))
                if count == 0:
                    if not current_run:
                        run_left = prior_row if reason is None and prior_row is not None and _supported(prior_row) else None
                        run_left_reason = reason or "UNSUPPORTED_OR_CLOSURE_ABSENT"
                    current_run.append(row)
                else:
                    emit_run(right_row=row)
                    current_run = []
            prior_row = row
        emit_run()
        emit_gap()
    return output
