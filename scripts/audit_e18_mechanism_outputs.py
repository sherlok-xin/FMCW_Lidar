"""Independent read-only E19 accounting checker. Never run a real-data DP."""
import csv
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path

import numpy as np


ROOT = Path('/home/xin/FMCW_LIDAR')
OLD = ROOT / 'results/background_motion_dp_mvp'
NEW = ROOT / 'results/e18_mechanism_generalization_audit'
ARMS = ('original', 'background_only', 'motion_only', 'background_motion_E18')
FLAGS = {'original': (False, False), 'background_only': (True, False),
         'motion_only': (False, True), 'background_motion_E18': (True, True)}
checks = defaultdict(int)


def rows(path):
    with path.open(newline='', encoding='utf-8-sig') as stream:
        return list(csv.DictReader(stream))


def sha(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        while block := stream.read(1048576):
            result.update(block)
    return result.hexdigest()


def digest(path):
    return hashlib.sha256(json.dumps(list(path), separators=(',', ':')).encode()).hexdigest()


def close(a, b):
    assert math.isclose(float(a), float(b), rel_tol=0.0, abs_tol=1e-9), (a, b)


def pathmap(data, kind):
    groups = defaultdict(list)
    for row in data:
        groups[(row['family'], row['condition_id'], row[kind])].append(row)
    return {key: tuple(int(row['node_id']) for row in sorted(group, key=lambda row: int(row['path_order'])))
            for key, group in groups.items()}


def points(row, prefix, suffix):
    return np.asarray([float(row[f'{prefix}{axis}{suffix}']) for axis in 'xyz'])


def describe(path, nodes, edge_lookup, arm):
    background, motion = FLAGS[arm]
    score_old = sum(float(nodes[n]['score']) for n in path)
    bg_raw = sum(float(nodes[n]['background_evidence_term']) for n in path)
    selected_edges = [edge_lookup[(u, v)] for u, v in zip(path[:-1], path[1:])]
    gaps = sum(float(edge['gap_penalty']) for edge in selected_edges)
    speed_penalty = direction_penalty = 0.0
    for edge in selected_edges:
        u, v = int(edge['u']), int(edge['v'])
        dt = float(edge['dt_s'])
        close(dt, float(nodes[v]['bag_time']) - float(nodes[u]['bag_time']))
        gap = int(nodes[v]['frame_index']) - int(nodes[u]['frame_index'])
        assert 1 <= gap <= 3 and 0 < dt <= 3.5
        velocity = points(edge, 'velocity_', '_mps')
        assert np.linalg.norm(velocity) <= 25.0
        np.testing.assert_allclose(velocity,
            (points(nodes[v], 'center_', '_m') - points(nodes[u], 'center_', '_m')) / dt,
            rtol=0, atol=1e-9)
        checks['selected_edge_gate_checks'] += 1
    for previous, edge in zip(selected_edges[:-1], selected_edges[1:]):
        v0, v = points(previous, 'velocity_', '_mps'), points(edge, 'velocity_', '_mps')
        adt = 0.5 * (float(previous['dt_s']) + float(edge['dt_s']))
        assert np.linalg.norm(v - v0) / adt <= 15.0 + 1e-10
        s0, s = float(np.linalg.norm(v0)), float(np.linalg.norm(v))
        product = s0 * s
        speed_penalty += max(0.5 * ((s-s0)/(15.0*adt))**2, 0.0)
        direction_penalty += max(0.5*(product-float(np.dot(v, v0))) /
                                 (product+(1.0/adt)**2), 0.0)
        checks['selected_acceleration_gate_checks'] += 1
    duration = float(nodes[path[-1]]['bag_time']) - float(nodes[path[0]]['bag_time']) if path else 0.0
    if path:
        assert len(path) >= 3 and duration >= 2.0
    return {'old_node_score_total': score_old,
        'background_evidence_term_total': bg_raw, 'background_contribution_total': bg_raw if background else 0.0,
        'gap_penalty_total': gaps, 'speed_penalty_total': speed_penalty,
        'direction_penalty_total': direction_penalty, 'speed_penalty_applied': speed_penalty if motion else 0.0,
        'direction_penalty_applied': direction_penalty if motion else 0.0,
        'cumulative_score': score_old + (bg_raw if background else 0.0) - gaps -
                            (speed_penalty+direction_penalty if motion else 0.0),
        'duration_s': duration, 'path_nodes': len(path)}


def count_hits(path, nodes, labels):
    return sum(int(nodes[n]['frame_index']) in labels and
        np.linalg.norm(points(nodes[n], 'center_', '_m') - labels[int(nodes[n]['frame_index'])]) <= 3.0
        for n in path)


def verify_description(record, path, nodes, lookup):
    expected = describe(path, nodes, lookup, record['arm'])
    for key, value in expected.items():
        close(record[key], value)
        checks['independent_path_component_checks'] += 1
    if 'path_sha256' in record:
        assert record['path_sha256'] == digest(path)


assert (NEW/'evaluation_manifest.json').is_file(), 'Wait for completed E19 evaluation before checking'
for name in ('input_identity.json',):
    for filename, identity in json.loads((NEW/name).read_text()).items():
        assert sha(ROOT/filename) == identity, filename
        checks['protected_file_hashes'] += 1
for manifest_name in ('run_manifest.json', 'evaluation_manifest.json'):
    manifest = json.loads((NEW/manifest_name).read_text())
    for filename, identity in manifest['output_sha256'].items():
        assert sha(ROOT/filename) == identity, (manifest_name, filename)
        checks['manifest_output_hashes'] += 1
run_manifest = json.loads((NEW/'run_manifest.json').read_text())
assert run_manifest['labels_parsed_or_used_for_inference'] is False
assert run_manifest['graph_generation'] is False
assert json.loads((NEW/'protocol.json').read_text())['silver_gate_m'] == 3.0
if (NEW/'manifest_reconciliation.json').is_file():
    reconciliation = json.loads((NEW/'manifest_reconciliation.json').read_text())
    assert reconciliation['status'] == 'BOOKKEEPING_ONLY_RECONCILIATION'
    assert reconciliation['affected_file'] == 'results/e18_mechanism_generalization_audit/generalization_inventory.csv'
    before_path = NEW/reconciliation['original_manifest_preserved_as']
    assert sha(before_path) == reconciliation['original_run_manifest_sha256']
    before = json.loads(before_path.read_text())
    after = dict(run_manifest)
    note = after.pop('metadata_reconciliation')
    assert 'manifest_reconciliation.json' in note
    assert set(before['output_sha256']) == set(after['output_sha256'])
    changed_hashes = [key for key in before['output_sha256']
                      if before['output_sha256'][key] != after['output_sha256'][key]]
    assert changed_hashes == [reconciliation['affected_file']]
    affected = reconciliation['affected_file']
    assert before['output_sha256'][affected] == reconciliation['original_sha256']
    assert after['output_sha256'][affected] == reconciliation['final_sha256'] == sha(ROOT/affected)
    after['output_sha256'] = dict(before['output_sha256'])
    assert after == before, 'Non-inventory manifest fields must remain exact'
    assert reconciliation['all_other_output_hashes_unchanged'] is True
    checks['bookkeeping_reconciliation_exact_diff'] += 1

selected = pathmap(rows(NEW/'selected_paths.csv'), 'arm')
old_selected = pathmap(rows(OLD/'selected_paths.csv'), 'method')
fixed = pathmap(rows(OLD/'fixed_reference_paths.csv'), 'path_type')
summaries = {(r['family'], r['condition_id'], r['arm']): r for r in rows(NEW/'unconditioned_evaluation.csv')}
fixed_scores = rows(NEW/'fixed_path_scores.csv')
old_ranks = {(r['family'], r['condition_id'], r['method'], r['path_type']): r
             for r in rows(OLD/'fixed_path_ranks.csv')}
new_ranks = rows(NEW/'fixed_path_ranks.csv')
frame_evaluation = rows(NEW/'silver_frame_evaluation.csv')
labels_all = rows(ROOT/'results/research_dev_v0/silver_labels.csv')
labels_by_condition = defaultdict(dict)
for row in labels_all:
    labels_by_condition[row['condition_id']][int(row['frame_index'])] = points(row, 'center_', '_m')

graphs = json.loads((OLD/'graph_index.json').read_text())['graphs']
graph_data = {}
for entry in graphs:
    family, cid = entry['family'], entry['condition_id']
    stem = f'{family}__{cid}'
    nodes = rows(OLD/'graphs'/f'{stem}__nodes.csv')
    edges = rows(OLD/'graphs'/f'{stem}__edges.csv')
    graph_data[(family,cid)] = nodes, {(int(e['u']),int(e['v'])): e for e in edges}
    assert len(nodes) == entry['nodes'] and len(edges) == entry['edges']
    assert sha(OLD/'graphs'/f'{stem}__nodes.csv') == entry['node_sha256']
    assert sha(OLD/'graphs'/f'{stem}__edges.csv') == entry['edge_sha256']
    for nid, node in enumerate(nodes):
        assert int(node['node_id']) == nid
        fi = int(node['frame_index'])
        assert float(node['background_prior_frames']) == fi
        hits = float(node['background_prior_hits'])
        assert 0 <= hits <= fi
        probability = (hits+1.0)/(fi+2.0)
        close(node['background_p_beta11'], probability)
        close(node['background_evidence_term'], 1.0-2.0*probability)
        close(node['new_score'], float(node['score'])+1.0-2.0*probability)
        checks['frozen_node_annotation_checks'] += 1
    lookup = graph_data[(family,cid)][1]
    for arm in ARMS:
        key = family,cid,arm
        path, summary = selected[key], summaries[key]
        verify_description(summary,path,nodes,lookup)
        close(summary['dp_best_score'], summary['cumulative_score'])
        hits = count_hits(path,nodes,labels_by_condition[cid])
        assert hits == int(summary['silver_hits'])
        assert int(summary['silver_labels']) == len(labels_by_condition[cid])
        close(summary['silver_recall'],hits/len(labels_by_condition[cid]))
        close(summary['candidate_volume_per_frame'],entry['nodes']/entry['frames'])
        close(summary['output_volume_per_frame'],len(path)/entry['frames'])
        terminal = rows(NEW/'terminal_scores'/f'{stem}__{arm}__full.csv')
        assert len(terminal) == int(summary['qualified_representatives'])
        close(max(float(r['score']) for r in terminal),summary['cumulative_score'])
        checks['full_terminal_pool_records'] += len(terminal)
        if arm in ('original','background_motion_E18'):
            method = 'original_score' if arm == 'original' else 'background_motion'
            assert path == old_selected[(family,cid,method)]
            old_pool = rows(OLD/'winner_pools'/f'{stem}__{method}__full.csv')
            old_scores = {int(r['edge_id']): float(r['score']) for r in old_pool}
            assert old_scores == {int(r['edge_id']):float(r['score']) for r in terminal}
            checks['exact_e18_full_pool_replays'] += 1
        checks['full_selected_paths'] += 1

for record in fixed_scores:
    key = record['family'],record['condition_id']
    nodes,lookup = graph_data[key]
    path = fixed[(*key,record['path_type'])]
    verify_description(record,path,nodes,lookup)
    checks['fixed_reference_path_records'] += 1
fixed_index = {(r['family'],r['condition_id'],r['arm'],r['path_type']):r for r in fixed_scores}
for record in rows(NEW/'score_margins.csv'):
    prefix = record['family'],record['condition_id'],record['arm']
    uav = fixed_index[(*prefix,'silver_feasible')]
    old = fixed_index[(*prefix,'top_clutter_label_span')]
    close(record['uav_minus_old_reference'],float(uav['cumulative_score'])-float(old['cumulative_score']))
    close(record['duration_difference_s'],float(uav['duration_s'])-float(old['duration_s']))
    checks['fixed_same_span_margins'] += 1
for record in new_ranks:
    assert record['scope'] == 'matched_silver_span' and record['global_combinatorial_rank'] == 'UNKNOWN'
    assert int(record['score_rank_lower_bound']) == int(record['strict_exceedance_count'])+1
    if record['reference_retained'] == 'True':
        assert record['exact_rank_in_representatives'] == record['score_rank_lower_bound']
    else:
        assert record['exact_rank_in_representatives'] == 'UNKNOWN_NOT_RETAINED'
    if record['arm'] in ('original','background_motion_E18'):
        method = 'original_score' if record['arm'] == 'original' else 'background_motion'
        prior = old_ranks[(record['family'],record['condition_id'],method,record['path_type'])]
        for column in ('strict_exceedance_count','score_rank_lower_bound','reference_retained','exact_rank_in_representatives'):
            assert record[column] == prior[column], (column,record,prior)
        close(record['path_score'],prior['path_score'])
        stem = f"{record['family']}__{record['condition_id']}__{method}__span.csv"
        old_span = rows(OLD/'winner_pools'/stem)
        above = sum(float(r['score']) > float(record['path_score'])+1e-9 for r in old_span)
        reference = fixed[(record['family'],record['condition_id'],record['path_type'])]
        retained = any(r['path_sha256'] == digest(reference) for r in old_span)
        assert above == int(record['strict_exceedance_count'])
        assert len(old_span) == int(record['representatives'])
        assert retained == (record['reference_retained'] == 'True')
        checks['existing_e18_span_pool_score_recounts'] += 1
        checks['existing_e18_span_rank_replays'] += 1
    checks['rank_semantic_checks'] += 1

for row in frame_evaluation:
    key = row['family'],row['condition_id'],row['arm']
    nodes,_ = graph_data[key[:2]]
    fi = int(row['frame_index'])
    path_frame = {int(nodes[n]['frame_index']):n for n in selected[key]}
    nid = path_frame.get(fi)
    assert float(row['match_gate_m']) == 3.0
    if nid is None:
        assert row['selected_node'] == 'NONE' and row['silver_hit'] == 'False'
    else:
        assert int(row['selected_node']) == nid
        distance = np.linalg.norm(points(nodes[nid],'center_','_m')-labels_by_condition[key[1]][fi])
        close(distance,row['distance_m'])
        assert (row['silver_hit'] == 'True') == (distance <= 3.0)
    checks['silver_frame_checks'] += 1

for row in rows(NEW/'summary_by_range.csv'):
    children = [r for key,r in summaries.items() if key[0] == row['family'] and key[2] == row['arm'] and
                int(r['nominal_range_m']) == int(row['nominal_range_m'])]
    for group_column, child_column in (('silver_hits','silver_hits'),('silver_labels','silver_labels'),
                                     ('candidate_nodes','candidate_nodes'),('output_nodes','path_nodes'),('frames','frames')):
        assert int(row[group_column]) == sum(int(r[child_column]) for r in children)
    close(row['silver_recall'],int(row['silver_hits'])/int(row['silver_labels']))
    checks['range_group_records'] += 1

slow_descriptors = rows(NEW/'short_slow_windows.csv')
expected_windows = set()
for entry in graphs:
    labels = labels_by_condition[entry['condition_id']]
    scale = 1.0/float(np.median(np.diff(entry['times'])))
    segments,segment = [],[]
    for fi in sorted(labels):
        if segment:
            prev = segment[-1]
            coherent = fi == prev+1 and np.linalg.norm(labels[fi]-labels[prev]) / (entry['times'][fi]-entry['times'][prev]) <= scale
            if not coherent:
                segments.append(segment)
                segment = []
        segment.append(fi)
    if segment:
        segments.append(segment)
    for segment in segments:
        if len(segment) >= 3 and entry['times'][segment[-1]]-entry['times'][segment[0]] >= 2.0:
            expected_windows.add((entry['family'],entry['condition_id'],segment[0],segment[-1],len(segment)))
assert expected_windows == {(r['family'],r['condition_id'],int(r['first_frame']),int(r['last_frame']),int(r['observations'])) for r in slow_descriptors}
times_by_graph = {(r['family'],r['condition_id']):r['times'] for r in graphs}
for row in slow_descriptors:
    key = row['family'],row['condition_id']
    times = times_by_graph[key]
    lo,hi = int(row['first_frame']),int(row['last_frame'])
    labels = labels_by_condition[key[1]]
    speeds = [float(np.linalg.norm(labels[i+1]-labels[i])/(times[i+1]-times[i])) for i in range(lo,hi)]
    close(row['inherited_static_scale_mps'],1.0/float(np.median(np.diff(times))))
    close(row['duration_s'],times[hi]-times[lo])
    close(row['silver_speed_min_mps'],min(speeds))
    close(row['silver_speed_max_mps'],max(speeds))
    close(row['silver_speed_median_mps'],float(np.median(speeds)))
    checks['maximal_short_slow_descriptors'] += 1
window_by_id = {(r['family'],r['condition_id'],r['window_id']):r for r in slow_descriptors}
# Slow paths need a fourth key because a condition can contain multiple windows.
slow_groups = defaultdict(list)
for r in rows(NEW/'short_slow_selected_paths.csv'):
    slow_groups[(r['family'],r['condition_id'],r['window_id'],r['arm'])].append(r)
slow_paths = {k:tuple(int(r['node_id']) for r in sorted(v,key=lambda r:int(r['path_order']))) for k,v in slow_groups.items()}
for row in rows(NEW/'short_slow_evaluation.csv'):
    key = row['family'],row['condition_id'],row['window_id'],row['arm']
    descriptor = window_by_id[key[:3]]
    nodes,lookup = graph_data[key[:2]]
    path = slow_paths.get(key,tuple())
    verify_description(row,path,nodes,lookup)
    assert row['occupancy_reset'] == 'False'
    lo,hi = int(descriptor['first_frame']),int(descriptor['last_frame'])
    local_labels = {fi:c for fi,c in labels_by_condition[key[1]].items() if lo <= fi <= hi}
    assert all(lo <= int(nodes[n]['frame_index']) <= hi for n in path)
    assert int(row['silver_hits']) == count_hits(path,nodes,local_labels)
    assert int(row['silver_labels']) == len(local_labels)
    checks['short_slow_output_records'] += 1
for row in rows(NEW/'short_slow_fixed_path_scores.csv'):
    key = row['family'],row['condition_id'],row['window_id']
    descriptor = window_by_id[key]
    nodes,lookup = graph_data[key[:2]]
    lo,hi = int(descriptor['first_frame']),int(descriptor['last_frame'])
    path = tuple(n for n in fixed[(*key[:2],'silver_feasible')] if lo <= int(nodes[n]['frame_index']) <= hi)
    verify_description(row,path,nodes,lookup)
    checks['short_slow_fixed_records'] += 1

if (NEW/'summary_manifest.json').is_file():
    summary_manifest = json.loads((NEW/'summary_manifest.json').read_text())
    for filename,identity in summary_manifest['output_sha256'].items():
        assert sha(ROOT/filename) == identity
        checks['summary_output_hashes'] += 1
    for row in rows(NEW/'term_contrasts.csv'):
        arms = {arm:int(summaries[(row['family'],row['condition_id'],arm)]['silver_hits']) for arm in ARMS}
        for arm in ARMS:
            assert int(row[f'{arm}_hits']) == arms[arm]
        assert int(row['background_increment_without_motion']) == arms['background_only']-arms['original']
        assert int(row['motion_increment_without_background']) == arms['motion_only']-arms['original']
        assert int(row['motion_increment_with_background']) == arms['background_motion_E18']-arms['background_only']
        assert int(row['background_increment_with_motion']) == arms['background_motion_E18']-arms['motion_only']
        checks['selected_path_term_contrasts'] += 1

print(json.dumps({'status':'PASS','checks':dict(checks),
    'coverage_limits':['No real DP rerun; no independent optimality certificate for new two ablation-arm spans.',
        'Span rank original/E18 matched prior E18 artifacts; new arm span ranks checked for semantics only.',
        'Cell causal annotation algebra checked on frozen nodes; original occupancy-history correctness inherited from prior E18 audit.']},indent=2))
