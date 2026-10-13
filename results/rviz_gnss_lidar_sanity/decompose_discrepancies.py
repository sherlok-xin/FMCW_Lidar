#!/usr/bin/env python3
"""Describe all 80 frozen GNSS-to-observed-anchor discrepancies; no fitting.

Reads only saved CSV/JSON geometry, never a bag, optimizer, detector or label
writer. The primary orthonormal basis uses full 3D chord tangent, horizontal
left cross-track, and vertical-normal based on the frozen ENU-Up direction.
Strict ENU-Up and horizontal-only along-track are also retained explicitly.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import math
import platform
from pathlib import Path
import sys
from datetime import datetime, timezone
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
ORDER = ['100m_cross_5', '100m_cross_10', '200m_cross_5', '200m_cross_10', '300m_cross_10']
ZERO_TOL = 1e-12  # Numerical zero only; not a research gate or new threshold.
RECIPES = ['GNSS_1m', 'GNSS_3m', 'GNSS_10m', 'ANCHOR_1m', 'ANCHOR_3m']

def read_csv(path):
    return list(csv.DictReader(Path(path).open(encoding='utf-8-sig', newline='')))

def write_csv(path, rows):
    with Path(path).open('w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader(); w.writerows(rows)

def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(4*1024*1024), b''): h.update(b)
    return h.hexdigest()

def vector(row, prefix, suffix, axes='xyz'):
    return np.array([float(row[prefix + a + suffix]) for a in axes])

def basis(velocity, up):
    speed = float(np.linalg.norm(velocity))
    if speed <= ZERO_TOL: raise ValueError('Undefined zero-chord tangent')
    t = velocity / speed
    c = np.cross(up, t)
    if np.linalg.norm(c) <= ZERO_TOL: raise ValueError('Undefined horizontal cross-track for vertical chord')
    c /= np.linalg.norm(c)
    b = np.cross(t, c)
    B = np.column_stack([t, c, b])
    if not np.allclose(B.T @ B, np.eye(3), atol=1e-12, rtol=0):
        raise ValueError('Nonorthonormal diagnostic frame')
    return B, speed

def decompose(error, velocity, up):
    B, speed = basis(velocity, up)
    d = B.T @ error
    return d, speed, B

def self_test():
    # Analytic pitched-chord fixture: exact orthogonality and strict-Up distinction.
    up = np.array([0., 0., 1.]); v = np.array([3., 0., 4.])
    e = np.array([5., 7., -2.])
    d, speed, B = decompose(e, v, up)
    assert np.allclose(d, [1.4, 7., -5.2], atol=1e-12)
    assert math.isclose(speed, 5.)
    assert np.allclose(B @ d, e, atol=1e-12)
    assert math.isclose(float(e @ up), -2.)
    assert math.isclose(d[0]/speed, float(e @ v/(v @ v)))
    # Rigid rotation preserves basis coordinates and time equivalent.
    angle = .43; R = np.array([[math.cos(angle), 0., math.sin(angle)], [0.,1.,0.], [-math.sin(angle),0.,math.cos(angle)]])
    dr, sr, _ = decompose(R@e, R@v, R@up)
    assert np.allclose(dr, d, atol=1e-12) and math.isclose(sr, speed)
    for vv in [np.zeros(3), up]:
        try: basis(vv, up)
        except ValueError: pass
        else: raise AssertionError('Undefined frame was accepted')
    print('Analytic decomposition tests PASS')

def describe(values):
    a = np.asarray(values, dtype=float)
    q1, med, q3 = np.quantile(a, [.25,.5,.75], method='linear')
    aq1, amed, aq3 = np.quantile(np.abs(a), [.25,.5,.75], method='linear')
    return dict(n=len(a), median=float(med), q25=float(q1), q75=float(q3),
                iqr=float(q3-q1), min_signed=float(a.min()), max_signed=float(a.max()),
                median_abs=float(amed), q25_abs=float(aq1), q75_abs=float(aq3), iqr_abs=float(aq3-aq1),
                max_abs=float(np.abs(a).max()))

def build(root):
    source = root/'results/gnss_conditioned_uav_phenotype/run_02'
    paths = {
        'frames': source/'frames.csv', 'regions': source/'region_features.csv',
        'segments': source/'gnss_source_segments.csv',
        'anchors': root/'results/research_dev_v0/silver_labels.csv',
        'calibration': root/'results/calibration_closure/alignment.json',
        'offsets': root/'results/calibration_closure/alignment_results.csv',
    }
    hashes_before = {str(p.relative_to(root)): sha(p) for p in paths.values()}
    frames = {(r['condition_id'], int(r['frame_index'])): r for r in read_csv(paths['frames'])}
    anchors = read_csv(paths['anchors'])
    if len(anchors) != 80: raise ValueError('Frozen anchor count changed')
    if len({(r['condition_id'],int(r['frame_index'])) for r in anchors}) != 80:
        raise ValueError('Nonunique frozen anchors')
    segments = {(r['condition_id'],int(r['segment_index'])): r for r in read_csv(paths['segments'])}
    counts = {(r['condition_id'],int(r['frame_index']),r['alignment_scenario']): int(r['raw_points'])
              for r in read_csv(paths['regions']) if r['region_role']=='UAV_CONDITIONED' and r['alignment_scenario'] in RECIPES}
    calibration = json.loads(paths['calibration'].read_text())['full_6dof']
    R = np.array(calibration['rotation_matrix'], dtype=float)
    if not np.allclose(R.T@R, np.eye(3), atol=1e-12, rtol=0): raise ValueError('Saved R not orthonormal')
    up = R[:,2]
    rows = []
    for a in sorted(anchors, key=lambda a: (ORDER.index(a['condition_id']), int(a['frame_index']))):
        cid, fi = a['condition_id'], int(a['frame_index']); f = frames[cid,fi]
        if a['interpolated'] != 'False' or a['paper_ground_truth'] != 'False' or a['label_status'] != 'DEV_SILVER':
            raise ValueError('Observed-anchor evidence boundary changed')
        if f['gnss_nominal_supported'] != 'True' or f['anchor_available'] != 'True':
            raise ValueError('Missing frozen anchor or nominal GNSS support')
        anchor = vector(a,'center_','_m'); saved_anchor = vector(f,'anchor_center_','_m')
        if not np.array_equal(anchor, saved_anchor) or a['candidate_id'] != f['anchor_candidate_id']:
            raise ValueError('Anchor table differs from frozen frame export')
        if abs(float(a['bag_time_unix'])-float(f['bag_time']))>1e-6: raise ValueError('Anchor/frame record time mismatch')
        g = vector(f,'nominal_center_','_m'); e = anchor-g
        v = vector(f,'nominal_velocity_','_mps'); ve = vector(f,'nominal_enu_velocity_','_mps', 'enu')
        seg = segments[cid,int(f['nominal_segment_index'])]
        seg_v = vector(seg,'chord_velocity_','_mps','enu')
        if not np.allclose(ve, seg_v, atol=1e-12, rtol=0) or not np.allclose(R@ve,v,atol=1e-12,rtol=0):
            raise ValueError('Frozen velocity/segment/calibration mismatch')
        if not math.isclose(float(f['nominal_time_offset_s']),float(f['query_gps_time_unix_s'])-float(f['bag_time']),abs_tol=1e-6):
            raise ValueError('Frozen bag/query/offset mismatch')
        d, speed, B = decompose(e,v,up)
        norm = float(np.linalg.norm(e)); tau = float(d[0]/speed)
        enu_e = R.T@e
        horizontal_speed = float(np.linalg.norm(ve[:2]))
        horizontal_t = np.array([ve[0],ve[1],0.])/horizontal_speed
        horizontal_c = np.cross(np.array([0.,0.,1.]),horizontal_t)
        dh = float(enu_e@horizontal_t); dc = float(enu_e@horizontal_c); du = float(enu_e[2])
        # Sensor analysis +Z is not gravity Up. Retain it as a named sensitivity only.
        sensor_d,_,_ = decompose(e,v,np.array([0.,0.,1.]))
        qt = float(f['query_gps_time_unix_s']); left=float(seg['gps_start_unix_s']); right=float(seg['gps_end_unix_s'])
        supported = [s for (cc,_),s in segments.items() if cc==cid]
        first=min(float(s['gps_start_unix_s']) for s in supported); last=max(float(s['gps_end_unix_s']) for s in supported)
        cf = qt+tau  # Linearized counterfactual clock ONLY; never query/interpolate/refit.
        out = dict(condition_id=cid, frame_index=fi, candidate_id=a['candidate_id'], label_status='DEV_SILVER_OBSERVED_NOT_GT',
                   bag_record_time=f"{f['bag_time_sec']}.{int(f['bag_time_nsec']):09d}",
                   bag_time_sec=int(f['bag_time_sec']),bag_time_nsec=int(f['bag_time_nsec']),
                   header_sec=int(f['header_sec']),header_nsec=int(f['header_nsec']),
                   frozen_time_offset_s=float(f['nominal_time_offset_s']),query_gps_time_unix_s=qt,
                   gnss_segment_index=int(f['nominal_segment_index']),gnss_left_source_cell=f['gnss_left_source_cell'],
                   gnss_right_source_cell=f['gnss_right_source_cell'],segment_dt_s=float(seg['segment_dt_s']),
                   segment_fraction=float(f['nominal_segment_fraction']),
                   nearest_gnss_knot_distance_s=float(f['nearest_gnss_knot_distance_s']),
                   at_last_gnss_endpoint=abs(qt-last)<=1e-6,at_interior_gnss_knot=f['at_interior_gnss_knot'],
                   interpolation_status='FROZEN_PIECEWISE_GNSS_NO_NEW_INTERPOLATION',
                   frame_basis='FROZEN_ENU_UP_3D_TANGENT_HORIZONTAL_LEFT_VERTICAL_NORMAL')
        for prefix, vec in [('gnss_analysis_',g),('anchor_analysis_',anchor),('error_analysis_',e),('chord_velocity_analysis_',v),
                            ('along_unit_analysis_',B[:,0]),('cross_unit_analysis_',B[:,1]),('vertical_normal_unit_analysis_',B[:,2])]:
            for axis,value in zip('xyz',vec): out[prefix+axis]=float(value)
        for axis,value in zip('enu',enu_e):out['error_enu_'+axis+'_m']=float(value)
        for axis,value in zip('enu',ve):out['chord_velocity_enu_'+axis+'_mps']=float(value)
        out.update(discrepancy_norm_m=norm,chord_speed_mps=speed,chord_horizontal_speed_mps=horizontal_speed,
                   chord_inclination_deg=math.degrees(math.asin(ve[2]/speed)),d_parallel_m=float(d[0]),
                   d_cross_horizontal_m=float(d[1]),d_vertical_normal_m=float(d[2]),
                   d_up_enu_m=du,d_parallel_horizontal_m=dh,equivalent_time_bias_s=tau,
                   equivalent_time_bias_horizontal_s=dh/horizontal_speed,
                   counterfactual_query_plus_tau_unix_s=cf,
                   counterfactual_within_same_chord=left<=cf<=right,
                   counterfactual_within_gnss_support=first<=cf<=last,
                   parallel_squared_fraction=float(d[0]**2/norm**2),
                   cross_squared_fraction=float(d[1]**2/norm**2),vertical_normal_squared_fraction=float(d[2]**2/norm**2),
                   error_to_forward_chord_deg=math.degrees(math.acos(np.clip(d[0]/norm,-1,1))),
                   abs_error_to_chord_axis_deg=math.degrees(math.acos(np.clip(abs(d[0])/norm,-1,1))),
                   sensor_z_basis_cross_m=float(sensor_d[1]),sensor_z_basis_vertical_normal_m=float(sensor_d[2]),
                   reconstruction_max_abs_error_m=float(np.max(np.abs(e-B@d))),
                   squared_norm_identity_error_m2=abs(float(d@d)-norm**2),anchor_raw_support_prior=int(a['evidence_raw_points']))
        for n in RECIPES: out[n+'_raw_count']=counts[cid,fi,n]
        if not math.isclose(norm,float(f['anchor_minus_nominal_center_distance_m']),abs_tol=1e-12):raise ValueError('Discrepancy changed')
        if abs(dc-d[1])>1e-11 or abs(d@d-norm**2)>1e-10 or np.max(np.abs(e-B@d))>1e-11:raise ValueError('Decomposition identity failed')
        rows.append(out)
    hashes_after={str(p.relative_to(root)):sha(p) for p in paths.values()}
    if hashes_before!=hashes_after: raise ValueError('Scientific inputs changed during read-only calculation')
    return rows,dict(input_sha256=hashes_before,input_sha256_after=hashes_after,inputs_unchanged=True,
                     frozen_rotation_matrix=R.tolist(),frozen_translation_m=calibration['translation_m'],
                     frozen_up_analysis_xyz=up.tolist(),scalar_statistics='numpy.quantile(method=linear), equally weighted observations; signed median/IQR/min/max plus max absolute',
                     conventions={'error':'anchor minus projected GNSS','along':'3D frozen GNSS chord direction',
                                  'cross':'normalized(frozen ENU Up cross along), positive horizontal left',
                                  'vertical_normal':'along cross horizontal-left, positive near Up; not strict ENU height',
                                  'equivalent_time':'d_parallel / full 3D chord speed = error dot velocity / speed^2; positive means later GNSS on local linear chord, NOT fitted offset',
                                  'strict_up':'error dot frozen R[:,2]','horizontal_along':'horizontal projected chord; orthogonal alternative with cross and strict Up'},
                     no_fit_or_optimization=True,no_bag_reads=True,numerical_zero_tolerance=ZERO_TOL)

def summaries(rows):
    metrics=['discrepancy_norm_m','d_parallel_m','d_cross_horizontal_m','d_vertical_normal_m','d_up_enu_m',
             'd_parallel_horizontal_m','equivalent_time_bias_s','equivalent_time_bias_horizontal_s','chord_speed_mps','chord_inclination_deg']
    summary=[]; directions=[]; chord_summary=[]
    for cid in ORDER:
        rs=[r for r in rows if r['condition_id']==cid]
        for metric in metrics:
            desc=describe([r[metric] for r in rs]);maxrow=max(rs,key=lambda r:abs(r[metric]))
            summary.append(dict(condition_id=cid,metric=metric,**desc,max_abs_frame_index=maxrow['frame_index'],max_abs_value_signed=maxrow[metric]))
        norms=np.array([r['discrepancy_norm_m'] for r in rs])
        errors=np.array([[r['error_analysis_'+a] for a in 'xyz'] for r in rs])
        local=np.array([[r[k] for k in ['d_parallel_m','d_cross_horizontal_m','d_vertical_normal_m']] for r in rs])
        unit_mean=(errors/norms[:,None]).mean(axis=0);local_mean=(local/norms[:,None]).mean(axis=0)
        ds=dict(condition_id=cid,n=len(rs),fixed_frame_unit_vector_resultant=float(np.linalg.norm(unit_mean)),
                chord_frame_unit_vector_resultant=float(np.linalg.norm(local_mean)),
                total_parallel_squared_fraction=float(np.sum(local[:,0]**2)/np.sum(norms**2)),
                total_cross_squared_fraction=float(np.sum(local[:,1]**2)/np.sum(norms**2)),
                total_vertical_normal_squared_fraction=float(np.sum(local[:,2]**2)/np.sum(norms**2)),
                median_absolute_cos_error_chord=float(np.median(np.abs(local[:,0])/norms)),
                counterfactual_outside_same_chord_count=sum(not r['counterfactual_within_same_chord'] for r in rs),
                counterfactual_outside_gnss_support_count=sum(not r['counterfactual_within_gnss_support'] for r in rs))
        for a,value in zip('xyz',unit_mean):ds['fixed_frame_mean_unit_error_'+a]=float(value)
        for metric in ['d_parallel_m','d_cross_horizontal_m','d_vertical_normal_m','d_up_enu_m']:
            vals=np.array([r[metric] for r in rs]);pos=int(np.sum(vals>ZERO_TOL));neg=int(np.sum(vals<-ZERO_TOL));zero=len(vals)-pos-neg
            ds[metric+'_positive_count']=pos;ds[metric+'_negative_count']=neg;ds[metric+'_numerical_zero_count']=zero
            ds[metric+'_majority_sign_fraction']=max(pos,neg)/len(vals)
        for a in 'xyz':ds['median_error_analysis_'+a+'_m']=float(np.median([r['error_analysis_'+a] for r in rs]))
        directions.append(ds)
        for seg in sorted({r['gnss_segment_index'] for r in rs}):
            ss=[r for r in rs if r['gnss_segment_index']==seg];desc=describe([r['equivalent_time_bias_s'] for r in ss])
            chord_summary.append(dict(condition_id=cid,gnss_segment_index=seg,first_frame=min(r['frame_index'] for r in ss),
                                      last_frame=max(r['frame_index'] for r in ss),positive_count=sum(r['d_parallel_m']>ZERO_TOL for r in ss),
                                      negative_count=sum(r['d_parallel_m']<-ZERO_TOL for r in ss),both_signs_in_same_chord=any(r['d_parallel_m']>ZERO_TOL for r in ss) and any(r['d_parallel_m']<-ZERO_TOL for r in ss),**desc))
    return summary,directions,chord_summary

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,default=ROOT);p.add_argument('--out',type=Path,default=Path(__file__).parent)
    p.add_argument('--self-test',action='store_true');args=p.parse_args()
    self_test()
    if args.self_test:return
    args.out.mkdir(parents=True,exist_ok=True)
    rows, provenance=build(args.root.resolve());summary,directions,chords=summaries(rows)
    write_csv(args.out/'all_anchor_gnss_discrepancies.csv',rows)
    write_csv(args.out/'sequence_component_summary.csv',summary)
    write_csv(args.out/'sequence_direction_consistency.csv',directions)
    write_csv(args.out/'chord_time_bias_summary.csv',chords)
    maxrow=max(rows,key=lambda r:r['discrepancy_norm_m'])
    write_csv(args.out/'maximum_discrepancy_neighbors.csv',[r for r in rows if r['condition_id']==maxrow['condition_id'] and 15<=r['frame_index']<=21])
    provenance.update(utc=datetime.now(timezone.utc).isoformat(),command=sys.argv,script_sha256=sha(__file__),
                      python=platform.python_version(),numpy=np.__version__,anchor_count=len(rows),sequence_counts={c:sum(r['condition_id']==c for r in rows) for c in ORDER},
                      maximum_discrepancy=maxrow,all_reconstruction_checks_pass=True,
                      files_sha256={p.name:sha(p) for p in args.out.glob('*.csv')})
    (args.out/'decomposition_provenance.json').write_text(json.dumps(provenance,ensure_ascii=False,indent=2))
    print(json.dumps(dict(anchor_count=len(rows),sequence_counts=provenance['sequence_counts'],max_condition=maxrow['condition_id'],max_frame=maxrow['frame_index'],
                          norm_m=maxrow['discrepancy_norm_m'],along_m=maxrow['d_parallel_m'],cross_m=maxrow['d_cross_horizontal_m'],vertical_normal_m=maxrow['d_vertical_normal_m'],
                          strict_up_m=maxrow['d_up_enu_m'],time_equivalent_s=maxrow['equivalent_time_bias_s']),ensure_ascii=False,indent=2))

if __name__=='__main__': main()
