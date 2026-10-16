"""Forensic bookkeeping from frozen choices; no filter or full tracker execution."""
import ast,csv,hashlib,json,re,contextlib
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import open3d as o3d
from scipy.spatial import KDTree
R=Path(__file__).resolve().parents[2];O=Path(__file__).resolve().parent
B=R/'results/historical_pcd_baseline_reproduction';CACHE=Path('/tmp/fmcw_historical_pcd_viewer/frames');SRC=R/'target_track_without_initialize_and_grid.py'
def rows(p):return list(csv.DictReader(p.open()))
def write(name,data):
 with (O/name).open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=list(dict.fromkeys(k for r in data for k in r)));w.writeheader();w.writerows(data)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
protected={str(p):sha(p) for p in [SRC,*sorted(x for x in B.rglob('*') if x.is_file())]}
(O/'frozen_input_identity.json').write_text(json.dumps(protected,indent=2)+'\n')
T=rows(B/'tracker_trace.csv');END=rows(B/'track_terminal_states.csv');A=rows(B/'association_trace.csv');DB=rows(B/'dbscan_trace.csv');P=rows(B/'pending_trace.csv');SUM=rows(B/'track_summary.csv')
source=SRC.read_text();assert sha(SRC)==json.loads((B/'source_identity.json').read_text())['source_sha256'][SRC.name]
def cluster(points):
 if not len(points):return np.array([],dtype=int)
 pc=o3d.geometry.PointCloud();pc.points=o3d.utility.Vector3dVector(points)
 return np.asarray(pc.cluster_dbscan(eps=2.,min_points=1,print_progress=False))
# Birth components are from the frozen frame9 refined point set, not the post-update tracker centers.
with np.load(CACHE/'target_track_without_initialize_and_grid_009.npz') as s:points=s['D'];vel=s['Dv']
labels=cluster(points);births=[];SUPPORT={}
for tid,label in enumerate(set(labels.tolist())-{-1}):
 idx=np.flatnonzero(labels==label);cp=points[idx];c=np.array([np.mean(cp[:,j]) for j in range(3)]);v=float(vel[idx].mean())
 births.append({'track_id':tid,'first_scan':9,'birth_mode':'single_scan_global_initialization','component_label':int(label),'component_points':len(idx),'x':float(c[0]),'y':float(c[1]),'z':float(c[2]),'mean_velocity':v,'distinct_birth_evidence_scans':1,'bypasses_pending':True,'component_point_indices_json':json.dumps(idx.tolist()),'component_to_ID_provenance':'original set(labels) traversal, checked against all15 frozen initialization log lines'})
 SUPPORT[(tid,9,'birth_seed')]=idx.tolist()
assert len(births)==15 and len(points)==35
# Check printed 2dp initializer values and exact cluster counts against frozen stdout.
text=(B/'target_track_without_initialize_and_grid_stdout.txt').read_text()
for b in births:
 m=re.search(r'初始化目标 ID '+str(b['track_id'])+r': 位置\(([-\d.]+), ([-\d.]+)\), 点数: (\d+), 初始速度: ([-\d.]+)',text);assert m
 assert (f"{b['x']:.2f}",f"{b['y']:.2f}",str(b['component_points']),f"{b['mean_velocity']:.2f}")==m.groups(),b
# Execute ONLY original pending bookkeeping, with original fixed constants, on frozen candidate memberships.
tree=ast.parse(source);cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='MultiTargetTracker');fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='track_frame')
start=next(i for i,n in enumerate(fn.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='matched_pending' for t in n.targets))
stop=next(i for i,n in enumerate(fn.body) if isinstance(n,ast.Expr) and isinstance(n.value,ast.Call) and isinstance(n.value.func,ast.Attribute) and n.value.func.attr=='_remove_inactive_trackers')
code=compile(ast.Module(body=fn.body[start:stop],type_ignores=[]),str(SRC),'exec')
state=SimpleNamespace(pending_trackers={},pending_id_counter=0,pending_min_frames=6,pending_max_lost_frames=3,max_association_distance=30.,search_radius=15.,min_points=1,target_id_counter=15,trackers=[])
promotions=[];events=[];dups=[];candidate_log=[];checks=[]
current=9
(O/'pending_bookkeeping_stdout.txt').write_text('')
# The callback only observes original promotion locals and captures no new association decisions.
def capture_birth(**kwargs):
 import inspect
 loc=inspect.currentframe().f_back.f_locals;pid=loc['best_pid'];obs=loc['obs_list'];scans=[x['scan_index'] for x in obs];last=scans[-6:]
 promotions.append({'track_id':kwargs['target_id'],'pending_id':pid,'promotion_scan':current,'stored_observations':len(obs),'distinct_scan_indices':len(set(scans)),'all_scan_indices_json':json.dumps(scans),'last6_scan_indices_json':json.dumps(last),'last6_distinct_scans':len(set(last)),'promoted_without_6_distinct_total_frames':len(set(scans))<6,'repeated_same_scan_observations':len(scans)-len(set(scans)),'verified_against_frozen_pending_and_birth_logs':True})
 c=np.array([np.mean(kwargs['initial_points'][:,j]) for j in range(3)])
 births.append({'track_id':kwargs['target_id'],'first_scan':current,'birth_mode':'pending_promotion','component_label':'','component_points':len(kwargs['initial_points']),'x':float(c[0]),'y':float(c[1]),'z':float(c[2]),'mean_velocity':float(np.mean(kwargs['initial_velocities'])),'distinct_birth_evidence_scans':len(set(scans)),'bypasses_pending':False,'component_point_indices_json':json.dumps(loc['cand']['indices']),'component_to_ID_provenance':'original pending block and frozen promoted tracker center'})
 SUPPORT[(kwargs['target_id'],current,'pending_birth')]=loc['cand']['indices']
 return SimpleNamespace(**kwargs)
for current in range(9,61):
 with np.load(CACHE/f'target_track_without_initialize_and_grid_{current:03d}.npz') as s:pts=s['D'];vs=s['Dv']
 used=set();selected=[];localcalls=[]
 groups={}
 for a in A:
  if int(a['frame_index'])==current:groups.setdefault(int(a['target_id']),[]).append(a)
 for tid,aa in groups.items():
  query=[float(aa[0]['association_query_x']),float(aa[0]['association_query_y'])]
  ids=np.array(KDTree(pts[:,:2]).query_ball_point(query,30.),dtype=int)
  lab=cluster(pts[ids]);localcalls.append((tid,len(ids),len(set(lab.tolist())-{-1})))
  for a in aa:
   if a['selected']!='True':continue
   matches=[]
   for label in set(lab.tolist())-{-1}:
    j=ids[lab==label];c=np.array([np.mean(pts[j][:,k]) for k in range(3)])
    if np.array_equal(c.astype(float),np.array(json.loads(a['center']))):matches.append(j)
   assert len(matches)==1,(current,tid,a)
   ids_selected=matches[0];used.update(ids_selected.tolist());selected.append((tid,set(ids_selected.tolist())));SUPPORT[(tid,current,'observed_association')]=ids_selected.tolist()
  frozen=[d for d in DB if int(d['frame_index'])==current and d['call_type']=='local' and str(d['target_id'])==str(tid)]
  assert len(frozen)==1 and int(frozen[0]['input_points'])==len(ids) and int(frozen[0]['components'])==len(set(lab.tolist())-{-1})
 for i,(ta,ia) in enumerate(selected):
  for tb,ib in selected[i+1:]:
   if ia&ib:dups.append({'frame_index':current,'track_id_a':ta,'track_id_b':tb,'shared_refined_point_count':len(ia&ib),'same_point_set':ia==ib,'point_indices_json':json.dumps(sorted(ia&ib))})
 remaining=np.array([i for i in range(len(pts)) if i not in used],dtype=int);lab=cluster(pts[remaining]);candidates=[]
 for label in set(lab.tolist())-{-1}:
  j=remaining[lab==label];c=np.array([np.mean(pts[j][:,k]) for k in range(3)])
  cand={'center':(c[0],c[1]),'center_3d':tuple(c),'num_points':len(j),'radial_velocity':np.mean(vs[j]),'points':pts[j],'velocities':vs[j],'scan_index':current,'indices':j.tolist()}
  candidates.append(cand);candidate_log.append({'scan':current,'candidate_order':len(candidates)-1,'x':float(c[0]),'y':float(c[1]),'z':float(c[2]),'point_indices_json':json.dumps(j.tolist())})
 frozen=[d for d in DB if int(d['frame_index'])==current and d['call_type']=='remaining']
 assert bool(frozen)==bool(len(remaining)),(current,len(frozen),len(remaining))
 if frozen:assert all(int(d['components'])==len(candidates) and int(d['input_points'])==len(remaining) for d in frozen)
 with (O/'pending_bookkeeping_stdout.txt').open('a') as log,contextlib.redirect_stdout(log):exec(code,{'self':state,'candidates':candidates,'np':np,'SingleTargetTracker':capture_birth})
 expected={int(p['pending_id']):p for p in P if int(p['frame_index'])==current}
 assert set(expected)==set(state.pending_trackers),(current,'pool identity')
 for pid,p in state.pending_trackers.items():
  e=expected[pid];last=p['observations'][-1]
  assert len(p['observations'])==int(e['observation_entries']) and p['lost_count']==int(e['lost_count']),(current,pid)
  for k,v in zip('xyz',last['center_3d']):assert str(v)==e[k],(current,pid,k,v,e[k])
  assert str(last['radial_velocity'])==e['radial_velocity']
  scans=[x['scan_index'] for x in p['observations']]
  events.append({'scan':current,'pending_id':pid,'observation_entries':len(scans),'distinct_scan_count':len(set(scans)),'scan_indices_json':json.dumps(scans)})
 checks.append({'scan':current,'frozen_remaining_components':len(candidates),'all_pending_rows_exact':True})
assert [p['track_id'] for p in promotions]==[15,16,17]
for b in births[15:]:
 t=next(t for t in T if int(t['target_id'])==b['track_id'])
 assert all(str(np.float32(b[k]))==t[k] for k in 'xyz')
seed_reuse=[]
for seed in range(15):
 initial=set(SUPPORT[(seed,9,'birth_seed')])
 for tid in range(15):
  if seed==tid:continue
  chosen=set(SUPPORT.get((tid,9,'observed_association'),[]))
  if initial&chosen:seed_reuse.append({'scan':9,'birth_seed_ID':seed,'observed_association_ID':tid,'shared_refined_points':len(initial&chosen),'entire_birth_seed_reused':initial<=chosen})
write('initial_seed_reuse.csv',seed_reuse)
write('birth_forensics.csv',births);write('pending_distinct_frame_audit.csv',promotions);write('pending_scan_history.csv',events);write('duplicate_association_events.csv',dups);write('candidate_order_audit.csv',candidate_log);write('membership_checks.csv',checks)
# Primary lifecycle denominator: each scan's post-update state, including frozen removed endpoint.
summary=[];physical=[]
for tid in range(18):
 states=[dict(t,terminal=False) for t in T if int(t['target_id'])==tid]
 for e in END:
  if int(e['target_id'])==tid:states.append(dict(e,status=e['terminal_state'],terminal=True))
 states.sort(key=lambda r:int(r['frame_index']));obs=[r for r in states if r['status']=='observed'];pred=[r for r in states if r['status']=='predicted']
 xyz=np.array([[float(r[k]) for k in 'xyz'] for r in obs]).reshape(-1,3);allxyz=np.array([[float(r[k]) for k in 'xyz'] for r in states]);scans=[int(r['frame_index']) for r in obs]
 steps=np.linalg.norm(np.diff(xyz[:,:2],axis=0),axis=1);gaps=[b-a-1 for a,b in zip(scans,scans[1:]) if b-a>1]
 span=np.ptp(xyz,axis=0) if len(xyz) else [None]*3;alls=np.ptp(allxyz,axis=0)
 old=next(r for r in SUM if int(r['target_id'])==tid)
 row={'track_id':tid,'first_scan':int(old['first_frame']),'last_scan':int(old['terminal_or_last_frame']),'span_in_scans':int(old['span_in_scans']),'observed_associations':len(obs),'predicted_states':len(pred),'observed_fraction':len(obs)/len(states),'prediction_fraction':len(pred)/len(states),'observed_scan_indices_json':json.dumps(scans),'x_span_observed_m':span[0],'y_span_observed_m':span[1],'z_span_observed_m':span[2],'horizontal_bbox_span_observed_m':float(np.linalg.norm(span[:2])) if len(obs) else None,'cumulative_observed_XY_displacement_m':float(steps.sum()) if len(obs) else None,'median_observed_interframe_displacement_m':float(np.median(steps)) if len(steps) else None,'max_observed_interframe_displacement_m':float(steps.max()) if len(steps) else None,'observed_step_pair_count':len(steps),'long_observation_gap_count':len(gaps),'missing_scan_counts_json':json.dumps(gaps),'removal_reason':old['terminal_reason'],'survived_final_frame':old['terminal_reason']=='active_at_end','single_scan_initial_birth_seed':tid<15,'x_span_allstates_m':float(alls[0]),'y_span_allstates_m':float(alls[1]),'z_span_allstates_m':float(alls[2])}
 assert len(states)==row['span_in_scans'],(tid,len(states),row)
 summary.append(row)
 for r in obs:physical.append({'track_id':tid,'scan':int(r['frame_index']),'evidence_type':('pending_birth' if tid>=15 and int(r['frame_index'])==row['first_scan'] else 'observed_association'),'x':r['x'],'y':r['y'],'z':r['z']})
 if tid<15:
  b=births[tid];physical.append({'track_id':tid,'scan':9,'evidence_type':'birth_seed','x':b['x'],'y':b['y'],'z':b['z']})
write('track_lifecycle_summary.csv',summary);write('physical_state_inspection_list.csv',physical)
(O/'support_membership.json').write_text(json.dumps({f'{a}:{b}:{c}':v for (a,b,c),v in SUPPORT.items()},indent=2)+'\n')
assert all(sha(Path(p))==h for p,h in protected.items())
(O/'numeric_validation.json').write_text(json.dumps({'source_and_frozen_artifacts_unchanged':True,'original_pending_block_only_no_filter_or_full_tracker_run':True,'all_52_end_scan_pending_tables_exact':True,'all_local_and_remaining_DBSCAN_counts_match_frozen':True,'all_promoted_birth_positions_match_frozen':True,'promotions':promotions,'duplicate_reuse_events':len(dups),'lifecycle_ids':18},indent=2)+'\n')
print('PROMOTIONS',promotions);print('duplicate reuse events',len(dups));print('initial component sizes',[b['component_points'] for b in births[:15]])
for r in summary:print(r['track_id'],'obs',r['observed_associations'],'pred',r['predicted_states'],'span',r['span_in_scans'],'obs spans',r['x_span_observed_m'],r['y_span_observed_m'],r['z_span_observed_m'])
