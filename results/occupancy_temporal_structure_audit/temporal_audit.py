#!/usr/bin/env python3
"""Frozen descriptive temporal audit; no detector, scorer or path solver.

freeze/compute use only frozen cell-event tables and existing graph positions.
No labels, path selections or GNSS are parsed until feature completion.
"""
from pathlib import Path
from datetime import datetime, timezone
from collections import defaultdict
import argparse,bisect,csv,hashlib,json,platform,subprocess,time
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
SCENE=ROOT/'results/scene_spatiotemporal_background_audit'
CIDS=('100m_cross_5','100m_cross_10','200m_cross_5','200m_cross_10','300m_cross_10')
FEATURES=('age_s','longest_previous_run','previous_episodes','recent_5_prior_frame_fraction')
PROTOCOL={
 'purpose':'DESCRIPTIVE_OCCUPANCY_CONDITIONAL_AUDIT_NO_E18_CHANGE_NO_E21_MODEL',
 'sequences':list(CIDS),'history_source':'existing frozen fine_1m cell-history events only, exactly E18 historical input; no bags/cache rebuilding or new scene map',
 'frequency_reference':'E18 centroid-cell H/F and p_beta11=(H+1)/(F+2); b=1-2p unchanged',
 'quantities':['p_beta11',*FEATURES],
 'recency':'bag_record_time[f]-time[last occupied frame <f]; UNKNOWN if never previously occupied',
 'run':'longest previously consecutive occupied-frame run in the centroid cell, not path or physical object persistence',
 'episodes':'previous maximal occupied-frame episodes separated by >=1 nonoccupied recorded frame',
 'recent_window_frames':5,'recent_window':'frames f-5,...,f-1, each cell counted once per frame; fraction hits/5; no current frame',
 'window_justification':'five preceding acquisition frames at approx1Hz give short approx5s history; timestamps/cadence only before new contrasts, no other length tested; actual bag-time span retained including0.296s intervals',
 'cold_start':'F=0 age/run/episodes/recent all UNKNOWN; recent UNKNOWN for F<5; H=0 with F>0 has known run=episodes=0 but recency UNKNOWN, not infinitely old',
 'primary_conditioning':'same sequence, SAME FRAME and EXACT H; hence identical F and p; no regression, classifier, score, gate or DP',
 'auxiliary_bins':'fixed width0.05 p bins [0,.05),...,[.95,1]; F=0 explicitly COLD_PRIOR instead of observed-frequency bin',
 'duration_controls':'same-frame exact-H contrasts; separate F>=5 complete-window restriction; fixed chronological thirds; full-path summaries explicitly exposure-confounded',
 'populations':['all_observed_anchor_matched_fine_nodes','nearest_observed_anchor_node','fixed_silver_feasibility_witness','original_unconditional_winner','background_only_winner','joint_E18_winner','historical_persistent_clutter_reference'],
 'joins':'reuse existing scene anchor_node_join IDs and existing fixed/selected paths ONLY after all fine+historical graph temporal features complete; no label moving/new match radius/path optimization',
 'independent_contrast':'same centroid cell pairs recorded as deterministic ties but excluded from distinct-cell evidence; cell distinction does NOT certify independent object identity or negative class',
 'kill_pool':'original/background_only/historical_reference cells at each observed frame with exact H, excluding anchor cell and deduplicating same control cell; per-frame median comparator equal frame weight, not path-length weight',
 'kill_min_support':'at least3 distinct finite paired observed frames in EACH of five sequences, F>=5; recency/recent additionally H>=1; run/episodes additionally H>=2 to exclude degeneracy',
 'kill_direction':'one individual quantity must have nonzero median contrast with SAME sign in all five, with >=2/3 paired frames having that sign in every sequence; both+and- allowed, no combinations/selection of windows/bins',
 'kill_action':'if no individual quantity meets this fixed rule: E18 occupancy sufficient AT CURRENT EVIDENCE LEVEL; DO_NOT_PROPOSE_E21_BACKGROUND_MODEL; unsupported comparisons UNKNOWN, not proven mathematical sufficiency',
 'numerical_zero_tolerance':{'age_s':1e-9,'longest_previous_run':1e-12,'previous_episodes':1e-12,'recent_5_prior_frame_fraction':1e-12},
 'uncertainty':'dependent selected DEV_SILVER; no p-values/AUC/precision/accuracy/operational or scene-generalization claims',
 'prior_knowledge':'historical scene/E18/anchor reports known; this is not blind held-out preregistration; new feature/window/bin/kill rules fixed before this task temporal joined outcomes',
 'randomness':'none', 'forbidden':['new_score','classifier','threshold_tuning','regression','feature_combination_search','DP','path_rescoring','future_cell_evidence']
}
def utc():return datetime.now(timezone.utc).isoformat()
def sha(p):
 h=hashlib.sha256()
 with open(p,'rb') as f:
  for b in iter(lambda:f.read(1<<20),b''):h.update(b)
 return h.hexdigest()
def readcsv(p):
 with open(p,encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def writecsv(p,rows):
 if not rows:return
 with open(p,'w',encoding='utf-8',newline='') as f:
  w=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)
def jwrite(p,obj):Path(p).write_text(json.dumps(obj,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def input_paths():
 ps=[SCENE/'SCENE_FEATURES_COMPLETE.json',SCENE/'protocol.json',SCENE/'node_scene_features.csv']
 for cid in CIDS:
  ps.append(SCENE/f'cell_history__fine_1m__{cid}.npz')
  for family in ('fine_4cm_pregeometry','historical_1m_graph'):
   ps.append(ROOT/f'results/background_motion_dp_mvp/graphs/{family}__{cid}__nodes.csv')
 return ps
def freeze():
 if (OUT/'FROZEN').exists():raise RuntimeError('refuse overwrite')
 cadence=[]
 for cid in CIDS:
  with np.load(SCENE/f'cell_history__fine_1m__{cid}.npz') as z:ts=z['times']
  dt=np.diff(ts);cadence.append({'condition_id':cid,'frames':len(ts),'median_dt_s':float(np.median(dt)),'q25_dt_s':float(np.quantile(dt,.25)),'q75_dt_s':float(np.quantile(dt,.75)),'min_dt_s':float(dt.min()),'max_dt_s':float(dt.max())})
 writecsv(OUT/'acquisition_cadence.csv',cadence);jwrite(OUT/'protocol.json',PROTOCOL)
 protected=json.loads((SCENE/'protected_inputs.json').read_text())
 for p in SCENE.rglob('*'):
  if p.is_file():protected[str(p.relative_to(ROOT))]=sha(p)
 jwrite(OUT/'protected_inputs.json',protected)
 jwrite(OUT/'freeze_manifest.json',{'utc':utc(),'git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'git_status':subprocess.check_output(['git','status','--short'],cwd=ROOT,text=True),'python':platform.python_version(),'numpy':np.__version__,'command':'PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/occupancy_temporal_structure_audit/temporal_audit.py freeze','script_sha256':sha(__file__),'protocol_sha256':sha(OUT/'protocol.json'),'compute_inputs':{str(p.relative_to(ROOT)):sha(p) for p in input_paths()}})
 (OUT/'FROZEN').write_text('Five interpretable quantities;5-prior-frame window; exact-H same-frame contrasts; strict five-sequence kill; no score.\n')
def check():
 m=json.loads((OUT/'freeze_manifest.json').read_text());assert m['script_sha256']==sha(__file__);assert json.loads((OUT/'protocol.json').read_text())==PROTOCOL
 for p,h in m['compute_inputs'].items():assert sha(ROOT/p)==h,p
class CellHistory:
 def __init__(self):self.events={};self.longest={};self.run={};self.episodes={}
 def query(self,cell,fi,times):
  events=self.events.get(tuple(cell),[]);h=len(events);last=events[-1] if events else -1
  assert all(f<fi for f in events)
  return {'prior_frames':fi,'prior_hits':h,'prior_rate':h/fi if fi else np.nan,'p_beta11':(h+1)/(fi+2),'age_s':float(times[fi]-times[last]) if last>=0 else np.nan,'longest_previous_run':self.longest.get(tuple(cell),0) if fi else np.nan,'previous_episodes':self.episodes.get(tuple(cell),0) if fi else np.nan,'recent_5_prior_frame_fraction':(h-bisect.bisect_left(events,fi-5))/5 if fi>=5 else np.nan,'last_occupied_frame':last,'recent_window_elapsed_s':float(times[fi]-times[fi-5]) if fi>=5 else np.nan,'support_state':'NO_PRIOR_FRAMES' if fi==0 else 'FIRST_VISIT_NO_PREVIOUS_OCCUPANCY' if h==0 else 'HAS_PREVIOUS_OCCUPANCY','p_bin':int(min(np.floor(((h+1)/(fi+2))*20),19)) if fi else 'COLD_PRIOR'}
 def update(self,cells,fi):
  assert len(cells)==len(set(map(tuple,cells)))
  for c in cells:
   k=tuple(c);events=self.events.setdefault(k,[]);consecutive=bool(events) and events[-1]==fi-1
   self.run[k]=self.run.get(k,0)+1 if consecutive else 1
   self.longest[k]=max(self.longest.get(k,0),self.run[k]);self.episodes[k]=self.episodes.get(k,0)+int(not consecutive);events.append(fi)
def compute():
 check()
 if (OUT/'TEMPORAL_FEATURES_COMPLETE.json').exists():raise RuntimeError('refuse completed overwrite')
 started=time.perf_counter();rows=[];identity=[]
 priorfine={}
 for r in readcsv(SCENE/'node_scene_features.csv'):
  if r['source']=='fine_1m' and float(r['persistent_cutoff'])==.5:priorfine[(r['condition_id'],int(r['node_id']))]=r
 for cid in CIDS:
  with np.load(SCENE/f'cell_history__fine_1m__{cid}.npz') as z:cells=z['cells'];off=z['offsets'];times=z['times'];savedh=z['prior_hits'];savedage=z['age_s'];savedrun=z['longest_previous_run'];savede=z['previous_episodes']
  byframe=[[] for _ in times]
  for family in ('fine_4cm_pregeometry','historical_1m_graph'):
   for n in readcsv(ROOT/f'results/background_motion_dp_mvp/graphs/{family}__{cid}__nodes.csv'):
    byframe[int(n['frame_index'])].append((family,n))
  history=CellHistory();nodecount=0
  for fi in range(len(times)):
   for family,n in byframe[fi]:
    cell=np.floor([float(n['center_'+a+'_m']) for a in 'xyz']).astype(np.int64);q=history.query(cell,fi,times)
    assert q['prior_hits']==int(n['background_prior_hits']);assert q['p_beta11']==float(n['background_p_beta11'])
    if family=='fine_4cm_pregeometry':
     old=priorfine[(cid,int(n['node_id']))]
     for k in ('age_s','longest_previous_run','previous_episodes'):
      if fi==0:continue
      assert np.allclose(q[k],float(old[k]),rtol=0,atol=0,equal_nan=True),(cid,n['node_id'],k)
    rows.append({'condition_id':cid,'family':family,'node_id':int(n['node_id']),'frame_index':fi,'bag_time':float(times[fi]),'cell_x':int(cell[0]),'cell_y':int(cell[1]),'cell_z':int(cell[2]),**q});nodecount+=1
   lo,hi=off[fi:fi+2]
   for i,c in enumerate(cells[lo:hi],start=lo):
    q=history.query(c,fi,times);assert q['prior_hits']==savedh[i]
    if fi:
     assert np.allclose(q['age_s'],savedage[i],rtol=0,atol=0,equal_nan=True)
     assert q['longest_previous_run']==savedrun[i] and q['previous_episodes']==savede[i]
   history.update(cells[lo:hi],fi)
  identity.append({'condition_id':cid,'frames':len(times),'fine_plus_historical_nodes':nodecount,'original_cell_records':len(cells),'existing_h_p_and_temporal_fields_exact':True})
  print(f'{cid}: {nodecount} original nodes, prior-only temporal identity exact',flush=True)
 writecsv(OUT/'all_node_temporal_features.csv',rows);writecsv(OUT/'feature_identity.csv',identity);check()
 jwrite(OUT/'TEMPORAL_FEATURES_COMPLETE.json',{'utc':utc(),'command':'PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/occupancy_temporal_structure_audit/temporal_audit.py compute','elapsed_s':time.perf_counter()-started,'script_sha256':sha(__file__),'protocol_sha256':sha(OUT/'protocol.json'),'original_scene_receipt_sha256':sha(SCENE/'SCENE_FEATURES_COMPLETE.json'),'new_anchor_or_selected_path_used':False,'rows':len(rows),'outputs':{p:sha(OUT/p) for p in ['all_node_temporal_features.csv','feature_identity.csv']}})
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('phase',choices=['freeze','compute']);a=p.parse_args();{'freeze':freeze,'compute':compute}[a.phase]()
