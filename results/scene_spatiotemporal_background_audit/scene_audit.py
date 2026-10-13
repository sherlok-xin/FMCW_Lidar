#!/usr/bin/env python3
"""Descriptive causal scene audit. No detector, score, label or pose writes.

freeze/compute do not import alignment, labels, GNSS or path evaluators.
Existing component membership alone is replayed with the frozen E16 DBSCAN.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse, csv, hashlib, json, platform, subprocess, sys, time
import numpy as np
import scipy
from scipy.spatial import cKDTree
from sklearn.cluster import DBSCAN

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
CIDS = ('100m_cross_5','100m_cross_10','200m_cross_5','200m_cross_10','300m_cross_10')
PROTOCOL = {
 'purpose':'DESCRIPTIVE_HYPOTHESIS_GENERATION_NO_SCORE_NO_DETECTOR',
 'cohort':list(CIDS), 'cohort_reason':'existing E18 five-bag graph cohort; all frames and full cached scene, no anchor ROI',
 'coordinate':'[sensor_x,sensor_z,-sensor_y]; sensor-relative, not surveyed gravity',
 'primary':'raw_1m: complete finite nonzero cached raw returns, no intensity/height rejection',
 'reference':'fine_1m: exactly E18 occupancy of existing post-ROI/intensity/leveled-height fine cache centres',
 'one_finer_scale':'fine_004m: same existing fine cache, zero-origin 0.04m floor; sensitivity only',
 'raw_volume_m':{'x':[0,520],'y':[-250,250],'z':[-100,120]},
 'grid':'zero-origin floor; raw cell keys taken from original integer q plus integer volume origin, NOT revoxelized means',
 'history':'independent bag state; query before current update; one hit per occupied cell/frame; empty frames in denominator',
 'occupancy':'H/F (undefined F=0); E18 Beta(1,1) p=(H+1)/(F+2)',
 'persistent':'previously seen cells with H>=2 and Beta p>=0.5; descriptive, not exact static GT',
 'cutoff_sensitivity':[0.25,0.5,0.75], 'cutoff_reason':'0.5 is frozen E18 background contrast zero; other two symmetric descriptive sensitivity, never selected by outcomes',
 'first_visit':'H=0', 'low_revisit':'H<=1 (first or only one previous hit)',
 'transient_novel':'current occupied cells outside causal persistent set; includes novel; does not certify motion',
 'age':'bag-record seconds since last occupied frame; undefined for first visit',
 'history_runs':'previous longest consecutive occupied-frame run and number of previous occupied episodes; gaps/empty frames break runs',
 'distance_to_persistent_scene':'exact distance from node centroid to union of closed grid-cell AABBs; not distance to a surveyed physical surface',
 'local_density':'persistent cell centres within fixed 5m Euclidean ball; count and count/(4*pi*5^3/3)',
 'node_overlap':'fraction of fine voxel centres and their retained raw-support weights in historically persistent cells; unique coarse/fine cells separately',
 'membership':'replay frozen E16 adaptive DBSCAN min_samples=1 only to recover children; verify every saved component ID/centre/support/extent; no new candidates',
 'spatial_concentration':'persistent 1m cells aggregated in fixed 10m cubes; top10 cube share and HHI; fixed sensor coordinate spans',
 'stabilization':'lag1 persistent-set Jaccard; novel-cell fraction; per-frame causal trajectories and fixed chronological thirds, no fitted stabilization cutoff',
 'comparison':'only after complete immutable feature receipt; nearest existing node <= historical 3m anchor gate, all matched nodes also retained; same-frame contrasts; no certified negatives/AUC/precision',
 'effect_sizes':'median/IQR and same-frame percentile; Cliff delta as dependent-sample descriptive proxy only; Spearman occupancy-distance, fixed occupancy bins',
 'missing':'empty persistent set => distance NaN, density 0, explicit persistent_cells=0; no silent zero-distance substitution',
 'randomness':'none in data calculation; synthetic validator deterministic seed only',
 'no_changes':['E18','score','graphs','labels','GNSS','calibration','offsets','thresholds','raw bags','historical results']
}

def sha(p):
 h=hashlib.sha256()
 with open(p,'rb') as f:
  for b in iter(lambda:f.read(1<<20),b''): h.update(b)
 return h.hexdigest()
def jwrite(p,obj):
 Path(p).write_text(json.dumps(obj,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def readcsv(p):
 with open(p,encoding='utf-8-sig',newline='') as f: return list(csv.DictReader(f))
def writecsv(p,rows):
 if not rows: return
 with open(p,'w',newline='',encoding='utf-8') as f:
  w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def utc(): return datetime.now(timezone.utc).isoformat()
def inputs():
 paths=[ROOT/'scripts/background_motion_core.py',ROOT/'scripts/background_motion_dp_mvp.py',ROOT/'scripts/published_baseline_dp_mvp.py',ROOT/'scripts/dbscan_fragmentation_attribution.py',ROOT/'results/published_baseline_dp_mvp/protocol.json']
 for cid in CIDS:
  for folder in ('calibration_closure/raw_voxel_cache','published_baseline_dp_mvp/raw_fine_cache'):
   paths += [ROOT/'results'/folder/(cid+ext) for ext in ('.npz','.json')]
  paths += [ROOT/'results/background_motion_dp_mvp/graphs'/f'fine_4cm_pregeometry__{cid}__nodes.csv']
 return paths
def freeze():
 if (OUT/'FROZEN').exists(): raise RuntimeError('refusing overwrite of frozen audit')
 jwrite(OUT/'protocol.json',PROTOCOL)
 # Bytes only: protect existing research without using its outcomes.
 protected=dict(json.loads((ROOT/'results/gnss_conditioned_uav_phenotype/run_02/protocol.json').read_text())['source_sha256'])
 for folder in ('background_motion_dp_mvp','e18_mechanism_generalization_audit','rviz_gnss_lidar_sanity'):
  for p in (ROOT/'results'/folder).rglob('*'):
   if p.is_file(): protected[str(p.relative_to(ROOT))]=sha(p)
 jwrite(OUT/'protected_inputs.json',protected)
 jwrite(OUT/'freeze_manifest.json',{'utc':utc(),'git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'git_status':subprocess.check_output(['git','status','--short'],cwd=ROOT,text=True),'python':platform.python_version(),'numpy':np.__version__,'scipy':scipy.__version__,'command':'PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/scene_spatiotemporal_background_audit/scene_audit.py freeze','source_sha256':sha(__file__),'protocol_sha256':sha(OUT/'protocol.json'),'compute_inputs':{str(p.relative_to(ROOT)):sha(p) for p in inputs()}})
 (OUT/'FROZEN').write_text('Causal scene rules frozen before compute and post-compute anchor join.\n')
def check():
 assert json.loads((OUT/'protocol.json').read_text())==PROTOCOL
 m=json.loads((OUT/'freeze_manifest.json').read_text())
 assert m['source_sha256']==sha(__file__)
 for p,h in m['compute_inputs'].items(): assert sha(ROOT/p)==h,p

class History:
 def __init__(self,size):
  self.size=size;self.lookup={};self.cells=[];self.hits=[];self.last=[];self.run=[];self.longest=[];self.episodes=[]
 def register(self,cells):
  ids=[]
  for c in cells:
   k=tuple(map(int,c))
   if k not in self.lookup:
    self.lookup[k]=len(self.cells);self.cells.append(k);self.hits.append(0);self.last.append(-1);self.run.append(0);self.longest.append(0);self.episodes.append(0)
   ids.append(self.lookup[k])
  return np.asarray(ids,dtype=np.int32)
 def query(self,cells,fi,times):
  ix=np.asarray([self.lookup.get(tuple(map(int,c)),-1) for c in cells]); good=ix>=0
  h=np.zeros(len(ix),np.int32);last=np.full(len(ix),-1,np.int32);run=h.copy();episodes=h.copy()
  if good.any():
   h[good]=np.asarray(self.hits)[ix[good]];last[good]=np.asarray(self.last)[ix[good]];run[good]=np.asarray(self.longest)[ix[good]];episodes[good]=np.asarray(self.episodes)[ix[good]]
  age=np.full(len(ix),np.nan);seen=last>=0;age[seen]=times[fi]-times[last[seen]]
  return h,last,age,run,episodes
 def persistent(self,fi,cutoff=.5):
  h=np.asarray(self.hits);mask=(h>=2)&((h+1)/(fi+2)>=cutoff)
  return np.asarray(self.cells,dtype=np.int32).reshape(-1,3)[mask]
 def update(self,ids,fi):
  for i in ids:
   consecutive=self.last[i]==fi-1 and self.hits[i]>0
   self.run[i]=self.run[i]+1 if consecutive else 1
   self.episodes[i]+=int(not consecutive);self.longest[i]=max(self.longest[i],self.run[i]);self.hits[i]+=1;self.last[i]=fi

def scene_geometry(points,cells,size):
 points=np.asarray(points,dtype=float).reshape(-1,3)
 if not len(cells): return np.full(len(points),np.nan),np.full(len(points),np.nan),np.zeros(len(points),int)
 centres=(cells+.5)*size;tree=cKDTree(centres)
 dcentre=tree.query(points)[0];distance=np.empty(len(points))
 for i,p in enumerate(points):
  ids=tree.query_ball_point(p,float(dcentre[i])+np.sqrt(3)*size/2+1e-10)
  # Any closer AABB must have its centre within dcentre + half diagonal.
  lo=cells[ids]*size;delta=np.maximum(np.maximum(lo-p,p-(lo+size)),0)
  distance[i]=np.sqrt(np.min(np.einsum('ij,ij->i',delta,delta)))
 density=tree.query_ball_point(points,5.,return_length=True)
 return distance,dcentre,density

def node_features(nodes,children,values,hist,fi,times,persistent,cutoff):
 points=np.array([[float(n['center_'+a+'_m']) for a in 'xyz'] for n in nodes]).reshape(-1,3)
 q=np.floor(points/hist.size).astype(np.int32)
 h,last,age,longest,episodes=hist.query(q,fi,times)
 d,dc,density=scene_geometry(points,persistent,hist.size)
 pset=set(map(tuple,persistent.tolist()));rows=[]
 for i,n in enumerate(nodes):
  child=values[children[int(n['source_component_id'])]];cc=np.floor(child[:,:3].astype(float)/hist.size).astype(np.int32)
  overlap=np.array([tuple(c) in pset for c in cc]);unique=np.unique(cc,axis=0)
  rows.append({'node_id':int(n['node_id']),'frame_index':fi,'prior_hits':int(h[i]),'prior_frames':fi,'prior_rate':float(h[i]/fi) if fi else np.nan,'p_beta11':float((h[i]+1)/(fi+2)),'age_s':float(age[i]),'longest_previous_run':int(longest[i]),'previous_episodes':int(episodes[i]),'first_visit':int(h[i]==0),'persistent_cells':len(persistent),'distance_to_persistent_scene_m':float(d[i]),'nearest_persistent_centre_m':float(dc[i]),'persistent_centres_within_5m':int(density[i]),'persistent_density_per_m3':float(density[i]/(4*np.pi*125/3)),'fine_point_overlap_fraction':float(overlap.mean()),'retained_raw_weight_overlap_fraction':float(child[overlap,3].sum()/child[:,3].sum()),'unique_cell_overlap_fraction':float(np.mean([tuple(c) in pset for c in unique]))})
 return rows

def compute():
 check()
 if (OUT/'SCENE_FEATURES_COMPLETE.json').exists(): raise RuntimeError('refusing completed feature overwrite')
 started=time.perf_counter();summary=[];features=[];lineage=[]
 cfg=json.loads((ROOT/'results/published_baseline_dp_mvp/protocol.json').read_text())['published_structure_baseline']['project_adapted']
 for cid in CIDS:
  rawmeta=json.loads((ROOT/f'results/calibration_closure/raw_voxel_cache/{cid}.json').read_text())
  with np.load(ROOT/f'results/calibration_closure/raw_voxel_cache/{cid}.npz') as z: raw,ro=z['values'],z['offsets']
  with np.load(ROOT/f'results/published_baseline_dp_mvp/raw_fine_cache/{cid}.npz') as z: fine,fo,times=z['values'],z['offsets'],z['times']
  assert np.array_equal(times,[r['bag_time'] for r in rawmeta['frames']]);assert len(ro)==len(fo)
  assert all(r['valid_returns']==r['nonzero_returns'] for r in rawmeta['frames'])
  nodes=readcsv(ROOT/f'results/background_motion_dp_mvp/graphs/fine_4cm_pregeometry__{cid}__nodes.csv')
  byframe=[[] for _ in times]
  for n in nodes: byframe[int(n['frame_index'])].append(n)
  histories={s:History(size) for s,size in [('raw_1m',1.),('fine_1m',1.),('fine_004m',.04)]}
  records={s:[] for s in histories};snapshots={s:[] for s in histories};previous={s:set() for s in histories}
  for fi in range(len(times)):
   fv=fine[fo[fi]:fo[fi+1]];rv=raw[ro[fi]:ro[fi+1]]
   eps=.6+cfg['dbscan_alpha']*max(float(np.mean(np.linalg.norm(fv[:,:3],axis=1)))-10,0) if len(fv) else .6
   labels=DBSCAN(eps=eps,min_samples=1,algorithm='kd_tree').fit_predict(fv[:,:3]) if len(fv) else np.array([],int)
   children={int(c):np.flatnonzero(labels==c) for c in np.unique(labels)}
   assert set(children)=={int(n['source_component_id']) for n in byframe[fi]}
   for n in byframe[fi]:
    c=fv[children[int(n['source_component_id'])]];centre=np.median(c[:,:3],axis=0) if len(c)>=3 else np.mean(c[:,:3],axis=0)
    assert np.array_equal(centre,[float(n['center_'+a+'_m']) for a in 'xyz'])
    assert len(c)==int(n['voxels']) and float(c[:,3].sum())==float(n['raw_support'])
    assert float(np.max(np.ptp(c[:,:3],axis=0)))==float(n['extent_m'])
   for source,hist in histories.items():
    if source=='raw_1m':
     cells=rv[:,8:11].astype(np.int32)+[0,-250,-100];weights=rv[:,4].astype(np.int64);counts=np.ones(len(cells),np.int32)
     assert np.array_equal(cells,np.floor(rv[:,1:4]).astype(np.int32))
    else:
     cells,inv,counts=np.unique(np.floor(fv[:,:3].astype(float)/hist.size).astype(np.int32),axis=0,return_inverse=True,return_counts=True)
     weights=np.bincount(inv,weights=fv[:,3],minlength=len(cells)).astype(np.int64)
    ids=hist.register(cells);h,last,age,longest,episodes=hist.query(cells,fi,times)
    persistent=hist.persistent(fi);pset=set(map(tuple,persistent.tolist()));seen=h>0;low=h<=1;overlap=np.array([tuple(c) in pset for c in cells])
    # Store all current occupied-cell causal records, compactly, before update.
    records[source].append({'cells':cells,'prior_hits':h,'prior_rate':h.astype(float)/fi if fi else np.full(len(h),np.nan),'p_beta11':(h+1.)/(fi+2),'first_visit':h==0,'revisit':h>0,'last_frame':last,'age_s':age,'longest_previous_run':longest,'previous_episodes':episodes,'raw_count':weights,'fine_count':counts,'persistent':overlap})
    snapshots[source].append(persistent)
    unique_union=sum(v>0 for v in hist.hits);union_with_current=len(hist.cells)
    groups=np.unique(persistent//10,axis=0,return_counts=True)[1] if len(persistent) else np.array([])
    jaccard=len(pset&previous[source])/len(pset|previous[source]) if pset|previous[source] else np.nan
    row={'condition_id':cid,'source':source,'frame_index':fi,'bag_time':float(times[fi]),'prior_frames':fi,'raw_returns':int(weights.sum()),'input_elements':int(counts.sum()),'current_cells':len(cells),'previously_occupied_return_fraction':float(weights[seen].sum()/weights.sum()) if weights.sum() else np.nan,'previously_occupied_cell_fraction':float(seen.mean()) if len(seen) else np.nan,'low_revisit_return_fraction':float(weights[low].sum()/weights.sum()) if weights.sum() else np.nan,'first_visit_return_fraction':float(weights[~seen].sum()/weights.sum()) if weights.sum() else np.nan,'novel_cells':int((~seen).sum()),'low_revisit_cells':int(low.sum()),'transient_nonpersistent_cells':int((~overlap).sum()),'persistent_cells':len(persistent),'persistent_return_fraction':float(weights[overlap].sum()/weights.sum()) if weights.sum() else np.nan,'persistent_current_cell_fraction':float(overlap.mean()) if len(overlap) else np.nan,'prior_union_cells':unique_union,'union_after_current_cells':union_with_current,'persistent_lag1_jaccard':float(jaccard),'persistent_top10_10m_cube_fraction':float(np.sort(groups)[-10:].sum()/groups.sum()) if len(groups) else np.nan,'persistent_10m_cube_hhi':float(np.sum((groups/groups.sum())**2)) if len(groups) else np.nan}
    for a,k in enumerate('xyz'):
     row[f'persistent_{k}_q05_m']=float(np.quantile((persistent[:,a]+.5)*hist.size,.05)) if len(persistent) else np.nan
     row[f'persistent_{k}_q95_m']=float(np.quantile((persistent[:,a]+.5)*hist.size,.95)) if len(persistent) else np.nan
    summary.append(row);previous[source]=pset
    for cutoff in ((.25,.5,.75) if source!='fine_004m' else (.5,)):
     pp=persistent if cutoff==.5 else hist.persistent(fi,cutoff)
     nr=node_features(byframe[fi],children,fv,hist,fi,times,pp,cutoff)
     for r,n in zip(nr,byframe[fi]):
      if source=='fine_1m' and cutoff==.5:
       assert r['prior_hits']==int(n['background_prior_hits']) and r['prior_frames']==int(n['background_prior_frames'])
       assert r['p_beta11']==float(n['background_p_beta11'])
      features.append({'condition_id':cid,'source':source,'persistent_cutoff':cutoff,'bag_time':float(times[fi]),**r})
    hist.update(ids,fi)
   if fi%20==0: print(f'{cid}: {fi+1}/{len(times)} causal frames, elapsed {time.perf_counter()-started:.1f}s',flush=True)
  for source,rs in records.items():
   offsets=np.r_[0,np.cumsum([len(r['cells']) for r in rs])]
   payload={k:np.concatenate([r[k] for r in rs]) for k in rs[0]};payload.update(offsets=offsets,times=times,cell_m=histories[source].size)
   np.savez_compressed(OUT/f'cell_history__{source}__{cid}.npz',**payload)
   sp=snapshots[source];np.savez_compressed(OUT/f'persistent_snapshots__{source}__{cid}.npz',cells=np.concatenate(sp),offsets=np.r_[0,np.cumsum([len(c) for c in sp])],times=times,cell_m=histories[source].size)
  lineage.append({'condition_id':cid,'frames':len(times),'nodes':len(nodes),'raw_nonzero_returns':int(raw[:,4].sum()),'retained_fine_raw_support':int(fine[:,3].sum()),'fine_elements':len(fine),'component_identity_verified':True,'e18_1m_annotations_exact':True,'raw_volume_contains_all_nonzero':True})
  print(f'COMPLETE {cid}: {len(nodes)} nodes, {len(times)} frames',flush=True)
 writecsv(OUT/'frame_scene_statistics.csv',summary);writecsv(OUT/'node_scene_features.csv',features);writecsv(OUT/'input_lineage.csv',lineage)
 check();outputs={p.name:sha(p) for p in OUT.glob('*.npz')}
 outputs.update({p:sha(OUT/p) for p in ['frame_scene_statistics.csv','node_scene_features.csv','input_lineage.csv']})
 jwrite(OUT/'SCENE_FEATURES_COMPLETE.json',{'utc':utc(),'command':'PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/scene_spatiotemporal_background_audit/scene_audit.py compute','elapsed_s':time.perf_counter()-started,'label_or_gnss_used':False,'source_sha256':sha(__file__),'protocol_sha256':sha(OUT/'protocol.json'),'outputs':outputs,'frames':len(summary)//3,'nodes':sum(r['nodes'] for r in lineage)})

if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('phase',choices=['freeze','compute']);args=a.parse_args()
 {'freeze':freeze,'compute':compute}[args.phase]()
