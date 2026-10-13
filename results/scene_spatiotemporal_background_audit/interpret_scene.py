#!/usr/bin/env python3
"""Post-compute descriptive joins. No scoring/search or map construction.

Uses only frozen causal cell tables/snapshots, all precomputed node features,
and existing observed anchors/path identities after verifying completion.
"""
import csv, json, math, sys, time
from pathlib import Path
from collections import defaultdict
from datetime import datetime, timezone
import numpy as np
from scipy.stats import spearmanr
import scene_audit as s

OUT=s.OUT;ROOT=s.ROOT;CIDS=s.CIDS
FEATURES=('p_beta11','distance_to_persistent_scene_m','persistent_centres_within_5m','retained_raw_weight_overlap_fraction','unique_cell_overlap_fraction','age_s','longest_previous_run')
def describe(values):
 v=np.asarray(values,dtype=float);v=v[np.isfinite(v)]
 if not len(v): return dict(n=0,q25=np.nan,median=np.nan,q75=np.nan,iqr=np.nan,min=np.nan,max=np.nan)
 q=np.quantile(v,[.25,.5,.75]);return dict(n=len(v),q25=q[0],median=q[1],q75=q[2],iqr=q[2]-q[0],min=v.min(),max=v.max())
def delta(x,y):
 x=np.asarray(x,float);y=np.sort(np.asarray(y,float));x=x[np.isfinite(x)];y=y[np.isfinite(y)]
 if not len(x) or not len(y):return np.nan
 return float(np.sum(np.searchsorted(y,x,side='left')+np.searchsorted(y,x,side='right')-len(y))/(len(x)*len(y)))
def correlate(x,y):
 x=np.asarray(x,float);y=np.asarray(y,float);ok=np.isfinite(x)&np.isfinite(y)
 return float(spearmanr(x[ok],y[ok]).statistic) if ok.sum()>2 and np.ptp(x[ok])>0 and np.ptp(y[ok])>0 else np.nan
def run():
 start=time.perf_counter();receipt=json.loads((OUT/'SCENE_FEATURES_COMPLETE.json').read_text())
 for p,h in receipt['outputs'].items():assert s.sha(OUT/p)==h,p
 join_started=s.utc(); assert join_started>receipt['utc']
 labels=s.readcsv(ROOT/'results/research_dev_v0/silver_labels.csv');assert len(labels)==80
 paths=s.readcsv(ROOT/'results/e18_mechanism_generalization_audit/selected_paths.csv')
 fixed=s.readcsv(ROOT/'results/background_motion_dp_mvp/fixed_reference_paths.csv')
 allfeat=s.readcsv(OUT/'node_scene_features.csv')
 lookup={};pool=defaultdict(list)
 for r in allfeat:
  for k in r:
   if k!='condition_id' and k!='source':r[k]=float(r[k])
  key=(r['condition_id'],r['source'],r['persistent_cutoff'],int(r['node_id']));lookup[key]=r;pool[key[:3]].append(r)
 graphs={cid:s.readcsv(ROOT/f'results/background_motion_dp_mvp/graphs/fine_4cm_pregeometry__{cid}__nodes.csv') for cid in CIDS}
 meta={(cid,int(n['node_id'])):n for cid,ns in graphs.items() for n in ns}
 byframe=defaultdict(list)
 for (cid,nid),n in meta.items():byframe[(cid,int(n['frame_index']))].append(n)
 groups=defaultdict(set);anchorjoin=[];percentiles=[]
 for a in labels:
  cid=a['condition_id'];fi=int(a['frame_index']);pos=np.array([float(a['center_'+k+'_m']) for k in 'xyz']);ns=byframe[(cid,fi)]
  distances=np.array([np.linalg.norm(np.array([float(n['center_'+k+'_m']) for k in 'xyz'])-pos) for n in ns]);ix=int(np.argmin(distances));nearest=int(ns[ix]['node_id'])
  ids=[int(ns[i]['node_id']) for i in np.flatnonzero(distances<=3)]
  assert ids and distances[ix]<=3
  groups[(cid,'anchor_matched_all_nodes')].update(ids);groups[(cid,'nearest_anchor_node')].add(nearest)
  row={'condition_id':cid,'frame_index':fi,'bag_time':a['bag_time_unix'],'candidate_id':a['candidate_id'],'anchor_status':'OBSERVED_DEV_SILVER_NOT_POINT_GT','nearest_node_id':nearest,'nearest_centre_distance_m':float(distances[ix]),'nodes_within_existing_3m_gate':len(ids),'matched_node_ids':';'.join(map(str,ids))}
  for source in ('raw_1m','fine_1m','fine_004m'):
   f=lookup[(cid,source,.5,nearest)]
   for feature in FEATURES:row[source+'__'+feature]=f[feature]
   contemporaries=[lookup[(cid,source,.5,int(n['node_id']))] for n in ns]
   for feature in ('distance_to_persistent_scene_m','p_beta11'):
    v=np.array([r[feature] for r in contemporaries]);v=v[np.isfinite(v)];x=f[feature]
    percentile=float((np.sum(v<x)+.5*np.sum(v==x))/len(v)) if len(v) and np.isfinite(x) else np.nan
    percentiles.append({'condition_id':cid,'frame_index':fi,'source':source,'feature':feature,'node_id':nearest,'value':x,'same_frame_all_node_median':float(np.median(v)) if len(v) else np.nan,'difference_from_same_frame_median':float(x-np.median(v)) if len(v) else np.nan,'same_frame_percentile_tie_half':percentile,'comparison_nodes':len(v)})
  anchorjoin.append(row)
 for cid in CIDS:groups[(cid,'all_fine_nodes')].update(int(n['node_id']) for n in graphs[cid])
 for p in paths:
  if p['family']=='fine_4cm_pregeometry':groups[(p['condition_id'],'winner_'+p['arm'])].add(int(p['node_id']))
 for p in fixed:
  if p['family']=='fine_4cm_pregeometry' and p['path_type']=='silver_feasible':groups[(p['condition_id'],'fixed_silver_feasibility_witness')].add(int(p['node_id']))
 joined=[]
 for (cid,group),ids in groups.items():
  for nid in sorted(ids):
   n=meta[(cid,nid)]
   for source,cut in [('raw_1m',.25),('raw_1m',.5),('raw_1m',.75),('fine_1m',.25),('fine_1m',.5),('fine_1m',.75),('fine_004m',.5)]:
    r=lookup[(cid,source,cut,nid)]
    joined.append({'condition_id':cid,'group':group,'source':source,'cutoff':cut,'node_id':nid,'frame_index':int(n['frame_index']),'e18_p_beta11':float(n['background_p_beta11']),**{k:r[k] for k in FEATURES}})
 # Point queries below reuse previously computed causal tables and snapshots.
 # They do not read clouds or refit/reconstruct a background using anchors.
 extra=defaultdict(list)
 for a in labels:
  extra[(a['condition_id'],int(a['frame_index']))].append({'group':'observed_anchor_position','node_id':a['candidate_id'],'xyz':np.array([float(a['center_'+k+'_m']) for k in 'xyz'])})
 for p in fixed:
  if p['family']=='historical_1m_graph' and p['path_type']=='top_clutter_full':
   extra[(p['condition_id'],int(p['frame_index']))].append({'group':'historical_persistent_clutter_reference','node_id':p['node_id'],'xyz':np.array([float(p['center_'+k+'_m']) for k in 'xyz'])})
 corrected=[]
 for cid in CIDS:
  for source in ('raw_1m','fine_1m','fine_004m'):
   with np.load(OUT/f'cell_history__{source}__{cid}.npz') as z:cells=z['cells'];offsets=z['offsets'];times=z['times'];size=float(z['cell_m'])
   with np.load(OUT/f'persistent_snapshots__{source}__{cid}.npz') as z:pc=z['cells'];po=z['offsets']
   state={}
   for fi,t in enumerate(times):
    pers=pc[po[fi]:po[fi+1]]
    cubes=np.floor((pers+.5)*size/10).astype(int);cnt=np.unique(cubes,axis=0,return_counts=True)[1]
    corrected.append({'condition_id':cid,'source':source,'frame_index':fi,'cube_side_m':10.,'persistent_top10_10m_cube_fraction':float(np.sort(cnt)[-10:].sum()/cnt.sum()) if len(cnt) else np.nan,'persistent_10m_cube_hhi':float(np.sum((cnt/cnt.sum())**2)) if len(cnt) else np.nan,'correction_note':'fine004m original concentration columns aggregate 10 cell indices = 0.4m; this table uses physical 10m for every source'})
    points=extra[(cid,fi)]
    if points:
     xyz=np.array([p['xyz'] for p in points]);d,dc,density=s.scene_geometry(xyz,pers,size)
     for i,p in enumerate(points):
      q=tuple(np.floor(p['xyz']/size).astype(int));h,last,longest,episodes=state.get(q,(0,-1,0,0));age=float(t-times[last]) if last>=0 else np.nan
      joined.append({'condition_id':cid,'group':p['group'],'source':source,'cutoff':.5,'node_id':p['node_id'],'frame_index':fi,'e18_p_beta11':np.nan,'p_beta11':(h+1)/(fi+2),'distance_to_persistent_scene_m':d[i],'persistent_centres_within_5m':int(density[i]),'retained_raw_weight_overlap_fraction':np.nan,'unique_cell_overlap_fraction':np.nan,'age_s':age,'longest_previous_run':longest})
    for c in cells[offsets[fi]:offsets[fi+1]]:
     q=tuple(c);h,last,longest,episodes=state.get(q,(0,-1,0,0));run=state.get(('run',q),0)+1 if last==fi-1 and h else 1
     state[q]=(h+1,fi,max(longest,run),episodes+int(last!=fi-1 or h==0));state[('run',q)]=run
 s.writecsv(OUT/'anchor_node_join.csv',anchorjoin);s.writecsv(OUT/'anchor_same_frame_contrasts.csv',percentiles)
 s.writecsv(OUT/'population_scene_features.csv',joined);s.writecsv(OUT/'corrected_spatial_concentration.csv',corrected)
 summaries=[];bucket=defaultdict(list)
 for r in joined:bucket[(r['condition_id'],r['group'],r['source'],r['cutoff'])].append(r)
 for key,rs in bucket.items():
  for f in FEATURES:summaries.append({'condition_id':key[0],'group':key[1],'source':key[2],'cutoff':key[3],'feature':f,**describe([r[f] for r in rs])})
 s.writecsv(OUT/'population_distribution_summary.csv',summaries)
 contrast=[]
 for cid in CIDS:
  for source in ('raw_1m','fine_1m','fine_004m'):
   ar=bucket[(cid,'nearest_anchor_node',source,.5)]
   for group in ('all_fine_nodes','winner_original','winner_background_only','winner_motion_only','winner_background_motion_E18','historical_persistent_clutter_reference'):
    rs=bucket[(cid,group,source,.5)]
    for f in ('distance_to_persistent_scene_m','p_beta11','persistent_centres_within_5m'):
     x=[r[f] for r in ar];y=[r[f] for r in rs];dx=describe(x);dy=describe(y)
     contrast.append({'condition_id':cid,'source':source,'feature':f,'reference_group':group,'anchor_nodes_n':dx['n'],'reference_n':dy['n'],'median_difference_anchor_minus_reference':dx['median']-dy['median'],'cliff_delta_descriptive_not_classifier':delta(x,y)})
 s.writecsv(OUT/'descriptive_effect_sizes.csv',contrast)
 correlations=[];bins=[]
 for cid in CIDS:
  for source in ('raw_1m','fine_1m','fine_004m'):
   rs=pool[(cid,source,.5)];ids=[int(r['node_id']) for r in rs];p=[r['p_beta11'] for r in rs];ep=[float(meta[(cid,i)]['background_p_beta11']) for i in ids];d=[r['distance_to_persistent_scene_m'] for r in rs]
   correlations.append({'condition_id':cid,'source':source,'population':'all_fine_nodes','n':len(rs),'spearman_distance_vs_same_source_p':correlate(d,p),'spearman_distance_vs_e18_fine_p':correlate(d,ep),'spearman_raw_vs_fine_distance':correlate([lookup[(cid,'raw_1m',.5,i)]['distance_to_persistent_scene_m'] for i in ids],[lookup[(cid,'fine_1m',.5,i)]['distance_to_persistent_scene_m'] for i in ids])})
   # Fixed bins are summaries, not score gates or tuned classification rules.
   for name,fn in [('H0',lambda r:r['prior_hits']==0),('H1',lambda r:r['prior_hits']==1),('Hge2_p_lt025',lambda r:r['prior_hits']>=2 and r['p_beta11']<.25),('Hge2_p025to05',lambda r:r['prior_hits']>=2 and .25<=r['p_beta11']<.5),('Hge2_pge05',lambda r:r['prior_hits']>=2 and r['p_beta11']>=.5)]:
    rr=[r for r in rs if fn(r)]
    bins.append({'condition_id':cid,'source':source,'occupancy_bin':name,**describe([r['distance_to_persistent_scene_m'] for r in rr])})
 s.writecsv(OUT/'occupancy_distance_associations.csv',correlations);s.writecsv(OUT/'distance_within_occupancy_bins.csv',bins)
 # Stronger nonredundancy check: H0 nodes in the SAME frame have identical p.
 h0rows=[]
 for cid in CIDS:
  for source in ('raw_1m','fine_1m','fine_004m'):
   for fi in range(len({int(n['frame_index']) for n in graphs[cid]})):
    rr=[r for r in pool[(cid,source,.5)] if int(r['frame_index'])==fi and r['prior_hits']==0]
    ds=describe([r['distance_to_persistent_scene_m'] for r in rr])
    h0rows.append({'condition_id':cid,'source':source,'frame_index':fi,'identical_H':0,'identical_p_beta11':1/(fi+2),**ds})
 s.writecsv(OUT/'same_frame_H0_distance_spread.csv',h0rows)
 # Return-weighted sequence aggregates plus chronological thirds; keep cold start.
 frame=s.readcsv(OUT/'frame_scene_statistics.csv');scene=[]
 for cid in CIDS:
  for source in ('raw_1m','fine_1m','fine_004m'):
   rs=[r for r in frame if r['condition_id']==cid and r['source']==source];nf=len(rs)
   for period in ('all','prior_history_ge2','first_third','middle_third','last_third'):
    selected=[r for r in rs if period=='all' or (period=='prior_history_ge2' and int(r['frame_index'])>=2) or (period=='first_third' and int(r['frame_index'])<nf/3) or (period=='middle_third' and nf/3<=int(r['frame_index'])<2*nf/3) or (period=='last_third' and int(r['frame_index'])>=2*nf/3)]
    w=np.array([float(r['raw_returns']) for r in selected]);row={'condition_id':cid,'source':source,'period':period,'frames':len(selected),'raw_support':int(w.sum())}
    for f in ('previously_occupied_return_fraction','low_revisit_return_fraction','first_visit_return_fraction','persistent_return_fraction'):
     row['weighted_'+f]=float(np.sum(w*np.array([float(r[f]) for r in selected]))/w.sum())
    for f in ('novel_cells','transient_nonpersistent_cells','persistent_cells','persistent_lag1_jaccard'):
     ds=describe([float(r[f]) for r in selected]);row[f+'_median']=ds['median'];row[f+'_iqr']=ds['iqr']
    scene.append(row)
 s.writecsv(OUT/'sequence_scene_summary.csv',scene)
 # Existing path geometry is described, never rescored; velocity uses bag dt.
 pathsummary=[]
 for cid in CIDS:
  for group in ('winner_original','winner_background_only','winner_motion_only','winner_background_motion_E18','historical_persistent_clutter_reference'):
   rs=bucket[(cid,group,'raw_1m',.5)];fine=group!='historical_persistent_clutter_reference'
   if fine:
    ns=sorted([meta[(cid,int(r['node_id']))] for r in rs],key=lambda n:int(n['frame_index']))
   else:ns=sorted([p for p in fixed if p['family']=='historical_1m_graph' and p['condition_id']==cid and p['path_type']=='top_clutter_full'],key=lambda n:int(n['frame_index']))
   xyz=np.array([[float(n['center_'+k+'_m']) for k in 'xyz'] for n in ns]);t=np.array([float(n['bag_time']) for n in ns]);speeds=np.linalg.norm(np.diff(xyz,axis=0),axis=1)/np.diff(t)
   row={'condition_id':cid,'group':group,'nodes':len(ns),'path_duration_s':float(t[-1]-t[0]),'median_speed_mps':float(np.median(speeds)),'max_speed_mps':float(np.max(speeds)),'max_pair_step_m':float(np.max(np.linalg.norm(np.diff(xyz,axis=0),axis=1))),'coordinate_span_x_m':float(np.ptp(xyz[:,0])),'coordinate_span_y_m':float(np.ptp(xyz[:,1])),'coordinate_span_z_m':float(np.ptp(xyz[:,2]))}
   for source in ('raw_1m','fine_1m'):
    rr=bucket[(cid,group,source,.5)];d=np.array([r['distance_to_persistent_scene_m'] for r in rr]);p=np.array([r['p_beta11'] for r in rr]);ok=np.isfinite(d)
    row[source+'__distance_median_m']=float(np.median(d[ok]));row[source+'__p_median']=float(np.median(p));row[source+'__distance_unknown_nodes']=int((~ok).sum())
    # Neutral continuous descriptions + preregistered geometric boundaries only.
    row[source+'__distance_zero_fraction']=float(np.mean(d[ok]<=1e-10));row[source+'__distance_above_fixed_5m_fraction']=float(np.mean(d[ok]>5))
   pathsummary.append(row)
 s.writecsv(OUT/'existing_path_characterization.csv',pathsummary)
 join_inputs=[ROOT/'results/research_dev_v0/silver_labels.csv',ROOT/'results/e18_mechanism_generalization_audit/selected_paths.csv',ROOT/'results/background_motion_dp_mvp/fixed_reference_paths.csv']
 s.jwrite(OUT/'interpretation_manifest.json',{'utc_started':join_started,'utc_completed':s.utc(),'scene_receipt_utc':receipt['utc'],'command':'PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/scene_spatiotemporal_background_audit/interpret_scene.py','script_sha256':s.sha(__file__),'scene_completion_sha256':s.sha(OUT/'SCENE_FEATURES_COMPLETE.json'),'join_inputs_sha256':{str(p.relative_to(ROOT)):s.sha(p) for p in join_inputs},'labels':80,'no_rescore':True,'elapsed_s':time.perf_counter()-start,'limitations':['full-period populations differ in support/frame/range; paired same-frame contrasts separate','all candidates include anchor-matched nodes and are not certified negatives','fine4cm original concentration columns physically0.4m; corrected physical10m table supplied without overwriting frozen results']})
 print('descriptive joins and summaries complete',flush=True)
if __name__=='__main__':run()
