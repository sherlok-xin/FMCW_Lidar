#!/usr/bin/env python3
"""Fixed-cutoff scene sensitivity and same-frame existing-path contrasts."""
from collections import defaultdict
import numpy as np
import scene_audit as s
from interpret_scene import describe,delta
O=s.OUT;R=s.ROOT

def run():
 feat=s.readcsv(O/'node_scene_features.csv');lookup={(r['condition_id'],r['source'],float(r['persistent_cutoff']),int(r['node_id'])):r for r in feat}
 anchors=s.readcsv(O/'anchor_node_join.csv');paths=s.readcsv(R/'results/e18_mechanism_generalization_audit/selected_paths.csv')
 pathlookup={(p['condition_id'],p['arm'],int(p['frame_index'])):p for p in paths if p['family']=='fine_4cm_pregeometry'}
 labels=s.readcsv(R/'results/research_dev_v0/silver_labels.csv');lab={(a['condition_id'],int(a['frame_index'])):np.array([float(a['center_'+k+'_m']) for k in 'xyz']) for a in labels}
 paired=[]
 for a in anchors:
  cid=a['condition_id'];fi=int(a['frame_index']);nid=int(a['nearest_node_id'])
  for arm in ('original','background_only','motion_only','background_motion_E18'):
   path=pathlookup.get((cid,arm,fi));pnid=int(path['node_id']) if path else None
   for source in ('raw_1m','fine_1m'):
    for cut in (.25,.5,.75):
     aa=lookup[(cid,source,cut,nid)];pp=lookup[(cid,source,cut,pnid)] if pnid is not None else None
     distance=float(aa['distance_to_persistent_scene_m']);pd=float(pp['distance_to_persistent_scene_m']) if pp else np.nan
     pxyz=np.array([float(path['center_'+k+'_m']) for k in 'xyz']) if path else np.full(3,np.nan)
     paired.append({'condition_id':cid,'frame_index':fi,'source':source,'cutoff':cut,'arm':arm,'anchor_node_id':nid,'path_node_id':pnid if pnid is not None else 'MISSING_PATH_FRAME','path_frame_present':int(path is not None),'anchor_distance_m':distance,'path_distance_m':pd,'distance_difference_anchor_minus_path_m':distance-pd,'anchor_p':aa['p_beta11'],'path_p':pp['p_beta11'] if pp else np.nan,'path_to_observed_anchor_m':float(np.linalg.norm(pxyz-lab[(cid,fi)])),'existing_anchor_gate_matched':int(np.linalg.norm(pxyz-lab[(cid,fi)])<=3) if path else 'MISSING'})
 s.writecsv(O/'anchor_vs_winner_same_frame.csv',paired)
 by=defaultdict(list)
 for r in paired:by[(r['condition_id'],r['source'],r['cutoff'],r['arm'])].append(r)
 summary=[]
 for k,rs in by.items():
  d=np.array([r['distance_difference_anchor_minus_path_m'] for r in rs]);valid=np.isfinite(d)
  summary.append({'condition_id':k[0],'source':k[1],'cutoff':k[2],'arm':k[3],'anchor_frames':len(rs),'paired_finite_frames':int(valid.sum()),'missing_path_frames':sum(not r['path_frame_present'] for r in rs),'paired_anchor_median_m':float(np.median([r['anchor_distance_m'] for r in rs if np.isfinite(r['distance_difference_anchor_minus_path_m'])])) if valid.any() else np.nan,'paired_path_median_m':float(np.median([r['path_distance_m'] for r in rs if np.isfinite(r['distance_difference_anchor_minus_path_m'])])) if valid.any() else np.nan,'paired_positive_fraction':float(np.mean(d[valid]>0)) if valid.any() else np.nan,**{'paired_difference_'+n:v for n,v in describe(d).items()}})
 s.writecsv(O/'same_frame_winner_contrast_summary.csv',summary)
 sensitivity=[];milestones=[]
 for cid in s.CIDS:
  for source in ('raw_1m','fine_1m','fine_004m'):
   with np.load(O/f'cell_history__{source}__{cid}.npz') as z:
    offsets=z['offsets'];weights=z['raw_count'];h=z['prior_hits'];p=z['p_beta11'];nf=len(offsets)-1
    for cut in (.25,.5,.75) if source!='fine_004m' else (.5,):
     for period in ('all','prior_history_ge2','last_third'):
      fs=[fi for fi in range(nf) if period=='all' or period=='prior_history_ge2' and fi>=2 or period=='last_third' and fi>=2*nf/3]
      total=persistent=occupied=pcells=0
      for fi in fs:
       lo,hi=offsets[fi:fi+2];mask=(h[lo:hi]>=2)&(p[lo:hi]>=cut);total+=int(weights[lo:hi].sum());persistent+=int(weights[lo:hi][mask].sum());occupied+=hi-lo;pcells+=int(mask.sum())
      sensitivity.append({'condition_id':cid,'source':source,'persistent_cutoff':cut,'period':period,'frames':len(fs),'returns':total,'return_fraction_in_persistent_cells':persistent/total,'current_cell_fraction_persistent':pcells/occupied,'return_fraction_in_nonpersistent_cells':1-persistent/total})
   stats=[r for r in s.readcsv(O/'frame_scene_statistics.csv') if r['condition_id']==cid and r['source']==source]
   for fi in sorted(set([2,5,10,20,nf-1])):
    if fi<nf:milestones.append(stats[fi])
 s.writecsv(O/'persistent_cutoff_scene_sensitivity.csv',sensitivity);s.writecsv(O/'fixed_causal_time_milestones.csv',milestones)
 s.jwrite(O/'supplement_manifest.json',{'utc':s.utc(),'command':'PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/scene_spatiotemporal_background_audit/supplement_summaries.py','source_sha256':s.sha(__file__),'phase':'POST_SCENE_FEATURE_RECEIPT_FIXED_RULE_DESCRIPTIVE_SUMMARIES','rules_unchanged':True,'outputs_sha256':{n:s.sha(O/n) for n in ['anchor_vs_winner_same_frame.csv','same_frame_winner_contrast_summary.csv','persistent_cutoff_scene_sensitivity.csv','fixed_causal_time_milestones.csv']}})
if __name__=='__main__':run()
