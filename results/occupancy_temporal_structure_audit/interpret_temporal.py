#!/usr/bin/env python3
"""Post-feature descriptive exact-frequency contrasts. No fit or rescore."""
from collections import defaultdict
import json,time
import numpy as np
from scipy.stats import spearmanr
import temporal_audit as a

OUT=a.OUT;ROOT=a.ROOT;FEATURES=a.FEATURES
GROUPS=('all_observed_anchor_matched_fine_nodes','nearest_observed_anchor_node','fixed_silver_feasibility_witness','original_unconditional_winner','background_only_winner','joint_E18_winner','historical_persistent_clutter_reference')
CONTROLS=GROUPS[2:]
KILLGROUPS=('original_unconditional_winner','background_only_winner','historical_persistent_clutter_reference')
def stats(v):
 v=np.asarray(v,float);v=v[np.isfinite(v)]
 if not len(v):return {'finite_n':0,'q25':np.nan,'median':np.nan,'q75':np.nan,'iqr':np.nan,'min':np.nan,'max':np.nan}
 q=np.quantile(v,[.25,.5,.75]);return {'finite_n':len(v),'q25':float(q[0]),'median':float(q[1]),'q75':float(q[2]),'iqr':float(q[2]-q[0]),'min':float(v.min()),'max':float(v.max())}
def cliff(x,y):
 x=np.asarray(x,float);x=x[np.isfinite(x)];y=np.sort(np.asarray(y,float));y=y[np.isfinite(y)]
 return float(np.sum(np.searchsorted(y,x,'left')+np.searchsorted(y,x,'right')-len(y))/(len(x)*len(y))) if len(x) and len(y) else np.nan
def cell(r):return tuple(int(r[k]) for k in ('cell_x','cell_y','cell_z'))
def run():
 start=time.perf_counter();a.check();receipt=json.loads((OUT/'TEMPORAL_FEATURES_COMPLETE.json').read_text())
 for p,h in receipt['outputs'].items():assert a.sha(OUT/p)==h
 join_utc=a.utc();assert join_utc>receipt['utc']
 rows=a.readcsv(OUT/'all_node_temporal_features.csv');features={}
 for r in rows:
  for k in r:
   if k not in ('condition_id','family','support_state','p_bin'):r[k]=float(r[k])
  features[(r['condition_id'],r['family'],int(r['node_id']))]=r
 nframes={r['condition_id']:int(r['frames']) for r in a.readcsv(OUT/'acquisition_cadence.csv')}
 anchors=a.readcsv(a.SCENE/'anchor_node_join.csv');assert len(anchors)==80
 selected=a.readcsv(ROOT/'results/e18_mechanism_generalization_audit/selected_paths.csv')
 fixed=a.readcsv(ROOT/'results/background_motion_dp_mvp/fixed_reference_paths.csv')
 groups=defaultdict(set);nearest={};pathframe={}
 for r in anchors:
  cid=r['condition_id'];fi=int(r['frame_index']);nid=int(r['nearest_node_id']);nearest[(cid,fi)]=nid
  groups[(cid,'nearest_observed_anchor_node')].add(('fine_4cm_pregeometry',nid))
  for n in r['matched_node_ids'].split(';'):groups[(cid,'all_observed_anchor_matched_fine_nodes')].add(('fine_4cm_pregeometry',int(n)))
 arms={'original':'original_unconditional_winner','background_only':'background_only_winner','background_motion_E18':'joint_E18_winner'}
 for r in selected:
  if r['family']=='fine_4cm_pregeometry' and r['arm'] in arms:
   cid=r['condition_id'];g=arms[r['arm']];nid=int(r['node_id']);groups[(cid,g)].add((r['family'],nid));pathframe[(cid,g,int(r['frame_index']))]=(r['family'],nid)
 for r in fixed:
  g='fixed_silver_feasibility_witness' if r['family']=='fine_4cm_pregeometry' and r['path_type']=='silver_feasible' else 'historical_persistent_clutter_reference' if r['family']=='historical_1m_graph' and r['path_type']=='top_clutter_full' else None
  if g:
   cid=r['condition_id'];nid=int(r['node_id']);groups[(cid,g)].add((r['family'],nid));pathframe[(cid,g,int(r['frame_index']))]=(r['family'],nid)
 population=[];bucket=defaultdict(list);durations=[];degeneracy=[]
 for (cid,g),members in groups.items():
  rs=[features[(cid,family,n)] for family,n in sorted(members)]
  for r in rs:population.append({'group':g,**r});bucket[(cid,g)].append(r)
  ts=[r['bag_time'] for r in rs];fs=[r['frame_index'] for r in rs]
  durations.append({'condition_id':cid,'group':g,'nodes':len(rs),'first_frame':int(min(fs)),'last_frame':int(max(fs)),'sample_span_s':max(ts)-min(ts),'frame_index_median':float(np.median(fs)),'cold_no_prior_frames':sum(r['prior_frames']==0 for r in rs),'recent_window_unknown_nodes':sum(not np.isfinite(r['recent_5_prior_frame_fraction']) for r in rs),'never_previously_occupied_nodes':sum(r['prior_hits']==0 for r in rs)})
  for name,fn in [('H0',lambda r:r['prior_hits']==0),('H1',lambda r:r['prior_hits']==1),('Hge2',lambda r:r['prior_hits']>=2)]:
   rr=[r for r in rs if fn(r)];degeneracy.append({'condition_id':cid,'group':g,'H_stratum':name,'nodes':len(rr),'warm_Fge5_nodes':sum(r['prior_frames']>=5 for r in rr),'recency_unknown':sum(not np.isfinite(r['age_s']) for r in rr),'run_episodes_description':'H0: known0 after F>0, F0 UNKNOWN' if name=='H0' else 'H1: both1 fixed by H' if name=='H1' else 'potential nondegenerate structure'})
 a.writecsv(OUT/'population_temporal_features.csv',population);a.writecsv(OUT/'population_exposure.csv',durations);a.writecsv(OUT/'H_stratum_support.csv',degeneracy)
 # Fixed narrow bins: full support and warm chronological phases separately.
 summaries=[];binsupport=[];bindiffs=[]
 for cid in a.CIDS:
  nf=nframes[cid]
  for period in ('all_Fge1','warm_Fge5','warm_first_third','warm_middle_third','warm_last_third'):
   def admitted(r):
    f=int(r['frame_index'])
    if period=='all_Fge1':return f>=1
    if f<5:return False
    return period=='warm_Fge5' or period=='warm_first_third' and f<nf/3 or period=='warm_middle_third' and nf/3<=f<2*nf/3 or period=='warm_last_third' and f>=2*nf/3
   for b in range(20):
    bygroup={g:[r for r in bucket[(cid,g)] if admitted(r) and int(r['p_bin'])==b] for g in GROUPS}
    for g,rs in bygroup.items():
     binsupport.append({'condition_id':cid,'period':period,'p_bin':b,'p_left':b*.05,'p_right':(b+1)*.05,'group':g,'nodes':len(rs),'unique_frames':len({int(r['frame_index']) for r in rs}),'frame_index_median':float(np.median([r['frame_index'] for r in rs])) if rs else np.nan,'prior_hits_median':float(np.median([r['prior_hits'] for r in rs])) if rs else np.nan})
     for f in ('p_beta11',*FEATURES):summaries.append({'condition_id':cid,'period':period,'p_bin':b,'group':g,'feature':f,'nodes':len(rs),**stats([r[f] for r in rs])})
    ar=bygroup['nearest_observed_anchor_node']
    for g in CONTROLS:
     cr=bygroup[g]
     for f in FEATURES:
      x=[r[f] for r in ar];y=[r[f] for r in cr];xs=stats(x);ys=stats(y)
      bindiffs.append({'condition_id':cid,'period':period,'p_bin':b,'p_left':b*.05,'p_right':(b+1)*.05,'comparison_group':g,'feature':f,'anchor_finite_n':xs['finite_n'],'comparison_finite_n':ys['finite_n'],'anchor_median':xs['median'],'comparison_median':ys['median'],'median_difference_anchor_minus_comparison':xs['median']-ys['median'],'cliff_delta_descriptive':cliff(x,y),'support':'NO_COMMON_SUPPORT' if not xs['finite_n'] or not ys['finite_n'] else 'SMALL_SUPPORT' if min(xs['finite_n'],ys['finite_n'])<3 else 'AVAILABLE_EXPOSURE_MAY_DIFFER'})
 a.writecsv(OUT/'frequency_bin_support.csv',binsupport);a.writecsv(OUT/'frequency_bin_distributions.csv',summaries);a.writecsv(OUT/'frequency_bin_contrasts.csv',bindiffs)
 pairs=[];killframe=[]
 for (cid,fi),nid in sorted(nearest.items()):
  ar=features[(cid,'fine_4cm_pregeometry',nid)]
  for g in CONTROLS:
   key=pathframe.get((cid,g,fi));cr=features[(cid,*key)] if key else None
   same=cr is not None and cr['prior_hits']==ar['prior_hits'];samecell=cr is not None and cell(cr)==cell(ar)
   for f in FEATURES:
    av=ar[f];cv=cr[f] if cr else np.nan;finite=same and np.isfinite(av) and np.isfinite(cv)
    pairs.append({'condition_id':cid,'frame_index':fi,'comparison_group':g,'feature':f,'anchor_node_id':nid,'comparison_family':key[0] if key else 'MISSING','comparison_node_id':key[1] if key else 'MISSING','anchor_H':int(ar['prior_hits']),'comparison_H':int(cr['prior_hits']) if cr else 'UNKNOWN','F':fi,'p_anchor':ar['p_beta11'],'p_comparison':cr['p_beta11'] if cr else np.nan,'same_frame_exact_H':int(same),'same_centroid_cell':int(samecell),'anchor_value':av,'comparison_value':cv,'occupancy_conditioned_difference':float(av-cv) if finite else np.nan,'status':'PATH_FRAME_MISSING' if cr is None else 'NONMATCHING_H' if not same else 'FEATURE_UNKNOWN' if not finite else 'IDENTICAL_CELL_DETERMINISTIC_TIE' if samecell else 'DISTINCT_CELL_EXACT_FREQUENCY'})
  for f in FEATURES:
   eligible=fi>=5 and ar['prior_hits']>=(2 if f in ('longest_previous_run','previous_episodes') else 1) and np.isfinite(ar[f])
   controls={}
   if eligible:
    for g in KILLGROUPS:
     key=pathframe.get((cid,g,fi));cr=features[(cid,*key)] if key else None
     if cr is not None and cr['prior_hits']==ar['prior_hits'] and cell(cr)!=cell(ar) and np.isfinite(cr[f]):controls[cell(cr)]=cr[f]
   cv=float(np.median(list(controls.values()))) if controls else np.nan
   killframe.append({'condition_id':cid,'frame_index':fi,'feature':f,'anchor_node_id':nid,'H':int(ar['prior_hits']),'F':fi,'p_beta11':ar['p_beta11'],'eligible_non_degenerate_warm_anchor':int(eligible),'distinct_exact_H_control_cells':len(controls),'anchor_value':ar[f],'control_median_value':cv,'frame_median_difference':float(ar[f]-cv) if np.isfinite(cv) else np.nan,'support':'AVAILABLE' if controls else 'UNKNOWN_NO_ELIGIBLE_DISTINCT_CELL_COMMON_SUPPORT'})
 a.writecsv(OUT/'same_frame_exact_H_pairs.csv',pairs);a.writecsv(OUT/'kill_frame_contrasts.csv',killframe)
 exactsummary=[]
 for cid in a.CIDS:
  for g in CONTROLS:
   for f in FEATURES:
    rs=[r for r in pairs if r['condition_id']==cid and r['comparison_group']==g and r['feature']==f]
    for condition in ('all_exact_H','warm_Fge5_exact_H','warm_Fge5_distinct_cell_exact_H','warm_Fge5_distinct_cell_non_degenerate'):
     rr=[r for r in rs if r['same_frame_exact_H'] and (condition=='all_exact_H' or r['F']>=5) and (condition not in ('warm_Fge5_distinct_cell_exact_H','warm_Fge5_distinct_cell_non_degenerate') or not r['same_centroid_cell']) and (condition!='warm_Fge5_distinct_cell_non_degenerate' or r['anchor_H']>=(2 if f in ('longest_previous_run','previous_episodes') else 1))]
     vals=np.array([r['occupancy_conditioned_difference'] for r in rr]);finite=vals[np.isfinite(vals)];tol=a.PROTOCOL['numerical_zero_tolerance'][f]
     exactsummary.append({'condition_id':cid,'comparison_group':g,'feature':f,'condition':condition,'anchor_frames':len(rs),'matching_frames':len(rr),'unknown_quantity_frames':int(np.sum(~np.isfinite(vals))),'identical_cell_frames':sum(r['same_centroid_cell'] for r in rr),'positive_frames':int(np.sum(finite>tol)),'negative_frames':int(np.sum(finite< -tol)),'tie_frames':int(np.sum(np.abs(finite)<=tol)),**stats(vals)})
 a.writecsv(OUT/'exact_frequency_contrast_summary.csv',exactsummary)
 killseq=[];killdecision=[]
 for f in FEATURES:
  direction=[];supported=True
  for cid in a.CIDS:
   rs=[r for r in killframe if r['condition_id']==cid and r['feature']==f];v=np.array([r['frame_median_difference'] for r in rs]);v=v[np.isfinite(v)];ss=stats(v);tol=a.PROTOCOL['numerical_zero_tolerance'][f]
   sign=1 if ss['finite_n'] and ss['median']>tol else -1 if ss['finite_n'] and ss['median']< -tol else 0
   agreement=float(np.mean(v*sign>tol)) if len(v) and sign else 0 if len(v) else np.nan
   valid=len(v)>=3 and sign!=0 and agreement>=2/3;supported&=valid;direction.append(sign)
   killseq.append({'condition_id':cid,'feature':f,'finite_distinct_paired_frames':len(v),'eligible_anchor_frames':sum(r['eligible_non_degenerate_warm_anchor'] for r in rs),'median_direction':sign,'same_direction_fraction':agreement,'sequence_pass':int(valid),'failure_reason':'INSUFFICIENT_COMMON_SUPPORT' if len(v)<3 else 'ZERO_MEDIAN' if not sign else 'DIRECTION_UNSTABLE' if agreement<2/3 else 'PASS',**ss})
  passed=supported and len(set(direction))==1 and direction[0]!=0
  killdecision.append({'feature':f,'passes_all_five':bool(passed),'same_nonzero_direction_all_five':bool(len(set(direction))==1 and direction[0]!=0),'sequence_directions':direction})
 a.writecsv(OUT/'kill_sequence_summary.csv',killseq)
 final='STABLE_CONDITIONAL_PHENOMENON_ONLY_NOT_SCORE_AUTHORIZATION' if any(r['passes_all_five'] for r in killdecision) else 'E18_OCCUPANCY_SUFFICIENT_AT_CURRENT_EVIDENCE_LEVEL_NO_E21_BACKGROUND_MODEL'
 a.jwrite(OUT/'kill_decision.json',{'utc':a.utc(),'status':final,'criteria':{k:v for k,v in a.PROTOCOL.items() if k.startswith('kill_')},'features':killdecision,'interpretation':'insufficient support is UNKNOWN, not zero effect; decision is evidence-level adequacy, not mathematical sufficiency; no E21 implementation'})
 # Unlabelled all-fine-node spread shows deterministic/non-deterministic aspects.
 spreads=[];correlation=[]
 for cid in a.CIDS:
  rr=[r for r in rows if r['condition_id']==cid and r['family']=='fine_4cm_pregeometry'];gg=defaultdict(list)
  for r in rr:gg[(int(r['frame_index']),int(r['prior_hits']))].append(r)
  for (fi,h),rs in sorted(gg.items()):
   for f in FEATURES:
    st=stats([r[f] for r in rs]);spreads.append({'condition_id':cid,'frame_index':fi,'H':h,'F':fi,'p_beta11':rs[0]['p_beta11'],'feature':f,'nodes':len(rs),**st})
  for f in FEATURES:
   for reference in ('p_beta11','prior_hits','prior_frames'):
    x=np.array([r[f] for r in rr]);y=np.array([r[reference] for r in rr]);mask=np.isfinite(x)&np.isfinite(y);rho=float(spearmanr(x[mask],y[mask]).statistic) if mask.sum()>2 and np.ptp(x[mask]) and np.ptp(y[mask]) else np.nan
    correlation.append({'condition_id':cid,'feature':f,'reference':reference,'finite_n':int(mask.sum()),'spearman_descriptive':rho,'not_incremental_test':'global correlation confounds F; exact-H same-frame test is primary'})
 a.writecsv(OUT/'all_fine_same_frame_H_structure_spread.csv',spreads);a.writecsv(OUT/'global_reference_associations.csv',correlation)
 inputs=[a.SCENE/'anchor_node_join.csv',ROOT/'results/e18_mechanism_generalization_audit/selected_paths.csv',ROOT/'results/background_motion_dp_mvp/fixed_reference_paths.csv']
 a.jwrite(OUT/'interpretation_manifest.json',{'utc_started':join_utc,'utc_completed':a.utc(),'feature_receipt_utc':receipt['utc'],'command':'PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python results/occupancy_temporal_structure_audit/interpret_temporal.py','source_sha256':a.sha(__file__),'join_inputs':{str(p.relative_to(ROOT)):a.sha(p) for p in inputs},'features_sha256':a.sha(OUT/'all_node_temporal_features.csv'),'no_path_search_or_rescore':True,'kill_status':final,'elapsed_s':time.perf_counter()-start})
 print(final,flush=True)
if __name__=='__main__':run()
