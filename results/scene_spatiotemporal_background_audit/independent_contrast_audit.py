import csv,json,hashlib,math
from pathlib import Path
from collections import defaultdict
from datetime import datetime,timezone
import numpy as np
from scipy.stats import spearmanr
R=Path('/home/xin/FMCW_LIDAR');O=R/'results/scene_spatiotemporal_background_audit'
def read(p):
 with open(p,encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def num(v):return float(v)
def dsum(x):
 x=np.asarray(x,float);x=x[np.isfinite(x)]
 if len(x)==0:return {'n':0}
 q=np.quantile(x,[.25,.5,.75]);return {'n':len(x),'q25':float(q[0]),'median':float(q[1]),'q75':float(q[2]),'iqr':float(q[2]-q[0]),'min':float(x.min()),'max':float(x.max())}
receipt=json.loads((O/'SCENE_FEATURES_COMPLETE.json').read_text())
assert receipt['label_or_gnss_used'] is False
assert all(sha(O/p)==h for p,h in receipt['outputs'].items())
F=read(O/'node_scene_features.csv');L=read(R/'results/research_dev_v0/silver_labels.csv');P=read(R/'results/e18_mechanism_generalization_audit/selected_paths.csv')
all_lookup={(r['condition_id'],r['source'],int(r['node_id'])):r for r in F if num(r['persistent_cutoff'])==.5}
F=[r for r in F if r['source']=='raw_1m' and num(r['persistent_cutoff'])==.5]
lookup={(r['condition_id'],int(r['node_id'])):r for r in F}
cids=json.loads((O/'protocol.json').read_text())['cohort']
G={}
for cid in cids:
 for r in read(R/f'results/background_motion_dp_mvp/graphs/fine_4cm_pregeometry__{cid}__nodes.csv'):G[cid,int(r['node_id'])]=r
labels=defaultdict(dict)
for a in L:labels[a['condition_id']][int(a['frame_index'])]=a
path=defaultdict(list)
for p in P:
 if p['family']=='fine_4cm_pregeometry':path[p['condition_id'],p['arm']].append(p)
summary={}
for cid in cids:
 ns=[r for (c,i),r in G.items() if c==cid];byframe=defaultdict(list)
 for r in ns:byframe[int(r['frame_index'])].append(r)
 anchor_rows=[];sfpercent=[]
 for fi,a in sorted(labels[cid].items()):
  ap=np.array([num(a['center_'+k+'_m']) for k in 'xyz']);nearest=min(byframe[fi],key=lambda n:np.linalg.norm(np.array([num(n['center_'+k+'_m']) for k in 'xyz'])-ap));af=lookup[cid,int(nearest['node_id'])];anchor_rows.append((fi,af))
  ds=np.array([num(lookup[cid,int(r['node_id'])]['distance_to_persistent_scene_m']) for r in byframe[fi]]);ds=ds[np.isfinite(ds)];x=num(af['distance_to_persistent_scene_m'])
  if np.isfinite(x):sfpercent.append((np.sum(ds<x)+.5*np.sum(ds==x))/len(ds))
 out={'anchors':len(anchor_rows),'anchor_nearest_raw_distance':dsum([num(a['distance_to_persistent_scene_m']) for fi,a in anchor_rows]),'anchor_nearest_raw_p':dsum([num(a['p_beta11']) for fi,a in anchor_rows]),'anchor_same_frame_distance_percentile':dsum(sfpercent),'paths':{}}
 for arm in ['original','background_only','background_motion_E18','motion_only']:
  ps=sorted(path[cid,arm],key=lambda p:int(p['path_order']));ds=[];sfd=[];sfad=[];sfp=[];sfap=[];geom=[];slowclose=[];lowcountclose=[];positions=[];times=[];equal_H=[]
  for p in ps:
   fi=int(p['frame_index']);f=lookup[cid,int(p['node_id'])];d=num(f['distance_to_persistent_scene_m']);ep=num(G[cid,int(p['node_id'])]['background_p_beta11']);positions.append([num(p['center_'+k+'_m']) for k in 'xyz']);times.append(num(p['bag_time']));ds.append(d)
   # Frozen occupancy bin and fixed neighborhood only; not classification.
   slowclose.append(np.isfinite(d) and d<=5 and ep<.25)
   lowcountclose.append(np.isfinite(d) and d<=5 and int(G[cid,int(p['node_id'])]['background_prior_hits'])<=1)
   if fi in labels[cid]:
    a=labels[cid][fi];ap=np.array([num(a['center_'+k+'_m']) for k in 'xyz']);geom.append(np.linalg.norm(np.array(positions[-1])-ap));sfd.append(d);sfp.append(ep)
    af=next(r for fi2,r in anchor_rows if fi2==fi);sfad.append(num(af['distance_to_persistent_scene_m']));sfap.append(num(G[cid,int(af['node_id'])]['background_p_beta11']))
    Hanchor=int(G[cid,int(af['node_id'])]['background_prior_hits']);Hpath=int(G[cid,int(p['node_id'])]['background_prior_hits'])
    if Hanchor==Hpath and np.isfinite(d) and np.isfinite(sfad[-1]):equal_H.append({'frame_index':fi,'identical_E18_H':Hanchor,'identical_E18_F':fi,'identical_E18_p':ep,'anchor_node_raw_distance_m':sfad[-1],'path_node_raw_distance_m':d,'anchor_minus_path_distance_m':sfad[-1]-d})
  xyz=np.array(positions);t=np.array(times);disp=np.diff(xyz,axis=0);step=np.linalg.norm(disp,axis=1);vel=disp/np.diff(t)[:,None]
  denom=step[1:]*step[:-1];valid=denom>0
  dot=np.sum(disp[1:]*disp[:-1],axis=1)[valid]/denom[valid];dot=np.clip(dot,-1,1)
  def runs(mask):
   rs=[];start=None
   for i,ok in enumerate(mask):
    fi=int(ps[i]['frame_index']);consecutive=i>0 and fi==int(ps[i-1]['frame_index'])+1
    if ok and (start is None or consecutive):
     if start is None:start=i
    elif ok:
     rs.append((start,i-1));start=i
    elif start is not None:rs.append((start,i-1));start=None
   if start is not None:rs.append((start,len(ps)-1))
   return [{'first_frame':int(ps[a]['frame_index']),'last_frame':int(ps[b]['frame_index']),'nodes':b-a+1,'duration_s':float(t[b]-t[a]),'raw_distance':dsum(ds[a:b+1]),'e18_p':dsum([num(G[cid,int(p['node_id'])]['background_p_beta11']) for p in ps[a:b+1]])} for a,b in sorted(rs,key=lambda ab:ab[1]-ab[0],reverse=True)[:3]]
  out['paths'][arm]={'nodes':len(ps),'fullpath_raw_distance':dsum(ds),'anchor_frame_nodes':len(sfd),'same_frame_path_raw_distance':dsum(sfd),'same_frame_anchor_raw_distance':dsum(sfad),'same_frame_anchor_minus_path_distance':dsum(np.array(sfad)-np.array(sfd)),'same_frame_e18_p_path':dsum(sfp),'same_frame_e18_p_anchor':dsum(sfap),'same_frame_anchor_geometry_distance':dsum(geom),'same_frame_geometry_within_existing_3m':int(np.sum(np.array(geom)<=3)),'step_m':dsum(step),'speed_mps':dsum(step/np.diff(t)),'turn_angle_deg':dsum(np.degrees(np.arccos(dot))),'turn_above_90_fraction':float(np.mean(dot<0)),'largest_step_frames':[[int(ps[i]['frame_index']),int(ps[i+1]['frame_index']),float(step[i])] for i in np.argsort(step)[-3:][::-1]],'longest_frozen_low_p_close_runs':runs(slowclose),'longest_H_le_1_close_runs':runs(lowcountclose)}
  out['paths'][arm]['same_frame_identical_E18_H_F_contrasts']=equal_H
 # Occupancy nonredundancy: exact same E18 H and F (same frame).
 bgroups=defaultdict(list)
 for n in ns:
  f=lookup[cid,int(n['node_id'])];d=num(f['distance_to_persistent_scene_m'])
  if np.isfinite(d):bgroups[int(n['frame_index']),int(n['background_prior_hits'])].append(d)
 spreads=[]
 for (fi,H),ds in bgroups.items():
  if len(ds)>=2:spreads.append({'frame_index':fi,'H':H,'F':fi,'e18_p_beta11':(H+1)/(fi+2),**dsum(ds)})
 alln=[];allep=[];alld=[]
 for n in ns:
  f=lookup[cid,int(n['node_id'])];d=num(f['distance_to_persistent_scene_m'])
  if np.isfinite(d):allep.append(num(n['background_p_beta11']));alld.append(d)
 out['nonredundancy']={'spearman_raw_distance_vs_exact_E18_p':float(spearmanr(allep,alld).statistic),'same_frame_same_E18_H_groups':len(spreads),'groups_with_nonzero_distance_range':sum(r['max']-r['min']>1e-9 for r in spreads),'H0_examples_largest_distance_iqr':sorted([r for r in spreads if r['H']==0],key=lambda r:r['iqr'],reverse=True)[:3]}
 out['same_frame_representation_comparison']={}
 for source in ('raw_1m','fine_1m','fine_004m'):
  ar=[num(all_lookup[cid,source,int(af['node_id'])]['distance_to_persistent_scene_m']) for fi,af in anchor_rows]
  entry={'nearest_anchor_node_distance':dsum(ar)}
  for arm in ('original','background_only'):
   pr=[num(all_lookup[cid,source,int(p['node_id'])]['distance_to_persistent_scene_m']) for p in path[cid,arm] if int(p['frame_index']) in labels[cid]]
   entry[arm+'_path_distance']=dsum(pr)
  out['same_frame_representation_comparison'][source]=entry
 summary[cid]=out
result={'status':'TESTED independent post-freeze descriptive audit','utc':datetime.now(timezone.utc).isoformat(),'command':'PYTHONDONTWRITEBYTECODE=1 fmcw/bin/python /tmp/scene_independent_contrast_audit.py','script_sha256':sha(__file__),'scene_receipt_utc':receipt['utc'],'scene_receipt_sha256':sha(O/'SCENE_FEATURES_COMPLETE.json'),'protocol_sha256':sha(O/'protocol.json'),'no_map_rebuild_or_rescore':True,'inputs_sha256':{str(p.relative_to(R)):sha(p) for p in [O/'node_scene_features.csv',R/'results/research_dev_v0/silver_labels.csv',R/'results/e18_mechanism_generalization_audit/selected_paths.csv']},'definitions':{'primary':'raw_1m cutoff 0.5 node-centroid distance to causal historical persistent cell AABBs','p':'actual frozen E18 fine_1m Beta(1,1) occupancy from graph','sameframe':'only preexisting observed-anchor frames with an existing path node, cold-start NaN excluded pairwise from feature summaries','low_p_close_segments':'existing path; E18 p<0.25 already frozen summary bin and distance<=5m frozen local-neighborhood scale; descriptive only','H_le_1_close_segments':'existing path; E18 H<=1 frozen low-revisit rule and distance<=5m','path_angles':'successive displacement directions, time irregularity retained, zero displacement pairs excluded; geometry cannot establish same object identity'},'sequence':summary}
(O/'independent_contrast_audit.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
print(json.dumps({cid:{k:v for k,v in s.items() if k!='paths'} for cid,s in summary.items()},indent=2))
for cid in ('100m_cross_10','300m_cross_10'):
 print('\n',cid)
 for arm,r in summary[cid]['paths'].items():
  print(arm,json.dumps(r))
