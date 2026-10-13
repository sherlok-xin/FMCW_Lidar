import csv,json,math,hashlib,pathlib,platform,sys,numpy as np
root=pathlib.Path('/home/xin/FMCW_LIDAR');out=root/'results/rviz_gnss_lidar_sanity'
def read(p):return list(csv.DictReader(p.open(encoding='utf-8-sig',newline='')))
def vec(r,p,s,axes='xyz'):return np.array([float(r[p+a+s]) for a in axes])
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
frames={(r['condition_id'],int(r['frame_index'])):r for r in read(root/'results/gnss_conditioned_uav_phenotype/run_02/frames.csv')}
anchors={(r['condition_id'],int(r['frame_index'])):r for r in read(root/'results/research_dev_v0/silver_labels.csv')}
segments={(r['condition_id'],int(r['segment_index'])):r for r in read(root/'results/gnss_conditioned_uav_phenotype/run_02/gnss_source_segments.csv')}
regions={(r['condition_id'],int(r['frame_index']),r['alignment_scenario']):int(r['raw_points']) for r in read(root/'results/gnss_conditioned_uav_phenotype/run_02/region_features.csv') if r['region_role']=='UAV_CONDITIONED'}
cal=json.loads((root/'results/calibration_closure/alignment.json').read_text())['full_6dof'];R=np.array(cal['rotation_matrix']); rows=read(out/'all_anchor_gnss_discrepancies.csv'); seen=[];expected=[];maxerr={};checks=0
assert len(rows)==len(anchors)==80
def check(actual,want,label):
 global checks
 checks+=1
 if isinstance(want,(str,bool,np.bool_,int,np.integer)):
  got=(actual=='True') if isinstance(want,(bool,np.bool_)) else int(actual) if isinstance(want,(int,np.integer)) else actual
  assert got==want,(label,got,want)
 else:
  got=float(actual);diff=abs(got-want); maxerr[label.split(':')[-1]]=max(maxerr.get(label.split(':')[-1],0),diff)
  assert math.isclose(got,want,rel_tol=0,abs_tol=2e-11),(label,got,want,diff)
for r in rows:
 key=(r['condition_id'],int(r['frame_index']));seen.append(key);a=anchors[key];f=frames[key];seg=segments[key[0],int(f['nominal_segment_index'])]
 g=vec(f,'nominal_center_','_m');anch=vec(a,'center_','_m');e=anch-g;eu=R.T@e;v=vec(seg,'chord_velocity_','_mps','enu');speed=np.linalg.norm(v);t=v/speed;c=np.array([-t[1],t[0],0.]);c/=np.linalg.norm(c);b=np.cross(t,c);B=np.stack([t,c,b],axis=1);d=B.T@eu
 hspeed=np.linalg.norm(v[:2]); ht=np.array([v[0],v[1],0])/hspeed;n=np.linalg.norm(e);tau=e@ (R@v)/(speed**2);q=float(f['nominal_query_gps_time_unix_s']);left=float(seg['gps_start_unix_s']);right=float(seg['gps_end_unix_s']);ss=[s for (cid,_),s in segments.items() if cid==key[0]];lo=min(float(s['gps_start_unix_s']) for s in ss);hi=max(float(s['gps_end_unix_s']) for s in ss)
 vals={'discrepancy_norm_m':n,'chord_speed_mps':speed,'chord_horizontal_speed_mps':hspeed,'chord_inclination_deg':math.degrees(math.asin(v[2]/speed)), 'd_parallel_m':d[0],'d_cross_horizontal_m':d[1],'d_vertical_normal_m':d[2],'d_up_enu_m':eu[2],'d_parallel_horizontal_m':eu@ht,'equivalent_time_bias_s':tau,'equivalent_time_bias_horizontal_s':eu@ht/hspeed,'counterfactual_query_plus_tau_unix_s':q+tau,'parallel_squared_fraction':d[0]**2/n**2,'cross_squared_fraction':d[1]**2/n**2,'vertical_normal_squared_fraction':d[2]**2/n**2,'error_to_forward_chord_deg':math.degrees(math.acos(np.clip(d[0]/n,-1,1))),'abs_error_to_chord_axis_deg':math.degrees(math.acos(np.clip(abs(d[0])/n,-1,1))), 'segment_fraction':float(f['nominal_segment_fraction']),'nearest_gnss_knot_distance_s':float(f['nominal_nearest_gnss_knot_distance_s']),'segment_dt_s':float(seg['segment_dt_s']),'query_gps_time_unix_s':q,'frozen_time_offset_s':float(f['nominal_time_offset_s'])}
 for k,x in vals.items():check(r[k],x,str(key)+':'+k)
 for p,x in [('gnss_analysis_',g),('anchor_analysis_',anch),('error_analysis_',e),('chord_velocity_analysis_',R@v),('along_unit_analysis_',R@t),('cross_unit_analysis_',R@c),('vertical_normal_unit_analysis_',R@b)]:
  for axis,value in zip('xyz',x):check(r[p+axis],value,str(key)+':'+p+axis)
 for axis,x in zip('enu',eu):check(r['error_enu_'+axis+'_m'],x,str(key)+':error_enu_'+axis)
 for axis,x in zip('enu',v):check(r['chord_velocity_enu_'+axis+'_mps'],x,str(key)+':velocity_enu_'+axis)
 for k,x in [('at_last_gnss_endpoint',abs(q-hi)<=1e-6),('counterfactual_within_same_chord',left<=q+tau<=right),('counterfactual_within_gnss_support',lo<=q+tau<=hi),('candidate_id',a['candidate_id']),('gnss_left_source_cell',seg['left_source_cell']),('gnss_right_source_cell',seg['right_source_cell']),('bag_record_time',f"{f['bag_time_sec']}.{int(f['bag_time_nsec']):09d}"),('anchor_raw_support_prior',int(a['evidence_raw_points']))]:check(r[k],x,str(key)+':'+k)
 for recipe in ['GNSS_1m','GNSS_3m','GNSS_10m','ANCHOR_1m','ANCHOR_3m']:check(r[recipe+'_raw_count'],regions[key[0],key[1],recipe],str(key)+':count_'+recipe)
 assert np.allclose(B.T@B,np.eye(3),rtol=0,atol=1e-14)
 assert np.allclose(R@B@d,e,rtol=0,atol=1e-13)
 assert np.allclose(R@(ht*(eu@ht)+c*d[1]+np.array([0,0,eu[2]])),e,rtol=0,atol=1e-13)
 expected.append((key,vals,e,d,n))
assert set(seen)==set(anchors)
def qlinear(a,p):
 s=sorted(a);h=(len(s)-1)*p;i=int(math.floor(h));j=int(math.ceil(h));return s[i]+(s[j]-s[i])*(h-i)
def desc(a):
 return {'n':len(a),'median':qlinear(a,.5),'q25':qlinear(a,.25),'q75':qlinear(a,.75),'iqr':qlinear(a,.75)-qlinear(a,.25),'min_signed':min(a),'max_signed':max(a),'median_abs':qlinear(list(map(abs,a)),.5),'q25_abs':qlinear(list(map(abs,a)),.25),'q75_abs':qlinear(list(map(abs,a)),.75),'iqr_abs':qlinear(list(map(abs,a)),.75)-qlinear(list(map(abs,a)),.25),'max_abs':max(map(abs,a))}
for r in read(out/'sequence_component_summary.csv'):
 a=[x[1][r['metric']] for x in expected if x[0][0]==r['condition_id']]
 for k,x in desc(a).items():check(r[k],x,r['condition_id']+':summary_'+k)
for r in read(out/'chord_time_bias_summary.csv'):
 a=[x for x in expected if x[0][0]==r['condition_id'] and int(frames[x[0]]['nominal_segment_index'])==int(r['gnss_segment_index'])]
 for k,x in desc([x[1]['equivalent_time_bias_s'] for x in a]).items():check(r[k],x,r['condition_id']+':chord_'+k)
 pos=sum(x[1]['d_parallel_m']>1e-12 for x in a);neg=sum(x[1]['d_parallel_m']<-1e-12 for x in a)
 for k,x in [('positive_count',pos),('negative_count',neg),('both_signs_in_same_chord',pos>0 and neg>0)]:check(r[k],x,r['condition_id']+':chord_'+k)
for r in read(out/'sequence_direction_consistency.csv'):
 a=[x for x in expected if x[0][0]==r['condition_id']];E=np.array([x[2] for x in a]);D=np.array([x[3] for x in a]);N=np.array([x[4] for x in a]);U=E/N[:,None];L=D/N[:,None]
 for k,x in [('fixed_frame_unit_vector_resultant',np.linalg.norm(U.mean(axis=0))),('chord_frame_unit_vector_resultant',np.linalg.norm(L.mean(axis=0))),('total_parallel_squared_fraction',sum(D[:,0]**2)/sum(N**2)),('total_cross_squared_fraction',sum(D[:,1]**2)/sum(N**2)),('total_vertical_normal_squared_fraction',sum(D[:,2]**2)/sum(N**2))]:check(r[k],x,r['condition_id']+':direction_'+k)
 for metric in ['d_parallel_m','d_cross_horizontal_m','d_vertical_normal_m','d_up_enu_m']:
  V=np.array([x[1][metric] for x in a]);pos=int(sum(V>1e-12));neg=int(sum(V<-1e-12))
  for k,x in [('positive_count',pos),('negative_count',neg),('numerical_zero_count',len(a)-pos-neg),('majority_sign_fraction',max(pos,neg)/len(a))]:check(r[metric+'_'+k],x,r['condition_id']+':'+metric+'_'+k)
prov=json.loads((out/'decomposition_provenance.json').read_text())
for path,h in prov['input_sha256'].items():assert sha(root/path)==h
for path,h in prov['files_sha256'].items():assert sha(out/path)==h
assert sha(out/'decompose_discrepancies.py')==prov['script_sha256']
audit={'status':'PASS','independent_method':'independent ENU-frame cross product basis projected through frozen R; manual sorted-data linear quantiles; source silver/frames/segments/region counts; no import of producer code','anchor_count':len(rows),'checked_scalar_values':checks,'maximum_scalar_absolute_difference':max(maxerr.values()),'maximum_scalar_difference_by_field':maxerr,'all_source_hashes_match_producer_provenance':True,'all_csv_hashes_match_producer_provenance':True,'orthonormal_and_two_reconstruction_checks_all_pass':True,'no_fit_no_optimizer_no_bag_read':True,'actual_bugs_found':[],'validator_script_sha256':sha(pathlib.Path(__file__)),'python':platform.python_version(),'numpy':np.__version__,'command':sys.argv,'validator_initial_failure':'First inline validator stopped on numpy.bool_ serialization/type handling; fixed in validator only; producer artifacts were not changed.'}
(out/'independent_decomposition_audit.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:v for k,v in audit.items() if k!='maximum_scalar_difference_by_field'},ensure_ascii=False,indent=2))
