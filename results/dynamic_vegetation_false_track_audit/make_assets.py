"""Write explicitly manual review records and descriptive plots, not a classifier."""
from pathlib import Path
import csv,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image,ImageDraw
O=Path(__file__).resolve().parent
# Manual judgments after RViz context inspection. No metric-based assignment rules.
manual=[
(0,'UNKNOWN','LOW','9 birth seed','x415,y-36,z14附近稀疏/杆状及冠层边缘场景','仅第9帧真实seed单点；之后均预测。无法区分植被、杆状结构或离群返回。'),
(1,'UAV_OPERATOR_CONFIRMED','OPERATOR_REPORT','9,30,60; operator inspected trajectory','中央上方开放空域，约x220，左右往返','用户明确逐轨迹指认为实际UAV；身份来源OPERATOR_VISUAL_IDENTITY，不是独立GT。'),
(2,'VEGETATION_ASSOCIATED','MEDIUM','9,10,11,12,13','中央场景树冠/枝状返回带附近','实际关联点在冠层及其邻近；多帧位置切换不能解释为同一叶片的真实运动。'),
(3,'VEGETATION_ASSOCIATED','LOW','9,10,11,12,13','中央树列上缘及邻近空间','初始和多帧关联邻近冠层，但部分点离开密集表面；仅空间关联，不能认证叶片或单个物体。'),
(4,'DUPLICATE_OR_FRAGMENT','HIGH_FOR_REUSE','9,10,11,12,13','与ID2相同冠层邻近场景','第10–13帧与ID2使用完全相同的实际refined点集；这是关联重复，不是两个独立物体证据。'),
(5,'UNKNOWN','LOW','9 seed;12,13 observed','x330,y-32,z10附近细竖向结构','两次实际关联仅相距约0.44m；背景可能是枝干或杆，不能确定植被。'),
(6,'DUPLICATE_OR_FRAGMENT','HIGH_FOR_REUSE','9 seed and observed','远端x446至471附近高处细竖向结构','第9帧实际关联点等于ID8的真实birth seed；不以之后预测位置判断物体身份。'),
(7,'UNKNOWN','LOW','9 seed;11,12 observed','远端x470至450附近高位稀疏结构','只有两次observed关联，邻近高处竖向细结构；不足区分枝叶、杆或离群点。'),
(8,'UNKNOWN','LOW','9 birth seed','x471,y19,z48附近高处竖向细结构','只有真实birth seed；post-update从未OBSERVED，不能用预测延续认证植被或物体。'),
(9,'UNKNOWN','LOW','9 seed and observed','近端x102,y25,z10附近平坦/低密集场景边缘','仅同一扫描seed和关联；地形/结构/植被身份不清，不能解释为长时运动。'),
(10,'UNKNOWN','LOW','9 birth seed','近端x124,y-24,z10附近细竖向结构','单点seed邻近杆状/枝状返回，之后全预测；无足够物体身份依据。'),
(11,'VEGETATION_ASSOCIATED','MEDIUM','9 seed;13 observed','第13帧中央冠层/枝状表面附近','唯一后续实际关联落在冠层邻近；seed位置不同，不能当成同一片植被的连续物理轨迹。'),
(12,'UNKNOWN','LOW','9 birth seed','x378,y-38,z15附近稀疏与细竖向场景','单点seed；之后无OBSERVED，不能将整条预测线称为植被运动。'),
(13,'UNKNOWN','LOW','9 birth seed','x472,y17,z20附近竖向结构','只有seed，结构性质不清；没有后续观测支持。'),
(14,'DUPLICATE_OR_FRAGMENT','HIGH_FOR_REUSE','9 seed and observed','远端细竖向结构，seed与关联高度不同','第9帧实际关联点等于ID6的真实birth seed；同一扫描跨点/跨高度换配不能解释为真实物体位移。'),
(15,'UNKNOWN','LOW','15 pending birth;19 observed','远端x490,z10附近稀疏/细竖向结构','两次真实支持，中间有缺测；不能确认枝叶、其它结构或离群返回。'),
(16,'UNKNOWN','LOW','47 pending birth;48,49,53 observed','x308至386附近多个稀疏/竖向及冠层邻近区域','实际关联位置跳变大，空间来源混杂；不把大跨度当成一个物体的真实运动，也不强行判植被。'),
(17,'VEGETATION_ASSOCIATED','MEDIUM','47 pending birth','x264,y-41,z11附近圆团状冠层返回','晋升当帧支持点邻近冠层；没有后续OBSERVED。仅空间植被关联，叶片运动未独立确认。')]
records=[]
for tid,cls,confidence,frames,loc,reason in manual:
 records.append(dict(track_id=tid,visual_class=cls,confidence=confidence,representative_frames=frames,scene_location_description=loc,reason=reason,identity_basis='OPERATOR_VISUAL_IDENTITY' if tid==1 else ('FROZEN_POINT_MEMBERSHIP_REUSE_PLUS_RVIZ' if cls=='DUPLICATE_OR_FRAGMENT' else 'ASSISTANT_RVIZ_CONTEXT_REVIEW'),independent_ground_truth=False))
with (O/'track_visual_identity.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)
# Publication figures are descriptive, no cutoff/fit/performance curve.
r=list(csv.DictReader((O/'track_lifecycle_summary.csv').open()));ids=np.array([int(x['track_id']) for x in r]);obs=np.array([int(x['observed_associations']) for x in r]);pred=np.array([int(x['predicted_states']) for x in r]);spans=np.array([int(x['span_in_scans']) for x in r])
plt.rcParams.update({'font.size':10,'pdf.fonttype':42,'axes.spines.top':False,'axes.spines.right':False})
fig,ax=plt.subplots(2,1,figsize=(11,6.6),layout='constrained')
a=ax[0];a.bar(ids,obs,color=['#2383be' if i==1 else '#526b7e' for i in ids],label='OBSERVED-coded states');a.bar(ids,pred,bottom=obs,color='#d5dbe0',label='PREDICTED (including removal endpoints)');a.set_ylabel('Scan states');a.set_xticks(ids);a.legend();a.set_title('Frozen lifecycle evidence: operator identity is interpretation, not independent GT')
a=ax[1];a.bar(ids,spans,color=['#2383be' if i==1 else '#9aa9b3' for i in ids]);a.set_xticks(ids);a.set_xlabel('Historical ID');a.set_ylabel('Span in distinct scans');a.grid(axis='y',alpha=.2);fig.savefig(O/'lifecycle_descriptive.png',dpi=180);fig.savefig(O/'lifecycle_descriptive.pdf');plt.close(fig)
# Compact per-ID RViz proof sheet. Source PNGs remain unmodified.
canvas=Image.new('RGB',(1800,6*330),'white');draw=ImageDraw.Draw(canvas)
for tid in range(18):
 choices=sorted((O/'representative_frames').glob(f'id{tid:02d}_*observed_association.png')) or sorted((O/'representative_frames').glob(f'id{tid:02d}_*pending_birth.png')) or sorted((O/'representative_frames').glob(f'id{tid:02d}_*birth_seed.png'))
 p=choices[0];im=Image.open(p);w,h=im.size;im=im.crop((int(w*.20),int(h*.05),w,int(h*.97)));im.thumbnail((595,300));x=(tid%3)*600;y=(tid//3)*330;canvas.paste(im,(x,y+27));draw.text((x+5,y+5),p.stem,fill='black')
canvas.save(O/'representative_frames/all_ids_contact_sheet.png')
print('Manual classes:',{c:sum(r['visual_class']==c for r in records) for c in sorted(set(r['visual_class'] for r in records))})
