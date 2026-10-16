"""Drive one RViz window through actual source-supported states; screenshot only."""
from pathlib import Path
import os,sys,csv,json,time,subprocess,re,ctypes,ctypes.util,yaml
import rclpy
from std_msgs.msg import String
R=Path(__file__).resolve().parents[2];O=Path(__file__).resolve().parent;TMP=Path('/tmp/fmcwforensics');TMP.mkdir(parents=True,exist_ok=True)
X=ctypes.CDLL(ctypes.util.find_library('X11'));T=ctypes.CDLL(ctypes.util.find_library('Xtst'))
X.XOpenDisplay.argtypes=[ctypes.c_char_p];X.XOpenDisplay.restype=ctypes.c_void_p;d=X.XOpenDisplay(None);assert d
X.XSetInputFocus.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int,ctypes.c_ulong];X.XRaiseWindow.argtypes=[ctypes.c_void_p,ctypes.c_ulong];X.XFlush.argtypes=[ctypes.c_void_p]
X.XKeysymToKeycode.argtypes=[ctypes.c_void_p,ctypes.c_ulong];X.XKeysymToKeycode.restype=ctypes.c_uint
T.XTestFakeKeyEvent.argtypes=[ctypes.c_void_p,ctypes.c_uint,ctypes.c_int,ctypes.c_ulong]
def press(sym,down):T.XTestFakeKeyEvent(d,X.XKeysymToKeycode(d,sym),down,0)
def key(sym):press(sym,1);press(sym,0);X.XFlush(d)
def combo(sym):press(0xffe3,1);key(sym);press(0xffe3,0);X.XFlush(d)
def window():
 text=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
 m=re.search(r'^\s*(0x[0-9a-f]+) "[^"\n]* - RViz"',text,re.M);assert m,text[-500:]
 return int(m.group(1),16)
def load(config):
 wid=window();X.XRaiseWindow(d,wid);X.XSetInputFocus(d,wid,1,0);X.XFlush(d);combo(ord('o'));time.sleep(.25);combo(ord('a'))
 for c in str(config):key(ord(c))
 key(0xff0d);time.sleep(2.)
 return wid
rclpy.init();node=rclpy.create_node('historical_forensic_capture');pub=node.create_publisher(String,'/historical_forensics/review',10)
while pub.get_subscription_count()==0:rclpy.spin_once(node,timeout_sec=.1)
base=yaml.safe_load((R/'rviz/historical_pcd_inspection.rviz').read_text())
for disp in base['Visualization Manager']['Displays']:
 if disp['Class']=='rviz_default_plugins/PointCloud2':
  disp['Enabled']=disp['Topic']['Value']=='/historical/stored_filtered'
  if disp['Enabled']:disp.update({'Color Transformer':'FlatColor','Color':'180; 200; 215','Alpha':.8,'Size (Pixels)':3})
 elif disp['Class']=='rviz_default_plugins/MarkerArray':disp['Enabled']=False
base['Visualization Manager']['Displays'].extend([
 {'Class':'rviz_default_plugins/PointCloud2','Name':'Exact frozen OBSERVED / birth support (magenta)','Enabled':True,'Topic':{'Value':'/historical_forensics/observed_support','Depth':1,'Durability Policy':'Transient Local','History Policy':'Keep Last','Reliability Policy':'Reliable'},'Color Transformer':'FlatColor','Color':'255; 40; 210','Position Transformer':'XYZ','Style':'Points','Size (Pixels)':12,'Size (m)':.15,'Decay Time':0.,'Alpha':1.},
 {'Class':'rviz_default_plugins/MarkerArray','Name':'Actual evidence annotation; no predicted objects','Enabled':True,'Topic':{'Value':'/historical_forensics/annotation','Depth':1,'Durability Policy':'Transient Local','History Policy':'Keep Last','Reliability Policy':'Reliable'}}])
items=list(csv.DictReader((O/'physical_state_inspection_list.csv').open()))
# Every non-ID1 observed association plus each initial seed; ID1 three fixed samples.
items=[r for r in items if int(r['track_id'])!=1 or (r['evidence_type']=='observed_association' and int(r['scan']) in [9,30,60])]
items.sort(key=lambda r:(int(r['track_id']),int(r['scan']),r['evidence_type']))
if os.environ.get('FMCW_CAPTURE_LIMIT'):items=items[:int(os.environ['FMCW_CAPTURE_LIMIT'])]
manifest=[]
for seq,r in enumerate(items):
 tid=int(r['track_id']);frame=int(r['scan']);kind=r['evidence_type'];center=[float(r[k]) for k in 'xyz']
 existing=O/'representative_frames'/f'id{tid:02d}_f{frame:03d}_{kind}.png'
 if existing.exists():
  memberships=json.loads((O/'support_membership.json').read_text())
  manifest.append({'track_id':tid,'frame':frame,'kind':kind,'file':str(existing.relative_to(O)),'requested_camera_focal_point_json':json.dumps(center),'requested_distance':85.,'observed_point_count':len(memberships[f'{tid}:{frame}:{kind}'])})
  print('ALREADY CAPTURED',tid,frame,kind,flush=True);continue
 config=TMP/f'view{seq:03d}.rviz';current=base['Visualization Manager']['Views']['Current']
 current.update({'Distance':85.,'Yaw':.85,'Pitch':.48,'Focal Point':dict(zip(['X','Y','Z'],center))})
 config.write_text(yaml.safe_dump(base,sort_keys=False));load(config)
 command={'track_id':tid,'frame':frame,'kind':kind};pub.publish(String(data=json.dumps(command)))
 deadline=time.monotonic()+5
 while time.monotonic()<deadline:
  rclpy.spin_once(node,timeout_sec=.05)
  try:
   state=json.loads((TMP/'review_state.json').read_text())
   if all(state.get(k)==v for k,v in command.items()):break
  except (FileNotFoundError,json.JSONDecodeError):pass
 else:raise RuntimeError(command)
 time.sleep(.7)
 file=O/'representative_frames'/f'id{tid:02d}_f{frame:03d}_{kind}.png'
 subprocess.run(['import','-window',hex(window()),str(file)],check=True)
 manifest.append({**command,'file':str(file.relative_to(O)),'requested_camera_focal_point_json':json.dumps(center),'requested_distance':85.,'observed_point_count':state['point_count']})
 print('CAPTURE',seq+1,len(items),tid,frame,kind,flush=True)
with (O/'rviz_inspection_manifest.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=list(manifest[0]));w.writeheader();w.writerows(manifest)
node.destroy_node();rclpy.shutdown()
