"""Read-only RViz overlay of exact frozen birth/observed-support memberships."""
from pathlib import Path
import sys,json,csv
import numpy as np
R=Path(__file__).resolve().parents[2];O=Path(__file__).resolve().parent
sys.path.insert(0,str(R/'tools'))
from historical_pcd_view_data import FrameStore,CACHE
from historical_pcd_rviz_player import Player,cloud_message,marker,point,FRAME
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker,MarkerArray
from sensor_msgs.msg import PointCloud2
class Review(Player):
 def __init__(self):
  Path('/tmp/fmcwforensics').mkdir(parents=True,exist_ok=True)
  super().__init__(FrameStore(prepare=False),start=9)
  q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
  self.support=self.create_publisher(PointCloud2,'/historical_forensics/observed_support',q)
  self.annotation=self.create_publisher(MarkerArray,'/historical_forensics/annotation',q)
  self.create_subscription(String,'/historical_forensics/review',self.review,10)
  self.members=json.loads((O/'support_membership.json').read_text())
 def publish_frame(self):
  super().publish_frame()
  if hasattr(self,'support'):
   self.support.publish(cloud_message(np.empty((0,5),dtype=np.float32),self.get_clock().now().to_msg()))
   self.annotation.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
 def review(self,msg):
  c=json.loads(msg.data);tid=int(c['track_id']);frame=int(c['frame']);kind=c['kind'];key=f'{tid}:{frame}:{kind}'
  if key not in self.members:raise ValueError('Not an actual frozen birth/observed point set: '+key)
  self.index=frame-1;self.playing=False;self.publish_frame()
  data=self.store.frame(self.index)['stages']['refined_dynamic'][self.members[key]]
  stamp=self.get_clock().now().to_msg();self.support.publish(cloud_message(data,stamp))
  center=np.array([data[:,j].mean() for j in range(3)])
  m=marker('review',0,Marker.TEXT_VIEW_FACING,stamp);m.pose.position=point(center+[0,0,3]);m.scale.z=1.1;m.color.r=1.;m.color.g=.9;m.color.b=.25
  m.text=f'ID{tid} / frame{frame}\n'+{'birth_seed':'BIRTH SEED only (1 scan)','pending_birth':'PENDING BIRTH / OBSERVED','observed_association':'OBSERVED association'}[kind]+'\nMAGENTA = exact source-selected points'
  self.annotation.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL),m]))
  out=dict(c,point_count=len(data),center=center.astype(float).tolist());Path('/tmp/fmcwforensics/review_state.json').write_text(json.dumps(out,indent=2)+'\n')
if __name__=='__main__':
 rclpy.init();n=Review()
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 finally:n.destroy_node();rclpy.shutdown()
