import collections,json,time,rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from fabric_state_msgs.msg import HandStates
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Bool
rclpy.init();n=Node('warmup_readiness_readonly');last={};data={};counts=collections.Counter();ages={h:[] for h in ('human_left','human_right')}
def hands(m):
 stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 for h in m.hands:
  if h.source_cameras:last[h.hand_id]=stamp
def diag(m):
 for s in m.status:
  if s.name.endswith('fabric_twin'):data.update({v.key:v.value for v in s.values},message=s.message)
n.create_subscription(HandStates,'/fabric/hand_states',hands,qos_profile_sensor_data)
n.create_subscription(DiagnosticArray,'/fabric/twin/status',diag,10)
for k in ('deadman','e_stop'):
 n.create_subscription(Bool,'/tablet_gui/'+k,lambda m,k=k:data.__setitem__(k,m.data),10)
start=time.monotonic();begin=None;end=start+25
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.02)
 if time.monotonic()-start<5:continue
 if begin is None:begin=dict(data)
 now=n.get_clock().now().nanoseconds/1e9
 counts['samples']+=1
 for h in ages:
  age=now-last.get(h,0)
  ages[h].append(age if age<100 else None)
  if -.01<=age<=.5:counts[h]+=1
frames=int(data.get('frame','0'))-int((begin or {}).get('frame','0'))
stale=int(data.get('stale_grip_frames','0'))-int((begin or {}).get('stale_grip_frames','0'))
result={'observed_seconds':20,'fresh_within_0_5_s_percent':{h:round(100*counts[h]/max(1,counts['samples']),1) for h in ages},'max_hand_age_s':{h:round(max(a for a in v if a is not None),3) if any(a is not None for a in v) else None for h,v in ages.items()},'twin_stale_grip_percent':round(100*stale/frames,2) if frames else None,'twin':data,'parameter_bank_running':'fabric_parameter_bank' in n.get_node_names()}
print(json.dumps(result,indent=2),flush=True)
n.destroy_node();rclpy.try_shutdown()
