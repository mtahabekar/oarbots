"""Read-only, spoken readiness check. Never arms or starts the warm-up."""
import collections,json,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from fabric_state_msgs.msg import HandStates
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Bool
rclpy.init(); n=Node('voice_readiness_readonly')
last={}; state={}; bridges={}; window=collections.deque(); start=time.monotonic(); last_say=start

def hands(m):
 stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 for h in m.hands:
  if h.source_cameras:last[h.hand_id]=stamp

def twin(m):
 for s in m.status:
  if s.name.endswith('fabric_twin'):state.update({v.key:v.value for v in s.values},message=s.message)

def bridge(m,r):
 for s in m.status:
  if s.name.endswith('fabric_cbf_bridge'):bridges[r]=s.message

def say(message):
 print(message,flush=True)
 subprocess.run(['spd-say','-r','-10',message],check=False)

n.create_subscription(HandStates,'/fabric/hand_states',hands,qos_profile_sensor_data)
n.create_subscription(DiagnosticArray,'/fabric/twin/status',twin,10)
for r in ('blue','silver'):
 n.create_subscription(DiagnosticArray,f'/oarbot_{r}/diagnostics',lambda m,r=r:bridge(m,r),10)
for k in ('deadman','e_stop'):
 n.create_subscription(Bool,'/tablet_gui/'+k,lambda m,k=k:state.__setitem__(k,m.data),10)
last_report=start
while time.monotonic()-start<600:
 rclpy.spin_once(n,timeout_sec=.05)
 now=time.monotonic(); stamp=n.get_clock().now().nanoseconds/1e9
 fresh=[-.01<=stamp-last.get(h,0)<=.5 for h in ('human_left','human_right')]
 if now-start>5:
  window.append((now,fresh,int(state.get('frame','0')),int(state.get('stale_grip_frames','0'))))
 while window and now-window[0][0]>20:window.popleft()
 pct=[100*sum(w[1][i] for w in window)/max(1,len(window)) for i in (0,1)]
 frames=window[-1][2]-window[0][2] if window else 0
 stale=100*(window[-1][3]-window[0][3])/frames if frames else 100
 if now-last_report>10:
  print(json.dumps({'fresh_percent':pct,'stale_grip_percent':stale,'state':state,'bridges':bridges}),flush=True);last_report=now
 bank='fabric_parameter_bank' in n.get_node_names() if now-last_say>25 or (window and now-window[0][0]>19) else False
 ready=(window and now-window[0][0]>19 and min(pct)>=98 and stale<=3 and not state.get('fault','missing')
        and state.get('commands_enabled')=='False' and state.get('deadman') is False and state.get('e_stop') is False
        and state.get('cbf_status')=='feasible' and all(bridges.get(r)=='not armed' for r in ('blue','silver')) and bank)
 if ready:
  result={'ready':True,'fresh_percent':pct,'stale_grip_percent':stale,'state':state,'bridges':bridges,'parameter_bank_running':bank,'checked_at_unix':time.time()}
  Path(__file__).with_name('readiness_pass.json').write_text(json.dumps(result,indent=2))
  say('The system is ready for the warm-up test. Both gloves are tracking. Both arms are at the grasp pose. The parameter bank is ready. Robot commands are off and both bridges are disarmed. Keep the sheet slightly slack and the deadman released until you ask to start the warm-up.')
  break
 if now-last_say>25:
  if state.get('deadman') is True:message='Please release the deadman while I check readiness.'
  elif not any(fresh):message='Neither glove is tracking right now. Hold both free corners in front of the robots, with both tags flat and facing the overhead camera.'
  elif not fresh[0]:message='The right glove is visible. The left glove is missing. Turn the left glove tag toward the overhead camera and keep it clear of the sheet.'
  elif not fresh[1]:message='The left glove is visible. The right glove is missing. Turn the right glove tag toward the overhead camera and keep it clear of the sheet.'
  else:message='Both gloves are visible now. Keep both hands steady while I check twenty seconds of tracking.'
  say(message);last_say=now
else:
 say('The hardware is running, but stable glove tracking has not passed. Commands remain off. Please tell me when both glove tags face the overhead camera.')
n.destroy_node();rclpy.try_shutdown()
