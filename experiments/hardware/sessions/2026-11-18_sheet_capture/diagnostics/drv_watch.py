import rclpy, time, sys
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
from kinova_msgs.msg import PoseVelocity, PoseVelocityWithFingers
r = sys.argv[1]
rclpy.init(); n = Node('drvwatch'); st = {'nz': 0, 'n': 0, 'q0': None, 'q': None, 'dead': None, 'mode': None, 'last': None}
def cmd(m):
    st['n'] += 1
    v = (m.twist_linear_x, m.twist_linear_y, m.twist_linear_z)
    if any(abs(x) > 0 for x in v): st['nz'] += 1; st['last'] = v
def js(m):
    if len(m.position) >= 6:
        st['q'] = list(m.position[:6]); st['q0'] = st['q0'] or st['q']
n.create_subscription(PoseVelocity, f'/oarbot_{r}/kinova/j2n6s300_driver/in/cartesian_velocity', cmd, 10)
n.create_subscription(PoseVelocityWithFingers, f'/oarbot_{r}/kinova/j2n6s300_driver/in/cartesian_velocity_with_fingers', cmd, 10)
n.create_subscription(JointState, f'/oarbot_{r}/joint_states', js, 10)
n.create_subscription(Bool, '/tablet_gui/deadman', lambda m: st.update(dead=m.data), 10)
n.create_subscription(String, f'/oarbot_{r}/control_mode', lambda m: st.update(mode=m.data), 10)
end = time.time() + 6
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)
dq = max(abs(a - b) for a, b in zip(st['q'], st['q0'])) if st['q'] else None
print(f"{r}: driver commands {st['n']/6:.0f} Hz, nonzero {st['nz']}, last nonzero {st['last']}, deadman {st['dead']}, mode {st['mode']}, joint change over 6 s {dq}")
