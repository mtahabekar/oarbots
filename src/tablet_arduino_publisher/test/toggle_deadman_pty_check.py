"""Integration test of tablet_arduino_publisher deadman_mode:=toggle against a fake Arduino on a pty."""
import os, subprocess, sys, threading, time
import rclpy
from std_msgs.msg import Bool
master, slave = os.openpty(); port = os.ttyname(slave)
exe = sys.argv[1]
node_proc = subprocess.Popen([exe, '--ros-args', '-p', f'com_port:={port}', '-p', 'arduino_deadman_switch_topic:=/t/deadman',
                              '-p', 'arduino_e_stop_topic:=/t/e_stop', '-p', 'deadman_mode:=toggle'],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
rclpy.init(); n = rclpy.create_node('toggle_test'); last = {'v': None, 't': 0}
n.create_subscription(Bool, '/t/deadman', lambda m: last.update(v=m.data, t=time.monotonic()), 10)
spin = threading.Thread(target=lambda: rclpy.spin(n), daemon=True); spin.start()
def send(ch, sec):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        os.write(master, ch.encode()); time.sleep(0.02)
def expect(v, what):
    time.sleep(0.15); ok = last['v'] == v and time.monotonic() - last['t'] < 0.3
    print(f"{'PASS' if ok else 'FAIL'}  {what}: deadman={last['v']} (want {v})"); return ok
R, P, EP, ER = '1', '0', '2', '3'   # released, pressed, e-stop+pressed, e-stop+released
res = []
time.sleep(1.0)
send(P, 1.0); res.append(expect(False, 'switch held at start-up does not start'))
send(R, 0.5); res.append(expect(False, 'released'))
send(P, 0.4); res.append(expect(True, 'first press starts'))
send(R, 1.0); res.append(expect(True, 'stays on after letting go'))
send(P, 0.02); send(R, 0.5); res.append(expect(True, 'single-sample bounce ignored'))
send(P, 0.4); res.append(expect(False, 'second press stops'))
send(R, 0.04); send(P, 0.04); send(R, 0.5); res.append(expect(False, 'release bounce after the stop press does not restart'))
send(P, 0.3); send(R, 0.5); res.append(expect(True, 'press starts again'))
send(ER, 0.3); res.append(expect(False, 'e-stop turns it off'))
send(R, 0.5); res.append(expect(False, 'stays off after e-stop released'))
send(P, 0.3); send(R, 0.5); res.append(expect(True, 'new press after e-stop starts'))
time.sleep(0.6); res.append(expect(False, 'Arduino silent (stale) turns it off'))
send(P, 0.6); res.append(expect(False, 'switch held through the outage does not restart'))
send(R, 0.3); send(P, 0.3); res.append(expect(True, 'release then press after the outage starts'))
send(P, 0.1); send(R, 0.3); send(P, 0.3); send(R, 0.3); res.append(expect(False, 'stop press'))
print('ALL PASS' if all(res) else 'SOME FAILED')
node_proc.terminate(); out = node_proc.communicate(timeout=5)[0]
print('\n'.join(l for l in out.splitlines() if 'toggle' in l or 'mode' in l)[-800:])
