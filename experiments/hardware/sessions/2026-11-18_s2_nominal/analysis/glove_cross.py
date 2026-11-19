"""Wrist glove measurements against the overhead's, in the wrist camera's frame: along-ray vs across."""
import sys, collections, numpy as np, rosbag2_py, rclpy.time
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer
from rclpy.duration import Duration
bag, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
buf = Buffer(cache_time=Duration(seconds=600)); gl = []
while r.has_next():
    topic, data, _ = r.read_next()
    m = deserialize_message(data, get_message(types[topic]))
    if topic in ('/tf', '/tf_static'):
        for tr in m.transforms:
            (buf.set_transform_static if topic == '/tf_static' else buf.set_transform)(tr, 'bag')
    elif topic.endswith('/fabric/gloves'):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t0 <= st <= t1:
            for h in m.hands:
                gl.append((m.camera_id, h.hand_id, st, m.header.frame_id, m.header.stamp, np.array([h.position.x, h.position.y, h.position.z])))
def pose(frame, stamp):
    tr = buf.lookup_transform('world', frame, rclpy.time.Time.from_msg(stamp)).transform
    q, t = tr.rotation, tr.translation; x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return R, np.array([t.x, t.y, t.z])
W = []
for cam, hand, st, f, s, p in gl:
    try: R, t = pose(f, s)
    except Exception: continue
    W.append((cam, hand, st, R, t, p, R @ p + t))
over = collections.defaultdict(list)
for cam, hand, st, R, t, p, w in W:
    if cam == 'overhead': over[hand].append((st, w))
res = collections.defaultdict(list)
for cam, hand, st, R, t, p, w in W:
    if cam == 'overhead' or not over[hand]: continue
    ts = np.array([o[0] for o in over[hand]]); k = int(np.argmin(abs(ts - st)))
    if abs(ts[k] - st) > 0.07: continue
    ref = over[hand][k][1]
    d_cam = R.T @ (w - ref)                   # wrist - overhead, in the wrist optical frame
    ray = p / np.linalg.norm(p)
    along = d_cam @ ray; across = np.linalg.norm(d_cam - along * ray)
    res[(cam, hand)].append((along, across, np.linalg.norm(p), np.linalg.norm(R.T @ (ref - t)), (w - ref)[2]))
for (cam, hand), v in sorted(res.items()):
    v = np.array(v)
    print(f'{cam:12s} {hand:11s} n={len(v):3d}  wrist range {np.median(v[:,2]):.2f} m vs overhead-implied range {np.median(v[:,3]):.2f} m; '
          f'difference along the ray {100*np.median(v[:,0]):+.1f} cm, across {100*np.median(v[:,1]):.1f} cm; world z {100*np.median(v[:,4]):+.1f} cm')
