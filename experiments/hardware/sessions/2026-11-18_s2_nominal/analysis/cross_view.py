"""Cross-view check from an S2 bag: each camera's sheet depth points in the world at capture time,
compared with the overhead's sheet surface at the nearest capture. Prints the wrist-minus-overhead
offset (nearest overhead point to each wrist point)."""
import sys, collections, numpy as np, rosbag2_py, rclpy.time
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer
from rclpy.duration import Duration
from scipy.spatial import cKDTree
bag, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
buf = Buffer(cache_time=Duration(seconds=600))
obs = collections.defaultdict(list)
while r.has_next():
    topic, data, _ = r.read_next()
    m = deserialize_message(data, get_message(types[topic]))
    if topic in ('/tf', '/tf_static'):
        for tr in m.transforms:
            (buf.set_transform_static if topic == '/tf_static' else buf.set_transform)(tr, 'bag')
    elif topic.endswith('/fabric/observations'):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t0 <= st <= t1 and len(m.depth) > 50:
            pts = np.array([[d.position.x, d.position.y, d.position.z] for d in m.depth])
            obs[m.camera_id].append((st, m.header.frame_id, m.header.stamp, pts))
def to_world(frame, stamp, pts):
    tr = buf.lookup_transform('world', frame, rclpy.time.Time.from_msg(stamp)).transform
    q, t = tr.rotation, tr.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return pts @ R.T + [t.x, t.y, t.z]
print({k: len(v) for k, v in obs.items()})
over = [(st, to_world(f, s, p)) for st, f, s, p in obs['overhead']]
ot = np.array([o[0] for o in over])
for cam in [c for c in obs if c != 'overhead']:
    offs, dist, zs = [], [], []
    for st, f, s, p in obs[cam]:
        try: W = to_world(f, s, p)
        except Exception as e: continue
        k = int(np.argmin(abs(ot - st)))
        if abs(ot[k] - st) > 0.1: continue
        O = over[k][1]
        dd, ii = cKDTree(O).query(W)
        offs.append(np.median(W - O[ii], axis=0)); dist.append(np.median(dd)); zs.append((np.median(W[:, 2]), np.median(O[:, 2])))
    offs = np.array(offs); zs = np.array(zs)
    print(f'{cam}: {len(offs)} paired captures; median offset to the nearest overhead sheet point (wrist - overhead) '
          f'{(100*np.median(offs, axis=0)).round(1)} cm, median distance {100*np.median(dist):.1f} cm; '
          f'sheet median height wrist {np.median(zs[:,0]):.3f} m vs overhead {np.median(zs[:,1]):.3f} m')
