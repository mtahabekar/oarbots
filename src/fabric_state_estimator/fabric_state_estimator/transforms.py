"""Recorded TF interpolation with no extrapolation or dependency on arrival time."""
import bisect
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


class TransformHistory:
    def __init__(self):
        self.edges = {}

    def add(self, parent, child, time_ns, translation, quaternion, static=False):
        if not parent or not child or parent == child:
            raise ValueError('Invalid TF edge')
        value = (int(time_ns), np.asarray(translation, float).reshape(3), np.asarray(quaternion, float).reshape(4))
        if not all(np.isfinite(v).all() for v in value[1:]) or abs(np.linalg.norm(value[2])-1) > .01:
            raise ValueError('Invalid TF sample')
        if child in self.edges and self.edges[child][0] != parent:
            raise ValueError('TF child changed parent')
        if child not in self.edges:
            self.edges[child] = (parent, static, [])
        old_parent, old_static, samples = self.edges[child]
        if old_static != static:
            raise ValueError('TF edge cannot be both static and dynamic')
        samples.append(value)
        samples.sort(key=lambda entry: entry[0])

    def _edge(self, child, time_ns):
        parent, static, samples = self.edges[child]
        times = [sample[0] for sample in samples]
        if static:
            translation, quaternion = samples[-1][1:]
        else:
            index = bisect.bisect_left(times, time_ns)
            if index < len(times) and times[index] == time_ns:
                translation, quaternion = samples[index][1:]
            elif index == 0 or index == len(times):
                raise ValueError(f'TF {parent} -> {child} does not bracket capture {time_ns}')
            else:
                left, right = samples[index-1], samples[index]
                alpha = (time_ns-left[0])/(right[0]-left[0])
                translation = (1-alpha)*left[1]+alpha*right[1]
                quaternion = Slerp([0., 1.], Rotation.from_quat([left[2], right[2]]))([alpha]).as_quat()[0]
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
        matrix[:3, 3] = translation
        return parent, matrix

    def lookup(self, target, source, time_ns):
        def ancestors(frame):
            result = [frame]
            while frame in self.edges:
                parent = self.edges[frame][0]
                if parent in result:
                    raise ValueError('Cycle in recorded TF tree')
                result.append(parent); frame = parent
            return result
        source_tree, target_tree = ancestors(source), ancestors(target)
        common = next((frame for frame in source_tree if frame in target_tree), None)
        if common is None:
            raise ValueError(f'No TF path {target} <- {source}')
        def to_common(frame):
            matrix = np.eye(4)
            while frame != common:
                parent, edge = self._edge(frame, time_ns)
                matrix = edge @ matrix; frame = parent
            return matrix
        # Edges above the common ancestor are irrelevant and need not bracket
        # this capture (e.g. querying world<-camera with a newer map<-world).
        return np.linalg.inv(to_common(target)) @ to_common(source)
