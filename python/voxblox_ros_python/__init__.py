import io
import numpy as np
from . import _voxblox_ros_python 

class EsdfMapClientInterface(object):
    def __init__(self, voxel_size, voxels_per_side):
        self._ptr = _voxblox_ros_python.create_esdf_map(voxel_size, voxels_per_side)

    def update(self, layer_msg):
        s = io.BytesIO()
        layer_msg.serialize(s)
        bindata = s.getvalue()
        self._update(bindata)

    def _update(self, msg_bindata):
        _voxblox_ros_python.update_esdf_map(self._ptr, msg_bindata)

    def get_distance(self, pts_, with_grad=False):
        if type(pts_) == np.ndarray:
            pts = pts_.tolist()
        else:
            pts = pts_
        if with_grad:
            dists, grads_ = _voxblox_ros_python.get_batch_dist_and_grad(self._ptr, pts)
            grads = np.array(grads_).reshape(-1, 3)
            return np.array(dists), grads
        else:
            dists = _voxblox_ros_python.get_batch_dist(self._ptr, pts)
            return np.array(dists)
