import io
import numpy as np
from . import _voxblox_ros_python 

class EsdfMapClientInterface(object):
    def __init__(self, voxel_size, voxels_per_side, fill_val=0.0):
        self._ptr = _voxblox_ros_python.create_esdf_map(voxel_size, voxels_per_side, fill_val)

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

    def debug_points(self, b_min, b_max, N=50, threshold=0.0):
        """
        Obtains points on which sdf is smaller than the specified threshold 
        """
        Ls = [np.linspace(l, h, N) for l, h in zip(b_min, b_max)]
        mgrids = np.meshgrid(*Ls)
        pts_3d = np.array(zip(*[mg.flatten() for mg in mgrids]))
        dists = np.array(self.get_distance(pts_3d))
        idxes_lower = np.where(dists < threshold)[0]
        pts_inside = pts_3d[idxes_lower]
        return pts_inside

