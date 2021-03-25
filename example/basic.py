import _voxblox_ros_python as vrp
import matplotlib.pyplot as plt
import numpy as np

# non-object-oriented style 
ptr = vrp.create_esdf_map(0.2, 16)

def update_map(msg_idx):
    filename = "../data/layer{0}.binmsg".format(msg_idx)
    with open(filename, "rb") as f:
        msg_serialized = f.read()
    vrp.update_esdf_map(ptr, msg_serialized)

if __name__=='__main__':
    try:
        for i in range(20):
            update_map(i)
    except TypeError:
        pass

    N = 50
    b_min = np.array([0.0, -3.0])
    b_max = np.array([4.0, 4.0])
    Ls = [np.linspace(l, h, N) for l, h in zip(b_min, b_max)]
    mgrids = np.meshgrid(*Ls)
    pts_2d = np.array(zip(*[mg.flatten() for mg in mgrids]))
    ones = np.ones(N*N)
    pts_3d = np.hstack((pts_2d, ones[:, None]))
    dists = np.array([vrp.get_dist(ptr, pt.tolist()) for pt in pts_3d])
    F = dists.reshape(N, N)
    plt.contourf(mgrids[0], mgrids[1], F)
    plt.colorbar()
    plt.show()
