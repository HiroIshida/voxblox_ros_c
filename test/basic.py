import pickle
import time
from voxblox_ros_python import EsdfMapClientInterface
import matplotlib.pyplot as plt
import numpy as np

# non-object-oriented style 
esdf_map = EsdfMapClientInterface(0.2, 16)

def update_map(msg_idx):
    filename = "../data/layer{0}.binmsg".format(msg_idx)
    with open(filename, "r") as f:
        string = f.read()
        esdf_map._update(string)

if __name__=='__main__':
    try:
        for i in range(39):
                update_map(i)
    except TypeError:
        pass

    print("finish updating")

    N = 20
    b_min = np.array([-1.0, -3.0])
    b_max = np.array([4.0, 4.0])
    Ls = [np.linspace(l, h, N) for l, h in zip(b_min, b_max)]
    mgrids = np.meshgrid(*Ls)
    pts_2d = np.array(zip(*[mg.flatten() for mg in mgrids]))
    ones = np.ones(N*N) * 1.5
    pts_3d = np.hstack((pts_2d, ones[:, None]))

    ts = time.time()
    for i in range(100):
        dists = esdf_map.get_distance(pts_3d)
    print("elapsed iter : {0}".format(time.time() - ts))

    dists, grads = esdf_map.get_distance(pts_3d, with_grad=True)
    print(grads)

    F = dists.reshape(N, N)
    plt.contourf(mgrids[0], mgrids[1], F)
    plt.colorbar()
    plt.show()

