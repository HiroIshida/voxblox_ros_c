import time
import rospy
import numpy as np
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D

from voxblox_msgs.msg import Layer
from voxblox_ros_python import EsdfMapClientInterface

if __name__=='__main__':
    esdf = EsdfMapClientInterface(0.2, 16)

    def show_slice(height):
        N = 20
        b_min = np.array([-1.0, -3.0])
        b_max = np.array([4.0, 4.0])
        Ls = [np.linspace(l, h, N) for l, h in zip(b_min, b_max)]
        mgrids = np.meshgrid(*Ls)
        pts_2d = np.array(zip(*[mg.flatten() for mg in mgrids]))
        ones = np.ones(N*N) * height
        pts_3d = np.hstack((pts_2d, ones[:, None]))

        dists = np.array(esdf.get_distance(pts_3d))
        F = dists.reshape(N, N)
        plt.contourf(mgrids[0], mgrids[1], F)
        plt.colorbar()
        plt.show()

    def show_cloud():
        N = 20
        fig = plt.figure()
        ax = fig.add_subplot(111 , projection='3d')
        x_range = [-1.0, 2.5]
        y_range = [-2.0, 2.0]
        z_range = [0., 1.5]
        b_min, b_max = zip(*[x_range, y_range, z_range])
        Ls = [np.linspace(l, h, N) for l, h in zip(b_min, b_max)]
        mgrids = np.meshgrid(*Ls)
        pts_3d = np.array(zip(*[mg.flatten() for mg in mgrids]))
        dists = np.array(esdf.get_distance(pts_3d))

        idxes_lower = np.where(dists < 0.0)[0]
        myscat = lambda X: ax.scatter(X[:, 0], X[:, 1], X[:, 2])
        myscat(pts_3d[idxes_lower])
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
        plt.show()

    topic_name = "/voxblox_node/esdf_map_out"
    rospy.init_node('listener', anonymous=True)
    def callback(msg):
        import time
        ts = time.time()
        esdf.update(msg)
        print("time to update {0}".format(time.time() - ts))
    rospy.Subscriber(topic_name, Layer, callback)

    import time
    time.sleep(20)
    show_slice(1.0)
