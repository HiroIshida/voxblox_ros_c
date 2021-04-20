### voxblox_ros_c: C and python (and julia at some moment) interface to voxblox's esdf map.
#### Installation 
We assume you already installed [voxblox](https://github.com/ethz-asl/voxblox) via catkin build. Then,
```
pip install -e . 
```

#### Example
The build python module is called `voxblox_ros_python` and can be used as follows:
```python
import numpy as np
import rospy
from voxblox_msgs.msg import Layer
from voxblox_ros_python import EsdfMapClientInterface

voxel_size, voxels_per_side = 0.05, 16
esdf = EsdfMapClientInterface(voxel_size, voxels_per_side)

topic_name = "/voxblox_node/esdf_map_out"
rospy.init_node('listener', anonymous=True)
s = rospy.Subscriber(topic_name, Layer, lambda msg: esdf.update(msg))
pts = np.random.randn(100, 3)
dists = esdf.get_distance(pts)
```
