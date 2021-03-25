import io
import rospy
from voxblox_msgs.msg import Layer, Block
import _voxblox_ros_python as vrp

class EsdfMapManager(object):

    def __init__(self):
        self._ptr = vrp.create_esdf_map(0.2, 16)

    def update(self, msg):
        print("updating...")
        s = io.BytesIO()
        msg.serialize(s)
        bindata = s.getvalue()
        vrp.update_esdf_map(self._ptr, bindata)

if __name__=='__main__':
    mapm = EsdfMapManager()

    topic_name = "/voxblox_node/esdf_map_out"
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber(topic_name, Layer, lambda msg: mapm.update(msg))
    rospy.spin()
