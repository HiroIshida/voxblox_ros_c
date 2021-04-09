import rospy
from voxblox_msgs.msg import Layer
from voxblox_ros_python import EsdfMapClientInterface

if __name__=='__main__':
    esdf = EsdfMapClientInterface(0.2, 16)
    topic_name = "/voxblox_node/esdf_map_out"
    rospy.init_node('listener', anonymous=True)
    def callback(msg):
        print("updating...")
        esdf.update(msg)
    rospy.Subscriber(topic_name, Layer, callback)
    rospy.spin()
