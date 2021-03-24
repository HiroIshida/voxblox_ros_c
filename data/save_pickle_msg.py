import pickle
import rospy
from voxblox_msgs.msg import Layer, Block

data = {"data_sequence": []}

def callback(msg):
    print("recording")
    data["data_sequence"].append(msg)

topic_name = "/voxblox_node/esdf_map_out"
rospy.init_node('listener', anonymous=True)
rospy.Subscriber(topic_name, Layer, callback)
rospy.spin()

filename = "layer_msg_seq.pickle"
with open(filename, "wb") as f:
    pickle.dump(data["data_sequence"], f)

