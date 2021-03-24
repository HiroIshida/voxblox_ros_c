import io
import pickle
import rospy
from voxblox_msgs.msg import Layer, Block

data = {"data_sequence": [], "counter": 0}

def callback(msg):
    print("rec")
    filename = "layer{0}.binmsg".format(data["counter"])

    s = io.BytesIO()
    msg.serialize(s)
    bindata = s.getvalue()
    with open(filename, "w") as f:
        f.write(bindata)
    data["counter"] += 1

topic_name = "/voxblox_node/esdf_map_out"
rospy.init_node('listener', anonymous=True)
rospy.Subscriber(topic_name, Layer, callback)
rospy.spin()

filename = "layer_msg_seq.pickle"
with open(filename, "wb") as f:
    pickle.dump(data["data_sequence"], f)

