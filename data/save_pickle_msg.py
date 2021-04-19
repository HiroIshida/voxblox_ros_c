import io
import pickle
import rospy
from voxblox_msgs.msg import Layer, Block

data = {"data_sequence": [], "counter": 0}

def callback(msg):
    print("rec {0}".format(data["counter"]))
    filename_serialized = "layer{0}.sermsg".format(data["counter"])
    s = io.BytesIO()
    msg.serialize(s)
    bindata = s.getvalue()
    with open(filename_serialized, "w") as f:
        f.write(bindata)

    filename_deserialized = "layer{0}.pickle".format(data["counter"])
    with open(filename_deserialized, "wb") as f:
        pickle.dump(msg, f)

    data["counter"] += 1

topic_name = "/voxblox_node/esdf_map_out"
rospy.init_node('listener', anonymous=True)
rospy.Subscriber(topic_name, Layer, callback)
rospy.spin()

