import _voxblox_ros_python as vrp

filename = "../data/layer0.binmsg"
with open(filename, "r") as f:
    msg_serialized = f.read()

# non-object-oriented style 
ptr = vrp.create_esdf_map(0.2, 16)
vrp.update_esdf_map(ptr, "")
