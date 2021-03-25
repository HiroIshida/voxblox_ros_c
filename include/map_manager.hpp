#include <fstream>
#include <memory>
#include "voxblox_ros/conversions.h"
#include <voxblox/core/layer.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox_msgs/Layer.h>
#include <voxblox_msgs/Block.h>

using EsdfLayer = voxblox::Layer<voxblox::EsdfVoxel>;
using EsdfLayerSharedPtr = std::shared_ptr<EsdfLayer>;

class EsdfMapManager
{
  public:
    voxblox::EsdfMap map_;

    EsdfMapManager(double esdf_voxel_size, int esdf_voxel_per_side); 
    void update(unsigned char* serialized_layer_msg_);
};
