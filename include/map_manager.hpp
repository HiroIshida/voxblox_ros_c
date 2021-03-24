#include <fstream>
#include <memory>
#include "voxblox_ros/conversions.h"
#include <voxblox/core/layer.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox_msgs/Layer.h>
#include <voxblox_msgs/Block.h>

using EsdfLayer = voxblox::Layer<voxblox::EsdfVoxel>;
using EsdfLayerSharedPtr = std::shared_ptr<EsdfLayer>;

namespace ser = ros::serialization;

class EsdfMapManager
{
  public:
    voxblox::EsdfMap map_;

    EsdfMapManager(double esdf_voxel_size, int esdf_voxel_per_side) : 
      map_(EsdfLayerSharedPtr(new EsdfLayer(esdf_voxel_size, esdf_voxel_per_side))) {}

    void update(voxblox_msgs::Layer layer_msg){
      voxblox::deserializeMsgToLayer<voxblox::EsdfVoxel>(layer_msg, map_.getEsdfLayerPtr());
    }

};
