#include <memory>
//#include "voxblox_ros/esdf_server.h"
#include <voxblox/core/layer.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox_msgs/Layer.h>
#include <voxblox_msgs/Block.h>

using EsdfLayer = voxblox::Layer<voxblox::EsdfVoxel>;
using EsdfLayerSharedPtr = std::shared_ptr<EsdfLayer>;

class EsdfMapManager
{
  public:
    EsdfMapManager(double esdf_voxel_size, int esdf_voxel_per_side) : 
      layer_(EsdfLayerSharedPtr(new EsdfLayer(esdf_voxel_size, esdf_voxel_per_side))),
      map_(layer_) {}

    EsdfLayerSharedPtr layer_;
    voxblox::EsdfMap map_;
};

int main() {
  // these parameter should not be hard coded!!!
  double esdf_voxel_size = 0.2;
  int esdf_voxel_per_side = 16;
  auto emm = EsdfMapManager(esdf_voxel_size, esdf_voxel_per_side);
}
