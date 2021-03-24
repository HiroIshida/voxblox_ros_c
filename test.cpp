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

int main() {
  // these parameter should not be hard coded!!!
  double esdf_voxel_size = 0.2;
  int esdf_voxel_per_side = 16;
  auto emm = EsdfMapManager(esdf_voxel_size, esdf_voxel_per_side);

  std::ifstream in("../data/layer12.binmsg");
  std::string contents((std::istreambuf_iterator<char>(in)), 
  std::istreambuf_iterator<char>());

  int n = contents.size() + 1;
  auto buffer = new uint8_t[n];
  for(int i=0; i<n; i++){
    buffer[i] = contents.c_str()[i];
  }

  for(int i=0; i<20; i++){
    voxblox_msgs::Layer msg;
    uint32_t serial_size = ros::serialization::serializationLength(msg);
    std::cout << serial_size << std::endl;
    ser::IStream stream(buffer, serial_size * 100000);
    ser::deserialize(stream, msg);
    emm.update(msg);
    //std::cout << msg << std::endl; 
  }
}
