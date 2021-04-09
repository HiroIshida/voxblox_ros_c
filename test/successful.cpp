#include <fstream>
#include <memory>
#include "map_manager.hpp"

using EsdfLayer = voxblox::Layer<voxblox::EsdfVoxel>;
using EsdfLayerSharedPtr = std::shared_ptr<EsdfLayer>;

namespace ser = ros::serialization;

/*
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
*/

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

  emm.update(buffer);

  /*
  for(int i=0; i<20; i++){
    voxblox_msgs::Layer msg;
    uint32_t serial_size = ros::serialization::serializationLength(msg);
    std::cout << serial_size << std::endl;
    ser::IStream stream(buffer, 1000 * 1000);
    ser::deserialize(stream, msg);
    emm.update(msg);
    //std::cout << msg << std::endl; 
  }
  */
}


/*
#include <fstream>
#include "map_manager.hpp"
#include "c_api.h"

int main() {
  // these parameter should not be hard coded!!!
  std::ifstream in("../data/layer0.binmsg");
  std::string contents((std::istreambuf_iterator<char>(in)), 
  std::istreambuf_iterator<char>());

  int n = contents.size() + 1;
  auto buffer = new uint8_t[n];
  for(int i=0; i<n; i++){
    buffer[i] = contents.c_str()[i];
  }

  auto mapm = EsdfMapManager(0.2, 10);

  voxblox_msgs::Layer msg;
  uint32_t serial_size = ros::serialization::serializationLength(msg);
  ros::serialization::IStream stream(buffer, serial_size * 100000);
  ros::serialization::deserialize(stream, msg);
  mapm.update(msg);
}
*/
