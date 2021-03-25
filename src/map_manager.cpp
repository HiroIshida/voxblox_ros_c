#include "map_manager.hpp"

EsdfMapManager::EsdfMapManager(double esdf_voxel_size, int esdf_voxel_per_side) : 
  map_(EsdfLayerSharedPtr(new EsdfLayer(esdf_voxel_size, esdf_voxel_per_side))) {}

void EsdfMapManager::update(unsigned char* serialized_layer_msg_){
  auto serialized_layer_msg = static_cast<uint8_t*>(serialized_layer_msg_);
  voxblox_msgs::Layer deserialized_msg;
  // I don't know yet good way to initialize 
  // when the size of msg is unknown, so take huge byte for now
  ros::serialization::IStream stream(serialized_layer_msg, 1000 * 1000);
  ros::serialization::deserialize(stream, deserialized_msg);
  voxblox::deserializeMsgToLayer<voxblox::EsdfVoxel>(deserialized_msg, map_.getEsdfLayerPtr());
}
