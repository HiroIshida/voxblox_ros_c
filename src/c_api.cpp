#include "map_manager.hpp"
#include "c_api.h"

void* C_create_esdf_map(double esdf_voxel_size, int esdf_voxel_per_side){
  auto mapm = static_cast<void*>(new EsdfMapManager(esdf_voxel_size, esdf_voxel_per_side));
  return mapm;
}

void C_update_esdf_map(void* mapm_, unsigned char* serialized_layer_msg_){
  auto serialized_layer_msg = static_cast<uint8_t*>(serialized_layer_msg_);
  voxblox_msgs::Layer deserialized_msg;
  // I don't know yet good way to initialize 
  // when the size of msg is unknown, so take huge byte for now
  ros::serialization::IStream stream(serialized_layer_msg, 1000 * 1000);
  ros::serialization::deserialize(stream, deserialized_msg);

  auto mapm = static_cast<EsdfMapManager*>(mapm_);
  mapm->update(deserialized_msg);
}
