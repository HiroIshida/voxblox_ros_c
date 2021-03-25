#include "map_manager.hpp"
#include "c_api.h"

void* C_create_esdf_map(double esdf_voxel_size, int esdf_voxel_per_side){
  auto mapm = static_cast<void*>(new EsdfMapManager(esdf_voxel_size, esdf_voxel_per_side));
  return mapm;
}

void C_update_esdf_map(void* mapm_, unsigned char* serialized_layer_msg_){
  auto mapm = static_cast<EsdfMapManager*>(mapm_);
  mapm->update(serialized_layer_msg_);
}
