#include "map_manager.hpp"

EsdfMapManager::EsdfMapManager(double esdf_voxel_size, int esdf_voxel_per_side) : 
  map_(EsdfLayerSharedPtr(new EsdfLayer(esdf_voxel_size, esdf_voxel_per_side))) {}

void EsdfMapManager::update(unsigned char* serialized_layer_msg){
  voxblox_msgs::Layer deserialized_msg;
  // I don't know yet good way to initialize 
  // when the size of msg is unknown, so take huge byte for now
  ros::serialization::IStream stream(serialized_layer_msg, 1000 * 1000);
  ros::serialization::deserialize(stream, deserialized_msg);
  this->update(deserialized_msg);
}

void EsdfMapManager::update(const voxblox_msgs::Layer& layer_msg){
  voxblox::deserializeMsgToLayer<voxblox::EsdfVoxel>(layer_msg, map_.getEsdfLayerPtr());
}

void EsdfMapManager::get_dist(double* pt, double* dist){
  Eigen::Vector3d pt_eigen;
  pt_eigen << pt[0], pt[1], pt[2];
  bool success = map_.getDistanceAtPosition(pt_eigen, dist);
}
