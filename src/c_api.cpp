#include <fstream>
#include <memory>
#include "voxblox_ros/conversions.h"
#include <voxblox/core/layer.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox_msgs/Layer.h>
#include <voxblox_msgs/Block.h>

#include "c_api.h"

using EsdfLayer = voxblox::Layer<voxblox::EsdfVoxel>;
using EsdfLayerSharedPtr = std::shared_ptr<EsdfLayer>;

void* C_create_esdf_map(double esdf_voxel_size, int esdf_voxel_per_side){
  auto esdf_layer_shared_ptr = EsdfLayerSharedPtr(new EsdfLayer(esdf_voxel_size, esdf_voxel_per_side));
  auto map = new voxblox::EsdfMap(esdf_layer_shared_ptr);
  auto map_ = static_cast<void*>(map);
  return map_;
}

void C_update_esdf_map(void* map_, unsigned char* serialized_layer_msg){
  auto map = static_cast<voxblox::EsdfMap*>(map_);
  voxblox_msgs::Layer deserialized_msg;
  // I don't know yet good way to initialize 
  // when the size of msg is unknown, so take huge byte for now
  ros::serialization::IStream stream(serialized_layer_msg, 100000000);
  ros::serialization::deserialize(stream, deserialized_msg);
  voxblox::deserializeMsgToLayer<voxblox::EsdfVoxel>(deserialized_msg, map->getEsdfLayerPtr());
}

void C_get_dist(void* map_, double* pt, double* dist){
  auto map = static_cast<voxblox::EsdfMap*>(map_);
  Eigen::Vector3d pt_eigen(pt);
  bool success = map->getDistanceAtPosition(pt_eigen, dist);
}

void C_get_dist_and_grad(void* map_, double* pt, double* dist, double* grad){
  auto map = static_cast<voxblox::EsdfMap*>(map_);
  Eigen::Vector3d pt_eigen(pt);
  Eigen::Vector3d grad_eigen;
  bool success = map->getDistanceAndGradientAtPosition(pt_eigen, dist, &grad_eigen);
  for(int i=0; i<3; i++){
    grad[i] = grad_eigen(i);
  }
}
