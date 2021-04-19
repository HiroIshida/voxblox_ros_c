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

class EsdfMapManager
{
  public:
    voxblox::EsdfMap map_;
    EsdfMapManager(double esdf_voxel_size, int esdf_voxel_per_side) : 
      map_(EsdfLayerSharedPtr(new EsdfLayer(esdf_voxel_size, esdf_voxel_per_side))) {}

    void update(unsigned char* serialized_layer_msg){
      voxblox_msgs::Layer deserialized_msg;
      // I don't know yet good way to initialize 
      // when the size of msg is unknown, so take huge byte for now
      ros::serialization::IStream stream(serialized_layer_msg, 1000 * 1000);
      ros::serialization::deserialize(stream, deserialized_msg);
      this->update(deserialized_msg);
    }

    void update(const voxblox_msgs::Layer& layer_msg){
      voxblox::deserializeMsgToLayer<voxblox::EsdfVoxel>(layer_msg, map_.getEsdfLayerPtr());
    }

    void get_dist(double* pt, double* dist){
      Eigen::Vector3d pt_eigen(pt);
      bool success = map_.getDistanceAtPosition(pt_eigen, dist);
    }
};

void* C_create_esdf_map(double esdf_voxel_size, int esdf_voxel_per_side){
  auto mapm = static_cast<void*>(new EsdfMapManager(esdf_voxel_size, esdf_voxel_per_side));
  return mapm;
}

void C_update_esdf_map(void* mapm_, unsigned char* serialized_layer_msg_){
  auto mapm = static_cast<EsdfMapManager*>(mapm_);
  mapm->update(serialized_layer_msg_);
}

void C_get_dist(void* mapm_, double* pt, double* dist){
  auto mapm = static_cast<EsdfMapManager*>(mapm_);
  mapm->get_dist(pt, dist);
}
