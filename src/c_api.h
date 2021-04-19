#ifdef __cplusplus
extern "C"{
#endif
  void* C_create_esdf_map(double esdf_voxel_size, int esdf_voxel_per_side);
  void C_update_esdf_map(void* mapm_, unsigned char* serialized_layer_msg_);
  void C_get_dist(void* mapm_, double* pt, double* dist);
  void C_get_dist_and_grad(void* mapm_, double* pt, double* dist, double* grad);
#ifdef __cplusplus
}
#endif
