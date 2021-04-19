#include <iostream>
#include <fstream>
#include <string>
#include <string>
#include "c_api.h"

void update_esdf_map(void* map, int idx_layer){
  auto idx_string = std::to_string(idx_layer);
  std::string filename = "../data/layer" + idx_string + ".binmsg";
  std::cout << filename << std::endl; 

  std::ifstream in(filename);
  std::string contents((std::istreambuf_iterator<char>(in)), 
  std::istreambuf_iterator<char>());
  int n = contents.size() + 1;
  auto buffer = new uint8_t[n];
  for(int i=0; i<n; i++){
    buffer[i] = contents.c_str()[i];
  }
  C_update_esdf_map(map, buffer);
  in.close();
}

int main() {
  // these parameter should not be hard coded!!!
  void* map = C_create_esdf_map(0.2, 16, 10000.0);

  for(int i=0; i<40; i++){
    update_esdf_map(map, i);
  }

  double pt[3] = {1.0, 0.0, 1.0};
  double dist = -1.0;
  C_get_dist(map, pt, &dist);
  std::cout << dist << std::endl; 

  double pt1[3] = {1.0, 0.0, 3.0001};
  C_get_dist(map, pt1, &dist);
  std::cout << dist << std::endl; 

  int N = 100000;
  {
    clock_t start = clock();
    double grad[3];
    for(int i=0; i<N; i++){
      C_get_dist_and_grad(map, pt, &dist, grad);
    }
    clock_t end = clock();
    std::cout << end - start << std::endl; 
  }
  {
    clock_t start = clock();
    C_debug_get_dist_and_grad(map, pt, 100000);
    clock_t end = clock();
    std::cout << end - start << std::endl; 
  }
}


