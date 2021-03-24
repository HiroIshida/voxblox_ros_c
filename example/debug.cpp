#include <fstream>
#include "map_manager.hpp"
#include "c_api.h"

int main() {
  // these parameter should not be hard coded!!!
  std::ifstream in("../data/layer12.binmsg");
  std::string contents((std::istreambuf_iterator<char>(in)), 
  std::istreambuf_iterator<char>());

  int n = contents.size() + 1;
  auto buffer = new uint8_t[n];
  for(int i=0; i<n; i++){
    buffer[i] = contents.c_str()[i];
  }

  void* map = C_create_esdf_map(0.02, 10);
  C_update_esdf_map(map, buffer);
}

