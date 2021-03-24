#include <stdio.h>
#include "c_api.h"

int main(){
    unsigned char buffer[18];
    FILE* ptr;
    ptr = fopen("../data/layer0.binmsg", "rb");
    fread(buffer,sizeof(buffer),1,ptr);
    void* map = C_create_esdf_map(0.02, 10);
    C_update_esdf_map(ptr, buffer);
}
