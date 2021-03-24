cmake_minimum_required(VERSION 3.0.2)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED 
    COMPONENTS 
        roscpp
        voxblox
        voxblox_ros
        voxblox_msgs
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(voxblox_ros_c SHARED src/c_api.cpp)

target_link_libraries(voxblox_ros_c
    ${catkin_LIBRARIES}
)

add_executable(example_basic example/basic.c)
target_link_libraries(example_basic voxblox_ros_c)
