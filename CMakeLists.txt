cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS_INIT} -Wall -fPIC")

#add_compile_options(-std=c++14)

find_package(catkin REQUIRED 
    COMPONENTS 
        roscpp
        voxblox
        voxblox_ros
        voxblox_msgs
    )
include_directories(src/ ${catkin_INCLUDE_DIRS})

add_library(voxblox_ros_c STATIC src/c_api.cpp)
target_link_libraries(voxblox_ros_c ${catkin_LIBRARIES})

add_library(_voxblox_ros_python MODULE src/python_wrap.c)
SET_TARGET_PROPERTIES(_voxblox_ros_python PROPERTIES PREFIX "")
target_link_libraries(_voxblox_ros_python voxblox_ros_c)
install(TARGETS _voxblox_ros_python DESTINATION .) # for pip install 

add_executable(simple_test_cpp test/basic.cpp)
target_link_libraries(simple_test_cpp voxblox_ros_c)
