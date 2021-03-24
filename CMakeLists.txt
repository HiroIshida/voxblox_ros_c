cmake_minimum_required(VERSION 3.0.2)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED 
    COMPONENTS 
        roscpp
        voxblox
        voxblox_ros
        voxblox_msgs
    )
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(voxblox_ros_c SHARED src/c_api.cpp)
target_link_libraries(voxblox_ros_c ${catkin_LIBRARIES})

find_package(PythonExtensions REQUIRED)
add_library(_voxblox_ros_python MODULE src/python_wrap.c)
target_link_libraries(_voxblox_ros_python voxblox_ros_c)
install(TARGETS _voxblox_ros_python LIBRARY DESTINATION python/voxblox_ros_python) # for pip install 

add_executable(example_basic example/basic.c)
target_link_libraries(example_basic voxblox_ros_c)
