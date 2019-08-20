cmake_minimum_required(VERSION 2.8.3)
project(stereo_mapper)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -DEIGEN_DONT_PARALLELIZE")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")



set(CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    #    -std=c++11 -O3 -gencode arch=compute_30,code=sm_30  # T440s
    #    -std=c++11 -O3 -gencode arch=compute_53,code=sm_53  # tx1
    # -std=c++11 -O3 -gencode arch=compute_61,code=sm_61  # gtx 980 Ti / 1080
      -gencode arch=compute_52,code=sm_52  # gtx 970M
    )

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    std_msgs
    sensor_msgs
    nav_msgs
    message_filters
    tf
    dynamic_reconfigure
    )

generate_dynamic_reconfigure_options(
    cfg/debug.cfg
    )

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


set(OpenCV_DIR "/usr/local/opencv2.4/share/OpenCV")
find_package(OpenCV 2.4.13 REQUIRED)
            

find_package(CUDA REQUIRED
            # PATHS /usr/local/cuda
            )

# catkin_package()
message("CUDA DIRECTORY: ${CUDA_INCLUDE_DIR}")



# include_directories(
#     include 
#     # ${catkin_INCLUDE_DIRS} 
# )



cuda_add_executable(stereo_mapper
    src/stereo_mapper_node.cpp
    src/calc_cost.cu
    src/stereo_mapper.cpp
    src/parameters.cpp
    )

add_dependencies(stereo_mapper ${PROJECT_NAME}_gencfg)

target_link_libraries(stereo_mapper ${OpenCV_LIBS}  ${catkin_LIBRARIES} tbb glog)

catkin_add_gtest(gpu_test src/gpu_test.cpp)
target_link_libraries(gpu_test  ${OpenCV_LIBS} ${catkin_LIBRARIES}  tbb glog)

