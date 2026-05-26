cmake_minimum_required(VERSION 3.16)

project(la_mono_vo)

set(CMAKE_CXX_STANDARD 20) # for std::format requires C++ version 20
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(la_mono_vo
        src/main.cpp
        src/utils.cpp
        src/frame.cpp
        src/camera_intrinsics.cpp
        src/pose_estimator.cpp
        src/feature_tracker.cpp
        src/visual_odometry.cpp
        src/opencv_visualizer.cpp
)

target_link_libraries(la_mono_vo
        ${OpenCV_LIBS}
        Sophus::Sophus
)

#add_compile_options(-O0 -g)
#target_compile_options(la_mono_vo PRIVATE -g -O0)

# /usr/local/include/sophus/