cmake_minimum_required(VERSION 3.5.0)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(ORB_SLAM3)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
   add_definitions(-DCOMPILEDWITHC17)
   message(STATUS "Using flag -std=c++17.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../cmake_modules)

find_package(OpenCV 4.0 QUIET)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

set(ROS_DEPS
      rclcpp
      ament_cmake
      cv_bridge
      sensor_msgs
      )

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../
${PROJECT_SOURCE_DIR}/../../include
${PROJECT_SOURCE_DIR}/../../include/CameraModels
${PROJECT_SOURCE_DIR}/../../Thirdparty/Sophus
${Pangolin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../lib/libORB_SLAM3.so
-lboost_system
)

# Node for monocular camera
add_executable(Mono
src/ros_mono.cc
)
target_link_libraries(Mono ${LIBS} )
ament_target_dependencies(Mono rclcpp ${ROS_DEPS})

# Node for monocular inrtial camera
add_executable(MonoInertial
src/ros_mono_inertial.cc
)
target_link_libraries(MonoInertial ${LIBS} )
ament_target_dependencies(MonoInertial rclcpp ${ROS_DEPS})
