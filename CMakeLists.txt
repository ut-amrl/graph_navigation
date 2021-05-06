PROJECT(reference_car)
cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

MESSAGE(STATUS "Compiling Reference Car Localization + Navigation")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")




include_directories("/home/amrl_user/.local/lib/python3.6/site-packages/torch/include")

link_directories("/home/amrl_user/.local/lib")
link_directories("/home/amrl_user/.local/lib/python3.6/site-packages/torch/lib")

set(CMAKE_PREFIX_PATH "/home/amrl_user/.local/lib/python3.6/site-packages/torch")
set(Torch_DIR "/home/amrl_user/.local/lib/python3.6/site-packages/torch")
find_package(Torch REQUIRED)




SET(CMAKE_CXX_FLAGS "-std=c++14 -Wall -Werror")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O2 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
ENDIF()

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

SET(libs roslib roscpp rosbag glog gflags amrl_shared_lib boost_system lua5.1
    pthread torch)

ADD_LIBRARY(shared_library
            src/visualization/visualization.cc
            src/vector_map/vector_map.cc)

ADD_SUBDIRECTORY(src/shared)
INCLUDE_DIRECTORIES(src/shared)
INCLUDE_DIRECTORIES(src/config_reader/include)
INCLUDE_DIRECTORIES(src/third_party)
INCLUDE_DIRECTORIES(src)

MESSAGE(STATUS "Includes: ${INCLUDE_DIRECTORIES}")

ROSBUILD_ADD_EXECUTABLE(navigation
                        src/navigation/navigation_main.cc
                        src/navigation/navigation.cc)
TARGET_LINK_LIBRARIES(navigation shared_library ${libs})
