PROJECT(graph_navigation)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

MESSAGE(STATUS "Compiling Navigation")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

SET(CMAKE_CXX_FLAGS "-std=c++14 -Wall -Werror")
SET(LIBRARY_NAME "graph_nav_lib"
    CACHE STRING "Name of compiled library")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O2 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
ENDIF()


INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()

SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS true)

# LIST(APPEND CMAKE_PREFIX_PATH "/opt/libtorch")
# FIND_PACKAGE(Torch REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")
MESSAGE(STATUS "LibTorch location: ${TORCH_INCLUDE_DIRS}")
MESSAGE(STATUS "LibTorch libraries: ${TORCH_LIBRARIES}")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

SET(libs roslib roscpp rosbag glog gflags amrl_shared_lib boost_system lua5.1
    pthread)

ADD_SUBDIRECTORY(src)
INCLUDE_DIRECTORIES(src/shared)
INCLUDE_DIRECTORIES(src/config_reader/include)
INCLUDE_DIRECTORIES(src/third_party)
INCLUDE_DIRECTORIES(src)

ROSBUILD_GENSRV()

MESSAGE(STATUS "Includes: ${INCLUDE_DIRECTORIES}")

ROSBUILD_ADD_EXECUTABLE(navigation
                        src/navigation/navigation_main.cc
                        src/visualization/visualization.cc)
TARGET_LINK_LIBRARIES(navigation ut_amrl_navigation ${libs})

# ROSBUILD_ADD_EXECUTABLE(social_nav
#                         src/navigation/social_main.cc
#                         src/visualization/visualization.cc)
# TARGET_LINK_LIBRARIES(social_nav ut_amrl_social_navigation)

ADD_DEPENDENCIES(navigation ${catkin_EXPORTED_TARGETS})
# ADD_DEPENDENCIES(social_nav ${catkin_EXPORTED_TARGETS})

# ROSBUILD_ADD_LIBRARY(graph_nav_rl
#                      src/navigation/graph_nav_rl.cc
#                      src/visualization/visualization.cc
#                      src/simulator/simulator.cc)
# TARGET_LINK_LIBRARIES(graph_nav_rl ut_amrl_navigation)
