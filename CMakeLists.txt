PROJECT(graph_navigation)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

MESSAGE(STATUS "Compiling Navigation")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror")
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
SET(ROS_BUILD_SHARED_LIBS true)


MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

SET(libs roslib roscpp rosbag glog gflags amrl_shared_lib boost_system lua5.1
    pthread)

ADD_LIBRARY(shared_library
            src/visualization/visualization.cc
            src/vector_map/vector_map.cc)

ADD_SUBDIRECTORY(src/shared)
INCLUDE_DIRECTORIES(src/shared)
INCLUDE_DIRECTORIES(src/config_reader/include)
INCLUDE_DIRECTORIES(src/third_party)
INCLUDE_DIRECTORIES(src)

ROSBUILD_GENMSG()
ROSBUILD_GENSRV()

MESSAGE(STATUS "Includes: ${INCLUDE_DIRECTORIES}")

ROSBUILD_ADD_EXECUTABLE(navigation
                        src/navigation/navigation_main.cc
                        src/navigation/navigation.cc
                        src/navigation/motion_primitives.cc
                        src/navigation/ackermann_motion_primitives.cc
                        src/navigation/constant_curvature_arcs.cc
                        src/navigation/linear_evaluator.cc
                        )
TARGET_LINK_LIBRARIES(navigation shared_library ${libs})


ADD_EXECUTABLE(navigation_tests
    src/navigation/navigation_tests.cc
    src/navigation/motion_primitives.cc
)
TARGET_LINK_LIBRARIES(navigation_tests
    gtest
    gtest_main
    ${libs})

ROSBUILD_ADD_EXECUTABLE(social_nav
                        src/navigation/social_nav.cc
                        src/navigation/social_main.cc
                        src/navigation/motion_primitives.cc
                        src/navigation/ackermann_motion_primitives.cc
                        src/navigation/constant_curvature_arcs.cc
                        src/navigation/linear_evaluator.cc
                        src/navigation/navigation.cc)
TARGET_LINK_LIBRARIES(social_nav shared_library ${libs})

add_dependencies(navigation ${catkin_EXPORTED_TARGETS})
add_dependencies(social_nav ${catkin_EXPORTED_TARGETS})
