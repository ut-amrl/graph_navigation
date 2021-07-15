PROJECT(reference_car)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

MESSAGE(STATUS "Compiling Reference Car Localization + Navigation")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

SET(CMAKE_CXX_FLAGS "-std=c++14 -Wall -Werror")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O2 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
ENDIF()


option(C10_USE_GLOG "use glog" ON)

list(APPEND CMAKE_PREFIX_PATH "/home/kavan/Research/libtorch/libtorch")
find_package(Torch REQUIRED)

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FILES}")

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
INCLUDE_DIRECTORIES(${TORCH_INCLUDE_DIRS})

MESSAGE(STATUS "Includes: ${INCLUDE_DIRECTORIES}")

ROSBUILD_ADD_EXECUTABLE(navigation
                        src/navigation/navigation_main.cc
                        src/navigation/navigation.cc
                        src/navigation/motion_primitives.cc
                        src/navigation/ackermann_motion_primitives.cc
                        src/navigation/constant_curvature_arcs.cc
                        src/navigation/linear_evaluator.cc
                        src/navigation/path_preference_evaluator.cc
                        )

MESSAGE(STATUS "Libraries: ${libs} ${TORCH_LIBRARIES}")

TARGET_LINK_LIBRARIES(navigation shared_library ${libs} ${TORCH_LIBRARIES})

set_property(TARGET navigation PROPERTY CXX_STANDARD 14)

# ADD_EXECUTABLE(navigation_tests
#     src/navigation/navigation_tests.cc
#     src/navigation/motion_primitives.cc
# )
# TARGET_LINK_LIBRARIES(navigation_tests
#     gtest
#     gtest_main
#     ${libs})