cmake_minimum_required(VERSION 3.16)
project(graph_navigation)

# Add the custom cmake folder to CMAKE_MODULE_PATH
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

message(STATUS "Compiling Navigation")
message(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Werror -fmax-errors=1")
set(LIBRARY_NAME "graph_nav_lib" CACHE STRING "Name of compiled library")

if("${CMAKE_BUILD_TYPE}" MATCHES "Release")
  message(STATUS "Additional Flags for Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -DNDEBUG")
elseif("${CMAKE_BUILD_TYPE}" MATCHES "Debug")
  message(STATUS "Additional Flags for Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()

# --------------------------------------------------------------------
# Determine ROS version
# --------------------------------------------------------------------
if(DEFINED ENV{ROS_VERSION})
  if("$ENV{ROS_VERSION}" STREQUAL "1")
    add_definitions(-DROS1)
    set(USE_ROS1 TRUE)
    message(STATUS "Defining ROS1 for compilation")
  elseif("$ENV{ROS_VERSION}" STREQUAL "2")
    add_definitions(-DROS2)
    set(USE_ROS1 FALSE)
    message(STATUS "Defining ROS2 for compilation")
  else()
    message(FATAL_ERROR "Unknown ROS_VERSION: $ENV{ROS_VERSION}")
  endif()
else()
  message(FATAL_ERROR "ROS_VERSION environment variable is not defined")
endif()

# --------------------------------------------------------------------
# ROS1 vs. ROS2 setup
# --------------------------------------------------------------------
if(USE_ROS1)
  # Use rosbuild for ROS1
  if(NOT DEFINED ENV{ROS_ROOT})
    message(FATAL_ERROR "ROS_ROOT is not set; cannot build for ROS1.")
  endif()
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()
  set(ROS_BUILD_STATIC_LIBS true)
  set(ROS_BUILD_SHARED_LIBS false)
else()
  # Use ament_cmake for ROS2
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(amrl_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2 REQUIRED)
  SET(CMAKE_INCLUDE_CURRENT_DIR ON)

  # For generating messages and services
  find_package(rosidl_default_generators REQUIRED)
  message(STATUS "Building for ROS2 using ament_cmake")
endif()

# --------------------------------------------------------------------
# Common Third-Party Dependencies
# --------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(${LibOSRM_LIBRARY_DIRS})
list(APPEND CMAKE_PREFIX_PATH "/opt/libtorch")
find_package(Torch REQUIRED)
find_package(OpenCV REQUIRED)
find_package(LibOSRM REQUIRED)
find_package(foxglove_msgs REQUIRED)

if(USE_ROS1)
  if(NOT ROS_BUILD_TYPE)
    set(ROS_BUILD_TYPE Release)
  endif()
  message(STATUS "ROS-Override Build Type: ${CMAKE_BUILD_TYPE}")
  message(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")
  message(STATUS "LibTorch location: ${TORCH_INCLUDE_DIRS}")
  message(STATUS "LibTorch libraries: ${TORCH_LIBRARIES}")
  message(STATUS "OSRM location: ${LibOSRM_INCLUDE_DIRS}")
  message(STATUS "OSRM libraries: ${LibOSRM_LIBRARIES}")

  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  # These are the libraries needed for ROS1.
  set(libs roslib roscpp rosbag glog gflags amrl_shared_lib boost_system lua5.1
      pthread costmap_2d)
else()
  # For ROS2, we rely on ament and rclcpp. (Some of the old libs may not be needed.)
  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
  set(libs glog gflags amrl_shared_lib boost_system lua5.1 pthread)
endif()

# --------------------------------------------------------------------
# Build shared library used by executables
# --------------------------------------------------------------------
add_library(shared_library
  src/visualization/visualization.cc
  src/vector_map/vector_map.cc
)
if(NOT USE_ROS1)
  target_compile_definitions(shared_library PRIVATE ROS2)
  target_link_libraries(shared_library PUBLIC rclcpp::rclcpp)
endif()

add_subdirectory(src/shared)
include_directories(src/shared)
include_directories(src/config_reader/include)
include_directories(src/third_party)
include_directories(src)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${LibOSRM_INCLUDE_DIRS})
include_directories(${amrl_msgs_INCLUDE_DIRS})
include_directories(${foxglove_msgs_INCLUDE_DIRS})

# --------------------------------------------------------------------
# Message and Service Generation
# --------------------------------------------------------------------
if(USE_ROS1)
  rosbuild_genmsg()
  rosbuild_gensrv()
else()
  # Custom custom message and service generation for ROS2
  rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/GraphNavSrv.srv"
    DEPENDENCIES std_msgs sensor_msgs geometry_msgs
  )
endif()

message(STATUS "Includes: ${INCLUDE_DIRECTORIES}")

# --------------------------------------------------------------------
# Add Executable Targets
# --------------------------------------------------------------------
if(USE_ROS1)
  rosbuild_add_executable(navigation
    src/navigation/navigation_main.cc
    src/navigation/navigation.cc
    src/navigation/motion_primitives.cc
    src/navigation/ackermann_motion_primitives.cc
    src/navigation/constant_curvature_arcs.cc
    src/navigation/linear_evaluator.cc
    src/navigation/image_tiler.cc
    src/navigation/deep_cost_map_evaluator.cc
    src/navigation/deep_cost_map_evaluator_service.cc
    src/navigation/image_based_evaluator.cc
    src/navigation/carrot_service.cc
    src/navigation/terrain_evaluator.cc
    src/navigation/ros1_adapter.cc
  )
  target_link_libraries(navigation 
    shared_library 
    ${libs} 
    ${TORCH_LIBRARIES} 
    ${OpenCV_LIBS} 
    ${LibOSRM_LIBRARIES} 
    ${LibOSRM_DEPENDENT_LIBRARIES}
  )

  rosbuild_add_executable(image_to_bev_node
    src/navigation/bev_image_node_main.cc
  )
  target_link_libraries(image_to_bev_node
    shared_library
    ${libs}
    ${OpenCV_LIBS}
  )
  add_dependencies(navigation ${catkin_EXPORTED_TARGETS})
else()
  add_executable(navigation
    src/navigation/navigation_main.cc
    src/navigation/navigation.cc
    src/navigation/motion_primitives.cc
    src/navigation/ackermann_motion_primitives.cc
    src/navigation/constant_curvature_arcs.cc
    src/navigation/linear_evaluator.cc
    src/navigation/image_tiler.cc
    src/navigation/deep_cost_map_evaluator.cc
    src/navigation/deep_cost_map_evaluator_service.cc
    src/navigation/image_based_evaluator.cc
    src/navigation/carrot_service.cc
    src/navigation/terrain_evaluator.cc
    src/navigation/ros2_adapter.cc
  )
  target_link_libraries(navigation
    shared_library
    # rclcpp
    ${TORCH_LIBRARIES}
    ${OpenCV_LIBS}
    ${LibOSRM_LIBRARIES}
    ${LibOSRM_DEPENDENT_LIBRARIES}
    ${libs}
  )
  ament_target_dependencies(navigation
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    amrl_msgs
    foxglove_msgs
    cv_bridge
    tf2
    tf2_ros
  )

  # add_executable(image_to_bev_node
  #   src/navigation/bev_image_node_main.cc
  # )
  # target_link_libraries(image_to_bev_node
  #   shared_library
  #   rclcpp
  #   ${OpenCV_LIBS}
  # )
endif()

# --------------------------------------------------------------------
# Install Targets
# --------------------------------------------------------------------
install(TARGETS navigation shared_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY src/config_reader DESTINATION share/${PROJECT_NAME})

if(NOT USE_ROS1)
  ament_package()
endif()
