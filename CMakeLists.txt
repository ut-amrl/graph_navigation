PROJECT(graph_navigation)
CMAKE_MINIMUM_REQUIRED(VERSION 3.8)

MESSAGE(STATUS "Compiling Graph Navigation for ROS2")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

# ROS2 compilation
MESSAGE(STATUS "Compiling with ROS2")
add_compile_definitions(ROS2)

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

# Convert AMENT_PREFIX_PATH to CMAKE_PREFIX_PATH (standard ROS2 pattern)
if(DEFINED ENV{AMENT_PREFIX_PATH})
  string(REPLACE ":" ";" ament_prefix_path_list "$ENV{AMENT_PREFIX_PATH}")
  foreach(path ${ament_prefix_path_list})
    list(APPEND CMAKE_PREFIX_PATH "${path}")
  endforeach()
endif()

# Find AMRL dependencies
find_package(amrl_msgs REQUIRED)

# Find ROS2 interface generation
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_cmake REQUIRED)

# Find system dependencies
FIND_PACKAGE(Lua51 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)

# Optional LibTorch for deep learning evaluator
LIST(APPEND CMAKE_PREFIX_PATH "/opt/libtorch")
FIND_PACKAGE(Torch QUIET)
if(Torch_FOUND)
  MESSAGE(STATUS "LibTorch found: ${TORCH_INCLUDE_DIRS}")
  MESSAGE(STATUS "LibTorch libraries: ${TORCH_LIBRARIES}")
  add_compile_definitions(TORCH_AVAILABLE)
else()
  MESSAGE(WARNING "LibTorch not found - deep learning evaluator disabled")
endif()

SET(CMAKE_CXX_FLAGS "-std=c++17 -Wall")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O2 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
ENDIF()

ADD_DEFINITIONS(-DROS2)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Add shared library subdirectory
ADD_SUBDIRECTORY(src/shared)

# Generate ROS2 interfaces (services)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GraphNav.srv"
  "srv/SocialNav.srv"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    amrl_msgs
)

# System libraries
SET(libs glog gflags boost_system lua5.1 pthread ${LUA_LIBRARIES})

# Optional LibTorch libraries
if(Torch_FOUND)
  LIST(APPEND libs ${TORCH_LIBRARIES})
endif()

# Shared library for navigation components
ADD_LIBRARY(shared_library
            src/visualization/visualization.cc
            src/vector_map/vector_map.cc)

target_link_libraries(shared_library
  ${libs}
  amrl_shared_lib
)

ament_target_dependencies(shared_library
  rclcpp
  visualization_msgs
  amrl_msgs
)

target_include_directories(shared_library PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/shared
    ${OpenCV_INCLUDE_DIRS}
    ${LUA_INCLUDE_DIR}
)

# Navigation library
ADD_LIBRARY(navigation_lib
            src/navigation/navigation.cc
            src/navigation/motion_primitives.cc
            src/navigation/ackermann_motion_primitives.cc
            src/navigation/constant_curvature_arcs.cc
            src/navigation/linear_evaluator.cc
            src/navigation/image_tiler.cc
            src/navigation/image_based_evaluator.cc
            src/navigation/deep_cost_map_evaluator.cc
            )

target_link_libraries(navigation_lib
    shared_library
    ${libs}
    ${OpenCV_LIBS}
    amrl_shared_lib
)
ament_target_dependencies(navigation_lib
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    visualization_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    image_transport
    cv_bridge
    amrl_msgs
    ament_index_cpp
    nav2_costmap_2d
)

target_include_directories(navigation_lib PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/navigation
    ${CMAKE_CURRENT_SOURCE_DIR}/src/config_reader/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/third_party
    ${AMENT_INCLUDE_DIRS}
    ${nav2_costmap_2d_INCLUDE_DIRS}
    ${Torch_INCLUDE_DIRS}
)

# Main navigation executable
ADD_EXECUTABLE(navigation
               src/navigation/navigation_main.cc)

target_link_libraries(navigation
    navigation_lib
    graph_navigation__rosidl_typesupport_cpp
)
ament_target_dependencies(navigation
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    visualization_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    image_transport
    cv_bridge
    amrl_msgs
    ament_index_cpp
    nav2_costmap_2d
)

target_include_directories(navigation PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/navigation
    ${CMAKE_CURRENT_SOURCE_DIR}/src/config_reader/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/third_party
    ${AMENT_INCLUDE_DIRS}
    ${nav2_costmap_2d_INCLUDE_DIRS}
    ${Torch_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# Social navigation executable
ADD_EXECUTABLE(social_nav
               src/navigation/social_main.cc
               src/navigation/social_nav.cc)

target_link_libraries(social_nav
    navigation_lib
    graph_navigation__rosidl_typesupport_cpp
)
ament_target_dependencies(social_nav
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    visualization_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    image_transport
    cv_bridge
    amrl_msgs
    ament_index_cpp
    nav2_costmap_2d
)

target_include_directories(social_nav PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/navigation
    ${CMAKE_CURRENT_SOURCE_DIR}/src/config_reader/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/third_party
    ${AMENT_INCLUDE_DIRS}
    ${nav2_costmap_2d_INCLUDE_DIRS}
    ${Torch_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# Navigation tests
ADD_EXECUTABLE(navigation_tests
               src/navigation/navigation_tests.cc
               src/navigation/motion_primitives.cc)

target_link_libraries(navigation_tests
    gtest
    gtest_main
    ${libs}
    rclcpp::rclcpp
)

target_include_directories(navigation_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/shared
    ${CMAKE_CURRENT_SOURCE_DIR}/src/navigation
    ${CMAKE_CURRENT_SOURCE_DIR}/src/config_reader/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/third_party
    ${AMENT_INCLUDE_DIRS}
    ${nav2_costmap_2d_INCLUDE_DIRS}
    ${Torch_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# Install targets
install(TARGETS navigation social_nav navigation_tests
  RUNTIME DESTINATION bin
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
)

install(FILES
  README.md
  CODE_EXPLANATION.md
  DESTINATION share/${PROJECT_NAME}
)

# Install the generated ROS2 interface libraries
install(
  DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_fastrtps_cpp/"
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so"
)

# Install the generated interface files (messages and services)
install(
  DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

# Export runtime dependencies
ament_export_dependencies(rosidl_default_runtime)

# ROS2 package finalization
ament_package()

add_dependencies(navigation graph_navigation__rosidl_typesupport_cpp)
add_dependencies(social_nav graph_navigation__rosidl_typesupport_cpp)
