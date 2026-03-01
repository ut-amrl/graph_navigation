CMAKE_MINIMUM_REQUIRED(VERSION 3.8)
PROJECT(graph_navigation)

MESSAGE(STATUS "Compiling Graph Navigation for ROS2")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

# Ensure installed binaries know where to find project / ROS / torch libs
set(INSTALL_RPATH_DIRS
    "${CMAKE_INSTALL_PREFIX}/lib"
    "${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}/lib")
if(DEFINED ENV{ROS_DISTRO})
  list(APPEND INSTALL_RPATH_DIRS "/opt/ros/$ENV{ROS_DISTRO}/lib")
endif()
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

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

# OpenMP (optional) for parallel sampling
find_package(OpenMP)

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
# Try to use system OpenCV first, prefer version 4.5 to avoid conflicts
FIND_PACKAGE(OpenCV 4.5 QUIET)
if(NOT OpenCV_FOUND)
  MESSAGE(STATUS "OpenCV 4.5 not found, falling back to any available version")
  FIND_PACKAGE(OpenCV REQUIRED)
endif()
MESSAGE(STATUS "Using OpenCV version: ${OpenCV_VERSION}")
FIND_PACKAGE(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

# Optional LibTorch for deep learning evaluator
LIST(APPEND CMAKE_PREFIX_PATH "/opt/libtorch")
FIND_PACKAGE(Torch QUIET)
if(Torch_FOUND)
  MESSAGE(STATUS "LibTorch found: ${TORCH_INCLUDE_DIRS}")
  MESSAGE(STATUS "LibTorch libraries: ${TORCH_LIBRARIES}")
  add_compile_definitions(TORCH_AVAILABLE)
  # Set C++17 standard required for PyTorch
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  # Find PyTorch required dependencies
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(GLOG REQUIRED libglog)
  # Torch forces -D_GLIBCXX_USE_CXX11_ABI=0 via INTERFACE_COMPILE_OPTIONS, but
  # all system ROS2/glog/gflags libs on Ubuntu 24.04 use new (cxx11) ABI.
  # Patch every torch imported target to remove the old-ABI flag.
  foreach(_torch_target torch torch_cpu torch_cuda c10 c10_cuda)
    if(TARGET ${_torch_target})
      get_target_property(_opts ${_torch_target} INTERFACE_COMPILE_OPTIONS)
      if(_opts)
        list(REMOVE_ITEM _opts "-D_GLIBCXX_USE_CXX11_ABI=0")
        set_target_properties(${_torch_target} PROPERTIES
          INTERFACE_COMPILE_OPTIONS "${_opts}")
      endif()
    endif()
  endforeach()
  # Explicitly enforce new ABI for this whole project
  add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=1)
else()
  MESSAGE(WARNING "LibTorch not found - deep learning evaluator disabled")
endif()

if(Torch_FOUND AND EXISTS "${TORCH_INSTALL_PREFIX}/lib")
  list(APPEND INSTALL_RPATH_DIRS "${TORCH_INSTALL_PREFIX}/lib")
endif()
list(REMOVE_DUPLICATES INSTALL_RPATH_DIRS)
set(CMAKE_INSTALL_RPATH "${INSTALL_RPATH_DIRS}" CACHE STRING "Install RPATH" FORCE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE CACHE BOOL "Use link paths in install RPATH" FORCE)
set(CMAKE_SKIP_INSTALL_RPATH FALSE CACHE BOOL "Keep RPATH on install" FORCE)

# Use C++20 for auto parameters, but fallback to C++17 for compatibility
if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "10.0")
  SET(CMAKE_CXX_FLAGS "-std=c++20 -Wall")
else()
  SET(CMAKE_CXX_FLAGS "-std=c++17 -Wall")
endif()

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
  # Add PyTorch libraries in proper linking order
  LIST(APPEND libs ${TORCH_LIBRARIES})
endif()

# Shared library for navigation components
ADD_LIBRARY(shared_library
            src/visualization/visualization.cc
            src/vector_map/vector_map.cc)

ament_target_dependencies(shared_library
  rclcpp
  visualization_msgs
  amrl_msgs
)

target_link_libraries(shared_library
  ${libs}
  amrl_shared_lib
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
            src/navigation/omnidirectional_motion_primitives.cc
            src/navigation/linear_evaluator.cc
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
)

target_link_libraries(navigation_lib
    shared_library
    ${libs}
    ${OpenCV_LIBS}
    amrl_shared_lib
)

if (OpenMP_CXX_FOUND)
  target_link_libraries(navigation_lib OpenMP::OpenMP_CXX)
endif()

target_include_directories(navigation_lib PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/navigation
    ${CMAKE_CURRENT_SOURCE_DIR}/src/config_reader/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/third_party
    ${AMENT_INCLUDE_DIRS}
)

# Main navigation executable
ADD_EXECUTABLE(navigation
               src/navigation/navigation_main.cc)

# Do NOT use ament_target_dependencies here: it places ROS libs AFTER
# navigation_lib's propagated torch deps in the link, where torch's
# -Wl,--as-needed toggle drops them. Instead, list imported targets
# explicitly BEFORE navigation_lib so they appear before the torch section.
target_link_libraries(navigation
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    tf2::tf2
    tf2_ros::tf2_ros
    tf2_geometry_msgs::tf2_geometry_msgs
    image_transport::image_transport
    cv_bridge::cv_bridge
    navigation_lib
    graph_navigation__rosidl_typesupport_cpp
    ${libs}
)



target_include_directories(navigation PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/navigation
    ${CMAKE_CURRENT_SOURCE_DIR}/src/config_reader/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/third_party
    ${AMENT_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# Navigation tests
# navigation_flags_stub.cc provides gflags variables defined in navigation_main.cc
# but used by navigation.cc (compiled into navigation_lib)
ADD_EXECUTABLE(navigation_tests
               src/navigation/navigation_tests.cc
               src/navigation/navigation_flags_stub.cc)

# navigation_lib already contains motion_primitives + all glog/gflags/rclcpp deps
target_link_libraries(navigation_tests
    navigation_lib
    shared_library
    amrl_shared_lib
    GTest::GTest
    GTest::Main
    ${libs}
    ${OpenCV_LIBS}
)

target_include_directories(navigation_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/shared
    ${CMAKE_CURRENT_SOURCE_DIR}/src/navigation
    ${CMAKE_CURRENT_SOURCE_DIR}/src/config_reader/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/third_party
    ${AMENT_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

set_target_properties(navigation PROPERTIES
    INSTALL_RPATH "${INSTALL_RPATH_DIRS}"
    BUILD_RPATH "${INSTALL_RPATH_DIRS}"
)

# --copy-dt-needed-entries: pull in transitive DT_NEEDED deps from shared libs
# --no-as-needed: force all listed .so files into DT_NEEDED
# Both are needed because Torch's cmake toggles --as-needed back on, which drops
# librclcpp.so even though navigation_main.cc.o has undefined rclcpp symbols.
target_link_options(navigation BEFORE PRIVATE
    -Wl,--copy-dt-needed-entries
    -Wl,--no-as-needed
)

set_target_properties(navigation_tests PROPERTIES
    INSTALL_RPATH "${INSTALL_RPATH_DIRS}"
    BUILD_RPATH "${INSTALL_RPATH_DIRS}"
)

target_link_options(navigation_tests BEFORE PRIVATE -Wl,--copy-dt-needed-entries)

# Install targets
install(TARGETS navigation navigation_tests
  DESTINATION lib/${PROJECT_NAME}
)

# Install shared libraries
install(TARGETS shared_library navigation_lib
  DESTINATION lib
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES
  README.md
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
