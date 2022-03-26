# Compile ut_amrl_navigation library
# Requirements:
#  - CMake 2.6 or higher
#  - LibTorch found in the main CMakeLists.txt, e.g. FIND_PACKAGE(Torch REQUIRED)
#  - OpenCV found in the main CMakeLists.txt, e.g. FIND_PACKAGE(OpenCV REQUIRED)

ADD_SUBDIRECTORY(shared)
INCLUDE_DIRECTORIES(shared)
INCLUDE_DIRECTORIES(config_reader/include)
INCLUDE_DIRECTORIES(third_party)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

ADD_LIBRARY(ut_amrl_navigation
            navigation/ackermann_motion_primitives.cc
            navigation/constant_curvature_arcs.cc
            navigation/deep_cost_map_evaluator.cc
            navigation/image_based_evaluator.cc
            navigation/image_tiler.cc
            navigation/linear_evaluator.cc
            navigation/motion_primitives.cc
            navigation/navigation.cc
            vector_map/vector_map.cc
)
TARGET_LINK_LIBRARIES(ut_amrl_navigation
                      amrl_shared_lib
                      glog
                      gflags
                      lua5.1
                      ${OpenCV_LIBS}
                      pthread
                      ${TORCH_LIBRARIES}
)

ADD_LIBRARY(ut_amrl_social_navigation navigation/social_nav.cc)
TARGET_LINK_LIBRARIES(ut_amrl_social_navigation ut_amrl_navigation)

ADD_EXECUTABLE(navigation_tests
    navigation/navigation_tests.cc
    navigation/motion_primitives.cc
)
TARGET_LINK_LIBRARIES(navigation_tests
    gtest
    gtest_main
    glog
    gflags
    pthread)
