[![Build Status](https://travis-ci.com/ut-amrl/amrl_shared_lib.svg?branch=master)](https://travis-ci.com/ut-amrl/amrl_shared_lib)

# amrl_shared_lib
Shared library for all AMRL C++ projects

## Build
1. Install dependencies:
   ```
   sudo apt-get install libgtest-dev libgoogle-glog-dev cmake build-essential
   ```
1. Compile
   ```
   make -j`nproc`
   ```

## Usage
1. In your own project, add this as a git submodule to a subdir (`src/shared` in this example):
   ```
   git submodule add git@github.com:umass-amrl/amrl_shared_lib.git src/shared
   ```
   Alternatively, if using HTTPS:
   ```
   git submodule add https://github.com/ut-amrl/amrl_shared_lib.git src/shared
   ```
1. In the main `CMakeLists.txt` for your project, add the directory as a cmake subdir, and add the subdir as a location to search for includes (the order of these commands matters):
   ```
   INCLUDE_DIRECTORIES(src/shared)
   ADD_SUBDIRECTORY(src/shared)
   ```
1. Add `amrl-shared-lib` to the linker step for any executables (`my-program` in this example) that use the shared library:
   ```
   TARGET_LINK_LIBRARIES(my-program amrl-shared-lib)
   ```
