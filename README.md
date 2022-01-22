# AMRL Graph Navigation

[![Build Status](https://github.com/ut-amrl/graph_navigation/actions/workflows/buildTest.yml/badge.svg)](https://github.com/ut-amrl/graph_navigation/actions)

## System Dependencies

1. [glog](https://github.com/google/glog)
1. [gflags](https://github.com/gflags/gflags)
1. [Lua5.1](http://www.lua.org/)
1. [LibTorch](https://pytorch.org/get-started/locally/): Requires the cxx11 ABI version libtorch. Unpack the libtorch zip
   file to `/opt/libtorch`.
1. *Optional, if using libtorch with an Nvidia GPU*:   
   Install **both** [CUDA](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html) and [CuDNN](https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html) - these are separate installations.

You can install the system dependencies on *buntu using:
```
sudo apt install libgoogle-glog-dev libgflags-dev liblua5.1-0-dev
```

## ROS Dependencies
1. [ROS](https://www.ros.org/)
1. [AMRL Maps](https://github.com/ut-amrl/amrl_maps)
1. [AMRL ROS Messages](https://github.com/ut-amrl/amrl_msgs)

## Build

1. Add the project directory to `ROS_PACKAGE_PATH`:
    ```
    export ROS_PACKAGE_PATH=MYDIRECTORY:$ROS_PACKAGE_PATH
    ```
    (Replace `MYDIRECTORY` with the actual directory)
    You can also add this to your `~/.bashrc` file so that you don't have to do
    this every time you open a new terminal.
1. Build the program:
    ```
    make
    ```
    Optionally, to compile on all cores (make sure you have sufficient RAM!)
    ```
    make -j
    ```
1. Do **not** run `cmake`, `catkin_make`, `rosbuild`.

## Run

Run `./bin/navigation`   
Some helpful flags:
* `_image_transport:=compressed` to tell it to subscribe to the compressed
  stream of the image transport (useful when playing back a bag file that only
  included the compressed stream).
* `--debug_images` to show debug visualization images.
