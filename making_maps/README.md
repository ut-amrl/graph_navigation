# Making Maps

This will teach you how to make maps using Cartographer.

## Installing Cartographer

Follow this [installation guide](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html).

You can verify that your installation is sound by running one of the examples found [here](https://google-cartographer-ros.readthedocs.io/en/latest/demos.html).

## Using Cartographer with UT AUTOmata

Copy the launch and config files found in `launch/`, `configuration_files/`, and `urdf` to your `catkin_ws/install_isolated/share/cartographer_ros/` directory using the following command:

```
cp -r ~/graph_navigation/making_maps/cartographer_files/* ~/catkin_ws/install_isolated/share/cartographer_ros/
```

This will ensure that cartographer has the necessary launch and configuration files it requires to make maps using data from the UT AUTOmata cars.

## Running Cartographer on UT AUTOmata bag files

From your `catkin_ws`, you can run the following command to run cartograhper on a pre-recorded bagfile:

```
source install_isolated/setup.bash
roslaunch cartographer_ros ut_automata_bagfile.launch bag_filename:=PATH_TO_BAGFILE
```

You can then open `rviz` and listen to the following topics to visualize the map being constructed:

```
/map
/trajectory_node_list
/landmark_poses_list
/constraint_list
```

You can donwload this [bag file](https://drive.google.com/file/d/1rQ6dcl-OQ9iRmeBB_gCmD-3s1sw6XVr9/view?usp=sharing) for testing.

## Actually Making Maps

Clone the [srabiee/rviz_vectormap_creator](https://github.com/srabiee/rviz_vectormap_creator) repository and follow the instructions for building it. 

Before running the program, make sure to run the following command, which is necessary for cartographer to work with the `\odom` topic:

```
rosparam set use_sim_time False
```

You can then run the program:

```
./bin/manual_map_creator
```

## Map-Making Controls

Be sure to listen to the `vector_map` topic in `rviz` to better interact with the drawn line segments.
You can use the following controls to trace maps straight from `rviz`:
* `p` - Selects `2D Pose Estimate` button. Choose a place on the map you want to be the starting point of a line segment and left-click the mouse. Select `p` again and click on the map the second point of the line segment. A line segment should now appear on the map. 
* `q` - Removes the latest drawn point on the map by clicking anywhere.

You can then save the map to a file by running :
```
rostopic pub /vector_map/save_to_file std_msgs/String "data: 'OUTPUT/PATH/TO/MAP'"
```