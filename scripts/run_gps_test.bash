#!/bin/bash
# Define the GPS coordinates
lat1=30.288698
lon1=-97.74137
lat2=30.287638
lon2=-97.741493

# Publish the GPS coordinates using Float64MultiArray
rostopic pub -1 /gps_goals std_msgs/Float64MultiArray "data: [$lat1, $lon1, $lat2, $lon2]"