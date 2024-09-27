#!/bin/bash

if [ -z "$VERBOSE" ]; then
    VERBOSE=0
fi

echo verbose=$VERBOSE

VERBOSE=$VERBOSE ./bin/navigation \
	/velodyne_2dscan:=/scan \
	/jackal_velocity_controller/odom:=/odom
