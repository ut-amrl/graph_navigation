#!/bin/bash

if [ -z "$VERBOSE_LOGGING" ]; then
    VERBOSE_LOGGING=0
fi

echo VERBOSE_LOGGING=$VERBOSE_LOGGING

VERBOSE_LOGGING=$VERBOSE_LOGGING ./bin/navigation \
	/velodyne_2dscan:=/scan \
	/jackal_velocity_controller/odom:=/odom
