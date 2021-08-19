#!/bin/sh

if [ $1 -eq 1 ]
then
#From the alternate region to the upper central bench, end at:
COMMAND="{ pose: { position: { x: 113.463, y: 27.468 } } }"
elif [ $1 -eq 2 ]
then
#From the upper central bench (same as above) to the lower right bench:
COMMAND="{ pose: { position: { x: 130.945068359375, y: 8.99323558807373 } } }"
elif [ $1 -eq 3 ]
then
# From the lower right bench to the upper left corner:
COMMAND="{ pose: { position: { x: 78.14508819580078, y: 25.590364456176758 } } }"
elif [ $1 -eq 4 ]
then
# Cone experiment (left vertical region to lower left corner)
COMMAND="{ pose: { position: { x: 78.14552917480469, y: 6.825026035308838 } } }"
else
  echo "Incorrect goal provided"
  exit 1
fi

rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "$COMMAND"
