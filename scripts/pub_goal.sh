#!/bin/sh

if [ $1 -eq 1 ]
then
COMMAND="{ pose: { position: { x: 113.463, y: 27.468 } } }"
elif [ $1 -eq 2 ]
then
COMMAND="{ pose: { position: { x: 130.945068359375, y: 8.99323558807373 } } }"
elif [ $1 -eq 3 ]
then
COMMAND="{ pose: { position: { x: 85.40889739990234, y: 26.2644100189209 } } }"
elif [ $1 -eq 4 ]
then
COMMAND="{ pose: { position: { x: 83.24552917480469, y: 6.825026035308838 } } }"
else
  echo "Incorrect goal provided"
  exit 1
fi

rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "$COMMAND"
