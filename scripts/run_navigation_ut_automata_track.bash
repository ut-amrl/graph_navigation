CARROT_DIST=1.5  # def: 2.5
CLEARANCE_WEIGHT=-0.5 #(Clearance weight) type: double default: -0.5
CONTROL_LOOP_PERIOD=0.025 # (Control loop period) type: double default: 0.025000000000000001
DISTANCE_WEIGHT=1  #(Distance weight) type: double default: 1
FREE_PATH_WEIGHT=-1 #(Free path weight) type: double default: -1
MAX_ACC=5 #(Maximum acceleration) type: double default: 1
MAX_ANG_ACC=5 #(Maximum angular acceleration) type: double default: 0.5
MAX_ANG_SPEED=2.0 #(Maximum angular speed) type: double default: 1
MAX_CURVATURE=.9 #Maximum curvature of turning) type: double default: 2
MAX_DECEL=5 #(Maximum deceleration) type: double default: 1
MAX_PLAN_DEVIATION=0.5 #(Maximum premissible deviation from the plan) type: double default: 0.5
MAX_SPEED=2.0 #(Maximum speed) type: double default: 0.5
NO_JOYSTICK="false" # (Disregards autonomy enable mode from joystick) type: bool default: false
NUM_OPTIONS=41 # (Number of options to consider) type: int32 default: 41
OBSTACLE_MARGIN=0.149 # (Margin to leave for obstacle avoidance) type: double default: 0.14999999999999999
SUB_OPT=1.5 # (Max path increase for clearance) type: double default: 1.5
LATENCY=0.1 # (System latency in seconds) type: double default: 0.23999999999999999
DRIVE_TOPIC="/twist_drive" # (ROS topic to publish twist messages to.) type: string default: "navigation/cmd_vel"
INIT_TOPIC="initialpose"
LASER_TOPIC="/scan" 
ENABLE_TOPIC="/autonomy_enabler" # enable_topic (ROS topic that indicates whether autonomy is enabled or not.) type: string default: "autonomy_arbiter/enabled"



../bin/navigation \
 --map="UT_AUTOmata_Track" \
 --maps_dir="/home/amrl_user/amrl_maps" \
 --odom_topic="/odom" \
 --carrot_dist=$CARROT_DIST \
 --cw=$CLEARANCE_WEIGHT \
 --dt=$CONTROL_LOOP_PERIOD \
 --dw=$DISTANCE_WEIGHT \
 --fw=$FREE_PATH_WEIGHT \
 --max_accel=$MAX_ACC \
 --max_ang_accel=$MAX_ANG_ACC \
 --max_ang_speed=$MAX_ANG_SPEED \
 --max_curvature=$MAX_CURVATURE \
 --max_decel=$MAX_DECEL \
 --max_plan_deviation=$MAX_PLAN_DEVIATION \
 --max_speed=$MAX_SPEED \
 --no_joystick=$NO_JOYSTICK \
 --num_options=$NUM_OPTIONS \
 --obstacle_margin=$OBSTACLE_MARGIN \
 --subopt=$SUB_OPT \
 --system_latency=$LATENCY \
 --twist_drive_topic=$DRIVE_TOPIC \
 --init_topic=$INIT_TOPIC \
 --laser_topic=$LASER_TOPIC \
 --enable_topic=$ENABLE_TOPIC \
 --test_avoidance=false \
 --test_obstacle=false \
 --tx="1.0" \
 --ty="-0.0"  


