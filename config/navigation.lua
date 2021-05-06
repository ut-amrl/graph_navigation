NavigationParameters = {
  imu_topic = "vectornav/IMU";
  laser_topic = "scan";
  odom_topic = "odom";
  localization_topic = "localization";
  init_topic = "initialpose";
  enable_topic = "autonomy_enabler";
  laser_loc = {
    x = 0.2;
    y = 0;
  };

  neural_lower = 0.0; -- 0.1
  neural_upper = 2.0;
  -- max_curvature = 1.35;

  dt = 0.025; --0.025
  max_linear_accel = 10; --5
  max_linear_decel = 10; --5
  max_linear_speed = 2.5; --2.8
  max_angular_accel = 20; --10
  max_angular_decel = 20; --10
  max_angular_speed = 4; --2
  carrot_dist = 1.5;
  system_latency = 0.1;
  obstacle_margin = 0.05; --0.149
  num_options = 161;
  robot_width = 0.281;
  robot_length = 0.535;
  base_link_offset = -0.162;
  max_free_path_length = 6.0; --6.0
  max_clearance = 0.5;--1.0
  can_traverse_stairs = false;
};
