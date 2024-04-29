function deg2rad(deg)
  return deg * (math.pi / 180)
end

NavigationParameters = {
  laser_topics = {
    "/scan",
    -- "/velodyne_2dscan",
    "/kinect_laserscan",
  };
  laser_frame = "base_link";
  odom_topic = "/odom";
  localization_topic = "localization";
  image_topic = "/camera/rgb/image_raw/compressed";
  init_topic = "initialpose";
  enable_topic = "autonomy_arbiter/enabled";
  dt = 0.025;
  max_linear_accel = 0.5;
  max_linear_decel = 0.5;
  max_linear_speed = 0.5;
  max_angular_accel = 0.5;
  max_angular_decel = 0.5;
  max_angular_speed = 1.0;
  carrot_dist = 1.5;
  system_latency = 0.24;
  obstacle_margin = 0.15;
  num_options = 31;
  robot_width = 0.44;
  robot_length = 0.5;
  base_link_offset = 0;
  max_free_path_length = 6.0;
  max_clearance = 1.0;
  can_traverse_stairs = false;
  use_map_speed = true;
  target_dist_tolerance = 0.1;
  target_vel_tolerance = 0.1;
  target_angle_tolerance = 0.05;
  local_fov = deg2rad(90);
  use_kinect = true;
  camera_calibration_path = "config/camera_calibration_kinect.yaml";
  model_path = "../preference_learning_models/jit_cost_model_outdoor_6dim.pt";
  evaluator_type = "linear";
  intermediate_goal_dist = 10; 
  max_inflation_radius = 1;
  min_inflation_radius = 0.3;
  local_costmap_resolution = 0.05;
  local_costmap_size = 20;
  global_costmap_resolution = 0.1;
  global_costmap_size_x = 100;
  global_costmap_size_y = 100;
  global_costmap_origin_x = -10;
  global_costmap_origin_y = 0;
  lidar_range_min = 0.1;
  lidar_range_max = 10.0;
  replan_dist = 2;
  object_lifespan = 15;
  inflation_coeff = 8;
  distance_weight = 3;
  recovery_carrot_dist = 0.7;
};

AckermannSampler = {
  max_curvature = 2.5;
  clearance_path_clip_fraction = 0.8;
};
