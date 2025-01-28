function deg2rad(deg)
  return deg * (math.pi / 180)
end

OSMPlannerParameters = {
  gps_topic = "/vectornav/GPSHeading";
  gps_goals_topic = "/gps_goals";
  osrm_file = "osrm_texas_cbf_mld/texas-latest.osrm";
  osrm_path_resolution = 10; -- meters between GPS points
}

NavigationParameters = {
  laser_topics = {
    "/scan",
    "/velodyne_2dscan_lowbeam",
    -- "/kinect_laserscan",
  };
  laser_frame = "base_link";
  -- odom_topic = "/jackal_velocity_controller/odom";
  odom_topic = "/odometry/filtered";
  localization_topic = "localization";
  image_topic = "/bev_image"; -- TODO: change back
  init_topic = "initialpose";
  enable_topic = "autonomy_arbiter/enabled";
  dt = 0.060;
  max_linear_accel = 0.5;
  max_linear_decel = 0.5;
  max_linear_speed = 0.75;
  max_angular_accel = 0.5;
  max_angular_decel = 0.5;
  max_angular_speed = 1.0;
  carrot_dist = 12.8;
  system_latency = 0.24;
  obstacle_margin = 0.15;
  num_options = 31;-- 31, 63;
  robot_width = 0.44;
  robot_length = 0.5;
  robot_wheelbase = 0.26;
  base_link_offset = 0.1;
  max_free_path_length = 5.0;
  max_clearance = 1.0; -- was 1.0
  can_traverse_stairs = false;
  use_map_speed = true;
  target_dist_tolerance = 0.1; -- meters
  target_vel_tolerance = 0.1;
  target_angle_tolerance = 0.05;
  local_fov = deg2rad(270);
  use_kinect = true;
  camera_calibration_path = "config/camera_calibration_left_flir.yaml";
  model_path = "../preference_learning_models/jit_cost_model_outdoor_6dim.pt";
  evaluator_type = "terrain2"; -- linear,  cost_map_service
  carrot_planner_type = "geometric"; -- geometric, service
  intermediate_goal_tolerance = 15; -- final goal distance will be half this (meters)
  max_inflation_radius = 1;
  min_inflation_radius = 0.3;
  local_costmap_resolution = 0.01; -- for bev homography
  local_costmap_size = 20;
  global_costmap_resolution = 0.1;
  global_costmap_size_x = 128;
  global_costmap_size_y = 256;
  global_costmap_origin_x = -12.8;
  global_costmap_origin_y = 12.8;
  lidar_range_min = 0.1;
  lidar_range_max = 25.6;
  replan_dist = 2;
  object_lifespan = 15;
  inflation_coeff = 8;
  distance_weight = 3;
  recovery_carrot_dist = 0.7;
};

AckermannSampler = {
  max_curvature = 2.5;
  max_fov = deg2rad(90);
  clearance_path_clip_fraction = 0.8;
};

ImageToBEVParameters = {
  image_topic = "/stereo/left/image_raw/compressed";
  bev_topic = "/bev_image"; -- TODO change back
  bev_image_height = 640;
  bev_image_width = 1280;
  calibration_file = "config/camera_calibration_left_flir.yaml";
};

TerrainEvaluator = {
  patch_size_pixels = 1;
  bev_pixels_per_meter = 100;
  min_cost = 0.0;
  max_cost = 1;
  discount_factor = 0.95;
  -- discount_factor = 0.8; -- ahg demo
  rollout_density = 20;

  model_path = "../terrain_models/arthur_cuda_model.pt";
  -- context_path="../terrain_models/creste_urban_embedding.pt";

  -- model_path = "../terrain_models/model.pt";
  context_path = "../terrain_models/concrete_peb_mulch.pt";

  -- dist_to_goal_weight = -0.2;
  -- dist_to_goal_weight = -0.7;
  -- dist_to_goal_weight = -2.0;
  dist_to_goal_weight = 0.0;

  clearance_weight = -0.2; -- -0.25;
  fpl_weight = -0.4; -- -0.75;
  terrain_weight = 1.0;
}
