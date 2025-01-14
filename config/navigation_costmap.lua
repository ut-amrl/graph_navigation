function deg2rad(deg)
  return deg * (math.pi / 180)
end

OSMPlannerParameters = {
  gps_topic = "/vectornav/GPSHeading";
  gps_goals_topic = "/gps_goals";
  osrm_file = "osrm_texas_cbf_mld/texas-latest.osrm";
  osrm_path_resolution = 8; -- meters between GPS points
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
  image_topic = "/stereo/left/image_raw/compressed";
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
  num_options = 41;-- 31, 63;
  robot_width = 0.44;
  robot_length = 0.5;
  robot_wheelbase = 0.26;
  base_link_offset = 0.1;
  max_free_path_length = 5.0;
  max_clearance = 1.0; -- was 1.0
  can_traverse_stairs = false;
  use_map_speed = true;
  target_dist_tolerance = 0.1;
  target_vel_tolerance = 0.1;
  target_angle_tolerance = 0.05;
  local_fov = deg2rad(270);
  use_kinect = true;
  camera_calibration_path = "config/camera_calibration_left_flir.yaml";
  model_path = "../preference_learning_models/jit_cost_model_outdoor_6dim.pt";
  evaluator_type = "cost_map_service"; -- linear,  cost_map_service
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
  max_curvature = 1.5;
  clearance_path_clip_fraction = 0.05;
  max_fov = deg2rad(90);
};

DeepCostMapEvaluatorService = {
  service_name = "/navigation/deep_cost_map_service";

  -- costmap params
  crop_params = {
    center_x = 128.0; -- pixels right
    center_y = 96.0; -- pixels down
    width = 128.0; -- pixels
    height = 64.0; -- pixels
    output_width = 512; -- pixels
    output_height = 256; -- pixels
  };

  bev_pixels_per_meter = 40; -- 10 * 4x upscaling 128 -> 512
  min_cost = 0.0;
  max_cost = 1.0;
  discount_factor = 0.95;
  rollout_density = 3;

  angle_weight = 0.15;
  dist_to_goal_weight = 0.0;
  clearance_weight = 0.0;
  clearance_weight_beta = 0.0;
  fpl_weight = -0.2;
  learned_weight = 1.0;
  learned_weight_beta = 1.5;

  -- physical params
  base_link_offset_x = -0.2; -- m
  base_link_offset_y = 0.0; -- m

  -- visualization
  viz_radius = 3;
  viz_thickness = 2;
};