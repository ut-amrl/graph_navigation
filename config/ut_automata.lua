function deg2rad(deg)
  return deg * (math.pi / 180)
end

NavigationParameters = {
  laser_topic = "/scan";
  odom_topic = "/odom";
  localization_topic = "localization";
  image_topic = "/camera/rgb/image_raw/compressed";
  init_topic = "initialpose";
  enable_topic = "autonomy_arbiter/enabled";

  laser_loc = {
    x = 0.2;
    y = 0;
  };

  -- Control loop interval.
  dt = 0.025;

  -- Motion limits and dynamics constraints.
  max_linear_accel = 6;
  max_linear_decel = 9;
  max_linear_speed = 2.0;
  max_angular_accel = 0.5;
  max_angular_decel = 0.5;
  max_angular_speed = 1.0;
  system_latency = 0.24;

  -- Local planner parameters.
  evaluator_type = "linear";
  carrot_dist = 10;
  max_free_path_length = 3.5;
  max_clearance = 1.0;
  num_options = 41;

  -- Car dimensions.
  robot_width = 0.27;
  robot_length = 0.45;
  base_link_offset = -0.162;
  obstacle_margin = 0.001;

  -- Global planner parameters.
  can_traverse_stairs = false;
  use_map_speed = false; 
  target_dist_tolerance = 0.1;
  target_vel_tolerance = 0.1;
  target_angle_tolerance = 0.05;
  local_fov = deg2rad(90);

  -- VRL-PAP parameters.
  use_kinect = true;
  camera_calibration_path = "config/camera_calibration_kinect.yaml";
  model_path = "../preference_learning_models/jit_cost_model_outdoor_6dim.pt";
};

AckermannSampler = {
  max_curvature = 2.5;
  clearance_path_clip_fraction = 0.8;
  clip_cpoa = false;
};
