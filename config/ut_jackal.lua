function deg2rad(deg)
  return deg * (math.pi / 180)
end

NavigationParameters = {
  laser_topic = "/velodyne_2dscan";
  odom_topic = "/jackal_velocity_controller/odom";
  localization_topic = "localization";
  init_topic = "initialpose";
  enable_topic = "autonomy_arbiter/enabled";
  image_topic = "/camera/rgb/image_raw/compressed";
  laser_loc = {
    x = 0.12;
    y = 0;
  };
  dt = 0.025;
  max_linear_accel = 3;
  max_linear_decel = 3;
  max_linear_speed = 1.0;
  max_angular_accel = 5;
  max_angular_decel = 5;
  max_angular_speed = 5.0;
  carrot_dist = 1;
  system_latency = 0.24;
  obstacle_margin = 0.05;
  num_options = 71;
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
  
  local_fov = deg2rad(60);
  use_kinect = true;
  camera_calibration_path = "config/camera_calibration_kinect.yaml";
  model_path = "../preference_learning_models/jit_cost_model_outdoor_6dim.pt";
  evaluator_type = "linear";
};

AckermannSampler = {
  max_curvature = 10;
  clearance_path_clip_fraction = 0.8;
};
