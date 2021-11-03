function deg2rad(deg)
  return deg * (math.pi / 180)
end

NavigationParameters = {
  laser_topic = "scan";
  odom_topic = "odom";
  localization_topic = "localization";
  init_topic = "initialpose";
  enable_topic = "autonomy_arbiter/enabled";
  laser_loc = {
    x = 0.065;
    y = 0;
  };
  dt = 0.025;
  max_linear_accel = 0.5;
  max_linear_decel = 0.5;
  max_linear_speed = 0.5;
  max_angular_accel = 0.5;
  max_angular_decel = 0.5;
  max_angular_speed = 1.0;
  carrot_dist = 2.5;
  system_latency = 0.24;
  obstacle_margin = 0.15;
  num_options = 41;
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
  local_fov = deg2rad(50);
};

AckermannSampler = {
  max_curvature = 3;
  clearance_path_clip_fraction = 0.8;
};
