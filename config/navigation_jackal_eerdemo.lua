function deg2rad(deg)
  return deg * (math.pi / 180)
end

NavigationParameters = {
  laser_topic = "/scan";
  odom_topic = "/jackal_velocity_controller/odom";
  localization_topic = "/localization";
  image_topic = "/bev/single/compressed";
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
  carrot_dist = 5;
  system_latency = 0.24;
  obstacle_margin = 0.15;
  num_options = 31;
  robot_width = 0.44;
  robot_length = 0.5;
  base_link_offset = 0;
  max_free_path_length = 5;
  max_clearance = 1.0;
  can_traverse_stairs = false;
  evaluator_type = "terrain";
  camera_calibration_path = "config/camera_calibration_kinect.yaml";
  model_path = "";
  local_fov = deg2rad(180);  -- do not turn in place when the goal is behind the robot
  target_dist_tolerance = 0.5;
  target_vel_tolerance = 0.2;
  target_angle_tolerance = deg2rad(20);
  use_map_speed = false;
  use_kinect = false;
};

AckermannSampler = {
  max_curvature = 2.5;
  clearance_path_clip_fraction = 0.8;
};

TerrainEvaluator = {
  patch_size_pixels = 64;
  bev_pixels_per_meter = 150;
  min_cost = 0.0;
  max_cost = 2.5;
  -- discount_factor = 0.5;
  discount_factor = 0.8; -- ahg demo
  rollout_density = 20;

  -- model_path = "/home/eyang/tmp/outdoor-6t-10-16-16-58-43-e19.pt";

  -- trained with fall transitional grass
  model_path = "/home/amrl_user/rahul/models/eerdemo.pt";

  -- dist_to_goal_weight = -0.2;
  dist_to_goal_weight = -0.3;
  clearance_weight = 0; -- -0.25;
  fpl_weight = 0; -- -0.75;
  terrain_weight = 6.0;
}
