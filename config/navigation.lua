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
  init_topic = "initialpose";
  enable_topic = "autonomy_arbiter/enabled";
  dt = 0.060;
  max_linear_accel = 0.5;
  max_linear_decel = 0.5;
  max_linear_speed = 0.5;
  max_angular_accel = 0.5;
  max_angular_decel = 0.5;
  max_angular_speed = 1.0;
  carrot_dist = 3.5;
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
  evaluator_type = "linear";
  recovery_carrot_dist = 0.7;
  goto_topic = "/move_base_simple/goal";
  goto_amrl_topic = "/move_base_simple/goal_amrl";
  reset_nav_goals_topic = "/reset_nav_goals";
  halt_topic = "halt_robot";
  override_topic = "nav_override";
  ackermann_drive_topic = "ackermann_curvature_drive";
  nav_status_topic = "navigation_goal_status";
  visualization_topic = "visualization";
  fp_pcl_topic = "forward_predicted_pcl";
  path_topic = "trajectory";
  carrot_topic = "carrot";
};

motion_primitives_mode = "omni";  -- "ackermann" or "omni"

AckermannSampler = {
  max_curvature = 2.5;
  clearance_path_clip_fraction = 0.8;
};

OmniSampler = {
  max_speed = 0.5;
  max_angular_speed = 1.0;
  num_directions = 7;
};