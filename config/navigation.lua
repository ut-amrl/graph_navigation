function deg2rad(deg)
  return deg * (math.pi / 180)
end

NavigationParameters = {
  dt = 0.060;
  linear_limits = {
    max_acceleration = 0.5;
    max_deceleration = 0.5;
    max_speed = 0.5;
  };
  angular_limits = {
    max_acceleration = 0.5;
    max_deceleration = 0.5;
    max_speed = 1.0;
  };
  actuation_latency = 0.24;
  obstacle_margin = 0.15;
  num_options = 31;
  robot_width = 0.44;
  robot_length = 0.5;
  base_link_offset = 0;
  max_free_path_length = 6.0;
  max_clearance = 1.0;
  local_fov = deg2rad(90);
  use_map_speed = true;
  can_traverse_stairs = false;
  target_dist_tolerance = 0.1;
  target_vel_tolerance = 0.1;
  target_angle_tolerance = deg2rad(5);
  evaluator_type = "linear";
  carrot_dist = 3.5;
  recovery_carrot_dist = 0.7;
  motion_primitives_mode = "omni";
  do_ang_toc = false;
};

ROSTopics = {
  laser_topics = {
    "/scan",
    -- "/velodyne_2dscan",
    "/kinect_laserscan",
  };
  laser_frame = "base_link";
  odom_topic = "/odom";
  localization_topic = "localization";
  goto_topic = "/move_base_simple/goal";
  goto_amrl_topic = "/move_base_simple/goal_amrl";
  reset_nav_goals_topic = "/reset_nav_goals";
  halt_topic = "halt_robot";
  twist_drive_topic = "navigation/cmd_vel";
  ackermann_drive_topic = "ackermann_curvature_drive";
  nav_status_topic = "navigation_goal_status";
  visualization_topic = "visualization";
  fp_pcl_topic = "forward_predicted_pcl";
  path_topic = "trajectory";
  carrot_topic = "carrot";
};

AckermannSampler = {
  max_curvature = 2.5;
  clearance_path_clip_fraction = 0.8;
};