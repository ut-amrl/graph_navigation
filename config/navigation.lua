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
  base_link_offset_x = 0;
  base_link_offset_y = 0;
  max_free_path_length = 6.0;
  max_clearance = 1.0;
  local_half_fov = deg2rad(90);
  center_threshold = deg2rad(10);
  can_traverse_stairs = false;
  target_dist_tolerance = 0.1;
  target_vel_tolerance = 0.1;
  target_angle_tolerance = deg2rad(5);
  target_omega_tolerance = 0.15;
  evaluator_type = "linear";
  carrot_dist = 3.5;
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

-- Command mapping parameters for Cobot driver
CommandMapping = {
  apply_custom_cmd_map = false;
  linear_models = {
    x = {
      slope_pos = 16.8038;
      intercept_pos = 0.975299;
      slope_neg = 16.4849;
      intercept_neg = -0.0220109;
    };
    y = {
      slope_pos = 16.6057;
      intercept_pos = 0.967648;
      slope_neg = 16.6521;
      intercept_neg = 0.055238;
    };
    r = {
      slope_pos = 3.42393;
      intercept_pos = 0.98112;
      slope_neg = 3.39072;
      intercept_neg = -0.0108195;
    };
  };
};