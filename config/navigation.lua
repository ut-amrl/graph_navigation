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
  num_options = 31;
  robot_width = 0.44;
  robot_length = 0.5;
  geometric_center_offset = {
    x = 0;
    y = 0;
  };
  obstacle_margin = 0.15;
  max_rollout_length = 6.0;
  max_lookahead_fpl = 6.0;
  clearance_band = 1.0;
  lidar_fov_half_angle = deg2rad(60);
  can_traverse_stairs = false;
  target_dist_tolerance = 0.1;
  nudge_dist_tolerance = 0.3;
  target_vel_tolerance = 0.1;
  target_angle_tolerance = deg2rad(5);
  target_omega_tolerance = 0.15;
  evaluator_type = "linear";
  carrot_dist = 3.5;
  motion_primitives_mode = "omni";
  do_ang_toc = false;
  max_plan_deviation = 0.5;
  laser_height = 0.324;
  stuck_meta_control = {
    override_obstacle_margin = 0.1;
    stuck_timeout_sec = 10.0;
    improve_eps = 0.02;
  };
};

ROSTopics = {
  laser_topics = {
    "/scan",
    -- "/velodyne_2dscan",
    "/kinect_laserscan",
  };
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
  visualization_local_topic = "visualization_local";
  fp_pcl_topic = "forward_predicted_pcl";
  path_topic = "trajectory";
  carrot_topic = "carrot";
  current_map_topic = "/current_map";
  robot_geometry_topic = "/robot_geometry_update"; -- [width, length, offset_x, offset_y, obstacle_margin, do_ang_toc(0/1)]
};

ROSFrames = {
  map_frame = "map";
  robot_frame = "base_link";
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
