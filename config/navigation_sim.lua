NavigationParameters = {
  image_topic = "/left/image_color/compressed";
  laser_topic = "/robot0/scan";
  odom_topic = "/robot0/odom";
  localization_topic = "/robot0/localization";
  init_topic = "/robot0/initialpose";
  enable_topic = "autonomy_arbiter/enabled";
  laser_loc = {
    x = 0.065;
    y = 0;
  };
  dt = 0.05;
  max_linear_accel = 0.5;
  max_linear_decel = 1.0;
  max_linear_speed = 0.75;
  max_angular_accel = 0.25;
  max_angular_decel = 0.25;
  max_angular_speed = 0.5;
  carrot_dist = 100.0;
  system_latency = 0.24;
  obstacle_margin = 0.3;
  num_options = 31;
  robot_width = 0.6;
  robot_length = 0.5;
  base_link_offset = 0;
  max_free_path_length = 4.0;
  max_clearance = 1.0;
  can_traverse_stairs = false;
  use_map_speed = true;
  use_kinect = false;
  target_dist_tolerance = 1.2;
  target_vel_tolerance = 0.1;
  target_angle_tolerance = 0.05;
  model_path = "../preference_learning/comprehensive_models/jit_cost_model_indoor_6dim.pt";
  embedding_model_path = "../preference_learning/comprehensive_models/jit_emb_model_indoor_6dim.pt";
  evaluator_type = "cost_map";
  blur = true;
};

AckermannSampler = {
  max_curvature = 2;
};
