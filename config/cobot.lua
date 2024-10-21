function deg2rad(deg)
  return deg * (math.pi / 180)
end

NavigationParameters = {
    laser_topic = "/Cobot/Laser";
    odom_topic = "/odom";
    localization_topic = "/localization";
    image_topic = "/camera/rgb/image_raw/compressed";
    init_topic = "/initialpose";
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
    carrot_dist = 100; --10;
    system_latency = 1.0; --0.24;
    obstacle_margin = 0.15;
    num_options = 31;
    robot_width = 0.6; --0.44;
    robot_length = 0.6; --0.5;
    base_link_offset = 0;
    max_free_path_length = 100.0; --6.0;
    max_clearance = 1.0;
    can_traverse_stairs = false;
    use_map_speed = true;
    target_dist_tolerance = 0.1;
    target_vel_tolerance = 0.1;
    target_angle_tolerance = 0.008; --0.01;
    local_fov = deg2rad(15);
    use_kinect = true;
    camera_calibration_path = "/home/cchsu/cobot-moma/graph_navigation/config/camera_calibration_kinect.yaml";
    model_path = "../preference_learning_models/jit_cost_model_outdoor_6dim.pt";
    evaluator_type = "linear";
};

AckermannSampler = {
  max_curvature = 20; --2.5;
  clearance_path_clip_fraction = 0.8;
};
