#include "mpc_controller/mpc_controller.hpp"



namespace mpc_controller
{


void MPCController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, 
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  tf_buffer_ = tf_buffer;
  plugin_name_ = name;
  (void) costmap_ros;
  
  // logger_ = node->get_logger();
  clock_ = node->get_clock();

  params_callback_handle_ = node->add_on_set_parameters_callback(
    std::bind(&MPCController::paramsCallback, this, std::placeholders::_1)
  );

  node->declare_parameter(plugin_name_ + ".max_lin_vel", rclcpp::ParameterValue(0.5));
  node->declare_parameter(plugin_name_ + ".min_lin_vel", rclcpp::ParameterValue(0.5));
  node->declare_parameter(plugin_name_ + ".max_ang_vel", rclcpp::ParameterValue(-0.5));
  node->declare_parameter(plugin_name_ + ".min_ang_vel", rclcpp::ParameterValue(-0.5));
  node->declare_parameter(plugin_name_ + ".local_frame", rclcpp::ParameterValue(std::string("odom")));

  node->get_parameter(plugin_name_ + ".max_lin_vel", params_.max_lin_vel);
  node->get_parameter(plugin_name_ + ".min_lin_vel", params_.min_lin_vel);
  node->get_parameter(plugin_name_ + ".max_ang_vel", params_.max_ang_vel);
  node->get_parameter(plugin_name_ + ".min_ang_vel", params_.min_ang_vel);
  node->get_parameter(plugin_name_ + ".local_frame", params_.local_frame);

  closest_waypoint_publisher_ = node->create_publisher<geometry_msgs::msg::PointStamped>("closest_point", 10);
  lerp_ref_path_publisher_ = node->create_publisher<nav_msgs::msg::Path>("reference_path", 10);

  prediction_horizon_ = 5;

  nx_ = 3; //state dimension
  nu_ = 2; //control input dimension
  ny_ = 3; //output dimension

  x_k_.resize(nx_);
  X_ref_.resize(prediction_horizon_ * nx_);

  // Set matrix dimensions
  A_.resize(nx_, nx_);
  B_.resize(nx_, nu_);
  C_.resize(ny_, nx_);
  C_ << Eigen::MatrixXd::Identity(ny_, nx_);

  A_blk_.resize(prediction_horizon_ * ny_, nx_);
  B_blk_.resize(prediction_horizon_ * ny_, prediction_horizon_ * nu_);

  Q_.resize(ny_, ny_);
  Q_ << Eigen::MatrixXd::Identity(ny_, ny_) * 10;

  R_.resize(nu_, nu_);
  R_ << Eigen::MatrixXd::Identity(nu_, nu_) * 0.1;

  Q_blk_.resize(prediction_horizon_ * Q_.rows(), prediction_horizon_ * Q_.cols());
  R_blk_.resize(prediction_horizon_ * R_.rows(), prediction_horizon_ * R_.cols());
}

void MPCController::cleanup(){}

void MPCController::activate(){}

void MPCController::deactivate(){}

void MPCController::setSpeedLimit(const double &speed_limit, const bool &percentage){
  (void) speed_limit;
  (void) percentage;
}

void MPCController::setPlan(const nav_msgs::msg::Path& path){
  global_path_ = path;
}

geometry_msgs::msg::TwistStamped MPCController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped& robot_pose,
  const geometry_msgs::msg::Twist& robot_velocity,
  nav2_core::GoalChecker* goal_checker)
{
  (void) robot_velocity;
  (void) goal_checker;

  // nav_msgs::msg::Path pruned_path = prunePath(global_path_);

  // int global_path_section_num = global_path_.poses.size() - 1;
  // Eigen::VectorXd global_path_section_len(global_path_section_num);

  // int closest_waypoint_idx = findClosestWaypointIndex();


  int waypoints_num = global_path_.poses.size();
  RCLCPP_INFO_STREAM(logger_, "waypoints_num: " << waypoints_num);


  geometry_msgs::msg::PointStamped closest_wp;
  closest_wp.header.frame_id = global_path_.header.frame_id;
  closest_wp.header.stamp = clock_->now();
  closest_wp.point.x = global_path_.poses[0].pose.position.x;
  closest_wp.point.y = global_path_.poses[0].pose.position.y;
  closest_wp.point.z = 0;

  double closest_wp_dist = std::numeric_limits<double>::max();
  for(int i = 0; i < waypoints_num - 2; i++){

    double x_rob = robot_pose.pose.position.x;
    double y_rob = robot_pose.pose.position.y;

    double x_gp_i0 = global_path_.poses[i].pose.position.x;
    double y_gp_i0 = global_path_.poses[i].pose.position.y;

    double x_gp_i1 = global_path_.poses[i + 1].pose.position.x;
    double y_gp_i1 = global_path_.poses[i + 1].pose.position.y;

    double hypot = std::hypot(x_rob - x_gp_i0, y_rob - y_gp_i0);
    std::vector<double> v_rob = {x_rob - x_gp_i0, y_rob - y_gp_i0}; // vector from closest waypoint to robot
    std::vector<double> v_wps = {x_gp_i1 - x_gp_i0, y_gp_i1 - y_gp_i0}; // vector from closest waypoint to next waypoint along path
    
    double ip_rob = std::inner_product(v_rob.begin(), v_rob.end(), v_wps.begin(), 0.0);
    if (hypot < closest_wp_dist && ip_rob >= 0){
      closest_wp_dist = hypot;

      // linear interpolation between two waypoints
      double ip_wp = std::inner_product(v_wps.begin(), v_wps.end(), v_wps.begin(), 0.0);
      double t = ip_rob / ip_wp;

      closest_wp.point.x = x_gp_i0 + t * (x_gp_i1 - x_gp_i0);
      closest_wp.point.y = y_gp_i0 + t * (y_gp_i1 - y_gp_i0);
    }
  }

  closest_waypoint_publisher_->publish(closest_wp);

  std::vector<double> s_path_cum(waypoints_num, 0);
  for(int i = 1; i < waypoints_num; i++){
    double dx = global_path_.poses[i].pose.position.x - global_path_.poses[i - 1].pose.position.x;
    double dy = global_path_.poses[i].pose.position.y - global_path_.poses[i - 1].pose.position.y;
    s_path_cum[i] = s_path_cum[i - 1] + std::hypot(dx, dy);
  }


  // double s_predict = std::abs(linear_vel * dt); // predicted arc lengths
  double s_predict = 0.025;
  int segments_num = s_predict <= 0.0 ? 0.0 : std::ceil(s_path_cum.back() / s_predict);
  int fitted_points_num = std::min(prediction_horizon_ + 1, segments_num + 1);


  // RCLCPP_INFO_STREAM(logger_, "s_predict: " << s_predict);
  // RCLCPP_INFO_STREAM(logger_, "segments_num: " << segments_num);
  // RCLCPP_INFO_STREAM(logger_, "fitted_points_num: " << fitted_points_num);
  
  // Replace on tsd::begin(), std::end and std::partial_sum()
  std::vector<double> s_predict_cum(fitted_points_num, 0);
  for(int i = 1; i < fitted_points_num; i++){
    s_predict_cum[i] = s_predict_cum[i - 1] + s_predict;
  }

  std::stringstream ss;
  std::copy(s_predict_cum.begin(), s_predict_cum.end(), std::ostream_iterator<double>(ss, " "));
  // RCLCPP_INFO_STREAM(logger_, "arcl: " << ss.str());


  nav_msgs::msg::Path lerp_ref_path;
  lerp_ref_path.header.frame_id = global_path_.header.frame_id;
  lerp_ref_path.header.stamp = global_path_.header.stamp;
  lerp_ref_path.poses.resize(fitted_points_num);

  X_ref_.setZero();
  std::vector<double> x_refs(fitted_points_num);
  std::vector<double> y_refs(fitted_points_num);
  std::vector<double> theta_refs(fitted_points_num);
  int s_path_idx = 0;
  int s_predict_idx = 0;

  // while(s_predict_idx < fitted_points_num){
  //   double x_ref = 0.0;
  //   double y_ref = 0.0;
  //   double theta_ref = 0.0;

  //   if(waypoints_num == 1){
  //     x_ref = global_path_.poses[closest_wp_idx].pose.position.x;
  //     y_ref = global_path_.poses[closest_wp_idx].pose.position.y;
  //     theta_ref = tf2::getYaw(global_path_.poses[closest_wp_idx].pose.orientation);
  //   }else{

  //   }

  //   while(s_path_cum[s_path_idx] < s_predict_cum[s_predict_idx] && s_path_idx < waypoints_num){
  //     s_path_idx++;
  //   }

  //   double t = (s_predict_cum[s_predict_idx] - s_path_cum[s_path_idx - 1]) / (s_path_cum[s_path_idx] - s_path_cum[s_path_idx - 1]);
    
  //   double x_k0 = global_path_.poses[closest_wp_idx + s_path_idx - 1].pose.position.x;
  //   double x_k1 = global_path_.poses[closest_wp_idx + s_path_idx].pose.position.x;
  //   double x_ref = x_k0 + t * (x_k1 - x_k0);

  //   double y_k0 = global_path_.poses[closest_wp_idx + s_path_idx - 1].pose.position.y;
  //   double y_k1 = global_path_.poses[closest_wp_idx + s_path_idx].pose.position.y;
  //   double y_ref = y_k0 + t * (y_k1 - y_k0);

  //   double theta_ref = 0.0;
  //   // double theta_ref = tf2::getYaw(global_path_.poses[closest_wp_idx + s_path_idx].pose.orientation);









  //   lerp_ref_path.poses[s_predict_idx].pose.position.x = x_ref;
  //   lerp_ref_path.poses[s_predict_idx].pose.position.y = y_ref;

  //   X_ref_.segment(s_predict_idx * nx_, nx_) << x_ref, y_ref, theta_ref;

  //   RCLCPP_INFO_STREAM(logger_, "s_path_idx: " << s_path_idx);

  //   s_predict_idx++;
  // }

  
  // nav_msgs::msg::Path lerp_ref_path;
  // lerp_ref_path.header.frame_id = global_path_.header.frame_id;
  // lerp_ref_path.header.stamp = global_path_.header.stamp;
  // lerp_ref_path.poses.resize(prediction_horizon_);

  // for(int i = 0; i < prediction_horizon_; i++){
    
  //   double dx_ref = 0.0;
  //   double dy_ref = 0.0;
  //   double theta_ref = 0.0;
  //   tf2::Quaternion q;

  //   if(i == prediction_horizon_ - 1){
  //     theta_ref = 0.0;
  //   }else{
  //     dx_ref = x_refs[i + 1] - x_refs[i];
  //     dy_ref = y_refs[i + 1] - y_refs[i];
  //     theta_ref = std::atan2(dy_ref, dx_ref);
  //   }
    
  //   X_ref_.segment(i * nx_, nx_) << x_refs[i], y_refs[i], theta_ref;

  //   lerp_ref_path.poses[i].pose.position.x = x_refs[i];
  //   lerp_ref_path.poses[i].pose.position.y = y_refs[i];
  //   q.setRPY(0, 0, theta_ref);
  //   lerp_ref_path.poses[i].pose.orientation = tf2::toMsg(q);
  // }

  RCLCPP_INFO_STREAM(logger_, "X_ref_: \n" << X_ref_);
  lerp_ref_path_publisher_->publish(lerp_ref_path);
  
  // tf2::getYaw(global_path_.poses[i].pose.orientation);

  // TODO: express coordinates in a moving coordinate system instead of global

  double yaw = tf2::getYaw(robot_pose.pose.orientation);
  x_k_ << robot_pose.pose.position.x,
          robot_pose.pose.position.y,
          yaw;

  // Define system dynamic
  double a13 = -params_.max_lin_vel * std::sin(yaw) * dt;
  double a23 = params_.max_lin_vel * std::cos(yaw) * dt;

  A_ << 1, 0, a13,
        0, 1, a23,
        0, 0, 1;

  double b11 = std::cos(yaw) * dt;
  double b21 = std::sin(yaw) * dt;

  B_ << b11, 0,
        b21, 0,
        0 , dt;

  // =======================
  // === System stacking ===
  // =======================

  // Stacking A matrix
  Eigen::MatrixXd A_pow(A_.rows(), A_.cols());
  A_pow << A_;

  A_blk_.setZero();

  for(int i = 0; i < prediction_horizon_; i++){
    if(i) A_pow *= A_;
    A_blk_.block(i * ny_, 0, ny_, nx_) = C_ * A_pow;
  }

  // RCLCPP_INFO_STREAM_ONCE(logger_, "Block Matrix A: \n" << A_blk_);

  // Stacking B matrix
  A_pow.setZero();
  B_blk_.setZero();

  for(int i = 0; i < prediction_horizon_; i++){
    
    A_pow << Eigen::MatrixXd::Identity(ny_, nx_);
    for(int j = 0; j < prediction_horizon_ - i; j++){
      
      if(j) A_pow *= A_;
      B_blk_.block((j + i) * nx_, i * nu_, nx_, nu_) = C_ * A_pow * B_;
    }
  }

  // RCLCPP_INFO_STREAM_ONCE(logger_, "Block Matrix B: \n" << B_blk_);

  // ==================
  // === QP problem ===
  // ==================

  // Represent Cost function as QP problem
  // J = 0.5 u H u^T + f^T u
  // Subject to:
  // Du <= b

  Eigen::VectorXd Ax_blk(prediction_horizon_ * ny_);
  Ax_blk.setZero();

  for(int i = 0; i < prediction_horizon_; i++){
    Ax_blk.segment(i * ny_, nx_) = A_blk_.block(i * ny_, 0, ny_, nx_) * x_k_;
  }
  RCLCPP_INFO_STREAM(logger_, "Ax_blk: \n" << Ax_blk);

  for(int i = 0; i < prediction_horizon_; i++){
    Q_blk_.block(i * nx_, i * nx_, nx_, nx_) = Q_;
    R_blk_.block(i * nu_, i * nu_, nu_, nu_) = R_;
  }

  // Hessian matrixs
  Eigen::MatrixXd H(prediction_horizon_ * nu_, prediction_horizon_ * nu_); 
  H = 2 * (B_blk_.transpose() * Q_blk_ * B_blk_ + R_blk_);  

  // Linear term
  Eigen::VectorXd f(prediction_horizon_ * nu_);
  f = 2 * B_blk_.transpose() * Q_blk_ * (Ax_blk - X_ref_); 
  
  Eigen::MatrixXd D(prediction_horizon_ * nu_, nu_);

  for(int i = 0; i < prediction_horizon_; i++){
    D.block(i * nu_, 0, nu_, nu_) = Eigen::MatrixXd::Identity(nu_, nu_);
  }

  Eigen::VectorXd lb (prediction_horizon_ * nu_);
  Eigen::VectorXd ub (prediction_horizon_ * nu_);
  
  for(int i = 0; i < prediction_horizon_; i++){
    lb.segment(i * nu_, nu_) << params_.min_lin_vel, params_.min_ang_vel;
    ub.segment(i * nu_, nu_) << params_.max_lin_vel, params_.max_ang_vel;
  }

  // RCLCPP_INFO_STREAM_ONCE(logger_, "Block Matrix H: \n" << H);
  // RCLCPP_INFO_STREAM_ONCE(logger_, "Linear term f: \n" << f);

  // === Solve QP problem ===
  
  osqp::OSQPSolverInterface qp_solver(logger_);

  Eigen::VectorXd u(prediction_horizon_ * nu_);
  qp_solver.solve(H, f, D, lb, ub , u);

  RCLCPP_INFO_STREAM(logger_, "Control u: [" << u[0] << ", " << u[1] << "]");

  linear_vel = u[0];
  angular_vel = u[1];

  Eigen::VectorXd X_pred(prediction_horizon_ * nx_);
  X_pred << Ax_blk + B_blk_ * u;

  // RCLCPP_INFO_STREAM(logger_, "Optimized state:\n" << X_pred);
  RCLCPP_INFO_STREAM(logger_, "State error:\n" << X_ref_ - X_pred);
  
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = robot_pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = 0.0;
  // cmd_vel.twist.linear.x = linear_vel;
  // cmd_vel.twist.angular.z = angular_vel;
  return cmd_vel;
}


} // namespace mpc_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(mpc_controller::MPCController, nav2_core::Controller)