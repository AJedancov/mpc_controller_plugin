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
  Q_ << Eigen::MatrixXd::Identity(ny_, ny_);

  R_.resize(nu_, nu_);
  R_ << Eigen::MatrixXd::Identity(nu_, nu_);

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

  int closest_waypoint_idx = 0;
  double closest_waypoint_dist = 1e3;
  for(int i = 0; i < waypoints_num; i++){
    double dx = robot_pose.pose.position.x - global_path_.poses[i].pose.position.x;
    double dy = robot_pose.pose.position.y - global_path_.poses[i].pose.position.y;
    double hypot = std::hypot(dx, dy);
    if (hypot < closest_waypoint_dist){
      closest_waypoint_idx = i;
      closest_waypoint_dist = hypot;
    }
  }

  RCLCPP_INFO_STREAM(logger_, "closest_waypoint_idx: " << closest_waypoint_idx);
  RCLCPP_INFO_STREAM(logger_, "closest_waypoint_dist: " << closest_waypoint_dist);

  geometry_msgs::msg::PointStamped closest_waypoint;
  closest_waypoint.header.frame_id = global_path_.header.frame_id;
  closest_waypoint.header.stamp = clock_->now();
  closest_waypoint.point.x = global_path_.poses[closest_waypoint_idx].pose.position.x;
  closest_waypoint.point.y = global_path_.poses[closest_waypoint_idx].pose.position.y;
  closest_waypoint.point.z = 0;

  closest_waypoint_publisher_->publish(closest_waypoint);

  std::vector<double> s_path_cum(waypoints_num, 0);
  for(int i = 1; i < waypoints_num; i++){
    double dx = global_path_.poses[i].pose.position.x - global_path_.poses[i - 1].pose.position.x;
    double dy = global_path_.poses[i].pose.position.y - global_path_.poses[i - 1].pose.position.y;
    s_path_cum[i] = s_path_cum[i - 1] + std::hypot(dx, dy);
  }

  // RCLCPP_INFO_STREAM(logger_, "linear_vel: " << linear_vel);

  // double s_predict = linear_vel * dt; // predicted arc lengths
  double s_predict = 0.1;
  int segments_num = s_predict <= 0.0 ? 0.0 : std::floor(s_path_cum.back() / s_predict);
  int fitted_points_num = std::min(prediction_horizon_, segments_num) + 1;
  
  RCLCPP_INFO_STREAM(logger_, "s_predict: " << s_predict);
  RCLCPP_INFO_STREAM(logger_, "segments_num: " << segments_num);
  RCLCPP_INFO_STREAM(logger_, "fitted_points_num: " << fitted_points_num);
  
  // Replace on tsd::begin(), std::end and std::partial_sum()
  std::vector<double> s_predict_cum(fitted_points_num, 0);
  for(int i = 1; i < fitted_points_num; i++){
    s_predict_cum[i] = s_predict_cum[i - 1] + s_predict;
  }

  std::stringstream ss;
  std::copy(s_predict_cum.begin(), s_predict_cum.end(), std::ostream_iterator<double>(ss, " "));
  RCLCPP_INFO_STREAM(logger_, "arcl: " << ss.str());

  X_ref_.setZero();
  int s_path_idx = 1;
  int s_predict_idx = 1;

  while(s_predict_idx < fitted_points_num){

    while(s_path_cum[s_path_idx] < s_predict_cum[s_predict_idx] && s_path_idx < waypoints_num){
      s_path_idx++;
    }

    double t = (s_predict_cum[s_predict_idx] - s_path_cum[s_path_idx - 1]) / (s_path_cum[s_path_idx] - s_path_cum[s_path_idx - 1]);
    
    double x_k0 = global_path_.poses[closest_waypoint_idx + s_path_idx - 1].pose.position.x;
    double x_k1 = global_path_.poses[closest_waypoint_idx + s_path_idx].pose.position.x;
    double x_ref = x_k0 + t * (x_k1 - x_k0);

    double y_k0 = global_path_.poses[closest_waypoint_idx + s_path_idx - 1].pose.position.y;
    double y_k1 = global_path_.poses[closest_waypoint_idx + s_path_idx].pose.position.y;
    double y_ref = y_k0 + t * (y_k1 - y_k0);

    double theta_ref = 0.0;

    X_ref_.segment((s_predict_idx - 1) * nx_, nx_) << x_ref, y_ref, theta_ref;

    RCLCPP_INFO_STREAM(logger_, "s_path_idx: " << s_path_idx);
    RCLCPP_INFO_STREAM(logger_, "s_predict_idx: " << s_predict_idx);

    s_predict_idx++;
  }

  RCLCPP_INFO_STREAM(logger_, "X_ref_: \n" << X_ref_);
  
  // tf2::getYaw(global_path_.poses[i].pose.orientation);

  // TODO: express coordinates in a moving coordinate system instead of global

  double yaw = tf2::getYaw(robot_pose.pose.orientation);
  x_k_ << robot_pose.pose.position.x,
          robot_pose.pose.position.y,
          yaw;

  // Define system dynamic
  double a13 = -params_.max_lin_vel * std::sin(yaw) * dt;
  double a23 = params_.max_lin_vel * std::cos(yaw) * dt;

  // TODO: check dt in matrix
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

  // RCLCPP_INFO_STREAM(logger_, "\nControl u:\n" << u[0] << "\n" << u[1]);

  linear_vel = u[0];
  angular_vel = u[1];

  linear_vel = 0.0;
  angular_vel = 0.0;
  
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = robot_pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;

  return cmd_vel;
}


} // namespace mpc_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(mpc_controller::MPCController, nav2_core::Controller)