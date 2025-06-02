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

  prediction_horizon_ = 5;

  nx_ = 3; //state dimension
  nu_ = 2; //control input dimension
  ny_ = 3; //output dimension

  x_k_.resize(nx_);

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
  global_plan_ = path;
}

geometry_msgs::msg::TwistStamped MPCController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped& robot_pose,
  const geometry_msgs::msg::Twist& robot_velocity,
  nav2_core::GoalChecker* goal_checker)
{
  (void) robot_velocity;
  (void) goal_checker;

  // Obtain reference states
  // vector x_ref -> from path depending on horizon
  // std::vector<geometry_msgs::msg::PoseStamped> X_ref(
  //   global_plan_.poses.begin(),
  //   global_plan_.poses.begin() + prediction_horizon_
  // );

  Eigen::VectorXd X_ref (prediction_horizon_ * nx_);
  X_ref.setZero();

  // Get Yaw from Quaternion 
  double yaw = tf2::getYaw(robot_pose.pose.orientation);

  double dt = 0.05; // sampling time

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

  RCLCPP_INFO_STREAM_ONCE(logger_, "Block Matrix A: \n" << A_blk_);

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

  RCLCPP_INFO_STREAM_ONCE(logger_, "Block Matrix B: \n" << B_blk_);

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
  f = 2 * B_blk_.transpose() * Q_blk_ * (Ax_blk - X_ref); 
  
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

  Eigen::MatrixXd u(prediction_horizon_, nu_);
  u = qp_solver.solve(H, f, D, lb, ub);

  double linear_vel, angular_vel;
  // linear_vel = coeff(0, 0);
  // angular_vel = coeff(0, 1);

  linear_vel = 0.1;
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