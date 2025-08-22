#include "mpc_controller/mpc_controller.hpp"



namespace mpc_controller
{

void MPCController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
  std::string name, 
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  (void) costmap_ros;

  node_ = parent.lock();
  clock_ = node_->get_clock();
  tf_buffer_ = tf_buffer;
  plugin_name_ = name;
  
  parameters_manager_.configure(parent, name);
  params_ = parameters_manager_.get_parameters();

  path_manager_.configure(parent, params_);

  x_k_.resize(params_->nx);
  X_ref_.resize(params_->prediction_horizon * params_->nx);

  // Set matrix dimensions
  A_.resize(params_->nx, params_->nx);
  B_.resize(params_->nx, params_->nu);
  C_.resize(params_->ny, params_->nx);
  C_ << Eigen::MatrixXd::Identity(params_->ny, params_->nx);

  A_blk_.resize(params_->prediction_horizon * params_->ny, params_->nx);
  B_blk_.resize(params_->prediction_horizon * params_->ny, params_->prediction_horizon * params_->nu);

  Q_.resize(params_->ny, params_->ny);
  Q_ << Eigen::MatrixXd::Identity(params_->ny, params_->ny) * 10;

  R_.resize(params_->nu, params_->nu);
  R_ << Eigen::MatrixXd::Identity(params_->nu, params_->nu) * 0.1;

  Q_blk_.resize(params_->prediction_horizon * Q_.rows(), params_->prediction_horizon * Q_.cols());
  R_blk_.resize(params_->prediction_horizon * R_.rows(), params_->prediction_horizon * R_.cols());
}

void MPCController::cleanup(){}

void MPCController::activate(){}

void MPCController::deactivate(){}

void MPCController::setSpeedLimit(const double &speed_limit, const bool &percentage){
  (void) speed_limit;
  (void) percentage;
}

void MPCController::setPlan(const nav_msgs::msg::Path& path){
  path_manager_.setGlobalPath(path);
}

geometry_msgs::msg::TwistStamped MPCController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped& robot_pose,
  const geometry_msgs::msg::Twist& robot_velocity,
  nav2_core::GoalChecker* goal_checker)
{
  (void) robot_velocity;
  (void) goal_checker;

  X_ref_ = path_manager_.computeReferencePath(robot_pose);

  double yaw = tf2::getYaw(robot_pose.pose.orientation);
  x_k_ << robot_pose.pose.position.x,
          robot_pose.pose.position.y,
          yaw;

  // Define system dynamic
  double linear_vel = robot_velocity.linear.x;
  double a13 = -linear_vel * std::sin(yaw) * params_->dt;
  double a23 = linear_vel * std::cos(yaw) * params_->dt;

  A_ << 1, 0, a13,
        0, 1, a23,
        0, 0, 1;

  double b11 = std::cos(yaw) * params_->dt;
  double b21 = std::sin(yaw) * params_->dt;

  B_ << b11, 0,
        b21, 0,
        0, params_->dt;

  // =======================
  // === System stacking ===
  // =======================

  // Stacking A matrix
  Eigen::MatrixXd A_pow(A_.rows(), A_.cols());
  A_pow << A_;

  A_blk_.setZero();
  for(int row = 0; row < params_->prediction_horizon; row++){
    if(row) A_pow *= A_;
    A_blk_.block(row * params_->ny, 0, params_->ny, params_->nx) = C_ * A_pow;
  }

  // Stacking B matrix
  A_pow.setZero();
  B_blk_.setZero();
  for(int col = 0; col < params_->prediction_horizon; col++){
    A_pow << Eigen::MatrixXd::Identity(A_pow.rows(), A_pow.cols());
    for(int row = 0; row < params_->prediction_horizon - col; row++){
      if(row) A_pow *= A_;
      B_blk_.block((row + col) * params_->nx, col * params_->nu, params_->nx, params_->nu) = C_ * A_pow * B_;
    }
  }

  // ==================
  // === QP problem ===
  // ==================

  // Represent Cost function as QP problem
  // J = 0.5 u H u^T + f^T u
  // Subject to:
  // Du <= b

  Eigen::VectorXd Ax_blk(params_->prediction_horizon * params_->ny);
  Ax_blk.setZero();
  for(int row = 0; row < params_->prediction_horizon; row++){
    Ax_blk.segment(row * params_->ny, params_->nx) = A_blk_.block(row * params_->ny, 0, params_->ny, params_->nx) * x_k_;
  }

  for(int row = 0; row < params_->prediction_horizon; row++){
    Q_blk_.block(row * params_->nx, row * params_->nx, params_->nx, params_->nx) = Q_;
    R_blk_.block(row * params_->nu, row * params_->nu, params_->nu, params_->nu) = R_;
  }

  // Hessian matrixs
  Eigen::MatrixXd H(params_->prediction_horizon * params_->nu, params_->prediction_horizon * params_->nu); 
  H = 2 * (B_blk_.transpose() * Q_blk_ * B_blk_ + R_blk_);  

  // Linear term
  Eigen::VectorXd f(params_->prediction_horizon * params_->nu);
  f = 2 * B_blk_.transpose() * Q_blk_ * (Ax_blk - X_ref_); 
  
  Eigen::MatrixXd D(params_->prediction_horizon * params_->nu, params_->prediction_horizon * params_->nu);
  D << Eigen::MatrixXd::Identity(params_->prediction_horizon * params_->nu, params_->prediction_horizon * params_->nu);

  Eigen::VectorXd lb (params_->prediction_horizon * params_->nu);
  Eigen::VectorXd ub (params_->prediction_horizon * params_->nu);
  
  for(int row = 0; row < params_->prediction_horizon; row++){
    lb.segment(row * params_->nu, params_->nu) << params_->min_lin_vel, params_->min_ang_vel;
    ub.segment(row * params_->nu, params_->nu) << params_->max_lin_vel, params_->max_ang_vel;
  }

  // === Solve QP problem ===
  
  osqp::OSQPSolverInterface qp_solver(logger_);

  Eigen::VectorXd u(params_->prediction_horizon * params_->nu);
  qp_solver.solve(H, f, D, lb, ub , u);

  Eigen::VectorXd X_pred(params_->prediction_horizon * params_->nx);
  X_pred << Ax_blk + B_blk_ * u;
  path_manager_.publishOptimalTrajectory(X_pred);

  // RCLCPP_INFO_STREAM(logger_, "Optimized state:\n" << X_pred);
  // RCLCPP_INFO_STREAM(logger_, "State error:\n" << X_ref_ - X_pred);
  
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = robot_pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist.linear.x = u[0];
  cmd_vel.twist.angular.z = u[1];
  return cmd_vel;
}


} // namespace mpc_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(mpc_controller::MPCController, nav2_core::Controller)