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

  x0_.resize(params_->nx);
  X_ref_.resize(params_->prediction_horizon * params_->nx);

  // Set matrix dimensions
  A_.resize(params_->nx, params_->nx);
  B_.resize(params_->nx, params_->nu);
  C_.resize(params_->ny, params_->nx);
  C_ << Eigen::MatrixXd::Identity(params_->ny, params_->nx);

  A_stacked_.resize(params_->prediction_horizon * params_->ny, params_->nx);
  B_stacked_.resize(params_->prediction_horizon * params_->ny, 
    params_->prediction_horizon * params_->nu);

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
  x0_ << robot_pose.pose.position.x,
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

  // === Input increment model ===
  // [x_k+1] = [A B]*[x_k  ]+[B]*delta_u
  // [u_k]     [0 I] [u_k-1] [I]
  // 
  // y_k = [C 0]*[x_k  ]
  //             [u_K-1] 

  int A_rows_number = A_.rows();
  int A_cols_number = A_.cols();
  int B_rows_number = B_.rows();
  int B_cols_number = B_.cols();
  Eigen::MatrixXd A_augmented(A_rows_number + B_cols_number, A_cols_number + B_cols_number);
  A_augmented.block(0, 0, A_rows_number, A_cols_number) = A_;
  A_augmented.block(0, A_cols_number, B_rows_number, B_cols_number) = B_;
  A_augmented.block(A_rows_number, A_cols_number, B_cols_number, B_cols_number) =
    Eigen::MatrixXd::Identity(B_cols_number, B_cols_number);

  Eigen::MatrixXd B_augmented(B_rows_number + B_cols_number, B_cols_number);
  B_augmented.block(0, 0, B_rows_number, B_cols_number) = B_;
  B_augmented.block(B_rows_number, 0, B_cols_number, B_cols_number) =
    Eigen::MatrixXd::Identity(B_cols_number, B_cols_number);

  int C_rows_number = C_.rows();
  int C_cols_number = C_.cols();
  Eigen::MatrixXd C_augmented(C_rows_number, C_cols_number + B_cols_number);
  C_augmented.block(0, 0, C_rows_number, C_cols_number) = C_;


  // =======================
  // === System stacking ===
  // =======================
  // A_stacked_ = stackMatrixA(A_, C_);
  // B_stacked_ = stackMatrixB(A_, B_, C_);

  A_stacked_ = stackMatrixA(A_augmented, C_augmented);
  B_stacked_ = stackMatrixB(A_augmented, B_augmented, C_augmented);

  // ==================
  // === QP problem ===
  // ==================

  // Represent Cost function as QP problem
  // J = 0.5 u H u^T + f^T u
  // Subject to:
  // Du <= b

  Eigen::VectorXd Ax0_stacked(params_->prediction_horizon * params_->ny);
  Ax0_stacked.setZero();
  for(int row = 0; row < params_->prediction_horizon; row++){
    Ax0_stacked.segment(row * params_->ny, params_->nx) = 
      A_stacked_.block(row * params_->ny, 0, params_->ny, params_->nx) * x0_;
  }

  for(int row = 0; row < params_->prediction_horizon; row++){
    Q_blk_.block(row * params_->nx, row * params_->nx, params_->nx, params_->nx) = Q_;
    R_blk_.block(row * params_->nu, row * params_->nu, params_->nu, params_->nu) = R_;
  }

  // Hessian matrixs
  Eigen::MatrixXd H(params_->prediction_horizon * params_->nu, params_->prediction_horizon * params_->nu); 
  H = 2 * (B_stacked_.transpose() * Q_blk_ * B_stacked_ + R_blk_);  

  // Linear term
  Eigen::VectorXd f(params_->prediction_horizon * params_->nu);
  f = 2 * B_stacked_.transpose() * Q_blk_ * (Ax0_stacked - X_ref_); 
  
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
  X_pred << Ax0_stacked + B_stacked_ * u;
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


Eigen::MatrixXd MPCController::stackMatrixA(
  const Eigen::MatrixXd& A,
  const Eigen::MatrixXd& C)
{
  int A_rows_number = A.rows();
  int A_cols_number = A.cols();
  int C_rows_number = C.rows();
  Eigen::MatrixXd A_pow(A_rows_number, A_cols_number);
  A_pow << A;

  Eigen::MatrixXd A_blk(params_->prediction_horizon * C_rows_number, A_cols_number);
  for(int predict_step = 0; predict_step < params_->prediction_horizon; predict_step++){
      if(predict_step){
        A_pow *= A;
      }
    A_blk.block(predict_step * C_rows_number, 0, C_rows_number, A_cols_number) = C * A_pow;
  }
  return A_blk;
}


Eigen::MatrixXd MPCController::stackMatrixB(
  const Eigen::MatrixXd& A,
  const Eigen::MatrixXd& B,
  const Eigen::MatrixXd& C)
{
  int A_rows_number = A.rows();
  int A_cols_number = A.cols();
  int C_rows_number = C.rows();
  Eigen::MatrixXd A_pow(A_rows_number, A_cols_number);
  A_pow.setZero();

  int B_cols_number = B.cols();
  Eigen::MatrixXd B_blk(params_->prediction_horizon * C_rows_number, 
    params_->prediction_horizon * B_cols_number);
  for(int state_step = 0; state_step < params_->prediction_horizon; state_step++){
    A_pow << Eigen::MatrixXd::Identity(A_pow.rows(), A_pow.cols());
    for(int predict_step = 0; predict_step < params_->prediction_horizon - state_step; predict_step++){
      if(predict_step){
        A_pow *= A;
      }
      B_blk.block((predict_step + state_step) * C_rows_number, state_step * B_cols_number,
        C_rows_number, B_cols_number) = C * A_pow * B;
    }
  }
  return B_blk;
}


} // namespace mpc_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(mpc_controller::MPCController, nav2_core::Controller)