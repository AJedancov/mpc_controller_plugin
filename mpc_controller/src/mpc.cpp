#include "mpc_controller/mpc.hpp"

MPC::MPC(){}

void MPC::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
  Parameters* params)
{
  node_ = parent.lock();
  clock_ = node_->get_clock();
  params_ = params;

  path_manager_.configure(parent, params_);
  qp_problem_manager_.configure(parent, params_);

  X_init_.resize(params_->nx);
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
  R_.resize(params_->nu, params_->nu);

  lower_bound_constraints_.resize(params_->nu);
  upper_bound_constraints_.resize(params_->nu);

  u_optimal_.resize(params_->prediction_horizon * params_->nu);

  u_last_.resize(params_->prediction_horizon * params_->nu);
  u_last_.setZero();
}


void MPC::setGlobalPath(const nav_msgs::msg::Path& path){
  path_manager_.setGlobalPath(path);
}


void MPC::updateState(
  const geometry_msgs::msg::PoseStamped& robot_pose,
  const geometry_msgs::msg::Twist& robot_velocity)
{
  X_ref_ = path_manager_.computeReferencePath(robot_pose);

  double yaw = tf2::getYaw(robot_pose.pose.orientation);
  X_init_ << robot_pose.pose.position.x,
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

  Q_ << Eigen::MatrixXd::Identity(params_->ny, params_->ny) * 10;
  R_ << Eigen::MatrixXd::Identity(params_->nu, params_->nu) * 0.1;

  lower_bound_constraints_ << params_->min_lin_vel, params_->min_ang_vel;
  upper_bound_constraints_ << params_->max_lin_vel, params_->max_ang_vel;
}


geometry_msgs::msg::Twist MPC::computeControl(){

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
  A_augmented.setZero();
  A_augmented.block(0, 0, A_rows_number, A_cols_number) = A_;
  A_augmented.block(0, A_cols_number, B_rows_number, B_cols_number) = B_;
  A_augmented.block(A_rows_number, A_cols_number, B_cols_number, B_cols_number) =
    Eigen::MatrixXd::Identity(B_cols_number, B_cols_number);

  Eigen::MatrixXd B_augmented(B_rows_number + B_cols_number, B_cols_number);
  B_augmented.setZero();
  B_augmented.block(0, 0, B_rows_number, B_cols_number) = B_;
  B_augmented.block(B_rows_number, 0, B_cols_number, B_cols_number) =
    Eigen::MatrixXd::Identity(B_cols_number, B_cols_number);

  int C_rows_number = C_.rows();
  int C_cols_number = C_.cols();
  Eigen::MatrixXd C_augmented(C_rows_number, C_cols_number + B_cols_number);
  C_augmented.setZero();
  C_augmented.block(0, 0, C_rows_number, C_cols_number) = C_;


  // =======================
  // === System stacking ===
  // =======================
  A_stacked_ = stackMatrixA(A_augmented, C_augmented);
  B_stacked_ = stackMatrixB(A_augmented, B_augmented, C_augmented);

  Eigen::MatrixXd Q_stacked = stackWeightMatrix(Q_);
  Eigen::MatrixXd R_stacked = stackWeightMatrix(R_);

  Eigen::VectorXd lower_bound_stacked = stackConstraints(lower_bound_constraints_);
  Eigen::VectorXd upper_bound_stacked = stackConstraints(upper_bound_constraints_);
  

  // ==================
  // === QP problem ===
  // ==================
  Eigen::VectorXd X_free = propagateFreeDynamics(A_stacked_, X_init_);
  Eigen::VectorXd state_error = X_free - X_ref_;

  qp_problem_manager_.update(
    B_stacked_, 
    state_error,
    Q_stacked,
    R_stacked,
    lower_bound_stacked - u_last_,
    upper_bound_stacked - u_last_);
  
  Eigen::VectorXd delta_u_optimal(params_->prediction_horizon * params_->nu);
  delta_u_optimal = qp_problem_manager_.solve();
  
  // To get current control from increment control: u_k = u_k-1 + delta_u
  u_optimal_ << u_last_ + delta_u_optimal;
  u_last_ = u_optimal_;

  Eigen::VectorXd X_pred(params_->prediction_horizon * params_->nx);
  X_pred << X_free + B_stacked_ * u_optimal_;
  path_manager_.publishOptimalTrajectory(X_pred);
  
  // Apply only first control input
  double lin_vel = u_optimal_[0];
  double ang_vel = u_optimal_[1];

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = lin_vel;
  cmd_vel.angular.z = ang_vel;

  return cmd_vel;
}


Eigen::MatrixXd MPC::stackMatrixA(
  const Eigen::MatrixXd& A,
  const Eigen::MatrixXd& C)
{
  int A_rows_number = A.rows();
  int A_cols_number = A.cols();
  int C_rows_number = C.rows();
  Eigen::MatrixXd A_pow(A_rows_number, A_cols_number);
  A_pow << A;

  Eigen::MatrixXd A_blk(params_->prediction_horizon * C_rows_number, A_cols_number);
  A_blk.setZero();
  for(int predict_step = 0; predict_step < params_->prediction_horizon; predict_step++){
      if(predict_step){
        A_pow *= A;
      }
    A_blk.block(predict_step * C_rows_number, 0, C_rows_number, A_cols_number) = C * A_pow;
  }
  return A_blk;
}


Eigen::MatrixXd MPC::stackMatrixB(
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
  B_blk.setZero();
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


Eigen::VectorXd MPC::propagateFreeDynamics(
  const Eigen::MatrixXd& A, 
  const Eigen::VectorXd& X_init)
{
  Eigen::VectorXd X_k(A.rows());
  X_k.setZero();
  for(int predict_step = 0; predict_step < params_->prediction_horizon; predict_step++){
    X_k.segment(predict_step * params_->ny, params_->nx) =
      A.block(predict_step * params_->ny, 0, params_->ny, params_->nx) * X_init;
  }
  return X_k;
}


Eigen::VectorXd MPC::stackConstraints(const Eigen::VectorXd& constraints){
  int constraints_rows = constraints.rows();
  Eigen::VectorXd sonstraints_blk(params_->prediction_horizon * constraints_rows);
  sonstraints_blk.setZero();
  for(int predict_step = 0; predict_step < params_->prediction_horizon; predict_step++){
    sonstraints_blk.segment(predict_step * constraints_rows, constraints_rows) << constraints;
  }
  return sonstraints_blk;
}


Eigen::MatrixXd MPC::stackWeightMatrix(const Eigen::MatrixXd& matrix){
  int matrix_rows = matrix.rows();
  int matrix_cols = matrix.cols();
  Eigen::MatrixXd matrix_blk(params_->prediction_horizon * matrix_rows, params_->prediction_horizon * matrix_cols);
  matrix_blk.setZero();
  for(int predict_step = 0; predict_step < params_->prediction_horizon; predict_step++){
    matrix_blk.block(
      predict_step * matrix_rows, predict_step * matrix_cols, 
      matrix_rows, matrix_cols) = matrix;
  }
  return matrix_blk;
}