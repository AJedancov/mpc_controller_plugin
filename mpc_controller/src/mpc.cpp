#include "mpc_controller/mpc.hpp"

MPC::MPC(){}

void MPC::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
  Parameters* params)
{
  node_ = parent.lock();
  clock_ = node_->get_clock();
  params_ = params;

  qp_problem_.configure(parent, params_);

  X_init_.resize(params_->nx);
  X_ref_.resize(params_->prediction_horizon * params_->nx);
  X_pred_.resize(params_->prediction_horizon * params_->nx);

  // Set matrix dimensions
  A_.resize(params_->nx, params_->nx);
  B_.resize(params_->nx, params_->nu);
  C_.resize(params_->ny, params_->nx);

  A_stacked_.resize(params_->prediction_horizon * params_->ny, params_->nx);
  B_stacked_.resize(params_->prediction_horizon * params_->ny, 
    params_->prediction_horizon * params_->nu);
  
  Q_.resize(params_->ny, params_->ny);
  Q_term_.resize(params_->ny, params_->ny);
  R_.resize(params_->nu, params_->nu);

  lower_bound_constraints_.resize(params_->nu);
  upper_bound_constraints_.resize(params_->nu);

  u_optimal_.resize(params_->prediction_horizon * params_->nu);

  u_last_.resize(params_->prediction_horizon * params_->nu);
  u_last_.setZero();
}


void MPC::setReferencePath(const Eigen::VectorXd& X_ref){
  X_ref_ = X_ref;
}


void MPC::updateState(
  const geometry_msgs::msg::PoseStamped& robot_pose,
  const geometry_msgs::msg::Twist& robot_velocity)
{

  X_init_.resize(params_->nx);
  double yaw = tf2::getYaw(robot_pose.pose.orientation);
  X_init_ << robot_pose.pose.position.x,
             robot_pose.pose.position.y,
             yaw;

  // Define system dynamic
  A_.resize(params_->nx, params_->nx);
  double a13 = -1 * std::sin(yaw) * params_->dt;
  double a23 = std::cos(yaw) * params_->dt;
  A_ << 1, 0, a13,
        0, 1, a23,
        0, 0, 1;

  B_.resize(params_->nx, params_->nu);
  double b11 = std::cos(yaw) * params_->dt;
  double b21 = std::sin(yaw) * params_->dt;
  B_ << b11, 0,
        b21, 0,
        0, params_->dt;
  
  C_.setIdentity(params_->ny, params_->nx);

  std::vector<double> q = params_->state_weights_diag;
  std::vector<double> q_term = params_->state_weights_term_diag;
  std::vector<double> r = params_->control_weights_diag;
  Q_ = Eigen::Map<Eigen::VectorXd>(q.data(), q.size()).asDiagonal();
  Q_term_ = Eigen::Map<Eigen::VectorXd>(q_term.data(), q_term.size()).asDiagonal();
  R_ = Eigen::Map<Eigen::VectorXd>(r.data(), r.size()).asDiagonal();

  lower_bound_constraints_ << params_->min_lin_vel, params_->min_ang_vel;
  upper_bound_constraints_ << params_->max_lin_vel, params_->max_ang_vel;

  if(params_->use_input_increment){
    // === Input increment model ===
    // [x_k+1] = [A B]*[x_k  ]+[B]*delta_u
    // [u_k]     [0 I] [u_k-1] [I]
    // 
    // y_k = [C 0]*[x_k  ]
    //             [u_K-1] 

    int A_rows = A_.rows();
    int A_cols = A_.cols();
    int B_rows_number = B_.rows();
    int B_cols = B_.cols();
    Eigen::MatrixXd A_augmented(A_rows + B_cols, A_cols + B_cols);
    A_augmented.setZero();
    A_augmented.block(0, 0, A_rows, A_cols) = A_;
    A_augmented.block(0, A_cols, B_rows_number, B_cols) = B_;
    A_augmented.block(A_rows, A_cols, B_cols, B_cols).setIdentity();

    Eigen::MatrixXd B_augmented(B_rows_number + B_cols, B_cols);
    B_augmented.setZero();
    B_augmented.block(0, 0, B_rows_number, B_cols) = B_;
    B_augmented.block(B_rows_number, 0, B_cols, B_cols).setIdentity();

    int C_rows = C_.rows();
    int C_cols = C_.cols();
    Eigen::MatrixXd C_augmented(C_rows, C_cols + B_cols);
    C_augmented.setZero();
    C_augmented.block(0, 0, C_rows, C_cols) = C_;

    Eigen::VectorXd X_augmented(params_->nx + params_->nu);
    X_augmented.segment(0, params_->nx) = X_init_;
    X_augmented.segment(params_->nx, params_->nu) << u_last_[0], u_last_[1];

    A_ = A_augmented;
    B_ = B_augmented;
    C_ = C_augmented;
    X_init_ = X_augmented;
  }
}


geometry_msgs::msg::Twist MPC::computeControl(){

  // =======================
  // === System stacking ===
  // =======================
  A_stacked_ = stackMatrixA(A_, C_, params_->prediction_horizon);
  B_stacked_ = stackMatrixB(A_, B_, C_, params_->prediction_horizon);

  Eigen::MatrixXd Q_stacked = stackWeightMatrix(Q_, params_->prediction_horizon);
  Eigen::MatrixXd R_stacked = stackWeightMatrix(R_, params_->prediction_horizon);

  int predict_step = params_->prediction_horizon - 1;
  Q_stacked.block(
    predict_step * Q_.rows(), predict_step * Q_.cols(), 
    Q_.rows(), Q_.cols()) = Q_term_;


  Eigen::VectorXd lower_bound_stacked = lower_bound_constraints_.replicate(params_->prediction_horizon, 1);
  Eigen::VectorXd upper_bound_stacked = upper_bound_constraints_.replicate(params_->prediction_horizon, 1);
  
  if(params_->use_input_increment){
    lower_bound_stacked -= u_last_;
    upper_bound_stacked -= u_last_;
  }
 
  // ==================
  // === QP problem ===
  // ==================
  Eigen::VectorXd state_error = A_stacked_ * X_init_ - X_ref_;
  int state_ordinal_number = 2;
  normalizeAngles(state_error, state_ordinal_number, params_->prediction_horizon);

  qp_problem_.update(
    B_stacked_, 
    state_error,
    Q_stacked,
    R_stacked,
    lower_bound_stacked,
    upper_bound_stacked);
  
  Eigen::VectorXd qp_optimal_solution = qp_problem_.solve();
  
  if(params_->use_input_increment){
    // To get current control u_k from control increment delta_u: u_k = u_k-1 + delta_u
    u_optimal_ = u_last_ + qp_optimal_solution;
    u_last_ = u_optimal_;
  }
  else{
    u_optimal_ = qp_optimal_solution;
  }

  X_pred_ = propagateSystemState(A_stacked_, B_stacked_, X_init_, u_optimal_);

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
  const Eigen::MatrixXd& C, 
  const int& horizon)
{
  int A_rows = A.rows();
  int A_cols = A.cols();
  int C_rows = C.rows();
  Eigen::MatrixXd A_pow(A_rows, A_cols);
  A_pow << A;

  Eigen::MatrixXd A_blk(horizon * C_rows, A_cols);
  A_blk.setZero();
  for(int predict_step = 0; predict_step < horizon; predict_step++){
      A_blk.block(
        predict_step * C_rows, 0,
        C_rows, A_cols) = C * A_pow;
      A_pow *= A;
  }
  return A_blk;
}


Eigen::MatrixXd MPC::stackMatrixB(
  const Eigen::MatrixXd& A,
  const Eigen::MatrixXd& B,
  const Eigen::MatrixXd& C, 
  const int& horizon)
{
  int A_rows = A.rows();
  int A_cols = A.cols();
  int C_rows = C.rows();
  Eigen::MatrixXd A_pow(A_rows, A_cols);
  A_pow.setZero();

  int B_cols = B.cols();
  Eigen::MatrixXd B_blk(
    horizon * C_rows, 
    horizon * B_cols);
  B_blk.setZero();
  for(int state_step = 0; state_step < horizon; state_step++){
    A_pow.setIdentity(A_rows, A_cols);
    for(int predict_step = 0; predict_step < horizon - state_step; predict_step++){
      B_blk.block(
        (predict_step + state_step) * C_rows, state_step * B_cols,
        C_rows, B_cols) = C * A_pow * B;
      A_pow *= A;
    }
  }
  return B_blk;
}


Eigen::MatrixXd MPC::stackWeightMatrix(
  const Eigen::MatrixXd& matrix, 
  const int& horizon)
{
  int matrix_rows = matrix.rows();
  int matrix_cols = matrix.cols();
  Eigen::MatrixXd matrix_blk(horizon * matrix_rows, horizon * matrix_cols);
  matrix_blk.setZero();
  for(int predict_step = 0; predict_step < horizon; predict_step++){
    matrix_blk.block(
      predict_step * matrix_rows, predict_step * matrix_cols, 
      matrix_rows, matrix_cols) = matrix;
  }
  return matrix_blk;
}


Eigen::VectorXd MPC::propagateSystemState(
  const Eigen::MatrixXd& A,
  const Eigen::MatrixXd& B,
  const Eigen::VectorXd& x,
  const Eigen::VectorXd& u)
{
  return A * x + B * u;
}


Eigen::VectorXd MPC::getOptimalTrajectory(){
  return X_pred_;
}


void MPC::normalizeAngles(
  Eigen::VectorXd& state_vector, 
  const int& state_ordinal_number,
  const int& horizon)
{
  int state_vector_size = state_vector.rows() / horizon;
  for(int predict_step = 0; predict_step < horizon; predict_step++){
    double& angle = state_vector[predict_step * state_vector_size + state_ordinal_number];
    if(angle > M_PI){
      angle -= 2 * M_PI;
    }  
    if(angle < -M_PI){
      angle += 2 * M_PI;
    }
  }
}