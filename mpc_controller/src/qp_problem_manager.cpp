#include "mpc_controller/qp_problem_manager.hpp"

QPProblemManager::QPProblemManager(){}

void QPProblemManager::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, 
  Parameters *params)
{
  node_ = parent.lock();
  clock_ = node_->get_clock();
  params_ = params;

  Q_.resize(params_->ny, params_->ny);
  Q_ << Eigen::MatrixXd::Identity(params_->ny, params_->ny) * 10;

  R_.resize(params_->nu, params_->nu);
  R_ << Eigen::MatrixXd::Identity(params_->nu, params_->nu) * 0.1;

  Q_blk_.resize(params_->prediction_horizon * Q_.rows(), params_->prediction_horizon * Q_.cols());
  R_blk_.resize(params_->prediction_horizon * R_.rows(), params_->prediction_horizon * R_.cols());

  H_.resize(params_->prediction_horizon * params_->nu, params_->prediction_horizon * params_->nu);
  f_.resize(params_->prediction_horizon * params_->nu);
  D_.resize(params_->prediction_horizon * params_->nu, params_->prediction_horizon * params_->nu);

  lower_bound_.resize(params_->prediction_horizon * params_->nu);
  upper_bound_.resize(params_->prediction_horizon * params_->nu);
  u_optimal_.resize(params_->prediction_horizon * params_->nu);
}

void QPProblemManager::update(
  Eigen::MatrixXd A, 
  Eigen::MatrixXd B,  
  Eigen::VectorXd X_init, 
  Eigen::MatrixXd X_ref)
{

  // Prepare matrices for the QP problem of the form:
  // J = 0.5 * uHu^T + f^Tu
  // Subject to:
  // lower_bound <= Du <= upper_bound

  Eigen::VectorXd AX_init(A.rows());
  AX_init.setZero();
  for(int predict_step = 0; predict_step < params_->prediction_horizon; predict_step++){
    AX_init.segment(predict_step * params_->ny, params_->nx) =
      A.block(predict_step * params_->ny, 0, params_->ny, params_->nx) * X_init;
  }

  Q_blk_.setZero();
  R_blk_.setZero();
  for(int predict_step = 0; predict_step < params_->prediction_horizon; predict_step++){
    Q_blk_.block(predict_step * params_->nx, predict_step * params_->nx, params_->nx, params_->nx) = Q_;
    R_blk_.block(predict_step * params_->nu, predict_step * params_->nu, params_->nu, params_->nu) = R_;
  }

  H_ = 2 * (B.transpose() * Q_blk_ * B + R_blk_);
  f_ = 2 * B.transpose() * Q_blk_ * (AX_init - X_ref);

  int B_cols = B.cols();
  D_ << Eigen::MatrixXd::Identity(B_cols, B_cols);

  lower_bound_.setZero();
  upper_bound_.setZero();
  for(int predict_step = 0; predict_step < params_->prediction_horizon; predict_step++){
    lower_bound_.segment(predict_step * params_->nu, params_->nu) << params_->min_lin_vel, params_->min_ang_vel;
    upper_bound_.segment(predict_step * params_->nu, params_->nu) << params_->max_lin_vel, params_->max_ang_vel;
  }

  osqp_solver_.setup(H_, f_, D_, lower_bound_, upper_bound_);
}

Eigen::VectorXd QPProblemManager::solve()
{
  osqp_solver_.solve(u_optimal_);
  return u_optimal_;
}
