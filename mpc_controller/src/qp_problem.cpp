#include "mpc_controller/qp_problem.hpp"

QPProblem::QPProblem(){}


void QPProblem::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, 
  Parameters *params)
{
  node_ = parent.lock();
  clock_ = node_->get_clock();
  params_ = params;

  H_.resize(params_->prediction_horizon * params_->nu, params_->prediction_horizon * params_->nu);
  f_.resize(params_->prediction_horizon * params_->nu);
  D_.resize(params_->prediction_horizon * params_->nu, params_->prediction_horizon * params_->nu);

  u_optimal_.resize(params_->prediction_horizon * params_->nu);
}

void QPProblem::update(
  const Eigen::MatrixXd& B,
  const Eigen::VectorXd& state_error, 
  const Eigen::MatrixXd& Q,
  const Eigen::MatrixXd& R,
  const Eigen::VectorXd& lower_bound,
  const Eigen::VectorXd& upper_bound)
{

  // Prepare matrices for the QP problem of the form:
  // J = 0.5 * uHu^T + f^Tu
  // Subject to:
  // lower_bound <= Du <= upper_bound

  H_ = 2 * (B.transpose() * Q * B + R);
  f_ = 2 * B.transpose() * Q * state_error;

  int B_cols = B.cols();
  D_.setIdentity(B_cols, B_cols);

  osqp_solver_.setup(H_, f_, D_, lower_bound, upper_bound);
}


Eigen::VectorXd QPProblem::solve()
{
  osqp_solver_.solve(u_optimal_);
  return u_optimal_;
}