#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "mpc_controller/parameter_manager.hpp"
#include "mpc_controller/osqp_solver_wrapper.hpp"
#include <Eigen/Core>


#ifndef QP_PROBLEM_MANAGER_HPP_
#define QP_PROBLEM_MANAGER_HPP_


class QPProblemManager{
public:
  QPProblemManager();

  void configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
  Parameters* params);

  void update(
  const Eigen::MatrixXd& A,
  const Eigen::MatrixXd& B,
  const Eigen::VectorXd& state_error, 
  const Eigen::VectorXd& lower_bound,
  const Eigen::VectorXd& upper_bound);

  Eigen::VectorXd solve();

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  Parameters* params_;

  // Hessian matrix
  Eigen::MatrixXd H_;

  // Linear term
  Eigen::VectorXd f_;

  // Weighting matrices
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;

  Eigen::MatrixXd Q_blk_;
  Eigen::MatrixXd R_blk_;
  
  // Constraints matrix
  Eigen::MatrixXd D_;

  Eigen::VectorXd u_optimal_;

  osqp::OSQPSolverWrapper osqp_solver_;

};
#endif  //QP_PROBLEM_MANAGER_HPP_