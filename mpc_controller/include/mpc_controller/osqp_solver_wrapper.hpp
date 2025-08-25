#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <memory>
#include "osqp/osqp.h"
#include "rclcpp/rclcpp.hpp"


#ifndef OSQP_SOLVER_WRAPPER_HPP_
#define OSQP_SOLVER_WRAPPER_HPP_

namespace osqp
{

struct OSQPCscMatrixHolder {
  std::unique_ptr<OSQPCscMatrix> matrix_ptr_ = std::make_unique<OSQPCscMatrix>();
  std::vector<OSQPInt> pointers_;
  std::vector<OSQPInt> indices_;
  std::vector<OSQPFloat> values_;
};

class OSQPSolverWrapper {
public:
  OSQPSolverWrapper();
  OSQPSolverWrapper(const rclcpp::Logger &logger);

  void solve(
    Eigen::MatrixXd& H, const Eigen::VectorXd& f,
    Eigen::MatrixXd& D, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
    Eigen::VectorXd& u);

private:
  void convert_to_osqp_csc_matrix(Eigen::MatrixXd& M, OSQPCscMatrixHolder& M_osqp_csc);
  OSQPSolver* osqp_solver_;
  rclcpp::Logger logger_ = rclcpp::get_logger("");

};

} // namespace osqp


#endif  //OSQP_SOLVER_WRAPPER_HPP_