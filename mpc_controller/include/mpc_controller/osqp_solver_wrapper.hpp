#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <memory>
#include "osqp/osqp.h"
#include "rclcpp/rclcpp.hpp"


#ifndef OSQP_SOLVER_WRAPPER_HPP_
#define OSQP_SOLVER_WRAPPER_HPP_

namespace osqp
{

class OSQPSolverWrapper {
public:
  OSQPSolverWrapper();
  ~OSQPSolverWrapper();

  void setup(
    Eigen::MatrixXd& H, const Eigen::VectorXd& f,
    Eigen::MatrixXd& D, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub);

  void solve(Eigen::VectorXd& u);

private:
  OSQPSolver* osqp_solver_;
  OSQPSettings* settings_;

  OSQPCscMatrix* H_osqp_csc_;
  OSQPCscMatrix* D_osqp_csc_;
  Eigen::SparseMatrix<OSQPFloat, Eigen::ColMajor, OSQPInt> H_csc_;
  Eigen::SparseMatrix<OSQPFloat, Eigen::ColMajor, OSQPInt> D_csc_;
  
  const OSQPFloat* f_osqp_;

  const OSQPFloat* lb_osqp_;
  const OSQPFloat* ub_osqp_;
  OSQPInt number_constraints_;
  OSQPInt number_variables_;
};

} // namespace osqp


#endif  //OSQP_SOLVER_WRAPPER_HPP_