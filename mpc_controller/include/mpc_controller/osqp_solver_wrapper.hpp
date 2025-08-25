#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <memory>
#include "osqp/osqp.h"
#include "rclcpp/rclcpp.hpp"


#ifndef OSQP_SOLVER_WRAPPER_HPP_
#define OSQP_SOLVER_WRAPPER_HPP_

namespace osqp
{

struct OSQPCscMatrixHolder{
  std::unique_ptr<OSQPCscMatrix> matrix_ptr_ = std::make_unique<OSQPCscMatrix>();
  std::vector<OSQPInt> pointers_;
  std::vector<OSQPInt> indices_;
  std::vector<OSQPFloat> values_;
};

class OSQPSolverWrapper {
public:
  OSQPSolverWrapper();

  void setup(
    Eigen::MatrixXd& H, const Eigen::VectorXd& f,
    Eigen::MatrixXd& D, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub);

  void solve(Eigen::VectorXd& u);

private:
  OSQPSolver* osqp_solver_;

  const OSQPFloat* f_osqp_;
  const OSQPFloat* lb_osqp_;
  const OSQPFloat* ub_osqp_;
  OSQPInt number_constraints_;
  OSQPInt number_variables_;
  OSQPSettings* settings_;

  OSQPCscMatrixHolder H_osqp_csc_;
  OSQPCscMatrixHolder D_osqp_csc_;
  
  void convertToOSQPCscMatrix(Eigen::MatrixXd& M, OSQPCscMatrixHolder& M_osqp_csc);

};

} // namespace osqp


#endif  //OSQP_SOLVER_WRAPPER_HPP_