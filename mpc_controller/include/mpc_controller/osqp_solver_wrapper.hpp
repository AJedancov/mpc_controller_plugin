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
  ~OSQPCscMatrixHolder(){
    OSQPCscMatrix_free(matrix_ptr_);
  }

  OSQPCscMatrix* matrix_ptr_ = nullptr;
  std::vector<OSQPInt> pointers_;
  std::vector<OSQPInt> indices_;
  std::vector<OSQPFloat> values_;
};

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
  const OSQPFloat* f_osqp_;

  OSQPCscMatrix* D_osqp_csc_;
  const OSQPFloat* lb_osqp_;
  const OSQPFloat* ub_osqp_;
  OSQPInt number_constraints_;
  OSQPInt number_variables_;
};

} // namespace osqp


#endif  //OSQP_SOLVER_WRAPPER_HPP_