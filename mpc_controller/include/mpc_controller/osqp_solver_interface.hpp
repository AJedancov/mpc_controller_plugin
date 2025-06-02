#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <memory>
#include "osqp/osqp.h"
#include "rclcpp/rclcpp.hpp"


#ifndef OSQP_SOLVER_INTERFACE_HPP_
#define OSQP_SOLVER_INTERFACE_HPP_

namespace osqp
{

class OSQPSolverInterface {
public:
  OSQPSolverInterface();

  Eigen::MatrixXd solve(
    const Eigen::MatrixXd& H, const Eigen::VectorXd& f,
    const Eigen::VectorXd& lb, const Eigen::VectorXd& ub);

private:
  
  OSQPCscMatrix toOSQPCscMatrix(const Eigen::MatrixXd& M);
  OSQPSolver* osqp_solver;
  rclcpp::Logger logger_ {rclcpp::get_logger("DEBUG")};

};

} // namespace osqp



#endif  //OSQP_SOLVER_INTERFACE_HPP_