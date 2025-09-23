#include "mpc_controller/osqp_solver_wrapper.hpp"


osqp::OSQPSolverWrapper::OSQPSolverWrapper():
  H_osqp_csc_(nullptr), f_osqp_(nullptr),
  D_osqp_csc_(nullptr), lb_osqp_(nullptr), ub_osqp_(nullptr),
  number_constraints_(0), number_variables_(0)
{
  settings_ = OSQPSettings_new();
}


osqp::OSQPSolverWrapper::~OSQPSolverWrapper(){
  osqp_cleanup(osqp_solver_);
  OSQPSettings_free(settings_);
}


void osqp::OSQPSolverWrapper::setup(
  Eigen::MatrixXd& H, const Eigen::VectorXd& f,
  Eigen::MatrixXd& D, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub)
{
  H.triangularView<Eigen::StrictlyLower>().setZero();
  H_csc_ = H.sparseView();
  H_osqp_csc_ = OSQPCscMatrix_new(
    H.rows(),
    H.cols(),
    H_csc_.nonZeros(),
    H_csc_.valuePtr(),      // Values
    H_csc_.innerIndexPtr(), // Row indices
    H_csc_.outerIndexPtr()  // Pointers 
  );
  
  D.triangularView<Eigen::StrictlyLower>().setZero();
  D_csc_ = D.sparseView();
  D_osqp_csc_ = OSQPCscMatrix_new(
    D.rows(),
    D.cols(),
    D_csc_.nonZeros(),
    D_csc_.valuePtr(),
    D_csc_.innerIndexPtr(),
    D_csc_.outerIndexPtr()
  );

  f_osqp_ = f.data();
  lb_osqp_ = lb.data();
  ub_osqp_ = ub.data();
  number_constraints_ = D.rows();
  number_variables_ = H.cols();

  osqp_setup(
    &osqp_solver_,
    H_osqp_csc_, 
    f_osqp_, 
    D_osqp_csc_, 
    lb_osqp_, 
    ub_osqp_, 
    number_constraints_,
    number_variables_, 
    settings_);
}


void osqp::OSQPSolverWrapper::solve(Eigen::VectorXd& u){
  osqp_solve(osqp_solver_);
  for(int i = 0; i < number_variables_; i++){
    u[i] = osqp_solver_->solution->x[i];
  }
  OSQPCscMatrix_free(H_osqp_csc_);
  OSQPCscMatrix_free(D_osqp_csc_);
}