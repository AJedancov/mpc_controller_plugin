#include "mpc_controller/osqp_solver_wrapper.hpp"


osqp::OSQPSolverWrapper::OSQPSolverWrapper():
  f_osqp_(nullptr),
  lb_osqp_(nullptr), ub_osqp_(nullptr),
  number_constraints_(0), number_variables_(0)
{
  settings_ = OSQPSettings_new();
}


void osqp::OSQPSolverWrapper::setup(
  Eigen::MatrixXd& H, const Eigen::VectorXd& f,
  Eigen::MatrixXd& D, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub)
{ 
  convertToOSQPCscMatrix(H, H_osqp_csc_);
  convertToOSQPCscMatrix(D, D_osqp_csc_);

  f_osqp_ = f.data();
  lb_osqp_ = lb.data();
  ub_osqp_ = ub.data();
  number_constraints_ = D.rows();
  number_variables_ = H.cols();

  osqp_setup(
    &osqp_solver_,
    H_osqp_csc_.matrix_ptr_.get(), 
    f_osqp_, 
    D_osqp_csc_.matrix_ptr_.get(), 
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
}


void osqp::OSQPSolverWrapper::convertToOSQPCscMatrix(Eigen::MatrixXd& M, OSQPCscMatrixHolder& M_osqp_csc){

  M.triangularView<Eigen::StrictlyLower>().setZero();

  Eigen::SparseMatrix<double, Eigen::ColMajor> M_csc = M.sparseView();
  M_csc.makeCompressed();

  int M_rows = M.rows();
  int M_cols = M.cols(); 
  int M_csc_non_zeros = M_csc.nonZeros();

  int *outerIndexPtr = M_csc.outerIndexPtr(); // Pointers 
  int *innerIndexPtr = M_csc.innerIndexPtr(); // Row indices
  double *valuePtr = M_csc.valuePtr(); // Values

  M_osqp_csc.pointers_.reserve(M_cols + 1);
  M_osqp_csc.indices_.reserve(M_csc_non_zeros);
  M_osqp_csc.values_.reserve(M_csc_non_zeros);
  
  for(int i = 0; i < M_cols + 1; i++){
    M_osqp_csc.pointers_[i] = static_cast<OSQPInt>(outerIndexPtr[i]);
  }

  for(int i = 0; i < M_csc_non_zeros; i++){
    M_osqp_csc.indices_[i] = static_cast<OSQPInt>(innerIndexPtr[i]);
    M_osqp_csc.values_[i] = static_cast<OSQPFloat>(valuePtr[i]);
  }

  OSQPCscMatrix_set_data(
    M_osqp_csc.matrix_ptr_.get(),
    M_rows,
    M_cols,
    M_csc_non_zeros,
    M_osqp_csc.values_.data(),
    M_osqp_csc.indices_.data(),
    M_osqp_csc.pointers_.data()
  );
}