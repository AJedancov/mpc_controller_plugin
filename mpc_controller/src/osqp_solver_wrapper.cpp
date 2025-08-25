#include "mpc_controller/osqp_solver_wrapper.hpp"


osqp::OSQPSolverWrapper::OSQPSolverWrapper(){}


osqp::OSQPSolverWrapper::OSQPSolverWrapper(const rclcpp::Logger &logger){
  logger_ = logger;
}


void osqp::OSQPSolverWrapper::solve(
  Eigen::MatrixXd& H, const Eigen::VectorXd& f,
  Eigen::MatrixXd& D, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, 
  Eigen::VectorXd& u)
{ 

  OSQPCscMatrixHolder H_osqp_csc;
  OSQPCscMatrixHolder D_osqp_csc;

  convert_to_osqp_csc_matrix(H, H_osqp_csc);
  convert_to_osqp_csc_matrix(D, D_osqp_csc);

  const OSQPFloat* f_osqp = f.data();
  const OSQPFloat* lb_osqp = lb.data();
  const OSQPFloat* ub_osqp = ub.data();
  const OSQPInt m = D.rows();
  const OSQPInt n = H.cols();
  OSQPSettings* settings = OSQPSettings_new();

  OSQPInt exitflag = 0;
  exitflag = osqp_setup(
    &osqp_solver_,
    H_osqp_csc.matrix_ptr_.get(), 
    f_osqp, 
    D_osqp_csc.matrix_ptr_.get(), 
    lb_osqp, 
    ub_osqp, 
    m, 
    n, 
    settings
  );

  // RCLCPP_INFO_STREAM_ONCE(logger_, "osqp_setup exitflag: " << exitflag);
  
  if(!exitflag){
    exitflag = osqp_solve(osqp_solver_);
  }
  // RCLCPP_INFO_STREAM_ONCE(logger_, "osqp_solve exitflag: " << exitflag);

  for(int i = 0; i < f.rows(); i++){
    u[i] = osqp_solver_->solution->x[i];
  }
}


void osqp::OSQPSolverWrapper::convert_to_osqp_csc_matrix(Eigen::MatrixXd& M, OSQPCscMatrixHolder& M_osqp_csc){

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