#include "mpc_controller/osqp_solver_interface.hpp"


osqp::OSQPSolverInterface::OSQPSolverInterface(){
  
}


osqp::OSQPSolverInterface::OSQPSolverInterface(const rclcpp::Logger &logger){
  logger_ = logger;
}


Eigen::MatrixXd osqp::OSQPSolverInterface::solve(
  const Eigen::MatrixXd& H, const Eigen::VectorXd& f,
  const Eigen::MatrixXd& D, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub)
{

  const OSQPCscMatrix H_osqp_csc = toOSQPCscMatrix(H);
  const OSQPFloat* f_osqp = f.data();
  const OSQPCscMatrix D_osqp = toOSQPCscMatrix(D);
  const OSQPFloat* lb_osqp = lb.data();
  const OSQPFloat* ub_osqp = ub.data();
  const OSQPInt m = D.rows();
  const OSQPInt n = H.cols();
  OSQPSettings* settings = OSQPSettings_new();

  OSQPInt exitflag = 0;
  exitflag = osqp_setup(&osqp_solver, &H_osqp_csc, f_osqp, &D_osqp, lb_osqp, ub_osqp, m, n, settings);

  RCLCPP_INFO_STREAM_ONCE(logger_, "osqp_setup exitflag: " << exitflag);
  
  // osqp_solve(osqp_solver);

  Eigen::MatrixXd u;
  return u;
}


OSQPCscMatrix osqp::OSQPSolverInterface::toOSQPCscMatrix(const Eigen::MatrixXd& M){
  
  Eigen::SparseMatrix<double, Eigen::ColMajor> M_csc = M.sparseView();
  M_csc.makeCompressed();

  int M_rows = M.rows(), M_cols = M.cols(); 
  int M_non_zeros = M.nonZeros();

  std::vector<OSQPInt> osqp_pointers;
  std::vector<OSQPInt> osqp_indices;
  std::vector<OSQPFloat> osqp_values;

  osqp_pointers.reserve(M_cols);
  osqp_indices.reserve(M_non_zeros);
  osqp_values.reserve(M_non_zeros);

  int *outerIndexPtr = M_csc.outerIndexPtr(); // Pointers 
  int *innerIndexPtr = M_csc.innerIndexPtr(); // Row indices
  double *valuePtr = M_csc.valuePtr(); // Values

  // std::stringstream sss;
  // sss << "\nPointers: "; 
  // for(int i = 0; i < M_cols; i++){
  //   sss << outerIndexPtr[i] << " ";
  // }
  
  // sss << "\nRow indices: "; 
  // for(int i = 0; i < M_non_zeros; i++){
  //   sss << innerIndexPtr[i] << " ";
  // }
  
  // sss << "\nValues: "; 
  // for(int i = 0; i < M_non_zeros; i++){
  //   sss << valuePtr[i] << " ";
  // }
  // RCLCPP_INFO_STREAM_ONCE(logger_, sss.str());
  
  for(int i = 0; i < M_cols; i++){
    osqp_pointers[i] = static_cast<OSQPInt>(outerIndexPtr[i]);
  }

  for(int i = 0; i < M_non_zeros; i++){
    osqp_indices[i] = static_cast<OSQPInt>(innerIndexPtr[i]);
    osqp_values[i] = static_cast<OSQPFloat>(valuePtr[i]);
  }

  // std::stringstream ss1;
  // ss1 << "\nOsqp_pointers: ";
  // for(int i = 0; i < M.cols(); i++){
  //    ss1 << osqp_pointers[i] << " ";
  // }

  // RCLCPP_INFO_STREAM_ONCE(logger_, ss1.str());


  OSQPCscMatrix M_osqp_csc;
  M_osqp_csc.m = M_rows;
  M_osqp_csc.n = M_cols;
  M_osqp_csc.p = osqp_pointers.data();
  M_osqp_csc.i = osqp_indices.data();
  M_osqp_csc.x = osqp_values.data();
  M_osqp_csc.nzmax = M_non_zeros;
  M_osqp_csc.nz = -1;   // -1 for csc
  M_osqp_csc.owned = 0; // 0 if owned by the user


  return M_osqp_csc;
}