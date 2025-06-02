#include "mpc_controller/osqp_solver_interface.hpp"


osqp::OSQPSolverInterface::OSQPSolverInterface(){
  
}

Eigen::MatrixXd osqp::OSQPSolverInterface::solve(
  const Eigen::MatrixXd& H, const Eigen::VectorXd& f,
  const Eigen::VectorXd& lb, const Eigen::VectorXd& ub)
{

  const OSQPCscMatrix H_osqp_csc = toOSQPCscMatrix(H);
  // const OSQPFloat* f_osqp = f.data();
  // const OSQPCscMatrix* A_osqp = nullptr;
  // const OSQPFloat* lb_osqp = nullptr;
  // const OSQPFloat* ub_osqp = nullptr;
  // const OSQPInt m = 1;
  // const OSQPInt n = 2;
  // OSQPSettings* settings = OSQPSettings_new();

  // osqp_set_default_settings(settings);

  OSQPInt exitflag = 0;
  // exitflag = osqp_setup(&osqp_solver, &H_osqp_csc, f_osqp, A_osqp, lb_osqp, ub_osqp, m, n, settings);
  // osqp_solve(osqp_solver);

  Eigen::MatrixXd u;
  return u;
}


OSQPCscMatrix osqp::OSQPSolverInterface::toOSQPCscMatrix(const Eigen::MatrixXd& M){
  
  Eigen::SparseMatrix<double, Eigen::ColMajor> M_csc = M.sparseView();
  M_csc.makeCompressed();

  std::vector<OSQPInt> osqp_pointers;
  std::vector<OSQPInt> osqp_indices;
  std::vector<OSQPFloat> osqp_values;

  // int *outerIndexPtr = M_csc.outerIndexPtr(); // Pointers 
  // int *innerIndexPtr = M_csc.innerIndexPtr(); // Row indices
  // double *valuePtr = M_csc.valuePtr(); // Values

  // for(int i; i < M.cols(); i++){
  //   osqp_pointers[i] = static_cast<OSQPInt>(outerIndexPtr[i]);
  //   osqp_indices[i] = static_cast<OSQPInt>(innerIndexPtr[i]);
  //   osqp_values[i] = static_cast<OSQPInt>(valuePtr[i]);
  // }

  OSQPCscMatrix M_osqp_csc;
  // M_osqp_csc.m = M_csc.rows();
  // M_osqp_csc.n = M_csc.cols();
  // M_osqp_csc.p = osqp_pointers.data();
  // M_osqp_csc.i = osqp_indices.data();
  // M_osqp_csc.x = osqp_values.data();
  // M_osqp_csc.nzmax = M_csc.nonZeros();
  // M_osqp_csc.nz = -1;   // -1 for csc
  // M_osqp_csc.owned = 0; // 0 if owned by the user


  return M_osqp_csc;
}