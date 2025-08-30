#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "mpc_controller/path_manager.hpp"
#include "mpc_controller/qp_problem_manager.hpp"
#include "mpc_controller/parameter_manager.hpp"
#include <Eigen/Core>


#ifndef SYSTEM_MODELS_HPP_
#define SYSTEM_MODELS_HPP_


class MPC{
public:
  MPC();

  void configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, 
    Parameters* params);

  void setGlobalPath(const nav_msgs::msg::Path& path);

  void updateState(
    const geometry_msgs::msg::PoseStamped& robot_pose,
    const geometry_msgs::msg::Twist& robot_velocity);
  
  geometry_msgs::msg::Twist computeControl();

  
private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  Parameters* params_;

  PathManager path_manager_;
  QPProblemManager qp_problem_manager_;

  Eigen::VectorXd X_init_;
  Eigen::VectorXd X_ref_;

  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;
  Eigen::MatrixXd C_;
  
  Eigen::MatrixXd A_stacked_;
  Eigen::MatrixXd B_stacked_;

  Eigen::VectorXd lower_bound_constraints_;
  Eigen::VectorXd upper_bound_constraints_;

  Eigen::VectorXd u_optimal_;
  Eigen::VectorXd u_last_;

  Eigen::MatrixXd stackMatrixA(
    const Eigen::MatrixXd& A, 
    const Eigen::MatrixXd& C);
  
  Eigen::MatrixXd stackMatrixB(
    const Eigen::MatrixXd& A, 
    const Eigen::MatrixXd& B, 
    const Eigen::MatrixXd& C);

  Eigen::VectorXd propagateFreeDynamics(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& X_init);

  Eigen::VectorXd stackConstraints(const Eigen::VectorXd& sonstraints);
  
};


#endif  // SYSTEM_MODELS_HPP_