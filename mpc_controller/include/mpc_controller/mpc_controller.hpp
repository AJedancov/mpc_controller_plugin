#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller.hpp"

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "mpc_controller/parameter_manager.hpp"
#include "mpc_controller/path_manager.hpp"
#include "mpc_controller/qp_problem_manager.hpp"
#include <Eigen/Core>

#ifndef MPC_CONTROLLER_HPP_
#define MPC_CONTROLLER_HPP_

namespace mpc_controller
{

class MPCController: public nav2_core::Controller
{
public:
  // MPCController() = default;
  ~MPCController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
  ) override;


  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_velocity,
    nav2_core::GoalChecker * goal_checker
  ) override;


private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_ = rclcpp::get_logger("MPCController");


  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  PathManager path_manager_;
  ParameterManager parameters_manager_;
  QPProblemManager qp_problem_manager_;
  Parameters* params_;

  Eigen::VectorXd X_init_;
  Eigen::VectorXd X_ref_;

  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;
  Eigen::MatrixXd C_;
  
  Eigen::MatrixXd A_stacked_;
  Eigen::MatrixXd B_stacked_;

  Eigen::MatrixXd stackMatrixA(
    const Eigen::MatrixXd& A, 
    const Eigen::MatrixXd& C);
  
  Eigen::MatrixXd stackMatrixB(
    const Eigen::MatrixXd& A, 
    const Eigen::MatrixXd& B, 
    const Eigen::MatrixXd& C);
};

} // namespace mpc_controller


#endif  // MPC_CONTROLLER_HPP_