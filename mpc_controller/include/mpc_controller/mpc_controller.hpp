#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.h"

#include "mpc_controller/osqp_solver_interface.hpp"
#include <Eigen/Core>

#ifndef MPC_CONTROLLER_HPP_
#define MPC_CONTROLLER_HPP_

namespace mpc_controller
{

struct Parameters{
  double max_lin_vel;
  double min_lin_vel;
  double max_ang_vel;
  double min_ang_vel;
  std::string local_frame;
};

class MPCController: public nav2_core::Controller
{
public:
  MPCController() = default;
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


protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_ = rclcpp::get_logger("MPCController");


  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  nav_msgs::msg::Path global_path_;

  int prediction_horizon_;
  int control_horizon_;

  int nx_; //state dimension
  int nu_; //control input dimension
  int ny_; //output dimension

  Eigen::VectorXd x_k_;
  Eigen::VectorXd X_ref_;

  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;
  Eigen::MatrixXd C_;
  
  Eigen::MatrixXd A_blk_;
  Eigen::MatrixXd B_blk_;

  // Weighting matrices
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;

  Eigen::MatrixXd Q_blk_;
  Eigen::MatrixXd R_blk_;

  double dt = 0.05; // sampling time
  double linear_vel = 0.5;
  double angular_vel = 1;
  
  // Node parameters
  Parameters params_;

private:

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
  rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter> &params){
  
    rcl_interfaces::msg::SetParametersResult result;

    for(auto &param : params){
      if (param.get_name() == plugin_name_ + ".max_lin_vel"){
        params_.max_lin_vel = param.as_double();
      }else if (param.get_name() == plugin_name_ + ".min_lin_vel"){
        params_.min_lin_vel = param.as_double();
      }else if (param.get_name() == plugin_name_ + ".max_ang_vel"){
        params_.max_ang_vel = param.as_double();
      }else if (param.get_name() == plugin_name_ + ".min_ang_vel"){
        params_.min_ang_vel = param.as_double();
      }else if (param.get_name() == plugin_name_ + ".local_frame"){
        params_.local_frame = param.as_string();
      }
    }

    result.successful = true;
    return result;
  }

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PointStamped>> closest_waypoint_publisher_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> lerp_ref_path_publisher_;

};

} // namespace mpc_controller


#endif  // MPC_CONTROLLER_HPP_