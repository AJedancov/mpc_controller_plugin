#include "mpc_controller/mpc_controller.hpp"



namespace mpc_controller
{


void MPCController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, 
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  tf_buffer_ = tf_buffer;
  plugin_name_ = name;
  (void) costmap_ros;
  
  clock_ = node->get_clock();

  params_callback_handle_ = node->add_on_set_parameters_callback(
    std::bind(&MPCController::paramsCallback, this, std::placeholders::_1)
  );

  node->declare_parameter(plugin_name_ + ".max_lin_vel", rclcpp::ParameterValue(0.5));
  node->declare_parameter(plugin_name_ + ".min_lin_vel", rclcpp::ParameterValue(0.5));
  node->declare_parameter(plugin_name_ + ".max_ang_vel", rclcpp::ParameterValue(-0.5));
  node->declare_parameter(plugin_name_ + ".min_ang_vel", rclcpp::ParameterValue(-0.5));
  node->declare_parameter(plugin_name_ + ".local_frame", rclcpp::ParameterValue(std::string("odom")));

  node->get_parameter(plugin_name_ + ".max_lin_vel", params_.max_lin_vel);
  node->get_parameter(plugin_name_ + ".min_lin_vel", params_.min_lin_vel);
  node->get_parameter(plugin_name_ + ".max_ang_vel", params_.max_ang_vel);
  node->get_parameter(plugin_name_ + ".min_ang_vel", params_.min_ang_vel);
  node->get_parameter(plugin_name_ + ".local_frame", params_.local_frame);

  prediction_horizon_ = 5;

  nx_ = 3; //state dimension
  nu_ = 2; //control input dimension
  ny_ = 3; //output dimension

  // Set matrix dimensions
  A_.resize(nx_, nx_);
  B_.resize(nx_, nu_);
  C_.resize(ny_, nx_);
  C_ << Eigen::MatrixXd::Identity(ny_, nx_);

  A_blk_.resize(prediction_horizon_ * nx_, nx_);
  B_blk_.resize(prediction_horizon_ * nx_, prediction_horizon_ * nx_);
}

void MPCController::cleanup(){}

void MPCController::activate(){}

void MPCController::deactivate(){}

void MPCController::setSpeedLimit(const double &speed_limit, const bool &percentage){
  (void) speed_limit;
  (void) percentage;
}

void MPCController::setPlan(const nav_msgs::msg::Path& path){
  global_plan_ = path;
}

geometry_msgs::msg::TwistStamped MPCController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped& robot_pose,
  const geometry_msgs::msg::Twist& robot_velocity,
  nav2_core::GoalChecker* goal_checker)
{
  (void) robot_velocity;
  (void) goal_checker;
 

  double linear_vel, angular_vel;

  linear_vel = 0.1;
  angular_vel = 0.0;
  
  // Send calculated speed to nav2 controller_server
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = robot_pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;

  return cmd_vel;
}


} // namespace mpc_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(mpc_controller::MPCController, nav2_core::Controller)