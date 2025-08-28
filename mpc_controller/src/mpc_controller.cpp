#include "mpc_controller/mpc_controller.hpp"



namespace mpc_controller
{

void MPCController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
  std::string name, 
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  (void) costmap_ros;

  node_ = parent.lock();
  clock_ = node_->get_clock();
  tf_buffer_ = tf_buffer;
  plugin_name_ = name;
  
  parameters_manager_.configure(parent, name);
  params_ = parameters_manager_.get_parameters();

  mpc_.configure(parent, params_);
}

void MPCController::cleanup(){}

void MPCController::activate(){}

void MPCController::deactivate(){}

void MPCController::setSpeedLimit(const double &speed_limit, const bool &percentage){
  (void) speed_limit;
  (void) percentage;
}

void MPCController::setPlan(const nav_msgs::msg::Path& path){
  mpc_.setGlobalPath(path);
}

geometry_msgs::msg::TwistStamped MPCController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped& robot_pose,
  const geometry_msgs::msg::Twist& robot_velocity,
  nav2_core::GoalChecker* goal_checker)
{
  (void) goal_checker;

  mpc_.updateState(robot_pose, robot_velocity);

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = robot_pose.header;
  cmd_vel.twist = mpc_.computeControl();

  return cmd_vel;
}


} // namespace mpc_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(mpc_controller::MPCController, nav2_core::Controller)