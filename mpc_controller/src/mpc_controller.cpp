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
  B_blk_.resize(prediction_horizon_ * nx_, prediction_horizon_ * nu_);
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

  // Obtain reference states
  // vector x_ref -> from path depending on horizon
  // std::vector<geometry_msgs::msg::PoseStamped> X_ref(
  //   global_plan_.poses.begin(),
  //   global_plan_.poses.begin() + prediction_horizon_
  // );


  // Transform Quaternion into RPY
  tf2::Quaternion q;
  tf2::fromMsg(robot_pose.pose.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  double dt = 0.05; // sampling time

  // Define system dynamic
  double a13 = -params_.max_lin_vel * std::sin(yaw) * dt;
  double a23 = params_.max_lin_vel * std::cos(yaw) * dt;

  A_ << 1, 0, a13,
        0, 1, a23,
        0, 0, 1;

  double b11 = std::cos(yaw) * dt;
  double b21 = std::sin(yaw) * dt;

  B_ << b11, 0,
        b21, 0,
        0 , dt;

  // =======================
  // === System stacking ===
  // =======================

  // Stacking A matrix
  Eigen::MatrixXd A_pow(A_.rows(), A_.cols());
  A_pow << A_;

  A_blk_.setZero();

  for(int i = 0; i < prediction_horizon_; i++){
    if(i) A_pow *= A_;
    A_blk_.block(i * ny_, 0, ny_, nx_) = C_ * A_pow;
  }

  RCLCPP_INFO_STREAM_ONCE(logger_, "Block Matrix A: \n" << A_blk_);

  // Stacking B matrix
  A_pow.setZero();
  B_blk_.setZero();

  for(int i = 0; i < prediction_horizon_; i++){
    
    A_pow << Eigen::MatrixXd::Identity(ny_, nx_);
    for(int j = 0; j < prediction_horizon_ - i; j++){
      
      if(j) A_pow *= A_;
      B_blk_.block((j + i) * nx_, i * nu_, nx_, nu_) = C_ * A_pow * B_;
    }
  }

  RCLCPP_INFO_STREAM_ONCE(logger_, "Block Matrix B: \n" << B_blk_);
 

  double linear_vel, angular_vel;

  linear_vel = 0.1;
  angular_vel = 0.0;
  
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