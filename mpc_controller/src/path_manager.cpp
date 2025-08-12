#include "mpc_controller/path_manager.hpp"

PathManager::PathManager(){}

void PathManager::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
  Parameters* params)
{
  node_ = parent.lock();
  clock_ = node_->get_clock();
  params_ = params;

  projection_point_publisher_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(
    "closest_point",
    10);
  lerp_ref_path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
    "reference_path", 
    10);
  optimal_trajectory_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
    "optimal_trajectory", 
    10);
}


void PathManager::setGlobalPath(const nav_msgs::msg::Path &global_path){
  global_path_ = global_path;
}


Eigen::VectorXd PathManager::computeReferencePath(
  const geometry_msgs::msg::PoseStamped& robot_pose)
{
  waypoints_num_ = global_path_.poses.size();
  
  int ref_wp_idx = findReferenceWaypointIndex(robot_pose);

  geometry_msgs::msg::PointStamped projection_point;
  projection_point = findProjectionPoint(robot_pose, ref_wp_idx);
  projection_point_publisher_->publish(projection_point);

  std::vector<double> s_path_cum;
  s_path_cum.push_back(0.0);
  for(int i = ref_wp_idx; i < waypoints_num_ - 1; i++){
    double dx = global_path_.poses[i + 1].pose.position.x - global_path_.poses[i].pose.position.x;
    double dy = global_path_.poses[i + 1].pose.position.y - global_path_.poses[i].pose.position.y;
    s_path_cum.push_back(s_path_cum.back() + std::hypot(dx, dy));
  }
  
  double s_predict = params_->max_lin_vel * params_->dt;
  double s_predict_total = s_predict * params_->prediction_horizon;
  double s_path_total = s_path_cum.back();

  if(s_predict_total > s_path_total){
    s_predict = s_path_total / params_->prediction_horizon;
  }

  // TODO: Replace on tsd::begin(), std::end and std::partial_sum()
  std::vector<double> s_predict_cum;
  s_predict_cum.push_back(0.0);
  for(int i = 0; i < params_->prediction_horizon; i++){
    s_predict_cum.push_back(s_predict_cum.back() + s_predict);
  }

  nav_msgs::msg::Path lerp_ref_path;
  lerp_ref_path.header.frame_id = global_path_.header.frame_id;
  lerp_ref_path.header.stamp = clock_->now();
  lerp_ref_path.poses.resize(params_->prediction_horizon);

  reference_path_.resize(params_->prediction_horizon * params_->nx);
  reference_path_.setZero();
  std::vector<double> x_refs;
  std::vector<double> y_refs;
  int s_path_idx = 1;
  int s_predict_idx = 1;

  while(s_predict_idx < params_->prediction_horizon + 1){

    while(s_path_cum[s_path_idx] < s_predict_cum[s_predict_idx] && s_path_idx < waypoints_num_ - 1){
      s_path_idx++;
    }

    double ratio = (s_predict_cum[s_predict_idx] - s_path_cum[s_path_idx - 1]) / (s_path_cum[s_path_idx] - s_path_cum[s_path_idx - 1]);
    
    double x_k0 = global_path_.poses[ref_wp_idx + s_path_idx - 1].pose.position.x;
    double x_k1 = global_path_.poses[ref_wp_idx + s_path_idx].pose.position.x;
    double x_ref = lerp(x_k0, x_k1, ratio);
    x_refs.push_back(x_ref);
    
    double y_k0 = global_path_.poses[ref_wp_idx + s_path_idx - 1].pose.position.y;
    double y_k1 = global_path_.poses[ref_wp_idx + s_path_idx].pose.position.y;
    double y_ref = lerp(y_k0, y_k1, ratio);
    y_refs.push_back(y_ref);

    s_predict_idx++;
  }

  
  for(int i = 0; i < params_->prediction_horizon; i++){
    
    double dx_ref = 0.0;
    double dy_ref = 0.0;
    if(i == params_->prediction_horizon - 1){
      dx_ref = x_refs[i] - x_refs[i - 1];
      dy_ref = y_refs[i] - y_refs[i - 1];
    }else{
      dx_ref = x_refs[i + 1] - x_refs[i];
      dy_ref = y_refs[i + 1] - y_refs[i];
    }
    double theta_ref = std::atan2(dy_ref, dx_ref);
    
    reference_path_.segment(i * params_->nx, params_->nx) << x_refs[i], y_refs[i], theta_ref;
    
    lerp_ref_path.poses[i].pose.position.x = x_refs[i];
    lerp_ref_path.poses[i].pose.position.y = y_refs[i];
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_ref);
    lerp_ref_path.poses[i].pose.orientation = tf2::toMsg(q);
  }

  lerp_ref_path_publisher_->publish(lerp_ref_path);
  
  // TODO: express coordinates in a moving coordinate system instead of global
return reference_path_;
}


int PathManager::findReferenceWaypointIndex(
  const geometry_msgs::msg::PoseStamped& robot_pose)
{
  int ref_wp_idx = 0;
  double ref_wp_dist = std::numeric_limits<double>::max();
  for(int i = 0; i < waypoints_num_ - 2; i++){

    double x_rob = robot_pose.pose.position.x;
    double y_rob = robot_pose.pose.position.y;

    double x_gp_i0 = global_path_.poses[i].pose.position.x;
    double y_gp_i0 = global_path_.poses[i].pose.position.y;

    double x_gp_i1 = global_path_.poses[i + 1].pose.position.x;
    double y_gp_i1 = global_path_.poses[i + 1].pose.position.y;

    double hypot = std::hypot(x_rob - x_gp_i0, y_rob - y_gp_i0);
    std::vector<double> v_rob = {x_rob - x_gp_i0, y_rob - y_gp_i0}; // vector from closest waypoint to robot
    std::vector<double> v_wps = {x_gp_i1 - x_gp_i0, y_gp_i1 - y_gp_i0}; // vector from closest waypoint to next waypoint along path
    
    double ip_rob = std::inner_product(v_rob.begin(), v_rob.end(), v_wps.begin(), 0.0);
    if (hypot < ref_wp_dist && ip_rob >= 0){
      ref_wp_dist = hypot;
      ref_wp_idx = i;
    }
  }
  return ref_wp_idx;
}


geometry_msgs::msg::PointStamped 
PathManager::findProjectionPoint(
  const geometry_msgs::msg::PoseStamped& robot_pose,
  int ref_wp_idx)
{
  geometry_msgs::msg::PointStamped projection_point;
  projection_point.header.frame_id = global_path_.header.frame_id;
  projection_point.header.stamp = clock_->now();

  double x_rob = robot_pose.pose.position.x;
  double y_rob = robot_pose.pose.position.y;

  double x_gp_i0 = global_path_.poses[ref_wp_idx].pose.position.x;
  double y_gp_i0 = global_path_.poses[ref_wp_idx].pose.position.y;

  double x_gp_i1 = global_path_.poses[ref_wp_idx + 1].pose.position.x;
  double y_gp_i1 = global_path_.poses[ref_wp_idx + 1].pose.position.y;

  std::vector<double> v_rob = {x_rob - x_gp_i0, y_rob - y_gp_i0}; // vector from closest waypoint to robot
  std::vector<double> v_wps = {x_gp_i1 - x_gp_i0, y_gp_i1 - y_gp_i0}; // vector from closest waypoint to next waypoint along path
  
  double ip_rob = std::inner_product(v_rob.begin(), v_rob.end(), v_wps.begin(), 0.0);
  double ip_wp = std::inner_product(v_wps.begin(), v_wps.end(), v_wps.begin(), 0.0);
    
  double ratio = ip_rob / ip_wp;
  projection_point.point.x = lerp(x_gp_i0, x_gp_i1, ratio);
  projection_point.point.y = lerp(y_gp_i0, y_gp_i1, ratio);
  projection_point.point.z = 0;

  return projection_point;
}


void PathManager::publishOptimalTrajectory(
  const Eigen::VectorXd& predicted_state)
{
  nav_msgs::msg::Path optimal_trajectory;
  optimal_trajectory.header.frame_id = global_path_.header.frame_id;
  optimal_trajectory.header.stamp = clock_->now();
  optimal_trajectory.poses.resize(params_->prediction_horizon);
  
  tf2::Quaternion q;
  for(int i = 0; i < params_->prediction_horizon; i++){
    optimal_trajectory.poses[i].pose.position.x = predicted_state[i * params_->nx];
    optimal_trajectory.poses[i].pose.position.y = predicted_state[i * params_->nx + 1];
    q.setRPY(0, 0, predicted_state[i * params_->nx + 2]);
    optimal_trajectory.poses[i].pose.orientation = tf2::toMsg(q);
  }

  optimal_trajectory_publisher_->publish(optimal_trajectory);
}