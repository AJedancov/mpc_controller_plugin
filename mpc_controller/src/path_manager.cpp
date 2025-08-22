#include "mpc_controller/path_manager.hpp"

PathManager::PathManager(){}

void PathManager::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
  Parameters* params)
{
  node_ = parent.lock();
  clock_ = node_->get_clock();
  params_ = params;

  projection_point_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "projection_point",
    10);
  reference_path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
    "reference_path", 
    10);
  optimal_trajectory_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
    "optimal_trajectory", 
    10);
}


void PathManager::setGlobalPath(const nav_msgs::msg::Path &global_path){
  global_path_ = global_path;
  waypoints_num_ = global_path_.poses.size();
}


Eigen::VectorXd PathManager::computeReferencePath(
  const geometry_msgs::msg::PoseStamped& robot_pose)
{
  
  int ref_wp_idx = findReferenceWaypointIndex(robot_pose);

  geometry_msgs::msg::PoseStamped projection_point;
  projection_point = findProjectionPoint(robot_pose, ref_wp_idx);
  projection_point_publisher_->publish(projection_point);

  double arc_len_predict = params_->max_lin_vel * params_->dt;
  double arc_len_predict_total = arc_len_predict * params_->prediction_horizon;

  nav_msgs::msg::Path pruned_path; // prunePath(ref_wp_idx, projection_point)
  pruned_path.poses.push_back(projection_point);
  
  std::vector<double> arc_len_path_cum;
  arc_len_path_cum.push_back(0.0);
  int wp_idx = ref_wp_idx;
  while(arc_len_path_cum.back() < arc_len_predict_total && wp_idx < waypoints_num_ - 1){
    double dx, dy;
    if(wp_idx == ref_wp_idx){
      dx = global_path_.poses[wp_idx + 1].pose.position.x - projection_point.pose.position.x;
      dy = global_path_.poses[wp_idx + 1].pose.position.y - projection_point.pose.position.y;
    }else{
      dx = global_path_.poses[wp_idx + 1].pose.position.x - global_path_.poses[wp_idx].pose.position.x;
      dy = global_path_.poses[wp_idx + 1].pose.position.y - global_path_.poses[wp_idx].pose.position.y;
    }
    arc_len_path_cum.push_back(arc_len_path_cum.back() + std::hypot(dx, dy));
    pruned_path.poses.push_back(global_path_.poses[wp_idx + 1]);
    wp_idx++;
  }

  double arc_len_path_total = arc_len_path_cum.back();
  
  bool reached_last_waypoint = false;
  if(arc_len_predict_total > arc_len_path_total){
    arc_len_predict = arc_len_path_total / params_->prediction_horizon;
    reached_last_waypoint = true;
  }

  // TODO: Replace on tsd::begin(), std::end and std::partial_sum()
  std::vector<double> arc_len_predict_cum;
  arc_len_predict_cum.push_back(0.0);
  for(int i = 0; i < params_->prediction_horizon; i++){
    arc_len_predict_cum.push_back(arc_len_predict_cum.back() + arc_len_predict);
  }


  std::vector<double> x_refs;
  std::vector<double> y_refs;
  int arc_len_path_idx = 1;
  int arc_len_predict_idx = 1;

  while(arc_len_predict_idx < params_->prediction_horizon + 1){

    while(arc_len_path_cum[arc_len_path_idx] < arc_len_predict_cum[arc_len_predict_idx]
      && arc_len_path_idx < (int)arc_len_path_cum.size() - 1)
    {
      arc_len_path_idx++;
    }

    double ratio = (arc_len_predict_cum[arc_len_predict_idx] - arc_len_path_cum[arc_len_path_idx - 1]) / 
      (arc_len_path_cum[arc_len_path_idx] - arc_len_path_cum[arc_len_path_idx - 1]);
    
    double x_k0 = pruned_path.poses[arc_len_path_idx - 1].pose.position.x;
    double x_k1 = pruned_path.poses[arc_len_path_idx].pose.position.x;
    double x_ref = lerp(x_k0, x_k1, ratio);
    x_refs.push_back(x_ref);
    
    double y_k0 = pruned_path.poses[arc_len_path_idx - 1].pose.position.y;
    double y_k1 = pruned_path.poses[arc_len_path_idx].pose.position.y;
    double y_ref = lerp(y_k0, y_k1, ratio);
    y_refs.push_back(y_ref);

    arc_len_predict_idx++;
  }


  std::vector<double> theta_refs;
  
  for(int i = 0; i < params_->prediction_horizon - 1; i++){
    double dx_ref = 0.0;
    double dy_ref = 0.0;
    dx_ref = x_refs[i + 1] - x_refs[i];
    dy_ref = y_refs[i + 1] - y_refs[i];
    theta_refs.push_back(std::atan2(dy_ref, dx_ref));
  }

  // Orientation of the last reference point
  if(reached_last_waypoint){
    theta_refs.push_back(tf2::getYaw(pruned_path.poses.back().pose.orientation));
  }else{
    theta_refs.push_back(theta_refs.back());
  }


  reference_path_.resize(params_->prediction_horizon * params_->nx);
  reference_path_.setZero();
  for(int i = 0; i < params_->prediction_horizon; i++){
    reference_path_.segment(i * params_->nx, params_->nx) << x_refs[i], y_refs[i], theta_refs[i];
  }

  publishReferencePath(reference_path_);
  
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

    std::vector<double> v_rob = {x_rob - x_gp_i0, y_rob - y_gp_i0}; // vector from i waypoint to robot
    std::vector<double> v_wps = {x_gp_i1 - x_gp_i0, y_gp_i1 - y_gp_i0}; // vector from i waypoint to i+1 waypoint along path
    double ip_rob = std::inner_product(v_rob.begin(), v_rob.end(), v_wps.begin(), 0.0);
    if (hypot < ref_wp_dist && ip_rob >= 0){
      ref_wp_dist = hypot;
      ref_wp_idx = i;
    }
  }
  return ref_wp_idx;
}


geometry_msgs::msg::PoseStamped 
PathManager::findProjectionPoint(
  const geometry_msgs::msg::PoseStamped& robot_pose,
  int ref_wp_idx)
{
  geometry_msgs::msg::PoseStamped projection_point;
  projection_point.header.frame_id = global_path_.header.frame_id;
  projection_point.header.stamp = clock_->now();

  double x_rob = robot_pose.pose.position.x;
  double y_rob = robot_pose.pose.position.y;

  double x_gp_i0 = global_path_.poses[ref_wp_idx].pose.position.x;
  double y_gp_i0 = global_path_.poses[ref_wp_idx].pose.position.y;

  double x_gp_i1 = global_path_.poses[ref_wp_idx + 1].pose.position.x;
  double y_gp_i1 = global_path_.poses[ref_wp_idx + 1].pose.position.y;

  std::vector<double> v_rob = {x_rob - x_gp_i0, y_rob - y_gp_i0}; // vector from i waypoint to robot
  std::vector<double> v_wps = {x_gp_i1 - x_gp_i0, y_gp_i1 - y_gp_i0}; // vector from i waypoint to i+1 waypoint along path
  
  double ip_rob = std::inner_product(v_rob.begin(), v_rob.end(), v_wps.begin(), 0.0);
  double ip_wp = std::inner_product(v_wps.begin(), v_wps.end(), v_wps.begin(), 0.0);
    
  double ratio = ip_rob / ip_wp;
  projection_point.pose.position.x = lerp(x_gp_i0, x_gp_i1, ratio);
  projection_point.pose.position.y = lerp(y_gp_i0, y_gp_i1, ratio);
  projection_point.pose.position.z = 0;

  return projection_point;
}


void PathManager::publishOptimalTrajectory(
  const Eigen::VectorXd& predicted_state)
{
  nav_msgs::msg::Path optimal_trajectory = convertEigenVectorToPathMsg(predicted_state);
  optimal_trajectory_publisher_->publish(optimal_trajectory);
}


void PathManager::publishReferencePath(
  const Eigen::VectorXd& reference_path)
{
  nav_msgs::msg::Path reference_path_msg = convertEigenVectorToPathMsg(reference_path);
  reference_path_publisher_->publish(reference_path_msg);
}


nav_msgs::msg::Path 
PathManager::convertEigenVectorToPathMsg(
  const Eigen::VectorXd& path)
{ 
  nav_msgs::msg::Path path_msg;
  path_msg.poses.resize(params_->prediction_horizon);
  path_msg.header.frame_id = global_path_.header.frame_id;
  path_msg.header.stamp = clock_->now();
  tf2::Quaternion q;

  for(int i = 0; i < params_->prediction_horizon; i++){
    path_msg.poses[i].pose.position.x = path[i * params_->nx];
    path_msg.poses[i].pose.position.y = path[i * params_->nx + 1];
    q.setRPY(0, 0, path[i * params_->nx + 2]);
    path_msg.poses[i].pose.orientation = tf2::toMsg(q);
  }
  return path_msg;
}