#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"

#include "mpc_controller/parameter_manager.hpp"
#include <Eigen/Core>


#ifndef PATH_MANAGER_HPP_
#define PATH_MANAGER_HPP_


class PathManager{
public:
  PathManager();

  void configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
  Parameters* params);

  void setGlobalPath(const nav_msgs::msg::Path& global_path);
  
  Eigen::VectorXd computeReferencePath(const geometry_msgs::msg::PoseStamped& robot_pose);
  void publishOptimalTrajectory(const Eigen::VectorXd& predicted_state);

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  nav_msgs::msg::Path global_path_;
  Eigen::VectorXd reference_path_;
  rclcpp::Clock::SharedPtr clock_;
  Parameters* params_;
  
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> projection_point_publisher_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> reference_path_publisher_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> optimal_trajectory_publisher_;

  int waypoints_num_;

  int findReferenceWaypointIndex(const geometry_msgs::msg::PoseStamped& robot_pose);
  geometry_msgs::msg::PoseStamped findProjectionPoint(const geometry_msgs::msg::PoseStamped& robot_pose, int ref_wp_idx);
  
  void publishReferencePath(const Eigen::VectorXd& reference_path);
  
  template<typename TypeT>
  inline TypeT lerp(TypeT first_point, TypeT second_point, TypeT ratio){
    return first_point + ratio * (second_point - first_point);
  }
  
  nav_msgs::msg::Path convertEigenVectorToPathMsg(const Eigen::VectorXd& reference_path);

};
#endif  //PATH_MANAGER_HPP_